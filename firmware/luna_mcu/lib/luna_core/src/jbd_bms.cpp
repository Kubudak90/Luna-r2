#include "jbd_bms.h"

#include <limits>

namespace luna {
namespace {

constexpr float NaN = std::numeric_limits<float>::quiet_NaN();

uint16_t be16(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }

}  // namespace

uint16_t jbd_checksum(const uint8_t *bytes, size_t n) {
  uint16_t sum = 0;
  for (size_t i = 0; i < n; i++) sum += bytes[i];
  return (uint16_t)(0x10000u - sum);
}

size_t jbd_build_read_request(uint8_t reg, uint8_t out[7]) {
  const uint8_t body[2] = {reg, 0x00};
  const uint16_t chk = jbd_checksum(body, 2);
  out[0] = JBD_START; out[1] = JBD_READ; out[2] = reg; out[3] = 0x00;
  out[4] = (uint8_t)(chk >> 8); out[5] = (uint8_t)chk; out[6] = JBD_END;
  return 7;
}

bool jbd_decode_basic(const uint8_t *d, size_t len, JbdBasicInfo &out) {
  if (len < JBD_BASIC_MIN_LEN) return false;
  const uint8_t ntc = d[22];
  if (len < JBD_BASIC_MIN_LEN + 2u * ntc) return false;
  JbdBasicInfo r;
  r.voltage_v = be16(d + 0) * 0.01f;                  // 10 mV
  r.current_a = (int16_t)be16(d + 2) * 0.01f;         // 10 mA, signed
  r.remaining_ah = be16(d + 4) * 0.01f;               // 10 mAh
  r.nominal_ah = be16(d + 6) * 0.01f;
  r.cycles = be16(d + 8);
  r.protection = be16(d + 16);
  r.rsoc_pct = d[19];
  r.fet = d[20];
  r.cells = d[21];
  r.ntc_count = ntc < JBD_MAX_NTC ? ntc : (uint8_t)JBD_MAX_NTC;
  for (uint8_t i = 0; i < r.ntc_count; i++)
    r.ntc_c[i] = ((int)be16(d + 23 + 2 * i) - 2731) * 0.1f;   // 0.1 K
  out = r;
  return true;
}

bool jbd_decode_cells(const uint8_t *d, size_t len, JbdCellInfo &out) {
  if (len == 0 || len % 2 || len / 2 > JBD_MAX_CELLS) return false;
  JbdCellInfo r;
  r.count = (uint8_t)(len / 2);
  for (uint8_t i = 0; i < r.count; i++) r.v[i] = be16(d + 2 * i) * 0.001f;   // mV
  out = r;
  return true;
}

void JbdParser::fail(uint8_t b) {
  ++bad_;
  state_ = b == JBD_START ? REG : WAIT_START;   // the offending byte may start the next frame
}

JbdFrame JbdParser::feed(uint8_t b) {
  switch (state_) {
    case WAIT_START:
      if (b == JBD_START) state_ = REG;
      return JbdFrame::None;
    case REG:
      if (b != JBD_REG_BASIC && b != JBD_REG_CELLS) { fail(b); return JbdFrame::None; }
      reg_ = b; state_ = STATUS;
      return JbdFrame::None;
    case STATUS:
      status_ = b; sum_ = b; state_ = LEN;
      return JbdFrame::None;
    case LEN:
      if (b > JBD_MAX_DATA) { fail(b); return JbdFrame::None; }
      len_ = b; sum_ += b; n_ = 0;
      state_ = len_ ? DATA : CHK_HI;
      return JbdFrame::None;
    case DATA:
      data_[n_++] = b; sum_ += b;
      if (n_ == len_) state_ = CHK_HI;
      return JbdFrame::None;
    case CHK_HI:
      rx_chk_ = (uint16_t)(b << 8); state_ = CHK_LO;
      return JbdFrame::None;
    case CHK_LO:
      rx_chk_ |= b; state_ = END;
      return JbdFrame::None;
    case END: {
      if (b != JBD_END) { fail(b); return JbdFrame::None; }
      state_ = WAIT_START;
      if (rx_chk_ != (uint16_t)(0x10000u - sum_) || status_ != 0x00) { ++bad_; return JbdFrame::None; }
      if (reg_ == JBD_REG_BASIC) {
        JbdBasicInfo decoded;
        if (!jbd_decode_basic(data_, len_, decoded)) { ++bad_; return JbdFrame::None; }
        info_ = decoded; ++ok_;
        return JbdFrame::Basic;
      }
      JbdCellInfo decoded;
      if (!jbd_decode_cells(data_, len_, decoded)) { ++bad_; return JbdFrame::None; }
      cells_ = decoded; ++ok_;
      return JbdFrame::Cells;
    }
  }
  return JbdFrame::None;
}

BatteryFields battery_fields_from(const JbdBasicInfo &info) {
  BatteryFields f;
  f.present = true;
  f.voltage = info.voltage_v;
  f.current = info.current_a;
  f.charge = info.remaining_ah;
  f.capacity = info.nominal_ah;           // basic info has no learned full capacity: use the nominal one
  f.design_capacity = info.nominal_ah;
  if (info.nominal_ah > 0) {
    const float s = info.remaining_ah / info.nominal_ah;
    f.percentage = s < 0 ? 0 : s > 1 ? 1 : s;
  } else {
    f.percentage = info.rsoc_pct <= 100 ? info.rsoc_pct * 0.01f : NaN;
  }
  f.temperature = NaN;
  for (uint8_t i = 0; i < info.ntc_count; i++)            // hottest sensor drives the overtemp policy
    if (!(f.temperature >= info.ntc_c[i])) f.temperature = info.ntc_c[i];

  if (info.current_a > 0.05f) f.status = BATT_STATUS_CHARGING;
  else if (info.current_a < -0.05f) f.status = BATT_STATUS_DISCHARGING;
  else if (f.percentage >= 0.99f) f.status = BATT_STATUS_FULL;
  else f.status = BATT_STATUS_NOT_CHARGING;

  const uint16_t p = info.protection;
  if (p == 0) f.health = BATT_HEALTH_GOOD;
  else if (p & (JBD_P_CHG_OT | JBD_P_DSG_OT)) f.health = BATT_HEALTH_OVERHEAT;
  else if (p & (JBD_P_CHG_UT | JBD_P_DSG_UT)) f.health = BATT_HEALTH_COLD;
  else if (p & (JBD_P_CELL_OV | JBD_P_PACK_OV)) f.health = BATT_HEALTH_OVERVOLTAGE;
  else if (p & (JBD_P_CELL_UV | JBD_P_PACK_UV)) f.health = BATT_HEALTH_DEAD;
  else f.health = BATT_HEALTH_UNSPEC_FAILURE;
  f.technology = BATT_TECH_LION;
  return f;
}

BatteryFields battery_fields_absent() {
  BatteryFields f;
  f.present = false;
  f.voltage = 0;
  f.current = 0;
  f.charge = f.capacity = f.design_capacity = f.percentage = f.temperature = NaN;
  f.status = BATT_STATUS_UNKNOWN;
  f.health = BATT_HEALTH_UNKNOWN;
  f.technology = BATT_TECH_LION;
  return f;
}

void BatteryMonitor::on_frame(const JbdBasicInfo &info, uint32_t now_ms) {
  last_ = battery_fields_from(info);
  have_ = true;
  t_ms_ = now_ms;
}

void BatteryMonitor::on_cells(const JbdCellInfo &cells, uint32_t now_ms) {
  cells_ = cells;
  have_cells_ = true;
  cells_ms_ = now_ms;
}

bool BatteryMonitor::present(uint32_t now_ms) const {
  return have_ && (uint32_t)(now_ms - t_ms_) <= stale_ms_;
}

BatteryFields BatteryMonitor::fields(uint32_t now_ms) const {
  if (!present(now_ms)) return battery_fields_absent();
  BatteryFields f = last_;
  if (have_cells_ && (uint32_t)(now_ms - cells_ms_) <= stale_ms_) {
    f.cell_count = cells_.count;
    for (uint8_t i = 0; i < cells_.count; i++) f.cell_v[i] = cells_.v[i];
  }
  return f;
}

}  // namespace luna
