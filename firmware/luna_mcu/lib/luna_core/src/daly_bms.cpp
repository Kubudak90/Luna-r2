#include "daly_bms.h"

#include <limits>

namespace luna {
namespace {

constexpr float NaN = std::numeric_limits<float>::quiet_NaN();

uint16_t be16(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }
int32_t be32(const uint8_t *p) {
  return (int32_t)(((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3]);
}

bool known_cmd(uint8_t c) {
  return c == DALY_SOC || c == DALY_TEMPS || c == DALY_MOSFET || c == DALY_STATUS || c == DALY_CELLS ||
         c == DALY_ALARMS;
}

}  // namespace

uint8_t daly_checksum(const uint8_t *bytes, size_t n) {
  uint8_t sum = 0;
  for (size_t i = 0; i < n; i++) sum = (uint8_t)(sum + bytes[i]);
  return sum;
}

size_t daly_build_request(uint8_t cmd, uint8_t out[DALY_FRAME]) {
  for (size_t i = 0; i < DALY_FRAME; i++) out[i] = 0;
  out[0] = DALY_START; out[1] = DALY_HOST; out[2] = cmd; out[3] = DALY_LEN;
  out[12] = daly_checksum(out, 12);
  return DALY_FRAME;
}

bool DalyParser::prefix_ok() const {
  return (n_ < 1 || buf_[0] == DALY_START) && (n_ < 2 || buf_[1] == DALY_BMS) &&
         (n_ < 3 || known_cmd(buf_[2])) && (n_ < 4 || buf_[3] == DALY_LEN);
}

void DalyParser::resync() {
  // The frame is bad: drop its first byte and try again from the next 0xA5 already read.
  while (n_ > 0) {
    size_t k = 1;
    while (k < n_ && buf_[k] != DALY_START) k++;
    for (size_t i = k; i < n_; i++) buf_[i - k] = buf_[i];
    n_ -= k;
    if (prefix_ok()) return;
  }
}

DalyFrame DalyParser::decode() {
  const uint8_t *d = buf_ + 4;
  switch (buf_[2]) {
    case DALY_SOC: {
      DalySoc s;
      s.voltage_v = be16(d) * 0.1f;
      s.current_a = -((int32_t)be16(d + 4) - (int32_t)DALY_CURRENT_OFFSET) * 0.1f;   // above the offset = discharging
      s.soc_pct = be16(d + 6) * 0.1f;
      soc_ = s;
      return DalyFrame::Soc;
    }
    case DALY_TEMPS:
      temps_.max_c = (float)((int)d[0] - 40);
      temps_.min_c = (float)((int)d[2] - 40);
      return DalyFrame::Temps;
    case DALY_MOSFET:
      mosfet_.mode = d[0];
      mosfet_.charge_fet = d[1] != 0;
      mosfet_.discharge_fet = d[2] != 0;
      mosfet_.remaining_ah = be32(d + 4) * 0.001f;
      return DalyFrame::Mosfet;
    case DALY_STATUS:
      status_.cells = d[0];
      status_.ntc = d[1];
      status_.charger = d[2] != 0;
      status_.load = d[3] != 0;
      status_.cycles = be16(d + 5);
      have_status_ = true;
      return DalyFrame::Status;
    case DALY_CELLS: {
      uint8_t expected = have_status_ && status_.cells ? status_.cells : default_cells_;
      if (expected > DALY_MAX_CELLS) expected = (uint8_t)DALY_MAX_CELLS;
      const uint8_t no = d[0];
      if (no == 1) { partial_ = DalyCells(); expect_frame_ = 1; }
      if (no != expect_frame_ || no == 0) {           // a frame of this reply is missing
        expect_frame_ = 1;
        ++bad_;
        return DalyFrame::None;
      }
      for (int k = 0; k < 3; k++) {
        const int idx = (no - 1) * 3 + k;
        if (idx < expected) partial_.v[idx] = be16(d + 1 + 2 * k) * 0.001f;
      }
      partial_.count = (uint8_t)(no * 3 < expected ? no * 3 : expected);
      ++expect_frame_;
      if (partial_.count < expected) return DalyFrame::None;
      cells_ = partial_;
      expect_frame_ = 1;
      return DalyFrame::Cells;
    }
    case DALY_ALARMS:
      for (int i = 0; i < 8; i++) alarms_.bits[i] = d[i];
      return DalyFrame::Alarms;
  }
  return DalyFrame::None;
}

DalyFrame DalyParser::feed(uint8_t b) {
  if (n_ == 0 && b != DALY_START) return DalyFrame::None;
  buf_[n_++] = b;
  if (!prefix_ok()) { ++bad_; resync(); return DalyFrame::None; }
  if (n_ < DALY_FRAME) return DalyFrame::None;
  if (daly_checksum(buf_, 12) != buf_[12]) { ++bad_; resync(); return DalyFrame::None; }
  const uint32_t bad_before = bad_;
  const DalyFrame f = decode();
  n_ = 0;
  if (bad_ == bad_before) ++ok_;
  return f;
}

uint8_t daly_health(const DalyAlarms &a) {
  const uint8_t *b = a.bits;
  if ((b[1] & (DALY_A1_CHG_HOT_L2 | DALY_A1_DSG_HOT_L2)) || (b[4] & (DALY_A4_CHG_FET_HOT | DALY_A4_DSG_FET_HOT)))
    return BATT_HEALTH_OVERHEAT;
  if (b[1] & (DALY_A1_CHG_COLD_L2 | DALY_A1_DSG_COLD_L2)) return BATT_HEALTH_COLD;
  if (b[0] & (DALY_A0_CELL_HIGH_L2 | DALY_A0_SUM_HIGH_L2)) return BATT_HEALTH_OVERVOLTAGE;
  if (b[0] & (DALY_A0_CELL_LOW_L2 | DALY_A0_SUM_LOW_L2)) return BATT_HEALTH_DEAD;
  if ((b[2] & (DALY_A2_CHG_OC_L2 | DALY_A2_DSG_OC_L2)) || (b[3] & (DALY_A3_DIFF_V_L2 | DALY_A3_DIFF_T_L2)) ||
      (b[4] & 0xFC) || b[5] || b[6])                  // byte 7 is a fault code, not interpreted
    return BATT_HEALTH_UNSPEC_FAILURE;
  return BATT_HEALTH_GOOD;
}

BatteryFields battery_fields_absent() {
  BatteryFields f;
  f.present = false;
  f.voltage = 0;
  f.current = 0;
  f.charge = f.capacity = f.design_capacity = f.percentage = f.temperature = NaN;
  f.status = BATT_STATUS_UNKNOWN;
  f.health = BATT_HEALTH_UNKNOWN;
  f.technology = BATT_TECH_LIFE;
  return f;
}

BatteryFields BatteryMonitor::fields(uint32_t now_ms) const {
  if (!present(now_ms)) return battery_fields_absent();
  BatteryFields f;
  f.present = true;
  f.voltage = soc_.voltage_v;
  f.current = soc_.current_a;
  f.percentage = soc_.soc_pct >= 0 && soc_.soc_pct <= 100 ? soc_.soc_pct * 0.01f : NaN;
  f.charge = fresh(have_mosfet_, mosfet_ms_, now_ms) ? mosfet_.remaining_ah : NaN;
  f.capacity = f.design_capacity = design_ah_;     // Daly reports no learned full capacity on these commands
  f.temperature = fresh(have_temps_, temps_ms_, now_ms) ? temps_.max_c : NaN;   // hottest sensor: overtemp policy

  if (f.current > 0.05f) f.status = BATT_STATUS_CHARGING;
  else if (f.current < -0.05f) f.status = BATT_STATUS_DISCHARGING;
  else if (f.percentage >= 0.99f) f.status = BATT_STATUS_FULL;
  else f.status = BATT_STATUS_NOT_CHARGING;

  f.health = fresh(have_alarms_, alarms_ms_, now_ms) ? daly_health(alarms_) : (uint8_t)BATT_HEALTH_UNKNOWN;
  f.technology = BATT_TECH_LIFE;
  if (fresh(have_cells_, cells_ms_, now_ms)) {
    f.cell_count = cells_.count;
    for (uint8_t i = 0; i < cells_.count; i++) f.cell_v[i] = cells_.v[i];
  }
  return f;
}

}  // namespace luna
