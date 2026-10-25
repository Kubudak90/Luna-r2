// Daly smart BMS UART protocol (4S LiFePO4 boards, 9600 8N1): the read commands Luna polls and the mapping to
// sensor_msgs/BatteryState fields. Plain C++, no Arduino: unit-tested on the host.
//
// Every frame is 13 bytes:  0xA5 | addr | cmd | 0x08 | data[8] | chk      chk = sum of the 12 bytes before it
// Request: addr 0x40 (host on the UART port), data all zero. Reply: addr 0x01 (the BMS), same cmd.
// Payloads, big-endian (as decoded by python-daly-bms and dbus-serialbattery):
//   0x90  pack voltage 0.1 V | unused | current 0.1 A with a 30000 offset, above it = discharging | SoC 0.1 %
//   0x92  highest temperature + 40 °C | its sensor | lowest temperature + 40 °C | its sensor
//   0x93  mode (0 idle, 1 charging, 2 discharging) | charge FET | discharge FET | cycles | remaining mAh (u32)
//   0x94  cell count | temperature sensor count | charger | load | DIO bits | cycles (u16)
//   0x95  cell voltages: one reply frame per 3 cells, data[0] = frame number from 1, then 3 × mV
//   0x98  alarm and fault bits, 8 bytes (DalyAlarm)
// The current sign is the one point the two references and some Daly firmware disagree on: CHECK it on the bench
// with the charger connected (BatteryState.current must be positive while charging).
#pragma once
#include <cstddef>
#include <cstdint>

namespace luna {

constexpr uint8_t DALY_START = 0xA5, DALY_HOST = 0x40, DALY_BMS = 0x01, DALY_LEN = 8;
constexpr size_t DALY_FRAME = 13;
constexpr uint8_t DALY_SOC = 0x90, DALY_TEMPS = 0x92, DALY_MOSFET = 0x93, DALY_STATUS = 0x94, DALY_CELLS = 0x95,
                  DALY_ALARMS = 0x98;
constexpr size_t DALY_MAX_CELLS = 16;
constexpr uint16_t DALY_CURRENT_OFFSET = 30000;

uint8_t daly_checksum(const uint8_t *bytes, size_t n);
// Writes the 13-byte read request for `cmd` into out; returns 13.
size_t daly_build_request(uint8_t cmd, uint8_t out[DALY_FRAME]);

struct DalySoc {           // 0x90
  float voltage_v = 0;
  float current_a = 0;     // + charging, - discharging (BatteryState convention)
  float soc_pct = 0;
};

struct DalyTemps {         // 0x92
  float max_c = 0, min_c = 0;
};

struct DalyMosfet {        // 0x93
  uint8_t mode = 0;
  bool charge_fet = false, discharge_fet = false;
  float remaining_ah = 0;
};

struct DalyStatus {        // 0x94
  uint8_t cells = 0, ntc = 0;
  bool charger = false, load = false;
  uint16_t cycles = 0;
};

struct DalyCells {         // all 0x95 frames of one reply
  uint8_t count = 0;
  float v[DALY_MAX_CELLS] = {};   // volts, cell 1 first
};

// 0x98 bits as alarm[byte] & (1 << bit). L1 is Daly's warning level, L2 its protection level (FETs off).
enum DalyAlarm : uint8_t {
  // byte 0: voltages
  DALY_A0_CELL_HIGH_L1 = 1u << 0, DALY_A0_CELL_HIGH_L2 = 1u << 1, DALY_A0_CELL_LOW_L1 = 1u << 2,
  DALY_A0_CELL_LOW_L2 = 1u << 3, DALY_A0_SUM_HIGH_L1 = 1u << 4, DALY_A0_SUM_HIGH_L2 = 1u << 5,
  DALY_A0_SUM_LOW_L1 = 1u << 6, DALY_A0_SUM_LOW_L2 = 1u << 7,
  // byte 1: temperatures
  DALY_A1_CHG_HOT_L1 = 1u << 0, DALY_A1_CHG_HOT_L2 = 1u << 1, DALY_A1_CHG_COLD_L1 = 1u << 2,
  DALY_A1_CHG_COLD_L2 = 1u << 3, DALY_A1_DSG_HOT_L1 = 1u << 4, DALY_A1_DSG_HOT_L2 = 1u << 5,
  DALY_A1_DSG_COLD_L1 = 1u << 6, DALY_A1_DSG_COLD_L2 = 1u << 7,
  // byte 2: currents and SoC
  DALY_A2_CHG_OC_L1 = 1u << 0, DALY_A2_CHG_OC_L2 = 1u << 1, DALY_A2_DSG_OC_L1 = 1u << 2, DALY_A2_DSG_OC_L2 = 1u << 3,
  DALY_A2_SOC_HIGH_L1 = 1u << 4, DALY_A2_SOC_HIGH_L2 = 1u << 5, DALY_A2_SOC_LOW_L1 = 1u << 6,
  DALY_A2_SOC_LOW_L2 = 1u << 7,
  // byte 3: imbalance
  DALY_A3_DIFF_V_L1 = 1u << 0, DALY_A3_DIFF_V_L2 = 1u << 1, DALY_A3_DIFF_T_L1 = 1u << 2, DALY_A3_DIFF_T_L2 = 1u << 3,
  // byte 4: FETs
  DALY_A4_CHG_FET_HOT = 1u << 0, DALY_A4_DSG_FET_HOT = 1u << 1,
  // bytes 4 (bits 2–7), 5 and 6: hardware faults (sensors, FETs stuck or open, AFE, EEPROM, short circuit ...)
};

struct DalyAlarms {        // 0x98
  uint8_t bits[8] = {};
};

enum class DalyFrame : uint8_t { None, Soc, Temps, Mosfet, Status, Cells, Alarms };

// Byte-at-a-time parser for BMS replies. Checks start byte, BMS address, length and checksum; on a bad frame it
// resynchronises on the next 0xA5 inside the bytes already read. 0x95 replies complete when all expected cells are
// in (the cell count of the last 0x94 frame, else `default_cells`).
class DalyParser {
 public:
  explicit DalyParser(uint8_t default_cells = 4) : default_cells_(default_cells) {}
  DalyFrame feed(uint8_t b);
  void reset() { n_ = 0; }    // before each request: a half frame must not swallow the reply
  const DalySoc &soc() const { return soc_; }
  const DalyTemps &temps() const { return temps_; }
  const DalyMosfet &mosfet() const { return mosfet_; }
  const DalyStatus &status() const { return status_; }
  const DalyCells &cells() const { return cells_; }
  const DalyAlarms &alarms() const { return alarms_; }
  uint32_t frames_ok() const { return ok_; }
  uint32_t frames_bad() const { return bad_; }

 private:
  bool prefix_ok() const;
  void resync();
  DalyFrame decode();
  uint8_t default_cells_;
  uint8_t buf_[DALY_FRAME] = {};
  size_t n_ = 0;
  uint8_t expect_frame_ = 1;           // next 0x95 frame number
  DalyCells partial_;
  DalySoc soc_;
  DalyTemps temps_;
  DalyMosfet mosfet_;
  DalyStatus status_;
  DalyCells cells_;
  DalyAlarms alarms_;
  bool have_status_ = false;
  uint32_t ok_ = 0, bad_ = 0;
};

// Same numeric values as sensor_msgs/BatteryState (main.cpp static_asserts them).
enum : uint8_t {
  BATT_STATUS_UNKNOWN = 0, BATT_STATUS_CHARGING = 1, BATT_STATUS_DISCHARGING = 2,
  BATT_STATUS_NOT_CHARGING = 3, BATT_STATUS_FULL = 4,
};
enum : uint8_t {
  BATT_HEALTH_UNKNOWN = 0, BATT_HEALTH_GOOD = 1, BATT_HEALTH_OVERHEAT = 2, BATT_HEALTH_DEAD = 3,
  BATT_HEALTH_OVERVOLTAGE = 4, BATT_HEALTH_UNSPEC_FAILURE = 5, BATT_HEALTH_COLD = 6,
};
enum : uint8_t { BATT_TECH_UNKNOWN = 0, BATT_TECH_LIFE = 4 };

// Health from 0x98: Daly's L2 (protection) levels and the hardware faults. L1 warnings are left out: the voltage,
// temperature and SoC they warn about are published as numbers and the AGX has its own thresholds.
uint8_t daly_health(const DalyAlarms &a);

// The BatteryState fields the firmware fills.
struct BatteryFields {
  float voltage = 0, current = 0, charge = 0, capacity = 0, design_capacity = 0;
  float percentage = 0, temperature = 0;          // percentage 0..1, temperature °C (NaN = unknown)
  uint8_t status = BATT_STATUS_UNKNOWN, health = BATT_HEALTH_UNKNOWN, technology = BATT_TECH_LIFE;
  bool present = false;
  uint8_t cell_count = 0;                         // 0 = no fresh 0x95 reply
  float cell_v[DALY_MAX_CELLS] = {};
};

// The fail-safe "no BMS" message: present=false and voltage 0.0. Voltage must stay 0 and never
// NaN: the AGX computes pwm_cap = 12 V / voltage, which is 0 for 0 V but NaN for NaN.
BatteryFields battery_fields_absent();

// Holds the last valid replies. "Present" follows the 0x90 frames alone (voltage, current, SoC): without them it
// drops back to absent. Every other field is added only while its own reply is fresh.
class BatteryMonitor {
 public:
  BatteryMonitor(uint32_t stale_ms, float design_ah) : stale_ms_(stale_ms), design_ah_(design_ah) {}
  void on_soc(const DalySoc &s, uint32_t now_ms) { soc_ = s; soc_ms_ = now_ms; have_soc_ = true; }
  void on_temps(const DalyTemps &t, uint32_t now_ms) { temps_ = t; temps_ms_ = now_ms; have_temps_ = true; }
  void on_mosfet(const DalyMosfet &m, uint32_t now_ms) { mosfet_ = m; mosfet_ms_ = now_ms; have_mosfet_ = true; }
  void on_cells(const DalyCells &c, uint32_t now_ms) { cells_ = c; cells_ms_ = now_ms; have_cells_ = true; }
  void on_alarms(const DalyAlarms &a, uint32_t now_ms) { alarms_ = a; alarms_ms_ = now_ms; have_alarms_ = true; }
  bool present(uint32_t now_ms) const { return fresh(have_soc_, soc_ms_, now_ms); }
  BatteryFields fields(uint32_t now_ms) const;

 private:
  bool fresh(bool have, uint32_t t_ms, uint32_t now_ms) const { return have && (uint32_t)(now_ms - t_ms) <= stale_ms_; }
  uint32_t stale_ms_;
  float design_ah_;
  DalySoc soc_;
  DalyTemps temps_;
  DalyMosfet mosfet_;
  DalyCells cells_;
  DalyAlarms alarms_;
  bool have_soc_ = false, have_temps_ = false, have_mosfet_ = false, have_cells_ = false, have_alarms_ = false;
  uint32_t soc_ms_ = 0, temps_ms_ = 0, mosfet_ms_ = 0, cells_ms_ = 0, alarms_ms_ = 0;
};

}  // namespace luna
