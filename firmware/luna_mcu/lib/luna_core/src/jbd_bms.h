// JBD (Xiaoxiang) BMS UART protocol: basic info (register 0x03) and cell voltages (0x04), and the
// mapping to sensor_msgs/BatteryState fields. Plain C++, no Arduino: unit-tested on the host.
//
// Frame:   0xDD | reg | status | len | data[len] | chk_hi | chk_lo | 0x77
// Request: 0xDD | 0xA5 | reg | 0x00 | chk_hi | chk_lo | 0x77
// Checksum = 0x10000 - sum(status, len, data)  (for a request: sum(reg, len)).
#pragma once
#include <cstddef>
#include <cstdint>

namespace luna {

constexpr uint8_t JBD_START = 0xDD, JBD_END = 0x77, JBD_READ = 0xA5;
constexpr uint8_t JBD_REG_BASIC = 0x03, JBD_REG_CELLS = 0x04;
constexpr size_t JBD_MAX_DATA = 64;         // basic info is 23 + 2*NTC (+ extras on newer boards)
constexpr size_t JBD_MAX_CELLS = JBD_MAX_DATA / 2;
constexpr size_t JBD_BASIC_MIN_LEN = 23;
constexpr size_t JBD_MAX_NTC = 8;

uint16_t jbd_checksum(const uint8_t *bytes, size_t n);
// Writes the 7-byte read request for `reg` into out[7]; returns 7.
size_t jbd_build_read_request(uint8_t reg, uint8_t out[7]);

// JBD protection status bits (basic info bytes 16..17).
enum JbdProtection : uint16_t {
  JBD_P_CELL_OV = 1u << 0, JBD_P_CELL_UV = 1u << 1, JBD_P_PACK_OV = 1u << 2, JBD_P_PACK_UV = 1u << 3,
  JBD_P_CHG_OT = 1u << 4, JBD_P_CHG_UT = 1u << 5, JBD_P_DSG_OT = 1u << 6, JBD_P_DSG_UT = 1u << 7,
  JBD_P_CHG_OC = 1u << 8, JBD_P_DSG_OC = 1u << 9, JBD_P_SHORT = 1u << 10, JBD_P_AFE_ERROR = 1u << 11,
  JBD_P_SOFT_LOCK = 1u << 12,
};

struct JbdBasicInfo {
  float voltage_v = 0;        // pack voltage
  float current_a = 0;        // + charging, - discharging (same sign as BatteryState)
  float remaining_ah = 0;
  float nominal_ah = 0;
  uint16_t cycles = 0;
  uint16_t protection = 0;    // JbdProtection bits
  uint8_t rsoc_pct = 0;       // BMS' own SoC estimate
  uint8_t fet = 0;            // bit0 charge FET on, bit1 discharge FET on
  uint8_t cells = 0;
  uint8_t ntc_count = 0;      // number of temperatures stored in ntc_c (capped at JBD_MAX_NTC)
  float ntc_c[JBD_MAX_NTC] = {};
};

struct JbdCellInfo {
  uint8_t count = 0;
  float v[JBD_MAX_CELLS] = {};  // volts, cell 1 first
};

// Decodes the data part of a basic-info response. False if too short for its own NTC count.
bool jbd_decode_basic(const uint8_t *data, size_t len, JbdBasicInfo &out);
// Decodes a cell-voltage response (2 bytes per cell, mV). False if empty or odd length.
bool jbd_decode_cells(const uint8_t *data, size_t len, JbdCellInfo &out);

enum class JbdFrame : uint8_t { None, Basic, Cells };

// Byte-at-a-time frame parser. Accepts 0x03/0x04 responses with status 0x00 and a valid
// checksum. Call reset() before each request so a truncated frame cannot swallow the next one.
class JbdParser {
 public:
  // Returns which frame this byte completed (then see info() or cells()), else None.
  JbdFrame feed(uint8_t b);
  void reset() { state_ = WAIT_START; }
  const JbdBasicInfo &info() const { return info_; }
  const JbdCellInfo &cells() const { return cells_; }
  uint32_t frames_ok() const { return ok_; }
  uint32_t frames_bad() const { return bad_; }   // checksum, end byte, status, length or decode errors

 private:
  enum State { WAIT_START, REG, STATUS, LEN, DATA, CHK_HI, CHK_LO, END };
  void fail(uint8_t b);
  State state_ = WAIT_START;
  uint8_t reg_ = 0, status_ = 0, len_ = 0;
  size_t n_ = 0;
  uint16_t sum_ = 0, rx_chk_ = 0;
  uint8_t data_[JBD_MAX_DATA] = {};
  JbdBasicInfo info_;
  JbdCellInfo cells_;
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
enum : uint8_t { BATT_TECH_UNKNOWN = 0, BATT_TECH_LION = 2 };

// The BatteryState fields the firmware fills.
struct BatteryFields {
  float voltage = 0, current = 0, charge = 0, capacity = 0, design_capacity = 0;
  float percentage = 0, temperature = 0;          // percentage 0..1, temperature °C (NaN = unknown)
  uint8_t status = BATT_STATUS_UNKNOWN, health = BATT_HEALTH_UNKNOWN, technology = BATT_TECH_LION;
  bool present = false;
  uint8_t cell_count = 0;                         // 0 = no fresh 0x04 frame
  float cell_v[JBD_MAX_CELLS] = {};
};

// Pure mapping of one decoded frame to BatteryState fields (present = true).
BatteryFields battery_fields_from(const JbdBasicInfo &info);

// The fail-safe "no BMS" message: present=false and voltage 0.0. Voltage must stay 0 and never
// NaN: the AGX computes pwm_cap = 12 V / voltage, which is 0 for 0 V but NaN for NaN.
BatteryFields battery_fields_absent();

// Holds the last valid frames and drops back to "absent" when basic frames stop arriving.
// Cell voltages are added only while their own frame is fresh; they never make it "present".
class BatteryMonitor {
 public:
  explicit BatteryMonitor(uint32_t stale_ms) : stale_ms_(stale_ms) {}
  void on_frame(const JbdBasicInfo &info, uint32_t now_ms);
  void on_cells(const JbdCellInfo &cells, uint32_t now_ms);
  BatteryFields fields(uint32_t now_ms) const;
  bool present(uint32_t now_ms) const;

 private:
  uint32_t stale_ms_;
  bool have_ = false, have_cells_ = false;
  uint32_t t_ms_ = 0, cells_ms_ = 0;
  BatteryFields last_;
  JbdCellInfo cells_;
};

}  // namespace luna
