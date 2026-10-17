// P4 DOCK_SENSE identification window (Elektrik R01, 2026-10-17). Plain C++, host-tested.
// The dock pulls P4 up to 5 V through 10 kΩ and the robot holds 10 kΩ (R_ID) to GND, so P4 reads 2.50 V
// while the robot sits on a powered dock and 0 V when it is off the dock or the dock has no power. With
// DOCK_INHIBIT high the robot adds 2.2 kΩ: P4 = 0.76 V, which the dock reads as "switch the contacts off"
// and the robot still reads as "seated". Unlike the old charge-voltage sense this does not depend on the
// charger's CC/CV phase or on the pack voltage (in CC at low SoC the contacts sit near the pack voltage).
#pragma once
#include <cmath>
#include <cstdint>

namespace luna {

constexpr float P4_SEATED_V = 2.50f;    // 5.0 V × 10k / (10k + 10k)
constexpr float P4_INHIBIT_V = 0.76f;   // 5.0 V × (10k ∥ 2.2k) / (10k + 10k ∥ 2.2k)
constexpr float P4_SCORE_ON = 0.45f;    // |V − expected| ≤ 0.55 V → seated
constexpr float P4_SCORE_OFF = 0.25f;   // |V − expected| > 0.75 V → not seated

// Input for ThresholdDebounce(P4_SCORE_ON, P4_SCORE_OFF, hold_ms): 1 − |V − expected|. NaN stays NaN ("off").
inline float p4_score(float v, bool inhibit) {
  return 1.0f - std::fabs(v - (inhibit ? P4_INHIBIT_V : P4_SEATED_V));
}

// /luna/mcu/charge_enable from dock_node: false raises DOCK_INHIBIT, so the dock switches the contacts off
// before the robot reverses away. Charging is the default: a false only holds while it keeps arriving
// (dock_node sends it every 50 ms), so a stopped dock_node cannot leave the robot parked on the dock
// without charge; main.cpp also resets it while the micro-ROS agent is lost.
class ChargeEnable {
 public:
  explicit ChargeEnable(uint32_t lease_ms) : lease_ms_(lease_ms) {}
  void on_msg(bool enable, uint32_t now_ms) { inhibit_ = !enable; since_ms_ = now_ms; }
  void reset() { inhibit_ = false; }
  bool inhibit(uint32_t now_ms) {               // call every tick: an expired false is dropped here
    if (inhibit_ && (uint32_t)(now_ms - since_ms_) >= lease_ms_) inhibit_ = false;
    return inhibit_;
  }

 private:
  uint32_t lease_ms_;
  bool inhibit_ = false;
  uint32_t since_ms_ = 0;
};

}  // namespace luna
