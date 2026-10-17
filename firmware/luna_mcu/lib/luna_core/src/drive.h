// Track speed loop (feed-forward + PI) and duty → PWM mapping. Plain C++, host-tested.
// Stops (enable=false, duty 0, integrators cleared) on E-stop, driver fault, no command for
// cmd_timeout_ms, a zero/invalid pwm_cap, or non-finite targets.
#pragma once
#include <cstdint>

namespace luna {

struct DriveConfig {
  float counts_per_rev = 6533.0f;      // Pololu 4755 at the output shaft
  float kp = 0.08f, ki = 0.6f, max_i = 0.5f;   // CALIBRATE on the real tracks
  float ff_rad_s_at_full_duty = 10.0f; // ~10 rad/s at full 12 V duty; CALIBRATE
  uint32_t cmd_timeout_ms = 300;
  float max_integ_dt_s = 0.05f;        // a late tick must not kick the integrator
};

struct DriveOutput {
  bool enable = false;                 // drive DRV_SLP high
  float duty[2] = {0, 0};              // -1..1, already limited by pwm_cap
};

class DriveController {
 public:
  explicit DriveController(const DriveConfig &cfg = DriveConfig()) : cfg_(cfg) {}
  // /luna/mcu/wheel_target: [left_rad_s, right_rad_s, pwm_cap]. NaN/inf anywhere → stop.
  void set_target(float left_rad_s, float right_rad_s, float pwm_cap, uint32_t now_ms);
  // counts: raw 32-bit encoder positions; dt_s: seconds since the previous step.
  DriveOutput step(const int32_t counts[2], float dt_s, uint32_t now_ms, bool estop, bool driver_fault);
  float position_rad(int i) const { return (float)pos_[i]; }
  float velocity_rad_s(int i) const { return vel_[i]; }

 private:
  DriveConfig cfg_;
  float target_[2] = {0, 0}, cap_ = 0, integ_[2] = {0, 0}, vel_[2] = {0, 0};
  double pos_[2] = {0, 0};
  int32_t last_counts_[2] = {0, 0};
  bool have_counts_ = false, have_cmd_ = false;
  uint32_t cmd_ms_ = 0;
};

struct PwmCommand {
  bool reverse = false;                // DIR pin HIGH
  uint16_t level = 0;                  // 0..full_scale
};
PwmCommand duty_to_pwm(float duty, uint16_t full_scale);   // non-finite → 0

// Clamp, but replace NaN/inf with `fallback` (e.g. keep the previous servo pulse).
float clamp_finite(float v, float lo, float hi, float fallback);

}  // namespace luna
