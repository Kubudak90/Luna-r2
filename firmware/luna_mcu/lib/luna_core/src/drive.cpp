#include "drive.h"

#include <cmath>

namespace luna {
namespace {

constexpr double TWO_PI_D = 6.283185307179586;

float clampf(float v, float lo, float hi) { return v < lo ? lo : v > hi ? hi : v; }

}  // namespace

float clamp_finite(float v, float lo, float hi, float fallback) {
  return std::isfinite(v) ? clampf(v, lo, hi) : fallback;
}

void DriveController::set_target(float left, float right, float pwm_cap, uint32_t now_ms) {
  have_cmd_ = true;
  cmd_ms_ = now_ms;
  if (!std::isfinite(left) || !std::isfinite(right) || !std::isfinite(pwm_cap)) {
    target_[0] = target_[1] = 0; cap_ = 0;
    return;
  }
  target_[0] = left; target_[1] = right;
  cap_ = clampf(pwm_cap, 0.0f, 1.0f);
}

DriveOutput DriveController::step(const int32_t counts[2], float dt_s, uint32_t now_ms, bool estop, bool fault) {
  const double rad_per_count = TWO_PI_D / cfg_.counts_per_rev;
  for (int i = 0; i < 2; i++) {
    // Velocity from the wrapped 32-bit count delta, not from float positions (precision).
    const int32_t d = have_counts_ ? (int32_t)((uint32_t)counts[i] - (uint32_t)last_counts_[i]) : 0;
    vel_[i] = dt_s > 0 ? (float)(d * rad_per_count / dt_s) : 0.0f;
    pos_[i] = counts[i] * rad_per_count;
    last_counts_[i] = counts[i];
  }
  have_counts_ = true;

  DriveOutput out;
  const bool timed_out = !have_cmd_ || (uint32_t)(now_ms - cmd_ms_) > cfg_.cmd_timeout_ms;
  if (estop || fault || timed_out || !(cap_ > 0)) {
    integ_[0] = integ_[1] = 0;
    return out;
  }
  const float dti = dt_s > 0 ? (dt_s < cfg_.max_integ_dt_s ? dt_s : cfg_.max_integ_dt_s) : 0.0f;
  out.enable = true;
  for (int i = 0; i < 2; i++) {
    const float err = target_[i] - vel_[i];
    integ_[i] = clampf(integ_[i] + err * cfg_.ki * dti, -cfg_.max_i, cfg_.max_i);
    const float ff = target_[i] / cfg_.ff_rad_s_at_full_duty;
    out.duty[i] = clampf(ff + cfg_.kp * err + integ_[i], -cap_, cap_);
  }
  return out;
}

PwmCommand duty_to_pwm(float duty, uint16_t full_scale) {
  PwmCommand p;
  if (!std::isfinite(duty)) return p;
  p.reverse = duty < 0;
  const float mag = clampf(std::fabs(duty), 0.0f, 1.0f);
  p.level = (uint16_t)(mag * full_scale + 0.5f);
  return p;
}

}  // namespace luna
