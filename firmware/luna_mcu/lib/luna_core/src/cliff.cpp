#include "cliff.h"

namespace luna {

void CliffChannel::on_sample(uint8_t range_status, uint16_t distance_mm, uint32_t now_ms) {
  if (!alive_) return;
  have_ = true;
  t_ms_ = now_ms;
  m_ = range_status == TOF_STATUS_VALID ? distance_mm * 0.001f : 0.0f;   // invalid → 0 at once
}

float CliffChannel::metres(uint32_t now_ms) const {
  if (!alive_ || !have_ || (uint32_t)(now_ms - t_ms_) > stale_ms_) return 0.0f;
  return m_;
}

int CliffRetry::next(const bool dead[4], bool motors_off, uint32_t now_ms) {
  if (!motors_off || (uint32_t)(now_ms - last_ms_) < interval_ms_) return -1;
  for (int k = 0; k < 4; k++) {
    const int i = (rr_ + k) % 4;
    if (dead[i] && tries_[i] < max_tries_) {
      ++tries_[i];
      rr_ = (i + 1) % 4;
      last_ms_ = now_ms;
      return i;
    }
  }
  return -1;
}

}  // namespace luna
