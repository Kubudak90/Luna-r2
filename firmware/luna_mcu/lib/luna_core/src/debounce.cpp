#include "debounce.h"

namespace luna {

bool ThresholdDebounce::update(float v, uint32_t now_ms) {
  const bool raw = state_ ? v >= off_ : v >= on_;
  if (raw == state_) {
    pending_ = false;
  } else if (!pending_) {
    pending_ = true;
    since_ms_ = now_ms;
  } else if ((uint32_t)(now_ms - since_ms_) >= hold_ms_) {
    state_ = raw;
    pending_ = false;
  }
  return state_;
}

}  // namespace luna
