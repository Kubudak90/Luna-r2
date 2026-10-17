// Threshold with hysteresis + hold time, for the dock charge-voltage sense. Plain C++, host-tested.
// The output only changes after the input has stayed on the other side of its threshold for
// hold_ms, so a single noisy ADC sample can neither start nor end a "docked" state.
#pragma once
#include <cstdint>

namespace luna {

class ThresholdDebounce {
 public:
  ThresholdDebounce(float on_at, float off_below, uint32_t hold_ms)
      : on_(on_at), off_(off_below), hold_ms_(hold_ms) {}
  bool update(float v, uint32_t now_ms);   // NaN counts as "below"
  bool state() const { return state_; }

 private:
  float on_, off_;
  uint32_t hold_ms_;
  bool state_ = false, pending_ = false;
  uint32_t since_ms_ = 0;
};

}  // namespace luna
