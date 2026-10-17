// Floor-facing VL53L4CD channel → metres for /luna/mcu/cliff. Plain C++, host-tested.
// Fail-safe contract with the AGX safety gate: 0.0 means "no usable floor reading" and is
// treated as a cliff. A channel reports 0.0 when the sensor never initialised, dropped off the
// bus, has not produced a sample yet, its last sample was not valid, or that sample is stale.
#pragma once
#include <cstdint>

namespace luna {

// VL53L4CD ULD range_status after the driver's remapping: 0 is the only "distance valid".
// Everything else (1 sigma, 2 signal, 3 below min range, 4/7 wrap, 5 HW fail, 6 no wrap check,
// 255 other ...) is treated as invalid on purpose: this sensor guards a stair edge.
constexpr uint8_t VL53L4CD_STATUS_VALID = 0;

class CliffChannel {
 public:
  explicit CliffChannel(uint32_t stale_ms) : stale_ms_(stale_ms) {}
  void set_alive(bool alive) { alive_ = alive; if (!alive) have_ = false; }
  bool alive() const { return alive_; }
  void on_sample(uint8_t range_status, uint16_t distance_mm, uint32_t now_ms);
  float metres(uint32_t now_ms) const;

 private:
  uint32_t stale_ms_;
  bool alive_ = false, have_ = false;
  float m_ = 0;
  uint32_t t_ms_ = 0;
};

// Picks the next dead floor sensor to re-initialise: at most one attempt per interval, only while
// the motors are off (the driver's init blocks for ~25 ms), and at most max_tries per sensor.
class CliffRetry {
 public:
  CliffRetry(uint32_t interval_ms, uint8_t max_tries) : interval_ms_(interval_ms), max_tries_(max_tries) {}
  // dead[i]: channel i is not alive. Returns the channel to try now, or -1.
  int next(const bool dead[4], bool motors_off, uint32_t now_ms);

 private:
  uint32_t interval_ms_, last_ms_ = 0;
  uint8_t max_tries_, tries_[4] = {0, 0, 0, 0};
  int rr_ = 0;
};

}  // namespace luna
