// Luna MCU firmware skeleton (Teensy 4.1 + micro-ROS, Humble). NOT COMPILED in this delivery.
// Safety layers: hardware E-stop cuts driver SLP and servo EN; this firmware also stops on
// E-stop sense, on fault pins, and when no wheel target arrives for 300 ms.
#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <QuadEncoder.h>
#include <VL53L1X.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/u_int8.h>
#include <std_msgs/msg/bool.h>
#include <sensor_msgs/msg/battery_state.h>
#include "pins.h"
#include "topics.h"

static QuadEncoder enc1(1, ENC1_A, ENC1_B), enc2(2, ENC2_A, ENC2_B);
static Servo pan, tilt;
static VL53L1X corner[4];
constexpr float COUNTS_PER_REV = 6533.0f;          // Pololu 4755 at the output shaft
constexpr float KP = 0.08f, KI = 0.6f, MAX_I = 0.5f; // CALIBRATE on the real tracks
constexpr uint32_t CMD_TIMEOUT_MS = 300;

static float target[2] = {0, 0}, pwm_cap = 0.0f, integ[2] = {0, 0}, pos[2] = {0, 0}, vel[2] = {0, 0};
static uint32_t last_cmd_ms = 0;
static float head_us[2] = {1500, 1500};

rcl_publisher_t p_wheels, p_bump, p_cliff, p_corner, p_estop, p_sense, p_batt;
rcl_subscription_t s_target, s_head;
std_msgs__msg__Float32MultiArray m_wheels, m_cliff, m_corner, m_target, m_head;
static float b_wheels[4], b_cliff[4], b_corner[4], b_target[3], b_head[2];
std_msgs__msg__UInt8 m_bump;
std_msgs__msg__Bool m_estop, m_sense;
sensor_msgs__msg__BatteryState m_batt;
rclc_executor_t exec;
rclc_support_t support;
rcl_node_t node;
rcl_allocator_t alloc;
rcl_timer_t t_fast;

static void bind(std_msgs__msg__Float32MultiArray &m, float *buf, size_t n) {
  m.data.data = buf; m.data.capacity = n; m.data.size = n;
}

static bool estop_pressed() { return digitalRead(ESTOP_SENSE) == HIGH; }
static bool driver_fault() { return digitalRead(M1_FLT) == LOW || digitalRead(M2_FLT) == LOW; }

static void drive(int ch, float duty) {      // duty -1..1, already limited by pwm_cap
  const int pwm = ch == 0 ? M1_PWM : M2_PWM, dir = ch == 0 ? M1_DIR : M2_DIR;
  digitalWrite(dir, duty < 0 ? HIGH : LOW);
  analogWrite(pwm, (int)(fabsf(duty) * 4095));
}

static void stop_all() {
  digitalWrite(DRV_SLP, LOW); drive(0, 0); drive(1, 0); integ[0] = integ[1] = 0;
}

static void on_target(const void *msg) {
  const auto *m = (const std_msgs__msg__Float32MultiArray *)msg;
  if (m->data.size < 3) return;
  target[0] = m->data.data[0]; target[1] = m->data.data[1];
  pwm_cap = constrain(m->data.data[2], 0.0f, 1.0f);
  last_cmd_ms = millis();
}

static void on_head(const void *msg) {
  const auto *m = (const std_msgs__msg__Float32MultiArray *)msg;
  if (m->data.size < 2) return;
  head_us[0] = constrain(m->data.data[0], 1100.0f, 1900.0f);   // ±60° mechanical window
  head_us[1] = constrain(m->data.data[1], 1420.0f, 1580.0f);   // ±12°
}

static uint8_t read_bumpers() {
  const int pins[4] = {BUMP_FL, BUMP_FR, BUMP_BL, BUMP_BR};
  uint8_t b = 0;
  for (int i = 0; i < 4; i++) if (digitalRead(pins[i]) == LOW) b |= 1 << i;
  return b;
}

static void control_100hz() {
  static uint32_t last_us = micros();
  const uint32_t now = micros();
  const float dt = (now - last_us) * 1e-6f; last_us = now;
  const int32_t c[2] = {enc1.read(), enc2.read()};
  for (int i = 0; i < 2; i++) {
    const float p = c[i] / COUNTS_PER_REV * TWO_PI;
    vel[i] = dt > 0 ? (p - pos[i]) / dt : 0; pos[i] = p;
  }
  const bool timed_out = millis() - last_cmd_ms > CMD_TIMEOUT_MS;
  if (estop_pressed() || driver_fault() || timed_out || pwm_cap <= 0) { stop_all(); return; }
  digitalWrite(DRV_SLP, HIGH);
  for (int i = 0; i < 2; i++) {
    const float err = target[i] - vel[i];
    integ[i] = constrain(integ[i] + err * KI * dt, -MAX_I, MAX_I);
    const float ff = target[i] / 10.0f;          // ~10 rad/s at full 12 V duty; CALIBRATE
    drive(i, constrain(ff + KP * err + integ[i], -pwm_cap, pwm_cap));
  }
}

static void timer_cb(rcl_timer_t *, int64_t) {
  control_100hz();
  pan.writeMicroseconds((int)head_us[0]);
  tilt.writeMicroseconds((int)head_us[1]);
  digitalWrite(SERVO_EN, estop_pressed() ? LOW : HIGH);
  static uint8_t div = 0;
  if (++div % 2) return;                         // publish at 50 Hz
  b_wheels[0] = pos[0]; b_wheels[1] = pos[1]; b_wheels[2] = vel[0]; b_wheels[3] = vel[1];
  rcl_publish(&p_wheels, &m_wheels, NULL);
  m_bump.data = read_bumpers(); rcl_publish(&p_bump, &m_bump, NULL);
  m_estop.data = estop_pressed(); rcl_publish(&p_estop, &m_estop, NULL);
  m_sense.data = analogRead(VCHG_ADC) * (3.3f / 1023.0f) * VCHG_DIVIDER > 15.0f; rcl_publish(&p_sense, &m_sense, NULL);
  if (div % 10 == 0) {                           // 5 Hz: ranges and battery
    for (int i = 0; i < 4; i++) b_corner[i] = corner[i].read(false) / 1000.0f;
    // TODO(R02 yazılım): VL53L4CD floor sensors on Wire1 → b_cliff; until then report 0 = "no floor" (safe stop).
    for (int i = 0; i < 4; i++) b_cliff[i] = 0.0f;
    rcl_publish(&p_corner, &m_corner, NULL);
    rcl_publish(&p_cliff, &m_cliff, NULL);
    // TODO(R02 yazılım): parse JBD BMS frames on Serial1 (0xDD 0xA5 0x03 …) into m_batt.
    rcl_publish(&p_batt, &m_batt, NULL);
  }
}

static void init_tof() {
  for (int i = 0; i < 4; i++) { pinMode(XSHUT_CORNER[i], OUTPUT); digitalWrite(XSHUT_CORNER[i], LOW); }
  delay(10);
  for (int i = 0; i < 4; i++) {                  // wake one at a time and move it off 0x29
    digitalWrite(XSHUT_CORNER[i], HIGH); delay(10);
    corner[i].setTimeout(50);
    if (corner[i].init()) { corner[i].setAddress(0x30 + i); corner[i].startContinuous(50); }
  }
}

void setup() {
  for (int p : {M1_DIR, M2_DIR, DRV_SLP, SERVO_EN}) pinMode(p, OUTPUT);
  for (int p : {ESTOP_SENSE, BUMP_FL, BUMP_FR, BUMP_BL, BUMP_BR, M1_FLT, M2_FLT}) pinMode(p, INPUT_PULLUP);
  analogWriteResolution(12);
  analogWriteFrequency(M1_PWM, 20000); analogWriteFrequency(M2_PWM, 20000);
  stop_all();
  enc1.setInitConfig(); enc1.init(); enc2.setInitConfig(); enc2.init();
  pan.attach(SERVO_PAN, 500, 2500); tilt.attach(SERVO_TILT, 500, 2500);
  Wire.begin(); Wire.setClock(400000); init_tof();
  Serial1.begin(9600);                            // JBD default UART speed

  set_microros_serial_transports(Serial);
  alloc = rcl_get_default_allocator();
  while (rclc_support_init(&support, 0, NULL, &alloc) != RCL_RET_OK) delay(500);   // wait for the agent
  rclc_node_init_default(&node, "luna_mcu", "", &support);
  const auto F = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray);
  rclc_publisher_init_best_effort(&p_wheels, &node, F, T_WHEELS);
  rclc_publisher_init_default(&p_bump, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), T_BUMPERS);
  rclc_publisher_init_default(&p_cliff, &node, F, T_CLIFF);
  rclc_publisher_init_default(&p_corner, &node, F, T_CORNERS);
  rclc_publisher_init_default(&p_estop, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), T_ESTOP);
  rclc_publisher_init_default(&p_sense, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), T_DOCK_SENSE);
  rclc_publisher_init_default(&p_batt, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState), T_BATTERY);
  rclc_subscription_init_default(&s_target, &node, F, T_WHEEL_TARGET);
  rclc_subscription_init_default(&s_head, &node, F, T_HEAD_US);
  bind(m_wheels, b_wheels, 4); bind(m_cliff, b_cliff, 4); bind(m_corner, b_corner, 4);
  bind(m_target, b_target, 3); bind(m_head, b_head, 2);
  rclc_timer_init_default(&t_fast, &support, RCL_MS_TO_NS(10), timer_cb);
  rclc_executor_init(&exec, &support.context, 3, &alloc);
  rclc_executor_add_timer(&exec, &t_fast);
  rclc_executor_add_subscription(&exec, &s_target, &m_target, &on_target, ON_NEW_DATA);
  rclc_executor_add_subscription(&exec, &s_head, &m_head, &on_head, ON_NEW_DATA);
}

void loop() {
  rclc_executor_spin_some(&exec, RCL_MS_TO_NS(2));
}
