// Luna R2 MCU firmware (Teensy 4.1 + micro-ROS, Humble). Logic lives in lib/luna_core (host-tested).
// Safety layers: hardware E-stop cuts the motor driver's SLP; this firmware also stops on
// E-stop sense, on fault pins, when no wheel target arrives for 300 ms and while the micro-ROS
// agent is unreachable; a 0.5 s hardware watchdog resets the MCU if the control loop hangs.
// DOCK_INHIBIT follows /luna/mcu/charge_enable; charging is allowed unless the AGX keeps asking otherwise.
#include <Arduino.h>
#include <Wire.h>
#include <QuadEncoder.h>
#include <VL53L1X.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/u_int8.h>
#include <std_msgs/msg/bool.h>
#include <sensor_msgs/msg/battery_state.h>
#include "pins.h"
#include "topics.h"
#include "cliff.h"
#include "debounce.h"
#include "dock_id.h"
#include "drive.h"
#include "daly_bms.h"

static_assert((int)luna::BATT_STATUS_CHARGING == (int)sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_CHARGING, "BatteryState constants");
static_assert((int)luna::BATT_STATUS_FULL == (int)sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_FULL, "BatteryState constants");
static_assert((int)luna::BATT_HEALTH_COLD == (int)sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_COLD, "BatteryState constants");
static_assert((int)luna::BATT_TECH_LIFE == (int)sensor_msgs__msg__BatteryState__POWER_SUPPLY_TECHNOLOGY_LIFE, "BatteryState constants");

static QuadEncoder enc1(1, ENC1_A, ENC1_B), enc2(2, ENC2_A, ENC2_B);
static VL53L1X corner[4];
static VL53L1X cliff_tof[4];                    // floor sensors on Wire1
constexpr uint8_t CLIFF_ADDR[4] = {0x40, 0x41, 0x42, 0x43};
constexpr uint32_t CLIFF_STALE_MS = 100, BMS_STALE_MS = 3000, BMS_POLL_MS = 125;
constexpr float PACK_DESIGN_AH = 12.0f;         // 4S2P LiFePO4 32700, 2 × 6 Ah
constexpr uint32_t PING_MS = 500, TIME_SYNC_MS = 60000;
constexpr uint32_t CHARGE_LEASE_MS = 1000;      // a charge_enable=false expires after this (dock_node: 20 Hz)
static luna::CliffChannel cliff_ch[4] = {luna::CliffChannel(CLIFF_STALE_MS), luna::CliffChannel(CLIFF_STALE_MS),
                                         luna::CliffChannel(CLIFF_STALE_MS), luna::CliffChannel(CLIFF_STALE_MS)};
static luna::CliffRetry cliff_retry(2000, 5);
static luna::DalyParser bms_parser;
static luna::BatteryMonitor bms(BMS_STALE_MS, PACK_DESIGN_AH);
static luna::DriveController ctl;
static luna::ThresholdDebounce dock_sense(luna::P4_SCORE_ON, luna::P4_SCORE_OFF, 100);   // P4 ID window, 100 ms hold
static luna::ChargeEnable charge_enable(CHARGE_LEASE_MS);   // DOCK_INHIBIT request (Elektrik R01 README §5, Faz 2)
static bool motors_enabled = false;
static uint32_t publish_errors = 0, last_sync_ms = 0;

rcl_publisher_t p_wheels, p_bump, p_cliff, p_corner, p_estop, p_sense, p_batt;
rcl_subscription_t s_target, s_charge;
std_msgs__msg__Float32MultiArray m_wheels, m_cliff, m_corner, m_target;
static float b_wheels[4], b_cliff[4], b_corner[4], b_target[3], b_cells[luna::DALY_MAX_CELLS];
std_msgs__msg__UInt8 m_bump;
std_msgs__msg__Bool m_estop, m_sense, m_charge;
sensor_msgs__msg__BatteryState m_batt;
rclc_executor_t exec;
rclc_support_t support;
rcl_node_t node;
rcl_allocator_t alloc;
rcl_timer_t t_fast;

enum class Agent { Waiting, Available, Connected, Disconnected };
static Agent agent = Agent::Waiting;

static void ignore(int32_t) {}                 // consumes warn_unused_result (a (void) cast does not)

// WDOG1: timeout (WT + 1) x 0.5 s = 0.5 s. WDE is write-once: once started it cannot be stopped.
// A timeout resets the whole chip (CrashReport reports it as WDOG_RST_B).
static void watchdog_start() {
  CCM_CCGR3 |= CCM_CCGR3_WDOG1(3);
  WDOG1_WMCR = 0;                              // power-down counter off
  WDOG1_WCR = WDOG_WCR_SRS | WDOG_WCR_WDA | WDOG_WCR_WT(0);
  WDOG1_WCR |= WDOG_WCR_WDE;
}

static void watchdog_feed() { WDOG1_WSR = 0x5555; WDOG1_WSR = 0xAAAA; }

static void bind(std_msgs__msg__Float32MultiArray &m, float *buf, size_t n) {
  m.data.data = buf; m.data.capacity = n; m.data.size = n;
}

static void pub(rcl_publisher_t &p, const void *m) {
  if (rcl_publish(&p, m, NULL) != RCL_RET_OK) ++publish_errors;
}

static bool estop_pressed() { return digitalRead(ESTOP_SENSE) == HIGH; }
static bool driver_fault() { return digitalRead(M1_FLT) == LOW || digitalRead(M2_FLT) == LOW; }

static void drive(int ch, float duty) {      // duty -1..1, already limited by pwm_cap
  const int pwm = ch == 0 ? M1_PWM : M2_PWM, dir = ch == 0 ? M1_DIR : M2_DIR;
  const luna::PwmCommand p = luna::duty_to_pwm(duty, 4095);
  digitalWrite(dir, p.reverse ? HIGH : LOW);
  analogWrite(pwm, p.level);
}

static void stop_all() {
  digitalWrite(DRV_SLP, LOW); drive(0, 0); drive(1, 0);
  motors_enabled = false;
}

// No agent: motors off, charging allowed, watchdog fed from loop().
static void hold_safe() {
  stop_all();
  charge_enable.reset(); digitalWrite(DOCK_INHIBIT, LOW);
  watchdog_feed();
}

static void on_target(const void *msg) {
  const auto *m = (const std_msgs__msg__Float32MultiArray *)msg;
  if (m->data.size < 3) return;
  ctl.set_target(m->data.data[0], m->data.data[1], m->data.data[2], millis());   // NaN → stop
}

static void on_charge(const void *msg) {
  charge_enable.on_msg(((const std_msgs__msg__Bool *)msg)->data, millis());
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
  const luna::DriveOutput o = ctl.step(c, dt, millis(), estop_pressed(), driver_fault());
  if (!o.enable) { stop_all(); return; }
  digitalWrite(DRV_SLP, HIGH);
  drive(0, o.duty[0]); drive(1, o.duty[1]);
  motors_enabled = true;
}

// Probe first: a sensor that dropped off the bus would otherwise read as "data ready" (a NACKed register reads 0).
static bool cliff_probe(int i) {
  Wire1.beginTransmission(CLIFF_ADDR[i]);
  return Wire1.endTransmission() == 0;
}

static bool cliff_start(int i) {               // sensor i must be the only one that can sit at 0x29
  digitalWrite(XSHUT_CLIFF[i], LOW); delay(2);
  digitalWrite(XSHUT_CLIFF[i], HIGH); delay(2);  // VL53L1X boots in ~1.2 ms, at 0x29
  VL53L1X &t = cliff_tof[i];
  t = VL53L1X();                                // forget the old address
  t.setBus(&Wire1);
  t.setTimeout(50);
  if (!t.init()) return false;
  t.setAddress(CLIFF_ADDR[i]);
  // Floor 60 mm below the window: short mode (to 1.3 m, least disturbed by sunlight), 20 ms budget, 50 Hz.
  if (!t.setDistanceMode(VL53L1X::Short) || !t.setMeasurementTimingBudget(20000)) return false;
  t.startContinuous(20);
  return t.last_status == 0;
}

static void poll_cliff(int i) {
  if (!cliff_ch[i].alive()) return;
  VL53L1X &t = cliff_tof[i];
  if (!cliff_probe(i)) { cliff_ch[i].set_alive(false); return; }   // 0.0 (= cliff) until a re-init succeeds
  if (!t.dataReady()) return;
  const uint16_t mm = t.read(false);
  if (t.last_status != 0) { cliff_ch[i].set_alive(false); return; }
  cliff_ch[i].on_sample((uint8_t)t.ranging_data.range_status, mm, millis());
}

static void retry_dead_cliff() {
  bool dead[4];
  for (int k = 0; k < 4; k++) dead[k] = !cliff_ch[k].alive();
  const int i = cliff_retry.next(dead, !motors_enabled, millis());
  if (i < 0) return;
  for (int k = 0; k < 4; k++) if (dead[k]) digitalWrite(XSHUT_CLIFF[k], LOW);   // the others back off 0x29
  watchdog_feed();                             // init blocks for ~25 ms
  cliff_ch[i].set_alive(cliff_start(i));
}

// One request every BMS_POLL_MS: 0x90 (voltage, current, SoC) every other slot → 4 Hz; the other five in turn,
// each every 1.25 s. A 0x95 reply for 4 cells is two frames, 27 ms at 9600 baud: done before the next request.
static void poll_bms() {
  static uint32_t last_req = 0;
  static uint8_t slot = 0;
  static const uint8_t others[5] = {luna::DALY_CELLS, luna::DALY_TEMPS, luna::DALY_MOSFET, luna::DALY_ALARMS,
                                    luna::DALY_STATUS};
  while (Serial1.available()) {
    const uint32_t now = millis();
    switch (bms_parser.feed((uint8_t)Serial1.read())) {
      case luna::DalyFrame::Soc: bms.on_soc(bms_parser.soc(), now); break;
      case luna::DalyFrame::Temps: bms.on_temps(bms_parser.temps(), now); break;
      case luna::DalyFrame::Mosfet: bms.on_mosfet(bms_parser.mosfet(), now); break;
      case luna::DalyFrame::Cells: bms.on_cells(bms_parser.cells(), now); break;
      case luna::DalyFrame::Alarms: bms.on_alarms(bms_parser.alarms(), now); break;
      case luna::DalyFrame::Status:              // only tells the parser how many cells a 0x95 reply has
      case luna::DalyFrame::None: break;
    }
  }
  if (millis() - last_req >= BMS_POLL_MS) {
    last_req = millis();
    const uint8_t cmd = slot % 2 == 0 ? luna::DALY_SOC : others[slot / 2];
    slot = (uint8_t)((slot + 1) % 10);
    uint8_t req[luna::DALY_FRAME];
    bms_parser.reset();                          // a half frame must not swallow the next reply
    Serial1.write(req, luna::daly_build_request(cmd, req));
  }
}

static void fill_battery() {
  const luna::BatteryFields f = bms.fields(millis());   // present=false, voltage 0 when absent/stale
  m_batt.voltage = f.voltage; m_batt.current = f.current; m_batt.charge = f.charge;
  m_batt.capacity = f.capacity; m_batt.design_capacity = f.design_capacity;
  m_batt.percentage = f.percentage; m_batt.temperature = f.temperature;
  m_batt.power_supply_status = f.status; m_batt.power_supply_health = f.health;
  m_batt.power_supply_technology = f.technology; m_batt.present = f.present;
  for (uint8_t i = 0; i < f.cell_count; i++) b_cells[i] = f.cell_v[i];
  m_batt.cell_voltage.data = b_cells; m_batt.cell_voltage.capacity = luna::DALY_MAX_CELLS;
  m_batt.cell_voltage.size = f.cell_count;
  if (rmw_uros_epoch_synchronized()) {
    const int64_t ns = rmw_uros_epoch_nanos();
    m_batt.header.stamp.sec = (int32_t)(ns / 1000000000LL);
    m_batt.header.stamp.nanosec = (uint32_t)(ns % 1000000000LL);
  }
}

static void timer_cb(rcl_timer_t *, int64_t) {
  watchdog_feed();
  control_100hz();
  static uint8_t rr = 0;                         // two floor sensors per tick → each at 50 Hz
  poll_cliff(rr); poll_cliff(rr + 1); rr = rr ? 0 : 2;
  retry_dead_cliff();
  poll_bms();
  const bool inhibit = charge_enable.inhibit(millis());
  digitalWrite(DOCK_INHIBIT, inhibit ? HIGH : LOW);
  dock_sense.update(luna::p4_score(analogRead(DOCK_ID_ADC) * (3.3f / 1023.0f), inhibit), millis());
  static uint8_t div = 0;
  if (++div % 2) return;                         // publish at 50 Hz
  b_wheels[0] = ctl.position_rad(0); b_wheels[1] = ctl.position_rad(1);
  b_wheels[2] = ctl.velocity_rad_s(0); b_wheels[3] = ctl.velocity_rad_s(1);
  pub(p_wheels, &m_wheels);
  m_bump.data = read_bumpers(); pub(p_bump, &m_bump);
  m_estop.data = estop_pressed(); pub(p_estop, &m_estop);
  for (int i = 0; i < 4; i++) b_cliff[i] = cliff_ch[i].metres(millis());   // [FL, BL, FR, BR], 0 = cliff
  pub(p_cliff, &m_cliff);
  m_sense.data = dock_sense.state(); pub(p_sense, &m_sense);
  if (div % 10 == 0) {                           // 5 Hz: ranges and battery
    for (int i = 0; i < 4; i++) b_corner[i] = corner[i].read(false) / 1000.0f;
    pub(p_corner, &m_corner);
    fill_battery();
    pub(p_batt, &m_batt);
  }
}

static void sync_time() {
  ignore(rmw_uros_sync_session(100));
  last_sync_ms = millis();
}

#define TRY(x) do { watchdog_feed(); if ((x) != RCL_RET_OK) return false; } while (0)

static bool create_entities() {
  p_wheels = p_bump = p_cliff = p_corner = p_estop = p_sense = p_batt = rcl_get_zero_initialized_publisher();
  s_target = s_charge = rcl_get_zero_initialized_subscription();
  t_fast = rcl_get_zero_initialized_timer();
  node = rcl_get_zero_initialized_node();
  exec = rclc_executor_get_zero_initialized_executor();
  alloc = rcl_get_default_allocator();
  TRY(rclc_support_init(&support, 0, NULL, &alloc));
  TRY(rclc_node_init_default(&node, "luna_mcu", "", &support));
  const auto F = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray);
  TRY(rclc_publisher_init_best_effort(&p_wheels, &node, F, T_WHEELS));
  TRY(rclc_publisher_init_default(&p_bump, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), T_BUMPERS));
  TRY(rclc_publisher_init_default(&p_cliff, &node, F, T_CLIFF));
  TRY(rclc_publisher_init_default(&p_corner, &node, F, T_CORNERS));
  TRY(rclc_publisher_init_default(&p_estop, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), T_ESTOP));
  TRY(rclc_publisher_init_default(&p_sense, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), T_DOCK_SENSE));
  TRY(rclc_publisher_init_default(&p_batt, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState), T_BATTERY));
  TRY(rclc_subscription_init_default(&s_target, &node, F, T_WHEEL_TARGET));
  TRY(rclc_subscription_init_default(&s_charge, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), T_CHARGE_ENABLE));
  TRY(rclc_timer_init_default(&t_fast, &support, RCL_MS_TO_NS(10), timer_cb));
  TRY(rclc_executor_init(&exec, &support.context, 3, &alloc));
  TRY(rclc_executor_add_timer(&exec, &t_fast));
  TRY(rclc_executor_add_subscription(&exec, &s_target, &m_target, &on_target, ON_NEW_DATA));
  TRY(rclc_executor_add_subscription(&exec, &s_charge, &m_charge, &on_charge, ON_NEW_DATA));
  sync_time();
  return true;
}

static void destroy_entities() {
  rmw_context_t *ctx = rcl_context_get_rmw_context(&support.context);
  ignore(rmw_uros_set_context_entity_destroy_session_timeout(ctx, 0));   // agent is gone: do not wait
  for (rcl_publisher_t *p : {&p_wheels, &p_bump, &p_cliff, &p_corner, &p_estop, &p_sense, &p_batt})
    ignore(rcl_publisher_fini(p, &node));
  ignore(rcl_subscription_fini(&s_target, &node));
  ignore(rcl_subscription_fini(&s_charge, &node));
  ignore(rcl_timer_fini(&t_fast));
  ignore(rclc_executor_fini(&exec));
  ignore(rcl_node_fini(&node));
  ignore(rclc_support_fini(&support));
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

static void init_cliff() {
  for (int i = 0; i < 4; i++) { pinMode(XSHUT_CLIFF[i], OUTPUT); digitalWrite(XSHUT_CLIFF[i], LOW); }
  Wire1.begin(); Wire1.setClock(400000);
  delay(10);
  for (int i = 0; i < 4; i++) cliff_ch[i].set_alive(cliff_start(i));   // one at a time off 0x29
}

void setup() {
  for (int p : {M1_DIR, M2_DIR, DRV_SLP, DOCK_INHIBIT}) pinMode(p, OUTPUT);
  digitalWrite(DOCK_INHIBIT, LOW);                // charging allowed (also the hardware default: 100k pull-down)
  for (int p : {ESTOP_SENSE, BUMP_FL, BUMP_FR, BUMP_BL, BUMP_BR, M1_FLT, M2_FLT}) pinMode(p, INPUT_PULLUP);
  analogWriteResolution(12);
  analogWriteFrequency(M1_PWM, 20000); analogWriteFrequency(M2_PWM, 20000);
  stop_all();
  enc1.setInitConfig(); enc1.init(); enc2.setInitConfig(); enc2.init();
  Wire.begin(); Wire.setClock(400000); init_tof(); init_cliff();
  Serial1.begin(9600);                            // Daly UART speed

  sensor_msgs__msg__BatteryState__init(&m_batt);   // allocates the empty strings micro-ROS serialises
  bind(m_wheels, b_wheels, 4); bind(m_cliff, b_cliff, 4); bind(m_corner, b_corner, 4);
  bind(m_target, b_target, 3);
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  watchdog_start();
}

// Agent state machine (micro-ROS reconnection pattern): motors stay off unless connected.
void loop() {
  static uint32_t last_ping = 0;
  switch (agent) {
    case Agent::Waiting:
      hold_safe();
      if (millis() - last_ping >= PING_MS) {
        last_ping = millis();
        if (rmw_uros_ping_agent(50, 1) == RMW_RET_OK) agent = Agent::Available;
      }
      break;
    case Agent::Available:
      agent = create_entities() ? Agent::Connected : Agent::Disconnected;
      break;
    case Agent::Connected:
      if (millis() - last_ping >= PING_MS) {
        last_ping = millis();
        if (rmw_uros_ping_agent(50, 2) != RMW_RET_OK) { agent = Agent::Disconnected; break; }
        if (millis() - last_sync_ms >= TIME_SYNC_MS) sync_time();
      }
      ignore(rclc_executor_spin_some(&exec, RCL_MS_TO_NS(2)));
      break;
    case Agent::Disconnected:
      hold_safe();
      destroy_entities();
      agent = Agent::Waiting;
      break;
  }
}
