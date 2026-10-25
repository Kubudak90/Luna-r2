// Host unit tests for lib/luna_core (no Arduino, no hardware). Run: ./host_test/run.sh
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <initializer_list>
#include <vector>

#include "cliff.h"
#include "debounce.h"
#include "dock_id.h"
#include "drive.h"
#include "daly_bms.h"

using namespace luna;

static int g_checks = 0, g_fail = 0;
#define CHECK(c)                                                              \
  do {                                                                        \
    ++g_checks;                                                               \
    if (!(c)) { ++g_fail; std::printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); } \
  } while (0)
#define CHECK_NEAR(a, b, eps) CHECK(std::fabs((double)(a) - (double)(b)) <= (eps))

using Bytes = std::vector<uint8_t>;

// A BMS reply frame: A5 01 cmd 08 data[8] chk.
static Bytes reply(uint8_t cmd, const Bytes &data, uint8_t addr = DALY_BMS) {
  Bytes f = {DALY_START, addr, cmd, DALY_LEN};
  for (size_t i = 0; i < 8; i++) f.push_back(i < data.size() ? data[i] : 0);
  f.push_back(daly_checksum(f.data(), 12));
  return f;
}

static Bytes be16(uint16_t v) { return {(uint8_t)(v >> 8), (uint8_t)v}; }

static Bytes cat(std::initializer_list<Bytes> parts) {
  Bytes out;
  for (const Bytes &p : parts) out.insert(out.end(), p.begin(), p.end());
  return out;
}

// Luna pack: 4S2P LiFePO4 32700, here 13.2 V, discharging 2.3 A (raw 30023), SoC 64.5 %.
static Bytes luna_soc(uint16_t raw_current = 30023) { return cat({be16(132), be16(0), be16(raw_current), be16(645)}); }

static int feed_all(DalyParser &p, const Bytes &bytes, DalyFrame want = DalyFrame::None) {
  int n = 0;
  for (uint8_t b : bytes) {
    const DalyFrame f = p.feed(b);
    if (f != DalyFrame::None && (want == DalyFrame::None || f == want)) ++n;
  }
  return n;
}

static void test_request_bytes() {
  uint8_t req[DALY_FRAME];
  CHECK(daly_build_request(DALY_SOC, req) == 13);
  const uint8_t want[13] = {0xA5, 0x40, 0x90, 0x08, 0, 0, 0, 0, 0, 0, 0, 0, 0x7D};   // documented 0x90 request
  bool same = true;
  for (int i = 0; i < 13; i++) same = same && req[i] == want[i];
  CHECK(same);
  daly_build_request(DALY_CELLS, req);
  CHECK(req[2] == 0x95 && req[12] == 0x82);
  daly_build_request(DALY_ALARMS, req);
  CHECK(req[12] == 0x85);
}

static void test_soc_frame() {
  DalyParser p;
  CHECK(feed_all(p, reply(DALY_SOC, luna_soc())) == 1);
  CHECK_NEAR(p.soc().voltage_v, 13.2, 1e-4);
  CHECK_NEAR(p.soc().current_a, -2.3, 1e-4);           // above the offset = discharging = negative
  CHECK_NEAR(p.soc().soc_pct, 64.5, 1e-4);
  CHECK(feed_all(p, reply(DALY_SOC, luna_soc(29600))) == 1);
  CHECK_NEAR(p.soc().current_a, 40.0, 1e-4);           // 4 A charging
  CHECK(feed_all(p, reply(DALY_SOC, luna_soc(30000))) == 1);
  CHECK(p.soc().current_a == 0.0f);
  CHECK(p.frames_ok() == 3 && p.frames_bad() == 0);
}

static void test_other_frames() {
  DalyParser p;
  CHECK(feed_all(p, reply(DALY_TEMPS, {66, 1, 63, 2})) == 1);           // 26 °C hottest (sensor 1), 23 °C
  CHECK(p.temps().max_c == 26.0f && p.temps().min_c == 23.0f);
  CHECK(feed_all(p, reply(DALY_TEMPS, {30, 1, 25, 1})) == 1);           // below zero
  CHECK(p.temps().max_c == -10.0f && p.temps().min_c == -15.0f);
  CHECK(feed_all(p, reply(DALY_MOSFET, {2, 1, 1, 7, 0x00, 0x00, 0x1E, 0x14})) == 1);   // 7700 mAh left
  CHECK(p.mosfet().mode == 2 && p.mosfet().charge_fet && p.mosfet().discharge_fet);
  CHECK_NEAR(p.mosfet().remaining_ah, 7.7, 1e-4);
  CHECK(feed_all(p, reply(DALY_STATUS, {4, 1, 0, 1, 0, 0x01, 0x2C})) == 1);
  CHECK(p.status().cells == 4 && p.status().ntc == 1 && !p.status().charger && p.status().load);
  CHECK(p.status().cycles == 300);
  CHECK(feed_all(p, reply(DALY_ALARMS, {0, 0x02, 0, 0, 0, 0, 0, 0})) == 1);
  CHECK(p.alarms().bits[1] == 0x02);
}

static void test_cell_frames() {
  DalyParser p;                                        // 4 cells by default: two frames
  const Bytes f1 = reply(DALY_CELLS, cat({{1}, be16(3301), be16(3305), be16(3299)}));
  const Bytes f2 = reply(DALY_CELLS, cat({{2}, be16(3310), be16(0), be16(0)}));
  CHECK(feed_all(p, f1) == 0);                         // first frame alone completes nothing
  CHECK(feed_all(p, f2) == 1);
  CHECK(p.cells().count == 4);
  CHECK_NEAR(p.cells().v[0], 3.301, 1e-6);
  CHECK_NEAR(p.cells().v[3], 3.310, 1e-6);
  CHECK(feed_all(p, f2) == 0 && p.frames_bad() == 1);  // frame 2 without its frame 1
  CHECK(feed_all(p, cat({f1, f2})) == 1);              // and a whole reply after it is fine again

  DalyParser q;                                        // a 0x94 cell count wins over the default
  feed_all(q, reply(DALY_STATUS, {3, 1}));
  CHECK(feed_all(q, f1) == 1 && q.cells().count == 3);
  DalyParser r;
  feed_all(r, reply(DALY_STATUS, {200, 1}));           // nonsense count: capped
  int done = 0;
  for (uint8_t no = 1; no <= 6; no++) done += feed_all(r, reply(DALY_CELLS, cat({{no}, be16(3300), be16(3300), be16(3300)})));
  CHECK(done == 1 && r.cells().count == DALY_MAX_CELLS);
}

static void test_rejects_bad_frames() {
  DalyParser p;
  Bytes bad = reply(DALY_SOC, luna_soc());
  bad[12] ^= 0x01;                                     // checksum
  CHECK(feed_all(p, bad) == 0);
  CHECK(feed_all(p, reply(DALY_SOC, luna_soc(), DALY_HOST)) == 0);     // our own request echoed back
  Bytes len = reply(DALY_SOC, luna_soc());
  len[3] = 0x07; len[12] = daly_checksum(len.data(), 12);
  CHECK(feed_all(p, len) == 0);
  CHECK(feed_all(p, reply(0x91, {})) == 0);           // a command Luna does not poll
  CHECK(p.frames_ok() == 0 && p.frames_bad() >= 4);
  CHECK(feed_all(p, reply(DALY_SOC, luna_soc())) == 1);
}

static void test_resync() {
  DalyParser p;
  const Bytes good = reply(DALY_SOC, luna_soc());
  CHECK(feed_all(p, cat({{0x00, 0x13, 0xA5, 0x77}, good})) == 1);    // noise, a stray start byte
  Bytes cut(good.begin(), good.begin() + 7);
  CHECK(feed_all(p, cat({cut, good})) == 1);           // a truncated reply does not hide the next one
  Bytes corrupt = good;
  corrupt[9] ^= 0xFF;
  CHECK(feed_all(p, cat({corrupt, good, good})) == 2);
  p.reset();
  CHECK(feed_all(p, cat({cut})) == 0);
  p.reset();                                           // main.cpp resets before each request
  CHECK(feed_all(p, good) == 1);
}

static void test_health() {
  DalyAlarms a;
  CHECK(daly_health(a) == BATT_HEALTH_GOOD);
  a.bits[1] = DALY_A1_DSG_HOT_L1; a.bits[0] = DALY_A0_CELL_LOW_L1; a.bits[2] = DALY_A2_SOC_LOW_L2;
  CHECK(daly_health(a) == BATT_HEALTH_GOOD);            // warnings and SoC alarms: numbers, AGX decides
  a = DalyAlarms(); a.bits[1] = DALY_A1_CHG_HOT_L2;
  CHECK(daly_health(a) == BATT_HEALTH_OVERHEAT);
  a = DalyAlarms(); a.bits[4] = DALY_A4_DSG_FET_HOT;
  CHECK(daly_health(a) == BATT_HEALTH_OVERHEAT);
  a = DalyAlarms(); a.bits[1] = DALY_A1_CHG_COLD_L2;
  CHECK(daly_health(a) == BATT_HEALTH_COLD);
  a = DalyAlarms(); a.bits[0] = DALY_A0_SUM_HIGH_L2;
  CHECK(daly_health(a) == BATT_HEALTH_OVERVOLTAGE);
  a = DalyAlarms(); a.bits[0] = DALY_A0_CELL_LOW_L2;
  CHECK(daly_health(a) == BATT_HEALTH_DEAD);
  a.bits[1] = DALY_A1_DSG_HOT_L2;
  CHECK(daly_health(a) == BATT_HEALTH_OVERHEAT);        // heat first
  for (int byte : {2, 3, 5, 6}) {
    a = DalyAlarms(); a.bits[byte] = byte == 2 ? DALY_A2_DSG_OC_L2 : byte == 3 ? DALY_A3_DIFF_V_L2 : 0x04;
    CHECK(daly_health(a) == BATT_HEALTH_UNSPEC_FAILURE);
  }
  a = DalyAlarms(); a.bits[4] = 0x10;                  // charge FET stuck
  CHECK(daly_health(a) == BATT_HEALTH_UNSPEC_FAILURE);
  a = DalyAlarms(); a.bits[7] = 0x33;
  CHECK(daly_health(a) == BATT_HEALTH_GOOD);
}

static void test_battery_monitor_fields() {
  DalyParser p;
  BatteryMonitor m(3000, 12.0f);
  feed_all(p, reply(DALY_SOC, luna_soc()));
  m.on_soc(p.soc(), 1000);
  BatteryFields f = m.fields(1000);
  CHECK(f.present && f.technology == BATT_TECH_LIFE);
  CHECK_NEAR(f.voltage, 13.2, 1e-4);
  CHECK_NEAR(f.percentage, 0.645, 1e-5);
  CHECK(f.status == BATT_STATUS_DISCHARGING && f.health == BATT_HEALTH_UNKNOWN);
  CHECK(std::isnan(f.charge) && std::isnan(f.temperature) && f.cell_count == 0);   // their replies not in yet
  CHECK(f.capacity == 12.0f && f.design_capacity == 12.0f);

  feed_all(p, reply(DALY_TEMPS, {72, 1, 64, 2}));
  m.on_temps(p.temps(), 1100);
  feed_all(p, reply(DALY_MOSFET, {2, 1, 1, 0, 0x00, 0x00, 0x1E, 0x14}));
  m.on_mosfet(p.mosfet(), 1200);
  feed_all(p, reply(DALY_ALARMS, {}));
  m.on_alarms(p.alarms(), 1300);
  feed_all(p, cat({reply(DALY_CELLS, cat({{1}, be16(3301), be16(3305), be16(3299)})),
                   reply(DALY_CELLS, cat({{2}, be16(3310)}))}));
  m.on_cells(p.cells(), 1400);
  f = m.fields(1500);
  CHECK(f.temperature == 32.0f && f.health == BATT_HEALTH_GOOD && f.cell_count == 4);
  CHECK_NEAR(f.charge, 7.7, 1e-4);
  CHECK_NEAR(f.cell_v[2], 3.299, 1e-6);

  m.on_soc(p.soc(), 4000);                             // only 0x90 keeps arriving
  f = m.fields(4100);
  CHECK(f.present && f.temperature == 32.0f);          // temps from 1100 still fresh at 4100
  f = m.fields(4401);
  CHECK(f.present && std::isnan(f.temperature) && std::isnan(f.charge) && f.cell_count == 0);
  CHECK(f.health == BATT_HEALTH_UNKNOWN);

  feed_all(p, reply(DALY_SOC, cat({be16(146), be16(0), be16(30000), be16(1000)})));   // full, idle on the dock
  m.on_soc(p.soc(), 5000);
  CHECK(m.fields(5000).status == BATT_STATUS_FULL);
  feed_all(p, reply(DALY_SOC, cat({be16(140), be16(0), be16(29960), be16(800)})));
  m.on_soc(p.soc(), 5100);
  CHECK(m.fields(5100).status == BATT_STATUS_CHARGING);
  feed_all(p, reply(DALY_SOC, cat({be16(132), be16(0), be16(30000), be16(2000)})));   // SoC 200 %: nonsense
  m.on_soc(p.soc(), 5200);
  CHECK(std::isnan(m.fields(5200).percentage) && m.fields(5200).status == BATT_STATUS_NOT_CHARGING);
}

static void test_battery_monitor_fail_safe() {
  const BatteryFields a = battery_fields_absent();
  CHECK(!a.present);
  CHECK(a.voltage == 0.0f && !std::isnan(a.voltage));   // AGX: pwm_cap(0 V) = 0, pwm_cap(NaN) = NaN
  CHECK(a.current == 0.0f);
  CHECK(std::isnan(a.percentage) && std::isnan(a.temperature));

  BatteryMonitor m(3000, 12.0f);
  CHECK(!m.present(0) && !m.fields(0).present && m.fields(0).voltage == 0.0f);
  DalyParser p;
  feed_all(p, reply(DALY_TEMPS, {66, 1, 63, 2}));
  m.on_temps(p.temps(), 0);
  CHECK(!m.present(0));                                // other replies never make it present
  CHECK(feed_all(p, reply(DALY_SOC, luna_soc())) == 1);
  m.on_soc(p.soc(), 1000);
  CHECK(m.present(1000) && m.fields(4000).present);
  CHECK(!m.fields(4001).present);
  CHECK(m.fields(4001).voltage == 0.0f);

  BatteryMonitor w(3000, 12.0f);                       // millis() wrap-around
  w.on_soc(p.soc(), 0xFFFFFF00u);
  CHECK(w.present(0x00000100u));
  CHECK(!w.present(0xFFFFFF00u + 3001u));
}

static void test_cliff_channel() {
  CliffChannel c(100);
  CHECK(c.metres(0) == 0.0f);                   // never initialised
  c.on_sample(0, 55, 0);
  CHECK(c.metres(0) == 0.0f);                   // samples ignored while not alive
  c.set_alive(true);
  CHECK(c.metres(10) == 0.0f);                  // alive, no sample yet
  c.on_sample(0, 55, 10);
  CHECK_NEAR(c.metres(10), 0.055, 1e-6);
  CHECK_NEAR(c.metres(110), 0.055, 1e-6);
  CHECK(c.metres(111) == 0.0f);                 // stale
  c.on_sample(0, 57, 200);
  c.on_sample(2, 57, 220);                      // weak signal → invalid immediately
  CHECK(c.metres(220) == 0.0f);
  for (uint8_t s : {1, 3, 4, 5, 6, 7, 255}) { c.on_sample(0, 55, 300); c.on_sample(s, 55, 301); CHECK(c.metres(301) == 0.0f); }
  c.on_sample(0, 0, 400);
  CHECK(c.metres(400) == 0.0f);
  c.on_sample(0, 1200, 500);                    // a real drop reads long, the gate decides
  CHECK_NEAR(c.metres(500), 1.2, 1e-6);
  c.set_alive(false);                           // dropped off the bus
  CHECK(c.metres(500) == 0.0f);
  c.set_alive(true);
  CHECK(c.metres(500) == 0.0f);                 // old sample not resurrected

  CliffChannel w(100);                          // millis() wrap-around
  w.set_alive(true);
  w.on_sample(0, 60, 0xFFFFFFF0u);
  CHECK_NEAR(w.metres(0x00000020u), 0.060, 1e-6);
  CHECK(w.metres(0xFFFFFFF0u + 101u) == 0.0f);
}

static void test_dock_debounce() {
  ThresholdDebounce d(15.0f, 14.5f, 100);
  CHECK(!d.update(16.8f, 0));                   // rising edge starts the hold
  CHECK(!d.update(16.8f, 99));
  CHECK(d.update(16.8f, 100));                  // held 100 ms → docked
  CHECK(d.update(0.0f, 120));                   // one bad sample does not end it
  CHECK(d.update(16.8f, 140));                  // ... and the hold restarts
  CHECK(d.update(0.0f, 160));
  CHECK(d.update(0.0f, 259));
  CHECK(!d.update(0.0f, 260));                  // 100 ms low → undocked
  CHECK(!d.update(14.8f, 300));                 // inside the hysteresis band: stays off
  CHECK(!d.update(14.8f, 1000));
  CHECK(!d.update(std::nanf(""), 1100));        // NaN reads as "no voltage"
  ThresholdDebounce e(15.0f, 14.5f, 100);
  e.update(16.0f, 0); e.update(16.0f, 100);
  CHECK(e.state());
  CHECK(e.update(14.8f, 200) && e.update(14.8f, 900));   // band keeps "on"
  ThresholdDebounce w(15.0f, 14.5f, 100);       // millis() wrap-around
  w.update(16.0f, 0xFFFFFFC0u);
  CHECK(w.update(16.0f, 0x00000030u));
}

static void test_p4_window() {
  ThresholdDebounce d(P4_SCORE_ON, P4_SCORE_OFF, 100);
  CHECK(!d.update(p4_score(2.50f, false), 0));
  CHECK(d.update(p4_score(2.50f, false), 100));            // seated on a powered dock
  CHECK(d.update(p4_score(2.38f, false), 150));            // tolerance corners stay inside
  CHECK(d.update(p4_score(2.63f, false), 200));
  CHECK(d.update(p4_score(3.15f, false), 300) && d.update(p4_score(3.15f, false), 900));   // hysteresis band
  CHECK(d.update(p4_score(0.0f, false), 1000));            // one bad sample does not end it
  CHECK(!d.update(p4_score(0.0f, false), 1100));           // 100 ms at 0 V → off the dock / dock unpowered
  CHECK(!d.update(p4_score(3.15f, false), 1200) && !d.update(p4_score(3.15f, false), 1900));
  ThresholdDebounce i(P4_SCORE_ON, P4_SCORE_OFF, 100);     // our own inhibit keeps "seated"
  i.update(p4_score(2.50f, false), 0); i.update(p4_score(2.50f, false), 100);
  CHECK(i.state());
  CHECK(i.update(p4_score(2.50f, true), 110));             // P4 still settling (RC ~10 ms) when inhibit is set
  CHECK(i.update(p4_score(0.76f, true), 130) && i.update(p4_score(0.72f, true), 400));
  CHECK(i.update(p4_score(2.50f, false), 410));            // released again
  ThresholdDebounce e(P4_SCORE_ON, P4_SCORE_OFF, 100);
  e.update(p4_score(0.76f, false), 0);                     // inhibit level without our request
  CHECK(!e.update(p4_score(0.76f, false), 200));
  e.update(p4_score(3.30f, false), 300);                   // P1–P4 short: clamped to 3.3 V
  CHECK(!e.update(p4_score(3.30f, false), 500));
  CHECK(!e.update(p4_score(std::nanf(""), false), 700));   // NaN never docks
  CHECK(!e.update(p4_score(std::nanf(""), false), 900));
}

static void test_charge_enable() {
  ChargeEnable c(1000);
  CHECK(!c.inhibit(0));                                    // default: charging allowed
  c.on_msg(false, 100);
  CHECK(c.inhibit(100) && c.inhibit(1099));                // dock_node asks before reversing
  c.on_msg(false, 1050);                                   // refreshed every 50 ms keeps it
  CHECK(c.inhibit(2049));
  CHECK(!c.inhibit(2050));                                 // dock_node stopped sending: charging again
  CHECK(!c.inhibit(2100));
  c.on_msg(false, 3000);
  c.on_msg(true, 3050);                                    // back to IDLE
  CHECK(!c.inhibit(3060));
  c.on_msg(false, 4000);
  c.reset();                                               // micro-ROS agent lost
  CHECK(!c.inhibit(4010));
  c.on_msg(false, 4020);
  CHECK(c.inhibit(4030));
  ChargeEnable w(1000);                                    // millis() wrap-around
  w.on_msg(false, 0xFFFFFF00u);
  CHECK(w.inhibit(0x00000100u));
  CHECK(!w.inhibit(0x00000300u));
  CHECK(!w.inhibit(0xFFFFFF10u));                          // an expired false stays expired
}

static void test_cliff_retry() {
  CliffRetry r(2000, 2);
  bool dead[4] = {false, true, false, true};
  CHECK(r.next(dead, true, 1999) == -1);        // not before the first interval
  CHECK(r.next(dead, false, 2000) == -1);       // never while the motors may run
  CHECK(r.next(dead, true, 2000) == 1);
  CHECK(r.next(dead, true, 3999) == -1);
  CHECK(r.next(dead, true, 4000) == 3);         // round robin
  CHECK(r.next(dead, true, 6000) == 1);
  CHECK(r.next(dead, true, 8000) == 3);
  CHECK(r.next(dead, true, 10000) == -1);       // max_tries reached for both
  bool none[4] = {false, false, false, false};
  CliffRetry s(2000, 5);
  CHECK(s.next(none, true, 5000) == -1);
}

static void test_drive_stops() {
  const int32_t c0[2] = {0, 0};
  DriveController d;
  DriveOutput o = d.step(c0, 0.01f, 0, false, false);
  CHECK(!o.enable && o.duty[0] == 0 && o.duty[1] == 0);   // no command yet

  d.set_target(5, 5, 0.7f, 100);
  o = d.step(c0, 0.01f, 100, false, false);
  CHECK(o.enable && o.duty[0] > 0 && o.duty[1] > 0 && o.duty[0] <= 0.7f);
  CHECK(!d.step(c0, 0.01f, 100, true, false).enable);    // E-stop
  CHECK(!d.step(c0, 0.01f, 100, false, true).enable);    // driver fault
  CHECK(d.step(c0, 0.01f, 400, false, false).enable);    // 300 ms is still fresh
  CHECK(!d.step(c0, 0.01f, 401, false, false).enable);   // timeout

  const float nan = std::nanf(""), inf = INFINITY;
  const float bad[][3] = {{nan, 1, 0.5f}, {1, nan, 0.5f}, {1, 1, nan}, {inf, 1, 0.5f}, {1, 1, inf}, {1, 1, -0.5f}, {1, 1, 0}};
  for (const auto &b : bad) {
    DriveController e;
    e.set_target(b[0], b[1], b[2], 0);
    const DriveOutput r = e.step(c0, 0.01f, 0, false, false);
    CHECK(!r.enable && r.duty[0] == 0 && r.duty[1] == 0);
  }

  DriveController f;
  f.set_target(100, -100, 3.0f, 0);                      // cap > 1 clamps to 1
  o = f.step(c0, 0.01f, 0, false, false);
  CHECK(o.enable && o.duty[0] == 1.0f && o.duty[1] == -1.0f);
}

static void test_drive_velocity_and_wrap() {
  DriveController d;
  int32_t c[2] = {1000, -1000};
  d.step(c, 0.01f, 0, false, false);
  CHECK(d.velocity_rad_s(0) == 0 && d.velocity_rad_s(1) == 0);   // first step has no delta
  c[0] += 65; c[1] -= 65;
  d.step(c, 0.01f, 10, false, false);
  const double w = 65 * 6.283185307179586 / 4480.0 / 0.01;
  CHECK_NEAR(d.velocity_rad_s(0), w, 1e-3);
  CHECK_NEAR(d.velocity_rad_s(1), -w, 1e-3);
  CHECK_NEAR(d.position_rad(0), 1065 * 6.283185307179586 / 4480.0, 1e-5);

  DriveController e;
  int32_t k[2] = {INT32_MAX - 10, INT32_MIN + 10};
  e.step(k, 0.01f, 0, false, false);
  k[0] = INT32_MIN + 10; k[1] = INT32_MAX - 10;         // +21 / -21 counts across the wrap
  e.step(k, 0.01f, 10, false, false);
  CHECK_NEAR(e.velocity_rad_s(0), 21 * 6.283185307179586 / 4480.0 / 0.01, 1e-3);
  CHECK_NEAR(e.velocity_rad_s(1), -21 * 6.283185307179586 / 4480.0 / 0.01, 1e-3);

  e.step(k, 0.0f, 20, false, false);                    // dt = 0 must not divide by zero
  CHECK(e.velocity_rad_s(0) == 0);
}

static void test_drive_integrator_bounds() {
  DriveConfig cfg;
  DriveController d(cfg);
  const int32_t c[2] = {0, 0};
  d.set_target(10, -10, 1.0f, 0);
  d.step(c, 0.01f, 0, false, false);
  const DriveOutput big = d.step(c, 1.0f, 0, false, false);   // late tick: integrator step capped
  const float max_one_step = 10 * cfg.ki * (cfg.max_integ_dt_s + 0.01f);
  CHECK(big.duty[0] <= 10 / cfg.ff_rad_s_at_full_duty + cfg.kp * 10 + max_one_step + 1e-5f);
  DriveOutput o;
  for (int i = 0; i < 1000; i++) o = d.step(c, 0.01f, 0, false, false);   // stalled wheel, 10 s
  CHECK(o.duty[0] <= 1.0f && o.duty[1] >= -1.0f);
  d.set_target(0.5f, -0.5f, 1.0f, 0);
  o = d.step(c, 0.01f, 0, false, false);
  CHECK(o.duty[0] <= 0.5f / cfg.ff_rad_s_at_full_duty + cfg.kp * 0.5f + cfg.max_i + 1e-4f);

  DriveController e(cfg);                                // a stop clears the integrators
  e.set_target(10, 10, 1.0f, 0);
  for (int i = 0; i < 100; i++) e.step(c, 0.01f, 0, false, false);
  e.step(c, 0.01f, 0, true, false);
  e.set_target(1, 1, 1.0f, 0);
  o = e.step(c, 0.01f, 0, false, false);
  CHECK_NEAR(o.duty[0], 1 / cfg.ff_rad_s_at_full_duty + cfg.kp * 1 + 1 * cfg.ki * 0.01f, 1e-5);
}

// First-order wheel model: the loop must converge on the target without sign errors.
static void test_drive_closed_loop() {
  DriveController d;
  double pos[2] = {0, 0}, vel[2] = {0, 0};
  const float target[2] = {5.0f, -3.0f};
  const double tau = 0.15, dt = 0.01, k_rad_s_per_duty = 13.5;  // model 10 % off the feed-forward
  uint32_t t = 0;
  for (int n = 0; n < 300; n++, t += 10) {
    d.set_target(target[0], target[1], 0.714f, t);
    const int32_t counts[2] = {(int32_t)std::lround(pos[0] * 4480 / 6.283185307179586),
                               (int32_t)std::lround(pos[1] * 4480 / 6.283185307179586)};
    const DriveOutput o = d.step(counts, (float)dt, t, false, false);
    for (int i = 0; i < 2; i++) {
      vel[i] += (o.duty[i] * k_rad_s_per_duty - vel[i]) * dt / tau;
      pos[i] += vel[i] * dt;
    }
  }
  CHECK_NEAR(vel[0], 5.0, 0.15);
  CHECK_NEAR(vel[1], -3.0, 0.15);
}

static void test_pwm_and_clamp() {
  PwmCommand p = duty_to_pwm(0.5f, 4095);
  CHECK(!p.reverse && p.level == 2048);
  p = duty_to_pwm(-1.0f, 4095);
  CHECK(p.reverse && p.level == 4095);
  p = duty_to_pwm(1.5f, 4095);
  CHECK(!p.reverse && p.level == 4095);
  p = duty_to_pwm(std::nanf(""), 4095);
  CHECK(!p.reverse && p.level == 0);
  p = duty_to_pwm(-0.0f, 4095);
  CHECK(!p.reverse && p.level == 0);
}

int main() {
  struct { const char *name; void (*fn)(); } tests[] = {
      {"daly request bytes", test_request_bytes},
      {"daly 0x90 voltage/current/SoC", test_soc_frame},
      {"daly 0x92/0x93/0x94/0x98", test_other_frames},
      {"daly 0x95 cell voltages", test_cell_frames},
      {"daly rejects bad frames", test_rejects_bad_frames},
      {"daly resync after noise/truncation", test_resync},
      {"daly alarms → health", test_health},
      {"battery monitor fields + freshness", test_battery_monitor_fields},
      {"battery monitor fail-safe + stale", test_battery_monitor_fail_safe},
      {"dock sense debounce", test_dock_debounce},
      {"P4 dock ID window", test_p4_window},
      {"charge_enable lease", test_charge_enable},
      {"cliff re-init policy", test_cliff_retry},
      {"cliff channel fail-safe", test_cliff_channel},
      {"drive stop conditions", test_drive_stops},
      {"drive velocity + encoder wrap", test_drive_velocity_and_wrap},
      {"drive integrator bounds", test_drive_integrator_bounds},
      {"drive closed loop converges", test_drive_closed_loop},
      {"pwm mapping", test_pwm_and_clamp},
  };
  for (const auto &t : tests) {
    const int before = g_fail;
    t.fn();
    std::printf("%s %s\n", g_fail == before ? "ok  " : "FAIL", t.name);
  }
  std::printf("%d checks, %d failed, %zu tests\n", g_checks, g_fail, sizeof(tests) / sizeof(tests[0]));
  return g_fail == 0 ? 0 : 1;
}
