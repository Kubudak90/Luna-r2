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
#include "jbd_bms.h"

using namespace luna;

static int g_checks = 0, g_fail = 0;
#define CHECK(c)                                                              \
  do {                                                                        \
    ++g_checks;                                                               \
    if (!(c)) { ++g_fail; std::printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); } \
  } while (0)
#define CHECK_NEAR(a, b, eps) CHECK(std::fabs((double)(a) - (double)(b)) <= (eps))

using Bytes = std::vector<uint8_t>;

static Bytes frame(uint8_t status, const Bytes &data, uint8_t reg = JBD_REG_BASIC) {
  Bytes body = {status, (uint8_t)data.size()};
  body.insert(body.end(), data.begin(), data.end());
  const uint16_t chk = jbd_checksum(body.data(), body.size());
  Bytes f = {JBD_START, reg};
  f.insert(f.end(), body.begin(), body.end());
  f.push_back((uint8_t)(chk >> 8)); f.push_back((uint8_t)chk); f.push_back(JBD_END);
  return f;
}

static void put16(Bytes &d, size_t at, uint16_t v) { d[at] = (uint8_t)(v >> 8); d[at + 1] = (uint8_t)v; }

// Luna pack: 4S3P 26650, here 15.12 V, -2.35 A, 9.80 of 15.00 Ah, 3 NTCs.
static Bytes luna_data(int16_t centiamps = -235, uint16_t protection = 0) {
  Bytes d(23 + 6, 0);
  put16(d, 0, 1512); put16(d, 2, (uint16_t)centiamps); put16(d, 4, 980); put16(d, 6, 1500);
  put16(d, 8, 12); put16(d, 16, protection);
  d[18] = 0x10; d[19] = 65; d[20] = 0x03; d[21] = 4; d[22] = 3;
  put16(d, 23, 2731 + 253); put16(d, 25, 2731 + 268); put16(d, 27, 2731 - 50);   // 25.3, 26.8, -5.0 °C
  return d;
}

static int feed_all(JbdParser &p, const Bytes &b) {
  int frames = 0;
  for (uint8_t x : b) frames += p.feed(x) != JbdFrame::None ? 1 : 0;
  return frames;
}

static void test_request_bytes() {
  uint8_t req[7];
  CHECK(jbd_build_read_request(JBD_REG_BASIC, req) == 7);
  const uint8_t want[7] = {0xDD, 0xA5, 0x03, 0x00, 0xFF, 0xFD, 0x77};
  for (int i = 0; i < 7; i++) CHECK(req[i] == want[i]);
  jbd_build_read_request(JBD_REG_CELLS, req);
  const uint8_t cells[7] = {0xDD, 0xA5, 0x04, 0x00, 0xFF, 0xFC, 0x77};
  for (int i = 0; i < 7; i++) CHECK(req[i] == cells[i]);
}

static Bytes cell_data(std::initializer_list<uint16_t> mv) {
  Bytes d;
  for (uint16_t v : mv) { d.push_back((uint8_t)(v >> 8)); d.push_back((uint8_t)v); }
  return d;
}

static void test_cell_frames() {
  JbdParser p;
  JbdFrame got = JbdFrame::None;
  for (uint8_t x : frame(0x00, cell_data({3780, 3781, 3779, 4210}), JBD_REG_CELLS)) {
    const JbdFrame f = p.feed(x);
    if (f != JbdFrame::None) got = f;
  }
  CHECK(got == JbdFrame::Cells);
  CHECK(p.cells().count == 4);
  CHECK_NEAR(p.cells().v[0], 3.780, 1e-6);
  CHECK_NEAR(p.cells().v[3], 4.210, 1e-6);
  CHECK(p.info().voltage_v == 0);                              // basic info untouched

  JbdParser q;
  Bytes odd = cell_data({3780, 3781}); odd.pop_back();
  CHECK(feed_all(q, frame(0x00, odd, JBD_REG_CELLS)) == 0);
  CHECK(feed_all(q, frame(0x00, Bytes(), JBD_REG_CELLS)) == 0);
  CHECK(q.frames_bad() == 2);
  Bytes all32(64, 0x0F);
  CHECK(feed_all(q, frame(0x00, all32, JBD_REG_CELLS)) == 1);
  CHECK(q.cells().count == 32);

  BatteryMonitor m(3000);                                     // cells alone never make it present
  m.on_cells(p.cells(), 0);
  CHECK(!m.present(0) && m.fields(0).cell_count == 0);
  JbdParser b;
  CHECK(feed_all(b, frame(0x00, luna_data())) == 1);
  m.on_frame(b.info(), 100);
  CHECK(m.fields(100).cell_count == 4);
  CHECK_NEAR(m.fields(100).cell_v[1], 3.781, 1e-6);
  CHECK(m.fields(3001).present && m.fields(3001).cell_count == 0);   // cells stale, basic fresh
  m.on_cells(p.cells(), 3001);
  CHECK(m.fields(3001).cell_count == 4);
  CHECK(m.fields(3101).cell_count == 0 && !m.fields(3101).present);  // basic stale → absent, no cells
}

// Independent vector (15S pack), checksum 0xFBFF verified by hand — guards against the frame()
// helper and the parser sharing the same checksum mistake.
static void test_reference_frame() {
  const Bytes f = {0xDD, 0x03, 0x00, 0x1B, 0x17, 0x00, 0x00, 0x00, 0x02, 0xD0, 0x03, 0xE8, 0x00, 0x00,
                   0x20, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x48, 0x03, 0x0F, 0x02, 0x0B,
                   0x76, 0x0B, 0x82, 0xFB, 0xFF, 0x77};
  JbdParser p;
  CHECK(feed_all(p, f) == 1);
  const JbdBasicInfo &i = p.info();
  CHECK_NEAR(i.voltage_v, 58.88, 1e-4);
  CHECK_NEAR(i.current_a, 0.0, 1e-6);
  CHECK_NEAR(i.remaining_ah, 7.20, 1e-4);
  CHECK_NEAR(i.nominal_ah, 10.00, 1e-4);
  CHECK(i.rsoc_pct == 72 && i.cells == 15 && i.fet == 3 && i.ntc_count == 2);
  CHECK_NEAR(i.ntc_c[0], 20.3, 1e-4);
  CHECK_NEAR(i.ntc_c[1], 21.5, 1e-4);
  const BatteryFields b = battery_fields_from(i);
  CHECK(b.present);
  CHECK_NEAR(b.percentage, 0.72, 1e-6);
  CHECK_NEAR(b.temperature, 21.5, 1e-4);
  CHECK(b.status == BATT_STATUS_NOT_CHARGING);
  CHECK(b.health == BATT_HEALTH_GOOD);
}

static void test_luna_frame_fields() {
  JbdParser p;
  CHECK(feed_all(p, frame(0x00, luna_data())) == 1);
  const BatteryFields b = battery_fields_from(p.info());
  CHECK_NEAR(b.voltage, 15.12, 1e-4);
  CHECK_NEAR(b.current, -2.35, 1e-4);
  CHECK_NEAR(b.charge, 9.80, 1e-4);
  CHECK_NEAR(b.capacity, 15.00, 1e-4);
  CHECK_NEAR(b.design_capacity, 15.00, 1e-4);
  CHECK_NEAR(b.percentage, 9.80 / 15.00, 1e-5);
  CHECK_NEAR(b.temperature, 26.8, 1e-4);               // hottest NTC
  CHECK_NEAR(p.info().ntc_c[2], -5.0, 1e-4);           // below 0 °C decodes signed
  CHECK(b.status == BATT_STATUS_DISCHARGING);
  CHECK(b.technology == BATT_TECH_LION);
  CHECK(p.info().cycles == 12 && p.info().cells == 4);

  JbdParser c;
  CHECK(feed_all(c, frame(0x00, luna_data(150))) == 1);
  CHECK(battery_fields_from(c.info()).status == BATT_STATUS_CHARGING);
}

static void test_rejects_bad_frames() {
  JbdParser p;
  Bytes f = frame(0x00, luna_data());
  Bytes bad_chk = f; bad_chk[bad_chk.size() - 2] ^= 0x01;
  CHECK(feed_all(p, bad_chk) == 0);
  CHECK(p.frames_ok() == 0 && p.frames_bad() == 1);

  Bytes bad_end = f; bad_end.back() = 0x78;
  CHECK(feed_all(p, bad_end) == 0);

  Bytes bad_data = f; bad_data[6] ^= 0x10;               // payload flipped, checksum stale
  CHECK(feed_all(p, bad_data) == 0);

  CHECK(feed_all(p, frame(0x80, luna_data())) == 0);     // BMS error status

  CHECK(feed_all(p, frame(0x00, luna_data(), 0x05)) == 0);   // register we do not request

  Bytes short_ntc = luna_data(); short_ntc.resize(23 + 4); // claims 3 NTCs, carries 2
  CHECK(feed_all(p, frame(0x00, short_ntc)) == 0);

  CHECK(feed_all(p, frame(0x00, Bytes(20, 0))) == 0);    // shorter than the fixed part

  Bytes too_long = {0xDD, 0x03, 0x00, (uint8_t)(JBD_MAX_DATA + 1)};
  CHECK(feed_all(p, too_long) == 0);
  CHECK(p.frames_ok() == 0);
  CHECK(feed_all(p, f) == 1);                            // and it still recovers
}

static void test_resync() {
  JbdParser p;
  const Bytes f = frame(0x00, luna_data());
  Bytes stream = {0x00, 0x77, 0xDD, 0x99, 0xDD, 0xDD};   // noise, including stray start bytes
  stream.insert(stream.end(), f.begin(), f.end());
  stream.insert(stream.end(), f.begin(), f.end());
  CHECK(feed_all(p, stream) == 2);

  // Truncated reply: with reset() before the next request the next frame is not lost.
  JbdParser q;
  CHECK(feed_all(q, Bytes(f.begin(), f.begin() + 12)) == 0);
  q.reset();
  CHECK(feed_all(q, f) == 1);

  // Without reset() the truncated frame may eat the next one, but the parser recovers after it.
  JbdParser r;
  Bytes s(f.begin(), f.begin() + 12);
  s.insert(s.end(), f.begin(), f.end());
  s.insert(s.end(), f.begin(), f.end());
  CHECK(feed_all(r, s) >= 1);
}

static void test_longer_and_wider_frames() {
  Bytes d = luna_data();
  d.insert(d.end(), {0x00, 0x00, 0x05, 0xDC, 0x03, 0xD4});   // newer boards append fields
  JbdParser p;
  CHECK(feed_all(p, frame(0x00, d)) == 1);
  CHECK_NEAR(p.info().voltage_v, 15.12, 1e-4);

  Bytes many(23 + 2 * 10, 0);                                 // 10 NTCs: keep the first 8
  many[22] = 10;
  for (int i = 0; i < 10; i++) put16(many, 23 + 2 * i, (uint16_t)(2731 + 100 * i));
  JbdParser q;
  CHECK(feed_all(q, frame(0x00, many)) == 1);
  CHECK(q.info().ntc_count == JBD_MAX_NTC);
  CHECK_NEAR(battery_fields_from(q.info()).temperature, 70.0, 1e-4);

  Bytes none = luna_data(); none[22] = 0; none.resize(23); // no NTC → unknown temperature
  JbdParser r;
  CHECK(feed_all(r, frame(0x00, none)) == 1);
  CHECK(std::isnan(battery_fields_from(r.info()).temperature));
}

static void test_soc_and_health() {
  JbdBasicInfo i;
  i.nominal_ah = 0; i.rsoc_pct = 40;
  CHECK_NEAR(battery_fields_from(i).percentage, 0.40, 1e-6);   // no nominal → BMS RSOC
  i.rsoc_pct = 0xFF;
  CHECK(std::isnan(battery_fields_from(i).percentage));
  i.nominal_ah = 15; i.remaining_ah = 15.2f; i.current_a = 0.01f;
  CHECK_NEAR(battery_fields_from(i).percentage, 1.0, 1e-6);    // clamped
  CHECK(battery_fields_from(i).status == BATT_STATUS_FULL);

  struct { uint16_t p; uint8_t h; } cases[] = {
      {0, BATT_HEALTH_GOOD},
      {JBD_P_CHG_OT, BATT_HEALTH_OVERHEAT}, {JBD_P_DSG_OT, BATT_HEALTH_OVERHEAT},
      {JBD_P_CHG_UT, BATT_HEALTH_COLD}, {JBD_P_DSG_UT, BATT_HEALTH_COLD},
      {JBD_P_CELL_OV, BATT_HEALTH_OVERVOLTAGE}, {JBD_P_PACK_OV, BATT_HEALTH_OVERVOLTAGE},
      {JBD_P_CELL_UV, BATT_HEALTH_DEAD}, {JBD_P_PACK_UV, BATT_HEALTH_DEAD},
      {JBD_P_SHORT, BATT_HEALTH_UNSPEC_FAILURE}, {JBD_P_DSG_OC, BATT_HEALTH_UNSPEC_FAILURE},
      {JBD_P_AFE_ERROR, BATT_HEALTH_UNSPEC_FAILURE}, {JBD_P_SOFT_LOCK, BATT_HEALTH_UNSPEC_FAILURE},
      {JBD_P_DSG_OT | JBD_P_CELL_OV, BATT_HEALTH_OVERHEAT},
  };
  for (const auto &c : cases) {
    JbdParser p;
    CHECK(feed_all(p, frame(0x00, luna_data(-235, c.p))) == 1);
    CHECK(p.info().protection == c.p);
    CHECK(battery_fields_from(p.info()).health == c.h);
  }
}

static void test_battery_monitor_fail_safe() {
  const BatteryFields a = battery_fields_absent();
  CHECK(!a.present);
  CHECK(a.voltage == 0.0f && !std::isnan(a.voltage));   // AGX: pwm_cap(0 V) = 0, pwm_cap(NaN) = NaN
  CHECK(a.current == 0.0f);
  CHECK(std::isnan(a.percentage) && std::isnan(a.temperature));

  BatteryMonitor m(3000);
  CHECK(!m.present(0) && !m.fields(0).present && m.fields(0).voltage == 0.0f);
  JbdParser p;
  CHECK(feed_all(p, frame(0x00, luna_data())) == 1);
  m.on_frame(p.info(), 1000);
  CHECK(m.present(1000) && m.fields(4000).present);
  CHECK_NEAR(m.fields(4000).voltage, 15.12, 1e-4);
  CHECK(!m.fields(4001).present);
  CHECK(m.fields(4001).voltage == 0.0f);

  BatteryMonitor w(3000);                       // millis() wrap-around
  w.on_frame(p.info(), 0xFFFFFF00u);
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
  const double w = 65 * 6.283185307179586 / 6533.0 / 0.01;
  CHECK_NEAR(d.velocity_rad_s(0), w, 1e-3);
  CHECK_NEAR(d.velocity_rad_s(1), -w, 1e-3);
  CHECK_NEAR(d.position_rad(0), 1065 * 6.283185307179586 / 6533.0, 1e-5);

  DriveController e;
  int32_t k[2] = {INT32_MAX - 10, INT32_MIN + 10};
  e.step(k, 0.01f, 0, false, false);
  k[0] = INT32_MIN + 10; k[1] = INT32_MAX - 10;         // +21 / -21 counts across the wrap
  e.step(k, 0.01f, 10, false, false);
  CHECK_NEAR(e.velocity_rad_s(0), 21 * 6.283185307179586 / 6533.0 / 0.01, 1e-3);
  CHECK_NEAR(e.velocity_rad_s(1), -21 * 6.283185307179586 / 6533.0 / 0.01, 1e-3);

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

// First-order track model: the loop must converge on the target without sign errors.
static void test_drive_closed_loop() {
  DriveController d;
  double pos[2] = {0, 0}, vel[2] = {0, 0};
  const float target[2] = {5.0f, -3.0f};
  const double tau = 0.15, dt = 0.01, k_rad_s_per_duty = 9.0;   // model 10 % off the feed-forward
  uint32_t t = 0;
  for (int n = 0; n < 300; n++, t += 10) {
    d.set_target(target[0], target[1], 0.714f, t);
    const int32_t counts[2] = {(int32_t)std::lround(pos[0] * 6533 / 6.283185307179586),
                               (int32_t)std::lround(pos[1] * 6533 / 6.283185307179586)};
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
  CHECK(clamp_finite(2000, 1100, 1900, 1500) == 1900);
  CHECK(clamp_finite(1000, 1100, 1900, 1500) == 1100);
  CHECK(clamp_finite(1234, 1100, 1900, 1500) == 1234);
  CHECK(clamp_finite(std::nanf(""), 1100, 1900, 1500) == 1500);
  CHECK(clamp_finite(-INFINITY, 1100, 1900, 1600) == 1600);
}

int main() {
  struct { const char *name; void (*fn)(); } tests[] = {
      {"jbd request bytes", test_request_bytes},
      {"jbd reference frame", test_reference_frame},
      {"jbd luna frame → BatteryState", test_luna_frame_fields},
      {"jbd rejects bad frames", test_rejects_bad_frames},
      {"jbd resync after noise/truncation", test_resync},
      {"jbd longer frames, NTC count", test_longer_and_wider_frames},
      {"jbd SoC and health flags", test_soc_and_health},
      {"battery monitor fail-safe + stale", test_battery_monitor_fail_safe},
      {"jbd cell voltages (0x04)", test_cell_frames},
      {"dock sense debounce", test_dock_debounce},
      {"P4 dock ID window", test_p4_window},
      {"charge_enable lease", test_charge_enable},
      {"cliff re-init policy", test_cliff_retry},
      {"cliff channel fail-safe", test_cliff_channel},
      {"drive stop conditions", test_drive_stops},
      {"drive velocity + encoder wrap", test_drive_velocity_and_wrap},
      {"drive integrator bounds", test_drive_integrator_bounds},
      {"drive closed loop converges", test_drive_closed_loop},
      {"pwm mapping + clamp_finite", test_pwm_and_clamp},
  };
  for (const auto &t : tests) {
    const int before = g_fail;
    t.fn();
    std::printf("%s %s\n", g_fail == before ? "ok  " : "FAIL", t.name);
  }
  std::printf("%d checks, %d failed, %zu tests\n", g_checks, g_fail, sizeof(tests) / sizeof(tests[0]));
  return g_fail == 0 ? 0 : 1;
}
