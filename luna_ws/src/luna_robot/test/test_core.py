"""Unit tests for the ROS-free core. Run from luna_ws/src/luna_robot:  python3 -m unittest discover -s test"""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from luna_robot.core import kinematics as K
from luna_robot.core.battery import decide, soc_from_voltage
from luna_robot.core.dock import Dock, Obs
from luna_robot.core.face_mood import FACE_EVENTS, MoodMapper
from luna_robot.core.mock_plant import DockPlant, PlantConfig
from luna_robot.core.safety import gate


class Kinematics(unittest.TestCase):
    def test_wheel_geometry(self):
        self.assertAlmostEqual(K.WHEEL_RADIUS * 2, 0.080)          # Pololu 80 × 10 mm
        self.assertAlmostEqual(K.WHEEL_SEPARATION, 0.300)

    def test_straight_and_turn_round_trip(self):
        for v, w in [(0.2, 0.0), (0.0, 0.8), (0.15, -0.5), (-0.1, 0.3)]:
            l, r = K.twist_to_wheels(v, w)
            v2, w2 = K.wheels_to_twist(l, r)
            self.assertAlmostEqual(v, v2, places=9)
            self.assertAlmostEqual(w, w2, places=9)

    def test_turn_in_place_spins_the_wheels_opposite(self):
        l, r = K.twist_to_wheels(0.0, 1.0)
        self.assertAlmostEqual(l, -r)
        self.assertAlmostEqual(r * K.WHEEL_RADIUS, 1.0 * K.WHEEL_SEPARATION / 2)

    def test_saturation_keeps_turn_ratio(self):
        lim = K.DriveLimits(max_linear=10, max_angular=10, max_wheel_rad_s=5)
        l, r = K.twist_to_wheels(2.0, 1.0, lim)
        self.assertLessEqual(max(abs(l), abs(r)), 5 + 1e-9)
        l0, r0 = (2.0 - 1.0 * K.WHEEL_SEPARATION / 2 * lim.turn_slip), (2.0 + 1.0 * K.WHEEL_SEPARATION / 2 * lim.turn_slip)
        self.assertAlmostEqual(l / r, l0 / r0, places=9)

    def test_command_limits(self):
        l, r = K.twist_to_wheels(5.0, 0.0)
        self.assertAlmostEqual(l * K.WHEEL_RADIUS, K.DriveLimits().max_linear, places=9)
        self.assertLess(K.DriveLimits().max_linear / K.WHEEL_RADIUS, K.DriveLimits().max_wheel_rad_s)   # reachable

    def test_pwm_cap_protects_12v_motor(self):
        self.assertAlmostEqual(K.pwm_cap(14.6), 12 / 14.6)         # LiFePO4 4S on the charger
        self.assertEqual(K.pwm_cap(11.0), 1.0)
        self.assertEqual(K.pwm_cap(0), 0.0)
        self.assertEqual(K.pwm_cap(float('nan')), 0.0)

    def test_odometry_straight_line(self):
        o = K.Odometry()
        o.update(0, 0)
        turns = 1.0 / (2 * math.pi * K.WHEEL_RADIUS)            # 1 m of wheel travel
        x, y, yaw = o.update(turns * 2 * math.pi, turns * 2 * math.pi)
        self.assertAlmostEqual(x, 1.0, places=6)
        self.assertAlmostEqual(y, 0.0, places=6)
        self.assertAlmostEqual(yaw, 0.0, places=6)

    def test_odometry_takes_the_gyro_heading_when_given(self):
        o = K.Odometry()
        o.update(0, 0)
        quarter = math.pi / 2 * K.WHEEL_SEPARATION / 2 / K.WHEEL_RADIUS       # wheel angle for a 90° turn in place
        _, _, yaw = o.update(-quarter, quarter, dyaw_gyro=math.radians(80))    # the wheels slipped: the gyro is right
        self.assertAlmostEqual(math.degrees(yaw), 80.0)
        _, _, yaw = o.update(-2 * quarter, 2 * quarter)                        # no gyro: wheels again
        self.assertAlmostEqual(math.degrees(yaw), 170.0)

    def test_gyro_bias_is_learnt_only_standing_still(self):
        b = K.GyroBias(tau_s=1.0)
        for _ in range(500):
            b.update(0.01, 0.0, 0.0, 0.02)                     # standing: the gyro reads its bias
        self.assertAlmostEqual(b.bias, 0.01, places=4)
        self.assertAlmostEqual(b.update(0.51, 3.0, -3.0, 0.02), 0.5, places=4)   # turning: corrected, not learnt
        self.assertAlmostEqual(b.bias, 0.01, places=4)

    def test_encoder_counts(self):
        self.assertAlmostEqual(K.counts_to_rad(4480), 2 * math.pi)       # 64 CPR × 70:1


class Safety(unittest.TestCase):
    floor = [0.060] * 4

    def test_estop_and_timeout(self):
        self.assertEqual(gate(0.2, 0.5, estop=True, bumpers=0, cliff=self.floor, cmd_age_s=0).reason, 'estop')
        self.assertEqual(gate(0.2, 0.5, estop=False, bumpers=0, cliff=self.floor, cmd_age_s=1).linear, 0.0)

    def test_front_bumper_blocks_forward_only(self):
        v = gate(0.2, 0, estop=False, bumpers=0b01, cliff=self.floor, cmd_age_s=0)
        self.assertEqual((v.linear, v.reason), (0.0, 'front_blocked'))
        v = gate(-0.2, 0, estop=False, bumpers=0b01, cliff=self.floor, cmd_age_s=0)
        self.assertEqual(v.linear, -0.2)

    def test_cliff(self):
        v = gate(0.2, 0, estop=False, bumpers=0, cliff=[0.3, 0.060, 0.060, 0.060], cmd_age_s=0)
        self.assertEqual(v.linear, 0.0)
        v = gate(0.2, 0.3, estop=False, bumpers=0, cliff=[0.3, 0.3, 0.060, 0.060], cmd_age_s=0)
        self.assertEqual((v.linear, v.angular, v.reason), (0.0, 0.0, 'lifted_or_cliff'))

    def test_invalid_reading_is_a_cliff(self):
        v = gate(0.2, 0, estop=False, bumpers=0, cliff=[0.0, 0.060, 0.060, 0.060], cmd_age_s=0)
        self.assertEqual(v.linear, 0.0)


class Battery(unittest.TestCase):
    def test_soc_curve(self):
        self.assertEqual(soc_from_voltage(14.6), 100.0)       # LiFePO4 4S, full on the charger
        self.assertEqual(soc_from_voltage(11.0), 0.0)
        self.assertAlmostEqual(soc_from_voltage(13.24), 50.0)  # 3.31 V/cell: the flat middle of the curve
        self.assertLess(soc_from_voltage(12.8), soc_from_voltage(13.0))

    def test_policy(self):
        self.assertEqual(decide(50, charging=False, docked=False, temp_c=25, seeking=False), ('run', 15))
        self.assertEqual(decide(20, charging=False, docked=False, temp_c=25, seeking=False)[0], 'seek_dock')
        self.assertEqual(decide(28, charging=False, docked=False, temp_c=25, seeking=True)[0], 'seek_dock')   # hysteresis
        self.assertEqual(decide(5, charging=False, docked=False, temp_c=25, seeking=False)[0], 'critical')
        self.assertEqual(decide(50, charging=True, docked=True, temp_c=25, seeking=False), ('charge', 10))
        self.assertEqual(decide(50, charging=False, docked=False, temp_c=55, seeking=False)[0], 'overtemp')


class Docking(unittest.TestCase):
    def run_to(self, d, obs, ticks=400, dt=0.05):
        out = []
        for _ in range(ticks):
            out.append(d.step(obs, dt))
        return out

    def test_happy_path(self):
        d = Dock()
        d.request('dock')
        self.assertEqual(d.state, 'SEEK')
        lin, ang = d.step(Obs(), 0.05)
        self.assertGreater(ang, 0)
        d.step(Obs(tag_visible=True, tag_offset=0.3), 0.05)
        self.assertEqual(d.state, 'ALIGN')
        _, ang = d.step(Obs(tag_visible=True, tag_offset=0.3), 0.05)
        self.assertLess(ang, 0)                                       # turn toward the tag
        d.step(Obs(tag_visible=True, tag_offset=0.0), 0.05)
        self.assertEqual(d.state, 'APPROACH')
        lin, _ = d.step(Obs(tag_visible=True, tag_distance=0.5), 0.05)
        self.assertAlmostEqual(lin, d.cfg.approach_speed)
        lin, _ = d.step(Obs(tag_visible=True, tag_distance=0.03), 0.05)
        self.assertAlmostEqual(lin, d.cfg.creep_speed)
        d.step(Obs(tag_visible=True, dock_sense=True), 0.05)
        self.assertEqual(d.state, 'CONTACT')
        self.run_to(d, Obs(dock_sense=True, charging=True), ticks=30)
        self.assertEqual(d.state, 'CHARGING')
        self.run_to(d, Obs(dock_sense=True, charging=False), ticks=round(d.cfg.charge_loss_s / 0.05) + 1)
        self.assertEqual(d.state, 'DOCKED')

    def test_retry_then_fault(self):
        d = Dock()
        d.request('dock')
        for _ in range(d.cfg.max_retries + 1):
            d.state = 'APPROACH'
            d.step(Obs(tag_visible=True, front_bumper=True), 0.05)
        self.assertEqual(d.state, 'FAULT')

    def test_no_charge_after_contact_backs_off(self):
        d = Dock()
        d.state = 'CONTACT'
        self.run_to(d, Obs(dock_sense=True, charging=False), ticks=200)
        self.assertIn(d.state, ('BACKOFF', 'SEEK', 'ALIGN', 'FAULT'))

    def test_contact_bounce_does_not_back_off(self):
        # DOCK_SENSE is one undebounced ADC read per MCU tick: a lone false sample (charge current gone
        # with it) is contact bounce, not a lost dock.
        for state in ('CONTACT', 'CHARGING'):
            with self.subTest(state=state):
                d = Dock()
                d.state = state
                self.assertEqual(d.step(Obs(), 0.05), (0.0, 0.0))
                self.assertEqual((d.state, d.retries), (state, 0))
                d.step(Obs(dock_sense=True, charging=True), 0.05)
                self.assertEqual((d.state, d.retries), (state, 0))

    def test_any_contact_sample_restarts_the_loss_window(self):
        d = Dock()
        d.state = 'CHARGING'
        just_under = round(d.cfg.sense_loss_s / 0.05) - 1
        for _ in range(10):
            self.run_to(d, Obs(), ticks=just_under)
            self.run_to(d, Obs(dock_sense=True, charging=True), ticks=1)
        self.assertEqual((d.state, d.retries), ('CHARGING', 0))

    def test_sustained_contact_loss_backs_off(self):
        # Contact lost before charging started: the dock attempt failed.
        d = Dock()
        d.state = 'CONTACT'
        window = round(d.cfg.sense_loss_s / 0.05)
        self.run_to(d, Obs(), ticks=window - 1)
        self.assertEqual(d.state, 'CONTACT')                          # still inside the debounce window
        self.run_to(d, Obs(), ticks=2)
        self.assertEqual((d.state, d.retries), ('BACKOFF', 1))

    def test_charge_current_lagging_a_bounce_keeps_charging(self):
        # Charge current comes from the BMS at 5 Hz, so after a bounce it can return later than DOCK_SENSE.
        d = Dock()
        d.state = 'CHARGING'
        d.step(Obs(), 0.05)                                            # bounce: contact and current gone
        self.run_to(d, Obs(dock_sense=True, charging=False), ticks=4)  # contact back, BMS not yet
        self.run_to(d, Obs(dock_sense=True, charging=True), ticks=1)
        self.assertEqual((d.state, d.retries), ('CHARGING', 0))

    def test_charge_stop_parks_after_the_loss_window(self):
        d = Dock()
        d.state = 'CHARGING'
        window = round(d.cfg.charge_loss_s / 0.05)
        self.run_to(d, Obs(dock_sense=True, charging=False), ticks=window - 1)
        self.assertEqual(d.state, 'CHARGING')
        self.run_to(d, Obs(dock_sense=True, charging=False), ticks=2)
        self.assertEqual((d.state, d.retries), ('DOCKED', 0))

    def test_docked_resumes_charging_when_current_returns(self):
        # The charger paused, or the pack drained below the charger's restart point.
        d = Dock()
        d.state = 'DOCKED'
        self.run_to(d, Obs(dock_sense=True, charging=False), ticks=100)
        self.assertEqual(d.state, 'DOCKED')
        self.assertEqual(d.step(Obs(dock_sense=True, charging=True), 0.05), (0.0, 0.0))
        self.assertEqual((d.state, d.retries), ('CHARGING', 0))

    def test_docked_contact_bounce_stays_docked(self):
        d = Dock()
        d.state = 'DOCKED'
        just_under = round(d.cfg.sense_loss_s / 0.05) - 1
        for _ in range(10):
            self.run_to(d, Obs(), ticks=just_under)
            self.run_to(d, Obs(dock_sense=True), ticks=1)
        self.assertEqual((d.state, d.retries), ('DOCKED', 0))

    def test_lifted_off_the_dock_goes_idle(self):
        # Not a failed dock: Luna may be used before it is full. IDLE lets dock_node's battery policy send
        # it back to the dock when the pack needs it.
        for state in ('CHARGING', 'DOCKED'):
            with self.subTest(state=state):
                d = Dock()
                d.state = state
                window = round(d.cfg.sense_loss_s / 0.05)
                self.run_to(d, Obs(), ticks=window - 1)
                self.assertEqual(d.state, state)                      # still inside the debounce window
                out = self.run_to(d, Obs(), ticks=2)
                self.assertEqual((d.state, d.retries), ('IDLE', 0))
                self.assertEqual(set(out), {(0.0, 0.0)})              # no reversing off a dock it left

    def test_backing_off_the_dock_releases_the_contacts_first(self):
        # Elektrik R01 §5: charge_enable=false makes the MCU raise DOCK_INHIBIT and the dock switch the contacts
        # off (~10 ms) before they slide apart, so they never break charge current. dock_node publishes
        # charge_enable after every step.
        cases = [('DOCKED', 'undock', Obs(dock_sense=True), 'IDLE'),
                 ('CHARGING', 'undock', Obs(dock_sense=True, charging=True), 'IDLE'),
                 ('CONTACT', None, Obs(dock_sense=True), 'SEEK')]       # seated, no charge current: retry
        for start, request, obs, end in cases:
            with self.subTest(start=start):
                d = Dock()
                d.state = start
                self.assertTrue(d.charge_enable)
                if request:
                    d.request(request)
                while d.state == start:
                    d.step(obs, 0.05)
                leaving = d.state
                self.assertIn(leaving, ('UNDOCK', 'BACKOFF'))
                self.assertFalse(d.charge_enable)
                lin, published = [], []
                while d.state == leaving:
                    lin.append(d.step(obs, 0.05)[0])
                    published.append(d.charge_enable)
                first = next(i for i, v in enumerate(lin) if v < 0)
                self.assertEqual(set(lin[:first]), {0.0})
                self.assertGreaterEqual(first * 0.05, 0.2 - 1e-9)     # ≥ 0.2 s from the first false to reversing
                self.assertLessEqual(first * 0.05, 0.3 + 1e-9)
                self.assertFalse(any(published[:-1]))                  # false for the whole manoeuvre
                self.assertAlmostEqual(sum(v < 0 for v in lin) * 0.05, d.cfg.backoff_time, delta=0.051)
                self.assertEqual(d.state, end)
                self.assertTrue(published[-1] and d.charge_enable)    # charging allowed again


class DockingLoop(unittest.TestCase):
    """Dock state machine closed around the mock MCU plant at the node rates (dock 20 Hz, MCU 50 Hz)."""

    def simulate(self, plant, dock, seconds, trace=None):
        states, ticks_per_dock = [], 0
        for i in range(int(seconds / 0.02)):
            if i * 0.02 >= ticks_per_dock * 0.05 - 1e-9:
                tag = plant.tag()
                obs = Obs(tag_visible=tag is not None, dock_sense=plant.docked, charging=plant.charging)
                if tag:
                    obs.tag_offset, obs.tag_distance = tag
                lin, ang = dock.step(obs, 0.05)
                plant.target = list(K.twist_to_wheels(lin, ang))
                plant.charge_enable = dock.charge_enable
                states.append(dock.state)
                if trace is not None:
                    trace.append((plant.x, plant.charging))
                ticks_per_dock += 1
            plant.step(0.02)
        return states

    def test_mock_dock_reaches_charging_and_stays(self):
        plant, dock = DockPlant(PlantConfig(dock_distance_m=1.2)), Dock()
        dock.request('dock')
        states = self.simulate(plant, dock, seconds=40)
        path = [s for i, s in enumerate(states) if i == 0 or s != states[i - 1]]
        self.assertEqual(path, ['ALIGN', 'APPROACH', 'CONTACT', 'CHARGING'])
        self.assertAlmostEqual(plant.x, 1.2, delta=0.01)
        soc = plant.soc
        self.assertEqual(set(self.simulate(plant, dock, seconds=10)), {'CHARGING'})   # contact holds
        self.assertGreater(plant.soc, soc)

    def test_mock_undock_stops_charging_before_moving(self):
        plant, dock = DockPlant(PlantConfig(dock_distance_m=1.2)), Dock()
        dock.request('dock')
        self.simulate(plant, dock, seconds=40)
        self.assertEqual((dock.state, plant.charging), ('CHARGING', True))
        x0, trace = plant.x, []
        dock.request('undock')
        self.simulate(plant, dock, seconds=5, trace=trace)
        moved = next(i for i, (x, _) in enumerate(trace) if x < x0 - 1e-4)
        self.assertGreaterEqual(moved * 0.05, 0.2 - 1e-9)             # held still ≥ 0.2 s ...
        self.assertFalse(any(charging for _, charging in trace))       # ... with the contacts already dead
        self.assertEqual(dock.state, 'IDLE')
        self.assertTrue(dock.charge_enable)
        self.assertLess(plant.x, x0 - 0.1)


class Mood(unittest.TestCase):
    def test_events_are_face_events(self):
        m = MoodMapper()
        ev = m.update(bumpers=1, dock_state='IDLE', battery_action='run', gaze=(0.5, 0))
        self.assertEqual([e['type'] for e in ev], ['tap', 'gaze'])
        ev = m.update(bumpers=1, dock_state='IDLE', battery_action='run', gaze=(0.5, 0))
        self.assertEqual(ev, [])                                     # held bumper does not repeat
        ev = m.update(bumpers=0, dock_state='APPROACH', battery_action='seek_dock', gaze=(0.5, 0))
        self.assertEqual([e['type'] for e in ev], ['low-battery'])
        ev = m.update(bumpers=0, dock_state='CONTACT', battery_action='seek_dock', gaze=(0.5, 0))
        self.assertEqual([e['type'] for e in ev], ['stroke'])
        ev = m.update(bumpers=0, dock_state='CHARGING', battery_action='charge', gaze=(0.0, 0))
        self.assertEqual([e['type'] for e in ev], ['sleep', 'battery-ok'])   # no gaze while asleep
        ev = m.update(bumpers=0, dock_state='UNDOCK', battery_action='run', gaze=None)
        self.assertEqual([e['type'] for e in ev], ['wake', 'gaze-clear'])
        for e in ev:
            self.assertIn(e['type'], FACE_EVENTS)

    def test_mute_is_a_face_state(self):
        m = MoodMapper()
        self.assertEqual(m.update(bumpers=0, dock_state='IDLE', battery_action='run', gaze=None, muted=True),
                         [{'type': 'muted'}])
        self.assertEqual(m.update(bumpers=0, dock_state='IDLE', battery_action='run', gaze=None, muted=True), [])
        self.assertEqual(m.update(bumpers=0, dock_state='IDLE', battery_action='run', gaze=None, muted=False),
                         [{'type': 'unmuted'}])

    def test_snapshot_brings_connecting_face_to_current_state(self):
        m = MoodMapper()
        for dock in ('CHARGING', 'DOCKED'):
            ev = m.snapshot(bumpers=0, dock_state=dock, battery_action='charge', gaze=(0.4, 0.1))
            self.assertEqual(ev, [{'type': 'sleep'}])                # asleep on the dock, gaze ignored
        ev = m.snapshot(bumpers=1, dock_state='APPROACH', battery_action='seek_dock', gaze=(0.3, -0.2))
        self.assertEqual(ev, [{'type': 'wake'}, {'type': 'gaze', 'x': 0.3, 'y': -0.2}, {'type': 'low-battery'}])
        ev = m.snapshot(bumpers=0, dock_state='IDLE', battery_action='run', gaze=None, muted=True, voice='speaking')
        self.assertEqual(ev, [{'type': 'wake'}, {'type': 'speaking'}, {'type': 'muted'}])
        ev = m.update(bumpers=0, dock_state='CHARGING', battery_action='charge', gaze=(0.0, 0))
        self.assertEqual(ev, [{'type': 'sleep'}])                    # snapshot left the live mapper untouched


if __name__ == '__main__':
    unittest.main()
