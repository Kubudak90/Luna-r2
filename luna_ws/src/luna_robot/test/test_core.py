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
from luna_robot.core.head import Head
from luna_robot.core.mock_plant import DockPlant, PlantConfig
from luna_robot.core.safety import gate


class Kinematics(unittest.TestCase):
    def test_sprocket_radius_matches_16t_24mm_pitch(self):
        self.assertAlmostEqual(K.SPROCKET_RADIUS * 2, 0.1230, places=3)

    def test_straight_and_turn_round_trip(self):
        for v, w in [(0.2, 0.0), (0.0, 0.8), (0.15, -0.5), (-0.1, 0.3)]:
            l, r = K.twist_to_tracks(v, w)
            v2, w2 = K.tracks_to_twist(l, r)
            self.assertAlmostEqual(v, v2, places=9)
            self.assertAlmostEqual(w, w2, places=9)

    def test_saturation_keeps_turn_ratio(self):
        lim = K.DriveLimits(max_linear=10, max_angular=10, max_wheel_rad_s=5)
        l, r = K.twist_to_tracks(2.0, 1.0, lim)
        self.assertLessEqual(max(abs(l), abs(r)), 5 + 1e-9)
        l0, r0 = (2.0 - 1.0 * K.TRACK_GAUGE / 2 * lim.turn_slip), (2.0 + 1.0 * K.TRACK_GAUGE / 2 * lim.turn_slip)
        self.assertAlmostEqual(l / r, l0 / r0, places=9)

    def test_command_limits(self):
        l, r = K.twist_to_tracks(5.0, 0.0)
        self.assertAlmostEqual(l * K.SPROCKET_RADIUS, K.DriveLimits().max_linear, places=9)

    def test_pwm_cap_protects_12v_motor(self):
        self.assertAlmostEqual(K.pwm_cap(16.8), 12 / 16.8)
        self.assertEqual(K.pwm_cap(11.0), 1.0)
        self.assertEqual(K.pwm_cap(0), 0.0)

    def test_odometry_straight_line(self):
        o = K.Odometry()
        o.update(0, 0)
        turns = 1.0 / (2 * math.pi * K.SPROCKET_RADIUS)          # 1 m of track travel
        x, y, yaw = o.update(turns * 2 * math.pi, turns * 2 * math.pi)
        self.assertAlmostEqual(x, 1.0, places=6)
        self.assertAlmostEqual(y, 0.0, places=6)
        self.assertAlmostEqual(yaw, 0.0, places=6)

    def test_encoder_counts(self):
        self.assertAlmostEqual(K.counts_to_rad(6533), 2 * math.pi)


class HeadMapping(unittest.TestCase):
    def test_limits_and_pulse_width(self):
        h = Head()
        self.assertAlmostEqual(h.pan.to_us(60), 1900, places=6)
        self.assertAlmostEqual(h.pan.to_us(-90), 1100, places=6)      # clamped to -60°
        self.assertAlmostEqual(h.tilt.to_us(12), 1580, places=6)

    def test_slew_rate(self):
        h = Head()
        h.set_target(60, 0)
        h.step(0.1)
        self.assertAlmostEqual(h.state[0], 9.0, places=6)              # 90°/s × 0.1 s
        for _ in range(100):
            h.step(0.1)
        self.assertAlmostEqual(h.state[0], 60.0)
        self.assertAlmostEqual(h.gaze()[0], 1.0)


class Safety(unittest.TestCase):
    floor = [0.055] * 4

    def test_estop_and_timeout(self):
        self.assertEqual(gate(0.2, 0.5, estop=True, bumpers=0, cliff=self.floor, cmd_age_s=0).reason, 'estop')
        self.assertEqual(gate(0.2, 0.5, estop=False, bumpers=0, cliff=self.floor, cmd_age_s=1).linear, 0.0)

    def test_front_bumper_blocks_forward_only(self):
        v = gate(0.2, 0, estop=False, bumpers=0b01, cliff=self.floor, cmd_age_s=0)
        self.assertEqual((v.linear, v.reason), (0.0, 'front_blocked'))
        v = gate(-0.2, 0, estop=False, bumpers=0b01, cliff=self.floor, cmd_age_s=0)
        self.assertEqual(v.linear, -0.2)

    def test_cliff(self):
        v = gate(0.2, 0, estop=False, bumpers=0, cliff=[0.3, 0.055, 0.055, 0.055], cmd_age_s=0)
        self.assertEqual(v.linear, 0.0)
        v = gate(0.2, 0.3, estop=False, bumpers=0, cliff=[0.3, 0.3, 0.055, 0.055], cmd_age_s=0)
        self.assertEqual((v.linear, v.angular, v.reason), (0.0, 0.0, 'lifted_or_cliff'))

    def test_invalid_reading_is_a_cliff(self):
        v = gate(0.2, 0, estop=False, bumpers=0, cliff=[0.0, 0.055, 0.055, 0.055], cmd_age_s=0)
        self.assertEqual(v.linear, 0.0)


class Battery(unittest.TestCase):
    def test_soc_curve(self):
        self.assertEqual(soc_from_voltage(16.8), 100.0)
        self.assertEqual(soc_from_voltage(13.0), 0.0)
        self.assertTrue(40 < soc_from_voltage(15.08) < 55)   # 3.77 V/cell

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

    def test_undock_from_docked_reverses_then_idle(self):
        d = Dock()
        d.state = 'DOCKED'
        d.request('undock')
        self.assertEqual(d.state, 'UNDOCK')
        lin, _ = d.step(Obs(dock_sense=True), 0.05)
        self.assertLess(lin, 0)
        self.run_to(d, Obs(), ticks=100)
        self.assertEqual(d.state, 'IDLE')

    def test_undock_reverses_then_idle(self):
        d = Dock()
        d.state = 'CHARGING'
        d.request('undock')
        lin, _ = d.step(Obs(), 0.05)
        self.assertLess(lin, 0)
        self.run_to(d, Obs(), ticks=100)
        self.assertEqual(d.state, 'IDLE')


class DockingLoop(unittest.TestCase):
    """Dock state machine closed around the mock MCU plant at the node rates (dock 20 Hz, MCU 50 Hz)."""

    def simulate(self, plant, dock, seconds):
        states, ticks_per_dock = [], 0
        for i in range(int(seconds / 0.02)):
            if i * 0.02 >= ticks_per_dock * 0.05 - 1e-9:
                tag = plant.tag()
                obs = Obs(tag_visible=tag is not None, dock_sense=plant.docked, charging=plant.charging)
                if tag:
                    obs.tag_offset, obs.tag_distance = tag
                lin, ang = dock.step(obs, 0.05)
                plant.target = list(K.twist_to_tracks(lin, ang))
                states.append(dock.state)
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


class Mood(unittest.TestCase):
    def test_events_are_face_events(self):
        m = MoodMapper()
        ev = m.update(bumpers=1, dock_state='IDLE', battery_action='run', gaze=(0.5, 0))
        self.assertEqual([e['type'] for e in ev], ['tap', 'gaze'])
        ev = m.update(bumpers=1, dock_state='IDLE', battery_action='run', gaze=(0.5, 0))
        self.assertEqual(ev, [])                                     # held bumper does not repeat
        ev = m.update(bumpers=0, dock_state='APPROACH', battery_action='seek_dock', gaze=(0.5, 0))
        self.assertEqual([e['type'] for e in ev], ['curious'])
        ev = m.update(bumpers=0, dock_state='CONTACT', battery_action='seek_dock', gaze=(0.5, 0))
        self.assertEqual([e['type'] for e in ev], ['stroke'])
        ev = m.update(bumpers=0, dock_state='CHARGING', battery_action='charge', gaze=(0.0, 0))
        self.assertEqual([e['type'] for e in ev], ['sleep'])
        ev = m.update(bumpers=0, dock_state='UNDOCK', battery_action='run', gaze=(0.0, 0))
        self.assertIn('wake', [e['type'] for e in ev])
        for e in ev:
            self.assertIn(e['type'], FACE_EVENTS)

    def test_snapshot_brings_connecting_face_to_current_state(self):
        m = MoodMapper()
        for dock in ('CHARGING', 'DOCKED'):
            ev = m.snapshot(bumpers=0, dock_state=dock, battery_action='charge', gaze=(0.4, 0.1))
            self.assertEqual(ev, [{'type': 'sleep'}])                # asleep on the dock, gaze ignored
        ev = m.snapshot(bumpers=1, dock_state='APPROACH', battery_action='seek_dock', gaze=(0.3, -0.2))
        self.assertEqual(ev, [{'type': 'wake'}, {'type': 'gaze', 'x': 0.3, 'y': -0.2}])  # tap/curious are moments
        ev = m.snapshot(bumpers=0, dock_state='IDLE', battery_action='run', gaze=(0.0, 0.0))
        self.assertEqual(ev, [{'type': 'wake'}])                     # a face that slept before a drop wakes up
        ev = m.update(bumpers=0, dock_state='CHARGING', battery_action='charge', gaze=(0.0, 0))
        self.assertEqual(ev, [{'type': 'sleep'}])                    # snapshot left the live mapper untouched


if __name__ == '__main__':
    unittest.main()
