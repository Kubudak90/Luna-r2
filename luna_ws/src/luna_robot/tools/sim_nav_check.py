#!/usr/bin/env python3
"""Scenario check for `ros2 launch luna_robot luna_nav.launch.py sim:=true` (run in the ROS container while the
launch is up). Judges with the simulator's ground truth (/luna/sim/truth), which the robot stack never sees.

  python3 tools/sim_nav_check.py mapping --map /ws/maps/room
      Nav2 goal behind the cabinet while the map is being built, E-stop pressed mid-drive, go-to-dock, map saved.
  python3 tools/sim_nav_check.py localization --map /ws/maps/room
      launched with slam_mode:=localization sim_soc:=0.24: the low battery must send Luna to the dock by itself.
  python3 tools/sim_nav_check.py course --world table --label lidar
      obstacle course (launched with world:=table, and obstacle_sensors:= the label): one Nav2 goal past the
      obstacle (go-to-dock for world dock, on the saved map of `room` with slam_mode:=localization). Counts the
      simulator's contacts (/luna/sim/contact) and prints one RESULT {json} line for the A/B table.
Exit code 0 only if every check passes."""
import argparse
import json
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from slam_toolbox.srv import SaveMap, SerializePoseGraph
from std_msgs.msg import Bool, Float32MultiArray, String
from tf2_ros import Buffer, TransformException, TransformListener

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from luna_robot import topics as T                                          # noqa: E402
from luna_robot.core.dock_nav import Pose2D, dock_frame, parse_dock_pose, pose_error, pre_dock_pose  # noqa: E402
from luna_robot.core.sim2d import clearance, default_room                   # noqa: E402
from luna_robot.core.sim_worlds import world as sim_world                  # noqa: E402

ROOM = default_room()
PRE = pre_dock_pose(ROOM.dock, 0.6)
# Behind the cabinet: unknown at the start but inside the first map's bounds (the planner can't plan to a goal
# outside the map it has so far; the cabinet hides everything east of x ≈ 2.5 m from the start pose).
EXPLORE = Pose2D(2.3, 1.35, 0.0)
# Obstacle courses (core/sim_worlds.py): world → (goal in map = world, what all sensors should achieve, timeout s).
# 'reach': get there without touching anything; 'refuse': the only way is blocked, Nav2 must give up untouched;
# 'dock': go-to-dock past the clutter.
GOAL = Pose2D(3.5, 0.0, 0.0)
COURSES = {'toy': (GOAL, 'reach', 180), 'table': (GOAL, 'reach', 180), 'chair': (GOAL, 'reach', 180),
           'door': (GOAL, 'refuse', 300), 'dock': (None, 'dock', 240)}


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Check(Node):
    def __init__(self):
        super().__init__('luna_sim_nav_check')
        self.t0 = time.monotonic()
        self.truth, self.speed = None, 0.0
        self.states = {T.DOCK_STATE: None, T.NAV_DOCK_STATE: None, T.POWER_ACTION: None, T.SAFETY: None}
        self.events, self.results = [], []
        self.map_info = None
        self.cmd_vel_count = 0
        self.wheel_target = None
        self.at_handover = None
        self.contacts = []
        self.obstacles, self.clear, self.trace, self.next_trace = (), {}, [], 0.0
        self.create_subscription(Odometry, T.SIM_TRUTH, self._truth, 10)
        self.create_subscription(String, T.SIM_CONTACT, self._contact, 50)
        for topic in self.states:
            self.create_subscription(String, topic, lambda m, t=topic: self._state(t, m.data), 10)
        self.create_subscription(Twist, T.CMD_VEL, self._cmd_vel, 10)
        self.create_subscription(Float32MultiArray, T.MCU_WHEEL_TARGET,
                                 lambda m: setattr(self, 'wheel_target', list(m.data)), 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', lambda m: setattr(self, 'map_info', m.info), latched)
        self.pub_go = self.create_publisher(String, T.NAV_DOCK, 10)
        self.pub_estop = self.create_publisher(Bool, T.SIM_ESTOP, 10)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)

    # --- inputs ---------------------------------------------------------------------------------------------------
    def _truth(self, m):
        self.truth = Pose2D(m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation))
        self.speed = abs(m.twist.twist.linear.x)
        for ob in self.obstacles:                    # closest approach to each course obstacle, footprint in plan view
            self.clear[ob.name] = min(self.clear.get(ob.name, math.inf), clearance(self.truth, ob))
        if self.obstacles and time.monotonic() >= self.next_trace:
            self.next_trace = time.monotonic() + 1.0
            self.trace.append((round(self.truth.x, 2), round(self.truth.y, 2)))

    def _state(self, topic, value):
        if self.states[topic] != value:
            self.states[topic] = value
            self.log(f'{topic} = {value}')
            if topic == T.NAV_DOCK_STATE and value.startswith('HANDOVER') and self.truth is not None:
                self.at_handover = self.truth

    def _contact(self, m):
        t, name, kind, bands = m.data.split()
        self.contacts.append({'t': float(t), 'name': name, 'kind': kind, 'bands': bands})
        self.log(f'CONTACT {name} ({kind}, {bands})')

    def _cmd_vel(self, _m):
        self.cmd_vel_count += 1

    # --- helpers --------------------------------------------------------------------------------------------------
    def log(self, text):
        print(f'[{time.monotonic() - self.t0:7.2f}s] {text}', flush=True)

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait(self, pred, timeout, what):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            if pred():
                return True
        self.log(f'TIMEOUT waiting for {what}')
        return False

    def check(self, name, ok, detail=''):
        self.results.append((name, bool(ok), detail))
        self.log(f'{"PASS" if ok else "FAIL"}  {name}  {detail}')

    def map_pose(self):
        try:
            tf = self.tf.lookup_transform('map', 'base_footprint', Time())
        except TransformException:
            return None
        return Pose2D(tf.transform.translation.x, tf.transform.translation.y, yaw_of(tf.transform.rotation))

    def call(self, srv_type, name, request, timeout=30.0):
        client = self.create_client(srv_type, name)
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        future = client.call_async(request)
        self.wait(future.done, timeout, name)
        return future.result()

    # --- shared checks --------------------------------------------------------------------------------------------
    def check_bringup(self):
        ok = self.wait(lambda: self.truth is not None and self.map_info is not None and self.map_pose() is not None
                       and self.states[T.NAV_DOCK_STATE] is not None, 90, 'truth, /map, map→base_footprint, dock_nav')
        self.check('bringup: sim, SLAM map, TF and dock_nav up', ok)
        ok = self.nav.wait_for_server(timeout_sec=90)
        self.check('Nav2 navigate_to_pose server up', ok)
        self.spin_for(2.0)

    def check_velocity_graph(self):
        pubs = lambda topic: sorted(i.node_name for i in self.get_publishers_info_by_topic(topic))
        self.check('only base_node drives the MCU', pubs(T.MCU_WHEEL_TARGET) == ['luna_base'],
                   f'{T.MCU_WHEEL_TARGET}: {pubs(T.MCU_WHEEL_TARGET)}')
        self.check('Nav2 reaches /cmd_vel only via collision_monitor', pubs(T.CMD_VEL) == ['collision_monitor'],
                   f'{T.CMD_VEL}: {pubs(T.CMD_VEL)}')
        self.check('only dock_node publishes the dock twist', pubs(T.DOCK_CMD_VEL) == ['luna_dock'],
                   f'{T.DOCK_CMD_VEL}: {pubs(T.DOCK_CMD_VEL)}')

    def check_localization(self, label, xy_tol=0.10, yaw_tol=0.08):
        self.spin_for(1.0)
        est, truth = self.map_pose(), self.truth
        if est is None or truth is None:
            self.check(f'{label}: SLAM pose vs truth', False, 'no pose')
            return
        d, dyaw = pose_error(est, truth)
        self.check(f'{label}: SLAM pose vs truth', d <= xy_tol and dyaw <= yaw_tol,
                   f'{d * 100:.1f} cm, {math.degrees(dyaw):.1f}°  (map {est.x:.3f},{est.y:.3f}  truth {truth.x:.3f},{truth.y:.3f})')

    def check_docking(self, label, timeout=240):
        ok = self.wait(lambda: self.states[T.DOCK_STATE] == 'CHARGING' and
                       (self.states[T.NAV_DOCK_STATE] or '').startswith('DOCKED'), timeout, 'CHARGING + DOCKED')
        self.check(f'{label}: dock_node CHARGING and dock_nav DOCKED', ok, f'{time.monotonic() - self.t_go:.1f} s after start')
        if self.at_handover is not None:
            along, lateral, dyaw = dock_frame(self.at_handover, ROOM.dock)
            self.check(f'{label}: true pose at hand-over inside the approach cone', abs(lateral) <= 0.11
                       and -0.8 <= along <= 0.0, f'{-along:.3f} m before the dock, {lateral * 100:+.1f} cm beside '
                       f'the axis, {math.degrees(dyaw):+.1f}°')
        else:
            self.check(f'{label}: hand-over happened', False)
        if self.truth is not None:
            d, dyaw = pose_error(self.truth, ROOM.dock)
            self.check(f'{label}: true final pose on the dock', d <= 0.02 and dyaw <= 0.2,
                       f'{d * 1000:.0f} mm, {math.degrees(dyaw):.1f}°')

    def goal(self, pose):
        g = NavigateToPose.Goal()
        g.pose = PoseStamped()
        g.pose.header.frame_id = 'map'
        g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose.position.x, g.pose.pose.position.y = pose.x, pose.y
        g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(pose.yaw / 2), math.cos(pose.yaw / 2)
        return g

    def contact_summary(self):
        crashes = [c for c in self.contacts if c['kind'] == 'crash']
        return (f'{len(self.contacts)} contacts ({len(crashes)} unfelt): '
                + (', '.join(f"{c['name']}/{c['kind']}/{c['bands']}" for c in self.contacts) or 'none'))

    # --- scenarios ------------------------------------------------------------------------------------------------
    def mapping(self, map_base):
        self.check_bringup()
        self.check_velocity_graph()
        self.check_localization('start')
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = EXPLORE.x, EXPLORE.y
        goal.pose.pose.orientation.w = 1.0
        self.log(f'Nav2 goal behind the cabinet {EXPLORE}')
        sent = self.nav.send_goal_async(goal)
        self.wait(sent.done, 10, 'goal response')
        handle = sent.result()
        self.check('exploration goal accepted', handle is not None and handle.accepted)
        result = handle.get_result_async()
        # E-stop mid-drive: Nav2 keeps commanding, base_node's gate must stop the tracks on its own.
        self.wait(lambda: self.speed > 0.15, 60, 'driving at > 0.15 m/s')
        self.log(f'E-stop pressed at {self.speed:.2f} m/s')
        self.pub_estop.publish(Bool(data=True))
        pressed, count0 = time.monotonic(), self.cmd_vel_count
        gated = self.wait(lambda: self.wheel_target is not None and self.wheel_target[:2] == [0.0, 0.0], 2.0,
                          'zero wheel target')
        gate_s = time.monotonic() - pressed
        stopped = self.wait(lambda: self.speed < 0.01, 2.0, 'standstill')
        stop_s = time.monotonic() - pressed
        self.spin_for(2.0)
        moved = self.speed
        commands = self.cmd_vel_count - count0
        self.pub_estop.publish(Bool(data=False))
        self.log('E-stop released')
        # Standstill includes the track motors' modelled lag (tau 0.15 s: 0.15 → 0.01 m/s takes ~0.4 s).
        self.check('E-stop stops the robot while Nav2 still commands',
                   gated and stopped and stop_s <= 1.0 and moved < 0.01 and commands > 20,
                   f'wheel target 0 after {gate_s:.2f} s, standstill after {stop_s:.2f} s, '
                   f'{commands} /cmd_vel msgs while pressed')
        self.wait(result.done, 120, 'exploration goal result')
        status = result.result().status if result.done() else None
        self.check('exploration goal reached after the E-stop', status == 4, f'status {status}')   # 4 = SUCCEEDED
        self.check_localization('behind the cabinet')
        self.t_go = time.monotonic()
        self.pub_go.publish(String(data='go'))
        self.log('go-to-dock requested')
        self.check_docking('mapping')
        self.check_localization('on the dock')
        dock_file = map_base + '.dock.yaml'
        self.spin_for(1.0)
        learned = parse_dock_pose(open(dock_file).read()) if os.path.exists(dock_file) else None
        if learned is None:
            self.check('dock pose learned and saved', False, dock_file)
        else:                                                         # map ≈ world: the robot started at 0, 0, 0
            d, dyaw = pose_error(learned, ROOM.dock)
            self.check('dock pose learned and saved, axis from the wall', d <= 0.04 and dyaw <= math.radians(2),
                       f'{learned}: {d * 100:.1f} cm, {math.degrees(dyaw):.1f}° from the true dock')
        os.makedirs(os.path.dirname(map_base), exist_ok=True)
        r = self.call(SerializePoseGraph, '/slam_toolbox/serialize_map', SerializePoseGraph.Request(filename=map_base))
        self.check('pose graph serialized', r is not None and os.path.exists(map_base + '.posegraph'),
                   f'{map_base}.posegraph')
        req = SaveMap.Request()
        req.name.data = map_base
        self.call(SaveMap, '/slam_toolbox/save_map', req, timeout=90.0)
        self.check('occupancy map saved (.pgm/.yaml)',
                   self.wait(lambda: os.path.exists(map_base + '.yaml'), 30, 'map files'), f'{map_base}.pgm')
        self.log(f'simulator contacts: {self.contact_summary()}')
        i = self.map_info
        self.log(f'map {i.width}×{i.height} cells at {i.resolution} m, origin ({i.origin.position.x:.2f}, '
                 f'{i.origin.position.y:.2f})')

    def localization(self, _map_base):
        self.check_bringup()
        self.check_velocity_graph()
        self.check_localization('start on the saved map')
        self.t_go = time.monotonic()
        ok = self.wait(lambda: self.states[T.POWER_ACTION] == 'seek_dock' and
                       (self.states[T.NAV_DOCK_STATE] or '') not in ('IDLE', ''), 30, 'low-battery go-to-dock')
        self.check('low battery starts go-to-dock by itself', ok,
                   f'power {self.states[T.POWER_ACTION]}, dock_nav {self.states[T.NAV_DOCK_STATE]}')
        self.check_docking('localization')
        self.check_localization('on the dock')
        self.wait(lambda: self.states[T.POWER_ACTION] == 'charge', 10, 'power action charge')
        self.check('battery policy switches to charge', self.states[T.POWER_ACTION] == 'charge')
        self.log(f'simulator contacts: {self.contact_summary()}')

    def course(self, _map_base, world, label):
        goal, expect, timeout = COURSES[world]
        self.obstacles = tuple(o for o in sim_world(world).obstacles if o.contact)
        self.check_bringup()
        self.t_go, status = time.monotonic(), None
        if expect == 'dock':
            self.pub_go.publish(String(data='go'))
            self.log('go-to-dock requested')
            done = self.wait(lambda: self.states[T.DOCK_STATE] == 'CHARGING' and
                             (self.states[T.NAV_DOCK_STATE] or '').startswith('DOCKED'), timeout, 'CHARGING + DOCKED')
        else:
            self.log(f'Nav2 goal {goal} past the {world}')
            sent = self.nav.send_goal_async(self.goal(goal))
            self.wait(sent.done, 10, 'goal response')
            handle = sent.result()
            result = handle.get_result_async() if handle is not None and handle.accepted else None
            done = result is not None and self.wait(result.done, timeout, 'goal result')
            status = result.result().status if done else None
            if result is not None and not done:
                handle.cancel_goal_async()
                self.spin_for(2.0)
        elapsed = time.monotonic() - self.t_go
        truth = self.truth
        reached = status == 4 if expect != 'dock' else done                     # 4 = SUCCEEDED
        if expect == 'reach':
            self.check(f'{world}: goal reached', reached, f'status {status}, {elapsed:.1f} s')
        elif expect == 'refuse':
            self.check(f'{world}: Nav2 gives up (aborted), goal not reached', status == 6 and not reached,
                       f'status {status}, {elapsed:.1f} s')                     # 6 = ABORTED
        else:
            self.check(f'{world}: docked (CHARGING + DOCKED)', reached, f'{elapsed:.1f} s')
        self.check(f'{world}: no contact with any obstacle', not self.contacts, self.contact_summary())
        print('RESULT ' + json.dumps({
            'world': world, 'sensors': label, 'expect': expect, 'status': status, 'reached': bool(reached),
            'time_s': round(elapsed, 1), 'contacts': len(self.contacts),
            'unfelt': sum(c['kind'] == 'crash' for c in self.contacts), 'items': self.contacts,
            'final': [round(truth.x, 3), round(truth.y, 3)] if truth else None,
            'clearance_cm': {k: round(v * 100, 1) for k, v in sorted(self.clear.items())}}), flush=True)
        print('TRACE ' + ' '.join(f'{x},{y}' for x, y in self.trace), flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('scenario', choices=['mapping', 'localization', 'course'])
    ap.add_argument('--map', default=os.path.expanduser('~/.luna/maps/home'))
    ap.add_argument('--world', choices=sorted(COURSES), help='course only: the launch\'s world:=')
    ap.add_argument('--label', default='', help='course only: the launch\'s obstacle_sensors:= (for RESULT)')
    args = ap.parse_args()
    rclpy.init()
    node = Check()
    try:
        if args.scenario == 'course':
            node.course(os.path.expanduser(args.map), args.world, args.label)
        else:
            getattr(node, args.scenario)(os.path.expanduser(args.map))
    finally:
        failed = [r for r in node.results if not r[1]]
        print(f'\n{len(node.results) - len(failed)}/{len(node.results)} checks passed')
        for name, ok, detail in node.results:
            print(f'  {"PASS" if ok else "FAIL"}  {name}  {detail}')
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(1 if failed or not node.results else 0)


if __name__ == '__main__':
    main()
