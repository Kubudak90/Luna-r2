"""Go-to-dock: Nav2 drives to the pre-dock pose in front of the learned dock, then dock_node docks on the tag.
Decisions live in core/dock_nav.py; this node feeds it the map pose (TF), the dock/battery states, the wall seen
ahead while docked (/scan) and Nav2's action results, and runs Nav2's navigate_to_pose and backup actions. Requests on /luna/nav/dock (go | cancel); with `auto` it also starts when the
battery policy (/luna/power/action) asks for the dock. Run dock_node with auto_dock off next to it, so only one
of the two decides when to dock. Nav2 itself only ever reaches /cmd_vel, i.e. base_node's safety gate."""
import math
import os
from collections import deque

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import BackUp, NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from . import topics as T
from .entry import run
from .core.dock_nav import (DockNav, DockNavConfig, NavObs, Pose2D, format_dock_pose, front_points, parse_dock_pose,
                            wall_normal_yaw, wrap)
from .core.robot_model import load_frames

STATUS = {GoalStatus.STATUS_SUCCEEDED: 'succeeded', GoalStatus.STATUS_CANCELED: 'canceled'}
POSE_MAX_AGE_S = 2.0          # map→base older than this (SLAM stopped) counts as not localized
LASER_X = load_frames().origin('laser')[0]   # urdf laser_joint
WALL_RANGE_M, WALL_HALF_ANGLE = 1.0, math.radians(35)   # the wall behind the dock, seen while docked
BACKUP_SPEED = 0.08


class DockNavNode(Node):
    def __init__(self):
        super().__init__('luna_dock_nav')
        self.declare_parameter('auto', True)
        self.declare_parameter('standoff_m', DockNavConfig.standoff_m)
        self.declare_parameter('dock_file', '')                       # learned dock pose, kept next to the map
        self.declare_parameter('dock_pose', [math.nan] * 3)           # [x, y, yaw] in map, if there is no dock_file
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('nav2_manager', 'lifecycle_manager_navigation')
        p = lambda name: self.get_parameter(name).value
        self.map_frame, self.base_frame = p('map_frame'), p('base_frame')
        self.dock_file = os.path.expanduser(p('dock_file'))
        self.nav = DockNav(DockNavConfig(standoff_m=p('standoff_m'), auto=p('auto')), dock=self._load_dock())
        self.obs = NavObs()
        self.results = deque()                                        # (seq, status) from Nav2, one per tick
        self.handles, self.cancel_early = {}, set()
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.backup_client = ActionClient(self, BackUp, 'backup')
        # A goal sent while Nav2 is still activating is queued and accepted seconds later. Its stamp then predates the
        # TF data in bt_navigator / planner_server, which look the robot pose up at that stamp and fail forever (seen
        # in the simulator: go-to-dock never left the start). So goals wait for the lifecycle manager's "active".
        self.nav2_active = False
        self.active_client = self.create_client(Trigger, f"/{p('nav2_manager')}/is_active")
        self.active_call = None
        self.scan = None
        self.create_subscription(LaserScan, T.SCAN, lambda m: setattr(self, 'scan', m), qos_profile_sensor_data)
        self.create_subscription(String, T.DOCK_STATE, lambda m: setattr(self.obs, 'dock_state', m.data), 10)
        self.create_subscription(String, T.POWER_ACTION, lambda m: setattr(self.obs, 'power_action', m.data), 10)
        self.create_subscription(String, T.NAV_DOCK, lambda m: self.nav.request(m.data), 10)
        self.pub_dock = self.create_publisher(String, T.DOCK_REQUEST, 10)
        self.pub_state = self.create_publisher(String, T.NAV_DOCK_STATE, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_pre = self.create_publisher(PoseStamped, T.NAV_PRE_DOCK, latched)
        self.create_service(Trigger, T.NAV_SAVE_DOCK, self._save_here)
        self.dt = 0.1
        self.create_timer(self.dt, self._tick)
        self._publish_pre_dock()

    # --- dock pose file -------------------------------------------------------------------------------------------
    def _load_dock(self):
        if self.dock_file and os.path.exists(self.dock_file):
            with open(self.dock_file) as f:
                dock = parse_dock_pose(f.read())
            self.get_logger().info(f'dock pose from {self.dock_file}: {dock}')
            return dock
        xyz = list(self.get_parameter('dock_pose').value)
        if len(xyz) == 3 and not any(math.isnan(v) for v in xyz):
            return Pose2D(*xyz)
        self.get_logger().warn('no dock pose yet: go-to-dock falls back to dock_node tag search until one is learned')
        return None

    def _save_dock(self, pose):
        if not self.dock_file:
            self.get_logger().info(f'dock pose learned (not saved, dock_file unset): {pose}')
            return
        os.makedirs(os.path.dirname(self.dock_file) or '.', exist_ok=True)
        tmp = self.dock_file + '.tmp'
        with open(tmp, 'w') as f:
            f.write(format_dock_pose(pose))
        os.replace(tmp, self.dock_file)
        self.get_logger().info(f'dock pose saved to {self.dock_file}: {pose}')

    def _save_here(self, _req, res):
        pose = self._pose()
        if pose is None:
            res.success, res.message = False, f'no {self.map_frame}→{self.base_frame} transform'
            return res
        wall = self._wall_yaw()
        if wall is not None:
            pose = Pose2D(pose.x, pose.y, wrap(pose.yaw + wall))
        self.nav.dock = pose
        self._save_dock(pose)
        self._publish_pre_dock()
        res.success, res.message = True, f'dock = {pose}'
        return res

    # --- loop -----------------------------------------------------------------------------------------------------
    def _pose(self):
        try:
            tf = self.tf.lookup_transform(self.map_frame, self.base_frame, Time())
        except TransformException:
            return None
        if (self.get_clock().now() - Time.from_msg(tf.header.stamp)).nanoseconds * 1e-9 > POSE_MAX_AGE_S:
            return None
        q, t = tf.transform.rotation, tf.transform.translation
        return Pose2D(t.x, t.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def _wall_yaw(self):
        m = self.scan
        if m is None:
            return None
        pts = front_points(m.ranges, m.angle_min, m.angle_increment, LASER_X, WALL_RANGE_M, WALL_HALF_ANGLE)
        return wall_normal_yaw(pts)

    def _tick(self):
        self._nav2_ready()                                            # keeps asking until Nav2 is active
        self.obs.pose = self._pose()
        self.obs.wall_yaw = self._wall_yaw() if self.obs.dock_state in ('CONTACT', 'CHARGING') else None
        self.obs.nav_result = self.results.popleft() if self.results else None
        before = self.nav.state
        for cmd in self.nav.step(self.obs, self.dt):
            self._do(cmd)
        if self.nav.state != before:
            self.get_logger().info(f'{before} → {self.nav.state} {self.nav.reason}'.rstrip())
        self.pub_state.publish(String(data=self.nav.state + (f':{self.nav.reason}' if self.nav.reason else '')))

    def _do(self, cmd):
        kind = cmd[0]
        if kind == 'navigate':
            self._navigate(cmd[1], cmd[2])
        elif kind == 'backup':
            self._backup(cmd[1], cmd[2])
        elif kind == 'cancel_nav':
            handle = self.handles.pop(cmd[1], None)
            if handle is not None:
                handle.cancel_goal_async()
            else:
                self.cancel_early.add(cmd[1])                         # goal response not back yet
        elif kind == 'dock':
            self.pub_dock.publish(String(data=cmd[1]))
        elif kind == 'save_dock':
            self._save_dock(cmd[1])
            self._publish_pre_dock()
        elif kind == 'reject_dock':
            self.get_logger().warn(f'charging at {cmd[1]}, far from the stored dock {self.nav.dock}: not learned '
                                   '(localization off, or the dock moved: then call ' + T.NAV_SAVE_DOCK + ')')

    def _nav2_ready(self):
        if not self.nav2_active and self.active_client.service_is_ready():
            if self.active_call is not None and self.active_call.done():
                result = self.active_call.result()
                self.nav2_active, self.active_call = bool(result and result.success), None
            elif self.active_call is None:
                self.active_call = self.active_client.call_async(Trigger.Request())
        return self.nav2_active and self.client.server_is_ready()

    def _navigate(self, seq, pose):
        if not self._nav2_ready():
            self.get_logger().warn('Nav2 navigate_to_pose is not available yet', throttle_duration_sec=10.0)
            self.results.append((seq, 'unavailable'))
            return
        goal = NavigateToPose.Goal()
        goal.pose = self._stamped(pose)
        self._publish_pre_dock()
        self.client.send_goal_async(goal).add_done_callback(lambda f: self._accepted(seq, f))

    def _backup(self, seq, metres):
        if not self.backup_client.server_is_ready():
            self.get_logger().warn('Nav2 backup behaviour is not available')
            self.results.append((seq, 'unavailable'))
            return
        goal = BackUp.Goal()
        goal.target.x, goal.speed = metres, BACKUP_SPEED       # behavior_server reverses; base_node still gates it
        goal.time_allowance = Duration(sec=int(metres / BACKUP_SPEED * 2) + 5)
        self.backup_client.send_goal_async(goal).add_done_callback(lambda f: self._accepted(seq, f))

    def _accepted(self, seq, future):
        handle = future.result()
        if not handle.accepted:
            self.results.append((seq, 'rejected'))
            return
        if seq in self.cancel_early:
            self.cancel_early.discard(seq)
            handle.cancel_goal_async()
        else:
            self.handles[seq] = handle
        handle.get_result_async().add_done_callback(lambda f: self._finished(seq, f))

    def _finished(self, seq, future):
        self.handles.pop(seq, None)
        self.results.append((seq, STATUS.get(future.result().status, 'aborted')))

    def _stamped(self, pose):
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.map_frame
        m.pose.position.x, m.pose.position.y = pose.x, pose.y
        m.pose.orientation.z, m.pose.orientation.w = math.sin(pose.yaw / 2), math.cos(pose.yaw / 2)
        return m

    def _publish_pre_dock(self):
        if self.nav.pre_dock is not None:
            self.pub_pre.publish(self._stamped(self.nav.pre_dock))


def main():
    run(DockNavNode)


if __name__ == '__main__':
    main()
