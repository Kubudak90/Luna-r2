"""Restarts a locked-up slam_toolbox (localization mode) and hands it the robot's pose (core/slam_watchdog.py).
Started by launch/slam.launch.py next to slam_toolbox, which runs there with respawn: killing it brings it back with
the same saved map. Reads /tf directly (map→odom from slam_toolbox, odom→base_footprint from base_node) instead of a
tf2 buffer, and publishes the pose on /initialpose, where slam_toolbox relocalizes near it."""
import math
import os
import signal
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

from . import topics as T
from .entry import run
from .core.dock_nav import Pose2D
from .core.slam_watchdog import SlamWatchdog, WatchdogConfig, compose


def _planar(tf):
    q, t = tf.transform.rotation, tf.transform.translation
    return Pose2D(t.x, t.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


def slam_pids(node_name='slam_toolbox'):
    """PIDs of the slam_toolbox process(es), from /proc (the launch prefix execs into the binary)."""
    pids = []
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        try:
            with open(f'/proc/{entry}/cmdline', 'rb') as f:
                args = f.read().split(b'\0')
        except OSError:
            continue
        if any(a.endswith(b'slam_toolbox_node') for a in args) and f'__node:={node_name}'.encode() in args:
            pids.append(int(entry))
    return pids


class SlamWatchdogNode(Node):
    def __init__(self):
        super().__init__('luna_slam_watchdog')
        self.declare_parameter('stall_s', WatchdogConfig.stall_s)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        p = lambda name: self.get_parameter(name).value
        self.frames = (p('map_frame'), p('odom_frame'), p('base_frame'))
        self.dog = SlamWatchdog(WatchdogConfig(stall_s=p('stall_s')))
        self.map_odom = self.odom_base = None
        self.map_odom_stamp, self.advanced_at, self.scan_at = None, None, None
        self.create_subscription(TFMessage, '/tf', self._tf, 100)
        self.create_subscription(LaserScan, T.SCAN, lambda m: setattr(self, 'scan_at', time.monotonic()),
                                 qos_profile_sensor_data)
        self.pub_pose = self.create_publisher(PoseWithCovarianceStamped, T.INITIAL_POSE, 10)
        self.probe = self.create_client(GetParameters, '/slam_toolbox/get_parameters')
        self.answered_at = None
        self.create_timer(0.5, self._tick)

    def _tf(self, m):
        map_frame, odom_frame, base_frame = self.frames
        for tf in m.transforms:
            if tf.header.frame_id == odom_frame and tf.child_frame_id == base_frame:
                self.odom_base = _planar(tf)
            elif tf.header.frame_id == map_frame and tf.child_frame_id == odom_frame:
                stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
                if self.map_odom_stamp is None or stamp > self.map_odom_stamp:
                    self.map_odom_stamp, self.advanced_at = stamp, time.monotonic()
                    if self.dog.state == 'WATCH':           # only a healthy SLAM's correction is worth keeping
                        self.map_odom = _planar(tf)

    def _tick(self):
        listening = self.count_subscribers(T.INITIAL_POSE) > 0
        for action in self.dog.step(time.monotonic(), self.advanced_at, self.scan_at, listening, self.answered_at):
            if action == 'probe':
                self.get_logger().warn('map→odom has stopped while scans arrive: is slam_toolbox alive?')
                self.probe.call_async(GetParameters.Request(names=['mode'])).add_done_callback(self._answered)
            elif action == 'restart':
                pids = slam_pids()
                self.get_logger().error(f'slam_toolbox is locked up (no map→odom, no answer): restarting it '
                                        f'(pid {pids}, restart #{self.dog.restarts})')
                for pid in pids:
                    os.kill(pid, signal.SIGKILL)
            elif action == 'initial_pose':
                self._send_pose()

    def _answered(self, future):
        if future.result() is not None:
            self.answered_at = time.monotonic()
            self.get_logger().warn('slam_toolbox answers: only slow, not restarted')

    def _send_pose(self):
        if self.map_odom is None or self.odom_base is None:
            self.get_logger().warn('no pose to hand to slam_toolbox yet')
            return
        pose = compose(self.map_odom, self.odom_base)
        m = PoseWithCovarianceStamped()
        m.header.stamp, m.header.frame_id = self.get_clock().now().to_msg(), self.frames[0]
        m.pose.pose.position.x, m.pose.pose.position.y = pose.x, pose.y
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(pose.yaw / 2), math.cos(pose.yaw / 2)
        m.pose.covariance[0] = m.pose.covariance[7] = 0.05 ** 2
        m.pose.covariance[35] = math.radians(5) ** 2
        self.pub_pose.publish(m)
        self.get_logger().warn(f'handed slam_toolbox the pose {pose}')


def main():
    run(SlamWatchdogNode)


if __name__ == '__main__':
    main()
