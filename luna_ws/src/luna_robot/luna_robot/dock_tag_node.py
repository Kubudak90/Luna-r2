"""Dock tag → /luna/dock/tag for dock_node.

apriltag_ros runs once per head camera (namespaces /camera_left, /camera_right) and publishes the tag as TF frame
`dock_tag_<cam>` under that camera's optical frame. On every detection this node looks the tag up in base_link at
the image stamp; TF carries it through head tilt and pan (head_node → /joint_states → robot_state_publisher), so
the head may point anywhere. The cameras are fused and turned into (offset, distance) by core/dock_tag.py."""
import time
from functools import partial

from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from . import topics as T
from .entry import run
from .core.dock_tag import TagFuser, TagGeometry, observe


class DockTagNode(Node):
    def __init__(self):
        super().__init__('luna_dock_tag')
        self.declare_parameter('cameras', ['left', 'right'])
        self.declare_parameter('tag_id', 0)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('max_age_s', 0.3)
        self.declare_parameter('tf_wait_s', 0.2)
        p = self.get_parameter
        self.g = TagGeometry(tag_id=p('tag_id').value)
        self.base = p('base_frame').value
        self.tf_wait = p('tf_wait_s').value
        self.fuser = TagFuser(self.g, max_age_s=p('max_age_s').value)
        # No blocking lookups: apriltag_ros publishes `detections` before the tag's /tf, and a callback waiting for
        # it would hold up the very executor that delivers /tf (TransformListener(spin_thread=True) in Humble adds
        # this node to a second executor, so it doesn't help). Detections wait in `pending` until their TF is in.
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.pending = []                       # (camera, stamp, received monotonic s)
        for cam in p('cameras').value:
            self.create_subscription(AprilTagDetectionArray, f'/camera_{cam}/detections', partial(self._detections, cam), 10)
        self.pub_tag = self.create_publisher(PointStamped, T.DOCK_TAG, 10)
        self.pub_pos = self.create_publisher(PointStamped, T.DOCK_TAG_POSITION, 10)
        self.retry = self.create_timer(0.01, self._resolve)     # runs only while detections wait for TF
        self.retry.cancel()

    def _detections(self, cam, msg):
        if any(d.id == self.g.tag_id for d in msg.detections):
            self.pending.append((cam, msg.header.stamp, time.monotonic()))
            self._resolve()

    def _resolve(self):
        waiting = []
        for cam, stamp, received in self.pending:
            try:
                tf = self.tf.lookup_transform(self.base, f'dock_tag_{cam}', Time.from_msg(stamp))
            except TransformException as e:
                if time.monotonic() - received < self.tf_wait:
                    waiting.append((cam, stamp, received))
                else:
                    self.get_logger().warn(f'{cam}: {e}', throttle_duration_sec=5.0)
                continue
            self._use(cam, stamp, tf)
        self.pending = waiting
        if waiting and self.retry.is_canceled():
            self.retry.reset()
        elif not waiting:
            self.retry.cancel()

    def _use(self, cam, stamp, tf):
        t = tf.transform.translation
        now = stamp.sec + stamp.nanosec * 1e-9
        self.fuser.add(cam, now, (t.x, t.y, t.z))
        pos = self.fuser.position(now)
        obs = observe(pos, self.g) if pos else None
        if obs is None:
            self.get_logger().warn(f'{cam}: tag at ({t.x:.2f}, {t.y:.2f}, {t.z:.2f}) m is not where the dock tag can be',
                                   throttle_duration_sec=5.0)
            return
        out = PointStamped()
        out.header.stamp, out.header.frame_id = stamp, self.base
        out.point.x, out.point.z = obs
        self.pub_tag.publish(out)
        out = PointStamped()
        out.header.stamp, out.header.frame_id = stamp, self.base
        out.point.x, out.point.y, out.point.z = pos
        self.pub_pos.publish(out)


def main():
    run(DockTagNode)


if __name__ == '__main__':
    main()
