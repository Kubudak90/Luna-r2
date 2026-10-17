#!/usr/bin/env python3
"""Watches a running stack (e.g. `ros2 launch luna_robot luna.launch.py tags:=sim`) and reports the dock run:
state changes with times, /luna/dock/tag rate and range, and the mock's true final pose.
    python3 tools/dock_watch.py --seconds 45 [--pan 30]      (--pan: head target sent for the whole run)"""
import argparse
import math
import time

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, Vector3
from rclpy.node import Node
from std_msgs.msg import String

from luna_robot import topics as T


class Watch(Node):
    def __init__(self, pan):
        super().__init__('luna_dock_watch')
        self.t0 = time.monotonic()
        self.states, self.tags, self.truth, self.pan = [], [], None, pan
        self.create_subscription(String, T.DOCK_STATE, self._state, 10)
        self.create_subscription(PointStamped, T.DOCK_TAG, lambda m: self.tags.append((self._t(), m.point.x, m.point.z)), 50)
        self.create_subscription(PoseStamped, T.MOCK_TRUTH, lambda m: setattr(self, 'truth', m.pose), 10)
        self.pub_head = self.create_publisher(Vector3, T.HEAD_TARGET, 10)
        if pan is not None:
            self.create_timer(0.2, lambda: self.pub_head.publish(Vector3(x=float(pan), y=0.0, z=0.0)))

    def _t(self):
        return time.monotonic() - self.t0

    def _state(self, m):
        if not self.states or self.states[-1][1] != m.data:
            self.states.append((self._t(), m.data))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=45.0)
    ap.add_argument('--pan', type=float, default=None)
    a = ap.parse_args()
    rclpy.init()
    w = Watch(a.pan)
    end = time.monotonic() + a.seconds
    while time.monotonic() < end:
        rclpy.spin_once(w, timeout_sec=0.1)
    t_first = w.states[0][0] if w.states else 0.0
    print('durumlar: ' + ' → '.join(f'{s} ({t - t_first:.1f} s)' for t, s in w.states))
    if w.tags:
        span = w.tags[-1][0] - w.tags[0][0]
        print(f'/luna/dock/tag: {len(w.tags)} mesaj, {len(w.tags) / span if span > 0 else 0:.1f} Hz; '
              f'offset {min(t[1] for t in w.tags):+.3f}..{max(t[1] for t in w.tags):+.3f}, '
              f'mesafe {min(t[2] for t in w.tags):.3f}..{max(t[2] for t in w.tags):.3f} m; son mesafe {w.tags[-1][2]:.3f} m')
        gaps = [b[0] - a[0] for a, b in zip(w.tags, w.tags[1:])]
        print(f'en uzun etiket boşluğu {max(gaps) if gaps else 0:.2f} s')
    else:
        print('/luna/dock/tag: mesaj yok')
    if w.truth:
        q = w.truth.orientation
        yaw = math.degrees(2 * math.atan2(q.z, q.w))
        print(f'gerçek son poz (dock çerçevesi, oturmuş = 0): x {w.truth.position.x:+.4f} m, y {w.truth.position.y:+.4f} m, yaw {yaw:+.2f}°')
    w.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
