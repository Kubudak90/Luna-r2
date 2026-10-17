#!/usr/bin/env python3
"""Synthetic-image check of the dock-tag pipeline without hardware.

For each case (robot pose in the dock frame: d = travel left to the seated pose, y left, yaw left; head pan/tilt;
defocus) it publishes the head joint states, renders the
left camera's view of the tag (core/tag_scene.py, camera pose from core/frames.UrdfChain — not from TF, so the URDF
chain is cross-checked against robot_state_publisher too) and feeds the images to the real apriltag_ros node.
It then compares what dock_tag_node publishes on /luna/dock/tag and /luna/dock/tag_position with the truth.

    ros2 launch tools/sweep.launch.py decimate:=2.0 &
    python3 tools/tag_sweep.py --out /tmp/sweep.json [--width 1920 --height 1080]"""
import argparse
import array
import json
import math
import os
import threading
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, JointState

from luna_robot.core.camera_model import Pinhole
from luna_robot.core.dock_tag import TagGeometry, observe, tag_in_base
from luna_robot.core.frames import Pose, UrdfChain
from luna_robot.core.tag_scene import dock_T_tag, render

G = TagGeometry()


def cases():
    out = []
    for d in (0.0, 0.03, 0.06, 0.1, 0.2, 0.3, 0.5, 0.75, 1.0, 1.25, 1.5, 1.75, 2.0, 2.25, 2.5, 3.0):
        out.append(dict(group='menzil', d=d, y=0.0, yaw=0.0, pan=0.0, tilt=0.0, blur=0.0))
    for d, y, yaw in ((0.5, 0.15, 0.0), (1.0, -0.25, 0.0), (1.0, 0.0, 20.0), (1.0, 0.0, -25.0), (0.3, 0.05, 10.0), (1.5, 0.3, -10.0)):
        out.append(dict(group='yanal/yaw', d=d, y=y, yaw=yaw, pan=0.0, tilt=0.0, blur=0.0))
    for pan in (-35.0, -20.0, 20.0, 35.0):
        out.append(dict(group='pan', d=0.8, y=0.0, yaw=0.0, pan=pan, tilt=0.0, blur=0.0))
    for pan, yaw in ((-30.0, 25.0), (30.0, -25.0), (-50.0, 45.0)):     # head turned towards the tag
        out.append(dict(group='pan+yaw', d=0.8, y=0.0, yaw=yaw, pan=pan, tilt=0.0, blur=0.0))
    for tilt, d in ((10.0, 0.0), (-10.0, 0.5), (12.0, 1.0)):
        out.append(dict(group='tilt', d=d, y=0.0, yaw=0.0, pan=0.0, tilt=tilt, blur=0.0))
    for d, blur in ((0.0, 1.5), (0.0, 3.0), (0.3, 1.5), (1.0, 1.0), (1.5, 1.0)):
        out.append(dict(group='bulanık', d=d, y=0.0, yaw=0.0, pan=0.0, tilt=0.0, blur=blur))
    return out


class Sweep(Node):
    def __init__(self, cam_model):
        super().__init__('luna_tag_sweep')
        self.model = cam_model
        k = [cam_model.fx, 0.0, cam_model.cx, 0.0, cam_model.fy, cam_model.cy, 0.0, 0.0, 1.0]
        self.info = CameraInfo(width=cam_model.width, height=cam_model.height, distortion_model='plumb_bob',
                               d=[0.0] * 5, k=k, r=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                               p=[k[0], 0.0, k[2], 0.0, 0.0, k[4], k[5], 0.0, 0.0, 0.0, 1.0, 0.0])
        self.info.header.frame_id = 'camera_left_optical_frame'
        self.joints = (0.0, 0.0)
        self.tag, self.pos, self.det = {}, {}, set()
        self.pub_js = self.create_publisher(JointState, '/joint_states', 10)
        self.pub_img = self.create_publisher(Image, '/camera_left/image_raw', 5)
        self.pub_info = self.create_publisher(CameraInfo, '/camera_left/camera_info', 5)
        self.create_subscription(PointStamped, '/luna/dock/tag', lambda m: self._got(self.tag, m), 50)
        self.create_subscription(PointStamped, '/luna/dock/tag_position', lambda m: self._got(self.pos, m), 50)
        self.create_subscription(AprilTagDetectionArray, '/camera_left/detections', self._dets, 50)
        self.create_timer(0.02, self._js)

    def _dets(self, m):
        if any(d.id == G.tag_id for d in m.detections):
            self.det.add((m.header.stamp.sec, m.header.stamp.nanosec))

    @staticmethod
    def _got(store, m):
        store[(m.header.stamp.sec, m.header.stamp.nanosec)] = (m.point.x, m.point.y, m.point.z, time.monotonic())

    def _js(self):
        js = JointState(name=['head_pan', 'head_tilt'], position=list(self.joints))
        js.header.stamp = self.get_clock().now().to_msg()
        self.pub_js.publish(js)

    def show(self, img):
        stamp = self.get_clock().now().to_msg()
        msg = Image(height=self.model.height, width=self.model.width, encoding='mono8', step=self.model.width)
        msg.header.stamp, msg.header.frame_id = stamp, 'camera_left_optical_frame'
        msg.data = array.array('B', img.tobytes())        # bytes would be range-checked element by element
        self.info.header.stamp = stamp
        self.pub_info.publish(self.info)
        self.pub_img.publish(msg)
        return (stamp.sec, stamp.nanosec), time.monotonic()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--width', type=int, default=1280)
    ap.add_argument('--height', type=int, default=720)
    ap.add_argument('--frames', type=int, default=8)
    ap.add_argument('--out', default='')
    ap.add_argument('--groups', default='', help='comma-separated case groups to run (default: all)')
    a = ap.parse_args()
    import numpy as np

    with open(os.path.join(get_package_share_directory('luna_robot'), 'urdf', 'luna.urdf')) as f:
        chain = UrdfChain(f.read())
    model = Pinhole.from_hfov(a.width, a.height, 82.0)
    rclpy.init()
    node = Sweep(model)
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    time.sleep(2.0)
    rng = np.random.default_rng(7)
    rows = []
    groups = set(a.groups.split(',')) if a.groups else None
    for c in cases():
        if groups and c['group'] not in groups:
            continue
        joints = {'head_pan': math.radians(c['pan']), 'head_tilt': math.radians(c['tilt'])}
        node.joints = (joints['head_pan'], joints['head_tilt'])
        time.sleep(0.4)                       # robot_state_publisher has the new head pose in TF
        yaw = math.radians(c['yaw'])
        p_true = tag_in_base(-c['d'], c['y'], yaw)
        obs_true = observe(p_true) or (float('nan'), float('nan'))      # beyond max_range: no observation expected
        base_T_opt = chain.pose('base_link', 'camera_left_optical_frame', joints)
        opt_T_tag = (Pose.planar(-c['d'], c['y'], yaw) * base_T_opt).inverse() * dock_T_tag()
        p_opt = opt_T_tag.apply((0.0, 0.0, 0.0))
        expect = model.sees(p_opt, G.size_m * 5 / 8)          # whole tag incl. white ring inside the image
        sent = []
        for _ in range(a.frames):
            img = render(model, opt_T_tag, blur_px=c['blur'], noise=2.0, rng=rng)
            sent.append(node.show(img))
            time.sleep(0.1)
        time.sleep(0.6)
        hits = [(node.tag[s], node.pos.get(s), t0) for s, t0 in sent if s in node.tag]
        n_det = sum(s in node.det for s, _ in sent)
        row = dict(c, beklenen_gorunur=expect, apriltag_tespit=n_det, tespit=len(hits), kare=a.frames,
                   gercek_offset=obs_true[0], gercek_mesafe=obs_true[1], piksel_kenar=model.fx * G.size_m / p_opt[2])
        if hits:
            off = [h[0][0] for h in hits]
            dist = [h[0][2] for h in hits]
            perr = [math.dist(h[1][:3], p_true) for h in hits if h[1]]
            row.update(offset=sum(off) / len(off), mesafe=sum(dist) / len(dist),
                       offset_hata=max(abs(o - obs_true[0]) for o in off),
                       mesafe_hata=max(abs(x - obs_true[1]) for x in dist),
                       konum_hata_m=max(perr) if perr else None,
                       gecikme_ms=1000 * sum(h[0][3] - h[2] for h in hits) / len(hits))
        rows.append(row)
        print(f"{c['group']:9s} d={c['d']:+.3f} y={c['y']:+.2f} yaw={c['yaw']:+5.1f} pan={c['pan']:+5.1f} tilt={c['tilt']:+5.1f} "
              f"blur={c['blur']:.1f} kenar={row['piksel_kenar']:5.1f}px görünür={'E' if expect else 'H'} "
              f"apriltag={n_det}/{a.frames} yayın={len(hits)}/{a.frames}"
              + (f" offset {row['offset']:+.3f} (gerçek {obs_true[0]:+.3f}) mesafe {row['mesafe']:.3f} (gerçek {obs_true[1]:.3f}) "
                 f"konum hata {1000 * (row['konum_hata_m'] or 0):.1f} mm" if hits else ''), flush=True)
    if a.out:
        with open(a.out, 'w') as f:
            json.dump(dict(width=a.width, height=a.height, rows=rows), f, indent=1, ensure_ascii=False)
    rclpy.try_shutdown()            # ends spin() first; destroying the node under a live spin aborts the process
    spinner.join(timeout=2.0)
    node.destroy_node()


if __name__ == '__main__':
    main()
