"""Simulated Gemini 336 colour camera for `mock:=true tags:=sim`: renders what it would see of the dock tag and
publishes it the way OrbbecSDK_ROS2 does (/camera/color/image_raw + camera_info, frame camera_color_optical_frame),
so apriltag_ros and dock_tag_node run exactly as on the robot.

Robot pose: the mock's ground truth (/luna/mock/truth, dock frame). Camera pose on the robot: TF base_link →
camera_color_optical_frame (the URDF). Images: core/tag_scene.py (numpy)."""
import array
import os

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from . import topics as T
from .entry import run
from .core.camera_model import Pinhole
from .core.frames import Pose, quat_to_rot
from .core.tag_scene import dock_T_tag, render

FRAME = 'camera_color_optical_frame'


class SimCamera(Node):
    def __init__(self):
        super().__init__('luna_sim_camera')
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('noise', 2.0)            # grey levels, sensor noise sigma
        self.declare_parameter('blur_px', 0.0)          # defocus sigma, px
        p = self.get_parameter
        share = get_package_share_directory('luna_robot')
        with open(os.path.join(share, 'config', 'camera_info', 'color_camera.yaml')) as f:
            info = yaml.safe_load(f)
        self.model, self.info = Pinhole.from_camera_info(info), self._info_msg(info)
        self.pub_img = self.create_publisher(Image, T.COLOR_IMAGE, 5)
        self.pub_info = self.create_publisher(CameraInfo, '/camera/color/camera_info', 5)
        self.blur = p('blur_px').value
        rng = np.random.default_rng(1)
        m = self.model
        self.noise = [rng.normal(0.0, p('noise').value, (m.height, m.width)).astype(np.float32) for _ in range(4)]
        self.frame = 0                                  # a few noise frames reused in turn: fresh noise costs ~16 ms
        self.truth = None
        self.tag = dock_T_tag()
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.create_subscription(PoseStamped, T.MOCK_TRUTH, lambda m: setattr(self, 'truth', m.pose), 10)
        self.create_timer(1.0 / p('rate_hz').value, self._tick)

    @staticmethod
    def _info_msg(info):
        m = CameraInfo()
        m.header.frame_id = FRAME
        m.width, m.height = int(info['image_width']), int(info['image_height'])
        m.distortion_model = info['distortion_model']
        m.d = [float(v) for v in info['distortion_coefficients']['data']]
        m.k = [float(v) for v in info['camera_matrix']['data']]
        m.r = [float(v) for v in info['rectification_matrix']['data']]
        m.p = [float(v) for v in info['projection_matrix']['data']]
        return m

    def _tick(self):
        if self.truth is None:
            return
        try:
            tf = self.tf.lookup_transform('base_link', FRAME, Time())
        except TransformException:
            return                                      # robot_state_publisher not up yet
        q, t = self.truth.orientation, self.truth.position
        dock_T_base = Pose(quat_to_rot(q.x, q.y, q.z, q.w), (t.x, t.y, t.z))
        r, o = tf.transform.rotation, tf.transform.translation
        dock_T_opt = dock_T_base * Pose(quat_to_rot(r.x, r.y, r.z, r.w), (o.x, o.y, o.z))
        self.frame += 1
        img = render(self.model, dock_T_opt.inverse() * self.tag, blur_px=self.blur,
                     noise=self.noise[self.frame % len(self.noise)])
        stamp = self.get_clock().now().to_msg()
        msg = Image()
        msg.header.stamp, msg.header.frame_id = stamp, FRAME
        msg.height, msg.width, msg.encoding, msg.step = self.model.height, self.model.width, 'mono8', self.model.width
        msg.data = array.array('B', img.tobytes())        # bytes would be range-checked element by element (~0.3 s/frame)
        self.info.header.stamp = stamp
        self.pub_img.publish(msg)
        self.pub_info.publish(self.info)


def main():
    run(SimCamera)


if __name__ == '__main__':
    main()
