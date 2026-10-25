"""Face camera → /luna/vision/face (Vector3: x = bearing deg in base_link, y = elevation deg, z = face width px), only
while a face is seen. Logic in core/face_target.py.

Camera: the Raspberry Pi Camera v2 (IMX219) on the Jetson's CSI port, read through GStreamer (nvarguscamerasrc) with
device:=auto, or any V4L2 device path / index for a USB camera. Detector: OpenCV's FaceDetectorYN (YuNet, OpenCV ≥
4.5.4) with the ONNX model at `model` (opencv_zoo face_detection_yunet_2023mar.onnx, see README_TR.md). Without
OpenCV, the model or the camera the node logs why and publishes nothing: Luna's eyes then wander by themselves.

Images stay in memory: nothing is recorded, nothing leaves the robot. With the privacy slider closed the lens is
covered and the detector simply finds no faces."""
import os
import threading
import time

import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Vector3
from rclpy.node import Node

from . import topics as T
from .entry import run
from .core.camera_model import Pinhole
from .core.face_target import FaceCamera, FaceSelector, SelectorConfig
from .core.robot_model import load_frames

ARGUS = ('nvarguscamerasrc sensor-id={sensor} ! video/x-raw(memory:NVMM),width=1640,height=1232,framerate=30/1 ! '
         'nvvidconv ! video/x-raw,width={w},height={h},format=BGRx ! videoconvert ! video/x-raw,format=BGR ! '
         'appsink drop=true max-buffers=1')


class FaceTrackNode(Node):
    def __init__(self):
        super().__init__('luna_face_track')
        for name, default in [('device', 'auto'), ('sensor_id', 0), ('width', 640), ('height', 480), ('rate_hz', 10.0),
                              ('model', '/models/face/face_detection_yunet_2023mar.onnx'), ('min_score', 0.75),
                              ('min_width_px', 24.0), ('camera_info', 'face_camera.yaml')]:
            self.declare_parameter(name, default)
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        with open(os.path.join(get_package_share_directory('luna_robot'), 'config', 'camera_info', p('camera_info'))) as f:
            pinhole = Pinhole.from_camera_info(yaml.safe_load(f))
        rot = load_frames().pose('face_camera_optical_frame')[1]
        self.selector = FaceSelector(FaceCamera(pinhole, rot),
                                     SelectorConfig(float(p('min_score')), float(p('min_width_px'))))
        self.pub = self.create_publisher(Vector3, T.VISION_FACE, 10)
        self.args = (p('device'), int(p('sensor_id')), pinhole.width, pinhole.height, float(p('rate_hz')), p('model'))
        threading.Thread(target=self._run, daemon=True).start()

    def _open(self, cv2, device, sensor, w, h):
        if device == 'auto':
            cap = cv2.VideoCapture(ARGUS.format(sensor=sensor, w=w, h=h), cv2.CAP_GSTREAMER)
            if cap.isOpened():
                return cap
            device = 0
        cap = cv2.VideoCapture(int(device) if str(device).isdigit() else device)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        return cap if cap.isOpened() else None

    def _run(self):
        device, sensor, w, h, rate, model = self.args
        try:
            import cv2
        except ImportError:
            self.get_logger().error('OpenCV (python3-opencv) is not installed: no face tracking')
            return
        if not hasattr(cv2, 'FaceDetectorYN') or not os.path.isfile(model):
            self.get_logger().error(f'no face detector: OpenCV {cv2.__version__} needs ≥ 4.5.4 and the YuNet model '
                                    f'at {model}')
            return
        detector = cv2.FaceDetectorYN.create(model, '', (w, h), score_threshold=self.selector.cfg.min_score)
        while True:
            cap = self._open(cv2, device, sensor, w, h)
            if cap is None:
                self.get_logger().warning(f'face camera {device} not available; retrying in 5 s',
                                          throttle_duration_sec=60.0)
                time.sleep(5.0)
                continue
            self.get_logger().info(f'face camera {device} open, {w}×{h} at {rate:.0f} Hz')
            period, t = 1.0 / rate, time.monotonic()
            while True:
                ok, frame = cap.read()
                if not ok:
                    break
                if frame.shape[1] != w or frame.shape[0] != h:
                    frame = cv2.resize(frame, (w, h))
                _, faces = detector.detect(frame)
                dets = [] if faces is None else [(f[0] + f[2] / 2, f[1] + f[3] / 2, f[2], f[3], f[14]) for f in faces]
                face = self.selector.update(dets)
                if face is not None:
                    self.pub.publish(Vector3(x=face.bearing_deg, y=face.elevation_deg, z=float(face.width_px)))
                t += period
                time.sleep(max(0.0, t - time.monotonic()))
                t = max(t, time.monotonic() - period)
            cap.release()
            self.get_logger().warning('face camera stopped delivering frames; reopening')


def main():
    run(FaceTrackNode)


if __name__ == '__main__':
    main()
