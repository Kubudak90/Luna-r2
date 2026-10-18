"""Sensor poses on Luna's body, read from urdf/luna.urdf. The URDF is the only place a mount pose is written down
(LiDAR, depth camera, corner ToF): robot_state_publisher puts it on TF for Nav2, and the simulator, dock_nav_node
and the tests read the same numbers through this module. Pure Python (xml + 3×3 lists), so it runs without ROS.

A pose is (t, R): a point p in the frame is R·p + t in base_link. Movable joints count at their zero position."""
import math
import os
import xml.etree.ElementTree as ET
from functools import lru_cache

IDENTITY = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def rpy_matrix(roll, pitch, yaw):
    """URDF rpy: fixed-axis roll about x, then pitch about y, then yaw about z (R = Rz·Ry·Rx)."""
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw),
                              math.sin(yaw))
    return ((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr))


def matmul(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)) for i in range(3))


def apply(r, v):
    return tuple(r[i][0] * v[0] + r[i][1] * v[1] + r[i][2] * v[2] for i in range(3))


def _floats(text, default):
    return tuple(float(v) for v in text.split()) if text else default


class Frames:
    """The URDF joint tree, child link → (parent link, xyz, R)."""

    def __init__(self, urdf_text):
        self.joints = {}
        for j in ET.fromstring(urdf_text).iter('joint'):
            origin = j.find('origin')
            xyz = _floats(origin.get('xyz') if origin is not None else None, (0.0, 0.0, 0.0))
            rpy = _floats(origin.get('rpy') if origin is not None else None, (0.0, 0.0, 0.0))
            self.joints[j.find('child').get('link')] = (j.find('parent').get('link'), xyz, rpy_matrix(*rpy))

    def pose(self, frame, base='base_link'):
        t, r = (0.0, 0.0, 0.0), IDENTITY
        while frame != base:
            if frame not in self.joints:
                raise KeyError(f'{frame} is not below {base} in the URDF')
            parent, xyz, rj = self.joints[frame]
            t = tuple(a + b for a, b in zip(apply(rj, t), xyz))
            r = matmul(rj, r)
            frame = parent
        return t, r

    def origin(self, frame):
        return self.pose(frame)[0]

    def yaw(self, frame):
        """Heading of the frame's x axis in base_link (rad)."""
        r = self.pose(frame)[1]
        return math.atan2(r[1][0], r[0][0])


def default_urdf_path():
    """urdf/luna.urdf of the source tree (tests, --symlink-install), else of the installed package."""
    src = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', 'urdf', 'luna.urdf')
    if os.path.exists(src):
        return os.path.normpath(src)
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('luna_robot'), 'urdf', 'luna.urdf')


@lru_cache(maxsize=None)
def load_frames(path=None):
    with open(path or default_urdf_path()) as f:
        return Frames(f.read())
