"""Sensor mount poses on Luna's body, read from urdf/luna.urdf through core/frames.UrdfChain. The URDF is the only
place a mount pose is written down (LiDAR, depth camera, corner ToF): robot_state_publisher puts it on TF for Nav2,
and the simulator, dock_nav_node and the tests read the same numbers through load_frames(). Pure Python.

A pose is (t, R) of the frame in base_link: a point p in the frame is R·p + t. Movable joints count at zero."""
import math
import os
from functools import lru_cache

from .frames import UrdfChain


class Frames:
    def __init__(self, urdf_text):
        self.chain = UrdfChain(urdf_text)
        self.links = set(self.chain.parent) | {j[2] for j in self.chain.parent.values()}

    def pose(self, frame, base='base_link'):
        for link in (frame, base):
            if link not in self.links:
                raise KeyError(f'{link} is not a link of the URDF')
        p = self.chain.pose(base, frame)
        return p.t, p.r

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
