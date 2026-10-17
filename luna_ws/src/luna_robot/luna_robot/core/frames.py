"""Rigid transforms and a URDF joint chain, so ROS-free code and tests use exactly the frames robot_state_publisher
publishes from urdf/luna.urdf. Rotations are 3×3 row tuples; poses compose like TF: `a_T_b * b_T_c = a_T_c`."""
import math
import xml.etree.ElementTree as ET


def _mul(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)) for i in range(3))


def _apply(r, v):
    return tuple(sum(r[i][k] * v[k] for k in range(3)) for i in range(3))


def _transpose(r):
    return tuple(tuple(r[j][i] for j in range(3)) for i in range(3))


def rot_rpy(roll, pitch, yaw):
    """URDF/TF fixed-axis roll-pitch-yaw: R = Rz(yaw) · Ry(pitch) · Rx(roll)."""
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return ((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr))


def rot_axis(axis, angle):
    """Rotation by `angle` rad about the unit vector `axis` (Rodrigues)."""
    n = math.sqrt(sum(a * a for a in axis))
    x, y, z = (a / n for a in axis)
    c, s, C = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    return ((c + x * x * C, x * y * C - z * s, x * z * C + y * s),
            (y * x * C + z * s, c + y * y * C, y * z * C - x * s),
            (z * x * C - y * s, z * y * C + x * s, c + z * z * C))


def quat_to_rot(x, y, z, w):
    return ((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
            (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)))


IDENTITY = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


class Pose:
    """Rigid transform a_T_b: maps points written in frame b into frame a (TF lookup_transform(a, b))."""

    def __init__(self, r=IDENTITY, t=(0.0, 0.0, 0.0)):
        self.r, self.t = r, tuple(t)

    def __mul__(self, other):
        return Pose(_mul(self.r, other.r), tuple(a + b for a, b in zip(_apply(self.r, other.t), self.t)))

    def apply(self, p):
        return tuple(a + b for a, b in zip(_apply(self.r, p), self.t))

    def inverse(self):
        rt = _transpose(self.r)
        return Pose(rt, tuple(-v for v in _apply(rt, self.t)))

    @staticmethod
    def planar(x, y, yaw, z=0.0):
        return Pose(rot_rpy(0.0, 0.0, yaw), (x, y, z))


class UrdfChain:
    """Fixed and revolute joints of a URDF. `pose(a, b, joints)` is what TF would return for lookup_transform(a, b)
    with robot_state_publisher fed `joints` ({joint name: rad}; missing revolute joints count as 0)."""

    def __init__(self, urdf_xml):
        self.parent = {}                    # child link -> (joint name, type, parent link, origin Pose, axis)
        for j in ET.fromstring(urdf_xml).iter('joint'):
            o = j.find('origin')
            xyz = [float(v) for v in (o.get('xyz', '0 0 0') if o is not None else '0 0 0').split()]
            rpy = [float(v) for v in (o.get('rpy', '0 0 0') if o is not None else '0 0 0').split()]
            a = j.find('axis')
            axis = [float(v) for v in a.get('xyz').split()] if a is not None else [1.0, 0.0, 0.0]
            child = j.find('child').get('link')
            self.parent[child] = (j.get('name'), j.get('type'), j.find('parent').get('link'), Pose(rot_rpy(*rpy), xyz), axis)

    def _from_root(self, link, joints):
        pose = Pose()
        while link in self.parent:
            name, kind, parent, origin, axis = self.parent[link]
            step = origin
            if kind in ('revolute', 'continuous'):
                step = origin * Pose(rot_axis(axis, joints.get(name, 0.0)))
            elif kind != 'fixed':
                raise ValueError(f'joint type {kind} not supported: {name}')
            pose = step * pose
            link = parent
        return pose

    def pose(self, a, b, joints=None):
        joints = joints or {}
        return self._from_root(a, joints).inverse() * self._from_root(b, joints)
