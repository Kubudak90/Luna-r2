"""Dock AprilTag → the observation dock_node reads on /luna/dock/tag (PointStamped x = offset, z = distance).

Tag: tag36h11 ID 0, black square 48 mm (8 cells × 6 mm) plus its 1-cell white ring = 60 mm, centred on the
64 mm REF_R02_TAG plate (2 mm left for cutting). apriltag's tag `size` is the black-square edge, 0.048.

R02 CAD in the docked pose, in ROS base_link axes (x = −Y_cad, y = X_cad, z = Z_cad):
  tag face   x 0.371 (wall plane Y−376, 5 mm plate), y 0, centre z 0.382
  mouth      x 0.170 (receiver mouth Y−170)
  nose tip   x 0.154 (Y−154): 0.217 m in front of the tag face, 16 mm inside the mouth when seated.
The node gets the tag centre in base_link through TF (camera optical frame → head tilt → head pan → base_link),
so head pan/tilt are already taken out: offset and distance are relative to the body however the head is turned."""
import math
from dataclasses import dataclass

TAG_FACE_X_DOCKED = 0.371


@dataclass(frozen=True)
class TagGeometry:
    tag_id: int = 0
    size_m: float = 0.048             # black-square edge = apriltag `size`
    height_m: float = 0.382           # tag centre above the floor (base_link z = 0 on the floor)
    height_tol_m: float = 0.10        # a "tag" further off this height is another tag or a wrong pose
    mouth_x_m: float = 0.170
    nose_to_tag_m: float = 0.217
    half_angle_deg: float = 35.0      # body bearing reported as |offset| = 1 (Dock.k_turn and the mock use it)
    min_range_m: float = 0.25         # seated, the tag is 0.371 m from base_link; nothing real is nearer
    max_range_m: float = 3.0


def observe(p, g=TagGeometry()):
    """Tag centre p = (x, y, z) in base_link → (offset, distance), or None when p can't be the dock tag.

    offset: bearing of the tag from base_link (the turning centre), right positive like the image x, scaled so
            ±half_angle_deg is ±1 and clamped there.
    distance: receiver mouth to nose tip, taking the nose to lie nose_to_tag_m in front of the tag on the line to
            the mouth; exact when lined up with the dock. 0 when the nose reaches the mouth, −0.016 seated."""
    x, y, z = p
    if not all(math.isfinite(v) for v in p) or abs(z - g.height_m) > g.height_tol_m:
        return None
    if not g.min_range_m <= math.hypot(x, y) <= g.max_range_m:
        return None
    offset = -math.atan2(y, x) / math.radians(g.half_angle_deg)
    return max(-1.0, min(1.0, offset)), math.hypot(x - g.mouth_x_m, y) - g.nose_to_tag_m


def tag_in_base(robot_x, robot_y, robot_yaw, g=TagGeometry()):
    """Tag centre in base_link for a robot pose in the dock frame (origin = seated base_link pose, x towards the
    wall). Used by the mock/sim and the tests."""
    dx, dy = TAG_FACE_X_DOCKED - robot_x, -robot_y
    c, s = math.cos(robot_yaw), math.sin(robot_yaw)
    return c * dx + s * dy, -s * dx + c * dy, g.height_m


class TagFuser:
    """Latest tag centre (base_link) per camera. Both head cameras usually see the tag; fresh, plausible ones are
    averaged. When they disagree by more than max_spread_m one pose is wrong, and the one nearer the known tag
    height is kept."""

    def __init__(self, g=TagGeometry(), max_age_s=0.3, max_spread_m=0.08):
        self.g, self.max_age_s, self.max_spread_m = g, max_age_s, max_spread_m
        self.seen = {}

    def add(self, camera, stamp_s, p):
        self.seen[camera] = (stamp_s, tuple(p))

    def position(self, now_s):
        fresh = [p for t, p in self.seen.values() if -self.max_age_s <= now_s - t <= self.max_age_s and observe(p, self.g)]
        if not fresh:
            return None
        if len(fresh) > 1 and max(math.dist(a, b) for a in fresh for b in fresh) > self.max_spread_m:
            return min(fresh, key=lambda p: abs(p[2] - self.g.height_m))
        return tuple(sum(c) / len(fresh) for c in zip(*fresh))

    def observation(self, now_s):
        p = self.position(now_s)
        return observe(p, self.g) if p else None
