"""Dock AprilTag → the observation dock_node reads on /luna/dock/tag (PointStamped x = offset, z = distance).

Tag: tag36h11 ID 0, black square 80 mm (8 cells × 10 mm) plus its 1-cell white ring = 100 mm, centred on a 104 mm
plate on the dock's tag mast. apriltag's tag `size` is the black-square edge, 0.080. R1's 48 mm tag was bounded by its
plate; the R2 mast leaves room for a larger one, which decodes from further away and gives a steadier pose.

R2 dock (docs/TASARIM_R2.md, "Dock"), in the seated pose in base_link axes: the tag face is 0.45 m straight ahead of
the turning centre, its centre 0.25 m above the floor (the Gemini 336 colour camera is at Z270, 15° nose down, so
the tag stays in its 55° vertical field down to the seat). Under the robot the dock's spring pins meet copper strips;
a wheel funnel centres the robot and wheel stops end the drive at the seat.
The node gets the tag centre in base_link through TF (camera optical frame → camera_link → base_link)."""
import math
from dataclasses import dataclass

TAG_FACE_X_DOCKED = 0.45


@dataclass(frozen=True)
class TagGeometry:
    tag_id: int = 0
    size_m: float = 0.080             # black-square edge = apriltag `size`
    plate_m: float = 0.104
    height_m: float = 0.25            # tag centre above the floor (base_link z = 0 on the floor)
    height_tol_m: float = 0.10        # a "tag" further off this height is another tag or a wrong pose
    seat_m: float = TAG_FACE_X_DOCKED  # tag face ahead of base_link when seated
    half_angle_deg: float = 35.0      # body bearing reported as |offset| = 1 (Dock.k_turn and the mock use it)
    min_range_m: float = 0.30         # seated, the tag is 0.45 m from base_link; nothing real is nearer
    max_range_m: float = 4.0


def observe(p, g=TagGeometry()):
    """Tag centre p = (x, y, z) in base_link → (offset, distance), or None when p can't be the dock tag.

    offset: bearing of the tag from base_link (the turning centre), right positive like the image x, scaled so
            ±half_angle_deg is ±1 and clamped there.
    distance: how far the robot still has to drive to the seat, i.e. its distance from the tag minus seat_m;
            exact when lined up with the dock, 0 at the wheel stops."""
    x, y, z = p
    if not all(math.isfinite(v) for v in p) or abs(z - g.height_m) > g.height_tol_m:
        return None
    if not g.min_range_m <= math.hypot(x, y) <= g.max_range_m:
        return None
    offset = -math.atan2(y, x) / math.radians(g.half_angle_deg)
    return max(-1.0, min(1.0, offset)), math.hypot(x, y) - g.seat_m


def tag_in_base(robot_x, robot_y, robot_yaw, g=TagGeometry()):
    """Tag centre in base_link for a robot pose in the dock frame (origin = seated base_link pose, x towards the
    tag). Used by the mock/sim and the tests."""
    dx, dy = g.seat_m - robot_x, -robot_y
    c, s = math.cos(robot_yaw), math.sin(robot_yaw)
    return c * dx + s * dy, -s * dx + c * dy, g.height_m


class TagFuser:
    """Latest tag centre (base_link) per camera. R2 reads the tag with one camera (the Gemini colour camera); a
    second one would be averaged with it. When two disagree by more than max_spread_m one pose is wrong, and the one
    nearer the known tag height is kept."""

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
