"""World behind sim2d_node: a room as the LiDAR plane sees it (line segments), the wall-backed dock with its tag and
contacts, a lagged skid-steer base whose tracks slip more than wheel odometry assumes, bumpers and the pack.
Pure Python, so mapping, navigation and docking scenarios can be checked without ROS.

Frame 'world' is the start frame of the default room. With the robot starting at (0, 0, 0), slam_toolbox puts
the map origin there too, so map ≈ world (up to SLAM error)."""
import math
import random
from dataclasses import dataclass
from typing import NamedTuple

from . import kinematics as K
from .dock_nav import Pose2D, dock_frame, wrap
from .mock_plant import TAG_HALF_FOV

HALF_LENGTH = 0.170     # R06: 340 mm along x (track axles C = 192 mm + sprockets)
HALF_WIDTH = 0.189      # R06: track band |X| 141..189 mm → 378 mm across
LASER_X = -0.070        # urdf laser_joint; the scan plane (z 0.58 m) sees walls and tall furniture only
BUMPER_DEPTH = 0.03     # front/rear bumper strips: this deep from the face inwards...
BUMPER_REACH = 0.01     # ...and pressed this far before the hard contact


def box(x0, y0, x1, y1):
    return ((x0, y0, x1, y0), (x1, y0, x1, y1), (x1, y1, x0, y1), (x0, y1, x0, y0))


class Room(NamedTuple):
    walls: tuple            # (x0, y0, x1, y1) segments in the scan plane
    dock: Pose2D            # robot pose when docked, facing the wall


def default_room():
    """5 × 4 m room in the start frame: robot at the origin facing +x, a tall cabinet in the way, a pillar, and the
    dock in the middle of the north wall. The dock body (tag centre ≈ 0.38 m) is below the scan plane."""
    walls = box(-1.0, -2.0, 4.0, 2.0) + box(1.0, -0.8, 1.8, 0.9) + box(3.0, -1.4, 3.3, -1.1)
    return Room(walls, Pose2D(3.0, 1.75, math.pi / 2))


def _cross(ax, ay, bx, by):
    return ax * by - ay * bx


def raycast(ox, oy, angle, walls, max_range):
    """Distance along the ray to the nearest wall, inf if none within max_range."""
    dx, dy = math.cos(angle), math.sin(angle)
    best, hit = max_range, False
    for x0, y0, x1, y1 in walls:
        ex, ey = x1 - x0, y1 - y0
        den = _cross(dx, dy, ex, ey)
        if abs(den) < 1e-12:
            continue
        wx, wy = x0 - ox, y0 - oy
        t, u = _cross(wx, wy, ex, ey) / den, _cross(wx, wy, dx, dy) / den
        if 0.0 <= u <= 1.0 and 0.0 < t <= best:
            best, hit = t, True
    return best if hit else math.inf


def _segment_blocked(ax, ay, bx, by, walls):
    dx, dy = bx - ax, by - ay
    for x0, y0, x1, y1 in walls:
        ex, ey = x1 - x0, y1 - y0
        den = _cross(dx, dy, ex, ey)
        if abs(den) < 1e-12:
            continue
        wx, wy = x0 - ax, y0 - ay
        t, u = _cross(wx, wy, ex, ey) / den, _cross(wx, wy, dx, dy) / den
        if 0.0 <= u <= 1.0 and 0.0 < t < 1.0:
            return True
    return False


def _touches_box(p0, p1, xmin, xmax, ymin, ymax):
    """Liang–Barsky: does segment p0-p1 reach into the axis-aligned box?"""
    t0, t1 = 0.0, 1.0
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    for p, q in ((-dx, p0[0] - xmin), (dx, xmax - p0[0]), (-dy, p0[1] - ymin), (dy, ymax - p0[1])):
        if p == 0:
            if q < 0:
                return False
        elif p < 0:
            t0 = max(t0, q / p)
        else:
            t1 = min(t1, q / p)
        if t0 > t1:
            return False
    return True


@dataclass(frozen=True)
class SimConfig:
    # Same physics as core/mock_plant.PlantConfig
    start_soc: float = 0.6
    drain_w: float = 44.0
    charge_w: float = 50.0
    pack_wh: float = 170.0
    motor_tau_s: float = 0.15
    dock_y_tol_m: float = 0.02        # lateral misalignment the receiver mouth still accepts
    contact_travel_m: float = 0.003
    # 2D world
    dock_yaw_tol: float = 0.2         # rad the receiver funnel accepts (not measured on the real dock)
    dock_capture_m: float = 0.25      # dock body half width + margin: hard stop inside this lateral band
    true_turn_slip: float = 1.42      # tracks really skid more than base_node's turn_slip (1.35) assumes
    linear_slip: float = 0.02         # and lose this fraction of travel: wheel odometry over-reports
    beams: int = 500                  # RPLIDAR C1: 5 kHz samples at 10 Hz
    range_max: float = 12.0
    range_noise_m: float = 0.01
    tag_range_m: float = 3.0          # AprilTag still decoded by the head camera
    tag_max_incidence: float = math.radians(60)
    corner_range_m: float = 4.0       # VL53L1X long mode
    seed: int = 1


class Sim2D:
    def __init__(self, room, cfg=SimConfig(), start=Pose2D(0.0, 0.0, 0.0)):
        self.room, self.cfg = room, cfg
        self.pose = start
        self.target = [0.0, 0.0]
        self.vel = [0.0, 0.0]
        self.pos = [0.0, 0.0]
        self.wh = cfg.pack_wh * cfg.start_soc
        self.bumpers = 0
        self.docked = False
        self.rng = random.Random(cfg.seed)

    @property
    def charging(self):
        return self.docked

    @property
    def soc(self):
        return self.wh / self.cfg.pack_wh

    @property
    def twist(self):
        """True body velocity (m/s, rad/s) of the tracks, slip included."""
        vl, vr = (v * K.SPROCKET_RADIUS * (1.0 - self.cfg.linear_slip) for v in self.vel)
        return (vl + vr) / 2, (vr - vl) / (K.TRACK_GAUGE * self.cfg.true_turn_slip)

    def _local(self, pose, x, y):
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        dx, dy = x - pose.x, y - pose.y
        return dx * c + dy * s, -dx * s + dy * c

    def _touches(self, pose, xmin, xmax, ymin, ymax):
        for x0, y0, x1, y1 in self.room.walls:
            if _touches_box(self._local(pose, x0, y0), self._local(pose, x1, y1), xmin, xmax, ymin, ymax):
                return True
        return False

    def collides(self, pose):
        return self._touches(pose, -HALF_LENGTH, HALF_LENGTH, -HALF_WIDTH, HALF_WIDTH)

    def _wall_bumpers(self):
        L, W, d, e = HALF_LENGTH, HALF_WIDTH, BUMPER_DEPTH, BUMPER_REACH
        zones = ((L - d, L + e, 0.0, W), (L - d, L + e, -W, 0.0), (-L - e, -L + d, 0.0, W), (-L - e, -L + d, -W, 0.0))
        return sum(1 << i for i, z in enumerate(zones) if self._touches(self.pose, *z))

    def step(self, dt):
        c = self.cfg
        prev = list(self.pos)
        for i in range(2):
            self.vel[i] += (self.target[i] - self.vel[i]) * min(1.0, dt / c.motor_tau_s)
            self.pos[i] += self.vel[i] * dt
        dl, dr = ((self.pos[i] - prev[i]) * K.SPROCKET_RADIUS * (1.0 - c.linear_slip) for i in range(2))
        ds, dyaw = (dl + dr) / 2, (dr - dl) / (K.TRACK_GAUGE * c.true_turn_slip)
        p, mid = self.pose, self.pose.yaw + dyaw / 2
        cand = Pose2D(p.x + ds * math.cos(mid), p.y + ds * math.sin(mid), wrap(p.yaw + dyaw))
        along, lateral, _ = dock_frame(cand, self.room.dock)
        if (along > 0.0 and abs(lateral) < c.dock_capture_m) or self.collides(cand):
            self.pos, self.vel = prev, [0.0, 0.0]          # tracks stall against the wall or the dock nose
        else:
            self.pose = cand
        along, lateral, dyaw_err = dock_frame(self.pose, self.room.dock)
        at_dock = abs(lateral) < c.dock_capture_m
        aligned = abs(lateral) < c.dock_y_tol_m and abs(dyaw_err) < c.dock_yaw_tol
        self.docked = at_dock and aligned and along >= -c.contact_travel_m
        nose_bump = at_dock and not aligned and along >= -BUMPER_REACH    # dock nose hits the face beside the mouth
        self.bumpers = self._wall_bumpers() | (0b0011 if nose_bump else 0)
        self.wh = min(c.pack_wh, self.wh + (c.charge_w if self.charging else -c.drain_w) * dt / 3600)

    def scan(self):
        """RPLIDAR C1 ranges (m, inf = no return), angle_min = -pi, increment 2*pi/beams, in the laser frame."""
        c, p = self.cfg, self.pose
        lx, ly = p.x + LASER_X * math.cos(p.yaw), p.y + LASER_X * math.sin(p.yaw)
        inc = 2 * math.pi / c.beams
        out = []
        for i in range(c.beams):
            r = raycast(lx, ly, p.yaw - math.pi + i * inc, self.room.walls, c.range_max)
            if r != math.inf and c.range_noise_m:
                r += self.rng.gauss(0.0, c.range_noise_m)
            out.append(r)
        return out

    def corners(self):
        """VL53L1X ranges [FL, FR, BL, BR], each looking diagonally outwards from its corner."""
        p, out = self.pose, []
        for sx, sy in ((1, 1), (1, -1), (-1, 1), (-1, -1)):
            lx, ly = sx * HALF_LENGTH, sy * HALF_WIDTH
            ox = p.x + lx * math.cos(p.yaw) - ly * math.sin(p.yaw)
            oy = p.y + lx * math.sin(p.yaw) + ly * math.cos(p.yaw)
            r = raycast(ox, oy, p.yaw + math.atan2(sy, sx), self.room.walls, self.cfg.corner_range_m)
            out.append(min(r, self.cfg.corner_range_m))
        return out

    def tag(self):
        """(offset -1..1, distance m) like core/mock_plant.DockPlant.tag, or None when the head camera can't see it."""
        c, d = self.cfg, self.room.dock
        along, lateral, dyaw = dock_frame(self.pose, d)
        dist = -along
        if dist < -0.01 or math.hypot(along, lateral) > c.tag_range_m:
            return None
        if dist > 1e-3 and math.atan2(abs(lateral), dist) > c.tag_max_incidence:
            return None                                    # tag seen too obliquely to decode
        bearing = math.atan2(-lateral, max(1e-3, dist)) - dyaw
        if abs(bearing) >= TAG_HALF_FOV or _segment_blocked(self.pose.x, self.pose.y, d.x, d.y, self.room.walls):
            return None
        return -bearing / TAG_HALF_FOV, max(0.0, dist)
