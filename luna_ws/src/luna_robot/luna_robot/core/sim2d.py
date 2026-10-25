"""World behind sim2d_node: a room with walls, furniture and the R2 dock (wheel funnel and stops, spring pins, tag
mast), a lagged differential-drive base whose wheels slip a little more than odometry assumes, the bumper ring and the
pack. Pure Python, so mapping, navigation, docking and obstacle scenarios can be checked without ROS.

2.5D: walls are full-height line segments; every other obstacle is an axis-aligned Block or a vertical Cylinder
with its own height band z0..z1. A sensor sees what its rays meet in 3D: the LiDAR's horizontal plane (z of the
urdf laser frame) only meets obstacles that span it; the corner ToF cones (and the depth camera,
core/depth_camera.py) also see low obstacles, table tops and the floor. Contact uses the robot's height bands
(robot_bands, docs/TASARIM_R2.md): R2 is round, so most bands are discs about the turning centre. Only the bumper
ring (Z15–110, all the way round) reports a touch on /luna/mcu/bumpers; any other contact (a cable under the ring,
a table top against the head) goes unnoticed by the robot and is logged as a crash.

Frame 'world' is the start frame of the default room. With the robot starting at (0, 0, 0), slam_toolbox puts
the map origin there too, so map ≈ world (up to SLAM error)."""
import math
import random
from dataclasses import dataclass
from typing import NamedTuple

from . import kinematics as K
from .dock_nav import Pose2D, dock_frame, wrap
from .dock_tag import TagGeometry, tag_in_base
from .frames import Pose
from .mock_plant import in_colour_view
from .robot_model import load_frames
from .tof import FRAMES as TOF_FRAMES

ROBOT_RADIUS = 0.180    # bumper ring outer surface = the robot's outline (urdf base_link)
BUMPER_TRAVEL = 0.010   # the ring gives this much before the hard stop behind it
BUMPER_Z = (0.015, 0.110)
BUMPER_REACH = 0.010    # the ring's switches close this far out, before anything is hard against the robot
LASER_X, _, LASER_Z = load_frames().origin('laser')   # urdf laser_joint: the scan plane sees only what spans it
LASER_YAW = load_frames().yaw('laser')                # −90°, the C1's +x points to the robot's right
C1_HEIGHT = 0.0413      # RPLIDAR C1: 41.3 mm tall, laser plane 29.8 mm above its base
C1_BASE_Z = 0.520       # top of the head
WALL_HEIGHT = 2.5
DOCK_MAST = (0.45, 0.48, 0.06, 0.36)   # tag mast in the dock frame: x from, x to, half width, height


def beam_toward(angle, beams=500, laser_yaw=None):
    """Index of the scan beam (angle_min = −π, laser frame) that points at `angle` in the base frame."""
    a = angle - (LASER_YAW if laser_yaw is None else laser_yaw) + math.pi
    return round(a / (2 * math.pi / beams)) % beams


def box(x0, y0, x1, y1):
    return ((x0, y0, x1, y0), (x1, y0, x1, y1), (x1, y1, x0, y1), (x0, y1, x0, y0))


class Block(NamedTuple):
    """Axis-aligned box in the world frame."""
    name: str
    x0: float
    y0: float
    x1: float
    y1: float
    z0: float
    z1: float
    contact: bool = True     # False: only the sensors see it (the dock plate; the dock capture models its contact)


class Cylinder(NamedTuple):
    """Vertical cylinder (table or chair leg, ball, pillar)."""
    name: str
    x: float
    y: float
    r: float
    z0: float
    z1: float
    contact: bool = True


class Band(NamedTuple):
    """A part of the robot as a box in base_link."""
    name: str
    x0: float
    x1: float
    y0: float
    y1: float
    z0: float
    z1: float


class Disc(NamedTuple):
    """A part of the robot as a vertical cylinder in base_link."""
    name: str
    x: float
    y: float
    r: float
    z0: float
    z1: float


class Contact(NamedTuple):
    t: float                 # sim time s
    name: str                # obstacle ('wall' for walls)
    kind: str                # 'bumper' (the robot felt it) or 'crash' (it did not)
    bands: tuple             # robot parts in hard contact


class Room(NamedTuple):
    walls: tuple            # (x0, y0, x1, y1) full-height segments
    dock: Pose2D            # robot pose when docked, facing the tag mast
    obstacles: tuple = ()   # Block / Cylinder


def robot_bands(frames=None):
    """Luna R2's envelope by height (docs/TASARIM_R2.md). The LiDAR follows its urdf frame."""
    frames = frames or load_frames()
    lx, ly, _ = frames.origin('laser')
    return (
        Disc('chassis', 0.0, 0.0, 0.165, 0.0, BUMPER_Z[0]),                        # wheels, casters, belly plate
        Disc('bumper', 0.0, 0.0, ROBOT_RADIUS - BUMPER_TRAVEL, *BUMPER_Z),         # the ring, fully pressed
        Disc('body', 0.0, 0.0, 0.170, BUMPER_Z[1], 0.345),                         # torso shell, depth camera window
        Disc('neck', 0.0, 0.0, 0.040, 0.345, 0.360),
        Band('head', -0.050, 0.070, -0.110, 0.110, 0.360, C1_BASE_Z),              # fixed head with the screen
        Disc('lidar', lx, ly, 0.028, C1_BASE_Z, C1_BASE_Z + C1_HEIGHT),            # RPLIDAR C1 on the head
    )


def dock_parts(dock):
    """The R2 dock for the sensors: the low base plate (rolled over, sensors only) and the tag mast against the wall
    behind it (a real obstacle: a robot that overruns the seat by 0.27 m bumps into it). Axis along x or y only."""
    c, s = round(math.cos(dock.yaw)), round(math.sin(dock.yaw))
    if abs(c) + abs(s) != 1:
        raise ValueError('dock axis must be along x or y')

    def block(name, a0, a1, half, z1, contact):
        xs = (dock.x + c * a0, dock.x + c * a1) if c else (dock.x - half, dock.x + half)
        ys = (dock.y + s * a0, dock.y + s * a1) if s else (dock.y - half, dock.y + half)
        return Block(name, min(xs), min(ys), max(xs), max(ys), 0.0, z1, contact=contact)

    m0, m1, mh, mz = DOCK_MAST
    return (block('dock_plate', -0.30, m0, 0.22, 0.006, False), block('dock_mast', m0, m1, mh, mz, True))


def default_room():
    """5 × 4 m room in the start frame: robot at the origin facing +x, a tall cabinet in the way, a pillar, and the
    dock in the middle of the north wall, its mast against the wall (below the scan plane: Z0.36 < Z0.55)."""
    walls = box(-1.0, -2.0, 4.0, 2.0) + box(1.0, -0.8, 1.8, 0.9) + box(3.0, -1.4, 3.3, -1.1)
    dock = Pose2D(3.0, 2.0 - DOCK_MAST[1], math.pi / 2)
    return Room(walls, dock, dock_parts(dock))


def slice_at(room, z):
    """(segments, circles) that the horizontal plane at height z meets: walls plus the obstacles spanning z."""
    segs, circles = list(room.walls), []
    for o in room.obstacles:
        if o.z0 <= z <= o.z1:
            if isinstance(o, Block):
                segs.extend(box(o.x0, o.y0, o.x1, o.y1))
            else:
                circles.append((o.x, o.y, o.r))
    return tuple(segs), tuple(circles)


def _cross(ax, ay, bx, by):
    return ax * by - ay * bx


def _ray_circle(ox, oy, dx, dy, cx, cy, r):
    """Smallest t > 0 where o + t·d enters the circle (d need not be unit), inf if never."""
    a = dx * dx + dy * dy
    if a < 1e-18:
        return math.inf
    fx, fy = ox - cx, oy - cy
    b = (fx * dx + fy * dy) / a
    disc = b * b - (fx * fx + fy * fy - r * r) / a
    if disc < 0.0:
        return math.inf
    t = -b - math.sqrt(disc)
    return t if t > 0.0 else math.inf


def raycast(ox, oy, angle, walls, max_range, circles=()):
    """Distance along the ray to the nearest wall or circle, inf if none within max_range."""
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
    for cx, cy, r in circles:
        t = _ray_circle(ox, oy, dx, dy, cx, cy, r)
        if t <= best:
            best, hit = t, True
    return best if hit else math.inf


def _slab(o, d, lo, hi):
    if abs(d) < 1e-12:
        return (-math.inf, math.inf) if lo <= o <= hi else (math.inf, -math.inf)
    t0, t1 = (lo - o) / d, (hi - o) / d
    return (t0, t1) if t0 <= t1 else (t1, t0)


def ray_enters(ob, o, d):
    """Smallest t > 0 where the 3D ray o + t·d enters obstacle `ob`, inf if never."""
    tz0, tz1 = _slab(o[2], d[2], ob.z0, ob.z1)
    if isinstance(ob, Block):
        tx0, tx1 = _slab(o[0], d[0], ob.x0, ob.x1)
        ty0, ty1 = _slab(o[1], d[1], ob.y0, ob.y1)
        t0, t1 = max(tx0, ty0, tz0), min(tx1, ty1, tz1)
    else:
        a = d[0] * d[0] + d[1] * d[1]
        fx, fy = o[0] - ob.x, o[1] - ob.y
        if a < 1e-18:                                          # vertical ray
            if fx * fx + fy * fy > ob.r * ob.r:
                return math.inf
            t0, t1 = tz0, tz1
        else:
            b = (fx * d[0] + fy * d[1]) / a
            disc = b * b - (fx * fx + fy * fy - ob.r * ob.r) / a
            if disc < 0.0:
                return math.inf
            s = math.sqrt(disc)
            t0, t1 = max(-b - s, tz0), min(-b + s, tz1)
    return t0 if 0.0 < t0 <= t1 else math.inf


def raycast3d(o, d, room, max_t):
    """Distance along the unit ray o + t·d to the floor, a wall or an obstacle; max_t if nothing is nearer."""
    ox, oy, oz = o
    dx, dy, dz = d
    best = max_t
    if dz < 0.0:
        best = min(best, -oz / dz)
    for x0, y0, x1, y1 in room.walls:
        ex, ey = x1 - x0, y1 - y0
        den = _cross(dx, dy, ex, ey)
        if abs(den) < 1e-12:
            continue
        wx, wy = x0 - ox, y0 - oy
        t, u = _cross(wx, wy, ex, ey) / den, _cross(wx, wy, dx, dy) / den
        if 0.0 <= u <= 1.0 and 0.0 < t < best and 0.0 <= oz + t * dz <= WALL_HEIGHT:
            best = t
    for ob in room.obstacles:
        best = min(best, ray_enters(ob, o, d))
    return best


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


def rect_overlaps(pose, x0, x1, y0, y1, ob):
    """Does the rectangle [x0, x1] × [y0, y1] of the robot frame at `pose` overlap obstacle `ob` in plan view?
    Touching edges do not count."""
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    if isinstance(ob, Cylinder):
        dx, dy = ob.x - pose.x, ob.y - pose.y
        lx, ly = dx * c + dy * s, -dx * s + dy * c
        nx, ny = max(x0 - lx, 0.0, lx - x1), max(y0 - ly, 0.0, ly - y1)
        return nx * nx + ny * ny < ob.r * ob.r
    # separating axis test: the world axes, then the robot's
    corners = [(pose.x + x * c - y * s, pose.y + x * s + y * c) for x in (x0, x1) for y in (y0, y1)]
    if (max(p[0] for p in corners) <= ob.x0 or min(p[0] for p in corners) >= ob.x1 or
            max(p[1] for p in corners) <= ob.y0 or min(p[1] for p in corners) >= ob.y1):
        return False
    local = [((x - pose.x) * c + (y - pose.y) * s, -(x - pose.x) * s + (y - pose.y) * c)
             for x in (ob.x0, ob.x1) for y in (ob.y0, ob.y1)]
    return not (max(p[0] for p in local) <= x0 or min(p[0] for p in local) >= x1 or
                max(p[1] for p in local) <= y0 or min(p[1] for p in local) >= y1)


def _to_world(pose, x, y):
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    return pose.x + x * c - y * s, pose.y + x * s + y * c


def _gap_to(px, py, ob):
    """Plan-view distance from a point (world) to obstacle `ob`, 0 inside it."""
    if isinstance(ob, Cylinder):
        return max(0.0, math.hypot(px - ob.x, py - ob.y) - ob.r)
    return math.hypot(max(ob.x0 - px, 0.0, px - ob.x1), max(ob.y0 - py, 0.0, py - ob.y1))


def _seg_dist(px, py, x0, y0, x1, y1):
    ex, ey = x1 - x0, y1 - y0
    n = ex * ex + ey * ey
    t = 0.0 if n < 1e-18 else max(0.0, min(1.0, ((px - x0) * ex + (py - y0) * ey) / n))
    return math.hypot(px - (x0 + t * ex), py - (y0 + t * ey))


def disc_overlaps(pose, disc, ob):
    """Does the robot-frame disc at `pose` overlap obstacle `ob` in plan view? Touching does not count."""
    return _gap_to(*_to_world(pose, disc.x, disc.y), ob) < disc.r - 1e-12


def clearance(pose, ob, radius=ROBOT_RADIUS):
    """Plan-view gap (m) between the robot's round outline at `pose` and obstacle `ob`, 0 when they overlap."""
    return max(0.0, _gap_to(pose.x, pose.y, ob) - radius)


def _nearest_point(pose, ob):
    """Nearest point of obstacle `ob` to the robot centre, in the robot frame."""
    if isinstance(ob, Cylinder):
        dx, dy = ob.x - pose.x, ob.y - pose.y
        d = math.hypot(dx, dy) or 1e-9
        wx, wy = ob.x - dx / d * ob.r, ob.y - dy / d * ob.r
    else:
        wx, wy = min(max(pose.x, ob.x0), ob.x1), min(max(pose.y, ob.y0), ob.y1)
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    dx, dy = wx - pose.x, wy - pose.y
    return dx * c + dy * s, -dx * s + dy * c


def bumper_bits(angle):
    """Switches closed by a touch at `angle` (rad, robot frame) on the bumper ring: the front half pivots on two
    switches at ±45° (bit 0 FL, bit 1 FR), the rear half likewise (bit 2 BL, bit 3 BR). Within ±20° of straight
    ahead or back the half pushes in square and closes both."""
    a = wrap(angle)
    front = abs(a) < math.pi / 2
    off = a if front else wrap(a - math.pi)
    left, right = (0b0001, 0b0010) if front else (0b0100, 0b1000)   # behind, "left" is still the robot's left
    if abs(off) <= math.radians(20):
        return left | right
    return left if (off > 0) == front else right


@dataclass(frozen=True)
class SimConfig:
    # Same physics as core/mock_plant.PlantConfig
    start_soc: float = 0.6
    drain_w: float = 35.0
    charge_w: float = 50.0
    pack_wh: float = 154.0
    motor_tau_s: float = 0.15
    dock_y_tol_m: float = 0.045       # lateral misalignment the wheel funnel still takes in
    contact_travel_m: float = 0.003
    # 2D world
    dock_yaw_tol: float = 0.25        # rad the wheel funnel takes in (not measured on the real dock)
    dock_capture_m: float = 0.22      # dock plate half width: the wheel stops end the drive inside this band
    true_turn_slip: float = 1.03      # the wheels slip a little in turns, more than base_node's turn_slip (1.0) assumes
    linear_slip: float = 0.01         # and lose this fraction of travel: wheel odometry over-reports
    beams: int = 500                  # RPLIDAR C1: 5 kHz samples at 10 Hz
    range_max: float = 12.0
    range_noise_m: float = 0.01
    tag_range_m: float = 4.0          # AprilTag still decoded by the colour camera (80 mm tag)
    tag_max_incidence: float = math.radians(60)
    # Corner ToF (VL53L1X): rays fanned over the ROI, nearest return wins, floor included
    corner_range_m: float = 4.0       # what /luna/mcu/corners reports with nothing in the cone (long mode)
    tof_fov_h: float = math.radians(27)
    tof_fov_v: float = math.radians(15)   # assumed ROI 16 × 4 SPADs (core/tof.py); 27° would see the floor at 0.42 m
    tof_rays_h: int = 9
    tof_rays_v: int = 5
    tof_noise_m: float = 0.005
    contact_release_s: float = 0.5    # a contact ends after this long without touching
    seed: int = 1


def _tof_dirs(cfg, r_base):
    out = []
    for i in range(cfg.tof_rays_h):
        h = -cfg.tof_fov_h / 2 + i * cfg.tof_fov_h / (cfg.tof_rays_h - 1)
        for j in range(cfg.tof_rays_v):
            v = -cfg.tof_fov_v / 2 + j * cfg.tof_fov_v / (cfg.tof_rays_v - 1)
            out.append(Pose(r_base).apply((math.cos(v) * math.cos(h), math.cos(v) * math.sin(h), math.sin(v))))
    return out


class Sim2D:
    def __init__(self, room, cfg=SimConfig(), start=Pose2D(0.0, 0.0, 0.0), frames=None):
        self.room, self.cfg = room, cfg
        frames = frames or load_frames()
        self.pose = start
        self.target = [0.0, 0.0]
        self.vel = [0.0, 0.0]
        self.pos = [0.0, 0.0]
        self.wh = cfg.pack_wh * cfg.start_soc
        self.bumpers = 0
        self.docked = False
        self.rng = random.Random(cfg.seed)
        self.tof_rng = random.Random(cfg.seed + 1)
        self.t = 0.0
        self.laser_x, self.laser_y, self.laser_z = frames.origin('laser')
        self.laser_yaw = frames.yaw('laser')
        self.scan_walls, self.scan_circles = slice_at(room, self.laser_z)
        self.bands = robot_bands(frames)
        self.tofs = [(frames.origin(f), _tof_dirs(cfg, frames.pose(f)[1])) for f in TOF_FRAMES]
        self.solid = tuple(o for o in room.obstacles if o.contact)
        self.contacts = []                 # Contact log, oldest first
        self._last_touch = {}              # obstacle name → sim time it was last touched

    @property
    def charging(self):
        return self.docked

    @property
    def soc(self):
        return self.wh / self.cfg.pack_wh

    @property
    def twist(self):
        """True body velocity (m/s, rad/s) of the base, slip included."""
        vl, vr = (v * K.WHEEL_RADIUS * (1.0 - self.cfg.linear_slip) for v in self.vel)
        return (vl + vr) / 2, (vr - vl) / (K.WHEEL_SEPARATION * self.cfg.true_turn_slip)

    def _wall_hits(self, pose, radius, x=0.0, y=0.0):
        """Nearest wall points (robot frame) closer than `radius` to the robot-frame point (x, y)."""
        px, py = _to_world(pose, x, y)
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        out = []
        for x0, y0, x1, y1 in self.room.walls:
            if _seg_dist(px, py, x0, y0, x1, y1) < radius:
                ex, ey = x1 - x0, y1 - y0
                n = ex * ex + ey * ey
                t = max(0.0, min(1.0, ((pose.x - x0) * ex + (pose.y - y0) * ey) / n)) if n > 1e-18 else 0.0
                dx, dy = x0 + t * ex - pose.x, y0 + t * ey - pose.y
                out.append((dx * c + dy * s, -dx * s + dy * c))
        return out

    def _local(self, pose, x, y):
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        dx, dy = x - pose.x, y - pose.y
        return dx * c + dy * s, -dx * s + dy * c

    def _box_touches_wall(self, pose, xmin, xmax, ymin, ymax):
        for x0, y0, x1, y1 in self.room.walls:
            if _touches_box(self._local(pose, x0, y0), self._local(pose, x1, y1), xmin, xmax, ymin, ymax):
                return True
        return False

    def _near(self, pose):
        for ob in self.solid:
            if _gap_to(pose.x, pose.y, ob) < ROBOT_RADIUS + BUMPER_REACH + 0.02:
                yield ob

    def _overlaps(self, pose, band, ob):
        if isinstance(band, Disc):
            return disc_overlaps(pose, band, ob)
        return rect_overlaps(pose, band.x0, band.x1, band.y0, band.y1, ob)

    def hard_contacts(self, pose):
        """{obstacle name: robot bands overlapping it} at `pose`; 'wall' when a band reaches a wall."""
        out = {}
        for b in self.bands:
            hit = (self._wall_hits(pose, b.r, b.x, b.y) if isinstance(b, Disc)
                   else self._box_touches_wall(pose, b.x0, b.x1, b.y0, b.y1))
            if hit:
                out.setdefault('wall', set()).add(b.name)
        for ob in self._near(pose):
            for b in self.bands:
                if b.z0 < ob.z1 and ob.z0 < b.z1 and self._overlaps(pose, b, ob):
                    out.setdefault(ob.name, set()).add(b.name)
        return out

    def collides(self, pose):
        return bool(self.hard_contacts(pose))

    def _wall_bumpers(self):
        bits = 0
        for lx, ly in self._wall_hits(self.pose, ROBOT_RADIUS + BUMPER_REACH):
            bits |= bumper_bits(math.atan2(ly, lx))
        return bits

    def _obstacle_bumpers(self):
        """(bits, names) of the obstacles pressing the bumper ring."""
        bits, names = 0, set()
        reach = Disc('ring', 0.0, 0.0, ROBOT_RADIUS + BUMPER_REACH, *BUMPER_Z)
        for ob in self._near(self.pose):
            if ob.z0 < BUMPER_Z[1] and BUMPER_Z[0] < ob.z1 and disc_overlaps(self.pose, reach, ob):
                bits |= bumper_bits(math.atan2(*reversed(_nearest_point(self.pose, ob))))
                names.add(ob.name)
        return bits, names

    def _log_contacts(self, hard, pressed, wall_bits):
        touching = set(hard) | pressed | ({'wall'} if wall_bits else set())
        for name in touching:
            last = self._last_touch.get(name)
            if last is None or self.t - last > self.cfg.contact_release_s:
                felt = name in pressed or 'bumper' in hard.get(name, ()) or (name == 'wall' and wall_bits)
                self.contacts.append(Contact(round(self.t, 3), name, 'bumper' if felt else 'crash',
                                             tuple(sorted(hard.get(name, ())))))
            self._last_touch[name] = self.t

    def step(self, dt):
        c = self.cfg
        self.t += dt
        prev = list(self.pos)
        for i in range(2):
            self.vel[i] += (self.target[i] - self.vel[i]) * min(1.0, dt / c.motor_tau_s)
            self.pos[i] += self.vel[i] * dt
        dl, dr = ((self.pos[i] - prev[i]) * K.WHEEL_RADIUS * (1.0 - c.linear_slip) for i in range(2))
        ds, dyaw = (dl + dr) / 2, (dr - dl) / (K.WHEEL_SEPARATION * c.true_turn_slip)
        p, mid = self.pose, self.pose.yaw + dyaw / 2
        cand = Pose2D(p.x + ds * math.cos(mid), p.y + ds * math.sin(mid), wrap(p.yaw + dyaw))
        along, lateral, _ = dock_frame(cand, self.room.dock)
        hard = self.hard_contacts(cand) if cand != p else {}
        if (along > 0.0 and abs(lateral) < c.dock_capture_m) or hard:
            self.pos, self.vel = prev, [0.0, 0.0]          # wheels stall at the dock's wheel stops, a wall or furniture
        else:
            self.pose = cand
        along, lateral, dyaw_err = dock_frame(self.pose, self.room.dock)
        at_dock = abs(lateral) < c.dock_capture_m
        aligned = abs(lateral) < c.dock_y_tol_m and abs(dyaw_err) < c.dock_yaw_tol
        self.docked = at_dock and aligned and along >= -c.contact_travel_m
        wall_bits = self._wall_bumpers()
        ob_bits, pressed = self._obstacle_bumpers()
        self.bumpers = wall_bits | ob_bits
        self._log_contacts(hard, pressed, wall_bits)
        self.wh = min(c.pack_wh, self.wh + (c.charge_w if self.charging else -c.drain_w) * dt / 3600)

    def scan(self, pose=None):
        """RPLIDAR C1 ranges (m, inf = no return), angle_min = -pi, increment 2*pi/beams, in the laser frame (turned by
        the urdf laser yaw), seen from `pose` (default: now). Only walls and obstacles spanning the scan plane return."""
        c, p = self.cfg, pose or self.pose
        cy, sy = math.cos(p.yaw), math.sin(p.yaw)
        lx, ly = p.x + self.laser_x * cy - self.laser_y * sy, p.y + self.laser_x * sy + self.laser_y * cy
        inc = 2 * math.pi / c.beams
        out = []
        for i in range(c.beams):
            r = raycast(lx, ly, p.yaw + self.laser_yaw - math.pi + i * inc, self.scan_walls, c.range_max,
                        self.scan_circles)
            if r != math.inf and c.range_noise_m:
                r += self.rng.gauss(0.0, c.range_noise_m)
            out.append(r)
        return out

    def corners(self):
        """VL53L1X ranges [FL, FR, BL, BR] (m): nearest return in each cone, floor included; corner_range_m when
        nothing is in range. Window positions and headings come from the urdf tof_* frames."""
        p, c = self.pose, self.cfg
        cy, sy = math.cos(p.yaw), math.sin(p.yaw)
        out = []
        for (bx, by, bz), dirs in self.tofs:
            o = (p.x + bx * cy - by * sy, p.y + bx * sy + by * cy, bz)
            best = c.corner_range_m
            for dx, dy, dz in dirs:
                best = raycast3d(o, (dx * cy - dy * sy, dx * sy + dy * cy, dz), self.room, best)
            if best < c.corner_range_m and c.tof_noise_m:
                best = max(0.0, best + self.tof_rng.gauss(0.0, c.tof_noise_m))
            out.append(best)
        return out

    def tag(self):
        """(offset -1..1, distance to the seat m) like core/mock_plant.DockPlant.tag, or None when the colour camera
        can't see the tag on the mast."""
        c, d = self.cfg, self.room.dock
        along, lateral, dyaw = dock_frame(self.pose, d)
        g = TagGeometry()
        dist, to_tag = -along, g.seat_m - along
        if dist < -0.01 or math.hypot(to_tag, lateral) > c.tag_range_m:
            return None
        if math.atan2(abs(lateral), to_tag) > c.tag_max_incidence:
            return None                                    # tag seen too obliquely to decode
        x, y, _ = tag_in_base(along, lateral, dyaw, g)
        tx, ty = d.x + g.seat_m * math.cos(d.yaw), d.y + g.seat_m * math.sin(d.yaw)
        if not in_colour_view(x, y) or _segment_blocked(self.pose.x, self.pose.y, tx, ty, self.room.walls):
            return None
        offset = max(-1.0, min(1.0, -math.atan2(y, x) / math.radians(g.half_angle_deg)))
        return offset, max(0.0, dist)


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
