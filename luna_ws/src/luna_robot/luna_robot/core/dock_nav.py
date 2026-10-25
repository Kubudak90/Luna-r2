"""Go-to-dock coordinator: Nav2 drives to a saved pre-dock pose in front of the dock, then dock_node takes over.

The dock pose is the robot's position (map frame) while docked plus the dock axis; it is learned when a docking run
reaches CHARGING. The wheel funnel accepts a skewed robot, so the axis comes from the wall behind the dock (a line
fitted to the LiDAR points ahead; the tag mast stands against it), not from the robot's heading. The pre-dock pose lies
`standoff_m` straight back along that axis: the colour camera sees the tag from there and dock_node's tag approach
(0.6 m at 0.06 m/s ≈ 10 s)
stays well inside its 25 s approach timeout.
Velocity ownership: Nav2 drives (via /cmd_vel) only in NAVIGATE, dock_node (via /luna/dock/cmd_vel) only after the
hand-over. Both still pass base_node's safety gate.

Pure logic: dock_nav_node feeds observations and executes the returned commands:
  ('navigate', seq, Pose2D)   send a NavigateToPose goal in the map frame
  ('backup', seq, metres)     reverse straight back (Nav2 BackUp behaviour)
  ('cancel_nav', seq)         cancel that goal (navigate or backup)
  ('dock', 'dock'|'cancel')   publish on /luna/dock/request
  ('save_dock', Pose2D)       store the learned dock pose
  ('reject_dock', Pose2D)     charging pose too far from the stored one: keep the old pose and warn"""
import math
from dataclasses import dataclass
from typing import NamedTuple, Optional


class Pose2D(NamedTuple):
    x: float
    y: float
    yaw: float


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def pre_dock_pose(dock, standoff):
    return Pose2D(dock.x - standoff * math.cos(dock.yaw), dock.y - standoff * math.sin(dock.yaw), dock.yaw)


def pose_error(a, b):
    """(distance m, |yaw difference| rad) between two poses."""
    return math.hypot(b.x - a.x, b.y - a.y), abs(wrap(b.yaw - a.yaw))


def dock_frame(pose, dock):
    """(along, lateral, yaw error) of `pose` relative to the docked pose. along < 0 is in front of the dock."""
    dx, dy = pose.x - dock.x, pose.y - dock.y
    c, s = math.cos(dock.yaw), math.sin(dock.yaw)
    return dx * c + dy * s, -dx * s + dy * c, wrap(pose.yaw - dock.yaw)


def front_points(ranges, angle_min, increment, laser_x, max_range, half_angle, laser_yaw=0.0, laser_y=0.0):
    """LaserScan ranges → (x, y) points in the base frame within ±half_angle of straight ahead and max_range.
    The laser frame is the base frame shifted by (laser_x, laser_y) and turned by laser_yaw (urdf laser_joint; the
    C1 mount has yaw −90°: its +x points to the robot's right)."""
    pts = []
    for i, r in enumerate(ranges):
        a = laser_yaw + angle_min + i * increment                 # beam direction in the base frame
        if 0.0 < r <= max_range and abs(wrap(a)) <= half_angle:
            pts.append((laser_x + r * math.cos(a), laser_y + r * math.sin(a)))
    return pts


def wall_normal_yaw(points, max_rms=0.03, min_points=15):
    """Direction (rad, robot frame) straight into a flat wall ahead, from a least-squares line through `points`;
    None when there are too few points or they do not lie on one straight line (corner, furniture, open space).
    max_rms allows for the C1's ±30 mm ranging error."""
    n = len(points)
    if n < min_points:
        return None
    mx, my = sum(p[0] for p in points) / n, sum(p[1] for p in points) / n
    sxx = sum((p[0] - mx) ** 2 for p in points) / n
    syy = sum((p[1] - my) ** 2 for p in points) / n
    sxy = sum((p[0] - mx) * (p[1] - my) for p in points) / n
    half_trace = (sxx + syy) / 2
    residual = half_trace - math.sqrt(max(0.0, half_trace ** 2 - (sxx * syy - sxy ** 2)))   # smaller eigenvalue
    if math.sqrt(max(0.0, residual)) > max_rms:
        return None
    along = 0.5 * math.atan2(2 * sxy, sxx - syy)          # direction of the line
    nx, ny = -math.sin(along), math.cos(along)
    if nx * mx + ny * my < 0:                              # point from the robot towards the wall
        nx, ny = -nx, -ny
    return math.atan2(ny, nx)


def format_dock_pose(p):
    return ('# Luna dock (map frame, written by dock_nav_node): robot position while docked, yaw = dock axis\n'
            f'x: {p.x:.4f}\ny: {p.y:.4f}\nyaw: {p.yaw:.4f}\n')


def parse_dock_pose(text):
    vals = {}
    for line in text.splitlines():
        line = line.split('#', 1)[0]
        if ':' in line:
            k, v = line.split(':', 1)
            vals[k.strip()] = float(v)
    try:
        return Pose2D(vals['x'], vals['y'], vals['yaw'])
    except KeyError as e:
        raise ValueError(f'dock file is missing {e}') from None


DOCK_APPROACH = {'SEEK', 'ALIGN', 'APPROACH', 'CONTACT', 'BACKOFF'}    # dock_node driving towards the dock
DOCK_PARKED = {'CHARGING', 'DOCKED'}
NEEDS_DOCK = {'seek_dock', 'critical'}                                 # battery policy actions (core/battery.py)


@dataclass(frozen=True)
class DockNavConfig:
    standoff_m: float = 0.6           # pre-dock pose: this far straight back from the docked pose
    # dock_node steers at the tag, 0.45 m beyond the seat, so a robot starting beside the axis arrives at the seat with
    # ≈ 0.43 of that offset: hand over only inside a cone around the axis (the wheel funnel takes ±0.045 m).
    handover_entry: float = 0.13      # rad, cone half angle (0.078 m at 0.6 m)
    handover_lateral_m: float = 0.10  # cone cap
    handover_lateral_min_m: float = 0.02
    handover_slack_m: float = 0.15    # accepted this much further back than standoff_m
    handover_yaw: float = 0.3         # rad; dock_node's ALIGN turns the rest, the tag must stay in the ±40° view
    backup_m: float = 0.4             # after a dock FAULT: reverse off the dock plate before re-planning
    backup_timeout_s: float = 30.0
    nav_timeout_s: float = 240.0
    retry_delay_s: float = 2.0
    nav2_wait_s: float = 60.0         # Nav2 action servers not up yet (boot): keep retrying this long
    max_nav_attempts: int = 3
    max_dock_attempts: int = 2        # dock_node FAULT (3 tag retries) → drive back to the pre-dock pose once
    handover_timeout_s: float = 2.0   # dock_node must leave IDLE/FAULT this soon after the request
    start_timeout_s: float = 15.0     # waiting for localization or an UNDOCK to finish
    retry_cooldown_s: float = 120.0   # battery-triggered retry after a failure or a user cancel
    relearn_max_shift_m: float = 0.5  # a charging pose further than this from the stored dock is not learned
    auto: bool = True                 # start by itself when the battery policy asks for the dock


@dataclass
class NavObs:
    dock_state: str = 'IDLE'
    power_action: str = 'unknown'
    pose: Optional[Pose2D] = None     # robot in the map frame, None while not localized
    nav_result: Optional[tuple] = None  # (seq, succeeded|aborted|canceled|rejected|unavailable), one tick only
    wall_yaw: Optional[float] = None  # wall_normal_yaw of the scan ahead (robot frame), None if no flat wall


class DockNav:
    """IDLE → WAIT → NAVIGATE → HANDOVER → DOCKING → DOCKED (dock FAULT: BACKUP → NAVIGATE once more);
    FAILED holds off battery retries for a cooldown."""

    def __init__(self, cfg=DockNavConfig(), dock=None):
        self.cfg = cfg
        self.dock = dock
        self.state, self.reason = 'IDLE', ''
        self.t = self.clock = 0.0
        self.seq, self.goal = 0, None     # last goal number sent, active goal (None = none in flight)
        self.nav_attempts = self.dock_attempts = self.unavailable = 0
        self.hold_until = 0.0
        self.prev_dock = None
        self._req = None

    @property
    def pre_dock(self):
        return pre_dock_pose(self.dock, self.cfg.standoff_m) if self.dock else None

    def request(self, what):
        if what in ('go', 'cancel'):
            self._req = what

    def ready_for_handover(self, pose):
        """In the cone around the dock axis between the pre-dock pose and the dock, facing it: dock_node can take it
        from here."""
        c = self.cfg
        along, lateral, dyaw = dock_frame(pose, self.dock)
        if not -(c.standoff_m + c.handover_slack_m) <= along <= 0.0:
            return False
        limit = min(c.handover_lateral_m, max(c.handover_lateral_min_m, -along * math.tan(c.handover_entry)))
        return abs(lateral) <= limit and abs(dyaw) <= c.handover_yaw

    def step(self, o, dt):
        self.clock += dt
        self.t += dt
        out = []
        req, self._req = self._req, None
        if req == 'cancel':
            out += self._abandon()
            self._go('IDLE', 'canceled')
            self.hold_until = self.clock + self.cfg.retry_cooldown_s
        elif req == 'go' and self.state in ('IDLE', 'FAILED', 'DOCKED'):
            self._begin()
        if self.state in ('IDLE', 'FAILED'):
            if o.dock_state in DOCK_PARKED:
                self._go('DOCKED', self.reason)
            elif (self.cfg.auto and o.power_action in NEEDS_DOCK and o.dock_state in ('IDLE', 'FAULT')
                  and self.clock >= self.hold_until):
                self._begin()
        handler = {'WAIT': self._wait, 'NAVIGATE': self._navigate, 'BACKUP': self._backing,
                   'HANDOVER': self._handover_wait, 'DOCKING': self._docking, 'DOCKED': self._docked}.get(self.state)
        if handler:
            out += handler(o)
        out += self._learn(o)
        self.prev_dock = o.dock_state
        return out

    # --- states -------------------------------------------------------------------------------------------------
    def _wait(self, o):
        if o.dock_state in DOCK_PARKED:
            self._go('DOCKED')
        elif o.dock_state in DOCK_APPROACH:
            self.dock_attempts = 1
            self._go('DOCKING')
        elif o.dock_state == 'UNDOCK' or (self.dock is not None and o.pose is None):
            if self.t > self.cfg.start_timeout_s:
                self._fail('not_localized' if o.pose is None else 'undock_stuck')
        elif self.dock is None:
            return self._handover('no_dock_pose')
        elif self.ready_for_handover(o.pose):
            return self._handover()
        else:
            return self._send()
        return []

    def _navigate(self, o):
        if o.dock_state in DOCK_PARKED | DOCK_APPROACH:      # dock_node took over, or Luna was put on the dock
            out = self._cancel_goal()
            self._go('DOCKED' if o.dock_state in DOCK_PARKED else 'DOCKING')
            return out
        if self.goal is None:                                # waiting to retry
            return self._send() if self.t >= self.cfg.retry_delay_s else []
        r = o.nav_result
        if r is not None and r[0] == self.goal:
            self.goal = None
            if r[1] == 'succeeded':
                if o.pose is not None and self.ready_for_handover(o.pose):
                    return self._handover()
                return self._nav_failed('pose_off')          # Nav2 is done but the tag would not be in view
            if r[1] == 'canceled':                           # someone else's goal or cancel: not ours to retry
                self._go('IDLE', 'nav_canceled')
                return []
            if r[1] == 'unavailable':                        # Nav2 still starting: not a failed attempt
                self.unavailable += 1
                if self.unavailable * self.cfg.retry_delay_s > self.cfg.nav2_wait_s:
                    self._fail('nav2_unavailable')
                    return []
                self.nav_attempts -= 1
                return self._nav_failed('nav2_unavailable')
            return self._nav_failed('nav_failed')
        if self.t > self.cfg.nav_timeout_s:
            return self._cancel_goal() + self._nav_failed('nav_timeout')
        return []

    def _handover_wait(self, o):
        if o.dock_state in DOCK_PARKED:
            self._go('DOCKED')
        elif o.dock_state in DOCK_APPROACH:
            self._go('DOCKING', self.reason)
        elif self.t > self.cfg.handover_timeout_s:           # FAULT is fine for a tick: dock_node accepts from there
            self._fail('dock_node_silent')
        return []

    def _docking(self, o):
        if o.dock_state in DOCK_PARKED:
            self._go('DOCKED')
        elif o.dock_state == 'FAULT':
            if self.dock is None or self.dock_attempts >= self.cfg.max_dock_attempts:
                self._fail('dock_fault')
            else:                                            # a fresh, straight approach from the pre-dock pose
                self.nav_attempts, self.reason = 0, 'dock_fault'
                return self._backup()
        elif o.dock_state in ('IDLE', 'UNDOCK'):
            self._go('IDLE', 'dock_canceled')
        return []

    def _backing(self, o):
        r = o.nav_result
        if r is not None and r[0] == self.goal:
            self.goal = None
            if r[1] == 'succeeded':
                return self._send()
            self._fail('backup_failed')
        elif self.t > self.cfg.backup_timeout_s:
            out = self._cancel_goal()
            self._fail('backup_failed')
            return out
        return []

    def _docked(self, o):
        if o.dock_state not in DOCK_PARKED:                  # lifted off or undocked
            self._go('IDLE')
        return []

    # --- helpers ------------------------------------------------------------------------------------------------
    def _go(self, state, reason=''):
        self.state, self.reason, self.t = state, reason, 0.0

    def _begin(self):
        self.nav_attempts = self.dock_attempts = self.unavailable = 0
        self._go('WAIT')

    def _send(self):
        self.nav_attempts += 1
        self.seq += 1
        self.goal = self.seq
        self._go('NAVIGATE', self.reason)
        return [('navigate', self.seq, self.pre_dock)]

    def _backup(self):
        self.seq += 1
        self.goal = self.seq
        self._go('BACKUP', self.reason)
        return [('backup', self.seq, self.cfg.backup_m)]

    def _handover(self, reason=''):
        self.dock_attempts += 1
        self._go('HANDOVER', reason)
        return [('dock', 'dock')]

    def _nav_failed(self, reason):
        self.goal = None
        if self.nav_attempts >= self.cfg.max_nav_attempts:
            self._fail(reason)
        else:
            self.reason, self.t = reason, 0.0                # retry after retry_delay_s
        return []

    def _fail(self, reason):
        self._go('FAILED', reason)
        self.hold_until = self.clock + self.cfg.retry_cooldown_s

    def _cancel_goal(self):
        if self.goal is None:
            return []
        out, self.goal = [('cancel_nav', self.goal)], None
        return out

    def _abandon(self):
        if self.state in ('NAVIGATE', 'BACKUP'):
            return self._cancel_goal()
        if self.state in ('HANDOVER', 'DOCKING'):
            return [('dock', 'cancel')]
        return []

    def _learn(self, o):
        if o.dock_state != 'CHARGING' or self.prev_dock not in DOCK_APPROACH or o.pose is None:
            return []
        yaw = o.pose.yaw if o.wall_yaw is None else wrap(o.pose.yaw + o.wall_yaw)
        learned = Pose2D(o.pose.x, o.pose.y, yaw)
        if self.dock is not None and pose_error(self.dock, learned)[0] > self.cfg.relearn_max_shift_m:
            return [('reject_dock', learned)]
        self.dock = learned
        return [('save_dock', learned)]
