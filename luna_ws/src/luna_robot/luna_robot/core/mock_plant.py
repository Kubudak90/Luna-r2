"""World model behind mock_mcu_node: two lagged track motors, a wall-backed dock ahead of the start pose
along +x with the AprilTag above it, dock contacts and a 170 Wh pack. Pure Python so the docking loop
can be tested end to end without ROS."""
import math
from dataclasses import dataclass

from . import kinematics as K

TAG_HALF_FOV = math.radians(35)       # head camera half field of view used for tag visibility


@dataclass
class PlantConfig:
    dock_distance_m: float = 1.2
    start_soc: float = 0.6
    drain_w: float = 44.0             # Donanım ve Güç R01, normal mode
    charge_w: float = 50.0
    pack_wh: float = 170.0
    motor_tau_s: float = 0.15         # first-order lag of the track speed
    dock_y_tol_m: float = 0.02        # lateral misalignment the receiver mouth still accepts
    contact_travel_m: float = 0.003   # spring contacts touch this far before the hard stop
    start_y_m: float = 0.0            # start pose relative to the dock axis (left +)
    start_yaw_deg: float = 0.0        # start heading relative to the dock axis (left +)


class DockPlant:
    def __init__(self, cfg=PlantConfig()):
        self.cfg = cfg
        self.target = [0.0, 0.0]
        self.vel = [0.0, 0.0]
        self.pos = [0.0, 0.0]
        self.odo = K.Odometry()
        self.wh = cfg.pack_wh * cfg.start_soc
        self.x, self.y, self.yaw = 0.0, cfg.start_y_m, math.radians(cfg.start_yaw_deg)
        self.dist = cfg.dock_distance_m       # mouth-to-nose distance along +x
        self.docked = False                   # DOCK_SENSE: seated on the powered dock (P4 ID)
        self.charge_enable = True             # /luna/mcu/charge_enable; false = DOCK_INHIBIT, contacts dead

    @property
    def charging(self):
        return self.docked and self.charge_enable

    @property
    def soc(self):
        return self.wh / self.cfg.pack_wh

    def step(self, dt):
        c = self.cfg
        for i in range(2):
            self.vel[i] += (self.target[i] - self.vel[i]) * min(1.0, dt / c.motor_tau_s)
            self.pos[i] += self.vel[i] * dt
        ox, oy, oyaw = self.odo.update(*self.pos)
        y0 = math.radians(c.start_yaw_deg)
        self.x = math.cos(y0) * ox - math.sin(y0) * oy
        self.y = c.start_y_m + math.sin(y0) * ox + math.cos(y0) * oy
        self.yaw = y0 + oyaw
        self.dist = c.dock_distance_m - self.x
        # Contact must hold while the robot rests against the wall. With `dist <= 0` it only lasted the
        # one tick of overshoot before the wall pushed the robot back, so DOCK_SENSE flickered.
        self.docked = self.dist <= c.contact_travel_m and abs(self.y) < c.dock_y_tol_m
        if self.dist < 0:                                 # the dock is a wall: no driving through it
            self.pos = [p - v * dt for p, v in zip(self.pos, self.vel)]
            self.vel = [0.0, 0.0]
        self.wh = min(c.pack_wh, self.wh + (c.charge_w if self.charging else -c.drain_w) * dt / 3600)

    def tag(self):
        """(offset -1..1, distance m) when the dock tag is inside the head camera's view, else None."""
        bearing = math.atan2(-self.y, max(1e-3, self.dist)) - self.yaw
        if abs(bearing) < TAG_HALF_FOV and self.dist > -0.01:
            return -bearing / TAG_HALF_FOV, max(0.0, self.dist)
        return None
