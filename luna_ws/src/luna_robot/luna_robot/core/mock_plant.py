"""World model behind mock_mcu_node: two lagged wheel motors, the R2 dock ahead of the start pose along +x (wheel
funnel, wheel stops at the seat, spring pins, tag mast), and the LiFePO4 pack. Pure Python so the docking loop can be
tested end to end without ROS."""
import math
from dataclasses import dataclass

from . import kinematics as K
from .dock_tag import TagGeometry, tag_in_base

CAMERA_X = 0.165                      # Gemini 336 colour lens ahead of base_link (URDF camera_link)
TAG_HALF_FOV = math.radians(40)       # its 86° wide view seen from the lens, less a margin for the tag's own size


def in_colour_view(x, y):
    """Is the tag centre (x, y) in base_link inside the colour camera's horizontal view? Seen from the lens, which
    sits ahead of the turning centre: turned 34° away at the pre-dock pose already puts the tag at the image edge."""
    return x - CAMERA_X > 0 and abs(math.atan2(y, x - CAMERA_X)) < TAG_HALF_FOV


@dataclass
class PlantConfig:
    dock_distance_m: float = 1.2      # start pose to the seat along +x
    start_soc: float = 0.6
    drain_w: float = 35.0             # docs/TASARIM_R2.md, "Güç": Orin NX at 15 W plus sensors, screen, motors
    charge_w: float = 50.0            # 14.6 V 4 A LiFePO4 charger in the dock
    pack_wh: float = 154.0            # 4S2P LiFePO4 32700: 12.8 V × 12 Ah
    motor_tau_s: float = 0.15         # first-order lag of the wheel speed
    dock_y_tol_m: float = 0.045       # lateral misalignment the wheel funnel still takes in
    dock_yaw_tol: float = 0.25        # rad, the same for heading
    contact_travel_m: float = 0.003   # spring pins touch this far before the wheel stops
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
        self.dist = cfg.dock_distance_m       # distance to the seat along +x
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
        # The pins must keep touching while the wheels rest against their stops. With `dist <= 0` they only did on
        # the one tick of overshoot before the stop pushed the robot back, so DOCK_SENSE flickered.
        aligned = abs(self.y) < c.dock_y_tol_m and abs(self.yaw) < c.dock_yaw_tol
        self.docked = self.dist <= c.contact_travel_m and aligned
        if self.dist < 0:                                 # wheel stops (or the funnel rail): no driving on
            self.pos = [p - v * dt for p, v in zip(self.pos, self.vel)]
            self.vel = [0.0, 0.0]
        self.wh = min(c.pack_wh, self.wh + (c.charge_w if self.charging else -c.drain_w) * dt / 3600)

    def tag(self):
        """(offset -1..1, distance m) when the dock tag is inside the colour camera's view, else None."""
        x, y, _ = tag_in_base(-self.dist, self.y, self.yaw)
        if in_colour_view(x, y) and self.dist > -0.01:          # offset: bearing from base_link, like observe()
            offset = -math.atan2(y, x) / math.radians(TagGeometry().half_angle_deg)
            return max(-1.0, min(1.0, offset)), max(0.0, self.dist)    # clamped like core/dock_tag.observe
        return None
