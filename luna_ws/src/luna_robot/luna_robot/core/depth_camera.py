"""Simulated Orbbec Gemini 336 depth (active stereo with an IR dot projector) for the 2.5D world: a low-resolution
pinhole raycast against the floor, the walls and every obstacle, returned as points in the depth camera's optical
frame (z out of the lens, x right, y down), like OrbbecSDK_ROS2's /camera/depth/points. numpy only; the pose comes
from the urdf camera_depth_optical_frame.

What it models: field of view (90° × 65°), the minimum depth, maximum range and stereo depth noise growing with z²
(σz = k·z²; k from Orbbec's ≤ 0.8 % at 2 m). What it does not: glass and mirrors, very dark or shiny surfaces, the
projector washed out by sunlight through a window, motion blur, the body pitching when braking. R1's passive OAK-D
Lite also failed on plain walls and in the dark; the projector covers those, so they are not modelled either."""
from dataclasses import dataclass

import numpy as np

from .sim2d import WALL_HEIGHT, Block

FRAME = 'camera_depth_optical_frame'


@dataclass(frozen=True)
class DepthConfig:
    width: int = 64
    height: int = 46
    hfov_deg: float = 90.0
    vfov_deg: float = 65.0
    min_z_m: float = 0.15          # Gemini 336: 0.10 m by the datasheet, best from 0.26 m; 0.15 m here
    max_z_m: float = 8.0
    noise_k: float = 0.004         # σz = k·z² (m⁻¹)
    rate_hz: float = 5.0


def pixel_rays(cfg):
    """(N, 3) ray directions in the optical frame, scaled so that z = 1: the ray parameter is the depth."""
    fx = cfg.width / 2 / np.tan(np.radians(cfg.hfov_deg) / 2)
    fy = cfg.height / 2 / np.tan(np.radians(cfg.vfov_deg) / 2)
    u, v = np.meshgrid(np.arange(cfg.width) + 0.5, np.arange(cfg.height) + 0.5)
    return np.stack([(u.ravel() - cfg.width / 2) / fx, (v.ravel() - cfg.height / 2) / fy,
                     np.ones(cfg.width * cfg.height)], axis=1)


def _safe(d):
    return np.where(np.abs(d) < 1e-12, 1e-12, d)


def _slab(o, d, lo, hi):
    inv = 1.0 / _safe(d)
    t0, t1 = (lo - o) * inv, (hi - o) * inv
    return np.minimum(t0, t1), np.maximum(t0, t1)


def depth_along(origin, dirs, room):
    """Ray parameter t of the first hit for every ray origin + t·dirs (world frame), inf for no hit."""
    ox, oy, oz = origin
    dx, dy, dz = dirs[:, 0], dirs[:, 1], dirs[:, 2]
    best = np.full(len(dirs), np.inf)
    floor = np.where(dz < 0.0, -oz / _safe(dz), np.inf)
    best = np.minimum(best, floor)
    for x0, y0, x1, y1 in room.walls:
        ex, ey = x1 - x0, y1 - y0
        den = dx * ey - dy * ex
        wx, wy = x0 - ox, y0 - oy
        sden = _safe(den)
        t = (wx * ey - wy * ex) / sden
        u = (wx * dy - wy * dx) / sden
        z = oz + t * dz
        ok = (np.abs(den) > 1e-12) & (u >= 0.0) & (u <= 1.0) & (t > 0.0) & (z >= 0.0) & (z <= WALL_HEIGHT)
        best = np.where(ok & (t < best), t, best)
    for ob in room.obstacles:
        tz0, tz1 = _slab(oz, dz, ob.z0, ob.z1)
        if isinstance(ob, Block):
            tx0, tx1 = _slab(ox, dx, ob.x0, ob.x1)
            ty0, ty1 = _slab(oy, dy, ob.y0, ob.y1)
            t0, t1 = np.maximum(np.maximum(tx0, ty0), tz0), np.minimum(np.minimum(tx1, ty1), tz1)
        else:
            a = _safe(dx * dx + dy * dy)
            fx, fy = ox - ob.x, oy - ob.y
            b = (fx * dx + fy * dy) / a
            disc = b * b - (fx * fx + fy * fy - ob.r * ob.r) / a
            s = np.sqrt(np.maximum(disc, 0.0))
            t0 = np.where(disc >= 0.0, np.maximum(-b - s, tz0), np.inf)
            t1 = np.where(disc >= 0.0, np.minimum(-b + s, tz1), -np.inf)
        hit = (t0 > 0.0) & (t0 <= t1) & (t0 < best)
        best = np.where(hit, t0, best)
    return best


def render(pose, room, frames, cfg=DepthConfig(), rng=None, rays=None):
    """Point cloud (M, 3) float32 in the FRAME optical frame for the robot at `pose` (Pose2D, world)."""
    t_base, r_base = frames.pose(FRAME)
    c, s = np.cos(pose.yaw), np.sin(pose.yaw)
    rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    r_world = rz @ np.array(r_base)
    origin = rz @ np.array(t_base) + np.array([pose.x, pose.y, 0.0])
    rays = pixel_rays(cfg) if rays is None else rays
    z = depth_along(origin, rays @ r_world.T, room)              # = depth, since every ray has z = 1
    ok = np.isfinite(z) & (z >= cfg.min_z_m) & (z <= cfg.max_z_m)
    z = z[ok]
    if rng is not None and cfg.noise_k:
        z = z + rng.normal(0.0, 1.0, z.shape) * cfg.noise_k * z * z
    return (rays[ok] * z[:, None]).astype(np.float32)
