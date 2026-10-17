"""Synthetic head-camera images of the dock tag, for the simulated camera and the detection checks. Needs numpy
(present in the ROS images); the rest of core does not.

The tag is ray-traced on the wall plane through an ideal pinhole (what apriltag_ros expects as `image_rect`):
every pixel is supersampled, then optional defocus blur and sensor noise are added."""
import numpy as np

from . import tag36h11
from .dock_tag import TAG_FACE_X_DOCKED, TagGeometry
from .frames import Pose

# Tag frame as in apriltag: centre of the tag, x right and y down as seen from the front, z into the wall.
# Dock frame: origin at the seated base_link pose on the floor, x towards the wall, y left, z up.
DOCK_R_TAG = ((0.0, 0.0, 1.0), (-1.0, 0.0, 0.0), (0.0, -1.0, 0.0))


def dock_T_tag(g=TagGeometry()):
    return Pose(DOCK_R_TAG, (TAG_FACE_X_DOCKED, 0.0, g.height_m))


def _blur(img, sigma):
    r = max(1, int(3 * sigma + 0.5))
    k = np.exp(-0.5 * (np.arange(-r, r + 1) / sigma) ** 2)
    k /= k.sum()
    pad = np.pad(img, r, mode='edge')
    tmp = np.apply_along_axis(lambda m: np.convolve(m, k, mode='valid'), 1, pad)
    return np.apply_along_axis(lambda m: np.convolve(m, k, mode='valid'), 0, tmp)


def render(cam, opt_T_tag, g=TagGeometry(), plate_m=0.064, tag_id=0, black=30.0, white=220.0, plate=70.0,
           wall=120.0, blur_px=0.0, noise=2.0, ss=3, rng=None):
    """uint8 grey image (cam.height, cam.width) of the tag whose pose in the camera optical frame is opt_T_tag.
    noise: sensor noise sigma in grey levels, or a ready (height, width) float32 noise frame (cheaper per frame)."""
    img = np.full((cam.height, cam.width), wall, dtype=np.float32)
    corners = [opt_T_tag.apply((sx * plate_m / 2, sy * plate_m / 2, 0.0)) for sx in (-1, 1) for sy in (-1, 1)]
    if all(c[2] > 1e-3 for c in corners):
        uv = [cam.project(c) for c in corners]
        pad = 3 + 3 * blur_px
        u0, u1 = int(max(0, min(u for u, _ in uv) - pad)), int(min(cam.width, max(u for u, _ in uv) + pad + 1))
        v0, v1 = int(max(0, min(v for _, v in uv) - pad)), int(min(cam.height, max(v for _, v in uv) + pad + 1))
    elif any(c[2] > 1e-3 for c in corners):
        u0, u1, v0, v1 = 0, cam.width, 0, cam.height
    else:
        u0 = u1 = v0 = v1 = 0
    if u1 > u0 and v1 > v0:
        offs = (np.arange(ss) + 0.5) / ss - 0.5
        us = (np.arange(u0, u1)[:, None] + offs[None, :]).ravel()
        vs = (np.arange(v0, v1)[:, None] + offs[None, :]).ravel()
        uu, vv = np.meshgrid(us, vs)
        d = np.stack([(uu - cam.cx) / cam.fx, (vv - cam.cy) / cam.fy, np.ones_like(uu)], axis=-1)
        inv = opt_T_tag.inverse()
        r, o = np.array(inv.r), np.array(inv.t)
        dt = d @ r.T                                    # ray directions in the tag frame
        with np.errstate(divide='ignore', invalid='ignore'):
            s = -o[2] / dt[..., 2]
        hit = s > 0
        X, Y = o[0] + s * dt[..., 0], o[1] + s * dt[..., 1]
        val = np.full(uu.shape, wall)
        on_plate = hit & (np.abs(X) <= plate_m / 2) & (np.abs(Y) <= plate_m / 2)
        val[on_plate] = plate
        cell = g.size_m / tag36h11.BLACK_WIDTH
        half = cell * tag36h11.TOTAL_WIDTH / 2
        on_tag = hit & (np.abs(X) < half) & (np.abs(Y) < half)
        bits = np.array(tag36h11.bitmap(tag_id), dtype=bool)
        col = np.clip(((X[on_tag] + half) / cell).astype(int), 0, tag36h11.TOTAL_WIDTH - 1)
        row = np.clip(((Y[on_tag] + half) / cell).astype(int), 0, tag36h11.TOTAL_WIDTH - 1)
        val[on_tag] = np.where(bits[row, col], white, black)
        img[v0:v1, u0:u1] = val.reshape(v1 - v0, ss, u1 - u0, ss).mean(axis=(1, 3))
        if blur_px > 0:
            img[v0:v1, u0:u1] = _blur(img[v0:v1, u0:u1], blur_px)
    if isinstance(noise, np.ndarray):
        img += noise
    elif noise > 0:
        img += (rng or np.random.default_rng()).normal(0.0, noise, img.shape).astype(np.float32)
    return np.clip(img + 0.5, 0, 255).astype(np.uint8)
