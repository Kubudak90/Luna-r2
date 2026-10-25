"""Faces in the face camera's image → the one face Luna looks at, as a base_link bearing and elevation (degrees,
bearing + = Luna's left, elevation + = up), for attention (eye contact, and a body turn before the face leaves the
picture). Pure Python.

The camera (Raspberry Pi Camera v2, IMX219) sits over the screen, pitched up 12° (urdf face_camera_link). A pixel
becomes a ray in the optical frame (x right, y down, z out of the lens), and the URDF rotation turns it into
base_link. Detections come from OpenCV's YuNet face detector in face_track_node as (u, v, w, h, score) with (u, v)
the box centre.

Which face: the one already followed while one is still near it (it may be the child who is talking), otherwise the
largest, i.e. the nearest. Small or unsure detections are ignored: a face narrower than min_width_px is further away
than Luna can make eye contact (≈ 3 m at 640 px), and YuNet scores below min_score are mostly posters and toys."""
import math
from dataclasses import dataclass
from typing import NamedTuple

from .camera_model import Pinhole


class Face(NamedTuple):
    bearing_deg: float
    elevation_deg: float
    width_px: float


@dataclass(frozen=True)
class FaceCamera:
    pinhole: Pinhole
    rot: tuple                 # optical frame → base_link rotation (3 × 3 rows), from the URDF

    def direction(self, u, v):
        """(bearing deg, elevation deg) in base_link of the ray through pixel (u, v)."""
        p = self.pinhole
        ray = ((u - p.cx) / p.fx, (v - p.cy) / p.fy, 1.0)
        dx, dy, dz = (sum(self.rot[i][k] * ray[k] for k in range(3)) for i in range(3))
        return math.degrees(math.atan2(dy, dx)), math.degrees(math.atan2(dz, math.hypot(dx, dy)))


@dataclass(frozen=True)
class SelectorConfig:
    min_score: float = 0.75
    min_width_px: float = 24.0
    keep_deg: float = 12.0     # the followed face may move this much between frames and still be the same one


class FaceSelector:
    def __init__(self, camera, cfg=SelectorConfig()):
        self.camera, self.cfg = camera, cfg
        self.current = None

    def update(self, detections):
        """detections: iterable of (u, v, w, h, score) → Face or None."""
        faces = []
        for u, v, w, _h, score in detections:
            if score >= self.cfg.min_score and w >= self.cfg.min_width_px:
                faces.append(Face(*self.camera.direction(u, v), w))
        if not faces:
            self.current = None
            return None
        pick = None
        if self.current is not None:
            near = [f for f in faces if abs(f.bearing_deg - self.current.bearing_deg) <= self.cfg.keep_deg]
            if near:
                pick = min(near, key=lambda f: abs(f.bearing_deg - self.current.bearing_deg))
        self.current = pick or max(faces, key=lambda f: f.width_px)
        return self.current
