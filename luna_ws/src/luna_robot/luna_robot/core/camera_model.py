"""Pinhole models of R2's cameras: the Gemini 336 colour camera (dock tag) and the Raspberry Pi Camera v2 over the
screen (faces). Points are in the ROS optical frame: z forward out of the lens, x right, y down."""
import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Pinhole:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float

    @staticmethod
    def from_hfov(width, height, hfov_deg):
        f = width / 2 / math.tan(math.radians(hfov_deg) / 2)
        return Pinhole(width, height, f, f, (width - 1) / 2, (height - 1) / 2)

    @staticmethod
    def from_camera_info(info):
        """`info`: dict as written by camera_calibration (camera_info YAML)."""
        k = info['camera_matrix']['data']
        return Pinhole(int(info['image_width']), int(info['image_height']), k[0], k[4], k[2], k[5])

    def project(self, p):
        """(u, v) pixel of an optical-frame point, or None behind the camera."""
        x, y, z = p
        if z <= 1e-6:
            return None
        return self.cx + self.fx * x / z, self.cy + self.fy * y / z

    def sees(self, p, half_size=0.0, margin_px=2.0):
        """True when a square of `half_size` m facing the camera, centred at p, projects fully inside the image."""
        uv = self.project(p)
        if uv is None:
            return False
        r = self.fx * half_size / p[2]
        return (margin_px + r <= uv[0] <= self.width - 1 - margin_px - r and
                margin_px + r <= uv[1] <= self.height - 1 - margin_px - r)


# Gemini 336 colour at 1280 × 720 (the driver scales its 1920 × 1080 sensor), 86° × 55° per Orbbec. Nominal: the real
# driver publishes the factory calibration on /camera/color/camera_info; sim_camera_node and the checks use this.
GEMINI_COLOR_720P = Pinhole.from_hfov(1280, 720, 86.0)
# Raspberry Pi Camera v2 (IMX219, 3.04 mm) binned to 640 × 480 for the face detector, 62.2° × 48.8°. Nominal.
PICAM_V2_480P = Pinhole.from_hfov(640, 480, 62.2)
