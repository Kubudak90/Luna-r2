"""Pinhole model of the head cameras (Arducam inVisCam B0630, 1/2.8" STARVIS 2, 3.7 mm M12 lens, 82°×66° FOV per
the UCTRONICS listing). Points are in the ROS optical frame: z forward out of the lens, x right, y down."""
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


# 1280×720 MJPG is the chosen mode (B0630 offers YUY2 only up to 640×480). Nominal until calibrated: the lens
# has < −13 % distortion and the 82° figure is the vendor's, so camera_info/*.yaml must be replaced by a real
# calibration (ros2 run camera_calibration cameracalibrator) before distances are trusted near the image edges.
B0630_720P = Pinhole.from_hfov(1280, 720, 82.0)
