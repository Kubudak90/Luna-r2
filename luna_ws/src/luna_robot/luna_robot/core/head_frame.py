"""XVF3800 direction of arrival in the head → base_link bearing (TF head_tilt_link → base_link, urdf/luna.urdf).

R07: the array sits under the head top shell, logo face up (board plane = head_tilt_link x-y plane), USB-C toward
the robot's left (+y), and turns with pan and tilt. Translations are irrelevant for a far-field direction, so
only the two joint rotations matter:
  pan  + = left, about +z (URDF head_pan)
  tilt + = nose up (head.py / attention convention). The URDF head_tilt axis is +y, so its joint angle is −tilt.

A planar array measures the azimuth of the voice projected onto its own plane; it cannot see elevation. With
the head tilted, that projection depends on how high the voice is, so a speaker elevation above base_link's
horizontal plane is assumed (speaker_elevation_deg, CALIBRATE; 25° ≈ an adult 1–2 m away, Luna 589 mm tall).
For true elevations 0–40° the bearing error stays under 6° at the ±12° tilt limit (test_audio.HeadFrame)."""
import math

PAN_LIMIT_DEG, TILT_LIMIT_DEG = 60.0, 12.0          # core/head.py mechanical limits


def _wrap(a):
    a = (a + 180.0) % 360.0 - 180.0
    return 180.0 if a == -180.0 else a


def _clamp(v, lim):
    return max(-lim, min(lim, v))


def _head_to_base_vec(v, pan_deg, tilt_deg):
    x, y, z = v
    t, p = math.radians(_clamp(tilt_deg, TILT_LIMIT_DEG)), math.radians(_clamp(pan_deg, PAN_LIMIT_DEG))
    x, z = x * math.cos(t) - z * math.sin(t), x * math.sin(t) + z * math.cos(t)          # nose up by t
    return x * math.cos(p) - y * math.sin(p), x * math.sin(p) + y * math.cos(p), z


def _base_to_head_vec(v, pan_deg, tilt_deg):
    x, y, z = v
    t, p = math.radians(_clamp(tilt_deg, TILT_LIMIT_DEG)), math.radians(_clamp(pan_deg, PAN_LIMIT_DEG))
    x, y = x * math.cos(p) + y * math.sin(p), -x * math.sin(p) + y * math.cos(p)
    return x * math.cos(t) + z * math.sin(t), y, -x * math.sin(t) + z * math.cos(t)


def head_to_base(azimuth_head_deg, pan_deg, tilt_deg, speaker_elevation_deg=25.0):
    """Array azimuth (head frame, after board calibration) → base_link azimuth in (-180, 180]. The joint angles are
    held to the head limits. The voice lies in the plane through the array normal n and the measured in-plane
    direction u: s = cos(b)·u + sin(b)·n, with b chosen so that s has the assumed elevation in base_link."""
    a = math.radians(azimuth_head_deg)
    u = _head_to_base_vec((math.cos(a), math.sin(a), 0.0), pan_deg, tilt_deg)
    n = _head_to_base_vec((0.0, 0.0, 1.0), pan_deg, tilt_deg)
    r, g = math.hypot(u[2], n[2]), math.atan2(u[2], n[2])
    b = math.asin(_clamp(math.sin(math.radians(speaker_elevation_deg)) / r, 1.0)) - g
    sx, sy = math.cos(b) * u[0] + math.sin(b) * n[0], math.cos(b) * u[1] + math.sin(b) * n[1]
    return _wrap(math.degrees(math.atan2(sy, sx)))


def base_to_head(azimuth_deg, elevation_deg, pan_deg, tilt_deg):
    """What the head-mounted planar array measures for a voice at this base_link azimuth/elevation (mock, tests)."""
    a, e = math.radians(azimuth_deg), math.radians(elevation_deg)
    x, y, _ = _base_to_head_vec((math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e)), pan_deg, tilt_deg)
    return _wrap(math.degrees(math.atan2(y, x)))
