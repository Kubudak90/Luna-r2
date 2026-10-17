"""ReSpeaker XVF3800 (Seeed 101991441) host control over USB, without the xvf_host binary.

Protocol as in Seeed's official Python example (wiki "reSpeaker XVF3800 Control with Python", read 2026-10-17):
a vendor control transfer to the device with bRequest 0, wValue = command id (| 0x80 for a read), wIndex =
resource id. A read returns one status byte, then the payload. Status 0 = success and 64 = retry are from
Seeed's I2C command page; the USB page does not document the status byte.

DOA_VALUE (GPO servicer) is Seeed's [azimuth 0–359°, speech 0/1]. Which way the board's 0° points and which
way it counts is not documented: `board_to_head` takes both as calibration values. The board sits in the head
(R07), so the angle is head-relative; attention adds the head pan.

Audio itself is plain USB Audio Class 2.0 (ALSA card "reSpeaker XVF3800 4-Mic Array"). Playback through the same
card is what the board's echo canceller can hear as reference, so Luna's speaker output goes there too."""
import re
import struct
import time
from dataclasses import dataclass

VID, PID = 0x2886, 0x001A
REQ_OUT = 0x40          # CTRL_OUT | CTRL_TYPE_VENDOR | CTRL_RECIPIENT_DEVICE
REQ_IN = 0xC0           # CTRL_IN  | CTRL_TYPE_VENDOR | CTRL_RECIPIENT_DEVICE
READ_BIT = 0x80
STATUS_OK, STATUS_RETRY = 0, 64


@dataclass(frozen=True)
class Param:
    resid: int
    cmdid: int
    nbytes: int
    fmt: str


PARAMS = {
    'VERSION': Param(48, 0, 3, '<3B'),
    'DOA_VALUE': Param(20, 18, 4, '<2H'),                       # [azimuth deg 0..359, speech 0/1]
    'AEC_AZIMUTH_VALUES': Param(33, 75, 16, '<4f'),             # rad: beam 1, beam 2, free-running, auto-select
    'AEC_SPENERGY_VALUES': Param(33, 80, 16, '<4f'),            # speech energy, same beam order
    'AUDIO_MGR_SELECTED_AZIMUTHS': Param(35, 11, 8, '<2f'),     # rad: processed DoA (NaN = no speech), auto beam
}


class XvfError(Exception):
    pass


@dataclass(frozen=True)
class DoaReading:
    azimuth_deg: int
    speech: bool


class XvfControl:
    """`dev` is a pyusb device (or anything with its ctrl_transfer signature)."""

    def __init__(self, dev, timeout_ms=500, retries=5, sleep=time.sleep):
        self.dev, self.timeout_ms, self.retries, self.sleep = dev, timeout_ms, retries, sleep

    def read(self, name):
        p = PARAMS[name]
        for _ in range(self.retries + 1):
            raw = bytes(self.dev.ctrl_transfer(REQ_IN, 0, READ_BIT | p.cmdid, p.resid, p.nbytes + 1, self.timeout_ms))
            if not raw:
                raise XvfError(f'{name}: empty reply')
            if raw[0] == STATUS_OK:
                if len(raw) < p.nbytes + 1:
                    raise XvfError(f'{name}: short reply ({len(raw)} bytes)')
                return struct.unpack(p.fmt, raw[1:1 + p.nbytes])
            if raw[0] != STATUS_RETRY:
                raise XvfError(f'{name}: status {raw[0]}')
            self.sleep(0.005)
        raise XvfError(f'{name}: device stayed busy')

    def doa(self):
        azimuth, speech = self.read('DOA_VALUE')
        return DoaReading(azimuth % 360, bool(speech))


def wrap_deg(a):
    """Angle in (-180, 180]."""
    a = (a + 180.0) % 360.0 - 180.0
    return 180.0 if a == -180.0 else a


def board_to_head(azimuth_deg, zero_deg=0.0, direction=1):
    """Board azimuth → head bearing in the head pan convention. zero_deg: board reading for a voice straight
    ahead of the face; direction: +1 if the board counts the same way as pan, -1 if not (both CALIBRATE)."""
    return wrap_deg(direction * (azimuth_deg - zero_deg))


def find_alsa_device(cards_text, name='XVF3800'):
    """ALSA device for the array from /proc/asound/cards text, by card id (stable across USB re-plugs,
    unlike the card number)."""
    for line in cards_text.splitlines():
        m = re.match(r'\s*\d+\s+\[(\S+)\s*\]:.*' + re.escape(name), line)
        if m:
            return f'plughw:CARD={m.group(1)},DEV=0'
    return None


def _import_usb():
    import usb.core
    return usb.core


def open_usb(vid=VID, pid=PID, import_usb=_import_usb):
    try:
        core = import_usb()
    except ImportError as e:
        raise XvfError('pyusb is not installed (python3-usb); DoA needs it') from e
    dev = core.find(idVendor=vid, idProduct=pid)
    if dev is None:
        raise XvfError(f'XVF3800 {vid:04x}:{pid:04x} not found on USB')
    return dev
