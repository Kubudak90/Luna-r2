"""ReSpeaker Mic Array v2.0 (Seeed, XMOS XVF-3000, VID 2886 PID 0018) host control over USB, without Seeed's
tuning.py.

Protocol as in Seeed's respeaker/usb_4_mic_array tuning.py (read 2026-10-25): a vendor control transfer to the
device, bRequest 0, wIndex = parameter id. A read uses wValue = 0x80 | offset, plus 0x40 for an int parameter, and
returns 8 bytes = two little-endian int32 (an int is the first; a float is first · 2^second).
  DOAANGLE       id 21, offset 0, int 0..359: direction of arrival ("orientation depends on build configuration")
  SPEECHDETECTED id 19, offset 22, int 0/1
  VOICEACTIVITY  id 19, offset 32, int 0/1
Which way the board's 0° points and which way it counts is not documented: `board_to_base` takes both as calibration
values. The array lies flat on the shoulder plate, fixed to the body (urdf mic_array), so after that calibration the
angle is a base_link bearing; nothing turns it but the whole robot.

No mute over USB: the privacy slider cuts the array's USB power (docs/TASARIM_R2.md, "Gizlilik"), so the array
simply disappears, and with it the speaker output: with the slider closed Luna neither hears nor talks, the face
goes on. The software mute (/luna/audio/mute) only drops audio in audio_node.

Audio is USB Audio Class 1.0 (ALSA card "ReSpeaker 4 Mic Array (UAC1.0)"), 16 kHz. With Seeed's default 6-channel
firmware channel 0 is the processed beam for speech recognition, 1–4 the raw mics, 5 the playback reference; the
1-channel firmware has the processed beam only. Playback through the same card (its 3.5 mm jack, into the speaker
amplifier) is what the echo canceller hears as reference, so Luna's voice goes there too."""
import re
import struct
from dataclasses import dataclass

VID, PID = 0x2886, 0x0018
REQ_IN = 0xC0           # CTRL_IN | CTRL_TYPE_VENDOR | CTRL_RECIPIENT_DEVICE
READ_BIT, INT_BIT = 0x80, 0x40
ALSA_NAME = 'ReSpeaker 4 Mic Array'
CARD_ID = 'ArrayUAC10'  # the card id ALSA gives it (/proc/asound/cards "[ArrayUAC10 ]"), used while it is unplugged


@dataclass(frozen=True)
class Param:
    pid: int
    offset: int
    kind: str          # 'int' | 'float'


PARAMS = {
    'DOAANGLE': Param(21, 0, 'int'),
    'SPEECHDETECTED': Param(19, 22, 'int'),
    'VOICEACTIVITY': Param(19, 32, 'int'),
}


class ArrayError(Exception):
    pass


@dataclass(frozen=True)
class DoaReading:
    azimuth_deg: int
    speech: bool


class ArrayControl:
    """`dev` is a pyusb device (or anything with its ctrl_transfer signature)."""

    def __init__(self, dev, timeout_ms=500):
        self.dev, self.timeout_ms = dev, timeout_ms

    def read(self, name):
        p = PARAMS[name]
        cmd = READ_BIT | p.offset | (INT_BIT if p.kind == 'int' else 0)
        raw = bytes(self.dev.ctrl_transfer(REQ_IN, 0, cmd, p.pid, 8, self.timeout_ms))
        if len(raw) < 8:
            raise ArrayError(f'{name}: short reply ({len(raw)} bytes)')
        value, exponent = struct.unpack('<ii', raw[:8])
        return value if p.kind == 'int' else value * 2.0 ** exponent

    def doa(self):
        return DoaReading(self.read('DOAANGLE') % 360, bool(self.read('SPEECHDETECTED')))


def wrap_deg(a):
    """Angle in (-180, 180]."""
    a = (a + 180.0) % 360.0 - 180.0
    return 180.0 if a == -180.0 else a


def board_to_base(azimuth_deg, zero_deg=0.0, direction=1):
    """Board azimuth → base_link bearing (+ = Luna's left). zero_deg: board reading for a voice straight ahead of the
    face; direction: +1 if the board counts counter-clockwise seen from above, -1 if not (both CALIBRATE)."""
    return wrap_deg(direction * (azimuth_deg - zero_deg))


def find_alsa_device(cards_text, name=ALSA_NAME):
    """ALSA device for the array from /proc/asound/cards text, by card id (stable across USB re-plugs, unlike the
    card number)."""
    for line in cards_text.splitlines():
        m = re.match(r'\s*\d+\s+\[(\S+)\s*\]:.*' + re.escape(name), line)
        if m:
            return f'plughw:CARD={m.group(1)},DEV=0'
    return None


def _import_usb():
    import usb.core
    return usb.core


def open_usb(vid=VID, pid=PID, import_usb=_import_usb):
    """The array's pyusb device. ArrayError when pyusb is missing or the array is not on the bus (unplugged, or its
    power cut by the privacy slider)."""
    try:
        core = import_usb()
    except ImportError as e:
        raise ArrayError('pyusb is not installed (python3-usb); DoA needs it') from e
    dev = core.find(idVendor=vid, idProduct=pid)
    if dev is None:
        raise ArrayError(f'mic array {vid:04x}:{pid:04x} not on USB')
    return dev
