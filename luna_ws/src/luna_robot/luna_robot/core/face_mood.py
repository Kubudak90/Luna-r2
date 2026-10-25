"""Robot state → events understood by Luna's face (web/face.js, the R2 screen face; robot-link.js forwards them).

Moments (the face shows them, then goes back): tap (a bump surprises Luna), stroke (happy: the dock contacts
touched), curious (an attentive glance).
States (the face keeps them until told otherwise): sleep / wake (the dock decides), gaze {x, y} / gaze-clear
(where the eyes look; without gaze they wander by themselves), listening / thinking / speaking / idle (voice),
low-battery / battery-ok, muted / unmuted.
R1's phone face only knew the moments, sleep/wake and gaze; R2's face has a pose for every voice state, so the
voice no longer borrows `curious` and `stroke`. This module is the single place that decides how Luna "feels" about
what the robot is doing."""

FACE_EVENTS = {'tap', 'stroke', 'curious', 'sleep', 'wake', 'gaze', 'gaze-clear', 'listening', 'thinking', 'speaking',
               'idle', 'low-battery', 'battery-ok', 'muted', 'unmuted'}
DOCK_SLEEP = ('CHARGING', 'DOCKED')
LOW = ('seek_dock', 'critical')


class MoodMapper:
    def __init__(self, gaze_deadband=0.04):
        self.prev = {'bumpers': 0, 'dock': 'IDLE', 'battery': 'run', 'gaze': None, 'muted': False}
        self.deadband = gaze_deadband

    def update(self, *, bumpers, dock_state, battery_action, gaze, muted=False):
        """Face events triggered by this state change (possibly empty). gaze: (x, y) or None (eyes on their own)."""
        ev = []
        p = self.prev
        if bumpers and not p['bumpers']:
            ev.append({'type': 'tap'})                       # a bump surprises Luna
        if dock_state in DOCK_SLEEP and p['dock'] not in DOCK_SLEEP:
            ev.append({'type': 'sleep'})                     # resting on the dock
        if dock_state in ('UNDOCK', 'IDLE', 'SEEK') and p['dock'] in DOCK_SLEEP:
            ev.append({'type': 'wake'})
        if dock_state == 'CONTACT' and p['dock'] == 'APPROACH':
            ev.append({'type': 'stroke'})                    # happy: contacts touched
        if battery_action in LOW and p['battery'] not in LOW:
            ev.append({'type': 'low-battery'})
        elif battery_action not in LOW and p['battery'] in LOW:
            ev.append({'type': 'battery-ok'})
        if muted != p['muted']:
            ev.append({'type': 'muted' if muted else 'unmuted'})
        if dock_state not in DOCK_SLEEP:
            if gaze is None and p['gaze'] is not None:
                ev.append({'type': 'gaze-clear'})
                p['gaze'] = None
            elif gaze is not None and (p['gaze'] is None or abs(gaze[0] - p['gaze'][0]) > self.deadband
                                       or abs(gaze[1] - p['gaze'][1]) > self.deadband):
                ev.append({'type': 'gaze', 'x': round(gaze[0], 3), 'y': round(gaze[1], 3)})
                p['gaze'] = gaze
        p.update(bumpers=bumpers, dock=dock_state, battery=battery_action, muted=muted)
        assert all(e['type'] in FACE_EVENTS for e in ev)
        return ev

    @staticmethod
    def snapshot(*, bumpers, dock_state, battery_action, gaze, muted=False, voice='idle'):
        """The events that bring a connecting face to the current state. Pure: the live mapper is not touched. Takes
        the same state as update() plus the voice state; only lasting state is replayed (asleep or awake, where Luna
        looks, voice, battery, mute). Bumps and contact are moments, already over.

        Off the dock the snapshot always starts with `wake`: a reconnecting page may still be asleep from before the
        drop, and on an awake face `wake` only restarts its idle timer."""
        if dock_state in DOCK_SLEEP:
            ev = [{'type': 'sleep'}]
        else:
            ev = [{'type': 'wake'}]
            if gaze is not None:
                ev.append({'type': 'gaze', 'x': round(gaze[0], 3), 'y': round(gaze[1], 3)})   # after `wake`
            if voice != 'idle':
                ev.append({'type': voice})
        if battery_action in LOW:
            ev.append({'type': 'low-battery'})
        if muted:
            ev.append({'type': 'muted'})
        assert all(e['type'] in FACE_EVENTS for e in ev)
        return ev


VOICE_STATES = ('listening', 'thinking', 'speaking', 'idle')


def voice_state(*, listening, thinking, speaking):
    if speaking:
        return 'speaking'
    if thinking:
        return 'thinking'
    return 'listening' if listening else 'idle'


class VoiceMood:
    """Voice state → one face event per change. Nothing while Luna sleeps on the dock: there the dock owns sleep and
    wake, and the face would show a voice pose on closed eyes."""

    def __init__(self):
        self.last = 'idle'

    def update(self, *, voice, dock_state):
        if dock_state in DOCK_SLEEP:
            self.last = 'idle'
            return []
        if voice == self.last:
            return []
        self.last = voice
        return [{'type': voice}]


def fresh_gaze(gaze, age_s, hold_s=1.0):
    """Attention's gaze while it is fresh; None afterwards, so the face goes back to looking around by itself."""
    return gaze if gaze is not None and age_s <= hold_s else None
