"""Robot state → events understood by the Luna face core (Telefon Yüzü R01, dist/face/core.js).

The face accepts: tap, stroke, curious, sleep, wake, gaze {x, y}, gaze-clear. Anything else is
rejected by the browser adapter, so this module is the single place that decides how Luna
"feels" about what the robot is doing."""

FACE_EVENTS = {'tap', 'stroke', 'curious', 'sleep', 'wake', 'gaze', 'gaze-clear'}


class MoodMapper:
    def __init__(self, gaze_deadband=0.04):
        self.prev = {'bumpers': 0, 'dock': 'IDLE', 'battery': 'run', 'gaze': (0.0, 0.0)}
        self.deadband = gaze_deadband

    def update(self, *, bumpers, dock_state, battery_action, gaze):
        """Return the list of face events triggered by this state change (possibly empty)."""
        ev = []
        p = self.prev
        if bumpers and not p['bumpers']:
            ev.append({'type': 'tap'})                       # a bump surprises Luna
        if dock_state in ('CHARGING', 'DOCKED') and p['dock'] not in ('CHARGING', 'DOCKED'):
            ev.append({'type': 'sleep'})                     # resting on the dock
        if dock_state in ('UNDOCK', 'IDLE', 'SEEK') and p['dock'] in ('CHARGING', 'DOCKED'):
            ev.append({'type': 'wake'})
        if dock_state == 'CONTACT' and p['dock'] == 'APPROACH':
            ev.append({'type': 'stroke'})                    # happy: contacts touched
        if battery_action in ('seek_dock', 'critical') and p['battery'] not in ('seek_dock', 'critical'):
            ev.append({'type': 'curious'})
        gx, gy = gaze
        if dock_state not in ('CHARGING', 'DOCKED') and (abs(gx - p['gaze'][0]) > self.deadband or abs(gy - p['gaze'][1]) > self.deadband):
            ev.append({'type': 'gaze', 'x': round(gx, 3), 'y': round(gy, 3)})
            p['gaze'] = (gx, gy)
        p.update(bumpers=bumpers, dock=dock_state, battery=battery_action)
        assert all(e['type'] in FACE_EVENTS for e in ev)
        return ev

    @staticmethod
    def snapshot(*, bumpers, dock_state, battery_action, gaze):
        """Return the events that bring a connecting face to the current state. Pure: the live mapper is
        not touched. Takes the same state as update(); only lasting state is replayed (asleep or awake,
        where Luna looks). Bumps, contact and a low battery are moments, already over.

        Off the dock the snapshot always starts with `wake`: a reconnecting page may still be asleep
        from before the drop, and on an awake face `wake` only restarts its idle timer."""
        if dock_state in ('CHARGING', 'DOCKED'):
            ev = [{'type': 'sleep'}]
        else:
            ev = [{'type': 'wake'}]
            x, y = round(gaze[0], 3), round(gaze[1], 3)
            if x or y:
                ev.append({'type': 'gaze', 'x': x, 'y': y})        # after `wake`, which clears the gaze
        assert all(e['type'] in FACE_EVENTS for e in ev)
        return ev


# Voice: listening / thinking / speaking shown with the face's existing vocabulary only. The face core has no
# "talking" pose, so: `curious` (attentive look, 1.6 s in the face core) while Luna listens or waits for the
# cloud, `stroke` (happy, 2.8 s) while Luna speaks, each re-sent before it runs out. Every event also counts as
# activity in the face core, so Luna does not doze off mid-conversation.
VOICE_EVENTS = {'listening': 'curious', 'thinking': 'curious', 'speaking': 'stroke', 'idle': None}
VOICE_REFRESH_S = {'curious': 1.5, 'stroke': 2.5}
DOCK_SLEEP = ('CHARGING', 'DOCKED')


def voice_state(*, listening, thinking, speaking):
    if speaking:
        return 'speaking'
    if thinking:
        return 'thinking'
    return 'listening' if listening else 'idle'


class VoiceMood:
    def __init__(self):
        self.last = None            # (event type, time sent)

    def update(self, now, *, voice, dock_state):
        """Face events for the current voice state (call periodically). Nothing while Luna sleeps on the dock:
        any of these events would wake the face core, and on the dock the dock owns sleep and wake."""
        kind = VOICE_EVENTS[voice]
        if dock_state in DOCK_SLEEP or kind is None:
            self.last = None
            return []
        if self.last and self.last[0] == kind and now - self.last[1] < VOICE_REFRESH_S[kind]:
            return []
        self.last = (kind, now)
        ev = [{'type': kind}]
        assert all(e['type'] in FACE_EVENTS for e in ev)
        return ev


def pick_gaze(head_gaze, attention_gaze, attention_age_s, hold_s=0.5):
    """Attention's eye gaze (looking at a voice) wins while fresh; otherwise the eyes follow the head."""
    if attention_gaze is not None and attention_age_s <= hold_s:
        return attention_gaze
    return head_gaze
