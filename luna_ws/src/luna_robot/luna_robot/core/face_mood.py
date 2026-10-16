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
        """Return the events that bring a freshly connected face to the current state. Pure: the live
        mapper is not touched. Takes the same state as update(); only lasting state is replayed (asleep
        on the dock, where Luna looks). Bumps, contact and a low battery are moments, already over."""
        if dock_state in ('CHARGING', 'DOCKED'):
            ev = [{'type': 'sleep'}]
        else:
            x, y = round(gaze[0], 3), round(gaze[1], 3)
            ev = [{'type': 'gaze', 'x': x, 'y': y}] if x or y else []   # a fresh face already looks ahead
        assert all(e['type'] in FACE_EVENTS for e in ev)
        return ev
