/**
 * Connects Luna's face core (web/face-core.js) to the robot's face bridge (face_bridge_node.py).
 * The bridge streams Server-Sent Events: {"type":"tap"} | {"type":"gaze","x":0.4,"y":0} | {"type":"thinking"} | …
 * Only events the face core understands are forwarded; everything else is ignored, so a broken
 * or hostile stream cannot throw inside the face loop.
 *
 * Push-to-talk: with `talkTarget` (the face on the head's screen), a tap or a hold — not a stroke — POSTs to the
 * bridge's /talk and Luna listens for one question. `canTalk()` false (asleep on the dock, microphones cut) keeps
 * the touch local: onTalk(kind, false) instead of a POST.
 *
 *   import { connectRobot } from './robot-link.js';
 *   const stop = connectRobot(face, { url: 'http://127.0.0.1:8765/events', onStatus: s => …,
 *                                     talkTarget: canvas, canTalk: face.canTalk });
 */
import { FACE_EVENTS } from './face-core.js';

export const ROBOT_FACE_EVENTS = FACE_EVENTS;

const clamp = (n, lo, hi) => Math.min(hi, Math.max(lo, n));

export function parseRobotEvent(raw) {
  let msg;
  try { msg = JSON.parse(raw); } catch { return null; }
  if (!msg || typeof msg !== 'object' || !ROBOT_FACE_EVENTS.includes(msg.type)) return null;
  if (msg.type === 'gaze') {
    const x = Number(msg.x), y = Number(msg.y);
    if (!Number.isFinite(x) || !Number.isFinite(y)) return null;
    return { type: 'gaze', data: { x: clamp(x, -1, 1), y: clamp(y, -1, 1) } };
  }
  return { type: msg.type, data: {} };
}

// Touch thresholds (kept from R1's phone face): a tap is ≤ 360 ms and ≤ 14 px, a hold fires at 430 ms.
// Moving further than 14 px is a stroke (petting), not a question.
export const TALK_GESTURE = Object.freeze({ tapMs: 360, holdMs: 430, slopPx: 14 });

/** Calls onTalk('tap' | 'hold' | 'key') for a tap, a hold or Enter/Space on `target`; returns an unsubscribe. */
export function watchTalkGestures(target, onTalk, {
  tapMs = TALK_GESTURE.tapMs, holdMs = TALK_GESTURE.holdMs, slopPx = TALK_GESTURE.slopPx,
  releaseTarget = target, now = () => performance.now(), setTimer = setTimeout, clearTimer = clearTimeout,
} = {}) {
  let active = null;
  const end = () => { if (active) clearTimer(active.timer); active = null; };
  const down = ev => {
    if (!ev.isPrimary || (ev.button ?? 0) !== 0 || active) return;
    const touch = { id: ev.pointerId, x: ev.clientX, y: ev.clientY, t: now(), fired: false };
    touch.timer = setTimer(() => {
      if (active === touch) { touch.fired = true; onTalk('hold'); }
    }, holdMs);
    active = touch;
  };
  const move = ev => {
    if (active && ev.pointerId === active.id && Math.hypot(ev.clientX - active.x, ev.clientY - active.y) > slopPx) end();
  };
  const up = ev => {
    if (!active || ev.pointerId !== active.id) return;
    move(ev);
    const touch = active;
    end();
    if (touch && !touch.fired && now() - touch.t <= tapMs) onTalk('tap');
  };
  const cancel = ev => { if (active && ev.pointerId === active.id) end(); };
  const key = ev => { if (!ev.repeat && (ev.key === 'Enter' || ev.key === ' ')) onTalk('key'); };
  const listeners = [[target, 'pointerdown', down], [target, 'pointermove', move], [releaseTarget, 'pointerup', up],
    [releaseTarget, 'pointercancel', cancel], [target, 'keydown', key]];
  for (const [t, type, fn] of listeners) t.addEventListener(type, fn);
  return () => { end(); for (const [t, type, fn] of listeners) t.removeEventListener(type, fn); };
}

/** POST /talk to the face bridge; resolves true if the bridge took it. Never throws. */
export function sendTalk(talkUrl, { fetchImpl = globalThis.fetch, onStatus = () => {} } = {}) {
  if (!fetchImpl) { onStatus('talk-unavailable'); return Promise.resolve(false); }
  return Promise.resolve()
    .then(() => fetchImpl(talkUrl, { method: 'POST', mode: 'cors', cache: 'no-store', keepalive: true }))
    .then(res => { onStatus(res.ok ? 'talk-sent' : 'talk-refused'); return res.ok; },
          () => { onStatus('talk-failed'); return false; });
}

export function connectRobot(face, {
  url, onStatus = () => {}, EventSourceImpl = globalThis.EventSource,
  talkTarget = null, talkUrl = null, fetchImpl = globalThis.fetch, releaseTarget = globalThis.window ?? talkTarget,
  canTalk = () => true, onTalk = () => {},
} = {}) {
  if (!EventSourceImpl || !url) { onStatus('unavailable'); return () => {}; }
  const source = new EventSourceImpl(url);
  source.onopen = () => onStatus('connected');
  source.onerror = () => onStatus('reconnecting');   // EventSource retries on its own (bridge sends retry: 2000)
  source.onmessage = ev => {
    const parsed = parseRobotEvent(ev.data);
    if (parsed) face.dispatch(parsed.type, parsed.data);
  };
  const talkTo = talkTarget ? (talkUrl ?? new URL('talk', url).href) : null;
  const stopTalk = talkTo
    ? watchTalkGestures(talkTarget, kind => {
      const send = canTalk();
      onTalk(kind, send);
      if (send) sendTalk(talkTo, { fetchImpl, onStatus });
    }, { releaseTarget })
    : () => {};
  return () => { stopTalk(); source.close(); onStatus('closed'); };
}
