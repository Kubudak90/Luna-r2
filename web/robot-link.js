/**
 * Connects the Luna face core (Telefon Yüzü R01, dist/face/core.js) to the robot's face bridge.
 * The bridge streams Server-Sent Events: {"type":"tap"} | {"type":"gaze","x":0.4,"y":0} | …
 * Only events the face core understands are forwarded; everything else is ignored, so a broken
 * or hostile stream cannot throw inside the face loop.
 *
 *   import { connectRobot } from './robot-link.js';
 *   const stop = connectRobot(face, { url: 'http://127.0.0.1:8765/events', onStatus: s => … });
 */
export const ROBOT_FACE_EVENTS = Object.freeze(['tap', 'stroke', 'curious', 'sleep', 'wake', 'gaze', 'gaze-clear']);

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

export function connectRobot(face, { url, onStatus = () => {}, EventSourceImpl = globalThis.EventSource } = {}) {
  if (!EventSourceImpl || !url) { onStatus('unavailable'); return () => {}; }
  const source = new EventSourceImpl(url);
  source.onopen = () => onStatus('connected');
  source.onerror = () => onStatus('reconnecting');   // EventSource retries on its own (bridge sends retry: 2000)
  source.onmessage = ev => {
    const parsed = parseRobotEvent(ev.data);
    if (parsed) face.dispatch(parsed.type, parsed.data);
  };
  return () => { source.close(); onStatus('closed'); };
}
