// node --test web/robot-link.test.mjs   (uses the real face core from Telefon Yüzü R01)
import test from 'node:test';
import assert from 'node:assert/strict';
import { fileURLToPath } from 'node:url';
import path from 'node:path';
import { connectRobot, parseRobotEvent } from './robot-link.js';

const here = path.dirname(fileURLToPath(import.meta.url));
const corePath = path.join(here, '..', '..', 'Telefon Yüzü R01 2026-10-07', 'Kaynaklar', 'luna-face', 'dist', 'face', 'core.js');
const { createFaceCore } = await import(corePath);

class FakeSource {
  constructor(url) { this.url = url; FakeSource.last = this; this.closed = false; }
  emit(obj) { this.onmessage?.({ data: typeof obj === 'string' ? obj : JSON.stringify(obj) }); }
  close() { this.closed = true; }
}

test('only face-core events pass, gaze is clamped', () => {
  assert.deepEqual(parseRobotEvent('{"type":"tap"}'), { type: 'tap', data: {} });
  assert.deepEqual(parseRobotEvent('{"type":"gaze","x":4,"y":-0.5}'), { type: 'gaze', data: { x: 1, y: -0.5 } });
  assert.equal(parseRobotEvent('{"type":"explode"}'), null);
  assert.equal(parseRobotEvent('not json'), null);
  assert.equal(parseRobotEvent('{"type":"gaze","x":"a","y":0}'), null);
});

test('robot bump surprises the real face, docking puts it to sleep, undock wakes it', () => {
  const face = createFaceCore({ seed: 7 });
  const statuses = [];
  const stop = connectRobot(face, { url: 'http://127.0.0.1:8765/events', EventSourceImpl: FakeSource, onStatus: s => statuses.push(s) });
  const src = FakeSource.last;
  src.onopen();
  face.step(1000);
  src.emit({ type: 'tap' });
  face.step(600);
  assert.equal(face.sample().mode, 'surprised');
  src.emit({ type: 'explode' });                 // ignored, no throw
  src.emit('garbage');
  src.emit({ type: 'sleep' });
  face.step(500);
  assert.equal(face.sample().mode, 'asleep');
  src.emit({ type: 'wake' });
  face.step(500);
  assert.notEqual(face.sample().mode, 'asleep');
  stop();
  assert.equal(src.closed, true);
  assert.deepEqual(statuses, ['connected', 'closed']);
});

test('missing EventSource degrades quietly', () => {
  const seen = [];
  const stop = connectRobot({ dispatch() { throw new Error('no'); } }, { url: 'x', EventSourceImpl: undefined, onStatus: s => seen.push(s) });
  stop();
  assert.deepEqual(seen, ['unavailable']);
});

test('voice states reach the real face with existing events: listening looks curious, speaking looks happy', () => {
  const face = createFaceCore({ seed: 3 });
  const stop = connectRobot(face, { url: 'http://127.0.0.1:8765/events', EventSourceImpl: FakeSource });
  const src = FakeSource.last;
  face.step(1000);
  src.emit({ type: 'curious' });              // core/face_mood.VoiceMood: listening, thinking
  face.step(500);
  assert.equal(face.sample().mode, 'curious');
  src.emit({ type: 'stroke' });               // VoiceMood: speaking
  face.step(500);
  assert.equal(face.sample().mode, 'happy');
  face.step(2_600);                           // the bridge re-sends before the reaction runs out
  src.emit({ type: 'stroke' });
  face.step(500);
  assert.equal(face.sample().mode, 'happy');
  stop();
});
