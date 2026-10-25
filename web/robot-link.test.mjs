// node --test web/robot-link.test.mjs   (uses the real face core from Telefon Yüzü R01)
import test from 'node:test';
import assert from 'node:assert/strict';
import { fileURLToPath } from 'node:url';
import path from 'node:path';
import { connectRobot, parseRobotEvent, sendTalk, watchTalkGestures } from './robot-link.js';

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

class FakeTarget {
  constructor() { this.listeners = {}; }
  addEventListener(type, fn) { (this.listeners[type] ??= new Set()).add(fn); }
  removeEventListener(type, fn) { this.listeners[type]?.delete(fn); }
  emit(type, ev = {}) { for (const fn of this.listeners[type] ?? []) fn({ isPrimary: true, button: 0, pointerId: 1, clientX: 100, clientY: 100, ...ev }); }
  count() { return Object.values(this.listeners).reduce((n, s) => n + s.size, 0); }
}

function fakeClock() {
  let t = 0, timers = [];
  return {
    now: () => t,
    setTimer: (fn, ms) => { const id = { fn, at: t + ms }; timers.push(id); return id; },
    clearTimer: id => { timers = timers.filter(x => x !== id); },
    advance(ms) {
      t += ms;
      for (const id of timers.filter(x => x.at <= t)) { timers = timers.filter(x => x !== id); id.fn(); }
    },
  };
}

test('a tap or a hold on the face asks Luna to listen; a stroke (petting) does not', () => {
  const stage = new FakeTarget(), clock = fakeClock(), talks = [];
  const stop = watchTalkGestures(stage, kind => talks.push(kind), clock);
  stage.emit('pointerdown'); clock.advance(120); stage.emit('pointerup', { clientX: 104 });   // tap
  stage.emit('pointerdown'); clock.advance(500); stage.emit('pointerup');                    // hold: fires at 430 ms, once
  stage.emit('pointerdown'); clock.advance(80);                                              // stroke
  stage.emit('pointermove', { clientX: 160 }); clock.advance(600); stage.emit('pointerup', { clientX: 220 });
  stage.emit('pointerdown'); clock.advance(390); stage.emit('pointerup');                    // too long for a tap, too short for a hold
  stage.emit('pointerdown'); clock.advance(100); stage.emit('pointercancel'); clock.advance(600);
  stage.emit('pointerdown', { isPrimary: false }); clock.advance(600);                       // second finger
  stage.emit('pointerdown', { button: 2 }); clock.advance(600);                              // right mouse button
  stage.emit('keydown', { key: 'Enter' }); stage.emit('keydown', { key: ' ', repeat: true });
  assert.deepEqual(talks, ['tap', 'hold', 'key']);
  stop();
  assert.equal(stage.count(), 0);
});

test('talk posts to the bridge without throwing, whatever the network does', async () => {
  const calls = [], statuses = [];
  const ok = async (url, init) => { calls.push([url, init.method]); return { ok: true, status: 204 }; };
  assert.equal(await sendTalk('http://127.0.0.1:8765/talk', { fetchImpl: ok, onStatus: s => statuses.push(s) }), true);
  assert.deepEqual(calls, [['http://127.0.0.1:8765/talk', 'POST']]);
  assert.equal(await sendTalk('x', { fetchImpl: async () => ({ ok: false, status: 403 }), onStatus: s => statuses.push(s) }), false);
  assert.equal(await sendTalk('x', { fetchImpl: async () => { throw new TypeError('offline'); }, onStatus: s => statuses.push(s) }), false);
  assert.equal(await sendTalk('x', { fetchImpl: () => { throw new TypeError('sync'); }, onStatus: s => statuses.push(s) }), false);
  assert.equal(await sendTalk('x', { fetchImpl: null, onStatus: s => statuses.push(s) }), false);
  assert.deepEqual(statuses, ['talk-sent', 'talk-refused', 'talk-failed', 'talk-failed', 'talk-unavailable']);
});

test('connectRobot sends a tap on the real face to /talk next to /events and still drives the face', async () => {
  const face = createFaceCore({ seed: 11 });
  const stage = new FakeTarget(), posts = [];
  const stop = connectRobot(face, { url: 'http://127.0.0.1:8765/events', EventSourceImpl: FakeSource, talkTarget: stage,
    fetchImpl: async url => { posts.push(url); return { ok: true }; } });
  const src = FakeSource.last;
  stage.emit('pointerdown'); stage.emit('pointerup');
  await new Promise(r => setTimeout(r, 0));
  assert.deepEqual(posts, ['http://127.0.0.1:8765/talk']);
  face.step(1000);
  src.emit({ type: 'curious' });                 // the bridge: Luna listens
  face.step(500);
  assert.equal(face.sample().mode, 'curious');
  stop();
  assert.equal(stage.count(), 0);
  assert.equal(src.closed, true);
});

test('without a talk target nothing listens for touches (old behaviour)', () => {
  const stage = new FakeTarget();
  const stop = connectRobot(createFaceCore({ seed: 1 }), { url: 'http://127.0.0.1:8765/events', EventSourceImpl: FakeSource });
  stage.emit('pointerdown'); stage.emit('pointerup');
  assert.equal(stage.count(), 0);
  stop();
});
