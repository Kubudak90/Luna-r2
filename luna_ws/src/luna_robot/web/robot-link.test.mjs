// cd web && npm test   (robot-link.js driving the real face core, web/face-core.js)
import test from 'node:test';
import assert from 'node:assert/strict';
import { createFaceCore } from './face-core.js';
import { ROBOT_FACE_EVENTS, connectRobot, parseRobotEvent, sendTalk, watchTalkGestures } from './robot-link.js';

class FakeSource {
  constructor(url) { this.url = url; FakeSource.last = this; this.closed = false; }
  emit(obj) { this.onmessage?.({ data: typeof obj === 'string' ? obj : JSON.stringify(obj) }); }
  close() { this.closed = true; }
}

test('only face-core events pass, gaze is clamped', () => {
  assert.deepEqual(parseRobotEvent('{"type":"tap"}'), { type: 'tap', data: {} });
  for (const type of ['thinking', 'speaking', 'low-battery', 'battery-ok', 'muted', 'unmuted']) {
    assert.deepEqual(parseRobotEvent(JSON.stringify({ type })), { type, data: {} }, type);
  }
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
  assert.equal(face.sample().mode, 'sleep');
  src.emit({ type: 'wake' });
  face.step(500);
  assert.notEqual(face.sample().mode, 'sleep');
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

test('every face_mood event reaches the real face: voice, battery and mute each have their own look', () => {
  const face = createFaceCore({ seed: 3 });
  const stop = connectRobot(face, { url: 'http://127.0.0.1:8765/events', EventSourceImpl: FakeSource });
  const src = FakeSource.last;
  face.step(1000);
  for (const [event, mode] of [['listening', 'listening'], ['thinking', 'thinking'], ['speaking', 'speaking'],
    ['idle', 'idle']]) {
    src.emit({ type: event });
    face.step(500);
    assert.equal(face.sample().mode, mode, event);
  }
  src.emit({ type: 'low-battery' });
  face.step(500);
  assert.equal(face.sample().mode, 'tired');
  assert.ok(face.sample().expr.tired > 0.9);                // battery glyph
  src.emit({ type: 'muted' });
  face.step(500);
  assert.ok(face.sample().muted && face.sample().expr.mute > 0.9);
  src.emit({ type: 'battery-ok' }); src.emit({ type: 'unmuted' });
  face.step(1500);
  assert.equal(face.sample().mode, 'idle');
  assert.ok(face.sample().expr.tired < 0.01 && face.sample().expr.mute < 0.01);
  stop();
});

test('robot gaze moves the eyes: +x (Luna\'s left) to the screen\'s right, +y up; gaze-clear lets them wander', () => {
  const face = createFaceCore({ seed: 5 });
  const stop = connectRobot(face, { url: 'http://127.0.0.1:8765/events', EventSourceImpl: FakeSource });
  const src = FakeSource.last;
  src.emit({ type: 'gaze', x: 0.8, y: 0.5 });
  face.step(600);
  assert.ok(face.sample().gaze.x > 0.7 && face.sample().gaze.y < -0.35, JSON.stringify(face.sample().gaze));
  src.emit({ type: 'gaze', x: 9, y: 'up' });                 // not a number: dropped by robot-link
  face.step(300);
  assert.ok(face.sample().gaze.x > 0.7);
  src.emit({ type: 'gaze-clear' });
  face.step(100);
  assert.equal(face.sample().looking, false);
  stop();
});

test('the ROBOT_FACE_EVENTS list is the face core\'s', () => {
  assert.deepEqual([...ROBOT_FACE_EVENTS].sort(), ['battery-ok', 'curious', 'gaze', 'gaze-clear', 'idle', 'listening',
    'low-battery', 'muted', 'sleep', 'speaking', 'stroke', 'tap', 'thinking', 'unmuted', 'wake']);
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
  src.emit({ type: 'listening' });               // the bridge: Luna listens
  face.step(500);
  assert.equal(face.sample().mode, 'listening');
  stop();
  assert.equal(stage.count(), 0);
  assert.equal(src.closed, true);
});

test('asleep on the dock or with the microphones cut, a touch stays on the face and nothing is posted', async () => {
  const face = createFaceCore({ seed: 2 });
  const stage = new FakeTarget(), posts = [], talks = [];
  const stop = connectRobot(face, { url: 'http://127.0.0.1:8765/events', EventSourceImpl: FakeSource, talkTarget: stage,
    canTalk: face.canTalk, onTalk: (kind, sent) => { talks.push([kind, sent]); if (!sent) face.touch('refused'); },
    fetchImpl: async url => { posts.push(url); return { ok: true }; } });
  const src = FakeSource.last;
  src.emit({ type: 'sleep' });
  stage.emit('pointerdown'); stage.emit('pointerup');
  face.step(300);
  assert.ok(face.sample().expr.R.h > face.sample().expr.L.h + 0.1);   // one sleepy eye peeks open
  src.emit({ type: 'wake' }); src.emit({ type: 'muted' });
  stage.emit('pointerdown'); stage.emit('pointerup');
  assert.ok(face.sample().shakeLeft > 0);                              // "no": the microphones are off
  src.emit({ type: 'unmuted' });
  stage.emit('pointerdown'); stage.emit('pointerup');
  await new Promise(r => setTimeout(r, 0));
  assert.deepEqual(talks, [['tap', false], ['tap', false], ['tap', true]]);
  assert.deepEqual(posts, ['http://127.0.0.1:8765/talk']);
  stop();
});

test('without a talk target nothing listens for touches (old behaviour)', () => {
  const stage = new FakeTarget();
  const stop = connectRobot(createFaceCore({ seed: 1 }), { url: 'http://127.0.0.1:8765/events', EventSourceImpl: FakeSource });
  stage.emit('pointerdown'); stage.emit('pointerup');
  assert.equal(stage.count(), 0);
  stop();
});
