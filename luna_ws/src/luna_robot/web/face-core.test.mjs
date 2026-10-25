// cd web && npm test   (Luna's face core, web/face-core.js, without a browser)
import test from 'node:test';
import assert from 'node:assert/strict';
import { FACE_EVENTS, TIMING, createFaceCore } from './face-core.js';

const run = (face, events, ms = 500) => { for (const [type, data] of events) { face.dispatch(type, data); face.step(ms); } };

test('the same seed and the same events draw the same face', () => {
  const a = createFaceCore({ seed: 42 }), b = createFaceCore({ seed: 42 }), c = createFaceCore({ seed: 43 });
  const script = [['wake'], ['gaze', { x: 0.3, y: 0.1 }], ['listening'], ['thinking'], ['speaking'], ['idle'], ['tap']];
  for (const f of [a, b, c]) { f.step(3000); run(f, script); f.step(7000); }
  assert.deepEqual(a.sample(), b.sample());
  assert.notDeepEqual(a.sample().gaze, c.sample().gaze);       // idle eyes wander their own way
});

test('moments come and go: a bump surprises, a stroke pleases, curious is a glance', () => {
  const face = createFaceCore({ seed: 1 });
  for (const [event, mode, seconds] of [['tap', 'surprised', TIMING.surprised], ['stroke', 'happy', TIMING.happy],
    ['curious', 'curious', TIMING.curious]]) {
    face.dispatch(event);
    face.step(seconds * 1000 - 100);
    assert.equal(face.sample().mode, mode, event);
    face.step(200);
    assert.equal(face.sample().mode, 'idle', `${event} is over`);
  }
});

test('the voice decides the face; a bump still surprises, low battery shows only when Luna is not talking', () => {
  const face = createFaceCore({ seed: 2 });
  face.dispatch('low-battery');
  face.step(100);
  assert.equal(face.sample().mode, 'tired');
  face.dispatch('speaking');
  face.step(100);
  assert.equal(face.sample().mode, 'speaking');
  assert.ok(face.sample().expr.tired > 0);                    // the battery glyph stays
  face.dispatch('stroke');
  face.step(100);
  assert.equal(face.sample().mode, 'speaking');                // happiness shows as a smile on top
  face.dispatch('tap');
  face.step(300);
  assert.equal(face.sample().mode, 'surprised');
  assert.ok(face.sample().expr.L.smile < 0.2 && face.sample().expr.L.h > 1.1);   // wide eyes, the smile gone
  face.step(TIMING.surprised * 1000);
  assert.equal(face.sample().mode, 'speaking');
  face.dispatch('idle');
  face.step(TIMING.happy * 1000);
  assert.equal(face.sample().mode, 'tired');
});

test('thinking looks up and away, speaking opens the mouth, listening raises the bars unless muted', () => {
  const face = createFaceCore({ seed: 3 });
  face.dispatch('thinking');
  face.step(800);
  assert.ok(face.sample().gaze.x > 0.4 && face.sample().gaze.y < -0.4);
  assert.ok(face.sample().expr.think > 0.9);
  face.dispatch('speaking');
  let open = 0;
  for (let i = 0; i < 100; i++) { face.step(20); open = Math.max(open, face.sample().mouth); }
  assert.ok(open > 0.5, `mouth ${open}`);
  face.dispatch('listening');
  face.step(800);
  assert.ok(face.sample().expr.listen > 0.9 && face.sample().level > 0.3);
  face.dispatch('muted');
  face.step(800);
  assert.ok(face.sample().expr.listen < 0.01 && face.sample().level < 0.01);
});

test('asleep on the dock: eyes shut, robot gaze, voice and moments ignored; waking blinks twice', () => {
  const face = createFaceCore({ seed: 4 });
  face.step(1000);
  run(face, [['speaking'], ['sleep'], ['gaze', { x: 1, y: 1 }], ['listening'], ['tap'], ['stroke'], ['curious']]);
  const s = face.sample();
  assert.equal(s.mode, 'sleep');
  assert.ok(s.expr.L.h < 0.15 && s.expr.R.h < 0.15);
  assert.equal(s.looking, false);
  assert.equal(s.voice, 'idle');
  let blinks = 0;
  for (let i = 0; i < 1000; i++) { face.step(20); if (face.sample().blink > 0.9) blinks++; }
  assert.equal(blinks, 0);                                    // no blinking with closed eyes
  face.dispatch('wake');
  let peak = 0, closedFrames = 0;
  for (let i = 0; i < 60; i++) { face.step(20); const b = face.sample().blink; peak = Math.max(peak, b); if (b > 0.9) closedFrames++; }
  assert.equal(face.sample().mode, 'idle');
  assert.ok(peak > 0.9 && closedFrames >= 2);
  face.dispatch('wake');                                       // awake already: nothing changes
  assert.equal(face.sample().mode, 'idle');
});

test('a refused touch: asleep, one eye peeks; muted, a shake; otherwise nothing', () => {
  const face = createFaceCore({ seed: 5 });
  face.dispatch('sleep');
  face.step(1000);
  face.touch('refused');
  face.step(300);
  assert.ok(face.sample().expr.R.h > 0.3 && face.sample().expr.L.h < 0.15);
  face.step(TIMING.crack * 1000 + 500);
  assert.ok(face.sample().expr.R.h < 0.15);
  face.dispatch('wake'); face.dispatch('muted');
  face.touch('refused');
  assert.ok(face.sample().shakeLeft > 0);
  face.step(TIMING.shake * 1000 + 20);
  assert.equal(face.sample().shakeLeft, 0);
  face.dispatch('unmuted');
  face.touch('refused');
  assert.equal(face.sample().shakeLeft, 0);
  assert.equal(face.canTalk(), true);
  face.dispatch('muted');
  assert.equal(face.canTalk(), false);
});

test('petting makes Luna happy, but not in her sleep', () => {
  const face = createFaceCore({ seed: 6 });
  face.touch('pet');
  face.step(800);
  assert.equal(face.sample().mode, 'happy');
  assert.ok(face.sample().expr.cheek > 0.8);
  face.dispatch('sleep');
  face.touch('pet');
  face.step(800);
  assert.equal(face.sample().mode, 'sleep');
});

test('idle eyes blink every few seconds and wander; a robot gaze holds them', () => {
  const face = createFaceCore({ seed: 7 });
  let blinks = 0, wasClosed = false;
  const xs = new Set();
  for (let i = 0; i < 1500; i++) {                              // 30 s
    face.step(20);
    const s = face.sample();
    const closed = s.blink > 0.9;
    if (closed && !wasClosed) blinks++;
    wasClosed = closed;
    xs.add(Math.round(s.gaze.x * 10));
  }
  assert.ok(blinks >= 4 && blinks <= 20, `blinks ${blinks}`);
  assert.ok(xs.size >= 4, 'eyes wander');
  face.dispatch('gaze', { x: -0.6, y: 0 });
  face.step(1000);
  for (let i = 0; i < 100; i++) { face.step(20); assert.ok(Math.abs(face.sample().gaze.x + 0.6) < 0.06); }
});

test('reduced motion keeps the eyes still on their target (no micro-saccades)', () => {
  const face = createFaceCore({ seed: 8, reduceMotion: true });
  face.dispatch('gaze', { x: 0.5, y: 0 });
  face.step(2000);
  for (let i = 0; i < 50; i++) { face.step(20); assert.ok(Math.abs(face.sample().gaze.x - 0.5) < 1e-3); }
});

test('bad input is ignored without throwing', () => {
  const face = createFaceCore({ seed: 9 });
  assert.equal(face.dispatch('explode'), false);
  assert.equal(face.dispatch('gaze', { x: 'a', y: 0 }), true);
  assert.equal(face.sample().looking, false);
  face.dispatch('gaze', { x: 5, y: -5 });
  face.step(2000);
  assert.ok(face.sample().gaze.x <= 1.06 && face.sample().gaze.y <= 0.9);   // clamped to ±1 before mapping
  const before = face.sample().t;
  face.step(NaN); face.step(-50); face.step(undefined);
  assert.equal(face.sample().t, before);
  assert.equal(FACE_EVENTS.length, 15);
});
