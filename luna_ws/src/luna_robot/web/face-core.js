/**
 * Luna's face without the DOM: robot events in, an expression to draw out (web/face.js draws it on the head's
 * AMOLED). Pure and seeded, so node --test runs the same face the robot shows.
 *
 * Events (luna_robot/core/face_mood.py, sent by the face bridge):
 *   moments  tap (a bump: surprised), stroke (happy), curious (an attentive glance)
 *   states   sleep / wake, gaze {x, y} / gaze-clear (+x Luna's left = the viewer's right, +y up),
 *            listening / thinking / speaking / idle, low-battery / battery-ok, muted / unmuted
 * Touches on the screen itself: touch('pet') makes Luna happy; touch('refused') is a talk touch the page did not
 * send (asleep: one eye peeks open; muted: a little "no" shake).
 *
 *   const face = createFaceCore({ seed: 7 });
 *   face.dispatch('gaze', { x: 0.4, y: 0 }); face.step(16); const s = face.sample();
 */
export const FACE_EVENTS = Object.freeze(['tap', 'stroke', 'curious', 'sleep', 'wake', 'gaze', 'gaze-clear',
  'listening', 'thinking', 'speaking', 'idle', 'low-battery', 'battery-ok', 'muted', 'unmuted']);
export const VOICE = Object.freeze(['listening', 'thinking', 'speaking', 'idle']);
// Seconds. The happy glow outlasts the bridge's re-send interval, so a long stroke stays one smile.
export const TIMING = Object.freeze({ surprised: 1.3, happy: 2.8, curious: 1.2, crack: 1.2, shake: 0.55 });

const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
const ease = x => x * x * (3 - 2 * x);
const approach = (cur, target, rate, dt) => cur + (target - cur) * (1 - Math.exp(-rate * dt));

function mulberry32(seed) {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6D2B79F5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

const eye = () => ({ w: 1, h: 1, lid: 0, slope: 0, smile: 0 });
const makeExpr = () => ({ L: eye(), R: eye(), roll: 0, y: 0, bright: 1, mouth: 0, cheek: 0, listen: 0, think: 0,
  tired: 0, sleep: 0, mute: 0 });
const GLOBAL_KEYS = ['roll', 'y', 'bright', 'mouth', 'cheek', 'listen', 'think', 'tired', 'sleep', 'mute'];
const copyExpr = e => ({ ...e, L: { ...e.L }, R: { ...e.R } });

export function createFaceCore({ seed = 1, reduceMotion = false } = {}) {
  const rand = mulberry32(seed);
  const S = {
    t: 0, asleep: false, voice: 'idle', lowBattery: false, muted: false, gaze: null, reduce: !!reduceMotion,
    surprisedUntil: -1, happyUntil: -1, curiousUntil: -1, crackUntil: -1, shakeUntil: -1,
    gx: 0, gy: 0, wx: 0, wy: 0, nextWander: 0.8, mx: 0, my: 0, nextMicro: 0.4,
    blinkAt: -10, blinkTwice: false, nextBlink: 1.6, pendingBlinks: [], level: 0, mouthV: 0,
  };
  const P = makeExpr();

  function mode() {
    if (S.asleep) return 'sleep';
    if (S.t < S.surprisedUntil) return 'surprised';
    if (S.voice !== 'idle') return S.voice;
    if (S.t < S.happyUntil) return 'happy';
    if (S.t < S.curiousUntil) return 'curious';
    return S.lowBattery ? 'tired' : 'idle';
  }

  function targetExpr(m, t) {
    const e = makeExpr();
    const both = f => { f(e.L); f(e.R); };
    switch (m) {
      case 'listening':
        Object.assign(e.L, { w: 1.06, h: 1.12 });
        Object.assign(e.R, { w: 0.97, h: 0.9, lid: 0.16 });
        e.roll = 6; e.y = -5; e.listen = S.muted ? 0 : 1; break;
      case 'curious':
        Object.assign(e.L, { w: 1.04, h: 1.08 });
        Object.assign(e.R, { h: 0.94, lid: 0.1 });
        e.roll = 8; e.y = -3; break;
      case 'thinking':
        both(p => Object.assign(p, { h: 0.9, lid: 0.2, slope: 0.14 })); e.roll = -3; e.think = 1; break;
      case 'speaking':
        both(p => { p.smile = 0.34; }); e.mouth = 1; break;
      case 'happy':
        both(p => { p.smile = 1; }); e.cheek = 1; break;
      case 'surprised':
        both(p => Object.assign(p, { w: 1.14, h: 1.26 })); e.y = -7; break;
      case 'tired':
        both(p => Object.assign(p, { lid: 0.42, slope: -0.14 })); e.bright = 0.8; e.y = 5; break;
      case 'sleep':
        both(p => { p.h = 0.07; }); e.bright = 0.42 + 0.1 * Math.sin(t * 1.3); e.y = 12; e.sleep = 1;
        if (t < S.crackUntil) Object.assign(e.R, { h: 0.42, lid: 0.1 });   // one sleepy eye peeks, then closes
        break;
    }
    const smiles = m !== 'sleep' && m !== 'listening' && m !== 'happy' && m !== 'surprised';   // a bump wipes the smile
    if (t < S.happyUntil && smiles) { both(p => { p.smile = 1; }); e.cheek = 1; }
    if (S.lowBattery && m !== 'sleep') e.tired = 1;        // the battery glyph, whatever the face does
    if (S.muted && m !== 'sleep') { e.mute = 1; e.bright *= 0.9; }
    return e;
  }

  function smoothExpr(m, e, dt) {
    const rate = S.reduce ? 14 : (m === 'surprised' ? 24 : 9);
    for (const side of ['L', 'R']) for (const key in P[side]) P[side][key] = approach(P[side][key], e[side][key], rate, dt);
    for (const key of GLOBAL_KEYS) P[key] = approach(P[key], e[key], key === 'bright' ? 5 : rate, dt);
  }

  // Blink: 70 ms close, 40 ms hold, 130 ms open; sometimes twice.
  function blinkAmount(t) {
    let u = t - S.blinkAt;
    if (S.blinkTwice && u > 0.3) u -= 0.3;
    if (u < 0 || u > 0.24) return 0;
    if (u < 0.07) return ease(u / 0.07);
    if (u < 0.11) return 1;
    return 1 - ease((u - 0.11) / 0.13);
  }
  function blink(t, twice = rand() < 0.18) {
    if (S.asleep || t - S.blinkAt < 0.62) return;
    S.blinkAt = t; S.blinkTwice = twice;
  }

  function updateGaze(m, t, dt) {
    let tx, ty;
    if (m === 'sleep') { tx = 0; ty = 0.25; }
    else if (m === 'thinking') { tx = 0.55 + 0.05 * Math.sin(t * 0.9); ty = -0.55; }     // looks up and away
    else if (S.gaze) { tx = S.gaze.x; ty = -S.gaze.y * 0.85; }                       // screen y grows downwards
    else {
      if (t >= S.nextWander) {
        const lazy = m === 'tired' ? 0.55 : 1;
        const centre = rand() < 0.45 || m === 'listening' || m === 'speaking';
        const nx = centre ? (rand() - 0.5) * 0.2 : (rand() * 2 - 1) * 0.7 * lazy;
        const ny = centre ? (rand() - 0.5) * 0.14 : (rand() * 2 - 1) * 0.35 * lazy + (m === 'tired' ? 0.25 : 0);
        if (Math.hypot(nx - S.wx, ny - S.wy) > 0.5 && rand() < 0.35) blink(t, false);   // big glances blink
        S.wx = nx; S.wy = ny;
        S.nextWander = t + (1.2 + rand() * 2.6) * (m === 'tired' ? 1.6 : 1);
      }
      tx = S.wx; ty = S.wy;
    }
    if (S.reduce || m === 'sleep') { S.mx = 0; S.my = 0; }
    else if (t >= S.nextMicro) { S.mx = (rand() - 0.5) * 0.05; S.my = (rand() - 0.5) * 0.04; S.nextMicro = t + 0.35 + rand() * 0.6; }
    const rate = m === 'tired' ? 9 : 20;
    S.gx = approach(S.gx, tx + S.mx, rate, dt);
    S.gy = approach(S.gy, ty + S.my, rate, dt);
  }

  // The listening bars breathe (the bridge sends no audio level); the mouth moves in syllables while speaking.
  const listenLevel = (m, t) => (m === 'listening' && !S.muted ? 0.45 + 0.3 * Math.abs(Math.sin(t * 2.4)) : 0);
  function mouthOpen(t) {
    const gate = Math.sin(t * 1.7) > -0.55 ? 1 : 0.1;
    const syl = Math.max(0, Math.sin(t * 10.5 + Math.sin(t * 2.1) * 1.8));
    return gate * (0.25 + 0.75 * syl);
  }

  function tick(dt) {
    S.t += dt;
    const t = S.t, m = mode();
    if (t >= S.nextBlink) {
      blink(t);
      S.nextBlink = t + (2 + rand() * 4) * (m === 'tired' ? 1.7 : 1);
    }
    while (S.pendingBlinks.length && S.pendingBlinks[0] <= t) { S.pendingBlinks.shift(); S.blinkAt = -10; blink(t, false); }
    updateGaze(m, t, dt);
    smoothExpr(m, targetExpr(m, t), dt);
    S.level = approach(S.level, listenLevel(m, t), 18, dt);
    S.mouthV = approach(S.mouthV, mouthOpen(t) * P.mouth, 25, dt);
  }

  function dispatch(type, data = {}) {
    const t = S.t;
    switch (type) {
      case 'tap': if (!S.asleep) S.surprisedUntil = t + TIMING.surprised; break;
      case 'stroke': if (!S.asleep) S.happyUntil = t + TIMING.happy; break;
      case 'curious': if (!S.asleep) S.curiousUntil = t + TIMING.curious; break;
      case 'sleep':
        S.asleep = true; S.voice = 'idle'; S.gaze = null;
        S.surprisedUntil = S.happyUntil = S.curiousUntil = -1; break;
      case 'wake':
        if (S.asleep) { S.asleep = false; S.pendingBlinks = [t + 0.4, t + 0.8]; }   // awake already: nothing to do
        break;
      case 'gaze': {
        const x = Number(data.x), y = Number(data.y);
        if (!S.asleep && Number.isFinite(x) && Number.isFinite(y)) S.gaze = { x: clamp(x, -1, 1), y: clamp(y, -1, 1) };
        break;
      }
      case 'gaze-clear': S.gaze = null; break;
      case 'listening': case 'thinking': case 'speaking': case 'idle':
        if (!S.asleep) S.voice = type;          // on the dock the dock owns the face (face_mood.VoiceMood)
        break;
      case 'low-battery': S.lowBattery = true; break;
      case 'battery-ok': S.lowBattery = false; break;
      case 'muted': S.muted = true; break;
      case 'unmuted': S.muted = false; break;
      default: return false;
    }
    return true;
  }

  function touch(kind) {
    const t = S.t;
    if (kind === 'pet' && !S.asleep) S.happyUntil = t + TIMING.happy;
    else if (kind === 'refused') {
      if (S.asleep) S.crackUntil = t + TIMING.crack;
      else if (S.muted) S.shakeUntil = t + TIMING.shake;
    }
  }

  return {
    dispatch,
    touch,
    /** Advances the face by `ms` milliseconds, in steps of at most 20 ms. */
    step(ms) {
      let left = Math.max(0, Number(ms) || 0) / 1000;
      while (left > 1e-9) { const dt = Math.min(0.02, left); tick(dt); left -= dt; }
    },
    sample() {
      return {
        t: S.t, mode: mode(), asleep: S.asleep, voice: S.voice, muted: S.muted, lowBattery: S.lowBattery,
        looking: !!S.gaze, gaze: { x: S.gx, y: S.gy }, expr: copyExpr(P), blink: blinkAmount(S.t),
        level: S.level, mouth: S.mouthV, shakeLeft: Math.max(0, S.shakeUntil - S.t), reduce: S.reduce,
      };
    },
    /** Talk touches are sent only while Luna is awake and her microphones are on. */
    canTalk: () => !S.asleep && !S.muted,
    setReduceMotion(v) { S.reduce = !!v; },
  };
}
