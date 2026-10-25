/**
 * Luna's face on the head's 5.5" AMOLED: draws web/face-core.js on a full-screen canvas and connects it to the face
 * bridge that serves this page (face_bridge_node.py, http://127.0.0.1:8765/). The panel is black where nothing
 * glows, so only the eyes light up.
 *
 * Touch: a tap or a hold asks Luna to listen (robot-link.js POSTs /talk); moving a finger over the face pets her.
 * ?events=<url> points the page at another bridge (development on a laptop).
 */
import { createFaceCore } from './face-core.js';
import { connectRobot } from './robot-link.js';

const FACE_W = 400, FACE_H = 240;                  // face units: a 5:3 panel, origin at its centre
const EYE = { w: 66, h: 82, r: 29, sep: 74 };
const EYE_HI = '#F5FBFF', EYE_MID = '#C4E6FF', EYE_LO = '#84C3F4', GLOW = 'rgba(120, 196, 255, .9)';
const MUTE = '#FF7D8E', BATTERY = '#FFB45C', CHEEK = '255, 128, 168';
const PET_SLOP_PX = 14, PET_EVERY_PX = 30;

const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));

function roundRect(c, x, y, w, h, r) {
  r = Math.max(0, Math.min(r, w / 2, h / 2));
  c.moveTo(x + r, y);
  c.arcTo(x + w, y, x + w, y + h, r); c.arcTo(x + w, y + h, x, y + h, r);
  c.arcTo(x, y + h, x, y, r); c.arcTo(x, y, x + w, y, r); c.closePath();
}

function drawEye(c, cx, cy, w, h, p, inner, gx, gy) {
  h = Math.max(h, 2.2);
  const r = Math.min(EYE.r * Math.min(p.w, 1.1), w / 2, h / 2);
  c.save();
  c.beginPath(); roundRect(c, cx - w / 2, cy - h / 2, w, h, r);
  // the bright core drifts with the gaze a little more than the eye: it reads as a round eyeball
  const g = c.createRadialGradient(cx + gx * w * 0.16, cy - h * 0.14 + gy * h * 0.1, 1, cx, cy, Math.max(w, h) * 0.78);
  g.addColorStop(0, EYE_HI); g.addColorStop(0.5, EYE_MID); g.addColorStop(1, EYE_LO);
  c.fillStyle = g; c.fill();
  c.globalCompositeOperation = 'destination-out';
  c.fillStyle = '#000';
  if (p.lid > 0.002 || Math.abs(p.slope) > 0.002) {      // upper lid: slope > 0 lowers the inner corner
    const top = cy - h / 2;
    const yAt = dx => top + p.lid * h + p.slope * h * 0.5 * (dx / (w / 2)) * inner;
    c.beginPath();
    c.moveTo(cx - w, yAt(-w)); c.lineTo(cx + w, yAt(w));
    c.lineTo(cx + w, top - h * 2); c.lineTo(cx - w, top - h * 2); c.closePath(); c.fill();
  }
  if (p.smile > 0.002) {                                   // cheeks push the lower lid up: happy arch
    c.beginPath(); c.ellipse(cx, cy + h * (0.62 + 0.62 * (1 - p.smile)), w * 0.74, h * 0.72, 0, 0, Math.PI * 2); c.fill();
  }
  c.restore();
}

/** Draws one face sample (createFaceCore().sample()) on ctx; `layer` is an offscreen canvas of the same size. */
export function drawFace(ctx, layer, s, k) {
  const lctx = layer.getContext('2d');
  const cw = ctx.canvas.width, ch = ctx.canvas.height, P = s.expr, t = s.t;
  ctx.setTransform(1, 0, 0, 1, 0, 0);
  ctx.globalAlpha = 1; ctx.shadowBlur = 0;
  ctx.fillStyle = '#000'; ctx.fillRect(0, 0, cw, ch);
  lctx.setTransform(1, 0, 0, 1, 0, 0);
  lctx.clearRect(0, 0, cw, ch);

  const shake = s.shakeLeft > 0 ? Math.sin(s.shakeLeft * 42) * 7 * (s.shakeLeft / 0.55) : 0;   // "no": a quick shake
  const sway = s.reduce ? 0 : P.cheek * Math.sin(t * 2.2) * 3;
  const roll = (P.roll + sway) * Math.PI / 180;
  const place = c => { c.setTransform(k, 0, 0, k, cw / 2, ch / 2); c.rotate(roll); c.translate(shake, 0); };

  // glowing layer: eyes, mouth, listening bars, thinking dots
  place(lctx);
  const b = s.blink, gx = s.gaze.x, gy = s.gaze.y;
  const ox = gx * 30, oy = gy * 20 + P.y - s.mouth * 2.2;
  for (const [side, sx, inner] of [['L', -1, 1], ['R', 1, -1]]) {
    const p = P[side];
    const persp = 1 + 0.08 * gx * sx;           // the eye on the side Luna looks towards grows a little
    const w = EYE.w * p.w * persp * (1 + 0.06 * b);
    const h = EYE.h * p.h * persp * (1 - 0.92 * b);
    drawEye(lctx, sx * EYE.sep + ox, oy, w, h, p, inner, gx, gy);
  }
  if (P.mouth > 0.02) {
    const o = s.mouth, mw = 26 + 10 * o, mh = 4 + 13 * o;
    lctx.globalAlpha = clamp(P.mouth, 0, 1);
    lctx.fillStyle = EYE_MID;
    lctx.beginPath(); roundRect(lctx, ox * 0.4 - mw / 2, 64 + P.y * 0.4 - mh / 2, mw, mh, Math.min(mh / 2, 8)); lctx.fill();
  }
  if (P.listen > 0.02) {
    lctx.globalAlpha = clamp(P.listen, 0, 1);
    lctx.fillStyle = EYE_MID;
    const shape = [0.45, 0.75, 1, 0.75, 0.45];
    for (let i = 0; i < 5; i++) {
      const hh = 5 + 20 * s.level * shape[i] * (0.75 + 0.25 * Math.sin(t * (7 + i * 1.7) + i));
      lctx.beginPath(); roundRect(lctx, (i - 2) * 13 - 2.5, 72 - hh / 2, 5, hh, 2.5); lctx.fill();
    }
  }
  if (P.think > 0.02) {
    lctx.fillStyle = EYE_MID;
    for (let i = 0; i < 3; i++) {
      lctx.globalAlpha = clamp(P.think, 0, 1) * (0.2 + 0.8 * Math.max(0, Math.sin(t * 5 - i * 0.9)));
      lctx.beginPath(); lctx.arc(132 + i * 14, -74 - i * 5, 4.2, 0, Math.PI * 2); lctx.fill();
    }
  }
  lctx.globalAlpha = 1;

  ctx.globalAlpha = clamp(P.bright, 0, 1);
  ctx.shadowColor = GLOW; ctx.shadowBlur = 14 * k;
  ctx.drawImage(layer, 0, 0);
  ctx.shadowBlur = 0; ctx.globalAlpha = 1;

  // soft, unglowing extras: cheeks, sleep z's, mute and battery glyphs
  place(ctx);
  if (P.cheek > 0.02) {
    for (const sx of [-1, 1]) {
      const cx = sx * (EYE.sep + 26) + ox * 0.6, cy = 46 + P.y;
      const g = ctx.createRadialGradient(cx, cy, 1, cx, cy, 28);
      g.addColorStop(0, `rgba(${CHEEK}, ${(0.5 * P.cheek).toFixed(3)})`); g.addColorStop(1, `rgba(${CHEEK}, 0)`);
      ctx.fillStyle = g; ctx.beginPath(); ctx.ellipse(cx, cy, 28, 16, 0, 0, Math.PI * 2); ctx.fill();
    }
  }
  if (P.sleep > 0.05) {
    ctx.fillStyle = EYE_MID;
    for (let i = 0; i < 3; i++) {
      const ph = (t * 0.32 + i / 3) % 1;
      ctx.globalAlpha = P.sleep * Math.sin(ph * Math.PI) * 0.75;
      ctx.font = `600 ${(13 + ph * 11).toFixed(1)}px system-ui, sans-serif`;
      ctx.fillText('z', 116 + ph * 30, -6 - ph * 72);
    }
    ctx.globalAlpha = 1;
  }
  const bothGlyphs = P.mute > 0.02 && P.tired > 0.02;
  if (P.mute > 0.02) {
    const x0 = bothGlyphs ? -24 : 0, y0 = 88;
    ctx.globalAlpha = P.mute * (s.shakeLeft > 0 ? 1 : 0.6);
    ctx.strokeStyle = MUTE; ctx.lineWidth = 2.4; ctx.lineCap = 'round';
    ctx.beginPath(); roundRect(ctx, x0 - 5, y0 - 14, 10, 16, 5); ctx.stroke();
    ctx.beginPath(); ctx.arc(x0, y0 - 6, 9, 0.15 * Math.PI, 0.85 * Math.PI); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(x0, y0 + 3); ctx.lineTo(x0, y0 + 7); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(x0 - 11, y0 - 16); ctx.lineTo(x0 + 11, y0 + 8); ctx.stroke();
    ctx.globalAlpha = 1;
  }
  if (P.tired > 0.02) {
    const x0 = bothGlyphs ? 24 : 0, y0 = 84;
    ctx.globalAlpha = P.tired * 0.9;
    ctx.strokeStyle = EYE_MID; ctx.lineWidth = 2;
    ctx.beginPath(); roundRect(ctx, x0 - 15, y0 - 6.5, 28, 13, 3); ctx.stroke();
    ctx.fillStyle = EYE_MID; ctx.fillRect(x0 + 13.5, y0 - 2.5, 3, 5);
    ctx.fillStyle = BATTERY; ctx.fillRect(x0 - 12.5, y0 - 4, 4 + (s.reduce ? 0 : 2 * Math.sin(t * 3)), 8);
    ctx.globalAlpha = 1;
  }
}

function eventsUrl() {
  const q = new URLSearchParams(location.search).get('events');
  if (q) return q;
  return location.protocol.startsWith('http') ? new URL('events', location.href).href : 'http://127.0.0.1:8765/events';
}

/** Moving a finger over the face (past the tap slop) pets Luna, once every PET_EVERY_PX of travel. */
function watchPetting(target, onPet) {
  let touch = null;
  target.addEventListener('pointerdown', ev => {
    if (!ev.isPrimary || touch) return;
    try { target.setPointerCapture(ev.pointerId); } catch {}
    touch = { id: ev.pointerId, x: ev.clientX, y: ev.clientY, lx: ev.clientX, ly: ev.clientY, stroke: false, dist: 0 };
  });
  target.addEventListener('pointermove', ev => {
    if (!touch || ev.pointerId !== touch.id) return;
    if (!touch.stroke && Math.hypot(ev.clientX - touch.x, ev.clientY - touch.y) > PET_SLOP_PX) touch.stroke = true;
    if (touch.stroke) {
      touch.dist += Math.hypot(ev.clientX - touch.lx, ev.clientY - touch.ly);
      if (touch.dist > PET_EVERY_PX) { onPet(); touch.dist = 0; }
    }
    touch.lx = ev.clientX; touch.ly = ev.clientY;
  });
  const end = ev => { if (touch && ev.pointerId === touch.id) touch = null; };
  target.addEventListener('pointerup', end);
  target.addEventListener('pointercancel', end);
}

export function startFace(canvas) {
  const ctx = canvas.getContext('2d');
  const layer = document.createElement('canvas');
  const reduceMQ = matchMedia('(prefers-reduced-motion: reduce)');
  const face = createFaceCore({ seed: (Date.now() & 0xffff) + 1, reduceMotion: reduceMQ.matches });
  reduceMQ.addEventListener?.('change', e => face.setReduceMotion(e.matches));
  let k = 1;
  const resize = () => {
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const w = canvas.clientWidth, h = canvas.clientHeight;
    canvas.width = layer.width = Math.max(1, Math.round(w * dpr));
    canvas.height = layer.height = Math.max(1, Math.round(h * dpr));
    k = Math.min(w / FACE_W, h / FACE_H) * dpr;
  };
  new ResizeObserver(resize).observe(canvas);
  resize();

  watchPetting(canvas, () => face.touch('pet'));
  const status = document.documentElement.dataset;
  connectRobot(face, {
    url: eventsUrl(), talkTarget: canvas, canTalk: face.canTalk,
    onTalk: (_, sent) => { if (!sent) face.touch('refused'); },
    onStatus: s => { if (!s.startsWith('talk')) status.link = s; },
  });
  canvas.addEventListener('keydown', ev => { if (ev.key === ' ' || ev.key === 'Enter') ev.preventDefault(); });

  let last = performance.now();
  const frame = now => {
    face.step(Math.min(50, Math.max(0, now - last)));
    last = now;
    drawFace(ctx, layer, face.sample(), k);
    requestAnimationFrame(frame);
  };
  requestAnimationFrame(frame);
  return face;
}
