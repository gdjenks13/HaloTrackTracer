"use strict";
// ============================================================================================
// Planner: lays Forge pieces along the traced centerlines. The result is exported as an HMS prefab.
// Units: world units (wu), Y up. Arc lengths are horizontal, since grade = rise / run.
// ============================================================================================
const DENSE_STEP = 0.25, STEP_GRAIN = 0.05, DEFAULT_TOL = 0.05;
const FIT_LADDER = [0.1, 0.2, 0.35, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0];
const V = {
  add: (a, b) => [a[0] + b[0], a[1] + b[1]], sub: (a, b) => [a[0] - b[0], a[1] - b[1]],
  mul: (a, s) => [a[0] * s, a[1] * s], dot: (a, b) => a[0] * b[0] + a[1] * b[1],
  len: a => Math.hypot(a[0], a[1]), lerp: (a, b, t) => [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t],
  norm: a => { const l = Math.hypot(a[0], a[1]); return l > 0 ? [a[0] / l, a[1] / l] : [0, 0]; },
};
function clean(pts, closed) {
  const out = [];
  for (const p of pts) if (!out.length || V.len(V.sub(p, out[out.length - 1])) > 1e-4) out.push(p);
  if (closed && out.length > 2 && V.len(V.sub(out[0], out[out.length - 1])) < 1e-4) out.pop();
  return out;
}
function catmullRom(pts, closed, spacing) {
  const p = clean(pts, closed), n = p.length;
  if (n < 3) return p;
  spacing = Math.max(spacing, 0.05);
  const get = i => closed ? p[((i % n) + n) % n]
    : i < 0 ? V.sub(V.mul(p[0], 2), p[1]) : i >= n ? V.sub(V.mul(p[n - 1], 2), p[n - 2]) : p[i];
  const segs = closed ? n : n - 1, out = [];
  const tj = (a, b) => Math.sqrt(Math.max(V.len(V.sub(b, a)), 1e-4));
  for (let s = 0; s < segs; s++) {
    const p0 = get(s - 1), p1 = get(s), p2 = get(s + 1), p3 = get(s + 2);
    const t0 = 0, t1 = t0 + tj(p0, p1), t2 = t1 + tj(p1, p2), t3 = t2 + tj(p2, p3);
    const k = Math.max(1, Math.ceil(V.len(V.sub(p2, p1)) / spacing));
    for (let j = 0; j < k; j++) {
      const t = t1 + (t2 - t1) * (j / k);
      const a1 = V.add(V.mul(p0, (t1 - t) / (t1 - t0)), V.mul(p1, (t - t0) / (t1 - t0)));
      const a2 = V.add(V.mul(p1, (t2 - t) / (t2 - t1)), V.mul(p2, (t - t1) / (t2 - t1)));
      const a3 = V.add(V.mul(p2, (t3 - t) / (t3 - t2)), V.mul(p3, (t - t2) / (t3 - t2)));
      const b1 = V.add(V.mul(a1, (t2 - t) / (t2 - t0)), V.mul(a2, (t - t0) / (t2 - t0)));
      const b2 = V.add(V.mul(a2, (t3 - t) / (t3 - t1)), V.mul(a3, (t - t1) / (t3 - t1)));
      out.push(V.add(V.mul(b1, (t2 - t) / (t2 - t1)), V.mul(b2, (t - t1) / (t2 - t1))));
    }
  }
  if (!closed) out.push(p[n - 1]);
  return out;
}
// One Catmull-Rom span from p1 to p2 (p2 itself left out); same maths as catmullRom above.
function crSeg(p0, p1, p2, p3, spacing) {
  const tj = (a, b) => Math.sqrt(Math.max(V.len(V.sub(b, a)), 1e-4)), out = [];
  const t0 = 0, t1 = t0 + tj(p0, p1), t2 = t1 + tj(p1, p2), t3 = t2 + tj(p2, p3);
  const k = Math.max(1, Math.ceil(V.len(V.sub(p2, p1)) / spacing));
  for (let j = 0; j < k; j++) {
    const t = t1 + (t2 - t1) * (j / k);
    const a1 = V.add(V.mul(p0, (t1 - t) / (t1 - t0)), V.mul(p1, (t - t0) / (t1 - t0)));
    const a2 = V.add(V.mul(p1, (t2 - t) / (t2 - t1)), V.mul(p2, (t - t1) / (t2 - t1)));
    const a3 = V.add(V.mul(p2, (t3 - t) / (t3 - t2)), V.mul(p3, (t - t2) / (t3 - t2)));
    const b1 = V.add(V.mul(a1, (t2 - t) / (t2 - t0)), V.mul(a2, (t - t0) / (t2 - t0)));
    const b2 = V.add(V.mul(a2, (t3 - t) / (t3 - t1)), V.mul(a3, (t - t1) / (t3 - t1)));
    out.push(V.add(V.mul(b1, (t2 - t) / (t2 - t1)), V.mul(b2, (t - t1) / (t2 - t1))));
  }
  return out;
}
// Old point-traced paths (pts + segs) are only read to turn them into a dashed guide line.
// Segment i was smooth ("s"), straight ("l") or a curve ("c") bent through its handle h.
const segKind = (p, i) => { const s = p.segs && p.segs[i]; return p.smooth === false ? "l" : s ? s.k : "s"; };
// The centerline through an old path's points. conv converts stored handles into the units of pts.
function pathLine(p, pts, closed, spacing, conv = x => x) {
  const n = pts.length;
  if (n < 2) return pts;
  closed = closed && n >= 3;
  const m = closed ? n : n - 1, kinds = Array.from({ length: m }, (_, i) => segKind(p, i));
  if (kinds.every(k => k === "s")) return n >= 3 ? catmullRom(pts, closed, spacing) : pts;
  if (kinds.every(k => k === "l")) return pts;
  spacing = Math.max(spacing, 0.05);
  const at = i => pts[((i % n) + n) % n];
  const smooth = i => closed ? kinds[((i % m) + m) % m] === "s" : i >= 0 && i < m && kinds[i] === "s";
  const out = [];
  for (let i = 0; i < m; i++) {
    const a = pts[i], b = at(i + 1);
    if (kinds[i] === "l") out.push(a);
    else if (kinds[i] === "c") {   // quadratic curve that passes through the handle at its middle
      const h = conv(p.segs[i].h), c = V.sub(V.mul(h, 2), V.lerp(a, b, 0.5));
      const k = Math.max(1, Math.ceil((V.len(V.sub(h, a)) + V.len(V.sub(b, h))) / spacing));
      for (let j = 0; j < k; j++) { const t = j / k, u = 1 - t; out.push(V.add(V.add(V.mul(a, u * u), V.mul(c, 2 * u * t)), V.mul(b, t * t))); }
    } else out.push(...crSeg(smooth(i - 1) ? at(i - 1) : V.sub(V.mul(a, 2), b), a, b, smooth(i + 1) ? at(i + 2) : V.sub(V.mul(b, 2), a), spacing));
  }
  if (!closed) out.push(pts[n - 1]);
  return out;
}
class Arc {
  static make(src, closed) {
    const p = clean(src, closed);
    if (p.length < 2) return null;
    const pts = [], m = closed ? p.length : p.length - 1;
    for (let i = 0; i < m; i++) {
      const a = p[i], b = p[(i + 1) % p.length];
      const k = Math.max(1, Math.ceil(V.len(V.sub(b, a)) / DENSE_STEP));
      for (let j = 0; j < k; j++) pts.push(V.lerp(a, b, j / k));
    }
    if (!closed) pts.push(p[p.length - 1]);
    const cum = [0], ring = closed ? pts.length : pts.length - 1;
    for (let i = 0; i < ring; i++) cum.push(cum[i] + V.len(V.sub(pts[(i + 1) % pts.length], pts[i])));
    const total = cum[cum.length - 1];
    if (!(total > 1e-3)) return null;
    const a = new Arc(); Object.assign(a, { pts, cum, total, closed }); return a;
  }
  at(s) {
    s = this.closed ? ((s % this.total) + this.total) % this.total : Math.min(Math.max(s, 0), this.total);
    let lo = 0, hi = this.cum.length - 1;
    while (hi - lo > 1) { const mid = (lo + hi) >> 1; if (this.cum[mid] <= s) lo = mid; else hi = mid; }
    const a = this.pts[lo], b = this.pts[(lo + 1) % this.pts.length], seg = this.cum[lo + 1] - this.cum[lo];
    if (seg < 1e-6) return a;
    return V.lerp(a, b, Math.min(Math.max((s - this.cum[lo]) / seg, 0), 1));
  }
  tangent(s, h) {
    const a = this.closed ? s - h : Math.max(s - h, 0), b = this.closed ? s + h : Math.min(s + h, this.total);
    const d = V.sub(this.at(b), this.at(a));
    return V.dot(d, d) < 1e-10 ? [1, 0] : V.norm(d);
  }
}
const perp = t => [-t[1], t[0]];

// Sections: a path is a start point, a heading and a chain of sections, each starting where the
// last one ends and carrying on in the same direction. A straight has a length; a turn has the
// angle the heading changes by (+ = left, counter-clockwise seen from above) and a radius; a curve
// is a free cubic Bezier (c1, c2, e: its two handles and end, in the frame of its start: x ahead,
// y to the left). s.kink turns a section's start away from the heading it arrives with, which is
// how a curve's neighbours stay where they are. Points and lengths are image px; headings are
// degrees, 0 = image right.
const RAD = Math.PI / 180;
const dirPx = h => [Math.cos(h * RAD), -Math.sin(h * RAD)];
const leftPx = h => [-Math.sin(h * RAD), -Math.cos(h * RAD)];
const wrap180 = a => ((a % 360) + 540) % 360 - 180;
const toWorld = (pose, l) => V.add(V.add(pose.pt, V.mul(dirPx(pose.head), l[0])), V.mul(leftPx(pose.head), l[1]));
const toLocal = (pose, w) => { const d = V.sub(w, pose.pt); return [V.dot(d, dirPx(pose.head)), V.dot(d, leftPx(pose.head))]; };
const bez = (a, b, c, d, t) => { const u = 1 - t; return V.add(V.add(V.mul(a, u * u * u), V.mul(b, 3 * u * u * t)), V.add(V.mul(c, 3 * u * t * t), V.mul(d, t * t * t))); };
const curveStartHead = (pose, s) => { const v = [s.c1, s.c2, s.e].find(q => V.len(q) > 1e-6) || [1, 0]; return pose.head + Math.atan2(v[1], v[0]) / RAD; };
const curveEndHead = (pose, s) => {   // direction the curve leaves its end
  const o = [0, 0], v = [s.e, s.c2, s.c1, o].map(q => V.sub(s.e, q)).find(q => V.len(q) > 1e-6) || [1, 0];
  return pose.head + Math.atan2(v[1], v[0]) / RAD;
};
function secEnd(pose, s) {
  if (s.k === "s") return { pt: V.add(pose.pt, V.mul(dirPx(pose.head), s.len)), head: pose.head };
  if (s.k === "c") return { pt: toWorld(pose, s.e), head: curveEndHead(pose, s) };
  const sg = Math.sign(s.ang) || 1, c = V.add(pose.pt, V.mul(leftPx(pose.head), sg * s.r)), h = pose.head + s.ang;
  return { pt: V.sub(c, V.mul(leftPx(h), sg * s.r)), head: h };
}
function secPoints(pose, s, step) {   // its start (exactly pose.pt) and points along it; the end is left out
  step = Math.max(step, 1e-3);
  if (s.k === "s") { const n = Math.max(1, Math.ceil(s.len / step)), d = dirPx(pose.head); return Array.from({ length: n }, (_, j) => j ? V.add(pose.pt, V.mul(d, s.len * j / n)) : pose.pt); }
  if (s.k === "c") {
    const o = [0, 0], reach = V.len(s.c1) + V.len(V.sub(s.c2, s.c1)) + V.len(V.sub(s.e, s.c2)), n = Math.min(4000, Math.max(2, Math.ceil(reach / step)));
    return Array.from({ length: n }, (_, j) => j ? toWorld(pose, bez(o, s.c1, s.c2, s.e, j / n)) : pose.pt);
  }
  const sg = Math.sign(s.ang) || 1, c = V.add(pose.pt, V.mul(leftPx(pose.head), sg * s.r));
  const n = Math.min(4000, Math.max(2, Math.ceil(Math.max(Math.abs(s.ang) / 2, Math.abs(s.ang) * RAD * s.r / step))));
  return Array.from({ length: n }, (_, j) => j ? V.sub(c, V.mul(leftPx(pose.head + s.ang * j / n), sg * s.r)) : pose.pt);
}
// poses[i] is where section i starts and which way (its kink included); poses[n] is where the last ends.
function posesOf(p) {
  const poses = [{ pt: p.start, head: p.head == null ? 0 : p.head }];
  p.secs.forEach((s, i) => {
    if (i > 0 && s.kink) poses[i] = { pt: poses[i].pt, head: poses[i].head + s.kink };
    poses.push(secEnd(poses[i], s));
  });
  return poses;
}
// Any section as an absolute cubic Bezier (a, c1, c2, b), and back to a curve section.
function cubicAbs(pose, s) {
  const a = pose.pt;
  if (s.k === "c") return { a, c1: toWorld(pose, s.c1), c2: toWorld(pose, s.c2), b: toWorld(pose, s.e) };
  const end = secEnd(pose, s), b = end.pt;
  if (s.k === "s") return { a, c1: V.add(a, V.mul(dirPx(pose.head), s.len / 3)), c2: V.sub(b, V.mul(dirPx(pose.head), s.len / 3)), b };
  const k = 4 / 3 * Math.tan(Math.abs(s.ang) * RAD / 4) * s.r;   // the usual cubic for a circular arc
  return { a, c1: V.add(a, V.mul(dirPx(pose.head), k)), c2: V.sub(b, V.mul(dirPx(end.head), k)), b };
}
const curveFrom = (pose, A) => ({ k: "c", c1: toLocal(pose, A.c1), c2: toLocal(pose, A.c2), e: toLocal(pose, A.b) });
// Reshapes section i of p (absolute cubic A becomes B) while every other section stays where it
// is: the next section gets a kink so it keeps its own direction.
function reshape(p, i, B) {
  const ps = posesOf(p), old = p.secs[i], before = secEnd(ps[i], old).head, s = curveFrom(ps[i], B);
  if (old.kink) s.kink = old.kink;
  p.secs[i] = s;
  const next = p.secs[i + 1];
  if (next) { const k = wrap180((next.kink || 0) + before - secEnd(ps[i], s).head); if (Math.abs(k) > 1e-9) next.kink = k; else delete next.kink; }
}
// Shift + diamond: the ends stay put and the middle follows the cursor; both handles move together.
function bendFree(p, i, q) {
  const A = cubicAbs(posesOf(p)[i], p.secs[i]), d = V.mul(V.sub(q, bez(A.a, A.c1, A.c2, A.b, 0.5)), 4 / 3);
  reshape(p, i, { a: A.a, c1: V.add(A.c1, d), c2: V.add(A.c2, d), b: A.b });
}
// Shift + end dot: the start and the middle stay put and the end follows the cursor.
function moveEndFree(p, i, q) {
  const A = cubicAbs(posesOf(p)[i], p.secs[i]);
  reshape(p, i, { a: A.a, c1: A.c1, c2: V.sub(A.c2, V.mul(V.sub(q, A.b), 1 / 3)), b: q });
}
function pathGeom(p, step) {
  if (!p.start) return null;
  const poses = posesOf(p), line = [];
  p.secs.forEach((s, i) => line.push(...secPoints(poses[i], s, step)));
  line.push(poses[poses.length - 1].pt);
  return { poses, line };
}

// Elevation profile: grade (in %) is piecewise-linear along the path and height is its integral,
// so a constant grade is a straight ramp and a changing grade is a parabolic vertical curve.
const FLAT_PROFILE = { z0: 0, closure: 0, active: false, pieces: [], z: () => 0, gradeAt: () => 0, sample: () => ({ z: 0, grade: 0 }), cos: () => 1 };
// One section as ranges in [0, total]; a section running through a loop's seam becomes two.
function gradeRanges(sa, sb, g0, g1, total) {
  if (sb > sa) return [{ a: sa, b: sb, g0, g1 }];
  const seam = g0 + (g1 - g0) * (total - sa) / (total - sa + sb);
  return [{ a: sa, b: total, g0, g1: seam }, { a: 0, b: sb, g0: seam, g1 }].filter(r => r.b - r.a > 1e-6);
}
function makeProfile(ranges, total, closed, z0 = 0) {
  const pieces = ranges.filter(r => r.b - r.a > 1e-6).sort((p, q) => p.a - q.a);
  const raw = s => {
    let z = z0;
    for (const p of pieces) {
      if (s <= p.a) break;
      const L = p.b - p.a, u = Math.min(s, p.b) - p.a;
      z += (p.g0 * u + (p.g1 - p.g0) * u * u / (2 * L)) / 100;
    }
    return z;
  };
  const closure = closed ? raw(total) - z0 : 0;
  // On a loop, past the seam keeps climbing by any closure error so block chords stay continuous.
  const z = s => {
    if (!closed) return raw(Math.min(Math.max(s, 0), total));
    const n = Math.floor(s / total);
    return raw(s - n * total) + n * closure;
  };
  const gradeAt = s => {
    if (closed) s = ((s % total) + total) % total;
    for (const p of pieces) if (s >= p.a && s <= p.b) return p.g0 + (p.g1 - p.g0) * (s - p.a) / (p.b - p.a);
    return 0;
  };
  // A block is the chord between the curve heights under its two ends, so joints meet on the curve.
  const sample = (s, hl) => {
    const a = closed ? s - hl : Math.max(s - hl, 0), b = closed ? s + hl : Math.min(s + hl, total);
    const za = z(a), zb = z(b);
    return { z: (za + zb) / 2, grade: b > a ? (zb - za) / (b - a) : 0 };
  };
  const cos = (s, hl) => { const g = sample(s, hl).grade; return 1 / Math.sqrt(1 + g * g); };
  return { z0, closure, active: pieces.length > 0 || z0 !== 0, pieces, z, gradeAt, sample, cos };
}
// Largest height step between neighbouring block ends. Overlap on a graded bend causes it, so
// tightening the bend gap would make it worse; it is reported rather than planned around.
function maxLip(poses, len) {
  let worst = 0;
  for (let i = 1; i < poses.length; i++) {
    const a = poses[i - 1], b = poses[i];
    if (a.path !== b.path || a.row !== b.row || a.kind || b.kind) continue;
    worst = Math.max(worst, Math.abs((b.z - 0.5 * len * Math.sin(b.pitch)) - (a.z + 0.5 * len * Math.sin(a.pitch))));
  }
  return worst;
}

function edgeGap(arc, a, b, len, wid, h, prof = FLAT_PROFILE) {
  const ca = arc.at(a), cb = arc.at(b), ta = arc.tangent(a, h), tb = arc.tangent(b, h);
  const halfA = 0.5 * len * prof.cos(a, 0.5 * len), halfB = 0.5 * len * prof.cos(b, 0.5 * len);   // pitched blocks cover less ground
  let m = V.norm(V.add(ta, tb)); if (m[0] === 0 && m[1] === 0) m = ta;
  const na = perp(ta), nb = perp(tb);
  let worst = -Infinity;
  for (const o of [-0.5 * wid, 0.5 * wid]) {
    const front = V.add(V.add(ca, V.mul(ta, halfA)), V.mul(na, o));
    const back = V.add(V.sub(cb, V.mul(tb, halfB)), V.mul(nb, o));
    worst = Math.max(worst, V.dot(V.sub(back, front), m));
  }
  return worst;
}
function planPositions(arc, len, wid, tol, prof = FLAT_PROFILE) {
  const h = Math.min(Math.max(len * 0.25, 0.1), 1.0), dmin = Math.max(len * 0.2, STEP_GRAIN);
  const gap = (a, b) => edgeGap(arc, a, b, len, wid, h, prof);
  const bestStep = s => { let d = len * prof.cos(s, 0.5 * len); while (d > dmin) { if (gap(s, s + d) <= tol) return d; d -= STEP_GRAIN; } return dmin; };
  const out = [], guard = (Math.ceil(arc.total / dmin) + 4) * 2;
  if (arc.closed) {
    const end = arc.total; let s = 0; out.push(s);
    for (let g = 0; g < guard; g++) {
      if (end - s <= len + 1e-4 && gap(s, end) <= tol) break;
      const d = bestStep(s);
      if (s + d >= end - 1e-4) { const mid = 0.5 * (s + end); if (mid - s < 1e-3) break; s = mid; } else s += d;
      out.push(s);
    }
  } else {
    const start = Math.min(0.5 * len, 0.5 * arc.total), end = Math.max(arc.total - 0.5 * len, start);
    let s = start; out.push(s);
    for (let g = 0; g < guard; g++) {
      if (end - s < 1e-3) break;
      if (end - s <= len + 1e-4 && gap(s, end) <= tol) { out.push(end); break; }
      s = Math.min(s + bestStep(s), end); out.push(s);
    }
  }
  return out;
}
// A line beside the centerline at a sideways offset (positive = left of travel) that may change
// along it. toS maps a distance along the line back to the centerline, which heights are read from.
function offsetLine(arc, offFn, h) {
  const n = arc.pts.length, pts = [];
  for (let i = 0; i < n; i++) pts.push(V.add(arc.pts[i], V.mul(perp(arc.tangent(arc.cum[i], h)), offFn(arc.cum[i]))));
  const cum = [0], ring = arc.closed ? n : n - 1;
  for (let i = 0; i < ring; i++) cum.push(cum[i] + V.len(V.sub(pts[(i + 1) % n], pts[i])));
  const total = cum[cum.length - 1], line = Arc.make(pts, arc.closed);
  if (!line) return null;
  const toS = so => {
    let wraps = 0;
    if (arc.closed) { wraps = Math.floor(so / total); so -= wraps * total; } else so = Math.min(Math.max(so, 0), total);
    let lo = 0, hi = cum.length - 1;
    while (hi - lo > 1) { const mid = (lo + hi) >> 1; if (cum[mid] <= so) lo = mid; else hi = mid; }
    const seg = cum[lo + 1] - cum[lo], f = seg > 1e-9 ? (so - cum[lo]) / seg : 0;
    return wraps * arc.total + arc.cum[lo] + f * (arc.cum[lo + 1] - arc.cum[lo]);
  };
  return { arc: line, total: line.total, toS };
}
// The centerline's height profile as seen from an offset line.
function lineProfile(prof, line) {
  if (!prof.active) return FLAT_PROFILE;
  const sample = (so, hl) => {
    const a = line.arc.closed ? so - hl : Math.max(so - hl, 0), b = line.arc.closed ? so + hl : Math.min(so + hl, line.total);
    const za = prof.z(line.toS(a)), zb = prof.z(line.toS(b));
    return { z: (za + zb) / 2, grade: b > a ? (zb - za) / (b - a) : 0 };
  };
  return { sample, cos: (so, hl) => { const g = sample(so, hl).grade; return 1 / Math.sqrt(1 + g * g); } };
}
function linePose(line, lp, so, len, h) {
  const t = line.arc.tangent(so, h), q = lp.sample(so, 0.5 * len);
  return { pos: line.arc.at(so), yaw: Math.atan2(t[1], t[0]), z: q.z, pitch: Math.atan(q.grade), s: line.toS(so) };
}
// A track wider than one piece: rows fan evenly between the two edges, so the edges stay straight
// when the width changes. Where the track narrows, rows that are no longer needed drop out.
function planRows(arc, prof, len, wid, tol, h, width, widest, tapered) {
  const N = Math.ceil(widest / wid - 1e-6), out = [], reach = wid - (tapered ? 0.15 : 0);
  const spread = s => (width(s) - wid) / (N - 1);        // gap between neighbouring row centres
  const kept = s => {                                    // fewest rows that still cover the width here
    const d = spread(s);
    if (d * (N - 1) <= 0.1) return new Set([0]);          // all rows coincide: one is enough
    const keep = new Set([0]), jump = Math.max(1, Math.floor(reach / d + 1e-6));
    for (let i = 0; i < N - 1;) { i = Math.min(N - 1, i + jump); keep.add(i); }
    return keep;
  };
  for (let j = 0; j < N; j++) {
    const line = offsetLine(arc, s => (j - (N - 1) / 2) * spread(s), h);
    if (!line) continue;
    const lp = lineProfile(prof, line);
    for (const so of planPositions(line.arc, len, wid, tol, lp)) {
      if ([-0.5, 0, 0.5].some(f => kept(line.toS(so + f * len)).has(j))) out.push({ ...linePose(line, lp, so, len, h), row: j });
    }
  }
  return out;
}
// Border pieces outside both edges, base level with the track surface. b.every spaces single
// pieces that far apart; without it the pieces run end to end as a rail.
function planBorder(arc, prof, b, tol, h, width, roadHgt) {
  // Normally the piece's base rests on the track surface. b.show sinks it so only that much of its
  // height stands above the surface.
  const out = [], lift = roadHgt / 2 + (b.show !== undefined ? b.show - b.pc.hgt / 2 : b.pc.hgt / 2);
  for (const side of [1, -1]) {
    const line = offsetLine(arc, s => side * (width(s) + b.pc.wid) / 2, h);
    if (!line) continue;
    const lp = lineProfile(prof, line);
    let at;
    if (b.every) {
      const start = arc.closed ? 0 : Math.min(b.every, line.total) / 2;
      const count = arc.closed ? Math.max(1, Math.round(line.total / b.every)) : Math.floor((line.total - start) / b.every + 1e-6) + 1;
      const step = arc.closed ? line.total / count : b.every;
      at = Array.from({ length: count }, (_, i) => start + i * step);
    } else at = planPositions(line.arc, b.pc.len, b.pc.wid, tol, lp);
    for (const so of at) { const q = linePose(line, lp, so, b.pc.len, h); out.push({ ...q, z: q.z + lift, pc: b.pc, kind: "border", row: side }); }
  }
  return out;
}
function planPath(path, len, wid, tol) {
  const prof = path.profile || FLAT_PROFILE;
  const arc = Arc.make(path.pts, path.closed);
  if (!arc) return path.pts.length ? [{ pos: path.pts[0], yaw: 0, z: prof.z0, pitch: 0, s: 0 }] : [];
  const h = Math.min(Math.max(len * 0.25, 0.1), 1.0);
  const single = () => planPositions(arc, len, wid, tol, prof).map(s => {
    const t = arc.tangent(s, h), q = prof.sample(s, 0.5 * len);
    return { pos: arc.at(s), yaw: Math.atan2(t[1], t[0]), z: q.z, pitch: Math.atan(q.grade), s };
  });
  // path.w0 / path.w1: track width at the start and end (a loop keeps w0). One piece wide by default.
  // path.wfn (width at a distance along the centerline, with its wmin / wmax) overrides that when
  // the width changes in the middle of the path.
  const w0 = Math.max(path.w0 || wid, wid), w1 = arc.closed ? w0 : Math.max(path.w1 || wid, wid);
  const width = path.wfn || (s => w0 + (w1 - w0) * Math.min(Math.max(s / arc.total, 0), 1));
  const wmax = path.wfn ? path.wmax : Math.max(w0, w1), varies = path.wfn ? path.wmax - path.wmin > 1e-6 : w0 !== w1;
  const poses = path.noRoad ? [] : wmax <= wid + 1e-6 ? single() : planRows(arc, prof, len, wid, tol, h, width, wmax, varies);
  if (path.border) poses.push(...planBorder(arc, prof, path.border, tol, h, width, path.roadHgt || 0));
  return poses;
}
// One flat floor under every path that has no road pieces of its own (path.noRoad): a grid of
// path.base pieces laid edge to edge, square to the image, covering those paths and their borders.
function basePlate(paths, wid) {
  const on = paths.filter(p => p.noRoad && p.base);
  if (!on.length) return [];
  let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
  for (const p of on) {
    const arc = Arc.make(p.pts, p.closed); if (!arc) continue;
    const w0 = Math.max(p.w0 || wid, wid), w1 = arc.closed ? w0 : Math.max(p.w1 || wid, wid);
    for (let i = 0; i < arc.pts.length; i++) {
      const s = arc.cum[i], n = perp(arc.tangent(s, 1)), half = (p.wfn ? p.wfn(s) : w0 + (w1 - w0) * s / arc.total) / 2 + (p.border ? p.border.pc.wid : 0);
      for (const side of [1, -1]) {
        const c = V.add(arc.pts[i], V.mul(n, side * half));
        x0 = Math.min(x0, c[0]); x1 = Math.max(x1, c[0]); y0 = Math.min(y0, c[1]); y1 = Math.max(y1, c[1]);
      }
    }
  }
  if (!(x1 > x0 || y1 > y0)) return [];
  const b = on[0].base, nx = Math.max(1, Math.ceil((x1 - x0) / b.len - 1e-6)), ny = Math.max(1, Math.ceil((y1 - y0) / b.wid - 1e-6));
  const ox = (x0 + x1) / 2 - nx * b.len / 2, oy = (y0 + y1) / 2 - ny * b.wid / 2;
  const z = on[0].profile.z0 + (on[0].roadHgt || 0) / 2 - b.hgt / 2, out = [];   // top face = track surface
  for (let i = 0; i < nx; i++) for (let j = 0; j < ny; j++) out.push({ pos: [ox + (i + 0.5) * b.len, oy + (j + 0.5) * b.wid], yaw: 0, z, pitch: 0, s: 0, pc: b, kind: "base", path: -1 });
  return out;
}
function totalLength(paths) { return paths.reduce((acc, p) => { const a = Arc.make(p.pts, p.closed); return acc + (a ? a.total : 0); }, 0); }
function plan(paths, len, wid, tol, budget, fit) {
  const run = t => paths.flatMap(p => planPath(p, len, wid, t)).map((q, i) => q);
  const runTagged = t => { const out = []; paths.forEach((p, k) => planPath(p, len, wid, t).forEach(q => out.push({ ...q, path: k }))); out.push(...basePlate(paths, wid)); return out; };
  const length = totalLength(paths);
  const first = runTagged(tol);
  if (first.length <= budget) return { ok: true, poses: first, tolUsed: tol, length };
  if (!fit) return { ok: false, poses: first, tolUsed: tol, length, why: `${first.length} blocks needed but only ${budget} slots are free` };
  for (const t of FIT_LADDER.filter(t => t > tol)) {
    const poses = runTagged(t);
    if (poses.length <= budget) return { ok: true, poses, tolUsed: t, length, relaxed: true };
  }
  return { ok: false, poses: first, tolUsed: tol, length, why: `even with ${FIT_LADDER[FIT_LADDER.length - 1]} wu gaps on bends this needs more than ${budget} blocks` };
}

// ============================================================================================
// State
// ============================================================================================
const COLORS = ["#ffb020", "#c084fc", "#2dd4bf", "#f472b6", "#a3e635", "#f87171"];
const S = {
  game: "halo4",                 // which Halo the prefab is for: halo4 | reach
  img: null, imgW: 1024, imgH: 768, sample: true, imgRot: 0,   // imgRot: degrees, counter-clockwise
  wuPerPx: 4 / 30,               // sample: lanes are ~30 px wide
  paths: [], active: 0, layout: 1, ghost: null,
  kind: "s", turnMode: "set",   // Trace tool: next section is a straight or a turn; a turn uses a set angle or bends freely
  jumps: [], jumpSel: null,     // jump markers: { pos: [x, y] image px, rot: degrees (0 = image right, counter-clockwise), size }
  mode: "trace", view: { z: 1, x: 0, y: 0 },
  cal: [], sel: null, drag: null, hover: null,
  gradePick: [], gradeHover: null, profHover: null,   // pending section ends, and hover dots
  undo: [],
};
const $ = id => document.getElementById(id);
const cv = $("cv"), ctx = cv.getContext("2d"), board = $("board");
const pv = $("prof"), pctx = pv.getContext("2d");

// Forge pieces for each game, from HMS's cross-game catalog (crates/hms-app/src/xgame/catalog.json).
// Each row: [catalog key, tag, dims, center, palette spot on that game's canvas]. dims and center
// are in the piece's own axes; center is the middle of its box measured from its pivot.
const PIECE_DB = {
  halo4: {
    bb_1x1: ["fi_unsc_c_plat_1x1#bb_1x1","levels\\dlc\\dlc_forge_island\\forge\\fi_unsc_c_plat_1x1\\fi_unsc_c_plat_1x1",[1,1,1],[0,0,0.5],[11,91,0]],
    bb_1x1_flat: ["fi_unsc_c_plat_1x1_flat#bb_1x1_flat","levels\\dlc\\dlc_forge_island\\objects\\fi_unsc_c_plat_1x1_flat\\fi_unsc_c_plat_1x1_flat",[1,1,0.2],[0,0,0],[11,91,1]],
    bb_1x4: ["fi_unsc_c_plat_1x4#bb_1x4","levels\\dlc\\dlc_forge_island\\objects\\fi_unsc_c_plat_1x4\\fi_unsc_c_plat_1x4",[4,1,1],[0,0,0],[11,91,6]],
    bb_2x1_flat: ["fi_unsc_c_plat_2x1_flat#bb_2x1_flat","levels\\dlc\\dlc_forge_island\\objects\\fi_unsc_c_plat_2x1_flat\\fi_unsc_c_plat_2x1_flat",[2,1,0.2],[0,0,0],[11,91,7]],
    bb_2x2: ["fi_unsc_c_plat_2x2#bb_2x2","levels\\dlc\\dlc_forge_island\\objects\\fi_unsc_c_plat_2x2\\fi_unsc_c_plat_2x2",[2,2,1],[0,0,0],[11,91,8]],
    bb_2x2_flat: ["fi_unsc_c_plat_2x2_flat#bb_2x2_flat","levels\\dlc\\dlc_forge_island\\objects\\fi_unsc_c_plat_2x2_flat\\fi_unsc_c_plat_2x2_flat",[2,2.0195,0.207],[0,0.0005,0.0034],[11,91,9]],
    bb_2x2_short: ["fi_unsc_c_plat_2x2_short#bb_2x2_short","levels\\dlc\\dlc_forge_island\\objects\\fi_unsc_c_plat_2x2_short\\fi_unsc_c_plat_2x2_short",[2,2,0.5],[0,0,0.05],[11,91,10]],
    bb_2x3: ["fi_unsc_c_plat_2x3#bb_2x3","levels\\dlc\\dlc_forge_island\\objects\\fi_unsc_c_plat_2x3\\fi_unsc_c_plat_2x3",[2,2.9943,0.9977],[0,-0.0012,0.0036],[11,91,12]],
    bb_2x4: ["fi_unsc_c_plat_2x4#bb_2x4","levels\\dlc\\dlc_forge_island\\objects\\fi_unsc_c_plat_2x4\\fi_unsc_c_plat_2x4",[2,4,1.0198],[0,0,0.0099],[11,91,13]],
    bb_3x1_flat: ["fi_unsc_c_plat_3x1_flat#bb_3x1_flat","levels\\dlc\\dlc_forge_island\\objects\\fi_unsc_c_plat_3x1_flat\\fi_unsc_c_plat_3x1_flat",[3,1.0308,0.2118],[0,0,0.0059],[11,91,14]],
    bb_3x3: ["fi_unsc_c_plat_3x3#bb_3x3","levels\\dlc\\dlc_forge_island\\objects\\fi_unsc_c_plat_3x3\\fi_unsc_c_plat_3x3",[3,3,1],[0,0,0.0009],[11,91,15]],
    bb_3x3_flat: ["fi_unsc_c_plat_3x3_flat#bb_3x3_flat","levels\\dlc\\dlc_forge_island\\objects\\fi_unsc_c_plat_3x3_flat\\fi_unsc_c_plat_3x3_flat",[3,3.0308,0.2111],[0,0,0.0056],[11,91,16]],
    bb_3x3_short: ["fi_unsc_c_plat_3x3_short#bb_3x3_short","levels\\dlc\\dlc_forge_island\\objects\\fi_unsc_c_plat_3x3_short\\fi_unsc_c_plat_3x3_short",[3,3,0.5],[0,0,-0.05],[11,91,17]],
    bb_3x4: ["fi_unsc_c_plat_3x4#bb_3x4","levels\\dlc\\dlc_forge_island\\objects\\fi_unsc_c_plat_3x4\\fi_unsc_c_plat_3x4",[4,3,1],[0,0,0],[11,91,19]],
    bb_4x4: ["fi_unsc_c_plat_4x4#bb_4x4","levels\\dlc\\dlc_forge_island\\forge\\fi_unsc_c_plat_4x4\\fi_unsc_c_plat_4x4",[4,4,1],[0,0,0],[11,91,20]],
    bb_4x4_flat: ["fi_unsc_c_plat_4x4_flat#bb_4x4_flat","levels\\dlc\\dlc_forge_island\\objects\\fi_unsc_c_plat_4x4_flat\\fi_unsc_c_plat_4x4_flat",[3.9996,4,0.2],[0,0,0],[11,91,21]],
    bb_4x4_short: ["fi_unsc_c_plat_4x4_short#bb_4x4_short","levels\\dlc\\dlc_forge_island\\forge\\fi_unsc_c_plat_4x4_short\\fi_unsc_c_plat_4x4_short",[4,4,0.5],[0,0,0.05],[11,91,22]],
    bb_5x1_short: ["fi_unsc_c_plat_5x1_flat#bb_5x1_short","levels\\dlc\\dlc_forge_island\\forge\\fi_unsc_c_plat_5x1_flat\\fi_unsc_c_plat_5x1_flat",[5.0499,1.0097,0.2389],[0.0004,0.0006,0.1001],[11,91,24]],
    bb_5x5_flat: ["fi_unsc_c_plat_5x5_flat#bb_5x5_flat","levels\\dlc\\dlc_forge_island\\objects\\fi_unsc_c_plat_5x5_flat\\fi_unsc_c_plat_5x5_flat",[5.0516,5.051,0.2407],[0.0007,0.0001,-0.0011],[11,91,25]],
    bp_bridge_med: ["fi_unsc_c_bridge_2x2#bp_bridge_med","levels\\dlc\\dlc_forge_island\\forge\\fi_unsc_c_bridge_2x2\\fi_unsc_c_bridge_2x2",[1.9965,2.4,0.5983],[-0.0026,0,0.1006],[11,92,1]],
    bp_bridge_lg: ["fi_unsc_c_bridge_2x3#bp_bridge_lg","levels\\dlc\\dlc_forge_island\\forge\\fi_unsc_c_bridge_2x3\\fi_unsc_c_bridge_2x3",[3.0009,2.4,0.5983],[0.0004,0,0.1006],[11,92,2]],
    bp_bridge_xlg: ["fi_unsc_c_bridge_2x5#bp_bridge_xlg","levels\\dlc\\dlc_forge_island\\forge\\fi_unsc_c_bridge_2x5\\fi_unsc_c_bridge_2x5",[5.0022,2.4,0.5983],[0.0052,0,0.1006],[11,92,3]],
    drw_wall: ["fi_unsc_c_wall_2x2#drw_wall","levels\\dlc\\dlc_forge_island\\forge\\fi_unsc_c_wall_2x2\\fi_unsc_c_wall_2x2",[2,0.2,2],[0,0,0],[11,95,4]],
    drw_wall_double: ["fi_unsc_c_wall_2x4#drw_wall_double","levels\\dlc\\dlc_forge_island\\objects\\fi_unsc_c_wall_2x4\\fi_unsc_c_wall_2x4",[4,0.2,2],[0,0,0],[11,95,5]],
    drw_wall_coliseum: ["fi_unsc_c_arena_wall#drw_wall_coliseum","levels\\dlc\\dlc_forge_island\\forge\\fi_unsc_c_arena_wall\\fi_unsc_c_arena_wall",[10,0.6,11.8],[0,0,0],[11,95,8]],
    dec_column: ["fi_unsc_c_column#dec_column","levels\\dlc\\dlc_forge_island\\objects\\fi_unsc_c_column\\fi_unsc_c_column",[0.3,0.3032,1],[0.05,-0.0516,0.5],[11,94,5]],
    dec_cover: ["fi_unsc_c_crenel_full#dec_cover","levels\\dlc\\dlc_forge_island\\objects\\fi_unsc_c_crenel_full\\fi_unsc_c_crenel_full",[2.4,0.4538,1.6],[0,-0.073,-0.1],[11,94,6]],
    skyline_wall_panel: ["ca_rattler_wallpanel#skyline_wall_panel","objects\\ca\\dlc\\ca_rattler\\ca_rattler_wallpanel\\ca_rattler_wallpanel",[0.1812,2.286,1.025],[0,0.0003,0],[16,123,4]],
  },
  reach: {
    block_1x1: ["ff_plat_1x1#block_1x1","objects\\levels\\forge\\ff_plat_1x1\\ff_plat_1x1",[1.0182,1.0182,1],[0,0,0],[8,70,0]],
    block_1x1_flat: ["ff_plat_1x1_flat#block_1x1_flat","objects\\levels\\forge\\ff_plat_1x1_flat\\ff_plat_1x1_flat",[1.0176,1.0176,0.2],[-0.0001,0.0001,0],[8,70,1]],
    block_1x4: ["ff_plat_1x4#block_1x4","objects\\levels\\forge\\ff_plat_1x4\\ff_plat_1x4",[4.0155,1.0182,1.0182],[0,0.0001,0],[8,70,6]],
    block_2x1_flat: ["ff_plat_2x1_flat#block_2x1_flat","objects\\levels\\forge\\ff_plat_2x1_flat\\ff_plat_2x1_flat",[2,1.0294,0.255],[0,0.0147,-0.0175],[8,70,7]],
    block_2x2: ["ff_plat_2x2#block_2x2","objects\\levels\\forge\\ff_plat_2x2\\ff_plat_2x2",[2,2,0.9999],[0,0,-0.0001],[8,70,8]],
    block_2x2_flat: ["ff_plat_2x2_flat#block_2x2_flat","objects\\levels\\forge\\ff_plat_2x2_flat\\ff_plat_2x2_flat",[2,2,0.255],[0,0,-0.0175],[8,70,9]],
    block_2x2_short: ["ff_plat_2x2_short#block_2x2_short","objects\\levels\\forge\\ff_plat_2x2_short\\ff_plat_2x2_short",[2,2,0.4999],[0,0,0.05],[8,70,10]],
    block_2x3: ["ff_plat_2x3#block_2x3","objects\\levels\\forge\\ff_plat_2x3\\ff_plat_2x3",[2,3,1],[0,0,0],[8,70,12]],
    block_2x4: ["ff_plat_2x4#block_2x4","objects\\levels\\forge\\ff_plat_2x4\\ff_plat_2x4",[2,4,1],[0,0,0],[8,70,13]],
    block_3x1_flat: ["ff_plat_3x1_flat#block_3x1_flat","objects\\levels\\forge\\ff_plat_3x1_flat\\ff_plat_3x1_flat",[3,1,0.2],[0,0,0],[8,70,14]],
    block_3x3: ["ff_plat_3x3#block_3x3","objects\\levels\\forge\\ff_plat_3x3\\ff_plat_3x3",[3,3,1],[0,0,0],[8,70,15]],
    block_3x3_flat: ["ff_plat_3x3_flat#block_3x3_flat","objects\\levels\\forge\\ff_plat_3x3_flat\\ff_plat_3x3_flat",[3,3,0.2995],[0,0,-0.0497],[8,70,16]],
    block_3x3_short: ["ff_plat_3x3_short#block_3x3_short","objects\\levels\\forge\\ff_plat_3x3_short\\ff_plat_3x3_short",[3,3,0.4999],[0,0,0.05],[8,70,17]],
    block_3x4: ["ff_plat_3x4#block_3x4","objects\\levels\\forge\\ff_plat_3x4\\ff_plat_3x4",[3.9987,3.0003,0.9999],[0.0008,0,0.0001],[8,70,19]],
    block_4x4: ["ff_plat_4x4#block_4x4","objects\\levels\\forge\\ff_plat_4x4\\ff_plat_4x4",[4.0005,4.0005,0.9999],[0,0,0],[8,70,20]],
    block_4x4_flat: ["ff_plat_4x4_flat#block_4x4_flat","objects\\levels\\forge\\ff_plat_4x4_flat\\ff_plat_4x4_flat",[4,4,0.2],[0,0,0],[8,70,21]],
    block_4x4_short: ["ff_plat_4x4_short#block_4x4_short","objects\\levels\\forge\\ff_plat_4x4_short\\ff_plat_4x4_short",[4,4,0.5],[0,0,0],[8,70,22]],
    block_5x1_flat: ["ff_plat_5x1_flat#block_5x1_flat","objects\\levels\\forge\\ff_plat_5x1_flat\\ff_plat_5x1_flat",[5,1,0.2217],[0,0,0],[8,70,24]],
    block_5x5_flat: ["ff_plat_5x5_flat#block_5x5_flat","objects\\levels\\forge\\ff_plat_5x5_flat\\ff_plat_5x5_flat",[5,5,0.2],[0,0,0],[8,70,25]],
    bridge_2x2: ["ff_bridge_2x2#bridge_2x2","objects\\levels\\forge\\ff_bridge_2x2\\ff_bridge_2x2",[2,1.9999,0.6024],[0,0.0001,0.0988],[8,71,1]],
    bridge_2x3: ["ff_bridge_2x3#bridge_2x3","objects\\levels\\forge\\ff_bridge_2x3\\ff_bridge_2x3",[2.9989,1.9999,0.6024],[0,0,0.1012],[8,71,2]],
    bridge_2x5: ["ff_bridge_2x5#bridge_2x5","objects\\levels\\forge\\ff_bridge_2x5\\ff_bridge_2x5",[5,1.9999,0.6024],[0,0,0.1012],[8,71,3]],
    wall: ["ff_wall_2x2#wall","objects\\levels\\forge\\ff_wall_2x2\\ff_wall_2x2",[2.0006,0.2325,1.9974],[0,0,0.0014],[8,74,4]],
    wall_double: ["ff_wall_2x4#wall_double","objects\\levels\\forge\\ff_wall_2x4\\ff_wall_2x4",[3.9999,0.2325,1.9974],[0,0,0.0014],[8,74,5]],
    wall_coliseum: ["ff_arena_wall#wall_coliseum","objects\\levels\\forge\\ff_arena_wall\\ff_arena_wall",[10,0.5,12],[0,0,0],[8,74,8]],
    ff_column: ["ff_column","objects\\levels\\forge\\ff_column\\ff_column",[0.3,0.3,1],[0,0,0.5],[8,73,5]],
    crenelation: ["ff_crenel_full#crenelation","objects\\levels\\forge\\ff_crenel_full\\ff_crenel_full",[2.0001,0.3382,1.3],[0,-0.0292,-0.15],[8,73,6]],
    railing_short: ["ff_railing_1#railing_short","objects\\levels\\forge\\ff_railing_1\\ff_railing_1",[0.2,0.9979,0.3988],[0,0,0.1994],[8,73,10]],
    railing_medium: ["ff_railing_2#railing_medium","objects\\levels\\forge\\ff_railing_2\\ff_railing_2",[0.2,1.999,0.3988],[0,0,0.1994],[8,73,11]],
    railing_long: ["ff_railing_3#railing_long","objects\\levels\\forge\\ff_railing_3\\ff_railing_3",[0.2,2.9998,0.3988],[0,0,0.1994],[8,73,12]],
  },

};
const GAMES = {
  halo4: { label: "Halo 4", map: "Forge Island", canvas: "dlc_forge_island" },
  reach: { label: "Halo: Reach", map: "Forge World", canvas: "forge_halo" },
};
// lay says where the piece's x, y, z axes point on the track: A = along travel, L = left, U = up
// (lower case = reversed). maxAllowed: Forge's per-object limit where it is known (0 = unknown).
function mkPiece(game, variant, display, lay = "ALU", label, maxAllowed = 0) {
  const [key, tag, dims, center, spot] = PIECE_DB[game][variant];
  const size = c => dims[lay.toUpperCase().indexOf(c)];
  return { name: variant, game, display, label: label || display, dims, center, lay, maxAllowed, len: size("A"), wid: size("L"), hgt: size("U"),
    prefab: { key, display, variant, spot, tag, canvas: GAMES[game].canvas } };
}
// What each game offers: road pieces (the path), border pieces (the outline), the baseplate under a
// flat bordered track, the jump marker, and the border a start straight uses.
function buildKit(game, road, border, base, jump, start) {
  return {
    road: Object.fromEntries(road.map(r => [r[0], mkPiece(game, ...r)])),
    border: Object.fromEntries(Object.entries(border).map(([id, b]) => [id, { ...b, pc: mkPiece(game, ...b.pc) }])),
    base: mkPiece(game, ...base), jump: mkPiece(game, ...jump), start: { ...start, pc: mkPiece(game, ...start.pc) },
  };
}
const KIT = {
  halo4: buildKit("halo4", [
    ["bb_4x4", "Block, 4X4", "ALU", "", 300], ["bb_4x4_short", "Block, 4X4, Short", "ALU", "", 300], ["bb_4x4_flat", "Block, 4X4, Flat", "ALU", "", 300],
    ["bb_3x4", "Block, 3X4", "ALU", "", 300], ["bb_3x3", "Block, 3X3", "ALU", "", 300], ["bb_3x3_short", "Block, 3X3, Short", "ALU", "", 300], ["bb_3x3_flat", "Block, 3X3, Flat", "ALU", "", 300],
    ["bb_2x4", "Block, 2X4", "ALU", "", 300], ["bb_2x3", "Block, 2X3", "ALU", "", 300], ["bb_2x2", "Block, 2X2", "ALU", "", 300], ["bb_2x2_short", "Block, 2X2, Short", "ALU", "", 300], ["bb_2x2_flat", "Block, 2X2, Flat", "ALU", "", 300],
    ["bb_5x5_flat", "Block, 5X5, Flat", "ALU", "", 300], ["bb_1x4", "Block, 1X4", "ALU", "", 300], ["bb_5x1_short", "Block, 5X1, Short", "ALU", "", 300],
    ["bb_3x1_flat", "Block, 3X1, Flat", "ALU", "", 300], ["bb_2x1_flat", "Block, 2X1, Flat", "ALU", "", 300],
    ["bp_bridge_xlg", "Bridge, Extra Large", "ALU", "", 300], ["bp_bridge_lg", "Bridge, Large", "ALU", "", 300], ["bp_bridge_med", "Bridge, Medium", "ALU", "", 300],
    ["drw_wall_double", "Wall, Double", "LUA", "Wall, Double, laid flat", 150], ["drw_wall", "Wall", "LUA", "Wall, laid flat", 150],
  ], {
    column: { label: "Columns, every 4 wu", pc: ["dec_column", "Column", "LUA", "", 300], every: 4 },
    railEdge: { label: "5X1 rail, on edge", pc: ["bb_5x1_short", "Block, 5X1, Short", "AUl", "", 300] },
    railFlat: { label: "5X1 rail, flat", pc: ["bb_5x1_short", "Block, 5X1, Short", "ALU", "", 300] },
    flat3Edge: { label: "3X1 flat, on edge", pc: ["bb_3x1_flat", "Block, 3X1, Flat", "AUl", "", 300] },
    flat2Edge: { label: "2X1 flat, on edge", pc: ["bb_2x1_flat", "Block, 2X1, Flat", "AUl", "", 300] },
    curb: { label: "1X4 block curb", pc: ["bb_1x4", "Block, 1X4", "ALU", "", 300] },
    wall: { label: "Walls, upright", pc: ["drw_wall", "Wall", "ALU", "", 150] },
    wallDouble: { label: "Double walls, upright", pc: ["drw_wall_double", "Wall, Double", "ALU", "", 150] },
    cover: { label: "Cover (crenellation)", pc: ["dec_cover", "Cover", "ALU", "", 150] },
    skyline: { label: "Skyline wall panels, every 4 wu", pc: ["skyline_wall_panel", "Skyline Wall Panel", "lAU", "", 300], every: 4, show: 0.1 },
  }, ["drw_wall_coliseum", "Wall, Coliseum", "LUA", "", 150], ["bb_4x4_flat", "Block, 4X4, Flat", "LUA", "", 300],
  // a start straight swaps columns for Skyline Wall Panels at the same spacing
  { pc: ["skyline_wall_panel", "Skyline Wall Panel", "lAU", "", 300], every: 4, show: 0.1 }),
  reach: buildKit("reach", [
    ["block_4x4", "Block 4x4"], ["block_4x4_short", "Block 4x4 Short"], ["block_4x4_flat", "Block 4x4 Flat"],
    ["block_3x4", "Block 3x4"], ["block_3x3", "Block 3x3"], ["block_3x3_short", "Block 3x3 Short"], ["block_3x3_flat", "Block 3x3 Flat"],
    ["block_2x4", "Block 2x4"], ["block_2x3", "Block 2x3"], ["block_2x2", "Block 2x2"], ["block_2x2_short", "Block 2x2 Short"], ["block_2x2_flat", "Block 2x2 Flat"],
    ["block_5x5_flat", "Block 5x5 Flat"], ["block_1x4", "Block 1x4"], ["block_5x1_flat", "Block 5x1 Flat"],
    ["block_3x1_flat", "Block 3x1 Flat"], ["block_2x1_flat", "Block 2x1 Flat"],
    ["bridge_2x5", "Bridge, Large"], ["bridge_2x3", "Bridge, Medium"], ["bridge_2x2", "Bridge, Small"],
    ["wall_double", "Wall, Double", "LUA", "Wall, Double, laid flat"], ["wall", "Wall", "LUA", "Wall, laid flat"],
  ], {
    column: { label: "Columns, every 4 wu", pc: ["ff_column", "Column", "LUA"], every: 4 },
    railEdge: { label: "5x1 rail, on edge", pc: ["block_5x1_flat", "Block 5x1 Flat", "AUl"] },
    railFlat: { label: "5x1 rail, flat", pc: ["block_5x1_flat", "Block 5x1 Flat", "ALU"] },
    flat3Edge: { label: "3x1 flat, on edge", pc: ["block_3x1_flat", "Block 3x1 Flat", "AUl"] },
    flat2Edge: { label: "2x1 flat, on edge", pc: ["block_2x1_flat", "Block 2x1 Flat", "AUl"] },
    curb: { label: "1x4 block curb", pc: ["block_1x4", "Block 1x4"] },
    railing: { label: "Railings, long", pc: ["railing_long", "Railing, Long", "lAU"] },
    wall: { label: "Walls, upright", pc: ["wall", "Wall"] },
    wallDouble: { label: "Double walls, upright", pc: ["wall_double", "Wall, Double"] },
    cover: { label: "Crenellation", pc: ["crenelation", "Crenellation"] },
  }, ["wall_coliseum", "Wall, Coliseum", "LUA"], ["block_4x4_flat", "Block 4x4 Flat", "LUA"],
  // Reach has no Skyline panel: a start straight gets low wall strips at the same spacing
  { pc: ["wall", "Wall"], every: 4, show: 0.1 }),
};
const kit = () => KIT[S.game] || KIT.halo4;
const DEFAULT_ROAD = { halo4: "bb_4x4", reach: "block_4x4" };
// Fill the Piece and Border style lists for the current game, keeping the choices where they exist.
function fillPieceLists() {
  const k = kit(), pick = $("piece").value, bord = $("borderKind").value;
  $("piece").textContent = ""; $("borderKind").textContent = "";
  for (const [id, pc] of Object.entries(k.road)) {
    const o = document.createElement("option");
    o.value = id; o.textContent = `${pc.label} (${fmt(pc.len)} × ${fmt(pc.wid)} × ${fmt(pc.hgt)})`; $("piece").append(o);
  }
  for (const [id, b] of Object.entries(k.border)) {
    const o = document.createElement("option"); o.value = id; o.textContent = b.label; $("borderKind").append(o);
  }
  $("piece").value = k.road[pick] ? pick : DEFAULT_ROAD[S.game];
  $("borderKind").value = k.border[bord] ? bord : "column";
  $("game").value = S.game;
}
const JUMPS = { whoop: 0.25, small: 0.5, medium: 0.75, large: 1 };
const cleanJumps = arr => (Array.isArray(arr) ? arr : []).filter(j => j && Array.isArray(j.pos) && JUMPS[j.size])
  .map(j => ({ pos: [+j.pos[0], +j.pos[1]], rot: +j.rot || 0, size: j.size }));
function cleanSec(s) {
  const v2 = a => Array.isArray(a) && a.length >= 2 && isFinite(+a[0]) && isFinite(+a[1]) ? [+a[0], +a[1]] : null;
  const out = !s ? null : s.k === "s" && +s.len > 0 ? { k: "s", len: +s.len }
    : s.k === "t" && +s.r > 0 && +s.ang ? { k: "t", ang: Math.max(-355, Math.min(355, +s.ang)), r: +s.r, ...(s.free ? { free: true } : {}) }
    : s.k === "c" && v2(s.c1) && v2(s.c2) && v2(s.e) && V.len(v2(s.e)) > 1e-6 ? { k: "c", c1: v2(s.c1), c2: v2(s.c2), e: v2(s.e) } : null;
  if (out && isFinite(+s.kink) && Math.abs(+s.kink) > 1e-9) out.kink = wrap180(+s.kink);
  return out;
}
// Paths from older versions (traced points) become an empty path with the old line as a guide.
function cleanPath(p) {
  const anchor = a => ({ seg: Math.max(0, a.seg | 0), t: Math.min(Math.max(+a.t || 0, 0), 1) });
  const pt = q => Array.isArray(q) && q.length >= 2 && isFinite(+q[0]) && isFinite(+q[1]) ? [+q[0], +q[1]] : null;
  const modern = Array.isArray(p.secs), out = {
    name: String(p.name || "Path"), closed: !!p.closed, z0: +p.z0 || 0, border: !!p.border,
    taper: p.taper ? { w0: +p.taper.w0 || 20, w1: +p.taper.w1 || 4 } : undefined,
    width: +p.width > 0 ? Math.min(+p.width, 200) : undefined, width1: +p.width1 > 0 ? Math.min(+p.width1, 200) : undefined,
    wstops: modern && Array.isArray(p.wstops) ? p.wstops.filter(s => s && s.a && +s.w > 0).map(s => ({ a: anchor(s.a), w: Math.min(+s.w, 200) })) : [],
    start: modern ? pt(p.start) : null, head: modern && p.head != null && isFinite(+p.head) ? +p.head : null,
    secs: modern ? p.secs.map(cleanSec).filter(Boolean) : [],
    grades: modern && Array.isArray(p.grades) ? p.grades.filter(g => g && g.a && g.b).map(g => ({ a: anchor(g.a), b: anchor(g.b), g0: clampGrade(+g.g0 || 0), g1: clampGrade(+g.g1 || 0), curve: !!g.curve })) : [],
  };
  if (!out.start) out.secs = [];
  const old = !modern && Array.isArray(p.pts) ? p.pts.map(pt).filter(Boolean) : [];
  if (old.length >= 2) {
    const closed = out.closed && old.length >= 3, segs = Array.isArray(p.segs) ? p.segs.map(s => s && s.k === "c" && pt(s.h) ? { k: "c", h: pt(s.h) } : s && s.k === "l" ? { k: "l" } : null) : [];
    out.guide = pathLine({ smooth: p.smooth !== false, segs }, old, closed, 4);
    if (closed) out.guide.push(out.guide[0]);
  } else if (Array.isArray(p.guide)) out.guide = p.guide.map(pt).filter(Boolean);
  return out;
}

function snapshot() { return JSON.stringify({ paths: S.paths, jumps: S.jumps, active: S.active, wuPerPx: S.wuPerPx, layout: S.layout, imgRot: S.imgRot }); }
function pushUndo() { S.undo.push(snapshot()); if (S.undo.length > 100) S.undo.shift(); }
function undo() {
  const s = S.undo.pop(); if (!s) return;
  const o = JSON.parse(s); S.paths = o.paths; S.active = Math.max(0, Math.min(o.active, S.paths.length - 1)); S.wuPerPx = o.wuPerPx; S.layout = clampLayout(o.layout); S.imgRot = +o.imgRot || 0; S.jumps = o.jumps || []; S.jumpSel = null; S.sel = null; S.ghost = null;
  changed();
}
function save() { try { localStorage.setItem("trackTracer.v1", JSON.stringify({ paths: S.paths, jumps: S.jumps, active: S.active, wuPerPx: S.wuPerPx, layout: S.layout, imgRot: S.imgRot, imgW: S.imgW, imgH: S.imgH, sample: S.sample, game: S.game })); } catch (e) {} }
const clampRot = v => Math.min(Math.max(+v || 0, -45), 45);

// image px <-> world units (origin = image centre, +Y = image top)
// S.wuPerPx comes from Set scale (lane width); S.layout then resizes the whole layout while blocks keep their size.
const clampLayout = v => Math.min(Math.max(+v || 1, 0.5), 2);
const wpp = () => S.wuPerPx * S.layout;
const toWu = p => [(p[0] - S.imgW / 2) * wpp(), (S.imgH / 2 - p[1]) * wpp()];
const toPx = w => [w[0] / wpp() + S.imgW / 2, S.imgH / 2 - w[1] / wpp()];

// ============================================================================================
// Sample board (a synthetic stadium layout, so the tool opens in a working state)
// ============================================================================================
function samplePaths() {   // image px on a 1024x768 board
  const st = len => ({ k: "s", len }), turn = (ang, r) => ({ k: "t", ang, r });
  return [
    { name: "Main loop", closed: true, start: [230, 230], head: 0, grades: [],
      secs: [st(470), turn(-180, 85), st(320), turn(180, 80), st(380), turn(-180, 65), st(530), turn(-90, 60), st(340), turn(-90, 60)] },
    { name: "Start straight", closed: false, start: [95, 180], head: 0, grades: [], secs: [st(325)] },
  ];
}
function drawSampleImage() {
  const c = document.createElement("canvas"); c.width = 1024; c.height = 768;
  const g = c.getContext("2d");
  g.fillStyle = "#8f98a3"; g.fillRect(0, 0, 1024, 768);
  g.fillStyle = "#d9dde2"; g.beginPath(); g.roundRect(40, 120, 944, 610, 40); g.fill();
  const lane = (p, closed) => {
    const sp = pathGeom(p, 3).line;
    g.lineJoin = "round"; g.lineCap = "round";
    g.strokeStyle = "#f4f4f4"; g.lineWidth = 36; g.beginPath(); sp.forEach((p, i) => i ? g.lineTo(p[0], p[1]) : g.moveTo(p[0], p[1])); if (closed) g.closePath(); g.stroke();
    g.strokeStyle = "#8a5a2b"; g.lineWidth = 30; g.stroke();
  };
  const sp = samplePaths(); lane(sp[0], true); lane(sp[1], false);
  g.fillStyle = "#3c4148"; g.font = "600 22px sans-serif"; g.fillText("SAMPLE LAYOUT — load your own track image", 60, 90);
  const im = new Image(); im.src = c.toDataURL(); return im;
}

// ============================================================================================
// Rendering
// ============================================================================================
let planCache = null, raf = 0;
function piece() {
  const k = kit(); return k.road[$("piece").value] || k.road[DEFAULT_ROAD[S.game]];
}
function wuPaths() {
  const out = S.paths.filter(p => p.start && p.secs.length).map(p => {
    const g = pathGeom(p, 0.25 / wpp()), pts = g.line.map(toWu), ctrl = g.poses.map(q => toWu(q.pt));
    const closed = p.closed && pts.length >= 3, pc = piece(), tw = Math.max(+$("trackW").value || 0, pc.wid);
    if (closed && V.len(V.sub(ctrl[ctrl.length - 1], ctrl[0])) < 1e-3) { pts.pop(); ctrl.pop(); }   // a loop that closes exactly
    const taper = p.taper && !closed ? p.taper : null;
    // Width: a start straight runs from its first width to its second. Any other path has one width
    // (p.width, else the Blocks track width) and, on an open path, may end at another (p.width1).
    const at = v => Math.max(+v || 0, pc.wid), start = taper ? at(taper.w0) : +p.width > 0 ? at(p.width) : tw;
    const end = closed ? start : taper ? at(taper.w1) : +p.width1 > 0 ? at(p.width1) : start;
    const o = attachProfile({
      closed, pts, src: p, w0: start, w1: end,
      // a start straight swaps the columns for Skyline Wall Panels at the same spacing
      border: p.border ? (taper && $("borderKind").value === "column" ? kit().start : kit().border[$("borderKind").value] || kit().border.column) : null, roadHgt: pc.hgt,
    }, ctrl);
    attachWidth(o, p, pc, start, end);
    return o;
  });
  // On a flat track, bordered paths drop their road pieces and share one baseplate instead.
  const bordered = out.filter(w => w.border);
  const flat = out.every(w => !w.profile.pieces.length) && bordered.every(w => w.profile.z0 === bordered[0].profile.z0);
  for (const w of bordered) { w.noRoad = flat; w.base = kit().base; }
  return out;
}
// Width points (p.wstops) sit on the path like grade sections do. The width runs in straight lines
// from the start width through each point to the end width, so a point in the middle widens (or
// narrows) the track there and eases back.
function attachWidth(o, p, pc, start, end) {
  o.wstopsR = [];
  if (!o.arc || !o.station) return;
  for (const st of p.wstops || []) o.wstopsR.push({ src: st, s: o.station(st.a), w: Math.max(+st.w || 0, pc.wid) });
  o.wstopsR.sort((a, b) => a.s - b.s);
  if (!o.wstopsR.length) return;
  const total = o.total, ctrl = [{ s: 0, w: start }, ...o.wstopsR, { s: total, w: end }];
  o.wmin = Math.min(...ctrl.map(c => c.w)); o.wmax = Math.max(...ctrl.map(c => c.w));
  o.wfn = s => {
    s = o.closed ? ((s % total) + total) % total : Math.min(Math.max(s, 0), total);
    for (let i = 1; i < ctrl.length; i++) {
      const a = ctrl[i - 1], b = ctrl[i];
      if (s <= b.s && b.s - a.s > 1e-9) return a.w + (b.w - a.w) * (s - a.s) / (b.s - a.s);
    }
    return end;
  };
}
// Works out where each control point falls along the curve, resolves the path's grade sections
// to arc lengths, and builds its height profile.
function attachProfile(out, ctrl) {
  const p = out.src, z0 = +p.z0 || 0, { pts, closed } = out;
  out.arc = Arc.make(pts, closed); out.sections = [];
  if (!out.arc) { out.profile = makeProfile([], 0, false, z0); return out; }
  const total = out.total = out.arc.total, cum = out.cum = [0];
  const edges = closed ? pts.length : pts.length - 1;
  for (let i = 0; i < edges; i++) cum.push(cum[i] + V.len(V.sub(pts[(i + 1) % pts.length], pts[i])));
  const st = out.stations = [];
  let j = 0;
  for (const c of ctrl) {   // the curve passes exactly through each control point, in order
    let k = j;
    while (k < pts.length && V.len(V.sub(pts[k], c)) > 2e-4) k++;
    if (k < pts.length) j = k;
    st.push(cum[j]);
  }
  if (closed) st.push(cum[cum.length - 1]);
  const segs = st.length - 1;
  out.station = a => {
    const seg = Math.max(0, Math.min(a.seg, segs - 1)), s = st[seg] + a.t * (st[seg + 1] - st[seg]);
    return closed ? s % total : Math.min(s, total);
  };
  const ranges = [];
  (p.grades || []).forEach(g => {
    const sa = out.station(g.a), sb = out.station(g.b);
    const ok = closed ? Math.abs(sb - sa) > 1e-3 : sb - sa > 1e-3;
    const L = !ok ? 0 : sb > sa ? sb - sa : total - sa + sb;
    out.sections.push({ src: g, sa, sb, L, ok, rise: L * (g.g0 + g.g1) / 200 });
    if (ok) ranges.push(...gradeRanges(sa, sb, g.g0, g.g1, total));
  });
  out.sections.sort((a, b) => a.sa - b.sa);
  out.profile = makeProfile(ranges, total, closed, z0);
  return out;
}
function computePlan() {
  const pc = piece(), tol = Math.max(0, +$("tol").value || 0), slots = Math.max(1, Math.min(651, +$("slots").value || 651));
  const paths = wuPaths();
  if (!paths.length) return null;
  const pl = { ...plan(paths, pc.len, pc.wid, tol, Math.max(1, slots - S.jumps.length), $("fit").checked), paths, pc, slots, tol };
  pl.poses.push(...jumpPoses(paths, pc));
  pl.zMin = Infinity; pl.zMax = -Infinity; pl.steepest = 0;
  for (const q of pl.poses.filter(q => !q.kind)) { pl.zMin = Math.min(pl.zMin, q.z); pl.zMax = Math.max(pl.zMax, q.z); pl.steepest = Math.max(pl.steepest, Math.abs(Math.tan(q.pitch)) * 100); }
  pl.lip = maxLip(pl.poses, pc.len);
  pl.elevated = paths.some(w => w.profile.active);
  pl.baseOff = paths.some(w => w.border) && !paths.some(w => w.noRoad);
  return pl;
}
// The closest spot on any centerline to a point (world units).
function nearestOnPaths(paths, c) {
  let best = null;
  for (const w of paths) {
    if (!w.arc) continue;
    const edges = w.closed ? w.pts.length : w.pts.length - 1;
    for (let i = 0; i < edges; i++) {
      const a = w.pts[i], ab = V.sub(w.pts[(i + 1) % w.pts.length], a), l2 = V.dot(ab, ab);
      const u = l2 < 1e-12 ? 0 : Math.min(Math.max(V.dot(V.sub(c, a), ab) / l2, 0), 1), d = V.len(V.sub(V.add(a, V.mul(ab, u)), c));
      if (!best || d < best.d) best = { w, d, s: w.cum[i] + u * Math.sqrt(l2) };
    }
  }
  return best;
}
// Each jump marker stands on the track surface of the nearest path, with JUMPS[size] wu showing.
function jumpPoses(paths, pc) {
  return S.jumps.map((j, i) => {
    const c = toWu(j.pos), near = nearestOnPaths(paths, c), surface = (near ? near.w.profile.z(near.s) : 0) + pc.hgt / 2;
    return { pos: c, yaw: j.rot * RAD, z: surface + JUMPS[j.size] - kit().jump.hgt / 2, pitch: 0, s: 0, pc: kit().jump, kind: "jump", path: -1, jump: i };
  });
}
const activeWu = () => planCache && planCache.paths.find(w => w.src === S.paths[S.active]);
function resize() {
  const r = board.getBoundingClientRect(), dpr = window.devicePixelRatio || 1;
  cv.width = Math.max(1, Math.round(r.width * dpr)); cv.height = Math.max(1, Math.round(r.height * dpr));
  draw();
}
function fit() {
  const r = board.getBoundingClientRect();
  const z = Math.min(r.width / S.imgW, r.height / S.imgH) * 0.94;
  S.view = { z, x: (r.width - S.imgW * z) / 2, y: (r.height - S.imgH * z) / 2 };
  draw();
}
const scr = p => [p[0] * S.view.z + S.view.x, p[1] * S.view.z + S.view.y];
const img = q => [(q[0] - S.view.x) / S.view.z, (q[1] - S.view.y) / S.view.z];
function css(name) { return getComputedStyle(document.documentElement).getPropertyValue(name).trim(); }
function draw() {
  cancelAnimationFrame(raf);
  raf = requestAnimationFrame(() => {
    const dpr = window.devicePixelRatio || 1;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    const r = board.getBoundingClientRect();
    ctx.clearRect(0, 0, r.width, r.height);
    ctx.save();
    ctx.translate(S.view.x, S.view.y); ctx.scale(S.view.z, S.view.z);
    ctx.save(); ctx.translate(S.imgW / 2, S.imgH / 2); ctx.rotate(-S.imgRot * RAD);   // the image turns about its centre
    if (S.img && S.img.complete) ctx.drawImage(S.img, -S.imgW / 2, -S.imgH / 2, S.imgW, S.imgH);
    else { ctx.fillStyle = "#888"; ctx.fillRect(-S.imgW / 2, -S.imgH / 2, S.imgW, S.imgH); }
    ctx.restore();
    const lw = 1 / S.view.z;
    // block footprints
    if (planCache && $("showBlocks").checked) {
      const pc = planCache.pc, pieceCol = css("--piece"), accent = css("--accent"), borderCol = css("--ink");
      const zSpan = planCache.zMax - planCache.zMin;
      // on a track with height, one hue gets stronger the higher the block sits
      const heightFill = z => accent + Math.round((0.15 + 0.65 * (z - planCache.zMin) / zSpan) * 255).toString(16).padStart(2, "0");
      ctx.lineWidth = 1.2 * lw;
      const isBase = q => q.kind === "base";   // the baseplate is drawn first, under everything
      for (const q of planCache.poses.filter(isBase).concat(planCache.poses.filter(q => !isBase(q)))) {
        const c = Math.cos(q.yaw), s = Math.sin(q.yaw), qp = q.pc || pc, hl = qp.len / 2 * Math.cos(q.pitch), hw = qp.wid / 2;
        const corners = [[-hl, -hw], [hl, -hw], [hl, hw], [-hl, hw]].map(([a, b]) => toPx([q.pos[0] + a * c - b * s, q.pos[1] + a * s + b * c]));
        ctx.beginPath(); corners.forEach((p, i) => i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1])); ctx.closePath();
        ctx.fillStyle = q.kind === "border" ? borderCol + "88" : q.kind ? pieceCol + "1c" : zSpan > 1e-3 ? heightFill(q.z) : pieceCol + "33"; ctx.fill();
        ctx.strokeStyle = q.kind === "border" ? borderCol : pieceCol; ctx.stroke();
      }
    }
    // centerlines, section ends and the next section under the cursor
    const accentCol = css("--accent");
    ctx.lineCap = "round"; ctx.lineJoin = "round";
    S.paths.forEach((p, k) => {
      const col = COLORS[k % COLORS.length], active = k === S.active;
      if (p.guide && p.guide.length >= 2) {   // the old point-traced line, to trace over
        ctx.globalAlpha = 0.6; ctx.setLineDash([5 * lw, 5 * lw]); ctx.strokeStyle = col; ctx.lineWidth = 1.5 * lw; strokeLine(p.guide); ctx.setLineDash([]); ctx.globalAlpha = 1;
      }
      const g = pathGeom(p, 2 * lw); if (!g) return;
      ctx.strokeStyle = col; ctx.lineWidth = (active ? 3 : 2) * lw; strokeLine(g.line);
      const end = g.poses[g.poses.length - 1];
      if (p.closed && p.secs.length && V.len(V.sub(end.pt, p.start)) > 0.5) {   // a loop not yet closed: the straight join
        ctx.setLineDash([4 * lw, 4 * lw]); strokeLine([end.pt, p.start]); ctx.setLineDash([]);
      }
      if (active) {   // section labels beside the line
        ctx.font = `600 ${10.5 * lw}px ${css("--font-mono")}`; ctx.textAlign = "center"; ctx.textBaseline = "middle";
        p.secs.forEach((s, i) => {
          const pts = secPoints(g.poses[i], s, 2 * lw), m = pts[pts.length >> 1], h = g.poses[i].head + (s.k === "t" ? s.ang / 2 : 0);
          const at = V.add(m, V.mul(leftPx(h), (s.k === "t" ? -Math.sign(s.ang) : 1) * 14 * lw));
          const txt = s.k === "s" ? `${fmt(s.len * wpp(), 2)} wu · ${showDeg(g.poses[i].head)}°` : s.k === "c" ? `curve ${showDeg(curveStartHead(g.poses[i], s))}° → ${showDeg(secEnd(g.poses[i], s).head)}°`
            : `${fmt(Math.abs(s.ang), 1)}° ${s.ang > 0 ? "L" : "R"} · r ${fmt(s.r * wpp(), 2)}`;
          const tw = ctx.measureText(txt).width + 8 * lw;
          ctx.fillStyle = css("--panel") + "dd"; ctx.fillRect(at[0] - tw / 2, at[1] - 7 * lw, tw, 14 * lw);
          ctx.fillStyle = col; ctx.fillText(txt, at[0], at[1]);
        });
      }
      if (active && S.mode === "trace") p.secs.forEach((s, i) => {   // diamonds: drag to bend a section
        const [x, y] = secMid(g.poses[i], s), hov = S.hover && S.hover.mid && S.hover.path === k && S.hover.i === i, r = (hov ? 7.5 : 6) * lw, turn = s.k === "t";
        ctx.beginPath(); ctx.moveTo(x, y - r); ctx.lineTo(x + r, y); ctx.lineTo(x, y + r); ctx.lineTo(x - r, y); ctx.closePath();
        ctx.fillStyle = s.k === "c" ? col : turn ? accentCol : "#ffffff"; ctx.fill(); ctx.lineWidth = 1.5 * lw; ctx.strokeStyle = turn || s.k === "c" ? "#ffffff" : accentCol; ctx.stroke();
      });
      if (active) p.secs.forEach((s, i) => {   // a kink where a section leaves the heading it arrives with
        if (!i || !s.kink || Math.abs(s.kink) < 0.05) return;
        const q = g.poses[i].pt; ctx.beginPath(); ctx.arc(q[0], q[1], 9 * lw, 0, Math.PI * 2); ctx.strokeStyle = css("--warn"); ctx.lineWidth = 2 * lw; ctx.setLineDash([3 * lw, 3 * lw]); ctx.stroke(); ctx.setLineDash([]);
      });
      g.poses.forEach((q, j) => {
        const i = j - 1, sel = S.sel && S.sel.path === k && S.sel.i === i, hov = S.hover && !S.hover.mid && S.hover.path === k && S.hover.i === i;
        if (!active && i >= 0 && !hov) return;
        ctx.beginPath(); ctx.arc(q.pt[0], q.pt[1], (sel ? 7 : hov ? 6 : i < 0 ? 5 : 4) * lw, 0, Math.PI * 2);
        ctx.fillStyle = sel ? accentCol : i < 0 ? col : "#ffffff"; ctx.fill();
        ctx.lineWidth = 2 * lw; ctx.strokeStyle = i < 0 ? "#ffffff" : col; ctx.stroke();
      });
      if (active && (p.secs.length || p.head != null)) {   // which way the track carries on
        const d = dirPx(end.head), n = leftPx(end.head), tip = V.add(end.pt, V.mul(d, 16 * lw)), c = 6 * lw;
        ctx.strokeStyle = col; ctx.lineWidth = 2 * lw;
        strokeLine([V.add(V.sub(tip, V.mul(d, c)), V.mul(n, c)), tip, V.sub(V.sub(tip, V.mul(d, c)), V.mul(n, c))]);
      }
      if (active && S.ghost && S.mode === "trace" && !S.drag) {
        const pose = { pt: end.pt, head: S.ghost.head !== undefined ? S.ghost.head : end.head + (S.ghost.sec.kink || 0) };
        const gl = secPoints(pose, S.ghost.sec, 2 * lw); gl.push(secEnd(pose, S.ghost.sec).pt);
        ctx.setLineDash([6 * lw, 5 * lw]); ctx.strokeStyle = accentCol; ctx.lineWidth = 2.5 * lw; strokeLine(gl); ctx.setLineDash([]);
      }
    });
    drawSections(lw); drawJumps(lw);
    // calibration line
    if (S.cal.length) {
      ctx.strokeStyle = css("--accent"); ctx.lineWidth = 2 * lw; ctx.setLineDash([6 * lw, 4 * lw]);
      ctx.beginPath(); ctx.moveTo(S.cal[0][0], S.cal[0][1]); ctx.lineTo((S.cal[1] || S.calHover || S.cal[0])[0], (S.cal[1] || S.calHover || S.cal[0])[1]); ctx.stroke(); ctx.setLineDash([]);
      for (const c of S.cal) { ctx.beginPath(); ctx.arc(c[0], c[1], 5 * lw, 0, Math.PI * 2); ctx.fillStyle = css("--accent"); ctx.fill(); }
    }
    ctx.restore();
    if (planCache || !S.drag) drawProfile();
    readout();
  });
}
const gradeNum = (x, d = 2) => String(+(+x).toFixed(d));
const signedNum = (x, d = 2) => { const r = +(+x).toFixed(d); return (r > 0 ? "+" : "") + r; };
const gradeText = g => g.curve ? `${signedNum(g.g0)} to ${signedNum(g.g1)}%` : `${signedNum(g.g0)}%`;
function sectionLine(w, sa, L) {
  const n = Math.max(2, Math.ceil(L));
  return Array.from({ length: n + 1 }, (_, i) => toPx(w.arc.at(sa + L * i / n)));
}
function strokeLine(line) { ctx.beginPath(); line.forEach((p, i) => i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1])); ctx.stroke(); }
function dot(p, r, fill, ring, lw) { ctx.beginPath(); ctx.arc(p[0], p[1], r * lw, 0, Math.PI * 2); ctx.fillStyle = fill; ctx.fill(); ctx.lineWidth = 2 * lw; ctx.strokeStyle = ring; ctx.stroke(); }
function drawSections(lw) {
  if (!planCache) return;
  const accent = css("--accent"), panel = css("--panel"), ink = css("--ink");
  ctx.lineCap = "round"; ctx.lineJoin = "round"; ctx.textAlign = "center"; ctx.textBaseline = "middle";
  ctx.font = `600 ${11 * lw}px ${css("--font-mono")}`;
  for (const w of planCache.paths) {
    if (!w.arc) continue;
    const isActive = w.src === S.paths[S.active];
    w.sections.forEach((sec, n) => {
      if (!sec.ok) return;
      const line = sectionLine(w, sec.sa, sec.L), m = line.length >> 1;
      ctx.globalAlpha = isActive ? 0.95 : 0.5;
      ctx.strokeStyle = accent; ctx.lineWidth = 5 * lw; strokeLine(line);
      ctx.globalAlpha = 1;
      dot(line[0], 4, panel, accent, lw); dot(line[line.length - 1], 4, panel, accent, lw);
      // chevron pointing in the direction of travel, then the number and grade beside the line
      const a = line[m - 1], b = line[m + 1], ang = Math.atan2(b[1] - a[1], b[0] - a[0]), mid = line[m], c = 7 * lw;
      ctx.strokeStyle = panel; ctx.lineWidth = 2 * lw;
      strokeLine([[mid[0] - c * Math.cos(ang - 0.6), mid[1] - c * Math.sin(ang - 0.6)], mid, [mid[0] - c * Math.cos(ang + 0.6), mid[1] - c * Math.sin(ang + 0.6)]]);
      const text = `${n + 1}  ${gradeText(sec.src)}`, tw = ctx.measureText(text).width + 10 * lw, th = 17 * lw;
      const lx = mid[0] - Math.sin(ang) * 22 * lw, ly = mid[1] + Math.cos(ang) * 22 * lw;
      ctx.beginPath(); ctx.roundRect(lx - tw / 2, ly - th / 2, tw, th, 3 * lw); ctx.fillStyle = panel; ctx.fill();
      ctx.lineWidth = lw; ctx.strokeStyle = accent; ctx.stroke();
      ctx.fillStyle = ink; ctx.fillText(text, lx, ly);
    });
  }
  // the section being picked
  if (S.gradePick.length) {
    const w = planCache.paths.find(x => x.src === S.paths[S.gradePick[0].path]);
    if (w && w.arc) {
      const [a, b] = S.gradePick;
      if (b) {
        const sa = w.station(a), sb = w.station(b);
        ctx.setLineDash([8 * lw, 6 * lw]); ctx.strokeStyle = accent; ctx.lineWidth = 4 * lw;
        strokeLine(sectionLine(w, sa, sb > sa ? sb - sa : w.total - sa + sb)); ctx.setLineDash([]);
      }
      for (const q of S.gradePick) dot(toPx(q.wu), 6, accent, panel, lw);
    }
  }
  if ((S.mode === "grade" || S.mode === "width") && S.gradeHover) dot(toPx(S.gradeHover), 5, panel, accent, lw);
  // width points: a bar across the track at each one, with its width
  for (const w of planCache.paths) {
    if (!w.arc || !w.wstopsR.length) continue;
    const isActive = w.src === S.paths[S.active];
    ctx.globalAlpha = isActive ? 1 : 0.5; ctx.font = `600 ${10.5 * lw}px ${css("--font-mono")}`;
    for (const st of w.wstopsR) {
      const c = w.arc.at(st.s), n = perp(w.arc.tangent(st.s, 1)), a = toPx(V.add(c, V.mul(n, st.w / 2))), b = toPx(V.sub(c, V.mul(n, st.w / 2)));
      ctx.strokeStyle = accent; ctx.lineWidth = 2.5 * lw; strokeLine([a, b]);
      dot(a, 3.5, panel, accent, lw); dot(b, 3.5, panel, accent, lw);
      const m = toPx(c), txt = `w ${fmt(st.w, 2)}`, tw = ctx.measureText(txt).width + 8 * lw;
      ctx.fillStyle = panel; ctx.fillRect(m[0] - tw / 2, m[1] - 7 * lw, tw, 14 * lw); ctx.fillStyle = accent; ctx.fillText(txt, m[0], m[1]);
    }
    ctx.globalAlpha = 1;
  }
  if (S.profHover !== null) {
    const w = activeWu();
    if (w && w.arc) dot(toPx(w.arc.at(Math.min(S.profHover, w.total))), 7, accent, panel, lw);
  }
}
// The turn handle of a jump marker sits 30 screen px out along the way the marker faces.
const jumpHandle = j => { const r = 30 / S.view.z, a = j.rot * Math.PI / 180; return [j.pos[0] + r * Math.cos(a), j.pos[1] - r * Math.sin(a)]; };
function drawJumps(lw) {
  const accent = css("--accent"), panel = css("--panel"), ink = css("--ink"), half = 2 / wpp();
  ctx.font = `600 ${11 * lw}px ${css("--font-mono")}`; ctx.textAlign = "center"; ctx.textBaseline = "middle"; ctx.lineCap = "butt";
  S.jumps.forEach((j, i) => {
    const a = j.rot * Math.PI / 180, ax = -Math.sin(a), ay = -Math.cos(a), sel = S.mode === "jump" && S.jumpSel === i;
    ctx.strokeStyle = accent; ctx.lineWidth = 3 * lw;   // the block, across the lane
    strokeLine([[j.pos[0] - ax * half, j.pos[1] - ay * half], [j.pos[0] + ax * half, j.pos[1] + ay * half]]);
    if (sel) {
      const h = jumpHandle(j);
      ctx.lineWidth = 1.5 * lw; ctx.beginPath(); ctx.moveTo(j.pos[0], j.pos[1]); ctx.lineTo(h[0], h[1]); ctx.stroke();
      dot(h, 5, panel, accent, lw);
    }
    dot(j.pos, sel ? 6 : 4, sel ? accent : panel, accent, lw);
    const tx = j.pos[0] - Math.cos(a) * 15 * lw, ty = j.pos[1] + Math.sin(a) * 15 * lw;   // size letter, behind the block
    ctx.beginPath(); ctx.arc(tx, ty, 8 * lw, 0, Math.PI * 2); ctx.fillStyle = panel; ctx.fill();
    ctx.fillStyle = ink; ctx.fillText(i + 1, tx, ty);   // matches its row in the list
  });
}
const PROF_PAD = { l: 40, r: 12, t: 16, b: 20 };
function niceStep(span, count) { const raw = span / count, m = 10 ** Math.floor(Math.log10(raw)), f = raw / m; return (f <= 1 ? 1 : f <= 2 ? 2 : f <= 5 ? 5 : 10) * m; }
function drawProfile() {
  const r = pv.getBoundingClientRect(), dpr = window.devicePixelRatio || 1, W = r.width, H = r.height, g = pctx;
  const pw = Math.max(1, Math.round(W * dpr)), ph = Math.max(1, Math.round(H * dpr));
  if (pv.width !== pw || pv.height !== ph) { pv.width = pw; pv.height = ph; }
  g.setTransform(dpr, 0, 0, dpr, 0, 0); g.clearRect(0, 0, W, H);
  const w = activeWu(), muted = css("--muted"), ink = css("--ink"), panel = css("--panel"), mono = css("--font-mono");
  $("profName").textContent = S.paths[S.active] ? `· ${S.paths[S.active].name}` : "";
  g.font = `10px ${mono}`;
  if (!w || !w.arc) {
    g.fillStyle = muted; g.textAlign = "center"; g.textBaseline = "middle"; g.fillText("Trace a path to see its height", W / 2, H / 2);
    $("profRead").textContent = ""; return;
  }
  const prof = w.profile, total = w.total, x0 = PROF_PAD.l, x1 = W - PROF_PAD.r, y0 = PROF_PAD.t, y1 = H - PROF_PAD.b;
  const n = Math.max(2, Math.ceil(x1 - x0)), zs = Array.from({ length: n + 1 }, (_, i) => prof.z(total * i / n));
  let lo = Math.min(...zs), hi = Math.max(...zs);
  if (hi - lo < 2) { const mid = (hi + lo) / 2; lo = mid - 1; hi = mid + 1; }
  const yStep = niceStep(hi - lo, 3), xStep = niceStep(total, 6);
  lo = Math.floor(lo / yStep) * yStep; hi = Math.ceil(hi / yStep) * yStep;
  const X = s => x0 + (x1 - x0) * s / total, Y = z => y1 - (y1 - y0) * (z - lo) / (hi - lo);
  // recessive grid and axis labels
  g.lineWidth = 1; g.strokeStyle = css("--line"); g.fillStyle = muted; g.textAlign = "right"; g.textBaseline = "middle";
  for (let z = lo; z <= hi + yStep / 2; z += yStep) {
    const y = Math.round(Y(z)) + 0.5;
    g.beginPath(); g.moveTo(x0, y); g.lineTo(x1, y); g.stroke();
    g.fillText(gradeNum(z), x0 - 6, y);
  }
  g.textAlign = "center"; g.textBaseline = "top";
  for (let s = 0; s <= total; s += xStep) g.fillText(s ? gradeNum(s) : "0 wu", Math.min(X(s), x1 - 12), y1 + 6);
  // sections behind the line
  w.sections.forEach((sec, i) => {
    if (!sec.ok) return;
    g.fillStyle = css("--accent") + "22";
    const parts = gradeRanges(sec.sa, sec.sb, 0, 0, total);
    for (const q of parts) g.fillRect(X(q.a), y0 - 14, X(q.b) - X(q.a), y1 - y0 + 14);
    const text = `${i + 1}  ${gradeText(sec.src)}`, widest = parts.reduce((m, q) => X(q.b) - X(q.a) > X(m.b) - X(m.a) ? q : m);
    if (g.measureText(text).width + 6 < X(widest.b) - X(widest.a)) {
      g.fillStyle = ink; g.textAlign = "center"; g.textBaseline = "top"; g.fillText(text, (X(widest.a) + X(widest.b)) / 2, 2);
    }
  });
  // height line with a soft fill under it
  const track = css("--track");
  g.beginPath(); zs.forEach((z, i) => { const x = X(total * i / n), y = Y(z); i ? g.lineTo(x, y) : g.moveTo(x, y); });
  g.strokeStyle = track; g.lineWidth = 2; g.lineJoin = "round"; g.stroke();
  g.lineTo(x1, y1); g.lineTo(x0, y1); g.closePath(); g.fillStyle = track + "22"; g.fill();
  if (S.profHover !== null) {
    const s = Math.min(S.profHover, total), z = prof.z(s), x = X(s), y = Y(z);
    g.strokeStyle = muted; g.lineWidth = 1; g.beginPath(); g.moveTo(Math.round(x) + 0.5, y0); g.lineTo(Math.round(x) + 0.5, y1); g.stroke();
    g.beginPath(); g.arc(x, y, 4, 0, Math.PI * 2); g.fillStyle = track; g.fill(); g.strokeStyle = panel; g.lineWidth = 2; g.stroke();
    $("profRead").textContent = `${gradeNum(s, 1)} wu along · height ${gradeNum(z)} wu · grade ${signedNum(prof.gradeAt(s))}%`;
  } else {
    $("profRead").textContent = "Height in wu, vertical scale exaggerated";
  }
  pv.setAttribute("aria-label", `Height along ${w.src.name}: from ${gradeNum(Math.min(...zs))} to ${gradeNum(Math.max(...zs))} wu over ${gradeNum(total, 0)} wu of track`);
}

// ============================================================================================
// Panel
// ============================================================================================
function fmt(n, d = 2) { return (Math.round(n * 10 ** d) / 10 ** d).toFixed(d).replace(/(\.\d*?)0+$/, "$1").replace(/\.$/, "") || "0"; }
const clampGrade = v => Math.min(Math.max(v, -100), 100);
function gradeInput(value, label, onSet) {
  const i = document.createElement("input");
  i.type = "number"; i.step = "0.5"; i.min = "-100"; i.max = "100"; i.value = gradeNum(value, 3); i.setAttribute("aria-label", label);
  i.addEventListener("change", () => {
    if (i.value.trim() === "" || !isFinite(+i.value)) { i.value = gradeNum(value, 3); return; }
    pushUndo(); onSet(clampGrade(+i.value)); changed();
  });
  return i;
}
function renderGrades() {
  const el = $("gradeList"), p = S.paths[S.active], w = activeWu();
  el.textContent = "";
  $("z0").disabled = !p;
  if (document.activeElement !== $("z0")) $("z0").value = p ? gradeNum(p.z0 || 0, 3) : "0";
  if (!p || !w) return;
  if (!w.sections.length) {
    const note = document.createElement("p"); note.className = "sub"; note.textContent = `“${p.name}” is flat. Use the Grade tool to add a slope.`;
    el.append(note); return;
  }
  w.sections.forEach((sec, n) => {
    const g = sec.src, row = document.createElement("div"); row.className = "sect";
    const tag = document.createElement("b"); tag.textContent = n + 1;
    const shape = document.createElement("select"); shape.setAttribute("aria-label", `Section ${n + 1} shape`);
    [["const", "Constant"], ["curve", "Gradual"]].forEach(([v, t]) => { const o = document.createElement("option"); o.value = v; o.textContent = t; shape.append(o); });
    shape.value = g.curve ? "curve" : "const";
    shape.addEventListener("change", () => { pushUndo(); g.curve = shape.value === "curve"; if (!g.curve) g.g1 = g.g0; changed(); });
    row.append(tag, shape);
    if (g.curve) {
      const to = document.createElement("span"); to.className = "mono"; to.textContent = "to";
      row.append(gradeInput(g.g0, `Section ${n + 1} start grade percent`, v => { g.g0 = v; }), to, gradeInput(g.g1, `Section ${n + 1} end grade percent`, v => { g.g1 = v; }));
    } else {
      row.append(gradeInput(g.g0, `Section ${n + 1} grade percent`, v => { g.g0 = v; g.g1 = v; }));
    }
    const pct = document.createElement("span"); pct.className = "mono"; pct.textContent = "%";
    const acts = document.createElement("div"); acts.className = "acts";
    if (w.closed && sec.ok && Math.abs(w.profile.closure) > 0.005) {
      const close = document.createElement("button"); close.textContent = "Close loop"; close.title = "Change this section's grade so the loop ends at its start height";
      close.addEventListener("click", () => { const d = -w.profile.closure / sec.L * 100; pushUndo(); g.g0 = clampGrade(g.g0 + d); g.g1 = clampGrade(g.g1 + d); changed(); });
      acts.append(close);
    }
    const del = document.createElement("button"); del.textContent = "✕"; del.title = "Delete section"; del.setAttribute("aria-label", `Delete section ${n + 1}`);
    del.addEventListener("click", () => { pushUndo(); p.grades.splice(p.grades.indexOf(g), 1); changed(); });
    acts.append(del);
    const meta = document.createElement("div"); meta.className = "meta";
    meta.textContent = sec.ok ? `${gradeNum(sec.L, 1)} wu long · ${signedNum(sec.rise)} wu` : "Inactive: it runs past the end of this open path";
    row.append(pct, acts, meta);
    el.append(row);
  });
}
const el = (tag, props = {}, ...kids) => { const e = Object.assign(document.createElement(tag), props); e.append(...kids); return e; };
const showDeg = h => fmt((((+h % 360) + 360) % 360) % 360, 2);   // 0 to 360, counter-clockwise from image right
function numIn(value, step, label, onSet, min) {
  const i = el("input", { type: "number", step: String(step), value: gradeNum(value, 3) });
  i.setAttribute("aria-label", label);
  if (min !== undefined) i.min = String(min);
  i.addEventListener("change", () => {
    if (i.value.trim() === "" || !isFinite(+i.value) || (min !== undefined && +i.value < min)) { i.value = gradeNum(value, 3); return; }
    pushUndo(); onSet(+i.value); changed();
  });
  return i;
}
const prop = (label, ...kids) => el("div", { className: "prop" }, el("span", { textContent: label }), ...kids);
const pair = (...kids) => el("div", { className: "pair" }, ...kids);
const unit = t => el("i", { textContent: t });
function check(label, on, onSet, title) {
  const cb = el("input", { type: "checkbox", checked: !!on });
  cb.addEventListener("change", () => { pushUndo(); onSet(cb.checked); changed(); });
  return el("label", { className: "chk", title: title || "" }, cb, " " + label);
}
// Points section i of p so it ends heading H (degrees, 0 = image right, counter-clockwise). A
// curve turns its last handle about its end point; a turn changes its angle; a straight turns
// about its start. The next section then carries straight on from H (its corner is removed).
function setEndHeading(p, i, H) {
  const ps = posesOf(p), s = p.secs[i], d = wrap180(H - secEnd(ps[i], s).head);
  if (Math.abs(d) < 1e-9) { if (p.secs[i + 1]) delete p.secs[i + 1].kink; return; }
  if (s.k === "c") {
    const A = cubicAbs(ps[i], s), reach = V.len(V.sub(A.b, A.c2)) > 1e-6 ? V.len(V.sub(A.b, A.c2)) : V.len(V.sub(A.b, A.a)) / 3;
    const c = curveFrom(ps[i], { a: A.a, c1: A.c1, c2: V.sub(A.b, V.mul(dirPx(H), reach)), b: A.b });
    if (s.kink) c.kink = s.kink;
    p.secs[i] = c;
  } else if (s.k === "t") {
    const ang = s.ang + d;
    if (Math.abs(ang) < 0.01) { p.secs[i] = { k: "s", len: V.len(V.sub(secEnd(ps[i], s).pt, ps[i].pt)), ...(s.kink ? { kink: s.kink } : {}) }; }
    else s.ang = Math.max(-355, Math.min(355, ang));
  } else if (i === 0) p.head = H;
  else { const k = wrap180((s.kink || 0) + d); if (Math.abs(k) > 1e-9) s.kink = k; else delete s.kink; }
  if (p.secs[i + 1]) delete p.secs[i + 1].kink;
}
function headIn(p, i, poses) {
  const s = p.secs[i], cur = showDeg(secEnd(poses[i], s).head);
  const inp = el("input", { type: "number", className: "hd", step: snapOn() ? "5" : "1", value: cur, title: "The heading this section ends at, counter-clockwise from image right (0 = right, 90 = up, 180 = left, 270 = down). Type an exact value; the next section carries on from it." });
  inp.setAttribute("aria-label", `Section ${i + 1} end heading in degrees`);
  inp.addEventListener("change", () => {
    if (inp.value.trim() === "" || !isFinite(+inp.value)) { inp.value = cur; return; }
    pushUndo(); setEndHeading(p, i, +inp.value); changed();
  });
  return inp;
}
// The Active path panel: the path's own settings, then its sections in order.
// A number box that may be left empty (undefined = use the default).
function optNum(value, placeholder, label, onSet) {
  const i = el("input", { type: "number", step: "0.5", min: "0.5", max: "200", placeholder, value: value > 0 ? gradeNum(value, 3) : "" });
  i.setAttribute("aria-label", label);
  i.addEventListener("change", () => {
    const v = i.value.trim() === "" ? undefined : +i.value;
    if (v !== undefined && !(v >= 0.5)) { i.value = value > 0 ? gradeNum(value, 3) : ""; return; }
    pushUndo(); onSet(v === undefined ? undefined : Math.min(v, 200)); changed();
  });
  return i;
}
function renderSections(wp) {
  const box = $("secList"), p = S.paths[S.active]; box.textContent = "";
  const w = p && (wp || wuPaths()).find(x => x.src === p);
  if (!p) { $("secHead").textContent = "Active path"; box.append(el("p", { className: "sub", textContent: "Add a path in the Paths panel." })); return; }
  $("secHead").textContent = `Active path · ${p.name}`;
  const k = wpp(), props = el("div", { className: "pathprops" });
  props.append(prop("Options", pair(check("Loop", p.closed, v => { p.closed = v; }, "A closed lap: the end joins back to the start"),
    check("Border", p.border, v => { p.border = v; }, "Border pieces outside both edges (style in Blocks)"))));
  const dflt = fmt(Math.max(+$("trackW").value || 0, piece().wid), 2);
  if (p.taper) props.append(prop("Width", pair(
    numIn(p.taper.w0, 1, "Start width in wu", v => { p.taper.w0 = Math.min(v, 200); }, 0.5), unit("to"),
    numIn(p.taper.w1, 1, "End width in wu", v => { p.taper.w1 = Math.min(v, 200); }, 0.5), unit("wu"))));
  else if (p.closed) props.append(prop("Width", pair(optNum(p.width, dflt, "Track width in wu", v => { p.width = v; }), unit("wu"))));
  else props.append(prop("Width", pair(optNum(p.width, dflt, "Start width in wu", v => { p.width = v; }), unit("to"),
    optNum(p.width1, "same", "End width in wu", v => { p.width1 = v; }), unit("wu"))));
  if (w && w.wstopsR.length) {   // width points, in order along the path
    const list = el("div", { className: "seclist" });
    w.wstopsR.forEach((st, n) => {
      const del = el("button", { textContent: "✕", title: "Delete this width point" });
      del.setAttribute("aria-label", `Delete width point ${n + 1}`);
      del.addEventListener("click", () => { pushUndo(); p.wstops.splice(p.wstops.indexOf(st.src), 1); changed(); });
      list.append(el("div", { className: "sect" }, el("b", { textContent: n + 1 }), el("span", { className: "kind t", textContent: "Width" }),
        numIn(st.src.w, 0.5, `Width point ${n + 1} in wu`, v => { st.src.w = Math.min(v, 200); }, 0.5), unit("wu"),
        el("i", { textContent: `at ${gradeNum(st.s, 1)} wu` }), el("div", { className: "acts" }, del)));
    });
    props.append(prop("Width points", list));
  }
  const addW = el("button", { textContent: "+ Width point", title: "Pick the Width tool, then click the path where its width should change" });
  addW.addEventListener("click", () => setMode("width"));
  props.append(prop("", addW));
  if (p.start) props.append(prop("Start heading", pair(numIn(p.head == null ? 0 : p.head, snapOn() ? 5 : 1, "Start heading in degrees", v => { p.head = snapHead(v); }), unit("°"),
    p.head == null ? el("i", { textContent: "set by first straight" }) : "")));
  if (p.guide) {
    const b = el("button", { textContent: "Remove", title: "Remove the dashed line left from the old point tracing" });
    b.addEventListener("click", () => { pushUndo(); delete p.guide; changed(); });
    props.append(prop("Old trace", pair(b)));
  }
  box.append(props);
  const list = el("div", { className: "seclist" }); box.append(list);
  if (!p.start) { list.append(el("p", { className: "sub", textContent: "Click the board where this path starts." })); return; }
  if (!p.secs.length) { list.append(el("p", { className: "sub", textContent: "Click ahead of the start to add a straight." })); return; }
  const poses = posesOf(p);
  p.secs.forEach((s, i) => {
    const row = el("div", { className: "sect" + (S.sel && S.sel.path === S.active && S.sel.i === i ? " on" : "") }, el("b", { textContent: i + 1 }));
    if (s.k === "s") row.append(el("span", { className: "kind s", textContent: "Straight" }), numIn(s.len * k, 0.25, `Section ${i + 1} length in wu`, v => { s.len = v / k; }, 0.25), unit("wu"),
      unit("at"), headIn(p, i, poses), unit("°"));
    else if (s.k === "c") {
      const straighten = el("button", { textContent: "Make arc", title: "Turn this curve back into a straight or a circular turn from its start to its middle; the sections after it follow" });
      straighten.addEventListener("click", () => { pushUndo(); bendSection(p, i, secMid(poses[i], s)); if (p.secs[i].k === "c") p.secs[i] = { k: "s", len: V.len(V.sub(secEnd(poses[i], s).pt, poses[i].pt)), ...(s.kink ? { kink: s.kink } : {}) }; changed(); });
      row.append(el("span", { className: "kind t", textContent: "Curve", title: "A free Bezier curve. Shift+drag its diamond or end dot to reshape it." }),
        el("i", { textContent: `${fmt(V.len(V.sub(secEnd(poses[i], s).pt, poses[i].pt)) * k, 2)} wu across` }), straighten);
    } else {
      const side = el("button", { className: "side", textContent: s.ang > 0 ? "L" : "R", title: "Turns left (L) or right (R). Click to flip." });
      side.setAttribute("aria-label", `Section ${i + 1} turns ${s.ang > 0 ? "left" : "right"}; click to flip`);
      side.addEventListener("click", () => { pushUndo(); s.ang = -s.ang; changed(); });
      row.append(el("span", { className: "kind t", textContent: s.free ? "Free" : "Turn", title: s.free ? "Free turn: dragging its end dot changes its angle and radius" : "Turn: dragging its end dot changes its radius" }),
        numIn(Math.abs(s.ang), snapOn() ? 5 : 1, `Section ${i + 1} angle in degrees`, v => { s.ang = Math.sign(s.ang) * snapAng(v); }, 0.5), unit("°"), side,
        unit("r"), numIn(s.r * k, 0.25, `Section ${i + 1} radius in wu`, v => { s.r = v / k; }, 0.5));
    }
    const del = el("button", { textContent: "✕", title: "Delete this section; the ones after it join the one before" });
    del.setAttribute("aria-label", `Delete section ${i + 1}`);
    del.addEventListener("click", () => { pushUndo(); removeSection(p, i); changed(); });
    row.append(el("div", { className: "acts" }, del));
    if (s.k !== "s") row.append(el("div", { className: "meta endhead" }, `heading ${showDeg(s.k === "c" ? curveStartHead(poses[i], s) : poses[i].head)}° → ends at`, headIn(p, i, poses), "°"));
    if (i > 0 && s.kink && Math.abs(s.kink) >= 0.05) {   // a corner where this section starts
      const smooth = el("button", { className: "txt", textContent: "Smooth", title: "Remove the corner: this section and the ones after it turn to carry on from the one before" });
      smooth.addEventListener("click", () => { pushUndo(); delete s.kink; changed(); });
      row.append(el("div", { className: "meta warn" }, `corner of ${fmt(Math.abs(s.kink), 1)}° at its start `, smooth));
    }
    list.append(row);
  });
  if (p.closed) {
    const end = poses[poses.length - 1], gap = V.len(V.sub(end.pt, p.start)) * k, turn = Math.abs(((end.head - (p.head || 0)) % 360 + 540) % 360 - 180), ok = gap < 0.01 && turn < 0.01;
    box.append(el("p", { className: "msg " + (ok ? "ok" : "bad"), textContent: ok ? "The loop closes exactly."
      : gap < 0.01 ? `The loop closes, with a corner of ${fmt(turn, 1)}° at its start.`
      : `Loop ends ${fmt(gap, 2)} wu from its start${turn >= 0.01 ? `, ${fmt(turn, 2)}° off its heading` : ""}; a straight joins them.` }));
  }
}
const JUMP_NAMES = { whoop: "Whoop", small: "Small", medium: "Medium", large: "Large" };
function fillJumpSelect(sel, value) {
  sel.textContent = "";
  for (const k of Object.keys(JUMPS)) { const o = document.createElement("option"); o.value = k; o.textContent = `${JUMP_NAMES[k]} (${JUMPS[k]} wu)`; sel.append(o); }
  sel.value = JUMPS[value] ? value : "whoop";
}
function renderJumps() {
  const el = $("jumpList"); el.textContent = "";
  if (!S.jumps.length) return;
  S.jumps.forEach((j, i) => {
    const row = document.createElement("div"); row.className = "sect";
    const tag = document.createElement("b"); tag.textContent = i + 1;
    const size = document.createElement("select"); size.setAttribute("aria-label", `Jump ${i + 1} size`);
    fillJumpSelect(size, j.size);
    size.addEventListener("change", () => { pushUndo(); j.size = size.value; changed(); });
    const ang = document.createElement("input"); ang.type = "number"; ang.step = "5"; ang.value = gradeNum(j.rot, 1); ang.setAttribute("aria-label", `Jump ${i + 1} direction in degrees`);
    ang.addEventListener("change", () => { if (ang.value.trim() === "" || !isFinite(+ang.value)) { ang.value = gradeNum(j.rot, 1); return; } pushUndo(); j.rot = ((+ang.value % 360) + 360) % 360; changed(); });
    const deg = document.createElement("span"); deg.className = "mono"; deg.textContent = "°";
    const acts = document.createElement("div"); acts.className = "acts";
    const del = document.createElement("button"); del.textContent = "✕"; del.title = "Delete marker"; del.setAttribute("aria-label", `Delete jump ${i + 1}`);
    del.addEventListener("click", () => { pushUndo(); S.jumps.splice(i, 1); S.jumpSel = null; changed(); });
    row.append(tag, size, ang, deg, acts); acts.append(del); el.append(row);
  });
}
// The Paths panel: one outliner row per path.
function renderPaths() {
  const wp = wuPaths();
  renderGrades(); renderSections(wp);
  const box = $("paths"); box.textContent = "";
  if (!S.paths.length) { box.append(el("div", { className: "empty", textContent: "No paths yet. Add a loop, then click where it starts." })); return; }
  S.paths.forEach((p, k) => {
    const row = el("div", { className: "orow" + (k === S.active ? " active" : ""), tabIndex: 0 });
    const sw = el("span", { className: "sw" }); sw.style.background = COLORS[k % COLORS.length];
    const name = el("input", { type: "text", value: p.name, title: "Rename" });
    name.setAttribute("aria-label", "Path name");
    name.addEventListener("focus", () => { if (S.active !== k) { S.active = k; renderSections(); renderGrades(); draw(); box.querySelectorAll(".orow").forEach((r, j) => r.classList.toggle("active", j === k)); } });
    name.addEventListener("change", () => { pushUndo(); p.name = name.value; changed(); });
    const w = wp.find(x => x.src === p), a = w && Arc.make(w.pts, w.closed);
    const meta = el("span", { className: "meta", textContent: `${p.closed ? "loop" : p.taper ? "start" : "open"} · ${a ? fmt(a.total, 0) : 0} wu`, title: `${p.secs.length} section${p.secs.length === 1 ? "" : "s"}` });
    const del = el("button", { textContent: "✕", title: "Delete path" });
    del.setAttribute("aria-label", "Delete path " + p.name);
    del.addEventListener("click", e => { e.stopPropagation(); pushUndo(); S.paths.splice(k, 1); S.active = Math.max(0, Math.min(S.active, S.paths.length - 1)); S.sel = null; changed(); });
    row.append(sw, name, meta, del);
    row.addEventListener("click", e => { if (e.target.closest("input, button")) return; S.active = k; renderPaths(); draw(); });
    row.addEventListener("keydown", e => { if (e.key === "Enter" && e.target === row) { S.active = k; renderPaths(); draw(); } });
    box.append(row);
  });
}
function originTokens() {
  const v = ["ox", "oy", "oz"].map(id => $(id).value.trim());
  if (v.every(x => x !== "" && isFinite(+x))) return ` at ${v.map(x => fmt(+x, 3)).join(" ")}`;
  return "";
}
// The reference image as a model HMS can import (Tools ▸ Import geometry reads .glb, not images):
// one flat, textured quad in world units, turned like the image on the board, centred on the
// image centre at height 0. glTF is Y-up; HMS maps (x, y, z) to Halo (x, -z, y).
async function referenceGlb() {
  if (!S.img || !S.img.complete || !S.img.naturalWidth) return null;
  const k = Math.min(1, 2048 / Math.max(S.imgW, S.imgH)), c = document.createElement("canvas");
  c.width = Math.max(1, Math.round(S.imgW * k)); c.height = Math.max(1, Math.round(S.imgH * k));
  c.getContext("2d").drawImage(S.img, 0, 0, c.width, c.height);
  const png = new Uint8Array(await (await new Promise(res => c.toBlob(res, "image/png"))).arrayBuffer());
  const a = -S.imgRot * RAD, ca = Math.cos(a), sa = Math.sin(a);
  // corners in image px as drawn on the board (top-left, top-right, bottom-right, bottom-left)
  const corners = [[-1, -1], [1, -1], [1, 1], [-1, 1]].map(([sx, sy]) => {
    const dx = sx * S.imgW / 2, dy = sy * S.imgH / 2;
    return toWu([S.imgW / 2 + dx * ca - dy * sa, S.imgH / 2 + dx * sa + dy * ca]);
  });
  const pos = new Float32Array(corners.flatMap(([x, y]) => [x, 0, -y]));
  const nor = new Float32Array([0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0]);
  const uv = new Float32Array([0, 0, 1, 0, 1, 1, 0, 1]);
  const idx = new Uint16Array([0, 3, 2, 0, 2, 1]);   // counter-clockwise seen from above
  const parts = [pos, nor, uv, idx, png].map(p => new Uint8Array(p.buffer, p.byteOffset, p.byteLength));
  const views = [], pad4 = n => (n + 3) & ~3;
  let off = 0;
  for (const p of parts) { views.push({ buffer: 0, byteOffset: off, byteLength: p.byteLength }); off = pad4(off + p.byteLength); }
  const min = [0, 1, 2].map(i => Math.min(...[0, 1, 2, 3].map(v => pos[v * 3 + i])));
  const max = [0, 1, 2].map(i => Math.max(...[0, 1, 2, 3].map(v => pos[v * 3 + i])));
  const json = {
    asset: { version: "2.0", generator: "Track Tracer" }, scene: 0, scenes: [{ nodes: [0] }],
    nodes: [{ mesh: 0, name: "track reference" }],
    meshes: [{ primitives: [{ attributes: { POSITION: 0, NORMAL: 1, TEXCOORD_0: 2 }, indices: 3, material: 0 }] }],
    materials: [{ pbrMetallicRoughness: { baseColorTexture: { index: 0 }, metallicFactor: 0, roughnessFactor: 1 }, doubleSided: true, extensions: { KHR_materials_unlit: {} } }],
    extensionsUsed: ["KHR_materials_unlit"],
    textures: [{ source: 0, sampler: 0 }], samplers: [{ magFilter: 9729, minFilter: 9729 }],
    images: [{ bufferView: 4, mimeType: "image/png" }],
    accessors: [
      { bufferView: 0, componentType: 5126, count: 4, type: "VEC3", min, max },
      { bufferView: 1, componentType: 5126, count: 4, type: "VEC3" },
      { bufferView: 2, componentType: 5126, count: 4, type: "VEC2" },
      { bufferView: 3, componentType: 5123, count: 6, type: "SCALAR" },
    ],
    bufferViews: views, buffers: [{ byteLength: off }],
  };
  let jb = new TextEncoder().encode(JSON.stringify(json));
  const jlen = pad4(jb.length), total = 12 + 8 + jlen + 8 + off, out = new Uint8Array(total), dv = new DataView(out.buffer);
  dv.setUint32(0, 0x46546C67, true); dv.setUint32(4, 2, true); dv.setUint32(8, total, true);
  dv.setUint32(12, jlen, true); dv.setUint32(16, 0x4E4F534A, true);
  out.fill(0x20, 20, 20 + jlen); out.set(jb, 20);   // JSON chunk, padded with spaces
  const b0 = 20 + jlen;
  dv.setUint32(b0, off, true); dv.setUint32(b0 + 4, 0x004E4942, true);
  parts.forEach((p, i) => out.set(p, b0 + 8 + views[i].byteOffset));
  return out;
}
function trackName() { const n = (S.paths[0] && S.paths[0].name) || "track"; return (S.imgName || "track").replace(/\.[a-z0-9]+$/i, "").replace(/[^A-Za-z0-9 _-]+/g, "").trim().replace(/\s+/g, "-").toLowerCase() || "track"; }
// Every export gets its own time-stamped name, used for both the file and the prefab inside it.
// A repeated name is how a stale track gets placed: the browser saves the new file as "name (1).hmsprefab"
// and HMS still holds the first prefab under that name.
let lastPrefab = null, lastPrefabSig = "";
const stamp = () => { const d = new Date(), p2 = n => String(n).padStart(2, "0"); return p2(d.getHours()) + p2(d.getMinutes()) + p2(d.getSeconds()); };
const prefabSig = pl => JSON.stringify(pl.poses.map(q => [q.pos, q.z, q.yaw, q.pitch, (q.pc || pl.pc).name, (q.pc || pl.pc).lay]));
function prefabJson(pl, name) {
  const id = pl.pc.prefab;
  const objects = pl.poses.map(q => {
    const pc = q.pc || pl.pc, id = pc.prefab, c = pc.center || [0, 0, 0];
    const cy = Math.cos(q.yaw), sy = Math.sin(q.yaw), cp = Math.cos(q.pitch), sp = Math.sin(q.pitch);
    const axes = { A: [cy * cp, sy * cp, sp], L: [-sy, cy, 0], U: [-cy * sp, -sy * sp, cp] };
    // world directions of the piece's own x, y, z axes; HMS wants fwd = x and up = z
    const [X, Y, Z] = [...(pc.lay || "ALU")].map(ch => ch === ch.toUpperCase() ? axes[ch] : axes[ch.toUpperCase()].map(v => -v));
    // q is the middle of the piece's box; the pivot HMS positions is offset from it by center
    const pos = [q.pos[0], q.pos[1], q.z].map((v, k) => +(v - (X[k] * c[0] + Y[k] * c[1] + Z[k] * c[2])).toFixed(4));
    return {
      key: id.key, display: id.display, tag: id.tag, variant: id.variant, spot: id.spot, pos,
      fwd: X.map(v => +v.toFixed(6)), up: Z.map(v => +v.toFixed(6)),
      team: 8, placement: 204,   // 0xCC: spawn at start, phased physics (12 would be normal physics)
    };
  });
  const now = new Date();
  return JSON.stringify({
    format: "hms-prefab", version: 1, name: name || trackName(), game: pl.pc.game, canvas: id.canvas,
    created: now.toISOString().replace(/\.\d+Z$/, "Z"), created_unix: Math.floor(now / 1000), hms: "track-tracer",
    notes: `Traced track: ${pl.poses.length} blocks, ${Math.round(pl.length)} wu of centerline, scale ${fmt(wpp(), 5)} wu/px. Positions are relative to the image centre; Z = block centres.`,
    anchor: "footprint-centre-floor", objects,
  }, null, 1);
}
function update() {
  planCache = computePlan();
  $("scaleOut").textContent = `${fmt(wpp(), 4)} wu per pixel · image ${fmt(S.imgW * wpp(), 0)} × ${fmt(S.imgH * wpp(), 0)} wu`;
  $("layout").value = Math.round(S.layout * 100);
  const pcNow = piece(), tw = +$("trackW").value || 0, rows = Math.ceil(Math.max(tw, pcNow.wid) / pcNow.wid - 1e-6);
  $("rowsOut").textContent = rows > 1 ? `${rows} rows side by side` : pcNow.wid > tw + 1e-6 ? `this piece is ${fmt(pcNow.wid)} wu wide` : "";
  if (document.activeElement !== $("layoutNum")) $("layoutNum").value = Math.round(S.layout * 100);
  $("refScale").textContent = fmt(wpp(), 5);
  const br = $("budgetRow"); br.textContent = "";
  const pill = (cls, txt) => { const s = document.createElement("span"); s.className = "pill " + cls; s.textContent = txt; br.append(s); };
  if (!planCache) {
    ["sBlocks", "sLen", "sTol"].forEach(id => $(id).textContent = "–");
    $("prefabCmd").value = ""; $("refCmd").value = "";
    pill("warn", "trace a path to see blocks");
  } else {
    const pl = planCache;
    $("sBlocks").textContent = pl.poses.length; $("sLen").textContent = fmt(pl.length, 0); $("sTol").textContent = fmt(pl.tolUsed, 2);
    if (pl.ok) pill("ok", `${pl.poses.length} of ${pl.slots} free slots`); else pill("bad", pl.why);
    if (pl.relaxed) pill("warn", `gap relaxed from ${fmt(pl.tol, 2)} to fit`);
    if (pl.baseOff) pill("warn", "Track isn't flat, so bordered paths keep their road pieces and get no baseplate");
    const off = offStepCount();
    if (snapOn() && off) pill("warn", `${off} path${off === 1 ? " has" : "s have"} headings off the 5° steps — press Snap all to 5°`);
    const counts = new Map();
    for (const q of pl.poses) { const k = q.pc || pl.pc; counts.set(k, (counts.get(k) || 0) + 1); }
    if (counts.size > 1) pill("ok", [...counts].map(([k, n]) => `${n} × ${k.display || k.name}`).join(" + "));
    for (const [k, n] of counts) if (k.maxAllowed && n > k.maxAllowed) pill("warn", `${n} × ${k.display || k.name} is over Forge's ${k.maxAllowed} max${$("lift").checked ? "" : " — tick Ignore Forge limits"}`);
    const nm = lastPrefab, g = GAMES[pl.pc.game];
    const at = originTokens() || " camera";
    $("prefabCmd").value = !nm ? `Save the prefab first. The two commands for that exact file will appear here. Place it in a ${g.label} map variant (made for ${g.map}).`
      : prefabSig(pl) !== lastPrefabSig ? `The track has changed since ${nm}.hmsprefab was saved. Save the prefab again to get a fresh file and commands.`
      : `prefab import C:\\Users\\YOU\\Downloads\\${nm}.hmsprefab\nprefab place ${nm}${at}${$("lift").checked ? " noquota nobudget" : ""}`;
    const ov = ["ox", "oy", "oz"].map(id => $(id).value.trim());
    const pos = ov.every(x => x !== "" && isFinite(+x)) ? `${fmt(+ov[0], 3)} ${fmt(+ov[1], 3)} ${fmt(+ov[2] - pl.pc.hgt / 2 - 0.05, 3)}` : "X Y Z";
    $("refCmd").value = `import C:\\Users\\YOU\\Downloads\\${trackName()}-reference.glb\nimport moveto ${pos}`;
  }
  $("dlRef").disabled = !(S.img && S.img.complete);
  if (document.activeElement !== $("rotNum")) $("rotNum").value = gradeNum(S.imgRot, 2);
  $("rot").value = S.imgRot;
  $("dlPrefab").disabled = !planCache;
  $("copyPrefab").disabled = !planCache;
  $("sampleTag").hidden = !S.sample;
  const hints = {
    trace: !S.paths.length ? "Click where the track starts." : !(S.paths[S.active] || {}).start ? `Click where “${S.paths[S.active].name}” starts.`
      : S.kind === "p" ? `Point: click anywhere for the next point${snapOn() ? " (direction in 5° steps; untick 5° steps for any angle)" : ""}. Drag a point to move it, drag a diamond to bend between two points · right-click or Del deletes`
      : (S.kind === "s" ? "Straight: click ahead of the end to set its length." : S.turnMode === "free" ? "Free turn: click where the turn should end; it bends to meet it." : fixedR() ? `Turn ${fmt(turnAbs(), 1)}°, radius ${fmt(fixedR() * wpp(), 2)} wu: click the side it turns to.` : `Turn ${fmt(turnAbs(), 1)}°: click on the side it turns to, as far along as it should reach.`)
        + " Drag a dot to resize · drag a diamond to bend · Shift+drag keeps the other points fixed (free curve) · right-click or Del deletes",
    width: "Width: click the centerline where the track should be that wide. It widens or narrows in a straight line from the width before it. Edit the points in Active path.",
    cal: S.cal.length === 0 ? "Set scale: click one edge of a lane." : S.cal.length === 1 ? "Now click the opposite edge." : `Measured ${fmt(V.len(V.sub(S.cal[1], S.cal[0])), 1)} px. Enter its real width and press Apply.`,
    pan: "Drag to move the view. Scroll to zoom.",
    jump: "Jump: click the track to drop a marker · drag it to move · drag its round handle to turn (snaps every 90°, Ctrl turns freely) · Del removes it.",
    grade: S.gradePick.length === 0 ? "Grade: click where the section starts on a centerline." : S.gradePick.length === 1 ? "Now click where it ends. On a loop, it runs forward in the direction you traced." : "Set the grade in the Elevation panel, then press Add section.",
  };
  $("hint").textContent = hints[S.mode];
  $("calApply").disabled = S.cal.length !== 2;
  updateElevation(); renderJumps();
}
function updateElevation() {
  const pl = planCache, box = $("elevPills");
  box.textContent = "";
  const pill = (cls, txt) => { const s = document.createElement("span"); s.className = "pill " + cls; s.textContent = txt; box.append(s); };
  $("sRise").textContent = pl && pl.elevated ? gradeNum(pl.zMax - pl.zMin) : "–";
  $("sSteep").textContent = pl && pl.elevated ? gradeNum(pl.steepest, 1) : "–";
  $("sLip").textContent = pl && pl.elevated ? gradeNum(pl.lip, 3) : "–";
  if (pl) {
    for (const w of pl.paths) {
      if (w.closed && Math.abs(w.profile.closure) > 0.01) pill("bad", `${w.src.name} misses its start by ${signedNum(w.profile.closure)} wu`);
    }
    if (pl.elevated && pl.lip > 0.1) pill("warn", `Graded bends step up to ${gradeNum(pl.lip)} wu`);
  }
  const picking = S.gradePick.length === 2;
  $("gradeForm").hidden = !picking;
  if (picking) gradePreview();
  renderGrades();
}
// The two ends of the section being added, resolved on the current curve.
function pendingRange() {
  const w = planCache && planCache.paths.find(x => x.src === S.paths[S.gradePick[0].path]);
  if (!w || !w.arc) return null;
  const sa = w.station(S.gradePick[0]), sb = w.station(S.gradePick[1]);
  return { w, sa, sb, L: sb > sa ? sb - sa : w.total - sa + sb };
}
function gradePreview() {
  if (S.gradePick.length !== 2) return;
  const r = pendingRange(); if (!r) return;
  const curve = $("gShape").value === "curve", g0 = +$("gStart").value || 0, g1 = curve ? +$("gEnd").value || 0 : g0;
  const rise = r.L * (g0 + g1) / 200;
  $("gPreview").textContent = `${gradeNum(r.L, 1)} wu long · ${rise >= 0 ? "rises" : "drops"} ${gradeNum(Math.abs(rise))} wu`;
}
function changed(withSave = true) { S.gradePick = []; if (withSave) save(); renderPaths(); update(); draw(); }

// ============================================================================================
// Input
// ============================================================================================
function setMode(m) {
  S.mode = m; S.cal = m === "cal" ? [] : S.cal; S.sel = null; S.gradePick = []; S.gradeHover = null; S.ghost = null;
  $("tWidth").setAttribute("aria-pressed", m === "width");
  $("tTrace").setAttribute("aria-pressed", m === "trace"); $("tCal").setAttribute("aria-pressed", m === "cal"); $("tPan").setAttribute("aria-pressed", m === "pan"); $("tGrade").setAttribute("aria-pressed", m === "grade"); $("tJump").setAttribute("aria-pressed", m === "jump");
  document.querySelectorAll(".toolhdr .ts").forEach(t => { t.hidden = t.dataset.tool !== m; });
  board.style.cursor = m === "pan" ? "grab" : "crosshair"; update(); draw();
}
// Tool settings for Trace: what the next click adds, and how a turn gets its angle.
function setKind(k, tmode) {
  if (k) S.kind = k;
  if (tmode) S.turnMode = tmode;
  document.querySelectorAll("[data-kind]").forEach(b => b.setAttribute("aria-pressed", b.dataset.kind === S.kind));
  document.querySelectorAll("[data-tmode]").forEach(b => b.setAttribute("aria-pressed", b.dataset.tmode === S.turnMode));
  $("turnModeWrap").hidden = S.kind !== "t";
  $("turnWrap").hidden = S.kind !== "t" || S.turnMode !== "set";
  try { localStorage.setItem("trackTracer.tool", JSON.stringify({ kind: S.kind, turnMode: S.turnMode })); } catch (e) {}
  if (S.mode !== "trace") setMode("trace"); else { update(); draw(); }
}
document.querySelectorAll("[data-kind]").forEach(b => b.addEventListener("click", () => setKind(b.dataset.kind)));
document.querySelectorAll("[data-tmode]").forEach(b => b.addEventListener("click", () => setKind(null, b.dataset.tmode)));
// Turn angle presets
const markPreset = () => document.querySelectorAll("[data-ang]").forEach(b => b.setAttribute("aria-pressed", +b.dataset.ang === turnAbs()));
document.querySelectorAll("[data-ang]").forEach(b => b.addEventListener("click", () => { $("turnAng").value = b.dataset.ang; markPreset(); update(); draw(); }));
$("turnAng").addEventListener("input", markPreset);
$("turnR").addEventListener("change", () => { update(); draw(); });
// Heading readout in the status bar: the section under the cursor, or where the active path ends.
function readout() {
  const p = S.paths[S.active], out = $("readout");
  if (S.mode !== "trace" || !p || !p.start) { out.textContent = ""; return; }
  const k = wpp(), ps = posesOf(p), end = ps[ps.length - 1];
  const g = S.drag && S.drag.i >= 0 && p.secs[S.drag.i] ? { sec: p.secs[S.drag.i], from: ps[S.drag.i] } : S.ghost ? { sec: S.ghost.sec, from: { pt: end.pt, head: S.ghost.head !== undefined ? S.ghost.head : end.head + (S.ghost.sec.kink || 0) } } : null;
  let html;
  if (g && g.sec.k === "s") html = `Straight <b>${fmt(g.sec.len * k, 2)} wu</b> at <b>${showDeg(g.from.head)}°</b>`;
  else if (g && g.sec.k === "c") html = `Curve <b>${showDeg(curveStartHead(g.from, g.sec))}°</b> → <b>${showDeg(secEnd(g.from, g.sec).head)}°</b>, ${fmt(V.len(V.sub(secEnd(g.from, g.sec).pt, g.from.pt)) * k, 2)} wu across`;
  else if (g) html = `${g.sec.free ? "Free turn" : "Turn"} <b>${fmt(Math.abs(g.sec.ang), 2)}° ${g.sec.ang > 0 ? "left" : "right"}</b> r <b>${fmt(g.sec.r * k, 2)} wu</b> → heading <b>${showDeg(g.from.head + g.sec.ang)}°</b>`;
  else html = p.head == null ? "Heading not set yet" : `End heading <b>${showDeg(end.head)}°</b> · start heading ${showDeg(p.head)}°`;
  out.innerHTML = html;
}
function local(e) { const r = cv.getBoundingClientRect(); return [e.clientX - r.left, e.clientY - r.top]; }
// ---- building paths from sections ----------------------------------------------------------
const snapOn = () => $("snap5").checked;
const snapHead = h => snapOn() ? Math.round(h / 5) * 5 : +(+h).toFixed(4);
const snapAng = a => snapOn() ? Math.min(Math.max(Math.round(Math.abs(a) / 5) * 5, 5), 270) : Math.min(Math.max(Math.abs(a), 0.5), 270);
const turnAbs = () => snapAng(+$("turnAng").value || 90);
const gridPx = () => Math.max(0, +$("gridWu").value || 0) / wpp();   // lengths and radii land on multiples of this
const snapLen = (v, minWu) => { const g = gridPx(), m = minWu / wpp(); return Math.max(g > 0 ? Math.round(v / g) * g : v, m); };
const onGridPt = pt => { const g = gridPx(); return g > 0 ? [Math.round((pt[0] - S.imgW / 2) / g) * g + S.imgW / 2, Math.round((pt[1] - S.imgH / 2) / g) * g + S.imgH / 2] : pt; };
const headTo = (a, b) => Math.atan2(-(b[1] - a[1]), b[0] - a[0]) / RAD;
// The section of this kind from pose that reaches closest to q (image px). freeHead: a first
// straight also sets the heading. closeTo: a loop's start, which a straight snaps its length to
// when it lies on the line.
function solveSec(pose, kind, q, angAbs, freeHead, closeTo) {
  const d = V.sub(q, pose.pt);
  if (kind === "s") {
    const head = freeHead ? snapHead(headTo(pose.pt, q)) : pose.head, u = dirPx(head), along = V.dot(d, u);
    if (along <= 0 && !freeHead) return null;
    let len = snapLen(Math.max(along, 0), 0.25);
    if (closeTo) {
      const e = V.sub(closeTo, pose.pt), to = V.dot(e, u);
      if (to > 0 && Math.abs(V.dot(e, leftPx(head))) * wpp() < 0.01 && Math.abs(to - along) * S.view.z < 14) len = to;
    }
    return { sec: { k: "s", len }, head };
  }
  if (pose.head == null) return null;
  const side = V.dot(d, leftPx(pose.head)), free = angAbs == null;
  if (free) {   // the arc that leaves along the heading and passes through q: angle = twice the chord's angle
    if (Math.abs(side) * wpp() < 1e-3) return null;
    const a = Math.abs(2 * Math.atan2(side, V.dot(d, dirPx(pose.head))) / RAD);
    if (a > 270 + 1e-9) return null;
    if (!snapOn()) return a < 0.5 ? null : { sec: { k: "t", ang: Math.sign(side) * +a.toFixed(4), r: snapLen(V.dot(d, d) / (2 * Math.abs(side)), 0.5), free: true }, head: pose.head };
    angAbs = snapAng(a);   // with 5° steps the angle snaps, then the click sets the radius as usual
  }
  const sg = side >= 0 ? 1 : -1, ang = sg * angAbs, chord = V.dot(d, dirPx(pose.head + ang / 2));
  if (chord <= 0) return null;
  const sec = { k: "t", ang, r: snapLen(chord / (2 * Math.sin(angAbs * RAD / 2)), 0.5) };
  if (free) sec.free = true;
  return { sec, head: pose.head };
}
// A radius typed in the Turn settings (image px), or null when the click sets it.
const fixedR = () => { const v = +$("turnR").value; return v > 0 ? Math.max(v, 0.5) / wpp() : null; };
// What a click at q would add to the end of p.
function nextSection(p, q) {
  const kind = S.kind, n = p.secs.length, first = kind !== "t" && n === 0, end = posesOf(p)[n];
  if (kind === "p") return nextPoint(p, q, end, n);
  if (kind === "t" && p.head == null) return { err: "Start with a straight, or type a start heading in Active path" };
  const R = kind === "t" && S.turnMode === "set" ? fixedR() : null;
  if (R) {   // a set radius: the click only picks the side
    const ang = (V.dot(V.sub(q, end.pt), leftPx(end.head)) >= 0 ? 1 : -1) * turnAbs();
    return { sec: { k: "t", ang, r: R } };
  }
  const r = solveSec({ pt: end.pt, head: p.head == null ? null : end.head }, kind, q, S.turnMode === "free" ? null : turnAbs(), first, p.closed ? p.start : null);
  if (!r) return { err: kind === "s" ? "Click ahead of the end of the track" : "Click ahead of the end, on the side it turns to" };
  return { sec: r.sec, head: first ? r.head : undefined };
}
// Point mode: a straight from the end to wherever you click, in any direction (a corner where it
// meets the last section). On a loop, a click on the start dot closes it exactly.
function nextPoint(p, q, end, n) {
  const toStart = p.closed && n > 0 && V.len(V.sub(scr(p.start), scr(q))) < 10, to = toStart ? p.start : q, d = V.sub(to, end.pt);
  if (V.len(d) * wpp() < 0.25) return { err: "Click further from the end" };
  const H = toStart || !snapOn() ? headTo(end.pt, to) : snapHead(headTo(end.pt, to));
  const len = toStart ? V.len(d) : snapLen(Math.max(V.dot(d, dirPx(H)), 0), 0.25), sec = { k: "s", len };
  if (n === 0) return { sec, head: H };
  const k = wrap180(H - end.head);
  if (Math.abs(k) > 1e-9) sec.kink = k;
  return { sec };
}
// Point mode, dragging a point: it moves on its own. The section ending at it and the one starting
// from it reach to it (a straight stays straight; a turn or curve keeps its far end and handles),
// and every other section stays exactly where it is.
function vertexMove(p, j, q) {
  const ps = posesOf(p), n = p.secs.length, H = ps.slice(0, n).map(x => x.head);
  const moved = V.sub(q, j < 0 ? p.start : secEnd(ps[j], p.secs[j]).pt);
  if (j >= 0) {
    const s = p.secs[j], A = cubicAbs(ps[j], s);
    if (s.k === "s") { H[j] = headTo(A.a, q); p.secs[j] = { k: "s", len: Math.max(V.len(V.sub(q, A.a)), 1e-3) }; }
    else p.secs[j] = curveFrom(ps[j], { a: A.a, c1: A.c1, c2: V.add(A.c2, moved), b: q });
  } else p.start = q;
  if (j + 1 < n) {
    const k = j + 1, s = p.secs[k], A = cubicAbs(ps[k], s);
    if (s.k === "s") { H[k] = headTo(q, A.b); p.secs[k] = { k: "s", len: Math.max(V.len(V.sub(A.b, q)), 1e-3) }; }
    else p.secs[k] = curveFrom({ pt: q, head: H[k] }, { a: q, c1: V.add(A.c1, moved), c2: A.c2, b: A.b });
  }
  relink(p, H);
}
// Gives every section the absolute start heading H[i], adding kinks where needed.
function relink(p, H) {
  if (!p.secs.length) return;
  p.head = H[0];
  let pose = { pt: p.start, head: H[0] };
  p.secs.forEach((s, i) => {
    if (i > 0) { const k = wrap180(H[i] - pose.head); if (Math.abs(k) > 1e-9) s.kink = k; else delete s.kink; }
    pose = secEnd({ pt: pose.pt, head: H[i] }, s);
  });
}
// Section i deleted: the ones after it join on to the one before. Grade sections that end on it go too.
function removeSection(p, i) {
  p.secs.splice(i, 1);
  const shift = a => a.seg > i ? { seg: a.seg - 1, t: a.t } : a;
  p.grades = (p.grades || []).filter(g => g.a.seg !== i && g.b.seg !== i).map(g => ({ ...g, a: shift(g.a), b: shift(g.b) }));
  p.wstops = (p.wstops || []).filter(s => s.a.seg !== i).map(s => ({ ...s, a: shift(s.a) }));
}
function removeAt(h) {   // h.i = -1 is the start point, which goes only once the path has no sections
  const p = S.paths[h.path]; if (!p) return false;
  if (h.i >= 0) { removeSection(p, h.i); return true; }
  if (p.secs.length) { flash($("msgSec"), "Delete its sections first, or delete the whole path", false); return false; }
  p.start = null; return true;
}
// The diamond: the middle of a section, which bends it. A turn's middle follows the cursor, so
// its angle and radius change. A straight dragged off its line becomes a turn that bends through
// the cursor, and dropped back on the line it is a straight again. With Shift, see bendFree.
function secMid(pose, s) {
  if (s.k === "s") return V.add(pose.pt, V.mul(dirPx(pose.head), s.len / 2));
  if (s.k === "c") return toWorld(pose, bez([0, 0], s.c1, s.c2, s.e, 0.5));
  const sg = Math.sign(s.ang) || 1, c = V.add(pose.pt, V.mul(leftPx(pose.head), sg * s.r));
  return V.sub(c, V.mul(leftPx(pose.head + s.ang / 2), sg * s.r));
}
function bendSec(pose, q) {
  if (pose.head == null) return null;
  const d = V.sub(q, pose.pt), along = V.dot(d, dirPx(pose.head)), side = V.dot(d, leftPx(pose.head));
  if (Math.abs(side) * S.view.z < 4) return along > 0 ? { k: "s", len: snapLen(2 * along, 0.25) } : null;   // on the line: straight
  // The arc leaves along the heading and its middle is at q, so its angle is four times the angle to q.
  let a = 4 * Math.atan2(Math.abs(side), along) / RAD;
  a = snapOn() ? Math.min(270, Math.max(5, Math.round(a / 5) * 5)) : Math.min(270, Math.max(0.5, a));
  const sg = side > 0 ? 1 : -1, m = V.mul(V.sub(leftPx(pose.head), leftPx(pose.head + sg * a / 2)), sg);   // the arc's middle, per unit radius
  const r = V.dot(d, m) / V.dot(m, m);
  return r > 0 ? { k: "t", ang: sg * a, r: snapLen(r, 0.5) } : null;
}
function bendSection(dp, i, pt) {
  const old = dp.secs[i], r = bendSec(posesOf(dp)[i], pt);
  if (!r) return;
  if (r.k === "t" && (old.k !== "t" || old.free)) r.free = true;
  if (old.kink) r.kink = old.kink;
  dp.secs[i] = r;
}
// A path's start dot (i = -1) or the dot at the end of section i, under the cursor; failing that, a
// diamond (mid: true) on the active path. The active path wins.
function hitHandle(q) {
  let best = null, bd = 9;
  S.paths.forEach((p, k) => {
    if (!p.start) return;
    posesOf(p).forEach((pose, j) => {
      if (j > 0 && k !== S.active) return;
      const s = scr(pose.pt), d = Math.hypot(s[0] - q[0], s[1] - q[1]) - (k === S.active ? 0.5 : 0);
      if (d < bd) { bd = d; best = { path: k, i: j - 1 }; }
    });
  });
  const p = S.paths[S.active];
  if (!best && p && p.start && S.mode === "trace") {
    const ps = posesOf(p); bd = 10;
    p.secs.forEach((s, i) => { const m = scr(secMid(ps[i], s)), d = Math.hypot(m[0] - q[0], m[1] - q[1]); if (d < bd) { bd = d; best = { path: S.active, i, mid: true }; } });
  }
  return best;
}
// Every move starts again from the path as it was when the drag began, so Shift can be pressed or
// let go mid-drag.
function dragSection(e, q) {
  const dp = S.paths[S.drag.path], i = S.drag.i, pt = img(q);
  if (S.drag.base) { const b = JSON.parse(S.drag.base); dp.secs = b.secs; dp.head = b.head; dp.start = b.start; }
  const pointMode = S.kind === "p";   // in Point mode a drag never moves the other points
  if (S.drag.mid) { if (e.shiftKey || pointMode) bendFree(dp, i, pt); else bendSection(dp, i, pt); return; }
  if (pointMode && !e.shiftKey && (i >= 0 || dp.secs.length)) { vertexMove(dp, i, pt); return; }
  if (i < 0) { dp.start = e.ctrlKey ? pt : onGridPt(pt); return; }
  const s = dp.secs[i];
  if (e.shiftKey || s.k === "c") { moveEndFree(dp, i, pt); return; }
  const first = s.k === "s" && i === 0, pose = posesOf(dp)[i];
  const r = solveSec({ pt: pose.pt, head: first && dp.head == null ? null : pose.head }, s.k, pt, s.free ? null : Math.abs(s.ang || 0), first, dp.closed && i === dp.secs.length - 1 ? dp.start : null);
  if (r) { if (s.kink) r.sec.kink = s.kink; dp.secs[i] = r.sec; if (first) dp.head = r.head; }
}
const startDrag = h => { const p = S.paths[h.path]; S.drag = { ...h, base: JSON.stringify({ secs: p.secs, head: p.head, start: p.start }) }; };
// Headings that break the 5° rule: a start heading or a turn angle that is not a multiple of 5.
const off5 = v => Math.abs(v / 5 - Math.round(v / 5)) > 1e-6;
const offStepCount = () => S.paths.filter(p => (p.head != null && off5(p.head)) || p.secs.some(s => s.k === "t" && off5(s.ang))).length;
function snapAll() {
  for (const p of S.paths) {
    if (p.head != null) p.head = Math.round(p.head / 5) * 5;
    for (const s of p.secs) if (s.k === "t") s.ang = Math.sign(s.ang) * Math.max(5, Math.round(Math.abs(s.ang) / 5) * 5);
  }
}
// Turns the image, and if asked the paths and markers with it, by delta degrees about its centre.
function turnTracing(delta) {
  const a = delta * RAD, cx = S.imgW / 2, cy = S.imgH / 2, cs = Math.cos(a), sn = Math.sin(a);
  const rot = pt => { const dx = pt[0] - cx, dy = pt[1] - cy; return [cx + dx * cs + dy * sn, cy - dx * sn + dy * cs]; };
  for (const p of S.paths) {
    if (p.start) p.start = rot(p.start);
    if (p.head != null) p.head = +(p.head + delta).toFixed(6);
    if (p.guide) p.guide = p.guide.map(rot);
  }
  for (const j of S.jumps) { j.pos = rot(j.pos); j.rot = +((((j.rot + delta) % 360) + 360) % 360).toFixed(6); }
}
// The spot on a centerline under the cursor, snapped to nearby section ends and control points.
function pickOnPath(q, only) {
  if (!planCache) return null;
  const c = toWu(img(q)), wuPerScreenPx = wpp() / S.view.z;
  let best = null, bd = 14 * wuPerScreenPx;
  for (const w of planCache.paths) {
    const k = S.paths.indexOf(w.src);
    if (!w.arc || (only !== null && k !== only)) continue;
    const edges = w.closed ? w.pts.length : w.pts.length - 1;
    for (let i = 0; i < edges; i++) {
      const a = w.pts[i], ab = V.sub(w.pts[(i + 1) % w.pts.length], a), l2 = V.dot(ab, ab);
      const u = l2 < 1e-12 ? 0 : Math.min(Math.max(V.dot(V.sub(c, a), ab) / l2, 0), 1);
      const d = V.len(V.sub(V.add(a, V.mul(ab, u)), c)) - (k === S.active ? 2 * wuPerScreenPx : 0);
      if (d < bd) { bd = d; best = { w, k, s: w.cum[i] + u * Math.sqrt(l2) }; }
    }
  }
  if (!best) return null;
  const { w, k, s } = best, st = w.stations, segs = st.length - 1;
  const away = x => { const d = Math.abs(x - s); return w.closed ? Math.min(d, w.total - d) : d; };
  let anchor = null, ad = 9 * wuPerScreenPx;
  for (const sec of w.sections) for (const [x, a] of [[sec.sa, sec.src.a], [sec.sb, sec.src.b]]) if (away(x) < ad) { ad = away(x); anchor = { seg: a.seg, t: a.t }; }
  if (!anchor) st.forEach((x, i) => { if (away(x) < ad) { ad = away(x); anchor = i < segs ? { seg: i, t: 0 } : { seg: segs - 1, t: 1 }; } });
  if (!anchor) {
    let seg = 0; while (seg < segs - 1 && st[seg + 1] <= s) seg++;
    const span = st[seg + 1] - st[seg];
    anchor = { seg, t: span > 0 ? Math.min(Math.max((s - st[seg]) / span, 0), 1) : 0 };
  }
  return { path: k, seg: anchor.seg, t: anchor.t, wu: w.arc.at(w.station(anchor)) };
}
function hitJump(q) {
  const near = (pt, r) => { const s = scr(pt); return Math.hypot(s[0] - q[0], s[1] - q[1]) < r; };
  if (S.jumpSel !== null && S.jumps[S.jumpSel] && near(jumpHandle(S.jumps[S.jumpSel]), 10)) return { jump: S.jumpSel, rot: true };
  for (let i = S.jumps.length - 1; i >= 0; i--) if (near(S.jumps[i].pos, 11)) return { jump: i };
  return null;
}
// Direction of travel (degrees) of the centerline nearest a point, and how far away it is.
function trackDir(c) {
  const n = planCache && nearestOnPaths(planCache.paths, c); if (!n) return null;
  const t = n.w.arc.tangent(n.s, 1); return { deg: Math.atan2(t[1], t[0]) * 180 / Math.PI, d: n.d };
}
const jumpSpot = q => { const pk = pickOnPath(q, null); return pk ? toPx(pk.wu) : img(q); };   // sticks to a centerline when close
function jumpDown(q) {
  const h = hitJump(q);
  pushUndo();
  if (h) { S.jumpSel = h.jump; S.drag = h; }
  else {   // a new marker faces along the track, so the block stands across the lane
    const pos = jumpSpot(q), dir = trackDir(toWu(pos));
    S.jumps.push({ pos, rot: dir && dir.d < 4 ? +((dir.deg + 360) % 360).toFixed(2) : 0, size: $("jumpSize").value });
    S.jumpSel = S.jumps.length - 1; S.drag = { jump: S.jumpSel };
  }
  update(); draw();
}
function jumpDrag(q, e) {
  const j = S.jumps[S.drag.jump]; if (!j) return;
  if (!S.drag.rot) j.pos = e.ctrlKey ? img(q) : jumpSpot(q);
  else {
    const p = img(q); let deg = Math.atan2(-(p[1] - j.pos[1]), p[0] - j.pos[0]) * 180 / Math.PI;
    if (!e.ctrlKey) {   // hard snaps: every 90 degrees on the image, and square to the track under the marker
      const dir = trackDir(toWu(j.pos));
      for (const b of dir && dir.d < 4 ? [0, dir.deg] : [0]) { const k = Math.round((deg - b) / 90) * 90 + b; if (Math.abs(deg - k) <= 10) { deg = k; break; } }
    }
    j.rot = +(((deg % 360) + 360) % 360).toFixed(2);
  }
  update(); draw();
}
function gradeClick(q) {
  if (S.gradePick.length === 2) S.gradePick = [];
  const only = S.gradePick.length ? S.gradePick[0].path : null, pick = pickOnPath(q, only);
  if (!pick) { flash($("msgGrade"), only === null ? "Click on a centerline" : `Click on “${S.paths[only].name}”`, false); return; }
  if (only === null) { S.active = pick.path; S.gradePick = [pick]; renderPaths(); update(); draw(); return; }
  const w = planCache.paths.find(x => x.src === S.paths[only]);
  let a = S.gradePick[0], b = pick;
  if (Math.abs(w.station(a) - w.station(b)) < 0.1) { flash($("msgGrade"), "Pick a different end point", false); return; }
  if (!w.closed && w.station(b) < w.station(a)) [a, b] = [b, a];
  const mine = gradeRanges(w.station(a), w.station(b), 0, 0, w.total);
  const clash = w.sections.findIndex(sec => sec.ok && gradeRanges(sec.sa, sec.sb, 0, 0, w.total).some(r => mine.some(m => Math.max(r.a, m.a) < Math.min(r.b, m.b) - 1e-3)));
  if (clash >= 0) { S.gradePick = []; flash($("msgGrade"), `That overlaps section ${clash + 1}`, false); update(); draw(); return; }
  S.gradePick = [a, b];
  if ($("gShape").value === "curve") $("gStart").value = gradeNum(w.profile.gradeAt(w.station(a) - 0.01));   // start from the grade coming in
  update(); draw(); $("gStart").focus(); $("gStart").select();
}
let pan = null;
cv.addEventListener("contextmenu", e => e.preventDefault());
cv.addEventListener("pointerdown", e => {
  cv.setPointerCapture(e.pointerId);
  const q = local(e), p = img(q);
  const onDot = S.mode === "trace" && e.button === 2 && hitHandle(q);
  if (e.button === 1 || e.button === 2 && !onDot || S.mode === "pan" || spaceDown) { pan = { q, x: S.view.x, y: S.view.y }; board.style.cursor = "grabbing"; return; }
  if (e.button === 2) { pushUndo(); if (removeAt(onDot)) { S.sel = null; changed(); } else S.undo.pop(); return; }
  if (S.mode === "cal") { if (S.cal.length >= 2) S.cal = []; S.cal.push(p); update(); draw(); return; }
  if (S.mode === "grade") { gradeClick(q); return; }
  if (S.mode === "width") { widthClick(q); return; }
  if (S.mode === "jump") { jumpDown(q); return; }
  // trace: grab a dot, set a start, or add the next section (held down, it follows the mouse)
  S.ghost = null;
  let h = hitHandle(q);
  if (h && S.kind === "p" && h.i === -1 && h.path === S.active) {   // Point mode: a click on a loop's start dot closes it
    const lp = S.paths[h.path], ps = posesOf(lp);
    if (lp.closed && lp.secs.length >= 2 && V.len(V.sub(ps[ps.length - 1].pt, lp.start)) * wpp() > 1e-3) h = null;
  }
  if (h) { pushUndo(); S.sel = { path: h.path, i: h.i }; S.active = h.path; startDrag(h); renderPaths(); draw(); return; }
  if (!S.paths.length) { pushUndo(); S.paths.push({ name: "Main loop", closed: true, start: null, head: null, secs: [], grades: [] }); S.active = 0; }
  const path = S.paths[S.active];
  if (!path.start) { pushUndo(); path.start = e.ctrlKey ? p : onGridPt(p); S.sel = { path: S.active, i: -1 }; S.drag = { ...S.sel }; changed(); return; }
  const r = nextSection(path, p);
  if (r.err) { flash($("msgSec"), r.err, false); return; }
  pushUndo(); path.secs.push(r.sec); if (r.head !== undefined) path.head = r.head;
  S.sel = { path: S.active, i: path.secs.length - 1 }; startDrag(S.sel); changed();
});
cv.addEventListener("pointerleave", () => { if (S.ghost) { S.ghost = null; draw(); } });
cv.addEventListener("pointermove", e => {
  const q = local(e);
  if (pan) { S.viewTouched = true; S.view.x = pan.x + q[0] - pan.q[0]; S.view.y = pan.y + q[1] - pan.q[1]; draw(); return; }
  if (S.drag && S.drag.jump !== undefined) { jumpDrag(q, e); return; }
  if (S.drag) { dragSection(e, q); planCache = null; draw(); return; }
  if (S.mode === "cal" && S.cal.length === 1) { S.calHover = img(q); draw(); return; }
  if (S.mode === "grade" || S.mode === "width") { const pk = pickOnPath(q, S.mode === "grade" && S.gradePick.length === 1 ? S.gradePick[0].path : null); S.gradeHover = pk ? pk.wu : null; draw(); return; }
  if (S.mode !== "trace") return;
  const h = hitHandle(q), p = S.paths[S.active], gh = !h && p && p.start ? nextSection(p, img(q)) : null;
  S.hover = h; S.ghost = gh && !gh.err ? gh : null; draw();
});
cv.addEventListener("pointerup", () => {
  if (pan) { pan = null; board.style.cursor = S.mode === "pan" ? "grab" : "crosshair"; return; }
  if (S.drag) { S.drag = null; changed(); }
});
cv.addEventListener("wheel", e => {
  e.preventDefault();
  const q = local(e), before = img(q), f = Math.exp(-e.deltaY * 0.0015);
  S.viewTouched = true;
  S.view.z = Math.min(Math.max(S.view.z * f, 0.05), 40);
  S.view.x = q[0] - before[0] * S.view.z; S.view.y = q[1] - before[1] * S.view.z; draw();
}, { passive: false });
let spaceDown = false;
window.addEventListener("keydown", e => {
  if (e.target.closest && e.target.closest("input, textarea, select")) return;
  if (e.key === " ") { spaceDown = true; e.preventDefault(); }
  else if ((e.ctrlKey || e.metaKey) && e.key.toLowerCase() === "z") { e.preventDefault(); undo(); }
  else if ((e.key === "Delete" || e.key === "Backspace") && S.mode === "jump" && S.jumpSel !== null) { pushUndo(); S.jumps.splice(S.jumpSel, 1); S.jumpSel = null; changed(); }
  else if (e.key.toLowerCase() === "j") setMode("jump");
  else if ((e.key === "Delete" || e.key === "Backspace") && S.sel && S.mode === "trace") { pushUndo(); if (removeAt(S.sel)) { S.sel = null; changed(); } else S.undo.pop(); }
  else if (e.key.toLowerCase() === "s" && !e.ctrlKey && !e.metaKey) setKind("s");
  else if (e.key.toLowerCase() === "r" && !e.ctrlKey && !e.metaKey) setKind("t");
  else if (e.key.toLowerCase() === "e" && !e.ctrlKey && !e.metaKey) setKind("p");
  else if (e.key.toLowerCase() === "t") setMode("trace");
  else if (e.key.toLowerCase() === "g") setMode("grade");
  else if (e.key.toLowerCase() === "w") setMode("width");
  else if (e.key.toLowerCase() === "c") setMode("cal");
  else if (e.key.toLowerCase() === "p") setMode("pan");
  else if (e.key.toLowerCase() === "f") fit();
  else if (e.key === "Escape") { S.sel = null; S.cal = []; S.gradePick = []; S.ghost = null; update(); draw(); }
});
window.addEventListener("keyup", e => { if (e.key === " ") spaceDown = false; });

$("tTrace").onclick = () => setMode("trace");
$("tCal").onclick = () => setMode("cal");
$("tPan").onclick = () => setMode("pan");
$("tGrade").onclick = () => setMode("grade");
$("tWidth").onclick = () => setMode("width");
function widthClick(q) {
  const pk = pickOnPath(q, null);
  if (!pk) { flash($("msgProj"), "Click on a centerline to add a width point", false); return; }
  const p = S.paths[pk.path], w = Math.min(Math.max(+$("wNew").value || 0, 0.5), 200);
  pushUndo(); S.active = pk.path;
  (p.wstops || (p.wstops = [])).push({ a: { seg: pk.seg, t: pk.t }, w });
  changed();
}
$("tJump").onclick = () => setMode("jump");
$("turnAng").addEventListener("change", () => { $("turnAng").value = gradeNum(turnAbs(), 2); update(); });
$("snap5").addEventListener("change", () => {
  $("turnAng").step = snapOn() ? "5" : "1";
  if (snapOn() && offStepCount()) { pushUndo(); snapAll(); flash($("msgSec"), "Every heading and turn snapped to 5° steps", true); }
  $("turnAng").value = gradeNum(turnAbs(), 2); changed();
});
$("snapAll").onclick = () => {
  if (!offStepCount()) { flash($("msgSec"), "All headings are already on 5° steps", true); return; }
  pushUndo(); snapAll(); changed(); flash($("msgSec"), "Every heading and turn snapped to 5° steps", true);
};
let rotEditing = false;   // one undo step per slider drag
function setRot(deg) {
  deg = clampRot(deg);
  if (!rotEditing) { pushUndo(); rotEditing = true; }
  const delta = deg - S.imgRot; S.imgRot = deg;
  if (delta && $("rotPaths").checked) turnTracing(delta);
  changed();
}
$("rot").addEventListener("input", () => setRot(+$("rot").value));
$("rot").addEventListener("change", () => { rotEditing = false; });
$("rotNum").addEventListener("change", () => { setRot(+$("rotNum").value); rotEditing = false; });
$("rotZero").onclick = () => { setRot(0); rotEditing = false; };
$("gShape").onchange = () => {
  const curve = $("gShape").value === "curve";
  $("gEndWrap").hidden = !curve; $("gStartLabel").textContent = curve ? "From" : "Grade";
  gradePreview();
};
["gStart", "gEnd"].forEach(id => {
  $(id).addEventListener("input", gradePreview);
  $(id).addEventListener("keydown", e => { if (e.key === "Enter") $("gAdd").click(); });
});
$("gCancel").onclick = () => { S.gradePick = []; update(); draw(); };
$("gAdd").onclick = () => {
  if (S.gradePick.length !== 2) return;
  const [a, b] = S.gradePick, p = S.paths[a.path], curve = $("gShape").value === "curve";
  const g0 = clampGrade(+$("gStart").value || 0), g1 = curve ? clampGrade(+$("gEnd").value || 0) : g0;
  pushUndo();
  (p.grades || (p.grades = [])).push({ a: { seg: a.seg, t: a.t }, b: { seg: b.seg, t: b.t }, g0, g1, curve });
  changed();
};
$("z0").onchange = () => {
  const p = S.paths[S.active]; if (!p) return;
  pushUndo(); p.z0 = isFinite(+$("z0").value) ? +$("z0").value : 0; changed();
};
pv.addEventListener("pointermove", e => {
  const w = activeWu(); if (!w || !w.arc) return;
  const r = pv.getBoundingClientRect(), x0 = PROF_PAD.l, x1 = r.width - PROF_PAD.r;
  S.profHover = Math.min(Math.max((e.clientX - r.left - x0) / (x1 - x0), 0), 1) * w.total; draw();
});
pv.addEventListener("pointerleave", () => { S.profHover = null; draw(); });
$("bFit").onclick = fit;
$("bUndo").onclick = undo;
$("calApply").onclick = () => {
  const d = +$("calDist").value, px = S.cal.length === 2 ? V.len(V.sub(S.cal[1], S.cal[0])) : 0;
  if (!(d > 0 && px > 0.5)) return;
  pushUndo(); S.wuPerPx = d / px; S.cal = []; setMode("trace"); changed();
};
let layoutEditing = false;   // one undo step per slider drag
function setLayout(pct) {
  if (!(pct > 0)) return;
  if (!layoutEditing) { pushUndo(); layoutEditing = true; }
  S.layout = clampLayout(pct / 100); changed();
}
$("layout").addEventListener("input", () => setLayout(+$("layout").value));
$("layout").addEventListener("change", () => { layoutEditing = false; });
$("layoutNum").addEventListener("change", () => { setLayout(+$("layoutNum").value); layoutEditing = false; $("layoutNum").value = Math.round(S.layout * 100); });
const newPath = (name, closed, extra) => ({ name, closed, start: null, head: null, secs: [], grades: [], wstops: [], ...extra });
$("addOpen").onclick = () => { pushUndo(); S.paths.push(newPath(`Path ${S.paths.length + 1}`, false)); S.active = S.paths.length - 1; setMode("trace"); changed(); };
$("addLoop").onclick = () => { pushUndo(); S.paths.push(newPath(S.paths.length ? `Loop ${S.paths.length + 1}` : "Main loop", true)); S.active = S.paths.length - 1; setMode("trace"); changed(); };
$("addStart").onclick = () => {
  pushUndo(); S.paths.push(newPath("Start straight", false, { taper: { w0: 20, w1: 4 } }));
  S.active = S.paths.length - 1; setKind("s"); setMode("trace"); changed();
};
$("clearAll").onclick = () => { if (!S.paths.length && !S.jumps.length) return; pushUndo(); S.jumps = []; S.jumpSel = null; S.paths = []; S.active = 0; S.sel = null; S.sample = false; changed(); $("msgProj").textContent = "Cleared. Undo brings the paths back."; };
$("piece").onchange = () => changed();
$("game").onchange = () => { S.game = KIT[$("game").value] ? $("game").value : "halo4"; fillPieceLists(); changed(); };
["lift", "tol", "slots", "fit", "showBlocks", "ox", "oy", "oz", "trackW", "borderKind"].forEach(id => $(id).addEventListener("input", () => { update(); draw(); }));

function loadImageFile(f) {
  if (!f || !/^image\//.test(f.type)) return;
  const url = URL.createObjectURL(f), im = new Image();
  im.onload = () => {
    pushUndo();
    if (S.sample) { S.paths = []; S.jumps = []; S.jumpSel = null; S.active = 0; S.imgRot = 0; }
    S.img = im; S.imgW = im.naturalWidth; S.imgH = im.naturalHeight; S.imgName = f.name; S.sample = false; S.sel = null;
    fit(); changed();
    $("msgProj").textContent = `Loaded ${f.name} (${S.imgW} × ${S.imgH}). Set the scale next.`;
  };
  im.src = url;
}
$("file").onchange = e => loadImageFile(e.target.files[0]);
board.addEventListener("dragover", e => { e.preventDefault(); board.classList.add("drop"); });
board.addEventListener("dragleave", () => board.classList.remove("drop"));
board.addEventListener("drop", e => { e.preventDefault(); board.classList.remove("drop"); loadImageFile(e.dataTransfer.files[0]); });

// ---- clipboard + files ----------------------------------------------------------------------
async function copy(text, msgEl, ta) {
  try { await navigator.clipboard.writeText(text); flash(msgEl, "Copied", true); }
  catch (e) { if (ta) { ta.focus(); ta.select(); } flash(msgEl, "Press Ctrl+C to copy the selected text", false); }
}
function flash(el, t, ok) { el.textContent = t; el.className = "msg " + (ok ? "ok" : "bad"); clearTimeout(el._t); el._t = setTimeout(() => { el.textContent = ""; }, 4000); }
let downloads = null;   // claude.ai's download capability; stays null when the page is opened as a normal file
function browserDownload(filename, data) {
  const url = URL.createObjectURL(new Blob([data], { type: "application/octet-stream" }));
  const a = document.createElement("a");
  a.href = url; a.download = filename; a.hidden = true;
  document.body.append(a); a.click(); a.remove();
  setTimeout(() => URL.revokeObjectURL(url), 10000);
}
async function saveFile(filename, data, msgEl) {
  if (!downloads) { browserDownload(filename, data); flash(msgEl, `Downloaded ${filename}`, true); return; }
  try { await downloads.save({ filename, data }); flash(msgEl, `Saved ${filename}`, true); }
  catch (e) { flash(msgEl, e && e.code === "declined" ? "Save cancelled" : "Could not save the file", false); }
}
$("copyPrefab").onclick = () => copy($("prefabCmd").value, $("msgPrefab"), $("prefabCmd"));
$("dlRef").onclick = async () => {
  try { const glb = await referenceGlb(); if (glb) saveFile(`${trackName()}-reference.glb`, glb, $("msgRef")); }
  catch (e) { flash($("msgRef"), "Could not build the model from this image", false); }
};
$("dlPrefab").onclick = () => {
  update();   // plan from what is on screen right now
  const pl = planCache; if (!pl) return;
  const name = `${trackName()}-${stamp()}`;
  lastPrefab = name; lastPrefabSig = prefabSig(pl);
  saveFile(`${name}.hmsprefab`, prefabJson(pl, name), $("msgPrefab"));
  update();
};
$("dlProj").onclick = () => saveFile(`${trackName()}-tracing.json`, JSON.stringify({ format: "track-tracer", version: 2, game: S.game, imgName: S.imgName || "", imgW: S.imgW, imgH: S.imgH, imgRot: S.imgRot, wuPerPx: S.wuPerPx, layout: S.layout, paths: S.paths, jumps: S.jumps }, null, 1), $("msgProj"));
$("projFile").onchange = e => {
  const f = e.target.files[0]; if (!f) return;
  f.text().then(t => {
    const o = JSON.parse(t);
    if (o.format !== "track-tracer" || !Array.isArray(o.paths)) throw new Error("not a tracing file");
    pushUndo(); S.paths = o.paths.map(cleanPath);
    S.wuPerPx = +o.wuPerPx > 0 ? +o.wuPerPx : S.wuPerPx; S.layout = clampLayout(o.layout); S.imgRot = clampRot(o.imgRot); S.jumps = cleanJumps(o.jumps); S.jumpSel = null;
    if (KIT[o.game]) { S.game = o.game; fillPieceLists(); }
    if (!S.img || S.sample) { S.imgW = +o.imgW || S.imgW; S.imgH = +o.imgH || S.imgH; }
    S.active = 0; S.sel = null; changed(); fit();
    const old = S.paths.some(p => p.guide && !p.secs.length);
    flash($("msgProj"), `Opened ${f.name}${old ? " — its old point tracing is shown dashed; trace over it with sections" : ""}${o.imgName ? ` — load ${o.imgName} to see it under the paths` : ""}`, true);
  }).catch(err => flash($("msgProj"), `Could not open ${f.name}: ${err.message}`, false));
};

// ============================================================================================
// Boot
// ============================================================================================
(function boot() {
  let restored = null;
  try { restored = JSON.parse(localStorage.getItem("trackTracer.v1") || "null"); } catch (e) {}
  fillJumpSelect($("jumpSize"), "whoop");
  if (restored && KIT[restored.game]) S.game = restored.game;
  fillPieceLists();
  if (restored && Array.isArray(restored.paths) && restored.paths.length && !restored.sample) {
    S.paths = restored.paths.map(cleanPath); S.active = Math.min(restored.active || 0, S.paths.length - 1); S.wuPerPx = restored.wuPerPx || S.wuPerPx; S.layout = clampLayout(restored.layout); S.imgRot = clampRot(restored.imgRot); S.jumps = cleanJumps(restored.jumps);
    S.imgW = restored.imgW || 1024; S.imgH = restored.imgH || 768; S.sample = false;
    S.img = null;
    $("msgProj").textContent = S.paths.some(p => p.guide && !p.secs.length)
      ? "Restored your last tracing. Its old point tracing is shown dashed: trace over it with straights and turns. Load the image again to see it underneath."
      : "Restored your last tracing. Load the image again to see it underneath.";
  } else {
    S.paths = samplePaths(); S.img = drawSampleImage(); S.img.onload = draw;
  }
  new ResizeObserver(() => { resize(); if (!S.viewTouched) fit(); }).observe(board);
  try { const t = JSON.parse(localStorage.getItem("trackTracer.tool") || "null"); if (t) { S.kind = t.kind === "t" || t.kind === "p" ? t.kind : "s"; S.turnMode = t.turnMode === "free" ? "free" : "set"; } } catch (e) {}
  // panels remember whether they are open
  let panels = {}; try { panels = JSON.parse(localStorage.getItem("trackTracer.panels") || "{}") || {}; } catch (e) {}
  document.querySelectorAll("details.panel").forEach(d => {
    if (typeof panels[d.dataset.key] === "boolean") d.open = panels[d.dataset.key];
    d.addEventListener("toggle", () => { panels[d.dataset.key] = d.open; try { localStorage.setItem("trackTracer.panels", JSON.stringify(panels)); } catch (e) {} });
  });
  setMode("trace"); setKind(); markPreset(); renderPaths(); update(); fit();
  if (window.claude && window.claude.use) window.claude.use("downloads").then(d => { downloads = d; if (!d) { $("dlRef").hidden = true; $("dlProj").hidden = true; $("dlPrefab").textContent = "Copy prefab JSON"; $("dlPrefab").onclick = () => { const j = planCache && prefabJson(planCache); if (j) copy(j, $("msgPrefab")); }; } }).catch(() => {});
})();
window.TT = { plan, planPath, pathGeom, posesOf, cleanPath, KIT, Arc, edgeGap, makeProfile, gradeRanges, maxLip, prefabJson: () => planCache && prefabJson(planCache), state: S };
