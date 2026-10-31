/**
 * Pure helpers for the C-space topology lesson (no DOM / three.js here, so they are unit-testable):
 *  - the square -> cylinder -> torus "gluing" morph used by the 3D view,
 *  - wrap-aware handling of joint-angle paths (unwrapping, splitting at the ±π seams, densifying),
 *  - small planar-2R utilities (the mirror elbow solution, nearest inverse-kinematics branch).
 *
 * Angles are radians. A configuration is Th = [θ1, θ2]; "wrapped" means each angle is in [-π, π],
 * "unwrapped" means a continuous lift to R² (the universal cover of the torus).
 */
import { planar2RIK, wrapToPi } from '@/mr';

export type Th = [number, number];
export type V3 = [number, number, number];

export const PI = Math.PI;
export const TWO_PI = 2 * Math.PI;

/** Wrap to [-π, π] but leave values that are already inside (so +π stays +π for display). */
export function wrapAngle(x: number): number {
  return x >= -PI && x <= PI ? x : wrapToPi(x);
}

export function wrapTh(t: Th): Th {
  return [wrapAngle(t[0]), wrapAngle(t[1])];
}

/** Distance on the flat torus (shortest way round in each coordinate). */
export function torusDist(a: Th, b: Th): number {
  return Math.hypot(wrapToPi(a[0] - b[0]), wrapToPi(a[1] - b[1]));
}

export function smooth01(x: number): number {
  const t = Math.min(1, Math.max(0, x));
  return t * t * (3 - 2 * t);
}

/**
 * Split the glue progress g ∈ [0, 2] into the two bending parameters:
 * a ∈ [0, 1] rolls the θ2 direction into a tube (glues the top/bottom edges),
 * b ∈ [0, 1] bends the tube into a ring (glues the left/right edges).
 */
export function glueStages(g: number): { a: number; b: number } {
  return { a: smooth01(g), b: smooth01(g - 1) };
}

const EPS = 1e-6;

/**
 * Bending of the flat sheet into the torus.
 *
 * Flat sheet (a = b = 0): the rectangle x = R + r, y = R θ1, z = r θ2.
 * Stage a: the θ2 direction is bent into a circular arc of radius r/a (a = 1 closes the tube); this stage is
 * isometric (lengths along θ2 stay r·Δθ2).
 * Stage b: the tube's centre line is bent into an arc of radius R/b about the z axis (b = 1 closes the ring).
 * This stage necessarily stretches the outer side and compresses the inner side (a flat torus has no smooth
 * isometric embedding in R³), so it only preserves lengths along the centre line.
 * At a = b = 1 this is the standard torus ((R + r cos θ2) cos θ1, (R + r cos θ2) sin θ1, r sin θ2).
 * The 1 − cos terms are written as 2 sin²(·/2) so small a, b stay accurate.
 */
export function morphPoint(t1: number, t2: number, a: number, b: number, R: number, r: number): V3 {
  let d: number;
  let z: number;
  if (a < EPS) {
    d = r;
    z = r * t2;
  } else {
    const s = Math.sin((a * t2) / 2);
    d = r - ((2 * r) / a) * s * s;
    z = (r / a) * Math.sin(a * t2);
  }
  let x: number;
  let y: number;
  if (b < EPS) {
    x = R + d;
    y = R * t1;
  } else {
    const s = Math.sin((b * t1) / 2);
    const rho = R / b + d;
    x = R + d - rho * 2 * s * s;
    y = rho * Math.sin(b * t1);
  }
  return [x, y, z];
}

/** Unit surface normal of the morphed sheet (central differences; outward on the closed torus). */
export function morphNormal(t1: number, t2: number, a: number, b: number, R: number, r: number): V3 {
  const h = 1e-4;
  const p1 = morphPoint(t1 + h, t2, a, b, R, r);
  const m1 = morphPoint(t1 - h, t2, a, b, R, r);
  const p2 = morphPoint(t1, t2 + h, a, b, R, r);
  const m2 = morphPoint(t1, t2 - h, a, b, R, r);
  const du: V3 = [p1[0] - m1[0], p1[1] - m1[1], p1[2] - m1[2]];
  const dv: V3 = [p2[0] - m2[0], p2[1] - m2[1], p2[2] - m2[2]];
  const n: V3 = [du[1] * dv[2] - du[2] * dv[1], du[2] * dv[0] - du[0] * dv[2], du[0] * dv[1] - du[1] * dv[0]];
  const L = Math.hypot(n[0], n[1], n[2]) || 1;
  return [n[0] / L, n[1] / L, n[2] / L];
}

// ---------------------------------------------------------------------------
// Paths in angle space
// ---------------------------------------------------------------------------

/** Lift a sequence of wrapped samples to a continuous (unwrapped) sequence. */
export function unwrapSeq(wrapped: Th[]): Th[] {
  if (!wrapped.length) return [];
  const out: Th[] = [[wrapped[0][0], wrapped[0][1]]];
  for (let i = 1; i < wrapped.length; i++) {
    const prev = out[i - 1];
    const q = wrapped[i];
    out.push([prev[0] + wrapToPi(q[0] - wrapToPi(prev[0])), prev[1] + wrapToPi(q[1] - wrapToPi(prev[1]))]);
  }
  return out;
}

/** A trail of wrapped samples; `null` marks a break where the configuration jumped instead of moving. */
export type Trail = (Th | null)[];

/**
 * Append a wrapped sample to a trail (in place). An explicit jump, or a step longer than `jumpDist` on the torus,
 * starts a new piece so no motion is drawn between the two configurations; samples closer than `minStep` to the
 * previous one are skipped. At most `max` entries are kept and the trail never starts with a break.
 * Returns true when the trail changed.
 */
export function pushTrail(trail: Trail, w: Th, o: { minStep: number; max: number; jump?: boolean; jumpDist?: number }): boolean {
  const last = trail[trail.length - 1] ?? null;
  let changed = true;
  if (last && (o.jump || torusDist(last, w) > (o.jumpDist ?? 1))) trail.push(null, [w[0], w[1]]);
  else if (!last || torusDist(last, w) > o.minStep) trail.push([w[0], w[1]]);
  else changed = false;
  while (trail.length > o.max || (trail.length && trail[0] === null)) trail.shift();
  return changed;
}

/** The continuous pieces of a trail (pieces with fewer than two samples are dropped). */
export function trailPieces(trail: Trail): Th[][] {
  const out: Th[][] = [];
  let cur: Th[] = [];
  for (const p of trail) {
    if (p) { cur.push(p); continue; }
    if (cur.length > 1) out.push(cur);
    cur = [];
  }
  if (cur.length > 1) out.push(cur);
  return out;
}

/**
 * Samples for drawing a trail as dots on the 3D surface: each continuous piece is unwrapped, densified so that
 * neighbouring dots are at most `maxStep` apart (a fast drag must not leave gaps) and wrapped back into [-π, π]²
 * (the unglued sheet is not periodic). Breaks are never bridged; only the newest `max` samples are kept.
 */
export function trailSamples(trail: Trail, maxStep: number, max: number): Th[] {
  const out: Th[] = [];
  for (const piece of trailPieces(trail)) for (const p of densify(unwrapSeq(piece), maxStep)) out.push(wrapTh(p));
  return out.length > max ? out.slice(out.length - max) : out;
}

/** Append a wrapped sample to an unwrapped path (used while the learner draws). */
export function appendUnwrapped(path: Th[], wrapped: Th): Th {
  const prev = path[path.length - 1];
  const p: Th = prev
    ? [prev[0] + wrapToPi(wrapped[0] - wrapToPi(prev[0])), prev[1] + wrapToPi(wrapped[1] - wrapToPi(prev[1]))]
    : [wrapped[0], wrapped[1]];
  path.push(p);
  return p;
}

/** Insert points so that consecutive samples are at most `maxStep` apart (Euclidean in angle space). */
export function densify(pts: Th[], maxStep: number): Th[] {
  if (pts.length < 2) return pts.map((p) => [p[0], p[1]] as Th);
  const out: Th[] = [[pts[0][0], pts[0][1]]];
  for (let i = 1; i < pts.length; i++) {
    const a = pts[i - 1];
    const b = pts[i];
    const L = Math.hypot(b[0] - a[0], b[1] - a[1]);
    const n = Math.max(1, Math.ceil(L / maxStep));
    for (let k = 1; k <= n; k++) out.push([a[0] + ((b[0] - a[0]) * k) / n, a[1] + ((b[1] - a[1]) * k) / n]);
  }
  return out;
}

export function pathLength(pts: Th[]): number {
  let L = 0;
  for (let i = 1; i < pts.length; i++) L += Math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]);
  return L;
}

/** Point at arc-length fraction s ∈ [0, 1] along an (unwrapped) polyline. */
export function samplePath(pts: Th[], s: number): Th {
  if (!pts.length) return [0, 0];
  if (pts.length === 1 || s <= 0) return [pts[0][0], pts[0][1]];
  const total = pathLength(pts);
  let target = Math.min(1, s) * total;
  for (let i = 1; i < pts.length; i++) {
    const seg = Math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]);
    if (target <= seg || i === pts.length - 1) {
      const u = seg > 0 ? Math.min(1, target / seg) : 1;
      return [pts[i - 1][0] + (pts[i][0] - pts[i - 1][0]) * u, pts[i - 1][1] + (pts[i][1] - pts[i - 1][1]) * u];
    }
    target -= seg;
  }
  const last = pts[pts.length - 1];
  return [last[0], last[1]];
}

/** Index of the 2π-tile that contains x (tile k covers [-π + 2πk, π + 2πk)). */
function tile(x: number): number {
  return Math.floor((x + PI) / TWO_PI);
}

/**
 * Draw an unwrapped path on the square [-π, π]²: returns wrapped polylines, cut exactly where the path
 * crosses a seam (the crossing point is added on both sides, so each piece ends on the square's edge).
 */
export function wrapPolylines(unwrapped: Th[]): Th[][] {
  const out: Th[][] = [];
  if (unwrapped.length < 2) return out;
  let k: [number, number] = [tile(unwrapped[0][0]), tile(unwrapped[0][1])];
  const shift = (p: Th, kk: [number, number]): Th => [p[0] - TWO_PI * kk[0], p[1] - TWO_PI * kk[1]];
  let cur: Th[] = [shift(unwrapped[0], k)];
  for (let i = 1; i < unwrapped.length; i++) {
    const a = unwrapped[i - 1];
    const b = unwrapped[i];
    const kb: [number, number] = [tile(b[0]), tile(b[1])];
    // crossings of seam lines x = (2m+1)π inside this segment, per coordinate
    const events: { tau: number; c: 0 | 1; dir: number }[] = [];
    for (const c of [0, 1] as const) {
      if (kb[c] === k[c]) continue;
      const dir = Math.sign(kb[c] - k[c]);
      for (let m = k[c]; m !== kb[c]; m += dir) {
        const seam = dir > 0 ? PI + TWO_PI * m : -PI + TWO_PI * m;
        const tau = (seam - a[c]) / (b[c] - a[c]);
        events.push({ tau: Math.min(1, Math.max(0, tau)), c, dir });
      }
    }
    events.sort((e1, e2) => e1.tau - e2.tau);
    for (const e of events) {
      const pc: Th = [a[0] + (b[0] - a[0]) * e.tau, a[1] + (b[1] - a[1]) * e.tau];
      cur.push(shift(pc, k));
      if (cur.length > 1) out.push(cur);
      k = [k[0], k[1]];
      k[e.c] += e.dir;
      cur = [shift(pc, k)];
    }
    cur.push(shift(b, k));
  }
  if (cur.length > 1) out.push(cur);
  return out;
}

export interface PathPreset {
  label: string;
  title: string;
  /** Unwrapped polyline (vertices); densify before use. */
  pts: Th[];
  /** Suggested 3D view (camera azimuth / elevation in rad) that shows the interesting part of the path. */
  cam?: { az: number; el: number };
}

/** Preset paths in unwrapped coordinates. */
export const PATH_PRESETS: PathPreset[] = [
  { label: '直線穿越 θ₁ = ±π', title: '方格中的一條直線，中途越過右邊界', pts: [[1.9, -1.1], [4.4, 0.5]], cam: { az: PI - 0.3, el: 0.3 } },
  { label: 'θ₁ 繞一圈', title: 'θ₂ 固定、θ₁ 轉 360°', pts: [[-0.6, 0.9], [-0.6 + TWO_PI, 0.9]], cam: { az: -0.6, el: 1.0 } },
  { label: 'θ₂ 繞一圈', title: 'θ₁ 固定、θ₂ 轉 360°', pts: [[0.9, -0.4], [0.9, -0.4 + TWO_PI]], cam: { az: -0.25, el: 0.35 } },
  { label: '(1, 1) 斜繞', title: 'θ₁ 與 θ₂ 同時各轉一圈', pts: [[-1.2, -2.2], [-1.2 + TWO_PI, -2.2 + TWO_PI]], cam: { az: -0.75, el: 0.8 } },
];

// ---------------------------------------------------------------------------
// Planar 2R helpers
// ---------------------------------------------------------------------------

export interface OtherElbow {
  /** The mirror configuration that puts the tip at the same point (wrapped). */
  th: Th;
  /** θ2 ∈ {0, π}: the two solutions coincide (tip on a workspace boundary circle). */
  coincident: boolean;
  /** L1 = L2 and θ2 = π: the tip is at the base and θ1 is arbitrary (infinitely many solutions). */
  atOrigin: boolean;
}

/**
 * Second inverse-kinematics solution of a planar 2R arm (same tip, θ2 → −θ2).
 * With tip angle α = θ1 + atan2(L2 sin θ2, L1 + L2 cos θ2), the mirror has θ1' = θ1 + 2 atan2(L2 sin θ2, L1 + L2 cos θ2).
 */
export function otherElbow(L1: number, L2: number, th: Th): OtherElbow {
  const s2 = Math.sin(th[1]);
  const c2 = Math.cos(th[1]);
  const k = L1 + L2 * c2;
  const m = L2 * s2;
  const atOrigin = Math.hypot(k, m) < 1e-9;
  const coincident = Math.abs(s2) < 1e-9;
  const t1 = atOrigin ? th[0] : th[0] + 2 * Math.atan2(m, k);
  return { th: [wrapToPi(t1), wrapToPi(-th[1])], coincident, atOrigin };
}

export interface IKPick {
  th: Th;
  /** The target was outside the reachable annulus and was projected onto its boundary. */
  clamped: boolean;
  /** Target at the base of an L1 = L2 arm: θ1 is undetermined, so the current θ1 is kept. */
  singular: boolean;
}

/**
 * Inverse kinematics for dragging the tip: projects the target into the annulus |L1 − L2| ≤ r ≤ L1 + L2 and
 * returns the solution closest (on the torus) to the current configuration, so the elbow branch is preserved.
 */
export function ikNearest(L1: number, L2: number, target: [number, number], current: Th): IKPick {
  let [x, y] = target;
  let r = Math.hypot(x, y);
  const rMin = Math.abs(L1 - L2);
  const rMax = L1 + L2;
  let clamped = false;
  if (rMin < 1e-9 && r < 1e-6) return { th: [current[0], PI], clamped: false, singular: true };
  if (r < 1e-12) {
    // direction undefined: keep the current tip direction
    const phi = current[0] + Math.atan2(L2 * Math.sin(current[1]), L1 + L2 * Math.cos(current[1]));
    x = Math.cos(phi) * 1e-9;
    y = Math.sin(phi) * 1e-9;
    r = 1e-9;
  }
  if (r > rMax) {
    const f = (rMax * (1 - 1e-12)) / r;
    x *= f; y *= f; clamped = true;
  } else if (r < rMin) {
    const f = (rMin * (1 + 1e-12)) / r;
    x *= f; y *= f; clamped = true;
  }
  const ik = planar2RIK(L1, L2, x, y);
  if (!ik.solutions.length) return { th: [current[0], current[1]], clamped: true, singular: false };
  let best: Th = [ik.solutions[0].theta1, ik.solutions[0].theta2];
  let bestD = Infinity;
  for (const s of ik.solutions) {
    const cand: Th = [wrapToPi(s.theta1), wrapToPi(s.theta2)];
    const d = torusDist(cand, current);
    if (d < bestD) { bestD = d; best = cand; }
  }
  return { th: best, clamped, singular: false };
}

/** Implicit (embedded) coordinates of a 2R configuration and the two constraint residuals. */
export function implicitCoords(th: Th): { c1: number; s1: number; c2: number; s2: number; g1: number; g2: number } {
  const c1 = Math.cos(th[0]);
  const s1 = Math.sin(th[0]);
  const c2 = Math.cos(th[1]);
  const s2 = Math.sin(th[1]);
  return { c1, s1, c2, s2, g1: c1 * c1 + s1 * s1 - 1, g2: c2 * c2 + s2 * s2 - 1 };
}
