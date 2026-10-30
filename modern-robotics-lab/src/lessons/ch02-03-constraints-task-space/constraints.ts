/**
 * Pure helpers for the constraints / task-space lesson (no DOM):
 *  - exact planar rigid-body integration for a constant body twist (unicycle and omnidirectional base),
 *  - the rolling constraint in Pfaffian form A(q) q̇ = 0 with q = (φ, x, y),
 *  - a four-move plan that produces a pure sideways displacement, and plan playback,
 *  - random-input "reachable clouds" for a nonholonomic unicycle and a holonomic point-on-a-rod,
 *  - a small 2R inverse-kinematics picker for the task-space vs workspace card.
 */
import { planar2RIK, wrapToPi } from '@/mr';

export interface Pose { x: number; y: number; phi: number }
export type V3 = [number, number, number];

/**
 * Exact pose update for a body twist held constant for h seconds: turn rate w, forward speed v and
 * sideways speed vs (vs = 0 for a unicycle). This is T ← T · exp([V_b] h) on SE(2), so long runs do not drift
 * the way an explicit Euler step would.
 */
export function bodyTwistStep(q: Pose, v: number, vs: number, w: number, h: number): Pose {
  const a = w * h;
  let dxb: number;
  let dyb: number;
  if (Math.abs(a) < 1e-9) {
    dxb = h * (v - (vs * a) / 2);
    dyb = h * (vs + (v * a) / 2);
  } else {
    const s = Math.sin(a);
    const c1 = 1 - Math.cos(a);
    dxb = (s * v - c1 * vs) / w;
    dyb = (c1 * v + s * vs) / w;
  }
  const c = Math.cos(q.phi);
  const sn = Math.sin(q.phi);
  return { x: q.x + c * dxb - sn * dyb, y: q.y + sn * dxb + c * dyb, phi: wrapToPi(q.phi + a) };
}

/** q̇ = (φ̇, ẋ, ẏ) produced by the body twist (w, v, vs) at heading φ. */
export function qdotOf(phi: number, v: number, vs: number, w: number): V3 {
  const c = Math.cos(phi);
  const s = Math.sin(phi);
  return [w, c * v - s * vs, s * v + c * vs];
}

/** The no-side-slip constraint ẋ sin φ − ẏ cos φ = 0 as a Pfaffian row A(q), q = (φ, x, y). */
export function pfaffianRow(phi: number): V3 {
  return [0, Math.sin(phi), -Math.cos(phi)];
}

export function constraintResidual(phi: number, qd: V3): number {
  const A = pfaffianRow(phi);
  return A[0] * qd[0] + A[1] * qd[1] + A[2] * qd[2];
}

/** Basis of the allowed velocities (null space of A(q)): g1 = drive along the heading, g2 = turn in place. */
export function allowedBasis(phi: number): { g1: V3; g2: V3 } {
  return { g1: [0, Math.cos(phi), Math.sin(phi)], g2: [1, 0, 0] };
}

// ---------------------------------------------------------------------------
// Plans (piecewise-constant inputs)
// ---------------------------------------------------------------------------

export interface Segment {
  v: number;
  vs?: number;
  w: number;
  /** Duration in seconds. */
  T: number;
  label: string;
}

const deg = (r: number) => Math.round((r * 180) / Math.PI);

/**
 * Turn in place by α, drive s = d / sin α, turn back, back up s cos α. Net motion: d to the left of the
 * heading (negative d: to the right), heading unchanged. Every segment respects the rolling constraint.
 */
export function lateralShiftPlan(d: number, alpha = Math.PI / 4, speed = 0.6, turnRate = 0.9): Segment[] {
  const sg = d < 0 ? -1 : 1;
  const s = Math.abs(d) / Math.sin(alpha);
  const back = s * Math.cos(alpha);
  return [
    { v: 0, w: sg * turnRate, T: alpha / turnRate, label: `原地${sg > 0 ? '左' : '右'}轉 ${deg(alpha)}°` },
    { v: speed, w: 0, T: s / speed, label: `前進 ${s.toFixed(2)} m` },
    { v: 0, w: -sg * turnRate, T: alpha / turnRate, label: `轉回 ${deg(alpha)}°` },
    { v: -speed, w: 0, T: back / speed, label: `後退 ${back.toFixed(2)} m` },
  ];
}

/** A plan that a sideways-moving (omnidirectional) base can use: slide directly. */
export function slidePlan(d: number, speed = 0.4): Segment[] {
  return [{ v: 0, vs: Math.sign(d) * speed, w: 0, T: Math.abs(d) / speed, label: `直接側移 ${Math.abs(d).toFixed(2)} m` }];
}

export function planDuration(plan: Segment[]): number {
  return plan.reduce((s, g) => s + g.T, 0);
}

/** Exact pose after following `plan` from `start` for t seconds (clamped to the plan's end). */
export function poseAlongPlan(start: Pose, plan: Segment[], t: number): { pose: Pose; seg: number; done: boolean } {
  let q = { ...start };
  let left = Math.max(0, t);
  for (let i = 0; i < plan.length; i++) {
    const g = plan[i];
    if (left < g.T) return { pose: bodyTwistStep(q, g.v, g.vs ?? 0, g.w, left), seg: i, done: false };
    q = bodyTwistStep(q, g.v, g.vs ?? 0, g.w, g.T);
    left -= g.T;
  }
  return { pose: q, seg: plan.length - 1, done: true };
}

/** The same motions in reverse order with negated inputs: following it undoes `plan`. */
export function reversePlan(plan: Segment[]): Segment[] {
  return plan.slice().reverse().map((g) => ({ ...g, v: -g.v, vs: -(g.vs ?? 0), w: -g.w }));
}

/** Sample the poses visited along a plan every `dt` seconds (including both ends). */
export function samplePlan(start: Pose, plan: Segment[], dt = 0.05): Pose[] {
  const T = planDuration(plan);
  const n = Math.max(1, Math.ceil(T / dt));
  const out: Pose[] = [];
  for (let k = 0; k <= n; k++) out.push(poseAlongPlan(start, plan, (T * k) / n).pose);
  return out;
}

// ---------------------------------------------------------------------------
// Tasks
// ---------------------------------------------------------------------------

export function poseError(q: Pose, target: Pose): { dist: number; dphi: number } {
  return { dist: Math.hypot(q.x - target.x, q.y - target.y), dphi: wrapToPi(q.phi - target.phi) };
}

/** Parking succeeds only when position AND heading (wrapped) are both within tolerance. */
export function isParked(q: Pose, target: Pose, tolPos = 0.1, tolAng = (6 * Math.PI) / 180): boolean {
  const e = poseError(q, target);
  return e.dist <= tolPos && Math.abs(e.dphi) <= tolAng;
}

// ---------------------------------------------------------------------------
// Random rollouts (deterministic RNG so the picture is reproducible)
// ---------------------------------------------------------------------------

export function mulberry32(seed: number): () => number {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

/** End poses of n unicycle rollouts with random piecewise-constant (v, ω). */
export function reachableCloud(start: Pose, n: number, rand: () => number, opts: { segments?: number; T?: number; vmax?: number; wmax?: number } = {}): Pose[] {
  const segs = opts.segments ?? 3;
  const T = opts.T ?? 1;
  const vmax = opts.vmax ?? 0.8;
  const wmax = opts.wmax ?? 1.6;
  const out: Pose[] = [];
  for (let k = 0; k < n; k++) {
    let q = { ...start };
    for (let s = 0; s < segs; s++) q = bodyTwistStep(q, (2 * rand() - 1) * vmax, 0, (2 * rand() - 1) * wmax, T);
    out.push(q);
  }
  return out;
}

/**
 * End points of n rollouts of a point held at distance L from a pivot (holonomic constraint
 * x² + y² = L²): any commanded velocity only moves it along the circle, so every end point is on it.
 */
export function holonomicCloud(L: number, psi0: number, n: number, rand: () => number, opts: { segments?: number; T?: number; umax?: number } = {}): [number, number][] {
  const segs = opts.segments ?? 3;
  const T = opts.T ?? 1;
  const umax = opts.umax ?? 1;
  const out: [number, number][] = [];
  for (let k = 0; k < n; k++) {
    let psi = psi0;
    for (let s = 0; s < segs; s++) {
      // random planar velocity command; only its tangential component survives the constraint
      const ux = (2 * rand() - 1) * umax;
      const uy = (2 * rand() - 1) * umax;
      const tangential = -Math.sin(psi) * ux + Math.cos(psi) * uy;
      psi += (tangential * T) / L;
    }
    out.push([L * Math.cos(psi), L * Math.sin(psi)]);
  }
  return out;
}

// ---------------------------------------------------------------------------
// Task space vs workspace (planar 2R drawing on a sheet of paper)
// ---------------------------------------------------------------------------

export function inAnnulus(L1: number, L2: number, p: [number, number]): boolean {
  const r = Math.hypot(p[0], p[1]);
  return r >= Math.abs(L1 - L2) - 1e-12 && r <= L1 + L2 + 1e-12;
}

/**
 * IK for the pen: if the target is reachable, the solution closest to `current`; otherwise the arm points at
 * the target with the tip on the nearest workspace boundary (reachable = false).
 */
export function penIK(L1: number, L2: number, target: [number, number], current: [number, number]): { th: [number, number]; reachable: boolean } {
  let [x, y] = target;
  const r = Math.hypot(x, y);
  const reachable = inAnnulus(L1, L2, target);
  if (!reachable) {
    const rr = r > L1 + L2 ? (L1 + L2) * (1 - 1e-9) : Math.abs(L1 - L2) * (1 + 1e-9);
    const f = r > 1e-12 ? rr / r : 0;
    x = r > 1e-12 ? x * f : rr;
    y = r > 1e-12 ? y * f : 0;
  }
  const ik = planar2RIK(L1, L2, x, y);
  if (!ik.solutions.length) return { th: [current[0], current[1]], reachable: false };
  let best: [number, number] = [ik.solutions[0].theta1, ik.solutions[0].theta2];
  let bestD = Infinity;
  for (const s of ik.solutions) {
    const d = Math.hypot(wrapToPi(s.theta1 - current[0]), wrapToPi(s.theta2 - current[1]));
    if (d < bestD) { bestD = d; best = [s.theta1, s.theta2]; }
  }
  return { th: best, reachable };
}
