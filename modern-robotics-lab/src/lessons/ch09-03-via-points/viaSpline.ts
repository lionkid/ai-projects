/**
 * Via-point trajectories in the plane (book §9.3).
 *
 * The piecewise cubic itself comes from `viaPointCubic` in `@/mr`; this module adds what the
 * lesson needs around it: user-defined via velocities, a global C² cubic spline and quintic
 * segments for comparison, a single high-degree polynomial (to show why one is not enough),
 * overshoot detection, and inverse kinematics of a 2R arm that follows the path.
 */
import { viaPointCubic, planar2RIK, solve, wrapToPi, type ViaPointSpline } from '@/mr';

export type Pt2 = [number, number];
export type VelStrategy = 'zero' | 'heuristic' | 'custom';

/** State at time t. */
export interface Sample { q: Pt2; qd: Pt2; qdd: Pt2 }

/** Anything that can be evaluated over time (the cubic spline and the comparison curves). */
export interface Trajectory2 {
  times: number[];
  evaluate(t: number): Sample;
}

const toPt = (v: number[]): Pt2 => [v[0], v[1]];

/** Piecewise cubic through the via points. Custom velocities are forced to zero at both ends. */
export function buildSpline(points: Pt2[], times: number[], strategy: VelStrategy, custom?: Pt2[]): ViaPointSpline {
  if (strategy === 'custom') {
    const K = points.length;
    const v = points.map((_, k) => (k === 0 || k === K - 1 || !custom?.[k] ? [0, 0] : [custom[k][0], custom[k][1]]));
    return viaPointCubic(points, times, v);
  }
  return viaPointCubic(points, times, strategy);
}

/** Wrap an `@/mr` spline as a Trajectory2. */
export function asTrajectory(sp: ViaPointSpline): Trajectory2 {
  return {
    times: sp.times,
    evaluate(t) {
      const r = sp.evaluate(t);
      return { q: toPt(r.q), qd: toPt(r.qdot), qdd: toPt(r.qddot) };
    },
  };
}

/**
 * Interior via velocities of the C² ("natural-in-the-middle, clamped at the ends") cubic spline:
 * zero velocity at both ends, acceleration continuous at every interior via point. Per coordinate
 *   v_{k-1}/h_{k-1} + 2 v_k (1/h_{k-1} + 1/h_k) + v_{k+1}/h_k = 3 (Δ_{k-1}/h_{k-1}² + Δ_k/h_k²),
 * a tridiagonal system solved with the Thomas algorithm.
 */
export function c2SplineVelocities(points: Pt2[], times: number[]): Pt2[] {
  const K = points.length;
  const vel: Pt2[] = points.map(() => [0, 0]);
  const n = K - 2;
  if (n <= 0) return vel;
  for (let j = 0; j < 2; j++) {
    const a = new Array(n).fill(0), b = new Array(n).fill(0), c = new Array(n).fill(0), d = new Array(n).fill(0);
    for (let i = 0; i < n; i++) {
      const k = i + 1;
      const h0 = times[k] - times[k - 1], h1 = times[k + 1] - times[k];
      const d0 = points[k][j] - points[k - 1][j], d1 = points[k + 1][j] - points[k][j];
      a[i] = 1 / h0; b[i] = 2 * (1 / h0 + 1 / h1); c[i] = 1 / h1;
      d[i] = 3 * (d0 / (h0 * h0) + d1 / (h1 * h1));
    }
    // Thomas algorithm (end velocities are zero, so the first/last a, c terms drop out).
    for (let i = 1; i < n; i++) {
      const w = a[i] / b[i - 1];
      b[i] -= w * c[i - 1];
      d[i] -= w * d[i - 1];
    }
    const x = new Array(n).fill(0);
    x[n - 1] = d[n - 1] / b[n - 1];
    for (let i = n - 2; i >= 0; i--) x[i] = (d[i] - c[i] * x[i + 1]) / b[i];
    for (let i = 0; i < n; i++) vel[i + 1][j] = x[i];
  }
  return vel;
}

/** Acceleration just before (left) and just after (right) every via point of a cubic spline. */
export function viaAccelerations(sp: ViaPointSpline): { left: Pt2[]; right: Pt2[] } {
  const K = sp.times.length;
  const left: Pt2[] = [], right: Pt2[] = [];
  for (let k = 0; k < K; k++) {
    const r: Pt2 = [0, 0], l: Pt2 = [0, 0];
    for (let j = 0; j < 2; j++) {
      if (k < K - 1) { const [, , a2] = sp.coefficients[k][j]; r[j] = 2 * a2; }
      if (k > 0) {
        const [, , a2, a3] = sp.coefficients[k - 1][j];
        const T = sp.times[k] - sp.times[k - 1];
        l[j] = 2 * a2 + 6 * a3 * T;
      }
    }
    left.push(l); right.push(r);
  }
  // Before the start and after the end the robot is at rest.
  left[0] = [0, 0]; right[K - 1] = [0, 0];
  return { left, right };
}

/** Acceleration jumps right − left at every via point (including the start and the end). */
export function accelJumps(sp: ViaPointSpline): { t: number; jump: Pt2; left: Pt2; right: Pt2 }[] {
  const { left, right } = viaAccelerations(sp);
  return sp.times.map((t, k) => ({ t, left: left[k], right: right[k], jump: [right[k][0] - left[k][0], right[k][1] - left[k][1]] as Pt2 }));
}

/** Coefficients c0..c5 (in τ from the segment start) of the quintic with the given end conditions. */
export function quinticCoefficients(q0: number, v0: number, a0: number, q1: number, v1: number, a1: number, T: number): number[] {
  const c0 = q0, c1 = v0, c2 = a0 / 2;
  const D = q1 - c0 - c1 * T - c2 * T * T;
  const Dv = v1 - c1 - 2 * c2 * T;
  const Da = a1 - 2 * c2;
  const T2 = T * T, T3 = T2 * T;
  return [
    c0, c1, c2,
    (10 * D - 4 * Dv * T + 0.5 * Da * T2) / T3,
    (-15 * D + 7 * Dv * T - Da * T2) / (T3 * T),
    (6 * D - 3 * Dv * T + 0.5 * Da * T2) / (T3 * T2),
  ];
}

/** Find the segment containing t (clamped). */
function segmentOf(times: number[], t: number): { k: number; tau: number } {
  const K = times.length;
  const tc = Math.max(times[0], Math.min(times[K - 1], t));
  let k = 0;
  while (k < K - 2 && tc > times[k + 1]) k++;
  return { k, tau: tc - times[k] };
}

/**
 * Quintic segments through the via points with the given via velocities and accelerations: position,
 * velocity AND acceleration are continuous by construction.
 */
export function quinticTrajectory(points: Pt2[], times: number[], vel: Pt2[], acc: Pt2[]): Trajectory2 {
  const coeffs: number[][][] = [];
  for (let k = 0; k < points.length - 1; k++) {
    const T = times[k + 1] - times[k];
    coeffs.push([0, 1].map((j) => quinticCoefficients(points[k][j], vel[k][j], acc[k][j], points[k + 1][j], vel[k + 1][j], acc[k + 1][j], T)));
  }
  return {
    times: times.slice(),
    evaluate(t) {
      const { k, tau } = segmentOf(times, t);
      const out: Sample = { q: [0, 0], qd: [0, 0], qdd: [0, 0] };
      for (let j = 0; j < 2; j++) {
        const c = coeffs[k][j];
        let q = 0, qd = 0, qdd = 0;
        for (let p = 5; p >= 0; p--) q = q * tau + c[p];
        for (let p = 5; p >= 1; p--) qd = qd * tau + p * c[p];
        for (let p = 5; p >= 2; p--) qdd = qdd * tau + p * (p - 1) * c[p];
        out.q[j] = q; out.qd[j] = qd; out.qdd[j] = qdd;
      }
      return out;
    },
  };
}

/**
 * Quintic comparison that keeps the via velocities of the given cubic spline and uses the mean of
 * its left/right accelerations at interior via points (zero at both ends).
 */
export function quinticFromCubic(sp: ViaPointSpline): Trajectory2 {
  const { left, right } = viaAccelerations(sp);
  const K = sp.times.length;
  const acc: Pt2[] = sp.times.map((_, k) => (k === 0 || k === K - 1 ? [0, 0] : [0.5 * (left[k][0] + right[k][0]), 0.5 * (left[k][1] + right[k][1])]));
  return quinticTrajectory(sp.points.map(toPt), sp.times, sp.velocities.map(toPt), acc);
}

/**
 * A single polynomial through all via points (degree K + 1) with zero velocity at both ends,
 * written in normalised time u = (t − t_0)/(t_end − t_0) for conditioning.
 */
export function globalPolynomial(points: Pt2[], times: number[]): Trajectory2 {
  const K = points.length;
  const t0 = times[0], span = times[K - 1] - times[0];
  const n = K + 2; // number of coefficients
  const A: number[][] = [];
  for (let k = 0; k < K; k++) {
    const u = (times[k] - t0) / span;
    A.push(Array.from({ length: n }, (_, p) => u ** p));
  }
  A.push(Array.from({ length: n }, (_, p) => (p === 1 ? 1 : 0)));
  A.push(Array.from({ length: n }, (_, p) => (p === 0 ? 0 : p)));
  const coeffs = [0, 1].map((j) => solve(A, [...points.map((p) => p[j]), 0, 0]));
  return {
    times: times.slice(),
    evaluate(t) {
      const u = (Math.max(t0, Math.min(times[K - 1], t)) - t0) / span;
      const out: Sample = { q: [0, 0], qd: [0, 0], qdd: [0, 0] };
      for (let j = 0; j < 2; j++) {
        const c = coeffs[j];
        let q = 0, qd = 0, qdd = 0;
        for (let p = n - 1; p >= 0; p--) q = q * u + c[p];
        for (let p = n - 1; p >= 1; p--) qd = qd * u + p * c[p];
        for (let p = n - 1; p >= 2; p--) qdd = qdd * u + p * (p - 1) * c[p];
        out.q[j] = q; out.qd[j] = qd / span; out.qdd[j] = qdd / (span * span);
      }
      return out;
    },
  };
}

export interface Overshoot {
  seg: number;
  coord: 0 | 1;
  /** Absolute time of the extreme value. */
  t: number;
  /** Position of the curve at that time. */
  q: Pt2;
  /** How far the coordinate leaves the range spanned by the segment's two end points. */
  excess: number;
}

/**
 * Overshoot: inside a segment, a coordinate goes beyond the interval spanned by that segment's
 * two via points (it has an interior extremum outside [min, max]). Reported when the excess is
 * larger than `tol`.
 */
export function findOvershoots(sp: ViaPointSpline, tol = 0.03): Overshoot[] {
  const out: Overshoot[] = [];
  for (let k = 0; k < sp.coefficients.length; k++) {
    const T = sp.times[k + 1] - sp.times[k];
    for (let j = 0 as 0 | 1; j < 2; j = (j + 1) as 0 | 1) {
      const [a0, a1, a2, a3] = sp.coefficients[k][j];
      const lo = Math.min(sp.points[k][j], sp.points[k + 1][j]);
      const hi = Math.max(sp.points[k][j], sp.points[k + 1][j]);
      // Interior critical points of the cubic: a1 + 2 a2 τ + 3 a3 τ² = 0.
      const roots: number[] = [];
      if (Math.abs(a3) < 1e-12) { if (Math.abs(a2) > 1e-12) roots.push(-a1 / (2 * a2)); } else {
        const disc = 4 * a2 * a2 - 12 * a3 * a1;
        if (disc >= 0) { const r = Math.sqrt(disc); roots.push((-2 * a2 + r) / (6 * a3), (-2 * a2 - r) / (6 * a3)); }
      }
      let best: { tau: number; excess: number } | null = null;
      for (const tau of roots) {
        if (!(tau > 1e-9 && tau < T - 1e-9)) continue;
        const q = a0 + a1 * tau + a2 * tau * tau + a3 * tau * tau * tau;
        const excess = Math.max(q - hi, lo - q);
        if (excess > tol && (!best || excess > best.excess)) best = { tau, excess };
      }
      if (best) {
        const t = sp.times[k] + best.tau;
        const r = sp.evaluate(t);
        out.push({ seg: k, coord: j, t, q: toPt(r.q), excess: best.excess });
      }
    }
  }
  return out;
}

/** Average (chord length / duration) and peak speed of segment k. */
export function segmentSpeeds(tr: Trajectory2, points: Pt2[], k: number, samples = 60): { avg: number; peak: number; tPeak: number } {
  const t0 = tr.times[k], t1 = tr.times[k + 1];
  const avg = Math.hypot(points[k + 1][0] - points[k][0], points[k + 1][1] - points[k][1]) / (t1 - t0);
  let peak = 0, tPeak = t0;
  for (let i = 0; i <= samples; i++) {
    const t = t0 + ((t1 - t0) * i) / samples;
    const v = tr.evaluate(t).qd;
    const sp = Math.hypot(v[0], v[1]);
    if (sp > peak) { peak = sp; tPeak = t; }
  }
  return { avg, peak, tPeak };
}

export interface IKTrack {
  thetas: Pt2[];
  reachable: boolean[];
}

/**
 * Joint angles of a planar 2R arm (base at the origin) whose tip follows the given points. At
 * every sample the IK branch closest to the previous solution is used, so the arm never flips
 * its elbow on the way; unreachable points are projected onto the workspace boundary.
 */
export function trackIK(pts: Pt2[], L1 = 1, L2 = 1, firstElbow: 'up' | 'down' = 'up'): IKTrack {
  const thetas: Pt2[] = [];
  const reachable: boolean[] = [];
  let prev: Pt2 | null = null;
  const rMax = L1 + L2 - 1e-6, rMin = Math.abs(L1 - L2) + 1e-6;
  for (const p of pts) {
    let [x, y] = p;
    const r = Math.hypot(x, y);
    let ok = true;
    if (r > rMax || r < rMin) {
      ok = false;
      const rc = Math.max(rMin, Math.min(rMax, r));
      const a = r > 1e-12 ? Math.atan2(y, x) : 0;
      x = rc * Math.cos(a); y = rc * Math.sin(a);
    }
    const sol = planar2RIK(L1, L2, x, y).solutions;
    let pick: Pt2;
    if (!sol.length) pick = prev ?? [Math.atan2(y, x), 0];
    else if (!prev) {
      const s = sol.find((q) => q.elbow === firstElbow) ?? sol[0];
      pick = [s.theta1, s.theta2];
    } else {
      const pv = prev;
      const cands: Pt2[] = sol.map((q) => [pv[0] + wrapToPi(q.theta1 - pv[0]), pv[1] + wrapToPi(q.theta2 - pv[1])]);
      cands.sort((u, v) => Math.hypot(u[0] - pv[0], u[1] - pv[1]) - Math.hypot(v[0] - pv[0], v[1] - pv[1]));
      pick = cands[0];
    }
    thetas.push(pick);
    reachable.push(ok);
    prev = pick;
  }
  return { thetas, reachable };
}

/** Clamp a dragged via time between its neighbours with a minimum segment duration. */
export function clampViaTime(times: number[], k: number, t: number, minGap = 0.15, tMax = 8): number {
  const lo = k > 0 ? times[k - 1] + minGap : 0;
  const hi = k < times.length - 1 ? times[k + 1] - minGap : tMax;
  return Math.max(lo, Math.min(hi, t));
}

/** Insert a via point in the middle (in time and along the chord) of segment k. */
export function insertVia(points: Pt2[], times: number[], k: number): { points: Pt2[]; times: number[] } {
  const p: Pt2 = [0.5 * (points[k][0] + points[k + 1][0]), 0.5 * (points[k][1] + points[k + 1][1])];
  const t = 0.5 * (times[k] + times[k + 1]);
  return { points: [...points.slice(0, k + 1), p, ...points.slice(k + 1)], times: [...times.slice(0, k + 1), t, ...times.slice(k + 1)] };
}

/** Remove via point k (keeps at least `minPoints`); times are shifted so that the first one stays 0. */
export function removeVia(points: Pt2[], times: number[], k: number, minPoints = 3): { points: Pt2[]; times: number[] } {
  if (points.length <= minPoints) return { points: points.slice(), times: times.slice() };
  const ts = times.filter((_, i) => i !== k);
  const t0 = ts[0];
  return { points: points.filter((_, i) => i !== k), times: ts.map((t) => t - t0) };
}

/** Centred-difference ("Catmull–Rom") via velocities: a reasonable starting point for custom arrows. */
export function centredVelocities(points: Pt2[], times: number[]): Pt2[] {
  const K = points.length;
  return points.map((_, k) => {
    if (k === 0 || k === K - 1) return [0, 0] as Pt2;
    const dt = times[k + 1] - times[k - 1];
    return [(points[k + 1][0] - points[k - 1][0]) / dt, (points[k + 1][1] - points[k - 1][1]) / dt] as Pt2;
  });
}
