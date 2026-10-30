/**
 * Trajectory generation (book chapter 9): time scalings s(t) in [0, 1] and
 * point-to-point paths in joint space and in SE(3).
 *
 * Every time-scaling function takes `(Tf, t)` in that order, as the book's
 * code does (this includes `TrapezoidalTimeScaling(Tf, t, spec)`); `t` outside
 * [0, Tf] is clamped so that samples before the start or after the end stay at
 * rest. Point-to-point paths need N >= 2 samples and throw otherwise.
 */

import { type Mat, type Vec, clamp, matMul, matScale, vecAdd, vecScale, vecSub } from './linalg';
import { MatrixExp3, MatrixLog3, RotInv } from './so3';
import { MatrixExp6, MatrixLog6, RpToTrans, TransInv, TransToRp } from './se3';

export interface TimeScalingSample {
  s: number;
  sdot: number;
  sddot: number;
}

/** Cubic (third-order polynomial) time scaling: zero velocity at both ends. */
export function CubicTimeScaling(Tf: number, t: number): number {
  const x = clamp(t / Tf, 0, 1);
  return 3 * x * x - 2 * x * x * x;
}

/** Cubic time scaling with its first two derivatives. */
export function CubicTimeScalingDerivs(Tf: number, t: number): TimeScalingSample {
  const x = clamp(t / Tf, 0, 1);
  const inside = t >= 0 && t <= Tf;
  return {
    s: 3 * x * x - 2 * x * x * x,
    sdot: inside ? (6 * x - 6 * x * x) / Tf : 0,
    sddot: inside ? (6 - 12 * x) / (Tf * Tf) : 0,
  };
}

/** Quintic (fifth-order polynomial) time scaling: zero velocity AND acceleration at both ends. */
export function QuinticTimeScaling(Tf: number, t: number): number {
  const x = clamp(t / Tf, 0, 1);
  const x3 = x * x * x;
  return 10 * x3 - 15 * x3 * x + 6 * x3 * x * x;
}

/** Quintic time scaling with its first two derivatives. */
export function QuinticTimeScalingDerivs(Tf: number, t: number): TimeScalingSample {
  const x = clamp(t / Tf, 0, 1);
  const inside = t >= 0 && t <= Tf;
  const x2 = x * x;
  const x3 = x2 * x;
  return {
    s: 10 * x3 - 15 * x3 * x + 6 * x3 * x2,
    sdot: inside ? (30 * x2 - 60 * x3 + 30 * x2 * x2) / Tf : 0,
    sddot: inside ? (60 * x - 180 * x2 + 120 * x3) / (Tf * Tf) : 0,
  };
}

export interface TrapezoidParams {
  /** Total duration (s). */
  Tf: number;
  /** Cruise velocity sdot_max (1/s). */
  v: number;
  /** Acceleration magnitude (1/s^2). */
  a: number;
  /** Duration of the acceleration (and of the deceleration) phase: v / a. */
  tAccel: number;
  /** False when no trapezoid with the given numbers covers s = 0 -> 1. */
  valid: boolean;
  /** Human-readable reason when `valid` is false. */
  reason?: string;
}

/** Tolerance for accepting inputs that sit on a feasibility boundary up to rounding. */
const TRAPEZOID_TOL = 1e-12;

/**
 * Completes and validates a trapezoidal (constant-acceleration) profile from
 * any two of {Tf, v, a}. The constraints are
 *   v Tf > 1 (fast enough to arrive), v^2 / a <= 1 (there is room to reach v),
 * which for (Tf, v) read 1 < v Tf <= 2 and for (Tf, a) read a Tf^2 >= 4.
 * Invalid combinations are returned with `valid: false` instead of throwing so
 * that a UI can explain the problem.
 *
 * Inputs within 1e-12 of the triangular limit (v Tf = 2, a Tf^2 = 4, v^2 = a)
 * are accepted and snapped onto it, so a `valid` result always describes an
 * exactly feasible profile: finite numbers and a cruise phase of duration
 * Tf - 2 tAccel >= 0 (up to rounding).
 */
export function trapezoidParams(spec: { Tf?: number; v?: number; a?: number }): TrapezoidParams {
  const invalid = (reason: string): TrapezoidParams => ({
    Tf: spec.Tf ?? NaN,
    v: spec.v ?? NaN,
    a: spec.a ?? NaN,
    tAccel: NaN,
    valid: false,
    reason,
  });
  const given = [spec.Tf, spec.v, spec.a].filter((x) => x !== undefined).length;
  if (given !== 2) return invalid('exactly two of Tf, v, a must be given');
  let { Tf, v, a } = spec;
  if (Tf !== undefined && v !== undefined) {
    if (!(Tf > 0 && v > 0)) return invalid('Tf and v must be positive');
    if (v * Tf <= 1) return invalid(`v Tf = ${(v * Tf).toFixed(3)} must exceed 1 (otherwise the motion cannot complete)`);
    if (v * Tf > 2 + TRAPEZOID_TOL) return invalid(`v Tf = ${(v * Tf).toFixed(3)} must not exceed 2 (the profile would need to overshoot)`);
    if (v * Tf > 2) v = 2 / Tf; // just past the triangular limit by rounding: snap onto it
    a = (v * v) / (v * Tf - 1);
  } else if (Tf !== undefined && a !== undefined) {
    if (!(Tf > 0 && a > 0)) return invalid('Tf and a must be positive');
    if (a * Tf * Tf < 4 - TRAPEZOID_TOL) return invalid(`a Tf^2 = ${(a * Tf * Tf).toFixed(3)} must be at least 4 (too slow to arrive in time)`);
    if (a * Tf * Tf <= 4) {
      // On the triangular limit (or just below it by rounding): snap onto it. The
      // square root below has an infinite slope here, so it must not be evaluated.
      a = 4 / (Tf * Tf);
      v = 2 / Tf;
    } else {
      // v is the smaller root of v^2 - a Tf v + a = 0 (the one with v Tf < 2),
      // written in the cancellation-free form 2 / (Tf + sqrt(Tf^2 - 4/a)); the
      // clamp guards the discriminant against rounding right above the limit.
      v = 2 / (Tf + Math.sqrt(Math.max(0, Tf * Tf - 4 / a)));
    }
  } else if (v !== undefined && a !== undefined) {
    if (!(v > 0 && a > 0)) return invalid('v and a must be positive');
    if (v * v > a * (1 + TRAPEZOID_TOL)) return invalid(`v^2 / a = ${((v * v) / a).toFixed(3)} must not exceed 1 (v is never reached)`);
    if (v * v > a) a = v * v; // just past the triangular limit by rounding: snap onto it
    Tf = 1 / v + v / a;
  }
  const tAccel = (v as number) / (a as number);
  // Last line of defence: a profile is only `valid` when every number in it is finite.
  if (![Tf, v, a, tAccel].every((x) => Number.isFinite(x))) {
    return invalid('the profile could not be completed with finite numbers');
  }
  return { Tf: Tf as number, v: v as number, a: a as number, tAccel, valid: true };
}

/** The parameter of a trapezoid that is not fixed by its duration: cruise velocity or acceleration. */
export type TrapezoidSpec = { v: number } | { a: number };

/**
 * Trapezoidal time scaling s(t), sdot(t), sddot(t) for a motion of duration
 * `Tf` with either the cruise velocity `v` or the acceleration `a` given.
 * The argument order is `(Tf, t, spec)`, the same `(Tf, t)` convention as the
 * polynomial scalings and the custom `TimeScalingMethod` callbacks. Throws
 * when the parameters are infeasible (check with `trapezoidParams` first in
 * interactive code).
 */
export function TrapezoidalTimeScaling(Tf: number, t: number, spec: TrapezoidSpec): TimeScalingSample {
  const P = trapezoidParams({ Tf, ...spec });
  if (!P.valid) throw new Error(`TrapezoidalTimeScaling: ${P.reason}`);
  return trapezoidSample(t, P);
}

/**
 * Packages a trapezoid specification as a `TimeScalingMethod`, so that it can
 * be handed to JointTrajectory / ScrewTrajectory / CartesianTrajectory in
 * place of 3 or 5. The profile is completed for whatever `Tf` the path uses
 * and, like TrapezoidalTimeScaling, throws when that combination is
 * infeasible.
 */
export function trapezoidScaling(spec: TrapezoidSpec): (Tf: number, t: number) => number {
  let cached: TrapezoidParams | undefined;
  return (Tf, t) => {
    if (!cached || cached.Tf !== Tf) {
      const P = trapezoidParams({ Tf, ...spec });
      if (!P.valid) throw new Error(`trapezoidScaling: ${P.reason}`);
      cached = P;
    }
    return trapezoidSample(t, cached).s;
  };
}

/** Evaluates a validated trapezoid profile at time t. */
export function trapezoidSample(t: number, P: TrapezoidParams): TimeScalingSample {
  const { Tf, v, a, tAccel } = P;
  if (t <= 0) return { s: 0, sdot: 0, sddot: 0 };
  if (t >= Tf) return { s: 1, sdot: 0, sddot: 0 };
  if (t <= tAccel) return { s: 0.5 * a * t * t, sdot: a * t, sddot: a };
  if (t <= Tf - tAccel) return { s: v * t - (v * v) / (2 * a), sdot: v, sddot: 0 };
  const r = Tf - t;
  return { s: 1 - 0.5 * a * r * r, sdot: a * r, sddot: -a };
}

/**
 * How to scale time along a path: 3 = cubic, 5 = quintic, or any custom
 * function (Tf, t) -> s in [0, 1].
 */
export type TimeScalingMethod = 3 | 5 | ((Tf: number, t: number) => number);

function scaling(method: TimeScalingMethod): (Tf: number, t: number) => number {
  if (method === 3) return CubicTimeScaling;
  if (method === 5) return QuinticTimeScaling;
  return method;
}

/** Point-to-point paths divide Tf into N - 1 steps, so fewer than two samples is an error (not NaN). */
function checkSampleCount(fn: string, N: number): void {
  if (!(Number.isInteger(N) && N >= 2)) throw new Error(`${fn}: N must be an integer >= 2 (got ${N})`);
}

/**
 * Straight line in joint space from `thetastart` to `thetaend` sampled at N
 * points (N >= 2) that are equally spaced in time; returns N joint vectors.
 */
export function JointTrajectory(thetastart: Vec, thetaend: Vec, Tf: number, N: number, method: TimeScalingMethod): Vec[] {
  checkSampleCount('JointTrajectory', N);
  const s = scaling(method);
  const out: Vec[] = [];
  const delta = vecSub(thetaend, thetastart);
  for (let k = 0; k < N; k++) {
    const t = (Tf * k) / (N - 1);
    out.push(vecAdd(thetastart, vecScale(delta, s(Tf, t))));
  }
  return out;
}

/**
 * Screw-motion path in SE(3): X(s) = Xstart exp(log(Xstart^-1 Xend) s). The
 * frame follows a constant-pitch screw, so the origin generally does NOT move
 * along a straight line.
 */
export function ScrewTrajectory(Xstart: Mat, Xend: Mat, Tf: number, N: number, method: TimeScalingMethod): Mat[] {
  checkSampleCount('ScrewTrajectory', N);
  const s = scaling(method);
  const logRel = MatrixLog6(matMul(TransInv(Xstart), Xend));
  const out: Mat[] = [];
  for (let k = 0; k < N; k++) {
    const t = (Tf * k) / (N - 1);
    out.push(matMul(Xstart, MatrixExp6(matScale(logRel, s(Tf, t)))));
  }
  return out;
}

/**
 * "Decoupled" Cartesian path: the origin moves along a straight line while
 * the orientation follows the rotation geodesic R(s) = Rs exp(log(Rs^T Re) s).
 */
export function CartesianTrajectory(Xstart: Mat, Xend: Mat, Tf: number, N: number, method: TimeScalingMethod): Mat[] {
  checkSampleCount('CartesianTrajectory', N);
  const s = scaling(method);
  const [Rs, ps] = TransToRp(Xstart);
  const [Re, pe] = TransToRp(Xend);
  const logRel = MatrixLog3(matMul(RotInv(Rs), Re));
  const dp = vecSub(pe, ps);
  const out: Mat[] = [];
  for (let k = 0; k < N; k++) {
    const t = (Tf * k) / (N - 1);
    const sk = s(Tf, t);
    out.push(RpToTrans(matMul(Rs, MatrixExp3(matScale(logRel, sk))), vecAdd(ps, vecScale(dp, sk))));
  }
  return out;
}

// ---------------------------------------------------------------------------
// Via points: piecewise-cubic interpolation
// ---------------------------------------------------------------------------

/** Coefficients a0 + a1 τ + a2 τ² + a3 τ³ of one joint on one segment (τ measured from the segment start). */
export type CubicCoefficients = [number, number, number, number];

export interface ViaPointSpline {
  /** Via-point times (copy of the input). */
  times: number[];
  /** Via-point positions (copy of the input). */
  points: Vec[];
  /** Velocities actually used at each via point. */
  velocities: Vec[];
  /** coefficients[segment][joint]. */
  coefficients: CubicCoefficients[][];
  /** Position, velocity and acceleration at time t (clamped to the spline's time range). */
  evaluate(t: number): { q: Vec; qdot: Vec; qddot: Vec };
  /** N equally spaced samples over the whole duration. */
  sample(N: number): { t: number; q: Vec; qdot: Vec; qddot: Vec }[];
}

export type ViaVelocitySpec = 'zero' | 'heuristic' | Vec[];

/**
 * Piecewise cubic through via points `points[k]` at increasing `times[k]`.
 * Velocities at the via points are either all zero (`'zero'`, the default:
 * the robot pauses at every via point), chosen by the usual heuristic
 * (`'heuristic'`: the average of the adjacent segment slopes, or 0 when the
 * slopes change sign, and 0 at both ends) or given explicitly. Positions and
 * velocities are continuous; accelerations in general are not.
 */
export function viaPointCubic(points: Vec[], times: number[], velocities: ViaVelocitySpec = 'zero'): ViaPointSpline {
  const K = points.length;
  if (K < 2) throw new Error('viaPointCubic: at least two via points are required');
  if (times.length !== K) throw new Error('viaPointCubic: times and points must have the same length');
  for (let k = 1; k < K; k++) {
    if (!(times[k] > times[k - 1])) throw new Error('viaPointCubic: times must be strictly increasing');
  }
  const n = points[0].length;
  let vel: Vec[];
  if (velocities === 'zero') {
    vel = points.map(() => new Array(n).fill(0));
  } else if (velocities === 'heuristic') {
    vel = points.map((_, k) => {
      if (k === 0 || k === K - 1) return new Array(n).fill(0);
      const before = vecScale(vecSub(points[k], points[k - 1]), 1 / (times[k] - times[k - 1]));
      const after = vecScale(vecSub(points[k + 1], points[k]), 1 / (times[k + 1] - times[k]));
      return before.map((b, j) => (b * after[j] > 0 ? 0.5 * (b + after[j]) : 0));
    });
  } else {
    if (velocities.length !== K) throw new Error('viaPointCubic: one velocity vector per via point is required');
    vel = velocities.map((v) => v.slice());
  }
  const coefficients: CubicCoefficients[][] = [];
  for (let k = 0; k < K - 1; k++) {
    const T = times[k + 1] - times[k];
    const seg: CubicCoefficients[] = [];
    for (let j = 0; j < n; j++) {
      const q0 = points[k][j];
      const q1 = points[k + 1][j];
      const v0 = vel[k][j];
      const v1 = vel[k + 1][j];
      const a2 = (3 * (q1 - q0) - (2 * v0 + v1) * T) / (T * T);
      const a3 = (2 * (q0 - q1) + (v0 + v1) * T) / (T * T * T);
      seg.push([q0, v0, a2, a3]);
    }
    coefficients.push(seg);
  }
  const evaluate = (t: number) => {
    const tc = clamp(t, times[0], times[K - 1]);
    let k = 0;
    while (k < K - 2 && tc > times[k + 1]) k++;
    const tau = tc - times[k];
    const q: Vec = [];
    const qdot: Vec = [];
    const qddot: Vec = [];
    for (let j = 0; j < n; j++) {
      const [a0, a1, a2, a3] = coefficients[k][j];
      q.push(a0 + a1 * tau + a2 * tau * tau + a3 * tau * tau * tau);
      qdot.push(a1 + 2 * a2 * tau + 3 * a3 * tau * tau);
      qddot.push(2 * a2 + 6 * a3 * tau);
    }
    return { q, qdot, qddot };
  };
  const sample = (N: number) => {
    const out: { t: number; q: Vec; qdot: Vec; qddot: Vec }[] = [];
    for (let i = 0; i < N; i++) {
      const t = times[0] + ((times[K - 1] - times[0]) * i) / Math.max(1, N - 1);
      out.push({ t, ...evaluate(t) });
    }
    return out;
  };
  return { times: times.slice(), points: points.map((p) => p.slice()), velocities: vel, coefficients, evaluate, sample };
}
