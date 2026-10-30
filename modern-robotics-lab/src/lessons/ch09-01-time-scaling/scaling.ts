/**
 * Time scalings s(t) for a rest-to-rest motion s: 0 -> 1 (pure logic, no DOM).
 *
 * - cubic / quintic polynomials come straight from @/mr (CubicTimeScalingDerivs, QuinticTimeScalingDerivs);
 * - the trapezoid is completed by @/mr trapezoidParams({ Tf, a }) and sampled with trapezoidSample;
 * - the jerk-limited "S-curve" (7 constant-jerk segments) is not in the toolkit, so it lives here;
 * - a flexible load (spring-mass behind the joint) is integrated with @/mr rk4Step to show residual vibration.
 *
 * Convention for the limited profiles (trapezoid, S-curve): the duration T is given and the profile uses the
 * full acceleration limit a_max; the cruise speed it needs follows from T. When that speed exceeds v_max (or no
 * profile fits at all), the request is infeasible and the minimum-time profile under the limits is returned
 * instead, flagged with `feasible: false` so the UI can explain the gap.
 */
import {
  CubicTimeScalingDerivs, QuinticTimeScalingDerivs, trapezoidParams, trapezoidSample, rk4Step,
  type TrapezoidParams,
} from '@/mr';

export type ScalingKind = 'cubic' | 'quintic' | 'trapezoid' | 'scurve';
export const KINDS: ScalingKind[] = ['cubic', 'quintic', 'trapezoid', 'scurve'];

export interface ScalingState {
  s: number;
  sd: number;
  sdd: number;
  /** Third derivative (finite part; impulses at `accelJumps` are not included). */
  jerk: number;
}

export interface Limits {
  vmax: number;
  amax: number;
  jmax: number;
}

export interface Profile {
  kind: ScalingKind;
  /** Duration actually used (s). Equals the requested T when `feasible`, otherwise the minimum time. */
  T: number;
  /** The duration the learner asked for. */
  Treq: number;
  /** False when no profile of this kind meets the limits within Treq (then T = Tmin > Treq). */
  feasible: boolean;
  /** Minimum duration under the limits (Infinity is never returned; cubic/quintic report 0 = no limits). */
  Tmin: number;
  /** Times where s̈ jumps, i.e. the jerk contains an impulse. */
  accelJumps: number[];
  /** Piece boundaries, used to sample discontinuities crisply. */
  breaks: number[];
  sample(t: number): ScalingState;
  /** Peak |ṡ|, |s̈| and |jerk| (jerk = Infinity when s̈ is discontinuous). */
  peak: { sd: number; sdd: number; jerk: number };
  /** Kind-specific details for the UI. */
  info: {
    /** Cruise speed ṡ (trapezoid / S-curve). */
    v?: number;
    /** Peak acceleration actually used. */
    a?: number;
    /** Acceleration phase duration (trapezoid) or jerk ramp duration (S-curve). */
    tAccel?: number;
    tJerk?: number;
    /** Constant-acceleration time per ramp (S-curve). */
    tConstA?: number;
    /** Cruise duration. */
    tCruise?: number;
    /** No cruise phase (trapezoid -> triangle). */
    triangle?: boolean;
    /** The S-curve does not reach a_max. */
    aSaturated?: boolean;
  };
}

const EPS_REL = 1e-9;

// ---------------------------------------------------------------------------
// Polynomials (from @/mr) + their jerk
// ---------------------------------------------------------------------------

export function cubicProfile(T: number): Profile {
  const sample = (t: number): ScalingState => {
    const d = CubicTimeScalingDerivs(T, t);
    const inside = t >= 0 && t <= T;
    return { s: d.s, sd: d.sdot, sdd: d.sddot, jerk: inside ? -12 / (T * T * T) : 0 };
  };
  return {
    kind: 'cubic', T, Treq: T, feasible: true, Tmin: 0,
    accelJumps: [0, T], breaks: [0, T], sample,
    peak: { sd: 1.5 / T, sdd: 6 / (T * T), jerk: Infinity },
    info: {},
  };
}

export function quinticProfile(T: number): Profile {
  const sample = (t: number): ScalingState => {
    const d = QuinticTimeScalingDerivs(T, t);
    const inside = t >= 0 && t <= T;
    const x = Math.min(1, Math.max(0, t / T));
    return { s: d.s, sd: d.sdot, sdd: d.sddot, jerk: inside ? (60 - 360 * x + 360 * x * x) / (T * T * T) : 0 };
  };
  // max s̈ at x = (1 - 1/sqrt(3)) / 2: 10 / sqrt(3) / T^2 ≈ 5.774 / T^2; max |jerk| = 60 / T^3 at the ends.
  return {
    kind: 'quintic', T, Treq: T, feasible: true, Tmin: 0,
    accelJumps: [], breaks: [0, T], sample,
    peak: { sd: 1.875 / T, sdd: 10 / Math.sqrt(3) / (T * T), jerk: 60 / (T * T * T) },
    info: {},
  };
}

// ---------------------------------------------------------------------------
// Trapezoid
// ---------------------------------------------------------------------------

/**
 * Minimum duration of a trapezoid under ṡ ≤ vmax, |s̈| ≤ amax. With v²/a ≤ 1 the cruise speed is reached
 * and T = 1/v + v/a = (a + v²)/(va); otherwise v is never reached and the fastest profile is a triangle
 * (bang-bang) with T = 2/√a and peak speed √a.
 */
export function trapezoidMinTime(vmax: number, amax: number): { T: number; triangle: boolean; vPeak: number } {
  if (vmax * vmax <= amax) {
    const P = trapezoidParams({ v: vmax, a: amax });
    return { T: P.Tf, triangle: false, vPeak: vmax };
  }
  return { T: 2 / Math.sqrt(amax), triangle: true, vPeak: Math.sqrt(amax) };
}

export function trapezoidProfile(Treq: number, lim: Limits): Profile {
  const min = trapezoidMinTime(lim.vmax, lim.amax);
  const asked = trapezoidParams({ Tf: Treq, a: lim.amax });
  const feasible = asked.valid && asked.v <= lim.vmax * (1 + EPS_REL);
  let P: TrapezoidParams;
  if (feasible) P = asked;
  else if (min.triangle) P = trapezoidParams({ Tf: min.T, a: lim.amax });
  else P = trapezoidParams({ v: lim.vmax, a: lim.amax });
  const T = P.Tf;
  const ta = P.tAccel;
  // Near the triangular limit the square root inside trapezoidParams turns rounding into ~1e-8 T of cruise.
  const triangle = T - 2 * ta <= 1e-6 * T;
  const tCruise = triangle ? 0 : T - 2 * ta;
  const jumps = triangle ? [0, T / 2, T] : [0, ta, T - ta, T];
  return {
    kind: 'trapezoid', T, Treq, feasible, Tmin: min.T,
    accelJumps: jumps, breaks: jumps,
    sample: (t) => { const r = trapezoidSample(t, P); return { s: r.s, sd: r.sdot, sdd: r.sddot, jerk: 0 }; },
    peak: { sd: P.v, sdd: P.a, jerk: Infinity },
    info: { v: P.v, a: P.a, tAccel: ta, tCruise, triangle },
  };
}

// ---------------------------------------------------------------------------
// Jerk-limited S-curve (7 constant-jerk segments)
// ---------------------------------------------------------------------------

export interface SCurveParams {
  T: number;
  /** Jerk ramp duration (s). */
  tj: number;
  /** Constant-acceleration duration inside each acceleration phase (s). */
  tc: number;
  /** Cruise duration (s). */
  tv: number;
  /** Peak acceleration (≤ amax). */
  A: number;
  /** Cruise speed. */
  V: number;
  /** Jerk magnitude used on the ramps. */
  J: number;
  /** True when the peak acceleration equals amax (a constant-acceleration part exists). */
  saturated: boolean;
}

/**
 * The S-curve of duration T that uses jerk ±jmax on its ramps and accelerates as hard as allowed (up to amax),
 * then cruises at the lowest speed that still arrives at s = 1 at time T. Returns null when no such profile
 * exists (T too short for these limits). Two cases:
 *  - unsaturated: the acceleration is a triangle (tc = 0): J tj² (T − 2 tj) = 1, peak J tj ≤ amax;
 *  - saturated: tj = amax / J and u = tj + tc solves amax u (T − tj − u) = 1 (smaller root = longer cruise).
 */
export function scurveForT(T: number, amax: number, jmax: number): SCurveParams | null {
  const J = jmax;
  if (!(T > 0 && amax > 0 && J > 0)) return null;
  // The fastest jerk-limited move (no cruise, no saturation) lasts (32 / J)^(1/3).
  if (J * T * T * T < 32 * (1 - 1e-12)) return null;
  // Unsaturated case: f(tj) = J tj^2 (T - 2 tj) is increasing on (0, T/3); the root lies in (0, T/4].
  const f = (tj: number) => J * tj * tj * (T - 2 * tj);
  let lo = 0, hi = T / 4;
  for (let i = 0; i < 100; i++) {
    const mid = 0.5 * (lo + hi);
    if (f(mid) < 1) lo = mid; else hi = mid;
  }
  const tjB = hi;
  if (J * tjB <= amax * (1 + 1e-12)) {
    return { T, tj: tjB, tc: 0, tv: Math.max(0, T - 4 * tjB), A: J * tjB, V: J * tjB * tjB, J, saturated: false };
  }
  // Saturated case.
  const tj = amax / J;
  const b = T - tj;
  const disc = b * b - 4 / amax;
  if (disc < 0) return null;
  // Smaller root of u^2 - b u + 1/amax = 0, written without cancellation.
  const u = 2 / (amax * (b + Math.sqrt(disc)));
  const tc = u - tj;
  const tv = T - 2 * tj - 2 * u;
  if (tc < -1e-12 * T || tv < -1e-12 * T) return null;
  return { T, tj, tc: Math.max(0, tc), tv: Math.max(0, tv), A: amax, V: amax * u, J, saturated: true };
}

/** Smallest T for which an S-curve exists with cruise speed ≤ vmax (bisection; feasibility is monotone in T). */
export function scurveMinTime(lim: Limits): number {
  const ok = (T: number) => {
    const p = scurveForT(T, lim.amax, lim.jmax);
    return !!p && p.V <= lim.vmax * (1 + EPS_REL);
  };
  let lo = Math.cbrt(32 / lim.jmax) * (1 - 1e-9);
  let hi = Math.max(lo * 1.01, 1e-3);
  let guard = 0;
  while (!ok(hi) && guard++ < 200) hi *= 1.3;
  for (let i = 0; i < 100; i++) {
    const mid = 0.5 * (lo + hi);
    if (ok(mid)) hi = mid; else lo = mid;
  }
  return hi;
}

interface Segment { t0: number; dur: number; j: number; s0: number; v0: number; a0: number }

function scurveSegments(p: SCurveParams): Segment[] {
  const durs = [p.tj, p.tc, p.tj, p.tv, p.tj, p.tc, p.tj];
  const jerks = [p.J, 0, -p.J, 0, -p.J, 0, p.J];
  const segs: Segment[] = [];
  let t = 0, s = 0, v = 0, a = 0;
  for (let k = 0; k < 7; k++) {
    const d = durs[k], j = jerks[k];
    segs.push({ t0: t, dur: d, j, s0: s, v0: v, a0: a });
    s += v * d + (a * d * d) / 2 + (j * d * d * d) / 6;
    v += a * d + (j * d * d) / 2;
    a += j * d;
    t += d;
  }
  return segs;
}

/** Evaluates an S-curve; outside [0, T] the profile is at rest (s = 0 before, s = 1 after). */
export function scurveSample(p: SCurveParams, segs: Segment[], t: number): ScalingState {
  if (t <= 0) return { s: 0, sd: 0, sdd: 0, jerk: 0 };
  if (t >= p.T) return { s: 1, sd: 0, sdd: 0, jerk: 0 };
  let k = 0;
  while (k < 6 && t > segs[k].t0 + segs[k].dur) k++;
  const g = segs[k];
  const tau = Math.min(g.dur, Math.max(0, t - g.t0));
  return {
    s: g.s0 + g.v0 * tau + (g.a0 * tau * tau) / 2 + (g.j * tau * tau * tau) / 6,
    sd: g.v0 + g.a0 * tau + (g.j * tau * tau) / 2,
    sdd: g.a0 + g.j * tau,
    jerk: g.j,
  };
}

export function scurveProfile(Treq: number, lim: Limits): Profile {
  const Tmin = scurveMinTime(lim);
  let p = scurveForT(Treq, lim.amax, lim.jmax);
  const feasible = !!p && p.V <= lim.vmax * (1 + EPS_REL);
  if (!feasible) p = scurveForT(Tmin, lim.amax, lim.jmax);
  // scurveMinTime guarantees a profile at Tmin; the fallback below is only a guard against rounding.
  if (!p) p = scurveForT(Tmin * (1 + 1e-9), lim.amax, lim.jmax) as SCurveParams;
  const P = p;
  const segs = scurveSegments(P);
  const breaks = [...segs.map((g) => g.t0), P.T].filter((x, i, a) => i === 0 || x > a[i - 1] + 1e-12);
  return {
    kind: 'scurve', T: P.T, Treq, feasible, Tmin,
    accelJumps: [], breaks,
    sample: (t) => scurveSample(P, segs, t),
    peak: { sd: P.V, sdd: P.A, jerk: P.J },
    info: { v: P.V, a: P.A, tJerk: P.tj, tConstA: P.tc, tCruise: P.tv, aSaturated: P.saturated },
  };
}

// ---------------------------------------------------------------------------
// Minimum durations under the same limits (for a fair comparison)
// ---------------------------------------------------------------------------

/**
 * Shortest T for which a cubic / quintic stays within ṡ ≤ vmax and |s̈| ≤ amax. Their peaks scale as
 * 1/T and 1/T² (cubic 1.5/T, 6/T²; quintic 1.875/T, 5.774/T²); the jerk limit is not considered.
 */
export function polyMinTime(kind: 'cubic' | 'quintic', lim: Limits): number {
  const [kv, ka] = kind === 'cubic' ? [1.5, 6] : [1.875, 10 / Math.sqrt(3)];
  return Math.max(kv / lim.vmax, Math.sqrt(ka / lim.amax));
}

export function minTime(kind: ScalingKind, lim: Limits): number {
  if (kind === 'cubic' || kind === 'quintic') return polyMinTime(kind, lim);
  if (kind === 'trapezoid') return trapezoidMinTime(lim.vmax, lim.amax).T;
  return scurveMinTime(lim);
}

// ---------------------------------------------------------------------------
// Dispatcher + sampling helpers
// ---------------------------------------------------------------------------

export function makeProfile(kind: ScalingKind, T: number, lim: Limits): Profile {
  switch (kind) {
    case 'cubic': return cubicProfile(T);
    case 'quintic': return quinticProfile(T);
    case 'trapezoid': return trapezoidProfile(T, lim);
    default: return scurveProfile(T, lim);
  }
}

/** Sample times on [t0, t1]: a uniform grid plus points just before/after every break (crisp jumps). */
export function sampleTimes(p: Profile, t0: number, t1: number, n = 300): number[] {
  const ts: number[] = [];
  for (let i = 0; i <= n; i++) ts.push(t0 + ((t1 - t0) * i) / n);
  const d = 1e-7 * Math.max(1, p.T);
  for (const b of p.breaks) { ts.push(b - d, b + d); }
  return ts.filter((t) => t >= t0 && t <= t1).sort((a, b) => a - b);
}

/** Time at which the (monotone) profile reaches path parameter s (bisection). */
export function timeAtS(p: Profile, s: number): number {
  if (s <= 0) return 0;
  if (s >= 1) return p.T;
  let lo = 0, hi = p.T;
  for (let i = 0; i < 60; i++) {
    const mid = 0.5 * (lo + hi);
    if (p.sample(mid).s < s) lo = mid; else hi = mid;
  }
  return 0.5 * (lo + hi);
}

// ---------------------------------------------------------------------------
// Flexible load: a spring-mass behind the joint, driven by the commanded s(t)
// ---------------------------------------------------------------------------

export interface FlexResult {
  /** Sample times (every `stride` integration steps). */
  t: number[];
  /** Load position q(t) (same units as s). */
  q: number[];
  /** Deviation e(t) = q(t) − s(t). */
  e: number[];
  /** max |q − 1| after the motion ends (t ≥ T): residual vibration amplitude. */
  residual: number;
  /** max |q − s| during the motion. */
  maxDev: number;
}

/**
 * q̈ = ωn² (s − q) + 2 ζ ωn (ṡ − q̇): the load follows the commanded joint through a spring and a damper.
 * A jump in s̈ is a step in the spring force, which rings at ωn; a bounded jerk spreads the step over time.
 * Integrated with @/mr rk4Step from t = 0 to tEnd with fixed step h.
 */
export function flexResponse(p: Profile, fn: number, zeta: number, tEnd: number, h = 0.002, stride = 2): FlexResult {
  const wn = 2 * Math.PI * fn;
  const f = (t: number, x: number[]) => {
    const r = p.sample(t);
    return [x[1], wn * wn * (r.s - x[0]) + 2 * zeta * wn * (r.sd - x[1])];
  };
  let x = [0, 0];
  const out: FlexResult = { t: [0], q: [0], e: [0], residual: 0, maxDev: 0 };
  const steps = Math.ceil(tEnd / h);
  for (let k = 0; k < steps; k++) {
    const t = k * h;
    x = rk4Step(f, t, x, h);
    const tn = (k + 1) * h;
    const s = p.sample(tn).s;
    const dev = x[0] - s;
    if (tn >= p.T) out.residual = Math.max(out.residual, Math.abs(x[0] - 1));
    else out.maxDev = Math.max(out.maxDev, Math.abs(dev));
    if ((k + 1) % stride === 0) { out.t.push(tn); out.q.push(x[0]); out.e.push(dev); }
  }
  return out;
}

/** Linear interpolation into a FlexResult at time t (clamped). */
export function flexAt(r: FlexResult, t: number): { q: number; e: number } {
  const n = r.t.length;
  if (t <= r.t[0]) return { q: r.q[0], e: r.e[0] };
  if (t >= r.t[n - 1]) return { q: r.q[n - 1], e: r.e[n - 1] };
  const dt = r.t[1] - r.t[0];
  const k = Math.min(n - 2, Math.floor((t - r.t[0]) / dt));
  const u = (t - r.t[k]) / (r.t[k + 1] - r.t[k]);
  return { q: r.q[k] + u * (r.q[k + 1] - r.q[k]), e: r.e[k] + u * (r.e[k + 1] - r.e[k]) };
}
