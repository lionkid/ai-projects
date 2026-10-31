/**
 * Pure models for lesson ch11-01 (error dynamics and single-joint PID).
 *
 * - `simulateJoint` runs one closed-loop trial of a single revolute joint that is driven
 *   either by a torque (second-order plant  M θ̈ + b θ̇ + τ_g(θ) + τ_dist = τ) or, as an
 *   idealisation, directly by a velocity command (θ̇ = u). The controller is the discrete
 *   PID of `@/mr` (`pidStep`, derivative of the error, optional conditional-integration
 *   anti-windup); the plant is integrated with `rk4Step` under a zero-order hold.
 * - The analytic side used by the UI: closed-loop characteristic polynomials and their
 *   roots, second-order predictions (ζ, ω_n, overshoot, 2 % settling time), measured
 *   step-response metrics, the map from a desired pole pair to PD gains, and the static
 *   sag of a 2R arm under independent joint PD (preview of the next lesson).
 *
 * Conventions: θ is measured counter-clockwise from the horizontal, gravity acts along −y,
 * so the gravity load torque is τ_g(θ) = m g r cos θ. Error θ_e = θ_d − θ. SI units, rad.
 */
import {
  pidStep, pidInitialState, rk4Step, planar2RParams, planar2RDynamics,
  type PIDOptions, type PIDState, type Planar2RParams,
} from '@/mr';

export type InputMode = 'torque' | 'velocity';
export type RefKind = 'step' | 'ramp' | 'sine';

export interface JointPlant {
  /** Inertia about the joint axis (kg·m²). */
  M: number;
  /** Viscous friction coefficient (N·m·s/rad). */
  b: number;
  /** Gravity load amplitude m g r (N·m); the load torque is mgr·cos θ when gravity is on. */
  mgr: number;
}

export const DEFAULT_PLANT: JointPlant = { M: 0.5, b: 0.1, mgr: 3 };

export interface PIDGains { kp: number; ki: number; kd: number }

export interface Reference {
  kind: RefKind;
  /** Step height (rad), ramp slope (rad/s) or sine amplitude (rad). */
  amp: number;
  /** Sine frequency (Hz); ignored otherwise. */
  freq: number;
}

/** Desired position and velocity at time t (the reference starts at θ0). */
export function refAt(ref: Reference, theta0: number, t: number): { thd: number; dthd: number } {
  if (ref.kind === 'step') return { thd: theta0 + ref.amp, dthd: 0 };
  if (ref.kind === 'ramp') return { thd: theta0 + ref.amp * t, dthd: ref.amp };
  const w = 2 * Math.PI * ref.freq;
  return { thd: theta0 + ref.amp * Math.sin(w * t), dthd: ref.amp * w * Math.cos(w * t) };
}

export interface TrialOptions {
  mode: InputMode;
  plant: JointPlant;
  gains: PIDGains;
  ref: Reference;
  /** Initial angle (rad); the joint starts at rest. Default 0. */
  theta0?: number;
  /** Include the gravity load mgr·cos θ (torque mode only). */
  gravity?: boolean;
  /** Constant disturbance/load torque the motor has to overcome (N·m, torque mode only). */
  tauDist?: number;
  /** Actuator torque limit (N·m); Infinity = ideal actuator. */
  tauMax?: number;
  /** Conditional-integration anti-windup (stop integrating while saturated). */
  antiWindup?: boolean;
  /** Velocity mode only: add the reference velocity θ̇_d as feedforward. */
  velFeedforward?: boolean;
  /** Trial duration (s). */
  T: number;
  /** Control period = integration step (s). Default 2 ms. */
  h?: number;
  /** Keep every `every`-th sample in the output. Default 5 (100 Hz at the default step). */
  every?: number;
}

export interface Trial {
  t: number[];
  theta: number[];
  dtheta: number[];
  thd: number[];
  err: number[];
  /** Applied command: torque (N·m) in torque mode, joint velocity (rad/s) in velocity mode. */
  u: number[];
  /** Controller output before the actuator limit. */
  uCmd: number[];
  /** Integral contribution K_i ∫θ_e (same unit as u). */
  iTerm: number[];
  /** The actuator limit was active at some instant. */
  saturated: boolean;
  /** The state left a sane range (unstable gains); the arrays stop there. */
  diverged: boolean;
}

/** One closed-loop trial; see the module comment for the plant and controller. */
export function simulateJoint(o: TrialOptions): Trial {
  const h = o.h ?? 2e-3;
  const every = Math.max(1, Math.round(o.every ?? 5));
  const theta0 = o.theta0 ?? 0;
  const torque = o.mode === 'torque';
  const tauMax = torque ? (o.tauMax ?? Infinity) : Infinity;
  const limited = Number.isFinite(tauMax);
  const tauDist = torque ? (o.tauDist ?? 0) : 0;
  const gravity = torque && !!o.gravity;
  const { M, b, mgr } = o.plant;
  const kd = torque ? o.gains.kd : 0;
  const pid: PIDOptions = { kp: o.gains.kp, ki: o.gains.ki, kd };
  if (limited && o.antiWindup) { pid.outputMin = -tauMax; pid.outputMax = tauMax; }

  const out: Trial = { t: [], theta: [], dtheta: [], thd: [], err: [], u: [], uCmd: [], iTerm: [], saturated: false, diverged: false };
  let st: PIDState = pidInitialState();
  let x = [theta0, 0];
  const N = Math.round(o.T / h);
  for (let k = 0; k <= N; k++) {
    const t = k * h;
    const r = refAt(o.ref, theta0, t);
    const e = r.thd - x[0];
    const prevErr = st.prevError;
    const res = pidStep(pid, st, e, h);
    st = res.state;
    const deriv = prevErr === null ? 0 : (e - prevErr) / h;
    const uCmd = pid.kp * e + pid.ki * st.integral + kd * deriv;
    let u = res.output;
    if (torque) {
      if (limited && Math.abs(uCmd) > tauMax + 1e-9) out.saturated = true;
      u = Math.max(-tauMax, Math.min(tauMax, u));
    } else if (o.velFeedforward) {
      u += r.dthd;
    }
    if (k % every === 0 || k === N) {
      out.t.push(t); out.theta.push(x[0]); out.dtheta.push(torque ? x[1] : u); out.thd.push(r.thd); out.err.push(e);
      out.u.push(u); out.uCmd.push(torque ? uCmd : u); out.iTerm.push(pid.ki * st.integral);
    }
    if (k === N) break;
    if (torque) {
      const tau = u;
      const f = (_t: number, s: number[]) => [s[1], (tau - b * s[1] - (gravity ? mgr * Math.cos(s[0]) : 0) - tauDist) / M];
      x = rk4Step(f, t, x, h);
    } else {
      x = [x[0] + u * h, u];
    }
    if (!Number.isFinite(x[0]) || !Number.isFinite(x[1]) || Math.abs(x[1]) > 500 || Math.abs(x[0] - theta0) > 200) {
      out.diverged = true;
      break;
    }
  }
  return out;
}

// ---------------------------------------------------------------------------
// Linear analysis
// ---------------------------------------------------------------------------

/**
 * Characteristic polynomial (highest power first) of the linear error dynamics, without
 * gravity and actuator limits. Torque input, PD: M s² + (b + K_d) s + K_p; with an I term the
 * error equation is differentiated once: M s³ + (b + K_d) s² + K_p s + K_i. Velocity input,
 * P: s + K_p; PI: s² + K_p s + K_i.
 */
export function charPoly(mode: InputMode, plant: JointPlant, g: PIDGains): number[] {
  if (mode === 'velocity') return g.ki > 0 ? [1, g.kp, g.ki] : [1, g.kp];
  return g.ki > 0 ? [plant.M, plant.b + g.kd, g.kp, g.ki] : [plant.M, plant.b + g.kd, g.kp];
}

export interface Complex { re: number; im: number }

/** Roots of a real polynomial of degree 1–3 (coefficients highest power first). */
export function polyRoots(coeffs: number[]): Complex[] {
  let c = coeffs.slice();
  while (c.length > 1 && Math.abs(c[0]) < 1e-14) c = c.slice(1);
  const n = c.length - 1;
  if (n <= 0) return [];
  if (n === 1) return [{ re: -c[1] / c[0], im: 0 }];
  if (n === 2) return quadRoots(c[0], c[1], c[2]);
  if (n !== 3) throw new Error('polyRoots: degree > 3 not supported');
  const A = c[1] / c[0], B = c[2] / c[0], C = c[3] / c[0];
  const r = cubicRealRoot(A, B, C);
  // Deflate: s³ + A s² + B s + C = (s − r)(s² + (A + r) s + (B + (A + r) r)).
  const q = quadRoots(1, A + r, B + (A + r) * r);
  return [{ re: r, im: 0 }, ...q];
}

function quadRoots(a: number, b: number, c: number): Complex[] {
  const disc = b * b - 4 * a * c;
  if (disc >= 0) {
    const sq = Math.sqrt(disc);
    const q = -0.5 * (b + (b >= 0 ? sq : -sq));
    if (Math.abs(q) < 1e-300) return [{ re: 0, im: 0 }, { re: 0, im: 0 }];
    return [{ re: q / a, im: 0 }, { re: c / q, im: 0 }];
  }
  const re = -b / (2 * a), im = Math.sqrt(-disc) / (2 * Math.abs(a));
  return [{ re, im }, { re, im: -im }];
}

/** One real root of s³ + A s² + B s + C (Cardano / trigonometric form, Newton-polished). */
function cubicRealRoot(A: number, B: number, C: number): number {
  const p = B - (A * A) / 3;
  const q = (2 * A * A * A) / 27 - (A * B) / 3 + C;
  const D = (q * q) / 4 + (p * p * p) / 27;
  let y: number;
  if (Math.abs(p) < 1e-14) y = Math.cbrt(-q);
  else if (D > 0) { const sq = Math.sqrt(D); y = Math.cbrt(-q / 2 + sq) + Math.cbrt(-q / 2 - sq); }
  else {
    const m = 2 * Math.sqrt(-p / 3);
    const arg = Math.max(-1, Math.min(1, ((3 * q) / (2 * p)) * Math.sqrt(-3 / p)));
    y = m * Math.cos(Math.acos(arg) / 3);
  }
  let r = y - A / 3;
  for (let i = 0; i < 3; i++) {
    const f = ((r + A) * r + B) * r + C;
    const df = (3 * r + 2 * A) * r + B;
    if (Math.abs(df) < 1e-14) break;
    r -= f / df;
  }
  return r;
}

export interface SecondOrderPrediction {
  wn: number;
  zeta: number;
  /** Percent overshoot of the step response, 100·exp(−πζ/√(1−ζ²)) (0 when ζ ≥ 1). */
  overshootPct: number;
  /** 2 % settling-time rule of thumb: 4/(ζω_n) when ζ < 1, 4/|slow root| otherwise. */
  ts: number;
  /** Exact 2 % settling time of the linear model (`linearSettlingTime`). */
  tsExact: number;
}

/** Prediction for torque input + PD: M θ̈_e + (b + K_d) θ̇_e + K_p θ_e = 0. */
export function pdPrediction(plant: JointPlant, kp: number, kd: number): SecondOrderPrediction | null {
  if (!(kp > 0)) return null;
  const wn = Math.sqrt(kp / plant.M);
  const zeta = (plant.b + kd) / (2 * Math.sqrt(kp * plant.M));
  const overshootPct = zeta < 1 ? 100 * Math.exp((-Math.PI * zeta) / Math.sqrt(1 - zeta * zeta)) : 0;
  const ts = zeta < 1 ? 4 / (zeta * wn) : 4 / (wn * (zeta - Math.sqrt(zeta * zeta - 1)));
  return { wn, zeta, overshootPct, ts, tsExact: linearSettlingTime(zeta, wn) };
}

/**
 * Exact settling time of the LINEAR second-order error response ë + 2ζωn ė + ωn² e = 0 started
 * from e(0) = 1, ė(0) = 0: the time after which |e| stays within `band`. This is what the linear
 * model actually predicts; the familiar 4/(ζωn) is only an envelope estimate (it overestimates
 * for ζ ≈ 0.4 and is far too short near ζ = 1, where the exact value is ≈ 5.8/ωn).
 */
export function linearSettlingTime(zeta: number, wn: number, band = 0.02): number {
  if (!(wn > 0) || !(zeta > 0)) return Infinity;
  let e: (t: number) => number;
  let slow: number;
  if (Math.abs(zeta - 1) < 1e-6) {
    e = (t) => (1 + wn * t) * Math.exp(-wn * t);
    slow = wn;
  } else if (zeta < 1) {
    const wd = wn * Math.sqrt(1 - zeta * zeta), k = (zeta * wn) / wd;
    e = (t) => Math.exp(-zeta * wn * t) * (Math.cos(wd * t) + k * Math.sin(wd * t));
    slow = zeta * wn;
  } else {
    const r = wn * Math.sqrt(zeta * zeta - 1), s1 = -zeta * wn + r, s2 = -zeta * wn - r;
    e = (t) => (s1 * Math.exp(s2 * t) - s2 * Math.exp(s1 * t)) / (s1 - s2);
    slow = -s1;
  }
  // Past T the envelope is far inside the band (the +8 covers the 1/√(1−ζ²) amplitude near ζ = 1).
  const T = (Math.log(1 / band) + 8) / slow;
  const N = 4000;
  let last = -1;
  for (let k = N; k >= 0; k--) if (Math.abs(e((k * T) / N)) > band) { last = k; break; }
  if (last < 0) return 0;
  // Refine the band exit inside [t_last, t_last + T/N] by bisection.
  let lo = (last * T) / N, hi = ((last + 1) * T) / N;
  for (let i = 0; i < 40; i++) {
    const mid = 0.5 * (lo + hi);
    if (Math.abs(e(mid)) > band) lo = mid; else hi = mid;
  }
  return hi;
}

/** Right-most (slowest, dominant) root; for a complex pair the one with Im ≥ 0. */
export function dominantRoot(roots: Complex[]): Complex | null {
  let best: Complex | null = null;
  for (const z of roots) {
    if (!Number.isFinite(z.re) || !Number.isFinite(z.im)) continue;
    if (!best || z.re > best.re + 1e-9 || (Math.abs(z.re - best.re) <= 1e-9 && z.im > best.im)) best = z;
  }
  return best;
}

/** PD gains that place the roots of M s² + (b + K_d) s + K_p at re ± j·im. */
export function pdGainsFromPole(plant: JointPlant, re: number, im: number): { kp: number; kd: number } {
  return { kp: plant.M * (re * re + im * im), kd: -2 * re * plant.M - plant.b };
}

/** Routh–Hurwitz bound for the PID loop: stable iff 0 < K_i < (b + K_d) K_p / M. */
export function pidKiBound(plant: JointPlant, kp: number, kd: number): number {
  return ((plant.b + kd) * kp) / plant.M;
}

export interface StepMetrics {
  e0: number;
  /** Final error (last sample of the trial). */
  ess: number;
  /** Overshoot past the final value, in percent of |e0 − ess|. */
  overshootPct: number;
  peakT: number;
  peakE: number;
  /** Time after which |e − ess| stays within 2 % of |e0 − ess|; null if not settled in the trial. */
  ts: number | null;
}

/** Measured metrics of an error response that starts at e[0] (a step). */
export function stepMetrics(t: number[], e: number[], band = 0.02): StepMetrics {
  const n = e.length;
  const e0 = e[0];
  const ess = e[n - 1];
  // Samples in the final 5 % of the trial count as "the end": a response that still leaves
  // the band there has not settled.
  const m = Math.max(1, Math.floor(n * 0.05));
  const span = e0 - ess;
  if (Math.abs(span) < 1e-9) return { e0, ess, overshootPct: 0, peakT: 0, peakE: e0, ts: 0 };
  const sgn = Math.sign(span);
  let worst = 0, peakT = 0, peakE = ess;
  for (let i = 0; i < n; i++) {
    const over = (ess - e[i]) * sgn;
    if (over > worst) { worst = over; peakT = t[i]; peakE = e[i]; }
  }
  const tol = band * Math.abs(span);
  let last = -1;
  for (let i = n - 1; i >= 0; i--) if (Math.abs(e[i] - ess) > tol) { last = i; break; }
  // Still outside the band inside the averaging window → not settled within the trial.
  const ts = last < 0 ? 0 : last >= n - m ? null : t[last + 1];
  return { e0, ess, overshootPct: (100 * worst) / Math.abs(span), peakT, peakE, ts };
}

/** Peak |e| over the last `window` seconds (used for sine references). */
export function tailAmplitude(t: number[], e: number[], window: number): number {
  const tEnd = t[t.length - 1];
  let a = 0;
  for (let i = 0; i < t.length; i++) if (t[i] >= tEnd - window) a = Math.max(a, Math.abs(e[i]));
  return a;
}

// ---------------------------------------------------------------------------
// Preview of the next lesson: a 2R arm under independent joint PD
// ---------------------------------------------------------------------------

/** Uniform 1 m / 1 kg rods, the arm used in lesson ch11-02. */
export const PREVIEW_ARM: Planar2RParams = planar2RParams([1, 1], [1, 1], 'rod');

/**
 * Static equilibrium of a 2R arm held by independent joint PD (no gravity compensation):
 * K_p,i (θ_d,i − θ_i) = g_i(θ). Solved by a relaxed fixed-point iteration, which converges
 * whenever the PD stiffness dominates the gravity stiffness.
 */
export function pdSag2R(thd: [number, number], kp: [number, number], P: Planar2RParams = PREVIEW_ARM): { theta: [number, number]; residual: number } {
  let th: [number, number] = [thd[0], thd[1]];
  for (let k = 0; k < 200; k++) {
    const g = planar2RDynamics(P, th, [0, 0]).g;
    const next: [number, number] = [thd[0] - g[0] / kp[0], thd[1] - g[1] / kp[1]];
    th = [0.5 * th[0] + 0.5 * next[0], 0.5 * th[1] + 0.5 * next[1]];
  }
  const g = planar2RDynamics(P, th, [0, 0]).g;
  const residual = Math.hypot(kp[0] * (thd[0] - th[0]) - g[0], kp[1] * (thd[1] - th[1]) - g[1]);
  return { theta: th, residual };
}
