/**
 * Trajectory tracking for a wheeled base (pure kinematic models, fixed control period h).
 *
 *  - Differential drive (unicycle inputs u = (v, ω)): feedforward + feedback law with the pose
 *    error expressed in the ROBOT frame {b} (or, as a counter-example, in the world frame {s}):
 *        v = v_d cos φ_e + k1 x_e,   ω = ω_d + k2 v_d y_e + k3 |v_d| sin φ_e.
 *    For v_d > 0 this is v_d (k2 y_e + k3 sin φ_e); |v_d| keeps the heading term stabilising when
 *    the reference drives in reverse. Wheel speeds saturate (curvature is preserved).
 *  - Omnidirectional base (any body twist): V_b = [Ad_{X⁻¹X_d}] V_d + K_p X_err + K_i ∫X_err dt with
 *    [X_err] = log(X⁻¹X_d) — the task-space feedforward + PI law, applied to the chassis.
 *
 * The robot pose is advanced exactly for each period with q ← q·exp([V] h) (V held constant).
 */
import { advance, adjointPose, between, compose, logPose, mul3, type Pose, type Twist2 } from './se2';
import type { Reference } from './references';

export type BaseKind = 'diff' | 'omni';
export type ErrFrame = 'body' | 'world';

export interface DiffGains { k1: number; k2: number; k3: number }
export interface OmniGains { kp: number; ki: number }

/** Wheel radius r (m), half track d (m) and wheel-speed limit (rad/s) of the differential drive. */
export const DIFF_DRIVE = { r: 0.08, d: 0.2, uMax: 15 } as const;
/** Speed limits of the omnidirectional base. */
export const OMNI_LIMITS = { v: 1.2, w: 4 } as const;

export interface TrackingConfig {
  base: BaseKind;
  /** Use the feedforward terms (v_d cos φ_e, ω_d) / ([Ad] V_d). */
  ff: boolean;
  /** Use the feedback terms. */
  fb: boolean;
  /** Where the differential-drive law reads its position error (omni always uses log(X⁻¹X_d)). */
  errFrame: ErrFrame;
  diff: DiffGains;
  omni: OmniGains;
  /** Constant extra yaw rate (rad/s) added to the body twist, e.g. one wheel running fast. */
  bias: number;
  /** Optional push: at time t the robot is displaced by the body-frame pose d. */
  push: { t: number; d: Pose } | null;
  /** Initial robot pose. */
  start: Pose;
  /** Control / integration period (s), default 0.01. */
  h?: number;
}

export interface TrackingTrial {
  h: number;
  t: number[];
  /** Robot pose. */
  q: Pose[];
  /** Reference pose. */
  qd: Pose[];
  /** Pose error in the robot frame (φ_e, x_e, y_e) = rotation and translation of X⁻¹X_d. */
  err: Twist2[];
  /** Applied (saturated) body twist [ω, v_x, v_y], before the bias disturbance. */
  u: Twist2[];
  /** Feedforward and feedback parts of the command (before saturation). */
  uff: Twist2[];
  ufb: Twist2[];
  /** Reference body twist [ω_d, v_d, v_{y,d}]. */
  ud: Twist2[];
  /** ∫X_err dt (omni with feedback; zeros otherwise). */
  integ: Twist2[];
  sat: boolean[];
  /** Sample index of the push (−1 when there is none). */
  pushIndex: number;
  /** True when the state became non-finite (should not happen with saturation). */
  diverged: boolean;
}

export interface DiffCommand { ff: Twist2; fb: Twist2 }

/** Differential-drive tracking law; E = X⁻¹X_d, dWorld = p_d − p in {s}. */
export function diffLaw(E: Pose, dWorld: [number, number], Vd: Twist2, cfg: Pick<TrackingConfig, 'ff' | 'fb' | 'errFrame' | 'diff'>): DiffCommand {
  const vd = Vd[1], wd = Vd[0];
  const [ex, ey] = cfg.errFrame === 'body' ? [E.x, E.y] : dWorld;
  const { k1, k2, k3 } = cfg.diff;
  const ff: Twist2 = cfg.ff ? [wd, vd * Math.cos(E.phi), 0] : [0, 0, 0];
  const fb: Twist2 = cfg.fb ? [k2 * vd * ey + k3 * Math.abs(vd) * Math.sin(E.phi), k1 * ex, 0] : [0, 0, 0];
  return { ff, fb };
}

/** Scale (v, ω) so that both wheel speeds stay within ±uMax; the path curvature is unchanged. */
export function saturateDiff(v: number, w: number): { v: number; w: number; sat: boolean } {
  const { r, d, uMax } = DIFF_DRIVE;
  const uR = (v + d * w) / r, uL = (v - d * w) / r;
  const m = Math.max(Math.abs(uR), Math.abs(uL));
  if (m <= uMax) return { v, w, sat: false };
  const s = uMax / m;
  return { v: v * s, w: w * s, sat: true };
}

/** Wheel speeds (u_L, u_R) in rad/s for unicycle inputs. */
export function wheelSpeeds(v: number, w: number): [number, number] {
  const { r, d } = DIFF_DRIVE;
  return [(v - d * w) / r, (v + d * w) / r];
}

/** Clamp an omni twist to the speed limits (direction preserved). */
export function saturateOmni(V: Twist2): { V: Twist2; sat: boolean } {
  const sp = Math.hypot(V[1], V[2]);
  const s = Math.min(1, OMNI_LIMITS.v / Math.max(sp, 1e-12), OMNI_LIMITS.w / Math.max(Math.abs(V[0]), 1e-12));
  return s < 1 ? { V: [V[0] * s, V[1] * s, V[2] * s], sat: true } : { V, sat: false };
}

/** Linearised lateral error dynamics of the diff-drive law at constant v_d (straight reference). */
export function lateralMode(vd: number, k2: number, k3: number): { wn: number; zeta: number } {
  const wn = Math.abs(vd) * Math.sqrt(Math.max(k2, 0));
  const zeta = k2 > 0 ? k3 / (2 * Math.sqrt(k2)) : Infinity;
  return { wn, zeta };
}

/** Simulate a whole tracking run against `ref` (cheap enough to recompute on every UI change). */
export function simulateTracking(ref: Reference, cfg: TrackingConfig, T = ref.T): TrackingTrial {
  const h = cfg.h ?? 0.01;
  const N = Math.max(1, Math.round(T / h));
  const out: TrackingTrial = { h, t: [], q: [], qd: [], err: [], u: [], uff: [], ufb: [], ud: [], integ: [], sat: [], pushIndex: -1, diverged: false };
  let q: Pose = { ...cfg.start };
  let I: Twist2 = [0, 0, 0];
  for (let k = 0; k <= N; k++) {
    const t = k * h;
    if (cfg.push && out.pushIndex < 0 && t >= cfg.push.t - 1e-9) { q = compose(q, cfg.push.d); out.pushIndex = k; }
    const r = ref.sample(t);
    const E = between(q, r.q);
    let ff: Twist2, fb: Twist2, V: Twist2, sat: boolean;
    if (cfg.base === 'diff') {
      const c = diffLaw(E, [r.q.x - q.x, r.q.y - q.y], r.V, cfg);
      ff = c.ff; fb = c.fb;
      const s = saturateDiff(ff[1] + fb[1], ff[0] + fb[0]);
      V = [s.w, s.v, 0]; sat = s.sat;
    } else {
      const Xerr = logPose(E);
      ff = cfg.ff ? mul3(adjointPose(E), r.V) : [0, 0, 0];
      fb = cfg.fb ? [0, 1, 2].map((i) => cfg.omni.kp * Xerr[i] + cfg.omni.ki * I[i]) as Twist2 : [0, 0, 0];
      const s = saturateOmni([ff[0] + fb[0], ff[1] + fb[1], ff[2] + fb[2]]);
      V = s.V; sat = s.sat;
      // Conditional integration (a simple anti-windup): freeze ∫X_err while the base saturates.
      if (cfg.fb && cfg.omni.ki > 0 && !sat) I = [I[0] + Xerr[0] * h, I[1] + Xerr[1] * h, I[2] + Xerr[2] * h];
    }
    out.t.push(t); out.q.push(q); out.qd.push(r.q); out.err.push([E.phi, E.x, E.y]);
    out.u.push(V); out.uff.push(ff); out.ufb.push(fb); out.ud.push(r.V); out.integ.push(I); out.sat.push(sat);
    if (!Number.isFinite(q.x) || !Number.isFinite(q.y) || !Number.isFinite(q.phi)) { out.diverged = true; break; }
    if (k < N) q = advance(q, [V[0] + cfg.bias, V[1], V[2]], h);
  }
  return out;
}

/** Index of the sample at time t (clamped). */
export function indexAt(trial: { t: number[]; h: number }, t: number): number {
  return Math.max(0, Math.min(trial.t.length - 1, Math.round(t / trial.h)));
}

/** Root-mean-square of the position error over [t0, end]. */
export function rmsPositionError(trial: TrackingTrial, t0 = 0): number {
  let s = 0, n = 0;
  for (let k = indexAt(trial, t0); k < trial.err.length; k++) { s += trial.err[k][1] ** 2 + trial.err[k][2] ** 2; n++; }
  return n ? Math.sqrt(s / n) : 0;
}
