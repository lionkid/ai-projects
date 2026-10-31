/**
 * Planar mobile manipulator: a wheeled base (differential drive or omnidirectional) carrying a
 * 2R arm. The end-effector body twist is
 *     V_e = J_e [u; θ̇] = [J_base  J_arm] [u; θ̇],   J_base = [Ad_{T_eb}] F,
 * where F maps the base inputs u to the chassis twist V_b (diff: u = (v, ω); omni: u = V_b) and
 * J_arm is the arm's body Jacobian in {e} (@/mr planarBodyJacobian). The end-effector follows a
 * straight line with fixed orientation using the feedforward + P task-space law
 *     V = [Ad_{X⁻¹X_d}] V_d + K_p X_err,   [X_err] = log(X⁻¹X_d),
 * and the redundancy is resolved either by the minimum-norm solution J_e^# V or by adding the
 * null-space motion (I − J_e^# J_e) z that pulls the arm back to a comfortable posture.
 * J_e^# is a damped least-squares pseudo-inverse with a tiny damping (it equals J_e^† whenever
 * J_e has full rank and stays bounded if it ever loses rank).
 */
import { QuinticTimeScalingDerivs, eye, inv, matMul, planarBodyJacobian, planarFK, se2, transInv2, transpose, type Mat } from '@/mr';
import { adjointPose, adjointT2, advance, between, compose, logPose, mul3, t2ToPose, type Pose, type Twist2 } from './se2';
import type { BaseKind } from './tracking';

export type Redundancy = 'minnorm' | 'center';

/** Arm link lengths (m) and the mount offset of joint 1 along x_b (m). */
export const ARM = { L: [0.5, 0.4] as [number, number], mount: 0.12 };
const deg = Math.PI / 180;
/** Comfortable arm posture (elbow bent, |sin θ₂| ≈ 0.77). */
export const HOME: [number, number] = [140 * deg, -50 * deg];
/** Damping of the pseudo-inverse. */
export const DLS_LAMBDA = 1e-3;

/** Base input map F: columns are the chassis twists [ω, v_x, v_y] produced by unit inputs. */
export function baseInputMap(base: BaseKind): Mat {
  // diff: u = (v, ω);  omni: u = (ω, v_x, v_y) = V_b
  return base === 'diff' ? [[0, 1], [1, 0], [0, 0]] : eye(3);
}

/** Pose of the end-effector frame {e} in the base frame {b} (3×3). */
export function baseToEE(theta: [number, number]): Mat {
  return matMul(se2(0, ARM.mount, 0), planarFK(ARM.L, theta).T);
}

/** End-effector pose in {s} (heading kept unwrapped: φ + θ₁ + θ₂). */
export function eePose(q: Pose, theta: [number, number]): Pose {
  const p = t2ToPose(baseToEE(theta));
  return compose(q, { phi: theta[0] + theta[1], x: p.x, y: p.y });
}

export interface MobileJacobian { J: Mat; Jbase: Mat; Jarm: Mat; nb: number }

/** J_e = [J_base | J_arm] for the end-effector body twist [ω, v_x, v_y] in {e}. */
export function mobileJacobian(base: BaseKind, theta: [number, number]): MobileJacobian {
  const Jarm = planarBodyJacobian(ARM.L, theta);
  const Teb = transInv2(baseToEE(theta));
  const Jbase = matMul(adjointT2(Teb), baseInputMap(base));
  const J = Jbase.map((row, i) => [...row, ...Jarm[i]]);
  return { J, Jbase, Jarm, nb: Jbase[0].length };
}

/** Damped least-squares pseudo-inverse Jᵀ(JJᵀ + λ²I)⁻¹ of a 3×n Jacobian. */
export function dlsPinv(J: Mat, lambda = DLS_LAMBDA): Mat {
  const Jt = transpose(J);
  const A = matMul(J, Jt);
  for (let i = 0; i < A.length; i++) A[i][i] += lambda * lambda;
  return matMul(Jt, inv(A));
}

/** Arm manipulability measure |sin θ₂| (1 = elbow at 90°, 0 = arm stretched or folded: singular). */
export const armManipulability = (theta: [number, number]): number => Math.abs(Math.sin(theta[1]));

export interface MMConfig {
  base: BaseKind;
  mode: Redundancy;
  /** Line start / end for the end-effector (m) and its fixed heading (rad). */
  A: [number, number];
  B: [number, number];
  phiE: number;
  /** Duration of the line (quintic time scaling) and extra settling time (s). */
  Tf: number;
  settle?: number;
  /** Task-space feedback gain (1/s) and null-space gain (1/s). */
  kp: number;
  kn: number;
  home?: [number, number];
  h?: number;
}

export interface MMTrial {
  h: number;
  T: number;
  t: number[];
  q: Pose[];
  theta: [number, number][];
  X: Pose[];
  Xd: Pose[];
  /** Base inputs u and joint rates θ̇. */
  u: number[][];
  thd: [number, number][];
  /** World-frame linear velocity of the end-effector produced by the base and by the arm. */
  vBase: [number, number][];
  vArm: [number, number][];
  /** Component along the line direction of vBase / vArm and of the desired velocity. */
  sBase: number[];
  sArm: number[];
  sDes: number[];
  w: number[];
  errNorm: number[];
  /** J_e at every sample (3 × (nb + 2)). */
  J: Mat[];
}

/** Base pose that puts the end-effector at (p, φ_e) with the arm at `theta`. */
export function baseForEE(p: [number, number], phiE: number, theta: [number, number]): Pose {
  // X = q · T_be  →  q = X · T_be⁻¹ (heading written out so that it stays unwrapped)
  const q = compose({ phi: phiE, x: p[0], y: p[1] }, t2ToPose(transInv2(baseToEE(theta))));
  return { phi: phiE - (theta[0] + theta[1]), x: q.x, y: q.y };
}

/** Desired end-effector pose and body twist on the straight line at time t. */
export function lineReference(cfg: MMConfig, t: number): { Xd: Pose; Vd: Twist2; speed: number } {
  const d = QuinticTimeScalingDerivs(cfg.Tf, Math.min(Math.max(t, 0), cfg.Tf));
  const dx = cfg.B[0] - cfg.A[0], dy = cfg.B[1] - cfg.A[1];
  const Xd = { phi: cfg.phiE, x: cfg.A[0] + d.s * dx, y: cfg.A[1] + d.s * dy };
  const c = Math.cos(cfg.phiE), s = Math.sin(cfg.phiE);
  const vx = d.sdot * dx, vy = d.sdot * dy;
  return { Xd, Vd: [0, c * vx + s * vy, -s * vx + c * vy], speed: d.sdot * Math.hypot(dx, dy) };
}

/** Joint-space command for one control period. */
export function resolveRates(cfg: Pick<MMConfig, 'base' | 'mode' | 'kn' | 'home'>, theta: [number, number], V: Twist2): { rates: number[]; jac: MobileJacobian } {
  const jac = mobileJacobian(cfg.base, theta);
  const Jp = dlsPinv(jac.J);
  const n = jac.J[0].length;
  const rates = Jp.map((row) => row[0] * V[0] + row[1] * V[1] + row[2] * V[2]);
  if (cfg.mode === 'center') {
    const home = cfg.home ?? HOME;
    const z = new Array(n).fill(0);
    z[jac.nb] = cfg.kn * (home[0] - theta[0]);
    z[jac.nb + 1] = cfg.kn * (home[1] - theta[1]);
    // (I − J^# J) z: the end-effector twist is unchanged, only the split between base and arm.
    const Jz = jac.J.map((row) => row.reduce((a, x, j) => a + x * z[j], 0));
    for (let i = 0; i < n; i++) rates[i] += z[i] - (Jp[i][0] * Jz[0] + Jp[i][1] * Jz[1] + Jp[i][2] * Jz[2]);
  }
  return { rates, jac };
}

/** Simulate the whole line-following run. */
export function simulateMobile(cfg: MMConfig): MMTrial {
  const h = cfg.h ?? 0.02;
  const T = cfg.Tf + (cfg.settle ?? 3);
  const N = Math.max(1, Math.round(T / h));
  const home = cfg.home ?? HOME;
  let theta: [number, number] = [home[0], home[1]];
  let q = baseForEE(cfg.A, cfg.phiE, theta);
  const F = baseInputMap(cfg.base);
  const len = Math.hypot(cfg.B[0] - cfg.A[0], cfg.B[1] - cfg.A[1]) || 1;
  const dir: [number, number] = [(cfg.B[0] - cfg.A[0]) / len, (cfg.B[1] - cfg.A[1]) / len];
  const out: MMTrial = { h, T, t: [], q: [], theta: [], X: [], Xd: [], u: [], thd: [], vBase: [], vArm: [], sBase: [], sArm: [], sDes: [], w: [], errNorm: [], J: [] };
  for (let k = 0; k <= N; k++) {
    const t = k * h;
    const ref = lineReference(cfg, t);
    const X = eePose(q, theta);
    const E = between(X, ref.Xd);
    const Xerr = logPose(E);
    const ffV = mul3(adjointPose(E), ref.Vd);
    const V: Twist2 = [ffV[0] + cfg.kp * Xerr[0], ffV[1] + cfg.kp * Xerr[1], ffV[2] + cfg.kp * Xerr[2]];
    const { rates, jac } = resolveRates({ ...cfg, home }, theta, V);
    const u = rates.slice(0, jac.nb);
    const thd: [number, number] = [rates[jac.nb], rates[jac.nb + 1]];
    // End-effector velocity contributions (body twists in {e}) → world linear velocities.
    const Ve_b = jac.Jbase.map((row) => row.reduce((a, x, j) => a + x * u[j], 0));
    const Ve_a = jac.Jarm.map((row) => row[0] * thd[0] + row[1] * thd[1]);
    const c = Math.cos(X.phi), s = Math.sin(X.phi);
    const vB: [number, number] = [c * Ve_b[1] - s * Ve_b[2], s * Ve_b[1] + c * Ve_b[2]];
    const vA: [number, number] = [c * Ve_a[1] - s * Ve_a[2], s * Ve_a[1] + c * Ve_a[2]];
    out.t.push(t); out.q.push(q); out.theta.push(theta); out.X.push(X); out.Xd.push(ref.Xd);
    out.u.push(u); out.thd.push(thd); out.vBase.push(vB); out.vArm.push(vA);
    out.sBase.push(vB[0] * dir[0] + vB[1] * dir[1]);
    out.sArm.push(vA[0] * dir[0] + vA[1] * dir[1]);
    out.sDes.push(ref.speed);
    out.w.push(armManipulability(theta));
    out.errNorm.push(Math.hypot(E.x, E.y));
    out.J.push(jac.J);
    if (k < N) {
      const Vb = F.map((row) => row.reduce((a, x, j) => a + x * u[j], 0)) as Twist2;
      q = advance(q, Vb, h);
      theta = [theta[0] + thd[0] * h, theta[1] + thd[1] * h];
    }
  }
  return out;
}

/** Smallest arm manipulability reached during a run, and when. */
export function minManipulability(trial: MMTrial): { w: number; t: number } {
  let best = Infinity, tb = 0;
  trial.w.forEach((w, i) => { if (w < best) { best = w; tb = trial.t[i]; } });
  return { w: best, t: tb };
}
