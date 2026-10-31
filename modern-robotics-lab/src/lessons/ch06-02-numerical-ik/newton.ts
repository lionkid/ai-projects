/**
 * Planar reduction of body-frame Newton–Raphson inverse kinematics for the lab's planar 3R arm.
 *
 * Everything is computed with the 6-D machinery of `@/mr` (FKinBody, JacobianBody, MatrixLog6,
 * Adjoint) and then reduced to the three planar components [ω_z; v_x; v_y] (or [v_x; v_y] for a
 * position-only task). Because the remaining rows of J_b and of the error twist are identically zero
 * for a planar chain, the reduced pseudo-inverse / DLS steps coincide with the 6-D ones, and the
 * pseudo-inverse iteration reproduces `IKinBody` exactly (see newton.test.ts).
 */
import {
  type Mat,
  type Vec,
  planar3R,
  FKinBody,
  JacobianBody,
  MatrixLog6,
  MatrixExp6,
  VecTose3,
  TransInv,
  Adjoint,
  RotZ,
  RpToTrans,
  matMul,
  matVec,
  se3ToVec,
  pinv,
  svd,
  transpose,
  inv,
  norm,
  dot,
  cross,
  wrapToPi,
} from '@/mr';

export type Method = 'pinv' | 'transpose' | 'dls';

export interface Task {
  x: number;
  y: number;
  /** Desired orientation of the tip frame (ignored when `positionOnly`). */
  phi: number;
  positionOnly: boolean;
}

export interface StepOptions {
  method: Method;
  /** Damping factor for DLS (ignored by the other methods). */
  lambda: number;
  /** Cap on |Δθ| per iteration; Infinity disables the cap. */
  maxStep: number;
  /** Optional joint weights W = diag(w) for a weighted minimum-norm step (pinv method only). */
  weights?: number[] | null;
}

export interface Residual {
  /** Current end-effector pose T_sb(θ). */
  Tsb: Mat;
  /** Full body error twist log(T_sb^{-1} T_sd) as a 6-vector [ω; v] (pure translation for a position task). */
  V6: Vec;
  /** Reduced residual: [ω_z, v_x, v_y] or [v_x, v_y]. */
  V: Vec;
  omegaNorm: number;
  vNorm: number;
}

export interface StepResult {
  dtheta: Vec;
  /** Singular values of the reduced Jacobian, descending. */
  sigma: number[];
  /** True when the step was shortened by `maxStep`. */
  clipped: boolean;
}

export const MODEL = planar3R;
export const LINKS: number[] = planar3R.linkLengths ?? [1, 0.8, 0.6];
export const REACH = LINKS.reduce((a, b) => a + b, 0);

/** Desired configuration T_sd built from a planar pose. */
export function targetTransform(t: Task): Mat {
  return RpToTrans(RotZ(t.positionOnly ? 0 : t.phi), [t.x, t.y, 0]);
}

/** Body error twist and its reduced form for the current joint vector. */
export function residual(theta: Vec, t: Task): Residual {
  const Tsb = FKinBody(MODEL.M, MODEL.Blist, theta);
  if (t.positionOnly) {
    // Position error expressed in {b}: R_sb^T (p_d - p_b). Only the x–y components are non-zero.
    const c = Tsb[0][0];
    const s = Tsb[1][0];
    const dx = t.x - Tsb[0][3];
    const dy = t.y - Tsb[1][3];
    const vx = c * dx + s * dy;
    const vy = -s * dx + c * dy;
    return { Tsb, V6: [0, 0, 0, vx, vy, 0], V: [vx, vy], omegaNorm: 0, vNorm: Math.hypot(vx, vy) };
  }
  const V6 = se3ToVec(MatrixLog6(matMul(TransInv(Tsb), targetTransform(t))));
  return {
    Tsb,
    V6,
    V: [V6[2], V6[3], V6[4]],
    omegaNorm: norm(V6.slice(0, 3)),
    vNorm: norm(V6.slice(3, 6)),
  };
}

/** Rows [ω_z; v_x; v_y] (or [v_x; v_y]) of the body Jacobian J_b(θ). */
export function reducedJacobian(theta: Vec, positionOnly: boolean): Mat {
  const J = JacobianBody(MODEL.Blist, theta);
  return positionOnly ? [J[3], J[4]] : [J[2], J[3], J[4]];
}

/** Singular values of a matrix, descending. */
export function singularValues(J: Mat): number[] {
  return svd(J).S.slice();
}

/**
 * Weighted minimum-norm solution of J Δθ = V: minimises Δθᵀ W Δθ, i.e. Δθ = W⁻¹Jᵀ(J W⁻¹ Jᵀ)⁻¹ V.
 * With W = I this is the Moore–Penrose step J⁺V (for a full-row-rank J).
 */
export function weightedMinNormStep(J: Mat, V: Vec, w: number[]): Vec {
  const Winv = w.map((x) => 1 / x);
  const JW = J.map((row) => row.map((x, j) => x * Winv[j]));
  const A = matMul(JW, transpose(J));
  const y = matVec(inv(A), V);
  return matVec(transpose(JW), y);
}

/** One update Δθ for the linearised problem J Δθ = V with the chosen method. */
export function solveStep(J: Mat, V: Vec, o: StepOptions): StepResult {
  let d: Vec;
  if (o.method === 'dls') {
    const lam = Math.max(o.lambda, 1e-9);
    const JJt = matMul(J, transpose(J));
    for (let i = 0; i < JJt.length; i++) JJt[i][i] += lam * lam;
    d = matVec(transpose(J), matVec(inv(JJt), V));
  } else if (o.method === 'transpose') {
    // Jacobian-transpose method with the step size that minimises |V - α J Jᵀ V| (first-order optimal).
    const JJtV = matVec(J, matVec(transpose(J), V));
    const den = dot(JJtV, JJtV);
    const alpha = den > 1e-18 ? dot(V, JJtV) / den : 1;
    d = matVec(transpose(J), V).map((x) => x * alpha);
  } else if (o.weights && J.length < J[0].length) {
    try { d = weightedMinNormStep(J, V, o.weights); } catch { d = matVec(pinv(J), V); }
  } else {
    d = matVec(pinv(J), V);
  }
  const len = norm(d);
  const clipped = Number.isFinite(o.maxStep) && len > o.maxStep;
  if (clipped) d = d.map((x) => (x * o.maxStep) / len);
  return { dtheta: d, sigma: singularValues(J), clipped };
}

/**
 * Unit null-space direction of a 2 × 3 position Jacobian (the self-motion of the redundant arm),
 * oriented so that it is continuous with `prev` when given. Returns null when J loses rank.
 */
export function nullSpaceDirection(J: Mat, prev?: Vec | null): Vec | null {
  if (J.length !== 2 || J[0].length !== 3) return null;
  const n = cross(J[0], J[1]);
  const len = norm(n);
  if (len < 1e-9) return null;
  let u = n.map((x) => x / len);
  if (prev) {
    if (dot(u, prev) < 0) u = u.map((x) => -x);
  } else if (u[0] < 0) {
    u = u.map((x) => -x);
  }
  return u;
}

/**
 * Path of the tip under the constant body twist V6 for s ∈ [0, 1]: T_sb exp([V6] s).
 * At s = 1 the frame reaches the target exactly, so this draws "the error as a screw motion".
 */
export function screwArc(Tsb: Mat, V6: Vec, samples = 24): Mat[] {
  const out: Mat[] = [];
  for (let k = 0; k <= samples; k++) {
    const s = k / samples;
    out.push(matMul(Tsb, MatrixExp6(VecTose3(V6.map((x) => x * s)))));
  }
  return out;
}

/** Convergence test (IKinBody uses eomg = 1e-3, ev = 1e-4 by default; the lesson uses stricter values). */
export function converged(r: Residual, eomg = 1e-3, ev = 1e-4): boolean {
  return r.omegaNorm <= eomg && r.vNorm <= ev;
}

// ---------------------------------------------------------------------------
// Whole runs (records for plotting) and stall detection
// ---------------------------------------------------------------------------

export interface IterRecord {
  /** Configuration at which the residual was evaluated (angles wrapped to [-π, π)). */
  theta: Vec;
  omegaNorm: number;
  vNorm: number;
  /** Reduced residual at `theta`. */
  V: Vec;
  /** Step taken FROM this configuration (raw, before wrapping); null until a step is taken. */
  dtheta: Vec | null;
  dthetaNorm: number;
  sigmaMin: number;
  sigmaMax: number;
  clipped: boolean;
}

export type RunStatus = 'running' | 'converged' | 'stalled' | 'maxIter';

export interface Tolerance { eomg: number; ev: number }
export const LESSON_TOL: Tolerance = { eomg: 1e-6, ev: 1e-6 };

/** Residual record (without a step) for a configuration. */
export function evaluate(theta: Vec, task: Task): IterRecord {
  const r = residual(theta, task);
  const S = singularValues(reducedJacobian(theta, task.positionOnly));
  return {
    theta: theta.map(wrapToPi),
    omegaNorm: r.omegaNorm,
    vNorm: r.vNorm,
    V: r.V,
    dtheta: null,
    dthetaNorm: 0,
    sigmaMin: S[S.length - 1],
    sigmaMax: S[0],
    clipped: false,
  };
}

/**
 * One Newton-type iteration from `rec` (which is updated with the step it takes).
 * Returns the record of the new configuration.
 */
export function iterate(rec: IterRecord, task: Task, o: StepOptions): IterRecord {
  const J = reducedJacobian(rec.theta, task.positionOnly);
  const st = solveStep(J, rec.V, o);
  rec.dtheta = st.dtheta;
  rec.dthetaNorm = norm(st.dtheta);
  rec.clipped = st.clipped;
  const next = rec.theta.map((x, i) => x + st.dtheta[i]);
  return evaluate(next, task);
}

/** Total residual used for stall detection (mixes rad and m on purpose, like the book's two tests). */
export const errorSize = (r: { omegaNorm: number; vNorm: number }): number => r.omegaNorm + r.vNorm;

/**
 * True when the last `window` iterations improved the best residual by less than the fraction
 * `minImprove`: the iteration is stuck (unreachable target, oscillation near a singularity, ...).
 */
export function isStalled(records: IterRecord[], window = 8, minImprove = 1e-3): boolean {
  const n = records.length;
  if (n <= window) return false;
  let before = Infinity;
  for (let i = 0; i < n - window; i++) before = Math.min(before, errorSize(records[i]));
  let recent = Infinity;
  for (let i = n - window; i < n; i++) recent = Math.min(recent, errorSize(records[i]));
  return recent > before * (1 - minImprove);
}

export function statusOf(records: IterRecord[], tol: Tolerance, maxIter: number): RunStatus {
  const last = records[records.length - 1];
  if (last.omegaNorm <= tol.eomg && last.vNorm <= tol.ev) return 'converged';
  if (isStalled(records)) return 'stalled';
  if (records.length - 1 >= maxIter) return 'maxIter';
  return 'running';
}

/** Runs the iteration to completion (used for the comparison overlay and the tests). */
export function runNewton(theta0: Vec, task: Task, o: StepOptions, maxIter = 60, tol: Tolerance = LESSON_TOL): { records: IterRecord[]; status: RunStatus } {
  const records = [evaluate(theta0, task)];
  let status = statusOf(records, tol, maxIter);
  while (status === 'running') {
    records.push(iterate(records[records.length - 1], task, o));
    status = statusOf(records, tol, maxIter);
  }
  return { records, status };
}

// ---------------------------------------------------------------------------
// Inverse velocity kinematics (tracking a moving target)
// ---------------------------------------------------------------------------

export interface MovingTarget {
  x: number; y: number; phi: number;
  /** Time derivatives of the target pose (in {s}). */
  dx: number; dy: number; dphi: number;
}

/**
 * Joint rates θ̇ = J_b⁺ (Ad_{T_bd} V_d + K V_err): feedforward of the target's own body twist V_d
 * (moved into the current end-effector frame) plus a proportional correction of the remaining error
 * twist V_err = log(T_sb⁻¹ T_sd). For a position-only task only the linear rows are used.
 */
export function trackingRates(theta: Vec, tgt: MovingTarget, positionOnly: boolean, K: number, o: StepOptions): { thetadot: Vec; err: Residual; clipped: boolean } {
  const task: Task = { x: tgt.x, y: tgt.y, phi: tgt.phi, positionOnly };
  const err = residual(theta, task);
  let Vcmd: Vec;
  if (positionOnly) {
    const c = err.Tsb[0][0], s = err.Tsb[1][0];
    const vx = tgt.dx + K * (tgt.x - err.Tsb[0][3]);
    const vy = tgt.dy + K * (tgt.y - err.Tsb[1][3]);
    Vcmd = [c * vx + s * vy, -s * vx + c * vy];
  } else {
    const Tsd = targetTransform(task);
    const cd = Math.cos(tgt.phi), sd = Math.sin(tgt.phi);
    // Body twist of {d}: ω = φ̇ ẑ, v = R_sdᵀ ṗ_d.
    const Vd: Vec = [0, 0, tgt.dphi, cd * tgt.dx + sd * tgt.dy, -sd * tgt.dx + cd * tgt.dy, 0];
    const ff = matVec(Adjoint(matMul(TransInv(err.Tsb), Tsd)), Vd);
    const V6 = ff.map((x, i) => x + K * err.V6[i]);
    Vcmd = [V6[2], V6[3], V6[4]];
  }
  const J = reducedJacobian(theta, positionOnly);
  const st = solveStep(J, Vcmd, { ...o, maxStep: Infinity });
  return { thetadot: st.dtheta, err, clipped: st.clipped };
}

/** Wrist centre (start of the last link) of the 3R for a pose task, and whether it is reachable. */
export function poseReachable(t: Task): { reachable: boolean; w: [number, number]; r: number } {
  const [L1, L2, L3] = LINKS;
  if (t.positionOnly) {
    const r = Math.hypot(t.x, t.y);
    return { reachable: r <= REACH + 1e-12, w: [t.x, t.y], r };
  }
  const w: [number, number] = [t.x - L3 * Math.cos(t.phi), t.y - L3 * Math.sin(t.phi)];
  const r = Math.hypot(w[0], w[1]);
  return { reachable: r <= L1 + L2 + 1e-12 && r >= Math.abs(L1 - L2) - 1e-12, w, r };
}
