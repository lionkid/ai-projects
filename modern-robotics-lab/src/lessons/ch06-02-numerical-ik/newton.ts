/**
 * Planar reduction of body-frame Newton–Raphson inverse kinematics for the lab's planar 3R arm.
 *
 * Everything is computed with the 6-D machinery of `@/mr` (FKinBody, JacobianBody, MatrixLog6) and
 * then reduced to the three planar components [ω_z; v_x; v_y] (or [v_x; v_y] for a position-only
 * task). Because the remaining rows of J_b and of the error twist are identically zero for a planar
 * chain, the reduced pseudo-inverse / DLS steps coincide with the 6-D ones (see newton.test.ts).
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

/** One update Δθ for the linearised problem J Δθ = V with the chosen method. */
export function solveStep(J: Mat, V: Vec, o: StepOptions): StepResult {
  let d: Vec;
  if (o.method === 'dls') {
    const JJt = matMul(J, transpose(J));
    for (let i = 0; i < JJt.length; i++) JJt[i][i] += o.lambda * o.lambda;
    d = matVec(transpose(J), matVec(inv(JJt), V));
  } else if (o.method === 'transpose') {
    // Jacobian-transpose method with the step size that minimises |V - α J Jᵀ V| (first-order optimal).
    const JJtV = matVec(J, matVec(transpose(J), V));
    const den = dot(JJtV, JJtV);
    const alpha = den > 1e-18 ? dot(V, JJtV) / den : 1;
    d = matVec(transpose(J), V).map((x) => x * alpha);
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
export function screwArc(Tsb: Mat, V6: Vec, samples = 24): [number, number][] {
  const out: [number, number][] = [];
  for (let k = 0; k <= samples; k++) {
    const s = k / samples;
    const T = matMul(Tsb, MatrixExp6(VecTose3(V6.map((x) => x * s))));
    out.push([T[0][3], T[1][3]]);
  }
  return out;
}

/** Convergence test with the same defaults as IKinBody (eomg = 1e-3, ev = 1e-4). */
export function converged(r: Residual, eomg = 1e-3, ev = 1e-4): boolean {
  return r.omegaNorm <= eomg && r.vNorm <= ev;
}
