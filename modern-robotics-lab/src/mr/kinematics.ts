/**
 * Product-of-exponentials forward kinematics, Jacobians and numerical inverse
 * kinematics for open chains (book chapters 4-6).
 *
 * Conventions (identical to the book)
 *  - `M` is the end-effector home configuration (4x4).
 *  - `Slist` / `Blist` are 6 x n matrices whose COLUMNS are the joint screw
 *    axes expressed in the space frame {s} / the end-effector frame {b} at the
 *    home configuration.
 *  - `thetalist` is the joint vector (length n, radians or metres).
 */

import {
  type Mat,
  type Vec,
  eigSym,
  eye,
  getCol,
  inv,
  matMul,
  matVec,
  norm,
  pinv,
  transpose,
  vecAdd,
  zeros,
} from './linalg';
import { Adjoint, MatrixExp6, MatrixLog6, TransInv, VecTose3, se3ToVec } from './se3';

/** exp([S] theta) for a screw axis given as a 6-vector. */
function screwExp(S: Vec, theta: number): Mat {
  return MatrixExp6(VecTose3(S.map((x) => x * theta)));
}

/** Forward kinematics in the body form: T = M exp([B1]θ1) ... exp([Bn]θn). */
export function FKinBody(M: Mat, Blist: Mat, thetalist: Vec): Mat {
  let T = M;
  for (let i = 0; i < thetalist.length; i++) {
    T = matMul(T, screwExp(getCol(Blist, i), thetalist[i]));
  }
  return T;
}

/** Forward kinematics in the space form: T = exp([S1]θ1) ... exp([Sn]θn) M. */
export function FKinSpace(M: Mat, Slist: Mat, thetalist: Vec): Mat {
  let T = M;
  for (let i = thetalist.length - 1; i >= 0; i--) {
    T = matMul(screwExp(getCol(Slist, i), thetalist[i]), T);
  }
  return T;
}

/**
 * Cumulative space-frame products P_i = exp([S1]θ1) ... exp([Si]θi) for
 * i = 1..n. P_i moves anything attached to link i (joint axis i+1, link
 * geometry) from its home pose to the current pose, which is exactly what a
 * renderer needs. `spaceJointTransforms(...)[n]` times M is the FK result.
 */
export function spaceJointTransforms(Slist: Mat, thetalist: Vec): Mat[] {
  const out: Mat[] = [];
  let P = eye(4);
  for (let i = 0; i < thetalist.length; i++) {
    P = matMul(P, screwExp(getCol(Slist, i), thetalist[i]));
    out.push(P);
  }
  return out;
}

/** Converts space-frame screw axes to body-frame ones: Blist = [Ad_{M^-1}] Slist. */
export function SlistToBlist(Slist: Mat, M: Mat): Mat {
  return matMul(Adjoint(TransInv(M)), Slist);
}

/** Converts body-frame screw axes to space-frame ones: Slist = [Ad_M] Blist. */
export function BlistToSlist(Blist: Mat, M: Mat): Mat {
  return matMul(Adjoint(M), Blist);
}

/**
 * Body Jacobian J_b(θ) (6 x n): V_b = J_b(θ) θdot. Column i is B_i transported
 * through the joints that come AFTER it:
 *   J_bi = [Ad_{exp(-[Bn]θn) ... exp(-[B_{i+1}]θ_{i+1})}] B_i.
 */
export function JacobianBody(Blist: Mat, thetalist: Vec): Mat {
  const n = thetalist.length;
  const J = zeros(6, n);
  let T = eye(4);
  for (let i = n - 1; i >= 0; i--) {
    const Bi = getCol(Blist, i);
    if (i < n - 1) {
      T = matMul(T, screwExp(getCol(Blist, i + 1), -thetalist[i + 1]));
    }
    const col = matVec(Adjoint(T), Bi);
    for (let r = 0; r < 6; r++) J[r][i] = col[r];
  }
  return J;
}

/**
 * Space Jacobian J_s(θ) (6 x n): V_s = J_s(θ) θdot. Column i is S_i transported
 * through the joints that come BEFORE it:
 *   J_si = [Ad_{exp([S1]θ1) ... exp([S_{i-1}]θ_{i-1})}] S_i.
 */
export function JacobianSpace(Slist: Mat, thetalist: Vec): Mat {
  const n = thetalist.length;
  const J = zeros(6, n);
  let T = eye(4);
  for (let i = 0; i < n; i++) {
    if (i > 0) T = matMul(T, screwExp(getCol(Slist, i - 1), thetalist[i - 1]));
    const col = matVec(Adjoint(T), getCol(Slist, i));
    for (let r = 0; r < 6; r++) J[r][i] = col[r];
  }
  return J;
}

/**
 * Manipulability measures of a Jacobian block (e.g. the 3 x n linear part).
 * The ellipsoid semi-axes are the singular values sigma_i of J (square roots
 * of the eigenvalues of A = J J^T); returned are the volume proportional to
 * prod(sigma_i), the isotropy ratio sigma_max / sigma_min (1 = isotropic,
 * Infinity = singular) and the extreme singular values themselves.
 */
export function manipulability(J: Mat): { volume: number; isotropy: number; sigmaMin: number; sigmaMax: number } {
  const { values } = eigSym(matMul(J, transpose(J)));
  const sig = values.map((v) => Math.sqrt(Math.max(v, 0)));
  const sigmaMax = sig[0];
  const sigmaMin = sig[sig.length - 1];
  const volume = sig.reduce((p, s) => p * s, 1);
  return { volume, isotropy: sigmaMin > 0 ? sigmaMax / sigmaMin : Infinity, sigmaMin, sigmaMax };
}

export interface IKOptions {
  /** Maximum number of Newton-Raphson iterations (default 20). */
  maxIterations?: number;
  /**
   * Damping factor for damped least squares (Levenberg-Marquardt style):
   *   Δθ = J^T (J J^T + λ² I)^-1 V.
   * 0 (default) uses the plain Moore-Penrose pseudo-inverse.
   */
  lambda?: number;
  /** Optional cap on |Δθ| per iteration (radians); useful for animations. */
  maxStep?: number;
}

export interface IKIteration {
  /** Joint vector at the START of this iteration (copy). */
  theta: Vec;
  /** |omega_b| of the remaining error twist. */
  errorOmega: number;
  /** |v_b| of the remaining error twist. */
  errorV: number;
}

export interface IKResult {
  thetalist: Vec;
  /** True when both error tolerances were met. */
  success: boolean;
  /** Number of Newton-Raphson updates performed. */
  iterations: number;
  /** One entry per evaluated configuration (iterations + 1 entries), for replaying in the UI. */
  history: IKIteration[];
}

function dampedStep(J: Mat, V: Vec, lambda: number): Vec {
  const JJt = matMul(J, transpose(J));
  for (let i = 0; i < JJt.length; i++) JJt[i][i] += lambda * lambda;
  return matVec(transpose(J), matVec(inv(JJt), V));
}

function limitStep(d: Vec, maxStep?: number): Vec {
  if (maxStep === undefined) return d;
  const len = norm(d);
  return len > maxStep ? d.map((x) => (x * maxStep) / len) : d;
}

/**
 * Numerical inverse kinematics in the body frame (Newton-Raphson).
 * `T` is the desired end-effector configuration, `thetalist0` the initial
 * guess; `eomg` / `ev` are the tolerances on the angular and linear parts of
 * the body error twist V_b = log(T_sb^-1 T).
 */
export function IKinBody(
  Blist: Mat,
  M: Mat,
  T: Mat,
  thetalist0: Vec,
  eomg = 1e-3,
  ev = 1e-4,
  opts: IKOptions = {},
): IKResult {
  const maxIterations = opts.maxIterations ?? 20;
  const lambda = opts.lambda ?? 0;
  let theta = thetalist0.slice();
  const errorTwist = (th: Vec): Vec => se3ToVec(MatrixLog6(matMul(TransInv(FKinBody(M, Blist, th)), T)));
  let Vb = errorTwist(theta);
  let eo = norm(Vb.slice(0, 3));
  let el = norm(Vb.slice(3, 6));
  const history: IKIteration[] = [{ theta: theta.slice(), errorOmega: eo, errorV: el }];
  let it = 0;
  while ((eo > eomg || el > ev) && it < maxIterations) {
    const J = JacobianBody(Blist, theta);
    const step = lambda > 0 ? dampedStep(J, Vb, lambda) : matVec(pinv(J), Vb);
    theta = vecAdd(theta, limitStep(step, opts.maxStep));
    it++;
    Vb = errorTwist(theta);
    eo = norm(Vb.slice(0, 3));
    el = norm(Vb.slice(3, 6));
    history.push({ theta: theta.slice(), errorOmega: eo, errorV: el });
  }
  return { thetalist: theta, success: !(eo > eomg || el > ev), iterations: it, history };
}

/**
 * Numerical inverse kinematics in the space frame. The error twist is
 * V_s = [Ad_{T_sb}] log(T_sb^-1 T) and the update uses the space Jacobian.
 * Note that `errorOmega` / `errorV` are then norms of the SPACE twist.
 */
export function IKinSpace(
  Slist: Mat,
  M: Mat,
  T: Mat,
  thetalist0: Vec,
  eomg = 1e-3,
  ev = 1e-4,
  opts: IKOptions = {},
): IKResult {
  const maxIterations = opts.maxIterations ?? 20;
  const lambda = opts.lambda ?? 0;
  let theta = thetalist0.slice();
  const errorTwist = (th: Vec): Vec => {
    const Tsb = FKinSpace(M, Slist, th);
    return matVec(Adjoint(Tsb), se3ToVec(MatrixLog6(matMul(TransInv(Tsb), T))));
  };
  let Vs = errorTwist(theta);
  let eo = norm(Vs.slice(0, 3));
  let el = norm(Vs.slice(3, 6));
  const history: IKIteration[] = [{ theta: theta.slice(), errorOmega: eo, errorV: el }];
  let it = 0;
  while ((eo > eomg || el > ev) && it < maxIterations) {
    const J = JacobianSpace(Slist, theta);
    const step = lambda > 0 ? dampedStep(J, Vs, lambda) : matVec(pinv(J), Vs);
    theta = vecAdd(theta, limitStep(step, opts.maxStep));
    it++;
    Vs = errorTwist(theta);
    eo = norm(Vs.slice(0, 3));
    el = norm(Vs.slice(3, 6));
    history.push({ theta: theta.slice(), errorOmega: eo, errorV: el });
  }
  return { thetalist: theta, success: !(eo > eomg || el > ev), iterations: it, history };
}
