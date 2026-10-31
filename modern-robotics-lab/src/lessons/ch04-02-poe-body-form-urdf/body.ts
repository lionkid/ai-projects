/**
 * Pure helpers for the body-form lesson. The robotics math goes through @/mr (MatrixExp6, Adjoint,
 * TransInv, FKinSpace / FKinBody, SlistToBlist, JacobianSpace); this module organises the
 * partial products and the comparisons the simulation shows.
 */
import {
  type Mat,
  type Vec,
  Adjoint,
  JacobianSpace,
  MatrixExp6,
  ScrewAxisToPointDir,
  SlistToBlist,
  TransInv,
  Trans,
  VecTose3,
  cross,
  dot,
  eye,
  getCol,
  matMul,
  matSub,
  matVec,
  norm,
  normFro,
  transformPoint,
} from '@/mr';

/** exp([S] θ) for one screw axis. */
export function screwExp(S: Vec, theta: number): Mat {
  return MatrixExp6(VecTose3(S.map((x) => x * theta)));
}

/** Body transforms of the chain: L[0] = I, L[i] = e^{[S1]θ1} ⋯ e^{[Si]θi} (everything on link i moves by L[i]). */
export function chainTransforms(Slist: Mat, theta: Vec): Mat[] {
  const out: Mat[] = [eye(4)];
  let P = eye(4);
  for (let i = 0; i < theta.length; i++) {
    P = matMul(P, screwExp(getCol(Slist, i), theta[i]));
    out.push(P);
  }
  return out;
}

/** Body form as a product: M e^{[B1]θ1} ⋯ e^{[Bn]θn} (written out so each factor is visible). */
export function bodyProduct(M: Mat, Blist: Mat, theta: Vec): Mat {
  let T = M;
  for (let i = 0; i < theta.length; i++) T = matMul(T, screwExp(getCol(Blist, i), theta[i]));
  return T;
}

/**
 * The expression after M has moved left past the last `k` factors:
 * e^{[S1]θ1} ⋯ e^{[S(n−k)]θ(n−k)} · M · e^{[B(n−k+1)]θ(n−k+1)} ⋯ e^{[Bn]θn}.
 * k = 0 is the space form, k = n the body form; every k gives the same transform.
 */
export function passingProduct(Slist: Mat, Blist: Mat, M: Mat, theta: Vec, k: number): Mat {
  const n = theta.length;
  const split = n - Math.max(0, Math.min(n, k));
  let left = eye(4);
  for (let i = 0; i < split; i++) left = matMul(left, screwExp(getCol(Slist, i), theta[i]));
  let right = eye(4);
  for (let i = split; i < n; i++) right = matMul(right, screwExp(getCol(Blist, i), theta[i]));
  return matMul(matMul(left, M), right);
}

/** Joint values after the first `stage` factors of M e^{[B1]θ1} ⋯ have been applied (+ fraction u of the next). */
export function bodyPartial(theta: Vec, stage: number, u = 0): Vec {
  const s = Math.max(0, Math.min(theta.length, Math.floor(stage)));
  return theta.map((x, i) => (i < s ? x : i === s ? u * x : 0));
}

/** End-effector frame with a tool of length d along ẑ_b: M_tool = M · Trans(0, 0, d). */
export function toolFrame(M: Mat, d: number): Mat {
  return matMul(M, Trans(0, 0, d));
}

/** Body screw axes for a given end frame (thin wrapper so the lesson reads like the formula). */
export function bodyAxes(Slist: Mat, M: Mat): Mat {
  return SlistToBlist(Slist, M);
}

/**
 * A screw axis that is rigidly glued to the end-effector frame: B (coordinates in {b}) seen from
 * {s} when {b} is at T. With the end frame M at home this is Ad_{T M⁻¹} S.
 */
export function gluedToB(T: Mat, Mhome: Mat, S: Vec): Vec {
  return matVec(Adjoint(matMul(T, TransInv(Mhome))), S);
}

/** The current axis of every joint in {s} (columns of the space Jacobian). */
export function currentAxes(Slist: Mat, theta: Vec): Vec[] {
  const J = JacobianSpace(Slist, theta);
  return theta.map((_, i) => getCol(J, i));
}

/** True when two normalised revolute screw axes describe the same directed line. */
export function sameLine(A: Vec, B: Vec, tol = 1e-9): boolean {
  return A.every((x, k) => Math.abs(x - B[k]) < tol);
}

/** Velocity (in {s}) of the point p when only joint j turns at 1 rad/s: v_s + ω_s × p with V_s = J_s,j. */
export function pointVelocity(Slist: Mat, theta: Vec, j: number, p: Vec): Vec {
  const V = getCol(JacobianSpace(Slist, theta), j);
  const w = V.slice(0, 3);
  const c = cross(w, p);
  return [V[3] + c[0], V[4] + c[1], V[5] + c[2]];
}

/** Which columns of two screw lists differ (max-abs difference above tol). */
export function changedColumns(A: Mat, B: Mat, tol = 1e-9): boolean[] {
  return A[0].map((_, j) => A.some((rowA, r) => Math.abs(rowA[j] - B[r][j]) > tol));
}

export function transformGap(A: Mat, B: Mat): number {
  return normFro(matSub(A, B));
}

/**
 * Glyph placement of a screw axis: revolute → unit direction and the point of the axis nearest the
 * anchor; prismatic (ω = 0) → direction v̂ through the anchor (the UR5 has none, kept for safety).
 */
export function axisPlacement(S: Vec, anchor: Vec): { dir: Vec; point: Vec; prismatic: boolean } {
  const w = [S[0], S[1], S[2]];
  if (norm(w) < 1e-9) {
    const v = [S[3], S[4], S[5]];
    const nv = norm(v) || 1;
    return { dir: v.map((x) => x / nv), point: anchor.slice(0, 3), prismatic: true };
  }
  const { q, s } = ScrewAxisToPointDir(S);
  const t = dot([anchor[0] - q[0], anchor[1] - q[1], anchor[2] - q[2]], s);
  return { dir: s, point: [q[0] + t * s[0], q[1] + t * s[1], q[2] + t * s[2]], prismatic: false };
}

/** A home-frame point carried by a transform. */
export function carry(T: Mat, p: Vec): Vec {
  return transformPoint(T, p);
}
