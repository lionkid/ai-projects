/**
 * Pure product-of-exponentials helpers for the space-form lesson. All the robotics math goes
 * through @/mr (MatrixExp6, Adjoint, FKinSpace, JacobianSpace); this module only organises the
 * partial products that the simulation animates.
 */
import {
  type Mat,
  type Vec,
  Adjoint,
  FKinSpace,
  JacobianSpace,
  MatrixExp6,
  ScrewAxisToPointDir,
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

/** Number of joints of a 6 × n screw list. */
export const jointCount = (Slist: Mat): number => Slist[0].length;

/**
 * Body transforms of the correct PoE chain: L[0] = I (ground) and
 * L[i] = e^{[S1]θ1} ⋯ e^{[Si]θi} moves everything attached to link i from home to its pose.
 * The end-effector pose is L[n] · M.
 */
export function chainTransforms(Slist: Mat, theta: Vec): Mat[] {
  const out: Mat[] = [eye(4)];
  let P = eye(4);
  for (let i = 0; i < theta.length; i++) {
    P = matMul(P, screwExp(getCol(Slist, i), theta[i]));
    out.push(P);
  }
  return out;
}

/**
 * Body transforms when the joint motions are applied in the WRONG order: joint 1 first, then
 * joint 2 about its HOME axis, … i.e. L[i] = e^{[Si]θi} ⋯ e^{[S1]θ1}. The links generally come
 * apart, which is the point of the demonstration.
 */
export function reversedTransforms(Slist: Mat, theta: Vec): Mat[] {
  const out: Mat[] = [eye(4)];
  let P = eye(4);
  for (let i = 0; i < theta.length; i++) {
    P = matMul(screwExp(getCol(Slist, i), theta[i]), P);
    out.push(P);
  }
  return out;
}

/**
 * Joint values after `stage` factors of e^{[S1]θ1} ⋯ e^{[Sn]θn} M have been applied from the
 * RIGHT (joints n, n−1, … first), plus the fraction `u` ∈ [0, 1] of the next factor.
 */
export function rightToLeftPartial(theta: Vec, stage: number, u = 0): Vec {
  const n = theta.length;
  const s = Math.max(0, Math.min(n, Math.floor(stage)));
  return theta.map((x, i) => (i >= n - s ? x : i === n - s - 1 ? u * x : 0));
}

/** Joint values after the first `stage` joints have been applied from the LEFT (+ fraction u of the next). */
export function leftToRightPartial(theta: Vec, stage: number, u = 0): Vec {
  const n = theta.length;
  const s = Math.max(0, Math.min(n, Math.floor(stage)));
  return theta.map((x, i) => (i < s ? x : i === s ? u * x : 0));
}

/** Index (0-based) of the joint whose factor is applied at `stage` → stage + 1. */
export function factorIndex(n: number, stage: number, order: 'rtl' | 'ltr'): number {
  return order === 'rtl' ? n - 1 - stage : stage;
}

/** The "current" screw axis of every joint in {s}: Ad_{e^{[S1]θ1}⋯e^{[S(i−1)]θ(i−1)}} S_i (= columns of J_s). */
export function currentAxes(Slist: Mat, theta: Vec): Vec[] {
  const L = chainTransforms(Slist, theta);
  return theta.map((_, i) => matVec(Adjoint(L[i]), getCol(Slist, i)));
}

/** Same as `currentAxes`, computed with the library's space Jacobian (used to cross-check). */
export function currentAxesFromJacobian(Slist: Mat, theta: Vec): Vec[] {
  const J = JacobianSpace(Slist, theta);
  return theta.map((_, i) => getCol(J, i));
}

/** Which current axes differ from their home axes (max-abs difference above `tol`). */
export function movedAxes(Slist: Mat, theta: Vec, tol = 1e-9): boolean[] {
  return currentAxes(Slist, theta).map((S, i) => {
    const H = getCol(Slist, i);
    return S.some((x, k) => Math.abs(x - H[k]) > tol);
  });
}

/**
 * Declare the pose θ0 the new home configuration (new joint variables φ = θ − θ0). The new end
 * frame is M' = T(θ0) and the new screw axes are the axes where the joints sit at θ0.
 */
export function rehome(M: Mat, Slist: Mat, theta0: Vec): { M: Mat; Slist: Mat } {
  const axes = currentAxes(Slist, theta0);
  const S2: Mat = [0, 1, 2, 3, 4, 5].map((r) => axes.map((a) => a[r]));
  return { M: FKinSpace(M, Slist, theta0), Slist: S2 };
}

/** Frobenius norm of the difference of two transforms. */
export function transformGap(A: Mat, B: Mat): number {
  return normFro(matSub(A, B));
}

/** Distance between the origins of two transforms. */
export function originGap(A: Mat, B: Mat): number {
  return Math.hypot(A[0][3] - B[0][3], A[1][3] - B[1][3], A[2][3] - B[2][3]);
}

/**
 * Glyph placement of a screw axis S (in {s}) for a joint located near `anchor`:
 * revolute → unit direction ω̂ and the point of the axis closest to the anchor;
 * prismatic (ω = 0) → direction v̂ through the anchor itself (a P axis has no location).
 */
export function axisPlacement(S: Vec, anchor: Vec): { dir: Vec; point: Vec; prismatic: boolean } {
  const w = [S[0], S[1], S[2]];
  if (norm(w) < 1e-9) {
    const v = [S[3], S[4], S[5]];
    const n = norm(v) || 1;
    return { dir: v.map((x) => x / n), point: anchor.slice(0, 3), prismatic: true };
  }
  const { q, s } = ScrewAxisToPointDir(S);
  const d = [anchor[0] - q[0], anchor[1] - q[1], anchor[2] - q[2]];
  const t = dot(d, s);
  return { dir: s, point: [q[0] + t * s[0], q[1] + t * s[1], q[2] + t * s[2]], prismatic: false };
}

/** Distance from a point to the line of a revolute screw axis (0 for prismatic axes). */
export function distanceToAxis(S: Vec, p: Vec): number {
  const w = [S[0], S[1], S[2]];
  if (norm(w) < 1e-9) return 0;
  const { q, s } = ScrewAxisToPointDir(S);
  return norm(cross([p[0] - q[0], p[1] - q[1], p[2] - q[2]], s));
}

/** A home-frame point carried by a body transform. */
export function carry(T: Mat, p: Vec): Vec {
  return transformPoint(T, p);
}
