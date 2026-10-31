/**
 * Pure helpers for the "homogeneous transforms, twists and the Adjoint" lesson.
 * No DOM and no three.js here, so everything can be unit-tested under vitest (node).
 * All rigid-body math goes through @/mr; this module only composes it for the demos.
 */
import {
  type Mat, type Vec, Adjoint, MatrixExp6, MatrixLog6, RpToTrans, ScrewAxisToPointDir, TransInv, TransToRp, VecTose3,
  cross, dot, matMul, matScale, matSub, matVec, norm, rotZYX, se3ToVec, transformPoint, transpose, vecAdd, vecScale, vecSub,
} from '@/mr';

/** Pose parameters used by the sliders: position (m) and ZYX angles (rad), R = Rot(z, yaw) Rot(y, pitch) Rot(x, roll). */
export interface PoseParams { x: number; y: number; z: number; yaw: number; pitch: number; roll: number }

export function poseFromParams(P: PoseParams): Mat {
  return RpToTrans(rotZYX([P.yaw, P.pitch, P.roll]), [P.x, P.y, P.z]);
}

// ---------------------------------------------------------------------------
// Inverse and the usual mistakes
// ---------------------------------------------------------------------------

/**
 * Candidates for T^{-1}:
 *  - 'correct'   [Rᵀ, −Rᵀp; 0 1] (= TransInv)
 *  - 'negp'      [Rᵀ, −p; 0 1]   (transpose R, negate p: a frequent mistake)
 *  - 'transpose' the full 4×4 transpose (not even a rigid transform: its last row is [pᵀ 1])
 */
export type InverseKind = 'correct' | 'negp' | 'transpose';

export function inverseCandidate(T: Mat, kind: InverseKind): Mat {
  if (kind === 'correct') return TransInv(T);
  if (kind === 'transpose') return transpose(T);
  const [R, p] = TransToRp(T);
  return RpToTrans(transpose(R), vecScale(p, -1));
}

/** True when the last row is [0 0 0 1] (necessary for a homogeneous transform). */
export function hasHomogeneousRow(T: Mat, tol = 1e-9): boolean {
  const r = T[3];
  return Math.abs(r[0]) < tol && Math.abs(r[1]) < tol && Math.abs(r[2]) < tol && Math.abs(r[3] - 1) < tol;
}

/** Largest absolute entry of A − I (how far a product that "should be" the identity is from it). */
export function distanceFromIdentity(A: Mat): number {
  let m = 0;
  for (let i = 0; i < A.length; i++) for (let j = 0; j < A[i].length; j++) m = Math.max(m, Math.abs(A[i][j] - (i === j ? 1 : 0)));
  return m;
}

// ---------------------------------------------------------------------------
// Pre- and post-multiplication, applied gradually
// ---------------------------------------------------------------------------

/** T^u = exp(u log T): a fraction u ∈ [0, 1] of a rigid displacement (rotation angle below π). */
export function partialTransform(T: Mat, u: number): Mat {
  return MatrixExp6(matScale(MatrixLog6(T), u));
}

export type Side = 'pre' | 'post';

/** 'pre': T^u T_sb (T acts along the fixed {s} axes); 'post': T_sb T^u (T acts along the moving {b} axes). */
export function applyFraction(T: Mat, Tsb: Mat, u: number, side: Side): Mat {
  const Tu = partialTransform(T, u);
  return side === 'pre' ? matMul(Tu, Tsb) : matMul(Tsb, Tu);
}

/** Positions of the {b} origin while a fraction u = 0 … 1 of T is applied (for drawing the path). */
export function originPath(T: Mat, Tsb: Mat, side: Side, n = 48): Vec[] {
  const pts: Vec[] = [];
  for (let k = 0; k <= n; k++) {
    const P = applyFraction(T, Tsb, k / n, side);
    pts.push([P[0][3], P[1][3], P[2][3]]);
  }
  return pts;
}

// ---------------------------------------------------------------------------
// Twists
// ---------------------------------------------------------------------------

/**
 * Pose after t seconds of motion with a constant body twist: T(t) = T0 exp([V_b] t).
 * This is the exact solution of Ṫ = T [V_b] (no Euler drift, T stays in SE(3)).
 */
export function constantBodyTwistPose(T0: Mat, Vb: Vec, t: number): Mat {
  return matMul(T0, MatrixExp6(matScale(VecTose3(Vb), t)));
}

/** [V_s] = Ṫ T⁻¹ and [V_b] = T⁻¹ Ṫ (both read back as 6-vectors [ω; v]). */
export function twistsFromDerivative(T: Mat, Tdot: Mat): { Vs: Vec; Vb: Vec } {
  const Ti = TransInv(T);
  return { Vs: se3ToVec(matMul(Tdot, Ti)), Vb: se3ToVec(matMul(Ti, Tdot)) };
}

/** Central-difference time derivative of a pose trajectory (error O(h²)). */
export function numericDerivative(f: (t: number) => Mat, t: number, h = 1e-4): Mat {
  return matScale(matSub(f(t + h), f(t - h)), 1 / (2 * h));
}

/**
 * Velocity of the body point (or of the body's rigid extension) that is currently at r,
 * when the twist V = [ω; v] and r are written in the same frame: ṙ = ω × r + v.
 * In particular v itself is the velocity of the point at that frame's origin.
 */
export function pointVelocity(V: Vec, r: Vec): Vec {
  return vecAdd(cross(V.slice(0, 3), r), V.slice(3, 6));
}

/** Twist of the same motion written in another frame: V_a = [Ad_{T_ab}] V_b. */
export function changeFrame(Tab: Mat, Vb: Vec): Vec {
  return matVec(Adjoint(Tab), Vb);
}

export type ScrewInfo =
  | { kind: 'zero' }
  | { kind: 'translation'; dir: Vec; speed: number }
  | { kind: 'screw'; q: Vec; dir: Vec; h: number; rate: number };

/**
 * Instantaneous screw axis of a twist (in the twist's own frame): the line through q (the point of
 * the axis closest to the frame origin) with direction ω̂, and the pitch h = ωᵀv / ‖ω‖².
 * ω = 0 is a pure translation (pitch "∞"), and V = 0 has no axis at all.
 */
export function screwInfo(V: Vec, eps = 1e-9): ScrewInfo {
  const w = V.slice(0, 3);
  const wn = norm(w);
  if (wn < eps) {
    const v = V.slice(3, 6);
    const vn = norm(v);
    return vn < eps ? { kind: 'zero' } : { kind: 'translation', dir: vecScale(v, 1 / vn), speed: vn };
  }
  const { q, s, h } = ScrewAxisToPointDir(V);
  return { kind: 'screw', q, dir: s, h, rate: wn };
}

/** Point of the line {q + λ d} closest to p (d a unit vector). */
export function closestPointOnLine(q: Vec, d: Vec, p: Vec): Vec {
  return vecAdd(q, vecScale(d, dot(vecSub(p, q), d)));
}

/** Positions of a body-fixed point r_b while the body moves with constant V_b for t ∈ [0, D]. */
export function bodyPointPath(T0: Mat, Vb: Vec, rb: Vec, D: number, n = 120): Vec[] {
  const pts: Vec[] = [];
  for (let k = 0; k <= n; k++) pts.push(transformPoint(constantBodyTwistPose(T0, Vb, (D * k) / n), rb));
  return pts;
}

/**
 * Loop length for the twist animation: one full turn (2π/‖ω‖) when the body rotates, clamped
 * to [3, 12] s so slow rotations and pure translations still loop in a reasonable time.
 */
export function loopDuration(Vb: Vec): number {
  const w = norm(Vb.slice(0, 3));
  if (w < 1e-6) return 5;
  return Math.min(12, Math.max(3, (2 * Math.PI) / w));
}

/** The 3×3 blocks of [Ad_T] = [[R, 0], [[p]R, R]], for labelling. */
export function adjointBlocks(T: Mat): { R: Mat; pR: Mat; Ad: Mat } {
  const Ad = Adjoint(T);
  const R = [0, 1, 2].map((i) => Ad[i].slice(0, 3));
  const pR = [3, 4, 5].map((i) => Ad[i].slice(0, 3));
  return { R, pR, Ad };
}
