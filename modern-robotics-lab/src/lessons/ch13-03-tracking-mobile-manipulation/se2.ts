/**
 * Planar rigid-body helpers for ch13-03, written on top of the SE(3) routines of @/mr.
 *
 * A planar pose q = (φ, x, y) (the book's chapter-13 ordering) is embedded in SE(3) as a
 * rotation about z plus a translation in the xy plane; a planar twist V = [ω, v_x, v_y] is the
 * 6-vector [0, 0, ω, v_x, v_y, 0]. Exponentials, logarithms and adjoints are evaluated with
 * MatrixExp6 / MatrixLog6 / Adjoint and the planar rows/columns are extracted, so the lesson
 * reuses the library instead of re-deriving the SE(2) formulas.
 */
import { Adjoint, MatrixExp6, MatrixLog6, VecTose3, matMul, se2, se3ToVec, transInv2, wrapToPi, type Mat } from '@/mr';

/** Planar pose (heading φ in rad, position x, y in m). Headings are kept unwrapped. */
export interface Pose { phi: number; x: number; y: number }
/** Planar twist [ω, v_x, v_y] (angular part first, like the 6-vector twists of @/mr). */
export type Twist2 = [number, number, number];

export const pose = (phi: number, x: number, y: number): Pose => ({ phi, x, y });

/** 4×4 homogeneous transform of a planar pose. */
export function poseToT(q: Pose): Mat {
  const c = Math.cos(q.phi), s = Math.sin(q.phi);
  return [[c, -s, 0, q.x], [s, c, 0, q.y], [0, 0, 1, 0], [0, 0, 0, 1]];
}

/** 3×3 planar transform → pose (heading in [−π, π)). */
export function t2ToPose(T: Mat): Pose {
  return { phi: Math.atan2(T[1][0], T[0][0]), x: T[0][2], y: T[1][2] };
}

/** 3×3 planar transform → 4×4 transform. */
export function t2ToT(T: Mat): Mat {
  return [[T[0][0], T[0][1], 0, T[0][2]], [T[1][0], T[1][1], 0, T[1][2]], [0, 0, 1, 0], [0, 0, 0, 1]];
}

/** Composition a·b; the heading is the plain sum φ_a + φ_b (never wrapped). */
export function compose(a: Pose, b: Pose): Pose {
  const T = matMul(se2(a.phi, a.x, a.y), se2(b.phi, b.x, b.y));
  return { phi: a.phi + b.phi, x: T[0][2], y: T[1][2] };
}

/** Relative pose a⁻¹·b, i.e. b seen from a; heading difference wrapped to [−π, π). */
export function between(a: Pose, b: Pose): Pose {
  const T = matMul(transInv2(se2(a.phi, a.x, a.y)), se2(b.phi, b.x, b.y));
  return { phi: wrapToPi(b.phi - a.phi), x: T[0][2], y: T[1][2] };
}

/** Planar displacement exp([V] t) computed with MatrixExp6 (Δφ = ω t, not wrapped). */
export function expTwist(V: Twist2, t: number): Pose {
  const T = MatrixExp6(VecTose3([0, 0, V[0] * t, V[1] * t, V[2] * t, 0]));
  return { phi: V[0] * t, x: T[0][3], y: T[1][3] };
}

/** Moves a pose with the constant body twist V for t seconds: q·exp([V] t). */
export function advance(q: Pose, V: Twist2, t: number): Pose {
  return compose(q, expTwist(V, t));
}

/** Twist coordinates of a relative pose: log(E) with MatrixLog6 (|ω| ≤ π). */
export function logPose(E: Pose): Twist2 {
  const v = se3ToVec(MatrixLog6(poseToT({ phi: wrapToPi(E.phi), x: E.x, y: E.y })));
  return [v[2], v[3], v[4]];
}

const PLANAR = [2, 3, 4];

/** Planar 3×3 block of a 6×6 matrix acting on twists (rows/cols ω_z, v_x, v_y). */
export function planarBlock(A6: Mat): Mat {
  return PLANAR.map((i) => PLANAR.map((j) => A6[i][j]));
}

/** 3×3 planar adjoint [Ad_E] (maps a twist expressed in the child frame to the parent frame). */
export function adjointPose(E: Pose): Mat {
  return planarBlock(Adjoint(poseToT(E)));
}

/** 3×3 planar adjoint of a 3×3 planar transform. */
export function adjointT2(T: Mat): Mat {
  return planarBlock(Adjoint(t2ToT(T)));
}

/** 3×3 matrix times 3-vector. */
export function mul3(A: Mat, v: ArrayLike<number>): Twist2 {
  return [
    A[0][0] * v[0] + A[0][1] * v[1] + A[0][2] * v[2],
    A[1][0] * v[0] + A[1][1] * v[1] + A[1][2] * v[2],
    A[2][0] * v[0] + A[2][1] * v[1] + A[2][2] * v[2],
  ];
}

/** World-frame point of a body-frame point p_b seen from pose q. */
export function toWorld(q: Pose, p: [number, number]): [number, number] {
  const c = Math.cos(q.phi), s = Math.sin(q.phi);
  return [q.x + c * p[0] - s * p[1], q.y + s * p[0] + c * p[1]];
}
