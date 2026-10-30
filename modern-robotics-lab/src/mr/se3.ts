/**
 * Rigid-body motions: SE(3), its Lie algebra se(3), twists, screws and the
 * adjoint representation.
 *
 * Conventions
 *  - A transform T is a 4x4 `Mat` [[R, p], [0, 1]] mapping body coordinates to
 *    space coordinates.
 *  - A twist V is a 6-vector [omega; v] (angular part FIRST, as in the book);
 *    a wrench F is [m; f] (moment first). `[V]` is the 4x4 matrix from
 *    `VecTose3`.
 *  - A screw axis S is a normalised twist: either |omega| = 1, or omega = 0
 *    and |v| = 1.
 */

import {
  type Mat,
  type Vec,
  block,
  cross,
  det,
  dot,
  eye,
  matAdd,
  matMul,
  matScale,
  matSub,
  matVec,
  norm,
  normFro,
  normalize,
  transpose,
  vecAdd,
  vecScale,
  zeros,
} from './linalg';
import { MatrixExp3, MatrixLog3, ProjectToSO3, VecToso3, rodriguesCoefficients, so3ToVec } from './so3';

/** Builds the 4x4 transform from a rotation matrix and a position vector. */
export function RpToTrans(R: Mat, p: Vec): Mat {
  return [
    [R[0][0], R[0][1], R[0][2], p[0]],
    [R[1][0], R[1][1], R[1][2], p[1]],
    [R[2][0], R[2][1], R[2][2], p[2]],
    [0, 0, 0, 1],
  ];
}

/** Splits a transform into its rotation matrix and position vector. */
export function TransToRp(T: Mat): [Mat, Vec] {
  const R: Mat = [
    [T[0][0], T[0][1], T[0][2]],
    [T[1][0], T[1][1], T[1][2]],
    [T[2][0], T[2][1], T[2][2]],
  ];
  return [R, [T[0][3], T[1][3], T[2][3]]];
}

/** Inverse of a transform computed as [[R^T, -R^T p], [0, 1]] (no general matrix inverse needed). */
export function TransInv(T: Mat): Mat {
  const [R, p] = TransToRp(T);
  const Rt = transpose(R);
  return RpToTrans(Rt, vecScale(matVec(Rt, p), -1));
}

/** 4x4 se(3) matrix [V] = [[ [omega], v ], [0, 0]] of a twist V = [omega; v]. */
export function VecTose3(V: Vec): Mat {
  return [
    [0, -V[2], V[1], V[3]],
    [V[2], 0, -V[0], V[4]],
    [-V[1], V[0], 0, V[5]],
    [0, 0, 0, 0],
  ];
}

/** Inverse of `VecTose3`. */
export function se3ToVec(se3mat: Mat): Vec {
  return [se3mat[2][1], se3mat[0][2], se3mat[1][0], se3mat[0][3], se3mat[1][3], se3mat[2][3]];
}

/**
 * 6x6 adjoint representation [Ad_T] = [[R, 0], [[p] R, R]] that changes the
 * frame of a twist: V_a = [Ad_{T_ab}] V_b.
 */
export function Adjoint(T: Mat): Mat {
  const [R, p] = TransToRp(T);
  return block([
    [R, zeros(3)],
    [matMul(VecToso3(p), R), R],
  ]);
}

/**
 * Screw axis S = [omega; v] of a screw through point q with unit direction s
 * and pitch h (m per radian): omega = s, v = -s x q + h s.
 */
export function ScrewToAxis(q: Vec, s: Vec, h: number): Vec {
  const v = vecAdd(cross(q, s), vecScale(s, h)); // q x s = -(s x q)
  return [s[0], s[1], s[2], v[0], v[1], v[2]];
}

/**
 * Inverse of `ScrewToAxis` for a normalised screw axis: returns the point on
 * the axis closest to the origin, the unit direction and the pitch. For a pure
 * translation (omega = 0) the pitch is reported as Infinity and q = 0.
 */
export function ScrewAxisToPointDir(S: Vec): { q: Vec; s: Vec; h: number } {
  const omega = S.slice(0, 3);
  const v = S.slice(3, 6);
  const w2 = dot(omega, omega);
  if (w2 < 1e-12) return { q: [0, 0, 0], s: normalize(v), h: Infinity };
  const h = dot(omega, v) / w2;
  const q = vecScale(cross(omega, v), 1 / w2);
  return { q, s: vecScale(omega, 1 / Math.sqrt(w2)), h };
}

/**
 * Splits exponential coordinates `expc6 = S * theta` into the unit screw axis
 * and the scalar theta (theta = |omega|, or |v| when omega = 0).
 */
export function AxisAng6(expc6: Vec): [Vec, number] {
  let theta = norm(expc6.slice(0, 3));
  if (theta < 1e-12) theta = norm(expc6.slice(3, 6));
  if (theta === 0) return [expc6.slice(), 0];
  return [vecScale(expc6, 1 / theta), theta];
}

/**
 * Matrix exponential of an se(3) matrix [S] theta. With W = [omega theta]:
 *   exp([V]) = [[exp(W), G v], [0, 1]],  G = I + B W + C W^2
 * (see `rodriguesCoefficients`); a pure translation is handled by the same
 * formula since W = 0 gives G = I.
 */
export function MatrixExp6(se3mat: Mat): Mat {
  const W: Mat = [
    [se3mat[0][0], se3mat[0][1], se3mat[0][2]],
    [se3mat[1][0], se3mat[1][1], se3mat[1][2]],
    [se3mat[2][0], se3mat[2][1], se3mat[2][2]],
  ];
  const v: Vec = [se3mat[0][3], se3mat[1][3], se3mat[2][3]];
  const theta = norm(so3ToVec(W));
  const { B, C } = rodriguesCoefficients(theta);
  const W2 = matMul(W, W);
  const R = MatrixExp3(W);
  const G = matAdd(matAdd(eye(3), matScale(W, B)), matScale(W2, C));
  return RpToTrans(R, matVec(G, v));
}

/**
 * Matrix logarithm of a transform: the se(3) matrix [S] theta with theta in
 * [0, pi]. The translation part is G^{-1} p with
 *   G^{-1} = I - W/2 + D W^2,  D = 1/theta^2 - cot(theta/2) / (2 theta),
 * where W = MatrixLog3(R); D is evaluated by its series for small theta.
 */
export function MatrixLog6(T: Mat): Mat {
  const [R, p] = TransToRp(T);
  const W = MatrixLog3(R);
  const theta = norm(so3ToVec(W));
  if (theta === 0) {
    return [
      [0, 0, 0, p[0]],
      [0, 0, 0, p[1]],
      [0, 0, 0, p[2]],
      [0, 0, 0, 0],
    ];
  }
  const t2 = theta * theta;
  let D: number;
  if (theta < 1e-3) {
    D = 1 / 12 + t2 / 720 + (t2 * t2) / 30240;
  } else {
    D = 1 / t2 - Math.cos(theta / 2) / (2 * theta * Math.sin(theta / 2));
  }
  const Ginv = matAdd(matSub(eye(3), matScale(W, 0.5)), matScale(matMul(W, W), D));
  const v = matVec(Ginv, p);
  return [
    [W[0][0], W[0][1], W[0][2], v[0]],
    [W[1][0], W[1][1], W[1][2], v[1]],
    [W[2][0], W[2][1], W[2][2], v[2]],
    [0, 0, 0, 0],
  ];
}

/**
 * 6x6 matrix [ad_V] = [[ [omega], 0 ], [ [v], [omega] ]] such that
 * [ad_V1] V2 is the Lie bracket of the twists V1 and V2.
 */
export function ad(V: Vec): Mat {
  const omg = VecToso3([V[0], V[1], V[2]]);
  const vm = VecToso3([V[3], V[4], V[5]]);
  return block([
    [omg, zeros(3)],
    [vm, omg],
  ]);
}

/** Projects a 4x4 matrix onto SE(3) (rotation part via `ProjectToSO3`, translation kept). */
export function ProjectToSE3(mat: Mat): Mat {
  return RpToTrans(ProjectToSO3([mat[0].slice(0, 3), mat[1].slice(0, 3), mat[2].slice(0, 3)]), [
    mat[0][3],
    mat[1][3],
    mat[2][3],
  ]);
}

/**
 * Distance of a 4x4 matrix from SE(3): the Frobenius norm of
 * [[R^T R, 0], [last row of mat]] - I when det R > 0, and 1e9 otherwise.
 */
export function DistanceToSE3(mat: Mat): number {
  const R: Mat = [mat[0].slice(0, 3), mat[1].slice(0, 3), mat[2].slice(0, 3)];
  if (det(R) <= 0) return 1e9;
  const RtR = matMul(transpose(R), R);
  const test: Mat = [
    [RtR[0][0], RtR[0][1], RtR[0][2], 0],
    [RtR[1][0], RtR[1][1], RtR[1][2], 0],
    [RtR[2][0], RtR[2][1], RtR[2][2], 0],
    [mat[3][0], mat[3][1], mat[3][2], mat[3][3]],
  ];
  return normFro(matSub(test, eye(4)));
}

/** True when the matrix is within 1e-3 of being a rigid-body transform. */
export function TestIfSE3(mat: Mat): boolean {
  return DistanceToSE3(mat) < 1e-3;
}

/** Applies T to a point: R p + t. */
export function transformPoint(T: Mat, p: Vec): Vec {
  return [
    T[0][0] * p[0] + T[0][1] * p[1] + T[0][2] * p[2] + T[0][3],
    T[1][0] * p[0] + T[1][1] * p[1] + T[1][2] * p[2] + T[1][3],
    T[2][0] * p[0] + T[2][1] * p[1] + T[2][2] * p[2] + T[2][3],
  ];
}

/** Applies only the rotation part of T to a free vector: R v. */
export function transformVector(T: Mat, v: Vec): Vec {
  return [
    T[0][0] * v[0] + T[0][1] * v[1] + T[0][2] * v[2],
    T[1][0] * v[0] + T[1][1] * v[1] + T[1][2] * v[2],
    T[2][0] * v[0] + T[2][1] * v[1] + T[2][2] * v[2],
  ];
}

/** Pure translation transform. */
export function Trans(x: number, y: number, z: number): Mat {
  return RpToTrans(eye(3), [x, y, z]);
}
