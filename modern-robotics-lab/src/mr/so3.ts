/**
 * Rotations: SO(3), its Lie algebra so(3) and exponential coordinates.
 *
 * Function names follow the open-source Modern Robotics code library so that
 * learners can cross-reference, but all implementations here are original.
 *
 * Conventions
 *  - A rotation matrix R is a 3x3 `Mat`; R maps body-frame coordinates to
 *    space-frame coordinates.
 *  - `omg` is an angular-velocity / rotation-axis 3-vector; `[omg]` denotes
 *    the 3x3 skew-symmetric matrix produced by `VecToso3`.
 *  - Exponential coordinates `omg * theta` with `theta` in [0, pi].
 */

import {
  type Mat,
  type Vec,
  clamp,
  det,
  dot,
  eye,
  matAdd,
  matMul,
  matScale,
  matSub,
  nearZero,
  norm,
  normFro,
  normalize,
  svd,
  transpose,
  vecScale,
} from './linalg';

/** 3x3 skew-symmetric matrix [omg] such that [omg] v = omg x v. */
export function VecToso3(omg: Vec): Mat {
  return [
    [0, -omg[2], omg[1]],
    [omg[2], 0, -omg[0]],
    [-omg[1], omg[0], 0],
  ];
}

/** Inverse of `VecToso3`: extracts the 3-vector from a skew-symmetric matrix. */
export function so3ToVec(so3mat: Mat): Vec {
  return [so3mat[2][1], so3mat[0][2], so3mat[1][0]];
}

/**
 * Splits exponential coordinates `expc3 = omghat * theta` into the unit axis
 * and the angle. For the zero vector the axis is returned as the zero vector
 * and theta = 0 (the book's version would divide by zero).
 */
export function AxisAng3(expc3: Vec): [Vec, number] {
  const theta = norm(expc3);
  return [theta === 0 ? [0, 0, 0] : vecScale(expc3, 1 / theta), theta];
}

/**
 * Coefficients of the closed-form matrix exponential written in terms of the
 * UNnormalised matrix W = [omg * theta]:
 *   exp(W)   = I + A W + B W^2
 *   G(theta) = I + B W + C W^2      (translation part of exp on se(3))
 * with A = sin(t)/t, B = (1 - cos t)/t^2, C = (t - sin t)/t^3. Series
 * expansions are used for small theta so the formulas are exact at theta = 0
 * and free of cancellation error nearby.
 */
export function rodriguesCoefficients(theta: number): { A: number; B: number; C: number } {
  const t2 = theta * theta;
  if (theta < 1e-3) {
    return {
      A: 1 - t2 / 6 + (t2 * t2) / 120,
      B: 0.5 - t2 / 24 + (t2 * t2) / 720,
      C: 1 / 6 - t2 / 120 + (t2 * t2) / 5040,
    };
  }
  const half = Math.sin(theta / 2);
  return {
    A: Math.sin(theta) / theta,
    B: (2 * half * half) / t2, // = (1 - cos theta) / theta^2 without cancellation
    C: (theta - Math.sin(theta)) / (t2 * theta),
  };
}

/** Matrix exponential of an so(3) element (Rodrigues' formula). */
export function MatrixExp3(so3mat: Mat): Mat {
  const theta = norm(so3ToVec(so3mat));
  const { A, B } = rodriguesCoefficients(theta);
  const W2 = matMul(so3mat, so3mat);
  return matAdd(matAdd(eye(3), matScale(so3mat, A)), matScale(W2, B));
}

/**
 * Matrix logarithm of a rotation matrix: returns [omghat] * theta with
 * theta in [0, pi].
 *
 * Numerically robust version of the book's algorithm:
 *  - theta is obtained from atan2(|R - R^T| / 2, (tr R - 1) / 2), which is
 *    accurate for every angle (acos alone loses digits near 0 and pi);
 *  - away from pi the axis comes from the antisymmetric part (R - R^T);
 *  - near theta = pi the antisymmetric part vanishes, so the axis is taken
 *    from the symmetric part (R + R^T)/2 = I + (1 - cos theta) [w]^2, i.e.
 *    the largest diagonal entry of w w^T, with its sign fixed by R - R^T
 *    whenever that is still informative. At exactly pi both +w and -w are
 *    valid logarithms.
 */
export function MatrixLog3(R: Mat): Mat {
  const anti = [
    0.5 * (R[2][1] - R[1][2]),
    0.5 * (R[0][2] - R[2][0]),
    0.5 * (R[1][0] - R[0][1]),
  ]; // = sin(theta) * omghat
  const s = norm(anti);
  const c = clamp(0.5 * ((R[0][0] + R[1][1] + R[2][2]) - 1), -1, 1);
  const theta = Math.atan2(s, c);
  if (theta < 1e-12) return [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
  if (theta > Math.PI - 1e-4) {
    // omghat omghat^T = I + ((R + R^T)/2 - I) / (1 - cos theta)
    const k = 1 / (1 - c);
    const P: Mat = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
    for (let i = 0; i < 3; i++) {
      for (let j = 0; j < 3; j++) {
        const sym = 0.5 * (R[i][j] + R[j][i]) - (i === j ? 1 : 0);
        P[i][j] = (i === j ? 1 : 0) + sym * k;
      }
    }
    let best = 0;
    for (let i = 1; i < 3; i++) if (P[i][i] > P[best][best]) best = i;
    let w = normalize([P[0][best], P[1][best], P[2][best]]);
    if (dot(w, anti) < 0) w = vecScale(w, -1);
    return VecToso3(vecScale(w, theta));
  }
  return VecToso3(vecScale(anti, theta / s));
}

/** Inverse of a rotation matrix, i.e. its transpose. */
export function RotInv(R: Mat): Mat {
  return transpose(R);
}

/**
 * Projects an arbitrary 3x3 matrix onto SO(3): the closest rotation in the
 * Frobenius norm, obtained from the SVD as U V^T (with the last column of U
 * flipped when that product would be a reflection).
 */
export function ProjectToSO3(mat: Mat): Mat {
  const { U, V } = svd(mat);
  let R = matMul(U, transpose(V));
  if (det(R) < 0) {
    const U2 = U.map((row) => [row[0], row[1], -row[2]]);
    R = matMul(U2, transpose(V));
  }
  return R;
}

/**
 * Distance of a 3x3 matrix from SO(3): ||R^T R - I||_F when det R > 0, and a
 * large number (1e9) otherwise.
 */
export function DistanceToSO3(mat: Mat): number {
  if (det(mat) > 0) return normFro(matSub(matMul(transpose(mat), mat), eye(3)));
  return 1e9;
}

/** True when the matrix is within 1e-3 (Frobenius) of being a rotation matrix. */
export function TestIfSO3(mat: Mat): boolean {
  return DistanceToSO3(mat) < 1e-3;
}

/** Rotation about the x-axis by theta (radians). */
export function RotX(theta: number): Mat {
  const c = Math.cos(theta);
  const s = Math.sin(theta);
  return [
    [1, 0, 0],
    [0, c, -s],
    [0, s, c],
  ];
}

/** Rotation about the y-axis by theta (radians). */
export function RotY(theta: number): Mat {
  const c = Math.cos(theta);
  const s = Math.sin(theta);
  return [
    [c, 0, s],
    [0, 1, 0],
    [-s, 0, c],
  ];
}

/** Rotation about the z-axis by theta (radians). */
export function RotZ(theta: number): Mat {
  const c = Math.cos(theta);
  const s = Math.sin(theta);
  return [
    [c, -s, 0],
    [s, c, 0],
    [0, 0, 1],
  ];
}

/**
 * ZYX Euler angles (yaw-pitch-roll) to a rotation matrix:
 * R = RotZ(alpha) RotY(beta) RotX(gamma), i.e. successive rotations about the
 * BODY axes z, then y, then x.
 */
export function rotZYX(euler: Vec): Mat {
  const [alpha, beta, gamma] = euler;
  return matMul(matMul(RotZ(alpha), RotY(beta)), RotX(gamma));
}

/**
 * Inverse of `rotZYX`. Returns [alpha, beta, gamma] with beta in [-pi/2, pi/2].
 * At the gimbal-lock singularity (|cos beta| ~ 0) only alpha +/- gamma is
 * determined; the convention here is gamma = 0. The second field of the
 * result tells the caller whether that happened.
 */
export function eulerZYX(R: Mat): { angles: Vec; gimbalLock: boolean } {
  const cosBeta = Math.hypot(R[0][0], R[1][0]);
  const beta = Math.atan2(-R[2][0], cosBeta);
  if (cosBeta < 1e-10) {
    // sin(beta) = +/-1: R[0][1] = -sin(alpha -/+ gamma), R[1][1] = cos(alpha -/+ gamma)
    const alpha = Math.atan2(-R[0][1], R[1][1]);
    return { angles: [alpha, beta, 0], gimbalLock: true };
  }
  const alpha = Math.atan2(R[1][0], R[0][0]);
  const gamma = Math.atan2(R[2][1], R[2][2]);
  return { angles: [alpha, beta, gamma], gimbalLock: false };
}

/**
 * Unit quaternion [w, x, y, z] (scalar first) of a rotation matrix, using the
 * numerically stable "largest component first" branch selection. The returned
 * quaternion has w >= 0.
 */
export function RotToQuat(R: Mat): Vec {
  const tr = R[0][0] + R[1][1] + R[2][2];
  let q: Vec;
  if (tr > 0) {
    const s = Math.sqrt(tr + 1) * 2;
    q = [0.25 * s, (R[2][1] - R[1][2]) / s, (R[0][2] - R[2][0]) / s, (R[1][0] - R[0][1]) / s];
  } else if (R[0][0] > R[1][1] && R[0][0] > R[2][2]) {
    const s = Math.sqrt(1 + R[0][0] - R[1][1] - R[2][2]) * 2;
    q = [(R[2][1] - R[1][2]) / s, 0.25 * s, (R[0][1] + R[1][0]) / s, (R[0][2] + R[2][0]) / s];
  } else if (R[1][1] > R[2][2]) {
    const s = Math.sqrt(1 + R[1][1] - R[0][0] - R[2][2]) * 2;
    q = [(R[0][2] - R[2][0]) / s, (R[0][1] + R[1][0]) / s, 0.25 * s, (R[1][2] + R[2][1]) / s];
  } else {
    const s = Math.sqrt(1 + R[2][2] - R[0][0] - R[1][1]) * 2;
    q = [(R[1][0] - R[0][1]) / s, (R[0][2] + R[2][0]) / s, (R[1][2] + R[2][1]) / s, 0.25 * s];
  }
  if (q[0] < 0) q = vecScale(q, -1);
  return normalize(q);
}

/** Rotation matrix of a quaternion [w, x, y, z] (normalised internally). */
export function QuatToRot(q: Vec): Mat {
  const [w, x, y, z] = normalize(q);
  return [
    [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
    [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
    [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
  ];
}

/** Rotation angle (radians, in [0, pi]) between two rotation matrices. */
export function rotationAngleBetween(R1: Mat, R2: Mat): number {
  const Rrel = matMul(transpose(R1), R2);
  return norm(so3ToVec(MatrixLog3(Rrel)));
}

/** Helper re-exported for callers that want the book's tolerance semantics. */
export const so3Tolerance = { nearZero };
