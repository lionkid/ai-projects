/**
 * Pure helpers for the rotation-matrix lesson (ch03-01): planar frames, composition of
 * elementary rotations about fixed ({s}) or body ({b}) axes, SO(3) membership diagnostics,
 * and a few geometric constructions used by the 3D view. All rotation math comes from `@/mr`.
 */
import {
  type Mat, type Vec, RotX, RotY, RotZ, MatrixExp3, MatrixLog3, QuatToRot, VecToso3, so3ToVec,
  det, dot, eye, getCol, matMul, matSub, matVec, norm, normFro, se2, transInv2, transformPoint2, transpose, vecScale,
} from '@/mr';

export type AxisName = 'x' | 'y' | 'z';
/** 'body' = post-multiply (rotate about the current {b} axis); 'fixed' = pre-multiply (rotate about the {s} axis). */
export type ComposeMode = 'body' | 'fixed';
export interface ElemRot { axis: AxisName; angle: number }

export const AXIS_VEC: Record<AxisName, Vec> = { x: [1, 0, 0], y: [0, 1, 0], z: [0, 0, 1] };

/** Elementary rotation about a coordinate axis (radians). */
export function rotAbout(axis: AxisName, angle: number): Mat {
  if (axis === 'x') return RotX(angle);
  if (axis === 'y') return RotY(angle);
  return RotZ(angle);
}

/**
 * Apply elementary rotations one after another to a start orientation R0.
 *  - 'body'  (post-multiply): rotation k turns about an axis of the CURRENT body frame, R = R0 R1 R2 ...
 *  - 'fixed' (pre-multiply):  rotation k turns about an axis of the fixed frame {s},  R = ... R2 R1 R0
 * `progress` in [0, n] animates the sequence: rotation k is applied with the fraction clamp(progress - k, 0, 1).
 */
export function composeSequence(R0: Mat, rots: ElemRot[], mode: ComposeMode, progress = rots.length): Mat {
  let R = R0;
  rots.forEach((r, k) => {
    const f = Math.min(1, Math.max(0, progress - k));
    if (f === 0) return;
    const E = rotAbout(r.axis, r.angle * f);
    R = mode === 'body' ? matMul(R, E) : matMul(E, R);
  });
  return R;
}

/**
 * Unit axis, written in {s}, about which the frame turns while rotation k of a sequence is applied.
 * Fixed axes: the {s} axis itself. Body axes: that axis of the body frame as it stands when rotation k
 * starts (it stays put during the rotation, since a frame never moves its own rotation axis).
 */
export function sequenceAxisInSpace(R0: Mat, rots: ElemRot[], mode: ComposeMode, k: number): Vec {
  const a = AXIS_VEC[rots[k].axis];
  if (mode === 'fixed') return a.slice();
  return matVec(composeSequence(R0, rots, 'body', k), a);
}

/** Columns of a 3×3 matrix (the {b} axes written in {s} when the matrix is R_sb). */
export function columns(M: Mat): [Vec, Vec, Vec] {
  return [getCol(M, 0), getCol(M, 1), getCol(M, 2)];
}

export interface SO3Report {
  det: number;
  RtR: Mat;
  /** ||RᵀR − I||_F */
  orthoError: number;
  colNorms: [number, number, number];
  /** x·y, y·z, z·x of the three columns */
  colDots: [number, number, number];
  isRotation: boolean;
  /** Orthogonal but det = −1: a mirror image, not a rotation. */
  isReflection: boolean;
}

/** SO(3) membership diagnostics for an arbitrary 3×3 matrix. */
export function so3Report(M: Mat, tol = 1e-3): SO3Report {
  const RtR = matMul(transpose(M), M);
  const orthoError = normFro(matSub(RtR, eye(3)));
  const [cx, cy, cz] = columns(M);
  const d = det(M);
  const orthogonal = orthoError < tol;
  return {
    det: d,
    RtR,
    orthoError,
    colNorms: [norm(cx), norm(cy), norm(cz)],
    colDots: [dot(cx, cy), dot(cy, cz), dot(cz, cx)],
    isRotation: orthogonal && d > 0,
    isReflection: orthogonal && d < 0,
  };
}

/** The 12 edges of the image of the unit cube [0,1]³ under M (its signed volume is det M). */
export function parallelepipedEdges(M: Mat): [Vec, Vec][] {
  const [a, b, c] = columns(M);
  const corner = (i: number, j: number, k: number): Vec => [0, 1, 2].map((r) => i * a[r] + j * b[r] + k * c[r]);
  const edges: [Vec, Vec][] = [];
  for (const i of [0, 1]) for (const j of [0, 1]) {
    edges.push([corner(0, i, j), corner(1, i, j)]);
    edges.push([corner(i, 0, j), corner(i, 1, j)]);
    edges.push([corner(i, j, 0), corner(i, j, 1)]);
  }
  return edges;
}

/** Dashed "drop lines" that read off the {s} coordinates of a point t: down to the xy-plane, then to the x and y axes. */
export function dropLines(t: Vec): [Vec, Vec][] {
  const foot: Vec = [t[0], t[1], 0];
  return [[t, foot], [foot, [t[0], 0, 0]], [foot, [0, t[1], 0]], [[0, 0, 0], [0, 0, t[2]]]];
}

/** Points along the single-axis rotation that carries v to R v (axis and angle from log R). */
export function rotationArc(R: Mat, v: Vec, n = 32): Vec[] {
  const W = MatrixLog3(R);
  const pts: Vec[] = [];
  for (let i = 0; i <= n; i++) pts.push(matVec(MatrixExp3(matScaleSkew(W, i / n)), v));
  return pts;
}

function matScaleSkew(W: Mat, s: number): Mat {
  return VecToso3(vecScale(so3ToVec(W), s));
}

/** Equivalent single rotation of R: unit axis (null when R = I) and angle in [0, π]. */
export function equivalentAxisAngle(R: Mat): { axis: Vec | null; angle: number } {
  const w = so3ToVec(MatrixLog3(R));
  const angle = norm(w);
  return { axis: angle > 1e-9 ? vecScale(w, 1 / angle) : null, angle };
}

/** Uniformly distributed random rotation (normalised Gaussian quaternion). */
export function randomRotation(rand: () => number = Math.random): Mat {
  const gauss = () => {
    const u = Math.max(1e-12, rand());
    const v = rand();
    return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
  };
  return QuatToRot([gauss(), gauss(), gauss(), gauss()]);
}

// ---------------------------------------------------------------------------
// Planar frames (steps 1–2)
// ---------------------------------------------------------------------------

export interface PlanarPose { x: number; y: number; theta: number }

/** 2×2 rotation part of a planar pose. */
export function planarR(theta: number): Mat {
  const c = Math.cos(theta), s = Math.sin(theta);
  return [[c, -s], [s, c]];
}

/** p_s = R_sb p_b + p_sb */
export function planarToSpace(pose: PlanarPose, pb: Vec): Vec {
  return transformPoint2(se2(pose.theta, pose.x, pose.y), pb);
}

/** p_b = R_sbᵀ (p_s − p_sb) */
export function planarToBody(pose: PlanarPose, ps: Vec): Vec {
  return transformPoint2(transInv2(se2(pose.theta, pose.x, pose.y)), ps);
}

/** Pose of {s} as seen from {b}: R_bs = R_sbᵀ, p_bs = −R_sbᵀ p_sb. */
export function invertPlanarPose(pose: PlanarPose): PlanarPose {
  const T = transInv2(se2(pose.theta, pose.x, pose.y));
  return { x: T[0][2], y: T[1][2], theta: Math.atan2(T[1][0], T[0][0]) };
}
