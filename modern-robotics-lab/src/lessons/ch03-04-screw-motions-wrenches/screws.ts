/**
 * Pure helpers for the "screw motions and wrenches" lesson (no DOM, no three.js, unit-tested).
 * All rigid-body math goes through @/mr (ScrewToAxis, MatrixExp6/Log6, Adjoint, ...); this module
 * only composes it for the demos.
 */
import {
  type Mat, type Vec, Adjoint, AxisAng6, MatrixExp3, MatrixExp6, MatrixLog3, MatrixLog6, RpToTrans, ScrewToAxis, TransInv,
  TransToRp, VecTose3, VecToso3, cross, det, dot, matAdd, matMul, matScale, matVec, norm, normalize, se3ToVec, transformPoint,
  transpose, vecAdd, vecScale,
} from '@/mr';

/** Unit direction from an azimuth (about z, measured from +x) and an elevation (above the xy-plane). */
export function dirFromAngles(az: number, el: number): Vec {
  return [Math.cos(el) * Math.cos(az), Math.cos(el) * Math.sin(az), Math.sin(el)];
}

/** Screw axis through q with direction s (normalised here) and pitch h: S = (ŝ, −ŝ × q + h ŝ). */
export function screwFromQSH(q: Vec, s: Vec, h: number): Vec {
  return ScrewToAxis(q, normalize(s), h);
}

/** Pure translation along d: S = (0, d̂), so θ is a distance in metres. */
export function prismaticScrew(d: Vec): Vec {
  const u = normalize(d);
  return [0, 0, 0, u[0], u[1], u[2]];
}

/** The two parts of v for a revolute-type screw: −ŝ × q (from rotating about a line through q) and h ŝ (sliding). */
export function vParts(q: Vec, s: Vec, h: number): { rot: Vec; slide: Vec } {
  const u = normalize(s);
  return { rot: vecScale(cross(u, q), -1), slide: vecScale(u, h) };
}

/** T(θ) = exp([S]θ) T0: the screw S is written in the fixed frame {s}, so it pre-multiplies. */
export function screwMotion(S: Vec, theta: number, T0: Mat): Mat {
  return matMul(MatrixExp6(matScale(VecTose3(S), theta)), T0);
}

/**
 * The three terms of the translation part of exp([S]θ) for a unit-ω screw:
 *   G(θ) v = θ v + (1 − cos θ) [ω] v + (θ − sin θ) [ω]² v.
 * For a pure translation (ω = 0) only the first term remains.
 */
export function translationTerms(S: Vec, theta: number): [Vec, Vec, Vec] {
  const w = S.slice(0, 3), v = S.slice(3, 6);
  const t1 = vecScale(v, theta);
  if (norm(w) < 1e-12) return [t1, [0, 0, 0], [0, 0, 0]];
  const W = VecToso3(w);
  const Wv = matVec(W, v);
  return [t1, vecScale(Wv, 1 - Math.cos(theta)), vecScale(matVec(W, Wv), theta - Math.sin(theta))];
}

/** World positions of a body point r_b for θ = 0 … θmax along the screw (a helix around the axis). */
export function screwPath(S: Vec, thetaMax: number, T0: Mat, rb: Vec, n = 120): Vec[] {
  const pts: Vec[] = [];
  for (let k = 0; k <= n; k++) pts.push(transformPoint(screwMotion(S, (thetaMax * k) / n, T0), rb));
  return pts;
}

export interface ScrewInfo {
  /** Unit screw axis (‖ω‖ = 1, or ω = 0 and ‖v‖ = 1); zero vector for the identity. */
  S: Vec;
  /** Rotation angle in [0, π] (rad), or the distance (m) of a pure translation. */
  theta: number;
  kind: 'identity' | 'translation' | 'screw';
  /** Pitch (m/rad); Infinity for a translation. */
  h: number;
  /** Point of the axis closest to the {s} origin (screws only). */
  q: Vec;
}

/** Split exponential coordinates S·θ (a 6-vector) into the unit screw, θ and the axis data. */
export function screwFromExpCoords(expc6: Vec, eps = 1e-10): ScrewInfo {
  const [S, theta] = AxisAng6(expc6);
  if (theta < eps) return { S: [0, 0, 0, 0, 0, 0], theta: 0, kind: 'identity', h: 0, q: [0, 0, 0] };
  const w = S.slice(0, 3), v = S.slice(3, 6);
  if (norm(w) < 1e-9) return { S, theta, kind: 'translation', h: Infinity, q: [0, 0, 0] };
  return { S, theta, kind: 'screw', h: dot(w, v), q: cross(w, v) };
}

/** The screw (in {s}) that carries T1 onto T2: [S]θ = log(T2 T1⁻¹), so that T2 = exp([S]θ) T1. */
export function screwBetween(T1: Mat, T2: Mat): ScrewInfo {
  return screwFromExpCoords(se3ToVec(MatrixLog6(matMul(T2, TransInv(T1)))));
}

export type InterpMethod = 'screw' | 'decoupled' | 'elementwise';

/**
 * Pose at fraction τ ∈ [0, 1] between T1 and T2:
 *  - 'screw'       exp(τ log(T2 T1⁻¹)) T1: one screw motion about a fixed axis;
 *  - 'decoupled'   R1 exp(τ log(R1ᵀR2)) with the origin on the straight line p1 → p2;
 *  - 'elementwise' (1 − τ) T1 + τ T2 entry by entry: the 3×3 block is generally NOT a rotation.
 */
export function interpolatePose(T1: Mat, T2: Mat, tau: number, method: InterpMethod): Mat {
  if (method === 'screw') return matMul(MatrixExp6(matScale(MatrixLog6(matMul(T2, TransInv(T1))), tau)), T1);
  if (method === 'elementwise') return matAdd(matScale(T1, 1 - tau), matScale(T2, tau));
  const [R1, p1] = TransToRp(T1);
  const [R2, p2] = TransToRp(T2);
  const R = matMul(R1, MatrixExp3(matScale(MatrixLog3(matMul(transpose(R1), R2)), tau)));
  return RpToTrans(R, vecAdd(vecScale(p1, 1 - tau), vecScale(p2, tau)));
}

/** Path of a body point r_b while interpolating from T1 to T2 with a given method. */
export function interpolationPath(T1: Mat, T2: Mat, rb: Vec, method: InterpMethod, n = 60): Vec[] {
  const pts: Vec[] = [];
  for (let k = 0; k <= n; k++) pts.push(transformPoint(interpolatePose(T1, T2, k / n, method), rb));
  return pts;
}

/** det of the 3×3 block (1 for a rotation; < 1 when an element-wise interpolation shrinks the body). */
export function rotationBlockDet(T: Mat): number {
  return det(TransToRp(T)[0]);
}

// ---------------------------------------------------------------------------
// Wrenches
// ---------------------------------------------------------------------------

/** Wrench of a force f applied at the point r (both in the same frame): F = (r × f, f). */
export function wrenchAtPoint(r: Vec, f: Vec): Vec {
  const m = cross(r, f);
  return [m[0], m[1], m[2], f[0], f[1], f[2]];
}

/** The same wrench in another frame: F_a = [Ad_{T_ba}]ᵀ F_b (note the order of the subscripts). */
export function wrenchInFrame(Tba: Mat, Fb: Vec): Vec {
  return matVec(transpose(Adjoint(Tba)), Fb);
}

/** Power delivered by a wrench on a body moving with a twist written in the same frame: P = Vᵀ F. */
export function power(V: Vec, F: Vec): number {
  return dot(V, F);
}
