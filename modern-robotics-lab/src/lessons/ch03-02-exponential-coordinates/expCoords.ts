/**
 * Pure helpers for the exponential-coordinates lesson (ch03-02): Rodrigues' formula split into its
 * three terms, the branch structure of the matrix logarithm, angular velocity from Ṙ, two example
 * motions, and the "single axis" vs. "ZYX Euler" paths between the same two orientations.
 * The core exp/log math is the library's (`MatrixExp3`, `MatrixLog3`, `eulerZYX`); this file only
 * arranges it for the visualisation.
 */
import {
  type Mat, type Vec, MatrixExp3, MatrixLog3, RotX, RotY, RotZ, VecToso3, so3ToVec, eulerZYX,
  clamp, cross, dot, eye, matAdd, matMul, matScale, matSub, matVec, norm, transpose, vecAdd, vecScale, wrapTo2Pi,
} from '@/mr';

// ---------------------------------------------------------------------------
// Axis parametrisation
// ---------------------------------------------------------------------------

/** Unit vector from azimuth az (about ẑ, from x̂) and elevation el (from the xy-plane), radians. */
export function axisFromAngles(az: number, el: number): Vec {
  return [Math.cos(el) * Math.cos(az), Math.cos(el) * Math.sin(az), Math.sin(el)];
}

/** Inverse of `axisFromAngles` (az in (−π, π], el in [−π/2, π/2]). */
export function anglesFromAxis(w: Vec): { az: number; el: number } {
  const n = norm(w) || 1;
  return { az: Math.atan2(w[1], w[0]), el: Math.asin(clamp(w[2] / n, -1, 1)) };
}

/** exp([ω̂]θ) through the library. */
export function rotAxisAngle(omghat: Vec, theta: number): Mat {
  return MatrixExp3(VecToso3(vecScale(omghat, theta)));
}

// ---------------------------------------------------------------------------
// Rodrigues' formula, term by term
// ---------------------------------------------------------------------------

export interface RodriguesTerms {
  /** sin θ [ω̂] */
  sinTerm: Mat;
  /** (1 − cos θ) [ω̂]² */
  cosTerm: Mat;
  /** I + (enabled terms); equals exp([ω̂]θ) when both terms are on. */
  R: Mat;
}

export function rodriguesTerms(omghat: Vec, theta: number, use: { sin: boolean; cos: boolean } = { sin: true, cos: true }): RodriguesTerms {
  const W = VecToso3(omghat);
  const sinTerm = matScale(W, Math.sin(theta));
  const cosTerm = matScale(matMul(W, W), 1 - Math.cos(theta));
  let R = eye(3);
  if (use.sin) R = matAdd(R, sinTerm);
  if (use.cos) R = matAdd(R, cosTerm);
  return { sinTerm, cosTerm, R };
}

export interface PointRotation {
  /** start point p */
  p: Vec;
  /** sin θ (ω̂ × p): pushes along the tangent of the circle */
  sinPart: Vec;
  /** (1 − cos θ) ω̂ × (ω̂ × p): pulls back towards the axis */
  cosPart: Vec;
  /** p + sinPart */
  mid: Vec;
  /** p + (enabled parts) = exp([ω̂]θ) p when both are enabled */
  end: Vec;
  /** foot of p on the axis, (ω̂·p) ω̂: centre of the circle the point travels on */
  center: Vec;
  radius: number;
}

/** Rodrigues' formula applied to one point, with each term as a vector (for head-to-tail drawing). */
export function rotatePointTerms(omghat: Vec, theta: number, p: Vec, use: { sin: boolean; cos: boolean } = { sin: true, cos: true }): PointRotation {
  const wxp = cross(omghat, p);
  const sinPart = vecScale(wxp, Math.sin(theta));
  const cosPart = vecScale(cross(omghat, wxp), 1 - Math.cos(theta));
  const mid = vecAdd(p, use.sin ? sinPart : [0, 0, 0]);
  const end = vecAdd(mid, use.cos ? cosPart : [0, 0, 0]);
  const center = vecScale(omghat, dot(omghat, p));
  return { p, sinPart, cosPart, mid, end, center, radius: norm(vecSub3(p, center)) };
}

function vecSub3(a: Vec, b: Vec): Vec { return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]; }

/** Points exp([ω̂]t) p for t from t0 to t1 (the arc a body point sweeps). */
export function orbitPoints(omghat: Vec, p: Vec, t0: number, t1: number, n = 48): Vec[] {
  const pts: Vec[] = [];
  for (let i = 0; i <= n; i++) pts.push(matVec(rotAxisAngle(omghat, t0 + ((t1 - t0) * i) / n), p));
  return pts;
}

// ---------------------------------------------------------------------------
// Matrix logarithm: value and branch
// ---------------------------------------------------------------------------

/**
 * Which formula the logarithm uses (mirrors the thresholds of `MatrixLog3`):
 *  - 'identity': θ = 0, R = I — the axis is undefined, log R = 0;
 *  - 'general' : axis from the antisymmetric part (R − Rᵀ) / (2 sin θ);
 *  - 'nearPi'  : θ within 1e-4 of π — axis from the symmetric part, sign from R − Rᵀ;
 *  - 'pi'      : θ = π to machine precision — both ±ω̂ are valid, the sign is a convention.
 */
export type LogBranch = 'identity' | 'general' | 'nearPi' | 'pi';

export interface LogInfo {
  /** angle in [0, π] */
  theta: number;
  /** unit axis, or null when θ = 0 */
  omghat: Vec | null;
  /** exponential coordinates ω̂θ */
  expc: Vec;
  branch: LogBranch;
  /** |R − Rᵀ| / 2 = sin θ, the quantity the general formula divides by */
  sinTheta: number;
  /** cos θ = (tr R − 1) / 2 */
  cosTheta: number;
}

export function logInfo(R: Mat): LogInfo {
  const anti = [0.5 * (R[2][1] - R[1][2]), 0.5 * (R[0][2] - R[2][0]), 0.5 * (R[1][0] - R[0][1])];
  const s = norm(anti);
  const c = clamp(0.5 * (R[0][0] + R[1][1] + R[2][2] - 1), -1, 1);
  const expc = so3ToVec(MatrixLog3(R));
  const theta = norm(expc);
  let branch: LogBranch;
  const th = Math.atan2(s, c);
  if (th < 1e-12) branch = 'identity';
  else if (th > Math.PI - 1e-4) branch = s > 1e-14 ? 'nearPi' : 'pi';
  else branch = 'general';
  return { theta, omghat: theta > 0 ? vecScale(expc, 1 / theta) : null, expc, branch, sinTheta: s, cosTheta: c };
}

/**
 * What the logarithm returns for a rotation by θ_in (any real) about a fixed unit axis ω̂:
 * the angle folded into [0, π], and whether the returned axis is −ω̂ (θ_in mod 2π > π).
 */
export function foldedAngle(thetaIn: number): { theta: number; flipped: boolean } {
  const t = wrapTo2Pi(thetaIn);
  return t <= Math.PI ? { theta: t, flipped: false } : { theta: 2 * Math.PI - t, flipped: true };
}

// ---------------------------------------------------------------------------
// Angular velocity
// ---------------------------------------------------------------------------

export interface MotionSample { R: Mat; wS: Vec; wB: Vec }

/** Rotation about an axis fixed in {s}: R(t) = exp([ω̂] rate t) R0. Then ω_s = ω̂ rate and ω_b = R0ᵀ ω̂ rate. */
export function fixedAxisMotion(omghat: Vec, rate: number, R0: Mat, t: number): MotionSample {
  const R = matMul(rotAxisAngle(omghat, rate * t), R0);
  const wS = vecScale(omghat, rate);
  return { R, wS, wB: matVec(transpose(R), wS) };
}

export interface TopParams { precession: number; spin: number; tilt: number }

/**
 * A spinning top: R(t) = Rot(ẑ, Ω_p t) Rot(x̂, tilt) Rot(ẑ, Ω_s t) (precession about ẑ_s, spin about ẑ_b).
 * Analytically ω_s = Ω_p ẑ + Ω_s R ẑ and ω_b = Ω_p Rᵀ ẑ + Ω_s ẑ: both change with time.
 */
export function topMotion(t: number, p: TopParams): MotionSample {
  const R = matMul(matMul(RotZ(p.precession * t), RotX(p.tilt)), RotZ(p.spin * t));
  const z: Vec = [0, 0, 1];
  const wS = vecAdd(vecScale(z, p.precession), vecScale(matVec(R, z), p.spin));
  const wB = vecAdd(vecScale(matVec(transpose(R), z), p.precession), vecScale(z, p.spin));
  return { R, wS, wB };
}

export interface OmegaFromRdot { Rdot: Mat; WS: Mat; WB: Mat; wS: Vec; wB: Vec }

/**
 * Angular velocity read off a rotation-valued function by a central difference:
 * Ṙ ≈ (R(t+h) − R(t−h)) / 2h, [ω_s] = Ṙ Rᵀ, [ω_b] = Rᵀ Ṙ. The two products come out skew-symmetric.
 */
export function omegaFromDerivative(Rfun: (t: number) => Mat, t: number, h = 1e-5): OmegaFromRdot {
  const R = Rfun(t);
  const Rdot = matScale(matSub(Rfun(t + h), Rfun(t - h)), 1 / (2 * h));
  const WS = matMul(Rdot, transpose(R));
  const WB = matMul(transpose(R), Rdot);
  const vee = (W: Mat): Vec => [0.5 * (W[2][1] - W[1][2]), 0.5 * (W[0][2] - W[2][0]), 0.5 * (W[1][0] - W[0][1])];
  return { Rdot, WS, WB, wS: vee(WS), wB: vee(WB) };
}

// ---------------------------------------------------------------------------
// Two paths to the same orientation
// ---------------------------------------------------------------------------

/** Single-axis path from I to R: exp(u · log R), u ∈ [0, 1]. */
export function expPath(R: Mat, u: number): Mat {
  return MatrixExp3(matScale(MatrixLog3(R), clamp(u, 0, 1)));
}

/**
 * ZYX Euler path from I to R in three equal parts: rotate about ẑ_b by α, then ŷ_b by β, then x̂_b by γ
 * (R = Rot(ẑ, α) Rot(ŷ, β) Rot(x̂, γ)).
 */
export function eulerPath(R: Mat, u: number): Mat {
  const [a, b, g] = eulerZYX(R).angles;
  const v = clamp(u, 0, 1) * 3;
  const f = (k: number) => clamp(v - k, 0, 1);
  return matMul(matMul(RotZ(a * f(0)), RotY(b * f(1))), RotX(g * f(2)));
}

/**
 * ∫ω_s dt along each path. For the single-axis path it equals the exponential coordinates ω̂θ; for the
 * Euler path it is α ẑ + β Rot(ẑ,α) ŷ + γ Rot(ẑ,α)Rot(ŷ,β) x̂, which in general is NOT log R.
 */
export function integratedOmega(R: Mat, path: 'exp' | 'euler', u = 1): Vec {
  if (path === 'exp') return vecScale(so3ToVec(MatrixLog3(R)), clamp(u, 0, 1));
  const [a, b, g] = eulerZYX(R).angles;
  const v = clamp(u, 0, 1) * 3;
  const f = (k: number) => clamp(v - k, 0, 1);
  const z: Vec = [0, 0, 1];
  const y = matVec(RotZ(a), [0, 1, 0]);
  const x = matVec(matMul(RotZ(a), RotY(b)), [1, 0, 0]);
  return vecAdd(vecAdd(vecScale(z, a * f(0)), vecScale(y, b * f(1))), vecScale(x, g * f(2)));
}

/** Screw-motion teaser: rotate p about ω̂ by θ and slide pitch·θ along ω̂. */
export function helixPoint(omghat: Vec, pitch: number, theta: number, p: Vec): Vec {
  return vecAdd(matVec(rotAxisAngle(omghat, theta), p), vecScale(omghat, pitch * theta));
}
