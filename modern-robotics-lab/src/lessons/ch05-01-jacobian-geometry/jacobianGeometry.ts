/**
 * Pure helpers for the "geometry of the space / body Jacobian" lesson.
 *
 * Everything robotics-specific (FK, J_s, J_b, Adjoint, screw-axis geometry) is delegated to
 * `@/mr`; this module only packages the results the way the simulation needs them:
 *  - a snapshot of T_sb, J_s, J_b and [Ad_{T_sb}] at one configuration (plus the residual of
 *    J_s = [Ad_{T_sb}] J_b),
 *  - the twist / point-velocity breakdown for a joint-rate vector θ̇ (per-joint contributions,
 *    v_s versus the tip-point velocity v_s + ω × p),
 *  - planar extraction in the twist order [ω_z, v_x, v_y] (same order as `planarJacobian`),
 *  - the instantaneous centre of a planar twist and a column-change detector.
 */
import {
  type Mat,
  type Vec,
  Adjoint,
  FKinSpace,
  JacobianBody,
  JacobianSpace,
  ScrewAxisToPointDir,
  TransToRp,
  cross,
  det,
  getCol,
  matMul,
  matSub,
  matVec,
  normFro,
  vecAdd,
  vecScale,
} from '@/mr';

/** The PoE description of an open chain (a subset of `RobotModel`). */
export interface ChainModel {
  M: Mat;
  Slist: Mat;
  Blist: Mat;
}

export interface JacobianSnapshot {
  theta: Vec;
  /** End-effector configuration T_sb(θ). */
  T: Mat;
  /** Rotation part of T_sb. */
  R: Mat;
  /** Position of the {b} origin (the tip) in {s}. */
  p: Vec;
  /** Space Jacobian (6 × n), columns = current joint screw axes in {s}. */
  Js: Mat;
  /** Body Jacobian (6 × n), columns = current joint screw axes in {b}. */
  Jb: Mat;
  /** [Ad_{T_sb}] (6 × 6). */
  Ad: Mat;
  /** ‖J_s − [Ad_{T_sb}] J_b‖_F, zero up to round-off. */
  residual: number;
}

export function jacobianSnapshot(model: ChainModel, theta: Vec): JacobianSnapshot {
  const T = FKinSpace(model.M, model.Slist, theta);
  const [R, p] = TransToRp(T);
  const Js = JacobianSpace(model.Slist, theta);
  const Jb = JacobianBody(model.Blist, theta);
  const Ad = Adjoint(T);
  const residual = normFro(matSub(Js, matMul(Ad, Jb)));
  return { theta: theta.slice(), T, R, p, Js, Jb, Ad, residual };
}

/**
 * Velocity of the point of the moving body that is momentarily at `p`, for a twist
 * V = [ω; v] expressed in the same frame as `p`: v + ω × p.
 */
export function pointVelocity(V: Vec, p: Vec): Vec {
  return vecAdd([V[3], V[4], V[5]], cross([V[0], V[1], V[2]], p));
}

export interface JointContribution {
  /** θ̇_i · (column i of J_s). */
  twistS: Vec;
  /** θ̇_i · (column i of J_b). */
  twistB: Vec;
  /** Velocity of the tip point in {s} produced by joint i alone. */
  tipVel: Vec;
}

export interface VelocityBreakdown {
  /** Space twist V_s = J_s θ̇. */
  Vs: Vec;
  /** Body twist V_b = J_b θ̇. */
  Vb: Vec;
  /** ω_s (first three entries of V_s). */
  omegaS: Vec;
  /** v_s: velocity of the body point that is momentarily at the {s} origin. */
  vS: Vec;
  /** ω_s × p_tip. */
  omegaCrossP: Vec;
  /** Tip-point velocity in {s}: v_s + ω_s × p_tip. */
  tipVel: Vec;
  /** Linear part of V_b, i.e. the tip velocity written in {b} coordinates. */
  vB: Vec;
  contributions: JointContribution[];
}

export function velocityBreakdown(snap: JacobianSnapshot, thetadot: Vec): VelocityBreakdown {
  const Vs = matVec(snap.Js, thetadot);
  const Vb = matVec(snap.Jb, thetadot);
  const omegaS = Vs.slice(0, 3);
  const vS = Vs.slice(3, 6);
  const omegaCrossP = cross(omegaS, snap.p);
  const contributions: JointContribution[] = thetadot.map((rate, i) => {
    const twistS = vecScale(getCol(snap.Js, i), rate);
    return { twistS, twistB: vecScale(getCol(snap.Jb, i), rate), tipVel: pointVelocity(twistS, snap.p) };
  });
  return { Vs, Vb, omegaS, vS, omegaCrossP, tipVel: vecAdd(vS, omegaCrossP), vB: Vb.slice(3, 6), contributions };
}

/**
 * Point on the axis of a (revolute) Jacobian column closest to the origin of the frame the
 * column is expressed in. Returns null for a prismatic column (ω = 0).
 */
export function axisPoint(column: Vec): Vec | null {
  const w2 = column[0] * column[0] + column[1] * column[1] + column[2] * column[2];
  if (w2 < 1e-12) return null;
  return ScrewAxisToPointDir(column).q;
}

/** Twist rows used for planar chains: [ω_z, v_x, v_y] (indices into a 6-vector). */
export const PLANAR_IDX: readonly number[] = [2, 3, 4];

/** Rows [ω_z, v_x, v_y] of a 6 × n Jacobian (the other rows vanish for a chain in the x–y plane). */
export function planarRows(J: Mat): Mat {
  return PLANAR_IDX.map((r) => J[r].slice());
}

/** [ω_z, v_x, v_y] of a 6-vector twist. */
export function planarTwist(V: Vec): Vec {
  return PLANAR_IDX.map((r) => V[r]);
}

/** The 3 × 3 block of a 6 × 6 adjoint acting on planar twists [ω_z, v_x, v_y]. */
export function planarAdjoint(Ad: Mat): Mat {
  return PLANAR_IDX.map((r) => PLANAR_IDX.map((c) => Ad[r][c]));
}

/** For each column, whether any entry differs by more than `tol` between A and B. */
export function changedColumns(A: Mat, B: Mat, tol = 1e-9): boolean[] {
  const n = A[0]?.length ?? 0;
  const out: boolean[] = [];
  for (let j = 0; j < n; j++) {
    let changed = false;
    for (let i = 0; i < A.length && !changed; i++) changed = Math.abs(A[i][j] - B[i][j]) > tol;
    out.push(changed);
  }
  return out;
}

/**
 * Instantaneous centre of rotation of a planar twist [ω_z, v_x, v_y]: the point c with
 * v + ω ẑ × c = 0, i.e. c = (−v_y / ω_z, v_x / ω_z). Null when |ω_z| < eps (pure translation).
 */
export function instantCenter(V: Vec, eps = 1e-9): [number, number] | null {
  const [w, vx, vy] = V;
  if (Math.abs(w) < eps) return null;
  return [-vy / w, vx / w];
}

/**
 * Tip-point velocity Jacobian (3 × n): column i is the velocity of the tip point in {s} for
 * θ̇ = e_i, i.e. v_i + ω_i × p_tip with [ω_i; v_i] = column i of J_s.
 */
export function tipPointJacobian(snap: JacobianSnapshot): Mat {
  const n = snap.Js[0].length;
  const cols: Vec[] = [];
  for (let i = 0; i < n; i++) cols.push(pointVelocity(getCol(snap.Js, i), snap.p));
  return [0, 1, 2].map((r) => cols.map((c) => c[r]));
}

/**
 * A scalar singularity indicator for 3-joint arms: det of the planar J_s (3 × 3 in
 * [ω_z, v_x, v_y]) for planar chains, or det of the 3 × 3 tip-point Jacobian otherwise.
 */
export function singularityMeasure(snap: JacobianSnapshot, planar: boolean): number {
  return planar ? det(planarRows(snap.Js)) : det(tipPointJacobian(snap));
}
