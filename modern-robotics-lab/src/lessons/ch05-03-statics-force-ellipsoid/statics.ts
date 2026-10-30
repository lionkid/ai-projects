/**
 * Pure helpers for the statics lesson (no DOM).
 *
 * The arm is a planar nR chain from `@/mr` (`planar2R` or `planar3R`). A pure force f (no moment)
 * acts at the tip. Torques are computed with the PoE Jacobians of `@/mr` and correctly paired
 * wrenches, τ = J_bᵀ F_b = J_sᵀ F_s, where F is the wrench the tip applies to the environment.
 */
import {
  JacobianBody, JacobianSpace, FKinBody, Adjoint, TransInv, matVec, transpose, planarFK, planarTipJacobian,
  planar2RIK, wrapToPi, planar2R, planar3R, type Mat, type Vec, type RobotModel,
} from '@/mr';

export type Pt2 = [number, number];
export type ModelKey = '2R' | '3R';
export const MODELS: Record<ModelKey, RobotModel> = { '2R': planar2R, '3R': planar3R };
export const lengthsOf = (k: ModelKey): number[] => MODELS[k].linkLengths ?? (k === '2R' ? [1, 1] : [1, 0.8, 0.6]);

/** Wrap to (−π, π] so a folded joint reads +180°. */
export function wrapAngle(a: number): number {
  const w = wrapToPi(a);
  return w <= -Math.PI ? Math.PI : w;
}

export interface StaticsResult {
  /** τ = J_bᵀ F_b (the lesson's answer). */
  tau: number[];
  /** Same torques from the space side, τ = J_sᵀ F_s (equal up to rounding). */
  tauSpace: number[];
  /** Deliberately mismatched frames, J_sᵀ F_b (generally wrong). */
  tauWrong: number[];
  /** Wrench [m; f] in {b} (tip frame) and in {s}; moment first. */
  Fb: number[];
  Fs: number[];
}

/**
 * Joint torques that hold the arm still while the tip applies the force f (given in {s}
 * coordinates, N) to the environment. Uses JacobianBody / JacobianSpace / Adjoint from `@/mr`.
 */
export function staticTorques(model: RobotModel, th: Vec, f: Pt2): StaticsResult {
  const Tsb = FKinBody(model.M, model.Blist, th);
  // body-frame components of the force: f_b = R_sbᵀ f_s; the force acts at the {b} origin → no moment
  const fs3 = [f[0], f[1], 0];
  const fb3 = [0, 1, 2].map((i) => Tsb[0][i] * fs3[0] + Tsb[1][i] * fs3[1] + Tsb[2][i] * fs3[2]);
  const Fb = [0, 0, 0, fb3[0], fb3[1], fb3[2]];
  // F_s = [Ad_{T_bs}]ᵀ F_b  (power is frame independent: V_sᵀ F_s = V_bᵀ F_b)
  const Fs = matVec(transpose(Adjoint(TransInv(Tsb))), Fb);
  const Jb = JacobianBody(model.Blist, th);
  const Js = JacobianSpace(model.Slist, th);
  return {
    tau: matVec(transpose(Jb), Fb),
    tauSpace: matVec(transpose(Js), Fs),
    tauWrong: matVec(transpose(Js), Fb),
    Fb,
    Fs,
  };
}

/** 2 x n linear tip Jacobian (tip velocity in {s} coordinates), from `@/mr`. */
export function tipJacobian(L: number[], th: Vec): Mat {
  return planarTipJacobian(L, th);
}

export interface AxesInfo {
  /** Singular values of J_v (velocity-ellipse semi-axes per unit |θ̇|), descending. */
  s1: number;
  s2: number;
  /** Major / minor axis of the VELOCITY ellipse (the force ellipse swaps their lengths). */
  u1: Pt2;
  u2: Pt2;
  rank: number;
}

/** Closed-form singular values of a 2 x n matrix (det via Cauchy–Binet, robust at singularities). */
export function ellipseAxes(J: Mat): AxesInfo {
  const n = J[0].length;
  let a = 0, b = 0, d = 0, det = 0;
  for (let i = 0; i < n; i++) { a += J[0][i] ** 2; b += J[0][i] * J[1][i]; d += J[1][i] ** 2; }
  for (let i = 0; i < n; i++) for (let j = i + 1; j < n; j++) { const m = J[0][i] * J[1][j] - J[0][j] * J[1][i]; det += m * m; }
  const l1 = (a + d) / 2 + Math.hypot((a - d) / 2, b);
  const l2 = l1 > 0 ? det / l1 : 0;
  const s1 = Math.sqrt(Math.max(0, l1)), s2 = Math.sqrt(Math.max(0, l2));
  const ang = 0.5 * Math.atan2(2 * b, a - d);
  const u1: Pt2 = [Math.cos(ang), Math.sin(ang)];
  return { s1, s2, u1, u2: [-u1[1], u1[0]], rank: s1 < 1e-12 ? 0 : s2 > 1e-6 * s1 ? 2 : 1 };
}

/** Euclidean norm of a vector. */
export const vnorm = (v: ArrayLike<number>): number => Math.sqrt(Array.from(v).reduce((s, x) => s + x * x, 0));

export interface MomentArm {
  /** Signed moment arm d_i (m): τ_i = d_i |f| for the "tip applies f" convention. */
  d: number;
  /** Foot of the perpendicular from joint i onto the line of action of f. */
  foot: Pt2;
  joint: Pt2;
}

/** Moment arms of the tip force about every joint (planar picture of τ_i = ((p − p_i) × f)_z). */
export function momentArms(L: number[], th: Vec, f: Pt2): MomentArm[] {
  const joints = planarFK(L, th).joints;
  const p = joints[joints.length - 1];
  const m = Math.hypot(f[0], f[1]);
  const fh: Pt2 = m > 1e-12 ? [f[0] / m, f[1] / m] : [1, 0];
  return joints.slice(0, L.length).map((q) => {
    const rx = p[0] - q[0], ry = p[1] - q[1];
    const along = -(rx * fh[0] + ry * fh[1]);
    return { d: rx * fh[1] - ry * fh[0], foot: [p[0] + along * fh[0], p[1] + along * fh[1]], joint: [q[0], q[1]] };
  });
}

/** Virtual link (joint 2 → tip) of a 3R with θ3 frozen; for a 2R it is link 2 itself. */
function virtualLink(L: number[], th: Vec): { lp: number; beta: number } {
  if (L.length === 2) return { lp: L[1], beta: 0 };
  const x = L[1] + L[2] * Math.cos(th[2]), y = L[2] * Math.sin(th[2]);
  return { lp: Math.hypot(x, y), beta: Math.atan2(y, x) };
}

/** Elbow branch (+1 / −1) of the virtual 2R; 0 on the boundary where both branches coincide. */
export function elbowBranch(L: number[], th: Vec): 1 | -1 | 0 {
  const { beta } = virtualLink(L, th);
  const e = wrapToPi(th[1] + beta);
  if (Math.abs(Math.sin(e)) < 1e-9) return 0;
  return e > 0 ? 1 : -1;
}

/**
 * Put the tip at `target` keeping θ3 (3R) and the elbow branch; targets outside the reachable
 * annulus are projected onto it. Uses `planar2RIK` from `@/mr` on the virtual 2R.
 */
export function solveTip(L: number[], th: Vec, target: Pt2, branch: 1 | -1): number[] {
  const { lp, beta } = virtualLink(L, th);
  const rmin = Math.abs(L[0] - lp), rmax = L[0] + lp;
  let [x, y] = target;
  const r = Math.hypot(x, y);
  if (r < 1e-9) {
    const tip = planarFK(L, th).tip;
    const tr = Math.hypot(tip[0], tip[1]) || 1;
    x = (tip[0] / tr) * Math.max(rmin, 1e-9); y = (tip[1] / tr) * Math.max(rmin, 1e-9);
  } else if (r > rmax) { x *= rmax / r; y *= rmax / r; }
  else if (r < rmin) { x *= rmin / r; y *= rmin / r; }
  const ik = planar2RIK(L[0], lp, x, y);
  if (!ik.reachable || ik.solutions.length === 0) return th.slice();
  const sol = ik.solutions.length === 1 ? ik.solutions[0]
    : ik.solutions.find((s) => (branch > 0 ? s.theta2 >= 0 : s.theta2 < 0)) ?? ik.solutions[0];
  const out = th.slice();
  out[0] = wrapAngle(sol.theta1);
  out[1] = wrapAngle(sol.theta2 - beta);
  return out;
}

/** The other inverse-kinematics solution for the same tip (elbow flipped); null on the boundary. */
export function otherSolution(L: number[], th: Vec): number[] | null {
  const br = elbowBranch(L, th);
  if (br === 0) return null;
  const tip = planarFK(L, th).tip;
  return solveTip(L, th, [tip[0], tip[1]], br > 0 ? -1 : 1);
}

export interface Posture { label: string; th: number[] }

/** Posture presets for the 2R comparison table (same load, different poses). */
export const POSTURES_2R: Posture[] = [
  { label: '水平伸直', th: [0, 0] },
  { label: '垂直伸直', th: [Math.PI / 2, 0] },
  { label: '肘下：末端 (1, 1)', th: [0, Math.PI / 2] },
  { label: '肘上：末端 (1, 1)', th: [Math.PI / 2, -Math.PI / 2] },
  { label: '收在身前', th: [Math.PI / 3, (2 * Math.PI) / 3] },
];

/** Joint power θ̇ᵀτ and tip power Vᵀ F for a virtual joint velocity (both in W). */
export function powers(model: RobotModel, th: Vec, qdot: Vec, f: Pt2): { joint: number; tip: number; v: Pt2 } {
  const { tau } = staticTorques(model, th, f);
  const J = tipJacobian(model.linkLengths ?? [], th);
  const v = matVec(J, qdot);
  return {
    joint: qdot.reduce((s, q, i) => s + q * tau[i], 0),
    tip: v[0] * f[0] + v[1] * f[1],
    v: [v[0], v[1]],
  };
}
