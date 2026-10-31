/**
 * Pure helpers for the singularity / manipulability lesson (no DOM).
 *
 * Everything works with the 2 x n LINEAR tip Jacobian J_v of a planar nR arm
 * (rows = tip velocity v_x, v_y in the fixed frame), taken from `@/mr`
 * (`planarTipJacobian`). Angular velocity rows are deliberately left out so the
 * ellipse never mixes m/s with rad/s.
 */
import { planarFK, planarTipJacobian, planar2RIK, pinv, matVec, wrapToPi, type Mat, type Vec } from '@/mr';

export type Pt2 = [number, number];

/** Link lengths of the two lab models (same numbers as `planar2R` / `planar3R` in `@/mr`). */
export const L2R = [1, 1];
export const L3R = [1, 0.8, 0.6];

/** smin / smax below this ratio is reported as rank deficient (exactly singular up to rounding). */
export const SING_RATIO = 1e-6;

/** Wrap an angle to (−π, π] (so a folded joint reads +180°, matching the slider's maximum). */
export function wrapAngle(a: number): number {
  const w = wrapToPi(a);
  return w <= -Math.PI ? Math.PI : w;
}

export interface SigmaInfo {
  /** Largest / smallest singular value of J_v (semi-axes of the velocity ellipse for |θ̇| = 1). */
  smax: number;
  smin: number;
  /** μ1 = smax / smin (Infinity when rank deficient). */
  mu1: number;
  /** μ3 = sqrt(det(J Jᵀ)) = σ1 σ2 (proportional to the ellipse area). */
  mu3: number;
  /** Numerical rank of J_v (0, 1 or 2). */
  rank: number;
  /** Unit vector of the major / minor axis (eigenvectors of J Jᵀ). */
  u1: Pt2;
  u2: Pt2;
  /** Entries of the symmetric J Jᵀ = [[a, b], [b, d]]. */
  JJt: [[number, number], [number, number]];
}

/** 2 x n linear tip Jacobian (thin wrapper so the lesson has one call site). */
export function tipJacobian(L: number[], th: Vec): Mat {
  return planarTipJacobian(L, th);
}

/**
 * Singular values of a 2 x n matrix in closed form.
 * det(J Jᵀ) is evaluated with the Cauchy–Binet sum of squared 2x2 minors, which stays
 * accurate (and non-negative) right at a singularity where a·d − b² would cancel.
 */
export function sigmaInfo(J: Mat): SigmaInfo {
  const n = J[0].length;
  let a = 0, b = 0, d = 0, det = 0;
  for (let i = 0; i < n; i++) {
    a += J[0][i] * J[0][i];
    b += J[0][i] * J[1][i];
    d += J[1][i] * J[1][i];
  }
  for (let i = 0; i < n; i++) {
    for (let j = i + 1; j < n; j++) {
      const m = J[0][i] * J[1][j] - J[0][j] * J[1][i];
      det += m * m;
    }
  }
  const l1 = (a + d) / 2 + Math.hypot((a - d) / 2, b);
  const l2 = l1 > 0 ? det / l1 : 0;
  const smax = Math.sqrt(Math.max(0, l1));
  const smin = Math.sqrt(Math.max(0, l2));
  const ang = 0.5 * Math.atan2(2 * b, a - d);
  const u1: Pt2 = [Math.cos(ang), Math.sin(ang)];
  const u2: Pt2 = [-u1[1], u1[0]];
  let rank = 0;
  if (smax > 1e-12) rank = smin > SING_RATIO * smax ? 2 : 1;
  return {
    smax, smin,
    mu1: rank === 2 ? smax / smin : Infinity,
    mu3: Math.sqrt(det),
    rank, u1, u2,
    JJt: [[a, b], [b, d]],
  };
}

/** Joint positions (base … joint n) and tip of a planar chain, from `planarFK`. */
export function armPoints(L: number[], th: Vec): Pt2[] {
  return planarFK(L, th).joints.map((p) => [p[0], p[1]] as Pt2);
}

/**
 * Links 2..n of a 3R arm with θ3 frozen behave like ONE virtual link from joint 2 to the tip:
 * length lp and angular offset β relative to link 2. For a 2R arm lp = L2, β = 0.
 */
export function virtualLink(L: number[], th: Vec): { lp: number; beta: number } {
  if (L.length === 2) return { lp: L[1], beta: 0 };
  const t3 = th[2];
  const x = L[1] + L[2] * Math.cos(t3), y = L[2] * Math.sin(t3);
  return { lp: Math.hypot(x, y), beta: Math.atan2(y, x) };
}

/** Elbow branch of the (virtual) 2R: +1 when the virtual elbow angle θ2 + β ∈ [0, π], else −1. */
export function elbowBranch(L: number[], th: Vec): 1 | -1 {
  const { beta } = virtualLink(L, th);
  return wrapToPi(th[1] + beta) >= 0 ? 1 : -1;
}

/** Radii reachable by the tip when θ3 (3R) is frozen: [|L1 − lp|, L1 + lp]. */
export function reachAnnulus(L: number[], th: Vec): [number, number] {
  const { lp } = virtualLink(L, th);
  return [Math.abs(L[0] - lp), L[0] + lp];
}

/**
 * Move the tip to `target` keeping θ3 (for 3R) and the elbow branch. Targets outside the
 * reachable annulus are projected onto it (`clamped` = true). Uses `planar2RIK` from `@/mr`
 * on the virtual 2R (L1, lp).
 */
export function solveTip(L: number[], th: Vec, target: Pt2, branch: 1 | -1): { th: number[]; clamped: boolean } {
  const { lp, beta } = virtualLink(L, th);
  const [rmin, rmax] = [Math.abs(L[0] - lp), L[0] + lp];
  let [x, y] = target;
  let r = Math.hypot(x, y);
  let clamped = false;
  if (r < 1e-9) {
    // Direction undefined at the base: keep the current heading of the tip.
    const tip = planarFK(L, th).tip;
    const tr = Math.hypot(tip[0], tip[1]);
    [x, y] = tr > 1e-9 ? [(tip[0] / tr) * 1e-9, (tip[1] / tr) * 1e-9] : [1e-9, 0];
    r = 1e-9;
  }
  if (r > rmax) { x *= rmax / r; y *= rmax / r; clamped = true; }
  else if (r < rmin) { x *= rmin / r; y *= rmin / r; clamped = true; }
  const ik = planar2RIK(L[0], lp, x, y);
  if (!ik.reachable || ik.solutions.length === 0) return { th: th.slice(), clamped: true };
  const sol = ik.solutions.length === 1 ? ik.solutions[0]
    : ik.solutions.find((s) => (branch > 0 ? s.theta2 >= 0 : s.theta2 < 0)) ?? ik.solutions[0];
  const out = th.slice();
  out[0] = wrapAngle(sol.theta1);
  out[1] = wrapAngle(sol.theta2 - beta);
  return { th: out, clamped };
}

export interface Sweep {
  /** θ2 samples in degrees. */
  deg: number[];
  smax: number[];
  smin: number[];
  mu3: number[];
  /** μ1, with NaN where J is rank deficient (the plot lifts the pen there). */
  mu1: number[];
}

/** σ's as functions of θ2 with the other joints fixed (θ1 does not matter: it only rotates the ellipse). */
export function sweepTheta2(L: number[], th: Vec, N = 361): Sweep {
  const out: Sweep = { deg: [], smax: [], smin: [], mu3: [], mu1: [] };
  for (let k = 0; k < N; k++) {
    const t2 = -Math.PI + (2 * Math.PI * k) / (N - 1);
    const q = th.slice(); q[1] = t2;
    const s = sigmaInfo(tipJacobian(L, q));
    out.deg.push((t2 * 180) / Math.PI);
    out.smax.push(s.smax); out.smin.push(s.smin); out.mu3.push(s.mu3);
    out.mu1.push(Number.isFinite(s.mu1) ? s.mu1 : NaN);
  }
  return out;
}

export interface RadialProfile { rmin: number; rmax: number; values: Float64Array }

/**
 * σ_min as a function of the tip radius r for the configuration family "θ3 frozen, given elbow
 * branch". θ1 only rotates the arm, so σ_min is rotationally symmetric and a 1-D table is enough
 * to paint the whole workspace heat map quickly.
 */
export function sigmaMinProfile(L: number[], th: Vec, branch: 1 | -1, N = 400): RadialProfile {
  const [rmin, rmax] = reachAnnulus(L, th);
  const values = new Float64Array(N);
  for (let k = 0; k < N; k++) {
    const r = rmin + ((rmax - rmin) * k) / (N - 1);
    const q = solveTip(L, th, [Math.max(r, 1e-9), 0], branch).th;
    values[k] = sigmaInfo(tipJacobian(L, q)).smin;
  }
  return { rmin, rmax, values };
}

/** Linear interpolation in a radial profile; NaN outside the reachable annulus. */
export function sampleProfile(p: RadialProfile, r: number): number {
  if (r < p.rmin - 1e-9 || r > p.rmax + 1e-9) return NaN;
  const N = p.values.length;
  const u = p.rmax > p.rmin ? ((r - p.rmin) / (p.rmax - p.rmin)) * (N - 1) : 0;
  const k = Math.max(0, Math.min(N - 2, Math.floor(u)));
  const f = Math.max(0, Math.min(1, u - k));
  return p.values[k] * (1 - f) + p.values[k + 1] * f;
}

export interface RadialRates {
  /** Desired tip velocity (radially outward, |v| = speed). */
  v: Pt2;
  /** Minimum-norm joint velocity J⁺ v (equals J⁻¹ v for 2R); null when J is rank deficient. */
  qdot: number[] | null;
  /** |θ̇| (Infinity when singular). */
  norm: number;
}

/** Joint rates needed for a radial tip speed `speed` (m/s, positive = away from the base). */
export function radialRates(L: number[], th: Vec, speed: number): RadialRates {
  const tip = planarFK(L, th).tip;
  const r = Math.hypot(tip[0], tip[1]);
  const v: Pt2 = r > 1e-9 ? [(speed * tip[0]) / r, (speed * tip[1]) / r] : [speed, 0];
  const J = tipJacobian(L, th);
  const s = sigmaInfo(J);
  if (s.rank < 2) return { v, qdot: null, norm: Infinity };
  const qdot = matVec(pinv(J), v);
  return { v, qdot, norm: Math.hypot(...qdot) };
}

/**
 * Slow-motion factor (simulated seconds per real second) for showing a radial run of the 2R:
 * `playback` far from the singularity; closer in, small enough that the shown θ2 decays like
 * e^(−logRate·t_real), i.e. warp·|dθ2/dt| = logRate·|θ2|. At constant tip speed the joint rates blow
 * up within the last few millimetres, which plain slow motion would show for only a few frames.
 */
export function radialWarp(L: number[], th: Vec, speed: number, playback: number, logRate: number): number {
  const rr = radialRates(L, th, speed);
  const q2 = rr.qdot ? Math.abs(rr.qdot[1]) : Infinity;
  if (!Number.isFinite(q2) || q2 < 1e-12) return playback;
  return Math.min(playback, (logRate * Math.abs(wrapAngle(th[1]))) / q2);
}

/**
 * Configuration of a 2R arm whose tip moves radially outward along the ray through `tip0`
 * (exact, by inverse kinematics on the same branch) after travelling distance `s`.
 */
export function radialPathConfig(L: number[], tip0: Pt2, s: number, branch: 1 | -1, th0: Vec): number[] {
  const r0 = Math.hypot(tip0[0], tip0[1]);
  const dir: Pt2 = r0 > 1e-9 ? [tip0[0] / r0, tip0[1] / r0] : [1, 0];
  const r = r0 + s;
  return solveTip(L, th0, [dir[0] * r, dir[1] * r], branch).th;
}

/** Image of a unit joint velocity: v = J θ̇. */
export function tipVelocity(J: Mat, qdot: Vec): Pt2 {
  const v = matVec(J, qdot);
  return [v[0], v[1]];
}
