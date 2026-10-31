/**
 * Closed-form planar 2R inverse kinematics helpers for the analytic-IK lesson.
 * The solutions themselves come from `planar2RIK` in `@/mr`; this file adds the bookkeeping the
 * lesson needs: branch selection, continuity (nearest-solution) tracking along a moving target,
 * the degenerate origin case (L1 = L2), the nearest reachable pose for unreachable targets,
 * boundary snapping, the planar "wrist decoupling" of a 3R arm and a tiny Newton preview.
 */
import { planar2RIK, planarFK, planarTipJacobian, matVec, pinv, wrapToPi } from '@/mr';

export type Pt2 = [number, number];
export type Branch = 'down' | 'up';

export interface Solution {
  theta1: number;
  theta2: number;
  /** 'down' when θ₂ ≥ 0, 'up' when θ₂ < 0 (same convention as planar2RIK's `elbow`). */
  branch: Branch;
}

export type Reach = 'two' | 'one' | 'outside' | 'inside';

export interface IK2R {
  reach: Reach;
  /** Distance from the base to the target. */
  r: number;
  /** Raw right-hand side of the cosine law; may fall outside [-1, 1] (then there is no solution). */
  c2: number;
  /** Direction to the target, atan2(y, x). */
  gamma: number;
  /** 'down' first, then 'up'; a single entry on the workspace boundary; empty when unreachable. */
  solutions: Solution[];
  /** Nearest reachable configuration (arm stretched or folded towards the target) when unreachable. */
  fallback: Solution | null;
  /**
   * True at the base when L1 = L2: the target is reachable with θ₂ = π and ANY θ₁
   * (infinitely many solutions). `solutions` then holds one representative.
   */
  degenerate: boolean;
}

/** Radius below which a target at the base counts as "the origin" (degenerate case). */
const ORIGIN_EPS = 1e-6;

/** Inner and outer radius of the reachable annulus. */
export function annulus(L1: number, L2: number): { rIn: number; rOut: number } {
  return { rIn: Math.abs(L1 - L2), rOut: L1 + L2 };
}

/** Correction angle inside the base–elbow–target triangle: β = atan2(L₂ sin θ₂, L₁ + L₂ cos θ₂). */
export function beta(L1: number, L2: number, theta2: number): number {
  return Math.atan2(L2 * Math.sin(theta2), L1 + L2 * Math.cos(theta2));
}

export function solve2R(L1: number, L2: number, x: number, y: number): IK2R {
  const ik = planar2RIK(L1, L2, x, y);
  const r = ik.distance;
  const c2 = (r * r - L1 * L1 - L2 * L2) / (2 * L1 * L2);
  const gamma = Math.atan2(y, x);
  if (!ik.reachable) {
    const outside = c2 > 1;
    const theta2 = outside ? 0 : Math.PI;
    const theta1 = wrapToPi(gamma - beta(L1, L2, theta2));
    return { reach: outside ? 'outside' : 'inside', r, c2, gamma, solutions: [], fallback: { theta1, theta2, branch: 'down' }, degenerate: false };
  }
  const sols: Solution[] = ik.solutions.map((s) => ({ theta1: wrapToPi(s.theta1), theta2: s.theta2, branch: s.elbow }));
  sols.sort((a, b) => (a.branch === b.branch ? 0 : a.branch === 'down' ? -1 : 1));
  const degenerate = r < ORIGIN_EPS;
  return { reach: sols.length === 1 ? 'one' : 'two', r, c2, gamma, solutions: sols, fallback: null, degenerate };
}

/** The solution on a given branch; on the boundary the merged solution counts for both branches. */
export function byBranch(ik: IK2R, b: Branch): Solution | null {
  if (ik.solutions.length === 1) return ik.solutions[0];
  return ik.solutions.find((s) => s.branch === b) ?? null;
}

/** Distance in joint space on the torus (both angles wrapped). */
export function jointDistance(a: Solution, b: Solution): number {
  return Math.hypot(wrapToPi(a.theta1 - b.theta1), wrapToPi(a.theta2 - b.theta2));
}

/** The solution closest (on the torus) to the previous one: keeps the motion continuous. */
export function pickNearest(ik: IK2R, prev: Solution | null): Solution | null {
  if (!ik.solutions.length) return null;
  if (!prev) return ik.solutions[0];
  let best = ik.solutions[0];
  let bestD = Infinity;
  for (const s of ik.solutions) {
    const d = jointDistance(s, prev);
    if (d < bestD) { bestD = d; best = s; }
  }
  return best;
}

/** Display policy: lock one branch, or follow the solution nearest to the previous configuration. */
export type FollowMode = Branch | 'nearest';

/**
 * Configuration shown for a target: the locked branch or the nearest solution; the fallback pose
 * when unreachable. At the degenerate origin (L1 = L2) every θ₁ works, so the previous θ₁ is kept.
 */
export function choose(ik: IK2R, mode: FollowMode, prev: Solution | null): Solution {
  if (!ik.solutions.length) return ik.fallback!;
  if (ik.degenerate && prev) return { theta1: prev.theta1, theta2: Math.PI, branch: 'down' };
  if (mode === 'nearest') return pickNearest(ik, prev)!;
  return byBranch(ik, mode)!;
}

export interface FollowResult {
  sol: Solution;
  ik: IK2R;
  /** Largest joint-space step (rad, on the torus) between consecutive sub-steps. */
  jump: number;
  /** Configuration just before that largest step (for a "before the jump" ghost). */
  jumpFrom: Solution | null;
}

/**
 * Moves the target along the straight segment from → to in sub-steps of at most `ds` (m), applying
 * `choose` at every sub-step. Small sub-steps make the nearest-solution rule reliable even for fast
 * drags, and a large `jump` then really means the displayed configuration was discontinuous.
 */
export function followSegment(L1: number, L2: number, prev: Solution | null, from: Pt2, to: Pt2, mode: FollowMode, ds = 0.01): FollowResult {
  const len = Math.hypot(to[0] - from[0], to[1] - from[1]);
  const n = Math.max(1, Math.min(2000, Math.ceil(len / ds)));
  let cur = prev;
  let jump = 0;
  let jumpFrom: Solution | null = null;
  let ik = solve2R(L1, L2, to[0], to[1]);
  for (let k = 1; k <= n; k++) {
    const s = k / n;
    ik = solve2R(L1, L2, from[0] + (to[0] - from[0]) * s, from[1] + (to[1] - from[1]) * s);
    const next = choose(ik, mode, cur);
    if (cur) {
      const d = jointDistance(next, cur);
      if (d > jump) { jump = d; jumpFrom = cur; }
    }
    cur = next;
  }
  return { sol: cur!, ik, jump, jumpFrom };
}

/** Elbow position of a 2R configuration. */
export function elbowOf(L1: number, s: { theta1: number }): Pt2 {
  return [L1 * Math.cos(s.theta1), L1 * Math.sin(s.theta1)];
}

/** Tip position of a 2R configuration. */
export function tipOf(L1: number, L2: number, s: { theta1: number; theta2: number }): Pt2 {
  const a = s.theta1 + s.theta2;
  return [L1 * Math.cos(s.theta1) + L2 * Math.cos(a), L1 * Math.sin(s.theta1) + L2 * Math.sin(a)];
}

/**
 * Snaps a target onto the outer/inner boundary circle (or onto the base when L1 = L2) when it is
 * within `tol` of it, so the learner can land exactly on the merged single solution.
 */
export function snapToBoundary(L1: number, L2: number, p: Pt2, tol: number): Pt2 {
  const { rIn, rOut } = annulus(L1, L2);
  const r = Math.hypot(p[0], p[1]);
  if (rIn < 1e-9 && r < tol) return [0, 0];
  if (r < 1e-12) return p;
  if (Math.abs(r - rOut) < tol) return [(p[0] * rOut) / r, (p[1] * rOut) / r];
  if (rIn > 1e-9 && Math.abs(r - rIn) < tol) return [(p[0] * rIn) / r, (p[1] * rIn) / r];
  return p;
}

/** Quadrant of a point, for the atan2 readout. */
export function quadrantOf(x: number, y: number): string {
  if (Math.abs(x) < 1e-12 || Math.abs(y) < 1e-12) return '座標軸上';
  if (x > 0) return y > 0 ? '第一象限' : '第四象限';
  return y > 0 ? '第二象限' : '第三象限';
}

/** Wrist centre of a planar 3R arm whose last link (length L3) must point along φ at the target. */
export function wristCenter(x: number, y: number, phi: number, L3: number): Pt2 {
  return [x - L3 * Math.cos(phi), y - L3 * Math.sin(phi)];
}

/** Third joint angle that realises the orientation φ once θ₁, θ₂ are known. */
export function theta3For(sol: Solution, phi: number): number {
  return wrapToPi(phi - sol.theta1 - sol.theta2);
}

export interface Solution3R extends Solution { theta3: number }

/**
 * Planar 3R IK by wrist decoupling: the orientation φ fixes the last link, the wrist centre is
 * solved with the 2R formulas, and θ₃ = φ − θ₁ − θ₂ restores the orientation.
 */
export function solve3RDecoupled(L1: number, L2: number, L3: number, x: number, y: number, phi: number): { w: Pt2; ik: IK2R; sols: Solution3R[] } {
  const w = wristCenter(x, y, phi, L3);
  const ik = solve2R(L1, L2, w[0], w[1]);
  const sols = ik.solutions.map((s) => ({ ...s, theta3: theta3For(s, phi) }));
  return { w, ik, sols };
}

/** θ₁ computed with the single-argument arctangent: off by π whenever x < 0. */
export function atanTheta1(x: number, y: number, b: number): number {
  const g = x === 0 ? (y >= 0 ? Math.PI / 2 : -Math.PI / 2) : Math.atan(y / x);
  return wrapToPi(g - b);
}

/**
 * A few Newton–Raphson iterations on the 2R tip position (preview of the next lesson):
 * θ ← θ + J⁺(θ) (p_d − f(θ)). Returns every visited configuration and its position error.
 */
export function newton2R(L1: number, L2: number, theta0: Pt2, target: Pt2, maxIter = 8, tol = 1e-9): { thetas: Pt2[]; errors: number[]; converged: boolean } {
  const L = [L1, L2];
  let th: Pt2 = [theta0[0], theta0[1]];
  const thetas: Pt2[] = [];
  const errors: number[] = [];
  for (let k = 0; k <= maxIter; k++) {
    const tip = planarFK(L, th).tip;
    const e = [target[0] - tip[0], target[1] - tip[1]];
    thetas.push([th[0], th[1]]);
    errors.push(Math.hypot(e[0], e[1]));
    if (errors[k] < tol || k === maxIter) break;
    const d = matVec(pinv(planarTipJacobian(L, th)), e);
    th = [wrapToPi(th[0] + d[0]), wrapToPi(th[1] + d[1])];
  }
  return { thetas, errors, converged: errors[errors.length - 1] < tol };
}
