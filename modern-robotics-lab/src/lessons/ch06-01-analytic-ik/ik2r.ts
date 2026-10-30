/**
 * Closed-form planar 2R inverse kinematics helpers for the analytic-IK lesson.
 * The solutions themselves come from `planar2RIK` in `@/mr`; this file adds the bookkeeping the
 * lesson needs: branch selection, continuity (nearest-solution) tracking, the nearest reachable
 * pose for unreachable targets and the planar "wrist decoupling" of a 3R arm.
 */
import { planar2RIK, wrapToPi } from '@/mr';

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
  /** Raw right-hand side of the cosine law; may fall marginally outside [-1, 1] on the boundary. */
  c2: number;
  /** Direction to the target, atan2(y, x). */
  gamma: number;
  /** 'down' first, then 'up'; a single entry on the workspace boundary; empty when unreachable. */
  solutions: Solution[];
  /** Nearest reachable configuration (arm stretched or folded towards the target) when unreachable. */
  fallback: Solution | null;
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
    return { reach: outside ? 'outside' : 'inside', r, c2, gamma, solutions: [], fallback: { theta1, theta2, branch: 'down' } };
  }
  const sols: Solution[] = ik.solutions.map((s) => ({ theta1: wrapToPi(s.theta1), theta2: s.theta2, branch: s.elbow }));
  sols.sort((a, b) => (a.branch === b.branch ? 0 : a.branch === 'down' ? -1 : 1));
  return { reach: sols.length === 1 ? 'one' : 'two', r, c2, gamma, solutions: sols, fallback: null };
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

/** Wrist centre of a planar 3R arm whose last link (length L3) must point along φ at the target. */
export function wristCenter(x: number, y: number, phi: number, L3: number): [number, number] {
  return [x - L3 * Math.cos(phi), y - L3 * Math.sin(phi)];
}

/** Third joint angle that realises the orientation φ once θ₁, θ₂ are known. */
export function theta3For(sol: Solution, phi: number): number {
  return wrapToPi(phi - sol.theta1 - sol.theta2);
}

/** θ₁ computed with the single-argument arctangent: off by π whenever x < 0. */
export function atanTheta1(x: number, y: number, b: number): number {
  const g = x === 0 ? (y >= 0 ? Math.PI / 2 : -Math.PI / 2) : Math.atan(y / x);
  return wrapToPi(g - b);
}
