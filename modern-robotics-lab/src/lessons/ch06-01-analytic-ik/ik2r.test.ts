import { describe, expect, it } from 'vitest';
import { planarFK, wrapToPi } from '@/mr';
import {
  annulus, atanTheta1, beta, byBranch, choose, followSegment, newton2R, pickNearest, quadrantOf,
  snapToBoundary, solve2R, solve3RDecoupled, tipOf, type Pt2,
} from './ik2r';

const close = (a: number, b: number, tol = 1e-9) => Math.abs(a - b) < tol;

describe('solve2R', () => {
  it('returns two solutions whose forward kinematics hit the target', () => {
    const cases: [number, number, number, number][] = [
      [1, 1, 1.2, 0.9], [1, 1, -0.7, 1.1], [1.2, 0.7, -1.1, -0.9], [0.8, 1.3, 0.4, -1.5],
    ];
    for (const [L1, L2, x, y] of cases) {
      const ik = solve2R(L1, L2, x, y);
      expect(ik.reach).toBe('two');
      expect(ik.solutions.map((s) => s.branch)).toEqual(['down', 'up']);
      expect(ik.solutions[0].theta2).toBeGreaterThan(0);
      expect(ik.solutions[1].theta2).toBeLessThan(0);
      // the two branches are mirror images: θ₂ ↦ −θ₂
      expect(close(ik.solutions[0].theta2, -ik.solutions[1].theta2)).toBe(true);
      for (const s of ik.solutions) {
        const tip = planarFK([L1, L2], [s.theta1, s.theta2]).tip;
        expect(close(tip[0], x, 1e-9) && close(tip[1], y, 1e-9)).toBe(true);
        expect(close(Math.cos(s.theta2), ik.c2, 1e-12)).toBe(true);
      }
    }
  });

  it('merges the two solutions on the boundary and has none outside', () => {
    const outer = solve2R(1, 0.6, 1.6 * Math.cos(2), 1.6 * Math.sin(2));
    expect(outer.reach).toBe('one');
    expect(outer.solutions[0].theta2).toBeCloseTo(0, 6);
    const inner = solve2R(1, 0.6, 0.4 * Math.cos(-1), 0.4 * Math.sin(-1));
    expect(inner.reach).toBe('one');
    expect(Math.abs(inner.solutions[0].theta2)).toBeCloseTo(Math.PI, 6);
    expect(byBranch(inner, 'up')).toBe(inner.solutions[0]);

    const far = solve2R(1, 0.6, 1.5, 1.0);
    expect(far.reach).toBe('outside');
    expect(far.c2).toBeGreaterThan(1);
    const hole = solve2R(1, 0.6, 0.1, -0.2);
    expect(hole.reach).toBe('inside');
    expect(hole.c2).toBeLessThan(-1);
  });

  it('gives the closest reachable pose as fallback for unreachable targets', () => {
    for (const [L1, L2, p] of [[1, 0.6, [1.5, 1.0]], [1, 0.6, [0.1, -0.2]], [0.6, 1, [-0.1, 0.2]]] as [number, number, Pt2][]) {
      const ik = solve2R(L1, L2, p[0], p[1]);
      const tip = tipOf(L1, L2, ik.fallback!);
      const { rIn, rOut } = annulus(L1, L2);
      const want = ik.reach === 'outside' ? rOut : rIn;
      const r = Math.hypot(p[0], p[1]);
      expect(Math.hypot(tip[0] - (p[0] * want) / r, tip[1] - (p[1] * want) / r)).toBeLessThan(1e-9);
    }
  });

  it('flags the base as degenerate when L1 = L2 (any θ₁ works)', () => {
    const ik = solve2R(1, 1, 0, 0);
    expect(ik.degenerate).toBe(true);
    expect(ik.reach).toBe('one');
    const keep = choose(ik, 'nearest', { theta1: 1.234, theta2: 3.0, branch: 'down' });
    expect(keep.theta1).toBe(1.234);
    expect(keep.theta2).toBe(Math.PI);
    expect(solve2R(1, 1, 0.3, 0.1).degenerate).toBe(false);
  });

  it('β matches the half-angle identity for equal links', () => {
    for (const t2 of [0.3, 1.2, 2.9, -2.0]) expect(beta(1, 1, t2)).toBeCloseTo(t2 / 2, 12);
  });
});

describe('atan vs atan2', () => {
  it('agrees in the right half-plane and is off by π in the left half-plane', () => {
    const right = solve2R(1, 1, 1.1, 0.6);
    const sR = right.solutions[0];
    expect(atanTheta1(1.1, 0.6, beta(1, 1, sR.theta2))).toBeCloseTo(sR.theta1, 12);

    const ik = solve2R(1, 1, -1.2, -0.5);
    const s = ik.solutions[0];
    const wrong = atanTheta1(-1.2, -0.5, beta(1, 1, s.theta2));
    expect(Math.abs(wrapToPi(wrong - s.theta1))).toBeCloseTo(Math.PI, 12);
    const tip = tipOf(1, 1, { theta1: wrong, theta2: s.theta2 });
    expect(tip[0]).toBeCloseTo(1.2, 12);
    expect(tip[1]).toBeCloseTo(0.5, 12);
  });

  it('names the quadrants', () => {
    expect(quadrantOf(1, 1)).toBe('第一象限');
    expect(quadrantOf(-1, 1)).toBe('第二象限');
    expect(quadrantOf(-1, -1)).toBe('第三象限');
    expect(quadrantOf(1, -1)).toBe('第四象限');
  });
});

describe('following a moving target', () => {
  const start = (mode: 'down' | 'up' | 'nearest', p: Pt2) => choose(solve2R(1, 1, p[0], p[1]), mode, null);

  it('a locked branch jumps by π when the path crosses the base (L1 = L2)', () => {
    const a: Pt2 = [-1.5, 0];
    const res = followSegment(1, 1, start('down', a), a, [1.5, 0], 'down');
    expect(res.jump).toBeGreaterThan(3);
    expect(res.sol.branch).toBe('down');
  });

  it('nearest-solution tracking stays continuous and switches the branch label', () => {
    const a: Pt2 = [-1.5, 0];
    const res = followSegment(1, 1, start('down', a), a, [1.5, 0], 'nearest');
    expect(res.jump).toBeLessThan(0.1);
    expect(res.sol.branch).toBe('up');
    // the elbow never changed side: it stays above the path
    expect(Math.sin(res.sol.theta1)).toBeGreaterThan(0.5);
  });

  it('a path that avoids the base and the outer circle keeps one branch continuously', () => {
    const a: Pt2 = [-1.5, 0.2];
    const locked = followSegment(1, 1, start('down', a), a, [1.5, 0.2], 'down');
    expect(locked.jump).toBeLessThan(0.35);
    const nearest = followSegment(1, 1, start('down', a), a, [1.5, 0.2], 'nearest');
    expect(nearest.sol.branch).toBe('down');
    expect(nearest.sol.theta1).toBeCloseTo(locked.sol.theta1, 12);
  });

  it('pickNearest returns the solution closest on the torus', () => {
    const ik = solve2R(1, 1, 1.0, 1.0);
    const up = byBranch(ik, 'up')!;
    expect(pickNearest(ik, { ...up, theta1: up.theta1 + 0.05 })).toBe(up);
  });
});

describe('snapToBoundary', () => {
  it('lands exactly on a circle so the merged solution appears', () => {
    const p = snapToBoundary(1, 0.6, [1.598, 0.02], 0.01);
    expect(Math.hypot(p[0], p[1])).toBeCloseTo(1.6, 12);
    expect(solve2R(1, 0.6, p[0], p[1]).reach).toBe('one');
    const q = snapToBoundary(1, 0.6, [0.0, 0.405], 0.01);
    expect(Math.hypot(q[0], q[1])).toBeCloseTo(0.4, 12);
    expect(snapToBoundary(1, 1, [0.004, -0.003], 0.01)).toEqual([0, 0]);
    expect(snapToBoundary(1, 1, [0.9, 0.3], 0.01)).toEqual([0.9, 0.3]);
  });
});

describe('3R wrist decoupling', () => {
  it('reaches the target position and orientation with both 2R branches', () => {
    const [L1, L2, L3] = [1, 1, 0.5];
    const [x, y, phi] = [0.9, 1.1, 0.7];
    const { sols } = solve3RDecoupled(L1, L2, L3, x, y, phi);
    expect(sols.length).toBe(2);
    for (const s of sols) {
      const fk = planarFK([L1, L2, L3], [s.theta1, s.theta2, s.theta3]);
      expect(fk.tip[0]).toBeCloseTo(x, 10);
      expect(fk.tip[1]).toBeCloseTo(y, 10);
      expect(wrapToPi(fk.phi - phi)).toBeCloseTo(0, 10);
    }
  });
});

describe('newton2R preview', () => {
  it('converges to one of the analytic solutions from a reasonable guess', () => {
    const target: Pt2 = [1.1, 0.8];
    const res = newton2R(1, 1, [0, 0.6], target, 10);
    expect(res.converged).toBe(true);
    expect(res.thetas.length).toBeLessThanOrEqual(8);
    const last = res.thetas[res.thetas.length - 1];
    const ik = solve2R(1, 1, target[0], target[1]);
    const match = ik.solutions.some((s) => Math.abs(wrapToPi(s.theta1 - last[0])) < 1e-6 && Math.abs(wrapToPi(s.theta2 - last[1])) < 1e-6);
    expect(match).toBe(true);
    // errors shrink monotonically near the end (quadratic convergence)
    const e = res.errors;
    expect(e[e.length - 1]).toBeLessThan(e[e.length - 2]);
  });
});
