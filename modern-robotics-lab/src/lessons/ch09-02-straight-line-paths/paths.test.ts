import { describe, expect, it } from 'vitest';
import { planarFK, RotZ, RpToTrans, wrapToPi, type Mat } from '@/mr';
import {
  LINKS, ik3R, planarPaths, trackPoses, compareSE3, armCollides, wristPoint, angDist,
  type Pose2,
} from './paths';
import { PRESETS_2D } from './presets';

const deg = Math.PI / 180;
const P = (x: number, y: number, d: number): Pose2 => ({ x, y, phi: d * deg });

describe('planar 3R inverse kinematics', () => {
  it('round-trips random configurations through FK (both branches present)', () => {
    let seed = 7;
    const rnd = () => { seed = (seed * 16807) % 2147483647; return seed / 2147483647; };
    for (let i = 0; i < 200; i++) {
      const th = [(-1 + 2 * rnd()) * Math.PI, (-0.95 + 1.9 * rnd()) * Math.PI, (-1 + 2 * rnd()) * Math.PI];
      const f = planarFK(LINKS, th);
      const sols = ik3R({ x: f.tip[0], y: f.tip[1], phi: f.phi });
      expect(sols.length).toBe(2);
      expect(Math.min(...sols.map((s) => angDist(s.theta, th)))).toBeLessThan(1e-8);
      for (const s of sols) {
        const g = planarFK(LINKS, s.theta);
        expect(g.tip[0]).toBeCloseTo(f.tip[0], 9);
        expect(g.tip[1]).toBeCloseTo(f.tip[1], 9);
        expect(wrapToPi(g.phi - f.phi)).toBeCloseTo(0, 9);
      }
    }
  });

  it('reports no solution when the wrist point leaves the annulus 0.2 ≤ |w| ≤ 1.8', () => {
    expect(ik3R(P(2.5, 0, 0))).toHaveLength(0); // |w| = 1.9
    expect(ik3R(P(0.6, 0.1, 0))).toHaveLength(0); // |w| = 0.1
    expect(Math.hypot(...wristPoint(P(0.6, 0.1, 0)))).toBeCloseTo(0.1, 12);
  });
});

describe('the three straight-line paths (planar)', () => {
  const basic = PRESETS_2D.basic;
  const r = planarPaths(basic.start, basic.end, basic.elbow, basic.policy, null);

  it('all three start and end at the requested poses', () => {
    expect(r.ok).toBe(true);
    for (const k of ['joint', 'screw', 'cartesian'] as const) {
      const p = r.paths[k];
      const a = p.target[0], b = p.target[p.target.length - 1];
      expect(a.x).toBeCloseTo(basic.start.x, 9); expect(a.y).toBeCloseTo(basic.start.y, 9);
      expect(b.x).toBeCloseTo(basic.end.x, 9); expect(b.y).toBeCloseTo(basic.end.y, 9);
      expect(wrapToPi(b.phi - basic.end.phi)).toBeCloseTo(0, 9);
    }
  });

  it('joint path: θ(s) is linear, so dθ/ds is constant', () => {
    const p = r.paths.joint;
    const d0 = p.dtheta[0]!;
    for (const d of p.dtheta) for (let i = 0; i < 3; i++) expect(d![i]).toBeCloseTo(d0[i], 9);
    for (let i = 0; i < 3; i++) expect(d0[i]).toBeCloseTo(r.thetaEnd[i] - r.thetaStart[i], 9);
  });

  it('screw path: the tip turns about a fixed pole (a circular arc) by Δφ·s', () => {
    const p = r.paths.screw;
    const [cx, cy] = r.pole!;
    const R0 = Math.hypot(basic.start.x - cx, basic.start.y - cy);
    p.target.forEach((t, k) => {
      expect(Math.hypot(t.x - cx, t.y - cy)).toBeCloseTo(R0, 9);
      expect(wrapToPi(t.phi - basic.start.phi - r.dphi * p.s[k])).toBeCloseTo(0, 9);
    });
  });

  it('Cartesian path: the tip moves on a straight line with evenly spaced samples', () => {
    const p = r.paths.cartesian;
    const N = p.target.length;
    p.target.forEach((t, k) => {
      const s = k / (N - 1);
      expect(t.x).toBeCloseTo(basic.start.x + s * (basic.end.x - basic.start.x), 9);
      expect(t.y).toBeCloseTo(basic.start.y + s * (basic.end.y - basic.start.y), 9);
    });
    expect(p.tipLength).toBeCloseTo(Math.hypot(basic.end.x - basic.start.x, basic.end.y - basic.start.y), 9);
  });

  it('IK along the tracked paths reproduces every commanded pose', () => {
    for (const k of ['screw', 'cartesian'] as const) {
      const p = r.paths[k];
      expect(p.failCount).toBe(0);
      expect(p.jumps).toHaveLength(0);
      p.theta.forEach((th, i) => {
        const f = planarFK(LINKS, th!);
        expect(f.tip[0]).toBeCloseTo(p.target[i].x, 9);
        expect(f.tip[1]).toBeCloseTo(p.target[i].y, 9);
      });
    }
  });

  it('the three tip paths really are different', () => {
    const mid = (k: 'joint' | 'screw' | 'cartesian') => r.paths[k].target[100];
    const d = (a: Pose2, b: Pose2) => Math.hypot(a.x - b.x, a.y - b.y);
    expect(d(mid('joint'), mid('cartesian'))).toBeGreaterThan(0.3);
    expect(d(mid('screw'), mid('cartesian'))).toBeGreaterThan(0.3);
    expect(d(mid('joint'), mid('screw'))).toBeGreaterThan(0.1);
  });
});

describe('lesson presets behave as the narrative claims', () => {
  it('near the base: continuous, but the Cartesian joint speed spikes far above the joint path', () => {
    const p = PRESETS_2D.nearBase;
    const r = planarPaths(p.start, p.end, p.elbow, p.policy, null);
    const c = r.paths.cartesian;
    expect(c.failCount).toBe(0);
    expect(c.jumps).toHaveLength(0);
    expect(c.minSin2).toBeLessThan(0.25);
    expect(c.maxSpeed).toBeGreaterThan(3 * r.paths.joint.maxSpeed);
    // A pure translation: the screw path coincides with the straight line.
    expect(Math.abs(r.dphi)).toBeLessThan(1e-12);
    expect(r.pole).toBeNull();
  });

  it('through the unreachable core: IK failures followed by a jump', () => {
    const p = PRESETS_2D.unreachable;
    const r = planarPaths(p.start, p.end, p.elbow, p.policy, null);
    const c = r.paths.cartesian;
    expect(c.failCount).toBeGreaterThan(5);
    expect(c.jumps.length).toBeGreaterThan(0);
    expect(r.paths.joint.failCount).toBe(0);
  });

  it('cold-started IK switches branches while warm-started IK stays continuous', () => {
    const p = PRESETS_2D.cold;
    const cold = planarPaths(p.start, p.end, p.elbow, 'cold', null).paths.cartesian;
    const warm = planarPaths(p.start, p.end, p.elbow, 'warm', null).paths.cartesian;
    expect(cold.failCount).toBe(0);
    expect(cold.jumps.length).toBeGreaterThan(0);
    expect(cold.minSin2).toBeGreaterThan(0.3); // not a singularity effect
    expect(warm.jumps).toHaveLength(0);
  });

  it('obstacle A blocks the joint arc but not the Cartesian line; obstacle B the reverse', () => {
    const a = PRESETS_2D.obstacleA, b = PRESETS_2D.obstacleB;
    const ra = planarPaths(a.start, a.end, a.elbow, a.policy, a.obstacle!);
    expect(ra.paths.joint.firstCollision).toBeGreaterThanOrEqual(0);
    expect(ra.paths.cartesian.firstCollision).toBe(-1);
    const rb = planarPaths(b.start, b.end, b.elbow, b.policy, b.obstacle!);
    expect(rb.paths.cartesian.firstCollision).toBeGreaterThanOrEqual(0);
    expect(rb.paths.joint.firstCollision).toBe(-1);
    // Obstacle B is not on the straight tip line itself: the arm's links hit it.
    const ob = b.obstacle!;
    const tipClear = rb.paths.cartesian.target.every((t) => Math.hypot(t.x - ob.c[0], t.y - ob.c[1]) > ob.r);
    expect(tipClear).toBe(true);
  });
});

describe('tracking helpers', () => {
  it('armCollides detects a disk on a link', () => {
    expect(armCollides([0, 0, 0], { c: [0.5, 0.05], r: 0.1 })).toBe(true);
    expect(armCollides([0, 0, 0], { c: [0.5, 0.5], r: 0.1 })).toBe(false);
  });

  it('trackPoses flags unreachable samples', () => {
    const tr = trackPoses([P(1.5, 0, 0), P(3, 0, 0), P(1.5, 0.1, 0)], ik3R(P(1.5, 0, 0))[0].theta, 'warm');
    expect(tr.fail).toEqual([false, true, false]);
  });
});

describe('SE(3): screw vs decoupled interpolation', () => {
  const R0 = RotZ(-30 * deg);
  const Xs = RpToTrans(R0, [0.5, -0.45, 0.25]);
  const mk = (a: number, d: number[]): Mat => RpToTrans(
    [[Math.cos(a), -Math.sin(a), 0], [Math.sin(a), Math.cos(a), 0], [0, 0, 1]].map((row) => [0, 1, 2].map((j) => row[0] * R0[0][j] + row[1] * R0[1][j] + row[2] * R0[2][j])),
    [0.5 + d[0], -0.45 + d[1], 0.25 + d[2]],
  );

  it('same R(s), different p(s), same endpoints', () => {
    const c = compareSE3(Xs, mk(120 * deg, [-0.9, 0.9, 0.4]));
    expect(c.maxRotDiff).toBeLessThan(1e-9);
    expect(c.maxPosDiff).toBeGreaterThan(0.1);
    const last = c.screw.length - 1;
    for (let i = 0; i < 3; i++) expect(c.screw[last][i][3]).toBeCloseTo(c.cartesian[last][i][3], 9);
    expect(c.angle).toBeCloseTo(120 * deg, 9);
    expect(c.axisDir![2]).toBeCloseTo(1, 9); // vertical screw axis
    expect(c.pitch).toBeCloseTo(0.4 / (120 * deg), 9); // rise along the axis per radian
  });

  it('the screw origin stays on a helix: constant distance from the axis', () => {
    const c = compareSE3(Xs, mk(120 * deg, [-0.9, 0.9, 0.4]));
    const q = c.axisPoint!;
    const r0 = Math.hypot(c.screw[0][0][3] - q[0], c.screw[0][1][3] - q[1]);
    for (const T of c.screw) expect(Math.hypot(T[0][3] - q[0], T[1][3] - q[1])).toBeCloseTo(r0, 9);
  });

  it('pure translation, or a start origin on the axis, makes the two paths identical', () => {
    expect(compareSE3(Xs, mk(0, [-0.9, 0.9, 0.4])).maxPosDiff).toBeLessThan(1e-9);
    expect(compareSE3(Xs, mk(150 * deg, [0, 0, 0.5])).maxPosDiff).toBeLessThan(1e-9);
  });
});
