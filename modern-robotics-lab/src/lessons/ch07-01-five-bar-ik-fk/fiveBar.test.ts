import { describe, expect, it } from 'vitest';
import { planarFK } from '@/mr';
import {
  DEFAULT_GEOM, DEFAULT_RPR, RPR_PRESETS, BRANCHES, assembleConfig, basePoint, branchOf, circleIntersections, driveMotors,
  driveTip, elbowPoint, fkAll, fkJacobianFD, fkNearest, grubler, ikAll, ikConfig, legIK, loopResidual, motorSweep,
  rprFKAll, rprIK, sigmaMax, symmetricMarginAngle, tipShift, type FiveBarGeom, type Pt,
} from './fiveBar';

const D = Math.PI / 180;
const g: FiveBarGeom = { ...DEFAULT_GEOM };
const near = (a: Pt, b: Pt, tol = 1e-9) => Math.hypot(a[0] - b[0], a[1] - b[1]) < tol;

describe('Grübler count', () => {
  it('a planar five-bar (N = 5 incl. ground, J = 5 revolute joints) has 2 dof', () => {
    expect(grubler(3, 5, 5, 5)).toBe(2);
    expect(grubler(3, 4, 4, 4)).toBe(1); // four-bar for comparison
  });
});

describe('circle intersection', () => {
  it('two, tangent, none and coincident cases', () => {
    const two = circleIntersections([0, 0], 1, [1, 0], 1);
    expect(two.status).toBe('two');
    expect(two.points[0][1]).toBeGreaterThan(0); // left of c1 -> c2 first
    for (const p of two.points) {
      expect(Math.hypot(p[0], p[1])).toBeCloseTo(1, 12);
      expect(Math.hypot(p[0] - 1, p[1])).toBeCloseTo(1, 12);
    }
    const tan = circleIntersections([0, 0], 1, [2, 0], 1);
    expect(tan.status).toBe('tangent');
    expect(near(tan.points[0], [1, 0], 1e-12)).toBe(true);
    expect(circleIntersections([0, 0], 1, [2.5, 0], 1).status).toBe('none');
    expect(circleIntersections([0, 0], 1, [0.1, 0], 0.5).status).toBe('none'); // one inside the other
    expect(circleIntersections([0, 0], 1, [0, 0], 1).status).toBe('coincident');
  });
});

describe('five-bar IK (two 2R legs) and FK (two circles)', () => {
  it('reference pose: motors (115°, 65°) give a symmetric tip with both elbows outward', () => {
    const r = fkAll(g, 115 * D, 65 * D);
    expect(r.status).toBe('two');
    const up = r.solutions[0];
    expect(up.mode).toBe(1);
    expect(up.P[0]).toBeCloseTo(0, 12);
    expect(up.P[1]).toBeGreaterThan(1.5);
    expect(branchOf(up)).toEqual([1, 1]);
    expect(r.solutions[1].mode).toBe(-1);
    expect(r.solutions[1].P[1]).toBeLessThan(up.P[1]);
  });

  it('every IK branch closes the loop and is reproduced by FK', () => {
    let seed = 3;
    const rnd = () => { seed = (seed * 16807) % 2147483647; return seed / 2147483647; };
    let checked = 0;
    for (let trial = 0; trial < 200; trial++) {
      const P: Pt = [-1.5 + 3 * rnd(), -1.5 + 3.5 * rnd()];
      const sols = ikAll(g, P);
      if (!sols.length) continue;
      expect(sols.length).toBe(4);
      for (const { branch, cfg } of sols) {
        expect(branchOf(cfg)).toEqual(branch);
        const res = loopResidual(g, cfg);
        expect(Math.hypot(res[0], res[1])).toBeLessThan(1e-12);
        expect(near(cfg.P, P, 1e-12)).toBe(true);
        // the left leg as an open 2R chain (planarFK from @/mr) ends at the tip
        const fk = planarFK([g.L1, g.L2], [cfg.th1, cfg.phi1]).tip;
        const A1 = basePoint(g, 0);
        expect(near([A1[0] + fk[0], A1[1] + fk[1]], P, 1e-12)).toBe(true);
        // FK of the motor angles contains this tip
        const f = fkAll(g, cfg.th1, cfg.th2);
        expect(f.solutions.some((s) => near(s.P, P, 1e-9))).toBe(true);
        checked++;
      }
    }
    expect(checked).toBeGreaterThan(100);
  });

  it('a tip outside one leg annulus has no IK; on the boundary the two elbow choices coincide', () => {
    expect(ikConfig(g, [3, 0.5], [1, 1])).toBeNull();
    const A1 = basePoint(g, 0);
    const edge: Pt = [A1[0] + (g.L1 + g.L2) * Math.cos(1), A1[1] + (g.L1 + g.L2) * Math.sin(1)];
    const leg = legIK(g, 0, edge);
    expect(leg.reachable).toBe(true);
    expect(near(leg.sols[0].elbow, leg.sols[1].elbow, 1e-6)).toBe(true);
  });

  it('IK motor angles depend only on the own leg (decoupling)', () => {
    const P: Pt = [0.2, 1.6];
    const a = ikConfig(g, P, [1, 1])!;
    const b = ikConfig(g, P, [1, -1])!;
    expect(a.th1).toBeCloseTo(b.th1, 12); // left leg unchanged when only the right elbow flips
    expect(a.th2).not.toBeCloseTo(b.th2, 3);
  });

  it('assembleConfig recovers passive angles and the tip joint angle', () => {
    const c = fkNearest(g, 115 * D, 65 * D, null, 1)!;
    const again = assembleConfig(g, c.th1, c.th2, c.P);
    expect(again.phi1).toBeCloseTo(c.phi1, 12);
    expect(near(elbowPoint(g, 0, c.th1), c.B1)).toBe(true);
    // distal links nearly perpendicular in the reference pose
    expect(Math.abs(c.psi)).toBeCloseTo(Math.PI / 2, 1);
  });
});

describe('assembly-mode tracking and the tangency boundary', () => {
  it('small motor steps keep the assembly mode', () => {
    let c = fkNearest(g, 115 * D, 65 * D, null, 1)!;
    for (let k = 0; k < 40; k++) {
      c = driveMotors(g, c, c.th1 + 0.5 * D, c.th2 - 0.3 * D).cfg;
      expect(c.mode).toBe(1);
    }
  });

  it('driving the motors apart stops exactly where the distal links become collinear', () => {
    const start = fkNearest(g, 115 * D, 65 * D, null, 1)!;
    const res = driveMotors(g, start, 175 * D, 5 * D);
    expect(res.blocked).toBe(true);
    const D12 = Math.hypot(res.cfg.B2[0] - res.cfg.B1[0], res.cfg.B2[1] - res.cfg.B1[1]);
    expect(D12).toBeCloseTo(g.L2 + g.L4, 6);
    expect(Math.abs(Math.abs(res.cfg.psi) - Math.PI)).toBeLessThan(1e-3); // distal links (almost) collinear
  });

  it('coincident elbows (L2 = L4): the tip keeps its previous direction', () => {
    // with d = 1 and L1 = L3 = 1, th1 = 60°, th2 = 120° put both elbows at (0, sin 60°)
    const prev: Pt = [0.5, 2];
    const c = fkNearest(g, 60 * D, 120 * D, prev)!;
    expect(c).not.toBeNull();
    expect(Math.hypot(c.P[0] - c.B1[0], c.P[1] - c.B1[1])).toBeCloseTo(g.L2, 12);
    const dir = Math.atan2(prev[1] - c.B1[1], prev[0] - c.B1[0]);
    expect(Math.atan2(c.P[1] - c.B1[1], c.P[0] - c.B1[0])).toBeCloseTo(dir, 12);
  });

  it('the tip becomes more sensitive to the motors as the circles approach tangency', () => {
    const sens = [0.6, 0.1, 0.01].map((m) => {
      const t2 = symmetricMarginAngle(g, m)!;
      const c = fkNearest(g, Math.PI - t2, t2, null, 1)!;
      return sigmaMax(fkJacobianFD(g, c)!);
    });
    expect(sens[1]).toBeGreaterThan(sens[0]);
    expect(sens[2]).toBeGreaterThan(2 * sens[1]);
    const t2 = symmetricMarginAngle(g, 0)!;
    expect(t2).toBeCloseTo(Math.acos(0.8), 12);
  });

  it('tipShift follows a 1° motor step and refuses steps past the boundary', () => {
    const c = fkNearest(g, 115 * D, 65 * D, null, 1)!;
    const s = tipShift(g, c, 0, D)!;
    expect(Math.hypot(s[0], s[1])).toBeGreaterThan(0.005);
    expect(Math.hypot(s[0], s[1])).toBeLessThan(0.05);
    const t2 = symmetricMarginAngle(g, 1e-4)!;
    const edge = fkNearest(g, Math.PI - t2, t2, null, 1)!;
    expect(tipShift(g, edge, 0, 2 * D)).toBeNull(); // turning the left motor outwards cannot be assembled
  });

  it('driveTip stops on the workspace boundary (a leg fully stretched)', () => {
    const c = ikConfig(g, [0, 1.6], [1, 1])!;
    const res = driveTip(g, c, [0, 4], [1, 1]);
    expect(res.blocked).toBe(true);
    const A1 = basePoint(g, 0);
    expect(Math.hypot(res.cfg.P[0] - A1[0], res.cfg.P[1] - A1[1])).toBeCloseTo(g.L1 + g.L2, 6);
  });

  it('motorSweep with one motor locked traces a 1-dof path bounded by tangency', () => {
    const c = fkNearest(g, 115 * D, 65 * D, null, 1)!;
    const sw = motorSweep(g, c, 1, 0.02);
    expect(sw.configs.length).toBe(sw.angles.length);
    expect(sw.blockedLo || sw.blockedHi).toBe(true);
    for (const q of sw.configs) expect(q.th1).toBeCloseTo(c.th1, 12);
    for (let i = 1; i < sw.angles.length; i++) expect(sw.angles[i]).toBeGreaterThan(sw.angles[i - 1]);
  });

  it('all four branches are listed in a fixed order', () => {
    expect(BRANCHES.map((b) => b.join(''))).toEqual(['11', '1-1', '-11', '-1-1']);
  });
});

describe('3xRPR platform', () => {
  it('IK lengths reproduce the platform joints and FK finds the original pose', () => {
    for (const pose of Object.values(RPR_PRESETS)) {
      const L = rprIK(DEFAULT_RPR, pose);
      const sols = rprFKAll(DEFAULT_RPR, L);
      expect(sols.some((s) => Math.hypot(s.x - pose.x, s.y - pose.y) < 1e-6 && Math.abs(s.theta - pose.theta) < 1e-6)).toBe(true);
      for (const s of sols) {
        const L2 = rprIK(DEFAULT_RPR, s);
        for (let i = 0; i < 3; i++) expect(L2[i]).toBeCloseTo(L[i], 8);
      }
    }
  });

  it('the presets have 6, 4 and 2 real solutions', () => {
    expect(rprFKAll(DEFAULT_RPR, rprIK(DEFAULT_RPR, RPR_PRESETS.six)).length).toBe(6);
    expect(rprFKAll(DEFAULT_RPR, rprIK(DEFAULT_RPR, RPR_PRESETS.four)).length).toBe(4);
    expect(rprFKAll(DEFAULT_RPR, rprIK(DEFAULT_RPR, RPR_PRESETS.two)).length).toBe(2);
  });

  it('random poses: the original pose is always found and the count is even', () => {
    let seed = 11;
    const rnd = () => { seed = (seed * 16807) % 2147483647; return seed / 2147483647; };
    for (let i = 0; i < 60; i++) {
      const pose = { x: -0.5 + rnd(), y: 0.7 + 0.7 * rnd(), theta: (-50 + 100 * rnd()) * D };
      const sols = rprFKAll(DEFAULT_RPR, rprIK(DEFAULT_RPR, pose));
      expect(sols.some((s) => Math.hypot(s.x - pose.x, s.y - pose.y) < 1e-6)).toBe(true);
      expect(sols.length % 2).toBe(0);
    }
  });
});
