import { describe, expect, it } from 'vitest';
import { eye, matMul, type Vec } from './linalg';
import { RotX, RotZ } from './so3';
import { RpToTrans, TestIfSE3, TransToRp } from './se3';
import {
  CartesianTrajectory,
  CubicTimeScaling,
  CubicTimeScalingDerivs,
  JointTrajectory,
  QuinticTimeScaling,
  QuinticTimeScalingDerivs,
  ScrewTrajectory,
  TrapezoidalTimeScaling,
  trapezoidParams,
  trapezoidSample,
  viaPointCubic,
} from './trajectory';
import { expectClose } from './testUtils';

describe('polynomial time scalings', () => {
  it('satisfy the boundary conditions', () => {
    const Tf = 2.5;
    for (const fn of [CubicTimeScalingDerivs, QuinticTimeScalingDerivs]) {
      const a = fn(Tf, 0);
      const b = fn(Tf, Tf);
      expect(a.s).toBeCloseTo(0, 14);
      expect(b.s).toBeCloseTo(1, 14);
      expect(a.sdot).toBeCloseTo(0, 14);
      expect(b.sdot).toBeCloseTo(0, 14);
      expect(fn(Tf, Tf / 2).s).toBeCloseTo(0.5, 14);
      expect(fn(Tf, -1).s).toBe(0);
      expect(fn(Tf, Tf + 1).s).toBe(1);
    }
    const q0 = QuinticTimeScalingDerivs(Tf, 0);
    const q1 = QuinticTimeScalingDerivs(Tf, Tf);
    expect(q0.sddot).toBeCloseTo(0, 14);
    expect(q1.sddot).toBeCloseTo(0, 14);
    expect(CubicTimeScaling(Tf, Tf / 2)).toBeCloseTo(0.5, 14);
    expect(QuinticTimeScaling(Tf, Tf / 2)).toBeCloseTo(0.5, 14);
  });

  it('derivatives agree with finite differences and s is monotone', () => {
    const Tf = 1.7;
    const h = 1e-5;
    for (const [fn, plain] of [
      [CubicTimeScalingDerivs, CubicTimeScaling],
      [QuinticTimeScalingDerivs, QuinticTimeScaling],
    ] as const) {
      let prev = 0;
      for (let k = 1; k < 20; k++) {
        const t = (Tf * k) / 20;
        const d = fn(Tf, t);
        expect(d.s).toBeGreaterThan(prev);
        prev = d.s;
        expect(d.sdot).toBeCloseTo((plain(Tf, t + h) - plain(Tf, t - h)) / (2 * h), 6);
        expect(d.sddot).toBeCloseTo((fn(Tf, t + h).sdot - fn(Tf, t - h).sdot) / (2 * h), 5);
      }
    }
  });
});

describe('trapezoidal time scaling', () => {
  it('completes parameters and validates them', () => {
    const P = trapezoidParams({ Tf: 1, v: 1.5 });
    expect(P.valid).toBe(true);
    expect(P.a).toBeCloseTo(4.5, 12);
    expect(P.tAccel).toBeCloseTo(1 / 3, 12);
    const Q = trapezoidParams({ Tf: 1, a: 4.5 });
    expect(Q.valid).toBe(true);
    expect(Q.v).toBeCloseTo(1.5, 12);
    const R = trapezoidParams({ v: 1.5, a: 4.5 });
    expect(R.valid).toBe(true);
    expect(R.Tf).toBeCloseTo(1, 12);
    expect(trapezoidParams({ Tf: 1, v: 0.9 }).valid).toBe(false);
    expect(trapezoidParams({ Tf: 1, v: 2.5 }).valid).toBe(false);
    expect(trapezoidParams({ Tf: 1, v: 2 }).valid).toBe(true); // triangular limit
    expect(trapezoidParams({ Tf: 1, a: 3 }).valid).toBe(false);
    expect(trapezoidParams({ Tf: 1, a: 4 }).valid).toBe(true);
    expect(trapezoidParams({ v: 1, a: 0.5 }).valid).toBe(false);
    expect(trapezoidParams({ Tf: 1 }).valid).toBe(false);
    expect(() => TrapezoidalTimeScaling(0.5, 1, { v: 0.5 })).toThrow();
  });

  it('starts at 0, ends at 1, is continuous and its velocity integrates to 1', () => {
    const Tf = 1.3;
    for (const spec of [{ v: 1.5 }, { a: 6 }, { v: 2 / Tf }] as const) {
      const P = trapezoidParams({ Tf, ...spec });
      expect(P.valid).toBe(true);
      expect(TrapezoidalTimeScaling(0, Tf, spec).s).toBe(0);
      expect(TrapezoidalTimeScaling(Tf, Tf, spec).s).toBe(1);
      // Continuity at the phase boundaries.
      for (const tb of [P.tAccel, Tf - P.tAccel]) {
        const a = trapezoidSample(tb - 1e-9, P);
        const b = trapezoidSample(tb + 1e-9, P);
        expect(Math.abs(a.s - b.s)).toBeLessThan(1e-7);
        expect(Math.abs(a.sdot - b.sdot)).toBeLessThan(1e-7);
      }
      // Trapezoid area = 1 (composite trapezoid rule on a fine grid).
      const n = 20000;
      let area = 0;
      for (let k = 0; k < n; k++) {
        const t0 = (Tf * k) / n;
        const t1 = (Tf * (k + 1)) / n;
        area += 0.5 * (trapezoidSample(t0, P).sdot + trapezoidSample(t1, P).sdot) * (t1 - t0);
      }
      expect(area).toBeCloseTo(1, 6);
      // Peak velocity equals v and acceleration magnitude equals a.
      expect(trapezoidSample(Tf / 2, P).sdot).toBeCloseTo(P.v, 12);
      expect(trapezoidSample(P.tAccel / 2, P).sddot).toBeCloseTo(P.a, 12);
      expect(trapezoidSample(Tf - P.tAccel / 2, P).sddot).toBeCloseTo(-P.a, 12);
    }
  });
});

describe('point-to-point trajectories', () => {
  const Xstart = RpToTrans(eye(3), [1, 0, 0]);
  const Xend = RpToTrans(matMul(RotZ(Math.PI / 2), RotX(0.3)), [0, 1, 1]);

  it('JointTrajectory interpolates between the endpoints', () => {
    const traj = JointTrajectory([0, 1, -1], [1, 1, 1], 2, 11, 5);
    expect(traj.length).toBe(11);
    expectClose(traj[0], [0, 1, -1], 1e-15);
    expectClose(traj[10], [1, 1, 1], 1e-15);
    expectClose(traj[5], [0.5, 1, 0], 1e-12);
    for (let k = 1; k < 11; k++) expect(traj[k][0]).toBeGreaterThanOrEqual(traj[k - 1][0]);
    const custom = JointTrajectory([0], [2], 1, 3, (Tf, t) => t / Tf);
    expectClose(custom.map((q) => q[0]), [0, 1, 2], 1e-15);
  });

  it('ScrewTrajectory stays in SE(3) and hits both endpoints', () => {
    const traj = ScrewTrajectory(Xstart, Xend, 1, 7, 3);
    expect(traj.length).toBe(7);
    expectClose(traj[0], Xstart, 1e-12);
    expectClose(traj[6], Xend, 1e-10);
    for (const X of traj) expect(TestIfSE3(X)).toBe(true);
  });

  it('CartesianTrajectory moves the origin on a straight line', () => {
    const traj = CartesianTrajectory(Xstart, Xend, 1, 5, 3);
    expectClose(traj[0], Xstart, 1e-12);
    expectClose(traj[4], Xend, 1e-10);
    const [, pMid] = TransToRp(traj[2]);
    expectClose(pMid, [0.5, 0.5, 0.5], 1e-12);
    const [, pScrewMid] = TransToRp(ScrewTrajectory(Xstart, Xend, 1, 5, 3)[2]);
    // A screw path generally curves, so the midpoints differ.
    expect(Math.hypot(pScrewMid[0] - 0.5, pScrewMid[1] - 0.5, pScrewMid[2] - 0.5)).toBeGreaterThan(1e-3);
  });
});

describe('viaPointCubic', () => {
  const points: Vec[] = [[0, 0], [1, 2], [3, 1], [2, 0]];
  const times = [0, 1, 2.5, 4];

  it('passes through the via points with zero velocities by default', () => {
    const sp = viaPointCubic(points, times);
    for (let k = 0; k < points.length; k++) {
      const e = sp.evaluate(times[k]);
      expectClose(e.q, points[k], 1e-12);
      expectClose(e.qdot, [0, 0], 1e-12);
    }
    expect(sp.sample(5).length).toBe(5);
    expectClose(sp.evaluate(-5).q, points[0], 0);
    expectClose(sp.evaluate(99).q, points[3], 0);
  });

  it('honours explicit velocities and keeps q, qdot continuous at the knots', () => {
    const vel: Vec[] = [[0, 0], [1, -1], [0.5, 0.5], [0, 0]];
    const sp = viaPointCubic(points, times, vel);
    for (let k = 0; k < points.length; k++) {
      expectClose(sp.evaluate(times[k]).qdot, vel[k], 1e-12);
    }
    for (let k = 1; k < points.length - 1; k++) {
      const before = sp.evaluate(times[k] - 1e-7);
      const after = sp.evaluate(times[k] + 1e-7);
      expectClose(before.q, after.q, 1e-5);
      expectClose(before.qdot, after.qdot, 1e-5);
    }
  });

  it('heuristic velocities average the neighbouring slopes or vanish at reversals', () => {
    const sp = viaPointCubic(points, times, 'heuristic');
    // joint 0 at via point 1: slopes 1 and (3-1)/1.5 -> average
    expect(sp.velocities[1][0]).toBeCloseTo(0.5 * (1 + 2 / 1.5), 12);
    // joint 1 at via point 1: slopes 2 and -1/1.5 -> sign change -> 0
    expect(sp.velocities[1][1]).toBe(0);
    expectClose(sp.velocities[0], [0, 0], 0);
    expectClose(sp.velocities[3], [0, 0], 0);
  });

  it('rejects malformed input', () => {
    expect(() => viaPointCubic([[0]], [0])).toThrow();
    expect(() => viaPointCubic([[0], [1]], [1, 1])).toThrow();
    expect(() => viaPointCubic([[0], [1]], [0, 1], [[0]])).toThrow();
  });
});
