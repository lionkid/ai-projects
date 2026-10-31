import { describe, expect, it } from 'vitest';
import { planarFK } from '@/mr';
import {
  accelJumps, asTrajectory, buildSpline, c2SplineVelocities, centredVelocities, clampViaTime, findOvershoots,
  globalPolynomial, insertVia, quinticFromCubic, removeVia, segmentSpeeds, trackIK, type Pt2,
} from './viaSpline';
import { SCENARIOS, SCENARIO_IDS } from './scenarios';

const P: Pt2[] = SCENARIOS.demo.points;
const T = SCENARIOS.demo.times;

describe('piecewise cubic through via points', () => {
  it('interpolates the via points with the requested via velocities', () => {
    const custom: Pt2[] = P.map((_, k) => [0.1 * k, -0.2 * k] as Pt2);
    for (const strat of ['zero', 'heuristic', 'custom'] as const) {
      const sp = buildSpline(P, T, strat, custom);
      P.forEach((p, k) => {
        const r = sp.evaluate(T[k]);
        expect(r.q[0]).toBeCloseTo(p[0], 10);
        expect(r.q[1]).toBeCloseTo(p[1], 10);
      });
      // endpoints always at rest
      expect(sp.velocities[0]).toEqual([0, 0]);
      expect(sp.velocities[P.length - 1]).toEqual([0, 0]);
      if (strat === 'custom') expect(sp.velocities[2]).toEqual([0.2, -0.4]);
      if (strat === 'zero') sp.velocities.forEach((v) => expect(v).toEqual([0, 0]));
    }
  });

  it('segment coefficients follow a2 = (3Δ − (2v0 + v1)T)/T², a3 = (−2Δ + (v0 + v1)T)/T³', () => {
    const sp = buildSpline(P, [0, 0.7, 2, 2.6, 4], 'heuristic');
    for (let k = 0; k < P.length - 1; k++) {
      const Tk = sp.times[k + 1] - sp.times[k];
      for (let j = 0; j < 2; j++) {
        const D = P[k + 1][j] - P[k][j], v0 = sp.velocities[k][j], v1 = sp.velocities[k + 1][j];
        const [a0, a1, a2, a3] = sp.coefficients[k][j];
        expect(a0).toBeCloseTo(P[k][j], 12);
        expect(a1).toBeCloseTo(v0, 12);
        expect(a2).toBeCloseTo((3 * D - (2 * v0 + v1) * Tk) / (Tk * Tk), 10);
        expect(a3).toBeCloseTo((-2 * D + (v0 + v1) * Tk) / (Tk * Tk * Tk), 10);
      }
    }
  });

  it('zero via velocities: every segment is rest-to-rest with peak speed 1.5× its average', () => {
    const tr = asTrajectory(buildSpline(P, T, 'zero'));
    for (let k = 0; k < P.length - 1; k++) {
      const { avg, peak, tPeak } = segmentSpeeds(tr, P, k, 400);
      expect(peak / avg).toBeCloseTo(1.5, 3);
      expect(tPeak).toBeCloseTo(0.5 * (T[k] + T[k + 1]), 2);
    }
  });

  it('accelerations jump at via points for the cubic but not for the C² spline or the quintic', () => {
    const sp = buildSpline(P, T, 'heuristic');
    const jumps = accelJumps(sp).slice(1, -1).map((j) => Math.hypot(j.jump[0], j.jump[1]));
    expect(Math.max(...jumps)).toBeGreaterThan(0.1);
    const c2 = buildSpline(P, T, 'custom', c2SplineVelocities(P, T));
    for (const j of accelJumps(c2).slice(1, -1)) expect(Math.hypot(j.jump[0], j.jump[1])).toBeLessThan(1e-9);
    const q5 = quinticFromCubic(sp);
    for (let k = 1; k < P.length - 1; k++) {
      const a = q5.evaluate(T[k] - 1e-7), b = q5.evaluate(T[k] + 1e-7);
      for (let j = 0; j < 2; j++) {
        expect(b.q[j]).toBeCloseTo(a.q[j], 6);
        expect(b.qd[j]).toBeCloseTo(a.qd[j], 5);
        expect(b.qdd[j]).toBeCloseTo(a.qdd[j], 4);
      }
      // quintic keeps the cubic's via velocities
      expect(q5.evaluate(T[k]).qd[0]).toBeCloseTo(sp.velocities[k][0], 9);
    }
    const end = q5.evaluate(T[T.length - 1]);
    expect(Math.hypot(...end.qd)).toBeLessThan(1e-9);
    expect(Math.hypot(...end.qdd)).toBeLessThan(1e-9);
  });

  it('a single global polynomial passes through every point and starts/ends at rest', () => {
    const gp = globalPolynomial(P, T);
    P.forEach((p, k) => {
      const r = gp.evaluate(T[k]);
      expect(r.q[0]).toBeCloseTo(p[0], 9);
      expect(r.q[1]).toBeCloseTo(p[1], 9);
    });
    expect(Math.hypot(...gp.evaluate(0).qd)).toBeLessThan(1e-9);
    expect(Math.hypot(...gp.evaluate(4).qd)).toBeLessThan(1e-9);
  });
});

describe('overshoot', () => {
  it('collinear points with unequal times stay on the line but overshoot past the last point', () => {
    const { points, times } = SCENARIOS.collinear;
    const sp = buildSpline(points, times, 'heuristic');
    const d: Pt2 = [points[2][0] - points[0][0], points[2][1] - points[0][1]];
    for (let i = 0; i <= 200; i++) {
      const q = sp.evaluate((times[2] * i) / 200).q;
      const cross = (q[0] - points[0][0]) * d[1] - (q[1] - points[0][1]) * d[0];
      expect(Math.abs(cross)).toBeLessThan(1e-9);
    }
    const os = findOvershoots(sp);
    expect(os.length).toBeGreaterThan(0);
    expect(os.every((o) => o.seg === 1)).toBe(true);
    // it goes beyond the final point along the line
    const far = os.find((o) => o.coord === 0)!;
    expect(far.q[0]).toBeGreaterThan(points[2][0]);
    // equal durations: no overshoot
    expect(findOvershoots(buildSpline(points, [0, 2.5, 4.5], 'heuristic')).length).toBe(0);
  });

  it('the lesson presets behave as the text says', () => {
    expect(findOvershoots(buildSpline(SCENARIOS.demo.points, SCENARIOS.demo.times, 'heuristic')).length).toBe(0);
    const os = findOvershoots(buildSpline(SCENARIOS.overshoot.points, SCENARIOS.overshoot.times, 'heuristic'));
    expect(os.some((o) => o.seg === 1 && o.coord === 1)).toBe(true);
    for (const id of SCENARIO_IDS) {
      const s = SCENARIOS[id];
      for (const p of s.points) expect(Math.hypot(p[0], p[1])).toBeLessThan(2);
      for (let k = 1; k < s.times.length; k++) expect(s.times[k]).toBeGreaterThan(s.times[k - 1]);
    }
  });
});

describe('2R arm following the path', () => {
  it('reproduces the path with a continuous elbow branch and flags unreachable points', () => {
    const tr = asTrajectory(buildSpline(P, T, 'heuristic'));
    const pts: Pt2[] = Array.from({ length: 301 }, (_, i) => tr.evaluate((4 * i) / 300).q);
    const ik = trackIK(pts);
    ik.thetas.forEach((th, i) => {
      const tip = planarFK([1, 1], th).tip;
      expect(tip[0]).toBeCloseTo(pts[i][0], 8);
      expect(tip[1]).toBeCloseTo(pts[i][1], 8);
      if (i > 0) {
        expect(Math.abs(th[0] - ik.thetas[i - 1][0])).toBeLessThan(0.2);
        expect(Math.abs(th[1] - ik.thetas[i - 1][1])).toBeLessThan(0.2);
      }
    });
    expect(ik.reachable.every(Boolean)).toBe(true);
    const far = trackIK([[1.5, 0], [2.5, 0], [1.5, 0.2]]);
    expect(far.reachable).toEqual([true, false, true]);
  });
});

describe('editing helpers', () => {
  it('keeps times strictly increasing and the first time at zero', () => {
    expect(clampViaTime([0, 1, 2], 1, 5)).toBeCloseTo(1.85, 12);
    expect(clampViaTime([0, 1, 2], 1, -1)).toBeCloseTo(0.15, 12);
    expect(clampViaTime([0, 1, 2], 2, 20)).toBe(8);
    const ins = insertVia(P, T, 1);
    expect(ins.points.length).toBe(P.length + 1);
    expect(ins.times[2]).toBeCloseTo(0.5 * (T[1] + T[2]), 12);
    const rem = removeVia(P, T, 0);
    expect(rem.times[0]).toBe(0);
    expect(rem.points.length).toBe(P.length - 1);
    expect(removeVia(P.slice(0, 3), T.slice(0, 3), 1).points.length).toBe(3);
    const cv = centredVelocities(P, T);
    expect(cv[0]).toEqual([0, 0]);
    expect(cv[2][0]).toBeCloseTo((P[3][0] - P[1][0]) / (T[3] - T[1]), 12);
  });
});
