import { describe, expect, it } from 'vitest';
import { InverseDynamics, planar2R, planarFK } from '@/mr';
import {
  ENDPOINTS, PATHS, PATH_IDS, accelBounds, buildTable, checkCurve, curveFromDrawing, holdable, pathCoefficients,
  refreshLimits, solveGridDP, solveSwitching, solveTimeOptimal, timing, unholdableIntervals, xRangeTorque,
  type PathTable,
} from './phasePlane';

const DEFAULT: [number, number] = [45, 18];
const tables = new Map<string, PathTable>();
const table = (id: (typeof PATH_IDS)[number], tau = DEFAULT, cap: number | null = null): PathTable => {
  const key = id;
  let t = tables.get(key);
  if (!t) { t = buildTable(PATHS[id], { tauMax: tau, dthetaMax: cap }); tables.set(key, t); }
  refreshLimits(t, { tauMax: tau, dthetaMax: cap });
  return t;
};

describe('path dynamics m(s), c(s), g(s)', () => {
  it('matches Newton–Euler InverseDynamics on the planar2R model', () => {
    const { Mlist, Glist, Slist } = planar2R;
    for (const id of PATH_IDS) {
      for (const s of [0.05, 0.3, 0.45, 0.62, 0.9]) {
        const path = PATHS[id];
        const co = pathCoefficients(path, s);
        const th = path.theta(s), d1 = path.d1(s), d2 = path.d2(s);
        for (const [sd, sdd] of [[0.7, 1.3], [1.9, -2.2], [0, 0.5]]) {
          const dth = [d1[0] * sd, d1[1] * sd];
          const ddth = [d1[0] * sdd + d2[0] * sd * sd, d1[1] * sdd + d2[1] * sd * sd];
          const tauNE = InverseDynamics(th, dth, ddth, [0, -9.81, 0], [0, 0, 0, 0, 0, 0], Mlist!, Glist!, Slist);
          for (let i = 0; i < 2; i++) {
            expect(co.m[i] * sdd + co.c[i] * sd * sd + co.g[i]).toBeCloseTo(tauNE[i], 5);
          }
        }
      }
    }
  });

  it('line and corner share their endpoints; the Cartesian path keeps the tip on x = 1.3', () => {
    for (const id of ['line', 'corner'] as const) {
      const a = PATHS[id].theta(0), b = PATHS[id].theta(1);
      expect(a[0]).toBeCloseTo(ENDPOINTS[0][0], 9); expect(a[1]).toBeCloseTo(ENDPOINTS[0][1], 9);
      expect(b[0]).toBeCloseTo(ENDPOINTS[1][0], 9); expect(b[1]).toBeCloseTo(ENDPOINTS[1][1], 9);
    }
    for (let k = 0; k <= 20; k++) {
      const tip = planarFK([1, 1], PATHS.cartesian.theta(k / 20)).tip;
      expect(tip[0]).toBeCloseTo(1.3, 9);
      expect(tip[1]).toBeCloseTo(-1 + (2 * k) / 20, 9);
    }
  });

  it('derivatives d1, d2 agree with finite differences of θ(s)', () => {
    const h = 1e-5;
    for (const id of PATH_IDS) {
      for (const s of [0.2, 0.44, 0.47, 0.8]) {
        const p = PATHS[id];
        const fp = p.theta(s + h), fm = p.theta(s - h), f0 = p.theta(s);
        for (let i = 0; i < 2; i++) {
          expect(p.d1(s)[i]).toBeCloseTo((fp[i] - fm[i]) / (2 * h), 4);
          expect(p.d2(s)[i]).toBeCloseTo((fp[i] - 2 * f0[i] + fm[i]) / (h * h), id === 'cartesian' ? 0 : 1);
        }
      }
    }
  });

  it('has no zero-inertia points (|m_i| stays away from 0) on any path', () => {
    for (const id of PATH_IDS) {
      const t = table(id);
      for (let j = 0; j < t.m[0].length; j++) {
        expect(Math.abs(t.m[0][j])).toBeGreaterThan(0.4);
        expect(Math.abs(t.m[1][j])).toBeGreaterThan(0.4);
      }
    }
  });
});

describe('feasible acceleration interval and the velocity limit curve', () => {
  it('closed-form x-range agrees with bisection on L(x) ≤ U(x)', () => {
    for (const id of PATH_IDS) {
      const t = table(id);
      for (const j of [37, 400, 900, 1333, 1990]) {
        const [lo, hi] = xRangeTorque(t, j);
        expect(lo).toBeLessThanOrEqual(1e-9);
        // bisection
        let a = 0, b = 400;
        const ok = (x: number) => { const [L, U] = accelBounds(t, j, x); return L <= U; };
        if (ok(b)) { expect(hi).toBeGreaterThan(b); continue; }
        for (let it = 0; it < 80; it++) { const m = 0.5 * (a + b); if (ok(m)) a = m; else b = m; }
        expect(hi).toBeCloseTo(a, 6);
      }
    }
  });

  it('a lower torque limit lowers the whole limit curve', () => {
    const t = table('corner');
    const hiA = Float64Array.from(t.xHi);
    refreshLimits(t, { tauMax: [35, 14] });
    for (let j = 0; j < hiA.length; j += 50) expect(t.xHi[j]).toBeLessThan(hiA[j]);
  });
});

describe('time-optimal solutions', () => {
  it('the switching algorithm agrees with the grid DP solver and has the expected switch structure', () => {
    const expected: Record<string, number> = { line: 1, corner: 3, cartesian: 1 };
    for (const id of PATH_IDS) {
      for (const tau of [DEFAULT, [35, 14], [60, 25]] as [number, number][]) {
        const t = table(id, tau);
        const sw = solveSwitching(t);
        const dp = solveGridDP(t);
        expect(sw.ok, `${id} ${tau}: ${sw.message}`).toBe(true);
        expect(dp.ok).toBe(true);
        expect(Math.abs(sw.T - dp.T) / dp.T).toBeLessThan(0.006);
        if (tau === DEFAULT) expect(sw.switches.length).toBe(expected[id]);
      }
    }
  });

  it('respects the torque limits and keeps at least one joint saturated (bang-bang)', () => {
    for (const id of PATH_IDS) {
      const t = table(id);
      const sol = solveSwitching(t);
      let saturated = 0;
      const n = t.N;
      for (let k = 0; k < n; k++) {
        const a = (sol.x[k + 1] - sol.x[k]) / (2 * t.ds);
        const xm = 0.5 * (sol.x[k] + sol.x[k + 1]);
        const j = 2 * k + 1;
        let any = false;
        for (let i = 0; i < 2; i++) {
          const tau = t.m[i][j] * a + t.c[i][j] * xm + t.g[i][j];
          expect(Math.abs(tau)).toBeLessThan(t.tauMax[i] * 1.02 + 0.05);
          if (Math.abs(tau) > t.tauMax[i] * 0.97) any = true;
        }
        if (any) saturated++;
      }
      expect(saturated / n).toBeGreaterThan(0.95);
      const chk = checkCurve(t, sol.x);
      expect(chk.bad).toBeLessThanOrEqual(3);
    }
  });

  it('is slower with smaller torque limits', () => {
    for (const id of PATH_IDS) {
      const T = [[60, 25], [45, 18], [35, 14], [28, 11]].map((tau) => solveTimeOptimal(table(id, tau as [number, number])).T);
      for (let i = 1; i < T.length; i++) expect(T[i]).toBeGreaterThan(T[i - 1]);
    }
  });

  it('detects infeasible paths and allows coasting through stretches that cannot be held at rest', () => {
    const corner = table('corner', [20, 10]);
    const solC = solveTimeOptimal(corner);
    expect(solC.ok).toBe(false);
    expect(solC.infeasible).toBe(true);
    expect(solveGridDP(corner).ok).toBe(false);
    const line = table('line', [22, 10]);
    expect(unholdableIntervals(line).length).toBeGreaterThan(0);
    expect(holdable(line, 0)).toBe(true);
    const solL = solveTimeOptimal(line);
    const dpL = solveGridDP(line);
    expect(dpL.ok).toBe(solL.ok);
    if (solL.ok) expect(Math.abs(solL.T - dpL.T) / dpL.T).toBeLessThan(0.01);
  });

  it('with a joint speed limit the solution stays under the ceiling and is not faster', () => {
    for (const id of PATH_IDS) {
      const free = solveTimeOptimal(table(id)).T;
      const t = table(id, DEFAULT, 3);
      const sol = solveTimeOptimal(t);
      expect(sol.ok).toBe(true);
      expect(sol.T).toBeGreaterThanOrEqual(free * 0.999);
      for (let k = 0; k <= t.N; k++) {
        const sd = Math.sqrt(sol.x[k]);
        for (let i = 0; i < 2; i++) expect(Math.abs(t.d1[i][2 * k]) * sd).toBeLessThan(3 * 1.001);
      }
    }
  });
});

describe('curves as trajectories', () => {
  it('timing reproduces s(0) = 0, s(T) = 1 and integrates ṡ consistently', () => {
    const t = table('corner');
    const sol = solveTimeOptimal(t);
    const tm = timing(sol.x, t.ds);
    expect(tm.T).toBeCloseTo(sol.T, 9);
    expect(tm.at(0).s).toBe(0);
    expect(tm.at(tm.T).s).toBeCloseTo(1, 6);
    let prev = 0, integral = 0;
    const steps = 4000;
    for (let i = 1; i <= steps; i++) {
      const st = tm.at((i / steps) * tm.T);
      expect(st.s).toBeGreaterThanOrEqual(prev - 1e-12);
      prev = st.s;
      integral += st.sd * (tm.T / steps);
    }
    expect(integral).toBeCloseTo(1, 2);
  });

  it('freehand curves start and end at rest; a curve above the optimum is flagged', () => {
    const t = table('line');
    const sol = solveTimeOptimal(t);
    const drawn: [number, number][] = [];
    for (let k = 1; k < 20; k++) drawn.push([k / 20, 0.5]);
    const x = curveFromDrawing(t.N, drawn);
    expect(x[0]).toBe(0);
    expect(x[t.N]).toBe(0);
    expect(x[500]).toBeCloseTo(0.25, 9);
    const faster = Float64Array.from(sol.x, (v) => v * 1.15);
    expect(checkCurve(t, faster).bad).toBeGreaterThan(50);
    expect(checkCurve(t, x).T).toBeGreaterThan(sol.T);
  });
});
