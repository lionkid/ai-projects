import { describe, expect, it } from 'vitest';
import { InverseDynamics, planar2R, planarFK } from '@/mr';
import {
  ENDPOINTS, PATHS, PATH_IDS, accelBounds, buildTable, checkCurve, coeffsAt, cubicScalingCurve, curveFromDrawing,
  fastestCubic, holdable, jointIntervals, pathCoefficients, refreshLimits, solveGridDP, solveSwitching,
  solveTimeOptimal, timing, torquesAt, trajectorySamples, unholdableIntervals, xRangeTorque,
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

describe('interval diagram, cubic reference and time histories', () => {
  it('per-joint intervals intersect to [L, U] and the binding joint sits on its torque limit at s̈ = U', () => {
    for (const id of PATH_IDS) {
      const t = table(id);
      for (const j of [100, 640, 901, 1500]) {
        const s = j / (2 * t.N);
        for (const sd of [0, 0.8, 1.6]) {
          const x = sd * sd;
          const iv = jointIntervals(t, s, x);
          const [L, U] = accelBounds(t, j, x);
          expect(iv.L).toBeCloseTo(L, 9);
          expect(iv.U).toBeCloseTo(U, 9);
          if (iv.L <= iv.U) {
            const tauU = torquesAt(t, s, x, iv.U);
            const bind = iv.hi[0] < iv.hi[1] ? 0 : 1;
            expect(Math.abs(tauU[bind])).toBeCloseTo(t.tauMax[bind], 6);
            expect(Math.abs(tauU[1 - bind])).toBeLessThanOrEqual(t.tauMax[1 - bind] + 1e-9);
          }
        }
      }
      const co = coeffsAt(t, 0.3);
      expect(co.m[0]).toBeCloseTo(pathCoefficients(PATHS[id], 0.3).m[0], 3);
    }
  });

  it('the cubic reference starts/ends at rest, lasts T, and its fastest feasible T is exact and slower than T*', () => {
    const x = cubicScalingCurve(1000, 2);
    expect(x[0]).toBe(0);
    expect(x[1000]).toBe(0);
    expect(Math.sqrt(x[500])).toBeCloseTo(1.5 / 2, 6);
    expect(timing(x, 1 / 1000).T).toBeCloseTo(2, 2);
    // Largest |τ_i| / τ_i,max of a cubic of duration T, evaluated per cell exactly as fastestCubic does.
    const ratio = (t: PathTable, T: number) => {
      const xc = cubicScalingCurve(t.N, T);
      let r = 0;
      for (let k = 0; k < t.N; k++) {
        const j = 2 * k + 1, a = (xc[k + 1] - xc[k]) / (2 * t.ds), xm = 0.5 * (xc[k] + xc[k + 1]);
        for (let i = 0; i < 2; i++) r = Math.max(r, Math.abs(t.m[i][j] * a + t.c[i][j] * xm + t.g[i][j]) / t.tauMax[i]);
      }
      return r;
    };
    for (const id of PATH_IDS) {
      const t = table(id);
      const fc = fastestCubic(t)!;
      expect(fc).not.toBeNull();
      expect(checkCurve(t, fc.x).bad).toBe(0);
      expect(ratio(t, fc.T)).toBeCloseTo(1, 9);
      expect(ratio(t, fc.T * 0.97)).toBeGreaterThan(1.01);
      expect(fc.T).toBeGreaterThan(solveTimeOptimal(t).T);
    }
    // The corner forces a cubic to crawl through the whole path; bang-bang does not.
    const tc = table('corner');
    expect(fastestCubic(tc)!.T / solveTimeOptimal(tc).T).toBeGreaterThan(1.8);
  });

  it('time histories of the optimum stay within the limits with one joint saturated, and end at s = 1', () => {
    for (const id of PATH_IDS) {
      const t = table(id);
      const sol = solveTimeOptimal(t);
      const h = trajectorySamples(t, sol.x)!;
      expect(h.t[h.t.length - 1]).toBeCloseTo(sol.T, 9);
      expect(h.s[h.s.length - 1]).toBeCloseTo(1, 12);
      let sat = 0;
      for (let k = 0; k < h.t.length; k++) {
        const a = Math.abs(h.tau[0][k]) / t.tauMax[0], b = Math.abs(h.tau[1][k]) / t.tauMax[1];
        expect(Math.max(a, b)).toBeLessThan(1.02);
        if (Math.max(a, b) > 0.97) sat++;
      }
      expect(sat / h.t.length).toBeGreaterThan(0.93);
    }
    // A curve that stops half way never finishes.
    const t = table('line');
    const x = new Float64Array(t.N + 1);
    for (let k = 0; k <= t.N / 2; k++) x[k] = 0.25 * Math.sin((2 * Math.PI * k) / t.N) ** 2;
    expect(trajectorySamples(t, x)).toBeNull();
  });
});
