import { describe, expect, it } from 'vitest';
import { ImpedanceSim, clampTarget, HOME, H, type ImpParams, type Vec2 } from './impedanceSim';

/** Start at HOME, move the target to x0, run T seconds; stats over the last `win` seconds. */
function run(p: Partial<ImpParams>, x0: Vec2, T: number, win = 1.5) {
  const sim = new ImpedanceSim(p);
  sim.x0 = [...HOME];
  sim.reset();
  sim.x0 = x0;
  const N = Math.round(T / H);
  let lost = 0, was = false, fsum = 0, n = 0, fmax = 0, fmin = Infinity;
  for (let k = 0; k < N; k++) {
    sim.step();
    if (!sim.healthy()) break;
    if (sim.t > T - win) {
      if (was && sim.fWall === 0) lost++;
      fsum += sim.fWall; n++; fmax = Math.max(fmax, sim.fWall); fmin = Math.min(fmin, sim.fWall);
    }
    was = sim.fWall > 0;
  }
  const err = Math.hypot(sim.x[0] - x0[0], sim.x[1] - x0[1]);
  return { sim, lost, mean: fsum / Math.max(1, n), fmax, fmin, err };
}

describe('free space', () => {
  it('impedance on a backdrivable arm settles on the target', () => {
    expect(run({ mode: 'impedance', arm: 'direct', K: 300, B: 30, wallOn: false }, [1.4, 0.35], 3).err).toBeLessThan(0.002);
  });

  it('impedance on a geared arm sticks short of the target (friction), admittance does not', () => {
    const imp = run({ mode: 'impedance', arm: 'geared', K: 100, B: 30, wallOn: false }, [1.4, 0.35], 4);
    const adm = run({ mode: 'admittance', arm: 'geared', K: 100, B: 30, M: 2, wallOn: false }, [1.4, 0.35], 4);
    expect(imp.err).toBeGreaterThan(0.02);
    expect(adm.err).toBeLessThan(0.002);
  });
});

describe('wall contact', () => {
  const x0: Vec2 = [1.62, 0.25];

  it('steady impedance contact force = series spring K·k_w/(K + k_w) × target depth', () => {
    const r = run({ mode: 'impedance', arm: 'direct', K: 300, B: 60, kw: 5e3, wallX: 1.5 }, x0, 4);
    const expected = (300 * 5e3) / (300 + 5e3) * 0.12;
    expect(Math.abs(r.mean - expected) / expected).toBeLessThan(0.03);
  });

  it('impedance stays in steady contact even with a very stiff wall', () => {
    const r = run({ mode: 'impedance', arm: 'direct', K: 300, B: 60, kw: 5e4 }, x0, 4);
    expect(r.lost).toBe(0);
    expect(r.fmax - r.fmin).toBeLessThan(0.5);
  });

  it('admittance is stable without extra delay but chatters with a 15 ms sensor delay', () => {
    for (const arm of ['direct', 'geared'] as const) {
      const ok = run({ mode: 'admittance', arm, M: 2, B: 60, K: 300, kw: 5e3, delayMs: 0 }, x0, 4);
      expect(ok.lost).toBe(0);
      expect(ok.fmax - ok.fmin).toBeLessThan(1);
      const bad = run({ mode: 'admittance', arm, M: 2, B: 60, K: 300, kw: 5e3, delayMs: 15 }, x0, 4);
      expect(bad.lost > 0 || bad.fmax - bad.fmin > 10).toBe(true);
    }
  });
});

describe('sensor and helpers', () => {
  it('the wrist reading lags the true external force by (delay + 1) samples', () => {
    const sim = new ImpedanceSim({ mode: 'admittance', wallOn: false, delayMs: 7 });
    sim.push([20, 0], 1);
    let firstSeen = -1;
    for (let k = 0; k < 400 && firstSeen < 0; k++) {
      sim.step();
      if (sim.fMeas[0] > 10) firstSeen = sim.t;
    }
    expect(firstSeen).toBeGreaterThan(0.0075);
    expect(firstSeen).toBeLessThan(0.0105);
  });

  it('clampTarget leaves reachable points untouched and pulls far ones inside', () => {
    const p: Vec2 = [1.3934, 0.3467];
    expect(clampTarget(p)).toEqual(p);
    const q = clampTarget([3, 0]);
    expect(Math.hypot(q[0], q[1])).toBeCloseTo(1.85, 9);
  });
});
