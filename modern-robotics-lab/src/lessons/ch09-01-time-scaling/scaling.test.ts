import { describe, expect, it } from 'vitest';
import { CubicTimeScaling, QuinticTimeScaling } from '@/mr';
import {
  cubicProfile, quinticProfile, trapezoidProfile, trapezoidMinTime, scurveForT, scurveMinTime, scurveProfile,
  makeProfile, sampleTimes, timeAtS, flexResponse, minTime, type Profile, type Limits,
} from './scaling';

const LIM: Limits = { vmax: 0.8, amax: 0.8, jmax: 4 };

/** Max over a fine grid of |numerical derivative − reported derivative|. */
function derivativeMismatch(p: Profile): { sd: number; sdd: number } {
  const h = 1e-5 * p.T;
  let e1 = 0, e2 = 0;
  for (let i = 1; i < 400; i++) {
    const t = (p.T * i) / 400;
    if (p.breaks.some((b) => Math.abs(b - t) < 3 * h)) continue;
    const a = p.sample(t - h), b = p.sample(t + h), c = p.sample(t);
    e1 = Math.max(e1, Math.abs((b.s - a.s) / (2 * h) - c.sd));
    e2 = Math.max(e2, Math.abs((b.sd - a.sd) / (2 * h) - c.sdd));
  }
  return { sd: e1, sdd: e2 };
}

function boundary(p: Profile) {
  const a = p.sample(0), b = p.sample(p.T);
  return { s0: a.s, sd0: a.sd, s1: b.s, sd1: b.sd };
}

describe('polynomial scalings agree with @/mr', () => {
  it('cubic and quintic match the library and their peaks', () => {
    const T = 2;
    const c = cubicProfile(T), q = quinticProfile(T);
    for (const t of [0.1, 0.5, 1, 1.7]) {
      expect(c.sample(t).s).toBeCloseTo(CubicTimeScaling(T, t), 12);
      expect(q.sample(t).s).toBeCloseTo(QuinticTimeScaling(T, t), 12);
    }
    expect(c.sample(1).sd).toBeCloseTo(c.peak.sd, 12);
    expect(q.sample(1).sd).toBeCloseTo(q.peak.sd, 12);
    // Cubic s̈ jumps to 6/T² at the start; quintic starts at zero acceleration.
    expect(c.sample(0).sdd).toBeCloseTo(6 / (T * T), 12);
    expect(q.sample(0).sdd).toBeCloseTo(0, 12);
    // Quintic peak acceleration is slightly LOWER than the cubic's (5.77 vs 6) / T².
    expect(q.peak.sdd).toBeLessThan(c.peak.sdd);
    const d = derivativeMismatch(q);
    expect(d.sd).toBeLessThan(1e-6);
    expect(d.sdd).toBeLessThan(1e-5);
  });

  it('the cubic jerk is the slope of its acceleration', () => {
    const c = cubicProfile(1.5);
    const h = 1e-6;
    const t = 0.6;
    expect((c.sample(t + h).sdd - c.sample(t - h).sdd) / (2 * h)).toBeCloseTo(c.sample(t).jerk, 5);
  });
});

describe('trapezoid', () => {
  it('minimum time follows T = (a + v²)/(va) when v²/a ≤ 1 and 2/√a otherwise', () => {
    expect(trapezoidMinTime(1, 2).T).toBeCloseTo(1.5, 12);
    expect(trapezoidMinTime(1, 2).triangle).toBe(false);
    const tri = trapezoidMinTime(1.2, 0.8);
    expect(tri.triangle).toBe(true);
    expect(tri.T).toBeCloseTo(2 / Math.sqrt(0.8), 12);
    expect(tri.vPeak).toBeCloseTo(Math.sqrt(0.8), 12);
  });

  it('a feasible request uses a = a_max, finishes exactly at T and respects v_max', () => {
    const p = trapezoidProfile(2.5, LIM);
    expect(p.feasible).toBe(true);
    expect(p.T).toBe(2.5);
    const b = boundary(p);
    expect(b.s0).toBe(0);
    expect(b.s1).toBeCloseTo(1, 12);
    expect(p.info.a).toBeCloseTo(LIM.amax, 12);
    expect(p.info.v!).toBeLessThanOrEqual(LIM.vmax);
    expect(p.sample(p.T / 2).sd).toBeCloseTo(p.info.v!, 12);
    expect(p.accelJumps).toHaveLength(4);
  });

  it('too short a T is reported infeasible and replaced by the minimum-time profile', () => {
    const Tmin = trapezoidMinTime(LIM.vmax, LIM.amax).T; // 1/0.8 + 0.8/0.8 = 2.25 s
    expect(Tmin).toBeCloseTo(2.25, 12);
    const p = trapezoidProfile(2.0, LIM);
    expect(p.feasible).toBe(false);
    expect(p.T).toBeCloseTo(Tmin, 9);
    expect(p.info.v!).toBeCloseTo(LIM.vmax, 9);
    expect(trapezoidProfile(Tmin * (1 + 1e-6), LIM).feasible).toBe(true);
  });

  it('degenerates into a triangle at the minimum time when v_max is never reached', () => {
    const lim = { vmax: 1.2, amax: 0.8, jmax: 4 };
    const Tmin = trapezoidMinTime(lim.vmax, lim.amax).T;
    const p = trapezoidProfile(Tmin, lim);
    expect(p.feasible).toBe(true);
    expect(p.info.triangle).toBe(true);
    expect(p.accelJumps).toHaveLength(3);
    expect(p.peak.sd).toBeCloseTo(Math.sqrt(0.8), 6);
  });
});

describe('S-curve', () => {
  it('existence threshold: no profile below (32/J)^(1/3)', () => {
    const J = 4;
    const Tb = Math.cbrt(32 / J);
    expect(scurveForT(Tb * 0.999, 100, J)).toBeNull();
    const p = scurveForT(Tb * 1.0001, 100, J)!;
    expect(p).not.toBeNull();
    expect(p.saturated).toBe(false);
    expect(p.tv).toBeLessThan(1e-3);
  });

  for (const [T, lim] of [[2.5, LIM], [3.5, LIM], [6, LIM], [2.5, { vmax: 2, amax: 3, jmax: 30 }]] as [number, Limits][]) {
    it(`satisfies boundary conditions and limits (T = ${T}, a = ${lim.amax}, j = ${lim.jmax})`, () => {
      const p = scurveProfile(T, lim);
      expect(p.feasible).toBe(true);
      const b = boundary(p);
      expect(b.s0).toBe(0);
      expect(b.sd0).toBe(0);
      // Evaluate just inside the end so that the closed-form segments (not the clamp) are tested.
      const end = p.sample(p.T * (1 - 1e-12));
      expect(end.s).toBeCloseTo(1, 9);
      expect(end.sd).toBeCloseTo(0, 9);
      expect(end.sdd).toBeCloseTo(0, 9);
      let prev = -1;
      for (const t of sampleTimes(p, 0, p.T, 600)) {
        const r = p.sample(t);
        expect(r.s).toBeGreaterThanOrEqual(prev - 1e-12);
        prev = r.s;
        expect(r.sd).toBeLessThanOrEqual(lim.vmax * (1 + 1e-9));
        expect(Math.abs(r.sdd)).toBeLessThanOrEqual(lim.amax * (1 + 1e-9));
        expect(Math.abs(r.jerk)).toBeLessThanOrEqual(lim.jmax * (1 + 1e-9));
      }
      const d = derivativeMismatch(p);
      expect(d.sd).toBeLessThan(1e-6);
      expect(d.sdd).toBeLessThan(1e-4);
    });
  }

  it('feasibility is monotone in T and the minimum time is tight', () => {
    const Tmin = scurveMinTime(LIM);
    expect(scurveProfile(Tmin * 0.999, LIM).feasible).toBe(false);
    expect(scurveProfile(Tmin * 1.001, LIM).feasible).toBe(true);
    let seenOk = false;
    for (let T = 0.5; T < 8; T += 0.01) {
      const ok = scurveProfile(T, LIM).feasible;
      if (seenOk) expect(ok).toBe(true);
      seenOk = seenOk || ok;
    }
    expect(seenOk).toBe(true);
  });

  it('approaches the trapezoid as the jerk limit grows', () => {
    const lim = { vmax: 0.8, amax: 0.8, jmax: 1e5 };
    expect(scurveMinTime(lim)).toBeCloseTo(trapezoidMinTime(0.8, 0.8).T, 3);
  });
});

describe('minimum times under the same limits', () => {
  it('polynomials just touch the binding limit at their minimum time', () => {
    for (const k of ['cubic', 'quintic'] as const) {
      const T = minTime(k, LIM);
      const p = makeProfile(k, T, LIM);
      const sdOk = p.peak.sd <= LIM.vmax * (1 + 1e-9);
      const sddOk = p.peak.sdd <= LIM.amax * (1 + 1e-9);
      expect(sdOk && sddOk).toBe(true);
      const tight = Math.abs(p.peak.sd - LIM.vmax) < 1e-9 || Math.abs(p.peak.sdd - LIM.amax) < 1e-9;
      expect(tight).toBe(true);
    }
  });

  it('the trapezoid is the fastest of the four under the same v and a limits', () => {
    const Ttrap = minTime('trapezoid', LIM);
    for (const k of ['cubic', 'quintic', 'scurve'] as const) expect(minTime(k, LIM)).toBeGreaterThan(Ttrap);
  });
});

describe('helpers', () => {
  it('timeAtS inverts s(t)', () => {
    for (const k of ['cubic', 'quintic', 'trapezoid', 'scurve'] as const) {
      const p = makeProfile(k, 2.5, LIM);
      for (const s of [0.1, 0.5, 0.93]) expect(p.sample(timeAtS(p, s)).s).toBeCloseTo(s, 9);
    }
  });

  it('flexible load: a very slow move leaves (almost) no residual vibration', () => {
    const slow = flexResponse(quinticProfile(20), 2, 0.02, 22);
    expect(slow.residual).toBeLessThan(1e-4);
    const fast = flexResponse(cubicProfile(1), 2, 0.02, 3);
    expect(fast.residual).toBeGreaterThan(10 * slow.residual);
  });

  it('flexible load: with the lesson defaults, smooth profiles ring less over a whole range of T', () => {
    // Lesson defaults: v_max 0.8, a_max 1.0, j_max 3, f_n = 3 Hz, ζ = 0.02 (jerk ramp ≈ one natural period).
    const lim = { vmax: 0.8, amax: 1.0, jmax: 3 };
    const fn = 3, z = 0.02;
    const r = (k: 'cubic' | 'quintic' | 'trapezoid' | 'scurve', T: number) => {
      const p = makeProfile(k, T, lim);
      return flexResponse(p, fn, z, p.T + 1.5).residual;
    };
    for (let T = 2.4; T <= 4.0 + 1e-9; T += 0.1) {
      expect(r('quintic', T)).toBeLessThan(r('cubic', T));
      expect(r('scurve', T)).toBeLessThan(0.25 * r('trapezoid', T));
    }
    // Raising the jerk limit makes the S-curve ring more (it tends towards the trapezoid's sharp corners).
    const s = (j: number) => { const p = makeProfile('scurve', 2.5, { ...lim, jmax: j }); return flexResponse(p, fn, z, p.T + 1.5).residual; };
    expect(s(3)).toBeLessThan(s(6));
    expect(s(6)).toBeLessThan(s(10));
  });
});
