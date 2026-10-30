import { describe, expect, it } from 'vitest';
import { TotalEnergy, planar2R } from '@/mr';
import {
  Twin,
  analyzeAccel,
  armEnergy,
  armParams,
  assembleForwardDynamics,
  errorSweep,
  fitLogSlope,
  gravityVec,
  simulateTo,
} from './integrators';

const SMALL = [-Math.PI / 2 + 0.4, 0.3, 0, 0];
const UPRIGHT = [Math.PI / 2 - 0.15, 0.2, 0, 0];

describe('forward dynamics at one state', () => {
  it('assembling M and c + g from inverse-dynamics calls matches ForwardDynamics and the closed form', () => {
    const states = [
      { th: [0.3, -0.7], dth: [1.2, -0.4], tau: [3, -1] },
      { th: [-1.9, 2.4], dth: [-2, 3], tau: [0, 0] },
      { th: [2.8, 0.1], dth: [0, 0], tau: [-5, 2] },
    ];
    for (const gravityOn of [true, false]) {
      for (const s of states) {
        const a = assembleForwardDynamics(s.th, s.dth, s.tau, gravityOn);
        expect(a.calls).toHaveLength(3);
        expect(a.calls[0].kind).toBe('bias');
        for (let i = 0; i < 2; i++) {
          expect(a.ddtheta[i]).toBeCloseTo(a.ddthetaLibrary[i], 9);
          expect(a.ddtheta[i]).toBeCloseTo(a.ddthetaClosed[i], 9);
        }
        // the closed-form M equals the column-by-column assembly
        const an = analyzeAccel(armParams(gravityOn), s.th, s.dth, s.tau);
        for (let i = 0; i < 2; i++) {
          for (let j = 0; j < 2; j++) expect(a.M[i][j]).toBeCloseTo(an.M[i][j], 9);
          expect(a.bias[i]).toBeCloseTo(an.c[i] + an.g[i], 9);
        }
      }
    }
  });

  it('M is symmetric positive definite and τ = c + g gives zero acceleration', () => {
    const P = armParams(true);
    for (let k = 0; k < 40; k++) {
      const th = [Math.sin(1.3 * k) * 3, Math.cos(0.7 * k) * 3];
      const dth = [Math.sin(k), -Math.cos(2 * k)];
      const base = analyzeAccel(P, th, dth, [0, 0]);
      expect(base.M[0][1]).toBeCloseTo(base.M[1][0], 12);
      expect(base.eig[1]).toBeGreaterThan(0);
      const hold = analyzeAccel(P, th, dth, [base.c[0] + base.g[0], base.c[1] + base.g[1]]);
      expect(Math.abs(hold.ddtheta[0]) + Math.abs(hold.ddtheta[1])).toBeLessThan(1e-9);
    }
  });

  it('armEnergy agrees with the Newton–Euler TotalEnergy', () => {
    const P = armParams(true);
    const x = [0.4, -1.1, 0.8, 1.7];
    const ref = TotalEnergy([x[0], x[1]], [x[2], x[3]], gravityVec(true), planar2R.Mlist!, planar2R.Glist!, planar2R.Slist);
    expect(armEnergy(P, x).total).toBeCloseTo(ref, 9);
  });
});

describe('integration', () => {
  it('explicit Euler pumps energy into a swinging arm while RK4 conserves it', () => {
    const P = armParams(true);
    const e = new Twin('euler', SMALL, P);
    const r = new Twin('rk4', SMALL, P);
    e.advanceTo(10, 0.01, P, () => [0, 0]);
    r.advanceTo(10, 0.01, P, () => [0, 0]);
    expect(e.energy(P) - e.E0).toBeGreaterThan(1);
    expect(Math.abs(r.energy(P) - r.E0)).toBeLessThan(1e-5);
  });

  it('global error orders: Euler ≈ 1, RK4 ≈ 4', () => {
    for (const x0 of [SMALL, UPRIGHT]) {
      const s = errorSweep(armParams(true), x0);
      expect(s.slopeEuler).toBeGreaterThan(0.85);
      expect(s.slopeEuler).toBeLessThan(1.2);
      expect(s.slopeRK4).toBeGreaterThan(3.6);
      expect(s.slopeRK4).toBeLessThan(4.4);
      // halving Δt: Euler error ≈ ½, RK4 error ≈ 1/16 (in the asymptotic range)
      const rows = s.rows.filter((row) => row.dt <= 0.0125);
      for (let i = 1; i < rows.length; i++) {
        expect(rows[i].euler / rows[i - 1].euler).toBeGreaterThan(0.4);
        expect(rows[i].euler / rows[i - 1].euler).toBeLessThan(0.6);
        expect(rows[i].rk4 / rows[i - 1].rk4).toBeLessThan(0.1);
      }
    }
  });

  it('twins take whole steps only and flag blow-ups', () => {
    const P = armParams(true);
    const tw = new Twin('rk4', SMALL, P);
    expect(tw.advanceTo(0.1, 0.03, P, () => [0, 0])).toBe(3);
    expect(tw.t).toBeCloseTo(0.09, 12);
    const bad = new Twin('euler', UPRIGHT, P);
    bad.advanceTo(10, 0.15, P, () => [0, 0]);
    expect(bad.diverged).toBe(true);
    expect(bad.x.every(Number.isFinite)).toBe(true);
    expect(simulateTo('euler', P, UPRIGHT, 10, 0.15)).toBeNull();
  });

  it('fitLogSlope recovers a power law', () => {
    const dts = [0.1, 0.05, 0.025, 0.0125];
    expect(fitLogSlope(dts, dts.map((d) => 2 * d ** 3))).toBeCloseTo(3, 9);
    expect(Number.isNaN(fitLogSlope([0.1], [1]))).toBe(true);
  });
});
