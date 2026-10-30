import { describe, expect, it } from 'vitest';
import { planarFK, svd, matVec, transpose } from '@/mr';
import {
  MODELS, lengthsOf, staticTorques, tipJacobian, ellipseAxes, momentArms, solveTip, otherSolution, elbowBranch,
  POSTURES_2R, powers, vnorm, type ModelKey,
} from './statics';

let seed = 11;
const rnd = (lo = -1, hi = 1) => { seed = (seed * 16807) % 2147483647; return lo + ((hi - lo) * seed) / 2147483647; };

describe('τ = Jᵀ F with correctly paired frames', () => {
  it('J_bᵀ F_b = J_sᵀ F_s = J_vᵀ f, and equals the moment-arm picture', () => {
    for (const key of ['2R', '3R'] as ModelKey[]) {
      const model = MODELS[key];
      const L = lengthsOf(key);
      for (let trial = 0; trial < 20; trial++) {
        const th = L.map(() => rnd(-Math.PI, Math.PI));
        const f: [number, number] = [rnd(-30, 30), rnd(-30, 30)];
        const r = staticTorques(model, th, f);
        const tauV = matVec(transpose(tipJacobian(L, th)), f);
        const arms = momentArms(L, th, f);
        const fm = Math.hypot(f[0], f[1]);
        r.tau.forEach((t, i) => {
          expect(r.tauSpace[i]).toBeCloseTo(t, 9);
          expect(tauV[i]).toBeCloseTo(t, 9);
          expect(arms[i].d * fm).toBeCloseTo(t, 9);
        });
      }
    }
  });

  it('F_s carries the moment p × f about the {s} origin; mixing frames gives a different answer', () => {
    const model = MODELS['2R'];
    const th = [0.4, 0.9];
    const f: [number, number] = [5, -12];
    const r = staticTorques(model, th, f);
    const p = planarFK([1, 1], th).tip;
    expect(r.Fs[2]).toBeCloseTo(p[0] * f[1] - p[1] * f[0], 9);
    expect(r.Fs[3]).toBeCloseTo(f[0], 9);
    expect(r.Fs[4]).toBeCloseTo(f[1], 9);
    expect(Math.hypot(r.Fb[3], r.Fb[4])).toBeCloseTo(Math.hypot(f[0], f[1]), 9);
    expect(r.Fb[2]).toBe(0);
    expect(Math.abs(r.tauWrong[0] - r.tau[0]) + Math.abs(r.tauWrong[1] - r.tau[1])).toBeGreaterThan(1);
  });

  it('a straight arm pushing along its links needs no torque', () => {
    const a = 0.5;
    const r = staticTorques(MODELS['2R'], [a, 0], [20 * Math.cos(a), 20 * Math.sin(a)]);
    r.tau.forEach((t) => expect(Math.abs(t)).toBeLessThan(1e-9));
    const r3 = staticTorques(MODELS['3R'], [a, 0, 0], [-15 * Math.cos(a), -15 * Math.sin(a)]);
    r3.tau.forEach((t) => expect(Math.abs(t)).toBeLessThan(1e-9));
  });
});

describe('velocity and force ellipses', () => {
  it('semi-axes are the singular values; the force ellipse boundary has |Jᵀ f| = 1 with reciprocal axes', () => {
    for (const key of ['2R', '3R'] as ModelKey[]) {
      const L = lengthsOf(key);
      const th = L.map(() => rnd(-2, 2));
      const J = tipJacobian(L, th);
      const ax = ellipseAxes(J);
      const S = svd(J).S;
      expect(ax.s1).toBeCloseTo(S[0], 9);
      expect(ax.s2).toBeCloseTo(S[1], 9);
      // force-ellipse boundary points along the principal directions
      const JT = transpose(J);
      expect(vnorm(matVec(JT, [ax.u1[0] / ax.s1, ax.u1[1] / ax.s1]))).toBeCloseTo(1, 9);
      expect(vnorm(matVec(JT, [ax.u2[0] / ax.s2, ax.u2[1] / ax.s2]))).toBeCloseTo(1, 9);
      // velocity ellipse: J (unit θ̇) reaches s1 along u1
      expect(ax.s1 * (1 / ax.s1)).toBeCloseTo(1, 12);
    }
  });

  it('reports rank 1 at full extension', () => {
    expect(ellipseAxes(tipJacobian([1, 1], [0.3, 0])).rank).toBe(1);
    expect(ellipseAxes(tipJacobian([1, 1], [0.3, 1])).rank).toBe(2);
  });
});

describe('power balance θ̇ᵀτ = Vᵀ F', () => {
  it('holds for arbitrary virtual joint velocities', () => {
    for (const key of ['2R', '3R'] as ModelKey[]) {
      const model = MODELS[key];
      const L = lengthsOf(key);
      for (let k = 0; k < 10; k++) {
        const th = L.map(() => rnd(-3, 3));
        const qd = L.map(() => rnd(-1, 1));
        const p = powers(model, th, qd, [rnd(-20, 20), rnd(-20, 20)]);
        expect(p.joint).toBeCloseTo(p.tip, 9);
      }
    }
  });
});

describe('inverse kinematics helpers', () => {
  it('solveTip reaches the target and otherSolution flips the elbow with the same tip', () => {
    for (const key of ['2R', '3R'] as ModelKey[]) {
      const L = lengthsOf(key);
      const th = key === '2R' ? [0.3, 1.1] : [0.3, 1.1, 0.7];
      const q = solveTip(L, th, [0.4, 1.2], 1);
      const tip = planarFK(L, q).tip;
      expect(tip[0]).toBeCloseTo(0.4, 9);
      expect(tip[1]).toBeCloseTo(1.2, 9);
      const o = otherSolution(L, q)!;
      const tip2 = planarFK(L, o).tip;
      expect(tip2[0]).toBeCloseTo(0.4, 9);
      expect(tip2[1]).toBeCloseTo(1.2, 9);
      expect(elbowBranch(L, o)).toBe(-elbowBranch(L, q));
    }
    expect(otherSolution([1, 1], [0.2, 0])).toBeNull();
  });

  it('2R: both elbow solutions share τ1 = (p × f)_z but not τ2', () => {
    const f: [number, number] = [0, 19.6];
    const a = staticTorques(MODELS['2R'], [0, Math.PI / 2], f).tau;
    const b = staticTorques(MODELS['2R'], [Math.PI / 2, -Math.PI / 2], f).tau;
    expect(a[0]).toBeCloseTo(b[0], 9);
    expect(a[1]).toBeCloseTo(0, 9);
    expect(b[1]).toBeCloseTo(19.6, 9);
  });

  it('posture table: holding a load with the arm straight up costs nothing', () => {
    const up = POSTURES_2R.find((p) => p.label === '垂直伸直')!;
    staticTorques(MODELS['2R'], up.th, [0, 19.6]).tau.forEach((t) => expect(Math.abs(t)).toBeLessThan(1e-9));
    const flat = POSTURES_2R.find((p) => p.label === '水平伸直')!;
    const t = staticTorques(MODELS['2R'], flat.th, [0, 19.6]).tau;
    expect(t[0]).toBeCloseTo(39.2, 9);
    expect(t[1]).toBeCloseTo(19.6, 9);
  });
});
