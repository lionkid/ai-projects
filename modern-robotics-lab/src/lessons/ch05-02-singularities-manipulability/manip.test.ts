import { describe, expect, it } from 'vitest';
import { svd, planarFK, matVec, manipulability } from '@/mr';
import {
  L2R, L3R, sigmaInfo, tipJacobian, solveTip, elbowBranch, reachAnnulus, sweepTheta2,
  sigmaMinProfile, sampleProfile, radialRates, radialPathConfig, virtualLink,
} from './manip';

const D = Math.PI / 180;

describe('sigmaInfo (closed-form singular values of a 2 x n matrix)', () => {
  it('matches the SVD from @/mr on random 2x2 and 2x3 matrices', () => {
    let seed = 7;
    const rnd = () => { seed = (seed * 16807) % 2147483647; return (seed / 2147483647) * 4 - 2; };
    for (let trial = 0; trial < 40; trial++) {
      const n = trial % 2 ? 3 : 2;
      const J = [Array.from({ length: n }, rnd), Array.from({ length: n }, rnd)];
      const s = sigmaInfo(J);
      const S = svd(J).S;
      expect(s.smax).toBeCloseTo(S[0], 9);
      expect(s.smin).toBeCloseTo(S[1], 9);
      expect(s.mu3).toBeCloseTo(S[0] * S[1], 9);
      // u1 is an eigenvector of J Jᵀ with eigenvalue smax²
      const [[a, b], [, d]] = s.JJt;
      const Au = [a * s.u1[0] + b * s.u1[1], b * s.u1[0] + d * s.u1[1]];
      expect(Au[0]).toBeCloseTo(s.smax * s.smax * s.u1[0], 9);
      expect(Au[1]).toBeCloseTo(s.smax * s.smax * s.u1[1], 9);
    }
  });

  it('agrees with mr.manipulability', () => {
    const J = tipJacobian(L3R, [0.2, 0.9, -0.4]);
    const m = manipulability(J);
    const s = sigmaInfo(J);
    expect(s.smax).toBeCloseTo(m.sigmaMax, 9);
    expect(s.smin).toBeCloseTo(m.sigmaMin, 9);
    expect(s.mu1).toBeCloseTo(m.isotropy, 6);
  });

  it('2R: rank drops exactly at θ2 = 0 and the known values at θ2 = 90°', () => {
    const straight = sigmaInfo(tipJacobian(L2R, [0.4, 0]));
    expect(straight.rank).toBe(1);
    expect(straight.smin).toBeLessThan(1e-7);
    expect(straight.mu3).toBeLessThan(1e-12);
    expect(straight.mu1).toBe(Infinity);
    expect(straight.smax).toBeCloseTo(Math.sqrt(5), 9);
    const right = sigmaInfo(tipJacobian(L2R, [0, Math.PI / 2]));
    expect(right.rank).toBe(2);
    expect(right.smax).toBeCloseTo((1 + Math.sqrt(5)) / 2, 9);
    expect(right.smin).toBeCloseTo((Math.sqrt(5) - 1) / 2, 9);
    // μ3 = |det J| = L1 L2 |sin θ2| for the 2R
    for (const t2 of [0.1, 0.7, 2.5, -1.2]) expect(sigmaInfo(tipJacobian(L2R, [0.3, t2])).mu3).toBeCloseTo(Math.abs(Math.sin(t2)), 10);
  });

  it('3R: rank 1 exactly when all joints line up with the tip', () => {
    for (const t2 of [0, Math.PI]) for (const t3 of [0, Math.PI]) expect(sigmaInfo(tipJacobian(L3R, [0.5, t2, t3])).rank).toBe(1);
    expect(sigmaInfo(tipJacobian(L3R, [0.5, 0, Math.PI / 2])).rank).toBe(2);
    expect(sigmaInfo(tipJacobian(L3R, [0.5, 0.3, Math.PI])).rank).toBe(2);
  });
});

describe('solveTip (virtual 2R inverse kinematics with θ3 frozen)', () => {
  it('reaches reachable targets, keeps θ3 and the elbow branch', () => {
    for (const L of [L2R, L3R]) {
      for (const branch of [1, -1] as const) {
        const th0 = L.length === 3 ? [0.2, 0.5, 1.1] : [0.2, 0.5];
        const target: [number, number] = [0.6, 0.9];
        const { th, clamped } = solveTip(L, th0, target, branch);
        expect(clamped).toBe(false);
        const tip = planarFK(L, th).tip;
        expect(tip[0]).toBeCloseTo(target[0], 9);
        expect(tip[1]).toBeCloseTo(target[1], 9);
        if (L.length === 3) expect(th[2]).toBe(th0[2]);
        expect(elbowBranch(L, th)).toBe(branch);
      }
    }
  });

  it('projects unreachable targets onto the annulus', () => {
    const { th, clamped } = solveTip(L2R, [0, 0.3], [5, 0], 1);
    expect(clamped).toBe(true);
    expect(th[1]).toBeCloseTo(0, 6);
    const tip = planarFK(L2R, th).tip;
    expect(Math.hypot(tip[0], tip[1])).toBeCloseTo(2, 9);
    const th3 = [0, 0.4, Math.PI];
    const [rmin, rmax] = reachAnnulus(L3R, th3);
    expect(rmin).toBeCloseTo(0.8, 9);
    expect(rmax).toBeCloseTo(1.2, 9);
    const inner = solveTip(L3R, th3, [0.1, 0.1], 1);
    expect(inner.clamped).toBe(true);
    const t = planarFK(L3R, inner.th).tip;
    expect(Math.hypot(t[0], t[1])).toBeCloseTo(0.8, 9);
  });

  it('the singular 3R pose A and a bent pose B share the same tip', () => {
    const A = [30 * D, 0, Math.PI];
    const tipA = planarFK(L3R, A).tip;
    const B = solveTip(L3R, [0, 0, Math.PI / 2], [tipA[0], tipA[1]], 1).th;
    const tipB = planarFK(L3R, B).tip;
    expect(tipB[0]).toBeCloseTo(tipA[0], 9);
    expect(tipB[1]).toBeCloseTo(tipA[1], 9);
    expect(sigmaInfo(tipJacobian(L3R, A)).rank).toBe(1);
    expect(sigmaInfo(tipJacobian(L3R, B)).smin).toBeGreaterThan(0.5);
  });

  it('virtual link of a 2R is just link 2', () => {
    expect(virtualLink(L2R, [0.1, 0.2])).toEqual({ lp: 1, beta: 0 });
  });
});

describe('sweeps, radial profiles and rates', () => {
  it('sweepTheta2 puts σmin = 0 at θ2 = 0 and ±180° for the 2R', () => {
    const s = sweepTheta2(L2R, [0, 0], 361);
    expect(s.deg[180]).toBeCloseTo(0, 9);
    expect(s.smin[180]).toBeLessThan(1e-7);
    expect(Number.isNaN(s.mu1[180])).toBe(true);
    expect(s.smin[0]).toBeLessThan(1e-7);
    expect(s.mu1[190]).toBeGreaterThan(20); // θ2 = 10°: already badly conditioned
  });

  it('the radial σmin profile matches direct evaluation', () => {
    for (const [L, th] of [[L2R, [0, 1]], [L3R, [0, 0.7, Math.PI / 2]], [L3R, [0, 0.7, Math.PI]]] as [number[], number[]][]) {
      const br = elbowBranch(L, th);
      const p = sigmaMinProfile(L, th, br, 801);
      for (const r of [p.rmin + 0.05, (p.rmin + p.rmax) / 2, p.rmax - 0.05]) {
        const q = solveTip(L, th, [r * Math.cos(0.8), r * Math.sin(0.8)], br).th;
        expect(sampleProfile(p, r)).toBeCloseTo(sigmaInfo(tipJacobian(L, q)).smin, 3);
      }
      expect(Number.isNaN(sampleProfile(p, p.rmax + 0.1))).toBe(true);
    }
    // θ3 = π folds link 3 back: the dark rings sit at r = 0.8 and 1.2 (inside the 3R workspace)
    const fold = sigmaMinProfile(L3R, [0, 0.5, Math.PI], 1, 401);
    expect(sampleProfile(fold, 1.2)).toBeLessThan(1e-6);
    expect(sampleProfile(fold, 0.8)).toBeLessThan(1e-6);
  });

  it('radialRates gives J θ̇ = v and the 2R closed form |θ̇2| = r |v| / (L1 L2 |sin θ2|)', () => {
    for (const t2 of [30 * D, 3 * D, -40 * D]) {
      const th = [0.3, t2];
      const rr = radialRates(L2R, th, 0.1);
      expect(rr.qdot).not.toBeNull();
      const v = matVec(tipJacobian(L2R, th), rr.qdot!);
      expect(v[0]).toBeCloseTo(rr.v[0], 9);
      expect(v[1]).toBeCloseTo(rr.v[1], 9);
      const tip = planarFK(L2R, th).tip;
      const r = Math.hypot(tip[0], tip[1]);
      expect(Math.abs(rr.qdot![1])).toBeCloseTo((r * 0.1) / Math.abs(Math.sin(t2)), 9);
    }
    expect(radialRates(L2R, [0.3, 0], 0.1).qdot).toBeNull();
    // 3R uses the minimum-norm solution
    const rr3 = radialRates(L3R, [0.2, 0.6, -0.9], 0.1);
    const v3 = matVec(tipJacobian(L3R, [0.2, 0.6, -0.9]), rr3.qdot!);
    expect(v3[0]).toBeCloseTo(rr3.v[0], 9);
  });

  it('radialPathConfig keeps the tip on the ray', () => {
    const th0 = [0.4, 1.0];
    const tip0 = planarFK(L2R, th0).tip as [number, number];
    const th = radialPathConfig(L2R, tip0, 0.2, 1, th0);
    const tip = planarFK(L2R, th).tip;
    const r0 = Math.hypot(...tip0);
    expect(Math.hypot(tip[0], tip[1])).toBeCloseTo(r0 + 0.2, 9);
    expect(Math.atan2(tip[1], tip[0])).toBeCloseTo(Math.atan2(tip0[1], tip0[0]), 9);
    expect(th[1]).toBeGreaterThan(0);
  });
});
