import { describe, expect, it } from 'vitest';
import {
  type Mat,
  type Vec,
  FKinSpace,
  MatrixLog6,
  TransInv,
  getCol,
  matMul,
  matScale,
  matSub,
  matVec,
  norm,
  planar3R,
  planarFK,
  planarJacobian,
  rrr3,
  se3ToVec,
  transpose,
  vecSub,
} from '@/mr';
import {
  axisPoint,
  changedColumns,
  instantCenter,
  jacobianSnapshot,
  planarAdjoint,
  planarRows,
  planarTwist,
  pointVelocity,
  singularityMeasure,
  tipPointJacobian,
  velocityBreakdown,
} from './jacobianGeometry';

const DEG = Math.PI / 180;
const L = planar3R.linkLengths!;
const configs: Vec[] = [
  [0, 0, 0],
  [20 * DEG, 50 * DEG, 40 * DEG],
  [-1.1, 2.3, -0.7],
  [2.9, -1.4, 0.3],
];

const maxAbsDiff = (a: Mat | Vec, b: Mat | Vec): number => {
  const fa = (a as unknown[]).flat() as number[];
  const fb = (b as unknown[]).flat() as number[];
  return Math.max(...fa.map((x, i) => Math.abs(x - fb[i])));
};

describe('jacobianSnapshot', () => {
  it('satisfies J_s = [Ad_Tsb] J_b for planar3R and rrr3', () => {
    for (const th of configs) {
      expect(jacobianSnapshot(planar3R, th).residual).toBeLessThan(1e-12);
      expect(jacobianSnapshot(rrr3, [th[0], th[1] * 0.5, th[2]]).residual).toBeLessThan(1e-12);
    }
  });

  it('keeps the first column of J_s equal to S_1 and the last column of J_b equal to B_n', () => {
    for (const th of configs) {
      const s = jacobianSnapshot(planar3R, th);
      expect(maxAbsDiff(getCol(s.Js, 0), getCol(planar3R.Slist, 0))).toBeLessThan(1e-12);
      expect(maxAbsDiff(getCol(s.Jb, 2), getCol(planar3R.Blist, 2))).toBeLessThan(1e-12);
    }
  });

  it('planar3R: J_s ignores θ3 and J_b ignores θ1', () => {
    const a = jacobianSnapshot(planar3R, [0.3, 0.8, -0.4]);
    const b = jacobianSnapshot(planar3R, [0.3, 0.8, 1.9]); // only θ3 changed
    const c = jacobianSnapshot(planar3R, [-1.2, 0.8, -0.4]); // only θ1 changed
    expect(changedColumns(a.Js, b.Js)).toEqual([false, false, false]);
    expect(changedColumns(a.Js, c.Js)).toEqual([false, true, true]);
    expect(changedColumns(a.Jb, c.Jb)).toEqual([false, false, false]);
    expect(changedColumns(a.Jb, b.Jb)).toEqual([true, true, false]);
  });

  it('planar3R: only the rows [ω_z, v_x, v_y] are non-zero', () => {
    for (const th of configs) {
      const s = jacobianSnapshot(planar3R, th);
      for (const r of [0, 1, 5]) for (const J of [s.Js, s.Jb]) expect(Math.max(...J[r].map(Math.abs))).toBeLessThan(1e-12);
      // the planar 3 × 3 blocks still satisfy the Adjoint relation
      const res = matSub(planarRows(s.Js), matMul(planarAdjoint(s.Ad), planarRows(s.Jb)));
      expect(Math.max(...res.flat().map(Math.abs))).toBeLessThan(1e-12);
    }
  });

  it('the axis point of J_s column i is the current position of joint i', () => {
    for (const th of configs) {
      const s = jacobianSnapshot(planar3R, th);
      const fk = planarFK(L, th);
      for (let i = 0; i < 3; i++) {
        const q = axisPoint(getCol(s.Js, i))!;
        expect(Math.hypot(q[0] - fk.joints[i][0], q[1] - fk.joints[i][1])).toBeLessThan(1e-12);
        // the same axis seen from {b}: q_b = R^T (q_s - p)
        const qb = axisPoint(getCol(s.Jb, i))!;
        const expected = matVec(transpose(s.R), vecSub(q, s.p));
        expect(maxAbsDiff(qb, expected)).toBeLessThan(1e-12);
      }
      expect(maxAbsDiff([s.p[0], s.p[1]], fk.tip)).toBeLessThan(1e-12);
    }
  });

  it('reports det J_s = L1 L2 sin θ2 for the planar arm (singular when θ2 = 0)', () => {
    for (const th of configs) {
      expect(singularityMeasure(jacobianSnapshot(planar3R, th), true)).toBeCloseTo(L[0] * L[1] * Math.sin(th[1]), 12);
    }
    expect(Math.abs(singularityMeasure(jacobianSnapshot(planar3R, [0.7, 0, 1.1]), true))).toBeLessThan(1e-12);
  });

  it('flags the rrr3 tip-point Jacobian as singular with a straight elbow', () => {
    expect(Math.abs(singularityMeasure(jacobianSnapshot(rrr3, [0.4, -0.5, 0]), false))).toBeLessThan(1e-12);
    expect(Math.abs(singularityMeasure(jacobianSnapshot(rrr3, [0.4, -0.5, 1.0]), false))).toBeGreaterThan(1e-3);
  });
});

describe('velocityBreakdown', () => {
  const thd: Vec = [0.6, -0.5, 0.9];

  it('gives the tip-point velocity of planarJacobian (v_s + ω × p), not v_s itself', () => {
    for (const th of configs.slice(1)) {
      const s = jacobianSnapshot(planar3R, th);
      const v = velocityBreakdown(s, thd);
      const Jtip = planarJacobian(L, th);
      const expected = matVec(Jtip, thd); // [ω_z, ẋ, ẏ]
      expect(v.omegaS[2]).toBeCloseTo(expected[0], 12);
      expect(v.tipVel[0]).toBeCloseTo(expected[1], 12);
      expect(v.tipVel[1]).toBeCloseTo(expected[2], 12);
      expect(norm(vecSub(v.vS, v.tipVel))).toBeGreaterThan(1e-3);
    }
  });

  it('sums the per-joint contributions to the total twist and tip velocity', () => {
    const s = jacobianSnapshot(planar3R, configs[1]);
    const v = velocityBreakdown(s, thd);
    const sumTwist = v.contributions.reduce((acc, c) => acc.map((x, i) => x + c.twistS[i]), [0, 0, 0, 0, 0, 0]);
    const sumTip = v.contributions.reduce((acc, c) => acc.map((x, i) => x + c.tipVel[i]), [0, 0, 0]);
    expect(maxAbsDiff(sumTwist, v.Vs)).toBeLessThan(1e-12);
    expect(maxAbsDiff(sumTip, v.tipVel)).toBeLessThan(1e-12);
  });

  it('matches finite differences of the forward kinematics (rrr3)', () => {
    const th: Vec = [0.5, -0.6, 1.2];
    const rate: Vec = [0.7, -0.4, 0.9];
    const h = 1e-6;
    const s = jacobianSnapshot(rrr3, th);
    const v = velocityBreakdown(s, rate);
    const T0 = FKinSpace(rrr3.M, rrr3.Slist, th);
    const T1 = FKinSpace(rrr3.M, rrr3.Slist, th.map((x, i) => x + h * rate[i]));
    // tip velocity
    const pdot = [0, 1, 2].map((r) => (T1[r][3] - T0[r][3]) / h);
    expect(maxAbsDiff(pdot, v.tipVel)).toBeLessThan(1e-5);
    // space twist: [V_s] ≈ log(T1 T0^-1) / h, body twist: [V_b] ≈ log(T0^-1 T1) / h
    const Vs = se3ToVec(matScale(MatrixLog6(matMul(T1, TransInv(T0))), 1 / h));
    const Vb = se3ToVec(matScale(MatrixLog6(matMul(TransInv(T0), T1)), 1 / h));
    expect(maxAbsDiff(Vs, v.Vs)).toBeLessThan(1e-5);
    expect(maxAbsDiff(Vb, v.Vb)).toBeLessThan(1e-5);
    // the linear part of V_b is the tip velocity written in {b}
    expect(maxAbsDiff(v.vB, matVec(transpose(s.R), v.tipVel))).toBeLessThan(1e-12);
    // tip-point Jacobian times θ̇ is the same tip velocity
    expect(maxAbsDiff(matVec(tipPointJacobian(s), rate), v.tipVel)).toBeLessThan(1e-12);
  });
});

describe('instantCenter', () => {
  it('is a point with zero velocity', () => {
    const s = jacobianSnapshot(planar3R, configs[1]);
    const V = velocityBreakdown(s, [0.6, -0.5, 0.9]).Vs;
    const c = instantCenter(planarTwist(V))!;
    expect(norm(pointVelocity(V, [c[0], c[1], 0]))).toBeLessThan(1e-12);
  });

  it('is the joint itself when a single joint moves', () => {
    const th = configs[2];
    const s = jacobianSnapshot(planar3R, th);
    const fk = planarFK(L, th);
    for (let i = 0; i < 3; i++) {
      const e: Vec = [0, 0, 0];
      e[i] = 1;
      const c = instantCenter(planarTwist(velocityBreakdown(s, e).Vs))!;
      expect(Math.hypot(c[0] - fk.joints[i][0], c[1] - fk.joints[i][1])).toBeLessThan(1e-12);
    }
  });

  it('does not exist for a pure translation (Σθ̇ = 0)', () => {
    const s = jacobianSnapshot(planar3R, configs[1]);
    const v = velocityBreakdown(s, [0.5, -1, 0.5]);
    expect(Math.abs(v.omegaS[2])).toBeLessThan(1e-12);
    expect(instantCenter(planarTwist(v.Vs))).toBeNull();
    // for a pure translation v_s equals the tip velocity
    expect(maxAbsDiff(v.vS, v.tipVel)).toBeLessThan(1e-12);
  });
});

describe('changedColumns / axisPoint', () => {
  it('detects changed columns with a tolerance', () => {
    const A: Mat = [[1, 2], [3, 4]];
    expect(changedColumns(A, [[1, 2 + 1e-12], [3, 4]])).toEqual([false, false]);
    expect(changedColumns(A, [[1, 2], [3.1, 4]])).toEqual([true, false]);
  });

  it('returns null for a prismatic column', () => {
    expect(axisPoint([0, 0, 0, 1, 0, 0])).toBeNull();
  });
});
