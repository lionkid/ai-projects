import { describe, expect, it } from 'vitest';
import { FKinSpace, getCol, matMul, rrr3, transformPoint } from '@/mr';
import { MODELS, MODEL_ORDER, ONE_R, RRRP } from './models';
import {
  axisPlacement, chainTransforms, currentAxes, currentAxesFromJacobian, distanceToAxis, factorIndex,
  leftToRightPartial, movedAxes, originGap, rehome, reversedTransforms, rightToLeftPartial, screwExp,
  transformGap,
} from './poe';

const DEG = Math.PI / 180;

/** Element-wise closeness (also treats +0 and −0 alike). */
function expectVec(a: number[], b: number[], digits = 12): void {
  expect(a.length).toBe(b.length);
  a.forEach((x, i) => expect(x).toBeCloseTo(b[i], digits));
}

/** Deterministic pseudo-random joint vectors inside the model limits. */
function samples(lim: [number, number][], count: number, seed = 7): number[][] {
  let s = seed;
  const rnd = () => { s = (s * 16807) % 2147483647; return s / 2147483647; };
  return Array.from({ length: count }, () => lim.map(([lo, hi]) => lo + (hi - lo) * rnd()));
}

describe('models', () => {
  it('one-joint arm: S = (ω, −ω × q) with q = (0.2, 0, 0.3)', () => {
    const S = getCol(MODELS.oneR.Slist, 0);
    expectVec(S, [0, 1, 0, -ONE_R.h, 0, ONE_R.x]);
    expectVec(S, [0, 1, 0, -0.3, 0, 0.2]);
  });

  it('RRRP: three vertical revolute axes and a pure-translation fourth axis', () => {
    const m = MODELS.rrrp;
    expectVec(getCol(m.Slist, 1), [0, 0, 1, 0, -RRRP.A1, 0]);
    expectVec(getCol(m.Slist, 2), [0, 0, 1, 0, -(RRRP.A1 + RRRP.A2), 0]);
    expectVec(getCol(m.Slist, 3), [0, 0, 0, 0, 0, 1]);
    expect(m.types).toEqual(['R', 'R', 'R', 'P']);
  });

  it('planar 3R: S_i = (0, 0, 1, y_i, −x_i, 0)', () => {
    const S = MODELS.planar3R.Slist;
    expectVec(getCol(S, 0), [0, 0, 1, 0, 0, 0]);
    expectVec(getCol(S, 1), [0, 0, 1, 0, -1, 0]);
    expectVec(getCol(S, 2), [0, 0, 1, 0, -1.8, 0]);
    expectVec([MODELS.planar3R.M[0][3], MODELS.planar3R.M[1][3], MODELS.planar3R.M[2][3]], [2.4, 0, 0]);
  });

  for (const key of MODEL_ORDER) {
    const m = MODELS[key];
    it(`${key}: drawing geometry agrees with (M, Slist)`, () => {
      const n = m.Slist[0].length;
      expect(m.types.length).toBe(n);
      expect(m.limits.length).toBe(n);
      expect(m.anchors.length).toBe(n);
      expect(m.demo.length).toBe(n);
      // every revolute anchor lies on its axis
      m.anchors.forEach((a, i) => expect(distanceToAxis(getCol(m.Slist, i), a)).toBeLessThan(1e-12));
      // the last link ends at the tool point M
      const last = m.segments.filter((s) => s.body === n && s.role === 'link').pop()!;
      const ends = [last.a, last.b];
      const tip = [m.M[0][3], m.M[1][3], m.M[2][3]];
      expect(Math.min(...ends.map((e) => Math.hypot(e[0] - tip[0], e[1] - tip[1], e[2] - tip[2])))).toBeLessThan(1e-12);
      // demo pose inside the limits
      m.demo.forEach((x, i) => { expect(x).toBeGreaterThanOrEqual(m.limits[i][0]); expect(x).toBeLessThanOrEqual(m.limits[i][1]); });
    });
  }
});

describe('partial products', () => {
  it('chainTransforms(…)[n] · M equals FKinSpace for every model', () => {
    for (const key of MODEL_ORDER) {
      const m = MODELS[key];
      for (const th of samples(m.limits, 6)) {
        const L = chainTransforms(m.Slist, th);
        expect(transformGap(matMul(L[th.length], m.M), FKinSpace(m.M, m.Slist, th))).toBeLessThan(1e-12);
      }
    }
  });

  it('right-to-left stages reproduce e^{[Sk]θk} ⋯ e^{[Sn]θn} M', () => {
    const m = MODELS.rrrp;
    const th = m.demo;
    const n = th.length;
    for (let stage = 0; stage <= n; stage++) {
      const part = rightToLeftPartial(th, stage);
      let T = m.M;
      for (let k = n - 1; k >= n - stage; k--) T = matMul(screwExp(getCol(m.Slist, k), th[k]), T);
      expect(transformGap(FKinSpace(m.M, m.Slist, part), T)).toBeLessThan(1e-12);
    }
    expectVec(rightToLeftPartial(th, 0), [0, 0, 0, 0]);
    expectVec(rightToLeftPartial(th, n), th);
    // half of the next factor: joint n − s − 1 at half its value
    expectVec(rightToLeftPartial([1, 2, 3], 1, 0.5), [0, 1, 3]);
    expectVec(leftToRightPartial([1, 2, 3], 1, 0.5), [1, 1, 0]);
    expect(factorIndex(3, 0, 'rtl')).toBe(2);
    expect(factorIndex(3, 0, 'ltr')).toBe(0);
  });

  it('applying the factors left-to-right about the HOME axes breaks the chain', () => {
    const m = MODELS.rrr3;
    const th = m.demo;
    const L = reversedTransforms(m.Slist, th);
    const wrong = matMul(L[3], m.M);
    expect(originGap(wrong, FKinSpace(m.M, m.Slist, th))).toBeGreaterThan(0.05);
    // the elbow carried by link 2 and by link 3 no longer coincide
    const elbow = [0.4, 0, 0.3];
    const a = transformPoint(L[2], elbow), b = transformPoint(L[3], elbow);
    expect(Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2])).toBeGreaterThan(0.01);
    // …while the correct order keeps every joint attached
    const C = chainTransforms(m.Slist, th);
    const c2 = transformPoint(C[2], elbow), c3 = transformPoint(C[3], elbow);
    expect(Math.hypot(c2[0] - c3[0], c2[1] - c3[1], c2[2] - c3[2])).toBeLessThan(1e-12);
    // with a single moving joint both orders agree
    const single = [0, 0.7, 0];
    expect(transformGap(reversedTransforms(m.Slist, single)[3], chainTransforms(m.Slist, single)[3])).toBeLessThan(1e-12);
  });
});

describe('current axes', () => {
  it('match the columns of the space Jacobian', () => {
    for (const key of MODEL_ORDER) {
      const m = MODELS[key];
      for (const th of samples(m.limits, 4, 11)) {
        const a = currentAxes(m.Slist, th);
        const b = currentAxesFromJacobian(m.Slist, th);
        a.forEach((S, i) => expectVec(S, b[i]));
      }
    }
  });

  it('moving joint k only moves the axes downstream of k', () => {
    const S = rrr3.Slist;
    expect(movedAxes(S, [50 * DEG, 0, 0])).toEqual([false, true, true]);
    expect(movedAxes(S, [0, 30 * DEG, 0])).toEqual([false, false, true]);
    expect(movedAxes(S, [0, 0, 80 * DEG])).toEqual([false, false, false]);
  });

  it('axisPlacement centres revolute glyphs at the anchor and keeps P axes at the anchor', () => {
    const r = axisPlacement([0, 1, 0, -0.3, 0, 0.2], [0.2, 5, 0.3]);
    expect(r.prismatic).toBe(false);
    expectVec(r.point, [0.2, 5, 0.3]);
    expectVec(r.dir, [0, 1, 0]);
    const p = axisPlacement([0, 0, 0, 0, 0, 2], [1, 2, 3]);
    expect(p.prismatic).toBe(true);
    expectVec(p.dir, [0, 0, 1]);
    expectVec(p.point, [1, 2, 3]);
  });
});

describe('rehoming', () => {
  it('leaves the forward kinematics unchanged: T′(φ) = T(θ0 + φ)', () => {
    for (const key of MODEL_ORDER) {
      const m = MODELS[key];
      const [theta0] = samples(m.limits, 1, 23);
      const re = rehome(m.M, m.Slist, theta0);
      for (const phi of samples(m.limits.map(() => [-1, 1] as [number, number]), 5, 31)) {
        const T1 = FKinSpace(re.M, re.Slist, phi);
        const T0 = FKinSpace(m.M, m.Slist, theta0.map((x, i) => x + phi[i]));
        expect(transformGap(T1, T0)).toBeLessThan(1e-12);
      }
    }
  });

  it('rrr3 with θ0 = (90°, −90°, 90°): S′1 = S1, M′ and the other axes change as predicted', () => {
    const re = rehome(rrr3.M, rrr3.Slist, [90 * DEG, -90 * DEG, 90 * DEG]);
    const S = (j: number) => getCol(re.Slist, j);
    expectVec(S(0), getCol(rrr3.Slist, 0));
    expectVec(S(1), [-1, 0, 0, 0, -0.3, 0]);
    expectVec(S(2), [-1, 0, 0, 0, -0.7, 0]);
    expectVec([re.M[0][3], re.M[1][3], re.M[2][3]], [0, 0.3, 0.7]);
  });
});
