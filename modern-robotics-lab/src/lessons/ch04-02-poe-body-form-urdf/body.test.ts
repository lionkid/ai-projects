import { describe, expect, it } from 'vitest';
import { FKinBody, FKinSpace, UR5_DIMENSIONS, getCol, matMul, ur5 } from '@/mr';
import {
  bodyAxes, bodyPartial, bodyProduct, carry, chainTransforms, changedColumns, currentAxes, gluedToB, passingProduct,
  pointVelocity, sameLine, toolFrame, transformGap,
} from './body';
import { BOOK_P, POSES, TOOL_LENGTH, TOOL_SEGMENT, UR5_ANCHORS, UR5_SEGMENTS } from './ur5';

const DEG = Math.PI / 180;
const { W1, W2, L1, L2, H2 } = UR5_DIMENSIONS;

function expectVec(a: number[], b: number[], digits = 12): void {
  expect(a.length).toBe(b.length);
  a.forEach((x, i) => expect(x).toBeCloseTo(b[i], digits));
}

function samples(count: number, seed = 5): number[][] {
  let s = seed;
  const rnd = () => { s = (s * 16807) % 2147483647; return s / 2147483647; };
  return Array.from({ length: count }, () => Array.from({ length: 6 }, () => (rnd() * 2 - 1) * Math.PI));
}

describe('UR5 body screw axes', () => {
  it('B_i = [Ad_{M⁻¹}] S_i reproduces the familiar closed forms', () => {
    const B = bodyAxes(ur5.Slist, ur5.M);
    expectVec(getCol(B, 0), [0, 1, 0, W1 + W2, 0, L1 + L2]);
    expectVec(getCol(B, 1), [0, 0, 1, H2, -(L1 + L2), 0]);
    expectVec(getCol(B, 2), [0, 0, 1, H2, -L2, 0]);
    expectVec(getCol(B, 3), [0, 0, 1, H2, 0, 0]);
    expectVec(getCol(B, 4), [0, -1, 0, -W2, 0, 0]);
    expectVec(getCol(B, 5), [0, 0, 1, 0, 0, 0]);
    // and agrees with the model's own Blist
    for (let j = 0; j < 6; j++) expectVec(getCol(B, j), getCol(ur5.Blist, j));
  });

  it('book example pose: p ≈ (0.095, 0.109, 0.988) both ways', () => {
    const th = POSES.find((p) => p.key === 'book')!.theta;
    const Ts = FKinSpace(ur5.M, ur5.Slist, th);
    const Tb = FKinBody(ur5.M, ur5.Blist, th);
    expect(transformGap(Ts, Tb)).toBeLessThan(1e-12);
    expectVec([Ts[0][3], Ts[1][3], Ts[2][3]], BOOK_P, 3);
  });
});

describe('moving M to the front', () => {
  it('every intermediate expression equals T(θ)', () => {
    for (const th of samples(5)) {
      const T = FKinSpace(ur5.M, ur5.Slist, th);
      for (let k = 0; k <= 6; k++) expect(transformGap(passingProduct(ur5.Slist, ur5.Blist, ur5.M, th, k), T)).toBeLessThan(1e-12);
      expect(transformGap(bodyProduct(ur5.M, ur5.Blist, th), FKinBody(ur5.M, ur5.Blist, th))).toBeLessThan(1e-12);
    }
  });

  it('body step-through: after k factors the arm sits at (θ1..θk, 0..0)', () => {
    const th = POSES[2].theta;
    expectVec(bodyPartial(th, 0), [0, 0, 0, 0, 0, 0]);
    expectVec(bodyPartial(th, 6), th);
    expectVec(bodyPartial([1, 2, 3], 1, 0.5), [1, 1, 0]);
    for (let k = 0; k <= 6; k++) {
      let T = ur5.M;
      for (let i = 0; i < k; i++) T = bodyProduct(T, ur5.Blist.map((r) => [r[i]]), [th[i]]);
      expect(transformGap(FKinSpace(ur5.M, ur5.Slist, bodyPartial(th, k)), T)).toBeLessThan(1e-12);
    }
  });
});

describe('axes glued to {b}', () => {
  it('only θ6 moving: all S_i stay on their joints, only B6 does', () => {
    const th = [0, 0, 0, 0, 0, 70 * DEG];
    const T = FKinSpace(ur5.M, ur5.Slist, th);
    const now = currentAxes(ur5.Slist, th);
    const sOn = now.map((S, j) => sameLine(S, getCol(ur5.Slist, j)));
    const bOn = now.map((S, j) => sameLine(S, gluedToB(T, ur5.M, getCol(ur5.Slist, j))));
    expect(sOn).toEqual([true, true, true, true, true, true]);
    expect(bOn).toEqual([false, false, false, false, false, true]);
  });

  it('only θ1 moving: every B_i (glued to {b}) still lies on its joint', () => {
    const th = [50 * DEG, 0, 0, 0, 0, 0];
    const T = FKinSpace(ur5.M, ur5.Slist, th);
    const now = currentAxes(ur5.Slist, th);
    expect(now.every((S, j) => sameLine(S, gluedToB(T, ur5.M, getCol(ur5.Slist, j))))).toBe(true);
    expect(now.map((S, j) => sameLine(S, getCol(ur5.Slist, j)))).toEqual([true, false, false, false, false, false]);
  });

  it('during the body step-through, the next joint’s glued B axis coincides with the real axis', () => {
    const th = POSES[2].theta;
    for (let k = 0; k < 6; k++) {
      const part = th.map((x, i) => (i < k ? x : 0));
      const T = FKinSpace(ur5.M, ur5.Slist, part);
      const real = currentAxes(ur5.Slist, part)[k];
      expect(sameLine(real, gluedToB(T, ur5.M, getCol(ur5.Slist, k)), 1e-9)).toBe(true);
    }
  });
});

describe('tool offset', () => {
  it('changes M and only the v part of B_i with ω ∦ ẑ_b (B1, B5); the S_i never change', () => {
    const Mt = toolFrame(ur5.M, TOOL_LENGTH);
    const B0 = bodyAxes(ur5.Slist, ur5.M);
    const Bt = bodyAxes(ur5.Slist, Mt);
    // joints 2, 3, 4, 6 are parallel to ẑ_b, so ω × p = 0 for a tool along ẑ_b
    expect(changedColumns(B0, Bt)).toEqual([true, false, false, false, true, false]);
    for (let j = 0; j < 6; j++) expectVec(getCol(Bt, j).slice(0, 3), getCol(B0, j).slice(0, 3));
    expectVec(getCol(Bt, 0), [0, 1, 0, W1 + W2 + TOOL_LENGTH, 0, L1 + L2]);
    expectVec(getCol(Bt, 4), [0, -1, 0, -W2 - TOOL_LENGTH, 0, 0]);
    // same end pose either way for any θ
    for (const th of samples(3, 9)) {
      expect(transformGap(FKinBody(Mt, Bt, th), FKinSpace(Mt, ur5.Slist, th))).toBeLessThan(1e-12);
    }
    // the drawn tool ends at M_tool
    expectVec(TOOL_SEGMENT.b, [Mt[0][3], Mt[1][3], Mt[2][3]]);
  });
});

describe('drawing data', () => {
  it('anchors lie on the joint axes and the flange ends at M', () => {
    UR5_ANCHORS.forEach((a, j) => {
      const S = getCol(ur5.Slist, j);
      const w = S.slice(0, 3);
      const v = S.slice(3, 6);
      // a point p is on the axis when −ω × p = v
      const c = [-(w[1] * a[2] - w[2] * a[1]), -(w[2] * a[0] - w[0] * a[2]), -(w[0] * a[1] - w[1] * a[0])];
      expectVec(c, v);
    });
    const flange = UR5_SEGMENTS.filter((s) => s.body === 6 && s.role === 'link').pop()!;
    expectVec(flange.b, [ur5.M[0][3], ur5.M[1][3], ur5.M[2][3]]);
  });

  it('segments follow their bodies: joints stay connected for any θ', () => {
    for (const th of samples(3, 13)) {
      const L = chainTransforms(ur5.Slist, th);
      for (let j = 0; j < 6; j++) {
        const a = carry(L[j], UR5_ANCHORS[j]);
        const b = carry(L[j + 1], UR5_ANCHORS[j]);
        expect(Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2])).toBeLessThan(1e-12);
      }
      expect(transformGap(matMul(L[6], ur5.M), FKinSpace(ur5.M, ur5.Slist, th))).toBeLessThan(1e-12);
    }
  });
});

describe('tip velocity preview', () => {
  it('matches a finite difference of the forward kinematics', () => {
    const th = POSES[2].theta;
    const Mt = toolFrame(ur5.M, TOOL_LENGTH);
    const T = FKinSpace(Mt, ur5.Slist, th);
    const p = [T[0][3], T[1][3], T[2][3]];
    const h = 1e-6;
    for (let j = 0; j < 6; j++) {
      const th2 = th.slice();
      th2[j] += h;
      const T2 = FKinSpace(Mt, ur5.Slist, th2);
      const fd = [(T2[0][3] - p[0]) / h, (T2[1][3] - p[1]) / h, (T2[2][3] - p[2]) / h];
      expectVec(pointVelocity(ur5.Slist, th, j, p), fd, 5);
    }
  });
});
