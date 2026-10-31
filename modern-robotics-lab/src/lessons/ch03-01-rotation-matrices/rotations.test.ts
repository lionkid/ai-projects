import { describe, expect, it } from 'vitest';
import { MatrixExp3, VecToso3, approxEqual, det, eye, matMul, matVec, norm, RotX, RotY, RotZ, TestIfSO3, transpose, vecScale } from '@/mr';
import {
  composeSequence, dropLines, equivalentAxisAngle, invertPlanarPose, parallelepipedEdges, planarToBody, planarToSpace,
  randomRotation, rotationArc, sequenceAxisInSpace, so3Report, type ElemRot,
} from './rotations';

const deg = Math.PI / 180;

describe('composeSequence', () => {
  const A: ElemRot = { axis: 'z', angle: 90 * deg };
  const B: ElemRot = { axis: 'x', angle: 40 * deg };

  it('body axes post-multiply, fixed axes pre-multiply', () => {
    const R0 = RotY(25 * deg);
    expect(approxEqual(composeSequence(R0, [A, B], 'body'), matMul(matMul(R0, RotZ(A.angle)), RotX(B.angle)))).toBe(true);
    expect(approxEqual(composeSequence(R0, [A, B], 'fixed'), matMul(matMul(RotX(B.angle), RotZ(A.angle)), R0))).toBe(true);
  });

  it('fixed axes in one order equal body axes in the reverse order (start at I)', () => {
    const I = eye(3);
    expect(approxEqual(composeSequence(I, [A, B], 'fixed'), composeSequence(I, [B, A], 'body'))).toBe(true);
    // ...but the two modes differ for the same order: 3-D rotations do not commute
    expect(approxEqual(composeSequence(I, [A, B], 'fixed'), composeSequence(I, [A, B], 'body'), 1e-3)).toBe(false);
  });

  it('partial progress interpolates each rotation in turn', () => {
    const R0 = RotX(10 * deg);
    expect(approxEqual(composeSequence(R0, [A, B], 'body', 0), R0)).toBe(true);
    expect(approxEqual(composeSequence(R0, [A, B], 'body', 0.5), matMul(R0, RotZ(45 * deg)))).toBe(true);
    expect(approxEqual(composeSequence(R0, [A, B], 'body', 1.5), matMul(matMul(R0, RotZ(A.angle)), RotX(20 * deg)))).toBe(true);
  });

  it('while rotation k runs, the frame turns about the axis reported in {s}', () => {
    const R0 = RotY(-35 * deg);
    for (const mode of ['body', 'fixed'] as const) {
      for (const k of [0, 1]) {
        const axis = sequenceAxisInSpace(R0, [A, B], mode, k);
        expect(norm(axis)).toBeCloseTo(1, 12);
        const before = composeSequence(R0, [A, B], mode, k);
        const f = 0.6;
        const during = composeSequence(R0, [A, B], mode, k + f);
        const ang = [A, B][k].angle * f;
        // the motion so far is a rotation about the fixed-in-space axis: R(f) = Rot_s(axis, ang) R(0)
        const expected = matMul(MatrixExp3(VecToso3(vecScale(axis, ang))), before);
        expect(approxEqual(during, expected, 1e-9)).toBe(true);
      }
    }
  });
});

describe('so3Report', () => {
  it('accepts a rotation', () => {
    const r = so3Report(matMul(RotZ(0.3), RotX(-1.1)));
    expect(r.isRotation).toBe(true);
    expect(r.isReflection).toBe(false);
    expect(r.det).toBeCloseTo(1, 12);
    expect(r.orthoError).toBeLessThan(1e-12);
  });

  it('flags a column swap as a reflection (orthogonal, det = −1)', () => {
    const M = [[1, 0, 0], [0, 0, 1], [0, 1, 0]]; // columns x, z, y
    const r = so3Report(M);
    expect(r.orthoError).toBeLessThan(1e-12);
    expect(r.det).toBeCloseTo(-1, 12);
    expect(r.isRotation).toBe(false);
    expect(r.isReflection).toBe(true);
  });

  it('measures stretch and shear', () => {
    const stretched = so3Report([[1.5, 0, 0], [0, 1, 0], [0, 0, 1]]);
    expect(stretched.colNorms[0]).toBeCloseTo(1.5, 12);
    expect(stretched.isRotation).toBe(false);
    const sheared = so3Report([[1, 0.5, 0], [0, 1, 0], [0, 0, 1]]);
    expect(sheared.colDots[0]).toBeCloseTo(0.5, 12);
    expect(sheared.det).toBeCloseTo(1, 12); // volume preserved, but not a rotation
    expect(sheared.isRotation).toBe(false);
  });

  it('agrees with the library test for random rotations', () => {
    let seed = 7;
    const rand = () => { seed = (seed * 16807) % 2147483647; return seed / 2147483647; };
    for (let i = 0; i < 20; i++) {
      const R = randomRotation(rand);
      expect(TestIfSO3(R)).toBe(true);
      expect(so3Report(R).isRotation).toBe(true);
    }
  });
});

describe('geometry helpers', () => {
  it('parallelepiped has 12 edges spanned by the columns', () => {
    const edges = parallelepipedEdges([[2, 0, 0], [0, 1, 0], [0, 0, 1]]);
    expect(edges).toHaveLength(12);
    const lengths = edges.map(([a, b]) => Math.hypot(b[0] - a[0], b[1] - a[1], b[2] - a[2])).sort();
    expect(lengths.filter((L) => Math.abs(L - 2) < 1e-12)).toHaveLength(4);
    expect(lengths.filter((L) => Math.abs(L - 1) < 1e-12)).toHaveLength(8);
  });

  it('drop lines end on the coordinate axes', () => {
    const lines = dropLines([0.3, -0.4, 0.8]);
    expect(lines[1][1]).toEqual([0.3, 0, 0]);
    expect(lines[2][1]).toEqual([0, -0.4, 0]);
    expect(lines[3][1]).toEqual([0, 0, 0.8]);
  });

  it('rotation arc starts at v, ends at R v and keeps |v|', () => {
    const R = matMul(RotZ(1.2), RotY(0.7));
    const v = [0.4, -0.2, 0.9];
    const arc = rotationArc(R, v, 20);
    expect(approxEqual(arc[0], v, 1e-12)).toBe(true);
    expect(approxEqual(arc[20], matVec(R, v), 1e-9)).toBe(true);
    for (const p of arc) expect(norm(p)).toBeCloseTo(norm(v), 10);
  });

  it('equivalent axis-angle reproduces R and is undefined for I', () => {
    const R = matMul(RotX(0.5), RotZ(-0.9));
    const { axis, angle } = equivalentAxisAngle(R);
    expect(axis).not.toBeNull();
    expect(approxEqual(MatrixExp3(VecToso3(vecScale(axis!, angle))), R, 1e-9)).toBe(true);
    expect(equivalentAxisAngle(eye(3)).axis).toBeNull();
  });
});

describe('planar frames', () => {
  it('p_s = R_sb p_b + p_sb (quiz example)', () => {
    const pose = { x: 2, y: 0, theta: Math.PI / 2 };
    const ps = planarToSpace(pose, [1, 0]);
    expect(ps[0]).toBeCloseTo(2, 12);
    expect(ps[1]).toBeCloseTo(1, 12);
  });

  it('round trip between {s} and {b} coordinates', () => {
    const pose = { x: 1.6, y: 0.6, theta: 40 * deg };
    const ps = [2.2, 1.9];
    const pb = planarToBody(pose, ps);
    expect(approxEqual(planarToSpace(pose, pb), ps, 1e-12)).toBe(true);
    // the same point keeps its {s} coordinates while {b} moves, but its {b} coordinates change
    const pb2 = planarToBody({ ...pose, theta: pose.theta + 0.3 }, ps);
    expect(approxEqual(pb, pb2, 1e-3)).toBe(false);
  });

  it('inverse pose is R_bs = R_sbᵀ, p_bs = −R_sbᵀ p_sb', () => {
    const pose = { x: 1.6, y: 0.6, theta: 40 * deg };
    const inv = invertPlanarPose(pose);
    expect(inv.theta).toBeCloseTo(-pose.theta, 12);
    const c = Math.cos(pose.theta), s = Math.sin(pose.theta);
    expect(inv.x).toBeCloseTo(-(c * pose.x + s * pose.y), 12);
    expect(inv.y).toBeCloseTo(-(-s * pose.x + c * pose.y), 12);
    // composing a pose with its inverse returns every point to itself
    const p = [0.3, -0.7];
    expect(approxEqual(planarToSpace(inv, planarToSpace(pose, p)), p, 1e-12)).toBe(true);
  });
});

describe('sanity of the transpose-inverse rule', () => {
  it('R_bs = R_sbᵀ undoes R_sb', () => {
    const Rsb = matMul(RotZ(0.4), RotX(1.0));
    expect(approxEqual(matMul(transpose(Rsb), Rsb), eye(3), 1e-12)).toBe(true);
    expect(det(Rsb)).toBeCloseTo(1, 12);
  });
});
