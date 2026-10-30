import { describe, expect, it } from 'vitest';
import { eye, inv, matMul, matVec, norm, normalize, vecScale, type Vec } from './linalg';
import { MatrixExp3, RotZ, VecToso3 } from './so3';
import {
  Adjoint,
  AxisAng6,
  DistanceToSE3,
  MatrixExp6,
  MatrixLog6,
  ProjectToSE3,
  RpToTrans,
  ScrewAxisToPointDir,
  ScrewToAxis,
  TestIfSE3,
  Trans,
  TransInv,
  TransToRp,
  VecTose3,
  ad,
  se3ToVec,
  transformPoint,
  transformVector,
} from './se3';
import { expectClose, mulberry32, randRot, randTrans, randVec } from './testUtils';

const rng = mulberry32(99);

describe('SE(3) basics', () => {
  it('RpToTrans / TransToRp / TransInv', () => {
    const R = randRot(rng);
    const p = [0.1, -2, 3];
    const T = RpToTrans(R, p);
    const [R2, p2] = TransToRp(T);
    expectClose(R2, R, 0);
    expectClose(p2, p, 0);
    expectClose(matMul(T, TransInv(T)), eye(4), 1e-12);
    expectClose(TransInv(T), inv(T), 1e-12);
  });

  it('VecTose3 / se3ToVec round trip', () => {
    const V = [1, 2, 3, 4, 5, 6];
    expectClose(se3ToVec(VecTose3(V)), V, 0);
    expectClose(VecTose3(V), [
      [0, -3, 2, 4],
      [3, 0, -1, 5],
      [-2, 1, 0, 6],
      [0, 0, 0, 0],
    ], 0);
  });

  it('transformPoint applies R p + t, transformVector only R', () => {
    const T = RpToTrans(RotZ(Math.PI / 2), [1, 2, 3]);
    expectClose(transformPoint(T, [1, 0, 0]), [1, 3, 3], 1e-15);
    expectClose(transformVector(T, [1, 0, 0]), [0, 1, 0], 1e-15);
    expectClose(Trans(1, 2, 3), RpToTrans(eye(3), [1, 2, 3]), 0);
  });
});

describe('Adjoint', () => {
  it('is a homomorphism: Ad(T1 T2) = Ad(T1) Ad(T2)', () => {
    for (let i = 0; i < 20; i++) {
      const T1 = randTrans(rng);
      const T2 = randTrans(rng);
      expectClose(Adjoint(matMul(T1, T2)), matMul(Adjoint(T1), Adjoint(T2)), 1e-12);
    }
  });

  it('Ad(T^-1) = Ad(T)^-1', () => {
    for (let i = 0; i < 20; i++) {
      const T = randTrans(rng);
      expectClose(Adjoint(TransInv(T)), inv(Adjoint(T)), 1e-10);
      expectClose(matMul(Adjoint(T), Adjoint(TransInv(T))), eye(6), 1e-12);
    }
  });

  it('has the block structure [[R, 0], [[p]R, R]]', () => {
    const R = randRot(rng);
    const p = [1, 2, 3];
    const A = Adjoint(RpToTrans(R, p));
    const pR = matMul(VecToso3(p), R);
    for (let i = 0; i < 3; i++) {
      for (let j = 0; j < 3; j++) {
        expect(A[i][j]).toBe(R[i][j]);
        expect(A[i][j + 3]).toBe(0);
        expect(A[i + 3][j]).toBeCloseTo(pR[i][j], 14);
        expect(A[i + 3][j + 3]).toBe(R[i][j]);
      }
    }
  });
});

describe('screws', () => {
  it('ScrewToAxis and its inverse', () => {
    const q = [1, 2, 0];
    const s = [0, 0, 1];
    const S = ScrewToAxis(q, s, 0.5);
    expectClose(S, [0, 0, 1, 2, -1, 0.5], 1e-15); // v = -s x q + h s = (2, -1, 0) + (0, 0, 0.5)
    const back = ScrewAxisToPointDir(S);
    expectClose(back.q, q, 1e-12); // q was already the closest point to the origin
    expectClose(back.s, s, 1e-12);
    expect(back.h).toBeCloseTo(0.5, 12);
    const pure = ScrewAxisToPointDir([0, 0, 0, 0, 3, 0]);
    expect(pure.h).toBe(Infinity);
    expectClose(pure.s, [0, 1, 0], 0);
  });

  it('AxisAng6 normalises by |omega| or |v|', () => {
    const [S, th] = AxisAng6([0, 0, 2, 0, 4, 0]);
    expect(th).toBeCloseTo(2, 15);
    expectClose(S, [0, 0, 1, 0, 2, 0], 1e-15);
    const [S2, th2] = AxisAng6([0, 0, 0, 0, 3, 4]);
    expect(th2).toBeCloseTo(5, 15);
    expectClose(S2, [0, 0, 0, 0, 0.6, 0.8], 1e-15);
  });
});

describe('MatrixExp6 / MatrixLog6', () => {
  it('matches an independently built screw motion about an offset axis', () => {
    const q = [1, 0, 0];
    const h = 0.2;
    for (const th of [0, 1e-8, 0.4, 2.0, Math.PI]) {
      const S = ScrewToAxis(q, [0, 0, 1], h);
      const T = MatrixExp6(VecTose3(vecScale(S, th)));
      const R = RotZ(th);
      // The point q stays on the axis, so p = (I - R) q + h theta * z.
      const p = [q[0] - (R[0][0] * q[0] + R[0][1] * q[1]), q[1] - (R[1][0] * q[0] + R[1][1] * q[1]), h * th];
      expectClose(T, RpToTrans(R, p), 1e-12, `theta=${th}`);
    }
  });

  it('pure translation', () => {
    const T = MatrixExp6(VecTose3([0, 0, 0, 1, 2, 3]));
    expectClose(T, Trans(1, 2, 3), 0);
    expectClose(se3ToVec(MatrixLog6(T)), [0, 0, 0, 1, 2, 3], 0);
  });

  it('round trips for theta near 0, near pi, at pi and random', () => {
    const omegas: Vec[] = [[0, 0, 1], [1, 0, 0], normalize([1, 1, 1]), normalize(randVec(rng, 3)), normalize(randVec(rng, 3))];
    const thetas = [1e-9, 1e-6, 1e-3, 0.5, 2, Math.PI - 1e-6, Math.PI - 1e-3, Math.PI];
    for (const w of omegas) {
      for (const th of thetas) {
        const v = randVec(rng, 3, 1);
        const S = [...w, ...v];
        const T = MatrixExp6(VecTose3(vecScale(S, th)));
        expect(TestIfSE3(T)).toBe(true);
        const back = se3ToVec(MatrixLog6(T));
        if (th < Math.PI - 1e-9) {
          expectClose(back, vecScale(S, th), 5e-8, `log(exp) w=${w} th=${th}`);
        }
        expectClose(MatrixExp6(MatrixLog6(T)), T, 1e-9, `exp(log) w=${w} th=${th}`);
      }
    }
    for (let i = 0; i < 40; i++) {
      const T = randTrans(rng);
      expectClose(MatrixExp6(MatrixLog6(T)), T, 1e-10);
      const V = randVec(rng, 6, 2);
      if (norm(V.slice(0, 3)) < Math.PI - 1e-3) {
        expectClose(se3ToVec(MatrixLog6(MatrixExp6(VecTose3(V)))), V, 1e-9);
      }
    }
  });

  it('exp(log(T)) is consistent with MatrixExp3 on the rotation block', () => {
    const T = randTrans(rng);
    const L = MatrixLog6(T);
    const [R] = TransToRp(T);
    expectClose(MatrixExp3([L[0].slice(0, 3), L[1].slice(0, 3), L[2].slice(0, 3)]), R, 1e-10);
  });
});

describe('ad (Lie bracket)', () => {
  it('is antisymmetric and matches the matrix commutator', () => {
    for (let i = 0; i < 10; i++) {
      const V1 = randVec(rng, 6, 2);
      const V2 = randVec(rng, 6, 2);
      expectClose(matVec(ad(V1), V2), vecScale(matVec(ad(V2), V1), -1), 1e-12);
      const A = VecTose3(V1);
      const B = VecTose3(V2);
      const commutator = matMul(A, B).map((row, r) => row.map((x, c) => x - matMul(B, A)[r][c]));
      expectClose(se3ToVec(commutator), matVec(ad(V1), V2), 1e-12);
    }
  });
});

describe('ProjectToSE3 / DistanceToSE3', () => {
  it('cleans a perturbed transform', () => {
    const T = randTrans(rng);
    const noisy = T.map((row, i) => row.map((x, j) => (i < 3 && j < 3 ? x + 0.01 * (2 * rng() - 1) : x)));
    expect(TestIfSE3(noisy)).toBe(false);
    const P = ProjectToSE3(noisy);
    expect(TestIfSE3(P)).toBe(true);
    expect(DistanceToSE3(P)).toBeLessThan(1e-10);
    expectClose(P, T, 0.05);
    expect(DistanceToSE3([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 2]])).toBeCloseTo(1, 12);
  });
});
