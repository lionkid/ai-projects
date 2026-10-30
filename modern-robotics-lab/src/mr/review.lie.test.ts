/**
 * Independent adversarial review tests for so3.ts / se3.ts.
 *
 * Every expected value here is computed WITHOUT the function under test:
 * brute-force RK4 integration of the matrix ODE for the exponential, the
 * closed-form R = 2 w w^T - I for half turns, quaternion products for
 * rotations, and the defining identities of Ad and ad.
 */
import { describe, expect, it } from 'vitest';
import {
  type Mat,
  type Vec,
  Adjoint,
  AxisAng3,
  AxisAng6,
  MatrixExp3,
  MatrixExp6,
  MatrixLog3,
  MatrixLog6,
  ProjectToSO3,
  QuatToRot,
  RotToQuat,
  RotX,
  RotY,
  RotZ,
  RpToTrans,
  ScrewAxisToPointDir,
  ScrewToAxis,
  TransInv,
  VecTose3,
  VecToso3,
  ad,
  cross,
  det,
  dot,
  eulerZYX,
  eye,
  matAdd,
  matMul,
  matScale,
  matSub,
  matVec,
  norm,
  normalize,
  outer,
  rotZYX,
  rotationAngleBetween,
  se3ToVec,
  so3ToVec,
  transformPoint,
  transpose,
  vecAdd,
  vecScale,
  vecSub,
} from './index';
import { expectClose, maxDiff, mulberry32, randVec } from './testUtils';

const rng = mulberry32(20261030);

/** exp(A) by RK4 integration of X' = X A from X(0) = I (no Rodrigues formula involved). */
function matExpBrute(A: Mat, steps = 2000): Mat {
  let X = eye(A.length);
  const h = 1 / steps;
  const f = (Y: Mat) => matMul(Y, A);
  for (let k = 0; k < steps; k++) {
    const k1 = f(X);
    const k2 = f(matAdd(X, matScale(k1, h / 2)));
    const k3 = f(matAdd(X, matScale(k2, h / 2)));
    const k4 = f(matAdd(X, matScale(k3, h)));
    X = matAdd(X, matScale(matAdd(matAdd(k1, matScale(k2, 2)), matAdd(matScale(k3, 2), k4)), h / 6));
  }
  return X;
}

/** Rotation about unit axis w by theta built from the unit quaternion (cos t/2, sin t/2 w). */
function rotFromQuat(w: Vec, theta: number): Mat {
  const a = Math.cos(theta / 2);
  const [b, c, d] = vecScale(w, Math.sin(theta / 2));
  return [
    [a * a + b * b - c * c - d * d, 2 * (b * c - a * d), 2 * (b * d + a * c)],
    [2 * (b * c + a * d), a * a - b * b + c * c - d * d, 2 * (c * d - a * b)],
    [2 * (b * d - a * c), 2 * (c * d + a * b), a * a - b * b - c * c + d * d],
  ];
}

function randUnit(): Vec {
  return normalize(randVec(rng, 3, 1));
}

describe('review: so(3) conventions', () => {
  it('[w] v = w x v with the hand-written cross product, and so3ToVec inverts the hat', () => {
    for (let k = 0; k < 20; k++) {
      const w = randVec(rng, 3, 2);
      const v = randVec(rng, 3, 2);
      const byHand = [w[1] * v[2] - w[2] * v[1], w[2] * v[0] - w[0] * v[2], w[0] * v[1] - w[1] * v[0]];
      expectClose(matVec(VecToso3(w), v), byHand, 1e-14, 'hat action');
      expectClose(cross(w, v), byHand, 1e-14, 'cross');
      expectClose(so3ToVec(VecToso3(w)), w, 0, 'round trip');
      const W = VecToso3(w);
      expectClose(transpose(W), matScale(W, -1), 0, 'skew');
    }
  });

  it('elementary rotations are right-handed (x->y about z, y->z about x, z->x about y)', () => {
    expectClose(matVec(RotZ(Math.PI / 2), [1, 0, 0]), [0, 1, 0], 1e-15);
    expectClose(matVec(RotX(Math.PI / 2), [0, 1, 0]), [0, 0, 1], 1e-15);
    expectClose(matVec(RotY(Math.PI / 2), [0, 0, 1]), [1, 0, 0], 1e-15);
    expectClose(RotZ(0.7), MatrixExp3(VecToso3([0, 0, 0.7])), 1e-15);
    expectClose(RotX(-1.1), MatrixExp3(VecToso3([-1.1, 0, 0])), 1e-15);
    expectClose(RotY(2.3), MatrixExp3(VecToso3([0, 2.3, 0])), 1e-15);
  });

  it('MatrixExp3 agrees with brute-force ODE integration and with the quaternion formula', () => {
    for (const theta of [0.3, 1.7, 3.0, Math.PI, 4.5]) {
      for (let k = 0; k < 3; k++) {
        const w = randUnit();
        const W = VecToso3(vecScale(w, theta));
        const R = MatrixExp3(W);
        expectClose(R, matExpBrute(W), 1e-9, `ode theta=${theta}`);
        expectClose(R, rotFromQuat(w, theta), 1e-13, `quat theta=${theta}`);
      }
    }
  });

  it('AxisAng3 of the zero vector is safe and otherwise splits correctly', () => {
    const [w0, t0] = AxisAng3([0, 0, 0]);
    expect(t0).toBe(0);
    expect(w0.every((x) => Number.isFinite(x))).toBe(true);
    const [w, t] = AxisAng3([0, 3, 4]);
    expect(t).toBeCloseTo(5, 14);
    expectClose(w, [0, 0.6, 0.8], 1e-15);
  });
});

describe('review: MatrixLog3 branches', () => {
  it('exact half turns about non-axis-aligned directions (R = 2 w w^T - I)', () => {
    const dirs: Vec[] = [
      normalize([1, 2, 3]),
      normalize([1, -1, 0]),
      normalize([-0.2, 0.9, 0.4]),
      normalize([1, 1, 1]),
      normalize([0.001, 1, 0]),
      normalize([0, 0.7, -0.7]),
    ];
    for (const w of dirs) {
      const R = matSub(matScale(outer(w, w), 2), eye(3));
      const logR = so3ToVec(MatrixLog3(R));
      expect(norm(logR), `|log| for w=${w}`).toBeCloseTo(Math.PI, 10);
      // Parallel to w (either sign is a valid logarithm at pi).
      expect(Math.abs(dot(normalize(logR), w)), `axis for w=${w}`).toBeCloseTo(1, 10);
      expectClose(MatrixExp3(MatrixLog3(R)), R, 1e-12, `exp(log) w=${w}`);
    }
  });

  it('strictly below pi the logarithm is unique and must carry the + sign (both sides of the 1e-4 branch switch)', () => {
    const deltas = [1e-3, 2e-4, 1e-4 + 1e-9, 1e-4 - 1e-9, 5e-5, 1e-5, 1e-7, 1e-9, 1e-11];
    for (const d of deltas) {
      const theta = Math.PI - d;
      for (let k = 0; k < 4; k++) {
        const w = randUnit();
        const R = rotFromQuat(w, theta); // independent of MatrixExp3
        const logR = so3ToVec(MatrixLog3(R));
        expectClose(logR, vecScale(w, theta), 2e-8, `delta=${d} w=${w}`);
      }
    }
  });

  it('small angles keep full relative precision on the axis', () => {
    for (const theta of [1e-3, 1e-5, 1e-7, 1e-9]) {
      const w = randUnit();
      const R = rotFromQuat(w, theta);
      const logR = so3ToVec(MatrixLog3(R));
      expect(norm(vecSub(logR, vecScale(w, theta))) / theta, `theta=${theta}`).toBeLessThan(1e-6);
    }
  });

  it('is insensitive to the trace clamp: slightly non-orthonormal inputs near I and near pi do not produce NaN', () => {
    const noisyI = matAdd(eye(3), matScale([[1, 2, -1], [0, 1, 1], [2, -1, 0]], 1e-15));
    const logI = MatrixLog3(noisyI);
    expect(logI.flat().every((x) => Number.isFinite(x))).toBe(true);
    const w = normalize([1, 2, 3]);
    const Rpi = matAdd(matSub(matScale(outer(w, w), 2), eye(3)), matScale([[1, 0, 1], [0, 1, 0], [1, 0, 1]], 1e-15));
    const logPi = so3ToVec(MatrixLog3(Rpi));
    expect(logPi.every((x) => Number.isFinite(x))).toBe(true);
    expect(norm(logPi)).toBeCloseTo(Math.PI, 6);
  });

  it('rotationAngleBetween is symmetric and bounded by pi', () => {
    for (let k = 0; k < 10; k++) {
      const R1 = rotFromQuat(randUnit(), 3 * rng());
      const R2 = rotFromQuat(randUnit(), 3 * rng());
      const a = rotationAngleBetween(R1, R2);
      expect(a).toBeGreaterThanOrEqual(0);
      expect(a).toBeLessThanOrEqual(Math.PI + 1e-12);
      expect(rotationAngleBetween(R2, R1)).toBeCloseTo(a, 10);
    }
  });
});

describe('review: quaternions, Euler angles, projection', () => {
  it('RotToQuat(exp([w] theta)) = (cos theta/2, sin theta/2 w) for theta < pi', () => {
    for (const theta of [0.1, 1.2, 2.9, Math.PI - 1e-6]) {
      const w = randUnit();
      const q = RotToQuat(rotFromQuat(w, theta));
      expectClose(q, [Math.cos(theta / 2), ...vecScale(w, Math.sin(theta / 2))], 1e-9, `theta=${theta}`);
      expectClose(QuatToRot(q), rotFromQuat(w, theta), 1e-9);
    }
  });

  it('rotZYX matches the hand-expanded matrix and eulerZYX inverts it, including gimbal lock', () => {
    const [a, b, g] = [0.4, -0.9, 2.2];
    const [ca, sa, cb, sb, cg, sg] = [Math.cos(a), Math.sin(a), Math.cos(b), Math.sin(b), Math.cos(g), Math.sin(g)];
    const byHand: Mat = [
      [ca * cb, ca * sb * sg - sa * cg, ca * sb * cg + sa * sg],
      [sa * cb, sa * sb * sg + ca * cg, sa * sb * cg - ca * sg],
      [-sb, cb * sg, cb * cg],
    ];
    expectClose(rotZYX([a, b, g]), byHand, 1e-15);
    const back = eulerZYX(byHand);
    expect(back.gimbalLock).toBe(false);
    expectClose(back.angles, [a, b, g], 1e-12);
    for (const beta of [Math.PI / 2, -Math.PI / 2]) {
      const R = rotZYX([0.4, beta, 0.7]);
      const e = eulerZYX(R);
      expect(e.gimbalLock).toBe(true);
      expectClose(rotZYX(e.angles), R, 1e-12, `gimbal beta=${beta}`);
    }
  });

  it('ProjectToSO3 returns a proper rotation even for reflections and recovers a lightly perturbed rotation', () => {
    const R = rotFromQuat(randUnit(), 1.3);
    const refl = matMul(R, [[1, 0, 0], [0, 1, 0], [0, 0, -1]]);
    const P = ProjectToSO3(refl);
    expect(det(P)).toBeCloseTo(1, 10);
    expectClose(matMul(transpose(P), P), eye(3), 1e-12);
    const noisy = matAdd(R, matScale([[1, -2, 0.5], [0.3, 1, -1], [2, 0.1, -0.4]], 1e-4));
    const Q = ProjectToSO3(noisy);
    expect(det(Q)).toBeCloseTo(1, 10);
    expect(maxDiff(Q, R)).toBeLessThan(5e-4);
    expect(maxDiff(Q, noisy)).toBeLessThanOrEqual(maxDiff(R, noisy) + 1e-12);
  });
});

describe('review: se(3) conventions', () => {
  it('VecTose3 puts omega in the top-left block and v in the last column (twist order [omega; v])', () => {
    const V = [1, 2, 3, 4, 5, 6];
    const M = VecTose3(V);
    expectClose([M[0].slice(0, 3), M[1].slice(0, 3), M[2].slice(0, 3)], VecToso3([1, 2, 3]), 0);
    expectClose([M[0][3], M[1][3], M[2][3]], [4, 5, 6], 0);
    expectClose(M[3], [0, 0, 0, 0], 0);
    expectClose(se3ToVec(M), V, 0);
  });

  it('Adjoint satisfies its defining identity [Ad_T] V = (T [V] T^-1)^vee for random T, V', () => {
    for (let k = 0; k < 20; k++) {
      const T = RpToTrans(rotFromQuat(randUnit(), 3 * rng()), randVec(rng, 3, 2));
      const V = randVec(rng, 6, 2);
      const lhs = matVec(Adjoint(T), V);
      const rhs = se3ToVec(matMul(matMul(T, VecTose3(V)), TransInv(T)));
      expectClose(lhs, rhs, 1e-12);
    }
  });

  it('wrench transformation F_a = Ad_{T_ba}^T F_b conserves power', () => {
    for (let k = 0; k < 10; k++) {
      const Tab = RpToTrans(rotFromQuat(randUnit(), 2 * rng()), randVec(rng, 3, 1));
      const Vb = randVec(rng, 6, 1);
      const Fb = randVec(rng, 6, 1);
      const Va = matVec(Adjoint(Tab), Vb);
      const Fa = matVec(transpose(Adjoint(TransInv(Tab))), Fb);
      expect(dot(Va, Fa)).toBeCloseTo(dot(Vb, Fb), 10);
    }
  });

  it('ad satisfies [ad_V1] V2 = ([V1][V2] - [V2][V1])^vee and ad_V V = 0', () => {
    for (let k = 0; k < 20; k++) {
      const V1 = randVec(rng, 6, 2);
      const V2 = randVec(rng, 6, 2);
      const A = VecTose3(V1);
      const B = VecTose3(V2);
      expectClose(matVec(ad(V1), V2), se3ToVec(matSub(matMul(A, B), matMul(B, A))), 1e-12);
      expectClose(matVec(ad(V1), V1), [0, 0, 0, 0, 0, 0], 1e-12);
    }
  });

  it('ScrewToAxis: exp([S] theta) slides points on the axis by h theta and rotates by exp([s] theta)', () => {
    for (let k = 0; k < 10; k++) {
      const q = randVec(rng, 3, 2);
      const s = randUnit();
      const h = (rng() - 0.5) * 2;
      const theta = 0.2 + 2.5 * rng();
      const S = ScrewToAxis(q, s, h);
      expectClose(S.slice(0, 3), s, 0, 'omega = s');
      const T = MatrixExp6(VecTose3(vecScale(S, theta)));
      expectClose(transformPoint(T, q), vecAdd(q, vecScale(s, h * theta)), 1e-12, 'axis point');
      const q2 = vecAdd(q, vecScale(s, 1.7)); // another point on the axis
      expectClose(transformPoint(T, q2), vecAdd(q2, vecScale(s, h * theta)), 1e-12, 'second axis point');
      expectClose([T[0].slice(0, 3), T[1].slice(0, 3), T[2].slice(0, 3)], rotFromQuat(s, theta), 1e-12, 'rotation');
      // Inverse map returns the closest point, the direction and the pitch.
      const back = ScrewAxisToPointDir(S);
      expect(Math.abs(dot(back.q, s))).toBeLessThan(1e-12);
      expectClose(back.s, s, 1e-12);
      expect(back.h).toBeCloseTo(h, 12);
      expectClose(ScrewToAxis(back.q, back.s, back.h), S, 1e-12);
    }
  });

  it('MatrixExp6 agrees with brute-force integration of T\' = T [S] (rotation + pitch + pure translation)', () => {
    const cases: Vec[] = [
      [0, 0, 1, 1, 0, 0],
      [0, 0, 0, 0.3, -0.4, 1.2],
      ...Array.from({ length: 6 }, () => [...randUnit(), ...randVec(rng, 3, 1.5)]),
    ];
    for (const S of cases) {
      for (const theta of [0.5, 2.0, Math.PI]) {
        const A = VecTose3(vecScale(S, theta));
        expectClose(MatrixExp6(A), matExpBrute(A), 1e-9, `S=${S} theta=${theta}`);
      }
    }
  });

  it('MatrixLog6 recovers S theta for theta < pi from brute-force exponentials, and handles pure/near-pure translation', () => {
    for (let k = 0; k < 10; k++) {
      const w = randUnit();
      const theta = 0.1 + 2.9 * rng();
      const S = [...w, ...randVec(rng, 3, 1.5)];
      const A = VecTose3(vecScale(S, theta));
      expectClose(MatrixLog6(matExpBrute(A)), A, 1e-8, `k=${k}`);
    }
    // Pure translation.
    const Tp = RpToTrans(eye(3), [0.3, -1.2, 2.5]);
    expectClose(MatrixLog6(Tp), [[0, 0, 0, 0.3], [0, 0, 0, -1.2], [0, 0, 0, 2.5], [0, 0, 0, 0]], 0);
    const [Sp, tp] = AxisAng6(se3ToVec(MatrixLog6(Tp)));
    expect(tp).toBeCloseTo(norm([0.3, -1.2, 2.5]), 12);
    expectClose(Sp.slice(0, 3), [0, 0, 0], 0);
    // Rotation tiny compared with translation: the translation part must stay ~ p.
    for (const tiny of [1e-5, 1e-8, 1e-11, 1e-14]) {
      const w = normalize([1, 2, -1]);
      const T = RpToTrans(rotFromQuat(w, tiny), [1, 2, 3]);
      const L = MatrixLog6(T);
      expectClose(MatrixExp6(L), T, 1e-12, `exp(log) tiny=${tiny}`);
      const v = [L[0][3], L[1][3], L[2][3]];
      expect(norm(vecSub(v, [1, 2, 3])), `v ~ p tiny=${tiny}`).toBeLessThan(10 * tiny + 1e-12);
    }
  });

  it('MatrixLog6 at theta = pi is a valid logarithm (exp(log T) = T) for screws with and without pitch', () => {
    for (let k = 0; k < 8; k++) {
      const S = ScrewToAxis(randVec(rng, 3, 1), randUnit(), k % 2 === 0 ? 0 : 0.5 * rng());
      const T = MatrixExp6(VecTose3(vecScale(S, Math.PI)));
      const L = MatrixLog6(T);
      expectClose(MatrixExp6(L), T, 1e-10, `k=${k}`);
      expect(norm(so3ToVec([L[0].slice(0, 3), L[1].slice(0, 3), L[2].slice(0, 3)]))).toBeCloseTo(Math.PI, 8);
    }
  });

  it('TransInv equals the explicit [[R^T, -R^T p], [0, 1]] and T T^-1 = I', () => {
    for (let k = 0; k < 5; k++) {
      const R = rotFromQuat(randUnit(), 2 * rng());
      const p = randVec(rng, 3, 3);
      const T = RpToTrans(R, p);
      expectClose(TransInv(T), RpToTrans(transpose(R), vecScale(matVec(transpose(R), p), -1)), 1e-15);
      expectClose(matMul(T, TransInv(T)), eye(4), 1e-14);
    }
  });
});
