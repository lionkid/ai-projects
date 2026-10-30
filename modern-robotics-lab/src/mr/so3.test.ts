import { describe, expect, it } from 'vitest';
import { det, eye, matMul, norm, normalize, transpose, vecScale, type Vec } from './linalg';
import {
  AxisAng3,
  DistanceToSO3,
  MatrixExp3,
  MatrixLog3,
  ProjectToSO3,
  QuatToRot,
  RotInv,
  RotToQuat,
  RotX,
  RotY,
  RotZ,
  TestIfSO3,
  VecToso3,
  eulerZYX,
  rotZYX,
  rotationAngleBetween,
  so3ToVec,
} from './so3';
import { expectClose, mulberry32, randRot, randVec } from './testUtils';

const rng = mulberry32(2024);
const axes: Vec[] = [
  [1, 0, 0],
  [0, 1, 0],
  [0, 0, 1],
  normalize([1, 1, 0]),
  normalize([1, -1, 1]),
  normalize([0.2, -0.7, 0.4]),
  ...Array.from({ length: 5 }, () => normalize(randVec(rng, 3))),
];
const thetas = [1e-9, 1e-7, 1e-5, 1e-3, 0.3, 1.0, 2.0, 3.0, Math.PI - 1e-3, Math.PI - 1e-7, Math.PI];

describe('so(3) basics', () => {
  it('VecToso3 / so3ToVec round trip and skew action', () => {
    const w = [0.3, -1.2, 2.5];
    const W = VecToso3(w);
    expectClose(so3ToVec(W), w, 0);
    expectClose(transpose(W), W.map((r) => r.map((x) => -x)), 0);
    const v = [1, 2, 3];
    const Wv = [W[0][0] * v[0] + W[0][1] * v[1] + W[0][2] * v[2], W[1][0] * v[0] + W[1][1] * v[1] + W[1][2] * v[2], W[2][0] * v[0] + W[2][1] * v[1] + W[2][2] * v[2]];
    expectClose(Wv, [w[1] * v[2] - w[2] * v[1], w[2] * v[0] - w[0] * v[2], w[0] * v[1] - w[1] * v[0]], 1e-15);
  });

  it('AxisAng3 splits exponential coordinates', () => {
    const [axis, theta] = AxisAng3([0, 3, 4]);
    expectClose(axis, [0, 0.6, 0.8], 1e-15);
    expect(theta).toBeCloseTo(5, 15);
    expect(AxisAng3([0, 0, 0])[1]).toBe(0);
  });
});

describe('MatrixExp3', () => {
  it('produces orthonormal matrices with det +1', () => {
    for (let i = 0; i < 20; i++) {
      const R = MatrixExp3(VecToso3(randVec(rng, 3, 4)));
      expectClose(matMul(transpose(R), R), eye(3), 1e-12);
      expect(det(R)).toBeCloseTo(1, 12);
      expectClose(RotInv(R), transpose(R), 0);
    }
  });

  it('agrees with the elementary rotations', () => {
    for (const th of [-2, -0.5, 0, 0.7, 3]) {
      expectClose(MatrixExp3(VecToso3([th, 0, 0])), RotX(th), 1e-14);
      expectClose(MatrixExp3(VecToso3([0, th, 0])), RotY(th), 1e-14);
      expectClose(MatrixExp3(VecToso3([0, 0, th])), RotZ(th), 1e-14);
    }
    expectClose(MatrixExp3(VecToso3([0, 0, 0])), eye(3), 0);
  });
});

describe('MatrixLog3 <-> MatrixExp3 round trips', () => {
  it('log(I) = 0', () => {
    expectClose(MatrixLog3(eye(3)), [[0, 0, 0], [0, 0, 0], [0, 0, 0]], 0);
  });

  it('recovers exponential coordinates for theta in (0, pi) on many axes incl. near 0 and near pi', () => {
    for (const w of axes) {
      for (const th of thetas) {
        const R = MatrixExp3(VecToso3(vecScale(w, th)));
        const logR = so3ToVec(MatrixLog3(R));
        const thetaBack = norm(logR);
        expect(Math.abs(thetaBack - th), `theta axis=${w} th=${th}`).toBeLessThanOrEqual(1e-8);
        if (th < Math.PI - 1e-9) {
          expectClose(logR, vecScale(w, th), 2e-8, `axis=${w} theta=${th}`);
        } else {
          // At pi both +w and -w are valid logarithms.
          const agree = Math.min(norm(logR.map((x, i) => x - w[i] * th)), norm(logR.map((x, i) => x + w[i] * th)));
          expect(agree).toBeLessThanOrEqual(1e-8);
        }
        expectClose(MatrixExp3(MatrixLog3(R)), R, 1e-9, `exp(log) axis=${w} theta=${th}`);
      }
    }
  });

  it('exp(log(R)) = R for random rotations', () => {
    for (let i = 0; i < 50; i++) {
      const R = randRot(rng);
      expectClose(MatrixExp3(MatrixLog3(R)), R, 1e-11);
    }
  });

  it('handles exact theta = pi rotation matrices (diagonal cases)', () => {
    const cases = [
      [[1, 0, 0], [0, -1, 0], [0, 0, -1]],
      [[-1, 0, 0], [0, 1, 0], [0, 0, -1]],
      [[-1, 0, 0], [0, -1, 0], [0, 0, 1]],
    ];
    for (const R of cases) {
      const logR = so3ToVec(MatrixLog3(R));
      expect(norm(logR)).toBeCloseTo(Math.PI, 10);
      expectClose(MatrixExp3(MatrixLog3(R)), R, 1e-12);
    }
  });

  it('rotationAngleBetween is the geodesic angle', () => {
    expect(rotationAngleBetween(RotZ(0.3), RotZ(1.1))).toBeCloseTo(0.8, 10);
  });
});

describe('ProjectToSO3 / TestIfSO3', () => {
  it('projects a perturbed rotation back onto SO(3)', () => {
    for (let i = 0; i < 10; i++) {
      const R = randRot(rng);
      const noisy = R.map((row) => row.map((x) => x + 0.01 * (2 * rng() - 1)));
      const P = ProjectToSO3(noisy);
      expect(TestIfSO3(P)).toBe(true);
      expect(DistanceToSO3(P)).toBeLessThan(1e-10);
      expectClose(P, R, 0.05);
      expectClose(ProjectToSO3(R), R, 1e-10);
    }
    expect(TestIfSO3([[1, 0, 0], [0, 1, 0], [0, 0, -1]])).toBe(false);
    expect(DistanceToSO3([[1, 0, 0], [0, 1, 0], [0, 0, -1]])).toBeGreaterThan(1);
    expect(TestIfSO3([[1.1, 0, 0], [0, 1, 0], [0, 0, 1]])).toBe(false);
  });
});

describe('Euler angles and quaternions', () => {
  it('rotZYX / eulerZYX round trip', () => {
    for (let i = 0; i < 30; i++) {
      const e = [(2 * rng() - 1) * Math.PI, (2 * rng() - 1) * 1.5, (2 * rng() - 1) * Math.PI];
      const R = rotZYX(e);
      const { angles, gimbalLock } = eulerZYX(R);
      expect(gimbalLock).toBe(false);
      expectClose(angles, e, 1e-10);
      expectClose(rotZYX(angles), R, 1e-12);
    }
    expectClose(rotZYX([0.3, 0, 0]), RotZ(0.3), 1e-15);
    const Rlock = rotZYX([0.4, Math.PI / 2, 0.9]);
    const res = eulerZYX(Rlock);
    expect(res.gimbalLock).toBe(true);
    expectClose(rotZYX(res.angles), Rlock, 1e-10);
  });

  it('RotToQuat / QuatToRot round trip including half-turns', () => {
    const cases = [...Array.from({ length: 20 }, () => randRot(rng)), RotX(Math.PI), RotY(Math.PI), RotZ(Math.PI), eye(3)];
    for (const R of cases) {
      const q = RotToQuat(R);
      expect(norm(q)).toBeCloseTo(1, 12);
      expect(q[0]).toBeGreaterThanOrEqual(0);
      expectClose(QuatToRot(q), R, 1e-10);
    }
    expectClose(QuatToRot([Math.SQRT1_2, 0, 0, Math.SQRT1_2]), RotZ(Math.PI / 2), 1e-12);
  });
});
