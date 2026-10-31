import { describe, expect, it } from 'vitest';
import { matVec, pinv } from '@/mr';
import {
  Hmatrix, Hphi, bodyFromQdot, chassisFromWheels, clampTwist, contactVelocity, decomposeContact, expTwist2,
  fourMecanum, isotropicSpeed, leftNullBasis, maxWheelSpeed, omegaLimit, qdotFromBody, rankOf, rayLimit, sliceAt,
  sliceUnbounded, stepPose, threeOmni, wheelGeometry, wheelRow, type Twist2, type Wheel,
} from './wheels';

const close = (a: number, b: number, tol = 1e-9) => expect(Math.abs(a - b)).toBeLessThan(tol);
const closeVec = (a: ArrayLike<number>, b: ArrayLike<number>, tol = 1e-9) => {
  expect(a.length).toBe(b.length);
  for (let i = 0; i < a.length; i++) close(a[i], b[i], tol);
};

/** Small deterministic PRNG so the randomised checks are reproducible. */
function rng(seed: number): () => number {
  let s = seed >>> 0;
  return () => {
    s = (s + 0x6d2b79f5) >>> 0;
    let t = s;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

describe('H(0) rows', () => {
  it('reproduces the standard three-omniwheel matrix', () => {
    const d = 0.22, r = 0.07;
    const H = Hmatrix(threeOmni(d, r));
    const s3 = Math.sqrt(3) / 2;
    const expected = [[-d, 1, 0], [-d, -0.5, -s3], [-d, -0.5, s3]].map((row) => row.map((x) => x / r));
    H.forEach((row, i) => closeVec(row, expected[i], 1e-12));
  });

  it('reproduces the standard four-mecanum matrix', () => {
    const l = 0.25, w = 0.22, r = 0.07, a = l + w;
    const H = Hmatrix(fourMecanum(l, w, r, Math.PI / 4));
    const expected = [[-a, 1, -1], [a, 1, 1], [a, 1, -1], [-a, 1, 1]].map((row) => row.map((x) => x / r));
    H.forEach((row, i) => closeVec(row, expected[i], 1e-12));
  });

  it('agrees with a direct rolling + roller-sliding decomposition of the contact velocity', () => {
    const rand = rng(7);
    for (let k = 0; k < 200; k++) {
      const w: Wheel = {
        x: rand() * 2 - 1, y: rand() * 2 - 1, beta: (rand() * 2 - 1) * Math.PI,
        gamma: (rand() * 2 - 1) * 1.2, r: 0.03 + rand() * 0.1,
      };
      const V: Twist2 = [rand() * 4 - 2, rand() * 2 - 1, rand() * 2 - 1];
      const h = wheelRow(w);
      const u = h[0] * V[0] + h[1] * V[1] + h[2] * V[2];
      const { u: u2 } = decomposeContact(w, contactVelocity(w, V));
      close(u, u2, 1e-9 * Math.max(1, Math.abs(u)));
    }
  });

  it('has a free-sliding direction perpendicular to the direction the wheel must supply', () => {
    const g = wheelGeometry({ x: 0.3, y: -0.1, beta: 0.4, gamma: -0.6, r: 0.05 });
    close(g.dir[0] * g.slide[0] + g.dir[1] * g.slide[1], 0, 1e-12);
    // a pure roller slide needs no wheel rotation
    const { u, s } = decomposeContact({ x: 0.3, y: -0.1, beta: 0.4, gamma: -0.6, r: 0.05 }, g.slide);
    close(u, 0, 1e-12);
    close(s, 1, 1e-12);
  });

  it('H(φ) q̇ equals H(0) V_b', () => {
    const H0 = Hmatrix(fourMecanum(0.25, 0.22, 0.07, Math.PI / 4));
    const phi = 0.9;
    const V: Twist2 = [0.4, 0.3, -0.2];
    const qd = qdotFromBody(phi, V);
    closeVec(matVec(Hphi(H0, phi), qd), matVec(H0, V), 1e-12);
    closeVec(bodyFromQdot(phi, qd), V, 1e-12);
  });
});

describe('rank and inconsistent wheel speeds', () => {
  it('three omniwheels: square, full rank, no inconsistent combination', () => {
    const H = Hmatrix(threeOmni(0.22, 0.07));
    expect(rankOf(H)).toBe(3);
    expect(leftNullBasis(H)).toHaveLength(0);
  });

  it('four mecanum wheels: one inconsistent combination (front pair against rear pair)', () => {
    const H = Hmatrix(fourMecanum(0.25, 0.22, 0.07, Math.PI / 4));
    expect(rankOf(H)).toBe(3);
    const N = leftNullBasis(H);
    expect(N).toHaveLength(1);
    closeVec(N[0], [0.5, 0.5, -0.5, -0.5], 1e-9);
    // Hᵀ n = 0
    for (let c = 0; c < 3; c++) close(H.reduce((acc, row, i) => acc + row[c] * N[0][i], 0), 0, 1e-9);
  });

  it('roller angle 0 removes the sideways column (rank 2)', () => {
    const H = Hmatrix(fourMecanum(0.25, 0.22, 0.07, 0));
    for (const row of H) close(row[2], 0, 1e-12);
    expect(rankOf(H)).toBe(2);
    expect(leftNullBasis(H)).toHaveLength(2);
    expect(sliceUnbounded(H)).toBe(true);
    expect(sliceUnbounded(Hmatrix(fourMecanum(0.25, 0.22, 0.07, Math.PI / 4)))).toBe(false);
  });

  it('least-squares chassis twist: exact for consistent speeds, zero motion for the null combination', () => {
    const H = Hmatrix(fourMecanum(0.25, 0.22, 0.07, Math.PI / 4));
    const Hp = pinv(H);
    const V: Twist2 = [0.3, 0.2, -0.4];
    const fit = chassisFromWheels(H, Hp, matVec(H, V));
    closeVec(fit.V, V, 1e-9);
    close(fit.slipNorm, 0, 1e-9);
    const bad = chassisFromWheels(H, Hp, [5, 5, -5, -5]);
    closeVec(bad.V, [0, 0, 0], 1e-9);
    closeVec(bad.slip, [5, 5, -5, -5], 1e-9);
    // driving a single mecanum wheel cannot be consistent
    expect(chassisFromWheels(H, Hp, [4, 0, 0, 0]).slipNorm).toBeGreaterThan(1);
    // ... but with three omniwheels it always is
    const H3 = Hmatrix(threeOmni(0.22, 0.07));
    close(chassisFromWheels(H3, pinv(H3), [4, 0, 0]).slipNorm, 0, 1e-9);
  });

  it('pure sideways motion: diagonal mecanum wheels turn the same way', () => {
    const u = matVec(Hmatrix(fourMecanum(0.25, 0.22, 0.07, Math.PI / 4)), [0, 0, 0.5]);
    expect(Math.sign(u[0])).toBe(Math.sign(u[2]));
    expect(Math.sign(u[1])).toBe(Math.sign(u[3]));
    expect(Math.sign(u[0])).toBe(-Math.sign(u[1]));
  });
});

describe('wheel-speed limits', () => {
  const U = 10, r = 0.07;
  it('mecanum slice at ω = 0 is a diamond with vertices at ±U r on both axes', () => {
    const H = Hmatrix(fourMecanum(0.25, 0.22, r, Math.PI / 4));
    const poly = sliceAt(H, U, 0, 5);
    const radii = poly.map((p) => Math.hypot(p[0], p[1]));
    close(Math.max(...radii), U * r, 1e-9);
    close(rayLimit(H, U, 0, [1, 0]), U * r, 1e-9);
    close(rayLimit(H, U, 0, [0, 1]), U * r, 1e-9);
    close(rayLimit(H, U, 0, [Math.SQRT1_2, Math.SQRT1_2]), (U * r) / Math.SQRT2, 1e-9);
    close(isotropicSpeed(H, U, 0), (U * r) / Math.SQRT2, 1e-9);
  });

  it('three-omni slice at ω = 0 is a regular hexagon: slowest along a wheel, fastest across it', () => {
    const H = Hmatrix(threeOmni(0.22, r));
    const poly = sliceAt(H, U, 0, 5);
    expect(poly.length).toBe(6);
    close(rayLimit(H, U, 0, [1, 0]), U * r, 1e-9);
    close(rayLimit(H, U, 0, [0, 1]), (U * r) / Math.cos(Math.PI / 6), 1e-9);
    close(isotropicSpeed(H, U, 0), U * r, 1e-9);
  });

  it('spinning shrinks the slice; the pure-spin limit empties it', () => {
    const H = Hmatrix(fourMecanum(0.25, 0.22, r, Math.PI / 4));
    const wMax = omegaLimit(H, U);
    close(wMax, (U * r) / 0.47, 1e-9);
    close(isotropicSpeed(H, U, 0.5 * wMax), 0.5 * isotropicSpeed(H, U, 0), 1e-9);
    expect(sliceAt(H, U, 1.01 * wMax, 5)).toHaveLength(0);
  });

  it('clamping keeps the heading of the request and lands on the boundary', () => {
    const H = Hmatrix(threeOmni(0.22, r));
    const req: Twist2 = [0.8, 2, 1];
    const c = clampTwist(H, U, req);
    expect(c.clamped).toBe(true);
    close(c.V[0], 0.8);
    close(c.V[1] / c.V[2], 2, 1e-9);
    close(maxWheelSpeed(H, c.V), U, 1e-9);
    const ok = clampTwist(H, U, [0.1, 0.1, 0.05]);
    expect(ok.clamped).toBe(false);
    closeVec(ok.V, [0.1, 0.1, 0.05]);
    const spin = clampTwist(H, U, [50, 0, 0]);
    expect(spin.omegaClamped).toBe(true);
    expect(maxWheelSpeed(H, spin.V)).toBeLessThanOrEqual(U + 1e-9);
  });
});

describe('integration on SE(2)', () => {
  it('expTwist2 (MatrixExp6) matches the closed-form arc', () => {
    const V: Twist2 = [0.7, 0.4, -0.25];
    const t = 1.3, th = V[0] * t;
    const d = expTwist2(V, t);
    close(d.x, (V[1] * Math.sin(th) + V[2] * (Math.cos(th) - 1)) / V[0], 1e-12);
    close(d.y, (V[1] * (1 - Math.cos(th)) + V[2] * Math.sin(th)) / V[0], 1e-12);
    const straight = expTwist2([0, 0.4, -0.25], 2);
    closeVec([straight.x, straight.y, straight.phi], [0.8, -0.5, 0], 1e-12);
  });

  it('a constant twist is integrated exactly: one big step equals many small ones', () => {
    const q0 = { phi: 0.3, x: -1, y: 0.5 };
    const V: Twist2 = [1.1, 0.3, 0.2];
    const big = stepPose(q0, V, 2);
    let q = q0;
    for (let k = 0; k < 400; k++) q = stepPose(q, V, 2 / 400);
    closeVec([q.phi, q.x, q.y], [big.phi, big.x, big.y], 1e-10);
  });

  it('a full turn with a constant twist returns to the start', () => {
    const q0 = { phi: -0.4, x: 0.2, y: 0.1 };
    const V: Twist2 = [0.8, 0.5, -0.3];
    const q = stepPose(q0, V, (2 * Math.PI) / 0.8);
    closeVec([q.x, q.y], [q0.x, q0.y], 1e-9);
    close(q.phi - q0.phi, 2 * Math.PI, 1e-12);
  });
});
