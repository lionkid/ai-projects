/**
 * Helpers shared by the unit tests (not part of the public API; not exported
 * from index.ts). Deterministic pseudo-random numbers keep the tests
 * reproducible.
 */
import { expect } from 'vitest';
import type { Mat, Vec } from './linalg';
import { MatrixExp3, VecToso3 } from './so3';
import { RpToTrans } from './se3';

/** Small, fast seeded PRNG returning uniforms in [0, 1). */
export function mulberry32(seed: number): () => number {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

/** Uniform random vector with entries in [-scale, scale]. */
export function randVec(rng: () => number, n: number, scale = 1): Vec {
  const v: Vec = new Array(n);
  for (let i = 0; i < n; i++) v[i] = (2 * rng() - 1) * scale;
  return v;
}

export function randMat(rng: () => number, rows: number, cols: number, scale = 1): Mat {
  const A: Mat = [];
  for (let i = 0; i < rows; i++) A.push(randVec(rng, cols, scale));
  return A;
}

/** Random rotation via the exponential map of a random axis-angle vector. */
export function randRot(rng: () => number): Mat {
  return MatrixExp3(VecToso3(randVec(rng, 3, 2.5)));
}

/** Random rigid-body transform with translation entries in [-1, 1] m. */
export function randTrans(rng: () => number): Mat {
  return RpToTrans(randRot(rng), randVec(rng, 3, 1));
}

/** Largest absolute entry-wise difference between two vectors or matrices. */
export function maxDiff(a: Vec | Mat, b: Vec | Mat): number {
  if (a.length !== b.length) return Infinity;
  let m = 0;
  for (let i = 0; i < a.length; i++) {
    const x = a[i];
    const y = b[i];
    if (Array.isArray(x) && Array.isArray(y)) m = Math.max(m, maxDiff(x, y));
    else if (typeof x === 'number' && typeof y === 'number') m = Math.max(m, Math.abs(x - y));
    else return Infinity;
  }
  return m;
}

/** Asserts entry-wise closeness and reports the worst entry on failure. */
export function expectClose(actual: Vec | Mat, expected: Vec | Mat, tol = 1e-9, label = ''): void {
  const d = maxDiff(actual, expected);
  if (!(d <= tol)) {
    // eslint-disable-next-line no-console
    console.error(`${label} max |diff| = ${d} > ${tol}\nactual  = ${JSON.stringify(actual)}\nexpected= ${JSON.stringify(expected)}`);
  }
  expect(d, `${label} max |diff| = ${d}`).toBeLessThanOrEqual(tol);
}
