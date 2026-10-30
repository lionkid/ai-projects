import { describe, expect, it } from 'vitest';
import {
  block,
  cross,
  det,
  eigSym,
  eye,
  flatten,
  hstack,
  inv,
  matMul,
  matVec,
  normalize,
  pinv,
  reshape,
  solve,
  svd,
  transpose,
  vstack,
  wrapToPi,
  diag,
  cond,
  type Mat,
} from './linalg';
import { expectClose, mulberry32, randMat, randVec } from './testUtils';

describe('linalg basics', () => {
  it('multiplies, transposes and stacks', () => {
    const A: Mat = [
      [1, 2, 3],
      [4, 5, 6],
    ];
    const B: Mat = [
      [7, 8],
      [9, 10],
      [11, 12],
    ];
    expectClose(matMul(A, B), [
      [58, 64],
      [139, 154],
    ]);
    expectClose(transpose(A), [
      [1, 4],
      [2, 5],
      [3, 6],
    ]);
    expectClose(matVec(A, [1, 0, -1]), [-2, -2]);
    expectClose(hstack(eye(2), [[5], [6]]), [
      [1, 0, 5],
      [0, 1, 6],
    ]);
    expectClose(vstack(eye(2), [[5, 6]]), [
      [1, 0],
      [0, 1],
      [5, 6],
    ]);
    expectClose(
      block([
        [eye(2), [[0], [0]]],
        [[[7, 8]], [[9]]],
      ]),
      [
        [1, 0, 0],
        [0, 1, 0],
        [7, 8, 9],
      ],
    );
    expect(flatten(A)).toEqual([1, 2, 3, 4, 5, 6]);
    expectClose(reshape([1, 2, 3, 4, 5, 6], 2, 3), A);
    expect(() => matMul(A, A)).toThrow();
  });

  it('cross product and normalisation', () => {
    expectClose(cross([1, 0, 0], [0, 1, 0]), [0, 0, 1]);
    expectClose(normalize([3, 0, 4]), [0.6, 0, 0.8]);
    expectClose(normalize([0, 0, 0]), [0, 0, 0]);
  });

  it('wraps angles to [-pi, pi)', () => {
    expect(wrapToPi(0.5)).toBeCloseTo(0.5, 12);
    expect(wrapToPi(2 * Math.PI + 0.5)).toBeCloseTo(0.5, 12);
    expect(wrapToPi(-2 * Math.PI - 0.5)).toBeCloseTo(-0.5, 12);
    expect(wrapToPi(Math.PI)).toBeCloseTo(-Math.PI, 12);
    expect(wrapToPi(3 * Math.PI)).toBeCloseTo(-Math.PI, 12);
    expect(wrapToPi(-Math.PI)).toBeCloseTo(-Math.PI, 12);
  });
});

describe('det / inv / solve', () => {
  const rng = mulberry32(7);

  it('computes known determinants', () => {
    expect(det(diag([2, 3, 4]))).toBeCloseTo(24, 12);
    expect(
      det([
        [1, 2],
        [3, 4],
      ]),
    ).toBeCloseTo(-2, 12);
    expect(
      det([
        [0, 1, 2],
        [1, 0, 3],
        [4, -3, 8],
      ]),
    ).toBeCloseTo(-2, 12);
    expect(
      det([
        [1, 2],
        [2, 4],
      ]),
    ).toBeCloseTo(0, 12);
  });

  it('inverts random well-conditioned matrices', () => {
    for (let trial = 0; trial < 5; trial++) {
      const n = 3 + trial;
      const A = randMat(rng, n, n, 1);
      for (let i = 0; i < n; i++) A[i][i] += n; // diagonal dominance
      expectClose(matMul(A, inv(A)), eye(n), 1e-10);
      expectClose(matMul(inv(A), A), eye(n), 1e-10);
      const b = randVec(rng, n, 1);
      expectClose(solve(A, b), matVec(inv(A), b), 1e-10);
      expectClose(matVec(A, solve(A, b)), b, 1e-10);
    }
  });

  it('throws on singular matrices', () => {
    const S: Mat = [
      [1, 2, 3],
      [2, 4, 6],
      [1, 1, 1],
    ];
    expect(() => inv(S)).toThrow(/singular/);
    expect(() => solve(S, [1, 2, 3])).toThrow(/singular/);
    expect(() => inv([[0]])).toThrow();
  });
});

describe('svd / pinv', () => {
  const rng = mulberry32(11);

  it('matches a known diagonal case and sorts singular values', () => {
    const { U, S, V } = svd([
      [3, 0],
      [0, 4],
    ]);
    expectClose(S, [4, 3], 1e-12);
    expectClose(matMul(matMul(U, diag(S)), transpose(V)), [
      [3, 0],
      [0, 4],
    ], 1e-12);
  });

  it('reconstructs tall, wide and rank-deficient matrices with orthonormal factors', () => {
    const cases: Mat[] = [
      randMat(rng, 5, 3),
      randMat(rng, 3, 5),
      randMat(rng, 4, 4),
      [
        [1, 2],
        [2, 4],
      ],
      [
        [1, 2, 3],
        [2, 4, 6],
        [3, 6, 9],
        [0, 0, 0],
      ],
    ];
    for (const A of cases) {
      const { U, S, V } = svd(A);
      const r = S.length;
      expectClose(matMul(matMul(U, diag(S)), transpose(V)), A, 1e-10, 'reconstruction');
      expectClose(matMul(transpose(U), U), eye(r), 1e-10, 'U orthonormal');
      expectClose(matMul(transpose(V), V), eye(r), 1e-10, 'V orthonormal');
      for (let i = 1; i < r; i++) expect(S[i]).toBeLessThanOrEqual(S[i - 1] + 1e-15);
      for (const s of S) expect(s).toBeGreaterThanOrEqual(0);
    }
    const rankOne = svd([
      [1, 2],
      [2, 4],
    ]);
    expectClose(rankOne.S, [5, 0], 1e-10);
  });

  it('pinv satisfies the Moore-Penrose identities', () => {
    const cases: Mat[] = [
      randMat(rng, 6, 3),
      randMat(rng, 3, 6),
      [
        [1, 2],
        [2, 4],
      ],
    ];
    for (const A of cases) {
      const P = pinv(A);
      expectClose(matMul(matMul(A, P), A), A, 1e-9, 'A A+ A');
      expectClose(matMul(matMul(P, A), P), P, 1e-9, 'A+ A A+');
      expectClose(transpose(matMul(A, P)), matMul(A, P), 1e-9, '(A A+) symmetric');
      expectClose(transpose(matMul(P, A)), matMul(P, A), 1e-9, '(A+ A) symmetric');
    }
    expectClose(
      pinv([
        [1, 2],
        [2, 4],
      ]),
      [
        [0.04, 0.08],
        [0.08, 0.16],
      ],
      1e-12,
    );
    const A = randMat(rng, 4, 4);
    for (let i = 0; i < 4; i++) A[i][i] += 4;
    expectClose(pinv(A), inv(A), 1e-10);
    expect(cond(eye(3))).toBeCloseTo(1, 12);
    expect(cond([[1, 2], [2, 4]])).toBe(Infinity);
  });
});

describe('eigSym', () => {
  const rng = mulberry32(5);

  it('solves a known 2x2 problem', () => {
    const { values, vectors } = eigSym([
      [2, 1],
      [1, 2],
    ]);
    expectClose(values, [3, 1], 1e-12);
    const s = Math.SQRT1_2;
    // Eigenvectors are determined up to sign.
    expect(Math.abs(vectors[0][0] * s + vectors[1][0] * s)).toBeCloseTo(1, 10);
    expect(Math.abs(vectors[0][1] * s - vectors[1][1] * s)).toBeCloseTo(1, 10);
  });

  it('reconstructs random symmetric matrices with orthonormal eigenvectors', () => {
    for (let trial = 0; trial < 4; trial++) {
      const n = 3 + trial;
      const B = randMat(rng, n, n);
      const A = matMul(transpose(B), B);
      const { values, vectors } = eigSym(A);
      expectClose(matMul(matMul(vectors, diag(values)), transpose(vectors)), A, 1e-9, 'V diag V^T');
      expectClose(matMul(transpose(vectors), vectors), eye(n), 1e-10, 'V orthonormal');
      for (let k = 0; k < n; k++) {
        const v = vectors.map((row) => row[k]);
        expectClose(matVec(A, v), v.map((x) => x * values[k]), 1e-9, 'A v = lambda v');
        expect(values[k]).toBeGreaterThanOrEqual(-1e-12);
        if (k > 0) expect(values[k]).toBeLessThanOrEqual(values[k - 1] + 1e-12);
      }
    }
  });
});
