/**
 * Dense linear algebra on plain JavaScript arrays.
 *
 * Conventions
 *  - `Mat` is `number[][]`, row-major: `A[i][j]` is row i, column j.
 *  - `Vec` is `number[]`.
 *  - Every function is pure (arguments are never mutated) unless the doc
 *    comment says otherwise (`setCol`).
 *  - Dimension mismatches throw an `Error` with a readable message so that
 *    lesson code fails loudly instead of silently producing NaN.
 *
 * The routines are written for the small matrices that appear in robot
 * kinematics and dynamics (3x3, 4x4, 6xn, nxn with n <= ~10). They favour
 * clarity and numerical robustness over raw speed.
 */

export type Vec = number[];
export type Mat = number[][];

// ---------------------------------------------------------------------------
// Construction / shape helpers
// ---------------------------------------------------------------------------

/** rows x cols matrix of zeros (square when `cols` is omitted). */
export function zeros(rows: number, cols: number = rows): Mat {
  const A: Mat = new Array(rows);
  for (let i = 0; i < rows; i++) A[i] = new Array(cols).fill(0);
  return A;
}

/** Zero vector of length n. */
export function zerosVec(n: number): Vec {
  return new Array(n).fill(0);
}

/** n x n identity matrix. */
export function eye(n: number): Mat {
  const A = zeros(n, n);
  for (let i = 0; i < n; i++) A[i][i] = 1;
  return A;
}

/** Square diagonal matrix built from the entries of `d`. */
export function diag(d: Vec): Mat {
  const A = zeros(d.length, d.length);
  for (let i = 0; i < d.length; i++) A[i][i] = d[i];
  return A;
}

/** Diagonal entries of a (not necessarily square) matrix. */
export function diagOf(A: Mat): Vec {
  const n = Math.min(A.length, A[0]?.length ?? 0);
  const d: Vec = new Array(n);
  for (let i = 0; i < n; i++) d[i] = A[i][i];
  return d;
}

/** Deep copy of a matrix. */
export function clone(A: Mat): Mat {
  return A.map((row) => row.slice());
}

/** Number of rows and columns of a matrix as `[rows, cols]`. */
export function shape(A: Mat): [number, number] {
  return [A.length, A.length > 0 ? A[0].length : 0];
}

/** Column j of A as a fresh vector. */
export function getCol(A: Mat, j: number): Vec {
  return A.map((row) => row[j]);
}

/** Writes vector v into column j of A IN PLACE (the only mutating helper). */
export function setCol(A: Mat, j: number, v: Vec): void {
  for (let i = 0; i < A.length; i++) A[i][j] = v[i];
}

/** Transpose of A. */
export function transpose(A: Mat): Mat {
  const [m, n] = shape(A);
  const T = zeros(n, m);
  for (let i = 0; i < m; i++) for (let j = 0; j < n; j++) T[j][i] = A[i][j];
  return T;
}

/** Row-major flattening of a matrix into a vector. */
export function flatten(A: Mat): Vec {
  const out: Vec = [];
  for (const row of A) for (const x of row) out.push(x);
  return out;
}

/** Inverse of `flatten`: builds a rows x cols matrix from a row-major vector. */
export function reshape(v: Vec, rows: number, cols: number): Mat {
  if (v.length !== rows * cols) {
    throw new Error(`reshape: cannot reshape length ${v.length} into ${rows}x${cols}`);
  }
  const A = zeros(rows, cols);
  for (let i = 0; i < rows; i++) for (let j = 0; j < cols; j++) A[i][j] = v[i * cols + j];
  return A;
}

/** Horizontal concatenation [A | B | ...]; all blocks need the same row count. */
export function hstack(...blocks: Mat[]): Mat {
  const rows = blocks[0].length;
  for (const B of blocks) {
    if (B.length !== rows) throw new Error('hstack: row counts differ');
  }
  const out: Mat = [];
  for (let i = 0; i < rows; i++) {
    let row: Vec = [];
    for (const B of blocks) row = row.concat(B[i]);
    out.push(row);
  }
  return out;
}

/** Vertical concatenation; all blocks need the same column count. */
export function vstack(...blocks: Mat[]): Mat {
  const cols = blocks[0][0]?.length ?? 0;
  const out: Mat = [];
  for (const B of blocks) {
    for (const row of B) {
      if (row.length !== cols) throw new Error('vstack: column counts differ');
      out.push(row.slice());
    }
  }
  return out;
}

/** Block matrix from a 2-D grid of blocks, e.g. block([[R, zeros(3)], [pR, R]]). */
export function block(grid: Mat[][]): Mat {
  return vstack(...grid.map((blockRow) => hstack(...blockRow)));
}

// ---------------------------------------------------------------------------
// Elementwise matrix arithmetic
// ---------------------------------------------------------------------------

function assertSameShape(A: Mat, B: Mat, op: string): void {
  if (A.length !== B.length || (A[0]?.length ?? 0) !== (B[0]?.length ?? 0)) {
    throw new Error(`${op}: shapes ${shape(A)} and ${shape(B)} differ`);
  }
}

export function matAdd(A: Mat, B: Mat): Mat {
  assertSameShape(A, B, 'matAdd');
  return A.map((row, i) => row.map((x, j) => x + B[i][j]));
}

export function matSub(A: Mat, B: Mat): Mat {
  assertSameShape(A, B, 'matSub');
  return A.map((row, i) => row.map((x, j) => x - B[i][j]));
}

export function matScale(A: Mat, s: number): Mat {
  return A.map((row) => row.map((x) => x * s));
}

/** Matrix product A (m x k) times B (k x n). */
export function matMul(A: Mat, B: Mat): Mat {
  const [m, k] = shape(A);
  const [k2, n] = shape(B);
  if (k !== k2) throw new Error(`matMul: inner dimensions ${k} and ${k2} differ`);
  const C = zeros(m, n);
  for (let i = 0; i < m; i++) {
    const Ai = A[i];
    const Ci = C[i];
    for (let p = 0; p < k; p++) {
      const a = Ai[p];
      if (a === 0) continue;
      const Bp = B[p];
      for (let j = 0; j < n; j++) Ci[j] += a * Bp[j];
    }
  }
  return C;
}

/** Matrix-vector product A (m x n) times v (n). */
export function matVec(A: Mat, v: Vec): Vec {
  const [m, n] = shape(A);
  if (n !== v.length) throw new Error(`matVec: matrix has ${n} columns but vector has ${v.length} entries`);
  const out: Vec = new Array(m);
  for (let i = 0; i < m; i++) {
    let s = 0;
    const Ai = A[i];
    for (let j = 0; j < n; j++) s += Ai[j] * v[j];
    out[i] = s;
  }
  return out;
}

/** Trace (sum of diagonal entries) of a square matrix. */
export function trace(A: Mat): number {
  let s = 0;
  for (let i = 0; i < A.length; i++) s += A[i][i];
  return s;
}

/** Frobenius norm sqrt(sum A_ij^2). */
export function normFro(A: Mat): number {
  let s = 0;
  for (const row of A) for (const x of row) s += x * x;
  return Math.sqrt(s);
}

/** Largest absolute entry of a matrix. */
export function maxAbs(A: Mat): number {
  let m = 0;
  for (const row of A) for (const x of row) m = Math.max(m, Math.abs(x));
  return m;
}

// ---------------------------------------------------------------------------
// Vector arithmetic
// ---------------------------------------------------------------------------

function assertSameLength(a: Vec, b: Vec, op: string): void {
  if (a.length !== b.length) throw new Error(`${op}: lengths ${a.length} and ${b.length} differ`);
}

export function vecAdd(a: Vec, b: Vec): Vec {
  assertSameLength(a, b, 'vecAdd');
  return a.map((x, i) => x + b[i]);
}

export function vecSub(a: Vec, b: Vec): Vec {
  assertSameLength(a, b, 'vecSub');
  return a.map((x, i) => x - b[i]);
}

export function vecScale(a: Vec, s: number): Vec {
  return a.map((x) => x * s);
}

export function vecNeg(a: Vec): Vec {
  return a.map((x) => -x);
}

export function dot(a: Vec, b: Vec): number {
  assertSameLength(a, b, 'dot');
  let s = 0;
  for (let i = 0; i < a.length; i++) s += a[i] * b[i];
  return s;
}

/** Cross product of two 3-vectors. */
export function cross(a: Vec, b: Vec): Vec {
  if (a.length !== 3 || b.length !== 3) throw new Error('cross: both vectors must have length 3');
  return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
}

/** Euclidean norm. */
export function norm(a: Vec): number {
  return Math.sqrt(dot(a, a));
}

/** Unit vector in the direction of a; the zero vector is returned unchanged. */
export function normalize(a: Vec): Vec {
  const n = norm(a);
  return n === 0 ? a.slice() : vecScale(a, 1 / n);
}

/** Outer product a b^T (length(a) x length(b)). */
export function outer(a: Vec, b: Vec): Mat {
  return a.map((x) => b.map((y) => x * y));
}

// ---------------------------------------------------------------------------
// Scalars
// ---------------------------------------------------------------------------

/** True when |x| < eps (default 1e-6, the tolerance used throughout the book's code). */
export function nearZero(x: number, eps = 1e-6): boolean {
  return Math.abs(x) < eps;
}

export function clamp(x: number, lo: number, hi: number): number {
  return x < lo ? lo : x > hi ? hi : x;
}

/** Wraps an angle to the half-open interval [-pi, pi). */
export function wrapToPi(x: number): number {
  const twoPi = 2 * Math.PI;
  let y = (x + Math.PI) % twoPi;
  if (y < 0) y += twoPi;
  return y - Math.PI;
}

/** Wraps an angle to [0, 2*pi). */
export function wrapTo2Pi(x: number): number {
  const twoPi = 2 * Math.PI;
  let y = x % twoPi;
  if (y < 0) y += twoPi;
  return y;
}

/** Elementwise `wrapToPi`. */
export function wrapToPiVec(v: Vec): Vec {
  return v.map(wrapToPi);
}

/** Elementwise closeness test for two vectors or two matrices (absolute tolerance). */
export function approxEqual(a: Vec | Mat, b: Vec | Mat, tol = 1e-9): boolean {
  if (a.length !== b.length) return false;
  for (let i = 0; i < a.length; i++) {
    const x = a[i];
    const y = b[i];
    if (Array.isArray(x) && Array.isArray(y)) {
      if (!approxEqual(x, y, tol)) return false;
    } else if (typeof x === 'number' && typeof y === 'number') {
      if (!(Math.abs(x - y) <= tol)) return false;
    } else {
      return false;
    }
  }
  return true;
}

// ---------------------------------------------------------------------------
// Determinant, inverse, linear solve
// ---------------------------------------------------------------------------

function assertSquare(A: Mat, op: string): number {
  const [m, n] = shape(A);
  if (m !== n) throw new Error(`${op}: matrix is ${m}x${n}, not square`);
  return n;
}

/** Determinant via LU decomposition with partial pivoting. */
export function det(A: Mat): number {
  const n = assertSquare(A, 'det');
  const M = clone(A);
  let d = 1;
  for (let k = 0; k < n; k++) {
    let p = k;
    let best = Math.abs(M[k][k]);
    for (let i = k + 1; i < n; i++) {
      const v = Math.abs(M[i][k]);
      if (v > best) {
        best = v;
        p = i;
      }
    }
    if (best === 0) return 0;
    if (p !== k) {
      const tmp = M[k];
      M[k] = M[p];
      M[p] = tmp;
      d = -d;
    }
    const pivot = M[k][k];
    d *= pivot;
    for (let i = k + 1; i < n; i++) {
      const f = M[i][k] / pivot;
      if (f === 0) continue;
      for (let j = k; j < n; j++) M[i][j] -= f * M[k][j];
    }
  }
  return d;
}

/**
 * Inverse by Gauss-Jordan elimination with partial pivoting.
 * Throws when a pivot is smaller than `tol * max|A_ij|` (singular matrix).
 */
export function inv(A: Mat, tol = 1e-13): Mat {
  const n = assertSquare(A, 'inv');
  const scale = maxAbs(A);
  if (scale === 0) throw new Error('inv: matrix is singular (all entries are zero)');
  const I = eye(n);
  const M: Mat = A.map((row, i) => row.concat(I[i])); // augmented [A | I]
  for (let k = 0; k < n; k++) {
    let p = k;
    let best = Math.abs(M[k][k]);
    for (let i = k + 1; i < n; i++) {
      const v = Math.abs(M[i][k]);
      if (v > best) {
        best = v;
        p = i;
      }
    }
    if (best <= tol * scale) throw new Error(`inv: matrix is singular (pivot ${best} at column ${k})`);
    if (p !== k) {
      const tmp = M[k];
      M[k] = M[p];
      M[p] = tmp;
    }
    const pivot = M[k][k];
    const rowK = M[k];
    for (let j = 0; j < 2 * n; j++) rowK[j] /= pivot;
    for (let i = 0; i < n; i++) {
      if (i === k) continue;
      const f = M[i][k];
      if (f === 0) continue;
      const rowI = M[i];
      for (let j = 0; j < 2 * n; j++) rowI[j] -= f * rowK[j];
    }
  }
  return M.map((row) => row.slice(n));
}

/**
 * Solves the square linear system A x = b by Gaussian elimination with
 * partial pivoting and back substitution. Throws when A is singular.
 */
export function solve(A: Mat, b: Vec, tol = 1e-13): Vec {
  const n = assertSquare(A, 'solve');
  if (b.length !== n) throw new Error(`solve: A is ${n}x${n} but b has length ${b.length}`);
  const scale = maxAbs(A);
  if (scale === 0) throw new Error('solve: matrix is singular (all entries are zero)');
  const M = clone(A);
  const x = b.slice();
  for (let k = 0; k < n; k++) {
    let p = k;
    let best = Math.abs(M[k][k]);
    for (let i = k + 1; i < n; i++) {
      const v = Math.abs(M[i][k]);
      if (v > best) {
        best = v;
        p = i;
      }
    }
    if (best <= tol * scale) throw new Error(`solve: matrix is singular (pivot ${best} at column ${k})`);
    if (p !== k) {
      const tmp = M[k];
      M[k] = M[p];
      M[p] = tmp;
      const tb = x[k];
      x[k] = x[p];
      x[p] = tb;
    }
    for (let i = k + 1; i < n; i++) {
      const f = M[i][k] / M[k][k];
      if (f === 0) continue;
      for (let j = k; j < n; j++) M[i][j] -= f * M[k][j];
      x[i] -= f * x[k];
    }
  }
  for (let i = n - 1; i >= 0; i--) {
    let s = x[i];
    for (let j = i + 1; j < n; j++) s -= M[i][j] * x[j];
    x[i] = s / M[i][i];
  }
  return x;
}

// ---------------------------------------------------------------------------
// SVD, pseudo-inverse, symmetric eigendecomposition
// ---------------------------------------------------------------------------

export interface SVDResult {
  /** m x r with orthonormal columns (r = min(m, n)). */
  U: Mat;
  /** Singular values, sorted in descending order (length r). */
  S: Vec;
  /** n x r with orthonormal columns, so that A = U diag(S) V^T. */
  V: Mat;
}

/**
 * Thin singular value decomposition of any m x n matrix using one-sided
 * (Hestenes) Jacobi rotations. Returns A = U diag(S) V^T with the singular
 * values sorted in descending order. Columns of U that belong to zero
 * singular values are completed to an orthonormal set by Gram-Schmidt so
 * that U^T U = I always holds.
 */
export function svd(A: Mat, maxSweeps = 100): SVDResult {
  const [m, n] = shape(A);
  if (m < n) {
    const r = svd(transpose(A), maxSweeps);
    return { U: r.V, S: r.S, V: r.U };
  }
  const U = clone(A); // columns are rotated until mutually orthogonal
  const V = eye(n);
  const eps = 1e-15;
  for (let sweep = 0; sweep < maxSweeps; sweep++) {
    let rotated = false;
    for (let i = 0; i < n - 1; i++) {
      for (let j = i + 1; j < n; j++) {
        let alpha = 0;
        let beta = 0;
        let gamma = 0;
        for (let k = 0; k < m; k++) {
          const ui = U[k][i];
          const uj = U[k][j];
          alpha += ui * ui;
          beta += uj * uj;
          gamma += ui * uj;
        }
        if (gamma === 0 || Math.abs(gamma) <= eps * Math.sqrt(alpha * beta)) continue;
        rotated = true;
        const zeta = (beta - alpha) / (2 * gamma);
        const t = Math.sign(zeta || 1) / (Math.abs(zeta) + Math.sqrt(1 + zeta * zeta));
        const c = 1 / Math.sqrt(1 + t * t);
        const s = c * t;
        for (let k = 0; k < m; k++) {
          const ui = U[k][i];
          const uj = U[k][j];
          U[k][i] = c * ui - s * uj;
          U[k][j] = s * ui + c * uj;
        }
        for (let k = 0; k < n; k++) {
          const vi = V[k][i];
          const vj = V[k][j];
          V[k][i] = c * vi - s * vj;
          V[k][j] = s * vi + c * vj;
        }
      }
    }
    if (!rotated) break;
  }
  // Singular values are the column norms of the rotated matrix.
  const sv: Vec = new Array(n);
  for (let j = 0; j < n; j++) {
    let s = 0;
    for (let k = 0; k < m; k++) s += U[k][j] * U[k][j];
    sv[j] = Math.sqrt(s);
  }
  const order = sv.map((_, j) => j).sort((a, b) => sv[b] - sv[a]);
  const S: Vec = order.map((j) => sv[j]);
  const Uo = zeros(m, n);
  const Vo = zeros(n, n);
  const smax = S[0] ?? 0;
  const zeroTol = Math.max(m, n) * Number.EPSILON * smax;
  for (let c = 0; c < n; c++) {
    const j = order[c];
    if (sv[j] > zeroTol && sv[j] > 0) {
      for (let k = 0; k < m; k++) Uo[k][c] = U[k][j] / sv[j];
    }
    for (let k = 0; k < n; k++) Vo[k][c] = V[k][j];
  }
  // Complete U for (numerically) zero singular values with an orthonormal basis.
  for (let c = 0; c < n; c++) {
    const j = order[c];
    if (sv[j] > zeroTol && sv[j] > 0) continue;
    for (let e = 0; e < m; e++) {
      const cand: Vec = zerosVec(m);
      cand[e] = 1;
      for (let d = 0; d < n; d++) {
        if (d === c) continue;
        const colD = getCol(Uo, d);
        const proj = dot(cand, colD);
        if (proj !== 0) for (let k = 0; k < m; k++) cand[k] -= proj * colD[k];
      }
      const len = norm(cand);
      if (len > 1e-8) {
        for (let k = 0; k < m; k++) Uo[k][c] = cand[k] / len;
        break;
      }
    }
  }
  return { U: Uo, S, V: Vo };
}

/**
 * Moore-Penrose pseudo-inverse via SVD. Singular values below
 * `rcond * S[0]` are treated as zero, which makes the result well behaved
 * at kinematic singularities (no huge joint velocities).
 */
export function pinv(A: Mat, rcond = 1e-10): Mat {
  const [m, n] = shape(A);
  const { U, S, V } = svd(A);
  const cutoff = rcond * (S[0] ?? 0);
  const out = zeros(n, m);
  for (let k = 0; k < S.length; k++) {
    if (S[k] <= cutoff || S[k] === 0) continue;
    const invS = 1 / S[k];
    for (let i = 0; i < n; i++) {
      const vik = V[i][k] * invS;
      if (vik === 0) continue;
      for (let j = 0; j < m; j++) out[i][j] += vik * U[j][k];
    }
  }
  return out;
}

export interface EigSymResult {
  /** Eigenvalues sorted in descending order. */
  values: Vec;
  /** Orthonormal eigenvectors as COLUMNS; `vectors[i][k]` is component i of eigenvector k. */
  vectors: Mat;
}

/**
 * Eigendecomposition of a real symmetric matrix by the cyclic Jacobi method.
 * Only the symmetric part of A is used. Returns A = V diag(values) V^T.
 */
export function eigSym(A: Mat, maxSweeps = 100): EigSymResult {
  const n = assertSquare(A, 'eigSym');
  const M = zeros(n, n);
  for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) M[i][j] = 0.5 * (A[i][j] + A[j][i]);
  const V = eye(n);
  const scale = maxAbs(M) || 1;
  for (let sweep = 0; sweep < maxSweeps; sweep++) {
    let off = 0;
    for (let p = 0; p < n; p++) for (let q = p + 1; q < n; q++) off += M[p][q] * M[p][q];
    if (Math.sqrt(off) <= 1e-15 * scale) break;
    for (let p = 0; p < n - 1; p++) {
      for (let q = p + 1; q < n; q++) {
        const apq = M[p][q];
        if (Math.abs(apq) <= 1e-300) continue;
        const theta = (M[q][q] - M[p][p]) / (2 * apq);
        const t = Math.sign(theta || 1) / (Math.abs(theta) + Math.sqrt(theta * theta + 1));
        const c = 1 / Math.sqrt(t * t + 1);
        const s = t * c;
        for (let k = 0; k < n; k++) {
          const akp = M[k][p];
          const akq = M[k][q];
          M[k][p] = c * akp - s * akq;
          M[k][q] = s * akp + c * akq;
        }
        for (let k = 0; k < n; k++) {
          const apk = M[p][k];
          const aqk = M[q][k];
          M[p][k] = c * apk - s * aqk;
          M[q][k] = s * apk + c * aqk;
        }
        M[p][q] = 0;
        M[q][p] = 0;
        for (let k = 0; k < n; k++) {
          const vkp = V[k][p];
          const vkq = V[k][q];
          V[k][p] = c * vkp - s * vkq;
          V[k][q] = s * vkp + c * vkq;
        }
      }
    }
  }
  const vals = diagOf(M);
  const order = vals.map((_, i) => i).sort((a, b) => vals[b] - vals[a]);
  const values = order.map((i) => vals[i]);
  const vectors = zeros(n, n);
  for (let c = 0; c < n; c++) for (let k = 0; k < n; k++) vectors[k][c] = V[k][order[c]];
  return { values, vectors };
}

/** Matrix 2-norm condition number sigma_max / sigma_min (Infinity when rank deficient). */
export function cond(A: Mat): number {
  const { S } = svd(A);
  const smin = S[S.length - 1];
  return smin > 0 ? S[0] / smin : Infinity;
}
