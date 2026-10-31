/**
 * Pure helpers for the 2R C-obstacle lesson: collision / clearance queries for a planar
 * 2R arm with thick links against disk obstacles, exact (capsule vs. disk) or via a
 * conservative circle cover, plus torus (wrap-around) utilities for the (θ₁, θ₂) map.
 * No DOM here so everything is unit-testable.
 */
import { planarFK, segPointDistance, wrapToPi, type P2 } from '@/mr';

/** Bit flags stored in a C-obstacle code grid: which link of the arm is in collision. */
export const HIT_LINK1 = 1;
export const HIT_LINK2 = 2;

export interface Obstacle { c: P2; r: number }
export interface Circle { c: P2; r: number; link: number }
export type CollisionMode = 'exact' | 'cover';

export interface ClearanceResult {
  /** Signed clearance (negative = penetration). +Infinity when there are no obstacles. */
  d: number;
  /** Index of the link (0 or 1) realising the minimum, -1 if none. */
  link: number;
  /** Index of the obstacle realising the minimum, -1 if none. */
  obs: number;
  /** Closest point on the arm and on the obstacle disk boundary (for drawing). */
  pArm: P2;
  pObs: P2;
  /** For cover mode: the index of the cover circle realising the minimum. */
  circle: number;
}

/** Joint positions [base, elbow, tip] of the 2R arm. */
export function armPoints(L: readonly [number, number], q: P2): [P2, P2, P2] {
  const fk = planarFK([L[0], L[1]], [q[0], q[1]]);
  return [[0, 0], [fk.joints[1][0], fk.joints[1][1]], [fk.joints[2][0], fk.joints[2][1]]];
}

function closestOnSeg(a: P2, b: P2, p: P2): P2 {
  const dx = b[0] - a[0], dy = b[1] - a[1];
  const L2 = dx * dx + dy * dy;
  if (L2 < 1e-18) return [a[0], a[1]];
  const t = Math.max(0, Math.min(1, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L2));
  return [a[0] + t * dx, a[1] + t * dy];
}

/** Exact signed clearance between a thick 2R arm (links are capsules of half-width w) and disk obstacles. */
export function exactClearance(L: readonly [number, number], w: number, q: P2, obstacles: readonly Obstacle[]): ClearanceResult {
  const pts = armPoints(L, q);
  let best: ClearanceResult = { d: Infinity, link: -1, obs: -1, pArm: [0, 0], pObs: [0, 0], circle: -1 };
  for (let k = 0; k < 2; k++) {
    for (let o = 0; o < obstacles.length; o++) {
      const ob = obstacles[o];
      const d = segPointDistance(pts[k], pts[k + 1], ob.c) - ob.r - w;
      if (d < best.d) {
        const pa = closestOnSeg(pts[k], pts[k + 1], ob.c);
        const vx = pa[0] - ob.c[0], vy = pa[1] - ob.c[1];
        const n = Math.hypot(vx, vy) || 1;
        best = { d, link: k, obs: o, pArm: [pa[0] - (vx / n) * w, pa[1] - (vy / n) * w], pObs: [ob.c[0] + (vx / n) * ob.r, ob.c[1] + (vy / n) * ob.r], circle: -1 };
      }
    }
  }
  return best;
}

/**
 * Circle cover of the arm: K + 1 circles per link, centred at s = i·L/K (i = 0…K) with radius
 * r_c = sqrt(w² + (L/2K)²). Their union contains the whole capsule of half-width w (including the
 * round caps at both joints), so the cover is conservative: it never misses a true collision.
 */
export function circleCover(L: readonly [number, number], w: number, q: P2, K = 5): Circle[] {
  const pts = armPoints(L, q);
  const out: Circle[] = [];
  for (let k = 0; k < 2; k++) {
    const a = pts[k], b = pts[k + 1];
    const rc = Math.hypot(w, L[k] / (2 * K));
    for (let i = 0; i <= K; i++) {
      const s = i / K;
      out.push({ c: [a[0] + s * (b[0] - a[0]), a[1] + s * (b[1] - a[1])], r: rc, link: k });
    }
  }
  return out;
}

/** Clearance using only centre distances between cover circles and obstacle disks. */
export function coverClearance(L: readonly [number, number], w: number, q: P2, obstacles: readonly Obstacle[], K = 5): ClearanceResult {
  const circles = circleCover(L, w, q, K);
  let best: ClearanceResult = { d: Infinity, link: -1, obs: -1, pArm: [0, 0], pObs: [0, 0], circle: -1 };
  for (let i = 0; i < circles.length; i++) {
    const ci = circles[i];
    for (let o = 0; o < obstacles.length; o++) {
      const ob = obstacles[o];
      const vx = ob.c[0] - ci.c[0], vy = ob.c[1] - ci.c[1];
      const dist = Math.hypot(vx, vy);
      const d = dist - ci.r - ob.r;
      if (d < best.d) {
        const n = dist || 1;
        best = { d, link: ci.link, obs: o, pArm: [ci.c[0] + (vx / n) * ci.r, ci.c[1] + (vy / n) * ci.r], pObs: [ob.c[0] - (vx / n) * ob.r, ob.c[1] - (vy / n) * ob.r], circle: i };
      }
    }
  }
  return best;
}

export function clearance(mode: CollisionMode, L: readonly [number, number], w: number, q: P2, obstacles: readonly Obstacle[], K = 5): ClearanceResult {
  return mode === 'cover' ? coverClearance(L, w, q, obstacles, K) : exactClearance(L, w, q, obstacles);
}

export function collides(mode: CollisionMode, L: readonly [number, number], w: number, q: P2, obstacles: readonly Obstacle[], K = 5): boolean {
  return clearance(mode, L, w, q, obstacles, K).d <= 0;
}

/**
 * Does the link from a to b (length len, capsule half-width w) touch any obstacle?
 * 'exact' tests the capsule; 'cover' tests the K + 1 cover circles used by circleCover.
 */
function linkHits(mode: CollisionMode, ax: number, ay: number, bx: number, by: number, len: number, w: number, obstacles: readonly Obstacle[], K: number): boolean {
  if (mode === 'exact') {
    for (const ob of obstacles) if (segPointDistance([ax, ay], [bx, by], ob.c) <= ob.r + w) return true;
    return false;
  }
  const rc = Math.hypot(w, len / (2 * K));
  for (let i = 0; i <= K; i++) {
    const s = i / K;
    const cx = ax + s * (bx - ax), cy = ay + s * (by - ay);
    for (const ob of obstacles) if (Math.hypot(ob.c[0] - cx, ob.c[1] - cy) <= rc + ob.r) return true;
  }
  return false;
}

/** Which links collide at configuration q (bit flags HIT_LINK1 / HIT_LINK2), same tests as computeCobsCodes. */
export function linkCollisions(mode: CollisionMode, L: readonly [number, number], w: number, q: P2, obstacles: readonly Obstacle[], K = 5): number {
  const [b, e, t] = armPoints(L, q);
  return (linkHits(mode, b[0], b[1], e[0], e[1], L[0], w, obstacles, K) ? HIT_LINK1 : 0)
    | (linkHits(mode, e[0], e[1], t[0], t[1], L[1], w, obstacles, K) ? HIT_LINK2 : 0);
}

/**
 * Rasterised C-obstacle with per-link information: codes[j * n + i] has bit HIT_LINK1 when link 1
 * collides and bit HIT_LINK2 when link 2 collides at (θ₁, θ₂) = (cellAngle(i), cellAngle(j)).
 *
 * Link 1 only depends on θ₁, so its test runs once per column (that is why link-1 collisions
 * always form full-height bands). Link 2 reuses the elbow of its column and a forearm vector
 * that depends only on the absolute angle φ = θ₁ + θ₂, both obtained from planarFK, so the
 * O(n²) loop does no trigonometry.
 */
export function computeCobsCodes(n: number, mode: CollisionMode, L: readonly [number, number], w: number, obstacles: readonly Obstacle[], K = 5): Uint8Array {
  const codes = new Uint8Array(n * n);
  if (obstacles.length === 0) return codes;
  const ex = new Float64Array(n), ey = new Float64Array(n), hit1 = new Uint8Array(n);
  for (let i = 0; i < n; i++) {
    const e = planarFK([L[0]], [cellAngle(i, n)]).tip;
    ex[i] = e[0]; ey[i] = e[1];
    hit1[i] = linkHits(mode, 0, 0, e[0], e[1], L[0], w, obstacles, K) ? HIT_LINK1 : 0;
  }
  // cellAngle(i) + cellAngle(j) = -2π + (i + j + 1)·2π/n, so the forearm direction depends on k = i + j only.
  const fx = new Float64Array(2 * n - 1), fy = new Float64Array(2 * n - 1);
  for (let k = 0; k < 2 * n - 1; k++) {
    const f = planarFK([L[1]], [-2 * Math.PI + ((k + 1) / n) * 2 * Math.PI]).tip;
    fx[k] = f[0]; fy[k] = f[1];
  }
  for (let j = 0; j < n; j++) {
    for (let i = 0; i < n; i++) {
      const tx = ex[i] + fx[i + j], ty = ey[i] + fy[i + j];
      codes[j * n + i] = hit1[i] | (linkHits(mode, ex[i], ey[i], tx, ty, L[1], w, obstacles, K) ? HIT_LINK2 : 0);
    }
  }
  return codes;
}

/** Fraction of cells with a non-zero code. */
export function blockedFraction(codes: Uint8Array): number {
  let k = 0;
  for (let i = 0; i < codes.length; i++) if (codes[i]) k++;
  return codes.length ? k / codes.length : 0;
}

/**
 * Where does the C-obstacle meet a seam of the unwrapped square? The seam lies between index
 * s − 1 and s (mod n) along `axis` (0: a θ₁ = const seam, scanned over rows; 1: a θ₂ = const seam,
 * scanned over columns). Returns runs [k0, k1] (inclusive) of blocked cells touching the seam on
 * either side, so the same runs can be marked on both opposite edges of the square.
 */
export function seamRuns(codes: Uint8Array, n: number, axis: 0 | 1, s: number): [number, number][] {
  const a = (s - 1 + n) % n, b = s % n;
  const runs: [number, number][] = [];
  let start = -1;
  for (let k = 0; k <= n; k++) {
    const on = k < n && (axis === 0 ? (codes[k * n + a] || codes[k * n + b]) : (codes[a * n + k] || codes[b * n + k]));
    if (on && start < 0) start = k;
    if (!on && start >= 0) { runs.push([start, k - 1]); start = -1; }
  }
  return runs;
}

/** θ value of grid column/row index k in an n-cell discretisation of [-π, π). */
export function cellAngle(k: number, n: number): number { return -Math.PI + ((k + 0.5) / n) * 2 * Math.PI; }
/** Grid index of an angle (wrapped into [-π, π)). */
export function angleCell(theta: number, n: number): number {
  const u = (wrapToPi(theta) + Math.PI) / (2 * Math.PI);
  return Math.min(n - 1, Math.max(0, Math.floor(u * n)));
}

/**
 * Rasterised C-obstacle: grid[j * n + i] = 1 when (θ₁ = cellAngle(i), θ₂ = cellAngle(j)) collides.
 * i indexes θ₁ (columns), j indexes θ₂ (rows, j = 0 at θ₂ = -π).
 */
export function computeCobsGrid(n: number, mode: CollisionMode, L: readonly [number, number], w: number, obstacles: readonly Obstacle[], K = 5): Uint8Array {
  const grid = new Uint8Array(n * n);
  if (obstacles.length === 0) return grid;
  for (let j = 0; j < n; j++) {
    const th2 = cellAngle(j, n);
    for (let i = 0; i < n; i++) {
      if (collides(mode, L, w, [cellAngle(i, n), th2], obstacles, K)) grid[j * n + i] = 1;
    }
  }
  return grid;
}

export interface Components { labels: Int32Array; count: number; sizes: number[] }

/** Connected components of the free cells on the torus (4-neighbourhood with wrap-around). Blocked cells get label -1. */
export function torusComponents(grid: Uint8Array, n: number): Components {
  const labels = new Int32Array(n * n).fill(-2);
  const stack = new Int32Array(n * n);
  const sizes: number[] = [];
  let count = 0;
  for (let s = 0; s < n * n; s++) {
    if (grid[s]) { labels[s] = -1; continue; }
    if (labels[s] !== -2) continue;
    let top = 0; stack[top++] = s; labels[s] = count; let size = 0;
    while (top > 0) {
      const k = stack[--top]; size++;
      const i = k % n, j = (k - i) / n;
      const nb = [((i + 1) % n) + j * n, ((i + n - 1) % n) + j * n, i + ((j + 1) % n) * n, i + ((j + n - 1) % n) * n];
      for (const m of nb) if (!grid[m] && labels[m] === -2) { labels[m] = count; stack[top++] = m; }
    }
    sizes.push(size); count++;
  }
  return { labels, count, sizes };
}

/** Component label at a configuration (-1 inside the C-obstacle). */
export function componentAt(comp: Components, n: number, q: P2): number {
  return comp.labels[angleCell(q[1], n) * n + angleCell(q[0], n)];
}

/** Shortest torus displacement from a to b (each component wrapped into [-π, π)). */
export function torusDelta(a: P2, b: P2): P2 { return [wrapToPi(b[0] - a[0]), wrapToPi(b[1] - a[1])]; }
export function torusDist(a: P2, b: P2): number { const d = torusDelta(a, b); return Math.hypot(d[0], d[1]); }
export function wrapQ(q: P2): P2 { return [wrapToPi(q[0]), wrapToPi(q[1])]; }

/**
 * Split the shortest torus segment from q1 to q2 into straight pieces that all lie inside the
 * fundamental square [-π, π)². Used to draw a path that crosses the wrap-around edge.
 */
export function torusSegmentPieces(q1: P2, q2: P2): [P2, P2][] {
  const a = wrapQ(q1);
  const d = torusDelta(a, q2);
  const cuts = [0, 1];
  for (let k = 0; k < 2; k++) {
    if (Math.abs(d[k]) < 1e-12) continue;
    for (const bnd of [-Math.PI, Math.PI]) {
      const s = (bnd - a[k]) / d[k];
      if (s > 1e-9 && s < 1 - 1e-9) cuts.push(s);
    }
  }
  cuts.sort((x, y) => x - y);
  const pieces: [P2, P2][] = [];
  for (let i = 0; i + 1 < cuts.length; i++) {
    const s0 = cuts[i], s1 = cuts[i + 1];
    if (s1 - s0 < 1e-9) continue;
    const sm = (s0 + s1) / 2;
    const off: P2 = [0, 1].map((k) => 2 * Math.PI * Math.round((a[k] + sm * d[k]) / (2 * Math.PI))) as P2;
    pieces.push([
      [a[0] + s0 * d[0] - off[0], a[1] + s0 * d[1] - off[1]],
      [a[0] + s1 * d[0] - off[0], a[1] + s1 * d[1] - off[1]],
    ]);
  }
  return pieces;
}

export interface PathSample { q: P2; hit: boolean; seg: number }
export interface PathCheck { samples: PathSample[]; hits: number; badSegments: number; length: number }

/** Sample a joint-space polyline (shortest torus segments between consecutive points) and test every sample. */
export function checkPath(path: readonly P2[], mode: CollisionMode, L: readonly [number, number], w: number, obstacles: readonly Obstacle[], ds = 0.02, K = 5): PathCheck {
  const samples: PathSample[] = [];
  let hits = 0, badSegments = 0, length = 0;
  if (path.length === 0) return { samples, hits, badSegments, length };
  const push = (q: P2, seg: number) => {
    const hit = collides(mode, L, w, q, obstacles, K);
    if (hit) hits++;
    samples.push({ q: wrapQ(q), hit, seg });
    return hit;
  };
  push(path[0], 0);
  for (let s = 0; s + 1 < path.length; s++) {
    const a = wrapQ(path[s]);
    const d = torusDelta(a, path[s + 1]);
    const len = Math.hypot(d[0], d[1]);
    length += len;
    const m = Math.max(1, Math.ceil(len / ds));
    let bad = false;
    for (let k = 1; k <= m; k++) {
      const t = k / m;
      if (push([a[0] + t * d[0], a[1] + t * d[1]], s)) bad = true;
    }
    if (bad) badSegments++;
  }
  return { samples, hits, badSegments, length };
}

/** Configuration at arc-length fraction u ∈ [0, 1] along the path (shortest torus segments). */
export function pathAt(path: readonly P2[], u: number): P2 {
  if (path.length === 0) return [0, 0];
  if (path.length === 1) return wrapQ(path[0]);
  const lens: number[] = [];
  let total = 0;
  for (let s = 0; s + 1 < path.length; s++) { const d = torusDelta(path[s], path[s + 1]); const l = Math.hypot(d[0], d[1]); lens.push(l); total += l; }
  let target = Math.max(0, Math.min(1, u)) * total;
  for (let s = 0; s < lens.length; s++) {
    if (target <= lens[s] || s === lens.length - 1) {
      const t = lens[s] > 1e-12 ? Math.min(1, target / lens[s]) : 1;
      const a = wrapQ(path[s]); const d = torusDelta(a, path[s + 1]);
      return wrapQ([a[0] + t * d[0], a[1] + t * d[1]]);
    }
    target -= lens[s];
  }
  return wrapQ(path[path.length - 1]);
}
