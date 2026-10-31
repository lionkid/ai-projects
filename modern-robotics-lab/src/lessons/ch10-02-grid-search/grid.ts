/**
 * Pure grid-planning helpers for the grid-search lesson (no DOM, unit-tested):
 *  - square occupancy grids, optionally toroidal (for joint-angle C-spaces),
 *  - 4- / 8-connected neighbourhoods with costs 1 and √2 and no corner cutting,
 *  - a best-first search that covers Dijkstra (ε = 0), A* (ε = 1) and weighted A* (ε > 1)
 *    and records the whole expansion trace so the lesson can replay / scrub it,
 *  - a cost-to-go field from the goal ("wavefront") and steepest descent on it,
 *  - conservative down-sampling from a fine master bitmap,
 *  - a tiny seeded RRT used only as a preview of the next lesson.
 *
 * Cell index k = j·n + i, column i along x, row j along y (j = 0 at the bottom).
 */

export type Conn = 4 | 8;
export type HeuristicKind = 'euclid' | 'octile' | 'manhattan';

export interface Grid {
  /** Cells per axis (the grid is n × n). */
  n: number;
  /** 1 = obstacle cell. */
  blocked: Uint8Array;
  /** Torus: column 0 touches column n − 1 and row 0 touches row n − 1. */
  wrap: boolean;
}

const SQRT2 = Math.SQRT2;

/** Neighbour moves [di, dj, cost]: the 4 axis moves first, then the diagonals (a fixed order makes ties deterministic). */
export const DIRS: readonly (readonly [number, number, number])[] = [
  [1, 0, 1], [0, 1, 1], [-1, 0, 1], [0, -1, 1],
  [1, 1, SQRT2], [-1, 1, SQRT2], [-1, -1, SQRT2], [1, -1, SQRT2],
];

export const cellIndex = (n: number, i: number, j: number): number => j * n + i;
export const cellIJ = (n: number, k: number): [number, number] => [k % n, (k - (k % n)) / n];

/** The cell next to k in direction d (index into DIRS), or −1 when that is off a non-wrapping map. */
export function stepCell(g: Grid, k: number, d: number): number {
  const n = g.n;
  let i = (k % n) + DIRS[d][0];
  let j = ((k - (k % n)) / n) + DIRS[d][1];
  if (g.wrap) { i = (i + n) % n; j = (j + n) % n; }
  else if (i < 0 || j < 0 || i >= n || j >= n) return -1;
  return j * n + i;
}

export type EdgeStatus = 'ok' | 'blocked' | 'corner' | 'off';
export interface Edge { to: number; dir: number; cost: number; status: EdgeStatus }

/**
 * Every candidate edge out of k with the reason it is (not) usable, for drawing the neighbourhood:
 * 'blocked' = the neighbour is an obstacle, 'corner' = a diagonal that would squeeze past an
 * obstacle corner (one of the two side cells is blocked), 'off' = outside a non-wrapping map.
 */
export function edgesFrom(g: Grid, k: number, conn: Conn): Edge[] {
  const out: Edge[] = [];
  for (let d = 0; d < conn; d++) {
    const to = stepCell(g, k, d);
    const cost = DIRS[d][2];
    if (to < 0) { out.push({ to, dir: d, cost, status: 'off' }); continue; }
    if (g.blocked[to]) { out.push({ to, dir: d, cost, status: 'blocked' }); continue; }
    if (d >= 4) {
      const a = stepCell(g, k, DIRS[d][0] > 0 ? 0 : 2);
      const b = stepCell(g, k, DIRS[d][1] > 0 ? 1 : 3);
      if (a < 0 || b < 0 || g.blocked[a] || g.blocked[b]) { out.push({ to, dir: d, cost, status: 'corner' }); continue; }
    }
    out.push({ to, dir: d, cost, status: 'ok' });
  }
  return out;
}

/** Calls cb for every traversable neighbour of k (same rules as edgesFrom, without allocations). */
export function forNeighbors(g: Grid, k: number, conn: Conn, cb: (nb: number, cost: number, dir: number) => void): void {
  const bl = g.blocked;
  for (let d = 0; d < conn; d++) {
    const nb = stepCell(g, k, d);
    if (nb < 0 || bl[nb]) continue;
    if (d >= 4) {
      const a = stepCell(g, k, DIRS[d][0] > 0 ? 0 : 2);
      const b = stepCell(g, k, DIRS[d][1] > 0 ? 1 : 3);
      if (a < 0 || b < 0 || bl[a] || bl[b]) continue;
    }
    cb(nb, DIRS[d][2], d);
  }
}

/** Per-axis cell distance between a and b (shortest way round on a torus). */
export function cellDelta(g: Grid, a: number, b: number): [number, number] {
  const n = g.n;
  let dx = Math.abs((a % n) - (b % n));
  let dy = Math.abs((a - (a % n)) / n - (b - (b % n)) / n);
  if (g.wrap) { dx = Math.min(dx, n - dx); dy = Math.min(dy, n - dy); }
  return [dx, dy];
}

/**
 * Heuristic cost-to-go estimates in cell units. Euclidean is admissible for both neighbourhoods,
 * octile is the exact free-space distance of the 8-connected grid, Manhattan is exact for the
 * 4-connected grid but overestimates diagonal moves (2 instead of √2) on the 8-connected one.
 */
export function heuristic(g: Grid, a: number, b: number, kind: HeuristicKind): number {
  const [dx, dy] = cellDelta(g, a, b);
  if (kind === 'manhattan') return dx + dy;
  if (kind === 'octile') return Math.max(dx, dy) + (SQRT2 - 1) * Math.min(dx, dy);
  return Math.hypot(dx, dy);
}

/** Binary min-heap of search entries ordered by (f, h, insertion order), keys quantised so ties are exact. */
class EntryHeap {
  private cell = new Int32Array(64);
  private f = new Float64Array(64);
  private h = new Float64Array(64);
  private gv = new Float64Array(64);
  private seq = new Float64Array(64);
  private heap = new Int32Array(64);
  private free: number[] = [];
  private count = 0;
  private slots = 0;
  private nextSeq = 0;
  get size(): number { return this.count; }

  private grow(): void {
    const cap = this.cell.length * 2;
    const g = <T extends Int32Array | Float64Array>(a: T, mk: (c: number) => T): T => { const b = mk(cap); b.set(a); return b; };
    this.cell = g(this.cell, (c) => new Int32Array(c));
    this.f = g(this.f, (c) => new Float64Array(c));
    this.h = g(this.h, (c) => new Float64Array(c));
    this.gv = g(this.gv, (c) => new Float64Array(c));
    this.seq = g(this.seq, (c) => new Float64Array(c));
    this.heap = g(this.heap, (c) => new Int32Array(c));
  }

  private less(a: number, b: number): boolean {
    if (this.f[a] !== this.f[b]) return this.f[a] < this.f[b];
    if (this.h[a] !== this.h[b]) return this.h[a] < this.h[b];
    return this.seq[a] < this.seq[b];
  }

  push(cell: number, f: number, h: number, g: number): void {
    let s = this.free.pop();
    if (s === undefined) { if (this.slots >= this.cell.length) this.grow(); s = this.slots++; }
    this.cell[s] = cell;
    // quantise to 1e-6 cell: float noise in sums of 1 and √2 must not decide ties
    this.f[s] = Math.round(f * 1e6);
    this.h[s] = Math.round(h * 1e6);
    this.gv[s] = g;
    this.seq[s] = this.nextSeq++;
    if (this.count >= this.heap.length) this.grow();
    let i = this.count++;
    this.heap[i] = s;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!this.less(this.heap[i], this.heap[p])) break;
      const t = this.heap[i]; this.heap[i] = this.heap[p]; this.heap[p] = t; i = p;
    }
  }

  /** Removes the smallest entry and returns [cell, g]. */
  pop(): [number, number] {
    const top = this.heap[0];
    const res: [number, number] = [this.cell[top], this.gv[top]];
    this.free.push(top);
    this.count--;
    if (this.count > 0) {
      this.heap[0] = this.heap[this.count];
      let i = 0;
      for (;;) {
        const l = 2 * i + 1, r = l + 1;
        let m = i;
        if (l < this.count && this.less(this.heap[l], this.heap[m])) m = l;
        if (r < this.count && this.less(this.heap[r], this.heap[m])) m = r;
        if (m === i) break;
        const t = this.heap[i]; this.heap[i] = this.heap[m]; this.heap[m] = t; i = m;
      }
    }
    return res;
  }
}

export interface SearchOptions {
  conn: Conn;
  /** Heuristic weight: 0 = Dijkstra, 1 = A*, > 1 = weighted (greedier) A*. Ignored without a goal. */
  eps?: number;
  heuristic?: HeuristicKind;
}

export interface SearchTrace {
  n: number;
  source: number;
  /** −1 = no goal: expand every reachable cell (used for the wavefront). */
  goal: number;
  /** Cells in the order they were expanded (closed). */
  order: Int32Array;
  expanded: number;
  /** Per cell: index of its expansion (−1 if never expanded). Cell c is closed after k expansions iff 0 ≤ closedAt[c] < k. */
  closedAt: Int32Array;
  /** Per cell: number of expansions done when it was first put on the open list (−1 if never). */
  openedAt: Int32Array;
  /** Per cell: cost from the source when it was expanded (Infinity if never reached). */
  g: Float64Array;
  parent: Int32Array;
  /** Size of the open list after k expansions (length expanded + 1). */
  openCount: Int32Array;
  /** Number of expansions after which the goal was closed (−1 if it never was). */
  goalStep: number;
  /** source … goal (empty when no path). */
  path: number[];
  cost: number;
}

/**
 * Best-first graph search on the grid. Priority f = g + ε·h with ties broken by the smaller
 * (weighted) h and then first-come-first-served. Closed cells are never re-opened: that keeps every
 * cell expanded at most once and, with a consistent h, still guarantees cost ≤ ε · optimal.
 */
export function bestFirst(g: Grid, source: number, goal: number, opts: SearchOptions): SearchTrace {
  const N = g.n * g.n;
  const eps = goal >= 0 ? Math.max(0, opts.eps ?? 0) : 0;
  const kind = opts.heuristic ?? 'euclid';
  const gBest = new Float64Array(N).fill(Infinity);
  const parent = new Int32Array(N).fill(-1);
  const closedAt = new Int32Array(N).fill(-1);
  const openedAt = new Int32Array(N).fill(-1);
  const order = new Int32Array(N);
  const openCount = new Int32Array(N + 1);
  const heap = new EntryHeap();
  const hOf = (k: number): number => (eps > 0 ? eps * heuristic(g, k, goal, kind) : 0);
  let expanded = 0, open = 0, goalStep = -1;
  if (source >= 0 && source < N && !g.blocked[source]) {
    gBest[source] = 0; openedAt[source] = 0; open = 1;
    const h = hOf(source);
    heap.push(source, h, h, 0);
  }
  openCount[0] = open;
  let cur = -1, curG = 0;
  const relax = (nb: number, c: number): void => {
    if (closedAt[nb] >= 0) return;
    const ng = curG + c;
    if (ng < gBest[nb] - 1e-12) {
      if (openedAt[nb] < 0) { openedAt[nb] = expanded; open++; }
      gBest[nb] = ng;
      parent[nb] = cur;
      const h = hOf(nb);
      heap.push(nb, ng + h, h, ng);
    }
  };
  while (heap.size > 0) {
    const [k, gk] = heap.pop();
    if (closedAt[k] >= 0 || gk > gBest[k]) continue; // stale entry of an improved cell
    closedAt[k] = expanded;
    order[expanded++] = k;
    open--;
    if (k === goal) { goalStep = expanded; openCount[expanded] = open; break; }
    cur = k; curG = gk;
    forNeighbors(g, k, opts.conn, relax);
    openCount[expanded] = open;
  }
  const path: number[] = [];
  if (goalStep > 0) for (let k = goal; k >= 0; k = parent[k]) path.push(k);
  path.reverse();
  return {
    n: g.n, source, goal, order, expanded, closedAt, openedAt, g: gBest, parent,
    openCount: openCount.slice(0, expanded + 1), goalStep, path, cost: goalStep > 0 ? gBest[goal] : Infinity,
  };
}

/** Cost-to-go from every cell to `goal` (a goal-less Dijkstra run backwards; the grid graph is symmetric). */
export function wavefront(g: Grid, goal: number, conn: Conn): SearchTrace {
  return bestFirst(g, goal, -1, { conn, eps: 0 });
}

/** The neighbour a robot at k should move to when it only knows the cost-to-go field d (−1 at the goal or if stuck). */
export function downhill(g: Grid, d: Float64Array, k: number, conn: Conn): number {
  if (!Number.isFinite(d[k]) || d[k] <= 1e-9) return -1;
  let best = -1, bestV = Infinity;
  forNeighbors(g, k, conn, (nb, c) => {
    const v = d[nb] + c;
    if (v < bestV - 1e-9) { bestV = v; best = nb; }
  });
  return best >= 0 && d[best] < d[k] ? best : -1;
}

/** Steepest descent on the cost-to-go field from `from` to the cell where d = 0 (empty if unreachable). */
export function descend(g: Grid, d: Float64Array, from: number, conn: Conn): number[] {
  if (from < 0 || !Number.isFinite(d[from])) return [];
  const path = [from];
  let k = from;
  for (let guard = 0; guard < g.n * g.n && d[k] > 1e-9; guard++) {
    const nb = downhill(g, d, k, conn);
    if (nb < 0) break;
    path.push(nb);
    k = nb;
  }
  return path;
}

/** Length of a cell path (1 per axis move, √2 per diagonal move). */
export function pathCost(g: Grid, path: readonly number[]): number {
  let c = 0;
  for (let s = 0; s + 1 < path.length; s++) {
    const [dx, dy] = cellDelta(g, path[s], path[s + 1]);
    c += dx && dy ? SQRT2 : 1;
  }
  return c;
}

/** Conservative down-sampling: a coarse cell is blocked when any of the master cells it covers is blocked. */
export function downsample(master: Uint8Array, M: number, n: number): Uint8Array {
  if (M % n !== 0) throw new Error(`downsample: ${M} is not a multiple of ${n}`);
  const f = M / n;
  const out = new Uint8Array(n * n);
  for (let J = 0; J < M; J++) {
    const row = ((J / f) | 0) * n;
    for (let I = 0; I < M; I++) if (master[J * M + I]) out[row + ((I / f) | 0)] = 1;
  }
  return out;
}

/** Set (value 1) or clear (0) every master cell under coarse cell (i, j) of an n × n grid. */
export function paintMaster(master: Uint8Array, M: number, n: number, i: number, j: number, value: 0 | 1): void {
  const f = M / n;
  for (let J = j * f; J < (j + 1) * f; J++) master.fill(value, J * M + i * f, J * M + (i + 1) * f);
}

/** Nearest free cell to k (breadth-first over all cells), or −1 if the grid is completely blocked. */
export function nearestFree(g: Grid, k: number): number {
  if (!g.blocked[k]) return k;
  const seen = new Uint8Array(g.n * g.n);
  const queue = [k];
  seen[k] = 1;
  for (let h = 0; h < queue.length; h++) {
    const c = queue[h];
    if (!g.blocked[c]) return c;
    for (let d = 0; d < 4; d++) {
      const nb = stepCell(g, c, d);
      if (nb >= 0 && !seen[nb]) { seen[nb] = 1; queue.push(nb); }
    }
  }
  return -1;
}

/** Small deterministic PRNG (mulberry32). */
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

export interface RRTPreview { nodes: [number, number][]; parent: number[]; goalNode: number }

/**
 * A plain RRT in continuous coordinates over a bitmap (world units = cells of `occ`, no wrap-around),
 * only used to preview the next lesson: it never builds a grid, it just asks "is this segment free?".
 */
export function rrtPreview(occ: Grid, start: [number, number], goal: [number, number],
  opts: { iters?: number; step?: number; seed?: number; goalBias?: number } = {}): RRTPreview {
  const n = occ.n;
  const iters = opts.iters ?? 1500;
  const stepLen = opts.step ?? n / 18;
  const bias = opts.goalBias ?? 0.08;
  const rnd = mulberry32(opts.seed ?? 7);
  const free = (x: number, y: number): boolean => x >= 0 && y >= 0 && x < n && y < n && !occ.blocked[Math.floor(y) * n + Math.floor(x)];
  const segFree = (a: [number, number], b: [number, number]): boolean => {
    const m = Math.max(1, Math.ceil(Math.hypot(b[0] - a[0], b[1] - a[1]) / 0.25));
    for (let s = 0; s <= m; s++) if (!free(a[0] + ((b[0] - a[0]) * s) / m, a[1] + ((b[1] - a[1]) * s) / m)) return false;
    return true;
  };
  const nodes: [number, number][] = [start];
  const parent = [-1];
  let goalNode = -1;
  if (!free(start[0], start[1])) return { nodes, parent, goalNode };
  for (let it = 0; it < iters && goalNode < 0; it++) {
    const target: [number, number] = rnd() < bias ? goal : [rnd() * n, rnd() * n];
    let best = 0, bestD = Infinity;
    for (let k = 0; k < nodes.length; k++) {
      const d = (nodes[k][0] - target[0]) ** 2 + (nodes[k][1] - target[1]) ** 2;
      if (d < bestD) { bestD = d; best = k; }
    }
    const a = nodes[best];
    const dist = Math.sqrt(bestD);
    if (dist < 1e-9) continue;
    const s = Math.min(1, stepLen / dist);
    const b: [number, number] = [a[0] + (target[0] - a[0]) * s, a[1] + (target[1] - a[1]) * s];
    if (!segFree(a, b)) continue;
    nodes.push(b);
    parent.push(best);
    if (Math.hypot(b[0] - goal[0], b[1] - goal[1]) <= stepLen && segFree(b, goal)) {
      nodes.push([goal[0], goal[1]]);
      parent.push(nodes.length - 2);
      goalNode = nodes.length - 1;
    }
  }
  return { nodes, parent, goalNode };
}
