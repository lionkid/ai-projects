/**
 * Pure sampling-based planning helpers for the RRT / PRM lesson.
 *
 * - `CSpace`: a 2-D configuration space, either a bounded box (point robot on a map) or a flat
 *   torus (the two joint angles of a 2R arm), with a collision-query counter, the shortest
 *   difference / distance / steering in that metric and a fixed-spacing local-path check.
 * - `GridIndex`: bucket grid for nearest-neighbour and radius queries (wraps on the torus).
 * - `RRT` (goal bias, optional "naive random tree" variant), `RRTConnect`, `PRM` + `prmQuery`,
 *   `shortcut` smoothing and small trial runners for success-rate experiments.
 *
 * No DOM here so everything is unit-testable. Angles are radians; distances are in the units of
 * the space (metres on the map, radians on the torus).
 */

export type Q = [number, number];
export interface Bounds { xmin: number; xmax: number; ymin: number; ymax: number }
export interface CheckPoint { q: Q; free: boolean }
export type Status = 'searching' | 'success' | 'failed' | 'limit';

/** Small, fast, seedable PRNG (mulberry32). Returns numbers in [0, 1). */
export function mulberry32(seed: number): () => number {
  let a = (seed >>> 0) || 0x9e3779b9;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

const mod = (x: number, m: number): number => ((x % m) + m) % m;

export class CSpace {
  /** Number of single-configuration collision queries made so far (planners + local paths). */
  checks = 0;
  readonly W: number;
  readonly H: number;
  constructor(
    readonly bounds: Bounds,
    /** true: both coordinates wrap around (flat torus). */
    readonly torus: boolean,
    private readonly freeFn: (q: Q) => boolean,
    /** Spacing of the collision checks along a straight local path. */
    public eps = 0.05,
  ) {
    this.W = bounds.xmax - bounds.xmin;
    this.H = bounds.ymax - bounds.ymin;
  }

  inBounds(q: Q): boolean {
    if (this.torus) return true;
    const b = this.bounds;
    return q[0] >= b.xmin && q[0] <= b.xmax && q[1] >= b.ymin && q[1] <= b.ymax;
  }

  /** Counted collision query (this is what a planner pays for). */
  isFree(q: Q): boolean {
    this.checks++;
    return this.inBounds(q) && this.freeFn(q);
  }

  /** Same query without touching the counter (for drawing and metrics). */
  freeQuiet(q: Q): boolean { return this.inBounds(q) && this.freeFn(q); }

  /** Map a configuration back into the fundamental domain (torus only). */
  wrap(q: Q): Q {
    if (!this.torus) return [q[0], q[1]];
    const b = this.bounds;
    return [b.xmin + mod(q[0] - b.xmin, this.W), b.ymin + mod(q[1] - b.ymin, this.H)];
  }

  /** Shortest displacement from a to b (on the torus: through the seam when that is shorter). */
  diff(a: Q, b: Q): Q {
    let dx = b[0] - a[0], dy = b[1] - a[1];
    if (this.torus) {
      dx = mod(dx + this.W / 2, this.W) - this.W / 2;
      dy = mod(dy + this.H / 2, this.H) - this.H / 2;
    }
    return [dx, dy];
  }

  dist(a: Q, b: Q): number { const d = this.diff(a, b); return Math.hypot(d[0], d[1]); }

  /** Point at fraction s ∈ [0, 1] of the straight local path from a to b. */
  interp(a: Q, b: Q, s: number): Q {
    const d = this.diff(a, b);
    return this.wrap([a[0] + s * d[0], a[1] + s * d[1]]);
  }

  /** q_new = q_near + δ (q_rand − q_near)/‖q_rand − q_near‖, or q_rand itself when it is closer than δ. */
  steer(from: Q, to: Q, delta: number): Q {
    const d = this.diff(from, to);
    const L = Math.hypot(d[0], d[1]);
    if (L <= delta) return this.wrap([from[0] + d[0], from[1] + d[1]]);
    return this.wrap([from[0] + (delta * d[0]) / L, from[1] + (delta * d[1]) / L]);
  }

  sample(rng: () => number): Q {
    const b = this.bounds;
    return [b.xmin + rng() * this.W, b.ymin + rng() * this.H];
  }

  /**
   * Collision-check the straight local path a→b at spacing ≤ eps (a itself is assumed free, b is
   * checked). Stops at the first colliding point. `record` receives every checked point.
   */
  checkEdge(a: Q, b: Q, record?: CheckPoint[]): boolean {
    const n = Math.max(1, Math.ceil(this.dist(a, b) / this.eps - 1e-9));
    for (let k = 1; k <= n; k++) {
      const q = this.interp(a, b, k / n);
      const free = this.isFree(q);
      record?.push({ q, free });
      if (!free) return false;
    }
    return true;
  }
}

/** Uniform bucket grid over the space bounds for nearest-neighbour / radius queries. */
export class GridIndex {
  private readonly nx: number;
  private readonly ny: number;
  private readonly cw: number;
  private readonly ch: number;
  private cells: number[][];
  private readonly stamp: Uint32Array;
  private tag = 0;
  private count = 0;

  constructor(private readonly space: CSpace, private readonly pts: readonly Q[], cell: number) {
    this.nx = Math.max(1, Math.min(256, Math.ceil(space.W / cell)));
    this.ny = Math.max(1, Math.min(256, Math.ceil(space.H / cell)));
    this.cw = space.W / this.nx;
    this.ch = space.H / this.ny;
    this.cells = Array.from({ length: this.nx * this.ny }, () => []);
    this.stamp = new Uint32Array(this.nx * this.ny);
  }

  get size(): number { return this.count; }

  private cellOf(q: Q): [number, number] {
    const b = this.space.bounds;
    let i = Math.floor((q[0] - b.xmin) / this.cw);
    let j = Math.floor((q[1] - b.ymin) / this.ch);
    if (this.space.torus) { i = mod(i, this.nx); j = mod(j, this.ny); }
    else { i = Math.min(this.nx - 1, Math.max(0, i)); j = Math.min(this.ny - 1, Math.max(0, j)); }
    return [i, j];
  }

  insert(id: number): void {
    const [i, j] = this.cellOf(this.pts[id]);
    this.cells[j * this.nx + i].push(id);
    this.count++;
  }

  /** Visit the cells on the square ring at Chebyshev offset k around (ci, cj) (each cell once per query). */
  private ring(ci: number, cj: number, k: number, visit: (ids: number[]) => void): void {
    const torus = this.space.torus;
    const take = (di: number, dj: number) => {
      let i = ci + di, j = cj + dj;
      if (torus) { i = mod(i, this.nx); j = mod(j, this.ny); }
      else if (i < 0 || j < 0 || i >= this.nx || j >= this.ny) return;
      const c = j * this.nx + i;
      if (this.stamp[c] === this.tag) return;
      this.stamp[c] = this.tag;
      const ids = this.cells[c];
      if (ids.length) visit(ids);
    };
    if (k === 0) { take(0, 0); return; }
    for (let d = -k; d <= k; d++) { take(d, -k); take(d, k); }
    for (let d = -k + 1; d <= k - 1; d++) { take(-k, d); take(k, d); }
  }

  private nextTag(): void {
    this.tag++;
    if (this.tag >= 0xffffffff) { this.stamp.fill(0); this.tag = 1; }
  }

  /** Index of the stored point nearest to q (−1 when empty). */
  nearest(q: Q): number {
    if (this.count === 0) return -1;
    this.nextTag();
    const [ci, cj] = this.cellOf(q);
    const cmin = Math.min(this.cw, this.ch);
    const maxK = Math.max(this.nx, this.ny);
    let best = -1, bestD = Infinity;
    for (let k = 0; k <= maxK; k++) {
      this.ring(ci, cj, k, (ids) => {
        for (const id of ids) {
          const d = this.space.dist(q, this.pts[id]);
          if (d < bestD) { bestD = d; best = id; }
        }
      });
      // every point in ring k + 1 or beyond is at least k·cmin away
      if (best >= 0 && bestD <= k * cmin) break;
    }
    return best;
  }

  /** Indices of all stored points within distance r of q (unsorted). */
  within(q: Q, r: number): number[] {
    const out: number[] = [];
    if (this.count === 0) return out;
    this.nextTag();
    const [ci, cj] = this.cellOf(q);
    const K = Math.min(Math.max(this.nx, this.ny), Math.ceil(r / Math.min(this.cw, this.ch)) + 1);
    for (let k = 0; k <= K; k++) {
      this.ring(ci, cj, k, (ids) => {
        for (const id of ids) if (this.space.dist(q, this.pts[id]) <= r) out.push(id);
      });
    }
    return out;
  }
}

/** A tree of configurations rooted at nodes[0]. */
export class Tree {
  readonly nodes: Q[] = [];
  readonly parent: number[] = [];
  readonly index: GridIndex;
  constructor(readonly space: CSpace, root: Q, cell: number) {
    this.index = new GridIndex(space, this.nodes, cell);
    this.add(root, -1);
  }
  get size(): number { return this.nodes.length; }
  add(q: Q, parent: number): number {
    this.nodes.push(q);
    this.parent.push(parent);
    const id = this.nodes.length - 1;
    this.index.insert(id);
    return id;
  }
  nearest(q: Q): number { return this.index.nearest(q); }
  /** Configurations from the root to node `id`. */
  pathTo(id: number): Q[] {
    const out: Q[] = [];
    for (let k = id; k >= 0; k = this.parent[k]) out.push(this.nodes[k]);
    return out.reverse();
  }
}

export interface RRTOptions {
  /** Step size δ of the local planner. */
  delta: number;
  /** Probability of using the goal as q_rand. */
  pGoal: number;
  maxNodes: number;
  maxIterations: number;
  /** 'rrt': extend the nearest node toward a uniform sample; 'naive': extend a uniformly chosen node in a random direction. */
  variant?: 'rrt' | 'naive';
}

/**
 * One RRT iteration split into its stages so a UI can reveal them one by one:
 * propose() = sample + nearest + steer, evaluate() = collision-check the local path, commit() = add.
 */
export interface Proposal {
  qRand: Q;
  /** q_rand was the goal (goal bias). */
  fromGoal: boolean;
  near: number;
  qNew: Q;
  /** null until evaluate() has checked the local path q_near → q_new. */
  ok: boolean | null;
  checks: CheckPoint[];
}

export class RRT {
  readonly tree: Tree;
  iterations = 0;
  rejected = 0;
  status: Status = 'searching';
  path: Q[] | null = null;
  goalNode = -1;
  /** Check points of the last attempted connection to the goal. */
  goalChecks: CheckPoint[] = [];

  constructor(
    readonly space: CSpace,
    readonly start: Q,
    readonly goal: Q | null,
    readonly opts: RRTOptions,
    private readonly rng: () => number,
  ) {
    this.tree = new Tree(space, start, Math.max(opts.delta, Math.min(space.W, space.H) / 48));
  }

  get finished(): boolean { return this.status !== 'searching'; }

  /** Stages 1–3: sample q_rand (goal with probability p_goal), find q_near, steer by δ. No collision checks yet. */
  propose(): Proposal {
    const sp = this.space, T = this.tree;
    if (this.opts.variant === 'naive') {
      const near = Math.min(T.size - 1, Math.floor(this.rng() * T.size));
      const a = this.rng() * 2 * Math.PI;
      const from = T.nodes[near];
      const qNew = sp.wrap([from[0] + this.opts.delta * Math.cos(a), from[1] + this.opts.delta * Math.sin(a)]);
      return { qRand: qNew, fromGoal: false, near, qNew, ok: null, checks: [] };
    }
    const fromGoal = !!this.goal && this.rng() < this.opts.pGoal;
    const qRand: Q = fromGoal ? [this.goal![0], this.goal![1]] : sp.sample(this.rng);
    const near = T.nearest(qRand);
    const qNew = sp.steer(T.nodes[near], qRand, this.opts.delta);
    return { qRand, fromGoal, near, qNew, ok: null, checks: [] };
  }

  /** Stage 4a: collision-check the local path q_near → q_new (a zero-length extension adds nothing). */
  evaluate(p: Proposal): boolean {
    if (p.ok !== null) return p.ok;
    const from = this.tree.nodes[p.near];
    p.checks = [];
    p.ok = this.space.dist(from, p.qNew) > 1e-9 && this.space.checkEdge(from, p.qNew, p.checks);
    return p.ok;
  }

  /** Stage 4b: add q_new (if its local path is free) and try to connect it to the goal. */
  commit(p: Proposal): void {
    if (this.finished) return;
    this.evaluate(p);
    this.iterations++;
    if (p.ok) {
      const id = this.tree.add(p.qNew, p.near);
      const g = this.goal;
      if (g) {
        const d = this.space.dist(p.qNew, g);
        if (d < 1e-9) {
          this.goalNode = id;
        } else if (d <= this.opts.delta) {
          this.goalChecks = [];
          if (this.space.checkEdge(p.qNew, g, this.goalChecks)) this.goalNode = this.tree.add([g[0], g[1]], id);
        }
        if (this.goalNode >= 0) { this.status = 'success'; this.path = this.tree.pathTo(this.goalNode); return; }
      }
    } else {
      this.rejected++;
    }
    if (this.tree.size >= this.opts.maxNodes || this.iterations >= this.opts.maxIterations) this.status = this.goal ? 'failed' : 'limit';
  }

  /** One full iteration (propose + commit); returns the proposal, or null when already finished. */
  step(): Proposal | null {
    if (this.finished) return null;
    const p = this.propose();
    this.commit(p);
    return p;
  }

  run(maxIterations = Infinity): void {
    for (let k = 0; k < maxIterations && !this.finished; k++) this.step();
  }
}

export type ExtendStatus = 'reached' | 'advanced' | 'trapped';

export interface ConnectSegment { tree: 0 | 1; from: Q; to: Q; ok: boolean }

/** Bidirectional RRT (RRT-Connect): extend one tree toward a sample, then greedily connect the other tree to it. */
export class RRTConnect {
  readonly trees: [Tree, Tree];
  iterations = 0;
  status: Status = 'searching';
  path: Q[] | null = null;
  /** Ids of the meeting node in tree 0 (start) and tree 1 (goal). */
  meet: [number, number] | null = null;
  /** Segments tried during the last iteration (for highlighting). */
  last: ConnectSegment[] = [];
  lastSample: Q | null = null;
  private active: 0 | 1 = 0;

  constructor(
    readonly space: CSpace,
    readonly start: Q,
    readonly goal: Q,
    readonly opts: { delta: number; maxNodes: number; maxIterations: number },
    private readonly rng: () => number,
  ) {
    const cell = Math.max(opts.delta, Math.min(space.W, space.H) / 48);
    this.trees = [new Tree(space, start, cell), new Tree(space, goal, cell)];
  }

  get finished(): boolean { return this.status !== 'searching'; }
  get size(): number { return this.trees[0].size + this.trees[1].size; }

  private extend(t: 0 | 1, q: Q): { status: ExtendStatus; id: number } {
    const T = this.trees[t];
    const near = T.nearest(q);
    const from = T.nodes[near];
    const qNew = this.space.steer(from, q, this.opts.delta);
    if (this.space.dist(from, qNew) < 1e-12) return { status: 'reached', id: near };
    const ok = this.space.checkEdge(from, qNew);
    this.last.push({ tree: t, from, to: qNew, ok });
    if (!ok) return { status: 'trapped', id: near };
    const id = T.add(qNew, near);
    return { status: this.space.dist(qNew, q) < 1e-9 ? 'reached' : 'advanced', id };
  }

  step(): void {
    if (this.finished) return;
    this.iterations++;
    this.last = [];
    const a = this.active, b = (1 - a) as 0 | 1;
    const qRand = this.space.sample(this.rng);
    this.lastSample = qRand;
    const ext = this.extend(a, qRand);
    if (ext.status !== 'trapped') {
      const target = this.trees[a].nodes[ext.id];
      let r: { status: ExtendStatus; id: number };
      let guard = 0;
      do { r = this.extend(b, target); } while (r.status === 'advanced' && ++guard < 100000);
      if (r.status === 'reached') {
        this.meet = a === 0 ? [ext.id, r.id] : [r.id, ext.id];
        const p0 = this.trees[0].pathTo(this.meet[0]);
        const p1 = this.trees[1].pathTo(this.meet[1]).reverse();
        this.path = p0.concat(p1.slice(1));
        this.status = 'success';
        return;
      }
    }
    this.active = b;
    if (this.size >= this.opts.maxNodes || this.iterations >= this.opts.maxIterations) this.status = 'failed';
  }

  run(maxIterations = Infinity): void {
    for (let k = 0; k < maxIterations && !this.finished; k++) this.step();
  }
}

export interface PRMOptions {
  /** Number of collision-free roadmap nodes to sample. */
  n: number;
  /** Connection radius. */
  radius: number;
  /** At most this many nearest neighbours are tried per node. */
  kMax: number;
}

/** Probabilistic roadmap, built incrementally (sampling phase, then connection phase). */
export class PRM {
  readonly nodes: Q[] = [];
  readonly adj: number[][] = [];
  readonly edges: [number, number][] = [];
  readonly index: GridIndex;
  phase: 'sample' | 'connect' | 'done' = 'sample';
  rejectedSamples = 0;
  attempts = 0;
  blocked = 0;
  /** Edges tried during the last call to step() (for highlighting). */
  lastTried: { a: number; b: number; ok: boolean }[] = [];
  /** Collision checks spent on the build. */
  buildChecks = 0;
  private cursor = 0;
  private tried = new Set<number>();
  private uf: number[] = [];
  private compCache: number[] | null = null;

  constructor(readonly space: CSpace, readonly opts: PRMOptions, private readonly rng: () => number) {
    this.index = new GridIndex(space, this.nodes, Math.max(opts.radius / 2, Math.min(space.W, space.H) / 64));
  }

  private find(i: number): number {
    while (this.uf[i] !== i) { this.uf[i] = this.uf[this.uf[i]]; i = this.uf[i]; }
    return i;
  }

  /** Neighbour candidates of node i: within the radius, nearest first, at most kMax. */
  neighbours(q: Q, self = -1): number[] {
    const ids = this.index.within(q, this.opts.radius).filter((j) => j !== self);
    ids.sort((u, v) => this.space.dist(q, this.nodes[u]) - this.space.dist(q, this.nodes[v]));
    return ids.slice(0, this.opts.kMax);
  }

  /** Do up to `work` units (one sample or one node's connections each). Returns false once the roadmap is complete. */
  step(work = 1): boolean {
    this.lastTried = [];
    const c0 = this.space.checks;
    for (let w = 0; w < work && this.phase !== 'done'; w++) {
      if (this.phase === 'sample') {
        const q = this.space.sample(this.rng);
        if (this.space.isFree(q)) {
          this.nodes.push(q); this.adj.push([]); this.uf.push(this.nodes.length - 1);
          this.index.insert(this.nodes.length - 1);
        } else this.rejectedSamples++;
        if (this.nodes.length >= this.opts.n || this.rejectedSamples > 50 * this.opts.n) this.phase = this.nodes.length ? 'connect' : 'done';
      } else {
        const i = this.cursor++;
        for (const j of this.neighbours(this.nodes[i], i)) {
          const key = Math.min(i, j) * 1048576 + Math.max(i, j);
          if (this.tried.has(key)) continue;
          this.tried.add(key);
          this.attempts++;
          const ok = this.space.checkEdge(this.nodes[i], this.nodes[j]);
          this.lastTried.push({ a: i, b: j, ok });
          if (ok) {
            this.edges.push([i, j]); this.adj[i].push(j); this.adj[j].push(i);
            const ri = this.find(i), rj = this.find(j);
            if (ri !== rj) this.uf[ri] = rj;
          } else this.blocked++;
        }
        if (this.cursor >= this.nodes.length) this.phase = 'done';
      }
    }
    this.buildChecks += this.space.checks - c0;
    this.compCache = null;
    return this.phase !== 'done';
  }

  finish(): void { while (this.step(64)); }

  /** Connected-component label per node (labels are 0..count−1, largest component first). */
  components(): { labels: number[]; count: number; sizes: number[] } {
    const roots = this.nodes.map((_, i) => this.find(i));
    const sizeByRoot = new Map<number, number>();
    for (const r of roots) sizeByRoot.set(r, (sizeByRoot.get(r) ?? 0) + 1);
    const order = [...sizeByRoot.entries()].sort((a, b) => b[1] - a[1]);
    const label = new Map<number, number>();
    order.forEach(([r], k) => label.set(r, k));
    this.compCache = roots.map((r) => label.get(r)!);
    return { labels: this.compCache, count: order.length, sizes: order.map((o) => o[1]) };
  }

  sameComponent(i: number, j: number): boolean { return this.find(i) === this.find(j); }
}

export interface QueryResult {
  path: Q[] | null;
  /** Collision checks spent on this query (connecting start and goal to the roadmap). */
  checks: number;
  /** Roadmap nodes that start / goal could be connected to. */
  startLinks: number[];
  goalLinks: number[];
  reason: 'ok' | 'start' | 'goal' | 'disconnected';
  /** Nodes popped by A*. */
  expanded: number;
  /** Roadmap node ids along the path (without start/goal). */
  nodeIds: number[];
}

/** Binary min-heap on (key, value) pairs. */
class MinHeap {
  private k: number[] = [];
  private v: number[] = [];
  get size(): number { return this.k.length; }
  push(key: number, val: number): void {
    const k = this.k, v = this.v;
    k.push(key); v.push(val);
    let i = k.length - 1;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (k[p] <= k[i]) break;
      [k[p], k[i]] = [k[i], k[p]]; [v[p], v[i]] = [v[i], v[p]];
      i = p;
    }
  }
  pop(): [number, number] {
    const k = this.k, v = this.v;
    const top: [number, number] = [k[0], v[0]];
    const lk = k.pop()!, lv = v.pop()!;
    if (k.length) {
      k[0] = lk; v[0] = lv;
      let i = 0;
      for (;;) {
        const l = 2 * i + 1, r = l + 1;
        let m = i;
        if (l < k.length && k[l] < k[m]) m = l;
        if (r < k.length && k[r] < k[m]) m = r;
        if (m === i) break;
        [k[m], k[i]] = [k[i], k[m]]; [v[m], v[i]] = [v[i], v[m]];
        i = m;
      }
    }
    return top;
  }
}

/** Answer one query on a built roadmap: connect start and goal to nearby nodes, then A* on the graph. */
export function prmQuery(prm: PRM, start: Q, goal: Q): QueryResult {
  const sp = prm.space;
  const c0 = sp.checks;
  const n = prm.nodes.length;
  const S = n, G = n + 1;
  const link = (q: Q): number[] => prm.neighbours(q).filter((j) => sp.checkEdge(q, prm.nodes[j]));
  const startLinks = sp.isFree(start) ? link(start) : [];
  const goalLinks = sp.isFree(goal) ? link(goal) : [];
  const direct = sp.dist(start, goal) <= prm.opts.radius && sp.freeQuiet(start) && sp.freeQuiet(goal) && sp.checkEdge(start, goal);
  const base = { checks: 0, startLinks, goalLinks, expanded: 0, nodeIds: [] as number[] };
  if (!direct && !startLinks.length) return { ...base, checks: sp.checks - c0, path: null, reason: 'start' };
  if (!direct && !goalLinks.length) return { ...base, checks: sp.checks - c0, path: null, reason: 'goal' };
  const pos = (i: number): Q => (i === S ? start : i === G ? goal : prm.nodes[i]);
  const goalSet = new Set(goalLinks);
  const nbrs = (i: number): number[] => {
    if (i === S) return direct ? [...startLinks, G] : startLinks;
    const out = prm.adj[i].slice();
    if (goalSet.has(i)) out.push(G);
    return out;
  };
  const g = new Float64Array(n + 2).fill(Infinity);
  const came = new Int32Array(n + 2).fill(-1);
  const closed = new Uint8Array(n + 2);
  const heap = new MinHeap();
  g[S] = 0;
  heap.push(sp.dist(start, goal), S);
  let expanded = 0;
  while (heap.size) {
    const [, u] = heap.pop();
    if (closed[u]) continue;
    closed[u] = 1;
    expanded++;
    if (u === G) break;
    for (const v of nbrs(u)) {
      if (closed[v]) continue;
      const nd = g[u] + sp.dist(pos(u), pos(v));
      if (nd < g[v]) { g[v] = nd; came[v] = u; heap.push(nd + sp.dist(pos(v), goal), v); }
    }
  }
  const checks = sp.checks - c0;
  if (!Number.isFinite(g[G])) return { ...base, checks, path: null, reason: 'disconnected', expanded };
  const ids: number[] = [];
  for (let k = G; k >= 0; k = came[k]) ids.push(k);
  ids.reverse();
  return { ...base, checks, path: ids.map(pos), reason: 'ok', expanded, nodeIds: ids.filter((i) => i < n) };
}

export function pathLength(space: CSpace, path: readonly Q[]): number {
  let L = 0;
  for (let i = 0; i + 1 < path.length; i++) L += space.dist(path[i], path[i + 1]);
  return L;
}

/** Point at arc length u along the path, with the index of the segment it lies on. */
export function pointAlong(space: CSpace, path: readonly Q[], u: number): { q: Q; seg: number } {
  let acc = 0;
  for (let i = 0; i + 1 < path.length; i++) {
    const L = space.dist(path[i], path[i + 1]);
    if (u <= acc + L || i + 2 === path.length) {
      const s = L > 1e-12 ? Math.min(1, Math.max(0, (u - acc) / L)) : 0;
      return { q: space.interp(path[i], path[i + 1], s), seg: i };
    }
    acc += L;
  }
  return { q: path[0], seg: 0 };
}

/**
 * Shortcut smoothing: pick two random points along the path; if the straight local path between
 * them is collision-free, replace everything in between. Returns the new path and how many
 * shortcuts were accepted.
 */
export function shortcut(space: CSpace, path: readonly Q[], attempts: number, rng: () => number): { path: Q[]; accepted: number } {
  let p = path.map((q) => [q[0], q[1]] as Q);
  let accepted = 0;
  for (let k = 0; k < attempts && p.length > 2; k++) {
    const L = pathLength(space, p);
    let u1 = rng() * L, u2 = rng() * L;
    if (u1 > u2) [u1, u2] = [u2, u1];
    const a = pointAlong(space, p, u1), b = pointAlong(space, p, u2);
    if (b.seg <= a.seg) continue;
    if (!space.checkEdge(a.q, b.q)) continue;
    const next: Q[] = [...p.slice(0, a.seg + 1), a.q, b.q, ...p.slice(b.seg + 1)];
    // drop (near-)duplicate consecutive points
    p = next.filter((q, i) => i === 0 || space.dist(q, next[i - 1]) > 1e-9);
    accepted++;
  }
  return { path: p, accepted };
}

/** Is every local path of `path` collision-free when checked at spacing `eps`? (uses quiet queries) */
export function pathIsFree(space: CSpace, path: readonly Q[], eps: number): boolean {
  for (let i = 0; i + 1 < path.length; i++) {
    const n = Math.max(1, Math.ceil(space.dist(path[i], path[i + 1]) / eps));
    for (let k = 0; k <= n; k++) if (!space.freeQuiet(space.interp(path[i], path[i + 1], k / n))) return false;
  }
  return true;
}

/** Fraction of the free coarse cells (nx × ny, judged at their centres) that contain at least one of the points. */
export function coverage(space: CSpace, pts: readonly Q[], nx: number, ny: number): number {
  const b = space.bounds;
  const hit = new Uint8Array(nx * ny);
  for (const q of pts) {
    const i = Math.min(nx - 1, Math.max(0, Math.floor(((q[0] - b.xmin) / space.W) * nx)));
    const j = Math.min(ny - 1, Math.max(0, Math.floor(((q[1] - b.ymin) / space.H) * ny)));
    hit[j * nx + i] = 1;
  }
  let free = 0, covered = 0;
  for (let j = 0; j < ny; j++) {
    for (let i = 0; i < nx; i++) {
      const c: Q = [b.xmin + ((i + 0.5) / nx) * space.W, b.ymin + ((j + 0.5) / ny) * space.H];
      if (!space.freeQuiet(c)) continue;
      free++;
      if (hit[j * nx + i]) covered++;
    }
  }
  return free ? covered / free : 0;
}

export interface TrialResult { success: boolean; iterations: number; nodes: number; checks: number; length: number }

/** One RRT / RRT-Connect run with a fixed seed and iteration budget (for success-rate experiments). */
export function rrtTrial(space: CSpace, start: Q, goal: Q, o: { delta: number; pGoal: number; budget: number; connect: boolean }, seed: number): TrialResult {
  space.checks = 0;
  const rng = mulberry32(seed);
  if (o.connect) {
    const p = new RRTConnect(space, start, goal, { delta: o.delta, maxNodes: 20000, maxIterations: o.budget }, rng);
    p.run();
    return { success: p.status === 'success', iterations: p.iterations, nodes: p.size, checks: space.checks, length: p.path ? pathLength(space, p.path) : NaN };
  }
  const p = new RRT(space, start, goal, { delta: o.delta, pGoal: o.pGoal, maxNodes: 20000, maxIterations: o.budget }, rng);
  p.run();
  return { success: p.status === 'success', iterations: p.iterations, nodes: p.tree.size, checks: space.checks, length: p.path ? pathLength(space, p.path) : NaN };
}

/** Build a PRM with n nodes and answer one query (for success-rate experiments). */
export function prmTrial(space: CSpace, start: Q, goal: Q, o: PRMOptions, seed: number): TrialResult {
  space.checks = 0;
  const prm = new PRM(space, o, mulberry32(seed));
  prm.finish();
  const r = prmQuery(prm, start, goal);
  return { success: !!r.path, iterations: prm.nodes.length, nodes: prm.nodes.length, checks: space.checks, length: r.path ? pathLength(space, r.path) : NaN };
}
