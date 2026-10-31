import { describe, expect, it } from 'vitest';
import {
  bestFirst, cellDelta, cellIndex, descend, downhill, downsample, edgesFrom, forNeighbors, heuristic, mulberry32, nearestFree,
  paintMaster, pathCost, rrtPreview, wavefront, type Conn, type Grid, type HeuristicKind,
} from './grid';
import { MAPS, MASTER, PROBE, type MapKey } from './maps';

const cellOf = (n: number, u: readonly [number, number]): number => Math.min(n - 1, Math.floor(u[1] * n)) * n + Math.min(n - 1, Math.floor(u[0] * n));
function mapGrid(key: MapKey, n: number, wrap?: boolean): { g: Grid; S: number; G: number } {
  const m = MAPS.find((x) => x.key === key)!;
  return { g: { n, blocked: downsample(m.build(), MASTER, n), wrap: wrap ?? m.wrap }, S: cellOf(n, m.S), G: cellOf(n, m.G) };
}
function randomGrid(n: number, density: number, seed: number, wrap = false): Grid {
  const r = mulberry32(seed);
  const blocked = new Uint8Array(n * n);
  for (let k = 0; k < n * n; k++) blocked[k] = r() < density ? 1 : 0;
  return { n, blocked, wrap };
}
const neighbours = (g: Grid, k: number, conn: Conn): number[] => { const out: number[] = []; forNeighbors(g, k, conn, (nb) => out.push(nb)); return out; };
/** Path is a chain of free, traversable neighbours from a to b. */
function validPath(g: Grid, p: number[], conn: Conn, a: number, b: number): boolean {
  if (p[0] !== a || p[p.length - 1] !== b) return false;
  for (let s = 0; s + 1 < p.length; s++) if (!neighbours(g, p[s], conn).includes(p[s + 1])) return false;
  return p.every((k) => !g.blocked[k]);
}

describe('neighbourhoods', () => {
  const empty: Grid = { n: 5, blocked: new Uint8Array(25), wrap: false };
  it('4 / 8 neighbours inside, fewer at the border, all on a torus', () => {
    expect(neighbours(empty, cellIndex(5, 2, 2), 4)).toHaveLength(4);
    expect(neighbours(empty, cellIndex(5, 2, 2), 8)).toHaveLength(8);
    expect(neighbours(empty, cellIndex(5, 0, 0), 8)).toHaveLength(3);
    expect(neighbours({ ...empty, wrap: true }, cellIndex(5, 0, 0), 8)).toHaveLength(8);
    expect(neighbours({ ...empty, wrap: true }, cellIndex(5, 0, 0), 4)).toContain(cellIndex(5, 4, 0));
  });
  it('a diagonal that would squeeze past an obstacle corner is removed', () => {
    const g: Grid = { n: 5, blocked: new Uint8Array(25), wrap: false };
    g.blocked[cellIndex(5, 3, 2)] = 1; // east of the centre
    const e = edgesFrom(g, cellIndex(5, 2, 2), 8);
    expect(e.find((x) => x.dir === 0)!.status).toBe('blocked');
    expect(e.find((x) => x.dir === 4)!.status).toBe('corner'); // NE
    expect(e.find((x) => x.dir === 7)!.status).toBe('corner'); // SE
    expect(e.filter((x) => x.status === 'ok')).toHaveLength(5);
    expect(neighbours(g, cellIndex(5, 2, 2), 8)).toHaveLength(5);
  });
  it("the step-1 probe cell shows an obstacle edge and a corner-cut diagonal", () => {
    const { g } = mapGrid('blocks', 16);
    const e = edgesFrom(g, cellOf(16, PROBE), 8);
    expect(e.some((x) => x.status === 'blocked')).toBe(true);
    expect(e.some((x) => x.status === 'corner')).toBe(true);
    expect(e.some((x) => x.status === 'ok')).toBe(true);
  });
  it('torus deltas take the short way round', () => {
    const g: Grid = { n: 10, blocked: new Uint8Array(100), wrap: true };
    expect(cellDelta(g, cellIndex(10, 0, 0), cellIndex(10, 9, 8))).toEqual([1, 2]);
  });
});

describe('heuristics', () => {
  for (const seed of [1, 2, 3]) {
    it(`admissibility on a random map (seed ${seed})`, () => {
      const g = randomGrid(24, 0.25, seed);
      let goal = cellIndex(24, 20, 20); g.blocked[goal] = 0;
      goal = nearestFree(g, goal);
      const d8 = wavefront(g, goal, 8).g, d4 = wavefront(g, goal, 4).g;
      let manhattanOver = false;
      for (let k = 0; k < 24 * 24; k++) {
        if (Number.isFinite(d8[k])) {
          expect(heuristic(g, k, goal, 'euclid')).toBeLessThanOrEqual(d8[k] + 1e-9);
          expect(heuristic(g, k, goal, 'octile')).toBeLessThanOrEqual(d8[k] + 1e-9);
          if (heuristic(g, k, goal, 'manhattan') > d8[k] + 1e-9) manhattanOver = true;
        }
        if (Number.isFinite(d4[k])) expect(heuristic(g, k, goal, 'manhattan')).toBeLessThanOrEqual(d4[k] + 1e-9);
      }
      expect(manhattanOver).toBe(true); // Manhattan overestimates on the 8-connected grid
    });
  }
});

describe('best-first search', () => {
  it('A* (admissible h) returns the Dijkstra cost and never expands more nodes', () => {
    for (let seed = 1; seed <= 12; seed++) {
      const g = randomGrid(32, 0.28, seed);
      const s = nearestFree(g, cellIndex(32, 2, 3)), t = nearestFree(g, cellIndex(32, 29, 27));
      for (const conn of [4, 8] as Conn[]) {
        const d = bestFirst(g, s, t, { conn, eps: 0 });
        const hs: HeuristicKind[] = conn === 8 ? ['euclid', 'octile'] : ['euclid', 'octile', 'manhattan'];
        for (const h of hs) {
          const a = bestFirst(g, s, t, { conn, eps: 1, heuristic: h });
          if (!Number.isFinite(d.cost)) { expect(a.cost).toBe(Infinity); continue; }
          expect(a.cost).toBeCloseTo(d.cost, 9);
          expect(a.expanded).toBeLessThanOrEqual(d.expanded);
          expect(validPath(g, a.path, conn, s, t)).toBe(true);
          expect(pathCost(g, a.path)).toBeCloseTo(a.cost, 9);
        }
      }
    }
  });
  it('weighted A* stays within ε times the optimal cost', () => {
    for (let seed = 20; seed < 32; seed++) {
      const g = randomGrid(32, 0.3, seed);
      const s = nearestFree(g, cellIndex(32, 1, 1)), t = nearestFree(g, cellIndex(32, 30, 30));
      const opt = bestFirst(g, s, t, { conn: 8, eps: 0 }).cost;
      if (!Number.isFinite(opt)) continue;
      for (const eps of [1.5, 2, 3]) for (const h of ['euclid', 'octile'] as HeuristicKind[]) {
        const a = bestFirst(g, s, t, { conn: 8, eps, heuristic: h });
        expect(a.cost).toBeLessThanOrEqual(eps * opt + 1e-9);
        expect(a.cost).toBeGreaterThanOrEqual(opt - 1e-9);
      }
    }
  });
  it('records a consistent replay trace', () => {
    const { g, S, G } = mapGrid('blocks', 32);
    const t = bestFirst(g, S, G, { conn: 8, eps: 1, heuristic: 'euclid' });
    expect(t.openCount).toHaveLength(t.expanded + 1);
    expect(t.openCount[0]).toBe(1);
    expect(t.goalStep).toBe(t.expanded);
    expect(t.order[t.expanded - 1]).toBe(G);
    for (let e = 0; e < t.expanded; e++) {
      const c = t.order[e];
      expect(t.closedAt[c]).toBe(e);
      expect(t.openedAt[c]).toBeGreaterThanOrEqual(0);
      expect(t.openedAt[c]).toBeLessThanOrEqual(e);
    }
    let open = 0;
    for (let k = 0; k < g.n * g.n; k++) if (t.openedAt[k] >= 0 && t.closedAt[k] < 0) open++;
    expect(t.openCount[t.expanded]).toBe(open);
  });
  it('reports failure when the goal is unreachable', () => {
    const { g, S, G } = mapGrid('gap', 16);
    const t = bestFirst(g, S, G, { conn: 8, eps: 1 });
    expect(t.goalStep).toBe(-1);
    expect(t.path).toEqual([]);
    expect(t.cost).toBe(Infinity);
  });
});

describe('wavefront', () => {
  it('cost-to-go equals the Dijkstra cost and descent follows a shortest path from any start', () => {
    for (const conn of [4, 8] as Conn[]) {
      const { g, G } = mapGrid('utrap', 32);
      const w = wavefront(g, G, conn);
      for (const k of [cellIndex(32, 2, 2), cellIndex(32, 16, 16), cellIndex(32, 30, 3), cellIndex(32, 5, 29)]) {
        const s = nearestFree(g, k);
        const dj = bestFirst(g, s, G, { conn, eps: 0 });
        expect(w.g[s]).toBeCloseTo(dj.cost, 9);
        const p = descend(g, w.g, s, conn);
        expect(validPath(g, p, conn, s, G)).toBe(true);
        expect(pathCost(g, p)).toBeCloseTo(w.g[s], 9);
      }
      expect(downhill(g, w.g, G, conn)).toBe(-1);
    }
  });
  it('4-connected labels are integer step counts (BFS)', () => {
    const { g, G } = mapGrid('utrap', 16);
    const w = wavefront(g, G, 4);
    for (let k = 0; k < 256; k++) if (Number.isFinite(w.g[k])) expect(w.g[k]).toBe(Math.round(w.g[k]));
  });
});

describe('maps and the claims made in the lesson', () => {
  it('S and G are free at every resolution', () => {
    for (const m of MAPS) for (const n of [16, 32, 64, 128]) {
      const { g, S, G } = mapGrid(m.key, n);
      expect(g.blocked[S]).toBe(0);
      expect(g.blocked[G]).toBe(0);
    }
  });
  it('down-sampling is conservative and painting a coarse cell paints its master block', () => {
    const master = MAPS[0].build();
    for (const n of [16, 32, 64]) {
      const c = downsample(master, MASTER, n);
      const f = MASTER / n;
      for (let j = 0; j < n; j++) for (let i = 0; i < n; i++) {
        if (c[j * n + i]) continue;
        for (let J = j * f; J < (j + 1) * f; J++) for (let I = i * f; I < (i + 1) * f; I++) expect(master[J * MASTER + I]).toBe(0);
      }
    }
    const m2 = new Uint8Array(MASTER * MASTER);
    paintMaster(m2, MASTER, 16, 3, 5, 1);
    expect(m2.reduce((a, b) => a + b, 0)).toBe(64);
    expect(downsample(m2, MASTER, 16)[5 * 16 + 3]).toBe(1);
    paintMaster(m2, MASTER, 32, 6, 10, 0);
    expect(downsample(m2, MASTER, 32)[10 * 32 + 6]).toBe(0);
  });
  it("'blocks': A* finds the same cost as Dijkstra with far fewer expansions", () => {
    const { g, S, G } = mapGrid('blocks', 32);
    const d = bestFirst(g, S, G, { conn: 8, eps: 0 });
    const a = bestFirst(g, S, G, { conn: 8, eps: 1, heuristic: 'euclid' });
    expect(a.cost).toBeCloseTo(d.cost, 9);
    expect(a.expanded * 5).toBeLessThan(d.expanded);
  });
  it("'utrap': weighted A* trades optimality for fewer expansions", () => {
    const { g, S, G } = mapGrid('utrap', 32);
    const opt = bestFirst(g, S, G, { conn: 8, eps: 0 });
    const a1 = bestFirst(g, S, G, { conn: 8, eps: 1, heuristic: 'euclid' });
    const a2 = bestFirst(g, S, G, { conn: 8, eps: 2, heuristic: 'euclid' });
    const a3 = bestFirst(g, S, G, { conn: 8, eps: 3, heuristic: 'euclid' });
    expect(a1.cost).toBeCloseTo(opt.cost, 9);
    expect(a2.expanded).toBeLessThan(a1.expanded);
    expect(a3.expanded).toBeLessThan(a1.expanded);
    expect(a2.cost).toBeGreaterThan(opt.cost * 1.03);
    expect(a3.cost).toBeGreaterThan(opt.cost * 1.1);
    // the A* run explores the inside of the U (the heuristic pulls it there)
    const inside = cellIndex(32, 17, 15);
    expect(g.blocked[inside]).toBe(0);
    expect(a1.closedAt[inside]).toBeGreaterThanOrEqual(0);
  });
  it("'gap': resolution complete — closed at 16 × 16, found at 32 × 32 and finer", () => {
    expect(bestFirst(mapGrid('gap', 16).g, mapGrid('gap', 16).S, mapGrid('gap', 16).G, { conn: 8, eps: 1 }).cost).toBe(Infinity);
    for (const n of [32, 64, 128]) {
      const { g, S, G } = mapGrid('gap', n);
      expect(Number.isFinite(bestFirst(g, S, G, { conn: 8, eps: 1 }).cost)).toBe(true);
    }
  });
  it("'cspace2r': the path needs the wrap-around, and Manhattan on 8 neighbours is not admissible there", () => {
    const { g, S, G } = mapGrid('cspace2r', 32);
    const d = bestFirst(g, S, G, { conn: 8, eps: 0 });
    expect(Number.isFinite(d.cost)).toBe(true);
    const wraps = d.path.some((k, s) => s > 0 && Math.abs((k % 32) - (d.path[s - 1] % 32)) > 1);
    expect(wraps).toBe(true);
    const flat = mapGrid('cspace2r', 32, false);
    expect(bestFirst(flat.g, flat.S, flat.G, { conn: 8, eps: 0 }).cost).toBe(Infinity);
    const m = bestFirst(g, S, G, { conn: 8, eps: 1, heuristic: 'manhattan' });
    expect(m.cost).toBeGreaterThan(d.cost * 1.1);
  });
  it('the RRT preview crosses the narrow gap with collision-free edges', () => {
    const m = MAPS.find((x) => x.key === 'gap')!;
    const occ: Grid = { n: MASTER, blocked: m.build(), wrap: false };
    const r = rrtPreview(occ, [m.S[0] * MASTER, m.S[1] * MASTER], [m.G[0] * MASTER, m.G[1] * MASTER], { seed: 7 });
    expect(r.goalNode).toBeGreaterThan(0);
    for (let k = 1; k < r.nodes.length; k++) {
      const a = r.nodes[r.parent[k]], b = r.nodes[k];
      for (let s = 0; s <= 40; s++) {
        const x = a[0] + ((b[0] - a[0]) * s) / 40, y = a[1] + ((b[1] - a[1]) * s) / 40;
        expect(occ.blocked[Math.floor(y) * MASTER + Math.floor(x)]).toBe(0);
      }
    }
  });
});
