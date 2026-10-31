/**
 * Every statement the lesson text, hints and quiz make about a concrete preset is checked here
 * against the planner itself, so the narrative cannot drift away from what the simulation shows.
 */
import { describe, expect, it } from 'vitest';
import { bestFirst, descend, downsample, wavefront, type Conn, type Grid } from './grid';
import { MAPS, MASTER, arm2RHits, cellAngles, cspace2R, type MapKey } from './maps';

const cellOf = (n: number, u: readonly [number, number]): number => Math.min(n - 1, Math.floor(u[1] * n)) * n + Math.min(n - 1, Math.floor(u[0] * n));
function setup(key: MapKey, n: number): { g: Grid; S: number; G: number } {
  const m = MAPS.find((x) => x.key === key)!;
  return { g: { n, blocked: downsample(m.build(), MASTER, n), wrap: m.wrap }, S: cellOf(n, m.S), G: cellOf(n, m.G) };
}

describe('steps 2–3: Dijkstra vs A* on "three blocks" (32 × 32, 8-connected)', () => {
  it('A* returns the same cost with a small fraction of the expansions', () => {
    const { g, S, G } = setup('blocks', 32);
    const d = bestFirst(g, S, G, { conn: 8, eps: 0 });
    const a = bestFirst(g, S, G, { conn: 8, eps: 1, heuristic: 'euclid' });
    expect(a.cost).toBeCloseTo(d.cost, 9);
    expect(d.expanded).toBeGreaterThan(0.85 * 920); // Dijkstra floods almost every free cell
    expect(a.expanded).toBeLessThan(0.15 * d.expanded);
  });
});

describe('step 4 and quiz 2: weighted A* on the U trap (32 × 32, 8-connected)', () => {
  it('ε = 2 expands roughly 40 % fewer nodes and returns a path about 5 % longer, within the ε bound', () => {
    const { g, S, G } = setup('utrap', 32);
    const opt = bestFirst(g, S, G, { conn: 8, eps: 0 }).cost;
    const a1 = bestFirst(g, S, G, { conn: 8, eps: 1, heuristic: 'euclid' });
    const a2 = bestFirst(g, S, G, { conn: 8, eps: 2, heuristic: 'euclid' });
    expect(a1.cost).toBeCloseTo(opt, 9);
    expect(a2.expanded).toBeLessThan(0.65 * a1.expanded);
    expect(a2.expanded).toBeGreaterThan(0.5 * a1.expanded);
    expect(a2.cost / opt).toBeGreaterThan(1.03);
    expect(a2.cost / opt).toBeLessThan(1.08);
    expect(a2.cost).toBeLessThanOrEqual(2 * opt);
  });
  it('the ε sweep is monotone enough for the plot: ε = 0 is Dijkstra, larger ε never expands more than A*', () => {
    const { g, S, G } = setup('utrap', 32);
    const d = bestFirst(g, S, G, { conn: 8, eps: 0 });
    const a0 = bestFirst(g, S, G, { conn: 8, eps: 0, heuristic: 'euclid' });
    expect(a0.expanded).toBe(d.expanded);
    const a1 = bestFirst(g, S, G, { conn: 8, eps: 1, heuristic: 'euclid' });
    for (let e = 1.1; e <= 3.001; e += 0.1) {
      const a = bestFirst(g, S, G, { conn: 8, eps: e, heuristic: 'euclid' });
      expect(a.expanded).toBeLessThanOrEqual(a1.expanded);
      expect(a.cost).toBeLessThanOrEqual(e * d.cost + 1e-9);
    }
  });
});

describe('step 5: wavefront on the 2R C-space (16 × 16, 4-connected torus)', () => {
  const { g, S, G } = setup('cspace2r', 16);
  const w = wavefront(g, G, 4 as Conn);
  it('labels are BFS step counts and the descent from S reaches G', () => {
    expect(Number.isFinite(w.g[S])).toBe(true);
    for (let k = 0; k < 256; k++) if (Number.isFinite(w.g[k])) expect(w.g[k]).toBe(Math.round(w.g[k]));
    const p = descend(g, w.g, S, 4);
    expect(p[p.length - 1]).toBe(G);
    expect(p.length - 1).toBe(w.g[S]);
  });
  it('the wave can only reach S through the θ₁ = ±180° seam (on a flat square S stays unreached)', () => {
    let crosses = 0;
    for (let k = 0; k < 256; k++) {
      const p = w.parent[k];
      if (p >= 0 && Math.abs((k % 16) - (p % 16)) === 15) crosses++;
    }
    expect(crosses).toBeGreaterThan(0);
    const flat = wavefront({ ...g, wrap: false }, G, 4);
    expect(flat.g[S]).toBe(Infinity);
  });
  it('S and G are collision-free arm postures and the quoted numbers fit in a 16 × 16 cell', () => {
    const m = MAPS.find((x) => x.key === 'cspace2r')!;
    expect(arm2RHits([(-40 * Math.PI) / 180, (40 * Math.PI) / 180])).toBe(0);
    expect(arm2RHits([(130 * Math.PI) / 180, (-40 * Math.PI) / 180])).toBe(0);
    expect(m.wrap).toBe(true);
    let maxD = 0;
    for (let k = 0; k < 256; k++) if (Number.isFinite(w.g[k])) maxD = Math.max(maxD, w.g[k]);
    expect(maxD).toBeLessThan(100); // at most two digits per cell
  });
});

describe('step 6 and quiz 4: resolution completeness on the narrow gap', () => {
  it('16 × 16 reports failure while 32 × 32 finds a path', () => {
    const coarse = setup('gap', 16);
    expect(bestFirst(coarse.g, coarse.S, coarse.G, { conn: 8, eps: 1 }).cost).toBe(Infinity);
    const fine = setup('gap', 32);
    expect(Number.isFinite(bestFirst(fine.g, fine.S, fine.G, { conn: 8, eps: 1 }).cost)).toBe(true);
  });
});

describe('quiz 1: Manhattan distance is not admissible on the 8-connected grid', () => {
  it('on "three blocks" it makes A* return a clearly longer path', () => {
    const { g, S, G } = setup('blocks', 32);
    const d = bestFirst(g, S, G, { conn: 8, eps: 0 });
    const m = bestFirst(g, S, G, { conn: 8, eps: 1, heuristic: 'manhattan' });
    expect(m.cost).toBeGreaterThan(1.1 * d.cost);
    // ...while on the 4-connected grid it is admissible and keeps the optimum
    const d4 = bestFirst(g, S, G, { conn: 4, eps: 0 });
    const m4 = bestFirst(g, S, G, { conn: 4, eps: 1, heuristic: 'manhattan' });
    expect(m4.cost).toBeCloseTo(d4.cost, 9);
  });
});

describe('2R arm helpers', () => {
  it('the per-configuration test agrees with the rasterised C-obstacle at every cell centre', () => {
    const M = 32;
    const raster = cspace2R(M);
    for (let k = 0; k < M * M; k++) expect(raster[k]).toBe(arm2RHits(cellAngles(M, k)) ? 1 : 0);
  });
  it('cell angles run from −180° at the left/bottom edge', () => {
    const [t1, t2] = cellAngles(16, 0);
    expect(t1).toBeCloseTo(-Math.PI + Math.PI / 16, 12);
    expect(t2).toBeCloseTo(-Math.PI + Math.PI / 16, 12);
    const [u1, u2] = cellAngles(16, 16 * 15 + 15);
    expect(u1).toBeCloseTo(Math.PI - Math.PI / 16, 12);
    expect(u2).toBeCloseTo(Math.PI - Math.PI / 16, 12);
  });
});
