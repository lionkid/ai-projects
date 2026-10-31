import { describe, expect, it } from 'vitest';
import {
  CSpace, GridIndex, PRM, RRT, RRTConnect, coverage, mulberry32, pathIsFree, pathLength, prmQuery, prmTrial, rrtTrial, shortcut,
  type Q,
} from './planners';
import { ARM_OBSTACLES, armFree, makeMap, makeSpace, narrowObstacles, pointFree, segmentHits } from './maps';

const box = (free: (q: Q) => boolean = () => true, eps = 0.05) => new CSpace({ xmin: 0, xmax: 10, ymin: 0, ymax: 10 }, false, free, eps);
const torus = (free: (q: Q) => boolean = () => true, eps = 0.02) =>
  new CSpace({ xmin: -Math.PI, xmax: Math.PI, ymin: -Math.PI, ymax: Math.PI }, true, free, eps);

describe('CSpace metric', () => {
  it('wraps differences on the torus and not in a box', () => {
    const T = torus();
    expect(T.dist([-3, 0], [3, 0])).toBeCloseTo(2 * Math.PI - 6, 12);
    expect(T.diff([-3, 0], [3, 0])[0]).toBeCloseTo(-(2 * Math.PI - 6), 12);
    expect(box().dist([1, 0], [9, 0])).toBeCloseTo(8, 12);
  });
  it('steers exactly δ toward the sample, or lands on it when closer than δ', () => {
    const B = box();
    const q = B.steer([1, 1], [4, 5], 0.5);
    expect(B.dist([1, 1], q)).toBeCloseTo(0.5, 12);
    expect(q[0]).toBeCloseTo(1 + 0.3, 12);
    expect(B.steer([1, 1], [1.2, 1], 0.5)).toEqual([1.2, 1]);
    // across the seam on the torus the step goes the short way and is wrapped back into the square
    const T = torus();
    const r = T.steer([3, 0], [-3, 0], 0.2);
    expect(r[0]).toBeCloseTo(3.2 - 2 * Math.PI, 12);
  });
  it('checks local paths at spacing ≤ ε and counts every query', () => {
    const wall = (q: Q) => !(q[0] > 4.9 && q[0] < 5.1);
    const B = box(wall, 0.05);
    expect(B.checkEdge([1, 1], [9, 1])).toBe(false);
    expect(B.checkEdge([1, 1], [4, 1])).toBe(true);
    expect(B.checks).toBeGreaterThan(60);
    // a coarse spacing can jump over a thin wall: the exact test still sees it
    const coarse = box(wall, 0.5);
    expect(coarse.checkEdge([4.75, 2], [5.25, 2])).toBe(true);
  });
});

describe('GridIndex', () => {
  for (const kind of ['box', 'torus'] as const) {
    it(`nearest / within agree with brute force (${kind})`, () => {
      const sp = kind === 'box' ? box() : torus();
      const rng = mulberry32(7);
      const pts: Q[] = [];
      const idx = new GridIndex(sp, pts, kind === 'box' ? 0.7 : 0.3);
      for (let k = 0; k < 400; k++) { pts.push(sp.sample(rng)); idx.insert(k); }
      for (let t = 0; t < 300; t++) {
        const q = sp.sample(rng);
        let best = -1, bd = Infinity;
        pts.forEach((p, i) => { const d = sp.dist(q, p); if (d < bd) { bd = d; best = i; } });
        expect(sp.dist(q, pts[idx.nearest(q)])).toBeCloseTo(bd, 12);
        void best;
        const r = kind === 'box' ? 1.1 : 0.6;
        const brute = pts.map((p, i) => (sp.dist(q, p) <= r ? i : -1)).filter((i) => i >= 0).sort((a, b) => a - b);
        expect(idx.within(q, r).sort((a, b) => a - b)).toEqual(brute);
      }
    });
  }
  it('finds neighbours across the torus seam', () => {
    const sp = torus();
    const pts: Q[] = [[3.1, 0], [0, 0]];
    const idx = new GridIndex(sp, pts, 0.25);
    idx.insert(0); idx.insert(1);
    expect(idx.nearest([-3.1, 0])).toBe(0);
  });
});

describe('RRT', () => {
  it('finds a collision-free path from start to goal on the cluttered map', () => {
    const m = makeMap('clutter');
    const sp = makeSpace(m, 0.05);
    const rrt = new RRT(sp, m.start, m.goal, { delta: 0.6, pGoal: 0.05, maxNodes: 4000, maxIterations: 4000 }, mulberry32(3));
    rrt.run();
    expect(rrt.status).toBe('success');
    const p = rrt.path!;
    expect(p[0]).toEqual(m.start);
    expect(p[p.length - 1]).toEqual(m.goal);
    expect(pathIsFree(sp, p, 0.01)).toBe(true);
    for (let i = 0; i + 1 < p.length; i++) expect(segmentHits(m.obstacles, p[i], p[i + 1])).toBe(false);
  });
  it('stages: propose does no collision checks, evaluate/commit do', () => {
    const m = makeMap('clutter');
    const sp = makeSpace(m, 0.05);
    const rrt = new RRT(sp, m.start, m.goal, { delta: 0.6, pGoal: 0, maxNodes: 100, maxIterations: 100 }, mulberry32(1));
    const p = rrt.propose();
    expect(sp.checks).toBe(0);
    expect(p.ok).toBeNull();
    expect(sp.dist(rrt.tree.nodes[p.near], p.qNew)).toBeLessThanOrEqual(0.6 + 1e-12);
    rrt.commit(p);
    expect(sp.checks).toBeGreaterThan(0);
    expect(rrt.iterations).toBe(1);
    expect(rrt.tree.size).toBe(p.ok ? 2 : 1);
  });
  it('p_goal = 1 is greedy and gets stuck behind the wall', () => {
    const m = makeMap('wall');
    const sp = makeSpace(m, 0.05);
    const rrt = new RRT(sp, m.start, m.goal, { delta: 0.6, pGoal: 1, maxNodes: 4000, maxIterations: 1500 }, mulberry32(5));
    rrt.run();
    expect(rrt.status).toBe('failed');
    expect(rrt.tree.size).toBeLessThan(15);
    expect(rrt.rejected).toBeGreaterThan(1400);
  });
  it('explores faster than the naive random tree (Voronoi bias)', () => {
    const m = makeMap('empty');
    let covR = 0, covN = 0;
    for (let s = 1; s <= 5; s++) {
      const sp = makeSpace(m, 0.05);
      const a = new RRT(sp, m.start, null, { delta: 0.5, pGoal: 0, maxNodes: 200, maxIterations: 5000 }, mulberry32(s));
      a.run();
      const b = new RRT(sp, m.start, null, { delta: 0.5, pGoal: 0, maxNodes: 200, maxIterations: 5000, variant: 'naive' }, mulberry32(s));
      b.run();
      expect(a.status).toBe('limit');
      covR += coverage(sp, a.tree.nodes, 24, 18);
      covN += coverage(sp, b.tree.nodes, 24, 18);
    }
    expect(covR).toBeGreaterThan(1.8 * covN);
  });
  it('plans through the ±π seam on the 2R arm torus but fails when the seam is treated as a wall', () => {
    const m = makeMap('arm');
    const T = makeSpace(m, 0.02, true);
    const r1 = rrtTrial(T, m.start, m.goal!, { delta: 0.25, pGoal: 0.05, budget: 4000, connect: false }, 2);
    expect(r1.success).toBe(true);
    const Bx = makeSpace(m, 0.02, false);
    const r2 = rrtTrial(Bx, m.start, m.goal!, { delta: 0.25, pGoal: 0.05, budget: 4000, connect: false }, 2);
    expect(r2.success).toBe(false);
    expect(armFree(ARM_OBSTACLES, m.start) && armFree(ARM_OBSTACLES, m.goal!)).toBe(true);
  });
});

describe('RRT-Connect', () => {
  it('connects the two trees on the maze and returns a valid start→goal path', () => {
    const m = makeMap('maze');
    const sp = makeSpace(m, 0.05);
    const c = new RRTConnect(sp, m.start, m.goal!, { delta: 0.6, maxNodes: 20000, maxIterations: 4000 }, mulberry32(4));
    c.run();
    expect(c.status).toBe('success');
    const p = c.path!;
    expect(p[0]).toEqual(m.start);
    expect(p[p.length - 1]).toEqual(m.goal);
    expect(pathIsFree(sp, p, 0.01)).toBe(true);
  });
});

describe('PRM', () => {
  it('builds a roadmap once and answers several queries with few extra checks', () => {
    const m = makeMap('clutter');
    const sp = makeSpace(m, 0.05);
    const prm = new PRM(sp, { n: 200, radius: 1.5, kMax: 15 }, mulberry32(11));
    prm.finish();
    expect(prm.nodes.length).toBe(200);
    for (const q of prm.nodes) expect(pointFree(m.obstacles, q)).toBe(true);
    const queries: [Q, Q][] = [[m.start, m.goal!], [[1, 8], [11.5, 4.5]], [[4, 3.5], [10.5, 5]]];
    for (const [s, g] of queries) {
      const r = prmQuery(prm, s, g);
      expect(r.reason).toBe('ok');
      expect(pathIsFree(sp, r.path!, 0.01)).toBe(true);
      expect(r.checks).toBeLessThan(prm.buildChecks / 20);
    }
  });
  it('reports disconnected components across a closed wall', () => {
    const closed = (q: Q) => !(q[1] > 4 && q[1] < 5);
    const sp = box(closed);
    const prm = new PRM(sp, { n: 150, radius: 1.5, kMax: 15 }, mulberry32(2));
    prm.finish();
    const r = prmQuery(prm, [5, 1], [5, 9]);
    expect(r.path).toBeNull();
    expect(r.reason).toBe('disconnected');
    expect(prm.components().count).toBeGreaterThanOrEqual(2);
  });
  it('success rate in the narrow passage grows with N (probabilistic completeness)', () => {
    const m = makeMap('narrow', 0.6);
    const rate = (n: number) => {
      let ok = 0;
      for (let s = 1; s <= 10; s++) if (prmTrial(makeSpace(m, 0.05), m.start, m.goal!, { n, radius: 1.5, kMax: 15 }, s).success) ok++;
      return ok / 10;
    };
    const lo = rate(60), hi = rate(500);
    expect(hi).toBeGreaterThan(lo);
    expect(hi).toBeGreaterThanOrEqual(0.7);
    expect(narrowObstacles(0.6)).toHaveLength(2);
  });
});

describe('shortcut smoothing', () => {
  it('never lengthens the path and keeps it collision-free', () => {
    const m = makeMap('clutter');
    const sp = makeSpace(m, 0.05);
    const rrt = new RRT(sp, m.start, m.goal, { delta: 0.6, pGoal: 0.05, maxNodes: 4000, maxIterations: 4000 }, mulberry32(9));
    rrt.run();
    const raw = rrt.path!;
    const s = shortcut(sp, raw, 80, mulberry32(1));
    expect(s.accepted).toBeGreaterThan(0);
    expect(pathLength(sp, s.path)).toBeLessThan(pathLength(sp, raw));
    expect(s.path[0]).toEqual(raw[0]);
    expect(s.path[s.path.length - 1]).toEqual(raw[raw.length - 1]);
    expect(pathIsFree(sp, s.path, 0.01)).toBe(true);
  });
});
