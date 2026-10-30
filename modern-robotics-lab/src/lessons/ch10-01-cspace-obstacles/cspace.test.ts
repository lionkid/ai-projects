import { describe, expect, it } from 'vitest';
import {
  angleCell, cellAngle, checkPath, circleCover, componentAt, computeCobsGrid, coverClearance, exactClearance, pathAt,
  torusComponents, torusDelta, torusSegmentPieces, type Obstacle,
} from './cspace';

const L: [number, number] = [1, 1];

describe('clearance queries', () => {
  it('exact clearance of a straight arm against a disk beside it', () => {
    const obs: Obstacle[] = [{ c: [1, 0.5], r: 0.2 }];
    const r = exactClearance(L, 0.05, [0, 0], obs);
    expect(r.d).toBeCloseTo(0.5 - 0.2 - 0.05, 9);
    expect(r.obs).toBe(0);
  });
  it('reports penetration as a negative distance and finds the responsible link', () => {
    const obs: Obstacle[] = [{ c: [1.6, 0.1], r: 0.3 }];
    const r = exactClearance(L, 0.05, [0, 0], obs);
    expect(r.d).toBeLessThan(0);
    expect(r.link).toBe(1);
  });
  it('circle cover is conservative: cover clearance never exceeds the exact one', () => {
    const obs: Obstacle[] = [{ c: [0.8, 0.9], r: 0.25 }, { c: [-1.2, -0.3], r: 0.4 }];
    for (let k = 0; k < 200; k++) {
      const q: [number, number] = [-Math.PI + (2 * Math.PI * k) / 200, Math.sin(k) * Math.PI];
      const e = exactClearance(L, 0.06, q, obs).d;
      const c = coverClearance(L, 0.06, q, obs).d;
      // outside the obstacle the cover (a superset of the arm) is never farther away than the arm itself
      if (e > 0) { expect(c).toBeLessThanOrEqual(e + 1e-12); expect(e - c).toBeLessThan(0.15); }
      // a true collision is always reported by the cover
      if (e <= 0) expect(c).toBeLessThanOrEqual(1e-12);
    }
  });
  it('cover circles have the enlarged radius that covers the capsule', () => {
    const cs = circleCover(L, 0.05, [0.3, -0.4], 5);
    expect(cs).toHaveLength(12);
    expect(cs[0].r).toBeCloseTo(Math.hypot(0.05, 0.1), 12);
  });
});

describe('torus helpers', () => {
  it('cell / angle round trip', () => {
    for (let k = 0; k < 40; k++) expect(angleCell(cellAngle(k, 40), 40)).toBe(k);
    expect(angleCell(Math.PI, 40)).toBe(0); // +π wraps to −π
  });
  it('torusDelta takes the short way round', () => {
    const d = torusDelta([3, 0], [-3, 0]);
    expect(d[0]).toBeCloseTo(2 * Math.PI - 6, 12);
  });
  it('a segment crossing the θ₁ = ±π edge is split into two in-square pieces', () => {
    const pieces = torusSegmentPieces([3, 0.2], [-3, 0.4]);
    expect(pieces).toHaveLength(2);
    expect(pieces[0][1][0]).toBeCloseTo(Math.PI, 9);
    expect(pieces[1][0][0]).toBeCloseTo(-Math.PI, 9);
    for (const [a, b] of pieces) for (const p of [a, b]) { expect(p[0]).toBeGreaterThanOrEqual(-Math.PI - 1e-9); expect(p[0]).toBeLessThanOrEqual(Math.PI + 1e-9); }
  });
  it('a segment inside the square is a single piece', () => {
    expect(torusSegmentPieces([0, 0], [1, 1])).toHaveLength(1);
  });
  it('pathAt walks the shortest torus route', () => {
    const q = pathAt([[3, 0], [-3, 0]], 0.5);
    expect(Math.abs(q[0])).toBeCloseTo(Math.PI, 9);
  });
});

describe('C-obstacle grid and components', () => {
  it('a disk that blocks link 1 produces a full-height band that does not disconnect the torus', () => {
    const obs: Obstacle[] = [{ c: [0, 0.7], r: 0.25 }];
    const n = 60;
    const grid = computeCobsGrid(n, 'exact', L, 0.05, obs);
    // column at θ₁ = 90° must be fully blocked (link 1 points straight into the disk)
    const i = angleCell(Math.PI / 2, n);
    for (let j = 0; j < n; j++) expect(grid[j * n + i]).toBe(1);
    const comp = torusComponents(grid, n);
    expect(comp.count).toBe(1);
  });
  it('two disks on opposite sides of the base split C-free into two components', () => {
    const obs: Obstacle[] = [{ c: [0, 0.75], r: 0.3 }, { c: [0, -0.75], r: 0.3 }];
    const n = 80;
    const grid = computeCobsGrid(n, 'exact', L, 0.05, obs);
    const comp = torusComponents(grid, n);
    expect(comp.count).toBe(2);
    expect(componentAt(comp, n, [0, 0.3])).not.toBe(componentAt(comp, n, [Math.PI, 0.3]));
    expect(componentAt(comp, n, [Math.PI / 2, 0])).toBe(-1);
  });
  it('an empty workspace has a single free component', () => {
    const grid = computeCobsGrid(20, 'exact', L, 0.05, []);
    expect(torusComponents(grid, 20).count).toBe(1);
  });
});

describe('path checking', () => {
  it('flags the segment that sweeps link 1 through the disk and passes the one that goes around', () => {
    const obs: Obstacle[] = [{ c: [0, 0.7], r: 0.25 }];
    const bad = checkPath([[0, 0.3], [Math.PI - 0.1, 0.3]], 'exact', L, 0.05, obs);
    expect(bad.badSegments).toBe(1);
    const good = checkPath([[0, 0.3], [-Math.PI + 0.1, 0.3]], 'exact', L, 0.05, obs);
    expect(good.badSegments).toBe(0);
    expect(good.length).toBeCloseTo(Math.PI - 0.1, 9);
  });
});
