import { describe, expect, it } from 'vitest';
import { planarFK } from '@/mr';
import {
  PATH_PRESETS, PI, TWO_PI, densify, glueStages, ikNearest, implicitCoords, morphNormal, morphPoint, otherElbow,
  pathLength, pushTrail, samplePath, torusDist, trailPieces, unwrapSeq, wrapAngle, wrapPolylines, type Th, type Trail,
} from './torus';

const R = 1;
const r = 0.42;
const close = (a: number[], b: number[], tol = 1e-9) => a.forEach((x, i) => expect(Math.abs(x - b[i])).toBeLessThan(tol));

describe('square → torus morph', () => {
  it('a = b = 1 is the standard torus parametrisation', () => {
    for (const [t1, t2] of [[0, 0], [1, 2], [-2.5, 0.7], [PI, -PI], [3, -3]]) {
      const p = morphPoint(t1, t2, 1, 1, R, r);
      close(p, [(R + r * Math.cos(t2)) * Math.cos(t1), (R + r * Math.cos(t2)) * Math.sin(t1), r * Math.sin(t2)]);
    }
  });

  it('a = b = 0 is the flat sheet x = R + r, y = R θ1, z = r θ2', () => {
    close(morphPoint(1.3, -2, 0, 0, R, r), [R + r, 1.3 * R, -2 * r]);
  });

  it('is continuous as the bending parameters leave 0', () => {
    close(morphPoint(2, 2.5, 1e-7, 1e-7, R, r), morphPoint(2, 2.5, 0, 0, R, r), 1e-5);
    close(morphPoint(2, 2.5, 1e-7, 0.5, R, r), morphPoint(2, 2.5, 0, 0.5, R, r), 1e-5);
  });

  it('opposite edges meet only after gluing', () => {
    const e = 1e-7;
    // glued: θ1 = ±π and θ2 = ±π become the same points
    close(morphPoint(PI - e, 0.4, 1, 1, R, r), morphPoint(-PI + e, 0.4, 1, 1, R, r), 1e-5);
    close(morphPoint(0.4, PI - e, 1, 1, R, r), morphPoint(0.4, -PI + e, 1, 1, R, r), 1e-5);
    // only the tube is closed (b = 0): θ2 seam glued, θ1 seam still 2πR apart
    close(morphPoint(0.4, PI - e, 1, 0, R, r), morphPoint(0.4, -PI + e, 1, 0, R, r), 1e-5);
    const p = morphPoint(PI, 0.4, 1, 0, R, r);
    const q = morphPoint(-PI, 0.4, 1, 0, R, r);
    expect(Math.hypot(p[0] - q[0], p[1] - q[1], p[2] - q[2])).toBeCloseTo(TWO_PI * R, 9);
  });

  it('normals are the outward torus normals when fully glued', () => {
    for (const [t1, t2] of [[0, 0], [1, 2], [-2.5, 0.7]]) {
      const n = morphNormal(t1, t2, 1, 1, R, r);
      close(n, [Math.cos(t2) * Math.cos(t1), Math.cos(t2) * Math.sin(t1), Math.sin(t2)], 1e-6);
    }
    close(morphNormal(0.3, 0.2, 0, 0, R, r), [1, 0, 0], 1e-9);
  });

  it('glue progress runs the tube stage before the ring stage', () => {
    expect(glueStages(0)).toEqual({ a: 0, b: 0 });
    expect(glueStages(1)).toEqual({ a: 1, b: 0 });
    expect(glueStages(2)).toEqual({ a: 1, b: 1 });
  });
});

describe('wrap-aware paths', () => {
  it('keeps ±π for display and wraps the rest', () => {
    expect(wrapAngle(PI)).toBe(PI);
    expect(wrapAngle(-PI)).toBe(-PI);
    expect(wrapAngle(PI + 0.1)).toBeCloseTo(-PI + 0.1, 12);
    expect(torusDist([3.1, 0], [-3.1, 0])).toBeCloseTo(TWO_PI - 6.2, 12);
  });

  it('a straight path across θ1 = π is cut into two pieces that end on the edges', () => {
    const path = densify(PATH_PRESETS[0].pts, 0.05);
    const pieces = wrapPolylines(path);
    expect(pieces.length).toBe(2);
    expect(pieces[0][pieces[0].length - 1][0]).toBeCloseTo(PI, 9);
    expect(pieces[1][0][0]).toBeCloseTo(-PI, 9);
    // both sides of the cut are the same configuration
    expect(pieces[0][pieces[0].length - 1][1]).toBeCloseTo(pieces[1][0][1], 9);
    // cutting does not change the length of the drawn path
    const total = pieces.reduce((s, p) => s + pathLength(p), 0);
    expect(total).toBeCloseTo(pathLength(path), 9);
  });

  it('a diagonal loop crosses both seams', () => {
    const pieces = wrapPolylines(densify(PATH_PRESETS[3].pts, 0.05));
    expect(pieces.length).toBe(3);
    for (const piece of pieces) for (const p of piece) {
      expect(Math.abs(p[0])).toBeLessThanOrEqual(PI + 1e-9);
      expect(Math.abs(p[1])).toBeLessThanOrEqual(PI + 1e-9);
    }
  });

  it('unwrapping recovers a continuous sequence from wrapped samples', () => {
    const truth: Th[] = Array.from({ length: 50 }, (_, i) => [2.5 + 0.1 * i, -3 - 0.08 * i]);
    const wrapped = truth.map((p) => [wrapAngle(p[0]), wrapAngle(p[1])] as Th);
    const back = unwrapSeq(wrapped);
    back.forEach((p, i) => close(p, truth[i], 1e-9));
  });

  it('a trail breaks at jumps instead of drawing a motion that never happened', () => {
    const trail: Trail = [];
    const o = { minStep: 0.025, max: 50 };
    for (let k = 0; k < 10; k++) pushTrail(trail, [0.1 * k, 0.5], o);
    expect(pushTrail(trail, [0.9 + 0.01, 0.5], o)).toBe(false);       // closer than minStep: skipped
    pushTrail(trail, [-2.5, -1], { ...o, jump: true });               // explicit jump (e.g. preset loaded)
    pushTrail(trail, [-2.45, -1], o);
    pushTrail(trail, [2.0, 2.0], o);                                  // a 4-rad leap counts as a jump too
    pushTrail(trail, [2.05, 2.0], o);
    const pieces = trailPieces(trail);
    expect(pieces.map((p) => p.length)).toEqual([10, 2, 2]);
    // crossing the ±π seam is a small move on the torus, not a jump
    const seam: Trail = [];
    pushTrail(seam, [PI - 0.05, 0], o);
    pushTrail(seam, [-PI + 0.05, 0], o);
    expect(trailPieces(seam)).toHaveLength(1);
    // the cap drops the oldest samples and never leaves a leading break
    const capped: Trail = [];
    pushTrail(capped, [0, 0], { minStep: 0.01, max: 3 });
    pushTrail(capped, [2, 2], { minStep: 0.01, max: 3, jump: true });
    pushTrail(capped, [2.1, 2], { minStep: 0.01, max: 3 });
    expect(capped).toEqual([[2, 2], [2.1, 2]]);
  });

  it('densify bounds the step and samplePath hits both ends', () => {
    const d = densify([[0, 0], [1, 0], [1, 2]], 0.1);
    for (let i = 1; i < d.length; i++) expect(Math.hypot(d[i][0] - d[i - 1][0], d[i][1] - d[i - 1][1])).toBeLessThanOrEqual(0.1 + 1e-12);
    close(samplePath(d, 0), [0, 0]);
    close(samplePath(d, 1), [1, 2]);
    close(samplePath(d, 1 / 3), [1, 0], 1e-9);
  });
});

describe('planar 2R helpers', () => {
  const tip = (L1: number, L2: number, t: Th) => planarFK([L1, L2], [t[0], t[1]]).tip;

  it('the mirror elbow reaches the same tip with −θ2 and is an involution', () => {
    for (const [L1, L2] of [[1, 1], [1, 0.5], [0.6, 1.2]]) {
      for (const t of [[0.6, 1.2], [-2, -0.4], [3, 2.9]] as Th[]) {
        const o = otherElbow(L1, L2, t);
        close(tip(L1, L2, o.th), tip(L1, L2, t), 1e-9);
        expect(o.th[1]).toBeCloseTo(-t[1], 12);
        expect(torusDist(otherElbow(L1, L2, o.th).th, t)).toBeLessThan(1e-9);
      }
    }
  });

  it('flags the boundary and the base singularity', () => {
    expect(otherElbow(1, 0.5, [0.3, 0]).coincident).toBe(true);
    const s = otherElbow(1, 1, [0.3, PI]);
    expect(s.atOrigin).toBe(true);
    expect(s.th[0]).toBeCloseTo(0.3, 12);
  });

  it('ikNearest keeps the current elbow branch and projects unreachable targets', () => {
    const cur: Th = [0.6, 1.2];
    const p = tip(1, 1, cur);
    const res = ikNearest(1, 1, [p[0] + 0.01, p[1]], cur);
    expect(res.clamped).toBe(false);
    expect(res.th[1]).toBeGreaterThan(0);
    expect(torusDist(res.th, cur)).toBeLessThan(0.05);
    const mirror = otherElbow(1, 1, cur).th;
    const res2 = ikNearest(1, 1, [p[0] + 0.01, p[1]], mirror);
    expect(res2.th[1]).toBeLessThan(0);
    const far = ikNearest(1, 0.5, [5, 0], cur);
    expect(far.clamped).toBe(true);
    close(tip(1, 0.5, far.th), [1.5, 0], 1e-6);
    const inner = ikNearest(1, 0.5, [0.1, 0], cur);
    expect(inner.clamped).toBe(true);
    close(tip(1, 0.5, inner.th), [0.5, 0], 1e-6);
    const sing = ikNearest(1, 1, [0, 0], cur);
    expect(sing.singular).toBe(true);
    expect(sing.th[0]).toBe(cur[0]);
  });

  it('implicit coordinates satisfy their constraints', () => {
    const c = implicitCoords([2.2, -0.7]);
    expect(Math.abs(c.g1)).toBeLessThan(1e-15);
    expect(Math.abs(c.g2)).toBeLessThan(1e-15);
    expect(c.c1).toBeCloseTo(Math.cos(2.2), 15);
  });
});
