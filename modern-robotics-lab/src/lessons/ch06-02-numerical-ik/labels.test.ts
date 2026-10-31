import { describe, expect, it } from 'vitest';
import { boxCost, discHitsRect, placeLabel, rectOverlap, segHitsRect, type Rect, type Scene } from './labels';

const R: Rect = { x0: 10, y0: 10, x1: 30, y1: 20 };
const empty = (): Scene => ({ width: 200, height: 100, placed: [], segs: [], discs: [] });

describe('geometry helpers', () => {
  it('rectangle overlap area', () => {
    expect(rectOverlap(R, { x0: 20, y0: 15, x1: 40, y1: 40 })).toBe(50);
    expect(rectOverlap(R, { x0: 30, y0: 10, x1: 40, y1: 20 })).toBe(0);
  });

  it('segment–rectangle intersection covers crossing, inside, and miss cases', () => {
    expect(segHitsRect([0, 15], [40, 15], R)).toBe(true); // crosses
    expect(segHitsRect([12, 12], [14, 14], R)).toBe(true); // fully inside
    expect(segHitsRect([0, 0], [40, 5], R)).toBe(false); // passes above
    expect(segHitsRect([0, 0], [8, 30], R)).toBe(false); // passes left
    expect(segHitsRect([20, 0], [20, 5], R)).toBe(false); // vertical, stops short
    expect(segHitsRect([0, 30], [30, 0], R)).toBe(true); // diagonal through a corner region
  });

  it('disc–rectangle contact', () => {
    expect(discHitsRect([35, 15], 6, R)).toBe(true);
    expect(discHitsRect([40, 15], 6, R)).toBe(false);
    expect(discHitsRect([35, 25], 6, R)).toBe(false); // corner distance √50 > 6
  });

  it('cost grows when leaving the canvas or covering obstacles', () => {
    const s = empty();
    expect(boxCost(R, s)).toBe(0);
    expect(boxCost({ x0: -5, y0: 10, x1: 15, y1: 20 }, s)).toBeGreaterThan(1e4);
    s.segs.push({ a: [0, 15], b: [100, 15], w: 1000 });
    expect(boxCost(R, s)).toBe(1000);
  });
});

describe('placeLabel', () => {
  it('takes the preferred side when it is free', () => {
    const p = placeLabel({ anchor: [100, 50], w: 30, h: 16, prefer: 0, radii: [10] }, empty());
    expect(p.cost).toBe(0);
    expect(p.rect.x0).toBeCloseTo(110, 9); // gap of 10 px to the right of the anchor
    expect(p.cy).toBeCloseTo(50, 9);
  });

  it('moves away from a link lying on the preferred side', () => {
    const s = empty();
    s.segs.push({ a: [100, 50], b: [190, 50], w: 3000 }); // link to the right of the anchor
    const p = placeLabel({ anchor: [100, 50], w: 30, h: 16, prefer: 0, radii: [10, 20] }, s);
    expect(p.cost).toBeLessThan(100);
    expect(segHitsRect([100, 50], [190, 50], p.rect)).toBe(false);
  });

  it('two labels on the same anchor do not overlap', () => {
    const s = empty();
    const a = placeLabel({ anchor: [100, 50], w: 30, h: 16, prefer: 0, radii: [10, 20] }, s);
    s.placed.push(a.rect);
    const b = placeLabel({ anchor: [100, 50], w: 30, h: 16, prefer: 0, radii: [10, 20] }, s);
    expect(rectOverlap(a.rect, b.rect)).toBe(0);
  });

  it('stays inside the canvas near an edge', () => {
    const p = placeLabel({ anchor: [195, 50], w: 30, h: 16, prefer: 0, radii: [10] }, empty());
    expect(p.rect.x1).toBeLessThanOrEqual(200);
    expect(p.rect.x0).toBeGreaterThanOrEqual(0);
  });
});
