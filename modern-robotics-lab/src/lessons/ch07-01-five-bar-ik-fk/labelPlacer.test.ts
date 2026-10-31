import { describe, expect, it } from 'vitest';
import { pickFree, rectsOverlap, type Rect } from './labelPlacer';

const r = (x: number, y: number, w = 20, h = 16): Rect => ({ x, y, w, h });
const bounds = r(0, 30, 400, 300);

describe('label placement helpers', () => {
  it('overlap test respects the gap', () => {
    expect(rectsOverlap(r(0, 0), r(10, 5))).toBe(true);
    expect(rectsOverlap(r(0, 0), r(30, 0))).toBe(false);
    expect(rectsOverlap(r(0, 0), r(21, 0), 2)).toBe(true); // 1 px apart, closer than the gap
    expect(rectsOverlap(r(0, 0), r(21, 0), 0)).toBe(false);
  });

  it('takes the first candidate that is inside and free', () => {
    const taken = [r(100, 100)];
    expect(pickFree([r(105, 102), r(50, 50), r(200, 200)], taken, bounds)).toBe(1);
    expect(pickFree([r(50, 50), r(105, 102)], taken, bounds)).toBe(0);
  });

  it('skips candidates above the top margin or outside the canvas', () => {
    expect(pickFree([r(50, 10), r(395, 100), r(60, 60)], [], bounds)).toBe(2);
  });

  it('falls back to the inside candidate with the least overlap, else to 0', () => {
    const all = [r(0, 30, 400, 300)];
    expect(pickFree([r(50, 10), r(60, 60), r(70, 70)], all, bounds)).toBe(1); // equal overlap: earliest
    const blobs = [r(100, 100, 30, 30), r(200, 100, 30, 30)];
    // the first candidate covers a blob fully, the second one only touches the other blob's corner
    expect(pickFree([r(105, 107), r(220, 120), r(500, 60)], blobs, bounds)).toBe(1);
    expect(pickFree([r(-50, 10), r(500, 60)], all, bounds)).toBe(0);
  });
});
