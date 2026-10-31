import { describe, expect, it } from 'vitest';
import { planarFK } from '@/mr';
import { type Box, panZoomToFit, sameBox, scaleAbout, zoomToFit } from './viewFit';

const W: Box = { xmin: -1.75, xmax: 2.55, ymin: -0.78, ymax: 1.75 };
const M: [number, number, number, number] = [0.12, 0.12, 0.15, 0.15];
const L = [1, 0.8, 0.6];
const DEG = Math.PI / 180;

describe('zoomToFit / scaleAbout', () => {
  it('keeps the tuned window (k = 1) when everything already fits', () => {
    const fk = planarFK(L, [15 * DEG, 55 * DEG, 90 * DEG]);
    expect(zoomToFit(W, [0, 0], fk.joints, M)).toBe(1);
  });

  it('zooms out just enough that a downward arm lands on the safe-area edge', () => {
    const fk = planarFK(L, [-90 * DEG, 55 * DEG, 90 * DEG]);
    const k = zoomToFit(W, [0, 0], fk.joints, M);
    expect(k).toBeGreaterThan(1.5);
    const w = scaleAbout(W, [0, 0], k);
    // every joint is inside the scaled safe area, and the lowest one touches its bottom edge
    const minY = Math.min(...fk.joints.map((p) => p[1]));
    for (const p of fk.joints) {
      expect(p[0]).toBeGreaterThanOrEqual(w.xmin + k * M[0] - 1e-12);
      expect(p[0]).toBeLessThanOrEqual(w.xmax - k * M[1] + 1e-12);
      expect(p[1]).toBeGreaterThanOrEqual(w.ymin + k * M[2] - 1e-12);
      expect(p[1]).toBeLessThanOrEqual(w.ymax - k * M[3] + 1e-12);
    }
    expect(minY).toBeCloseTo(w.ymin + k * M[2], 12);
  });

  it('keeps the reference point fixed on screen and the aspect ratio unchanged', () => {
    const ref: [number, number] = [1.2, 0.12];
    const w = scaleAbout(W, ref, 2.5);
    const fx = (ref[0] - W.xmin) / (W.xmax - W.xmin);
    const fy = (ref[1] - W.ymin) / (W.ymax - W.ymin);
    expect((ref[0] - w.xmin) / (w.xmax - w.xmin)).toBeCloseTo(fx, 12);
    expect((ref[1] - w.ymin) / (w.ymax - w.ymin)).toBeCloseTo(fy, 12);
    expect((w.xmax - w.xmin) / (w.ymax - w.ymin)).toBeCloseTo((W.xmax - W.xmin) / (W.ymax - W.ymin), 12);
  });

  it('ignores non-finite points and caps the zoom', () => {
    expect(zoomToFit(W, [0, 0], [[NaN, 3], [Infinity, 0]], M)).toBe(1);
    expect(zoomToFit(W, [0, 0], [[0, -100]], M, 4)).toBe(4);
  });
});

describe('panZoomToFit', () => {
  const inside = (w: Box, k: number, p: ArrayLike<number>) =>
    p[0] >= w.xmin + k * M[0] - 1e-12 && p[0] <= w.xmax - k * M[1] + 1e-12 && p[1] >= w.ymin + k * M[2] - 1e-12 && p[1] <= w.ymax - k * M[3] + 1e-12;

  it('returns the tuned window unchanged when the arm already fits', () => {
    const fk = planarFK(L, [15 * DEG, 55 * DEG, 90 * DEG]);
    const w = panZoomToFit(W, [0, 0], fk.joints, M);
    expect(sameBox(w, W, 1e-12)).toBe(true);
  });

  it('only pans (arm keeps its size) for a downward arm that is smaller than the window', () => {
    const fk = planarFK(L, [-90 * DEG, 55 * DEG, 90 * DEG]);
    const w = panZoomToFit(W, [0, 0], fk.joints, M);
    expect(w.xmax - w.xmin).toBeCloseTo(W.xmax - W.xmin, 12);
    expect(w.ymax - w.ymin).toBeCloseTo(W.ymax - W.ymin, 12);
    expect(w.ymin).toBeLessThan(W.ymin);
    for (const p of fk.joints) expect(inside(w, 1, p)).toBe(true);
  });

  it('zooms out (keeping the aspect) when the content is taller than the window', () => {
    const pts = [[0, 0.5], [0, -3]];
    const w = panZoomToFit(W, [0, 0], pts, M);
    const k = (w.xmax - w.xmin) / (W.xmax - W.xmin);
    expect(k).toBeGreaterThan(1);
    expect((w.ymax - w.ymin) / (W.ymax - W.ymin)).toBeCloseTo(k, 12);
    for (const p of pts) expect(inside(w, k, p)).toBe(true);
  });
});
