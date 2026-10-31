/**
 * Keep-in-view framing for the planar canvas.
 *
 * The 2D window is tuned for the preset pose, but the learner can drag θ₁ anywhere (or press play),
 * which used to push most of the arm off the canvas. Two strategies, both of which leave the tuned
 * window untouched while everything already fits:
 *  - `panZoomToFit` (normal view): shift the window as little as possible, and zoom out only when
 *    the content is larger than the window, so the arm keeps its size;
 *  - `zoomToFit` + `scaleAbout` (camera riding on {b}): zoom out about a fixed screen point only,
 *    so {b} never moves on screen.
 */

export interface Box { xmin: number; xmax: number; ymin: number; ymax: number }

/** Margins of the safe area inside a window (world units at zoom 1): [left, right, bottom, top]. */
export type Margins = [number, number, number, number];

/**
 * Smallest zoom factor k ≥ 1 such that every point lies inside the safe area of `world` scaled by
 * k about `ref` (k is capped at `maxK`). `ref` must lie inside the safe area; non-finite points are
 * ignored.
 */
export function zoomToFit(world: Box, ref: readonly [number, number], pts: readonly ArrayLike<number>[], margin: Margins = [0, 0, 0, 0], maxK = 4): number {
  const left = ref[0] - (world.xmin + margin[0]);
  const right = world.xmax - margin[1] - ref[0];
  const down = ref[1] - (world.ymin + margin[2]);
  const up = world.ymax - margin[3] - ref[1];
  let k = 1;
  for (const p of pts) {
    const dx = p[0] - ref[0];
    const dy = p[1] - ref[1];
    if (!Number.isFinite(dx) || !Number.isFinite(dy)) continue;
    if (dx > 0 && right > 0) k = Math.max(k, dx / right);
    else if (dx < 0 && left > 0) k = Math.max(k, -dx / left);
    if (dy > 0 && up > 0) k = Math.max(k, dy / up);
    else if (dy < 0 && down > 0) k = Math.max(k, -dy / down);
  }
  return Math.min(k, maxK);
}

/** `world` scaled by k about `ref` (ref keeps its place on screen). */
export function scaleAbout(world: Box, ref: readonly [number, number], k: number): Box {
  return {
    xmin: ref[0] + k * (world.xmin - ref[0]),
    xmax: ref[0] + k * (world.xmax - ref[0]),
    ymin: ref[1] + k * (world.ymin - ref[1]),
    ymax: ref[1] + k * (world.ymax - ref[1]),
  };
}

/**
 * Window with the aspect ratio of `world` whose safe area contains every point: zoom k ≥ 1 (about
 * `ref`) just large enough for the bounding box of the points to fit, then the smallest shift that
 * brings them inside. Returns `world` itself (k = 1, no shift) when everything already fits.
 */
export function panZoomToFit(world: Box, ref: readonly [number, number], pts: readonly ArrayLike<number>[], margin: Margins = [0, 0, 0, 0], maxK = 4): Box {
  let x0 = Infinity;
  let x1 = -Infinity;
  let y0 = Infinity;
  let y1 = -Infinity;
  for (const p of pts) {
    if (!Number.isFinite(p[0]) || !Number.isFinite(p[1])) continue;
    x0 = Math.min(x0, p[0]); x1 = Math.max(x1, p[0]);
    y0 = Math.min(y0, p[1]); y1 = Math.max(y1, p[1]);
  }
  if (!(x0 <= x1 && y0 <= y1)) return { ...world };
  const safeW = world.xmax - world.xmin - margin[0] - margin[1];
  const safeH = world.ymax - world.ymin - margin[2] - margin[3];
  const k = Math.min(maxK, Math.max(1, (x1 - x0) / safeW, (y1 - y0) / safeH));
  const w = scaleAbout(world, ref, k);
  const sx0 = w.xmin + k * margin[0];
  const sx1 = w.xmax - k * margin[1];
  const sy0 = w.ymin + k * margin[2];
  const sy1 = w.ymax - k * margin[3];
  const dx = x0 < sx0 ? x0 - sx0 : x1 > sx1 ? x1 - sx1 : 0;
  const dy = y0 < sy0 ? y0 - sy0 : y1 > sy1 ? y1 - sy1 : 0;
  return { xmin: w.xmin + dx, xmax: w.xmax + dx, ymin: w.ymin + dy, ymax: w.ymax + dy };
}

/** Whether two windows agree to within `tol` (avoids redundant re-layouts). */
export function sameBox(a: Box, b: Box, tol = 1e-6): boolean {
  return Math.abs(a.xmin - b.xmin) < tol && Math.abs(a.xmax - b.xmax) < tol && Math.abs(a.ymin - b.ymin) < tol && Math.abs(a.ymax - b.ymax) < tol;
}
