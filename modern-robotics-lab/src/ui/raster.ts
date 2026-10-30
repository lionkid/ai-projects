/** Offscreen rasterisation of a 2D scalar field / predicate into a cached image drawn in world coordinates. */
import type { Canvas2D, World } from './canvas2d';

export type RGBA = [number, number, number, number];
export type Colormap = (v: number) => RGBA;

/** v ∈ [0,1] → perceptual-ish blue→teal→yellow ramp (viridis-like). */
export const viridisLike: Colormap = (v) => {
  const t = Math.max(0, Math.min(1, v));
  const stops: [number, RGBA][] = [[0, [68, 1, 84, 255]], [0.25, [59, 82, 139, 255]], [0.5, [33, 145, 140, 255]], [0.75, [94, 201, 98, 255]], [1, [253, 231, 37, 255]]];
  for (let i = 1; i < stops.length; i++) {
    if (t <= stops[i][0]) {
      const [t0, c0] = stops[i - 1], [t1, c1] = stops[i];
      const u = (t - t0) / (t1 - t0);
      return [c0[0] + (c1[0] - c0[0]) * u, c0[1] + (c1[1] - c0[1]) * u, c0[2] + (c1[2] - c0[2]) * u, 255];
    }
  }
  return stops[stops.length - 1][1];
};

/** v ∈ [-1,1] → blue (negative) / white / red (positive). */
export const diverging: Colormap = (v) => {
  const t = Math.max(-1, Math.min(1, v));
  if (t < 0) { const u = -t; return [255 - u * 200, 255 - u * 160, 255, 255]; }
  return [255, 255 - t * 170, 255 - t * 210, 255];
};

export const grayscale: Colormap = (v) => { const g = 255 * (1 - Math.max(0, Math.min(1, v))); return [g, g, g, 255]; };

export class RasterLayer {
  readonly canvas: HTMLCanvasElement;
  private ctx: CanvasRenderingContext2D;
  private img: ImageData;
  constructor(public world: World, public readonly nx: number, public readonly ny: number) {
    this.canvas = document.createElement('canvas');
    this.canvas.width = nx; this.canvas.height = ny;
    this.ctx = this.canvas.getContext('2d')!;
    this.img = this.ctx.createImageData(nx, ny);
  }
  /** World coordinates of pixel centre (i, j); row 0 is the top (ymax). */
  cell(i: number, j: number): [number, number] {
    const { xmin, xmax, ymin, ymax } = this.world;
    return [xmin + ((i + 0.5) / this.nx) * (xmax - xmin), ymax - ((j + 0.5) / this.ny) * (ymax - ymin)];
  }
  /**
   * Fill from a predicate (true → `color`, false → transparent) or a scalar field through a colormap.
   * For scalar fields pass `range` to normalise values into [0,1] (or [-1,1] for diverging maps).
   */
  paint(f: (x: number, y: number) => number | boolean, opts: { color?: RGBA; colormap?: Colormap; range?: [number, number]; alpha?: number } = {}): this {
    const d = this.img.data;
    const color = opts.color ?? [220, 38, 38, 110];
    const cmap = opts.colormap ?? viridisLike;
    const [lo, hi] = opts.range ?? [0, 1];
    const alpha = opts.alpha ?? 1;
    for (let j = 0; j < this.ny; j++) {
      for (let i = 0; i < this.nx; i++) {
        const [x, y] = this.cell(i, j);
        const v = f(x, y);
        const k = 4 * (j * this.nx + i);
        let c: RGBA;
        if (typeof v === 'boolean') c = v ? color : [0, 0, 0, 0];
        else if (!Number.isFinite(v)) c = [0, 0, 0, 0];
        else c = cmap((v - lo) / (hi - lo || 1));
        d[k] = c[0]; d[k + 1] = c[1]; d[k + 2] = c[2]; d[k + 3] = c[3] * alpha;
      }
    }
    this.ctx.putImageData(this.img, 0, 0);
    return this;
  }
  /** Draw onto a Canvas2D covering `world` (call inside the draw callback). */
  draw(c: Canvas2D, alpha = 1, smooth = false): void { c.image(this.canvas, this.world, { alpha, smooth }); }
}
