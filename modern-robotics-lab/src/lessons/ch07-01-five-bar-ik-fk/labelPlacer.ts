/**
 * Greedy placement of canvas labels in a crowded view (the 3xRPR step of ch07-01): every label gets a
 * list of candidate anchors and is drawn at the first one whose pill neither leaves the canvas nor covers
 * an earlier label or a blocked spot (drag handles, joints). Works in screen pixels.
 * Only types come from @/ui, so the pure helpers can be unit-tested without a DOM.
 */
import type { Canvas2D, TextStyle } from '@/ui';

type Pt = [number, number];

export interface Rect { x: number; y: number; w: number; h: number }

/** Do two rectangles overlap (with an extra gap in px)? */
export function rectsOverlap(a: Rect, b: Rect, gap = 2): boolean {
  return a.x < b.x + b.w + gap && b.x < a.x + a.w + gap && a.y < b.y + b.h + gap && b.y < a.y + a.h + gap;
}

/** Overlap area of two rectangles (px²). */
export function overlapArea(a: Rect, b: Rect): number {
  const w = Math.min(a.x + a.w, b.x + b.w) - Math.max(a.x, b.x);
  const h = Math.min(a.y + a.h, b.y + b.h) - Math.max(a.y, b.y);
  return w > 0 && h > 0 ? w * h : 0;
}

/**
 * Index of the first candidate rectangle that fits inside `bounds` and overlaps none of `taken`;
 * otherwise the fitting candidate that covers the least area of `taken` (earliest on ties); otherwise 0.
 */
export function pickFree(cands: Rect[], taken: Rect[], bounds: Rect): number {
  const fits = (r: Rect) => r.x >= bounds.x && r.y >= bounds.y && r.x + r.w <= bounds.x + bounds.w && r.y + r.h <= bounds.y + bounds.h;
  const free = cands.findIndex((r) => fits(r) && !taken.some((t) => rectsOverlap(t, r)));
  if (free >= 0) return free;
  let best = -1;
  let bestArea = Infinity;
  cands.forEach((r, i) => {
    if (!fits(r)) return;
    const area = taken.reduce((s, t) => s + overlapArea(t, r), 0);
    if (area < bestArea) { best = i; bestArea = area; }
  });
  return best >= 0 ? best : 0;
}

export class LabelPlacer {
  private taken: Rect[] = [];

  /** `top`: labels stay below this screen y (px), e.g. under the canvas overlay line. */
  constructor(private cv: Canvas2D, private top = 0) {}

  /** Keep labels off a disc of radius r px around a world point. */
  block(p: Pt, r: number): void {
    const [x, y] = this.cv.toScreen(p);
    this.taken.push({ x: x - r, y: y - r, w: 2 * r, h: 2 * r });
  }

  /** Draw `str` (centred, with a background pill) at the first free candidate anchor (world points). */
  place(cands: Pt[], str: string, style: TextStyle): void {
    const c = this.cv;
    c.ctx.font = style.font ?? '12px sans-serif';
    const w = c.ctx.measureText(str).width + 8;
    const h = 16;
    const rects = cands.map((p) => {
      const [sx, sy] = c.toScreen(p);
      return { x: sx - w / 2, y: sy - h / 2, w, h };
    });
    const bounds = { x: 2, y: this.top, w: c.width - 4, h: c.height - this.top - 2 };
    const r = { ...rects[pickFree(rects, this.taken, bounds)] };
    // clamp into the canvas (only matters when no candidate fits)
    r.x = Math.min(bounds.x + bounds.w - r.w, Math.max(bounds.x, r.x));
    r.y = Math.min(bounds.y + bounds.h - r.h, Math.max(bounds.y, r.y));
    this.taken.push(r);
    c.text(c.toWorld(r.x + r.w / 2, r.y + r.h / 2), str, { ...style, align: 'center', dx: 0, dy: 0 });
  }
}
