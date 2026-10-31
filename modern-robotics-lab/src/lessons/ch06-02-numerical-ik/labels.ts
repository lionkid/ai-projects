/**
 * Greedy label placement for this lesson's planar canvas.
 *
 * While the scene is drawn, labels ({b}, {d}, V_b, θ⁰, iteration numbers …) are queued together with
 * the screen-space obstacles they should not cover: links, frame axes, paths and handles. `flush()`
 * then draws the labels on top of everything, each at the candidate offset around its anchor with
 * the lowest cost (leaving the canvas ≫ covering a link, a handle or an earlier label ≫ covering a
 * light path ≫ deviating from the preferred side). The geometry helpers are pure functions in screen
 * pixels (tested in labels.test.ts); only `LabelLayer` touches the canvas.
 */
import type { Canvas2D, Pt, TextStyle } from '@/ui';

export type P = [number, number];
export interface Rect { x0: number; y0: number; x1: number; y1: number }
export interface SegObstacle { a: P; b: P; w: number }
export interface DiscObstacle { c: P; r: number; w: number }

export interface Scene {
  width: number;
  height: number;
  placed: Rect[];
  segs: SegObstacle[];
  discs: DiscObstacle[];
}

/** Overlap area of two rectangles (0 when disjoint). */
export function rectOverlap(a: Rect, b: Rect): number {
  const w = Math.min(a.x1, b.x1) - Math.max(a.x0, b.x0);
  const h = Math.min(a.y1, b.y1) - Math.max(a.y0, b.y0);
  return w > 0 && h > 0 ? w * h : 0;
}

/** True when the segment ab crosses or lies inside the rectangle (Liang–Barsky clipping). */
export function segHitsRect(a: P, b: P, r: Rect): boolean {
  let t0 = 0;
  let t1 = 1;
  const dx = b[0] - a[0];
  const dy = b[1] - a[1];
  const p = [-dx, dx, -dy, dy];
  const q = [a[0] - r.x0, r.x1 - a[0], a[1] - r.y0, r.y1 - a[1]];
  for (let i = 0; i < 4; i++) {
    if (p[i] === 0) {
      if (q[i] < 0) return false;
      continue;
    }
    const t = q[i] / p[i];
    if (p[i] < 0) {
      if (t > t1) return false;
      if (t > t0) t0 = t;
    } else {
      if (t < t0) return false;
      if (t < t1) t1 = t;
    }
  }
  return true;
}

/** True when a disc (centre c, radius rad) touches the rectangle. */
export function discHitsRect(c: P, rad: number, r: Rect): boolean {
  const x = Math.max(r.x0, Math.min(c[0], r.x1));
  const y = Math.max(r.y0, Math.min(c[1], r.y1));
  return Math.hypot(c[0] - x, c[1] - y) < rad;
}

/** Cost of putting a label box at `r`. */
export function boxCost(r: Rect, s: Scene): number {
  const out = Math.max(0, -r.x0) + Math.max(0, r.x1 - s.width) + Math.max(0, -r.y0) + Math.max(0, r.y1 - s.height);
  let cost = out * 1e4;
  for (const p of s.placed) cost += rectOverlap(r, p) * 60;
  for (const g of s.segs) if (segHitsRect(g.a, g.b, r)) cost += g.w;
  for (const d of s.discs) if (discHitsRect(d.c, d.r, r)) cost += d.w;
  return cost;
}

export interface PlaceInput {
  /** Anchor point (screen px). */
  anchor: P;
  /** Label box size (screen px). */
  w: number;
  h: number;
  /** Preferred direction from the anchor, in screen radians (0 = right, π/2 = down). */
  prefer: number;
  /** Gaps between the anchor and the box to try, nearest first (px). */
  radii: number[];
}

export interface Placement { cx: number; cy: number; rect: Rect; cost: number }

/** Best box position on rings of 16 directions around the anchor (the box never covers the anchor). */
export function placeLabel(inp: PlaceInput, s: Scene): Placement {
  let best: Placement | null = null;
  inp.radii.forEach((R, ri) => {
    for (let k = 0; k < 16; k++) {
      const a = inp.prefer + (k * Math.PI) / 8;
      const cx = inp.anchor[0] + Math.cos(a) * (R + inp.w / 2);
      const cy = inp.anchor[1] + Math.sin(a) * (R + inp.h / 2);
      const rect: Rect = { x0: cx - inp.w / 2, y0: cy - inp.h / 2, x1: cx + inp.w / 2, y1: cy + inp.h / 2 };
      const dev = Math.abs(Math.atan2(Math.sin(a - inp.prefer), Math.cos(a - inp.prefer)));
      const cost = boxCost(rect, s) + 8 * dev + 6 * ri;
      if (!best || cost < best.cost) best = { cx, cy, rect, cost };
    }
  });
  return best!;
}

/** Screen direction (radians, y down) of a world direction (radians, y up). */
export const screenDir = (worldAngle: number): number => -worldAngle;

// ---------------------------------------------------------------------------
// Canvas binding
// ---------------------------------------------------------------------------

export interface LabelOpts extends TextStyle {
  /** Preferred direction in WORLD radians (y up); default: up-right. */
  prefer?: number;
  radii?: number[];
  /** Placement order: lower values choose their spot first (default 5; ties keep queue order). */
  priority?: number;
}

interface Queued { anchor: Pt; text: string; style: TextStyle; font: string; prefer: number; radii: number[]; priority: number }
interface Fixed { anchor: Pt; text: string; style: TextStyle; font: string; stack: number }

/** Collects obstacles and labels for one frame of a Canvas2D; `flush()` draws the labels. */
export class LabelLayer {
  private segs: SegObstacle[] = [];
  private discs: DiscObstacle[] = [];
  private queue: Queued[] = [];
  private fixed: Fixed[] = [];

  constructor(private readonly c: Canvas2D, private readonly font: string) {}

  /** A line the labels should avoid (weight ≈ how bad covering it is). */
  seg(a: Pt, b: Pt, w = 1000): void {
    this.segs.push({ a: this.c.toScreen(a), b: this.c.toScreen(b), w });
  }

  polyline(pts: ArrayLike<Pt>, w = 300): void {
    for (let i = 0; i + 1 < pts.length; i++) this.seg(pts[i], pts[i + 1], w);
  }

  /** A round obstacle with radius in px (joints, handles, arrow heads). */
  disc(p: Pt, rPx: number, w = 1000): void {
    this.discs.push({ c: this.c.toScreen(p), r: rPx, w });
  }

  /** The two arrows (and the origin dot) of a frame drawn with Canvas2D.frame. */
  frame(x: number, y: number, theta: number, size: number, w = 1500): void {
    const o: Pt = [x, y];
    const ex: Pt = [x + size * Math.cos(theta), y + size * Math.sin(theta)];
    const ey: Pt = [x - size * Math.sin(theta), y + size * Math.cos(theta)];
    this.seg(o, ex, w);
    this.seg(o, ey, w);
    this.disc(ex, 6, w);
    this.disc(ey, 6, w);
  }

  /** Queue a label; it is placed next to `anchor` in `flush()`. */
  label(anchor: Pt, text: string, o: LabelOpts = {}): void {
    const { prefer, radii, priority, ...style } = o;
    this.queue.push({
      anchor, text, style, font: o.font ?? this.font,
      prefer: screenDir(prefer ?? Math.PI / 4), radii: radii ?? [14, 24, 36], priority: priority ?? 5,
    });
  }

  /**
   * A label at a fixed screen offset (corner keys); fixed labels are drawn first and block the others.
   * `stack` (−1 up, +1 down) moves it in 18 px steps while it overlaps an earlier fixed label, so two
   * keys in the same row stack up on a narrow (phone) canvas.
   */
  fixedLabel(anchor: Pt, text: string, style: TextStyle = {}, stack = 0): void {
    this.fixed.push({ anchor, text, style, font: style.font ?? this.font, stack });
  }

  private boxOf(anchor: Pt, text: string, font: string, style: TextStyle): Rect {
    const ctx = this.c.ctx;
    ctx.font = font;
    const w = ctx.measureText(text).width + 8;
    const [ax, ay] = this.c.toScreen(anchor);
    const cx = ax + (style.dx ?? 0);
    const cy = ay + (style.dy ?? 0);
    const align = style.align ?? 'center';
    const x0 = align === 'left' ? cx - 4 : align === 'right' ? cx - w + 4 : cx - w / 2;
    return { x0, y0: cy - 8, x1: x0 + w, y1: cy + 8 };
  }

  flush(): void {
    const placed: Rect[] = [];
    for (const f of this.fixed) {
      let dy = f.style.dy ?? 0;
      let box = this.boxOf(f.anchor, f.text, f.font, { ...f.style, dy });
      for (let k = 0; k < 4 && f.stack !== 0 && placed.some((r) => rectOverlap(r, box) > 0); k++) {
        dy += 18 * f.stack;
        box = this.boxOf(f.anchor, f.text, f.font, { ...f.style, dy });
      }
      placed.push(box);
      this.c.text(f.anchor, f.text, { ...f.style, dy, font: f.font });
    }
    const ctx = this.c.ctx;
    // Array.prototype.sort is stable, so equal priorities keep their queue order.
    const order = this.queue.slice().sort((a, b) => a.priority - b.priority);
    for (const q of order) {
      ctx.font = q.font;
      const w = ctx.measureText(q.text).width + 8;
      const a = this.c.toScreen(q.anchor);
      const best = placeLabel(
        { anchor: a, w, h: 16, prefer: q.prefer, radii: q.radii },
        { width: this.c.width, height: this.c.height, placed, segs: this.segs, discs: this.discs },
      );
      placed.push(best.rect);
      this.c.text(q.anchor, q.text, { ...q.style, font: q.font, align: 'center', baseline: 'middle', dx: best.cx - a[0], dy: best.cy - a[1] });
    }
    this.queue = [];
    this.fixed = [];
  }
}
