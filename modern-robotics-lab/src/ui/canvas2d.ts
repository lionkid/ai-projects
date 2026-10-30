/** World-coordinate 2D canvas (y up, aspect preserved) with drawing primitives and draggable handles. */
import { el, cssVar, rafThrottle } from './dom';

export type Pt = [number, number];
export interface World { xmin: number; xmax: number; ymin: number; ymax: number }
export interface Style {
  color?: string;
  /** Line width in CSS px. */
  width?: number;
  dash?: number[];
  fill?: string;
  alpha?: number;
  cap?: CanvasLineCap;
}
export interface TextStyle {
  color?: string;
  font?: string;
  align?: CanvasTextAlign;
  baseline?: CanvasTextBaseline;
  /** Screen-space offset in px. */
  dx?: number;
  dy?: number;
  /** Draw a background pill behind the text. */
  bg?: string;
}
export interface Handle {
  get(): Pt;
  set(p: Pt): void;
  /** Radius in px (default 8). */
  radius?: number;
  color?: string;
  label?: string;
  enabled?: boolean;
  onStart?(): void;
  onEnd?(): void;
}
export interface Canvas2DOptions {
  /** Height in CSS px; omit to use `aspect`. */
  height?: number;
  /** width / height when height is not fixed. Default 4/3. */
  aspect?: number;
  world: World;
  /** Fractional margin inside the canvas. Default 0.04. */
  margin?: number;
  /** Show the mono-font overlay element at the top-left. */
  overlay?: boolean;
}

export type PointerHandler = (p: Pt, ev: PointerEvent) => void;

export class Canvas2D {
  readonly el: HTMLDivElement;
  readonly canvas: HTMLCanvasElement;
  readonly ctx: CanvasRenderingContext2D;
  readonly overlay: HTMLDivElement;
  world: World;
  /** Render callback: set this and call requestRender(). */
  draw: ((c: Canvas2D) => void) | null = null;
  readonly handles: Handle[] = [];
  readonly requestRender: () => void;
  private ro: ResizeObserver;
  private opts: Canvas2DOptions;
  private w = 0;
  private h = 0;
  private sc = 1;
  private ox = 0;
  private oy = 0;
  private active: Handle | null = null;
  private hover: Handle | null = null;
  private pointerHandlers: { down?: PointerHandler; move?: PointerHandler; up?: PointerHandler }[] = [];
  private onTheme = () => this.requestRender();

  constructor(parent: HTMLElement, opts: Canvas2DOptions) {
    this.opts = opts;
    this.world = { ...opts.world };
    this.canvas = el('canvas');
    this.overlay = el('div', { class: 'mr-overlay' });
    this.el = el('div', { class: 'mr-canvas-wrap' }, this.canvas, opts.overlay ? this.overlay : null);
    parent.appendChild(this.el);
    this.ctx = this.canvas.getContext('2d')!;
    this.requestRender = rafThrottle(() => this.render());
    this.ro = new ResizeObserver(() => this.resize());
    this.ro.observe(this.el);
    window.addEventListener('mr-theme', this.onTheme);
    this.bindPointer();
    this.resize();
  }

  // ---- geometry -------------------------------------------------------
  private resize(): void {
    const W = this.el.clientWidth;
    if (W <= 0) return;
    const H = this.opts.height ?? Math.round(W / (this.opts.aspect ?? 4 / 3));
    this.el.style.height = `${H}px`;
    const dpr = Math.min(2, window.devicePixelRatio || 1);
    this.canvas.width = Math.round(W * dpr);
    this.canvas.height = Math.round(H * dpr);
    this.w = W; this.h = H;
    this.updateTransform();
    this.requestRender();
  }

  private updateTransform(): void {
    const { xmin, xmax, ymin, ymax } = this.world;
    const m = this.opts.margin ?? 0.04;
    const sx = (this.w * (1 - 2 * m)) / (xmax - xmin);
    const sy = (this.h * (1 - 2 * m)) / (ymax - ymin);
    this.sc = Math.min(sx, sy);
    this.ox = this.w / 2 - ((xmin + xmax) / 2) * this.sc;
    this.oy = this.h / 2 + ((ymin + ymax) / 2) * this.sc;
  }

  setWorld(w: World): void { this.world = { ...w }; this.updateTransform(); this.requestRender(); }

  /** Pixels per world unit. */
  get scale(): number { return this.sc; }
  get width(): number { return this.w; }
  get height(): number { return this.h; }

  toScreen(p: Pt): Pt { return [this.ox + p[0] * this.sc, this.oy - p[1] * this.sc]; }
  toWorld(sx: number, sy: number): Pt { return [(sx - this.ox) / this.sc, (this.oy - sy) / this.sc]; }
  /** Visible world rectangle (may be larger than `world` due to aspect fitting). */
  visible(): World {
    const [xmin, ymax] = this.toWorld(0, 0);
    const [xmax, ymin] = this.toWorld(this.w, this.h);
    return { xmin, xmax, ymin, ymax };
  }

  // ---- render ---------------------------------------------------------
  private render(): void {
    if (this.w <= 0) return;
    const dpr = Math.min(2, window.devicePixelRatio || 1);
    this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    this.ctx.clearRect(0, 0, this.w, this.h);
    this.ctx.fillStyle = cssVar('--sim-bg', '#fff');
    this.ctx.fillRect(0, 0, this.w, this.h);
    this.draw?.(this);
    this.drawHandles();
  }

  clear(): void {
    this.ctx.fillStyle = cssVar('--sim-bg', '#fff');
    this.ctx.fillRect(0, 0, this.w, this.h);
  }

  private applyStyle(s?: Style): void {
    const c = this.ctx;
    c.strokeStyle = s?.color ?? cssVar('--fg', '#222');
    c.fillStyle = s?.fill ?? s?.color ?? cssVar('--fg', '#222');
    c.lineWidth = s?.width ?? 1.5;
    c.setLineDash(s?.dash ?? []);
    c.globalAlpha = s?.alpha ?? 1;
    c.lineCap = s?.cap ?? 'round';
    c.lineJoin = 'round';
  }
  private resetStyle(): void { this.ctx.globalAlpha = 1; this.ctx.setLineDash([]); }

  /** Light grid with the given world spacing; the axes through the origin are stronger. */
  grid(step = 1, style?: Style): void {
    const v = this.visible();
    const c = this.ctx;
    this.applyStyle({ color: cssVar('--grid', '#e5e5e5'), width: 1, ...style });
    c.beginPath();
    for (let x = Math.ceil(v.xmin / step) * step; x <= v.xmax; x += step) {
      const [sx] = this.toScreen([x, 0]);
      c.moveTo(Math.round(sx) + 0.5, 0); c.lineTo(Math.round(sx) + 0.5, this.h);
    }
    for (let y = Math.ceil(v.ymin / step) * step; y <= v.ymax; y += step) {
      const [, sy] = this.toScreen([0, y]);
      c.moveTo(0, Math.round(sy) + 0.5); c.lineTo(this.w, Math.round(sy) + 0.5);
    }
    c.stroke();
    this.applyStyle({ color: cssVar('--grid-strong', '#ccc'), width: 1 });
    const [ox, oy] = this.toScreen([0, 0]);
    c.beginPath();
    c.moveTo(Math.round(ox) + 0.5, 0); c.lineTo(Math.round(ox) + 0.5, this.h);
    c.moveTo(0, Math.round(oy) + 0.5); c.lineTo(this.w, Math.round(oy) + 0.5);
    c.stroke();
    this.resetStyle();
  }

  /** Axis labels at the right/top edges (e.g. 'x', 'y'). */
  axisLabels(xl = 'x', yl = 'y'): void {
    const v = this.visible();
    this.text([v.xmax, 0], xl, { dx: -10, dy: -10, color: cssVar('--fg-muted') });
    this.text([0, v.ymax], yl, { dx: 10, dy: 12, color: cssVar('--fg-muted') });
  }

  line(a: Pt, b: Pt, s?: Style): void {
    const c = this.ctx; this.applyStyle(s);
    const [ax, ay] = this.toScreen(a); const [bx, by] = this.toScreen(b);
    c.beginPath(); c.moveTo(ax, ay); c.lineTo(bx, by); c.stroke(); this.resetStyle();
  }

  polyline(pts: ArrayLike<Pt>, s?: Style): void {
    if (pts.length < 2) return;
    const c = this.ctx; this.applyStyle(s);
    c.beginPath();
    for (let i = 0; i < pts.length; i++) { const [x, y] = this.toScreen(pts[i]); i ? c.lineTo(x, y) : c.moveTo(x, y); }
    c.stroke(); this.resetStyle();
  }

  polygon(pts: ArrayLike<Pt>, s?: Style): void {
    if (pts.length < 2) return;
    const c = this.ctx; this.applyStyle(s);
    c.beginPath();
    for (let i = 0; i < pts.length; i++) { const [x, y] = this.toScreen(pts[i]); i ? c.lineTo(x, y) : c.moveTo(x, y); }
    c.closePath();
    if (s?.fill) c.fill();
    if (s?.width !== 0) c.stroke();
    this.resetStyle();
  }

  circle(center: Pt, r: number, s?: Style): void {
    const c = this.ctx; this.applyStyle(s);
    const [x, y] = this.toScreen(center);
    c.beginPath(); c.arc(x, y, r * this.sc, 0, Math.PI * 2);
    if (s?.fill) c.fill();
    if (s?.width !== 0) c.stroke();
    this.resetStyle();
  }

  /** Filled disk; radius in world units. */
  disk(center: Pt, r: number, s?: Style): void { this.circle(center, r, { width: 0, ...s, fill: s?.fill ?? s?.color ?? cssVar('--fg') }); }

  /** Dot with radius in px (does not scale with the world). */
  point(p: Pt, rPx = 4, s?: Style): void {
    const c = this.ctx; this.applyStyle(s);
    const [x, y] = this.toScreen(p);
    c.beginPath(); c.arc(x, y, rPx, 0, Math.PI * 2); c.fill(); this.resetStyle();
  }

  /** Arc from angle a0 to a1 (radians, CCW positive in world coords). */
  arc(center: Pt, r: number, a0: number, a1: number, s?: Style): void {
    const c = this.ctx; this.applyStyle(s);
    const [x, y] = this.toScreen(center);
    // Canvas y is flipped, so CCW in world = clockwise on screen with negated angles.
    c.beginPath(); c.arc(x, y, r * this.sc, -a0, -a1, a1 > a0);
    c.stroke(); this.resetStyle();
  }

  /** Ellipse with semi-axes rx, ry (world units) rotated by `angle` radians. */
  ellipse(center: Pt, rx: number, ry: number, angle: number, s?: Style): void {
    const c = this.ctx; this.applyStyle(s);
    const [x, y] = this.toScreen(center);
    c.beginPath(); c.ellipse(x, y, Math.max(0, rx * this.sc), Math.max(0, ry * this.sc), -angle, 0, Math.PI * 2);
    if (s?.fill) c.fill();
    if (s?.width !== 0) c.stroke();
    this.resetStyle();
  }

  /** Arrow from a to b; head size in px. */
  arrow(a: Pt, b: Pt, s?: Style & { head?: number }): void {
    const c = this.ctx; this.applyStyle(s);
    const [ax, ay] = this.toScreen(a); const [bx, by] = this.toScreen(b);
    const dx = bx - ax, dy = by - ay; const L = Math.hypot(dx, dy);
    if (L < 1e-6) { this.resetStyle(); return; }
    const head = Math.min(s?.head ?? 10, L * 0.5);
    const ux = dx / L, uy = dy / L;
    const ex = bx - ux * head * 0.8, ey = by - uy * head * 0.8;
    c.beginPath(); c.moveTo(ax, ay); c.lineTo(ex, ey); c.stroke();
    c.beginPath();
    c.moveTo(bx, by);
    c.lineTo(bx - ux * head - uy * head * 0.45, by - uy * head + ux * head * 0.45);
    c.lineTo(bx - ux * head + uy * head * 0.45, by - uy * head - ux * head * 0.45);
    c.closePath(); c.fillStyle = s?.color ?? cssVar('--fg'); c.fill();
    this.resetStyle();
  }

  /**
   * Coordinate frame. `pose` is {x, y, theta} or a 3×3 homogeneous matrix.
   * Axis colours default to the x (red) / y (green) convention used across the lab.
   */
  frame(pose: { x: number; y: number; theta: number } | number[][], size = 0.5,
    o: { labels?: [string, string] | string; colors?: [string, string]; width?: number; alpha?: number } = {}): void {
    let x: number, y: number, cth: number, sth: number;
    if (Array.isArray(pose)) { x = pose[0][2]; y = pose[1][2]; cth = pose[0][0]; sth = pose[1][0]; }
    else { x = pose.x; y = pose.y; cth = Math.cos(pose.theta); sth = Math.sin(pose.theta); }
    const [cx, cy] = o.colors ?? [cssVar('--axis-x', '#e11'), cssVar('--axis-y', '#1a1')];
    const w = o.width ?? 2.5;
    const ex: Pt = [x + cth * size, y + sth * size];
    const ey: Pt = [x - sth * size, y + cth * size];
    this.arrow([x, y], ex, { color: cx, width: w, alpha: o.alpha });
    this.arrow([x, y], ey, { color: cy, width: w, alpha: o.alpha });
    this.point([x, y], 3, { color: cssVar('--fg') , alpha: o.alpha });
    if (o.labels) {
      const [lx, ly] = typeof o.labels === 'string' ? [`x${o.labels}`, `y${o.labels}`] : o.labels;
      this.text(ex, lx, { color: cx, dx: 8 * cth, dy: -8 * sth });
      this.text(ey, ly, { color: cy, dx: -8 * sth, dy: -8 * cth });
    }
  }

  text(p: Pt, str: string, s: TextStyle = {}): void {
    const c = this.ctx;
    const [x, y] = this.toScreen(p);
    c.font = s.font ?? `12px ${cssVar('--font') || 'sans-serif'}`;
    c.textAlign = s.align ?? 'center';
    c.textBaseline = s.baseline ?? 'middle';
    const tx = x + (s.dx ?? 0), ty = y + (s.dy ?? 0);
    if (s.bg) {
      const m = c.measureText(str);
      const w = m.width + 8, h = 16;
      let bx = tx - w / 2;
      if (c.textAlign === 'left') bx = tx - 4; else if (c.textAlign === 'right') bx = tx - w + 4;
      c.fillStyle = s.bg; c.globalAlpha = 0.85;
      c.beginPath(); c.roundRect(bx, ty - h / 2, w, h, 4); c.fill(); c.globalAlpha = 1;
    }
    c.fillStyle = s.color ?? cssVar('--fg', '#222');
    c.fillText(str, tx, ty);
  }

  /** Convenience: thick rounded "link" segment with joint circles at both ends. */
  link(a: Pt, b: Pt, o: { color?: string; width?: number; joints?: boolean; alpha?: number } = {}): void {
    this.line(a, b, { color: o.color ?? cssVar('--fg-muted'), width: o.width ?? 8, cap: 'round', alpha: o.alpha });
    if (o.joints !== false) {
      this.point(a, (o.width ?? 8) * 0.75, { color: cssVar('--bg-elev'), alpha: o.alpha });
      this.point(a, (o.width ?? 8) * 0.75, { color: cssVar('--fg'), alpha: o.alpha, width: 1 });
    }
  }

  /** Draw an image/canvas so that it covers the world rectangle `rect` (used by RasterLayer). */
  image(img: CanvasImageSource, rect: World, o: { alpha?: number; smooth?: boolean } = {}): void {
    const [sx, sy] = this.toScreen([rect.xmin, rect.ymax]);
    const w = (rect.xmax - rect.xmin) * this.sc, h = (rect.ymax - rect.ymin) * this.sc;
    const c = this.ctx;
    c.save();
    c.globalAlpha = o.alpha ?? 1;
    c.imageSmoothingEnabled = !!o.smooth;
    c.drawImage(img, sx, sy, w, h);
    c.restore();
  }

  // ---- handles & pointer ---------------------------------------------
  addHandle(h: Handle): Handle { this.handles.push(h); this.requestRender(); return h; }
  removeHandle(h: Handle): void { const i = this.handles.indexOf(h); if (i >= 0) this.handles.splice(i, 1); }

  drawHandles(): void {
    for (const h of this.handles) {
      if (h.enabled === false) continue;
      const r = h.radius ?? 8;
      const color = h.color ?? cssVar('--accent', '#36f');
      const isActive = h === this.active || h === this.hover;
      const [x, y] = this.toScreen(h.get());
      const c = this.ctx;
      c.globalAlpha = isActive ? 1 : 0.85;
      c.fillStyle = cssVar('--bg-elev', '#fff'); c.beginPath(); c.arc(x, y, r, 0, Math.PI * 2); c.fill();
      c.lineWidth = isActive ? 3 : 2; c.strokeStyle = color; c.stroke();
      c.fillStyle = color; c.beginPath(); c.arc(x, y, r * 0.4, 0, Math.PI * 2); c.fill();
      c.globalAlpha = 1;
      if (h.label) this.text(h.get(), h.label, { dx: 0, dy: -r - 9, color });
    }
  }

  private hit(sx: number, sy: number): Handle | null {
    let best: Handle | null = null, bestD = Infinity;
    for (const h of this.handles) {
      if (h.enabled === false) continue;
      const [x, y] = this.toScreen(h.get());
      const d = Math.hypot(sx - x, sy - y);
      if (d <= (h.radius ?? 8) + 6 && d < bestD) { best = h; bestD = d; }
    }
    return best;
  }

  private local(ev: PointerEvent): Pt {
    const r = this.canvas.getBoundingClientRect();
    return [ev.clientX - r.left, ev.clientY - r.top];
  }

  /** Register raw pointer callbacks (world coordinates). Returns an unsubscribe function. */
  onPointer(h: { down?: PointerHandler; move?: PointerHandler; up?: PointerHandler }): () => void {
    this.pointerHandlers.push(h);
    return () => { const i = this.pointerHandlers.indexOf(h); if (i >= 0) this.pointerHandlers.splice(i, 1); };
  }

  private bindPointer(): void {
    const cv = this.canvas;
    cv.addEventListener('pointerdown', (ev) => {
      const [sx, sy] = this.local(ev);
      const h = this.hit(sx, sy);
      if (h) {
        this.active = h; cv.setPointerCapture(ev.pointerId); h.onStart?.(); ev.preventDefault();
        this.requestRender();
        return;
      }
      const p = this.toWorld(sx, sy);
      for (const ph of this.pointerHandlers) ph.down?.(p, ev);
    });
    cv.addEventListener('pointermove', (ev) => {
      const [sx, sy] = this.local(ev);
      const p = this.toWorld(sx, sy);
      if (this.active) { this.active.set(p); this.requestRender(); return; }
      const hv = this.hit(sx, sy);
      if (hv !== this.hover) { this.hover = hv; cv.style.cursor = hv ? 'grab' : ''; this.requestRender(); }
      for (const ph of this.pointerHandlers) ph.move?.(p, ev);
    });
    const end = (ev: PointerEvent) => {
      const [sx, sy] = this.local(ev);
      const p = this.toWorld(sx, sy);
      if (this.active) { const h = this.active; this.active = null; h.onEnd?.(); this.requestRender(); return; }
      for (const ph of this.pointerHandlers) ph.up?.(p, ev);
    };
    cv.addEventListener('pointerup', end);
    cv.addEventListener('pointercancel', end);
    cv.addEventListener('pointerleave', () => { if (this.hover) { this.hover = null; cv.style.cursor = ''; this.requestRender(); } });
  }

  destroy(): void {
    this.ro.disconnect();
    window.removeEventListener('mr-theme', this.onTheme);
    this.el.remove();
  }
}
