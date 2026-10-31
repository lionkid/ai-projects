/** Lightweight canvas line plot for time series and XY curves (DPR-aware, themed). */
import { el, cssVar, rafThrottle, themeColor } from './dom';

export interface SeriesSpec {
  name: string;
  color?: string;
  width?: number;
  dash?: number[];
  /** Draw dots instead of a line (scatter). */
  points?: boolean;
}

export interface Marker {
  x?: number;
  y?: number;
  color?: string;
  label?: string;
  dash?: number[];
}

export interface Plot2DOptions {
  title?: string;
  xLabel?: string;
  yLabel?: string;
  series: SeriesSpec[];
  /** Fixed x range; omit for auto. */
  xRange?: [number, number];
  /** Fixed y range; omit for auto. */
  yRange?: [number, number];
  /** Streaming mode: keep only the last `xWindow` units of x visible/stored. */
  xWindow?: number;
  /** Max points stored per series (ring buffer). Default 4000. */
  maxPoints?: number;
  /** Canvas height in CSS px. Default 220. */
  height?: number;
  legend?: boolean;
  grid?: boolean;
  /** Fractional padding around auto y range. Default 0.08. */
  yPad?: number;
  /** Minimum y span for auto range, avoids flat-line zoom. Default 1e-3. */
  yMinSpan?: number;
  /** Use equal scaling for x and y (phase portraits, workspaces). */
  equalAspect?: boolean;
}

function niceStep(span: number, target = 5): number {
  const raw = span / target;
  const mag = Math.pow(10, Math.floor(Math.log10(raw)));
  const r = raw / mag;
  const step = r < 1.5 ? 1 : r < 3 ? 2 : r < 7 ? 5 : 10;
  return step * mag;
}

export function niceTicks(min: number, max: number, target = 5): number[] {
  if (!(max > min)) return [min];
  const step = niceStep(max - min, target);
  const out: number[] = [];
  const start = Math.ceil(min / step - 1e-9) * step;
  for (let v = start; v <= max + 1e-9; v += step) out.push(Math.abs(v) < step * 1e-6 ? 0 : v);
  return out;
}

function tickFmt(step: number): (v: number) => string {
  const d = Math.max(0, Math.min(6, Math.ceil(-Math.log10(step) - 1e-9)));
  return (v) => {
    const s = v.toFixed(d);
    return s === `-${(0).toFixed(d)}` ? (0).toFixed(d) : s;
  };
}

export class Plot2D {
  readonly el: HTMLDivElement;
  readonly canvas: HTMLCanvasElement;
  private ctx: CanvasRenderingContext2D;
  private xs: number[][];
  private ys: number[][];
  private markers: Marker[] = [];
  private ro: ResizeObserver;
  private opts: Plot2DOptions;
  private onTheme = () => this.requestRender();
  readonly requestRender: () => void;

  constructor(parent: HTMLElement, opts: Plot2DOptions) {
    this.opts = { ...opts };
    this.canvas = el('canvas');
    this.el = el('div', { class: 'mr-plot' }, this.canvas);
    parent.appendChild(this.el);
    this.ctx = this.canvas.getContext('2d')!;
    this.xs = opts.series.map(() => []);
    this.ys = opts.series.map(() => []);
    this.requestRender = rafThrottle(() => this.draw());
    this.ro = new ResizeObserver(() => this.requestRender());
    this.ro.observe(this.el);
    window.addEventListener('mr-theme', this.onTheme);
    this.requestRender();
  }

  /** Append one sample to every series (time-series usage). */
  push(x: number, ys: ArrayLike<number>): void {
    const max = this.opts.maxPoints ?? 4000;
    for (let i = 0; i < this.xs.length; i++) {
      const y = ys[i];
      if (y === undefined) continue;
      this.xs[i].push(x);
      this.ys[i].push(y);
      if (this.xs[i].length > max) { this.xs[i].splice(0, this.xs[i].length - max); this.ys[i].splice(0, this.ys[i].length - max); }
      const w = this.opts.xWindow;
      if (w != null) {
        const cutoff = x - w;
        let k = 0;
        while (k < this.xs[i].length - 1 && this.xs[i][k + 1] < cutoff) k++;
        if (k > 0) { this.xs[i].splice(0, k); this.ys[i].splice(0, k); }
      }
    }
    this.requestRender();
  }

  /** Replace series `i` with the given points ([x, y] tuples or {x, y}). */
  setSeries(i: number, pts: ArrayLike<[number, number]> | { x: number; y: number }[]): void {
    const xs: number[] = []; const ys: number[] = [];
    for (let k = 0; k < pts.length; k++) {
      const p = pts[k] as [number, number] | { x: number; y: number };
      if (Array.isArray(p)) { xs.push(p[0]); ys.push(p[1]); } else { xs.push(p.x); ys.push(p.y); }
    }
    this.xs[i] = xs; this.ys[i] = ys;
    this.requestRender();
  }

  /** Replace series `i` from separate x and y arrays. */
  setXY(i: number, xs: ArrayLike<number>, ys: ArrayLike<number>): void {
    this.xs[i] = Array.from(xs); this.ys[i] = Array.from(ys);
    this.requestRender();
  }

  setAll(data: [number, number][][]): void { data.forEach((d, i) => this.setSeries(i, d)); }

  clear(): void {
    this.xs = this.xs.map(() => []);
    this.ys = this.ys.map(() => []);
    this.requestRender();
  }

  setMarkers(m: Marker[]): void { this.markers = m; this.requestRender(); }

  setRanges(x?: [number, number] | null, y?: [number, number] | null): void {
    if (x !== undefined) this.opts.xRange = x ?? undefined;
    if (y !== undefined) this.opts.yRange = y ?? undefined;
    this.requestRender();
  }

  setOptions(o: Partial<Plot2DOptions>): void { Object.assign(this.opts, o); this.requestRender(); }

  destroy(): void {
    this.ro.disconnect();
    window.removeEventListener('mr-theme', this.onTheme);
    this.el.remove();
  }

  private computeRange(axis: 'x' | 'y'): [number, number] {
    const fixed = axis === 'x' ? this.opts.xRange : this.opts.yRange;
    if (fixed) return fixed;
    const data = axis === 'x' ? this.xs : this.ys;
    let lo = Infinity, hi = -Infinity;
    // Only the configured series count (arrays left over after setOptions must not drive the range).
    for (const arr of data.slice(0, this.opts.series.length)) for (const v of arr) { if (Number.isFinite(v)) { if (v < lo) lo = v; if (v > hi) hi = v; } }
    for (const m of this.markers) { const v = axis === 'x' ? m.x : m.y; if (v != null && Number.isFinite(v)) { if (v < lo) lo = v; if (v > hi) hi = v; } }
    if (!Number.isFinite(lo)) return axis === 'x' ? [0, 1] : [-1, 1];
    if (axis === 'x') {
      if (this.opts.xWindow != null) return [Math.max(lo, hi - this.opts.xWindow), Math.max(hi, lo + 1e-9)];
      return hi > lo ? [lo, hi] : [lo - 0.5, hi + 0.5];
    }
    const minSpan = this.opts.yMinSpan ?? 1e-3;
    if (hi - lo < minSpan) { const c = (hi + lo) / 2; lo = c - minSpan / 2; hi = c + minSpan / 2; }
    const pad = (hi - lo) * (this.opts.yPad ?? 0.08);
    return [lo - pad, hi + pad];
  }

  private draw(): void {
    const W = this.el.clientWidth;
    if (W <= 0) return;
    const H = this.opts.height ?? 220;
    const dpr = Math.min(2, window.devicePixelRatio || 1);
    if (this.canvas.width !== Math.round(W * dpr) || this.canvas.height !== Math.round(H * dpr)) {
      this.canvas.width = Math.round(W * dpr);
      this.canvas.height = Math.round(H * dpr);
      this.canvas.style.height = `${H}px`;
    }
    const ctx = this.ctx;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    const fg = cssVar('--fg', '#222');
    const muted = cssVar('--fg-muted', '#777');
    const gridC = cssVar('--grid', '#e5e5e5');
    const gridS = cssVar('--grid-strong', '#ccc');
    const bg = cssVar('--sim-bg', '#fff');
    const palette = Array.from({ length: 8 }, (_, i) => cssVar(`--series-${i + 1}`, '#36f'));
    ctx.fillStyle = bg;
    ctx.fillRect(0, 0, W, H);

    const o = this.opts;
    const mL = o.yLabel ? 58 : 46, mR = 14, mT = o.title ? 26 : 10, mB = o.xLabel ? 36 : 24;
    const pw = Math.max(10, W - mL - mR), ph = Math.max(10, H - mT - mB);

    let [x0, x1] = this.computeRange('x');
    let [y0, y1] = this.computeRange('y');
    if (o.equalAspect) {
      const sx = pw / (x1 - x0), sy = ph / (y1 - y0);
      if (sx > sy) { const cx = (x0 + x1) / 2, half = (pw / sy) / 2; x0 = cx - half; x1 = cx + half; }
      else { const cy = (y0 + y1) / 2, half = (ph / sx) / 2; y0 = cy - half; y1 = cy + half; }
    }
    const X = (x: number) => mL + ((x - x0) / (x1 - x0)) * pw;
    const Y = (y: number) => mT + ph - ((y - y0) / (y1 - y0)) * ph;

    // Grid + ticks
    ctx.font = '11px ' + (cssVar('--mono') || 'monospace');
    ctx.textBaseline = 'middle';
    const xt = niceTicks(x0, x1, Math.max(3, Math.floor(pw / 70)));
    const yt = niceTicks(y0, y1, Math.max(3, Math.floor(ph / 40)));
    const fx = tickFmt(xt.length > 1 ? xt[1] - xt[0] : 1);
    const fy = tickFmt(yt.length > 1 ? yt[1] - yt[0] : 1);
    ctx.lineWidth = 1;
    for (const v of xt) {
      const px = Math.round(X(v)) + 0.5;
      if (o.grid !== false) { ctx.strokeStyle = v === 0 ? gridS : gridC; ctx.beginPath(); ctx.moveTo(px, mT); ctx.lineTo(px, mT + ph); ctx.stroke(); }
      ctx.fillStyle = muted; ctx.textAlign = 'center'; ctx.fillText(fx(v), px, mT + ph + 11);
    }
    for (const v of yt) {
      const py = Math.round(Y(v)) + 0.5;
      if (o.grid !== false) { ctx.strokeStyle = v === 0 ? gridS : gridC; ctx.beginPath(); ctx.moveTo(mL, py); ctx.lineTo(mL + pw, py); ctx.stroke(); }
      ctx.fillStyle = muted; ctx.textAlign = 'right'; ctx.fillText(fy(v), mL - 6, py);
    }
    // Frame
    ctx.strokeStyle = gridS; ctx.strokeRect(mL + 0.5, mT + 0.5, pw, ph);

    // Labels
    ctx.fillStyle = fg;
    ctx.font = '12px ' + (cssVar('--font') || 'sans-serif');
    if (o.title) { ctx.textAlign = 'left'; ctx.fillText(o.title, mL, 12); }
    if (o.xLabel) { ctx.textAlign = 'center'; ctx.fillText(o.xLabel, mL + pw / 2, H - 9); }
    if (o.yLabel) {
      ctx.save(); ctx.translate(12, mT + ph / 2); ctx.rotate(-Math.PI / 2); ctx.textAlign = 'center'; ctx.fillText(o.yLabel, 0, 0); ctx.restore();
    }

    // Series (clipped)
    ctx.save();
    ctx.beginPath(); ctx.rect(mL, mT, pw, ph); ctx.clip();
    o.series.forEach((s, i) => {
      const xs = this.xs[i], ys = this.ys[i];
      if (!xs?.length) return;
      const color = s.color ? themeColor(s.color, palette[i % palette.length]) : palette[i % palette.length];
      ctx.strokeStyle = color; ctx.fillStyle = color;
      ctx.lineWidth = s.width ?? 1.8;
      ctx.lineJoin = 'round'; ctx.lineCap = 'round';
      ctx.setLineDash(s.dash ?? []);
      if (s.points) {
        for (let k = 0; k < xs.length; k++) { ctx.beginPath(); ctx.arc(X(xs[k]), Y(ys[k]), 2.2, 0, Math.PI * 2); ctx.fill(); }
      } else {
        ctx.beginPath();
        let pen = false;
        for (let k = 0; k < xs.length; k++) {
          const x = xs[k], y = ys[k];
          if (!Number.isFinite(x) || !Number.isFinite(y)) { pen = false; continue; }
          if (!pen) { ctx.moveTo(X(x), Y(y)); pen = true; } else ctx.lineTo(X(x), Y(y));
        }
        ctx.stroke();
      }
      ctx.setLineDash([]);
    });
    // Markers
    for (const m of this.markers) {
      const mc = m.color ? themeColor(m.color, muted) : muted;
      ctx.strokeStyle = mc; ctx.fillStyle = mc; ctx.lineWidth = 1; ctx.setLineDash(m.dash ?? [4, 4]);
      if (m.x != null && m.y != null) {
        ctx.setLineDash([]); ctx.beginPath(); ctx.arc(X(m.x), Y(m.y), 4, 0, Math.PI * 2); ctx.fill();
        if (m.label) { ctx.textAlign = 'left'; ctx.fillText(m.label, X(m.x) + 6, Y(m.y) - 8); }
      } else if (m.x != null) {
        ctx.beginPath(); ctx.moveTo(X(m.x), mT); ctx.lineTo(X(m.x), mT + ph); ctx.stroke();
        if (m.label) { ctx.textAlign = 'left'; ctx.fillText(m.label, X(m.x) + 4, mT + 10); }
      } else if (m.y != null) {
        ctx.beginPath(); ctx.moveTo(mL, Y(m.y)); ctx.lineTo(mL + pw, Y(m.y)); ctx.stroke();
        if (m.label) { ctx.textAlign = 'right'; const ly = Y(m.y) - 8 < mT + 8 ? Y(m.y) + 12 : Y(m.y) - 8; ctx.fillText(m.label, mL + pw - 4, ly); }
      }
      ctx.setLineDash([]);
    }
    ctx.restore();

    // Legend
    if (o.legend !== false && (o.series.length > 1 || o.legend)) {
      ctx.font = '11px ' + (cssVar('--font') || 'sans-serif');
      ctx.textAlign = 'left';
      let lx = mL + 8, ly = mT + 10;
      o.series.forEach((s, i) => {
        if (!this.xs[i]?.length && o.series.length > 1) return; // no data yet: keep it out of the legend
        const color = s.color ? themeColor(s.color, palette[i % palette.length]) : palette[i % palette.length];
        const tw = ctx.measureText(s.name).width;
        if (lx + 18 + tw > mL + pw - 4) { lx = mL + 8; ly += 15; }
        ctx.fillStyle = color; ctx.fillRect(lx, ly - 4, 12, 3);
        ctx.fillStyle = fg; ctx.fillText(s.name, lx + 16, ly - 2);
        lx += 24 + tw;
      });
    }
  }
}
