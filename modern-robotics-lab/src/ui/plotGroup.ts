/** Keep several Plot2D instances aligned: shared x range and a common time cursor. */
import type { Plot2D, Marker } from './plot2d';

export interface PlotGroupHandle {
  /** Draw a vertical cursor at x on every plot (null hides it). */
  setCursor(x: number | null, label?: string): void;
  setXRange(r: [number, number] | null): void;
  /** Per-plot static markers that survive cursor updates. */
  setMarkers(i: number, markers: Marker[]): void;
  clear(): void;
}

export function plotGroup(plots: Plot2D[]): PlotGroupHandle {
  const base: Marker[][] = plots.map(() => []);
  let cursor: Marker | null = null;
  const paint = () => plots.forEach((p, i) => p.setMarkers(cursor ? [...base[i], cursor] : base[i]));
  return {
    setCursor(x, label) { cursor = x == null ? null : { x, label, dash: [2, 3] }; paint(); },
    setXRange(r) { plots.forEach((p) => p.setRanges(r, undefined)); },
    setMarkers(i, m) { base[i] = m; paint(); },
    clear() { plots.forEach((p) => p.clear()); },
  };
}
