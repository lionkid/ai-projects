/** Signed horizontal bars (joint torques, energies, errors) with an optional saturation limit. */
import { el, fmt } from './dom';

/** CSS colour for a bar: '--x' becomes var(--x) so the browser follows theme switches. */
const cssColor = (c: string | undefined): string | undefined => (c && c.startsWith('--') ? `var(${c})` : c);

export interface BarsOpts {
  labels: string[];
  /** Full-scale magnitude (bar reaches the edge at ±max). */
  max: number;
  unit?: string;
  /** Values beyond ±limit are drawn in the danger colour (actuator saturation). */
  limit?: number;
  colors?: string[];
  digits?: number;
  title?: string;
}

export interface BarsHandle { el: HTMLElement; set(values: ArrayLike<number>): void; setMax(max: number): void; setLimit(limit?: number): void }

export function torqueBars(parent: HTMLElement, opts: BarsOpts): BarsHandle {
  let max = opts.max; let limit = opts.limit;
  const rows = opts.labels.map((label, i) => {
    const fill = el('span', { class: 'mr-bar-fill', style: { background: cssColor(opts.colors?.[i]) ?? `var(--series-${(i % 8) + 1})` } });
    const lim1 = el('span', { class: 'mr-bar-limit' }); const lim2 = el('span', { class: 'mr-bar-limit' });
    const track = el('span', { class: 'mr-bar-track' }, el('span', { class: 'mr-bar-zero' }), lim1, lim2, fill);
    const val = el('span', { class: 'mr-bar-value' }, '–');
    const row = el('div', { class: 'mr-bar-row' }, el('span', { class: 'mr-bar-label' }, label), track, val);
    return { row, fill, val, lim1, lim2 };
  });
  const wrap = el('div', { class: 'mr-bars' }, opts.title ? el('div', { class: 'mr-bars-title' }, opts.title) : null, rows.map((r) => r.row));
  parent.appendChild(wrap);
  const unit = opts.unit ? ` ${opts.unit}` : '';
  const digits = opts.digits ?? 2;
  const paintLimits = () => rows.forEach((r) => {
    const show = limit != null && limit < max;
    r.lim1.style.display = r.lim2.style.display = show ? '' : 'none';
    if (show) { const f = (limit! / max) * 50; r.lim1.style.left = `${50 - f}%`; r.lim2.style.left = `${50 + f}%`; }
  });
  paintLimits();
  return {
    el: wrap,
    set(values) {
      rows.forEach((r, i) => {
        const v = values[i] ?? 0;
        const frac = Math.max(-1, Math.min(1, v / max));
        const w = Math.abs(frac) * 50;
        r.fill.style.width = `${w}%`;
        r.fill.style.left = frac >= 0 ? '50%' : `${50 - w}%`;
        const sat = limit != null && Math.abs(v) > limit + 1e-9;
        r.fill.classList.toggle('sat', sat);
        r.val.textContent = fmt(v, digits) + unit + (sat ? ' ⚠' : '');
      });
    },
    setMax(m) { max = m; paintLimits(); },
    setLimit(l) { limit = l; paintLimits(); },
  };
}
