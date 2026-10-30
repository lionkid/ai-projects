/** Form-like controls for simulation panels (sliders, toggles, readouts, matrices, playback). */
import { el, fmt } from './dom';

/** A responsive grid that holds controls. */
export function controlPanel(parent: HTMLElement, opts: { compact?: boolean } = {}): HTMLDivElement {
  const panel = el('div', { class: `mr-controls${opts.compact ? ' compact' : ''}` });
  parent.appendChild(panel);
  return panel;
}

/** A full-width row inside a control panel (for buttons, toggles, readouts). */
export function row(panel: HTMLElement): HTMLDivElement {
  const r = el('div', { class: 'mr-row' });
  panel.appendChild(r);
  return r;
}

export interface SliderOpts {
  label: string;
  min: number;
  max: number;
  step?: number;
  value: number;
  /** Appended after the formatted value, e.g. 'm' or 'rad'. */
  unit?: string;
  format?: (v: number) => string;
  /** Fired continuously while dragging. */
  onInput?: (v: number) => void;
  /** Fired when the drag ends. */
  onChange?: (v: number) => void;
  title?: string;
}

export interface SliderHandle {
  el: HTMLElement;
  input: HTMLInputElement;
  readonly value: number;
  /** Set the value; fires onInput unless `silent`. */
  set(v: number, o?: { silent?: boolean }): void;
  setRange(min: number, max: number, step?: number): void;
  setLabel(label: string): void;
  disabled: boolean;
}

export function slider(parent: HTMLElement, opts: SliderOpts): SliderHandle {
  const format = opts.format ?? ((v: number) => fmt(v, decimalsFor(opts.step ?? 0.01)));
  const unit = opts.unit ? ` ${opts.unit}` : '';
  const labelEl = el('span', {}, opts.label);
  const valueEl = el('span', { class: 'mr-value' });
  const input = el('input', {
    type: 'range', min: opts.min, max: opts.max, step: opts.step ?? 0.01, value: opts.value, title: opts.title,
  });
  const wrap = el('label', { class: 'mr-control' }, el('span', { class: 'mr-label' }, labelEl, valueEl), input);
  parent.appendChild(wrap);
  const show = () => { valueEl.textContent = format(Number(input.value)) + unit; };
  show();
  input.addEventListener('input', () => { show(); opts.onInput?.(Number(input.value)); });
  input.addEventListener('change', () => { opts.onChange?.(Number(input.value)); });
  return {
    el: wrap,
    input,
    get value() { return Number(input.value); },
    set(v, o) {
      input.value = String(v);
      show();
      if (!o?.silent) opts.onInput?.(Number(input.value));
    },
    setRange(min, max, step) {
      input.min = String(min); input.max = String(max);
      if (step != null) input.step = String(step);
      show();
    },
    setLabel(l) { labelEl.textContent = l; },
    get disabled() { return input.disabled; },
    set disabled(d: boolean) { input.disabled = d; },
  };
}

function decimalsFor(step: number): number {
  if (step >= 1) return 0;
  return Math.min(4, Math.max(0, Math.ceil(-Math.log10(step))));
}

/** Slider for an angle stored in radians but displayed in degrees. */
export function angleSlider(
  parent: HTMLElement,
  opts: { label: string; value: number; min?: number; max?: number; step?: number; onInput?: (rad: number) => void; onChange?: (rad: number) => void; title?: string },
): SliderHandle {
  return slider(parent, {
    label: opts.label,
    min: opts.min ?? -Math.PI,
    max: opts.max ?? Math.PI,
    step: opts.step ?? Math.PI / 180,
    value: opts.value,
    format: (v) => `${fmt((v * 180) / Math.PI, 0)}°`,
    onInput: opts.onInput,
    onChange: opts.onChange,
    title: opts.title,
  });
}

export interface ToggleHandle { el: HTMLElement; readonly value: boolean; set(v: boolean, o?: { silent?: boolean }): void }

export function toggle(parent: HTMLElement, opts: { label: string; value?: boolean; onChange?: (v: boolean) => void; title?: string }): ToggleHandle {
  const input = el('input', { type: 'checkbox' });
  input.checked = !!opts.value;
  const wrap = el('label', { class: 'mr-toggle', title: opts.title }, input, el('span', {}, opts.label));
  parent.appendChild(wrap);
  input.addEventListener('change', () => opts.onChange?.(input.checked));
  return {
    el: wrap,
    get value() { return input.checked; },
    set(v, o) { input.checked = v; if (!o?.silent) opts.onChange?.(v); },
  };
}

export function button(
  parent: HTMLElement,
  opts: { label: string; onClick: (ev: MouseEvent) => void; variant?: 'primary' | 'ghost' | ''; small?: boolean; title?: string },
): HTMLButtonElement {
  const b = el('button', {
    class: `btn ${opts.variant ?? ''} ${opts.small ? 'small' : ''}`.trim(), type: 'button', title: opts.title, onClick: opts.onClick,
  }, opts.label);
  parent.appendChild(b);
  return b;
}

export interface SelectHandle<T extends string> { el: HTMLElement; readonly value: T; set(v: T, o?: { silent?: boolean }): void }

export function select<T extends string>(
  parent: HTMLElement,
  opts: { label?: string; options: { value: T; label: string }[]; value: T; onChange?: (v: T) => void },
): SelectHandle<T> {
  const sel = el('select');
  for (const o of opts.options) sel.appendChild(el('option', { value: o.value }, o.label));
  sel.value = opts.value;
  const wrap = el('label', { class: 'mr-control' }, opts.label ? el('span', { class: 'mr-label' }, el('span', {}, opts.label)) : null, sel);
  parent.appendChild(wrap);
  sel.addEventListener('change', () => opts.onChange?.(sel.value as T));
  return {
    el: wrap,
    get value() { return sel.value as T; },
    set(v, o) { sel.value = v; if (!o?.silent) opts.onChange?.(v); },
  };
}

export interface ReadoutHandle { el: HTMLElement; set(v: string | number, digits?: number): void }

/** A labelled live value, e.g. `θ = 1.234 rad`. */
export function readout(parent: HTMLElement, opts: { label: string; value?: string; unit?: string; title?: string }): ReadoutHandle {
  const valueEl = el('span', { class: 'mr-value' }, opts.value ?? '–');
  const wrap = el('div', { class: 'mr-readout', title: opts.title }, el('span', { class: 'mr-label' }, opts.label), valueEl);
  parent.appendChild(wrap);
  const unit = opts.unit ? ` ${opts.unit}` : '';
  return {
    el: wrap,
    set(v, digits = 3) { valueEl.textContent = (typeof v === 'number' ? fmt(v, digits) : v) + unit; },
  };
}

export type MatrixHighlight = ('x' | 'y' | 'z' | 'hl' | null)[];

export interface MatrixHandle {
  el: HTMLElement;
  /** Accepts a nested array (rows) or a flat vector (rendered as a column). */
  set(M: number[][] | number[], o?: { highlightCols?: MatrixHighlight; highlightRows?: MatrixHighlight; digits?: number }): void;
  setLabel(label: string): void;
}

/** Render a matrix with brackets, optionally colouring columns (x/y/z) to match frame axes. */
export function matrixView(parent: HTMLElement, opts: { label?: string; digits?: number; title?: string }): MatrixHandle {
  const labelEl = el('span', { class: 'mr-mlabel' }, opts.label ?? '');
  const grid = el('div', { class: 'mr-grid' });
  const wrap = el('div', { class: 'mr-matrix', title: opts.title }, labelEl, grid);
  parent.appendChild(wrap);
  const digits = opts.digits ?? 3;
  return {
    el: wrap,
    setLabel(l) { labelEl.textContent = l; },
    set(M, o) {
      const rows: number[][] = Array.isArray(M[0]) ? (M as number[][]) : (M as number[]).map((x) => [x]);
      const cols = rows[0]?.length ?? 1;
      grid.style.gridTemplateColumns = `repeat(${cols}, auto)`;
      grid.replaceChildren();
      const d = o?.digits ?? digits;
      rows.forEach((r, i) => r.forEach((x, j) => {
        const hc = o?.highlightCols?.[j];
        const hr = o?.highlightRows?.[i];
        const cls = hc ? (hc === 'hl' ? 'hl' : `hl-${hc}`) : hr ? (hr === 'hl' ? 'hl' : `hl-${hr}`) : '';
        grid.appendChild(el('span', { class: `mr-cell ${cls}`.trim() }, fmt(x, d)));
      }));
    },
  };
}

export interface PlaybackOpts {
  onPlay?: () => void;
  onPause?: () => void;
  onReset?: () => void;
  onSpeed?: (s: number) => void;
  speeds?: number[];
  /** When given, a scrubber is shown and `onScrub` is called with the time. */
  duration?: number;
  onScrub?: (t: number) => void;
  showTime?: boolean;
  /** Start in the playing state (default false). */
  playing?: boolean;
}

export interface PlaybackHandle {
  el: HTMLElement;
  readonly playing: boolean;
  setPlaying(p: boolean, o?: { silent?: boolean }): void;
  setTime(t: number): void;
  setDuration(d: number): void;
  readonly speed: number;
}

/** Transport controls: play/pause, reset, speed, optional scrubber. */
export function playback(parent: HTMLElement, opts: PlaybackOpts = {}): PlaybackHandle {
  let playing = !!opts.playing;
  let duration = opts.duration ?? 0;
  const playBtn = el('button', { class: 'btn small', type: 'button', title: '播放 / 暫停 (space)' }, playing ? '⏸ 暫停' : '▶ 播放');
  const resetBtn = el('button', { class: 'btn small', type: 'button', title: '重設' }, '⟲ 重設');
  const speeds = opts.speeds ?? [0.25, 0.5, 1, 2, 4];
  const speedSel = el('select', { title: '播放速度' });
  for (const s of speeds) speedSel.appendChild(el('option', { value: s }, `${s}×`));
  speedSel.value = '1';
  const scrub = el('input', { type: 'range', min: 0, max: duration || 1, step: 0.001, value: 0 });
  const timeEl = el('span', { class: 'mr-time' }, '0.00 s');
  const wrap = el('div', { class: 'mr-playback' }, playBtn, resetBtn, speedSel,
    opts.duration != null ? scrub : null,
    opts.showTime !== false ? timeEl : null);
  parent.appendChild(wrap);

  const setPlaying = (p: boolean, o?: { silent?: boolean }) => {
    playing = p;
    playBtn.textContent = p ? '⏸ 暫停' : '▶ 播放';
    if (!o?.silent) (p ? opts.onPlay : opts.onPause)?.();
  };
  playBtn.addEventListener('click', () => setPlaying(!playing));
  resetBtn.addEventListener('click', () => { opts.onReset?.(); });
  speedSel.addEventListener('change', () => opts.onSpeed?.(Number(speedSel.value)));
  scrub.addEventListener('input', () => { opts.onScrub?.(Number(scrub.value)); timeEl.textContent = `${fmt(Number(scrub.value), 2)} s`; });
  return {
    el: wrap,
    get playing() { return playing; },
    setPlaying,
    setTime(t) {
      timeEl.textContent = `${fmt(t, 2)} s`;
      if (document.activeElement !== scrub) scrub.value = String(t);
    },
    setDuration(d) { duration = d; scrub.max = String(d || 1); },
    get speed() { return Number(speedSel.value); },
  };
}

/** Small grey explanatory text under a widget. */
export function note(parent: HTMLElement, text: string): HTMLElement {
  const n = el('p', { class: 'mr-note' }, text);
  parent.appendChild(n);
  return n;
}

/** Title line inside the sim pane, optionally with a badge. */
export function simTitle(parent: HTMLElement, text: string, badge?: string): HTMLElement {
  const t = el('div', { class: 'mr-sim-title' }, text, badge ? el('span', { class: 'badge accent' }, badge) : null);
  parent.appendChild(t);
  return t;
}

/** Colour legend. */
export function legend(parent: HTMLElement, items: { color: string; label: string }[]): HTMLElement {
  const l = el('div', { class: 'mr-legend' }, items.map((i) => el('span', {}, el('span', { class: 'sw', style: { background: i.color } }), i.label)));
  parent.appendChild(l);
  return l;
}

export interface KVHandle { el: HTMLElement; set(key: string, value: string | number, digits?: number): void }

/** Key/value table for live numeric state. */
export function kv(parent: HTMLElement, keys: string[]): KVHandle {
  const dl = el('dl', { class: 'mr-kv' });
  const cells = new Map<string, HTMLElement>();
  for (const k of keys) {
    const dd = el('dd', {}, '–');
    cells.set(k, dd);
    dl.append(el('dt', {}, k), dd);
  }
  parent.appendChild(dl);
  return {
    el: dl,
    set(k, v, digits = 3) {
      let dd = cells.get(k);
      if (!dd) { dd = el('dd', {}, '–'); cells.set(k, dd); dl.append(el('dt', {}, k), dd); }
      dd.textContent = typeof v === 'number' ? fmt(v, digits) : v;
    },
  };
}

/** Two-column layout helper for placing e.g. a canvas next to a plot. */
export function split(parent: HTMLElement): [HTMLDivElement, HTMLDivElement] {
  const a = el('div');
  const b = el('div');
  parent.appendChild(el('div', { class: 'mr-split' }, a, b));
  return [a, b];
}
