/** A bank of angle sliders generated from a robot model (or a joint count). */
import { angleSlider, type SliderHandle } from './controls';
import { el } from './dom';

export interface JointSlidersOpts {
  /** Robot model (dof + optional jointLimits) or a joint count. */
  model?: { dof: number; jointLimits?: [number, number][] };
  n?: number;
  limits?: [number, number][];
  values?: number[];
  labels?: string[];
  /** Called on every slider move with the full joint vector and the index that changed. */
  onInput?: (thetas: number[], index: number) => void;
  compact?: boolean;
}

export interface JointSlidersHandle {
  el: HTMLElement;
  sliders: SliderHandle[];
  get(): number[];
  /** Set all joint values (fires onInput once with index −1 unless silent). */
  set(thetas: ArrayLike<number>, o?: { silent?: boolean }): void;
  disabled: boolean;
}

const SUB = '₀₁₂₃₄₅₆₇₈₉';
export const subscript = (n: number): string => String(n).replace(/\d/g, (d) => SUB[Number(d)]);

export function jointSliders(parent: HTMLElement, opts: JointSlidersOpts = {}): JointSlidersHandle {
  const n = opts.model?.dof ?? opts.n ?? 2;
  const limits = opts.limits ?? opts.model?.jointLimits ?? Array.from({ length: n }, () => [-Math.PI, Math.PI] as [number, number]);
  const values = Array.from({ length: n }, (_, i) => opts.values?.[i] ?? 0);
  const wrap = el('div', { class: `mr-controls ${opts.compact !== false ? 'compact' : ''}` });
  parent.appendChild(wrap);
  const sliders: SliderHandle[] = [];
  for (let i = 0; i < n; i++) {
    sliders.push(angleSlider(wrap, {
      label: opts.labels?.[i] ?? `θ${subscript(i + 1)}`,
      value: values[i], min: limits[i][0], max: limits[i][1],
      onInput: (v) => { values[i] = v; opts.onInput?.(values.slice(), i); },
    }));
  }
  return {
    el: wrap,
    sliders,
    get: () => values.slice(),
    set(th, o) {
      for (let i = 0; i < n; i++) { values[i] = th[i] ?? values[i]; sliders[i].set(values[i], { silent: true }); }
      if (!o?.silent) opts.onInput?.(values.slice(), -1);
    },
    get disabled() { return sliders[0]?.disabled ?? false; },
    set disabled(d: boolean) { sliders.forEach((s) => { s.disabled = d; }); },
  };
}
