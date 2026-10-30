/** Transport for iterative algorithms: single step / auto-run / reset / speed / iteration counter. */
import { el } from './dom';

export interface AlgoStepperOpts {
  /** Perform one iteration. Return false when the algorithm has finished. */
  onStep: () => boolean | void;
  onReset: () => void;
  /** Milliseconds between auto steps at 1× (default 400). */
  intervalMs?: number;
  speeds?: number[];
  countLabel?: string;
}

export interface AlgoStepperHandle {
  el: HTMLElement;
  step(): void;
  run(): void;
  pause(): void;
  reset(): void;
  readonly running: boolean;
  readonly count: number;
  /** Mark finished (disables step/run until reset) and show a status text. */
  finish(status?: string): void;
  setStatus(text: string): void;
  destroy(): void;
}

export function algorithmStepper(parent: HTMLElement, opts: AlgoStepperOpts): AlgoStepperHandle {
  let timer = 0;
  let count = 0;
  let finished = false;
  const stepBtn = el('button', { class: 'btn small', type: 'button', title: '執行一步' }, '▸ 單步');
  const runBtn = el('button', { class: 'btn small primary', type: 'button', title: '自動執行 / 暫停' }, '▶ 自動');
  const resetBtn = el('button', { class: 'btn small', type: 'button' }, '⟲ 重設');
  const speeds = opts.speeds ?? [0.5, 1, 2, 5, 20];
  const speedSel = el('select', { title: '速度' });
  for (const s of speeds) speedSel.appendChild(el('option', { value: s }, `${s}×`));
  speedSel.value = '1';
  const countEl = el('span', { class: 'mr-time' }, `${opts.countLabel ?? '迭代'} 0`);
  const statusEl = el('span', { class: 'mr-note', style: { margin: 0 } });
  const wrap = el('div', { class: 'mr-playback' }, stepBtn, runBtn, resetBtn, speedSel, countEl, statusEl);
  parent.appendChild(wrap);

  const paint = () => {
    countEl.textContent = `${opts.countLabel ?? '迭代'} ${count}`;
    stepBtn.disabled = finished; runBtn.disabled = finished;
    runBtn.textContent = timer ? '⏸ 暫停' : '▶ 自動';
  };
  const doStep = () => {
    if (finished) return;
    count++;
    const r = opts.onStep();
    if (r === false) { finished = true; pause(); }
    paint();
  };
  const run = () => {
    if (timer || finished) return;
    const ms = (opts.intervalMs ?? 400) / Number(speedSel.value);
    timer = window.setInterval(doStep, ms);
    paint();
  };
  const pause = () => { if (timer) { clearInterval(timer); timer = 0; } paint(); };
  const reset = () => { pause(); count = 0; finished = false; statusEl.textContent = ''; opts.onReset(); paint(); };
  stepBtn.addEventListener('click', doStep);
  runBtn.addEventListener('click', () => (timer ? pause() : run()));
  resetBtn.addEventListener('click', reset);
  speedSel.addEventListener('change', () => { if (timer) { pause(); run(); } });
  paint();
  return {
    el: wrap,
    step: doStep, run, pause, reset,
    get running() { return !!timer; },
    get count() { return count; },
    finish(status) { finished = true; pause(); if (status) statusEl.textContent = status; },
    setStatus(t) { statusEl.textContent = t; },
    destroy() { pause(); },
  };
}
