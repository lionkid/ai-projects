/**
 * A small matrix display for this lesson: like the toolkit's matrixView, but with stable cells so that
 * columns can be hovered / clicked (to highlight the matching {b} axis) and edited in place.
 */
import { el, fmt } from '@/ui';

export type ColTone = 'x' | 'y' | 'z' | null;

export interface MatrixWidgetOpts {
  label?: Node | string;
  rows?: number;
  cols?: number;
  digits?: number;
  colTones?: ColTone[];
  onHoverCol?: (j: number | null) => void;
  onClickCol?: (j: number) => void;
  onEdit?: (i: number, j: number, v: number) => void;
}

export interface MatrixWidget {
  el: HTMLElement;
  set(M: number[][]): void;
  setEditable(on: boolean): void;
  setHighlight(j: number | null): void;
  setLabel(label: Node | string): void;
  readonly editable: boolean;
}

export function matrixWidget(parent: HTMLElement, opts: MatrixWidgetOpts = {}): MatrixWidget {
  const rows = opts.rows ?? 3;
  const cols = opts.cols ?? 3;
  const digits = opts.digits ?? 3;
  const labelEl = el('span', { class: 'mr-mlabel' });
  const grid = el('div', { class: 'mr-grid' });
  grid.style.gridTemplateColumns = `repeat(${cols}, auto)`;
  const wrap = el('div', { class: 'mr-matrix c301-mat' }, labelEl, grid);
  parent.appendChild(wrap);
  let editable = false;
  let values: number[][] = Array.from({ length: rows }, () => new Array(cols).fill(0));
  const cells: HTMLSpanElement[][] = [];
  const inputs: HTMLInputElement[][] = [];

  for (let i = 0; i < rows; i++) {
    cells.push([]);
    inputs.push([]);
    for (let j = 0; j < cols; j++) {
      const tone = opts.colTones?.[j];
      const cell = el('span', { class: `mr-cell${tone ? ` hl-${tone}` : ''}`, dataset: { col: String(j) } });
      const input = el('input', { type: 'number', step: 0.1, title: `第 ${i + 1} 列、第 ${j + 1} 欄` });
      input.addEventListener('input', () => {
        const v = Number(input.value);
        if (input.value !== '' && Number.isFinite(v)) opts.onEdit?.(i, j, v);
      });
      // on commit (blur / Enter) show the value actually in use: the owner may clamp it, and an
      // emptied or invalid field falls back to the last accepted number
      input.addEventListener('change', () => paint());
      cells[i].push(cell);
      inputs[i].push(input);
      grid.appendChild(cell);
    }
  }

  const paint = () => {
    for (let i = 0; i < rows; i++) for (let j = 0; j < cols; j++) {
      const cell = cells[i][j];
      const t = fmt(values[i][j], digits);
      if (editable) {
        const inp = inputs[i][j];
        if (cell.firstChild !== inp) cell.replaceChildren(inp);
        if (document.activeElement !== inp && inp.value !== t) inp.value = t;
      } else if (cell.firstChild !== null && cell.firstChild.nodeType !== Node.TEXT_NODE) {
        cell.replaceChildren(t); // leaving edit mode: replace the <input> by plain text
      } else if (cell.textContent !== t) {
        cell.textContent = t; // called every animation frame: write only what changed
      }
    }
  };

  const colOf = (t: EventTarget | null): number | null => {
    const c = (t as HTMLElement | null)?.closest?.('[data-col]') as HTMLElement | null;
    return c ? Number(c.dataset.col) : null;
  };
  grid.addEventListener('pointerover', (ev) => { const j = colOf(ev.target); if (j != null) opts.onHoverCol?.(j); });
  grid.addEventListener('pointerleave', () => opts.onHoverCol?.(null));
  grid.addEventListener('click', (ev) => {
    if (editable) return;
    const j = colOf(ev.target);
    if (j != null) opts.onClickCol?.(j);
  });

  const handle: MatrixWidget = {
    el: wrap,
    get editable() { return editable; },
    set(M) { values = M.map((r) => r.slice()); paint(); },
    setEditable(on) {
      editable = on;
      wrap.classList.toggle('c301-editing', on);
      if (!on) for (const r of inputs) for (const inp of r) inp.blur();
      paint();
    },
    setHighlight(j) {
      for (let i = 0; i < rows; i++) for (let k = 0; k < cols; k++) cells[i][k].classList.toggle('c301-hot', j === k);
    },
    setLabel(label) { labelEl.replaceChildren(label); },
  };
  if (opts.label != null) handle.setLabel(opts.label);
  return handle;
}
