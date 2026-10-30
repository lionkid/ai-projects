/**
 * In-place matrix display (local extension of the toolkit's `matrixView`).
 *
 * `matrixView.set` rebuilds every cell on each call and only knows the x/y/z/hl highlight
 * classes. This lesson updates its Jacobians every animation frame and colours each column
 * with its joint colour, so this widget keeps the cell elements between updates (text is only
 * touched when it changes), supports per-column / per-row colours, column emphasis, a
 * "changed" marker and optional row labels. It reuses the toolkit CSS classes.
 */
import { el, fmt } from '@/ui';

export interface LiveMatrixOpts {
  label?: Node | string;
  digits?: number;
  rowLabels?: string[] | null;
  title?: string;
}

export interface LiveMatrixHandle {
  el: HTMLElement;
  /** Nested rows, or a flat vector rendered as a column. */
  set(M: number[][] | number[]): void;
  setRowLabels(labels: string[] | null): void;
  /** Text colour per column (any CSS colour, e.g. `var(--series-4)`); null keeps the default. */
  colorColumns(colors: (string | null)[]): void;
  /** Text colour per row (used for column vectors such as θ̇). */
  colorRows(colors: (string | null)[]): void;
  /** Bold text on a soft accent background for the flagged columns. */
  emphasizeColumns(flags: boolean[]): void;
  /** Soft warning background for the flagged columns (e.g. "changed by the last drag"). */
  markColumns(flags: boolean[]): void;
}

export function liveMatrix(parent: HTMLElement, opts: LiveMatrixOpts = {}): LiveMatrixHandle {
  const digits = opts.digits ?? 2;
  const labelEl = el('span', { class: 'mr-mlabel' });
  if (opts.label != null) labelEl.append(opts.label);
  // Row labels sit outside the brackets; their padding/gap mirror `.mr-grid` so rows line up.
  const rowLabelEl = el('div', {
    style: { display: 'none', gap: '0.1rem 0', padding: '0.2rem 0', color: 'var(--fg-muted)', textAlign: 'right' },
  });
  const grid = el('div', { class: 'mr-grid', style: { columnGap: '0.45rem' } });
  const wrap = el('div', { class: 'mr-matrix', title: opts.title, style: { marginRight: '0.25rem' } }, labelEl, rowLabelEl, grid);
  parent.appendChild(wrap);

  let rows = 0;
  let cols = 0;
  let cells: HTMLSpanElement[][] = [];
  let colColors: (string | null)[] = [];
  let rowColors: (string | null)[] = [];
  let emph: boolean[] = [];
  let marks: boolean[] = [];
  let rowLabels: string[] | null = opts.rowLabels ?? null;

  const paintRowLabels = () => {
    rowLabelEl.replaceChildren();
    if (!rowLabels || rowLabels.length !== rows) { rowLabelEl.style.display = 'none'; return; }
    rowLabelEl.style.display = 'grid';
    // same font size as the cells (inherited from .mr-matrix) so the rows line up
    for (const l of rowLabels) rowLabelEl.appendChild(el('span', {}, l));
  };

  const paintStyles = () => {
    for (let i = 0; i < rows; i++) {
      for (let j = 0; j < cols; j++) {
        const c = cells[i][j];
        c.style.color = colColors[j] ?? rowColors[i] ?? '';
        c.style.fontWeight = emph[j] ? '700' : '';
        c.style.background = marks[j] ? 'var(--warn-soft)' : emph[j] ? 'var(--accent-soft)' : '';
        c.style.borderRadius = marks[j] || emph[j] ? '3px' : '';
        c.style.padding = '0 0.1rem';
      }
    }
  };

  const reshape = (r: number, c: number) => {
    rows = r; cols = c;
    grid.style.gridTemplateColumns = `repeat(${c}, auto)`;
    grid.replaceChildren();
    cells = [];
    for (let i = 0; i < r; i++) {
      const rowCells: HTMLSpanElement[] = [];
      for (let j = 0; j < c; j++) {
        const s = el('span', { class: 'mr-cell' }, '');
        grid.appendChild(s);
        rowCells.push(s);
      }
      cells.push(rowCells);
    }
    paintRowLabels();
    paintStyles();
  };

  return {
    el: wrap,
    set(M) {
      const R: number[][] = Array.isArray(M[0]) ? (M as number[][]) : (M as number[]).map((x) => [x]);
      const r = R.length;
      const c = R[0]?.length ?? 0;
      if (r !== rows || c !== cols) reshape(r, c);
      for (let i = 0; i < r; i++) {
        for (let j = 0; j < c; j++) {
          const txt = fmt(R[i][j], digits);
          if (cells[i][j].textContent !== txt) cells[i][j].textContent = txt;
        }
      }
    },
    setRowLabels(labels) { rowLabels = labels; paintRowLabels(); },
    colorColumns(colors) { colColors = colors.slice(); paintStyles(); },
    colorRows(colors) { rowColors = colors.slice(); paintStyles(); },
    emphasizeColumns(flags) {
      if (flags.length === emph.length && flags.every((f, i) => f === emph[i])) return;
      emph = flags.slice(); paintStyles();
    },
    markColumns(flags) {
      if (flags.length === marks.length && flags.every((f, i) => f === marks[i])) return;
      marks = flags.slice(); paintStyles();
    },
  };
}
