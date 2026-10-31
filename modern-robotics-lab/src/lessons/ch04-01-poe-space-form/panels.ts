/**
 * Small DOM widgets for the PoE lessons: a table of screw axes (one coloured column per joint)
 * and a "factor strip" that typesets T(θ) = e^{[S1]θ1} ⋯ M as separate chips so each factor can
 * be marked as pending / applied / active. Colours are CSS variables (theme-safe).
 */
import { el, fmt, mathEl } from '@/ui';
import type { Vec } from '@/mr';

const ROWS = ['ωx', 'ωy', 'ωz', 'vx', 'vy', 'vz'];

export interface ScrewTableHandle {
  el: HTMLElement;
  /**
   * Show the columns `cols` (6-vectors) with headers `heads` and colours `colors` (CSS variables).
   * `marked[j]` shades column j (e.g. "changed"), `selected` outlines one column.
   */
  set(cols: Vec[], heads: string[], colors: string[], o?: { marked?: boolean[]; selected?: number; digits?: number }): void;
  setTitle(t: string): void;
}

export function screwTable(parent: HTMLElement, title: string, onPick?: (j: number) => void): ScrewTableHandle {
  const titleEl = el('div', { style: { fontSize: '0.78rem', color: 'var(--fg-muted)', marginBottom: '0.15rem' } }, title);
  const grid = el('div', { style: { display: 'grid', gap: '0.05rem 0', fontFamily: 'var(--mono)', fontSize: '0.76rem', fontVariantNumeric: 'tabular-nums' } });
  const wrap = el('div', { style: { display: 'inline-block', verticalAlign: 'top', margin: '0.2rem 0.9rem 0.2rem 0' } }, titleEl, grid);
  parent.appendChild(wrap);
  let shape = '';
  let heads: HTMLButtonElement[] = [];
  let cells: HTMLSpanElement[] = [];
  return {
    el: wrap,
    setTitle(t) { if (titleEl.textContent !== t) titleEl.textContent = t; },
    set(cols, names, colors, o = {}) {
      const n = cols.length;
      const key = `${n}`;
      if (key !== shape) {
        shape = key;
        grid.style.gridTemplateColumns = `auto repeat(${n}, minmax(3.3em, auto))`;
        const kids: HTMLElement[] = [el('span')];
        heads = Array.from({ length: n }, (_, j) => {
          const b = el('button', {
            type: 'button',
            title: onPick ? '點一下選取這個關節' : undefined,
            style: { font: 'inherit', fontWeight: '700', background: 'none', border: 'none', borderBottom: '2px solid transparent', padding: '0 0.3rem', cursor: onPick ? 'pointer' : 'default', textAlign: 'right' },
          });
          if (onPick) b.addEventListener('click', () => onPick(j));
          return b;
        });
        kids.push(...heads);
        cells = [];
        for (let r = 0; r < 6; r++) {
          kids.push(el('span', { style: { color: 'var(--fg-muted)', paddingRight: '0.25rem', borderTop: r === 3 ? '1px dashed var(--border)' : '' } }, ROWS[r]));
          for (let j = 0; j < n; j++) {
            const c = el('span', { style: { textAlign: 'right', padding: '0 0.3rem', borderTop: r === 3 ? '1px dashed var(--border)' : '' } });
            cells.push(c);
            kids.push(c);
          }
        }
        grid.replaceChildren(...kids);
      }
      const d = o.digits ?? 2;
      for (let j = 0; j < n; j++) {
        const h = heads[j];
        if (h.textContent !== names[j]) h.textContent = names[j];
        h.style.color = `var(${colors[j]})`;
        h.style.borderBottomColor = o.selected === j ? `var(${colors[j]})` : 'transparent';
        for (let r = 0; r < 6; r++) {
          const c = cells[r * n + j];
          const t = fmt(cols[j][r], d);
          if (c.textContent !== t) c.textContent = t;
          const bg = o.marked?.[j] ? 'var(--warn-soft)' : o.selected === j ? 'var(--accent-soft)' : '';
          if (c.style.background !== bg) c.style.background = bg;
        }
      }
    },
  };
}

export type FactorState = 'plain' | 'identity' | 'pending' | 'applied' | 'active';

export interface StripHandle {
  el: HTMLElement;
  /** Typeset the factors (TeX) after the prefix; rebuilds the chips. */
  build(prefix: string, factors: string[]): void;
  /** Restyle the chips without re-typesetting. */
  paint(states: FactorState[], captions: string[]): void;
}

export function factorStrip(parent: HTMLElement): StripHandle {
  const wrap = el('div', { style: { display: 'flex', flexWrap: 'wrap', alignItems: 'flex-start', gap: '0.15rem 0.2rem', margin: '0.45rem 0 0.15rem', fontSize: '0.95rem' } });
  parent.appendChild(wrap);
  let chips: { box: HTMLElement; cap: HTMLElement }[] = [];
  let last = '';
  return {
    el: wrap,
    build(prefix, factors) {
      const key = prefix + '|' + factors.join('|');
      if (key === last) return;
      last = key;
      const pre = el('span', { style: { padding: '0.2rem 0.15rem' } }, mathEl(prefix));
      chips = factors.map((tex) => {
        const cap = el('span', { style: { fontSize: '0.68rem', color: 'var(--fg-muted)', minHeight: '0.9rem', lineHeight: '0.9rem', fontFamily: 'var(--mono)' } });
        const box = el('span', {
          style: { display: 'inline-flex', flexDirection: 'column', alignItems: 'center', padding: '0.12rem 0.3rem 0', borderRadius: '6px', border: '1px solid transparent', transition: 'background 0.15s, opacity 0.15s' },
        }, mathEl(tex), cap);
        return { box, cap };
      });
      wrap.replaceChildren(pre, ...chips.map((c) => c.box));
    },
    paint(states, captions) {
      chips.forEach(({ box, cap }, i) => {
        const s = states[i] ?? 'plain';
        const st = box.style;
        st.opacity = s === 'identity' || s === 'pending' ? '0.42' : '1';
        st.background = s === 'applied' ? 'var(--accent-soft)' : s === 'active' ? 'var(--warn-soft)' : 'transparent';
        st.borderColor = s === 'applied' ? 'var(--accent)' : s === 'active' ? 'var(--warn)' : s === 'pending' ? 'var(--border)' : 'transparent';
        st.borderStyle = s === 'pending' ? 'dashed' : 'solid';
        st.color = s === 'applied' ? 'var(--accent)' : '';
        const t = captions[i] ?? '';
        if (cap.textContent !== t) cap.textContent = t;
      });
    },
  };
}

/** Circled numbers ⓪①②… for stage labels. */
export const circled = (k: number): string => '⓪①②③④⑤⑥⑦⑧⑨'[k] ?? String(k);

/** "1.2e-16"-style formatting for tiny residuals. */
export const fmtSci = (x: number): string => (x === 0 ? '0' : x.toExponential(1).replace('e-', 'e−'));

/** Format a 3-vector "(a, b, c)" with `d` decimals. */
export const fmt3 = (v: ArrayLike<number>, d = 2): string => `(${fmt(v[0], d)}, ${fmt(v[1], d)}, ${fmt(v[2], d)})`;
