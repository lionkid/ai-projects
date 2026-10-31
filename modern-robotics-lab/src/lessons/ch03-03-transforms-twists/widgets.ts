/**
 * Small DOM / three.js helpers local to this lesson.
 *  - cellMatrix: like the toolkit matrixView, but every cell can get its own CSS class
 *    (needed to colour the four 3×3 blocks of a 6×6 Adjoint, or a single column of T).
 *  - eqRow / tex: a wrapping row of matrices and KaTeX operators ("[r_s;1] = T_sb [r_b;1]").
 *  - label3D / disposeSprites: theme-aware text sprites and texture clean-up.
 */
import * as THREE from 'three';
import { el, fmt, makeTextSprite, mathEl, type Scene3D } from '@/ui';

export interface CellMatrix {
  el: HTMLElement;
  set(M: number[][] | number[], digits?: number): void;
  /** Replace the class function (applied on the next set). */
  setClass(fn: ((i: number, j: number) => string) | null): void;
  setLabel(tex: string): void;
}

export function cellMatrix(
  parent: HTMLElement,
  o: { label?: string; digits?: number; cls?: (i: number, j: number) => string; title?: string } = {},
): CellMatrix {
  const labelEl = el('span', { class: 'mr-mlabel' });
  if (o.label) labelEl.appendChild(mathEl(o.label));
  const grid = el('div', { class: 'mr-grid' });
  const wrap = el('div', { class: 'mr-matrix', title: o.title }, labelEl, grid);
  parent.appendChild(wrap);
  let cells: HTMLSpanElement[] = [];
  let shape = '';
  let cls = o.cls ?? null;
  return {
    el: wrap,
    setClass(fn) { cls = fn; },
    setLabel(tex) { labelEl.replaceChildren(mathEl(tex)); },
    set(M, digits) {
      const rows: number[][] = Array.isArray(M[0]) ? (M as number[][]) : (M as number[]).map((x) => [x]);
      const nc = rows[0]?.length ?? 1;
      const key = `${rows.length}x${nc}`;
      if (key !== shape) {
        shape = key;
        grid.style.gridTemplateColumns = `repeat(${nc}, auto)`;
        cells = Array.from({ length: rows.length * nc }, () => el('span', { class: 'mr-cell' }));
        grid.replaceChildren(...cells);
      }
      const d = digits ?? o.digits ?? 2;
      rows.forEach((r, i) => r.forEach((x, j) => {
        const cell = cells[i * nc + j];
        const cn = `mr-cell ${cls ? cls(i, j) : ''}`.trim();
        if (cell.className !== cn) cell.className = cn;
        const t = fmt(x, d);
        if (cell.textContent !== t) cell.textContent = t;
      }));
    },
  };
}

export interface Captioned extends CellMatrix {
  box: HTMLElement;
  /** New caption: TeX part plus an optional plain-text note (CJK stays outside KaTeX). */
  setCaption(texPart: string, note?: string): void;
}

/** A cellMatrix with a small caption above it instead of a label on its left (keeps equation rows narrow). */
export function captioned(
  parent: HTMLElement,
  texPart: string,
  o: { note?: string; digits?: number; cls?: (i: number, j: number) => string } = {},
): Captioned {
  const cap = el('span', { class: 'c303-capt' });
  const box = el('span', { class: 'c303-cap' }, cap);
  parent.appendChild(box);
  const m = cellMatrix(box, { digits: o.digits, cls: o.cls });
  let key = '';
  const setCaption = (t: string, note?: string) => {
    const k = `${t}|${note ?? ''}`;
    if (k === key) return;
    key = k;
    cap.replaceChildren(mathEl(t), note ? el('span', {}, note) : '');
  };
  setCaption(texPart, o.note);
  return { ...m, box, setCaption };
}

/** A wrapping row for "matrix = matrix · matrix" style equations. */
export function eqRow(parent: HTMLElement, cls = ''): HTMLDivElement {
  const r = el('div', { class: `c303-eq ${cls}`.trim() });
  parent.appendChild(r);
  return r;
}

/** An inline KaTeX operator / label inside an equation row. */
export function tex(parent: HTMLElement, t: string): HTMLSpanElement {
  const s = mathEl(t);
  s.classList.add('c303-op');
  parent.appendChild(s);
  return s;
}

/** A camera-facing text label whose colour follows the theme (pass a CSS variable name). */
export function label3D(scene: Scene3D, text: string, color = '--fg', scale = 0.0021): THREE.Sprite {
  const s = makeTextSprite(text, color, 40, scale);
  scene.scene.add(s);
  return s;
}

/**
 * Replace the vertices of a THREE.Line. Scene3D.addPolyline().update() writes into the existing
 * position buffer, so a polyline created with 2 points silently keeps only the first 2 points of a
 * longer path (three.js warns "Buffer size too small"); this swaps in a fresh geometry instead.
 */
export function setLinePoints(line: THREE.Line, pts: ArrayLike<number>[], dashed = false): void {
  line.geometry.dispose();
  line.geometry = new THREE.BufferGeometry().setFromPoints(Array.from(pts, (p) => new THREE.Vector3(p[0], p[1], p[2])));
  if (dashed) line.computeLineDistances();
}

/** Dispose sprite textures (Scene3D.destroy disposes materials and geometries, not sprite maps). */
export function disposeSprites(root: THREE.Object3D): void {
  root.traverse((o) => {
    const sp = o as THREE.Sprite;
    if (sp.isSprite) (sp.material as THREE.SpriteMaterial).map?.dispose();
  });
}
