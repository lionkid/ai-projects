/**
 * A small static "gallery card" for the last step of ch07-01: an original sketch of a 6-leg
 * Stewart–Gough platform (orthographic view drawn as inline SVG) next to a short caption.
 * Colours are CSS variables in `style` attributes, so a theme switch needs no redraw.
 */
import { el, svgEl, mdElement } from '@/ui';

type V3 = [number, number, number];

const DEG = Math.PI / 180;
/** Base joints: three pairs on a circle (radius 1) around 90°, 210°, 330°. */
const BASE: V3[] = [90, 210, 330].flatMap((c) => [c - 12, c + 12]).map((a) => [Math.cos(a * DEG), Math.sin(a * DEG), 0]);
/** Platform joints: three pairs on a smaller circle at height 0.85, rotated by 60° w.r.t. the base pairs. */
const PLAT: V3[] = [30, 150, 270].flatMap((c) => [c - 15, c + 15]).map((a) => [0.55 * Math.cos(a * DEG), 0.55 * Math.sin(a * DEG), 0.85]);
/** Leg i joins base joint i to the nearest platform joint (the usual zig-zag 6-6 layout). */
const LEGS: [number, number][] = [[0, 1], [1, 2], [2, 3], [3, 4], [4, 5], [5, 0]];

/** Orthographic view: turn about z by `az`, then look down by `el` (screen y grows downwards). */
function project(p: V3, az = -28 * DEG, elev = 24 * DEG): [number, number] {
  const x = p[0] * Math.cos(az) - p[1] * Math.sin(az);
  const y = p[0] * Math.sin(az) + p[1] * Math.cos(az);
  return [x, -(y * Math.sin(elev) + p[2] * Math.cos(elev))];
}

function sketch(): SVGSVGElement {
  const pts2 = (ps: V3[]) => ps.map((p) => project(p));
  const base = pts2(BASE);
  const plat = pts2(PLAT);
  const all = [...base, ...plat];
  const xs = all.map((p) => p[0]);
  const ys = all.map((p) => p[1]);
  const pad = 0.12;
  const x0 = Math.min(...xs) - pad;
  const y0 = Math.min(...ys) - pad;
  const w = Math.max(...xs) - x0 + pad;
  const h = Math.max(...ys) - y0 + pad;
  const svg = svgEl('svg', { viewBox: `${x0} ${y0} ${w} ${h}`, width: 190, height: Math.round((190 * h) / w), role: 'img' });
  svg.setAttribute('aria-label', 'Stewart–Gough 平台示意圖：六邊形底座、較小的上平台與六條可伸縮的腿');
  const poly = (ps: [number, number][], style: string) => {
    const e = svgEl('polygon', { points: ps.map((p) => `${p[0].toFixed(4)},${p[1].toFixed(4)}`).join(' ') });
    e.setAttribute('style', style);
    svg.appendChild(e);
  };
  const line = (a: [number, number], b: [number, number], style: string) => {
    const e = svgEl('line', { x1: a[0], y1: a[1], x2: b[0], y2: b[1] });
    e.setAttribute('style', style);
    svg.appendChild(e);
  };
  poly(base, 'fill: var(--bg-sunken); stroke: var(--fg-muted); stroke-width: 0.025; stroke-linejoin: round');
  // legs: a fat cylinder (lower part) and a thinner piston (upper part) = one prismatic joint each
  for (const [i, j] of LEGS) {
    const a = base[i];
    const b = plat[j];
    const m: [number, number] = [a[0] + 0.55 * (b[0] - a[0]), a[1] + 0.55 * (b[1] - a[1])];
    line(a, m, 'stroke: var(--fg-muted); stroke-width: 0.075; stroke-linecap: round');
    line(m, b, 'stroke: var(--accent); stroke-width: 0.035; stroke-linecap: round');
  }
  poly(plat, 'fill: var(--accent-soft); fill-opacity: 0.85; stroke: var(--accent); stroke-width: 0.03; stroke-linejoin: round');
  for (const p of [...base, ...plat]) {
    const c = svgEl('circle', { cx: p[0], cy: p[1], r: 0.035 });
    c.setAttribute('style', 'fill: var(--bg-elev); stroke: var(--fg); stroke-width: 0.018');
    svg.appendChild(c);
  }
  return svg;
}

/**
 * Card with the sketch and a caption (markdown + KaTeX). Returns a plain wrapper, so callers can
 * toggle `style.display` without touching the card's own flex layout.
 */
export function stewartCard(parent: HTMLElement): HTMLElement {
  const caption = mdElement(
    '**Stewart–Gough 平台**（示意圖）：6 條腿各有一個移動關節馬達，平台有 6 個自由度。' +
    'IK：$\\ell_i = \\|p + R\\,b_i - a_i\\|$，六條腿各算各的；' +
    'FK：由 6 個 $\\ell_i$ 聯立反求 $(R, p)$，是耦合的多項式系統，最多 40 組解。',
  );
  const card = el('div', {
    class: 'mr-note',
    style: {
      display: 'flex', flexWrap: 'wrap', alignItems: 'center', gap: '0.4rem 0.9rem',
      border: '1px solid var(--border)', borderRadius: 'var(--radius-sm)', padding: '0.4rem 0.6rem', margin: '0.4rem 0',
    },
  }, sketch(), el('div', { style: { flex: '1 1 220px', minWidth: '0' } }, caption));
  const wrap = el('div', {}, card);
  parent.appendChild(wrap);
  return wrap;
}
