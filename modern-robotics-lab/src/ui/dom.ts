/** Tiny DOM helpers used by the shell and by lessons. */

export type Child = Node | string | number | null | undefined | false;
export type Attrs = Record<string, unknown> | null | undefined;

/** Create an element: `el('div', { class: 'x', onClick: fn }, 'text', otherNode)`. */
export function el<K extends keyof HTMLElementTagNameMap>(
  tag: K,
  attrs?: Attrs,
  ...children: (Child | Child[])[]
): HTMLElementTagNameMap[K] {
  const node = document.createElement(tag);
  applyAttrs(node, attrs);
  append(node, children);
  return node;
}

export function applyAttrs(node: HTMLElement, attrs?: Attrs): void {
  if (!attrs) return;
  for (const [k, v] of Object.entries(attrs)) {
    if (v == null || v === false) continue;
    if (k === 'class' || k === 'className') node.className = String(v);
    else if (k === 'style' && typeof v === 'object') Object.assign(node.style, v as Record<string, string>);
    else if (k === 'html') node.innerHTML = String(v);
    else if (k === 'text') node.textContent = String(v);
    else if (k === 'dataset' && typeof v === 'object') Object.assign(node.dataset, v as Record<string, string>);
    else if (k.startsWith('on') && typeof v === 'function') node.addEventListener(k.slice(2).toLowerCase(), v as EventListener);
    else if (v === true) node.setAttribute(k, '');
    else node.setAttribute(k, String(v));
  }
}

export function append(parent: Node, children: (Child | Child[])[]): void {
  for (const c of children) {
    if (Array.isArray(c)) { append(parent, c); continue; }
    if (c == null || c === false) continue;
    parent.appendChild(c instanceof Node ? c : document.createTextNode(String(c)));
  }
}

export function clear(node: Node): void {
  while (node.firstChild) node.removeChild(node.firstChild);
}

const SVG_NS = 'http://www.w3.org/2000/svg';
export function svgEl<K extends keyof SVGElementTagNameMap>(tag: K, attrs?: Record<string, string | number>): SVGElementTagNameMap[K] {
  const node = document.createElementNS(SVG_NS, tag);
  if (attrs) for (const [k, v] of Object.entries(attrs)) node.setAttribute(k, String(v));
  return node;
}

/** Fixed-point formatting that never prints "-0.000". */
export function fmt(x: number, digits = 3): string {
  if (!Number.isFinite(x)) return String(x);
  const s = x.toFixed(digits);
  return /^-0(\.0+)?$/.test(s) ? s.slice(1) : s;
}

/** Radians → degrees string, e.g. `fmtDeg(Math.PI/2)` = "90.0°". */
export function fmtDeg(rad: number, digits = 1): string {
  return `${fmt((rad * 180) / Math.PI, digits)}°`;
}

/** Format a vector as "(a, b, c)". */
export function fmtVec(v: ArrayLike<number>, digits = 3): string {
  return `(${Array.from(v, (x) => fmt(x, digits)).join(', ')})`;
}

/** Read a CSS custom property from the document root (e.g. '--accent'). */
export function cssVar(name: string, fallback = ''): string {
  const v = getComputedStyle(document.documentElement).getPropertyValue(name).trim();
  return v || fallback;
}

/**
 * Resolve a colour that may name a CSS custom property: '--accent' or 'var(--accent)'
 * (optionally 'var(--accent, #36f)') gives the current theme value; anything else is returned
 * unchanged. Canvas2D, Plot2D and torqueBars accept such strings, so colours chosen at creation
 * time stay correct after the learner switches theme.
 */
export function themeColor(c: string | undefined, fallback = ''): string {
  if (!c) return fallback;
  if (c.startsWith('--')) return cssVar(c, fallback);
  const m = /^var\(\s*(--[\w-]+)\s*(?:,\s*([^)]+))?\)$/.exec(c);
  if (m) return cssVar(m[1], m[2]?.trim() ?? fallback);
  return c;
}

/** Debounce via requestAnimationFrame: many calls per frame → one invocation. */
export function rafThrottle(fn: () => void): () => void {
  let pending = false;
  return () => {
    if (pending) return;
    pending = true;
    requestAnimationFrame(() => { pending = false; fn(); });
  };
}
