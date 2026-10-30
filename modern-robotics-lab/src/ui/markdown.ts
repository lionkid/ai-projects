/** Markdown + LaTeX rendering (marked + KaTeX). */
import { marked } from 'marked';
import katex from 'katex';
import { el } from './dom';

const MACROS: Record<string, string> = {
  '\\R': '\\mathbb{R}',
  '\\SO': '\\mathrm{SO}',
  '\\SE': '\\mathrm{SE}',
  '\\so': '\\mathfrak{so}',
  '\\se': '\\mathfrak{se}',
  '\\Ad': '\\mathrm{Ad}',
  '\\ad': '\\mathrm{ad}',
  '\\T': '\\mathsf{T}',
  '\\skew': '[#1]',
  '\\hatw': '\\hat{\\omega}',
  '\\vV': '\\mathcal{V}',
  '\\vF': '\\mathcal{F}',
  '\\vS': '\\mathcal{S}',
  '\\vB': '\\mathcal{B}',
  '\\vG': '\\mathcal{G}',
  '\\dd': '\\mathrm{d}',
  '\\tr': '\\operatorname{tr}',
  '\\diag': '\\operatorname{diag}',
  '\\atantwo': '\\operatorname{atan2}',
  '\\sgn': '\\operatorname{sgn}',
};

marked.setOptions({ gfm: true, breaks: false });

/** Render one TeX string to HTML. Errors are shown inline instead of thrown. */
export function renderMath(tex: string, display = false): string {
  return katex.renderToString(tex, {
    displayMode: display,
    throwOnError: false,
    strict: 'ignore',
    trust: false,
    macros: { ...MACROS },
  });
}

const PH_OPEN = '';
const PH_CLOSE = '';

/**
 * Convert markdown (with $...$ / $$...$$ math) to HTML. Math is rendered first and
 * protected by private-use placeholders so markdown emphasis rules never touch it.
 */
export function renderMarkdown(md: string): string {
  const stash: string[] = [];
  const keep = (html: string) => { stash.push(html); return `${PH_OPEN}${stash.length - 1}${PH_CLOSE}`; };
  let src = md.replace(/\\\$/g, ''); // escaped dollar
  // Display math: $$ ... $$ (may span lines)
  src = src.replace(/\$\$([\s\S]+?)\$\$/g, (_m, tex: string) => keep(renderMath(tex.trim(), true)));
  // Inline math: $ ... $ on a single line, not followed/preceded by digits like "$5"
  src = src.replace(/\$([^$\n]+?)\$/g, (_m, tex: string) => keep(renderMath(tex.trim(), false)));
  let html = marked.parse(src, { async: false }) as string;
  html = html.replace(new RegExp(`${PH_OPEN}(\\d+)${PH_CLOSE}`, 'g'), (_m, i: string) => stash[Number(i)]);
  return html.replace(//g, '$');
}

/** Render markdown into a `<div class="md">`. */
export function mdElement(md: string, extraClass = ''): HTMLDivElement {
  return el('div', { class: `md ${extraClass}`.trim(), html: renderMarkdown(md) });
}

/** Inline math element for use inside sim panels (labels, readouts). */
export function mathEl(tex: string, display = false): HTMLSpanElement {
  return el('span', { class: 'mr-math', html: renderMath(tex, display) });
}
