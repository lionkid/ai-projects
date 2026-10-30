import { describe, it, expect } from 'vitest';
import { renderMarkdown } from './markdown';
import { niceTicks } from './plot2d';

describe('renderMarkdown', () => {
  it('renders inline and display math with KaTeX', () => {
    const html = renderMarkdown('角度 $\\theta$ 與 $$R = e^{[\\hat\\omega]\\theta}$$');
    expect(html).toContain('katex');
    expect(html).toContain('katex-display');
    expect(html).not.toContain('');
  });
  it('keeps markdown emphasis away from underscores inside math', () => {
    const html = renderMarkdown('$T_{sb} = T_{sa} T_{ab}$ 與 **粗體**');
    expect(html).toContain('<strong>粗體</strong>');
    expect(html).not.toContain('<em>');
  });
  it('supports escaped dollar signs and custom macros', () => {
    const html = renderMarkdown('價格 \\$5，而 $\\R^3$ 與 $\\SO(3)$');
    expect(html).toContain('$5');
    expect(html).toContain('katex');
  });
  it('renders lists and blockquotes', () => {
    const html = renderMarkdown('> 提示\n\n- 一\n- 二');
    expect(html).toContain('<blockquote>');
    expect(html).toContain('<li>');
  });
});

describe('niceTicks', () => {
  it('produces round tick values covering the range', () => {
    const t = niceTicks(-1.3, 2.7, 5);
    expect(t[0]).toBeGreaterThanOrEqual(-1.3);
    expect(t[t.length - 1]).toBeLessThanOrEqual(2.7 + 1e-9);
    expect(t).toContain(0);
  });
  it('handles degenerate ranges', () => {
    expect(niceTicks(1, 1)).toEqual([1]);
  });
});
