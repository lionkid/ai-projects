import { describe, expect, it } from 'vitest';
import { renderMarkdown } from '@/ui/markdown';
import { quiz, steps } from './content';

/** Every markdown string the learner reads, labelled for error messages. */
function allStrings(): [string, string][] {
  const out: [string, string][] = [];
  steps.forEach((s, i) => {
    out.push([`step ${i + 1} md`, s.md]);
    if (s.simHint) out.push([`step ${i + 1} simHint`, s.simHint]);
  });
  quiz.forEach((q, i) => {
    out.push([`quiz ${i + 1} question`, q.question]);
    q.options.forEach((o, k) => out.push([`quiz ${i + 1} option ${k + 1}`, o]));
    out.push([`quiz ${i + 1} explanation`, q.explanation]);
  });
  return out;
}

/** CJK characters outside math, a rough measure of how much prose a step has. */
const cjkCount = (md: string): number => (md.replace(/\$\$[\s\S]+?\$\$|\$[^$\n]+?\$/g, '').match(/[一-鿿]/g) ?? []).length;

describe('lesson content', () => {
  it('has 4–7 steps (each with a hint) and 2–4 quiz questions with valid answers', () => {
    expect(steps.length).toBeGreaterThanOrEqual(4);
    expect(steps.length).toBeLessThanOrEqual(7);
    for (const s of steps) expect(s.simHint, s.title).toBeTruthy();
    expect(quiz.length).toBeGreaterThanOrEqual(2);
    expect(quiz.length).toBeLessThanOrEqual(4);
    for (const q of quiz) {
      expect(q.options.length).toBeGreaterThanOrEqual(2);
      expect(q.answer).toBeGreaterThanOrEqual(0);
      expect(q.answer).toBeLessThan(q.options.length);
    }
  });

  it('keeps every step short: one idea, at most two display equations', () => {
    for (const s of steps) {
      const n = cjkCount(s.md);
      expect(n, `${s.title}: ${n} CJK characters`).toBeGreaterThanOrEqual(60);
      expect(n, `${s.title}: ${n} CJK characters`).toBeLessThanOrEqual(230);
      expect((s.md.match(/\$\$/g) ?? []).length / 2, s.title).toBeLessThanOrEqual(2);
    }
  });

  it('renders without literal markdown markers, nested bold, stray dollars or KaTeX errors', () => {
    for (const [label, md] of allStrings()) {
      const html = renderMarkdown(md);
      expect(html, label).not.toContain('**');
      expect(html, label).not.toMatch(/<strong>(?:(?!<\/strong>)[\s\S])*<strong>/);
      expect(html, label).not.toMatch(/<em>(?:(?!<\/em>)[\s\S])*<em>/);
      expect(html, label).not.toContain('katex-error');
      expect(html, label).not.toContain('$');
      // a lone asterisk in "A*" must stay literal (not start an <em>)
      if (/A\*/.test(md)) expect(html, label).toContain('A*');
    }
  });
});
