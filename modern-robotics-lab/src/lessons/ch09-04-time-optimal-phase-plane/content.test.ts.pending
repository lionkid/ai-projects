import { describe, expect, it } from 'vitest';
import { renderMarkdown } from '@/ui/markdown';
import lesson from './index';
import { meta } from './meta';

describe(`${meta.id}: lesson content`, () => {
  it('has 4–7 steps with sim hints and 2–4 well-formed quiz questions', () => {
    expect(lesson.meta.id).toBe(meta.id);
    expect(lesson.steps.length).toBeGreaterThanOrEqual(4);
    expect(lesson.steps.length).toBeLessThanOrEqual(7);
    for (const s of lesson.steps) expect((s.simHint ?? '').length).toBeGreaterThan(10);
    const quiz = lesson.quiz ?? [];
    expect(quiz.length).toBeGreaterThanOrEqual(2);
    expect(quiz.length).toBeLessThanOrEqual(4);
    for (const q of quiz) {
      expect(q.answer).toBeGreaterThanOrEqual(0);
      expect(q.answer).toBeLessThan(q.options.length);
      expect(q.explanation.length).toBeGreaterThan(30);
    }
  });

  it('renders every markdown/LaTeX string without KaTeX errors or stray dollars', () => {
    const texts: string[] = [];
    for (const s of lesson.steps) texts.push(s.md, s.simHint ?? '');
    for (const q of lesson.quiz ?? []) texts.push(q.question, q.explanation, ...q.options);
    for (const t of texts) {
      const html = renderMarkdown(t);
      expect(html, t.slice(0, 60)).not.toContain('katex-error');
      expect(html.replace(/<[^>]+>/g, ''), t.slice(0, 60)).not.toMatch(/\$/);
    }
  });
});
