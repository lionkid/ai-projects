/** Lesson page: step-by-step reading pane + simulation pane + quiz. */
import type { Lesson, LessonHandle, QuizQuestion } from '@/types';
import { el, clear } from '@/ui/dom';
import { mdElement, renderMarkdown } from '@/ui/markdown';
import { chapterInfo } from '@/content/chapters';
import { type LessonEntry, nextLesson, prevLesson } from './registry';
import { progress } from './progress';
import { getTheme, onThemeChange } from './theme';
import { lessonHref } from './router';

export interface LessonPage { destroy(): void }

export async function renderLesson(outlet: HTMLElement, entry: LessonEntry, token: { cancelled: boolean }): Promise<LessonPage | null> {
  outlet.replaceChildren(el('div', { class: 'loading' }, '載入課程中…'));
  let lesson: Lesson;
  try {
    lesson = await entry.load();
  } catch (e) {
    outlet.replaceChildren(el('div', { class: 'notfound' }, el('h2', {}, '課程載入失敗'), el('pre', {}, String(e))));
    return null;
  }
  if (token.cancelled) return null;
  progress.setLastVisited(entry.dir);
  const { meta } = lesson;
  const ch = chapterInfo(meta.chapter);
  const steps = lesson.steps;
  let stepIndex = Math.min(progress.getStep(entry.dir), Math.max(0, steps.length - 1));

  // ---- header
  const head = el('div', { class: 'lesson-head' },
    el('div', { class: 'eyebrow' },
      el('span', { class: 'badge' }, `Ch ${meta.chapter} · ${ch.title}`),
      el('span', { class: 'badge book', title: '對應教材章節' }, `📖 ${meta.bookRef}`),
      meta.estMinutes ? el('span', { class: 'badge' }, `約 ${meta.estMinutes} 分鐘`) : null,
      ...(meta.tags ?? []).map((t) => el('span', { class: 'badge accent' }, t))),
    el('h1', {}, meta.title),
    el('p', { class: 'summary' }, meta.summary));

  // ---- stepper
  const bar = el('div', { class: 'stepper-bar' });
  const stepTitle = el('h2');
  const stepBody = el('div');
  const prevBtn = el('button', { class: 'btn', type: 'button' }, '← 上一步');
  const nextBtn = el('button', { class: 'btn primary', type: 'button' }, '下一步 →');
  const stepper = el('div', { class: 'card stepper' }, bar, stepTitle, stepBody,
    el('div', { class: 'stepper-nav' }, prevBtn, el('span', { class: 'kbd-hint' }, '鍵盤 ← → 可切換步驟'), nextBtn));

  // ---- sim pane
  const hint = el('div', { class: 'sim-hint' });
  const host = el('div', { class: 'sim-host' });
  const simPane = el('div', { class: 'sim-pane' }, el('div', { class: 'card sim-card' }, hint, host));

  // ---- quiz
  const quizEl = lesson.quiz?.length ? renderQuiz(entry.dir, lesson.quiz) : null;

  // ---- footer
  const prev = prevLesson(entry.dir);
  const next = nextLesson(entry.dir);
  const doneBox = el('input', { type: 'checkbox' });
  doneBox.checked = progress.isCompleted(entry.dir);
  doneBox.addEventListener('change', () => progress.setCompleted(entry.dir, doneBox.checked));
  const foot = el('div', { class: 'lesson-foot' },
    prev ? el('a', { class: 'btn', href: lessonHref(prev.dir) }, `← ${prev.meta.title}`) : el('a', { class: 'btn', href: '#/' }, '← 課程地圖'),
    el('span', { class: 'spacer' }),
    el('label', { class: 'complete-toggle' }, doneBox, '標記為完成'),
    el('span', { class: 'spacer' }),
    next ? el('a', { class: 'btn primary', href: lessonHref(next.dir) }, `${next.meta.title} →`) : el('a', { class: 'btn primary', href: '#/' }, '回到課程地圖'));

  const page = el('div', { class: 'lesson-page', dataset: { lesson: entry.dir } }, head,
    el('div', { class: 'lesson-grid' }, el('div', { class: 'reading-pane' }, stepper, quizEl, foot), simPane));
  outlet.replaceChildren(page);

  // ---- mount simulation
  let handle: LessonHandle | null = null;
  let unsubTheme = () => {};
  try {
    const ctx = {
      theme: getTheme(),
      onThemeChange: (cb: (t: 'light' | 'dark') => void) => { const u = onThemeChange(cb); const prevU = unsubTheme; unsubTheme = () => { prevU(); u(); }; return u; },
      currentStep: stepIndex,
    };
    handle = lesson.mount(host, ctx);
  } catch (e) {
    console.error(e);
    host.classList.add('error');
    host.textContent = `模擬初始化失敗：${e instanceof Error ? e.stack ?? e.message : String(e)}`;
  }

  function showStep(i: number, opts: { scroll?: boolean } = {}): void {
    stepIndex = Math.max(0, Math.min(steps.length - 1, i));
    progress.setStep(entry.dir, stepIndex);
    clear(bar);
    steps.forEach((_, k) => {
      const dot = el('button', { class: `dot ${k < stepIndex ? 'done' : ''} ${k === stepIndex ? 'current' : ''}`, type: 'button', title: steps[k].title, onClick: () => showStep(k) });
      bar.appendChild(dot);
    });
    bar.appendChild(el('span', { class: 'count' }, `步驟 ${stepIndex + 1} / ${steps.length}`));
    const s = steps[stepIndex];
    stepTitle.textContent = s.title;
    stepBody.replaceChildren(mdElement(s.md));
    hint.innerHTML = s.simHint ? renderMarkdown(s.simHint).replace(/^<p>|<\/p>\s*$/g, '') : '';
    prevBtn.disabled = stepIndex === 0;
    nextBtn.textContent = stepIndex === steps.length - 1 ? (quizEl ? '前往自我檢測 ↓' : '完成 ✓') : '下一步 →';
    try { handle?.onStep?.(stepIndex); } catch (e) { console.error(e); }
    if (opts.scroll) stepper.scrollIntoView({ behavior: 'smooth', block: 'start' });
  }
  prevBtn.addEventListener('click', () => showStep(stepIndex - 1, { scroll: true }));
  nextBtn.addEventListener('click', () => {
    if (stepIndex === steps.length - 1) {
      if (quizEl) quizEl.scrollIntoView({ behavior: 'smooth', block: 'start' });
      else { progress.setCompleted(entry.dir, true); doneBox.checked = true; }
    } else showStep(stepIndex + 1, { scroll: true });
  });
  const onKey = (ev: KeyboardEvent) => {
    const t = ev.target as HTMLElement | null;
    if (t && (t.tagName === 'INPUT' || t.tagName === 'SELECT' || t.tagName === 'TEXTAREA' || t.isContentEditable)) return;
    if (ev.key === 'ArrowRight') { showStep(stepIndex + 1); ev.preventDefault(); }
    else if (ev.key === 'ArrowLeft') { showStep(stepIndex - 1); ev.preventDefault(); }
  };
  window.addEventListener('keydown', onKey);
  showStep(stepIndex);
  page.dataset.lessonReady = entry.dir;

  return {
    destroy() {
      window.removeEventListener('keydown', onKey);
      unsubTheme();
      try { handle?.destroy(); } catch (e) { console.error(e); }
      handle = null;
    },
  };
}

function renderQuiz(lessonId: string, questions: QuizQuestion[]): HTMLElement {
  const answered: (boolean | null)[] = questions.map(() => null);
  const score = el('div', { class: 'quiz-score' });
  const update = () => {
    const done = answered.filter((a) => a !== null).length;
    const correct = answered.filter((a) => a === true).length;
    score.textContent = done ? `目前：${correct} / ${questions.length} 題正確${done < questions.length ? `（已作答 ${done} 題）` : ''}` : '';
    if (done === questions.length) progress.recordQuiz(lessonId, correct, questions.length);
  };
  const letters = 'ABCDE';
  const qs = questions.map((q, qi) => {
    const explain = el('div', { class: 'quiz-explain', style: { display: 'none' } });
    const opts = el('div', { class: 'quiz-opts' });
    const buttons: HTMLButtonElement[] = q.options.map((o, oi) => {
      const b = el('button', { class: 'quiz-opt', type: 'button' }, el('span', { class: 'letter' }, letters[oi]), el('span', { html: renderMarkdown(o).replace(/^<p>|<\/p>\s*$/g, '') }));
      b.addEventListener('click', () => {
        if (answered[qi] !== null) return;
        const ok = oi === q.answer;
        answered[qi] = ok;
        buttons.forEach((bb, k) => { bb.disabled = true; if (k === q.answer) bb.classList.add('correct'); else if (k === oi) bb.classList.add('wrong'); });
        explain.style.display = '';
        explain.className = `quiz-explain ${ok ? 'ok' : 'bad'}`;
        explain.innerHTML = `<b>${ok ? '答對了！' : `不對，正確答案是 ${letters[q.answer]}。`}</b> ${renderMarkdown(q.explanation)}`;
        update();
      });
      return b;
    });
    buttons.forEach((b) => opts.appendChild(b));
    return el('div', { class: 'quiz-q' },
      el('div', { class: 'md' }, el('span', { class: 'qnum' }, `Q${qi + 1}.`), el('span', { html: renderMarkdown(q.question).replace(/^<p>|<\/p>\s*$/g, '') })),
      opts, explain);
  });
  const reset = el('button', { class: 'btn small ghost', type: 'button', onClick: () => { wrap.replaceWith(renderQuiz(lessonId, questions)); } }, '重新作答');
  const wrap = el('section', { class: 'card quiz' }, el('h2', {}, '自我檢測'), ...qs, score, el('div', { style: { marginTop: '0.6rem' } }, reset));
  return wrap;
}
