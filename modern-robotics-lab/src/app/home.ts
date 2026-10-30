/** Landing page: how to use the lab + course map. */
import { el } from '@/ui/dom';
import { CHAPTERS, BOOK } from '@/content/chapters';
import { lessons, lessonsByChapter } from './registry';
import { progress } from './progress';
import { lessonHref } from './router';

export function renderHome(outlet: HTMLElement): void {
  const first = lessons[0];
  const last = progress.lastVisited && lessons.find((l) => l.dir === progress.lastVisited);
  outlet.replaceChildren(
    el('section', { class: 'hero' },
      el('h1', {}, '用互動模擬，一步一步理解機器人控制理論'),
      el('p', {}, '這是一座搭配 ', el('i', {}, BOOK.title), '（Lynch & Park, v2）的互動式實驗室。每一課都把教材裡的一個核心概念拆成幾個小步驟，旁邊放一個你可以親手操作的模擬：拉動滑桿、拖曳關節、按下播放，親眼看見公式在動。'),
      el('div', { class: 'actions' },
        last ? el('a', { class: 'btn primary', href: lessonHref(last.dir) }, `繼續：${last.meta.title}`) : null,
        first ? el('a', { class: `btn ${last ? '' : 'primary'}`, href: lessonHref(first.dir) }, '從第一課開始') : null,
        el('a', { class: 'btn', href: BOOK.site, target: '_blank', rel: 'noopener' }, '教材官網 ↗'),
        el('a', { class: 'btn ghost', href: BOOK.pdf, target: '_blank', rel: 'noopener' }, '教材 PDF (v2) ↗'))),
    el('section', { class: 'howto' },
      card(1, '讀一小步', '左側導讀每次只講一個想法，搭配公式與直覺說明。用「下一步」或鍵盤 ← → 前進。'),
      card(2, '動手玩模擬', '右側模擬會隨著步驟切換焦點。拖曳、拉滑桿、播放，觀察數值與圖形如何同時改變。'),
      card(3, '自我檢測', '每課最後有幾題小測驗，答錯會有解釋。完成後勾選「標記完成」，進度會保存在這個瀏覽器。'),
      card(4, '對照教材', '每課標示對應的教材章節（例如 §3.2.3），建議搭配原書閱讀；本站的文字是獨立撰寫的導讀，不是教材摘錄。')),
    el('h2', {}, '課程地圖'),
    el('p', { class: 'muted' }, '章節編號與教材一致（第 1 章為導論，故從第 2 章開始）。'),
    el('div', { class: 'chapter-grid' }, CHAPTERS.map(chapterCard)),
  );
}

function card(n: number, title: string, body: string): HTMLElement {
  return el('div', { class: 'card' }, el('span', { class: 'n' }, String(n)), el('h3', { style: { margin: '0 0 0.2rem' } }, title), el('p', { class: 'muted', style: { margin: 0, fontSize: '0.9rem' } }, body));
}

function chapterCard(ch: { number: number; title: string; titleEn: string; blurb: string }): HTMLElement {
  const list = lessonsByChapter(ch.number);
  const done = progress.completedCount(list.map((l) => l.dir));
  const pct = list.length ? (100 * done) / list.length : 0;
  return el('article', { class: 'card chapter-card' },
    el('header', {}, el('span', { class: 'num' }, `Ch ${ch.number}`), el('h3', {}, ch.title), el('small', {}, ch.titleEn)),
    el('p', { class: 'blurb' }, ch.blurb),
    list.length
      ? el('ol', {}, list.map((l) => el('li', { class: progress.isCompleted(l.dir) ? 'done' : '' },
        el('a', { href: lessonHref(l.dir) }, l.meta.title), ' ', el('span', { class: 'badge book' }, l.meta.bookRef))))
      : el('p', { class: 'empty-chapter' }, '課程製作中…'),
    list.length ? el('div', { class: 'progress-bar', title: `${done}/${list.length}` }, el('span', { style: { width: `${pct}%` } })) : null,
  );
}
