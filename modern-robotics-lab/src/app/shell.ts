/** Application chrome: sidebar navigation, top bar, content outlet. */
import { el, clear } from '@/ui/dom';
import { CHAPTERS } from '@/content/chapters';
import { lessons, lessonsByChapter } from './registry';
import { progress } from './progress';
import { getTheme, toggleTheme, onThemeChange } from './theme';
import { lessonHref } from './router';

export interface Shell {
  outlet: HTMLElement;
  setCrumbs(parts: (string | HTMLElement)[]): void;
  setActiveLesson(id: string | null): void;
  closeSidebar(): void;
}

export function mountShell(root: HTMLElement): Shell {
  const sidebar = el('aside', { class: 'sidebar', id: 'sidebar' });
  const crumbs = el('div', { class: 'crumbs' });
  const themeBtn = el('button', { class: 'btn small ghost', type: 'button', title: '切換深色 / 淺色主題', onClick: () => toggleTheme() });
  const menuBtn = el('button', { class: 'btn small menu-btn', type: 'button', 'aria-label': '開啟選單', onClick: () => sidebar.classList.toggle('open') }, '☰');
  const outlet = el('main', { class: 'content', id: 'content' });
  const topbar = el('header', { class: 'topbar' }, menuBtn, crumbs, el('span', { class: 'spacer' }), themeBtn);
  const main = el('div', { class: 'main' }, topbar, outlet);
  root.replaceChildren(el('div', { class: 'shell' }, sidebar, main));

  const paintTheme = () => { themeBtn.textContent = getTheme() === 'dark' ? '☀︎ 淺色' : '☾ 深色'; };
  paintTheme();
  onThemeChange(paintTheme);

  let activeId: string | null = null;
  const openChapters = new Set<number>();

  function renderSidebar(): void {
    clear(sidebar);
    sidebar.appendChild(el('div', { class: 'sidebar-brand' },
      el('span', { class: 'brand-mark' }),
      el('a', { href: '#/' }, 'Modern Robotics Lab', el('small', {}, '互動式機器人學實驗室'))));
    const total = lessons.length;
    const done = progress.completedCount(lessons.map((l) => l.dir));
    for (const ch of CHAPTERS) {
      const list = lessonsByChapter(ch.number);
      if (!list.length) continue;
      const chDone = progress.completedCount(list.map((l) => l.dir));
      const isOpen = openChapters.has(ch.number) || list.some((l) => l.dir === activeId);
      const details = el('details', { class: 'nav-chapter', open: isOpen },
        el('summary', {},
          el('span', { class: 'chap-num' }, ch.number === 0 ? '0' : String(ch.number)),
          el('span', {}, ch.title),
          el('span', { class: 'chap-progress' }, `${chDone}/${list.length}`)),
        el('ul', { class: 'nav-lessons' }, list.map((l) => el('li', {},
          el('a', { href: lessonHref(l.dir), class: l.dir === activeId ? 'active' : '', onClick: () => sidebar.classList.remove('open') },
            el('span', { class: `tick ${progress.isCompleted(l.dir) ? '' : 'pending'}` }, progress.isCompleted(l.dir) ? '✓' : '○'),
            el('span', {}, `${l.meta.order}. ${l.meta.title}`))))));
      details.addEventListener('toggle', () => { if (details.open) openChapters.add(ch.number); else openChapters.delete(ch.number); });
      sidebar.appendChild(details);
    }
    sidebar.appendChild(el('div', { class: 'sidebar-footer' },
      el('div', {}, `總進度：${done} / ${total} 課`),
      el('div', { class: 'progress-bar' }, el('span', { style: { width: `${total ? (100 * done) / total : 0}%` } })),
      el('button', { class: 'btn small ghost', type: 'button', onClick: () => { if (confirm('確定要清除所有學習進度嗎？')) progress.reset(); } }, '重設進度')));
  }
  renderSidebar();
  progress.subscribe(renderSidebar);

  return {
    outlet,
    setCrumbs(parts) {
      clear(crumbs);
      parts.forEach((p, i) => {
        if (i) crumbs.appendChild(document.createTextNode(' › '));
        crumbs.appendChild(typeof p === 'string' ? document.createTextNode(p) : p);
      });
    },
    setActiveLesson(id) { activeId = id; renderSidebar(); },
    closeSidebar() { sidebar.classList.remove('open'); },
  };
}
