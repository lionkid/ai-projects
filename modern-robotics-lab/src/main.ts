import 'katex/dist/katex.min.css';
import './styles.css';
import { el } from '@/ui/dom';
import { initTheme } from './app/theme';
import { mountShell } from './app/shell';
import { onRoute } from './app/router';
import { renderHome } from './app/home';
import { renderLesson, type LessonPage } from './app/lessonView';
import { getLesson, lessons, lessonsByChapter } from './app/registry';
import { chapterInfo } from './content/chapters';
import { lessonHref } from './app/router';

initTheme();
const root = document.getElementById('app')!;
const shell = mountShell(root);

let current: LessonPage | null = null;
let token = { cancelled: false };

onRoute(async (route) => {
  token.cancelled = true;
  token = { cancelled: false };
  current?.destroy();
  current = null;
  shell.closeSidebar();
  window.scrollTo({ top: 0 });
  if (route.name === 'home') {
    shell.setActiveLesson(null);
    shell.setCrumbs(['課程地圖']);
    renderHome(shell.outlet);
    document.title = 'Modern Robotics Lab｜互動式機器人學實驗室';
    return;
  }
  if (route.name === 'chapter') {
    const list = lessonsByChapter(route.chapter);
    const target = list[0];
    if (target) { location.replace(lessonHref(target.dir)); return; }
  }
  if (route.name === 'lesson') {
    const entry = getLesson(route.id);
    if (entry) {
      const ch = chapterInfo(entry.meta.chapter);
      shell.setActiveLesson(entry.dir);
      shell.setCrumbs([el('a', { href: '#/' }, '課程地圖'), `Ch ${ch.number} ${ch.title}`, el('b', {}, entry.meta.title)]);
      document.title = `${entry.meta.title}｜Modern Robotics Lab`;
      const myToken = token;
      const page = await renderLesson(shell.outlet, entry, myToken);
      if (myToken.cancelled) { page?.destroy(); return; }
      current = page;
      return;
    }
  }
  shell.setActiveLesson(null);
  shell.setCrumbs(['找不到頁面']);
  shell.outlet.replaceChildren(el('div', { class: 'notfound' }, el('h2', {}, '找不到這個頁面'), el('p', {}, el('a', { href: '#/' }, '回到課程地圖'))));
});

// Exposed for smoke tests and debugging.
(window as unknown as { __MRLAB__: unknown }).__MRLAB__ = {
  lessons: lessons.map((l) => ({ id: l.dir, chapter: l.meta.chapter, order: l.meta.order, title: l.meta.title })),
};
