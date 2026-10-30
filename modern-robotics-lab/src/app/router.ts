/** Minimal hash router: `#/`, `#/lesson/<id>`, `#/chapter/<n>`. */
export type Route =
  | { name: 'home' }
  | { name: 'lesson'; id: string }
  | { name: 'chapter'; chapter: number }
  | { name: 'notfound'; path: string };

export function parseRoute(hash: string = location.hash): Route {
  const path = hash.replace(/^#/, '') || '/';
  if (path === '/' || path === '') return { name: 'home' };
  let m = /^\/lesson\/([\w-]+)\/?$/.exec(path);
  if (m) return { name: 'lesson', id: m[1] };
  m = /^\/chapter\/(\d+)\/?$/.exec(path);
  if (m) return { name: 'chapter', chapter: Number(m[1]) };
  return { name: 'notfound', path };
}

export function lessonHref(id: string): string { return `#/lesson/${id}`; }
export function chapterHref(n: number): string { return `#/chapter/${n}`; }

export function onRoute(cb: (r: Route) => void): () => void {
  const handler = () => cb(parseRoute());
  window.addEventListener('hashchange', handler);
  handler();
  return () => window.removeEventListener('hashchange', handler);
}

export function navigate(hash: string): void {
  if (location.hash === hash) window.dispatchEvent(new HashChangeEvent('hashchange'));
  else location.hash = hash;
}
