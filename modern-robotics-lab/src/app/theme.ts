import type { ThemeName } from '@/types';

const KEY = 'mrlab.theme';
const listeners = new Set<(t: ThemeName) => void>();
let current: ThemeName = 'light';

export function initTheme(): void {
  let saved: string | null = null;
  try { saved = localStorage.getItem(KEY); } catch { /* ignore */ }
  const prefersDark = window.matchMedia?.('(prefers-color-scheme: dark)').matches;
  apply(saved === 'dark' || saved === 'light' ? saved : prefersDark ? 'dark' : 'light');
}

function apply(t: ThemeName): void {
  current = t;
  document.documentElement.dataset.theme = t;
  window.dispatchEvent(new CustomEvent('mr-theme', { detail: t }));
  for (const cb of listeners) cb(t);
}

export function getTheme(): ThemeName { return current; }

export function setTheme(t: ThemeName): void {
  try { localStorage.setItem(KEY, t); } catch { /* ignore */ }
  apply(t);
}

export function toggleTheme(): void { setTheme(current === 'dark' ? 'light' : 'dark'); }

export function onThemeChange(cb: (t: ThemeName) => void): () => void {
  listeners.add(cb);
  return () => { listeners.delete(cb); };
}
