/** Lesson discovery via Vite glob imports: no central list to maintain. */
import type { Lesson, LessonMeta } from '@/types';

const metaModules = import.meta.glob<LessonMeta>('../lessons/*/meta.ts', { eager: true, import: 'meta' });
const lessonLoaders = import.meta.glob<Lesson>('../lessons/*/index.ts', { import: 'default' });

export interface LessonEntry {
  meta: LessonMeta;
  dir: string;
  /** False while a lesson only has meta.ts (work in progress). */
  hasIndex: boolean;
  load(): Promise<Lesson>;
}

function dirOf(path: string): string {
  const m = /lessons\/([^/]+)\//.exec(path);
  return m ? m[1] : path;
}

const loaderByDir = new Map<string, () => Promise<Lesson>>();
for (const [path, loader] of Object.entries(lessonLoaders)) loaderByDir.set(dirOf(path), loader);

export const lessons: LessonEntry[] = Object.entries(metaModules)
  .map(([path, meta]) => {
    const dir = dirOf(path);
    if (meta.id !== dir) console.warn(`[mrlab] lesson meta.id "${meta.id}" differs from directory "${dir}"; using directory name.`);
    const loader = loaderByDir.get(dir);
    if (!loader) console.warn(`[mrlab] lesson "${dir}" has meta.ts but no index.ts`);
    return {
      meta: { ...meta, id: dir },
      dir,
      hasIndex: !!loader,
      load: loader ?? (() => Promise.reject(new Error(`lesson ${dir} has no index.ts`))),
    };
  })
  .sort((a, b) => a.meta.chapter - b.meta.chapter || a.meta.order - b.meta.order || a.dir.localeCompare(b.dir));

export function getLesson(id: string): LessonEntry | undefined { return lessons.find((l) => l.dir === id); }
export function lessonsByChapter(ch: number): LessonEntry[] { return lessons.filter((l) => l.meta.chapter === ch); }
export function nextLesson(id: string): LessonEntry | undefined { const i = lessons.findIndex((l) => l.dir === id); return i >= 0 ? lessons[i + 1] : undefined; }
export function prevLesson(id: string): LessonEntry | undefined { const i = lessons.findIndex((l) => l.dir === id); return i > 0 ? lessons[i - 1] : undefined; }
export const chapterNumbers: number[] = [...new Set(lessons.map((l) => l.meta.chapter))].sort((a, b) => a - b);
