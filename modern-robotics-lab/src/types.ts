/**
 * Core contracts shared by the app shell, the UI toolkit and every lesson.
 *
 * A lesson lives in its own directory under `src/lessons/<id>/` and consists of:
 *   - `meta.ts`  : exports `meta: LessonMeta` (light, loaded eagerly for navigation)
 *   - `index.ts` : default-exports a `Lesson` (heavy, loaded lazily when opened)
 * The registry discovers lessons automatically via `import.meta.glob`, so adding a
 * lesson never requires editing a shared file.
 */

export type ThemeName = 'light' | 'dark';

export interface LessonMeta {
  /** Must equal the directory name, e.g. `ch03-02-exponential-coordinates`. */
  id: string;
  /** Modern Robotics (v2) chapter number this lesson belongs to (2..13). */
  chapter: number;
  /** 1-based ordering inside the chapter. */
  order: number;
  /** Title shown in navigation (zh-TW; keep key English terms). */
  title: string;
  /** Book sections covered, e.g. `§3.2.3` or `§4.1.1–4.1.2`. */
  bookRef: string;
  /** One or two sentences describing what the learner will be able to do. */
  summary: string;
  /** Rough time budget in minutes. */
  estMinutes?: number;
  /** Free-form tags such as 'SO(3)', 'Jacobian'. */
  tags?: string[];
}

export interface QuizQuestion {
  /** Markdown + LaTeX ($...$ inline, $$...$$ display). */
  question: string;
  /** Markdown; 2–5 options. */
  options: string[];
  /** Index into `options`. */
  answer: number;
  /** Markdown shown after the learner answers. */
  explanation: string;
}

export interface LessonStep {
  /** Short step title, e.g. `旋轉矩陣的三個欄位`. */
  title: string;
  /** Markdown + LaTeX. Original wording only – never copy the book's text. */
  md: string;
  /**
   * Optional one-line instruction displayed above the simulation while this step
   * is active, e.g. `試著把 θ 拉到 π，觀察矩陣對數的行為`.
   */
  simHint?: string;
}

export interface LessonContext {
  /** Current theme at mount time. */
  theme: ThemeName;
  /** Subscribe to theme changes; returns an unsubscribe function. */
  onThemeChange(cb: (theme: ThemeName) => void): () => void;
  /** Index of the step the learner is currently reading (0-based). */
  currentStep: number;
}

export interface LessonHandle {
  /** Tear down timers, animation loops, WebGL contexts, observers, listeners. */
  destroy(): void;
  /** Called whenever the learner navigates to a different step (0-based). */
  onStep?(index: number): void;
}

export interface Lesson {
  meta: LessonMeta;
  /** Step-by-step narrative shown in the reading pane (3–8 steps recommended). */
  steps: LessonStep[];
  /** Optional self-check questions shown after the steps. */
  quiz?: QuizQuestion[];
  /**
   * Build the interactive simulation inside `host` (an empty block element that
   * fills the simulation pane). Must be synchronous and idempotent per call.
   */
  mount(host: HTMLElement, ctx: LessonContext): LessonHandle;
}

/** Chapter descriptor used by the course map and sidebar. */
export interface ChapterInfo {
  number: number;
  /** zh-TW chapter title. */
  title: string;
  /** Original English chapter title from the book, for cross-reference. */
  titleEn: string;
  /** One-sentence description. */
  blurb: string;
}
