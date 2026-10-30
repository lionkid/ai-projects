/** Learner progress persisted in localStorage. */
interface ProgressState {
  completed: Record<string, true>;
  step: Record<string, number>;
  quiz: Record<string, { correct: number; total: number }>;
  lastVisited?: string;
}

const KEY = 'mrlab.progress.v1';
const listeners = new Set<() => void>();
let state: ProgressState = load();

function load(): ProgressState {
  try {
    const raw = localStorage.getItem(KEY);
    if (raw) {
      const p = JSON.parse(raw) as Partial<ProgressState>;
      return { completed: p.completed ?? {}, step: p.step ?? {}, quiz: p.quiz ?? {}, lastVisited: p.lastVisited };
    }
  } catch { /* ignore */ }
  return { completed: {}, step: {}, quiz: {} };
}

function save(): void {
  try { localStorage.setItem(KEY, JSON.stringify(state)); } catch { /* ignore */ }
  for (const cb of listeners) cb();
}

export const progress = {
  isCompleted(id: string): boolean { return !!state.completed[id]; },
  setCompleted(id: string, v: boolean): void { if (v) state.completed[id] = true; else delete state.completed[id]; save(); },
  getStep(id: string): number { return state.step[id] ?? 0; },
  setStep(id: string, i: number): void { state.step[id] = i; save(); },
  recordQuiz(id: string, correct: number, total: number): void { state.quiz[id] = { correct, total }; save(); },
  getQuiz(id: string): { correct: number; total: number } | undefined { return state.quiz[id]; },
  setLastVisited(id: string): void { state.lastVisited = id; save(); },
  get lastVisited(): string | undefined { return state.lastVisited; },
  completedCount(ids: string[]): number { return ids.filter((i) => state.completed[i]).length; },
  reset(): void { state = { completed: {}, step: {}, quiz: {} }; save(); },
  subscribe(cb: () => void): () => void { listeners.add(cb); return () => { listeners.delete(cb); }; },
};
