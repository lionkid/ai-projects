import type { LessonMeta } from '@/types';

export const meta: LessonMeta = {
  id: 'ch11-01-error-dynamics-pid',
  chapter: 11,
  order: 1,
  title: '誤差動態與單關節 PID（error dynamics, PID）',
  bookRef: '§11.1–11.3.1, 11.4.1',
  summary: '把閉迴路行為寫成誤差的線性微分方程，用阻尼比與自然頻率預測超調與安定時間，觀察 I 項消除穩態誤差的代價與積分飽和。',
  estMinutes: 35,
  tags: ['error dynamics', 'PID', 'anti-windup'],
};
