import type { LessonMeta } from '@/types';

export const meta: LessonMeta = {
  id: 'ch11-02-multijoint-computed-torque',
  chapter: 11,
  order: 2,
  title: '多關節：PD＋重力補償 vs 計算力矩法（computed torque）',
  bookRef: '§11.4.2–11.4.3',
  summary: '看關節耦合如何讓單關節 PID 在多關節手臂失效，用重力補償消除靜態誤差、用計算力矩法把整個系統線性化並解耦，再評估模型誤差的影響與任務空間控制的差異。',
  estMinutes: 35,
  tags: ['gravity compensation', 'computed torque', 'feedback linearisation'],
};
