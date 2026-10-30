import type { LessonMeta } from '@/types';

export const meta: LessonMeta = {
  id: 'ch09-01-time-scaling',
  chapter: 9,
  order: 1,
  title: '路徑 vs 軌跡：時間縮放 s(t)（time scaling）',
  bookRef: '§9.1, 9.2.2',
  summary: '把軌跡拆成幾何路徑 θ(s) 與時間縮放 s(t)，比較三次、五次、梯形與 S 曲線的 ṡ、s̈ 特性，並依速度／加速度上限選擇合適的縮放。',
  estMinutes: 25,
  tags: ['time scaling', 'cubic', 'quintic', 'trapezoid'],
};
