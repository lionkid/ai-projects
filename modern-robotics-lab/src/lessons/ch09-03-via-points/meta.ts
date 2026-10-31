import type { LessonMeta } from '@/types';

export const meta: LessonMeta = {
  id: 'ch09-03-via-points',
  chapter: 9,
  order: 3,
  title: '經過點多項式軌跡（via points, cubic splines）',
  bookRef: '§9.3',
  summary: '用分段三次多項式通過多個經過點並保證速度連續，比較指定經過點速度的策略（零速、啟發式、自訂），並辨識過衝與不合理的中間速度。',
  estMinutes: 25,
  tags: ['via points', 'viaPointCubic', 'spline'],
};
