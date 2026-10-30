import type { LessonMeta } from '@/types';

export const meta: LessonMeta = {
  id: 'ch10-03-sampling-rrt-prm',
  chapter: 10,
  order: 3,
  title: '取樣式規劃：RRT 與 PRM（sampling-based planning）',
  bookRef: '§10.5',
  summary: '逐步執行 RRT 的 sample–nearest–steer–connect 迴圈，理解 Voronoi 偏置讓樹快速探索空曠區，比較單查詢（RRT）與多查詢（PRM）的適用場景與窄通道問題。',
  estMinutes: 30,
  tags: ['RRT', 'PRM', 'probabilistic completeness'],
};
