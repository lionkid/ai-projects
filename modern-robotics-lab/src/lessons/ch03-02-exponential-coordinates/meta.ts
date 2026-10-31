import type { LessonMeta } from '@/types';

export const meta: LessonMeta = {
  id: 'ch03-02-exponential-coordinates',
  chapter: 3,
  order: 2,
  title: '角速度與旋轉的指數座標（exponential coordinates, Rodrigues）',
  bookRef: '§3.2.2–3.2.3',
  summary: '把旋轉看成「繞某軸轉某角」：由角速度導出 so(3)，用 Rodrigues 公式計算 exp，再用矩陣對數反推軸與角，並處理 θ=0、π 的特殊情況。',
  estMinutes: 35,
  tags: ['angular velocity', 'exp/log', 'Rodrigues'],
};
