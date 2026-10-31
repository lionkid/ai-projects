import type { LessonMeta } from '@/types';

export const meta: LessonMeta = {
  id: 'ch03-01-rotation-matrices',
  chapter: 3,
  order: 1,
  title: '剛體位姿與旋轉矩陣（rotation matrix, SO(3)）',
  bookRef: '§3.1, 3.2.1',
  summary: '從平面座標系暖身到三維旋轉矩陣：把 R 的三個欄位讀成 {b} 軸的方向，理解 R 的三種用途，並建立「前乘＝固定軸、後乘＝物體軸」的規則。',
  estMinutes: 35,
  tags: ['SO(3)', 'frames', 'pre/post-multiply'],
};
