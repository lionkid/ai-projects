import type { LessonMeta } from '@/types';

export const meta: LessonMeta = {
  id: 'ch05-01-jacobian-geometry',
  chapter: 5,
  order: 1,
  title: '空間與物體 Jacobian 的幾何意義（space / body Jacobian）',
  bookRef: '§5.1.1–5.1.4',
  summary: '把 Jacobian 的每一欄讀成「該關節在目前構型的螺旋軸」，看懂末端 twist 是各欄乘關節速度的疊加，並用 Adjoint 連結 J_s 與 J_b。',
  estMinutes: 35,
  tags: ['Jacobian', 'JacobianSpace', 'JacobianBody'],
};
