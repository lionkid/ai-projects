import type { LessonMeta } from '@/types';

export const meta: LessonMeta = {
  id: 'ch03-03-transforms-twists',
  chapter: 3,
  order: 3,
  title: '齊次變換、旋量（twist）與 Adjoint',
  bookRef: '§3.3.1–3.3.2',
  summary: '把 R 與 p 裝進 4×4 的 T 來合成、求逆、換參考系；再把剛體速度寫成六維旋量，理解 V_s 與 V_b 的差異，並用 Adjoint 在座標系之間轉換。',
  estMinutes: 35,
  tags: ['SE(3)', 'twist', 'Adjoint'],
};
