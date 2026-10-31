import type { LessonMeta } from '@/types';

export const meta: LessonMeta = {
  id: 'ch03-04-screw-motions-wrenches',
  chapter: 3,
  order: 4,
  title: '螺旋運動的指數座標與力旋量（screw motion, wrench）',
  bookRef: '§3.3.3–3.4',
  summary: '任何剛體位移都是沿某條螺旋軸的一段運動：由 (q, ŝ, h) 建構 S，動畫 exp([S]θ)，再用 log 反推；最後把力與力矩合成 wrench，用 Adᵀ 換座標並確認功率不變。',
  estMinutes: 40,
  tags: ['screw axis', 'MatrixExp6', 'wrench'],
};
