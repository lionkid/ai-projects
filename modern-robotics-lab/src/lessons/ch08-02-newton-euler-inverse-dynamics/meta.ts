import type { LessonMeta } from '@/types';

export const meta: LessonMeta = {
  id: 'ch08-02-newton-euler-inverse-dynamics',
  chapter: 8,
  order: 2,
  title: '單剛體動力學與 Newton–Euler 逆動力學（inverse dynamics）',
  bookRef: '§8.2–8.3',
  summary: '先用 twist–wrench 形式寫單剛體方程，再看遞迴演算法如何前向傳遞速度與加速度、後向傳遞 wrench，沿一條軌跡產生各關節的力矩曲線並拆解成慣性、速度與重力貢獻。',
  estMinutes: 40,
  tags: ['Newton–Euler', 'spatial inertia', 'InverseDynamics'],
};
