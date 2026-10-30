import type { LessonMeta } from '@/types';

export const meta: LessonMeta = {
  id: 'ch11-03-force-hybrid-control',
  chapter: 11,
  order: 3,
  title: '力控制與混合運動—力控制（force control, hybrid motion–force）',
  bookRef: '§11.5–11.6',
  summary: '理解純力控制需要重力補償與阻尼，把環境約束寫成 A(θ)V = 0 分離運動與力的子空間，並用投影矩陣設計擦桌子任務的混合控制器。',
  estMinutes: 35,
  tags: ['force control', 'hybrid control', 'natural constraints'],
};
