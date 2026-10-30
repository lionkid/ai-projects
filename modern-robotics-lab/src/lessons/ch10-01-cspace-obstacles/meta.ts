import type { LessonMeta } from '@/types';

export const meta: LessonMeta = {
  id: 'ch10-01-cspace-obstacles',
  chapter: 10,
  order: 1,
  title: 'C-space 障礙物與碰撞檢測（C-obstacles）',
  bookRef: '§10.1–10.2.2',
  summary: '把工作空間裡的障礙映射成 2R 手臂的 C-space 障礙，理解規劃就是在 C-free 中找連續曲線，並認識以球體覆蓋做碰撞檢測與距離計算的做法。',
  estMinutes: 30,
  tags: ['C-obstacle', 'collision detection', 'planning problem'],
};
