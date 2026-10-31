import type { LessonMeta } from '@/types';

export const meta: LessonMeta = {
  id: 'ch07-01-five-bar-ik-fk',
  chapter: 7,
  order: 1,
  title: '平面五連桿並聯機構：IK 容易、FK 多解（closed chain）',
  bookRef: '§7.1',
  summary: '以兩顆底座馬達驅動的五連桿為例，體會閉鏈中致動關節與被動關節的區分，IK 可拆成兩個 2R 子問題，而 FK 需解兩圓交點且通常有多個解。',
  estMinutes: 30,
  tags: ['parallel mechanism', 'five-bar', 'loop closure'],
};
