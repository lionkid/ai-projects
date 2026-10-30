import type { LessonMeta } from '@/types';

export const meta: LessonMeta = {
  id: 'ch08-04-actuation-gearing-friction',
  chapter: 8,
  order: 4,
  title: '致動、齒輪與摩擦（gearing, reflected inertia, friction）',
  bookRef: '§8.9',
  summary: '把馬達與減速機加進模型：齒輪比帶來 G²J_m 的表觀慣量使動態近似解耦，摩擦模型帶來低速黏滑，並理解高減速比對背驅動性與安全的代價。',
  estMinutes: 30,
  tags: ['DC motor', 'gear ratio', 'friction'],
};
