import type { LessonMeta } from '@/types';

export const meta: LessonMeta = {
  id: 'ch08-01-lagrangian-mass-matrix',
  chapter: 8,
  order: 1,
  title: 'Lagrangian 動力學與質量矩陣 M(θ)（mass matrix）',
  bookRef: '§8.1',
  summary: '由動能與位能寫出 2R 手臂的 Lagrangian，導出 τ = M(θ)θ̈ + c(θ, θ̇) + g(θ)，把 M(θ) 讀成隨構型改變的慣量橢圓，並分辨科氏／向心項與重力項的物理來源。',
  estMinutes: 35,
  tags: ['Lagrangian', 'mass matrix', 'Coriolis'],
};
