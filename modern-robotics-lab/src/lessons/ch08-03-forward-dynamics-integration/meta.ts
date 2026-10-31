import type { LessonMeta } from '@/types';

export const meta: LessonMeta = {
  id: 'ch08-03-forward-dynamics-integration',
  chapter: 8,
  order: 3,
  title: '正動力學與數值積分：Euler vs RK4（forward dynamics）',
  bookRef: '§8.4–8.5',
  summary: '由力矩解出加速度 d²θ/dt² = M⁻¹(τ − c − g) 並積分出運動，比較 Euler 與 RK4 在同一步長下的能量漂移，學會選步長與積分法。',
  estMinutes: 30,
  tags: ['ForwardDynamics', 'RK4', 'energy drift'],
};
