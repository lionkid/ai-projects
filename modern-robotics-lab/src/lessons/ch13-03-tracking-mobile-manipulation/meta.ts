import type { LessonMeta } from '@/types';

export const meta: LessonMeta = {
  id: 'ch13-03-tracking-mobile-manipulation',
  chapter: 13,
  order: 3,
  title: '軌跡追蹤回授控制與移動操作（trajectory tracking, mobile manipulation）',
  bookRef: '§13.2.3, 13.3.3–13.3.4, 13.5',
  summary: '理解為何直線插值對車型機器人不可行而需要可行參考軌跡，設計前饋＋回授的追蹤控制律並觀察收斂，最後把底盤與手臂合併成移動操作的 Jacobian。',
  estMinutes: 40,
  tags: ['tracking control', 'feedforward', 'mobile manipulation'],
};
