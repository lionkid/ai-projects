import type { LessonMeta } from '@/types';

export const meta: LessonMeta = {
  id: 'ch13-01-omnidirectional-kinematics',
  chapter: 13,
  order: 1,
  title: '全向移動機器人運動學（omniwheel / mecanum）',
  bookRef: '§13.1–13.2.1',
  summary: '由輪子幾何建立底盤 twist 到輪速的關係 u = H(0)V_b，理解輪速限制如何變成底盤速度的多面體，並比較三輪 omni 與四輪 mecanum。',
  estMinutes: 30,
  tags: ['omnidirectional', 'mecanum', 'H matrix'],
};
