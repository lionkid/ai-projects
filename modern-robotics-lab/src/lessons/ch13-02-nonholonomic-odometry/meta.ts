import type { LessonMeta } from '@/types';

export const meta: LessonMeta = {
  id: 'ch13-02-nonholonomic-odometry',
  chapter: 13,
  order: 2,
  title: '非完整機器人：差速、車型與里程計（nonholonomic, odometry）',
  bookRef: '§13.3.1–13.3.2, 13.4',
  summary: '寫出單輪車、差速與車型機器人的控制系統 q̇ = G(q)u，用 Lie bracket 的直覺理解側滑方向為何仍可達（可控性），並由輪子編碼器積分里程計觀察漂移。',
  estMinutes: 35,
  tags: ['unicycle', 'differential drive', 'car-like', 'odometry'],
};
