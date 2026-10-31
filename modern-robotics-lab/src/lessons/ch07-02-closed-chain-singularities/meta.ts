import type { LessonMeta } from '@/types';

export const meta: LessonMeta = {
  id: 'ch07-02-closed-chain-singularities',
  chapter: 7,
  order: 2,
  title: '閉鏈的微分運動學與兩類奇異點（actuator / end-effector singularity）',
  bookRef: '§7.2–7.3',
  summary: '把閉鏈的速度關係寫成致動與被動兩塊的約束 Jacobian，區分「鎖住馬達末端仍能動」的致動器奇異與「某方向動不了」的末端奇異，並在五連桿上親手製造。',
  estMinutes: 30,
  tags: ['constraint Jacobian', 'actuator singularity', 'end-effector singularity'],
};
