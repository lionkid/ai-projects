import type { LessonMeta } from '@/types';

export const meta: LessonMeta = {
  id: 'ch09-02-straight-line-paths',
  chapter: 9,
  order: 2,
  title: '直線路徑：關節空間、螺旋與笛卡兒（joint / screw / Cartesian paths）',
  bookRef: '§9.2.1',
  summary: '在同一對起終點位姿上比較三種「直線」：關節空間直線、螺旋內插與解耦的笛卡兒直線，看末端軌跡與關節角曲線的差異，並判斷何時該用哪一種。',
  estMinutes: 30,
  tags: ['JointTrajectory', 'ScrewTrajectory', 'CartesianTrajectory'],
};
