import type { LessonMeta } from '@/types';

export const meta: LessonMeta = {
  id: 'ch06-01-analytic-ik',
  chapter: 6,
  order: 1,
  title: '解析逆運動學：2R 手臂的兩組解（analytic IK）',
  bookRef: '§6.1',
  summary: '用餘弦定理與 atan2 導出 2R 手臂的 elbow-up / elbow-down 解，判斷可達性與邊界上的單解，並預覽 6R 手臂「位置三關節＋姿態三關節」的分解思路。',
  estMinutes: 25,
  tags: ['IK', 'atan2', 'reachability'],
};
