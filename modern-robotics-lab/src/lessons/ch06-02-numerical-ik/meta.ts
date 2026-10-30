import type { LessonMeta } from '@/types';

export const meta: LessonMeta = {
  id: 'ch06-02-numerical-ik',
  chapter: 6,
  order: 2,
  title: '數值逆運動學：Newton–Raphson 與阻尼最小平方（numerical IK, DLS）',
  bookRef: '§6.2–6.3',
  summary: '把 IK 寫成求根問題，用 log 定義 twist 誤差並逐步執行 Newton 迭代觀察收斂曲線；近奇異點時改用阻尼最小平方，並連結到逆速度運動學。',
  estMinutes: 40,
  tags: ['IKinBody', 'Newton–Raphson', 'pseudoinverse', 'DLS'],
};
