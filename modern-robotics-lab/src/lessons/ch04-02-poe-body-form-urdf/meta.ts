import type { LessonMeta } from '@/types';

export const meta: LessonMeta = {
  id: 'ch04-02-poe-body-form-urdf',
  chapter: 4,
  order: 2,
  title: 'PoE 物體形式、UR5 範例與 URDF（body form）',
  bookRef: '§4.1.3, 4.2',
  summary: '用末端座標系的螺旋軸 B_i 寫出同一個 FK，推導 B_i = Ad_{M⁻¹} S_i，在 UR5 上驗證兩種形式一致，並對照 URDF 的關節描述。',
  estMinutes: 35,
  tags: ['FKinBody', 'UR5', 'URDF'],
};
