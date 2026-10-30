import type { ChapterInfo } from '@/types';

/** Chapter list. Chapter 0 is the lab's own orientation; 2–13 follow the book. */
export const CHAPTERS: ChapterInfo[] = [
  { number: 0, title: '開始之前', titleEn: 'Getting Started', blurb: '認識這個實驗室的操作方式、座標與顏色慣例。' },
  { number: 2, title: '組態空間', titleEn: 'Configuration Space', blurb: '自由度、C-space 的拓撲與表示、完整與非完整約束、任務空間與工作空間。' },
  { number: 3, title: '剛體運動', titleEn: 'Rigid-Body Motions', blurb: '旋轉矩陣、指數座標、齊次變換、旋量（twist）、螺旋軸與力旋量（wrench）。' },
  { number: 4, title: '正向運動學', titleEn: 'Forward Kinematics', blurb: '指數積（PoE）公式的空間形式與物體形式。' },
  { number: 5, title: '速度運動學與靜力學', titleEn: 'Velocity Kinematics and Statics', blurb: 'Jacobian、奇異點、可操作度橢球與靜力對偶關係。' },
  { number: 6, title: '逆向運動學', titleEn: 'Inverse Kinematics', blurb: '解析解與 Newton–Raphson 數值解法。' },
  { number: 7, title: '閉鏈運動學', titleEn: 'Kinematics of Closed Chains', blurb: '並聯機構的正逆運動學與奇異點。' },
  { number: 8, title: '開鏈動力學', titleEn: 'Dynamics of Open Chains', blurb: 'Lagrangian 與 Newton–Euler 動力學、質量矩陣、正向動力學模擬。' },
  { number: 9, title: '軌跡生成', titleEn: 'Trajectory Generation', blurb: '時間縮放、點對點與經由點軌跡、時間最佳化。' },
  { number: 10, title: '運動規劃', titleEn: 'Motion Planning', blurb: 'C-space 障礙物、格點搜尋、取樣式規劃與位能場。' },
  { number: 11, title: '機器人控制', titleEn: 'Robot Control', blurb: '誤差動態、PID、計算力矩法、力控制、混合控制與阻抗控制。' },
  { number: 12, title: '抓取與操作', titleEn: 'Grasping and Manipulation', blurb: '接觸運動學、摩擦錐、形封閉與力封閉。' },
  { number: 13, title: '輪式移動機器人', titleEn: 'Wheeled Mobile Robots', blurb: '全向與非完整移動機器人的運動學、里程計與回授控制。' },
];

export function chapterInfo(n: number): ChapterInfo {
  return CHAPTERS.find((c) => c.number === n) ?? { number: n, title: `第 ${n} 章`, titleEn: `Chapter ${n}`, blurb: '' };
}

export const BOOK = {
  title: 'Modern Robotics: Mechanics, Planning, and Control',
  authors: 'Kevin M. Lynch & Frank C. Park',
  edition: '2nd printing / online v2 preprint',
  site: 'https://modernrobotics.org',
  pdf: 'https://hades.mech.northwestern.edu/images/2/2e/MR-largefont-v2.pdf',
};
