/** Preset via-point configurations used by the lesson (and checked by the tests). */
import type { Pt2, VelStrategy } from './viaSpline';

export interface Scenario {
  label: string;
  title: string;
  points: Pt2[];
  times: number[];
  strategy: VelStrategy;
}

/** Start, three via points and the goal: an arch over an imaginary obstacle, all inside the 2R reach (r ≤ 2). */
const ARCH: Pt2[] = [[0.95, -0.3], [0.75, 0.5], [1.3, 0.9], [1.85, 0.5], [1.8, -0.3]];

export type ScenarioId = 'demo' | 'equal' | 'zero' | 'overshoot' | 'custom' | 'collinear';

export const SCENARIOS: Record<ScenarioId, Scenario> = {
  demo: { label: '三個經過點', title: '起點、三個經過點、終點；頭尾兩段較短，啟發式速度', points: ARCH, times: [0, 0.5, 2, 3.5, 4], strategy: 'heuristic' },
  equal: { label: '等時距', title: '同一組點，每段都給 1 s，啟發式速度：加速度只有溫和的跳變', points: ARCH, times: [0, 1, 2, 3, 4], strategy: 'heuristic' },
  zero: { label: '零速（停頓）', title: '同一組點，但每個經過點速度都設為 0', points: ARCH, times: [0, 1, 2, 3, 4], strategy: 'zero' },
  overshoot: { label: '短段 → 過衝', title: '第一段只給 0.4 s、第二段卻有 2.4 s：經過點速度被短段拉得很大', points: ARCH, times: [0, 0.4, 2.8, 3.4, 4.2], strategy: 'heuristic' },
  custom: { label: '自訂速度', title: '從中央差分速度出發，拖曳箭頭尖端自己調', points: ARCH, times: [0, 1, 2, 3, 4], strategy: 'custom' },
  collinear: { label: '共線三點', title: '三個點在同一直線上、兩段一樣長，但時間是 0.5 s 對 4 s', points: [[0.5, -0.45], [0.95, 0.15], [1.4, 0.75]], times: [0, 0.5, 4.5], strategy: 'heuristic' },
};

export const SCENARIO_IDS: ScenarioId[] = ['demo', 'equal', 'zero', 'overshoot', 'custom', 'collinear'];
