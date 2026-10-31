/**
 * Maps for the potential-field lesson: a 10 m × 7.5 m floor with a point robot. Each obstacle
 * group is dragged as a unit; concave shapes are unions of convex walls.
 */
import type { P2 } from '@/mr';
import { boxObs, diskObs, type Bounds, type ObstacleGroup, type SphereWorld } from './potential';

export const WORLD: Bounds = { xmin: 0, xmax: 10, ymin: 0, ymax: 7.5 };

export type SceneId = 'open' | 'convex' | 'utrap' | 'corridor' | 'sphere' | 'clutter';

export interface Scene {
  id: SceneId;
  label: string;
  groups: ObstacleGroup[];
  start: P2;
  goal: P2;
  /** Present for the navigation-function scene (obstacles are its disks, plus the outer circle). */
  sphere?: SphereWorld;
}

/**
 * U-shaped trap opening toward −x, built from three convex walls (so ∇U stays smooth outside).
 * The top arm comes first: the UI puts the group's drag handle on the first part, away from the
 * symmetry line where the trap's force arrows are drawn.
 */
export function uGroup(cx: number, cy: number, name = 'U 形障礙'): ObstacleGroup {
  const w = 0.4, depth = 2.4, half = 1.55;
  const xb = cx + depth / 2; // back wall (closed side) toward +x
  return {
    name,
    parts: [
      boxObs(xb - depth, cy + half - w, xb - w, cy + half),
      boxObs(xb - w, cy - half, xb, cy + half),
      boxObs(xb - depth, cy - half, xb - w, cy - half + w),
    ],
  };
}

export const SPHERE_WORLD: SphereWorld = {
  center: [5, 3.75],
  R: 3.55,
  disks: [
    { c: [5.0, 3.75], r: 0.75 },
    { c: [3.15, 5.0], r: 0.5 },
    { c: [6.55, 2.2], r: 0.55 },
  ],
};

export function makeScene(id: SceneId): Scene {
  switch (id) {
    case 'open':
      return { id, label: '空地（只有吸引位能）', groups: [], start: [1.3, 1.4], goal: [8.4, 5.6] };
    case 'convex':
      return {
        id, label: '凸障礙', start: [1.1, 1.3], goal: [8.8, 6.1],
        groups: [
          { name: '圓 A', parts: [diskObs(3.6, 3.0, 0.75)] },
          { name: '方塊', parts: [boxObs(5.3, 4.2, 6.3, 5.6)] },
          { name: '圓 B', parts: [diskObs(7.2, 2.5, 0.6)] },
        ],
      };
    case 'utrap':
      return { id, label: 'U 形陷阱', start: [1.3, 3.75], goal: [8.9, 3.75], groups: [uGroup(5.4, 3.75)] };
    case 'corridor':
      return {
        id, label: '窄走廊', start: [1.1, 6.3], goal: [9.0, 1.2],
        groups: [
          { name: '上牆', parts: [boxObs(3.2, 4.25, 6.8, 7.5)] },
          { name: '下牆', parts: [boxObs(3.2, 0, 6.8, 3.25)] },
        ],
      };
    case 'sphere':
      return {
        id, label: '球形世界（導航函數）', start: [2.2, 2.3], goal: [7.9, 4.6],
        groups: SPHERE_WORLD.disks.map((d, i) => ({ name: `圓 ${i + 1}`, parts: [diskObs(d.c[0], d.c[1], d.r)] })),
        sphere: SPHERE_WORLD,
      };
    case 'clutter':
    default:
      return {
        id: 'clutter', label: 'U 形＋雜物（RRT）', start: [1.0, 3.75], goal: [9.2, 3.75],
        groups: [
          uGroup(5.2, 3.75),
          { name: '圓 A', parts: [diskObs(2.6, 6.2, 0.55)] },
          { name: '圓 B', parts: [diskObs(2.7, 1.3, 0.6)] },
          { name: '方塊', parts: [boxObs(7.4, 5.6, 8.2, 7.5)] },
          { name: '方塊 2', parts: [boxObs(7.4, 0, 8.2, 1.9)] },
        ],
      };
  }
}
