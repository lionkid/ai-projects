/**
 * Planning problems for the RRT / PRM lesson: point-robot maps on a 12 m × 9 m floor and the
 * C-space of a planar 2R arm (a flat torus of joint angles) with disk obstacles in its workspace.
 * Collision geometry uses the 2-D predicates from `@/mr`.
 */
import {
  distPointPolygon, planarFK, pointInPolygon, rectPolygon, segCircleIntersects, segPointDistance, segPolygonIntersects,
  type P2,
} from '@/mr';
import { CSpace, type Bounds, type Q } from './planners';

export type Obstacle =
  | { kind: 'rect'; x0: number; y0: number; x1: number; y1: number }
  | { kind: 'circle'; c: P2; r: number };

export type MapId = 'clutter' | 'empty' | 'wall' | 'maze' | 'narrow' | 'arm';

export interface PlanMap {
  id: MapId;
  label: string;
  bounds: Bounds;
  torus: boolean;
  obstacles: Obstacle[];
  start: Q;
  goal: Q | null;
}

export const FLOOR: Bounds = { xmin: 0, xmax: 12, ymin: 0, ymax: 9 };
export const TORUS: Bounds = { xmin: -Math.PI, xmax: Math.PI, ymin: -Math.PI, ymax: Math.PI };

const rect = (x0: number, y0: number, x1: number, y1: number): Obstacle => ({ kind: 'rect', x0, y0, x1, y1 });
const disk = (x: number, y: number, r: number): Obstacle => ({ kind: 'circle', c: [x, y], r });

/** Wall thickness and limits of the adjustable narrow passage. */
export const NARROW = { y0: 3.7, y1: 5.3, cx: 6, minGap: 0.15, maxGap: 2 };

export function narrowObstacles(gap: number): Obstacle[] {
  const g = Math.max(NARROW.minGap, Math.min(NARROW.maxGap, gap));
  return [rect(0, NARROW.y0, NARROW.cx - g / 2, NARROW.y1), rect(NARROW.cx + g / 2, NARROW.y0, 12, NARROW.y1)];
}

// ---- 2R arm (joint-angle torus) -------------------------------------------------------------

export const ARM_L: [number, number] = [1, 1];
/** Half-width of the (capsule-shaped) links. */
export const ARM_W = 0.07;
export const ARM_OBSTACLES: { c: P2; r: number }[] = [
  { c: [1.05, 0.95], r: 0.3 },
  { c: [-0.15, -1.25], r: 0.28 },
  { c: [-1.55, 0.35], r: 0.22 },
];

export function makeMap(id: MapId, gap = 0.8): PlanMap {
  switch (id) {
    case 'empty':
      return { id, label: '空地', bounds: FLOOR, torus: false, obstacles: [], start: [6, 4.5], goal: null };
    case 'wall':
      return { id, label: '一道牆', bounds: FLOOR, torus: false, obstacles: [rect(5.6, 0, 6.4, 7.0), rect(3.2, 6.2, 5.6, 7.0)], start: [2.2, 2.5], goal: [10, 2.5] };
    case 'maze':
      return {
        id, label: '迷宮', bounds: FLOOR, torus: false,
        obstacles: [rect(2.8, 0, 3.4, 6.6), rect(5.7, 2.4, 6.3, 9), rect(8.6, 0, 9.2, 6.6)],
        start: [1.3, 1.3], goal: [10.7, 1.3],
      };
    case 'narrow':
      return { id, label: '窄通道', bounds: FLOOR, torus: false, obstacles: narrowObstacles(gap), start: [2.5, 1.6], goal: [9.5, 7.4] };
    case 'arm':
      return {
        id, label: '2R 手臂 C-space', bounds: TORUS, torus: true,
        obstacles: ARM_OBSTACLES.map((o) => disk(o.c[0], o.c[1], o.r)),
        start: [-2.55, 0.9], goal: [2.45, -0.75],
      };
    case 'clutter':
    default:
      return {
        id: 'clutter', label: '散落障礙', bounds: FLOOR, torus: false,
        obstacles: [
          rect(2.0, 1.4, 3.1, 4.8), disk(6.0, 2.4, 1.05), rect(4.4, 5.3, 7.6, 6.2), rect(8.7, 2.8, 9.7, 7.6),
          disk(3.4, 7.1, 0.75), rect(9.9, 0.9, 11.3, 1.7), rect(5.0, 3.9, 5.25, 5.3),
        ],
        start: [1.0, 0.9], goal: [11.0, 8.1],
      };
  }
}

// ---- collision predicates -------------------------------------------------------------------

export function pointFree(obstacles: readonly Obstacle[], q: Q): boolean {
  for (const o of obstacles) {
    if (o.kind === 'rect') { if (q[0] >= o.x0 && q[0] <= o.x1 && q[1] >= o.y0 && q[1] <= o.y1) return false; }
    else if (Math.hypot(q[0] - o.c[0], q[1] - o.c[1]) <= o.r) return false;
  }
  return true;
}

/** Exact test: does the straight segment a→b touch any obstacle? (used to expose missed collisions) */
export function segmentHits(obstacles: readonly Obstacle[], a: Q, b: Q): boolean {
  for (const o of obstacles) {
    if (o.kind === 'rect') { if (segPolygonIntersects(a, b, rectPolygon(o.x0, o.y0, o.x1, o.y1))) return true; }
    else if (segCircleIntersects(a, b, o.c, o.r)) return true;
  }
  return false;
}

/** Distance from q to the nearest obstacle (0 inside), +Infinity without obstacles. */
export function obstacleDistance(obstacles: readonly Obstacle[], q: Q): number {
  let d = Infinity;
  for (const o of obstacles) {
    if (o.kind === 'rect') d = Math.min(d, distPointPolygon(q, rectPolygon(o.x0, o.y0, o.x1, o.y1)));
    else d = Math.min(d, Math.max(0, Math.hypot(q[0] - o.c[0], q[1] - o.c[1]) - o.r));
  }
  return d;
}

/** Base, elbow and tip of the 2R arm at joint angles q (forward kinematics from @/mr). */
export function armPoints(q: Q): [P2, P2, P2] {
  const fk = planarFK(ARM_L, [q[0], q[1]]);
  return [[0, 0], [fk.joints[1][0], fk.joints[1][1]], [fk.joints[2][0], fk.joints[2][1]]];
}

/** Is the thick 2R arm (capsules of half-width ARM_W) clear of every disk obstacle? */
export function armFree(obstacles: readonly { c: P2; r: number }[], q: Q): boolean {
  const [b, e, t] = armPoints(q);
  for (const o of obstacles) {
    if (segPointDistance(b, e, o.c) <= o.r + ARM_W) return false;
    if (segPointDistance(e, t, o.c) <= o.r + ARM_W) return false;
  }
  return true;
}

/** Collision-checked configuration space for a map. For the arm, `torus = false` pretends the angle square has walls. */
export function makeSpace(m: PlanMap, eps: number, torus = m.torus): CSpace {
  if (m.id === 'arm') {
    const disks = m.obstacles.filter((o): o is Extract<Obstacle, { kind: 'circle' }> => o.kind === 'circle');
    return new CSpace(m.bounds, torus, (q) => armFree(disks, q), eps);
  }
  const obs = m.obstacles;
  return new CSpace(m.bounds, false, (q) => pointFree(obs, q), eps);
}

/** Polygon view of an obstacle (for drawing / exact tests). */
export function obstaclePolygon(o: Obstacle): P2[] | null {
  return o.kind === 'rect' ? rectPolygon(o.x0, o.y0, o.x1, o.y1) : null;
}

export { pointInPolygon };
