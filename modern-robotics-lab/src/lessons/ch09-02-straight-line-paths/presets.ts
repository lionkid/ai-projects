/** Preset scenarios shared by the lesson UI and its tests (so the narrative's claims are regression-tested). */
import type { Pose2, Elbow, IKPolicy, Obstacle } from './paths';

const deg = Math.PI / 180;
const P = (x: number, y: number, d: number): Pose2 => ({ x, y, phi: d * deg });

export interface Preset2D {
  start: Pose2;
  end: Pose2;
  elbow: Elbow;
  policy: IKPolicy;
  obstacle?: Obstacle;
}

const basic: Preset2D = { start: P(1.6, -0.4, -30), end: P(0.4, 1.5, 120), elbow: 'up', policy: 'warm' };

export const PRESETS_2D = {
  /** Three clearly different trails, everything reachable and far from singularities. */
  basic,
  /** Pure translation whose wrist point passes 0.25 m from the base: continuous, but the joint rates spike. */
  nearBase: { start: P(1.2, 0.85, 90), end: P(-1.2, 0.85, 90), elbow: 'down', policy: 'warm' } as Preset2D,
  /** The wrist line dips inside |w| < |L1 − L2| = 0.2 m: IK fails on a stretch, then the joints jump. */
  unreachable: { start: P(1.3, 0.72, 90), end: P(-0.9, 0.72, 90), elbow: 'down', policy: 'warm' } as Preset2D,
  /** Far from singularities, yet solving every sample from the start configuration switches branches. */
  cold: { start: P(1.85, -0.35, 20), end: P(-0.05, 0.85, -25), elbow: 'up', policy: 'cold' } as Preset2D,
  /** Blocks the outer arcs (joint and screw paths) while the Cartesian path passes. */
  obstacleA: { ...basic, obstacle: { c: [1.35, 0.95], r: 0.16 } } as Preset2D,
  /** Off the straight tip line, but the Cartesian path's links sweep through it. */
  obstacleB: { ...basic, obstacle: { c: [0.7, 0.1], r: 0.16 } } as Preset2D,
};

export interface Preset3D { alpha: number; dx: number; dy: number; dz: number }

export const PRESETS_3D: { label: string; p: Preset3D }[] = [
  { label: '純平移（α = 0）', p: { alpha: 0, dx: 0.8, dy: 0.8, dz: 0.45 } },
  { label: '水平轉 120°（h = 0）', p: { alpha: 120 * deg, dx: 0.8, dy: 0.8, dz: 0 } },
  { label: '螺旋：轉動 + 升降', p: { alpha: 120 * deg, dx: 0.8, dy: 0.8, dz: 0.45 } },
  { label: '起點在螺旋軸上', p: { alpha: 150 * deg, dx: 0, dy: 0, dz: 0.5 } },
];
