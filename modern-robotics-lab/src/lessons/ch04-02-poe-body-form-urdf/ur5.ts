/**
 * Drawing geometry and named poses for the lab's UR5 model (book dimensions W1, W2, L1, L2, H1, H2).
 * The PoE description itself comes from @/mr (`ur5.M`, `ur5.Slist`, `ur5.Blist`).
 * Geometry is given at the home configuration in {s}; body 0 = ground, body i = moved by joints 1..i.
 */
import { type Vec, UR5_DIMENSIONS } from '@/mr';

export interface Segment {
  body: number;
  a: Vec;
  b: Vec;
  r: number;
  role: 'base' | 'link' | 'joint';
}

const { W1, W2, L1, L2, H1, H2 } = UR5_DIMENSIONS;
const X = L1 + L2;

/** Stick figure of the UR5 at home (arm stretched along +x, as in the book's figure). */
export const UR5_SEGMENTS: Segment[] = [
  { body: 0, a: [0, 0, 0], b: [0, 0, 0.018], r: 0.085, role: 'base' },
  { body: 0, a: [0, 0, 0], b: [0, 0, 0.05], r: 0.052, role: 'joint' },
  { body: 1, a: [0, 0, 0.05], b: [0, 0, H1], r: 0.042, role: 'link' },
  { body: 1, a: [0, -0.062, H1], b: [0, 0.062, H1], r: 0.05, role: 'joint' },
  { body: 2, a: [0, 0, H1], b: [L1, 0, H1], r: 0.034, role: 'link' },
  { body: 2, a: [L1, -0.05, H1], b: [L1, 0.05, H1], r: 0.043, role: 'joint' },
  { body: 3, a: [L1, 0, H1], b: [X, 0, H1], r: 0.029, role: 'link' },
  { body: 3, a: [X, -0.038, H1], b: [X, 0.045, H1], r: 0.036, role: 'joint' },
  { body: 4, a: [X, 0, H1], b: [X, W1, H1], r: 0.028, role: 'link' },
  { body: 4, a: [X, W1, H1 - 0.042], b: [X, W1, H1 + 0.042], r: 0.035, role: 'joint' },
  { body: 5, a: [X, W1, H1], b: [X, W1, H1 - H2], r: 0.027, role: 'link' },
  { body: 5, a: [X, W1 - 0.036, H1 - H2], b: [X, W1 + 0.036, H1 - H2], r: 0.033, role: 'joint' },
  { body: 6, a: [X, W1, H1 - H2], b: [X, W1 + W2, H1 - H2], r: 0.025, role: 'link' },
];

/** Length of the optional tool along ẑ_b (m) and its drawing (attached to the flange, body 6). */
export const TOOL_LENGTH = 0.1;
export const TOOL_SEGMENT: Segment = { body: 6, a: [X, W1 + W2, H1 - H2], b: [X, W1 + W2 + TOOL_LENGTH, H1 - H2], r: 0.011, role: 'link' };

/** A point on every joint axis at home (where its glyph is centred); joint i rides on body i − 1. */
export const UR5_ANCHORS: Vec[] = [
  [0, 0, 0.03],
  [0, 0, H1],
  [L1, 0, H1],
  [X, 0, H1],
  [X, W1, H1],
  [X, W1, H1 - H2],
];

const DEG = Math.PI / 180;

export interface NamedPose { key: string; label: string; title: string; theta: number[] }

export const POSES: NamedPose[] = [
  { key: 'home', label: '零位', title: 'θ = 0：手臂沿 +x 伸直', theta: [0, 0, 0, 0, 0, 0] },
  { key: 'book', label: '教材範例', title: 'θ = (0, −90°, 0, 0, 90°, 0)', theta: [0, -90 * DEG, 0, 0, 90 * DEG, 0] },
  { key: 'work', label: '姿勢 A', title: '一般的工作姿勢', theta: [35 * DEG, -60 * DEG, 85 * DEG, -110 * DEG, -70 * DEG, 30 * DEG] },
  { key: 'reach', label: '姿勢 B', title: '手臂前伸、工具朝外', theta: [50 * DEG, -60 * DEG, 45 * DEG, -60 * DEG, 30 * DEG, 30 * DEG] },
];

/** Book example: end-effector position at θ = (0, −π/2, 0, 0, π/2, 0) (m, rounded as printed). */
export const BOOK_P: Vec = [0.095, 0.109, 0.988];

export const UR5_CAMERA = { pos: [1.32, -1.3, 1.08] as [number, number, number], target: [0.3, 0.08, 0.3] as [number, number, number] };
