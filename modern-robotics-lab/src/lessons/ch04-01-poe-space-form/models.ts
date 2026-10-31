/**
 * The four chains used in this lesson, each with its PoE description (M, Slist) and a small
 * stick-figure geometry for drawing. Geometry is given at the home configuration in {s}; every
 * piece is attached to a "body": 0 = ground, i = the link moved by joints 1..i.
 */
import { type Mat, type Vec, RpToTrans, ScrewToAxis, eye, planar3R, rrr3 } from '@/mr';

export type ModelKey = 'oneR' | 'rrr3' | 'planar3R' | 'rrrp';
export type JointType = 'R' | 'P';

/** A cylinder of the stick figure, rigidly attached to body `body`, given at home in {s}. */
export interface Segment {
  body: number;
  a: Vec;
  b: Vec;
  r: number;
  role: 'base' | 'link' | 'joint';
}

export interface ChainModel {
  key: ModelKey;
  /** Chip / badge label (zh-TW). */
  label: string;
  M: Mat;
  /** 6 × n, columns are the joint screw axes in {s} at home. */
  Slist: Mat;
  types: JointType[];
  limits: [number, number][];
  /** A point on every joint axis (at home, in {s}) where its glyph is centred; joint i rides on body i − 1. */
  anchors: Vec[];
  segments: Segment[];
  /** Typical link length (m): frames, glyphs and labels scale with it. */
  unit: number;
  /** Axis glyph extent behind / in front of the anchor, in units of `unit`. */
  glyph: { back: number; front: number };
  camera: { pos: [number, number, number]; target: [number, number, number] };
  grid: number;
  /** A pose that shows the arm well. */
  demo: number[];
}

const DEG = Math.PI / 180;
const col = (axes: Vec[]): Mat => [0, 1, 2, 3, 4, 5].map((r) => axes.map((a) => a[r]));

// ---------------------------------------------------------------------------
// 1R: one revolute joint on a pedestal that stands 0.2 m in front of {s}
// ---------------------------------------------------------------------------
/** Pedestal position, joint height and link length of the one-joint arm (m). */
export const ONE_R = { x: 0.2, h: 0.3, L: 0.45 } as const;

function oneRModel(): ChainModel {
  const { x, h, L } = ONE_R;
  const q = [x, 0, h];
  return {
    key: 'oneR',
    label: '1R 單關節',
    M: RpToTrans(eye(3), [x + L, 0, h]),
    Slist: col([ScrewToAxis(q, [0, 1, 0], 0)]),
    types: ['R'],
    limits: [[-Math.PI, 0.7]],
    anchors: [q],
    segments: [
      { body: 0, a: [x, 0, 0], b: [x, 0, 0.025], r: 0.07, role: 'base' },
      { body: 0, a: [x, 0, 0], b: [x, 0, h], r: 0.022, role: 'link' },
      { body: 0, a: [x, -0.05, h], b: [x, 0.05, h], r: 0.034, role: 'joint' },
      { body: 1, a: [x, 0, h], b: [x + L, 0, h], r: 0.019, role: 'link' },
    ],
    unit: 0.4,
    glyph: { back: 0.55, front: 0.55 },
    camera: { pos: [1.12, -0.9, 0.88], target: [0.36, 0.02, 0.31] },
    grid: 2,
    demo: [-40 * DEG],
  };
}

// ---------------------------------------------------------------------------
// Spatial RRR (the lab's rrr3): base yaw, shoulder and elbow about horizontal axes
// ---------------------------------------------------------------------------
function rrr3Model(): ChainModel {
  const L0 = 0.3, L1 = 0.4, L2 = 0.3;
  return {
    key: 'rrr3',
    label: '空間 RRR',
    M: rrr3.M,
    Slist: rrr3.Slist,
    types: ['R', 'R', 'R'],
    limits: rrr3.jointLimits ?? [[-Math.PI, Math.PI], [-Math.PI / 2, Math.PI / 2], [-2.6, 2.6]],
    anchors: [[0, 0, 0.16], [0, 0, L0], [L1, 0, L0]],
    segments: [
      { body: 0, a: [0, 0, 0], b: [0, 0, 0.025], r: 0.08, role: 'base' },
      { body: 0, a: [0, 0, 0], b: [0, 0, 0.07], r: 0.042, role: 'joint' },
      { body: 1, a: [0, 0, 0.07], b: [0, 0, L0], r: 0.025, role: 'link' },
      { body: 1, a: [0, -0.045, L0], b: [0, 0.045, L0], r: 0.035, role: 'joint' },
      { body: 2, a: [0, 0, L0], b: [L1, 0, L0], r: 0.02, role: 'link' },
      { body: 2, a: [L1, -0.04, L0], b: [L1, 0.04, L0], r: 0.03, role: 'joint' },
      { body: 3, a: [L1, 0, L0], b: [L1 + L2, 0, L0], r: 0.017, role: 'link' },
    ],
    unit: 0.4,
    glyph: { back: 0.5, front: 0.5 },
    camera: { pos: [0.98, -0.86, 0.74], target: [0.3, 0.02, 0.2] },
    grid: 2,
    demo: [35 * DEG, -35 * DEG, 70 * DEG],
  };
}

// ---------------------------------------------------------------------------
// Planar 3R (the lab's planar3R: 1.0, 0.8, 0.6 m) lying in the xy-plane
// ---------------------------------------------------------------------------
function planar3RModel(): ChainModel {
  const L = planar3R.linkLengths ?? [1, 0.8, 0.6];
  const x1 = L[0], x2 = L[0] + L[1], x3 = x2 + L[2];
  return {
    key: 'planar3R',
    label: '平面 3R',
    M: planar3R.M,
    Slist: planar3R.Slist,
    types: ['R', 'R', 'R'],
    limits: planar3R.jointLimits ?? [[-Math.PI, Math.PI], [-Math.PI, Math.PI], [-Math.PI, Math.PI]],
    anchors: [[0, 0, 0], [x1, 0, 0], [x2, 0, 0]],
    segments: [
      { body: 0, a: [0, 0, -0.03], b: [0, 0, 0.0], r: 0.16, role: 'base' },
      { body: 0, a: [0, 0, -0.02], b: [0, 0, 0.08], r: 0.075, role: 'joint' },
      { body: 1, a: [0, 0, 0], b: [x1, 0, 0], r: 0.045, role: 'link' },
      { body: 1, a: [x1, 0, -0.07], b: [x1, 0, 0.07], r: 0.065, role: 'joint' },
      { body: 2, a: [x1, 0, 0], b: [x2, 0, 0], r: 0.04, role: 'link' },
      { body: 2, a: [x2, 0, -0.06], b: [x2, 0, 0.06], r: 0.055, role: 'joint' },
      { body: 3, a: [x2, 0, 0], b: [x3, 0, 0], r: 0.034, role: 'link' },
    ],
    unit: 1.0,
    glyph: { back: 0.15, front: 0.55 },
    camera: { pos: [1.25, -2.05, 2.2], target: [1.05, 0.35, 0] },
    grid: 6,
    demo: [25 * DEG, 40 * DEG, 50 * DEG],
  };
}

// ---------------------------------------------------------------------------
// RRRP (SCARA-like): three vertical revolute joints, then a vertical prismatic quill
// ---------------------------------------------------------------------------
/** Arm height, link lengths, quill top and tool tip height at home (m). */
export const RRRP = { H: 0.4, A1: 0.35, A2: 0.25, top: 0.56, tip: 0.15 } as const;

function rrrpModel(): ChainModel {
  const { H, A1, A2, top, tip } = RRRP;
  const x2 = A1 + A2;
  const z: Vec = [0, 0, 1];
  return {
    key: 'rrrp',
    label: 'RRRP（SCARA 類）',
    // tool frame: x along +x, z pointing down (rotation by π about x)
    M: [
      [1, 0, 0, x2],
      [0, -1, 0, 0],
      [0, 0, -1, tip],
      [0, 0, 0, 1],
    ],
    Slist: col([
      ScrewToAxis([0, 0, 0], z, 0),
      ScrewToAxis([A1, 0, 0], z, 0),
      ScrewToAxis([x2, 0, 0], z, 0),
      [0, 0, 0, 0, 0, 1],
    ]),
    types: ['R', 'R', 'R', 'P'],
    limits: [[-150 * DEG, 150 * DEG], [-150 * DEG, 150 * DEG], [-Math.PI, Math.PI], [-0.12, 0.15]],
    // the P glyph sits beside the quill (a prismatic axis has a direction but no location)
    anchors: [[0, 0, H], [A1, 0, H], [x2, 0, 0.5], [x2 + 0.07, 0, 0.3]],
    segments: [
      { body: 0, a: [0, 0, 0], b: [0, 0, 0.025], r: 0.085, role: 'base' },
      { body: 0, a: [0, 0, 0], b: [0, 0, H - 0.03], r: 0.04, role: 'link' },
      { body: 0, a: [0, 0, H - 0.05], b: [0, 0, H + 0.04], r: 0.052, role: 'joint' },
      { body: 1, a: [0, 0, H], b: [A1, 0, H], r: 0.028, role: 'link' },
      { body: 1, a: [A1, 0, H - 0.045], b: [A1, 0, H + 0.04], r: 0.044, role: 'joint' },
      { body: 2, a: [A1, 0, H], b: [x2, 0, H], r: 0.024, role: 'link' },
      { body: 2, a: [x2, 0, H - 0.035], b: [x2, 0, H + 0.025], r: 0.042, role: 'joint' },
      { body: 3, a: [x2, 0, H + 0.025], b: [x2, 0, H + 0.11], r: 0.027, role: 'joint' },
      { body: 3, a: [x2, 0, H + 0.09], b: [x2 + 0.075, 0, H + 0.09], r: 0.009, role: 'link' },
      { body: 4, a: [x2, 0, top], b: [x2, 0, tip], r: 0.011, role: 'link' },
    ],
    unit: 0.35,
    glyph: { back: 0.45, front: 0.6 },
    camera: { pos: [1.14, -1.04, 0.95], target: [0.3, 0.04, 0.3] },
    grid: 2,
    demo: [30 * DEG, 60 * DEG, -45 * DEG, 0.08],
  };
}

export const MODELS: Record<ModelKey, ChainModel> = {
  oneR: oneRModel(),
  rrr3: rrr3Model(),
  planar3R: planar3RModel(),
  rrrp: rrrpModel(),
};

export const MODEL_ORDER: ModelKey[] = ['oneR', 'rrr3', 'planar3R', 'rrrp'];
