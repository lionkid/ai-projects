/**
 * Map presets for the grid-search lesson. Every map is a MASTER × MASTER bitmap; the lesson
 * down-samples it conservatively to the chosen resolution (see grid.ts `downsample`).
 * S and G are given as fractions of the map size so they survive resolution changes.
 */
import { planarFK, segPointDistance, type P2 } from '@/mr';

export const MASTER = 128;

export type MapKey = 'blocks' | 'utrap' | 'gap' | 'cspace2r' | 'empty';

export interface MapPreset {
  key: MapKey;
  label: string;
  /** Toroidal neighbourhood (joint angles wrap around). */
  wrap: boolean;
  /** Start / goal as fractions (u, v) ∈ [0, 1)² of the map. */
  S: P2;
  G: P2;
  build(): Uint8Array;
}

/** Fill axis-aligned rectangles [x0, x1) × [y0, y1) given in master cells (y up). */
export function rects(list: readonly [number, number, number, number][]): Uint8Array {
  const m = new Uint8Array(MASTER * MASTER);
  for (const [x0, x1, y0, y1] of list) {
    for (let y = Math.max(0, y0); y < Math.min(MASTER, y1); y++) m.fill(1, y * MASTER + Math.max(0, x0), y * MASTER + Math.min(MASTER, x1));
  }
  return m;
}

/** The 2R arm and disk obstacle of the previous lesson's "single obstacle" scenario. */
export const ARM_2R = { L: [1, 1] as [number, number], halfWidth: 0.06, obstacle: { c: [0.75, 0.65] as P2, r: 0.22 } };

/** Map fraction ↔ joint angle for the 2R C-space map (column/row 0 starts at −π). */
export const fracToAngle = (u: number): number => -Math.PI + u * 2 * Math.PI;
export const angleToFrac = (theta: number): number => (theta + Math.PI) / (2 * Math.PI);

/**
 * C-obstacle of the 2R arm sampled at the master cell centres: links are capsules of half-width w,
 * the obstacle a disk (same test as the previous lesson, recomputed here so lessons stay independent).
 */
export function cspace2R(M = MASTER): Uint8Array {
  const { L, halfWidth: w, obstacle } = ARM_2R;
  const out = new Uint8Array(M * M);
  const reach = obstacle.r + w;
  for (let I = 0; I < M; I++) {
    const t1 = fracToAngle((I + 0.5) / M);
    const elbow = planarFK([L[0]], [t1]).tip as P2;
    const hit1 = segPointDistance([0, 0], elbow, obstacle.c) <= reach;
    for (let J = 0; J < M; J++) {
      if (hit1) { out[J * M + I] = 1; continue; }
      const tip = planarFK([L[0], L[1]], [t1, fracToAngle((J + 0.5) / M)]).tip as P2;
      if (segPointDistance(elbow, tip, obstacle.c) <= reach) out[J * M + I] = 1;
    }
  }
  return out;
}

const deg = (a: number): number => angleToFrac((a * Math.PI) / 180);

export const MAPS: MapPreset[] = [
  // Three blocks between S and G: A* stays near the straight line, Dijkstra floods everything.
  {
    key: 'blocks', label: '三個方塊', wrap: false, S: [0.1, 0.2], G: [0.9, 0.8],
    build: () => rects([[32, 48, 32, 64], [56, 72, 72, 104], [84, 100, 24, 64]]),
  },
  // A U-shaped trap that opens towards S with G behind it: the heuristic first pulls the search into the U.
  {
    key: 'utrap', label: 'U 形陷阱', wrap: false, S: [0.12, 0.44], G: [0.88, 0.56],
    build: () => rects([[84, 92, 36, 92], [52, 92, 84, 92], [52, 92, 36, 44]]),
  },
  // A full-width wall with one gap (8 master cells wide, not aligned to coarse cells): closed at 16 × 16.
  { key: 'gap', label: '窄縫', wrap: false, S: [0.3, 0.8], G: [0.3, 0.2], build: () => rects([[0, 76, 60, 68], [84, 128, 60, 68]]) },
  // Previous lesson: C-obstacle of the 2R arm on the (θ₁, θ₂) torus.
  { key: 'cspace2r', label: '2R C-space（上一課）', wrap: true, S: [deg(-40), deg(40)], G: [deg(130), deg(-40)], build: () => cspace2R() },
  // Nothing at all: paint your own obstacles.
  { key: 'empty', label: '空白', wrap: false, S: [0.15, 0.5], G: [0.85, 0.5], build: () => new Uint8Array(MASTER * MASTER) },
];

/** Cell used to show the neighbourhood in step 1 (next to the top-left corner of the first block at 16 × 16). */
export const PROBE: P2 = [0.22, 0.47];
