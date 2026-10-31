/**
 * Scenario presets for the C-obstacle lesson (pure data, unit-tested so that every claim the
 * lesson text makes about them – "the straight path hits", "two components", … – stays true).
 */
import type { P2 } from '@/mr';
import type { Obstacle } from './cspace';

export const LINK_HALF_WIDTH = 0.06;
export const LINKS: [number, number] = [1, 1];

export interface Scenario {
  key: 'single' | 'behind' | 'near' | 'pinch';
  label: string;
  obstacles: Obstacle[];
  S: P2;
  G: P2;
  /** Configuration shown when the scenario is loaded. */
  q: P2;
}

const deg = (a: number, b: number): P2 => [(a * Math.PI) / 180, (b * Math.PI) / 180];

export const SCENARIOS: Scenario[] = [
  // Link 1 sweeps straight through the disk on the direct joint-space path while the tip stays far away.
  { key: 'single', label: '單一障礙', obstacles: [{ c: [0.75, 0.65], r: 0.22 }], S: deg(-40, 40), G: deg(130, -40), q: deg(-40, 40) },
  // Disk behind the base: its C-obstacle straddles the θ₁ = ±180° seam of the square.
  { key: 'behind', label: '障礙在背後', obstacles: [{ c: [-1.15, -0.05], r: 0.3 }], S: deg(120, 30), G: deg(-120, -30), q: deg(120, 30) },
  // Same disk as 'single', arm parked close to it to look at clearance and the circle cover.
  { key: 'near', label: '貼近障礙', obstacles: [{ c: [0.75, 0.65], r: 0.22 }], S: deg(-40, 40), G: deg(130, -40), q: deg(8, 62) },
  // Two disks pinch the base: link 1 can pass neither, C-free splits into "right" and "left" halves.
  { key: 'pinch', label: '夾住基座', obstacles: [{ c: [0, 0.72], r: 0.26 }, { c: [0, -0.72], r: 0.26 }], S: deg(0, 40), G: deg(180, -40), q: deg(0, 40) },
];
