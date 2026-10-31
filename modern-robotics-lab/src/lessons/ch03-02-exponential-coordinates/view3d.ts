/**
 * Three.js pieces for this lesson's 3D view (same building blocks as ch03-01; lessons do not import each other). A `Triad` draws the three COLUMNS of any 3×3 matrix as
 * arrows (so a non-orthogonal "frame" visibly skews instead of being silently re-orthonormalised), and
 * `DashedSegments` draws a set of dashed line segments (drop lines, the unit-cube image, projections).
 * All colours are CSS variable names so they follow the theme (see Scene3D).
 */
import * as THREE from 'three';
import { Arrow3D, makeTextSprite, setThemedColor, type Scene3D } from '@/ui';
import type { Mat, Vec } from '@/mr';

export const AXIS_VARS = ['--axis-x', '--axis-y', '--axis-z'] as const;

export interface TriadOpts {
  radius?: number;
  head?: number;
  opacity?: number;
  /** Tip labels, e.g. ['x_b', 'y_b', 'z_b']; omitted = no labels. */
  names?: [string, string, string];
  /** Label colour (CSS variable); default: the axis colours. */
  labelVar?: string;
  labelScale?: number;
}

export class Triad {
  readonly group = new THREE.Group();
  readonly arrows: Arrow3D[];
  private labels: THREE.Sprite[] = [];

  constructor(scene: Scene3D, o: TriadOpts = {}) {
    this.arrows = AXIS_VARS.map((c) => new Arrow3D(c, o.radius ?? 0.025, o.head ?? 0.13, o.opacity ?? 1));
    for (const a of this.arrows) this.group.add(a.group);
    if (o.names) {
      o.names.forEach((n, i) => {
        const s = makeTextSprite(n, o.labelVar ?? AXIS_VARS[i], 40);
        s.scale.multiplyScalar(o.labelScale ?? 0.8);
        this.labels.push(s);
        this.group.add(s);
      });
    }
    scene.scene.add(this.group);
  }

  /** Arrows from `origin` to origin + length · (column j of M). */
  set(M: Mat, length = 1, origin: Vec = [0, 0, 0]): void {
    for (let j = 0; j < 3; j++) {
      const c = [M[0][j] * length, M[1][j] * length, M[2][j] * length];
      const tip = [origin[0] + c[0], origin[1] + c[1], origin[2] + c[2]];
      this.arrows[j].setFromTo(origin, tip);
      const lab = this.labels[j];
      if (lab) {
        const L = Math.hypot(c[0], c[1], c[2]);
        lab.visible = L > 1e-6;
        if (L > 1e-6) {
          const k = (L + 0.14) / L;
          lab.position.set(origin[0] + c[0] * k, origin[1] + c[1] * k, origin[2] + c[2] * k);
        }
      }
    }
  }

  setVisible(v: boolean): void { this.group.visible = v; }

  setOpacity(op: number): void {
    this.group.traverse((o) => {
      const mesh = o as THREE.Mesh;
      if (!mesh.isMesh) return;
      const m = mesh.material as THREE.MeshStandardMaterial;
      // switching blending after the first render needs a program update
      if (m.transparent !== op < 1) m.needsUpdate = true;
      m.transparent = op < 1;
      m.opacity = op;
      m.depthWrite = op >= 1;
    });
  }

  /** Opacity of a single arrow (and its label). */
  setArrowOpacity(j: number, op: number): void {
    this.arrows[j].group.traverse((o) => {
      const mesh = o as THREE.Mesh;
      if (!mesh.isMesh) return;
      const m = mesh.material as THREE.MeshStandardMaterial;
      // switching blending after the first render needs a program update
      if (m.transparent !== op < 1) m.needsUpdate = true;
      m.transparent = op < 1;
      m.opacity = op;
      m.depthWrite = op >= 1;
    });
    const lab = this.labels[j];
    if (lab) (lab.material as THREE.SpriteMaterial).opacity = op < 1 ? Math.max(0.35, op) : 1;
  }

  /** Thicken arrow j (null = none). */
  emphasize(j: number | null, k = 1.8): void {
    this.arrows.forEach((a, i) => a.group.scale.set(i === j ? k : 1, 1, i === j ? k : 1));
  }

  /** Free the label textures (Scene3D.destroy frees geometries and materials, not textures). */
  dispose(): void {
    for (const s of this.labels) (s.material as THREE.SpriteMaterial).map?.dispose();
  }
}

export class DashedSegments {
  readonly lines: THREE.LineSegments;
  private max: number;

  constructor(scene: Scene3D, colorVar: string, max: number, opts: { dash?: number; gap?: number; opacity?: number } = {}) {
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(max * 6), 3));
    geo.setDrawRange(0, 0);
    const op = opts.opacity ?? 1;
    const mat = new THREE.LineDashedMaterial({ dashSize: opts.dash ?? 0.05, gapSize: opts.gap ?? 0.035, transparent: op < 1, opacity: op });
    setThemedColor(mat, colorVar);
    this.lines = new THREE.LineSegments(geo, mat);
    this.lines.frustumCulled = false;
    this.max = max;
    scene.scene.add(this.lines);
  }

  set(segs: [Vec, Vec][]): void {
    const n = Math.min(segs.length, this.max);
    const attr = this.lines.geometry.getAttribute('position') as THREE.BufferAttribute;
    for (let i = 0; i < n; i++) {
      const [a, b] = segs[i];
      attr.setXYZ(2 * i, a[0], a[1], a[2]);
      attr.setXYZ(2 * i + 1, b[0], b[1], b[2]);
    }
    attr.needsUpdate = true;
    this.lines.geometry.setDrawRange(0, 2 * n);
    this.lines.computeLineDistances();
  }

  setColor(colorVar: string): void { setThemedColor(this.lines.material as THREE.LineDashedMaterial, colorVar); }
  setVisible(v: boolean): void { this.lines.visible = v; }
}

/**
 * A polyline whose points can be replaced (tip trails, arcs). The buffer has a fixed capacity and a
 * draw range, because BufferGeometry.setFromPoints does not grow an existing buffer.
 */
export class Trail {
  readonly line: THREE.Line;
  private max: number;
  private dashed: boolean;
  constructor(scene: Scene3D, colorVar: string, opts: { opacity?: number; max?: number; dashed?: boolean } = {}) {
    const op = opts.opacity ?? 1;
    this.max = opts.max ?? 400;
    this.dashed = !!opts.dashed;
    const mat = this.dashed
      ? new THREE.LineDashedMaterial({ dashSize: 0.05, gapSize: 0.035, transparent: op < 1, opacity: op })
      : new THREE.LineBasicMaterial({ transparent: op < 1, opacity: op });
    setThemedColor(mat, colorVar);
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(this.max * 3), 3));
    geo.setDrawRange(0, 0);
    this.line = new THREE.Line(geo, mat);
    this.line.frustumCulled = false;
    scene.scene.add(this.line);
  }
  set(pts: Vec[]): void {
    const n = Math.min(pts.length, this.max);
    const attr = this.line.geometry.getAttribute('position') as THREE.BufferAttribute;
    for (let i = 0; i < n; i++) attr.setXYZ(i, pts[i][0], pts[i][1], pts[i][2]);
    attr.needsUpdate = true;
    this.line.geometry.setDrawRange(0, n);
    if (this.dashed) this.line.computeLineDistances();
  }
  setVisible(v: boolean): void { this.line.visible = v; }
}

/** A text sprite at a fixed position; returns it so the caller can move or hide it. */
export function label3D(scene: Scene3D, text: string, colorVar: string, scale = 0.8): THREE.Sprite {
  const s = makeTextSprite(text, colorVar, 40);
  s.scale.multiplyScalar(scale);
  scene.scene.add(s);
  return s;
}

export function disposeSprite(s: THREE.Sprite): void {
  (s.material as THREE.SpriteMaterial).map?.dispose();
}
