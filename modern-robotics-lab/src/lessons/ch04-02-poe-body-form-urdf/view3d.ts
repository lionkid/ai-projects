/**
 * Three.js pieces for the PoE lessons (a copy of the space-form lesson's helpers; lessons do not
 * import each other): a stick-figure chain drawn from per-body transforms, screw
 * axis glyphs (solid = where a joint axis is now, dashed = the home axis used in the formula),
 * small labelled frames and a themed ground grid. Every colour is a CSS variable name so a theme
 * switch re-resolves it (see Scene3D); `disposeObject` also frees the label textures.
 */
import * as THREE from 'three';
import { Arrow3D, Frame3D, Link3D, makeTextSprite, setThemedColor, themedStandardMaterial } from '@/ui';
import type { Mat, Vec } from '@/mr';
import type { Segment } from './ur5';
import { axisPlacement, carry } from './body';

const Y = new THREE.Vector3(0, 1, 0);

/**
 * A camera-facing text label `height` metres tall (40 px font → 56 px canvas). Its canvas texture
 * is freed by `disposeObject` (Scene3D.destroy frees geometries and materials, not textures).
 */
export function textSprite(text: string, color: string, height: number): THREE.Sprite {
  return makeTextSprite(text, color, 40, height / 56);
}

/** Dispose every geometry / material / texture under `root` and detach it. */
export function disposeObject(root: THREE.Object3D): void {
  root.traverse((o) => {
    const mesh = o as THREE.Mesh;
    mesh.geometry?.dispose();
    const mat = mesh.material as THREE.Material | THREE.Material[] | undefined;
    const mats = Array.isArray(mat) ? mat : mat ? [mat] : [];
    for (const m of mats) {
      (m as THREE.Material & { map?: THREE.Texture | null }).map?.dispose();
      m.dispose();
    }
  });
  root.parent?.remove(root);
}

/** Make every material under `root` (semi-)transparent. */
export function setOpacity(root: THREE.Object3D, op: number): void {
  root.traverse((o) => {
    const mat = (o as THREE.Mesh).material as THREE.Material | THREE.Material[] | undefined;
    const mats = Array.isArray(mat) ? mat : mat ? [mat] : [];
    for (const m of mats) {
      if (m.transparent !== op < 1) m.needsUpdate = true;
      m.transparent = op < 1;
      m.opacity = op;
      if (!(m instanceof THREE.SpriteMaterial)) m.depthWrite = op >= 1;
    }
  });
}

/** A themed ground grid in the xy-plane (Scene3D's own grid has a fixed size). */
export function makeGrid(size: number, divisions: number): THREE.GridHelper {
  const g = new THREE.GridHelper(size, divisions, 0xffffff, 0xffffff);
  g.rotation.x = Math.PI / 2;
  const mat = g.material as THREE.LineBasicMaterial;
  setThemedColor(mat, '--grid-strong');
  mat.transparent = true;
  mat.opacity = 0.75;
  return g;
}

/** A coordinate frame (x red, y green, z blue) with a small label. */
export function makeFrame(size: number, label: string, opts: { opacity?: number; labelColor?: string; labelHeight?: number } = {}): Frame3D {
  const f = new Frame3D({ size, opacity: opts.opacity ?? 1, thickness: size * 0.045 });
  if (label) {
    const s = textSprite(label, opts.labelColor ?? '--fg', opts.labelHeight ?? size * 0.32);
    s.position.set(-size * 0.3, -size * 0.3, size * 0.32);
    if ((opts.opacity ?? 1) < 1) (s.material as THREE.SpriteMaterial).opacity = Math.max(0.45, opts.opacity ?? 1);
    f.group.add(s);
  }
  return f;
}

export interface AxisGlyphOpts {
  color: string;
  dashed: boolean;
  /** Extent behind / in front of the glyph point (m). */
  back: number;
  front: number;
  radius: number;
  /** Arrow length at the front end (m). */
  arrow: number;
  label?: string;
  labelHeight?: number;
  /** Draw a small ring (revolute joints). */
  ring?: number;
  opacity?: number;
  /** Dash and gap lengths (m) of a dashed glyph (default 7 and 5 radii). */
  dash?: number;
  gap?: number;
  /** Put the label at the back end instead of beside the arrow (avoids clashes with a coincident glyph). */
  labelAtBack?: boolean;
}

/**
 * A joint axis glyph: a line along the axis (solid tube or dashes), an arrow at the front end, an
 * optional ring for revolute joints and a label. The group's local +Y is the axis direction.
 */
export class AxisGlyph {
  readonly group = new THREE.Group();
  /** Line, arrow, ring and marks (scaled by `emphasize`); the label stays outside so it is never stretched. */
  private body = new THREE.Group();
  private ring?: THREE.Mesh;
  private label?: THREE.Sprite;
  private prismaticMark: THREE.Group;

  constructor(o: AxisGlyphOpts) {
    const op = o.opacity ?? 1;
    const mat = themedStandardMaterial(o.color, { roughness: 0.6, transparent: op < 1, opacity: op });
    const len = o.back + o.front;
    if (o.dashed) {
      const dash = o.dash ?? o.radius * 7, gap = o.gap ?? o.radius * 5;
      const geo = new THREE.CylinderGeometry(o.radius, o.radius, dash, 8);
      for (let y = -o.back; y + dash <= o.front - o.arrow * 0.6; y += dash + gap) {
        const m = new THREE.Mesh(geo, mat);
        m.position.y = y + dash / 2;
        this.body.add(m);
      }
    } else {
      const shaft = len - o.arrow * 0.6;
      const m = new THREE.Mesh(new THREE.CylinderGeometry(o.radius, o.radius, shaft, 10), mat);
      m.position.y = -o.back + shaft / 2;
      this.body.add(m);
    }
    const arrow = new Arrow3D(o.color, o.radius * 1.25, o.arrow * 0.75, op);
    arrow.setLength(o.arrow);
    arrow.group.position.y = o.front - o.arrow;
    this.body.add(arrow.group);
    if (o.ring) {
      this.ring = new THREE.Mesh(new THREE.TorusGeometry(o.ring, o.radius * 0.9, 8, 32), mat);
      this.ring.rotation.x = Math.PI / 2;
      this.body.add(this.ring);
    }
    // two short bars across the line mark a prismatic (sliding) axis
    this.prismaticMark = new THREE.Group();
    for (const y of [-0.25, 0.25]) {
      const bar = new THREE.Mesh(new THREE.BoxGeometry(o.radius * 9, o.radius * 1.6, o.radius * 9), mat);
      bar.position.y = y * o.front;
      this.prismaticMark.add(bar);
    }
    this.prismaticMark.visible = false;
    this.body.add(this.prismaticMark);
    this.group.add(this.body);
    if (o.label) {
      this.label = textSprite(o.label, o.color, o.labelHeight ?? 0.05);
      const h = (o.labelHeight ?? 0.05) * 0.55;
      this.label.position.set(0, o.labelAtBack ? -o.back - h : o.front + h, 0);
      this.group.add(this.label);
    }
  }

  /** Place the glyph for the screw axis S (in {s}) of a joint located near `anchor`. */
  set(S: Vec, anchor: Vec): void {
    const { dir, point, prismatic } = axisPlacement(S, anchor);
    this.group.position.set(point[0], point[1], point[2]);
    this.group.quaternion.setFromUnitVectors(Y, new THREE.Vector3(dir[0], dir[1], dir[2]));
    if (this.ring) this.ring.visible = !prismatic;
    this.prismaticMark.visible = prismatic;
  }

  setVisible(v: boolean): void { this.group.visible = v; }

  /** Thicken (k > 1) or restore (k = 1) the glyph. */
  emphasize(k: number): void { this.body.scale.set(k, 1, k); }
}

export interface ChainViewOpts {
  opacity?: number;
  linkColor?: string;
  jointColor?: string;
  baseColor?: string;
}

/** Stick figure of a chain: each segment follows the transform of the body it is attached to. */
export class ChainView {
  readonly group = new THREE.Group();
  private items: { seg: Segment; link: Link3D }[] = [];
  private colors: Record<Segment['role'], string>;
  private baseOpacity: number;

  constructor(segments: Segment[], o: ChainViewOpts = {}) {
    const op = o.opacity ?? 1;
    this.baseOpacity = op;
    this.colors = { link: o.linkColor ?? '--fg-muted', joint: o.jointColor ?? '--fg', base: o.baseColor ?? '--grid-strong' };
    for (const seg of segments) {
      const link = new Link3D(seg.r, this.colors[seg.role], op);
      if (op < 1) (link.mesh.material as THREE.Material).depthWrite = false;
      this.items.push({ seg, link });
      this.group.add(link.mesh);
    }
  }

  /** `B[body]` moves the home geometry of that body to its current pose (B[0] = ground). */
  setBodies(B: Mat[]): void {
    for (const { seg, link } of this.items) {
      const T = B[seg.body] ?? B[B.length - 1];
      link.setEndpoints(carry(T, seg.a), carry(T, seg.b));
    }
  }

  /**
   * Tint the links of one body (null restores the default colours). `opacity` < 1 makes the tinted
   * links see-through, e.g. to reveal a frame drawn inside a link.
   */
  highlight(body: number | null, color = '--accent', opacity = 1): void {
    for (const { seg, link } of this.items) {
      const on = body !== null && seg.body === body && seg.role !== 'base';
      link.setColor(on ? color : this.colors[seg.role]);
      const m = link.mesh.material as THREE.MeshStandardMaterial;
      const op = on ? opacity : this.baseOpacity;
      if (m.opacity !== op) {
        if (m.transparent !== op < 1) m.needsUpdate = true;
        m.transparent = op < 1;
        m.opacity = op;
        m.depthWrite = op >= 1;
      }
    }
  }

  setVisible(v: boolean): void { this.group.visible = v; }
}
