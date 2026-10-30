/** Stick-figure renderer for PoE serial chains (M, Slist) inside a Scene3D. */
import * as THREE from 'three';
import type { Mat, Vec } from '@/mr';
import { spaceJointTransforms, Adjoint, matMul, matVec, getCol, cross, dot, norm } from '@/mr';
import { Scene3D, Frame3D, Link3D, ScrewAxis3D, vec3, themedStandardMaterial } from './scene3d';

/**
 * Home-configuration "joint points" derived from the screw axes alone: p_1 is the point on axis 1
 * closest to the origin, and p_i is the projection of p_{i-1} onto axis i. For a prismatic joint
 * (ω = 0) the point is simply p_{i-1}. This reproduces the natural stick figure for planar arms and
 * for the UR5 parameters used in the book.
 */
export function homeJointPoints(Slist: Mat): Vec[] {
  const n = Slist[0].length;
  const pts: Vec[] = [];
  let prev: Vec = [0, 0, 0];
  for (let i = 0; i < n; i++) {
    const S = getCol(Slist, i);
    const w = [S[0], S[1], S[2]], v = [S[3], S[4], S[5]];
    const wn = norm(w);
    let p: Vec;
    if (wn < 1e-9) p = prev;
    else {
      const wh = w.map((x) => x / wn);
      const q = cross(wh, v).map((x) => x / wn); // a point on the axis (closest to origin)
      const d = [prev[0] - q[0], prev[1] - q[1], prev[2] - q[2]];
      const t = dot(d, wh);
      p = [q[0] + wh[0] * t, q[1] + wh[1] * t, q[2] + wh[2] * t];
    }
    pts.push(p);
    prev = p;
  }
  return pts;
}

export interface Robot3DOptions {
  linkRadius?: number;
  linkColor?: THREE.ColorRepresentation;
  jointColor?: THREE.ColorRepresentation;
  /** Draw a frame {i} at every joint (orientation of link i). */
  showJointFrames?: boolean;
  /** Draw the current screw axis of every joint. */
  showAxes?: boolean;
  frameSize?: number;
  /** Draw the end-effector frame {b} (default true). */
  eeFrame?: boolean;
  eeLabel?: string;
  opacity?: number;
  /** Override the home joint points (one per joint, in {s}). */
  jointPoints?: Vec[];
  axisLength?: number;
}

export class Robot3D {
  readonly group = new THREE.Group();
  readonly eeFrame: Frame3D;
  readonly jointFrames: Frame3D[] = [];
  readonly axes: ScrewAxis3D[] = [];
  private links: Link3D[] = [];
  private joints: THREE.Mesh[] = [];
  private homePts: Vec[];
  private M: Mat;
  private Slist: Mat;
  private n: number;
  private scene: Scene3D;
  lastT: Mat;

  constructor(scene: Scene3D, model: { M: Mat; Slist: Mat }, opts: Robot3DOptions = {}) {
    this.scene = scene;
    this.M = model.M; this.Slist = model.Slist;
    this.n = model.Slist[0].length;
    this.homePts = opts.jointPoints ?? homeJointPoints(model.Slist);
    const op = opts.opacity ?? 1;
    const linkColor = opts.linkColor ?? '--fg-muted';
    const jointColor = opts.jointColor ?? '--fg';
    const r = opts.linkRadius ?? 0.03;
    for (let i = 0; i < this.n; i++) {
      const l = new Link3D(r, linkColor, op);
      this.links.push(l); this.group.add(l.mesh);
      const j = new THREE.Mesh(new THREE.SphereGeometry(r * 1.6, 16, 12), themedStandardMaterial(jointColor, { transparent: op < 1, opacity: op, roughness: 0.4 }));
      this.joints.push(j); this.group.add(j);
      if (opts.showJointFrames) { const f = new Frame3D({ size: (opts.frameSize ?? 0.25) * 0.8, opacity: op, label: `{${i + 1}}` }); this.jointFrames.push(f); this.group.add(f.group); }
      if (opts.showAxes) { const a = new ScrewAxis3D({ length: opts.axisLength ?? 0.6 }); this.axes.push(a); this.group.add(a.group); }
    }
    this.eeFrame = new Frame3D({ size: opts.frameSize ?? 0.25, opacity: op, label: opts.eeLabel ?? '{b}' });
    this.eeFrame.setVisible(opts.eeFrame !== false);
    this.group.add(this.eeFrame.group);
    scene.scene.add(this.group);
    this.lastT = model.M;
    this.setConfig(new Array(this.n).fill(0));
  }

  /** Update the pose for joint values θ. Returns the end-effector transform and the current joint points. */
  setConfig(thetas: Vec): { T: Mat; points: Vec[]; P: Mat[] } {
    const P = spaceJointTransforms(this.Slist, thetas);
    const T = matMul(P[this.n - 1], this.M);
    const pts: Vec[] = [];
    for (let i = 0; i < this.n; i++) {
      const hp = this.homePts[i];
      const p = matVec(P[i], [hp[0], hp[1], hp[2], 1]).slice(0, 3);
      pts.push(p);
    }
    const ee = [T[0][3], T[1][3], T[2][3]];
    for (let i = 0; i < this.n; i++) {
      const a = pts[i], b = i + 1 < this.n ? pts[i + 1] : ee;
      this.links[i].setEndpoints(a, b);
      this.joints[i].position.copy(vec3(a));
      if (this.jointFrames[i]) {
        const F = P[i].map((row) => row.slice());
        F[0][3] = a[0]; F[1][3] = a[1]; F[2][3] = a[2];
        this.jointFrames[i].setTransform(F);
      }
      if (this.axes[i]) {
        const S = getCol(this.Slist, i);
        const Sc = i === 0 ? S : matVec(Adjoint(P[i - 1]), S);
        this.axes[i].setScrew(Sc);
      }
    }
    this.eeFrame.setTransform(T);
    this.lastT = T;
    this.scene.requestRender();
    return { T, points: pts, P };
  }

  setVisible(v: boolean): void { this.group.visible = v; }
  showAxes(v: boolean): void { this.axes.forEach((a) => a.setVisible(v)); }
  showJointFrames(v: boolean): void { this.jointFrames.forEach((f) => f.setVisible(v)); }
  destroy(): void { this.scene.scene.remove(this.group); }
}
