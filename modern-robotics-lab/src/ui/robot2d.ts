/** Drawing helpers for planar serial arms and 2×2 ellipses on a Canvas2D. */
import { Canvas2D, type Pt, type Style } from './canvas2d';
import { cssVar } from './dom';

export interface PlanarArmStyle {
  linkColor?: string;
  linkWidth?: number;
  jointColor?: string;
  eeColor?: string;
  /** 0–1; use ~0.3 for ghost / reference configurations. */
  alpha?: number;
  /** Draw a small frame at every joint and at the end-effector. */
  frames?: boolean;
  frameSize?: number;
  /** Label joints θ₁, θ₂, … and the end-effector. */
  labels?: boolean;
  /** Show angle arcs at each joint (relative angles). */
  angleArcs?: boolean;
  eeRadius?: number;
}

/**
 * Joint positions of a planar revolute chain. Returns n+1 points: base, joint 2 … joint n, end-effector.
 * θ_i are relative joint angles (radians); base angle offset `base.theta` rotates the whole chain.
 */
export function planarArmPoints(lengths: ArrayLike<number>, thetas: ArrayLike<number>, base: { x: number; y: number; theta?: number } = { x: 0, y: 0 }): Pt[] {
  const pts: Pt[] = [[base.x, base.y]];
  let phi = base.theta ?? 0;
  let x = base.x, y = base.y;
  for (let i = 0; i < lengths.length; i++) {
    phi += thetas[i] ?? 0;
    x += lengths[i] * Math.cos(phi);
    y += lengths[i] * Math.sin(phi);
    pts.push([x, y]);
  }
  return pts;
}

/** Absolute link angles φ_i = Σ_{k≤i} θ_k (plus base angle). */
export function planarArmAngles(thetas: ArrayLike<number>, baseTheta = 0): number[] {
  const out: number[] = [];
  let phi = baseTheta;
  for (let i = 0; i < thetas.length; i++) { phi += thetas[i]; out.push(phi); }
  return out;
}

/** Draw a planar arm; returns the joint/end-effector points for further annotation. */
export function drawPlanarArm(c: Canvas2D, lengths: ArrayLike<number>, thetas: ArrayLike<number>, style: PlanarArmStyle = {}, base: { x: number; y: number; theta?: number } = { x: 0, y: 0 }): Pt[] {
  const pts = planarArmPoints(lengths, thetas, base);
  const phis = planarArmAngles(thetas, base.theta ?? 0);
  const alpha = style.alpha ?? 1;
  const linkColor = style.linkColor ?? cssVar('--fg-muted', '#888');
  const w = style.linkWidth ?? 9;
  // base
  const bx = pts[0][0], by = pts[0][1];
  c.polygon([[bx - 0.12, by - 0.12], [bx + 0.12, by - 0.12], [bx, by]], { fill: cssVar('--grid-strong', '#bbb'), color: cssVar('--grid-strong', '#bbb'), width: 1, alpha });
  c.line([bx - 0.18, by - 0.12], [bx + 0.18, by - 0.12], { color: cssVar('--fg-muted'), width: 2, alpha });
  // links
  for (let i = 0; i < lengths.length; i++) c.line(pts[i], pts[i + 1], { color: linkColor, width: w, cap: 'round', alpha });
  // angle arcs
  if (style.angleArcs) {
    for (let i = 0; i < lengths.length; i++) {
      const a0 = i === 0 ? (base.theta ?? 0) : phis[i - 1];
      const a1 = phis[i];
      const r = Math.min(0.28, lengths[i] * 0.4);
      const col = cssVar('--warn', '#d97706');
      if (Math.abs(a1 - a0) > 1e-3) c.arc(pts[i], r, Math.min(a0, a1), Math.max(a0, a1), { color: col, width: 2, alpha });
      // reference direction (dashed) for the arc
      c.line(pts[i], [pts[i][0] + r * 1.15 * Math.cos(a0), pts[i][1] + r * 1.15 * Math.sin(a0)], { color: col, width: 1, dash: [3, 3], alpha });
      const am = (a0 + a1) / 2;
      c.text([pts[i][0] + (r + 0.12) * Math.cos(am), pts[i][1] + (r + 0.12) * Math.sin(am)], `θ${sub(i + 1)}`, { color: col });
    }
  }
  // joints
  const jc = style.jointColor ?? cssVar('--fg', '#222');
  for (let i = 0; i < lengths.length; i++) {
    c.point(pts[i], w * 0.75, { color: cssVar('--bg-elev', '#fff'), alpha });
    c.circle(pts[i], (w * 0.75) / c.scale, { color: jc, width: 2, alpha });
  }
  // end-effector
  const ee = pts[pts.length - 1];
  c.point(ee, style.eeRadius ?? 6, { color: style.eeColor ?? cssVar('--accent', '#2563eb'), alpha });
  if (style.frames) {
    const fs = style.frameSize ?? 0.25;
    for (let i = 0; i < lengths.length; i++) c.frame({ x: pts[i][0], y: pts[i][1], theta: i === 0 ? (base.theta ?? 0) : phis[i - 1] }, fs, { width: 1.5, alpha: alpha * 0.8 });
    c.frame({ x: ee[0], y: ee[1], theta: phis[phis.length - 1] }, fs, { width: 2, alpha, labels: style.labels ? 'b' : undefined });
  }
  if (style.labels) {
    for (let i = 0; i < lengths.length; i++) c.text(pts[i], `J${sub(i + 1)}`, { dx: -14, dy: 12, color: cssVar('--fg-muted') });
    c.text(ee, 'end-effector', { dx: 0, dy: -14, color: style.eeColor ?? cssVar('--accent') });
  }
  return pts;
}

function sub(n: number): string {
  return String(n).replace(/\d/g, (d) => '₀₁₂₃₄₅₆₇₈₉'[Number(d)]);
}

/** Eigen-decomposition of a symmetric 2×2 matrix [[a, b], [b, d]]. Returns values (descending) and unit vectors. */
export function eig2x2Sym(a: number, b: number, d: number): { values: [number, number]; vectors: [Pt, Pt] } {
  const tr = a + d, det = a * d - b * b;
  const disc = Math.sqrt(Math.max(0, (tr * tr) / 4 - det));
  const l1 = tr / 2 + disc, l2 = tr / 2 - disc;
  let v1: Pt;
  if (Math.abs(b) > 1e-12) v1 = [l1 - d, b];
  else v1 = a >= d ? [1, 0] : [0, 1];
  const n = Math.hypot(v1[0], v1[1]) || 1;
  v1 = [v1[0] / n, v1[1] / n];
  return { values: [l1, l2], vectors: [v1, [-v1[1], v1[0]]] };
}

/**
 * Draw the ellipse { A^{1/2} u : |u| = 1 } for a symmetric positive semidefinite 2×2 matrix A
 * (e.g. J Jᵀ for a manipulability ellipse; semi-axes are √λ_i). `scale` multiplies the axes.
 * Pass `inverse: true` to draw the ellipse of A^{-1} (force ellipse from J Jᵀ).
 */
export function drawEllipse2x2(c: Canvas2D, center: Pt, A: number[][], opts: { scale?: number; inverse?: boolean; axes?: boolean } & Style = {}): { values: [number, number]; vectors: [Pt, Pt] } {
  const e = eig2x2Sym(A[0][0], A[0][1], A[1][1]);
  const s = opts.scale ?? 1;
  let r1 = Math.sqrt(Math.max(0, e.values[0])), r2 = Math.sqrt(Math.max(0, e.values[1]));
  if (opts.inverse) { r1 = r1 > 1e-9 ? 1 / r1 : 1e3; r2 = r2 > 1e-9 ? 1 / r2 : 1e3; }
  const angle = Math.atan2(e.vectors[0][1], e.vectors[0][0]);
  const cap = (v: number) => Math.min(v, 50);
  c.ellipse(center, cap(r1 * s), cap(r2 * s), angle, { color: opts.color, width: opts.width, fill: opts.fill, alpha: opts.alpha, dash: opts.dash });
  if (opts.axes) {
    const col = opts.color ?? cssVar('--accent');
    c.arrow(center, [center[0] + e.vectors[0][0] * cap(r1 * s), center[1] + e.vectors[0][1] * cap(r1 * s)], { color: col, width: 1.5, alpha: opts.alpha });
    c.arrow(center, [center[0] + e.vectors[1][0] * cap(r2 * s), center[1] + e.vectors[1][1] * cap(r2 * s)], { color: col, width: 1.5, alpha: opts.alpha });
  }
  return e;
}

/** Draw a polyline trail (e.g. end-effector path) with fading alpha. */
export function drawTrail(c: Canvas2D, pts: ArrayLike<Pt>, color = cssVar('--warn', '#d97706'), width = 2): void {
  if (pts.length < 2) return;
  c.polyline(pts, { color, width, alpha: 0.9 });
}
