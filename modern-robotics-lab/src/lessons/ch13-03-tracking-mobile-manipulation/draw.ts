/** Top-view drawings of the wheeled bases and the 2R arm (lesson-local wheeled-base renderer). */
import { cssVar, planarArmPoints, type Canvas2D, type Pt } from '@/ui';
import { toWorld, type Pose } from './se2';
import { ARM } from './mobileManip';
import type { BaseKind } from './tracking';

/** Chassis half-length / half-width (m) and wheel placement. */
export const BODY = { hx: 0.25, hy: 0.15, wheelY: 0.2 } as const;

const rect = (q: Pose, cx: number, cy: number, hx: number, hy: number): Pt[] =>
  [[cx - hx, cy - hy], [cx + hx, cy - hy], [cx + hx, cy + hy], [cx - hx, cy + hy]].map((p) => toWorld(q, p as [number, number]));

export interface BaseStyle {
  /** Dashed outline only (reference / comparison robot). */
  ghost?: boolean;
  alpha?: number;
  /** Draw the {b} frame (x red, y green). */
  frame?: boolean;
  frameLabel?: string;
  /** Outline colour (CSS variable name or colour). */
  color?: string;
}

/** Differential drive (two wheels + caster) or omnidirectional base (four mecanum wheels). */
export function drawBase(c: Canvas2D, q: Pose, kind: BaseKind, o: BaseStyle = {}): void {
  const a = o.alpha ?? 1;
  const ghost = !!o.ghost;
  const outline = o.color ?? (ghost ? '--fg-muted' : '--fg');
  const wheel = ghost ? undefined : '--fg-muted';
  const dash = ghost ? [4, 3] : undefined;
  if (kind === 'diff') {
    for (const sy of [-1, 1]) c.polygon(rect(q, 0, sy * BODY.wheelY, 0.085, 0.028), { fill: wheel, color: outline, width: 1, alpha: a, dash });
    c.circle(toWorld(q, [-0.19, 0]), 0.03, { color: outline, width: 1, alpha: a, dash });
  } else {
    for (const sx of [-1, 1]) for (const sy of [-1, 1]) {
      const cx = sx * 0.16, cy = sy * BODY.wheelY;
      c.polygon(rect(q, cx, cy, 0.07, 0.03), { fill: wheel, color: outline, width: 1, alpha: a, dash });
      if (!ghost) {
        // Rollers at ±45°: the classic X pattern seen from above.
        const s = sx * sy;
        for (const dx of [-0.035, 0, 0.035]) c.line(toWorld(q, [cx + dx - 0.02, cy - 0.02 * s]), toWorld(q, [cx + dx + 0.02, cy + 0.02 * s]), { color: '--bg-elev', width: 1, alpha: a });
      }
    }
  }
  c.polygon(rect(q, 0, 0, BODY.hx, BODY.hy), { fill: ghost ? undefined : '--bg-elev', color: outline, width: ghost ? 1.3 : 1.8, alpha: ghost ? a * 0.9 : a, dash });
  // Heading chevron at the front of the chassis.
  c.polyline([toWorld(q, [0.1, 0.09]), toWorld(q, [0.21, 0]), toWorld(q, [0.1, -0.09])], { color: outline, width: ghost ? 1.2 : 2, alpha: a });
  if (o.frame) c.frame({ x: q.x, y: q.y, theta: q.phi }, 0.32, { width: 2, alpha: a, labels: o.frameLabel ? [`x${o.frameLabel}`, `y${o.frameLabel}`] : undefined });
}

/** 2R arm on the base: returns [shoulder, elbow, tip] in world coordinates. */
export function drawArm(c: Canvas2D, q: Pose, theta: [number, number], o: { color?: string; alpha?: number; width?: number; gripper?: boolean } = {}): Pt[] {
  const sh = toWorld(q, [ARM.mount, 0]);
  const pts = planarArmPoints(ARM.L, theta, { x: sh[0], y: sh[1], theta: q.phi });
  const a = o.alpha ?? 1;
  const col = o.color ?? '--fg-muted';
  const w = o.width ?? 7;
  c.line(pts[0], pts[1], { color: col, width: w, alpha: a });
  c.line(pts[1], pts[2], { color: col, width: w * 0.85, alpha: a });
  for (const p of pts.slice(0, 2)) {
    c.point(p, w * 0.7, { color: cssVar('--bg-elev'), alpha: a });
    c.circle(p, (w * 0.7) / c.scale, { color: '--fg', width: 1.5, alpha: a });
  }
  if (o.gripper !== false) {
    // Two fingers along x_e.
    const phi = q.phi + theta[0] + theta[1];
    const ex: Pt = [Math.cos(phi), Math.sin(phi)], ey: Pt = [-Math.sin(phi), Math.cos(phi)];
    const tip = pts[2];
    const f = 0.06, g = 0.045;
    c.line([tip[0] - ey[0] * g, tip[1] - ey[1] * g], [tip[0] + ey[0] * g, tip[1] + ey[1] * g], { color: '--fg', width: 2.2, alpha: a });
    for (const s of [-1, 1]) {
      const b0: Pt = [tip[0] + s * ey[0] * g, tip[1] + s * ey[1] * g];
      c.line(b0, [b0[0] + ex[0] * f, b0[1] + ex[1] * f], { color: '--fg', width: 2.2, alpha: a });
    }
  }
  return pts;
}

/** Arrow with a label next to its head. */
export function labelledArrow(c: Canvas2D, a: Pt, b: Pt, color: string, label: string, o: { width?: number; dx?: number; dy?: number } = {}): void {
  if (Math.hypot(b[0] - a[0], b[1] - a[1]) < 1e-4) return;
  c.arrow(a, b, { color, width: o.width ?? 2.4, head: 9 });
  if (label) c.text(b, label, { color, dx: o.dx ?? 10, dy: o.dy ?? -8, align: 'left', bg: '--sim-bg' });
}
