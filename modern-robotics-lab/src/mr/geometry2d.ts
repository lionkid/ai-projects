/**
 * Planar geometry predicates for collision checking in the planning / trajectory lessons.
 * Points are [x, y]; polygons are arrays of vertices (either winding, implicitly closed).
 */
export type P2 = [number, number];
export type Polygon2 = P2[];

export function closestPointOnSegment(a: P2, b: P2, p: P2): P2 {
  const dx = b[0] - a[0], dy = b[1] - a[1];
  const L2 = dx * dx + dy * dy;
  if (L2 < 1e-18) return [a[0], a[1]];
  const t = Math.max(0, Math.min(1, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L2));
  return [a[0] + t * dx, a[1] + t * dy];
}

export function segPointDistance(a: P2, b: P2, p: P2): number {
  const q = closestPointOnSegment(a, b, p);
  return Math.hypot(p[0] - q[0], p[1] - q[1]);
}

export function segCircleIntersects(a: P2, b: P2, c: P2, r: number): boolean {
  return segPointDistance(a, b, c) <= r;
}

export function circleCircleIntersects(c1: P2, r1: number, c2: P2, r2: number): boolean {
  return Math.hypot(c1[0] - c2[0], c1[1] - c2[1]) <= r1 + r2;
}

function orient(a: P2, b: P2, c: P2): number {
  return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
}
function onSeg(a: P2, b: P2, p: P2): boolean {
  return Math.min(a[0], b[0]) - 1e-12 <= p[0] && p[0] <= Math.max(a[0], b[0]) + 1e-12 &&
    Math.min(a[1], b[1]) - 1e-12 <= p[1] && p[1] <= Math.max(a[1], b[1]) + 1e-12;
}

/** Segment–segment intersection including touching / collinear overlap. */
export function segSegIntersects(a: P2, b: P2, c: P2, d: P2): boolean {
  const o1 = orient(a, b, c), o2 = orient(a, b, d), o3 = orient(c, d, a), o4 = orient(c, d, b);
  if (((o1 > 0 && o2 < 0) || (o1 < 0 && o2 > 0)) && ((o3 > 0 && o4 < 0) || (o3 < 0 && o4 > 0))) return true;
  if (Math.abs(o1) < 1e-12 && onSeg(a, b, c)) return true;
  if (Math.abs(o2) < 1e-12 && onSeg(a, b, d)) return true;
  if (Math.abs(o3) < 1e-12 && onSeg(c, d, a)) return true;
  if (Math.abs(o4) < 1e-12 && onSeg(c, d, b)) return true;
  return false;
}

/** Ray-casting point-in-polygon (boundary counts as inside). */
export function pointInPolygon(p: P2, poly: Polygon2): boolean {
  let inside = false;
  const n = poly.length;
  for (let i = 0, j = n - 1; i < n; j = i++) {
    const a = poly[i], b = poly[j];
    if (segPointDistance(a, b, p) < 1e-12) return true;
    const cross = (a[1] > p[1]) !== (b[1] > p[1]) && p[0] < ((b[0] - a[0]) * (p[1] - a[1])) / (b[1] - a[1]) + a[0];
    if (cross) inside = !inside;
  }
  return inside;
}

export function distPointPolygon(p: P2, poly: Polygon2): number {
  if (pointInPolygon(p, poly)) return 0;
  let best = Infinity;
  for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) best = Math.min(best, segPointDistance(poly[j], poly[i], p));
  return best;
}

/** Segment intersects polygon (crosses an edge or lies inside). */
export function segPolygonIntersects(a: P2, b: P2, poly: Polygon2): boolean {
  if (pointInPolygon(a, poly) || pointInPolygon(b, poly)) return true;
  for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) if (segSegIntersects(a, b, poly[j], poly[i])) return true;
  return false;
}

export function circlePolygonIntersects(c: P2, r: number, poly: Polygon2): boolean {
  return distPointPolygon(c, poly) <= r;
}

/** Polyline (e.g. a robot arm) against a disk obstacle. */
export function polylineCircleIntersects(pts: P2[], c: P2, r: number): boolean {
  for (let i = 0; i + 1 < pts.length; i++) if (segCircleIntersects(pts[i], pts[i + 1], c, r)) return true;
  return false;
}

/** Polyline against a polygon obstacle. */
export function polylinePolygonIntersects(pts: P2[], poly: Polygon2): boolean {
  for (let i = 0; i + 1 < pts.length; i++) if (segPolygonIntersects(pts[i], pts[i + 1], poly)) return true;
  return false;
}

/** Minimum distance from a polyline to a disk boundary (negative when penetrating). */
export function polylineCircleClearance(pts: P2[], c: P2, r: number): number {
  let d = Infinity;
  for (let i = 0; i + 1 < pts.length; i++) d = Math.min(d, segPointDistance(pts[i], pts[i + 1], c));
  return d - r;
}

export function aabb(poly: Polygon2): { xmin: number; xmax: number; ymin: number; ymax: number } {
  let xmin = Infinity, xmax = -Infinity, ymin = Infinity, ymax = -Infinity;
  for (const [x, y] of poly) { xmin = Math.min(xmin, x); xmax = Math.max(xmax, x); ymin = Math.min(ymin, y); ymax = Math.max(ymax, y); }
  return { xmin, xmax, ymin, ymax };
}

/** Axis-aligned rectangle as a polygon. */
export function rectPolygon(x0: number, y0: number, x1: number, y1: number): Polygon2 {
  return [[x0, y0], [x1, y0], [x1, y1], [x0, y1]];
}

/** Regular polygon (for round-ish obstacles drawn as polygons). */
export function regularPolygon(c: P2, r: number, n = 12, phase = 0): Polygon2 {
  return Array.from({ length: n }, (_, k) => { const a = phase + (2 * Math.PI * k) / n; return [c[0] + r * Math.cos(a), c[1] + r * Math.sin(a)] as P2; });
}
