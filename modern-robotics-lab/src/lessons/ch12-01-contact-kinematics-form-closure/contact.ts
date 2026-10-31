/**
 * Planar contact kinematics for frictionless point contacts on polygonal bodies:
 * contact normal wrenches, first-order contact classification, centre-of-rotation (CoR)
 * regions, form-closure tests (combinatorial + LP certificate), a tiny dense LP solver and
 * an automatic 4-contact form-closure search.
 *
 * Conventions (same as the planar helpers in @/mr):
 *   planar twist  V = [ω, vx, vy]   (angular part first)
 *   planar wrench F = [mz, fx, fy]  (moment first)
 * Both are expressed in the fixed frame {s}. The body sits at its reference pose with its area
 * centroid at the origin of {s}, so wrench moments are taken about the centroid.
 * Contact normals n̂ are unit vectors pointing INTO the body (the direction a finger can push).
 */
import { MatrixExp6, VecTose3, matVec, pinv, svd } from '@/mr';

export type P2 = [number, number];
export type V3 = [number, number, number];

// ---------------------------------------------------------------- small vector helpers
export const add2 = (a: P2, b: P2): P2 => [a[0] + b[0], a[1] + b[1]];
export const sub2 = (a: P2, b: P2): P2 => [a[0] - b[0], a[1] - b[1]];
export const scale2 = (a: P2, s: number): P2 => [a[0] * s, a[1] * s];
export const dot2 = (a: P2, b: P2): number => a[0] * b[0] + a[1] * b[1];
/** z-component of the planar cross product a × b. */
export const cross2 = (a: P2, b: P2): number => a[0] * b[1] - a[1] * b[0];
export const norm2 = (a: P2): number => Math.hypot(a[0], a[1]);
/** Rotate by +90° (counter-clockwise). */
export const perp = (a: P2): P2 => [-a[1], a[0]];
export const unit2 = (a: P2): P2 => { const L = norm2(a) || 1; return [a[0] / L, a[1] / L]; };

export const dot3 = (a: ArrayLike<number>, b: ArrayLike<number>): number => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
export const cross3 = (a: ArrayLike<number>, b: ArrayLike<number>): V3 => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];
export const sub3 = (a: ArrayLike<number>, b: ArrayLike<number>): V3 => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
export const scale3 = (a: ArrayLike<number>, s: number): V3 => [a[0] * s, a[1] * s, a[2] * s];
export const norm3 = (a: ArrayLike<number>): number => Math.hypot(a[0], a[1], a[2]);

// ---------------------------------------------------------------- shapes
export interface PolyShape {
  key: string;
  name: string;
  /** CCW vertices with the area centroid at the origin. */
  verts: P2[];
  /** Centroid of the vertices as given (subtract it to convert given coordinates to `verts` coordinates). */
  offset: P2;
}

export function signedArea(verts: P2[]): number {
  let a = 0;
  for (let i = 0; i < verts.length; i++) a += cross2(verts[i], verts[(i + 1) % verts.length]);
  return a / 2;
}

export function polygonCentroid(verts: P2[]): P2 {
  let cx = 0, cy = 0, a = 0;
  for (let i = 0; i < verts.length; i++) {
    const p = verts[i], q = verts[(i + 1) % verts.length];
    const w = cross2(p, q);
    a += w; cx += (p[0] + q[0]) * w; cy += (p[1] + q[1]) * w;
  }
  return [cx / (3 * a), cy / (3 * a)];
}

/** Polygon with CCW vertex order whose area centroid is moved to the origin. */
export function makeShape(key: string, name: string, verts: P2[]): PolyShape {
  let v = verts.map((p) => [p[0], p[1]] as P2);
  if (signedArea(v) < 0) v = v.reverse();
  const c = polygonCentroid(v);
  return { key, name, verts: v.map((p) => sub2(p, c)), offset: c };
}

/** Largest distance from the centroid (origin) to a vertex: a characteristic length ρ. */
export function shapeRadius(verts: P2[]): number {
  return Math.max(...verts.map(norm2));
}

/** Ray-casting point-in-polygon test (boundary counts as inside). */
export function insidePolygon(q: P2, verts: P2[]): boolean {
  let inside = false;
  for (let i = 0, j = verts.length - 1; i < verts.length; j = i++) {
    const a = verts[i], b = verts[j];
    if ((a[1] > q[1]) !== (b[1] > q[1]) && q[0] < ((b[0] - a[0]) * (q[1] - a[1])) / (b[1] - a[1]) + a[0]) inside = !inside;
  }
  return inside;
}

// ---------------------------------------------------------------- boundary parameterisation
export interface BoundaryPoint {
  /** Arc length from vertex 0 (CCW). */
  s: number;
  p: P2;
  /** Unit inward normal. */
  n: P2;
  edge: number;
}

interface Edge { a: P2; L: number; u: P2; n: P2; s0: number }

function edges(verts: P2[]): Edge[] {
  const out: Edge[] = [];
  let s0 = 0;
  for (let k = 0; k < verts.length; k++) {
    const a = verts[k], b = verts[(k + 1) % verts.length];
    const d = sub2(b, a);
    const L = norm2(d);
    const u = scale2(d, 1 / L);
    out.push({ a, L, u, n: perp(u), s0 }); // CCW polygon: the interior lies to the left of each edge
    s0 += L;
  }
  return out;
}

export function perimeter(verts: P2[]): number {
  return edges(verts).reduce((s, e) => s + e.L, 0);
}

/**
 * Boundary point at arc length `s` (taken modulo the perimeter). Points closer than `margin` to a
 * vertex are pushed back onto the edge, because the normal is undefined at a corner.
 */
export function boundaryAt(verts: P2[], s: number, margin = 0.06): BoundaryPoint {
  const E = edges(verts);
  const P = E.reduce((acc, e) => acc + e.L, 0);
  let r = ((s % P) + P) % P;
  for (let k = 0; k < E.length; k++) {
    const e = E[k];
    if (r <= e.L || k === E.length - 1) {
      const m = Math.min(margin, e.L / 3);
      const t = Math.min(Math.max(r, m), e.L - m);
      return { s: e.s0 + t, p: add2(e.a, scale2(e.u, t)), n: e.n, edge: k };
    }
    r -= e.L;
  }
  /* c8 ignore next */
  throw new Error('boundaryAt: empty polygon');
}

/** Closest boundary point to q (kept `margin` away from the corners). */
export function projectToBoundary(verts: P2[], q: P2, margin = 0.06): BoundaryPoint {
  let best: BoundaryPoint | null = null;
  let bestD = Infinity;
  edges(verts).forEach((e, k) => {
    const m = Math.min(margin, e.L / 3);
    const t = Math.min(Math.max(dot2(sub2(q, e.a), e.u), m), e.L - m);
    const p = add2(e.a, scale2(e.u, t));
    const d = norm2(sub2(q, p));
    if (d < bestD) { bestD = d; best = { s: e.s0 + t, p, n: e.n, edge: k }; }
  });
  return best!;
}

// ---------------------------------------------------------------- twists, wrenches, contacts
/** Wrench of a unit force along `f` applied at point p: [p × f, fx, fy]. */
export function forceWrench(p: P2, f: P2): V3 {
  return [cross2(p, f), f[0], f[1]];
}

/** Wrench of a unit normal force at a frictionless contact. */
export function contactWrench(p: P2, n: P2): V3 {
  return forceWrench(p, n);
}

/** Planar twist of a rotation about point c with angular velocity ω. */
export function twistAbout(c: P2, omega: number): V3 {
  return [omega, omega * c[1], -omega * c[0]];
}

/** Planar twist of a pure translation with velocity v. */
export function translationTwist(v: P2): V3 {
  return [0, v[0], v[1]];
}

/** Centre of rotation of a planar twist, or null for a (near) pure translation. */
export function corOf(V: V3, eps = 1e-9): P2 | null {
  if (Math.abs(V[0]) <= eps * Math.max(1, Math.hypot(V[1], V[2]))) return null;
  return [-V[2] / V[0], V[1] / V[0]];
}

/** Velocity of the body point that is currently at p: v + ω × p. */
export function pointVelocity(V: V3, p: P2): P2 {
  return [V[1] - V[0] * p[1], V[2] + V[0] * p[0]];
}

export type ContactMode = 'rolling' | 'sliding' | 'breaking' | 'penetrating';

export interface ContactState {
  /** Fᵀ V for the body twist alone (finger at rest). */
  power: number;
  /** n̂ᵀ(ṗ_body − ṗ_finger): positive separates, negative penetrates. */
  normalRate: number;
  /** Relative velocity of the body point w.r.t. the finger point. */
  rel: P2;
  mode: ContactMode;
}

/**
 * First-order classification of one frictionless contact for a body moving with twist V while the
 * finger's contact point moves with velocity vFinger (zero for a fixed finger).
 */
export function classifyContact(p: P2, n: P2, V: V3, vFinger: P2 = [0, 0], tol = 1e-7): ContactState {
  const power = dot3(contactWrench(p, n), V);
  const rel = sub2(pointVelocity(V, p), vFinger);
  const normalRate = dot2(n, rel);
  let mode: ContactMode;
  if (normalRate > tol) mode = 'breaking';
  else if (normalRate < -tol) mode = 'penetrating';
  else mode = norm2(rel) <= tol ? 'rolling' : 'sliding';
  return { power, normalRate, rel, mode };
}

/** Rigid displacement exp([V] s) of the plane, computed with the SE(3) exponential from @/mr. */
export function planarExp(V: V3, s: number): { x: number; y: number; theta: number } {
  const T = MatrixExp6(VecTose3([0, 0, V[0] * s, V[1] * s, V[2] * s, 0]));
  return { x: T[0][3], y: T[1][3], theta: Math.atan2(T[1][0], T[0][0]) };
}

export function applyPose(pose: { x: number; y: number; theta: number }, q: P2): P2 {
  const c = Math.cos(pose.theta), s = Math.sin(pose.theta);
  return [c * q[0] - s * q[1] + pose.x, s * q[0] + c * q[1] + pose.y];
}

/** Parameter s so that exp([V]s) moves the farthest of `pts` by about `dist`. */
export function displacementScale(V: V3, pts: P2[], dist: number): number {
  let vmax = 0;
  for (const q of pts) vmax = Math.max(vmax, norm2(pointVelocity(V, q)));
  return vmax > 1e-12 ? dist / vmax : 0;
}

// ---------------------------------------------------------------- feasible twist cone
/** Numerical rank of a set of planar wrenches (rank of the 3 × j matrix with them as columns). */
export function wrenchRank(Fs: V3[], rtol = 1e-9): number {
  if (!Fs.length) return 0;
  const A = [0, 1, 2].map((r) => Fs.map((F) => F[r]));
  const { S } = svd(A);
  const smax = S[0] ?? 0;
  if (smax <= 1e-14) return 0;
  return S.filter((x) => x > rtol * smax).length;
}

/**
 * Candidate extreme rays of the cone {V : Fᵢᵀ V ≥ 0 ∀i}. For rank 3 every extreme ray is
 * ±(Fᵢ × Fⱼ) for some independent pair; for rank ≤ 2 the null space of the wrenches is added.
 * All candidates have unit length.
 */
export function twistCandidates(Fs: V3[]): V3[] {
  const out: V3[] = [];
  for (let i = 0; i < Fs.length; i++) {
    for (let j = i + 1; j < Fs.length; j++) {
      const c = cross3(Fs[i], Fs[j]);
      const L = norm3(c);
      if (L > 1e-12 * (norm3(Fs[i]) * norm3(Fs[j]))) { out.push(scale3(c, 1 / L), scale3(c, -1 / L)); }
    }
  }
  if (out.length) return out;
  // rank ≤ 1: every wrench is parallel to F0 (or there are none)
  const F0 = Fs.find((F) => norm3(F) > 1e-12);
  if (!F0) return [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]];
  const basis: V3[] = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
  let e = basis[0];
  let best = Infinity;
  for (const b of basis) { const d = Math.abs(dot3(b, F0)); if (d < best) { best = d; e = b; } }
  const u = cross3(F0, e); const uL = norm3(u);
  const w = cross3(F0, u); const wL = norm3(w);
  const f = scale3(F0, 1 / norm3(F0));
  return [scale3(u, 1 / uL), scale3(u, -1 / uL), scale3(w, 1 / wL), scale3(w, -1 / wL), f];
}

export function isFeasibleTwist(Fs: V3[], V: V3, tol = 1e-9): boolean {
  return Fs.every((F) => dot3(F, V) >= -tol * norm3(F));
}

/** A nonzero twist allowed by every contact (Fᵢᵀ V ≥ 0), or null if only V = 0 remains. */
export function escapeTwist(Fs: V3[], tol = 1e-9): V3 | null {
  for (const V of twistCandidates(Fs)) if (isFeasibleTwist(Fs, V, tol)) return V;
  return null;
}

/** First-order form closure: the only twist compatible with all contacts is zero. */
export function isFormClosure(Fs: V3[], tol = 1e-9): boolean {
  return escapeTwist(Fs, tol) === null;
}

/** All feasible candidate rays (unit, de-duplicated). */
export function feasibleRays(Fs: V3[], tol = 1e-9): V3[] {
  const out: V3[] = [];
  for (const V of twistCandidates(Fs)) {
    if (!isFeasibleTwist(Fs, V, tol)) continue;
    if (out.some((U) => dot3(U, V) > 1 - 1e-9)) continue;
    out.push(V);
  }
  return out;
}

/**
 * A representative escape twist that lies inside the feasible cone (sum of its feasible extreme rays),
 * normalised so that |ω| = 1 for rotations or |v| = 1 for translations. Null under form closure.
 */
export function representativeEscape(Fs: V3[], tol = 1e-9): V3 | null {
  const rays = feasibleRays(Fs, tol);
  if (!rays.length) return null;
  let V: V3 = [0, 0, 0];
  for (const r of rays) V = [V[0] + r[0], V[1] + r[1], V[2] + r[2]];
  if (norm3(V) < 1e-6 || !isFeasibleTwist(Fs, V, tol)) V = rays[0];
  const w = Math.abs(V[0]);
  const v = Math.hypot(V[1], V[2]);
  const s = w > 1e-9 * Math.max(1, v) ? w : v;
  return scale3(V, 1 / s);
}

/**
 * An escape twist that is easy to describe in words: a pure translation (|v| = 1) whenever the
 * contacts allow one (the middle of the widest allowed sector of directions), otherwise
 * `representativeEscape`. Null under form closure.
 */
export function preferredEscape(Fs: V3[], tol = 1e-9): V3 | null {
  // For V = (0, v): Fᵢᵀ V = (fx, fy)ᵢ · v, so the allowed translations form the cone of `translationCone`.
  const tc = translationCone(Fs.map((F) => [F[1], F[2]] as P2), tol);
  if (tc.kind === 'all') return [0, 0, 1];
  if (tc.kind === 'arcs') {
    let a: number | null = null;
    let widest = -1;
    for (const [a0, a1] of tc.arcs) if (a1 - a0 > widest) { widest = a1 - a0; a = (a0 + a1) / 2; }
    if (a === null && tc.rays.length) a = tc.rays[0];
    if (a !== null) return [0, Math.cos(a), Math.sin(a)];
  }
  return representativeEscape(Fs, tol);
}

// ---------------------------------------------------------------- convex hull in wrench space
export interface Facet { normal: V3; offset: number; idx: number[] }

/**
 * Supporting planes of conv(points): outward unit normal N with N·x ≤ offset for every point.
 * Brute force over triples (O(n⁴)), fine for the ≤ 20 points used here.
 */
export function hullFacets(pts: V3[], eps = 1e-9): Facet[] {
  const n = pts.length;
  const out: Facet[] = [];
  const addFacet = (N: V3, d: number) => {
    if (out.some((f) => dot3(f.normal, N) > 1 - 1e-9 && Math.abs(f.offset - d) < 1e-7)) return;
    const idx: number[] = [];
    for (let l = 0; l < n; l++) if (Math.abs(dot3(N, pts[l]) - d) <= 1e-7) idx.push(l);
    out.push({ normal: N, offset: d, idx });
  };
  for (let i = 0; i < n; i++) {
    for (let j = i + 1; j < n; j++) {
      for (let k = j + 1; k < n; k++) {
        let N = cross3(sub3(pts[j], pts[i]), sub3(pts[k], pts[i]));
        const L = norm3(N);
        if (L < 1e-10) continue;
        N = scale3(N, 1 / L);
        const d = dot3(N, pts[i]);
        let pos = false, neg = false;
        for (let l = 0; l < n && !(pos && neg); l++) {
          const s = dot3(N, pts[l]) - d;
          if (s > eps) pos = true; else if (s < -eps) neg = true;
        }
        if (!pos) addFacet(N, d);
        if (!neg) addFacet(scale3(N, -1), -d);
      }
    }
  }
  return out;
}

/**
 * Distance from the origin to the boundary of conv(points) when the origin is strictly inside
 * (a Ferrari–Canny style quality); ≤ 0 otherwise.
 */
export function originMargin(pts: V3[]): number {
  const facets = hullFacets(pts);
  if (!facets.length) return 0;
  return Math.min(...facets.map((f) => f.offset));
}

/** Wrenches with the moment divided by a characteristic length ρ, so all components are forces. */
export function scaleMoments(Fs: V3[], rho: number): V3[] {
  return Fs.map((F) => [F[0] / rho, F[1], F[2]] as V3);
}

// ---------------------------------------------------------------- CoR regions (planar picture)
/** A directed line of action: point p and unit direction d (contact normal or friction-cone edge). */
export interface ForceLine { p: P2; d: P2 }

/** Keep the part of a convex polygon where a·x ≤ b (Sutherland–Hodgman, one half-plane). */
export function clipHalfPlane(poly: P2[], a: P2, b: number): P2[] {
  const out: P2[] = [];
  for (let i = 0; i < poly.length; i++) {
    const P = poly[i], Q = poly[(i + 1) % poly.length];
    const fp = dot2(a, P) - b, fq = dot2(a, Q) - b;
    if (fp <= 0) out.push(P);
    if ((fp < 0 && fq > 0) || (fp > 0 && fq < 0)) {
      const t = fp / (fp - fq);
      out.push([P[0] + t * (Q[0] - P[0]), P[1] + t * (Q[1] - P[1])]);
    }
  }
  return out;
}

/**
 * Sign label of point c for one line: +1 when c lies to the left of the directed line (only CCW
 * rotations about c are compatible), −1 to the right (only CW), 0 on the line (both).
 * Equals sgn(((p − c) × d)_z) with a dead band, which is the sign of Fᵀ V / ω.
 */
export function corSign(line: ForceLine, c: P2, tol = 1e-9): -1 | 0 | 1 {
  const v = cross2(sub2(line.p, c), line.d);
  return v > tol ? 1 : v < -tol ? -1 : 0;
}

/** Which rotation senses about c are compatible with every line. */
export function corPointLabel(lines: ForceLine[], c: P2, tol = 1e-9): { plus: boolean; minus: boolean } {
  let plus = true, minus = true;
  for (const L of lines) {
    const s = corSign(L, c, tol);
    if (s < 0) plus = false;
    if (s > 0) minus = false;
  }
  return { plus, minus };
}

/** Polygon (clipped to `box`) of rotation centres whose rotation with the given sense is compatible with every line. */
export function corRegion(lines: ForceLine[], sense: 1 | -1, box: P2[]): P2[] {
  let poly = box;
  for (const { p, d } of lines) {
    const m = cross2(p, d);
    // sense · ((p − c) × d) ≥ 0  ⇔  sense · (cx·dy − cy·dx) ≤ sense · m
    poly = sense > 0 ? clipHalfPlane(poly, [d[1], -d[0]], m) : clipHalfPlane(poly, [-d[1], d[0]], -m);
    if (!poly.length) break;
  }
  return poly;
}

export function polygonAreaAbs(poly: P2[]): number {
  return poly.length < 3 ? 0 : Math.abs(signedArea(poly));
}

export function lineIntersection(a: ForceLine, b: ForceLine): P2 | null {
  const den = cross2(a.d, b.d);
  if (Math.abs(den) < 1e-12) return null;
  const t = cross2(sub2(b.p, a.p), b.d) / den;
  return add2(a.p, scale2(a.d, t));
}

/** Pairwise intersections of the lines that are compatible CoRs (handles regions that shrink to a point). */
export function feasibleCorPoints(lines: ForceLine[], tol = 1e-7): { c: P2; plus: boolean; minus: boolean }[] {
  const out: { c: P2; plus: boolean; minus: boolean }[] = [];
  for (let i = 0; i < lines.length; i++) {
    for (let j = i + 1; j < lines.length; j++) {
      const c = lineIntersection(lines[i], lines[j]);
      if (!c) continue;
      const lab = corPointLabel(lines, c, tol);
      if (!lab.plus && !lab.minus) continue;
      if (out.some((q) => norm2(sub2(q.c, c)) < 1e-6)) continue;
      out.push({ c, ...lab });
    }
  }
  return out;
}

// ---------------------------------------------------------------- feasible translations
export type TranslationCone =
  | { kind: 'all' }
  | { kind: 'none' }
  | { kind: 'arcs'; arcs: [number, number][]; rays: number[] };

/** Directions v with dᵢ·v ≥ 0 for every given direction (the translations allowed by the contacts). */
export function translationCone(dirs: P2[], tol = 1e-9): TranslationCone {
  if (!dirs.length) return { kind: 'all' };
  const ok = (a: number) => dirs.every((d) => d[0] * Math.cos(a) + d[1] * Math.sin(a) >= -tol);
  const cand: number[] = [];
  for (const d of dirs) {
    for (const q of [perp(d), scale2(perp(d), -1)]) {
      let a = Math.atan2(q[1], q[0]);
      if (a < 0) a += 2 * Math.PI;
      if (ok(a) && !cand.some((b) => Math.abs(b - a) < 1e-9 || Math.abs(Math.abs(b - a) - 2 * Math.PI) < 1e-9)) cand.push(a);
    }
  }
  if (!cand.length) return { kind: 'none' };
  cand.sort((x, y) => x - y);
  const arcs: [number, number][] = [];
  const covered = new Set<number>();
  for (let i = 0; i < cand.length; i++) {
    const a0 = cand[i];
    const a1 = i + 1 < cand.length ? cand[i + 1] : cand[0] + 2 * Math.PI;
    if (a1 - a0 < 1e-9) continue;
    if (ok((a0 + a1) / 2)) { arcs.push([a0, a1]); covered.add(i); covered.add((i + 1) % cand.length); }
  }
  const rays = cand.filter((_, i) => !covered.has(i));
  return { kind: 'arcs', arcs, rays };
}

// ---------------------------------------------------------------- LP solver
export type LPStatus = 'optimal' | 'infeasible' | 'unbounded';
export interface LPResult { status: LPStatus; x: number[]; value: number }

/**
 * Dense two-phase simplex with Bland's anti-cycling rule for
 *     minimise cᵀx  subject to  A x = b,  x ≥ 0.
 * Meant for the tiny problems in this lesson (a handful of rows, a few dozen columns).
 */
export function linprog(c: number[], A: number[][], b: number[], tol = 1e-9): LPResult {
  const m = A.length, n = c.length;
  const R = n + m; // rhs column
  const T: number[][] = A.map((row, i) => {
    const sg = b[i] < 0 ? -1 : 1;
    const r = new Array<number>(R + 1).fill(0);
    for (let j = 0; j < n; j++) r[j] = sg * row[j];
    r[n + i] = 1;
    r[R] = sg * b[i];
    return r;
  });
  const basis = Array.from({ length: m }, (_, i) => n + i);
  // Objective row: reduced costs d_j and −z in the rhs cell. Phase I minimises Σ artificials.
  const obj = new Array<number>(R + 1).fill(0);
  for (let j = 0; j <= R; j++) {
    if (j >= n && j < R) continue;
    let s = 0;
    for (let i = 0; i < m; i++) s += T[i][j];
    obj[j] = -s;
  }
  T.push(obj);
  const pivot = (r: number, col: number) => {
    const pr = T[r];
    const pv = pr[col];
    for (let j = 0; j <= R; j++) pr[j] /= pv;
    for (let i = 0; i <= m; i++) {
      if (i === r) continue;
      const f = T[i][col];
      if (f === 0) continue;
      const row = T[i];
      for (let j = 0; j <= R; j++) row[j] -= f * pr[j];
    }
    basis[r] = col;
  };
  const run = (allowed: (j: number) => boolean): 'optimal' | 'unbounded' => {
    for (let iter = 0; iter < 10000; iter++) {
      let col = -1;
      for (let j = 0; j < R; j++) if (allowed(j) && T[m][j] < -tol) { col = j; break; }
      if (col < 0) return 'optimal';
      let row = -1, best = Infinity;
      for (let i = 0; i < m; i++) {
        const a = T[i][col];
        if (a <= tol) continue;
        const ratio = T[i][R] / a;
        if (ratio < best - 1e-12 || (Math.abs(ratio - best) <= 1e-12 && basis[i] < basis[row])) { best = ratio; row = i; }
      }
      if (row < 0) return 'unbounded';
      pivot(row, col);
    }
    return 'optimal';
  };
  run(() => true);
  const bScale = 1 + b.reduce((s, x) => s + Math.abs(x), 0);
  if (-T[m][R] > 1e-7 * bScale) return { status: 'infeasible', x: new Array(n).fill(0), value: NaN };
  // Drive artificial variables out of the basis (their values are zero now).
  for (let i = 0; i < m; i++) {
    if (basis[i] < n) continue;
    let col = -1;
    for (let j = 0; j < n; j++) if (Math.abs(T[i][j]) > tol) { col = j; break; }
    if (col >= 0) pivot(i, col);
  }
  // Phase II objective.
  const row = T[m];
  row.fill(0);
  for (let j = 0; j < n; j++) row[j] = c[j];
  for (let i = 0; i < m; i++) {
    const bj = basis[i];
    if (bj >= n || c[bj] === 0) continue;
    for (let j = 0; j <= R; j++) row[j] -= c[bj] * T[i][j];
  }
  const st = run((j) => j < n);
  const x = new Array<number>(n).fill(0);
  for (let i = 0; i < m; i++) if (basis[i] < n) x[basis[i]] = Math.max(0, T[i][R]);
  if (st === 'unbounded') return { status: 'unbounded', x, value: -Infinity };
  return { status: 'optimal', x, value: x.reduce((s, xi, j) => s + xi * c[j], 0) };
}

/** Wrenches as the columns of a 3 × j matrix. */
export function wrenchMatrix(Fs: V3[]): number[][] {
  return [0, 1, 2].map((r) => Fs.map((F) => F[r]));
}

/**
 * LP certificate of form closure: k with kᵢ ≥ 1 and Σ kᵢ Fᵢ = 0, together with rank 3
 * (scaled so max kᵢ = 1). Null when no such certificate exists.
 */
export function closureCertificate(Fs: V3[]): number[] | null {
  const n = Fs.length;
  if (n < 4 || wrenchRank(Fs) < 3) return null;
  const A = wrenchMatrix(Fs);
  // k = 1 + k', k' ≥ 0:  A k' = −A·1
  const b = A.map((r) => -r.reduce((s, x) => s + x, 0));
  const lp = linprog(new Array(n).fill(1), A, b);
  if (lp.status !== 'optimal') return null;
  const k = lp.x.map((x) => 1 + x);
  const mx = Math.max(...k);
  return k.map((x) => x / mx);
}

export interface ResistResult {
  /** True when nonnegative contact forces can balance the external wrench. */
  ok: boolean;
  /** Minimum-total-force nonnegative solution (zeros when !ok). */
  k: number[];
  /** Unconstrained minimum-norm least-squares solution (may contain negative "pulls"). */
  kLS: number[];
  /** ‖Σ kLSᵢ Fᵢ + F_ext‖ — nonzero when even pulling contacts could not balance the wrench. */
  residual: number;
}

/** Contact force magnitudes kᵢ ≥ 0 (along each normal) with Σ kᵢ Fᵢ = −F_ext. */
export function resistWrench(Fs: V3[], Fext: V3): ResistResult {
  const n = Fs.length;
  if (!n) return { ok: norm3(Fext) < 1e-12, k: [], kLS: [], residual: norm3(Fext) };
  const A = wrenchMatrix(Fs);
  const b = [-Fext[0], -Fext[1], -Fext[2]];
  const kLS = matVec(pinv(A), b);
  const r = matVec(A, kLS);
  const residual = Math.hypot(r[0] - b[0], r[1] - b[1], r[2] - b[2]);
  const lp = linprog(new Array(n).fill(1), A, b);
  const ok = lp.status === 'optimal';
  return { ok, k: ok ? lp.x : new Array(n).fill(0), kLS, residual };
}

// ---------------------------------------------------------------- automatic 4-contact form closure
/** Inscribed-ball radius of the tetrahedron conv(P0..P3) around the origin (≤ 0 if the origin is not strictly inside). */
function tetraMargin(P: V3[]): number {
  // A flat tetrahedron (all four wrenches in one plane) never contains the origin in its interior.
  const vol6 = dot3(cross3(sub3(P[1], P[0]), sub3(P[2], P[0])), sub3(P[3], P[0]));
  if (Math.abs(vol6) < 1e-9) return 0;
  let margin = Infinity;
  const faces = [[0, 1, 2, 3], [0, 1, 3, 2], [0, 2, 3, 1], [1, 2, 3, 0]];
  for (const [a, b, c, d] of faces) {
    let N = cross3(sub3(P[b], P[a]), sub3(P[c], P[a]));
    const L = norm3(N);
    if (L < 1e-12) return 0;
    N = scale3(N, 1 / L);
    if (dot3(N, sub3(P[d], P[a])) > 0) N = scale3(N, -1); // outward
    margin = Math.min(margin, dot3(N, P[a]));
  }
  return margin;
}

/**
 * Search boundary positions (sampled `perEdge` per edge) for the 4 frictionless contacts with the
 * largest closure margin (moments scaled by the shape radius). Returns their arc-length parameters.
 */
export function autoFormClosure(verts: P2[], perEdge = 6, margin = 0.06): { s: number[]; quality: number } | null {
  const rho = shapeRadius(verts);
  const cand: { s: number; W: V3 }[] = [];
  for (const e of edges(verts)) {
    const m = Math.min(margin, e.L / 3);
    for (let i = 0; i < perEdge; i++) {
      const t = m + ((e.L - 2 * m) * (i + 0.5)) / perEdge;
      const p = add2(e.a, scale2(e.u, t));
      const F = contactWrench(p, e.n);
      cand.push({ s: e.s0 + t, W: [F[0] / rho, F[1], F[2]] });
    }
  }
  let best: { s: number[]; quality: number } | null = null;
  const N = cand.length;
  for (let a = 0; a < N; a++) for (let b = a + 1; b < N; b++) for (let c = b + 1; c < N; c++) for (let d = c + 1; d < N; d++) {
    const q = tetraMargin([cand[a].W, cand[b].W, cand[c].W, cand[d].W]);
    if (q > 1e-9 && (!best || q > best.quality + 1e-12)) best = { s: [cand[a].s, cand[b].s, cand[c].s, cand[d].s], quality: q };
  }
  return best;
}
