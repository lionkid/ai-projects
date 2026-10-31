/**
 * Planar grasp analysis with Coulomb friction.
 *
 *  - bodies: convex polygons and a disc, boundary parameterised by arc length, inward normals
 *  - friction cones |f_t| ≤ μ f_n and their two edge wrenches per contact
 *  - force closure: the two-finger criterion, the general test pos{𝓕ᵢ} = R³ (combinatorial,
 *    LP certificate, convex-hull margin) and convex-hull faces for a 3D wrench-space plot
 *  - minimum grip forces for a disturbance wrench (min–max LP) and the dual picture: free twists,
 *    CoR / moment-labelling regions built from the cone edges
 *
 * Conventions (same as the planar helpers in @/mr):
 *   planar twist  V = [ω, vx, vy]   (angular part first)
 *   planar wrench F = [mz, fx, fy]  (moment first)
 * Both are expressed in the fixed frame {s}. The body sits at its reference pose with its area
 * centroid at the origin of {s}, so wrench moments are taken about the centroid.
 * Contact normals n̂ are unit vectors pointing INTO the body; t̂ = n̂ rotated by +90°.
 */
import { MatrixExp6, VecTose3, svd } from '@/mr';

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

// ---------------------------------------------------------------- bodies
export interface PolyBody { kind: 'poly'; key: string; name: string; verts: P2[] }
export interface DiscBody { kind: 'disc'; key: string; name: string; R: number }
export type Body = PolyBody | DiscBody;

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
export function makePoly(key: string, name: string, verts: P2[]): PolyBody {
  let v = verts.map((p) => [p[0], p[1]] as P2);
  if (signedArea(v) < 0) v = v.reverse();
  const c = polygonCentroid(v);
  return { kind: 'poly', key, name, verts: v.map((p) => sub2(p, c)) };
}

export function makeDisc(key: string, name: string, R: number): DiscBody {
  return { kind: 'disc', key, name, R };
}

/** Boundary point: arc length `s` from the start (CCW), position `p` and unit INWARD normal `n`. */
export interface BoundaryPoint { s: number; p: P2; n: P2 }

interface Edge { a: P2; L: number; u: P2; n: P2; s0: number }

function polyEdges(verts: P2[]): Edge[] {
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

export function perimeterOf(b: Body): number {
  return b.kind === 'disc' ? 2 * Math.PI * b.R : polyEdges(b.verts).reduce((s, e) => s + e.L, 0);
}

function discPoint(R: number, phi: number): BoundaryPoint {
  const u: P2 = [Math.cos(phi), Math.sin(phi)];
  const w = ((phi % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI);
  return { s: R * w, p: scale2(u, R), n: scale2(u, -1) };
}

/**
 * Boundary point at arc length `s` (taken modulo the perimeter). On a polygon, points closer than
 * `margin` to a vertex are pushed back onto the edge, because the normal is undefined at a corner.
 */
export function boundaryAt(b: Body, s: number, margin = 0.06): BoundaryPoint {
  if (b.kind === 'disc') return discPoint(b.R, s / b.R);
  const E = polyEdges(b.verts);
  const P = E.reduce((acc, e) => acc + e.L, 0);
  let r = ((s % P) + P) % P;
  for (let k = 0; k < E.length; k++) {
    const e = E[k];
    if (r <= e.L || k === E.length - 1) {
      const m = Math.min(margin, e.L / 3);
      const t = Math.min(Math.max(r, m), e.L - m);
      return { s: e.s0 + t, p: add2(e.a, scale2(e.u, t)), n: e.n };
    }
    r -= e.L;
  }
  /* c8 ignore next */
  throw new Error('boundaryAt: empty polygon');
}

/** Closest boundary point to q (kept `margin` away from polygon corners). */
export function projectToBoundary(b: Body, q: P2, margin = 0.06): BoundaryPoint {
  if (b.kind === 'disc') return discPoint(b.R, norm2(q) > 1e-12 ? Math.atan2(q[1], q[0]) : 0);
  let best: BoundaryPoint | null = null;
  let bestD = Infinity;
  for (const e of polyEdges(b.verts)) {
    const m = Math.min(margin, e.L / 3);
    const t = Math.min(Math.max(dot2(sub2(q, e.a), e.u), m), e.L - m);
    const p = add2(e.a, scale2(e.u, t));
    const d = norm2(sub2(q, p));
    if (d < bestD) { bestD = d; best = { s: e.s0 + t, p, n: e.n }; }
  }
  return best!;
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

export function insideBody(b: Body, q: P2): boolean {
  return b.kind === 'disc' ? norm2(q) <= b.R : insidePolygon(q, b.verts);
}

/** Closed outline for drawing (the disc is sampled with `n` points). */
export function outline(b: Body, n = 72): P2[] {
  if (b.kind === 'poly') return b.verts;
  return Array.from({ length: n }, (_, k) => scale2([Math.cos((2 * Math.PI * k) / n), Math.sin((2 * Math.PI * k) / n)], b.R));
}

/** Largest distance from the centroid to the boundary: a characteristic length ρ for moments. */
export function bodyRadius(b: Body): number {
  return b.kind === 'disc' ? b.R : Math.max(...b.verts.map(norm2));
}

/** Radius of gyration about the centroid for uniform density: √(J/A). */
export function gyrationRadius(b: Body): number {
  if (b.kind === 'disc') return b.R / Math.SQRT2;
  const v = b.verts;
  let J = 0;
  for (let i = 0; i < v.length; i++) {
    const [x0, y0] = v[i], [x1, y1] = v[(i + 1) % v.length];
    const w = x0 * y1 - x1 * y0;
    J += w * (x0 * x0 + x0 * x1 + x1 * x1 + y0 * y0 + y0 * y1 + y1 * y1);
  }
  return Math.sqrt(J / 12 / signedArea(v));
}

// ---------------------------------------------------------------- friction cones and wrenches
export interface Contact { p: P2; n: P2 }

/** Half-angle α = tan⁻¹ μ of the friction cone. */
export const frictionAngle = (mu: number): number => Math.atan(Math.max(0, mu));

/**
 * The two edges of the friction cone, n̂ + μt̂ (rotated CCW from n̂ by α) and n̂ − μt̂.
 * Their normal component is exactly 1, so a force k₊e₊ + k₋e₋ has f_n = k₊ + k₋ and f_t = μ(k₊ − k₋).
 */
export function coneEdges(n: P2, mu: number): [P2, P2] {
  const t = perp(n);
  return [add2(n, scale2(t, mu)), sub2(n, scale2(t, mu))];
}

/** Cone edges scaled to unit length. */
export function unitConeEdges(n: P2, mu: number): [P2, P2] {
  const [a, b] = coneEdges(n, mu);
  return [unit2(a), unit2(b)];
}

/** Wrench [p × f, fx, fy] of a force f applied at point p. */
export function forceWrench(p: P2, f: P2): V3 {
  return [cross2(p, f), f[0], f[1]];
}

/** Edge wrenches of every contact, two per contact in the order [c₀₊, c₀₋, c₁₊, c₁₋, …]. */
export function edgeWrenches(cs: Contact[], mu: number, unit = true): V3[] {
  const out: V3[] = [];
  for (const c of cs) for (const f of unit ? unitConeEdges(c.n, mu) : coneEdges(c.n, mu)) out.push(forceWrench(c.p, f));
  return out;
}

/** Normal / tangential components of a contact force and whether it lies inside the cone. */
export function coneCheck(n: P2, mu: number, f: P2, tol = 1e-9): { fn: number; ft: number; inside: boolean } {
  const fn = dot2(f, n);
  const ft = dot2(f, perp(n));
  return { fn, ft, inside: fn >= -tol && Math.abs(ft) <= mu * fn + tol };
}

// ---------------------------------------------------------------- twists
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

/** Normalise a twist for display: |ω| = 1 for rotations, |v| = 1 for translations. */
export function normalizeTwist(V: V3): V3 {
  const w = Math.abs(V[0]);
  const v = Math.hypot(V[1], V[2]);
  const s = w > 1e-9 * Math.max(1, v) ? w : v;
  return s > 0 ? scale3(V, 1 / s) : V;
}

// ---------------------------------------------------------------- force closure
/** Numerical rank of a set of planar wrenches (rank of the 3 × j matrix with them as columns). */
export function wrenchRank(Fs: V3[], rtol = 1e-9): number {
  if (!Fs.length) return 0;
  const A = [0, 1, 2].map((r) => Fs.map((F) => F[r]));
  const { S } = svd(A);
  const smax = S[0] ?? 0;
  if (smax <= 1e-14) return 0;
  return S.filter((x) => x > rtol * smax).length;
}

function orthonormalPair(F0: V3): [V3, V3] {
  const basis: V3[] = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
  let e = basis[0];
  let best = Infinity;
  for (const b of basis) { const d = Math.abs(dot3(b, F0)); if (d < best) { best = d; e = b; } }
  const u = cross3(F0, e);
  const w = cross3(F0, u);
  return [scale3(u, 1 / norm3(u)), scale3(w, 1 / norm3(w))];
}

/**
 * Candidate extreme directions (unit length) of the free-twist cone C = {V : Fᵢᵀ V ≥ 0 ∀i}.
 *   rank 3: C is pointed and every extreme ray is ±(Fᵢ × Fⱼ) for an independent pair;
 *   rank 2: all wrenches lie in a plane with normal N; C = span(N) ⊕ (a 2D cone in the plane),
 *           whose edges are perpendicular to some Fᵢ: candidates ±N and ±(N × Fᵢ);
 *   rank 1: C is a half-space (or a plane): the plane's basis ± and the wrench direction;
 *   rank 0: every direction.
 */
export function twistCandidates(Fs: V3[]): V3[] {
  const rank = wrenchRank(Fs);
  const out: V3[] = [];
  const push = (V: V3) => { const L = norm3(V); if (L > 1e-12) { out.push(scale3(V, 1 / L), scale3(V, -1 / L)); } };
  if (rank === 0) return [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]];
  if (rank === 1) {
    const F0 = Fs.reduce((a, F) => (norm3(F) > norm3(a) ? F : a), Fs[0]);
    const [u, w] = orthonormalPair(F0);
    push(u); push(w);
    out.push(scale3(F0, 1 / norm3(F0)), scale3(F0, -1 / norm3(F0)));
    return out;
  }
  if (rank === 3) {
    for (let i = 0; i < Fs.length; i++) {
      for (let j = i + 1; j < Fs.length; j++) {
        const c = cross3(Fs[i], Fs[j]);
        if (norm3(c) > 1e-9 * (norm3(Fs[i]) * norm3(Fs[j]))) push(c);
      }
    }
    return out;
  }
  // rank 2: normal of the plane from the best-conditioned pair
  let N: V3 = [0, 0, 0];
  for (let i = 0; i < Fs.length; i++) for (let j = i + 1; j < Fs.length; j++) {
    const c = cross3(Fs[i], Fs[j]);
    if (norm3(c) > norm3(N)) N = c;
  }
  N = scale3(N, 1 / norm3(N));
  push(N);
  for (const F of Fs) push(cross3(N, F));
  return out;
}

export function isFeasibleTwist(Fs: V3[], V: V3, tol = 1e-9): boolean {
  return Fs.every((F) => dot3(F, V) >= -tol * Math.max(1, norm3(F)));
}

/** Feasible candidate rays of the free-twist cone (unit, de-duplicated). */
export function freeTwistRays(Fs: V3[], tol = 1e-9): V3[] {
  const out: V3[] = [];
  for (const V of twistCandidates(Fs)) {
    if (!isFeasibleTwist(Fs, V, tol)) continue;
    if (out.some((U) => dot3(U, V) > 1 - 1e-9)) continue;
    out.push(V);
  }
  return out;
}

/** Force closure: no nonzero twist V with Fᵢᵀ V ≥ 0 for every wrench, i.e. pos{Fᵢ} = R³. */
export function isForceClosure(Fs: V3[], tol = 1e-9): boolean {
  return freeTwistRays(Fs, tol).length === 0;
}

/** A free twist inside the free cone (sum of its rays), normalised for display; null under force closure. */
export function representativeFreeTwist(Fs: V3[], tol = 1e-9): V3 | null {
  const rays = freeTwistRays(Fs, tol);
  if (!rays.length) return null;
  let V: V3 = [0, 0, 0];
  for (const r of rays) V = [V[0] + r[0], V[1] + r[1], V[2] + r[2]];
  if (norm3(V) < 1e-6 || !isFeasibleTwist(Fs, V, tol)) V = rays[0];
  return normalizeTwist(V);
}

/**
 * A free twist that is easy to describe: a pure translation (|v| = 1) whenever one is free (the middle
 * of the widest free sector of directions), otherwise `representativeFreeTwist`. Null under force closure.
 */
export function preferredFreeTwist(Fs: V3[], tol = 1e-9): V3 | null {
  const tc = translationCone(Fs.map((F) => [F[1], F[2]] as P2), tol);
  if (tc.kind === 'all') return [0, 0, 1];
  if (tc.kind === 'arcs') {
    let a: number | null = null;
    let widest = -1;
    for (const [a0, a1] of tc.arcs) if (a1 - a0 > widest) { widest = a1 - a0; a = (a0 + a1) / 2; }
    if (a === null && tc.rays.length) a = tc.rays[0];
    if (a !== null) return [0, Math.cos(a), Math.sin(a)];
  }
  return representativeFreeTwist(Fs, tol);
}

/** Wrenches as the columns of a 3 × j matrix. */
export function wrenchMatrix(Fs: V3[]): number[][] {
  return [0, 1, 2].map((r) => Fs.map((F) => F[r]));
}

/**
 * LP certificate of force closure: k with kᵢ ≥ 1 and Σ kᵢ Fᵢ = 0, together with rank 3
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

/** Wrenches with the moment divided by a characteristic length ρ, so all components are forces. */
export function scaleMoments(Fs: V3[], rho: number): V3[] {
  return Fs.map((F) => [F[0] / rho, F[1], F[2]] as V3);
}

// ---------------------------------------------------------------- convex hull in wrench space
export interface Facet { normal: V3; offset: number; idx: number[] }

/**
 * Supporting planes of conv(points): outward unit normal N with N·x ≤ offset for every point.
 * Brute force over triples (O(n⁴)), fine for the ≤ 12 points used here.
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
 * (a Ferrari–Canny style grasp quality); ≤ 0 otherwise (0 for flat or degenerate hulls).
 */
export function originMargin(pts: V3[]): number {
  const facets = hullFacets(pts);
  if (!facets.length) return 0;
  return Math.min(...facets.map((f) => f.offset));
}

/** Indices of the 2D convex hull of `pts` in CCW order (collinear boundary points dropped). */
export function convexHull2D(pts: P2[]): number[] {
  const idx = pts.map((_, i) => i).sort((a, b) => pts[a][0] - pts[b][0] || pts[a][1] - pts[b][1]);
  if (idx.length < 3) return idx;
  const turn = (o: number, a: number, b: number) => cross2(sub2(pts[a], pts[o]), sub2(pts[b], pts[o]));
  const lower: number[] = [];
  for (const i of idx) {
    while (lower.length >= 2 && turn(lower[lower.length - 2], lower[lower.length - 1], i) <= 1e-12) lower.pop();
    lower.push(i);
  }
  const upper: number[] = [];
  for (let k = idx.length - 1; k >= 0; k--) {
    const i = idx[k];
    while (upper.length >= 2 && turn(upper[upper.length - 2], upper[upper.length - 1], i) <= 1e-12) upper.pop();
    upper.push(i);
  }
  return lower.slice(0, -1).concat(upper.slice(0, -1));
}

/** Distinct points (within `tol`): indices of the first occurrence of each. */
export function distinctPoints(pts: V3[], tol = 1e-9): number[] {
  const keep: number[] = [];
  pts.forEach((p, i) => { if (!keep.some((j) => norm3(sub3(pts[j], p)) <= tol)) keep.push(i); });
  return keep;
}

/**
 * Faces of conv(points) as vertex loops ordered counter-clockwise about the outward normal
 * (for drawing). A flat point set gives a single face. Fewer than three non-collinear points give none.
 */
export function hullFaces(pts: V3[]): { normal: V3; loop: number[] }[] {
  const keep = distinctPoints(pts);
  const sub = keep.map((i) => pts[i]);
  const faces: { normal: V3; loop: number[] }[] = [];
  for (const f of hullFacets(sub)) {
    if (f.idx.length < 3) continue;
    const N = f.normal;
    const [u, w] = orthonormalPair(N);
    // (u, w, N) must be right-handed so that CCW in (u, w) means CCW about N
    const sgn = dot3(cross3(u, w), N) > 0 ? 1 : -1;
    const c = f.idx.reduce<V3>((acc, i) => [acc[0] + sub[i][0], acc[1] + sub[i][1], acc[2] + sub[i][2]], [0, 0, 0]);
    const ctr = scale3(c, 1 / f.idx.length);
    const uv = f.idx.map((i) => { const d = sub3(sub[i], ctr); return [dot3(d, u), sgn * dot3(d, w)] as P2; });
    const hull = convexHull2D(uv);
    if (hull.length < 3) continue;
    const loop = hull.map((k) => keep[f.idx[k]]);
    // a flat set yields the same polygon twice (±N): keep one
    const key = [...loop].sort((a, b) => a - b).join(',');
    if (faces.some((g) => [...g.loop].sort((a, b) => a - b).join(',') === key)) continue;
    faces.push({ normal: N, loop });
  }
  return faces;
}

// ---------------------------------------------------------------- two-finger criterion
export interface TwoFingerResult {
  ok: boolean;
  /** Friction angle α = tan⁻¹ μ. */
  alpha: number;
  /** Angle between each inward normal and the line towards the other contact. */
  beta: [number, number];
}

const angleBetween = (a: P2, b: P2): number => Math.atan2(Math.abs(cross2(a, b)), dot2(a, b));

/**
 * Planar two-finger test: force closure ⇔ the line through the two contacts lies strictly inside
 * both friction cones (the squeeze pair f, −f along the line is then an interior internal force).
 * Both senses of the line are checked, which also covers expanding (inside-out) grasps.
 */
export function twoFingerTest(c1: Contact, c2: Contact, mu: number, tol = 1e-9): TwoFingerResult {
  const alpha = frictionAngle(mu);
  const d = sub2(c2.p, c1.p);
  if (norm2(d) < 1e-9) return { ok: false, alpha, beta: [Math.PI / 2, Math.PI / 2] };
  const b1 = angleBetween(c1.n, d);
  const b2 = angleBetween(c2.n, scale2(d, -1));
  const squeeze = b1 < alpha - tol && b2 < alpha - tol;
  const expand = Math.PI - b1 < alpha - tol && Math.PI - b2 < alpha - tol;
  return { ok: mu > 0 && (squeeze || expand), alpha, beta: [b1, b2] };
}

// ---------------------------------------------------------------- grip forces for a disturbance
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
  // Objective row: reduced costs and −z in the rhs cell. Phase I minimises Σ artificials.
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

export interface HoldResult {
  /** True when some forces inside the cones balance the disturbance (no force limit). */
  feasible: boolean;
  /** Smallest achievable value of the largest normal force (Infinity when infeasible). */
  fStar: number;
  /** Per-contact normal force, tangential force (along t̂ = perp(n̂)) and force vector of that solution. */
  fn: number[];
  ft: number[];
  f: P2[];
}

/**
 * Contact forces inside the friction cones that balance the external wrench, Σ 𝓕_contact + 𝓕_ext = 0,
 * while making the largest normal force as small as possible:
 *     minimise t  s.t.  Σₑ kₑ 𝓕ₑ = −𝓕_ext,  k_{i+} + k_{i−} ≤ t,  k ≥ 0
 * with the non-unit cone edges (so k_{i+} + k_{i−} is finger i's normal force). The optimum is
 * positively homogeneous: scaling 𝓕_ext by λ ≥ 0 scales every force by λ.
 */
export function minGripForces(cs: Contact[], mu: number, Fext: V3): HoldResult {
  const m = cs.length;
  const none = (feasible: boolean): HoldResult => ({ feasible, fStar: feasible ? 0 : Infinity, fn: cs.map(() => 0), ft: cs.map(() => 0), f: cs.map(() => [0, 0] as P2) });
  if (norm3(Fext) < 1e-12) return none(true);
  if (!m) return none(false);
  const E = edgeWrenches(cs, mu, false);
  const nE = 2 * m;
  const nv = nE + m + 1; // k (2m), slacks s (m), t
  const A: number[][] = [];
  const b: number[] = [];
  for (let r = 0; r < 3; r++) {
    const rowA = new Array<number>(nv).fill(0);
    for (let e = 0; e < nE; e++) rowA[e] = E[e][r];
    A.push(rowA); b.push(-Fext[r]);
  }
  for (let i = 0; i < m; i++) {
    const rowA = new Array<number>(nv).fill(0);
    rowA[2 * i] = 1; rowA[2 * i + 1] = 1; rowA[nE + i] = 1; rowA[nv - 1] = -1;
    A.push(rowA); b.push(0);
  }
  const cost = new Array<number>(nv).fill(0);
  cost[nv - 1] = 1;
  const lp = linprog(cost, A, b);
  if (lp.status !== 'optimal') return none(false);
  const fn: number[] = [], ft: number[] = [], f: P2[] = [];
  cs.forEach((c, i) => {
    const kp = lp.x[2 * i], km = lp.x[2 * i + 1];
    fn.push(kp + km);
    ft.push(mu * (kp - km));
    f.push(add2(scale2(c.n, kp + km), scale2(perp(c.n), mu * (kp - km))));
  });
  return { feasible: true, fStar: Math.max(...fn), fn, ft, f };
}

/** Direction of the free-body acceleration [α; a] caused by a wrench (up to 1/mass), ρ_g = radius of gyration. */
export function accelTwist(F: V3, rhoG: number): V3 {
  return [F[0] / (rhoG * rhoG), F[1], F[2]];
}

// ---------------------------------------------------------------- the dual picture
/**
 * A free twist (Fᵢᵀ V ≥ 0 for every edge wrench) on which the disturbance does positive power,
 * chosen among the extreme rays of the free cone to maximise 𝓕_extᵀ V / ‖V‖ with ‖V‖² = (ρω)² + |v|².
 * By Farkas' lemma such a twist exists exactly when −𝓕_ext is outside pos{Fᵢ}. Null otherwise.
 */
export function drivenFreeTwist(Fs: V3[], Fext: V3, rho: number, tol = 1e-9): V3 | null {
  let best: V3 | null = null;
  let bestVal = tol * Math.max(1, norm3(Fext));
  for (const V of freeTwistRays(Fs, tol)) {
    const val = dot3(Fext, V) / Math.hypot(rho * V[0], V[1], V[2]);
    if (val > bestVal) { bestVal = val; best = V; }
  }
  return best ? normalizeTwist(best) : null;
}

/**
 * An example disturbance that the grasp cannot resist when V is free: a unit force applied inside the
 * body that pushes along the motion V (for a rotation about c: tangential, at a point far from c).
 */
export function pushAlong(b: Body, V: V3): { a: P2; f: P2 } {
  const c = corOf(V);
  if (!c) return { a: [0, 0], f: unit2([V[1], V[2]]) };
  let q: P2 = [0, 0];
  let bestD = -1;
  for (const v of outline(b, 36)) { const d = norm2(sub2(v, c)); if (d > bestD) { bestD = d; q = v; } }
  const a = scale2(q, 0.72);
  const r = sub2(a, c);
  return { a, f: scale2(unit2(perp(r)), Math.sign(V[0])) };
}

// ---------------------------------------------------------------- CoR / moment-labelling regions
/** A directed line of action: point p and unit direction d (a friction-cone edge or a contact normal). */
export interface ForceLine { p: P2; d: P2 }

/** Directed lines along the (unit) cone edges of every contact, two per contact. */
export function edgeLines(cs: Contact[], mu: number): ForceLine[] {
  const out: ForceLine[] = [];
  for (const c of cs) for (const d of unitConeEdges(c.n, mu)) out.push({ p: c.p, d });
  return out;
}

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
 * Sign label of point c for one line: +1 when c lies to the left of the directed line (the force has a
 * CCW moment about c, so only CCW rotations about c are free), −1 to the right, 0 on the line.
 */
export function corSign(line: ForceLine, c: P2, tol = 1e-9): -1 | 0 | 1 {
  const v = cross2(sub2(line.p, c), line.d);
  return v > tol ? 1 : v < -tol ? -1 : 0;
}

/** Which rotation senses about c are free with respect to every line. */
export function corPointLabel(lines: ForceLine[], c: P2, tol = 1e-9): { plus: boolean; minus: boolean } {
  let plus = true, minus = true;
  for (const L of lines) {
    const s = corSign(L, c, tol);
    if (s < 0) plus = false;
    if (s > 0) minus = false;
  }
  return { plus, minus };
}

/** Polygon (clipped to `box`) of rotation centres whose rotation with the given sense is free for every line. */
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

export type TranslationCone =
  | { kind: 'all' }
  | { kind: 'none' }
  | { kind: 'arcs'; arcs: [number, number][]; rays: number[] };

/** Directions v with dᵢ·v ≥ 0 for every given direction (the free translations). Angles in [0, 2π). */
export function translationCone(dirs: P2[], tol = 1e-9): TranslationCone {
  if (!dirs.length) return { kind: 'all' };
  const ok = (a: number) => dirs.every((d) => d[0] * Math.cos(a) + d[1] * Math.sin(a) >= -tol * Math.max(1, norm2(d)));
  const cand: number[] = [];
  for (const d of dirs) {
    if (norm2(d) < 1e-12) continue;
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
