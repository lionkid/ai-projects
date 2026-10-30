/**
 * Planar linkage model for the dof / Grübler lesson (ch02-01).
 *
 * A mechanism is a list of rigid bodies (each with a planar pose and attachment
 * "pins" given in body coordinates) plus R / P joints between pins. Exactly one
 * body is fixed: the ground. Everything here is plain data + pure functions so the
 * counting, the rank test and the constraint solver can be unit tested without a DOM.
 *
 * Two different dof numbers are computed:
 *   - Grübler's count m(N − 1 − J) + Σ f_i, which assumes independent constraints;
 *   - the instantaneous mobility 3·(moving bodies) − rank(constraint Jacobian),
 *     i.e. what the mechanism can actually do at its current configuration.
 * They differ exactly when some joint constraints are redundant (special geometry).
 *
 * Solver design (see `dragPin`): each iteration solves a small KKT system that
 * moves the dragged pin towards the cursor *inside* the tangent space of the
 * constraint manifold while minimising a "pin displacement" metric, then the
 * configuration is projected back onto the manifold (M-weighted Gauss–Newton).
 * Steps are clamped and warm-started from the current pose, which keeps a closed
 * chain on its assembly branch.
 */
import { inv, planarFK, solve, svd, wrapToPi, type Mat } from '@/mr';

export type Pt = [number, number];
export interface Pose { x: number; y: number; phi: number }

export interface Body {
  name: string;
  /** Attachment pins in body coordinates. */
  pins: Pt[];
  pose: Pose;
  fixed: boolean;
}

/** Pin `pa` of body a and pin `pb` of body b coincide (2 scalar constraints). */
export interface JointR { type: 'R'; a: number; b: number; pa: number; pb: number }
/**
 * Pin `pb` of body b slides along the line through pin `pa` of body a whose
 * direction is `dir` (unit, in a's frame); the relative orientation φ_b − φ_a is
 * locked to `dphi` (2 scalar constraints).
 */
export interface JointP {
  type: 'P'; a: number; b: number; pa: number; pb: number;
  dir: Pt;
  dphi: number;
  /** Drawn rail extent along `dir` measured from pin `pa` (display only). */
  extent?: [number, number];
}
export type Joint = JointR | JointP;

export interface Mechanism { bodies: Body[]; joints: Joint[] }
export interface PointRef { body: number; pin: number }

/** A planar rigid body has m = 3 freedoms (x, y, φ). */
export const M_PLANAR = 3;
/** Hard cap used by the sandbox editor. */
export const MAX_BODIES = 12;

// ---------------------------------------------------------------------------
// Geometry helpers
// ---------------------------------------------------------------------------

export function worldPt(b: Body, pin: number): Pt {
  const [lx, ly] = b.pins[pin];
  const c = Math.cos(b.pose.phi), s = Math.sin(b.pose.phi);
  return [b.pose.x + c * lx - s * ly, b.pose.y + s * lx + c * ly];
}

export function worldDir(b: Body, d: Pt): Pt {
  const c = Math.cos(b.pose.phi), s = Math.sin(b.pose.phi);
  return [c * d[0] - s * d[1], s * d[0] + c * d[1]];
}

/** World point → body coordinates. */
export function toLocal(b: Body, p: Pt): Pt {
  const dx = p[0] - b.pose.x, dy = p[1] - b.pose.y;
  const c = Math.cos(b.pose.phi), s = Math.sin(b.pose.phi);
  return [c * dx + s * dy, -s * dx + c * dy];
}

export function fixedBody(m: Mechanism): number { return m.bodies.findIndex((b) => b.fixed); }

export function cloneMechanism(m: Mechanism): Mechanism {
  return {
    bodies: m.bodies.map((b) => ({ name: b.name, pins: b.pins.map((p) => [p[0], p[1]] as Pt), pose: { ...b.pose }, fixed: b.fixed })),
    joints: m.joints.map((j) => (j.type === 'R' ? { ...j } : { ...j, dir: [j.dir[0], j.dir[1]] as Pt, extent: j.extent ? [j.extent[0], j.extent[1]] as [number, number] : undefined })),
  };
}

/** Intersection of two circles; `upper` picks the point to the left of the direction c1 → c2. */
export function circleIntersect(c1: Pt, r1: number, c2: Pt, r2: number, upper = true): Pt | null {
  const dx = c2[0] - c1[0], dy = c2[1] - c1[1];
  const d = Math.hypot(dx, dy);
  if (d < 1e-12 || d > r1 + r2 || d < Math.abs(r1 - r2)) return null;
  const a = (r1 * r1 - r2 * r2 + d * d) / (2 * d);
  const h = Math.sqrt(Math.max(0, r1 * r1 - a * a));
  const ux = dx / d, uy = dy / d;
  const px = c1[0] + a * ux, py = c1[1] + a * uy;
  return upper ? [px - h * uy, py + h * ux] : [px + h * uy, py - h * ux];
}

// ---------------------------------------------------------------------------
// Counting (Grübler)
// ---------------------------------------------------------------------------

/** Relative freedoms f of the joint types available in the sandbox (same in the plane and in space). */
export const JOINT_FREEDOM: Record<Joint['type'], number> = { R: 1, P: 1 };

export interface GrublerCount { m: number; N: number; J: number; sumF: number; sumC: number; dof: number }

/** dof = m(N − 1 − J) + Σ f_i  (N counts the ground). */
export function grublerDof(m: number, N: number, J: number, sumF: number): number {
  return m * (N - 1 - J) + sumF;
}

export function grubler(mech: Mechanism, m = M_PLANAR): GrublerCount {
  const N = mech.bodies.length;
  const J = mech.joints.length;
  const sumF = mech.joints.reduce((s, j) => s + JOINT_FREEDOM[j.type], 0);
  return { m, N, J, sumF, sumC: m * J - sumF, dof: grublerDof(m, N, J, sumF) };
}

// ---------------------------------------------------------------------------
// Constraints
// ---------------------------------------------------------------------------

export interface ConstraintSystem {
  /** Residuals, one per scalar constraint (zero when the mechanism is assembled). */
  C: number[];
  /** Jacobian of C with respect to the stacked (x, y, φ) of every moving body. */
  J: Mat;
  nVars: number;
  /** Index of the first variable of each body, or −1 for the fixed body. */
  varBase: number[];
}

export function constraintSystem(m: Mechanism): ConstraintSystem {
  const varBase = m.bodies.map(() => -1);
  let nVars = 0;
  m.bodies.forEach((b, i) => { if (!b.fixed) { varBase[i] = nVars; nVars += 3; } });
  const C: number[] = [];
  const J: number[][] = [];
  const row = () => { const r = new Array<number>(nVars).fill(0); J.push(r); return r; };
  for (const j of m.joints) {
    const A = m.bodies[j.a], B = m.bodies[j.b];
    const pA = worldPt(A, j.pa), pB = worldPt(B, j.pb);
    // Lever arms from each body origin to its pin (d p / d φ = perp(r)).
    const rA: Pt = [pA[0] - A.pose.x, pA[1] - A.pose.y];
    const rB: Pt = [pB[0] - B.pose.x, pB[1] - B.pose.y];
    const ba = varBase[j.a], bb = varBase[j.b];
    if (j.type === 'R') {
      const r0 = row(), r1 = row();
      C.push(pA[0] - pB[0], pA[1] - pB[1]);
      if (ba >= 0) { r0[ba] += 1; r0[ba + 2] += -rA[1]; r1[ba + 1] += 1; r1[ba + 2] += rA[0]; }
      if (bb >= 0) { r0[bb] -= 1; r0[bb + 2] += rB[1]; r1[bb + 1] -= 1; r1[bb + 2] -= rB[0]; }
    } else {
      const u = worldDir(A, j.dir);
      const n: Pt = [-u[1], u[0]];
      const d: Pt = [pB[0] - pA[0], pB[1] - pA[1]];
      const r0 = row(), r1 = row();
      C.push(n[0] * d[0] + n[1] * d[1], wrapToPi(B.pose.phi - A.pose.phi - j.dphi));
      if (ba >= 0) {
        r0[ba] += -n[0]; r0[ba + 1] += -n[1];
        // d n / d φ_a = perp(n); d pA / d φ_a = perp(rA)
        r0[ba + 2] += (-n[1] * d[0] + n[0] * d[1]) - (n[0] * -rA[1] + n[1] * rA[0]);
        r1[ba + 2] += -1;
      }
      if (bb >= 0) {
        r0[bb] += n[0]; r0[bb + 1] += n[1];
        r0[bb + 2] += n[0] * -rB[1] + n[1] * rB[0];
        r1[bb + 2] += 1;
      }
    }
  }
  return { C, J, nVars, varBase };
}

function maxAbsVec(v: number[]): number {
  let m = 0;
  for (const x of v) m = Math.max(m, Math.abs(x));
  return m;
}

// ---------------------------------------------------------------------------
// Rank / tangent space analysis
// ---------------------------------------------------------------------------

/** Singular values below RANK_REL_TOL · max(1, σ_max) count as zero. */
export const RANK_REL_TOL = 1e-7;

export interface Analysis {
  /** 3 × (number of moving bodies). */
  nVars: number;
  /** Number of scalar constraints written down (= Σ c_i in the plane). */
  sumC: number;
  /** Number of independent constraints (numerical rank of the constraint Jacobian). */
  rank: number;
  /** Instantaneous mobility nVars − rank. */
  mobility: number;
  /** sumC − rank: constraints that repeat what the others already impose. */
  redundant: number;
  maxResidual: number;
  /** Orthonormal basis of the tangent space (null space of the constraint Jacobian). */
  basis: number[][];
  /** Singular values of the (row-padded) constraint Jacobian, descending. */
  singular: number[];
  sys: ConstraintSystem;
}

export function analyze(m: Mechanism): Analysis {
  const sys = constraintSystem(m);
  const n = sys.nVars, rows = sys.C.length;
  let rank = 0;
  const basis: number[][] = [];
  let singular: number[] = [];
  if (n > 0) {
    if (rows === 0) {
      for (let c = 0; c < n; c++) { const e = new Array<number>(n).fill(0); e[c] = 1; basis.push(e); }
    } else {
      // Pad with zero rows so that the thin SVD returns a complete n × n V.
      const A = rows >= n ? sys.J : [...sys.J, ...Array.from({ length: n - rows }, () => new Array<number>(n).fill(0))];
      const { S, V } = svd(A);
      singular = S.slice();
      const tol = RANK_REL_TOL * Math.max(1, S[0] ?? 0);
      for (let c = 0; c < n; c++) {
        if ((S[c] ?? 0) > tol) rank++;
        else basis.push(V.map((r) => r[c]));
      }
    }
  }
  return { nVars: n, sumC: rows, rank, mobility: n - rank, redundant: rows - rank, maxResidual: maxAbsVec(sys.C), basis, singular, sys };
}

export interface PinMotion {
  /** Number of independent directions (0, 1 or 2) in which the pin can currently move. */
  dof: number;
  /** Main motion direction (unit) when dof ≥ 1. */
  dir: Pt | null;
}

/** First-order motion of one pin, from the tangent-space basis of `an`. */
export function pinMotion(m: Mechanism, an: Analysis, ref: PointRef): PinMotion {
  const k = an.sys.varBase[ref.body];
  if (k == null || k < 0 || an.basis.length === 0) return { dof: 0, dir: null };
  const b = m.bodies[ref.body];
  const p = worldPt(b, ref.pin);
  const rx = p[0] - b.pose.x, ry = p[1] - b.pose.y;
  let a = 0, c = 0, d = 0;
  for (const v of an.basis) {
    const vx = v[k] - ry * v[k + 2];
    const vy = v[k + 1] + rx * v[k + 2];
    a += vx * vx; c += vx * vy; d += vy * vy;
  }
  const tr = a + d, det = a * d - c * c;
  const disc = Math.sqrt(Math.max(0, (tr * tr) / 4 - det));
  const l1 = tr / 2 + disc, l2 = tr / 2 - disc;
  const tol = 1e-10;
  const dof = (l1 > tol ? 1 : 0) + (l2 > tol ? 1 : 0);
  let dir: Pt | null = null;
  if (dof >= 1) {
    let v: Pt = Math.abs(c) > 1e-14 ? [l1 - d, c] : a >= d ? [1, 0] : [0, 1];
    const L = Math.hypot(v[0], v[1]) || 1;
    v = [v[0] / L, v[1] / L];
    dir = v;
  }
  return { dof, dir };
}

/** Connected groups of moving bodies that are not linked to the ground by any chain of joints. */
export function ungroundedComponents(m: Mechanism): number {
  const parent = m.bodies.map((_, i) => i);
  const find = (i: number): number => { while (parent[i] !== i) { parent[i] = parent[parent[i]]; i = parent[i]; } return i; };
  for (const j of m.joints) { const ra = find(j.a), rb = find(j.b); if (ra !== rb) parent[ra] = rb; }
  const grounded = new Set<number>();
  m.bodies.forEach((b, i) => { if (b.fixed) grounded.add(find(i)); });
  const roots = new Set<number>();
  m.bodies.forEach((b, i) => { if (!b.fixed) { const r = find(i); if (!grounded.has(r)) roots.add(r); } });
  return roots.size;
}

/**
 * Mobility of the same physical mechanism placed in 3-D space (all R axes along z,
 * all P directions in the plane). Every joint also locks the relative out-of-plane
 * motion (z and the two tilts), so a chain connected to the ground stays in its plane
 * and only an ungrounded group gains 3 extra freedoms (lift + two tilts).
 */
export function spatialMobility(m: Mechanism, an: Analysis = analyze(m)): number {
  return an.mobility + 3 * ungroundedComponents(m);
}

// ---------------------------------------------------------------------------
// Solver
// ---------------------------------------------------------------------------

/** Regulariser of the per-body metric (keeps it positive definite for one-pin bodies). */
const METRIC_REG = 0.05;

/** 3×3 metric of one body: Σ over pins of J_pinᵀ J_pin (+ ε I) — "how far do my pins move". */
function metricBlock(b: Body): number[][] {
  const B = [[METRIC_REG, 0, 0], [0, METRIC_REG, 0], [0, 0, METRIC_REG]];
  for (let i = 0; i < b.pins.length; i++) {
    const p = worldPt(b, i);
    const rx = p[0] - b.pose.x, ry = p[1] - b.pose.y;
    B[0][0] += 1; B[1][1] += 1;
    B[0][2] -= ry; B[2][0] -= ry;
    B[1][2] += rx; B[2][1] += rx;
    B[2][2] += rx * rx + ry * ry;
  }
  return B;
}

function applyStep(m: Mechanism, sys: ConstraintSystem, dq: number[], maxStep: number): number {
  let big = 0;
  for (const v of dq) big = Math.max(big, Math.abs(v));
  const scale = big > maxStep ? maxStep / big : 1;
  m.bodies.forEach((b, i) => {
    const k = sys.varBase[i];
    if (k < 0) return;
    b.pose.x += dq[k] * scale;
    b.pose.y += dq[k + 1] * scale;
    b.pose.phi = wrapToPi(b.pose.phi + dq[k + 2] * scale);
  });
  return big * scale;
}

/**
 * Project the mechanism back onto the constraint manifold with M-weighted damped
 * Gauss–Newton steps dq = −W Jᵀ (J W Jᵀ + λI)⁻¹ C, W = M⁻¹ (minimal pin motion).
 * Returns the final max |residual|.
 */
export function project(m: Mechanism, maxIter = 30, tol = 1e-11): number {
  for (let it = 0; it < maxIter; it++) {
    const sys = constraintSystem(m);
    const res = maxAbsVec(sys.C);
    if (res < tol || sys.nVars === 0 || sys.C.length === 0) return res;
    const n = sys.nVars, rows = sys.C.length;
    // JW = J · blockdiag(M_b⁻¹)
    const JW: number[][] = sys.J.map((r) => r.slice());
    m.bodies.forEach((b, bi) => {
      const k = sys.varBase[bi];
      if (k < 0) return;
      const Wb = inv(metricBlock(b));
      for (const [ri, r] of sys.J.entries()) {
        const x0 = r[k], x1 = r[k + 1], x2 = r[k + 2];
        if (x0 === 0 && x1 === 0 && x2 === 0) continue;
        for (let c = 0; c < 3; c++) JW[ri][k + c] = x0 * Wb[0][c] + x1 * Wb[1][c] + x2 * Wb[2][c];
      }
    });
    const A: number[][] = Array.from({ length: rows }, () => new Array<number>(rows).fill(0));
    for (let r = 0; r < rows; r++) {
      for (let s = r; s < rows; s++) {
        let acc = 0;
        const a = JW[r], bRow = sys.J[s];
        for (let i = 0; i < n; i++) acc += a[i] * bRow[i];
        A[r][s] = acc; A[s][r] = acc;
      }
      A[r][r] += 1e-6;
    }
    let y: number[];
    try { y = solve(A, sys.C); } catch { return res; }
    const dq = new Array<number>(n).fill(0);
    for (let r = 0; r < rows; r++) {
      const yr = y[r];
      if (yr === 0) continue;
      const a = JW[r];
      for (let i = 0; i < n; i++) dq[i] -= a[i] * yr;
    }
    if (dq.some((v) => !Number.isFinite(v))) return res;
    applyStep(m, sys, dq, 0.3);
  }
  return maxAbsVec(constraintSystem(m).C);
}

export interface DragOpts {
  /** Maximum iterations per call (default 12). */
  maxIter?: number;
  /** Largest pin displacement requested per iteration, world units (default 0.05). */
  clamp?: number;
}

export interface DragResult {
  /** Remaining distance between the pin and the target. */
  remaining: number;
  /** True when the pin could not move at all towards the target. */
  locked: boolean;
}

/** One KKT step: move pin `ref` towards `target` within the tangent space (also corrects drift). */
function kktStep(m: Mechanism, ref: PointRef, target: Pt, clampLen: number): void {
  const sys = constraintSystem(m);
  const n = sys.nVars, rows = sys.C.length;
  const k = sys.varBase[ref.body];
  if (k < 0 || n === 0) return;
  const body = m.bodies[ref.body];
  const p = worldPt(body, ref.pin);
  let rx = p[0] - target[0], ry = p[1] - target[1];
  const dist = Math.hypot(rx, ry);
  if (dist > clampLen) { rx *= clampLen / dist; ry *= clampLen / dist; }
  const lx = p[0] - body.pose.x, ly = p[1] - body.pose.y;
  // Drag rows: d(pin)/dq for the dragged body (x row and y row).
  const d0 = [1, 0, -ly], d1 = [0, 1, lx];
  const size = n + rows;
  const K: number[][] = Array.from({ length: size }, () => new Array<number>(size).fill(0));
  const rhs = new Array<number>(size).fill(0);
  const mu = 1e-3;
  // H = (drag rows)ᵀ(drag rows) + μ M
  m.bodies.forEach((b, bi) => {
    const kb = sys.varBase[bi];
    if (kb < 0) return;
    const B = metricBlock(b);
    for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) K[kb + r][kb + c] += mu * B[r][c];
  });
  for (let r = 0; r < 3; r++) {
    for (let c = 0; c < 3; c++) K[k + r][k + c] += d0[r] * d0[c] + d1[r] * d1[c];
    rhs[k + r] = -(d0[r] * rx + d1[r] * ry);
  }
  // Constraint blocks [Jᵀ; J  −εI]; right-hand side −C keeps the step on the manifold.
  const eps = 1e-8;
  for (let r = 0; r < rows; r++) {
    const Jr = sys.J[r];
    for (let i = 0; i < n; i++) { const v = Jr[i]; if (v !== 0) { K[i][n + r] = v; K[n + r][i] = v; } }
    K[n + r][n + r] = -eps;
    rhs[n + r] = -sys.C[r];
  }
  let x: number[];
  try { x = solve(K, rhs, 1e-16); } catch { return; }
  const dq = x.slice(0, n);
  if (dq.some((v) => !Number.isFinite(v))) return;
  applyStep(m, sys, dq, 0.2);
}

/**
 * Move pin `ref` towards `target` while keeping every joint assembled. Small clamped
 * steps + projection keep closed chains on their current assembly branch.
 */
export function dragPin(m: Mechanism, ref: PointRef, target: Pt, o: DragOpts = {}): DragResult {
  const body = m.bodies[ref.body];
  if (!body || body.fixed) return { remaining: Infinity, locked: true };
  const maxIter = o.maxIter ?? 12;
  const clampLen = o.clamp ?? 0.05;
  const distOf = () => { const p = worldPt(body, ref.pin); return Math.hypot(p[0] - target[0], p[1] - target[1]); };
  const start = worldPt(body, ref.pin);
  let prev = distOf();
  for (let it = 0; it < maxIter; it++) {
    if (prev < 1e-5) break;
    const saved = cloneMechanism(m);
    kktStep(m, ref, target, clampLen);
    project(m, 8);
    let d = distOf();
    if (!(d < prev - 1e-7)) {
      // First-order step made no progress (e.g. a dead-centre position): probe along
      // each tangent direction and keep the best second-order improvement, if any.
      restore(m, saved);
      d = prev;
      const an = analyze(m);
      let best: Mechanism | null = null;
      for (const v of an.basis) {
        for (const sgn of [1, -1]) {
          const trial = cloneMechanism(m);
          applyStep(trial, an.sys, v.map((x) => sgn * x * 0.04), 0.04);
          project(trial, 12);
          const bp = worldPt(trial.bodies[ref.body], ref.pin);
          const td = Math.hypot(bp[0] - target[0], bp[1] - target[1]);
          if (td < d - 1e-6 && maxAbsVec(constraintSystem(trial).C) < 1e-8) { d = td; best = trial; }
        }
      }
      if (!best) break;
      restore(m, best);
    }
    prev = d;
  }
  const end = worldPt(body, ref.pin);
  const moved = Math.hypot(end[0] - start[0], end[1] - start[1]);
  const remaining = distOf();
  return { remaining, locked: moved < 1e-7 && remaining > 1e-4 };
}

function restore(m: Mechanism, src: Mechanism): void {
  src.bodies.forEach((b, i) => { m.bodies[i].pose = { ...b.pose }; });
}

// ---------------------------------------------------------------------------
// Point groups (pins merged by R joints) and drawing roles
// ---------------------------------------------------------------------------

export interface PointGroup {
  members: PointRef[];
  /** True when one of the members belongs to the fixed body. */
  fixed: boolean;
  /** Indices of R joints located at this point. */
  rJoints: number[];
  /** Indices of P joints whose sliding pin is in this group. */
  sliders: number[];
  /** True when every pin of the group only serves as the reference point of a P rail. */
  railOnly: boolean;
}

export function pointGroups(m: Mechanism): PointGroup[] {
  const keyOf = (b: number, p: number) => `${b}:${p}`;
  const parent = new Map<string, string>();
  const find = (k: string): string => {
    let r = k;
    while (parent.get(r) !== r) r = parent.get(r)!;
    let c = k;
    while (parent.get(c) !== r) { const nx = parent.get(c)!; parent.set(c, r); c = nx; }
    return r;
  };
  m.bodies.forEach((b, bi) => b.pins.forEach((_, pi) => parent.set(keyOf(bi, pi), keyOf(bi, pi))));
  for (const j of m.joints) {
    if (j.type !== 'R') continue;
    const ra = find(keyOf(j.a, j.pa)), rb = find(keyOf(j.b, j.pb));
    if (ra !== rb) parent.set(ra, rb);
  }
  const usedR = new Set<string>(), usedSlider = new Set<string>(), usedRail = new Set<string>();
  for (const j of m.joints) {
    if (j.type === 'R') { usedR.add(keyOf(j.a, j.pa)); usedR.add(keyOf(j.b, j.pb)); }
    else { usedRail.add(keyOf(j.a, j.pa)); usedSlider.add(keyOf(j.b, j.pb)); }
  }
  const groups = new Map<string, PointGroup>();
  m.bodies.forEach((b, bi) => b.pins.forEach((_, pi) => {
    const k = keyOf(bi, pi);
    const r = find(k);
    let g = groups.get(r);
    if (!g) { g = { members: [], fixed: false, rJoints: [], sliders: [], railOnly: true }; groups.set(r, g); }
    g.members.push({ body: bi, pin: pi });
    if (b.fixed) g.fixed = true;
    if (usedR.has(k) || usedSlider.has(k) || !usedRail.has(k)) g.railOnly = false;
  }));
  m.joints.forEach((j, ji) => {
    if (j.type === 'R') groups.get(find(keyOf(j.a, j.pa)))?.rJoints.push(ji);
    else groups.get(find(keyOf(j.b, j.pb)))?.sliders.push(ji);
  });
  return [...groups.values()];
}

/** World position of a group (all members coincide when assembled). */
export function groupPos(m: Mechanism, g: PointGroup): Pt {
  return worldPt(m.bodies[g.members[0].body], g.members[0].pin);
}

/** Pins that define a body's drawn shape (everything except pure rail reference points). */
export function shapePins(m: Mechanism, bi: number, groups: PointGroup[] = pointGroups(m)): number[] {
  const hidden = new Set<number>();
  for (const g of groups) if (g.railOnly) for (const r of g.members) if (r.body === bi) hidden.add(r.pin);
  return m.bodies[bi].pins.map((_, i) => i).filter((i) => !hidden.has(i));
}

/** Unit direction of a body's long axis (two farthest shape pins), or its x axis. */
export function bodyAxis(m: Mechanism, bi: number): Pt {
  const b = m.bodies[bi];
  const pins = shapePins(m, bi).map((i) => worldPt(b, i));
  let best = 0; let dir: Pt = worldDir(b, [1, 0]);
  for (let i = 0; i < pins.length; i++) {
    for (let j = i + 1; j < pins.length; j++) {
      const dx = pins[j][0] - pins[i][0], dy = pins[j][1] - pins[i][1];
      const L = Math.hypot(dx, dy);
      if (L > best + 1e-9) { best = L; dir = [dx / L, dy / L]; }
    }
  }
  return dir;
}

// ---------------------------------------------------------------------------
// Editing operations used by the sandbox (they mutate `m`; callers clone first)
// ---------------------------------------------------------------------------

/** Add (or reuse) a pin of body `bi` located at world point p. */
export function addPinAt(m: Mechanism, bi: number, p: Pt): number {
  const b = m.bodies[bi];
  for (let i = 0; i < b.pins.length; i++) {
    const q = worldPt(b, i);
    if (Math.hypot(q[0] - p[0], q[1] - p[1]) < 1e-6) return i;
  }
  b.pins.push(toLocal(b, p));
  return b.pins.length - 1;
}

export type Endpoint = { ref: PointRef } | { free: Pt };

function endpointPos(m: Mechanism, e: Endpoint): Pt {
  return 'ref' in e ? worldPt(m.bodies[e.ref.body], e.ref.pin) : e.free;
}

/**
 * Add a binary link between two endpoints. Endpoints that reference an existing pin
 * get an R joint; free endpoints become loose ends. Returns the new body index or null.
 */
export function addLink(m: Mechanism, e1: Endpoint, e2: Endpoint, name: string): number | null {
  if (m.bodies.length >= MAX_BODIES) return null;
  const p = endpointPos(m, e1), q = endpointPos(m, e2);
  const L = Math.hypot(q[0] - p[0], q[1] - p[1]);
  if (L < 0.08) return null;
  const body: Body = {
    name,
    pins: [[-L / 2, 0], [L / 2, 0]],
    pose: { x: (p[0] + q[0]) / 2, y: (p[1] + q[1]) / 2, phi: Math.atan2(q[1] - p[1], q[0] - p[0]) },
    fixed: false,
  };
  m.bodies.push(body);
  const bi = m.bodies.length - 1;
  if ('ref' in e1) m.joints.push({ type: 'R', a: e1.ref.body, b: bi, pa: e1.ref.pin, pb: 0 });
  if ('ref' in e2) m.joints.push({ type: 'R', a: e2.ref.body, b: bi, pa: e2.ref.pin, pb: 1 });
  return bi;
}

/** R joint between two pins of different bodies (the solver then pulls them together). */
export function addRJoint(m: Mechanism, a: PointRef, b: PointRef): boolean {
  if (a.body === b.body) return false;
  const dup = m.joints.some((j) => j.type === 'R' && (
    (j.a === a.body && j.pa === a.pin && j.b === b.body && j.pb === b.pin)
    || (j.a === b.body && j.pa === b.pin && j.b === a.body && j.pb === a.pin)));
  if (dup) return false;
  m.joints.push({ type: 'R', a: a.body, b: b.body, pa: a.pin, pb: b.pin });
  return true;
}

/**
 * P joint: pin `slider` slides along a line fixed on body `rail` that passes through
 * the pin's current position, parallel to the rail's long axis. The current relative
 * orientation is locked, so the joint is satisfied immediately.
 */
export function addPJoint(m: Mechanism, slider: PointRef, rail: number): boolean {
  if (slider.body === rail || !m.bodies[rail]) return false;
  const A = m.bodies[rail], B = m.bodies[slider.body];
  const p = worldPt(B, slider.pin);
  const axis = bodyAxis(m, rail);
  const pa = addPinAt(m, rail, p);
  const c = Math.cos(A.pose.phi), s = Math.sin(A.pose.phi);
  const dirLocal: Pt = [c * axis[0] + s * axis[1], -s * axis[0] + c * axis[1]];
  m.joints.push({ type: 'P', a: rail, b: slider.body, pa, pb: slider.pin, dir: dirLocal, dphi: wrapToPi(B.pose.phi - A.pose.phi), extent: [-0.55, 0.55] });
  return true;
}

/** Remove a moving body and every joint attached to it. The fixed body cannot be removed. */
export function removeBody(m: Mechanism, bi: number): boolean {
  if (bi < 0 || bi >= m.bodies.length || m.bodies[bi].fixed) return false;
  m.bodies.splice(bi, 1);
  m.joints = m.joints.filter((j) => j.a !== bi && j.b !== bi).map((j) => ({
    ...j,
    a: j.a > bi ? j.a - 1 : j.a,
    b: j.b > bi ? j.b - 1 : j.b,
  }));
  return true;
}

export function removeJoint(m: Mechanism, ji: number): boolean {
  if (ji < 0 || ji >= m.joints.length) return false;
  m.joints.splice(ji, 1);
  return true;
}

/** Re-centre a body frame on the centroid of its pins (rotation unchanged). */
function recenter(b: Body): void {
  if (!b.pins.length) return;
  const w = b.pins.map((_, i) => worldPt(b, i));
  const cx = w.reduce((s, p) => s + p[0], 0) / w.length;
  const cy = w.reduce((s, p) => s + p[1], 0) / w.length;
  b.pose = { x: cx, y: cy, phi: b.pose.phi };
  b.pins = w.map((p) => toLocal(b, p));
}

/** Make body `bi` the (only) fixed body — a kinematic inversion. Returns false if refused. */
export function setGround(m: Mechanism, bi: number): boolean {
  const cur = fixedBody(m);
  if (bi < 0 || bi >= m.bodies.length || bi === cur) return false;
  if (cur >= 0 && m.bodies[cur].pins.length === 0) return false;
  m.bodies.forEach((b, i) => { b.fixed = i === bi; });
  if (cur >= 0) recenter(m.bodies[cur]);
  return true;
}

// ---------------------------------------------------------------------------
// Builder: describe a mechanism with world-space points, get local pins back.
// ---------------------------------------------------------------------------

interface BuildBody { name: string; pts: Pt[]; fixed: boolean; phi?: number }
type BuildJoint =
  | { type: 'R'; a: number; b: number; p: Pt }
  | { type: 'P'; a: number; b: number; p: Pt; dir: Pt; ref: Pt; extent?: [number, number] };

export class MechanismBuilder {
  private bodies: BuildBody[] = [];
  private joints: BuildJoint[] = [];

  /** The fixed body (frame = world frame). `pts` are its anchor points. */
  ground(pts: Pt[] = []): number { return this.add('地面', pts, true, 0); }

  /** A moving body; its frame sits at the centroid of `pts`, x axis from pts[0] to pts[1] (or `phi`). */
  body(name: string, pts: Pt[], phi?: number): number { return this.add(name, pts, false, phi); }

  private add(name: string, pts: Pt[], fixed: boolean, phi?: number): number {
    this.bodies.push({ name, pts: pts.map((p) => [p[0], p[1]] as Pt), fixed, phi });
    return this.bodies.length - 1;
  }

  /** Revolute joint between bodies a and b at world point p (pins are added when missing). */
  R(a: number, b: number, p: Pt): this { this.joints.push({ type: 'R', a, b, p }); return this; }

  /** Prismatic joint: b's pin at p slides along world direction `dir` on a line of body a through `ref` (default p). */
  P(a: number, b: number, p: Pt, dir: Pt, o: { ref?: Pt; extent?: [number, number] } = {}): this {
    this.joints.push({ type: 'P', a, b, p, dir, ref: o.ref ?? p, extent: o.extent });
    return this;
  }

  build(): Mechanism {
    const pinIndex = (bb: BuildBody, p: Pt): number => {
      const i = bb.pts.findIndex((q) => Math.hypot(q[0] - p[0], q[1] - p[1]) < 1e-9);
      if (i >= 0) return i;
      bb.pts.push([p[0], p[1]]);
      return bb.pts.length - 1;
    };
    const pinIdx = this.joints.map((j) => (j.type === 'R'
      ? { pa: pinIndex(this.bodies[j.a], j.p), pb: pinIndex(this.bodies[j.b], j.p) }
      : { pa: pinIndex(this.bodies[j.a], j.ref), pb: pinIndex(this.bodies[j.b], j.p) }));
    const bodies: Body[] = this.bodies.map((bb) => {
      let pose: Pose = { x: 0, y: 0, phi: 0 };
      if (!bb.fixed && bb.pts.length) {
        const n = bb.pts.length;
        const cx = bb.pts.reduce((s, p) => s + p[0], 0) / n;
        const cy = bb.pts.reduce((s, p) => s + p[1], 0) / n;
        let phi = bb.phi ?? 0;
        if (bb.phi == null && n >= 2) phi = Math.atan2(bb.pts[1][1] - bb.pts[0][1], bb.pts[1][0] - bb.pts[0][0]);
        pose = { x: cx, y: cy, phi };
      }
      const body: Body = { name: bb.name, pins: [], pose, fixed: bb.fixed };
      body.pins = bb.pts.map((p) => toLocal(body, p));
      return body;
    });
    const joints: Joint[] = this.joints.map((j, ji) => {
      const { pa, pb } = pinIdx[ji];
      if (j.type === 'R') return { type: 'R', a: j.a, b: j.b, pa, pb };
      const A = bodies[j.a], B = bodies[j.b];
      const L = Math.hypot(j.dir[0], j.dir[1]) || 1;
      const dw: Pt = [j.dir[0] / L, j.dir[1] / L];
      const c = Math.cos(A.pose.phi), s = Math.sin(A.pose.phi);
      return { type: 'P', a: j.a, b: j.b, pa, pb, dir: [c * dw[0] + s * dw[1], -s * dw[0] + c * dw[1]], dphi: wrapToPi(B.pose.phi - A.pose.phi), extent: j.extent };
    });
    return { bodies, joints };
  }
}

// ---------------------------------------------------------------------------
// Presets
// ---------------------------------------------------------------------------

export type PresetKey =
  | 'free' | 'twoFree' | 'twoR' | 'twoP' | 'fourbar' | 'slider' | 'fivebar' | 'sixbar'
  | 'parallelogram' | 'stacked' | 'stackedSkew' | 'truss' | 'arm2R';

export interface World2 { xmin: number; xmax: number; ymin: number; ymax: number }

export interface PresetDef { key: PresetKey; label: string; title: string; world?: World2; build: () => Mechanism }

export const DEFAULT_WORLD: World2 = { xmin: -2.25, xmax: 2.25, ymin: -0.45, ymax: 1.95 };

const DEG = Math.PI / 180;
const polar = (c: Pt, r: number, a: number): Pt => [c[0] + r * Math.cos(a), c[1] + r * Math.sin(a)];

/** 2R arm used by the last step (base, link lengths, initial joint angles). */
export const ARM2R = { base: [0, 0.35] as Pt, L: [0.85, 0.65] as [number, number], theta0: [35 * DEG, 50 * DEG] as [number, number] };

function arm2RPoints(thetas: [number, number]): Pt[] {
  const fk = planarFK(ARM2R.L, thetas);
  return fk.joints.map((q) => [q[0] + ARM2R.base[0], q[1] + ARM2R.base[1]] as Pt);
}

export const PRESETS: PresetDef[] = [
  {
    key: 'free', label: '單一自由桿', title: '一根沒有任何關節的桿：N = 2（含地面）、J = 0',
    build: () => { const b = new MechanismBuilder(); b.ground(); b.body('桿', [[-0.85, 0.62], [0.85, 1.2]]); return b.build(); },
  },
  {
    key: 'twoFree', label: '兩根自由桿', title: '兩根互不相連的桿：3 + 3 = 6',
    build: () => {
      const b = new MechanismBuilder(); b.ground();
      b.body('桿 1', [[-1.5, 0.55], [0.15, 0.95]]);
      b.body('桿 2', [[0.5, 1.05], [1.55, 0.45]]);
      return b.build();
    },
  },
  {
    key: 'twoR', label: '兩桿 + R', title: '兩根桿用一個旋轉關節相接',
    build: () => {
      const b = new MechanismBuilder(); b.ground();
      const l1 = b.body('桿 1', [[-1.5, 0.55], [0.2, 0.98]]);
      const l2 = b.body('桿 2', [[0.2, 0.98], [1.35, 0.4]]);
      b.R(l1, l2, [0.2, 0.98]);
      return b.build();
    },
  },
  {
    key: 'twoP', label: '兩桿 + P', title: '直桿沿著另一根桿滑動（相對角度鎖住）',
    build: () => {
      const b = new MechanismBuilder(); b.ground();
      const rail = b.body('桿 1（滑軌）', [[-1.5, 0.5], [1.1, 0.5]]);
      const mast = b.body('桿 2', [[-0.2, 0.5], [-0.2, 1.45]]);
      b.P(rail, mast, [-0.2, 0.5], [1, 0]);
      return b.build();
    },
  },
  {
    key: 'fourbar', label: '四連桿', title: 'four-bar linkage（曲柄搖桿）',
    build: () => {
      const O1: Pt = [-0.9, 0], O2: Pt = [0.9, 0];
      const A = polar(O1, 0.5, 70 * DEG);
      const B = circleIntersect(A, 1.5, O2, 1.2, true)!;
      const b = new MechanismBuilder();
      const g = b.ground([O1, O2]);
      const crank = b.body('曲柄', [O1, A]);
      const coupler = b.body('連接桿', [A, B]);
      const rocker = b.body('搖桿', [B, O2]);
      b.R(g, crank, O1).R(crank, coupler, A).R(coupler, rocker, B).R(rocker, g, O2);
      return b.build();
    },
  },
  {
    key: 'slider', label: '曲柄滑塊', title: 'crank–slider：3 個 R + 1 個 P',
    build: () => {
      const O: Pt = [-1.25, 0];
      const A = polar(O, 0.5, 60 * DEG);
      const B: Pt = [A[0] + Math.sqrt(1.5 * 1.5 - A[1] * A[1]), 0];
      const b = new MechanismBuilder();
      const g = b.ground([O]);
      const crank = b.body('曲柄', [O, A]);
      const rod = b.body('連桿', [A, B]);
      const block = b.body('滑塊', [B], 0);
      b.R(g, crank, O).R(crank, rod, A).R(rod, block, B).P(g, block, B, [1, 0], { ref: [0.3, 0], extent: [-0.85, 0.75] });
      return b.build();
    },
  },
  {
    key: 'fivebar', label: '五連桿', title: 'five-bar linkage：兩個接地馬達、一個頂點',
    build: () => {
      const O1: Pt = [-0.75, 0], O2: Pt = [0.75, 0];
      const A = polar(O1, 0.8, 115 * DEG), B = polar(O2, 0.8, 65 * DEG);
      const C = circleIntersect(A, 1.25, B, 1.25, true)!;
      const b = new MechanismBuilder();
      const g = b.ground([O1, O2]);
      const l1 = b.body('桿 1', [O1, A]);
      const l2 = b.body('桿 2', [A, C]);
      const l3 = b.body('桿 3', [C, B]);
      const l4 = b.body('桿 4', [B, O2]);
      b.R(g, l1, O1).R(l1, l2, A).R(l2, l3, C).R(l3, l4, B).R(l4, g, O2);
      return b.build();
    },
  },
  {
    key: 'sixbar', label: '六連桿', title: 'Stephenson 型六連桿：四連桿的三角形連接桿再帶一組雙桿',
    build: () => {
      // Grashof crank–rocker (0.45 / 1.15 / 0.85 on a 1.4 base); the 1.5 / 1.3 dyad keeps a ≥ 0.49
      // margin from full extension/folding over a whole crank turn, so the crank can spin freely.
      const O1: Pt = [-1.35, 0], O2: Pt = [0.05, 0], O3: Pt = [0.9, 0];
      const A = polar(O1, 0.45, 75 * DEG);
      const B = circleIntersect(A, 1.15, O2, 0.85, true)!;
      const L = Math.hypot(B[0] - A[0], B[1] - A[1]);
      const u: Pt = [(B[0] - A[0]) / L, (B[1] - A[1]) / L];
      const C: Pt = [(A[0] + B[0]) / 2 - 0.3 * u[1], (A[1] + B[1]) / 2 + 0.3 * u[0]];
      const D = circleIntersect(C, 1.5, O3, 1.3, true)!;
      const b = new MechanismBuilder();
      const g = b.ground([O1, O2, O3]);
      const crank = b.body('曲柄', [O1, A]);
      const coupler = b.body('三角連接桿', [A, B, C]);
      const rocker = b.body('搖桿', [B, O2]);
      const l5 = b.body('桿 5', [C, D]);
      const l6 = b.body('桿 6', [D, O3]);
      b.R(g, crank, O1).R(crank, coupler, A).R(coupler, rocker, B).R(rocker, g, O2);
      b.R(coupler, l5, C).R(l5, l6, D).R(l6, g, O3);
      return b.build();
    },
  },
  {
    key: 'parallelogram', label: '平行四邊形', title: '平行四邊形四連桿：連接桿始終保持水平',
    build: () => {
      const b = new MechanismBuilder();
      const g = b.ground([[-1, 0], [1, 0]]);
      const l1 = b.body('桿 1', [[-1, 0], [-0.6, 1.0]]);
      const coupler = b.body('連接桿', [[-0.6, 1.0], [1.4, 1.0]]);
      const l2 = b.body('桿 2', [[1, 0], [1.4, 1.0]]);
      b.R(g, l1, [-1, 0]).R(l1, coupler, [-0.6, 1.0]).R(coupler, l2, [1.4, 1.0]).R(l2, g, [1, 0]);
      return b.build();
    },
  },
  {
    key: 'stacked', label: '平行四邊形疊加', title: '三根等長且平行的桿：Grübler 給 0，機構卻能動',
    build: () => {
      const b = new MechanismBuilder();
      const g = b.ground([[-1, 0], [0, 0], [1, 0]]);
      const l1 = b.body('桿 1', [[-1, 0], [-0.6, 1.0]]);
      const coupler = b.body('連接桿', [[-0.6, 1.0], [1.4, 1.0]]);
      const l2 = b.body('桿 2', [[1, 0], [1.4, 1.0]]);
      const l3 = b.body('中間桿', [[0, 0], [0.4, 1.0]]);
      b.R(g, l1, [-1, 0]).R(l1, coupler, [-0.6, 1.0]).R(coupler, l2, [1.4, 1.0]).R(l2, g, [1, 0]);
      b.R(g, l3, [0, 0]).R(l3, coupler, [0.4, 1.0]);
      return b.build();
    },
  },
  {
    key: 'stackedSkew', label: '中間桿歪一點', title: '同樣的 N、J，但中間桿的支點右移 0.25：不再平行',
    build: () => {
      const b = new MechanismBuilder();
      const g = b.ground([[-1, 0], [0.25, 0], [1, 0]]);
      const l1 = b.body('桿 1', [[-1, 0], [-0.6, 1.0]]);
      const coupler = b.body('連接桿', [[-0.6, 1.0], [1.4, 1.0]]);
      const l2 = b.body('桿 2', [[1, 0], [1.4, 1.0]]);
      const l3 = b.body('中間桿', [[0.25, 0], [0.4, 1.0]]);
      b.R(g, l1, [-1, 0]).R(l1, coupler, [-0.6, 1.0]).R(coupler, l2, [1.4, 1.0]).R(l2, g, [1, 0]);
      b.R(g, l3, [0.25, 0]).R(l3, coupler, [0.4, 1.0]);
      return b.build();
    },
  },
  {
    key: 'truss', label: '三角桁架', title: '兩根桿與地面圍成三角形：dof = 0，是結構不是機構',
    build: () => {
      const b = new MechanismBuilder();
      const g = b.ground([[-0.8, 0], [0.8, 0]]);
      const l1 = b.body('桿 1', [[-0.8, 0], [0, 1.3]]);
      const l2 = b.body('桿 2', [[0, 1.3], [0.8, 0]]);
      b.R(g, l1, [-0.8, 0]).R(l1, l2, [0, 1.3]).R(l2, g, [0.8, 0]);
      return b.build();
    },
  },
  {
    key: 'arm2R', label: '2R 手臂', title: '開鏈 2R 手臂：組態 (θ₁, θ₂)',
    world: { xmin: -2.25, xmax: 2.25, ymin: -1.25, ymax: 1.95 },
    build: () => {
      const P = arm2RPoints(ARM2R.theta0);
      const b = new MechanismBuilder();
      const g = b.ground([P[0]]);
      const l1 = b.body('連桿 1', [P[0], P[1]]);
      const l2 = b.body('連桿 2', [P[1], P[2]]);
      b.R(g, l1, P[0]).R(l1, l2, P[1]);
      return b.build();
    },
  },
];

export function presetByKey(key: PresetKey): PresetDef {
  const p = PRESETS.find((d) => d.key === key);
  if (!p) throw new Error(`unknown preset ${key}`);
  return p;
}

// ---------------------------------------------------------------------------
// 2R arm helpers (bodies 1 and 2 of the arm2R preset)
// ---------------------------------------------------------------------------

/** Joint angles (θ₁, θ₂) read back from the body orientations of the 2R preset. */
export function arm2RAngles(m: Mechanism): [number, number] {
  const g = m.bodies[0].pose.phi;
  return [wrapToPi(m.bodies[1].pose.phi - g), wrapToPi(m.bodies[2].pose.phi - m.bodies[1].pose.phi)];
}

/** Place the 2R preset at joint angles `thetas` (forward kinematics from @/mr). */
export function setArm2R(m: Mechanism, thetas: [number, number]): void {
  const P = arm2RPoints(thetas);
  m.bodies[1].pose = { x: (P[0][0] + P[1][0]) / 2, y: (P[0][1] + P[1][1]) / 2, phi: wrapToPi(thetas[0]) };
  m.bodies[2].pose = { x: (P[1][0] + P[2][0]) / 2, y: (P[1][1] + P[2][1]) / 2, phi: wrapToPi(thetas[0] + thetas[1]) };
}
