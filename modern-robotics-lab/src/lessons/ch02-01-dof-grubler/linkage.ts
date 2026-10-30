/**
 * Planar linkage model for the dof / Grübler lesson.
 *
 * A mechanism is a list of rigid bodies (each with a pose and a set of attachment
 * "pins" in body coordinates) and a list of R / P joints between pins. Body 0 is
 * the ground by convention. Everything here is plain data + pure functions so the
 * counting and the constraint solver can be unit tested without a DOM.
 *
 * Besides Grübler's count the module computes the *instantaneous* mobility
 * 3·(moving bodies) − rank(constraint Jacobian), which is what the lesson uses to
 * expose the cases where the formula's independence assumption fails
 * (stacked parallelograms).
 */
import { svd, solve, wrapToPi, type Mat } from '@/mr';

export type Pt = [number, number];
export interface Pose { x: number; y: number; phi: number }

export interface Body {
  name: string;
  /** Attachment pins in body coordinates (joint anchors and spare pins). */
  pins: Pt[];
  pose: Pose;
  fixed: boolean;
}

/** Pin `pa` of body a and pin `pb` of body b coincide (2 scalar constraints). */
export interface JointR { type: 'R'; a: number; b: number; pa: number; pb: number }
/**
 * Pin `pb` of body b slides along the line through pin `pa` of body a whose
 * direction is `dir` in a's frame; the relative orientation is locked to `dphi`
 * (2 scalar constraints).
 */
export interface JointP { type: 'P'; a: number; b: number; pa: number; pb: number; dir: Pt; dphi: number }
export type Joint = JointR | JointP;

export interface Mechanism { bodies: Body[]; joints: Joint[] }

/** Planar bodies have m = 3 freedoms. */
export const M_PLANAR = 3;

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

export function worldPins(b: Body): Pt[] {
  return b.pins.map((_, i) => worldPt(b, i));
}

/** World point of a joint (average of the two pins, which coincide for R joints). */
export function jointPoint(m: Mechanism, j: Joint): Pt {
  if (j.type === 'R') {
    const p = worldPt(m.bodies[j.a], j.pa), q = worldPt(m.bodies[j.b], j.pb);
    return [(p[0] + q[0]) / 2, (p[1] + q[1]) / 2];
  }
  return worldPt(m.bodies[j.b], j.pb);
}

/** Map "body:pin" → number of joints using that pin. */
export function pinUsage(m: Mechanism): Map<string, number> {
  const u = new Map<string, number>();
  const bump = (b: number, p: number) => { const k = `${b}:${p}`; u.set(k, (u.get(k) ?? 0) + 1); };
  for (const j of m.joints) { bump(j.a, j.pa); bump(j.b, j.pb); }
  return u;
}

export function cloneMechanism(m: Mechanism): Mechanism {
  return {
    bodies: m.bodies.map((b) => ({ name: b.name, pins: b.pins.map((p) => [p[0], p[1]] as Pt), pose: { ...b.pose }, fixed: b.fixed })),
    joints: m.joints.map((j) => (j.type === 'R' ? { ...j } : { ...j, dir: [j.dir[0], j.dir[1]] as Pt })),
  };
}

// ---------------------------------------------------------------------------
// Counting
// ---------------------------------------------------------------------------

export interface GrublerCount { N: number; J: number; sumF: number; dof: number; m: number }

/** Grübler's formula. Every planar R and P joint keeps f_i = 1. */
export function grubler(m: Mechanism, dim = M_PLANAR): GrublerCount {
  const N = m.bodies.length, J = m.joints.length;
  const sumF = J;
  return { N, J, sumF, dof: dim * (N - 1 - J) + sumF, m: dim };
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
  /** Index of the first variable of each body, or −1 for a fixed body. */
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
      if (ba >= 0) { r0[ba] = 1; r0[ba + 2] = -rA[1]; r1[ba + 1] = 1; r1[ba + 2] = rA[0]; }
      if (bb >= 0) { r0[bb] = -1; r0[bb + 2] = rB[1]; r1[bb + 1] = -1; r1[bb + 2] = -rB[0]; }
    } else {
      const u = worldDir(A, j.dir);
      const n: Pt = [-u[1], u[0]];
      const d: Pt = [pB[0] - pA[0], pB[1] - pA[1]];
      const r0 = row(), r1 = row();
      C.push(n[0] * d[0] + n[1] * d[1], wrapToPi(B.pose.phi - A.pose.phi - j.dphi));
      if (ba >= 0) {
        r0[ba] = -n[0]; r0[ba + 1] = -n[1];
        // d n / d φ_a = perp(n); d pA / d φ_a = perp(rA)
        r0[ba + 2] = (-n[1] * d[0] + n[0] * d[1]) - (n[0] * -rA[1] + n[1] * rA[0]);
        r1[ba + 2] = -1;
      }
      if (bb >= 0) {
        r0[bb] = n[0]; r0[bb + 1] = n[1];
        r0[bb + 2] = n[0] * -rB[1] + n[1] * rB[0];
        r1[bb + 2] = 1;
      }
    }
  }
  return { C, J, nVars, varBase };
}

/** Numerical rank of the constraint Jacobian (relative tolerance on the singular values). */
export function constraintRank(J: Mat, relTol = 1e-6): number {
  if (J.length === 0 || (J[0]?.length ?? 0) === 0) return 0;
  const { S } = svd(J);
  const smax = S[0] ?? 0;
  if (smax <= 1e-12) return 0;
  return S.filter((s) => s > relTol * smax).length;
}

export interface Mobility { nVars: number; rank: number; mobility: number; maxResidual: number }

/** Instantaneous mobility = (#variables) − rank(J). */
export function mobility(m: Mechanism): Mobility {
  const sys = constraintSystem(m);
  const rank = constraintRank(sys.J);
  const maxResidual = sys.C.reduce((a, c) => Math.max(a, Math.abs(c)), 0);
  return { nVars: sys.nVars, rank, mobility: sys.nVars - rank, maxResidual };
}

// ---------------------------------------------------------------------------
// Solver: damped Gauss–Newton on the constraint residual with an optional
// soft "drag" target for one pin. Small steps + warm start keep the mechanism
// on its current assembly branch.
// ---------------------------------------------------------------------------

function applyStep(m: Mechanism, sys: ConstraintSystem, dq: number[], maxStep: number): void {
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
}

/**
 * One damped least-squares step. Rows = constraints (weight 1) plus, when a
 * target is given, the drag residual of pin (bi, pin) (weight `wDrag`).
 * Returns the drag residual length before the step (Infinity when no target).
 */
function gnStep(m: Mechanism, target: { bi: number; pin: number; p: Pt; w: number; clamp: number } | null, lambda: number, maxStep: number): number {
  const sys = constraintSystem(m);
  const n = sys.nVars;
  if (n === 0) return 0;
  const rows: number[][] = sys.J;
  const res: number[] = sys.C.slice();
  const w: number[] = sys.C.map(() => 1);
  let dragLen = Infinity;
  if (target) {
    const body = m.bodies[target.bi];
    const base = sys.varBase[target.bi];
    if (base >= 0) {
      const p = worldPt(body, target.pin);
      const r: Pt = [p[0] - body.pose.x, p[1] - body.pose.y];
      let dx = p[0] - target.p[0], dy = p[1] - target.p[1];
      dragLen = Math.hypot(dx, dy);
      if (dragLen > target.clamp) { dx *= target.clamp / dragLen; dy *= target.clamp / dragLen; }
      const d0 = new Array<number>(n).fill(0); d0[base] = 1; d0[base + 2] = -r[1];
      const d1 = new Array<number>(n).fill(0); d1[base + 1] = 1; d1[base + 2] = r[0];
      rows.push(d0, d1); res.push(dx, dy); w.push(target.w, target.w);
    }
  }
  // Normal equations (JᵀWJ + λI) dq = −JᵀW r
  const A: number[][] = Array.from({ length: n }, () => new Array<number>(n).fill(0));
  const g = new Array<number>(n).fill(0);
  rows.forEach((row, i) => {
    const wi = w[i], ri = res[i];
    for (let a = 0; a < n; a++) {
      const ra = row[a];
      if (ra === 0) continue;
      g[a] -= wi * ra * ri;
      for (let b = 0; b < n; b++) { const rb = row[b]; if (rb !== 0) A[a][b] += wi * ra * rb; }
    }
  });
  for (let i = 0; i < n; i++) A[i][i] += lambda;
  let dq: number[];
  try { dq = solve(A, g); } catch { return dragLen; }
  if (dq.some((v) => !Number.isFinite(v))) return dragLen;
  applyStep(m, sys, dq, maxStep);
  return dragLen;
}

/** Project the mechanism back onto the constraint manifold (Gauss–Newton). */
export function project(m: Mechanism, iters = 6, tol = 1e-10): number {
  let maxRes = 0;
  for (let it = 0; it < iters; it++) {
    const sys = constraintSystem(m);
    maxRes = sys.C.reduce((a, c) => Math.max(a, Math.abs(c)), 0);
    if (maxRes < tol) break;
    gnStep(m, null, 1e-6, 0.5);
  }
  return maxRes;
}

export interface DragOpts {
  /** Largest drag residual used per iteration (world units). Default 0.04. */
  clamp?: number;
  /** Maximum iterations per call. Default 40. */
  iters?: number;
  /** Weight of the drag rows relative to the constraints. Default 0.5. */
  weight?: number;
}

/**
 * Move pin `pin` of body `bi` towards `target` while keeping every joint
 * assembled. Returns the remaining distance to the target (0 when reached,
 * > 0 when the mechanism cannot get there).
 */
export function dragPin(m: Mechanism, bi: number, pin: number, target: Pt, o: DragOpts = {}): number {
  const body = m.bodies[bi];
  if (!body || body.fixed) return Infinity;
  const clamp = o.clamp ?? 0.04;
  const iters = o.iters ?? 40;
  const weight = o.weight ?? 0.5;
  let last = Infinity;
  for (let it = 0; it < iters; it++) {
    const d = gnStep(m, { bi, pin, p: target, w: weight, clamp }, 1e-6, 0.25);
    if (d < 1e-5) break;
    // Stuck (target unreachable from this branch): stop pulling.
    if (last - d < 1e-6 && it > 2) break;
    last = d;
  }
  project(m, 6);
  const p = worldPt(body, pin);
  return Math.hypot(p[0] - target[0], p[1] - target[1]);
}

// ---------------------------------------------------------------------------
// Editing operations used by the sandbox
// ---------------------------------------------------------------------------

/** Add a binary link between two existing pins (two new R joints). Returns the new body index. */
export function addBinaryLink(m: Mechanism, a: { body: number; pin: number }, b: { body: number; pin: number }, name: string): number {
  const p = worldPt(m.bodies[a.body], a.pin), q = worldPt(m.bodies[b.body], b.pin);
  const cx = (p[0] + q[0]) / 2, cy = (p[1] + q[1]) / 2;
  const body: Body = {
    name,
    pins: [[p[0] - cx, p[1] - cy], [q[0] - cx, q[1] - cy]],
    pose: { x: cx, y: cy, phi: 0 },
    fixed: false,
  };
  m.bodies.push(body);
  const bi = m.bodies.length - 1;
  m.joints.push({ type: 'R', a: a.body, b: bi, pa: a.pin, pb: 0 });
  m.joints.push({ type: 'R', a: b.body, b: bi, pa: b.pin, pb: 1 });
  return bi;
}

/** Remove a body and every joint attached to it. Body 0 (ground) cannot be removed. */
export function removeBody(m: Mechanism, bi: number): boolean {
  if (bi <= 0 || bi >= m.bodies.length) return false;
  m.bodies.splice(bi, 1);
  m.joints = m.joints.filter((j) => j.a !== bi && j.b !== bi).map((j) => ({
    ...j,
    a: j.a > bi ? j.a - 1 : j.a,
    b: j.b > bi ? j.b - 1 : j.b,
  })) as Joint[];
  return true;
}

/** Make body `bi` the (only) fixed body. */
export function setGround(m: Mechanism, bi: number): void {
  m.bodies.forEach((b, i) => { b.fixed = i === bi; });
}

// ---------------------------------------------------------------------------
// Builder: describe a mechanism with world-space points, get local pins back.
// ---------------------------------------------------------------------------

interface BuildBody { name: string; pts: Pt[]; fixed: boolean }
type BuildJoint = { type: 'R'; a: number; b: number; p: Pt } | { type: 'P'; a: number; b: number; p: Pt; dir: Pt };

export class MechanismBuilder {
  private bodies: BuildBody[] = [];
  private joints: BuildJoint[] = [];

  /** Body 0: the ground. `pts` are optional anchor points. */
  ground(pts: Pt[] = []): number { return this.body('地面', pts, true); }

  body(name: string, pts: Pt[], fixed = false): number {
    this.bodies.push({ name, pts: pts.map((p) => [p[0], p[1]]), fixed });
    return this.bodies.length - 1;
  }

  /** Revolute joint between bodies a and b at world point p (pins are added when missing). */
  R(a: number, b: number, p: Pt): this { this.joints.push({ type: 'R', a, b, p }); return this; }

  /** Prismatic joint: pin of b at p slides along the line through p with world direction dir fixed on a. */
  P(a: number, b: number, p: Pt, dir: Pt): this { this.joints.push({ type: 'P', a, b, p, dir }); return this; }

  build(): Mechanism {
    const pinIndex = (bb: BuildBody, p: Pt): number => {
      const i = bb.pts.findIndex((q) => Math.hypot(q[0] - p[0], q[1] - p[1]) < 1e-9);
      if (i >= 0) return i;
      bb.pts.push([p[0], p[1]]);
      return bb.pts.length - 1;
    };
    const joints: Joint[] = this.joints.map((j) => {
      const pa = pinIndex(this.bodies[j.a], j.p), pb = pinIndex(this.bodies[j.b], j.p);
      if (j.type === 'R') return { type: 'R', a: j.a, b: j.b, pa, pb };
      const L = Math.hypot(j.dir[0], j.dir[1]) || 1;
      return { type: 'P', a: j.a, b: j.b, pa, pb, dir: [j.dir[0] / L, j.dir[1] / L], dphi: 0 };
    });
    const bodies: Body[] = this.bodies.map((bb) => {
      const n = bb.pts.length;
      const cx = n ? bb.pts.reduce((s, p) => s + p[0], 0) / n : 0;
      const cy = n ? bb.pts.reduce((s, p) => s + p[1], 0) / n : 0;
      return { name: bb.name, pins: bb.pts.map((p) => [p[0] - cx, p[1] - cy] as Pt), pose: { x: cx, y: cy, phi: 0 }, fixed: bb.fixed };
    });
    return { bodies, joints };
  }
}

// ---------------------------------------------------------------------------
// Presets
// ---------------------------------------------------------------------------

export interface PresetDef { key: string; label: string; title: string; build: () => Mechanism }

export const PRESETS: PresetDef[] = [
  {
    key: 'free', label: '單一自由桿', title: '一根沒有任何關節的桿：N = 2（含地面）、J = 0',
    build: () => { const b = new MechanismBuilder(); b.ground(); b.body('桿 1', [[-0.7, 0.9], [0.7, 0.9]]); return b.build(); },
  },
  {
    key: 'twoR', label: '兩桿 + R', title: '兩根自由桿用一個旋轉關節相接',
    build: () => {
      const b = new MechanismBuilder(); b.ground();
      const l1 = b.body('桿 1', [[-1.3, 0.9], [0, 0.9]]);
      const l2 = b.body('桿 2', [[0, 0.9], [1.3, 0.9]]);
      b.R(l1, l2, [0, 0.9]);
      return b.build();
    },
  },
  {
    key: 'twoP', label: '兩桿 + P', title: '一個滑塊沿著另一根桿滑動',
    build: () => {
      const b = new MechanismBuilder(); b.ground();
      const rail = b.body('桿 1（滑軌）', [[-1.3, 0.9], [0.9, 0.9]]);
      const block = b.body('桿 2（滑塊）', [[-0.3, 0.9]]);
      b.P(rail, block, [-0.3, 0.9], [1, 0]);
      return b.build();
    },
  },
  {
    key: 'fourbar', label: '四連桿', title: 'four-bar linkage：dof = 1',
    build: () => {
      const b = new MechanismBuilder();
      const g = b.ground([[-1, 0], [1, 0]]);
      const crank = b.body('曲柄', [[-1, 0], [-0.6, 0.9]]);
      const coupler = b.body('連接桿', [[-0.6, 0.9], [0.9, 1.3]]);
      const rocker = b.body('搖桿', [[0.9, 1.3], [1, 0]]);
      b.R(g, crank, [-1, 0]).R(crank, coupler, [-0.6, 0.9]).R(coupler, rocker, [0.9, 1.3]).R(rocker, g, [1, 0]);
      return b.build();
    },
  },
  {
    key: 'slider', label: '曲柄滑塊', title: 'crank–slider：3 個 R + 1 個 P，dof = 1',
    build: () => {
      const b = new MechanismBuilder();
      const g = b.ground([[-1.3, 0], [1.6, 0]]);
      const crank = b.body('曲柄', [[-1.3, 0], [-0.8, 0.75]]);
      const rod = b.body('連桿', [[-0.8, 0.75], [0.9, 0]]);
      const slider = b.body('滑塊', [[0.9, 0]]);
      b.R(g, crank, [-1.3, 0]).R(crank, rod, [-0.8, 0.75]).R(rod, slider, [0.9, 0]).P(g, slider, [0.9, 0], [1, 0]);
      return b.build();
    },
  },
  {
    key: 'fivebar', label: '五連桿', title: 'five-bar linkage：dof = 2',
    build: () => {
      const b = new MechanismBuilder();
      const g = b.ground([[-1.2, 0], [1.2, 0]]);
      const l1 = b.body('桿 1', [[-1.2, 0], [-1.0, 0.95]]);
      const l2 = b.body('桿 2', [[-1.0, 0.95], [0.1, 1.6]]);
      const l3 = b.body('桿 3', [[0.1, 1.6], [1.05, 1.0]]);
      const l4 = b.body('桿 4', [[1.05, 1.0], [1.2, 0]]);
      b.R(g, l1, [-1.2, 0]).R(l1, l2, [-1.0, 0.95]).R(l2, l3, [0.1, 1.6]).R(l3, l4, [1.05, 1.0]).R(l4, g, [1.2, 0]);
      return b.build();
    },
  },
  {
    key: 'parallelogram', label: '平行四邊形', title: '平行四邊形四連桿，地面與連接桿各留一個備用接點',
    build: () => {
      const b = new MechanismBuilder();
      const g = b.ground([[-1, 0], [0, 0], [1, 0]]);
      const l1 = b.body('桿 1', [[-1, 0], [-0.6, 1.0]]);
      const coupler = b.body('連接桿', [[-0.6, 1.0], [0.4, 1.0], [1.4, 1.0]]);
      const l2 = b.body('桿 2', [[1, 0], [1.4, 1.0]]);
      b.R(g, l1, [-1, 0]).R(l1, coupler, [-0.6, 1.0]).R(coupler, l2, [1.4, 1.0]).R(l2, g, [1, 0]);
      return b.build();
    },
  },
  {
    key: 'stacked', label: '平行四邊形疊加', title: '三根平行桿：Grübler 給 0，機構卻能動',
    build: () => {
      const b = new MechanismBuilder();
      const g = b.ground([[-1, 0], [0, 0], [1, 0]]);
      const l1 = b.body('桿 1', [[-1, 0], [-0.6, 1.0]]);
      const coupler = b.body('連接桿', [[-0.6, 1.0], [0.4, 1.0], [1.4, 1.0]]);
      const l2 = b.body('桿 2', [[1, 0], [1.4, 1.0]]);
      const l3 = b.body('桿 3', [[0, 0], [0.4, 1.0]]);
      b.R(g, l1, [-1, 0]).R(l1, coupler, [-0.6, 1.0]).R(coupler, l2, [1.4, 1.0]).R(l2, g, [1, 0]);
      b.R(g, l3, [0, 0]).R(l3, coupler, [0.4, 1.0]);
      return b.build();
    },
  },
  {
    key: 'truss', label: '三角桁架', title: '三根桿圍成三角形：dof = 0，是結構不是機構',
    build: () => {
      const b = new MechanismBuilder();
      const g = b.ground([[-0.9, 0], [0.9, 0]]);
      const l1 = b.body('桿 1', [[-0.9, 0], [0, 1.3]]);
      const l2 = b.body('桿 2', [[0, 1.3], [0.9, 0]]);
      b.R(g, l1, [-0.9, 0]).R(l1, l2, [0, 1.3]).R(l2, g, [0.9, 0]);
      return b.build();
    },
  },
  {
    key: 'arm2R', label: '2R 手臂', title: '開鏈 2R：dof = 2，組態 (θ₁, θ₂)',
    build: () => {
      const b = new MechanismBuilder();
      const g = b.ground([[0, 0.4]]);
      const l1 = b.body('連桿 1', [[0, 0.4], [1, 0.4]]);
      const l2 = b.body('連桿 2', [[1, 0.4], [2, 0.4]]);
      b.R(g, l1, [0, 0.4]).R(l1, l2, [1, 0.4]);
      return b.build();
    },
  },
];

export function presetByKey(key: string): PresetDef {
  const p = PRESETS.find((d) => d.key === key);
  if (!p) throw new Error(`unknown preset ${key}`);
  return p;
}

/** Joint angles (θ₁, θ₂) of the 2R preset, read back from the body orientations. */
export function arm2RAngles(m: Mechanism): [number, number] | null {
  if (m.bodies.length < 3) return null;
  const th1 = wrapToPi(m.bodies[1].pose.phi);
  const th2 = wrapToPi(m.bodies[2].pose.phi - m.bodies[1].pose.phi);
  return [th1, th2];
}
