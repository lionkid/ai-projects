/**
 * Time-optimal time scaling of a planar 2R arm along a fixed path (book §9.4).
 *
 * The path is θ(s), s ∈ [0, 1]. Substituting θ̇ = θ'ṡ and θ̈ = θ's̈ + θ''ṡ² into the arm
 * dynamics M(θ)θ̈ + c(θ, θ̇) + g(θ) = τ gives, joint by joint,
 *
 *     m_i(s) s̈ + c_i(s) ṡ² + g_i(s) = τ_i,
 *     m = M θ',   c = M θ'' + c(θ, θ'),   g = g(θ).
 *
 * Everything below works with x = ṡ² instead of ṡ. Along a path the torque limits are affine
 * in (s̈, x), the limit curve is the upper end of an x-interval that can be computed in closed
 * form, and the phase-plane ODE dx/ds = 2 s̈ stays regular at ṡ = 0 (no division by ṡ).
 *
 * Two solvers are provided:
 *  - `solveSwitching`: the forward/backward integration algorithm with a binary search for the
 *    tangent points on the velocity limit curve (the algorithm the lesson animates);
 *  - `solveGridDP`: a discretised dynamic-programming solver (backward reachable intervals +
 *    greedy forward pass) used as an independent check in the tests and as a fallback.
 *
 * Dynamics come from `planar2RDynamics` in `@/mr` (closed form, cross-checked against the
 * Newton–Euler `InverseDynamics` in the tests).
 */
import { planar2RDynamics, planar2RParams, planar2RIK, wrapToPi, type Planar2RParams } from '@/mr';

export type Vec2 = [number, number];
export type PathId = 'line' | 'corner' | 'cartesian';

export interface ArmPath {
  id: PathId;
  /** Joint angles θ(s). */
  theta(s: number): Vec2;
  /** dθ/ds. */
  d1(s: number): Vec2;
  /** d²θ/ds². */
  d2(s: number): Vec2;
}

const DEG = Math.PI / 180;

/** The lab's planar 2R model: L1 = L2 = 1 m, 1 kg point masses at the link tips, gravity −y. */
export const ARM: Planar2RParams = planar2RParams([1, 1], [1, 1], 'point-mass');

/** Joint-space straight line θ(s) = θ_A + s (θ_B − θ_A). */
function jointLine(id: PathId, a: Vec2, b: Vec2): ArmPath {
  const d: Vec2 = [b[0] - a[0], b[1] - a[1]];
  return {
    id,
    theta: (s) => [a[0] + s * d[0], a[1] + s * d[1]],
    d1: () => [d[0], d[1]],
    d2: () => [0, 0],
  };
}

/**
 * "Shoulder first, then elbow": the joint-space direction turns from +θ1 to +θ2 around s = c
 * within a width ~w (a rounded corner). θ'' is large only near the corner, so the velocity
 * product terms c(s) peak there. Both joint velocities stay ≥ 0, so m(s) never vanishes
 * (no zero-inertia points) for this arm.
 */
function cornerPath(a: Vec2, d: Vec2, w: number, c: number): ArmPath {
  const S = (s: number) => 0.5 * (1 + Math.tanh((s - c) / w));
  const Sp = (s: number) => 0.5 / (w * Math.cosh((s - c) / w) ** 2);
  const lc = (u: number) => { const v = Math.abs(u); return v + Math.log1p(Math.exp(-2 * v)) - Math.LN2; }; // log cosh
  const I = (s: number) => 0.5 * (s + w * lc((s - c) / w) - w * lc(-c / w)); // ∫₀ˢ S
  const I1 = I(1);
  return {
    id: 'corner',
    theta: (s) => [a[0] + (d[0] * (s - I(s))) / (1 - I1), a[1] + (d[1] * I(s)) / I1],
    d1: (s) => [(d[0] * (1 - S(s))) / (1 - I1), (d[1] * S(s)) / I1],
    d2: (s) => [(-d[0] * Sp(s)) / (1 - I1), (d[1] * Sp(s)) / I1],
  };
}

/** Straight line of the TIP in the plane, mapped to joint space with the analytic 2R IK (elbow θ2 > 0). */
function cartesianLine(a: Vec2, b: Vec2): ArmPath {
  const tip = (s: number): Vec2 => [a[0] + s * (b[0] - a[0]), a[1] + s * (b[1] - a[1])];
  const ik = (s: number): Vec2 => {
    const p = tip(s);
    const sol = planar2RIK(ARM.L1, ARM.L2, p[0], p[1]);
    const best = sol.solutions.find((q) => q.theta2 >= 0) ?? sol.solutions[0];
    if (!best) throw new Error('cartesianLine: target out of reach');
    return [best.theta1, best.theta2];
  };
  const ref = ik(0);
  // Keep θ1 continuous with the start (atan2 could wrap if the line crossed the −x axis).
  const theta = (s: number): Vec2 => {
    const q = ik(s);
    return [ref[0] + wrapToPi(q[0] - ref[0]), q[1]];
  };
  const h = 1e-4;
  return {
    id: 'cartesian',
    theta,
    d1: (s) => {
      const p = theta(s + h), m = theta(s - h);
      return [(p[0] - m[0]) / (2 * h), (p[1] - m[1]) / (2 * h)];
    },
    d2: (s) => {
      const p = theta(s + h), c = theta(s), m = theta(s - h);
      return [(p[0] - 2 * c[0] + m[0]) / (h * h), (p[1] - 2 * c[1] + m[1]) / (h * h)];
    },
  };
}

/** Start and end configuration shared by the 'line' and 'corner' paths. */
export const ENDPOINTS: [Vec2, Vec2] = [[-100 * DEG, 10 * DEG], [40 * DEG, 120 * DEG]];

export const PATHS: Record<PathId, ArmPath> = {
  line: jointLine('line', ENDPOINTS[0], ENDPOINTS[1]),
  corner: cornerPath(ENDPOINTS[0], [ENDPOINTS[1][0] - ENDPOINTS[0][0], ENDPOINTS[1][1] - ENDPOINTS[0][1]], 0.03, 0.45),
  cartesian: cartesianLine([1.3, -1.0], [1.3, 1.0]),
};

export const PATH_IDS: PathId[] = ['line', 'corner', 'cartesian'];

export interface PathCoefficients {
  m: Vec2;
  c: Vec2;
  g: Vec2;
  theta: Vec2;
  dtheta: Vec2;
}

/** m(s), c(s), g(s) of the scalar dynamics along the path (one entry per joint). */
export function pathCoefficients(path: ArmPath, s: number, P: Planar2RParams = ARM): PathCoefficients {
  const th = path.theta(s), d1 = path.d1(s), d2 = path.d2(s);
  // c(θ, θ̇) is quadratic in θ̇, so c(θ, θ'ṡ) = c(θ, θ') ṡ².
  const { M, c: cv, g } = planar2RDynamics(P, th, d1);
  return {
    m: [M[0][0] * d1[0] + M[0][1] * d1[1], M[1][0] * d1[0] + M[1][1] * d1[1]],
    c: [M[0][0] * d2[0] + M[0][1] * d2[1] + cv[0], M[1][0] * d2[0] + M[1][1] * d2[1] + cv[1]],
    g: [g[0], g[1]],
    theta: th,
    dtheta: d1,
  };
}

// ---------------------------------------------------------------------------------------------
// Tabulated path data
// ---------------------------------------------------------------------------------------------

export interface Limits {
  /** Symmetric torque limits |τ_i| ≤ tauMax[i] (N·m). */
  tauMax: Vec2;
  /** Optional joint speed limit |θ̇_i| ≤ dthetaMax (rad/s). */
  dthetaMax?: number | null;
}

/**
 * Path data on a uniform grid. Index j runs over HALF steps: s_j = j·ds/2, j = 0..2N, so that
 * RK4 midpoints of a step between grid nodes k and k+1 (j = 2k, 2k+1, 2k+2) need no interpolation.
 */
export interface PathTable {
  path: ArmPath;
  N: number;
  ds: number;
  tauMax: Vec2;
  dthetaMax: number | null;
  m: [Float64Array, Float64Array];
  c: [Float64Array, Float64Array];
  g: [Float64Array, Float64Array];
  d1: [Float64Array, Float64Array];
  /** Feasible x = ṡ² interval at every half step (torque limits plus optional speed limit). */
  xLo: Float64Array;
  xHi: Float64Array;
  /** Upper end from the torque limits alone (for drawing both curves when a speed limit is on). */
  xHiTorque: Float64Array;
}

const EPS_M = 1e-9;

export function buildTable(path: ArmPath, lim: Limits, N = 1000, P: Planar2RParams = ARM): PathTable {
  const J = 2 * N + 1;
  const mk = (): [Float64Array, Float64Array] => [new Float64Array(J), new Float64Array(J)];
  const tab: PathTable = {
    path, N, ds: 1 / N, tauMax: [lim.tauMax[0], lim.tauMax[1]], dthetaMax: lim.dthetaMax ?? null,
    m: mk(), c: mk(), g: mk(), d1: mk(),
    xLo: new Float64Array(J), xHi: new Float64Array(J), xHiTorque: new Float64Array(J),
  };
  for (let j = 0; j < J; j++) {
    const co = pathCoefficients(path, j / (2 * N), P);
    for (let i = 0; i < 2; i++) {
      tab.m[i][j] = co.m[i]; tab.c[i][j] = co.c[i]; tab.g[i][j] = co.g[i]; tab.d1[i][j] = co.dtheta[i];
    }
  }
  refreshLimits(tab, lim);
  return tab;
}

/** Recompute the feasible x-intervals for new limits without re-evaluating the dynamics. */
export function refreshLimits(tab: PathTable, lim: Limits): void {
  tab.tauMax = [lim.tauMax[0], lim.tauMax[1]];
  tab.dthetaMax = lim.dthetaMax ?? null;
  for (let j = 0; j < tab.xHi.length; j++) {
    const [lo, hi] = xRangeTorque(tab, j);
    tab.xHiTorque[j] = hi;
    let h = hi;
    if (tab.dthetaMax != null) h = Math.min(h, speedCap(tab, j));
    tab.xLo[j] = lo;
    tab.xHi[j] = h;
  }
}

/** Largest x = ṡ² allowed by |θ̇_i| = |θ_i'| ṡ ≤ θ̇max. */
export function speedCap(tab: PathTable, j: number): number {
  if (tab.dthetaMax == null) return Infinity;
  let cap = Infinity;
  for (let i = 0; i < 2; i++) {
    const d = Math.abs(tab.d1[i][j]);
    if (d > 1e-12) cap = Math.min(cap, (tab.dthetaMax / d) ** 2);
  }
  return cap;
}

/**
 * Feasible acceleration interval [L, U] at half-step j and x = ṡ² from |m_i s̈ + c_i x + g_i| ≤ τ_i.
 * A joint with m_i ≈ 0 (a zero-inertia point) does not bound s̈; its constraint only restricts x
 * and is handled in `xRangeTorque`.
 */
export function accelBounds(tab: PathTable, j: number, x: number): [number, number] {
  let L = -1e9, U = 1e9;
  for (let i = 0; i < 2; i++) {
    const mi = tab.m[i][j];
    if (Math.abs(mi) < EPS_M) continue;
    const rest = tab.c[i][j] * x + tab.g[i][j];
    const tau = tab.tauMax[i];
    let lo = (-tau - rest) / mi, hi = (tau - rest) / mi;
    if (mi < 0) { const t = lo; lo = hi; hi = t; }
    if (lo > L) L = lo;
    if (hi < U) U = hi;
  }
  return [L, U];
}

/**
 * Closed-form x-interval where L(x) ≤ U(x). Each joint's bounds on s̈ are affine in x, so every
 * pairwise condition L_i(x) ≤ U_k(x) is a half-line α + βx ≤ 0 and their intersection is one
 * interval [lo, hi] (hi = +∞ when nothing bounds it). lo > hi means "empty".
 */
export function xRangeTorque(tab: PathTable, j: number): [number, number] {
  let lo = 0, hi = Infinity;
  const los: [number, number][] = [];
  const his: [number, number][] = [];
  const cons: [number, number][] = [];
  for (let i = 0; i < 2; i++) {
    const mi = tab.m[i][j], ci = tab.c[i][j], gi = tab.g[i][j], tau = tab.tauMax[i];
    if (Math.abs(mi) < EPS_M) {
      cons.push([gi - tau, ci], [-gi - tau, -ci]);
      continue;
    }
    const aP = (tau - gi) / mi, aN = (-tau - gi) / mi, b = -ci / mi;
    if (mi > 0) { los.push([aN, b]); his.push([aP, b]); } else { los.push([aP, b]); his.push([aN, b]); }
  }
  for (let a = 0; a < los.length; a++) {
    for (let b = 0; b < his.length; b++) {
      if (a === b) continue; // a single joint always leaves a non-empty interval
      cons.push([los[a][0] - his[b][0], los[a][1] - his[b][1]]);
    }
  }
  for (const [al, be] of cons) {
    if (Math.abs(be) < 1e-14) { if (al > 1e-12) return [1, 0]; continue; }
    const r = -al / be;
    if (be > 0) hi = Math.min(hi, r); else lo = Math.max(lo, r);
  }
  return [lo, hi];
}

/** Velocity-limit curve value ṡ_lim at half-step j (0 when the state is not even statically feasible). */
export function sdotLimit(tab: PathTable, j: number, torqueOnly = false): number {
  const hi = torqueOnly ? tab.xHiTorque[j] : tab.xHi[j];
  if (!(tab.xLo[j] <= hi)) return 0;
  return Math.sqrt(Math.max(0, hi));
}

/** Half-step index of the grid value nearest to s. */
export function nearestHalf(tab: PathTable, s: number): number {
  return Math.max(0, Math.min(2 * tab.N, Math.round(s * 2 * tab.N)));
}

/** Linear interpolation of a half-step table at arbitrary s. */
export function interpTable(tab: PathTable, arr: Float64Array, s: number): number {
  const u = Math.max(0, Math.min(1, s)) * 2 * tab.N;
  const j = Math.min(2 * tab.N - 1, Math.floor(u));
  const f = u - j;
  return arr[j] * (1 - f) + arr[j + 1] * f;
}

/** [L, U] at arbitrary s (coefficients interpolated). */
export function boundsAt(tab: PathTable, s: number, x: number): [number, number] {
  const j = nearestHalf(tab, s);
  if (Math.abs(j / (2 * tab.N) - s) < 1e-12) return accelBounds(tab, j, x);
  // Interpolate the two neighbouring half steps.
  const u = Math.max(0, Math.min(1, s)) * 2 * tab.N;
  const j0 = Math.min(2 * tab.N - 1, Math.floor(u));
  const f = u - j0;
  const [L0, U0] = accelBounds(tab, j0, x), [L1, U1] = accelBounds(tab, j0 + 1, x);
  return [L0 * (1 - f) + L1 * f, U0 * (1 - f) + U1 * f];
}

/** Torques needed at (s, ṡ², s̈). */
export function torquesAt(tab: PathTable, s: number, x: number, sdd: number): Vec2 {
  const out: Vec2 = [0, 0];
  for (let i = 0; i < 2; i++) {
    out[i] = interpTable(tab, tab.m[i], s) * sdd + interpTable(tab, tab.c[i], s) * x + interpTable(tab, tab.g[i], s);
  }
  return out;
}

/** Can the arm stand still at half-step j, i.e. is s̈ = 0 inside [L, U] when ṡ = 0? */
export function holdable(tab: PathTable, j: number): boolean {
  if (tab.xLo[j] > 1e-12 || !(tab.xHi[j] >= 0)) return false;
  const [L, U] = accelBounds(tab, j, 0);
  return L <= 1e-12 && U >= -1e-12;
}

/**
 * s-intervals where the arm cannot stand still (the torque limits cannot even balance gravity).
 * Such a stretch is not automatically infeasible: the arm may still coast through it with speed.
 */
export function unholdableIntervals(tab: PathTable): [number, number][] {
  const out: [number, number][] = [];
  let start = -1;
  const J = 2 * tab.N;
  for (let j = 0; j <= J; j++) {
    const bad = !holdable(tab, j);
    if (bad && start < 0) start = j;
    if ((!bad || j === J) && start >= 0) { out.push([start / J, (bad ? j : j - 1) / J]); start = -1; }
  }
  return out;
}

// ---------------------------------------------------------------------------------------------
// Phase-plane integration in x = ṡ²:  dx/ds = 2 s̈,  s̈ = U (max acceleration) or L (max deceleration)
// ---------------------------------------------------------------------------------------------

export type Mode = 'U' | 'L';
export type StopReason = 'limit' | 'zero' | 'cross' | 'end';

export interface PhaseCurve {
  mode: Mode;
  /** +1: integrated forward in s; −1: backward. */
  dir: 1 | -1;
  /** Polyline in (s, x) ordered along the integration direction, ending at the stop point. */
  pts: [number, number][];
  /** x at grid nodes covered by the curve (NaN elsewhere). */
  nodeX: Float64Array;
  reason: StopReason;
  /** Where the integration stopped (interpolated). */
  stop: [number, number];
  /** Last grid node reached before the stop event. */
  lastNode: number;
}

function slope(tab: PathTable, j: number, x: number, mode: Mode): number {
  const [L, U] = accelBounds(tab, j, x);
  return 2 * (mode === 'U' ? U : L);
}

/** Small relative tolerance for "above the limit curve". */
function aboveLimit(tab: PathTable, j: number, x: number): boolean {
  const hi = tab.xHi[j];
  return x > hi * (1 + 1e-9) + 1e-10 || x < tab.xLo[j] - 1e-9;
}

export interface IntegrateOpts {
  /** Stop when crossing this node-indexed curve (NaN where undefined). */
  target?: Float64Array;
  /** Allow penetration of the limit curve within this many nodes of the start (start on the curve). */
  graceNodes?: number;
}

/**
 * Integrate from grid node k0 with initial x0, in direction dir, with s̈ = U or L (RK4 on the
 * half-step table). Stops on: penetrating the limit curve, x reaching 0, crossing `target`,
 * or reaching the end of [0, 1].
 */
export function integrate(tab: PathTable, k0: number, x0: number, dir: 1 | -1, mode: Mode, opts: IntegrateOpts = {}): PhaseCurve {
  const { N, ds } = tab;
  const nodeX = new Float64Array(N + 1).fill(NaN);
  const pts: [number, number][] = [[k0 * ds, x0]];
  nodeX[k0] = x0;
  let k = k0, x = x0;
  const h = dir * ds;
  const target = opts.target;
  const grace = opts.graceNodes ?? 0;
  const finish = (reason: StopReason, stop: [number, number]): PhaseCurve => {
    if (pts[pts.length - 1][0] !== stop[0] || pts[pts.length - 1][1] !== stop[1]) pts.push(stop);
    return { mode, dir, pts, nodeX, reason, stop, lastNode: k };
  };
  if (target && Number.isFinite(target[k0]) && Math.abs(x0 - target[k0]) <= 1e-12 * Math.max(1, x0)) {
    return finish('cross', [k0 * ds, x0]);
  }
  while (dir > 0 ? k < N : k > 0) {
    const j0 = 2 * k, jm = 2 * k + dir, j1 = 2 * k + 2 * dir;
    const k1 = slope(tab, j0, x, mode);
    const k2 = slope(tab, jm, x + 0.5 * h * k1, mode);
    const k3 = slope(tab, jm, x + 0.5 * h * k2, mode);
    const k4 = slope(tab, j1, x + h * k3, mode);
    const xn = x + (h / 6) * (k1 + 2 * k2 + 2 * k3 + k4);
    const kn = k + dir;
    const sOld = k * ds, sNew = kn * ds;
    if (!Number.isFinite(xn)) return finish('limit', [sOld, x]);
    // Crossing the target curve.
    if (target && Number.isFinite(target[k]) && Number.isFinite(target[kn])) {
      const d0 = x - target[k], d1 = xn - target[kn];
      if (d0 === 0 || d0 * d1 < 0 || d1 === 0) {
        const f = d0 === d1 ? 0 : d0 / (d0 - d1);
        const sc = sOld + f * (sNew - sOld), xc = x + f * (xn - x);
        if (f >= 1) { k = kn; x = xn; nodeX[kn] = xn; }
        return finish('cross', [sc, xc]);
      }
    }
    // Reaching zero speed.
    if (xn <= 0) {
      const f = x / (x - xn);
      return finish('zero', [sOld + f * (sNew - sOld), 0]);
    }
    // Penetrating the velocity limit curve.
    if (Math.abs(kn - k0) > grace && aboveLimit(tab, 2 * kn, xn)) {
      const d0 = x - tab.xHi[2 * k], d1 = xn - tab.xHi[2 * kn];
      const f = d0 < 0 && d1 > d0 ? Math.min(1, Math.max(0, -d0 / (d1 - d0))) : 0;
      return finish('limit', [sOld + f * (sNew - sOld), x + f * (xn - x)]);
    }
    k = kn; x = xn;
    nodeX[k] = x;
    pts.push([sNew, x]);
  }
  return finish('end', [k * ds, x]);
}

// ---------------------------------------------------------------------------------------------
// The switching algorithm
// ---------------------------------------------------------------------------------------------

export type StageKind = 'backward' | 'forward' | 'probe' | 'tangent' | 'switch' | 'done' | 'fail';

export interface Stage {
  kind: StageKind;
  /** Curve to draw for this stage (if any). */
  curve?: PhaseCurve;
  /** Or a plain polyline in (s, x). */
  poly?: [number, number][];
  /** A marked point (switch, tangent, penetration). */
  point?: [number, number];
  /** Switch number for 'switch' stages (1-based). */
  index?: number;
  /** Short zh-TW description for the UI. */
  text: string;
}

export interface Switch { s: number; x: number; type: 'acc→dec' | 'dec→acc' }

export interface Solution {
  ok: boolean;
  /** x = ṡ² at every grid node (N + 1 values). */
  x: Float64Array;
  /** Polyline of the optimal curve in (s, x) including the exact switch points. */
  pts: [number, number][];
  switches: Switch[];
  T: number;
  stages: Stage[];
  /** Which solver produced `x`. */
  method: 'switching' | 'grid';
  message?: string;
  /** True when the failure is a property of the problem (not a numerical hiccup). */
  infeasible?: boolean;
  /** Grid solver only: largest x at every node from which the end can still be reached at rest. */
  envelope?: Float64Array;
}

const fmt2 = (v: number) => v.toFixed(2);

/**
 * Time-optimal scaling by forward/backward integration (bang-bang with switches):
 *  1. integrate L backward from (1, 0) → final curve F;
 *  2. integrate U forward from the current start until it crosses F (last switch, done) or
 *     penetrates the limit curve at s_lim;
 *  3. binary-search the speed at s_lim for the L-curve that just touches the limit curve
 *     (tangent point), integrate that L-curve backward to meet the U-curve (acc→dec switch),
 *     restart from the tangent point (dec→acc switch) and go to 2.
 */
export function solveSwitching(tab: PathTable, maxRounds = 12): Solution {
  const { N, ds } = tab;
  const stages: Stage[] = [];
  const fail = (msg: string, infeasible = false): Solution => {
    stages.push({ kind: 'fail', text: msg });
    return { ok: false, infeasible, x: new Float64Array(N + 1), pts: [], switches: [], T: Infinity, stages, method: 'switching', message: msg };
  };
  if (!holdable(tab, 0)) return fail('起點無法靜止：力矩上限撐不住起點的重力，無法從靜止出發', true);
  if (!holdable(tab, 2 * N)) return fail('終點無法靜止：力矩上限撐不住終點的重力，無法停在終點', true);

  const F = integrate(tab, N, 0, -1, 'L');
  stages.push({
    kind: 'backward', curve: F,
    text: `從終點 (1, 0) 以最大減速 L 往回積分${F.reason === 'limit' ? `，在 s ≈ ${fmt2(F.stop[0])} 碰到極限曲線` : F.reason === 'zero' ? `，在 s ≈ ${fmt2(F.stop[0])} 速度降到 0` : ''}`,
  });
  if (F.reason === 'zero') return fail(`在 s ≈ ${fmt2(F.stop[0])} 之後減速能力不足（重力一直往前推），以最大減速往回積分時速度歸零`);

  const pieces: { curve: PhaseCurve; from: number; to: number }[] = [];
  const switches: Switch[] = [];
  let k0 = 0, x0 = 0, startOnLimit = false;
  for (let round = 0; round < maxRounds; round++) {
    const A = integrate(tab, k0, x0, 1, 'U', { target: F.nodeX, graceNodes: startOnLimit ? 2 : 0 });
    const where = fmt2(A.stop[0]);
    stages.push({
      kind: 'forward', curve: A,
      text: `從 s = ${fmt2(k0 * ds)} 以最大加速 U 往前積分${A.reason === 'limit' ? `，在 s ≈ ${where} 撞上極限曲線` : A.reason === 'cross' ? `，在 s ≈ ${where} 與減速曲線交會` : A.reason === 'zero' ? `，在 s ≈ ${where} 速度降到 0` : ''}`,
    });
    if (A.reason === 'cross') {
      switches.push({ s: A.stop[0], x: A.stop[1], type: 'acc→dec' });
      stages.push({ kind: 'switch', point: A.stop, index: switches.length, text: `切換 ${switches.length}：s = ${where}，加速 → 減速（最後一次）` });
      pieces.push({ curve: A, from: k0 * ds, to: A.stop[0] }, { curve: F, from: A.stop[0], to: 1 });
      return assemble(tab, pieces, switches, stages);
    }
    if (A.reason === 'zero') {
      return fail(`即使全力加速，手臂在 s ≈ ${where} 仍停了下來：力矩不足以把它推過這一段，路徑不可行`, true);
    }
    if (A.reason !== 'limit') return fail('前向積分到了終點卻沒有與減速曲線交會（數值問題）');

    // Search at the last grid node of A that is still below the limit curve.
    const kl = Math.max(k0, A.lastNode);
    const xA = A.nodeX[kl];
    stages.push({ kind: 'probe', point: [kl * ds, xA], text: `在 s_lim ≈ ${fmt2(kl * ds)} 用二分法找「剛好擦過」極限曲線的減速曲線` });
    let lo = 0, hi = Number.isFinite(xA) ? xA : tab.xHi[2 * kl];
    let shown = 0;
    for (let it = 0; it < 60 && hi - lo > 1e-13 * Math.max(1, hi); it++) {
      const mid = 0.5 * (lo + hi);
      const C = integrate(tab, kl, mid, 1, 'L');
      const tooHigh = C.reason === 'limit' || C.reason === 'end';
      if (shown < 6) {
        stages.push({ kind: 'probe', curve: C, text: `試 ṡ = ${fmt2(Math.sqrt(mid))}：${tooHigh ? '穿過極限曲線 → 太快' : '速度降到 0 → 太慢'}` });
        shown++;
      }
      if (tooHigh) hi = mid; else lo = mid;
    }
    // The touching deceleration curve and its tangent point (closest approach to the limit curve).
    const C = integrate(tab, kl, lo, 1, 'L');
    let kt = kl, best = Infinity;
    for (let k = kl; k <= N; k++) {
      const v = C.nodeX[k], h = tab.xHi[2 * k];
      if (!Number.isFinite(v) || !Number.isFinite(h)) continue;
      const gap = (h - v) / Math.max(1e-9, h);
      if (gap < best) { best = gap; kt = k; }
    }
    const xt = C.nodeX[kt];
    if (!(kt > k0) || !Number.isFinite(xt)) return fail('找不到切點（數值問題）');
    stages.push({ kind: 'tangent', curve: C, point: [kt * ds, xt], text: `切點：s ≈ ${fmt2(kt * ds)}，ṡ ≈ ${fmt2(Math.sqrt(xt))}` });

    // Follow that deceleration curve backward from s_lim until it meets the acceleration curve A.
    let sw: [number, number];
    if (!(xA - lo > 1e-10 * Math.max(1, xA))) {
      sw = [kl * ds, xA];
      pieces.push({ curve: A, from: k0 * ds, to: kl * ds });
    } else {
      const B = integrate(tab, kl, lo, -1, 'L', { target: A.nodeX });
      if (B.reason !== 'cross') return fail('減速曲線往回積分沒有與加速曲線交會（數值問題）');
      sw = B.stop;
      pieces.push({ curve: A, from: k0 * ds, to: sw[0] }, { curve: B, from: sw[0], to: kl * ds });
    }
    pieces.push({ curve: C, from: kl * ds, to: kt * ds });
    switches.push({ s: sw[0], x: sw[1], type: 'acc→dec' });
    stages.push({ kind: 'switch', point: sw, index: switches.length, text: `切換 ${switches.length}：s = ${fmt2(sw[0])}，加速 → 減速` });
    switches.push({ s: kt * ds, x: xt, type: 'dec→acc' });
    stages.push({ kind: 'switch', point: [kt * ds, xt], index: switches.length, text: `切換 ${switches.length}：在切點 s = ${fmt2(kt * ds)}，減速 → 加速` });
    k0 = kt; x0 = xt; startOnLimit = true;
  }
  return fail('切換次數超過上限');
}

/** Build the node values and polyline of the solution from consecutive curve pieces. */
function assemble(tab: PathTable, pieces: { curve: PhaseCurve; from: number; to: number }[], switches: Switch[], stages: Stage[]): Solution {
  const { N, ds } = tab;
  const x = new Float64Array(N + 1).fill(NaN);
  const pts: [number, number][] = [];
  for (const p of pieces) {
    const a = Math.min(p.from, p.to), b = Math.max(p.from, p.to);
    for (let k = Math.ceil(a / ds - 1e-9); k <= Math.floor(b / ds + 1e-9); k++) {
      const v = p.curve.nodeX[k];
      if (Number.isFinite(v)) x[k] = v;
    }
    const seg = p.curve.pts.filter(([s]) => s >= a - 1e-12 && s <= b + 1e-12).sort((u, v) => u[0] - v[0]);
    for (const q of seg) if (!pts.length || q[0] > pts[pts.length - 1][0] + 1e-12) pts.push(q);
  }
  x[0] = 0; x[N] = 0;
  // Fill any node left uncovered by rounding at piece borders.
  for (let k = 1; k < N; k++) if (!Number.isFinite(x[k])) x[k] = interpPolyline(pts, k * ds);
  for (let k = 0; k <= N; k++) x[k] = Math.max(0, x[k]);
  if (!pts.length || pts[0][0] > 0) pts.unshift([0, 0]);
  if (pts[pts.length - 1][0] < 1) pts.push([1, 0]);
  const T = curveTime(x, ds);
  stages.push({ kind: 'done', text: `完成：${switches.length} 個切換點，總時間 T* = ${T.toFixed(3)} s` });
  return { ok: true, x, pts, switches, T, stages, method: 'switching' };
}

function interpPolyline(pts: [number, number][], s: number): number {
  if (!pts.length) return 0;
  if (s <= pts[0][0]) return pts[0][1];
  for (let i = 1; i < pts.length; i++) {
    if (s <= pts[i][0]) {
      const [s0, x0] = pts[i - 1], [s1, x1] = pts[i];
      return s1 > s0 ? x0 + ((s - s0) / (s1 - s0)) * (x1 - x0) : x1;
    }
  }
  return pts[pts.length - 1][1];
}

// ---------------------------------------------------------------------------------------------
// Grid dynamic programming (independent check / fallback)
// ---------------------------------------------------------------------------------------------

/** max or min of x over {(x, u): a·u + b·x ≤ c for all rows} (2-variable LP by vertex enumeration). */
function lpExtremeX(rows: [number, number, number][], sense: 1 | -1): number | null {
  let best: number | null = null;
  const n = rows.length;
  for (let p = 0; p < n; p++) {
    for (let q = p + 1; q < n; q++) {
      const [a1, b1, c1] = rows[p], [a2, b2, c2] = rows[q];
      const det = a1 * b2 - a2 * b1;
      if (Math.abs(det) < 1e-14) continue;
      const u = (c1 * b2 - c2 * b1) / det;
      const x = (a1 * c2 - a2 * c1) / det;
      let ok = true;
      for (let r = 0; r < n && ok; r++) {
        const [a, b, c] = rows[r];
        if (a * u + b * x > c + 1e-9 * (1 + Math.abs(c))) ok = false;
      }
      if (ok && (best === null || sense * x > sense * best)) best = x;
    }
  }
  return best;
}

/**
 * First-order discretisation: x_{k+1} = x_k + 2 ds u_k with the torque limits imposed at node k.
 * Backward pass: interval of x_k from which the end (x_N = 0) is still reachable.
 * Forward pass: from x_0 = 0 always take the largest admissible x_{k+1}.
 */
export function solveGridDP(tab: PathTable): Solution {
  const { N, ds } = tab;
  const lo = new Float64Array(N + 1), hi = new Float64Array(N + 1);
  lo[N] = 0; hi[N] = 0;
  const BIG = 1e7;
  const fail = (message: string): Solution =>
    ({ ok: false, infeasible: true, x: new Float64Array(N + 1), pts: [], switches: [], T: Infinity, stages: [], method: 'grid', message });
  for (let k = N - 1; k >= 0; k--) {
    const j = 2 * k;
    const rows: [number, number, number][] = [];
    for (let i = 0; i < 2; i++) {
      const m = tab.m[i][j], c = tab.c[i][j], g = tab.g[i][j], t = tab.tauMax[i];
      rows.push([m, c, t - g], [-m, -c, t + g]);
    }
    rows.push([2 * ds, 1, hi[k + 1]], [-2 * ds, -1, -lo[k + 1]]);
    rows.push([0, -1, 0], [0, 1, Math.min(BIG, speedCap(tab, j))], [1, 0, BIG], [-1, 0, BIG]);
    const xmax = lpExtremeX(rows, 1), xmin = lpExtremeX(rows, -1);
    if (xmax === null || xmin === null) return fail(`從 s ≈ ${fmt2(k * ds)} 起無論如何都無法停在終點`);
    lo[k] = Math.max(0, xmin); hi[k] = Math.max(0, xmax);
  }
  if (lo[0] > 1e-12) return fail('從靜止出發無法走完這條路徑（力矩不足）');
  const x = new Float64Array(N + 1);
  x[0] = 0;
  for (let k = 0; k < N; k++) {
    const [, U] = accelBounds(tab, 2 * k, x[k]);
    const next = x[k] + 2 * ds * U;
    if (next < lo[k + 1] - 1e-9 * (1 + lo[k + 1])) return fail(`即使全力加速，在 s ≈ ${fmt2(k * ds)} 也跟不上`);
    x[k + 1] = Math.max(lo[k + 1], Math.min(hi[k + 1], next));
  }
  const pts: [number, number][] = Array.from(x, (v, k) => [k * ds, v] as [number, number]);
  const switches = detectSwitches(tab, x);
  return { ok: true, x, pts, switches, T: curveTime(x, ds), stages: [], method: 'grid', envelope: hi };
}

/**
 * The solver the lesson uses: the switching algorithm for pure torque limits (with the grid
 * solver as a fallback if it ever fails numerically) and the grid solver when a joint speed
 * limit is active (the optimum may then ride along the speed ceiling, which is not bang-bang).
 */
export function solveTimeOptimal(tab: PathTable): Solution {
  const { ds } = tab;
  if (tab.dthetaMax != null) {
    const g = solveGridDP(tab);
    if (!g.ok) { g.stages = [{ kind: 'fail', text: g.message ?? '不可行' }]; return g; }
    const env = g.envelope!;
    g.stages = [
      { kind: 'backward', poly: Array.from(env, (v, k) => [k * ds, v] as [number, number]), text: '從終點往回：每個 s 最快能多快、還來得及停下（已含速度上限）' },
      { kind: 'forward', poly: g.pts, text: '從起點全力加速，但不超過這條包絡線；碰到速度上限就沿著它走' },
      { kind: 'done', text: `完成（數值動態規劃）：總時間 T* = ${g.T.toFixed(3)} s` },
    ];
    return g;
  }
  const sw = solveSwitching(tab);
  if (sw.ok || sw.infeasible) return sw;
  const g = solveGridDP(tab);
  if (!g.ok) return { ...sw, infeasible: true };
  g.message = `切換演算法在這組設定下沒有收斂（${sw.message}），改用數值動態規劃的結果`;
  g.stages = [...sw.stages.filter((st) => st.kind !== 'fail'), { kind: 'forward', poly: g.pts, text: g.message }, { kind: 'done', text: `完成：總時間 T* = ${g.T.toFixed(3)} s` }];
  return g;
}

/** Switches of a node-sampled curve: where the cell acceleration flips between ≈U and ≈L. */
export function detectSwitches(tab: PathTable, x: Float64Array): Switch[] {
  const { N, ds } = tab;
  const out: Switch[] = [];
  let prev: 'U' | 'L' | null = null;
  for (let k = 0; k < N; k++) {
    const a = (x[k + 1] - x[k]) / (2 * ds);
    const [L, U] = accelBounds(tab, 2 * k + 1, 0.5 * (x[k] + x[k + 1]));
    const w = Math.max(1e-9, U - L);
    const cur: 'U' | 'L' | null = Math.abs(a - U) < 0.08 * w ? 'U' : Math.abs(a - L) < 0.08 * w ? 'L' : null;
    if (cur && prev && cur !== prev) out.push({ s: k * ds, x: x[k], type: prev === 'U' ? 'acc→dec' : 'dec→acc' });
    if (cur) prev = cur;
  }
  return out;
}

// ---------------------------------------------------------------------------------------------
// Curves as trajectories
// ---------------------------------------------------------------------------------------------

/**
 * Traversal time of a node-sampled curve x(s) with x linear between nodes (constant s̈ per
 * cell): each cell takes 2 ds / (√x_k + √x_{k+1}). Infinite when the curve stops in between.
 */
export function curveTime(x: ArrayLike<number>, ds: number): number {
  let T = 0;
  for (let k = 0; k + 1 < x.length; k++) {
    const d = Math.sqrt(Math.max(0, x[k])) + Math.sqrt(Math.max(0, x[k + 1]));
    if (d <= 1e-12) return Infinity;
    T += (2 * ds) / d;
  }
  return T;
}

export interface Timing {
  /** Cumulative time at every node. */
  t: Float64Array;
  T: number;
  /** State at time t: s, ṡ, s̈ (s̈ constant in each cell). */
  at(time: number): { s: number; sd: number; sdd: number };
}

export function timing(x: ArrayLike<number>, ds: number): Timing {
  const n = x.length;
  const t = new Float64Array(n);
  for (let k = 0; k + 1 < n; k++) {
    const d = Math.sqrt(Math.max(0, x[k])) + Math.sqrt(Math.max(0, x[k + 1]));
    t[k + 1] = t[k] + (d > 1e-12 ? (2 * ds) / d : Infinity);
  }
  const T = t[n - 1];
  const at = (time: number) => {
    const tc = Math.max(0, Math.min(T, time));
    let a = 0, b = n - 1;
    while (b - a > 1) { const m = (a + b) >> 1; if (t[m] <= tc) a = m; else b = m; }
    const k = a;
    const v0 = Math.sqrt(Math.max(0, x[k]));
    const acc = (x[k + 1] - x[k]) / (2 * ds);
    const tau = tc - t[k];
    return { s: Math.min(1, k * ds + v0 * tau + 0.5 * acc * tau * tau), sd: Math.max(0, v0 + acc * tau), sdd: acc };
  };
  return { t, T, at };
}

export interface CurveCheck {
  /** Per cell: true when the cell satisfies the limits. */
  ok: boolean[];
  /** Number of violating cells. */
  bad: number;
  T: number;
  /** Largest violation of the acceleration interval (1/s²) or of the limit curve (in ṡ). */
  worst: number;
}

/** Check a node-sampled curve against the limits (cell acceleration within [L, U], x ≤ x_hi). */
export function checkCurve(tab: PathTable, x: ArrayLike<number>, tolAcc = 0.02): CurveCheck {
  const { N, ds } = tab;
  const ok: boolean[] = [];
  let bad = 0, worst = 0;
  for (let k = 0; k < N; k++) {
    const a = (x[k + 1] - x[k]) / (2 * ds);
    const xm = 0.5 * (x[k] + x[k + 1]);
    const j = 2 * k + 1;
    const [L, U] = accelBounds(tab, j, xm);
    const tol = tolAcc * Math.max(1, Math.abs(U - L));
    const overA = Math.max(0, a - U, L - a);
    const overX = Math.max(0, Math.sqrt(Math.max(0, xm)) - Math.sqrt(Math.max(0, tab.xHi[j])) - 1e-6);
    const good = overA <= tol && overX <= 1e-3 && L <= U + tol;
    ok.push(good);
    if (!good) { bad++; worst = Math.max(worst, overA, overX); }
  }
  return { ok, bad, T: curveTime(x, ds), worst };
}

/**
 * Turn a freehand polyline of (s, ṡ) points into node values x = ṡ² over the whole grid. The
 * motion always starts and ends at rest; before the first and after the last drawn point the
 * curve uses constant acceleration / deceleration (x linear in s).
 */
export function curveFromDrawing(N: number, drawn: [number, number][]): Float64Array {
  const x = new Float64Array(N + 1).fill(NaN);
  const ds = 1 / N;
  const put = (s: number, sd: number) => {
    const k = Math.round(s / ds);
    if (k >= 0 && k <= N) x[k] = Math.max(0, sd) ** 2;
  };
  for (let i = 0; i < drawn.length; i++) {
    const [s1, v1] = drawn[i];
    if (i === 0) { put(s1, v1); continue; }
    const [s0, v0] = drawn[i - 1];
    const k0 = Math.round(s0 / ds), k1 = Math.round(s1 / ds);
    if (k0 === k1) { put(s1, v1); continue; }
    const step = k1 > k0 ? 1 : -1;
    for (let k = k0; k !== k1 + step; k += step) {
      if (k < 0 || k > N) continue;
      const f = (k - k0) / (k1 - k0);
      x[k] = Math.max(0, v0 + f * (v1 - v0)) ** 2;
    }
  }
  const painted: number[] = [];
  for (let k = 0; k <= N; k++) if (Number.isFinite(x[k])) painted.push(k);
  if (!painted.length) return new Float64Array(N + 1);
  const first = painted[0], last = painted[painted.length - 1];
  for (let p = 0; p + 1 < painted.length; p++) {
    const a = painted[p], b = painted[p + 1];
    for (let k = a + 1; k < b; k++) {
      const va = Math.sqrt(x[a]), vb = Math.sqrt(x[b]);
      x[k] = (va + ((k - a) / (b - a)) * (vb - va)) ** 2;
    }
  }
  const xf = first === 0 ? 0 : x[first];
  for (let k = 0; k < first; k++) x[k] = xf * (k / first);
  const xl = last === N ? 0 : x[last];
  for (let k = last + 1; k <= N; k++) x[k] = xl * ((N - k) / (N - last));
  x[0] = 0; x[N] = 0;
  return x;
}

// ---------------------------------------------------------------------------------------------
// Per-joint view of the acceleration interval (the lesson's interval diagram)
// ---------------------------------------------------------------------------------------------

/** m, c, g at arbitrary s (linear interpolation between half steps). */
export function coeffsAt(tab: PathTable, s: number): { m: Vec2; c: Vec2; g: Vec2 } {
  const pick = (a: [Float64Array, Float64Array]): Vec2 => [interpTable(tab, a[0], s), interpTable(tab, a[1], s)];
  return { m: pick(tab.m), c: pick(tab.c), g: pick(tab.g) };
}

export interface JointIntervals {
  /** s̈ range allowed by joint i alone: lo[i] ≤ s̈ ≤ hi[i] (±Infinity when a zero-inertia joint does not bound s̈). */
  lo: Vec2;
  hi: Vec2;
  /** Intersection [L, U]; L > U means that no acceleration satisfies every joint. */
  L: number;
  U: number;
}

/**
 * Each joint's own s̈ interval from |m_i s̈ + c_i ṡ² + g_i| ≤ τ_i,max at (s, x = ṡ²), and their
 * intersection. A joint with m_i ≈ 0 does not bound s̈: it allows everything when its torque
 * c_i x + g_i is within the limit and nothing otherwise.
 */
export function jointIntervals(tab: PathTable, s: number, x: number): JointIntervals {
  const co = coeffsAt(tab, s);
  const lo: Vec2 = [-Infinity, -Infinity];
  const hi: Vec2 = [Infinity, Infinity];
  for (let i = 0; i < 2; i++) {
    const m = co.m[i], rest = co.c[i] * x + co.g[i], tau = tab.tauMax[i];
    if (Math.abs(m) < EPS_M) {
      if (Math.abs(rest) > tau) { lo[i] = Infinity; hi[i] = -Infinity; }
      continue;
    }
    const a = (-tau - rest) / m, b = (tau - rest) / m;
    lo[i] = Math.min(a, b);
    hi[i] = Math.max(a, b);
  }
  return { lo, hi, L: Math.max(lo[0], lo[1]), U: Math.min(hi[0], hi[1]) };
}

// ---------------------------------------------------------------------------------------------
// Reference: the cubic time scaling of the previous lessons
// ---------------------------------------------------------------------------------------------

/** x = ṡ² at the grid nodes for s(t) = 3(t/T)² − 2(t/T)³ (rest to rest in time T). */
export function cubicScalingCurve(N: number, T: number): Float64Array {
  const x = new Float64Array(N + 1);
  for (let k = 1; k < N; k++) {
    const s = k / N;
    // Invert s = 3u² − 2u³ (monotone on [0, 1]) by bisection.
    let lo = 0, hi = 1;
    for (let it = 0; it < 52; it++) {
      const u = 0.5 * (lo + hi);
      if (u * u * (3 - 2 * u) < s) lo = u; else hi = u;
    }
    const u = 0.5 * (lo + hi);
    const sd = (6 * u * (1 - u)) / T;
    x[k] = sd * sd;
  }
  return x;
}

/**
 * The shortest duration T for which the cubic time scaling respects the limits. Scaling time
 * by T multiplies both s̈ and ṡ² by λ = 1/T², so every torque constraint, written per cell like
 * `checkCurve`, is linear in λ: −τ_i ≤ λ h_i + g_i ≤ τ_i with h_i = m_i s̈₁ + c_i ṡ₁² (values for
 * T = 1). The feasible λ form one interval and T = 1/√λ_max. Returns null if no T works (e.g.
 * the arm cannot even hold some configuration of the path against gravity).
 */
export function fastestCubic(tab: PathTable): { T: number; x: Float64Array } | null {
  const { N, ds } = tab;
  const x1 = cubicScalingCurve(N, 1);
  let lamLo = 0, lamHi = Infinity;
  for (let k = 0; k < N; k++) {
    const j = 2 * k + 1;
    const a = (x1[k + 1] - x1[k]) / (2 * ds), xm = 0.5 * (x1[k] + x1[k + 1]);
    for (let i = 0; i < 2; i++) {
      const h = tab.m[i][j] * a + tab.c[i][j] * xm, g = tab.g[i][j], tau = tab.tauMax[i];
      if (Math.abs(h) < 1e-12) { if (Math.abs(g) > tau) return null; continue; }
      const r1 = (tau - g) / h, r2 = (-tau - g) / h;
      if (h > 0) { lamHi = Math.min(lamHi, r1); lamLo = Math.max(lamLo, r2); } else { lamHi = Math.min(lamHi, r2); lamLo = Math.max(lamLo, r1); }
    }
    // The (optional) joint speed limit caps ṡ² directly.
    if (xm > 0 && Number.isFinite(tab.xHi[j])) lamHi = Math.min(lamHi, tab.xHi[j] / xm);
  }
  if (!(lamHi > 0) || !Number.isFinite(lamHi) || lamLo > lamHi) return null;
  const T = 1 / Math.sqrt(lamHi);
  return { T, x: Float64Array.from(x1, (v) => v / (T * T)) };
}

// ---------------------------------------------------------------------------------------------
// Curves as time histories (for the τ(t) plot and the playback)
// ---------------------------------------------------------------------------------------------

export interface TrajectorySamples {
  t: number[];
  s: number[];
  sd: number[];
  sdd: number[];
  tau: [number[], number[]];
}

/**
 * Time history of a node-sampled curve x(s): the start, the midpoint of every cell (where the
 * cell's constant s̈ is a second-order accurate value, as in `checkCurve`) and the end. Null when
 * the curve stops before the end of the path (infinite duration).
 */
export function trajectorySamples(tab: PathTable, x: ArrayLike<number>): TrajectorySamples | null {
  const { N, ds } = tab;
  const tm = timing(x, ds);
  if (!Number.isFinite(tm.T)) return null;
  const out: TrajectorySamples = { t: [], s: [], sd: [], sdd: [], tau: [[], []] };
  const push = (t: number, s: number, xv: number, sdd: number) => {
    const xc = Math.max(0, xv);
    out.t.push(t); out.s.push(s); out.sd.push(Math.sqrt(xc)); out.sdd.push(sdd);
    const tau = torquesAt(tab, s, xc, sdd);
    out.tau[0].push(tau[0]); out.tau[1].push(tau[1]);
  };
  const acc = (k: number) => (x[k + 1] - x[k]) / (2 * ds);
  push(0, 0, x[0], acc(0));
  for (let k = 0; k < N; k++) {
    // Time to reach the cell midpoint with constant acceleration from (s_k, ṡ_k).
    const v0 = Math.sqrt(Math.max(0, x[k])), xm = 0.5 * (x[k] + x[k + 1]);
    const tMid = tm.t[k] + (2 * (0.5 * ds)) / (v0 + Math.sqrt(Math.max(0, xm)));
    push(tMid, (k + 0.5) * ds, xm, acc(k));
  }
  push(tm.T, 1, x[N], acc(N - 1));
  return out;
}
