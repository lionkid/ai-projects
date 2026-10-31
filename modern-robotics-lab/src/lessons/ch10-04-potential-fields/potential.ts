/**
 * Pure helpers for the potential-field lesson (no DOM; unit-tested in potential.test.ts).
 *
 * - Obstacles are disks and convex polygons. The distance d(q) to a convex obstacle is smooth
 *   outside it, so the summed repulsive potential is smooth in free space (a concave shape such
 *   as a U is built from several convex walls).
 * - U(q) = ½ k_att ‖q − q_goal‖² + Σ_i ½ k (1/d_i − 1/d0)²   (each term only where d_i < d0).
 * - Two robot models: a first-order controller that reads −∇U every Δt and holds that velocity
 *   (zero-order hold, speed limit v_max), and a damped point mass q̈ = sat(−∇U) − b q̇ integrated
 *   with fixed semi-implicit Euler sub-steps.
 * - Local-minimum detection / refinement, basin-of-attraction classification on a grid,
 *   marching-squares contours, the sphere-world navigation function of Koditschek & Rimon, and a
 *   small RRT whose samples can be biased toward low potential.
 *
 * Units: metres and seconds; the robot is a point (it is only drawn with a radius).
 */
import type { P2 } from '@/mr';

// ---------------------------------------------------------------------------------------------
// Obstacles and clearance
// ---------------------------------------------------------------------------------------------

export type Obstacle = { kind: 'disk'; c: P2; r: number } | { kind: 'poly'; pts: P2[] };

/** A draggable group of convex parts (e.g. the three walls of a U). */
export interface ObstacleGroup { name: string; parts: Obstacle[] }

export interface Bounds { xmin: number; xmax: number; ymin: number; ymax: number }

export const diskObs = (x: number, y: number, r: number): Obstacle => ({ kind: 'disk', c: [x, y], r });
/** Axis-aligned box as a counter-clockwise convex polygon. */
export const boxObs = (x0: number, y0: number, x1: number, y1: number): Obstacle => ({
  kind: 'poly', pts: [[x0, y0], [x1, y0], [x1, y1], [x0, y1]],
});

export function translateObstacle(o: Obstacle, dx: number, dy: number): Obstacle {
  return o.kind === 'disk'
    ? { kind: 'disk', c: [o.c[0] + dx, o.c[1] + dy], r: o.r }
    : { kind: 'poly', pts: o.pts.map((p) => [p[0] + dx, p[1] + dy] as P2) };
}

/** Centre of the bounding box of a group (used as its drag handle). */
export function groupCenter(g: ObstacleGroup): P2 {
  let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
  for (const o of g.parts) {
    if (o.kind === 'disk') {
      x0 = Math.min(x0, o.c[0] - o.r); x1 = Math.max(x1, o.c[0] + o.r);
      y0 = Math.min(y0, o.c[1] - o.r); y1 = Math.max(y1, o.c[1] + o.r);
    } else {
      for (const p of o.pts) { x0 = Math.min(x0, p[0]); x1 = Math.max(x1, p[0]); y0 = Math.min(y0, p[1]); y1 = Math.max(y1, p[1]); }
    }
  }
  return [(x0 + x1) / 2, (y0 + y1) / 2];
}

export interface Clearance {
  /** Distance to the obstacle surface (0 inside). */
  d: number;
  /** Unit vector pointing away from the obstacle (= ∇d outside; the way out when inside). */
  n: P2;
}

/**
 * Distance from (x, y) to one obstacle and the outward direction. Written without allocations in
 * the inner loop because heat maps evaluate it tens of thousands of times per repaint
 * (potential.test.ts checks it against `distPointPolygon` from @/mr).
 */
export function obstacleClearance(o: Obstacle, x: number, y: number): Clearance {
  if (o.kind === 'disk') {
    const vx = x - o.c[0], vy = y - o.c[1];
    const L = Math.hypot(vx, vy);
    if (L < 1e-12) return { d: 0, n: [1, 0] };
    return { d: Math.max(0, L - o.r), n: [vx / L, vy / L] };
  }
  const P = o.pts;
  let best = Infinity, bx = 0, by = 0;
  let inside = false;
  for (let i = 0, j = P.length - 1; i < P.length; j = i++) {
    const ax = P[j][0], ay = P[j][1], ex = P[i][0] - ax, ey = P[i][1] - ay;
    const L2 = ex * ex + ey * ey;
    let t = L2 > 1e-18 ? ((x - ax) * ex + (y - ay) * ey) / L2 : 0;
    t = t < 0 ? 0 : t > 1 ? 1 : t;
    const cx = ax + t * ex, cy = ay + t * ey;
    const dd = (x - cx) * (x - cx) + (y - cy) * (y - cy);
    if (dd < best) { best = dd; bx = cx; by = cy; }
    // crossing-number test (ray toward +x)
    const yi = P[i][1], yj = ay;
    if ((yi > y) !== (yj > y) && x < ((ax - P[i][0]) * (y - yi)) / (yj - yi) + P[i][0]) inside = !inside;
  }
  const dist = Math.sqrt(best);
  if (inside || dist < 1e-12) {
    // inside (or on the boundary): the way out is toward the nearest boundary point
    return dist < 1e-12 ? { d: 0, n: [1, 0] } : { d: 0, n: [(bx - x) / dist, (by - y) / dist] };
  }
  return { d: dist, n: [(x - bx) / dist, (y - by) / dist] };
}

/** Clearance to the nearest of several obstacles (d = +∞ when there are none). */
export function nearestClearance(obstacles: readonly Obstacle[], x: number, y: number): Clearance {
  let best: Clearance = { d: Infinity, n: [1, 0] };
  for (const o of obstacles) {
    const c = obstacleClearance(o, x, y);
    if (c.d < best.d) best = c;
  }
  return best;
}

// ---------------------------------------------------------------------------------------------
// Attractive + repulsive potential
// ---------------------------------------------------------------------------------------------

export interface FieldParams {
  kAtt: number;
  /** Repulsive gain k. */
  kRep: number;
  /** Influence distance d0. */
  d0: number;
  goal: P2;
}

/**
 * The repulsive term is evaluated with d clamped to at least D_CLAMP: without this U and ∇U are
 * infinite on the obstacle boundary (the "clipping" the lesson mentions).
 */
export const D_CLAMP = 0.03;

export interface FieldEval {
  U: number;
  Uatt: number;
  Urep: number;
  /** ∇U and its attractive / repulsive parts. */
  g: P2;
  gAtt: P2;
  gRep: P2;
  /** Clearance to the nearest obstacle. */
  d: number;
}

export function evalField(p: FieldParams, obstacles: readonly Obstacle[], x: number, y: number): FieldEval {
  const ex = x - p.goal[0], ey = y - p.goal[1];
  const Uatt = 0.5 * p.kAtt * (ex * ex + ey * ey);
  let Urep = 0, rx = 0, ry = 0, dmin = Infinity;
  for (const o of obstacles) {
    const c = obstacleClearance(o, x, y);
    if (c.d < dmin) dmin = c.d;
    if (c.d >= p.d0) continue;
    const de = Math.max(c.d, D_CLAMP);
    const a = 1 / de - 1 / p.d0;
    Urep += 0.5 * p.kRep * a * a;
    // dU/dd = −k (1/d − 1/d0) / d²  and  ∇d = n
    const s = (-p.kRep * a) / (de * de);
    rx += s * c.n[0];
    ry += s * c.n[1];
  }
  const gAtt: P2 = [p.kAtt * ex, p.kAtt * ey];
  return { U: Uatt + Urep, Uatt, Urep, g: [gAtt[0] + rx, gAtt[1] + ry], gAtt, gRep: [rx, ry], d: dmin };
}

/** Scalar potential only (cheaper for rasters). */
export function potentialAt(p: FieldParams, obstacles: readonly Obstacle[], x: number, y: number): number {
  const ex = x - p.goal[0], ey = y - p.goal[1];
  let U = 0.5 * p.kAtt * (ex * ex + ey * ey);
  for (const o of obstacles) {
    const c = obstacleClearance(o, x, y);
    if (c.d >= p.d0) continue;
    const a = 1 / Math.max(c.d, D_CLAMP) - 1 / p.d0;
    U += 0.5 * p.kRep * a * a;
  }
  return U;
}

// ---------------------------------------------------------------------------------------------
// Sphere-world navigation function (Koditschek & Rimon)
// ---------------------------------------------------------------------------------------------

export interface SphereWorld {
  /** Circular workspace boundary. */
  center: P2;
  R: number;
  disks: { c: P2; r: number }[];
}

export interface NavEval {
  phi: number;
  /** ∇φ (can underflow to 0 far from the goal when κ is large). */
  g: P2;
  /** Unit vector of −∇φ, computed without cancellation (zero at critical points and outside). */
  dir: P2;
  free: boolean;
}

/**
 * φ(q) = γ / (γ^κ + β)^{1/κ} with γ = ‖q − q_goal‖², β = β0 Π βi, β0 = R² − ‖q − c0‖²,
 * βi = ‖q − ci‖² − ri². φ ∈ [0, 1), φ → 1 on every boundary; for κ large enough the goal is its
 * only minimum. Returns φ = 1 and ∇φ = 0 outside the free space.
 *
 * Evaluated through r = β/γ^κ:  φ = (1 + r)^{−1/κ},  ∇φ = −(φ/κ)·r/(1 + r)·(∇β/β − κ ∇γ/γ).
 * The direct formula subtracts two nearly equal terms when φ ≈ 1 (large κ) and returns noise.
 */
export function navFunction(w: SphereWorld, goal: P2, kappa: number, x: number, y: number): NavEval {
  // ∇ log β = Σ ∇β_i / β_i
  let logBeta = 0, lbx = 0, lby = 0;
  const b0 = w.R * w.R - ((x - w.center[0]) ** 2 + (y - w.center[1]) ** 2);
  if (b0 <= 0) return { phi: 1, g: [0, 0], dir: [0, 0], free: false };
  logBeta += Math.log(b0);
  lbx += (-2 * (x - w.center[0])) / b0;
  lby += (-2 * (y - w.center[1])) / b0;
  for (const d of w.disks) {
    const bi = (x - d.c[0]) ** 2 + (y - d.c[1]) ** 2 - d.r * d.r;
    if (bi <= 0) return { phi: 1, g: [0, 0], dir: [0, 0], free: false };
    logBeta += Math.log(bi);
    lbx += (2 * (x - d.c[0])) / bi;
    lby += (2 * (y - d.c[1])) / bi;
  }
  const ex = x - goal[0], ey = y - goal[1];
  const gamma = ex * ex + ey * ey;
  if (gamma < 1e-300) return { phi: 0, g: [0, 0], dir: [0, 0], free: true };
  // D = ∇ log β − κ ∇ log γ  (the direction of −∇φ)
  const Dx = lbx - (kappa * 2 * ex) / gamma, Dy = lby - (kappa * 2 * ey) / gamma;
  const lr = logBeta - kappa * Math.log(gamma); // log r
  const softplus = lr > 35 ? lr : lr < -35 ? Math.exp(lr) : Math.log1p(Math.exp(lr)); // log(1 + r)
  const phi = Math.exp(-softplus / kappa);
  const sig = lr > 35 ? 1 : lr < -700 ? 0 : 1 / (1 + Math.exp(-lr)); // r / (1 + r)
  const c = (phi / kappa) * sig;
  const Dn = Math.hypot(Dx, Dy);
  return { phi, g: [-c * Dx, -c * Dy], dir: Dn > 0 ? [Dx / Dn, Dy / Dn] : [0, 0], free: true };
}

/** Clearance inside a sphere world (to the nearest disk or to the outer circle). */
export function sphereClearance(w: SphereWorld, x: number, y: number): Clearance {
  const vx = x - w.center[0], vy = y - w.center[1];
  const L = Math.hypot(vx, vy);
  let best: Clearance = L < 1e-12 ? { d: w.R, n: [1, 0] } : { d: Math.max(0, w.R - L), n: [-vx / L, -vy / L] };
  for (const dk of w.disks) {
    const c = obstacleClearance({ kind: 'disk', c: dk.c, r: dk.r }, x, y);
    if (c.d < best.d) best = c;
  }
  return best;
}

// ---------------------------------------------------------------------------------------------
// Point-robot simulation
// ---------------------------------------------------------------------------------------------

export interface Env {
  /** Potential and gradient the robot feels. */
  field(x: number, y: number): { U: number; g: P2 };
  clearance(x: number, y: number): Clearance;
  goal: P2;
  /** Optional rectangular walls around the map (the point stops at them). */
  bounds?: Bounds;
  /**
   * Optional velocity command for the first-order model (default: −∇U clipped to v_max). Used by
   * the navigation-function mode, which follows the direction of −∇φ at a set speed.
   */
  command?(x: number, y: number, vMax: number): P2;
}

export type Model = 'first' | 'second';

export interface RobotParams {
  model: Model;
  /** Control period of the first-order controller (s). */
  dtCtrl: number;
  vMax: number;
  /** Clip on |−∇U| used as acceleration by the second-order model. */
  aMax: number;
  /** Damping b of the second-order model (1/s). */
  damping: number;
  goalTol: number;
}

export const DEFAULT_ROBOT: RobotParams = { model: 'first', dtCtrl: 0.1, vMax: 1.5, aMax: 4, damping: 1.5, goalTol: 0.1 };

export type RunStatus = 'running' | 'reached' | 'stuck' | 'timeout';

/** Distance below which the point counts as touching an obstacle and is pushed back out. */
export const D_CONTACT = 0.01;

export function saturate(v: P2, max: number): P2 {
  const L = Math.hypot(v[0], v[1]);
  return L > max ? [(v[0] * max) / L, (v[1] * max) / L] : [v[0], v[1]];
}

/**
 * A point robot driven by a potential. `step(h)` advances by h seconds (the caller sub-steps
 * with a fixed h). The first-order model recomputes its velocity command every `dtCtrl` seconds
 * and holds it in between, which is what a real controller running at 1/dtCtrl Hz does.
 */
export class PointRobot {
  q: P2;
  v: P2 = [0, 0];
  t = 0;
  /** Held velocity command (first-order model). */
  cmd: P2 = [0, 0];
  status: RunStatus = 'running';
  /** Positions every `sampleDt` seconds (trace and progress window). */
  readonly trace: P2[];
  /** Positions at the control updates (first-order model). */
  readonly ticks: P2[] = [];
  /** [t, ‖q − q_goal‖] every `sampleDt`. */
  readonly dist: [number, number][] = [];
  pathLength = 0;
  contacts = 0;
  /** Largest ‖q − q_goal‖ seen after first coming within 2·goalTol of the goal (overshoot). */
  overshoot = 0;
  /** In the progress window: moving back and forth without getting anywhere. */
  oscillating = false;
  private sinceTick = Infinity;
  private sinceSample = 0;
  private touching = false;
  private nearedGoal = false;

  constructor(
    public env: Env,
    public params: RobotParams,
    q0: P2,
    readonly sampleDt = 0.05,
    readonly tMax = 60,
    /** Seconds without net progress before the run counts as stuck. */
    readonly stallWindow = 2,
  ) {
    this.q = [q0[0], q0[1]];
    this.trace = [[q0[0], q0[1]]];
    this.dist.push([0, Math.hypot(q0[0] - env.goal[0], q0[1] - env.goal[1])]);
  }

  get distToGoal(): number { return Math.hypot(this.q[0] - this.env.goal[0], this.q[1] - this.env.goal[1]); }

  get speed(): number {
    return this.params.model === 'first' ? Math.hypot(this.cmd[0], this.cmd[1]) : Math.hypot(this.v[0], this.v[1]);
  }

  step(h: number): void {
    if (this.status !== 'running') return;
    const P = this.params;
    const x0 = this.q[0], y0 = this.q[1];
    if (P.model === 'first') {
      if (this.sinceTick >= P.dtCtrl - 1e-9) {
        this.sinceTick = 0;
        this.cmd = this.env.command
          ? this.env.command(this.q[0], this.q[1], P.vMax)
          : saturate(negate(this.env.field(this.q[0], this.q[1]).g), P.vMax);
        this.ticks.push([this.q[0], this.q[1]]);
      }
      this.sinceTick += h;
      this.v = [this.cmd[0], this.cmd[1]];
      this.q = [this.q[0] + this.cmd[0] * h, this.q[1] + this.cmd[1] * h];
    } else {
      // semi-implicit Euler: v ← v + h (sat(−∇U) − b v);  q ← q + h v
      const f = saturate(negate(this.env.field(this.q[0], this.q[1]).g), P.aMax);
      this.v = [this.v[0] + h * (f[0] - P.damping * this.v[0]), this.v[1] + h * (f[1] - P.damping * this.v[1])];
      this.q = [this.q[0] + h * this.v[0], this.q[1] + h * this.v[1]];
    }
    // contact guard: a bounded command cannot always stop a fast point mass at the boundary
    const c = this.env.clearance(this.q[0], this.q[1]);
    if (c.d < D_CONTACT) {
      const push = D_CONTACT - c.d;
      this.q = [this.q[0] + push * c.n[0], this.q[1] + push * c.n[1]];
      const vn = this.v[0] * c.n[0] + this.v[1] * c.n[1];
      if (vn < 0) this.v = [this.v[0] - vn * c.n[0], this.v[1] - vn * c.n[1]];
      if (!this.touching) this.contacts++;
      this.touching = true;
    } else this.touching = false;
    const B = this.env.bounds;
    if (B) {
      const qx = Math.min(B.xmax, Math.max(B.xmin, this.q[0])), qy = Math.min(B.ymax, Math.max(B.ymin, this.q[1]));
      if (qx !== this.q[0]) this.v = [0, this.v[1]];
      if (qy !== this.q[1]) this.v = [this.v[0], 0];
      this.q = [qx, qy];
    }
    this.pathLength += Math.hypot(this.q[0] - x0, this.q[1] - y0);
    this.t += h;
    this.sinceSample += h;
    if (this.sinceSample >= this.sampleDt - 1e-9) {
      this.sinceSample = 0;
      this.sample();
    }
  }

  private sample(): void {
    const P = this.params;
    const dg = this.distToGoal;
    this.trace.push([this.q[0], this.q[1]]);
    this.dist.push([this.t, dg]);
    if (dg < 2 * P.goalTol) this.nearedGoal = true;
    if (this.nearedGoal) this.overshoot = Math.max(this.overshoot, dg);
    if (dg < P.goalTol && this.speed < 0.15) { this.status = 'reached'; return; }
    if (this.t >= this.tMax) { this.status = 'timeout'; return; }
    // progress window: net displacement vs. distance travelled over the last `stallWindow` seconds
    const W = Math.round(this.stallWindow / this.sampleDt);
    const n = this.trace.length;
    if (n > W && dg > Math.max(0.25, 2 * P.goalTol)) {
      const a = this.trace[n - 1 - W], b = this.trace[n - 1];
      const net = Math.hypot(b[0] - a[0], b[1] - a[1]);
      let travelled = 0;
      for (let k = n - W; k < n; k++) travelled += Math.hypot(this.trace[k][0] - this.trace[k - 1][0], this.trace[k][1] - this.trace[k - 1][1]);
      this.oscillating = travelled > 0.3 && net < 0.25 * travelled;
      // The first-order controller can chatter in place around a minimum; a damped point mass
      // swinging through the goal also has ~0 net displacement, so it must really be at rest.
      if (net < 0.03 && (P.model === 'first' || travelled < 0.3)) this.status = 'stuck';
    }
  }

  /** Run until the status changes (or tMax), with fixed sub-step h. */
  run(h = 1 / 240): this {
    while (this.status === 'running') this.step(h);
    return this;
  }
}

function negate(g: P2): P2 { return [-g[0], -g[1]]; }

/** Environment of the attractive + repulsive field. */
export function fieldEnv(p: FieldParams, obstacles: readonly Obstacle[], bounds?: Bounds): Env {
  return {
    goal: p.goal,
    bounds,
    field: (x, y) => { const e = evalField(p, obstacles, x, y); return { U: e.U, g: e.g }; },
    clearance: (x, y) => nearestClearance(obstacles, x, y),
  };
}

/**
 * Environment of the sphere-world navigation function. φ spans many decades, so the robot follows
 * the direction of −∇φ at v_max and slows down linearly inside `slowRadius` of the goal.
 */
export function navEnv(w: SphereWorld, goal: P2, kappa: number, slowRadius = 0.6): Env {
  return {
    goal,
    field: (x, y) => { const e = navFunction(w, goal, kappa, x, y); return { U: e.phi, g: e.g }; },
    clearance: (x, y) => sphereClearance(w, x, y),
    command: (x, y, vMax) => {
      const e = navFunction(w, goal, kappa, x, y);
      const speed = vMax * Math.min(1, Math.hypot(x - goal[0], y - goal[1]) / slowRadius);
      return [speed * e.dir[0], speed * e.dir[1]];
    },
  };
}

/** Pseudo-gradient for normalized descent on a navigation function: −(unit descent direction). */
export function navDescentField(w: SphereWorld, goal: P2, kappa: number): (x: number, y: number) => { U: number; g: P2 } {
  return (x, y) => {
    const e = navFunction(w, goal, kappa, x, y);
    return { U: e.phi, g: [-e.dir[0], -e.dir[1]] };
  };
}

/**
 * Converge onto the critical point near q0 by gradient descent with a backtracking line search
 * (used to place the local-minimum marker precisely). Returns the point, U and ‖∇U‖ there.
 */
export function refineMinimum(field: (x: number, y: number) => { U: number; g: P2 }, q0: P2, iters = 300): { q: P2; U: number; gradNorm: number } {
  let q: P2 = [q0[0], q0[1]];
  let f = field(q[0], q[1]);
  let step = 0.05;
  for (let k = 0; k < iters; k++) {
    const gn = Math.hypot(f.g[0], f.g[1]);
    if (gn < 1e-9) break;
    let accepted = false;
    for (let tries = 0; tries < 30; tries++) {
      const s = step / gn;
      const cand: P2 = [q[0] - s * f.g[0], q[1] - s * f.g[1]];
      const fc = field(cand[0], cand[1]);
      if (fc.U <= f.U - 1e-4 * step * gn) { q = cand; f = fc; accepted = true; step = Math.min(0.2, step * 1.6); break; }
      step *= 0.5;
    }
    if (!accepted || step < 1e-10) break;
  }
  return { q, U: f.U, gradNorm: Math.hypot(f.g[0], f.g[1]) };
}

// ---------------------------------------------------------------------------------------------
// Basins of attraction: where does gradient descent end up from each start?
// ---------------------------------------------------------------------------------------------

export const BASIN_GOAL = 1;
export const BASIN_STUCK = 2;
export const BASIN_BLOCKED = 3;

export interface BasinOptions {
  /** Length of one descent step along −∇U/‖∇U‖ (before adaptive shrinking). */
  step: number;
  maxSteps: number;
  /** Starts whose descent comes this close to the goal count as reaching it. */
  goalTol: number;
}

/**
 * Classifies a grid of start positions (cell centres) by running a descent from each one: steps
 * of fixed length along −∇U/‖∇U‖ (so fields whose gradient spans many decades, such as a
 * navigation function, work too), halved whenever two consecutive steps reverse direction and
 * regrown otherwise. A descent stops when it reaches the goal, converges (step < 10⁻³ m: a
 * minimum), or enters a cell whose fate is already known (paths merge quickly, which makes the
 * whole grid cheap). Work is split into `run(budgetMs)` calls so a UI can spread it over frames.
 */
export class BasinJob {
  readonly cells: Uint8Array;
  /** Distinct non-goal end points (local minima, merged within 0.3 m). */
  readonly minima: P2[] = [];
  private next = 0;
  readonly cw: number;
  readonly ch: number;

  constructor(
    readonly bounds: Bounds,
    readonly nx: number,
    readonly ny: number,
    private readonly field: (x: number, y: number) => { U: number; g: P2 },
    private readonly blocked: (x: number, y: number) => boolean,
    readonly goal: P2,
    readonly opts: BasinOptions,
  ) {
    this.cells = new Uint8Array(nx * ny);
    this.cw = (bounds.xmax - bounds.xmin) / nx;
    this.ch = (bounds.ymax - bounds.ymin) / ny;
  }

  get done(): boolean { return this.next >= this.nx * this.ny; }
  get progress(): number { return this.next / (this.nx * this.ny); }

  center(k: number): P2 {
    const i = k % this.nx, j = (k - i) / this.nx;
    return [this.bounds.xmin + (i + 0.5) * this.cw, this.bounds.ymin + (j + 0.5) * this.ch];
  }

  private cellOf(x: number, y: number): number {
    const i = Math.floor((x - this.bounds.xmin) / this.cw), j = Math.floor((y - this.bounds.ymin) / this.ch);
    if (i < 0 || j < 0 || i >= this.nx || j >= this.ny) return -1;
    return j * this.nx + i;
  }

  /** Classify cells until `budgetMs` has elapsed (or everything is done). Returns `done`. */
  run(budgetMs = 8): boolean {
    const t0 = performance.now();
    while (!this.done) {
      for (let k = 0; k < 16 && !this.done; k++) this.classify(this.next++);
      if (performance.now() - t0 > budgetMs) break;
    }
    return this.done;
  }

  private classify(k: number): void {
    if (this.cells[k]) return;
    const [x0, y0] = this.center(k);
    if (this.blocked(x0, y0)) { this.cells[k] = BASIN_BLOCKED; return; }
    const o = this.opts;
    let x = x0, y = y0;
    // previous unit step direction and the adaptive step length: crossing a valley back and
    // forth halves the step, so the descent settles onto a minimum instead of bouncing across it
    let ux = 0, uy = 0, len = o.step;
    let fate = 0;
    for (let s = 0; s < o.maxSteps; s++) {
      if (Math.hypot(x - this.goal[0], y - this.goal[1]) < o.goalTol) { fate = BASIN_GOAL; break; }
      const c = this.cellOf(x, y);
      if (c >= 0 && c !== k && (this.cells[c] === BASIN_GOAL || this.cells[c] === BASIN_STUCK)) { fate = this.cells[c]; break; }
      const g = this.field(x, y).g;
      const gn = Math.hypot(g[0], g[1]);
      if (!(gn > 0)) break; // exactly critical
      const dx = -g[0] / gn, dy = -g[1] / gn;
      if (dx * ux + dy * uy < -0.3) len *= 0.5;
      else len = Math.min(o.step, len * 1.3);
      if (len < 1e-3) break; // converged onto a minimum
      ux = dx; uy = dy;
      x += len * dx; y += len * dy;
    }
    if (!fate) {
      // converged (or out of steps): reaching the goal's own minimum counts as success
      fate = Math.hypot(x - this.goal[0], y - this.goal[1]) < Math.max(o.goalTol, 2 * o.step) ? BASIN_GOAL : BASIN_STUCK;
      if (fate === BASIN_STUCK) this.addMinimum([x, y]);
    }
    this.cells[k] = fate;
  }

  private addMinimum(q: P2): void {
    if (this.minima.some((m) => Math.hypot(m[0] - q[0], m[1] - q[1]) < 0.3)) return;
    this.minima.push(q);
  }

  /** Fraction of free start cells that end at the goal. */
  goalFraction(): number {
    let free = 0, ok = 0;
    for (const c of this.cells) { if (c === BASIN_GOAL || c === BASIN_STUCK) free++; if (c === BASIN_GOAL) ok++; }
    return free ? ok / free : 0;
  }
}

// ---------------------------------------------------------------------------------------------
// Marching squares
// ---------------------------------------------------------------------------------------------

/**
 * Level set f = level of a scalar field sampled on an nx × ny grid of points (row-major, row 0 at
 * ymin, spanning the bounds edge to edge). Appends segments as [x0, y0, x1, y1] to `out`.
 * Saddle cells are resolved with the cell-centre average.
 */
export function contourSegments(values: ArrayLike<number>, nx: number, ny: number, b: Bounds, level: number, out: number[]): void {
  const dx = (b.xmax - b.xmin) / (nx - 1), dy = (b.ymax - b.ymin) / (ny - 1);
  const lerp = (va: number, vb: number): number => (Math.abs(vb - va) < 1e-300 ? 0.5 : (level - va) / (vb - va));
  for (let j = 0; j + 1 < ny; j++) {
    for (let i = 0; i + 1 < nx; i++) {
      const v00 = values[j * nx + i], v10 = values[j * nx + i + 1];
      const v01 = values[(j + 1) * nx + i], v11 = values[(j + 1) * nx + i + 1];
      const idx = (v00 > level ? 1 : 0) | (v10 > level ? 2 : 0) | (v11 > level ? 4 : 0) | (v01 > level ? 8 : 0);
      if (idx === 0 || idx === 15) continue;
      const x0 = b.xmin + i * dx, y0 = b.ymin + j * dy;
      // edge points: B(ottom) 00–10, R(ight) 10–11, T(op) 01–11, L(eft) 00–01
      const B = (): [number, number] => [x0 + lerp(v00, v10) * dx, y0];
      const R = (): [number, number] => [x0 + dx, y0 + lerp(v10, v11) * dy];
      const T = (): [number, number] => [x0 + lerp(v01, v11) * dx, y0 + dy];
      const L = (): [number, number] => [x0, y0 + lerp(v00, v01) * dy];
      const seg = (p: [number, number], q: [number, number]) => { out.push(p[0], p[1], q[0], q[1]); };
      switch (idx) {
        case 1: case 14: seg(L(), B()); break;
        case 2: case 13: seg(B(), R()); break;
        case 3: case 12: seg(L(), R()); break;
        case 4: case 11: seg(R(), T()); break;
        case 6: case 9: seg(B(), T()); break;
        case 7: case 8: seg(L(), T()); break;
        case 5: {
          const above = (v00 + v10 + v11 + v01) / 4 > level;
          if (above) { seg(B(), R()); seg(L(), T()); } else { seg(L(), B()); seg(R(), T()); }
          break;
        }
        case 10: {
          const above = (v00 + v10 + v11 + v01) / 4 > level;
          if (above) { seg(L(), B()); seg(R(), T()); } else { seg(B(), R()); seg(L(), T()); }
          break;
        }
      }
    }
  }
}

// ---------------------------------------------------------------------------------------------
// RRT with optional potential-biased sampling
// ---------------------------------------------------------------------------------------------

/** Small seedable PRNG (mulberry32), numbers in [0, 1). */
export function mulberry32(seed: number): () => number {
  let a = (seed >>> 0) || 0x9e3779b9;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

export interface BiasedRRTOptions {
  delta: number;
  /** Collision-check spacing along a local path. */
  eps: number;
  maxIterations: number;
  /** Probability of a potential-biased sample (0 = plain RRT). */
  pBias: number;
  /** Candidates per biased sample: the one with the lowest U becomes q_rand. */
  K: number;
}

/**
 * RRT on a box. With probability pBias the random configuration is "the best of K uniform
 * samples" (lowest potential), otherwise uniform — the uniform part keeps every region reachable,
 * so the planner stays probabilistically complete.
 */
export class BiasedRRT {
  readonly nodes: P2[] = [];
  readonly parent: number[] = [];
  iterations = 0;
  status: 'searching' | 'success' | 'failed' = 'searching';
  path: P2[] | null = null;
  lastSample: P2 | null = null;
  lastBiased = false;
  /** Biased samples drawn so far. */
  biased = 0;

  constructor(
    readonly bounds: Bounds,
    private readonly isFree: (x: number, y: number) => boolean,
    private readonly U: (x: number, y: number) => number,
    readonly start: P2,
    readonly goal: P2,
    readonly opts: BiasedRRTOptions,
    private readonly rng: () => number,
  ) {
    this.nodes.push([start[0], start[1]]);
    this.parent.push(-1);
  }

  get finished(): boolean { return this.status !== 'searching'; }

  private uniform(): P2 {
    const b = this.bounds;
    return [b.xmin + this.rng() * (b.xmax - b.xmin), b.ymin + this.rng() * (b.ymax - b.ymin)];
  }

  private sample(): P2 {
    this.lastBiased = this.opts.pBias > 0 && this.rng() < this.opts.pBias;
    if (!this.lastBiased) return this.uniform();
    this.biased++;
    let best = this.uniform(), bu = this.U(best[0], best[1]);
    for (let k = 1; k < this.opts.K; k++) {
      const q = this.uniform();
      const u = this.U(q[0], q[1]);
      if (u < bu) { best = q; bu = u; }
    }
    return best;
  }

  private nearest(q: P2): number {
    let best = 0, bd = Infinity;
    for (let i = 0; i < this.nodes.length; i++) {
      const d = (this.nodes[i][0] - q[0]) ** 2 + (this.nodes[i][1] - q[1]) ** 2;
      if (d < bd) { bd = d; best = i; }
    }
    return best;
  }

  edgeFree(a: P2, b: P2): boolean {
    const L = Math.hypot(b[0] - a[0], b[1] - a[1]);
    const n = Math.max(1, Math.ceil(L / this.opts.eps));
    for (let k = 1; k <= n; k++) {
      const s = k / n;
      if (!this.isFree(a[0] + s * (b[0] - a[0]), a[1] + s * (b[1] - a[1]))) return false;
    }
    return true;
  }

  step(): void {
    if (this.finished) return;
    this.iterations++;
    const qr = this.sample();
    this.lastSample = qr;
    const near = this.nearest(qr);
    const a = this.nodes[near];
    const dx = qr[0] - a[0], dy = qr[1] - a[1];
    const L = Math.hypot(dx, dy);
    if (L > 1e-9) {
      const s = Math.min(1, this.opts.delta / L);
      const qn: P2 = [a[0] + s * dx, a[1] + s * dy];
      if (this.edgeFree(a, qn)) {
        this.nodes.push(qn);
        this.parent.push(near);
        const id = this.nodes.length - 1;
        if (Math.hypot(qn[0] - this.goal[0], qn[1] - this.goal[1]) <= this.opts.delta && this.edgeFree(qn, this.goal)) {
          this.nodes.push([this.goal[0], this.goal[1]]);
          this.parent.push(id);
          this.status = 'success';
          const path: P2[] = [];
          for (let k = this.nodes.length - 1; k >= 0; k = this.parent[k]) path.push(this.nodes[k]);
          this.path = path.reverse();
          return;
        }
      }
    }
    if (this.iterations >= this.opts.maxIterations) this.status = 'failed';
  }

  run(): this {
    while (!this.finished) this.step();
    return this;
  }
}
