/**
 * Pure logic for the forward-dynamics / numerical-integration lesson.
 *
 * The arm is the lab's `planar2R` (L1 = L2 = 1 m, 1 kg point masses at the link
 * tips, gravity along −y). Two equivalent descriptions from @/mr are used:
 *  - the closed-form Lagrangian model (`planar2RDynamics` / `planar2RAccel`, the book's 2R example
 *    in §8.1; §8.4 gives the general closed form), cheap enough to integrate in real time with tiny steps;
 *  - the Newton–Euler model (`InverseDynamics` on `planar2R.Mlist/Glist/Slist`),
 *    used to show how forward dynamics is assembled from n + 1 inverse-dynamics calls (§8.5).
 * Integrators are the library's `eulerStepGeneric` (explicit Euler) and `rk4Step`.
 */
import {
  type Mat,
  type Vec,
  type Planar2RParams,
  planar2R,
  planar2RParams,
  planar2RDynamics,
  planar2RAccel,
  planarFK,
  InverseDynamics,
  ForwardDynamics,
  solve,
  eigSym,
  rk4Step,
  eulerStepGeneric,
} from '@/mr';

export const LINKS: [number, number] = [1, 1];
export const MASSES: [number, number] = [1, 1];
export const G0 = 9.81;
/** |θ̇_i| above this (rad/s) is treated as a numerical blow-up. */
export const OMEGA_LIMIT = 40;

export type Method = 'euler' | 'rk4';
/** Joint torque as a function of time (open loop). */
export type TorqueFn = (t: number) => Vec;
export const zeroTorque: TorqueFn = () => [0, 0];

/** Closed-form parameters of the point-mass 2R arm; gravity can be switched off. */
export function armParams(gravityOn: boolean): Planar2RParams {
  return planar2RParams(LINKS, MASSES, 'point-mass', gravityOn ? G0 : 0);
}

/** Gravity vector of the PoE / Newton–Euler model (arm in the x–y plane, gravity along −y). */
export function gravityVec(gravityOn: boolean): Vec {
  return [0, gravityOn ? -G0 : 0, 0];
}

/** State derivative of x = [θ1, θ2, θ̇1, θ̇2]: ẋ = [θ̇; M⁻¹(τ − c − g)]. */
export function armODE(P: Planar2RParams, tau: TorqueFn): (t: number, x: Vec) => Vec {
  return (t, x) => {
    const dd = planar2RAccel(P, [x[0], x[1]], [x[2], x[3]], tau(t));
    return [x[2], x[3], dd[0], dd[1]];
  };
}

/** One integration step of the arm with the chosen method. */
export function stepArm(method: Method, P: Planar2RParams, tau: TorqueFn, t: number, x: Vec, h: number): Vec {
  const f = armODE(P, tau);
  return method === 'rk4' ? rk4Step(f, t, x, h) : eulerStepGeneric(f, t, x, h);
}

/** Forward-dynamics evaluations per step (cost model). */
export const EVALS_PER_STEP: Record<Method, number> = { euler: 1, rk4: 4 };

export interface Energy { kinetic: number; potential: number; total: number }

/** Kinetic ½θ̇ᵀM(θ)θ̇ plus gravitational potential (zero level at the base). */
export function armEnergy(P: Planar2RParams, x: Vec): Energy {
  const { M } = planar2RDynamics(P, [x[0], x[1]], [x[2], x[3]]);
  const d1 = x[2];
  const d2 = x[3];
  const kinetic = 0.5 * (M[0][0] * d1 * d1 + 2 * M[0][1] * d1 * d2 + M[1][1] * d2 * d2);
  const y1 = P.r1 * Math.sin(x[0]);
  const y2 = P.L1 * Math.sin(x[0]) + P.r2 * Math.sin(x[0] + x[1]);
  const potential = P.g * (P.m1 * y1 + P.m2 * y2);
  return { kinetic, potential, total: kinetic + potential };
}

/** Tip position of the arm for state x. */
export function tipOf(x: Vec): [number, number] {
  const fk = planarFK(LINKS, [x[0], x[1]]);
  return [fk.tip[0], fk.tip[1]];
}

/** True when the state is finite and the joint rates are below the blow-up limit. */
export function isSane(x: Vec, omegaLimit = OMEGA_LIMIT): boolean {
  for (const v of x) if (!Number.isFinite(v)) return false;
  return Math.abs(x[2]) <= omegaLimit && Math.abs(x[3]) <= omegaLimit;
}

// ---------------------------------------------------------------------------
// Step 1: θ̈ = M⁻¹(τ − c − g) at one state
// ---------------------------------------------------------------------------

export interface AccelAnalysis {
  M: Mat;
  c: Vec;
  g: Vec;
  /** τ − c − g */
  net: Vec;
  ddtheta: Vec;
  /** Eigenvalues of M (descending); both positive because M is positive definite. */
  eig: [number, number];
}

export function analyzeAccel(P: Planar2RParams, theta: Vec, dtheta: Vec, tau: Vec): AccelAnalysis {
  const { M, c, g } = planar2RDynamics(P, theta, dtheta);
  const net = [tau[0] - c[0] - g[0], tau[1] - c[1] - g[1]];
  const ddtheta = solve(M, net);
  const e = eigSym(M).values;
  return { M, c, g, net, ddtheta, eig: [e[0], e[1]] };
}

// ---------------------------------------------------------------------------
// Step 2: forward dynamics from n + 1 inverse-dynamics calls (Newton–Euler)
// ---------------------------------------------------------------------------

export interface IdCall {
  /** 'bias' = ID(θ, θ̇, 0, g) = c + g; 'column' = ID(θ, 0, e_j, 0) = column j of M. */
  kind: 'bias' | 'column';
  /** Column index for kind 'column' (0-based), −1 otherwise. */
  index: number;
  dtheta: Vec;
  ddtheta: Vec;
  gravity: Vec;
  result: Vec;
}

export interface Assembly {
  calls: IdCall[];
  /** c(θ, θ̇) + g(θ) from the first call. */
  bias: Vec;
  /** Mass matrix assembled column by column. */
  M: Mat;
  /** Solution of M θ̈ = τ − bias. */
  ddtheta: Vec;
  /** The library's ForwardDynamics (same Newton–Euler model) for comparison. */
  ddthetaLibrary: Vec;
  /** Closed-form (Lagrangian, §8.4) result for comparison. */
  ddthetaClosed: Vec;
}

export function assembleForwardDynamics(theta: Vec, dtheta: Vec, tau: Vec, gravityOn: boolean): Assembly {
  const { Mlist, Glist, Slist } = planar2R;
  if (!Mlist || !Glist) throw new Error('planar2R has no dynamics description');
  const n = theta.length;
  const g = gravityVec(gravityOn);
  const zero6 = [0, 0, 0, 0, 0, 0];
  const zeroN = new Array(n).fill(0);
  const calls: IdCall[] = [];
  const bias = InverseDynamics(theta, dtheta, zeroN, g, zero6, Mlist, Glist, Slist);
  calls.push({ kind: 'bias', index: -1, dtheta: dtheta.slice(), ddtheta: zeroN.slice(), gravity: g, result: bias });
  const M: Mat = Array.from({ length: n }, () => new Array(n).fill(0));
  for (let j = 0; j < n; j++) {
    const e = zeroN.slice();
    e[j] = 1;
    const col = InverseDynamics(theta, zeroN, e, [0, 0, 0], zero6, Mlist, Glist, Slist);
    for (let i = 0; i < n; i++) M[i][j] = col[i];
    calls.push({ kind: 'column', index: j, dtheta: zeroN.slice(), ddtheta: e, gravity: [0, 0, 0], result: col });
  }
  const rhs = tau.map((t, i) => t - bias[i]);
  const ddtheta = solve(M, rhs);
  const ddthetaLibrary = ForwardDynamics(theta, dtheta, tau, g, zero6, Mlist, Glist, Slist);
  const ddthetaClosed = planar2RAccel(armParams(gravityOn), theta, dtheta, tau);
  return { calls, bias, M, ddtheta, ddthetaLibrary, ddthetaClosed };
}

// ---------------------------------------------------------------------------
// Live twins: whole fixed steps, divergence guard
// ---------------------------------------------------------------------------

/** One simulated copy of the arm advanced with a fixed method and whole steps. */
export class Twin {
  x: Vec;
  t = 0;
  steps = 0;
  diverged = false;
  E0 = 0;
  constructor(readonly method: Method, x0: Vec, P: Planar2RParams) {
    this.x = x0.slice();
    this.reset(x0, P);
  }

  reset(x0: Vec, P: Planar2RParams): void {
    this.x = x0.slice();
    this.t = 0;
    this.steps = 0;
    this.diverged = false;
    this.E0 = armEnergy(P, this.x).total;
  }

  /**
   * Take whole steps of size h while t + h ≤ tEnd (no partial steps, so the step
   * size really is h). Stops and flags `diverged` on a blow-up; the last sane
   * state is kept for display. Returns the number of steps taken.
   */
  advanceTo(tEnd: number, h: number, P: Planar2RParams, tau: TorqueFn, maxSteps = 20000): number {
    let k = 0;
    while (!this.diverged && this.t + h <= tEnd + 1e-9 && k < maxSteps) {
      const next = stepArm(this.method, P, tau, this.t, this.x, h);
      if (!isSane(next)) {
        this.diverged = true;
        break;
      }
      this.x = next;
      this.t += h;
      this.steps++;
      k++;
    }
    return k;
  }

  energy(P: Planar2RParams): number {
    return armEnergy(P, this.x).total;
  }
}

// ---------------------------------------------------------------------------
// Step 5: global error versus step size
// ---------------------------------------------------------------------------

/** Integrate from x0 over [0, T] with N = round(T / h) steps; null if it blows up. */
export function simulateTo(method: Method, P: Planar2RParams, x0: Vec, T: number, h: number, tau: TorqueFn = zeroTorque): Vec | null {
  const N = Math.max(1, Math.round(T / h));
  const hh = T / N;
  let x = x0.slice();
  for (let k = 0; k < N; k++) {
    x = stepArm(method, P, tau, k * hh, x, hh);
    if (!isSane(x)) return null;
  }
  return x;
}

export interface SweepRow {
  dt: number;
  /** Tip-position error at time T (m); NaN when the run diverged. */
  euler: number;
  rk4: number;
}

export interface Sweep {
  rows: SweepRow[];
  /** Reference final state (RK4 with a very small step). */
  ref: Vec;
  T: number;
  /** Least-squares slopes of log10(error) vs log10(Δt) over the smallest steps. */
  slopeEuler: number;
  slopeRK4: number;
}

export const SWEEP_DTS = [0.2, 0.1, 0.05, 0.025, 0.0125, 0.00625, 0.003125];

function tipError(x: Vec | null, ref: Vec): number {
  if (!x) return Number.NaN;
  const a = tipOf(x);
  const b = tipOf(ref);
  return Math.hypot(a[0] - b[0], a[1] - b[1]);
}

/**
 * Fit y = a + s·x by least squares on the `count` smallest-x points with finite,
 * positive errors (log–log). Returns NaN when fewer than two points qualify.
 */
export function fitLogSlope(dts: number[], errs: number[], count = 4): number {
  const pts = dts
    .map((d, i) => [d, errs[i]] as const)
    .filter(([d, e]) => d > 0 && Number.isFinite(e) && e > 1e-14)
    .sort((a, b) => a[0] - b[0])
    .slice(0, count)
    .map(([d, e]) => [Math.log10(d), Math.log10(e)] as const);
  if (pts.length < 2) return Number.NaN;
  const n = pts.length;
  const mx = pts.reduce((s, p) => s + p[0], 0) / n;
  const my = pts.reduce((s, p) => s + p[1], 0) / n;
  let sxy = 0;
  let sxx = 0;
  for (const [x, y] of pts) {
    sxy += (x - mx) * (y - my);
    sxx += (x - mx) * (x - mx);
  }
  return sxx > 0 ? sxy / sxx : Number.NaN;
}

/** Run Euler and RK4 for every Δt over [0, T] (no torque) and compare with a fine RK4 reference. */
export function errorSweep(P: Planar2RParams, x0: Vec, T = 1, dts: number[] = SWEEP_DTS, refSteps = 4096): Sweep {
  const ref = simulateTo('rk4', P, x0, T, T / refSteps);
  if (!ref) throw new Error('errorSweep: reference solution diverged');
  const rows = dts.map((dt) => ({
    dt,
    euler: tipError(simulateTo('euler', P, x0, T, dt), ref),
    rk4: tipError(simulateTo('rk4', P, x0, T, dt), ref),
  }));
  return {
    rows,
    ref,
    T,
    slopeEuler: fitLogSlope(rows.map((r) => r.dt), rows.map((r) => r.euler)),
    slopeRK4: fitLogSlope(rows.map((r) => r.dt), rows.map((r) => r.rk4)),
  };
}

/** Tip positions along a run (one point per step, stops at a blow-up) for drawing paths. */
export function tipPath(method: Method, P: Planar2RParams, x0: Vec, T: number, h: number): [number, number][] {
  const N = Math.max(1, Math.round(T / h));
  const hh = T / N;
  let x = x0.slice();
  const pts: [number, number][] = [tipOf(x)];
  for (let k = 0; k < N; k++) {
    x = stepArm(method, P, zeroTorque, k * hh, x, hh);
    if (!isSane(x)) break;
    pts.push(tipOf(x));
  }
  return pts;
}
