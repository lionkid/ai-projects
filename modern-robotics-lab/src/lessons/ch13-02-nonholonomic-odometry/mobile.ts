/**
 * Nonholonomic wheeled bases for ch13-02: unicycle, differential-drive and car-like models written
 * as drift-free control systems q̇ = G(q) u, their input sets, the Lie-bracket manoeuvres, a
 * parallel-parking plan and wheel-encoder odometry integrated with the SE(2) exponential.
 *
 * Conventions (Modern Robotics, chapter 13): q = (φ, x, y); body twist V_b = (ω, v_x, v_y) with the
 * angular part first; these bases cannot slide sideways, so v_y = 0. Planar twists are embedded in
 * SE(3) as [0, 0, ω, v_x, v_y, 0] and exponentiated with MatrixExp6 from @/mr.
 */
import { MatrixExp6, VecTose3, wrapToPi, type Mat } from '@/mr';

export interface Pose { phi: number; x: number; y: number }
export type Pt = [number, number];
/** Planar twist (ω, v_x, v_y). */
export type Twist2 = [number, number, number];
export type ModelKey = 'unicycle' | 'diff' | 'car';

/** Wheel radius r and HALF the distance between the two drive wheels d (m). */
export interface DiffParams { r: number; d: number }

export interface Limits {
  /** Forward-speed limit (m/s) of the unicycle and the car. */
  vmax: number;
  /** Turn-rate limit (rad/s) of the unicycle. */
  wmax: number;
  /** Wheel-speed limit (rad/s) of the differential drive. */
  umax: number;
  /** Minimum turning radius of the car (m). */
  rmin: number;
}

// ---------------------------------------------------------------------------------------------
// SE(2) integration
// ---------------------------------------------------------------------------------------------

/** Displacement exp([V] t) in the starting body frame, evaluated with MatrixExp6. */
export function expTwist2(V: Twist2, t: number): Pose {
  const T = MatrixExp6(VecTose3([0, 0, V[0] * t, V[1] * t, V[2] * t, 0]));
  return { phi: V[0] * t, x: T[0][3], y: T[1][3] };
}

/** Exact motion under a constant body twist: T ← T exp([V] t). */
export function stepPose(q: Pose, V: Twist2, t: number): Pose {
  const d = expTwist2(V, t);
  const c = Math.cos(q.phi), s = Math.sin(q.phi);
  return { phi: q.phi + d.phi, x: q.x + c * d.x - s * d.y, y: q.y + s * d.x + c * d.y };
}

/** First-order (Euler) update that moves along the OLD heading: what odometry should not do. */
export function stepEuler(q: Pose, V: Twist2, t: number): Pose {
  const c = Math.cos(q.phi), s = Math.sin(q.phi);
  return { phi: q.phi + V[0] * t, x: q.x + (c * V[1] - s * V[2]) * t, y: q.y + (s * V[1] + c * V[2]) * t };
}

// ---------------------------------------------------------------------------------------------
// q̇ = G(q) u
// ---------------------------------------------------------------------------------------------

/** Unicycle / car: inputs u = (v, ω); rows (φ̇, ẋ, ẏ). */
export function Gunicycle(phi: number): Mat {
  return [[0, 1], [Math.cos(phi), 0], [Math.sin(phi), 0]];
}

/** Differential drive: inputs u = (u_L, u_R) wheel speeds (rad/s); rows (φ̇, ẋ, ẏ). */
export function Gdiff(phi: number, p: DiffParams): Mat {
  const k = p.r / 2;
  return [[-p.r / (2 * p.d), p.r / (2 * p.d)], [k * Math.cos(phi), k * Math.cos(phi)], [k * Math.sin(phi), k * Math.sin(phi)]];
}

/** (u_L, u_R) → (v, ω). */
export function wheelsToVW(uL: number, uR: number, p: DiffParams): [number, number] {
  return [(p.r * (uL + uR)) / 2, (p.r * (uR - uL)) / (2 * p.d)];
}

/** (v, ω) → (u_L, u_R). */
export function vwToWheels(v: number, w: number, p: DiffParams): [number, number] {
  return [(v - w * p.d) / p.r, (v + w * p.d) / p.r];
}

/** Steering angle of a car with wheelbase ℓ that turns at ω while driving at v (0 when v = 0). */
export function steeringAngle(v: number, w: number, ell: number): number {
  if (Math.abs(v) < 1e-9) return 0;
  return Math.atan((ell * w) / v);
}

/**
 * Feasible inputs in the (v, ω) plane, as polygons: a rectangle for the unicycle, a diamond for the
 * differential drive (|u_L|, |u_R| ≤ u_max) and a bow tie for the car (|ω| ≤ |v| / r_min).
 */
export function inputSetPolys(model: ModelKey, lim: Limits, p: DiffParams): Pt[][] {
  if (model === 'unicycle') return [[[-lim.vmax, -lim.wmax], [lim.vmax, -lim.wmax], [lim.vmax, lim.wmax], [-lim.vmax, lim.wmax]]];
  if (model === 'diff') {
    const v = p.r * lim.umax, w = (p.r * lim.umax) / p.d;
    return [[[v, 0], [0, w], [-v, 0], [0, -w]]];
  }
  const w = lim.vmax / lim.rmin;
  return [[[0, 0], [lim.vmax, -w], [lim.vmax, w]], [[0, 0], [-lim.vmax, w], [-lim.vmax, -w]]];
}

/**
 * Keeps (v, ω) inside the input set. Unicycle: clip each input. Differential drive: scale both
 * inputs together until both wheels are within u_max (keeps the turning radius). Car: clip v, then
 * clip ω to ±|v| / r_min (so with v = 0 the car cannot turn at all).
 */
export function clampInput(model: ModelKey, v: number, w: number, lim: Limits, p: DiffParams): { v: number; w: number; clamped: boolean } {
  if (model === 'unicycle') {
    const v2 = Math.max(-lim.vmax, Math.min(lim.vmax, v));
    const w2 = Math.max(-lim.wmax, Math.min(lim.wmax, w));
    return { v: v2, w: w2, clamped: v2 !== v || w2 !== w };
  }
  if (model === 'diff') {
    const [uL, uR] = vwToWheels(v, w, p);
    const m = Math.max(Math.abs(uL), Math.abs(uR));
    if (m <= lim.umax) return { v, w, clamped: false };
    const s = lim.umax / m;
    return { v: v * s, w: w * s, clamped: true };
  }
  const v2 = Math.max(-lim.vmax, Math.min(lim.vmax, v));
  const wl = Math.abs(v2) / lim.rmin;
  const w2 = Math.max(-wl, Math.min(wl, w));
  return { v: v2, w: w2, clamped: v2 !== v || Math.abs(w2 - w) > 1e-12 };
}

// ---------------------------------------------------------------------------------------------
// Lie brackets and manoeuvres
// ---------------------------------------------------------------------------------------------

/**
 * [g₁, g₂] = (∂g₂/∂q) g₁ − (∂g₁/∂q) g₂ for g₁ = "drive forward" (0, cos φ, sin φ) and
 * g₂ = "turn" (1, 0, 0): (0, sin φ, −cos φ), the direction to the RIGHT of the heading.
 */
export function bracketUnicycle(phi: number): Twist2 {
  return [0, Math.sin(phi), -Math.cos(phi)];
}

/**
 * Car: g_L = (κ, cos φ, sin φ) (forward, full left lock) and g_R = (−κ, cos φ, sin φ), κ = 1/r_min.
 * [g_L, g_R] = 2κ (0, −sin φ, cos φ): sideways to the LEFT.
 */
export function bracketCar(phi: number, rmin: number): Twist2 {
  const k = 2 / rmin;
  return [0, -k * Math.sin(phi), k * Math.cos(phi)];
}

export interface Segment { v: number; w: number; T: number; label: string }

/**
 * The four-move manoeuvre whose net effect approximates a Lie bracket.
 * Unicycle / diff drive: forward a, turn left b (in place), backward a, turn right b → net ≈ a·b to
 * the right. Car: forward-left a, forward-right a, backward-left a, backward-right a (full lock) →
 * net ≈ 2a²/r_min to the left. `speed` (m/s) and `turnRate` (rad/s) only set the timing.
 */
export function bracketSequence(model: ModelKey, a: number, b: number, rmin: number, speed = 0.35, turnRate = 0.7): Segment[] {
  if (model === 'car') {
    const w = speed / rmin, T = a / speed;
    return [
      { v: speed, w, T, label: '① 前進＋左打' },
      { v: speed, w: -w, T, label: '② 前進＋右打' },
      { v: -speed, w: -w, T, label: '③ 後退＋左打' },
      { v: -speed, w, T, label: '④ 後退＋右打' },
    ];
  }
  return [
    { v: speed, w: 0, T: a / speed, label: '① 前進' },
    { v: 0, w: turnRate, T: b / turnRate, label: '② 左轉' },
    { v: -speed, w: 0, T: a / speed, label: '③ 後退' },
    { v: 0, w: -turnRate, T: b / turnRate, label: '④ 右轉' },
  ];
}

/** Second-order prediction of the net displacement (world x, y) of `bracketSequence` from q0. */
export function bracketPrediction(model: ModelKey, a: number, b: number, rmin: number, q0: Pose): Pt {
  const g = model === 'car' ? bracketCar(q0.phi, rmin).map((x) => x * a * a) : bracketUnicycle(q0.phi).map((x) => x * a * b);
  return [g[1], g[2]];
}

export const planDuration = (segs: Segment[]): number => segs.reduce((s, g) => s + g.T, 0);

/** Pose at time t along a plan of constant-input segments (exact; t is clamped to the plan). */
export function poseAlong(q0: Pose, segs: Segment[], t: number): { pose: Pose; seg: number; done: boolean } {
  let q = { ...q0 };
  let rest = Math.max(0, t);
  for (let i = 0; i < segs.length; i++) {
    const g = segs[i];
    if (rest <= g.T) return { pose: stepPose(q, [g.w, g.v, 0], rest), seg: i, done: false };
    q = stepPose(q, [g.w, g.v, 0], g.T);
    rest -= g.T;
  }
  return { pose: q, seg: segs.length - 1, done: true };
}

/** Sampled path (points every ≤ ds metres or dθ radians) and segment end poses of a plan. */
export function samplePlan(q0: Pose, segs: Segment[], ds = 0.02): { path: Pt[]; ends: Pose[]; final: Pose } {
  const path: Pt[] = [[q0.x, q0.y]];
  const ends: Pose[] = [];
  let q = { ...q0 };
  for (const g of segs) {
    const rate = Math.max(Math.abs(g.v), Math.abs(g.w) * 0.2, 1e-9);
    const n = Math.max(1, Math.ceil((rate * g.T) / ds));
    for (let k = 1; k <= n; k++) {
      const p = stepPose(q, [g.w, g.v, 0], (g.T * k) / n);
      path.push([p.x, p.y]);
    }
    q = stepPose(q, [g.w, g.v, 0], g.T);
    ends.push({ ...q });
  }
  return { path, ends, final: q };
}

/**
 * Parallel parking with a car: shift `lateral` metres to the left (negative = right) without
 * changing the heading. Two full-lock arcs of angle α (left then right) give the sideways shift
 * 2 r_min (1 − cos α) and a forward run of 2 r_min sin α, which a straight reverse cancels.
 */
export function parallelParkPlan(lateral: number, rmin: number, speed = 0.35): Segment[] {
  const L = Math.min(Math.abs(lateral), 3.9 * rmin);
  const sg = Math.sign(lateral) || 1;
  const alpha = Math.acos(1 - L / (2 * rmin));
  const Tarc = (alpha * rmin) / speed;
  const back = 2 * rmin * Math.sin(alpha);
  const w = speed / rmin;
  return [
    { v: speed, w: sg * w, T: Tarc, label: sg > 0 ? '① 前進＋左打' : '① 前進＋右打' },
    { v: speed, w: -sg * w, T: Tarc, label: sg > 0 ? '② 前進＋右打' : '② 前進＋左打' },
    { v: -speed, w: 0, T: back / speed, label: '③ 直線倒車' },
  ];
}

/** Distance and heading error between a pose and a target. */
export function poseError(q: Pose, target: Pose): { dist: number; dphi: number } {
  return { dist: Math.hypot(q.x - target.x, q.y - target.y), dphi: wrapToPi(q.phi - target.phi) };
}

// ---------------------------------------------------------------------------------------------
// Wheel-encoder odometry
// ---------------------------------------------------------------------------------------------

/** Deterministic PRNG (mulberry32) so noisy runs can be replayed. */
export function mulberry32(seed: number): () => number {
  let s = seed >>> 0;
  return () => {
    s = (s + 0x6d2b79f5) >>> 0;
    let t = s;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

/** Standard normal sample (Box–Muller). */
export function gaussian(rand: () => number): number {
  const u1 = Math.max(rand(), 1e-12), u2 = rand();
  return Math.sqrt(-2 * Math.log(u1)) * Math.cos(2 * Math.PI * u2);
}

/** Body displacement (Δφ, Δx_b, 0) measured from wheel-angle increments with the NOMINAL r, d. */
export function odomDelta(dL: number, dR: number, p: DiffParams): Twist2 {
  return [(p.r * (dR - dL)) / (2 * p.d), (p.r * (dL + dR)) / 2, 0];
}

export type OdoRule = 'exp' | 'euler';

export interface OdoConfig {
  /** Parameters the estimator believes in. */
  p: DiffParams;
  /** Encoder sampling rate (Hz). */
  rate: number;
  rule: OdoRule;
  /** Standard deviation of the fractional wheel slip per encoder sample. */
  slip: number;
  /** Relative radius mismatch δ: true radii r(1 − δ/2) (left) and r(1 + δ/2) (right). */
  delta: number;
  seed: number;
}

/**
 * Dead reckoning from wheel encoders. The TRUE motion is given (v, ω per time step); the wheels
 * that produce it have the true radii, and slip perturbs every encoder sample. The estimator
 * integrates the measured increments with the nominal radius, once per sample, by the exponential
 * (exact for a constant twist during the sample) or by the Euler shortcut.
 */
export class WheelOdometry {
  est: Pose;
  cfg: OdoConfig;
  /** Last measured wheel increments (rad) and the body displacement they imply. */
  lastDL = 0;
  lastDR = 0;
  lastDelta: Twist2 = [0, 0, 0];
  updates = 0;
  private accL = 0;
  private accR = 0;
  private tAcc = 0;
  private rand: () => number;

  constructor(q0: Pose, cfg: OdoConfig) {
    this.est = { ...q0 };
    this.cfg = { ...cfg };
    this.rand = mulberry32(cfg.seed);
  }

  reset(q0: Pose, seed = this.cfg.seed): void {
    this.est = { ...q0 };
    this.accL = this.accR = this.tAcc = 0;
    this.lastDL = this.lastDR = 0;
    this.lastDelta = [0, 0, 0];
    this.updates = 0;
    this.rand = mulberry32(seed);
  }

  /** True wheel radii. */
  radii(): [number, number] {
    const { r } = this.cfg.p, dl = this.cfg.delta;
    return [r * (1 - dl / 2), r * (1 + dl / 2)];
  }

  /**
   * Time left until the current encoder sample closes (s). A caller that never steps past it (by
   * splitting its integration step there) gets samples exactly every 1/rate seconds; otherwise a
   * sample closes at the first step that reaches the period, so the effective rate is quantised to
   * the step size (e.g. 50 Hz with 1/120 s steps would really be 40 Hz).
   */
  timeToSample(): number {
    return Math.max(0, 1 / this.cfg.rate - this.tAcc);
  }

  /** Feed the true motion (v, ω) for h seconds. Returns true when an odometry update happened. */
  step(v: number, w: number, h: number): boolean {
    const [rL, rR] = this.radii();
    const d = this.cfg.p.d;
    this.accL += ((v - w * d) * h) / rL;
    this.accR += ((v + w * d) * h) / rR;
    this.tAcc += h;
    if (this.tAcc + 1e-12 < 1 / this.cfg.rate) return false;
    this.update();
    return true;
  }

  /** Close the current encoder sample now (also used to flush before a reset). */
  update(): void {
    const s = this.cfg.slip;
    const dL = this.accL * (1 + (s > 0 ? s * gaussian(this.rand) : 0));
    const dR = this.accR * (1 + (s > 0 ? s * gaussian(this.rand) : 0));
    this.accL = this.accR = this.tAcc = 0;
    const D = odomDelta(dL, dR, this.cfg.p);
    this.est = this.cfg.rule === 'exp' ? stepPose(this.est, D, 1) : stepEuler(this.est, D, 1);
    this.lastDL = dL;
    this.lastDR = dR;
    this.lastDelta = D;
    this.updates++;
  }
}

/**
 * Small-δ prediction of the odometry heading error after driving a signed distance s with the
 * radius mismatch δ: φ_est − φ_true ≈ −δ s / (2d) (the larger right wheel turns less, so the
 * estimator believes the base turned right).
 */
export function headingDriftPrediction(delta: number, s: number, d: number): number {
  return (-delta * s) / (2 * d);
}
