/**
 * Pure models for the actuation / gearing / friction lesson (book §8.9).
 *
 *  - Ideal gearbox with ratio G: τ_joint = G τ_m, θ̇_joint = θ̇_m / G.
 *  - Reflected (apparent) rotor inertia G² J_m.
 *  - Friction: viscous b θ̇ + Coulomb τ_c sgn(θ̇) with static (breakaway) friction τ_s ≥ τ_c,
 *    and three numerical treatments of sgn(θ̇) at zero speed: Karnopp stick/dead-zone,
 *    tanh smoothing, and the naive sgn (which chatters).
 *  - A single geared joint: DC motor (torque–speed line) → gearbox → link (pendulum),
 *    with motor-side Coulomb friction reflected to the joint as G τ_c,m.
 *  - The lab's planar 2R arm (1 m links, 1 kg tip masses) with a geared motor at each
 *    joint. Motor 1 sits on the base; motor 2 is mounted on link 1, so its rotor turns at
 *    θ̇1 + G θ̇2. The rotor kinetic energy adds the CONSTANT matrix
 *        ΔM = [[G1² J1 + J2, G2 J2], [G2 J2, G2² J2]]
 *    to M(θ); being constant it changes neither c(θ, θ̇) nor g(θ) (rotor masses are
 *    lumped into the link masses).
 */
import {
  type Mat,
  type Vec,
  type Planar2RParams,
  planar2R,
  planar2RParams,
  planar2RDynamics,
  solve,
  InverseDynamics,
  VelQuadraticForces,
  GravityForces,
  QuinticTimeScalingDerivs,
  rk4Step,
} from '@/mr';

export const G0 = 9.81;
/** Gear ratios offered by the G slider. */
export const GEAR_RATIOS = [1, 2, 3, 5, 7, 10, 15, 20, 30, 50, 70, 100, 150, 200];

// ---------------------------------------------------------------------------
// Gearbox and reflected inertia
// ---------------------------------------------------------------------------

export interface ShaftState { torque: number; speed: number }

/** Ideal (lossless) gearbox: motor-side torque/speed → joint-side torque/speed. */
export function gearToJoint(G: number, motor: ShaftState): ShaftState {
  return { torque: G * motor.torque, speed: motor.speed / G };
}

/** Rotor inertia as seen from the joint. */
export function reflectedInertia(G: number, Jm: number): number {
  return G * G * Jm;
}

// ---------------------------------------------------------------------------
// Friction
// ---------------------------------------------------------------------------

export type FrictionMode = 'karnopp' | 'tanh' | 'sgn';

export interface Friction {
  /** Viscous coefficient b (N·m·s/rad). */
  b: number;
  /** Coulomb (kinetic) friction magnitude τ_c (N·m). */
  tc: number;
  /** Static (breakaway) friction τ_s (N·m); values below τ_c are treated as τ_c. Used by 'karnopp'. */
  ts: number;
  mode: FrictionMode;
  /** Velocity scale (rad/s): the stick window for 'karnopp', the smoothing width for 'tanh'. */
  vs?: number;
}

export const DEFAULT_VS: Record<FrictionMode, number> = { karnopp: 1e-3, tanh: 0.02, sgn: 0 };

/** Friction torque while sliding at speed v (sgn or tanh version of the Coulomb term). */
export function slidingFriction(v: number, f: Friction): number {
  const vs = f.vs ?? DEFAULT_VS[f.mode];
  const s = f.mode === 'tanh' ? Math.tanh(v / Math.max(vs, 1e-9)) : Math.sign(v);
  return f.b * v + f.tc * s;
}

export interface JointStep {
  theta: number;
  omega: number;
  /** True while static friction holds the joint (Karnopp only). */
  stuck: boolean;
  /** Friction torque that acted during the step (opposes motion / applied torque). */
  tauF: number;
}

/**
 * One semi-implicit Euler step of I θ̈ = τ_app − τ_f for a single joint, where τ_app is
 * the sum of every non-friction torque (motor, gravity, spring, external push).
 *  - 'karnopp': inside the window |ω| < vs the joint stays put as long as |τ_app| ≤ τ_s
 *    (friction balances τ_app exactly); a step that would reverse a sliding joint's
 *    velocity ends at rest instead. No chatter, and genuine stick–slip.
 *  - 'tanh' / 'sgn': the friction law is applied as is. sgn flips sign every step around
 *    ω = 0 (chatter); tanh is smooth but lets the joint creep slowly instead of sticking.
 */
export function jointStep(s: { theta: number; omega: number }, I: number, tauApp: number, f: Friction, h: number): JointStep {
  if (f.mode === 'karnopp') {
    const vs = f.vs ?? DEFAULT_VS.karnopp;
    const ts = Math.max(f.ts, f.tc);
    if (Math.abs(s.omega) < vs) {
      if (Math.abs(tauApp) <= ts) return { theta: s.theta, omega: 0, stuck: true, tauF: tauApp };
      // breakaway: friction drops to the kinetic level, opposing the applied torque
      const tauF = f.tc * Math.sign(tauApp);
      const omega = s.omega + (h * (tauApp - tauF)) / I;
      return { theta: s.theta + h * omega, omega, stuck: false, tauF };
    }
    const tauF = f.b * s.omega + f.tc * Math.sign(s.omega);
    const omega = s.omega + (h * (tauApp - tauF)) / I;
    if (Math.sign(omega) !== Math.sign(s.omega) && Math.abs(tauApp) <= ts) {
      // the joint comes to rest inside this step and static friction takes over
      return { theta: s.theta, omega: 0, stuck: true, tauF: tauApp + (I * s.omega) / h };
    }
    return { theta: s.theta + h * omega, omega, stuck: false, tauF };
  }
  const tauF = slidingFriction(s.omega, f);
  const omega = s.omega + (h * (tauApp - tauF)) / I;
  return { theta: s.theta + h * omega, omega, stuck: false, tauF };
}

// ---------------------------------------------------------------------------
// Single geared joint: DC motor → gearbox → link
// ---------------------------------------------------------------------------

/** Small DC motor with a current-limited drive: torque available falls linearly with speed. */
export const MOTOR = { stallTorque: 0.5, noLoadSpeed: 600 } as const;

/** Torque the motor actually delivers for command τ_cmd at motor speed ω_m (torque–speed line). */
export function motorTorque(tauCmd: number, omegaM: number): number {
  if (tauCmd === 0) return 0;
  const s = Math.sign(tauCmd);
  const avail = Math.max(0, MOTOR.stallTorque * (1 - (s * omegaM) / MOTOR.noLoadSpeed));
  return s * Math.min(Math.abs(tauCmd), avail);
}

export interface SingleJointParams {
  G: number;
  /** Rotor inertia (kg·m²). */
  Jm: number;
  /** Link: point mass m (kg) at distance L (m) from the joint. */
  m: number;
  L: number;
  gravity: boolean;
  /** Joint-side friction (bearings, output stage). */
  friction: Friction;
  /** Motor-side Coulomb friction (N·m); the joint feels G times this value. */
  tcMotor: number;
}

export type Drive =
  /** Constant motor torque command (N·m, motor side). */
  | { kind: 'motor'; tauCmd: number }
  /** Motor unpowered; an external torque tauExt acts on the link during [t0, t0 + dur). */
  | { kind: 'push'; tauExt: number; t0: number; dur: number }
  /**
   * The gearbox output is driven (by a speed-controlled motor) along a slow triangle wave of
   * amplitude amp and speed omegaOut; the link follows through a torsional spring k.
   */
  | { kind: 'spring'; k: number; omegaOut: number; amp: number };

/** Link inertia about the joint plus (when the rotor is rigidly geared to it) G² J_m. */
export function jointInertia(p: SingleJointParams, rotorCoupled = true): number {
  return p.m * p.L * p.L + (rotorCoupled ? reflectedInertia(p.G, p.Jm) : 0);
}

/** Friction felt at the joint: the joint-side law plus motor-side Coulomb friction × G. */
export function jointFriction(p: SingleJointParams, rotorCoupled = true): Friction {
  const extra = rotorCoupled ? p.G * p.tcMotor : 0;
  const tc = p.friction.tc + extra;
  return { ...p.friction, tc, ts: Math.max(p.friction.ts, p.friction.tc) + extra };
}

/** Gravity torque on the link, θ measured from the hanging-down position (CCW positive). */
export function gravityTorque(p: SingleJointParams, theta: number): number {
  return p.gravity ? p.m * G0 * p.L * Math.sin(theta) : 0;
}

/** Triangle wave starting at 0 with slope +omegaOut, bounded by ±amp. */
export function triangleWave(t: number, omegaOut: number, amp: number): { angle: number; rate: number } {
  if (amp <= 0 || omegaOut <= 0) return { angle: 0, rate: 0 };
  const period = (4 * amp) / omegaOut;
  const u = (((t % period) + period) % period) / period; // 0..1
  if (u < 0.25) return { angle: 4 * amp * u, rate: omegaOut };
  if (u < 0.75) return { angle: amp - 4 * amp * (u - 0.25), rate: -omegaOut };
  return { angle: -amp + 4 * amp * (u - 0.75), rate: omegaOut };
}

/** Torque needed from outside to start the joint at angular acceleration alpha from rest (θ = 0). */
export function backdriveTorque(p: SingleJointParams, alpha: number): number {
  const f = jointFriction(p, true);
  return jointInertia(p, true) * alpha + Math.max(f.ts, f.tc);
}

export class SingleJointSim {
  theta: number;
  omega = 0;
  t = 0;
  stuck = false;
  /** Last-step torques at the joint (N·m). */
  tauF = 0;
  tauDrive = 0;
  tauGrav = 0;
  /** Delivered motor torque (motor side). */
  tauMotor = 0;
  maxAbsTheta = 0;

  constructor(public p: SingleJointParams, public drive: Drive, theta0 = 0) {
    this.theta = theta0;
  }

  get rotorCoupled(): boolean {
    return this.drive.kind !== 'spring';
  }

  /** Gearbox output angle (spring drive) or the link angle itself. */
  outputAngle(): number {
    return this.drive.kind === 'spring' ? triangleWave(this.t, this.drive.omegaOut, this.drive.amp).angle : this.theta;
  }

  /** Motor shaft speed (rad/s). */
  motorSpeed(): number {
    if (this.drive.kind === 'spring') return this.p.G * triangleWave(this.t, this.drive.omegaOut, this.drive.amp).rate;
    return this.p.G * this.omega;
  }

  pushActive(): boolean {
    const d = this.drive;
    return d.kind === 'push' && this.t >= d.t0 && this.t < d.t0 + d.dur;
  }

  step(h: number): void {
    const p = this.p;
    const coupled = this.rotorCoupled;
    const I = jointInertia(p, coupled);
    const f = coupled ? jointFriction(p, true) : p.friction;
    this.tauGrav = gravityTorque(p, this.theta);
    const d = this.drive;
    this.tauMotor = 0;
    if (d.kind === 'motor') {
      this.tauMotor = motorTorque(d.tauCmd, p.G * this.omega);
      this.tauDrive = p.G * this.tauMotor;
    } else if (d.kind === 'push') {
      this.tauDrive = this.pushActive() ? d.tauExt : 0;
    } else {
      this.tauDrive = d.k * (triangleWave(this.t, d.omegaOut, d.amp).angle - this.theta);
    }
    const r = jointStep({ theta: this.theta, omega: this.omega }, I, this.tauDrive - this.tauGrav, f, h);
    this.theta = r.theta;
    this.omega = r.omega;
    this.stuck = r.stuck;
    this.tauF = r.tauF;
    this.t += h;
    this.maxAbsTheta = Math.max(this.maxAbsTheta, Math.abs(this.theta));
  }

  /** Kinetic energy split: link ½ I_L θ̇² and rotor ½ G²J_m θ̇² (rotor geared to the link). */
  kineticSplit(): { link: number; rotor: number } {
    const w = this.omega;
    const rotorSpeed = this.motorSpeed();
    return { link: 0.5 * this.p.m * this.p.L * this.p.L * w * w, rotor: 0.5 * this.p.Jm * rotorSpeed * rotorSpeed };
  }
}

// ---------------------------------------------------------------------------
// Planar 2R arm with geared motors
// ---------------------------------------------------------------------------

/** Closed-form parameters of the lab's planar2R (L = 1, 1 m; 1 kg tip masses). */
export function arm2Params(gravity: boolean): Planar2RParams {
  return planar2RParams([1, 1], [1, 1], 'point-mass', gravity ? G0 : 0);
}

/** Constant rotor contribution ΔM (motor 1 on the base, motor 2 on link 1). */
export function rotorMatrix(G1: number, J1: number, G2: number, J2: number): Mat {
  return [
    [G1 * G1 * J1 + J2, G2 * J2],
    [G2 * J2, G2 * G2 * J2],
  ];
}

/** M_eff(θ) = M(θ) + ΔM with the same G and J_m at both joints. */
export function effectiveMassMatrix(P: Planar2RParams, theta: Vec, G: number, Jm: number): Mat {
  const { M } = planar2RDynamics(P, theta, [0, 0]);
  const D = rotorMatrix(G, Jm, G, Jm);
  return [
    [M[0][0] + D[0][0], M[0][1] + D[0][1]],
    [M[1][0] + D[1][0], M[1][1] + D[1][1]],
  ];
}

/** Normalised coupling |M12| / √(M11 M22) ∈ [0, 1): 0 = decoupled joints. */
export function couplingIndex(M: Mat): number {
  return Math.abs(M[0][1]) / Math.sqrt(M[0][0] * M[1][1]);
}

/** θ̈ = M_eff⁻¹(τ − c − g) for the geared arm (no friction). G = 1, Jm = 0 gives the bare arm. */
export function gearedAccel(P: Planar2RParams, theta: Vec, dtheta: Vec, tau: Vec, G: number, Jm: number): Vec {
  const { M, c, g } = planar2RDynamics(P, theta, dtheta);
  const D = rotorMatrix(G, Jm, G, Jm);
  const Me = [
    [M[0][0] + D[0][0], M[0][1] + D[0][1]],
    [M[1][0] + D[1][0], M[1][1] + D[1][1]],
  ];
  return solve(Me, [tau[0] - c[0] - g[0], tau[1] - c[1] - g[1]]);
}

/** Kinetic energy ½ θ̇ᵀ M_eff θ̇ (links + rotors). */
export function gearedKinetic(P: Planar2RParams, theta: Vec, dtheta: Vec, G: number, Jm: number): number {
  const M = effectiveMassMatrix(P, theta, G, Jm);
  const [a, b] = dtheta;
  return 0.5 * (M[0][0] * a * a + 2 * M[0][1] * a * b + M[1][1] * b * b);
}

export interface Arm2Drive {
  G: number;
  Jm: number;
  gravity: boolean;
  /** Joint-side friction (same law at both joints). */
  friction: Friction;
  /** Motor-side Coulomb friction (N·m), felt as G·tcMotor at each joint. */
  tcMotor: number;
}

export interface TorqueBreakdown {
  /** M(θ)θ̈ of the links. */
  inertial: Vec;
  /** c(θ, θ̇). */
  velocity: Vec;
  /** g(θ). */
  gravity: Vec;
  /** ΔM θ̈ of the rotors. */
  rotor: Vec;
  /** Friction (smoothed with tanh so the bars do not flicker at the turning points). */
  friction: Vec;
  /** Joint torque τ and the motor torque τ / G. */
  total: Vec;
  motor: Vec;
}

/**
 * Joint torques needed to follow (θ, θ̇, θ̈): the link part comes from recursive
 * Newton–Euler (@/mr InverseDynamics on planar2R), plus the rotor term ΔM θ̈ and friction.
 */
export function torqueBreakdown(theta: Vec, dtheta: Vec, ddtheta: Vec, d: Arm2Drive): TorqueBreakdown {
  const { Mlist, Glist, Slist } = planar2R;
  if (!Mlist || !Glist) throw new Error('planar2R has no dynamics description');
  const gv = [0, d.gravity ? -G0 : 0, 0];
  const zero6 = [0, 0, 0, 0, 0, 0];
  const link = InverseDynamics(theta, dtheta, ddtheta, gv, zero6, Mlist, Glist, Slist);
  const velocity = VelQuadraticForces(theta, dtheta, Mlist, Glist, Slist);
  const gravity = GravityForces(theta, gv, Mlist, Glist, Slist);
  const inertial = link.map((v, i) => v - velocity[i] - gravity[i]);
  const D = rotorMatrix(d.G, d.Jm, d.G, d.Jm);
  const rotor = [D[0][0] * ddtheta[0] + D[0][1] * ddtheta[1], D[1][0] * ddtheta[0] + D[1][1] * ddtheta[1]];
  const fj: Friction = { ...d.friction, mode: 'tanh', tc: d.friction.tc + d.G * d.tcMotor, vs: 0.05 };
  const friction = dtheta.map((w) => slidingFriction(w, fj));
  const total = link.map((v, i) => v + rotor[i] + friction[i]);
  return { inertial, velocity, gravity, rotor, friction, total, motor: total.map((v) => v / d.G) };
}

export interface TrajSample {
  theta: Vec;
  dtheta: Vec;
  ddtheta: Vec;
  /** Time-scaling value of the current segment and its derivatives. */
  s: number;
  sdot: number;
  sddot: number;
  /** Time inside the current segment. */
  tau: number;
  /** +1 while moving A → B, −1 while moving back. */
  direction: 1 | -1;
}

/** Back-and-forth joint motion A → B → A … with a quintic time scaling of duration Tf per leg. */
export function swingTrajectory(t: number, A: Vec, B: Vec, Tf: number): TrajSample {
  const k = Math.floor(t / Tf);
  const tau = t - k * Tf;
  const forward = k % 2 === 0;
  const { s, sdot, sddot } = QuinticTimeScalingDerivs(Tf, tau);
  const from = forward ? A : B;
  const to = forward ? B : A;
  const d = to.map((v, i) => v - from[i]);
  return {
    theta: from.map((v, i) => v + s * d[i]),
    dtheta: d.map((v) => sdot * v),
    ddtheta: d.map((v) => sddot * v),
    s,
    sdot,
    sddot,
    tau,
    direction: forward ? 1 : -1,
  };
}

// ---------------------------------------------------------------------------
// Coupling experiment: joint 1 follows a prescribed move, joint 2 is unpowered
// ---------------------------------------------------------------------------

/**
 * Joint 1 is servoed perfectly along a quintic move of `delta` rad in `T` s (identical in
 * every model); joint 2 receives no torque, so it only moves because of the coupling terms:
 *     M22,eff θ̈2 = −(M21,eff θ̈1 + c2 + g2).
 * A larger M22,eff (the rotor's G²J_m) means joint 2 is dragged along less.
 */
export class CouplingTwin {
  theta2: number;
  omega2 = 0;
  t = 0;
  /** Largest |θ2 − θ2(0)| seen so far (rad). */
  maxDev = 0;

  constructor(readonly P: Planar2RParams, readonly G: number, readonly Jm: number, readonly theta0: Vec, readonly delta: number, readonly T: number) {
    this.theta2 = theta0[1];
  }

  joint1(t: number): { th: number; dth: number; ddth: number } {
    const { s, sdot, sddot } = QuinticTimeScalingDerivs(this.T, Math.min(Math.max(t, 0), this.T));
    return { th: this.theta0[0] + s * this.delta, dth: sdot * this.delta, ddth: sddot * this.delta };
  }

  /** Joint-2 acceleration for the given joint-2 state at time t. */
  accel2(t: number, th2: number, w2: number): number {
    const j = this.joint1(t);
    const { M, c, g } = planar2RDynamics(this.P, [j.th, th2], [j.dth, w2]);
    const D = rotorMatrix(this.G, this.Jm, this.G, this.Jm);
    return -((M[1][0] + D[1][0]) * j.ddth + c[1] + g[1]) / (M[1][1] + D[1][1]);
  }

  step(h: number): void {
    const f = (t: number, x: Vec): Vec => [x[1], this.accel2(t, x[0], x[1])];
    const x = rk4Step(f, this.t, [this.theta2, this.omega2], h);
    this.theta2 = x[0];
    this.omega2 = x[1];
    this.t += h;
    this.maxDev = Math.max(this.maxDev, Math.abs(this.theta2 - this.theta0[1]));
  }

  /** Current joint angles. */
  theta(): Vec {
    return [this.joint1(this.t).th, this.theta2];
  }

  /** Torque joint 1 must deliver to follow its move (includes the rotor inertia). */
  tau1(): number {
    const j = this.joint1(this.t);
    const th = [j.th, this.theta2];
    const { M, c, g } = planar2RDynamics(this.P, th, [j.dth, this.omega2]);
    const D = rotorMatrix(this.G, this.Jm, this.G, this.Jm);
    const dd2 = this.accel2(this.t, this.theta2, this.omega2);
    return (M[0][0] + D[0][0]) * j.ddth + (M[0][1] + D[0][1]) * dd2 + c[0] + g[0];
  }
}

/** (t, s(t)) samples of the quintic time scaling over [0, Tf] (for the chapter-9 preview plot). */
export function quinticSamples(Tf: number, n: number): [number, number][] {
  return Array.from({ length: n + 1 }, (_, k) => {
    const t = (Tf * k) / n;
    return [t, QuinticTimeScalingDerivs(Tf, t).s] as [number, number];
  });
}
