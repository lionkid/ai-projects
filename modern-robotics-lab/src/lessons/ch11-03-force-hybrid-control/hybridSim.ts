/**
 * Planar 2R arm pressing on and wiping a table: plant + four controllers.
 *
 * Plant
 *  - The lab's planar 2R arm (L1 = L2 = 1 m, 1 kg point masses, gravity along −y) using the
 *    closed-form dynamics from `@/mr` (`planar2RDynamics`, same numbers as `mr.planar2R`).
 *  - The table is a unilateral spring–damper (force only while the tip is below the surface,
 *    never pulling) and frictionless, so it can only push along its normal.
 *  - Semi-implicit Euler with 0.2 ms sub-steps (the contact is a stiff ODE: even the hardest
 *    table, ω ≈ √(k_e/m) ≈ 450 rad/s, gets ~70 sub-steps per period).
 *
 * Geometry / sign conventions
 *  - Task space = tip position (x, y) in {s}; twists reduce to (v_x, v_y), wrenches to (f_x, f_y).
 *  - The controller's MODEL of the table: a line through C with unit tangent t = (cos α, sin α)
 *    and outward normal n = (−sin α, cos α). The REAL surface is that line shifted by Δ·n; the
 *    controller does not know Δ (it only matters to pure position control).
 *  - f ≥ 0 is the pressing force. The wrist sensor measures the force the TABLE applies to the
 *    robot, F_sensor = +f n; the force the robot applies to the table is F_tip = −F_sensor.
 *
 * Controller
 *  - Runs at 1 kHz on measurements that are one sample old; τ is held between samples (ZOH)
 *    and clipped to ±TAU_MAX. This finite rate/delay is what makes stiff contact hard.
 *  - Modes: 'position' (task-space PD on a target lying on the model surface), 'force'
 *    (pure force control τ = g + Jᵀ(F_d + K_fp F_e + K_fi∫F_e − K_damp V)), 'hybrid' (P / I−P
 *    split) and 'conflict' (hybrid plus a normal position PID: position AND force along n).
 */
import {
  planar2RParams, planar2RDynamics, planar2RIK, planarFK, planarTipJacobian, manipulability,
  matMul, matVec, matSub, matAdd, matScale, transpose, inv, eye,
  type Mat, type Vec,
} from '@/mr';

export type Vec2 = [number, number];
export type Mode = 'position' | 'force' | 'hybrid' | 'conflict';

export const LINKS = [1, 1];
/** Same parameters as `mr.planar2R` (1 kg point masses at the link tips). */
export const ARM = planar2RParams([1, 1], [1, 1], 'point-mass');
/** Controller period (s). */
export const TC = 1e-3;
/** Physics sub-step (s). */
export const H = 2e-4;
/** Joint torque limit (N·m). */
export const TAU_MAX = 200;
/** Point on the modelled table surface (m). */
export const SURF_C: Vec2 = [1.2, -0.5];
/** Anti-windup bound on the force integral term K_fi∫f_e (N). */
export const I_F_MAX = 30;
/** Bound on the conflicting normal position integral term (m/s²). */
const I_P_MAX = 150;
/** Regularisation added to Λ⁻¹ before inverting it (1/kg). */
const LAMBDA_REG = 1e-4;

export const GAINS = {
  /** Pure position control: task-space spring/damper (N/m, N·s/m). */
  position: { Kp: 5000, Kd: 150 },
  /** Tangential motion loop of the hybrid controller (1/s², 1/s). */
  motion: { Kp: 400, Kd: 40 },
  /** Extra normal position PID of the conflicting controller (1/s², 1/s, 1/s³). */
  conflict: { Kp: 400, Kd: 40, Ki: 2000 },
};

export interface SimParams {
  mode: Mode;
  /** Table stiffness (N/m). */
  ke: number;
  /** Table tilt α (rad). */
  alpha: number;
  /** Real surface offset along n relative to the model (m); unknown to the controller. */
  delta: number;
  /** Desired pressing force f_d (N). */
  fd: number;
  Kfp: number;
  /** Force integral gain (1/s). */
  Kfi: number;
  /** Damping in the force loop (N·s/m). */
  Kdamp: number;
  /** Tangential target s_d(t) = s0 + amp·sin(2π·freq·t). */
  s0: number;
  amp: number;
  freq: number;
}

export const DEFAULT_PARAMS: SimParams = {
  mode: 'hybrid', ke: 2e4, alpha: 0, delta: 0, fd: 10, Kfp: 1, Kfi: 10, Kdamp: 60,
  s0: 0, amp: 0.25, freq: 0.25,
};

export function surfaceAxes(alpha: number): { t: Vec2; n: Vec2 } {
  return { t: [Math.cos(alpha), Math.sin(alpha)], n: [-Math.sin(alpha), Math.cos(alpha)] };
}

/** Contact damping of the table: about 10 % of critical for a 1 kg probe. */
export function tableDamping(ke: number): number { return 0.2 * Math.sqrt(ke); }

const dot2 = (a: ArrayLike<number>, b: ArrayLike<number>) => a[0] * b[0] + a[1] * b[1];

export interface TaskModel {
  x: Vec2;
  v: Vec2;
  /** 2×2 tip Jacobian (rows v_x, v_y). */
  J: Mat;
  M: Mat;
  c: Vec;
  g: Vec;
  /** Λ⁻¹ = J M⁻¹ Jᵀ (always well defined). */
  Linv: Mat;
  /** Regularised task-space mass matrix Λ̃ = (Λ⁻¹ + εI)⁻¹. */
  L: Mat;
  /** J̇ θ̇ (tip acceleration when θ̈ = 0). */
  Jdqd: Vec2;
}

/** Task-space model of the arm; all robotics math comes from `@/mr`. */
export function taskModel(theta: Vec, dtheta: Vec): TaskModel {
  const fk = planarFK(LINKS, theta);
  const J = planarTipJacobian(LINKS, theta);
  const dyn = planar2RDynamics(ARM, theta, dtheta);
  const Linv = matMul(matMul(J, inv(dyn.M)), transpose(J));
  const L = inv(matAdd(Linv, matScale(eye(2), LAMBDA_REG)));
  // J̇θ̇ as the directional derivative of J along θ̇ (central difference).
  const e = 1e-6;
  const Jp = planarTipJacobian(LINKS, [theta[0] + e * dtheta[0], theta[1] + e * dtheta[1]]);
  const Jm = planarTipJacobian(LINKS, [theta[0] - e * dtheta[0], theta[1] - e * dtheta[1]]);
  const Jdqd = matVec(matScale(matSub(Jp, Jm), 1 / (2 * e)), dtheta) as Vec2;
  const v = matVec(J, dtheta) as Vec2;
  return { x: [fk.tip[0], fk.tip[1]], v, J, M: dyn.M, c: dyn.c, g: dyn.g, Linv, L, Jdqd };
}

/** Smallest singular value of the tip Jacobian (singularity indicator for the UI). */
export function sigmaMin(theta: Vec): number { return manipulability(planarTipJacobian(LINKS, theta)).sigmaMin; }

/**
 * Inertia-weighted projection P = I − Aᵀ(AΛ⁻¹Aᵀ)⁻¹AΛ⁻¹ for k constraint rows A (k×n).
 * P maps a commanded wrench to the part that accelerates the end-effector along the constraint;
 * I − P maps it to the part that only pushes against the constraint. Pass Λ⁻¹ = I for the
 * plain orthogonal projection.
 */
export function projectionP(A: Mat, Linv: Mat, reg = 1e-9): Mat {
  const ALinv = matMul(A, Linv);
  const S = matMul(ALinv, transpose(A));
  for (let i = 0; i < S.length; i++) S[i][i] += reg;
  return matSub(eye(Linv.length), matMul(matMul(transpose(A), inv(S)), ALinv));
}

/** Elbow-up joint angles that put the tip at p (falls back to a safe pose if unreachable). */
export function elbowUpIK(p: Vec2): Vec {
  const ik = planar2RIK(1, 1, p[0], p[1]);
  const sol = ik.solutions.find((s) => s.elbow === 'up') ?? ik.solutions[0];
  return sol ? [sol.theta1, sol.theta2] : [0.5, -1.7];
}

export interface TickInfo {
  tm: TaskModel;
  /** Constraint row in task coordinates (A = nᵀ) and in joint coordinates (A J(θ)). */
  A: Vec2;
  Ajoint: Vec2;
  P: Mat;
  IP: Mat;
  /** Motion-loop wrench before projection and force-loop wrench before projection. */
  Fm: Vec2;
  Ff: Vec2;
  /** Their projected parts actually sent to the arm. */
  Fmotion: Vec2;
  Fforce: Vec2;
  /** Tangential coordinate, its target and the model-surface height of the tip. */
  s: number;
  sd: number;
  hModel: number;
  /** Force integral term (N) and conflicting position integral term (m/s²). */
  intF: number;
  intP: number;
  /** Normal push (N, + = into the table) requested by the force loop and by the conflicting position loop. */
  pushForce: number;
  pushPos: number;
  saturated: boolean;
}

export class HybridSim {
  p: SimParams;
  t = 0;
  theta: Vec = [0, 0];
  dtheta: Vec = [0, 0];
  tau: Vec = [0, 0];
  /** True pressing force f (N) and contact flag, updated every sub-step. */
  f = 0;
  contact = false;
  /** Height of the tip above the REAL surface (m), negative = penetration. */
  h = 0;
  info: TickInfo | null = null;
  /** Per-mode controller memory. */
  private iF2: Vec2 = [0, 0];
  private iFn = 0;
  private iPn = 0;
  private phase = 0;
  private nextTick = 0;
  private acc = 0;
  private meas = { theta: [0, 0] as Vec, dtheta: [0, 0] as Vec, f: 0 };

  constructor(p: Partial<SimParams> = {}) {
    this.p = { ...DEFAULT_PARAMS, ...p };
    this.reset();
  }

  /** Put the tip `h0` above the real surface at the current tangential target, at rest. */
  reset(h0 = 0.03): void {
    const { t, n } = surfaceAxes(this.p.alpha);
    const s = this.p.s0;
    const p: Vec2 = [SURF_C[0] + s * t[0] + (this.p.delta + h0) * n[0], SURF_C[1] + s * t[1] + (this.p.delta + h0) * n[1]];
    this.theta = elbowUpIK(p);
    this.dtheta = [0, 0];
    this.t = 0; this.phase = 0; this.nextTick = 0; this.acc = 0;
    this.iF2 = [0, 0]; this.iFn = 0; this.iPn = 0;
    this.f = 0; this.contact = false;
    this.meas = { theta: this.theta.slice(), dtheta: [0, 0], f: 0 };
    this.tau = planar2RDynamics(ARM, this.theta, [0, 0]).g.slice();
    this.updateContact();
    this.controlTick();
  }

  /** Clear the integrators (e.g. after switching mode). */
  clearIntegrators(): void { this.iF2 = [0, 0]; this.iFn = 0; this.iPn = 0; }

  /** Advance by dt seconds using exact H sub-steps (the remainder carries over). */
  advance(dt: number): void {
    this.acc += dt;
    while (this.acc >= H - 1e-12) {
      this.step();
      this.acc -= H;
    }
  }

  /** Wiping target (s_d, ṡ_d, s̈_d) at the current phase. */
  target(): { sd: number; sdd: number; sddd: number } {
    const w = 2 * Math.PI * this.p.freq;
    const { s0, amp } = this.p;
    return { sd: s0 + amp * Math.sin(this.phase), sdd: amp * w * Math.cos(this.phase), sddd: -amp * w * w * Math.sin(this.phase) };
  }

  /** Real-surface height h, its rate, and the resulting pressing force (unilateral). */
  private contactAt(x: Vec2, v: Vec2): { h: number; f: number } {
    const { n } = surfaceAxes(this.p.alpha);
    const h = (x[0] - SURF_C[0]) * n[0] + (x[1] - SURF_C[1]) * n[1] - this.p.delta;
    if (h >= 0) return { h, f: 0 };
    const hd = dot2(v, n);
    const f = Math.max(0, -this.p.ke * h - tableDamping(this.p.ke) * hd);
    return { h, f };
  }

  private updateContact(): void {
    const fk = planarFK(LINKS, this.theta);
    const J = planarTipJacobian(LINKS, this.theta);
    const v = matVec(J, this.dtheta) as Vec2;
    const c = this.contactAt([fk.tip[0], fk.tip[1]], v);
    this.h = c.h; this.f = c.f; this.contact = c.h < 0;
  }

  /** One physics sub-step of length H (runs the controller when a sample is due). */
  step(): void {
    if (this.t >= this.nextTick - 1e-9) {
      this.controlTick();
      this.nextTick += TC;
    }
    const fk = planarFK(LINKS, this.theta);
    const J = planarTipJacobian(LINKS, this.theta);
    const v = matVec(J, this.dtheta) as Vec2;
    const c = this.contactAt([fk.tip[0], fk.tip[1]], v);
    this.h = c.h; this.f = c.f; this.contact = c.h < 0;
    const { n } = surfaceAxes(this.p.alpha);
    // Contact force on the robot is +f n → generalised force Jᵀ(f n).
    const tauC = [J[0][0] * n[0] * c.f + J[1][0] * n[1] * c.f, J[0][1] * n[0] * c.f + J[1][1] * n[1] * c.f];
    const dyn = planar2RDynamics(ARM, this.theta, this.dtheta);
    const b0 = this.tau[0] + tauC[0] - dyn.c[0] - dyn.g[0];
    const b1 = this.tau[1] + tauC[1] - dyn.c[1] - dyn.g[1];
    const M = dyn.M;
    const det = M[0][0] * M[1][1] - M[0][1] * M[1][0];
    const a0 = (M[1][1] * b0 - M[0][1] * b1) / det;
    const a1 = (-M[1][0] * b0 + M[0][0] * b1) / det;
    // semi-implicit Euler
    this.dtheta = [this.dtheta[0] + a0 * H, this.dtheta[1] + a1 * H];
    this.theta = [this.theta[0] + this.dtheta[0] * H, this.theta[1] + this.dtheta[1] * H];
    this.t += H;
    this.phase += 2 * Math.PI * this.p.freq * H;
  }

  /** Controller sample: uses the PREVIOUS sample's measurements, then stores the current ones. */
  private controlTick(): void {
    const p = this.p;
    const tm = taskModel(this.meas.theta, this.meas.dtheta);
    const fm = this.meas.f;
    const { t, n } = surfaceAxes(p.alpha);
    const A: Mat = [[n[0], n[1]]];
    const P = projectionP(A, tm.Linv);
    const IP = matSub(eye(2), P);
    const rel: Vec2 = [tm.x[0] - SURF_C[0], tm.x[1] - SURF_C[1]];
    const s = dot2(rel, t);
    const sdot = dot2(tm.v, t);
    const hModel = dot2(rel, n);
    const vn = dot2(tm.v, n);
    const { sd, sdd, sddd } = this.target();
    const fe = p.fd - fm;
    let Fm: Vec2 = [0, 0];
    let Ff: Vec2 = [0, 0];
    let Fmotion: Vec2 = [0, 0];
    let Fforce: Vec2 = [0, 0];
    let F: Vec2;
    let useC = true;
    let pushForce = 0;
    let pushPos = 0;
    const LJdqd = matVec(tm.L, tm.Jdqd);

    if (p.mode === 'position') {
      // Stiff task-space PD towards a target ON THE MODEL SURFACE (the real one is Δ away).
      const { Kp, Kd } = GAINS.position;
      const xd: Vec2 = [SURF_C[0] + sd * t[0], SURF_C[1] + sd * t[1]];
      const vd: Vec2 = [sdd * t[0], sdd * t[1]];
      const ad: Vec2 = [sddd * t[0], sddd * t[1]];
      const ff = matVec(tm.L, [ad[0] - tm.Jdqd[0], ad[1] - tm.Jdqd[1]]);
      Fm = [ff[0] + Kp * (xd[0] - tm.x[0]) + Kd * (vd[0] - tm.v[0]), ff[1] + Kp * (xd[1] - tm.x[1]) + Kd * (vd[1] - tm.v[1])];
      Fmotion = Fm;
      F = Fm;
    } else if (p.mode === 'force') {
      // Pure force control in every direction (book form, gravity compensation only).
      const Fd: Vec2 = [-p.fd * n[0], -p.fd * n[1]];
      const Ftip: Vec2 = [-fm * n[0], -fm * n[1]];
      const Fe: Vec2 = [Fd[0] - Ftip[0], Fd[1] - Ftip[1]];
      for (let k = 0; k < 2; k++) {
        this.iF2[k] += Fe[k] * TC;
        const lim = p.Kfi > 1e-9 ? I_F_MAX / p.Kfi : 0;
        this.iF2[k] = Math.max(-lim, Math.min(lim, this.iF2[k]));
      }
      Ff = [
        Fd[0] + p.Kfp * Fe[0] + p.Kfi * this.iF2[0] - p.Kdamp * tm.v[0],
        Fd[1] + p.Kfp * Fe[1] + p.Kfi * this.iF2[1] - p.Kdamp * tm.v[1],
      ];
      Fforce = Ff;
      F = Ff;
      useC = false;
    } else {
      // Hybrid: tangential motion loop + normal force loop, split by P and I − P.
      const { Kp, Kd } = GAINS.motion;
      const at = sddd + Kp * (sd - s) + Kd * (sdd - sdot);
      Fm = matVec(tm.L, [at * t[0], at * t[1]]) as Vec2;
      this.iFn += fe * TC;
      const lim = p.Kfi > 1e-9 ? I_F_MAX / p.Kfi : 0;
      this.iFn = Math.max(-lim, Math.min(lim, this.iFn));
      const fcmd = p.fd + p.Kfp * fe + p.Kfi * this.iFn;
      Ff = [-fcmd * n[0] - p.Kdamp * vn * n[0], -fcmd * n[1] - p.Kdamp * vn * n[1]];
      pushForce = fcmd + p.Kdamp * vn;
      Fmotion = matVec(P, Fm) as Vec2;
      Fforce = matVec(IP, Ff) as Vec2;
      if (p.mode === 'conflict') {
        // ALSO servo the normal position to the model surface (h_model → 0) with a PID: this
        // specifies position and force along the same direction, which is inconsistent.
        const G = GAINS.conflict;
        const en = -hModel;
        this.iPn += en * TC;
        this.iPn = Math.max(-I_P_MAX / G.Ki, Math.min(I_P_MAX / G.Ki, this.iPn));
        const an = G.Kp * en + G.Ki * this.iPn - G.Kd * vn;
        const Fn = matVec(tm.L, [an * n[0], an * n[1]]);
        pushPos = -(Fn[0] * n[0] + Fn[1] * n[1]);
        // No projections: both loops act on the normal direction at full strength.
        Fmotion = [Fm[0] + Fn[0], Fm[1] + Fn[1]];
        Fforce = Ff;
      }
      F = [Fmotion[0] + Fforce[0] - LJdqd[0], Fmotion[1] + Fforce[1] - LJdqd[1]];
    }

    const JT = transpose(tm.J);
    const tauF = matVec(JT, F);
    let saturated = false;
    const tau = [0, 1].map((i) => {
      const raw = tm.g[i] + (useC ? tm.c[i] : 0) + tauF[i];
      if (Math.abs(raw) > TAU_MAX) saturated = true;
      return Math.max(-TAU_MAX, Math.min(TAU_MAX, raw));
    });
    this.tau = tau;
    const Aj = matMul(A, tm.J)[0];
    this.info = {
      tm, A: [n[0], n[1]], Ajoint: [Aj[0], Aj[1]], P, IP, Fm, Ff, Fmotion, Fforce,
      s, sd, hModel,
      intF: p.mode === 'force' ? Math.hypot(p.Kfi * this.iF2[0], p.Kfi * this.iF2[1]) * Math.sign(-dot2(this.iF2, n) || 1) : p.Kfi * this.iFn,
      intP: p.mode === 'conflict' ? GAINS.conflict.Ki * this.iPn : 0,
      pushForce, pushPos,
      saturated,
    };
    // Sample the sensors for the NEXT tick (one-sample delay).
    this.meas = { theta: this.theta.slice(), dtheta: this.dtheta.slice(), f: this.f };
  }

  /** Finite, not spinning wildly, and the tip still near the table (else the run is declared lost). */
  healthy(): boolean {
    if (![...this.theta, ...this.dtheta].every(Number.isFinite)) return false;
    if (Math.abs(this.dtheta[0]) > 60 || Math.abs(this.dtheta[1]) > 60) return false;
    const tip = planarFK(LINKS, this.theta).tip;
    const { t } = surfaceAxes(this.p.alpha);
    const s = (tip[0] - SURF_C[0]) * t[0] + (tip[1] - SURF_C[1]) * t[1];
    return Math.abs(s) < 0.75 && this.h < 0.6;
  }
}
