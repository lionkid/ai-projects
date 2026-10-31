/**
 * Planar 2R arm rendering a virtual mass–spring–damper at its tip, near a stiff wall.
 *
 * Plant
 *  - The lab's planar 2R arm (`@/mr` closed-form dynamics, L = 1, 1 m, 1 kg point masses,
 *    gravity along −y) plus per-joint rotor inertia and joint friction (viscous + Coulomb with
 *    true stiction: a joint stays at rest while the torque on it is below the Coulomb level).
 *    Two hardware presets: a direct-drive (backdrivable) arm and a highly geared one (large
 *    reflected inertia and friction).
 *  - A vertical wall at x = x_w (occupying x > x_w): unilateral spring–damper, never pulls.
 *  - External forces on the tip: wall + the user's "rubber band" + an optional push pulse.
 *  - Semi-implicit Euler, 0.2 ms sub-steps.
 *
 * Controllers (1 kHz, measurements one sample old, τ held between samples)
 *  - impedance: measure motion, output force:  τ = g̃(θ) + Jᵀ(K(x₀ − x) − B ẋ).
 *    The rendered mass is the arm's own Λ(θ) (the ẍ terms cancel when M = Λ̃).
 *  - admittance: measure force, output motion. The (delayed) wrist force drives the virtual
 *    model M ẍ_c + B ẋ_c + K(x_c − x₀) = f_ext, integrated at 1 kHz; a stiff inner position
 *    loop (operational-space PD) makes the tip follow x_c.
 *
 * Sign convention: f_ext is the force the ENVIRONMENT (wall, user) applies to the robot tip,
 * which is also what the wrist sensor reports. The wall pushes the tip towards −x.
 */
import {
  planar2RParams, planar2RDynamics, planar2RIK, planarFK, planarTipJacobian,
  matMul, matVec, matAdd, matScale, transpose, inv, eye, diag,
  type Mat, type Vec,
} from '@/mr';

export type Vec2 = [number, number];
export type Mode = 'impedance' | 'admittance';
export type ArmKind = 'direct' | 'geared';

export const LINKS = [1, 1];
export const ARM = planar2RParams([1, 1], [1, 1], 'point-mass');
export const TC = 1e-3;
export const H = 2e-4;
export const TAU_MAX = 150;
/** Maximum sensor delay (ms) the ring buffer supports. */
export const MAX_DELAY_MS = 60;

export interface ArmSpec { rotor: number; coulomb: number; viscous: number }
export const ARM_KINDS: Record<ArmKind, ArmSpec> = {
  /** Backdrivable direct-drive arm: tiny rotor inertia and friction. */
  direct: { rotor: 0.02, coulomb: 0.3, viscous: 0.05 },
  /** Highly geared arm: reflected rotor inertia I_m N² and gear friction dominate. */
  geared: { rotor: 1.0, coulomb: 6, viscous: 1.5 },
};

/**
 * Inner position loop of the admittance controller (1/s², 1/s): a stiff servo whose derivative
 * acts on the measured velocity only (no velocity feed-forward), like a typical position drive.
 */
export const INNER_W = 200;
/** PD gains (1/s², 1/s) of the inner loop for natural frequency w (rad/s), ζ ≈ 0.8. */
export function innerGains(w: number): { Kp: number; Kd: number } { return { Kp: w * w, Kd: 1.6 * w }; }

export interface ImpParams {
  mode: Mode;
  arm: ArmKind;
  /** Virtual mass (kg, admittance only), damping (N·s/m) and stiffness (N/m). */
  M: number;
  B: number;
  K: number;
  /** Wall stiffness (N/m), position (m) and presence. */
  kw: number;
  wallX: number;
  wallOn: boolean;
  /** Extra force-sensor delay (ms) on top of the one-sample controller delay. */
  delayMs: number;
  /** Inner position-loop natural frequency (rad/s). */
  innerW?: number;
}

export const DEFAULT_PARAMS: ImpParams = {
  mode: 'impedance', arm: 'direct', M: 2, B: 30, K: 300, kw: 5e4, wallX: 1.5, wallOn: true, delayMs: 0,
};

export const HOME: Vec2 = [1.2, 0.25];

export function wallDamping(kw: number): number { return 0.2 * Math.sqrt(kw); }

/** Keep a target inside the reachable annulus (away from the singular boundary). */
export function clampTarget(p: Vec2): Vec2 {
  const r = Math.hypot(p[0], p[1]);
  if (r >= 0.45 && r <= 1.85) return [p[0], p[1]];
  const rc = r > 1.85 ? 1.85 : 0.45;
  return r > 1e-9 ? [(p[0] * rc) / r, (p[1] * rc) / r] : [rc, 0];
}

export function elbowUpIK(p: Vec2): Vec {
  const ik = planar2RIK(1, 1, p[0], p[1]);
  const sol = ik.solutions.find((s) => s.elbow === 'up') ?? ik.solutions[0];
  return sol ? [sol.theta1, sol.theta2] : [1.1, -1.8];
}

/**
 * Coulomb friction with stiction for one time step of a 2-joint system.
 * Given the joint velocities (v0, v1) the step would reach without Coulomb friction and the
 * inverse mass matrix W = M⁻¹ (symmetric, W00, W01, W11), find friction impulses p with
 * |p_i| ≤ pMax that bring the velocities as close to zero as the bound allows: minimise the
 * kinetic energy after the step ½ uᵀ M u with u = v + W p (maximal dissipation). Inside the
 * bound a joint ends exactly at rest (sticks); at the bound it slides with full Coulomb friction
 * opposing its motion. Projected Gauss–Seidel finds which joints stick (W is SPD, two unknowns;
 * each sweep shrinks the error by about W01²/(W00 W11) ≤ 0.65 for this arm), then the reduced
 * linear system is solved exactly so that a sticking joint ends exactly at rest.
 */
export function coulombStep(v0: number, v1: number, W00: number, W01: number, W11: number, pMax: number): [number, number] {
  if (pMax <= 0) return [v0, v1];
  const clampP = (x: number) => Math.max(-pMax, Math.min(pMax, x));
  let p0 = 0, p1 = 0;
  for (let it = 0; it < 20; it++) {
    p0 = clampP(p0 - (v0 + W00 * p0 + W01 * p1) / W00);
    p1 = clampP(p1 - (v1 + W01 * p0 + W11 * p1) / W11);
  }
  const stick0 = Math.abs(p0) < pMax, stick1 = Math.abs(p1) < pMax;
  if (stick0 && stick1) {
    // Both joints stick: W p = −v.
    const det = W00 * W11 - W01 * W01;
    const q0 = -(W11 * v0 - W01 * v1) / det, q1 = -(-W01 * v0 + W00 * v1) / det;
    if (Math.abs(q0) <= pMax && Math.abs(q1) <= pMax) return [0, 0];
  } else if (stick0) {
    const q0 = -(v0 + W01 * p1) / W00;
    if (Math.abs(q0) <= pMax) return [0, v1 + W01 * q0 + W11 * p1];
  } else if (stick1) {
    const q1 = -(v1 + W01 * p0) / W11;
    if (Math.abs(q1) <= pMax) return [v0 + W00 * p0 + W01 * q1, 0];
  }
  return [v0 + W00 * p0 + W01 * p1, v1 + W01 * p0 + W11 * p1];
}

export class ImpedanceSim {
  p: ImpParams;
  t = 0;
  theta: Vec = [0, 0];
  dtheta: Vec = [0, 0];
  tau: Vec = [0, 0];
  /** Equilibrium / target point x₀ of the virtual spring. */
  x0: Vec2 = [...HOME];
  /** Admittance virtual state (also kept in sync in impedance mode for display). */
  xc: Vec2 = [...HOME];
  vc: Vec2 = [0, 0];
  /** Force the user applies through the rubber band (set by the UI every frame). */
  userForce: Vec2 = [0, 0];
  /** Tip state and contact information, updated every sub-step. */
  x: Vec2 = [...HOME];
  v: Vec2 = [0, 0];
  fWall = 0;
  fExt: Vec2 = [0, 0];
  /** Delayed sensor reading used by the controller, and the controller's task-space force. */
  fMeas: Vec2 = [0, 0];
  Fcmd: Vec2 = [0, 0];
  saturated = false;
  /** Λ̃ of the last controller sample (2×2). */
  Lambda: Mat = eye(2);
  private pulse: { f: Vec2; until: number } | null = null;
  private ring: Vec2[] = [];
  private ringIdx = 0;
  private nextTick = 0;
  private acc = 0;
  private meas = { theta: [0, 0] as Vec, dtheta: [0, 0] as Vec };

  constructor(p: Partial<ImpParams> = {}) {
    this.p = { ...DEFAULT_PARAMS, ...p };
    this.reset();
  }

  /** Place the tip at `at` (default x₀) at rest; clears the admittance state and sensor history. */
  reset(at?: Vec2): void {
    const p0 = clampTarget(at ?? this.x0);
    this.theta = elbowUpIK(p0);
    this.dtheta = [0, 0];
    this.t = 0; this.nextTick = 0; this.acc = 0;
    this.xc = [...p0]; this.vc = [0, 0];
    this.pulse = null;
    this.ring = Array.from({ length: MAX_DELAY_MS + 2 }, () => [0, 0] as Vec2);
    this.ringIdx = 0;
    this.meas = { theta: this.theta.slice(), dtheta: [0, 0] };
    this.tau = planar2RDynamics(ARM, this.theta, [0, 0]).g.slice();
    this.updateTip();
  }

  /** Apply a constant push f (N) to the tip for `duration` seconds. */
  push(f: Vec2, duration = 0.15): void { this.pulse = { f, until: this.t + duration }; }

  /** Resynchronise the virtual admittance state with the real tip (e.g. after a mode switch). */
  syncVirtual(): void { this.xc = [...this.x]; this.vc = [...this.v]; }

  advance(dt: number): void {
    this.acc += dt;
    while (this.acc >= H - 1e-12) {
      this.step();
      this.acc -= H;
    }
  }

  private externalForce(x: Vec2, v: Vec2): { fw: number; f: Vec2 } {
    let fw = 0;
    if (this.p.wallOn && x[0] > this.p.wallX) {
      fw = Math.max(0, this.p.kw * (x[0] - this.p.wallX) + wallDamping(this.p.kw) * v[0]);
    }
    const f: Vec2 = [this.userForce[0] - fw, this.userForce[1]];
    if (this.pulse && this.t < this.pulse.until) { f[0] += this.pulse.f[0]; f[1] += this.pulse.f[1]; }
    return { fw, f };
  }

  private updateTip(): void {
    const fk = planarFK(LINKS, this.theta);
    const J = planarTipJacobian(LINKS, this.theta);
    this.x = [fk.tip[0], fk.tip[1]];
    this.v = matVec(J, this.dtheta) as Vec2;
    const e = this.externalForce(this.x, this.v);
    this.fWall = e.fw; this.fExt = e.f;
  }

  step(): void {
    if (this.t >= this.nextTick - 1e-9) {
      this.controlTick();
      this.nextTick += TC;
    }
    const spec = ARM_KINDS[this.p.arm];
    const J = planarTipJacobian(LINKS, this.theta);
    const fk = planarFK(LINKS, this.theta);
    this.x = [fk.tip[0], fk.tip[1]];
    this.v = matVec(J, this.dtheta) as Vec2;
    const e = this.externalForce(this.x, this.v);
    this.fWall = e.fw; this.fExt = e.f;
    const tauExt = [J[0][0] * e.f[0] + J[1][0] * e.f[1], J[0][1] * e.f[0] + J[1][1] * e.f[1]];
    const dyn = planar2RDynamics(ARM, this.theta, this.dtheta);
    const b0 = this.tau[0] + tauExt[0] - dyn.c[0] - dyn.g[0] - spec.viscous * this.dtheta[0];
    const b1 = this.tau[1] + tauExt[1] - dyn.c[1] - dyn.g[1] - spec.viscous * this.dtheta[1];
    const M00 = dyn.M[0][0] + spec.rotor, M01 = dyn.M[0][1], M11 = dyn.M[1][1] + spec.rotor;
    const det = M00 * M11 - M01 * M01;
    // W = M⁻¹ (with rotor inertia); velocity update without Coulomb friction (semi-implicit Euler).
    const W00 = M11 / det, W01 = -M01 / det, W11 = M00 / det;
    const v0 = this.dtheta[0] + (W00 * b0 + W01 * b1) * H;
    const v1 = this.dtheta[1] + (W01 * b0 + W11 * b1) * H;
    const [u0, u1] = coulombStep(v0, v1, W00, W01, W11, spec.coulomb * H);
    this.dtheta = [u0, u1];
    this.theta = [this.theta[0] + u0 * H, this.theta[1] + u1 * H];
    this.t += H;
  }

  private controlTick(): void {
    const p = this.p;
    const spec = ARM_KINDS[p.arm];
    // Wrist sensor: record the current external force, read the one from `delay` samples ago.
    this.ring[this.ringIdx] = [this.fExt[0], this.fExt[1]];
    const d = Math.max(0, Math.min(MAX_DELAY_MS, Math.round(p.delayMs))) + 1; // +1: controller delay
    const len = this.ring.length;
    this.fMeas = this.ring[(this.ringIdx - d + len * 2) % len];
    this.ringIdx = (this.ringIdx + 1) % len;

    const th = this.meas.theta, dth = this.meas.dtheta;
    const fk = planarFK(LINKS, th);
    const x: Vec2 = [fk.tip[0], fk.tip[1]];
    const J = planarTipJacobian(LINKS, th);
    const v = matVec(J, dth) as Vec2;
    const dyn = planar2RDynamics(ARM, th, dth);
    const Mfull = matAdd(dyn.M, diag([spec.rotor, spec.rotor]));
    const Linv = matMul(matMul(J, inv(Mfull)), transpose(J));
    const L = inv(matAdd(Linv, matScale(eye(2), 1e-4)));
    this.Lambda = L;

    let F: Vec2;
    let tauExtra: Vec = [0, 0];
    if (p.mode === 'impedance') {
      F = [p.K * (this.x0[0] - x[0]) - p.B * v[0], p.K * (this.x0[1] - x[1]) - p.B * v[1]];
      // Keep the virtual state glued to the real tip so a switch to admittance is bumpless.
      this.xc = [...x]; this.vc = [...v];
    } else {
      // Virtual dynamics driven by the measured force (semi-implicit Euler at the control rate).
      const M = Math.max(1e-3, p.M);
      for (let k = 0; k < 2; k++) {
        const acc = (this.fMeas[k] - p.B * this.vc[k] - p.K * (this.xc[k] - this.x0[k])) / M;
        this.vc[k] += acc * TC;
        this.xc[k] += this.vc[k] * TC;
      }
      const cl = clampTarget(this.xc);
      if (cl[0] !== this.xc[0] || cl[1] !== this.xc[1]) { this.xc = cl; this.vc = [0, 0]; }
      // Stiff inner loop (operational-space PD, derivative on measurement); cancels Coriolis terms.
      const G = innerGains(p.innerW ?? INNER_W);
      const a: Vec2 = [G.Kp * (this.xc[0] - x[0]) - G.Kd * v[0], G.Kp * (this.xc[1] - x[1]) - G.Kd * v[1]];
      F = matVec(L, a) as Vec2;
      tauExtra = dyn.c;
    }
    this.Fcmd = F;
    const tf = matVec(transpose(J), F);
    let sat = false;
    this.tau = [0, 1].map((i) => {
      const raw = dyn.g[i] + tauExtra[i] + tf[i];
      if (Math.abs(raw) > TAU_MAX) sat = true;
      return Math.max(-TAU_MAX, Math.min(TAU_MAX, raw));
    });
    this.saturated = sat;
    this.meas = { theta: this.theta.slice(), dtheta: this.dtheta.slice() };
  }

  /** Kinetic energy of the arm (incl. rotors) + energy stored in the virtual spring and the wall. */
  energy(): number {
    const spec = ARM_KINDS[this.p.arm];
    const dyn = planar2RDynamics(ARM, this.theta, this.dtheta);
    const Mf = matAdd(dyn.M, diag([spec.rotor, spec.rotor]));
    const q = this.dtheta;
    const ke = 0.5 * (q[0] * (Mf[0][0] * q[0] + Mf[0][1] * q[1]) + q[1] * (Mf[1][0] * q[0] + Mf[1][1] * q[1]));
    const ref = this.p.mode === 'admittance' ? this.xc : this.x;
    const dx = ref[0] - this.x0[0], dy = ref[1] - this.x0[1];
    const pen = this.p.wallOn ? Math.max(0, this.x[0] - this.p.wallX) : 0;
    return ke + 0.5 * this.p.K * (dx * dx + dy * dy) + 0.5 * this.p.kw * pen * pen;
  }

  healthy(): boolean {
    if (![...this.theta, ...this.dtheta].every(Number.isFinite)) return false;
    return Math.abs(this.dtheta[0]) < 60 && Math.abs(this.dtheta[1]) < 60;
  }
}
