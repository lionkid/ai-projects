/**
 * Pure models for lesson ch11-02: a planar 2R arm (two uniform 1 m / 1 kg rods moving in a
 * vertical plane, gravity along −y) follows a back-and-forth motion under four controllers:
 *
 *   'pd'   independent joint PD(+I)      τ = M̄ (Kp e + Ki ∫e + Kd ė)
 *   'pdg'  PD + model gravity            τ = M̄ (Kp e + Ki ∫e + Kd ė) + g̃(θ)
 *   'ct'   joint-space computed torque   τ = M̃(θ)(θ̈d + Kp e + Ki ∫e + Kd ė) + c̃(θ, θ̇) + g̃(θ)
 *   'task' task-space computed torque    τ = M̃(θ) J#(θ)(ẍd + Kp xe + Ki ∫xe + Kd ẋe − J̇θ̇) + c̃ + g̃
 *
 * Gains are designed at the acceleration level from (ωn, ζ): Kp = ωn², Kd = 2ζωn, Ki = 0.5 ωn³.
 * The PD-type controllers multiply them by M̄ = diag(M̃(θ_ref)), a constant nominal inertia per
 * joint, so all four controllers aim at the same closed-loop bandwidth and can be compared fairly.
 * J# is the damped-least-squares inverse Jᵀ(JJᵀ + λ²I)⁻¹ (λ = 0 gives J⁻¹).
 *
 * The plant always uses the true masses; the controller's model (M̃, c̃, g̃) uses masses scaled by
 * (1 + δ). Dynamics: closed-form 2R model of `@/mr` (`planar2RDynamics`, `planar2RAccel`), which
 * the library tests against Newton–Euler; kinematics: `planarFK`, `planarTipJacobian`; plant
 * integration: `rk4Step` with the torque held constant over each control period (ZOH).
 */
import {
  planar2RParams, planar2RDynamics, planar2RAccel, planarFK, planarTipJacobian, planar2RIK,
  QuinticTimeScalingDerivs, rk4Step, type Planar2RParams, type Vec,
} from '@/mr';

export type ControllerKind = 'pd' | 'pdg' | 'ct' | 'task';
export type RouteKind = 'ab' | 'cd';

export const LINKS: [number, number] = [1, 1];
export const MASSES: [number, number] = [1, 1];
export const GRAV = 9.81;
/** Duration of each rest phase (s). */
export const HOLD = 0.8;
/** Ki = KI_FACTOR · ωn³ when the integral term is on (stable for ζ > KI_FACTOR / 2). */
export const KI_FACTOR = 0.5;

/** Arm parameters with every mass (and therefore every inertia) multiplied by `massScale`. */
export function armParams(massScale = 1): Planar2RParams {
  return planar2RParams(LINKS, [MASSES[0] * massScale, MASSES[1] * massScale], 'rod', GRAV);
}

export interface Route {
  /** Joint configurations at the two ends (elbow θ2 > 0 branch). */
  qa: Vec;
  qb: Vec;
  /** Tip positions at the two ends. */
  pa: Vec;
  pb: Vec;
}

function routeFromJoints(qa: Vec, qb: Vec): Route {
  return { qa, qb, pa: planarFK(LINKS, qa).tip, pb: planarFK(LINKS, qb).tip };
}

function ikElbow(p: Vec): Vec {
  const ik = planar2RIK(LINKS[0], LINKS[1], p[0], p[1]);
  const sol = ik.solutions.find((s) => s.theta2 >= 0) ?? ik.solutions[0];
  if (!sol) throw new Error(`armControl: (${p[0]}, ${p[1]}) is out of reach`);
  return [sol.theta1, sol.theta2];
}

function routeFromTips(pa: Vec, pb: Vec): Route {
  return { qa: ikElbow(pa), qb: ikElbow(pb), pa, pb };
}

export const ROUTES: Record<RouteKind, Route> = {
  /** Joint 1 swings 90° while joint 2 is asked to stay at 90°: any joint-2 error is coupling. */
  ab: routeFromJoints([-Math.PI / 4, Math.PI / 2], [Math.PI / 4, Math.PI / 2]),
  /** End points on a horizontal line passing 0.15 m above the base (near the folded singularity). */
  cd: routeFromTips([-1, 0.15], [1, 0.15]),
};

/** Phase s(t) ∈ [0, 1]: rest, quintic a→b in Tf, rest, quintic b→a in Tf, rest. */
export function phase(t: number, Tf: number, hold = HOLD): { s: number; sd: number; sdd: number } {
  if (t < hold) return { s: 0, sd: 0, sdd: 0 };
  if (t < hold + Tf) { const q = QuinticTimeScalingDerivs(Tf, t - hold); return { s: q.s, sd: q.sdot, sdd: q.sddot }; }
  if (t < 2 * hold + Tf) return { s: 1, sd: 0, sdd: 0 };
  if (t < 2 * hold + 2 * Tf) { const q = QuinticTimeScalingDerivs(Tf, t - 2 * hold - Tf); return { s: 1 - q.s, sd: -q.sdot, sdd: -q.sddot }; }
  return { s: 0, sd: 0, sdd: 0 };
}

export const trialDuration = (Tf: number, hold = HOLD): number => 3 * hold + 2 * Tf;

/** Straight-line interpolation a + s (b − a) with its first two time derivatives. */
function lerpRef(a: Vec, b: Vec, ph: { s: number; sd: number; sdd: number }): { p: Vec; v: Vec; a: Vec } {
  const d = [b[0] - a[0], b[1] - a[1]];
  return {
    p: [a[0] + ph.s * d[0], a[1] + ph.s * d[1]],
    v: [ph.sd * d[0], ph.sd * d[1]],
    a: [ph.sdd * d[0], ph.sdd * d[1]],
  };
}

/** J̇(θ, θ̇) θ̇ for the tip of a planar 2R arm (the velocity-product part of the tip acceleration). */
export function jdotQdot(L: [number, number], th: Vec, dth: Vec): Vec {
  const c1 = Math.cos(th[0]), s1 = Math.sin(th[0]);
  const c12 = Math.cos(th[0] + th[1]), s12 = Math.sin(th[0] + th[1]);
  const w1 = dth[0], w12 = dth[0] + dth[1];
  return [-L[0] * c1 * w1 * w1 - L[1] * c12 * w12 * w12, -L[0] * s1 * w1 * w1 - L[1] * s12 * w12 * w12];
}

/**
 * Damped-least-squares solution Jᵀ(JJᵀ + λ²I)⁻¹ v of J q = v for a 2×2 J. A tiny floor on λ²
 * keeps the exactly singular case finite; for λ = 0 and |det J| ≫ 1e-5 this is J⁻¹ v.
 */
export function dlsSolve(J: number[][], v: Vec, lambda: number): Vec {
  const l2 = lambda * lambda + 1e-10;
  const a = J[0][0] * J[0][0] + J[0][1] * J[0][1] + l2;
  const b = J[0][0] * J[1][0] + J[0][1] * J[1][1];
  const d = J[1][0] * J[1][0] + J[1][1] * J[1][1] + l2;
  const det = a * d - b * b;
  const y0 = (d * v[0] - b * v[1]) / det;
  const y1 = (-b * v[0] + a * v[1]) / det;
  return [J[0][0] * y0 + J[1][0] * y1, J[0][1] * y0 + J[1][1] * y1];
}

export interface Gains { kp: number; kd: number; ki: number }

/** Acceleration-level gains for the error equation ë + Kd ė + Kp e (+ Ki ∫e) = 0. */
export function designGains(wn: number, zeta: number, useKi: boolean): Gains {
  return { kp: wn * wn, kd: 2 * zeta * wn, ki: useKi ? KI_FACTOR * wn * wn * wn : 0 };
}

export interface ControlOutput {
  tau: Vec;
  /** Model-based part (M̃θ̈d + c̃ + g̃ for computed torque, g̃ for PD+g, 0 for PD). */
  ff: Vec;
  /** Error-driven part. */
  fb: Vec;
}

const mv = (M: number[][], v: Vec): Vec => [M[0][0] * v[0] + M[0][1] * v[1], M[1][0] * v[0] + M[1][1] * v[1]];
const add = (a: Vec, b: Vec): Vec => [a[0] + b[0], a[1] + b[1]];

/** Joint-space controllers. `e = θd − θ`, `eint = ∫e`. */
export function jointControl(
  kind: 'pd' | 'pdg' | 'ct', model: Planar2RParams, Mbar: Vec, g: Gains,
  th: Vec, dth: Vec, thd: Vec, dthd: Vec, ddthd: Vec, eint: Vec,
): ControlOutput {
  const e = [thd[0] - th[0], thd[1] - th[1]];
  const ed = [dthd[0] - dth[0], dthd[1] - dth[1]];
  const v = [g.kp * e[0] + g.ki * eint[0] + g.kd * ed[0], g.kp * e[1] + g.ki * eint[1] + g.kd * ed[1]];
  if (kind === 'ct') {
    const dyn = planar2RDynamics(model, th, dth);
    const fb = mv(dyn.M, v);
    const ff = add(add(mv(dyn.M, ddthd), dyn.c), dyn.g);
    return { tau: add(ff, fb), ff, fb };
  }
  const fb = [Mbar[0] * v[0], Mbar[1] * v[1]];
  const ff = kind === 'pdg' ? planar2RDynamics(model, th, [0, 0]).g : [0, 0];
  return { tau: add(ff, fb), ff, fb };
}

/** Task-space computed torque on the tip position. `eint = ∫(xd − x)`. */
export function taskControl(
  model: Planar2RParams, g: Gains, lambda: number,
  th: Vec, dth: Vec, xd: Vec, dxd: Vec, ddxd: Vec, eint: Vec,
): ControlOutput {
  const x = planarFK(LINKS, th).tip;
  const J = planarTipJacobian(LINKS, th);
  const xdot = mv(J, dth);
  const e = [xd[0] - x[0], xd[1] - x[1]];
  const ed = [dxd[0] - xdot[0], dxd[1] - xdot[1]];
  const bias = jdotQdot(LINKS, th, dth);
  const qFF = dlsSolve(J, [ddxd[0] - bias[0], ddxd[1] - bias[1]], lambda);
  const qFB = dlsSolve(J, [g.kp * e[0] + g.ki * eint[0] + g.kd * ed[0], g.kp * e[1] + g.ki * eint[1] + g.kd * ed[1]], lambda);
  const dyn = planar2RDynamics(model, th, dth);
  const ff = add(add(mv(dyn.M, qFF), dyn.c), dyn.g);
  const fb = mv(dyn.M, qFB);
  return { tau: add(ff, fb), ff, fb };
}

export interface ArmTrialOptions {
  kind: ControllerKind;
  route: RouteKind;
  /** Duration of each move (s). */
  Tf: number;
  wn: number;
  zeta: number;
  useKi: boolean;
  /** Model mass error δ: the controller believes every mass is (1 + δ) times the true one. */
  massError: number;
  /** DLS damping λ for the task-space controller. */
  lambda?: number;
  /** Initial joint offset θ(0) − θd(0) (rad). */
  offset?: Vec;
  hold?: number;
  /** Control period = integration step (s). Default 2 ms. */
  h?: number;
  /** Keep every `every`-th sample. Default 5 (100 Hz). */
  every?: number;
}

export interface ArmStats {
  /** RMS / max distance between the tip and its reference (m). */
  tipRms: number;
  tipMax: number;
  /** Largest joint speed during the trial (rad/s). */
  maxRate: number;
  /** RMS(τ_fb) / RMS(τ): how much of the effort comes from the error feedback. */
  fbShare: number;
  /** Largest |τ_i| (N·m). */
  peakTau: number;
  /** Error at the end of the trial (rad in joint space, m in task space). */
  finalErr: Vec;
}

export interface ArmTrial {
  t: number[];
  th: Vec[];
  dth: Vec[];
  /** Joint reference (joint-space controllers) or null (task space). */
  thd: Vec[] | null;
  tip: Vec[];
  tipd: Vec[];
  /** θd − θ (rad) for joint-space controllers, xd − x (m) for the task-space one. */
  err: Vec[];
  tau: Vec[];
  ff: Vec[];
  fb: Vec[];
  /** det J at each sample (sign shows the elbow branch; |det J| → 0 at singularities). */
  detJ: number[];
  T: number;
  diverged: boolean;
  stats: ArmStats;
}

/** One closed-loop trial (see the module comment). */
export function simulateArm(o: ArmTrialOptions): ArmTrial {
  const h = o.h ?? 2e-3;
  const every = Math.max(1, Math.round(o.every ?? 5));
  const hold = o.hold ?? HOLD;
  const T = trialDuration(o.Tf, hold);
  const route = ROUTES[o.route];
  const truth = armParams(1);
  const model = armParams(1 + o.massError);
  const g = designGains(o.wn, o.zeta, o.useKi);
  const qMid = [(route.qa[0] + route.qb[0]) / 2, (route.qa[1] + route.qb[1]) / 2];
  const Mmid = planar2RDynamics(model, qMid, [0, 0]).M;
  const Mbar = [Mmid[0][0], Mmid[1][1]];
  const task = o.kind === 'task';
  const lambda = o.lambda ?? 0;

  const out: ArmTrial = {
    t: [], th: [], dth: [], thd: task ? null : [], tip: [], tipd: [], err: [], tau: [], ff: [], fb: [], detJ: [],
    T, diverged: false, stats: { tipRms: 0, tipMax: 0, maxRate: 0, fbShare: 0, peakTau: 0, finalErr: [0, 0] },
  };
  const off = o.offset ?? [0, 0];
  let x = [route.qa[0] + off[0], route.qa[1] + off[1], 0, 0];
  let eint = [0, 0];
  let sumTip2 = 0, sumFb2 = 0, sumTau2 = 0, nS = 0;
  const N = Math.round(T / h);
  for (let k = 0; k <= N; k++) {
    const t = k * h;
    const th = [x[0], x[1]], dth = [x[2], x[3]];
    const ph = phase(t, o.Tf, hold);
    let ctl: ControlOutput, e: Vec, tipd: Vec, thdNow: Vec | null = null;
    if (task) {
      const r = lerpRef(route.pa, route.pb, ph);
      const tipNow = planarFK(LINKS, th).tip;
      e = [r.p[0] - tipNow[0], r.p[1] - tipNow[1]];
      eint = [eint[0] + e[0] * h, eint[1] + e[1] * h];
      ctl = taskControl(model, g, lambda, th, dth, r.p, r.v, r.a, eint);
      tipd = r.p;
    } else {
      const r = lerpRef(route.qa, route.qb, ph);
      e = [r.p[0] - th[0], r.p[1] - th[1]];
      eint = [eint[0] + e[0] * h, eint[1] + e[1] * h];
      ctl = jointControl(o.kind as 'pd' | 'pdg' | 'ct', model, Mbar, g, th, dth, r.p, r.v, r.a, eint);
      tipd = planarFK(LINKS, r.p).tip;
      thdNow = r.p;
    }
    if (k % every === 0 || k === N) {
      const tip = planarFK(LINKS, th).tip;
      const J = planarTipJacobian(LINKS, th);
      out.t.push(t); out.th.push(th); out.dth.push(dth); out.tip.push(tip); out.tipd.push(tipd); out.err.push(e);
      out.tau.push(ctl.tau); out.ff.push(ctl.ff); out.fb.push(ctl.fb);
      out.detJ.push(J[0][0] * J[1][1] - J[0][1] * J[1][0]);
      if (out.thd && thdNow) out.thd.push(thdNow);
      const dTip2 = (tip[0] - tipd[0]) ** 2 + (tip[1] - tipd[1]) ** 2;
      sumTip2 += dTip2; nS++;
      out.stats.tipMax = Math.max(out.stats.tipMax, Math.sqrt(dTip2));
      out.stats.maxRate = Math.max(out.stats.maxRate, Math.abs(dth[0]), Math.abs(dth[1]));
      out.stats.peakTau = Math.max(out.stats.peakTau, Math.abs(ctl.tau[0]), Math.abs(ctl.tau[1]));
      sumFb2 += ctl.fb[0] ** 2 + ctl.fb[1] ** 2;
      sumTau2 += ctl.tau[0] ** 2 + ctl.tau[1] ** 2;
      out.stats.finalErr = e;
    }
    if (k === N) break;
    const tau = ctl.tau;
    const f = (_t: number, s: number[]) => {
      const acc = planar2RAccel(truth, [s[0], s[1]], [s[2], s[3]], tau);
      return [s[2], s[3], acc[0], acc[1]];
    };
    x = rk4Step(f, t, x, h);
    if (!x.every(Number.isFinite) || Math.abs(x[2]) > 300 || Math.abs(x[3]) > 300) { out.diverged = true; break; }
  }
  out.stats.tipRms = nS ? Math.sqrt(sumTip2 / nS) : 0;
  out.stats.fbShare = sumTau2 > 0 ? Math.sqrt(sumFb2 / sumTau2) : 0;
  return out;
}

/** Free response of ë + 2ζωn ė + ωn² e = 0 from e(0) = e0, ė(0) = 0 (the ideal computed-torque error). */
export function secondOrderFree(e0: number, wn: number, zeta: number, t: number): number {
  if (Math.abs(zeta - 1) < 1e-6) return e0 * (1 + wn * t) * Math.exp(-wn * t);
  if (zeta < 1) {
    const wd = wn * Math.sqrt(1 - zeta * zeta);
    return e0 * Math.exp(-zeta * wn * t) * (Math.cos(wd * t) + ((zeta * wn) / wd) * Math.sin(wd * t));
  }
  const r = wn * Math.sqrt(zeta * zeta - 1);
  const s1 = -zeta * wn + r, s2 = -zeta * wn - r;
  // e = A e^{s1 t} + B e^{s2 t}, A + B = e0, s1 A + s2 B = 0
  const A = (-s2 * e0) / (s1 - s2), B = (s1 * e0) / (s1 - s2);
  return A * Math.exp(s1 * t) + B * Math.exp(s2 * t);
}
