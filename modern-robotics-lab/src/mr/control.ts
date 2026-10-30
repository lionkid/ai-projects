/**
 * Robot control (book chapter 11): computed-torque control of an open chain,
 * a closed-loop simulator and a small discrete PID helper for 1-DOF lessons.
 */

import { type Mat, type Vec, clamp, matVec, vecAdd, vecScale, vecSub, zerosVec } from './linalg';
import {
  type Integrator,
  ForwardDynamics,
  InverseDynamics,
  MassMatrix,
  eulerStepGeneric,
  manipulatorODE,
  rk4Step,
} from './dynamics';

/** Scalar gain (applied to every joint) or one gain per joint. */
export type Gain = number | Vec;

function gainTimes(K: Gain, e: Vec): Vec {
  return typeof K === 'number' ? vecScale(e, K) : e.map((x, i) => x * K[i]);
}

/**
 * Computed-torque (feedback-linearising) control law
 *   τ = M(θ) (θdd_d + Kp e + Ki ∫e + Kd ė) + c(θ, θd) + g(θ),
 * implemented as M(θ)(Kp e + Ki eint + Kd ė) + InverseDynamics(θ, θd, θdd_d).
 * `eint` is the integral of the position error supplied by the caller
 * (see `SimulateControl`). Model parameters (`Mlist`, `Glist`, `g`) are the
 * controller's MODEL of the robot, which may differ from the true plant.
 */
export function ComputedTorque(
  thetalist: Vec,
  dthetalist: Vec,
  eint: Vec,
  g: Vec,
  Mlist: Mat[],
  Glist: Mat[],
  Slist: Mat,
  thetalistd: Vec,
  dthetalistd: Vec,
  ddthetalistd: Vec,
  Kp: Gain,
  Ki: Gain,
  Kd: Gain,
): Vec {
  const e = vecSub(thetalistd, thetalist);
  const de = vecSub(dthetalistd, dthetalist);
  const feedback = vecAdd(vecAdd(gainTimes(Kp, e), gainTimes(Ki, eint)), gainTimes(Kd, de));
  const M = MassMatrix(thetalist, Mlist, Glist, Slist);
  const feedforward = InverseDynamics(thetalist, dthetalist, ddthetalistd, g, zerosVec(6), Mlist, Glist, Slist);
  return vecAdd(matVec(M, feedback), feedforward);
}

export interface SimulateControlResult {
  /** Commanded joint torques, one row per time sample (N rows). */
  taumat: Vec[];
  /** Actual joint positions at each sample time k dt (N rows, row 0 = initial state). */
  thetamat: Vec[];
  /** Actual joint velocities at each sample time (N rows). */
  dthetamat: Vec[];
  /** Tracking error θ_d - θ at each sample (N rows). */
  errmat: Vec[];
}

/**
 * Closed-loop simulation of computed-torque control. The TRUE plant uses
 * (`g`, `Mlist`, `Glist`) and is disturbed by the tip wrenches `Ftipmat[k]`;
 * the CONTROLLER uses the possibly inaccurate model (`gtilde`, `Mtildelist`,
 * `Gtildelist`). At each of the N sample times the torque is computed from the
 * current state and the desired (`thetamatd`, `dthetamatd`, `ddthetamatd`)
 * sample, then held constant while the plant is integrated over `dt` with
 * `intRes` sub-steps. Returned rows are aligned with the desired trajectory.
 */
export function SimulateControl(
  thetalist: Vec,
  dthetalist: Vec,
  g: Vec,
  Ftipmat: Vec[],
  Mlist: Mat[],
  Glist: Mat[],
  Slist: Mat,
  thetamatd: Vec[],
  dthetamatd: Vec[],
  ddthetamatd: Vec[],
  gtilde: Vec,
  Mtildelist: Mat[],
  Gtildelist: Mat[],
  Kp: Gain,
  Ki: Gain,
  Kd: Gain,
  dt: number,
  intRes = 1,
  integrator: Integrator = 'rk4',
): SimulateControlResult {
  const n = thetalist.length;
  const N = thetamatd.length;
  let theta = thetalist.slice();
  let dtheta = dthetalist.slice();
  let eint = zerosVec(n);
  const taumat: Vec[] = [];
  const thetamat: Vec[] = [];
  const dthetamat: Vec[] = [];
  const errmat: Vec[] = [];
  const h = dt / Math.max(1, intRes);
  for (let k = 0; k < N; k++) {
    const e = vecSub(thetamatd[k], theta);
    eint = vecAdd(eint, vecScale(e, dt));
    const tau = ComputedTorque(
      theta, dtheta, eint, gtilde, Mtildelist, Gtildelist, Slist,
      thetamatd[k], dthetamatd[k], ddthetamatd[k], Kp, Ki, Kd,
    );
    taumat.push(tau);
    thetamat.push(theta.slice());
    dthetamat.push(dtheta.slice());
    errmat.push(e);
    const Ftip = Ftipmat[k] ?? zerosVec(6);
    const f = manipulatorODE(() => tau, g, () => Ftip, Mlist, Glist, Slist);
    let x = theta.concat(dtheta);
    for (let j = 0; j < intRes; j++) {
      const t = k * dt + j * h;
      x = integrator === 'rk4' ? rk4Step(f, t, x, h) : eulerStepGeneric(f, t, x, h);
    }
    theta = x.slice(0, n);
    dtheta = x.slice(n, 2 * n);
  }
  return { taumat, thetamat, dthetamat, errmat };
}

/** Convenience wrapper: joint accelerations of the true plant under a torque (for custom loops). */
export function plantAcceleration(
  theta: Vec,
  dtheta: Vec,
  tau: Vec,
  g: Vec,
  Ftip: Vec,
  Mlist: Mat[],
  Glist: Mat[],
  Slist: Mat,
): Vec {
  return ForwardDynamics(theta, dtheta, tau, g, Ftip, Mlist, Glist, Slist);
}

// ---------------------------------------------------------------------------
// Discrete PID for scalar (1-DOF) lessons
// ---------------------------------------------------------------------------

export interface PIDOptions {
  kp: number;
  ki: number;
  kd: number;
  /** Output saturation (anti-windup stops integrating when saturated in the same direction). */
  outputMin?: number;
  outputMax?: number;
  /** Clamp on |integral of error| (anti-windup by clamping). */
  integralLimit?: number;
  /**
   * Derivative on measurement (true) avoids the "derivative kick" when the
   * setpoint jumps; the caller then passes `measurement` to `pidStep`.
   * Default false: derivative of the error.
   */
  derivativeOnMeasurement?: boolean;
}

export interface PIDState {
  integral: number;
  prevError: number | null;
  prevMeasurement: number | null;
  /** Last computed (saturated) output. */
  output: number;
}

/** Fresh PID state. */
export function pidInitialState(): PIDState {
  return { integral: 0, prevError: null, prevMeasurement: null, output: 0 };
}

/**
 * One discrete PID update (pure: returns the next state). `error` is
 * setpoint - measurement; `measurement` is only needed when
 * `derivativeOnMeasurement` is on.
 */
export function pidStep(
  opts: PIDOptions,
  state: PIDState,
  error: number,
  dt: number,
  measurement = 0,
): { output: number; state: PIDState } {
  let derivative = 0;
  if (dt > 0) {
    if (opts.derivativeOnMeasurement) {
      if (state.prevMeasurement !== null) derivative = -(measurement - state.prevMeasurement) / dt;
    } else if (state.prevError !== null) {
      derivative = (error - state.prevError) / dt;
    }
  }
  let integral = state.integral + error * dt;
  if (opts.integralLimit !== undefined) integral = clamp(integral, -opts.integralLimit, opts.integralLimit);
  let output = opts.kp * error + opts.ki * integral + opts.kd * derivative;
  const lo = opts.outputMin ?? -Infinity;
  const hi = opts.outputMax ?? Infinity;
  if (output > hi || output < lo) {
    // Saturated: undo the integration if it pushes further into saturation.
    const saturated = clamp(output, lo, hi);
    if ((output > hi && error > 0) || (output < lo && error < 0)) {
      integral = state.integral;
      output = opts.kp * error + opts.ki * integral + opts.kd * derivative;
    }
    output = clamp(output, lo, hi);
    if (Number.isNaN(output)) output = saturated;
  }
  return {
    output,
    state: { integral, prevError: error, prevMeasurement: measurement, output },
  };
}

export interface PIDController {
  /** Returns the control output for the given error (and optional measurement) over step dt. */
  update(error: number, dt: number, measurement?: number): number;
  reset(): void;
  readonly state: PIDState;
  readonly options: PIDOptions;
}

/** Stateful convenience wrapper around `pidStep`. */
export function createPID(opts: PIDOptions): PIDController {
  let state = pidInitialState();
  return {
    update(error, dt, measurement = 0) {
      const r = pidStep(opts, state, error, dt, measurement);
      state = r.state;
      return r.output;
    },
    reset() {
      state = pidInitialState();
    },
    get state() {
      return state;
    },
    get options() {
      return opts;
    },
  };
}
