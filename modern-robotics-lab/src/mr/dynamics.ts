/**
 * Dynamics of open chains via the recursive Newton-Euler algorithm (book
 * chapter 8), plus simple integrators.
 *
 * Conventions (identical to the book)
 *  - `Mlist[i]` (i = 0..n) is M_{i,i+1}: the configuration of link frame {i+1}
 *    in link frame {i} at the home position; {0} is the base, {n+1} is the
 *    end-effector frame, so Mlist has n + 1 entries. Link frames sit at the
 *    links' centres of mass.
 *  - `Glist[i]` (i = 0..n-1) is the 6x6 spatial inertia of link i+1 expressed
 *    in its own frame: [[I_b, 0], [0, m I]].
 *  - `Slist` columns are the joint screw axes in the space frame.
 *  - `g` is the gravity acceleration vector in the space frame, e.g.
 *    [0, 0, -9.81]; `Ftip` is the wrench [m; f] applied BY the end-effector to
 *    the environment, expressed in the end-effector frame.
 *  - Units: m, kg, s, rad, N, N m.
 */

import {
  type Mat,
  type Vec,
  eye,
  getCol,
  matMul,
  matVec,
  solve,
  transpose,
  vecAdd,
  vecScale,
  vecSub,
  zeros,
  zerosVec,
} from './linalg';
import { Adjoint, MatrixExp6, TransInv, VecTose3, ad } from './se3';

/**
 * Joint torques/forces required to produce `ddthetalist` at state
 * (`thetalist`, `dthetalist`) with gravity `g` and tip wrench `Ftip`
 * (recursive Newton-Euler: forward pass for twists and accelerations,
 * backward pass for wrenches).
 */
export function InverseDynamics(
  thetalist: Vec,
  dthetalist: Vec,
  ddthetalist: Vec,
  g: Vec,
  Ftip: Vec,
  Mlist: Mat[],
  Glist: Mat[],
  Slist: Mat,
): Vec {
  const n = thetalist.length;
  if (Mlist.length !== n + 1) throw new Error(`InverseDynamics: Mlist needs ${n + 1} entries, got ${Mlist.length}`);
  if (Glist.length !== n) throw new Error(`InverseDynamics: Glist needs ${n} entries, got ${Glist.length}`);
  let Mi = eye(4); // M_{0,i}: home pose of link frame i in the base frame
  const Ai: Vec[] = []; // joint screw axes in their own link frames
  const AdTi: Mat[] = new Array(n + 1);
  const Vi: Vec[] = [zerosVec(6)];
  const Vdi: Vec[] = [[0, 0, 0, -g[0], -g[1], -g[2]]]; // base "accelerates" upward against gravity
  AdTi[n] = Adjoint(TransInv(Mlist[n]));
  for (let i = 0; i < n; i++) {
    Mi = matMul(Mi, Mlist[i]);
    const A = matVec(Adjoint(TransInv(Mi)), getCol(Slist, i));
    Ai.push(A);
    // T_{i, i-1}(θ) = exp(-[A_i] θ_i) M_{i,i-1}
    const Ti = matMul(MatrixExp6(VecTose3(vecScale(A, -thetalist[i]))), TransInv(Mlist[i]));
    AdTi[i] = Adjoint(Ti);
    const V = vecAdd(matVec(AdTi[i], Vi[i]), vecScale(A, dthetalist[i]));
    Vi.push(V);
    const Vd = vecAdd(
      vecAdd(matVec(AdTi[i], Vdi[i]), vecScale(A, ddthetalist[i])),
      matVec(ad(V), vecScale(A, dthetalist[i])),
    );
    Vdi.push(Vd);
  }
  let Fi = Ftip.slice();
  const tau = zerosVec(n);
  for (let i = n - 1; i >= 0; i--) {
    const G = Glist[i];
    const V = Vi[i + 1];
    const GV = matVec(G, V);
    Fi = vecSub(
      vecAdd(matVec(transpose(AdTi[i + 1]), Fi), matVec(G, Vdi[i + 1])),
      matVec(transpose(ad(V)), GV),
    );
    let s = 0;
    for (let k = 0; k < 6; k++) s += Fi[k] * Ai[i][k];
    tau[i] = s;
  }
  return tau;
}

/** n x n mass matrix M(θ), built one column at a time from unit accelerations. */
export function MassMatrix(thetalist: Vec, Mlist: Mat[], Glist: Mat[], Slist: Mat): Mat {
  const n = thetalist.length;
  const M = zeros(n, n);
  const zero = zerosVec(n);
  for (let j = 0; j < n; j++) {
    const dd = zerosVec(n);
    dd[j] = 1;
    const col = InverseDynamics(thetalist, zero, dd, [0, 0, 0], zerosVec(6), Mlist, Glist, Slist);
    for (let i = 0; i < n; i++) M[i][j] = col[i];
  }
  return M;
}

/** Coriolis and centripetal joint forces c(θ, θdot). */
export function VelQuadraticForces(thetalist: Vec, dthetalist: Vec, Mlist: Mat[], Glist: Mat[], Slist: Mat): Vec {
  const n = thetalist.length;
  return InverseDynamics(thetalist, dthetalist, zerosVec(n), [0, 0, 0], zerosVec(6), Mlist, Glist, Slist);
}

/** Gravity joint forces g(θ). */
export function GravityForces(thetalist: Vec, g: Vec, Mlist: Mat[], Glist: Mat[], Slist: Mat): Vec {
  const n = thetalist.length;
  return InverseDynamics(thetalist, zerosVec(n), zerosVec(n), g, zerosVec(6), Mlist, Glist, Slist);
}

/** Joint forces J_b(θ)^T F_tip needed to produce the tip wrench. */
export function EndEffectorForces(thetalist: Vec, Ftip: Vec, Mlist: Mat[], Glist: Mat[], Slist: Mat): Vec {
  const n = thetalist.length;
  return InverseDynamics(thetalist, zerosVec(n), zerosVec(n), [0, 0, 0], Ftip, Mlist, Glist, Slist);
}

/**
 * Joint accelerations produced by joint forces `taulist`:
 *   θdd = M(θ)^-1 (τ - c(θ, θd) - g(θ) - J^T F_tip).
 */
export function ForwardDynamics(
  thetalist: Vec,
  dthetalist: Vec,
  taulist: Vec,
  g: Vec,
  Ftip: Vec,
  Mlist: Mat[],
  Glist: Mat[],
  Slist: Mat,
): Vec {
  const M = MassMatrix(thetalist, Mlist, Glist, Slist);
  const bias = vecAdd(
    vecAdd(VelQuadraticForces(thetalist, dthetalist, Mlist, Glist, Slist), GravityForces(thetalist, g, Mlist, Glist, Slist)),
    EndEffectorForces(thetalist, Ftip, Mlist, Glist, Slist),
  );
  return solve(M, vecSub(taulist, bias));
}

/** One explicit (semi-)Euler step: returns [θ + θd dt, θd + θdd dt]. */
export function EulerStep(thetalist: Vec, dthetalist: Vec, ddthetalist: Vec, dt: number): [Vec, Vec] {
  return [vecAdd(thetalist, vecScale(dthetalist, dt)), vecAdd(dthetalist, vecScale(ddthetalist, dt))];
}

/** State derivative function x' = f(t, x) used by the generic integrators. */
export type StateDerivative = (t: number, x: Vec) => Vec;

/** One classical 4th-order Runge-Kutta step for x' = f(t, x). */
export function rk4Step(f: StateDerivative, t: number, x: Vec, dt: number): Vec {
  const k1 = f(t, x);
  const k2 = f(t + dt / 2, vecAdd(x, vecScale(k1, dt / 2)));
  const k3 = f(t + dt / 2, vecAdd(x, vecScale(k2, dt / 2)));
  const k4 = f(t + dt, vecAdd(x, vecScale(k3, dt)));
  const incr = vecAdd(vecAdd(k1, vecScale(k2, 2)), vecAdd(vecScale(k3, 2), k4));
  return vecAdd(x, vecScale(incr, dt / 6));
}

/** One explicit Euler step for x' = f(t, x) (same signature as `rk4Step`). */
export function eulerStepGeneric(f: StateDerivative, t: number, x: Vec, dt: number): Vec {
  return vecAdd(x, vecScale(f(t, x), dt));
}

export type Integrator = 'euler' | 'rk4';

/**
 * Builds the first-order state derivative x = [θ; θd] -> [θd; θdd] of a
 * manipulator driven by a (possibly time-varying) torque and tip wrench.
 */
export function manipulatorODE(
  tau: (t: number, theta: Vec, dtheta: Vec) => Vec,
  g: Vec,
  Ftip: (t: number) => Vec,
  Mlist: Mat[],
  Glist: Mat[],
  Slist: Mat,
): StateDerivative {
  const n = Glist.length;
  return (t, x) => {
    const theta = x.slice(0, n);
    const dtheta = x.slice(n, 2 * n);
    const ddtheta = ForwardDynamics(theta, dtheta, tau(t, theta, dtheta), g, Ftip(t), Mlist, Glist, Slist);
    return dtheta.concat(ddtheta);
  };
}

/**
 * Joint forces along a trajectory. `thetamat[k]`, `dthetamat[k]`,
 * `ddthetamat[k]` and `Ftipmat[k]` describe time sample k; returns one torque
 * vector per sample.
 */
export function InverseDynamicsTrajectory(
  thetamat: Vec[],
  dthetamat: Vec[],
  ddthetamat: Vec[],
  g: Vec,
  Ftipmat: Vec[],
  Mlist: Mat[],
  Glist: Mat[],
  Slist: Mat,
): Vec[] {
  return thetamat.map((theta, k) =>
    InverseDynamics(theta, dthetamat[k], ddthetamat[k], g, Ftipmat[k], Mlist, Glist, Slist),
  );
}

/**
 * Simulates the open-loop response to a torque history. `taumat[k]` is held
 * constant (zero-order hold) during the k-th interval of length `dt`, which is
 * subdivided into `intRes` integrator steps. The returned arrays have the same
 * number of rows as `taumat`; row 0 is the initial state, row k is the state
 * at time k dt (so the last torque row is not used, exactly as in the book).
 */
export function ForwardDynamicsTrajectory(
  thetalist: Vec,
  dthetalist: Vec,
  taumat: Vec[],
  g: Vec,
  Ftipmat: Vec[],
  Mlist: Mat[],
  Glist: Mat[],
  Slist: Mat,
  dt: number,
  intRes = 1,
  integrator: Integrator = 'rk4',
): { thetamat: Vec[]; dthetamat: Vec[] } {
  const n = thetalist.length;
  const N = taumat.length;
  const thetamat: Vec[] = [thetalist.slice()];
  const dthetamat: Vec[] = [dthetalist.slice()];
  let x = thetalist.concat(dthetalist);
  const h = dt / Math.max(1, intRes);
  for (let k = 0; k < N - 1; k++) {
    const f = manipulatorODE(() => taumat[k], g, () => Ftipmat[k], Mlist, Glist, Slist);
    for (let j = 0; j < intRes; j++) {
      const t = k * dt + j * h;
      x = integrator === 'rk4' ? rk4Step(f, t, x, h) : eulerStepGeneric(f, t, x, h);
    }
    thetamat.push(x.slice(0, n));
    dthetamat.push(x.slice(n, 2 * n));
  }
  return { thetamat, dthetamat };
}

/**
 * Poses T_{0,i}(θ) of the link frames {1..n} and the end-effector frame {n+1}
 * in the base frame (n + 1 matrices). Useful for drawing links at their
 * centres of mass and for the energy helpers below.
 */
export function linkFramePoses(thetalist: Vec, Mlist: Mat[], Slist: Mat): Mat[] {
  const n = thetalist.length;
  const poses: Mat[] = [];
  let Mi = eye(4);
  let P = eye(4); // exp([S1]θ1) ... exp([Si]θi)
  for (let i = 0; i <= n; i++) {
    Mi = matMul(Mi, Mlist[i]);
    if (i < n) P = matMul(P, MatrixExp6(VecTose3(vecScale(getCol(Slist, i), thetalist[i]))));
    poses.push(matMul(P, Mi));
  }
  return poses;
}

/** Kinetic energy 1/2 θd^T M(θ) θd (J). */
export function KineticEnergy(thetalist: Vec, dthetalist: Vec, Mlist: Mat[], Glist: Mat[], Slist: Mat): number {
  const M = MassMatrix(thetalist, Mlist, Glist, Slist);
  const Md = matVec(M, dthetalist);
  let s = 0;
  for (let i = 0; i < dthetalist.length; i++) s += dthetalist[i] * Md[i];
  return 0.5 * s;
}

/**
 * Gravitational potential energy -sum_i m_i g . p_i (J), with p_i the centre
 * of mass of link i (the origin of its frame) in the base frame. Zero level:
 * the base-frame origin.
 */
export function PotentialEnergy(thetalist: Vec, g: Vec, Mlist: Mat[], Glist: Mat[], Slist: Mat): number {
  const poses = linkFramePoses(thetalist, Mlist, Slist);
  let U = 0;
  for (let i = 0; i < Glist.length; i++) {
    const m = Glist[i][3][3];
    const p = [poses[i][0][3], poses[i][1][3], poses[i][2][3]];
    U -= m * (g[0] * p[0] + g[1] * p[1] + g[2] * p[2]);
  }
  return U;
}

/** Total mechanical energy (kinetic + gravitational potential). */
export function TotalEnergy(thetalist: Vec, dthetalist: Vec, g: Vec, Mlist: Mat[], Glist: Mat[], Slist: Mat): number {
  return KineticEnergy(thetalist, dthetalist, Mlist, Glist, Slist) + PotentialEnergy(thetalist, g, Mlist, Glist, Slist);
}
