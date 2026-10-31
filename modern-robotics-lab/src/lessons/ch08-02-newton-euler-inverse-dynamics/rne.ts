/**
 * Pure helpers for the Newton–Euler lesson (no DOM).
 *
 *  - `bodyWrench`, `transformInertia`, `spinAt`: the single-rigid-body equation
 *    F = G V̇ − [ad_V]ᵀ G V for a body spinning about a body-fixed axis, written in the
 *    centre-of-mass frame {b} or in any other body-fixed frame {a}.
 *  - `rneaTrace`: the recursive Newton–Euler algorithm with every intermediate quantity kept
 *    (A_i, T_{i,i-1}, V_i, V̇_i, F_i, τ_i) so the lesson can replay it link by link. It follows
 *    the same conventions as `InverseDynamics` in '@/mr' and is tested against it.
 *  - `quinticTrajectory`, `torqueParts`, `trajectoryTorques`: torque curves along a joint
 *    trajectory split into inertial M θ̈, velocity c(θ, θ̇), gravity g(θ) and tip-load Jᵀ F parts,
 *    each obtained from one `InverseDynamics` call with the other inputs set to zero.
 *  - `measureCosts`: wall-clock cost of InverseDynamics vs MassMatrix on planar n-link chains.
 */
import {
  type Mat, type Vec,
  Adjoint, InverseDynamics, MassMatrix, MatrixExp6, QuinticTimeScalingDerivs, TransInv, TransToRp, VecTose3,
  ad, buildPlanarChain, diag, eye, getCol, linkFramePoses, matMul, matVec, transpose, vecAdd, vecScale, vecSub,
  zerosVec, FKinSpace,
} from '@/mr';

export interface ChainModel {
  /** Home pose of the end-effector frame. */
  M: Mat;
  Mlist: Mat[];
  Glist: Mat[];
  Slist: Mat;
}

const dot6 = (a: Vec, b: Vec): number => a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3] + a[4] * b[4] + a[5] * b[5];

// ---------------------------------------------------------------------------
// Single rigid body
// ---------------------------------------------------------------------------

/** F = G V̇ − [ad_V]ᵀ G V (the velocity-product term can be switched off to show it matters). */
export function bodyWrench(G: Mat, V: Vec, Vd: Vec, withAd = true): Vec {
  const inertial = matVec(G, Vd);
  if (!withAd) return inertial;
  return vecSub(inertial, matVec(transpose(ad(V)), matVec(G, V)));
}

/** The two parts of the body wrench: G V̇ and −[ad_V]ᵀ G V. */
export function bodyWrenchParts(G: Mat, V: Vec, Vd: Vec): { inertial: Vec; velocity: Vec } {
  return { inertial: matVec(G, Vd), velocity: vecScale(matVec(transpose(ad(V)), matVec(G, V)), -1) };
}

/** Spatial inertia expressed in another body-fixed frame {a}: G_a = [Ad_{T_ba}]ᵀ G_b [Ad_{T_ba}]. */
export function transformInertia(Gb: Mat, Tba: Mat): Mat {
  const A = Adjoint(Tba);
  return matMul(transpose(A), matMul(Gb, A));
}

/** 6×6 spatial inertia of a solid box (side lengths a, b, c along body x, y, z) at its centre of mass. */
export function boxSpatialInertia(m: number, dims: [number, number, number]): Mat {
  const [a, b, c] = dims;
  return diag([(m * (b * b + c * c)) / 12, (m * (a * a + c * c)) / 12, (m * (a * a + b * b)) / 12, m, m, m]);
}

export interface SpinSpec {
  /** Unit direction of the spin axis in body coordinates. */
  axis: Vec;
  /** A point on the axis in body coordinates (the COM frame {b} sits at the body's centre of mass). */
  point: Vec;
  /** Mean spin rate (rad/s). */
  omega0: number;
  /** Modulate the rate as ω0 (1 + ½ sin(2πt/period)). */
  varying: boolean;
  period: number;
  /** Pose of {b} at t = 0. */
  Ts0: Mat;
}

export interface SpinState {
  phi: number;
  dphi: number;
  ddphi: number;
  /** Constant screw axis of the motion in {b}: S_b = (ŝ, q × ŝ). */
  Sb: Vec;
  /** Body twist V_b = S_b φ̇ and its derivative V̇_b = S_b φ̈ (the axis is fixed in the body). */
  Vb: Vec;
  Vdb: Vec;
  /** T_sb(t) = T_sb(0) exp([S_b] φ). */
  Tsb: Mat;
}

/** Rotation of a rigid body about a body-fixed (and space-fixed) axis. */
export function spinAt(spec: SpinSpec, t: number): SpinState {
  const w0 = spec.omega0;
  let phi: number, dphi: number, ddphi: number;
  if (spec.varying) {
    const k = (2 * Math.PI) / spec.period;
    phi = w0 * t + (0.5 * w0 * (1 - Math.cos(k * t))) / k;
    dphi = w0 * (1 + 0.5 * Math.sin(k * t));
    ddphi = 0.5 * w0 * k * Math.cos(k * t);
  } else {
    phi = w0 * t; dphi = w0; ddphi = 0;
  }
  const s = spec.axis, q = spec.point;
  const v = [q[1] * s[2] - q[2] * s[1], q[2] * s[0] - q[0] * s[2], q[0] * s[1] - q[1] * s[0]];
  const Sb = [s[0], s[1], s[2], v[0], v[1], v[2]];
  return {
    phi, dphi, ddphi, Sb,
    Vb: vecScale(Sb, dphi),
    Vdb: vecScale(Sb, ddphi),
    Tsb: matMul(spec.Ts0, MatrixExp6(VecTose3(vecScale(Sb, phi)))),
  };
}

/** The same motion and wrench seen from a body-fixed frame {a} given by T_ba. */
export function wrenchInFrame(Gb: Mat, Tba: Mat, Vb: Vec, Vdb: Vec, withAd = true): { Ga: Mat; Va: Vec; Vda: Vec; Fa: Vec; FaFromFb: Vec; Fb: Vec } {
  const Ga = transformInertia(Gb, Tba);
  const AdAb = Adjoint(TransInv(Tba));
  const Va = matVec(AdAb, Vb);
  const Vda = matVec(AdAb, Vdb);
  const Fa = bodyWrench(Ga, Va, Vda, withAd);
  const Fb = bodyWrench(Gb, Vb, Vdb, withAd);
  const FaFromFb = matVec(transpose(Adjoint(Tba)), Fb);
  return { Ga, Va, Vda, Fa, FaFromFb, Fb };
}

// ---------------------------------------------------------------------------
// Recursive Newton–Euler with a full trace
// ---------------------------------------------------------------------------

export interface RneaTrace {
  n: number;
  /** A_i: screw axis of joint i in link frame {i} (index i − 1). */
  A: Vec[];
  /** T_{i,i−1}(θ) for i = 1..n (index i − 1) and T_{n+1,n} = M_{n,n+1}⁻¹ at index n. */
  Tparent: Mat[];
  /** Link twists V_0..V_n (V_0 = 0), each in its own link frame. */
  V: Vec[];
  /** Link accelerations V̇_0..V̇_n; V̇_0 = (0, −g) folds gravity into the recursion. */
  Vd: Vec[];
  /** F_1..F_{n+1} (index i − 1): wrench on link i from link i − 1, in {i}; F_{n+1} = F_tip. */
  F: Vec[];
  tau: Vec;
  /** Link frames T_{0,i} for i = 1..n + 1 (index i − 1); frame n + 1 is the end-effector. */
  poses: Mat[];
}

/** Recursive Newton–Euler inverse dynamics keeping every intermediate quantity. */
export function rneaTrace(theta: Vec, dtheta: Vec, ddtheta: Vec, g: Vec, Ftip: Vec, model: ChainModel): RneaTrace {
  const { Mlist, Glist, Slist } = model;
  const n = theta.length;
  let Mi = eye(4);
  const A: Vec[] = [];
  const Tparent: Mat[] = [];
  const AdT: Mat[] = [];
  const V: Vec[] = [zerosVec(6)];
  const Vd: Vec[] = [[0, 0, 0, -g[0], -g[1], -g[2]]];
  // forward pass: base → tip
  for (let i = 0; i < n; i++) {
    Mi = matMul(Mi, Mlist[i]);
    const Ai = matVec(Adjoint(TransInv(Mi)), getCol(Slist, i));
    A.push(Ai);
    const T = matMul(MatrixExp6(VecTose3(vecScale(Ai, -theta[i]))), TransInv(Mlist[i]));
    Tparent.push(T);
    const AdTi = Adjoint(T);
    AdT.push(AdTi);
    const Vi = vecAdd(matVec(AdTi, V[i]), vecScale(Ai, dtheta[i]));
    V.push(Vi);
    Vd.push(vecAdd(vecAdd(matVec(AdTi, Vd[i]), vecScale(Ai, ddtheta[i])), matVec(ad(Vi), vecScale(Ai, dtheta[i]))));
  }
  const Tend = TransInv(Mlist[n]);
  Tparent.push(Tend);
  AdT.push(Adjoint(Tend));
  // backward pass: tip → base
  const F: Vec[] = new Array(n + 1);
  F[n] = Ftip.slice();
  const tau = zerosVec(n);
  for (let i = n - 1; i >= 0; i--) {
    const G = Glist[i];
    const Vi = V[i + 1];
    F[i] = vecSub(vecAdd(matVec(transpose(AdT[i + 1]), F[i + 1]), matVec(G, Vd[i + 1])), matVec(transpose(ad(Vi)), matVec(G, Vi)));
    tau[i] = dot6(F[i], A[i]);
  }
  return { n, A, Tparent, V, Vd, F, tau, poses: linkFramePoses(theta, Mlist, Slist) };
}

// ---------------------------------------------------------------------------
// Torque decomposition along a trajectory
// ---------------------------------------------------------------------------

export interface TorqueParts {
  /** M(θ) θ̈ */
  inertial: Vec;
  /** c(θ, θ̇) */
  velocity: Vec;
  /** g(θ) */
  gravity: Vec;
  /** J_bᵀ F_tip */
  load: Vec;
  /** Full InverseDynamics(θ, θ̇, θ̈, g, F_tip); equals the sum of the four parts. */
  total: Vec;
}

export function torqueParts(model: ChainModel, th: Vec, dth: Vec, ddth: Vec, g: Vec, Ftip: Vec): TorqueParts {
  const { Mlist, Glist, Slist } = model;
  const n = th.length;
  const z = zerosVec(n), z6 = zerosVec(6), g0 = [0, 0, 0];
  return {
    inertial: InverseDynamics(th, z, ddth, g0, z6, Mlist, Glist, Slist),
    velocity: InverseDynamics(th, dth, z, g0, z6, Mlist, Glist, Slist),
    gravity: InverseDynamics(th, z, z, g, z6, Mlist, Glist, Slist),
    load: InverseDynamics(th, z, z, g0, Ftip, Mlist, Glist, Slist),
    total: InverseDynamics(th, dth, ddth, g, Ftip, Mlist, Glist, Slist),
  };
}

/**
 * Wrench the end-effector applies (in its own frame {b}) to hold a point mass at its origin
 * against gravity: it pushes on the load with −m g, so F_tip = (0, R_sbᵀ(−m g)). Quasi-static:
 * the load's own inertia is ignored.
 */
export function holdLoadWrench(Tsb: Mat, mass: number, g: Vec): Vec {
  const [R] = TransToRp(Tsb);
  const fs = [-mass * g[0], -mass * g[1], -mass * g[2]];
  return [0, 0, 0,
    R[0][0] * fs[0] + R[1][0] * fs[1] + R[2][0] * fs[2],
    R[0][1] * fs[0] + R[1][1] * fs[1] + R[2][1] * fs[2],
    R[0][2] * fs[0] + R[1][2] * fs[1] + R[2][2] * fs[2]];
}

export interface JointTraj {
  /** Normalised time t / T_f of every sample (0..1). */
  s: number[];
  theta: Vec[];
  dtheta: Vec[];
  ddtheta: Vec[];
}

/** Straight-line joint motion with quintic time scaling (rest to rest), N samples over [0, T_f]. */
export function quinticTrajectory(start: Vec, end: Vec, Tf: number, N: number): JointTraj {
  const out: JointTraj = { s: [], theta: [], dtheta: [], ddtheta: [] };
  const d = end.map((e, i) => e - start[i]);
  for (let k = 0; k < N; k++) {
    const sn = k / (N - 1);
    const ts = QuinticTimeScalingDerivs(Tf, sn * Tf);
    out.s.push(sn);
    out.theta.push(start.map((q, i) => q + d[i] * ts.s));
    out.dtheta.push(d.map((di) => di * ts.sdot));
    out.ddtheta.push(d.map((di) => di * ts.sddot));
  }
  return out;
}

/** Torque parts at every sample of a trajectory; the tip load is a point mass held at the tool. */
export function trajectoryTorques(model: ChainModel, traj: JointTraj, g: Vec, loadMass: number): TorqueParts[] {
  return traj.theta.map((th, k) => {
    const Ftip = loadMass > 0 ? holdLoadWrench(FKinSpace(model.M, model.Slist, th), loadMass, g) : zerosVec(6);
    return torqueParts(model, th, traj.dtheta[k], traj.ddtheta[k], g, Ftip);
  });
}

/**
 * Torque curves of one path computed once and rescaled exactly afterwards: with s = t/T_f,
 * θ̇ ∝ 1/T_f and θ̈ ∝ 1/T_f², so the inertial and velocity parts scale with 1/T_f², while the
 * gravity part does not depend on T_f and the load part is linear in the load mass.
 */
export interface BaseCurves {
  s: number[];
  /** Parts for T_f = 1 s and a 1 kg load (gravity included). */
  ref: TorqueParts[];
}

export function baseCurves(model: ChainModel, start: Vec, end: Vec, N: number, g: Vec): BaseCurves {
  const traj = quinticTrajectory(start, end, 1, N);
  return { s: traj.s, ref: trajectoryTorques(model, traj, g, 1) };
}

/** Parts for duration T_f, gravity on/off and a load of `loadMass` kg (the load only has weight). */
export function scaleCurves(base: BaseCurves, Tf: number, gravityOn: boolean, loadMass: number): TorqueParts[] {
  const k = 1 / (Tf * Tf);
  const gk = gravityOn ? 1 : 0;
  const lk = gravityOn ? loadMass : 0;
  return base.ref.map((p) => {
    const inertial = p.inertial.map((v) => v * k);
    const velocity = p.velocity.map((v) => v * k);
    const gravity = p.gravity.map((v) => v * gk);
    const load = p.load.map((v) => v * lk);
    return { inertial, velocity, gravity, load, total: inertial.map((v, i) => v + velocity[i] + gravity[i] + load[i]) };
  });
}

// ---------------------------------------------------------------------------
// Cost of the recursion vs. building M(θ)
// ---------------------------------------------------------------------------

export interface CostSample { n: number; idUs: number; massUs: number }

/** Best-of-batches wall-clock cost (µs per call) of `fn`, after a short time-bounded warm-up. */
function timeIt(fn: () => void, batchMs: number, batches: number): number {
  const w0 = performance.now();
  for (let k = 0; k < 20 && performance.now() - w0 < batchMs; k++) fn();
  let best = Infinity;
  for (let b = 0; b < batches; b++) {
    let calls = 0;
    const t0 = performance.now();
    let el = 0;
    while (el < batchMs || calls < 2) { fn(); calls++; el = performance.now() - t0; }
    best = Math.min(best, (el * 1000) / calls);
  }
  return best;
}

/**
 * Cost of InverseDynamics and of MassMatrix (n inverse-dynamics passes) on a planar n-link chain.
 * Best of five short batches, so other work on the machine inflates the result as little as possible.
 */
export function measureCost(n: number, batchMs = 3): CostSample {
  const chain = buildPlanarChain(new Array(n).fill(0.3), { model: 'rod' });
  const th = Array.from({ length: n }, (_, i) => 0.3 * Math.sin(i + 1));
  const dth = Array.from({ length: n }, (_, i) => 0.5 * Math.cos(i));
  const ddth = Array.from({ length: n }, (_, i) => 0.2 * (i % 3));
  const g = [0, -9.81, 0], z6 = zerosVec(6);
  const idUs = timeIt(() => { InverseDynamics(th, dth, ddth, g, z6, chain.Mlist, chain.Glist, chain.Slist); }, batchMs, 5);
  const massUs = timeIt(() => { MassMatrix(th, chain.Mlist, chain.Glist, chain.Slist); }, batchMs, 5);
  return { n, idUs, massUs };
}

/** Wall-clock cost of one InverseDynamics call on a given model (µs). */
export function measureModelCost(model: ChainModel, th: Vec, batchMs = 3): number {
  const z6 = zerosVec(6), g = [0, 0, -9.81];
  return timeIt(() => { InverseDynamics(th, th, th, g, z6, model.Mlist, model.Glist, model.Slist); }, batchMs, 3);
}

/**
 * Least-squares slope k of log t against log n, i.e. the growth exponent in t ≈ c·n^k.
 * Unlike the doubling ratio of a fitted a + b·n or a + b·n² curve (which is fixed by the model
 * choice), k is read off the measurements themselves.
 */
export function growthExponent(ns: number[], ts: number[]): number {
  const pts = ns.map((n, i) => [Math.log(n), Math.log(ts[i])]).filter(([x, y]) => Number.isFinite(x) && Number.isFinite(y));
  const m = pts.length;
  if (m < 2) return NaN;
  const sx = pts.reduce((a, p) => a + p[0], 0), sy = pts.reduce((a, p) => a + p[1], 0);
  const sxx = pts.reduce((a, p) => a + p[0] * p[0], 0), sxy = pts.reduce((a, p) => a + p[0] * p[1], 0);
  const den = m * sxx - sx * sx;
  return den !== 0 ? (m * sxy - sx * sy) / den : NaN;
}
