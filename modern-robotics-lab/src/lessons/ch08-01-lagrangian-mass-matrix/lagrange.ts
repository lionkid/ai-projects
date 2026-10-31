/**
 * Pure model code for the Lagrangian 2R lesson (no DOM).
 *
 * The arm is the classic planar 2R example: two massless links of lengths L1, L2 carrying point
 * masses m1, m2 at their far ends. Joint angles are measured counter-clockwise (θ1 from +x, θ2
 * relative to link 1) and gravity of magnitude g acts along −y; g = 0 models an arm moving on a
 * horizontal table. The closed-form M, c, g come from `planar2RDynamics` in '@/mr' (the Lagrangian
 * result); this module adds what the lesson visualises on top of it:
 *   - kinetic / potential energy and the individual Euler–Lagrange terms,
 *   - the split of c(θ, θ̇) into centripetal (θ̇ᵢ²) and Coriolis (θ̇₁θ̇₂) parts,
 *   - the constant-kinetic-energy ellipse of M(θ) in the joint-velocity plane,
 *   - "hybrid" dynamics where every joint is either free (torque given) or driven along a
 *     prescribed motion (acceleration given), and fixed-step RK4 / explicit Euler integration,
 *   - a side-by-side comparison with the Newton–Euler routines of '@/mr'.
 */
import {
  type Mat, type PlanarChain, type Planar2RParams, type Vec,
  GravityForces, InverseDynamics, MassMatrix, QuinticTimeScalingDerivs, VelQuadraticForces,
  buildPlanarChain, eigSym, eulerStepGeneric, planar2RDynamics, planar2RParams, rk4Step,
} from '@/mr';

export interface Arm {
  L1: number;
  L2: number;
  m1: number;
  m2: number;
  /** Gravity magnitude (m/s²) along −y; 0 = horizontal plane. */
  g: number;
}

export const GRAVITY = 9.81;
export const DEFAULT_ARM: Arm = { L1: 1, L2: 1, m1: 1, m2: 1, g: GRAVITY };

/** Parameters for the '@/mr' closed form (point masses at the link tips). */
export function closedFormParams(a: Arm): Planar2RParams {
  return planar2RParams([a.L1, a.L2], [a.m1, a.m2], 'point-mass', a.g);
}

export interface Terms { M: Mat; c: Vec; g: Vec }

/** M(θ), c(θ, θ̇), g(θ) of the Lagrangian closed form. */
export function dynamicsTerms(a: Arm, th: Vec, dth: Vec): Terms {
  return planar2RDynamics(closedFormParams(a), th, dth);
}

/** h(θ2) = m2 L1 L2 sin θ2: the single coefficient behind every velocity-product term. */
export function hCoef(a: Arm, th: Vec): number {
  return a.m2 * a.L1 * a.L2 * Math.sin(th[1]);
}

/** ∂M/∂θ2 (M does not depend on θ1). */
export function dMdTheta2(a: Arm, th: Vec): Mat {
  const h = hCoef(a, th);
  return [[-2 * h, -h], [-h, 0]];
}

/** c split into its centripetal (θ̇ᵢ² terms) and Coriolis (θ̇₁θ̇₂ terms) parts; they sum to c. */
export function splitC(a: Arm, th: Vec, dth: Vec): { centripetal: Vec; coriolis: Vec } {
  const h = hCoef(a, th);
  return {
    centripetal: [-h * dth[1] * dth[1], h * dth[0] * dth[0]],
    coriolis: [-2 * h * dth[0] * dth[1], 0],
  };
}

/** Positions of the two point masses (= elbow and tip). */
export function massPositions(a: Arm, th: Vec): [Vec, Vec] {
  const p1 = [a.L1 * Math.cos(th[0]), a.L1 * Math.sin(th[0])];
  const p2 = [p1[0] + a.L2 * Math.cos(th[0] + th[1]), p1[1] + a.L2 * Math.sin(th[0] + th[1])];
  return [p1, p2];
}

/** K = ½ θ̇ᵀ M(θ) θ̇. */
export function kineticEnergy(a: Arm, th: Vec, dth: Vec): number {
  const { M } = dynamicsTerms(a, th, [0, 0]);
  return 0.5 * (M[0][0] * dth[0] * dth[0] + 2 * M[0][1] * dth[0] * dth[1] + M[1][1] * dth[1] * dth[1]);
}

/**
 * Gravitational potential energy with its zero at the arm hanging straight down
 * (so U ≥ 0 for every configuration): U = g [m1 (y1 + L1) + m2 (y2 + L1 + L2)].
 */
export function potentialEnergy(a: Arm, th: Vec): number {
  const [p1, p2] = massPositions(a, th);
  return a.g * (a.m1 * (p1[1] + a.L1) + a.m2 * (p2[1] + a.L1 + a.L2));
}

/** State x = [θ1, θ2, θ̇1, θ̇2] → { K, U, E }. */
export function energies(a: Arm, x: Vec): { K: number; U: number; E: number } {
  const K = kineticEnergy(a, [x[0], x[1]], [x[2], x[3]]);
  const U = potentialEnergy(a, [x[0], x[1]]);
  return { K, U, E: K + U };
}

export interface EulerLagrangeTerms {
  /** Generalised momentum p = ∂L/∂θ̇ = M θ̇. */
  momentum: Vec;
  /** d/dt ∂L/∂θ̇ = Ṁ θ̇ + M θ̈. */
  dMomentum: Vec;
  /** ∂L/∂θ = ½ θ̇ᵀ (∂M/∂θᵢ) θ̇ − ∂U/∂θᵢ. */
  dLdTheta: Vec;
  /** τ = d/dt ∂L/∂θ̇ − ∂L/∂θ. */
  tau: Vec;
}

/** The two sides of the Euler–Lagrange equations evaluated at (θ, θ̇, θ̈). */
export function eulerLagrangeTerms(a: Arm, th: Vec, dth: Vec, ddth: Vec): EulerLagrangeTerms {
  const { M, g } = dynamicsTerms(a, th, dth);
  const dM = dMdTheta2(a, th);
  const momentum = [M[0][0] * dth[0] + M[0][1] * dth[1], M[1][0] * dth[0] + M[1][1] * dth[1]];
  // Ṁ = (∂M/∂θ2) θ̇2
  const Md = dM.map((r) => r.map((v) => v * dth[1]));
  const dMomentum = [
    Md[0][0] * dth[0] + Md[0][1] * dth[1] + M[0][0] * ddth[0] + M[0][1] * ddth[1],
    Md[1][0] * dth[0] + Md[1][1] * dth[1] + M[1][0] * ddth[0] + M[1][1] * ddth[1],
  ];
  const quad2 = 0.5 * (dM[0][0] * dth[0] * dth[0] + 2 * dM[0][1] * dth[0] * dth[1] + dM[1][1] * dth[1] * dth[1]);
  const dLdTheta = [-g[0], quad2 - g[1]];
  return { momentum, dMomentum, dLdTheta, tau: [dMomentum[0] - dLdTheta[0], dMomentum[1] - dLdTheta[1]] };
}

// ---------------------------------------------------------------------------
// Mass-matrix ellipse
// ---------------------------------------------------------------------------

export interface EnergyEllipse {
  /** Eigenvalues of M, descending (λ1 ≥ λ2 > 0). */
  lambda: [number, number];
  /** Unit eigenvectors (columns of the eigenbasis) matching `lambda`. */
  vectors: [Vec, Vec];
  /** Semi-axes √(2K/λᵢ) of { θ̇ : ½ θ̇ᵀ M θ̇ = K }, matching `lambda` (so semi[0] ≤ semi[1]). */
  semi: [number, number];
  /** Angle (rad) of the LONG axis (direction of λ2, the "light" direction) from the θ̇1 axis. */
  longAngle: number;
}

/** The set of joint velocities with kinetic energy K is an ellipse whose axes are M's eigenvectors. */
export function energyEllipse(M: Mat, K: number): EnergyEllipse {
  const { values, vectors } = eigSym(M);
  const l1 = values[0], l2 = values[1];
  const v1 = [vectors[0][0], vectors[1][0]];
  const v2 = [vectors[0][1], vectors[1][1]];
  return {
    lambda: [l1, l2],
    vectors: [v1, v2],
    semi: [Math.sqrt((2 * K) / l1), Math.sqrt((2 * K) / l2)],
    longAngle: Math.atan2(v2[1], v2[0]),
  };
}

/** Largest semi-axis of the K-ellipse over θ2 ∈ [0, π] (for choosing a fixed plot range). */
export function maxEllipseSemiAxis(a: Arm, K: number, samples = 61): number {
  let best = 0;
  for (let k = 0; k < samples; k++) {
    const th2 = (Math.PI * k) / (samples - 1);
    const e = energyEllipse(dynamicsTerms(a, [0, th2], [0, 0]).M, K);
    best = Math.max(best, e.semi[1]);
  }
  return best;
}

// ---------------------------------------------------------------------------
// Hybrid dynamics: each joint is free (torque given) or driven (acceleration given)
// ---------------------------------------------------------------------------

export type Drive = { kind: 'free'; tau: number } | { kind: 'driven'; ddq: number };

export interface HybridResult { ddth: Vec; tau: Vec; M: Mat; c: Vec; g: Vec }

/**
 * Solve τ = M θ̈ + c + g for the unknowns: θ̈ of free joints and τ of driven joints.
 * `dropC` removes the velocity-product term (a deliberately wrong model used to show that the
 * energy check catches modelling errors).
 */
export function solveHybrid(a: Arm, th: Vec, dth: Vec, drives: [Drive, Drive], dropC = false): HybridResult {
  const t = dynamicsTerms(a, th, dth);
  const M = t.M, g = t.g;
  const c = dropC ? [0, 0] : t.c;
  const b = [c[0] + g[0], c[1] + g[1]];
  const [d1, d2] = drives;
  let a1: number, a2: number;
  if (d1.kind === 'free' && d2.kind === 'free') {
    const r0 = d1.tau - b[0], r1 = d2.tau - b[1];
    const det = M[0][0] * M[1][1] - M[0][1] * M[1][0];
    a1 = (M[1][1] * r0 - M[0][1] * r1) / det;
    a2 = (-M[1][0] * r0 + M[0][0] * r1) / det;
  } else if (d1.kind === 'driven' && d2.kind === 'driven') {
    a1 = d1.ddq; a2 = d2.ddq;
  } else if (d1.kind === 'driven' && d2.kind === 'free') {
    a1 = d1.ddq;
    a2 = (d2.tau - b[1] - M[1][0] * a1) / M[1][1];
  } else {
    // joint 1 free, joint 2 driven
    a2 = (d2 as { ddq: number }).ddq;
    a1 = ((d1 as { tau: number }).tau - b[0] - M[0][1] * a2) / M[0][0];
  }
  const tau = [M[0][0] * a1 + M[0][1] * a2 + b[0], M[1][0] * a1 + M[1][1] * a2 + b[1]];
  return { ddth: [a1, a2], tau, M, c, g };
}

export interface JointSample { q: number; dq: number; ddq: number }

/** What a joint does: free with a constant torque, or driven along q(t). */
export type JointProgram =
  | { kind: 'free'; tau: number }
  | { kind: 'driven'; at: (t: number) => JointSample };

export const freeJoint = (tau = 0): JointProgram => ({ kind: 'free', tau });

/** q(t) = q0 + w t. */
export function constantRate(q0: number, w: number): JointProgram {
  return { kind: 'driven', at: (t) => ({ q: q0 + w * t, dq: w, ddq: 0 }) };
}

export const holdAt = (q0: number): JointProgram => constantRate(q0, 0);

/** q(t) = q0 + A sin(2πt/T + φ). */
export function sineMotion(q0: number, amp: number, period: number, phase = 0): JointProgram {
  const w = (2 * Math.PI) / period;
  return {
    kind: 'driven',
    at: (t) => ({
      q: q0 + amp * Math.sin(w * t + phase),
      dq: amp * w * Math.cos(w * t + phase),
      ddq: -amp * w * w * Math.sin(w * t + phase),
    }),
  };
}

/**
 * Periodic fold / unfold with rests: hold `qa` for `rest` s, move to `qb` in `move` s (quintic),
 * hold `qb` for `rest` s, move back, repeat. Period 2 (rest + move).
 */
export function foldCycle(qa: number, qb: number, rest: number, move: number): JointProgram {
  const period = 2 * (rest + move);
  return {
    kind: 'driven',
    at: (t) => {
      let u = t % period;
      if (u < 0) u += period;
      let from = qa, to = qb;
      if (u < rest) return { q: qa, dq: 0, ddq: 0 };
      u -= rest;
      if (u >= move) {
        u -= move;
        if (u < rest) return { q: qb, dq: 0, ddq: 0 };
        u -= rest;
        from = qb; to = qa;
      }
      const s = QuinticTimeScalingDerivs(move, u);
      const d = to - from;
      return { q: from + d * s.s, dq: d * s.sdot, ddq: d * s.sddot };
    },
  };
}

export type Method = 'rk4' | 'euler';

function drivesAt(programs: [JointProgram, JointProgram], t: number): [Drive, Drive] {
  return programs.map((p): Drive => (p.kind === 'free' ? { kind: 'free', tau: p.tau } : { kind: 'driven', ddq: p.at(t).ddq })) as [Drive, Drive];
}

/**
 * Advance x = [θ1, θ2, θ̇1, θ̇2] by one fixed step h. Driven joints are re-synchronised with their
 * prescribed motion after the step, so they never drift.
 */
export function stepState(a: Arm, x: Vec, t: number, h: number, programs: [JointProgram, JointProgram], method: Method = 'rk4', dropC = false): Vec {
  const f = (tt: number, xx: Vec): Vec => {
    const r = solveHybrid(a, [xx[0], xx[1]], [xx[2], xx[3]], drivesAt(programs, tt), dropC);
    return [xx[2], xx[3], r.ddth[0], r.ddth[1]];
  };
  const xn = method === 'rk4' ? rk4Step(f, t, x, h) : eulerStepGeneric(f, t, x, h);
  programs.forEach((p, i) => {
    if (p.kind === 'driven') { const s = p.at(t + h); xn[i] = s.q; xn[2 + i] = s.dq; }
  });
  return xn;
}

/** The solved accelerations and torques at state x and time t (for readouts). */
export function evaluate(a: Arm, x: Vec, t: number, programs: [JointProgram, JointProgram], dropC = false): HybridResult {
  return solveHybrid(a, [x[0], x[1]], [x[2], x[3]], drivesAt(programs, t), dropC);
}

/** Initial state consistent with the programs: driven joints start on their prescribed motion. */
export function initialState(th: Vec, dth: Vec, programs: [JointProgram, JointProgram]): Vec {
  const x = [th[0], th[1], dth[0], dth[1]];
  programs.forEach((p, i) => {
    if (p.kind === 'driven') { const s = p.at(0); x[i] = s.q; x[2 + i] = s.dq; }
  });
  return x;
}

/** Wrap both angles to (−π, π] (the dynamics only depend on sin/cos). */
export function wrapAngles(x: Vec): Vec {
  const w = (q: number) => q - 2 * Math.PI * Math.floor((q + Math.PI) / (2 * Math.PI));
  return [w(x[0]), w(x[1]), x[2], x[3]];
}

// ---------------------------------------------------------------------------
// Comparison with the recursive Newton–Euler routines of '@/mr'
// ---------------------------------------------------------------------------

/** The same arm in the PoE / Newton–Euler description (frames at the point masses). */
export function chainFor(a: Arm): PlanarChain {
  return buildPlanarChain([a.L1, a.L2], { masses: [a.m1, a.m2], model: 'point-mass' });
}

export interface Comparison {
  closed: { M: Mat; c: Vec; g: Vec; tau: Vec };
  ne: { M: Mat; c: Vec; g: Vec; tau: Vec };
  /** Largest absolute difference over all entries of M, c, g and τ. */
  maxDiff: number;
}

/**
 * Closed-form (Lagrangian) M, c, g and τ = M θ̈ + c + g next to MassMatrix / VelQuadraticForces /
 * GravityForces / InverseDynamics on the equivalent chain (gravity [0, −g, 0] in the plane).
 */
export function compareWithNewtonEuler(a: Arm, chain: PlanarChain, th: Vec, dth: Vec, ddth: Vec): Comparison {
  const cf = dynamicsTerms(a, th, dth);
  const tauC = [
    cf.M[0][0] * ddth[0] + cf.M[0][1] * ddth[1] + cf.c[0] + cf.g[0],
    cf.M[1][0] * ddth[0] + cf.M[1][1] * ddth[1] + cf.c[1] + cf.g[1],
  ];
  const gv = [0, -a.g, 0];
  const zero6 = [0, 0, 0, 0, 0, 0];
  const M = MassMatrix(th, chain.Mlist, chain.Glist, chain.Slist);
  const c = VelQuadraticForces(th, dth, chain.Mlist, chain.Glist, chain.Slist);
  const g = GravityForces(th, gv, chain.Mlist, chain.Glist, chain.Slist);
  const tau = InverseDynamics(th, dth, ddth, gv, zero6, chain.Mlist, chain.Glist, chain.Slist);
  let maxDiff = 0;
  for (let i = 0; i < 2; i++) {
    for (let j = 0; j < 2; j++) maxDiff = Math.max(maxDiff, Math.abs(M[i][j] - cf.M[i][j]));
    maxDiff = Math.max(maxDiff, Math.abs(c[i] - cf.c[i]), Math.abs(g[i] - cf.g[i]), Math.abs(tau[i] - tauC[i]));
  }
  return { closed: { M: cf.M, c: cf.c, g: cf.g, tau: tauC }, ne: { M, c, g, tau }, maxDiff };
}
