/**
 * Planar (2-D) helpers for the introductory lessons: 3x3 homogeneous
 * transforms, nR forward kinematics, the planar Jacobian, analytic 2R inverse
 * kinematics, closed-form 2R dynamics and a bridge that turns a planar chain
 * into the 3-D PoE / Newton-Euler description used by the rest of the library.
 *
 * Conventions
 *  - The base (space) frame sits at joint 1; link i points along +x when all
 *    joint angles are zero; joint angles are measured counter-clockwise.
 *  - Gravity acts along -y with magnitude `g` (default 9.81 m/s^2).
 *  - Units: m, kg, s, rad, N, N m.
 */

import { type Mat, type Vec, eye, matMul, zeros } from './linalg';
import { Adjoint, RpToTrans, TransInv } from './se3';

// ---------------------------------------------------------------------------
// SE(2)
// ---------------------------------------------------------------------------

/** 3x3 planar rotation by theta. */
export function rot2(theta: number): Mat {
  const c = Math.cos(theta);
  const s = Math.sin(theta);
  return [
    [c, -s, 0],
    [s, c, 0],
    [0, 0, 1],
  ];
}

/** 3x3 planar translation by (x, y). */
export function trans2(x: number, y: number): Mat {
  return [
    [1, 0, x],
    [0, 1, y],
    [0, 0, 1],
  ];
}

/** Planar transform with rotation theta followed by translation (x, y): [[R, p], [0, 1]]. */
export function se2(theta: number, x: number, y: number): Mat {
  const c = Math.cos(theta);
  const s = Math.sin(theta);
  return [
    [c, -s, x],
    [s, c, y],
    [0, 0, 1],
  ];
}

/** Inverse of a 3x3 planar transform. */
export function transInv2(T: Mat): Mat {
  const c = T[0][0];
  const s = T[1][0];
  const x = T[0][2];
  const y = T[1][2];
  return [
    [c, s, -(c * x + s * y)],
    [-s, c, s * x - c * y],
    [0, 0, 1],
  ];
}

/** Applies a 3x3 planar transform to a 2-D point. */
export function transformPoint2(T: Mat, p: Vec): Vec {
  return [T[0][0] * p[0] + T[0][1] * p[1] + T[0][2], T[1][0] * p[0] + T[1][1] * p[1] + T[1][2]];
}

// ---------------------------------------------------------------------------
// Planar nR kinematics
// ---------------------------------------------------------------------------

export interface PlanarFK {
  /** Positions of joint 1 (the base), joint 2, ..., joint n and the tip: n + 1 points. */
  joints: Vec[];
  /** Tip position (same as the last entry of `joints`). */
  tip: Vec;
  /** Absolute orientation of the last link: theta_1 + ... + theta_n. */
  phi: number;
  /** 3x3 tip frame (x-axis along the last link). */
  T: Mat;
}

/** Forward kinematics of a planar nR chain with the given link lengths. */
export function planarFK(linkLengths: number[], thetas: Vec): PlanarFK {
  const n = linkLengths.length;
  if (thetas.length !== n) throw new Error(`planarFK: ${n} link lengths but ${thetas.length} joint angles`);
  const joints: Vec[] = [[0, 0]];
  let x = 0;
  let y = 0;
  let phi = 0;
  for (let i = 0; i < n; i++) {
    phi += thetas[i];
    x += linkLengths[i] * Math.cos(phi);
    y += linkLengths[i] * Math.sin(phi);
    joints.push([x, y]);
  }
  return { joints, tip: [x, y], phi, T: se2(phi, x, y) };
}

/**
 * Planar Jacobian (3 x n) in the TWIST ORDER [omega_z; v_x; v_y]: row 0 is the
 * angular velocity of the last link (always 1 for each revolute joint), rows
 * 1-2 are the linear velocity of the TIP POINT expressed in the fixed frame.
 * (This is the tip-point Jacobian, not the book's space Jacobian whose
 * linear part is the velocity of the point at the origin.)
 */
export function planarJacobian(linkLengths: number[], thetas: Vec): Mat {
  const n = linkLengths.length;
  const J = zeros(3, n);
  const phis: number[] = [];
  let acc = 0;
  for (let i = 0; i < n; i++) {
    acc += thetas[i];
    phis.push(acc);
  }
  for (let j = 0; j < n; j++) {
    let vx = 0;
    let vy = 0;
    for (let k = j; k < n; k++) {
      vx -= linkLengths[k] * Math.sin(phis[k]);
      vy += linkLengths[k] * Math.cos(phis[k]);
    }
    J[0][j] = 1;
    J[1][j] = vx;
    J[2][j] = vy;
  }
  return J;
}

/** The 2 x n position rows [v_x; v_y] of `planarJacobian` (used for manipulability ellipses). */
export function planarTipJacobian(linkLengths: number[], thetas: Vec): Mat {
  const J = planarJacobian(linkLengths, thetas);
  return [J[1], J[2]];
}

export interface Planar2RSolution {
  theta1: number;
  theta2: number;
  /**
   * 'down' when theta2 >= 0 (the elbow lies to the right of the base->tip
   * direction, i.e. below the chord when the arm points along +x), 'up'
   * otherwise. The two solutions are often called "righty" / "lefty".
   */
  elbow: 'up' | 'down';
}

export interface Planar2RIK {
  /** False when the target is outside the annulus |L1 - L2| <= r <= L1 + L2. */
  reachable: boolean;
  /** Zero, one (boundary) or two solutions. */
  solutions: Planar2RSolution[];
  /** Distance from the base to the target. */
  distance: number;
}

/**
 * Analytic inverse kinematics of a planar 2R arm for the tip position (x, y):
 *   cos theta2 = (x^2 + y^2 - L1^2 - L2^2) / (2 L1 L2),
 *   theta1 = atan2(y, x) - atan2(L2 sin theta2, L1 + L2 cos theta2).
 */
export function planar2RIK(L1: number, L2: number, x: number, y: number): Planar2RIK {
  const r2 = x * x + y * y;
  const distance = Math.sqrt(r2);
  const c2 = (r2 - L1 * L1 - L2 * L2) / (2 * L1 * L2);
  if (c2 > 1 + 1e-12 || c2 < -1 - 1e-12) return { reachable: false, solutions: [], distance };
  const cc = Math.max(-1, Math.min(1, c2));
  const s2 = Math.sqrt(Math.max(0, 1 - cc * cc));
  const build = (sin2: number): Planar2RSolution => {
    const theta2 = Math.atan2(sin2, cc);
    const theta1 = Math.atan2(y, x) - Math.atan2(L2 * sin2, L1 + L2 * cc);
    return { theta1, theta2, elbow: theta2 >= 0 ? 'down' : 'up' };
  };
  // On the workspace boundary (|cos theta2| = 1 up to rounding) the two elbow solutions coincide.
  const solutions = 1 - Math.abs(cc) < 1e-12 ? [build(0)] : [build(s2), build(-s2)];
  return { reachable: true, solutions, distance };
}

// ---------------------------------------------------------------------------
// Closed-form 2R dynamics
// ---------------------------------------------------------------------------

export type PlanarLinkModel = 'point-mass' | 'rod';

export interface Planar2RParams {
  L1: number;
  L2: number;
  m1: number;
  m2: number;
  /** Distance from joint i to the centre of mass of link i along the link. */
  r1: number;
  r2: number;
  /** Rotational inertia of link i about its own centre of mass (z-axis). */
  I1: number;
  I2: number;
  /** Gravity magnitude (acts along -y). */
  g: number;
}

/**
 * Parameters for the two standard link models:
 *  - 'point-mass': all of link i's mass sits at its far end (r_i = L_i, I_i = 0);
 *  - 'rod': uniform thin rod (r_i = L_i / 2, I_i = m_i L_i^2 / 12).
 */
export function planar2RParams(L: [number, number], m: [number, number], model: PlanarLinkModel = 'point-mass', g = 9.81): Planar2RParams {
  const rod = model === 'rod';
  return {
    L1: L[0],
    L2: L[1],
    m1: m[0],
    m2: m[1],
    r1: rod ? L[0] / 2 : L[0],
    r2: rod ? L[1] / 2 : L[1],
    I1: rod ? (m[0] * L[0] * L[0]) / 12 : 0,
    I2: rod ? (m[1] * L[1] * L[1]) / 12 : 0,
    g,
  };
}

export interface Planar2RDynamics {
  /** 2x2 mass matrix M(θ). */
  M: Mat;
  /** Velocity-product (Coriolis + centripetal) vector c(θ, θd). */
  c: Vec;
  /** Gravity vector g(θ). */
  g: Vec;
}

/**
 * Closed-form M(θ), c(θ, θd), g(θ) of a planar 2R arm so that
 *   τ = M(θ) θdd + c(θ, θd) + g(θ).
 * Derived from the Lagrangian with link COM distances r_i and COM inertias
 * I_i (general enough for both point-mass and rod models).
 */
export function planar2RDynamics(P: Planar2RParams, theta: Vec, dtheta: Vec): Planar2RDynamics {
  const { L1, m1, m2, r1, r2, I1, I2, g } = P;
  const c2 = Math.cos(theta[1]);
  const s2 = Math.sin(theta[1]);
  const M11 = m1 * r1 * r1 + I1 + m2 * (L1 * L1 + r2 * r2 + 2 * L1 * r2 * c2) + I2;
  const M12 = m2 * (r2 * r2 + L1 * r2 * c2) + I2;
  const M22 = m2 * r2 * r2 + I2;
  const d1 = dtheta[0];
  const d2 = dtheta[1];
  const h = m2 * L1 * r2 * s2;
  const c: Vec = [-h * (2 * d1 * d2 + d2 * d2), h * d1 * d1];
  const c1 = Math.cos(theta[0]);
  const c12 = Math.cos(theta[0] + theta[1]);
  const gv: Vec = [(m1 * r1 + m2 * L1) * g * c1 + m2 * r2 * g * c12, m2 * r2 * g * c12];
  return {
    M: [
      [M11, M12],
      [M12, M22],
    ],
    c,
    g: gv,
  };
}

/** Joint accelerations of the closed-form 2R model under torques tau. */
export function planar2RAccel(P: Planar2RParams, theta: Vec, dtheta: Vec, tau: Vec): Vec {
  const { M, c, g } = planar2RDynamics(P, theta, dtheta);
  const b0 = tau[0] - c[0] - g[0];
  const b1 = tau[1] - c[1] - g[1];
  const detM = M[0][0] * M[1][1] - M[0][1] * M[1][0];
  return [(M[1][1] * b0 - M[0][1] * b1) / detM, (-M[1][0] * b0 + M[0][0] * b1) / detM];
}

// ---------------------------------------------------------------------------
// Bridge to the 3-D PoE / Newton-Euler description
// ---------------------------------------------------------------------------

export interface PlanarChain {
  /** Home configuration of the tip frame (4x4). */
  M: Mat;
  /** 6 x n space-frame screw axes (all joints rotate about +z). */
  Slist: Mat;
  /** 6 x n body-frame screw axes. */
  Blist: Mat;
  /** n + 1 link-frame transforms M_{i-1,i} (frames at the centres of mass). */
  Mlist: Mat[];
  /** n spatial inertia matrices. */
  Glist: Mat[];
  linkLengths: number[];
  masses: number[];
  model: PlanarLinkModel;
  /** COM distance of each link from its joint. */
  comDistances: number[];
}

export interface PlanarChainOptions {
  /** Link masses (kg); default 1 kg each. */
  masses?: number[];
  /** Mass distribution model; default 'rod'. */
  model?: PlanarLinkModel;
}

/**
 * Builds the PoE and Newton-Euler description of a planar nR chain lying in
 * the x-y plane with all joint axes along +z, so that results from
 * `FKinSpace`, `MassMatrix`, ... can be compared with the planar formulas.
 * Rods are modelled as thin (zero inertia about their own axis); point masses
 * have zero rotational inertia.
 */
export function buildPlanarChain(linkLengths: number[], opts: PlanarChainOptions = {}): PlanarChain {
  const n = linkLengths.length;
  const masses = opts.masses ?? new Array(n).fill(1);
  const model = opts.model ?? 'rod';
  if (masses.length !== n) throw new Error('buildPlanarChain: masses and linkLengths must have the same length');
  const r = linkLengths.map((L) => (model === 'rod' ? L / 2 : L));
  const Slist = zeros(6, n);
  let x = 0;
  for (let i = 0; i < n; i++) {
    Slist[2][i] = 1; // omega = z
    Slist[4][i] = -x; // v = -omega x q with q = (x, 0, 0)
    x += linkLengths[i];
  }
  const M = RpToTrans(eye(3), [x, 0, 0]);
  const Blist = matMul(Adjoint(TransInv(M)), Slist);
  const Mlist: Mat[] = [];
  const Glist: Mat[] = [];
  for (let i = 0; i <= n; i++) {
    let dx: number;
    if (i === 0) dx = r[0];
    else if (i === n) dx = linkLengths[n - 1] - r[n - 1];
    else dx = linkLengths[i - 1] - r[i - 1] + r[i];
    Mlist.push(RpToTrans(eye(3), [dx, 0, 0]));
  }
  for (let i = 0; i < n; i++) {
    const m = masses[i];
    const L = linkLengths[i];
    const Irod = model === 'rod' ? (m * L * L) / 12 : 0;
    const G = zeros(6, 6);
    G[0][0] = 0; // thin rod: no inertia about its own axis
    G[1][1] = Irod;
    G[2][2] = Irod;
    G[3][3] = m;
    G[4][4] = m;
    G[5][5] = m;
    Glist.push(G);
  }
  return { M, Slist, Blist, Mlist, Glist, linkLengths: linkLengths.slice(), masses: masses.slice(), model, comDistances: r };
}
