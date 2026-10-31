/**
 * Kinematics of omnidirectional wheeled bases (omniwheels and mecanum wheels) for ch13-01.
 *
 * Conventions (Modern Robotics, chapter 13)
 *  - Chassis configuration q = (φ, x, y); body twist V_b = (ω, v_x, v_y), angular part first like
 *    the 6-vector twists of @/mr. Planar twists are embedded in SE(3) as [0, 0, ω, v_x, v_y, 0].
 *  - Wheel i sits at p_i = (x_i, y_i) in {b}. Its driving (rolling) direction makes the angle β_i
 *    with x_b. The rollers slide freely along the direction that makes the angle γ_i with the wheel
 *    axle (the axle is perpendicular to the driving direction): γ = 0 is an omniwheel, γ = ±π/4 a
 *    mecanum wheel. u_i is the wheel's driving speed in rad/s, r_i its radius.
 *
 * Row i of H(0): only the component of the contact-point velocity along d̂_i (perpendicular to the
 * free-sliding direction, at angle β_i + γ_i) has to be produced by rolling, and rolling at speed u_i
 * moves the contact point by r_i u_i cos γ_i along d̂_i, hence
 *     u_i = [ (p_i × d̂_i)_z , d̂_iᵀ ] V_b / (r_i cos γ_i).
 */
import { MatrixExp6, VecTose3, matMul, matVec, pinv, svd, type Mat, type Vec } from '@/mr';

export interface Wheel {
  /** Wheel centre in {b} (m). */
  x: number;
  y: number;
  /** Driving direction measured from x_b (rad). */
  beta: number;
  /** Free-sliding direction of the rollers measured from the wheel axle (rad); |γ| < π/2. */
  gamma: number;
  /** Wheel radius (m). */
  r: number;
}

/** Planar twist (ω, v_x, v_y). */
export type Twist2 = [number, number, number];
/** Planar pose; φ is kept unwrapped by the integrators (wrap only for display). */
export interface Pose { phi: number; x: number; y: number }
export type P2 = [number, number];

// ---------------------------------------------------------------------------------------------
// Geometry of one wheel and the H matrix
// ---------------------------------------------------------------------------------------------

export interface WheelGeometry {
  /** Unit driving direction (β). */
  drive: P2;
  /** Unit free-sliding direction of the rollers (β + π/2 + γ). */
  slide: P2;
  /** Unit direction that the wheel must supply by rolling (β + γ), perpendicular to `slide`. */
  dir: P2;
  /** Moment arm (p × d̂)_z: how strongly a spin of the chassis loads this wheel. */
  lever: number;
  /** 1 / (r cos γ). */
  gain: number;
}

export function wheelGeometry(w: Wheel): WheelGeometry {
  const a = w.beta + w.gamma;
  const dir: P2 = [Math.cos(a), Math.sin(a)];
  return {
    drive: [Math.cos(w.beta), Math.sin(w.beta)],
    slide: [-Math.sin(a), Math.cos(a)],
    dir,
    lever: w.x * dir[1] - w.y * dir[0],
    gain: 1 / (w.r * Math.cos(w.gamma)),
  };
}

/** Row h_i(0) of H(0), so that u_i = h_i(0) · V_b. */
export function wheelRow(w: Wheel): Twist2 {
  const g = wheelGeometry(w);
  return [g.gain * g.lever, g.gain * g.dir[0], g.gain * g.dir[1]];
}

/** H(0): one row per wheel (m × 3). */
export function Hmatrix(wheels: Wheel[]): Mat {
  return wheels.map(wheelRow);
}

/** Rotation that maps q̇ = (φ̇, ẋ, ẏ) to the body twist V_b. */
export function qdotToBody(phi: number): Mat {
  const c = Math.cos(phi), s = Math.sin(phi);
  return [[1, 0, 0], [0, c, s], [0, -s, c]];
}

/** H(φ) = H(0) · [[1, 0, 0], [0, cos φ, sin φ], [0, −sin φ, cos φ]], so that u = H(φ) q̇. */
export function Hphi(H0: Mat, phi: number): Mat {
  return matMul(H0, qdotToBody(phi));
}

/** Velocity of the contact point of wheel `w` (in {b}) for the chassis twist V: v + ω ẑ × p. */
export function contactVelocity(w: Wheel, V: Twist2): P2 {
  return [V[1] - V[0] * w.y, V[2] + V[0] * w.x];
}

/**
 * Splits a contact-point velocity into rolling and roller sliding: v_c = r u ê_drive + s ŝ.
 * Returns the wheel speed u (rad/s) and the roller sliding speed s (m/s). Independent of `wheelRow`
 * (it solves the 2×2 system directly), so the tests can check one against the other.
 */
export function decomposeContact(w: Wheel, vc: P2): { u: number; s: number } {
  const g = wheelGeometry(w);
  // [r ê_drive, ŝ] [u; s] = v_c
  const a = w.r * g.drive[0], b = g.slide[0], c = w.r * g.drive[1], d = g.slide[1];
  const det = a * d - b * c;
  return { u: (vc[0] * d - b * vc[1]) / det, s: (a * vc[1] - c * vc[0]) / det };
}

// ---------------------------------------------------------------------------------------------
// The two bases of the lesson
// ---------------------------------------------------------------------------------------------

/**
 * Three omniwheels at distance d from the centre, 120° apart, each driving along the clockwise
 * tangent. Numbered clockwise from the top wheel: (0, d), (d√3/2, −d/2), (−d√3/2, −d/2).
 */
export function threeOmni(d: number, r: number): Wheel[] {
  return [90, -30, 210].map((deg) => {
    const a = (deg * Math.PI) / 180;
    return { x: d * Math.cos(a), y: d * Math.sin(a), beta: a - Math.PI / 2, gamma: 0, r };
  });
}

/**
 * Four mecanum wheels at (±l, ±w), all driving along x_b, numbered front-left, front-right,
 * rear-right, rear-left. Diagonal wheels share the roller angle: γ = (−g, +g, −g, +g).
 */
export function fourMecanum(l: number, w: number, r: number, g: number): Wheel[] {
  return [
    { x: l, y: w, beta: 0, gamma: -g, r },
    { x: l, y: -w, beta: 0, gamma: g, r },
    { x: -l, y: -w, beta: 0, gamma: -g, r },
    { x: -l, y: w, beta: 0, gamma: g, r },
  ];
}

// ---------------------------------------------------------------------------------------------
// Rank, inconsistent wheel speeds, least-squares chassis motion
// ---------------------------------------------------------------------------------------------

/** Numerical rank of H (singular values above tol · σ_max). */
export function rankOf(H: Mat, tol = 1e-9): number {
  const { S } = svd(H);
  const smax = S[0] ?? 0;
  return S.filter((s) => s > tol * Math.max(1, smax)).length;
}

/**
 * Orthonormal basis of the left null space of H: wheel-speed combinations that no chassis twist
 * can produce (they only make the wheels skid). Built from the projector I − H H⁺. Each vector is
 * signed so that its first clearly non-zero entry is positive.
 */
export function leftNullBasis(H: Mat, tol = 1e-8): Vec[] {
  const m = H.length;
  const HHp = matMul(H, pinv(H));
  const P = HHp.map((row, i) => row.map((x, j) => (i === j ? 1 : 0) - x));
  const basis: Vec[] = [];
  for (let j = 0; j < m; j++) {
    const v = P.map((row) => row[j]);
    for (const b of basis) {
      const p = b.reduce((acc, bi, k) => acc + bi * v[k], 0);
      for (let k = 0; k < m; k++) v[k] -= p * b[k];
    }
    const n = Math.hypot(...v);
    if (n > tol) basis.push(v.map((x) => x / n));
  }
  return basis.map((v) => {
    const k = v.findIndex((x) => Math.abs(x) > 1e-6);
    return k >= 0 && v[k] < 0 ? v.map((x) => -x) : v;
  });
}

export interface ChassisFit {
  /** Least-squares chassis twist H⁺ u. */
  V: Twist2;
  /** Per-wheel skid u − H V (rad/s); zero when u is consistent. */
  slip: Vec;
  /** Euclidean norm of `slip`. */
  slipNorm: number;
}

/**
 * Chassis motion produced by commanded wheel speeds. With m = 3 independent wheels H is invertible
 * and the fit is exact; with m = 4 a wheel-speed vector outside the column space of H cannot be
 * realised without skidding, and the least-squares twist is used as the model of what happens
 * (equal "cost" of skidding at every wheel). `Hp` is H⁺, passed in so it can be cached.
 */
export function chassisFromWheels(H: Mat, Hp: Mat, u: Vec): ChassisFit {
  const V = matVec(Hp, u) as Twist2;
  const slip = u.map((ui, i) => ui - (H[i][0] * V[0] + H[i][1] * V[1] + H[i][2] * V[2]));
  return { V: [V[0], V[1], V[2]], slip, slipNorm: Math.hypot(...slip) };
}

// ---------------------------------------------------------------------------------------------
// Wheel-speed limits: feasible twists, clamping and the (v_x, v_y) slice
// ---------------------------------------------------------------------------------------------

/** Largest |ω| reachable with every |u_i| ≤ U (pure spin); Infinity if no wheel limits it. */
export function omegaLimit(H: Mat, U: number): number {
  let lim = Infinity;
  for (const h of H) if (Math.abs(h[0]) > 1e-12) lim = Math.min(lim, U / Math.abs(h[0]));
  return lim;
}

/** Largest wheel speed |u_i| needed for V. */
export function maxWheelSpeed(H: Mat, V: Twist2): number {
  let m = 0;
  for (const h of H) m = Math.max(m, Math.abs(h[0] * V[0] + h[1] * V[1] + h[2] * V[2]));
  return m;
}

export interface ClampResult {
  V: Twist2;
  /** Factor applied to (v_x, v_y) (1 = untouched). */
  scale: number;
  /** True when ω had to be reduced to the pure-spin limit. */
  omegaClamped: boolean;
  clamped: boolean;
}

/**
 * Keeps a requested twist inside the feasible set {V : |H V|_∞ ≤ U}. ω is first limited to the
 * pure-spin range (so that (v_x, v_y) = 0 is feasible), then (v_x, v_y) is shrunk towards the
 * origin of the current ω-slice. This keeps the direction of travel and matches the slice plot.
 */
export function clampTwist(H: Mat, U: number, V: Twist2): ClampResult {
  const wLim = omegaLimit(H, U);
  let w = V[0];
  const omegaClamped = Math.abs(w) > wLim;
  if (omegaClamped) w = Math.sign(w) * wLim * (1 - 1e-12);
  const t = Math.min(1, rayLimit(H, U, w, [V[1], V[2]]));
  return { V: [w, t * V[1], t * V[2]], scale: t, omegaClamped, clamped: omegaClamped || t < 1 - 1e-12 };
}

/**
 * Largest t ≥ 0 such that (ω, t·dir) is feasible (Infinity when the ray never leaves the set).
 * Requires |H_i0 ω| ≤ U for every wheel (true after clampTwist's ω limit).
 */
export function rayLimit(H: Mat, U: number, omega: number, dir: P2): number {
  let t = Infinity;
  for (const h of H) {
    const a = h[0] * omega;
    const b = h[1] * dir[0] + h[2] * dir[1];
    if (Math.abs(b) < 1e-12) continue;
    const lim = b > 0 ? (U - a) / b : (-U - a) / b;
    t = Math.min(t, Math.max(0, lim));
  }
  return t;
}

/** Speed that is reachable in EVERY direction of the ω-slice (radius of the largest centred disc). */
export function isotropicSpeed(H: Mat, U: number, omega: number): number {
  let s = Infinity;
  for (const h of H) {
    const nb = Math.hypot(h[1], h[2]);
    if (nb < 1e-12) continue;
    s = Math.min(s, Math.max(0, U - Math.abs(h[0] * omega)) / nb);
  }
  return s;
}

function clipHalfPlane(poly: P2[], a: P2, c: number): P2[] {
  // keeps {v : a·v ≤ c} (Sutherland–Hodgman against one edge)
  const out: P2[] = [];
  for (let i = 0; i < poly.length; i++) {
    const P = poly[i], Q = poly[(i + 1) % poly.length];
    const fp = a[0] * P[0] + a[1] * P[1] - c;
    const fq = a[0] * Q[0] + a[1] * Q[1] - c;
    if (fp <= 0) out.push(P);
    if ((fp < 0 && fq > 0) || (fp > 0 && fq < 0)) {
      const t = fp / (fp - fq);
      out.push([P[0] + t * (Q[0] - P[0]), P[1] + t * (Q[1] - P[1])]);
    }
  }
  return out;
}

/**
 * The ω-slice of the feasible set, {(v_x, v_y) : |H (ω, v_x, v_y)|_∞ ≤ U}, intersected with the
 * square |v_x|, |v_y| ≤ box (the slice is unbounded when some direction loads no wheel).
 * Returns the polygon counter-clockwise; empty when ω itself is infeasible.
 */
export function sliceAt(H: Mat, U: number, omega: number, box: number): P2[] {
  let poly: P2[] = [[-box, -box], [box, -box], [box, box], [-box, box]];
  for (const h of H) {
    const a = h[0] * omega;
    const b: P2 = [h[1], h[2]];
    if (Math.hypot(b[0], b[1]) < 1e-12) {
      if (Math.abs(a) > U) return [];
      continue;
    }
    poly = clipHalfPlane(poly, b, U - a);
    poly = clipHalfPlane(poly, [-b[0], -b[1]], U + a);
    if (poly.length < 3) return [];
  }
  return poly;
}

/** True when the slice reaches the bounding box (some direction is not limited by any wheel). */
export function sliceUnbounded(H: Mat): boolean {
  return rankOf(H.map((h) => [h[1], h[2]])) < 2;
}

// ---------------------------------------------------------------------------------------------
// Integration on SE(2)
// ---------------------------------------------------------------------------------------------

/** Displacement exp([V] t) in the starting body frame, evaluated with MatrixExp6. */
export function expTwist2(V: Twist2, t: number): Pose {
  const T = MatrixExp6(VecTose3([0, 0, V[0] * t, V[1] * t, V[2] * t, 0]));
  return { phi: V[0] * t, x: T[0][3], y: T[1][3] };
}

/** Moves a pose with the constant body twist V for t seconds: T ← T exp([V] t). */
export function stepPose(q: Pose, V: Twist2, t: number): Pose {
  const d = expTwist2(V, t);
  const c = Math.cos(q.phi), s = Math.sin(q.phi);
  return { phi: q.phi + d.phi, x: q.x + c * d.x - s * d.y, y: q.y + s * d.x + c * d.y };
}

/** q̇ = (φ̇, ẋ, ẏ) from the body twist: the body velocity rotated by φ. */
export function qdotFromBody(phi: number, V: Twist2): Twist2 {
  const c = Math.cos(phi), s = Math.sin(phi);
  return [V[0], c * V[1] - s * V[2], s * V[1] + c * V[2]];
}

/** Body twist from q̇ (inverse of qdotFromBody). */
export function bodyFromQdot(phi: number, qd: Twist2): Twist2 {
  const c = Math.cos(phi), s = Math.sin(phi);
  return [qd[0], c * qd[1] + s * qd[2], -s * qd[1] + c * qd[2]];
}
