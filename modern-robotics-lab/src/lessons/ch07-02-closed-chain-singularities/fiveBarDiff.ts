/**
 * Differential kinematics and singularities of the planar five-bar (ch07-02 lesson).
 *
 * Geometry, IK and FK are a trimmed copy of the ch07-01 helper (lessons may not import each other):
 *   motors at A1 = (-d/2, 0) and A2 = (d/2, 0); left leg L1 (motor th1) + L2, right leg L3 (motor th2) + L4;
 *   elbow angles phi1, phi2 are measured relative to the proximal links; the distal links meet at the tip P.
 *
 * Velocity analysis. With actuated angles θa = (th1, th2) and passive angles θp = (phi1, phi2), the loop
 * closure g(θ) = p_L(th1, phi1) - p_R(th2, phi2) = 0 differentiates to H_a θ̇a + H_p θ̇p = 0 with
 *   H_a = [ perp(P - A1), -perp(P - A2) ],   H_p = [ perp(P - B1), -perp(P - B2) ],   perp(w) = (-w_y, w_x).
 * Through the left leg v = J_a θ̇a + J_p θ̇p with J_a = [perp(P - A1), 0], J_p = [perp(P - B1), 0], so
 *   v = J θ̇a,   J = J_a - J_p H_p^{-1} H_a          (needs det H_p != 0).
 * Per-leg form: |P - B_i| = const gives A v = B θ̇a with A = [(P - B1)ᵀ; (P - B2)ᵀ] and
 *   B = diag(L1 L2 sin phi1, L3 L4 sin phi2),  hence J = A^{-1} B,  det A = -det H_p.
 *   det H_p = 0  (distal links collinear)       → actuator singularity (J blows up),
 *   sin phi_i = 0 (a leg straight or folded)    → end-effector singularity (J loses rank).
 * The inverse map θ̇a = K v, K = B^{-1} A, stays finite at actuator singularities, so ellipses are built from K.
 * Redundant actuation (third motor at the left elbow, θa = (th1, phi1, th2)) uses K3 (3x2).
 */
import { matMul, matVec, pinv, solve, svd, wrapToPi, planarFK } from '@/mr';

export type Pt = [number, number];
export type Side = 0 | 1;
export type Mode = 1 | -1;
type Mat = number[][];

export interface FiveBarGeom { L1: number; L2: number; L3: number; L4: number; d: number }
export const DEFAULT_GEOM: Readonly<FiveBarGeom> = { L1: 1, L2: 1.3, L3: 1, L4: 1.3, d: 1 };

export interface Config {
  th1: number;
  th2: number;
  phi1: number;
  phi2: number;
  P: Pt;
  B1: Pt;
  B2: Pt;
  /** +1 when the tip is left of the directed line B1 -> B2. */
  mode: Mode;
}

const cross = (a: Pt, b: Pt): number => a[0] * b[1] - a[1] * b[0];
const sub = (a: Pt, b: Pt): Pt => [a[0] - b[0], a[1] - b[1]];
const perp = (w: Pt): Pt => [-w[1], w[0]];
const dist2 = (a: Pt, b: Pt): number => (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2;

export function basePoint(g: FiveBarGeom, side: Side): Pt {
  return side === 0 ? [-g.d / 2, 0] : [g.d / 2, 0];
}
export function legLinks(g: FiveBarGeom, side: Side): [number, number] {
  return side === 0 ? [g.L1, g.L2] : [g.L3, g.L4];
}
export function elbowPoint(g: FiveBarGeom, side: Side, theta: number): Pt {
  const A = basePoint(g, side);
  const Lp = legLinks(g, side)[0];
  return [A[0] + Lp * Math.cos(theta), A[1] + Lp * Math.sin(theta)];
}

export function assembleConfig(g: FiveBarGeom, th1: number, th2: number, P: Pt): Config {
  const B1 = elbowPoint(g, 0, th1);
  const B2 = elbowPoint(g, 1, th2);
  const a1 = Math.atan2(P[1] - B1[1], P[0] - B1[0]);
  const a2 = Math.atan2(P[1] - B2[1], P[0] - B2[0]);
  return {
    th1: wrapToPi(th1),
    th2: wrapToPi(th2),
    phi1: wrapToPi(a1 - th1),
    phi2: wrapToPi(a2 - th2),
    P: [P[0], P[1]],
    B1,
    B2,
    mode: cross(sub(B2, B1), sub(P, B1)) >= 0 ? 1 : -1,
  };
}

/** Loop-closure residual p_left - p_right from the four joint angles (legs as open 2R chains, planarFK from @/mr). */
export function loopResidual(g: FiveBarGeom, c: Config): Pt {
  const A1 = basePoint(g, 0);
  const A2 = basePoint(g, 1);
  const l = planarFK([g.L1, g.L2], [c.th1, c.phi1]).tip;
  const r = planarFK([g.L3, g.L4], [c.th2, c.phi2]).tip;
  return [A1[0] + l[0] - A2[0] - r[0], A1[1] + l[1] - A2[1] - r[1]];
}

// ---------------------------------------------------------------------------
// FK (circle intersection) with assembly-mode tracking
// ---------------------------------------------------------------------------

export function fkAll(g: FiveBarGeom, th1: number, th2: number): { status: 'two' | 'tangent' | 'none' | 'coincident'; solutions: Config[]; B1: Pt; B2: Pt } {
  const B1 = elbowPoint(g, 0, th1);
  const B2 = elbowPoint(g, 1, th2);
  const r1 = g.L2;
  const r2 = g.L4;
  const dx = B2[0] - B1[0];
  const dy = B2[1] - B1[1];
  const Dd = Math.hypot(dx, dy);
  const margin = Math.min(r1 + r2 - Dd, Dd - Math.abs(r1 - r2));
  if (Dd < 1e-12) return { status: Math.abs(r1 - r2) < 1e-12 ? 'coincident' : 'none', solutions: [], B1, B2 };
  if (margin < -1e-10) return { status: 'none', solutions: [], B1, B2 };
  const ux = dx / Dd;
  const uy = dy / Dd;
  const a = (r1 * r1 - r2 * r2 + Dd * Dd) / (2 * Dd);
  const mx = B1[0] + a * ux;
  const my = B1[1] + a * uy;
  const h2 = r1 * r1 - a * a;
  if (margin <= 1e-10 || h2 <= 0) return { status: 'tangent', solutions: [assembleConfig(g, th1, th2, [mx, my])], B1, B2 };
  const h = Math.sqrt(h2);
  const pts: Pt[] = [[mx - h * uy, my + h * ux], [mx + h * uy, my - h * ux]];
  return { status: 'two', solutions: pts.map((P) => assembleConfig(g, th1, th2, P)), B1, B2 };
}

export function fkNearest(g: FiveBarGeom, th1: number, th2: number, prev: Pt | null, preferMode: Mode = 1): Config | null {
  const res = fkAll(g, th1, th2);
  if (res.status === 'coincident') {
    const B = res.B1;
    const ref: Pt = prev ?? [B[0], B[1] + 1];
    const n = Math.hypot(ref[0] - B[0], ref[1] - B[1]) || 1;
    return assembleConfig(g, th1, th2, [B[0] + (g.L2 * (ref[0] - B[0])) / n, B[1] + (g.L2 * (ref[1] - B[1])) / n]);
  }
  if (!res.solutions.length) return null;
  if (prev) return res.solutions.reduce((best, s) => (dist2(s.P, prev) < dist2(best.P, prev) ? s : best));
  return res.solutions.find((s) => s.mode === preferMode) ?? res.solutions[0];
}

/** The FK solution with a given assembly mode (the single one at tangency). */
export function fkMode(g: FiveBarGeom, th1: number, th2: number, mode: Mode): Config | null {
  const res = fkAll(g, th1, th2);
  if (res.status === 'tangent') return res.solutions[0];
  return res.solutions.find((s) => s.mode === mode) ?? null;
}

export function bisectBoundary(ok: (s: number) => boolean, lo = 0, hi = 1, iters = 48): number {
  let a = lo;
  let b = hi;
  for (let k = 0; k < iters; k++) {
    const m = 0.5 * (a + b);
    if (ok(m)) a = m;
    else b = m;
  }
  return a;
}

export interface DriveResult { cfg: Config; blocked: boolean }

/** Turn the motors towards (th1, th2) in small sub-steps following the assembly mode; stop at tangency. */
export function driveMotors(g: FiveBarGeom, from: Config, th1: number, th2: number, maxStep = 0.01): DriveResult {
  const d1 = wrapToPi(th1 - from.th1);
  const d2 = wrapToPi(th2 - from.th2);
  const n = Math.max(1, Math.ceil(Math.max(Math.abs(d1), Math.abs(d2)) / maxStep));
  let cur = from;
  for (let k = 1; k <= n; k++) {
    const s = k / n;
    const next = fkNearest(g, from.th1 + s * d1, from.th2 + s * d2, cur.P);
    if (next) { cur = next; continue; }
    const base = cur;
    const sb = bisectBoundary((t) => fkAll(g, from.th1 + t * d1, from.th2 + t * d2).status !== 'none', (k - 1) / n, s);
    return { cfg: fkNearest(g, from.th1 + sb * d1, from.th2 + sb * d2, base.P) ?? base, blocked: true };
  }
  return { cfg: cur, blocked: false };
}

// ---------------------------------------------------------------------------
// IK with elbow continuity (each leg keeps the elbow nearest to its previous one)
// ---------------------------------------------------------------------------

function legElbows(g: FiveBarGeom, side: Side, P: Pt): Pt[] | null {
  const A = basePoint(g, side);
  const [Lp, Ld] = legLinks(g, side);
  // elbow = intersection of circle(A, Lp) and circle(P, Ld)
  const dx = P[0] - A[0];
  const dy = P[1] - A[1];
  const r = Math.hypot(dx, dy);
  if (r > Lp + Ld + 1e-12 || r < Math.abs(Lp - Ld) - 1e-12 || r < 1e-12) return null;
  const a = (Lp * Lp - Ld * Ld + r * r) / (2 * r);
  const h = Math.sqrt(Math.max(0, Lp * Lp - a * a));
  const ux = dx / r;
  const uy = dy / r;
  const mx = A[0] + a * ux;
  const my = A[1] + a * uy;
  return [[mx - h * uy, my + h * ux], [mx + h * uy, my - h * ux]];
}

/** IK keeping each elbow on the side it was on (nearest elbow). Null when the tip is unreachable. */
export function ikNearest(g: FiveBarGeom, P: Pt, prev: Config): Config | null {
  const e1 = legElbows(g, 0, P);
  const e2 = legElbows(g, 1, P);
  if (!e1 || !e2) return null;
  const B1 = dist2(e1[0], prev.B1) <= dist2(e1[1], prev.B1) ? e1[0] : e1[1];
  const B2 = dist2(e2[0], prev.B2) <= dist2(e2[1], prev.B2) ? e2[0] : e2[1];
  const A1 = basePoint(g, 0);
  const A2 = basePoint(g, 1);
  return assembleConfig(g, Math.atan2(B1[1] - A1[1], B1[0] - A1[0]), Math.atan2(B2[1] - A2[1], B2[0] - A2[0]), P);
}

/** Move the tip in small steps (elbow continuity); stop on the workspace boundary. */
export function driveTip(g: FiveBarGeom, from: Config, target: Pt, maxStep = 0.01): DriveResult {
  const dx = target[0] - from.P[0];
  const dy = target[1] - from.P[1];
  const n = Math.max(1, Math.ceil(Math.hypot(dx, dy) / maxStep));
  let cur = from;
  const at = (s: number): Pt => [from.P[0] + s * dx, from.P[1] + s * dy];
  for (let k = 1; k <= n; k++) {
    const next = ikNearest(g, at(k / n), cur);
    if (next) { cur = next; continue; }
    const base = cur;
    const sb = bisectBoundary((t) => ikNearest(g, at(t), base) !== null, (k - 1) / n, k / n);
    return { cfg: ikNearest(g, at(sb), base) ?? base, blocked: true };
  }
  return { cfg: cur, blocked: false };
}

// ---------------------------------------------------------------------------
// Velocity kinematics
// ---------------------------------------------------------------------------

export interface ConstraintJacobians {
  /** ∂g/∂θa (2x2; 2x3 with the third motor: columns th1, phi1, th2). */
  Ha: Mat;
  /** ∂g/∂θp (2x2; 2x1 with the third motor: column phi2). */
  Hp: Mat;
  /** Tip velocity through the left leg: v = Ja θ̇a + Jp θ̇p. */
  Ja: Mat;
  Jp: Mat;
}

const cols = (...c: Pt[]): Mat => [c.map((v) => v[0]), c.map((v) => v[1])];

export function constraintJacobians(g: FiveBarGeom, c: Config, redundant = false): ConstraintJacobians {
  const A1 = basePoint(g, 0);
  const A2 = basePoint(g, 1);
  const tA1 = perp(sub(c.P, A1));
  const tB1 = perp(sub(c.P, c.B1));
  const tA2 = perp(sub(c.P, A2));
  const tB2 = perp(sub(c.P, c.B2));
  const neg = (v: Pt): Pt => [-v[0], -v[1]];
  if (!redundant) {
    return { Ha: cols(tA1, neg(tA2)), Hp: cols(tB1, neg(tB2)), Ja: cols(tA1, [0, 0]), Jp: cols(tB1, [0, 0]) };
  }
  return { Ha: cols(tA1, tB1, neg(tA2)), Hp: cols(neg(tB2)), Ja: cols(tA1, tB1, [0, 0]), Jp: cols([0, 0]) };
}

export function det2(M: Mat): number {
  return M[0][0] * M[1][1] - M[0][1] * M[1][0];
}

/** Passive-rate map −Hp^{-1} Ha (θ̇p = −Hp^{-1} Ha θ̇a, two motors). Null when Hp is singular. */
export function passiveRateMap(g: FiveBarGeom, c: Config, tol = 1e-12): Mat | null {
  const { Ha, Hp } = constraintJacobians(g, c);
  if (Math.abs(det2(Hp)) < tol) return null;
  const X0 = solve(Hp, [Ha[0][0], Ha[1][0]]);
  const X1 = solve(Hp, [Ha[0][1], Ha[1][1]]);
  return [[-X0[0], -X1[0]], [-X0[1], -X1[1]]];
}

/** End-effector Jacobian J = Ja - Jp Hp^{-1} Ha (two motors). Null when Hp is singular. */
export function eeJacobian(g: FiveBarGeom, c: Config, tol = 1e-12): Mat | null {
  const { Ha, Hp, Ja, Jp } = constraintJacobians(g, c);
  if (Math.abs(det2(Hp)) < tol) return null;
  // Hp^{-1} Ha column by column (solve from @/mr)
  const X0 = solve(Hp, [Ha[0][0], Ha[1][0]]);
  const X1 = solve(Hp, [Ha[0][1], Ha[1][1]]);
  const X: Mat = [[X0[0], X1[0]], [X0[1], X1[1]]];
  const JpX = matMul(Jp, X);
  return [[Ja[0][0] - JpX[0][0], Ja[0][1] - JpX[0][1]], [Ja[1][0] - JpX[1][0], Ja[1][1] - JpX[1][1]]];
}

/** Per-leg form A v = B θ̇a (two motors). */
export function perLegForm(g: FiveBarGeom, c: Config): { A: Mat; Bdiag: [number, number] } {
  const a = sub(c.P, c.B1);
  const b = sub(c.P, c.B2);
  return { A: [[a[0], a[1]], [b[0], b[1]]], Bdiag: [g.L1 * g.L2 * Math.sin(c.phi1), g.L3 * g.L4 * Math.sin(c.phi2)] };
}

/** Normalised indicators in [-1, 1]: det Hp / (L2 L4) for actuator singularities, sin phi_i for end-effector ones. */
export function indicators(g: FiveBarGeom, c: Config): { act: number; s1: number; s2: number } {
  return { act: det2(constraintJacobians(g, c).Hp) / (g.L2 * g.L4), s1: Math.sin(c.phi1), s2: Math.sin(c.phi2) };
}

const clampSin = (s: number): number => (Math.abs(s) < 1e-9 ? (s < 0 ? -1e-9 : 1e-9) : s);

/**
 * Inverse velocity map θ̇a = K v (motor rates needed for a tip velocity). 2x2 for two motors (K = B^{-1} A),
 * 3x2 with the third motor at the left elbow (rows th1, phi1, th2). sin phi is clamped away from 0 so K stays
 * finite (but huge) exactly at an end-effector singularity.
 */
export function inverseRates(g: FiveBarGeom, c: Config, redundant = false): Mat {
  const A1 = basePoint(g, 0);
  const a = sub(c.P, c.B1);
  const b = sub(c.P, c.B2);
  const q = sub(c.P, A1);
  const k1 = 1 / (g.L1 * g.L2 * clampSin(Math.sin(c.phi1)));
  const k2 = 1 / (g.L3 * g.L4 * clampSin(Math.sin(c.phi2)));
  const rowTh1 = [k1 * a[0], k1 * a[1]];
  const rowTh2 = [k2 * b[0], k2 * b[1]];
  if (!redundant) return [rowTh1, rowTh2];
  return [rowTh1, [-k1 * q[0], -k1 * q[1]], rowTh2];
}

export interface Ellipse {
  /** Semi-axes (major first), in m/s of tip speed per 1 rad/s of motor-rate norm; Infinity when unbounded. */
  r: [number, number];
  /** Unit directions of the semi-axes. */
  dirs: [Pt, Pt];
}

/** Tip velocity ellipse { v : |K v| <= 1 } from the SVD of K (@/mr): semi-axes 1/σ_i along V's columns. */
export function velocityEllipse(K: Mat): Ellipse {
  const { S, V } = svd(K);
  const sMax = S[0];
  const sMin = S[1];
  const rel = sMin / (sMax || 1);
  const big = rel < 1e-12 ? Infinity : 1 / sMin;
  return { r: [big, sMax > 0 ? 1 / sMax : Infinity], dirs: [[V[0][1], V[1][1]], [V[0][0], V[1][0]]] };
}

/** Finite-difference Jacobian dP/dθa by FK inside the current assembly mode (null if not assemblable nearby). */
export function jacobianFD(g: FiveBarGeom, c: Config, h = 1e-6): Mat | null {
  const out: Pt[] = [];
  for (const m of [0, 1]) {
    const f = fkNearest(g, c.th1 + (m === 0 ? h : 0), c.th2 + (m === 1 ? h : 0), c.P);
    const b = fkNearest(g, c.th1 - (m === 0 ? h : 0), c.th2 - (m === 1 ? h : 0), c.P);
    if (f && b) out.push([(f.P[0] - b.P[0]) / (2 * h), (f.P[1] - b.P[1]) / (2 * h)]);
    else if (f) out.push([(f.P[0] - c.P[0]) / h, (f.P[1] - c.P[1]) / h]);
    else if (b) out.push([(c.P[0] - b.P[0]) / h, (c.P[1] - b.P[1]) / h]);
    else return null;
  }
  return [[out[0][0], out[1][0]], [out[0][1], out[1][1]]];
}

/** Tip velocity for motor rates (two motors), via the per-leg form; null when A is singular (unbounded). */
export function tipVelocity(g: FiveBarGeom, c: Config, rates: [number, number]): Pt | null {
  const { A, Bdiag } = perLegForm(g, c);
  if (Math.abs(det2(A)) < 1e-12 * (g.L2 * g.L4)) return null;
  const v = solve(A, [Bdiag[0] * rates[0], Bdiag[1] * rates[1]]);
  return [v[0], v[1]];
}

/**
 * Minimum-norm motor torques that balance a force f exerted at the tip (power balance τᵀθ̇a = fᵀv ⇒ Kᵀτ = f).
 * `exact` is false when no finite torque can balance f (actuator singularity).
 */
export function holdingTorques(g: FiveBarGeom, c: Config, f: Pt, redundant = false): { tau: number[]; exact: boolean } {
  const K = inverseRates(g, c, redundant);
  const KT = K[0].map((_, j) => K.map((row) => row[j]));
  const tau = matVec(pinv(KT, 1e-14), [f[0], f[1]]);
  const back = matVec(KT, tau);
  const err = Math.hypot(back[0] - f[0], back[1] - f[1]);
  return { tau, exact: err < 1e-6 * Math.max(1, Math.hypot(f[0], f[1])) };
}

// ---------------------------------------------------------------------------
// Locked motors: what does a virtual tip displacement cost?
// ---------------------------------------------------------------------------

export interface LockTest {
  /** Stretch of the left / right distal link (m) needed to put the tip at the virtual point. */
  e1: number;
  e2: number;
  /** With the third motor the left leg is rigid: e1 is then the distance from the locked tip. */
  rigidLeft: boolean;
}

export function lockStretch(g: FiveBarGeom, c: Config, Pv: Pt, redundant = false): LockTest {
  const e2 = Math.hypot(Pv[0] - c.B2[0], Pv[1] - c.B2[1]) - g.L4;
  if (redundant) return { e1: Math.hypot(Pv[0] - c.P[0], Pv[1] - c.P[1]), e2, rigidLeft: true };
  return { e1: Math.hypot(Pv[0] - c.B1[0], Pv[1] - c.B1[1]) - g.L2, e2, rigidLeft: false };
}

/**
 * Softest tip direction with the motors locked: first-order stretch per unit displacement is |Â δ| with
 * Â = rows of unit vectors along the distal links (two motors) or [I; â2ᵀ] (third motor locked too).
 */
export function softestDirection(g: FiveBarGeom, c: Config, redundant = false): { n: Pt; sigma: number } {
  const a = sub(c.P, c.B1);
  const b = sub(c.P, c.B2);
  const na = Math.hypot(a[0], a[1]) || 1;
  const nb = Math.hypot(b[0], b[1]) || 1;
  const rows: Mat = redundant ? [[1, 0], [0, 1], [b[0] / nb, b[1] / nb]] : [[a[0] / na, a[1] / na], [b[0] / nb, b[1] / nb]];
  const { S, V } = svd(rows);
  return { n: [V[0][1], V[1][1]], sigma: S[1] };
}

// ---------------------------------------------------------------------------
// Sweeps, presets and the snap-through path
// ---------------------------------------------------------------------------

/** Sweep th1 with th2 fixed from the current configuration both ways until assembly fails (or ±π). */
export function motorSlice(g: FiveBarGeom, c: Config, step = 0.01, span = Math.PI): { th1: number[]; configs: Config[]; blockedLo: boolean; blockedHi: boolean } {
  const walk = (dir: 1 | -1) => {
    const out: Config[] = [];
    let cur = c;
    for (let k = 1; k <= Math.ceil(span / step); k++) {
      const r = driveMotors(g, cur, cur.th1 + dir * step, cur.th2, step / 4);
      out.push(r.cfg);
      if (r.blocked) return { out, blocked: true };
      cur = r.cfg;
    }
    return { out, blocked: false };
  };
  const lo = walk(-1);
  const hi = walk(1);
  const configs = [...lo.out.reverse(), c, ...hi.out];
  // unwrapped angles, monotonic
  const th1: number[] = [];
  let prev = configs[0].th1;
  let acc = configs[0].th1;
  for (const q of configs) {
    acc += wrapToPi(q.th1 - prev);
    prev = q.th1;
    th1.push(acc);
  }
  const shift = c.th1 - th1[lo.out.length];
  return { th1: th1.map((t) => t + shift), configs, blockedLo: lo.blocked, blockedHi: hi.blocked };
}

/** Symmetric pose th1 = π - th2 with a given gap to tangency (symmetric geometry only). */
export function symmetricMarginAngle(g: FiveBarGeom, margin: number): number | null {
  if (Math.abs(g.L1 - g.L3) > 1e-9 || Math.abs(g.L2 - g.L4) > 1e-9) return null;
  const cth = (2 * g.L2 - margin - g.d) / (2 * g.L1);
  if (cth < -1 || cth > 1) return null;
  return Math.acos(cth);
}

/** Configuration with leg `side` fully stretched along direction alpha (null if the other leg cannot reach). */
export function straightLegConfig(g: FiveBarGeom, side: Side, alpha: number, otherOuter = true): Config | null {
  const A = basePoint(g, side);
  const [Lp, Ld] = legLinks(g, side);
  const P: Pt = [A[0] + (Lp + Ld) * Math.cos(alpha), A[1] + (Lp + Ld) * Math.sin(alpha)];
  const other: Side = side === 0 ? 1 : 0;
  const el = legElbows(g, other, P);
  if (!el) return null;
  const Ao = basePoint(g, other);
  // "outer" elbow: left leg → left of A1→P (first returned), right leg → right of A2→P (second returned)
  const Bo = other === 0 ? (otherOuter ? el[0] : el[1]) : (otherOuter ? el[1] : el[0]);
  const thO = Math.atan2(Bo[1] - Ao[1], Bo[0] - Ao[0]);
  return side === 0 ? assembleConfig(g, alpha, thO, P) : assembleConfig(g, thO, alpha, P);
}

/**
 * A configuration that is singular in both ways at once: the left leg folded (phi1 = π) and the two distal
 * links collinear (B1, P, B2 on one line, B2 beyond P). Found by solving |B2 - A2| = L3 for the folding
 * direction (bisection); null if the geometry has no such pose.
 */
export function doubleSingularConfig(g: FiveBarGeom): Config | null {
  const A1 = basePoint(g, 0);
  const A2 = basePoint(g, 1);
  const f = (al: number): number => {
    const u: Pt = [Math.cos(al), Math.sin(al)];
    // folded left leg: P = A1 + (L1 - L2) u, distal link from B1 back through P; B2 continues past P
    const B2: Pt = [A1[0] + (g.L1 - g.L2 - g.L4) * u[0], A1[1] + (g.L1 - g.L2 - g.L4) * u[1]];
    return Math.hypot(B2[0] - A2[0], B2[1] - A2[1]) - g.L3;
  };
  // scan the upper half-plane for a sign change of f
  let prevA = Math.PI / 2;
  let prevF = f(prevA);
  for (let k = 1; k <= 360; k++) {
    const al = Math.PI / 2 + (k * Math.PI) / 360;
    const fa = f(al);
    if (Math.sign(fa) !== Math.sign(prevF)) {
      let lo = prevA;
      let hi = al;
      for (let it = 0; it < 80; it++) {
        const m = 0.5 * (lo + hi);
        if (Math.sign(f(m)) === Math.sign(f(lo))) lo = m; else hi = m;
      }
      const a = 0.5 * (lo + hi);
      const u: Pt = [Math.cos(a), Math.sin(a)];
      const P: Pt = [A1[0] + (g.L1 - g.L2) * u[0], A1[1] + (g.L1 - g.L2) * u[1]];
      const B2: Pt = [A1[0] + (g.L1 - g.L2 - g.L4) * u[0], A1[1] + (g.L1 - g.L2 - g.L4) * u[1]];
      return assembleConfig(g, a, Math.atan2(B2[1] - A2[1], B2[0] - A2[0]), P);
    }
    prevA = al;
    prevF = fa;
  }
  return null;
}

export interface SnapPath {
  th1Start: number;
  th1Edge: number;
  th2: number;
  startMode: Mode;
  /** Motor speed (rad/s) and the time to reach the singular configuration. */
  omega: number;
  T1: number;
}

/**
 * Motor 1 turns at constant speed from th1Start until the distal links become collinear (tangency) and back.
 * Returns null if no tangency is met within half a turn.
 */
export function buildSnapPath(g: FiveBarGeom, start: Config, omega: number): SnapPath | null {
  const r = driveMotors(g, start, start.th1 + Math.PI * 0.999, start.th2, 0.005);
  if (!r.blocked) return null;
  const d = r.cfg.th1 - start.th1;
  const dUnwrapped = d < 0 ? d + 2 * Math.PI : d;
  return { th1Start: start.th1, th1Edge: start.th1 + dUnwrapped, th2: start.th2, startMode: start.mode, omega, T1: dUnwrapped / omega };
}

/** Configuration at time t on the snap path (flip = come back in the other assembly mode). */
export function snapConfig(g: FiveBarGeom, p: SnapPath, t: number, flip: boolean): { cfg: Config | null; th1: number; rate: number; phase: 0 | 1 } {
  const tt = Math.max(0, Math.min(2 * p.T1, t));
  const phase: 0 | 1 = tt <= p.T1 ? 0 : 1;
  const th1 = phase === 0 ? p.th1Start + p.omega * tt : p.th1Start + p.omega * (2 * p.T1 - tt);
  const mode: Mode = phase === 0 || !flip ? p.startMode : ((-p.startMode) as Mode);
  return { cfg: fkMode(g, th1, p.th2, mode), th1, rate: phase === 0 ? p.omega : -p.omega, phase };
}
