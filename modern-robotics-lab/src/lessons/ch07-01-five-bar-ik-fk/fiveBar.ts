/**
 * Planar five-bar (5R) closed chain for the ch07-01 lesson, plus the planar 3xRPR platform
 * used in the last step.
 *
 * Five-bar layout (all joints revolute, all angles in radians, CCW positive):
 *   - two motors on the ground at A1 = (-d/2, 0) (left) and A2 = (d/2, 0) (right);
 *   - left leg:  proximal link L1 (motor angle th1 measured from +x), distal link L2;
 *   - right leg: proximal link L3 (motor angle th2 measured from +x), distal link L4;
 *   - the distal links meet at the tip P (a passive joint); the elbows B1, B2 are passive too.
 *   - phi1, phi2 are the elbow angles of each leg (distal link relative to proximal link).
 * Seen as two open 2R legs that must end at the same point, the loop-closure equation is
 *   g(th1, th2, phi1, phi2) = p_left(th1, phi1) - p_right(th2, phi2) = 0.
 * IK (tip -> motors) splits into two independent 2R problems (`planar2RIK` from @/mr); FK
 * (motors -> tip) is the intersection of the circles |P - B1| = L2 and |P - B2| = L4.
 */
import { planar2RIK, planarFK, svd, wrapToPi } from '@/mr';

export type Pt = [number, number];
export type Side = 0 | 1;
/** Elbow side of one leg: +1 = "outer" elbow, -1 = "inner" elbow (see `elbowSideOf`). */
export type ElbowSide = 1 | -1;
/** Assembly mode: +1 when the tip lies to the left of the directed line B1 -> B2 (above it in the usual pose). */
export type Mode = 1 | -1;
/** IK branch = elbow side of the left leg and of the right leg. */
export type Branch = [ElbowSide, ElbowSide];

export interface FiveBarGeom {
  /** Left proximal (motor) link. */
  L1: number;
  /** Left distal link. */
  L2: number;
  /** Right proximal (motor) link. */
  L3: number;
  /** Right distal link. */
  L4: number;
  /** Distance between the two motors (the ground link). */
  d: number;
}

export const DEFAULT_GEOM: Readonly<FiveBarGeom> = { L1: 1, L2: 1.3, L3: 1, L4: 1.3, d: 1 };

/** The four IK branch combinations, outer/outer first. */
export const BRANCHES: readonly Branch[] = [[1, 1], [1, -1], [-1, 1], [-1, -1]];

/** A fully assembled five-bar configuration (all five joint angles plus the key points). */
export interface Config {
  th1: number;
  th2: number;
  phi1: number;
  phi2: number;
  /** Relative angle of the right distal link w.r.t. the left one at the tip; 0 or ±π ⇔ distal links collinear. */
  psi: number;
  P: Pt;
  B1: Pt;
  B2: Pt;
  mode: Mode;
}

const dist2 = (a: Pt, b: Pt): number => (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2;
const cross = (a: Pt, b: Pt): number => a[0] * b[1] - a[1] * b[0];

export function basePoint(g: FiveBarGeom, side: Side): Pt {
  return side === 0 ? [-g.d / 2, 0] : [g.d / 2, 0];
}

/** [proximal, distal] link lengths of a leg. */
export function legLinks(g: FiveBarGeom, side: Side): [number, number] {
  return side === 0 ? [g.L1, g.L2] : [g.L3, g.L4];
}

export function elbowPoint(g: FiveBarGeom, side: Side, theta: number): Pt {
  const A = basePoint(g, side);
  const Lp = legLinks(g, side)[0];
  return [A[0] + Lp * Math.cos(theta), A[1] + Lp * Math.sin(theta)];
}

/** A leg treated as an open 2R chain (forward kinematics from @/mr, shifted to the leg's base). */
export function legPoints(g: FiveBarGeom, side: Side, theta: number, phi: number): { base: Pt; elbow: Pt; tip: Pt } {
  const A = basePoint(g, side);
  const fk = planarFK(legLinks(g, side), [theta, phi]);
  return {
    base: A,
    elbow: [A[0] + fk.joints[1][0], A[1] + fk.joints[1][1]],
    tip: [A[0] + fk.tip[0], A[1] + fk.tip[1]],
  };
}

/** Loop-closure residual g = p_left(th1, phi1) - p_right(th2, phi2) (zero for an assembled mechanism). */
export function loopResidual(g: FiveBarGeom, q: { th1: number; th2: number; phi1: number; phi2: number }): Pt {
  const l = legPoints(g, 0, q.th1, q.phi1).tip;
  const r = legPoints(g, 1, q.th2, q.phi2).tip;
  return [l[0] - r[0], l[1] - r[1]];
}

/**
 * Elbow side from the elbow angle. "Outer" (+1) means: left leg elbow on the left of the chord
 * A1 -> P (phi1 < 0), right leg elbow on the right of the chord A2 -> P (phi2 > 0). With the tip
 * above the ground line this is the familiar "both elbows pointing outwards" pose.
 * A straight or folded leg (sin phi = 0) counts as +1.
 */
export function elbowSideOf(side: Side, phi: number): ElbowSide {
  const s = side === 0 ? -Math.sin(phi) : Math.sin(phi);
  return s >= 0 ? 1 : -1;
}

export function modeOf(B1: Pt, B2: Pt, P: Pt): Mode {
  return cross([B2[0] - B1[0], B2[1] - B1[1]], [P[0] - B1[0], P[1] - B1[1]]) >= 0 ? 1 : -1;
}

/** Build the full configuration from the two motor angles and the tip position (passive angles follow). */
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
    psi: wrapToPi(a2 - a1),
    P: [P[0], P[1]],
    B1,
    B2,
    mode: modeOf(B1, B2, P),
  };
}

// ---------------------------------------------------------------------------
// Inverse kinematics: two independent 2R legs
// ---------------------------------------------------------------------------

export interface LegSolution {
  theta: number;
  phi: number;
  elbow: Pt;
  side: ElbowSide;
}

export interface LegIK {
  reachable: boolean;
  /** Distance from the leg's base to the tip. */
  r: number;
  /** Outer (+1) solution first, then inner (-1); both entries are the same pose on the leg's boundary. */
  sols: LegSolution[];
}

export function legIK(g: FiveBarGeom, side: Side, P: Pt): LegIK {
  const A = basePoint(g, side);
  const [Lp, Ld] = legLinks(g, side);
  const ik = planar2RIK(Lp, Ld, P[0] - A[0], P[1] - A[1]);
  if (!ik.reachable) return { reachable: false, r: ik.distance, sols: [] };
  const mk = (theta1: number, theta2: number): LegSolution => {
    const theta = wrapToPi(theta1);
    const phi = wrapToPi(theta2);
    return { theta, phi, elbow: elbowPoint(g, side, theta), side: elbowSideOf(side, phi) };
  };
  let sols = ik.solutions.map((s) => mk(s.theta1, s.theta2));
  if (sols.length === 1) sols = [{ ...sols[0], side: 1 }, { ...sols[0], side: -1 }];
  else if (sols[0].side === sols[1].side) sols[1] = { ...sols[1], side: (-sols[0].side) as ElbowSide };
  sols.sort((a, b) => b.side - a.side);
  return { reachable: true, r: ik.distance, sols };
}

/** Is the tip inside each leg's annulus |Lp - Ld| <= r <= Lp + Ld? */
export function reachableLegs(g: FiveBarGeom, P: Pt): [boolean, boolean] {
  return [legIK(g, 0, P).reachable, legIK(g, 1, P).reachable];
}

/** IK for a chosen branch; null when the tip is outside either leg's annulus. */
export function ikConfig(g: FiveBarGeom, P: Pt, branch: Branch): Config | null {
  const l = legIK(g, 0, P);
  const r = legIK(g, 1, P);
  if (!l.reachable || !r.reachable) return null;
  const sl = l.sols.find((s) => s.side === branch[0]) ?? l.sols[0];
  const sr = r.sols.find((s) => s.side === branch[1]) ?? r.sols[0];
  return assembleConfig(g, sl.theta, sr.theta, P);
}

/** All IK branch combinations for a tip (up to four; fewer distinct poses on a leg's boundary). */
export function ikAll(g: FiveBarGeom, P: Pt): { branch: Branch; cfg: Config }[] {
  const out: { branch: Branch; cfg: Config }[] = [];
  for (const b of BRANCHES) {
    const cfg = ikConfig(g, P, b);
    if (cfg) out.push({ branch: [b[0], b[1]], cfg });
  }
  return out;
}

export function branchOf(cfg: Config): Branch {
  return [elbowSideOf(0, cfg.phi1), elbowSideOf(1, cfg.phi2)];
}

/**
 * Geometrically distinct elbow choices per leg for a tip: 2, or 1 when the two elbows coincide within
 * `tol` (the leg is fully stretched or folded, i.e. the tip lies on that leg's workspace boundary), or 0 when
 * the tip is out of reach. The number of distinct IK branches is the product of the two counts.
 */
export function legBranchCounts(g: FiveBarGeom, P: Pt, tol = 1e-3): [number, number] {
  const count = (side: Side): number => {
    const leg = legIK(g, side, P);
    if (!leg.reachable) return 0;
    const [a, b] = leg.sols;
    return Math.hypot(a.elbow[0] - b.elbow[0], a.elbow[1] - b.elbow[1]) < tol ? 1 : 2;
  };
  return [count(0), count(1)];
}

// ---------------------------------------------------------------------------
// Forward kinematics: intersection of two circles
// ---------------------------------------------------------------------------

export type CircleStatus = 'two' | 'tangent' | 'none' | 'coincident';

export interface CircleHit {
  status: CircleStatus;
  /** 'two': [left of c1->c2, right of c1->c2]; 'tangent': the single touching point. */
  points: Pt[];
  /** min(r1 + r2 - D, D - |r1 - r2|): positive when the circles cross, 0 when tangent, negative when apart. */
  margin: number;
  /** Distance between the centres. */
  D: number;
}

export function circleIntersections(c1: Pt, r1: number, c2: Pt, r2: number, tol = 1e-10): CircleHit {
  const dx = c2[0] - c1[0];
  const dy = c2[1] - c1[1];
  const D = Math.hypot(dx, dy);
  const margin = Math.min(r1 + r2 - D, D - Math.abs(r1 - r2));
  if (D < 1e-12) return { status: Math.abs(r1 - r2) < 1e-12 ? 'coincident' : 'none', points: [], margin, D };
  if (margin < -tol) return { status: 'none', points: [], margin, D };
  const ux = dx / D;
  const uy = dy / D;
  const a = (r1 * r1 - r2 * r2 + D * D) / (2 * D);
  const mx = c1[0] + a * ux;
  const my = c1[1] + a * uy;
  const h2 = r1 * r1 - a * a;
  if (margin <= tol || h2 <= 0) return { status: 'tangent', points: [[mx, my]], margin: Math.max(0, margin), D };
  const h = Math.sqrt(h2);
  return { status: 'two', points: [[mx - h * uy, my + h * ux], [mx + h * uy, my - h * ux]], margin, D };
}

export interface FKResult {
  B1: Pt;
  B2: Pt;
  status: CircleStatus;
  margin: number;
  /** Mode +1 first when there are two solutions. */
  solutions: Config[];
}

/** Every FK solution for the motor angles (0, 1 or 2; 'coincident' means infinitely many). */
export function fkAll(g: FiveBarGeom, th1: number, th2: number): FKResult {
  const B1 = elbowPoint(g, 0, th1);
  const B2 = elbowPoint(g, 1, th2);
  const hit = circleIntersections(B1, g.L2, B2, g.L4);
  return { B1, B2, status: hit.status, margin: hit.margin, solutions: hit.points.map((P) => assembleConfig(g, th1, th2, P)) };
}

/** Can the mechanism be assembled for these motor angles? */
export function assemblable(g: FiveBarGeom, th1: number, th2: number): boolean {
  return fkAll(g, th1, th2).status !== 'none';
}

/**
 * FK with continuity: the solution nearest to `prev` (or of `preferMode` when there is no previous tip).
 * When both elbows coincide and L2 = L4, every point of the common circle is a solution; the tip then
 * keeps its previous direction as seen from the elbow.
 */
export function fkNearest(g: FiveBarGeom, th1: number, th2: number, prev: Pt | null, preferMode: Mode = 1): Config | null {
  const res = fkAll(g, th1, th2);
  if (res.status === 'coincident') {
    const B = res.B1;
    const ref: Pt = prev ?? [B[0], B[1] + 1];
    const vx = ref[0] - B[0];
    const vy = ref[1] - B[1];
    const n = Math.hypot(vx, vy) || 1;
    return assembleConfig(g, th1, th2, [B[0] + (g.L2 * vx) / n, B[1] + (g.L2 * vy) / n]);
  }
  if (!res.solutions.length) return null;
  if (prev) return res.solutions.reduce((best, s) => (dist2(s.P, prev) < dist2(best.P, prev) ? s : best));
  return res.solutions.find((s) => s.mode === preferMode) ?? res.solutions[0];
}

/**
 * Bisection on s in [lo, hi] where ok(lo) holds and ok(hi) fails: returns the largest s found with ok(s).
 */
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

export interface DriveResult {
  cfg: Config;
  /** True when the motion stopped early because the mechanism could not be assembled further. */
  blocked: boolean;
}

/**
 * Turn the motors from `from` towards (th1, th2) along a straight line in motor space (shortest angular
 * way), following the assembly mode continuously with small sub-steps. If the path leaves the set of
 * assemblable motor angles, the motion stops at the boundary (where the two circles are tangent).
 */
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
    const sb = bisectBoundary((t) => assemblable(g, from.th1 + t * d1, from.th2 + t * d2), (k - 1) / n, s);
    const edge = fkNearest(g, from.th1 + sb * d1, from.th2 + sb * d2, base.P);
    return { cfg: edge ?? base, blocked: true };
  }
  return { cfg: cur, blocked: false };
}

/**
 * Move the tip from `from` towards `target` with a fixed IK branch; stops at the workspace boundary
 * (where one leg becomes straight or folded) if the straight path leaves the workspace.
 */
export function driveTip(g: FiveBarGeom, from: Config, target: Pt, branch: Branch, maxStep = 0.01): DriveResult {
  const dx = target[0] - from.P[0];
  const dy = target[1] - from.P[1];
  const n = Math.max(1, Math.ceil(Math.hypot(dx, dy) / maxStep));
  let cur = from;
  const at = (s: number): Pt => [from.P[0] + s * dx, from.P[1] + s * dy];
  for (let k = 1; k <= n; k++) {
    const s = k / n;
    const next = ikConfig(g, at(s), branch);
    if (next) { cur = next; continue; }
    const sb = bisectBoundary((t) => ikConfig(g, at(t), branch) !== null, (k - 1) / n, s);
    return { cfg: ikConfig(g, at(sb), branch) ?? cur, blocked: true };
  }
  return { cfg: cur, blocked: false };
}

/** Tip displacement when motor m turns by delta (same assembly mode); null if that leaves the assemblable set. */
export function tipShift(g: FiveBarGeom, cfg: Config, m: Side, delta: number): Pt | null {
  const res = driveMotors(g, cfg, cfg.th1 + (m === 0 ? delta : 0), cfg.th2 + (m === 1 ? delta : 0), Math.abs(delta) / 16 || 1e-3);
  if (res.blocked) return null;
  return [res.cfg.P[0] - cfg.P[0], res.cfg.P[1] - cfg.P[1]];
}

/**
 * Finite-difference FK Jacobian dP/d(th1, th2) (2x2, columns = motors) inside the current assembly mode.
 * Central differences when both sides are assemblable, one-sided otherwise; null if neither works.
 */
export function fkJacobianFD(g: FiveBarGeom, cfg: Config, h = 1e-6): number[][] | null {
  const cols: Pt[] = [];
  for (const m of [0, 1] as Side[]) {
    const fwd = fkNearest(g, cfg.th1 + (m === 0 ? h : 0), cfg.th2 + (m === 1 ? h : 0), cfg.P);
    const bwd = fkNearest(g, cfg.th1 - (m === 0 ? h : 0), cfg.th2 - (m === 1 ? h : 0), cfg.P);
    if (fwd && bwd) cols.push([(fwd.P[0] - bwd.P[0]) / (2 * h), (fwd.P[1] - bwd.P[1]) / (2 * h)]);
    else if (fwd) cols.push([(fwd.P[0] - cfg.P[0]) / h, (fwd.P[1] - cfg.P[1]) / h]);
    else if (bwd) cols.push([(cfg.P[0] - bwd.P[0]) / h, (cfg.P[1] - bwd.P[1]) / h]);
    else return null;
  }
  return [[cols[0][0], cols[1][0]], [cols[0][1], cols[1][1]]];
}

/** Largest singular value of a matrix (SVD from @/mr). */
export function sigmaMax(J: number[][]): number {
  return svd(J).S[0];
}

export interface SweepResult {
  /** Motor angle samples (radians, increasing). */
  angles: number[];
  configs: Config[];
  /** The sweep stopped because the mechanism could not be assembled (not because a full turn was done). */
  blockedLo: boolean;
  blockedHi: boolean;
}

/**
 * Sweep motor m (the other motor fixed) from the current configuration in both directions, following the
 * assembly mode, until the mechanism cannot be assembled or half a turn has been covered each way.
 */
export function motorSweep(g: FiveBarGeom, cfg: Config, m: Side, step = 0.01, span = Math.PI): SweepResult {
  const walk = (dir: 1 | -1): { cfgs: Config[]; blocked: boolean } => {
    const out: Config[] = [];
    let cur = cfg;
    const n = Math.ceil(span / step);
    for (let k = 1; k <= n; k++) {
      const res = driveMotors(g, cur, cur.th1 + (m === 0 ? dir * step : 0), cur.th2 + (m === 1 ? dir * step : 0), step / 4);
      out.push(res.cfg);
      if (res.blocked) return { cfgs: out, blocked: true };
      cur = res.cfg;
    }
    return { cfgs: out, blocked: false };
  };
  const lo = walk(-1);
  const hi = walk(1);
  const configs = [...lo.cfgs.reverse(), cfg, ...hi.cfgs];
  const base = m === 0 ? cfg.th1 : cfg.th2;
  // Unwrapped angles relative to the start so the sweep is monotonic even across ±π.
  const angles = [
    ...lo.cfgs.map((_, i) => base - (lo.cfgs.length - i) * step),
    base,
    ...hi.cfgs.map((_, i) => base + (i + 1) * step),
  ];
  // The last sample of a blocked walk sits at the boundary, which can be closer than `step`.
  if (lo.blocked && lo.cfgs.length) angles[0] = base + wrapToPi((m === 0 ? configs[0].th1 : configs[0].th2) - base);
  if (hi.blocked && hi.cfgs.length) {
    const last = configs[configs.length - 1];
    angles[angles.length - 1] = base + wrapToPi((m === 0 ? last.th1 : last.th2) - base);
  }
  return { angles, configs, blockedLo: lo.blocked, blockedHi: hi.blocked };
}

/** Grübler's formula m(N - 1 - J) + Σ f_i (planar: m = 3). */
export function grubler(m: number, N: number, J: number, sumF: number): number {
  return m * (N - 1 - J) + sumF;
}

/**
 * Symmetric pose (th1 = π - th2) of a symmetric five-bar (L1 = L3, L2 = L4) whose elbow distance leaves
 * `margin` before the distal links become collinear (stretched apart). Returns th2, or null if the
 * geometry cannot reach that tangency with the tip above the ground line.
 */
export function symmetricMarginAngle(g: FiveBarGeom, margin: number): number | null {
  if (Math.abs(g.L1 - g.L3) > 1e-9 || Math.abs(g.L2 - g.L4) > 1e-9) return null;
  // elbow distance for th1 = π - th2: D = d + 2 L1 cos th2; tangency when D = 2 L2 - margin
  const c = (2 * g.L2 - margin - g.d) / (2 * g.L1);
  if (c < -1 || c > 1) return null;
  return Math.acos(c);
}

// ---------------------------------------------------------------------------
// Planar 3xRPR platform: IK = three distance formulas, FK = all real roots of a 1-D scan
// ---------------------------------------------------------------------------

export interface Pose2 { x: number; y: number; theta: number }

export interface RPRGeom {
  /** Fixed revolute joints on the ground (world frame). */
  base: [Pt, Pt, Pt];
  /** Platform revolute joints in the platform frame. */
  plat: [Pt, Pt, Pt];
}

/**
 * Default 3xRPR geometry. It was picked (by a random search over designs) so that ordinary leg lengths
 * have 2, 4 or 6 real FK solutions and all of them fit on the lesson canvas.
 */
export const DEFAULT_RPR: Readonly<RPRGeom> = {
  base: [[-1.296, 0.139], [1.252, 0.133], [0.409, 1.683]],
  plat: [[-0.887, -0.366], [0.701, -0.583], [-0.391, 0.93]],
};

/** Platform poses used as presets: their leg lengths give 6, 4 and 2 real FK solutions. */
export const RPR_PRESETS: Readonly<Record<'six' | 'four' | 'two', Pose2>> = {
  six: { x: 0.288, y: 1.22, theta: (44.21 * Math.PI) / 180 },
  four: { x: 0.2, y: 1.2, theta: 0 },
  two: { x: 0, y: 0.95, theta: 0 },
};

/** World positions of the platform joints for a pose. */
export function rprJoints(geom: RPRGeom, pose: Pose2): [Pt, Pt, Pt] {
  const c = Math.cos(pose.theta);
  const s = Math.sin(pose.theta);
  const w = (b: Pt): Pt => [pose.x + c * b[0] - s * b[1], pose.y + s * b[0] + c * b[1]];
  return [w(geom.plat[0]), w(geom.plat[1]), w(geom.plat[2])];
}

/** Inverse kinematics: one distance formula per leg. */
export function rprIK(geom: RPRGeom, pose: Pose2): [number, number, number] {
  const J = rprJoints(geom, pose);
  return [0, 1, 2].map((i) => Math.hypot(J[i][0] - geom.base[i][0], J[i][1] - geom.base[i][1])) as [number, number, number];
}

/**
 * Forward kinematics: every real platform pose with the given leg lengths.
 * For a fixed orientation theta, legs 1 and 2 put the platform origin on two circles (centres a_i - R b_i,
 * radii l_i); for each of the (up to) two intersections the third leg leaves a residual
 * f(theta) = |p + R b3 - a3| - l3. Scanning theta and bisecting every sign change of f on both
 * intersection branches (including changes across a branch merge) finds all real solutions.
 */
export function rprFKAll(geom: RPRGeom, lengths: [number, number, number], samples = 1440): Pose2[] {
  const [a1, a2, a3] = geom.base;
  const [b1, b2, b3] = geom.plat;
  const [l1, l2, l3] = lengths;
  type Eval = { ok: boolean; p: Pt[]; f: number[] };
  const evalAt = (th: number): Eval => {
    const c = Math.cos(th);
    const s = Math.sin(th);
    const rot = (b: Pt): Pt => [c * b[0] - s * b[1], s * b[0] + c * b[1]];
    const r1 = rot(b1);
    const r2 = rot(b2);
    const r3 = rot(b3);
    const hit = circleIntersections([a1[0] - r1[0], a1[1] - r1[1]], l1, [a2[0] - r2[0], a2[1] - r2[1]], l2, 1e-13);
    if (hit.status === 'none' || hit.status === 'coincident') return { ok: false, p: [], f: [] };
    const pts = hit.points.length === 1 ? [hit.points[0], hit.points[0]] : hit.points;
    const f = pts.map((p) => Math.hypot(p[0] + r3[0] - a3[0], p[1] + r3[1] - a3[1]) - l3);
    return { ok: true, p: pts, f };
  };
  const roots: Pose2[] = [];
  const push = (th: number, branch: number) => {
    const e = evalAt(th);
    if (!e.ok) return;
    const p = e.p[branch];
    const pose = { x: p[0], y: p[1], theta: wrapToPi(th) };
    if (!roots.some((r) => Math.abs(wrapToPi(r.theta - pose.theta)) < 1e-6 && Math.hypot(r.x - pose.x, r.y - pose.y) < 1e-6)) roots.push(pose);
  };
  const bisectRoot = (ta: number, tb: number, branch: number) => {
    let fa = evalAt(ta).f[branch];
    let lo = ta;
    let hi = tb;
    for (let k = 0; k < 60; k++) {
      const m = 0.5 * (lo + hi);
      const e = evalAt(m);
      if (!e.ok) return;
      const fm = e.f[branch];
      if (fm === 0) { lo = hi = m; break; }
      if (Math.sign(fm) === Math.sign(fa)) { lo = m; fa = fm; } else hi = m;
    }
    push(0.5 * (lo + hi), branch);
  };
  const h = (2 * Math.PI) / samples;
  const ts: number[] = [];
  const ev: Eval[] = [];
  for (let k = 0; k <= samples; k++) {
    ts.push(-Math.PI + k * h);
    ev.push(evalAt(ts[k]));
  }
  // Pass 1: sign changes between samples, and across the merge of the two branches.
  for (let k = 1; k <= samples; k++) {
    const prev = ev[k - 1];
    const cur = ev[k];
    if (prev.ok && cur.ok) {
      for (const br of [0, 1]) if (Math.sign(prev.f[br]) !== Math.sign(cur.f[br])) bisectRoot(ts[k - 1], ts[k], br);
    } else if (prev.ok !== cur.ok) {
      // The two branches merge where the circles become tangent: locate that edge and check both branches.
      const [tIn, tOut] = prev.ok ? [ts[k - 1], ts[k]] : [ts[k], ts[k - 1]];
      const edge = bisectBoundary((u) => evalAt(tIn + u * (tOut - tIn)).ok, 0, 1, 60);
      const tEdge = tIn + edge * (tOut - tIn);
      const eIn = evalAt(tIn);
      const eEdge = evalAt(tEdge);
      if (eEdge.ok) {
        for (const br of [0, 1]) {
          if (Math.sign(eIn.f[br]) !== Math.sign(eEdge.f[br])) bisectRoot(tIn, tEdge, br);
        }
        if (Math.abs(eEdge.f[0]) < 1e-9) push(tEdge, 0);
      }
    }
  }
  // Pass 2: two roots closer than one sample show up as a shallow local minimum of |f| without a sign
  // change; minimise s*f over the neighbouring interval and bisect both sides if it crosses zero.
  for (let k = 1; k < samples; k++) {
    if (!ev[k - 1].ok || !ev[k].ok || !ev[k + 1].ok) continue;
    for (const br of [0, 1]) {
      const fa = ev[k - 1].f[br];
      const fm = ev[k].f[br];
      const fb = ev[k + 1].f[br];
      const s = Math.sign(fm);
      if (s === 0 || Math.sign(fa) !== s || Math.sign(fb) !== s) continue;
      if (!(Math.abs(fm) < Math.abs(fa) && Math.abs(fm) <= Math.abs(fb) && Math.abs(fm) < 0.05)) continue;
      // golden-section search for the minimum of s*f on [t_{k-1}, t_{k+1}]
      const gr = (Math.sqrt(5) - 1) / 2;
      let a = ts[k - 1];
      let b = ts[k + 1];
      const val = (t: number): number => { const e = evalAt(t); return e.ok ? s * e.f[br] : Infinity; };
      let x1 = b - gr * (b - a);
      let x2 = a + gr * (b - a);
      let v1 = val(x1);
      let v2 = val(x2);
      for (let it = 0; it < 60; it++) {
        if (v1 < v2) { b = x2; x2 = x1; v2 = v1; x1 = b - gr * (b - a); v1 = val(x1); }
        else { a = x1; x1 = x2; v1 = v2; x2 = a + gr * (b - a); v2 = val(x2); }
      }
      const tMin = 0.5 * (a + b);
      const vMin = val(tMin);
      if (vMin < 0) {
        bisectRoot(ts[k - 1], tMin, br);
        bisectRoot(tMin, ts[k + 1], br);
      } else if (vMin < 1e-10) push(tMin, br);
    }
  }
  return roots;
}
