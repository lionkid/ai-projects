/**
 * Three kinds of "straight line" between two poses (pure logic, no DOM).
 *
 * Planar part: a planar 3R arm (link lengths of @/mr `planar3R`) whose tool pose (x, y, φ) is an SE(2)
 * element. Poses are embedded in SE(3) as rotations about z, so the @/mr path generators
 * JointTrajectory / ScrewTrajectory / CartesianTrajectory are used unchanged. Inverse kinematics along a path
 * is closed form: the wrist point w = p − L3 (cos φ, sin φ) goes through @/mr `planar2RIK`, and θ3 = φ − θ1 − θ2.
 * Among the (up to two) solutions we pick the one closest to a reference configuration:
 *   - 'warm': the previous sample's solution (what a tracking controller does; keeps the motion continuous),
 *   - 'cold': always the start configuration (every sample solved from scratch from the same initial guess).
 *
 * Spatial part: screw interpolation vs. decoupled (Cartesian) interpolation between two SE(3) frames.
 */
import {
  type Mat, type Vec, planar3R, planarFK, planar2RIK, wrapToPi, RotZ, RpToTrans, TransToRp, TransInv,
  MatrixLog6, se3ToVec, Adjoint, matMul, matVec, matSub, normFro, norm, vecSub, ScrewAxisToPointDir,
  JointTrajectory, ScrewTrajectory, CartesianTrajectory, segCircleIntersects, type P2,
} from '@/mr';

export const LINKS: number[] = planar3R.linkLengths ?? [1, 0.8, 0.6];

export interface Pose2 { x: number; y: number; phi: number }
export type PathKind = 'joint' | 'screw' | 'cartesian';
export const PATH_KINDS: PathKind[] = ['joint', 'screw', 'cartesian'];
/** @/mr planar2RIK convention: 'down' when θ2 ≥ 0, 'up' when θ2 < 0. */
export type Elbow = 'down' | 'up';
export type IKPolicy = 'warm' | 'cold';

/** Consecutive samples whose joint vectors differ by more than this (rad, max-norm) are flagged as a jump. */
export const JUMP_TOL = 0.35;
/** Path parameter sampled uniformly: the "time scaling" handed to the @/mr generators is s = t / Tf. */
const LINEAR = (Tf: number, t: number) => t / Tf;

export function poseToT(p: Pose2): Mat {
  return RpToTrans(RotZ(p.phi), [p.x, p.y, 0]);
}

export function TToPose(T: Mat): Pose2 {
  return { x: T[0][3], y: T[1][3], phi: Math.atan2(T[1][0], T[0][0]) };
}

export function wristPoint(p: Pose2, L: number[] = LINKS): P2 {
  return [p.x - L[2] * Math.cos(p.phi), p.y - L[2] * Math.sin(p.phi)];
}

export interface IKCandidate { theta: Vec; elbow: Elbow }

/** All (0, 1 or 2) joint solutions of the planar 3R for a tool pose; angles in [−π, π). */
export function ik3R(p: Pose2, L: number[] = LINKS): IKCandidate[] {
  const [wx, wy] = wristPoint(p, L);
  const r = planar2RIK(L[0], L[1], wx, wy);
  return r.solutions.map((s) => ({
    theta: [wrapToPi(s.theta1), wrapToPi(s.theta2), wrapToPi(p.phi - s.theta1 - s.theta2)],
    elbow: s.elbow,
  }));
}

/** Joint-space distance with each angle difference wrapped to [−π, π). */
export function angDist(a: Vec, b: Vec): number {
  let d = 0;
  for (let i = 0; i < a.length; i++) { const e = wrapToPi(a[i] - b[i]); d += e * e; }
  return Math.sqrt(d);
}

/** Adds multiples of 2π joint by joint so that `th` is as close as possible to `ref` (continuous plots). */
export function unwrapNear(th: Vec, ref: Vec): Vec {
  return th.map((x, i) => ref[i] + wrapToPi(x - ref[i]));
}

function maxAbsDiff(a: Vec, b: Vec): number {
  let m = 0;
  for (let i = 0; i < a.length; i++) m = Math.max(m, Math.abs(a[i] - b[i]));
  return m;
}

export interface Obstacle { c: P2; r: number }

export interface PathResult {
  kind: PathKind;
  /** Path parameter of every sample. */
  s: number[];
  /** Desired tool pose (screw / Cartesian) or the pose actually reached (joint path). */
  target: Pose2[];
  /** Joint solution per sample, null where IK has no solution (unreachable). */
  theta: (Vec | null)[];
  fail: boolean[];
  /** Sample indices k where the joints jump (|θ_k − θ_prev|∞ > JUMP_TOL), e.g. a branch switch. */
  jumps: number[];
  /** dθ/ds (finite differences), null where undefined (failures, jumps). */
  dtheta: (Vec | null)[];
  /** ‖dθ/ds‖, NaN where undefined. */
  speed: number[];
  /** |sin θ2|: 0 means the arm is fully stretched or folded (singular). NaN where IK failed. */
  sin2: number[];
  collide: boolean[];
  firstCollision: number;
  failCount: number;
  maxSpeed: number;
  minSin2: number;
  /** Length of the tool-tip path (m). */
  tipLength: number;
  /** Elbow branch of the last valid configuration. */
  endElbow: Elbow | null;
}

/** Collision of the arm (three link segments) with a disk obstacle. */
export function armCollides(theta: Vec, ob: Obstacle, L: number[] = LINKS): boolean {
  const j = planarFK(L, theta).joints;
  for (let i = 0; i < L.length; i++) {
    if (segCircleIntersects(j[i] as P2, j[i + 1] as P2, ob.c, ob.r)) return true;
  }
  return false;
}

/** Follows a list of tool poses with closed-form IK, choosing branches by `policy`. */
export function trackPoses(targets: Pose2[], theta0: Vec, policy: IKPolicy, L: number[] = LINKS): { theta: (Vec | null)[]; fail: boolean[]; jumps: number[] } {
  const theta: (Vec | null)[] = [];
  const fail: boolean[] = [];
  const jumps: number[] = [];
  let prev = theta0.slice();
  let lastValid: Vec | null = null;
  for (let k = 0; k < targets.length; k++) {
    const cands = ik3R(targets[k], L);
    if (!cands.length) { theta.push(null); fail.push(true); continue; }
    const ref = policy === 'warm' ? prev : theta0;
    let best = cands[0];
    for (const c of cands) if (angDist(c.theta, ref) < angDist(best.theta, ref)) best = c;
    const th = unwrapNear(best.theta, prev);
    if (lastValid && maxAbsDiff(th, lastValid) > JUMP_TOL) jumps.push(k);
    theta.push(th);
    fail.push(false);
    prev = th;
    lastValid = th;
  }
  return { theta, fail, jumps };
}

function finishPath(kind: PathKind, s: number[], target: Pose2[], theta: (Vec | null)[], fail: boolean[], jumps: number[], ob: Obstacle | null, L: number[]): PathResult {
  const N = s.length;
  const jumpSet = new Set(jumps);
  const dtheta: (Vec | null)[] = [];
  const speed: number[] = [];
  for (let k = 0; k < N; k++) {
    const a = k > 0 && theta[k - 1] && !jumpSet.has(k) ? k - 1 : k;
    const b = k < N - 1 && theta[k + 1] && !jumpSet.has(k + 1) ? k + 1 : k;
    const ta = theta[a], tb = theta[b];
    if (!theta[k] || !ta || !tb || a === b) { dtheta.push(null); speed.push(NaN); continue; }
    const ds = s[b] - s[a];
    const d = tb.map((x, i) => (x - ta[i]) / ds);
    dtheta.push(d);
    speed.push(norm(d));
  }
  const sin2 = theta.map((t) => (t ? Math.abs(Math.sin(t[1])) : NaN));
  const collide = theta.map((t) => (t && ob ? armCollides(t, ob, L) : false));
  let tipLength = 0;
  for (let k = 1; k < N; k++) tipLength += Math.hypot(target[k].x - target[k - 1].x, target[k].y - target[k - 1].y);
  let endElbow: Elbow | null = null;
  for (let k = N - 1; k >= 0; k--) { const t = theta[k]; if (t) { endElbow = Math.sin(t[1]) >= 0 ? 'down' : 'up'; break; } }
  const finite = speed.filter((x) => Number.isFinite(x));
  const finiteSin = sin2.filter((x) => Number.isFinite(x));
  return {
    kind, s, target, theta, fail, jumps, dtheta, speed, sin2, collide,
    firstCollision: collide.indexOf(true),
    failCount: fail.filter(Boolean).length,
    maxSpeed: finite.length ? Math.max(...finite) : NaN,
    minSin2: finiteSin.length ? Math.min(...finiteSin) : NaN,
    tipLength,
    endElbow,
  };
}

export interface PlanarPaths {
  ok: boolean;
  /** 'start' or 'end' when that pose has no IK solution. */
  unreachable?: 'start' | 'end';
  thetaStart: Vec;
  thetaEnd: Vec;
  paths: Record<PathKind, PathResult>;
  /** Fixed point of the planar screw motion (the screw axis is perpendicular to the plane through it); null for a pure translation. */
  pole: P2 | null;
  /** Signed rotation angle of the relative motion (rad), in (−π, π]. */
  dphi: number;
}

function pickBranch(p: Pose2, elbow: Elbow, L: number[]): Vec | null {
  const c = ik3R(p, L);
  if (!c.length) return null;
  return (c.find((x) => x.elbow === elbow) ?? c[0]).theta;
}

/**
 * Builds the joint-space, screw and decoupled Cartesian paths between two tool poses, N samples each.
 * The start (and the end of the joint-space line) use the requested elbow branch.
 */
export function planarPaths(start: Pose2, end: Pose2, elbow: Elbow, policy: IKPolicy, ob: Obstacle | null, N = 201, L: number[] = LINKS): PlanarPaths {
  const thS = pickBranch(start, elbow, L);
  const thE = pickBranch(end, elbow, L);
  const s = Array.from({ length: N }, (_, k) => k / (N - 1));
  const Ts = poseToT(start), Te = poseToT(end);
  const Vb = se3ToVec(MatrixLog6(matMul(TransInv(Ts), Te)));
  const dphi = Vb[2];
  let pole: P2 | null = null;
  if (Math.abs(dphi) > 1e-9) {
    const q = ScrewAxisToPointDir(matVec(Adjoint(Ts), Vb)).q;
    pole = [q[0], q[1]];
  }
  if (!thS || !thE) {
    const empty = (kind: PathKind) => finishPath(kind, [0], [start], [null], [true], [], null, L);
    return {
      ok: false, unreachable: !thS ? 'start' : 'end', thetaStart: thS ?? [0, 0, 0], thetaEnd: thE ?? [0, 0, 0],
      paths: { joint: empty('joint'), screw: empty('screw'), cartesian: empty('cartesian') }, pole, dphi,
    };
  }
  // Joint-space straight line.
  const thJ = JointTrajectory(thS, thE, 1, N, LINEAR);
  const tgtJ = thJ.map((t) => { const f = planarFK(L, t); return { x: f.tip[0], y: f.tip[1], phi: f.phi }; });
  const joint = finishPath('joint', s, tgtJ, thJ, thJ.map(() => false), [], ob, L);
  // Screw and decoupled Cartesian paths, followed with IK.
  const mk = (kind: PathKind, Xs: Mat[]) => {
    const tgt = Xs.map(TToPose);
    const tr = trackPoses(tgt, thS, policy, L);
    return finishPath(kind, s, tgt, tr.theta, tr.fail, tr.jumps, ob, L);
  };
  const screw = mk('screw', ScrewTrajectory(Ts, Te, 1, N, LINEAR));
  const cartesian = mk('cartesian', CartesianTrajectory(Ts, Te, 1, N, LINEAR));
  return { ok: true, thetaStart: thS, thetaEnd: thE, paths: { joint, screw, cartesian }, pole, dphi };
}

// ---------------------------------------------------------------------------
// SE(3): screw vs decoupled interpolation
// ---------------------------------------------------------------------------

export interface SE3Comparison {
  screw: Mat[];
  cartesian: Mat[];
  /** Body twist of the relative motion: [ω; v] = log(X_start⁻¹ X_end) (the screw path's constant velocity for s in [0, 1]). */
  Vb: Vec;
  /** The same twist expressed in {s}: Ad_{X_start} V_b (its line is the screw axis in space). */
  Vs: Vec;
  /** Rotation angle |ω| (rad). */
  angle: number;
  /** Pitch h = ω·v / |ω|² (Infinity for a pure translation). */
  pitch: number;
  /** Point on the screw axis closest to the origin and the unit direction (null for a pure translation). */
  axisPoint: Vec | null;
  axisDir: Vec | null;
  /** max over s of ‖R_screw(s) − R_cart(s)‖_F (zero up to rounding: both use R0 exp([ω] s)). */
  maxRotDiff: number;
  /** max over s of ‖p_screw(s) − p_cart(s)‖ (m). */
  maxPosDiff: number;
}

export function compareSE3(Xs: Mat, Xe: Mat, N = 41): SE3Comparison {
  const screw = ScrewTrajectory(Xs, Xe, 1, N, LINEAR);
  const cartesian = CartesianTrajectory(Xs, Xe, 1, N, LINEAR);
  const Vb = se3ToVec(MatrixLog6(matMul(TransInv(Xs), Xe)));
  const Vs = matVec(Adjoint(Xs), Vb);
  const angle = norm(Vb.slice(0, 3));
  let axisPoint: Vec | null = null, axisDir: Vec | null = null, pitch = Infinity;
  if (angle > 1e-9) {
    const a = ScrewAxisToPointDir(Vs);
    axisPoint = a.q; axisDir = a.s; pitch = a.h;
  }
  let maxRotDiff = 0, maxPosDiff = 0;
  for (let k = 0; k < N; k++) {
    const [Ra, pa] = TransToRp(screw[k]);
    const [Rb, pb] = TransToRp(cartesian[k]);
    maxRotDiff = Math.max(maxRotDiff, normFro(matSub(Ra, Rb)));
    maxPosDiff = Math.max(maxPosDiff, norm(vecSub(pa, pb)));
  }
  return { screw, cartesian, Vb, Vs, angle, pitch, axisPoint, axisDir, maxRotDiff, maxPosDiff };
}
