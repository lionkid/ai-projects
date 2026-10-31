/**
 * Reference trajectories for the tracking page.
 *
 * The feasible references are generated FROM the unicycle model: each "move" is a list of
 * constant-curvature pieces (straight lines and circular arcs, driven forward or in reverse)
 * traversed with one time scaling, so q̇_d = G(q_d) u_d holds by construction and the lateral
 * body velocity is zero. The "sideways" reference is the naive one: a straight-line
 * interpolation of q = (φ, x, y) between two poses, which asks a differential-drive base to slide.
 */
import { QuinticTimeScalingDerivs } from '@/mr';
import { expTwist, compose, pose, type Pose, type Twist2 } from './se2';

export type RefKind = 'circle' | 'scurve' | 'parking' | 'sideways' | 'still';

/** A constant body-twist piece: per unit progress the body turns by w (rad) and moves v (±1). */
export interface Piece { w: number; v: number; len: number }

/** Pieces traversed with one time scaling, optionally followed by a pause. */
export interface Move { pieces: Piece[]; duration: number; profile: 'const' | 'quintic'; pause?: number }

export interface RefSample {
  /** Desired pose q_d(t). */
  q: Pose;
  /** Desired body twist V_d = [ω_d, v_d, v_{y,d}] expressed in {d}. */
  V: Twist2;
}

export interface Reference {
  kind: RefKind;
  /** Total duration including the final hold (s). */
  T: number;
  start: Pose;
  sample(t: number): RefSample;
  /** Largest |lateral body velocity| (m/s); 0 for references a differential drive can follow. */
  maxLateral: number;
  /** Poses along the reference for drawing. */
  path: Pose[];
  /** Poses where the direction of travel reverses (gear changes). */
  cusps: Pose[];
}

export const REF_LABELS: Record<RefKind, string> = {
  circle: '圓（R = 1 m）',
  scurve: 'S 彎',
  parking: '停車（前進 S 彎＋倒車）',
  sideways: '直線插值（側移，不可行）',
  still: '停住不動（v_d = 0）',
};

/** Start and goal of the parking maneuver; the sideways reference uses the same pair. */
export const PARK_START = pose(0, -0.7, -0.4);
export const PARK_GOAL = pose(0, -0.7, 0.4);

interface Stage {
  t0: number;
  t1: number;
  start: Pose;
  move: Move | null;
  pieceStart: Pose[];
  cum: number[];
  total: number;
}

/** Build a reference from moves; `hold` seconds of rest are appended at the end. */
export function referenceFromMoves(kind: RefKind, start: Pose, moves: Move[], hold: number): Reference {
  const stages: Stage[] = [];
  let t = 0;
  let q = { ...start };
  for (const m of moves) {
    const pieceStart: Pose[] = [];
    const cum: number[] = [];
    let acc = 0;
    let qp = { ...q };
    for (const p of m.pieces) {
      pieceStart.push(qp);
      cum.push(acc);
      qp = compose(qp, expTwist([p.w, p.v, 0], p.len));
      acc += p.len;
    }
    stages.push({ t0: t, t1: t + m.duration, start: q, move: m, pieceStart, cum, total: acc });
    t += m.duration;
    q = qp;
    if (m.pause) { stages.push({ t0: t, t1: t + m.pause, start: q, move: null, pieceStart: [], cum: [], total: 0 }); t += m.pause; }
  }
  // A zero-length final rest would make sample(T) report V = 0 at the very last instant.
  if (hold > 0 || stages.length === 0) stages.push({ t0: t, t1: t + Math.max(hold, 0), start: q, move: null, pieceStart: [], cum: [], total: 0 });
  const T = t + Math.max(hold, 0);

  const sample = (tq: number): RefSample => {
    const tt = Math.min(Math.max(tq, 0), T);
    let st = stages[stages.length - 1];
    for (const s of stages) { if (tt < s.t1 || s === stages[stages.length - 1]) { st = s; break; } }
    const m = st.move;
    if (!m) return { q: { ...st.start }, V: [0, 0, 0] };
    const tau = tt - st.t0;
    let prog: number, rate: number;
    if (m.profile === 'const') { rate = st.total / m.duration; prog = rate * tau; }
    else { const d = QuinticTimeScalingDerivs(m.duration, tau); prog = st.total * d.s; rate = st.total * d.sdot; }
    prog = Math.min(Math.max(prog, 0), st.total);
    let k = 0;
    while (k < m.pieces.length - 1 && prog >= st.cum[k + 1]) k++;
    const pc = m.pieces[k];
    const q = compose(st.pieceStart[k], expTwist([pc.w, pc.v, 0], prog - st.cum[k]));
    return { q, V: [pc.w * rate, pc.v * rate, 0] };
  };

  const { path, cusps } = tracePath(sample, T, moves.length > 1 ? stages.filter((s) => s.move === null).map((s) => s.t0) : []);
  return { kind, T, start: { ...start }, sample, maxLateral: 0, path, cusps };
}

function tracePath(sample: (t: number) => RefSample, T: number, cuspTimes: number[]): { path: Pose[]; cusps: Pose[] } {
  const N = 360;
  const path: Pose[] = [];
  for (let i = 0; i <= N; i++) path.push(sample((T * i) / N).q);
  // Only rests between moves whose driving directions differ count as gear changes.
  const cusps: Pose[] = [];
  for (const tc of cuspTimes) {
    const before = sample(tc - 0.05).V[1];
    const after = sample(tc + 0.45).V[1];
    if (before * after < 0) cusps.push(sample(tc).q);
  }
  return { path, cusps };
}

/** Straight-line interpolation of q = (φ, x, y) from a to b over `duration` (quintic), then rest. */
export function interpolatedReference(a: Pose, b: Pose, duration: number, hold: number): Reference {
  const T = duration + hold;
  const sample = (tq: number): RefSample => {
    const d = QuinticTimeScalingDerivs(duration, Math.min(Math.max(tq, 0), duration));
    const q = pose(a.phi + d.s * (b.phi - a.phi), a.x + d.s * (b.x - a.x), a.y + d.s * (b.y - a.y));
    const qd = [d.sdot * (b.phi - a.phi), d.sdot * (b.x - a.x), d.sdot * (b.y - a.y)];
    // Express the world velocity in the reference's own frame {d}.
    const c = Math.cos(q.phi), s = Math.sin(q.phi);
    return { q, V: [qd[0], c * qd[1] + s * qd[2], -s * qd[1] + c * qd[2]] };
  };
  const { path } = tracePath(sample, T, []);
  let maxLat = 0;
  for (let i = 0; i <= 400; i++) maxLat = Math.max(maxLat, Math.abs(sample((duration * i) / 400).V[2]));
  return { kind: 'sideways', T, start: { ...a }, sample, maxLateral: maxLat, path, cusps: [] };
}

/**
 * The lesson's references. `speed` scales the reference speed (durations shrink, the circle
 * covers more of its loop in the same 16 s).
 */
export function makeReference(kind: RefKind, speed = 1): Reference {
  const m = Math.max(0.25, speed);
  switch (kind) {
    case 'circle': {
      // R = 1 m centred at the origin, counter-clockwise, v_d = 0.5 m/s at speed 1.
      return referenceFromMoves('circle', pose(0, 0, -1), [{ pieces: [{ w: 1, v: 1, len: 0.5 * m * 16 }], duration: 16, profile: 'const' }], 0);
    }
    case 'scurve': {
      // Straight, then arcs of radius 0.6 m: left 90°, right 180°, left 180°, right 90°, straight.
      const R = 0.6, k = 1 / R, H = Math.PI / 2;
      const pieces: Piece[] = [
        { w: 0, v: 1, len: 0.3 }, { w: k, v: 1, len: R * H }, { w: -k, v: 1, len: R * 2 * H },
        { w: k, v: 1, len: R * 2 * H }, { w: -k, v: 1, len: R * H }, { w: 0, v: 1, len: 0.3 },
      ];
      const len = pieces.reduce((a, p) => a + p.len, 0);
      return referenceFromMoves('scurve', pose(0, -2.1, -0.6), [{ pieces, duration: len / (0.5 * m), profile: 'const' }], 1.5);
    }
    case 'parking': {
      // Forward lane-change S (two 60° arcs, R = 0.8 m) then straight back: a pure 0.8 m side shift.
      const R = 0.8, a = Math.PI / 3, back = 2 * R * Math.sin(a);
      return referenceFromMoves('parking', PARK_START, [
        { pieces: [{ w: 1 / R, v: 1, len: R * a }, { w: -1 / R, v: 1, len: R * a }], duration: 5 / m, profile: 'quintic', pause: 0.4 },
        { pieces: [{ w: 0, v: -1, len: back }], duration: 4.5 / m, profile: 'quintic' },
      ], 1.5);
    }
    case 'sideways':
      return interpolatedReference(PARK_START, PARK_GOAL, 6 / m, 2);
    case 'still':
    default:
      // Parked left of the origin so that its ghost and labels stay clear of the {s} frame.
      return referenceFromMoves('still', pose(0, -0.5, -0.2), [], 8);
  }
}
