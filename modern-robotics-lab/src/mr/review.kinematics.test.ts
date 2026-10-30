/**
 * Independent adversarial review tests for kinematics.ts and robots.ts.
 * Expected values come from hand geometry (not from the PoE code).
 */
import { describe, expect, it } from 'vitest';
import {
  type Mat,
  type Vec,
  Adjoint,
  FKinBody,
  FKinSpace,
  IKinBody,
  IKinSpace,
  JacobianBody,
  JacobianSpace,
  RotY,
  RotZ,
  RpToTrans,
  ScrewAxisToPointDir,
  ScrewToAxis,
  SlistToBlist,
  Trans,
  TransInv,
  TransToRp,
  UR5_DIMENSIONS,
  RRR3_DIMENSIONS,
  cross,
  dot,
  eye,
  getCol,
  manipulability,
  matMul,
  matVec,
  norm,
  planar2R,
  planar2RIK,
  planar3R,
  rrr3,
  spaceJointTransforms,
  svd,
  transformPoint,
  transformVector,
  ur5,
  vecSub,
  wrapToPi,
} from './index';
import { expectClose, mulberry32, randVec } from './testUtils';

const rng = mulberry32(7777);
const { W1, W2, L1, L2, H1, H2 } = UR5_DIMENSIONS;

describe('review: UR5 model parameters (book example values)', () => {
  it('home pose M and FK at hand-computed single-joint configurations', () => {
    const pHome = [L1 + L2, W1 + W2, H1 - H2];
    expectClose(ur5.M, [[-1, 0, 0, pHome[0]], [0, 0, 1, pHome[1]], [0, 1, 0, pHome[2]], [0, 0, 0, 1]], 0);
    const MR: Mat = [[-1, 0, 0], [0, 0, 1], [0, 1, 0]];
    // Joint 1 = +pi/2: whole arm rotated about z.
    expectClose(
      FKinSpace(ur5.M, ur5.Slist, [Math.PI / 2, 0, 0, 0, 0, 0]),
      RpToTrans(matMul(RotZ(Math.PI / 2), MR), [-(W1 + W2), L1 + L2, H1 - H2]),
      1e-14,
      'joint 1',
    );
    // Joint 2 = +pi/2 about +y through (0, *, H1): the arm swings DOWN (x -> -z).
    expectClose(
      FKinSpace(ur5.M, ur5.Slist, [0, Math.PI / 2, 0, 0, 0, 0]),
      RpToTrans(matMul(RotY(Math.PI / 2), MR), [-H2, W1 + W2, H1 - (L1 + L2)]),
      1e-14,
      'joint 2',
    );
    // Joint 5 = +pi/2 about -z through (L1+L2, W1, *): tip swings from +y offset to +x offset.
    expectClose(
      FKinSpace(ur5.M, ur5.Slist, [0, 0, 0, 0, Math.PI / 2, 0]),
      RpToTrans(matMul(RotZ(-Math.PI / 2), MR), [L1 + L2 + W2, W1, H1 - H2]),
      1e-14,
      'joint 5',
    );
    // Joint 6 = +pi/2 about +y through the tool origin: position fixed, orientation changes.
    expectClose(
      FKinSpace(ur5.M, ur5.Slist, [0, 0, 0, 0, 0, Math.PI / 2]),
      RpToTrans(matMul(RotY(Math.PI / 2), MR), pHome),
      1e-14,
      'joint 6',
    );
    // Body form must give the same for a random configuration.
    const th = randVec(rng, 6, 2);
    expectClose(FKinBody(ur5.M, ur5.Blist, th), FKinSpace(ur5.M, ur5.Slist, th), 1e-12);
  });

  it('each space screw axis passes through the expected geometric point with the expected direction and zero pitch', () => {
    const expected: { dir: Vec; point: Vec }[] = [
      { dir: [0, 0, 1], point: [0, 0, 0] },
      { dir: [0, 1, 0], point: [0, 0, H1] },
      { dir: [0, 1, 0], point: [L1, 0, H1] },
      { dir: [0, 1, 0], point: [L1 + L2, 0, H1] },
      { dir: [0, 0, -1], point: [L1 + L2, W1, 0] },
      { dir: [0, 1, 0], point: [L1 + L2, 0, H1 - H2] },
    ];
    expected.forEach((e, i) => {
      const S = getCol(ur5.Slist, i);
      const { q, s, h } = ScrewAxisToPointDir(S);
      expectClose(s, e.dir, 1e-15, `dir ${i + 1}`);
      expect(Math.abs(h), `pitch ${i + 1}`).toBeLessThan(1e-15);
      // e.point must lie on the axis: (e.point - q) parallel to dir.
      expect(norm(cross(vecSub(e.point, q), e.dir)), `point ${i + 1}`).toBeLessThan(1e-12);
      expectClose(S, ScrewToAxis(e.point, e.dir, 0), 1e-15, `axis ${i + 1}`);
    });
  });

  it('Blist equals the textbook body-frame table (derived here by hand from M and the axis geometry)', () => {
    const Bexpected: Vec[] = [
      [0, 1, 0, W1 + W2, 0, L1 + L2],
      [0, 0, 1, H2, -(L1 + L2), 0],
      [0, 0, 1, H2, -L2, 0],
      [0, 0, 1, H2, 0, 0],
      [0, -1, 0, -W2, 0, 0],
      [0, 0, 1, 0, 0, 0],
    ];
    for (let i = 0; i < 6; i++) {
      expectClose(getCol(ur5.Blist, i), Bexpected[i], 1e-12, `B${i + 1}`);
      expectClose(getCol(SlistToBlist(ur5.Slist, ur5.M), i), Bexpected[i], 1e-12, `SlistToBlist B${i + 1}`);
    }
  });

  it('link-frame chain ends exactly at the book home pose (Mlist now uses the book joint distances)', () => {
    // The review found the URDF-derived chain 0.72 mm short of M; the Mlist is now built
    // from UR5_DIMENSIONS (URDF masses/inertias/COM placements kept), so the chain must
    // reproduce M to rounding and the model is no longer flagged as approximate.
    let T = eye(4);
    for (const Mi of ur5.Mlist!) T = matMul(T, Mi);
    const [Rchain, pchain] = TransToRp(T);
    const [RM, pM] = TransToRp(ur5.M);
    expectClose(Rchain, RM, 1e-15, 'orientation');
    const offset = norm(vecSub(pchain, pM));
    expect(offset).toBeLessThan(1e-15);
    expect(ur5.dynamicsApproximate).toBeFalsy();
    // Masses are positive, inertias positive, 6 links, 7 frames.
    expect(ur5.Mlist!.length).toBe(7);
    expect(ur5.Glist!.length).toBe(6);
    for (const G of ur5.Glist!) for (let i = 0; i < 6; i++) expect(G[i][i]).toBeGreaterThan(0);
  });
});

describe('review: RRR3 forward kinematics against hand geometry', () => {
  const { L0, L1: A1, L2: A2 } = RRR3_DIMENSIONS;
  const tipByHand = (th: Vec): Vec => {
    const c = A1 * Math.cos(th[1]) + A2 * Math.cos(th[1] + th[2]);
    const s = A1 * Math.sin(th[1]) + A2 * Math.sin(th[1] + th[2]);
    return matVec(RotZ(th[0]), [c, 0, L0 - s]);
  };

  it('tip position and orientation match Rz(t1) Ry(t2 + t3) geometry', () => {
    for (let k = 0; k < 10; k++) {
      const th = randVec(rng, 3, 2.5);
      const T = FKinSpace(rrr3.M, rrr3.Slist, th);
      const [R, p] = TransToRp(T);
      expectClose(p, tipByHand(th), 1e-12, `p k=${k}`);
      expectClose(R, matMul(RotZ(th[0]), RotY(th[1] + th[2])), 1e-12, `R k=${k}`);
    }
  });

  it('space-Jacobian linear velocity of the tip equals the analytic derivative of the hand formula', () => {
    for (let k = 0; k < 10; k++) {
      const th = randVec(rng, 3, 2.5);
      const p = tipByHand(th);
      const Rz = RotZ(th[0]);
      const dp: Vec[] = [
        cross([0, 0, 1], p),
        matVec(Rz, [-(A1 * Math.sin(th[1]) + A2 * Math.sin(th[1] + th[2])), 0, -(A1 * Math.cos(th[1]) + A2 * Math.cos(th[1] + th[2]))]),
        matVec(Rz, [-A2 * Math.sin(th[1] + th[2]), 0, -A2 * Math.cos(th[1] + th[2])]),
      ];
      const Js = JacobianSpace(rrr3.Slist, th);
      const Jb = JacobianBody(rrr3.Blist, th);
      const T = FKinSpace(rrr3.M, rrr3.Slist, th);
      for (let j = 0; j < 3; j++) {
        const col = getCol(Js, j);
        // pdot = v_s + omega_s x p
        const pdotS = [col[3] + (col[1] * p[2] - col[2] * p[1]), col[4] + (col[2] * p[0] - col[0] * p[2]), col[5] + (col[0] * p[1] - col[1] * p[0])];
        expectClose(pdotS, dp[j], 1e-12, `space col ${j}`);
        // pdot = R v_b
        const pdotB = transformVector(T, getCol(Jb, j).slice(3, 6));
        expectClose(pdotB, dp[j], 1e-12, `body col ${j}`);
      }
    }
  });
});

describe('review: Jacobian columns are the CURRENT joint screw axes', () => {
  it('J_s column i = ScrewToAxis(P_{i-1} q_i, R_{i-1} s_i, h_i) for every predefined model', () => {
    for (const model of [planar2R, planar3R, rrr3, ur5]) {
      const th = randVec(rng, model.dof, 2);
      const P = spaceJointTransforms(model.Slist, th);
      const Js = JacobianSpace(model.Slist, th);
      for (let i = 0; i < model.dof; i++) {
        const { q, s, h } = ScrewAxisToPointDir(getCol(model.Slist, i));
        const Pprev = i === 0 ? eye(4) : P[i - 1];
        const expected = ScrewToAxis(transformPoint(Pprev, q), transformVector(Pprev, s), h);
        expectClose(getCol(Js, i), expected, 1e-12, `${model.name} col ${i}`);
      }
    }
  });

  it('J_b is J_s pulled back by Ad_{T^-1}, and the angular part of J_b is R^T times that of J_s', () => {
    const th = randVec(rng, 6, 2);
    const T = FKinSpace(ur5.M, ur5.Slist, th);
    const Js = JacobianSpace(ur5.Slist, th);
    const Jb = JacobianBody(ur5.Blist, th);
    expectClose(Jb, matMul(Adjoint(TransInv(T)), Js), 1e-12);
    const [R] = TransToRp(T);
    for (let j = 0; j < 6; j++) {
      const ws = getCol(Js, j).slice(0, 3);
      const wb = getCol(Jb, j).slice(0, 3);
      expectClose(matVec(R, wb), ws, 1e-12, `col ${j}`);
    }
  });
});

describe('review: manipulability vs singular values', () => {
  it('volume, isotropy and extreme sigmas agree with svd of J', () => {
    const th = randVec(rng, 6, 2);
    const J = JacobianBody(ur5.Blist, th);
    const lin = [J[3], J[4], J[5]];
    const m = manipulability(lin);
    const { S } = svd(lin);
    expect(m.sigmaMax).toBeCloseTo(S[0], 10);
    expect(m.sigmaMin).toBeCloseTo(S[2], 10);
    expect(m.volume).toBeCloseTo(S[0] * S[1] * S[2], 10);
    expect(m.isotropy).toBeCloseTo(S[0] / S[2], 8);
  });
});

describe('review: inverse kinematics semantics', () => {
  it('errorOmega / errorV separate angular from linear error in the BODY frame', () => {
    const th0 = [0.3, -0.4, 0.5];
    const T0 = FKinSpace(rrr3.M, rrr3.Slist, th0);
    // Pure body rotation about the tool z-axis by 0.3 rad.
    const Trot = matMul(T0, RpToTrans(RotZ(0.3), [0, 0, 0]));
    const r1 = IKinBody(rrr3.Blist, rrr3.M, Trot, th0, 1e-3, 1e-4, { maxIterations: 0 });
    expect(r1.iterations).toBe(0);
    expect(r1.history.length).toBe(1);
    expect(r1.history[0].errorOmega).toBeCloseTo(0.3, 10);
    expect(r1.history[0].errorV).toBeCloseTo(0, 10);
    // Pure body translation along the tool x-axis by 0.1 m.
    const Ttr = matMul(T0, Trans(0.1, 0, 0));
    const r2 = IKinBody(rrr3.Blist, rrr3.M, Ttr, th0, 1e-3, 1e-4, { maxIterations: 0 });
    expect(r2.history[0].errorOmega).toBeCloseTo(0, 10);
    expect(r2.history[0].errorV).toBeCloseTo(0.1, 10);
    expect(r2.success).toBe(false);
  });

  it('IKinSpace reports the SPACE error twist: a body rotation about an offset tool origin shows up as linear error too', () => {
    const th0 = [0.3, -0.4, 0.5];
    const T0 = FKinSpace(rrr3.M, rrr3.Slist, th0);
    const Trot = matMul(T0, RpToTrans(RotZ(0.3), [0, 0, 0]));
    const r = IKinSpace(rrr3.Slist, rrr3.M, Trot, th0, 1e-3, 1e-4, { maxIterations: 0 });
    const [, p] = TransToRp(T0);
    const wS = transformVector(T0, [0, 0, 0.3]);
    expect(r.history[0].errorOmega).toBeCloseTo(0.3, 10);
    expect(r.history[0].errorV).toBeCloseTo(norm(cross(p, wS)), 10);
  });

  it('converges to the analytic planar 2R solution (both elbows, body and space forms)', () => {
    const [Lx, Ly] = [1, 1];
    for (const [x, y] of [
      [1.2, 0.8],
      [-0.5, 1.4],
      [0.3, -1.1],
    ]) {
      const ik = planar2RIK(Lx, Ly, x, y);
      expect(ik.reachable).toBe(true);
      expect(ik.solutions.length).toBe(2);
      for (const sol of ik.solutions) {
        const T = RpToTrans(RotZ(sol.theta1 + sol.theta2), [x, y, 0]);
        const guess = [sol.theta1 + 0.25, sol.theta2 - 0.3];
        const rb = IKinBody(planar2R.Blist, planar2R.M, T, guess, 1e-9, 1e-9, { maxIterations: 50 });
        const rs = IKinSpace(planar2R.Slist, planar2R.M, T, guess, 1e-9, 1e-9, { maxIterations: 50 });
        expect(rb.success).toBe(true);
        expect(rs.success).toBe(true);
        expectClose(rb.thetalist.map(wrapToPi), [sol.theta1, sol.theta2].map(wrapToPi), 1e-7, `body ${sol.elbow}`);
        expectClose(rs.thetalist.map(wrapToPi), [sol.theta1, sol.theta2].map(wrapToPi), 1e-7, `space ${sol.elbow}`);
        expectClose(FKinSpace(planar2R.M, planar2R.Slist, rb.thetalist), T, 1e-8);
      }
    }
  });

  it('damped least squares converges on the UR5 to a pose that the plain pseudo-inverse also reaches, and history is consistent', () => {
    const thTarget = [0.4, -1.1, 0.9, -0.6, 1.3, 0.2];
    const T = FKinSpace(ur5.M, ur5.Slist, thTarget);
    const guess = thTarget.map((t) => t + 0.2);
    const plain = IKinBody(ur5.Blist, ur5.M, T, guess, 1e-8, 1e-8, { maxIterations: 50 });
    const damped = IKinBody(ur5.Blist, ur5.M, T, guess, 1e-8, 1e-8, { maxIterations: 200, lambda: 0.05 });
    expect(plain.success).toBe(true);
    expect(damped.success).toBe(true);
    expectClose(FKinSpace(ur5.M, ur5.Slist, plain.thetalist), T, 1e-7);
    expectClose(FKinSpace(ur5.M, ur5.Slist, damped.thetalist), T, 1e-7);
    expect(plain.history.length).toBe(plain.iterations + 1);
    expect(damped.history.length).toBe(damped.iterations + 1);
    // Error must be monotonically shrinking in the last steps of the undamped Newton iteration.
    const last = plain.history.slice(-3);
    for (let i = 1; i < last.length; i++) {
      expect(last[i].errorOmega + last[i].errorV).toBeLessThanOrEqual(last[i - 1].errorOmega + last[i - 1].errorV + 1e-12);
    }
  });

  it('maxStep is applied to the joint-space norm of the increment, not per joint', () => {
    const T = FKinSpace(ur5.M, ur5.Slist, [1.5, -1.0, 1.0, 0.5, 1.0, -1.5]);
    const r = IKinBody(ur5.Blist, ur5.M, T, [0, 0, 0, 0, 0, 0], 1e-6, 1e-6, { maxIterations: 5, maxStep: 0.1 });
    for (let k = 1; k < r.history.length; k++) {
      const step = norm(vecSub(r.history[k].theta, r.history[k - 1].theta));
      expect(step).toBeLessThanOrEqual(0.1 + 1e-12);
    }
    expect(dot(r.history[1].theta, r.history[1].theta)).toBeGreaterThan(0); // actually moved
  });
});
