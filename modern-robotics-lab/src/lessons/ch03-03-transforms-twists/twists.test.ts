import { describe, expect, it } from 'vitest';
import {
  Adjoint, MatrixExp6, RotZ, RpToTrans, ScrewToAxis, TestIfSE3, TransInv, VecTose3, VecToso3, approxEqual, cross, eye, matMul,
  matScale, matVec, norm, transformPoint, vecScale, vecSub,
} from '@/mr';
import {
  adjointBlocks, applyFraction, bodyPointPath, changeFrame, closestPointOnLine, constantBodyTwistPose, distanceFromIdentity,
  hasHomogeneousRow, inverseCandidate, loopDuration, numericDerivative, originPath, pointVelocity, poseFromParams, screwInfo,
  twistsFromDerivative,
} from './twists';

const DEG = Math.PI / 180;
const Tsb = poseFromParams({ x: 0.9, y: 0.6, z: 0.35, yaw: 35 * DEG, pitch: 15 * DEG, roll: -25 * DEG });

describe('poses and inverses', () => {
  it('poseFromParams builds a rigid transform with the requested position', () => {
    expect(TestIfSE3(Tsb)).toBe(true);
    expect([Tsb[0][3], Tsb[1][3], Tsb[2][3]]).toEqual([0.9, 0.6, 0.35]);
  });

  it('only the correct candidate inverts T', () => {
    const good = inverseCandidate(Tsb, 'correct');
    expect(distanceFromIdentity(matMul(Tsb, good))).toBeLessThan(1e-12);
    expect(distanceFromIdentity(matMul(good, Tsb))).toBeLessThan(1e-12);
    const negp = inverseCandidate(Tsb, 'negp');
    expect(TestIfSE3(negp)).toBe(true); // a valid transform, just the wrong one
    expect(distanceFromIdentity(matMul(Tsb, negp))).toBeGreaterThan(0.1);
    const tr = inverseCandidate(Tsb, 'transpose');
    expect(hasHomogeneousRow(tr)).toBe(false); // last row is [pᵀ 1]
    expect(tr[3]).toEqual([0.9, 0.6, 0.35, 1]);
  });

  it('the wrong inverse rebuilds {s} at p − R p instead of the origin', () => {
    const W = matMul(Tsb, inverseCandidate(Tsb, 'negp'));
    const p = [0.9, 0.6, 0.35];
    const R = Tsb.slice(0, 3).map((r) => r.slice(0, 3));
    expect(approxEqual([W[0][3], W[1][3], W[2][3]], vecSub(p, matVec(R, p)), 1e-12)).toBe(true);
  });

  it('subscripts cancel: T_sc = T_sb T_bc maps {c} coordinates to {s}', () => {
    const Tbc = poseFromParams({ x: 0.5, y: -0.1, z: 0.3, yaw: -50 * DEG, pitch: 30 * DEG, roll: 0 });
    const Tsc = matMul(Tsb, Tbc);
    const rc = [0.1, 0.2, -0.3];
    expect(approxEqual(transformPoint(Tsc, rc), transformPoint(Tsb, transformPoint(Tbc, rc)), 1e-12)).toBe(true);
  });
});

describe('pre- and post-multiplication', () => {
  const Rz90 = RpToTrans(RotZ(Math.PI / 2), [0, 0, 0]);
  const T0 = poseFromParams({ x: 0.9, y: 0.2, z: 0.3, yaw: 50 * DEG, pitch: 0, roll: 0 });

  it('u = 0 is the identity and u = 1 the full product', () => {
    expect(approxEqual(applyFraction(Rz90, T0, 0, 'pre'), T0, 1e-12)).toBe(true);
    expect(approxEqual(applyFraction(Rz90, T0, 1, 'pre'), matMul(Rz90, T0), 1e-12)).toBe(true);
    expect(approxEqual(applyFraction(Rz90, T0, 1, 'post'), matMul(T0, Rz90), 1e-12)).toBe(true);
  });

  it('a rotation about z swings the origin around z_s when pre-multiplied, but not when post-multiplied', () => {
    const pre = originPath(Rz90, T0, 'pre', 12);
    const post = originPath(Rz90, T0, 'post', 12);
    const r0 = Math.hypot(0.9, 0.2);
    for (const p of pre) expect(Math.hypot(p[0], p[1])).toBeCloseTo(r0, 10); // circle about z_s
    for (const p of post) expect(approxEqual(p, [0.9, 0.2, 0.3], 1e-12)).toBe(true); // origin fixed
    expect(approxEqual(pre[12], [-0.2, 0.9, 0.3], 1e-12)).toBe(true);
  });
});

describe('twists', () => {
  const Vb = [0.2, -0.1, 0.8, 0.4, 0.05, 0.1];
  const T0 = poseFromParams({ x: 0.7, y: 0.4, z: 0.45, yaw: 30 * DEG, pitch: 0, roll: 25 * DEG });
  const f = (t: number) => constantBodyTwistPose(T0, Vb, t);

  it('integrates Ṫ = T[V_b] exactly and stays in SE(3)', () => {
    expect(TestIfSE3(f(7.3))).toBe(true);
    // semigroup property of the exponential
    expect(approxEqual(f(2.5), matMul(f(1), MatrixExp6(matScale(VecTose3(Vb), 1.5))), 1e-12)).toBe(true);
  });

  it('T⁻¹Ṫ returns V_b and ṪT⁻¹ equals [Ad_T] V_b (numerical derivative)', () => {
    for (const t of [0, 1.3, 4.2]) {
      const T = f(t);
      const { Vs, Vb: VbNum } = twistsFromDerivative(T, numericDerivative(f, t));
      expect(approxEqual(VbNum, Vb, 1e-7)).toBe(true);
      expect(approxEqual(Vs, matVec(Adjoint(T), Vb), 1e-7)).toBe(true);
      expect(approxEqual(Vs, changeFrame(T, Vb), 1e-7)).toBe(true);
    }
  });

  it('a constant body twist also has a constant spatial twist', () => {
    expect(approxEqual(changeFrame(f(0), Vb), changeFrame(f(3.7), Vb), 1e-10)).toBe(true);
  });

  it('v_s is the velocity of the body point passing through the {s} origin', () => {
    const t = 2.1, h = 1e-5;
    const T = f(t);
    const Vs = changeFrame(T, Vb);
    // the body point at the {s} origin right now has body coordinates T⁻¹·0
    const rb = transformPoint(TransInv(T), [0, 0, 0]);
    const vel = vecScale(vecSub(transformPoint(f(t + h), rb), transformPoint(f(t - h), rb)), 1 / (2 * h));
    expect(approxEqual(vel, Vs.slice(3, 6), 1e-7)).toBe(true);
    // and any other body point moves with ω_s × r + v_s
    const rb2 = [0.22, 0.12, 0.08];
    const vel2 = vecScale(vecSub(transformPoint(f(t + h), rb2), transformPoint(f(t - h), rb2)), 1 / (2 * h));
    expect(approxEqual(vel2, pointVelocity(Vs, transformPoint(T, rb2)), 1e-7)).toBe(true);
  });

  it('Adjoint blocks are [R, 0; [p]R, R]', () => {
    const { R, pR, Ad } = adjointBlocks(Tsb);
    const p = [Tsb[0][3], Tsb[1][3], Tsb[2][3]];
    expect(approxEqual(pR, matMul(VecToso3(p), R), 1e-12)).toBe(true);
    for (let i = 0; i < 3; i++) for (let j = 3; j < 6; j++) expect(Ad[i][j]).toBe(0);
  });
});

describe('instantaneous screw axis', () => {
  it('recovers the axis point, direction and pitch of a scaled screw', () => {
    const q = [0.3, -0.4, 0.2], s = [0, 0.6, 0.8], h = 0.15;
    const V = vecScale(ScrewToAxis(q, s, h), 1.7);
    const info = screwInfo(V);
    expect(info.kind).toBe('screw');
    if (info.kind !== 'screw') return;
    expect(info.h).toBeCloseTo(h, 12);
    expect(info.rate).toBeCloseTo(1.7, 12);
    expect(approxEqual(info.dir, s, 1e-12)).toBe(true);
    // q lies on the axis: q − q_given is parallel to s
    expect(norm(cross(vecSub(info.q, q), s))).toBeLessThan(1e-12);
  });

  it('classifies pure translations and the zero twist', () => {
    const tr = screwInfo([0, 0, 0, 0.3, 0, 0.4]);
    expect(tr.kind).toBe('translation');
    if (tr.kind === 'translation') { expect(tr.speed).toBeCloseTo(0.5, 12); expect(approxEqual(tr.dir, [0.6, 0, 0.8], 1e-12)).toBe(true); }
    expect(screwInfo([0, 0, 0, 0, 0, 0]).kind).toBe('zero');
  });

  it('putting {s} on the axis of a pure rotation makes v_s vanish', () => {
    const T0 = poseFromParams({ x: 0.8, y: 0.5, z: 0.4, yaw: 30 * DEG, pitch: 0, roll: 0 });
    const Vb = [0, 0, 1, 0, 0, 0];
    const Vw = changeFrame(T0, Vb);
    const info = screwInfo(Vw);
    if (info.kind !== 'screw') throw new Error('expected a screw');
    const s = closestPointOnLine(info.q, info.dir, [-0.5, -0.6, 0]);
    const Vs = changeFrame(TransInv(RpToTrans(eye(3), s)), Vw);
    expect(norm(Vs.slice(3, 6))).toBeLessThan(1e-12);
  });

  it('the body origin path wraps around the screw axis at constant distance', () => {
    const T0 = poseFromParams({ x: 0.6, y: 0.3, z: 0.3, yaw: 0, pitch: 0, roll: 0 });
    const Vb = [0, 0, 0.8, 0.35, 0, 0.06];
    const pts = bodyPointPath(T0, Vb, [0, 0, 0], loopDuration(Vb), 40);
    const info = screwInfo(changeFrame(T0, Vb));
    if (info.kind !== 'screw') throw new Error('expected a screw');
    const d0 = norm(vecSub(pts[0], closestPointOnLine(info.q, info.dir, pts[0])));
    for (const p of pts) expect(norm(vecSub(p, closestPointOnLine(info.q, info.dir, p)))).toBeCloseTo(d0, 9);
    expect(loopDuration(Vb)).toBeCloseTo((2 * Math.PI) / 0.8, 12);
    expect(loopDuration([0, 0, 0, 1, 0, 0])).toBe(5);
  });
});

describe('quiz facts', () => {
  it('Q1: pure rotation about an axis through q gives v_s = −θ̇ ŝ × q', () => {
    const q = [1, 2, 0], s = [0, 0, 1], rate = 0.7;
    const Vs = vecScale(ScrewToAxis(q, s, 0), rate);
    expect(approxEqual(Vs.slice(3, 6), vecScale(cross(s, q), -rate), 1e-12)).toBe(true);
    expect(norm(Vs.slice(3, 6))).toBeGreaterThan(0);
  });

  it('Q4: spinning about z_s at 1 rad/s, a frame at (2, 0, 0) has v_b = (0, 2, 0)', () => {
    const Tsb2 = RpToTrans(eye(3), [2, 0, 0]);
    const Vb = changeFrame(TransInv(Tsb2), [0, 0, 1, 0, 0, 0]);
    expect(approxEqual(Vb, [0, 0, 1, 0, 2, 0], 1e-12)).toBe(true);
  });

  it('Q2: a second body-fixed frame {c} has V_c = [Ad_{T_cb}] V_b', () => {
    const Tbc = poseFromParams({ x: 0.3, y: 0.1, z: -0.2, yaw: 20 * DEG, pitch: 10 * DEG, roll: 5 * DEG });
    const Vb = [0.3, -0.2, 0.9, 0.1, 0.4, -0.2];
    const T0 = poseFromParams({ x: 0.5, y: 0.5, z: 0.5, yaw: 0.3, pitch: 0.2, roll: 0.1 });
    const fc = (t: number) => matMul(constantBodyTwistPose(T0, Vb, t), Tbc);
    const { Vb: Vc } = twistsFromDerivative(fc(1.1), numericDerivative(fc, 1.1));
    expect(approxEqual(Vc, changeFrame(TransInv(Tbc), Vb), 1e-7)).toBe(true);
    expect(approxEqual(Vc, changeFrame(Tbc, Vb), 1e-3)).toBe(false);
  });
});
