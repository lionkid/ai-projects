/**
 * Independent adversarial review tests for trajectory.ts, control.ts,
 * planar.ts and the linalg corners the robotics code relies on.
 */
import { describe, expect, it } from 'vitest';
import {
  type Mat,
  type Vec,
  CartesianTrajectory,
  ComputedTorque,
  CubicTimeScaling,
  CubicTimeScalingDerivs,
  FKinSpace,
  GravityForces,
  JointTrajectory,
  MassMatrix,
  MatrixLog3,
  MatrixLog6,
  QuinticTimeScaling,
  QuinticTimeScalingDerivs,
  RotZ,
  RpToTrans,
  ScrewTrajectory,
  SimulateControl,
  TransInv,
  TransToRp,
  TrapezoidalTimeScaling,
  VelQuadraticForces,
  buildPlanarChain,
  createPID,
  det,
  dot,
  eigSym,
  eye,
  getCol,
  inv,
  linkFramePoses,
  matMul,
  matVec,
  norm,
  normalize,
  pinv,
  planar2R,
  planar2RDynamics,
  planar2RIK,
  planar2RParams,
  planarFK,
  planarJacobian,
  rot2,
  rrr3,
  se2,
  se3ToVec,
  so3ToVec,
  solve,
  svd,
  trans2,
  transInv2,
  transformPoint2,
  transpose,
  trapezoidParams,
  trapezoidSample,
  vecAdd,
  vecScale,
  vecSub,
  viaPointCubic,
  wrapTo2Pi,
  wrapToPi,
  zerosVec,
} from './index';
import { expectClose, mulberry32, randVec } from './testUtils';

const rng = mulberry32(31337);

describe('review: trapezoidal time scaling', () => {
  it('s(t) equals the double integral of sddot (brute force) and the cruise speed is exactly v', () => {
    for (const spec of [
      { Tf: 2, v: 0.75 },
      { Tf: 1.5, a: 3 },
      { v: 0.6, a: 0.9 },
    ]) {
      const P = trapezoidParams(spec);
      expect(P.valid, JSON.stringify(spec)).toBe(true);
      const N = 200000;
      const h = P.Tf / N;
      let s = 0;
      let sd = 0;
      let maxSd = 0;
      for (let k = 0; k < N; k++) {
        // Midpoint rule on sddot, then on sdot.
        const tm = (k + 0.5) * h;
        const acc = trapezoidSample(tm, P).sddot;
        const sdMid = sd + 0.5 * acc * h;
        s += sdMid * h;
        sd += acc * h;
        maxSd = Math.max(maxSd, sd);
        if (k % 20000 === 0) {
          const ref = trapezoidSample((k + 1) * h, P);
          // Midpoint rule across the two acceleration jumps costs O(a h) ~ 1e-5.
          expect(Math.abs(ref.s - s), `s at ${(k + 1) * h}`).toBeLessThan(1e-4);
          expect(Math.abs(ref.sdot - sd), `sdot at ${(k + 1) * h}`).toBeLessThan(1e-4);
        }
      }
      expect(s).toBeCloseTo(1, 3);
      expect(maxSd).toBeCloseTo(P.v, 4);
      expect(Math.abs(sd)).toBeLessThan(1e-4);
    }
  });

  it('(Tf, a) picks the slower root so that 1 < v Tf <= 2 and the completed triple is self-consistent', () => {
    for (const [Tf, a] of [
      [1, 4],
      [1, 5],
      [2, 1.5],
      [3, 100],
    ]) {
      const P = trapezoidParams({ Tf, a });
      expect(P.valid).toBe(true);
      expect(P.v * P.Tf).toBeGreaterThan(1);
      expect(P.v * P.Tf).toBeLessThanOrEqual(2 + 1e-12);
      expect(P.v).toBeLessThanOrEqual((a * Tf) / 2 + 1e-12);
      const Q = trapezoidParams({ Tf, v: P.v });
      expect(Math.abs(Q.a - a) / a).toBeLessThan(1e-8);
      const R = trapezoidParams({ v: P.v, a });
      expect(Math.abs(R.Tf - Tf) / Tf).toBeLessThan(1e-8);
    }
  });

  it('BUG: on the a Tf^2 = 4 boundary (within the 1e-12 tolerance) the profile is marked valid but v is NaN', () => {
    // a Tf^2 evaluates to 3.999999999999999 for a = 0.7: the feasibility test passes
    // (4 - 1e-12 tolerance) but the discriminant a^2 Tf^2 - 4 a is slightly
    // negative, so Math.sqrt yields NaN. A valid profile must have finite v.
    const offenders = [0.7, 1.3, 2.9, 7.3].map((a) => ({ a, Tf: Math.sqrt(4 / a) }));
    for (const { a, Tf } of offenders) {
      const P = trapezoidParams({ Tf, a });
      if (!P.valid) continue; // rounding happened to land on the safe side
      expect(Number.isFinite(P.v), `a=${a}: valid=${P.valid} but v=${P.v}`).toBe(true);
      expect(Number.isFinite(P.tAccel), `a=${a}: tAccel=${P.tAccel}`).toBe(true);
      // (Tf, t, spec): TrapezoidalTimeScaling now follows the (Tf, t) convention of the
      // other time scalings, as requested by the review (originally written as (t, Tf)).
      const mid = TrapezoidalTimeScaling(Tf, Tf / 2, { a });
      expect(Number.isFinite(mid.sdot)).toBe(true);
    }
  });

  it('boundary values are feasible and values just beyond them are rejected', () => {
    expect(trapezoidParams({ Tf: 2, v: 1 }).valid).toBe(true); // v Tf = 2 (triangle)
    expect(trapezoidParams({ Tf: 2, v: 1 + 1e-9 }).valid).toBe(false);
    expect(trapezoidParams({ Tf: 2, v: 0.5 }).valid).toBe(false); // v Tf = 1 is NOT enough
    expect(trapezoidParams({ Tf: 2, a: 1 }).valid).toBe(true); // a Tf^2 = 4
    expect(trapezoidParams({ Tf: 2, a: 1 - 1e-9 }).valid).toBe(false);
    expect(trapezoidParams({ v: 1, a: 1 }).valid).toBe(true); // v^2 = a
    expect(trapezoidParams({ v: 1, a: 1 - 1e-9 }).valid).toBe(false);
    expect(trapezoidParams({ Tf: -1, v: 1 }).valid).toBe(false);
    expect(trapezoidParams({ Tf: 1, v: 1.5, a: 4.5 }).valid).toBe(false);
    // Triangle profile: no cruise phase, peak at Tf/2.
    const tri = trapezoidParams({ Tf: 2, v: 1 });
    expect(tri.tAccel).toBeCloseTo(1, 12);
    expect(trapezoidSample(1, tri).s).toBeCloseTo(0.5, 12);
    expect(trapezoidSample(1, tri).sdot).toBeCloseTo(1, 12);
  });
});

describe('review: polynomial time scalings', () => {
  it('derivatives match central finite differences of s at interior points', () => {
    const Tf = 1.7;
    const h = 1e-5;
    for (const t of [0.1, 0.5, 0.85, 1.2, 1.6]) {
      const c = CubicTimeScalingDerivs(Tf, t);
      expect(c.s).toBeCloseTo(CubicTimeScaling(Tf, t), 14);
      expect(c.sdot).toBeCloseTo((CubicTimeScaling(Tf, t + h) - CubicTimeScaling(Tf, t - h)) / (2 * h), 7);
      expect(c.sddot).toBeCloseTo((CubicTimeScaling(Tf, t + h) - 2 * CubicTimeScaling(Tf, t) + CubicTimeScaling(Tf, t - h)) / (h * h), 4);
      const q = QuinticTimeScalingDerivs(Tf, t);
      expect(q.s).toBeCloseTo(QuinticTimeScaling(Tf, t), 14);
      expect(q.sdot).toBeCloseTo((QuinticTimeScaling(Tf, t + h) - QuinticTimeScaling(Tf, t - h)) / (2 * h), 7);
      expect(q.sddot).toBeCloseTo((QuinticTimeScaling(Tf, t + h) - 2 * QuinticTimeScaling(Tf, t) + QuinticTimeScaling(Tf, t - h)) / (h * h), 4);
    }
  });

  it('boundary accelerations: cubic has 6/Tf^2 at the ends, quintic has 0; outside [0, Tf] everything is at rest', () => {
    const Tf = 2;
    expect(CubicTimeScalingDerivs(Tf, 0).sddot).toBeCloseTo(6 / (Tf * Tf), 14);
    expect(CubicTimeScalingDerivs(Tf, Tf).sddot).toBeCloseTo(-6 / (Tf * Tf), 14);
    expect(QuinticTimeScalingDerivs(Tf, 0).sddot).toBe(0);
    expect(QuinticTimeScalingDerivs(Tf, Tf).sddot).toBe(0);
    expect(CubicTimeScalingDerivs(Tf, -0.1)).toEqual({ s: 0, sdot: 0, sddot: 0 });
    expect(CubicTimeScalingDerivs(Tf, 2.1)).toEqual({ s: 1, sdot: 0, sddot: 0 });
    // Peak velocities: cubic 1.5/Tf, quintic 1.875/Tf at the midpoint.
    expect(CubicTimeScalingDerivs(Tf, Tf / 2).sdot).toBeCloseTo(1.5 / Tf, 14);
    expect(QuinticTimeScalingDerivs(Tf, Tf / 2).sdot).toBeCloseTo(1.875 / Tf, 14);
  });
});

describe('review: SE(3) paths', () => {
  const Xs = RpToTrans(RotZ(0.3), [0.5, -0.2, 0.1]);
  const Xe = RpToTrans(matMul(RotZ(-1.1), [[1, 0, 0], [0, 0, -1], [0, 1, 0]]), [-0.4, 0.9, 0.6]);

  it('ScrewTrajectory with a linear scaling has a constant relative twist between consecutive samples', () => {
    const N = 9;
    const path = ScrewTrajectory(Xs, Xe, 1, N, (Tf, t) => t / Tf);
    expectClose(path[0], Xs, 1e-14);
    expectClose(path[N - 1], Xe, 1e-12);
    const first = se3ToVec(MatrixLog6(matMul(TransInv(path[0]), path[1])));
    for (let k = 1; k < N - 1; k++) {
      const rel = se3ToVec(MatrixLog6(matMul(TransInv(path[k]), path[k + 1])));
      expectClose(rel, first, 1e-10, `step ${k}`);
    }
    // Total screw = (N-1) x step.
    expectClose(vecScale(first, N - 1), se3ToVec(MatrixLog6(matMul(TransInv(Xs), Xe))), 1e-9);
  });

  it('CartesianTrajectory: rotation angle is proportional to s and the origin is the affine blend', () => {
    const N = 7;
    const path = CartesianTrajectory(Xs, Xe, 2, N, 5);
    const [Rs, ps] = TransToRp(Xs);
    const [Re, pe] = TransToRp(Xe);
    const total = norm(so3ToVec(MatrixLog3(matMul(transpose(Rs), Re))));
    for (let k = 0; k < N; k++) {
      const s = QuinticTimeScaling(2, (2 * k) / (N - 1));
      const [R, p] = TransToRp(path[k]);
      expectClose(p, vecAdd(ps, vecScale(vecSub(pe, ps), s)), 1e-12, `p ${k}`);
      expect(norm(so3ToVec(MatrixLog3(matMul(transpose(Rs), R))))).toBeCloseTo(s * total, 9);
    }
  });

  it('JointTrajectory returns N rows, hits both ends and is monotone for a monotone scaling', () => {
    const path = JointTrajectory([0, 1], [2, -1], 3, 11, 3);
    expect(path.length).toBe(11);
    expectClose(path[0], [0, 1], 0);
    expectClose(path[10], [2, -1], 1e-15);
    for (let k = 1; k < 11; k++) {
      expect(path[k][0]).toBeGreaterThanOrEqual(path[k - 1][0]);
      expect(path[k][1]).toBeLessThanOrEqual(path[k - 1][1]);
    }
    expectClose(path[5], [1, 0], 1e-14); // s(Tf/2) = 1/2 for the cubic
  });
});

describe('review: via-point cubic', () => {
  it('heuristic velocities computed by hand; knots reproduced; evaluation clamped outside the range', () => {
    const sp = viaPointCubic([[0], [1], [3], [2]], [0, 1, 2, 4], 'heuristic');
    // slopes 1, 2, -0.5 -> interior velocities 1.5 and 0 (sign change), zero at the ends
    expectClose(sp.velocities.map((v) => v[0]), [0, 1.5, 0, 0], 1e-15);
    for (let k = 0; k < 4; k++) {
      const e = sp.evaluate(sp.times[k]);
      expect(e.q[0]).toBeCloseTo(sp.points[k][0], 12);
      expect(e.qdot[0]).toBeCloseTo(sp.velocities[k][0], 10);
    }
    expect(sp.evaluate(-5).q[0]).toBe(0);
    expect(sp.evaluate(99).q[0]).toBeCloseTo(2, 12);
    // Continuity of q and qdot across the interior knots from both sides.
    for (const tk of [1, 2]) {
      const a = sp.evaluate(tk - 1e-9);
      const b = sp.evaluate(tk + 1e-9);
      expect(Math.abs(a.q[0] - b.q[0])).toBeLessThan(1e-7);
      expect(Math.abs(a.qdot[0] - b.qdot[0])).toBeLessThan(1e-6);
    }
    // Segment integrals: int qdot dt over a segment equals the displacement.
    const h = 1e-4;
    let acc = 0;
    for (let t = 1 + h / 2; t < 2; t += h) acc += sp.evaluate(t).qdot[0] * h;
    expect(acc).toBeCloseTo(2, 6);
  });

  it('explicit velocities are honoured exactly at the knots and sample(N) spans the time range', () => {
    const sp = viaPointCubic([[0, 0], [1, -1], [0, 2]], [0, 0.5, 2], [
      [0.3, 0],
      [-1, 2],
      [0, 0.1],
    ]);
    expectClose(sp.evaluate(0).qdot, [0.3, 0], 1e-15);
    expectClose(sp.evaluate(0.5).qdot, [-1, 2], 1e-12);
    expectClose(sp.evaluate(2).qdot, [0, 0.1], 1e-12);
    const samples = sp.sample(5);
    expect(samples.map((s) => s.t)).toEqual([0, 0.5, 1, 1.5, 2]);
  });
});

describe('review: control', () => {
  it('ComputedTorque at rest with P gain only is M(theta) Kp e + g(theta) (assembled by hand)', () => {
    const theta = [0.2, -0.5, 0.9];
    const thetad = [0.3, -0.4, 0.7];
    const g = [0, 0, -9.81];
    const tau = ComputedTorque(theta, [0, 0, 0], [0, 0, 0], g, rrr3.Mlist!, rrr3.Glist!, rrr3.Slist, thetad, [0, 0, 0], [0, 0, 0], 12, 0, 0);
    const e = vecSub(thetad, theta);
    const expected = vecAdd(matVec(MassMatrix(theta, rrr3.Mlist!, rrr3.Glist!, rrr3.Slist), vecScale(e, 12)), GravityForces(theta, g, rrr3.Mlist!, rrr3.Glist!, rrr3.Slist));
    expectClose(tau, expected, 1e-12);
    // Per-joint Kd acts on the velocity error with the right sign (moving away -> extra positive torque).
    const tauD = ComputedTorque(theta, [-1, 0, 0], [0, 0, 0], g, rrr3.Mlist!, rrr3.Glist!, rrr3.Slist, theta, [0, 0, 0], [0, 0, 0], 0, 0, [5, 0, 0]);
    const expectedD = vecAdd(
      vecAdd(matVec(MassMatrix(theta, rrr3.Mlist!, rrr3.Glist!, rrr3.Slist), [5, 0, 0]), GravityForces(theta, g, rrr3.Mlist!, rrr3.Glist!, rrr3.Slist)),
      VelQuadraticForces(theta, [-1, 0, 0], rrr3.Mlist!, rrr3.Glist!, rrr3.Slist),
    );
    expectClose(tauD, expectedD, 1e-12);
  });

  it('SimulateControl row semantics: N rows, row 0 is the initial state, errmat[0] = thetad[0] - theta0', () => {
    const N = 5;
    const theta0 = [0.1, 0.2];
    const thetad = Array.from({ length: N }, (_, k) => [0.1 + 0.01 * k, 0.2 - 0.01 * k]);
    const zeros2 = Array.from({ length: N }, () => [0, 0]);
    const res = SimulateControl(theta0, [0, 0], [0, -9.81, 0], Array(N).fill(zerosVec(6)), planar2R.Mlist!, planar2R.Glist!, planar2R.Slist,
      thetad, zeros2, zeros2, [0, -9.81, 0], planar2R.Mlist!, planar2R.Glist!, 20, 0, 5, 0.01, 2, 'euler');
    expect(res.taumat.length).toBe(N);
    expect(res.thetamat.length).toBe(N);
    expect(res.errmat.length).toBe(N);
    expectClose(res.thetamat[0], theta0, 0);
    expectClose(res.errmat[0], [0, 0], 1e-15);
    expectClose(res.errmat[1], vecSub(thetad[1], res.thetamat[1]), 1e-15);
  });

  it('PID on a first-order plant: P-only steady state Kp r/(1 + Kp), PI removes the offset, I-term sign is right', () => {
    const simulate = (kp: number, ki: number): number => {
      const pid = createPID({ kp, ki, kd: 0 });
      let x = 0;
      const dt = 1e-3;
      for (let k = 0; k < 60000; k++) {
        const u = pid.update(1 - x, dt);
        x += (-x + u) * dt; // plant: xdot = -x + u
      }
      return x;
    };
    expect(simulate(4, 0)).toBeCloseTo(4 / 5, 4);
    expect(simulate(4, 2)).toBeCloseTo(1, 4);
  });
});

describe('review: planar helpers', () => {
  it('se2 = trans2 * rot2, transInv2 is the true inverse, transformPoint2 applies R p + t', () => {
    const T = se2(0.7, 1.5, -2);
    expectClose(T, matMul(trans2(1.5, -2), rot2(0.7)), 1e-15);
    expectClose(matMul(T, transInv2(T)), eye(3), 1e-15);
    expectClose(transInv2(T), inv(T), 1e-14);
    expectClose(transformPoint2(T, [1, 0]), [1.5 + Math.cos(0.7), -2 + Math.sin(0.7)], 1e-15);
  });

  it('planarJacobian equals finite differences of planarFK (tip and orientation)', () => {
    const L = [1, 0.8, 0.6];
    const th = randVec(rng, 3, 2.5);
    const J = planarJacobian(L, th);
    const h = 1e-6;
    for (let j = 0; j < 3; j++) {
      const tp = th.slice();
      const tm = th.slice();
      tp[j] += h;
      tm[j] -= h;
      const fp = planarFK(L, tp);
      const fm = planarFK(L, tm);
      expect(J[0][j]).toBeCloseTo((fp.phi - fm.phi) / (2 * h), 8);
      expect(J[1][j]).toBeCloseTo((fp.tip[0] - fm.tip[0]) / (2 * h), 7);
      expect(J[2][j]).toBeCloseTo((fp.tip[1] - fm.tip[1]) / (2 * h), 7);
    }
  });

  it('planar2RIK elbow labels: "down" solutions have the elbow to the right of the base->tip chord', () => {
    for (const [x, y] of [
      [1.2, 0.3],
      [-0.4, 1.1],
      [0.2, -1.5],
      [-1, -0.8],
    ]) {
      const ik = planar2RIK(1, 1, x, y);
      expect(ik.solutions.length).toBe(2);
      for (const sol of ik.solutions) {
        const fk = planarFK([1, 1], [sol.theta1, sol.theta2]);
        expectClose(fk.tip, [x, y], 1e-12);
        const elbow = fk.joints[1];
        const side = x * elbow[1] - y * elbow[0]; // z of chord x elbow: <0 means elbow is to the right
        if (sol.elbow === 'down') expect(side).toBeLessThan(0);
        else expect(side).toBeGreaterThan(0);
      }
      expect(ik.solutions.map((s) => s.elbow).sort()).toEqual(['down', 'up']);
    }
    // Inner boundary with L1 < L2 must still reach the target (folded configuration).
    const inner = planar2RIK(1, 2, 0.6, 0.8);
    expect(inner.reachable).toBe(true);
    expect(inner.solutions.length).toBe(1);
    expectClose(planarFK([1, 2], [inner.solutions[0].theta1, inner.solutions[0].theta2]).tip, [0.6, 0.8], 1e-12);
    expect(planar2RIK(1, 1, 2.1, 0).reachable).toBe(false);
    expect(planar2RIK(1, 2, 0.5, 0).reachable).toBe(false);
  });

  it('closed-form planar2RDynamics agrees with a Lagrangian built from COM positions (point-mass and rod)', () => {
    for (const model of ['point-mass', 'rod'] as const) {
      const P = planar2RParams([1.1, 0.7], [1.3, 0.6], model, 9.81);
      const theta = randVec(rng, 2, 3);
      const dtheta = randVec(rng, 2, 2);
      // Kinetic energy from COM velocities (analytic) and COM inertias.
      const ke = (dth: Vec): number => {
        const [t1, t2] = theta;
        const [d1, d2] = dth;
        const v1 = vecScale([-Math.sin(t1), Math.cos(t1)], P.r1 * d1);
        const v2 = vecAdd(vecScale([-Math.sin(t1), Math.cos(t1)], P.L1 * d1), vecScale([-Math.sin(t1 + t2), Math.cos(t1 + t2)], P.r2 * (d1 + d2)));
        return 0.5 * P.m1 * dot(v1, v1) + 0.5 * P.I1 * d1 * d1 + 0.5 * P.m2 * dot(v2, v2) + 0.5 * P.I2 * (d1 + d2) * (d1 + d2);
      };
      const Mfd: Mat = [
        [2 * ke([1, 0]), ke([1, 1]) - ke([1, 0]) - ke([0, 1])],
        [ke([1, 1]) - ke([1, 0]) - ke([0, 1]), 2 * ke([0, 1])],
      ];
      const U = (th: Vec): number => {
        const y1 = P.r1 * Math.sin(th[0]);
        const y2 = P.L1 * Math.sin(th[0]) + P.r2 * Math.sin(th[0] + th[1]);
        return P.g * (P.m1 * y1 + P.m2 * y2);
      };
      const h = 1e-6;
      const gfd = [0, 1].map((i) => {
        const tp = theta.slice();
        const tm = theta.slice();
        tp[i] += h;
        tm[i] -= h;
        return (U(tp) - U(tm)) / (2 * h);
      });
      // Coriolis via Christoffel symbols: M depends on theta2 only.
      const dM2 = (() => {
        const tp = theta.slice();
        const tm = theta.slice();
        tp[1] += h;
        tm[1] -= h;
        const Mp = planar2RDynamics(P, tp, dtheta).M;
        const Mm = planar2RDynamics(P, tm, dtheta).M;
        return Mp.map((row, i) => row.map((x, j) => (x - Mm[i][j]) / (2 * h)));
      })();
      const dM: Mat[] = [[[0, 0], [0, 0]], dM2];
      const cfd = [0, 1].map((i) => {
        let s = 0;
        for (let j = 0; j < 2; j++) for (let k = 0; k < 2; k++) s += 0.5 * (dM[k][i][j] + dM[j][i][k] - dM[i][j][k]) * dtheta[j] * dtheta[k];
        return s;
      });
      const D = planar2RDynamics(P, theta, dtheta);
      expectClose(D.M, Mfd, 1e-12, `${model} M`);
      expectClose(D.g, gfd, 1e-6, `${model} g`);
      expectClose(D.c, cfd, 1e-6, `${model} c`);
    }
  });

  it('buildPlanarChain places link frames at the COMs and the screw axes through the joints', () => {
    const L = [1, 0.8, 0.6];
    for (const model of ['rod', 'point-mass'] as const) {
      const chain = buildPlanarChain(L, { model, masses: [2, 1, 0.5] });
      const poses = linkFramePoses([0, 0, 0], chain.Mlist, chain.Slist);
      let x = 0;
      for (let i = 0; i < 3; i++) {
        const r = model === 'rod' ? L[i] / 2 : L[i];
        expectClose([poses[i][0][3], poses[i][1][3], poses[i][2][3]], [x + r, 0, 0], 1e-15, `${model} COM ${i}`);
        expectClose(getCol(chain.Slist, i), [0, 0, 1, 0, -x, 0], 1e-15, `${model} S ${i}`);
        expect(chain.Glist[i][3][3]).toBe([2, 1, 0.5][i]);
        x += L[i];
      }
      expectClose(poses[3], chain.M, 1e-15);
      expectClose(chain.M, RpToTrans(eye(3), [2.4, 0, 0]), 0);
      // FK through the PoE matches planarFK for a random configuration.
      const th = randVec(rng, 3, 3);
      const T = FKinSpace(chain.M, chain.Slist, th);
      const fk = planarFK(L, th);
      expectClose([T[0][3], T[1][3]], fk.tip, 1e-12);
      expect(Math.atan2(T[1][0], T[0][0])).toBeCloseTo(wrapToPi(fk.phi) === -Math.PI ? Math.PI : Math.atan2(Math.sin(fk.phi), Math.cos(fk.phi)), 10);
    }
  });
});

describe('review: linalg corners used by the robotics code', () => {
  it('pinv gives the minimum-norm solution for a wide matrix and the least-squares solution for a tall one', () => {
    const J = planarJacobian([1, 0.8, 0.6], [0.3, -0.7, 1.1]); // 3 x 3 (full rank)
    const Jt = [J[1], J[2]]; // 2 x 3 position rows: wide
    const b = [0.3, -0.2];
    const x = matVec(pinv(Jt), b);
    expectClose(matVec(Jt, x), b, 1e-12);
    // Minimum norm: x is orthogonal to the null space of Jt.
    const { V, S } = svd(Jt);
    expect(S.length).toBe(2);
    const nullVec = normalize([
      Jt[0][1] * Jt[1][2] - Jt[0][2] * Jt[1][1],
      Jt[0][2] * Jt[1][0] - Jt[0][0] * Jt[1][2],
      Jt[0][0] * Jt[1][1] - Jt[0][1] * Jt[1][0],
    ]);
    expect(Math.abs(dot(x, nullVec))).toBeLessThan(1e-12);
    expect(V.length).toBe(3);
    // Tall: least squares.
    const A = [[1, 0], [0, 1], [1, 1]];
    const y = [1, 2, 4];
    const ls = matVec(pinv(A), y);
    const normalEq = solve(matMul(transpose(A), A), matVec(transpose(A), y));
    expectClose(ls, normalEq, 1e-12);
  });

  it('svd of a wide rank-deficient matrix has orthonormal U and V and reconstructs A', () => {
    const A = [[1, 2, 3], [2, 4, 6]];
    const { U, S, V } = svd(A);
    expect(S[1]).toBeLessThan(1e-12);
    expectClose(matMul(transpose(U), U), eye(2), 1e-12);
    expectClose(matMul(transpose(V), V), eye(2), 1e-12);
    const rec = matMul(matMul(U, [[S[0], 0], [0, S[1]]]), transpose(V));
    expectClose(rec, A, 1e-12);
    expectClose(pinv([[0, 0], [0, 0]]), [[0, 0], [0, 0]], 0);
  });

  it('det / inv / solve / eigSym on hand-solvable cases', () => {
    expect(det([[0, 1, 0], [1, 0, 0], [0, 0, 1]])).toBe(-1);
    expect(det(RotZ(0.4))).toBeCloseTo(1, 14);
    const A = [[4, 1], [2, 3]];
    expectClose(inv(A), [[0.3, -0.1], [-0.2, 0.4]], 1e-14);
    expectClose(solve(A, [1, 2]), [0.1, 0.6], 1e-14);
    const e = eigSym([[2, 1], [1, 2]]);
    expectClose(e.values, [3, 1], 1e-14);
    expect(Math.abs(dot(getCol(e.vectors, 0), normalize([1, 1])))).toBeCloseTo(1, 12);
    const f = eigSym([[0, 1], [1, 0]]);
    expectClose(f.values, [1, -1], 1e-14);
    expect(() => inv([[1, 2], [2, 4]])).toThrow();
  });

  it('angle wrapping boundaries', () => {
    expect(wrapToPi(Math.PI)).toBe(-Math.PI);
    expect(wrapToPi(-Math.PI)).toBe(-Math.PI);
    expect(wrapToPi(3 * Math.PI)).toBeCloseTo(-Math.PI, 12);
    expect(wrapToPi(0.5)).toBe(0.5);
    expect(wrapToPi(-0.5)).toBe(-0.5);
    expect(wrapTo2Pi(-0.5)).toBeCloseTo(2 * Math.PI - 0.5, 12);
    expect(wrapTo2Pi(2 * Math.PI)).toBe(0);
  });
});
