import { describe, expect, it } from 'vitest';
import { inv, matMul, svd } from '@/mr';
import {
  DEFAULT_GEOM, assembleConfig, basePoint, buildSnapPath, constraintJacobians, det2, doubleSingularConfig, driveTip,
  eeJacobian, fkMode, fkNearest, holdingTorques, ikNearest, indicators, inverseRates, jacobianFD, lockStretch,
  loopResidual, motorSlice, perLegForm, snapConfig, softestDirection, straightLegConfig, symmetricMarginAngle,
  tipVelocity, velocityEllipse, type Config, type FiveBarGeom, type Pt,
} from './fiveBarDiff';

const D = Math.PI / 180;
const g: FiveBarGeom = { ...DEFAULT_GEOM };
const general = (): Config => fkNearest(g, 115 * D, 65 * D, null, 1)!;
const tangent = (): Config => { const t2 = symmetricMarginAngle(g, 0)!; return fkNearest(g, Math.PI - t2, t2, null, 1)!; };
const close = (A: number[][], B: number[][], tol = 1e-6) => {
  for (let i = 0; i < A.length; i++) for (let j = 0; j < A[0].length; j++) expect(Math.abs(A[i][j] - B[i][j])).toBeLessThan(tol);
};

describe('constraint Jacobians from the loop closure', () => {
  it('H_a and H_p are the partial derivatives of g = p_L - p_R', () => {
    const c = general();
    const { Ha, Hp } = constraintJacobians(g, c);
    const h = 1e-7;
    const dg = (dq: Partial<Config>) => {
      const p = loopResidual(g, { ...c, ...dq } as Config);
      const m0 = loopResidual(g, c);
      return [(p[0] - m0[0]) / h, (p[1] - m0[1]) / h];
    };
    const cTh1 = dg({ th1: c.th1 + h });
    const cTh2 = dg({ th2: c.th2 + h });
    const cPh1 = dg({ phi1: c.phi1 + h });
    const cPh2 = dg({ phi2: c.phi2 + h });
    close(Ha, [[cTh1[0], cTh2[0]], [cTh1[1], cTh2[1]]], 1e-5);
    close(Hp, [[cPh1[0], cPh2[0]], [cPh1[1], cPh2[1]]], 1e-5);
  });

  it('J = J_a - J_p H_p^{-1} H_a equals A^{-1} B and the finite-difference FK Jacobian', () => {
    for (const [a, b] of [[115, 65], [100, 40], [130, 75], [95, 85]]) {
      const c = fkNearest(g, a * D, b * D, null, 1)!;
      const J = eeJacobian(g, c)!;
      const { A, Bdiag } = perLegForm(g, c);
      const AinvB = matMul(inv(A), [[Bdiag[0], 0], [0, Bdiag[1]]]);
      close(J, AinvB, 1e-9);
      close(J, jacobianFD(g, c)!, 1e-5);
      expect(det2(A)).toBeCloseTo(-det2(constraintJacobians(g, c).Hp), 12);
    }
  });

  it('K (motor rates from tip velocity) inverts J; the redundant K3 inverts the left 2R leg', () => {
    const c = general();
    const J = eeJacobian(g, c)!;
    close(matMul(inverseRates(g, c), J), [[1, 0], [0, 1]], 1e-9);
    const K3 = inverseRates(g, c, true);
    // left leg as a 2R chain: v = [perp(P - A1), perp(P - B1)] [th1dot; phi1dot]
    const A1 = basePoint(g, 0);
    const JL = [[-(c.P[1] - A1[1]), -(c.P[1] - c.B1[1])], [c.P[0] - A1[0], c.P[0] - c.B1[0]]];
    close(matMul([K3[0], K3[1]], JL), [[1, 0], [0, 1]], 1e-9);
    expect(K3[2][0]).toBeCloseTo(inverseRates(g, c)[1][0], 12);
  });
});

describe('the two kinds of singularity', () => {
  it('distal links collinear: det H_p = 0, ellipse unbounded across the links, no finite holding torque', () => {
    const c = tangent();
    expect(Math.abs(indicators(g, c).act)).toBeLessThan(1e-6);
    expect(eeJacobian(g, c)).toBeNull();
    const e = velocityEllipse(inverseRates(g, c));
    expect(e.r[0]).toBeGreaterThan(1e4);
    // the unbounded direction is perpendicular to the (horizontal) distal links
    expect(Math.abs(e.dirs[0][0])).toBeLessThan(1e-3);
    const soft = softestDirection(g, c);
    expect(soft.sigma).toBeLessThan(1e-6);
    expect(Math.abs(soft.n[1])).toBeCloseTo(1, 6);
    expect(holdingTorques(g, c, [0, 1]).exact).toBe(false);
    expect(tipVelocity(g, c, [1, 0])).toBeNull();
  });

  it('locked motors at the actuator singularity: moving across the links costs only second-order stretch', () => {
    const c = tangent();
    const d = 0.02;
    const across = lockStretch(g, c, [c.P[0], c.P[1] + d]);
    const along = lockStretch(g, c, [c.P[0] + d, c.P[1]]);
    expect(Math.abs(across.e1)).toBeLessThan(d * d); // ≈ d²/(2 L2)
    expect(Math.abs(along.e1)).toBeCloseTo(d, 6);
    // a non-singular pose is stiff in every direction
    expect(softestDirection(g, general()).sigma).toBeGreaterThan(0.5);
  });

  it('a straight leg: sin φ = 0, ellipse flat along the leg, the leg carries a load along it for free', () => {
    const c = straightLegConfig(g, 0, 60 * D)!;
    expect(c).not.toBeNull();
    expect(Math.abs(indicators(g, c).s1)).toBeLessThan(1e-9);
    const res = loopResidual(g, c);
    expect(Math.hypot(res[0], res[1])).toBeLessThan(1e-12);
    const e = velocityEllipse(inverseRates(g, c));
    expect(e.r[1]).toBeLessThan(1e-6);
    const A1 = basePoint(g, 0);
    const u: Pt = [(c.P[0] - A1[0]) / 2.3, (c.P[1] - A1[1]) / 2.3];
    expect(Math.abs(e.dirs[1][0] * u[0] + e.dirs[1][1] * u[1])).toBeCloseTo(1, 6); // lost direction = along the leg
    const t = holdingTorques(g, c, u);
    expect(t.exact).toBe(true);
    for (const x of t.tau) expect(Math.abs(x)).toBeLessThan(1e-6);
    const J = eeJacobian(g, c)!;
    expect(Math.abs(det2(J))).toBeLessThan(1e-9);
  });

  it('both at once: a folded left leg with collinear distal links', () => {
    const c = doubleSingularConfig(g)!;
    expect(c).not.toBeNull();
    const ind = indicators(g, c);
    expect(Math.abs(ind.s1)).toBeLessThan(1e-9);
    expect(Math.abs(ind.act)).toBeLessThan(1e-9);
    const res = loopResidual(g, c);
    expect(Math.hypot(res[0], res[1])).toBeLessThan(1e-9);
  });

  it('holding torques equal Jᵀ f away from singularities', () => {
    const c = general();
    const J = eeJacobian(g, c)!;
    const f: Pt = [0.3, -0.8];
    const t = holdingTorques(g, c, f);
    expect(t.exact).toBe(true);
    expect(t.tau[0]).toBeCloseTo(J[0][0] * f[0] + J[1][0] * f[1], 8);
    expect(t.tau[1]).toBeCloseTo(J[0][1] * f[0] + J[1][1] * f[1], 8);
  });

  it('a third motor at the left elbow removes the actuator singularity but not the end-effector one', () => {
    const c = tangent();
    expect(holdingTorques(g, c, [0, 1], true).exact).toBe(true);
    expect(softestDirection(g, c, true).sigma).toBeGreaterThan(0.5);
    const e = velocityEllipse(inverseRates(g, c, true));
    expect(Number.isFinite(e.r[0])).toBe(true);
    const { Hp } = constraintJacobians(g, c, true);
    expect(Hp.length).toBe(2);
    expect(Hp[0].length).toBe(1);
    expect(Math.hypot(Hp[0][0], Hp[1][0])).toBeCloseTo(g.L4, 12);
    const s = straightLegConfig(g, 0, 60 * D)!;
    expect(velocityEllipse(inverseRates(g, s, true)).r[1]).toBeLessThan(1e-6);
  });

  it('velocity ellipse semi-axes are the singular values of J in a regular pose', () => {
    const c = general();
    const e = velocityEllipse(inverseRates(g, c));
    const S = svd(eeJacobian(g, c)!).S;
    expect(e.r[0]).toBeCloseTo(S[0], 9);
    expect(e.r[1]).toBeCloseTo(S[1], 9);
  });
});

describe('sweeps, IK continuity and the snap-through path', () => {
  it('a θ1 slice ends where the distal links become collinear', () => {
    const sl = motorSlice(g, general(), 0.01);
    // with th2 = 65° the elbows can only separate too far on the side of increasing th1
    expect(sl.blockedHi).toBe(true);
    expect(Math.abs(indicators(g, sl.configs[sl.configs.length - 1]).act)).toBeLessThan(1e-3);
    if (sl.blockedLo) expect(Math.abs(indicators(g, sl.configs[0]).act)).toBeLessThan(1e-3);
    // a symmetric start close to tangency is blocked on both sides
    const t2 = symmetricMarginAngle(g, 0.3)!;
    const both = motorSlice(g, fkNearest(g, Math.PI - t2, t2, null, 1)!, 0.01);
    expect(both.blockedHi).toBe(true);
    for (let i = 1; i < sl.th1.length; i++) expect(sl.th1[i]).toBeGreaterThan(sl.th1[i - 1]);
  });

  it('IK keeps each elbow on its side and stops at the workspace boundary', () => {
    let c = general();
    for (let k = 0; k < 30; k++) {
      const next = ikNearest(g, [c.P[0] + 0.01, c.P[1] - 0.005], c)!;
      expect(Math.sign(Math.sin(next.phi1))).toBe(Math.sign(Math.sin(c.phi1)));
      c = next;
    }
    const r = driveTip(g, general(), [0, 5]);
    expect(r.blocked).toBe(true);
  });

  it('snap path: the tip speeds up towards the singular instant and can come back in the other mode', () => {
    const t2 = symmetricMarginAngle(g, 0.6)!;
    const start = fkMode(g, Math.PI - t2, t2, 1)!;
    const p = buildSnapPath(g, start, 0.3)!;
    expect(p).not.toBeNull();
    expect(p.T1).toBeGreaterThan(1);
    const speed = (t: number) => {
      const s = snapConfig(g, p, t, true);
      const v = tipVelocity(g, s.cfg!, [s.rate, 0]);
      return v ? Math.hypot(v[0], v[1]) : Infinity;
    };
    expect(speed(0.99 * p.T1)).toBeGreaterThan(3 * speed(0.5 * p.T1));
    const end = snapConfig(g, p, 2 * p.T1, true).cfg!;
    expect(end.mode).toBe(-1);
    expect(end.th1).toBeCloseTo(start.th1, 9);
    const same = snapConfig(g, p, 2 * p.T1, false).cfg!;
    expect(same.mode).toBe(1);
    expect(Math.hypot(same.P[0] - start.P[0], same.P[1] - start.P[1])).toBeLessThan(1e-9);
  });

  it('assembleConfig reproduces the loop', () => {
    const c = general();
    const again = assembleConfig(g, c.th1, c.th2, c.P);
    expect(again.phi2).toBeCloseTo(c.phi2, 12);
  });
});
