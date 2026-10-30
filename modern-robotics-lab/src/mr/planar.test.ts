import { describe, expect, it } from 'vitest';
import { cross, eye, matMul, matVec, type Vec } from './linalg';
import { FKinSpace, JacobianBody, JacobianSpace } from './kinematics';
import { ForwardDynamics } from './dynamics';
import { TransToRp } from './se3';
import {
  buildPlanarChain,
  planar2RAccel,
  planar2RIK,
  planar2RParams,
  planarFK,
  planarJacobian,
  planarTipJacobian,
  rot2,
  se2,
  trans2,
  transInv2,
  transformPoint2,
} from './planar';
import { expectClose, mulberry32, randVec } from './testUtils';

const rng = mulberry32(777);

describe('SE(2) helpers', () => {
  it('compose and invert', () => {
    const T = matMul(trans2(1, 2), rot2(0.7));
    expectClose(T, se2(0.7, 1, 2), 1e-15);
    expectClose(matMul(T, transInv2(T)), eye(3), 1e-14);
    expectClose(transformPoint2(se2(Math.PI / 2, 1, 0), [1, 0]), [1, 1], 1e-15);
  });
});

describe('planar FK / Jacobian vs. the 3-D PoE description', () => {
  const L = [1, 0.8, 0.6];
  const chain = buildPlanarChain(L);

  it('tip position and orientation agree', () => {
    for (let i = 0; i < 10; i++) {
      const th = randVec(rng, 3, Math.PI);
      const fk = planarFK(L, th);
      const [R, p] = TransToRp(FKinSpace(chain.M, chain.Slist, th));
      expectClose(fk.tip, [p[0], p[1]], 1e-12);
      expect(p[2]).toBeCloseTo(0, 12);
      expect(Math.atan2(R[1][0], R[0][0])).toBeCloseTo(Math.atan2(Math.sin(fk.phi), Math.cos(fk.phi)), 10);
      expect(fk.joints.length).toBe(4);
      expectClose(fk.joints[3], fk.tip, 0);
    }
  });

  it('planarJacobian matches finite differences and the PoE Jacobians', () => {
    for (let i = 0; i < 5; i++) {
      const th = randVec(rng, 3, Math.PI);
      const J = planarJacobian(L, th);
      const h = 1e-6;
      for (let j = 0; j < 3; j++) {
        const tp = th.slice();
        const tm = th.slice();
        tp[j] += h;
        tm[j] -= h;
        const fp = planarFK(L, tp);
        const fm = planarFK(L, tm);
        expect(J[0][j]).toBeCloseTo((fp.phi - fm.phi) / (2 * h), 6);
        expect(J[1][j]).toBeCloseTo((fp.tip[0] - fm.tip[0]) / (2 * h), 6);
        expect(J[2][j]).toBeCloseTo((fp.tip[1] - fm.tip[1]) / (2 * h), 6);
      }
      const dth = randVec(rng, 3, 1);
      const planarV = matVec(J, dth); // [wz, vx, vy]
      // Space twist -> tip velocity: v_tip = v_s + w x p_tip
      const Vs = matVec(JacobianSpace(chain.Slist, th), dth);
      const [, p] = TransToRp(FKinSpace(chain.M, chain.Slist, th));
      const vTip = cross(Vs.slice(0, 3), p).map((x, k) => x + Vs[3 + k]);
      expectClose([Vs[2], vTip[0], vTip[1]], planarV, 1e-10, 'space Jacobian');
      // Body twist -> tip velocity: v_tip = R v_b
      const Vb = matVec(JacobianBody(chain.Blist, th), dth);
      const [R] = TransToRp(FKinSpace(chain.M, chain.Slist, th));
      const vTipB = matVec(R, Vb.slice(3, 6));
      expectClose([Vb[2], vTipB[0], vTipB[1]], planarV, 1e-10, 'body Jacobian');
      expectClose(planarTipJacobian(L, th), [J[1], J[2]], 0);
    }
  });

  it('buildPlanarChain has consistent structure', () => {
    expect(chain.Mlist.length).toBe(4);
    expect(chain.Glist.length).toBe(3);
    let Mprod = eye(4);
    for (const Mi of chain.Mlist) Mprod = matMul(Mprod, Mi);
    expectClose(Mprod, chain.M, 1e-15);
    expectClose(chain.Slist.map((row) => row[2]), [0, 0, 1, 0, -1.8, 0], 1e-15);
    expect(() => buildPlanarChain([1, 1], { masses: [1] })).toThrow();
  });
});

describe('analytic 2R inverse kinematics', () => {
  const L1 = 1;
  const L2 = 0.7;

  it('both solutions reproduce the target', () => {
    for (let i = 0; i < 20; i++) {
      const r = 0.35 + rng() * 1.3; // inside the annulus [0.3, 1.7]
      const a = rng() * 2 * Math.PI;
      const target: Vec = [r * Math.cos(a), r * Math.sin(a)];
      const ik = planar2RIK(L1, L2, target[0], target[1]);
      expect(ik.reachable).toBe(true);
      expect(ik.solutions.length).toBe(2);
      const [s1, s2] = ik.solutions;
      expect(s1.elbow).not.toBe(s2.elbow);
      for (const sol of ik.solutions) {
        expectClose(planarFK([L1, L2], [sol.theta1, sol.theta2]).tip, target, 1e-10);
      }
      expect(s1.theta2).toBeCloseTo(-s2.theta2, 12);
    }
  });

  it('flags unreachable targets and boundary cases', () => {
    expect(planar2RIK(L1, L2, 2, 0).reachable).toBe(false);
    expect(planar2RIK(L1, L2, 0.1, 0.1).reachable).toBe(false);
    const edge = planar2RIK(L1, L2, 1.7, 0);
    expect(edge.reachable).toBe(true);
    expect(edge.solutions.length).toBe(1);
    expect(edge.solutions[0].theta2).toBeCloseTo(0, 10);
    expect(edge.solutions[0].theta1).toBeCloseTo(0, 10);
  });
});

describe('closed-form 2R acceleration', () => {
  it('agrees with ForwardDynamics on the equivalent PoE chain', () => {
    const chain = buildPlanarChain([1, 0.8], { masses: [2, 1], model: 'rod' });
    const P = planar2RParams([1, 0.8], [2, 1], 'rod');
    for (let i = 0; i < 5; i++) {
      const th = randVec(rng, 2, Math.PI);
      const dth = randVec(rng, 2, 2);
      const tau = randVec(rng, 2, 5);
      const a = planar2RAccel(P, th, dth, tau);
      const b = ForwardDynamics(th, dth, tau, [0, -9.81, 0], [0, 0, 0, 0, 0, 0], chain.Mlist, chain.Glist, chain.Slist);
      expectClose(a, b, 1e-8);
    }
  });
});
