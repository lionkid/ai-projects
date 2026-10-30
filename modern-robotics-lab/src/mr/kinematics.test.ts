import { describe, expect, it } from 'vitest';
import { eye, getCol, matMul, matScale, matSub, matVec, norm, type Mat, type Vec } from './linalg';
import { RotZ } from './so3';
import { Adjoint, RpToTrans, TransInv, TransToRp, se3ToVec } from './se3';
import {
  FKinBody,
  FKinSpace,
  IKinBody,
  IKinSpace,
  JacobianBody,
  JacobianSpace,
  SlistToBlist,
  BlistToSlist,
  manipulability,
  spaceJointTransforms,
} from './kinematics';
import { planar2R, planar3R, rrr3, ur5 } from './robots';
import { expectClose, mulberry32, randVec } from './testUtils';

const rng = mulberry32(31337);
const models = [planar2R, planar3R, rrr3, ur5];

describe('forward kinematics', () => {
  it('reproduces the UR5 configuration from the book example', () => {
    const theta = [0, -Math.PI / 2, 0, 0, Math.PI / 2, 0];
    const T = FKinSpace(ur5.M, ur5.Slist, theta);
    const [R, p] = TransToRp(T);
    expectClose(p, [0.095, 0.109, 0.988], 1e-3, 'position');
    expectClose(R, [
      [0, -1, 0],
      [1, 0, 0],
      [0, 0, 1],
    ], 1e-9, 'rotation');
    expectClose(FKinBody(ur5.M, ur5.Blist, theta), T, 1e-12);
  });

  it('FKinSpace equals FKinBody with Blist = Ad(M^-1) Slist for random joint values', () => {
    for (const model of models) {
      const Blist = SlistToBlist(model.Slist, model.M);
      expectClose(Blist, model.Blist, 1e-12, `${model.name} Blist`);
      expectClose(BlistToSlist(Blist, model.M), model.Slist, 1e-12, `${model.name} Slist`);
      for (let i = 0; i < 10; i++) {
        const theta = randVec(rng, model.dof, Math.PI);
        expectClose(FKinSpace(model.M, model.Slist, theta), FKinBody(model.M, Blist, theta), 1e-10, model.name);
      }
    }
  });

  it('zero joint angles give the home configuration', () => {
    for (const model of models) {
      expectClose(FKinSpace(model.M, model.Slist, new Array(model.dof).fill(0)), model.M, 1e-15);
    }
  });

  it('planar2R FK matches the elementary 2R formula', () => {
    const th = [0.4, -1.1];
    const T = FKinSpace(planar2R.M, planar2R.Slist, th);
    const x = Math.cos(th[0]) + Math.cos(th[0] + th[1]);
    const y = Math.sin(th[0]) + Math.sin(th[0] + th[1]);
    expectClose(T, RpToTrans(RotZ(th[0] + th[1]), [x, y, 0]), 1e-12);
  });

  it('spaceJointTransforms ends at the FK pose', () => {
    const theta = randVec(rng, 6, 2);
    const P = spaceJointTransforms(ur5.Slist, theta);
    expect(P.length).toBe(6);
    expectClose(matMul(P[5], ur5.M), FKinSpace(ur5.M, ur5.Slist, theta), 1e-12);
  });
});

function numericalBodyJacobian(M: Mat, Blist: Mat, theta: Vec, h = 1e-6): Mat {
  const n = theta.length;
  const T = FKinBody(M, Blist, theta);
  const Tinv = TransInv(T);
  const J: Mat = Array.from({ length: 6 }, () => new Array(n).fill(0));
  for (let j = 0; j < n; j++) {
    const tp = theta.slice();
    const tm = theta.slice();
    tp[j] += h;
    tm[j] -= h;
    const dT = matScale(matSub(FKinBody(M, Blist, tp), FKinBody(M, Blist, tm)), 1 / (2 * h));
    const col = se3ToVec(matMul(Tinv, dT));
    for (let r = 0; r < 6; r++) J[r][j] = col[r];
  }
  return J;
}

function numericalSpaceJacobian(M: Mat, Slist: Mat, theta: Vec, h = 1e-6): Mat {
  const n = theta.length;
  const Tinv = TransInv(FKinSpace(M, Slist, theta));
  const J: Mat = Array.from({ length: 6 }, () => new Array(n).fill(0));
  for (let j = 0; j < n; j++) {
    const tp = theta.slice();
    const tm = theta.slice();
    tp[j] += h;
    tm[j] -= h;
    const dT = matScale(matSub(FKinSpace(M, Slist, tp), FKinSpace(M, Slist, tm)), 1 / (2 * h));
    const col = se3ToVec(matMul(dT, Tinv));
    for (let r = 0; r < 6; r++) J[r][j] = col[r];
  }
  return J;
}

describe('Jacobians', () => {
  it('match central finite differences of the forward kinematics', () => {
    for (const model of models) {
      for (let i = 0; i < 5; i++) {
        const theta = randVec(rng, model.dof, 2);
        expectClose(JacobianBody(model.Blist, theta), numericalBodyJacobian(model.M, model.Blist, theta), 1e-6, `${model.name} Jb`);
        expectClose(JacobianSpace(model.Slist, theta), numericalSpaceJacobian(model.M, model.Slist, theta), 1e-6, `${model.name} Js`);
      }
    }
  });

  it('J_b = Ad(T^-1) J_s', () => {
    for (const model of models) {
      for (let i = 0; i < 5; i++) {
        const theta = randVec(rng, model.dof, Math.PI);
        const T = FKinSpace(model.M, model.Slist, theta);
        expectClose(JacobianBody(model.Blist, theta), matMul(Adjoint(TransInv(T)), JacobianSpace(model.Slist, theta)), 1e-11, model.name);
      }
    }
  });

  it('first space-Jacobian column is S1 and last body-Jacobian column is Bn', () => {
    const theta = randVec(rng, 6, 2);
    expectClose(getCol(JacobianSpace(ur5.Slist, theta), 0), getCol(ur5.Slist, 0), 1e-15);
    expectClose(getCol(JacobianBody(ur5.Blist, theta), 5), getCol(ur5.Blist, 5), 1e-15);
  });

  it('manipulability detects the stretched-out singularity of planar2R', () => {
    const Jsing = JacobianBody(planar2R.Blist, [0.3, 0]);
    const lin = [Jsing[3], Jsing[4]]; // in-plane linear part (x, y of the body frame)
    const m = manipulability(lin);
    expect(m.sigmaMin).toBeLessThan(1e-12);
    expect(m.isotropy).toBe(Infinity);
    const Jok = JacobianBody(planar2R.Blist, [0.3, Math.PI / 2]);
    const mok = manipulability([Jok[3], Jok[4]]);
    expect(mok.sigmaMin).toBeGreaterThan(0.5);
    expect(mok.volume).toBeGreaterThan(0.5);
  });
});

describe('inverse kinematics', () => {
  it('converges for a reachable planar3R target (body and space forms)', () => {
    const thetaStar = [0.3, -0.5, 0.8];
    const T = FKinSpace(planar3R.M, planar3R.Slist, thetaStar);
    const theta0 = [0.5, -0.8, 0.9];
    for (const solve of [
      () => IKinBody(planar3R.Blist, planar3R.M, T, theta0, 1e-6, 1e-6),
      () => IKinSpace(planar3R.Slist, planar3R.M, T, theta0, 1e-6, 1e-6),
    ]) {
      const r = solve();
      expect(r.success).toBe(true);
      expect(r.iterations).toBeLessThanOrEqual(20);
      expect(r.history.length).toBe(r.iterations + 1);
      expect(r.history[r.iterations].errorV).toBeLessThan(r.history[0].errorV);
      expectClose(FKinSpace(planar3R.M, planar3R.Slist, r.thetalist), T, 1e-5);
    }
  });

  it('converges for the UR5 from a perturbed initial guess, with and without damping', () => {
    for (let trial = 0; trial < 5; trial++) {
      const thetaStar = randVec(rng, 6, 1.2);
      const T = FKinSpace(ur5.M, ur5.Slist, thetaStar);
      const theta0 = thetaStar.map((x) => x + 0.15 * (2 * rng() - 1));
      const r1 = IKinBody(ur5.Blist, ur5.M, T, theta0, 1e-6, 1e-6);
      expect(r1.success, `plain trial ${trial}`).toBe(true);
      expectClose(FKinBody(ur5.M, ur5.Blist, r1.thetalist), T, 1e-5, 'plain FK');
      const r2 = IKinSpace(ur5.Slist, ur5.M, T, theta0, 1e-6, 1e-6, { lambda: 0.05, maxIterations: 200 });
      expect(r2.success, `damped trial ${trial}`).toBe(true);
      expectClose(FKinSpace(ur5.M, ur5.Slist, r2.thetalist), T, 1e-5, 'damped FK');
      expect(r2.history.length).toBe(r2.iterations + 1);
    }
  });

  it('reports failure for unreachable targets', () => {
    const farPlanar = RpToTrans(eye(3), [5, 0, 0]); // planar3R reach is 2.4 m
    const r = IKinBody(planar3R.Blist, planar3R.M, farPlanar, [0.1, 0.1, 0.1], 1e-4, 1e-4, { maxIterations: 30 });
    expect(r.success).toBe(false);
    expect(r.iterations).toBe(30);
    expect(r.history.length).toBe(31);
    const farUr5 = RpToTrans(eye(3), [3, 0, 0]);
    const r2 = IKinSpace(ur5.Slist, ur5.M, farUr5, new Array(6).fill(0.1), 1e-4, 1e-4, { maxIterations: 25, lambda: 0.1 });
    expect(r2.success).toBe(false);
    const d = norm(matVec(eye(3), [1, 1, 1])); // keep helper imports honest
    expect(d).toBeGreaterThan(1);
  });

  it('maxStep caps the joint-space step size', () => {
    const thetaStar = [0.9, -0.7, 0.4];
    const T = FKinSpace(planar3R.M, planar3R.Slist, thetaStar);
    const r = IKinBody(planar3R.Blist, planar3R.M, T, [0, 0, 0], 1e-6, 1e-6, { maxStep: 0.1, maxIterations: 200 });
    for (let k = 1; k < r.history.length; k++) {
      const step = norm(r.history[k].theta.map((x, i) => x - r.history[k - 1].theta[i]));
      expect(step).toBeLessThanOrEqual(0.1 + 1e-12);
    }
    expect(r.success).toBe(true);
  });
});
