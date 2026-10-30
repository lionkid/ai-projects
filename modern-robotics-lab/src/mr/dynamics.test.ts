import { describe, expect, it } from 'vitest';
import { diag, eigSym, eye, matMul, transpose, zerosVec, type Mat, type Vec } from './linalg';
import { FKinSpace } from './kinematics';
import {
  EndEffectorForces,
  EulerStep,
  ForwardDynamics,
  ForwardDynamicsTrajectory,
  GravityForces,
  InverseDynamics,
  InverseDynamicsTrajectory,
  KineticEnergy,
  MassMatrix,
  PotentialEnergy,
  TotalEnergy,
  VelQuadraticForces,
  linkFramePoses,
  rk4Step,
} from './dynamics';
import { buildPlanarChain, planar2RDynamics, planar2RParams, type PlanarLinkModel } from './planar';
import { planar2R, rrr3, ur5 } from './robots';
import { expectClose, mulberry32, randVec } from './testUtils';

const rng = mulberry32(4242);

/**
 * Three-link reference fixture with the published numerical results of the
 * open-source Modern Robotics library (same algorithm, independent code), used
 * as a regression check of the recursive Newton-Euler implementation.
 */
const ref = (() => {
  const Mlist: Mat[] = [
    [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.089159], [0, 0, 0, 1]],
    [[0, 0, 1, 0.28], [0, 1, 0, 0.13585], [-1, 0, 0, 0], [0, 0, 0, 1]],
    [[1, 0, 0, 0], [0, 1, 0, -0.1197], [0, 0, 1, 0.395], [0, 0, 0, 1]],
    [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.14225], [0, 0, 0, 1]],
  ];
  const Glist: Mat[] = [
    diag([0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7]),
    diag([0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393]),
    diag([0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275]),
  ];
  const Slist: Mat = transpose([
    [1, 0, 1, 0, 1, 0],
    [0, 1, 0, -0.089, 0, 0],
    [0, 1, 0, -0.089, 0, 0.425],
  ]);
  return {
    Mlist,
    Glist,
    Slist,
    theta: [0.1, 0.1, 0.1],
    dtheta: [0.1, 0.2, 0.3],
    ddtheta: [2, 1.5, 1],
    g: [0, 0, -9.8],
    Ftip: [1, 1, 1, 1, 1, 1],
  };
})();

describe('Newton-Euler reference values', () => {
  it('InverseDynamics', () => {
    const tau = InverseDynamics(ref.theta, ref.dtheta, ref.ddtheta, ref.g, ref.Ftip, ref.Mlist, ref.Glist, ref.Slist);
    expectClose(tau, [74.69616155, -33.06766016, -3.23057314], 1e-6);
  });
  it('MassMatrix / VelQuadraticForces / GravityForces / EndEffectorForces', () => {
    expectClose(MassMatrix(ref.theta, ref.Mlist, ref.Glist, ref.Slist), [
      [22.543338, -0.307146754, -0.00718426391],
      [-0.307146754, 1.96850717, 0.432157368],
      [-0.00718426391, 0.432157368, 0.191630858],
    ], 1e-7);
    expectClose(VelQuadraticForces(ref.theta, ref.dtheta, ref.Mlist, ref.Glist, ref.Slist), [0.26453118, -0.05505157, -0.00689132], 1e-6);
    expectClose(GravityForces(ref.theta, ref.g, ref.Mlist, ref.Glist, ref.Slist), [28.40331262, -37.64094817, -5.4415892], 1e-6);
    expectClose(EndEffectorForces(ref.theta, ref.Ftip, ref.Mlist, ref.Glist, ref.Slist), [1.40954608, 1.85771497, 1.392409], 1e-6);
  });
  it('ForwardDynamics', () => {
    const dd = ForwardDynamics(ref.theta, ref.dtheta, [0.5, 0.6, 0.7], ref.g, ref.Ftip, ref.Mlist, ref.Glist, ref.Slist);
    expectClose(dd, [-0.97392907, 25.58466784, -32.91499212], 1e-6);
  });
});

describe('mass matrix', () => {
  it('is symmetric positive definite for all predefined robots', () => {
    for (const model of [planar2R, rrr3, ur5]) {
      for (let i = 0; i < 5; i++) {
        const theta = randVec(rng, model.dof, Math.PI);
        const M = MassMatrix(theta, model.Mlist!, model.Glist!, model.Slist);
        expectClose(M, transpose(M), 1e-9, `${model.name} symmetry`);
        const { values } = eigSym(M);
        expect(values[values.length - 1], `${model.name} min eigenvalue`).toBeGreaterThan(1e-6);
      }
    }
  });
});

describe('forward / inverse dynamics consistency', () => {
  it('InverseDynamics(ForwardDynamics(tau)) == tau', () => {
    for (const model of [planar2R, rrr3, ur5]) {
      for (let i = 0; i < 5; i++) {
        const theta = randVec(rng, model.dof, Math.PI);
        const dtheta = randVec(rng, model.dof, 2);
        const tau = randVec(rng, model.dof, 10);
        const Ftip = randVec(rng, 6, 2);
        const g = [0, 0, -9.81];
        const dd = ForwardDynamics(theta, dtheta, tau, g, Ftip, model.Mlist!, model.Glist!, model.Slist);
        const tauBack = InverseDynamics(theta, dtheta, dd, g, Ftip, model.Mlist!, model.Glist!, model.Slist);
        expectClose(tauBack, tau, 1e-8, model.name);
      }
    }
  });

  it('matches the closed-form planar 2R equations (point-mass and rod models)', () => {
    const g = 9.81;
    const gvec = [0, -g, 0];
    for (const model of ['point-mass', 'rod'] as PlanarLinkModel[]) {
      const L: [number, number] = [1.0, 1.2];
      const m: [number, number] = [1.5, 0.7];
      const chain = buildPlanarChain(L, { masses: m, model });
      const P = planar2RParams(L, m, model, g);
      for (let i = 0; i < 6; i++) {
        const theta = randVec(rng, 2, Math.PI);
        const dtheta = randVec(rng, 2, 3);
        const cf = planar2RDynamics(P, theta, dtheta);
        expectClose(MassMatrix(theta, chain.Mlist, chain.Glist, chain.Slist), cf.M, 1e-9, `${model} M`);
        expectClose(VelQuadraticForces(theta, dtheta, chain.Mlist, chain.Glist, chain.Slist), cf.c, 1e-9, `${model} c`);
        expectClose(GravityForces(theta, gvec, chain.Mlist, chain.Glist, chain.Slist), cf.g, 1e-9, `${model} g`);
        const ddtheta = randVec(rng, 2, 4);
        const tauCF = [
          cf.M[0][0] * ddtheta[0] + cf.M[0][1] * ddtheta[1] + cf.c[0] + cf.g[0],
          cf.M[1][0] * ddtheta[0] + cf.M[1][1] * ddtheta[1] + cf.c[1] + cf.g[1],
        ];
        expectClose(InverseDynamics(theta, dtheta, ddtheta, gvec, zerosVec(6), chain.Mlist, chain.Glist, chain.Slist), tauCF, 1e-9, `${model} tau`);
      }
    }
  });
});

describe('integrators and energy', () => {
  it('rk4Step integrates x\' = x with 4th-order accuracy', () => {
    const f = (_t: number, x: Vec) => x;
    const errorFor = (dt: number) => {
      let x = [1];
      const steps = Math.round(1 / dt);
      for (let i = 0; i < steps; i++) x = rk4Step(f, i * dt, x, dt);
      return Math.abs(x[0] - Math.E);
    };
    const e1 = errorFor(0.1);
    const e2 = errorFor(0.05);
    expect(e1).toBeLessThan(1e-5);
    // Halving dt must shrink the global error by about 2^4 = 16.
    expect(e1 / e2).toBeGreaterThan(12);
    expect(e1 / e2).toBeLessThan(20);
    const [th, dth] = EulerStep([1, 2], [0.5, -0.5], [1, 1], 0.1);
    expectClose(th, [1.05, 1.95], 1e-15);
    expectClose(dth, [0.6, -0.4], 1e-15);
  });

  it('conserves energy in a frictionless 2R free swing with rk4, while Euler drifts more', () => {
    const chain = buildPlanarChain([1, 1], { masses: [1, 1], model: 'rod' });
    const g = [0, -9.81, 0];
    const theta0 = [1.0, 0.5];
    const dtheta0 = [0, 0];
    const E0 = TotalEnergy(theta0, dtheta0, g, chain.Mlist, chain.Glist, chain.Slist);
    expect(Math.abs(E0)).toBeGreaterThan(1);
    const dt = 2e-3;
    const N = Math.round(5 / dt) + 1;
    const taumat: Vec[] = Array.from({ length: N }, () => [0, 0]);
    const Ftipmat: Vec[] = Array.from({ length: N }, () => zerosVec(6));
    const drift = (integrator: 'rk4' | 'euler') => {
      const { thetamat, dthetamat } = ForwardDynamicsTrajectory(theta0, dtheta0, taumat, g, Ftipmat, chain.Mlist, chain.Glist, chain.Slist, dt, 1, integrator);
      expect(thetamat.length).toBe(N);
      let worst = 0;
      for (let k = 0; k < N; k += 250) {
        const E = TotalEnergy(thetamat[k], dthetamat[k], g, chain.Mlist, chain.Glist, chain.Slist);
        worst = Math.max(worst, Math.abs(E - E0) / Math.abs(E0));
      }
      const Eend = TotalEnergy(thetamat[N - 1], dthetamat[N - 1], g, chain.Mlist, chain.Glist, chain.Slist);
      return Math.max(worst, Math.abs(Eend - E0) / Math.abs(E0));
    };
    const rk4Drift = drift('rk4');
    const eulerDrift = drift('euler');
    expect(rk4Drift).toBeLessThan(1e-3);
    expect(eulerDrift).toBeGreaterThan(rk4Drift * 10);
  });

  it('kinetic energy is zero at rest and potential energy follows the COM heights', () => {
    const chain = buildPlanarChain([1, 1], { masses: [2, 3], model: 'rod' });
    const g = [0, -9.81, 0];
    expect(KineticEnergy([0.3, 0.2], [0, 0], chain.Mlist, chain.Glist, chain.Slist)).toBe(0);
    // Arm pointing straight up: COMs at heights 0.5 and 1.5.
    const U = PotentialEnergy([Math.PI / 2, 0], g, chain.Mlist, chain.Glist, chain.Slist);
    expect(U).toBeCloseTo(9.81 * (2 * 0.5 + 3 * 1.5), 9);
    expect(KineticEnergy([0, 0], [1, 0], chain.Mlist, chain.Glist, chain.Slist)).toBeGreaterThan(0);
  });

  it('linkFramePoses ends at the end-effector FK and InverseDynamicsTrajectory maps rows', () => {
    const theta = randVec(rng, 3, 1);
    const poses = linkFramePoses(theta, rrr3.Mlist!, rrr3.Slist);
    expect(poses.length).toBe(4);
    let Mhome = eye(4);
    for (const Mi of rrr3.Mlist!) Mhome = matMul(Mhome, Mi);
    expectClose(Mhome, rrr3.M, 1e-12, 'Mlist product equals M');
    expectClose(poses[3], FKinSpace(rrr3.M, rrr3.Slist, theta), 1e-12);
    const taus = InverseDynamicsTrajectory([theta, theta], [[0, 0, 0], [0, 0, 0]], [[0, 0, 0], [0, 0, 0]], [0, 0, -9.81], [zerosVec(6), zerosVec(6)], rrr3.Mlist!, rrr3.Glist!, rrr3.Slist);
    expect(taus.length).toBe(2);
    expectClose(taus[0], GravityForces(theta, [0, 0, -9.81], rrr3.Mlist!, rrr3.Glist!, rrr3.Slist), 1e-12);
  });
});
