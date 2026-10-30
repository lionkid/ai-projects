/**
 * Independent adversarial review tests for dynamics.ts.
 *
 * The Newton-Euler results are checked against a Lagrangian reconstruction
 * that never calls InverseDynamics:
 *   - M(theta) from the kinetic energy 1/2 sum_i V_i^T G_i V_i, with the link
 *     body twists V_i obtained by finite differences of the link frame poses;
 *   - g(theta) = dU/dtheta by finite differences of the potential energy;
 *   - c(theta, thetadot) from the Christoffel symbols of M(theta);
 *   - tip-wrench torques from J_b^T F_tip;
 * plus hand-computed planar values and a power-balance test of ForwardDynamics.
 */
import { describe, expect, it } from 'vitest';
import {
  type Mat,
  type Vec,
  Adjoint,
  EndEffectorForces,
  EulerStep,
  ForwardDynamics,
  ForwardDynamicsTrajectory,
  GravityForces,
  InverseDynamics,
  JacobianBody,
  JacobianSpace,
  KineticEnergy,
  MassMatrix,
  PotentialEnergy,
  TotalEnergy,
  TransInv,
  VelQuadraticForces,
  dot,
  eulerStepGeneric,
  linkFramePoses,
  matMul,
  matScale,
  matSub,
  matVec,
  norm,
  planar2R,
  planar2RAccel,
  planar2RParams,
  planar3R,
  planarTipJacobian,
  rk4Step,
  rrr3,
  se3ToVec,
  transpose,
  ur5,
  vecAdd,
  vecScale,
  vecSub,
  zerosVec,
  type RobotModel,
} from './index';
import { expectClose, mulberry32, randVec } from './testUtils';

const rng = mulberry32(424242);

function unit(n: number, i: number): Vec {
  const e = zerosVec(n);
  e[i] = 1;
  return e;
}

/** Body twists of link frames 1..n by central differences of linkFramePoses along thetadot. */
function kineticEnergyFD(model: RobotModel, theta: Vec, dtheta: Vec, delta = 1e-6): number {
  const Mlist = model.Mlist!;
  const Glist = model.Glist!;
  const plus = linkFramePoses(vecAdd(theta, vecScale(dtheta, delta)), Mlist, model.Slist);
  const minus = linkFramePoses(vecSub(theta, vecScale(dtheta, delta)), Mlist, model.Slist);
  const T0 = linkFramePoses(theta, Mlist, model.Slist);
  let ke = 0;
  for (let i = 0; i < Glist.length; i++) {
    const Tdot = matScale(matSub(plus[i], minus[i]), 1 / (2 * delta));
    const V = se3ToVec(matMul(TransInv(T0[i]), Tdot));
    ke += 0.5 * dot(V, matVec(Glist[i], V));
  }
  return ke;
}

/** Mass matrix by polarisation of the finite-difference kinetic energy. */
function massMatrixFD(model: RobotModel, theta: Vec): Mat {
  const n = model.dof;
  const M: Mat = [];
  for (let i = 0; i < n; i++) {
    M.push([]);
    for (let j = 0; j < n; j++) {
      if (i === j) M[i].push(2 * kineticEnergyFD(model, theta, unit(n, i)));
      else {
        M[i].push(
          kineticEnergyFD(model, theta, vecAdd(unit(n, i), unit(n, j))) -
            kineticEnergyFD(model, theta, unit(n, i)) -
            kineticEnergyFD(model, theta, unit(n, j)),
        );
      }
    }
  }
  return M;
}

/** dM/dtheta_k by central differences of the (Newton-Euler) mass matrix. */
function dMassFD(model: RobotModel, theta: Vec, k: number, delta = 1e-5): Mat {
  const n = model.dof;
  const e = vecScale(unit(n, k), delta);
  const Mp = MassMatrix(vecAdd(theta, e), model.Mlist!, model.Glist!, model.Slist);
  const Mm = MassMatrix(vecSub(theta, e), model.Mlist!, model.Glist!, model.Slist);
  return matScale(matSub(Mp, Mm), 1 / (2 * delta));
}

/** c(theta, thetadot) = Mdot thetadot - 1/2 d/dtheta (thetadot^T M thetadot). */
function coriolisChristoffel(model: RobotModel, theta: Vec, dtheta: Vec): Vec {
  const n = model.dof;
  const dM = Array.from({ length: n }, (_, k) => dMassFD(model, theta, k));
  const c = zerosVec(n);
  for (let i = 0; i < n; i++) {
    for (let j = 0; j < n; j++) {
      for (let k = 0; k < n; k++) {
        const gamma = 0.5 * (dM[k][i][j] + dM[j][i][k] - dM[i][j][k]);
        c[i] += gamma * dtheta[j] * dtheta[k];
      }
    }
  }
  return c;
}

function gravityFD(model: RobotModel, theta: Vec, g: Vec, delta = 1e-6): Vec {
  const n = model.dof;
  return Array.from({ length: n }, (_, i) => {
    const e = vecScale(unit(n, i), delta);
    const Up = PotentialEnergy(vecAdd(theta, e), g, model.Mlist!, model.Glist!, model.Slist);
    const Um = PotentialEnergy(vecSub(theta, e), g, model.Mlist!, model.Glist!, model.Slist);
    return (Up - Um) / (2 * delta);
  });
}

const dynModels = [planar2R, planar3R, rrr3, ur5];

describe('review: Newton-Euler vs Lagrangian reconstruction', () => {
  it('MassMatrix equals the kinetic-energy polarisation of finite-difference link twists', () => {
    for (const model of dynModels) {
      for (let k = 0; k < 3; k++) {
        const theta = randVec(rng, model.dof, 2.5);
        const Mne = MassMatrix(theta, model.Mlist!, model.Glist!, model.Slist);
        const Mfd = massMatrixFD(model, theta);
        expectClose(Mne, Mfd, 2e-6, `${model.name} k=${k}`);
        const dtheta = randVec(rng, model.dof, 1.5);
        expect(KineticEnergy(theta, dtheta, model.Mlist!, model.Glist!, model.Slist)).toBeCloseTo(
          kineticEnergyFD(model, theta, dtheta),
          6,
        );
      }
    }
  });

  it('GravityForces equals dU/dtheta for vertical AND tilted gravity vectors', () => {
    for (const model of dynModels) {
      for (const g of [
        [0, 0, -9.81],
        [0, -9.81, 0],
        [1.5, -2.5, -9.0],
      ]) {
        const theta = randVec(rng, model.dof, 2.5);
        const gNE = GravityForces(theta, g, model.Mlist!, model.Glist!, model.Slist);
        expectClose(gNE, gravityFD(model, theta, g), 1e-6, `${model.name} g=${g}`);
      }
    }
  });

  it('VelQuadraticForces equals the Christoffel form built from M(theta)', () => {
    for (const model of dynModels) {
      for (let k = 0; k < 3; k++) {
        const theta = randVec(rng, model.dof, 2.5);
        const dtheta = randVec(rng, model.dof, 2);
        const cNE = VelQuadraticForces(theta, dtheta, model.Mlist!, model.Glist!, model.Slist);
        expectClose(cNE, coriolisChristoffel(model, theta, dtheta), 2e-6, `${model.name} k=${k}`);
      }
    }
  });

  it('EndEffectorForces equals J_b^T F_tip, with J_b taken at the Newton-Euler end-effector frame', () => {
    for (const model of dynModels) {
      const theta = randVec(rng, model.dof, 2.5);
      const F = randVec(rng, 6, 3);
      const tauNE = EndEffectorForces(theta, F, model.Mlist!, model.Glist!, model.Slist);
      // End-effector frame as the dynamics sees it: last link-frame pose.
      const Tee = linkFramePoses(theta, model.Mlist!, model.Slist)[model.dof];
      const JbNE = matMul(Adjoint(TransInv(Tee)), JacobianSpace(model.Slist, theta));
      expectClose(tauNE, matVec(transpose(JbNE), F), 1e-9, `${model.name} (NE frame)`);
      if (!model.dynamicsApproximate) {
        expectClose(tauNE, matVec(transpose(JacobianBody(model.Blist, theta)), F), 1e-9, `${model.name} (Blist)`);
      }
    }
  });

  it('InverseDynamics is the sum M thetaddot + c + g + J^T F of the verified pieces (superposition)', () => {
    for (const model of dynModels) {
      const theta = randVec(rng, model.dof, 2.5);
      const dtheta = randVec(rng, model.dof, 2);
      const ddtheta = randVec(rng, model.dof, 3);
      const g = [0.3, -0.2, -9.81];
      const F = randVec(rng, 6, 2);
      const tau = InverseDynamics(theta, dtheta, ddtheta, g, F, model.Mlist!, model.Glist!, model.Slist);
      const sum = vecAdd(
        vecAdd(matVec(massMatrixFD(model, theta), ddtheta), coriolisChristoffel(model, theta, dtheta)),
        vecAdd(gravityFD(model, theta, g), EndEffectorForces(theta, F, model.Mlist!, model.Glist!, model.Slist)),
      );
      expectClose(tau, sum, 1e-5, model.name);
    }
  });

  it('UR5 Blist-based J_b^T F equals Newton-Euler EndEffectorForces (model made exactly self-consistent)', () => {
    // This test originally documented a ~1 mm x |F| discrepancy caused by mixing URDF link
    // offsets with the book's screw axes. The UR5 Mlist is now derived from the book
    // dimensions (URDF masses/inertias/COM placements kept), so the two must agree to rounding.
    for (let trial = 0; trial < 5; trial++) {
      const theta = randVec(rng, 6, 2);
      const F = trial === 0 ? [0, 0, 0, 10, 0, 0] : randVec(rng, 6, 10);
      const tauNE = EndEffectorForces(theta, F, ur5.Mlist!, ur5.Glist!, ur5.Slist);
      const tauJ = matVec(transpose(JacobianBody(ur5.Blist, theta)), F);
      expect(norm(vecSub(tauNE, tauJ)), `trial ${trial}`).toBeLessThan(1e-12);
    }
  });
});

describe('review: hand-computed planar values', () => {
  it('gravity torque of the horizontal planar2R (point masses) is (2 + 1) g and g', () => {
    const g = 9.81;
    const tau = GravityForces([0, 0], [0, -g, 0], planar2R.Mlist!, planar2R.Glist!, planar2R.Slist);
    expectClose(tau, [3 * g, g], 1e-12);
    // Hanging straight down: no gravity torque.
    expectClose(GravityForces([-Math.PI / 2, 0], [0, -g, 0], planar2R.Mlist!, planar2R.Glist!, planar2R.Slist), [0, 0], 1e-12);
    // Elbow bent up by 90 deg from horizontal: link 2 vertical -> only link masses on link 1 lever.
    expectClose(GravityForces([0, Math.PI / 2], [0, -g, 0], planar2R.Mlist!, planar2R.Glist!, planar2R.Slist), [2 * g, 0], 1e-12);
  });

  it('a 1 N push along space +x at theta = (0, pi/2) needs tau = (-1, -1) (F_tip given in the tool frame)', () => {
    // Tool frame rotated +90 deg: space +x is tool -y.
    const Ftip = [0, 0, 0, 0, -1, 0];
    const tau = EndEffectorForces([0, Math.PI / 2], Ftip, planar2R.Mlist!, planar2R.Glist!, planar2R.Slist);
    expectClose(tau, [-1, -1], 1e-12);
    // Cross-check via the planar tip Jacobian in the fixed frame.
    const Jt = planarTipJacobian([1, 1], [0, Math.PI / 2]);
    expectClose(matVec(transpose(Jt), [1, 0]), [-1, -1], 1e-12);
  });

  it('a pure tool-frame moment about z produces equal torques on all joints of a planar chain', () => {
    const tau = EndEffectorForces([0.3, -0.7, 1.1], [0, 0, 2.5, 0, 0, 0], planar3R.Mlist!, planar3R.Glist!, planar3R.Slist);
    expectClose(tau, [2.5, 2.5, 2.5], 1e-12);
  });

  it('ForwardDynamics with a tip wrench matches the closed-form 2R model with tau - J^T f', () => {
    const P = planar2RParams([1, 1], [1, 1], 'point-mass', 9.81);
    for (let k = 0; k < 5; k++) {
      const theta = randVec(rng, 2, 3);
      const dtheta = randVec(rng, 2, 2);
      const tau = randVec(rng, 2, 5);
      const fSpace = randVec(rng, 2, 3); // planar force applied by the tip to the environment, fixed frame
      const phi = theta[0] + theta[1];
      // Tool frame force components (tool x along the last link).
      const fTool = [Math.cos(phi) * fSpace[0] + Math.sin(phi) * fSpace[1], -Math.sin(phi) * fSpace[0] + Math.cos(phi) * fSpace[1], 0];
      const Ftip = [0, 0, 0, ...fTool];
      const ddNE = ForwardDynamics(theta, dtheta, tau, [0, -9.81, 0], Ftip, planar2R.Mlist!, planar2R.Glist!, planar2R.Slist);
      const Jt = planarTipJacobian([1, 1], theta);
      const ddCF = planar2RAccel(P, theta, dtheta, vecSub(tau, matVec(transpose(Jt), fSpace)));
      expectClose(ddNE, ddCF, 1e-9, `k=${k}`);
    }
  });
});

describe('review: forward dynamics, integrators, energy', () => {
  it('power balance: dE/dt along the forward-dynamics flow equals tau . thetadot (no tip wrench)', () => {
    for (const model of [rrr3, ur5, planar3R]) {
      const g = [0, 0, -9.81];
      const theta = randVec(rng, model.dof, 2);
      const dtheta = randVec(rng, model.dof, 1.5);
      const tau = randVec(rng, model.dof, 4);
      const dd = ForwardDynamics(theta, dtheta, tau, g, zerosVec(6), model.Mlist!, model.Glist!, model.Slist);
      const h = 1e-6;
      const E = (s: number) =>
        TotalEnergy(vecAdd(theta, vecScale(dtheta, s)), vecAdd(dtheta, vecScale(dd, s)), g, model.Mlist!, model.Glist!, model.Slist);
      const dEdt = (E(h) - E(-h)) / (2 * h);
      expect(Math.abs(dEdt - dot(tau, dtheta)), model.name).toBeLessThan(1e-5);
    }
  });

  it('a tip wrench removes power at the rate V_b . F_tip', () => {
    const model = rrr3;
    const g = [0, 0, -9.81];
    const theta = randVec(rng, 3, 2);
    const dtheta = randVec(rng, 3, 1.5);
    const tau = randVec(rng, 3, 4);
    const F = randVec(rng, 6, 2);
    const dd = ForwardDynamics(theta, dtheta, tau, g, F, model.Mlist!, model.Glist!, model.Slist);
    const h = 1e-6;
    const E = (s: number) =>
      TotalEnergy(vecAdd(theta, vecScale(dtheta, s)), vecAdd(dtheta, vecScale(dd, s)), g, model.Mlist!, model.Glist!, model.Slist);
    const dEdt = (E(h) - E(-h)) / (2 * h);
    const Vb = matVec(JacobianBody(model.Blist, theta), dtheta);
    expect(Math.abs(dEdt - (dot(tau, dtheta) - dot(Vb, F)))).toBeLessThan(1e-5);
  });

  it('rk4Step is exact to ~1e-10 on a harmonic oscillator over one period, Euler is first order', () => {
    const f = (_t: number, x: Vec) => [x[1], -x[0]];
    const period = 2 * Math.PI;
    const run = (stepper: typeof rk4Step, n: number) => {
      let x = [1, 0];
      const dt = period / n;
      for (let k = 0; k < n; k++) x = stepper(f, k * dt, x, dt);
      return norm(vecSub(x, [1, 0]));
    };
    expect(run(rk4Step, 2000)).toBeLessThan(1e-10);
    const e1 = run(eulerStepGeneric, 2000);
    const e2 = run(eulerStepGeneric, 4000);
    expect(e1 / e2).toBeGreaterThan(1.8);
    expect(e1 / e2).toBeLessThan(2.2);
    expectClose(EulerStep([1, 2], [3, 4], [5, 6], 0.1), [[1.3, 2.4], [3.5, 4.6]], 1e-15);
  });

  it('ForwardDynamicsTrajectory: N rows, row 0 is the initial state, rest stays at rest, matches a manual rk4 loop', () => {
    const model = rrr3;
    const g = [0, 0, -9.81];
    const N = 6;
    const theta0 = [0.2, -0.3, 0.4];
    const dtheta0 = [0, 0, 0];
    // Gravity-compensating constant torque keeps the arm at rest (zero velocity, zero acceleration).
    const tauHold = GravityForces(theta0, g, model.Mlist!, model.Glist!, model.Slist);
    const rest = ForwardDynamicsTrajectory(theta0, dtheta0, Array(N).fill(tauHold), g, Array(N).fill(zerosVec(6)), model.Mlist!, model.Glist!, model.Slist, 0.05, 4);
    expect(rest.thetamat.length).toBe(N);
    expect(rest.dthetamat.length).toBe(N);
    expectClose(rest.thetamat[0], theta0, 0);
    expectClose(rest.thetamat[N - 1], theta0, 1e-9);
    expectClose(rest.dthetamat[N - 1], [0, 0, 0], 1e-9);
    // Manual replay of a non-trivial torque history.
    const taumat = Array.from({ length: N }, () => randVec(rng, 3, 3));
    const sim = ForwardDynamicsTrajectory(theta0, dtheta0, taumat, g, Array(N).fill(zerosVec(6)), model.Mlist!, model.Glist!, model.Slist, 0.05, 2, 'rk4');
    let x = [...theta0, ...dtheta0];
    for (let k = 0; k < N - 1; k++) {
      const f = (_t: number, y: Vec) => {
        const th = y.slice(0, 3);
        const dth = y.slice(3, 6);
        return dth.concat(ForwardDynamics(th, dth, taumat[k], g, zerosVec(6), model.Mlist!, model.Glist!, model.Slist));
      };
      x = rk4Step(f, 0, x, 0.025);
      x = rk4Step(f, 0, x, 0.025);
      expectClose(sim.thetamat[k + 1], x.slice(0, 3), 1e-12, `row ${k + 1}`);
      expectClose(sim.dthetamat[k + 1], x.slice(3, 6), 1e-12, `row ${k + 1}`);
    }
  });

  it('rejects inconsistent Mlist / Glist lengths', () => {
    expect(() => InverseDynamics([0, 0], [0, 0], [0, 0], [0, 0, -9.81], zerosVec(6), planar2R.Mlist!.slice(0, 2), planar2R.Glist!, planar2R.Slist)).toThrow();
    expect(() => InverseDynamics([0, 0], [0, 0], [0, 0], [0, 0, -9.81], zerosVec(6), planar2R.Mlist!, planar2R.Glist!.slice(0, 1), planar2R.Slist)).toThrow();
  });

  it('potential energy zero level and sign: lifting a link raises U by m g dz', () => {
    const g = [0, 0, -9.81];
    const U0 = PotentialEnergy([0, 0, 0], g, rrr3.Mlist!, rrr3.Glist!, rrr3.Slist);
    const U1 = PotentialEnergy([0, -Math.PI / 2, 0], g, rrr3.Mlist!, rrr3.Glist!, rrr3.Slist); // arm points straight up
    const { L1, L2 } = { L1: 0.4, L2: 0.3 };
    // Link 2 COM rises from L0 to L0 + L1/2, link 3 COM from L0 to L0 + L1 + L2/2 (masses 1.5, 1.0 kg).
    expect(U1 - U0).toBeCloseTo(9.81 * (1.5 * (L1 / 2) + 1.0 * (L1 + L2 / 2)), 9);
  });
});
