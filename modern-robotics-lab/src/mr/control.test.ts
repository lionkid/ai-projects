import { describe, expect, it } from 'vitest';
import { norm, zerosVec, type Vec } from './linalg';
import { InverseDynamics } from './dynamics';
import { JointTrajectory, QuinticTimeScalingDerivs } from './trajectory';
import { ComputedTorque, SimulateControl, createPID, pidInitialState, pidStep } from './control';
import { buildPlanarChain } from './planar';
import { planar2R } from './robots';
import { expectClose } from './testUtils';

const g = [0, -9.81, 0];

describe('ComputedTorque', () => {
  it('reduces to inverse dynamics when the error is zero', () => {
    const theta = [0.4, -0.2];
    const dtheta = [0.3, 0.1];
    const dd = [1, -2];
    const tau = ComputedTorque(theta, dtheta, [0, 0], g, planar2R.Mlist!, planar2R.Glist!, planar2R.Slist, theta, dtheta, dd, 10, 1, 2);
    expectClose(tau, InverseDynamics(theta, dtheta, dd, g, zerosVec(6), planar2R.Mlist!, planar2R.Glist!, planar2R.Slist), 1e-12);
  });

  it('accepts per-joint gain vectors', () => {
    const theta = [0.4, -0.2];
    const tauScalar = ComputedTorque(theta, [0, 0], [0, 0], g, planar2R.Mlist!, planar2R.Glist!, planar2R.Slist, [0.5, 0], [0, 0], [0, 0], 10, 0, 0);
    const tauVec = ComputedTorque(theta, [0, 0], [0, 0], g, planar2R.Mlist!, planar2R.Glist!, planar2R.Slist, [0.5, 0], [0, 0], [0, 0], [10, 10], 0, 0);
    expectClose(tauVec, tauScalar, 1e-12);
  });
});

describe('SimulateControl', () => {
  const { Mlist, Glist, Slist } = planar2R;
  const dt = 0.005;

  it('tracks a quintic joint trajectory with an exact model', () => {
    const Tf = 1;
    const N = Math.round(Tf / dt) + 1;
    const thetaStart = [0, 0];
    const thetaEnd = [1.0, -0.6];
    const thetamatd = JointTrajectory(thetaStart, thetaEnd, Tf, N, 5);
    const dthetamatd: Vec[] = [];
    const ddthetamatd: Vec[] = [];
    for (let k = 0; k < N; k++) {
      const { sdot, sddot } = QuinticTimeScalingDerivs(Tf, (Tf * k) / (N - 1));
      dthetamatd.push(thetaEnd.map((x, i) => (x - thetaStart[i]) * sdot));
      ddthetamatd.push(thetaEnd.map((x, i) => (x - thetaStart[i]) * sddot));
    }
    const Ftipmat: Vec[] = Array.from({ length: N }, () => zerosVec(6));
    const res = SimulateControl(
      [0.1, -0.1], [0, 0], g, Ftipmat, Mlist!, Glist!, Slist,
      thetamatd, dthetamatd, ddthetamatd, g, Mlist!, Glist!, 100, 0, 20, dt, 2,
    );
    expect(res.taumat.length).toBe(N);
    expect(res.thetamat.length).toBe(N);
    expect(res.dthetamat.length).toBe(N);
    expectClose(res.thetamat[0], [0.1, -0.1], 0);
    expect(norm(res.errmat[0])).toBeCloseTo(norm([0.1, 0.1]), 12);
    expect(norm(res.errmat[N - 1])).toBeLessThan(1e-3);
    expectClose(res.thetamat[N - 1], thetaEnd, 1e-3);
  });

  it('integral action removes the steady-state error caused by a mass mismatch', () => {
    const Tf = 3;
    const N = Math.round(Tf / dt) + 1;
    const setpoint = [0.5, 0.3];
    const thetamatd: Vec[] = Array.from({ length: N }, () => setpoint.slice());
    const zero: Vec[] = Array.from({ length: N }, () => [0, 0]);
    const Ftipmat: Vec[] = Array.from({ length: N }, () => zerosVec(6));
    const heavy = buildPlanarChain([1, 1], { masses: [1.3, 1.3], model: 'point-mass' });
    const run = (Ki: number) =>
      SimulateControl(
        setpoint, [0, 0], g, Ftipmat, Mlist!, Glist!, Slist,
        thetamatd, zero, zero, g, heavy.Mlist, heavy.Glist, 100, Ki, 20, dt, 1,
      );
    const noI = run(0);
    const withI = run(200);
    const eNoI = norm(noI.errmat[N - 1]);
    const eWithI = norm(withI.errmat[N - 1]);
    expect(eNoI).toBeGreaterThan(2e-3);
    expect(eWithI).toBeLessThan(2e-3);
    expect(eWithI).toBeLessThan(eNoI);
  });
});

describe('PID helper', () => {
  it('proportional and derivative terms behave as expected', () => {
    const pid = createPID({ kp: 2, ki: 0, kd: 0 });
    expect(pid.update(1.5, 0.1)).toBeCloseTo(3, 12);
    const pd = createPID({ kp: 0, ki: 0, kd: 1 });
    expect(pd.update(1, 0.1)).toBe(0); // no previous error yet
    expect(pd.update(2, 0.1)).toBeCloseTo(10, 12);
    pd.reset();
    expect(pd.state.prevError).toBeNull();
  });

  it('integrates and respects the integral limit', () => {
    const r1 = pidStep({ kp: 0, ki: 1, kd: 0, integralLimit: 0.25 }, pidInitialState(), 1, 0.1);
    expect(r1.output).toBeCloseTo(0.1, 12);
    let st = r1.state;
    for (let i = 0; i < 10; i++) st = pidStep({ kp: 0, ki: 1, kd: 0, integralLimit: 0.25 }, st, 1, 0.1).state;
    expect(st.integral).toBeCloseTo(0.25, 12);
  });

  it('anti-windup stops integrating while the output is saturated', () => {
    const opts = { kp: 1, ki: 10, kd: 0, outputMax: 2, outputMin: -2 };
    const pid = createPID(opts);
    for (let i = 0; i < 50; i++) expect(pid.update(1, 0.1)).toBeLessThanOrEqual(2);
    // Without anti-windup the integral would be 5; it must stay near the saturation point.
    expect(pid.state.integral).toBeLessThan(0.2);
    // Once the error reverses the output must respond immediately.
    expect(pid.update(-1, 0.1)).toBeLessThan(0);
  });

  it('derivative on measurement ignores setpoint jumps', () => {
    const opts = { kp: 0, ki: 0, kd: 1, derivativeOnMeasurement: true };
    let st = pidInitialState();
    st = pidStep(opts, st, 0, 0.1, 1).state;
    const r = pidStep(opts, st, 5, 0.1, 1); // setpoint jumped, measurement unchanged
    expect(r.output).toBe(0);
    const r2 = pidStep(opts, r.state, 5, 0.1, 1.5); // measurement rises -> negative derivative term
    expect(r2.output).toBeCloseTo(-5, 12);
  });
});
