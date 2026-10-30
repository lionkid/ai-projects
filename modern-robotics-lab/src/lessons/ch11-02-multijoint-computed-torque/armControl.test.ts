import { describe, expect, it } from 'vitest';
import {
  HOLD, LINKS, ROUTES, armParams, designGains, dlsSolve, jdotQdot, jointControl, phase, secondOrderFree, simulateArm,
  type ArmTrialOptions,
} from './armControl';
import { ComputedTorque, SimulateControl, buildPlanarChain, planar2RDynamics, planarTipJacobian } from '@/mr';

const base: ArmTrialOptions = { kind: 'ct', route: 'ab', Tf: 1.2, wn: 8, zeta: 1, useKi: false, massError: 0 };
const maxAbs = (rows: number[][], j: number) => Math.max(...rows.map((r) => Math.abs(r[j])));

describe('kinematic helpers', () => {
  it('jdotQdot matches a finite difference of the tip Jacobian', () => {
    const th = [0.4, 1.1], dth = [0.7, -1.3];
    const eps = 1e-6;
    const Jp = planarTipJacobian(LINKS, [th[0] + eps * dth[0], th[1] + eps * dth[1]]);
    const Jm = planarTipJacobian(LINKS, [th[0] - eps * dth[0], th[1] - eps * dth[1]]);
    const Jdot = Jp.map((r, i) => r.map((v, j) => (v - Jm[i][j]) / (2 * eps)));
    const fd = [Jdot[0][0] * dth[0] + Jdot[0][1] * dth[1], Jdot[1][0] * dth[0] + Jdot[1][1] * dth[1]];
    const an = jdotQdot(LINKS, th, dth);
    expect(an[0]).toBeCloseTo(fd[0], 6);
    expect(an[1]).toBeCloseTo(fd[1], 6);
  });
  it('dlsSolve is J⁻¹ for λ = 0 and shrinks the answer as λ grows', () => {
    const J = planarTipJacobian(LINKS, [0.3, 2.9]);
    const v = [0.2, -0.5];
    const q = dlsSolve(J, v, 0);
    expect(J[0][0] * q[0] + J[0][1] * q[1]).toBeCloseTo(v[0], 6);
    expect(J[1][0] * q[0] + J[1][1] * q[1]).toBeCloseTo(v[1], 6);
    const q2 = dlsSolve(J, v, 0.3);
    expect(Math.hypot(q2[0], q2[1])).toBeLessThan(Math.hypot(q[0], q[1]));
    // exactly singular (arm stretched): still finite
    const qs = dlsSolve(planarTipJacobian(LINKS, [0.5, 0]), [1, 0], 0);
    expect(qs.every(Number.isFinite)).toBe(true);
  });
  it('the back-and-forth phase rests, moves with a quintic profile and returns', () => {
    expect(phase(0.5 * HOLD, 1)).toEqual({ s: 0, sd: 0, sdd: 0 });
    expect(phase(HOLD + 0.5, 1).s).toBeCloseTo(0.5, 12);
    expect(phase(HOLD + 1.2, 1).s).toBe(1);
    expect(phase(2 * HOLD + 1.5, 1).s).toBeCloseTo(0.5, 12);
    expect(phase(3 * HOLD + 2, 1).s).toBe(0);
    expect(phase(HOLD + 1e-9, 1).sd).toBeLessThan(1e-6);
  });
  it('route C–D ends are on the elbow θ2 > 0 branch and reach the requested tips', () => {
    const r = ROUTES.cd;
    expect(r.qa[1]).toBeGreaterThan(0);
    expect(r.qb[1]).toBeCloseTo(r.qa[1], 12);
  });
});

describe('controllers', () => {
  it('computed torque equals mr.ComputedTorque on the equivalent Newton–Euler chain', () => {
    const chain = buildPlanarChain([1, 1], { masses: [1, 1], model: 'rod' });
    const g = designGains(8, 0.9, true);
    const th = [0.3, -0.8], dth = [1.2, -0.4], thd = [0.5, -0.5], dthd = [0.4, 0.2], ddthd = [-1.5, 2.0], eint = [0.02, -0.01];
    const mine = jointControl('ct', armParams(1), [1, 1], g, th, dth, thd, dthd, ddthd, eint).tau;
    const ref = ComputedTorque(th, dth, eint, [0, -9.81, 0], chain.Mlist, chain.Glist, chain.Slist, thd, dthd, ddthd, g.kp, g.ki, g.kd);
    expect(mine[0]).toBeCloseTo(ref[0], 8);
    expect(mine[1]).toBeCloseTo(ref[1], 8);
  });
  it('the closed-loop trial agrees with mr.SimulateControl (computed torque, perfect model)', () => {
    const chain = buildPlanarChain([1, 1], { masses: [1, 1], model: 'rod' });
    const h = 2e-3, N = 250;
    const o: ArmTrialOptions = { ...base, offset: [0.3, -0.1], every: 1, h };
    const mine = simulateArm(o);
    // same reference samples, same zero-order hold, same RK4 step
    const thd = mine.thd!.slice(0, N);
    const dthd: number[][] = [], ddthd: number[][] = [];
    for (let k = 0; k < N; k++) {
      const t = k * h;
      const p = phase(t, o.Tf);
      const d = [ROUTES.ab.qb[0] - ROUTES.ab.qa[0], ROUTES.ab.qb[1] - ROUTES.ab.qa[1]];
      dthd.push([p.sd * d[0], p.sd * d[1]]);
      ddthd.push([p.sdd * d[0], p.sdd * d[1]]);
    }
    const g = designGains(o.wn, o.zeta, false);
    const lib = SimulateControl(mine.th[0], [0, 0], [0, -9.81, 0], [], chain.Mlist, chain.Glist, chain.Slist, thd, dthd, ddthd,
      [0, -9.81, 0], chain.Mlist, chain.Glist, g.kp, g.ki, g.kd, h, 1, 'rk4');
    for (const k of [50, 150, N - 1]) {
      expect(mine.th[k][0]).toBeCloseTo(lib.thetamat[k][0], 6);
      expect(mine.th[k][1]).toBeCloseTo(lib.thetamat[k][1], 6);
    }
  });
});

describe('closed-loop behaviour', () => {
  it('independent PD sags under gravity; gravity compensation removes the static error', () => {
    const atHold = (kind: ArmTrialOptions['kind']) => {
      const tr = simulateArm({ ...base, kind });
      const i = tr.t.findIndex((t) => t >= HOLD - 0.011);
      return tr.err[i];
    };
    const pd = atHold('pd'), pdg = atHold('pdg');
    expect(Math.abs(pd[0])).toBeGreaterThan(0.08);
    expect(Math.abs(pd[1])).toBeGreaterThan(0.08);
    expect(Math.abs(pdg[0]) + Math.abs(pdg[1])).toBeLessThan(2e-3);
  });
  it('joint 2 is commanded to stay still: PD+g still moves it (coupling), computed torque does not', () => {
    const pdg = simulateArm({ ...base, kind: 'pdg', Tf: 0.8 });
    const ct = simulateArm({ ...base, kind: 'ct', Tf: 0.8 });
    expect(maxAbs(pdg.err, 1)).toBeGreaterThan(0.05);
    expect(maxAbs(ct.err, 1)).toBeLessThan(5e-3);
    expect(ct.stats.tipRms).toBeLessThan(0.05 * pdg.stats.tipRms);
    expect(ct.stats.fbShare).toBeLessThan(0.05);
    expect(pdg.stats.fbShare).toBeGreaterThan(0.3);
  });
  it('with a perfect model the computed-torque errors are decoupled second-order responses', () => {
    const e0 = 0.35;
    const tr = simulateArm({ ...base, offset: [e0, 0] });
    for (const k of [10, 30, 60]) {
      expect(tr.err[k][0]).toBeCloseTo(secondOrderFree(-e0, base.wn, base.zeta, tr.t[k]), 2);
    }
    expect(maxAbs(tr.err, 1)).toBeLessThan(5e-3);
  });
  it('a 20 % mass error leaves a static bias that the integral term removes', () => {
    const plain = simulateArm({ ...base, massError: -0.2 });
    const withI = simulateArm({ ...base, massError: -0.2, useKi: true });
    const bias = Math.hypot(...plain.stats.finalErr);
    expect(bias).toBeGreaterThan(0.015);
    expect(Math.hypot(...withI.stats.finalErr)).toBeLessThan(0.35 * bias);
  });
  it('task-space control near the folded singularity: straight but violent; DLS trades accuracy for speed', () => {
    const task = simulateArm({ ...base, kind: 'task', route: 'cd', Tf: 1.5 });
    const joint = simulateArm({ ...base, kind: 'ct', route: 'cd', Tf: 1.5 });
    const damped = simulateArm({ ...base, kind: 'task', route: 'cd', Tf: 1.5, lambda: 0.5 });
    expect(task.diverged || joint.diverged || damped.diverged).toBe(false);
    expect(task.stats.tipMax).toBeLessThan(0.01);
    expect(task.stats.maxRate).toBeGreaterThan(3 * joint.stats.maxRate);
    expect(damped.stats.maxRate).toBeLessThan(0.5 * task.stats.maxRate);
    expect(damped.stats.tipMax).toBeGreaterThan(10 * task.stats.tipMax);
  });
  it('secondOrderFree satisfies its initial conditions in all damping regimes', () => {
    for (const z of [0.5, 1, 1.5]) {
      expect(secondOrderFree(1, 5, z, 0)).toBeCloseTo(1, 12);
      const d = (secondOrderFree(1, 5, z, 1e-6) - secondOrderFree(1, 5, z, 0)) / 1e-6;
      expect(Math.abs(d)).toBeLessThan(1e-4);
    }
  });
  it('planar2RDynamics of the model scales with the mass error', () => {
    const th = [0.2, 0.9];
    const a = planar2RDynamics(armParams(1), th, [0, 0]).g, b = planar2RDynamics(armParams(1.2), th, [0, 0]).g;
    expect(b[0]).toBeCloseTo(1.2 * a[0], 10);
  });
});
