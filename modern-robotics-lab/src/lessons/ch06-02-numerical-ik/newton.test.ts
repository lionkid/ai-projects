import { describe, expect, it } from 'vitest';
import { FKinBody, IKinBody, JacobianBody, dot, matVec, norm, pinv, wrapToPi } from '@/mr';
import {
  MODEL, REACH, TRACK_PATHS, advanceTracking, circleTarget, converged, evaluate, isStalled, iterate, nullSpaceDirection,
  poseReachable, reducedJacobian, residual, runNewton, screwArc, solveStep, targetTransform, trackingRates, trackingStart,
  type StepOptions, type Task,
} from './newton';

const D = Math.PI / 180;
const th = (deg: number[]) => deg.map((x) => x * D);
const PINV: StepOptions = { method: 'pinv', lambda: 0, maxStep: Infinity };
const dls = (lambda: number): StepOptions => ({ method: 'dls', lambda, maxStep: Infinity });
const TASK: Task = { x: 1.0, y: 1.2, phi: 60 * D, positionOnly: false };
const angleDiff = (a: number[], b: number[]) => Math.max(...a.map((x, i) => Math.abs(wrapToPi(x - b[i]))));

describe('residual and Jacobian reduction', () => {
  it('is zero at a configuration that realises the target, and planar rows of the 6-D twist vanish', () => {
    const q = th([30, 40, -20]);
    const T = FKinBody(MODEL.M, MODEL.Blist, q);
    const task: Task = { x: T[0][3], y: T[1][3], phi: Math.atan2(T[1][0], T[0][0]), positionOnly: false };
    const r0 = residual(q, task);
    expect(r0.omegaNorm + r0.vNorm).toBeLessThan(1e-12);
    const r = residual(th([10, 70, 5]), task);
    for (const i of [0, 1, 5]) expect(Math.abs(r.V6[i])).toBeLessThan(1e-12);
    expect(r.V).toEqual([r.V6[2], r.V6[3], r.V6[4]]);
  });

  it('the reduced pseudo-inverse step equals the full 6-D one', () => {
    const q = th([15, 50, -30]);
    const r = residual(q, TASK);
    const full = matVec(pinv(JacobianBody(MODEL.Blist, q)), r.V6);
    const red = solveStep(reducedJacobian(q, false), r.V, PINV).dtheta;
    expect(angleDiff(full, red)).toBeLessThan(1e-10);
  });

  it('the screw arc of the error twist ends exactly on the target', () => {
    const q = th([20, 60, 0]);
    const r = residual(q, TASK);
    const arc = screwArc(r.Tsb, r.V6, 10);
    const Td = targetTransform(TASK);
    const end = arc[arc.length - 1];
    for (let i = 0; i < 3; i++) for (let j = 0; j < 4; j++) expect(end[i][j]).toBeCloseTo(Td[i][j], 10);
  });
});

describe('Newton–Raphson runs', () => {
  it('reproduces IKinBody iterate by iterate', () => {
    const q0 = th([20, 60, 0]);
    const ref = IKinBody(MODEL.Blist, MODEL.M, targetTransform(TASK), q0, 1e-6, 1e-6, { maxIterations: 20 });
    const run = runNewton(q0, TASK, PINV);
    expect(run.status).toBe('converged');
    expect(ref.success).toBe(true);
    expect(run.records.length).toBe(ref.history.length);
    ref.history.forEach((h, k) => expect(angleDiff(h.theta, run.records[k].theta)).toBeLessThan(1e-9));
  });

  it('converges quadratically from a good initial guess', () => {
    const run = runNewton(th([0, 90, 0]), TASK, PINV);
    expect(run.status).toBe('converged');
    expect(run.records.length - 1).toBeLessThanOrEqual(5);
    const e = run.records.map((r) => r.omegaNorm + r.vNorm);
    // each late error is bounded by C·(previous error)², with a modest C
    for (let k = 2; k < e.length; k++) expect(e[k]).toBeLessThan(5 * e[k - 1] * e[k - 1]);
  });

  it('a bad guess lands on the other elbow branch', () => {
    const good = runNewton(th([0, 90, 0]), TASK, PINV);
    const bad = runNewton(th([-150, 60, 90]), TASK, PINV);
    expect(bad.status).toBe('converged');
    const qa = good.records[good.records.length - 1].theta;
    const qb = bad.records[bad.records.length - 1].theta;
    expect(Math.sign(qa[1])).not.toBe(Math.sign(qb[1]));
  });

  it('near a singularity the pseudo-inverse step explodes but DLS stays bounded', () => {
    const q0 = th([40, 3, -20]);
    const rec = evaluate(q0, TASK);
    expect(rec.sigmaMin).toBeLessThan(0.05);
    iterate(rec, TASK, PINV);
    expect(rec.dthetaNorm).toBeGreaterThan(10);
    const rec2 = evaluate(q0, TASK);
    iterate(rec2, TASK, dls(0.1));
    expect(rec2.dthetaNorm).toBeLessThan(2);
    expect(runNewton(q0, TASK, dls(0.1)).status).toBe('converged');
    // capping the step also tames the pseudo-inverse
    const capped = runNewton(q0, TASK, { ...PINV, maxStep: 1 });
    expect(capped.status).toBe('converged');
    expect(Math.max(...capped.records.map((r) => r.dthetaNorm))).toBeLessThanOrEqual(1 + 1e-12);
  });

  it('larger damping converges more slowly', () => {
    const its = [0.05, 0.1, 0.3].map((lam) => runNewton(th([0, 90, 0]), TASK, dls(lam)).records.length);
    expect(its[0]).toBeLessThanOrEqual(its[1]);
    expect(its[1]).toBeLessThan(its[2]);
  });

  it('detects stagnation for an unreachable target', () => {
    const far: Task = { x: 2.6, y: 0.6, phi: 0, positionOnly: true };
    expect(poseReachable(far).reachable).toBe(false);
    const run = runNewton(th([30, 30, 30]), far, dls(0.3));
    expect(run.status).toBe('stalled');
    const last = run.records[run.records.length - 1];
    expect(last.vNorm).toBeCloseTo(Math.hypot(2.6, 0.6) - REACH, 3);
    expect(isStalled(runNewton(th([0, 90, 0]), TASK, PINV).records)).toBe(false);
  });

  it('uses the IKinBody default tolerances in converged()', () => {
    const r = residual(th([20, 60, 0]), TASK);
    expect(converged(r)).toBe(false);
  });
});

describe('redundancy (position-only task)', () => {
  const pos: Task = { x: 1.0, y: 1.2, phi: 0, positionOnly: true };
  it('the pseudo-inverse step has no null-space component; the weighted one does', () => {
    const q = th([10, 60, 60]);
    const J = reducedJacobian(q, true);
    const V = residual(q, pos).V;
    const n = nullSpaceDirection(J)!;
    expect(Math.abs(dot(matVec(J, n), [1, 1]))).toBeLessThan(1e-12);
    const a = solveStep(J, V, PINV).dtheta;
    const b = solveStep(J, V, { ...PINV, weights: [10, 1, 1] }).dtheta;
    expect(Math.abs(dot(a, n))).toBeLessThan(1e-12);
    expect(Math.abs(dot(b, n))).toBeGreaterThan(0.05);
    // both solve the linearised equation exactly, the weighted one moves joint 1 less
    for (const d of [a, b]) matVec(J, d).forEach((x, i) => expect(x).toBeCloseTo(V[i], 12));
    expect(Math.abs(b[0])).toBeLessThan(Math.abs(a[0]));
    expect(norm(b)).toBeGreaterThan(norm(a));
  });
});

describe('inverse velocity kinematics', () => {
  it('with a static target the rate is the Newton step scaled by the gain', () => {
    const q = th([20, 60, 0]);
    const tgt = { x: TASK.x, y: TASK.y, phi: TASK.phi, dx: 0, dy: 0, dphi: 0 };
    const r = trackingRates(q, tgt, false, 2, PINV);
    const rec = evaluate(q, TASK);
    iterate(rec, TASK, PINV);
    r.thetadot.forEach((x, i) => expect(x).toBeCloseTo(2 * rec.dtheta![i], 10));
  });

  it('on target, the feedforward reproduces the target body twist', () => {
    const q = th([25, 70, -40]);
    const T = FKinBody(MODEL.M, MODEL.Blist, q);
    const tgt = { x: T[0][3], y: T[1][3], phi: Math.atan2(T[1][0], T[0][0]), dx: 0.2, dy: -0.1, dphi: 0.3 };
    const r = trackingRates(q, tgt, false, 5, PINV);
    const Vb = matVec(reducedJacobian(q, false), r.thetadot);
    const c = Math.cos(tgt.phi), s = Math.sin(tgt.phi);
    expect(Vb[0]).toBeCloseTo(0.3, 10);
    expect(Vb[1]).toBeCloseTo(c * 0.2 + s * -0.1, 10);
    expect(Vb[2]).toBeCloseTo(-s * 0.2 + c * -0.1, 10);
  });

  it('reports wrist-centre reachability for pose tasks', () => {
    expect(poseReachable(TASK).reachable).toBe(true);
    expect(poseReachable({ x: 2.0, y: 0.6, phi: Math.PI, positionOnly: false }).reachable).toBe(false);
  });
});

describe('moving targets and the tracking demo', () => {
  it('circle paths report analytic derivatives that match finite differences', () => {
    for (const P of TRACK_PATHS) {
      for (const t of [0, 0.7, 2.3, 5.1]) {
        const h = 1e-6;
        const a = circleTarget(P, t - h), b = circleTarget(P, t + h), m = circleTarget(P, t);
        expect(m.dx).toBeCloseTo((b.x - a.x) / (2 * h), 6);
        expect(m.dy).toBeCloseTo((b.y - a.y) / (2 * h), 6);
        expect(m.dphi).toBeCloseTo(wrapToPi(b.phi - a.phi) / (2 * h), 6);
      }
    }
    const r = circleTarget(TRACK_PATHS[1], 1.234);
    expect(r.phi).toBeCloseTo(Math.atan2(r.y, r.x), 12);
  });

  const simulate = (P: (typeof TRACK_PATHS)[number], o: StepOptions, T: number) => {
    const path = (t: number) => circleTarget(P, t);
    const s = trackingStart(path, th([0, 90, 0]));
    const start = s.err;
    let maxRate = 0, maxErr = 0, unreachable = 0;
    while (s.t < T) {
      advanceTracking(s, path, 1 / 60, 5, o);
      maxRate = Math.max(maxRate, norm(s.rates));
      maxErr = Math.max(maxErr, s.err);
      if (!s.reachable) unreachable += 1 / 60;
    }
    return { start, maxRate, maxErr, unreachable, endErr: s.err };
  };
  const period = (P: (typeof TRACK_PATHS)[number]) => (2 * Math.PI) / P.omega;

  it('the inner circle is tracked closely with modest joint rates', () => {
    const P = TRACK_PATHS[0];
    const r = simulate(P, PINV, period(P));
    expect(r.start).toBeLessThan(1e-6);
    expect(r.unreachable).toBe(0);
    expect(r.maxRate).toBeLessThan(2);
    expect(r.maxErr).toBeLessThan(1e-2);
  });

  it('leaving the workspace: J⁺ demands huge rates, DLS stays bounded and waits at the boundary', () => {
    const P = TRACK_PATHS[1];
    const overshoot = Math.hypot(P.cx, P.cy) + P.rho - REACH;
    const pinvRun = simulate(P, PINV, period(P));
    const dlsRun = simulate(P, dls(0.1), period(P));
    expect(pinvRun.start).toBeLessThan(1e-6);
    // about 2 s of the period is out of reach
    expect(dlsRun.unreachable).toBeGreaterThan(1.5);
    expect(dlsRun.unreachable).toBeLessThan(2.5);
    expect(pinvRun.maxRate).toBeGreaterThan(50);
    expect(dlsRun.maxRate).toBeLessThan(5);
    // DLS's worst error is the overshoot itself: the arm stretches towards the target and waits
    expect(dlsRun.maxErr).toBeGreaterThan(0.9 * overshoot);
    expect(dlsRun.maxErr).toBeLessThan(1.2 * overshoot);
    // both recover once the target is reachable again
    expect(pinvRun.endErr).toBeLessThan(1e-2);
    expect(dlsRun.endErr).toBeLessThan(2e-2);
  });
});
