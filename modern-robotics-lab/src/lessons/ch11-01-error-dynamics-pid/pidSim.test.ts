import { describe, expect, it } from 'vitest';
import {
  DEFAULT_PLANT, charPoly, pdGainsFromPole, pdPrediction, pdSag2R, pidKiBound, polyRoots, simulateJoint, stepMetrics,
  tailAmplitude, PREVIEW_ARM, type TrialOptions,
} from './pidSim';
import { planar2RDynamics } from '@/mr';

const P = DEFAULT_PLANT;
const trial = (o: Partial<TrialOptions>) => simulateJoint({
  mode: 'torque', plant: P, gains: { kp: 20, ki: 0, kd: 2.4 }, ref: { kind: 'step', amp: 1, freq: 0.4 }, T: 6, ...o,
});
const sortRoots = (r: { re: number; im: number }[]) => r.slice().sort((a, b) => a.re - b.re || a.im - b.im);

describe('polyRoots', () => {
  it('finds three real roots of (s+1)(s+2)(s+3)', () => {
    const r = sortRoots(polyRoots([1, 6, 11, 6]));
    expect(r.map((z) => z.re)).toEqual([expect.closeTo(-3, 9), expect.closeTo(-2, 9), expect.closeTo(-1, 9)]);
    r.forEach((z) => expect(Math.abs(z.im)).toBeLessThan(1e-9));
  });
  it('finds a real root and a complex pair of (s+4)(s²+2s+5)', () => {
    const r = sortRoots(polyRoots([1, 6, 13, 20]));
    expect(r[0].re).toBeCloseTo(-4, 9);
    expect(r[1].re).toBeCloseTo(-1, 9);
    expect(Math.abs(r[1].im)).toBeCloseTo(2, 9);
    expect(r[2].im).toBeCloseTo(-r[1].im, 12);
  });
  it('handles degree 1, 2 and leading zeros', () => {
    expect(polyRoots([2, 4])[0].re).toBeCloseTo(-2, 12);
    const q = polyRoots([0, 1, 2, 5]);
    expect(q).toHaveLength(2);
    expect(q[0].re).toBeCloseTo(-1, 12);
    expect(Math.abs(q[0].im)).toBeCloseTo(2, 12);
  });
});

describe('first-order error dynamics (velocity input)', () => {
  it('P control decays exponentially with time constant 1/Kp', () => {
    const kp = 2;
    const tr = trial({ mode: 'velocity', gains: { kp, ki: 0, kd: 0 } });
    const i = tr.t.findIndex((t) => t >= 1 / kp - 1e-9);
    // ZOH at 2 ms: the discrete pole (1 − Kp h) is within 0.5 % of exp(−Kp h).
    expect(tr.err[i]).toBeCloseTo(Math.exp(-1), 2);
    const m = stepMetrics(tr.t, tr.err);
    expect(m.overshootPct).toBe(0);
    expect(m.ts!).toBeGreaterThan(0.9 * (4 / kp));
    expect(m.ts!).toBeLessThan(1.1 * (4 / kp));
  });
  it('a ramp leaves e_ss = c/Kp under P; PI or velocity feedforward removes it', () => {
    const ramp = { kind: 'ramp' as const, amp: 0.5, freq: 0 };
    const p = trial({ mode: 'velocity', gains: { kp: 2, ki: 0, kd: 0 }, ref: ramp });
    expect(p.err[p.err.length - 1]).toBeCloseTo(0.5 / 2, 3);
    const pi = trial({ mode: 'velocity', gains: { kp: 2, ki: 3, kd: 0 }, ref: ramp, T: 12 });
    expect(Math.abs(pi.err[pi.err.length - 1])).toBeLessThan(2e-3);
    const ff = trial({ mode: 'velocity', gains: { kp: 2, ki: 0, kd: 0 }, ref: ramp, velFeedforward: true });
    expect(Math.max(...ff.err.map(Math.abs))).toBeLessThan(1e-9);
  });
});

describe('second-order error dynamics (torque input, PD)', () => {
  it('measured overshoot and settling time agree with the ζ, ωn predictions', () => {
    const tr = trial({ gains: { kp: 20, ki: 0, kd: 1 } });
    const m = stepMetrics(tr.t, tr.err);
    const pr = pdPrediction(P, 20, 1)!;
    expect(pr.zeta).toBeCloseTo(1.1 / (2 * Math.sqrt(10)), 12);
    expect(pr.wn).toBeCloseTo(Math.sqrt(40), 12);
    expect(Math.abs(m.overshootPct - pr.overshootPct)).toBeLessThan(1.5);
    expect(Math.abs(m.ts! - pr.ts) / pr.ts).toBeLessThan(0.1);
  });
  it('a constant disturbance leaves e_ss = τ_dist / Kp', () => {
    for (const kp of [10, 40]) {
      const tr = trial({ gains: { kp, ki: 0, kd: 6 }, tauDist: 2 });
      expect(stepMetrics(tr.t, tr.err).ess).toBeCloseTo(2 / kp, 4);
    }
  });
  it('the I term removes the gravity-induced steady-state error', () => {
    const pd = trial({ gains: { kp: 20, ki: 0, kd: 6.2 }, gravity: true });
    const pid = trial({ gains: { kp: 20, ki: 20, kd: 6.2 }, gravity: true });
    expect(stepMetrics(pd.t, pd.err).ess).toBeGreaterThan(0.08);
    expect(Math.abs(stepMetrics(pid.t, pid.err).ess)).toBeLessThan(2e-3);
  });
  it('integrator windup inflates the overshoot; anti-windup tames it', () => {
    const o: Partial<TrialOptions> = { gains: { kp: 25, ki: 30, kd: 4 }, gravity: true, tauMax: 5, ref: { kind: 'step', amp: 1.2, freq: 0 } };
    const off = trial(o);
    const on = trial({ ...o, antiWindup: true });
    expect(off.saturated && on.saturated).toBe(true);
    const mOff = stepMetrics(off.t, off.err), mOn = stepMetrics(on.t, on.err);
    expect(mOff.overshootPct).toBeGreaterThan(3 * mOn.overshootPct);
    expect(Math.max(...off.iTerm.map(Math.abs))).toBeGreaterThan(4 * Math.max(...on.iTerm.map(Math.abs)));
    expect(Math.max(...on.u.map(Math.abs))).toBeLessThanOrEqual(5 + 1e-12);
  });
});

describe('linear analysis helpers', () => {
  it('pdGainsFromPole places the closed-loop roots where requested', () => {
    const g = pdGainsFromPole(P, -3, 4);
    const r = polyRoots(charPoly('torque', P, { kp: g.kp, ki: 0, kd: g.kd }));
    expect(r[0].re).toBeCloseTo(-3, 9);
    expect(Math.abs(r[0].im)).toBeCloseTo(4, 9);
  });
  it('the Routh–Hurwitz bound on Ki separates stable from unstable PID loops', () => {
    const kp = 20, kd = 1;
    const bound = pidKiBound(P, kp, kd);
    expect(bound).toBeCloseTo(44, 9);
    const maxRe = (ki: number) => Math.max(...polyRoots(charPoly('torque', P, { kp, ki, kd })).map((z) => z.re));
    expect(maxRe(0.9 * bound)).toBeLessThan(0);
    expect(maxRe(1.1 * bound)).toBeGreaterThan(0);
    const unstable = trial({ gains: { kp, ki: 1.3 * bound, kd }, T: 12 });
    expect(unstable.diverged || tailAmplitude(unstable.t, unstable.err, 2) > 1).toBe(true);
  });
  it('stepMetrics reads a pure exponential correctly', () => {
    const t = Array.from({ length: 1001 }, (_, i) => i * 0.01);
    const e = t.map((x) => Math.exp(-x));
    const m = stepMetrics(t, e);
    expect(m.overshootPct).toBe(0);
    expect(m.ts!).toBeCloseTo(Math.log(50), 1);
  });
});

describe('2R preview', () => {
  it('pdSag2R finds the equilibrium K_p(θd − θ) = g(θ)', () => {
    const kp: [number, number] = [60, 15];
    const thd: [number, number] = [0.3, 0.9];
    const r = pdSag2R(thd, kp);
    expect(r.residual).toBeLessThan(1e-9);
    const g = planar2RDynamics(PREVIEW_ARM, r.theta, [0, 0]).g;
    expect(kp[0] * (thd[0] - r.theta[0])).toBeCloseTo(g[0], 9);
    expect(r.theta[0]).toBeLessThan(thd[0]);
    expect(r.theta[1]).toBeLessThan(thd[1]);
  });
});
