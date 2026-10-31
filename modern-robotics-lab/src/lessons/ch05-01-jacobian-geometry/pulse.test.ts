import { describe, expect, it } from 'vitest';
import { wrapToPi } from '@/mr';
import { PULSE_A, PULSE_T, planPulse, pulseState } from './pulse';

const DEG = Math.PI / 180;

/** Sample a pulse densely: angles, rates and finite-difference rates. */
function sample(theta0: number, lo: number, hi: number) {
  const plan = planPulse(theta0, lo, hi);
  const N = 4000;
  const out: { t: number; theta: number; rate: number }[] = [];
  for (let k = 0; k <= N; k++) {
    const t = (plan.period * k) / N;
    out.push({ t, ...pulseState(plan, theta0, t) });
  }
  return { plan, out };
}

describe('pulse schedule', () => {
  it('free joint: full swing to both sides, starts and ends at rate +1 and back at the start angle', () => {
    const th0 = 55 * DEG;
    const { plan, out } = sample(th0, -Math.PI / 2 - 1, Math.PI / 2 + 1);
    expect(plan).toEqual({ dir: 1, period: PULSE_T, wrap: false });
    expect(out[0].rate).toBeCloseTo(1, 12);
    expect(out[out.length - 1].rate).toBeCloseTo(1, 12);
    expect(out[out.length - 1].theta).toBeCloseTo(th0, 12);
    const thetas = out.map((s) => s.theta);
    expect(Math.max(...thetas) - th0).toBeCloseTo(PULSE_A, 6);
    expect(th0 - Math.min(...thetas)).toBeCloseTo(PULSE_A, 6);
    // the reported rate is the true derivative of the angle (A·Ω = 1)
    for (let k = 1; k < out.length - 1; k += 97) {
      const fd = (out[k + 1].theta - out[k - 1].theta) / (out[k + 1].t - out[k - 1].t);
      expect(out[k].rate).toBeCloseTo(fd, 5);
    }
  });

  it('next to the upper limit: one-sided swing downwards that never exceeds the limit', () => {
    const hi = Math.PI / 2;
    const { plan, out } = sample(hi, -Math.PI / 2, hi);
    expect(plan.dir).toBe(-1);
    expect(plan.period).toBe(PULSE_T / 2);
    for (const s of out) expect(s.theta).toBeLessThanOrEqual(hi + 1e-12);
    expect(out[out.length - 1].theta).toBeCloseTo(hi, 12);
    expect(Math.min(...out.map((s) => s.theta))).toBeCloseTo(hi - PULSE_A, 6);
  });

  it('next to the lower limit: one-sided swing upwards that never goes below the limit', () => {
    const lo = -Math.PI / 2;
    const { plan, out } = sample(lo + 5 * DEG, lo, Math.PI / 2);
    expect(plan.dir).toBe(1);
    expect(plan.period).toBe(PULSE_T / 2);
    for (const s of out) expect(s.theta).toBeGreaterThanOrEqual(lo - 1e-12);
  });

  it('full-turn joint near ±π: wrapped angles, full swing', () => {
    const th0 = 175 * DEG;
    const { plan, out } = sample(th0, -Math.PI, Math.PI);
    expect(plan).toEqual({ dir: 1, period: PULSE_T, wrap: true });
    for (const s of out) {
      expect(s.theta).toBeLessThanOrEqual(Math.PI + 1e-12);
      expect(s.theta).toBeGreaterThan(-Math.PI - 1e-12);
    }
    // the wrapped angle still follows the unwrapped swing
    expect(wrapToPi(out[1000].theta - (th0 + PULSE_A))).toBeCloseTo(0, 6);
  });
});
