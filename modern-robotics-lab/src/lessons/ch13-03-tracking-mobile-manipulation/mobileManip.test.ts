import { describe, expect, it } from 'vitest';
import { pinv } from '@/mr';
import { advance, between, logPose, pose, type Twist2 } from './se2';
import {
  HOME, baseForEE, baseInputMap, dlsPinv, eePose, minManipulability, mobileJacobian, resolveRates, simulateMobile, type MMConfig,
} from './mobileManip';
import type { BaseKind } from './tracking';

const near = (a: number, b: number, tol = 1e-9) => expect(Math.abs(a - b)).toBeLessThan(tol);
const deg = Math.PI / 180;

const LINE: MMConfig = { base: 'diff', mode: 'minnorm', A: [0, 1.05], B: [3, 1.05], phiE: 90 * deg, Tf: 16, kp: 2, kn: 1.5 };

describe('mobile manipulator Jacobian J_e = [J_base  J_arm]', () => {
  it.each(['diff', 'omni'] as BaseKind[])('%s: every column equals the finite-difference end-effector twist', (base) => {
    const q = pose(0.3, 0.4, -0.2);
    const theta: [number, number] = [1.1, -0.7];
    const { J, nb } = mobileJacobian(base, theta);
    const F = baseInputMap(base);
    const X0 = eePose(q, theta);
    const d = 1e-6;
    for (let j = 0; j < J[0].length; j++) {
      let q1 = q, th1 = theta;
      if (j < nb) q1 = advance(q, F.map((r) => r[j] * d) as Twist2, 1);
      else th1 = theta.map((v, i) => (i === j - nb ? v + d : v)) as [number, number];
      const V = logPose(between(X0, eePose(q1, th1))).map((x) => x / d);
      for (let i = 0; i < 3; i++) near(J[i][j], V[i], 1e-5);
    }
  });

  it('the base columns need the Adjoint: the base yaw rate moves the end-effector sideways', () => {
    // With the arm stretched along x_b, ω = 1 rad/s swings the tip at |r| m/s in {e}.
    const { Jbase } = mobileJacobian('diff', [0, 0]);
    near(Jbase[0][1], 1); // ω column: angular part 1
    near(Jbase[2][1], 0.12 + 0.5 + 0.4, 1e-12); // v_y in {e} = distance from the base centre
    near(Jbase[1][0], 1); // v column: pure forward speed
  });

  it('baseForEE places the end-effector exactly at the requested pose', () => {
    const q = baseForEE([0.3, 1.05], 90 * deg, HOME);
    const X = eePose(q, HOME);
    near(X.x, 0.3, 1e-12); near(X.y, 1.05, 1e-12); near(Math.cos(X.phi - 90 * deg), 1, 1e-12);
  });

  it('the damped pseudo-inverse matches the SVD pseudo-inverse for a full-rank J_e', () => {
    const { J } = mobileJacobian('omni', [2.4, -0.9]);
    const A = dlsPinv(J), B = pinv(J);
    for (let i = 0; i < A.length; i++) for (let j = 0; j < 3; j++) near(A[i][j], B[i][j], 1e-5);
  });

  it('null-space motion leaves the end-effector twist unchanged', () => {
    const theta: [number, number] = [1.9, -0.3];
    const V: Twist2 = [0.1, 0.2, -0.1];
    for (const base of ['diff', 'omni'] as BaseKind[]) {
      const a = resolveRates({ base, mode: 'minnorm', kn: 2 }, theta, V);
      const b = resolveRates({ base, mode: 'center', kn: 2 }, theta, V);
      const twist = (r: number[]) => a.jac.J.map((row) => row.reduce((s, x, j) => s + x * r[j], 0));
      const ta = twist(a.rates), tb = twist(b.rates);
      // The tiny damping (λ = 1e-3) of J^# leaves O(λ²/σ²) ≈ 1e-5 residuals.
      for (let i = 0; i < 3; i++) { near(ta[i], V[i], 1e-4); near(tb[i], V[i], 1e-4); }
      expect(Math.abs(b.rates[b.jac.nb + 1] - a.rates[a.jac.nb + 1])).toBeGreaterThan(0.1); // the split did change
    }
  });
});

describe('redundancy resolution along a 3 m line', () => {
  for (const base of ['diff', 'omni'] as BaseKind[]) {
    const mn = simulateMobile({ ...LINE, base, mode: 'minnorm' });
    const ce = simulateMobile({ ...LINE, base, mode: 'center' });

    it(`${base}: both solutions keep the end-effector on the line`, () => {
      expect(Math.max(...mn.errNorm)).toBeLessThan(2e-3);
      expect(Math.max(...ce.errNorm)).toBeLessThan(2e-3);
      const end = ce.X[ce.X.length - 1];
      near(end.x, 3, 2e-3); near(end.y, 1.05, 2e-3);
    });

    it(`${base}: minimum norm lets the arm drift toward its singular posture, centring does not`, () => {
      const wMn = minManipulability(mn).w, wCe = minManipulability(ce).w;
      expect(wMn).toBeLessThan(base === 'diff' ? 0.1 : 0.25);
      expect(wCe).toBeGreaterThan(base === 'diff' ? 0.25 : 0.6);
      expect(ce.w[0]).toBeCloseTo(Math.abs(Math.sin(HOME[1])), 9);
    });

    it(`${base}: base and arm contributions add up to the line speed`, () => {
      for (let k = 0; k < mn.t.length; k += 25) near(mn.sBase[k] + mn.sArm[k], mn.sDes[k], 0.02);
    });
  }
});
