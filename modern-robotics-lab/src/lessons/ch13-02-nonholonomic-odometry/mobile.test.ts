import { describe, expect, it } from 'vitest';
import { matVec, wrapToPi } from '@/mr';
import {
  Gdiff, Gunicycle, WheelOdometry, bracketCar, bracketPrediction, bracketSequence, bracketUnicycle, clampInput,
  expTwist2, gaussian, headingDriftPrediction, inputSetPolys, mulberry32, parallelParkPlan, planDuration, poseAlong,
  poseError, samplePlan, stepEuler, stepPose, steeringAngle, vwToWheels, wheelsToVW,
  type DiffParams, type Limits, type Pose,
} from './mobile';

const close = (a: number, b: number, tol = 1e-9) => expect(Math.abs(a - b)).toBeLessThan(tol);
const P: DiffParams = { r: 0.06, d: 0.18 };
const LIM: Limits = { vmax: 0.6, wmax: 1.5, umax: 10, rmin: 1.0 };

describe('q̇ = G(q) u', () => {
  it('differential drive and unicycle give the same q̇ for matching inputs', () => {
    const phi = 0.7;
    const [uL, uR] = [3, 7];
    const [v, w] = wheelsToVW(uL, uR, P);
    const a = matVec(Gdiff(phi, P), [uL, uR]);
    const b = matVec(Gunicycle(phi), [v, w]);
    a.forEach((x, i) => close(x, b[i], 1e-12));
    const back = vwToWheels(v, w, P);
    close(back[0], uL, 1e-12);
    close(back[1], uR, 1e-12);
  });

  it('never produces sideways velocity (the rolling constraint holds for any input)', () => {
    for (const phi of [0, 0.4, 2, -2.5]) {
      const qd = matVec(Gdiff(phi, P), [2.3, -4.1]);
      close(-Math.sin(phi) * qd[1] + Math.cos(phi) * qd[2], 0, 1e-12);
    }
  });

  it('input sets: diff-drive diamond, car bow tie', () => {
    const [diamond] = inputSetPolys('diff', LIM, P);
    close(diamond[0][0], P.r * LIM.umax);
    close(diamond[1][1], (P.r * LIM.umax) / P.d);
    const car = inputSetPolys('car', LIM, P);
    expect(car).toHaveLength(2);
    close(car[0][2][1], LIM.vmax / LIM.rmin);
  });

  it('clamping: diff keeps the turning radius, car cannot turn without moving', () => {
    const c = clampInput('diff', 1.0, 2.0, LIM, P);
    expect(c.clamped).toBe(true);
    close(c.v / c.w, 0.5, 1e-12);
    const [uL, uR] = vwToWheels(c.v, c.w, P);
    close(Math.max(Math.abs(uL), Math.abs(uR)), LIM.umax, 1e-9);
    const car0 = clampInput('car', 0, 1, LIM, P);
    close(car0.w, 0);
    const car = clampInput('car', 0.4, 1, LIM, P);
    close(car.w, 0.4 / LIM.rmin);
    close(steeringAngle(0.4, 0.4, 0.45), Math.atan(0.45));
  });
});

describe('Lie brackets', () => {
  it('unicycle four-move sequence ≈ a·b·[g₁, g₂] (to the right), exact x = a(1 − cos b), y = −a sin b', () => {
    const q0: Pose = { phi: 0, x: 0, y: 0 };
    const a = 0.05, b = 0.05;
    const { final } = samplePlan(q0, bracketSequence('unicycle', a, b, 1));
    close(final.phi, 0, 1e-12);
    close(final.x, a * (1 - Math.cos(b)), 1e-12);
    close(final.y, -a * Math.sin(b), 1e-12);
    const pred = bracketPrediction('unicycle', a, b, 1, q0);
    expect(Math.abs(final.y - pred[1]) / Math.abs(pred[1])).toBeLessThan(0.01);
  });

  it('the bracket direction rotates with the heading', () => {
    const phi = 1.1;
    const br = bracketUnicycle(phi);
    // perpendicular to the heading, pointing right
    close(br[1] * Math.cos(phi) + br[2] * Math.sin(phi), 0, 1e-12);
    close(-br[1] * Math.sin(phi) + br[2] * Math.cos(phi), -1, 1e-12);
    const q0: Pose = { phi, x: 1, y: -2 };
    const { final } = samplePlan(q0, bracketSequence('unicycle', 0.03, 0.03, 1));
    const pred = bracketPrediction('unicycle', 0.03, 0.03, 1, q0);
    close(final.x - q0.x, pred[0], 2e-5);
    close(final.y - q0.y, pred[1], 2e-5);
  });

  it('car four-arc sequence ≈ 2ε²/r_min to the left', () => {
    const q0: Pose = { phi: 0, x: 0, y: 0 };
    const rmin = 0.8, eps = 0.04;
    const { final } = samplePlan(q0, bracketSequence('car', eps, 0, rmin));
    const pred = bracketPrediction('car', eps, 0, rmin, q0);
    close(pred[1], (2 * eps * eps) / rmin, 1e-12);
    expect(final.y).toBeGreaterThan(0);
    expect(Math.abs(final.y - pred[1]) / pred[1]).toBeLessThan(0.05);
    close(wrapToPi(final.phi), 0, 1e-9);
    const br = bracketCar(0.3, rmin);
    close(Math.hypot(br[1], br[2]), 2 / rmin, 1e-12);
  });

  it('poseAlong follows the plan exactly and stops at its end', () => {
    const q0: Pose = { phi: 0.2, x: 0.5, y: 0.1 };
    const segs = bracketSequence('car', 0.5, 0, 1.2);
    const T = planDuration(segs);
    const end = poseAlong(q0, segs, T + 5);
    expect(end.done).toBe(true);
    const { final } = samplePlan(q0, segs);
    close(end.pose.x, final.x, 1e-12);
    close(end.pose.y, final.y, 1e-12);
    expect(poseAlong(q0, segs, 0.5 * segs[0].T).seg).toBe(0);
  });
});

describe('parallel parking plan', () => {
  it('shifts the car sideways by exactly the requested amount and keeps x and the heading', () => {
    for (const [lat, rmin] of [[1, 1], [-0.6, 1.4], [1.5, 0.7]]) {
      const q0: Pose = { phi: 0.3, x: -1, y: 0.2 };
      const { final } = samplePlan(q0, parallelParkPlan(lat, rmin));
      const target: Pose = { phi: q0.phi, x: q0.x - lat * Math.sin(q0.phi), y: q0.y + lat * Math.cos(q0.phi) };
      const e = poseError(final, target);
      expect(e.dist).toBeLessThan(1e-9);
      expect(Math.abs(e.dphi)).toBeLessThan(1e-9);
    }
  });
});

describe('odometry', () => {
  const drive = (odo: WheelOdometry, truth: Pose, segs: { v: number; w: number; T: number }[], h = 1 / 120) => {
    let q = { ...truth };
    for (const g of segs) {
      const n = Math.round(g.T / h);
      for (let k = 0; k < n; k++) { q = stepPose(q, [g.w, g.v, 0], h); odo.step(g.v, g.w, h); }
    }
    return q;
  };

  it('perfect wheels + exponential update reproduce the truth exactly (piecewise-constant inputs)', () => {
    const q0: Pose = { phi: 0, x: 0, y: 0 };
    const odo = new WheelOdometry(q0, { p: P, rate: 10, rule: 'exp', slip: 0, delta: 0, seed: 1 });
    const truth = drive(odo, q0, [{ v: 0.4, w: 0.8, T: 3 }, { v: 0.3, w: -0.5, T: 2 }, { v: -0.2, w: 0, T: 1 }]);
    const e = poseError(odo.est, truth);
    expect(e.dist).toBeLessThan(1e-9);
    expect(Math.abs(e.dphi)).toBeLessThan(1e-9);
  });

  it('the Euler shortcut drifts while turning, more at low encoder rates', () => {
    // Each Euler update moves along the heading at the START of the sample, i.e. it rotates that
    // sample's displacement by ωΔt/2. On a half circle (radius 0.4 m) the error is ≈ 2R·ωΔt/2.
    const q0: Pose = { phi: 0, x: 0, y: 0 };
    const err = (rate: number) => {
      const odo = new WheelOdometry(q0, { p: P, rate, rule: 'euler', slip: 0, delta: 0, seed: 1 });
      const truth = drive(odo, q0, [{ v: 0.4, w: 1.0, T: 3.2 }]);
      return poseError(odo.est, truth).dist;
    };
    expect(err(5)).toBeGreaterThan(0.06);
    expect(err(5)).toBeGreaterThan(5 * err(50));
    // heading itself is still exact: Δφ adds up correctly, only the position is off
    const odo = new WheelOdometry(q0, { p: P, rate: 5, rule: 'euler', slip: 0, delta: 0, seed: 1 });
    const truth = drive(odo, q0, [{ v: 0.4, w: 1.0, T: 3.2 }]);
    close(wrapToPi(odo.est.phi - truth.phi), 0, 1e-9);
  });

  it('a larger right wheel makes the estimate curve right: heading error ≈ −δ s / (2d)', () => {
    const q0: Pose = { phi: 0, x: 0, y: 0 };
    const delta = 0.01;
    const odo = new WheelOdometry(q0, { p: P, rate: 20, rule: 'exp', slip: 0, delta, seed: 1 });
    const truth = drive(odo, q0, [{ v: 0.5, w: 0, T: 10 }]);
    const s = 5;
    const dphi = wrapToPi(odo.est.phi - truth.phi);
    const pred = headingDriftPrediction(delta, s, P.d);
    expect(dphi).toBeLessThan(0);
    expect(Math.abs(dphi - pred) / Math.abs(pred)).toBeLessThan(0.01);
    // lateral error grows like κ s² / 2 (estimate ends to the right of the true straight path)
    expect(odo.est.y).toBeLessThan(0);
    close(odo.est.y, (pred * s) / 2, 0.02 * Math.abs((pred * s) / 2));
  });

  it('slip noise is reproducible with the same seed and grows the error', () => {
    const q0: Pose = { phi: 0, x: 0, y: 0 };
    const run = (seed: number) => {
      const odo = new WheelOdometry(q0, { p: P, rate: 20, rule: 'exp', slip: 0.05, delta: 0, seed });
      const truth = drive(odo, q0, [{ v: 0.5, w: 0.3, T: 20 }]);
      return poseError(odo.est, truth).dist;
    };
    expect(run(7)).toBe(run(7));
    expect(run(7)).toBeGreaterThan(0.01);
    const rand = mulberry32(3);
    const xs = Array.from({ length: 4000 }, () => gaussian(rand));
    const mean = xs.reduce((a, b) => a + b, 0) / xs.length;
    const varc = xs.reduce((a, b) => a + (b - mean) ** 2, 0) / xs.length;
    expect(Math.abs(mean)).toBeLessThan(0.06);
    expect(Math.abs(varc - 1)).toBeLessThan(0.08);
  });

  it('splitting the integration step at timeToSample() gives exactly `rate` samples per second', () => {
    const q0: Pose = { phi: 0, x: 0, y: 0 };
    for (const rate of [7, 25, 45, 50]) {
      const odo = new WheelOdometry(q0, { p: P, rate, rule: 'exp', slip: 0, delta: 0, seed: 1 });
      // same sub-stepping as the lesson: never step past the next sample instant
      for (let k = 0; k < 1200; k++) {
        let rest = 1 / 120;
        while (rest > 1e-12) {
          const dt = Math.min(rest, odo.timeToSample());
          odo.step(0.3, 0.4, dt);
          rest -= dt;
        }
      }
      expect(odo.updates).toBe(10 * rate);
    }
    // without splitting, a sample closes at the first whole step past the period (quantised rate)
    const coarse = new WheelOdometry(q0, { p: P, rate: 50, rule: 'exp', slip: 0, delta: 0, seed: 1 });
    for (let k = 0; k < 1200; k++) coarse.step(0.3, 0.4, 1 / 120);
    expect(coarse.updates).toBe(400);
  });

  it('exp and Euler agree for straight motion, and expTwist2 handles ω = 0', () => {
    const q: Pose = { phi: 0.4, x: 1, y: 2 };
    const a = stepPose(q, [0, 0.5, 0], 2);
    const b = stepEuler(q, [0, 0.5, 0], 2);
    close(a.x, b.x, 1e-12);
    close(a.y, b.y, 1e-12);
    const d = expTwist2([0, 0.3, 0], 1);
    close(d.x, 0.3, 1e-12);
  });
});
