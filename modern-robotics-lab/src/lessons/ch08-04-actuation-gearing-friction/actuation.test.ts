import { describe, expect, it } from 'vitest';
import { planar2RAccel } from '@/mr';
import {
  CouplingTwin,
  SingleJointSim,
  arm2Params,
  backdriveTorque,
  couplingIndex,
  effectiveMassMatrix,
  gearToJoint,
  gearedAccel,
  gearedKinetic,
  jointStep,
  motorTorque,
  reflectedInertia,
  rotorMatrix,
  swingTrajectory,
  torqueBreakdown,
  triangleWave,
  type Friction,
  type SingleJointParams,
} from './actuation';

const joint = (G: number, over: Partial<SingleJointParams> = {}): SingleJointParams => ({
  G, Jm: 3e-4, m: 1, L: 1, gravity: true, friction: { b: 0.05, tc: 0.1, ts: 0.15, mode: 'karnopp' }, tcMotor: 0.01, ...over,
});

describe('gearbox and reflected inertia', () => {
  it('trades speed for torque and conserves power', () => {
    const j = gearToJoint(100, { torque: 0.2, speed: 300 });
    expect(j.torque).toBeCloseTo(20, 12);
    expect(j.speed).toBeCloseTo(3, 12);
    expect(j.torque * j.speed).toBeCloseTo(0.2 * 300, 9);
  });

  it('reflected inertia scales with G²', () => {
    expect(reflectedInertia(100, 3e-4)).toBeCloseTo(3, 12);
    expect(reflectedInertia(200, 3e-4) / reflectedInertia(100, 3e-4)).toBeCloseTo(4, 12);
  });

  it('rotor matrix reproduces the rotor kinetic energy (motor 2 riding on link 1)', () => {
    const [G1, J1, G2, J2] = [80, 2e-4, 120, 5e-4];
    const D = rotorMatrix(G1, J1, G2, J2);
    for (const [a, b] of [[1.3, -0.4], [-2, 3], [0.1, 0.7]]) {
      const quad = 0.5 * (D[0][0] * a * a + 2 * D[0][1] * a * b + D[1][1] * b * b);
      const direct = 0.5 * J1 * (G1 * a) ** 2 + 0.5 * J2 * (a + G2 * b) ** 2;
      expect(quad).toBeCloseTo(direct, 9);
    }
  });
});

describe('geared 2R arm', () => {
  it('large G makes M_eff diagonal-dominant: the coupling index falls roughly like 1/G', () => {
    const P = arm2Params(false);
    for (const t2 of [0, 0.8, 2]) {
      const k = [1, 10, 100, 1000].map((G) => couplingIndex(effectiveMassMatrix(P, [0.3, t2], G, 3e-4)));
      for (let i = 1; i < k.length; i++) expect(k[i]).toBeLessThan(k[i - 1]);
      expect(k[3]).toBeLessThan(0.01);
    }
  });

  it('G = 1, Jm = 0 reduces to the bare closed-form arm', () => {
    const P = arm2Params(true);
    const th = [0.4, -1.1];
    const dth = [0.7, 2];
    const tau = [3, -1];
    const a = gearedAccel(P, th, dth, tau, 1, 0);
    const b = planar2RAccel(P, th, dth, tau);
    expect(a[0]).toBeCloseTo(b[0], 12);
    expect(a[1]).toBeCloseTo(b[1], 12);
  });

  it('conserves link + rotor kinetic energy when unforced (constant ΔM adds no Coriolis terms)', () => {
    const P = arm2Params(false);
    const [G, Jm] = [60, 3e-4];
    let x = [0.2, 0.9, 1.5, -2];
    const E0 = gearedKinetic(P, [x[0], x[1]], [x[2], x[3]], G, Jm);
    const h = 1e-3;
    for (let k = 0; k < 3000; k++) {
      const f = (xx: number[]) => { const d = gearedAccel(P, [xx[0], xx[1]], [xx[2], xx[3]], [0, 0], G, Jm); return [xx[2], xx[3], d[0], d[1]]; };
      const k1 = f(x);
      const k2 = f(x.map((v, i) => v + (h / 2) * k1[i]));
      const k3 = f(x.map((v, i) => v + (h / 2) * k2[i]));
      const k4 = f(x.map((v, i) => v + h * k3[i]));
      x = x.map((v, i) => v + (h / 6) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]));
    }
    expect(Math.abs(gearedKinetic(P, [x[0], x[1]], [x[2], x[3]], G, Jm) - E0)).toBeLessThan(1e-8);
  });

  it('torque breakdown (Newton–Euler + rotor + friction) is consistent with the geared forward dynamics', () => {
    const P = arm2Params(true);
    const drive = { G: 80, Jm: 3e-4, gravity: true, friction: { b: 0.2, tc: 0.3, ts: 0.4, mode: 'karnopp' as const }, tcMotor: 0.01 };
    const th = [-1.2, 0.8];
    const dth = [0.9, -1.4];
    const dd = [2.5, -3];
    const br = torqueBreakdown(th, dth, dd, drive);
    const tauNoFriction = br.total.map((v, i) => v - br.friction[i]);
    const back = gearedAccel(P, th, dth, tauNoFriction, drive.G, drive.Jm);
    expect(back[0]).toBeCloseTo(dd[0], 9);
    expect(back[1]).toBeCloseTo(dd[1], 9);
    for (let i = 0; i < 2; i++) {
      const sum = br.inertial[i] + br.velocity[i] + br.gravity[i] + br.rotor[i] + br.friction[i];
      expect(sum).toBeCloseTo(br.total[i], 9);
      expect(br.motor[i]).toBeCloseTo(br.total[i] / drive.G, 12);
    }
  });

  it('a geared, unpowered joint 2 is dragged along much less by the same joint-1 move', () => {
    const P = arm2Params(false);
    const bare = new CouplingTwin(P, 1, 0, [-0.9, 0.3], 1.2, 1);
    const geared = new CouplingTwin(P, 100, 3e-4, [-0.9, 0.3], 1.2, 1);
    for (let k = 0; k < 800; k++) { bare.step(1 / 400); geared.step(1 / 400); }
    expect(geared.theta()[0]).toBeCloseTo(bare.theta()[0], 12); // same joint-1 motion
    expect(geared.maxDev).toBeLessThan(bare.maxDev / 3);
  });

  it('swing trajectory is smooth at the turning points', () => {
    const A = [-2, 1];
    const B = [-1, -1];
    const at = swingTrajectory(1.2, A, B, 1.2);
    expect(at.theta[0]).toBeCloseTo(B[0], 9);
    expect(Math.abs(at.dtheta[0])).toBeLessThan(1e-9);
    expect(swingTrajectory(2.4, A, B, 1.2).theta[1]).toBeCloseTo(A[1], 9);
  });
});

describe('friction models', () => {
  const f = (mode: Friction['mode']): Friction => ({ b: 0, tc: 1, ts: 1.5, mode });

  it('Karnopp: static friction holds the joint exactly still below τ_s and lets go above it', () => {
    let s = { theta: 0, omega: 0 };
    for (let k = 0; k < 1000; k++) {
      const r = jointStep(s, 1, 1.4, f('karnopp'), 1e-3);
      expect(r.stuck).toBe(true);
      expect(r.tauF).toBeCloseTo(1.4, 12);
      s = r;
    }
    expect(s.theta).toBe(0);
    const r = jointStep(s, 1, 1.6, f('karnopp'), 1e-3);
    expect(r.stuck).toBe(false);
    expect(r.omega).toBeGreaterThan(0);
  });

  it('Karnopp stops a sliding joint without chatter; naive sgn chatters; tanh creeps', () => {
    const run = (mode: Friction['mode'], tau: number) => {
      let s = { theta: 0, omega: 0.5 };
      let flips = 0;
      for (let k = 0; k < 3000; k++) {
        const r = jointStep(s, 1, tau, f(mode), 1e-3);
        if (Math.sign(r.omega) * Math.sign(s.omega) < 0) flips++;
        s = r;
      }
      return { ...s, flips };
    };
    const k = run('karnopp', 0.5);
    expect(k.omega).toBe(0);
    expect(k.flips).toBe(0);
    expect(run('sgn', 0.5).flips).toBeGreaterThan(100);
    const t = run('tanh', 0.5);
    expect(t.flips).toBe(0);
    expect(t.omega).toBeGreaterThan(0); // never truly sticks: slow creep
  });

  it('free swing: viscous decay is geometric, Coulomb decay is linear', () => {
    const peaks = (fr: Friction) => {
      const sim = new SingleJointSim(joint(1, { Jm: 0, tcMotor: 0, friction: fr }), { kind: 'motor', tauCmd: 0 }, 0.3);
      const out: number[] = [];
      let prev = sim.omega;
      for (let k = 0; k < 20000 && out.length < 4; k++) {
        sim.step(1e-3);
        if (prev > 0 && sim.omega <= 0) out.push(sim.theta);
        prev = sim.omega;
      }
      return out;
    };
    const v = peaks({ b: 0.3, tc: 0, ts: 0, mode: 'karnopp' });
    expect(v[2] / v[1]).toBeCloseTo(v[1] / v[0], 2);
    const c = peaks({ b: 0, tc: 0.15, ts: 0.15, mode: 'karnopp' });
    // a full period loses about 4 τ_c / (m g L) of amplitude (small-angle estimate)
    const lossPerPeriod = (4 * 0.15) / 9.81;
    expect(c[0] - c[1]).toBeCloseTo(lossPerPeriod, 2);
    expect(c[1] - c[2]).toBeCloseTo(lossPerPeriod, 2);
  });

  it('slow drive through a spring produces stick–slip with Karnopp friction', () => {
    const p = joint(50, { gravity: false, friction: { b: 0.05, tc: 1, ts: 1.6, mode: 'karnopp' } });
    const sim = new SingleJointSim(p, { kind: 'spring', k: 20, omegaOut: 0.15, amp: 0.8 }, 0);
    let sticks = 0;
    let was = false;
    let maxW = 0;
    for (let k = 0; k < 10000; k++) {
      sim.step(1e-3);
      if (sim.stuck && !was) sticks++;
      was = sim.stuck;
      maxW = Math.max(maxW, sim.omega);
    }
    expect(sticks).toBeGreaterThan(4);
    expect(maxW).toBeGreaterThan(0.15); // slips overshoot the drive speed
  });
});

describe('single geared joint', () => {
  it('high G makes the joint hard to backdrive', () => {
    const reach = (G: number) => {
      const sim = new SingleJointSim(joint(G), { kind: 'push', tauExt: 3, t0: 0, dur: 0.5 }, 0);
      for (let k = 0; k < 3000; k++) sim.step(1e-3);
      return sim.maxAbsTheta;
    };
    expect(reach(1)).toBeGreaterThan(3 * reach(100));
    expect(reach(100)).toBeGreaterThan(reach(200));
    expect(backdriveTorque(joint(200), 2) / backdriveTorque(joint(1), 2)).toBeGreaterThan(8);
  });

  it('the motor torque falls along its torque–speed line', () => {
    expect(motorTorque(0.2, 0)).toBeCloseTo(0.2, 12);
    expect(motorTorque(0.4, 300)).toBeCloseTo(0.25, 12);
    expect(motorTorque(0.4, 700)).toBe(0);
    expect(motorTorque(-0.2, 100)).toBeCloseTo(-0.2, 12);
  });

  it('triangle wave stays within ±amp with slope ±omegaOut', () => {
    for (let t = 0; t < 30; t += 0.37) {
      const w = triangleWave(t, 0.15, 0.8);
      expect(Math.abs(w.angle)).toBeLessThanOrEqual(0.8 + 1e-12);
      expect(Math.abs(w.rate)).toBeCloseTo(0.15, 12);
    }
  });
});
