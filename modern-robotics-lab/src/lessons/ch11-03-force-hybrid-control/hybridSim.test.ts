import { describe, expect, it } from 'vitest';
import { MassMatrix, matMul, matSub, eye, planar2R, transpose, maxAbs } from '@/mr';
import { HybridSim, projectionP, taskModel, elbowUpIK, SURF_C, H, type SimParams } from './hybridSim';

/** Run a sim for T seconds; report the force statistics over the last `win` seconds. */
function run(p: Partial<SimParams>, T: number, win = 1) {
  const sim = new HybridSim(p);
  const N = Math.round(T / H);
  let fsum = 0, n = 0, fmin = Infinity, fmax = 0, lost = 0, emax = 0;
  let was = false;
  for (let k = 0; k < N; k++) {
    sim.step();
    if (sim.t > T - win) {
      fsum += sim.f; n++;
      fmin = Math.min(fmin, sim.f); fmax = Math.max(fmax, sim.f);
      if (was && !sim.contact) lost++;
      if (sim.info) emax = Math.max(emax, Math.abs(sim.info.sd - sim.info.s));
    }
    was = sim.contact;
  }
  return { sim, mean: fsum / n, fmin, fmax, lost, emax };
}

describe('projection matrix P', () => {
  // projectionP regularises AΛ⁻¹Aᵀ by 1e-9 (singularity guard), hence the 1e-7 tolerances.
  const Linv = [[0.95, 0.15], [0.15, 0.54]];
  const n = [-Math.sin(0.2), Math.cos(0.2)];
  const A = [n];
  const P = projectionP(A, Linv);
  const IP = matSub(eye(2), P);

  it('is idempotent and complementary', () => {
    expect(maxAbs(matSub(matMul(P, P), P))).toBeLessThan(1e-7);
    expect(maxAbs(matMul(IP, P))).toBeLessThan(1e-7);
  });

  it('only passes wrenches whose acceleration Λ⁻¹F satisfies the constraint', () => {
    expect(maxAbs(matMul(matMul(A, Linv), P))).toBeLessThan(1e-7);
  });

  it('leaves pure normal forces to I − P', () => {
    const Fn = [[3 * n[0]], [3 * n[1]]];
    expect(maxAbs(matMul(P, Fn))).toBeLessThan(1e-7);
    expect(maxAbs(matSub(matMul(IP, Fn), Fn))).toBeLessThan(1e-7);
  });

  it('is oblique for coupled inertia and orthogonal for Λ⁻¹ = I', () => {
    const Port = projectionP(A, eye(2));
    expect(maxAbs(matSub(Port, transpose(Port)))).toBeLessThan(1e-9);
    expect(maxAbs(matSub(P, transpose(P)))).toBeGreaterThan(0.05);
  });
});

describe('task model', () => {
  it('uses the same mass matrix as the Newton–Euler planar2R model', () => {
    const th = elbowUpIK([SURF_C[0], SURF_C[1]]);
    const tm = taskModel(th, [0, 0]);
    const Mne = MassMatrix([th[0], th[1]], planar2R.Mlist!, planar2R.Glist!, planar2R.Slist);
    expect(maxAbs(matSub(tm.M, Mne))).toBeLessThan(1e-9);
  });

  it('computes J̇θ̇ as the centripetal tip acceleration', () => {
    const th = [0.4, -1.6], dth = [1.2, -0.7];
    const tm = taskModel(th, dth);
    const p1 = th[0], p12 = th[0] + th[1];
    const w1 = dth[0], w12 = dth[0] + dth[1];
    expect(tm.Jdqd[0]).toBeCloseTo(-Math.cos(p1) * w1 * w1 - Math.cos(p12) * w12 * w12, 5);
    expect(tm.Jdqd[1]).toBeCloseTo(-Math.sin(p1) * w1 * w1 - Math.sin(p12) * w12 * w12, 5);
  });
});

describe('controllers', () => {
  it('position control: force = series stiffness × height error, or nothing at all', () => {
    const r = run({ mode: 'position', ke: 2e4, delta: 0.01, amp: 0 }, 2);
    const expected = (5000 * 2e4) / (5000 + 2e4) * 0.01;
    expect(Math.abs(r.mean - expected) / expected).toBeLessThan(0.05);
    expect(run({ mode: 'position', ke: 2e4, delta: -0.005, amp: 0 }, 2).fmax).toBe(0);
  });

  it('force control converges on a medium table but chatters on a hard one until the gain drops', () => {
    const med = run({ mode: 'force', ke: 2e4, amp: 0 }, 2);
    expect(Math.abs(med.mean - 10)).toBeLessThan(0.1);
    expect(med.lost).toBe(0);
    const hard = run({ mode: 'force', ke: 2e5, amp: 0 }, 3);
    expect(hard.lost).toBeGreaterThan(0);
    const hardLow = run({ mode: 'force', ke: 2e5, amp: 0, Kfp: 0.2 }, 3);
    expect(hardLow.lost).toBe(0);
    expect(Math.abs(hardLow.mean - 10)).toBeLessThan(0.2);
  });

  it('force control without damping does not settle', () => {
    const r = run({ mode: 'force', ke: 2e4, amp: 0, Kdamp: 0 }, 3);
    expect(r.lost > 0 || !r.sim.healthy() || r.fmax - r.fmin > 5).toBe(true);
  });

  it('hybrid control tracks the wiping motion and the normal force at once', () => {
    const r = run({ mode: 'hybrid', ke: 2e4, amp: 0.25, freq: 0.25, alpha: 0.15 }, 4, 2);
    expect(Math.abs(r.fmin - 10)).toBeLessThan(0.5);
    expect(Math.abs(r.fmax - 10)).toBeLessThan(0.5);
    expect(r.emax).toBeLessThan(0.002);
  });

  it('conflicting normal position + force: force runs away or the tool lifts off', () => {
    const up = run({ mode: 'conflict', ke: 2e4, delta: 0.01, amp: 0.25 }, 6);
    expect(up.mean).toBeGreaterThan(30);
    const down = run({ mode: 'conflict', ke: 2e4, delta: -0.01, amp: 0.25 }, 4);
    expect(down.fmax).toBe(0);
  });
});
