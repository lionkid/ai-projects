import { describe, expect, it } from 'vitest';
import {
  type Vec, Adjoint, InverseDynamics, JacobianBody, MassMatrix, MatrixLog6, TransInv, FKinSpace,
  RpToTrans, RotZ, matMul, matVec, planar2R, rrr3, se3ToVec, transpose, ur5, eye,
} from '@/mr';
import {
  type ChainModel, baseCurves, bodyWrench, bodyWrenchParts, boxSpatialInertia, holdLoadWrench, quinticTrajectory, rneaTrace,
  scaleCurves, spinAt, torqueParts, trajectoryTorques, transformInertia, wrenchInFrame, growthExponent,
} from './rne';

const asModel = (r: typeof rrr3): ChainModel => ({ M: r.M, Mlist: r.Mlist!, Glist: r.Glist!, Slist: r.Slist });
const MODELS: [string, ChainModel][] = [['rrr3', asModel(rrr3)], ['ur5', asModel(ur5)], ['planar2R', asModel(planar2R)]];
const G = [0, 0, -9.81];

function state(n: number, seed: number): { th: Vec; dth: Vec; ddth: Vec; F: Vec } {
  const r = (k: number) => Math.sin(seed * 12.9898 + k * 78.233) * 1.7;
  return {
    th: Array.from({ length: n }, (_, i) => r(i)),
    dth: Array.from({ length: n }, (_, i) => r(i + 10)),
    ddth: Array.from({ length: n }, (_, i) => r(i + 20)),
    F: Array.from({ length: 6 }, (_, i) => r(i + 30)),
  };
}

const maxAbsDiff = (a: Vec, b: Vec) => Math.max(...a.map((v, i) => Math.abs(v - b[i])));

describe('traced recursive Newton–Euler', () => {
  it('matches InverseDynamics from @/mr on rrr3, UR5 and the planar 2R arm', () => {
    for (const [, m] of MODELS) {
      const n = m.Glist.length;
      for (let seed = 1; seed <= 5; seed++) {
        const s = state(n, seed);
        const tr = rneaTrace(s.th, s.dth, s.ddth, G, s.F, m);
        const ref = InverseDynamics(s.th, s.dth, s.ddth, G, s.F, m.Mlist, m.Glist, m.Slist);
        expect(maxAbsDiff(tr.tau, ref)).toBeLessThan(1e-9);
        expect(tr.V).toHaveLength(n + 1);
        expect(maxAbsDiff(tr.Vd[0], [0, 0, 0, 0, 0, 9.81])).toBe(0);
        expect(tr.F[n]).toEqual(s.F);
        expect(tr.poses).toHaveLength(n + 1);
      }
    }
  });

  it('link twists are the body twists of the link frames (finite differences of T_{0,i})', () => {
    const m = asModel(rrr3);
    const s = state(3, 7);
    const h = 1e-6;
    const tr = rneaTrace(s.th, s.dth, s.ddth, G, [0, 0, 0, 0, 0, 0], m);
    const trP = rneaTrace(s.th.map((q, i) => q + s.dth[i] * h), s.dth, s.ddth, G, [0, 0, 0, 0, 0, 0], m);
    for (let i = 0; i < 3; i++) {
      // V_i = log(T_{0,i}(t)^-1 T_{0,i}(t + h)) / h
      const Vnum = se3ToVec(MatrixLog6(matMul(TransInv(tr.poses[i]), trP.poses[i]))).map((v) => v / h);
      expect(maxAbsDiff(tr.V[i + 1], Vnum)).toBeLessThan(1e-4);
    }
  });

  it('torque on each joint is the projection of F_i on A_i', () => {
    const m = asModel(ur5);
    const s = state(6, 3);
    const tr = rneaTrace(s.th, s.dth, s.ddth, G, s.F, m);
    for (let i = 0; i < 6; i++) {
      const p = tr.F[i].reduce((acc, v, k) => acc + v * tr.A[i][k], 0);
      expect(p).toBeCloseTo(tr.tau[i], 10);
    }
  });
});

describe('torque decomposition', () => {
  it('inertial + velocity + gravity + load = total, and inertial = M(θ) θ̈', () => {
    for (const [, m] of MODELS) {
      const n = m.Glist.length;
      const s = state(n, 11);
      const p = torqueParts(m, s.th, s.dth, s.ddth, G, s.F);
      const sum = p.inertial.map((v, i) => v + p.velocity[i] + p.gravity[i] + p.load[i]);
      expect(maxAbsDiff(sum, p.total)).toBeLessThan(1e-9);
      const Mq = matVec(MassMatrix(s.th, m.Mlist, m.Glist, m.Slist), s.ddth);
      expect(maxAbsDiff(p.inertial, Mq)).toBeLessThan(1e-9);
    }
  });

  it('the tip-load torques are J_bᵀ F_tip; a held mass loads the horizontal rrr3 arm as expected', () => {
    const m = asModel(rrr3);
    const th = [0.4, -0.3, 0.8];
    const F = holdLoadWrench(FKinSpace(m.M, m.Slist, th), 1.5, G);
    const p = torqueParts(m, th, [0, 0, 0], [0, 0, 0], G, F);
    const Jb = JacobianBody(rrr3.Blist, th);
    expect(maxAbsDiff(p.load, matVec(transpose(Jb), F))).toBeLessThan(1e-10);
    // horizontal arm (θ = 0): the weight m g at reach L1 + L2 pulls the shoulder and elbow down (+θ direction)
    const F0 = holdLoadWrench(m.M, 2, G);
    const p0 = torqueParts(m, [0, 0, 0], [0, 0, 0], [0, 0, 0], G, F0);
    expect(p0.load[0]).toBeCloseTo(0, 10);
    expect(p0.load[1]).toBeCloseTo(-2 * 9.81 * 0.7, 9);
    expect(p0.load[2]).toBeCloseTo(-2 * 9.81 * 0.3, 9);
  });

  it('halving the duration multiplies the inertial and velocity parts by 4 and leaves gravity unchanged', () => {
    const m = asModel(rrr3);
    const a = [-1, 0, 0.8], b = [1, -0.7, -0.4];
    const slow = trajectoryTorques(m, quinticTrajectory(a, b, 2, 21), G, 0.5);
    const fast = trajectoryTorques(m, quinticTrajectory(a, b, 1, 21), G, 0.5);
    for (let k = 0; k < 21; k++) {
      for (let i = 0; i < 3; i++) {
        expect(fast[k].inertial[i]).toBeCloseTo(4 * slow[k].inertial[i], 9);
        expect(fast[k].velocity[i]).toBeCloseTo(4 * slow[k].velocity[i], 9);
        expect(fast[k].gravity[i]).toBeCloseTo(slow[k].gravity[i], 12);
        expect(fast[k].load[i]).toBeCloseTo(slow[k].load[i], 12);
      }
    }
  });

  it('rescaled base curves equal a direct computation for any duration, load and gravity setting', () => {
    const m = asModel(rrr3);
    const a = [-0.8, 0, 0.8], b = [0.8, -0.8, -0.5];
    const base = baseCurves(m, a, b, 31, G);
    for (const [Tf, load] of [[1.7, 0.8], [0.6, 0], [3, 2.5]]) {
      const direct = trajectoryTorques(m, quinticTrajectory(a, b, Tf, 31), G, load);
      const scaled = scaleCurves(base, Tf, true, load);
      for (let k = 0; k < 31; k++) expect(maxAbsDiff(scaled[k].total, direct[k].total)).toBeLessThan(1e-9);
    }
    const noG = scaleCurves(base, 2, false, 1.5);
    const directNoG = trajectoryTorques(m, quinticTrajectory(a, b, 2, 31), [0, 0, 0], 1.5);
    for (let k = 0; k < 31; k++) expect(maxAbsDiff(noG[k].total, directNoG[k].total)).toBeLessThan(1e-9);
  });

  it('quintic trajectories start and end at rest and are internally consistent', () => {
    const tr = quinticTrajectory([0, 1], [2, -1], 1.5, 301);
    expect(maxAbsDiff(tr.dtheta[0], [0, 0])).toBe(0);
    expect(tr.ddtheta[300][0]).toBeCloseTo(0, 12);
    expect(tr.theta[300][1]).toBeCloseTo(-1, 12);
    const dt = 1.5 / 300;
    for (const k of [40, 150, 222]) {
      expect(tr.dtheta[k][0]).toBeCloseTo((tr.theta[k + 1][0] - tr.theta[k - 1][0]) / (2 * dt), 3);
      expect(tr.ddtheta[k][1]).toBeCloseTo((tr.dtheta[k + 1][1] - tr.dtheta[k - 1][1]) / (2 * dt), 3);
    }
  });
});

describe('single rigid body', () => {
  const Gb = boxSpatialInertia(3, [0.4, 0.2, 0.06]);
  const I = [Gb[0][0], Gb[1][1], Gb[2][2]];

  it('steady spin about a tilted axis through the COM needs exactly ω × Iω, and nothing without the ad term', () => {
    const a = 0.5;
    const w = [Math.sin(a) * 4, 0, Math.cos(a) * 4];
    const V = [...w, 0, 0, 0];
    const F = bodyWrench(Gb, V, [0, 0, 0, 0, 0, 0]);
    const Iw = [I[0] * w[0], I[1] * w[1], I[2] * w[2]];
    const wxIw = [w[1] * Iw[2] - w[2] * Iw[1], w[2] * Iw[0] - w[0] * Iw[2], w[0] * Iw[1] - w[1] * Iw[0]];
    expect(maxAbsDiff(F, [...wxIw, 0, 0, 0])).toBeLessThan(1e-12);
    expect(Math.abs(F[1])).toBeGreaterThan(0.1);
    expect(maxAbsDiff(bodyWrench(Gb, V, [0, 0, 0, 0, 0, 0], false), [0, 0, 0, 0, 0, 0])).toBe(0);
    // about a principal axis the velocity term vanishes
    expect(maxAbsDiff(bodyWrench(Gb, [0, 0, 4, 0, 0, 0], [0, 0, 0, 0, 0, 0]), [0, 0, 0, 0, 0, 0])).toBeLessThan(1e-14);
    const parts = bodyWrenchParts(Gb, V, [0.3, 0, 0.1, 0, 0, 0]);
    expect(maxAbsDiff(parts.inertial.map((v, i) => v + parts.velocity[i]), bodyWrench(Gb, V, [0.3, 0, 0.1, 0, 0, 0]))).toBeLessThan(1e-14);
  });

  it('G_a = Ad_{T_ba}ᵀ G_b Ad_{T_ba}: same kinetic energy, same wrench, parallel-axis term', () => {
    const Tba = RpToTrans(RotZ(0.4), [0.15, -0.05, 0.02]);
    const Ga = transformInertia(Gb, Tba);
    const Va = [0.3, -1.2, 2, 0.4, 0.1, -0.5];
    const Vb = matVec(Adjoint(Tba), Va);
    const Ka = 0.5 * Va.reduce((acc, v, i) => acc + v * matVec(Ga, Va)[i], 0);
    const Kb = 0.5 * Vb.reduce((acc, v, i) => acc + v * matVec(Gb, Vb)[i], 0);
    expect(Ka).toBeCloseTo(Kb, 12);
    const w = wrenchInFrame(Gb, Tba, Vb, matVec(Adjoint(Tba), [1, 0, -2, 0.5, 0, 0.3]));
    expect(maxAbsDiff(w.Fa, w.FaFromFb)).toBeLessThan(1e-12);
    // pure offset r along x: I_a,zz = I_zz + m r²
    const Ga2 = transformInertia(Gb, RpToTrans(eye(3), [0.2, 0, 0]));
    expect(Ga2[2][2]).toBeCloseTo(I[2] + 3 * 0.04, 12);
    // coupling blocks m[p]ᵀ (top right) and m[p] (bottom left) with p = (0.2, 0, 0)
    expect(Ga2[1][5]).toBeCloseTo(0.2 * 3, 12);
    expect(Ga2[5][1]).toBeCloseTo(0.2 * 3, 12);
    expect(Ga2[2][4]).toBeCloseTo(-0.2 * 3, 12);
  });

  it('spinAt gives the body twist of T_sb(t) and V̇_b = S_b φ̈', () => {
    const spec = { axis: [Math.sin(0.4), 0, Math.cos(0.4)], point: [0.18, 0, 0], omega0: 3, varying: true, period: 4, Ts0: RpToTrans(eye(3), [0, 0, 0.5]) };
    const h = 1e-6;
    for (const t of [0.2, 1.3, 2.9]) {
      const a = spinAt(spec, t), b = spinAt(spec, t + h), c = spinAt(spec, t - h);
      const Vnum = se3ToVec(MatrixLog6(matMul(TransInv(c.Tsb), b.Tsb))).map((v) => v / (2 * h));
      expect(maxAbsDiff(a.Vb, Vnum)).toBeLessThan(1e-6);
      expect(maxAbsDiff(a.Vdb, a.Vb.map((_, i) => (b.Vb[i] - c.Vb[i]) / (2 * h)))).toBeLessThan(1e-5);
    }
    // the pivot point stays fixed in space
    const p0 = matVec(spinAt(spec, 0).Tsb, [0.18, 0, 0, 1]);
    const p1 = matVec(spinAt(spec, 1.7).Tsb, [0.18, 0, 0, 1]);
    expect(maxAbsDiff(p0, p1)).toBeLessThan(1e-12);
  });
});

describe('growthExponent', () => {
  it('recovers the exponent of a power law and is not forced by a model choice', () => {
    const ns = [6, 12, 24];
    expect(growthExponent(ns, ns.map((n) => 3 * n))).toBeCloseTo(1, 12);
    expect(growthExponent(ns, ns.map((n) => 0.5 * n * n))).toBeCloseTo(2, 12);
    // a large constant overhead lowers the measured exponent below 1 (a fit of a + b·n would still claim ×2)
    expect(growthExponent(ns, ns.map((n) => 100 + n))).toBeLessThan(0.2);
    expect(Number.isNaN(growthExponent([5], [1]))).toBe(true);
  });
});
