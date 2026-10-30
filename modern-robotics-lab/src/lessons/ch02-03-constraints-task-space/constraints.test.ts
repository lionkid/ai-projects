import { describe, expect, it } from 'vitest';
import { MatrixExp6, VecTose3, matMul, planarFK } from '@/mr';
import {
  allowedBasis, bodyTwistStep, constraintResidual, holonomicCloud, isParked, lateralShiftPlan, mulberry32, penIK,
  pfaffianRow, planDuration, poseAlongPlan, poseError, qdotOf, reachableCloud, reversePlan, samplePlan, slidePlan,
  type Pose,
} from './constraints';

const toT = (q: Pose) => [
  [Math.cos(q.phi), -Math.sin(q.phi), 0, q.x],
  [Math.sin(q.phi), Math.cos(q.phi), 0, q.y],
  [0, 0, 1, 0],
  [0, 0, 0, 1],
];

describe('exact planar integration', () => {
  it('matches T · exp([V_b] h) computed with the @/mr SE(3) exponential', () => {
    const cases: [Pose, number, number, number, number][] = [
      [{ x: 0.3, y: -1, phi: 0.7 }, 0.8, 0, 1.2, 0.9],
      [{ x: -2, y: 0.5, phi: -2.5 }, -0.4, 0.3, -0.7, 1.7],
      [{ x: 1, y: 1, phi: 3 }, 0.5, -0.2, 0, 2],
      [{ x: 0, y: 0, phi: 0 }, 0.6, 0, 1e-12, 1],
    ];
    for (const [q, v, vs, w, h] of cases) {
      const T = matMul(toT(q), MatrixExp6(VecTose3([0, 0, w * h, v * h, vs * h, 0])));
      const r = bodyTwistStep(q, v, vs, w, h);
      expect(r.x).toBeCloseTo(T[0][3], 10);
      expect(r.y).toBeCloseTo(T[1][3], 10);
      expect(Math.cos(r.phi)).toBeCloseTo(T[0][0], 10);
      expect(Math.sin(r.phi)).toBeCloseTo(T[1][0], 10);
    }
  });

  it('a full circle returns exactly to the start', () => {
    const q0: Pose = { x: 0.4, y: -0.2, phi: 0.3 };
    let q = q0;
    for (let k = 0; k < 1000; k++) q = bodyTwistStep(q, 1, 0, 1, (2 * Math.PI) / 1000);
    expect(q.x).toBeCloseTo(q0.x, 9);
    expect(q.y).toBeCloseTo(q0.y, 9);
    expect(Math.abs(poseError(q, q0).dphi)).toBeLessThan(1e-9);
  });

  it('unicycle motion satisfies the Pfaffian constraint; side-slip violates it', () => {
    let q: Pose = { x: 0, y: 0, phi: 0.2 };
    const h = 1e-4;
    for (let k = 0; k < 50; k++) {
      const nq = bodyTwistStep(q, 0.7, 0, 0.9 * Math.sin(k), h);
      const fd: [number, number, number] = [(nq.phi - q.phi) / h, (nq.x - q.x) / h, (nq.y - q.y) / h];
      expect(Math.abs(constraintResidual((q.phi + nq.phi) / 2, fd))).toBeLessThan(1e-6);
      q = nq;
    }
    for (const phi of [0, 0.7, -2, 3]) {
      expect(constraintResidual(phi, qdotOf(phi, 0.8, 0, 0.5))).toBeCloseTo(0, 14);
      expect(constraintResidual(phi, qdotOf(phi, 0, 1, 0))).toBeCloseTo(-1, 14);
      const { g1, g2 } = allowedBasis(phi);
      const A = pfaffianRow(phi);
      expect(A[0] * g1[0] + A[1] * g1[1] + A[2] * g1[2]).toBeCloseTo(0, 14);
      expect(A[0] * g2[0] + A[1] * g2[1] + A[2] * g2[2]).toBeCloseTo(0, 14);
    }
  });
});

describe('plans', () => {
  it('the four-move plan shifts sideways with the heading restored', () => {
    for (const start of [{ x: 0, y: 0, phi: 0 }, { x: 1, y: 2, phi: 0.7 }, { x: -1, y: 0.5, phi: -2.9 }] as Pose[]) {
      for (const d of [1, -0.6]) {
        const plan = lateralShiftPlan(d);
        expect(plan).toHaveLength(4);
        expect(plan.every((g) => (g.vs ?? 0) === 0)).toBe(true);
        const end = poseAlongPlan(start, plan, planDuration(plan) + 1);
        expect(end.done).toBe(true);
        expect(end.pose.x).toBeCloseTo(start.x - d * Math.sin(start.phi), 9);
        expect(end.pose.y).toBeCloseTo(start.y + d * Math.cos(start.phi), 9);
        expect(Math.abs(poseError(end.pose, start).dphi)).toBeLessThan(1e-9);
      }
    }
  });

  it('a slide plan moves an omnidirectional base straight sideways', () => {
    const end = poseAlongPlan({ x: 0, y: 0, phi: Math.PI / 2 }, slidePlan(0.5), 100).pose;
    expect(end.x).toBeCloseTo(-0.5, 12);
    expect(end.y).toBeCloseTo(0, 12);
  });

  it('a reversed plan undoes the original and samples include both ends', () => {
    const start: Pose = { x: -1.2, y: -0.6, phi: 0.35 };
    const plan = [
      { v: 0.6, w: 0.5, T: 2, label: 'a' },
      { v: 0.6, w: -0.6, T: 1.8, label: 'b' },
    ];
    const end = poseAlongPlan(start, plan, 99).pose;
    const back = poseAlongPlan(end, reversePlan(plan), 99).pose;
    expect(back.x).toBeCloseTo(start.x, 9);
    expect(back.y).toBeCloseTo(start.y, 9);
    expect(Math.abs(poseError(back, start).dphi)).toBeLessThan(1e-9);
    const pts = samplePlan(start, plan, 0.1);
    expect(pts[0].x).toBe(start.x);
    expect(pts[0].phi).toBeCloseTo(start.phi, 12);
    expect(pts[pts.length - 1].x).toBeCloseTo(end.x, 12);
  });

  it('parking needs both position and wrapped heading', () => {
    const goal: Pose = { x: 0, y: 1, phi: Math.PI - 0.02 };
    expect(isParked({ x: 0.05, y: 1, phi: -Math.PI + 0.02 }, goal)).toBe(true);
    expect(isParked({ x: 0.05, y: 1, phi: 0 }, goal)).toBe(false);
    expect(isParked({ x: 0.3, y: 1, phi: goal.phi }, goal)).toBe(false);
  });
});

describe('reachable sets', () => {
  it('random unicycle inputs reach a spread of positions and every heading quadrant', () => {
    const cloud = reachableCloud({ x: 0, y: 0, phi: 0 }, 300, mulberry32(7));
    const xs = cloud.map((q) => q.x);
    const ys = cloud.map((q) => q.y);
    const std = (a: number[]) => { const m = a.reduce((s, v) => s + v, 0) / a.length; return Math.sqrt(a.reduce((s, v) => s + (v - m) ** 2, 0) / a.length); };
    expect(std(xs)).toBeGreaterThan(0.3);
    expect(std(ys)).toBeGreaterThan(0.3);
    const quadrants = new Set(cloud.map((q) => Math.floor(((q.phi + Math.PI) / (2 * Math.PI)) * 4)));
    expect(quadrants.size).toBe(4);
  });

  it('a holonomically constrained point always ends on its circle', () => {
    for (const p of holonomicCloud(1.1, 0.4, 200, mulberry32(3))) expect(Math.hypot(p[0], p[1])).toBeCloseTo(1.1, 12);
  });
});

describe('pen IK for the task-space card', () => {
  it('reaches reachable targets and stretches toward unreachable ones', () => {
    const ok = penIK(1, 0.6, [1.1, 0.5], [0.3, 0.5]);
    expect(ok.reachable).toBe(true);
    const tip = planarFK([1, 0.6], ok.th).tip;
    expect(tip[0]).toBeCloseTo(1.1, 9);
    expect(tip[1]).toBeCloseTo(0.5, 9);
    const far = penIK(1, 0.6, [2, 2], [0.3, 0.5]);
    expect(far.reachable).toBe(false);
    const ft = planarFK([1, 0.6], far.th).tip;
    expect(Math.hypot(ft[0], ft[1])).toBeCloseTo(1.6, 6);
    expect(Math.atan2(ft[1], ft[0])).toBeCloseTo(Math.PI / 4, 6);
    const near = penIK(1, 0.6, [0.1, 0], [0.3, 0.5]);
    expect(near.reachable).toBe(false);
    expect(Math.hypot(...(planarFK([1, 0.6], near.th).tip as [number, number]))).toBeCloseTo(0.4, 6);
  });
});
