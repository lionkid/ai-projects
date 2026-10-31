import { describe, expect, it } from 'vitest';
import { distPointPolygon, type P2 } from '@/mr';
import {
  BASIN_GOAL, BASIN_STUCK, BasinJob, BiasedRRT, D_CONTACT, DEFAULT_ROBOT, PointRobot, boxObs, contourSegments, evalField, fieldEnv, mulberry32,
  navDescentField, navEnv, navFunction, nearestClearance, obstacleClearance, potentialAt, refineMinimum, saturate,
  type FieldParams, type Obstacle, type RobotParams,
} from './potential';
import { SPHERE_WORLD, WORLD, makeScene, type SceneId } from './scenes';

const obsOf = (id: SceneId): Obstacle[] => makeScene(id).groups.flatMap((g) => g.parts);
const params = (id: SceneId, o: Partial<FieldParams> = {}): FieldParams => ({ kAtt: 1, kRep: 1, d0: 1, goal: makeScene(id).goal, ...o });
const run = (id: SceneId, o: Partial<FieldParams> = {}, r: Partial<RobotParams> = {}): PointRobot =>
  new PointRobot(fieldEnv(params(id, o), obsOf(id), WORLD), { ...DEFAULT_ROBOT, ...r }, makeScene(id).start).run();
const fieldFn = (id: SceneId, o: Partial<FieldParams> = {}) => {
  const p = params(id, o), obs = obsOf(id);
  return (x: number, y: number) => { const e = evalField(p, obs, x, y); return { U: e.U, g: e.g }; };
};

describe('clearance and potential', () => {
  it('box distance matches @/mr distPointPolygon and its direction is the gradient of d', () => {
    const o = boxObs(2, 1, 4, 3);
    const poly = (o as Extract<Obstacle, { kind: 'poly' }>).pts;
    const rng = mulberry32(3);
    for (let k = 0; k < 300; k++) {
      const q: P2 = [rng() * 6, rng() * 4];
      const c = obstacleClearance(o, q[0], q[1]);
      expect(c.d).toBeCloseTo(distPointPolygon(q, poly), 10);
      if (c.d > 1e-3) {
        const h = 1e-6;
        const gx = (obstacleClearance(o, q[0] + h, q[1]).d - obstacleClearance(o, q[0] - h, q[1]).d) / (2 * h);
        const gy = (obstacleClearance(o, q[0], q[1] + h).d - obstacleClearance(o, q[0], q[1] - h).d) / (2 * h);
        expect(gx).toBeCloseTo(c.n[0], 5);
        expect(gy).toBeCloseTo(c.n[1], 5);
      }
    }
    expect(obstacleClearance(o, 3, 2).d).toBe(0);
  });

  it('∇U matches finite differences, and the repulsive term vanishes smoothly at d0', () => {
    const p = params('utrap', { kAtt: 0.7, kRep: 1.3, d0: 1.2 });
    const obs = obsOf('utrap');
    const rng = mulberry32(11);
    let checked = 0;
    while (checked < 200) {
      const x = rng() * 10, y = rng() * 7.5;
      const e = evalField(p, obs, x, y);
      if (e.d < 0.06) continue; // the clamp region is deliberately not smooth
      const h = 1e-6;
      const gx = (potentialAt(p, obs, x + h, y) - potentialAt(p, obs, x - h, y)) / (2 * h);
      const gy = (potentialAt(p, obs, x, y + h) - potentialAt(p, obs, x, y - h)) / (2 * h);
      const scale = Math.max(1, Math.hypot(e.g[0], e.g[1]));
      expect(Math.abs(gx - e.g[0]) / scale).toBeLessThan(1e-4);
      expect(Math.abs(gy - e.g[1]) / scale).toBeLessThan(1e-4);
      expect(e.U).toBeCloseTo(potentialAt(p, obs, x, y), 10);
      checked++;
    }
    // just inside / outside the influence distance of a single disk
    const disk: Obstacle[] = [{ kind: 'disk', c: [5, 5], r: 1 }];
    const q = { kAtt: 0, kRep: 2, d0: 1, goal: [0, 0] as P2 };
    expect(evalField(q, disk, 5, 7.0001).Urep).toBe(0);
    expect(evalField(q, disk, 5, 6.999).Urep).toBeLessThan(1e-5);
    expect(Math.hypot(...evalField(q, disk, 5, 6.999).gRep)).toBeLessThan(1e-2);
    expect(evalField(q, disk, 5, 6.2).gRep[1]).toBeLessThan(0); // ∇U points into the obstacle, −∇U pushes away
  });
});

describe('first-order robot (gradient descent, Δt = 0.1 s)', () => {
  it('slides to the goal on the open map along an almost straight line', () => {
    const r = run('open');
    const sc = makeScene('open');
    expect(r.status).toBe('reached');
    expect(r.pathLength).toBeLessThan(1.02 * Math.hypot(sc.goal[0] - sc.start[0], sc.goal[1] - sc.start[1]));
  });

  it('gets around convex obstacles', () => {
    expect(run('convex').status).toBe('reached');
  });

  it('is trapped by the U: a true local minimum where attraction and repulsion cancel', () => {
    const r = run('utrap');
    expect(r.status).toBe('stuck');
    const m = refineMinimum(fieldFn('utrap'), r.q);
    expect(m.gradNorm).toBeLessThan(1e-6);
    expect(Math.hypot(m.q[0] - 8.9, m.q[1] - 3.75)).toBeGreaterThan(2);
    const e = evalField(params('utrap'), obsOf('utrap'), m.q[0], m.q[1]);
    const a = Math.hypot(...e.gAtt), b = Math.hypot(...e.gRep);
    expect(a).toBeGreaterThan(1);
    expect(b / a).toBeCloseTo(1, 4);
    expect((e.gAtt[0] * e.gRep[0] + e.gAtt[1] * e.gRep[1]) / (a * b)).toBeCloseTo(-1, 6);
    // doubling the attraction moves the minimum deeper into the cup but does not remove it
    const r2 = run('utrap', { kAtt: 2 });
    expect(r2.status).toBe('stuck');
    const m2 = refineMinimum(fieldFn('utrap', { kAtt: 2 }), r2.q);
    expect(m2.gradNorm).toBeLessThan(1e-6);
    expect(m2.q[0]).toBeGreaterThan(m.q[0] + 0.05);
  });

  it('corridor: small k passes smoothly, k = 1.5 zig-zags (longer path), k = 3 cannot enter', () => {
    const smooth = run('corridor', { kRep: 0.3 });
    const zig = run('corridor', { kRep: 1.5 });
    const blocked = run('corridor', { kRep: 3 });
    expect(smooth.status).toBe('reached');
    expect(zig.status).toBe('reached');
    expect(zig.pathLength).toBeGreaterThan(1.25 * smooth.pathLength);
    expect(blocked.status).toBe('stuck');
    expect(blocked.q[0]).toBeLessThan(3.2); // stopped in front of the corridor mouth
  });
});

describe('second-order robot q̈ = sat(−∇U) − b q̇', () => {
  it('overshoots the goal with little damping and still settles; heavy damping does not overshoot', () => {
    const light = run('open', {}, { model: 'second', damping: 0.5 });
    const heavy = run('open', {}, { model: 'second', damping: 2.5 });
    expect(light.status).toBe('reached');
    expect(light.overshoot).toBeGreaterThan(1);
    expect(heavy.status).toBe('reached');
    expect(heavy.overshoot).toBeLessThan(0.3);
  });

  it('loses energy U + ½‖v‖² when the force is not clipped', () => {
    const p: FieldParams = { kAtt: 1, kRep: 0, d0: 1, goal: [5, 4] };
    const env = fieldEnv(p, []);
    const r = new PointRobot(env, { ...DEFAULT_ROBOT, model: 'second', damping: 0.4, aMax: 100 }, [6.5, 4.5]);
    const E = () => env.field(r.q[0], r.q[1]).U + 0.5 * (r.v[0] ** 2 + r.v[1] ** 2);
    let prev = E();
    for (let k = 0; k < 2400; k++) {
      r.step(1 / 240);
      const e = E();
      expect(e).toBeLessThan(prev + 1e-4);
      prev = e;
    }
    expect(prev).toBeLessThan(0.2 * 0.5 * (1.5 ** 2 + 0.5 ** 2));
  });

  it('with little damping inertia carries it past the corridor mouth; the contact guard keeps it out of walls', () => {
    const r = run('corridor', { kRep: 1 }, { model: 'second', damping: 0.8 });
    expect(r.status).toBe('stuck');
    const obs = obsOf('corridor');
    for (const q of r.trace) expect(nearestClearance(obs, q[0], q[1]).d).toBeGreaterThanOrEqual(D_CONTACT * 0.5);
    expect(run('corridor', { kRep: 1 }, { model: 'second', damping: 1.5 }).status).toBe('reached');
  });

  it('saturate keeps the direction and clips the length', () => {
    expect(saturate([3, 4], 1)).toEqual([0.6, 0.8]);
    expect(saturate([0.3, 0.4], 1)).toEqual([0.3, 0.4]);
  });
});

describe('basins of attraction', () => {
  const basin = (id: SceneId) => {
    const sc = makeScene(id), obs = obsOf(id);
    const job = new BasinJob(WORLD, 50, 38, fieldFn(id), (x, y) => nearestClearance(obs, x, y).d <= 0.02, sc.goal, { step: 0.1, maxSteps: 1500, goalTol: 0.15 });
    job.run(1e9);
    return job;
  };
  it('the U has exactly one trap and it catches a large region of starts', () => {
    const j = basin('utrap');
    expect(j.done).toBe(true);
    expect(j.minima).toHaveLength(1);
    expect(Math.hypot(j.minima[0][0] - 5.67, j.minima[0][1] - 3.75)).toBeLessThan(0.15);
    expect(j.goalFraction()).toBeGreaterThan(0.5);
    expect(j.goalFraction()).toBeLessThan(0.8);
  });
  it('is mirror-symmetric on the symmetric U map and agrees with the simulated robot', () => {
    const j = basin('utrap');
    const nx = j.nx, ny = j.ny;
    let mismatched = 0;
    for (let r = 0; r < ny; r++) for (let i = 0; i < nx; i++) if (j.cells[r * nx + i] !== j.cells[(ny - 1 - r) * nx + i]) mismatched++;
    expect(mismatched).toBe(0);
    // starts near the top-left / bottom-left corners pass over or under the U; (1.5, 6) falls in
    for (const [q, fate] of [[[1, 7], BASIN_GOAL], [[1, 0.5], BASIN_GOAL], [[1.5, 6], BASIN_STUCK], [[0.5, 5], BASIN_STUCK]] as [P2, number][]) {
      expect(j.cells[j.cellOf(q[0], q[1])], `start ${q}`).toBe(fate);
      const r = new PointRobot(fieldEnv(params('utrap'), obsOf('utrap'), WORLD), DEFAULT_ROBOT, [q[0], q[1]]).run();
      expect(r.status, `robot from ${q}`).toBe(fate === BASIN_GOAL ? 'reached' : 'stuck');
    }
  });

  it('convex obstacles on this map leave every start able to reach the goal', () => {
    const j = basin('convex');
    expect(j.goalFraction()).toBe(1);
    expect(j.minima).toHaveLength(0);
  });
});

describe('sphere-world navigation function', () => {
  const goal = makeScene('sphere').goal;
  it('is 0 at the goal, → 1 at the boundaries, and its gradient matches finite differences', () => {
    expect(navFunction(SPHERE_WORLD, goal, 2, goal[0], goal[1]).phi).toBe(0);
    const d = SPHERE_WORLD.disks[0];
    expect(navFunction(SPHERE_WORLD, goal, 2, d.c[0] + d.r + 1e-7, d.c[1]).phi).toBeGreaterThan(0.999);
    expect(navFunction(SPHERE_WORLD, goal, 2, d.c[0], d.c[1]).free).toBe(false);
    const rng = mulberry32(5);
    for (const kappa of [1, 2, 5]) {
      let n = 0;
      while (n < 60) {
        const x = 1.5 + rng() * 7, y = 0.5 + rng() * 6.5;
        const e = navFunction(SPHERE_WORLD, goal, kappa, x, y);
        if (!e.free || e.phi > 0.999 || e.phi < 1e-6) continue;
        const h = 1e-6;
        const gx = (navFunction(SPHERE_WORLD, goal, kappa, x + h, y).phi - navFunction(SPHERE_WORLD, goal, kappa, x - h, y).phi) / (2 * h);
        const gy = (navFunction(SPHERE_WORLD, goal, kappa, x, y + h).phi - navFunction(SPHERE_WORLD, goal, kappa, x, y - h).phi) / (2 * h);
        const gn = Math.hypot(e.g[0], e.g[1]);
        expect(Math.hypot(gx - e.g[0], gy - e.g[1])).toBeLessThan(1e-5 * Math.max(1, gn) + 1e-8);
        if (gn > 1e-6) expect(-(e.g[0] * e.dir[0] + e.g[1] * e.dir[1]) / gn).toBeCloseTo(1, 8);
        n++;
      }
    }
  });

  it('κ = 1 has a spurious minimum that traps the robot; κ = 5 leaves only the goal', () => {
    const sc = makeScene('sphere');
    const basin = (kappa: number) => {
      const j = new BasinJob(WORLD, 50, 38, navDescentField(SPHERE_WORLD, goal, kappa),
        (x, y) => !navFunction(SPHERE_WORLD, goal, kappa, x, y).free, goal, { step: 0.08, maxSteps: 1500, goalTol: 0.15 });
      j.run(1e9);
      return j;
    };
    const low = basin(1), high = basin(5);
    expect(low.minima.length).toBeGreaterThanOrEqual(1);
    expect(low.goalFraction()).toBeLessThan(0.8);
    expect(high.minima).toHaveLength(0);
    expect(high.goalFraction()).toBe(1);
    expect(new PointRobot(navEnv(SPHERE_WORLD, goal, 1), DEFAULT_ROBOT, sc.start).run().status).toBe('stuck');
    expect(new PointRobot(navEnv(SPHERE_WORLD, goal, 5), DEFAULT_ROBOT, sc.start).run().status).toBe('reached');
  });
});

describe('marching squares', () => {
  it('traces the circle r = 1 of f = x² + y² and nothing for a constant field', () => {
    const n = 81, b = { xmin: -2, xmax: 2, ymin: -2, ymax: 2 };
    const v = new Float64Array(n * n);
    for (let j = 0; j < n; j++) for (let i = 0; i < n; i++) { const x = -2 + (4 * i) / (n - 1), y = -2 + (4 * j) / (n - 1); v[j * n + i] = x * x + y * y; }
    const out: number[] = [];
    contourSegments(v, n, n, b, 1, out);
    expect(out.length / 4).toBeGreaterThan(40);
    for (let k = 0; k < out.length; k += 2) expect(Math.abs(Math.hypot(out[k], out[k + 1]) - 1)).toBeLessThan(0.01);
    const none: number[] = [];
    contourSegments(new Float64Array(n * n).fill(3), n, n, b, 1, none);
    expect(none).toHaveLength(0);
  });
});

describe('potential-biased RRT', () => {
  const sc = makeScene('utrap'), obs = obsOf('utrap'), p = params('utrap');
  const free = (x: number, y: number) => nearestClearance(obs, x, y).d > 0.05;
  const U = (x: number, y: number) => potentialAt(p, obs, x, y);
  const trial = (pBias: number, K: number, seed: number) =>
    new BiasedRRT(WORLD, free, U, sc.start, sc.goal, { delta: 0.4, eps: 0.05, maxIterations: 2000, pBias, K }, mulberry32(seed * 7919 + 1)).run();

  it('returns a collision-free path from start to goal', () => {
    const r = trial(0.5, 4, 1);
    expect(r.status).toBe('success');
    const path = r.path!;
    expect(path[0]).toEqual(sc.start);
    expect(path[path.length - 1]).toEqual(sc.goal);
    for (let i = 0; i + 1 < path.length; i++) expect(r.edgeFree(path[i], path[i + 1])).toBe(true);
  });

  it('a moderate bias needs fewer iterations than uniform sampling; trusting the potential alone fails in the trap', () => {
    let uni = 0, bias = 0, greedyOk = 0;
    for (let s = 1; s <= 30; s++) {
      uni += trial(0, 1, s).iterations;
      bias += trial(0.5, 4, s).iterations;
      if (trial(1, 16, s).status === 'success') greedyOk++;
    }
    expect(bias).toBeLessThan(0.85 * uni);
    expect(greedyOk).toBeLessThan(6);
  });
});
