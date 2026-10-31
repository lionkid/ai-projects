import { describe, expect, it } from 'vitest';
import {
  HIT_LINK1, armPoints, blockedFraction, checkPath, computeCobsCodes, componentAt, coverClearance, exactClearance, linkCollisions, pathAt,
  torusComponents,
} from './cspace';
import { LINKS, LINK_HALF_WIDTH as W, SCENARIOS } from './scenarios';

const byKey = (k: string) => SCENARIOS.find((s) => s.key === k)!;
const N = 128;

describe('lesson scenarios', () => {
  it('every scenario starts and ends in C-free', () => {
    for (const s of SCENARIOS) {
      expect(exactClearance(LINKS, W, s.S, s.obstacles).d).toBeGreaterThan(0.1);
      expect(exactClearance(LINKS, W, s.G, s.obstacles).d).toBeGreaterThan(0.1);
      expect(exactClearance(LINKS, W, s.q, s.obstacles).d).toBeGreaterThan(0);
    }
  });
  it("'single': the straight joint path hits the disk with the arm while the tip stays clear, yet a path exists", () => {
    const s = byKey('single');
    expect(checkPath([s.S, s.G], 'exact', LINKS, W, s.obstacles).hits).toBeGreaterThan(10);
    let tipMin = Infinity;
    for (let k = 0; k <= 200; k++) {
      const tip = armPoints(LINKS, pathAt([s.S, s.G], k / 200))[2];
      const o = s.obstacles[0];
      tipMin = Math.min(tipMin, Math.hypot(tip[0] - o.c[0], tip[1] - o.c[1]) - o.r);
    }
    expect(tipMin).toBeGreaterThan(0.5);
    const comp = torusComponents(computeCobsCodes(N, 'exact', LINKS, W, s.obstacles), N);
    expect(comp.count).toBe(1);
    expect(componentAt(comp, N, s.S)).toBe(componentAt(comp, N, s.G));
    // going the other way round (through the ±180° seam) via one waypoint is collision-free
    expect(checkPath([s.S, [(-130 * Math.PI) / 180, 0], s.G], 'exact', LINKS, W, s.obstacles).hits).toBe(0);
  });
  it("'single': the disk is reachable by link 1, so its C-obstacle contains a full-height band", () => {
    const s = byKey('single');
    const codes = computeCobsCodes(N, 'exact', LINKS, W, s.obstacles);
    expect(codes.some((v) => (v & HIT_LINK1) !== 0)).toBe(true);
  });
  it("'behind': the shortest joint path crosses the seam and collides with link 1", () => {
    const s = byKey('behind');
    const chk = checkPath([s.S, s.G], 'exact', LINKS, W, s.obstacles);
    expect(chk.hits).toBeGreaterThan(0);
    expect(chk.samples.some((x) => x.hit && Math.abs(Math.abs(x.q[0]) - Math.PI) < 0.3)).toBe(true);
    expect(linkCollisions('exact', LINKS, W, [Math.PI, 0.5], s.obstacles) & HIT_LINK1).toBe(HIT_LINK1);
  });
  it("'near': the parked configuration is free but close to the disk", () => {
    const s = byKey('near');
    const d = exactClearance(LINKS, W, s.q, s.obstacles).d;
    expect(d).toBeGreaterThan(0.02);
    expect(d).toBeLessThan(0.2);
  });
  it("'pinch': start and goal lie in different components of C-free", () => {
    const s = byKey('pinch');
    const comp = torusComponents(computeCobsCodes(N, 'exact', LINKS, W, s.obstacles), N);
    expect(comp.count).toBe(2);
    expect(componentAt(comp, N, s.S)).not.toBe(componentAt(comp, N, s.G));
  });
});

describe('quiz claims', () => {
  it("lengthening L2 in 'single' leaves the link-1 band untouched and only grows the C-obstacle (13.6% → 14.5%)", () => {
    const s = byKey('single');
    const a = computeCobsCodes(N, 'exact', LINKS, W, s.obstacles);
    const b = computeCobsCodes(N, 'exact', [1, 1.3], W, s.obstacles);
    for (let k = 0; k < a.length; k++) {
      expect(a[k] & HIT_LINK1).toBe(b[k] & HIT_LINK1);
      if (a[k]) expect(b[k]).not.toBe(0); // the longer forearm sweeps a superset of the shorter one
    }
    const fa = blockedFraction(a), fb = blockedFraction(b);
    expect(fa).toBeGreaterThan(0.131); expect(fa).toBeLessThan(0.141);
    expect(fb).toBeGreaterThan(0.140); expect(fb).toBeLessThan(0.150);
  });
  it('a positive cover clearance is a lower bound on the true clearance', () => {
    const s = byKey('near');
    for (let k = 0; k < 400; k++) {
      const q: [number, number] = [-Math.PI + (2 * Math.PI * k) / 400, Math.cos(3 * k) * Math.PI];
      const cv = coverClearance(LINKS, W, q, s.obstacles).d;
      if (cv > 0) expect(exactClearance(LINKS, W, q, s.obstacles).d).toBeGreaterThanOrEqual(cv - 1e-12);
    }
  });
});

describe('per-link query', () => {
  it('matches the coded raster cell by cell', () => {
    const s = byKey('single');
    const n = 32;
    const codes = computeCobsCodes(n, 'exact', LINKS, W, s.obstacles);
    for (let j = 0; j < n; j++) for (let i = 0; i < n; i++) {
      const q: [number, number] = [-Math.PI + ((i + 0.5) / n) * 2 * Math.PI, -Math.PI + ((j + 0.5) / n) * 2 * Math.PI];
      expect(linkCollisions('exact', LINKS, W, q, s.obstacles)).toBe(codes[j * n + i]);
    }
  });
});
