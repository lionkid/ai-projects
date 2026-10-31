import { describe, expect, it } from 'vitest';
import {
  type P2, type V3, type Body, type Contact, makePoly, makeDisc, perimeterOf, boundaryAt, projectToBoundary, insideBody,
  bodyRadius, gyrationRadius, frictionAngle, coneEdges, unitConeEdges, coneCheck, forceWrench, edgeWrenches,
  isForceClosure, closureCertificate, originMargin, scaleMoments, wrenchRank, twoFingerTest, minGripForces, linprog,
  freeTwistRays, isFeasibleTwist, preferredFreeTwist, representativeFreeTwist, drivenFreeTwist, pushAlong, edgeLines,
  corRegion, twistAbout, hullFaces, hullFacets, dot3, accelTwist,
} from './grasp';

/** Deterministic pseudo-random numbers (mulberry32). */
function rng(seed: number): () => number {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

function randomConvexPolygon(r: () => number): P2[] {
  const n = 3 + Math.floor(r() * 5);
  const angles = Array.from({ length: n }, () => r() * 2 * Math.PI).sort((a, b) => a - b);
  return angles.map((a) => { const R = 0.6 + 0.6 * r(); return [R * Math.cos(a), R * Math.sin(a)] as P2; });
}

const square = makePoly('square', 'square', [[-0.75, -0.75], [0.75, -0.75], [0.75, 0.75], [-0.75, 0.75]]);
const tri = makePoly('tri', 'tri', [[0, 1.1], [-1.1 * Math.cos(Math.PI / 6), -0.55], [1.1 * Math.cos(Math.PI / 6), -0.55]]);
const disc = makeDisc('disc', 'disc', 0.85);

const at = (b: Body, q: P2): Contact => { const c = projectToBoundary(b, q); return { p: c.p, n: c.n }; };
const onDisc = (phiDeg: number): Contact => { const c = boundaryAt(disc, (disc.R * phiDeg * Math.PI) / 180); return { p: c.p, n: c.n }; };

describe('LP solver', () => {
  it('solves small problems and reports infeasible / unbounded', () => {
    const a = linprog([1, 1], [[1, 2]], [4]);
    expect(a.status).toBe('optimal');
    expect(a.x[1]).toBeCloseTo(2, 9);
    expect(linprog([1, 1], [[1, 1]], [-1]).status).toBe('infeasible');
    expect(linprog([-1, 0], [[1, -1]], [0]).status).toBe('unbounded');
    const red = linprog([1, 0], [[1, 1], [2, 2]], [1, 2]); // redundant equality
    expect(red.status).toBe('optimal');
    expect(red.x[1]).toBeCloseTo(1, 9);
  });
});

describe('bodies', () => {
  it('inward normals, projection round trip and perimeter', () => {
    expect(perimeterOf(square)).toBeCloseTo(6, 12);
    expect(perimeterOf(disc)).toBeCloseTo(2 * Math.PI * 0.85, 12);
    for (const b of [square, tri, disc]) {
      const P = perimeterOf(b);
      for (let s = 0.05; s < P; s += P / 17) {
        const c = boundaryAt(b, s);
        expect(c.n[0] * -c.p[0] + c.n[1] * -c.p[1]).toBeGreaterThan(0); // points towards the centroid
        const q: P2 = [c.p[0] - 0.2 * c.n[0], c.p[1] - 0.2 * c.n[1]];
        const c2 = projectToBoundary(b, q);
        expect(Math.hypot(c2.p[0] - c.p[0], c2.p[1] - c.p[1])).toBeLessThan(1e-9);
        expect(insideBody(b, [c.p[0] + 0.05 * c.n[0], c.p[1] + 0.05 * c.n[1]])).toBe(true);
      }
    }
    // disc normals point exactly at the centre
    const c = onDisc(37);
    expect(Math.abs(c.p[0] * c.n[1] - c.p[1] * c.n[0])).toBeLessThan(1e-12);
  });
  it('radius of gyration and characteristic length', () => {
    expect(gyrationRadius(square)).toBeCloseTo(Math.sqrt((1.5 * 1.5 + 1.5 * 1.5) / 12), 12);
    expect(gyrationRadius(disc)).toBeCloseTo(0.85 / Math.SQRT2, 12);
    expect(bodyRadius(square)).toBeCloseTo(0.75 * Math.SQRT2, 12);
    expect(bodyRadius(tri)).toBeCloseTo(1.1, 9);
  });
});

describe('friction cone', () => {
  it('edges sit at ±α about the normal and coneCheck matches |f_t| ≤ μ f_n', () => {
    const n: P2 = [0, 1];
    const mu = 0.5;
    const [e1, e2] = unitConeEdges(n, mu);
    expect(Math.atan2(Math.abs(e1[0]), e1[1])).toBeCloseTo(frictionAngle(mu), 12);
    expect(e1[0]).toBeLessThan(0); // CCW of an upward normal is to the left
    expect(e2[0]).toBeGreaterThan(0);
    const [k1, k2] = coneEdges(n, mu);
    expect(k1[1]).toBeCloseTo(1, 12);
    expect(k2[1]).toBeCloseTo(1, 12);
    expect(coneCheck(n, mu, [0.4, 1]).inside).toBe(true);
    expect(coneCheck(n, mu, [0.6, 1]).inside).toBe(false);
    expect(coneCheck(n, mu, [0, -1]).inside).toBe(false);
    expect(coneCheck(n, 0, [0, 2]).inside).toBe(true);
  });
});

describe('force closure', () => {
  it('two-finger criterion agrees with the LP certificate, the free-twist test and the hull margin', () => {
    const r = rng(21);
    let closed = 0, open = 0;
    for (let trial = 0; trial < 500; trial++) {
      const b: Body = trial % 4 === 0 ? disc : makePoly('p', 'p', randomConvexPolygon(r));
      const P = perimeterOf(b);
      const c1 = boundaryAt(b, r() * P), c2 = boundaryAt(b, r() * P);
      const mu = r() * 1.2;
      const two = twoFingerTest(c1, c2, mu);
      // skip razor-thin cases where tolerances legitimately disagree
      if (Math.abs(two.beta[0] - two.alpha) < 1e-4 || Math.abs(two.beta[1] - two.alpha) < 1e-4) continue;
      const Fs = edgeWrenches([c1, c2], mu);
      const comb = isForceClosure(Fs);
      const cert = closureCertificate(Fs) !== null;
      const margin = originMargin(scaleMoments(Fs, bodyRadius(b)));
      if (Math.abs(margin) < 1e-6 && two.ok) continue;
      expect(comb).toBe(two.ok);
      expect(cert).toBe(two.ok);
      expect(margin > 1e-9).toBe(two.ok);
      if (two.ok) closed++; else open++;
    }
    expect(closed).toBeGreaterThan(20);
    expect(open).toBeGreaterThan(20);
  });
  it('disc: two fingers close exactly when Δφ > 180° − 2 tan⁻¹ μ', () => {
    const mu = 0.2;
    const lim = 180 - (2 * frictionAngle(mu) * 180) / Math.PI; // ≈ 157.4°
    for (const dphi of [90, 150, lim - 1, lim + 1, 170, 180]) {
      const c1 = onDisc(200), c2 = onDisc(200 + dphi);
      const ok = dphi > lim;
      expect(twoFingerTest(c1, c2, mu).ok).toBe(ok);
      expect(isForceClosure(edgeWrenches([c1, c2], mu))).toBe(ok);
    }
  });
  it('parallel faces: the allowed tilt of the finger line is tan⁻¹ μ', () => {
    const mu = 0.3;
    const tilt = (dy: number) => Math.atan2(dy, 1.5);
    const close = (dy: number) => twoFingerTest(at(square, [-0.75, -dy / 2]), at(square, [0.75, dy / 2]), mu).ok;
    expect(close(1.5 * Math.tan(frictionAngle(mu)) - 0.01)).toBe(true);
    expect(close(1.5 * Math.tan(frictionAngle(mu)) + 0.01)).toBe(false);
    expect(tilt(1.5 * mu)).toBeCloseTo(frictionAngle(mu), 12);
  });
  it('three fingers at the edge midpoints of a triangle: closure with friction, none at μ = 0', () => {
    const mids = [0, 1, 2].map((k) => {
      const a = tri.verts[k], b = tri.verts[(k + 1) % 3];
      return at(tri, [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2]);
    });
    expect(isForceClosure(edgeWrenches(mids, 0.3))).toBe(true);
    expect(closureCertificate(edgeWrenches(mids, 0.3))).not.toBeNull();
    const F0 = edgeWrenches(mids, 0);
    expect(wrenchRank(F0)).toBe(2); // concurrent normals: no moment about the centroid
    expect(isForceClosure(F0)).toBe(false);
    const V = representativeFreeTwist(F0)!;
    expect(Math.abs(V[0])).toBeCloseTo(1, 9); // the escape is the rotation about the centroid
    expect(Math.hypot(V[1], V[2])).toBeLessThan(1e-9);
  });
  it('one finger is never enough and μ = 0 reduces to the frictionless (form-closure) test', () => {
    expect(isForceClosure(edgeWrenches([at(square, [0, -0.75])], 1))).toBe(false);
    // moments about the centroid alternate in sign (−, +, −, +) while the forces point up, left, down, right
    const pin = [at(square, [-0.43, -0.75]), at(square, [0.75, 0.43]), at(square, [0.43, 0.75]), at(square, [-0.75, -0.43])];
    expect(isForceClosure(edgeWrenches(pin, 0))).toBe(true); // frictionless form closure
    // all four moments of one sign: a couple of the other sign cannot be balanced
    const same = [at(square, [-0.43, -0.75]), at(square, [0.75, -0.43]), at(square, [0.43, 0.75]), at(square, [-0.75, 0.43])];
    expect(isForceClosure(edgeWrenches(same, 0))).toBe(false);
    const mids = [at(square, [0, -0.75]), at(square, [0.75, 0]), at(square, [0, 0.75]), at(square, [-0.75, 0])];
    expect(isForceClosure(edgeWrenches(mids, 0))).toBe(false);
    expect(isForceClosure(edgeWrenches(mids, 0.05))).toBe(true);
  });
  it('hull faces cover a closed hull and are ordered counter-clockwise about their normals', () => {
    const cube: V3[] = [];
    for (const x of [-1, 1]) for (const y of [-1, 1]) for (const z of [-1, 1]) cube.push([x, y, z]);
    const faces = hullFaces(cube);
    expect(faces.length).toBe(6);
    for (const f of faces) {
      expect(f.loop.length).toBe(4);
      const [a, b, c] = f.loop.map((i) => cube[i]);
      const n = [(b[1] - a[1]) * (c[2] - a[2]) - (b[2] - a[2]) * (c[1] - a[1]), (b[2] - a[2]) * (c[0] - a[0]) - (b[0] - a[0]) * (c[2] - a[2]), (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])];
      expect(dot3(n, f.normal)).toBeGreaterThan(0);
    }
    // a flat set gives one face, and its supporting planes pass through the origin (margin 0)
    const flat: V3[] = [[1, 0, 0], [-0.5, 0.8, 0], [-0.5, -0.8, 0]];
    expect(hullFaces(flat).length).toBe(1);
    expect(hullFacets(flat).every((f) => Math.abs(f.offset) < 1e-12)).toBe(true);
    expect(originMargin(flat)).toBeCloseTo(0, 12);
  });
});

describe('grip forces', () => {
  const pinch = [at(square, [-0.75, 0]), at(square, [0.75, 0])];
  it('horizontal pinch holding a weight needs W / (2μ) per finger', () => {
    for (const mu of [0.25, 0.4, 0.8]) {
      const W = 6;
      const res = minGripForces(pinch, mu, [0, 0, -W]);
      expect(res.feasible).toBe(true);
      expect(res.fStar).toBeCloseTo(W / (2 * mu), 7);
      // forces lie in the cones and balance the weight
      let sum: V3 = [0, 0, -W];
      res.f.forEach((f, i) => {
        expect(coneCheck(pinch[i].n, mu, f, 1e-7).inside).toBe(true);
        const F = forceWrench(pinch[i].p, f);
        sum = [sum[0] + F[0], sum[1] + F[1], sum[2] + F[2]];
      });
      for (let k = 0; k < 3; k++) expect(sum[k]).toBeCloseTo(0, 7);
    }
    expect(minGripForces(pinch, 0, [0, 0, -6]).feasible).toBe(false);
  });
  it('is positively homogeneous and feasible for every disturbance under force closure', () => {
    const r = rng(4);
    const grasp = [at(square, [-0.75, -0.2]), at(square, [0.75, 0.25])];
    const mu = 0.5;
    expect(twoFingerTest(grasp[0], grasp[1], mu).ok).toBe(true);
    for (let trial = 0; trial < 40; trial++) {
      const F: V3 = [r() * 4 - 2, r() * 10 - 5, r() * 10 - 5];
      const a = minGripForces(grasp, mu, F);
      const b = minGripForces(grasp, mu, [3 * F[0], 3 * F[1], 3 * F[2]]);
      expect(a.feasible && b.feasible).toBe(true);
      expect(b.fStar).toBeCloseTo(3 * a.fStar, 6);
    }
  });
  it('acceleration direction of a central force is a pure translation', () => {
    const V = accelTwist([0, 0, -6], gyrationRadius(square));
    expect(V[0]).toBe(0);
    expect(V[2]).toBeLessThan(0);
  });
});

describe('duality', () => {
  const offset = [at(square, [-0.75, -0.45]), at(square, [0.75, 0.45])];
  it('the offset pinch with small μ leaves a CCW rotation free; it disappears for μ > 0.6', () => {
    const Fs = edgeWrenches(offset, 0.25);
    expect(isForceClosure(Fs)).toBe(false);
    const V = preferredFreeTwist(Fs)!;
    expect(V[0]).toBeGreaterThan(0); // no free translation, so the example is a CCW rotation
    expect(isFeasibleTwist(Fs, V)).toBe(true);
    expect(isForceClosure(edgeWrenches(offset, 0.62))).toBe(true);
    expect(isForceClosure(edgeWrenches(offset, 0.58))).toBe(false);
  });
  it('points of the + region of the cone-edge CoR map are free CCW rotation centres', () => {
    const lines = edgeLines(offset, 0.25);
    const box: P2[] = [[-3, -3], [3, -3], [3, 3], [-3, 3]];
    const reg = corRegion(lines, 1, box);
    expect(reg.length).toBeGreaterThanOrEqual(3);
    const Fs = edgeWrenches(offset, 0.25);
    for (const q of reg) {
      const c: P2 = [0.9 * q[0], 0.9 * q[1]]; // pull vertices slightly inwards
      expect(isFeasibleTwist(Fs, twistAbout(c, 1), 1e-7)).toBe(true);
    }
    expect(corRegion(lines, -1, box).length).toBe(0);
  });
  it('an unresisted disturbance drives a free twist; a resisted one drives none (Farkas)', () => {
    const r = rng(17);
    for (let trial = 0; trial < 200; trial++) {
      const b = makePoly('p', 'p', randomConvexPolygon(r));
      const P = perimeterOf(b);
      const cs = Array.from({ length: 1 + Math.floor(r() * 3) }, () => boundaryAt(b, r() * P));
      const mu = r() * 0.8;
      const Fs = edgeWrenches(cs, mu);
      const Fext: V3 = [r() * 2 - 1, r() * 2 - 1, r() * 2 - 1];
      const hold = minGripForces(cs, mu, Fext);
      const V = drivenFreeTwist(Fs, Fext, bodyRadius(b));
      if (hold.feasible) expect(V).toBeNull();
      else {
        if (!V) continue; // numerically borderline
        expect(isFeasibleTwist(Fs, V, 1e-7)).toBe(true);
        expect(dot3(Fext, V)).toBeGreaterThan(0);
      }
    }
  });
  it('pushAlong produces a force that does positive power on the free twist and starts inside the body', () => {
    const Fs = edgeWrenches(offset, 0.25);
    for (const V of freeTwistRays(Fs)) {
      const { a, f } = pushAlong(square, V);
      expect(insideBody(square, a)).toBe(true);
      expect(dot3(forceWrench(a, f), V)).toBeGreaterThan(0);
    }
  });
});
