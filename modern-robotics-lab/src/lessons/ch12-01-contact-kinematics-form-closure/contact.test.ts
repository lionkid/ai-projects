import { describe, expect, it } from 'vitest';
import {
  type P2, type V3, makeShape, perimeter, boundaryAt, projectToBoundary, contactWrench, twistAbout, translationTwist,
  corOf, pointVelocity, classifyContact, planarExp, applyPose, escapeTwist, isFormClosure, closureCertificate,
  originMargin, scaleMoments, shapeRadius, linprog, resistWrench, corRegion, corPointLabel, translationCone,
  autoFormClosure, representativeEscape, feasibleCorPoints, dot3, wrenchRank, insidePolygon, polygonAreaAbs,
} from './contact';

const square = makeShape('square', 'square', [[-0.75, -0.75], [0.75, -0.75], [0.75, 0.75], [-0.75, 0.75]]);

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

function contactsAt(verts: P2[], ss: number[]) {
  return ss.map((s) => boundaryAt(verts, s));
}

describe('boundary parameterisation', () => {
  it('square perimeter, inward normals and projection round trip', () => {
    expect(perimeter(square.verts)).toBeCloseTo(6, 12);
    for (let s = 0.1; s < 6; s += 0.37) {
      const b = boundaryAt(square.verts, s);
      // inward normal points towards the centroid for a convex body
      expect(b.n[0] * -b.p[0] + b.n[1] * -b.p[1]).toBeGreaterThan(0);
      const q: P2 = [b.p[0] - 0.2 * b.n[0], b.p[1] - 0.2 * b.n[1]]; // a point just outside
      const b2 = projectToBoundary(square.verts, q);
      expect(Math.hypot(b2.p[0] - b.p[0], b2.p[1] - b.p[1])).toBeLessThan(1e-12);
    }
  });
  it('keeps contacts away from the corners', () => {
    const b = boundaryAt(square.verts, 0); // exactly at a vertex
    expect(Math.hypot(b.p[0] + 0.75, b.p[1] + 0.75)).toBeGreaterThan(0.05);
  });
  it('makeShape recentres the area centroid and orders vertices CCW', () => {
    const L = makeShape('L', 'L', [[0, 0], [0, 1.3], [0.55, 1.3], [0.55, 0.55], [1.5, 0.55], [1.5, 0]]); // given CW
    let cx = 0, cy = 0, a = 0;
    const v = L.verts;
    for (let i = 0; i < v.length; i++) {
      const p = v[i], q = v[(i + 1) % v.length];
      const w = p[0] * q[1] - q[0] * p[1];
      a += w; cx += (p[0] + q[0]) * w; cy += (p[1] + q[1]) * w;
    }
    expect(a).toBeGreaterThan(0);
    expect(cx / (3 * a)).toBeCloseTo(0, 12);
    expect(cy / (3 * a)).toBeCloseTo(0, 12);
    expect(insidePolygon([0, 0], v)).toBe(true);
  });
});

describe('first-order contact analysis', () => {
  const p: P2 = [0.1, -0.75];
  const n: P2 = [0, 1];
  it('Fᵀ V equals the normal velocity of the contact point', () => {
    const r = rng(1);
    for (let k = 0; k < 50; k++) {
      const q: P2 = [r() * 2 - 1, r() * 2 - 1];
      const a = r() * 2 * Math.PI;
      const nn: P2 = [Math.cos(a), Math.sin(a)];
      const V: V3 = [r() * 2 - 1, r() * 2 - 1, r() * 2 - 1];
      const v = pointVelocity(V, q);
      expect(dot3(contactWrench(q, nn), V)).toBeCloseTo(nn[0] * v[0] + nn[1] * v[1], 12);
    }
  });
  it('classifies breaking / sliding / penetrating / rolling', () => {
    expect(classifyContact(p, n, translationTwist([0, 1])).mode).toBe('breaking');
    expect(classifyContact(p, n, translationTwist([1, 0])).mode).toBe('sliding');
    expect(classifyContact(p, n, translationTwist([0.3, -1])).mode).toBe('penetrating');
    expect(classifyContact(p, n, twistAbout(p, 1)).mode).toBe('rolling');
    expect(classifyContact(p, n, twistAbout([0.1, 0.4], -1)).mode).toBe('sliding'); // CoR on the normal line
  });
  it('uses the relative velocity when the finger moves', () => {
    const V = translationTwist([0, 1]);
    expect(classifyContact(p, n, V, [0, 1]).mode).toBe('rolling');
    expect(classifyContact(p, n, V, [0.5, 1]).mode).toBe('sliding');
    expect(classifyContact(p, n, V, [0, 2]).mode).toBe('penetrating');
    expect(classifyContact(p, n, V, [0, 0.4]).mode).toBe('breaking');
  });
  it('twistAbout / corOf round trip and the CoR does not move', () => {
    const c: P2 = [0.3, -1.2];
    const V = twistAbout(c, -2);
    const c2 = corOf(V)!;
    expect(c2[0]).toBeCloseTo(c[0], 12);
    expect(c2[1]).toBeCloseTo(c[1], 12);
    const v = pointVelocity(V, c);
    expect(Math.hypot(v[0], v[1])).toBeLessThan(1e-12);
    expect(corOf(translationTwist([1, 2]))).toBeNull();
  });
  it('planarExp rotates about the CoR and translates for ω = 0', () => {
    const c: P2 = [0.4, 0.2];
    const pose = planarExp(twistAbout(c, 1), 0.7);
    expect(pose.theta).toBeCloseTo(0.7, 12);
    const q = applyPose(pose, c);
    expect(q[0]).toBeCloseTo(c[0], 12);
    expect(q[1]).toBeCloseTo(c[1], 12);
    const t = planarExp(translationTwist([1, -2]), 0.5);
    expect(t.x).toBeCloseTo(0.5, 12);
    expect(t.y).toBeCloseTo(-1, 12);
    expect(t.theta).toBeCloseTo(0, 12);
  });
});

describe('CoR regions', () => {
  it('one contact: left of the normal is +, right is −, the line is ±', () => {
    const line = { p: [0, -0.75] as P2, d: [0, 1] as P2 };
    expect(corPointLabel([line], [-1, 0])).toEqual({ plus: true, minus: false });
    expect(corPointLabel([line], [1, 0])).toEqual({ plus: false, minus: true });
    expect(corPointLabel([line], [0, 0.4])).toEqual({ plus: true, minus: true });
    const box: P2[] = [[-2, -2], [2, -2], [2, 2], [-2, 2]];
    expect(polygonAreaAbs(corRegion([line], 1, box))).toBeCloseTo(8, 9);
    expect(polygonAreaAbs(corRegion([line], -1, box))).toBeCloseTo(8, 9);
  });
  it('points of the shaded regions never give a penetrating rotation', () => {
    const r = rng(7);
    const box: P2[] = [[-3, -3], [3, -3], [3, 3], [-3, 3]];
    for (let trial = 0; trial < 20; trial++) {
      const cs = contactsAt(square.verts, [r() * 6, r() * 6]);
      const lines = cs.map((c) => ({ p: c.p, d: c.n }));
      for (const sense of [1, -1] as const) {
        const reg = corRegion(lines, sense, box);
        if (reg.length < 3) continue;
        const cx = reg.reduce((s, q) => s + q[0], 0) / reg.length;
        const cy = reg.reduce((s, q) => s + q[1], 0) / reg.length;
        const V = twistAbout([cx, cy], sense);
        for (const c of cs) expect(classifyContact(c.p, c.n, V).mode).not.toBe('penetrating');
      }
    }
  });
  it('concurrent normals: the common point is a ± centre', () => {
    const ss = [0.75, 2.25, 3.75, 5.25]; // midpoints of the four edges
    const lines = contactsAt(square.verts, ss).map((c) => ({ p: c.p, d: c.n }));
    const pts = feasibleCorPoints(lines);
    expect(pts.length).toBe(1);
    expect(Math.hypot(pts[0].c[0], pts[0].c[1])).toBeLessThan(1e-9);
    expect(pts[0].plus && pts[0].minus).toBe(true);
  });
});

describe('translation cone', () => {
  it('handles half-plane, wedge, line and empty cases', () => {
    const half = translationCone([[0, 1]]);
    expect(half.kind).toBe('arcs');
    if (half.kind === 'arcs') { expect(half.arcs.length).toBe(1); expect(half.arcs[0][1] - half.arcs[0][0]).toBeCloseTo(Math.PI, 9); }
    const wedge = translationCone([[0, 1], [1, 0]]);
    if (wedge.kind !== 'arcs') throw new Error('expected arcs');
    expect(wedge.arcs.length).toBe(1);
    expect(wedge.arcs[0][0]).toBeCloseTo(0, 9);
    expect(wedge.arcs[0][1]).toBeCloseTo(Math.PI / 2, 9);
    const line = translationCone([[0, 1], [0, -1]]);
    if (line.kind !== 'arcs') throw new Error('expected arcs');
    expect(line.arcs.length).toBe(0);
    expect(line.rays.length).toBe(2);
    const c = Math.cos(2 * Math.PI / 3), s = Math.sin(2 * Math.PI / 3);
    expect(translationCone([[1, 0], [c, s], [c, -s]]).kind).toBe('none');
    expect(translationCone([]).kind).toBe('all');
  });
});

describe('LP solver', () => {
  it('solves small textbook problems', () => {
    const a = linprog([1, 1], [[1, 2]], [4]);
    expect(a.status).toBe('optimal');
    expect(a.x[0]).toBeCloseTo(0, 9);
    expect(a.x[1]).toBeCloseTo(2, 9);
    expect(linprog([1, 1], [[1, 1]], [-1]).status).toBe('infeasible');
    expect(linprog([-1, 0], [[1, -1]], [0]).status).toBe('unbounded');
    const red = linprog([1, 0], [[1, 1], [2, 2]], [1, 2]); // redundant equality
    expect(red.status).toBe('optimal');
    expect(red.x[0]).toBeCloseTo(0, 9);
    expect(red.x[1]).toBeCloseTo(1, 9);
  });
  it('matches brute-force vertex enumeration on random 2×4 problems', () => {
    const r = rng(11);
    for (let trial = 0; trial < 200; trial++) {
      const A = [[r() * 2 - 1, r() * 2 - 1, r() * 2 - 1, r() * 2 - 1], [r() * 2 - 1, r() * 2 - 1, r() * 2 - 1, r() * 2 - 1]];
      const b = [r() * 2 - 1, r() * 2 - 1];
      const c = [r(), r(), r(), r()]; // c ≥ 0 keeps the problem bounded
      let best = Infinity;
      for (let i = 0; i < 4; i++) for (let j = i + 1; j < 4; j++) {
        const det = A[0][i] * A[1][j] - A[0][j] * A[1][i];
        if (Math.abs(det) < 1e-12) continue;
        const xi = (b[0] * A[1][j] - A[0][j] * b[1]) / det;
        const xj = (A[0][i] * b[1] - b[0] * A[1][i]) / det;
        if (xi < -1e-12 || xj < -1e-12) continue;
        best = Math.min(best, c[i] * xi + c[j] * xj);
      }
      const res = linprog(c, A, b);
      if (best === Infinity) expect(res.status).toBe('infeasible');
      else { expect(res.status).toBe('optimal'); expect(res.value).toBeCloseTo(best, 7); }
    }
  });
});

describe('form closure', () => {
  it('auto search closes the square, the L and the triangle with 4 contacts', () => {
    const shapes = [
      square,
      makeShape('L', 'L', [[0, 0], [1.5, 0], [1.5, 0.55], [0.55, 0.55], [0.55, 1.3], [0, 1.3]]),
      makeShape('tri', 'tri', [[-0.85, -0.49], [0.85, -0.49], [0, 0.98]]),
    ];
    for (const sh of shapes) {
      const res = autoFormClosure(sh.verts);
      expect(res).not.toBeNull();
      const Fs = contactsAt(sh.verts, res!.s).map((c) => contactWrench(c.p, c.n));
      expect(isFormClosure(Fs)).toBe(true);
      expect(closureCertificate(Fs)).not.toBeNull();
      expect(originMargin(scaleMoments(Fs, shapeRadius(sh.verts)))).toBeGreaterThan(0.05);
    }
  });
  it('three contacts are never enough', () => {
    const r = rng(3);
    for (let trial = 0; trial < 200; trial++) {
      const verts = makeShape('p', 'p', randomConvexPolygon(r)).verts;
      const P = perimeter(verts);
      const Fs = contactsAt(verts, [r() * P, r() * P, r() * P]).map((c) => contactWrench(c.p, c.n));
      const V = escapeTwist(Fs);
      expect(V).not.toBeNull();
      for (const F of Fs) expect(dot3(F, V!)).toBeGreaterThanOrEqual(-1e-9);
    }
  });
  it('concurrent normals (edge midpoints of a square) leave a rotation about the centre', () => {
    const Fs = contactsAt(square.verts, [0.75, 2.25, 3.75, 5.25]).map((c) => contactWrench(c.p, c.n));
    expect(wrenchRank(Fs)).toBe(2);
    expect(isFormClosure(Fs)).toBe(false);
    const V = representativeEscape(Fs)!;
    expect(Math.abs(V[0])).toBeCloseTo(1, 12);
    expect(Math.hypot(V[1], V[2])).toBeLessThan(1e-9);
  });
  it('combinatorial test, LP certificate and hull margin agree on random configurations', () => {
    const r = rng(5);
    let closed = 0;
    for (let trial = 0; trial < 300; trial++) {
      const verts = makeShape('p', 'p', randomConvexPolygon(r)).verts;
      const P = perimeter(verts);
      const j = 4 + Math.floor(r() * 3);
      const Fs = contactsAt(verts, Array.from({ length: j }, () => r() * P)).map((c) => contactWrench(c.p, c.n));
      const comb = isFormClosure(Fs);
      const cert = closureCertificate(Fs) !== null;
      const margin = originMargin(scaleMoments(Fs, shapeRadius(verts)));
      // skip razor-thin cases where the tolerances legitimately disagree
      if (Math.abs(margin) < 1e-6) continue;
      expect(cert).toBe(comb);
      expect(margin > 0).toBe(comb);
      if (comb) closed++;
    }
    expect(closed).toBeGreaterThan(10);
  });
  it('form closure resists any external wrench with pushing forces only', () => {
    const res = autoFormClosure(square.verts)!;
    const Fs = contactsAt(square.verts, res.s).map((c) => contactWrench(c.p, c.n));
    const r = rng(9);
    for (let trial = 0; trial < 50; trial++) {
      const Fext: V3 = [r() * 4 - 2, r() * 10 - 5, r() * 10 - 5];
      const out = resistWrench(Fs, Fext);
      expect(out.ok).toBe(true);
      const sum: V3 = [0, 0, 0];
      out.k.forEach((k, i) => { expect(k).toBeGreaterThanOrEqual(0); for (let a = 0; a < 3; a++) sum[a] += k * Fs[i][a]; });
      for (let a = 0; a < 3; a++) expect(sum[a] + Fext[a]).toBeCloseTo(0, 7);
    }
  });
  it('without closure some wrench needs a pulling contact', () => {
    const Fs = contactsAt(square.verts, [0.4, 2.0, 4.3]).map((c) => contactWrench(c.p, c.n));
    const V = escapeTwist(Fs)!;
    // A wrench that does positive power on the escape twist cannot be balanced by pushes.
    const Fext: V3 = [V[0], V[1], V[2]];
    const out = resistWrench(Fs, Fext);
    expect(out.ok).toBe(false);
    expect(out.kLS.some((k) => k < -1e-9) || out.residual > 1e-9).toBe(true);
  });
});
