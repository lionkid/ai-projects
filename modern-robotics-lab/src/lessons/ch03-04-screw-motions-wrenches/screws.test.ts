import { describe, expect, it } from 'vitest';
import {
  Adjoint, MatrixExp6, RotX, RotZ, RpToTrans, TestIfSE3, TransInv, VecTose3, approxEqual, cross, dot, eye, matMul, matScale, matVec,
  norm, rotZYX, transformPoint, vecAdd, vecScale, vecSub,
} from '@/mr';
import {
  dirFromAngles, interpolatePose, interpolationPath, power, prismaticScrew, rotationBlockDet, screwBetween, screwFromQSH,
  screwMotion, screwPath, translationTerms, vParts, wrenchAtPoint, wrenchInFrame,
} from './screws';

const DEG = Math.PI / 180;
const pose = (x: number, y: number, z: number, a: number, b: number, c: number) => RpToTrans(rotZYX([a * DEG, b * DEG, c * DEG]), [x, y, z]);

/** Distance from point p to the line through q with unit direction s. */
const distToLine = (p: number[], q: number[], s: number[]) => norm(cross(vecSub(p, q), s));

describe('screw axes', () => {
  it('builds S = (ŝ, −ŝ×q + hŝ) with a unit ω and ωᵀv = h', () => {
    const q = [0.6, 0.2, 0], s = dirFromAngles(30 * DEG, 60 * DEG), h = 0.12;
    const S = screwFromQSH(q, vecScale(s, 3), h); // direction is normalised
    expect(norm(S.slice(0, 3))).toBeCloseTo(1, 12);
    expect(dot(S.slice(0, 3), S.slice(3, 6))).toBeCloseTo(h, 12);
    const { rot, slide } = vParts(q, s, h);
    expect(approxEqual(vecAdd(rot, slide), S.slice(3, 6), 1e-12)).toBe(true);
  });

  it('a prismatic screw has ω = 0 and a unit v', () => {
    const S = prismaticScrew([0, 3, 4]);
    expect(approxEqual(S, [0, 0, 0, 0, 0.6, 0.8], 1e-15)).toBe(true);
  });
});

describe('exp([S]θ)', () => {
  const q = [0.6, 0.2, 0], s = [0, 0, 1];

  it('the three translation terms add up to the translation of MatrixExp6', () => {
    for (const [S, th] of [[screwFromQSH(q, s, 0.1), 2.3], [screwFromQSH([0.1, -0.4, 0.3], dirFromAngles(1, 0.4), -0.2), 5.1], [prismaticScrew([1, 2, 2]), 0.7]] as [number[], number][]) {
      const T = MatrixExp6(matScale(VecTose3(S), th));
      const [t1, t2, t3] = translationTerms(S, th);
      expect(approxEqual(vecAdd(vecAdd(t1, t2), t3), [T[0][3], T[1][3], T[2][3]], 1e-12)).toBe(true);
    }
  });

  it('every body point moves on a helix: constant distance to the axis, rise h per radian', () => {
    const h = 0.1;
    const S = screwFromQSH(q, s, h);
    const T0 = pose(0.1, -0.2, 0.3, 20, 10, 0);
    const rb = [0.2, 0.1, 0.05];
    const pts = screwPath(S, 2 * Math.PI, T0, rb, 24);
    const d0 = distToLine(pts[0], q, s);
    pts.forEach((p, k) => {
      expect(distToLine(p, q, s)).toBeCloseTo(d0, 10);
      expect(dot(vecSub(p, pts[0]), s)).toBeCloseTo((h * 2 * Math.PI * k) / 24, 10);
    });
  });

  it('a zero-pitch screw returns to the start after one turn', () => {
    const T0 = pose(0.3, 0.1, 0.2, 40, 0, 0);
    expect(approxEqual(screwMotion(screwFromQSH(q, s, 0), 2 * Math.PI, T0), T0, 1e-12)).toBe(true);
  });
});

describe('log: the screw between two poses', () => {
  const T1 = pose(-0.1, -0.4, 0.3, -20, 0, 0);
  const T2 = pose(0.6, 0.45, 0.55, 80, 0, 35);

  it('exp([S]θ) T1 reproduces T2 with θ in [0, π]', () => {
    const info = screwBetween(T1, T2);
    expect(info.kind).toBe('screw');
    expect(info.theta).toBeGreaterThan(0);
    expect(info.theta).toBeLessThanOrEqual(Math.PI);
    expect(approxEqual(screwMotion(info.S, info.theta, T1), T2, 1e-10)).toBe(true);
  });

  it('the screw in {b} (log of T1⁻¹T2) is the same axis seen from {b}', () => {
    const info = screwBetween(T1, T2);
    const infoB = screwBetween(eye(4), matMul(TransInv(T1), T2));
    expect(infoB.theta).toBeCloseTo(info.theta, 10);
    expect(approxEqual(matVec(Adjoint(T1), infoB.S), info.S, 1e-9)).toBe(true);
  });

  it('handles pure translations and the identity', () => {
    const Tt = matMul(RpToTrans(eye(3), [0.8, 0.6, 0]), T1);
    const tr = screwBetween(T1, Tt);
    expect(tr.kind).toBe('translation');
    expect(tr.theta).toBeCloseTo(1, 12);
    expect(approxEqual(tr.S, [0, 0, 0, 0.8, 0.6, 0], 1e-12)).toBe(true);
    expect(screwBetween(T1, T1).kind).toBe('identity');
  });

  it('a half turn (θ = π) still connects the poses', () => {
    const S = screwFromQSH([0.3, 0, 0.4], [1, 0, 0], 0.1);
    const T2pi = screwMotion(S, Math.PI, T1);
    const info = screwBetween(T1, T2pi);
    expect(info.theta).toBeCloseTo(Math.PI, 8);
    expect(approxEqual(screwMotion(info.S, info.theta, T1), T2pi, 1e-8)).toBe(true);
  });
});

describe('interpolation methods', () => {
  const T1 = eye(4);
  const T2 = RpToTrans(matMul(RotZ(120 * DEG), RotX(30 * DEG)), [0.7, 0.4, 0.5]);

  it('all methods hit both end poses', () => {
    for (const m of ['screw', 'decoupled', 'elementwise'] as const) {
      expect(approxEqual(interpolatePose(T1, T2, 0, m), T1, 1e-12)).toBe(true);
      expect(approxEqual(interpolatePose(T1, T2, 1, m), T2, 1e-12)).toBe(true);
    }
  });

  it('screw and decoupled stay rigid; element-wise interpolation shrinks the body', () => {
    expect(TestIfSE3(interpolatePose(T1, T2, 0.5, 'screw'))).toBe(true);
    expect(TestIfSE3(interpolatePose(T1, T2, 0.5, 'decoupled'))).toBe(true);
    expect(rotationBlockDet(interpolatePose(T1, T2, 0.5, 'elementwise'))).toBeLessThan(0.6);
  });

  it('decoupled moves the origin on a straight line', () => {
    const pts = interpolationPath(T1, T2, [0, 0, 0], 'decoupled', 10);
    pts.forEach((p, k) => expect(approxEqual(p, vecScale([0.7, 0.4, 0.5], k / 10), 1e-12)).toBe(true));
  });

  it('only the screw interpolation is independent of where {b} sits on the body', () => {
    const Tbc = pose(0.4, -0.2, 0.1, 15, 0, 0); // another body-fixed frame {c}
    for (const tau of [0.3, 0.7]) {
      const viaC = interpolatePose(matMul(T1, Tbc), matMul(T2, Tbc), tau, 'screw');
      expect(approxEqual(viaC, matMul(interpolatePose(T1, T2, tau, 'screw'), Tbc), 1e-10)).toBe(true);
      const viaCd = interpolatePose(matMul(T1, Tbc), matMul(T2, Tbc), tau, 'decoupled');
      expect(approxEqual(viaCd, matMul(interpolatePose(T1, T2, tau, 'decoupled'), Tbc), 1e-3)).toBe(false);
    }
  });
});

describe('wrenches', () => {
  const Tsb = pose(0.3, -0.5, 0.45, 35, 0, 0);
  const fb = [0, 5, -5];
  const rb = [0.6, 0, 0];

  it('sliding the force along its line of action leaves m unchanged', () => {
    const F1 = wrenchAtPoint(rb, fb);
    const F2 = wrenchAtPoint(vecAdd(rb, vecScale(fb, 0.37)), fb);
    expect(approxEqual(F1, F2, 1e-12)).toBe(true);
    expect(approxEqual(wrenchAtPoint(vecAdd(rb, [0.2, 0, 0]), fb), F1, 1e-6)).toBe(false);
  });

  it('F_s = [Ad_{T_bs}]ᵀ F_b matches the direct r_s × f_s', () => {
    const Fb = wrenchAtPoint(rb, fb);
    const R = Tsb.slice(0, 3).map((r) => r.slice(0, 3));
    const Fs = wrenchAtPoint(transformPoint(Tsb, rb), matVec(R, fb));
    expect(approxEqual(wrenchInFrame(TransInv(Tsb), Fb), Fs, 1e-12)).toBe(true);
    expect(approxEqual(wrenchInFrame(Tsb, Fs), Fb, 1e-12)).toBe(true); // F_b = [Ad_{T_sb}]ᵀ F_s
  });

  it('power is the same in every frame and equals f · (velocity of the application point)', () => {
    const Fb = wrenchAtPoint(rb, fb);
    const Fs = wrenchInFrame(TransInv(Tsb), Fb);
    const Vb = [0.3, -0.2, 0.5, 0.1, 0.4, -0.3];
    const Vs = matVec(Adjoint(Tsb), Vb);
    const P = power(Vb, Fb);
    expect(power(Vs, Fs)).toBeCloseTo(P, 12);
    const vPoint = vecAdd(cross(Vb.slice(0, 3), rb), Vb.slice(3, 6)); // in {b}
    expect(dot(fb, vPoint)).toBeCloseTo(P, 12);
  });
});

describe('step 6 intuition: shifting the reference point', () => {
  it('m gains p × f, v gains p × ω, and the two power terms cancel', () => {
    const w = [0.3, -0.7, 0.5], v = [0.2, 0.1, -0.4], m = [1, -2, 0.5], f = [3, 1, -2], p = [0.4, -0.9, 1.3];
    const Tsb = RpToTrans(eye(3), p); // same orientation, origin of {b} at p
    const Fs = wrenchInFrame(TransInv(Tsb), [...m, ...f]);
    const Vs = matVec(Adjoint(Tsb), [...w, ...v]);
    expect(approxEqual(Fs.slice(0, 3), vecAdd(m, cross(p, f)), 1e-12)).toBe(true);
    expect(approxEqual(Vs.slice(3, 6), vecAdd(v, cross(p, w)), 1e-12)).toBe(true);
    expect(dot(w, cross(p, f)) + dot(f, cross(p, w))).toBeCloseTo(0, 12);
  });
});

describe('quiz facts', () => {
  it('Q1: door hinge along z through (1, 0, 0), and the 2 mm-per-turn bolt', () => {
    expect(screwFromQSH([1, 0, 0], [0, 0, 1], 0)).toEqual([0, 0, 1, 0, -1, 0]);
    const h = 0.002 / (2 * Math.PI);
    const bolt = screwFromQSH([1, 0, 0], [0, 0, 1], h);
    expect(approxEqual(bolt, [0, 0, 1, 0, -1, h], 1e-15)).toBe(true);
    expect(h * 1000).toBeCloseTo(0.318, 3);
  });

  it('Q2: a pure force at the {b} origin has m_s = p × f_s', () => {
    const Tsb = pose(1, 0, 0, 30, 0, 0);
    const fb = [0, 0, -10];
    const Fs = wrenchInFrame(TransInv(Tsb), [0, 0, 0, ...fb]);
    const fs = matVec(Tsb.slice(0, 3).map((r) => r.slice(0, 3)), fb);
    expect(approxEqual(Fs.slice(0, 3), cross([1, 0, 0], fs), 1e-12)).toBe(true);
    expect(norm(Fs.slice(0, 3))).toBeGreaterThan(1);
  });

  it('Q3: log(T2 T1⁻¹) pre-multiplied works, log(T1⁻¹T2) pre-multiplied does not', () => {
    const T1 = pose(-0.1, -0.4, 0.3, -20, 0, 0), T2 = pose(0.6, 0.45, 0.55, 80, 0, 35);
    const good = screwBetween(T1, T2);
    expect(approxEqual(screwMotion(good.S, good.theta, T1), T2, 1e-10)).toBe(true);
    const bad = screwBetween(eye(4), matMul(TransInv(T1), T2));
    expect(approxEqual(screwMotion(bad.S, bad.theta, T1), T2, 1e-3)).toBe(false);
    expect(approxEqual(matMul(T1, MatrixExp6(matScale(VecTose3(bad.S), bad.theta))), T2, 1e-10)).toBe(true);
  });

  it('Q4: a force whose line of action passes through the rotation axis does no work', () => {
    expect(power([0, 0, 2, 0, 0, 0], [0, 0, 0, 0, 5, 0])).toBe(0);
  });
});
