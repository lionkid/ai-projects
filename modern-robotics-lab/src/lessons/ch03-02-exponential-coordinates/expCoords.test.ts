import { describe, expect, it } from 'vitest';
import { MatrixExp3, MatrixLog3, RotX, RotY, VecToso3, approxEqual, det, eye, matMul, matVec, norm, so3ToVec, transpose, vecScale } from '@/mr';
import {
  anglesFromAxis, axisFromAngles, eulerPath, expPath, fixedAxisMotion, foldedAngle, helixPoint, integratedOmega, logInfo,
  omegaFromDerivative, orbitPoints, rodriguesTerms, rotAxisAngle, rotatePointTerms, topMotion,
} from './expCoords';

const w = axisFromAngles(0.6, 0.85);

describe('axis parametrisation', () => {
  it('is a unit vector and round-trips', () => {
    expect(norm(w)).toBeCloseTo(1, 12);
    const { az, el } = anglesFromAxis(w);
    expect(az).toBeCloseTo(0.6, 12);
    expect(el).toBeCloseTo(0.85, 12);
  });
});

describe('Rodrigues terms', () => {
  it('sum to the library matrix exponential', () => {
    for (const th of [0, 1e-6, 0.5, 2, Math.PI, 5.3]) {
      const { R } = rodriguesTerms(w, th);
      expect(approxEqual(R, MatrixExp3(VecToso3(vecScale(w, th))), 1e-12)).toBe(true);
    }
  });

  it('dropping the (1 − cos θ) term breaks orthogonality: det = 1 + sin²θ', () => {
    const th = 1.1;
    const { R } = rodriguesTerms(w, th, { sin: true, cos: false });
    expect(det(R)).toBeCloseTo(1 + Math.sin(th) ** 2, 10);
    expect(approxEqual(matMul(transpose(R), R), eye(3), 1e-3)).toBe(false);
  });

  it('applied to a point: ends on the circle around the axis', () => {
    const p = [1, 0, 0];
    const th = 2.1;
    const r = rotatePointTerms(w, th, p);
    expect(approxEqual(r.end, matVec(rotAxisAngle(w, th), p), 1e-12)).toBe(true);
    expect(norm([r.end[0] - r.center[0], r.end[1] - r.center[1], r.end[2] - r.center[2]])).toBeCloseTo(r.radius, 12);
    // without the pull-back term the point leaves the circle along the tangent
    const r2 = rotatePointTerms(w, th, p, { sin: true, cos: false });
    const d = norm([r2.end[0] - r2.center[0], r2.end[1] - r2.center[1], r2.end[2] - r2.center[2]]);
    expect(d).toBeGreaterThan(r.radius + 1e-3);
  });

  it('orbit points stay at constant distance from the axis', () => {
    const pts = orbitPoints(w, [0, 1, 0.5], 0, 4, 30);
    const r0 = norm([pts[0][0], pts[0][1], pts[0][2]]);
    for (const q of pts) expect(norm(q)).toBeCloseTo(r0, 12);
  });
});

describe('matrix logarithm branches', () => {
  it('identity: θ = 0, no axis', () => {
    const info = logInfo(eye(3));
    expect(info.branch).toBe('identity');
    expect(info.omghat).toBeNull();
    expect(info.theta).toBe(0);
  });

  it('general and small angles use the antisymmetric formula', () => {
    for (const th of [1e-3, 0.4, 2.5]) {
      const info = logInfo(rotAxisAngle(w, th));
      expect(info.branch).toBe('general');
      expect(info.theta).toBeCloseTo(th, 9);
      expect(approxEqual(info.omghat!, w, 1e-6)).toBe(true);
    }
  });

  it('near π keeps the sign, exactly π picks a convention', () => {
    const near = logInfo(rotAxisAngle(w, Math.PI - 1e-6));
    expect(near.branch).toBe('nearPi');
    expect(approxEqual(near.omghat!, w, 1e-6)).toBe(true);
    const at = logInfo(rotAxisAngle(w, Math.PI));
    expect(at.branch).toBe('pi');
    expect(at.theta).toBeCloseTo(Math.PI, 12);
    // both signs reproduce the same R
    const Rp = MatrixExp3(VecToso3(vecScale(at.omghat!, Math.PI)));
    const Rm = MatrixExp3(VecToso3(vecScale(at.omghat!, -Math.PI)));
    expect(approxEqual(Rp, Rm, 1e-12)).toBe(true);
  });

  it('folded angle predicts what the log returns for θ_in ∈ [0, 2π)', () => {
    for (let k = 1; k < 72; k++) {
      const th = (k * 2 * Math.PI) / 72;
      if (Math.abs(th - Math.PI) < 1e-9) continue;
      const f = foldedAngle(th);
      const L = so3ToVec(MatrixLog3(rotAxisAngle(w, th)));
      expect(norm(L)).toBeCloseTo(f.theta, 9);
      expect(approxEqual(vecScale(L, 1 / norm(L)), f.flipped ? vecScale(w, -1) : w, 1e-6)).toBe(true);
    }
    expect(foldedAngle(1.5 * Math.PI)).toEqual({ theta: 0.5 * Math.PI, flipped: true });
  });
});

describe('angular velocity', () => {
  it('fixed axis: ω_s = ω̂ rate, ω_b = R0ᵀ ω_s, both constant', () => {
    const R0 = matMul(RotX(-0.6), RotY(0.4));
    for (const t of [0, 0.7, 2]) {
      const m = fixedAxisMotion(w, 1.3, R0, t);
      expect(approxEqual(m.wS, vecScale(w, 1.3), 1e-12)).toBe(true);
      expect(approxEqual(m.wB, matVec(transpose(R0), vecScale(w, 1.3)), 1e-12)).toBe(true);
      const fd = omegaFromDerivative((s) => fixedAxisMotion(w, 1.3, R0, s).R, t);
      expect(approxEqual(fd.wS, m.wS, 1e-6)).toBe(true);
      expect(approxEqual(fd.wB, m.wB, 1e-6)).toBe(true);
    }
  });

  it('spinning top: finite differences match the analytic ω_s, ω_b and ω_s = R ω_b', () => {
    const P = { precession: 0.8, spin: 2.5, tilt: 0.5 };
    for (const t of [0, 0.4, 1.9]) {
      const m = topMotion(t, P);
      const fd = omegaFromDerivative((s) => topMotion(s, P).R, t);
      expect(approxEqual(fd.wS, m.wS, 1e-6)).toBe(true);
      expect(approxEqual(fd.wB, m.wB, 1e-6)).toBe(true);
      expect(approxEqual(matVec(m.R, m.wB), m.wS, 1e-12)).toBe(true);
      // Ṙ Rᵀ is skew-symmetric (up to finite-difference error)
      expect(approxEqual(fd.WS, transpose(fd.WS).map((r) => r.map((x) => -x)), 1e-6)).toBe(true);
    }
    // ω_s changes direction for the top (unlike the fixed-axis motion)
    expect(approxEqual(topMotion(0, P).wS, topMotion(1, P).wS, 1e-3)).toBe(false);
  });
});

describe('exp path vs Euler path', () => {
  const R = matMul(matMul(RotX(0.7), RotY(-1.1)), MatrixExp3(VecToso3([0.2, 0.5, 0.9])));

  it('both paths end at R, from I', () => {
    expect(approxEqual(expPath(R, 0), eye(3), 1e-12)).toBe(true);
    expect(approxEqual(eulerPath(R, 0), eye(3), 1e-12)).toBe(true);
    expect(approxEqual(expPath(R, 1), R, 1e-9)).toBe(true);
    expect(approxEqual(eulerPath(R, 1), R, 1e-9)).toBe(true);
    expect(approxEqual(expPath(R, 0.5), eulerPath(R, 0.5), 1e-3)).toBe(false);
  });

  it('∫ω_s dt equals log R only along the single-axis path', () => {
    const L = so3ToVec(MatrixLog3(R));
    expect(approxEqual(integratedOmega(R, 'exp'), L, 1e-12)).toBe(true);
    expect(approxEqual(integratedOmega(R, 'euler'), L, 1e-2)).toBe(false);
  });

  it('Euler-path integral agrees with a numerical integration of ω_s', () => {
    const n = 3000;
    let acc = [0, 0, 0];
    for (let i = 0; i < n; i++) {
      const u = (i + 0.5) / n;
      const fd = omegaFromDerivative((s) => eulerPath(R, s), u, 1e-6);
      acc = acc.map((a, k) => a + fd.wS[k] / n);
    }
    expect(approxEqual(acc, integratedOmega(R, 'euler'), 1e-3)).toBe(true);
  });

  it('quiz: ω_s = x̂ for 1 s then ŷ for 1 s is not log of the final R', () => {
    const Rf = matMul(rotAxisAngle([0, 1, 0], 1), rotAxisAngle([1, 0, 0], 1));
    const L = so3ToVec(MatrixLog3(Rf));
    expect(L[0]).toBeCloseTo(0.913, 3);
    expect(L[1]).toBeCloseTo(0.913, 3);
    expect(L[2]).toBeCloseTo(-0.499, 3);
  });
});

describe('helix teaser', () => {
  it('advances pitch·θ along the axis', () => {
    const p = [1, 0, 0];
    const q = helixPoint(w, 0.2, 2, p);
    const alongStart = p[0] * w[0] + p[1] * w[1] + p[2] * w[2];
    const along = q[0] * w[0] + q[1] * w[1] + q[2] * w[2];
    expect(along - alongStart).toBeCloseTo(0.4, 12);
  });
});
