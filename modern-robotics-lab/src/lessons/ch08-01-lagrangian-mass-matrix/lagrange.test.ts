import { describe, expect, it } from 'vitest';
import {
  type Arm, DEFAULT_ARM, chainFor, compareWithNewtonEuler, constantRate, dMdTheta2, dynamicsTerms, energies,
  energyEllipse, eulerLagrangeTerms, evaluate, foldCycle, freeJoint, holdAt, initialState, kineticEnergy,
  maxEllipseSemiAxis, potentialEnergy, sineMotion, solveHybrid, splitC, stepState, wrapAngles,
  type JointProgram, type Method,
} from './lagrange';

const ARMS: Arm[] = [
  DEFAULT_ARM,
  { L1: 0.7, L2: 1.1, m1: 2, m2: 0.5, g: 9.81 },
  { L1: 1.2, L2: 0.5, m1: 0.4, m2: 2.5, g: 0 },
];
const STATES = [
  { th: [0.3, -0.7], dth: [1.2, -0.4], ddth: [0.5, 2] },
  { th: [-1.9, 2.4], dth: [-2, 3], ddth: [-1, 0] },
  { th: [2.8, 0.1], dth: [0, 0], ddth: [0, 0] },
  { th: [0.0, Math.PI], dth: [3, -1], ddth: [2, -2] },
];

const lagrangian = (a: Arm, th: number[], dth: number[]) => kineticEnergy(a, th, dth) - potentialEnergy(a, th);

describe('closed-form Lagrangian vs. Newton–Euler from @/mr', () => {
  it('M, c, g and τ agree to rounding error', () => {
    for (const a of ARMS) {
      const chain = chainFor(a);
      for (const s of STATES) {
        const cmp = compareWithNewtonEuler(a, chain, s.th, s.dth, s.ddth);
        expect(cmp.maxDiff).toBeLessThan(1e-10);
      }
    }
  });
});

describe('Euler–Lagrange terms', () => {
  it('reproduce τ = M θ̈ + c + g', () => {
    for (const a of ARMS) {
      for (const s of STATES) {
        const el = eulerLagrangeTerms(a, s.th, s.dth, s.ddth);
        const { M, c, g } = dynamicsTerms(a, s.th, s.dth);
        for (let i = 0; i < 2; i++) {
          const ref = M[i][0] * s.ddth[0] + M[i][1] * s.ddth[1] + c[i] + g[i];
          expect(el.tau[i]).toBeCloseTo(ref, 10);
        }
      }
    }
  });

  it('match finite differences of L = K − U', () => {
    const eps = 1e-6;
    for (const a of ARMS) {
      for (const s of STATES) {
        const el = eulerLagrangeTerms(a, s.th, s.dth, s.ddth);
        for (let i = 0; i < 2; i++) {
          const dp = s.dth.slice(); dp[i] += eps;
          const dm = s.dth.slice(); dm[i] -= eps;
          const pNum = (lagrangian(a, s.th, dp) - lagrangian(a, s.th, dm)) / (2 * eps);
          expect(el.momentum[i]).toBeCloseTo(pNum, 5);
          const tp = s.th.slice(); tp[i] += eps;
          const tm = s.th.slice(); tm[i] -= eps;
          const dLNum = (lagrangian(a, tp, s.dth) - lagrangian(a, tm, s.dth)) / (2 * eps);
          expect(el.dLdTheta[i]).toBeCloseTo(dLNum, 4);
        }
        // d/dt of the momentum along the motion (θ, θ̇, θ̈)
        const at = (t: number) => {
          const th = s.th.map((q, i) => q + s.dth[i] * t + 0.5 * s.ddth[i] * t * t);
          const dth = s.dth.map((w, i) => w + s.ddth[i] * t);
          return eulerLagrangeTerms(a, th, dth, s.ddth).momentum;
        };
        const pp = at(eps), pm = at(-eps);
        for (let i = 0; i < 2; i++) expect(el.dMomentum[i]).toBeCloseTo((pp[i] - pm[i]) / (2 * eps), 4);
      }
    }
  });

  it('∂M/∂θ2 matches a finite difference of M', () => {
    const eps = 1e-6;
    for (const a of ARMS) {
      for (const s of STATES) {
        const Mp = dynamicsTerms(a, [s.th[0], s.th[1] + eps], [0, 0]).M;
        const Mm = dynamicsTerms(a, [s.th[0], s.th[1] - eps], [0, 0]).M;
        const dM = dMdTheta2(a, s.th);
        for (let i = 0; i < 2; i++) for (let j = 0; j < 2; j++) expect(dM[i][j]).toBeCloseTo((Mp[i][j] - Mm[i][j]) / (2 * eps), 5);
      }
    }
  });
});

describe('velocity-product term', () => {
  it('splits into centripetal + Coriolis, and θ̇ᵀc = ½ θ̇ᵀ Ṁ θ̇', () => {
    for (const a of ARMS) {
      for (const s of STATES) {
        const { c } = dynamicsTerms(a, s.th, s.dth);
        const { centripetal, coriolis } = splitC(a, s.th, s.dth);
        for (let i = 0; i < 2; i++) expect(centripetal[i] + coriolis[i]).toBeCloseTo(c[i], 12);
        const dM = dMdTheta2(a, s.th);
        const Md = dM.map((r) => r.map((v) => v * s.dth[1]));
        const half = 0.5 * (s.dth[0] * (Md[0][0] * s.dth[0] + Md[0][1] * s.dth[1]) + s.dth[1] * (Md[1][0] * s.dth[0] + Md[1][1] * s.dth[1]));
        expect(s.dth[0] * c[0] + s.dth[1] * c[1]).toBeCloseTo(half, 10);
      }
    }
  });
});

describe('mass-matrix ellipse', () => {
  it('is a circle of radius √2 when the default arm is fully folded (M = I) and very flat when straight', () => {
    const folded = energyEllipse(dynamicsTerms(DEFAULT_ARM, [0.4, Math.PI], [0, 0]).M, 1);
    expect(folded.lambda[0]).toBeCloseTo(1, 10);
    expect(folded.lambda[1]).toBeCloseTo(1, 10);
    expect(folded.semi[0]).toBeCloseTo(Math.SQRT2, 10);
    const straight = energyEllipse(dynamicsTerms(DEFAULT_ARM, [-1, 0], [0, 0]).M, 1);
    expect(straight.lambda[0]).toBeCloseTo(3 + 2 * Math.SQRT2, 10);
    expect(straight.lambda[1]).toBeCloseTo(3 - 2 * Math.SQRT2, 10);
    // light direction: joints counter-rotate (θ̇2 ≈ −2.41 θ̇1)
    const v = straight.vectors[1];
    expect(v[1] / v[0]).toBeCloseTo(-(1 + Math.SQRT2), 6);
    expect(maxEllipseSemiAxis(DEFAULT_ARM, 1)).toBeCloseTo(Math.sqrt(2 / (3 - 2 * Math.SQRT2)), 6);
  });

  it('points on the ellipse have exactly the requested kinetic energy', () => {
    for (const a of ARMS) {
      const th = [0.2, 1.1];
      const M = dynamicsTerms(a, th, [0, 0]).M;
      const e = energyEllipse(M, 2.5);
      expect(e.lambda[1]).toBeGreaterThan(0);
      for (const phi of [0, 0.7, 2.1, 4]) {
        const d = [0, 1].map((k) => e.semi[0] * Math.cos(phi) * e.vectors[0][k] + e.semi[1] * Math.sin(phi) * e.vectors[1][k]);
        expect(kineticEnergy(a, th, d)).toBeCloseTo(2.5, 9);
      }
    }
  });
});

describe('energies', () => {
  it('U is zero hanging straight down and never negative', () => {
    for (const a of ARMS) {
      expect(potentialEnergy(a, [-Math.PI / 2, 0])).toBeCloseTo(0, 12);
      for (let k = 0; k < 50; k++) {
        const th = [Math.sin(k) * 3, Math.cos(1.7 * k) * 3];
        expect(potentialEnergy(a, th)).toBeGreaterThanOrEqual(-1e-12);
      }
    }
  });
});

function run(a: Arm, x0: number[], programs: [JointProgram, JointProgram], T: number, h: number, method: Method = 'rk4', dropC = false) {
  let x = x0.slice();
  let t = 0;
  const n = Math.round(T / h);
  for (let k = 0; k < n; k++) { x = stepState(a, x, t, h, programs, method, dropC); t += h; }
  return x;
}

describe('free swing (τ = 0)', () => {
  const free: [JointProgram, JointProgram] = [freeJoint(), freeJoint()];
  const x0 = [0, Math.PI / 6, 0, 0];

  it('RK4 at h = 1/240 s keeps the total energy to ~1e-6 J over 5 s', () => {
    const E0 = energies(DEFAULT_ARM, x0).E;
    const x = run(DEFAULT_ARM, x0, free, 5, 1 / 240);
    expect(Math.abs(energies(DEFAULT_ARM, x).E - E0)).toBeLessThan(1e-5);
  });

  it('explicit Euler at the same step gains energy', () => {
    const E0 = energies(DEFAULT_ARM, x0).E;
    const x = run(DEFAULT_ARM, x0, free, 5, 1 / 240, 'euler');
    expect(energies(DEFAULT_ARM, x).E - E0).toBeGreaterThan(0.05);
  });

  it('dropping c breaks energy conservation even with RK4', () => {
    const E0 = energies(DEFAULT_ARM, x0).E;
    const x = run(DEFAULT_ARM, x0, free, 5, 1 / 240, 'rk4', true);
    expect(Math.abs(energies(DEFAULT_ARM, x).E - E0)).toBeGreaterThan(0.05);
  });

  it('wrapping the angles does not change the energy', () => {
    const x = [7.5, -9.1, 1.2, -0.3];
    const w = wrapAngles(x);
    expect(w[0]).toBeGreaterThan(-Math.PI - 1e-12);
    expect(w[0]).toBeLessThanOrEqual(Math.PI + 1e-12);
    expect(energies(DEFAULT_ARM, w).E).toBeCloseTo(energies(DEFAULT_ARM, x).E, 10);
  });
});

describe('hybrid free / driven joints', () => {
  it('both driven: τ = M θ̈ + c + g', () => {
    const a = ARMS[1];
    const s = STATES[0];
    const r = solveHybrid(a, s.th, s.dth, [{ kind: 'driven', ddq: s.ddth[0] }, { kind: 'driven', ddq: s.ddth[1] }]);
    const { M, c, g } = dynamicsTerms(a, s.th, s.dth);
    for (let i = 0; i < 2; i++) expect(r.tau[i]).toBeCloseTo(M[i][0] * s.ddth[0] + M[i][1] * s.ddth[1] + c[i] + g[i], 10);
  });

  it('mixed modes satisfy all equations and respect the given torques', () => {
    for (const a of ARMS) {
      for (const s of STATES) {
        const cases: [Parameters<typeof solveHybrid>[3], number[]][] = [
          [[{ kind: 'driven', ddq: 1.5 }, { kind: 'free', tau: 0.7 }], [0, 1]],
          [[{ kind: 'free', tau: -2 }, { kind: 'driven', ddq: -0.4 }], [1, 0]],
          [[{ kind: 'free', tau: 1 }, { kind: 'free', tau: -1 }], [1, 1]],
        ];
        for (const [drives] of cases) {
          const r = solveHybrid(a, s.th, s.dth, drives);
          const { M, c, g } = dynamicsTerms(a, s.th, s.dth);
          for (let i = 0; i < 2; i++) {
            expect(M[i][0] * r.ddth[0] + M[i][1] * r.ddth[1] + c[i] + g[i]).toBeCloseTo(r.tau[i], 9);
            const d = drives[i];
            if (d.kind === 'free') expect(r.tau[i]).toBeCloseTo(d.tau, 9);
            else expect(r.ddth[i]).toBeCloseTo(d.ddq, 12);
          }
        }
      }
    }
  });

  it('spinning joint 1 at a constant rate with the elbow locked: τ2 = m2 L1 L2 sin θ2 ω², τ1 = 0 (no gravity)', () => {
    const a: Arm = { ...DEFAULT_ARM, g: 0, m2: 1.3, L1: 0.9 };
    const w = 2.5, th2 = 1.1;
    const programs: [JointProgram, JointProgram] = [constantRate(0.3, w), holdAt(th2)];
    const x = initialState([0.3, th2], [0, 0], programs);
    const r = evaluate(a, x, 0, programs);
    expect(r.tau[0]).toBeCloseTo(0, 10);
    expect(r.tau[1]).toBeCloseTo(a.m2 * a.L1 * a.L2 * Math.sin(th2) * w * w, 10);
    // the driven motion is followed exactly
    const x2 = run(a, x, programs, 1, 1 / 240);
    expect(x2[0]).toBeCloseTo(0.3 + w, 9);
    expect(x2[1]).toBeCloseTo(th2, 12);
  });

  it('skater: with τ1 = 0 and no gravity the momentum p1 is conserved and folding speeds joint 1 up', () => {
    const a: Arm = { ...DEFAULT_ARM, g: 0 };
    const qa = (10 * Math.PI) / 180, qb = (160 * Math.PI) / 180;
    const programs: [JointProgram, JointProgram] = [freeJoint(0), foldCycle(qa, qb, 1, 1)];
    let x = initialState([0, 0], [2, 0], programs);
    const p0 = eulerLagrangeTerms(a, [x[0], x[1]], [x[2], x[3]], [0, 0]).momentum[0];
    let t = 0;
    const h = 1 / 240;
    for (let k = 0; k < Math.round(2.5 / h); k++) {
      x = stepState(a, x, t, h, programs);
      t += h;
      const p = eulerLagrangeTerms(a, [x[0], x[1]], [x[2], x[3]], [0, 0]).momentum[0];
      expect(Math.abs(p - p0)).toBeLessThan(1e-6);
    }
    // after the fold (t = 2.5 s: resting at qb), θ̇1 = p1 / M11(qb)
    expect(x[1]).toBeCloseTo(qb, 9);
    const M11 = dynamicsTerms(a, [0, qb], [0, 0]).M[0][0];
    expect(x[2]).toBeCloseTo(p0 / M11, 5);
    expect(x[2]).toBeGreaterThan(8);
  });

  it('folding against the spin (θ2: −10° → −160°) never slows joint 1 below its starting rate', () => {
    const a: Arm = { ...DEFAULT_ARM, g: 0 };
    const programs: [JointProgram, JointProgram] = [freeJoint(0), foldCycle((-10 * Math.PI) / 180, (-160 * Math.PI) / 180, 1, 1)];
    let x = initialState([0, 0], [2, 0], programs);
    let t = 0;
    const h = 1 / 240;
    let minRate = Infinity;
    for (let k = 0; k < Math.round(2.0 / h); k++) {
      x = stepState(a, x, t, h, programs);
      t += h;
      minRate = Math.min(minRate, x[2]);
    }
    // (folding WITH the spin, θ2: +10° → +160°, makes θ̇1 dip to ≈ 1 rad/s first)
    expect(minRate).toBeGreaterThanOrEqual(2 - 1e-9);
    expect(x[2]).toBeGreaterThan(8);
  });

  it('foldCycle and sineMotion are smooth and consistent', () => {
    const p = foldCycle(0.2, 2.6, 0.5, 1.2);
    if (p.kind !== 'driven') throw new Error('expected a driven program');
    const eps = 1e-6;
    for (let t = 0; t < 8; t += 0.137) {
      const s = p.at(t);
      const num = (p.at(t + eps).q - p.at(t - eps).q) / (2 * eps);
      expect(s.dq).toBeCloseTo(num, 4);
    }
    expect(p.at(0.25).q).toBeCloseTo(0.2, 12);
    expect(p.at(0.5 + 1.2 + 0.25).q).toBeCloseTo(2.6, 12);
    const sm = sineMotion(0.3, 0.5, 2, 0.4);
    if (sm.kind !== 'driven') throw new Error('expected a driven program');
    const s = sm.at(0.77);
    expect(s.dq).toBeCloseTo((sm.at(0.77 + eps).q - sm.at(0.77 - eps).q) / (2 * eps), 5);
    expect(s.ddq).toBeCloseTo((sm.at(0.77 + eps).dq - sm.at(0.77 - eps).dq) / (2 * eps), 4);
  });
});
