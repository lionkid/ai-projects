import { describe, it, expect } from 'vitest';
import { planarSpaceJacobian, planarBodyJacobian, planarJacobian, planarFK, buildPlanarChain } from './planar';
import { JacobianSpace, JacobianBody, FKinSpace } from './kinematics';
import { Adjoint } from './se3';
import { matMul } from './linalg';

const rows = (J: number[][], idx: number[]) => idx.map((i) => J[i]);
const close = (A: number[][], B: number[][], tol = 1e-9) => {
  expect(A.length).toBe(B.length);
  A.forEach((r, i) => r.forEach((x, j) => expect(x).toBeCloseTo(B[i][j], -Math.log10(tol))));
};

describe('planar space / body Jacobians', () => {
  const L = [1, 0.8, 0.6];
  const chain = buildPlanarChain(L);
  const samples = [[0, 0, 0], [0.3, -1.1, 0.7], [2.5, 0.4, -2.9], [-1.2, 3.0, 1.0]];

  it('match rows [ωz, vx, vy] of JacobianSpace / JacobianBody', () => {
    for (const th of samples) {
      close(planarSpaceJacobian(L, th), rows(JacobianSpace(chain.Slist, th), [2, 3, 4]));
      close(planarBodyJacobian(L, th), rows(JacobianBody(chain.Blist, th), [2, 3, 4]));
    }
  });

  it('are related by the planar adjoint of T_sb', () => {
    for (const th of samples) {
      const T = FKinSpace(chain.M, chain.Slist, th);
      const Js6 = matMul(Adjoint(T), JacobianBody(chain.Blist, th));
      close(planarSpaceJacobian(L, th), rows(Js6, [2, 3, 4]));
    }
  });

  it('give the tip-point Jacobian through v_tip = v_s + ω × p_tip', () => {
    for (const th of samples) {
      const Js = planarSpaceJacobian(L, th);
      const { tip } = planarFK(L, th);
      const Jt = planarJacobian(L, th);
      for (let j = 0; j < L.length; j++) {
        expect(Jt[1][j]).toBeCloseTo(Js[1][j] - Js[0][j] * tip[1], 9);
        expect(Jt[2][j]).toBeCloseTo(Js[2][j] + Js[0][j] * tip[0], 9);
      }
    }
  });

  it('first space column is always (1, 0, 0) for a base joint at the origin', () => {
    const col = planarSpaceJacobian(L, [0.4, 0.5, 0.6]).map((r) => r[0]);
    expect(col[0]).toBe(1);
    expect(Math.abs(col[1])).toBe(0);
    expect(Math.abs(col[2])).toBe(0);
  });
});
