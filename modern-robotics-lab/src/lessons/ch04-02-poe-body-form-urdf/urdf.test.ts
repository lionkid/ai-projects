import { describe, expect, it } from 'vitest';
import { FKinSpace, RotX, RotY, UR5_DIMENSIONS, getCol, ur5 } from '@/mr';
import { homeJointFrames, rpyToR, screwFromFrame, urdfForward, urdfToPoE, urdfXml, UR5_URDF } from './urdf';

const { L1, H1 } = UR5_DIMENSIONS;

function expectMat(A: number[][], B: number[][], digits = 12): void {
  A.forEach((row, i) => row.forEach((x, j) => expect(x).toBeCloseTo(B[i][j], digits)));
}

describe('URDF → PoE', () => {
  it('rpy follows Rz(yaw) Ry(pitch) Rx(roll)', () => {
    expectMat(rpyToR([-Math.PI / 2, 0, 0]), RotX(-Math.PI / 2));
    expectMat(rpyToR([0, Math.PI / 2, 0]), RotY(Math.PI / 2));
  });

  it('accumulating the origins gives the book M and S_i of the UR5', () => {
    const { M, Slist } = urdfToPoE(UR5_URDF);
    expectMat(M, ur5.M);
    expectMat(Slist, ur5.Slist);
  });

  it('the elbow: origin (0, 0, L1) in a pitched parent frame is really +x in {s}', () => {
    const frames = homeJointFrames(UR5_URDF);
    const elbow = screwFromFrame(frames[2], UR5_URDF[2].axis);
    [L1, 0, H1].forEach((x, k) => expect(elbow.q[k]).toBeCloseTo(x, 12));
    getCol(ur5.Slist, 2).forEach((x, k) => expect(elbow.S[k]).toBeCloseTo(x, 12));
  });

  it('URDF forward kinematics equals the PoE formula for any θ', () => {
    let s = 3;
    const rnd = () => { s = (s * 16807) % 2147483647; return s / 2147483647; };
    for (let t = 0; t < 6; t++) {
      const th = Array.from({ length: 6 }, () => (rnd() * 2 - 1) * Math.PI);
      expectMat(urdfForward(UR5_URDF, th).T, FKinSpace(ur5.M, ur5.Slist, th));
    }
  });

  it('prints a readable joint snippet', () => {
    const x = urdfXml(UR5_URDF[1]);
    expect(x).toContain('<joint name="shoulder_lift_joint" type="revolute">');
    expect(x).toContain('rpy="0 1.5708 0"');
    expect(x).toContain('<axis xyz="0 1 0"/>');
    expect(urdfXml(UR5_URDF[6])).not.toContain('<axis');
  });
});
