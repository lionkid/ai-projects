/**
 * A small, original URDF-style description of the lab's UR5 (book dimensions, not the vendor file).
 * Each joint gives its frame relative to the PARENT link (origin xyz + rpy) and its axis in its own
 * frame, which is how URDF works. Converting to PoE means accumulating the origins along the chain.
 */
import { type Mat, type Vec, MatrixExp3, RotX, RotY, RotZ, RpToTrans, UR5_DIMENSIONS, VecToso3, cross, eye, matMul, matVec } from '@/mr';

export interface UrdfJoint {
  name: string;
  type: 'revolute' | 'fixed';
  parent: string;
  child: string;
  xyz: Vec;
  /** Fixed-axis roll, pitch, yaw (rad): R = Rz(yaw) · Ry(pitch) · Rx(roll). */
  rpy: Vec;
  /** Axis in the joint frame (revolute joints). */
  axis: Vec;
}

const { W1, W2, L1, L2, H1, H2 } = UR5_DIMENSIONS;
const HALF_PI = Math.PI / 2;

/**
 * The link frames of the upper arm and forearm are pitched by 90° so that their z axes run along
 * the arm (a common URDF habit); the origins after them are therefore written along z.
 */
export const UR5_URDF: UrdfJoint[] = [
  { name: 'shoulder_pan_joint', type: 'revolute', parent: 'base_link', child: 'shoulder_link', xyz: [0, 0, H1], rpy: [0, 0, 0], axis: [0, 0, 1] },
  { name: 'shoulder_lift_joint', type: 'revolute', parent: 'shoulder_link', child: 'upper_arm_link', xyz: [0, 0, 0], rpy: [0, HALF_PI, 0], axis: [0, 1, 0] },
  { name: 'elbow_joint', type: 'revolute', parent: 'upper_arm_link', child: 'forearm_link', xyz: [0, 0, L1], rpy: [0, 0, 0], axis: [0, 1, 0] },
  { name: 'wrist_1_joint', type: 'revolute', parent: 'forearm_link', child: 'wrist_1_link', xyz: [0, 0, L2], rpy: [0, HALF_PI, 0], axis: [0, 1, 0] },
  { name: 'wrist_2_joint', type: 'revolute', parent: 'wrist_1_link', child: 'wrist_2_link', xyz: [0, W1, 0], rpy: [0, 0, 0], axis: [0, 0, 1] },
  { name: 'wrist_3_joint', type: 'revolute', parent: 'wrist_2_link', child: 'wrist_3_link', xyz: [0, 0, H2], rpy: [0, 0, 0], axis: [0, 1, 0] },
  { name: 'tool0_fixed_joint', type: 'fixed', parent: 'wrist_3_link', child: 'tool0', xyz: [0, W2, 0], rpy: [-HALF_PI, 0, 0], axis: [0, 0, 0] },
];

/** URDF rpy convention: R = Rz(yaw) Ry(pitch) Rx(roll). */
export function rpyToR(rpy: Vec): Mat {
  return matMul(RotZ(rpy[2]), matMul(RotY(rpy[1]), RotX(rpy[0])));
}

/** Transform from the parent link frame to the joint frame (θ = 0). */
export function originTransform(j: UrdfJoint): Mat {
  return RpToTrans(rpyToR(j.rpy), j.xyz);
}

/** Rotation about the joint axis by θ, as a 4×4 transform in the joint frame. */
export function jointMotion(j: UrdfJoint, theta: number): Mat {
  if (j.type !== 'revolute') return eye(4);
  return RpToTrans(MatrixExp3(VecToso3(j.axis.map((x) => x * theta))), [0, 0, 0]);
}

/**
 * URDF forward kinematics: walk the chain, multiplying origin · motion. Returns the frame of every
 * joint (in {s}, before its own motion) and the final link frame.
 */
export function urdfForward(joints: UrdfJoint[], theta: Vec): { frames: Mat[]; T: Mat } {
  const frames: Mat[] = [];
  let T = eye(4);
  let k = 0;
  for (const j of joints) {
    T = matMul(T, originTransform(j));
    frames.push(T);
    if (j.type === 'revolute') T = matMul(T, jointMotion(j, theta[k++] ?? 0));
  }
  return { frames, T };
}

/** Joint frames at home, accumulated along the parent chain: T_{s,j_i} = T_origin,1 ⋯ T_origin,i. */
export function homeJointFrames(joints: UrdfJoint[]): Mat[] {
  return urdfForward(joints, joints.map(() => 0)).frames;
}

/** Space screw axis of a revolute joint from its accumulated home frame: ω = R·axis, q = p. */
export function screwFromFrame(F: Mat, axis: Vec): { omega: Vec; q: Vec; S: Vec } {
  const R = [F[0].slice(0, 3), F[1].slice(0, 3), F[2].slice(0, 3)];
  const omega = matVec(R, axis);
  const q = [F[0][3], F[1][3], F[2][3]];
  const v = cross(q, omega); // −ω × q
  return { omega, q, S: [...omega, ...v] };
}

/** PoE description (M, Slist) obtained from the URDF chain. */
export function urdfToPoE(joints: UrdfJoint[]): { M: Mat; Slist: Mat } {
  const frames = homeJointFrames(joints);
  const axes: Vec[] = [];
  joints.forEach((j, i) => { if (j.type === 'revolute') axes.push(screwFromFrame(frames[i], j.axis).S); });
  const Slist: Mat = [0, 1, 2, 3, 4, 5].map((r) => axes.map((a) => a[r]));
  return { M: urdfForward(joints, axes.map(() => 0)).T, Slist };
}

const num = (x: number): string => {
  if (Math.abs(x) < 1e-12) return '0';
  const r = Math.round(x * 1e4) / 1e4;
  return String(r);
};

/** The joint written as a URDF snippet (for display). */
export function urdfXml(j: UrdfJoint): string {
  const lines = [
    `<joint name="${j.name}" type="${j.type}">`,
    `  <parent link="${j.parent}"/>`,
    `  <child link="${j.child}"/>`,
    `  <origin xyz="${j.xyz.map(num).join(' ')}" rpy="${j.rpy.map(num).join(' ')}"/>`,
  ];
  if (j.type === 'revolute') lines.push(`  <axis xyz="${j.axis.map(num).join(' ')}"/>`);
  lines.push('</joint>');
  return lines.join('\n');
}
