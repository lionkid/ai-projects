/**
 * Predefined robot models for the lessons.
 *
 * Every model gives the PoE description (M, Slist, Blist) and, where
 * available, the Newton-Euler description (Mlist, Glist). Units are SI
 * throughout: metres, kilograms, seconds, radians.
 */

import { type Mat, type Vec, diag, eye, matMul } from './linalg';
import { Adjoint, RpToTrans, TransInv } from './se3';
import { buildPlanarChain } from './planar';

export interface RobotModel {
  /** Short identifier, e.g. 'ur5'. */
  name: string;
  /** Human-readable description (English). */
  description: string;
  /** Number of joints. */
  dof: number;
  /** Home configuration of the end-effector frame. */
  M: Mat;
  /** 6 x n space-frame screw axes at the home configuration. */
  Slist: Mat;
  /** 6 x n body-frame screw axes at the home configuration. */
  Blist: Mat;
  /** n + 1 link-frame transforms M_{i-1,i} (frames at the centres of mass). */
  Mlist?: Mat[];
  /** n spatial inertia matrices (6x6). */
  Glist?: Mat[];
  /** Link lengths for drawing (m), when the robot is a serial chain of straight links. */
  linkLengths?: number[];
  /** [min, max] per joint (rad). */
  jointLimits?: [number, number][];
  /** True when Mlist/Glist are approximate / not consistent with the kinematics to full precision. */
  dynamicsApproximate?: boolean;
  /** Free-form notes (sources of the numbers, caveats). */
  notes?: string;
}

const TWO_PI = 2 * Math.PI;

// ---------------------------------------------------------------------------
// Planar 2R: L1 = L2 = 1 m, point masses of 1 kg at the link tips
// ---------------------------------------------------------------------------

const planar2RChain = buildPlanarChain([1, 1], { masses: [1, 1], model: 'point-mass' });

export const planar2R: RobotModel = {
  name: 'planar2R',
  description: 'Planar 2R arm in the x-y plane, joint axes along +z. L1 = L2 = 1 m, 1 kg point masses at the end of each link, gravity along -y when used with g = [0, -9.81, 0].',
  dof: 2,
  M: planar2RChain.M,
  Slist: planar2RChain.Slist,
  Blist: planar2RChain.Blist,
  Mlist: planar2RChain.Mlist,
  Glist: planar2RChain.Glist,
  linkLengths: [1, 1],
  jointLimits: [
    [-Math.PI, Math.PI],
    [-Math.PI, Math.PI],
  ],
  notes: 'Built with buildPlanarChain([1, 1], { masses: [1, 1], model: "point-mass" }); compare with planar2RDynamics(planar2RParams([1, 1], [1, 1], "point-mass")).',
};

// ---------------------------------------------------------------------------
// Planar 3R: uniform rods
// ---------------------------------------------------------------------------

const planar3RChain = buildPlanarChain([1, 0.8, 0.6], { masses: [1, 0.8, 0.6], model: 'rod' });

export const planar3R: RobotModel = {
  name: 'planar3R',
  description: 'Planar 3R arm with link lengths 1.0, 0.8, 0.6 m modelled as uniform rods of 1.0, 0.8, 0.6 kg (redundant for planar positioning, useful for null-space demos).',
  dof: 3,
  M: planar3RChain.M,
  Slist: planar3RChain.Slist,
  Blist: planar3RChain.Blist,
  Mlist: planar3RChain.Mlist,
  Glist: planar3RChain.Glist,
  linkLengths: [1, 0.8, 0.6],
  jointLimits: [
    [-Math.PI, Math.PI],
    [-Math.PI, Math.PI],
    [-Math.PI, Math.PI],
  ],
};

// ---------------------------------------------------------------------------
// UR5 (6R)
// ---------------------------------------------------------------------------

/** UR5 kinematic constants (m) as used in the book's PoE example. */
export const UR5_DIMENSIONS = { W1: 0.109, W2: 0.082, L1: 0.425, L2: 0.392, H1: 0.089, H2: 0.095 } as const;

function ur5Kinematics(): { M: Mat; Slist: Mat } {
  const { W1, W2, L1, L2, H1, H2 } = UR5_DIMENSIONS;
  const M: Mat = [
    [-1, 0, 0, L1 + L2],
    [0, 0, 1, W1 + W2],
    [0, 1, 0, H1 - H2],
    [0, 0, 0, 1],
  ];
  // Columns: joint 1 about z through the base; joints 2-4 about +y; joint 5 about -z; joint 6 about +y.
  const axes: Vec[] = [
    [0, 0, 1, 0, 0, 0],
    [0, 1, 0, -H1, 0, 0],
    [0, 1, 0, -H1, 0, L1],
    [0, 1, 0, -H1, 0, L1 + L2],
    [0, 0, -1, -W1, L1 + L2, 0],
    [0, 1, 0, H2 - H1, 0, L1 + L2],
  ];
  const Slist: Mat = [0, 1, 2, 3, 4, 5].map((r) => axes.map((a) => a[r]));
  return { M, Slist };
}

/**
 * Link-frame transforms and spatial inertias of the UR5 for Newton-Euler
 * dynamics. Masses, inertias and the positions of the centres of mass along
 * the links follow the manufacturer's URDF (3.7, 8.393, 2.275, 1.219, 1.219,
 * 0.1879 kg; upper-arm COM 0.28 m from the shoulder, forearm COM 0.25 m from
 * the elbow, shoulder lateral offset 0.13585 m, upper-arm COM lateral offset
 * -0.1197 m). The joint-to-joint distances, however, come from UR5_DIMENSIONS
 * (the rounded book values) instead of the URDF (0.089159, 0.10915, 0.81725,
 * 0.09465, 0.0823 m, all within a millimetre of the book), so that the chain
 * M01 M12 ... M67 ends exactly at the book's M and Newton-Euler statics agree
 * with J_b^T F_tip to machine precision.
 */
function ur5Dynamics(): { Mlist: Mat[]; Glist: Mat[] } {
  const { W1, W2, L1, L2, H1, H2 } = UR5_DIMENSIONS;
  const upperArmCom = 0.28; // shoulder -> upper-arm COM, along the link (m)
  const forearmCom = 0.25; // elbow -> forearm COM, along the link (m)
  const shoulderOffset = 0.13585; // lateral (y) offset of the shoulder frame (m)
  const upperArmLateral = -0.1197; // lateral (y) offset of the upper-arm COM relative to the shoulder frame (m)
  // RotY(+90 deg): frames {2}, {3} have z along the arm (space +x); after a second one, frames {4}..{6} have z = space -z.
  const Ry: Mat = [
    [0, 0, 1],
    [0, 1, 0],
    [-1, 0, 0],
  ];
  // RotX(-90 deg) for the tool frame, whose z axis points along space +y like the book's M.
  const Rx: Mat = [
    [1, 0, 0],
    [0, 0, 1],
    [0, -1, 0],
  ];
  const Mlist: Mat[] = [
    RpToTrans(eye(3), [0, 0, H1]), // {0} -> {1}: up the base column
    RpToTrans(Ry, [upperArmCom, shoulderOffset, 0]), // {1} -> {2}: shoulder offset, then along the upper arm
    RpToTrans(eye(3), [0, upperArmLateral, L1 - upperArmCom + forearmCom]), // {2} -> {3}: past the elbow to the forearm COM
    RpToTrans(Ry, [0, 0, L2 - forearmCom]), // {3} -> {4}: to the wrist (end of the forearm)
    RpToTrans(eye(3), [0, W1 - shoulderOffset - upperArmLateral, 0]), // {4} -> {5}: remaining lateral offset W1
    RpToTrans(eye(3), [0, 0, H2]), // {5} -> {6}: down by H2 (frame z = space -z)
    RpToTrans(Rx, [0, W2, 0]), // {6} -> {7}: tool offset W2
  ];
  const Glist: Mat[] = [
    diag([0.010267495893, 0.010267495893, 0.00666, 3.7, 3.7, 3.7]),
    diag([0.22689067591, 0.22689067591, 0.0151074, 8.393, 8.393, 8.393]),
    diag([0.049443313556, 0.049443313556, 0.004095, 2.275, 2.275, 2.275]),
    diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219]),
    diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219]),
    diag([0.0171364731454, 0.0171364731454, 0.033822, 0.1879, 0.1879, 0.1879]),
  ];
  return { Mlist, Glist };
}

const ur5Kin = ur5Kinematics();
const ur5Dyn = ur5Dynamics();

export const ur5: RobotModel = {
  name: 'ur5',
  description: 'Universal Robots UR5, 6R industrial arm. Kinematics from the book\'s PoE example (W1 = 0.109, W2 = 0.082, L1 = 0.425, L2 = 0.392, H1 = 0.089, H2 = 0.095 m); home pose has the arm stretched horizontally along +x.',
  dof: 6,
  M: ur5Kin.M,
  Slist: ur5Kin.Slist,
  Blist: matMul(Adjoint(TransInv(ur5Kin.M)), ur5Kin.Slist),
  Mlist: ur5Dyn.Mlist,
  Glist: ur5Dyn.Glist,
  jointLimits: [
    [-TWO_PI, TWO_PI],
    [-TWO_PI, TWO_PI],
    [-TWO_PI, TWO_PI],
    [-TWO_PI, TWO_PI],
    [-TWO_PI, TWO_PI],
    [-TWO_PI, TWO_PI],
  ],
  notes: 'Mlist/Glist: masses, inertias and COM placements from the manufacturer URDF (COM frames, diagonal inertias), joint-to-joint distances from the book values above, so the dynamics are exactly consistent with the kinematics (product of Mlist = M).',
};

// ---------------------------------------------------------------------------
// RRR3: a simple spatial anthropomorphic arm for 3-D demos
// ---------------------------------------------------------------------------

/** Base height, upper-arm and forearm lengths (m). */
export const RRR3_DIMENSIONS = { L0: 0.3, L1: 0.4, L2: 0.3 } as const;

function rrr3Model(): RobotModel {
  const { L0, L1, L2 } = RRR3_DIMENSIONS;
  const m = [2.0, 1.5, 1.0]; // kg: base column, upper arm, forearm
  const rCol = 0.05; // radius of the base column (m)
  const rArm = 0.03; // radius of the arm cylinders (m)
  // Joint 1: vertical (z) through the origin. Joints 2, 3: horizontal (y) at height L0.
  const axes: Vec[] = [
    [0, 0, 1, 0, 0, 0],
    [0, 1, 0, -L0, 0, 0],
    [0, 1, 0, -L0, 0, L1],
  ];
  const Slist: Mat = [0, 1, 2, 3, 4, 5].map((r) => axes.map((a) => a[r]));
  const M = RpToTrans(eye(3), [L1 + L2, 0, L0]);
  // Link frames at the centres of mass, all aligned with the space frame at home.
  const Mlist: Mat[] = [
    RpToTrans(eye(3), [0, 0, L0 / 2]),
    RpToTrans(eye(3), [L1 / 2, 0, L0 / 2]),
    RpToTrans(eye(3), [L1 / 2 + L2 / 2, 0, 0]),
    RpToTrans(eye(3), [L2 / 2, 0, 0]),
  ];
  const cyl = (mass: number, radius: number, length: number, axis: 'x' | 'z'): Mat => {
    const It = (mass * (3 * radius * radius + length * length)) / 12; // transverse
    const Ia = (mass * radius * radius) / 2; // about the cylinder axis
    return axis === 'z' ? diag([It, It, Ia, mass, mass, mass]) : diag([Ia, It, It, mass, mass, mass]);
  };
  const Glist: Mat[] = [cyl(m[0], rCol, L0, 'z'), cyl(m[1], rArm, L1, 'x'), cyl(m[2], rArm, L2, 'x')];
  return {
    name: 'rrr3',
    description: 'Spatial RRR arm: a vertical base joint (z) followed by a shoulder and an elbow joint about horizontal (y) axes; L0 = 0.3 m column, 0.4 m upper arm, 0.3 m forearm, modelled as solid cylinders (2.0, 1.5, 1.0 kg). Home pose: arm horizontal along +x.',
    dof: 3,
    M,
    Slist,
    Blist: matMul(Adjoint(TransInv(M)), Slist),
    Mlist,
    Glist,
    linkLengths: [L0, L1, L2],
    jointLimits: [
      [-Math.PI, Math.PI],
      [-Math.PI / 2, Math.PI / 2],
      [-2.6, 2.6],
    ],
  };
}

export const rrr3: RobotModel = rrr3Model();

/** All predefined models keyed by name. */
export const robots: Record<string, RobotModel> = { planar2R, planar3R, ur5, rrr3 };
