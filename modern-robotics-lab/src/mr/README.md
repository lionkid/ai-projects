# `src/mr` – Modern Robotics math library (TypeScript)

Pure-function implementations of the mathematics used throughout the lessons:
rigid-body motions, product-of-exponentials kinematics, Newton–Euler dynamics,
trajectory generation and control. Function names mirror the open-source
*Modern Robotics* code library so learners can cross-reference, but every
implementation here is original and documented in English.

```ts
import { FKinSpace, MatrixLog6, IKinBody, ur5 } from '@/mr';
```

## Conventions

| Thing | Representation |
| --- | --- |
| Matrix `Mat` | `number[][]`, row-major (`A[i][j]` = row i, column j) |
| Vector `Vec` | `number[]` |
| Rotation `R` | 3×3, maps body coordinates to space coordinates |
| Transform `T` | 4×4 `[[R, p], [0, 1]]` |
| Twist `V` | 6-vector `[ωx, ωy, ωz, vx, vy, vz]` (angular part **first**) |
| Wrench `F` | 6-vector `[mx, my, mz, fx, fy, fz]` (moment first) |
| Screw list `Slist` / `Blist` | 6 × n, **columns** are the joint screw axes |
| Joint vector `thetalist` | length n |
| `Mlist` | n + 1 transforms `M_{i-1,i}`, link frames at the centres of mass |
| `Glist` | n spatial inertias `diag(Ib, m I)` (6×6) |
| Units | m, kg, s, rad, N, N·m |
| Angles returned by logs | θ ∈ [0, π] |
| Planar twists | `[ωz, vx, vy]` |

All functions are pure (inputs never mutated; only `setCol` mutates by design).
Dimension mismatches throw an `Error` with a readable message. Singular
matrices make `inv` / `solve` throw; `pinv` never throws.

## Modules

### `linalg.ts`
`zeros, zerosVec, eye, diag, diagOf, clone, shape, getCol, setCol, transpose,
flatten, reshape, hstack, vstack, block, matAdd, matSub, matScale, matMul,
matVec, trace, normFro, maxAbs, vecAdd, vecSub, vecScale, vecNeg, dot, cross,
norm, normalize, outer, nearZero(x, eps=1e-6), clamp, wrapToPi, wrapTo2Pi,
wrapToPiVec, approxEqual, det, inv, solve, svd, pinv, eigSym, cond`

* `svd(A)` – thin SVD of any m×n matrix (one-sided Jacobi). Returns `{U, S, V}`
  with `S` sorted descending and `A = U diag(S) Vᵀ`.
* `pinv(A, rcond=1e-10)` – Moore–Penrose pseudo-inverse via SVD.
* `eigSym(A)` – Jacobi eigendecomposition of a symmetric matrix; `vectors` are
  **columns**, `values` sorted descending.

### `so3.ts`
`VecToso3, so3ToVec, AxisAng3, MatrixExp3, MatrixLog3, RotInv, ProjectToSO3,
DistanceToSO3, TestIfSO3, RotX, RotY, RotZ, rotZYX, eulerZYX, RotToQuat,
QuatToRot, rotationAngleBetween, rodriguesCoefficients`

`MatrixLog3` is robust at θ = 0 (series) and θ = π (axis from the symmetric
part of R). At θ = π (to machine precision) both ±ω̂ are valid logarithms; the
sign is chosen deterministically so that the first component of ω̂ with
|ω̂ᵢ| > 10⁻⁶ is positive. Lesson code should still not rely on the sign at π.
`rotZYX([α, β, γ]) = RotZ(α) RotY(β) RotX(γ)`. Quaternions are `[w, x, y, z]`.

### `se3.ts`
`RpToTrans, TransToRp, TransInv, VecTose3, se3ToVec, Adjoint, ScrewToAxis,
ScrewAxisToPointDir, AxisAng6, MatrixExp6, MatrixLog6, ad, ProjectToSE3,
DistanceToSE3, TestIfSE3, transformPoint, transformVector, Trans`

### `kinematics.ts`
`FKinBody, FKinSpace, spaceJointTransforms, SlistToBlist, BlistToSlist,
JacobianBody, JacobianSpace, manipulability, IKinBody, IKinSpace`

`IKinBody(Blist, M, T, thetalist0, eomg=1e-3, ev=1e-4, opts)` and
`IKinSpace(Slist, M, T, thetalist0, eomg, ev, opts)` return
`{ thetalist, success, iterations, history }` where `history[k]` =
`{ theta, errorOmega, errorV }` (k = 0 is the initial guess) so a lesson can
replay the Newton–Raphson iterations. `opts = { maxIterations, lambda, maxStep }`;
`lambda > 0` switches to damped least squares.

### `dynamics.ts`
`InverseDynamics, MassMatrix, VelQuadraticForces, GravityForces,
EndEffectorForces, ForwardDynamics, EulerStep, rk4Step, eulerStepGeneric,
manipulatorODE, InverseDynamicsTrajectory, ForwardDynamicsTrajectory,
linkFramePoses, KineticEnergy, PotentialEnergy, TotalEnergy`

`g` is the gravity acceleration vector in the space frame (e.g. `[0, 0, -9.81]`),
`Ftip` the wrench the end-effector applies to the environment (in {b}).
Trajectory functions take/return arrays of per-sample vectors (`Vec[]`, one
row per time step). `ForwardDynamicsTrajectory(..., dt, intRes, integrator)`
accepts `'rk4'` (default) or `'euler'`.

### `trajectory.ts`
`CubicTimeScaling, CubicTimeScalingDerivs, QuinticTimeScaling,
QuinticTimeScalingDerivs, trapezoidParams, TrapezoidalTimeScaling,
trapezoidScaling, trapezoidSample, JointTrajectory, ScrewTrajectory,
CartesianTrajectory, viaPointCubic`

Time scalings are called as `(Tf, t)` – including
`TrapezoidalTimeScaling(Tf, t, { v } | { a })`; the `*Derivs` versions and the
trapezoid return `{ s, sdot, sddot }`. `trapezoidParams({Tf, v} | {Tf, a} |
{v, a})` completes the missing parameter and reports `valid` / `reason`;
inputs within 10⁻¹² of the triangular limit are snapped onto it, and a `valid`
result never contains NaN. Path methods accept `3` (cubic), `5` (quintic) or a
custom `(Tf, t) => s`; `trapezoidScaling({ v } | { a })` wraps a trapezoid as
such a method. `N` must be an integer ≥ 2 (the paths throw otherwise).
`viaPointCubic(points, times, 'zero' | 'heuristic' | velocities)` returns an
object with `evaluate(t)`, `sample(N)` and the segment coefficients.

### `control.ts`
`ComputedTorque, SimulateControl, plantAcceleration, pidStep, createPID,
pidInitialState`

`ComputedTorque` implements τ = M(θ)(Kp e + Ki ∫e + Kd ė) + InverseDynamics(θ, θ̇, θ̈d);
gains may be scalars or per-joint vectors. `SimulateControl` returns
`{ taumat, thetamat, dthetamat, errmat }` aligned with the desired trajectory.
The PID helper supports output saturation with anti-windup, an integral clamp
and derivative-on-measurement.

### `planar.ts`
`rot2, trans2, se2, transInv2, transformPoint2, planarFK, planarJacobian,
planarTipJacobian, planar2RIK, planar2RParams, planar2RDynamics,
planar2RAccel, buildPlanarChain`

* `planarJacobian` rows are `[ωz; vx; vy]` of the **tip point** in the fixed frame.
* `planar2RIK` returns both elbow solutions (`elbow: 'down'` for θ₂ ≥ 0) and a
  `reachable` flag.
* `planar2RDynamics` is the Lagrangian closed form with COM distances `r_i` and
  COM inertias `I_i`; `planar2RParams(L, m, 'point-mass' | 'rod')` fills them.
  Gravity acts along −y.
* `buildPlanarChain(L, { masses, model })` produces `{ M, Slist, Blist, Mlist,
  Glist }` for the same arm so planar and PoE/Newton–Euler results can be compared.

### `robots.ts`
`planar2R` (L = 1, 1 m; 1 kg point masses), `planar3R` (1, 0.8, 0.6 m rods),
`ur5` (book kinematics W1 = 0.109, W2 = 0.082, L1 = 0.425, L2 = 0.392,
H1 = 0.089, H2 = 0.095 m; `Mlist`/`Glist` with URDF masses, inertias and COM
placements but the book's joint-to-joint distances, so the product of `Mlist`
is exactly `M`), `rrr3` (spatial RRR arm, cylinders), and the `robots`
map. Each `RobotModel` has `M, Slist, Blist, dof` and optionally
`Mlist, Glist, linkLengths, jointLimits, notes`.

## Tests

```bash
cd /home/user/ai-projects/modern-robotics-lab && npx vitest run src/mr
```

The suites cover exp/log round trips (θ near 0 and π), adjoint identities, the
UR5 forward-kinematics example, Jacobians against finite differences,
convergence/failure of IK, symmetric positive-definite mass matrices,
forward/inverse dynamics consistency, closed-form vs. Newton–Euler 2R
dynamics, energy conservation under RK4, time-scaling boundary conditions and
the SVD / pseudo-inverse / eigen identities.
