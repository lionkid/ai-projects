# Lesson authoring guide

This document is the contract for anyone (human or agent) adding a lesson to Modern Robotics Lab.
Read `src/types.ts` first; this guide explains how to use it well.

## 1. Where a lesson lives

```
src/lessons/<id>/
  meta.ts      # export const meta: LessonMeta   (eager; navigation only)
  index.ts     # export default lesson: Lesson   (lazy; steps + quiz + mount)
  *.ts         # any helpers you need (keep them in this folder)
  *.test.ts    # optional vitest tests for pure logic in this folder
```

* `<id>` = `ch<CC>-<NN>-<kebab-slug>` with zero-padded chapter and order, e.g. `ch03-02-rotation-matrices`.
  `meta.id` MUST equal the directory name.
* The registry (`src/app/registry.ts`) discovers lessons automatically with `import.meta.glob`.
  **Never edit files outside your lesson directory** (no shared registries, no toolkit changes).
  If the toolkit lacks something, implement it locally in your folder and mention it in your report.
* Import the toolkit from `@/ui` and the math library from `@/mr`. Never import from another lesson.

## 2. Content rules

* UI text, step narrative, quiz: **Traditional Chinese (Taiwan)** with English technical terms kept
  (e.g. 旋量（twist）, Jacobian, PoE). Code comments: English.
* Wording must be **original**. The textbook is a reference for concepts and section numbers only;
  do not copy or closely paraphrase sentences, exercises or figure captions.
* `bookRef` uses v2 section numbers: `§3.2.3`, `§4.1.1–4.1.2`, `§8.1, 8.3`.
* Markdown + LaTeX: `$...$` inline, `$$...$$` display. Available KaTeX macros:
  `\R \SO \SE \so \se \Ad \ad \T (transpose) \skew{x} \hatw \vV \vF \vS \vB \dd \tr \diag \atantwo \sgn`.
  Inside TypeScript template strings remember to double the backslashes (`\\theta`).
* Steps: 4–7. Each step = one idea, ~80–200 Chinese characters plus at most one or two equations,
  and a `simHint` telling the learner exactly what to try in the simulation. End the last step with a
  bridge to the next lesson or to the quiz.
* Quiz: 2–4 multiple-choice questions that test understanding or transfer (predict what the sim will
  do, interpret a quantity), not recall. Explanations must say *why*.
* Use the lab conventions: x red / y green / z blue, z-up 3D scenes, radians internally
  (`angleSlider` shows degrees), frames written `{s}`, `{b}`, matrices as `number[][]` row-major.

## 3. The simulation (`mount`)

```ts
import type { Lesson, LessonContext, LessonHandle } from '@/types';
import { el, controlPanel, row, slider, angleSlider, toggle, button, select, readout, matrixView,
         playback, simTitle, note, legend, kv, split, Canvas2D, Plot2D, Scene3D, createLoop, substep,
         cssVar, fmt, fmtDeg, mdElement, mathEl } from '@/ui';
import * as mr from '@/mr';
import { meta } from './meta';

function mount(host: HTMLElement, ctx: LessonContext): LessonHandle {
  simTitle(host, '標題', '選用徽章');
  const cv = new Canvas2D(host, { aspect: 4 / 3, world: { xmin: -2, xmax: 2, ymin: -1.5, ymax: 1.5 }, overlay: true });
  const panel = controlPanel(host);
  const s1 = angleSlider(panel, { label: 'θ₁', value: 0, onInput: () => cv.requestRender() });
  cv.draw = (c) => { c.grid(0.5); c.frame({ x: 0, y: 0, theta: 0 }, 0.4, { labels: ['x', 'y'] }); /* ... */ };
  cv.requestRender();
  const focus = (i: number) => { /* highlight / reconfigure for step i */ };
  focus(ctx.currentStep);
  return { onStep: focus, destroy() { cv.destroy(); } };
}

const lesson: Lesson = { meta, steps, quiz, mount };
export default lesson;
```

Toolkit summary (see `src/ui/*.ts` for full signatures):

| Piece | Use |
|---|---|
| `Canvas2D` | world-coordinate 2D drawing (y up, aspect kept). `draw` callback + `requestRender()`. Primitives: `grid, frame, line, polyline, polygon, circle, disk, point, arc, ellipse, arrow, text, link`. Draggable `addHandle({get,set})`. `onPointer({down,move,up})` for raw clicks. `overlay` for a mono text readout. |
| `Scene3D` | Three.js scene, z-up, orbit camera, themed. `addFrame({size,label})` → `setTransform(T4x4)`; `addLink()` → `setEndpoints(a,b)`; `addArrow()` → `setFromTo/setDir`; `addSphere/addBox/addPolyline/addLabel/addScrewAxis(…).setScrew(S)`; `onTick(cb)`; `destroy()`. |
| `Plot2D` | line plots. Streaming: `push(x, [y1,y2])` with `xWindow`; static: `setSeries(i, pts)` / `setXY(i, xs, ys)`; `setMarkers([{x}, {y}, {x,y,label}])`; `equalAspect` for phase planes. |
| controls | `controlPanel(host)` grid; `slider`, `angleSlider`, `toggle`, `button`, `select`, `readout`, `matrixView(...).set(M, {highlightCols:['x','y','z']})`, `playback({onPlay,onPause,onReset,onSpeed,duration,onScrub})`, `kv`, `legend`, `note`, `simTitle`, `split`, `row`. |
| loop | `createLoop((dt, t) => {...})` with `start/stop/speed`; `substep(dt, h, step)` for fixed-step physics. |
| text | `mdElement(md)` renders markdown+LaTeX inside the sim; `mathEl(tex)` for a formula label. `fmt(x, 3)`, `fmtDeg(rad)`. |
| theme | `cssVar('--accent')`, `--axis-x/y/z`, `--series-1..8`, `--fg`, `--fg-muted`, `--grid`. Re-read inside draw callbacks so theme switches work (Canvas2D/Plot2D/Scene3D re-render automatically on theme change). |

Guidelines

* Make the simulation react to the step: `onStep(i)` should set the sim into the state the step talks
  about (enable/disable controls, show/hide overlays, pick a preset configuration, add `.mr-focus` to
  the element the learner should look at). Do not reset everything on each step if the learner's
  own exploration is valuable.
* Prefer direct manipulation (drag the end-effector, drag the obstacle) over sliders where it helps.
* Keep physics stable: fixed sub-steps (`substep`), semi-implicit Euler or RK4, clamp states, wrap
  angles, handle singular configurations (θ=π branch, |J| ≈ 0) explicitly and say so in the UI.
* Everything must be torn down in `destroy()`: loops, `Canvas2D/Plot2D/Scene3D.destroy()`, listeners.
* Avoid layout thrash: at most one `readout.set`/`matrixView.set` per quantity per frame.
* The sim pane is ~600–800 px wide on desktop and full width on mobile; use `split(host)` for
  side-by-side canvas + plot and keep total height reasonable (< ~900 px).
* No network requests, no external assets.

## 4. Math library (`@/mr`)

Function names mirror the open-source Modern Robotics library (so learners can cross-reference):
`VecToso3, so3ToVec, AxisAng3, MatrixExp3, MatrixLog3, RpToTrans, TransToRp, TransInv, VecTose3,
se3ToVec, Adjoint, ScrewToAxis, AxisAng6, MatrixExp6, MatrixLog6, ad, FKinBody, FKinSpace,
JacobianBody, JacobianSpace, IKinBody, IKinSpace, InverseDynamics, MassMatrix, VelQuadraticForces,
GravityForces, EndEffectorForces, ForwardDynamics, EulerStep, rk4Step, CubicTimeScaling,
QuinticTimeScaling, TrapezoidalTimeScaling, JointTrajectory, ScrewTrajectory, CartesianTrajectory,
ComputedTorque, SimulateControl`, plus `linalg` helpers (`matMul, matVec, transpose, inv, pinv, svd,
eigSym, cross, norm, ...`), planar helpers (`planar.ts`) and robot models (`robots.ts`: `planar2R,
planar3R, ur5, rrr3`). Read `src/mr/README.md` for conventions (screw axes are `[ω; v]`, Slist/Blist
are 6×n with axes as columns, matrices row-major `number[][]`).

## 5. Checklist before you report

```bash
npm run typecheck                       # no errors in your folder
npx vitest run src/lessons/<id>         # if you added tests
node scripts/smoke.mjs --lesson <id> --screens /tmp/claude-0/.../scratchpad/smoke   # must PASS
```

Then look at the screenshot: is the first thing the learner sees meaningful (not an empty canvas)?
Do the equations render? Is the text readable at ~700 px width?
