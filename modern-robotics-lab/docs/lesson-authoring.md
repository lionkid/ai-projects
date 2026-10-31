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
| robot2d | `drawPlanarArm(cv, lengths, thetas, {frames, labels, angleArcs, alpha})` → joint points; `planarArmPoints`, `planarArmAngles`; `drawEllipse2x2(cv, center, JJt, {scale, inverse, axes})` for manipulability/force ellipses; `eig2x2Sym`; `drawTrail`. |
| 3D theme colours | Pass colours as CSS variable names (`'--axis-x'`, `'--fg'`, `'--fg-muted'`, `'--accent'`, `'--warn'`, `'--series-3'` …) to `Scene3D.add*`, `Arrow3D`, `Link3D`, `ScrewAxis3D`, `makeTextSprite`, `themedStandardMaterial`, `Robot3D` options: they are re-resolved automatically when the learner switches theme (text sprites are redrawn). A resolved colour (`cssVar(...)` result or hex) stays fixed, so do not pass `cssVar('--fg')` to 3D objects. Defaults are already theme-bound. |
| robot3d | `new Robot3D(scene, {M, Slist}, {showAxes, showJointFrames, frameSize, opacity})` → `setConfig(thetas)` returns `{T, points}`; a stick figure for any PoE chain (`ur5`, `rrr3`, planar models). `homeJointPoints(Slist)`. |
| jointSliders | `jointSliders(panel, {model, values, onInput(thetas, i)})` → `get/set`; auto-generates θ₁…θₙ angle sliders from a RobotModel. |
| stepper | `algorithmStepper(panel, {onStep: () => boolean, onReset, intervalMs})` single-step / auto / reset / speed / iteration count for Newton IK, A*, RRT, Newton–Euler passes, phase-plane integration. |
| presets | `presetChips(panel, [{label, apply}])` → `activate(i)`, `clear()`; drive from `onStep`. |
| keyboard | `keyboardDrive(cv.el, {onChange})` → `axis('down','up')`, `has('brake')`; WASD/arrows scoped to the focused canvas with a hint overlay; call `destroy()`. |
| bars | `torqueBars(host, {labels, max, limit, unit})` → `set(values)`; signed per-joint bars with saturation marking (torques, energies). |
| raster | `new RasterLayer(world, nx, ny).paint((x,y) => bool \| number, {color \| colormap, range})` then `layer.draw(cv)` inside the draw callback; heat maps for C-obstacles, σ_min, potential fields. Colormaps: `viridisLike`, `diverging`, `grayscale`. Re-paint only when inputs change (it is O(nx·ny)). |
| poseHandle | `poseHandle(cv, {x,y,theta}, {onChange})` → draggable SE(2) target (position + heading knob); call `.draw()` in the draw callback. |
| physics | `physicsRunner({h, step(h,t), onFrame(t), guard, onBlowUp, onReset})` → `start/stop/reset/advance`, `speed`; fixed sub-steps with blow-up protection. Pair with `mr.rk4Step`. |
| plotGroup | `plotGroup([plotA, plotB]).setCursor(t)` shared time cursor + `setXRange`. |
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

### 3b. Toolkit updates (read this even if you know the toolkit)

* **Theme-aware colours everywhere.** In 2D, any colour string may be a CSS variable name:
  `'--accent'` or `'var(--accent)'`. `Canvas2D` styles, text, handles (`handle.color`), `Plot2D`
  series/markers and `torqueBars` resolve it at draw time, so a theme switch just works. `cssVar('--x')`
  returns a fixed value: fine inside a draw callback, wrong for anything stored at creation time.
  In 3D, pass the variable names to `Scene3D.add*`, `Arrow3D`, `Link3D`, `ScrewAxis3D`, `Robot3D`,
  `makeTextSprite`, `themedStandardMaterial` (see the 3D row above). `themeColor(c)` (from `@/ui`)
  resolves such a string yourself.
* `readout(panel, { label, unit, minWidth: '7ch' })` stops rows of readouts from re-wrapping as digits change;
  `readout.set`, `kv.set` and `matrixView.set` skip unchanged text and update matrix cells in place,
  so calling them every frame is cheap. `kv(...).setLabel(key, label)` and `.show(key, visible)`.
* `presetChips(...).highlight(i)` marks a chip without running it; `.setPresets(list)` swaps the set
  (e.g. one set per step).
* `Canvas2D`: `setOverlayText(text)` (throttled, use it instead of `overlay.textContent` in draw
  callbacks), `setSize({ aspect } | { height })` after construction. Resizes happen on the next frame,
  so creating a canvas in a hidden container no longer triggers ResizeObserver loop errors.
* `playback.setTime(t)` is throttled internally (~15 Hz DOM writes), so call it every frame freely.
* `physicsRunner({ ..., budgetMs: 8 })`: when a frame would exceed the CPU budget the rest of its
  sub-steps are dropped (slow motion instead of a frozen page).
* `Plot2D`: series without data are left out of the legend; stale series beyond `series.length`
  no longer affect auto-ranging; horizontal marker labels near the top are drawn below the line.
* 3D: `ScrewAxis3D({ size: 0.5, length })` scales the glyph for small robots and has `setColor(c)`;
  `makeTextSprite(text, color, fontPx, scale)` and `scene.addLabel(text, pos, color, scale)` size labels;
  `scene.addTube(points, { radius, color })` draws thick trails (WebGL lines are always 1 px).
* The simulation pane scrolls internally on desktop when it is taller than the viewport, but the first
  view should still show what matters without scrolling (~860 px at a 1400×950 window).
* Text rendering: combining-dot glyphs (θ̇, θ̈) render badly in canvas text, `<select>` options and plot
  legends; write `dθ/dt` or `ω` there and keep dots for KaTeX.
* Performance: `@/mr` `ForwardDynamics` costs ~0.3 ms per call for a 2R arm (it builds the mass matrix
  with n+1 Newton–Euler passes). For live planar 2R simulations use `planar2RAccel`; for 3D arms
  precompute trajectories or cap the integration rate.
* `scripts/smoke.mjs` runs Vite without HMR (other agents' edits no longer reload your page mid-run),
  takes screenshots with the viewport stretched to the page height, skips lessons without `index.ts`
  when no `--lesson` is given, and `--step-shots` saves the first view of every step as `<id>-s<k>.png`.

## 4. Math library (`@/mr`)

Function names mirror the open-source Modern Robotics library (so learners can cross-reference):
`VecToso3, so3ToVec, AxisAng3, MatrixExp3, MatrixLog3, RpToTrans, TransToRp, TransInv, VecTose3,
se3ToVec, Adjoint, ScrewToAxis, AxisAng6, MatrixExp6, MatrixLog6, ad, FKinBody, FKinSpace,
JacobianBody, JacobianSpace, IKinBody, IKinSpace, InverseDynamics, MassMatrix, VelQuadraticForces,
GravityForces, EndEffectorForces, ForwardDynamics, EulerStep, rk4Step, CubicTimeScaling,
QuinticTimeScaling, TrapezoidalTimeScaling, trapezoidScaling, JointTrajectory, ScrewTrajectory,
CartesianTrajectory,
ComputedTorque, SimulateControl`, plus `linalg` helpers (`matMul, matVec, transpose, inv, pinv, svd,
eigSym, cross, norm, ...`), planar helpers (`planar.ts`) and robot models (`robots.ts`: `planar2R,
planar3R, ur5, rrr3`). Planar Jacobians: `planarJacobian` / `planarTipJacobian` give the TIP-POINT Jacobian (use them for tip velocities, manipulability / force ellipses and τ = Jᵀ f with a force at the tip). When a lesson talks about the book's space or body Jacobian of a planar arm, use `planarSpaceJacobian` / `planarBodyJacobian` (rows `[ωz; vx; vy]`; column i = twist of joint i in {s} / {b}). 2D collision helpers live in `@/mr` too (`geometry2d.ts`: `segCircleIntersects, segPolygonIntersects, pointInPolygon, polylineCircleIntersects, polylinePolygonIntersects, distPointPolygon, rectPolygon, regularPolygon`). Read `src/mr/README.md` for conventions (screw axes are `[ω; v]`, Slist/Blist
are 6×n with axes as columns, matrices row-major `number[][]`).

## 4b. Working alongside other authors

Several lessons are built concurrently in the same checkout. `src/app/registry.ts` eagerly imports every
`src/lessons/*/meta.ts`, so a half-written `meta.ts` anywhere breaks the app for everyone: write `meta.ts`
in a single write, and make it valid TypeScript from the first save. If a smoke run fails with an error
that points at another lesson's files, wait ~30 s and run it again with `--lesson` limited to your ids.
The per-lesson design briefs live in `docs/curriculum.json` (and a readable overview in `docs/curriculum.md`).

The session scratchpad is shared by all agents: put every scratch file (scripts, screenshots, test files) in your own sub-folder named after your group or lesson, never at the scratchpad root.

## 5. Checklist before you report

```bash
npm run typecheck                       # no errors in your folder
npx vitest run src/lessons/<id>         # if you added tests
node scripts/smoke.mjs --lesson <id> --screens <scratch>/<your-folder>/smoke --step-shots   # must PASS; read every PNG
```

Then look at the screenshot: is the first thing the learner sees meaningful (not an empty canvas)?
Do the equations render? Is the text readable at ~700 px width?
