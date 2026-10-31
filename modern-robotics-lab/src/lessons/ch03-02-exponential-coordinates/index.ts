import * as THREE from 'three';
import type { Lesson, LessonContext, LessonHandle, LessonStep, QuizQuestion } from '@/types';
import {
  el, fmt, fmtVec, controlPanel, row, slider, angleSlider, toggle, button, select, matrixView, simTitle, kv,
  presetChips, mathEl, playback, setThemedColor, Plot2D, Scene3D, Arrow3D, createLoop, type MatrixHandle, type Marker,
} from '@/ui';
import {
  type Mat, type Vec, ProjectToSO3, RotX, cross, det, eulerZYX, eye, matMul, matSub, matVec, norm, normFro, transpose, vecAdd,
  vecScale, wrapTo2Pi, zeros,
} from '@/mr';
import { meta } from './meta';
import {
  type LogInfo, anglesFromAxis, axisFromAngles, eulerPath, expPath, foldedAngle, integratedOmega, logInfo, omegaFromDerivative,
  orbitPoints, rodriguesTerms, rotAxisAngle, rotatePointTerms, topMotion,
} from './expCoords';
import { AXIS_VARS, DashedSegments, Trail, Triad, TubePath, disposeSprite, label3D } from './view3d';
import { attachDrag3D } from './drag3d';

// ---------------------------------------------------------------------------
// Narrative
// ---------------------------------------------------------------------------

const steps: LessonStep[] = [
  {
    title: '旋轉中的物體：角速度向量',
    md: `物體在轉動時，每一瞬間都有一條轉軸 $\\hat\\omega$（單位向量）和一個轉速 $\\dot\\theta$。把兩者合成一個向量就是**角速度**：方向是轉軸（右手定則：四指沿轉動方向彎曲，拇指指向 $\\hat\\omega$），長度是轉速。

$$\\omega = \\hat\\omega\\,\\dot\\theta, \\qquad \\dot p = \\omega \\times p$$

物體上一點 $p$（從軸上一點量起）的速度是 $\\omega \\times p$：與軸和 $p$ 都垂直，離軸越遠越快，軸上的點完全不動。`,
    simHint: '按 ▶ 播放：三個軸的尖端沿虛線圓繞青色轉軸轉動，橘色箭頭是 $\\dot p = \\omega\\times p$。拖曳青色把手（或用方位角／仰角滑桿）換一條轉軸，再調轉速 $\\dot\\theta$，看紫色的 $\\omega$ 怎麼變。',
  },
  {
    title: 'ω_s 與 ω_b：同一支箭頭的兩組座標',
    md: `姿態 $R(t)$ 隨時間改變。$\\dot R$ 本身有九個數字，但乘上 $R^{\\T}$ 之後一定是反對稱矩陣，裡面只藏著一個三維向量：

$$\\dot R R^{\\T} = [\\omega_s], \\qquad R^{\\T}\\dot R = [\\omega_b]$$

$[\\omega]$ 表示把向量排成 $3\\times 3$ 反對稱矩陣，滿足 $[\\omega]p = \\omega \\times p$。$\\omega_s$ 與 $\\omega_b$ 是**同一支**角速度箭頭，分別寫成 $\\{s\\}$ 與 $\\{b\\}$ 的座標，所以 $\\omega_s = R\\,\\omega_b$。

為什麼一定是反對稱？把 $R^{\\T}R = I$ 對時間微分，得到 $\\dot R^{\\T}R + R^{\\T}\\dot R = 0$：$R^{\\T}\\dot R$ 加上自己的轉置等於零。`,
    simHint: '面板用有限差分即時算出 ṘRᵀ 與 RᵀṘ：兩者都是反對稱的。切換「投影到」{s} 或 {b}，看同一支紫色箭頭在兩組軸上的分量；再把運動換成「陀螺」，ω 會一直改變方向。',
  },
  {
    title: '繞固定軸轉 θ：Rodrigues 公式',
    md: `轉軸固定、轉速為 1 時，$\\dot R = [\\hat\\omega] R$ 的解是矩陣指數 $e^{[\\hat\\omega]\\theta}$。因為 $[\\hat\\omega]^3 = -[\\hat\\omega]$，無窮級數收成三項：

$$\\mathrm{Rot}(\\hat\\omega,\\theta) = e^{[\\hat\\omega]\\theta} = I + \\sin\\theta\\,[\\hat\\omega] + (1-\\cos\\theta)\\,[\\hat\\omega]^2$$

套到一個點 $p$ 上看：$\\sin\\theta\\,(\\hat\\omega\\times p)$ 沿圓的切線往前推，$(1-\\cos\\theta)\\,\\hat\\omega\\times(\\hat\\omega\\times p)$ 再朝軸心拉回，兩段頭尾相接，正好把點送到圓上轉過 $\\theta$ 的位置。

另外，$\\theta$ 每增加 $2\\pi$，兩個係數都回到原值，$R$ 完全不變：同一個 $R$ 對應無限多個 $\\theta$，這正是下一步反解時要面對的問題。`,
    simHint: '拖動 θ：橘色（sin 項）與紫色（1 − cos 項）箭頭頭尾相接，把追蹤點從灰點送到圓上。面板上 R 旁邊的兩個矩陣就是這兩項：sin 項反對稱、(1 − cos) 項對稱。關掉其中一項：框架變形，det R 與 ‖RᵀR − I‖ 立刻偏離。',
  },
  {
    title: '反向：矩陣對數',
    md: `反過來，給定 $R$ 要找回 $(\\hat\\omega, \\theta)$。跡（對角線和）只和角度有關，反對稱部分則指出轉軸：

$$\\theta = \\cos^{-1}\\!\\Big(\\frac{\\tr R - 1}{2}\\Big), \\qquad [\\hat\\omega] = \\frac{R - R^{\\T}}{2\\sin\\theta}$$

注意 $\\tr R$ 只告訴你轉了多少，對轉軸一無所知；轉軸的資訊全部藏在反對稱部分 $R - R^{\\T} = 2\\sin\\theta\\,[\\hat\\omega]$ 裡（Rodrigues 公式中的 $I$ 與 $[\\hat\\omega]^2$ 項都是對稱的，相減就消掉了）。$\\cos^{-1}$ 只給出 $[0, \\pi]$ 的角度，所以對數永遠挑「較短的轉法」：繞 $\\hat\\omega$ 轉 220° 和繞 $-\\hat\\omega$ 轉 140° 是同一個 $R$，$\\log R$ 回傳的是後者。`,
    simHint: '把 θ 從 0 拉到 360°：下方圖中 θ_out 在 180° 折返，紫色的 log 軸同時翻到 −ω̂。紅色粗弧是 x_b 實際轉過的 θ_in，紫色粗弧是 log 描述的「較短的路」。也可以把「輸入」切成「自由旋轉」，直接在 3D 畫面裡拖曳物體。',
  },
  {
    title: 'θ = π 與 θ = 0 的特殊情況',
    md: `上面的公式在 $\\sin\\theta = 0$ 時失效，分兩種情況：

- $\\theta = 0$：$R = I$。繞任何軸轉 0° 都一樣，**軸沒有定義**，$\\log R = 0$。
- $\\theta = \\pi$：軸存在，但 $R - R^{\\T} = 0$ 不再提供資訊，而且 $\\pm\\hat\\omega$ 給出同一個 $R$。這時改用對稱部分：

$$\\theta = \\pi: \\qquad \\hat\\omega\\hat\\omega^{\\T} = \\tfrac{1}{2}\\,(R + I)$$

可靠的程式會分支處理；畫面左上的標籤會告訴你目前走的是哪一條。`,
    simHint: '按「θ = π」：兩支紫色箭頭 ±ω̂ 都是合法答案。再按「θ = 179°」「θ = 0.001 rad」「θ = 0」，看左上角的標籤與 log 軸如何變化。',
  },
  {
    title: '指數座標不是 Euler 角',
    md: `指數座標 $\\hat\\omega\\theta$ 也是三個數字，但和 ZYX Euler 角 $(\\alpha, \\beta, \\gamma)$ 完全不同：前者是「繞一條軸轉一次」，後者是「依序繞三條座標軸各轉一次」。同一個 $R$，兩組數字不同，經過的姿態也不同。

另一個陷阱：三維中把角速度對時間積分，$\\int \\omega\\,dt$ **不會**得到指數座標，因為轉動不能交換順序；只有轉軸始終不變時兩者才相等。下一課把平移也加進來，轉軸就升級成螺旋軸（screw axis）。`,
    simHint: '按「▶ 一步：繞 ω̂」與「▶ 三步：ZYX」：終點相同、軌跡不同（青色 vs 灰色）。比較面板裡兩條路徑的 ∫ω_s dt 和 log R。勾選「預告：螺旋運動」看邊轉邊平移。',
  },
];

const quiz: QuizQuestion[] = [
  {
    question: '某旋轉矩陣的跡 $\\tr R = 1$。它的旋轉角是多少？轉軸能唯一決定嗎？',
    options: ['$\\theta = 0$，軸沒有定義', '$\\theta = \\pi/2$，軸可以唯一決定', '$\\theta = \\pi$，$\\pm\\hat\\omega$ 都對', '$\\theta = \\pi/2$，但軸的正負號無法決定'],
    answer: 1,
    explanation: '$\\tr R = 1 + 2\\cos\\theta = 1$ 得 $\\cos\\theta = 0$，$\\theta = \\pi/2$。此時 $\\sin\\theta = 1$，$[\\hat\\omega] = (R - R^{\\T})/2$ 完全確定，連正負號都有。只有 $\\theta = 0$（軸任意）與 $\\theta = \\pi$（$\\pm\\hat\\omega$ 都對）才不唯一。',
  },
  {
    question: '無人機上的陀螺儀量到的是物體座標的角速度 $\\omega_b$。要在世界座標 $\\{s\\}$ 的畫面中畫出這支角速度箭頭，該用哪個向量？',
    options: ['$R_{sb}\\,\\omega_b$', '$R_{sb}^{\\T}\\,\\omega_b$', '直接用 $\\omega_b$，角速度與座標系無關', '$[\\omega_b]\\,R_{sb}$'],
    answer: 0,
    explanation: '$\\omega_b$ 與 $\\omega_s$ 是同一支箭頭的兩組座標。從 $\\{b\\}$ 座標換到 $\\{s\\}$ 座標要左乘 $R_{sb}$（下標 $sb$ 與 $b$ 相消）：$\\omega_s = R_{sb}\\,\\omega_b$。箭頭本身沒變，但數字變了，所以不能把 $\\omega_b$ 的數字直接當成 $\\{s\\}$ 座標。',
  },
  {
    question: '指數座標 $\\hat\\omega\\theta = (0, 0, 3\\pi/2)$ 表示繞 $\\hat z$ 轉 270°。對這個 $R$ 取 $\\log$，會得到哪一組指數座標？',
    options: ['$(0, 0, \\pi/2)$', '$(0, 0, -\\pi/2)$', '$(0, 0, 3\\pi/2)$', '$(0, 0, -3\\pi/2)$'],
    answer: 1,
    explanation: '繞 $\\hat z$ 轉 270° 等於繞 $\\hat z$ 轉 −90°，也就是繞 $-\\hat z$ 轉 90°。$\\log$ 回傳的角度落在 $[0, \\pi]$，所以 $\\hat\\omega = (0, 0, -1)$、$\\theta = \\pi/2$，合起來是 $(0, 0, -\\pi/2)$。這正是模擬中 θ 超過 180° 時 log 軸翻面的現象。',
  },
  {
    question: '物體先以 $\\omega_s = (1, 0, 0)$ rad/s 轉 1 秒，再以 $\\omega_s = (0, 1, 0)$ rad/s 轉 1 秒，所以 $\\int \\omega_s\\,dt = (1, 1, 0)$。最終姿態的指數座標 $\\log R$ 是？',
    options: ['正好是 $(1, 1, 0)$', '約 $(0.91, 0.91, -0.50)$，和積分結果不同', '是 $(1, 1, 0)$ 換到 $\\{b\\}$ 座標後的結果', '$\\tfrac{1}{\\sqrt 2}(1, 1, 0)$'],
    answer: 1,
    explanation: '第二段繞的是固定軸，所以 $R = \\mathrm{Rot}(\\hat y, 1)\\,\\mathrm{Rot}(\\hat x, 1)$。轉動不能交換，$\\log R \\approx (0.913, 0.913, -0.499)$，多出一個 $z$ 分量，總角度約 1.38 rad 而不是 $\\sqrt2 \\approx 1.41$ rad。只有轉軸固定不變時，$\\int \\omega\\,dt$ 才等於 $\\hat\\omega\\theta$。',
  },
];

// ---------------------------------------------------------------------------
// Simulation constants
// ---------------------------------------------------------------------------

const DEG = Math.PI / 180;
const TWO_PI = 2 * Math.PI;
/** Radius of the sphere on which the ω̂ handle can be dragged. */
const HANDLE_R = 1.32;
/** Display scales: ω arrow length per rad/s, tip-velocity arrow length per m/s. */
const W_SCALE = 0.75;
/** The top spins fast (|ω| ≈ 2.8 rad/s), so its ω arrow is drawn shorter. */
const W_SCALE_TOP = 0.42;
const V_SCALE = 0.4;
const TOP = { precession: 0.7, spin: 2.2, tilt: 35 * DEG };
/** Tilted start pose for step 2: keeps all axes readable and makes ω_b differ clearly from ω_s. */
const R0_TILT = RotX(70 * DEG);
const PITCH = 0.12;
const PATH_T = 2.4;
const FULL = { sin: true, cos: true };
const PLOT_TITLE = 'log 回傳的角度 θ_out（紫）；灰色虛線：若不折返';
const ZERO3: Mat = zeros(3, 3);

type Input = 'axis' | 'free';
type Motion = 'axis' | 'top';
type Proj = 's' | 'b';

const CSS = `
.c302 .c302-eq { display: flex; flex-wrap: wrap; align-items: center; gap: 0.25rem 0.5rem; margin: 0.35rem 0; font-size: 0.85rem; }
.c302 .c302-eq .mr-matrix { margin: 0; }
.c302 .c302-name { font-size: 0.82rem; font-weight: 600; white-space: nowrap; }
.c302 .c302-pill { position: absolute; left: 0.5rem; top: 0.4rem; font-size: 0.75rem; padding: 0.1rem 0.55rem; border-radius: 999px; pointer-events: none; max-width: calc(100% - 1rem); z-index: 1; }
.c302 .c302-pill.ok { background: var(--ok-soft); color: var(--ok); }
.c302 .c302-pill.warn { background: var(--warn-soft); color: var(--warn); }
.c302 .c302-pill.bad { background: var(--danger-soft); color: var(--danger); }
.c302 .c302-pill.info { background: var(--accent-soft); color: var(--accent); }
.c302 .c302-hint { position: absolute; right: 0.5rem; bottom: 0.3rem; font-size: 0.72rem; color: var(--fg-muted); pointer-events: none; text-align: right; }
.c302 .mr-kv { margin: 0.2rem 0; }
.c302 .mr-plot { margin-top: 0.2rem; }
.c302 .c302-capm { display: inline-flex; flex-direction: column; align-items: center; gap: 0.05rem; }
.c302 .c302-capm .mr-matrix { margin: 0; }
.c302 .c302-capm .mr-mlabel { display: none; }
.c302 .c302-cap { font-size: 0.78rem; line-height: 1.15; color: var(--fg-muted); }
.c302 .c302-capm.off { opacity: 0.4; }
`;

// ---------------------------------------------------------------------------
// mount
// ---------------------------------------------------------------------------

function mount(host: HTMLElement, ctx: LessonContext): LessonHandle {
  const S = {
    scenario: -1,
    az: 60 * DEG,
    el: 45 * DEG,
    theta: 70 * DEG,
    rate: 1,
    input: 'axis' as Input,
    Rfree: eye(3),
    dragObject: true,
    R0: eye(3),
    motion: 'axis' as Motion,
    t: 0,
    proj: 'b' as Proj,
    terms: { sin: true, cos: true },
    track: 0,
    helix: false,
  };
  let playing = false;
  let pathAnim: { kind: 'exp' | 'euler'; u: number } | null = null;
  let topPts: Vec[][] = [[], [], []];

  const root = el('div', { class: 'c302' });
  root.appendChild(el('style', {}, CSS));
  host.appendChild(root);
  simTitle(root, '繞軸旋轉：角速度、exp 與 log');
  const chips = presetChips(root, SCENARIOS.map((sc, i) => ({ label: sc.label, title: sc.title, apply: () => applyScenario(i) })));

  // ---- 3D scene ----
  // phones get a taller 3D view (the pane is full width there and not sticky)
  const narrow = host.clientWidth > 0 && host.clientWidth < 520;
  const scene = new Scene3D(root, { aspect: narrow ? 1.25 : 1.7, grid: 3, axes: false, cameraPos: [2.85, -1.33, 1.75], target: [0.05, 0.05, 0.2], autoStart: false });
  const pill = el('div', { class: 'c302-pill info' });
  const hint3d = el('div', { class: 'c302-hint' });
  scene.el.append(pill, hint3d);

  const triS = new Triad(scene, { radius: 0.009, head: 0.08, opacity: 0.6, names: ['x_s', 'y_s', 'z_s'], labelVar: '--fg-muted', labelScale: 0.65 });
  const triB = new Triad(scene, { radius: 0.024, head: 0.12, names: ['x_b', 'y_b', 'z_b'] });
  // the rotation axis ω̂ with its drag handle
  const axisLine = new DashedSegments(scene, '--series-6', 1, { dash: 0.08, gap: 0.05 });
  const axisArrow = new Arrow3D('--series-6', 0.018, 0.11);
  scene.scene.add(axisArrow.group);
  const handleMesh = scene.addSphere(0.07, '--series-6');
  const axisLabel = label3D(scene, 'ω̂', '--series-6', 0.85);
  const guideMat = new THREE.MeshBasicMaterial({ transparent: true, opacity: 0.08, depthWrite: false });
  setThemedColor(guideMat, '--series-6');
  const guide = new THREE.Mesh(new THREE.SphereGeometry(HANDLE_R, 40, 24), guideMat);
  guide.visible = false;
  scene.scene.add(guide);
  // angular velocity and tip velocities
  const wArrow = new Arrow3D('--series-5', 0.026, 0.13);
  scene.scene.add(wArrow.group);
  const wLabel = label3D(scene, 'ω', '--series-5', 0.85);
  const velArrows = [0, 1, 2].map(() => { const a = new Arrow3D('--series-4', 0.012, 0.07); scene.scene.add(a.group); return a; });
  const orbits = AXIS_VARS.map((v) => new Trail(scene, v, { dashed: true, opacity: 0.45, max: 100 }));
  const arcs = AXIS_VARS.map((v) => new Trail(scene, v, { opacity: 0.9, max: 200 }));
  const topTrails = AXIS_VARS.map((v) => new Trail(scene, v, { opacity: 0.55, max: 1600 }));
  const wProj = new DashedSegments(scene, '--series-5', 3);
  const wFeet = AXIS_VARS.map((v) => scene.addSphere(0.03, v));
  // Rodrigues decomposition of one tracked point
  const rCircle = new Trail(scene, '--fg-muted', { dashed: true, max: 100 });
  const rRadii = new DashedSegments(scene, '--fg-muted', 2, { opacity: 0.8 });
  const rCenter = scene.addSphere(0.03, '--fg-muted');
  const rStart = scene.addSphere(0.04, '--fg-muted');
  const rEnd = scene.addSphere(0.045, '--fg');
  const rSin = new Arrow3D('--series-4', 0.016, 0.08);
  const rCos = new Arrow3D('--series-5', 0.016, 0.08);
  scene.scene.add(rSin.group, rCos.group);
  // matrix logarithm: its axis (both ±ω̂ at θ = π) and, as thick arcs, the way the x_b tip actually went
  // (θ_in about ω̂, red) versus the way log R describes it (θ_out ≤ π about the returned axis, purple)
  const logLine = new DashedSegments(scene, '--series-5', 1, { dash: 0.06, gap: 0.05 });
  const logArrows = [0, 1].map(() => { const a = new Arrow3D('--series-5', 0.02, 0.12); scene.scene.add(a.group); return a; });
  const logLabel = label3D(scene, 'log R', '--series-5', 0.75);
  const longArc = new TubePath(scene, '--axis-x', 0.008, 0.8);
  const logArc = new TubePath(scene, '--series-5', 0.014);
  // two paths from I to R (tubes, so they stay visible on small screens)
  const pathExp = [0, 1, 2].map(() => new TubePath(scene, '--series-6', 0.008));
  const pathEul = [0, 1, 2].map(() => new TubePath(scene, '--series-8', 0.008));
  let pathCache: { key: string; pe: Vec[][]; pu: Vec[][] } | null = null;
  const sprites = [axisLabel, wLabel, logLabel];

  const drag = attachDrag3D(scene, {
    handlePos: () => (axisVisible() ? new THREE.Vector3(...axisVec()).multiplyScalar(HANDLE_R) : null),
    radius: HANDLE_R,
    onAxis: (d) => {
      const a = anglesFromAxis(d);
      S.az = a.az; S.el = a.el;
      azS.set(S.az, { silent: true });
      elS.set(S.el, { silent: true });
      resetTopTrails();
      update();
    },
    objectMode: () => S.input === 'free' && S.dragObject && S.scenario === 3,
    onRotate: (axis, angle) => { S.Rfree = matMul(rotAxisAngle(axis, angle), S.Rfree); update(); },
    onDragState: (k) => {
      guide.visible = k === 'axis';
      // a long arcball drag multiplies many small rotations: snap the product back onto SO(3) when it ends
      if (k === null) S.Rfree = ProjectToSO3(S.Rfree);
      update();
    },
  });

  // ---- controls ----
  const axisPanel = controlPanel(root, { compact: true });
  const azS = angleSlider(axisPanel, { label: 'ω̂ 方位角 φ', value: S.az, onInput: (v) => { S.az = v; resetTopTrails(); update(); } });
  const elS = angleSlider(axisPanel, { label: 'ω̂ 仰角 λ', value: S.el, min: -Math.PI / 2, max: Math.PI / 2, onInput: (v) => { S.el = v; resetTopTrails(); update(); } });
  const thS = angleSlider(axisPanel, { label: 'θ（繞 ω̂ 轉的角度）', value: S.theta, min: 0, max: TWO_PI, onInput: (v) => { S.theta = v; stopPath(); update(); } });

  const tPanel = controlPanel(root);
  const tRow = row(tPanel);
  const pb = playback(tRow, {
    onPlay: () => { playing = true; stopPath(); loop.start(); },
    onPause: () => { playing = false; if (!pathAnim) loop.stop(); },
    onReset: () => { if (S.motion === 'top') { S.t = 0; resetTopTrails(); } else { S.theta = 0; thS.set(0, { silent: true }); } update(); },
    onSpeed: (s) => { loop.speed = s; },
  });
  const rateS = slider(tRow, { label: '轉速 dθ/dt', min: -2, max: 2, step: 0.05, value: S.rate, unit: 'rad/s', onInput: (v) => { S.rate = v; update(); } });
  rateS.el.style.flex = '1 1 160px';

  // step 2: motion and projection
  const box2 = el('div', { class: 'mr-row' });
  tRow.appendChild(box2);
  const motionSel = select<Motion>(box2, {
    options: [{ value: 'axis', label: '運動：繞固定軸' }, { value: 'top', label: '運動：陀螺（進動＋自轉）' }],
    value: S.motion,
    onChange: (m) => { S.motion = m; S.t = 0; resetTopTrails(); syncVisibility(); update(); },
  });
  const projSel = select<Proj>(box2, {
    options: [{ value: 'b', label: '投影到 {b} 的軸' }, { value: 's', label: '投影到 {s} 的軸' }],
    value: S.proj,
    onChange: (p) => { S.proj = p; update(); },
  });
  const tiltTog = toggle(box2, { label: '起始姿態 R₀ 傾斜', value: false, onChange: (v) => { S.R0 = v ? R0_TILT : eye(3); update(); } });

  // step 3: Rodrigues terms
  const box3 = el('div', { class: 'mr-row' });
  tRow.appendChild(box3);
  const sinTog = toggle(box3, { label: 'sin θ [ω̂] 項', value: true, onChange: (v) => { S.terms.sin = v; update(); } });
  const cosTog = toggle(box3, { label: '(1 − cos θ)[ω̂]² 項', value: true, onChange: (v) => { S.terms.cos = v; update(); } });
  const trackSel = select<'0' | '1' | '2'>(box3, {
    options: [{ value: '0', label: '追蹤 x_b 尖端' }, { value: '1', label: '追蹤 y_b 尖端' }, { value: '2', label: '追蹤 z_b 尖端' }],
    value: '0',
    onChange: (v) => { S.track = Number(v); update(); },
  });

  // step 4: inverse problem
  const box4 = el('div', { class: 'mr-row' });
  tRow.appendChild(box4);
  const inputSel = select<Input>(box4, {
    options: [{ value: 'axis', label: '輸入：軸角 (ω̂, θ)' }, { value: 'free', label: '輸入：自由旋轉（拖曳物體）' }],
    value: S.input,
    onChange: (v) => setInput(v),
  });
  const dragTog = toggle(box4, { label: '拖曳畫面＝轉物體', value: S.dragObject, onChange: (v) => { S.dragObject = v; update(); } });
  button(box4, { label: '隨機姿態', small: true, onClick: () => { setInput('free'); S.Rfree = randomR(); update(); } });

  // step 5: special angles
  const box5 = el('div', { class: 'mr-row' });
  tRow.appendChild(box5);
  const special: [string, number][] = [['θ = π', Math.PI], ['θ = 179°', 179 * DEG], ['θ = 0.001 rad', 0.001], ['θ = 0', 0]];
  for (const [label, v] of special) button(box5, { label, small: true, onClick: () => setTheta(v) });

  // step 6: two paths
  const box6 = el('div', { class: 'mr-row' });
  tRow.appendChild(box6);
  button(box6, { label: '▶ 一步：繞 ω̂', small: true, variant: 'primary', title: '沿 exp(u·log R) 從 I 轉到 R', onClick: () => startPath('exp') });
  button(box6, { label: '▶ 三步：ZYX', small: true, title: '先繞 ẑ_b 轉 α，再繞 ŷ_b 轉 β，最後繞 x̂_b 轉 γ', onClick: () => startPath('euler') });
  const helixTog = toggle(box6, { label: '預告：螺旋運動（邊轉邊平移）', value: false, onChange: (v) => { S.helix = v; update(); } });

  // ---- readouts ----
  const readRow = el('div', { class: 'c302-eq' });
  root.appendChild(readRow);
  const mvR = labelled(readRow, 'R =', 3);
  // step 3: R = I + sin θ [ω̂] + (1 − cos θ)[ω̂]², the two terms as numbers (captions on top keep the row narrow)
  const termSpan = el('span', { class: 'c302-eq', style: { margin: '0' } });
  readRow.appendChild(termSpan);
  termSpan.appendChild(mathEl('= I \\,+'));
  const sinM = captioned(termSpan, '\\sin\\theta\\,[\\hat\\omega]');
  termSpan.appendChild(mathEl('+'));
  const cosM = captioned(termSpan, '(1-\\cos\\theta)\\,[\\hat\\omega]^2');
  // step 2: [ω_s], [ω_b]
  const omegaSpan = el('span', { class: 'c302-eq', style: { margin: '0' } });
  readRow.appendChild(omegaSpan);
  const mvWs = labelled(omegaSpan, '\\dot R R^{\\T} = [\\omega_s] =', 2);
  const mvWb = labelled(omegaSpan, 'R^{\\T}\\dot R = [\\omega_b] =', 2);
  // step 3: the vector chain p + sin-part + cos-part = p'
  const chainSpan = el('span', { class: 'c302-eq', style: { margin: '0' } });
  readRow.appendChild(chainSpan);
  const mvP = labelled(chainSpan, 'p', 2);
  chainSpan.appendChild(mathEl('+'));
  const mvSinP = labelled(chainSpan, '\\sin\\theta\\,(\\hat\\omega\\times p)', 2);
  chainSpan.appendChild(mathEl('+'));
  const mvCosP = labelled(chainSpan, '(1-\\cos\\theta)\\,\\hat\\omega\\times(\\hat\\omega\\times p)', 2);
  chainSpan.appendChild(mathEl('='));
  const mvEnd = labelled(chainSpan, "p'", 2);

  const kv1 = kv(readRow, ['ω = ω̂ dθ/dt（rad/s）', '尖端速率 |ω × p|']);
  const kv2 = kv(readRow, ['ω_s（{s} 座標）', 'ω_b（{b} 座標）', 'R ω_b（應等於 ω_s）']);
  const kv3 = kv(readRow, ['det R', '‖RᵀR − I‖', '|p′ − 圓心| vs 半徑']);
  const kvLog = kv(readRow, ['tr R', 'θ = cos⁻¹((trR − 1)/2)', 'log 的軸 ω̂', 'ω̂θ（指數座標）']);
  const kvEul = kv(readRow, ['ZYX Euler (α, β, γ)', 'log R = ω̂θ', '∫ω_s dt：一步（繞 ω̂）', '∫ω_s dt：三步（ZYX）']);

  // colours are CSS variable names: Plot2D resolves them at draw time, so a theme switch just works
  let plotTitle = PLOT_TITLE;
  const plot = new Plot2D(root, {
    title: PLOT_TITLE,
    xLabel: 'θ_in（度）',
    yLabel: 'θ_out（度）',
    series: [
      { name: 'θ_out = ‖log R‖', color: '--series-5', width: 2.2 },
      { name: '若不折返：θ_out = θ_in', color: '--fg-muted', dash: [4, 4], width: 1.2 },
    ],
    xRange: [0, 360],
    yRange: [0, 360],
    height: 150,
    legend: false,
  });
  {
    const xs: number[] = [], ys: number[] = [];
    for (let k = 0; k <= 360; k += 2) { xs.push(k); ys.push(foldedAngle(k * DEG).theta / DEG); }
    plot.setXY(0, xs, ys);
    plot.setXY(1, [0, 360], [0, 360]);
  }

  // ---- helpers ----
  function axisVec(): Vec { return axisFromAngles(S.az, S.el); }
  function axisVisible(): boolean { return S.input === 'axis' && S.motion === 'axis' && S.scenario !== 1 && !pathAnim; }
  function terms() { return S.scenario === 2 ? S.terms : FULL; }

  /** R defined by the inputs (before any path animation). */
  function targetR(): Mat {
    if (S.input === 'free') return S.Rfree;
    if (S.motion === 'top') return topMotion(S.t, TOP).R;
    return matMul(rodriguesTerms(axisVec(), S.theta, terms()).R, S.R0);
  }

  function randomR(): Mat {
    const a = axisFromAngles((Math.random() * 2 - 1) * Math.PI, Math.asin(Math.random() * 2 - 1));
    return rotAxisAngle(a, Math.random() * Math.PI);
  }

  function setInput(v: Input): void {
    if (v === 'free' && S.input !== 'free') S.Rfree = targetR();
    S.input = v;
    inputSel.set(v, { silent: true });
    syncVisibility();
    update();
  }

  function setTheta(v: number): void {
    stopPath();
    if (S.input !== 'axis') setInput('axis');
    S.theta = v;
    thS.set(v, { silent: true });
    update();
  }

  function resetTopTrails(): void {
    topPts = [[], [], []];
    topTrails.forEach((t) => t.set([]));
  }

  function startPath(kind: 'exp' | 'euler'): void {
    playing = false;
    pb.setPlaying(false, { silent: true });
    pathAnim = { kind, u: 0 };
    loop.start();
  }

  function stopPath(): void {
    if (!pathAnim) return;
    pathAnim = null;
    if (!playing) loop.stop();
  }

  /** Tips of the three axes along the single-axis path and along the ZYX path (memoised by `key`). */
  function pathPoints(Rt: Mat, lg: LogInfo, key: string): { pe: Vec[][]; pu: Vec[][] } {
    if (pathCache?.key === key) return pathCache;
    const N = 72;
    const pe: Vec[][] = [[], [], []], pu: Vec[][] = [[], [], []];
    for (let k = 0; k <= N; k++) {
      const u = k / N;
      const Re = expPath(Rt, u), Ru = eulerPath(Rt, u);
      const off = S.helix && lg.omghat ? vecScale(lg.omghat, PITCH * lg.theta * u) : [0, 0, 0];
      for (let j = 0; j < 3; j++) {
        pe[j].push(vecAdd([Re[0][j], Re[1][j], Re[2][j]], off));
        pu[j].push([Ru[0][j], Ru[1][j], Ru[2][j]]);
      }
    }
    pathCache = { key, pe, pu };
    return pathCache;
  }

  const loop = createLoop((rawDt) => {
    // createLoop can report a slightly negative dt on its first frame (rAF timestamp < start time)
    const dt = Math.max(0, rawDt);
    if (pathAnim) {
      // the path demo (step 6) has no speed control of its own: undo the playback speed chosen in steps 1–2
      pathAnim.u = Math.min(1, pathAnim.u + dt / (loop.speed > 0 ? loop.speed : 1) / PATH_T);
      if (pathAnim.u >= 1) { pathAnim = null; if (!playing) loop.stop(); }
    } else if (playing) {
      if (S.input === 'free') { /* nothing moves in free mode */ } else if (S.motion === 'top') {
        S.t += dt;
        const R = topMotion(S.t, TOP).R;
        for (let j = 0; j < 3; j++) { topPts[j].push([R[0][j], R[1][j], R[2][j]]); if (topPts[j].length > 1500) topPts[j].shift(); }
        topTrails.forEach((tr, j) => tr.set(topPts[j]));
        pb.setTime(S.t);
      } else {
        S.theta = wrapTo2Pi(S.theta + S.rate * dt);
        thS.set(S.theta, { silent: true });
        pb.setTime(loop.time);
      }
    } else {
      loop.stop();
    }
    update();
  });

  // ---- main update ----
  function update(): void {
    const sc = S.scenario;
    const w = axisVec();
    const Rt = targetR();
    let R = Rt;
    let origin: Vec = [0, 0, 0];
    if (pathAnim) R = pathAnim.kind === 'exp' ? expPath(Rt, pathAnim.u) : eulerPath(Rt, pathAnim.u);
    const lg: LogInfo | null = sc === 2 ? null : logInfo(Rt);
    if (sc === 5 && S.helix && lg) {
      const u = pathAnim ? (pathAnim.kind === 'exp' ? pathAnim.u : 0) : 1;
      if (lg.omghat) origin = vecScale(lg.omghat, PITCH * lg.theta * u);
    }

    triS.set(eye(3), 1.25);
    triB.set(R, 1, origin);
    // in the Rodrigues step only the tracked axis stays solid, so the head-to-tail construction is readable
    for (let j = 0; j < 3; j++) triB.setArrowOpacity(j, sc === 2 && j !== S.track ? 0.22 : 1);

    // axis ω̂ and its handle
    const showAxis = axisVisible();
    axisLine.setVisible(showAxis);
    place(axisArrow, [0, 0, 0], vecScale(w, 1.05), showAxis && sc >= 2);
    handleMesh.visible = showAxis;
    axisLabel.visible = showAxis;
    if (showAxis) {
      axisLine.set([[vecScale(w, -1.5), vecScale(w, 1.5)]]);
      handleMesh.position.set(w[0] * HANDLE_R, w[1] * HANDLE_R, w[2] * HANDLE_R);
      axisLabel.position.set(w[0] * (HANDLE_R + 0.17), w[1] * (HANDLE_R + 0.17), w[2] * (HANDLE_R + 0.17));
    }

    // angular velocity (steps 1–2)
    const velOn = (sc === 0 || sc === 1) && S.input === 'axis';
    let om: ReturnType<typeof omegaFromDerivative> | null = null;
    if (velOn) {
      om = omegaFromDerivative((tau) => (S.motion === 'top' ? topMotion(S.t + tau, TOP).R : matMul(rotAxisAngle(w, S.theta + S.rate * tau), S.R0)), 0);
      const ws = om.wS;
      const wsc = S.motion === 'top' ? W_SCALE_TOP : W_SCALE;
      place(wArrow, [0, 0, 0], vecScale(ws, wsc), true);
      const L = norm(ws) * wsc;
      wLabel.visible = L > 0.05;
      const k = (L + 0.16) / Math.max(L, 1e-9);
      wLabel.position.set(ws[0] * wsc * k, ws[1] * wsc * k, ws[2] * wsc * k);
      for (let j = 0; j < 3; j++) {
        const p = [R[0][j], R[1][j], R[2][j]];
        place(velArrows[j], p, vecAdd(p, vecScale(cross(ws, p), V_SCALE)), sc === 0);
      }
    } else {
      wLabel.visible = false;
      wArrow.visible = false;
      velArrows.forEach((a) => { a.visible = false; });
    }

    // orbits and swept arcs of the three tips (fixed-axis motion)
    const arcsOn = (sc === 0 || sc === 1 || sc === 3 || sc === 4) && S.input === 'axis' && S.motion === 'axis' && !pathAnim;
    // in the log steps only the x_b tip is traced (as a thick arc), to compare with the log's short way round
    const onlyX = sc === 3 || sc === 4;
    for (let j = 0; j < 3; j++) {
      orbits[j].setVisible(arcsOn && (!onlyX || j === 0));
      arcs[j].setVisible(arcsOn && !onlyX);
      if (arcsOn) {
        const p0 = [S.R0[0][j], S.R0[1][j], S.R0[2][j]];
        orbits[j].set(orbitPoints(w, p0, 0, TWO_PI, 90));
        if (!onlyX) arcs[j].set(orbitPoints(w, p0, 0, S.theta, Math.max(2, Math.ceil(S.theta / (4 * DEG)))));
      }
    }
    const longOn = arcsOn && onlyX;
    longArc.setVisible(longOn);
    if (longOn) longArc.set(`${S.az},${S.el},${S.theta}`, () => orbitPoints(w, [1, 0, 0], 0, S.theta, Math.max(2, Math.ceil(S.theta / (3 * DEG)))));
    topTrails.forEach((t) => t.setVisible(sc === 1 && S.motion === 'top'));

    // projection of ω onto the axes of {s} or {b} (step 2)
    const projOn = sc === 1 && om != null;
    wProj.setVisible(projOn);
    wFeet.forEach((f) => { f.visible = projOn; });
    if (projOn && om) {
      const tip = vecScale(om.wS, S.motion === 'top' ? W_SCALE_TOP : W_SCALE);
      const F = S.proj === 's' ? eye(3) : R;
      const segs: [Vec, Vec][] = [0, 1, 2].map((j) => {
        const a = [F[0][j], F[1][j], F[2][j]];
        const c = tip[0] * a[0] + tip[1] * a[1] + tip[2] * a[2];
        return [tip, vecScale(a, c)];
      });
      wProj.set(segs);
      segs.forEach((s, j) => wFeet[j].position.set(s[1][0], s[1][1], s[1][2]));
    }

    // Rodrigues decomposition (step 3)
    const rodOn = sc === 2 && S.input === 'axis';
    rCircle.setVisible(rodOn);
    rSin.visible = rodOn;
    rCos.visible = rodOn;
    rRadii.setVisible(rodOn);
    [rCenter, rStart, rEnd].forEach((m) => { m.visible = rodOn; });
    let pr: ReturnType<typeof rotatePointTerms> | null = null;
    if (rodOn) {
      const p = [0, 0, 0];
      p[S.track] = 1;
      pr = rotatePointTerms(w, S.theta, p, S.terms);
      rCircle.set(orbitPoints(w, p, 0, TWO_PI, 90));
      rRadii.set([[pr.center, pr.p], [pr.center, pr.end]]);
      rCenter.position.set(pr.center[0], pr.center[1], pr.center[2]);
      rStart.position.set(p[0], p[1], p[2]);
      rEnd.position.set(pr.end[0], pr.end[1], pr.end[2]);
      place(rSin, pr.p, vecAdd(pr.p, pr.sinPart), S.terms.sin);
      place(rCos, pr.mid, vecAdd(pr.mid, pr.cosPart), S.terms.cos);
    }

    // matrix logarithm (steps 4–5)
    const logOn = (sc === 3 || sc === 4) && lg != null && !pathAnim;
    logLine.setVisible(logOn && lg!.omghat != null);
    logArrows.forEach((a) => { a.visible = false; });
    logLabel.visible = false;
    logArc.setVisible(logOn && lg?.omghat != null);
    if (logOn && lg && lg.omghat) {
      const a = lg.omghat;
      logLine.set([[vecScale(a, -1.4), vecScale(a, 1.4)]]);
      place(logArrows[0], [0, 0, 0], vecScale(a, 0.85), true);
      place(logArrows[1], [0, 0, 0], vecScale(a, -0.85), lg.branch === 'pi');
      logLabel.visible = true;
      logLabel.position.set(a[0], a[1], a[2] + 0.12);
      // the short way round for the x_b tip: log R rotates I into R, so the arc starts at x̂_s
      logArc.set(lg.expc.map((x) => x.toFixed(6)).join(','), () => orbitPoints(a, [1, 0, 0], 0, lg.theta, 60));
    }

    // two paths from I to R (step 6); the tubes are rebuilt only when R or the helix option changes
    const pathsOn = sc === 5 && lg != null;
    for (let j = 0; j < 3; j++) { pathExp[j].setVisible(pathsOn); pathEul[j].setVisible(pathsOn && !S.helix); }
    if (pathsOn && lg) {
      const key = `${Rt.flat().map((x) => x.toFixed(6)).join(',')}|${S.helix}`;
      const pts = () => pathPoints(Rt, lg, key);
      for (let j = 0; j < 3; j++) {
        pathExp[j].set(key, () => pts().pe[j]);
        pathEul[j].set(key, () => pts().pu[j]);
      }
    }

    updatePill(lg);
    updateReadouts(R, Rt, lg, om, pr);
    scene.requestRender();
  }

  function updatePill(lg: LogInfo | null): void {
    const sc = S.scenario;
    let cls = 'info', text = '';
    if (sc === 0 || sc === 1) {
      text = S.motion === 'top' ? '陀螺：ω 的方向在 {s} 與 {b} 中都在改變' : `繞固定軸轉動：dθ/dt = ${fmt(S.rate, 2)} rad/s`;
    } else if (sc === 2) {
      const R = targetR();
      const err = normFro(matSub(matMul(transpose(R), R), eye(3)));
      if (err < 1e-9) { cls = 'ok'; text = '✓ 三項都在：R 正交、det R = 1'; } else { cls = 'bad'; text = `✗ 少了一項：det R = ${fmt(det(R), 3)}，RᵀR ≠ I（不是旋轉）`; }
    } else if ((sc === 3 || sc === 4) && lg) {
      if (lg.branch === 'identity') { cls = 'warn'; text = 'θ = 0：R = I，log R = 0，軸沒有定義'; }
      else if (lg.branch === 'pi') { cls = 'warn'; text = 'θ = π：R − Rᵀ = 0，改由 (R + I)/2 取軸；±ω̂ 都對'; }
      else if (lg.branch === 'nearPi') { cls = 'warn'; text = 'θ ≈ π：由對稱部分取軸，正負號由 R − Rᵀ 決定'; }
      else if (lg.theta < 0.01) { cls = 'warn'; text = `一般公式，但 sin θ = ${lg.sinTheta.toExponential(1)}：θ → 0 時軸越來越不確定`; }
      else {
        const f = S.input === 'axis' ? foldedAngle(S.theta) : null;
        cls = f?.flipped ? 'warn' : 'ok';
        text = f?.flipped ? `一般公式：θ_in > 180°，log 改走另一邊（−ω̂ 轉 ${fmt(lg.theta / DEG, 0)}°）` : '一般公式：ω̂ 取自 (R − Rᵀ)/(2 sin θ)';
      }
    } else if (sc === 5) {
      text = pathAnim ? (pathAnim.kind === 'exp' ? '一步：一直繞同一條軸 ω̂' : 'ZYX：依序繞 ẑ_b、ŷ_b、x̂_b') : '同一個 R，兩條不同的路徑';
      if (S.helix) text = `預告：繞 ω̂ 轉的同時沿 ω̂ 平移 h·θ（h = ${PITCH} m/rad）＝螺旋運動`;
    }
    pill.className = `c302-pill ${cls}`;
    pill.textContent = text;
    pill.style.display = text ? '' : 'none';
    hint3d.textContent = S.input === 'free' && S.dragObject && sc === 3 ? '拖曳畫面：轉動物體　右鍵拖曳：平移' : (axisVisible() ? '拖曳青色把手：改變 ω̂　拖曳空白處：轉視角' : '拖曳：轉視角　滾輪：縮放');
  }

  function updateReadouts(R: Mat, Rt: Mat, lg: LogInfo | null, om: ReturnType<typeof omegaFromDerivative> | null, pr: ReturnType<typeof rotatePointTerms> | null): void {
    const sc = S.scenario;
    // two decimals in the Rodrigues step, so that R and its two terms fit on one line
    mvR.set(R, { highlightCols: ['x', 'y', 'z'], digits: sc === 2 ? 2 : 3 });
    if (sc === 0 && om) {
      kv1.set('ω = ω̂ dθ/dt（rad/s）', fmtVec(om.wS, 2));
      const w = norm(om.wS);
      const d = [0, 1, 2].map((j) => norm(cross(om.wS, [R[0][j], R[1][j], R[2][j]])));
      kv1.set('尖端速率 |ω × p|', `x_b：${fmt(d[0], 2)}　y_b：${fmt(d[1], 2)}　z_b：${fmt(d[2], 2)}　（|ω| = ${fmt(w, 2)} rad/s）`);
    }
    if (sc === 1 && om) {
      mvWs.set(om.WS);
      mvWb.set(om.WB);
      kv2.set('ω_s（{s} 座標）', fmtVec(om.wS, 3));
      kv2.set('ω_b（{b} 座標）', fmtVec(om.wB, 3));
      kv2.set('R ω_b（應等於 ω_s）', `${fmtVec(matVec(R, om.wB), 3)}　✓　|ω_s| = |ω_b| = ${fmt(norm(om.wB), 3)}`);
    }
    if (sc === 2 && pr) {
      // the matrix terms (sin term skew-symmetric, (1 − cos) term symmetric); a switched-off term shows as zeros
      const T = rodriguesTerms(axisVec(), S.theta);
      sinM.mv.set(S.terms.sin ? T.sinTerm : ZERO3);
      cosM.mv.set(S.terms.cos ? T.cosTerm : ZERO3);
      sinM.box.classList.toggle('off', !S.terms.sin);
      cosM.box.classList.toggle('off', !S.terms.cos);
      mvP.set(pr.p);
      mvSinP.set(S.terms.sin ? pr.sinPart : [0, 0, 0]);
      mvCosP.set(S.terms.cos ? pr.cosPart : [0, 0, 0]);
      mvEnd.set(pr.end);
      kv3.set('det R', fmt(det(Rt), 4));
      kv3.set('‖RᵀR − I‖', fmt(normFro(matSub(matMul(transpose(Rt), Rt), eye(3))), 4));
      const dist = norm([pr.end[0] - pr.center[0], pr.end[1] - pr.center[1], pr.end[2] - pr.center[2]]);
      kv3.set('|p′ − 圓心| vs 半徑', `${fmt(dist, 3)} vs ${fmt(pr.radius, 3)}${Math.abs(dist - pr.radius) < 1e-6 ? '（在圓上）' : '（離開圓了）'}`);
    }
    if ((sc === 3 || sc === 4) && lg) {
      const tr = Rt[0][0] + Rt[1][1] + Rt[2][2];
      kvLog.set('tr R', fmt(tr, 4));
      kvLog.set('θ = cos⁻¹((trR − 1)/2)', `${fmt(lg.theta, 4)} rad（${fmt(lg.theta / DEG, 1)}°）`);
      kvLog.set('log 的軸 ω̂', lg.omghat ? `${lg.branch === 'pi' ? '±' : ''}${fmtVec(lg.omghat, 3)}` : '沒有定義（任何軸都行）');
      kvLog.set('ω̂θ（指數座標）', fmtVec(lg.expc, 3));
      const marks: Marker[] = [{ x: 180, label: 'π：軸翻面' }];
      if (S.input === 'axis') {
        const thIn = S.theta / DEG;
        const f = foldedAngle(S.theta);
        // grey dot: where θ_out would be without the fold; purple dot: what log R returns. The numbers go in the
        // title (a label next to the dot would sit on the curve itself for θ_in < 180°)
        if (f.flipped) marks.push({ x: thIn, y: thIn, color: '--fg-muted' });
        marks.push({ x: thIn, y: f.theta / DEG, color: '--series-5' });
        const narrowPlot = plot.el.clientWidth > 0 && plot.el.clientWidth < 480;
        const title = `θ_in = ${fmt(thIn, 0)}° → ${narrowPlot ? '' : 'log 回傳 '}θ_out = ${fmt(f.theta / DEG, 0)}°；灰色：若不折返`;
        if (title !== plotTitle) { plotTitle = title; plot.setOptions({ title }); }
      }
      plot.setMarkers(marks);
    }
    if (sc === 5 && lg) {
      const e = eulerZYX(Rt);
      kvEul.set('ZYX Euler (α, β, γ)', `(${e.angles.map((a) => fmt(a / DEG, 1)).join('°, ')}°)${e.gimbalLock ? '　萬向鎖' : ''}`);
      kvEul.set('log R = ω̂θ', `${fmtVec(lg.expc, 3)}（θ = ${fmt(lg.theta / DEG, 1)}°）`);
      const ie = integratedOmega(Rt, 'exp');
      const iu = integratedOmega(Rt, 'euler');
      const diff = norm(vecAdd(iu, vecScale(lg.expc, -1)));
      kvEul.set('∫ω_s dt：一步（繞 ω̂）', `${fmtVec(ie, 3)}　＝ log R ✓`);
      kvEul.set('∫ω_s dt：三步（ZYX）', `${fmtVec(iu, 3)}　${diff < 1e-6 ? '＝ log R' : `≠ log R（差 ${fmt(diff, 3)}）`}`);
    }
  }

  // ---- scenarios ----
  function syncVisibility(): void {
    const sc = S.scenario;
    const axisInput = S.input === 'axis';
    azS.el.style.display = axisInput && S.motion === 'axis' ? '' : 'none';
    elS.el.style.display = azS.el.style.display;
    thS.el.style.display = axisInput && S.motion === 'axis' ? '' : 'none';
    rateS.el.style.display = sc === 0 || sc === 1 ? '' : 'none';
    pb.el.style.display = sc <= 1 ? '' : 'none';
    box2.style.display = sc === 1 ? '' : 'none';
    box3.style.display = sc === 2 ? '' : 'none';
    box4.style.display = sc === 3 ? '' : 'none';
    dragTog.el.style.display = S.input === 'free' ? '' : 'none';
    box5.style.display = sc === 4 ? '' : 'none';
    box6.style.display = sc === 5 ? '' : 'none';
    mvR.el.style.display = sc === 1 ? 'none' : '';
    omegaSpan.style.display = sc === 1 ? '' : 'none';
    termSpan.style.display = sc === 2 ? '' : 'none';
    chainSpan.style.display = sc === 2 ? '' : 'none';
    kv1.el.style.display = sc === 0 ? '' : 'none';
    kv2.el.style.display = sc === 1 ? '' : 'none';
    kv3.el.style.display = sc === 2 ? '' : 'none';
    kvLog.el.style.display = sc === 3 || sc === 4 ? '' : 'none';
    kvEul.el.style.display = sc === 5 ? '' : 'none';
    // the θ_in → θ_out plot only makes sense for axis-angle input
    plot.el.style.display = (sc === 3 || sc === 4) && axisInput ? '' : 'none';
    const targets: HTMLElement[] = [scene.el, omegaSpan, chainSpan, plot.el, box5, box6];
    targets.forEach((t) => t.classList.remove('mr-focus'));
    targets[sc]?.classList.add('mr-focus');
  }

  function applyScenario(i: number): void {
    S.scenario = i;
    playing = false;
    pb.setPlaying(false, { silent: true });
    pathAnim = null;
    loop.stop();
    resetTopTrails();
    S.input = 'axis';
    S.motion = 'axis';
    S.R0 = eye(3);
    S.terms = { sin: true, cos: true };
    S.helix = false;
    // a generic axis that is easy to read from the default camera; the Rodrigues step uses one that faces
    // the camera more, so the circle the tracked point travels on is not too foreshortened
    S.az = 60 * DEG;
    S.el = 45 * DEG;
    if (i === 0) { S.theta = 70 * DEG; S.rate = 1; }
    else if (i === 1) { S.R0 = R0_TILT; S.theta = 40 * DEG; S.rate = 1; S.proj = 'b'; }
    else if (i === 2) { S.theta = 120 * DEG; S.track = 0; S.az = -70 * DEG; S.el = 70 * DEG; }
    else if (i === 3) { S.theta = 220 * DEG; }
    else if (i === 4) { S.theta = Math.PI; }
    else if (i === 5) { S.theta = 120 * DEG; S.az = -165 * DEG; S.el = 60 * DEG; }
    // sync controls silently
    thS.set(S.theta, { silent: true });
    rateS.set(S.rate, { silent: true });
    azS.set(S.az, { silent: true });
    elS.set(S.el, { silent: true });
    motionSel.set(S.motion, { silent: true });
    projSel.set(S.proj, { silent: true });
    tiltTog.set(i === 1, { silent: true });
    sinTog.set(true, { silent: true });
    cosTog.set(true, { silent: true });
    trackSel.set(String(S.track) as '0' | '1' | '2', { silent: true });
    inputSel.set(S.input, { silent: true });
    helixTog.set(false, { silent: true });
    syncVisibility();
    update();
  }

  // ---- go ----
  const onStep = (i: number) => chips.activate(Math.max(0, Math.min(SCENARIOS.length - 1, i)));
  onStep(ctx.currentStep);

  return {
    onStep,
    destroy() {
      loop.stop();
      drag.destroy();
      triS.dispose();
      triB.dispose();
      sprites.forEach(disposeSprite);
      plot.destroy();
      scene.destroy();
      root.remove();
    },
  };
}

/** Put an arrow from a to b; hide it when `on` is false or the arrow has zero length (setFromTo resets visibility). */
function place(a: Arrow3D, from: Vec, to: Vec, on: boolean): void {
  a.setFromTo(from, to);
  a.visible = on && Math.hypot(to[0] - from[0], to[1] - from[1], to[2] - from[2]) > 1e-6;
}

/** A toolkit matrixView whose label is typeset with KaTeX. */
function labelled(parent: HTMLElement, tex: string, digits = 3): MatrixHandle {
  const mv = matrixView(parent, { digits });
  mv.el.querySelector('.mr-mlabel')?.replaceChildren(mathEl(tex));
  return mv;
}

/** A matrixView with a small KaTeX caption above it instead of a label on its left (keeps a row of matrices narrow). */
function captioned(parent: HTMLElement, tex: string, digits = 2): { mv: MatrixHandle; box: HTMLElement } {
  const box = el('span', { class: 'c302-capm' }, el('span', { class: 'c302-cap' }, mathEl(tex)));
  parent.appendChild(box);
  return { mv: matrixView(box, { digits }), box };
}

const SCENARIOS: { label: string; title: string }[] = [
  { label: '① 角速度', title: 'ω = ω̂ θ̇ 與 ṗ = ω × p' },
  { label: '② ω_s、ω_b', title: 'ṘRᵀ 與 RᵀṘ' },
  { label: '③ Rodrigues', title: 'exp 的三項' },
  { label: '④ log', title: '由 R 反推 (ω̂, θ)' },
  { label: '⑤ 特殊角', title: 'θ = π 與 θ = 0：對數的特殊情況' },
  { label: '⑥ 非 Euler 角', title: '一步 vs 三步' },
];

const lesson: Lesson = { meta, steps, quiz, mount };
export default lesson;
