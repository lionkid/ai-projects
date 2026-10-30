import type { Lesson, LessonContext, LessonHandle, LessonStep, QuizQuestion } from '@/types';
import {
  el, cssVar, fmt, controlPanel, row, slider, button, select, kv, note, simTitle, legend, split, matrixView, playback,
  Canvas2D, RasterLayer, physicsRunner, keyboardDrive, presetChips, drawPlanarArm, type Pt, type RGBA,
} from '@/ui';
import { planarFK, se2 } from '@/mr';
import { meta } from './meta';
import {
  bodyTwistStep, constraintResidual, holonomicCloud, inAnnulus, isParked, lateralShiftPlan, mulberry32, penIK,
  pfaffianRow, poseAlongPlan, poseError, qdotOf, reachableCloud, samplePlan, slidePlan,
  type Pose, type Segment,
} from './constraints';

const steps: LessonStep[] = [
  {
    title: '硬幣只能往前滾',
    md: `從正上方看一枚直立滾動的硬幣（或一台單輪車）。描述它需要朝向角與接觸點的位置：$q = (\\phi, x, y)$，C-space 是 3 維的 $\\R^2 \\times S^1$。

輪子**不打滑**時，接觸點只能沿著輪子的朝向移動：可以前進、後退、原地轉向，卻**無法直接往側邊滑**。可是經驗告訴我們，來回幾次之後，車子仍然停得進正側邊的車位。速度受到限制，能到達的組態卻好像一個也沒少——這一課要把這件事說清楚。`,
    simHint: '點一下上方畫布，用 W/S 前進後退、A/D 轉向（或拉 v、ω 滑桿）；再按「嘗試側滑」看看會怎樣。',
  },
  {
    title: '寫成 Pfaffian 形式',
    md: `「不能側滑」等於說：速度在車體側向 $(-\\sin\\phi, \\cos\\phi)$ 上的分量為零。寫成對 $\\dot q$ 線性的形式：

$$\\dot x\\sin\\phi - \\dot y\\cos\\phi = 0 \\iff \\underbrace{\\begin{bmatrix} 0 & \\sin\\phi & -\\cos\\phi \\end{bmatrix}}_{A(q)} \\begin{bmatrix} \\dot\\phi \\\\ \\dot x \\\\ \\dot y \\end{bmatrix} = 0$$

形如 $A(q)\\dot q = 0$、$A(q) \\in \\R^{k \\times n}$ 的速度約束稱為 **Pfaffian 約束**。這裡 $k = 1$、$n = 3$：每一瞬間，3 維的速度只剩 2 個方向可用——沿朝向前進（$v$）與原地轉向（$\\omega$），也就是 $\\dot q = (\\omega,\\ v\\cos\\phi,\\ v\\sin\\phi)$。`,
    simHint: '邊開邊看「數值」面板：A(q) 隨 φ 改變，A(q) q̇ 卻始終是 0。「速度空間」圖中的藍線就是此刻允許的 (ẋ, ẏ)。',
  },
  {
    title: '能不能積分？完整 vs 非完整',
    md: `若約束來自某個位置關係 $g(q) = 0$——例如質點綁在長 $L$ 的桿上：$x^2 + y^2 - L^2 = 0$——對時間微分就得到 Pfaffian 形式：

$$g(q) = 0 \\;\\Rightarrow\\; \\frac{\\partial g}{\\partial q}\\,\\dot q = 0$$

這種**可積分**的約束稱為**完整約束（holonomic）**：它把可到達的組態壓在低一維的曲面上，質點只能待在圓上。

輪子的約束卻找不到任何 $g$ 能積分出來，稱為**非完整約束（nonholonomic）**。它只在每一瞬間刪掉一個速度方向，可到達的組態仍然是 3 維。`,
    simHint: '按「撒點：隨機輸入」：「對照」圖中桿上質點的終點全落在圓上；單輪車的終點卻散布成一片，朝向也各不相同。',
  },
  {
    title: '停車挑戰',
    md: `挑戰：把車停到正**左方** 1 m、朝向不變的車位——恰好是被禁止的側向。

只用 $v$ 與 $\\omega$，可以這樣組合：① 原地轉 $\\alpha$ ② 前進 $s$ ③ 轉回 $-\\alpha$ ④ 後退 $s\\cos\\alpha$。以出發時的車體座標表示，淨效果是

$$(\\Delta x_b,\\ \\Delta y_b,\\ \\Delta\\phi) = (0,\\ s\\sin\\alpha,\\ 0)$$

每一段都遵守 $A(q)\\dot q = 0$，組合起來卻產生了瞬間做不到的側移：兩個輸入照樣能到達 3 維 C-space 的任何地方，只是要多繞幾步。`,
    simHint: '把車停進虛線車位（位置誤差 < 0.1 m、角度誤差 < 6°）；卡關就按「▶ 自動示範」。再把模型換成「全向平台」比較一下。',
  },
  {
    title: 'Task space 與 workspace',
    md: `**Task space（任務空間）**由任務決定，只放任務在乎的座標：送貨只管 $(x, y)$，是 2 維；停車還要管朝向 $(x, y, \\phi)$，是 3 維。同一台車，任務不同，task space 就不同。

**Workspace（工作空間）**則由機器人決定：末端實際能到達的範圍。「對照」圖中的 2R 手臂要在紙上寫字：紙面座標是 task space，手臂的 workspace 是一個環帶。兩者不必相符——紙上落在環帶外的部分（紅色）就是做不到的任務。反過來，若用 6 軸手臂寫字，task space 只有 2–3 維，多出來的自由度可以拿來避開障礙。`,
    simHint: '拖曳「對照」圖中的紙張與筆尖，或切換 L₂：紅色是 task space 中超出 workspace 的部分。上方畫布的送貨點則不在乎車頭朝向。',
  },
  {
    title: '下一步：描述剛體的位姿',
    md: `單輪車的組態 $(\\phi, x, y)$ 其實就是平面剛體的**位姿**：車體座標系 $\\{b\\}$ 相對於固定座標系 $\\{s\\}$ 的轉動與平移。寫成 3×3 矩陣：

$$T_{sb} = \\begin{bmatrix} \\cos\\phi & -\\sin\\phi & x \\\\ \\sin\\phi & \\cos\\phi & y \\\\ 0 & 0 & 1 \\end{bmatrix}$$

前兩欄是 $\\hat x_b$、$\\hat y_b$ 在 $\\{s\\}$ 中的方向，第三欄是位置。注意 $\\hat x_b$ 正是允許的前進方向，$\\hat y_b$ 正是被禁止的側滑方向。第 3 章會把這套語言推廣到三維的 $\\SO(3)$ 與 $\\SE(3)$；先完成下方的自我檢測吧。`,
    simHint: '開動車子，對照「數值」面板中 T_sb 的紅、綠兩欄與畫布上 {b} 的紅、綠軸。',
  },
];

const quiz: QuizQuestion[] = [
  {
    question: '單輪車此刻朝向 $\\phi = 90°$（車頭朝 $+y$）。下列哪一個速度 $(\\dot x, \\dot y)$ 會違反不側滑約束？',
    options: ['$(0,\\ 0.5)$', '$(0,\\ -0.3)$', '$(0.4,\\ 0)$', '$(0,\\ 0)$，也就是靜止'],
    answer: 2,
    explanation: '$\\phi = 90°$ 時 $A(q) = [0,\\ 1,\\ 0]$，約束化為 $\\dot x = 0$。$(0.4, 0)$ 是純側向運動，$A(q)\\dot q = 0.4 \\neq 0$；沿車頭方向前進、後退或靜止都滿足約束。',
  },
  {
    question: '一列火車在圓形軌道上只能前進或後退。它受的約束屬於哪一種？C-space 是幾維？',
    options: ['非完整約束；2 維', '完整約束；1 維（一個圓 $S^1$）', '非完整約束；1 維', '完整約束；3 維'],
    answer: 1,
    explanation: '「必須在軌道上、車身沿軌道切線」是對**位置與朝向**的限制，可寫成 $g(q) = 0$ 的形式，所以是完整約束。它真正壓縮了 C-space：火車的組態只需軌道上的一個位置，形狀是圓 $S^1$。對比單輪車，輪子的約束只限制速度，並沒有減少可到達組態的維度。',
  },
  {
    question: '焊接任務要指定焊槍尖端的位置，以及焊槍軸線的指向（繞軸線本身轉動不影響焊接）。task space 是幾維？改用 6 軸手臂執行會怎樣？',
    options: [
      '3 維；手臂多出 3 個自由度',
      '5 維；手臂多出 1 個自由度，可在不影響焊接的情況下調整姿勢（例如避開障礙）',
      '6 維；剛好用完所有自由度',
      '5 維；6 軸手臂無法執行',
    ],
    answer: 1,
    explanation: '位置 3 維＋軸線方向（單位向量，$S^2$）2 維 = 5 維；繞軸線的轉動是任務不在乎的方向。6 軸手臂因此有 1 個多餘自由度，對每個焊點都有一整族組態可用，能拿來避開障礙或關節極限。',
  },
  {
    question: '若一組 Pfaffian 約束 $A(q)\\dot q = 0$（$k$ 列、$n$ 個座標、各列獨立）的每一列都能寫成某個 $g_i(q)$ 的梯度，從 $q_0$ 出發能到達的組態集合是？',
    options: [
      '整個 $n$ 維 C-space',
      '滿足 $g_i(q) = g_i(q_0)$ 的 $(n-k)$ 維曲面',
      '只有 $q_0$ 這一點',
      '一條曲線，與 $k$ 無關',
    ],
    answer: 1,
    explanation: '因為 $\\tfrac{d}{dt} g_i(q) = \\tfrac{\\partial g_i}{\\partial q}\\dot q = A_i(q)\\dot q = 0$，每個 $g_i$ 在運動中都保持不變，系統只能沿著 $k$ 條等值面的交集移動，維度是 $n - k$。這正是完整約束的特徵；非完整約束找不到這樣的 $g_i$，所以不會壓縮可到達的維度。',
  },
];

// ---------------------------------------------------------------------------

const WORLD = { xmin: -3.4, xmax: 3.4, ymin: -1.7, ymax: 1.7 };
// a short S-shaped drive that ends at the initial pose, so the first view already shows a rolling trace
const PRE_START: Pose = { x: -3.0, y: -1.25, phi: 0.15 };
const PRE_DRIVE: Segment[] = [
  { v: 0.6, w: 0.75, T: 1.7, label: '' },
  { v: 0.6, w: -0.8, T: 1.9, label: '' },
  { v: 0.6, w: 0.35, T: 1.0, label: '' },
];
const VIEW_POSE: Pose = poseAlongPlan(PRE_START, PRE_DRIVE, Infinity).pose;
const PARK_START: Pose = { x: -1.5, y: -1.2, phi: 0 };
const PARK_GOAL: Pose = { x: -1.5, y: -0.2, phi: 0 };
const DELIVER_GOAL: [number, number] = [2.3, 1.0];
const DELIVER_TOL = 0.15;
const KEY_V = 0.7;
const KEY_W = 1.2;
const KEY_VS = 0.5;
const PEND_L = 1.1;
const ARM_L1 = 1;

type Task = 'free' | 'park' | 'deliver';
type SecMode = 'vel' | 'pend' | 'task';

function mount(host: HTMLElement, ctx: LessonContext): LessonHandle {
  // ---------------- state
  let q: Pose = { ...VIEW_POSE };
  let model: 'unicycle' | 'omni' = 'unicycle';
  let task: Task = 'free';
  let trail: Pt[] = samplePlan(PRE_START, PRE_DRIVE, 0.05).map((p) => [p.x, p.y] as Pt);
  let demo: { start: Pose; plan: Segment[]; t: number; seg: number } | null = null;
  let inputs = { v: 0, vs: 0, w: 0 };
  let flashUntil = 0;                 // wall-clock ms until which the side-slip attempt is shown
  let wallUntil = 0;
  let doneAt: number | null = null;   // sim time when the current task was completed
  let simT = 0;
  let cloud: Pose[] = [];
  let pendCloud: Pt[] = [];
  let secMode: SecMode = 'vel';
  let stepNow = ctx.currentStep;
  let dirty = true;
  // pendulum (holonomic comparison)
  let psi = -0.9;
  let pendPointer: Pt | null = null;
  // task-space card
  let L2 = 0.6;
  let paper = { cx: 0.95, cy: 0.6, w: 1.35, h: 0.85 };
  let penTarget: Pt = [1.0, 0.5];
  let penTh: [number, number] = [0.3, 1.2];
  let penReach = true;
  const rng = mulberry32(11);
  const timers: number[] = [];

  // ---------------- layout
  simTitle(host, '俯視單輪車：速度受限，組態不受限', 'A(q) q̇ = 0');
  const cv = new Canvas2D(host, { aspect: 2, world: WORLD, overlay: true, margin: 0.01 });
  Object.assign(cv.overlay.style, { whiteSpace: 'pre', background: 'color-mix(in srgb, var(--sim-bg) 82%, transparent)', padding: '0.05rem 0.3rem', borderRadius: '4px' });
  const kb = keyboardDrive(cv.el, { hint: '點一下畫布啟用鍵盤：W/S 前後、A/D 轉向、空白鍵煞車', onChange: () => { if (demo && kb.pressed.size) stopDemo(); dirty = true; } });

  const panel = controlPanel(host, { compact: true });
  const r1 = row(panel);
  const pb = playback(r1, {
    playing: true,
    showTime: false,
    onPlay: () => runner.start(),
    onPause: () => runner.stop(),
    onReset: () => resetTask(),
    onSpeed: (s) => { runner.speed = s; },
  });
  const modelSel = select(r1, {
    options: [{ value: 'unicycle', label: '單輪車（不能側滑）' }, { value: 'omni', label: '全向平台（對照組）' }],
    value: 'unicycle',
    onChange: (m) => setModel(m),
  });
  const taskSel = select(r1, {
    options: [{ value: 'free', label: '任務：自由駕駛' }, { value: 'park', label: '任務：停車 (x, y, φ)' }, { value: 'deliver', label: '任務：送貨 (x, y)' }],
    value: 'free',
    onChange: (t) => setTask(t),
  });
  const sV = slider(panel, { label: '前進速度 v', min: -1, max: 1, step: 0.05, value: 0, unit: 'm/s', onInput: () => { dirty = true; } });
  const sW = slider(panel, { label: '轉向率 ω', min: -1.5, max: 1.5, step: 0.05, value: 0, unit: 'rad/s', onInput: () => { dirty = true; } });
  const sVS = slider(panel, { label: '側向速度 v⊥', min: -0.8, max: 0.8, step: 0.05, value: 0, unit: 'm/s', onInput: () => { dirty = true; } });
  sVS.el.style.display = 'none';
  const r2 = row(panel);
  const slipBtn = button(r2, { label: '↔ 嘗試側滑', small: true, title: '命令車子直接往左側移動', onClick: () => trySlip() });
  button(r2, { label: '■ 煞車', small: true, title: '把 v、ω 歸零（空白鍵）', onClick: () => brake() });
  const cloudBtn = button(r2, { label: '✳ 撒點：隨機輸入', small: true, title: '從目前組態出發，試 300 組隨機的 (v, ω)', onClick: () => sampleClouds() });
  button(r2, { label: '清除軌跡', small: true, onClick: () => { trail = []; cloud = []; pendCloud = []; dirty = true; } });
  const demoBtn = button(r2, { label: '▶ 自動示範停車', small: true, variant: 'primary', title: '從車位起點自動執行一組只用 v、ω 的動作', onClick: () => startDemo() });

  const [secHost, numHost] = split(host);
  const sec = new Canvas2D(secHost, { aspect: 1.15, world: { xmin: -1.3, xmax: 1.3, ymin: -1.13, ymax: 1.13 }, overlay: true, margin: 0.02 });
  Object.assign(sec.overlay.style, { whiteSpace: 'pre', background: 'color-mix(in srgb, var(--sim-bg) 82%, transparent)', padding: '0.05rem 0.3rem', borderRadius: '4px' });
  const secCtl = el('div', { style: { display: 'none' } });
  secHost.appendChild(secCtl);
  const l2Chips = presetChips(secCtl, ([[1.0, '1.0'], [0.6, '0.6'], [0.35, '0.35']] as const).map(([v, txt]) => ({ label: txt, apply: () => { L2 = v; repaintPaper(); updatePen(); } })), { label: '手臂 L₁ = 1、L₂ =' });

  numHost.appendChild(el('div', { class: 'mr-note', style: { margin: '0 0 0.2rem', fontWeight: '600', color: 'var(--fg)' } }, '數值'));
  const info = kv(numHost, ['q = (φ, x, y)', 'q̇ = (φ̇, ẋ, ẏ)', '輸入', 'A(q) q̇']);
  const aView = matrixView(numHost, { label: 'A(q) =', digits: 3 });
  const tView = matrixView(numHost, { label: 'T_sb =', digits: 2 });
  const status = el('div', { class: 'mr-note', style: { minHeight: '2.4em', margin: '0.3rem 0 0' } });
  numHost.appendChild(status);

  legend(host, [
    { color: cssVar('--accent'), label: '允許的速度方向' },
    { color: cssVar('--danger'), label: '被禁止的側滑' },
    { color: cssVar('--warn'), label: '軌跡／實際速度' },
    { color: cssVar('--series-5'), label: '隨機輸入的終點' },
  ]);
  note(host, '位姿用 SE(2) 指數映射精確積分（等速圓弧），不會因 Euler 積分而慢慢漂移。');

  // ---------------- simulation
  const inBounds = (p: Pose) => p.x > WORLD.xmin + 0.25 && p.x < WORLD.xmax - 0.25 && p.y > WORLD.ymin + 0.25 && p.y < WORLD.ymax - 0.25;
  const runner = physicsRunner({
    h: 1 / 120,
    step: (h) => {
      simT += h;
      if (demo) {
        demo.t += h;
        const r = poseAlongPlan(demo.start, demo.plan, demo.t);
        const g = demo.plan[r.seg];
        inputs = { v: g.v, vs: g.vs ?? 0, w: g.w };
        q = r.pose;
        demo.seg = r.seg;
        if (r.done) { demo = null; inputs = { v: 0, vs: 0, w: 0 }; }
      } else {
        const keyV = kb.axis('down', 'up');
        const keyW = kb.axis('right', 'left');
        const keyS = (kb.has('a') ? 1 : 0) - (kb.has('b') ? 1 : 0);
        const keys = kb.pressed.size > 0;
        inputs = keys
          ? { v: keyV * KEY_V, w: keyW * KEY_W, vs: model === 'omni' ? keyS * KEY_VS : 0 }
          : { v: sV.value, w: sW.value, vs: model === 'omni' ? sVS.value : 0 };
        if (kb.has('brake')) { brake(); inputs = { v: 0, vs: 0, w: 0 }; }
        if (inputs.v === 0 && inputs.w === 0 && inputs.vs === 0) return;
        const nq = bodyTwistStep(q, inputs.v, inputs.vs, inputs.w, h);
        if (!inBounds(nq) && inBounds(q)) { wallUntil = performance.now() + 1500; return; }
        q = nq;
      }
      const last = trail[trail.length - 1];
      if (!last || Math.hypot(q.x - last[0], q.y - last[1]) > 0.02) {
        trail.push([q.x, q.y]);
        if (trail.length > 4000) trail.splice(0, trail.length - 4000);
      }
      dirty = true;
    },
    onFrame: () => {
      checkTask();
      if (performance.now() < flashUntil || performance.now() < wallUntil) dirty = true;
      if (!dirty) return;
      dirty = false;
      cv.requestRender();
      sec.requestRender();
      updatePanel();
    },
  });

  function brake(): void {
    sV.set(0, { silent: true });
    sW.set(0, { silent: true });
    sVS.set(0, { silent: true });
    stopDemo();
    dirty = true;
  }

  function stopDemo(): void {
    if (!demo) return;
    demo = null;
    inputs = { v: 0, vs: 0, w: 0 };
    dirty = true;
  }

  function trySlip(): void {
    if (model === 'omni') {
      // an omnidirectional base simply slides
      brake();
      demo = { start: { ...q }, plan: slidePlan(0.5), t: 0, seg: 0 };
    } else {
      flashUntil = performance.now() + 2600;
      timers.push(window.setTimeout(() => { dirty = true; cv.requestRender(); sec.requestRender(); updatePanel(); }, 2650));
    }
    if (!runner.running) { runner.start(); pb.setPlaying(true, { silent: true }); }
    dirty = true;
  }

  function setModel(m: 'unicycle' | 'omni'): void {
    model = m;
    modelSel.set(m, { silent: true });
    sVS.el.style.display = m === 'omni' ? '' : 'none';
    if (m === 'unicycle') sVS.set(0, { silent: true });
    stopDemo();
    dirty = true;
  }

  function setTask(t: Task): void {
    const prev = task;
    task = t;
    taskSel.set(t, { silent: true });
    doneAt = null;
    stopDemo();
    if (t === 'park' && prev !== 'park') resetTask();
    dirty = true;
  }

  function resetTask(): void {
    stopDemo();
    brake();
    q = task === 'park' ? { ...PARK_START } : { ...VIEW_POSE };
    trail = [[q.x, q.y]];
    cloud = [];
    doneAt = null;
    simT = 0;
    dirty = true;
  }

  function startDemo(): void {
    if (task !== 'park') setTask('park');
    brake();
    q = { ...PARK_START };
    trail = [[q.x, q.y]];
    doneAt = null;
    const d = PARK_GOAL.y - PARK_START.y;
    demo = { start: { ...q }, plan: model === 'omni' ? slidePlan(d) : lateralShiftPlan(d), t: 0, seg: 0 };
    if (!runner.running) { runner.start(); pb.setPlaying(true, { silent: true }); }
    dirty = true;
  }

  function checkTask(): void {
    let ok = false;
    if (task === 'park') ok = isParked(q, PARK_GOAL);
    else if (task === 'deliver') ok = Math.hypot(q.x - DELIVER_GOAL[0], q.y - DELIVER_GOAL[1]) <= DELIVER_TOL;
    if (ok && doneAt == null && !demo) { doneAt = simT; dirty = true; }
    else if (!ok && doneAt != null) { doneAt = null; dirty = true; }
  }

  function sampleClouds(): void {
    cloud = reachableCloud(q, 300, rng);
    pendCloud = holonomicCloud(PEND_L, psi, 300, rng);
    dirty = true;
  }

  // ---------------- drawing helpers
  const drawVehicle = (c: Canvas2D, p: Pose, o: { alpha?: number; ghost?: boolean; frameSize?: number; labels?: boolean } = {}) => {
    const alpha = o.alpha ?? 1;
    const cs = Math.cos(p.phi);
    const sn = Math.sin(p.phi);
    const P = (u: number, v: number): Pt => [p.x + cs * u - sn * v, p.y + sn * u + cs * v];
    const fg = cssVar('--fg');
    const muted = cssVar('--fg-muted');
    const dash = o.ghost ? [4, 3] : undefined;
    if (model === 'omni') {
      const hw = 0.23;
      c.polygon([P(-hw, -hw), P(hw, -hw), P(hw, hw), P(-hw, hw)], { fill: o.ghost ? undefined : cssVar('--bg-elev'), color: o.ghost ? muted : fg, width: 1.5, alpha, dash });
      if (!o.ghost) for (const [u, v] of [[1, 1], [1, -1], [-1, 1], [-1, -1]]) c.disk(P(u * 0.17, v * 0.17), 0.045, { fill: muted, alpha });
    } else {
      c.circle([p.x, p.y], 0.25, { fill: o.ghost ? undefined : cssVar('--bg-elev'), color: muted, width: 1.5, alpha, dash });
      c.polygon([P(-0.2, -0.05), P(0.2, -0.05), P(0.2, 0.05), P(-0.2, 0.05)], { fill: o.ghost ? undefined : fg, color: o.ghost ? muted : fg, width: 1.2, alpha, dash });
    }
    c.frame({ x: p.x, y: p.y, theta: p.phi }, o.frameSize ?? 0.42, { width: 2, alpha, labels: o.labels ? 'b' : undefined });
  };

  cv.draw = (c) => {
    const accent = cssVar('--accent');
    const danger = cssVar('--danger');
    const warn = cssVar('--warn');
    const okc = cssVar('--ok');
    const muted = cssVar('--fg-muted');
    const bg = cssVar('--sim-bg');
    const focusFrames = stepNow === 5;
    c.grid(0.5);
    c.frame({ x: 0, y: 0, theta: 0 }, focusFrames ? 0.7 : 0.5, { labels: 's', width: focusFrames ? 3 : 1.8 });
    // random-input end poses
    for (const p of cloud) c.arrow([p.x, p.y], [p.x + 0.17 * Math.cos(p.phi), p.y + 0.17 * Math.sin(p.phi)], { color: cssVar('--series-5'), width: 1.4, head: 5, alpha: 0.75 });
    // task targets
    if (task === 'park') {
      const g = PARK_GOAL;
      const cs = Math.cos(g.phi), sn = Math.sin(g.phi);
      const P = (u: number, v: number): Pt => [g.x + cs * u - sn * v, g.y + sn * u + cs * v];
      const done = doneAt != null;
      c.polygon([P(-0.46, -0.34), P(0.46, -0.34), P(0.46, 0.34), P(-0.46, 0.34)], { color: done ? okc : muted, width: 2, dash: [6, 4], fill: done ? cssVar('--ok-soft') : undefined });
      c.text(P(-0.36, 0.22), 'P', { color: done ? okc : muted, font: `600 13px ${cssVar('--font')}` });
      drawVehicle(c, g, { ghost: true, alpha: 0.6, frameSize: 0.32 });
      c.point([PARK_START.x, PARK_START.y], 3, { color: muted });
      c.text([PARK_START.x, PARK_START.y], '起點', { color: muted, dx: -40 });
    } else if (task === 'deliver') {
      const done = doneAt != null;
      const col = done ? okc : cssVar('--series-6');
      c.circle(DELIVER_GOAL, DELIVER_TOL, { color: col, width: 2, fill: done ? cssVar('--ok-soft') : undefined });
      c.line([DELIVER_GOAL[0] - 0.25, DELIVER_GOAL[1]], [DELIVER_GOAL[0] + 0.25, DELIVER_GOAL[1]], { color: col, width: 1.2 });
      c.line([DELIVER_GOAL[0], DELIVER_GOAL[1] - 0.25], [DELIVER_GOAL[0], DELIVER_GOAL[1] + 0.25], { color: col, width: 1.2 });
      c.text(DELIVER_GOAL, '送貨點 (x, y)：朝向不計', { color: col, dy: -22, bg });
    }
    if (trail.length > 1) c.polyline(trail, { color: warn, width: 2, alpha: 0.85 });
    // vehicle + allowed direction + actual velocity
    const cs = Math.cos(q.phi), sn = Math.sin(q.phi);
    if (model === 'unicycle') c.line([q.x - 0.7 * cs, q.y - 0.7 * sn], [q.x + 0.7 * cs, q.y + 0.7 * sn], { color: accent, width: 2, dash: [5, 4], alpha: 0.75 });
    drawVehicle(c, q, { labels: true, frameSize: focusFrames ? 0.6 : 0.42 });
    const qd = qdotOf(q.phi, inputs.v, inputs.vs, inputs.w);
    if (Math.hypot(qd[1], qd[2]) > 0.02) c.arrow([q.x, q.y], [q.x + qd[1] * 0.9, q.y + qd[2] * 0.9], { color: warn, width: 3, head: 9 });
    const flash = performance.now() < flashUntil && model === 'unicycle';
    if (flash) {
      for (const sgn of [1, -1]) {
        const a: Pt = [q.x - sn * 0.3 * sgn, q.y + cs * 0.3 * sgn];
        const b: Pt = [q.x - sn * 0.85 * sgn, q.y + cs * 0.85 * sgn];
        c.arrow(a, b, { color: danger, width: 3, dash: [6, 4], head: 11 });
      }
      c.text([q.x - sn * 1.02, q.y + cs * 1.02], '✕ 側滑被禁止', { color: danger, bg, font: `600 12px ${cssVar('--font')}` });
    }
    // overlay
    let line2 = '';
    if (demo) line2 = `自動示範 ${demo.seg + 1}/${demo.plan.length}：${demo.plan[demo.seg].label}`;
    else if (flash) line2 = '嘗試側滑：A(q)q̇ = −0.50 ≠ 0 → 被約束擋下';
    else if (performance.now() < wallUntil) line2 = '撞到邊界了：轉向或倒車離開';
    else if (task === 'park') {
      const e = poseError(q, PARK_GOAL);
      line2 = doneAt != null ? '✓ 停車成功！' : `車位誤差：|Δp| = ${fmt(e.dist, 2)} m，Δφ = ${fmt((e.dphi * 180) / Math.PI, 0)}°`;
    } else if (task === 'deliver') {
      const d = Math.hypot(q.x - DELIVER_GOAL[0], q.y - DELIVER_GOAL[1]);
      line2 = doneAt != null ? '✓ 送達！（不在乎朝向）' : `離送貨點 ${fmt(d, 2)} m`;
    }
    c.overlay.textContent = `q = (φ, x, y) = (${fmt((q.phi * 180) / Math.PI, 0)}°, ${fmt(q.x, 2)}, ${fmt(q.y, 2)})${line2 ? `\n${line2}` : ''}`;
  };

  // ---------------- secondary canvas: velocity space / holonomic comparison / task vs workspace
  const pendHandle = sec.addHandle({
    get: () => [PEND_L * Math.cos(psi), PEND_L * Math.sin(psi)],
    set: (p) => { psi = Math.atan2(p[1], p[0]); pendPointer = p; sec.requestRender(); },
    onEnd: () => { pendPointer = null; sec.requestRender(); },
    radius: 9,
    color: cssVar('--accent'),
    enabled: false,
  });
  const paperHandle = sec.addHandle({
    get: () => [paper.cx - paper.w / 2, paper.cy + paper.h / 2],
    set: (p) => {
      paper = { ...paper, cx: Math.max(-1.2, Math.min(1.5, p[0] + paper.w / 2)), cy: Math.max(-0.9, Math.min(1.45, p[1] - paper.h / 2)) };
      clampPen();
      repaintPaper();
      updatePen();
    },
    radius: 7,
    color: cssVar('--fg-muted'),
    label: '拖曳紙張',
    enabled: false,
  });
  const penHandle = sec.addHandle({
    get: () => penTarget,
    set: (p) => { penTarget = [p[0], p[1]]; clampPen(); updatePen(); },
    radius: 8,
    color: cssVar('--accent'),
    label: '筆尖',
    enabled: false,
  });
  const clampPen = () => {
    penTarget = [
      Math.max(paper.cx - paper.w / 2, Math.min(paper.cx + paper.w / 2, penTarget[0])),
      Math.max(paper.cy - paper.h / 2, Math.min(paper.cy + paper.h / 2, penTarget[1])),
    ];
  };
  const updatePen = () => {
    const r = penIK(ARM_L1, L2, penTarget, penTh);
    penTh = r.th;
    penReach = r.reachable;
    sec.requestRender();
  };
  const TASK_WORLD = { xmin: -1.75, xmax: 2.15, ymin: -1.45, ymax: 1.94 };
  const raster = new RasterLayer(TASK_WORLD, 160, 139);
  const repaintPaper = () => {
    const x0 = paper.cx - paper.w / 2, x1 = paper.cx + paper.w / 2;
    const y0 = paper.cy - paper.h / 2, y1 = paper.cy + paper.h / 2;
    raster.paint((x, y) => (x < x0 || x > x1 || y < y0 || y > y1 ? NaN : inAnnulus(ARM_L1, L2, [x, y]) ? 1 : 0), {
      colormap: (v): RGBA => (v > 0.5 ? [22, 163, 74, 70] : [220, 38, 38, 130]),
      range: [0, 1],
    });
    sec.requestRender();
  };
  l2Chips.activate(1); // L2 = 0.6: repaints the paper and solves the pen IK

  sec.draw = (c) => {
    const accent = cssVar('--accent');
    const danger = cssVar('--danger');
    const warn = cssVar('--warn');
    const muted = cssVar('--fg-muted');
    const fg = cssVar('--fg');
    const bg = cssVar('--sim-bg');
    const font = cssVar('--font');
    if (secMode === 'vel') {
      c.grid(0.25);
      c.text([1.2, 0], 'ẋ', { color: muted, dy: -9 });
      c.text([0, 1.05], 'ẏ', { color: muted, dx: 9 });
      c.circle([0, 0], 1, { color: cssVar('--grid-strong'), width: 1, dash: [3, 3] });
      const cs = Math.cos(q.phi), sn = Math.sin(q.phi);
      if (model === 'omni') {
        c.circle([0, 0], 1, { fill: cssVar('--accent-soft'), color: accent, width: 1.5 });
        c.text([0, -0.62], '全向平台：任何 (ẋ, ẏ) 都允許', { color: accent, bg, font: `12px ${font}` });
      } else {
        c.line([-1.15 * cs, -1.15 * sn], [1.15 * cs, 1.15 * sn], { color: accent, width: 4, alpha: 0.85 });
        c.text([1.0 * cs, 1.0 * sn], '允許', { color: accent, bg, dy: -12, font: `12px ${font}` });
        for (const sg of [1, -1]) c.arrow([0, 0], [-sn * 0.75 * sg, cs * 0.75 * sg], { color: danger, width: 1.8, dash: [5, 4], head: 8 });
        const lo = cs >= 0 ? -1 : 1; // label the tip that points down, away from the overlay text
        c.text([-sn * 0.88 * lo, cs * 0.88 * lo], '✕ 禁止', { color: danger, bg, font: `12px ${font}` });
      }
      const qd = qdotOf(q.phi, inputs.v, inputs.vs, inputs.w);
      if (Math.hypot(qd[1], qd[2]) > 0.01) c.arrow([0, 0], [qd[1], qd[2]], { color: warn, width: 3, head: 10 });
      c.point([qd[1], qd[2]], 4, { color: warn });
      if (performance.now() < flashUntil && model === 'unicycle') {
        const a: Pt = [-sn * 0.5, cs * 0.5];
        c.arrow([0, 0], a, { color: danger, width: 3.5, head: 11 });
        c.text(a, '嘗試的 q̇', { color: danger, dx: 30, dy: -8, bg, font: `12px ${font}` });
      }
      c.overlay.textContent = '速度空間 (ẋ, ẏ)\nφ̇ = ω 不受限制';
    } else if (secMode === 'pend') {
      c.grid(0.5);
      c.circle([0, 0], PEND_L, { color: accent, width: 3.5, alpha: 0.85 });
      c.text([-PEND_L * 0.71, PEND_L * 0.71], '可到達：一個圓（1 維）', { color: accent, dy: -14, bg, font: `12px ${font}` });
      for (const p of pendCloud) c.point(p, 2.5, { color: cssVar('--series-5'), alpha: 0.8 });
      c.polygon([[-0.12, 0.1], [0.12, 0.1], [0, 0]], { fill: cssVar('--grid-strong'), color: muted, width: 1 });
      c.line([-0.18, 0.1], [0.18, 0.1], { color: muted, width: 2 });
      const bob: Pt = [PEND_L * Math.cos(psi), PEND_L * Math.sin(psi)];
      c.line([0, 0], bob, { color: fg, width: 3 });
      if (pendPointer) {
        c.line(pendPointer, bob, { color: danger, width: 1.5, dash: [4, 3] });
        c.point(pendPointer, 4, { color: danger });
        c.text(pendPointer, '想拉到這裡', { color: danger, dy: -12, bg, font: `11px ${font}` });
      }
      c.overlay.textContent = '對照：桿上質點（完整約束）\nx² + y² = L²，A(q) = [2x  2y]';
    } else {
      c.grid(0.5);
      const ro = ARM_L1 + L2, ri = Math.abs(ARM_L1 - L2);
      c.circle([0, 0], ro, { fill: cssVar('--accent-soft'), color: accent, width: 1.2, dash: [5, 4] });
      if (ri > 1e-3) c.circle([0, 0], ri, { fill: bg, color: accent, width: 1.2, dash: [5, 4] });
      c.text([-ro * 0.62, -ro * 0.62], 'workspace', { color: accent, bg, font: `11px ${font}` });
      raster.draw(c, 1, false);
      const x0 = paper.cx - paper.w / 2, x1 = paper.cx + paper.w / 2;
      const y0 = paper.cy - paper.h / 2, y1 = paper.cy + paper.h / 2;
      c.polygon([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], { color: fg, width: 1.5 });
      c.text([x1, y0], '紙＝task space', { color: fg, dy: 11, align: 'right', bg, font: `11px ${font}` });
      drawPlanarArm(c, [ARM_L1, L2], penTh, { linkWidth: 7, eeRadius: 5 });
      const tip = planarFK([ARM_L1, L2], penTh).tip as Pt;
      if (!penReach) {
        c.line(tip, penTarget, { color: danger, width: 1.5, dash: [4, 3] });
        c.text(penTarget, '✕ 到不了', { color: danger, dy: 18, bg, font: `11px ${font}` });
      }
      c.overlay.textContent = 'Task space vs workspace\n紅：任務要求、手臂卻到不了';
    }
  };

  function setSecMode(m: SecMode): void {
    secMode = m;
    pendHandle.enabled = m === 'pend';
    paperHandle.enabled = m === 'task';
    penHandle.enabled = m === 'task';
    secCtl.style.display = m === 'task' ? '' : 'none';
    if (m === 'vel') sec.setWorld({ xmin: -1.3, xmax: 1.3, ymin: -1.13, ymax: 1.13 });
    else if (m === 'pend') sec.setWorld({ xmin: -1.5, xmax: 1.5, ymin: -1.3, ymax: 1.3 });
    else sec.setWorld(TASK_WORLD);
    sec.requestRender();
  }

  // ---------------- numbers
  function updatePanel(): void {
    const qd = qdotOf(q.phi, inputs.v, inputs.vs, inputs.w);
    const flash = performance.now() < flashUntil && model === 'unicycle';
    info.set('q = (φ, x, y)', `(${fmt((q.phi * 180) / Math.PI, 0)}°, ${fmt(q.x, 2)}, ${fmt(q.y, 2)})`);
    info.set('q̇ = (φ̇, ẋ, ẏ)', `(${fmt(qd[0], 2)}, ${fmt(qd[1], 2)}, ${fmt(qd[2], 2)})`);
    info.set('輸入', model === 'omni' ? `v ${fmt(inputs.v, 2)}, v⊥ ${fmt(inputs.vs, 2)}, ω ${fmt(inputs.w, 2)}` : `v ${fmt(inputs.v, 2)}, ω ${fmt(inputs.w, 2)}`);
    if (flash) {
      const slip = qdotOf(q.phi, 0, 0.5, 0);
      info.set('A(q) q̇', `${fmt(constraintResidual(q.phi, slip), 2)} ≠ 0（嘗試側滑）✕`);
    } else {
      const res = constraintResidual(q.phi, qd);
      info.set('A(q) q̇', Math.abs(res) < 5e-4 ? `${fmt(0, 3)} ✓ 滿足約束` : `${fmt(res, 3)}（全向平台不受此約束）`);
    }
    aView.set([pfaffianRow(q.phi)]);
    tView.set(se2(q.phi, q.x, q.y), { highlightCols: ['x', 'y', null] });
    let s = '';
    if (task === 'park') s = doneAt != null ? `✓ 停車成功（用時 ${fmt(doneAt, 1)} s）。只靠 v、ω 兩個輸入就移到了 3 維 C-space 中的正側方。` : '停車：位置與朝向（包裹到 ±180°）都要在容許誤差內。';
    else if (task === 'deliver') s = doneAt != null ? '✓ 送達：送貨任務只在乎 (x, y)，車頭朝哪都算數。' : '送貨任務的 task space 只有 (x, y) 兩維。';
    else s = model === 'omni' ? '全向平台沒有側滑約束：3 個輸入 (v, v⊥, ω) 對應 3 維速度。' : '單輪車：2 個輸入 (v, ω)，3 維 C-space。';
    status.textContent = s;
  }

  // ---------------- per-step configuration
  let focused: HTMLElement[] = [];
  const focusOn = (els: HTMLElement[]) => {
    focused.forEach((f) => f.classList.remove('mr-focus'));
    focused = els;
    focused.forEach((f) => f.classList.add('mr-focus'));
  };
  const onStep = (i: number) => {
    stepNow = i;
    if (i !== 2) { cloud = []; pendCloud = []; }
    if (i === 0) { setSecMode('vel'); setTask('free'); focusOn([slipBtn, cv.el]); }
    else if (i === 1) { setSecMode('vel'); setTask('free'); focusOn([aView.el, sec.el]); }
    else if (i === 2) { setSecMode('pend'); setTask('free'); if (!cloud.length) sampleClouds(); focusOn([cloudBtn, sec.el]); }
    else if (i === 3) { setSecMode('vel'); setTask('park'); focusOn([taskSel.el, demoBtn, cv.el]); }
    else if (i === 4) { setSecMode('task'); setTask('deliver'); focusOn([sec.el, taskSel.el]); }
    else { setSecMode('vel'); setTask('free'); focusOn([tView.el, cv.el]); }
    dirty = true;
    cv.requestRender();
    updatePanel();
  };
  onStep(ctx.currentStep);
  runner.start();

  return {
    onStep,
    destroy() {
      runner.stop();
      kb.destroy();
      timers.forEach((t) => clearTimeout(t));
      cv.destroy();
      sec.destroy();
    },
  };
}

const lesson: Lesson = { meta, steps, quiz, mount };
export default lesson;
