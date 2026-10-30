import type { Lesson, LessonContext, LessonHandle, LessonStep, QuizQuestion } from '@/types';
import {
  el, cssVar, fmt, fmtVec, controlPanel, row, slider, toggle, button, select, playback, readout, simTitle, note,
  kv, presetChips, mathEl, jointSliders, subscript, Canvas2D, Scene3D, Robot3D, Arrow3D, createLoop,
  type Pt,
} from '@/ui';
import { type Mat, type RobotModel, type Vec, clamp, getCol, matVec, planar3R, planarFK, rrr3, wrapToPi } from '@/mr';
import { meta } from './meta';
import {
  type JacobianSnapshot, type VelocityBreakdown, axisPoint, changedColumns, instantCenter, jacobianSnapshot,
  planarAdjoint, planarRows, planarTwist, singularityMeasure, velocityBreakdown,
} from './jacobianGeometry';
import { liveMatrix } from './liveMatrix';

// ---------------------------------------------------------------------------
// Narrative
// ---------------------------------------------------------------------------

const steps: LessonStep[] = [
  {
    title: '一次只動一個關節',
    md: `先把問題拆小：只讓關節 $i$ 以 $\\dot\\theta_i = 1$ rad/s 轉動，其餘關節全部鎖住。此時關節 $i$ 之後的整段手臂就是一個剛體，繞著關節 $i$ **此刻所在**的軸旋轉，所以末端的 twist 恰好就是這根軸的螺旋軸座標。把它寫成一欄、$n$ 個關節並排，就得到 Jacobian：

$$\\dot\\theta = e_i \\;\\Longrightarrow\\; \\vV_s = J_{s,i}(\\theta)$$

平面手臂的旋轉軸都垂直紙面（圖中的 ⊙）。若關節位於 $q = (q_x, q_y)$，這一欄就是 $[\\,1,\\; q_y,\\; -q_x\\,]^{\\T}$，列的順序是 $(\\omega_z, v_x, v_y)$。`,
    simHint: '按「θ̇₂ = 1」做一次脈衝：只有關節 2 擺動。對照藍色速度箭頭、虛線旁的 q₂ 座標，以及矩陣裡亮起的第 2 欄。',
  },
  {
    title: 'J_s 的欄位怎麼算',
    md: `零位時關節 $i$ 的螺旋軸是 $\\vS_i$。手臂動起來之後，真正會搬動這根軸的只有它**前面**的關節 $1,\\dots,i-1$；它們合起來是一個剛體運動，而把螺旋軸搬到新位置要用這個運動的 Adjoint：

$$J_{s,i}(\\theta) = \\big[\\Ad_{e^{[\\vS_1]\\theta_1}\\cdots e^{[\\vS_{i-1}]\\theta_{i-1}}}\\big]\\,\\vS_i$$

兩個推論：第 1 欄永遠等於 $\\vS_1$；第 $i$ 欄只依賴 $\\theta_1,\\dots,\\theta_{i-1}$。關節 $i$ 自己轉動，並不會移動自己的軸。`,
    simHint: '拖動 θ₁：下方的提示會列出 J_s 改變的欄（第 2、3 欄），第 1 欄始終不動。再拖動 θ₃：J_s 還會變嗎？',
  },
  {
    title: '疊加：V = J θ̇',
    md: `幾個關節同時轉動時，每個關節各自貢獻一個 twist $J_{s,i}\\dot\\theta_i$。它們寫在同一個座標系 $\\{s\\}$、描述同一個瞬間，因此可以直接相加：

$$\\vV_s = J_s(\\theta)\\,\\dot\\theta = \\sum_{i=1}^{n} J_{s,i}(\\theta)\\,\\dot\\theta_i$$

落到末端點上，就是各關節造成的速度向量頭尾相接（彩色箭頭），總和為藍色的 $\\dot p$。合成的 twist 仍是一個螺旋運動：在平面上相當於繞瞬時轉心 ✕ 轉動；若 $\\sum_i \\dot\\theta_i = 0$，則 $\\omega_z = 0$，變成純平移。`,
    simHint: '調整 θ̇₁～θ̇₃，看三支彩色分量如何頭尾相接成藍色的 ṗ；按 ▶ 播放，末端軌跡始終與藍色箭頭相切。也試試 θ̇ = (0.5, −1, 0.5)。',
  },
  {
    title: 'J_b：從末端看同一件事',
    md: `改在末端座標系 $\\{b\\}$ 中描述。從末端往回看，會改變關節 $i$ 的軸相對位置的，是它**後面**的關節 $i+1,\\dots,n$：

$$J_{b,i}(\\theta) = \\big[\\Ad_{e^{-[\\vB_n]\\theta_n}\\cdots e^{-[\\vB_{i+1}]\\theta_{i+1}}}\\big]\\,\\vB_i$$

所以最後一欄永遠是 $\\vB_n$，第 $i$ 欄只依賴 $\\theta_{i+1},\\dots,\\theta_n$。現在相機黏在 $\\{b\\}$ 上：轉動 $\\theta_1$ 時，只有地面與 $\\{s\\}$ 在轉，手臂相對於末端的形狀完全沒變，$J_b$ 也就不變。`,
    simHint: '相機已固定在 {b}。拖動 θ₁：J_b 有任何改變嗎？再拖動 θ₃：這次哪幾欄改變、哪一欄不動？',
  },
  {
    title: '兩個 Jacobian 只差一個 Adjoint',
    md: `$J_s$ 與 $J_b$ 描述的是同一組關節軸、同一個末端運動，差別只在寫在哪個座標系。twist 從 $\\{b\\}$ 換到 $\\{s\\}$ 要乘上 $[\\Ad_{T_{sb}}]$；Jacobian 的每一欄都是 twist，所以整個矩陣一起換：

$$J_s(\\theta) = [\\Ad_{T_{sb}}]\\,J_b(\\theta), \\qquad J_b(\\theta) = [\\Ad_{T_{bs}}]\\,J_s(\\theta)$$

下方即時計算 $\\|J_s - [\\Ad_{T_{sb}}]J_b\\|$：不論姿勢為何，都只剩 $10^{-16}$ 等級的浮點誤差。平面例子裡兩者的 $\\omega_z$ 列完全相同：$\\{s\\}$ 與 $\\{b\\}$ 的 $z$ 軸平行，同一個角速度在兩邊的 $z$ 分量自然一樣。`,
    simHint: '隨意拖動關節角，盯著 ‖J_s − Ad·J_b‖ 的讀數；Ad 矩陣第一欄下方兩格，正是末端位置 (p_y, −p_x)。',
  },
  {
    title: 'v_s 不是末端速度，以及下一課',
    md: `最常見的誤解：$\\vV_s$ 的線速度 $v_s$ **不是**末端點的速度，而是「把末端剛體想像成無限延伸後，此刻恰好位於 $\\{s\\}$ 原點的那一點」的速度。末端點還要加上轉動造成的部分：

$$\\dot p_{\\text{tip}} = v_s + \\omega_s \\times p_{\\text{tip}}$$

相對地，$\\vV_b$ 的 $v_b$ 正是末端點速度（寫在 $\\{b\\}$ 中），因為 $\\{b\\}$ 的原點就在末端。最後按「伸直手臂」：三個軸點排成一直線，$\\det J_s = L_1L_2\\sin\\theta_2 = 0$，Jacobian 掉秩，這就是下一課的**奇異點**。接著做下方的自我檢測。`,
    simHint: '比較原點的粉紅箭頭 v_s 與末端的藍色 ṗ：末端處的 v_s ＋ ω×p 恰好拼成 ṗ。最後按「伸直手臂」，看奇異檢查的 det J_s。',
  },
];

const quiz: QuizQuestion[] = [
  {
    question: '平面 3R 手臂（$J_s$ 取 $\\omega_z, v_x, v_y$ 三列，為 3×3）。只轉動 $\\theta_3$ 時，$J_s$ 的第 3 欄會改變嗎？只轉動 $\\theta_1$ 時呢？',
    options: ['兩種情況都會改變', '轉 $\\theta_3$ 時不變；轉 $\\theta_1$ 時會變', '轉 $\\theta_3$ 時會變；轉 $\\theta_1$ 時不變', '兩種情況都不變'],
    answer: 1,
    explanation: '$J_{s,3} = [\\Ad_{e^{[\\vS_1]\\theta_1}e^{[\\vS_2]\\theta_2}}]\\,\\vS_3$ 只含 $\\theta_1, \\theta_2$：關節 3 的軸位置由前兩個關節決定，關節 3 自己轉並不會搬動自己的軸。轉 $\\theta_1$ 則把關節 3 的位置 $q_3$ 繞原點轉走，欄中的 $(q_y, -q_x)$ 跟著改變。模擬裡「剛才拖動」的提示可以直接驗證。',
  },
  {
    question: '平面 3R 的 $J_s$ 第一列（$\\omega_z$ 列）是 $[\\,1\\;\\;1\\;\\;1\\,]$。要讓末端只平移、不轉動，$\\dot\\theta$ 必須滿足什麼？',
    options: ['$\\dot\\theta_1 = \\dot\\theta_2 = \\dot\\theta_3$', '$\\dot\\theta_1 + \\dot\\theta_2 + \\dot\\theta_3 = 0$', '$\\dot\\theta_3 = 0$', '做不到：旋轉關節一定會讓末端轉動'],
    answer: 1,
    explanation: '末端角速度 $\\omega_z$ 是 Jacobian 第一列與 $\\dot\\theta$ 的內積，也就是 $\\sum_i \\dot\\theta_i$。只要總和為零（例如 $(0.5, -1, 0.5)$），三個轉動疊加後角速度互相抵消，只剩平移；此時瞬時轉心 ✕ 跑到無窮遠，$v_s$ 也剛好等於末端點速度。',
  },
  {
    question: '把末端座標系 $\\{b\\}$ 沿 $\\hat x_b$ 再往外移 0.2 m（例如裝上工具，姿態不變）。下列何者正確？',
    options: ['$J_s$ 與 $J_b$ 都不變', '只有 $J_s$ 改變', '$J_s$ 不變；$J_b$ 的線速度列改變、角速度列不變', '兩者的角速度列都會改變'],
    answer: 2,
    explanation: '$J_s$ 的欄是各關節軸寫在 $\\{s\\}$ 中的螺旋軸，公式裡根本沒有 $M$，所以和 $\\{b\\}$ 放在哪裡無關。$J_b$ 的欄是同一組軸寫在 $\\{b\\}$ 中：原點移動後，軸相對於 $\\{b\\}$ 原點的位置 $q^b$ 改變，$v = -\\omega \\times q^b$ 跟著改變；姿態沒變，所以 $\\omega$ 列不變。',
  },
  {
    question: '模擬中只讓關節 2 以 1 rad/s 轉動（$\\dot\\theta = e_2$）。$\\vV_s$ 的線速度部分 $v_s$ 代表什麼？',
    options: ['末端點的速度', '關節 2 本身的移動速度', '與末端那段剛體固連、此刻位於 $\\{s\\}$ 原點的那一點的速度', '一定為零，因為基座固定不動'],
    answer: 2,
    explanation: '此時關節 2 之後的剛體繞 $q_2$ 旋轉。把這個剛體想像成延伸到 $\\{s\\}$ 原點，原點那一點的速度是 $\\hat z \\times (0 - q_2)$，這就是 $v_s$。它一般不為零：真正的基座雖然不動，但基座並不屬於這個轉動的剛體。末端點的速度還要再加上 $\\omega \\times p_{\\text{tip}}$。',
  },
];

// ---------------------------------------------------------------------------
// Simulation configuration
// ---------------------------------------------------------------------------

type ModelKey = 'planar3R' | 'rrr3';
type JMode = 'Js' | 'Jb';

const MODELS: Record<ModelKey, RobotModel> = { planar3R, rrr3 };
const DEG = Math.PI / 180;
const deg3 = (a: number, b: number, c: number): number[] => [a * DEG, b * DEG, c * DEG];

/** Joint colours (CSS variables); kept away from the x/y/z axis colours. */
const JOINT_VARS = ['--series-4', '--series-5', '--series-6'];
const JOINT_CSS = JOINT_VARS.map((v) => `var(${v})`);
const VS_VAR = '--series-7';
const WXP_VAR = '--series-8';
/** Arrow length = velocity × K seconds (2D / 3D). */
const K2 = 0.5;
const K3 = 0.5;
/** 2D window (aspect 1.7): centred on the preset pose, most of the upper workspace stays visible. */
const WORLD = { xmin: -1.75, xmax: 2.55, ymin: -0.78, ymax: 1.75 };
/** Tighter window for narrow (phone) layouts, where the pane is not sticky and can be taller. */
const WORLD_NARROW = { xmin: -0.85, xmax: 2.25, ymin: -0.78, ymax: 1.75 };
/** Where the {b} origin sits on screen when the camera rides on {b} (2D). */
const ANCHOR: Pt = [1.2, 0.12];
const PULSE_T = 1.4;
const PULSE_A = 0.35;
const ROWS_2D = ['ωz', 'vx', 'vy'];
const ROWS_3D = ['ωx', 'ωy', 'ωz', 'vx', 'vy', 'vz'];
const NONE = [false, false, false];

interface Preset {
  label: string;
  title: string;
  theta: Record<ModelKey, number[]>;
  thetadot: number[];
  mode: JMode;
  body: boolean;
  showVs: boolean;
  showAdj: boolean;
  /** Joint whose column / axis is emphasised (−1: none). */
  selected: number;
}

/**
 * One generic pose for every step. At (15°, 55°, 90°) the three tip-velocity contributions of
 * θ̇ = (1, 0.7, 1) turn by ~50° from one to the next, so the head-to-tail chain and its resultant
 * are easy to tell apart (picked by a small search over poses and rates).
 */
const POSE: Record<ModelKey, number[]> = { planar3R: deg3(15, 55, 90), rrr3: deg3(35, -35, 70) };
const MIX = [1, 0.7, 1];
const PRESETS: Preset[] = [
  { label: '① 單關節', title: 'θ̇ = e₂：twist 就是 J 的第 2 欄', theta: POSE, thetadot: [0, 1, 0], mode: 'Js', body: false, showVs: false, showAdj: false, selected: 1 },
  { label: '② J_s 的欄', title: '拖動 θ₁ / θ₃，觀察哪些欄改變', theta: POSE, thetadot: [0, 0, 1], mode: 'Js', body: false, showVs: false, showAdj: false, selected: 2 },
  { label: '③ 疊加', title: '三個關節同時轉：分量頭尾相接', theta: POSE, thetadot: MIX, mode: 'Js', body: false, showVs: false, showAdj: false, selected: -1 },
  { label: '④ J_b 視角', title: '相機黏在 {b}，改看 J_b', theta: POSE, thetadot: MIX, mode: 'Jb', body: true, showVs: false, showAdj: false, selected: -1 },
  { label: '⑤ Adjoint', title: 'J_s = [Ad_Tsb] J_b 的數值檢查', theta: POSE, thetadot: MIX, mode: 'Js', body: false, showVs: false, showAdj: true, selected: -1 },
  { label: '⑥ v_s 與 ṗ', title: 'twist 的 v_s 與末端點速度並列', theta: POSE, thetadot: [0.3, 0.4, 0.5], mode: 'Js', body: false, showVs: true, showAdj: false, selected: -1 },
];

const limitsFor = (k: ModelKey): [number, number][] =>
  MODELS[k].jointLimits ?? [[-Math.PI, Math.PI], [-Math.PI, Math.PI], [-Math.PI, Math.PI]];

const fmtSci = (x: number): string => (x === 0 ? '0' : x.toExponential(1));

type Anim = { kind: 'none' } | { kind: 'pulse'; joint: number; t: number; theta0: number[] } | { kind: 'play' };

/** Compact joint-axis glyph (the toolkit's ScrewAxis3D is sized for ~1 m scenes; this arm is 0.7 m). */
interface AxisGlyph3D {
  line: ReturnType<Scene3D['addPolyline']>;
  arrow: Arrow3D;
  dot: ReturnType<Scene3D['addSphere']>;
}

interface View3D {
  scene: Scene3D;
  robot: Robot3D;
  axes: AxisGlyph3D[];
  contrib: Arrow3D[];
  tip: Arrow3D;
  vs: Arrow3D;
  vsCopy: Arrow3D;
  wxp: Arrow3D;
  levers: ReturnType<Scene3D['addPolyline']>[];
  trail: ReturnType<Scene3D['addPolyline']>;
  tipLabel: ReturnType<Scene3D['addLabel']>;
  vsLabel: ReturnType<Scene3D['addLabel']>;
}

// ---------------------------------------------------------------------------
// mount
// ---------------------------------------------------------------------------

function mount(host: HTMLElement, ctx: LessonContext): LessonHandle {
  const S = {
    model: 'planar3R' as ModelKey,
    theta: POSE.planar3R.slice(),
    thetadot: [0, 1, 0],
    mode: 'Js' as JMode,
    bodyView: false,
    showVs: false,
    showAdj: false,
    selected: 1,
  };
  let snap: JacobianSnapshot = jacobianSnapshot(planar3R, S.theta);
  let vel: VelocityBreakdown = velocityBreakdown(snap, S.thetadot);
  let trail: Vec[] = [];
  let lastChange: { joint: number; s: boolean[]; b: boolean[] } | null = null;
  let status = '';
  let anim: Anim = { kind: 'none' };
  let playStart: number[] | null = null;
  let currentStep = ctx.currentStep;
  let cv: Canvas2D | null = null;
  let view3: View3D | null = null;
  let glyphPos: Pt[] = [];

  // ---- layout ---------------------------------------------------------
  const titleEl = simTitle(host, 'Jacobian 的每一欄 = 關節此刻的螺旋軸', '平面 3R');
  const chips = presetChips(host, PRESETS.map((p, i) => ({ label: p.label, title: p.title, apply: () => applyPreset(i) })), { label: '情境' });

  const optPanel = controlPanel(host);
  const optRow = row(optPanel);
  const modelSel = select<ModelKey>(optRow, {
    options: [{ value: 'planar3R', label: '平面 3R' }, { value: 'rrr3', label: '空間 RRR' }],
    value: 'planar3R',
    onChange: (k) => setModel(k),
  });
  const modeSel = select<JMode>(optRow, {
    options: [{ value: 'Js', label: 'J_s（{s}）' }, { value: 'Jb', label: 'J_b（{b}）' }],
    value: 'Js',
    onChange: (m) => { S.mode = m; manual(); syncControls(); refresh(); },
  });
  modelSel.el.title = '模型：平面 3R（2D 畫布）或空間 RRR（3D 場景）';
  modeSel.el.title = '用哪個 Jacobian：J_s 寫在 {s} 中，J_b 寫在 {b} 中';
  const bodyTog = toggle(optRow, { label: '相機黏在 {b}', title: '以末端座標系 {b} 為固定視角', onChange: (v) => { S.bodyView = v; manual(); refresh(); } });
  const vsTog = toggle(optRow, { label: 'v_s 分解', title: '顯示 v_s 與 ṗ = v_s + ω×p', onChange: (v) => { S.showVs = v; manual(); refresh(); } });
  const adjTog = toggle(optRow, { label: 'Adjoint', title: '改為顯示 J_s = [Ad_Tsb] J_b 的數值檢查', onChange: (v) => { S.showAdj = v; manual(); syncControls(); refresh(); } });

  const viewHost = el('div');
  host.appendChild(viewHost);

  // θ (row 1) and θ̇ (row 2) share one 3-column grid so each column belongs to one joint
  const thetaPanel = jointSliders(host, {
    n: 3,
    values: S.theta,
    limits: limitsFor('planar3R'),
    onInput: (th, i) => onThetaInput(th, i),
  });
  thetaPanel.el.style.gridTemplateColumns = 'repeat(3, minmax(0, 1fr))';
  const rateSliders = [0, 1, 2].map((i) => slider(thetaPanel.el, {
    label: `θ̇${subscript(i + 1)}`, min: -1, max: 1, step: 0.05, value: S.thetadot[i], unit: 'rad/s',
    onInput: (v) => onRateInput(i, v),
  }));

  const actPanel = controlPanel(host);
  const actRow = row(actPanel);
  actRow.appendChild(el('span', { class: 'mr-note', style: { margin: '0' } }, '脈衝'));
  [0, 1, 2].forEach((i) => button(actRow, {
    label: `θ̇${subscript(i + 1)} = 1`, small: true, title: `只讓關節 ${i + 1} 以 1 rad/s 轉動，並讓它擺動一下`, onClick: () => pulse(i),
  }));
  const pb = playback(actRow, {
    showTime: false,
    onPlay: () => startPlay(),
    onPause: () => stopAnim(),
    onReset: () => resetPlay(),
    onSpeed: (s) => { loop.speed = s; },
  });
  button(actRow, { label: '伸直手臂', small: true, title: '平面 3R：θ₂ = θ₃ = 0；空間 RRR：θ₃ = 0（奇異預告）', onClick: () => straighten() });

  // V = J θ̇ in numbers
  const eq1 = eqBlock(host, '\\vV_s = J_s(\\theta)\\,\\dot\\theta', true);
  const vMat = liveMatrix(eq1.body, { rowLabels: ROWS_2D });
  eqSym(eq1.body, '=');
  const jMat = liveMatrix(eq1.body, { title: '欄 i = 關節 i 此刻的螺旋軸' });
  const rMat = liveMatrix(eq1.body, { title: 'θ̇ (rad/s)' });
  jMat.colorColumns(JOINT_CSS);
  rMat.colorRows(JOINT_CSS);

  // J_s = [Ad_Tsb] J_b in numbers (replaces the row above while the "Adjoint" toggle is on)
  const eq2 = eqBlock(host, 'J_s(\\theta) = [\\Ad_{T_{sb}}]\\,J_b(\\theta)', false);
  const jsMat2 = liveMatrix(eq2.body, { title: 'J_s' });
  eqSym(eq2.body, '=');
  const adMat = liveMatrix(eq2.body, { title: '[Ad_Tsb]' });
  const jbMat2 = liveMatrix(eq2.body, { title: 'J_b' });
  jsMat2.colorColumns(JOINT_CSS);
  jbMat2.colorColumns(JOINT_CSS);
  const resid = readout(eq2.head, { label: '‖J_s − Ad·J_b‖ =', title: '浮點誤差等級即代表兩邊相等' });

  const changeEl = note(host, '');
  const K_TIP = 'ṗ 末端點速度（{s}）';
  const K_DEC = 'v_s ＋ ω×p（{s}）';
  const K_DEC_B = 'v_b = Rᵀṗ（{b}）';
  const K_SING = '奇異檢查';
  const info = kv(host, [K_TIP, K_DEC, K_SING]);
  const decLabel = info.el.querySelectorAll('dt')[1] as HTMLElement | undefined;
  const footNote = note(host, `3D：彩色虛線軸 = J_s 各欄（此刻的關節軸）· 拖曳可旋轉視角 · 箭頭 = 速度 × ${K3} s`);

  // ---- views ----------------------------------------------------------
  function buildPlanar(): void {
    const narrow = host.clientWidth > 0 && host.clientWidth < 520;
    const world = narrow ? WORLD_NARROW : WORLD;
    cv = new Canvas2D(viewHost, { aspect: (world.xmax - world.xmin) / (world.ymax - world.ymin), world, overlay: true });
    cv.el.appendChild(el('div', { class: 'mr-overlay', style: { top: 'auto', bottom: '0.35rem' } }, `箭頭 = 速度 × ${K2} s · 點一下 ⊙ 可做脈衝`));
    cv.draw = drawPlanar;
    cv.onPointer({
      down: (p) => {
        for (let i = 0; i < glyphPos.length; i++) {
          const q = glyphPos[i];
          if (cv && Math.hypot(p[0] - q[0], p[1] - q[1]) * cv.scale < 16) { pulse(i); return; }
        }
      },
    });
  }

  function build3D(): void {
    const scene = new Scene3D(viewHost, { aspect: 2.1, grid: 2, axes: 0.16, cameraPos: [1.12, -0.3, 0.72], target: [0.28, 0.14, 0.22], autoStart: false });
    const robot = new Robot3D(scene, rrr3, { frameSize: 0.1, linkRadius: 0.016 });
    const jc = JOINT_VARS.map((v) => cssVar(v, '#888'));
    const axes = jc.map((color) => ({
      line: scene.addPolyline([[0, 0, 0], [0, 0, 1]], color, { dashed: true }),
      arrow: scene.addArrow({ color, radius: 0.006, head: 0.035 }),
      dot: scene.addSphere(0.03, color),
    }));
    const contrib = jc.map((color) => scene.addArrow({ color, radius: 0.008, head: 0.04 }));
    const tip = scene.addArrow({ color: cssVar('--accent', '#2563eb'), radius: 0.011, head: 0.05 });
    const pink = cssVar(VS_VAR, '#db2777');
    const vs = scene.addArrow({ color: pink, radius: 0.01, head: 0.045 });
    const vsCopy = new Arrow3D(pink, 0.006, 0.035, 0.45);
    scene.scene.add(vsCopy.group);
    const wxp = scene.addArrow({ color: cssVar(WXP_VAR, '#4b5563'), radius: 0.008, head: 0.04 });
    const levers = jc.map((color) => scene.addPolyline([[0, 0, 0], [0, 0, 0]], color, { dashed: true }));
    const trailLine = scene.addPolyline([[0, 0, 0], [0, 0, 0]], cssVar('--warn', '#b45309'));
    const tipLabel = scene.addLabel('ṗ', [0, 0, 0], cssVar('--accent', '#2563eb'));
    const vsLabel = scene.addLabel('v_s', [0, 0, 0], pink);
    // the toolkit's text sprites ({s}, {b}, labels) are sized for ~1 m scenes: shrink them for this 0.7 m arm
    scene.scene.traverse((o) => { if ((o as { isSprite?: boolean }).isSprite) o.scale.multiplyScalar(0.6); });
    view3 = { scene, robot, axes, contrib, tip, vs, vsCopy, wxp, levers, trail: trailLine, tipLabel, vsLabel };
  }

  function teardownView(): void {
    cv?.destroy();
    cv = null;
    // dispose the scene first so the robot's meshes are still attached when geometries are freed
    if (view3) { view3.scene.destroy(); view3.robot.destroy(); view3 = null; }
    viewHost.replaceChildren();
    glyphPos = [];
  }

  // ---- 2D drawing -----------------------------------------------------
  function drawPlanar(c: Canvas2D): void {
    const muted = cssVar('--fg-muted', '#777');
    const jc = JOINT_VARS.map((v) => cssVar(v, '#888'));
    const accent = cssVar('--accent', '#2563eb');
    const pink = cssVar(VS_VAR, '#db2777');
    const slate = cssVar(WXP_VAR, '#4b5563');
    const pill = cssVar('--sim-bg', '#fff');
    const fk = planarFK(planar3R.linkLengths!, S.theta);
    const tip = fk.tip;
    const phi = fk.phi;
    const body = S.bodyView;
    const ch = Math.cos(phi);
    const sh = Math.sin(phi);
    // {s} point -> drawing coordinates (identity, or "camera attached to {b}")
    const P = (q: ArrayLike<number>): Pt => (body
      ? [ANCHOR[0] + ch * (q[0] - tip[0]) + sh * (q[1] - tip[1]), ANCHOR[1] - sh * (q[0] - tip[0]) + ch * (q[1] - tip[1])]
      : [q[0], q[1]]);
    const Pinv = (d: Pt): Pt => (body
      ? [tip[0] + ch * (d[0] - ANCHOR[0]) - sh * (d[1] - ANCHOR[1]), tip[1] + sh * (d[0] - ANCHOR[0]) + ch * (d[1] - ANCHOR[1])]
      : d);
    // free vector in {s} -> drawing coordinates, scaled by k
    const D = (v: ArrayLike<number>, k: number): Pt => (body
      ? [k * (ch * v[0] + sh * v[1]), k * (-sh * v[0] + ch * v[1])]
      : [k * v[0], k * v[1]]);
    const plus = (a: Pt, b: Pt): Pt => [a[0] + b[0], a[1] + b[1]];
    const len = (a: Pt): number => Math.hypot(a[0], a[1]);

    // ground
    if (body) drawWorldGrid(c, P, Pinv); else c.grid(0.5);
    drawBase(c, P);
    const o = P([0, 0]);
    c.frame({ x: o[0], y: o[1], theta: body ? -phi : 0 }, 0.4, { labels: 's', width: 2 });

    if (trail.length > 1) c.polyline(trail.map((q) => P(q)), { color: cssVar('--warn', '#b45309'), width: 2, alpha: 0.85 });

    // links
    const pts = fk.joints.map((q) => P(q));
    for (let i = 0; i < 3; i++) c.line(pts[i], pts[i + 1], { color: muted, width: 9, alpha: 0.5 });

    // joint axes recovered from the Jacobian columns (the column *is* the axis)
    const useS = S.mode === 'Js';
    const J = useS ? snap.Js : snap.Jb;
    const ref = useS ? o : pts[3];
    glyphPos = [];
    const local: Vec[] = [];
    for (let i = 0; i < 3; i++) {
      const q = axisPoint(getCol(J, i)) ?? [0, 0, 0];
      local.push(q);
      const qs: Pt = useS ? [q[0], q[1]] : [tip[0] + ch * q[0] - sh * q[1], tip[1] + sh * q[0] + ch * q[1]];
      glyphPos.push(P(qs));
    }
    for (let i = 0; i < 3; i++) {
      const sel = i === S.selected;
      c.line(ref, glyphPos[i], { color: jc[i], width: sel ? 2.2 : 1.3, dash: [6, 4], alpha: sel ? 0.95 : 0.5 });
    }

    // instantaneous centre of the resulting planar twist (multi-joint motion only)
    const moving = S.thetadot.filter((x) => Math.abs(x) > 1e-9).length;
    const chain = moving >= 2 && !S.showVs;
    const icr = chain ? instantCenter(planarTwist(vel.Vs)) : null;
    if (icr && Math.hypot(icr[0], icr[1]) < 8) {
      const ic = P(icr);
      const r = 7 / c.scale;
      c.line([ic[0] - r, ic[1] - r], [ic[0] + r, ic[1] + r], { color: muted, width: 2 });
      c.line([ic[0] - r, ic[1] + r], [ic[0] + r, ic[1] - r], { color: muted, width: 2 });
      c.text(ic, '瞬時轉心', { color: muted, dy: 14, font: `11px ${cssVar('--font') || 'sans-serif'}` });
    }

    // end-effector frame {b} (kept small so it does not hide the velocity arrows)
    c.frame({ x: pts[3][0], y: pts[3][1], theta: body ? 0 : phi }, 0.28, { labels: 'b', width: 1.8, alpha: 0.8 });

    // axis glyphs ⊙ with their joint number on the free side of the joint
    for (let i = 0; i < 3; i++) {
      const prev: Pt = i === 0 ? P([0, -0.3]) : pts[i - 1];
      axisGlyph(c, glyphPos[i], jc[i], i === S.selected, String(i + 1), outward(pts[i], prev, pts[i + 1]));
    }

    // coordinates of the emphasised axis point relative to the reference frame
    if (S.selected >= 0) {
      const i = S.selected;
      const q = local[i];
      const mid: Pt = [(ref[0] + glyphPos[i][0]) / 2, (ref[1] + glyphPos[i][1]) / 2];
      const name = useS ? `q${subscript(i + 1)}` : `q${subscript(i + 1)}ᵇ`;
      c.text(mid, `${name} = (${fmt(q[0], 2)}, ${fmt(q[1], 2)})`, { color: jc[i], bg: pill, dy: -2 });
    }

    // velocity arrows: per-joint contributions chained head-to-tail, then the total
    if (chain) {
      let start = pts[3];
      for (let i = 0; i < 3; i++) {
        const d = D(vel.contributions[i].tipVel, K2);
        if (len(d) < 1e-6) continue;
        const end = plus(start, d);
        c.arrow(start, end, { color: jc[i], width: 2.5, head: 9 });
        start = end;
      }
    }
    const tipV = D(vel.tipVel, K2);
    if (len(tipV) > 1e-6) {
      const end = plus(pts[3], tipV);
      c.arrow(pts[3], end, { color: accent, width: 3.5, head: 12 });
      tag(c, end, useS ? 'ṗ' : 'ṗ（= v_b）', { color: accent, side: Math.sign(tipV[0]) || 1, gap: 5, dy: -10, bg: pill });
    }

    // v_s (velocity of the body point at the {s} origin) and ṗ = v_s + ω × p at the tip
    if (S.showVs) {
      const vs = D(vel.vS, K2);
      const wxp = D(vel.omegaCrossP, K2);
      c.line(o, pts[3], { color: pink, width: 1.2, dash: [2, 5], alpha: 0.6 });
      if (len(vs) > 1e-6) {
        c.arrow(o, plus(o, vs), { color: pink, width: 3, head: 11 });
        tag(c, plus(o, vs), 'v_s', { color: pink, side: 1, gap: 6, dy: 6, bg: pill });
        const a = plus(pts[3], vs);
        c.arrow(pts[3], a, { color: pink, width: 2, dash: [5, 4], alpha: 0.75, head: 8 });
        if (len(wxp) > 1e-6) {
          c.arrow(a, plus(a, wxp), { color: slate, width: 2.5, head: 9 });
          tag(c, plus(a, [wxp[0] / 2, wxp[1] / 2]), 'ω×p', { color: slate, side: 1, gap: 8, dy: 8, bg: pill });
        }
      } else {
        c.text(o, 'v_s = 0', { color: pink, dx: 34, dy: 16, bg: pill });
      }
    }

    const w = vel.omegaS[2];
    const vNorm = Math.hypot(vel.vS[0], vel.vS[1]);
    const pure = Math.abs(w) < 1e-9 && vNorm > 1e-9 ? ' · ω = 0：純平移' : '';
    c.overlay.textContent = `${useS ? 'J_s：軸點相對 {s} 原點' : 'J_b：軸點相對 {b} 原點'}${body ? ' · 相機黏在 {b}' : ''}${pure}`;
  }

  function drawBase(c: Canvas2D, P: (q: ArrayLike<number>) => Pt): void {
    const g = cssVar('--grid-strong', '#bbb');
    const m = cssVar('--fg-muted', '#777');
    c.polygon([P([-0.13, -0.17]), P([0.13, -0.17]), P([0, 0])], { fill: g, color: g, width: 1 });
    c.line(P([-0.22, -0.17]), P([0.22, -0.17]), { color: m, width: 2 });
    for (let k = 0; k < 5; k++) {
      const x = -0.2 + k * 0.1;
      c.line(P([x, -0.17]), P([x - 0.07, -0.25]), { color: m, width: 1 });
    }
  }

  /** The {s} ground grid drawn through the camera transform (used when the camera rides on {b}). */
  function drawWorldGrid(c: Canvas2D, P: (q: ArrayLike<number>) => Pt, Pinv: (d: Pt) => Pt): void {
    const v = c.visible();
    const corners = ([[v.xmin, v.ymin], [v.xmax, v.ymin], [v.xmax, v.ymax], [v.xmin, v.ymax]] as Pt[]).map(Pinv);
    const xs = corners.map((q) => q[0]);
    const ys = corners.map((q) => q[1]);
    const [x0, x1, y0, y1] = [Math.min(...xs), Math.max(...xs), Math.min(...ys), Math.max(...ys)];
    const step = 0.5;
    const grid = cssVar('--grid', '#e5e5e5');
    const strong = cssVar('--grid-strong', '#ccc');
    for (let k = Math.ceil(x0 / step); k <= Math.floor(x1 / step); k++) {
      c.line(P([k * step, y0]), P([k * step, y1]), { color: k === 0 ? strong : grid, width: 1 });
    }
    for (let k = Math.ceil(y0 / step); k <= Math.floor(y1 / step); k++) {
      c.line(P([x0, k * step]), P([x1, k * step]), { color: k === 0 ? strong : grid, width: 1 });
    }
  }

  /** Unit direction pointing away from both neighbouring links of a joint (for its label). */
  function outward(at: Pt, a: Pt, b: Pt): Pt {
    const unit = (p: Pt): Pt => { const n = Math.hypot(p[0] - at[0], p[1] - at[1]) || 1; return [(p[0] - at[0]) / n, (p[1] - at[1]) / n]; };
    const ua = unit(a);
    const ub = unit(b);
    const s: Pt = [ua[0] + ub[0], ua[1] + ub[1]];
    const n = Math.hypot(s[0], s[1]);
    return n < 1e-6 ? [-ub[1], ub[0]] : [-s[0] / n, -s[1] / n];
  }

  /**
   * Label next to an anchor point: its near edge sits `gap` px to the preferred side, and it
   * flips to the other side (or vertically) when it would leave the canvas.
   */
  function tag(c: Canvas2D, p: Pt, text: string, st: { color: string; side: number; gap: number; dy: number; bg: string }): void {
    c.ctx.font = `12px ${cssVar('--font') || 'sans-serif'}`;
    const w = c.ctx.measureText(text).width + 8; // + the pill padding
    const [sx, sy] = c.toScreen(p);
    let side = st.side >= 0 ? 1 : -1;
    if (side > 0 && sx + st.gap + w > c.width - 2) side = -1;
    else if (side < 0 && sx - st.gap - w < 2) side = 1;
    let dy = st.dy;
    if (sy + dy < 10) dy = Math.abs(dy);
    else if (sy + dy > c.height - 10) dy = -Math.abs(dy);
    c.text(p, text, { color: st.color, dx: side * (st.gap + w / 2), dy, bg: st.bg });
  }

  function axisGlyph(c: Canvas2D, pos: Pt, color: string, emph: boolean, label: string, dir: Pt): void {
    const rpx = emph ? 12 : 9.5;
    c.circle(pos, rpx / c.scale, { color, width: emph ? 3 : 2, fill: cssVar('--bg-elev', '#fff') });
    c.point(pos, emph ? 3.6 : 2.8, { color });
    const d = rpx + 9;
    c.text(pos, label, { color, dx: dir[0] * d, dy: -dir[1] * d, font: `bold 12px ${cssVar('--font') || 'sans-serif'}` });
  }

  // ---- 3D drawing -----------------------------------------------------
  function update3D(): void {
    const v = view3;
    if (!v) return;
    v.robot.setConfig(S.theta);
    const p = snap.p;
    const add = (a: Vec, b: Vec, k = 1): Vec => [a[0] + k * b[0], a[1] + k * b[1], a[2] + k * b[2]];
    // joint axes drawn straight from the J_s columns: direction ω_i through the point q_i closest to {s}
    for (let i = 0; i < 3; i++) {
      const col = getCol(snap.Js, i);
      const q = axisPoint(col) ?? [0, 0, 0];
      const w: Vec = [col[0], col[1], col[2]];
      const a = v.axes[i];
      a.line.update([add(q, w, -0.32), add(q, w, 0.32)]);
      a.arrow.setDir(q, w, 0.2);
      a.dot.position.set(q[0], q[1], q[2]);
    }
    const moving = S.thetadot.filter((x) => Math.abs(x) > 1e-9).length;
    let start = p;
    for (let i = 0; i < 3; i++) {
      const end = add(start, vel.contributions[i].tipVel, K3);
      v.contrib[i].setFromTo(start, end);
      v.contrib[i].group.visible = v.contrib[i].group.visible && moving >= 2 && !S.showVs;
      start = end;
    }
    const tipEnd = add(p, vel.tipVel, K3);
    v.tip.setFromTo(p, tipEnd);
    v.tipLabel.position.set(tipEnd[0], tipEnd[1], tipEnd[2] + 0.04);
    v.tipLabel.visible = v.tip.group.visible;
    const o: Vec = [0, 0, 0];
    const vsEnd = add(o, vel.vS, K3);
    v.vs.setFromTo(o, vsEnd);
    const mid = add(p, vel.vS, K3);
    v.vsCopy.setFromTo(p, mid);
    v.wxp.setFromTo(mid, add(mid, vel.omegaCrossP, K3));
    v.vsLabel.position.set(vsEnd[0], vsEnd[1], vsEnd[2] + 0.04);
    for (const a of [v.vs, v.vsCopy, v.wxp]) a.group.visible = a.group.visible && S.showVs;
    v.vsLabel.visible = S.showVs;
    // position vectors from the reference origin to the closest point of each axis
    const useS = S.mode === 'Js';
    const ref = useS ? o : p;
    for (let i = 0; i < 3; i++) {
      const q = axisPoint(getCol(useS ? snap.Js : snap.Jb, i)) ?? [0, 0, 0];
      const qs = useS ? q : matVec(snap.T, [q[0], q[1], q[2], 1]).slice(0, 3);
      v.levers[i].update([ref, qs]);
    }
    v.trail.update(trail.length > 1 ? trail : [p, p]);
    v.scene.requestRender();
  }

  // ---- numbers --------------------------------------------------------
  function refresh(): void {
    snap = jacobianSnapshot(MODELS[S.model], S.theta);
    vel = velocityBreakdown(snap, S.thetadot);
    if (anim.kind !== 'none') {
      trail.push(snap.p.slice());
      if (trail.length > 1500) trail.splice(0, trail.length - 1500);
    }
    const planar = S.model === 'planar3R';
    const pick = (J: Mat): Mat => (planar ? planarRows(J) : J);
    const useS = S.mode === 'Js';
    if (S.showAdj) {
      jsMat2.set(pick(snap.Js));
      adMat.set(planar ? planarAdjoint(snap.Ad) : snap.Ad);
      jbMat2.set(pick(snap.Jb));
      jsMat2.markColumns(lastChange ? lastChange.s : NONE);
      jbMat2.markColumns(lastChange ? lastChange.b : NONE);
      resid.set(fmtSci(snap.residual));
    } else {
      const V = useS ? vel.Vs : vel.Vb;
      vMat.set(planar ? planarTwist(V) : V);
      jMat.set(pick(useS ? snap.Js : snap.Jb));
      rMat.set(S.thetadot);
      jMat.emphasizeColumns([0, 1, 2].map((i) => i === S.selected));
      jMat.markColumns(lastChange ? (useS ? lastChange.s : lastChange.b) : NONE);
    }
    const dims = planar ? 2 : 3;
    info.set(K_TIP, `${fmtVec(vel.tipVel.slice(0, dims), 2)} m/s`);
    if (decLabel) {
      decLabel.textContent = useS ? K_DEC : K_DEC_B;
      const show = S.showAdj ? 'none' : '';
      decLabel.style.display = show;
      if (decLabel.nextElementSibling instanceof HTMLElement) decLabel.nextElementSibling.style.display = show;
    }
    info.set(K_DEC, useS
      ? `${fmtVec(vel.vS.slice(0, dims), 2)} ＋ ${fmtVec(vel.omegaCrossP.slice(0, dims), 2)}`
      : `${fmtVec(vel.vB.slice(0, dims), 2)} m/s`);
    const d = singularityMeasure(snap, planar);
    const name = planar ? 'det J_s' : 'det J_v（末端點）';
    info.set(K_SING, `${name} = ${fmt(d, 3)}${Math.abs(d) < 1e-3 ? '  ≈ 0 → 奇異，Jacobian 掉秩' : ''}`);
    changeEl.textContent = describeChange();
    cv?.requestRender();
    update3D();
  }

  function describeChange(): string {
    if (status) return status;
    if (!lastChange) return '拖動任一個 θ 滑桿，這裡會列出 J_s 與 J_b 中跟著改變的欄（矩陣中以底色標出）。';
    const list = (f: boolean[]): string => {
      const idx = f.map((x, i) => (x ? i + 1 : 0)).filter((i) => i > 0);
      return idx.length ? `第 ${idx.join('、')} 欄改變` : '完全不變';
    };
    return `剛才拖動 θ${subscript(lastChange.joint + 1)}：J_s ${list(lastChange.s)}；J_b ${list(lastChange.b)}。`;
  }

  // ---- interaction ----------------------------------------------------
  function manual(): void { chips.clear(); }

  function syncControls(): void {
    thetaPanel.set(S.theta, { silent: true });
    rateSliders.forEach((s, i) => s.set(S.thetadot[i], { silent: true }));
    modeSel.set(S.mode, { silent: true });
    bodyTog.set(S.bodyView, { silent: true });
    vsTog.set(S.showVs, { silent: true });
    adjTog.set(S.showAdj, { silent: true });
    bodyTog.el.style.display = S.model === 'planar3R' ? '' : 'none';
    footNote.style.display = S.model === 'planar3R' ? 'none' : '';
    eq1.show(!S.showAdj);
    eq2.show(S.showAdj);
    eq1.setTex(S.mode === 'Js' ? '\\vV_s = J_s(\\theta)\\,\\dot\\theta' : '\\vV_b = J_b(\\theta)\\,\\dot\\theta');
  }

  function onThetaInput(th: number[], i: number): void {
    // Stopping a pulse restores its start pose; keep that for the joints the learner did not touch.
    stopAnim();
    playStart = null;
    status = '';
    const prev = jacobianSnapshot(MODELS[S.model], S.theta);
    if (i >= 0) S.theta[i] = th[i]; else S.theta = th.slice();
    thetaPanel.set(S.theta, { silent: true });
    snap = jacobianSnapshot(MODELS[S.model], S.theta);
    if (i >= 0) lastChange = { joint: i, s: changedColumns(prev.Js, snap.Js), b: changedColumns(prev.Jb, snap.Jb) };
    trail = [];
    manual();
    refresh();
  }

  function onRateInput(i: number, v: number): void {
    S.thetadot[i] = v;
    const nz = S.thetadot.map((x, k) => (Math.abs(x) > 1e-9 ? k : -1)).filter((k) => k >= 0);
    S.selected = nz.length === 1 ? nz[0] : -1;
    manual();
    refresh();
  }

  function applyPreset(i: number): void {
    const p = PRESETS[i];
    stopAnim();
    S.theta = p.theta[S.model].slice();
    S.thetadot = p.thetadot.slice();
    S.mode = p.mode;
    S.bodyView = p.body && S.model === 'planar3R';
    S.showVs = p.showVs;
    S.showAdj = p.showAdj;
    S.selected = p.selected;
    trail = [];
    lastChange = null;
    status = '';
    playStart = null;
    syncControls();
    refresh();
  }

  function setModel(k: ModelKey): void {
    stopAnim();
    S.model = k;
    modelSel.set(k, { silent: true });
    teardownView();
    if (k === 'planar3R') buildPlanar(); else build3D();
    const lim = limitsFor(k);
    thetaPanel.sliders.forEach((s, i) => s.setRange(lim[i][0], lim[i][1]));
    const pi = chips.active >= 0 ? chips.active : Math.min(currentStep, PRESETS.length - 1);
    S.theta = PRESETS[pi].theta[k].slice();
    if (k !== 'planar3R') S.bodyView = false;
    vMat.setRowLabels(k === 'planar3R' ? ROWS_2D : ROWS_3D);
    for (const m of [vMat, jMat, rMat]) m.el.style.fontSize = k === 'planar3R' ? '' : '0.72rem';
    for (const m of [jsMat2, adMat, jbMat2]) m.el.style.fontSize = k === 'planar3R' ? '' : '0.66rem';
    const badge = titleEl.querySelector('.badge');
    if (badge) badge.textContent = k === 'planar3R' ? '平面 3R' : '空間 RRR';
    trail = [];
    lastChange = null;
    status = '';
    playStart = null;
    syncControls();
    refresh();
  }

  function straighten(): void {
    stopAnim();
    playStart = null;
    const th = S.theta.slice();
    if (S.model === 'planar3R') { th[1] = 0; th[2] = 0; } else th[2] = 0;
    S.theta = th;
    trail = [];
    lastChange = null;
    status = '';
    manual();
    syncControls();
    refresh();
  }

  function pulse(i: number): void {
    const theta0 = anim.kind === 'pulse' ? anim.theta0 : S.theta.slice();
    if (anim.kind === 'play') pb.setPlaying(false, { silent: true });
    S.thetadot = [0, 0, 0];
    S.thetadot[i] = 1;
    S.selected = i;
    status = '';
    trail = [];
    anim = { kind: 'pulse', joint: i, t: 0, theta0 };
    syncControls();
    manual();
    loop.start();
    refresh();
  }

  function startPlay(): void {
    if (anim.kind === 'pulse') { S.theta = anim.theta0.slice(); thetaPanel.set(S.theta, { silent: true }); }
    if (!playStart) playStart = S.theta.slice();
    status = '';
    anim = { kind: 'play' };
    loop.start();
  }

  function stopAnim(): void {
    if (anim.kind === 'pulse') S.theta = anim.theta0.slice();
    anim = { kind: 'none' };
    loop.stop();
    pb.setPlaying(false, { silent: true });
    thetaPanel.set(S.theta, { silent: true });
  }

  function resetPlay(): void {
    stopAnim();
    if (playStart) S.theta = playStart.slice();
    playStart = null;
    trail = [];
    status = '';
    thetaPanel.set(S.theta, { silent: true });
    refresh();
  }

  const loop = createLoop((dt) => {
    if (anim.kind === 'pulse') {
      anim.t += dt;
      const k = Math.min(1, anim.t / PULSE_T);
      const th = anim.theta0.slice();
      const [lo, hi] = limitsFor(S.model)[anim.joint];
      th[anim.joint] = clamp(anim.theta0[anim.joint] + PULSE_A * Math.sin(2 * Math.PI * k), lo, hi);
      S.theta = th;
      if (k >= 1) {
        S.theta = anim.theta0.slice();
        anim = { kind: 'none' };
        loop.stop();
      }
      thetaPanel.set(S.theta, { silent: true });
      refresh();
    } else if (anim.kind === 'play') {
      // Kinematic integration θ += θ̇ dt; full-turn joints wrap, limited joints stop the playback.
      const lim = limitsFor(S.model);
      let hit = -1;
      S.theta = S.theta.map((x, j) => {
        const y = x + S.thetadot[j] * dt;
        const [lo, hi] = lim[j];
        if (hi - lo >= 2 * Math.PI - 1e-9) return wrapToPi(y);
        if (y < lo || y > hi) { hit = j; return clamp(y, lo, hi); }
        return y;
      });
      thetaPanel.set(S.theta, { silent: true });
      refresh();
      if (hit >= 0) {
        anim = { kind: 'none' };
        loop.stop();
        pb.setPlaying(false, { silent: true });
        status = `關節 ${hit + 1} 到達關節極限，播放已暫停（可按重設回到起點）。`;
        changeEl.textContent = status;
      }
    } else {
      loop.stop();
    }
  });

  // ---- step focus -----------------------------------------------------
  let focused: HTMLElement[] = [];
  function setFocus(i: number): void {
    focused.forEach((f) => f.classList.remove('mr-focus'));
    const targets: (HTMLElement | null)[][] = [
      [actRow, jMat.el],
      [thetaPanel.sliders[0].el, jMat.el, changeEl],
      [...rateSliders.map((r) => r.el), viewHost],
      [thetaPanel.sliders[0].el, jMat.el, changeEl],
      [eq2.el],
      [viewHost, info.el],
    ];
    focused = (targets[i] ?? []).filter((x): x is HTMLElement => !!x);
    focused.forEach((f) => f.classList.add('mr-focus'));
  }

  function onStep(i: number): void {
    currentStep = i;
    if (i >= 0 && i < PRESETS.length) chips.activate(i);
    setFocus(i);
  }

  const unsubTheme = ctx.onThemeChange(() => {
    // 3D materials capture colours at creation time: rebuild the scene for the new theme.
    if (S.model === 'rrr3') { teardownView(); build3D(); refresh(); }
  });

  buildPlanar();
  syncControls();
  onStep(Math.min(ctx.currentStep, steps.length - 1));

  return {
    onStep,
    destroy() {
      anim = { kind: 'none' };
      loop.stop();
      unsubTheme();
      teardownView();
    },
  };
}

// ---------------------------------------------------------------------------
// small DOM helpers
// ---------------------------------------------------------------------------

/** A formula followed by its numeric matrices, either on one line (`inline`) or below it. */
function eqBlock(parent: HTMLElement, tex: string, inline: boolean): { el: HTMLDivElement; head: HTMLDivElement; body: HTMLDivElement; setTex(t: string): void; show(v: boolean): void } {
  const flex = { display: 'flex', flexWrap: 'wrap', alignItems: 'center', gap: '0.15rem 0.3rem' };
  const formula = el('span', {}, mathEl(tex));
  const head = el('div', { style: { display: 'flex', flexWrap: 'wrap', alignItems: 'center', gap: '0.2rem 1rem', marginRight: '0.35rem', fontSize: '0.92rem' } }, formula);
  const body = el('div', { style: flex });
  const wrap = el('div', { style: inline ? { ...flex, margin: '0.35rem 0' } : { margin: '0.35rem 0' } }, head, body);
  parent.appendChild(wrap);
  let current = tex;
  return {
    el: wrap,
    head,
    body,
    setTex(t) { if (t !== current) { current = t; formula.replaceChildren(mathEl(t)); } },
    // keep the inline flex layout when showing again (an empty display would fall back to block)
    show(v) { wrap.style.display = v ? (inline ? 'flex' : 'block') : 'none'; },
  };
}

function eqSym(parent: HTMLElement, tex: string): void {
  parent.appendChild(el('span', { style: { fontSize: '0.92rem' } }, mathEl(tex)));
}

const lesson: Lesson = { meta, steps, quiz, mount };
export default lesson;
