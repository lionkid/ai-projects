import type { Lesson, LessonContext, LessonHandle, LessonStep, QuizQuestion } from '@/types';
import {
  el, cssVar, fmt, controlPanel, row, slider, select, readout, matrixView, playback, simTitle, note, split, button,
  Canvas2D, Plot2D, createLoop, substep, presetChips, torqueBars, drawPlanarArm, type Pt, type ReadoutHandle,
} from '@/ui';
import { meta } from './meta';
import {
  GEAR_RATIOS, MOTOR, SingleJointSim, CouplingTwin, arm2Params, backdriveTorque, couplingIndex, effectiveMassMatrix,
  jointFriction, jointInertia, reflectedInertia, swingTrajectory, torqueBreakdown, quinticSamples,
  type Drive, type FrictionMode, type SingleJointParams,
} from './actuation';

// ---------------------------------------------------------------------------
// Narrative
// ---------------------------------------------------------------------------

const steps: LessonStep[] = [
  {
    title: '馬達、減速機、連桿',
    md: `電動馬達的天性是「轉得快、力矩小」：小型 DC 馬達每分鐘轉上數千圈，連續力矩卻常不到 1 N·m。機器人關節需要的剛好相反，所以中間串一個**減速比 $G$** 的減速機（諧波減速機常見 $G\\approx 100$）：

$$\\tau = G\\,\\tau_m,\\qquad \\dot\\theta = \\dot\\theta_m / G$$

理想減速機沒有損失，功率 $\\tau_m\\dot\\theta_m=\\tau\\dot\\theta$ 不變——它只是把速度換成力矩。再加上馬達力矩會隨轉速下降（力矩–轉速線），$G$ 越大，關節力矩越大、最高轉速卻越低。`,
    simHint: '實驗①：馬達以固定的 τ_m 出力。把 G 從 10 換成 50、100：連桿停在幾度？G 夠大時連桿會被甩過頂點一直轉——這時馬達轉速頂到上限了嗎？',
  },
  {
    title: '反射慣量',
    md: `從關節這一側看，轉子轉得比連桿快 $G$ 倍，它的動能 $\\tfrac12J_m(G\\dot\\theta)^2$ 就像一個慣量 $G^2J_m$ 的飛輪跟著關節轉。這就是**反射（表觀）慣量**：

$$I_{\\text{eff}} = I_{\\text{link}} + G^2 J_m$$

轉子雖小，$G^2$ 放大得很兇：$J_m = 3\\times10^{-4}\\ \\text{kg·m}^2$、$G=100$ 時就有 $3\\ \\text{kg·m}^2$，是「長 1 m、末端 1 kg」連桿（$1\\ \\text{kg·m}^2$）的三倍。關節因此變「重」：自由擺動的週期按 $\\sqrt{I_{\\text{eff}}}$ 拉長；而且不論手臂擺成什麼姿勢，這份慣量都一樣，是個常數。`,
    simHint: '實驗②：馬達斷電、連桿從 60° 放開。G 依序換成 1、50、100、200（會自動重新放開），比較右側慣量長條與擺動的快慢；擺動很快停下，則是被放大的摩擦（第 4 步）。',
  },
  {
    title: '耦合變弱了',
    md: `多軸手臂的每顆馬達都把 $G^2J_m$ 加在 $M(\\theta)$ 的**對角線**上：

$$M_{\\text{eff}}(\\theta) \\approx M(\\theta) + \\diag(G_1^2J_1,\\ G_2^2J_2)$$

（第二顆馬達裝在連桿 1 上，還會帶來 $J_2$、$G_2J_2$ 這類較小的常數項，模擬已算入。）$G$ 大時對角項主導，耦合指標 $|M_{12}|/\\sqrt{M_{11}M_{22}}$ 明顯下降：一軸的加速度不太會拖動另一軸，各軸近似**解耦**——這是工業手臂常能每軸各用一個獨立 PID 的原因。`,
    simHint: '實驗③（2R、水平面）：關節 1 照同一條軌跡轉 69°，關節 2 斷電。比較直驅（紅）與有減速機（藍）的關節 2 被拖動多少，以及兩個耦合指標；再試 G = 200。',
  },
  {
    title: '背驅動性與安全',
    md: `**背驅動性（backdrivability）**指從連桿端反推馬達的難易。外力推動高減速比的關節時，得把轉子加速到 $G$ 倍轉速（慣量 $\\times G^2$），還要克服馬達端的摩擦（在關節端看是 $\\times G$）——關節變得又「重」又「黏」。

這也牽涉安全：撞到人時，轉子儲存的動能 $\\tfrac12G^2J_m\\dot\\theta^2$ 會一起撞上去，馬達又來不及被推開。所以協作機器人偏好低減速比、低轉子慣量，或在關節裝力矩感測器偵測碰撞。`,
    simHint: '實驗④：同樣 3 N·m 的外力推 0.5 秒。比較 G = 1、20、100、200 的最大偏轉、「起動所需外力矩」與轉子動能占比。',
  },
  {
    title: '摩擦模型',
    md: `軸承與減速機的摩擦常用兩項描述：

$$\\tau_f = b\\,\\dot\\theta + \\tau_c\\,\\sgn(\\dot\\theta)$$

**黏滯**項與速度成正比，讓自由擺動按比例（指數）衰減；**庫侖**項大小固定、方向隨速度翻轉，讓振幅每擺一次少掉固定量，最後停在偏離平衡處。靜止時還得先超過**靜摩擦** $\\tau_s>\\tau_c$ 才動得起來，於是低速拖動時會「卡住→突然滑一段→又卡住」，稱為**黏滑（stick–slip）**。數值上 $\\sgn(0)$ 沒有定義，要用死區判斷「卡住」，或用 $\\tanh$ 平滑。`,
    simHint: '實驗⑤：彈簧以 0.15 rad/s 慢慢拉動連桿，看 θ̇ 走走停停、τ_f 在卡住時跟著彈簧力矩爬升。再把「摩擦實作」切成 sgn（起步時 τ_f 每步正負亂跳，之後也不再黏滑）與 tanh（平滑，但不會真正卡住）比較。',
  },
  {
    title: '修正後的逆動力學與下一章',
    md: `把這些項都補上，手臂要走出軌跡 $\\theta(t)$，每個關節需要的力矩是

$$\\tau = \\underbrace{M(\\theta)\\ddot\\theta + c + g}_{\\text{連桿}} + \\underbrace{\\Delta M\\,\\ddot\\theta}_{\\text{轉子}} + \\underbrace{\\tau_f(\\dot\\theta)}_{\\text{摩擦}}$$

連桿部分照舊由 Newton–Euler 算出，$\\Delta M$ 是轉子貢獻的常數矩陣（§8.9.3 說明如何把它直接併入遞迴），馬達只需輸出 $\\tau_m=\\tau/G$。$G$ 大時 $\\tau_m$ 很小，但急加減速時轉子項可能比重力還大。

下一章的問題：這條 $\\theta(t)$ 要怎麼規劃，才能又平順、又不超出馬達的力矩與轉速？`,
    simHint: '實驗⑥：2R 手臂（有重力）沿五次多項式軌跡來回擺動。調 G：到 100 以上時，「轉子」長條在加減速時壓過重力；下方小圖是下一章的時間縮放 s(t)。',
  },
];

const quiz: QuizQuestion[] = [
  {
    question: '協作機器人（cobot）為什麼常選低減速比，甚至直驅馬達？',
    options: [
      '低減速比能讓同一顆馬達輸出更大的關節力矩',
      '反射慣量 $G^2J_m$ 與反射摩擦都小：關節容易被反推，撞到人時一起撞上去的有效慣量與動能也小',
      '低減速比會讓 $M(\\theta)$ 變成對角矩陣，控制比較簡單',
      '高減速比的齒輪一定會打滑，不安全',
    ],
    answer: 1,
    explanation: '$G$ 越大，關節端看到的轉子慣量以 $G^2$ 成長、馬達端摩擦以 $G$ 成長：外力幾乎推不動，碰撞時轉子動能 $\\tfrac12G^2J_m\\dot\\theta^2$ 也一起撞上去。低 $G$ 讓關節「柔軟」、可被反推與感測。代價是需要力矩更大的馬達，而且 $M(\\theta)$ 不再由對角項主導、耦合更強（C 剛好說反了），控制更難；A 也相反：同一顆馬達的關節力矩是 $G\\tau_m$。',
  },
  {
    question: '其他條件不變，把減速比從 $G$ 加倍成 $2G$。從關節端看，轉子的反射慣量變幾倍？若關節慣量原本就由轉子主導，要以同樣角加速度從外面推動它，所需力矩約變幾倍？',
    options: ['2 倍；2 倍', '4 倍；約 4 倍', '4 倍；2 倍', '不變；不變'],
    answer: 1,
    explanation: '$G^2J_m \\to (2G)^2J_m = 4G^2J_m$。若 $I_{\\text{eff}}\\approx G^2J_m$，所需力矩 $\\tau = I_{\\text{eff}}\\,\\alpha$ 也約變 4 倍；另外馬達端庫侖摩擦在關節端是 $G\\tau_{c,m}$，這一項變 2 倍——兩者都讓背驅動更困難（實驗④的「起動所需外力矩」可以驗證）。',
  },
  {
    question: '關節有庫侖／靜摩擦，位置控制器帶積分項（PI 或 PID）。常看到關節在目標附近小幅、反覆地來回（極限環，hunting）。最主要的原因是？',
    options: [
      '積分項在卡住時持續累積，直到力矩超過靜摩擦才突然滑動並衝過目標，接著反方向重演',
      '黏滯摩擦讓閉迴路不穩定',
      '減速比太低造成耦合',
      '數值積分的步長太大',
    ],
    answer: 0,
    explanation: '靜止時摩擦能抵銷任何小於 $\\tau_s$ 的力矩，所以誤差不大時關節卡住不動；積分項持續累積，直到超過 $\\tau_s$ 才突然起動。一旦滑動，摩擦掉到 $\\tau_c<\\tau_s$，多出的力矩讓關節衝過頭；誤差反號後又卡住、再累積……形成自持的振盪。黏滯摩擦其實是阻尼，有助穩定。',
  },
  {
    question: '模擬時直接用 $\\tau_c\\,\\sgn(\\dot\\theta)$ 並以固定步長積分。若外力小於摩擦、關節本應靜止不動，會看到什麼？',
    options: [
      '速度精確地停在 0，和真實情況一樣',
      '摩擦力矩每一步正負翻轉，速度在 0 附近抖動（chattering），而且表現不出靜摩擦',
      '關節以固定速度一直滑下去',
      '程式因除以零而當掉',
    ],
    answer: 1,
    explanation: '$\\sgn$ 在 0 兩側跳變：這一步速度略正，摩擦就反向把它推成略負，下一步又反過來，$\\tau_f$ 在 $\\pm\\tau_c$ 間每步翻轉。常見解法是加死區：$|\\dot\\theta|$ 夠小時視為卡住、摩擦等於外力（上限 $\\tau_s$），即 Karnopp 模型；或用 $\\tanh(\\dot\\theta/v_s)$ 平滑，代價是關節會極慢地「蠕動」而不會真正卡住。實驗⑤可切換三種實作驗證。',
  },
];

// ---------------------------------------------------------------------------
// Simulation
// ---------------------------------------------------------------------------

type ExpId = 'motor' | 'free' | 'coupling' | 'push' | 'spring' | 'decomp';
const EXPS: { id: ExpId; label: string; title: string }[] = [
  { id: 'motor', label: '① 馬達力矩', title: '馬達以固定力矩出力，經減速機抬起連桿' },
  { id: 'free', label: '② 自由擺動', title: '馬達斷電，連桿從 60° 放開' },
  { id: 'coupling', label: '③ 耦合', title: '關節 1 照軌跡轉動、關節 2 斷電：看它被拖動多少' },
  { id: 'push', label: '④ 外力推動', title: '從外面以 3 N·m 推連桿 0.5 秒（背驅動測試）' },
  { id: 'spring', label: '⑤ 黏滑', title: '減速機輸出端經扭簧慢慢拉動連桿（黏滑）' },
  { id: 'decomp', label: '⑥ 力矩分解', title: '2R 沿軌跡來回擺動，拆解所需力矩' },
];

const TC_MOTOR = 0.01; // motor-side Coulomb friction (N·m)
const LINK = { m: 1, L: 1 };
const PUSH = { tau: 3, t0: 0.3, dur: 0.5 };
const SPRING = { k: 20, omegaOut: 0.15, amp: 0.8 };
const FRICTION_DEFAULT = { b: 0.05, tc: 0.1, ts: 0.15 };
const FRICTION_STICKSLIP = { b: 0.05, tc: 1.0, ts: 1.6 };
const BACKDRIVE_ALPHA = 2; // rad/s² for the "torque needed to start moving" readout
const COUP = { th0: [-0.9, 0.3], delta: 1.2, T: 1.0, end: 2.5 };
const SWING = { A: [-2.3, 1.1], B: [-0.8, -0.9], Tf: 1.4 };
const PIVOT: Pt = [-0.15, 0.25];
const WORLD_COUPLING = { xmin: -2.3, xmax: 2.3, ymin: -2.15, ymax: 2.15 };
const WORLD_DECOMP = { xmin: -2.3, xmax: 2.3, ymin: -2.2, ymax: 0.75 };
const H_JOINT = 1e-3;
const H_ARM = 1 / 400;
const DEG = 180 / Math.PI;
const rpm = (w: number) => (w * 60) / (2 * Math.PI);

/** Gear with tick teeth rotating by `angle` (world radians). */
function drawGear(c: Canvas2D, center: Pt, r: number, teeth: number, angle: number, color: string, fill: string): void {
  c.circle(center, r, { color, width: 1.5, fill });
  const tooth = Math.min(0.045, r * 0.35);
  for (let k = 0; k < teeth; k++) {
    const a = angle + (2 * Math.PI * k) / teeth;
    c.line([center[0] + r * Math.cos(a), center[1] + r * Math.sin(a)], [center[0] + (r + tooth) * Math.cos(a), center[1] + (r + tooth) * Math.sin(a)], { color, width: 3, cap: 'butt' });
  }
  c.line(center, [center[0] + r * Math.cos(angle), center[1] + r * Math.sin(angle)], { color, width: 2 });
}

/** Zig-zag torsional spring drawn along an arc between two world angles. */
function drawArcSpring(c: Canvas2D, center: Pt, r: number, a0: number, a1: number, color: string): void {
  const n = 14;
  const pts: Pt[] = [];
  for (let k = 0; k <= n; k++) {
    const a = a0 + ((a1 - a0) * k) / n;
    const rr = k === 0 || k === n ? r : r + (k % 2 ? 0.05 : -0.05);
    pts.push([center[0] + rr * Math.cos(a), center[1] + rr * Math.sin(a)]);
  }
  c.polyline(pts, { color, width: 2 });
}

function mount(host: HTMLElement, ctx: LessonContext): LessonHandle {
  simTitle(host, '馬達 → 減速機 → 連桿', '§8.9');

  // ---------- parameters & state
  let exp: ExpId = 'motor';
  let gIndex = GEAR_RATIOS.indexOf(50);
  let Jm = 3e-4;
  let tauCmd = 0.12;
  const fr: { b: number; tc: number; ts: number; mode: FrictionMode } = { ...FRICTION_DEFAULT, mode: 'karnopp' };
  let startAngle = 0;
  let stuckTime = 0;
  let stuckRun = 0; // seconds the joint has stayed stuck without interruption
  let runTime = 0;
  let tArm = 0;
  let peakBare = 0;
  let peakGeared = 0;
  let peakTau = [0, 0];
  const G = (): number => GEAR_RATIOS[gIndex];
  const isArm = (e: ExpId = exp): boolean => e === 'coupling' || e === 'decomp';
  const jointParams = (): SingleJointParams => ({
    G: G(), Jm, m: LINK.m, L: LINK.L, gravity: exp !== 'spring', friction: { ...fr }, tcMotor: TC_MOTOR,
  });
  const driveFor = (e: ExpId): Drive => {
    if (e === 'motor') return { kind: 'motor', tauCmd };
    if (e === 'push') return { kind: 'push', tauExt: PUSH.tau, t0: PUSH.t0, dur: PUSH.dur };
    if (e === 'spring') return { kind: 'spring', k: SPRING.k, omegaOut: SPRING.omegaOut, amp: SPRING.amp };
    return { kind: 'motor', tauCmd: 0 };
  };
  let jsim = new SingleJointSim(jointParams(), driveFor('motor'), 0);
  let bare = new CouplingTwin(arm2Params(false), 1, 0, COUP.th0, COUP.delta, COUP.T);
  let geared = new CouplingTwin(arm2Params(false), G(), Jm, COUP.th0, COUP.delta, COUP.T);

  // ---------- layout
  const expChips = presetChips(host, EXPS.map((e) => ({ label: e.label, title: e.title, apply: () => setExp(e.id) })));
  const cvJ = new Canvas2D(host, { aspect: 2, world: { xmin: -1.65, xmax: 1.95, ymin: -1.2, ymax: 1.22 }, overlay: true });
  const cvR = new Canvas2D(host, { aspect: 2, world: WORLD_COUPLING, overlay: true });

  const shared = controlPanel(host, { compact: true });
  const sG = slider(shared, {
    label: '減速比 G', min: 0, max: GEAR_RATIOS.length - 1, step: 1, value: gIndex,
    format: (v) => `${GEAR_RATIOS[Math.round(v)]} : 1`,
    onInput: (v) => { gIndex = Math.round(v); onParamsChanged(true); },
  });
  const sJ = slider(shared, {
    label: '轉子 J_m', min: 0, max: 10, step: 0.1, value: Jm * 1e4,
    format: (v) => `${fmt(v, 1)}×10⁻⁴ kg·m²`,
    onInput: (v) => { Jm = v * 1e-4; onParamsChanged(true); },
  });
  const sTau = slider(shared, {
    label: 'τ_m 指令', min: 0, max: 0.3, step: 0.005, value: tauCmd, unit: 'N·m', title: '馬達端的力矩指令（電流控制）',
    onInput: (v) => { tauCmd = v; if (exp === 'motor') jsim.drive = driveFor('motor'); },
  });
  const sharedRow = row(shared);
  const pb = playback(sharedRow, {
    onPlay: () => play(),
    onPause: () => pause(),
    onReset: () => { restart(); },
    onSpeed: (s) => { loop.speed = s; },
  });
  const pushBtn = button(sharedRow, { label: '👉 再推一次', small: true, onClick: () => { pushAgain(); } });
  const selMode = select<FrictionMode>(sharedRow, {
    options: [
      { value: 'karnopp', label: '摩擦：死區＋靜摩擦（Karnopp）' },
      { value: 'tanh', label: '摩擦：tanh 平滑' },
      { value: 'sgn', label: '摩擦：直接用 sgn（會抖）' },
    ],
    value: fr.mode,
    onChange: (m) => {
      fr.mode = m;
      // restart the stick–slip run so the start-up phase (link held below τ_c) shows each treatment
      if (exp === 'spring') { restart(); play(); } else onParamsChanged(false);
    },
  });
  selMode.el.title = 'sgn(θ̇) 在零速附近的數值處理方式';

  // single-joint panel
  const panelJ = el('div');
  const panelR = el('div');
  host.append(panelJ, panelR);
  const ctlJ = controlPanel(panelJ, { compact: true });
  const sB = slider(ctlJ, { label: 'b（黏滯）', min: 0, max: 1, step: 0.01, value: fr.b, unit: 'N·m·s', onInput: (v) => { fr.b = v; onParamsChanged(false); } });
  const sTc = slider(ctlJ, { label: 'τ_c（庫侖）', min: 0, max: 2, step: 0.01, value: fr.tc, unit: 'N·m', onInput: (v) => { fr.tc = v; onParamsChanged(false); } });
  const sTs = slider(ctlJ, { label: 'τ_s（靜摩擦）', min: 0, max: 3, step: 0.01, value: fr.ts, unit: 'N·m', onInput: (v) => { fr.ts = v; onParamsChanged(false); } });
  const [plotLHost, plotRHost] = split(panelJ);
  const plotL = new Plot2D(plotLHost, { title: 'θ(t)', yLabel: 'θ (°)', xLabel: 't (s)', series: [{ name: 'θ (°)' }], xWindow: 10, height: 135, yMinSpan: 2, legend: false });
  const plotR = new Plot2D(plotRHost, {
    title: '關節端力矩 (N·m)', yLabel: 'N·m', xLabel: 't (s)', xWindow: 10, height: 135, yMinSpan: 0.2,
    series: [{ name: '驅動' }, { name: '摩擦' }, { name: '重力' }],
  });
  const infoJ = el('div', { style: { display: 'flex', flexWrap: 'wrap', gap: '0.15rem 1rem', marginTop: '0.25rem' } });
  panelJ.appendChild(infoJ);
  const noteJ = note(panelJ, '');

  // 2R panel: coupling
  const panelCoup = el('div');
  const panelDec = el('div');
  panelR.append(panelCoup, panelDec);
  const coupPlot = new Plot2D(panelCoup, {
    title: '關節角變化（關節 2 不出力）', yLabel: 'Δθ (°)', xLabel: 't (s)', height: 150, xRange: [0, COUP.end],
    series: [{ name: 'Δθ₂ 有減速機' }, { name: 'Δθ₂ 直驅' }, { name: 'Δθ₁（兩者相同）', dash: [5, 4] }],
  });
  const matRow = el('div', { style: { display: 'flex', flexWrap: 'wrap', alignItems: 'center', gap: '0.1rem 0.4rem' } });
  panelCoup.appendChild(matRow);
  const mvBare = matrixView(matRow, { label: 'M（直驅）=', digits: 2 });
  const mvGeared = matrixView(matRow, { label: 'M_eff =', digits: 2 });
  const infoC = el('div', { style: { display: 'flex', flexWrap: 'wrap', gap: '0.15rem 1rem' } });
  panelCoup.appendChild(infoC);

  // 2R panel: torque decomposition
  const [barsL, barsR] = split(panelDec);
  const BAR_LABELS = ['Mθ̈ + c', '重力 g', '轉子 ΔMθ̈', '摩擦', '合計 τ'];
  // CSS variable strings (not resolved colours) so the bars follow theme switches
  const barColors = () => ['var(--series-1)', 'var(--series-3)', 'var(--series-5)', 'var(--series-4)', 'var(--fg)'];
  const bars1 = torqueBars(barsL, { labels: BAR_LABELS, max: 20, unit: 'N·m', digits: 1, title: '關節 1 所需力矩', colors: barColors() });
  const bars2 = torqueBars(barsR, { labels: BAR_LABELS, max: 20, unit: 'N·m', digits: 1, title: '關節 2 所需力矩', colors: barColors() });
  const infoD = el('div', { style: { display: 'flex', flexWrap: 'wrap', gap: '0.15rem 1rem' } });
  panelDec.appendChild(infoD);
  const preview = new Plot2D(panelDec, {
    title: '下一章預告：時間縮放 s(t)（五次多項式，每段 1.4 s）', xLabel: 't (s)', height: 115, yRange: [-0.05, 1.05], xRange: [0, SWING.Tf],
    series: [{ name: 's(t)' }], legend: false,
  });
  const sCurve = quinticSamples(SWING.Tf, 80);
  preview.setXY(0, sCurve.map((p) => p[0]), sCurve.map((p) => p[1]));

  // ---------- readout rows rebuilt per experiment
  let info: Record<string, ReadoutHandle> = {};
  /** Values get a fixed minimum width so the row does not re-wrap (and the pane does not jump) every frame. */
  const WIDE: Record<string, number> = { wm: 12, pw: 9.5, eq: 13, st: 10, pg: 10.5, req: 6 };
  const buildInfo = (parent: HTMLElement, items: [string, string, string?][]) => {
    parent.replaceChildren();
    info = {};
    for (const [key, label, unit] of items) {
      info[key] = readout(parent, { label, unit });
      const v = info[key].el.querySelector<HTMLElement>('.mr-value');
      if (v) Object.assign(v.style, { display: 'inline-block', minWidth: `${WIDE[key] ?? 6}em` });
    }
  };
  const setInfo = (key: string, v: string | number, d = 2) => info[key]?.set(v, d);

  // ---------- handle: drag the link tip (single-joint view, while paused)
  const tipOf = (th: number): Pt => [PIVOT[0] + LINK.L * Math.sin(th), PIVOT[1] - LINK.L * Math.cos(th)];
  const tipHandle = cvJ.addHandle({
    get: () => tipOf(jsim.theta),
    set: (p) => {
      startAngle = Math.atan2(p[0] - PIVOT[0], -(p[1] - PIVOT[1]));
      jsim.theta = startAngle;
      jsim.omega = 0;
      cvJ.requestRender();
    },
    onStart: () => pause(),
    onEnd: () => { restart(); play(); },
    radius: 10,
  });

  // ---------- drawing: single joint
  cvJ.draw = (c) => {
    c.grid(0.25);
    const p = jsim.p;
    const th = jsim.theta;
    const muted = cssVar('--fg-muted');
    const fg = cssVar('--fg');
    const accent = cssVar('--accent');
    const v = c.visible();
    // gravity / plane indicator
    if (p.gravity) {
      c.arrow([v.xmin + 0.2, v.ymin + 0.6], [v.xmin + 0.2, v.ymin + 0.2], { color: muted, width: 2 });
      c.text([v.xmin + 0.33, v.ymin + 0.4], 'g', { color: muted, font: 'italic 14px serif' });
    } else {
      c.text([v.xmin + 0.05, v.ymin + 0.15], '俯視：水平面，無重力', { color: muted, align: 'left' });
    }
    // inertia composition bar
    const IL = LINK.m * LINK.L * LINK.L;
    const coupled = jsim.rotorCoupled;
    const IR = coupled ? reflectedInertia(p.G, p.Jm) : 0;
    const Ieff = IL + IR;
    const bx0 = 1.25;
    const bx1 = 1.42;
    const by0 = -0.95;
    const H = 1.65;
    const hL = (H * IL) / Ieff;
    c.polygon([[bx0, by0], [bx1, by0], [bx1, by0 + hL], [bx0, by0 + hL]], { fill: cssVar('--series-3'), color: cssVar('--series-3'), width: 1 });
    if (IR > 0) c.polygon([[bx0, by0 + hL], [bx1, by0 + hL], [bx1, by0 + H], [bx0, by0 + H]], { fill: cssVar('--series-5'), color: cssVar('--series-5'), width: 1 });
    c.text([(bx0 + bx1) / 2, by0 + H + 0.1], `I_eff = ${fmt(Ieff, 2)}`, { color: fg, font: '12px sans-serif' });
    c.text([bx1 + 0.05, by0 + hL / 2], `連桿 ${fmt(IL, 2)}`, { color: cssVar('--series-3'), align: 'left', font: '11px sans-serif' });
    if (IR > 0) c.text([bx1 + 0.05, by0 + hL + (H - hL) / 2], `轉子 ${fmt(IR, 2)}`, { color: cssVar('--series-5'), align: 'left', font: '11px sans-serif' });
    if (!coupled) c.text([(bx0 + bx1) / 2, by0 - 0.12], '轉子在彈簧另一側', { color: muted, font: '11px sans-serif' });
    // gear train: output gear at the pivot, pinion + rotor to the left
    const R = 0.2;
    const rp = Math.max(0.05, R / Math.min(p.G, 4));
    const pin: Pt = [PIVOT[0] - R - rp - 0.03, PIVOT[1] + 0.05];
    const outAngle = (jsim.drive.kind === 'spring' ? jsim.outputAngle() : th) - Math.PI / 2;
    const motorAngle = -p.G * (jsim.drive.kind === 'spring' ? jsim.outputAngle() : th);
    const wm = jsim.motorSpeed();
    c.disk(pin, 0.16, { fill: cssVar('--accent-soft'), alpha: 0.9 });
    if (Math.abs(wm) > 20) {
      const sweep = Math.min(2 * Math.PI - 0.1, Math.abs(wm) * 0.012);
      const a1 = motorAngle;
      const a0 = a1 - Math.sign(-wm || 1) * sweep;
      c.arc(pin, 0.13, Math.min(a0, a1), Math.max(a0, a1), { color: accent, width: 6, alpha: 0.25 });
    }
    drawGear(c, pin, rp, Math.max(6, Math.round(18 * (rp / R))), motorAngle, accent, cssVar('--bg-elev'));
    c.line(pin, [pin[0] + 0.15 * Math.cos(motorAngle), pin[1] + 0.15 * Math.sin(motorAngle)], { color: accent, width: 3 });
    c.text([pin[0] - 0.2, pin[1]], '馬達轉子', { color: accent, align: 'right', font: '12px sans-serif' });
    drawGear(c, PIVOT, R, 18, outAngle, muted, cssVar('--bg-elev'));
    c.text([PIVOT[0], PIVOT[1] + R + 0.16], `減速機 ${p.G} : 1`, { color: fg, font: '12px sans-serif' });
    // spring drive: output arm (dashed) + torsional spring to the link
    if (jsim.drive.kind === 'spring') {
      const ao = jsim.outputAngle() - Math.PI / 2;
      c.line(PIVOT, [PIVOT[0] + 0.55 * Math.cos(ao), PIVOT[1] + 0.55 * Math.sin(ao)], { color: muted, width: 3, dash: [6, 4] });
      drawArcSpring(c, PIVOT, 0.5, ao, th - Math.PI / 2, cssVar('--warn'));
    }
    // link and point mass
    const tip = tipOf(th);
    c.line(PIVOT, tip, { color: muted, width: 8, cap: 'round' });
    c.disk(tip, 0.08, { fill: fg });
    c.point(PIVOT, 4, { color: fg });
    // external push arrow
    if (jsim.pushActive()) {
      const tdir: Pt = [Math.cos(th), Math.sin(th)];
      c.arrow([tip[0] - 0.45 * tdir[0], tip[1] - 0.45 * tdir[1]], [tip[0] - 0.1 * tdir[0], tip[1] - 0.1 * tdir[1]], { color: cssVar('--danger'), width: 4 });
      c.text([tip[0] - 0.55 * tdir[0], tip[1] - 0.55 * tdir[1] + 0.12], `外力 ${PUSH.tau} N·m`, { color: cssVar('--danger') });
    }
    if (jsim.stuck) c.text([tip[0], tip[1] - 0.2], '卡住（靜摩擦）', { color: cssVar('--warn'), bg: cssVar('--sim-bg') });
    c.overlay.textContent = `θ = ${fmt(th * DEG, 1)}°  θ̇ = ${fmt(jsim.omega, 2)} rad/s  ω_m = ${fmt(wm, 0)} rad/s`;
  };

  // ---------- drawing: 2R views
  const drawMotors = (c: Canvas2D, pts: Pt[], color: string) => {
    for (let i = 0; i < 2; i++) {
      const [x, y] = pts[i];
      c.polygon([[x - 0.09, y - 0.09], [x + 0.09, y - 0.09], [x + 0.09, y + 0.09], [x - 0.09, y + 0.09]], { fill: color, color, width: 1, alpha: 0.35 });
    }
  };
  cvR.draw = (c) => {
    c.grid(0.5);
    const muted = cssVar('--fg-muted');
    const v = c.visible();
    if (exp === 'coupling') {
      c.text([v.xmin + 0.1, v.ymin + 0.2], '俯視：水平面，無重力', { color: muted, align: 'left' });
      drawPlanarArm(c, [1, 1], COUP.th0, { alpha: 0.15, linkWidth: 6 });
      const cBare = cssVar('--series-2');
      const cGear = cssVar('--series-1');
      const pb2 = drawPlanarArm(c, [1, 1], bare.theta(), { linkColor: cBare, eeColor: cBare, alpha: 0.8, linkWidth: 6 });
      const pg = drawPlanarArm(c, [1, 1], geared.theta(), { linkColor: cGear, eeColor: cGear, linkWidth: 10 });
      drawMotors(c, pg, cGear);
      c.text(pb2[2], '直驅（無減速機）', { color: cBare, dy: -16, bg: cssVar('--sim-bg') });
      c.text(pg[2], `G = ${G()}`, { color: cGear, dy: 16, bg: cssVar('--sim-bg') });
      c.overlay.textContent = `t = ${fmt(geared.t, 2)} s  關節 1：照五次多項式轉 ${fmt(COUP.delta * DEG, 0)}°（${COUP.T} s）`;
    } else {
      c.arrow([v.xmax - 0.3, v.ymax - 0.15], [v.xmax - 0.3, v.ymax - 0.6], { color: muted, width: 2 });
      c.text([v.xmax - 0.15, v.ymax - 0.38], 'g', { color: muted, font: 'italic 14px serif' });
      drawPlanarArm(c, [1, 1], SWING.A, { alpha: 0.15, linkWidth: 6 });
      drawPlanarArm(c, [1, 1], SWING.B, { alpha: 0.15, linkWidth: 6 });
      const s = swingTrajectory(tArm, SWING.A, SWING.B, SWING.Tf);
      const pts = drawPlanarArm(c, [1, 1], s.theta, { linkWidth: 9 });
      drawMotors(c, pts, cssVar('--accent'));
      c.disk(pts[1], 0.07, { fill: cssVar('--fg') });
      c.disk(pts[2], 0.07, { fill: cssVar('--fg') });
      c.overlay.textContent = `t = ${fmt(tArm, 2)} s  s = ${fmt(s.s, 3)}  θ = (${fmt(s.theta[0] * DEG, 0)}°, ${fmt(s.theta[1] * DEG, 0)}°)`;
    }
  };

  // ---------- simulation control
  const loop = createLoop((dt) => {
    if (!isArm()) {
      substep(dt, H_JOINT, (h) => {
        jsim.step(h);
        runTime += h;
        if (jsim.stuck) stuckTime += h;
        stuckRun = jsim.stuck ? stuckRun + h : 0;
      });
      plotL.push(jsim.t, [exp === 'spring' ? jsim.omega : jsim.theta * DEG]);
      plotR.push(jsim.t, [jsim.tauDrive, jsim.tauF, jsim.p.gravity ? jsim.tauGrav : Number.NaN]);
      pb.setTime(jsim.t);
      cvJ.requestRender();
      // nothing more can happen: the joint is held by static friction and no push is pending
      const pushPending = jsim.drive.kind === 'push' && jsim.t < jsim.drive.t0 + jsim.drive.dur;
      if ((exp !== 'spring' && stuckRun > 2 && !pushPending) || (exp === 'spring' && jsim.t > 45)) pause();
    } else if (exp === 'coupling') {
      substep(dt, H_ARM, (h) => {
        bare.step(h);
        geared.step(h);
        peakBare = Math.max(peakBare, Math.abs(bare.tau1()));
        peakGeared = Math.max(peakGeared, Math.abs(geared.tau1()));
      });
      coupPlot.push(geared.t, [(geared.theta2 - COUP.th0[1]) * DEG, (bare.theta2 - COUP.th0[1]) * DEG, (geared.theta()[0] - COUP.th0[0]) * DEG]);
      pb.setTime(geared.t);
      if (geared.t >= COUP.end) pause();
      cvR.requestRender();
    } else {
      tArm += dt;
      pb.setTime(tArm);
      cvR.requestRender();
    }
    updateInfo();
  });

  function play(): void {
    if (exp === 'coupling' && geared.t >= COUP.end) restart();
    loop.start();
    pb.setPlaying(true, { silent: true });
  }

  function pause(): void {
    loop.stop();
    pb.setPlaying(false, { silent: true });
    cvJ.requestRender();
  }

  function pushAgain(): void {
    if (exp !== 'push') return;
    jsim.drive = { kind: 'push', tauExt: PUSH.tau, t0: jsim.t + 0.05, dur: PUSH.dur };
    if (!loop.running) play();
  }

  function restart(): void {
    loop.stop();
    pb.setPlaying(false, { silent: true });
    if (!isArm()) {
      jsim = new SingleJointSim(jointParams(), driveFor(exp), startAngle);
      stuckTime = 0;
      stuckRun = 0;
      runTime = 0;
      plotL.clear();
      plotR.clear();
      plotL.push(0, [exp === 'spring' ? 0 : startAngle * DEG]);
      pb.setTime(0);
      cvJ.requestRender();
    } else if (exp === 'coupling') {
      const P = arm2Params(false);
      bare = new CouplingTwin(P, 1, 0, COUP.th0, COUP.delta, COUP.T);
      geared = new CouplingTwin(P, G(), Jm, COUP.th0, COUP.delta, COUP.T);
      peakBare = 0;
      peakGeared = 0;
      coupPlot.clear();
      coupPlot.push(0, [0, 0, 0]);
      const Mb = effectiveMassMatrix(P, COUP.th0, 1, 0);
      const Mg = effectiveMassMatrix(P, COUP.th0, G(), Jm);
      mvBare.set(Mb);
      mvGeared.set(Mg);
      pb.setTime(0);
      cvR.requestRender();
    } else {
      tArm = 0;
      peakTau = [0, 0];
      rescaleBars();
      pb.setTime(0);
      cvR.requestRender();
    }
    tipHandle.enabled = !isArm();
    updateInfo();
  }

  /** Bar scale: the largest torque component along one full back-and-forth period. */
  function rescaleBars(): void {
    let mx = 1;
    for (let t = 0; t < 2 * SWING.Tf; t += 0.05) {
      const s = swingTrajectory(t, SWING.A, SWING.B, SWING.Tf);
      const br = torqueBreakdown(s.theta, s.dtheta, s.ddtheta, armDrive());
      for (let i = 0; i < 2; i++) mx = Math.max(mx, Math.abs(br.inertial[i] + br.velocity[i]), Math.abs(br.gravity[i]), Math.abs(br.rotor[i]), Math.abs(br.friction[i]), Math.abs(br.total[i]));
    }
    bars1.setMax(mx * 1.05);
    bars2.setMax(mx * 1.05);
  }

  const armDrive = () => ({ G: G(), Jm, gravity: true, friction: { ...fr }, tcMotor: TC_MOTOR });

  function onParamsChanged(gearChanged: boolean): void {
    if (!isArm()) {
      jsim.p = jointParams();
      if (gearChanged && (exp === 'free' || exp === 'push')) { restart(); play(); return; }
      cvJ.requestRender();
    } else if (exp === 'coupling') {
      if (gearChanged) { restart(); play(); return; }
    } else {
      rescaleBars();
      peakTau = [0, 0];
    }
    updateInfo();
  }

  function setFriction(f: { b: number; tc: number; ts: number }): void {
    fr.b = f.b; fr.tc = f.tc; fr.ts = f.ts;
    sB.set(f.b, { silent: true });
    sTc.set(f.tc, { silent: true });
    sTs.set(f.ts, { silent: true });
  }

  function setExp(e: ExpId): void {
    exp = e;
    const arm = isArm(e);
    cvJ.el.style.display = arm ? 'none' : '';
    cvR.el.style.display = arm ? '' : 'none';
    if (arm) cvR.setWorld(e === 'coupling' ? WORLD_COUPLING : WORLD_DECOMP);
    panelJ.style.display = arm ? 'none' : '';
    panelR.style.display = arm ? '' : 'none';
    panelCoup.style.display = e === 'coupling' ? '' : 'none';
    panelDec.style.display = e === 'decomp' ? '' : 'none';
    sTau.el.style.display = e === 'motor' ? '' : 'none';
    pushBtn.style.display = e === 'push' ? '' : 'none';
    setFriction(e === 'spring' ? FRICTION_STICKSLIP : FRICTION_DEFAULT);
    startAngle = e === 'free' ? Math.PI / 3 : 0;
    plotL.setOptions(e === 'spring'
      ? { title: 'θ̇(t)：卡住時為 0', yLabel: 'θ̇ (rad/s)', series: [{ name: 'θ̇ (rad/s)' }], yMinSpan: 0.05 }
      : { title: 'θ(t)', yLabel: 'θ (°)', series: [{ name: 'θ (°)' }], yMinSpan: 2 });
    plotR.setOptions({ series: [{ name: e === 'spring' ? '彈簧' : e === 'push' ? '外力' : '驅動 Gτ_m' }, { name: '摩擦' }, { name: '重力' }] });
    noteJ.textContent = {
      motor: `馬達：失速力矩 ${MOTOR.stallTorque} N·m，轉速達 ${MOTOR.noLoadSpeed} rad/s（${fmt(rpm(MOTOR.noLoadSpeed), 0)} rpm）時力矩降為 0。可拖曳連桿末端改變起始角。`,
      free: `拖曳連桿末端可改變放開角度。關節端庫侖摩擦 = τ_c + G × ${TC_MOTOR} N·m（馬達端摩擦被放大）。`,
      push: '馬達斷電；外力在 t = 0.3 s 起作用 0.5 秒，「再推一次」可重複。',
      spring: `輸出端以 ${SPRING.omegaOut} rad/s 在 ±${fmt(SPRING.amp * DEG, 0)}° 間來回轉，經扭簧（k = ${SPRING.k} N·m/rad）拉動連桿；轉子在彈簧另一側。`,
      coupling: '', decomp: '',
    }[e];
    if (e === 'motor') buildInfo(infoJ, [['tm', 'τ_m', 'N·m'], ['wm', 'ω_m'], ['tj', 'τ = Gτ_m', 'N·m'], ['wj', 'θ̇', 'rad/s'], ['pw', '功率 τ_mω_m = τθ̇', 'W'], ['eq', '平衡角 θ*']]);
    else if (e === 'free') buildInfo(infoJ, [['il', 'I_link', 'kg·m²'], ['ir', 'G²J_m', 'kg·m²'], ['ie', 'I_eff', 'kg·m²'], ['per', '小角度週期', 's'], ['tc', '關節端 τ_c 總', 'N·m']]);
    else if (e === 'push') buildInfo(infoJ, [['max', '最大偏轉'], ['req', `起動所需外力矩（α = ${BACKDRIVE_ALPHA} rad/s²）`, 'N·m'], ['ke', '轉子動能占比'], ['ie', 'I_eff', 'kg·m²']]);
    else if (e === 'spring') buildInfo(infoJ, [['ks', '彈簧力矩', 'N·m'], ['tf', '摩擦 τ_f', 'N·m'], ['st', '狀態'], ['frac', '卡住時間比例']]);
    else if (e === 'coupling') buildInfo(infoC, [['kb', '耦合指標 直驅'], ['kg', '耦合指標 有減速機'], ['db', 'max|Δθ₂| 直驅'], ['dg', 'max|Δθ₂| 有減速機'], ['pb', '關節 1 峰值力矩 直驅', 'N·m'], ['pg', '有減速機（馬達端 τ/G）']]);
    else buildInfo(infoD, [['m1', '馬達力矩 τ₁/G', 'N·m'], ['m2', 'τ₂/G', 'N·m'], ['p1', '峰值 |τ₁|', 'N·m'], ['p2', '峰值 |τ₂|', 'N·m'], ['ir', 'G²J_m', 'kg·m²']]);
    restart();
    play();
  }

  function updateInfo(): void {
    if (exp === 'motor') {
      const wm = jsim.motorSpeed();
      setInfo('tm', jsim.tauMotor, 3);
      setInfo('wm', `${fmt(wm, 0)} rad/s（${fmt(rpm(wm), 0)} rpm）`);
      setInfo('tj', jsim.tauDrive, 2);
      setInfo('wj', jsim.omega, 3);
      setInfo('pw', `${fmt(jsim.tauMotor * wm, 2)} = ${fmt(jsim.tauDrive * jsim.omega, 2)}`);
      const ratio = (G() * tauCmd) / (LINK.m * 9.81 * LINK.L);
      setInfo('eq', ratio < 1 ? `${fmt(Math.asin(ratio) * DEG, 1)}°（Gτ_m = mgL sin θ*）` : 'Gτ_m > mgL：會越過頂點');
    } else if (exp === 'free') {
      const p = jsim.p;
      const Ie = jointInertia(p);
      setInfo('il', LINK.m * LINK.L * LINK.L, 2);
      setInfo('ir', reflectedInertia(p.G, p.Jm), 3);
      setInfo('ie', Ie, 3);
      setInfo('per', 2 * Math.PI * Math.sqrt(Ie / (LINK.m * 9.81 * LINK.L)), 2);
      setInfo('tc', jointFriction(p).tc, 2);
    } else if (exp === 'push') {
      const p = jsim.p;
      const ke = jsim.kineticSplit();
      const tot = ke.link + ke.rotor;
      setInfo('max', `${fmt(jsim.maxAbsTheta * DEG, 1)}°`);
      setInfo('req', backdriveTorque(p, BACKDRIVE_ALPHA), 2);
      setInfo('ke', tot > 1e-9 ? `${fmt((100 * ke.rotor) / tot, 0)}%` : `${fmt((100 * reflectedInertia(p.G, p.Jm)) / jointInertia(p), 0)}%`);
      setInfo('ie', jointInertia(p), 2);
    } else if (exp === 'spring') {
      setInfo('ks', jsim.tauDrive, 2);
      setInfo('tf', jsim.tauF, 2);
      setInfo('st', jsim.p.friction.mode !== 'karnopp' ? '（此實作沒有卡住狀態）' : jsim.stuck ? '卡住' : '滑動');
      setInfo('frac', runTime > 0 ? `${fmt((100 * stuckTime) / runTime, 0)}%` : '–');
    } else if (exp === 'coupling') {
      const P = arm2Params(false);
      setInfo('kb', couplingIndex(effectiveMassMatrix(P, COUP.th0, 1, 0)), 3);
      setInfo('kg', couplingIndex(effectiveMassMatrix(P, COUP.th0, G(), Jm)), 3);
      setInfo('db', `${fmt(bare.maxDev * DEG, 1)}°`);
      setInfo('dg', `${fmt(geared.maxDev * DEG, 1)}°`);
      setInfo('pb', peakBare, 1);
      setInfo('pg', `${fmt(peakGeared, 1)} N·m（${fmt(peakGeared / G(), 3)} N·m）`);
    } else {
      const s = swingTrajectory(tArm, SWING.A, SWING.B, SWING.Tf);
      const br = torqueBreakdown(s.theta, s.dtheta, s.ddtheta, armDrive());
      peakTau = [Math.max(peakTau[0], Math.abs(br.total[0])), Math.max(peakTau[1], Math.abs(br.total[1]))];
      const vals = (i: number) => [br.inertial[i] + br.velocity[i], br.gravity[i], br.rotor[i], br.friction[i], br.total[i]];
      bars1.set(vals(0));
      bars2.set(vals(1));
      setInfo('m1', br.motor[0], 3);
      setInfo('m2', br.motor[1], 3);
      setInfo('p1', peakTau[0], 1);
      setInfo('p2', peakTau[1], 1);
      setInfo('ir', reflectedInertia(G(), Jm), 2);
      preview.setMarkers([{ x: s.tau, y: s.s, label: s.direction > 0 ? 'A → B' : 'B → A' }]);
    }
  }

  // ---------- steps
  const STEP_EXP: ExpId[] = ['motor', 'free', 'coupling', 'push', 'spring', 'decomp'];
  const STEP_G: (number | null)[] = [50, 50, 100, 100, 50, 100];
  let focused: HTMLElement[] = [];
  const setFocus = (els: (HTMLElement | null)[]) => {
    focused.forEach((f) => f.classList.remove('mr-focus'));
    focused = els.filter((x): x is HTMLElement => !!x);
    focused.forEach((f) => f.classList.add('mr-focus'));
  };

  function onStep(i: number): void {
    const g = STEP_G[i];
    if (g != null) { gIndex = GEAR_RATIOS.indexOf(g); sG.set(gIndex, { silent: true }); }
    Jm = 3e-4;
    sJ.set(3, { silent: true });
    selMode.set('karnopp', { silent: true });
    fr.mode = 'karnopp';
    const e = STEP_EXP[i] ?? 'motor';
    expChips.activate(EXPS.findIndex((x) => x.id === e));
    const focusByStep: (HTMLElement | null)[][] = [
      [sG.el, sTau.el],
      [sG.el, cvJ.el],
      [coupPlot.el, matRow],
      [pushBtn, infoJ],
      [plotLHost, selMode.el],
      [barsL.parentElement, preview.el],
    ];
    setFocus(focusByStep[i] ?? []);
  }

  onStep(ctx.currentStep);

  return {
    onStep,
    destroy() {
      loop.stop();
      cvJ.destroy();
      cvR.destroy();
      plotL.destroy();
      plotR.destroy();
      coupPlot.destroy();
      preview.destroy();
    },
  };
}

const lesson: Lesson = { meta, steps, quiz, mount };
export default lesson;
