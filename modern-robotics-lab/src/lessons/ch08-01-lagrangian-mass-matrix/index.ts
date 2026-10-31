import type { Lesson, LessonContext, LessonHandle, LessonStep, QuizQuestion } from '@/types';
import {
  el, fmt, controlPanel, row, slider, angleSlider, toggle, select, readout, matrixView, playback, simTitle, note,
  split, legend, Canvas2D, Plot2D, createLoop, presetChips, torqueBars, drawPlanarArm, mathEl, mdElement,
  type Pt, type BarsHandle,
} from '@/ui';
import type { Vec } from '@/mr';
import { meta } from './meta';
import {
  type Arm, type HybridResult, type JointProgram, type Method, DEFAULT_ARM, GRAVITY, chainFor, compareWithNewtonEuler,
  constantRate, energies, energyEllipse, eulerLagrangeTerms, evaluate, foldCycle, freeJoint, holdAt, initialState,
  massPositions, maxEllipseSemiAxis, sineMotion, splitC, stepState, dynamicsTerms,
} from './lagrange';

// ---------------------------------------------------------------------------
// Narrative
// ---------------------------------------------------------------------------

const steps: LessonStep[] = [
  {
    title: '讓手臂自己擺',
    md: `右邊是一支馬達不出力（$\\tau = 0$）的 2R 手臂：兩根連桿本身不計質量，末端各掛一個質點 $m_1$、$m_2$，在重力下自由擺動——這正是 §8.1 推導用的模型。

這種「雙擺」看起來毫無規律，其實完全由一條二階微分方程決定：只要知道此刻的 $\\theta$ 與 $\\dot\\theta$，下一瞬間的 $\\ddot\\theta$ 就唯一確定。本課要從能量出發把這條方程寫出來：

$$\\tau = M(\\theta)\\,\\ddot\\theta + c(\\theta,\\dot\\theta) + g(\\theta)$$

並弄清楚每一項的物理來源。`,
    simHint: '手臂正在自由擺動。拖曳肘部或末端的質點換個起始姿勢（拖曳時暫停、放開後繼續）；下方曲線中動能 K 與位能 U 此消彼長，總能 E 卻維持水平。',
  },
  {
    title: '動能、位能與 Lagrangian',
    md: `Lagrange 的方法不必畫受力圖，只需要兩個純量函數：

- **動能** $K = \\sum_i \\tfrac12 m_i \\lVert\\dot p_i\\rVert^2$。質點速度 $\\dot p_i$ 對 $\\dot\\theta$ 是線性的，所以 $K$ 是 $\\dot\\theta$ 的二次式，係數則隨構型改變。
- **位能** $U = \\sum_i m_i\\,\\mathfrak g\\,y_i$，只跟構型 $\\theta$ 有關。

令 $\\mathcal L = K - U$，每個關節所需的力矩由 Euler–Lagrange 方程給出：

$$\\tau_i = \\frac{d}{dt}\\frac{\\partial\\mathcal L}{\\partial\\dot\\theta_i} - \\frac{\\partial\\mathcal L}{\\partial\\theta_i}$$

第一項是廣義動量 $p_i = \\partial\\mathcal L/\\partial\\dot\\theta_i$ 的變化率，第二項是 $\\mathcal L$ 隨構型的變化。自由擺動時 $\\tau = 0$，兩項必須時時相等。`,
    simHint: '看下方表格：$\\tfrac{d}{dt}p_i$ 與 $\\tfrac{\\partial\\mathcal L}{\\partial\\theta_i}$ 一直在變，卻始終相等，相減得到的 $\\tau_i$ 停在 0。按暫停可以逐格比對。',
  },
  {
    title: '質量矩陣是個橢圓',
    md: `把動能整理成 $K = \\tfrac12\\dot\\theta^{\\T}M(\\theta)\\dot\\theta$，本例的質量矩陣（記 $c_2=\\cos\\theta_2$）是

$$\\begin{aligned} M_{11} &= m_1L_1^2 + m_2\\big(L_1^2 + 2L_1L_2c_2 + L_2^2\\big)\\\\ M_{12} &= M_{21} = m_2\\big(L_1L_2c_2 + L_2^2\\big),\\quad M_{22} = m_2L_2^2 \\end{aligned}$$

$M$ 對稱，而且任何 $\\dot\\theta \\neq 0$ 都帶來正的動能，所以 $M$ 正定。在 $(\\dot\\theta_1,\\dot\\theta_2)$ 平面上，動能恰為 1 J 的速度圍成一個橢圓：主軸是 $M$ 的特徵向量，半軸長 $\\sqrt{2/\\lambda_i}$。**短軸方向「重」、長軸方向「輕」**。$M$ 只依賴 $\\theta_2$：伸直時兩特徵值相差最大、橢圓最扁長；預設參數下完全折疊時 $M = I$，橢圓正好是圓。`,
    simHint: '拉動「θ₂」滑桿或拖曳末端質點，看橢圓在虛線「伸直」與「折疊」之間變形；再拖曳肘部改變 θ₁——橢圓完全不變。右圖的空心點可設定初速。',
  },
  {
    title: '科氏與向心項',
    md: `因為 $M$ 隨 $\\theta_2$ 改變，Euler–Lagrange 方程中的 $\\tfrac{d}{dt}(M\\dot\\theta)$ 與 $\\partial K/\\partial\\theta$ 會產生**速度相乘**的項。令 $h = m_2L_1L_2\\sin\\theta_2$：

$$c(\\theta,\\dot\\theta) = \\begin{bmatrix} -h\\,(2\\dot\\theta_1\\dot\\theta_2 + \\dot\\theta_2^2) \\\\ h\\,\\dot\\theta_1^2 \\end{bmatrix}$$

- 含 $\\dot\\theta_i^2$ 的是**向心項**（centripetal）：關節 1 等速旋轉時，$m_2$ 需要指向轉軸的加速度；要讓手肘角度不變，關節 2 的馬達就得持續出力 $h\\dot\\theta_1^2$。
- 含 $\\dot\\theta_1\\dot\\theta_2$ 的是**科氏項**（Coriolis）：手肘一開一合改變了繞基座的慣量，關節 1 的轉速會被推著改變。

如果 $M$ 是常數，$c$ 就整個消失。`,
    simHint: '預設「手肘鎖住」、無重力：手臂剛性旋轉、手肘不動，τ₂ 卻不為零。改選「手肘放開」：手肘被甩開，關節 1 的馬達得用 τ₁ 抵銷科氏項才能維持等速；再選「溜冰收手」（τ₁ = 0），看科氏項如何讓關節 1 自己加速。',
  },
  {
    title: '重力項',
    md: `重力項是位能的梯度 $g(\\theta) = \\partial U/\\partial\\theta$：

$$g(\\theta) = \\begin{bmatrix} (m_1+m_2)\\,\\mathfrak gL_1\\cos\\theta_1 + m_2\\,\\mathfrak gL_2\\cos(\\theta_1+\\theta_2) \\\\ m_2\\,\\mathfrak gL_2\\cos(\\theta_1+\\theta_2) \\end{bmatrix}$$

它只看構型、不看速度。要讓手臂靜止撐住，馬達只需輸出 $\\tau = g(\\theta)$：水平伸直時最吃力，垂直朝上時為零。把手臂平放在水平桌面上，$U$ 不隨 $\\theta$ 改變、$g \\equiv 0$，但 $M$ 與 $c$ 完全不受影響。動作越慢，$M\\ddot\\theta$ 與 $c$ 越小，力矩幾乎全部用來對抗重力。`,
    simHint: '拖曳手臂到不同姿勢，看撐住所需的 τ = g(θ)；切到「水平面」g 消失。再比較「慢速揮動」與「快速揮動」：τ 曲線何時貼著 g 曲線？',
  },
  {
    title: '用能量檢查模擬',
    md: `沒有摩擦時，可以由運動方程推出

$$\\frac{dE}{dt} = \\dot\\theta^{\\T}\\tau ,$$

馬達不出力，總能量 $E = K + U$ 就必須守恆。這是檢查模擬最便宜的方法：

- **數值誤差**：顯式 Euler 每步沿切線外推，擺動的能量會一路上升；同樣的步長下，固定步長 RK4 的誤差小好幾個數量級。
- **模型錯誤**：拿掉 $c$，方程本身就不再守恆能量，因為 $\\dot\\theta^{\\T}c = \\tfrac12\\dot\\theta^{\\T}\\dot M\\dot\\theta$ 正好抵銷 $M$ 變化帶來的動能改變。

$\\Delta E(t)$ 明顯漂移時，先懷疑積分器或模型，而不是物理。`,
    simHint: '預設 RK4：ΔE 曲線貼著 0。改選「顯式 Euler」或「漏掉 c 項」，ΔE 開始漂移；淡色影子是正確的參考解，看兩者多快分道揚鑣。',
  },
  {
    title: '兩種推導，同一個答案',
    md: `Lagrange 法對 2R 手臂很漂亮，但每多一個關節，$K$ 的展開式就長一大截。下一課的 **Newton–Euler 遞迴**換一條路：沿著連桿逐一傳遞速度、加速度與力，完全不必展開 $M(\\theta)$。

兩種方法描述同一個物理系統，答案必須一致。右邊把閉式的 $M, c, g$ 與函式庫的 \`MassMatrix\`、\`VelQuadraticForces\`、\`GravityForces\`（內部都是 Newton–Euler）在同一個狀態下並排，再用測試加速度 $\\ddot\\theta = (1, -2)$ rad/s² 比較完整的 $\\tau$——差距只剩 $10^{-15}$ 等級的捨入誤差。

完成自我檢測後，下一課就來拆解這個遞迴。`,
    simHint: '手臂繼續擺動：兩欄數字逐位相同，最大差值停在 10⁻¹⁵ 左右。按「換一組參數」或拖曳手臂再比一次。',
  },
];

const quiz: QuizQuestion[] = [
  {
    question: '預設手臂伸直（$\\theta_2 = 0$）時，$M$ 的特徵值約為 5.83 與 0.17，1 J 的等動能橢圓又細又長。在**同樣的動能**下，哪一種關節速度組合轉得最快（$\\lVert\\dot\\theta\\rVert$ 最大）？',
    options: [
      '兩關節同向轉，$\\dot\\theta_2 \\approx 0.4\\,\\dot\\theta_1$（整支手臂像一根棍子甩動）',
      '兩關節反向轉，$\\dot\\theta_2 \\approx -2.4\\,\\dot\\theta_1$（手肘大幅擺動，末端只緩緩反向移動）',
      '只轉關節 1，$\\dot\\theta_2 = 0$',
      '三者一樣快：伸直時 $M$ 是單位矩陣',
    ],
    answer: 1,
    explanation: '同樣動能下速度最大的方向是橢圓的**長軸**，也就是最小特徵值 $\\lambda_2 \\approx 0.17$ 的特徵向量 $\\propto (1, -2.41)$。沿這個方向運動時，末端質點 $m_2$ 的速度只有手肘 $m_1$ 的約 0.4 倍（而且方向相反），動能大多落在較靠近基座的 $m_1$ 上（$m_2$ 只占約 15%），所以很「輕」。兩關節同向甩動對應最大特徵值，是最「重」的方向；只轉關節 1 介於兩者之間。',
  },
  {
    question: '把同一支 2R 手臂平放在光滑的水平桌面上運動（重力垂直於運動平面）。運動方程 $\\tau = M\\ddot\\theta + c + g$ 會怎麼變？',
    options: [
      '$g(\\theta) \\equiv 0$，$M$ 與 $c$ 不變',
      '$g(\\theta) \\equiv 0$，而且 $c = 0$：沒有重力就沒有離心效應',
      '$M$ 變成常數矩陣',
      '三項都不變，只是 $U$ 的零點改變',
    ],
    answer: 0,
    explanation: '$g(\\theta) = \\partial U/\\partial\\theta$，水平面上 $U$ 不隨構型改變，所以 $g \\equiv 0$。$M$ 與 $c$ 只來自動能 $K$，跟重力無關：$c$ 源自 $M$ 隨 $\\theta_2$ 變化，在桌面上一樣存在——本課「手肘鎖住」的情境就是在無重力下做的，τ₂ 照樣不為零。',
  },
  {
    question: '花式溜冰選手旋轉時把手臂收回胸前，轉速明顯變快。在模擬的「溜冰收手」情境（$\\tau_1 = 0$、無重力、關節 2 由馬達收折）裡，讓關節 1 加速的是方程中的哪一項？',
    options: [
      '重力項 $g(\\theta)$',
      '科氏項：$c_1$ 中與 $\\dot\\theta_1\\dot\\theta_2$ 成正比的部分',
      '關節 1 的馬達力矩 $\\tau_1$',
      '$M_{11}$ 本身，與任何速度都無關',
    ],
    answer: 1,
    explanation: '$\\tau_1 = 0$ 又沒有重力，關節 1 的方程只剩 $M_{11}\\ddot\\theta_1 + M_{12}\\ddot\\theta_2 + c_1 = 0$。收折時 $|\\theta_2|$ 變大，$\\theta_2$ 與 $\\dot\\theta_2$ 同號，所以 $h\\dot\\theta_2 = m_2L_1L_2\\sin\\theta_2\\,\\dot\\theta_2 > 0$，在 $\\dot\\theta_1 > 0$ 的旋轉下科氏項 $-2h\\dot\\theta_1\\dot\\theta_2 < 0$，移到等號右邊就成了推動 $\\ddot\\theta_1 > 0$ 的力矩。換個角度看：$\\theta_1$ 不出現在 $\\mathcal L$ 裡，所以動量 $p_1 = \\partial\\mathcal L/\\partial\\dot\\theta_1$ 守恆；$M_{11}$ 變小，$\\dot\\theta_1$ 只好變大。',
  },
  {
    question: '在無摩擦、$\\tau = 0$ 的自由擺動中用 RK4、步長 1/240 s，但把 $c(\\theta,\\dot\\theta)$ 整項拿掉，總能量 $E$ 開始大幅變動。原因是？',
    options: [
      'RK4 的精度不夠，換更小的步長就會好',
      '少了 $c$，方程不再滿足 $dE/dt = \\dot\\theta^{\\T}\\tau$：原本 $\\dot\\theta^{\\T}c = \\tfrac12\\dot\\theta^{\\T}\\dot M\\dot\\theta$ 恰好抵銷 $M$ 改變造成的動能變化',
      '拿掉 $c$ 等於關掉重力，位能不再計入',
      '$M(\\theta)$ 失去正定性，無法求解 $\\ddot\\theta$',
    ],
    answer: 1,
    explanation: '由 $K = \\tfrac12\\dot\\theta^{\\T}M\\dot\\theta$ 得 $\\dot K = \\dot\\theta^{\\T}M\\ddot\\theta + \\tfrac12\\dot\\theta^{\\T}\\dot M\\dot\\theta$。代入正確的方程，$\\tfrac12\\dot\\theta^{\\T}\\dot M\\dot\\theta$ 恰好被 $\\dot\\theta^{\\T}c$ 抵銷，剩下 $\\dot E = \\dot\\theta^{\\T}\\tau$。拿掉 $c$ 之後這一項沒人抵銷，就算積分完全精確，能量也會變——這是模型錯誤，縮小步長救不回來。',
  },
];

// ---------------------------------------------------------------------------
// Simulation constants and presets
// ---------------------------------------------------------------------------

const DEG = Math.PI / 180;
/** Integration step of the reference model (s). */
const H_REF = 1 / 240;
const DT_CHOICES = [1 / 240, 1 / 120, 1 / 60, 1 / 30];
/** Kinetic-energy level of the drawn mass-matrix ellipse (J). */
const K_REF = 1;
/** Test acceleration for the τ comparison (rad/s²). */
const TEST_ACC: Vec = [1, -2];
/** Joint-rate limit that flags a diverged (Euler) run (rad/s). */
const VEL_LIMIT = 80;

type ModeKey = 'free' | 'spinLocked' | 'spinFree' | 'skater' | 'hold' | 'servo' | 'sweep';

const MODE_LABEL: Record<ModeKey, string> = {
  free: '自由擺動 τ = 0',
  spinLocked: '關節 1 等速、手肘鎖住',
  spinFree: '關節 1 等速、手肘自由',
  skater: 'τ₁ = 0、關節 2 收放',
  hold: '馬達撐住姿勢',
  servo: '兩關節往復運動',
  sweep: 'θ₂ 自動掃描',
};

interface PresetDef {
  label: string;
  title?: string;
  mode: ModeKey;
  /** Start configuration in degrees. */
  th: [number, number];
  /** Initial joint rates (rad/s) for free joints. */
  dth?: [number, number];
  gravity: boolean;
  play: boolean;
  period?: number;
  method?: Method;
  dtIdx?: number;
  dropC?: boolean;
  arm?: Partial<Arm>;
}

const SWING: PresetDef[] = [
  { label: '水平釋放', title: 'θ = (0°, 30°)，從靜止放開', mode: 'free', th: [0, 30], gravity: true, play: true },
  { label: '小擺幅', title: 'θ = (−70°, 15°)：接近線性的小擺動', mode: 'free', th: [-70, 15], gravity: true, play: true },
  { label: '接近倒立', title: 'θ = (85°, 8°)：從幾乎直立倒下', mode: 'free', th: [85, 8], gravity: true, play: true },
];

const STEP_PRESETS: PresetDef[][] = [
  SWING,
  SWING,
  [
    { label: '伸直 θ₂ = 0°', mode: 'free', th: [-30, 0], gravity: true, play: false },
    { label: 'θ₂ = 90°', mode: 'free', th: [-30, 90], gravity: true, play: false },
    { label: '折疊 θ₂ = 160°', mode: 'free', th: [-30, 160], gravity: true, play: false },
    { label: '自動掃描 θ₂', title: 'θ₂ 在 5° 與 175° 之間來回（週期 8 s）', mode: 'sweep', th: [-30, 5], gravity: true, play: true },
  ],
  [
    { label: '手肘鎖住', title: '關節 1 等速旋轉、手肘鎖住、無重力', mode: 'spinLocked', th: [0, 90], gravity: false, play: true },
    { label: '手肘放開', title: '關節 1 等速旋轉、關節 2 自由（τ₂ = 0）、無重力', mode: 'spinFree', th: [0, 90], gravity: false, play: true },
    { label: '溜冰收手', title: 'τ₁ = 0、無重力；關節 2 由馬達在 −10° 與 −160° 之間收放', mode: 'skater', th: [0, -10], dth: [2, 0], gravity: false, play: true },
  ],
  [
    { label: '撐住姿勢', title: '靜止撐住：τ = g(θ)', mode: 'hold', th: [0, 0], gravity: true, play: false },
    { label: '水平面', title: '平放在桌面上：g(θ) ≡ 0', mode: 'hold', th: [0, 0], gravity: false, play: false },
    { label: '慢速揮動', title: '週期 6 s 的往復運動', mode: 'servo', th: [-20, 50], gravity: true, play: true, period: 6 },
    { label: '快速揮動', title: '週期 1 s 的往復運動', mode: 'servo', th: [-20, 50], gravity: true, play: true, period: 1 },
  ],
  [
    { label: 'RK4（正確）', mode: 'free', th: [0, 30], gravity: true, play: true, method: 'rk4', dtIdx: 0, dropC: false },
    { label: '顯式 Euler', title: '同樣步長 1/240 s，改用顯式 Euler', mode: 'free', th: [0, 30], gravity: true, play: true, method: 'euler', dtIdx: 0, dropC: false },
    { label: '漏掉 c 項', title: 'RK4，但運動方程少了 c(θ, dθ/dt)', mode: 'free', th: [0, 30], gravity: true, play: true, method: 'rk4', dtIdx: 0, dropC: true },
  ],
  [
    { label: '自由擺動', mode: 'free', th: [0, 30], gravity: true, play: true, arm: { ...DEFAULT_ARM } },
    { label: '換一組參數', title: 'm = (2, 0.5) kg、L = (0.8, 1.1) m', mode: 'free', th: [20, 60], gravity: true, play: true, arm: { m1: 2, m2: 0.5, L1: 0.8, L2: 1.1 } },
  ],
];

// ---------------------------------------------------------------------------
// Small DOM / format helpers
// ---------------------------------------------------------------------------

function texLabel(tex: string, prefix = '', suffix = ''): HTMLSpanElement {
  return el('span', {}, prefix, el('span', { style: { fontSize: '0.95em', lineHeight: '1' } }, mathEl(tex)), suffix);
}

/** Replace the text label of a toolkit control (slider, readout, matrix, bar) with a node. */
function relabel(root: Element | null, node: Node): void {
  root?.replaceChildren(node);
}

const SUP = '⁰¹²³⁴⁵⁶⁷⁸⁹';
/** Scientific notation with superscript exponent, e.g. 3.6×10⁻¹⁵. */
function sci(x: number, digits = 1): string {
  if (!Number.isFinite(x)) return String(x);
  if (x === 0) return '0';
  if (Math.abs(x) >= 0.01 && Math.abs(x) < 1e4) return fmt(x, Math.abs(x) >= 10 ? 1 : 3);
  let e = Math.floor(Math.log10(Math.abs(x)));
  let m = x / 10 ** e;
  if (Math.abs(Number(m.toFixed(digits))) >= 10) { e += 1; m /= 10; }
  const sup = String(e).replace('-', '⁻').replace(/\d/g, (d) => SUP[Number(d)]);
  return `${m.toFixed(digits)}×10${sup}`;
}

/** Smallest of 1, 2, 5 × 10^k that is ≥ v. */
function niceCeil(v: number): number {
  const p = 10 ** Math.floor(Math.log10(Math.max(v, 1e-9)));
  for (const k of [1, 2, 5, 10]) if (k * p >= v - 1e-12) return k * p;
  return 10 * p;
}

const wrapPi = (q: number) => q - 2 * Math.PI * Math.floor((q + Math.PI) / (2 * Math.PI));
const massRadius = (m: number) => 0.07 + 0.05 * Math.cbrt(m);

// ---------------------------------------------------------------------------
// Simulation
// ---------------------------------------------------------------------------

function mount(host: HTMLElement, ctx: LessonContext): LessonHandle {
  simTitle(host, '2R 手臂的 Lagrangian 動力學', '點質量 2R');

  // ---------- state
  let arm: Arm = { ...DEFAULT_ARM };
  let gravityOn = true;
  let chain = chainFor(arm);
  let mode: ModeKey = 'free';
  let omega1 = 3;
  let period = 6;
  let th0: Vec = [0, 30 * DEG];
  let dth0: Vec = [0, 0];
  let method: Method = 'rk4';
  let dtIdx = 0;
  let dropC = false;
  let programs: [JointProgram, JointProgram] = [freeJoint(), freeJoint()];
  let x: Vec = [0, 30 * DEG, 0, 0];
  let t = 0;
  let acc = 0;
  let ref: { x: Vec; t: number } | null = null;
  let E0 = 0;
  let Eref0 = 0;
  let blown = false;
  let curStep = -1;
  let last: HybridResult = evaluate(arm, x, 0, programs);
  const trail: Pt[] = [];
  let ellW = 4;
  let ghosts: { straight: ReturnType<typeof energyEllipse>; folded: ReturnType<typeof energyEllipse> } | null = null;
  let barPeak = 1;
  /** Current full scale of the torque bars (also scales the torque arcs on the arm). */
  let barMax = 10;
  let lastCompare = 0;

  // ---------- layout: chips, canvases
  const chips = presetChips(host, [], { label: '情境' });
  const [colA, colB] = split(host);
  const cvArm = new Canvas2D(colA, { aspect: 1, world: { xmin: -2.5, xmax: 2.5, ymin: -2.5, ymax: 2.5 }, overlay: true });
  const cvEll = new Canvas2D(colB, { aspect: 1, world: { xmin: -4, xmax: 4, ymin: -4, ymax: 4 } });

  // ---------- transport
  const transport = controlPanel(host, { compact: true });
  const tRow = row(transport);
  const pb = playback(tRow, {
    onPlay: () => play(),
    onPause: () => pause(),
    onReset: () => { restart(); },
    onSpeed: (s) => { loop.speed = s; },
    speeds: [0.25, 0.5, 1, 2],
  });
  const tGrav = toggle(tRow, { label: '重力（−y 方向）', value: true, onChange: (v) => setGravity(v) });
  const status = el('span', { class: 'mr-note', style: { margin: '0', color: 'var(--danger)' } });
  tRow.appendChild(status);

  const section = () => { const d = el('div'); host.appendChild(d); return d; };

  // ---------- section: energies (steps 1–2)
  const secEnergy = section();
  const eRow = el('div', { style: { display: 'flex', flexWrap: 'wrap', gap: '0.15rem 1.1rem' } });
  secEnergy.appendChild(eRow);
  const rK = readout(eRow, { label: '動能 K =', unit: 'J', minWidth: '6ch' });
  const rU = readout(eRow, { label: '位能 U =', unit: 'J', minWidth: '6ch' });
  const rE = readout(eRow, { label: '總能 E =', unit: 'J', minWidth: '6ch' });
  const rL = readout(eRow, { label: 'L =', unit: 'J', minWidth: '6ch' });
  relabel(rL.el.querySelector('.mr-label'), texLabel('\\mathcal L = K - U', '', ' ='));
  legend(secEnergy, [{ color: 'var(--series-1)', label: 'K 動能' }, { color: 'var(--series-3)', label: 'U 位能' }, { color: 'var(--fg)', label: 'E = K + U' }]);
  const ePlot = new Plot2D(secEnergy, {
    title: '能量（U 以手臂垂直下垂為零點）', xLabel: 't (s)', yLabel: '能量 (J)', height: 150, xWindow: 10, yPad: 0.12, legend: false,
    series: [{ name: 'K 動能', color: '--series-1' }, { name: 'U 位能', color: '--series-3' }, { name: 'E = K + U', color: '--fg', width: 2.4 }],
  });

  // ---------- section: Euler–Lagrange balance (step 2)
  const secEL = section();
  const thStyle = { style: { padding: '0.2rem 0.4rem', borderBottom: '1px solid var(--border)', fontWeight: '400', color: 'var(--fg-muted)', textAlign: 'right' } };
  const tdStyle = () => el('td', { style: { padding: '0.15rem 0.4rem', textAlign: 'right', fontFamily: 'var(--mono)', fontVariantNumeric: 'tabular-nums' } }, '–');
  const elCells: HTMLTableCellElement[][] = [];
  const elTable = el('table', { style: { borderCollapse: 'collapse', fontSize: '0.82rem', margin: '0.3rem 0 0.1rem', width: '100%' } },
    el('tr', {},
      el('th', thStyle, ''),
      el('th', thStyle, mathEl('p_i = \\tfrac{\\partial\\mathcal L}{\\partial\\dot\\theta_i}')),
      el('th', thStyle, mathEl('\\tfrac{d}{dt}p_i')),
      el('th', thStyle, mathEl('\\tfrac{\\partial\\mathcal L}{\\partial\\theta_i}')),
      el('th', thStyle, mathEl('\\tau_i = \\tfrac{d}{dt}p_i - \\tfrac{\\partial\\mathcal L}{\\partial\\theta_i}'))),
    ...[0, 1].map((i) => {
      const cells = [tdStyle(), tdStyle(), tdStyle(), tdStyle()];
      elCells.push(cells);
      return el('tr', {}, el('td', { style: { padding: '0.15rem 0.4rem', color: 'var(--fg-muted)' } }, `關節 ${i + 1}`), ...cells);
    }));
  const elWrap = el('div', { style: { overflowX: 'auto' } }, elTable);
  secEL.appendChild(elWrap);
  note(secEL, '單位：p 為 kg·m²/s，其餘為 N·m。表格約每 0.1 s 更新一次。');

  // ---------- section: mass-matrix ellipse (step 3)
  const secEll = section();
  const ellRow = el('div', { style: { display: 'flex', flexWrap: 'wrap', alignItems: 'center', gap: '0.2rem 1.2rem' } });
  secEll.appendChild(ellRow);
  const mvM = matrixView(ellRow, { label: 'M(θ) =', digits: 3 });
  const ellReads = el('div', { style: { display: 'grid', gap: '0.1rem' } });
  ellRow.appendChild(ellReads);
  const rL1 = readout(ellReads, { label: 'λ₁（重的方向）=', minWidth: '5ch' });
  const rL2 = readout(ellReads, { label: 'λ₂（輕的方向）=', minWidth: '5ch' });
  const rK0 = readout(ellReads, { label: '初速的動能 K₀ =', unit: 'J', minWidth: '5ch' });
  const ellCtl = controlPanel(secEll, { compact: true });
  const sTh2 = angleSlider(ellCtl, { label: 'θ₂（手肘角）', value: 0, onInput: (v) => setTheta2(v) });

  // ---------- section: torques (steps 4–5)
  const secTq = section();
  const tqCtl = controlPanel(secTq, { compact: true });
  const sOmega = slider(tqCtl, { label: 'dθ₁/dt 設定值', min: 0.5, max: 5, step: 0.1, value: omega1, unit: 'rad/s', onInput: (v) => { omega1 = v; if (mode === 'spinLocked' || mode === 'spinFree') { th0 = [x[0], mode === 'spinLocked' ? x[1] : th0[1]]; restart(); } } });
  relabel(sOmega.el.querySelector('.mr-label > span'), texLabel('\\dot\\theta_1', '', ' 設定值'));
  const sPeriod = slider(tqCtl, { label: '往復週期 T', min: 0.6, max: 8, step: 0.1, value: period, unit: 's', onInput: (v) => { period = v; if (mode === 'servo') restart(); } });
  const tqReads = el('div', { style: { display: 'grid', gridTemplateColumns: 'repeat(auto-fit, minmax(10rem, 1fr))', gap: '0.1rem 0.8rem' } });
  secTq.appendChild(tqReads);
  const rW1 = readout(tqReads, { label: 'dθ₁/dt =', unit: 'rad/s', minWidth: '6ch' });
  relabel(rW1.el.querySelector('.mr-label'), texLabel('\\dot\\theta_1', '', ' ='));
  const rP1 = readout(tqReads, { label: 'p₁ =', unit: 'kg·m²/s', minWidth: '6ch' });
  relabel(rP1.el.querySelector('.mr-label'), texLabel('p_1 = \\partial\\mathcal L/\\partial\\dot\\theta_1', '', ' ='));
  const [barsA, barsB] = split(secTq);
  const barLabels = ['馬達 τ', '慣性', '速度 c', '重力 g'];
  const barColors = ['--warn', '--series-1', '--series-5', '--series-3'];
  const bars1 = torqueBars(barsA, { labels: barLabels, max: 10, unit: 'N·m', title: '關節 1', colors: barColors });
  const bars2 = torqueBars(barsB, { labels: barLabels, max: 10, unit: 'N·m', title: '關節 2', colors: barColors });
  const barTex = (i: number) => [`\\tau_${i}`, `(M\\ddot\\theta)_${i}`, `c_${i}`, `g_${i}`];
  for (const [b, i] of [[bars1, 1], [bars2, 2]] as [BarsHandle, number][]) {
    b.el.querySelectorAll('.mr-bar-label').forEach((lab, k) => relabel(lab, mathEl(barTex(i)[k])));
  }
  const cBox = el('div');
  secTq.appendChild(cBox);
  legend(cBox, [{ color: 'var(--series-5)', label: 'c₁ 科氏項' }, { color: 'var(--series-4)', label: 'c₁ 向心項' }, { color: 'var(--series-2)', label: 'c₂ 向心項' }]);
  const cPlot = new Plot2D(cBox, {
    title: '速度相乘項：向心與科氏', xLabel: 't (s)', yLabel: 'N·m', height: 130, xWindow: 8, yPad: 0.12, yMinSpan: 2, legend: false,
    series: [{ name: 'c₁ 科氏', color: '--series-5' }, { name: 'c₁ 向心', color: '--series-4' }, { name: 'c₂ 向心', color: '--series-2' }],
  });
  const gBox = el('div');
  secTq.appendChild(gBox);
  legend(gBox, [{ color: 'var(--series-1)', label: '關節 1：τ₁ 實線、g₁ 虛線' }, { color: 'var(--series-2)', label: '關節 2：τ₂ 實線、g₂ 虛線' }]);
  const gPlot = new Plot2D(gBox, {
    title: '馬達力矩 τ 與重力項 g', xLabel: 't (s)', yLabel: 'N·m', height: 130, xWindow: 8, yPad: 0.12, yMinSpan: 2, legend: false,
    series: [{ name: 'τ₁', color: '--series-1' }, { name: 'g₁', color: '--series-1', dash: [5, 4] }, { name: 'τ₂', color: '--series-2' }, { name: 'g₂', color: '--series-2', dash: [5, 4] }],
  });

  // ---------- section: energy check (step 6)
  const secCheck = section();
  const ckCtl = controlPanel(secCheck, { compact: true });
  const selMethod = select<Method>(ckCtl, {
    label: '積分法', value: 'rk4',
    options: [{ value: 'rk4', label: 'RK4（四階）' }, { value: 'euler', label: '顯式 Euler' }],
    onChange: (v) => { method = v; chips.clear(); restart(); },
  });
  const selDt = select<string>(ckCtl, {
    label: '步長 Δt', value: '0',
    options: DT_CHOICES.map((d, i) => ({ value: String(i), label: `1/${Math.round(1 / d)} s` })),
    onChange: (v) => { dtIdx = Number(v); chips.clear(); restart(); },
  });
  const tDropC = toggle(row(ckCtl), { label: '漏掉 c 項（錯誤的模型）', value: false, onChange: (v) => { dropC = v; chips.clear(); restart(); } });
  const ckReads = el('div', { style: { display: 'grid', gridTemplateColumns: 'repeat(auto-fit, minmax(11rem, 1fr))', gap: '0.1rem 0.8rem' } });
  secCheck.appendChild(ckReads);
  const rDE = readout(ckReads, { label: 'ΔE（目前設定）=', unit: 'J', minWidth: '8ch' });
  const rDEref = readout(ckReads, { label: 'ΔE（參考解）=', minWidth: '8ch' });
  const dLegend = legend(secCheck, [{ color: 'var(--series-2)', label: '目前設定' }, { color: 'var(--fg-muted)', label: '參考解：RK4、完整模型（虛線）' }]);
  dLegend.style.marginTop = '0.4rem';
  const dPlot = new Plot2D(secCheck, {
    title: '能量誤差 ΔE(t) = E(t) − E(0)', xLabel: 't (s)', yLabel: 'ΔE (J)', height: 160, xWindow: 12, yMinSpan: 0.02, yPad: 0.12, legend: false,
    series: [{ name: '目前設定', color: '--series-2', width: 2.2 }, { name: '參考：RK4、完整模型', color: '--fg-muted', dash: [5, 4] }],
  });


  // ---------- section: closed form vs Newton–Euler (step 7)
  const secCmp = section();
  const cmpGrid = el('div', { style: { display: 'grid', gridTemplateColumns: 'repeat(auto-fit, minmax(14.5rem, 1fr))', gap: '0.3rem 1rem' } });
  secCmp.appendChild(cmpGrid);
  const mkCol = (title: string) => {
    const col = el('div', { style: { minWidth: '0' } }, el('div', { style: { fontWeight: '600', fontSize: '0.85rem', marginBottom: '0.15rem' } }, title));
    cmpGrid.appendChild(col);
    const box = el('div', { style: { display: 'flex', flexWrap: 'wrap', alignItems: 'center' } });
    col.appendChild(box);
    return { M: matrixView(box, { label: 'M =', digits: 4 }), c: matrixView(box, { label: 'c =', digits: 4 }), g: matrixView(box, { label: 'g =', digits: 4 }), tau: matrixView(box, { label: 'τ =', digits: 4 }) };
  };
  const cmpA = mkCol('閉式 Lagrange（planar2RDynamics）');
  const cmpB = mkCol('Newton–Euler（MassMatrix 等，@/mr）');
  const cmpReads = el('div', { style: { display: 'flex', flexWrap: 'wrap', gap: '0.1rem 1.2rem' } });
  secCmp.appendChild(cmpReads);
  const rDiff = readout(cmpReads, { label: '所有元素的最大差值 =', minWidth: '9ch' });
  secCmp.appendChild(mdElement('$\\tau$ 以測試加速度 $\\ddot\\theta = (1, -2)$ rad/s² 計算：閉式為 $M\\ddot\\theta + c + g$，函式庫為 `InverseDynamics`。重力向量 $(0, -\\mathfrak g, 0)$，與 2D 畫面的 −y 一致。', 'mr-note'));

  // ---------- parameters (always available)
  const details = el('details', { style: { margin: '0.4rem 0 0', fontSize: '0.85rem' } });
  details.appendChild(el('summary', { style: { cursor: 'pointer', color: 'var(--fg-muted)' } }, '手臂參數（質量、桿長）'));
  host.appendChild(details);
  const pCtl = controlPanel(details, { compact: true });
  const sM1 = slider(pCtl, { label: 'm₁', min: 0.2, max: 3, step: 0.1, value: arm.m1, unit: 'kg', onInput: (v) => setArm({ m1: v }) });
  const sM2 = slider(pCtl, { label: 'm₂', min: 0.2, max: 3, step: 0.1, value: arm.m2, unit: 'kg', onInput: (v) => setArm({ m2: v }) });
  const sL1 = slider(pCtl, { label: 'L₁', min: 0.4, max: 1.2, step: 0.05, value: arm.L1, unit: 'm', onInput: (v) => setArm({ L1: v }) });
  const sL2 = slider(pCtl, { label: 'L₂', min: 0.4, max: 1.2, step: 0.05, value: arm.L2, unit: 'm', onInput: (v) => setArm({ L2: v }) });

  // ---------- dynamics helpers
  const hNow = () => (mode === 'free' ? DT_CHOICES[dtIdx] : H_REF);
  const curTh = (): Vec => [x[0], x[1]];
  const needsRef = () => curStep === 5 && mode === 'free' && (method !== 'rk4' || dtIdx !== 0 || dropC);

  function buildPrograms(): [JointProgram, JointProgram] {
    switch (mode) {
      case 'free': return [freeJoint(), freeJoint()];
      case 'spinLocked': return [constantRate(th0[0], omega1), holdAt(th0[1])];
      case 'spinFree': return [constantRate(th0[0], omega1), freeJoint()];
      case 'skater': return [freeJoint(), foldCycle(-10 * DEG, -160 * DEG, 1, 1)];
      case 'hold': return [holdAt(th0[0]), holdAt(th0[1])];
      case 'servo': return [sineMotion(th0[0], 35 * DEG, period), sineMotion(th0[1], 40 * DEG, period, Math.PI / 2)];
      case 'sweep': return [holdAt(th0[0]), sineMotion(90 * DEG, 85 * DEG, 8, -Math.PI / 2)];
    }
  }

  function restart(): void {
    programs = buildPrograms();
    x = initialState(th0, mode === 'free' || mode === 'skater' || mode === 'spinFree' ? dth0 : [0, 0], programs);
    t = 0; acc = 0; blown = false;
    status.textContent = '';
    ref = needsRef() ? { x: x.slice(), t: 0 } : null;
    (dLegend.children[1] as HTMLElement).style.display = ref ? '' : 'none';
    rebaseline();
    barPeak = 1;
    refresh(true);
  }

  /** New energy reference (after a restart, a gravity switch or a parameter change). */
  function rebaseline(): void {
    E0 = energies(arm, x).E;
    if (ref) Eref0 = energies(arm, ref.x).E;
    trail.length = 0;
    ePlot.clear(); dPlot.clear(); cPlot.clear(); gPlot.clear();
    pushPlots();
  }

  function play(): void {
    if (blown) restart();
    if (mode === 'hold') { pause(); return; }
    loop.start();
    pb.setPlaying(true, { silent: true });
  }

  function pause(): void {
    loop.stop();
    pb.setPlaying(false, { silent: true });
  }

  function setGravity(on: boolean, silent = false): void {
    gravityOn = on;
    arm = { ...arm, g: on ? GRAVITY : 0 };
    tGrav.set(on, { silent: true });
    if (!silent) { rebaseline(); refresh(true); }
  }

  function setArm(p: Partial<Arm>, silent = false): void {
    arm = { ...arm, ...p, g: gravityOn ? GRAVITY : 0 };
    chain = chainFor(arm);
    sM1.set(arm.m1, { silent: true }); sM2.set(arm.m2, { silent: true });
    sL1.set(arm.L1, { silent: true }); sL2.set(arm.L2, { silent: true });
    const reach = arm.L1 + arm.L2;
    cvArm.setWorld({ xmin: -reach - 0.45, xmax: reach + 0.45, ymin: -reach - 0.45, ymax: reach + 0.45 });
    ellW = Math.max(2.5, 1.12 * maxEllipseSemiAxis(arm, K_REF));
    cvEll.setWorld({ xmin: -ellW, xmax: ellW, ymin: -ellW, ymax: ellW });
    ghosts = {
      straight: energyEllipse(dynamicsTerms(arm, [0, 0], [0, 0]).M, K_REF),
      folded: energyEllipse(dynamicsTerms(arm, [0, Math.PI], [0, 0]).M, K_REF),
    };
    dth0 = [Math.max(-ellW, Math.min(ellW, dth0[0])), Math.max(-ellW, Math.min(ellW, dth0[1]))];
    if (!silent) { rebaseline(); refresh(true); }
  }

  function setTheta2(v: number): void {
    pause();
    if (mode !== 'free' && mode !== 'hold') mode = 'free';
    chips.clear();
    th0 = [x[0], v];
    restart();
  }

  // ---------- main loop
  const loop = createLoop((dt) => {
    if (blown) return;
    const h = hNow();
    acc += dt;
    const start = performance.now();
    while (acc >= h - 1e-12) {
      x = stepState(arm, x, t, h, programs, method, dropC);
      t += h; acc -= h;
      if (performance.now() - start > 10) { acc = 0; break; }
    }
    if (ref) {
      while (ref.t + H_REF <= t + 1e-9) {
        ref.x = stepState(arm, ref.x, ref.t, H_REF, programs, 'rk4', false);
        ref.t += H_REF;
      }
    }
    if (!x.every(Number.isFinite) || Math.abs(x[2]) > VEL_LIMIT || Math.abs(x[3]) > VEL_LIMIT) {
      blown = true;
      pause();
      status.textContent = '數值發散：關節速度失控，已暫停（按「重設」重新開始）';
      return;
    }
    // Physics runs every frame; drawing (canvases, plots, readouts) at most every ~30 ms keeps the
    // page light on slow machines without changing the simulated motion.
    const now = performance.now();
    if (now - lastVisual < 30) return;
    lastVisual = now;
    const tip = massPositions(arm, curTh())[1];
    trail.push([tip[0], tip[1]]);
    if (trail.length > 100) trail.shift();
    pushPlots();
    refresh(false);
  });
  let lastVisual = 0;

  /** Feed only the plot shown in the current step (the others are cleared on every restart anyway). */
  function pushPlots(): void {
    if (curStep <= 1) {
      const e = energies(arm, x);
      ePlot.push(t, [e.K, e.U, e.E]);
      ePlot.setRanges([Math.max(0, t - 10), Math.max(10, t)]);
    } else if (curStep === 5) {
      const eRef = ref ? energies(arm, ref.x).E - Eref0 : undefined;
      dPlot.push(t, [energies(arm, x).E - E0, eRef] as unknown as number[]);
      dPlot.setRanges([Math.max(0, t - 12), Math.max(12, t)]);
    } else if (curStep === 3) {
      const sc = splitC(arm, curTh(), [x[2], x[3]]);
      cPlot.push(t, [sc.coriolis[0], sc.centripetal[0], sc.centripetal[1]]);
      cPlot.setRanges([Math.max(0, t - 8), Math.max(8, t)]);
    } else if (curStep === 4) {
      const r = evaluate(arm, x, t, programs, dropC);
      gPlot.push(t, [r.tau[0], r.g[0], r.tau[1], r.g[1]]);
      gPlot.setRanges([Math.max(0, t - 8), Math.max(8, t)]);
    }
  }

  // ---------- readouts
  let lastEL = 0;
  function refresh(force: boolean): void {
    last = evaluate(arm, x, t, programs, dropC);
    const th = curTh(), dth = [x[2], x[3]];
    const now = performance.now();
    pb.setTime(t);
    if (curStep <= 1) {
      const e = energies(arm, x);
      rK.set(e.K, 2); rU.set(e.U, 2); rE.set(e.E, 2); rL.set(e.K - e.U, 2);
    }
    if (curStep === 1 && (force || now - lastEL > 100)) {
      lastEL = now;
      const terms = eulerLagrangeTerms(arm, th, dth, last.ddth);
      for (let i = 0; i < 2; i++) {
        const vals = [terms.momentum[i], terms.dMomentum[i], terms.dLdTheta[i], terms.tau[i]];
        vals.forEach((v, k) => { const s = fmt(v, 2); if (elCells[i][k].textContent !== s) elCells[i][k].textContent = s; });
      }
    }
    if (curStep === 2) {
      mvM.set(last.M, { digits: 3 });
      const e = energyEllipse(last.M, K_REF);
      rL1.set(e.lambda[0], 3); rL2.set(e.lambda[1], 3);
      const M = dynamicsTerms(arm, th0, [0, 0]).M;
      rK0.set(0.5 * (M[0][0] * dth0[0] * dth0[0] + 2 * M[0][1] * dth0[0] * dth0[1] + M[1][1] * dth0[1] * dth0[1]), 3);
      sTh2.set(wrapPi(x[1]), { silent: true });
    }
    if (curStep === 3 || curStep === 4) {
      const parts = (i: number) => [last.tau[i], last.M[i][0] * last.ddth[0] + last.M[i][1] * last.ddth[1], last.c[i], last.g[i]];
      const v1 = parts(0), v2 = parts(1);
      const peak = Math.max(...v1.map(Math.abs), ...v2.map(Math.abs));
      barPeak = Math.max(peak, barPeak * 0.995);
      barMax = niceCeil(Math.max(2, barPeak * 1.1));
      bars1.setMax(barMax); bars2.setMax(barMax);
      bars1.set(v1); bars2.set(v2);
      rW1.set(x[2], 2);
      rP1.set(last.M[0][0] * x[2] + last.M[0][1] * x[3], 3);
    }
    if (curStep === 5) {
      rDE.set(sci(energies(arm, x).E - E0, 2));
      rDEref.set(ref ? `${sci(energies(arm, ref.x).E - Eref0, 2)} J` : '與目前設定相同');
    }
    if (curStep === 6 && (force || now - lastCompare > 200)) {
      lastCompare = now;
      const cmp = compareWithNewtonEuler(arm, chain, th, dth, TEST_ACC);
      cmpA.M.set(cmp.closed.M); cmpA.c.set(cmp.closed.c); cmpA.g.set(cmp.closed.g); cmpA.tau.set(cmp.closed.tau);
      cmpB.M.set(cmp.ne.M); cmpB.c.set(cmp.ne.c); cmpB.g.set(cmp.ne.g); cmpB.tau.set(cmp.ne.tau);
      rDiff.set(sci(cmp.maxDiff, 1));
    }
    cvArm.requestRender();
    // M(θ) only depends on θ2: redraw the ellipse panel when θ2 actually moved
    if (force || Math.abs(x[1] - lastEllTh2) > 0.01) { lastEllTh2 = x[1]; cvEll.requestRender(); }
  }
  let lastEllTh2 = NaN;

  // ---------- arm canvas
  let drag: 0 | 1 | 2 = 0;
  let wasPlaying = false;
  const torqueArc = (c: Canvas2D, center: Pt, tau: number, color: string, label: string) => {
    const span = Math.min(1.5 * Math.PI * Math.abs(tau) / barMax, 1.5 * Math.PI);
    if (span < 0.06) return;
    const r = 0.26, a0 = Math.PI / 2, a1 = a0 + Math.sign(tau) * span;
    c.arc(center, r, Math.min(a0, a1), Math.max(a0, a1), { color, width: 2.6 });
    const tip: Pt = [center[0] + r * Math.cos(a1), center[1] + r * Math.sin(a1)];
    const s = Math.sign(tau);
    const tan: Pt = [-Math.sin(a1) * s, Math.cos(a1) * s];
    c.arrow([tip[0] - tan[0] * 0.07, tip[1] - tan[1] * 0.07], [tip[0] + tan[0] * 0.03, tip[1] + tan[1] * 0.03], { color, width: 2.6, head: 9 });
    c.text([center[0] - 0.36, center[1] + 0.3], label, { color, font: '600 12px sans-serif' });
  };

  const angleArc = (c: Canvas2D, center: Pt, from: number, to: number, label: string) => {
    if (Math.abs(to - from) < 4 * DEG) return;
    const r = 0.34;
    c.line(center, [center[0] + (r + 0.14) * Math.cos(from), center[1] + (r + 0.14) * Math.sin(from)], { color: '--warn', width: 1, dash: [3, 3] });
    c.arc(center, r, Math.min(from, to), Math.max(from, to), { color: '--warn', width: 1.8 });
    const mid = (from + to) / 2;
    c.text([center[0] + (r + 0.2) * Math.cos(mid), center[1] + (r + 0.2) * Math.sin(mid)], label, { color: '--warn' });
  };

  cvArm.draw = (c) => {
    const th = curTh();
    const L = [arm.L1, arm.L2];
    const v = c.visible();
    c.grid(0.5);
    c.circle([0, 0], arm.L1 + arm.L2, { color: '--grid-strong', dash: [4, 4], width: 1 });
    if (gravityOn) {
      c.arrow([v.xmax - 0.3, v.ymin + 0.95], [v.xmax - 0.3, v.ymin + 0.35], { color: '--fg-muted', width: 2 });
      c.text([v.xmax - 0.3, v.ymin + 0.95], 'g', { dx: -10, color: '--fg-muted', font: 'italic 14px serif' });
    } else {
      c.text([v.xmax - 0.12, v.ymin + 0.25], '水平面：無重力', { align: 'right', color: '--fg-muted' });
    }
    if (trail.length > 1) c.polyline(trail, { color: '--warn', width: 1.5, alpha: 0.5 });
    if (ref) {
      const rth = [ref.x[0], ref.x[1]];
      drawPlanarArm(c, L, rth, { alpha: 0.28, linkWidth: 6, eeRadius: 0 });
      const [q1, q2] = massPositions(arm, rth);
      c.disk(q1 as Pt, massRadius(arm.m1), { fill: '--series-1', alpha: 0.28 });
      c.disk(q2 as Pt, massRadius(arm.m2), { fill: '--series-2', alpha: 0.28 });
      c.text(q2 as Pt, '參考解', { dy: -20, color: '--fg-muted' });
    }
    drawPlanarArm(c, L, th, { linkWidth: 6, eeRadius: 0 });
    const [p1, p2] = massPositions(arm, th) as [Pt, Pt];
    if (curStep === 2 || (curStep === 4 && mode === 'hold')) {
      const a1 = wrapPi(th[0]), a2 = wrapPi(th[1]);
      angleArc(c, [0, 0], 0, a1, 'θ₁');
      angleArc(c, p1, a1, a1 + a2, 'θ₂');
    }
    if (mode === 'spinLocked') {
      const d = Math.hypot(p2[0], p2[1]);
      if (d > 1e-6) {
        const u: Pt = [-p2[0] / d, -p2[1] / d];
        c.arrow(p2, [p2[0] + u[0] * 0.5, p2[1] + u[1] * 0.5], { color: '--series-5', width: 2 });
        c.text([p2[0] + u[0] * 0.3 - u[1] * 0.22, p2[1] + u[1] * 0.3 + u[0] * 0.22], '向心', { color: '--series-5', bg: '--bg-elev' });
      }
      c.polygon([[p1[0] - 0.13, p1[1] - 0.13], [p1[0] + 0.13, p1[1] - 0.13], [p1[0] + 0.13, p1[1] + 0.13], [p1[0] - 0.13, p1[1] + 0.13]], { color: '--warn', width: 1.5, dash: [3, 2] });
    }
    c.disk(p1, massRadius(arm.m1), { fill: '--series-1' });
    c.disk(p2, massRadius(arm.m2), { fill: '--series-2' });
    c.text(p1, 'm₁', { dx: 15, dy: 15, color: '--series-1' });
    c.text(p2, 'm₂', { dx: 15, dy: 15, color: '--series-2' });
    if (mode !== 'free' && (curStep === 3 || curStep === 4)) {
      if (programs[0].kind === 'driven') torqueArc(c, [0, 0], last.tau[0], '--warn', 'τ₁');
      if (programs[1].kind === 'driven') torqueArc(c, p1, last.tau[1], '--warn', 'τ₂');
    }
    if (drag) c.text(drag === 1 ? p1 : p2, '拖曳中', { dy: 22, color: '--accent' });
    c.setOverlayText(`θ = (${fmt(wrapPi(th[0]) / DEG, 0)}°, ${fmt(wrapPi(th[1]) / DEG, 0)}°)   ${MODE_LABEL[mode]}`);
  };

  // drag the point masses directly (elbow → θ1, tip → θ2)
  const tipDraggable = () => mode === 'free' || mode === 'hold' || mode === 'spinLocked' || mode === 'spinFree';
  const elbowDraggable = () => mode !== 'servo' && mode !== 'skater';
  const hitMass = (p: Pt): 0 | 1 | 2 => {
    const [p1, p2] = massPositions(arm, curTh());
    const tol = 14 / cvArm.scale;
    if (tipDraggable() && Math.hypot(p[0] - p2[0], p[1] - p2[1]) <= Math.max(tol, massRadius(arm.m2) * 1.3)) return 2;
    if (elbowDraggable() && Math.hypot(p[0] - p1[0], p[1] - p1[1]) <= Math.max(tol, massRadius(arm.m1) * 1.3)) return 1;
    return 0;
  };
  cvArm.onPointer({
    down: (p, ev) => {
      const which = hitMass(p);
      if (!which) return;
      drag = which;
      wasPlaying = loop.running;
      pause();
      cvArm.canvas.setPointerCapture(ev.pointerId);
      cvArm.canvas.style.cursor = 'grabbing';
      ev.preventDefault();
    },
    move: (p) => {
      if (!drag) { cvArm.canvas.style.cursor = hitMass(p) ? 'grab' : ''; return; }
      chips.clear();
      if (drag === 1) {
        th0 = [Math.atan2(p[1], p[0]), x[1]];
      } else {
        const [p1] = massPositions(arm, curTh());
        th0 = [x[0], wrapPi(Math.atan2(p[1] - p1[1], p[0] - p1[0]) - x[0])];
      }
      if (mode === 'sweep') mode = 'free';
      restart();
    },
    up: () => {
      if (!drag) return;
      drag = 0;
      cvArm.canvas.style.cursor = '';
      cvArm.requestRender();
      if (wasPlaying) play();
    },
  });

  // ---------- ellipse canvas
  cvEll.draw = (c) => {
    const v = c.visible();
    const u = 1 / c.scale; // world units per pixel
    c.grid(1);
    c.text([v.xmax, 0], 'dθ₁/dt', { align: 'right', dx: -4, dy: 11, color: '--fg-muted' });
    c.text([0, v.ymax], 'dθ₂/dt', { align: 'left', dx: 6, dy: 10, color: '--fg-muted' });
    if (ghosts) {
      const gs = ghosts.straight, gf = ghosts.folded;
      c.ellipse([0, 0], gs.semi[1], gs.semi[0], gs.longAngle, { color: '--fg-muted', dash: [5, 4], width: 1.2, alpha: 0.85 });
      c.ellipse([0, 0], gf.semi[1], gf.semi[0], gf.longAngle, { color: '--fg-muted', dash: [2, 3], width: 1.2, alpha: 0.85 });
    }
    const e = energyEllipse(last.M, K_REF);
    c.ellipse([0, 0], e.semi[1], e.semi[0], e.longAngle, { fill: '--accent', alpha: 0.12, width: 0 });
    c.ellipse([0, 0], e.semi[1], e.semi[0], e.longAngle, { color: '--accent', width: 2.4 });
    const hv = e.vectors[0], lv = e.vectors[1];
    c.arrow([0, 0], [hv[0] * e.semi[0], hv[1] * e.semi[0]], { color: '--series-2', width: 2.2 });
    c.arrow([0, 0], [lv[0] * e.semi[1], lv[1] * e.semi[1]], { color: '--series-3', width: 2.2 });
    // legends: caption + eigenvalues (top-left), ghost outlines (bottom-left)
    const lx = v.xmin + 8 * u, ly = v.ymax - 13 * u;
    c.text([lx, ly], 'K = 1 J 的等動能橢圓', { align: 'left', color: '--accent', bg: '--bg-elev' });
    c.text([lx, ly - 19 * u], `λ₁ = ${fmt(e.lambda[0], 2)}（重）`, { align: 'left', color: '--series-2', bg: '--bg-elev' });
    c.text([lx, ly - 38 * u], `λ₂ = ${fmt(e.lambda[1], 2)}（輕）`, { align: 'left', color: '--series-3', bg: '--bg-elev' });
    const gy = v.ymin + 30 * u;
    c.line([lx, gy], [lx + 22 * u, gy], { color: '--fg-muted', dash: [5, 4], width: 1.2 });
    c.text([lx + 28 * u, gy], '伸直 θ₂ = 0°', { align: 'left', color: '--fg-muted', bg: '--bg-elev' });
    c.line([lx, gy - 17 * u], [lx + 22 * u, gy - 17 * u], { color: '--fg-muted', dash: [2, 3], width: 1.2 });
    c.text([lx + 28 * u, gy - 17 * u], '折疊 θ₂ = 180°', { align: 'left', color: '--fg-muted', bg: '--bg-elev' });
    if (mode === 'free') c.text([dth0[0], dth0[1]], '初速', { align: 'left', dx: 10, dy: 13, color: '--warn', bg: '--bg-elev' });
  };
  cvEll.addHandle({
    get: () => [dth0[0], dth0[1]],
    set: (p) => {
      dth0 = [Math.max(-ellW, Math.min(ellW, p[0])), Math.max(-ellW, Math.min(ellW, p[1]))];
      th0 = curTh();
      chips.clear();
      restart();
    },
    onStart: () => { wasPlaying = loop.running; pause(); },
    onEnd: () => { if (wasPlaying) play(); },
    color: '--warn',
    radius: 7,
    get enabled() { return mode === 'free'; },
  });

  // ---------- presets & steps
  function applyPreset(p: PresetDef): void {
    pause();
    if (p.arm) setArm(p.arm, true);
    mode = p.mode;
    th0 = [p.th[0] * DEG, p.th[1] * DEG];
    dth0 = p.dth ? [p.dth[0], p.dth[1]] : [0, 0];
    setGravity(p.gravity, true);
    if (p.period != null) { period = p.period; sPeriod.set(period, { silent: true }); }
    if (p.method) { method = p.method; selMethod.set(method, { silent: true }); }
    if (p.dtIdx != null) { dtIdx = p.dtIdx; selDt.set(String(dtIdx), { silent: true }); }
    if (p.dropC != null) { dropC = p.dropC; tDropC.set(dropC, { silent: true }); }
    restart();
    updateVisibility();
    if (p.play) play();
  }

  const sections = [secEnergy, secEL, secEll, secTq, secCheck, secCmp];
  const visibleIn: HTMLElement[][] = [
    [secEnergy], [secEnergy, secEL], [secEll], [secTq], [secTq], [secCheck], [secCmp],
  ];
  const focusOf: (() => HTMLElement[])[] = [
    () => [cvArm.el],
    () => [elWrap],
    () => [cvEll.el],
    () => [bars2.el],
    () => [bars1.el, tGrav.el],
    () => [dPlot.el],
    () => [cmpGrid],
  ];
  let focused: HTMLElement[] = [];

  function updateVisibility(): void {
    const i = curStep;
    sOmega.el.style.display = i === 3 && (mode === 'spinLocked' || mode === 'spinFree') ? '' : 'none';
    sPeriod.el.style.display = i === 4 && mode === 'servo' ? '' : 'none';
    cBox.style.display = i === 3 ? '' : 'none';
    gBox.style.display = i === 4 && mode === 'servo' ? '' : 'none';
    tqReads.style.display = i === 3 ? '' : 'none';
  }

  function onStep(i: number): void {
    const prev = curStep;
    curStep = i;
    sections.forEach((s) => { s.style.display = visibleIn[i].includes(s) ? '' : 'none'; });
    updateVisibility();
    focused.forEach((f) => f.classList.remove('mr-focus'));
    focused = focusOf[i]();
    focused.forEach((f) => f.classList.add('mr-focus'));
    chips.setPresets(STEP_PRESETS[i].map((p) => ({ label: p.label, title: p.title, apply: () => applyPreset(p) })));
    const settingsDefault = method === 'rk4' && dtIdx === 0 && !dropC;
    // Free-swing steps continue the learner's current swing instead of resetting it.
    const continueSwing = (i === 0 || i === 1 || i === 6) && (prev === 0 || prev === 1 || prev === 6)
      && mode === 'free' && !blown && settingsDefault && prev !== i;
    if (continueSwing) {
      if (!loop.running) play();
      refresh(true);
    } else {
      if (i !== 5 && !settingsDefault) {
        method = 'rk4'; dtIdx = 0; dropC = false;
        selMethod.set('rk4', { silent: true }); selDt.set('0', { silent: true }); tDropC.set(false, { silent: true });
      }
      chips.activate(0);
    }
  }

  setArm({}, true);
  onStep(ctx.currentStep);

  return {
    onStep,
    destroy() {
      loop.stop();
      cvArm.destroy();
      cvEll.destroy();
      ePlot.destroy(); dPlot.destroy(); cPlot.destroy(); gPlot.destroy();
    },
  };
}

const lesson: Lesson = { meta, steps, quiz, mount };
export default lesson;
