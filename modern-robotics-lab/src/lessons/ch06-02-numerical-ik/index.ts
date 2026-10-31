import type { Lesson, LessonContext, LessonHandle, LessonStep, QuizQuestion } from '@/types';
import {
  el, fmt, cssVar, controlPanel, row, slider, angleSlider, select, toggle, simTitle, kv, legend, mdElement, split,
  Canvas2D, Plot2D, createLoop, presetChips, algorithmStepper, playback, planarArmPoints,
  type Pt, type Handle, type PresetChipsHandle, type SeriesSpec, type Marker,
} from '@/ui';
import { type Vec, dot, norm, wrapToPi } from '@/mr';
import { meta } from './meta';
import {
  DEFAULT_TASK, FAR_TASK, GUESS, LESSON_TOL, LINKS, REACH, TRACK_PATHS, W_JOINT1, advanceTracking, circleTarget, evaluate,
  iterate, nullSpaceDirection, poseReachable, reducedJacobian, residual, runNewton, screwArc, singularValues, statusOf,
  trackingStart, type IterRecord, type Method, type RunStatus, type StepOptions, type Task, type TrackState,
} from './newton';
import { LabelLayer } from './labels';

const D = Math.PI / 180;
const MAX_ITER = 60;
const KNOB = 0.45;
/** Slider value of "最大步長" that means "no cap". */
const STEP_INF = 3.2;
const K_TRACK = 5;
/** Seconds of tracking simulated instantly when a path (re)starts, so the plots are never empty. */
const PREROLL = 2.5;
/** Canvas world: covers every preset configuration and both tracking circles. */
const WORLD = { xmin: -1.6, xmax: 2.75, ymin: -1.45, ymax: 2.05 };

const TRACK_LABELS: { label: string; title: string }[] = [
  { label: '圓形路徑（全程可達）', title: '圓心 (1.1, 0.9)、半徑 0.5 m，朝向在 30° ± 17° 間擺動；離奇異構型很遠' },
  { label: '伸出工作空間', title: '圓心 (1.4, 1.05)、半徑 0.75 m，朝向始終背離基座；圓的外側約有 2 s 超出 2.4 m 的可達半徑' },
];

const steps: LessonStep[] = [
  {
    title: '把 IK 變成求根',
    md: `沒有封閉解時，就把 IK 改寫成**求根（root finding）**：找 $\\theta$，讓目前的末端位姿 $T_{sb}(\\theta)$ 與目標 $T_{sd}$ 之間的「差」等於零。

位姿不能直接相減：$T_{sd} - T_{sb}$ 既不是剛體運動，也不是 twist。正確的問法是：在 $\\{b\\}$ 裡要施加哪個 twist、持續一秒，才能把 $\\{b\\}$ 帶到 $\\{d\\}$？答案就是矩陣對數

$$[\\mathcal V_b] = \\log\\big(T_{sb}^{-1}(\\theta)\\,T_{sd}\\big).$$

IK 的目標是讓 $\\mathcal V_b = 0$；實作上要求 $\\|\\omega_b\\|$ 與 $\\|v_b\\|$ 都小於容許誤差。`,
    simHint: '青色虛線是誤差 twist V_b 的螺旋路徑：{b} 沿著它走一秒，正好疊到 {d}。拖曳紫色目標點與朝向旋鈕，看 ‖ω_b‖、‖v_b‖ 怎麼變。',
  },
  {
    title: '一步 Newton',
    md: `$\\mathcal V_b$ 對 $\\theta$ 是非線性的，但在目前的 $\\theta^i$ 附近可以用 body Jacobian 線性化：關節改變 $\\Delta\\theta$，末端大約產生 twist $J_b(\\theta^i)\\Delta\\theta$。令它等於 $\\mathcal V_b$，解出 $\\Delta\\theta$：

$$\\theta^{i+1} = \\theta^i + J_b^{\\dagger}(\\theta^i)\\,\\mathcal V_b .$$

$J_b^{\\dagger}$ 是偽逆（pseudoinverse）；平面 3R 追蹤位置加朝向時 $J_b$ 是 3×3，可逆時 $J_b^{\\dagger} = J_b^{-1}$。線性近似不完美，一步通常不會正好到位，但會更接近；在新的位置重新線性化，再走一步。`,
    simHint: '已先走了一步：灰色虛線是初值 θ⁰，藍色手臂是 θ¹，讀數是這一步的 Δθ。再按「▸ 單步」幾次，直到出現「✓ 收斂」。',
  },
  {
    title: '收斂曲線與初值',
    md: `左下圖用對數縱軸畫每次迭代的誤差。初值夠近時，誤差會**加速**下降，每一步的有效位數大約加倍（$10^{-2} \\to 10^{-4} \\to 10^{-8}$），這就是 Newton 法的**二次收斂**，通常 3–5 步就過門檻。（平面手臂的朝向 $\\theta_1 + \\theta_2 + \\theta_3$ 對 $\\theta$ 是線性的，所以 $\\|\\omega_b\\|$ 一步就歸零；請看 $\\|v_b\\|$ 那條。）

初值離得遠時線性化很不準，前幾步可能亂跳，最後可能收斂到**另一組解**（手肘翻到另一側），也可能停滯。數值 IK 一次只給一組解，給哪一組由初值決定；實務上常拿上一時刻的關節角當初值。`,
    simHint: '實線是「好的初值」跑完的結果，虛線是「壞的初值」：前幾步大跳，最後停在手肘朝另一側的解（玫瑰色虛線手臂）。拖曳灰色 θ⁰ 手臂的關節，曲線會即時重算。',
  },
  {
    title: '奇異附近會發生什麼',
    md: `用奇異值分解看偽逆：$J = \\sum_i \\sigma_i u_i v_i^{\\T}$，$J^{\\dagger}$ 把誤差在 $u_i$ 方向的分量除以 $\\sigma_i$。手臂接近奇異時（例如 $\\theta_2 \\approx 0$，前兩節幾乎伸直），最小的 $\\sigma$ 趨近 0，那個方向的誤差被放大成巨大的 $\\Delta\\theta$：手臂一步轉好幾圈，線性近似完全失效。

常見補救有兩種：限制每步的 $\\|\\Delta\\theta\\|$（最大步長），或改用下一步的阻尼最小平方。`,
    simHint: '已從近奇異初值（θ₂ = 3°）走了一步：手臂連轉好幾圈，右下圖 ‖Δθ‖ 衝到約 41 rad。按 ▶ 自動看後續；再按「最大步長 = 1 rad」比較。',
  },
  {
    title: '阻尼最小平方',
    md: `阻尼最小平方（damped least squares, DLS）改用

$$\\theta^{i+1} = \\theta^i + J_b^{\\T}\\big(J_bJ_b^{\\T} + \\lambda^2 I\\big)^{-1}\\mathcal V_b .$$

在每個奇異值方向上，它把 $1/\\sigma$ 換成 $\\sigma/(\\sigma^2 + \\lambda^2)$：$\\sigma \\gg \\lambda$ 時幾乎與偽逆相同；這個增益最大只有 $1/(2\\lambda)$（發生在 $\\sigma = \\lambda$），$\\sigma \\to 0$ 時反而趨近 0，所以步長永遠有界。代價是每一步都略偏離精確的 Newton 步，收斂從二次退化成線性——$\\lambda$ 越大越穩，也越慢。`,
    simHint: '同一個近奇異初值：實線是 DLS（λ = 0.1），虛線是 J⁺。比較兩張圖的步長與收斂步數；再按「λ = 0.5」，看誤差曲線變得多平緩。',
  },
  {
    title: '冗餘與最小範數解',
    md: `只要求末端**位置**時，3R 多出一個自由度：$J_v$ 是 2×3，$J_v\\Delta\\theta = v$ 有無限多解，彼此只差一個**零空間**（null space）分量——末端不動、只改變手臂姿勢的自運動（方向記為 $\\hat n$）。偽逆挑的是其中 $\\|\\Delta\\theta\\|$ 最小的一個，它與零空間正交：$\\Delta\\theta \\cdot \\hat n = 0$。

若某個關節比較「貴」（慢、耗能），可改用加權偽逆，最小化 $\\Delta\\theta^{\\T}W\\Delta\\theta$。這時步長帶有零空間分量，收斂到的姿勢也跟著改變。`,
    simHint: '已切到「只要位置」：粉紅箭頭是零空間自運動（末端不動），讀數顯示第 1 步的 Δθ·n̂ ≈ 0。玫瑰色虛線是加權 J⁺ 的終點：關節 1 只轉約 19°，一般 J⁺ 轉了 44°。按「加權 J⁺」讓兩者互換。',
  },
  {
    title: '逆速度運動學與下一章',
    md: `把 Newton 步除以時間，就是**逆速度運動學**：想讓末端以 twist $\\mathcal V$ 運動，就令

$$\\dot\\theta = J^{\\dagger}(\\theta)\\,\\mathcal V .$$

追蹤移動目標時，每個控制週期把「目標本身的 twist」加上「剩餘誤差 × 增益」當成 $\\mathcal V$，等於連續版的 Newton 迭代。目標若短暫跑出工作空間，手臂只能伸直、貼近奇異構型，$J^{\\dagger}$ 會要求極大的關節速度；DLS 則讓手臂停在邊界等待，犧牲一點追蹤誤差，換來有界的 $\\dot\\theta$。

下一章轉向閉鏈機構——在那裡，連正向運動學都得解方程式。`,
    simHint: '目標沿圓移動，手臂以 dθ/dt = J⁺V 跟隨。按「伸出工作空間」：圓的外側超出可達半徑，比較 J⁺ 與 DLS 的 ‖dθ/dt‖ 尖峰與追蹤誤差，最後做下方測驗。',
  },
];

const quiz: QuizQuestion[] = [
  {
    question: '只要位置的 3R 手臂（總長 2.4 m），目標在 $|p| = 2.67$ m 處，超出可達範圍。迭代會怎樣？',
    options: [
      '只要迭代次數夠多，殘差終究會降到容許誤差以下，只是比可達目標慢',
      '殘差卡在約 0.27 m；最近的姿勢正是伸直的奇異構型，$J^{\\dagger}$ 在附近亂跳',
      '每一步都把關節往同一方向推，關節角越轉越多，最後發散到無限大',
      '收斂到可達範圍內離目標最遠的姿勢（手臂指向反方向），殘差約 $|p| + 2.4$ m',
    ],
    answer: 1,
    explanation: '$f(\\theta) = 0$ 根本沒有解，多迭代幾次也不會收斂；迭代只會讓殘差變小，不會往「最遠」的姿勢跑。殘差最小的姿勢是手臂伸直、指向目標，殘差 $|p| - 2.4 \\approx 0.27$ m，偏偏這正是徑向的奇異構型：$\\sigma_{\\min} \\to 0$，偽逆步長 $\\propto 1/\\sigma_{\\min}$ 在附近暴衝、來回振盪（角度每步包裹，不會跑到無限大）。阻尼足夠（例如 $\\lambda = 0.5$）的 DLS 則會穩定停在伸直姿勢。實作上要偵測「殘差不再下降」並回報不可達。第 6 步的「不可達目標」預設可以直接驗證：實線（$J^{\\dagger}$）亂跳後停滯，虛線（DLS）停在伸直姿勢。',
  },
  {
    question: '冗餘手臂（$J$ 是 2×3）用 $\\Delta\\theta = J^{\\dagger}v$。這一步有什麼幾何意義？改用 $W = \\diag(10, 1, 1)$ 的加權偽逆又如何？',
    options: [
      '$J^{\\dagger}v$ 讓三個關節轉一樣多；加權後只剩關節 1 在動，其他關節不動',
      '$J^{\\dagger}v$ 是最小範數解；加權後末端不再沿 $v$ 的方向移動，需另外修正',
      '$J^{\\dagger}v$ 是最小範數解、與零空間正交；加權後關節 1 少動、多了零空間分量',
      '$J^{\\dagger}$ 只對方陣有定義，冗餘手臂得先鎖住一個關節才能用',
    ],
    answer: 2,
    explanation: '$J\\Delta\\theta = v$ 的解是一條直線 $\\Delta\\theta_p + s\\,\\hat n$。最小範數解是這條線上離原點最近的點，必然與 $\\hat n$ 正交。加權版本仍然精確滿足 $J\\Delta\\theta = v$（末端方向不變），只是改用 $W$ 量「長度」：關節 1 的權重是 10，最省的作法是讓它少動，於是步長在 $\\hat n$ 上有了分量，$\\|\\Delta\\theta\\|$ 反而變大。兩種解法下各關節的轉量一般都不相同；偽逆也不限於方陣，2×3 的 $J$ 正是它最典型的用途。',
  },
  {
    question: '為什麼逆速度運動學 $\\dot\\theta = J^{\\dagger}\\mathcal V$ 與 Newton 步 $\\Delta\\theta = J_b^{\\dagger}\\mathcal V_b$ 長得一模一樣？',
    options: [
      '巧合：一個是微分關係、一個是最佳化，兩者的推導互不相關',
      '因為對任何手臂都有 $J^{\\dagger} = J^{\\T}$，所以兩個式子其實都是轉置法',
      '因為收斂時 $\\mathcal V_b = 0$，兩式只在迭代終點那一刻才恰好相同，平常並不相等',
      '兩者都在解 $J\\,\\delta\\theta = \\mathcal V$；Newton 步把 $\\mathcal V_b$ 當成持續 1 秒的速度',
    ],
    answer: 3,
    explanation: '$\\mathcal V = J(\\theta)\\dot\\theta$ 是速度層級的精確關係；Newton 法把它用在有限的步長上：$J\\,\\Delta\\theta \\approx \\mathcal V_b \\cdot (1\\ \\text{s})$。把 Newton 步除以時間步長 $\\Delta t$ 就是 $\\dot\\theta$，所以追蹤控制可以看成「每個控制週期做一次 Newton 修正」，奇異附近的問題與補救（步長限制、DLS）也完全相同。這與 $J^{\\dagger} = J^{\\T}$ 無關（一般並不成立），也不是只在收斂時才一致：每一步的形式都相同。',
  },
  {
    question: '某一步 $J$ 的最小奇異值 $\\sigma = 0.01$，誤差在對應方向的分量是 0.1。偽逆與 DLS（$\\lambda = 0.1$）在這個方向各走多少？',
    options: [
      '$J^{\\dagger}$：10 rad；DLS：約 0.099 rad',
      '兩者都是 10 rad',
      '$J^{\\dagger}$：0.001 rad；DLS：10 rad',
      '$J^{\\dagger}$：10 rad；DLS：0（完全不動）',
    ],
    answer: 0,
    explanation: '偽逆除以 $\\sigma$：$0.1 / 0.01 = 10$ rad，一步轉超過一圈半。DLS 乘上 $\\sigma/(\\sigma^2 + \\lambda^2) = 0.01/(0.0001 + 0.01) \\approx 0.99$，只走約 0.099 rad。遠離奇異（$\\sigma \\gg \\lambda$）時兩者幾乎相同；接近奇異時 DLS 把增益壓在 $1/(2\\lambda)$ 以下，換來有界但不精確的步長。',
  },
];

const FIVE_MD = `**下一章預告：閉鏈（closed chain）**

五連桿的兩個馬達角一給定，末端 $P$ 就是兩個圓的交點。在閉鏈裡連**正向**運動學都要解方程式，而且有兩種組裝方式（虛線是另一種）。`;

const SUB = '₀₁₂₃₄₅₆₇₈₉';
const fe = (x: number) => (x === 0 ? '0' : x.toExponential(1));
const lg = (x: number) => Math.log10(Math.max(x, 1e-13));
/** Step norms: fixed point when readable, exponent form when tiny. */
const fmtNorm = (x: number) => (x >= 1e-3 || x === 0 ? fmt(x, 3) : x.toExponential(1));
const MONO = '12px ui-monospace, SFMono-Regular, Menlo, monospace';

/** Vertical bars from 0 to each value at x = k + dx; NaN entries break the plotted polyline between bars. */
function stems(vals: number[], dx: number): [number[], number[]] {
  const xs: number[] = [];
  const ys: number[] = [];
  vals.forEach((v, k) => { xs.push(k + dx, k + dx, NaN); ys.push(0, v, NaN); });
  return [xs, ys];
}

type CmpKind = 'method' | 'guess' | 'weights';
interface Comparison { records: IterRecord[]; status: RunStatus; label: string; kind: CmpKind }

function mount(host: HTMLElement, ctx: LessonContext): LessonHandle {
  // ------------------------------------------------------------------ state
  let task: Task = { ...DEFAULT_TASK };
  let theta0: Vec = GUESS.start.slice();
  const opts: StepOptions = { method: 'pinv', lambda: 0.1, maxStep: Infinity, weights: null };
  let records: IterRecord[] = [evaluate(theta0, task)];
  let status: RunStatus = 'running';
  let cmp: Comparison | null = null;
  const show = { twist: true, iterates: true, compare: false, nullspace: false };
  let anim: { from: Vec; d: Vec; t: number; dur: number } | null = null;
  /** True while a run is completed programmatically (no per-step animation or redraw). */
  let instant = false;
  let step = -1;
  let mode: 'iterate' | 'track' = 'iterate';
  let trackIdx = 0;
  const pathFn = (t: number) => circleTarget(TRACK_PATHS[trackIdx], t);
  let trk: TrackState = trackingStart(pathFn, GUESS.good);
  let tipTrail: Pt[] = [];
  /** (t, ‖dθ/dt‖) samples inside the plot window, to keep the rate plot's y-range zero-based. */
  let rateWin: [number, number][] = [];
  let prevNull: Vec | null = null;

  // ------------------------------------------------------------------ layout
  simTitle(host, '數值 IK：一步一步逼近目標', '平面 3R');
  const cv = new Canvas2D(host, { height: 312, world: WORLD, overlay: true });

  // One HTML legend line for both iteration plots (in-plot legends would cover the first iterates).
  // Solid = current settings; the dashed comparison run is described by `legendCmp`.
  const legendRow = el('div', { style: { display: 'flex', flexWrap: 'wrap', alignItems: 'center', gap: '0 0.9rem', margin: '0.35rem 0 0' } });
  host.appendChild(legendRow);
  legend(legendRow, [
    { color: 'var(--series-1)', label: '‖ω_b‖' },
    { color: 'var(--series-2)', label: '‖v_b‖' },
    { color: 'var(--series-4)', label: '‖Δθ‖' },
  ]).style.margin = '0';
  const legendCmp = el('span', { class: 'mr-legend', style: { margin: '0' } });
  legendRow.appendChild(legendCmp);

  // Comparison series are dashed copies of the main ones; they only get data while a comparison exists.
  const seriesE: SeriesSpec[] = [
    { name: '‖ω_b‖', color: '--series-1' }, { name: '‖v_b‖', color: '--series-2' },
    { name: '對照 ω', color: '--series-1', dash: [5, 4], width: 1.4 }, { name: '對照 v', color: '--series-2', dash: [5, 4], width: 1.4 },
  ];
  const seriesS: SeriesSpec[] = [{ name: '‖Δθ‖', color: '--series-4', width: 4 }, { name: '對照', color: '--series-4', dash: [3, 2], width: 2.5 }];
  const PLOT_H = 150;
  const [pl, pr] = split(host);
  const plotsRow = pl.parentElement as HTMLElement;
  plotsRow.style.marginTop = '0.2rem';
  const plotE = new Plot2D(pl, { title: '誤差 log₁₀‖·‖', xLabel: '迭代 k', height: PLOT_H, yRange: [-13, 2.5], xRange: [0, 6], series: seriesE, legend: false });
  const plotS = new Plot2D(pr, { title: '每步 ‖Δθ‖ (rad)', xLabel: '迭代 k', height: PLOT_H, xRange: [0, 6], yRange: [0, 1.1], series: seriesS, legend: false });
  const [tl, tr] = split(host);
  const trackRow = tl.parentElement as HTMLElement;
  trackRow.style.display = 'none';
  const plotRate = new Plot2D(tl, { title: '‖dθ/dt‖ (rad/s)', xLabel: 't (s)', height: PLOT_H, xWindow: 12, yRange: [0, 1], series: [{ name: '‖dθ/dt‖', color: '--series-1' }], legend: false });
  const plotTE = new Plot2D(tr, { title: '追蹤誤差 log₁₀‖V_err‖', xLabel: 't (s)', height: PLOT_H, xWindow: 12, yRange: [-7, 1], series: [{ name: '誤差', color: '--series-2' }], legend: false });

  // Presets and transport first, then the live readouts, then the less frequently used settings,
  // so the first view (≈ 860 px) shows the canvas, both plots, the stepper and the numbers.
  const panelTop = controlPanel(host, { compact: true });
  const chipRow = row(panelTop);
  const runRow = row(panelTop);
  const stepper = algorithmStepper(runRow, { onStep: doStep, onReset: resetRun, intervalMs: 700, speeds: [0.5, 1, 2, 5] });
  const pb = playback(runRow, {
    onPlay: () => trackLoop.start(),
    onPause: () => trackLoop.stop(),
    onReset: () => resetTrack(),
    onSpeed: (s) => { trackLoop.speed = s; },
  });
  pb.el.style.display = 'none';

  const K = { V: 'V_b（誤差）', dth: 'Δθ（最近一步）', sig: 'σ_min(J_b)', reach: '目標', ns: '第 1 步 Δθ·n̂', j1: '關節 1 共轉了' } as const;
  const info = kv(host, Object.values(K));
  const KT = { rate: 'dθ/dt (rad/s)', sig: 'σ_min(J_b)', reach: '目標' } as const;
  const infoT = kv(host, Object.values(KT));
  infoT.el.style.display = 'none';

  const panel = controlPanel(host, { compact: true });
  const selMethod = select<Method>(panel, {
    label: '方法',
    value: opts.method,
    options: [
      { value: 'pinv', label: 'J⁺（Newton）' },
      { value: 'dls', label: 'DLS（阻尼）' },
      { value: 'transpose', label: 'Jᵀ（轉置法）' },
    ],
    onChange: (m) => setMethod(m, true),
  });
  const sLam = slider(panel, { label: 'λ（DLS 阻尼）', min: 0.01, max: 1, step: 0.01, value: opts.lambda, onInput: (v) => { opts.lambda = v; inputsChanged(); } });
  const sMax = slider(panel, {
    label: '最大步長', min: 0.1, max: STEP_INF, step: 0.05, value: STEP_INF,
    format: (v) => (v >= STEP_INF - 1e-9 ? '∞（不限制）' : `${fmt(v, 2)} rad`),
    onInput: (v) => { opts.maxStep = v >= STEP_INF - 1e-9 ? Infinity : v; inputsChanged(); },
  });
  const s0 = [0, 1, 2].map((i) => angleSlider(panel, {
    label: `初值 θ⁰${SUB[i + 1]}`, value: theta0[i],
    onInput: (v) => { theta0[i] = v; inputsChanged(); },
  }));
  const togRow = row(panel);
  const tPos = toggle(togRow, { label: '只要位置', title: '只追蹤末端位置 (x, y)：3R 變成冗餘手臂，J 是 2×3', value: false, onChange: (v) => setPositionOnly(v, true) });
  const tTwist = toggle(togRow, { label: '誤差 twist', value: show.twist, onChange: (v) => { show.twist = v; cv.requestRender(); } });
  const tIter = toggle(togRow, { label: '每步姿勢', value: show.iterates, onChange: (v) => { show.iterates = v; cv.requestRender(); } });
  const tCmp = toggle(togRow, {
    label: '對照', title: '以虛線疊上同一題的對照組：第 3 步換初值，加權／一般 J⁺ 互換，其他情況 J⁺ ↔ DLS',
    value: show.compare, onChange: (v) => { show.compare = v; computeCompare(); refreshRun(); },
  });
  const tNull = toggle(togRow, { label: '零空間', title: '畫出零空間自運動時各關節的速度', value: show.nullspace, onChange: (v) => { show.nullspace = v; cv.requestRender(); } });
  const tW = toggle(togRow, { label: '加權 J⁺', title: 'W = diag(10, 1, 1)：關節 1 的「成本」是其他關節的 10 倍', value: false, onChange: (v) => { opts.weights = v ? W_JOINT1 : null; inputsChanged(); } });

  // The iteration readout keeps one pair per row (its vectors would wrap in a narrow column); the
  // short tracking readout uses two pairs per row when there is room. The style change is deferred to
  // the next frame (and skipped when nothing changes) so it never re-triggers this observer.
  let kvCols = '';
  let kvRaf = 0;
  const applyCols = () => {
    const cols = host.clientWidth >= 520 ? 'auto 1fr auto 1fr' : 'auto 1fr';
    if (cols === kvCols) return;
    kvCols = cols;
    infoT.el.style.gridTemplateColumns = cols;
  };
  applyCols();
  const ro = new ResizeObserver(() => { cancelAnimationFrame(kvRaf); kvRaf = requestAnimationFrame(applyCols); });
  ro.observe(host);
  info.el.style.margin = infoT.el.style.margin = '0.1rem 0 0.35rem';

  // Five-bar preview card (last step)
  const card = el('div', {
    style: {
      display: 'none', flexWrap: 'wrap', gap: '0.4rem 0.8rem', alignItems: 'center', border: '1px solid var(--border)',
      borderRadius: 'var(--radius-sm)', padding: '0.3rem 0.7rem', margin: '0.5rem 0 0', background: 'var(--bg-sunken)', fontSize: '0.9rem',
    },
  });
  host.appendChild(card);
  const fbWrap = el('div', { style: { flex: '0 0 170px' } });
  const fbText = el('div', { style: { flex: '1 1 220px', minWidth: '0' } }, mdElement(FIVE_MD));
  card.append(fbWrap, fbText);
  const fb = new Canvas2D(fbWrap, { height: 100, world: { xmin: -1.15, xmax: 1.15, ymin: -0.2, ymax: 1.3 } });

  // ------------------------------------------------------------------ handles
  const knobPos = (): Pt => [task.x + KNOB * Math.cos(task.phi), task.y + KNOB * Math.sin(task.phi)];
  const targetHandle: Handle = { get: () => [task.x, task.y], set: (p) => { task.x = p[0]; task.y = p[1]; inputsChanged(); }, radius: 9, color: '--series-5' };
  const knobHandle: Handle = { get: knobPos, set: (p) => { task.phi = Math.atan2(p[1] - task.y, p[0] - task.x); inputsChanged(); }, radius: 6, color: '--series-5' };
  const guessPts = (): Pt[] => planarArmPoints(LINKS, theta0);
  const guessHandles: Handle[] = [0, 1, 2].map((i) => ({
    get: () => guessPts()[i + 1],
    set: (p: Pt) => {
      const pts = guessPts();
      const a = Math.atan2(p[1] - pts[i][1], p[0] - pts[i][0]);
      const before = theta0.slice(0, i).reduce((s, x) => s + x, 0);
      theta0[i] = wrapToPi(a - before);
      s0[i].set(theta0[i], { silent: true });
      inputsChanged();
    },
    radius: 6,
    color: '--fg-muted',
  }));
  cv.addHandle(targetHandle);
  cv.addHandle(knobHandle);
  guessHandles.forEach((h) => cv.addHandle(h));

  // ------------------------------------------------------------------ run logic
  function statusText(): string {
    const last = records[records.length - 1];
    if (status === 'converged') return `✓ 收斂：${records.length - 1} 步`;
    if (status === 'stalled') {
      // An oscillating run can stop on a bad iterate: report the best residual reached as well.
      const best = Math.min(...records.map((r) => r.omegaNorm + r.vNorm));
      const now = last.omegaNorm + last.vNorm;
      return now > 1.5 * best ? `■ 停滯：殘差最低到 ${fe(best)} 後不再下降（目前 ${fe(now)}）` : `■ 停滯：殘差卡在 ${fe(now)}，不再下降`;
    }
    if (status === 'maxIter') return `■ ${MAX_ITER} 步仍未收斂`;
    return '';
  }

  function refreshRun(): void {
    updatePlots();
    updateReadouts();
    cv.requestRender();
  }

  /** Stepper reset: back to k = 0 at θ⁰ (also recomputes the comparison run). */
  function resetRun(): void {
    anim = null;
    animLoop.stop();
    prevNull = null;
    records = [evaluate(theta0, task)];
    status = statusOf(records, LESSON_TOL, MAX_ITER);
    computeCompare();
    refreshRun();
    if (status !== 'running') stepper.finish(statusText());
  }

  /** Steps whose first view shows a finished run; manual edits there re-run the whole iteration live. */
  const autoComplete = () => step === 2 || step === 4 || step === 5;

  /** Any input changed: restart from θ⁰ (and finish the run right away where the curves are the point). */
  function inputsChanged(): void {
    if (mode === 'track') { updateReadouts(); cv.requestRender(); return; }
    chipSets[step]?.clear();
    stepper.reset();
    if (autoComplete()) runToEnd();
  }

  function doStep(): boolean {
    if (status !== 'running') return false;
    const last = records[records.length - 1];
    records.push(iterate(last, task, opts));
    status = statusOf(records, LESSON_TOL, MAX_ITER);
    if (!instant) {
      anim = { from: last.theta.slice(), d: last.dtheta!.slice(), t: 0, dur: Math.min(1.4, 0.25 + 0.12 * last.dthetaNorm) };
      animLoop.start();
      refreshRun();
    }
    if (status !== 'running') { stepper.setStatus(statusText()); return false; }
    return true;
  }

  /** Completes the current run immediately (through the stepper, so its iteration counter stays right). */
  function runToEnd(): void {
    instant = true;
    try {
      for (let k = 0; k <= MAX_ITER && status === 'running'; k++) stepper.step();
    } finally {
      instant = false;
    }
    anim = null;
    animLoop.stop();
    refreshRun();
  }

  function rerun(): void { stepper.reset(); runToEnd(); }
  function stepOnce(): void { stepper.step(); }

  const nearly = (a: Vec, b: Vec) => a.every((x, i) => Math.abs(wrapToPi(x - b[i])) < 1e-6);

  /** Comparison run for the current step: another initial guess, the other weighting, or J⁺ ↔ DLS. */
  function computeCompare(): void {
    if (!show.compare || mode !== 'iterate') { cmp = null; return; }
    const unreachable = !poseReachable(task).reachable;
    const kind: CmpKind = step === 2 ? 'guess' : !unreachable && task.positionOnly && opts.method === 'pinv' ? 'weights' : 'method';
    let q0 = theta0;
    let o: StepOptions = { ...opts };
    let label: string;
    if (kind === 'guess') {
      const useGood = nearly(theta0, GUESS.bad);
      q0 = useGood ? GUESS.good : GUESS.bad;
      label = useGood ? '好的初值' : '壞的初值';
    } else if (kind === 'weights') {
      o = { ...opts, weights: opts.weights ? null : W_JOINT1 };
      label = opts.weights ? '一般 J⁺' : '加權 J⁺';
    } else {
      const m: Method = opts.method === 'pinv' ? 'dls' : 'pinv';
      // For an unreachable target the comparison shows a DLS that is damped enough to settle.
      const lam = unreachable ? Math.max(opts.lambda, 0.5) : opts.lambda;
      o = { ...opts, method: m, lambda: lam, weights: null };
      label = m === 'pinv' ? 'J⁺' : `DLS λ=${fmt(lam, 2)}`;
    }
    cmp = { ...runNewton(q0, task, o, MAX_ITER), label, kind };
  }

  function syncControls(): void {
    const it = mode === 'iterate';
    targetHandle.enabled = it;
    knobHandle.enabled = it && !task.positionOnly;
    guessHandles.forEach((h) => { h.enabled = it; });
    tNull.el.style.display = it && task.positionOnly ? '' : 'none';
    tW.el.style.display = it && task.positionOnly && opts.method === 'pinv' ? '' : 'none';
    sLam.el.style.opacity = opts.method === 'dls' ? '' : '0.45';
    info.show(K.ns, task.positionOnly);
    info.show(K.j1, task.positionOnly);
  }

  function setMethod(m: Method, restart = false): void {
    opts.method = m;
    selMethod.set(m, { silent: true });
    syncControls();
    if (!restart) return;
    if (mode === 'track') startTrack();
    else inputsChanged();
  }

  function setPositionOnly(v: boolean, restart = false): void {
    task.positionOnly = v;
    tPos.set(v, { silent: true });
    if (!v) { opts.weights = null; tW.set(false, { silent: true }); }
    syncControls();
    if (restart) inputsChanged();
  }

  function setGuess(q: Vec): void {
    theta0 = q.slice();
    s0.forEach((s, i) => s.set(theta0[i], { silent: true }));
  }

  function setTask(t: Task): void {
    task = { ...t };
    syncControls();
  }

  function setMaxStep(v: number): void {
    sMax.set(v, { silent: true });
    opts.maxStep = v >= STEP_INF - 1e-9 ? Infinity : v;
  }
  function setLambda(v: number): void {
    sLam.set(v, { silent: true });
    opts.lambda = v;
  }
  function setWeighted(v: boolean): void {
    tW.set(v, { silent: true });
    opts.weights = v ? W_JOINT1 : null;
  }

  // ------------------------------------------------------------------ plots & readouts
  function updatePlots(): void {
    legendCmp.textContent = cmp ? `實線：目前設定　虛線：對照（${cmp.label}）` : '實線：目前設定';
    const n = records.length;
    const ks = records.map((_, i) => i);
    const pos = task.positionOnly;
    plotE.setXY(0, pos ? [] : ks, pos ? [] : records.map((r) => lg(r.omegaNorm)));
    plotE.setXY(1, ks, records.map((r) => lg(r.vNorm)));
    // Step norms are per-iteration quantities: draw them as stems (a lone 41 rad step must be visible).
    // The comparison stems sit slightly to the right of the main ones.
    const dx = cmp ? 0.14 : 0;
    const [sx, sy] = stems(records.slice(0, n - 1).map((r) => r.dthetaNorm), -dx);
    plotS.setXY(0, sx, sy);
    if (cmp) {
      const kc = cmp.records.map((_, i) => i);
      plotE.setXY(2, pos ? [] : kc, pos ? [] : cmp.records.map((r) => lg(r.omegaNorm)));
      plotE.setXY(3, kc, cmp.records.map((r) => lg(r.vNorm)));
      const [cx, cy] = stems(cmp.records.slice(0, -1).map((r) => r.dthetaNorm), dx);
      plotS.setXY(1, cx, cy);
    } else {
      plotE.setXY(2, [], []); plotE.setXY(3, [], []); plotS.setXY(1, [], []);
    }
    const xmax = Math.max(6, n - 1, cmp ? Math.min(cmp.records.length - 1, MAX_ITER) : 0);
    // A small margin keeps k = 0 off the frame.
    const xr: [number, number] = [-0.4, xmax + 0.4];
    plotE.setRanges(xr, undefined);
    let ymax = 1;
    for (const r of records) ymax = Math.max(ymax, r.dthetaNorm);
    if (cmp) for (const r of cmp.records) ymax = Math.max(ymax, r.dthetaNorm);
    plotS.setRanges(xr, [0, ymax * 1.1]);
    const last = records[n - 1];
    const mk: Marker[] = [{ y: lg(LESSON_TOL.ev), label: '門檻 10⁻⁶', dash: [2, 3] }];
    if (!pos) mk.push({ x: n - 1, y: lg(last.omegaNorm), color: '--series-1' });
    mk.push({ x: n - 1, y: lg(last.vNorm), color: '--series-2' });
    plotE.setMarkers(mk);
    const ms: Marker[] = [];
    if (Number.isFinite(opts.maxStep)) ms.push({ y: opts.maxStep, label: '最大步長', dash: [2, 3] });
    if (n > 1) ms.push({ x: n - 2 - dx, y: records[n - 2].dthetaNorm, color: '--series-4' });
    plotS.setMarkers(ms);
  }

  function updateReadouts(): void {
    if (mode === 'track') {
      const rn = norm(trk.rates);
      infoT.set(KT.rate, `(${trk.rates.map((x) => fmt(x, 2)).join(', ')})${rn > 5 ? ' ⚠ 過大' : ''}`);
      const Sv = singularValues(reducedJacobian(trk.theta, false));
      const S = Sv[Sv.length - 1];
      infoT.set(KT.sig, `${fmt(S, 4)}${S < 0.05 ? ' ⚠ 近奇異' : ''}`);
      infoT.set(KT.reach, trk.reachable ? '可達' : '✗ 超出工作空間：只能盡量靠近');
      return;
    }
    const n = records.length;
    const last = records[n - 1];
    const prev = n > 1 ? records[n - 2] : null;
    info.set(K.V, `${task.positionOnly ? '(v_x, v_y)' : '(ω_z, v_x, v_y)'} = (${last.V.map((x) => fmt(x, 2)).join(', ')})`);
    info.set(K.dth, prev?.dtheta
      ? `(${prev.dtheta.map((x) => fmt(x, 2)).join(', ')})，‖Δθ‖ = ${fmtNorm(prev.dthetaNorm)} rad${prev.clipped ? '（已截斷）' : ''}`
      : '—（尚未迭代）');
    // The step from θᵏ⁻¹ was computed with J_b(θᵏ⁻¹): its σ_min is what explains a huge Δθ.
    // Position-only tasks use the 2×3 linear rows J_v of J_b.
    const jn = task.positionOnly ? 'J_v' : 'J_b';
    info.setLabel(K.sig, prev ? `σ_min(${jn}) 上一步 → 現在` : `σ_min(${jn})`);
    const near = last.sigmaMin < 0.05 || (prev != null && prev.sigmaMin < 0.05);
    info.set(K.sig, `${prev ? `${fmt(prev.sigmaMin, 3)} → ` : ''}${fmt(last.sigmaMin, 3)}${near ? ' ⚠ 近奇異' : ''}`);
    const reach = poseReachable(task);
    info.set(K.reach, reach.reachable
      ? '可達'
      : task.positionOnly
        ? `不可達（|p| = ${fmt(reach.r, 2)} > ${fmt(REACH, 1)} m）`
        : `不可達（腕心距基座 ${fmt(reach.r, 2)} m，不在 [0.2, 1.8]）`);
    if (task.positionOnly) {
      // The first step from θ⁰ is the clearest test of "minimum norm": compare it with the null space there.
      const first = n > 1 ? records[0] : null;
      const nv = first ? nullSpaceDirection(reducedJacobian(first.theta, true)) : null;
      if (first?.dtheta && nv) {
        const along = dot(first.dtheta, nv);
        const frac = first.dthetaNorm > 1e-15 ? Math.abs(along) / first.dthetaNorm : 0;
        info.set(K.ns, `${fmt(along, 3)}（占 ‖Δθ‖ 的 ${fmt(100 * frac, 0)}%，${opts.method !== 'pinv' ? '非偽逆' : opts.weights ? '加權 W' : '最小範數'}）`);
      } else info.set(K.ns, first ? 'J 降秩，零空間不只一維' : '—（尚未迭代）');
      // Net rotation of joint 1 from θ⁰ (the joint the weighted pseudo-inverse makes "expensive").
      const turn = (rs: IterRecord[]) => `${fmt(Math.abs(wrapToPi(rs[rs.length - 1].theta[0] - rs[0].theta[0])) / D, 0)}°`;
      info.set(K.j1, `${turn(records)}${cmp ? `（對照・${cmp.label}：${turn(cmp.records)}）` : ''}`);
    }
  }

  // ------------------------------------------------------------------ animation of a step (shows overshoot)
  const animLoop = createLoop((dt) => {
    if (!anim) { animLoop.stop(); return; }
    anim.t += dt;
    if (anim.t >= anim.dur) { anim = null; animLoop.stop(); }
    cv.requestRender();
  });
  const animTheta = (): Vec | null => {
    if (!anim) return null;
    const u = Math.min(1, anim.t / anim.dur);
    const s = u * u * (3 - 2 * u);
    return anim.from.map((x, i) => x + anim!.d[i] * s);
  };

  // ------------------------------------------------------------------ tracking (last step)
  function recordTrack(): void {
    tipTrail.push(planarArmPoints(LINKS, trk.theta)[3]);
    if (tipTrail.length > 600) tipTrail.shift();
    const rn = norm(trk.rates);
    rateWin.push([trk.t, rn]);
    while (rateWin.length && rateWin[0][0] < trk.t - 12) rateWin.shift();
    let top = 1;
    for (const [, v] of rateWin) top = Math.max(top, v * 1.1);
    // Fixed 12 s time window (0–12 s at first, then scrolling): auto-ranging a single sample after a
    // reset would give a degenerate axis.
    const xr: [number, number] = [Math.max(0, trk.t - 12), Math.max(12, trk.t)];
    plotRate.setRanges(xr, [0, top]);
    plotTE.setRanges(xr, undefined);
    plotRate.push(trk.t, [rn]);
    plotTE.push(trk.t, [lg(trk.err)]);
    // A dot on the newest sample: a lone sample (right after ⟲ 重設) draws no line, so the plots
    // would otherwise look empty until ▶ is pressed.
    plotRate.setMarkers([{ x: trk.t, y: rn, color: '--series-1' }]);
    plotTE.setMarkers([{ x: trk.t, y: lg(trk.err), color: '--series-2' }]);
  }

  /** Back to t = 0 on the current path, paused. */
  function resetTrack(): void {
    trackLoop.stop();
    pb.setPlaying(false, { silent: true });
    trk = trackingStart(pathFn, GUESS.good);
    tipTrail = [];
    rateWin = [];
    plotRate.clear();
    plotTE.clear();
    recordTrack();
    pb.setTime(0);
    updateReadouts();
    cv.requestRender();
  }

  /** Restart the current path, simulate the first PREROLL seconds instantly, then keep playing. */
  function startTrack(): void {
    resetTrack();
    const dt = 1 / 30;
    for (let k = 0; k < Math.round(PREROLL / dt); k++) {
      advanceTracking(trk, pathFn, dt, K_TRACK, opts);
      recordTrack();
    }
    pb.setTime(trk.t);
    pb.setPlaying(true, { silent: true });
    trackLoop.start();
    updateReadouts();
    cv.requestRender();
  }

  const trackLoop = createLoop((dt) => {
    advanceTracking(trk, pathFn, dt, K_TRACK, opts);
    recordTrack();
    pb.setTime(trk.t);
    updateReadouts();
    cv.requestRender();
  });

  function setMode(m: 'iterate' | 'track'): void {
    if (m === mode) return;
    mode = m;
    const it = m === 'iterate';
    stepper.pause();
    stepper.el.style.display = it ? '' : 'none';
    pb.el.style.display = it ? 'none' : '';
    plotsRow.style.display = it ? '' : 'none';
    // Its flex layout is an inline style, so restore 'flex' explicitly ('' would fall back to block).
    legendRow.style.display = it ? 'flex' : 'none';
    trackRow.style.display = it ? 'none' : '';
    info.el.style.display = it ? '' : 'none';
    infoT.el.style.display = it ? 'none' : '';
    s0.forEach((s) => { s.el.style.display = it ? '' : 'none'; });
    sMax.el.style.display = it ? '' : 'none';
    togRow.style.display = it ? '' : 'none';
    card.style.display = it ? 'none' : 'flex';
    syncControls();
    if (it) {
      trackLoop.stop();
      pb.setPlaying(false, { silent: true });
      stepper.reset();
    } else {
      anim = null;
      animLoop.stop();
    }
  }

  // ------------------------------------------------------------------ drawing
  // Text labels are queued in a LabelLayer while the scene is drawn and placed last (labels.ts): they
  // stay on top of the links and avoid each other wherever the learner drags the target or the guess.
  const tag = (color: string) => ({ color, font: MONO, bg: '--sim-bg' });

  function drawBase(c: Canvas2D, L: LabelLayer): void {
    c.polygon([[-0.1, -0.11], [0.1, -0.11], [0, 0]], { fill: '--grid-strong', color: '--grid-strong', width: 1 });
    c.line([-0.16, -0.11], [0.16, -0.11], { color: '--fg-muted', width: 2 });
    c.frame({ x: 0, y: 0, theta: 0 }, 0.28, { width: 1.5, alpha: 0.8, labels: 's' });
    L.frame(0, 0, 0, 0.28, 800);
    // the frame's own x_s / y_s labels sit just beyond the arrow tips
    L.disc([0.28, 0], 16, 600);
    L.disc([0, 0.28], 16, 600);
  }

  /** Draws a 3R configuration; with a LabelLayer and weight > 0 its links become label obstacles. */
  function drawArm(c: Canvas2D, q: Vec, color: string, style: 'solid' | 'ghost' | 'dashed', alpha = 1, L?: LabelLayer, weight = 0): Pt[] {
    const pts = planarArmPoints(LINKS, q);
    if (style === 'solid') {
      for (let i = 0; i < 3; i++) c.line(pts[i], pts[i + 1], { color, width: 7, alpha: 0.85 * alpha });
      for (let i = 0; i < 3; i++) {
        c.point(pts[i], 5, { color: '--bg-elev', alpha });
        c.circle(pts[i], 5 / c.scale, { color: '--fg', width: 1.6, alpha });
      }
    } else if (style === 'dashed') {
      for (let i = 0; i < 3; i++) c.line(pts[i], pts[i + 1], { color, width: 2, dash: [6, 4], alpha });
    } else {
      for (let i = 0; i < 3; i++) c.line(pts[i], pts[i + 1], { color, width: 4, alpha: 0.28 * alpha });
    }
    if (L && weight > 0) {
      L.polyline(pts, weight);
      if (style === 'solid') for (let i = 0; i < 3; i++) L.disc(pts[i], 7, weight);
    }
    return pts;
  }

  /** End-effector frame {b}; its label prefers the frame's −y side (away from the last link and the x arrow). */
  function tipFrame(c: Canvas2D, q: Vec, label: string | null, L: LabelLayer, alpha = 1): void {
    const phi = q[0] + q[1] + q[2];
    const tip = planarArmPoints(LINKS, q)[3];
    c.frame({ x: tip[0], y: tip[1], theta: phi }, 0.3, { width: 2, alpha });
    L.frame(tip[0], tip[1], phi, 0.3);
    if (label) L.label(tip, label, { ...tag('--fg'), prefer: phi - Math.PI / 2, priority: 1 });
  }

  function drawTwist(c: Canvas2D, q: Vec, L: LabelLayer): void {
    const r = residual(q, task);
    const tip: Pt = [r.Tsb[0][3], r.Tsb[1][3]];
    if (task.positionOnly) {
      if (r.vNorm > 1e-3) {
        c.arrow(tip, [task.x, task.y], { color: '--series-6', width: 2, dash: [5, 4], head: 9 });
        L.seg(tip, [task.x, task.y], 600);
      }
      return;
    }
    if (r.omegaNorm + r.vNorm < 2e-3) return;
    const arc = screwArc(r.Tsb, r.V6, 32);
    const pts = arc.map((T) => [T[0][3], T[1][3]] as Pt);
    c.polyline(pts, { color: '--series-6', width: 2.2, dash: [6, 4] });
    L.polyline(pts, 600);
    for (const k of [11, 21]) {
      const T = arc[k];
      const th = Math.atan2(T[1][0], T[0][0]);
      c.frame({ x: T[0][3], y: T[1][3], theta: th }, 0.14, { width: 1.3, alpha: 0.55 });
      L.frame(T[0][3], T[1][3], th, 0.14, 300);
    }
    // V_b beside the middle of the path, preferably on the side that faces away from the base
    const m = pts[16];
    let side = Math.atan2(pts[32][1] - pts[0][1], pts[32][0] - pts[0][0]) - Math.PI / 2;
    if (Math.cos(side) * m[0] + Math.sin(side) * m[1] < 0) side += Math.PI;
    L.label(m, 'V_b', { ...tag('--series-6'), prefer: side, radii: [8, 16, 28], priority: 2 });
  }

  function drawNull(c: Canvas2D, q: Vec, L: LabelLayer): void {
    const J = reducedJacobian(q, true);
    const n = nullSpaceDirection(J, prevNull);
    if (!n) return;
    prevNull = n;
    const eps = 1e-4;
    const p0 = planarArmPoints(LINKS, q);
    const p1 = planarArmPoints(LINKS, q.map((x, i) => x + eps * n[i]));
    const vel: Pt[] = [1, 2, 3].map((i) => [(p1[i][0] - p0[i][0]) / eps, (p1[i][1] - p0[i][1]) / eps] as Pt);
    // Longest arrow drawn 0.45 m long; the tip's velocity is zero (that is what "null space" means).
    const speeds = vel.map((v) => Math.hypot(v[0], v[1]));
    const scale = 0.45 / Math.max(1e-9, ...speeds);
    vel.forEach((v, k) => {
      const p = p0[k + 1];
      if (speeds[k] * scale > 0.03) {
        const e: Pt = [p[0] + v[0] * scale, p[1] + v[1] * scale];
        c.arrow(p, e, { color: '--series-7', width: 2.6, head: 10 });
        L.seg(p, e, 1200);
      } else {
        c.circle(p, 8 / c.scale, { color: '--series-7', width: 2 });
        L.disc(p, 9, 1200);
      }
    });
    // Key in the (empty) bottom-right corner rather than next to the arrows.
    const v = c.visible();
    L.fixedLabel([v.xmax, v.ymin], '→ 自運動 n̂：末端不動時各關節點的速度', { color: '--series-7', align: 'right', dx: -8, dy: -13, bg: '--sim-bg', font: '12px sans-serif' }, -1);
  }

  /** Target {d} (or the position target p_d) with its label; one shared label when {b} sits on it. */
  function drawTarget(c: Canvas2D, q: Vec, L: LabelLayer): void {
    const tip = planarArmPoints(LINKS, q)[3];
    const onTarget = Math.hypot(tip[0] - task.x, tip[1] - task.y) < 0.12;
    const P: Pt = [task.x, task.y];
    L.disc(P, 13, 2000); // the target handle, drawn by the canvas after this callback
    if (task.positionOnly) {
      const k = 0.08;
      c.line([task.x - k, task.y], [task.x + k, task.y], { color: '--series-5', width: 2 });
      c.line([task.x, task.y - k], [task.x, task.y + k], { color: '--series-5', width: 2 });
      if (!onTarget) L.label(P, 'p_d', { ...tag('--series-5'), prefer: -Math.PI / 4, priority: 0 });
      return;
    }
    c.frame({ x: task.x, y: task.y, theta: task.phi }, 0.35, { width: 2.5 });
    c.line(P, knobPos(), { color: '--series-5', width: 1.5, dash: [4, 3] });
    L.frame(task.x, task.y, task.phi, 0.35);
    L.seg(P, knobPos(), 800);
    L.disc(knobPos(), 10, 2000);
    if (onTarget) L.label(P, '{b} = {d}', { ...tag('--series-5'), prefer: task.phi - Math.PI / 2, priority: 0 });
    else L.label(P, '{d}', { ...tag('--series-5'), prefer: task.phi + Math.PI, priority: 0 });
  }

  function drawTrack(c: Canvas2D, L: LabelLayer): void {
    const P = TRACK_PATHS[trackIdx];
    const circ: Pt[] = [];
    for (let k = 0; k <= 72; k++) { const a = (k / 72) * 2 * Math.PI; circ.push([P.cx + P.rho * Math.cos(a), P.cy + P.rho * Math.sin(a)]); }
    c.polyline(circ, { color: '--series-5', width: 1.2, dash: [4, 4] });
    L.polyline(circ, 300);
    c.polyline(tipTrail, { color: '--series-7', width: 2, alpha: 0.8 });
    const tg = pathFn(trk.t);
    c.frame({ x: tg.x, y: tg.y, theta: tg.phi }, 0.35, { width: 2.5 });
    c.point([tg.x, tg.y], 6, { color: trk.reachable ? '--series-5' : '--danger' });
    L.frame(tg.x, tg.y, tg.phi, 0.35);
    L.disc([tg.x, tg.y], 8, 1500);
    if (!trk.reachable) L.label([tg.x, tg.y], '到不了！', { color: '--danger', bg: '--sim-bg', prefer: Math.PI / 2, priority: 0 });
    drawArm(c, trk.theta, '--series-1', 'solid', 1, L, 3000);
    tipFrame(c, trk.theta, '{b}', L);
    c.setOverlayText(`t = ${fmt(trk.t, 1)} s   ‖dθ/dt‖ = ${fmt(norm(trk.rates), 2)} rad/s   誤差 = ${fe(trk.err)}${trk.reachable ? '' : '   ⚠ 目標在工作空間外'}`);
  }

  cv.draw = (c) => {
    const L = new LabelLayer(c, `12px ${cssVar('--font') || 'sans-serif'}`);
    c.grid(0.5);
    c.circle([0, 0], REACH, { color: '--series-3', width: 1.2, dash: [6, 4] });
    drawBase(c, L);
    if (mode === 'track') { drawTrack(c, L); L.flush(); return; }

    const n = records.length;
    const a = animTheta();
    const q = a ?? records[n - 1].theta;
    drawTarget(c, q, L);
    // initial guess (dashed) with its drag handles; labelled once the arm has left it
    const g = drawArm(c, theta0, '--fg-muted', 'dashed', 0.9, L, 800);
    for (let i = 1; i <= 3; i++) L.disc(g[i], 10, 1500);
    if (n > 1 || a) L.label(g[1], 'θ⁰', { color: '--fg-muted', bg: '--sim-bg', prefer: (3 * Math.PI) / 4, priority: 3 });
    // where the comparison run ended
    if (cmp) {
      drawArm(c, cmp.records[cmp.records.length - 1].theta, '--series-2', 'dashed', 1, L, 800);
      // Key in the bottom-left corner: the top row belongs to the overlay readout.
      const v = c.visible();
      L.fixedLabel([v.xmin, v.ymin], `- - 對照終點（${cmp.label}）`, { color: '--series-2', align: 'left', dx: 8, dy: -13, bg: '--sim-bg', font: '12px sans-serif' }, -1);
    }
    if (show.iterates) {
      const from = Math.max(1, n - 7);
      for (let k = from; k < n - (a ? 0 : 1); k++) drawArm(c, records[k].theta, '--series-1', 'ghost', 0.35 + 0.65 * ((k - from + 1) / (n - from)), L, 150);
    }
    // tip path through the iterates; numbers on the first few (points that pile up near the target share one)
    const tips = records.map((r) => planarArmPoints(LINKS, r.theta)[3]);
    const path = a ? tips.slice(0, -1) : tips;
    if (path.length > 1) {
      c.polyline(path, { color: '--series-4', width: 1.5, alpha: 0.9 });
      L.polyline(path, 300);
    }
    const numbered: Pt[] = [];
    tips.forEach((p, k) => {
      if (a && k === n - 1) return;
      c.point(p, 3.5, { color: '--series-4' });
      if (k < 1 || k > 5) return;
      const s = c.toScreen(p);
      if (numbered.some((o) => Math.hypot(o[0] - s[0], o[1] - s[1]) < 14)) return;
      numbered.push(s);
      L.label(p, String(k), { color: '--series-4', font: '11px sans-serif', prefer: Math.PI / 4, radii: [3, 8, 14], priority: 6 });
    });
    drawArm(c, q, '--series-1', 'solid', 1, L, 3000);
    const tip = planarArmPoints(LINKS, q)[3];
    const onTarget = !task.positionOnly && Math.hypot(tip[0] - task.x, tip[1] - task.y) < 0.12;
    tipFrame(c, q, a || onTarget ? null : '{b}', L);
    if (!a && show.twist) drawTwist(c, q, L);
    if (!a && task.positionOnly && show.nullspace) drawNull(c, q, L);
    L.flush();
    const last = records[n - 1];
    // Short status here; the stepper next to the buttons shows the full message.
    const st = status === 'converged' ? `✓ 收斂：${n - 1} 步` : status === 'stalled' ? '■ 停滯' : status === 'maxIter' ? '■ 未收斂' : '';
    c.setOverlayText(`k = ${n - 1}   ${task.positionOnly ? '' : `‖ω_b‖ = ${fe(last.omegaNorm)}   `}‖v_b‖ = ${fe(last.vNorm)}   ${st}`);
  };

  fb.draw = (c) => {
    const A: Pt = [-0.45, 0], B: Pt = [0.45, 0];
    const crank = 0.55, coupler = 0.85;
    const C1: Pt = [A[0] + crank * Math.cos(115 * D), A[1] + crank * Math.sin(115 * D)];
    const C2: Pt = [B[0] + crank * Math.cos(70 * D), B[1] + crank * Math.sin(70 * D)];
    const dx = C2[0] - C1[0], dy = C2[1] - C1[1];
    const d = Math.hypot(dx, dy);
    const h = Math.sqrt(Math.max(0, coupler * coupler - (d / 2) * (d / 2)));
    const mid: Pt = [(C1[0] + C2[0]) / 2, (C1[1] + C2[1]) / 2];
    const P: Pt = [mid[0] - (dy / d) * h, mid[1] + (dx / d) * h];
    const P2: Pt = [mid[0] + (dy / d) * h, mid[1] - (dx / d) * h];
    c.line([-0.7, 0], [0.7, 0], { color: '--fg-muted', width: 2 });
    c.line(C1, P2, { color: '--fg-muted', width: 1.5, dash: [4, 3] });
    c.line(C2, P2, { color: '--fg-muted', width: 1.5, dash: [4, 3] });
    c.circle(P2, 4 / c.scale, { color: '--fg-muted', width: 1.5 });
    for (const [a, b] of [[A, C1], [B, C2]] as [Pt, Pt][]) c.line(a, b, { color: '--series-1', width: 5 });
    c.line(C1, P, { color: '--fg', width: 3 });
    c.line(C2, P, { color: '--fg', width: 3 });
    for (const p of [A, B, C1, C2]) { c.point(p, 3.5, { color: '--bg-elev' }); c.circle(p, 3.5 / c.scale, { color: '--fg', width: 1.5 }); }
    c.point(P, 5, { color: '--series-5' });
    c.text(P, 'P', { color: '--series-5', dx: 10, dy: -4 });
    c.text(A, '馬達 1', { color: '--fg-muted', dy: 11, font: '10px sans-serif' });
    c.text(B, '馬達 2', { color: '--fg-muted', dy: 11, font: '10px sans-serif' });
  };

  // ------------------------------------------------------------------ presets per step
  function resetBoth(q: Vec): void { setTask({ ...DEFAULT_TASK, positionOnly: task.positionOnly }); setGuess(q); }
  const chipSets: PresetChipsHandle[] = [
    presetChips(chipRow, [
      { label: '重設目標與初值', apply: () => { resetBoth(GUESS.start); stepper.reset(); } },
    ]),
    presetChips(chipRow, [
      { label: '重設並走一步', apply: () => { resetBoth(GUESS.start); stepper.reset(); stepOnce(); } },
    ]),
    presetChips(chipRow, [
      { label: '好的初值 (0°, 90°, 0°)', title: '目標重設為 (1.0, 1.2, 60°)', apply: () => { resetBoth(GUESS.good); rerun(); } },
      { label: '壞的初值 (−150°, 60°, 90°)', title: '目標重設為 (1.0, 1.2, 60°)', apply: () => { resetBoth(GUESS.bad); rerun(); } },
    ], { label: '初值：' }),
    presetChips(chipRow, [
      { label: '近奇異初值 (40°, 3°, −20°)', title: '重設並只走一步', apply: () => { resetBoth(GUESS.sing); setMaxStep(STEP_INF); stepper.reset(); stepOnce(); } },
      { label: '最大步長 = 1 rad', title: '同一初值，每步 ‖Δθ‖ ≤ 1 rad，跑到底', apply: () => { setMaxStep(1); rerun(); } },
      { label: '不限制步長', title: '同一初值，跑到底', apply: () => { setMaxStep(STEP_INF); rerun(); } },
    ]),
    presetChips(chipRow, [
      { label: 'λ = 0.05', apply: () => { setLambda(0.05); rerun(); } },
      { label: 'λ = 0.1', apply: () => { setLambda(0.1); rerun(); } },
      { label: 'λ = 0.5', apply: () => { setLambda(0.5); rerun(); } },
      { label: '好的初值', apply: () => { resetBoth(GUESS.good); rerun(); } },
      { label: '近奇異初值', apply: () => { resetBoth(GUESS.sing); rerun(); } },
    ]),
    presetChips(chipRow, [
      // Weights only apply to the pseudo-inverse method, so these presets switch back to J⁺.
      { label: '冗餘初值 (45°, 75°, 60°)', apply: () => { resetBoth(GUESS.redund); setWeighted(false); setMethod('pinv'); rerun(); } },
      { label: '加權 J⁺', apply: () => { setWeighted(true); setMethod('pinv'); rerun(); } },
      { label: '一般 J⁺', apply: () => { setWeighted(false); setMethod('pinv'); rerun(); } },
      // Plain J⁺ (solid) vs a well-damped DLS (dashed), as quiz question 1 describes.
      { label: '不可達目標 (2.6, 0.6)', title: '|p| ≈ 2.67 m > 2.4 m：殘差不可能歸零（實線為 J⁺，虛線為 λ ≥ 0.5 的 DLS）', apply: () => { setTask({ ...FAR_TASK }); setGuess(GUESS.far); setWeighted(false); setMethod('pinv'); rerun(); } },
    ]),
    presetChips(chipRow, TRACK_LABELS.map((P, i) => ({ label: P.label, title: P.title, apply: () => { trackIdx = i; startTrack(); } })), { label: '路徑：' }),
  ];

  // ------------------------------------------------------------------ per-step configuration
  let focused: HTMLElement[] = [];
  function setFocus(els: HTMLElement[]): void {
    focused.forEach((f) => f.classList.remove('mr-focus'));
    focused = els;
    focused.forEach((f) => f.classList.add('mr-focus'));
  }
  const setToggle = (t: { set(v: boolean, o?: { silent?: boolean }): void }, key: keyof typeof show, v: boolean) => { show[key] = v; t.set(v, { silent: true }); };

  function onStep(i: number): void {
    if (i === step) return;
    const prev = step;
    step = i;
    chipSets.forEach((cs, k) => { cs.el.style.display = k === i ? '' : 'none'; cs.clear(); });
    if (i === 6) {
      if (task.positionOnly) setPositionOnly(false);
      // The hint compares J⁺ with a lightly damped DLS: do not inherit step 5's λ = 0.5 preset, which
      // makes DLS lag ~0.1 behind the target even where it is reachable.
      setLambda(0.1);
      setMode('track');
      chipSets[6].activate(trackIdx);
      setFocus([trackRow, selMethod.el]);
      return;
    }
    const fromLater = prev >= 2 || mode === 'track';
    setMode('iterate');
    stepper.pause();
    setToggle(tCmp, 'compare', i === 2 || i === 4 || i === 5);
    setToggle(tNull, 'nullspace', i === 5);
    setToggle(tTwist, 'twist', i <= 1 || i === 5);
    setToggle(tIter, 'iterates', true);
    setPositionOnly(i === 5);
    setWeighted(false);
    setMethod(i === 4 ? 'dls' : 'pinv');
    if (i === 4) setLambda(0.1);
    if (i >= 3) setMaxStep(STEP_INF);
    // Coming back to the first two steps from a later preset: restore their scene. Moving between
    // steps 1 and 2 keeps whatever the learner set up.
    if (i <= 1 && fromLater) resetBoth(GUESS.start);
    switch (i) {
      case 0: stepper.reset(); break;
      case 1: stepper.reset(); stepOnce(); break;
      case 2: chipSets[2].activate(0); break;
      case 3: chipSets[3].activate(0); break;
      case 4: resetBoth(GUESS.sing); chipSets[4].highlight(1); rerun(); break;
      case 5: chipSets[5].activate(0); break;
    }
    const focusMap: HTMLElement[][] = [
      [cv.el, info.el],
      [stepper.el, info.el],
      [plotE.el],
      [plotS.el, sMax.el],
      [sLam.el, plotE.el],
      [cv.el, tW.el],
    ];
    setFocus(focusMap[i] ?? []);
  }

  syncControls();
  onStep(ctx.currentStep);

  return {
    onStep,
    destroy() {
      stepper.destroy();
      animLoop.stop();
      trackLoop.stop();
      ro.disconnect();
      cancelAnimationFrame(kvRaf);
      cv.destroy();
      fb.destroy();
      plotE.destroy();
      plotS.destroy();
      plotRate.destroy();
      plotTE.destroy();
    },
  };
}

const lesson: Lesson = { meta, steps, quiz, mount };
export default lesson;
