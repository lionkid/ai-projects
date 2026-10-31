import type { Lesson, LessonContext, LessonHandle, LessonStep, QuizQuestion } from '@/types';
import {
  el, cssVar, fmt, controlPanel, row, slider, angleSlider, select, toggle, simTitle, kv, mdElement, split,
  Canvas2D, Plot2D, createLoop, substep, presetChips, algorithmStepper, playback, planarArmPoints,
  type Pt, type Handle, type PresetChipsHandle, type SeriesSpec,
} from '@/ui';
import { type Vec, dot, norm, wrapToPi } from '@/mr';
import { meta } from './meta';
import {
  LESSON_TOL, LINKS, REACH, evaluate, iterate, nullSpaceDirection, poseReachable, reducedJacobian, residual,
  runNewton, screwArc, singularValues, statusOf, trackingRates, type IterRecord, type Method, type MovingTarget, type RunStatus,
  type StepOptions, type Task,
} from './newton';

const D = Math.PI / 180;
const MAX_ITER = 60;
const KNOB = 0.45;
/** Slider value of "最大步長" that means "no cap". */
const STEP_INF = 3.2;
const K_TRACK = 5;
const DEFAULT_TASK: Task = { x: 1.0, y: 1.2, phi: 60 * D, positionOnly: false };
const deg = (a: number[]): Vec => a.map((x) => x * D);
const GUESS = {
  start: deg([-40, 110, 40]),
  good: deg([0, 90, 0]),
  bad: deg([-150, 60, 90]),
  sing: deg([40, 3, -20]),
  redund: deg([10, 60, 60]),
};

interface TrackPath { label: string; title: string; cx: number; cy: number; rho: number; phi0: number; wiggle: number; omega: number }
const TRACKS: TrackPath[] = [
  { label: '圓形路徑（遠離奇異）', title: '圓心 (1.1, 0.9)、半徑 0.5 m，朝向在 30° ± 17° 間擺動', cx: 1.1, cy: 0.9, rho: 0.5, phi0: 30 * D, wiggle: 0.3, omega: 0.6 },
  { label: '擦過奇異（腕心靠近內圓）', title: '圓心 (1.2, 0.6)、半徑 0.7 m：途中腕心離基座只剩約 0.24 m，θ₂ 接近 180°', cx: 1.2, cy: 0.6, rho: 0.7, phi0: 20 * D, wiggle: 0.3, omega: 0.6 },
];

const pathAt = (P: TrackPath, t: number): MovingTarget => {
  const a = P.omega * t;
  return {
    x: P.cx + P.rho * Math.cos(a), y: P.cy + P.rho * Math.sin(a), phi: P.phi0 + P.wiggle * Math.sin(a),
    dx: -P.rho * P.omega * Math.sin(a), dy: P.rho * P.omega * Math.cos(a), dphi: P.wiggle * P.omega * Math.cos(a),
  };
};

const steps: LessonStep[] = [
  {
    title: '把 IK 變成求根',
    md: `沒有封閉解時，就把 IK 改寫成**求根（root finding）**：找 $\\theta$，讓目前的末端位姿 $T_{sb}(\\theta)$ 與目標 $T_{sd}$ 之間的「差」等於零。

位姿不能直接相減——$T_{sd} - T_{sb}$ 不是任何物理量。正確的問法是：在 $\\{b\\}$ 裡要施加哪個 twist、持續一秒，才能把 $\\{b\\}$ 帶到 $\\{d\\}$？答案就是矩陣對數

$$[\\mathcal V_b] = \\log\\big(T_{sb}^{-1}(\\theta)\\,T_{sd}\\big).$$

IK 的目標是讓 $\\mathcal V_b = 0$；實作上要求 $\\|\\omega_b\\|$ 與 $\\|v_b\\|$ 都小於容許誤差。`,
    simHint: '青色虛線是 V_b 的螺旋路徑：沿著它走一秒，{b} 正好落在目標 {d}。拖曳紫色目標點與朝向旋鈕，看 ‖ω_b‖、‖v_b‖ 怎麼變。',
  },
  {
    title: '一步 Newton',
    md: `$\\mathcal V_b$ 對 $\\theta$ 是非線性的，但在目前的 $\\theta^i$ 附近可以用 body Jacobian 線性化：關節改變 $\\Delta\\theta$，末端大約產生 twist $J_b(\\theta^i)\\Delta\\theta$。令它等於 $\\mathcal V_b$，解出 $\\Delta\\theta$：

$$\\theta^{i+1} = \\theta^i + J_b^{\\dagger}(\\theta^i)\\,\\mathcal V_b .$$

$J_b^{\\dagger}$ 是偽逆（pseudoinverse）；平面 3R 追蹤位置加朝向時 $J_b$ 是 3×3，可逆時 $J_b^{\\dagger} = J_b^{-1}$。線性近似不完美，一步通常不會正好到位，但會更接近；在新的位置重新線性化，再走一步。`,
    simHint: '按「▸ 單步」：灰色虛影是上一個姿勢，讀數顯示這一步的 Δθ 與新的殘差。多按幾次，直到出現「✓ 收斂」。',
  },
  {
    title: '收斂曲線與初值',
    md: `左下圖用對數縱軸畫每次迭代的誤差。初值夠近時，誤差會**加速**下降，每一步的有效位數大約加倍（$10^{-2} \\to 10^{-4} \\to 10^{-8}$），這就是 Newton 法的**二次收斂**，通常 3–5 步就過門檻。（平面手臂的朝向 $\\theta_1 + \\theta_2 + \\theta_3$ 對 $\\theta$ 是線性的，所以 $\\|\\omega_b\\|$ 一步就歸零；請看 $\\|v_b\\|$ 那條。）

初值離得遠時線性化很不準，前幾步可能亂跳，最後可能收斂到**另一組解**（手肘翻到另一側），也可能停滯。數值 IK 一次只給一組解，給哪一組由初值決定；實務上常拿上一時刻的關節角當初值。`,
    simHint: '按「好的初值」再按 ▶ 自動，看誤差曲線的陡降；換「壞的初值」重跑，注意它收斂到手肘朝另一側的解。也可以拖曳灰色 θ⁰ 手臂的關節。',
  },
  {
    title: '奇異附近會發生什麼',
    md: `用奇異值分解看偽逆：$J = \\sum_i \\sigma_i u_i v_i^{\\T}$，$J^{\\dagger}$ 把誤差在 $u_i$ 方向的分量除以 $\\sigma_i$。手臂接近奇異時（例如 $\\theta_2 \\approx 0$，前兩節幾乎伸直），最小的 $\\sigma$ 趨近 0，那個方向的誤差被放大成巨大的 $\\Delta\\theta$：手臂一步轉好幾圈，線性近似完全失效。

常見補救有兩種：限制每步的 $\\|\\Delta\\theta\\|$（最大步長），或改用下一步的阻尼最小平方。`,
    simHint: '「近奇異初值」θ₂ = 3° 已載入：按一次「▸ 單步」，看手臂狂轉、右下圖 ‖Δθ‖ 衝到約 41 rad；再按「最大步長 = 1 rad」重跑比較。',
  },
  {
    title: '阻尼最小平方',
    md: `阻尼最小平方（damped least squares, DLS）改用

$$\\theta^{i+1} = \\theta^i + J_b^{\\T}\\big(J_bJ_b^{\\T} + \\lambda^2 I\\big)^{-1}\\mathcal V_b .$$

在每個奇異值方向上，它把 $1/\\sigma$ 換成 $\\sigma/(\\sigma^2 + \\lambda^2)$：$\\sigma \\gg \\lambda$ 時幾乎與偽逆相同；$\\sigma \\to 0$ 時增益最多 $1/(2\\lambda)$，步長有界。代價是每一步都略偏離精確的 Newton 步，收斂從二次退化成線性——$\\lambda$ 越大越穩，也越慢。`,
    simHint: '方法已切到 DLS，虛線是同一初值下 J⁺ 的結果。按 ▶ 自動；再按「λ = 0.5」與「好的初值」重跑，比較收斂需要的步數。',
  },
  {
    title: '冗餘與最小範數解',
    md: `只要求末端**位置**時，3R 多出一個自由度：$J_v$ 是 2×3，$J_v\\Delta\\theta = v$ 有無限多解，彼此只差一個**零空間（null space）**分量——末端不動、只改變手臂姿勢的自運動 $\\hat n$。偽逆挑的是其中 $\\|\\Delta\\theta\\|$ 最小的一個，它與零空間正交：$\\Delta\\theta \\cdot \\hat n = 0$。

若某個關節比較「貴」（慢、耗能），可改用加權偽逆，最小化 $\\Delta\\theta^{\\T}W\\Delta\\theta$。這時步長帶有零空間分量，收斂到的姿勢也跟著改變。`,
    simHint: '已切到「只要位置」：粉紅箭頭是零空間自運動時各關節的速度（末端那支為零）。看讀數 Δθ·n̂ ≈ 0；再按「加權 J⁺」重跑，比較最後的姿勢。',
  },
  {
    title: '逆速度運動學與下一章',
    md: `把 Newton 步除以時間，就是**逆速度運動學**：想讓末端以 twist $\\mathcal V$ 運動，就令

$$\\dot\\theta = J^{\\dagger}(\\theta)\\,\\mathcal V .$$

追蹤移動目標時，每個控制週期把「目標本身的 twist」加上「剩餘誤差 × 增益」當成 $\\mathcal V$，等於連續版的 Newton 迭代。路徑擦過奇異構型時，$J^{\\dagger}$ 同樣會要求極大的關節速度；DLS 則犧牲一點追蹤誤差，換來有界的 $\\dot\\theta$。

下一章轉向閉鏈機構——在那裡，連正向運動學都得解方程式。`,
    simHint: '按 ▶ 播放：目標沿圓移動，手臂以 θ̇ = J⁺V 跟隨。換「擦過奇異」路徑，比較 J⁺ 與 DLS 的 ‖θ̇‖ 尖峰與追蹤誤差，最後做下方測驗。',
  },
];

const quiz: QuizQuestion[] = [
  {
    question: '只要位置的 3R 手臂（總長 2.4 m），目標在 $|p| = 2.67$ m 處，超出可達範圍。迭代會怎樣？',
    options: [
      '只要迭代次數夠多，最後一定收斂',
      '殘差降不到 0。最接近的姿勢是伸直指向目標，偏偏那裡 $J$ 降秩，$J^{\\dagger}$ 在附近亂跳；阻尼夠的 DLS 會停在伸直姿勢、殘差卡在約 0.27 m，所以要偵測「殘差不再下降」',
      '關節角會發散到無限大',
      '會收斂到可達範圍內離目標最遠的姿勢',
    ],
    answer: 1,
    explanation: '$f(\\theta) = 0$ 根本沒有解，能做的只是讓殘差最小：手臂伸直、指向目標，殘差 $|p| - 2.4 \\approx 0.27$ m。這個姿勢正是徑向的奇異構型，$\\sigma_{\\min} \\to 0$，偽逆步長 $\\propto 1/\\sigma_{\\min}$ 在附近暴衝、來回振盪；阻尼足夠時 DLS 會穩定停在那裡。角度會包裹，不會跑到無限大；正確做法是偵測停滯並回報「不可達」。',
  },
  {
    question: '冗餘手臂（$J$ 是 2×3）用 $\\Delta\\theta = J^{\\dagger}v$。這一步有什麼幾何意義？改用 $W = \\diag(10, 1, 1)$ 的加權偽逆又如何？',
    options: [
      '$J^{\\dagger}v$ 讓三個關節轉一樣多；加權後只有關節 1 會動',
      '加權偽逆會讓末端偏離 $v$ 的方向',
      '$J^{\\dagger}v$ 是所有滿足 $J\\Delta\\theta = v$ 的步長中 $\\|\\Delta\\theta\\|$ 最小者，沒有零空間分量；加權後改為最小化 $\\Delta\\theta^{\\T}W\\Delta\\theta$，關節 1 動得較少、其他關節補上，步長帶有零空間分量',
      '$J^{\\dagger}$ 只對方陣有定義，冗餘時不能用',
    ],
    answer: 2,
    explanation: '$J\\Delta\\theta = v$ 的解是一條直線 $\\Delta\\theta_p + s\\,\\hat n$。最小範數解是這條線上離原點最近的點，必然與 $\\hat n$ 正交。加權版本仍然精確滿足 $J\\Delta\\theta = v$（末端方向不變），只是改用 $W$ 量「長度」：關節 1 的權重是 10，最省的作法是讓它少動，於是步長在 $\\hat n$ 上有了分量，$\\|\\Delta\\theta\\|$ 反而變大。',
  },
  {
    question: '為什麼逆速度運動學 $\\dot\\theta = J^{\\dagger}\\mathcal V$ 與 Newton 步 $\\Delta\\theta = J_b^{\\dagger}\\mathcal V_b$ 長得一模一樣？',
    options: [
      '巧合，兩者推導無關',
      '因為 $J^{\\dagger} = J^{\\T}$',
      '因為誤差 $\\mathcal V_b$ 永遠是 0',
      '兩者解的是同一個線性化關係「關節變化 → 末端 twist」。$\\mathcal V_b$ 是「一秒內走完全部誤差」的 twist，把它當速度就是 Newton 步；每個瞬間給定想要的 twist、換成 $\\dot\\theta$，就是逆速度運動學',
    ],
    answer: 3,
    explanation: '$\\mathcal V = J(\\theta)\\dot\\theta$ 是速度層級的精確關係；Newton 法把它用在有限的步長上：$J\\,\\Delta\\theta \\approx \\mathcal V_b \\cdot (1\\ \\text{s})$。把 Newton 步除以時間步長 $\\Delta t$ 就是 $\\dot\\theta$，所以追蹤控制可以看成「每個控制週期做一次 Newton 修正」，奇異附近的問題與補救（步長限制、DLS）也完全相同。',
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

function mount(host: HTMLElement, ctx: LessonContext): LessonHandle {
  // ------------------------------------------------------------------ state
  let task: Task = { ...DEFAULT_TASK };
  let theta0: Vec = GUESS.start.slice();
  const opts: StepOptions = { method: 'pinv', lambda: 0.1, maxStep: Infinity, weights: null };
  let records: IterRecord[] = [evaluate(theta0, task)];
  let status: RunStatus = 'running';
  let cmp: { records: IterRecord[]; status: RunStatus; method: Method } | null = null;
  const show = { twist: true, iterates: true, compare: false, nullspace: false };
  let anim: { from: Vec; d: Vec; t: number; dur: number } | null = null;
  let step = -1;
  let mode: 'iterate' | 'track' = 'iterate';
  let trackIdx = 0;
  let tTrack = 0;
  let thetaT: Vec = GUESS.good.slice();
  let rates: Vec = [0, 0, 0];
  let trackErr = 0;
  let tipTrail: Pt[] = [];
  /** (t, ‖θ̇‖) samples inside the plot window, to keep the rate plot's y-range zero-based. */
  let rateWin: [number, number][] = [];
  let prevNull: Vec | null = null;

  // ------------------------------------------------------------------ layout
  simTitle(host, '數值 IK：一步一步逼近目標', '平面 3R');
  const cv = new Canvas2D(host, { height: 315, world: { xmin: -2.5, xmax: 2.6, ymin: -1.25, ymax: 2.45 }, overlay: true });

  // The comparison series are only listed (and drawn) while the comparison is on.
  const seriesE = (withCmp: boolean): SeriesSpec[] => {
    const c1 = cssVar('--series-1', '#2563eb'), c2 = cssVar('--series-2', '#e11d48');
    const base: SeriesSpec[] = [{ name: 'ω', color: c1 }, { name: 'v', color: c2 }];
    return withCmp ? [...base, { name: '對照 ω', color: c1, dash: [5, 4], width: 1.4 }, { name: '對照 v', color: c2, dash: [5, 4], width: 1.4 }] : base;
  };
  const seriesS = (withCmp: boolean): SeriesSpec[] => {
    const c4 = cssVar('--series-4', '#d97706');
    return withCmp ? [{ name: '本方法', color: c4 }, { name: '對照', color: c4, dash: [5, 4], width: 1.4 }] : [{ name: '本方法', color: c4 }];
  };
  const [pl, pr] = split(host);
  const plotsRow = pl.parentElement as HTMLElement;
  const plotE = new Plot2D(pl, { title: '誤差 log₁₀‖·‖', xLabel: '迭代 k', height: 132, yRange: [-13, 2.5], xRange: [0, 6], series: seriesE(true), legend: true });
  plotE.setOptions({ series: seriesE(false) });
  const plotS = new Plot2D(pr, { title: '每步 ‖Δθ‖ (rad)', xLabel: '迭代 k', height: 132, xRange: [0, 6], yRange: [0, 1.1], series: seriesS(true), legend: false });
  plotS.setOptions({ series: seriesS(false) });
  const [tl, tr] = split(host);
  const trackRow = tl.parentElement as HTMLElement;
  trackRow.style.display = 'none';
  const plotRate = new Plot2D(tl, { title: '‖θ̇‖ (rad/s)', xLabel: 't (s)', height: 132, xWindow: 12, yRange: [0, 1], series: [{ name: '‖θ̇‖' }] });
  const plotTE = new Plot2D(tr, { title: '追蹤誤差 log₁₀‖V_err‖', xLabel: 't (s)', height: 132, xWindow: 12, yRange: [-7, 1], series: [{ name: '誤差', color: cssVar('--series-2', '#e11d48') }] });

  const panel = controlPanel(host, { compact: true });
  const chipRow = row(panel);
  const runRow = row(panel);
  const stepper = algorithmStepper(runRow, { onStep: doStep, onReset: resetRun, intervalMs: 700, speeds: [0.5, 1, 2, 5] });
  const pb = playback(runRow, {
    onPlay: () => trackLoop.start(),
    onPause: () => trackLoop.stop(),
    onReset: () => resetTrack(),
    onSpeed: (s) => { trackLoop.speed = s; },
  });
  pb.el.style.display = 'none';
  const selMethod = select<Method>(panel, {
    label: '方法',
    value: opts.method,
    options: [
      { value: 'pinv', label: 'J⁺（Newton）' },
      { value: 'dls', label: 'DLS（阻尼）' },
      { value: 'transpose', label: 'Jᵀ（轉置法）' },
    ],
    onChange: (m) => setMethod(m),
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
  const tPos = toggle(togRow, { label: '只要位置', title: '只追蹤末端位置 (x, y)：3R 變成冗餘手臂，J 是 2×3', value: false, onChange: (v) => setPositionOnly(v) });
  const tTwist = toggle(togRow, { label: '誤差 twist', value: show.twist, onChange: (v) => { show.twist = v; cv.requestRender(); } });
  const tIter = toggle(togRow, { label: '每步姿勢', value: show.iterates, onChange: (v) => { show.iterates = v; cv.requestRender(); } });
  const tCmp = toggle(togRow, { label: '對照', title: '以虛線疊上另一種方法（J⁺ ↔ DLS）從同一初值的結果', value: show.compare, onChange: (v) => { show.compare = v; computeCompare(); updatePlots(); } });
  const tNull = toggle(togRow, { label: '零空間', title: '畫出零空間自運動時各關節的速度', value: show.nullspace, onChange: (v) => { show.nullspace = v; cv.requestRender(); } });
  const tW = toggle(togRow, { label: '加權 J⁺', title: 'W = diag(10, 1, 1)：關節 1 的「成本」是其他關節的 10 倍', value: false, onChange: (v) => { opts.weights = v ? [10, 1, 1] : null; inputsChanged(); } });

  const K = { V: 'V_b（誤差）', dth: 'Δθ（上一步）', sig: 'σ_min(J_b)', note: '目標／冗餘' } as const;
  const info = kv(host, Object.values(K));
  const KT = { rate: 'θ̇ (rad/s)', sig: 'σ_min(J_b)' } as const;
  const infoT = kv(host, Object.values(KT));
  infoT.el.style.display = 'none';
  // Two key/value pairs per row when there is room. The style change is deferred to the next frame
  // (and skipped when nothing changes) so it never re-triggers this observer inside its own callback.
  let kvCols = '';
  let kvRaf = 0;
  const applyCols = () => {
    const cols = host.clientWidth >= 520 ? 'auto 1fr auto 1fr' : 'auto 1fr';
    if (cols === kvCols) return;
    kvCols = cols;
    info.el.style.gridTemplateColumns = cols;
    infoT.el.style.gridTemplateColumns = cols;
  };
  applyCols();
  const ro = new ResizeObserver(() => { cancelAnimationFrame(kvRaf); kvRaf = requestAnimationFrame(applyCols); });
  ro.observe(host);
  info.el.style.margin = infoT.el.style.margin = '0.4rem 0 0';

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
  const purple = cssVar('--series-5', '#7c3aed');
  const targetHandle: Handle = { get: () => [task.x, task.y], set: (p) => { task.x = p[0]; task.y = p[1]; inputsChanged(); }, radius: 9, color: purple };
  const knobHandle: Handle = { get: knobPos, set: (p) => { task.phi = Math.atan2(p[1] - task.y, p[0] - task.x); inputsChanged(); }, radius: 6, color: purple };
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
    color: cssVar('--fg-muted', '#666'),
  }));
  cv.addHandle(targetHandle);
  cv.addHandle(knobHandle);
  guessHandles.forEach((h) => cv.addHandle(h));

  // ------------------------------------------------------------------ run logic
  function statusText(): string {
    const last = records[records.length - 1];
    if (status === 'converged') return `✓ 收斂：${records.length - 1} 步`;
    if (status === 'stalled') return `■ 停滯：殘差卡在 ${fe(last.omegaNorm + last.vNorm)}，不再下降`;
    if (status === 'maxIter') return `■ ${MAX_ITER} 步仍未收斂`;
    return '';
  }

  function resetRun(): void {
    anim = null;
    animLoop.stop();
    prevNull = null;
    records = [evaluate(theta0, task)];
    status = statusOf(records, LESSON_TOL, MAX_ITER);
    computeCompare();
    updatePlots();
    updateReadouts();
    cv.requestRender();
    if (status !== 'running') stepper.finish(statusText());
  }

  /** Any input changed: restart the iteration from θ⁰ (also resets the stepper's counter). */
  function inputsChanged(): void {
    if (mode === 'track') { updateReadouts(); cv.requestRender(); return; }
    chipSets[step]?.clear();
    stepper.reset();
  }

  function doStep(): boolean {
    if (status !== 'running') return false;
    const last = records[records.length - 1];
    const next = iterate(last, task, opts);
    records.push(next);
    status = statusOf(records, LESSON_TOL, MAX_ITER);
    const dn = last.dthetaNorm;
    anim = { from: last.theta.slice(), d: last.dtheta!.slice(), t: 0, dur: Math.min(1.4, 0.25 + 0.12 * dn) };
    animLoop.start();
    updatePlots();
    updateReadouts();
    cv.requestRender();
    if (status !== 'running') { stepper.setStatus(statusText()); return false; }
    return true;
  }

  const otherMethod = (m: Method): Method => (m === 'pinv' ? 'dls' : 'pinv');
  const methodShort = (m: Method) => (m === 'pinv' ? 'J⁺' : m === 'dls' ? `DLS λ=${fmt(opts.lambda, 2)}` : 'Jᵀ');

  function computeCompare(): void {
    if (!show.compare) { cmp = null; return; }
    const m = otherMethod(opts.method);
    const r = runNewton(theta0, task, { ...opts, method: m, weights: m === 'pinv' ? opts.weights : null }, MAX_ITER);
    cmp = { ...r, method: m };
  }

  function setMethod(m: Method): void {
    opts.method = m;
    selMethod.set(m, { silent: true });
    sLam.el.style.opacity = m === 'dls' ? '' : '0.45';
    tW.el.style.display = task.positionOnly && m === 'pinv' && mode === 'iterate' ? '' : 'none';
    inputsChanged();
  }

  function setPositionOnly(v: boolean): void {
    task.positionOnly = v;
    tPos.set(v, { silent: true });
    knobHandle.enabled = !v && mode === 'iterate';
    tNull.el.style.display = v && mode === 'iterate' ? '' : 'none';
    tW.el.style.display = v && opts.method === 'pinv' && mode === 'iterate' ? '' : 'none';
    if (!v) { opts.weights = null; tW.set(false, { silent: true }); }
    inputsChanged();
  }

  function setGuess(q: Vec): void {
    theta0 = q.slice();
    s0.forEach((s, i) => s.set(theta0[i], { silent: true }));
  }

  function setTask(t: Task): void {
    task = { ...t };
    knobHandle.enabled = !task.positionOnly && mode === 'iterate';
  }

  // ------------------------------------------------------------------ plots & readouts
  function updatePlots(): void {
    plotE.setOptions({ title: cmp ? `誤差 log₁₀‖·‖（虛線：${methodShort(cmp.method)}）` : '誤差 log₁₀‖·‖', series: seriesE(!!cmp) });
    plotS.setOptions({ title: cmp ? `每步 ‖Δθ‖ (rad)（虛線：${methodShort(cmp.method)}）` : '每步 ‖Δθ‖ (rad)', series: seriesS(!!cmp) });
    const n = records.length;
    const ks = records.map((_, i) => i);
    const pos = task.positionOnly;
    plotE.setXY(0, pos ? [] : ks, pos ? [] : records.map((r) => lg(r.omegaNorm)));
    plotE.setXY(1, ks, records.map((r) => lg(r.vNorm)));
    const steps = records.slice(0, n - 1);
    plotS.setXY(0, steps.map((_, i) => i), steps.map((r) => r.dthetaNorm));
    if (cmp) {
      const kc = cmp.records.map((_, i) => i);
      plotE.setXY(2, pos ? [] : kc, pos ? [] : cmp.records.map((r) => lg(r.omegaNorm)));
      plotE.setXY(3, kc, cmp.records.map((r) => lg(r.vNorm)));
      const cs = cmp.records.slice(0, -1);
      plotS.setXY(1, cs.map((_, i) => i), cs.map((r) => r.dthetaNorm));
    } else {
      plotE.setXY(2, [], []); plotE.setXY(3, [], []); plotS.setXY(1, [], []);
    }
    const xmax = Math.max(6, n - 1, cmp ? Math.min(cmp.records.length - 1, MAX_ITER) : 0);
    plotE.setRanges([0, xmax], undefined);
    let ymax = 1;
    for (const r of records) ymax = Math.max(ymax, r.dthetaNorm);
    if (cmp) for (const r of cmp.records) ymax = Math.max(ymax, r.dthetaNorm);
    plotS.setRanges([0, xmax], [0, ymax * 1.1]);
    const last = records[n - 1];
    const c1 = cssVar('--series-1', '#2563eb'), c2 = cssVar('--series-2', '#e11d48');
    const mk: { x?: number; y?: number; color?: string; label?: string; dash?: number[] }[] = [{ y: lg(LESSON_TOL.ev), label: '門檻 10⁻⁶', dash: [2, 3] }];
    if (!pos) mk.push({ x: n - 1, y: lg(last.omegaNorm), color: c1 });
    mk.push({ x: n - 1, y: lg(last.vNorm), color: c2 });
    plotE.setMarkers(mk);
    const ms: { x?: number; y?: number; color?: string; label?: string; dash?: number[] }[] = [];
    if (Number.isFinite(opts.maxStep)) ms.push({ y: opts.maxStep, label: '最大步長', dash: [2, 3] });
    if (n > 1) ms.push({ x: n - 2, y: records[n - 2].dthetaNorm, color: cssVar('--series-4', '#d97706') });
    plotS.setMarkers(ms);
  }

  function updateReadouts(): void {
    if (mode === 'track') {
      const rn = norm(rates);
      infoT.set(KT.rate, `(${rates.map((x) => fmt(x, 2)).join(', ')})${rn > 5 ? ' ⚠ 過大' : ''}`);
      const Sv = singularValues(reducedJacobian(thetaT, task.positionOnly));
      const S = Sv[Sv.length - 1];
      infoT.set(KT.sig, `${fmt(S, 4)}${S < 0.05 ? ' ⚠ 近奇異' : ''}`);
      return;
    }
    const n = records.length;
    const last = records[n - 1];
    const prev = n > 1 ? records[n - 2] : null;
    info.set(K.V, `${task.positionOnly ? '(v_x, v_y)' : '(ω_z, v_x, v_y)'} = (${last.V.map((x) => fmt(x, 2)).join(', ')})`);
    info.set(K.dth, prev?.dtheta ? `(${prev.dtheta.map((x) => fmt(x, 2)).join(', ')})，‖Δθ‖ = ${fmt(prev.dthetaNorm, 3)} rad${prev.clipped ? '（已截斷）' : ''}` : '—（尚未迭代）');
    info.set(K.sig, `${fmt(last.sigmaMin, 4)}${last.sigmaMin < 0.05 ? ' ⚠ 近奇異' : ''}`);
    const reach = poseReachable(task);
    const reachText = reach.reachable ? '可達' : task.positionOnly ? `不可達（|p| = ${fmt(reach.r, 2)} > ${fmt(REACH, 1)} m）` : `不可達（腕心距基座 ${fmt(reach.r, 2)} m，不在 [0.2, 1.8]）`;
    let nullText = '';
    if (task.positionOnly) {
      const nv = prev?.dtheta ? nullSpaceDirection(reducedJacobian(prev.theta, true)) : null;
      if (prev?.dtheta && nv) {
        const along = dot(prev.dtheta, nv);
        const frac = prev.dthetaNorm > 1e-15 ? Math.abs(along) / prev.dthetaNorm : 0;
        nullText = `；Δθ·n̂ = ${fmt(along, 4)}（占 ‖Δθ‖ 的 ${fmt(100 * frac, 0)}%，${opts.weights ? '加權' : '最小範數'}）`;
      } else nullText = prev?.dtheta ? '；Δθ·n̂：J 降秩' : '；Δθ·n̂ = —';
    }
    info.set(K.note, reachText + nullText);
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
  function resetTrack(): void {
    trackLoop.stop();
    pb.setPlaying(false, { silent: true });
    tTrack = 0;
    const tg = pathAt(TRACKS[trackIdx], 0);
    const start = runNewton(GUESS.good, { x: tg.x, y: tg.y, phi: tg.phi, positionOnly: task.positionOnly }, { method: 'pinv', lambda: 0, maxStep: 0.5 }, 100);
    thetaT = start.records[start.records.length - 1].theta.slice();
    rates = [0, 0, 0];
    trackErr = 0;
    tipTrail = [];
    rateWin = [];
    plotRate.setRanges(undefined, [0, 1]);
    plotRate.clear();
    plotTE.clear();
    pb.setTime(0);
    updateReadouts();
    cv.requestRender();
  }

  const trackLoop = createLoop((dt) => {
    const P = TRACKS[trackIdx];
    substep(dt, 1 / 120, (h) => {
      const r = trackingRates(thetaT, pathAt(P, tTrack), task.positionOnly, K_TRACK, opts);
      const qd = r.thetadot.map((x) => (Number.isFinite(x) ? x : 0));
      thetaT = thetaT.map((x, i) => wrapToPi(x + qd[i] * h));
      rates = qd;
      trackErr = r.err.omegaNorm + r.err.vNorm;
      tTrack += h;
    });
    const tip = planarArmPoints(LINKS, thetaT)[3];
    tipTrail.push(tip);
    if (tipTrail.length > 400) tipTrail.shift();
    const rn = norm(rates);
    rateWin.push([tTrack, rn]);
    while (rateWin.length && rateWin[0][0] < tTrack - 12) rateWin.shift();
    let top = 1;
    for (const [, v] of rateWin) top = Math.max(top, v * 1.1);
    plotRate.setRanges(undefined, [0, top]);
    plotRate.push(tTrack, [rn]);
    plotTE.push(tTrack, [lg(trackErr)]);
    pb.setTime(tTrack);
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
    trackRow.style.display = it ? 'none' : '';
    info.el.style.display = it ? '' : 'none';
    infoT.el.style.display = it ? 'none' : '';
    s0.forEach((s) => { s.el.style.display = it ? '' : 'none'; });
    sMax.el.style.display = it ? '' : 'none';
    togRow.style.display = it ? '' : 'none';
    [tTwist.el, tIter.el, tCmp.el].forEach((e) => { e.style.display = it ? '' : 'none'; });
    targetHandle.enabled = it;
    knobHandle.enabled = it && !task.positionOnly;
    guessHandles.forEach((h) => { h.enabled = it; });
    tNull.el.style.display = it && task.positionOnly ? '' : 'none';
    tW.el.style.display = it && task.positionOnly && opts.method === 'pinv' ? '' : 'none';
    card.style.display = it ? 'none' : 'flex';
    if (it) { trackLoop.stop(); pb.setPlaying(false, { silent: true }); stepper.reset(); } else { anim = null; animLoop.stop(); resetTrack(); }
  }

  // ------------------------------------------------------------------ drawing
  function drawBase(c: Canvas2D): void {
    const g = cssVar('--grid-strong', '#bbb');
    c.polygon([[-0.1, -0.11], [0.1, -0.11], [0, 0]], { fill: g, color: g, width: 1 });
    c.line([-0.16, -0.11], [0.16, -0.11], { color: cssVar('--fg-muted'), width: 2 });
    c.frame({ x: 0, y: 0, theta: 0 }, 0.28, { width: 1.5, alpha: 0.8, labels: 's' });
  }

  function drawArm(c: Canvas2D, q: Vec, color: string, style: 'solid' | 'ghost' | 'dashed', alpha = 1): Pt[] {
    const pts = planarArmPoints(LINKS, q);
    if (style === 'solid') {
      for (let i = 0; i < 3; i++) c.line(pts[i], pts[i + 1], { color, width: 7, alpha: 0.85 * alpha });
      for (let i = 0; i < 3; i++) {
        c.point(pts[i], 5, { color: cssVar('--bg-elev', '#fff'), alpha });
        c.circle(pts[i], 5 / c.scale, { color: cssVar('--fg', '#222'), width: 1.6, alpha });
      }
    } else if (style === 'dashed') {
      for (let i = 0; i < 3; i++) c.line(pts[i], pts[i + 1], { color, width: 2, dash: [6, 4], alpha });
    } else {
      for (let i = 0; i < 3; i++) c.line(pts[i], pts[i + 1], { color, width: 4, alpha: 0.28 * alpha });
    }
    return pts;
  }

  /** Tip frame {b}; the name is placed behind the frame (opposite to its x-axis) to keep the target area readable. */
  function tipFrame(c: Canvas2D, q: Vec, label: string | undefined, alpha = 1): void {
    const phi = q[0] + q[1] + q[2];
    const tip = planarArmPoints(LINKS, q)[3];
    c.frame({ x: tip[0], y: tip[1], theta: phi }, 0.3, { width: 2, alpha });
    if (label) c.text([tip[0] - 0.2 * Math.cos(phi), tip[1] - 0.2 * Math.sin(phi)], label, { color: cssVar('--fg', '#222'), font: `12px ${cssVar('--mono') || 'monospace'}`, dx: 0, dy: 0, bg: cssVar('--sim-bg', '#fff') });
  }

  function drawTwist(c: Canvas2D, q: Vec, color: string): void {
    const r = residual(q, task);
    const tip: Pt = [r.Tsb[0][3], r.Tsb[1][3]];
    if (task.positionOnly) {
      if (r.vNorm > 1e-3) c.arrow(tip, [task.x, task.y], { color, width: 2, dash: [5, 4], head: 9 });
      return;
    }
    if (r.omegaNorm + r.vNorm < 2e-3) return;
    const arc = screwArc(r.Tsb, r.V6, 32);
    const pts = arc.map((T) => [T[0][3], T[1][3]] as Pt);
    c.polyline(pts, { color, width: 2.2, dash: [6, 4] });
    for (const k of [11, 21]) {
      const T = arc[k];
      c.frame({ x: T[0][3], y: T[1][3], theta: Math.atan2(T[1][0], T[0][0]) }, 0.14, { width: 1.3, alpha: 0.55 });
    }
    const mid = pts[16];
    c.text(mid, 'V_b', { color, dx: 14, dy: 12, bg: cssVar('--sim-bg', '#fff'), font: `11px ${cssVar('--mono') || 'monospace'}` });
  }

  function drawNull(c: Canvas2D, q: Vec, color: string): void {
    const J = reducedJacobian(q, true);
    const n = nullSpaceDirection(J, prevNull);
    if (!n) return;
    prevNull = n;
    const eps = 1e-4;
    const p0 = planarArmPoints(LINKS, q);
    const p1 = planarArmPoints(LINKS, q.map((x, i) => x + eps * n[i]));
    const vel: Pt[] = [1, 2, 3].map((i) => [(p1[i][0] - p0[i][0]) / eps, (p1[i][1] - p0[i][1]) / eps] as Pt);
    // Longest arrow drawn 0.45 m long; the tip's velocity is zero (that is what "null space" means).
    const scale = 0.45 / Math.max(1e-9, ...vel.map((v) => Math.hypot(v[0], v[1])));
    vel.forEach((v, k) => {
      const p = p0[k + 1];
      if (Math.hypot(v[0], v[1]) * scale > 0.03) c.arrow(p, [p[0] + v[0] * scale, p[1] + v[1] * scale], { color, width: 2.6, head: 10 });
      else c.circle(p, 8 / c.scale, { color, width: 2 });
    });
    c.text(p0[1], '自運動 n̂', { color, dx: -8, dy: 18, bg: cssVar('--sim-bg', '#fff') });
  }

  function drawTarget(c: Canvas2D, purpleC: string): void {
    if (task.positionOnly) {
      const k = 0.08;
      c.line([task.x - k, task.y], [task.x + k, task.y], { color: purpleC, width: 2 });
      c.line([task.x, task.y - k], [task.x, task.y + k], { color: purpleC, width: 2 });
      return;
    }
    c.frame({ x: task.x, y: task.y, theta: task.phi }, 0.35, { width: 2.5 });
    c.line([task.x, task.y], knobPos(), { color: purpleC, width: 1.5, dash: [4, 3] });
    c.text([task.x + 0.24 * Math.cos(task.phi + 2.3), task.y + 0.24 * Math.sin(task.phi + 2.3)], '{d}', { color: purpleC, font: `12px ${cssVar('--mono') || 'monospace'}`, bg: cssVar('--sim-bg', '#fff') });
  }

  cv.draw = (c) => {
    const blue = cssVar('--series-1', '#2563eb');
    const purpleC = cssVar('--series-5', '#7c3aed');
    const cyan = cssVar('--series-6', '#0891b2');
    const muted = cssVar('--fg-muted', '#666');
    const pink = cssVar('--series-7', '#db2777');
    targetHandle.color = knobHandle.color = purpleC;
    guessHandles.forEach((h) => { h.color = muted; });
    c.grid(0.5);
    c.circle([0, 0], REACH, { color: cssVar('--series-3', '#16a34a'), width: 1.2, dash: [6, 4] });
    drawBase(c);

    if (mode === 'track') {
      const P = TRACKS[trackIdx];
      const circ: Pt[] = [];
      for (let k = 0; k <= 72; k++) { const a = (k / 72) * 2 * Math.PI; circ.push([P.cx + P.rho * Math.cos(a), P.cy + P.rho * Math.sin(a)]); }
      c.polyline(circ, { color: purpleC, width: 1.2, dash: [4, 4] });
      c.polyline(tipTrail, { color: pink, width: 2, alpha: 0.8 });
      const tg = pathAt(P, tTrack);
      if (!task.positionOnly) c.frame({ x: tg.x, y: tg.y, theta: tg.phi }, 0.35, { width: 2.5 });
      c.point([tg.x, tg.y], 6, { color: purpleC });
      drawArm(c, thetaT, blue, 'solid');
      tipFrame(c, thetaT, '{b}');
      c.overlay.textContent = `t = ${fmt(tTrack, 1)} s   ‖θ̇‖ = ${fmt(norm(rates), 2)} rad/s   誤差 = ${fe(trackErr)}`;
      return;
    }

    drawTarget(c, purpleC);
    // initial guess (ghost) with its drag handles
    const g = drawArm(c, theta0, muted, 'dashed', 0.9);
    c.text(g[3], 'θ⁰', { color: muted, dx: 12, dy: 10 });
    const n = records.length;
    const a = animTheta();
    if (show.iterates) {
      const from = Math.max(1, n - 7);
      for (let k = from; k < n - (a ? 0 : 1); k++) drawArm(c, records[k].theta, blue, 'ghost', 0.35 + 0.65 * ((k - from + 1) / (n - from)));
    }
    // tip path through the iterates
    const tips = records.map((r) => planarArmPoints(LINKS, r.theta)[3]);
    if (tips.length > 1) c.polyline(a ? tips.slice(0, -1) : tips, { color: cssVar('--series-4', '#d97706'), width: 1.5, alpha: 0.9 });
    tips.forEach((p, k) => {
      if (a && k === n - 1) return;
      c.point(p, 3.5, { color: cssVar('--series-4', '#d97706') });
      if (k > 0 && k < 6) c.text(p, String(k), { color: cssVar('--series-4', '#d97706'), dx: 9, dy: -9, font: '11px sans-serif' });
    });
    const q = a ?? records[n - 1].theta;
    drawArm(c, q, blue, 'solid');
    tipFrame(c, q, a ? undefined : '{b}');
    if (!a && show.twist) drawTwist(c, q, cyan);
    if (!a && task.positionOnly && show.nullspace) drawNull(c, q, pink);
    const last = records[n - 1];
    c.overlay.textContent = `k = ${n - 1}   ${task.positionOnly ? '' : `‖ω_b‖ = ${fe(last.omegaNorm)}   `}‖v_b‖ = ${fe(last.vNorm)}   ${statusText()}`;
  };

  fb.draw = (c) => {
    const fg = cssVar('--fg', '#222');
    const muted = cssVar('--fg-muted', '#666');
    const acc = cssVar('--series-5', '#7c3aed');
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
    c.line([-0.7, 0], [0.7, 0], { color: muted, width: 2 });
    c.line(C1, P2, { color: muted, width: 1.5, dash: [4, 3] });
    c.line(C2, P2, { color: muted, width: 1.5, dash: [4, 3] });
    c.circle(P2, 4 / c.scale, { color: muted, width: 1.5 });
    for (const [a, b] of [[A, C1], [B, C2]] as [Pt, Pt][]) c.line(a, b, { color: cssVar('--series-1', '#2563eb'), width: 5 });
    c.line(C1, P, { color: fg, width: 3 });
    c.line(C2, P, { color: fg, width: 3 });
    for (const p of [A, B, C1, C2]) { c.point(p, 3.5, { color: cssVar('--bg-elev', '#fff') }); c.circle(p, 3.5 / c.scale, { color: fg, width: 1.5 }); }
    c.point(P, 5, { color: acc });
    c.text(P, 'P', { color: acc, dx: 10, dy: -4 });
    c.text(A, '馬達 1', { color: muted, dy: 11, font: '10px sans-serif' });
    c.text(B, '馬達 2', { color: muted, dy: 11, font: '10px sans-serif' });
  };

  // ------------------------------------------------------------------ presets per step
  const resetBoth = (q: Vec) => { setTask({ ...DEFAULT_TASK, positionOnly: task.positionOnly }); setGuess(q); };
  const chipSets: PresetChipsHandle[] = [
    presetChips(chipRow, [
      { label: '重設目標與初值', apply: () => { resetBoth(GUESS.start); stepper.reset(); } },
    ]),
    presetChips(chipRow, [
      { label: '重設目標與初值', apply: () => { resetBoth(GUESS.start); stepper.reset(); } },
    ]),
    presetChips(chipRow, [
      { label: '好的初值 (0°, 90°, 0°)', title: '目標重設為 (1.0, 1.2, 60°)', apply: () => { resetBoth(GUESS.good); stepper.reset(); } },
      { label: '壞的初值 (−150°, 60°, 90°)', title: '目標重設為 (1.0, 1.2, 60°)', apply: () => { resetBoth(GUESS.bad); stepper.reset(); } },
    ], { label: '初值：' }),
    presetChips(chipRow, [
      { label: '近奇異初值 (40°, 3°, −20°)', apply: () => { resetBoth(GUESS.sing); setMaxStep(STEP_INF); } },
      { label: '最大步長 = 1 rad', apply: () => setMaxStep(1) },
      { label: '不限制步長', apply: () => setMaxStep(STEP_INF) },
    ]),
    presetChips(chipRow, [
      { label: 'λ = 0.05', apply: () => setLambda(0.05) },
      { label: 'λ = 0.1', apply: () => setLambda(0.1) },
      { label: 'λ = 0.5', apply: () => setLambda(0.5) },
      { label: '好的初值', apply: () => { resetBoth(GUESS.good); stepper.reset(); } },
      { label: '近奇異初值', apply: () => { resetBoth(GUESS.sing); stepper.reset(); } },
    ]),
    presetChips(chipRow, [
      { label: '冗餘初值 (10°, 60°, 60°)', apply: () => { resetBoth(GUESS.redund); stepper.reset(); } },
      { label: '加權 J⁺', apply: () => setWeighted(true) },
      { label: '一般 J⁺', apply: () => setWeighted(false) },
    ]),
    presetChips(chipRow, TRACKS.map((P, i) => ({ label: P.label, title: P.title, apply: () => { trackIdx = i; resetTrack(); } })), { label: '路徑：' }),
  ];

  function setMaxStep(v: number): void {
    sMax.set(v, { silent: true });
    opts.maxStep = v >= STEP_INF - 1e-9 ? Infinity : v;
    stepper.reset();
  }
  function setLambda(v: number): void {
    sLam.set(v, { silent: true });
    opts.lambda = v;
    stepper.reset();
  }
  /** Weighted vs. plain pseudo-inverse from the chips (keeps the chip highlighted, unlike a manual toggle). */
  function setWeighted(v: boolean): void {
    tW.set(v, { silent: true });
    opts.weights = v ? [10, 1, 1] : null;
    stepper.reset();
  }

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
    step = i;
    chipSets.forEach((cs, k) => { cs.el.style.display = k === i ? '' : 'none'; cs.clear(); });
    if (i === 6) {
      if (task.positionOnly) setPositionOnly(false);
      setMode('track');
      chipSets[6].activate(0);
      setFocus([trackRow, selMethod.el]);
      return;
    }
    setMode('iterate');
    stepper.pause();
    setToggle(tCmp, 'compare', i === 4);
    setToggle(tNull, 'nullspace', i === 5);
    setToggle(tTwist, 'twist', i <= 1 || i === 5);
    setToggle(tIter, 'iterates', true);
    if (i !== 5 && task.positionOnly) { task.positionOnly = false; tPos.set(false, { silent: true }); opts.weights = null; tW.set(false, { silent: true }); }
    knobHandle.enabled = !task.positionOnly;
    tNull.el.style.display = task.positionOnly ? '' : 'none';
    if (i <= 3 || i === 5) { opts.method = 'pinv'; selMethod.set('pinv', { silent: true }); }
    if (i === 4) { opts.method = 'dls'; selMethod.set('dls', { silent: true }); }
    sLam.el.style.opacity = opts.method === 'dls' ? '' : '0.45';
    if (i >= 3) { sMax.set(STEP_INF, { silent: true }); opts.maxStep = Infinity; }
    if (i === 5) {
      task.positionOnly = true; tPos.set(true, { silent: true });
      knobHandle.enabled = false;
      tNull.el.style.display = '';
    }
    tW.el.style.display = task.positionOnly && opts.method === 'pinv' ? '' : 'none';
    if (i === 2) chipSets[2].activate(0);
    else if (i === 3) chipSets[3].activate(0);
    else if (i === 4) { resetBoth(GUESS.sing); chipSets[4].activate(1); }
    else if (i === 5) { opts.weights = null; tW.set(false, { silent: true }); chipSets[5].activate(0); }
    else stepper.reset();
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

  const offTheme = ctx.onThemeChange(() => {
    plotE.setOptions({ series: seriesE(!!cmp) });
    plotS.setOptions({ series: seriesS(!!cmp) });
    plotTE.setOptions({ series: [{ name: '誤差', color: cssVar('--series-2', '#e11d48') }] });
  });

  onStep(ctx.currentStep);

  return {
    onStep,
    destroy() {
      offTheme();
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
