import type { Lesson, LessonContext, LessonHandle, LessonStep, QuizQuestion } from '@/types';
import {
  el, cssVar, fmt, controlPanel, row, slider, toggle, select, playback, simTitle, split,
  Canvas2D, Plot2D, plotGroup, presetChips, createLoop, drawPlanarArm,
  type Pt, type Marker, type SeriesSpec, type Handle,
} from '@/ui';
import { planar2RIK, planarFK } from '@/mr';
import { meta } from './meta';
import {
  DEFAULT_PLANT, charPoly, dominantRoot, pdGainsFromPole, pdPrediction, pdSag2R, pidKiBound, polyRoots, simulateJoint,
  stepMetrics, tailAmplitude, type Complex, type InputMode, type RefKind, type Reference, type Trial, type TrialOptions,
} from './pidSim';

const steps: LessonStep[] = [
  {
    title: '控制就是讓誤差消失',
    md: `控制器要讓關節角 $\\theta$ 跟上期望值 $\\theta_d$。與其盯著 $\\theta$ 本身，不如盯著兩者的差：

$$\\theta_e(t) = \\theta_d(t) - \\theta(t).$$

把閉迴路寫成 $\\theta_e$ 的微分方程，就是**誤差動態（error dynamics）**；好的控制器讓這個方程穩定而且收斂得快。評估一次階躍響應，常看三個數字：

- **穩態誤差** $e_{ss}$：等得夠久之後還剩多少誤差；
- **超調（overshoot）** $M_p$：誤差衝過終值多少，以起始差距 $|\\theta_e(0) - e_{ss}|$ 的百分比表示；
- **2% 安定時間** $t_s$：從這一刻起，誤差與終值的差距一直小於起始差距的 2%。

這個範例故意加了一點固定負載，所以 $e_{ss}$ 不是零，第 4 步會解釋原因。`,
    simHint: '播放動畫看關節從 0 轉到 θ_d = 1 rad。在誤差圖上找出 M_p 點、t_s 虛線與 e_ss 水平線，再對照下方表格的「模擬量測」列。',
  },
  {
    title: '速度輸入：一階誤差動態',
    md: `如果馬達驅動器內部已經有速度迴路，我們能直接**指定關節速度**。用比例控制 $\\dot\\theta = K_p\\theta_e$，目標固定（$\\dot\\theta_d = 0$）時

$$\\dot\\theta_e + K_p\\,\\theta_e = 0 \\quad\\Rightarrow\\quad \\theta_e(t) = \\theta_e(0)\\,e^{-K_p t}.$$

這是**一階**系統：時間常數 $1/K_p$，$t_s \\approx 4/K_p$，而且永遠不會超調；它唯一的特徵根就是實軸上的 $s = -K_p$，根越往左收斂越快。

若目標以固定速率 $c$ 前進（斜坡），P 控制會一直落後 $e_{ss} = c/K_p$。加上 I 項，或把 $\\dot\\theta_d$ 當作前饋加進指令，才追得平。`,
    simHint: '把 s 平面上的根往左拖（或拉 Kp），看 t_s 怎麼縮短、虛線理論曲線是否與模擬重合；再把參考輸入切到「斜坡」，量 e_ss 是否等於 c/Kp（c = 0.5 rad/s）。',
  },
  {
    title: '力矩輸入：二階誤差動態',
    md: `大多數關節真正能下的指令是**力矩**。關節模型 $M\\ddot\\theta + b\\dot\\theta = \\tau$ 配上 PD 控制 $\\tau = K_p\\theta_e + K_d\\dot\\theta_e$，目標固定時得到「質量—彈簧—阻尼器」形式：

$$M\\ddot\\theta_e + (b+K_d)\\,\\dot\\theta_e + K_p\\,\\theta_e = 0,$$

$$\\omega_n = \\sqrt{K_p/M}, \\qquad \\zeta = \\frac{b+K_d}{2\\sqrt{K_p M}} .$$

$K_p$ 像彈簧、$K_d$ 像阻尼（本頁關節 $M = 0.5$ kg·m²、$b = 0.1$ N·m·s/rad）。$\\zeta<1$ 時兩個共軛根讓響應振盪，超調 $M_p = e^{-\\pi\\zeta/\\sqrt{1-\\zeta^2}}$；安定時間可用包絡線粗估為 $t_s \\approx 4/(\\zeta\\omega_n)$，但 $\\zeta$ 接近 1 時這個估計明顯偏短（表格的「線性預測」是直接解線性方程得到的值）。$K_p$ 固定時，$\\zeta = 1$（臨界阻尼）是所有不超調的設定裡最快的。`,
    simHint: '目前 ζ ≈ 0.17，超調很大。把 Kd 往上拉，或直接把 s 平面上的根往左下拖，讓 ζ 接近 1；比較表格中的「線性預測」與「模擬量測」。',
  },
  {
    title: '擾動與穩態誤差',
    md: `真實關節還得扛著**擾動力矩** $\\tau_{dist}$：重力負載 $mgr\\cos\\theta$、摩擦、夾著的工件……模型變成 $M\\ddot\\theta + b\\dot\\theta + \\tau_{dist} = \\tau$。PD 控制停下來時 $\\dot\\theta_e = \\ddot\\theta_e = 0$，只剩

$$K_p\\, e_{ss} = \\tau_{dist} \\quad\\Rightarrow\\quad e_{ss} = \\frac{\\tau_{dist}}{K_p}.$$

想像一根彈簧：要撐住負載，它必須被拉長一點，所以誤差不可能是零。加大 $K_p$ 只能把誤差縮小，而且在 $K_d$ 不變時會讓 $\\zeta$ 下降、更容易振盪。`,
    simHint: '重力負載已開啟（mgr = 3 N·m）。把 Kp 從 20 拉到 40，看 e_ss 是否大約減半；也試試 τ_dist 滑桿，比較預測與量測的 e_ss。',
  },
  {
    title: '加上積分項',
    md: `積分項把過去的誤差全部累加起來：

$$\\tau = K_p\\theta_e + K_i\\!\\int_0^t\\!\\theta_e\\,dt + K_d\\dot\\theta_e .$$

只要還有一點誤差，積分就繼續長大，直到它獨自扛下 $\\tau_{dist}$、讓 $\\theta_e \\to 0$。代價是系統多了一階：對固定擾動微分一次可得

$$M\\dddot\\theta_e + (b+K_d)\\ddot\\theta_e + K_p\\dot\\theta_e + K_i\\theta_e = 0,$$

s 平面上多出第三個根。$K_i$ 越大，誤差歸零越快，但超調與慢振盪也越明顯；依 Routh–Hurwitz 判準，必須 $K_i < (b+K_d)K_p/M$ 才穩定。`,
    simHint: '從 Ki = 20 開始往上拉：誤差尾巴歸零了，但超調變大；看三個根怎麼移動。再把 Kd 調小，找出根跑進紅色不穩定區的 Ki。',
  },
  {
    title: '飽和與積分飽和（windup）',
    md: `致動器的力矩有上限 $|\\tau| \\le \\tau_{max}$。一旦飽和，迴路等於暫時斷開，前面的線性分析全部失效。更麻煩的是積分器還在累積誤差：等關節終於趕到目標附近，積分項已經存了一大筆「舊帳」，只能靠反方向的誤差慢慢抵銷，結果就是巨大的超調與拖很久的安定時間，這叫**積分飽和（integrator windup）**。

最簡單的**抗飽和（anti-windup）**：輸出已經飽和、而誤差還想把它推向同一側時，就暫停積分。`,
    simHint: '力矩上限已降到 5 N·m。看力矩圖中的 I 項（虛線）在飽和期間怎麼暴衝；再勾選「抗飽和」，比較誤差圖中的實線與灰色對照曲線。',
  },
  {
    title: '多關節會怎樣？',
    md: `單關節的分析之所以漂亮，是因為 $M$ 是常數、擾動只有一個。多關節手臂的質量矩陣 $M(\\theta)$ 會隨姿態改變，而且不是對角矩陣：關節 1 一加速，關節 2 就感受到一個力矩（**耦合**）；重力負載也隨整條手臂的姿態而變。

右側的 2R 手臂每個關節各用一組 PD、沒有任何模型補償：它像第 4 步一樣在重力下**下垂**，而且兩個關節的誤差會互相牽動。下一課要用動力學模型（重力補償、計算力矩法）解決這些問題。先完成下面的自我檢測吧！`,
    simHint: '拖曳淡色目標手臂的末端改變目標姿態：手臂越接近水平，下垂越明顯，因為重力力矩隨姿態改變。',
  },
];

const quiz: QuizQuestion[] = [
  {
    question: '一個力矩控制的關節用 PD 控制，目前 $\\zeta = 0.5$、$t_s = 2$ s。想讓 $t_s$ 縮短成 1 s、但超調維持不變，$K_p$ 與 $b+K_d$ 應該怎麼調？',
    options: ['$K_p$ 加倍，$K_d$ 不變', '$K_p$ 變 4 倍，$b+K_d$ 變 2 倍', '$K_p$ 與 $b+K_d$ 都加倍', '$K_p$ 不變，$b+K_d$ 加倍'],
    answer: 1,
    explanation: '超調只由 $\\zeta$ 決定，所以 $\\zeta$ 要固定；$t_s \\approx 4/(\\zeta\\omega_n)$ 減半代表 $\\omega_n$ 要加倍。$\\omega_n = \\sqrt{K_p/M}$，所以 $K_p$ 要變 4 倍；而 $\\zeta = (b+K_d)/(2\\sqrt{K_pM})$ 的分母因此變 2 倍，$b+K_d$ 也必須變 2 倍才能讓 $\\zeta$ 不變。',
  },
  {
    question: '速度輸入的關節用純 P 控制（$K_p = 4\\ \\text{s}^{-1}$）追蹤斜坡 $\\theta_d(t) = 0.8\\,t$ rad。等暫態消失之後，誤差 $\\theta_e$ 大約是多少？',
    options: ['0：P 控制最後總會追上', '0.2 rad，而且一直維持', '3.2 rad', '越來越大，沒有上限'],
    answer: 1,
    explanation: '穩態時關節必須以和目標相同的速度前進：$\\dot\\theta = 0.8$。但 P 控制給的速度是 $K_p\\theta_e$，所以 $\\theta_e = 0.8/4 = 0.2$ rad。誤差與斜率成正比、與 $K_p$ 成反比；要消除它，可加 I 項或把 $\\dot\\theta_d$ 當前饋。',
  },
  {
    question: '某個二階誤差響應的超調是 20%，振盪週期約 1 s。它的 $\\zeta$ 與 $\\omega_n$ 最接近哪一組？',
    options: ['$\\zeta \\approx 0.46,\\ \\omega_n \\approx 7.1$ rad/s', '$\\zeta \\approx 0.20,\\ \\omega_n \\approx 6.3$ rad/s', '$\\zeta \\approx 0.46,\\ \\omega_n \\approx 1.0$ rad/s', '$\\zeta \\approx 0.80,\\ \\omega_n \\approx 6.3$ rad/s'],
    answer: 0,
    explanation: '由 $M_p = e^{-\\pi\\zeta/\\sqrt{1-\\zeta^2}} = 0.2$ 解得 $\\zeta = -\\ln 0.2/\\sqrt{\\pi^2 + \\ln^2 0.2} \\approx 0.46$（超調百分比不等於 ζ）。週期 1 s 是**有阻尼**頻率：$\\omega_d = 2\\pi \\approx 6.28$ rad/s，所以 $\\omega_n = \\omega_d/\\sqrt{1-\\zeta^2} \\approx 7.1$ rad/s。',
  },
  {
    question: '一個 PID 關節做大幅階躍時，力矩飽和了將近一秒，而且**沒有**抗飽和。下列哪個預測最合理？',
    options: [
      '飽和期間積分項持續累積；關節追上目標後會明顯超調，還要一段時間把積分「放掉」',
      '積分項在飽和期間自動停止累積，所以響應和沒有飽和時一樣',
      '飽和只會讓上升變慢，不可能增加超調',
      '只要 $K_d$ 夠大，積分項就不會累積',
    ],
    answer: 0,
    explanation: '飽和時實際力矩被限制住，誤差消失得比線性模型預期慢，積分器卻照常累加這些誤差。關節到達目標時，積分項遠大於撐住負載所需，只能等誤差變號、反向累積來抵銷，於是產生大超調。抗飽和就是在飽和期間停止（或回授修正）積分。',
  },
];

// ---------------------------------------------------------------------------
// Simulation
// ---------------------------------------------------------------------------

const PLANT = DEFAULT_PLANT;
const T_END = 6;
const RAMP_C = 0.5;
const SINE_A = 0.5;
const SINE_F = 0.4;
const TAU_INF = 40;
const KP_MAX = { torque: 100, velocity: 20 } as const;
const PREVIEW_KP: [number, number] = [60, 12];

interface SimState {
  mode: InputMode;
  ref: RefKind;
  stepAmp: number;
  kpT: number; kiT: number; kdT: number;
  kpV: number; kiV: number;
  tauDist: number;
  gravity: boolean;
  tauMax: number;
  antiWindup: boolean;
  velFF: boolean;
  preview: boolean;
}

const BASE: SimState = {
  mode: 'torque', ref: 'step', stepAmp: 1, kpT: 20, kiT: 0, kdT: 2.4, kpV: 2, kiV: 0,
  tauDist: 0, gravity: false, tauMax: Infinity, antiWindup: false, velFF: false, preview: false,
};

const PRESETS: { label: string; title: string; s: Partial<SimState> }[] = [
  { label: '① 指標', title: 'PD、ζ ≈ 0.4，外加 1 N·m 固定負載', s: { kpT: 20, kiT: 0, kdT: 2.4, tauDist: 1 } },
  { label: '② 速度', title: '速度輸入、純 P（Kp = 2 1/s）', s: { mode: 'velocity', kpV: 2, kiV: 0 } },
  { label: '③ PD', title: '力矩輸入、PD、ζ ≈ 0.17', s: { kpT: 20, kiT: 0, kdT: 1 } },
  { label: '④ 擾動', title: '臨界阻尼 PD ＋ 重力負載', s: { kpT: 20, kiT: 0, kdT: 6.2, gravity: true } },
  { label: '⑤ PID', title: 'PID（Ki = 20）＋ 重力負載', s: { kpT: 20, kiT: 20, kdT: 6.2, gravity: true } },
  { label: '⑥ 飽和', title: 'τmax = 5 N·m，未開抗飽和', s: { kpT: 25, kiT: 30, kdT: 4, gravity: true, tauMax: 5, stepAmp: 1.2 } },
  { label: '⑦ 2R', title: '2R 手臂、獨立 PD 的下垂', s: { kpT: 20, kiT: 20, kdT: 6.2, gravity: true, preview: true } },
];

const niceStep = (span: number): number => {
  if (!(span > 0) || !Number.isFinite(span)) return 1;
  const raw = span / 6;
  const mag = Math.pow(10, Math.floor(Math.log10(raw)));
  const r = raw / mag;
  return (r < 1.5 ? 1 : r < 3 ? 2 : r < 7 ? 5 : 10) * mag;
};

function mount(host: HTMLElement, ctx: LessonContext): LessonHandle {
  const st: SimState = { ...BASE };
  let trial: Trial;
  let alt: Trial | null = null;
  let roots: Complex[] = [];
  let locus: Complex[] = [];
  let tCur = 0;
  let dragging = false;
  let destroyed = false;

  // ---- layout -------------------------------------------------------------
  simTitle(host, '單關節 PID 實驗台', '誤差動態');
  const chips = presetChips(host, PRESETS.map((p, i) => ({ label: p.label, title: p.title, apply: () => applyPreset(i) })));
  const [left, right] = split(host);
  const armCv = new Canvas2D(left, { aspect: 1.45, world: { xmin: -1.25, xmax: 1.25, ymin: -1.15, ymax: 1.25 }, overlay: true });
  armCv.overlay.style.whiteSpace = 'pre';
  const rootWrap = el('div');
  const previewWrap = el('div', { style: { display: 'none' } });
  right.append(rootWrap, previewWrap);
  const rootCv = new Canvas2D(rootWrap, { aspect: 1.45, world: { xmin: -8, xmax: 2, ymin: -6, ymax: 6 }, overlay: true });
  rootCv.overlay.style.whiteSpace = 'pre';
  // The preview canvas is created on first use: a Canvas2D built while hidden would get its
  // height inside its own ResizeObserver callback, which browsers report as an RO loop.
  const previewHolder = el('div');
  previewWrap.appendChild(previewHolder);
  let previewCv: Canvas2D | null = null;

  const panel = controlPanel(host, { compact: true });
  // One flex row for the two selects and the on/off switches keeps the panel to three rows.
  const topRow = row(panel);
  const modeSel = select<InputMode>(topRow, {
    options: [{ value: 'torque', label: '力矩輸入（二階）' }, { value: 'velocity', label: '速度輸入（一階）' }],
    value: st.mode,
    onChange: (v) => { st.mode = v; syncControls(); touched(); },
  });
  const refSel = select<RefKind>(topRow, {
    options: [{ value: 'step', label: '階躍 θ_d（可拖曳）' }, { value: 'ramp', label: '斜坡 θ_d = 0.5t' }, { value: 'sine', label: '正弦 θ_d = 0.5 sin(0.8πt)' }],
    value: st.ref,
    onChange: (v) => { st.ref = v; touched(); },
  });
  modeSel.el.title = '輸入型態';
  refSel.el.title = '參考輸入 θ_d(t)';
  const tGrav = toggle(topRow, { label: '重力負載', title: '加入 mgr·cosθ 的重力負載（mgr = 3 N·m）', value: st.gravity, onChange: (v) => { st.gravity = v; touched(); } });
  const tAw = toggle(topRow, { label: '抗飽和', title: 'anti-windup：飽和且誤差仍推向同側時暫停積分', value: st.antiWindup, onChange: (v) => { st.antiWindup = v; touched(); } });
  const tFF = toggle(topRow, { label: '速度前饋 dθ_d/dt', value: st.velFF, onChange: (v) => { st.velFF = v; touched(); } });
  const sKp = slider(panel, {
    label: 'Kp', min: 0, max: KP_MAX.torque, step: 0.5, value: st.kpT,
    format: (v) => `${fmt(v, 1)} ${st.mode === 'torque' ? 'N·m/rad' : '1/s'}`,
    onInput: (v) => { if (st.mode === 'torque') st.kpT = v; else st.kpV = v; touched(); },
  });
  const sKi = slider(panel, {
    label: 'Ki', min: 0, max: 100, step: 0.5, value: st.kiT,
    format: (v) => `${fmt(v, 1)} ${st.mode === 'torque' ? 'N·m/(rad·s)' : '1/s²'}`,
    onInput: (v) => { if (st.mode === 'torque') st.kiT = v; else st.kiV = v; touched(); },
  });
  const sKd = slider(panel, {
    label: 'Kd', min: 0, max: 15, step: 0.1, value: st.kdT, unit: 'N·m·s/rad',
    onInput: (v) => { st.kdT = v; touched(); },
  });
  const sDist = slider(panel, {
    label: '擾動 τ_dist', min: -4, max: 4, step: 0.1, value: st.tauDist, unit: 'N·m',
    onInput: (v) => { st.tauDist = v; touched(); },
  });
  const sMax = slider(panel, {
    label: '上限 τmax', min: 1, max: TAU_INF, step: 0.5, value: TAU_INF,
    format: (v) => (v >= TAU_INF ? '∞（理想）' : `${fmt(v, 1)} N·m`),
    onInput: (v) => { st.tauMax = v >= TAU_INF ? Infinity : v; touched(); },
  });
  const pbRow = row(panel);
  const pb = playback(pbRow, {
    duration: T_END,
    playing: true,
    onPlay: () => { if (tCur >= T_END - 1e-9) tCur = 0; loop.start(); },
    onPause: () => loop.stop(),
    onReset: () => { tCur = 0; frame(); },
    onSpeed: (s) => { loop.speed = s; },
    onScrub: (t) => { tCur = t; frame(); },
  });

  const errPlot = new Plot2D(host, { title: '誤差 θ_e(t)', yLabel: 'θ_e (rad)', series: [{ name: 'θ_e' }], xRange: [0, T_END], height: 108 });
  const uPlot = new Plot2D(host, { title: '致動指令', xLabel: 't (s)', yLabel: 'τ (N·m)', series: [{ name: 'τ' }], xRange: [0, T_END], height: 98 });
  const group = plotGroup([errPlot, uPlot]);

  const tableWrap = el('div', { class: 'md', style: { fontSize: '0.78rem', overflowX: 'auto' } });
  host.appendChild(tableWrap);
  const table = el('table', { style: { margin: '0.2rem 0', width: '100%' } });
  tableWrap.appendChild(table);
  const caption = el('div', { class: 'mr-note', style: { margin: '0.15rem 0' } });
  const warning = el('div', { class: 'mr-note', style: { margin: '0.15rem 0', color: 'var(--danger)' } });
  tableWrap.append(caption, warning);

  // ---- helpers ------------------------------------------------------------
  const gains = () => (st.mode === 'torque' ? { kp: st.kpT, ki: st.kiT, kd: st.kdT } : { kp: st.kpV, ki: st.kiV, kd: 0 });
  const reference = (): Reference => (st.ref === 'step' ? { kind: 'step', amp: st.stepAmp, freq: 0 }
    : st.ref === 'ramp' ? { kind: 'ramp', amp: RAMP_C, freq: 0 } : { kind: 'sine', amp: SINE_A, freq: SINE_F });
  const trialOpts = (over: Partial<TrialOptions> = {}): TrialOptions => ({
    mode: st.mode, plant: PLANT, gains: gains(), ref: reference(), theta0: 0, gravity: st.gravity, tauDist: st.tauDist,
    tauMax: st.tauMax, antiWindup: st.antiWindup, velFeedforward: st.velFF, T: T_END, ...over,
  });
  const canDragPole = () => !st.preview && gains().ki === 0;

  function sampleAt(t: number): { theta: number; thd: number; err: number; u: number; sat: boolean } {
    const n = trial.t.length;
    if (n < 2) return { theta: trial.theta[0], thd: trial.thd[0], err: trial.err[0], u: trial.u[0], sat: false };
    const dt = trial.t[1] - trial.t[0];
    const x = Math.max(0, Math.min(t, trial.t[n - 1])) / dt;
    const i = Math.min(n - 1, Math.floor(x));
    const j = Math.min(n - 1, i + 1);
    const f = Math.min(1, x - i);
    const lerp = (a: number[]) => a[i] + (a[j] - a[i]) * f;
    const u = trial.u[i];
    return { theta: lerp(trial.theta), thd: lerp(trial.thd), err: lerp(trial.err), u, sat: Math.abs(trial.uCmd[i] - u) > 1e-6 };
  }

  function computeLocus(): Complex[] {
    const g = gains();
    const kmax = KP_MAX[st.mode];
    const pts: Complex[] = [];
    const N = 90;
    for (let k = 0; k <= N; k++) {
      const kp = kmax * (k / N) * (k / N);
      for (const z of polyRoots(charPoly(st.mode, PLANT, { kp, ki: g.ki, kd: g.kd }))) pts.push(z);
    }
    return pts;
  }

  /**
   * Frame the s-plane around the current roots, the Kp = 0 roots and the near part of the Kp
   * locus (the far part may leave the view). The box matches the canvas aspect and the extra
   * width goes to the stable (left) side, so the unstable half-plane stays a narrow strip
   * instead of filling half the canvas.
   */
  function fitRootView(): void {
    const g = gains();
    const kpCap = st.mode === 'velocity' ? KP_MAX.velocity : Math.max(1.5 * g.kp, 40);
    const pts = [
      ...roots,
      ...polyRoots(charPoly(st.mode, PLANT, { ...g, kp: 0 })),
      ...polyRoots(charPoly(st.mode, PLANT, { ...g, kp: kpCap })),
    ];
    let xL = -1, xR = 0, Y = 1.5;
    for (const z of pts) {
      if (!Number.isFinite(z.re) || !Number.isFinite(z.im)) continue;
      xL = Math.min(xL, z.re); xR = Math.max(xR, z.re); Y = Math.max(Y, Math.abs(z.im));
    }
    xL = Math.max(xL, -80); xR = Math.min(xR, 20); Y = Math.min(Y, 60);
    const aspect = rootCv.width > 0 && rootCv.height > 0 ? rootCv.width / rootCv.height : 1.45;
    const W = Math.max((xR - xL) / 0.72, 2.3 * Y * aspect);
    const x1 = xR + 0.18 * W;
    const H = W / aspect;
    rootCv.setWorld({ xmin: x1 - W, xmax: x1, ymin: -H / 2, ymax: H / 2 });
  }

  /** Steady-state error predicted by the linear model, or null when it does not apply. */
  function predictedEss(): number | null {
    const g = gains();
    if (st.ref === 'sine') return null;
    // Along a ramp the gravity load keeps changing, so no constant steady state exists (even with I).
    if (st.mode === 'torque' && st.gravity && st.ref !== 'step') return null;
    if (g.ki > 0) return 0;
    if (st.mode === 'velocity') return st.ref === 'ramp' && !st.velFF ? (g.kp > 0 ? RAMP_C / g.kp : null) : 0;
    if (!(g.kp > 0)) return null;
    if (st.ref === 'ramp') return (PLANT.b * RAMP_C + st.tauDist) / g.kp;
    // Step: solve K_p e = τ_dist + mgr cos(θ_d − e) by a fixed-point iteration (a contraction when
    // K_p > mgr); report nothing if it has not converged.
    const thd = st.stepAmp;
    const rhs = (e: number) => (st.tauDist + (st.gravity ? PLANT.mgr * Math.cos(thd - e) : 0)) / g.kp;
    let e = 0;
    for (let k = 0; k < 60; k++) e = rhs(e);
    return Math.abs(rhs(e) - e) < 1e-6 ? e : null;
  }

  // ---- recompute (one full trial is cheap: 3000 RK4 steps) ----------------
  function recompute(): void {
    trial = simulateJoint(trialOpts());
    alt = st.mode === 'torque' && trial.saturated && !trial.diverged ? simulateJoint(trialOpts({ antiWindup: !st.antiWindup })) : null;
    roots = polyRoots(charPoly(st.mode, PLANT, gains()));
    locus = computeLocus();
    if (!dragging) fitRootView();
    poleHandle.enabled = canDragPole();
    targetHandle.enabled = st.ref === 'step' && !st.preview;
    updatePlots();
    updateTable();
    rootCv.requestRender();
    frame();
  }
  const scheduleRecompute = (() => {
    let pending = false;
    return () => {
      if (pending) return;
      pending = true;
      requestAnimationFrame(() => { pending = false; if (!destroyed) recompute(); });
    };
  })();

  function touched(): void { chips.clear(); scheduleRecompute(); }

  /** Set series data and empty the unused slots (Plot2D autoscales over every stored array). */
  function setSeries(plot: Plot2D, data: [ArrayLike<number>, ArrayLike<number>][]): void {
    data.forEach(([xs, ys], i) => plot.setXY(i, xs, ys));
    for (let i = data.length; i < 5; i++) plot.setXY(i, [], []);
  }

  function updatePlots(): void {
    const c1 = cssVar('--series-1', '#2563eb'), c4 = cssVar('--series-4', '#d97706'), c5 = cssVar('--series-5', '#7c3aed'), c8 = cssVar('--series-8', '#4b5563');
    const g = gains();
    // --- error plot: simulation, optional theory/envelope, optional anti-windup comparison
    const eSeries: SeriesSpec[] = [{ name: '模擬', color: c1, width: 2.2 }];
    const eData: [number[], number[]][] = [[trial.t, trial.err]];
    const pred = st.mode === 'torque' ? pdPrediction(PLANT, g.kp, g.kd) : null;
    if (st.ref === 'step' && g.ki === 0 && !trial.saturated && !trial.diverged) {
      if (st.mode === 'velocity' && !st.velFF && g.kp > 0) {
        eSeries.push({ name: '理論 e^(−Kp t)', color: c4, dash: [6, 4], width: 1.6 });
        eData.push([trial.t, trial.t.map((t) => st.stepAmp * Math.exp(-g.kp * t))]);
      } else if (st.mode === 'torque' && !st.gravity && pred && pred.zeta < 1 && pred.zeta > 0.02) {
        const ess = st.tauDist / g.kp;
        const a = (st.stepAmp - ess) / Math.sqrt(1 - pred.zeta * pred.zeta);
        const env = trial.t.map((t) => a * Math.exp(-pred.zeta * pred.wn * t));
        eSeries.push({ name: '包絡 e^(−ζωn t)', color: c4, dash: [6, 4], width: 1.4 });
        eData.push([[...trial.t, NaN, ...trial.t], [...env.map((v) => ess + v), NaN, ...env.map((v) => ess - v)]]);
      }
    }
    if (alt) {
      eSeries.push({ name: st.antiWindup ? '對照：未開抗飽和' : '對照：開啟抗飽和', color: c8, dash: [5, 4], width: 1.6 });
      eData.push([alt.t, alt.err]);
    }
    errPlot.setOptions({ series: eSeries });
    setSeries(errPlot, eData);
    // markers: steady-state level, settling time, overshoot peak (step input only)
    const marks: Marker[] = [];
    if (st.ref === 'step' && !trial.diverged) {
      const m = stepMetrics(trial.t, trial.err);
      marks.push({ y: m.ess, color: c5, label: 'e_ss', dash: [3, 3] });
      if (m.ts !== null && m.ts > 0) {
        marks.push({ x: m.ts, color: c5 });
        marks.push({ x: m.ts, y: m.ess + 0.02 * (m.e0 - m.ess), color: c5, label: 't_s' });
      }
      if (m.overshootPct > 0.5) marks.push({ x: m.peakT, y: m.peakE, color: c5, label: `M_p ${fmt(m.overshootPct, 0)}%` });
    }
    group.setMarkers(0, marks);

    // --- actuator plot
    if (st.mode === 'torque') {
      const uSeries: SeriesSpec[] = [{ name: 'τ 實際', color: c5, width: 2 }];
      const uData: [number[], number[]][] = [[trial.t, trial.u]];
      if (g.ki > 0) { uSeries.push({ name: 'I 項 Ki∫θ_e', color: c4, dash: [6, 4], width: 1.8 }); uData.push([trial.t, trial.iTerm]); }
      uPlot.setOptions({ title: '力矩 τ(t)', yLabel: 'τ (N·m)', series: uSeries });
      setSeries(uPlot, uData);
      const lim = st.tauMax;
      group.setMarkers(1, Number.isFinite(lim) ? [{ y: lim, color: cssVar('--danger', '#b91c1c'), label: '±τmax' }, { y: -lim, color: cssVar('--danger', '#b91c1c') }] : []);
    } else {
      const uSeries: SeriesSpec[] = [{ name: 'u = θ̇ 指令', color: c5, width: 2 }];
      const uData: [number[], number[]][] = [[trial.t, trial.u]];
      if (g.ki > 0) { uSeries.push({ name: 'I 項 Ki∫θ_e', color: c4, dash: [6, 4], width: 1.8 }); uData.push([trial.t, trial.iTerm]); }
      uPlot.setOptions({ title: '速度指令 u(t)', yLabel: 'u (rad/s)', series: uSeries });
      setSeries(uPlot, uData);
      group.setMarkers(1, []);
    }
  }

  function updateTable(): void {
    const g = gains();
    const pred = st.mode === 'torque' ? pdPrediction(PLANT, g.kp, g.kd) : null;
    const dash = '—';
    let pMp = dash, pTs = dash, mMp = dash, mTs = dash;
    const essP = predictedEss();
    const pEss = essP === null ? dash : `${fmt(essP, 3)} rad`;
    let mEss = dash;
    let essHead = '穩態誤差 e_ss';
    if (st.ref === 'step') {
      // "Linear prediction" = the exact solution of the linear error equation (no gravity
      // stiffness, no saturation); the 4/(ζωn) rule of thumb is shown in the caption instead.
      if (g.ki === 0) {
        if (st.mode === 'torque' && pred) { pMp = `${fmt(pred.overshootPct, 1)} %`; pTs = Number.isFinite(pred.tsExact) ? `${fmt(pred.tsExact, 2)} s` : '∞'; }
        if (st.mode === 'velocity' && g.kp > 0) { pMp = '0 %'; pTs = `${fmt(Math.log(50) / g.kp, 2)} s`; }
      }
      if (!trial.diverged) {
        const m = stepMetrics(trial.t, trial.err);
        mMp = `${fmt(m.overshootPct, 1)} %`;
        mTs = m.ts === null ? `> ${T_END} s（未安定）` : `${fmt(m.ts, 2)} s`;
        mEss = `${fmt(m.ess, 3)} rad`;
      }
    } else if (st.ref === 'ramp') {
      if (!trial.diverged) mEss = `${fmt(trial.err[trial.err.length - 1], 3)} rad`;
    } else {
      essHead = '誤差振幅（最後 2 s）';
      if (!trial.diverged) mEss = `${fmt(tailAmplitude(trial.t, trial.err, 2), 3)} rad`;
    }
    if (trial.diverged) { mMp = mTs = mEss = '發散'; }
    const pad = { padding: '0.12em 0.5em' };
    const cell = (tag: 'th' | 'td', text: string) => el(tag, { style: pad }, text);
    table.replaceChildren(
      el('thead', {}, el('tr', {}, cell('th', ''), cell('th', '超調 M_p'), cell('th', '安定時間 t_s'), cell('th', essHead))),
      el('tbody', {},
        el('tr', {}, cell('td', '線性預測'), cell('td', pMp), cell('td', pTs), cell('td', pEss)),
        el('tr', {}, cell('td', '模擬量測'), cell('td', mMp), cell('td', mTs), cell('td', mEss))),
    );
    if (st.mode === 'torque') {
      const parts: string[] = [];
      if (!pred) parts.push('Kp = 0：沒有恢復力');
      else if (g.ki === 0) {
        parts.push(`ζ = ${fmt(pred.zeta, 3)}，ωn = ${fmt(pred.wn, 2)} rad/s`);
        if (st.ref === 'step' && Number.isFinite(pred.ts)) parts.push(`粗估 ${pred.zeta < 1 ? '4/(ζωn)' : '4/|慢根|'} = ${fmt(pred.ts, 2)} s`);
        // The root plot and the M_p / t_s predictions use the linear model without gravity.
        if (st.gravity) parts.push('重力使等效剛性變為 Kp − mgr·sinθ');
      } else {
        // With an I term the loop is third order: ζ, ωn of the PD part no longer describe it.
        parts.push(`PD 部分 ζ = ${fmt(pred.zeta, 2)}、ωn = ${fmt(pred.wn, 2)}`);
        const dom = dominantRoot(roots);
        if (dom && dom.re < 0) parts.push(Math.abs(dom.im) > 1e-9 ? `主導根 ζ ≈ ${fmt(-dom.re / Math.hypot(dom.re, dom.im), 2)}` : `主導根 s = ${fmt(dom.re, 2)}（實根）`);
      }
      if (g.ki > 0) parts.push(`穩定需 Ki < (b+Kd)Kp/M = ${fmt(pidKiBound(PLANT, g.kp, g.kd), 1)}`);
      caption.textContent = parts.join('；');
    } else {
      caption.textContent = g.kp > 0 ? `時間常數 1/Kp = ${fmt(1 / g.kp, 3)} s（理想速度源：不受擾動力矩影響）` : 'Kp = 0：誤差不會被修正';
    }
    const rhp = roots.some((z) => z.re > 1e-9);
    warning.textContent = trial.diverged || rhp ? '⚠ 根在右半平面：閉迴路不穩定、誤差發散'
      : trial.saturated ? '⚠ 力矩曾飽和：飽和時迴路等於斷開，線性預測失效' : '';
  }

  // ---- drawing ------------------------------------------------------------
  function curvedArrow(c: Canvas2D, r: number, a0: number, span: number, color: string): Pt | null {
    if (Math.abs(span) < 0.04) return null;
    const a1 = a0 + span;
    c.arc([0, 0], r, Math.min(a0, a1), Math.max(a0, a1), { color, width: 2.5 });
    const back = a1 - Math.sign(span) * Math.min(0.3, Math.abs(span) * 0.5);
    const end: Pt = [r * Math.cos(a1), r * Math.sin(a1)];
    c.arrow([r * Math.cos(back), r * Math.sin(back)], end, { color, width: 2.5, head: 9 });
    return end;
  }

  armCv.draw = (c) => {
    const fg = cssVar('--fg'), muted = cssVar('--fg-muted'), accent = cssVar('--accent'), warn = cssVar('--warn');
    const s = sampleAt(tCur);
    c.grid(0.5);
    c.line([0, 0], [1.2, 0], { color: muted, width: 1, dash: [4, 4] });
    c.text([1.22, -0.15], 'θ = 0', { color: muted, align: 'right' });
    // mount
    c.polygon([[-0.11, -0.14], [0.11, -0.14], [0, 0]], { fill: cssVar('--grid-strong'), color: cssVar('--grid-strong'), width: 1 });
    c.line([-0.17, -0.14], [0.17, -0.14], { color: muted, width: 2 });
    // reference (ghost)
    const pd: Pt = [Math.cos(s.thd), Math.sin(s.thd)];
    c.line([0, 0], pd, { color: accent, width: 8, alpha: 0.2 });
    c.circle(pd, 0.085, { color: accent, width: 1.5, dash: [3, 3] });
    if (!targetHandle.enabled) c.text(pd, 'θ_d', { color: accent, dx: 16 * Math.cos(s.thd) + 4, dy: -16 * Math.sin(s.thd) });
    // error arc
    const r = 0.46;
    if (Math.abs(s.err) > 2e-3) c.arc([0, 0], r, Math.min(s.theta, s.thd), Math.max(s.theta, s.thd), { color: warn, width: 3.5 });
    // link
    const p: Pt = [Math.cos(s.theta), Math.sin(s.theta)];
    if (st.mode === 'torque' && st.gravity) {
      const com: Pt = [0.6 * p[0], 0.6 * p[1]];
      c.arrow(com, [com[0], com[1] - 0.32], { color: muted, width: 1.8, head: 8 });
      c.text([com[0], com[1] - 0.4], 'mg', { color: muted });
    }
    c.line([0, 0], p, { color: muted, width: 10 });
    c.disk(p, 0.075, { fill: fg });
    c.point([0, 0], 7, { color: cssVar('--bg-elev') });
    c.circle([0, 0], 7 / c.scale, { color: fg, width: 2 });
    // actuator command as a curved arrow at the joint
    const col = s.sat ? cssVar('--danger') : cssVar('--series-5');
    const span = st.mode === 'torque' ? Math.max(-1, Math.min(1, s.u / 20)) * 0.75 * Math.PI : Math.max(-1, Math.min(1, s.u / 2)) * 0.75 * Math.PI;
    const tip = curvedArrow(c, 0.24, s.theta, span, col);
    if (tip) c.text(tip, st.mode === 'torque' ? 'τ' : 'u', { color: col, dx: 10 * Math.sign(span), dy: -8 });
    if (Math.abs(s.err) > 0.25) {
      const mid = (s.theta + s.thd) / 2;
      c.text([0.72 * Math.cos(mid), 0.72 * Math.sin(mid)], 'θ_e', { color: warn, bg: cssVar('--sim-bg') });
    }
    const deg = (x: number) => `${fmt((x * 180) / Math.PI, 1)}°`;
    c.setOverlayText(`t = ${fmt(Math.min(tCur, T_END), 2)} s\nθ_e = ${deg(s.err)}`);
  };

  function cross(c: Canvas2D, z: Complex, color: string): void {
    const d = 6 / c.scale;
    c.line([z.re - d, z.im - d], [z.re + d, z.im + d], { color, width: 2.6 });
    c.line([z.re - d, z.im + d], [z.re + d, z.im - d], { color, width: 2.6 });
  }

  rootCv.draw = (c) => {
    const v = c.visible();
    const muted = cssVar('--fg-muted'), accent = cssVar('--accent'), danger = cssVar('--danger');
    c.polygon([[0, v.ymin], [v.xmax, v.ymin], [v.xmax, v.ymax], [0, v.ymax]], { fill: cssVar('--danger-soft'), width: 0 });
    const stp = niceStep(Math.max(v.xmax - v.xmin, v.ymax - v.ymin));
    c.grid(stp);
    const small = `10px ${cssVar('--font') || 'sans-serif'}`;
    for (let x = Math.ceil(v.xmin / stp) * stp; x <= v.xmax; x += stp) if (Math.abs(x) > 1e-9) c.text([x, 0], fmt(x, stp < 1 ? 1 : 0), { color: muted, dy: 9, font: small });
    for (let y = Math.ceil(v.ymin / stp) * stp; y <= v.ymax; y += stp) {
      // skip labels at the top/bottom edge, where the 'Im' and '不穩定區' labels sit
      const sy = c.toScreen([0, y])[1];
      if (Math.abs(y) > 1e-9 && sy > 18 && sy < c.height - 18) c.text([0, y], fmt(y, stp < 1 ? 1 : 0), { color: muted, dx: 4, align: 'left', font: small });
    }
    c.text([v.xmax, 0], 'Re', { color: muted, dx: -12, dy: -9 });
    c.text([0, v.ymax], 'Im', { color: muted, dx: 12, dy: 10 });
    c.text([v.xmax, v.ymin], '不穩定區', { color: danger, dx: -6, dy: -10, align: 'right', font: small });
    // constant-damping rays ζ = 0.7 and the t_s ≈ 1 s line (Re = −4)
    const L = 3 * Math.max(-v.xmin, v.ymax);
    const phi = Math.acos(0.7);
    for (const sg of [1, -1]) c.line([0, 0], [-L * Math.cos(phi), sg * L * Math.sin(phi)], { color: muted, width: 1, dash: [5, 5], alpha: 0.7 });
    const lab = 0.7 * Math.min(-v.xmin, -v.ymin / Math.tan(phi));
    c.text([-lab, -lab * Math.tan(phi)], 'ζ = 0.7', { color: muted, dx: -8, dy: 8, align: 'right', font: small });
    if (v.xmin < -4.5) {
      c.line([-4, v.ymin], [-4, v.ymax], { color: muted, width: 1, dash: [2, 4], alpha: 0.7 });
      c.text([-4, v.ymin], 't_s ≈ 1 s', { color: muted, dy: -10, dx: -4, align: 'right', font: small });
    }
    for (const z of locus) if (Number.isFinite(z.re)) c.point([z.re, z.im], 1.7, { color: accent, alpha: 0.35 });
    for (const z of roots) cross(c, z, z.re > 1e-9 ? danger : accent);
    const g = gains();
    const pred = st.mode === 'torque' ? pdPrediction(PLANT, g.kp, g.kd) : null;
    // With an I term the PD-part ζ, ωn no longer describe the loop: report the dominant root.
    const dom = g.ki > 0 ? dominantRoot(roots) : null;
    const line2 = dom ? (Math.abs(dom.im) > 1e-9 && dom.re < 0 ? `主導根 ζ ≈ ${fmt(-dom.re / Math.hypot(dom.re, dom.im), 2)}` : `主導根 s = ${fmt(dom.re, 2)}`)
      : pred ? `ζ = ${fmt(pred.zeta, 2)}  ωn = ${fmt(pred.wn, 2)}`
        : st.mode === 'velocity' ? `s = −Kp = ${fmt(-g.kp, 2)}` : '';
    c.setOverlayText(`× 特徵根　· Kp↑ 的軌跡${line2 ? `\n${line2}` : ''}`);
  };

  // Draggable dominant root (PD or P only): moving it solves for the gains.
  const poleHandle: Handle = rootCv.addHandle({
    get: () => {
      const up = roots.filter((z) => z.im >= -1e-12);
      if (!up.length) return [0, 0];
      const cpx = up.find((z) => z.im > 1e-9);
      const z = cpx ?? up.reduce((a, b) => (b.re > a.re ? b : a));
      return [z.re, Math.max(0, z.im)];
    },
    set: (p) => {
      if (st.mode === 'velocity') {
        st.kpV = Math.max(0, Math.min(KP_MAX.velocity, Math.round(-Math.min(0, p[0]) * 10) / 10));
        sKp.set(st.kpV, { silent: true });
      } else {
        const re = Math.min(p[0], -0.05), im = Math.max(0, p[1]);
        const g = pdGainsFromPole(PLANT, re, im);
        st.kpT = Math.max(0, Math.min(KP_MAX.torque, Math.round(g.kp * 2) / 2));
        st.kdT = Math.max(0, Math.min(15, Math.round(g.kd * 10) / 10));
        sKp.set(st.kpT, { silent: true });
        sKd.set(st.kdT, { silent: true });
      }
      touched();
    },
    onStart: () => { dragging = true; },
    onEnd: () => { dragging = false; scheduleRecompute(); },
    label: '拖曳根',
    radius: 9,
  });

  // Draggable target for step inputs.
  const targetHandle: Handle = armCv.addHandle({
    get: () => [Math.cos(st.stepAmp), Math.sin(st.stepAmp)],
    set: (p) => {
      const a = Math.atan2(p[1], p[0]);
      st.stepAmp = Math.max(-2.8, Math.min(2.8, a));
      touched();
    },
    label: 'θ_d',
    radius: 9,
  });

  // ---- 2R preview (step 7) ---------------------------------------------------
  let thdPrev: [number, number] = [0.2, 0.6];
  let sag = pdSag2R(thdPrev, PREVIEW_KP);
  function drawPreview(c: Canvas2D): void {
    const muted = cssVar('--fg-muted'), warn = cssVar('--warn');
    c.grid(0.5);
    c.circle([0, 0], 2, { color: cssVar('--grid-strong'), width: 1, dash: [4, 4] });
    drawPlanarArm(c, [1, 1], thdPrev, { alpha: 0.28, linkWidth: 7 });
    const pts = drawPlanarArm(c, [1, 1], sag.theta, { linkWidth: 8 });
    // Sag arcs: at joint 1 between the commanded and the actual link-1 direction; at joint 2
    // between where link 2 should point (θ1 + θd2) and where it points (θ1 + θ2).
    const arcs: [Pt, number, number][] = [
      [pts[0], sag.theta[0], thdPrev[0]],
      [pts[1], sag.theta[0] + sag.theta[1], sag.theta[0] + thdPrev[1]],
    ];
    for (const [p, a, b] of arcs) {
      if (Math.abs(b - a) < 1e-3) continue;
      const r = 0.38;
      c.arc(p, r, Math.min(a, b), Math.max(a, b), { color: warn, width: 3 });
      c.line(p, [p[0] + (r + 0.08) * Math.cos(b), p[1] + (r + 0.08) * Math.sin(b)], { color: warn, width: 1, dash: [3, 3] });
      const m = (a + b) / 2;
      c.text([p[0] + (r + 0.2) * Math.cos(m), p[1] + (r + 0.2) * Math.sin(m)], `${fmt(((b - a) * 180) / Math.PI, 1)}°`, { color: warn, bg: cssVar('--sim-bg') });
    }
    const v = c.visible();
    c.arrow([v.xmin + 0.25, v.ymin + 0.75], [v.xmin + 0.25, v.ymin + 0.3], { color: muted, width: 2, head: 9 });
    c.text([v.xmin + 0.25, v.ymin + 0.85], 'g', { color: muted });
    c.setOverlayText(`獨立 PD，Kp = (${PREVIEW_KP[0]}, ${PREVIEW_KP[1]})\n淡色＝目標　橘色弧＝下垂角`);
  }
  function ensurePreview(): void {
    if (previewCv) return;
    previewCv = new Canvas2D(previewHolder, { aspect: 1.45, world: { xmin: -1.15, xmax: 2.15, ymin: -1.35, ymax: 1.75 }, overlay: true });
    previewCv.overlay.style.whiteSpace = 'pre';
    previewCv.draw = drawPreview;
    previewCv.addHandle({
    get: () => { const fk = planarFK([1, 1], thdPrev); return [fk.tip[0], fk.tip[1]]; },
    set: (p) => {
      let [x, y] = p;
      const r = Math.hypot(x, y);
      if (r > 1.97) { x *= 1.97 / r; y *= 1.97 / r; }
      if (r < 0.2) return;
      const ik = planar2RIK(1, 1, x, y);
      const sol = ik.solutions.find((s) => s.theta2 >= 0) ?? ik.solutions[0];
      if (!sol) return;
      thdPrev = [sol.theta1, sol.theta2];
      sag = pdSag2R(thdPrev, PREVIEW_KP);
    },
    label: '目標',
    radius: 9,
    });
  }

  // ---- controls sync ----------------------------------------------------------
  function syncControls(): void {
    const torque = st.mode === 'torque';
    modeSel.set(st.mode, { silent: true });
    refSel.set(st.ref, { silent: true });
    sKp.setRange(0, KP_MAX[st.mode], torque ? 0.5 : 0.1);
    sKp.set(torque ? st.kpT : st.kpV, { silent: true });
    sKi.setRange(0, torque ? 100 : 50, 0.5);
    sKi.set(torque ? st.kiT : st.kiV, { silent: true });
    sKd.set(st.kdT, { silent: true });
    sDist.set(st.tauDist, { silent: true });
    sMax.set(Number.isFinite(st.tauMax) ? st.tauMax : TAU_INF, { silent: true });
    tGrav.set(st.gravity, { silent: true });
    tAw.set(st.antiWindup, { silent: true });
    tFF.set(st.velFF, { silent: true });
    sKd.disabled = !torque; sDist.disabled = !torque; sMax.disabled = !torque;
    for (const c of [sKd.el, sDist.el, sMax.el]) c.style.opacity = torque ? '' : '0.45';
    tGrav.el.style.display = torque ? '' : 'none';
    tAw.el.style.display = torque ? '' : 'none';
    tFF.el.style.display = torque ? 'none' : '';
    rootWrap.style.display = st.preview ? 'none' : '';
    previewWrap.style.display = st.preview ? '' : 'none';
    if (st.preview) ensurePreview();
  }

  function applyPreset(i: number): void {
    Object.assign(st, BASE, PRESETS[i].s);
    syncControls();
    tCur = 0;
    recompute();
    play();
  }
  /** Run the trial animation once from the current time; it stops on the last frame. */
  function play(): void {
    pb.setPlaying(true, { silent: true });
    loop.start();
  }

  // ---- animation ------------------------------------------------------------
  // Moving the plot cursor repaints both plots, so during playback it is updated at ~15 Hz.
  let lastCursor = -Infinity;
  function frame(): void {
    const t = Math.min(tCur, T_END);
    armCv.requestRender();
    const now = performance.now();
    if (!loop.running || t >= T_END || now - lastCursor > 66) { group.setCursor(t); lastCursor = now; }
    pb.setTime(t);
  }
  const loop = createLoop((dt) => {
    tCur = Math.min(T_END, tCur + dt);
    frame();
    if (tCur >= T_END) { loop.stop(); pb.setPlaying(false, { silent: true }); }
  });

  // ---- step focus ----------------------------------------------------------
  const focusSets: HTMLElement[][] = [
    [errPlot.el, tableWrap],
    [rootCv.el, sKp.el],
    [rootCv.el, sKd.el],
    [errPlot.el, sKp.el, sDist.el],
    [rootCv.el, sKi.el],
    [uPlot.el, tAw.el],
    [previewWrap],
  ];
  let focused: HTMLElement[] = [];
  function onStep(i: number): void {
    const k = Math.max(0, Math.min(PRESETS.length - 1, i));
    chips.activate(k);
    focused.forEach((f) => f.classList.remove('mr-focus'));
    focused = focusSets[k] ?? [];
    focused.forEach((f) => f.classList.add('mr-focus'));
  }

  const unsubTheme = ctx.onThemeChange(() => { updatePlots(); armCv.requestRender(); rootCv.requestRender(); previewCv?.requestRender(); });
  syncControls();
  onStep(ctx.currentStep);

  return {
    onStep,
    destroy() {
      destroyed = true;
      loop.stop();
      unsubTheme();
      armCv.destroy(); rootCv.destroy(); previewCv?.destroy();
      errPlot.destroy(); uPlot.destroy();
    },
  };
}

const lesson: Lesson = { meta, steps, quiz, mount };
export default lesson;
