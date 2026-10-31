import type { Lesson, LessonContext, LessonHandle, LessonStep, QuizQuestion } from '@/types';
import {
  el, cssVar, fmt, controlPanel, row, slider, select, toggle, button, readout, playback, simTitle, note,
  presetChips, split, Canvas2D, Plot2D, plotGroup, createLoop, type Pt, type Marker, type SeriesSpec, type World,
} from '@/ui';
import { clamp } from '@/mr';
import { meta } from './meta';
import {
  KINDS, makeProfile, sampleTimes, timeAtS, flexResponse, flexAt, minTime,
  type ScalingKind, type Profile, type Limits, type FlexResult,
} from './scaling';

const steps: LessonStep[] = [
  {
    title: '路徑不是軌跡',
    md: `**路徑**（path）只回答「經過哪些位置」：把幾何曲線寫成 $\\theta(s)$，參數 $s\\in[0,1]$，$s=0$ 是起點、$s=1$ 是終點，跟時間無關。**軌跡**（trajectory）再回答「什麼時候到哪裡」：

$$\\theta(t)=\\theta\\big(s(t)\\big),\\qquad s(0)=0,\\ s(T)=1 .$$

這個把時間映到路徑參數的函數 $s(t)$ 叫做**時間縮放（time scaling）**，通常取單調不減（不走回頭路）。同一條路徑配上不同的 $s(t)$，走過的地方一模一樣，差別只在快慢怎麼分配。

右圖的單關節從 $0^\\circ$ 轉到 $135^\\circ$：上方的 $s$ 條只管「到哪裡」，下方的 $t$ 條只管「何時」，連接兩條的細線就是 $s(t)$。`,
    simHint: '先拖曳上方 s 條上的圓點（純幾何、沒有時間）；再按「▶ 播放」，注意弧線上「等時間間隔」的刻度在兩端擠在一起。',
  },
  {
    title: '鏈鎖律：速度與加速度從哪裡來',
    md: `對 $\\theta(s(t))$ 微分，鏈鎖律給出

$$\\dot\\theta=\\theta'(s)\\,\\dot s,\\qquad \\ddot\\theta=\\theta''(s)\\,\\dot s^{2}+\\theta'(s)\\,\\ddot s .$$

$\\theta'(s)$、$\\theta''(s)$ 只由路徑的形狀決定；$\\dot s$、$\\ddot s$ 只由時間縮放決定。本課的路徑是關節空間的直線 $\\theta(s)=\\theta_{\\rm start}+s\\,\\Delta\\theta$，所以 $\\theta'=\\Delta\\theta$ 是常數、$\\theta''=0$：關節的速度與加速度曲線，就是 $\\dot s$、$\\ddot s$ 乘上同一個常數。

另一個好用的規律：對 $s=f(t/T)$ 這種「形狀固定」的縮放（例如三次、五次），總時間 $T$ 變成 $k$ 倍時，$\\dot s$ 的峰值變成 $1/k$ 倍，$\\ddot s$ 的峰值變成 $1/k^{2}$ 倍。`,
    simHint: '播放時盯著面板上 dθ/dt = θ′ṡ 的讀數；再把 T 從 2.5 s 拉到 5 s，比較 ṡ 與 s̈ 圖的高度（約變成 1/2 與 1/4）。',
  },
  {
    title: '三次與五次多項式',
    md: `**三次多項式**（cubic）有 4 個係數，剛好滿足兩端位置與兩端速度為零：

$$s(t)=3\\tau^{2}-2\\tau^{3},\\qquad \\tau=t/T .$$

代價是加速度在兩端**跳變**：起動瞬間 $\\ddot s$ 從 0 直接跳到 $6/T^{2}$。**五次多項式**（quintic）再要求兩端加速度為零，得到 $s=10\\tau^{3}-15\\tau^{4}+6\\tau^{5}$，加速度從 0 平滑爬升。

平滑不是免費的：五次的峰值速度是 $1.875/T$，比三次的 $1.5/T$ 高 25%；峰值加速度兩者相近（$5.77/T^{2}$ 對 $6/T^{2}$），只是五次的最大值出現在途中（$\\tau\\approx0.21$ 與 $0.79$），而不是一起動就出現。`,
    simHint: '在 s̈ 圖比較藍線（三次）與紅線（五次）在 t = 0 的樣子：一條垂直跳起、一條從零爬升；再看 ṡ 圖誰的峰值較高。',
  },
  {
    title: '梯形速度曲線',
    md: `馬達通常有速度上限 $v$ 與加速度上限 $a$（這裡以 $\\dot s$、$\\ddot s$ 表示）。最快的走法是：以 $a$ 加速 → 以 $v$ 等速 → 以 $a$ 減速，$\\dot s$ 畫出來是梯形。只有 $v^{2}/a\\le 1$ 時才來得及加速到 $v$，此時最短時間為

$$T_{\\min}=\\frac{a+v^{2}}{v\\,a}.$$

若 $v^{2}/a>1$，還沒到 $v$ 就得開始減速，梯形退化成**三角形**，$T_{\\min}=2/\\sqrt a$。要求的 $T$ 比 $T_{\\min}$ 短就**不可行**。模擬裡的梯形一律用 $a_{\\max}$ 加速，需要的巡航速度由 $T$ 決定；四個轉角的加速度都是跳變。`,
    simHint: '把 T 慢慢調小直到出現「不可行」警告，再按「T ← T_min」；接著點「三角形」情境，看等速段消失。',
  },
  {
    title: 'S 曲線與 jerk',
    md: `加速度跳變時，jerk（$\\dddot s$）是一個無限大的脈衝，驅動力也在一瞬間換成另一個值（階躍）。若關節後面接著有彈性的東西（皮帶、減速機、細長手臂），結構會在自己的自然頻率上**殘留振動**。

**S 曲線**把加速度的每一次跳變換成斜率 $\\pm j_{\\max}$ 的斜坡（共 7 段）：加速度連續、jerk 有上限，代價是在相同 $a_{\\max}$ 下比梯形慢。模擬中的柔性負載經由彈簧跟著關節走：

$$\\ddot q=\\omega_n^{2}(s-q)+2\\zeta\\omega_n(\\dot s-\\dot q).$$

預設的斜坡長 $a_{\\max}/j_{\\max}\\approx0.33$ s 恰好等於負載的週期 $1/f_n$，每段斜坡激起的振盪在斜坡結束時剛好抵銷，所以殘餘振動近乎零；換一個 $f_n$，S 曲線通常仍比梯形小，但不會是零。`,
    simHint: '在「柔性負載的振動」分頁比較梯形（綠）與 S 曲線（橘）結束後的晃動，再到「圖表」看兩者的 jerk；把 j_max 從 3 調到 10（T_min 變短、振動變大），或把 f_n 改成 2.5 Hz。',
  },
  {
    title: '選哪一個？以及下一課',
    md: `| 時間縮放 | 優點 | 缺點 |
|---|---|---|
| 三次 | 公式最簡單 | 兩端加速度跳變 |
| 五次 | 加速度連續、平順 | 峰值速度高 25% |
| 梯形 | 在 $v$、$a$ 限制下最快 | 加速度跳變（jerk 無限大） |
| S 曲線 | jerk 有界、振動小 | 參數多、比梯形稍慢 |

挑選的依據是限制條件：結構夠硬、要最快就用梯形；對振動敏感就用五次或 S 曲線。注意殘餘振動也取決於動作時間與自然週期的比例，偶爾會剛好互相抵消，所以要以「jerk 有界」作為穩健的設計原則。

下一課換問另一半：路徑 $\\theta(s)$ 本身怎麼選——關節空間直線、螺旋，或笛卡兒直線。`,
    simHint: '按「全部比較」，讀下方表格：在同一組 v_max、a_max 下，誰的最短 T 最小？誰的殘餘振動最小？',
  },
];

const quiz: QuizQuestion[] = [
  {
    question: '以 $s$ 計，$v_{\\max}=1\\ \\text{s}^{-1}$、$a_{\\max}=2\\ \\text{s}^{-2}$。梯形時間縮放的最短時間與形狀為何？',
    options: [
      '$T=1$ s，三角形（沒有等速段）',
      '$T=1.5$ s，中間有 0.5 s 的等速段',
      '$T=1.5$ s，三角形',
      '$T=2$ s，中間有 1 s 的等速段',
    ],
    answer: 1,
    explanation: '$v^2/a=0.5\\le1$，來得及加速到 $v$，所以是有等速段的梯形。$T=(a+v^2)/(va)=(2+1)/2=1.5$ s；加速與減速各花 $v/a=0.5$ s，剩下 $1.5-1=0.5$ s 等速。',
  },
  {
    question: '同一條路徑 $\\theta(s)$，把時間縮放從三次換成梯形（$T$ 不變）。下列敘述何者正確？',
    options: [
      '末端走過的幾何路線改變了',
      '路線不變；每一點的速度方向也不變，只有速度大小的分配改變',
      '路線不變，但速度方向會改變',
      '因為 $T$ 不變，各時刻的速度也完全不變',
    ],
    answer: 1,
    explanation: '路線由 $\\theta(s)$ 決定，與 $s(t)$ 無關。由鏈鎖律 $\\dot\\theta=\\theta\'(s)\\dot s$，方向來自路徑的切線 $\\theta\'(s)$，$\\dot s\\ge0$ 只是純量倍率，所以只改變大小與時間分配。',
  },
  {
    question: '為什麼加速度不連續（例如梯形的轉角）容易激發機械振動？',
    options: [
      '因為此時速度也不連續',
      '加速度跳變代表驅動力突然改變（jerk 是脈衝），對有彈性的結構就像猛然推一把，會激起它在自然頻率上的振盪',
      '因為位置在那一刻會跳一下',
      '只有總時間很長時才會發生',
    ],
    answer: 1,
    explanation: '力正比於加速度，加速度跳變等於施加一個階躍力；階躍含有各種頻率的成分，其中落在結構自然頻率附近的部分會被放大，運動結束後仍殘留振動。速度與位置在梯形中都是連續的；把 jerk 限制住（S 曲線）就是把階躍變成斜坡。',
  },
  {
    question: '三次時間縮放把總時間 $T$ 減半，最大 $|\\ddot s|$ 會變成原來的幾倍？',
    options: ['$1/2$ 倍', '2 倍', '4 倍', '8 倍'],
    answer: 2,
    explanation: '$s(t)=f(t/T)$，每對時間微分一次就多一個 $1/T$：$\\dot s\\propto1/T$、$\\ddot s\\propto1/T^2$。$T$ 減半，$\\ddot s$ 變成 $2^2=4$ 倍（三次的最大值 $6/T^2$）。',
  },
];

const LABEL: Record<ScalingKind, string> = { cubic: '三次（cubic）', quintic: '五次（quintic）', trapezoid: '梯形（trapezoid）', scurve: 'S 曲線（S-curve）' };
const SHORT: Record<ScalingKind, string> = { cubic: '三次', quintic: '五次', trapezoid: '梯形', scurve: 'S 曲線' };
const colorOf = (k: ScalingKind) => cssVar(`--series-${KINDS.indexOf(k) + 1}`, '#36f');

// Geometry of the single-joint demo (world units of the canvas).
const TH0 = 0;
const TH1 = (135 * Math.PI) / 180;
const DTH = TH1 - TH0;
const LEN = 1.05;
/**
 * Where the path bar (s) and the time bar (t) sit. Wide panes put them to the right of the arm; phones
 * put them under it, so the labels and handles are not squeezed into a strip a few centimetres wide.
 */
interface BarLayout { x0: number; x1: number; ys: number; yt: number; world: World; canvas: { height: number } | { aspect: number } }
const WIDE: BarLayout = { x0: 1.8, x1: 3.4, ys: 1.0, yt: 0.18, world: { xmin: -1.4, xmax: 3.6, ymin: -0.36, ymax: 1.36 }, canvas: { height: 160 } };
const NARROW: BarLayout = { x0: -1.15, x1: 1.55, ys: -0.75, yt: -1.35, world: { xmin: -1.4, xmax: 1.75, ymin: -1.62, ymax: 1.36 }, canvas: { aspect: 3.15 / 2.98 } };
const TAIL = 1.2; // seconds shown after the motion ends (residual vibration)
const ZETA = 0.02;
const AMP = 20; // visual amplification of the load deviation q − s

const dir = (th: number, r: number): Pt => [r * Math.cos(th), r * Math.sin(th)];

interface Config {
  kind?: ScalingKind;
  shown?: ScalingKind[];
  T?: number | 'min';
  vmax?: number;
  amax?: number;
  jmax?: number;
  fn?: number;
  flex?: boolean;
  t?: number;
}

function mount(host: HTMLElement, ctx: LessonContext): LessonHandle {
  const st = {
    kind: 'cubic' as ScalingKind,
    shown: { cubic: true, quintic: false, trapezoid: false, scurve: false } as Record<ScalingKind, boolean>,
    T: 2.5, vmax: 0.8, amax: 1.0, jmax: 3,
    flex: false, fn: 3,
    t: 0.9,
  };
  const lim = (): Limits => ({ vmax: st.vmax, amax: st.amax, jmax: st.jmax });
  const prof = {} as Record<ScalingKind, Profile>;
  const flexR = {} as Record<ScalingKind, FlexResult>;
  const tEnd = () => prof[st.kind].T + TAIL;

  // ---------------------------------------------------------------- layout
  simTitle(host, '時間縮放實驗台：單關節 0° → 135°', 'θ(t) = θ(s(t))');
  const G = host.clientWidth > 0 && host.clientWidth < 520 ? NARROW : WIDE;
  const BAR_X0 = G.x0, BAR_X1 = G.x1, YS = G.ys, YT = G.yt;
  const cv = new Canvas2D(host, { ...G.canvas, world: G.world, overlay: true });

  const panel = controlPanel(host, { compact: true });
  const chipRow = row(panel);
  const chips = presetChips(chipRow, [
    { label: '三次 vs 五次', apply: () => setConfig({ kind: 'cubic', shown: ['cubic', 'quintic'], T: 2.5, flex: false, t: 0.9 }) },
    { label: '梯形', apply: () => setConfig({ kind: 'trapezoid', shown: ['trapezoid', 'cubic'], T: 2.5, vmax: 0.8, amax: 1.0, flex: false, t: 0.9 }) },
    { label: 'T 太短', title: '梯形：要求的 T 小於 T_min', apply: () => setConfig({ kind: 'trapezoid', shown: ['trapezoid'], T: 1.8, vmax: 0.8, amax: 1.0, flex: false, t: 0.9 }) },
    { label: '三角形', title: '梯形退化成三角形：v_max 用不到', apply: () => setConfig({ kind: 'trapezoid', shown: ['trapezoid'], vmax: 1.2, amax: 0.8, T: 'min', flex: false, t: 0.9 }) },
    { label: 'S 曲線＋振動', apply: () => setConfig({ kind: 'scurve', shown: ['scurve', 'trapezoid'], T: 2.5, vmax: 0.8, amax: 1.0, jmax: 3, fn: 3, flex: true, t: 0.9 }) },
    { label: '全部比較', apply: () => setConfig({ kind: 'quintic', shown: [...KINDS], T: 2.5, vmax: 0.8, amax: 1.0, jmax: 3, fn: 3, flex: true, t: 0.9 }) },
  ], { label: '情境' });

  const typeRow = row(panel);
  const selKind = select<ScalingKind>(typeRow, {
    options: KINDS.map((k) => ({ value: k, label: `播放：${LABEL[k]}` })),
    value: st.kind,
    onChange: (k) => { st.kind = k; st.shown[k] = true; showToggles[k].set(true, { silent: true }); chips.clear(); refresh(); },
  });
  const showWrap = el('div', { class: 'mr-row', style: { gap: '0.2rem 0.8rem' } }, el('span', { class: 'mr-note', style: { margin: '0' } }, '疊加：'));
  typeRow.appendChild(showWrap);
  const showToggles = {} as Record<ScalingKind, ReturnType<typeof toggle>>;
  for (const k of KINDS) {
    showToggles[k] = toggle(showWrap, { label: SHORT[k], value: st.shown[k], onChange: (v) => { st.shown[k] = v; chips.clear(); refresh(); } });
  }

  const sT = slider(panel, { label: '總時間 T', min: 1, max: 5, step: 0.01, value: st.T, unit: 's', onInput: (v) => { st.T = v; chips.clear(); refresh(); } });
  const sV = slider(panel, { label: 'v_max', min: 0.3, max: 2, step: 0.05, value: st.vmax, unit: '1/s', onInput: (v) => { st.vmax = v; chips.clear(); refresh(); } });
  const sA = slider(panel, { label: 'a_max', min: 0.3, max: 3, step: 0.05, value: st.amax, unit: '1/s²', onInput: (v) => { st.amax = v; chips.clear(); refresh(); } });
  const sJ = slider(panel, { label: 'j_max（S 曲線）', min: 1, max: 30, step: 0.5, value: st.jmax, unit: '1/s³', onInput: (v) => { st.jmax = v; chips.clear(); refresh(); } });
  const sFn = slider(panel, { label: 'f_n（負載）', min: 0.5, max: 5, step: 0.1, value: st.fn, unit: 'Hz', onInput: (v) => { st.fn = v; refresh(); } });
  const optCell = el('div', { style: { display: 'flex', flexWrap: 'wrap', gap: '0.35rem 0.6rem', alignItems: 'center' } });
  panel.appendChild(optCell);
  button(optCell, {
    label: 'T ← T_min', small: true, title: '把 T 設成目前縮放在 v_max、a_max（S 曲線另含 j_max）限制下的最短時間',
    onClick: () => { setT(minTime(st.kind, lim())); chips.clear(); refresh(); },
  });
  const tFlex = toggle(optCell, { label: '畫出柔性負載', value: st.flex, onChange: (v) => { st.flex = v; refresh(); } });

  const pbRow = row(panel);
  const pb = playback(pbRow, {
    duration: 3.7,
    onPlay: () => { if (st.t >= tEnd() - 1e-6) st.t = 0; loop.start(); },
    onPause: () => loop.stop(),
    onReset: () => { loop.stop(); pb.setPlaying(false, { silent: true }); setTime(0); },
    onSpeed: (s) => { loop.speed = s; },
    onScrub: (t) => { setTime(t); },
  });

  const chainRow = row(panel);
  chainRow.style.gap = '0.2rem 0.9rem';
  // Chain rule shown with its factors: θ′ = Δθ (constant for this straight joint path) times ṡ or s̈.
  const rThd = readout(chainRow, { label: 'dθ/dt = θ′ṡ =', unit: 'rad/s', title: `θ′ = Δθ = ${fmt(DTH, 3)} rad` });
  const rThdd = readout(chainRow, { label: 'd²θ/dt² = θ′s̈ =', unit: 'rad/s²', title: 'θ″ = 0（直線路徑），所以 θ″ṡ² 項為零' });

  const status = el('div', { class: 'mr-note', style: { fontSize: '0.84rem', margin: '0.3rem 0' } });
  host.appendChild(status);
  const legendEl = el('div', { class: 'mr-legend' });
  host.appendChild(legendEl);

  // Bottom area: three tabs keep the pane short (plots / residual vibration / comparison table).
  const tabRow = el('div', { style: { display: 'flex', flexWrap: 'wrap', alignItems: 'center', gap: '0 0.8rem' } });
  host.appendChild(tabRow);
  const paneA = el('div'), paneB = el('div'), paneC = el('div');
  host.append(paneA, paneB, paneC);
  const panes = [paneA, paneB, paneC];
  const tabs = presetChips(tabRow, [
    { label: '圖表：s · ṡ · s̈ · jerk', apply: () => showPane(0) },
    { label: '柔性負載的振動', apply: () => showPane(1) },
    { label: '比較表', apply: () => showPane(2) },
  ]);
  function showPane(i: number): void { panes.forEach((p, k) => { p.style.display = k === i ? '' : 'none'; }); }
  tabRow.appendChild(el('span', { class: 'mr-note', style: { margin: '0' } }, '灰虛線＝上限；直虛線＝jerk 脈衝 δ'));

  const specs = (): SeriesSpec[] => KINDS.map((k) => ({ name: SHORT[k], width: k === st.kind ? 2.6 : 1.4, dash: k === st.kind ? undefined : [6, 4] }));
  const plotOpts = { series: specs(), height: 108, legend: false };
  const [c1, c2] = split(paneA);
  const pS = new Plot2D(c1, { ...plotOpts, title: '路徑參數 s(t)（橫軸 t, s）', yRange: [-0.05, 1.08] });
  const pSd = new Plot2D(c2, { ...plotOpts, title: '速度 ṡ(t)（1/s）' });
  const [c3, c4] = split(paneA);
  const pSdd = new Plot2D(c3, { ...plotOpts, title: '加速度 s̈(t)（1/s²）' });
  const pJ = new Plot2D(c4, { ...plotOpts, title: 'jerk d³s/dt³（1/s³）' });
  const pV = new Plot2D(paneB, { ...plotOpts, height: 180, title: '負載偏差 q − s：運動結束後仍在晃＝殘餘振動', xLabel: 't (s)' });
  note(paneB, `彈簧負載（f_n 可調，ζ = ${ZETA}）；動畫中的偏差放大 ${AMP} 倍，數值見「比較表」。`);
  const plots = [pS, pSd, pSdd, pJ, pV];
  const group = plotGroup(plots);

  const tableWrap = el('div', { class: 'md', style: { overflowX: 'auto' } });
  paneC.appendChild(tableWrap);
  const tbody = el('tbody');
  tableWrap.appendChild(el('table', { style: { fontSize: '0.8rem', margin: '0.3rem 0 0.2rem' } },
    el('thead', {}, el('tr', {},
      ['縮放', '限制下最短 T', '此時峰值 ṡ', '峰值 |s̈|', '峰值 |jerk|', '殘餘振動'].map((h) => el('th', {}, h)))),
    tbody));
  note(paneC, '最短 T 用同一組上限；峰值以目前的 T 計（不可行時以 T_min 計），紅字＝超出上限；殘餘振動＝結束後負載偏差最大值（相對整段行程）。');

  // ---------------------------------------------------------------- canvas
  const xS = (s: number) => BAR_X0 + s * (BAR_X1 - BAR_X0);
  const xT = (t: number, T: number) => BAR_X0 + (Math.min(Math.max(t, 0), T) / T) * (BAR_X1 - BAR_X0);
  const curState = () => prof[st.kind].sample(st.t);

  cv.draw = (c) => {
    const P = prof[st.kind];
    const col = colorOf(st.kind);
    const muted = cssVar('--fg-muted'), gridS = cssVar('--grid-strong'), fg = cssVar('--fg'), accent = cssVar('--accent'), danger = cssVar('--danger');
    const cur = P.sample(st.t);
    const th = TH0 + cur.s * DTH;

    // path (geometry only) with equal-time ticks of the current time scaling
    c.arc([0, 0], LEN, TH0, TH1, { color: gridS, width: 2, dash: [5, 4] });
    c.line([0, 0], dir(TH0, LEN), { color: muted, width: 6, alpha: 0.22 });
    c.line([0, 0], dir(TH1, LEN), { color: muted, width: 6, alpha: 0.22 });
    c.text([LEN, -0.17], 'start', { color: muted });
    c.text(dir(TH1, LEN + 0.24), 'end', { color: muted });
    for (let k = 0; k <= 10; k++) {
      const a = TH0 + P.sample((k * P.T) / 10).s * DTH;
      c.line(dir(a, LEN * 0.9), dir(a, LEN * 1.1), { color: col, width: 2 });
    }
    c.text([-1.35, -0.12], '刻度：每 T/10', { align: 'left', color: col });

    // link (rigid, or rigid + spring + load)
    if (st.flex) {
      const e = flexAt(flexR[st.kind], st.t).e;
      const thL = TH0 + (cur.s + AMP * e) * DTH;
      const knee = dir(th, LEN * 0.6);
      c.line([0, 0], knee, { color: muted, width: 8 });
      spring(c, knee, dir(thL, LEN), cssVar('--series-5', '#7c3aed'));
      c.circle(dir(th, LEN), 0.065, { color: accent, width: 1.5, dash: [3, 2] });
      c.disk(dir(thL, LEN), 0.08, { fill: cssVar('--series-5', '#7c3aed') });
      c.text([-1.35, -0.3], `負載 q（偏差×${AMP}）`, { align: 'left', color: cssVar('--series-5', '#7c3aed') });
    } else {
      c.line([0, 0], dir(th, LEN), { color: muted, width: 8 });
      c.disk(dir(th, LEN), 0.065, { fill: accent });
    }
    c.polygon([[-0.13, -0.15], [0.13, -0.15], [0, 0]], { fill: gridS, color: gridS, width: 1 });
    c.point([0, 0], 5, { color: fg });

    // path bar (where) and time bar (when), connected by s(t)
    c.text([BAR_X0, YS + 0.2], '路徑 s：到哪裡', { align: 'left', color: fg });
    c.line([BAR_X0, YS], [BAR_X1, YS], { color: gridS, width: 6 });
    c.text([BAR_X0, YS - 0.14], '0', { color: muted });
    c.text([BAR_X1, YS - 0.14], '1', { color: muted });
    c.text([BAR_X0, YT + 0.2], '時間 t：何時', { align: 'left', color: fg });
    c.line([BAR_X0, YT], [BAR_X1, YT], { color: gridS, width: 6 });
    c.text([BAR_X0, YT - 0.14], '0', { color: muted });
    c.text([BAR_X1, YT - 0.14], `T = ${fmt(P.T, 2)} s`, { color: muted, align: 'right' });
    for (let k = 0; k <= 10; k++) {
      const tk = (k * P.T) / 10;
      const sk = P.sample(tk).s;
      c.line([xT(tk, P.T), YT + 0.04], [xS(sk), YS - 0.04], { color: col, width: 1, alpha: 0.45 });
      c.point([xT(tk, P.T), YT], 2.5, { color: col });
      c.point([xS(sk), YS], 2.5, { color: col });
    }
    c.line([xT(st.t, P.T), YT], [xS(cur.s), YS], { color: accent, width: 2.2 });
    if (!P.feasible) {
      const x = xT(P.Treq, P.T);
      c.line([x, YT - 0.09], [x, YT + 0.09], { color: danger, width: 2.5 });
      c.text([x, YT + 0.13], 'T 要求', { color: danger });
    }
    c.overlay.textContent = `t = ${fmt(st.t, 2)} s   s = ${fmt(cur.s, 3)}   θ = ${fmt((th * 180) / Math.PI, 1)}°`;
  };

  function spring(c: Canvas2D, a: Pt, b: Pt, color: string): void {
    const dx = b[0] - a[0], dy = b[1] - a[1];
    const L = Math.hypot(dx, dy) || 1;
    const nx = -dy / L, ny = dx / L;
    const pts: Pt[] = [a];
    const n = 10;
    for (let i = 1; i < n; i++) {
      const f = i / n, sg = i % 2 ? 1 : -1;
      pts.push([a[0] + dx * f + nx * 0.035 * sg, a[1] + dy * f + ny * 0.035 * sg]);
    }
    pts.push(b);
    c.polyline(pts, { color, width: 1.6 });
  }

  let wasPlaying = false;
  const grab = () => { wasPlaying = loop.running; loop.stop(); pb.setPlaying(false, { silent: true }); };
  const release = () => { if (wasPlaying) { loop.start(); pb.setPlaying(true, { silent: true }); } };
  cv.addHandle({
    get: () => [xS(curState().s), YS],
    set: (p) => setTime(timeAtS(prof[st.kind], clamp((p[0] - BAR_X0) / (BAR_X1 - BAR_X0), 0, 1))),
    onStart: grab, onEnd: release, radius: 7,
  });
  cv.addHandle({
    get: () => [xT(st.t, prof[st.kind].T), YT],
    set: (p) => setTime(clamp((p[0] - BAR_X0) / (BAR_X1 - BAR_X0), 0, 1) * prof[st.kind].T),
    onStart: grab, onEnd: release, radius: 7, color: '--fg-muted',
  });
  cv.addHandle({
    get: () => dir(TH0 + curState().s * DTH, LEN),
    set: (p) => {
      const a = Math.atan2(p[1], p[0]);
      const s = clamp((a < -Math.PI / 2 ? a + 2 * Math.PI : a) / DTH, 0, 1);
      setTime(timeAtS(prof[st.kind], s));
    },
    onStart: grab, onEnd: release, radius: 9,
  });

  // ---------------------------------------------------------------- update logic
  function setT(T: number): void { st.T = T; sT.set(Math.min(5, T), { silent: true }); }

  function setTime(t: number): void {
    st.t = clamp(t, 0, tEnd());
    frame();
  }

  function frame(): void {
    const r = curState();
    rThd.set(`${fmt(DTH, 2)}×${fmt(r.sd, 2)} = ${fmt(r.sd * DTH, 2)}`);
    rThdd.set(`${fmt(DTH, 2)}×${fmt(r.sdd, 2)} = ${fmt(r.sdd * DTH, 2)}`);
    group.setCursor(st.t);
    pb.setTime(st.t);
    cv.requestRender();
  }

  function setConfig(c: Config): void {
    loop.stop(); pb.setPlaying(false, { silent: true });
    if (c.kind) { st.kind = c.kind; selKind.set(c.kind, { silent: true }); }
    if (c.shown) for (const k of KINDS) { st.shown[k] = c.shown.includes(k); showToggles[k].set(st.shown[k], { silent: true }); }
    if (c.vmax != null) { st.vmax = c.vmax; sV.set(c.vmax, { silent: true }); }
    if (c.amax != null) { st.amax = c.amax; sA.set(c.amax, { silent: true }); }
    if (c.jmax != null) { st.jmax = c.jmax; sJ.set(c.jmax, { silent: true }); }
    if (c.fn != null) { st.fn = c.fn; sFn.set(c.fn, { silent: true }); }
    if (c.T === 'min') setT(minTime(st.kind, lim()));
    else if (c.T != null) setT(c.T);
    if (c.flex != null) { st.flex = c.flex; tFlex.set(c.flex, { silent: true }); }
    if (c.t != null) st.t = c.t;
    refresh();
  }

  function refresh(): void {
    const L = lim();
    for (const k of KINDS) prof[k] = makeProfile(k, st.T, L);
    const shown = KINDS.filter((k) => st.shown[k] || k === st.kind);
    const Tview = Math.max(...shown.map((k) => prof[k].T));
    const x0 = -0.15, x1 = Math.max(3.7, Tview + TAIL);
    const Tall = Math.max(...KINDS.map((k) => prof[k].T)) + TAIL;
    for (const k of KINDS) flexR[k] = flexResponse(prof[k], st.fn, ZETA, Math.max(Tall, x1));

    // curves
    // The jerk floor of ±5 keeps labelled ticks (−5, 0, 5) on the short jerk plot.
    const ext = { sd: [0, 0.9], sdd: [-1, 1], jerk: [-5, 5], e: [-0.004, 0.004] };
    KINDS.forEach((k, i) => {
      if (!shown.includes(k)) { plots.forEach((p) => p.setXY(i, [], [])); return; }
      const P = prof[k];
      const ts = sampleTimes(P, x0, x1, 360);
      const S: number[] = [], Sd: number[] = [], Sdd: number[] = [], Jk: number[] = [];
      for (const t of ts) {
        const r = P.sample(t);
        S.push(r.s); Sd.push(r.sd); Sdd.push(r.sdd); Jk.push(r.jerk);
        ext.sd[1] = Math.max(ext.sd[1], r.sd * 1.08);
        ext.sdd[0] = Math.min(ext.sdd[0], r.sdd * 1.1); ext.sdd[1] = Math.max(ext.sdd[1], r.sdd * 1.1);
        ext.jerk[0] = Math.min(ext.jerk[0], r.jerk * 1.1); ext.jerk[1] = Math.max(ext.jerk[1], r.jerk * 1.1);
      }
      pS.setXY(i, ts, S); pSd.setXY(i, ts, Sd); pSdd.setXY(i, ts, Sdd); pJ.setXY(i, ts, Jk);
      const fr = flexR[k];
      pV.setXY(i, fr.t, fr.e);
      for (const v of fr.e) { ext.e[0] = Math.min(ext.e[0], v * 1.1); ext.e[1] = Math.max(ext.e[1], v * 1.1); }
    });
    // Expand-only ranges keep heights comparable when T changes (peaks scale with 1/T and 1/T²).
    pSd.setRanges(undefined, [-0.05, Math.max(ext.sd[1], st.vmax * 1.32)]);
    pSdd.setRanges(undefined, [Math.min(ext.sdd[0], -st.amax * 1.4), Math.max(ext.sdd[1], st.amax * 1.4)]);
    const jHi = st.shown.scurve || st.kind === 'scurve' ? st.jmax * 1.4 : 0;
    pJ.setRanges(undefined, [Math.min(ext.jerk[0], -jHi), Math.max(ext.jerk[1], jHi)]);
    pV.setRanges(undefined, [ext.e[0], ext.e[1]]);
    group.setXRange([x0, x1]);
    const sp = specs();
    plots.forEach((p) => p.setOptions({ series: sp }));

    // markers: limits, jerk impulses, requested T
    const danger = cssVar('--danger'), muted = cssVar('--fg-muted');
    const P = prof[st.kind];
    const reqT: Marker[] = P.feasible ? [] : [{ x: P.Treq, color: danger, label: 'T 要求', dash: [6, 3] }];
    group.setMarkers(0, reqT);
    group.setMarkers(1, [...reqT, { y: st.vmax, color: muted, dash: [3, 3] }]);
    group.setMarkers(2, [...reqT, { y: st.amax, color: muted, dash: [3, 3] }, { y: -st.amax, color: muted, dash: [3, 3] }]);
    const jm: Marker[] = [...reqT];
    shown.forEach((k) => {
      prof[k].accelJumps.forEach((x, j) => jm.push({ x, color: colorOf(k), dash: [2, 3], label: k === st.kind && j === 0 ? 'δ' : undefined }));
    });
    if (jHi > 0) jm.push({ y: st.jmax, color: muted, dash: [3, 3] }, { y: -st.jmax, color: muted, dash: [3, 3] });
    group.setMarkers(3, jm);
    group.setMarkers(4, [{ x: P.T, color: muted, label: '運動結束', dash: [4, 3] }]);

    pb.setDuration(tEnd());
    st.t = clamp(st.t, 0, tEnd());
    renderLegend(shown);
    renderStatus();
    renderTable();
    frame();
  }

  function renderLegend(shown: ScalingKind[]): void {
    legendEl.replaceChildren(...shown.map((k) => el('span', { style: { fontWeight: k === st.kind ? '700' : '400' } },
      el('span', { class: 'sw', style: { background: colorOf(k) } }), LABEL[k] + (k === st.kind ? '（播放中）' : ''))));
  }

  function renderStatus(): void {
    const P = prof[st.kind];
    const ok = cssVar('--ok'), danger = cssVar('--danger'), fg = cssVar('--fg');
    let text = '';
    let color = fg;
    if (st.kind === 'cubic' || st.kind === 'quintic') {
      const over: string[] = [];
      if (P.peak.sd > st.vmax * (1 + 1e-9)) over.push('v_max');
      if (P.peak.sdd > st.amax * (1 + 1e-9)) over.push('a_max');
      text = `${SHORT[st.kind]}多項式只看 T，不管上限：峰值 ṡ = ${fmt(P.peak.sd, 3)}、|s̈| = ${fmt(P.peak.sdd, 3)}` +
        (over.length ? `，超過 ${over.join(' 與 ')}（要守限制需 T ≥ ${fmt(minTime(st.kind, lim()), 2)} s）。` : '，都在上限內。');
      color = over.length ? danger : fg;
    } else if (!P.feasible) {
      const why = st.kind === 'trapezoid'
        ? (st.vmax * st.vmax > st.amax ? '（v²/a > 1：最快也只是三角形）' : '（以 a_max 加速、以 v_max 巡航仍來不及）')
        : '（含 jerk 斜坡後更慢）';
      text = `⚠ 不可行：要求 T = ${fmt(P.Treq, 2)} s，但在上限下最短需要 T_min = ${fmt(P.Tmin, 2)} s${why}。圖中改畫最短時間版本，紅色虛線是要求的 T。`;
      color = danger;
    } else if (st.kind === 'trapezoid') {
      const i = P.info;
      text = i.triangle
        ? `梯形退化成三角形：以 a = ${fmt(i.a!, 2)} 加速 ${fmt(i.tAccel!, 2)} s 後立刻減速，峰值 ṡ = ${fmt(i.v!, 3)}，沒有等速段（T = T_min = ${fmt(P.Tmin, 2)} s）。`
        : `梯形可行：以 a_max = ${fmt(i.a!, 2)} 加速 ${fmt(i.tAccel!, 2)} s，巡航 ṡ = ${fmt(i.v!, 3)}（≤ v_max ${fmt(st.vmax, 2)}），等速 ${fmt(i.tCruise!, 2)} s。T_min = ${fmt(P.Tmin, 2)} s。`;
      color = ok;
    } else {
      const i = P.info;
      text = `S 曲線可行：斜坡 ${fmt(i.tJerk!, 2)} s、等加速 ${fmt(i.tConstA!, 2)} s、巡航 ṡ = ${fmt(i.v!, 3)}、等速 ${fmt(i.tCruise!, 2)} s` +
        `${i.aSaturated ? '' : '（加速度未達 a_max）'}。T_min = ${fmt(P.Tmin, 2)} s。`;
      color = ok;
    }
    status.textContent = text;
    status.style.color = color;
  }

  function renderTable(): void {
    const danger = cssVar('--danger');
    const L = lim();
    const cell = (txt: string, bad = false) => el('td', { style: { fontFamily: 'var(--mono)', color: bad ? danger : '', whiteSpace: 'nowrap' } }, txt);
    tbody.replaceChildren(...KINDS.map((k) => {
      const P = prof[k];
      const Tm = minTime(k, L);
      const jerkTxt = Number.isFinite(P.peak.jerk) ? fmt(P.peak.jerk, 2) : '∞';
      const tr = el('tr', { style: { background: k === st.kind ? 'var(--accent-soft)' : '' } },
        el('td', { style: { whiteSpace: 'nowrap' } }, el('span', { style: { display: 'inline-block', width: '10px', height: '10px', borderRadius: '2px', background: colorOf(k), marginRight: '0.35rem' } }), SHORT[k]),
        cell(`${fmt(Tm, 2)} s`),
        cell(fmt(P.peak.sd, 3), P.peak.sd > L.vmax * (1 + 1e-9)),
        cell(fmt(P.peak.sdd, 3), P.peak.sdd > L.amax * (1 + 1e-9)),
        cell(jerkTxt, !(P.peak.jerk <= L.jmax * (1 + 1e-9))),
        cell(`${fmt(flexR[k].residual * 100, 2)} %`));
      return tr;
    }));
  }

  const loop = createLoop((dt) => {
    st.t += dt;
    if (st.t >= tEnd()) { st.t = tEnd(); loop.stop(); pb.setPlaying(false, { silent: true }); }
    frame();
  });

  // ---------------------------------------------------------------- steps
  const stepConfig: (() => void)[] = [
    () => setConfig({ kind: 'cubic', shown: ['cubic'], T: 2.5, vmax: 0.8, amax: 1.0, jmax: 3, fn: 3, flex: false, t: 0.9 }),
    () => setConfig({ kind: 'cubic', shown: ['cubic'], T: 2.5, flex: false, t: 0.9 }),
    () => chips.activate(0),
    () => chips.activate(1),
    () => chips.activate(4),
    () => chips.activate(5),
  ];
  const focusOf = (i: number): HTMLElement[] => [
    [cv.el],
    [chainRow, pSd.el, pSdd.el],
    [pSdd.el],
    [sT.el, sV.el, sA.el, status],
    [paneB, sJ.el],
    [paneC],
  ][i] ?? [];
  const tabOf = [0, 0, 0, 0, 1, 2];
  let focused: HTMLElement[] = [];
  const onStep = (i: number) => {
    stepConfig[i]?.();
    if (i < 2) chips.clear();
    tabs.activate(tabOf[i] ?? 0);
    focused.forEach((f) => f.classList.remove('mr-focus'));
    focused = focusOf(i);
    focused.forEach((f) => f.classList.add('mr-focus'));
  };

  refresh();
  onStep(ctx.currentStep);
  const unsubTheme = ctx.onThemeChange(() => refresh());

  return {
    onStep,
    destroy() {
      loop.stop();
      unsubTheme();
      cv.destroy();
      plots.forEach((p) => p.destroy());
    },
  };
}

const lesson: Lesson = { meta, steps, quiz, mount };
export default lesson;
