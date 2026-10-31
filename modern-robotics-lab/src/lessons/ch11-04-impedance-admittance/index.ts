import type { Lesson, LessonContext, LessonHandle, LessonStep, QuizQuestion } from '@/types';
import {
  el, cssVar, fmt, controlPanel, row, slider, select, button, simTitle, note, kv, legend, split,
  Canvas2D, Plot2D, createLoop, playback, presetChips, drawPlanarArm, mdElement,
  type Pt, type PresetChipsHandle, type Handle,
} from '@/ui';
import { meta } from './meta';
import { ImpedanceSim, DEFAULT_PARAMS, HOME, clampTarget, type ImpParams, type Mode, type ArmKind, type Vec2 } from './impedanceSim';

const steps: LessonStep[] = [
  {
    title: '把手臂變成彈簧',
    md: `上一課把方向切成「控制運動」與「控制力」兩半。這一課換個想法：不要求末端死守位置，也不直接指定力，而是規定它**被推的時候怎麼回應**——像一個掛在目標點 $x_0$ 上的虛擬質量—彈簧—阻尼器：

$$M\\ddot x + B\\dot x + K(x - x_0) = f_{ext}$$

$f_{ext}$ 是環境（人的手、牆）施加在末端上的力。$K$ 決定被推開多少、$B$ 決定晃多久會停、$M$ 決定多快開始讓步。這組「力與運動的動態關係」就是末端的**阻抗**（impedance）。`,
    simHint: '拖曳紫色旗子移動目標點 x₀，或直接抓住末端拉一下再放開（滑鼠的拉力就是 f_ext），感受「彈簧」怎麼回應。',
  },
  {
    title: '阻抗控制：量運動、出力',
    md: `**阻抗控制**（impedance control）用編碼器量末端的 $x$、$\\dot x$，算出虛擬彈簧與阻尼「應該」產生的力，再經 $J^\\T$ 交給馬達：

$$\\tau = J^\\T(\\theta)\\Big(\\tilde\\Lambda\\ddot x + \\tilde\\eta - \\big(M\\ddot x + B\\dot x + K(x - x_0)\\big)\\Big)$$

若接受手臂自己的慣量當作 $M$（取 $M = \\tilde\\Lambda$），含 $\\ddot x$ 的兩項正好抵消，低速時只剩 $\\tau \\approx \\tilde g + J^\\T\\big(K(x_0 - x) - B\\dot x\\big)$：不必量加速度，也不需要力感測器。代價是關節摩擦、減速機的反射慣量會原封不動地混進「手感」，所以它適合**可背驅動**（直驅、低摩擦）的手臂。`,
    simHint: '看「控制器 F_x」曲線怎麼隨位置誤差變化。點「高減速比＋K = 100」：目標移走了，末端卻被摩擦卡在半路。',
  },
  {
    title: '導納控制：量力、出運動',
    md: `**導納控制**（admittance control）把方向反過來：用腕部力感測器量 $f_{ext}$，把虛擬模型當作要積分的微分方程，算出「被推之後該去哪裡」：

$$\\ddot x_c = M^{-1}\\big(f_{ext} - B\\dot x_c - K(x_c - x_0)\\big)$$

積分得到的 $x_c$ 交給一個很硬的內層位置迴路去追。內迴路夠硬，摩擦與減速機慣量都被「蓋掉」，渲染出的 $M$、$B$、$K$ 很準，而且 $M$ 可以自由設定——這正是**高減速比、難以背驅動**的工業手臂常用的作法。模擬分兩層：虛擬模型與內迴路以 1 kHz 更新，手臂本身以 0.2 ms 子步積分。`,
    simHint: '高減速比手臂在導納模式下：拉一下末端，紫色的 x_c（方塊與曲線）與實際 x 幾乎重合。再點「M = 0.5」與「M = 8」比較讓步快慢。',
  },
  {
    title: '碰牆',
    md: `把目標點放進牆裡，讓末端壓著牆。穩態時虛擬彈簧與牆面串聯，接觸力約為

$$f \\approx \\frac{K\\,k_w}{K + k_w}\\,(x_0 - x_w) \\approx K\\,(x_0 - x_w)$$

- **阻抗控制**只輸出彈簧與阻尼的力，末端與牆的互動理想上是被動的（迴路本身不注入能量），牆再硬也穩定。
- **導納控制**把牆的反力經感測器（帶延遲）送回虛擬模型，再變成位置命令。牆越硬，$x_c$ 的一點點變化就造成很大的力；延遲讓這個迴路像負阻尼一樣不斷注入能量，於是開始敲牆。牆越軟，能容忍的延遲越長。`,
    simHint: '一開始是「導納＋延遲 15 ms」：看接觸力曲線的彈跳。改點「延遲 0」或「阻抗」，再試「硬牆 5×10⁴」兩種模式的差別。',
  },
  {
    title: '調 M、B、K',
    md: `撞上牆時的峰值接觸力與之後的彈跳，取決於渲染出的三個參數：

| 參數 | 調大時 |
|---|---|
| $K$ | 末端更「硬」：接近得更快、峰值更高，貼牆的穩態力 $\\approx K(x_0 - x_w)$ 也更大 |
| $B$ | 吸收能量：彈跳變少、衝擊較緩；太小則會在牆上彈跳 |
| $M$ | 讓步較慢、帶著較多動量撞牆，峰值也較高（阻抗模式下就是手臂自己的 $\\Lambda$） |

與人協作時通常選低 $K$、足夠 $B$ 的**低阻抗**；需要精準定位或抵抗干擾時才用高阻抗。`,
    simHint: '每個參數按鈕都會讓目標點跳到牆後 15 cm 撞一次：比較「峰值接觸力」讀數與力曲線的彈跳次數。',
  },
  {
    title: '選哪一種？與本章總結',
    md: `阻抗與導納渲染的是**同一個**目標動態，差別在於依賴哪種硬體：

| | 阻抗控制 | 導納控制 |
|---|---|---|
| 量測 → 輸出 | 位置、速度 → 力矩 | 力 → 位置命令 |
| 適合的手臂 | 直驅、低摩擦、可背驅動 | 高減速比、高摩擦 |
| 擅長 | 低阻抗、硬接觸也穩定 | 準確渲染、蓋掉摩擦 |
| 弱點 | 摩擦與反射慣量混進手感 | 硬接觸＋延遲易震盪 |

本章從誤差動態、PID、計算力矩法一路走到力控制與阻抗。下一章轉向手指與物體之間的接觸：接觸點只能推不能拉，切向力受摩擦限制，所有可行的接觸力構成一個**摩擦錐**。`,
    simHint: '四種「手臂 × 控制器」組合各點一次：先移動、再撞牆。對照表格驗證每一格；牆上的錐形是下一章的預告。',
  },
];

const quiz: QuizQuestion[] = [
  {
    question: '協作機器人要和人一起搬東西，人會隨時推拉末端，偶爾也會不小心撞到。末端應該設定成什麼樣的阻抗？',
    options: [
      '高阻抗（大 $K$、小 $B$）：位置最準，人推也推不動',
      '低阻抗（小 $K$、足夠的 $B$）：人用小力就能帶動，撞到時接觸力也小',
      '零阻尼（$B = 0$）：讓末端完全自由擺動',
      '阻抗大小無所謂，只要用力感測器就安全',
    ],
    answer: 1,
    explanation: '和人互動時希望「被推得動、撞到不痛」：小 $K$ 讓同樣的位移只產生小的力，足夠的 $B$ 吸收能量、避免來回晃。高阻抗適合需要抵抗干擾的精準定位；$B = 0$ 則任何擾動都會讓末端一直振盪。',
  },
  {
    question: '一台使用諧波減速機、用手幾乎推不動的工業手臂，想做出柔順的行為，較適合哪一種實作？',
    options: [
      '阻抗控制：直接用 $J^\\T$ 輸出彈簧力即可',
      '導納控制：裝腕部力感測器，量到的力經虛擬模型變成位置命令，再由硬的位置內迴路執行',
      '兩者都做不到，高減速比手臂不能柔順',
      '只要把位置控制增益調到零',
    ],
    answer: 1,
    explanation: '高減速比帶來大的摩擦與反射慣量。阻抗控制輸出的是力矩，這些摩擦直接擋在中間，手感變得黏滯、末端會卡住（模擬中「高減速比＋阻抗、K = 100」就看得到）。導納控制靠硬的位置內迴路把摩擦蓋掉，再用量到的力決定該怎麼讓步。',
  },
  {
    question: '腕部力感測器的訊號延遲了 20 ms。與一面很硬的牆接觸時，哪一種模式比較危險？',
    options: [
      '阻抗控制，因為它完全依賴力感測器',
      '導納控制，因為延遲的力訊號在迴路裡，硬牆會讓它震盪甚至敲牆',
      '兩者一樣，延遲對誰都沒差',
      '兩者都不受影響，因為牆是被動的',
    ],
    answer: 1,
    explanation: '最簡形式的阻抗控制只量位置與速度，根本不使用力感測器，延遲進不了迴路。導納控制則用量到的力推動虛擬模型；牆越硬，同樣的位置變化產生越大的力，延遲讓迴路相位落後、等效阻尼變負，於是彈跳。模擬第 4 步的「導納＋延遲 15 ms」與「阻抗」可以直接比較。',
  },
  {
    question: '阻抗模式下，$K = 300$ N/m，把目標點放在牆後 $0.1$ m，牆面剛度 $5\\times10^4$ N/m。穩態接觸力大約多少？',
    options: ['約 3 N', '約 30 N', '約 300 N', '約 5000 N'],
    answer: 1,
    explanation: '虛擬彈簧與牆串聯：$f = \\dfrac{K k_w}{K + k_w}(x_0 - x_w) = \\dfrac{300 \\times 5\\times10^4}{300 + 5\\times10^4}\\times 0.1 \\approx 29.8$ N。牆比虛擬彈簧硬得多，幾乎所有變形都在虛擬彈簧上，所以約等於 $K \\times 0.1 = 30$ N。想要更小的接觸力，就把 $K$ 調小或把目標點放淺一點。',
  },
];

// ---------------------------------------------------------------------------

const WORLD = { xmin: -0.3, xmax: 2.0, ymin: -0.45, ymax: 1.05 };
const F_SCALE = 0.006; // m per N for force arrows
const LOG_DT = 0.01;
const FRAME_BUDGET_MS = 8;
/** Plot samples are batched and handed to Plot2D at most this often (each push redraws every point). */
const PLOT_FLUSH_MS = 100;
/** Minimum interval between canvas redraws while the sim runs (ms): ~30 fps is plenty here. */
const DRAW_MIN_MS = 30;
const PLOT_WINDOW = 6;
const K_BAND = 300; // N/m, the mouse "rubber band"
const BAND_MAX = 60; // N
const FLAG = 0.14; // the x₀ handle hangs this far below the target point
const MODE_LABEL: Record<Mode, string> = { impedance: '阻抗控制（量運動 → 出力）', admittance: '導納控制（量力 → 出運動）' };
const ARM_LABEL: Record<ArmKind, string> = { direct: '直驅・可背驅動', geared: '高減速比・高摩擦' };
const SUP = '⁰¹²³⁴⁵⁶⁷⁸⁹';
function fmtStiff(k: number): string {
  const e = Math.floor(Math.log10(k) + 1e-9);
  return `${fmt(k / 10 ** e, 1)}×10${String(e).split('').map((d) => SUP[Number(d)]).join('')} N/m`;
}
const add = (a: ArrayLike<number>, b: ArrayLike<number>, s = 1): Pt => [a[0] + s * b[0], a[1] + s * b[1]];
function capLen(v: Vec2, max: number): Vec2 {
  const L = Math.hypot(v[0], v[1]);
  return L > max ? [(v[0] * max) / L, (v[1] * max) / L] : v;
}

/** Zig-zag spring from a to b (world units). */
function drawSpring(c: Canvas2D, a: Pt, b: Pt, color: string): void {
  const dx = b[0] - a[0], dy = b[1] - a[1];
  const L = Math.hypot(dx, dy);
  if (L < 0.01) return;
  const ux = dx / L, uy = dy / L, px = -uy, py = ux;
  const w = 0.035, coils = 7;
  const pts: Pt[] = [a, [a[0] + 0.15 * dx, a[1] + 0.15 * dy]];
  for (let i = 0; i < coils * 2; i++) {
    const s = 0.15 + (0.7 * (i + 0.5)) / (coils * 2);
    const side = i % 2 === 0 ? 1 : -1;
    pts.push([a[0] + s * dx + side * w * px, a[1] + s * dy + side * w * py]);
  }
  pts.push([a[0] + 0.85 * dx, a[1] + 0.85 * dy], b);
  c.polyline(pts, { color, width: 1.8 });
}

type Script = { at: number; x0: Vec2 }[];

function mount(host: HTMLElement, ctx: LessonContext): LessonHandle {
  const sim = new ImpedanceSim({ ...DEFAULT_PARAMS });
  let step = -1;
  let tip: Vec2 = [...HOME];
  let dragging = false;
  let pointer: Vec2 = [...HOME];
  let peak = 0;
  let script: Script = [];
  let lastUi = 0;
  let blownUp = false;
  const lossTimes: number[] = [];
  let wasContact = false;
  let stillSince = -1;

  // ---- layout -------------------------------------------------------------
  simTitle(host, '2R 手臂的虛擬彈簧：阻抗 vs 導納', '控制器 1 kHz');
  const diagram = el('div', { style: { display: 'flex', flexWrap: 'wrap', alignItems: 'center', gap: '0.25rem 0.35rem', fontSize: '0.8rem', margin: '0 0 0.45rem' } });
  host.appendChild(diagram);
  const cv = new Canvas2D(host, { aspect: 1.55, world: WORLD, overlay: true });
  const status = el('div', { class: 'mr-note', style: { minHeight: '2.5em', fontWeight: '600', color: 'var(--danger)' } });
  host.appendChild(status);
  const preview = el('div', { class: 'md' }, el('div', { class: 'callout' }, mdElement('**下一章預告：接觸與摩擦錐。** 真實的牆不只會推回來，還有摩擦：切向力最多是 $\\mu$ 倍的法向力，$|f_t| \\le \\mu f_n$。畫在牆上的錐形就是所有可行接觸力的集合。')));
  preview.style.display = 'none';
  host.appendChild(preview);
  legend(host, [
    { color: 'var(--series-1)', label: '末端 x ／ 控制器輸出 F' },
    { color: 'var(--series-5)', label: '目標 x₀（旗子可拖曳）／ 虛擬點 x_c' },
    { color: 'var(--series-4)', label: '外力 f_ext（牆＋你的拉力）' },
  ]);
  const [plotHostA, plotHostB] = split(host);
  const seriesX = () => [
    { name: 'x（實際）', color: cssVar('--series-1') },
    { name: 'x₀', color: cssVar('--fg-muted'), width: 1.2 },
    { name: 'x_c（導納命令）', color: cssVar('--series-5'), width: 1.2 },
  ];
  const seriesF = () => [
    { name: '牆的接觸力 f_w', color: cssVar('--series-4') },
    { name: '感測器（延遲後）', color: cssVar('--series-6'), width: 1.2 },
    { name: '控制器 F_x', color: cssVar('--series-1') },
  ];
  const plotX = new Plot2D(plotHostA, { title: '末端 x 座標 (m)', xLabel: 't (s)', height: 170, xWindow: PLOT_WINDOW, yMinSpan: 0.1, series: seriesX() });
  const plotF = new Plot2D(plotHostB, { title: '力 (N)', xLabel: 't (s)', height: 170, xWindow: PLOT_WINDOW, yMinSpan: 10, series: seriesF() });
  let pending: { t: number; x: number[]; f: number[] }[] = [];
  let lastFlush = 0;
  const flushPlots = (force = false): void => {
    const now = performance.now();
    if (!force && now - lastFlush < PLOT_FLUSH_MS) return;
    lastFlush = now;
    for (const s of pending) { plotX.push(s.t, s.x); plotF.push(s.t, s.f); }
    pending = [];
  };
  const offTheme = ctx.onThemeChange(() => { plotX.setOptions({ series: seriesX() }); plotF.setOptions({ series: seriesF() }); });

  const panel = controlPanel(host, { compact: true });
  const r1 = row(panel);
  const pbk = playback(r1, {
    playing: true,
    onPlay: () => { if (blownUp) resetSim([]); else loop.start(); },
    onPause: () => { loop.stop(); },
    onReset: () => resetSim([]),
    onSpeed: (s) => { loop.speed = s; },
  });
  button(r1, { label: '撞牆！', small: true, variant: 'primary', title: '目標點瞬間跳到牆後 15 cm', onClick: () => crash() });
  button(r1, { label: '推一下 →', small: true, title: '對末端施加 +x 方向 30 N、0.15 s 的外力', onClick: () => sim.push([30, 0], 0.15) });
  const r2 = row(panel);
  const modeSel = select<Mode>(r2, {
    label: '控制器', value: sim.p.mode,
    options: (Object.keys(MODE_LABEL) as Mode[]).map((m) => ({ value: m, label: MODE_LABEL[m] })),
    onChange: (m) => { chips?.clear(); setParams({ mode: m }); },
  });
  const armSel = select<ArmKind>(r2, {
    label: '手臂', value: sim.p.arm,
    options: (Object.keys(ARM_LABEL) as ArmKind[]).map((a) => ({ value: a, label: ARM_LABEL[a] })),
    onChange: (a) => { chips?.clear(); setParams({ arm: a }); },
  });
  const chipHost = row(panel);
  let chips: PresetChipsHandle | null = null;
  const sM = slider(panel, { label: '虛擬質量 M', min: 0.3, max: 8, step: 0.1, value: sim.p.M, unit: 'kg', onInput: (v) => { sim.p.M = v; chips?.clear(); } });
  const sB = slider(panel, { label: '阻尼 B', min: 0, max: 200, step: 5, value: sim.p.B, unit: 'N·s/m', onInput: (v) => { sim.p.B = v; chips?.clear(); } });
  const sK = slider(panel, { label: '勁度 K', min: 20, max: 1000, step: 10, value: sim.p.K, unit: 'N/m', onInput: (v) => { sim.p.K = v; chips?.clear(); } });
  const sKw = slider(panel, {
    label: '牆面剛度 k_w', min: 3, max: 4.7, step: 0.05, value: Math.log10(sim.p.kw), format: (v) => fmtStiff(10 ** v),
    onInput: (v) => { sim.p.kw = 10 ** v; chips?.clear(); },
  });
  const sDelay = slider(panel, { label: '感測器延遲', min: 0, max: 40, step: 1, value: sim.p.delayMs, unit: 'ms', onInput: (v) => { sim.p.delayMs = v; chips?.clear(); blockDiagram(); } });

  const info = kv(host, ['表觀質量', '牆面接觸力 f_w', '峰值接觸力', '感測器讀值', '|x − x₀|', '系統能量 E']);
  note(host, `模擬：2R 手臂（@/mr 閉式動力學）＋每關節轉子慣量與庫倫／黏滯摩擦；牆是單邊彈簧—阻尼（只推不拉）。拖曳末端時，滑鼠像一條 ${K_BAND} N/m 的橡皮筋（上限 ${BAND_MAX} N）。導納的內迴路是任務空間 PD（ω ≈ 200 rad/s），微分項只作用在量到的速度上。`);

  // ---- handles -----------------------------------------------------------
  const flagHandle: Handle = cv.addHandle({
    get: () => [sim.x0[0], sim.x0[1] - FLAG],
    set: (p) => {
      sim.x0 = clampTarget([p[0], p[1] + FLAG]);
      script = [];
      cv.requestRender();
    },
    label: 'x₀', radius: 8,
  });
  const grabHandle: Handle = cv.addHandle({
    get: () => (dragging ? pointer : tip),
    set: (p) => { pointer = [p[0], p[1]]; },
    onStart: () => { dragging = true; pointer = [...tip]; },
    onEnd: () => { dragging = false; sim.userForce = [0, 0]; },
    label: '抓住拉', radius: 9,
  });

  function blockDiagram(): void {
    const box = (text: string, hl = false) => el('span', {
      style: {
        border: `1px solid ${hl ? 'var(--accent)' : 'var(--border)'}`, borderRadius: '6px', padding: '0.12rem 0.45rem',
        background: hl ? 'var(--accent-soft)' : 'var(--bg-elev)', color: hl ? 'var(--accent)' : 'var(--fg)', whiteSpace: 'nowrap',
      },
    }, text);
    const arrow = () => el('span', { style: { color: 'var(--fg-muted)' } }, '→');
    const parts = sim.p.mode === 'impedance'
      ? [box('量：編碼器 x, ẋ', true), arrow(), box('F = K(x₀ − x) − B ẋ'), arrow(), box('τ = g̃ + Jᵀ F'), arrow(), box(`手臂（${ARM_LABEL[sim.p.arm]}）`)]
      : [box(`量：力感測器 f_ext（延遲 ${fmt(sim.p.delayMs, 0)} ms）`, true), arrow(), box('M ẍ_c + B ẋ_c + K(x_c − x₀) = f_ext'), arrow(), box('內迴路 PD 追 x_c'), arrow(), box(`手臂（${ARM_LABEL[sim.p.arm]}）`)];
    diagram.replaceChildren(...parts);
  }

  // ---- drawing -----------------------------------------------------------
  cv.draw = (c) => {
    const muted = cssVar('--fg-muted');
    const cX = cssVar('--series-1', '#2563eb'), cT = cssVar('--series-5', '#7c3aed'), cF = cssVar('--series-4', '#d97706');
    flagHandle.color = cT;
    grabHandle.color = cssVar('--series-6', '#0891b2');
    // Only label the grab handle in the steps that ask the learner to pull (keeps the contact area readable).
    grabHandle.label = step === 0 || step === 2 ? '抓住拉' : undefined;
    c.grid(0.25);
    const xw = sim.p.wallX;
    if (sim.p.wallOn) {
      c.polygon([[xw, WORLD.ymin - 0.2], [xw + 0.4, WORLD.ymin - 0.2], [xw + 0.4, WORLD.ymax + 0.2], [xw, WORLD.ymax + 0.2]], { fill: cssVar('--bg-sunken'), color: muted, width: 2 });
      for (let y = WORLD.ymin - 0.1; y < WORLD.ymax + 0.1; y += 0.08) c.line([xw, y], [xw + 0.07, y + 0.07], { color: muted, width: 1 });
      c.text([xw - 0.04, WORLD.ymin + 0.06], `牆  k_w = ${fmtStiff(sim.p.kw)}`, { color: muted, align: 'right', bg: cssVar('--sim-bg') });
    }
    // Base pedestal.
    c.polygon([[-0.1, WORLD.ymin - 0.1], [0.1, WORLD.ymin - 0.1], [0.06, -0.06], [-0.06, -0.06]], { fill: cssVar('--bg-sunken'), color: muted, width: 1 });
    const pts = drawPlanarArm(c, [1, 1], sim.theta, { eeColor: cX });
    tip = [pts[2][0], pts[2][1]];
    // Friction-cone preview at the contact point (next chapter).
    if (step === 5 && sim.p.wallOn) {
      const apex: Pt = [xw, tip[1]];
      const h = Math.atan(0.5), L = 0.3;
      const p1: Pt = [apex[0] - L * Math.cos(h), apex[1] + L * Math.sin(h)];
      const p2: Pt = [apex[0] - L * Math.cos(h), apex[1] - L * Math.sin(h)];
      c.polygon([apex, p1, p2], { fill: cssVar('--accent-soft'), color: cssVar('--accent'), width: 1.5, alpha: 0.8 });
      c.text([apex[0] - L * 0.5, apex[1] - L * 0.75], '摩擦錐 μ = 0.5（下一章）', { color: cssVar('--accent'), font: '11px sans-serif' });
    }
    // Target x₀: crosshair + flag pole down to its handle.
    const x0: Pt = [sim.x0[0], sim.x0[1]];
    c.line(x0, [x0[0], x0[1] - FLAG], { color: cT, width: 1.2, dash: [3, 3] });
    c.circle(x0, 0.025, { color: cT, width: 2 });
    c.line([x0[0] - 0.045, x0[1]], [x0[0] + 0.045, x0[1]], { color: cT, width: 1.5 });
    c.line([x0[0], x0[1] - 0.045], [x0[0], x0[1] + 0.045], { color: cT, width: 1.5 });
    // Virtual spring: x₀ → tip (impedance) or x₀ → x_c, then x_c ⇢ tip via the inner loop (admittance).
    if (sim.p.mode === 'impedance') {
      drawSpring(c, x0, tip, cT);
    } else {
      const xc: Pt = [sim.xc[0], sim.xc[1]];
      drawSpring(c, x0, xc, cT);
      c.polygon([[xc[0] - 0.03, xc[1] - 0.03], [xc[0] + 0.03, xc[1] - 0.03], [xc[0] + 0.03, xc[1] + 0.03], [xc[0] - 0.03, xc[1] + 0.03]], { color: cT, width: 2 });
      c.line(xc, tip, { color: cT, width: 1, dash: [2, 3] });
      // Below-left of x_c: clear of the grab-handle label above the tip and of the x₀ flag below.
      if (step === 2) c.text(add(xc, [-0.07, -0.08]), 'x_c（虛擬質量 M）', { color: cT, font: '11px sans-serif', align: 'right' });
    }
    // Rubber band from the tip to the mouse.
    if (dragging) c.line(tip, pointer, { color: cssVar('--series-6'), width: 1.5, dash: [6, 4] });
    // External force on the tip and the controller's task-space force.
    const fe = sim.fExt;
    if (Math.hypot(fe[0], fe[1]) > 0.2) {
      const v = capLen([fe[0] * F_SCALE, fe[1] * F_SCALE], 0.45);
      c.arrow(tip, add(tip, v), { color: cF, width: 3 });
      c.text(add(tip, v), `f_ext ${fmt(Math.hypot(fe[0], fe[1]), 1)} N`, { color: cF, dy: -12 });
    }
    if (step === 1 || step === 2) {
      const F = sim.Fcmd;
      if (Math.hypot(F[0], F[1]) > 0.2) {
        const v = capLen([F[0] * F_SCALE, F[1] * F_SCALE], 0.45);
        c.arrow(add(tip, [0, -0.03]), add(add(tip, [0, -0.03]), v), { color: cX, width: 2.5 });
        c.text(add(add(tip, [0, -0.03]), v), `F ${fmt(Math.hypot(F[0], F[1]), 1)} N`, { color: cX, dy: 12 });
      }
    }
  };
  const overlayText = (): string => `t = ${fmt(sim.t, 2)} s   x = (${fmt(sim.x[0], 3)}, ${fmt(sim.x[1], 3)}) m   f_w = ${fmt(sim.fWall, 1)} N`;
  let lastDraw = 0;

  // ---- simulation loop ---------------------------------------------------
  function logSample(): void {
    pending.push({ t: sim.t, x: [sim.x[0], sim.x0[0], sim.p.mode === 'admittance' ? sim.xc[0] : NaN], f: [sim.fWall, -sim.fMeas[0], sim.Fcmd[0]] });
    peak = Math.max(peak, sim.fWall);
    const contact = sim.fWall > 0;
    if (wasContact && !contact) lossTimes.push(sim.t);
    wasContact = contact;
    while (lossTimes.length && lossTimes[0] < sim.t - 1.5) lossTimes.shift();
  }

  const loop = createLoop((dt) => {
    if (blownUp) return;
    const t0 = performance.now();
    if (dragging) {
      const f = capLen([K_BAND * (pointer[0] - sim.x[0]), K_BAND * (pointer[1] - sim.x[1])], BAND_MAX);
      sim.userForce = f;
    }
    let remaining = dt;
    while (remaining > 1e-9) {
      while (script.length && sim.t >= script[0].at) { sim.x0 = clampTarget(script[0].x0); script.shift(); }
      const h = Math.min(LOG_DT, remaining);
      sim.advance(h);
      logSample();
      remaining -= h;
      if (!sim.healthy() || Math.abs(sim.x[0] - 0.85) > 1.3 || Math.abs(sim.x[1] - 0.3) > 1.2) {
        blownUp = true;
        loop.stop();
        pbk.setPlaying(false, { silent: true });
        status.textContent = '⚠ 不穩定：迴路發散，模擬已暫停。降低 K、加大 B 或縮短延遲後按「⟲ 重設」。';
        break;
      }
      if (performance.now() - t0 > FRAME_BUDGET_MS) break;
    }
    flushPlots();
    const now = performance.now();
    if (now - lastDraw >= DRAW_MIN_MS) { lastDraw = now; cv.requestRender(); }
    updateUi(false);
  });

  /** Back to the start pose, clear plots and run an optional target script (always plays). */
  function resetSim(s: Script): void {
    loop.stop();
    blownUp = false;
    dragging = false;
    sim.userForce = [0, 0];
    sim.x0 = [...HOME];
    sim.reset(HOME);
    script = s.map((e) => ({ at: e.at, x0: [...e.x0] as Vec2 }));
    pending = [];
    plotX.clear(); plotF.clear();
    plotX.setMarkers(sim.p.wallOn ? [{ y: sim.p.wallX, label: '牆 x_w', color: '--fg-muted' }] : []);
    peak = 0; lossTimes.length = 0; wasContact = false; stillSince = -1;
    status.textContent = '';
    loop.start();
    pbk.setPlaying(true, { silent: true });
    cv.requestRender();
    updateUi(true);
  }

  function crash(): void {
    const y = Math.max(-0.2, Math.min(0.7, sim.x[1]));
    sim.x0 = clampTarget([sim.p.wallX + 0.15, y]);
    script = [];
    peak = 0;
  }

  function syncControls(): void {
    const p = sim.p;
    modeSel.set(p.mode, { silent: true });
    armSel.set(p.arm, { silent: true });
    sM.set(p.M, { silent: true });
    sB.set(p.B, { silent: true });
    sK.set(p.K, { silent: true });
    sKw.set(Math.log10(p.kw), { silent: true });
    sDelay.set(p.delayMs, { silent: true });
    sM.disabled = p.mode === 'impedance';
    sM.setLabel(p.mode === 'impedance' ? 'M（阻抗：= Λ）' : '虛擬質量 M');
    sDelay.setLabel(p.mode === 'impedance' ? '延遲（阻抗不用）' : '感測器延遲');
    blockDiagram();
  }

  function setParams(q: Partial<ImpParams>): void {
    const modeChanged = q.mode != null && q.mode !== sim.p.mode;
    Object.assign(sim.p, q);
    if (modeChanged) { sim.syncVirtual(); peak = 0; }
    syncControls();
    cv.requestRender();
    updateUi(true);
  }

  // All DOM text (time, overlay, readouts) is refreshed on this ~90 ms tick, never per frame:
  // per-frame text changes force the surrounding page tiles to be re-rasterised.
  function updateUi(force: boolean): void {
    const now = performance.now();
    if (!force && now - lastUi < 90) return;
    lastUi = now;
    pbk.setTime(sim.t);
    cv.overlay.textContent = overlayText();
    const p = sim.p;
    // Mass felt when the tip is pushed along x (y free): 1/(Λ⁻¹)ₓₓ = Λxx − Λxy²/Λyy, not Λxx.
    const L = sim.Lambda;
    const mx = L[0][0] - (L[0][1] * L[1][0]) / L[1][1];
    info.set('表觀質量', p.mode === 'impedance' ? `沿 x 推時 1/(Λ⁻¹)ₓₓ ≈ ${fmt(mx, 2)} kg（手臂自己的慣量）` : `M = ${fmt(p.M, 1)} kg（虛擬，由導納模型決定）`);
    info.set('牆面接觸力 f_w', `${fmt(sim.fWall, 1)} N`);
    info.set('峰值接觸力', `${fmt(peak, 1)} N（自上次撞牆／重設）`);
    info.set('感測器讀值', p.mode === 'impedance'
      ? `(${fmt(sim.fMeas[0], 1)}, ${fmt(sim.fMeas[1], 1)}) N — 阻抗控制沒有用到它`
      : `(${fmt(sim.fMeas[0], 1)}, ${fmt(sim.fMeas[1], 1)}) N，比真實值晚 ${fmt(p.delayMs + 1, 0)} ms`);
    const err = Math.hypot(sim.x[0] - sim.x0[0], sim.x[1] - sim.x0[1]);
    info.set('|x − x₀|', `${fmt(err * 100, 1)} cm`);
    info.set('系統能量 E', `${fmt(sim.energy(), 2)} J（動能＋虛擬彈簧＋牆）`);
    if (blownUp) return;
    // Friction "stick" detector: far from the target, not moving, nothing pushing.
    const speed = Math.hypot(sim.v[0], sim.v[1]);
    const stuck = p.mode === 'impedance' && err > 0.01 && speed < 0.003 && sim.fWall === 0 && !dragging;
    if (stuck) { if (stillSince < 0) stillSince = sim.t; } else stillSince = -1;
    const msgs: string[] = [];
    if (lossTimes.length >= 3) msgs.push(p.mode === 'admittance' ? '⚠ 末端在牆上彈跳：導納迴路的阻尼不足以抵銷延遲與牆面剛度' : '⚠ 末端在牆上彈跳：B 太小');
    if (stillSince >= 0 && sim.t - stillSince > 0.4) msgs.push(`末端卡在離目標 ${fmt(err * 100, 1)} cm 處：彈簧力 K·距離 經 Jᵀ 換成的關節力矩小於靜摩擦（阻抗控制無法蓋掉摩擦）`);
    if (sim.saturated) msgs.push('關節力矩飽和');
    status.textContent = msgs.join('；');
  }

  // ---- per-step configuration ---------------------------------------------
  const W = DEFAULT_PARAMS.wallX;
  // Scripted target moves start at 0.2 s so the first view of every step already shows motion.
  const MOVE: Script = [{ at: 0.2, x0: [1.36, 0.42] }, { at: 2.4, x0: [1.08, 0.18] }];
  const HIT: Script = [{ at: 0.2, x0: [W + 0.15, 0.25] }];
  const PRESS: Script = [{ at: 0.2, x0: [W + 0.12, 0.25] }];
  type Chip = { label: string; q: Partial<ImpParams>; run?: Script; push?: boolean; title?: string };
  const STEP_SETUP: { q: Partial<ImpParams>; run: Script; chips: Chip[] }[] = [
    {
      q: { mode: 'impedance', arm: 'direct', M: 2, B: 25, K: 300, kw: 5e3, delayMs: 0 },
      run: MOVE,
      chips: [
        { label: '柔軟 K = 100', q: { K: 100 }, run: MOVE },
        { label: '中等 K = 300', q: { K: 300 }, run: MOVE },
        { label: '僵硬 K = 1000', q: { K: 1000 }, run: MOVE },
        { label: '幾乎沒阻尼 B = 5', q: { B: 5 }, run: MOVE },
      ],
    },
    {
      q: { mode: 'impedance', arm: 'direct', M: 2, B: 30, K: 300, kw: 5e3, delayMs: 0 },
      run: MOVE,
      chips: [
        { label: '直驅手臂', q: { arm: 'direct', K: 300 }, run: MOVE },
        { label: '高減速比手臂', q: { arm: 'geared', K: 300 }, run: MOVE },
        { label: '高減速比＋K = 100（卡住）', q: { arm: 'geared', K: 100 }, run: MOVE },
        { label: '推一下 30 N', q: {}, push: true },
      ],
    },
    {
      q: { mode: 'admittance', arm: 'geared', M: 2, B: 60, K: 300, kw: 5e3, delayMs: 0 },
      run: MOVE,
      chips: [
        { label: '導納＋高減速比', q: { arm: 'geared', M: 2 }, run: MOVE },
        { label: '導納＋直驅', q: { arm: 'direct', M: 2 }, run: MOVE },
        { label: 'M = 0.5 kg（讓步快）', q: { M: 0.5 }, run: MOVE },
        { label: 'M = 8 kg（讓步慢）', q: { M: 8 }, run: MOVE },
        { label: '推一下 30 N', q: {}, push: true },
      ],
    },
    {
      q: { mode: 'admittance', arm: 'geared', M: 2, B: 60, K: 300, kw: 5e3, delayMs: 15 },
      run: PRESS,
      chips: [
        { label: '導納＋延遲 15 ms（敲牆）', q: { mode: 'admittance', delayMs: 15, kw: 5e3 }, run: PRESS },
        { label: '導納＋延遲 0', q: { mode: 'admittance', delayMs: 0, kw: 5e3 }, run: PRESS },
        { label: '阻抗（不用力感測器）', q: { mode: 'impedance', arm: 'direct', delayMs: 15, kw: 5e3 }, run: PRESS },
        { label: '硬牆 5×10⁴：導納', q: { mode: 'admittance', arm: 'geared', delayMs: 0, kw: 5e4 }, run: PRESS },
        { label: '硬牆 5×10⁴：阻抗', q: { mode: 'impedance', arm: 'direct', delayMs: 0, kw: 5e4 }, run: PRESS },
      ],
    },
    {
      q: { mode: 'admittance', arm: 'geared', M: 2, B: 60, K: 300, kw: 5e3, delayMs: 0 },
      run: HIT,
      chips: [
        { label: '基準（M 2, B 60, K 300）', q: { M: 2, B: 60, K: 300 }, run: HIT },
        { label: 'K = 100', q: { M: 2, B: 60, K: 100 }, run: HIT },
        { label: 'K = 600', q: { M: 2, B: 60, K: 600 }, run: HIT },
        { label: 'B = 20', q: { M: 2, B: 20, K: 300 }, run: HIT },
        { label: 'B = 150', q: { M: 2, B: 150, K: 300 }, run: HIT },
        { label: 'M = 0.5 kg', q: { M: 0.5, B: 60, K: 300 }, run: HIT },
        { label: 'M = 6 kg', q: { M: 6, B: 60, K: 300 }, run: HIT },
      ],
    },
    {
      q: { mode: 'impedance', arm: 'direct', M: 2, B: 40, K: 300, kw: 2e4, delayMs: 10 },
      run: [...MOVE.slice(0, 1), { at: 2.0, x0: [W + 0.12, 0.3] }],
      chips: (['direct', 'geared'] as ArmKind[]).flatMap((arm) => (['impedance', 'admittance'] as Mode[]).map((mode) => ({
        label: `${arm === 'direct' ? '直驅' : '高減速比'}＋${mode === 'impedance' ? '阻抗' : '導納'}`,
        q: { arm, mode, M: 2, B: 60, K: 300, kw: 2e4, delayMs: 10 } as Partial<ImpParams>,
        run: [{ at: 0.2, x0: [1.36, 0.42] as Vec2 }, { at: 2.0, x0: [W + 0.12, 0.3] as Vec2 }],
      }))),
    },
  ];

  const FOCUS: (() => HTMLElement[])[] = [
    () => [cv.el],
    () => [diagram, plotF.el],
    () => [diagram, plotX.el, sM.el],
    () => [plotF.el, sDelay.el, sKw.el],
    () => [info.el, sM.el, sB.el, sK.el],
    () => [modeSel.el, armSel.el],
  ];
  let focused: HTMLElement[] = [];

  function onStep(i: number): void {
    const k = Math.max(0, Math.min(STEP_SETUP.length - 1, i));
    if (k === step) return;
    step = k;
    const setup = STEP_SETUP[k];
    setParams({ ...DEFAULT_PARAMS, ...setup.q });
    resetSim(setup.run);
    chips?.el.remove();
    chips = presetChips(chipHost, setup.chips.map((ch) => ({
      label: ch.label,
      title: ch.title,
      apply: () => {
        setParams(ch.q);
        if (ch.push) sim.push([30, 0], 0.15);
        if (ch.run) resetSim(ch.run);
      },
    })), { label: '本步實驗：' });
    preview.style.display = k === 5 ? '' : 'none';
    focused.forEach((f) => f.classList.remove('mr-focus'));
    focused = FOCUS[k]();
    focused.forEach((f) => f.classList.add('mr-focus'));
    cv.requestRender();
  }

  onStep(ctx.currentStep);

  return {
    onStep,
    destroy() {
      loop.stop();
      offTheme();
      cv.destroy();
      plotX.destroy();
      plotF.destroy();
    },
  };
}

const lesson: Lesson = { meta, steps, quiz, mount };
export default lesson;
