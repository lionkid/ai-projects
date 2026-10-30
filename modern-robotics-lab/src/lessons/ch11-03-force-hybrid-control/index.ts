import type { Lesson, LessonContext, LessonHandle, LessonStep, QuizQuestion } from '@/types';
import {
  el, cssVar, fmt, controlPanel, row, slider, angleSlider, select, button, matrixView, simTitle, note, kv, legend, split,
  Canvas2D, Plot2D, createLoop, playback, presetChips, torqueBars, drawPlanarArm, mdElement,
  type Pt, type PresetChipsHandle, type Handle,
} from '@/ui';
import { matVec } from '@/mr';
import { meta } from './meta';
import { HybridSim, DEFAULT_PARAMS, SURF_C, TAU_MAX, I_F_MAX, surfaceAxes, sigmaMin, type Mode, type SimParams, type Vec2 } from './hybridSim';

const steps: LessonStep[] = [
  {
    title: '推桌子：位置控制會出什麼事',
    md: `先用熟悉的方法擦桌子：把末端**目標位置**放在桌面上，用一個很硬的任務空間 PD（等於一根勁度 $K_p$ 的虛擬彈簧）去追。麻煩在於桌面真實高度和模型差了 $\\Delta$，哪怕只有幾毫米：

- 桌面比模型低：末端懸在半空，接觸力是 0，根本沒擦到；
- 桌面比模型高：虛擬彈簧被壓縮，接觸力約為

$$f \\approx \\frac{K_p\\,k_e}{K_p + k_e}\\,\\Delta \\;\\approx\\; K_p\\,\\Delta \\quad (k_e \\gg K_p)$$

力的大小由「誤差 × 剛度」決定，不是由我們的指令決定；一碰到桌面還會先出現衝擊尖峰。接觸任務需要**直接控制力**。`,
    simHint: '拖動「桌面高度誤差 Δ」（−5 → +20 mm），看法向力如何從 0 跳到數十牛頓；再點「硬桌面」比較剛接觸時的尖峰。',
  },
  {
    title: '純力控制',
    md: `改成直接指定末端要施加的力 $\\vF_d$（往桌面壓 10 N）。腕部感測器讀到的是桌面推手臂的力 $\\vF_{sensor}$，手臂施加在桌面上的力是 $\\vF_{tip} = -\\vF_{sensor}$。以力誤差 $\\vF_e = \\vF_d - \\vF_{tip}$ 做 PI 回授、經 $J^\\T$ 換成關節力矩，再加上重力補償與速度阻尼：

$$\\tau = \\tilde g(\\theta) + J^\\T(\\theta)\\Big(\\vF_d + K_{fp}\\vF_e + K_{fi}\\!\\int\\!\\vF_e\\,dt - K_{damp}\\,\\vV\\Big)$$

接觸時力迴路的等效勁度是 $(1+K_{fp})\\,k_e$：桌面越硬，接觸動態越快，1 kHz 取樣加上一拍延遲就跟不上而開始彈跳。阻尼項則限制末端衝向桌面的速度，也替接觸振盪踩煞車。`,
    simHint: '依序點「硬桌面」→「硬桌面＋K_fp = 0.2」→「拿掉阻尼」，比較法向力是收斂、彈跳還是失控。',
  },
  {
    title: '自然約束與人工約束',
    md: `擦桌子時，桌面已經替我們決定了一半的事，稱為**自然約束**（natural constraints）：末端穿不進桌面，所以法向速度 $v_n = 0$；無摩擦的桌面推不出切向力，所以 $f_t = 0$。

剩下的自由交給任務，稱為**人工約束**（artificial constraints）：切向指定運動 $v_t = v_d(t)$，法向指定力 $f_n = f_d$。每個方向恰好分到一個自然約束、一個人工約束。把「不能動的方向」寫成矩陣：

$$A(\\theta)\\,\\vV = 0, \\qquad A = n^\\T = \\begin{bmatrix} -\\sin\\alpha & \\cos\\alpha \\end{bmatrix}$$

在 $\\{s\\}$ 中描述平的桌面時 $A$ 是常數；改用關節速度表示，$A(\\theta) = n^\\T J(\\theta)$ 就會隨組態改變。`,
    simHint: '把桌面傾角 α 調到 ±10°，看 A 與 A(θ) 怎麼變，以及圖上的 t（運動方向）與 n（施力方向）如何跟著轉。',
  },
  {
    title: '投影矩陣 P',
    md: `控制器送出的力 $\\vF$ 有兩種去向：一部分讓末端沿桌面加速，另一部分只是壓在桌面上，被反作用力抵銷。投影矩陣把兩者拆開：

$$P = I - A^\\T\\big(A\\Lambda^{-1}A^\\T\\big)^{-1}A\\Lambda^{-1}$$

$P\\vF$ 是會產生運動的部分：它造成的加速度 $\\Lambda^{-1}P\\vF$ 滿足 $A\\Lambda^{-1}P\\vF = 0$，必定沿著桌面；$(I-P)\\vF$ 永遠沿法向，只改變接觸力。$\\Lambda$ 是任務空間質量矩陣；手臂慣性有耦合，所以 $P$ 是「斜的」投影，不等於單純刪掉法向分量。注意 $P$ 作用在**力**上，不是速度。`,
    simHint: '拖曳灰色的測試力 F：看 PF（藍）與 (I−P)F（粉紅）如何分解，以及 PF 造成的加速度（虛線）永遠貼著桌面。試試「純切向」：(I−P)F 是 0 嗎？',
  },
  {
    title: '擦桌子：混合控制器',
    md: `把兩個迴路各放進自己的子空間：運動迴路只管切向位置 $s$，力迴路只管法向壓力，動力學補償 $\\tilde\\eta$（重力、速度項）放在投影之外：

$$\\tau = J^\\T\\Big(P\\,\\tilde\\Lambda\\,a_{cmd} + (I-P)\\,\\vF_{cmd} + \\tilde\\eta\\Big)$$

其中 $a_{cmd} = t\\,(\\ddot s_d + K_p e_s + K_d \\dot e_s)$ 是沿切線 $t$ 的加速度指令，$\\vF_{cmd}$ 是上一步的 PI 力指令（方向 $-n$）。因為 $(I-P)P = 0$，運動迴路送出的力不會改變接觸力；$(I-P)\\vF_{cmd}$ 也不會讓末端沿桌面加速，兩個迴路互不干擾。`,
    simHint: '看兩張圖：法向力貼在 f_d，切向誤差只有零點幾毫米。點「快速擦拭」或改 f_d，兩條曲線還是互不影響嗎？',
  },
  {
    title: '矛盾的指令與下一課',
    md: `如果在法向**同時**要求「停在模型中的桌面高度」和「壓 10 N」呢？沿法向，桌面把兩者綁死：$f = k_e\\,\\delta$。除非剛好 $f_d = k_e\\,\\delta_d$，兩個要求不可能同時成立，兩個積分器只會互相拉扯：

- 桌面比模型高（$\\Delta > 0$）：位置迴路越壓越用力，壓力一路上升；
- 桌面比模型低（$\\Delta < 0$）：位置迴路把工具往上提，最後離開桌面。

所以每個方向只能指定運動**或**力。下一課換個角度：不再切分子空間，而是直接設計末端的「手感」，讓它像質量—彈簧—阻尼器一樣回應外力（阻抗控制）。完成下方測驗後繼續前進。`,
    simHint: '按「⚠ 錯誤示範」，觀察法向力與力矩長條如何失控；再點「Δ = −1 cm」，看工具如何離開桌面。',
  },
];

const quiz: QuizQuestion[] = [
  {
    question: '用一塊平的海綿擦窗戶（海綿平貼玻璃，假設無摩擦）。在海綿座標的 6 個方向（3 個轉動、3 個移動，$z$ 為玻璃法線）中，哪一種分配合理？',
    options: [
      '移動 $x$、$y$ 與繞 $z$ 轉動用運動控制；法向力 $f_z$ 與力矩 $m_x$、$m_y$ 用力控制',
      '6 個方向全部用運動控制，只要軌跡算得夠準',
      '法向同時控制位置 $z$ 與力 $f_z$，才能確保貼緊',
      '$x$、$y$ 用力控制，$z$ 用運動控制',
    ],
    answer: 0,
    explanation: '玻璃擋住了 $v_z$、$\\omega_x$、$\\omega_y$（自然的運動約束），又無法提供 $f_x$、$f_y$、$m_z$（無摩擦時的自然力約束）。人工約束就補在互補的方向：環境允許運動的方向（$v_x, v_y, \\omega_z$）指定運動，環境擋住運動的方向（$f_z, m_x, m_y$）指定力。同一方向同時指定位置與力會互相矛盾。',
  },
  {
    question: '純力控制在中等硬度的桌面上收斂得很好。把桌面換成硬 10 倍、增益都不變，最可能看到什麼？',
    options: [
      '一樣收斂，力控制不在乎環境剛度',
      '法向力開始震盪甚至彈離桌面；要降低 $K_{fp}$ 或加大 $K_{damp}$',
      '力穩定在 $f_d$ 的一半',
      '末端會穿過桌面',
    ],
    answer: 1,
    explanation: '接觸時力迴路的等效勁度約為 $(1+K_{fp})k_e$，接觸振盪頻率約 $\\sqrt{(1+K_{fp})k_e/\\Lambda}$。桌面越硬頻率越高，固定的取樣週期與量測延遲在高頻下相當於負阻尼，於是開始彈跳。降低 $K_{fp}$ 讓頻率下降、加大 $K_{damp}$ 增加阻尼，都能找回穩定裕度（模擬中的「硬桌面」預設可以直接驗證）。',
  },
  {
    question: '末端正壓在桌面上，控制器額外送出一個**純切向**的力 $\\vF = f\\,t$（完全沒有法向分量）。桌面感受到的壓力會改變嗎？',
    options: [
      '不會，因為這個力和法線垂直',
      '一般會，因為 $(I-P)t \\neq 0$：手臂慣性耦合讓切向推力也會把末端往桌面推或拉',
      '只有在桌面傾斜時才會',
      '一定會讓末端離開桌面',
    ],
    answer: 1,
    explanation: '$(I-P)\\vF = n\\,\\dfrac{n^\\T\\Lambda^{-1}\\vF}{n^\\T\\Lambda^{-1}n}$。當 $\\Lambda^{-1}$ 有 $n$–$t$ 交叉項（手臂慣性耦合），$n^\\T\\Lambda^{-1}t \\neq 0$，純切向力會產生朝向或遠離桌面的加速度，只好由桌面的反作用力抵銷，接觸力就變了。這正是 $P$ 要用 $\\Lambda^{-1}$ 加權、而不是單純刪掉法向分量的原因。在第 4 步點「測試力：純切向」就能看到。',
  },
  {
    question: '混合控制中，若在法向**同時**要求「末端停在模型中的桌面高度」與「壓力 10 N」，而真實桌面比模型高 1 cm，會發生什麼事？',
    options: [
      '兩個要求平均一下，壓力停在 5 N 左右',
      '位置迴路與力迴路互相拉扯，壓力持續上升到遠超過 10 N',
      '力迴路優先，壓力精確停在 10 N，位置也恰好達成',
      '末端立刻離開桌面',
    ],
    answer: 1,
    explanation: '沿法向，桌面把位置與力綁成 $f = k_e\\delta$：要讓末端停在模型高度（比真實桌面低 1 cm）需要的力遠大於 10 N。位置迴路的積分持續加壓，力迴路的積分項飽和後擋不住，壓力就一路上升；若桌面反而比模型低，位置迴路會把工具往上提，最後離開桌面。每個方向只能指定運動或力其中之一。',
  },
];

// ---------------------------------------------------------------------------

const WORLD = { xmin: -0.3, xmax: 2.0, ymin: -0.85, ymax: 0.72 };
const FLOOR_Y = -0.82;
/** Drawing scale for forces (m per N) and accelerations (m per m/s²). */
const F_SCALE = 0.012;
const TEST_SCALE = 0.032;
const ACC_SCALE = 0.03;
/** Plot sample period (s) and the per-frame CPU budget for physics (ms). */
const LOG_DT = 0.004;
const FRAME_BUDGET_MS = 14;
const MODE_LABEL: Record<Mode, string> = {
  position: '位置控制', force: '純力控制', hybrid: '混合運動—力控制', conflict: '錯誤：法向同時指定位置與力',
};
const SUP = '⁰¹²³⁴⁵⁶⁷⁸⁹';
function fmtStiff(k: number): string {
  const e = Math.floor(Math.log10(k) + 1e-9);
  return `${fmt(k / 10 ** e, 1)}×10${String(e).split('').map((d) => SUP[Number(d)]).join('')} N/m`;
}
const clamp = (x: number, lo: number, hi: number) => Math.max(lo, Math.min(hi, x));
const add = (a: ArrayLike<number>, b: ArrayLike<number>, s = 1): Pt => [a[0] + s * b[0], a[1] + s * b[1]];
function capLen(v: Vec2, max: number): Vec2 {
  const L = Math.hypot(v[0], v[1]);
  return L > max ? [(v[0] * max) / L, (v[1] * max) / L] : v;
}

function mount(host: HTMLElement, ctx: LessonContext): LessonHandle {
  const sim = new HybridSim({ ...DEFAULT_PARAMS, mode: 'position', delta: 0.01 });
  let step = -1;
  let Ftest: Vec2 = [6, -8];
  let tip: Vec2 = [SURF_C[0], SURF_C[1]];
  let lastUi = 0;
  const lossTimes: number[] = [];
  let wasContact = false;
  let lastContactT = 0;

  // ---- layout -------------------------------------------------------------
  simTitle(host, '2R 手臂擦桌子：位置、力與混合控制', '控制器 1 kHz');
  const cv = new Canvas2D(host, { aspect: 1.5, world: WORLD, overlay: true });
  const status = el('div', { class: 'mr-note', style: { minHeight: '2.5em', fontWeight: '600', color: 'var(--danger)' } });
  host.appendChild(status);
  const preview = el('div', { class: 'md' }, el('div', { class: 'callout' }, mdElement(`**下一課預告：阻抗控制。** 不再切成運動／力兩個子空間，而是讓末端整體表現得像質量—彈簧—阻尼器：$M\\ddot x + B\\dot x + Kx = f_{ext}$。碰到桌面時，壓力自然由「彈簧被壓多深」決定。`)));
  preview.style.display = 'none';
  host.appendChild(preview);
  const constraintTable = mdElement(`| 方向 | 自然約束（桌面決定） | 人工約束（任務指定） |
|---|---|---|
| 法向 $n$ | $v_n = 0$：穿不進桌面 | $f_n = f_d$：壓 10 N |
| 切向 $t$ | $f_t = 0$：無摩擦，推不出切向力 | $v_t = v_d(t)$：來回擦拭 |`);
  constraintTable.style.display = 'none';
  host.appendChild(constraintTable);
  legend(host, [
    { color: 'var(--series-4)', label: 'F_sensor（桌面推手臂）' },
    { color: 'var(--series-1)', label: '運動方向 t ／ P F' },
    { color: 'var(--series-7)', label: '施力方向 n ／ (I−P) F' },
    { color: 'var(--warn)', label: '切向目標 s_d（可拖曳 s₀）' },
  ]);
  const [plotHostA, plotHostB] = split(host);
  const seriesF = () => [{ name: 'f（實際）', color: cssVar('--series-4') }, { name: 'f_d', color: cssVar('--fg-muted'), dash: [5, 4] }];
  const seriesE = () => [{ name: 'e_s', color: cssVar('--series-1') }, { name: 'h', color: cssVar('--series-6') }];
  const plotF = new Plot2D(plotHostA, { title: '法向力 f (N)', xLabel: 't (s)', height: 160, xWindow: 8, yMinSpan: 20, series: seriesF() });
  const plotE = new Plot2D(plotHostB, { title: '切向誤差 e_s 與離桌高度 h (mm)', xLabel: 't (s)', height: 160, xWindow: 8, yMinSpan: 2, series: seriesE() });
  const offTheme = ctx.onThemeChange(() => { plotF.setOptions({ series: seriesF() }); plotE.setOptions({ series: seriesE() }); });

  const panel = controlPanel(host, { compact: true });
  const r1 = row(panel);
  const pbk = playback(r1, {
    playing: true,
    onPlay: () => { if (!blownUp) loop.start(); },
    onPause: () => { loop.stop(); },
    onReset: () => resetSim(),
    onSpeed: (s) => { loop.speed = s; },
  });
  const conflictBtn = button(r1, {
    label: '⚠ 錯誤示範：法向也指定位置', small: true,
    title: '在混合控制上再加一個法向位置 PID（目標：模型中的桌面高度）',
    onClick: () => {
      chips?.clear();
      applyParams({ mode: 'conflict', delta: Math.abs(sim.p.delta) < 0.002 ? 0.01 : sim.p.delta, amp: 0.25, freq: sim.p.freq || 0.25 }, true);
    },
  });
  const r2 = row(panel);
  const modeSel = select<Mode>(r2, {
    label: '控制器',
    options: (Object.keys(MODE_LABEL) as Mode[]).map((m) => ({ value: m, label: MODE_LABEL[m] })),
    value: sim.p.mode,
    onChange: (m) => { applyParams({ mode: m }, false); chips?.clear(); },
  });
  const chipHost = row(panel);
  let chips: PresetChipsHandle | null = null;

  const sKe = slider(panel, {
    label: '桌面剛度 k_e', min: 3, max: 5.3, step: 0.05, value: Math.log10(sim.p.ke), format: (v) => fmtStiff(10 ** v),
    onInput: (v) => { sim.p.ke = 10 ** v; chips?.clear(); },
  });
  const sDelta = slider(panel, {
    label: '桌面高度誤差 Δ', title: '真實桌面高度減去控制器模型中的高度', min: -10, max: 20, step: 0.5, value: sim.p.delta * 1000, unit: 'mm',
    onInput: (v) => { sim.p.delta = v / 1000; chips?.clear(); },
  });
  const sAlpha = angleSlider(panel, {
    label: '桌面傾角 α', value: sim.p.alpha, min: (-15 * Math.PI) / 180, max: (15 * Math.PI) / 180,
    onInput: (v) => { sim.p.alpha = v; chips?.clear(); resetSim(); },
  });
  const sFd = slider(panel, { label: '目標壓力 f_d', min: 0, max: 30, step: 0.5, value: sim.p.fd, unit: 'N', onInput: (v) => { sim.p.fd = v; chips?.clear(); } });
  const sKfp = slider(panel, { label: '力比例增益 K_fp', min: 0, max: 4, step: 0.1, value: sim.p.Kfp, onInput: (v) => { sim.p.Kfp = v; chips?.clear(); } });
  const sKfi = slider(panel, { label: '力積分增益 K_fi', min: 0, max: 40, step: 1, value: sim.p.Kfi, unit: '1/s', onInput: (v) => { sim.p.Kfi = v; chips?.clear(); } });
  const sKd = slider(panel, { label: '速度阻尼 K_damp', min: 0, max: 300, step: 5, value: sim.p.Kdamp, unit: 'N·s/m', onInput: (v) => { sim.p.Kdamp = v; chips?.clear(); } });
  const sFreq = slider(panel, {
    label: '擦拭頻率', title: '切向目標 s_d(t) = s₀ + 0.25 sin(2πft)', min: 0, max: 0.6, step: 0.05, value: sim.p.freq, unit: 'Hz',
    onInput: (v) => { sim.p.freq = v; sim.p.amp = v > 0 ? 0.25 : 0; chips?.clear(); },
  });

  const mats = el('div', { style: { display: 'flex', flexWrap: 'wrap', alignItems: 'center', gap: '0.15rem 0.3rem', margin: '0.35rem 0 0.2rem' } });
  host.appendChild(mats);
  const mA = matrixView(mats, { label: 'A = nᵀ =', digits: 2, title: '平面桌面在 {s} 中的約束列' });
  const mAj = matrixView(mats, { label: 'A(θ) = nᵀJ(θ) =', digits: 2, title: '同一個約束，用關節速度表示' });
  const mL = matrixView(mats, { label: 'Λ(θ) =', digits: 2, title: '任務空間質量矩陣（kg）' });
  const mP = matrixView(mats, { label: 'P =', digits: 2 });
  const mIP = matrixView(mats, { label: 'I − P =', digits: 2 });

  const info = kv(host, ['控制器', '接觸', 'f / f_d', 'F_sensor（桌面→手臂）', 'F_tip = −F_sensor', '切向誤差 e_s', '力積分 K_fi∫f_e', 'σ_min(J)']);
  const testInfo = kv(host, ['測試力 F', 'P F（產生運動）', '(I − P) F（壓桌面）', '加速度 Λ⁻¹P F']);
  testInfo.el.style.display = 'none';
  testInfo.el.style.marginTop = '0.3rem';
  const bars = torqueBars(host, { labels: ['τ₁', 'τ₂'], colors: ['var(--series-1)', 'var(--series-2)'], max: TAU_MAX, limit: TAU_MAX * 0.995, unit: 'N·m', digits: 1, title: `關節力矩（上限 ±${TAU_MAX} N·m）` });
  note(host, '接觸模型：單邊彈簧—阻尼（離開桌面時力為 0）、桌面無摩擦，以 0.2 ms 子步積分。擦拭振幅 0.25 m。控制器每 1 ms 取樣一次，使用上一拍的量測值。Λ 在接近奇異時以 (Λ⁻¹ + εI)⁻¹ 正則化。');

  // ---- canvas handles ----------------------------------------------------
  const surfPoint = (s: number): Pt => {
    const { t, n } = surfaceAxes(sim.p.alpha);
    return [SURF_C[0] + s * t[0] + sim.p.delta * n[0], SURF_C[1] + s * t[1] + sim.p.delta * n[1]];
  };
  const targetHandle: Handle = cv.addHandle({
    get: () => surfPoint(sim.p.s0),
    set: (p) => {
      const { t } = surfaceAxes(sim.p.alpha);
      sim.p.s0 = clamp((p[0] - SURF_C[0]) * t[0] + (p[1] - SURF_C[1]) * t[1], -0.35, 0.35);
      cv.requestRender();
    },
    label: 's₀', radius: 7,
  });
  const testHandle: Handle = cv.addHandle({
    get: () => add(tip, Ftest, TEST_SCALE),
    set: (p) => { Ftest = capLen([(p[0] - tip[0]) / TEST_SCALE, (p[1] - tip[1]) / TEST_SCALE], 25); chips?.clear(); cv.requestRender(); },
    label: 'F', radius: 7, enabled: false,
  });

  // ---- drawing -----------------------------------------------------------
  cv.draw = (c) => {
    const muted = cssVar('--fg-muted');
    const cT = cssVar('--series-1', '#2563eb'), cN = cssVar('--series-7', '#db2777'), cF = cssVar('--series-4', '#d97706');
    const warn = cssVar('--warn', '#b45309');
    targetHandle.color = warn;
    targetHandle.enabled = sim.p.mode !== 'force' && step !== 3;
    testHandle.color = muted;
    c.grid(0.25);
    c.line([WORLD.xmin - 1, FLOOR_Y], [WORLD.xmax + 1, FLOOR_Y], { color: muted, width: 2 });
    c.polygon([[-0.1, FLOOR_Y], [0.1, FLOOR_Y], [0.06, -0.06], [-0.06, -0.06]], { fill: cssVar('--bg-sunken'), color: muted, width: 1 });
    // Table: slab along the real surface plus two legs.
    const { t, n } = surfaceAxes(sim.p.alpha);
    const a = surfPoint(-0.62), b = surfPoint(0.62);
    const thick = 0.05;
    const a2 = add(a, n, -thick), b2 = add(b, n, -thick);
    const soft = Math.log10(sim.p.ke) < 3.8;
    c.polygon([a, b, b2, a2], { fill: soft ? cssVar('--warn-soft') : cssVar('--bg-sunken'), color: muted, width: 1.5 });
    for (const s of [-0.5, 0.5]) {
      const top = add(surfPoint(s), n, -thick);
      c.line(top, [top[0], FLOOR_Y], { color: muted, width: 4 });
    }
    c.text([WORLD.xmax - 0.03, WORLD.ymax - 0.05], `桌面剛度 k_e = ${fmtStiff(sim.p.ke)}`, { color: muted, align: 'right' });
    if (Math.abs(sim.p.delta) > 5e-4) {
      const m1: Pt = [SURF_C[0] - 0.62 * t[0], SURF_C[1] - 0.62 * t[1]];
      const m2: Pt = [SURF_C[0] + 0.62 * t[0], SURF_C[1] + 0.62 * t[1]];
      c.line(m1, m2, { color: cssVar('--accent'), width: 1.5, dash: [6, 4] });
      c.text(add(m1, n, sim.p.delta > 0 ? -0.09 : 0.08), `控制器以為的桌面（Δ = ${fmt(sim.p.delta * 1000, 1)} mm）`, { color: cssVar('--accent'), align: 'left' });
    }
    // Instantaneous tangential target.
    const sd = sim.info?.sd ?? sim.p.s0;
    const pd = surfPoint(sd);
    c.line(add(pd, n, -0.04), add(pd, n, 0.04), { color: warn, width: 3 });

    // Arm (drawPlanarArm returns base, elbow and tip).
    const pts = drawPlanarArm(c, [1, 1], sim.theta, { eeColor: cssVar('--accent') });
    tip = [pts[2][0], pts[2][1]];
    const inf = sim.info;

    // Subspace directions (steps ≥ 3 in the narrative, index ≥ 2).
    if (step >= 2) {
      c.line(add(tip, t, -0.32), add(tip, t, 0.32), { color: cT, width: 1.5, dash: [5, 4] });
      c.line(add(tip, n, -0.12), add(tip, n, 0.3), { color: cN, width: 1.5, dash: [5, 4] });
      c.text(add(tip, t, 0.38), 't', { color: cT, font: 'bold 13px sans-serif' });
      c.text(add(tip, n, 0.35), 'n', { color: cN, font: 'bold 13px sans-serif' });
    }
    // Sensor force (table → arm) along +n (hidden while the test wrench is shown).
    if (sim.f > 0.05 && step !== 3) {
      const v = capLen([sim.f * n[0] * F_SCALE, sim.f * n[1] * F_SCALE], 0.55);
      c.arrow(tip, add(tip, v), { color: cF, width: 3 });
      c.text(add(add(tip, v), n, 0.05), `${fmt(sim.f, 1)} N`, { color: cF });
    }
    // Controller wrench split (hybrid / conflict narrative steps).
    if ((step === 4 || step === 5) && inf && (sim.p.mode === 'hybrid' || sim.p.mode === 'conflict')) {
      // Offset sideways so the two loops' wrenches and the sensor arrow do not overlap.
      const fm = capLen([inf.Fmotion[0] * F_SCALE, inf.Fmotion[1] * F_SCALE], 0.32);
      const ff = capLen([inf.Fforce[0] * F_SCALE, inf.Fforce[1] * F_SCALE], 0.32);
      const om = add(tip, t, -0.05), of = add(tip, t, 0.05);
      if (Math.hypot(fm[0], fm[1]) > 0.02) {
        c.arrow(om, add(om, fm), { color: cT, width: 2.5 });
        c.text(add(om, fm), `${sim.p.mode === 'conflict' ? '位置迴路' : '運動迴路'} ${fmt(Math.hypot(inf.Fmotion[0], inf.Fmotion[1]), 0)} N`, { color: cT, align: 'right', dx: -6 });
      }
      if (Math.hypot(ff[0], ff[1]) > 0.02) {
        c.arrow(of, add(of, ff), { color: cN, width: 2.5 });
        c.text(add(of, ff), `力迴路 ${fmt(Math.hypot(inf.Fforce[0], inf.Fforce[1]), 0)} N`, { color: cN, align: 'left', dx: 6 });
      }
    }
    // Test wrench decomposition.
    if (step === 3 && inf) {
      const PF = matVec(inf.P, Ftest) as Vec2;
      const IPF = matVec(inf.IP, Ftest) as Vec2;
      const acc = matVec(inf.tm.Linv, PF) as Vec2;
      c.arrow(tip, add(tip, Ftest, TEST_SCALE), { color: muted, width: 2.5 });
      c.arrow(tip, add(tip, PF, TEST_SCALE), { color: cT, width: 3 });
      c.arrow(tip, add(tip, IPF, TEST_SCALE), { color: cN, width: 3 });
      c.line(add(tip, PF, TEST_SCALE), add(tip, Ftest, TEST_SCALE), { color: cN, width: 1, dash: [3, 3] });
      c.line(add(tip, IPF, TEST_SCALE), add(tip, Ftest, TEST_SCALE), { color: cT, width: 1, dash: [3, 3] });
      const av = capLen([acc[0] * ACC_SCALE, acc[1] * ACC_SCALE], 0.6);
      c.arrow(add(tip, n, -0.015), add(add(tip, n, -0.015), av), { color: cT, width: 1.5, dash: [4, 3] });
      c.text(add(tip, PF, TEST_SCALE), 'PF', { color: cT, dx: 14, dy: -8 });
      c.text(add(tip, IPF, TEST_SCALE), '(I−P)F', { color: cN, dx: 24, dy: 0 });
    }
    c.overlay.textContent = `t = ${fmt(sim.t, 2)} s   f = ${fmt(sim.f, 1)} N   e_s = ${fmt(((inf?.sd ?? 0) - (inf?.s ?? 0)) * 1000, 1)} mm`;
  };

  // ---- simulation loop ---------------------------------------------------
  function logSample(): void {
    const inf = sim.info;
    const es = inf ? (inf.sd - inf.s) * 1000 : 0;
    plotF.push(sim.t, [sim.f, sim.p.fd]);
    plotE.push(sim.t, [clamp(es, -100, 100), clamp(sim.h * 1000, -5, 60)]);
    if (wasContact && !sim.contact) lossTimes.push(sim.t);
    if (sim.contact) lastContactT = sim.t;
    wasContact = sim.contact;
    while (lossTimes.length && lossTimes[0] < sim.t - 1.5) lossTimes.shift();
  }

  // Fixed-step physics inside a rAF loop with a CPU budget: if the machine cannot keep up, the
  // simulation runs in slow motion instead of freezing the page.
  let blownUp = false;
  const loop = createLoop((dt) => {
    if (blownUp) return;
    const t0 = performance.now();
    let remaining = dt;
    while (remaining > 1e-9) {
      const h = Math.min(LOG_DT, remaining);
      sim.advance(h);
      logSample();
      remaining -= h;
      if (!sim.healthy()) {
        blownUp = true;
        loop.stop();
        pbk.setPlaying(false, { silent: true });
        status.textContent = '⚠ 失控：末端已飛離桌面，模擬暫停。調整增益後按「⟲ 重設」重新放下。';
        break;
      }
      if (performance.now() - t0 > FRAME_BUDGET_MS) break;
    }
    cv.requestRender();
    updateUi(false);
  });

  /** Put the arm back above the table and start a fresh run (always plays). */
  function resetSim(): void {
    loop.stop();
    blownUp = false;
    sim.reset();
    plotF.clear(); plotE.clear();
    lossTimes.length = 0; wasContact = false; lastContactT = 0;
    status.textContent = '';
    loop.start();
    pbk.setPlaying(true, { silent: true });
    cv.requestRender();
    updateUi(true);
  }

  function syncControls(): void {
    const p = sim.p;
    modeSel.set(p.mode, { silent: true });
    sKe.set(Math.log10(p.ke), { silent: true });
    sDelta.set(p.delta * 1000, { silent: true });
    sAlpha.set(p.alpha, { silent: true });
    sFd.set(p.fd, { silent: true });
    sKfp.set(p.Kfp, { silent: true });
    sKfi.set(p.Kfi, { silent: true });
    sKd.set(p.Kdamp, { silent: true });
    sFreq.set(p.amp > 0 ? p.freq : 0, { silent: true });
  }

  /** Update parameters (and the controls); optionally put the arm back above the table. */
  function applyParams(q: Partial<SimParams>, reset: boolean): void {
    const modeChanged = q.mode != null && q.mode !== sim.p.mode;
    Object.assign(sim.p, q);
    if (modeChanged) sim.clearIntegrators();
    syncControls();
    if (reset) resetSim(); else { cv.requestRender(); updateUi(true); }
  }

  function updateUi(force: boolean): void {
    const now = performance.now();
    pbk.setTime(sim.t);
    if (!force && now - lastUi < 90) return;
    lastUi = now;
    const inf = sim.info;
    const p = sim.p;
    const { n } = surfaceAxes(p.alpha);
    bars.set(sim.tau);
    info.set('控制器', MODE_LABEL[p.mode]);
    info.set('接觸', sim.contact ? `接觸中（穿入 ${fmt(-sim.h * 1000, 2)} mm）` : `離開桌面（高 ${fmt(sim.h * 1000, 1)} mm）`);
    info.set('f / f_d', `${fmt(sim.f, 2)} N / ${fmt(p.fd, 1)} N`);
    info.set('F_sensor（桌面→手臂）', `(${fmt(sim.f * n[0], 2)}, ${fmt(sim.f * n[1], 2)}) N`);
    info.set('F_tip = −F_sensor', `(${fmt(-sim.f * n[0], 2)}, ${fmt(-sim.f * n[1], 2)}) N`);
    if (inf) {
      info.set('切向誤差 e_s', `${fmt((inf.sd - inf.s) * 1000, 2)} mm`);
      const satF = Math.abs(Math.abs(inf.intF) - I_F_MAX) < 0.05 ? '（已飽和）' : '';
      info.set('力積分 K_fi∫f_e', p.mode === 'position' ? '—（位置控制沒有力迴路）' : `${fmt(inf.intF, 1)} N${satF}`);
      const sm = sigmaMin(sim.theta);
      info.set('σ_min(J)', `${fmt(sm, 3)}${sm < 0.15 ? '  ⚠ 接近奇異（Λ 已正則化）' : ''}`);
      mA.set([inf.A]);
      mAj.set([inf.Ajoint]);
      mL.set(inf.tm.L);
      mP.set(inf.P);
      mIP.set(inf.IP);
      if (step === 3) {
        const PF = matVec(inf.P, Ftest);
        const IPF = matVec(inf.IP, Ftest);
        const acc = matVec(inf.tm.Linv, PF);
        testInfo.set('測試力 F', `(${fmt(Ftest[0], 1)}, ${fmt(Ftest[1], 1)}) N`);
        testInfo.set('P F（產生運動）', `(${fmt(PF[0], 2)}, ${fmt(PF[1], 2)}) N`);
        testInfo.set('(I − P) F（壓桌面）', `(${fmt(IPF[0], 2)}, ${fmt(IPF[1], 2)}) N`);
        testInfo.set('加速度 Λ⁻¹P F', `(${fmt(acc[0], 2)}, ${fmt(acc[1], 2)}) m/s²，法向分量 ${fmt(acc[0] * n[0] + acc[1] * n[1], 3)}`);
      }
    }
    // Status line (the blow-up message wins).
    if (blownUp) return;
    const msgs: string[] = [];
    if (p.mode === 'conflict' && inf) {
      const pf = inf.pushForce;
      msgs.push(`⚠ 法向同時指定了位置與力：位置迴路要壓 ${fmt(inf.pushPos, 0)} N，力迴路卻要${pf >= 0 ? '壓' : '拉'} ${fmt(Math.abs(pf), 0)} N`);
      if (sim.f > 3 * Math.max(1, p.fd)) msgs.push(`壓力已達 ${fmt(sim.f, 0)} N`);
      if (!sim.contact && sim.t - lastContactT > 0.3 && sim.t > 1) msgs.push('工具已離開桌面');
    } else if (lossTimes.length >= 3) {
      msgs.push('⚠ 末端在桌面上彈跳：力迴路對這個剛度太積極（降低 K_fp 或加大 K_damp）');
    } else if (p.mode === 'position' && sim.t > 0.5 && !sim.contact) {
      msgs.push('位置控制：末端停在半空，根本沒碰到桌面');
    }
    if (inf?.saturated) msgs.push('關節力矩飽和');
    status.textContent = msgs.join('，');
  }

  // ---- per-step configuration ---------------------------------------------
  type Chip = { label: string; q: Partial<SimParams>; reset?: boolean; test?: Vec2; title?: string };
  const tn = (alpha: number, ct: number, cn: number): Vec2 => {
    const { t, n } = surfaceAxes(alpha);
    return [ct * t[0] + cn * n[0], ct * t[1] + cn * n[1]];
  };
  const STEP_SETUP: { q: Partial<SimParams>; chips: Chip[] }[] = [
    {
      q: { mode: 'position', ke: 2e4, delta: 0.01, alpha: 0, s0: 0, amp: 0.25, freq: 0.25 },
      chips: [
        { label: 'Δ = +1 cm（桌面較高）', q: { delta: 0.01 }, reset: true },
        { label: 'Δ = −0.5 cm（桌面較低）', q: { delta: -0.005 }, reset: true },
        { label: 'Δ = +2 cm', q: { delta: 0.02 }, reset: true },
        { label: '硬桌面 2×10⁵ N/m', q: { ke: 2e5 }, reset: true },
      ],
    },
    {
      q: { mode: 'force', ke: 2e4, delta: 0, alpha: 0, s0: 0, amp: 0, freq: 0, fd: 10, Kfp: 1, Kfi: 10, Kdamp: 60 },
      chips: [
        { label: '軟桌面 2×10³', q: { ke: 2e3, Kfp: 1, Kdamp: 60 }, reset: true },
        { label: '中等桌面 2×10⁴', q: { ke: 2e4, Kfp: 1, Kdamp: 60 }, reset: true },
        { label: '硬桌面 2×10⁵（看彈跳）', q: { ke: 2e5, Kfp: 1, Kdamp: 60 }, reset: true },
        { label: '硬桌面＋K_fp = 0.2', q: { ke: 2e5, Kfp: 0.2, Kdamp: 60 }, reset: true },
        { label: '拿掉阻尼 K_damp = 0', q: { ke: 2e4, Kfp: 1, Kdamp: 0 }, reset: true },
      ],
    },
    {
      q: { mode: 'hybrid', ke: 2e4, delta: 0, alpha: 0, s0: 0, amp: 0.25, freq: 0.25, fd: 10, Kfp: 1, Kfi: 10, Kdamp: 60 },
      chips: [
        { label: 'α = 0°', q: { alpha: 0 }, reset: true },
        { label: 'α = +10°', q: { alpha: (10 * Math.PI) / 180 }, reset: true },
        { label: 'α = −10°', q: { alpha: (-10 * Math.PI) / 180 }, reset: true },
      ],
    },
    {
      q: { mode: 'hybrid', ke: 2e4, delta: 0, s0: 0, amp: 0, freq: 0, fd: 10, Kfp: 1, Kfi: 10, Kdamp: 60 },
      chips: [
        { label: '測試力：純法向', q: {}, test: [0, -10] },
        { label: '測試力：純切向', q: {}, test: [10, 0] },
        { label: '測試力：斜向', q: {}, test: [8, -7] },
        { label: '換位置 s₀ = +0.3 m', q: { s0: 0.3 } },
        { label: '桌面 α = +10°', q: { alpha: (10 * Math.PI) / 180 }, reset: true },
      ],
    },
    {
      q: { mode: 'hybrid', ke: 2e4, delta: 0, s0: 0, amp: 0.25, freq: 0.25, fd: 10, Kfp: 1, Kfi: 10, Kdamp: 60 },
      chips: [
        { label: '慢速擦拭 0.25 Hz', q: { amp: 0.25, freq: 0.25 } },
        { label: '快速擦拭 0.5 Hz', q: { amp: 0.25, freq: 0.5 } },
        { label: 'f_d = 20 N', q: { fd: 20 } },
        { label: '傾斜桌面 α = −10°', q: { alpha: (-10 * Math.PI) / 180 }, reset: true },
        { label: '桌面高度誤差 Δ = +1 cm', q: { delta: 0.01 }, reset: true },
      ],
    },
    {
      q: { mode: 'conflict', ke: 2e4, delta: 0.01, alpha: 0, s0: 0, amp: 0.25, freq: 0.25, fd: 10, Kfp: 1, Kfi: 10, Kdamp: 60 },
      chips: [
        { label: 'Δ = +1 cm（力暴增）', q: { mode: 'conflict', delta: 0.01 }, reset: true },
        { label: 'Δ = −1 cm（離開桌面）', q: { mode: 'conflict', delta: -0.01 }, reset: true },
        { label: '改回混合控制（同樣的 Δ）', q: { mode: 'hybrid' }, reset: true },
      ],
    },
  ];

  const FOCUS: (() => HTMLElement[])[] = [
    () => [plotF.el, sDelta.el],
    () => [plotF.el, sKe.el, sKfp.el, sKd.el],
    () => [constraintTable, mA.el, mAj.el, sAlpha.el],
    () => [cv.el, mP.el, mIP.el],
    () => [plotF.el, plotE.el],
    () => [conflictBtn, plotF.el, bars.el],
  ];
  let focused: HTMLElement[] = [];

  function onStep(i: number): void {
    const k = Math.max(0, Math.min(STEP_SETUP.length - 1, i));
    if (k === step) return;
    step = k;
    const setup = STEP_SETUP[k];
    applyParams({ ...DEFAULT_PARAMS, ...setup.q }, true);
    chips?.el.remove();
    chips = presetChips(chipHost, setup.chips.map((ch) => ({
      label: ch.label,
      title: ch.title,
      apply: () => {
        if (ch.test) Ftest = tn(sim.p.alpha, ch.test[0], ch.test[1]);
        applyParams(ch.q, !!ch.reset);
      },
    })), { label: '本步實驗：' });
    testHandle.enabled = k === 3;
    testInfo.el.style.display = k === 3 ? '' : 'none';
    preview.style.display = k === 5 ? '' : 'none';
    constraintTable.style.display = k === 2 ? '' : 'none';
    focused.forEach((f) => f.classList.remove('mr-focus'));
    focused = FOCUS[k]();
    focused.forEach((f) => f.classList.add('mr-focus'));
    cv.requestRender();
  }

  onStep(ctx.currentStep);
  loop.start();

  return {
    onStep,
    destroy() {
      loop.stop();
      offTheme();
      cv.destroy();
      plotF.destroy();
      plotE.destroy();
    },
  };
}

const lesson: Lesson = { meta, steps, quiz, mount };
export default lesson;
