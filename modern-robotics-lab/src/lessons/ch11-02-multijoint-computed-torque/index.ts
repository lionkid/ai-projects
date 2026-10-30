import type { Lesson, LessonContext, LessonHandle, LessonStep, QuizQuestion } from '@/types';
import {
  el, cssVar, fmt, controlPanel, row, slider, toggle, select, playback, simTitle, split,
  Canvas2D, Plot2D, plotGroup, presetChips, createLoop, drawPlanarArm, torqueBars,
  type Pt, type Marker, type SeriesSpec,
} from '@/ui';
import { planar2RDynamics } from '@/mr';
import { meta } from './meta';
import {
  HOLD, LINKS, ROUTES, armParams, designGains, secondOrderFree, simulateArm,
  type ArmTrial, type ArmTrialOptions, type ControllerKind, type RouteKind,
} from './armControl';

const steps: LessonStep[] = [
  {
    title: '兩個獨立 PD 的下場',
    md: `把上一課的單關節 PD 原封不動搬到 2R 手臂上：每個關節只看自己的誤差，

$$\\tau_i = \\bar M_{ii}\\big(K_p\\,\\theta_{e,i} + K_d\\,\\dot\\theta_{e,i}\\big), \\qquad i = 1, 2 .$$

為了公平比較，本課所有控制器都用同一組 $\\omega_n$、$\\zeta$ 設計（$K_p = \\omega_n^2$、$K_d = 2\\zeta\\omega_n$）；PD 再乘上各關節的名目慣量 $\\bar M_{ii}$，換算成力矩。

結果出現兩個毛病：靜止時手臂在重力下**下垂**；運動時關節互相拉扯。路徑 A⇄B 中關節 2 的目標始終是 90°，它卻被關節 1 的擺動甩開。`,
    simHint: '播放並看誤差圖：一開始靜止時 e₁、e₂ 就不是零（下垂）；關節 1 開始擺動後，本來不該動的關節 2 也出現大誤差。',
  },
  {
    title: '補上重力',
    md: `下垂來自重力，那就用模型算出重力力矩，直接加進指令：

$$\\tau = \\bar M\\big(K_p\\theta_e + K_d\\dot\\theta_e\\big) + \\tilde g(\\theta).$$

（波浪號表示「控制器心中的模型」。）靜止時 PD 不必再拉長彈簧去撐重量，靜態誤差就消失了；對固定目標，還能用能量（Lyapunov）函數證明誤差一定收斂到零。

但手臂一動起來，慣性力 $M(\\theta)\\ddot\\theta$ 與科氏／離心力 $c(\\theta,\\dot\\theta)$ 仍然只能靠誤差回饋去扛，而它們正是把關節耦合在一起的來源。`,
    simHint: '對照右上表格中 PD 與 PD+g 的末端誤差；再把移動時間 T_f 縮短（動作變快），看 PD+g 的誤差如何跟著加減速一起長大。',
  },
  {
    title: '計算力矩：先抵消，再當線性系統控制',
    md: `**計算力矩法（computed torque）**先用模型把非線性動力學整個抵消，再把剩下的部分當成線性系統來設計：

$$\\tau = \\tilde M(\\theta)\\Big(\\ddot\\theta_d + K_p\\theta_e + K_i\\!\\int\\!\\theta_e\\,dt + K_d\\dot\\theta_e\\Big) + \\tilde c(\\theta,\\dot\\theta) + \\tilde g(\\theta).$$

若模型正確，代入 $\\tau = M\\ddot\\theta + c + g$：兩邊的 $c$、$g$ 互相抵銷，再約掉可逆的 $M$，每個關節只剩自己的線性方程 $\\ddot\\theta_{e,i} + K_d\\dot\\theta_{e,i} + K_p\\theta_{e,i} = 0$（取 $K_i = 0$）。這就是**回饋線性化（feedback linearization）**：耦合被抵消、誤差被解耦，上一課的 $\\omega_n$、$\\zeta$ 分析可以直接用在每個關節上。`,
    simHint: '已加入關節 1 的起始偏差 20°。e₁ 的實線與灰色虛線（理想二階響應）重合，e₂ 則一直貼著零：兩個關節的誤差被解耦了。切回 PD+g 比較看看。',
  },
  {
    title: '前饋與回饋各做什麼',
    md: `把計算力矩的輸出拆成兩塊：

- **模型前饋** $\\tilde M\\ddot\\theta_d + \\tilde c + \\tilde g$：依照期望的加速度，算出「本來就需要」的力矩；
- **回饋** $\\tilde M(K_p\\theta_e + K_d\\dot\\theta_e)$：只負責修正剩下的偏差。

模型準確時，幾乎所有力矩都由前饋提供，回饋只做微調（看長條圖與表格中的「回饋占比」）。PD+g 只有重力屬於前饋，加速與減速所需的力矩都得等誤差先長出來、再由回饋補上，所以動作越快、誤差越大。`,
    simHint: '移動時間已縮短為 0.6 s。力矩圖中的實線（前饋）幾乎包辦一切、虛線（回饋）貼近零；切到 PD+g，看回饋如何被迫扛下加減速。',
  },
  {
    title: '模型不準怎麼辦',
    md: `質量與慣量只能估計。模型有落差時抵消就不完全，誤差方程多出一個由模型誤差造成的擾動：

$$\\ddot\\theta_e + K_d\\dot\\theta_e + K_p\\theta_e = \\tilde M^{-1}\\big[(M-\\tilde M)\\ddot\\theta + (c-\\tilde c) + (g-\\tilde g)\\big].$$

回饋會吸收大部分的不匹配，所以性能是**逐漸變差**，不會一下子崩潰；動作越快，$(M-\\tilde M)\\ddot\\theta$ 這項越大。停住時剩下的偏差主要來自重力估錯，加上積分項 $K_i$ 就能把它消掉。`,
    simHint: '模型質量誤差已設為 −20%（控制器低估了質量）。看停住時的殘餘誤差，再勾選「積分項 Ki」；也試試 +20% 與更短的 T_f。',
  },
  {
    title: '任務空間控制與下一課',
    md: `也可以直接控制末端位置 $x$：由 $\\ddot x = J\\ddot\\theta + \\dot J\\dot\\theta$ 反解出關節加速度，再送進計算力矩，

$$\\ddot\\theta_{cmd} = J^{-1}(\\theta)\\big(\\ddot x_d + K_p x_e + K_d\\dot x_e - \\dot J\\dot\\theta\\big).$$

末端會走出漂亮的直線，代價是要反解 $J$。路徑 C⇄D 從基座上方 0.15 m 掠過，手臂接近摺疊的奇異姿態，關節速度與力矩暴增。用阻尼最小平方 $J^\\T(JJ^\\T+\\lambda^2 I)^{-1}$ 取代 $J^{-1}$ 能壓低關節速度，但末端會偏離直線；關節空間控制沒有這個問題，只是末端不走直線。

下一課要控制的不只是運動，還有手臂與環境之間的**力**。`,
    simHint: '比較表格中兩種計算力矩的「最大關節速度」；再把 DLS 阻尼 λ 往上拉，看關節速度與末端誤差如何取捨。',
  },
];

const quiz: QuizQuestion[] = [
  {
    question: '使用 PD＋重力補償的 2R 手臂靜止時誤差為零。它一開始快速加速，誤差就冒出來了。這個誤差主要來自哪裡？',
    options: [
      '重力 $g(\\theta)$ 沒有被補償',
      '控制器沒有提供慣性力 $M(\\theta)\\ddot\\theta_d$ 與速度相關的 $c(\\theta,\\dot\\theta)$，只能等誤差出現後由回饋補上',
      '數值積分的誤差',
      '$K_d$ 太大造成的延遲',
    ],
    answer: 1,
    explanation: '重力已經由 $\\tilde g$ 補上，所以靜止時誤差為零。加速需要的 $M\\ddot\\theta$ 與 $c$ 不在 PD+g 的指令裡，回饋必須先「看到」誤差才能產生這些力矩，因此加速越猛、誤差越大，而且 $M$ 的非對角項還會把一個關節的加速度傳到另一個關節。',
  },
  {
    question: '計算力矩法用了正確的 $\\tilde M$ 與 $\\tilde g$，但完全忽略科氏／離心項（令 $\\tilde c = 0$）。剩下的誤差有什麼特性？',
    options: [
      '多出擾動 $M^{-1}c(\\theta,\\dot\\theta)$，大小約與關節速度平方成正比，高速時最明顯、靜止時消失',
      '多出固定偏差，靜止時最明顯',
      '不會有任何影響，因為 $c$ 永遠很小',
      '系統一定會發散',
    ],
    answer: 0,
    explanation: '$c(\\theta,\\dot\\theta)$ 由 $\\dot\\theta_i\\dot\\theta_j$ 的乘積組成：靜止時為零，速度加倍時大約變成四倍。少補了這一項，誤差方程右邊就多出 $M^{-1}c$，回饋能把它壓小，但動作越快殘差越大。',
  },
  {
    question: '同樣兩個端點，「任務空間計算力矩（末端走直線，從基座附近掠過）」和「關節空間計算力矩（關節插值）」相比，下列哪個描述正確？',
    options: [
      '任務空間：末端走直線，但接近奇異時關節速度與力矩暴增；關節空間：關節運動平順，但末端路徑不是直線',
      '兩者的末端路徑相同，只是計算量不同',
      '關節空間控制在奇異附近一定會發散',
      '任務空間控制不需要 Jacobian',
    ],
    answer: 0,
    explanation: '任務空間控制每一刻都要用 $J^{-1}$（或 DLS）把末端加速度換成關節加速度；$J$ 接近奇異時，維持直線需要極大的關節速度。關節空間控制只在端點做逆運動學，中間照關節插值走，所以不受奇異影響，但末端走的是弧線。',
  },
  {
    question: '模型完全正確的計算力矩控制，$\\omega_n = 8$ rad/s、$\\zeta = 1$。若只有關節 1 有 20° 的起始誤差，關節 2 的誤差會怎樣？',
    options: ['一直保持為零', '先被關節 1 帶偏，再慢慢回到零', '和關節 1 走出一模一樣的曲線', '開始振盪'],
    answer: 0,
    explanation: '回饋線性化後每個關節的誤差各自滿足 $\\ddot\\theta_{e,i} + K_d\\dot\\theta_{e,i} + K_p\\theta_{e,i} = 0$。關節 2 的初始誤差與誤差速度都是零，所以它的解恆為零；$M(\\theta)$ 的耦合已經被 $\\tilde M$ 抵消掉了。',
  },
];

// ---------------------------------------------------------------------------
// Simulation
// ---------------------------------------------------------------------------

const KINDS: ControllerKind[] = ['pd', 'pdg', 'ct', 'task'];
const KIND_LABEL: Record<ControllerKind, string> = {
  pd: 'PD（各關節獨立）', pdg: 'PD＋重力補償', ct: '計算力矩（關節空間）', task: '計算力矩（任務空間）',
};
const KIND_SHORT: Record<ControllerKind, string> = { pd: 'PD', pdg: 'PD+g', ct: '計算力矩（關節）', task: '計算力矩（任務）' };
const OFFSET = (20 * Math.PI) / 180;

interface SimState {
  kind: ControllerKind;
  route: RouteKind;
  Tf: number;
  wn: number;
  zeta: number;
  useKi: boolean;
  massErrPct: number;
  lambda: number;
  offset: boolean;
}

const BASE: SimState = { kind: 'pd', route: 'ab', Tf: 1.2, wn: 8, zeta: 1, useKi: false, massErrPct: 0, lambda: 0, offset: false };

const PRESETS: { label: string; title: string; s: Partial<SimState> }[] = [
  { label: '① PD', title: '各關節獨立 PD', s: { kind: 'pd' } },
  { label: '② PD+g', title: 'PD＋重力補償', s: { kind: 'pdg' } },
  { label: '③ 計算力矩', title: '計算力矩，關節 1 起始偏差 20°', s: { kind: 'ct', offset: true } },
  { label: '④ 前饋/回饋', title: '計算力矩、快速動作 T_f = 0.6 s', s: { kind: 'ct', Tf: 0.6 } },
  { label: '⑤ 模型誤差', title: '計算力矩，模型質量 −20%', s: { kind: 'ct', massErrPct: -20 } },
  { label: '⑥ 任務空間', title: '任務空間計算力矩，路徑經過基座附近', s: { kind: 'task', route: 'cd', Tf: 1.5 } },
];

function mount(host: HTMLElement, ctx: LessonContext): LessonHandle {
  const st: SimState = { ...BASE };
  let trial: ArmTrial;
  let tipPts: Pt[] = [];
  let tipdPts: Pt[] = [];
  let tCur = 0;
  let destroyed = false;

  // ---- layout -------------------------------------------------------------
  simTitle(host, '2R 手臂：四種控制器', '計算力矩');
  const chips = presetChips(host, PRESETS.map((p, i) => ({ label: p.label, title: p.title, apply: () => applyPreset(i) })));
  const [left, right] = split(host);
  const cv = new Canvas2D(left, { aspect: 1.15, world: { xmin: -1.3, xmax: 1.8, ymin: -1.05, ymax: 1.75 }, overlay: true });
  cv.overlay.style.whiteSpace = 'pre';

  const tableWrap = el('div', { class: 'md', style: { fontSize: '0.74rem', overflowX: 'auto' } });
  right.appendChild(tableWrap);
  const table = el('table', { style: { width: '100%', margin: '0 0 0.3rem' } });
  tableWrap.appendChild(table);
  const pad = { padding: '0.12em 0.35em' };
  const head = el('tr', {}, ...['控制器', '末端誤差 RMS', '回饋占比', '最大 |θ̇|'].map((h) => el('th', { style: pad }, h)));
  const rows = new Map<ControllerKind, { tr: HTMLTableRowElement; cells: HTMLTableCellElement[] }>();
  const tbody = el('tbody');
  for (const k of KINDS) {
    const cells = [el('td', { style: pad }, KIND_SHORT[k]), el('td', { style: pad }, '…'), el('td', { style: pad }, '…'), el('td', { style: pad }, '…')];
    const tr = el('tr', {}, ...cells);
    rows.set(k, { tr, cells });
    tbody.appendChild(tr);
  }
  table.append(el('thead', {}, head), tbody);
  const bars = torqueBars(right, {
    labels: ['τ₁ 前饋', 'τ₁ 回饋', 'τ₂ 前饋', 'τ₂ 回饋'],
    max: 30,
    unit: 'N·m',
    digits: 1,
    colors: ['var(--series-1)', 'color-mix(in srgb, var(--series-1) 45%, transparent)', 'var(--series-4)', 'color-mix(in srgb, var(--series-4) 45%, transparent)'],
  });

  const panel = controlPanel(host, { compact: true });
  const topRow = row(panel);
  const kindSel = select<ControllerKind>(topRow, {
    options: KINDS.map((k) => ({ value: k, label: KIND_LABEL[k] })),
    value: st.kind,
    onChange: (v) => { st.kind = v; syncControls(); touched(); },
  });
  const routeSel = select<RouteKind>(topRow, {
    options: [{ value: 'ab', label: '路徑 A⇄B（關節 2 維持 90°）' }, { value: 'cd', label: '路徑 C⇄D（掠過基座上方）' }],
    value: st.route,
    onChange: (v) => { st.route = v; touched(); },
  });
  kindSel.el.title = '控制器';
  routeSel.el.title = '參考路徑';
  const tKi = toggle(topRow, { label: '積分項 Ki', title: 'Ki = 0.5 ωn³（乘上 M̄ 或 M̃）', value: st.useKi, onChange: (v) => { st.useKi = v; touched(); } });
  const tOff = toggle(topRow, { label: '起始偏差 Δθ₁ = 20°', value: st.offset, onChange: (v) => { st.offset = v; touched(); } });
  const sWn = slider(panel, { label: 'ωn', min: 3, max: 15, step: 0.5, value: st.wn, unit: 'rad/s', onInput: (v) => { st.wn = v; touched(); } });
  const sZeta = slider(panel, { label: 'ζ', min: 0.4, max: 1.5, step: 0.05, value: st.zeta, onInput: (v) => { st.zeta = v; touched(); } });
  const sTf = slider(panel, { label: '移動時間 T_f', min: 0.5, max: 3, step: 0.1, value: st.Tf, unit: 's', onInput: (v) => { st.Tf = v; touched(); } });
  const sMass = slider(panel, {
    label: '模型質量誤差', min: -40, max: 40, step: 5, value: st.massErrPct,
    format: (v) => `${v > 0 ? '+' : ''}${fmt(v, 0)} %`,
    onInput: (v) => { st.massErrPct = v; touched(); },
  });
  const sLam = slider(panel, { label: 'DLS 阻尼 λ', min: 0, max: 0.6, step: 0.02, value: st.lambda, onInput: (v) => { st.lambda = v; touched(); } });
  const pbRow = row(panel);
  const pb = playback(pbRow, {
    duration: 5,
    playing: true,
    onPlay: () => loop.start(),
    onPause: () => loop.stop(),
    onReset: () => { tCur = 0; frame(); },
    onSpeed: (s) => { loop.speed = s; },
    onScrub: (t) => { tCur = t; frame(); },
  });

  const gainNote = el('div', { class: 'mr-note', style: { margin: '0 0 0.2rem' } });
  host.appendChild(gainNote);
  const errPlot = new Plot2D(host, { title: '追蹤誤差', yLabel: 'θ_e (°)', series: [{ name: 'e₁' }, { name: 'e₂' }], height: 115 });
  const auxPlot = new Plot2D(host, { title: '力矩', xLabel: 't (s)', yLabel: 'τ (N·m)', series: [{ name: 'τ' }], height: 115 });
  const group = plotGroup([errPlot, auxPlot]);

  // ---- helpers ------------------------------------------------------------
  const trialOpts = (kind: ControllerKind): ArmTrialOptions => ({
    kind, route: st.route, Tf: st.Tf, wn: st.wn, zeta: st.zeta, useKi: st.useKi, massError: st.massErrPct / 100,
    lambda: st.lambda, offset: st.offset ? [OFFSET, 0] : [0, 0],
  });
  const idxAt = (t: number) => Math.max(0, Math.min(trial.t.length - 1, Math.round(t / (trial.t[1] - trial.t[0] || 1))));
  const deg = (x: number) => (x * 180) / Math.PI;

  function recompute(): void {
    trial = simulateArm(trialOpts(st.kind));
    tipPts = trial.tip.map((p): Pt => [p[0], p[1]]);
    tipdPts = trial.tipd.map((p): Pt => [p[0], p[1]]);
    pb.setDuration(trial.T);
    group.setXRange([0, trial.T]);
    if (tCur > trial.T) tCur = 0;
    let peak = 1;
    for (let i = 0; i < trial.t.length; i++) for (const v of [trial.ff[i][0], trial.ff[i][1], trial.fb[i][0], trial.fb[i][1]]) peak = Math.max(peak, Math.abs(v));
    bars.setMax(Math.ceil(peak / 5) * 5);
    updatePlots();
    updateGains();
    fillRow(st.kind, trial);
    scheduleTable();
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

  // The comparison table runs the other three controllers one per timer tick so a slider drag
  // never blocks for more than one trial.
  let tableJob = 0;
  let tableTimer = 0;
  function scheduleTable(): void {
    const job = ++tableJob;
    window.clearTimeout(tableTimer);
    for (const k of KINDS) if (k !== st.kind) rows.get(k)!.cells.slice(1).forEach((c) => { c.style.opacity = '0.45'; });
    const todo = KINDS.filter((k) => k !== st.kind);
    const next = () => {
      if (destroyed || job !== tableJob) return;
      const k = todo.shift();
      if (!k) return;
      fillRow(k, simulateArm(trialOpts(k)));
      tableTimer = window.setTimeout(next, 0);
    };
    tableTimer = window.setTimeout(next, 120);
    for (const k of KINDS) {
      const r = rows.get(k)!;
      const active = k === st.kind;
      r.tr.style.fontWeight = active ? '700' : '';
      r.tr.style.background = active ? 'var(--accent-soft)' : '';
    }
  }
  function fillRow(k: ControllerKind, tr: ArmTrial): void {
    const r = rows.get(k)!;
    const s = tr.stats;
    if (tr.diverged) { r.cells[1].textContent = '發散'; r.cells[2].textContent = '—'; r.cells[3].textContent = '—'; }
    else {
      r.cells[1].textContent = `${fmt(s.tipRms * 1000, s.tipRms < 0.01 ? 1 : 0)} mm`;
      r.cells[2].textContent = `${fmt(100 * s.fbShare, s.fbShare < 0.1 ? 1 : 0)} %`;
      r.cells[3].textContent = `${fmt(s.maxRate, 1)} rad/s`;
    }
    r.cells[3].style.color = s.maxRate > 8 ? 'var(--danger)' : '';
    r.cells.slice(1).forEach((c) => { c.style.opacity = ''; });
  }

  function updateGains(): void {
    const g = designGains(st.wn, st.zeta, st.useKi);
    if (st.kind === 'pd' || st.kind === 'pdg') {
      const route = ROUTES[st.route];
      const qMid = [(route.qa[0] + route.qb[0]) / 2, (route.qa[1] + route.qb[1]) / 2];
      const M = planar2RDynamics(armParams(1 + st.massErrPct / 100), qMid, [0, 0]).M;
      gainNote.textContent = `Kp = diag(${fmt(M[0][0] * g.kp, 0)}, ${fmt(M[1][1] * g.kp, 1)}) N·m/rad，Kd = diag(${fmt(M[0][0] * g.kd, 1)}, ${fmt(M[1][1] * g.kd, 1)}) N·m·s/rad`;
    } else {
      gainNote.textContent = `Kp = ${fmt(g.kp, 1)} s⁻²，Kd = ${fmt(g.kd, 1)} s⁻¹${g.ki ? `，Ki = ${fmt(g.ki, 0)} s⁻³` : ''}（再乘上 M̃(θ)）${st.kind === 'task' ? '，作用在末端位置' : ''}`;
    }
  }

  function updatePlots(): void {
    const c1 = cssVar('--series-1', '#2563eb'), c4 = cssVar('--series-4', '#d97706'), c8 = cssVar('--series-8', '#4b5563');
    const t = trial.t;
    if (st.kind === 'task') {
      errPlot.setOptions({ title: '末端追蹤誤差 x_d − x', yLabel: 'mm', series: [{ name: 'e_x', color: cssVar('--axis-x', '#e11') }, { name: 'e_y', color: cssVar('--axis-y', '#1a1') }] });
      errPlot.setXY(0, t, trial.err.map((e) => 1000 * e[0]));
      errPlot.setXY(1, t, trial.err.map((e) => 1000 * e[1]));
      auxPlot.setOptions({ title: '關節速度 θ̇（奇異附近暴增）', yLabel: 'rad/s', series: [{ name: 'θ̇₁', color: c1 }, { name: 'θ̇₂', color: c4 }] });
      auxPlot.setXY(0, t, trial.dth.map((d) => d[0]));
      auxPlot.setXY(1, t, trial.dth.map((d) => d[1]));
      let iMin = 0;
      trial.detJ.forEach((d, i) => { if (Math.abs(d) < Math.abs(trial.detJ[iMin])) iMin = i; });
      group.setMarkers(0, []);
      group.setMarkers(1, [{ x: t[iMin], color: cssVar('--danger', '#b91c1c'), label: `|det J| 最小 ${fmt(Math.abs(trial.detJ[iMin]), 2)}` }]);
      return;
    }
    const eSeries: SeriesSpec[] = [{ name: 'e₁', color: c1, width: 2 }, { name: 'e₂', color: c4, width: 2 }];
    if (st.offset) eSeries.push({ name: '理想二階 e₁', color: c8, dash: [5, 4], width: 1.5 });
    errPlot.setOptions({ title: '關節追蹤誤差 θ_d − θ', yLabel: 'θ_e (°)', series: eSeries });
    errPlot.setXY(0, t, trial.err.map((e) => deg(e[0])));
    errPlot.setXY(1, t, trial.err.map((e) => deg(e[1])));
    if (st.offset) errPlot.setXY(2, t, t.map((x) => deg(secondOrderFree(-OFFSET, st.wn, st.zeta, x))));
    auxPlot.setOptions({
      title: '力矩：實線＝模型前饋，虛線＝回饋', yLabel: 'τ (N·m)',
      series: [
        { name: 'τ₁ 前饋', color: c1, width: 2 }, { name: 'τ₁ 回饋', color: c1, dash: [5, 3], width: 1.6 },
        { name: 'τ₂ 前饋', color: c4, width: 2 }, { name: 'τ₂ 回饋', color: c4, dash: [5, 3], width: 1.6 },
      ],
    });
    auxPlot.setXY(0, t, trial.ff.map((v) => v[0]));
    auxPlot.setXY(1, t, trial.fb.map((v) => v[0]));
    auxPlot.setXY(2, t, trial.ff.map((v) => v[1]));
    auxPlot.setXY(3, t, trial.fb.map((v) => v[1]));
    // mark the two moves
    const H = HOLD;
    const moves: Marker[] = [{ x: H, color: c8, dash: [2, 4] }, { x: H + st.Tf, color: c8, dash: [2, 4] }, { x: 2 * H + st.Tf, color: c8, dash: [2, 4] }, { x: 2 * H + 2 * st.Tf, color: c8, dash: [2, 4] }];
    group.setMarkers(0, moves);
    group.setMarkers(1, []);
  }

  // ---- drawing --------------------------------------------------------------
  cv.draw = (c) => {
    const muted = cssVar('--fg-muted'), accent = cssVar('--accent'), warn = cssVar('--warn'), danger = cssVar('--danger');
    const i = idxAt(tCur);
    const th = trial.th[i];
    c.grid(0.5);
    c.circle([0, 0], 2, { color: cssVar('--grid-strong'), width: 1, dash: [4, 4] });
    c.frame({ x: 0, y: 0, theta: 0 }, 0.3, { labels: 's', width: 2 });
    // reference path of the tip and the end points
    c.polyline(tipdPts, { color: accent, width: 1.6, dash: [5, 4], alpha: 0.7 });
    const route = ROUTES[st.route];
    const names = st.route === 'ab' ? ['A', 'B'] : ['C', 'D'];
    [route.pa, route.pb].forEach((p, k) => {
      c.point([p[0], p[1]], 3.5, { color: accent });
      c.text([p[0], p[1]], names[k], { color: accent, dx: k ? -12 : 12, dy: k ? -10 : 12 });
    });
    // actual tip trail up to now
    if (i > 1) c.polyline(tipPts.slice(0, i + 1), { color: warn, width: 2 });
    // reference arm (joint-space controllers) or reference point (task space)
    const pd = trial.tipd[i];
    if (trial.thd) drawPlanarArm(c, LINKS, trial.thd[i], { alpha: 0.22, linkWidth: 7 });
    c.circle([pd[0], pd[1]], 0.06, { color: accent, width: 2 });
    drawPlanarArm(c, LINKS, th, { linkWidth: 8 });
    // singularity warning around the base
    const dJ = Math.abs(trial.detJ[i]);
    if (dJ < 0.35) {
      c.circle([0, 0], 0.3, { color: danger, width: 2, dash: [4, 3] });
      c.text([0, -0.42], `接近奇異 |det J| = ${fmt(dJ, 2)}`, { color: danger, bg: cssVar('--sim-bg') });
    }
    const v = c.visible();
    c.arrow([v.xmax - 0.2, v.ymax - 0.15], [v.xmax - 0.2, v.ymax - 0.55], { color: muted, width: 2, head: 8 });
    c.text([v.xmax - 0.2, v.ymax - 0.65], 'g', { color: muted });
    const e = trial.err[i];
    const errTxt = st.kind === 'task' ? `|x_e| = ${fmt(1000 * Math.hypot(e[0], e[1]), 1)} mm` : `θ_e = (${fmt(deg(e[0]), 1)}°, ${fmt(deg(e[1]), 1)}°)`;
    c.overlay.textContent = `t = ${fmt(trial.t[i], 2)} s　${KIND_SHORT[st.kind]}\n${errTxt}${trial.diverged ? '\n⚠ 發散，模擬中止' : ''}`;
  };

  // ---- animation --------------------------------------------------------------
  function frame(): void {
    const t = Math.min(tCur, trial.T);
    const i = idxAt(t);
    bars.set([trial.ff[i][0], trial.fb[i][0], trial.ff[i][1], trial.fb[i][1]]);
    cv.requestRender();
    group.setCursor(t);
    pb.setTime(t);
  }
  const loop = createLoop((dt) => {
    tCur += dt;
    if (tCur > trial.T + 0.8) tCur = 0;
    frame();
  });

  // ---- controls sync / presets -------------------------------------------------
  function syncControls(): void {
    kindSel.set(st.kind, { silent: true });
    routeSel.set(st.route, { silent: true });
    tKi.set(st.useKi, { silent: true });
    tOff.set(st.offset, { silent: true });
    sWn.set(st.wn, { silent: true });
    sZeta.set(st.zeta, { silent: true });
    sTf.set(st.Tf, { silent: true });
    sMass.set(st.massErrPct, { silent: true });
    sLam.set(st.lambda, { silent: true });
    const task = st.kind === 'task';
    sLam.disabled = !task;
    sLam.el.style.opacity = task ? '' : '0.45';
    sLam.el.title = task ? '' : '只有任務空間控制會用到 J 的反解';
  }
  function applyPreset(i: number): void {
    Object.assign(st, BASE, PRESETS[i].s);
    syncControls();
    tCur = 0;
    recompute();
  }

  const focusSets: HTMLElement[][] = [
    [errPlot.el, tableWrap],
    [errPlot.el, sTf.el],
    [errPlot.el, tOff.el],
    [auxPlot.el, bars.el],
    [sMass.el, tKi.el, errPlot.el],
    [auxPlot.el, sLam.el, tableWrap],
  ];
  let focused: HTMLElement[] = [];
  function onStep(i: number): void {
    const k = Math.max(0, Math.min(PRESETS.length - 1, i));
    chips.activate(k);
    focused.forEach((f) => f.classList.remove('mr-focus'));
    focused = focusSets[k] ?? [];
    focused.forEach((f) => f.classList.add('mr-focus'));
  }

  const unsubTheme = ctx.onThemeChange(() => { if (trial) { updatePlots(); cv.requestRender(); } });
  syncControls();
  onStep(ctx.currentStep);
  loop.start();

  return {
    onStep,
    destroy() {
      destroyed = true;
      loop.stop();
      window.clearTimeout(tableTimer);
      unsubTheme();
      cv.destroy();
      errPlot.destroy();
      auxPlot.destroy();
    },
  };
}

const lesson: Lesson = { meta, steps, quiz, mount };
export default lesson;
