import type { Lesson, LessonContext, LessonHandle, LessonStep, QuizQuestion } from '@/types';
import {
  el, cssVar, fmt, controlPanel, row, slider, toggle, button, readout, matrixView, playback, simTitle, note,
  Canvas2D, Plot2D, createLoop, presetChips, algorithmStepper, drawPlanarArm, mathEl, type Pt, type Marker,
} from '@/ui';
import { wrapToPi } from '@/mr';
import { meta } from './meta';
import {
  LINKS, OMEGA_LIMIT, Twin, analyzeAccel, armParams, assembleForwardDynamics, errorSweep, tipPath, tipOf,
  type Method, type Sweep, type TorqueFn,
} from './integrators';

// ---------------------------------------------------------------------------
// Narrative
// ---------------------------------------------------------------------------

const steps: LessonStep[] = [
  {
    title: '反過來：從力矩算加速度',
    md: `前兩課的運動方程式 $\\tau = M(\\theta)\\ddot\\theta + c(\\theta,\\dot\\theta) + g(\\theta)$ 回答的是「想這樣動，要多少力矩」（逆動力學）。模擬器面對的是反方向：此刻的狀態 $(\\theta,\\dot\\theta)$ 與馬達輸出的 $\\tau$ 已知，手臂**下一瞬間會怎麼加速**？把方程式移項：

$$\\ddot\\theta = M(\\theta)^{-1}\\big(\\tau - c(\\theta,\\dot\\theta) - g(\\theta)\\big)$$

這一步永遠有解：任何非零的 $\\dot\\theta$ 都帶來正的動能 $\\tfrac12\\dot\\theta^{\\T}M\\dot\\theta$，所以 $M$ 對稱正定、必定可逆。實作上是解線性方程組，而不是真的去算反矩陣。`,
    simHint: '拉動 τ₁、τ₂，看橘色 θ̈ 弧形箭頭與「≈0.3 s 後」的淡影怎麼變；按「τ = c + g」讓加速度歸零。拖曳手肘或末端換個姿勢：M 的特徵值永遠是正的嗎？',
  },
  {
    title: '用逆動力學湊出正動力學',
    md: `如果手邊只有一個逆動力學函式 $\\mathrm{ID}(\\theta,\\dot\\theta,\\ddot\\theta,\\mathfrak g)$——例如上一課的遞迴 Newton–Euler——照樣能拼出正動力學：

1. $\\mathrm{ID}(\\theta,\\dot\\theta,0,\\mathfrak g)$：令加速度為零，輸出正好是 $c+g$；
2. $\\mathrm{ID}(\\theta,0,e_j,0)$：關掉速度與重力、只給第 $j$ 軸單位加速度，輸出就是 $M$ 的第 $j$ 欄；
3. 解 $M\\ddot\\theta=\\tau-(c+g)$。

$n$ 軸共需 $n+1$ 次呼叫加一次線性求解。§8.4 的閉式 $M,c,g$ 只是同一組方程的展開寫法，兩條路算出的 $\\ddot\\theta$ 應該一模一樣。`,
    simHint: '按「▸ 單步」逐次呼叫 ID：c + g 與 M 的兩個欄位會依序被填上，最後與閉式公式的 θ̈ 比較差多少。',
  },
  {
    title: 'Euler 積分',
    md: `有了 $\\ddot\\theta$，把狀態疊成 $x=(\\theta,\\dot\\theta)$，運動方程式就成了一階 ODE $\\dot x=f(x,\\tau)$，可以一步步往前推。最簡單的是**顯式 Euler**：只拿區間起點的斜率走完整步，

$$\\theta_{k+1}=\\theta_k+\\Delta t\\,\\dot\\theta_k,\\qquad \\dot\\theta_{k+1}=\\dot\\theta_k+\\Delta t\\,\\ddot\\theta_k .$$

每步的局部誤差約 $\\Delta t^2$，累積到固定時刻的全域誤差是 $O(\\Delta t)$。對擺動系統，沿切線走一步總會稍微偏向外圈，所以即使沒有任何力矩輸入，總能量 $E$ 也一路上升——這是數值假象，不是物理。`,
    simHint: '按 ▶ 播放：只看紅色 Euler 分身與 ΔE(t) 曲線。再把 Δt 改成 0.02 s（會自動重設）播一次，能量爬升快了多少？',
  },
  {
    title: 'RK4 積分',
    md: `**四階 Runge–Kutta（RK4）**在一步裡取四個斜率：起點 $k_1$、兩次中點 $k_2,k_3$、終點 $k_4$，再加權平均：

$$x_{k+1}=x_k+\\tfrac{\\Delta t}{6}\\,(k_1+2k_2+2k_3+k_4)$$

每步要算四次正動力學，換來 $O(\\Delta t^4)$ 的全域誤差。同樣的 $\\Delta t$ 下，RK4 的能量誤差小到圖上看不出來。要比得公平，可以讓 Euler 改用 $\\Delta t/4$（每秒計算量相同）——它仍然落後好幾個數量級。`,
    simHint: '兩個分身一起播放，比較兩個 ΔE 讀數（RK4 約在 10⁻⁸ J 等級）。勾選「Euler 用 Δt/4」，在同樣計算量下再比一次。',
  },
  {
    title: '步長的代價',
    md: `把同一段 1 秒的運動用不同 $\\Delta t$ 各跑一次，與極細步長的參考解比較終點誤差，畫在 log–log 座標上：Euler 落在斜率約 1 的直線，RK4 約為 4。也就是說，**$\\Delta t$ 減半，Euler 誤差約減半，RK4 約剩 1/16**。

但 $\\Delta t$ 不能無限放大：當它接近系統最快動態的時間尺度（例如擺動週期的一小部分），兩種顯式方法都會失準甚至發散，誤差圖右端就偏離了直線。實用的檢查法：把 $\\Delta t$ 減半重跑，答案沒有明顯改變才算可信。`,
    simHint: '讀出兩條折線的擬合斜率；在畫布上拖曳手臂換個初始姿勢（放開後自動重算），斜率是否仍約為 1 與 4？最右邊 Δt = 0.2 s 的點偏離直線了嗎？',
  },
  {
    title: '開迴路力矩與下一課',
    md: `最後，給兩個分身同一段開迴路力矩脈衝 $\\tau(t)$。它們收到的輸入完全相同，差別只在積分誤差，但幾秒之內末端位置就分道揚鑣——擺動中的 2R 手臂會把微小差異迅速放大。真實機器人還有模型誤差與外界干擾，所以**只靠事先算好的 $\\tau(t)$（前饋）沒辦法讓手臂乖乖走在軌跡上**，這正是第 11 章回授控制要處理的問題。

下一課先把模型補完整：馬達、減速機與摩擦，會在 $M(\\theta)$ 與 $\\tau$ 上加進哪些項？`,
    simHint: '按 ▶ 播放，約 1 秒後按「⚡ 施加 τ 脈衝」，盯著「末端距離」：脈衝前只差一兩公分，之後幾秒內就拉開到數十公分。',
  },
];

const quiz: QuizQuestion[] = [
  {
    question: '用同一個初始狀態模擬 1 秒，把步長從 $\\Delta t = 0.02$ s 減半成 0.01 s。終點誤差大約會怎麼變？',
    options: ['Euler 與 RK4 都約減半', 'Euler 約減半，RK4 約剩 1/16', 'Euler 約剩 1/4，RK4 約剩 1/8', '都不變：誤差只取決於模擬總時間'],
    answer: 1,
    explanation: 'Euler 是一階方法，全域誤差 $\\propto \\Delta t$；RK4 是四階，誤差 $\\propto \\Delta t^4$，減半後約為 $(1/2)^4 = 1/16$。這正是 log–log 誤差圖上斜率 1 與 4 的兩條直線。前提是 $\\Delta t$ 已落在漸近範圍內；步長太大時兩者都會偏離這個規律。',
  },
  {
    question: '一個無摩擦、$\\tau = 0$ 的 2R 手臂模擬中，總能量 $E$ 隨時間穩定上升。最合理的解讀是？',
    options: ['重力一直對手臂做正功', '科氏力與向心力把能量注入系統', '這是數值積分誤差：應縮小 $\\Delta t$ 或改用更高階的方法', '質量矩陣 $M(\\theta)$ 在某些姿勢失去正定性'],
    answer: 2,
    explanation: '機械能的變化率等於輸入功率 $\\dot\\theta^{\\T}\\tau$（沒有摩擦時）。$\\tau = 0$ 時真實系統的 $E$ 守恆：重力是保守力，已經算在位能裡；速度乘積項 $c$ 只在關節之間搬移動能，淨功為零；$M$ 永遠正定。所以 $E$ 上升只能來自積分法本身——這是顯式 Euler 的典型症狀。',
  },
  {
    question: '若把 2R 手臂第二根連桿末端的質量設為 0（而且沒有任何轉動慣量），正動力學會出什麼問題？',
    options: ['沒有問題，$M$ 仍然正定', '$M$ 的第二列與第二欄全為 0，$M$ 奇異：$\\ddot\\theta_2$ 無法由 $\\tau$ 唯一決定', '只有 $c(\\theta,\\dot\\theta)$ 變成 0，其餘不變', '重力項 $g(\\theta)$ 變成負值'],
    answer: 1,
    explanation: '$M_{12}$ 與 $M_{22}$ 都來自第二根連桿的質量與慣量；全部為零時 $M = \\begin{bmatrix} m_1L_1^2 & 0\\\\ 0 & 0\\end{bmatrix}$ 不可逆。物理上，關節 2 推動的是「沒有慣性的東西」：任何非零的 $\\tau_2$ 都會造成無限大的加速度，$\\tau_2 = 0$ 時加速度又不確定。真實手臂一定有質量與馬達轉子慣量（下一課的 $G^2J_m$），所以 $M$ 總是正定。',
  },
  {
    question: '每模擬秒能做的正動力學計算次數固定（例如 400 次）。下列哪一種設定通常最準？',
    options: ['Euler，$\\Delta t = 0.0025$ s', 'RK4，$\\Delta t = 0.01$ s', '兩者一樣準，因為計算量相同', '要看螢幕更新率而定'],
    answer: 1,
    explanation: 'RK4 每步評估 4 次，所以 RK4 @ 0.01 s 與 Euler @ 0.0025 s 的成本相同。Euler 把步長縮成 1/4 只換來約 4 倍的精度；RK4 的誤差 $\\propto \\Delta t^4$，在這個步長下通常小好幾個數量級（在模擬中勾選「Euler 用 Δt/4」即可驗證）。前提是動態平滑——遇到碰撞、庫侖摩擦這類不連續，高階方法的優勢會打折。',
  },
];

// ---------------------------------------------------------------------------
// Simulation
// ---------------------------------------------------------------------------

const DT_CHOICES = [0.001, 0.002, 0.005, 0.01, 0.02, 0.05, 0.1, 0.15, 0.2];
const DEFAULT_DT_INDEX = 3; // 0.01 s
const PULSE_TAU = [12, -6];
const PULSE_DUR = 0.3;
const ZERO2 = [0, 0];
const TRAIL_MAX = 240;
const SWEEP_PATH_DT = 0.05;

interface InitPreset { label: string; title: string; x0: number[] }
const PRESETS: InitPreset[] = [
  { label: '小擺動', title: '從接近下垂的姿勢放開', x0: [-Math.PI / 2 + 0.4, 0.3, 0, 0] },
  { label: '水平釋放', title: '兩根連桿水平伸直後放開（大幅擺動）', x0: [0, 0, 0, 0] },
  { label: '倒立附近', title: '從接近倒立的不穩定平衡點放開', x0: [Math.PI / 2 - 0.15, 0.2, 0, 0] },
];

const SUP: Record<string, string> = { '-': '⁻', '+': '', '0': '⁰', '1': '¹', '2': '²', '3': '³', '4': '⁴', '5': '⁵', '6': '⁶', '7': '⁷', '8': '⁸', '9': '⁹' };

/** Compact number for wide dynamic ranges: 0.123, 12.3, 4.8×10⁻⁸. */
function sci(x: number, digits = 1): string {
  if (!Number.isFinite(x)) return '—';
  if (x === 0) return '0';
  const a = Math.abs(x);
  if (a >= 0.01 && a < 1e4) return fmt(x, a < 1 ? 3 : 2);
  const [m, e] = x.toExponential(digits).split('e');
  return `${m}×10${e.replace(/./g, (ch) => SUP[ch] ?? ch)}`;
}
const signed = (x: number): string => (x > 0 ? '+' : '') + sci(x);
const vec2 = (v: number[], d = 2): string => `(${fmt(v[0], d)}, ${fmt(v[1], d)})`;

/** Arc with an arrowhead around `center`, starting at angle a0 and sweeping `sweep` radians. */
function arcArrow(c: Canvas2D, center: Pt, r: number, a0: number, sweep: number, color: string, width = 2.5): void {
  if (Math.abs(sweep) < 0.03) return;
  const a1 = a0 + sweep;
  c.arc(center, r, Math.min(a0, a1), Math.max(a0, a1), { color, width });
  const end: Pt = [center[0] + r * Math.cos(a1), center[1] + r * Math.sin(a1)];
  const s = Math.sign(sweep);
  const back = 12 / c.scale;
  const start: Pt = [end[0] + Math.sin(a1) * s * back, end[1] - Math.cos(a1) * s * back];
  c.arrow(start, end, { color, width, head: 10 });
}

const ACC_ARC_SCALE = 0.35; // rad of arc per rad/s² of joint acceleration

interface LegendItem { color: string; label: string; dash?: boolean; faint?: boolean }

/** Small legend drawn in the bottom-left corner of a canvas (keeps the sim pane short). */
function canvasLegend(c: Canvas2D, items: LegendItem[]): void {
  // On narrow (mobile) canvases the legend would cover the arm; the twins are labelled at their tips anyway.
  if (!items.length || c.width < 460) return;
  const g = c.ctx;
  g.save();
  g.font = `12px ${cssVar('--font') || 'sans-serif'}`;
  const lh = 17;
  const w = Math.max(...items.map((i) => g.measureText(i.label).width)) + 38;
  const top = c.height - 8 - items.length * lh;
  g.globalAlpha = 0.85;
  g.fillStyle = cssVar('--sim-bg');
  g.fillRect(6, top - 4, w, items.length * lh + 6);
  g.globalAlpha = 1;
  items.forEach((it, k) => {
    const y = top + k * lh + lh / 2;
    g.strokeStyle = it.color;
    g.globalAlpha = it.faint ? 0.5 : 1;
    g.lineWidth = it.faint ? 5 : 3;
    g.setLineDash(it.dash ? [5, 3] : []);
    g.beginPath(); g.moveTo(12, y); g.lineTo(32, y); g.stroke();
    g.setLineDash([]);
    g.globalAlpha = 1;
    g.fillStyle = cssVar('--fg');
    g.textAlign = 'left';
    g.textBaseline = 'middle';
    g.fillText(it.label, 38, y);
  });
  g.restore();
}

/** A matrix display whose entries may still be unknown ("?"), styled like matrixView. */
function miniMatrix(parent: HTMLElement, label: string): { el: HTMLElement; set(rows: (number | null)[][], hlCol?: number): void } {
  const grid = el('div', { class: 'mr-grid' });
  const wrap = el('div', { class: 'mr-matrix' }, el('span', { class: 'mr-mlabel' }, label), grid);
  parent.appendChild(wrap);
  return {
    el: wrap,
    set(rows, hlCol) {
      grid.style.gridTemplateColumns = `repeat(${rows[0].length}, auto)`;
      grid.replaceChildren(...rows.flatMap((r) => r.map((v, j) =>
        el('span', { class: `mr-cell${hlCol === j && v != null ? ' hl' : ''}` }, v == null ? '?' : fmt(v, 3)))));
    },
  };
}

function mount(host: HTMLElement, ctx: LessonContext): LessonHandle {
  simTitle(host, '2R 手臂正動力學：Euler vs RK4', 'planar2R');

  // ---------- state
  let mode = -1; // 0 = θ̈ analysis, 1 = ID assembly, 2 = twins, 3 = error sweep
  let gravityOn = true;
  let P = armParams(true);
  const x0: number[] = [-Math.PI / 2 + 0.6, 0.8, 0, 0]; // shared initial / analysis state (θ1, θ2, θ̇1, θ̇2)
  let tau: number[] = [0, 0];
  let dtIndex = DEFAULT_DT_INDEX;
  let fairCost = false;
  let showEuler = true;
  let showRK4 = true;
  let simT = 0;
  let pulseStart = Number.NEGATIVE_INFINITY;
  let asmStage = 0;
  let sweep: Sweep | null = null;
  let sweepPaths: { ref: Pt[]; euler: Pt[]; rk4: Pt[] } | null = null;
  const twins: Record<Method, Twin> = { euler: new Twin('euler', x0, P), rk4: new Twin('rk4', x0, P) };
  const trails: Record<Method, Pt[]> = { euler: [], rk4: [] };
  const tauFn: TorqueFn = (t) => (t >= pulseStart && t < pulseStart + PULSE_DUR ? PULSE_TAU : ZERO2);
  const pulseActive = () => simT >= pulseStart && simT < pulseStart + PULSE_DUR;

  // ---------- layout
  const modeChips = presetChips(host, [
    { label: '① 求加速度', title: '由 τ 解出 θ̈ = M⁻¹(τ − c − g)', apply: () => setMode(0) },
    { label: '② ID 組裝', title: '用 n + 1 次逆動力學呼叫拼出正動力學', apply: () => setMode(1) },
    { label: '③ 積分比較', title: 'Euler 與 RK4 兩個分身並行模擬', apply: () => setMode(2) },
    { label: '④ 誤差 vs Δt', title: '全域誤差與步長的 log–log 關係', apply: () => setMode(3) },
  ], { label: '面板' });
  const tGrav = toggle(modeChips.el, { label: '重力（−y）', value: true, onChange: (v) => setGravity(v) });
  tGrav.el.style.marginLeft = 'auto';
  const cv = new Canvas2D(host, { aspect: 1.7, world: { xmin: -2.25, xmax: 2.25, ymin: -2.15, ymax: 2.2 }, overlay: true });

  const panelA = el('div');
  const panelB = el('div');
  const panelC = el('div');
  host.append(panelA, panelB, panelC);

  // ---------- panel A: θ̈ = M⁻¹(τ − c − g) and the ID assembly
  const ctlA = controlPanel(panelA, { compact: true });
  const sTau1 = slider(ctlA, { label: 'τ₁', min: -30, max: 30, step: 0.01, value: 0, unit: 'N·m', onInput: (v) => { tau[0] = v; refreshAnalysis(); } });
  const sTau2 = slider(ctlA, { label: 'τ₂', min: -30, max: 30, step: 0.01, value: 0, unit: 'N·m', onInput: (v) => { tau[1] = v; refreshAnalysis(); } });
  const sW1 = slider(ctlA, { label: 'θ̇₁', min: -4, max: 4, step: 0.01, value: 0, unit: 'rad/s', onInput: (v) => { x0[2] = v; onInitChanged(); } });
  const sW2 = slider(ctlA, { label: 'θ̇₂', min: -4, max: 4, step: 0.01, value: 0, unit: 'rad/s', onInput: (v) => { x0[3] = v; onInitChanged(); } });
  const rowA = row(ctlA);
  button(rowA, {
    label: 'τ = c + g（抵銷）', small: true, title: '讓 τ 剛好抵銷速度項與重力：θ̈ = 0',
    onClick: () => { const a = analyzeAccel(P, [x0[0], x0[1]], [x0[2], x0[3]], tau); setTau([a.c[0] + a.g[0], a.c[1] + a.g[1]]); },
  });
  button(rowA, { label: 'τ = 0', small: true, onClick: () => setTau([0, 0]) });
  button(rowA, { label: 'θ̇ = 0', small: true, onClick: () => { x0[2] = 0; x0[3] = 0; sW1.set(0, { silent: true }); sW2.set(0, { silent: true }); onInitChanged(); } });

  const dispA0 = el('div');
  panelA.appendChild(dispA0);
  const matRow = el('div', { style: { display: 'flex', flexWrap: 'wrap', alignItems: 'center', gap: '0.1rem 0.2rem' } });
  dispA0.appendChild(matRow);
  const mvM = matrixView(matRow, { label: 'M(θ) =', digits: 2 });
  const mvC = matrixView(matRow, { label: 'c =', digits: 2 });
  const mvG = matrixView(matRow, { label: 'g =', digits: 2 });
  const mvAcc = matrixView(matRow, { label: 'θ̈ =', digits: 2 });
  const eigNote = note(dispA0, '');

  const dispA1 = el('div');
  panelA.appendChild(dispA1);
  const stepper = algorithmStepper(dispA1, {
    onStep: () => { asmStage = Math.min(4, asmStage + 1); renderAssembly(); return asmStage < 4; },
    onReset: () => { asmStage = 0; renderAssembly(); },
    intervalMs: 900,
    countLabel: '步驟',
  });
  const callList = el('div', { style: { display: 'grid', gap: '0.25rem', fontSize: '0.85rem', margin: '0.2rem 0 0.4rem' } });
  dispA1.appendChild(callList);
  const CALLS = [
    { tex: '\\mathrm{ID}(\\theta,\\dot\\theta,0,\\mathfrak g)', what: '→ c + g =' },
    { tex: '\\mathrm{ID}(\\theta,0,e_1,0)', what: '→ M 第 1 欄 =' },
    { tex: '\\mathrm{ID}(\\theta,0,e_2,0)', what: '→ M 第 2 欄 =' },
    { tex: 'M\\ddot\\theta=\\tau-(c+g)', what: '解線性方程 → θ̈ =' },
  ];
  const callRows = CALLS.map((cl, i) => {
    const val = el('span', { style: { fontFamily: 'var(--mono)', fontVariantNumeric: 'tabular-nums' } }, '');
    const line = el('div', { style: { display: 'flex', flexWrap: 'wrap', gap: '0.2rem 0.5rem', alignItems: 'baseline', padding: '0.1rem 0.3rem', borderRadius: '6px' } },
      el('span', { style: { color: 'var(--accent)', fontWeight: '700' } }, '①②③④'[i]),
      mathEl(cl.tex),
      el('span', { style: { color: 'var(--fg-muted)' } }, cl.what),
      val);
    callList.appendChild(line);
    return { line, val };
  });
  const asmMats = el('div', { style: { display: 'flex', flexWrap: 'wrap', alignItems: 'center', gap: '0.1rem 0.2rem' } });
  dispA1.appendChild(asmMats);
  const mmM = miniMatrix(asmMats, 'M =');
  const mmB = miniMatrix(asmMats, 'c + g =');
  const mmA = miniMatrix(asmMats, 'θ̈ =');
  const asmNote = note(dispA1, '');

  // ---------- panel B: Euler / RK4 twins
  const initChips = presetChips(panelB, PRESETS.map((p, i) => ({ label: p.label, title: p.title, apply: () => applyPreset(i) })), { label: '初始狀態' });
  const ctlB = controlPanel(panelB, { compact: true });
  const rowB = row(ctlB);
  const dtWrap = el('div', { style: { width: '210px', maxWidth: '100%', marginRight: '0.6rem' } });
  rowB.appendChild(dtWrap);
  const sDt = slider(dtWrap, {
    label: '步長 Δt', min: 0, max: DT_CHOICES.length - 1, step: 1, value: dtIndex,
    format: (v) => `${DT_CHOICES[Math.round(v)]} s`,
    onInput: (v) => { dtIndex = Math.round(v); resetSim(); },
  });
  const pb = playback(rowB, {
    onPlay: () => startSim(),
    onPause: () => loop.stop(),
    onReset: () => resetSim(),
    onSpeed: (s) => { loop.speed = s; },
  });
  const rowB2 = row(ctlB);
  const tEuler = toggle(rowB2, { label: '顯示 Euler', value: true, onChange: (v) => { showEuler = v; redrawSim(); } });
  const tRK4 = toggle(rowB2, { label: '顯示 RK4', value: true, onChange: (v) => { showRK4 = v; redrawSim(); } });
  const tFair = toggle(rowB2, { label: 'Euler 用 Δt/4（同計算量）', value: false, onChange: (v) => { fairCost = v; resetSim(); } });
  const pulseBtn = button(rowB2, { label: '⚡ 施加 τ 脈衝', small: true, title: `τ = (${PULSE_TAU.join(', ')}) N·m，持續 ${PULSE_DUR} s`, onClick: () => firePulse() });
  const ePlot = new Plot2D(panelB, {
    title: '能量漂移 ΔE(t) = E(t) − E(0)', xLabel: 't (s)', yLabel: 'ΔE (J)',
    series: [{ name: 'RK4' }, { name: 'Euler' }], xWindow: 12, height: 140, yMinSpan: 0.02,
  });
  const rowB3 = el('div', { style: { display: 'flex', flexWrap: 'wrap', gap: '0.2rem 1rem', marginTop: '0.2rem' } });
  panelB.appendChild(rowB3);
  const rEe = readout(rowB3, { label: 'ΔE Euler', unit: 'J' });
  const rEr = readout(rowB3, { label: 'ΔE RK4', unit: 'J' });
  const rDist = readout(rowB3, { label: '末端距離', unit: 'm', title: 'Euler 與 RK4 分身末端位置的距離' });
  const rCost = readout(rowB3, { label: '每秒評估 E/R', title: '每模擬秒的正動力學計算次數：Euler 每步 1 次，RK4 每步 4 次' });
  const warnEl = note(panelB, '');
  warnEl.style.color = 'var(--danger)';
  const preview = el('div', {
    style: { border: '1px solid var(--border)', borderLeft: '4px solid var(--accent)', borderRadius: '8px', padding: '0.45rem 0.7rem', margin: '0.4rem 0', background: 'var(--accent-soft)', fontSize: '0.85rem' },
  },
  el('strong', {}, '下一課預告：'),
  '馬達轉子慣量經減速比放大成 G²J_m 加到 M(θ) 的對角線，τ 裡再多一項摩擦 τ_f(θ̇)。');
  panelB.appendChild(preview);

  // ---------- panel C: error versus Δt
  const ctlC = controlPanel(panelC, { compact: true });
  const rowC = row(ctlC);
  button(rowC, { label: '↻ 重新計算', small: true, onClick: () => computeSweep() });
  const rSlopeE = readout(rowC, { label: 'Euler 斜率 ≈', title: '在最小的 4 個 Δt 上做最小平方擬合' });
  const rSlopeR = readout(rowC, { label: 'RK4 斜率 ≈' });
  const errPlot = new Plot2D(panelC, {
    title: '模擬 1 秒後的末端位置誤差（log–log）', xLabel: 'log₁₀ Δt　（Δt 以 s 計）', yLabel: 'log₁₀ 誤差 (m)',
    series: [{ name: 'RK4' }, { name: 'Euler' }, { name: '斜率 4 參考', dash: [5, 4], width: 1.2 }, { name: '斜率 1 參考', dash: [5, 4], width: 1.2 }],
    height: 230,
  });
  const sweepNote = note(panelC, '');

  // ---------- handles: drag the elbow / tip of the initial (analysis) pose
  const elbow0 = (): Pt => [Math.cos(x0[0]), Math.sin(x0[0])];
  const tip0 = (): Pt => { const e = elbow0(); return [e[0] + Math.cos(x0[0] + x0[1]), e[1] + Math.sin(x0[0] + x0[1])]; };
  const hElbow = cv.addHandle({
    get: elbow0,
    set: (p) => { x0[0] = Math.atan2(p[1], p[0]); onInitChanged(true); },
    onEnd: () => onDragEnd(),
    radius: 9,
  });
  const hTip = cv.addHandle({
    get: tip0,
    set: (p) => { const e = elbow0(); x0[1] = wrapToPi(Math.atan2(p[1] - e[1], p[0] - e[0]) - x0[0]); onInitChanged(true); },
    onEnd: () => onDragEnd(),
    radius: 9,
  });
  const updateHandles = () => {
    const on = mode !== 2 || !loop.running;
    hElbow.enabled = on;
    hTip.enabled = on;
  };

  // ---------- drawing
  const drawGravity = (c: Canvas2D) => {
    const v = c.visible();
    const x = v.xmax - 0.35;
    if (!gravityOn) { c.text([x - 0.1, v.ymax - 0.25], '無重力', { color: cssVar('--fg-muted') }); return; }
    c.arrow([x, v.ymax - 0.15], [x, v.ymax - 0.65], { color: cssVar('--fg-muted'), width: 2 });
    c.text([x + 0.15, v.ymax - 0.42], 'g', { color: cssVar('--fg-muted'), font: 'italic 14px serif' });
  };
  const drawMasses = (c: Canvas2D, pts: Pt[], color: string, alpha = 1) => {
    c.disk(pts[1], 0.075, { fill: color, alpha });
    c.disk(pts[2], 0.075, { fill: color, alpha });
  };

  function drawAnalysis(c: Canvas2D): void {
    const th = [x0[0], x0[1]];
    const w = [x0[2], x0[3]];
    const a = analyzeAccel(P, th, w, tau);
    const tp = 0.3;
    const ghost = [th[0] + w[0] * tp + 0.5 * a.ddtheta[0] * tp * tp, th[1] + w[1] * tp + 0.5 * a.ddtheta[1] * tp * tp];
    const muted = cssVar('--fg-muted');
    const gp = drawPlanarArm(c, LINKS, ghost, { alpha: 0.25, linkWidth: 7 });
    c.text(gp[2], '≈0.3 s 後', { color: muted, dx: 30, dy: 14 });
    const pts = drawPlanarArm(c, LINKS, th, { linkWidth: 9 });
    drawMasses(c, pts, cssVar('--fg'));
    const warn = cssVar('--warn');
    const clampSweep = (v: number) => Math.max(-2.8, Math.min(2.8, ACC_ARC_SCALE * v));
    const s1 = clampSweep(a.ddtheta[0]);
    const s2 = clampSweep(a.ddtheta[1]);
    arcArrow(c, pts[0], 0.45, th[0], s1, warn, 3);
    arcArrow(c, pts[1], 0.34, th[0] + th[1], s2, warn, 3);
    // label just beyond the arrow tip, i.e. on the side away from the link
    const labelAt = (p: Pt, r: number, a0: number, sw: number): Pt => {
      const ang = a0 + sw + (sw >= 0 ? 0.55 : -0.55);
      return [p[0] + r * Math.cos(ang), p[1] + r * Math.sin(ang)];
    };
    c.text(labelAt(pts[0], 0.62, th[0], s1), `θ̈₁ = ${fmt(a.ddtheta[0], 2)}`, { color: warn, bg: cssVar('--sim-bg') });
    c.text(labelAt(pts[1], 0.55, th[0] + th[1], s2), `θ̈₂ = ${fmt(a.ddtheta[1], 2)}`, { color: warn, bg: cssVar('--sim-bg') });
    c.overlay.textContent = `θ = (${fmt((th[0] * 180) / Math.PI, 0)}°, ${fmt((th[1] * 180) / Math.PI, 0)}°)  θ̇ = ${vec2(w)} rad/s`;
  }

  function drawTwins(c: Canvas2D): void {
    if (simT > 0) drawPlanarArm(c, LINKS, [x0[0], x0[1]], { alpha: 0.18, linkWidth: 6 });
    const order: Method[] = ['rk4', 'euler'];
    for (const m of order) {
      if ((m === 'euler' && !showEuler) || (m === 'rk4' && !showRK4)) continue;
      const tw = twins[m];
      const color = cssVar(m === 'euler' ? '--series-2' : '--series-1');
      const alpha = tw.diverged ? 0.35 : 1;
      c.polyline(trails[m], { color, width: 1.6, alpha: 0.45 * alpha });
      const pts = drawPlanarArm(c, LINKS, [tw.x[0], tw.x[1]], { linkColor: color, eeColor: color, alpha, linkWidth: m === 'rk4' ? 11 : 5 });
      drawMasses(c, pts, color, alpha);
      const label = m === 'euler' ? 'Euler' : 'RK4';
      c.text(pts[2], tw.diverged ? `${label} ✕ 發散` : label, { color, dy: m === 'euler' ? -16 : 16, bg: cssVar('--sim-bg') });
      if (pulseActive() && !tw.diverged) {
        arcArrow(c, pts[0], 0.5, tw.x[0], 1.2 * Math.sign(PULSE_TAU[0]), cssVar('--warn'), 3);
        arcArrow(c, pts[1], 0.36, tw.x[0] + tw.x[1], 1.2 * Math.sign(PULSE_TAU[1]), cssVar('--warn'), 3);
      }
    }
    if (pulseActive()) c.text([0, 0.75], 'τ 脈衝作用中', { color: cssVar('--warn'), bg: cssVar('--sim-bg') });
    const items: LegendItem[] = [];
    if (showEuler) items.push({ color: cssVar('--series-2'), label: 'Euler 分身' });
    if (showRK4) items.push({ color: cssVar('--series-1'), label: 'RK4 分身' });
    if (simT > 0) items.push({ color: cssVar('--fg-muted'), label: '淡影＝初始姿勢', faint: true });
    canvasLegend(c, items);
    const hE = DT_CHOICES[dtIndex] / (fairCost ? 4 : 1);
    c.overlay.textContent = `t = ${fmt(simT, 2)} s   Δt = ${DT_CHOICES[dtIndex]} s${fairCost ? `（Euler ${fmt(hE, 4)} s）` : ''}`;
  }

  function drawSweep(c: Canvas2D): void {
    drawPlanarArm(c, LINKS, [x0[0], x0[1]], { alpha: 0.9, linkWidth: 8 });
    if (sweepPaths) {
      const c1 = cssVar('--series-1');
      const c2 = cssVar('--series-2');
      c.polyline(sweepPaths.ref, { color: cssVar('--fg-muted'), width: 5, alpha: 0.35 });
      c.polyline(sweepPaths.euler, { color: c2, width: 2 });
      c.polyline(sweepPaths.rk4, { color: c1, width: 2, dash: [6, 4] });
      const endE = sweepPaths.euler[sweepPaths.euler.length - 1];
      const endR = sweepPaths.rk4[sweepPaths.rk4.length - 1];
      const endRef = sweepPaths.ref[sweepPaths.ref.length - 1];
      c.point(endRef, 6, { color: cssVar('--fg-muted') });
      c.point(endE, 5, { color: c2 });
      c.point(endR, 4, { color: c1 });
      c.text(endE, 'Euler 終點', { color: c2, dy: -14, bg: cssVar('--sim-bg') });
    }
    canvasLegend(c, [
      { color: cssVar('--fg-muted'), label: '參考解', faint: true },
      { color: cssVar('--series-2'), label: `Euler（Δt = ${SWEEP_PATH_DT} s）` },
      { color: cssVar('--series-1'), label: `RK4（Δt = ${SWEEP_PATH_DT} s）`, dash: true },
    ]);
    c.overlay.textContent = `Δt = ${SWEEP_PATH_DT} s 時 1 秒內的末端軌跡`;
  }

  cv.draw = (c) => {
    c.grid(0.5);
    drawGravity(c);
    if (mode <= 1) drawAnalysis(c);
    else if (mode === 2) drawTwins(c);
    else drawSweep(c);
  };

  // ---------- analysis / assembly
  function setTau(v: number[]): void {
    tau = v.slice();
    sTau1.set(Math.max(-30, Math.min(30, v[0])), { silent: true });
    sTau2.set(Math.max(-30, Math.min(30, v[1])), { silent: true });
    refreshAnalysis();
  }

  function refreshAnalysis(): void {
    if (mode === 0) {
      const a = analyzeAccel(P, [x0[0], x0[1]], [x0[2], x0[3]], tau);
      mvM.set(a.M, { highlightCols: ['hl', null] });
      mvC.set(a.c);
      mvG.set(a.g);
      mvAcc.set(a.ddtheta, { highlightRows: ['hl', 'hl'] });
      eigNote.textContent = `M 的特徵值 λ = (${fmt(a.eig[0], 3)}, ${fmt(a.eig[1], 3)}) 都 > 0 ⇒ 正定、可逆。τ − c − g = ${vec2(a.net)} N·m。`;
    }
    if (mode === 1) renderAssembly();
    cv.requestRender();
  }

  function renderAssembly(): void {
    const a = assembleForwardDynamics([x0[0], x0[1]], [x0[2], x0[3]], tau, gravityOn);
    callRows.forEach((r, i) => {
      const done = asmStage > i;
      const next = asmStage === i;
      r.line.style.opacity = done || next ? '1' : '0.45';
      r.line.style.background = next ? 'var(--accent-soft)' : '';
      r.val.textContent = done ? vec2(i < 3 ? a.calls[i].result : a.ddtheta, 3) : '';
    });
    mmB.set([[asmStage >= 1 ? a.bias[0] : null], [asmStage >= 1 ? a.bias[1] : null]]);
    mmM.set([
      [asmStage >= 2 ? a.M[0][0] : null, asmStage >= 3 ? a.M[0][1] : null],
      [asmStage >= 2 ? a.M[1][0] : null, asmStage >= 3 ? a.M[1][1] : null],
    ], asmStage === 2 ? 0 : asmStage === 3 ? 1 : undefined);
    mmA.set([[asmStage >= 4 ? a.ddtheta[0] : null], [asmStage >= 4 ? a.ddtheta[1] : null]]);
    if (asmStage >= 4) {
      const dC = Math.max(Math.abs(a.ddtheta[0] - a.ddthetaClosed[0]), Math.abs(a.ddtheta[1] - a.ddthetaClosed[1]));
      const dL = Math.max(Math.abs(a.ddtheta[0] - a.ddthetaLibrary[0]), Math.abs(a.ddtheta[1] - a.ddthetaLibrary[1]));
      asmNote.textContent = `與閉式 Lagrange 公式（§8.4）的 θ̈ 相差 ${sci(dC)} rad/s²，與 mr.ForwardDynamics 相差 ${sci(dL)}：只剩捨入誤差。後面的積分為了速度改用閉式公式（同一個模型）。`;
    } else {
      asmNote.textContent = '按「▸ 單步」依序執行；也可以先改 τ、θ̇ 或拖曳手臂，數值會即時重算。';
    }
  }

  // ---------- twins
  function hOf(m: Method): number { const h = DT_CHOICES[dtIndex]; return m === 'euler' && fairCost ? h / 4 : h; }

  function resetSim(): void {
    loop.stop();
    pb.setPlaying(false, { silent: true });
    simT = 0;
    loop.reset();
    pulseStart = Number.NEGATIVE_INFINITY;
    twins.euler.reset(x0, P);
    twins.rk4.reset(x0, P);
    trails.euler = [];
    trails.rk4 = [];
    ePlot.clear();
    ePlot.push(0, [0, 0]);
    warnEl.textContent = '';
    updateSimReadouts();
    pb.setTime(0);
    updateHandles();
    cv.requestRender();
  }

  function startSim(): void {
    const alive = (showEuler && !twins.euler.diverged) || (showRK4 && !twins.rk4.diverged);
    if (!alive) { pb.setPlaying(false, { silent: true }); return; }
    loop.start();
    updateHandles();
  }

  function firePulse(): void {
    pulseStart = simT;
    if (!loop.running) { pb.setPlaying(true, { silent: true }); startSim(); }
  }

  function updateSimReadouts(): void {
    const eE = twins.euler.energy(P) - twins.euler.E0;
    const eR = twins.rk4.energy(P) - twins.rk4.E0;
    rEe.set(signed(eE));
    rEr.set(signed(eR));
    const a = tipOf(twins.euler.x);
    const b = tipOf(twins.rk4.x);
    rDist.set(sci(Math.hypot(a[0] - b[0], a[1] - b[1])));
    rCost.set(`${fmt(1 / hOf('euler'), 0)} / ${fmt(4 / hOf('rk4'), 0)}`);
  }

  function redrawSim(): void { cv.requestRender(); }

  const loop = createLoop((dt) => {
    simT += dt;
    for (const m of ['euler', 'rk4'] as Method[]) {
      const tw = twins[m];
      const was = tw.diverged;
      tw.advanceTo(simT, hOf(m), P, tauFn);
      if (!was && tw.diverged) {
        warnEl.textContent = `⚠ ${m === 'euler' ? 'Euler' : 'RK4'} 分身數值發散（|θ̇| > ${OMEGA_LIMIT} rad/s），已凍結在最後一個合理狀態。請縮小 Δt 後按「⟲ 重設」。`;
      }
      if (!tw.diverged) {
        trails[m].push(tipOf(tw.x));
        if (trails[m].length > TRAIL_MAX) trails[m].shift();
      }
    }
    const eE = showEuler && !twins.euler.diverged ? twins.euler.energy(P) - twins.euler.E0 : Number.NaN;
    const eR = showRK4 && !twins.rk4.diverged ? twins.rk4.energy(P) - twins.rk4.E0 : Number.NaN;
    ePlot.push(simT, [eR, eE]);
    updateSimReadouts();
    pb.setTime(simT);
    cv.requestRender();
    const alive = (showEuler && !twins.euler.diverged) || (showRK4 && !twins.rk4.diverged);
    if (!alive) { loop.stop(); pb.setPlaying(false, { silent: true }); updateHandles(); }
  });

  // ---------- error sweep
  function computeSweep(): void {
    sweep = errorSweep(P, x0.slice());
    sweepPaths = {
      ref: tipPath('rk4', P, x0, 1, 0.002),
      euler: tipPath('euler', P, x0, 1, SWEEP_PATH_DT),
      rk4: tipPath('rk4', P, x0, 1, SWEEP_PATH_DT),
    };
    renderSweep();
    cv.requestRender();
  }

  function renderSweep(): void {
    const c1 = cssVar('--series-1');
    const c2 = cssVar('--series-2');
    errPlot.setOptions({
      series: [{ name: 'RK4', color: c1 }, { name: 'Euler', color: c2 },
        { name: '斜率 4 參考', color: c1, dash: [5, 4], width: 1.2 }, { name: '斜率 1 參考', color: c2, dash: [5, 4], width: 1.2 }],
    });
    if (!sweep) return;
    const lg = (v: number) => (Number.isFinite(v) && v > 0 ? Math.log10(v) : Number.NaN);
    const xs = sweep.rows.map((r) => Math.log10(r.dt));
    const yR = sweep.rows.map((r) => lg(r.rk4));
    const yE = sweep.rows.map((r) => lg(r.euler));
    errPlot.setXY(0, xs, yR);
    errPlot.setXY(1, xs, yE);
    const refLine = (ys: number[], slope: number): number[] => {
      // anchor through the smallest-Δt finite point
      let k = ys.length - 1;
      while (k >= 0 && !Number.isFinite(ys[k])) k--;
      if (k < 0) return xs.map(() => Number.NaN);
      return xs.map((x) => ys[k] + slope * (x - xs[k]));
    };
    errPlot.setXY(2, xs, refLine(yR, 4));
    errPlot.setXY(3, xs, refLine(yE, 1));
    const marks: Marker[] = [];
    sweep.rows.forEach((r, i) => {
      if (Number.isFinite(yR[i])) marks.push({ x: xs[i], y: yR[i], color: c1 });
      if (Number.isFinite(yE[i])) marks.push({ x: xs[i], y: yE[i], color: c2 });
    });
    marks.push({ x: Math.log10(0.01) });
    errPlot.setMarkers(marks);
    rSlopeE.set(fmt(sweep.slopeEuler, 2));
    rSlopeR.set(fmt(sweep.slopeRK4, 2));
    const rows = sweep.rows;
    const a = rows[rows.length - 2];
    const b = rows[rows.length - 1];
    const diverged = rows.filter((r) => !Number.isFinite(r.euler) || !Number.isFinite(r.rk4)).map((r) => `${r.dt}`);
    sweepNote.textContent = `Δt 由 ${a.dt} 減半到 ${b.dt} s：Euler 誤差 ×${fmt(b.euler / a.euler, 2)}（理論 0.5），RK4 誤差 ×${fmt(b.rk4 / a.rk4, 3)}（理論 0.0625）。`
      + (diverged.length ? ` Δt = ${diverged.join('、')} s 的模擬發散，圖上略去。` : '')
      + ' 參考解：RK4，Δt = 1/4096 s；垂直虛線是積分比較面板的預設 Δt = 0.01 s。';
  }

  // ---------- shared state changes
  function onInitChanged(fromDrag = false): void {
    sweep = null;
    if (mode === 2) resetSim();
    if (mode === 3 && !fromDrag) computeSweep();
    refreshAnalysis();
  }
  function onDragEnd(): void {
    if (mode === 3) computeSweep();
  }

  function setGravity(on: boolean): void {
    gravityOn = on;
    P = armParams(on);
    sweep = null;
    resetSim();
    if (mode === 3) computeSweep();
    refreshAnalysis();
  }

  function applyPreset(i: number): void {
    const p = PRESETS[i];
    for (let k = 0; k < 4; k++) x0[k] = p.x0[k];
    sW1.set(x0[2], { silent: true });
    sW2.set(x0[3], { silent: true });
    sweep = null;
    resetSim();
    refreshAnalysis();
  }

  function setMode(m: number): void {
    mode = m;
    panelA.style.display = m <= 1 ? '' : 'none';
    dispA0.style.display = m === 0 ? '' : 'none';
    dispA1.style.display = m === 1 ? '' : 'none';
    panelB.style.display = m === 2 ? '' : 'none';
    panelC.style.display = m === 3 ? '' : 'none';
    ctlA.style.display = m === 0 ? '' : 'none';
    if (m !== 2) { loop.stop(); pb.setPlaying(false, { silent: true }); }
    if (m === 3 && !sweep) computeSweep();
    if (m === 3) renderSweep();
    updateHandles();
    refreshAnalysis();
  }

  // ---------- steps
  const STEP_MODE = [0, 1, 2, 2, 3, 2];
  let focused: HTMLElement[] = [];
  const setFocus = (els: (HTMLElement | null)[]) => {
    focused.forEach((f) => f.classList.remove('mr-focus'));
    focused = els.filter((x): x is HTMLElement => !!x);
    focused.forEach((f) => f.classList.add('mr-focus'));
  };
  const setVisibility = (e: boolean, r: boolean) => {
    showEuler = e; showRK4 = r;
    tEuler.set(e, { silent: true });
    tRK4.set(r, { silent: true });
  };

  function onStep(i: number): void {
    const m = STEP_MODE[i] ?? 2;
    modeChips.activate(m);
    preview.style.display = i === 5 ? '' : 'none';
    if (i === 1) stepper.reset();
    if (i === 2 || i === 3 || i === 5) {
      setVisibility(true, i !== 2);
      fairCost = false;
      tFair.set(false, { silent: true });
      dtIndex = DEFAULT_DT_INDEX;
      sDt.set(dtIndex, { silent: true });
      if (!gravityOn) tGrav.set(true);
      initChips.activate(0);
    }
    const focusByStep: (HTMLElement | null)[][] = [
      [cv.el, ctlA],
      [dispA1],
      [ePlot.el],
      [ePlot.el, tFair.el],
      [errPlot.el],
      [pulseBtn, rDist.el],
    ];
    setFocus(focusByStep[i] ?? []);
  }

  // ---------- initial view
  resetSim();
  onStep(ctx.currentStep);

  return {
    onStep,
    destroy() {
      loop.stop();
      stepper.destroy();
      cv.destroy();
      ePlot.destroy();
      errPlot.destroy();
    },
  };
}

const lesson: Lesson = { meta, steps, quiz, mount };
export default lesson;
