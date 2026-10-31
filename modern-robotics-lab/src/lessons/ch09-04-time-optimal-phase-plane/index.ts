import type { Lesson, LessonContext, LessonHandle, LessonStep, QuizQuestion } from '@/types';
import {
  el, cssVar, fmt, controlPanel, row, slider, toggle, select, button, readout, simTitle, legend,
  presetChips, algorithmStepper, playback, mdElement, Canvas2D, Plot2D, RasterLayer, createLoop,
  drawPlanarArm, type Pt, type Marker, type RGBA,
} from '@/ui';
import { segCircleIntersects } from '@/mr';
import { meta } from './meta';
import {
  PATHS, buildTable, checkCurve, curveFromDrawing, fastestCubic, integrate, jointIntervals, refreshLimits,
  sdotLimit, solveTimeOptimal, speedCap, timing, torquesAt, trajectorySamples,
  type CurveCheck, type PathId, type PathTable, type PhaseCurve, type Solution, type Timing, type TrajectorySamples,
  type Vec2,
} from './phasePlane';

const steps: LessonStep[] = [
  {
    title: '沿著路徑的一維動力學',
    md: `前幾課都是先選路徑 $\\theta(s)$、再選時間縮放 $s(t)$。這一課把路徑固定，只問一件事：**沿著它最快能走多快？**

因為 $\\dot\\theta=\\theta'\\dot s$、$\\ddot\\theta=\\theta'\\ddot s+\\theta''\\dot s^{2}$（$'$ 代表對 $s$ 微分），代入 $M(\\theta)\\ddot\\theta+c(\\theta,\\dot\\theta)+g(\\theta)=\\tau$ 之後，每個關節只剩一條純量方程式：

$$m_i(s)\\,\\ddot s+c_i(s)\\,\\dot s^{2}+g_i(s)=\\tau_i .$$

$m=M\\theta'$ 是沿路徑的等效慣量；$c$ 收集了 $M\\theta''$ 與科氏／離心項；$g$ 是重力。三者只和「走到哪裡」有關，所以不論手臂有幾個關節，狀態都只剩 $(s,\\dot s)$ 兩個數——這個平面就是**相平面**。`,
    simHint: '拖曳相平面上的黑色游標：旁邊的手臂移到 θ(s)，下方 c(s) 圖的游標跟著走，注意轉角 s ≈ 0.45 的尖峰（也可切到 m、g）。按 ▶ 播放，看一條三次時間縮放在相平面上畫出的曲線。',
  },
  {
    title: '力矩限制變成加速度區間',
    md: `致動器有上限 $|\\tau_i|\\le\\tau_{i,\\max}$。在狀態 $(s,\\dot s)$ 把上式解成 $\\ddot s$，若 $m_i>0$：

$$\\frac{-\\tau_{i,\\max}-c_i\\dot s^{2}-g_i}{m_i}\\;\\le\\;\\ddot s\\;\\le\\;\\frac{\\tau_{i,\\max}-c_i\\dot s^{2}-g_i}{m_i}$$

（$m_i<0$ 時兩端對調）。每個關節各給出一段區間，而它們必須同時成立，所以可行的加速度是交集 $[L(s,\\dot s),\\,U(s,\\dot s)]$。

相平面上的運動方向是 $(\\dot s,\\ddot s)$，因此每一點都有一個可行方向的扇形：上緣是全力加速 $U$，下緣是全力減速 $L$。`,
    simHint: '拖曳游標，看下方區間圖：藍、橘兩段是兩個關節各自允許的 s̈，綠色是交集 [L, U]，相平面上的綠色扇形就是它。再拖動區間圖上的 s̈ 標記（圓點），一超出某個關節的區間，該關節的力矩就變紅。',
  },
  {
    title: '速度極限曲線',
    md: `$c_i\\dot s^{2}$ 會把每個關節的區間整段平移，而且 $\\dot s$ 越大移得越遠。兩個關節的區間通常往不同方向移，到了某個速度以上就不再重疊，$L>U$：不論怎麼出力，手臂都無法留在這條路徑上。

把每個 $s$ 的臨界速度連起來，就是**速度極限曲線**；曲線上方（紅色區）不可行，運動狀態必須一直待在它下面。力矩上限調低，各區間變窄，整條曲線往下壓。路徑急轉彎（$\\theta''$ 大）或經過慣量大的構型時，曲線會在那裡凹下去。`,
    simHint: '游標停在紅色區裡：區間圖的交集是空的（L > U）。調低 τ₁,max 或 τ₂,max，看紅線整條往下壓；再切換三條路徑比較，轉角路徑在 s ≈ 0.45 有一個深谷。',
  },
  {
    title: '最快就是 bang-bang',
    md: `走完路徑的時間是 $T=\\int_0^1 \\frac{ds}{\\dot s(s)}$，所以每個 $s$ 的 $\\dot s$ 都越大越好。

從起點 $(0,0)$ 以 $\\ddot s=U$ 往前積分，得到「出發後最快能多快」；從終點 $(1,0)$ 以 $\\ddot s=L$ 往回積分，得到「再快就停不下來」的界線。任何可行的運動都不能高於兩者中較低的那條，而這條較低的曲線本身就做得到：先全力加速，到兩線交點**切換**成全力減速。

這種「不是全力加速、就是全力減速」的解叫 **bang-bang**：每一刻至少有一個關節的力矩頂在上限。`,
    simHint: '已切到「關節直線」：綠線從起點全力加速、橘線從終點全力減速，藍色粗線取兩者較低者。按 ▶ 播放：區間圖上的 s̈ 標記一直貼著綠色區間的一端，τ(t) 圖中總有一個關節頂到虛線。',
  },
  {
    title: '找切換點',
    md: `加速曲線若在碰到減速曲線之前，先撞上速度極限曲線，就不能硬穿過去。這時在撞上的位置 $s_{\\lim}$ 往下搜尋一個速度：從 $(s_{\\lim},\\dot s)$ 以全力減速往前積分，速度太高會穿過極限曲線、太低會降到 0；用二分法夾出剛好**擦過**極限曲線的那一條，擦過的點叫**切點**。

這條減速曲線往回積分，與加速曲線的交點是一次「加速→減速」切換；切點則是「減速→加速」切換。從切點再全力加速，重複以上步驟，直到與終點的減速曲線相交為止。`,
    simHint: '已切到「先肩後肘」：演算法停在加速曲線撞上極限曲線之處。按「▸ 單步」往下走：灰色虛線是二分法的試探、紫色是擦過極限曲線的那一條、圓點是切換點。',
  },
  {
    title: '手繪檢查',
    md: `自己畫一條 $\\dot s(s)$ 試試。可行的曲線必須同時滿足：

- 處處位於速度極限曲線之下；
- 每一小段的加速度 $\\ddot s=\\dot s\\,\\dfrac{d\\dot s}{ds}$ 都在 $[L,U]$ 內，也就是切線方向落在可行扇形裡。

違反的小段標成紅色。第二條常被忽略：極限曲線下方也可能 $U<0$（例如肘關節承受很大的離心負載），這時連等速前進都辦不到。可行的曲線不會比藍色最佳解快，因為最佳解在每個 $s$ 都已是「追得上、也停得下」的最高速度。`,
    simHint: '已打開「✎ 手繪」並放了一條直覺的梯形速度曲線：起步太陡、轉角穿過極限曲線，轉角前甚至連等速都撐不住，全是紅段。在相平面上由左往右拖曳重畫，比較「手繪 T」與 T*；按 ▶ 播放你的曲線，看 τ(t) 哪裡超出虛線。',
  },
  {
    title: '假設與下一章',
    md: `這套方法有幾個前提：路徑事先給定、模型準確、唯一的限制是只與狀態有關的力矩上下限。

若再加上關節速度上限 $|\\dot\\theta_i|=|\\theta_i'(s)|\\,\\dot s\\le\\omega_{\\max}$，相平面上會多一條上限 $\\dot s\\le\\omega_{\\max}/\\max_i|\\theta_i'(s)|$。最佳解可能有一段沿著它前進，那一段 $\\ddot s$ 落在 $[L,U]$ 內部，不再是 bang-bang。摩擦、馬達的轉速—力矩特性也會改變 $[L,U]$ 的形狀，但「在相平面上找最高的可行曲線」這個想法不變。

本章的路徑都是人給的；下一章的**運動規劃**處理更前面的問題：在障礙物之間找出一條不會碰撞的路徑。`,
    simHint: '已打開「關節速度上限」：橘色虛線是速度上限，藍色最佳解有一大段貼著它走，區間圖上的 s̈ 標記也離開了區間端點。調整 ω 上限觀察 T*；下方預告卡是同一條路徑在組態空間裡穿過障礙物的樣子。',
  },
];

const quiz: QuizQuestion[] = [
  {
    question: '同一台手臂、同樣的力矩上限，但路徑中段有一個急轉彎（$\\theta\'\'$ 很大）。速度極限曲線在那附近會怎樣？最佳解呢？',
    options: [
      '不受影響：極限曲線只由力矩上限決定，與路徑形狀無關',
      '在轉彎附近凹下去；最佳解得提早減速，在切點擦過谷底後再全力加速',
      '變高，因為轉彎時重力會幫忙',
      '整條消失，路徑一定不可行',
    ],
    answer: 1,
    explanation: '急轉彎讓 $c(s)$ 中的 $M\\theta\'\'$ 很大，$c_i\\dot s^{2}$ 把各關節的 $\\ddot s$ 區間推得很遠，於是在較低的速度下交集就變空，極限曲線凹陷。最佳解不能穿越不可行區，只能先減速、以切點擦過谷底再加速——「先肩後肘」路徑正是這樣，多出兩個切換點。路徑形狀透過 $m(s)$、$c(s)$ 影響極限曲線，所以 A 錯；凹陷不代表不可行，所以 D 錯。',
  },
  {
    question: '只有力矩限制時，時間最佳的 $\\dot s(s)$ 為什麼（除了碰到極限曲線的地方以外）不會出現一段水平線？',
    options: [
      '水平段表示 $\\ddot s=0$，而 0 通常落在 $[L,U]$ 內部：此時沒有任何關節用盡力矩，先多加速一點就能更早到達',
      '水平段會讓總時間變成無限大',
      '$\\dot s$ 固定時所有關節的力矩都必須是零',
      '其實水平段最好，因為速度最平均',
    ],
    answer: 0,
    explanation: '水平段上 $\\ddot s=\\dot s\\,d\\dot s/ds=0$。只要 $L<0<U$，這一段沒有任何關節出到上限，把曲線往上抬（多加速）就能讓 $\\int ds/\\dot s$ 變小，所以它不是最快的。時間不會變無限大（B 錯）；維持等速仍需要抵抗重力與速度項，力矩一般不為零（C 錯）。加上速度上限時，最佳解才可能沿著平緩的上限走（下一題）。',
  },
  {
    question: '加入關節速度上限 $|\\dot\\theta_i|\\le\\omega_{\\max}$ 之後，相平面上的可行區與最佳解會怎樣？',
    options: [
      '可行區不變，因為速度上限只會影響力矩',
      '可行區變大，因為速度小的時候需要的力矩比較小',
      '多一條上限 $\\dot s\\le\\omega_{\\max}/\\max_i|\\theta_i\'|$，可行區只會變小；最佳解可能有一段貼著它走，此段 $\\ddot s$ 在 $[L,U]$ 內部',
      '最佳解仍是純粹的 bang-bang，只是切換點位置不同',
    ],
    answer: 2,
    explanation: '$\\dot\\theta_i=\\theta_i\'\\dot s$，所以速度上限直接變成 $\\dot s$ 的上限，與原本的速度極限曲線取較低者，可行區只會縮小，$T^{*}$ 只會變長或不變。沿著這條上限走時，加速度由它的形狀決定，不再等於 $U$ 或 $L$，出現非 bang-bang 的一段。在模擬中打開「關節速度上限」就能看到。',
  },
  {
    question: '從起點全力加速的曲線，在 $s_{\\lim}$ 撞上了速度極限曲線。演算法下一步應該怎麼做？',
    options: [
      '沿著速度極限曲線一直走到終點',
      '讓曲線穿過極限曲線，之後再減速回來',
      '在 $s_{\\lim}$ 以二分法找一條剛好擦過極限曲線的全力減速曲線；它往回與加速曲線的交點是新的切換點，再從切點全力加速',
      '宣告這條路徑不可行',
    ],
    answer: 2,
    explanation: '極限曲線上方不可行，不能穿越（B 錯）。在極限曲線上 $L=U$，只剩一個加速度方向，它一般與曲線本身的斜率不同，所以通常無法沿著它走（A 錯）。擦過極限曲線的減速曲線保證之後仍有可行的路，並把加速曲線截在交點，形成「加速→減速→切點→加速」的結構。撞上極限曲線只代表要多切換幾次，並不表示不可行（D 錯）。',
  },
];

// ------------------------------------------------------------------------------------------------

type Tab = 'm' | 'c' | 'g' | 'tau';
type PlayCurve = 'opt' | 'cubic' | 'drawn';

const DEFAULT_TAU: Vec2 = [45, 18];
const DEFAULT_CAP = 2;
const PATH_ORDER: PathId[] = ['line', 'corner', 'cartesian'];
const PATH_LABEL: Record<PathId, string> = { line: '關節直線', corner: '先肩後肘（轉角）', cartesian: '末端走直線' };
const PATH_TITLE: Record<PathId, string> = {
  line: 'θ(s) 在關節空間是直線：兩個關節同時起步、同時停下',
  corner: '先轉肩、再轉肘，在 s ≈ 0.45 急轉彎（θ″ 很大）',
  cartesian: '末端沿 x = 1.3 m 的鉛直線往上走（逐點解 IK）',
};
/** Top of the ṡ axis per path (fixed, so moving the torque sliders visibly moves the limit curve). */
const SD_TOP: Record<PathId, number> = { line: 4.5, corner: 4, cartesian: 7 };
const COEF_TITLE: Record<'m' | 'c' | 'g', string> = {
  m: 'm₁(s)、m₂(s)：沿路徑的等效慣量（kg·m²）',
  c: 'c₁(s)、c₂(s)：乘在 ṡ² 前的係數（kg·m²）',
  g: 'g₁(s)、g₂(s)：重力項（N·m）',
};
const ARM_WORLD = { xmin: -0.8, xmax: 2.25, ymin: -2.2, ymax: 1.9 };
const OBSTACLE: { c: Pt; r: number } = { c: [1.4, 0.3], r: 0.3 };
const PP_H = 290;

const clamp = (v: number, lo: number, hi: number) => Math.max(lo, Math.min(hi, v));
const font = (px = 11) => `${px}px ${cssVar('--font') || 'sans-serif'}`;
const niceUp = (v: number, list: number[]) => list.find((x) => x >= v) ?? list[list.length - 1];
/**
 * s̈ inside [L, U] up to the discretisation of the curves: a node-sampled curve has a constant s̈
 * per cell, so at a point inside a saturated cell it can differ from U or L by a fraction of a percent.
 */
const insideInterval = (a: number, L: number, U: number) => {
  const tol = 0.005 * Math.max(1, Math.abs(L), Math.abs(U));
  return a >= L - tol && a <= U + tol;
};

/** '#rrggbb' / '#rgb' / 'rgb(r, g, b)' → RGBA (used to tint the C-space raster with a theme colour). */
function toRGBA(c: string, alpha: number): RGBA {
  const h = c.trim();
  let m = /^#([0-9a-f]{6})$/i.exec(h);
  if (m) { const n = parseInt(m[1], 16); return [(n >> 16) & 255, (n >> 8) & 255, n & 255, alpha]; }
  m = /^#([0-9a-f]{3})$/i.exec(h);
  if (m) { const [r, g, b] = m[1].split('').map((d) => parseInt(d + d, 16)); return [r, g, b, alpha]; }
  const r = /rgba?\(\s*([\d.]+)[ ,]+([\d.]+)[ ,]+([\d.]+)/i.exec(h);
  if (r) return [Number(r[1]), Number(r[2]), Number(r[3]), alpha];
  return [220, 38, 38, alpha];
}

/** Elbow and tip of the lab's 2R arm (L1 = L2 = 1 m). */
function armPoints(th: Vec2): { e: Pt; tip: Pt } {
  const e: Pt = [Math.cos(th[0]), Math.sin(th[0])];
  return { e, tip: [e[0] + Math.cos(th[0] + th[1]), e[1] + Math.sin(th[0] + th[1])] };
}

function collides(th: Vec2): boolean {
  const { e, tip } = armPoints(th);
  return segCircleIntersects([0, 0], e, OBSTACLE.c, OBSTACLE.r) || segCircleIntersects(e, tip, OBSTACLE.c, OBSTACLE.r);
}

function mount(host: HTMLElement, ctx: LessonContext): LessonHandle {
  const tables = new Map<PathId, PathTable>();
  const st = {
    path: 'corner' as PathId,
    tau: [DEFAULT_TAU[0], DEFAULT_TAU[1]] as Vec2,
    capOn: false,
    cap: DEFAULT_CAP,
    probe: { s: 0.3, sd: 0.6 },
    /** The learner's s̈ marker on the interval diagram. */
    sdd: 0,
    /** True while the probe follows the playback curve (playing or scrubbing). */
    following: false,
    t: 0,
    play: 'opt' as PlayCurve,
    tab: 'm' as Tab,
    zoomTop: null as number | null,
    show: { field: false, limit: true, full: false, algo: false, opt: true, cubic: true, strip: true, drawn: false },
    drawMode: false,
    stage: -1,
    /** Current lesson step (some readouts only make sense once their idea has been introduced). */
    step: 0,
    /** The first ▶ after entering a step starts from t = 0 even if the first view shows a later moment. */
    fresh: true,
  };
  let tab!: PathTable;
  let sol!: Solution;
  let cubic: { T: number; x: Float64Array } | null = null;
  let fullFwd: PhaseCurve | null = null;
  let fullBwd: PhaseCurve | null = null;
  let drawn: [number, number][] = [];
  let drawnX: Float64Array | null = null;
  let drawnChk: CurveCheck | null = null;
  let tm: Timing | null = null;
  let hist: TrajectorySamples | null = null;
  let playSdd = 0;
  let limSd = new Float64Array(0);
  let limTq = new Float64Array(0);
  let loSd = new Float64Array(0);
  let capSd = new Float64Array(0);
  let tipPath: Pt[] = [];
  let hitS = 0.5;
  let R = 10;

  // ---------------------------------------------------------------- layout
  simTitle(host, '相平面時間最佳化', '2R 手臂 · 力矩上限');
  const top = el('div', { style: { display: 'flex', flexWrap: 'wrap', gap: '0.5rem' } });
  host.appendChild(top);
  const ppHost = el('div', { style: { flex: '2 1 330px', minWidth: '0' } });
  const armHost = el('div', { style: { flex: '1 1 170px', minWidth: '0' } });
  top.append(ppHost, armHost);
  const cv = new Canvas2D(ppHost, { height: PP_H, world: { xmin: 0, xmax: 1, ymin: 0, ymax: 1 }, margin: 0, overlay: true });
  Object.assign(cv.overlay.style, { left: 'auto', right: '0.4rem', top: '0.3rem', padding: '0 0.3rem', borderRadius: '4px', background: 'color-mix(in srgb, var(--sim-bg) 82%, transparent)' });
  const armCv = new Canvas2D(armHost, { height: PP_H, world: ARM_WORLD, margin: 0.03 });
  legend(host, [
    { color: 'var(--ok)', label: '加速 U' }, { color: 'var(--series-4)', label: '減速 L' },
    { color: 'var(--danger)', label: '速度極限' }, { color: 'var(--accent)', label: '最佳解' },
    { color: 'var(--fg-muted)', label: '三次縮放' }, { color: 'var(--series-5)', label: '手繪' },
  ]);
  const iv = new Canvas2D(host, { height: 94, world: { xmin: 0, xmax: 1, ymin: 0, ymax: 1 }, margin: 0 });
  iv.el.style.marginTop = '0.2rem';

  const panel = controlPanel(host, { compact: true });
  const pathRow = row(panel);
  const pathChips = presetChips(pathRow, PATH_ORDER.map((id) => ({ label: PATH_LABEL[id], title: PATH_TITLE[id], apply: () => setPath(id) })), { label: '路徑' });
  const sT1 = slider(panel, { label: 'τ₁,max（肩）', min: 10, max: 80, step: 1, value: st.tau[0], unit: 'N·m', onInput: (v) => { st.tau[0] = v; recompute(); } });
  const sT2 = slider(panel, { label: 'τ₂,max（肘）', min: 4, max: 35, step: 0.5, value: st.tau[1], unit: 'N·m', onInput: (v) => { st.tau[1] = v; recompute(); } });
  const optRow = row(panel);
  const tgField = toggle(optRow, { label: '方向場', title: '在格點上畫出 U（綠）與 L（橘）兩個方向', onChange: (v) => { st.show.field = v; cv.requestRender(); } });
  const tgCubic = toggle(optRow, { label: '三次縮放', value: true, title: '可行範圍內最快的三次時間縮放 s = 3(t/T)² − 2(t/T)³', onChange: (v) => { st.show.cubic = v; cv.requestRender(); } });
  const tgDraw = toggle(optRow, { label: '✎ 手繪 ṡ(s)', title: '在相平面上由左往右拖曳畫出 ṡ(s)', onChange: (v) => setDrawMode(v) });
  const btnClear = button(optRow, { label: '清除手繪', small: true, onClick: () => clearDrawing() });
  const tgCap = toggle(optRow, { label: '關節速度上限', title: '|dθᵢ/dt| ≤ ω 上限', onChange: (v) => { st.capOn = v; syncCapUI(); recompute(); } });
  const sCap = slider(optRow, { label: 'ω 上限', min: 0.8, max: 6, step: 0.1, value: st.cap, unit: 'rad/s', onInput: (v) => { st.cap = v; recompute(); } });
  const algoRow = row(panel);
  const stepper = algorithmStepper(algoRow, {
    onStep: () => {
      const n = sol.stages.length;
      if (st.stage < n - 1) st.stage++;
      stepper.setStatus(sol.stages[st.stage]?.text ?? '');
      cv.requestRender();
      return st.stage < n - 1;
    },
    onReset: () => { st.stage = -1; stepper.setStatus('按「▸ 單步」從終點的減速曲線開始'); cv.requestRender(); },
    intervalMs: 900,
    countLabel: '階段',
  });
  const pbRow = row(panel);
  const pb = playback(pbRow, {
    duration: 1,
    onPlay: () => startPlay(),
    onPause: () => stopPlay(),
    onReset: () => { stopPlay(); st.t = 0; st.following = !!tm; applyPlayState(); },
    onSpeed: (s) => { loop.speed = s; },
    onScrub: (t) => { stopPlay(); st.fresh = false; st.t = t; st.following = !!tm; applyPlayState(); },
  });
  const playSel = select<PlayCurve>(pbRow, {
    options: [{ value: 'opt', label: '播放：時間最佳解' }, { value: 'cubic', label: '播放：三次縮放' }, { value: 'drawn', label: '播放：手繪曲線' }],
    value: st.play,
    onChange: (v) => { stopPlay(); st.play = v; st.t = 0; updatePlayCurve(); st.following = !!tm; applyPlayState(); renderAll(); },
  });

  const roRow = el('div', { style: { display: 'flex', flexWrap: 'wrap', gap: '0.2rem 1.1rem', alignItems: 'center', margin: '0.35rem 0 0.1rem' } });
  host.appendChild(roRow);
  const rLU = readout(roRow, { label: '游標處 [L, U] =', minWidth: '15ch', title: '游標狀態 (s, ṡ) 下可行的 s̈ 區間（1/s²）' });
  const rT = readout(roRow, { label: 'T* =', minWidth: '7ch', title: '時間最佳解走完路徑所需的時間' });
  const rSw = readout(roRow, { label: '切換點', minWidth: '2ch' });
  const rCub = readout(roRow, { label: '最快三次縮放 T =', minWidth: '7ch', title: '滿足力矩上限的三次時間縮放中最快的一條' });
  const rDraw = readout(roRow, { label: '手繪 T =', minWidth: '15ch' });
  const status = el('div', { class: 'mr-note', style: { margin: '0.1rem 0 0.3rem' } });
  host.appendChild(status);

  const plotArea = el('div');
  host.appendChild(plotArea);
  const tabRow = el('div');
  plotArea.appendChild(tabRow);
  const tabs = presetChips(tabRow, [
    { label: 'm(s)', apply: () => showTab('m') }, { label: 'c(s)', apply: () => showTab('c') },
    { label: 'g(s)', apply: () => showTab('g') }, { label: 'τ(t)', apply: () => showTab('tau') },
  ], { label: '圖表' });
  const pCoef = new Plot2D(plotArea, {
    height: 170, xLabel: 's', xRange: [0, 1], title: COEF_TITLE.m, legend: true, yPad: 0.22,
    series: [{ name: '關節 1', color: '--series-1' }, { name: '關節 2', color: '--series-4' }],
  });
  const pTau = new Plot2D(plotArea, {
    height: 170, xLabel: 't (s)', title: '關節力矩 τ(t)（N·m），虛線＝±上限', legend: true, yPad: 0.2,
    series: [{ name: 'τ₁', color: '--series-1' }, { name: 'τ₂', color: '--series-4' }],
  });

  const preview = el('div', { class: 'card', style: { padding: '0.45rem 0.6rem', margin: '0.3rem 0', display: 'none' } });
  host.appendChild(preview);
  preview.appendChild(mdElement('**下一章預告：運動規劃。** 左邊的工作空間多了一個圓形障礙物，手臂沿著這條給定的路徑走會撞上它。右邊是同一件事在組態空間 $(\\theta_1,\\theta_2)$ 裡的樣子：紅色是會碰撞的組態，路徑穿過紅區的那一段畫成紅色。第 10 章要在這張圖上**找出**一條避開紅區的路徑。'));
  const pvCv = new Canvas2D(preview, { aspect: 2.05, world: { xmin: -2.35, xmax: 7.55, ymin: -2.65, ymax: 2.55 } });

  // ---------------------------------------------------------------- phase-plane coordinates
  const sdTop = () => st.zoomTop ?? SD_TOP[st.path];
  const box = () => ({ l: 40, r: cv.width - 10, t: 10, b: cv.height - 30 });
  const scr = (s: number, sd: number): Pt => { const b = box(); return [b.l + s * (b.r - b.l), b.b - (sd / sdTop()) * (b.b - b.t)]; };
  const P = (s: number, sd: number): Pt => { const [x, y] = scr(s, sd); return cv.toWorld(x, y); };
  const toData = (w: Pt): [number, number] => { const [x, y] = cv.toScreen(w); const b = box(); return [(x - b.l) / (b.r - b.l), ((b.b - y) / (b.b - b.t)) * sdTop()]; };
  const shift = (c: Canvas2D, w: Pt, dx: number, dy: number): Pt => { const [x, y] = c.toScreen(w); return c.toWorld(x + dx, y + dy); };
  /** Screen direction (unit) of the phase-plane motion (ṡ, s̈) at speed sd with acceleration a. */
  const dirOf = (sd: number, a: number): Pt => {
    const b = box();
    const dx = sd * (b.r - b.l), dy = -a * ((b.b - b.t) / sdTop());
    const n = Math.hypot(dx, dy) || 1;
    return [dx / n, dy / n];
  };
  const curvePts = (pts: [number, number][], every = 1): Pt[] => {
    const out: Pt[] = [];
    for (let i = 0; i < pts.length; i += every) out.push(P(pts[i][0], Math.sqrt(Math.max(0, pts[i][1]))));
    const last = pts[pts.length - 1];
    if (last && (pts.length - 1) % every !== 0) out.push(P(last[0], Math.sqrt(Math.max(0, last[1]))));
    return out;
  };
  const capSdAt = (s: number) => {
    if (!st.capOn || !capSd.length) return Infinity;
    return capSd[clamp(Math.round(s * 2 * tab.N), 0, 2 * tab.N)];
  };
  const curSdd = () => (st.following ? playSdd : st.sdd);

  // ---------------------------------------------------------------- model updates
  function getTable(id: PathId): PathTable {
    let t = tables.get(id);
    if (!t) { t = buildTable(PATHS[id], { tauMax: st.tau, dthetaMax: st.capOn ? st.cap : null }); tables.set(id, t); }
    return t;
  }

  function recompute(): void {
    refreshLimits(tab, { tauMax: st.tau, dthetaMax: st.capOn ? st.cap : null });
    sol = solveTimeOptimal(tab);
    cubic = fastestCubic(tab);
    fullFwd = integrate(tab, 0, 0, 1, 'U');
    fullBwd = integrate(tab, tab.N, 0, -1, 'L');
    const J = 2 * tab.N;
    limSd = new Float64Array(J + 1); limTq = new Float64Array(J + 1); loSd = new Float64Array(J + 1); capSd = new Float64Array(st.capOn ? J + 1 : 0);
    for (let j = 0; j <= J; j++) {
      limSd[j] = sdotLimit(tab, j);
      limTq[j] = tab.xLo[j] <= tab.xHiTorque[j] ? Math.sqrt(Math.max(0, tab.xHiTorque[j])) : 0;
      loSd[j] = tab.xLo[j] > 0 ? Math.sqrt(tab.xLo[j]) : 0;
      if (st.capOn) capSd[j] = Math.sqrt(speedCap(tab, j));
    }
    if (drawn.length) { drawnX = curveFromDrawing(tab.N, drawn); drawnChk = checkCurve(tab, drawnX); }
    // Step 1 shows only the (slow) cubic scaling, so its ṡ axis is zoomed to fit that curve.
    st.zoomTop = null;
    if (st.step === 0 && cubic) {
      let m = 0;
      for (const v of cubic.x) m = Math.max(m, Math.sqrt(v));
      st.zoomTop = niceUp(2.2 * m, [0.5, 1, 1.5, 2, 3, 4, 5, 6, 8]);
      st.probe.sd = Math.min(st.probe.sd, st.zoomTop);
    }
    if (st.show.algo) restartAlgo(); else st.stage = -1;
    updatePlayCurve();
    if (st.following) applyPlayState();
    updateReadouts();
    renderAll();
  }

  function restartAlgo(): void {
    stepper.reset();
    // Start where it gets interesting: the forward curve has just hit the limit curve (or crossed F).
    stepper.step();
    stepper.step();
  }

  function curveFor(which: PlayCurve): Float64Array | null {
    if (which === 'opt') return sol.ok ? sol.x : null;
    if (which === 'cubic') return cubic ? cubic.x : null;
    return drawnX;
  }

  function updatePlayCurve(): void {
    const x = curveFor(st.play);
    tm = x ? timing(x, tab.ds) : null;
    if (tm && !Number.isFinite(tm.T)) tm = null;
    hist = x && tm ? trajectorySamples(tab, x) : null;
    pTau.setXY(0, hist ? hist.t : [], hist ? hist.tau[0] : []);
    pTau.setXY(1, hist ? hist.t : [], hist ? hist.tau[1] : []);
    const name = st.play === 'opt' ? '時間最佳解' : st.play === 'cubic' ? '三次縮放' : '手繪曲線';
    // Keep the ±limit lines readable when an infeasible curve asks for enormous torques.
    let peak = 0;
    if (hist) for (const arr of hist.tau) for (const v of arr) peak = Math.max(peak, Math.abs(v));
    const clip = 2.5 * Math.max(st.tau[0], st.tau[1]);
    pTau.setRanges(undefined, peak > clip ? [-clip, clip] : null);
    pTau.setOptions({ title: hist ? `${name}的關節力矩 τ(t)（N·m），虛線＝±上限${peak > clip ? '（超出部分截斷）' : ''}` : `${name}：無法播放（${st.play === 'drawn' && !drawnX ? '尚未手繪' : '不可行或中途停下'}）` });
    pb.setDuration(tm ? tm.T : 1);
    if (tm) st.t = Math.min(st.t, tm.T);
    else { st.t = 0; st.following = false; stopPlay(); }
    paintMarkers();
  }

  function updateCoefPlot(): void {
    const key = st.tab === 'tau' ? 'm' : st.tab;
    const arr = key === 'm' ? tab.m : key === 'c' ? tab.c : tab.g;
    const xs: number[] = [], y1: number[] = [], y2: number[] = [];
    for (let j = 0; j <= 2 * tab.N; j += 4) { xs.push(j / (2 * tab.N)); y1.push(arr[0][j]); y2.push(arr[1][j]); }
    pCoef.setXY(0, xs, y1);
    pCoef.setXY(1, xs, y2);
    pCoef.setOptions({ title: COEF_TITLE[key] });
  }

  function paintMarkers(): void {
    const cur: Marker[] = [{ x: st.probe.s, color: '--fg-muted', dash: [2, 3] }];
    pCoef.setMarkers(cur);
    const lim: Marker[] = [
      { y: st.tau[0], color: '--series-1', dash: [5, 4] }, { y: -st.tau[0], color: '--series-1', dash: [5, 4] },
      { y: st.tau[1], color: '--series-4', dash: [5, 4] }, { y: -st.tau[1], color: '--series-4', dash: [5, 4] },
    ];
    if (hist && st.following) lim.push({ x: st.t, color: '--fg-muted', dash: [2, 3] });
    pTau.setMarkers(lim);
  }

  function updateReadouts(): void {
    const late = st.step >= 3;
    rLU.el.style.display = st.step >= 1 ? '' : 'none';
    for (const r of [rT, rSw, rCub]) r.el.style.display = late ? '' : 'none';
    status.style.display = late || !sol.ok ? '' : 'none';
    rT.set(sol.ok ? `${fmt(sol.T, 3)} s` : '—（不可行）');
    rSw.set(sol.ok ? String(sol.switches.length) : '—');
    rCub.set(cubic ? `${fmt(cubic.T, 3)} s` : '—（任何 T 都不行）');
    rDraw.el.style.display = drawnChk && (st.show.drawn || st.drawMode) ? '' : 'none';
    btnClear.style.display = drawn.length && (st.show.drawn || st.drawMode) ? '' : 'none';
    if (drawnChk) {
      const pct = Math.round((100 * drawnChk.bad) / drawnChk.ok.length);
      rDraw.set(Number.isFinite(drawnChk.T)
        ? `${fmt(drawnChk.T, 3)} s${drawnChk.bad ? `（${pct}% 違規）` : '（可行）'}`
        : '∞（中途停下）');
      rDraw.el.style.color = drawnChk.bad || !Number.isFinite(drawnChk.T) ? 'var(--danger)' : '';
    }
    let msg: string;
    if (!sol.ok) msg = `⚠ ${sol.message ?? '在這組力矩上限下走不完這條路徑'}`;
    else if (sol.method === 'grid') msg = sol.message ?? '開啟速度上限後改用數值動態規劃求解：最佳解可能有一段貼著速度上限走，那一段不是 bang-bang。';
    else msg = `bang-bang：${sol.switches.length} 次切換；最快的三次縮放${cubic ? `要 ${fmt(cubic.T / sol.T, 2)} 倍的時間` : '在任何 T 都不可行'}。`;
    status.textContent = msg;
    status.style.color = sol.ok ? 'var(--fg-muted)' : 'var(--danger)';
  }

  function renderAll(): void {
    cv.requestRender();
    iv.requestRender();
    armCv.requestRender();
    paintMarkers();
    updateOverlay();
  }

  function updateOverlay(): void {
    const { s, sd } = st.probe;
    const head = st.following && tm ? `t = ${fmt(st.t, 2)} s  ` : '';
    cv.setOverlayText(`${head}s = ${fmt(s, 2)}  ṡ = ${fmt(sd, 2)}`);
    const J = jointIntervals(tab, s, sd * sd);
    rLU.set(J.L <= J.U ? `[${fmt(J.L, 2)}, ${fmt(J.U, 2)}]` : '空集合（L > U）');
  }

  // ---------------------------------------------------------------- path / mode changes
  function setPath(id: PathId): void {
    stopPlay();
    st.path = id;
    tab = getTable(id);
    pathChips.highlight(PATH_ORDER.indexOf(id));
    tipPath = [];
    let first = -1, last = -1;
    for (let k = 0; k <= 200; k++) {
      const th = PATHS[id].theta(k / 200);
      tipPath.push(armPoints(th).tip);
      if (collides(th)) { if (first < 0) first = k; last = k; }
    }
    hitS = first >= 0 ? (first + last) / 400 : 0.5;
    drawn = []; drawnX = null; drawnChk = null;
    if (st.play === 'drawn') { st.play = 'opt'; playSel.set('opt', { silent: true }); }
    st.t = 0; st.following = false;
    st.probe.sd = Math.min(st.probe.sd, sdTop());
    recompute();
    updateCoefPlot();
    pvCv.requestRender();
  }

  function setDrawMode(v: boolean): void {
    st.drawMode = v;
    tgDraw.set(v, { silent: true });
    probeHandle.enabled = !v;
    cv.canvas.style.cursor = v ? 'crosshair' : '';
  }

  function clearDrawing(): void {
    drawn = []; drawnX = null; drawnChk = null;
    if (st.play === 'drawn') { st.play = 'opt'; playSel.set('opt', { silent: true }); st.t = 0; }
    updatePlayCurve();
    updateReadouts();
    renderAll();
  }

  function finishDrawing(): void {
    if (drawn.length < 2) { clearDrawing(); return; }
    drawnX = curveFromDrawing(tab.N, drawn);
    drawnChk = checkCurve(tab, drawnX);
    st.play = 'drawn'; playSel.set('drawn', { silent: true });
    st.t = 0;
    updatePlayCurve();
    updateReadouts();
    renderAll();
  }

  /** A "natural" first guess: trapezoidal speed in s (ramp, plateau, ramp) at 85 % of the optimum's peak. */
  function exampleDrawing(): [number, number][] {
    let peak = 1;
    if (sol.ok) { peak = 0; for (const v of sol.x) peak = Math.max(peak, Math.sqrt(v)); }
    const h = 0.85 * peak;
    const out: [number, number][] = [];
    for (let k = 1; k < 50; k++) {
      const s = k / 50;
      out.push([s, s < 0.22 ? (h * s) / 0.22 : s > 0.8 ? (h * (1 - s)) / 0.2 : h]);
    }
    return out;
  }

  function syncCapUI(): void {
    tgCap.set(st.capOn, { silent: true });
    sCap.el.style.display = st.capOn ? '' : 'none';
  }

  function showTab(t: Tab): void {
    st.tab = t;
    pCoef.el.style.display = t === 'tau' ? 'none' : '';
    pTau.el.style.display = t === 'tau' ? '' : 'none';
    if (t !== 'tau') updateCoefPlot();
  }

  // ---------------------------------------------------------------- playback
  const loop = createLoop((dt) => {
    if (!tm) { stopPlay(); return; }
    st.t = Math.min(tm.T, st.t + dt);
    applyPlayState();
    if (st.t >= tm.T) stopPlay();
  });

  function startPlay(): void {
    if (!tm) {
      pb.setPlaying(false, { silent: true });
      status.textContent = st.play === 'drawn' && !drawnX ? '還沒有手繪曲線：打開「✎ 手繪」在相平面上畫一條。' : '這條曲線走不完路徑（不可行或中途停下），無法播放。';
      status.style.color = 'var(--danger)';
      status.style.display = '';
      return;
    }
    if (st.t >= tm.T - 1e-9 || st.fresh) st.t = 0;
    st.fresh = false;
    st.following = true;
    loop.start();
    pb.setPlaying(true, { silent: true });
  }

  function stopPlay(): void {
    loop.stop();
    pb.setPlaying(false, { silent: true });
  }

  function applyPlayState(): void {
    if (!tm) return;
    const q = tm.at(st.t);
    st.probe.s = q.s;
    st.probe.sd = q.sd;
    playSdd = q.sdd;
    pb.setTime(st.t);
    renderAll();
  }

  /** Time at which a node-sampled timing reaches path parameter s. */
  function timeAtS(s: number): number {
    if (!tm) return 0;
    const k = clamp(Math.round(s * tab.N), 0, tab.N);
    return Math.min(tm.T, tm.t[k]);
  }

  // ---------------------------------------------------------------- phase plane drawing
  cv.draw = (c) => {
    const b = box(), topSd = sdTop(), f = font();
    const sdStep = topSd > 5 ? 1 : topSd > 1.6 ? 0.5 : 0.2;
    for (let v = 0; v <= topSd + 1e-9; v += sdStep) {
      c.line(P(0, v), P(1, v), { color: '--grid', width: 1 });
      c.text(P(0, v), fmt(v, sdStep < 1 ? 1 : 0), { color: '--fg-muted', align: 'right', dx: -5, font: f });
    }
    for (let k = 0; k <= 10; k++) {
      c.line(P(k / 10, 0), P(k / 10, topSd), { color: '--grid', width: 1 });
      if (k % 2 === 0) c.text(P(k / 10, 0), fmt(k / 10, 1), { color: '--fg-muted', dy: 10, font: f });
    }
    c.text(P(0.5, 0), 's（路徑參數）', { color: '--fg-muted', dy: 23, font: f });
    c.text(P(0, topSd), 'ṡ（1/s）', { color: '--fg', align: 'left', dx: 6, dy: 9, font: font(12), bg: '--sim-bg' });

    const g = c.ctx;
    g.save();
    g.beginPath(); g.rect(b.l, b.t, b.r - b.l, b.b - b.t); g.clip();
    if (st.show.limit) drawLimit(c);
    if (st.show.field) drawField(c);
    if (st.show.cubic && cubic) drawCubic(c);
    if (st.show.full) drawFull(c);
    if (st.show.algo) drawStages(c);
    else if (st.show.opt && sol.ok) drawOptimal(c);
    if (drawn.length && (st.show.drawn || st.drawMode)) drawDrawn(c);
    if (st.show.strip) drawProbeCone(c);
    g.restore();
    c.line(P(0, 0), P(1, 0), { color: '--grid-strong', width: 1.4 });
    c.line(P(0, 0), P(0, topSd), { color: '--grid-strong', width: 1.4 });
    updateOverlay();
  };

  function drawLimit(c: Canvas2D): void {
    const J = 2 * tab.N, topSd = sdTop(), hi = topSd * 1.6;
    const cap = (v: number) => (Number.isFinite(v) ? Math.min(v, hi) : hi);
    const upper: Pt[] = [P(0, hi)];
    for (let j = 0; j <= J; j += 2) upper.push(P(j / J, cap(limSd[j])));
    upper.push(P(1, hi));
    const dark = document.documentElement.dataset.theme === 'dark';
    c.polygon(upper, { fill: '--danger', alpha: dark ? 0.2 : 0.12, width: 0 });
    // Low-speed band where the arm cannot even hold the path (gravity beats the torque limit):
    // below √x_lo the interval is empty too, so the arm must already be moving there.
    let s0 = -1, s1 = -1, band: Pt[] = [];
    const flush = () => {
      if (band.length > 1) c.polygon([...band, P(s1, 0), P(s0, 0)], { fill: '--danger', alpha: dark ? 0.3 : 0.22, width: 0 });
      band = []; s0 = -1;
    };
    for (let j = 0; j <= J; j += 2) {
      if (loSd[j] > 0) { if (s0 < 0) s0 = j / J; s1 = j / J; band.push(P(j / J, Math.min(loSd[j], hi))); } else flush();
    }
    flush();
    const curve: Pt[] = [];
    for (let j = 0; j <= J; j += 2) curve.push(P(j / J, cap(limTq[j])));
    c.polyline(curve, { color: '--danger', width: 2.2 });
    if (st.capOn && capSd.length) {
      const cc: Pt[] = [];
      for (let j = 0; j <= J; j += 2) cc.push(P(j / J, cap(capSd[j])));
      c.polyline(cc, { color: '--warn', width: 2.2, dash: [7, 4] });
      // label at the lowest point of the cap curve
      const ja = Math.round(0.15 * J), jb = Math.round(0.85 * J);
      let jm = ja;
      for (let j = ja; j <= jb; j += 2) if (capSd[j] < capSd[jm]) jm = j;
      const sm = jm / J;
      c.text(P(sm, Math.min(capSd[jm], 0.9 * topSd)), `關節速度上限 ω = ${fmt(st.cap, 1)} rad/s`,
        { color: '--warn', dy: -11, align: sm < 0.35 ? 'left' : sm > 0.65 ? 'right' : 'center', font: font(), bg: '--sim-bg' });
    }
    // Label the limit curve where it is inside the view, preferring the left part.
    for (const s of [0.12, 0.25, 0.7, 0.85]) {
      const v = limTq[Math.round(s * J)];
      if (v > 0.1 * topSd && v < 0.9 * topSd) {
        c.text(P(s, v), '速度極限曲線', { color: '--danger', dy: -11, font: font(), bg: '--sim-bg' });
        break;
      }
    }
  }

  function drawField(c: Canvas2D): void {
    const topSd = sdTop(), len = 12;
    for (let i = 0; i < 10; i++) {
      const s = 0.05 + 0.1 * i;
      for (let k = 0; k < 6; k++) {
        const sd = topSd * (0.08 + 0.155 * k);
        const o = P(s, sd);
        const J = jointIntervals(tab, s, sd * sd);
        if (J.L > J.U || sd > capSdAt(s)) { c.text(o, '×', { color: '--danger', font: font(10) }); continue; }
        for (const [a, col] of [[J.U, '--ok'], [J.L, '--series-4']] as const) {
          const d = dirOf(sd, a);
          c.arrow(o, shift(c, o, d[0] * len, d[1] * len), { color: col, width: 1.3, head: 5 });
        }
      }
    }
  }

  function drawCubic(c: Canvas2D): void {
    if (!cubic) return;
    const pts: Pt[] = [];
    for (let k = 0; k <= tab.N; k += 5) pts.push(P(k * tab.ds, Math.sqrt(cubic.x[k])));
    c.polyline(pts, { color: '--fg-muted', width: 1.7, dash: [6, 4] });
    if (!st.show.opt && !st.show.full) {
      const kp = Math.round(0.2 * tab.N);
      c.text(P(0.2, Math.sqrt(cubic.x[kp])), `三次時間縮放 T = ${fmt(cubic.T, 2)} s`, { color: '--fg-muted', align: 'left', dx: 8, dy: 14, font: font(), bg: '--sim-bg' });
    }
  }

  function drawFull(c: Canvas2D): void {
    if (fullFwd) {
      c.polyline(curvePts(fullFwd.pts, 2), { color: '--ok', width: 2 });
      const [s, x] = fullFwd.stop;
      c.text(P(s, Math.sqrt(Math.max(0, x))), '從起點全力加速', { color: '--ok', align: 'right', dx: -6, dy: -10, font: font(), bg: '--sim-bg' });
    }
    if (fullBwd) {
      c.polyline(curvePts(fullBwd.pts, 2), { color: '--series-4', width: 2 });
      const [s, x] = fullBwd.stop;
      c.text(P(s, Math.sqrt(Math.max(0, x))), '從終點全力減速', { color: '--series-4', align: 'left', dx: 6, dy: -10, font: font(), bg: '--sim-bg' });
    }
  }

  /** Switch marker; acc→dec labels go below-right, dec→acc (tangent) labels to the left. */
  function switchMark(c: Canvas2D, s: number, x: number, label: string, tangent: boolean, color = '--accent'): void {
    const p = P(s, Math.sqrt(Math.max(0, x)));
    c.point(p, 6, { color });
    c.point(p, 2.6, { color: '--sim-bg' });
    // While a hand-drawn curve is shown the dots are enough (the labels would cover the drawing).
    if (!label || st.show.drawn) return;
    const nearRight = s > 0.8;
    c.text(p, label, tangent
      ? { color, align: 'right', dx: -10, dy: 2, font: font(), bg: '--sim-bg' }
      : { color, align: nearRight ? 'right' : 'left', dx: nearRight ? -8 : 8, dy: 13, font: font(), bg: '--sim-bg' });
  }
  const switchLabel = (i: number) => (sol.switches[i]?.type === 'dec→acc' ? `切換 ${i + 1}（切點）` : `切換 ${i + 1}`);

  function drawOptimal(c: Canvas2D): void {
    c.polyline(curvePts(sol.pts, 2), { color: '--accent', width: 3.2 });
    sol.switches.forEach((w, i) => switchMark(c, w.s, w.x, switchLabel(i), w.type === 'dec→acc'));
  }

  function drawStages(c: Canvas2D): void {
    const stages = sol.stages;
    const marks: (() => void)[] = [];
    for (let i = 0; i <= st.stage && i < stages.length; i++) {
      const sg = stages[i];
      const latest = i === st.stage;
      const pts = sg.curve ? sg.curve.pts : sg.poly;
      switch (sg.kind) {
        case 'backward':
          if (pts) c.polyline(curvePts(pts, 2), { color: '--series-4', width: 2.2 });
          break;
        case 'forward':
          if (pts) c.polyline(curvePts(pts, 2), { color: '--ok', width: 2.2 });
          if (sg.curve?.reason === 'limit') {
            const [s, x] = sg.curve.stop;
            marks.push(() => {
              const p = P(s, Math.sqrt(Math.max(0, x)));
              c.point(p, 7, { color: '--danger', alpha: 0.35 });
              if (latest) c.text(p, '撞上極限曲線', { color: '--danger', dx: 8, dy: -12, align: 'left', font: font(), bg: '--sim-bg' });
            });
          }
          break;
        case 'probe':
          if (pts) c.polyline(curvePts(pts, 3), { color: '--fg-muted', width: 1.1, dash: [3, 3], alpha: latest ? 1 : 0.5 });
          else if (sg.point) c.point(P(sg.point[0], Math.sqrt(Math.max(0, sg.point[1]))), 4, { color: '--fg-muted' });
          break;
        case 'tangent':
          if (pts) c.polyline(curvePts(pts, 2), { color: '--series-5', width: 2.4 });
          if (sg.point && latest) { const [s, x] = sg.point; marks.push(() => switchMark(c, s, x, '切點', true, '--series-5')); }
          break;
        case 'switch':
          if (sg.point) {
            const [s, x] = sg.point;
            const n = sg.index ?? 1;
            marks.push(() => switchMark(c, s, x, switchLabel(n - 1), sol.switches[n - 1]?.type === 'dec→acc'));
          }
          break;
        case 'done':
          if (sol.ok) c.polyline(curvePts(sol.pts, 2), { color: '--accent', width: 3.2 });
          break;
        default:
          break;
      }
    }
    marks.forEach((m) => m());
  }

  function drawDrawn(c: Canvas2D): void {
    if (!drawnX || !drawnChk) {
      // still drawing: raw stroke
      c.polyline(drawn.map(([s, sd]) => P(s, sd)), { color: '--series-5', width: 2.6 });
      return;
    }
    const N = tab.N, x = drawnX, ok = drawnChk.ok;
    const all: Pt[] = [];
    for (let k = 0; k <= N; k += 2) all.push(P(k / N, Math.sqrt(x[k])));
    c.polyline(all, { color: '--series-5', width: 2.6 });
    let run: Pt[] = [];
    for (let k = 0; k < N; k++) {
      if (!ok[k]) {
        if (!run.length) run.push(P(k / N, Math.sqrt(x[k])));
        run.push(P((k + 1) / N, Math.sqrt(x[k + 1])));
      } else if (run.length) { c.polyline(run, { color: '--danger', width: 3.6 }); run = []; }
    }
    if (run.length) c.polyline(run, { color: '--danger', width: 3.6 });
  }

  function drawProbeCone(c: Canvas2D): void {
    const { s, sd } = st.probe;
    const J = jointIntervals(tab, s, sd * sd);
    const o = P(s, sd);
    const capped = sd > capSdAt(s) + 1e-9;
    if (J.L > J.U || capped) {
      c.text(o, '✕', { color: '--danger', font: font(18) });
      c.text(o, J.L > J.U ? 'L > U：沒有可行的加速度' : '超過關節速度上限', { color: '--danger', align: 'left', dx: 12, dy: -14, font: font(), bg: '--sim-bg' });
      return;
    }
    const len = 40;
    const wedge: Pt[] = [o];
    for (let i = 0; i <= 14; i++) {
      const d = dirOf(sd, J.L + ((J.U - J.L) * i) / 14);
      wedge.push(shift(c, o, d[0] * len, d[1] * len));
    }
    c.polygon(wedge, { fill: '--ok', alpha: 0.16, width: 0 });
    const du = dirOf(sd, J.U), dl = dirOf(sd, J.L);
    c.arrow(o, shift(c, o, du[0] * len, du[1] * len), { color: '--ok', width: 2.2, head: 8 });
    c.arrow(o, shift(c, o, dl[0] * len, dl[1] * len), { color: '--series-4', width: 2.2, head: 8 });
    c.text(shift(c, o, du[0] * (len + 9), du[1] * (len + 9)), 'U', { color: '--ok', font: font(12) });
    c.text(shift(c, o, dl[0] * (len + 9), dl[1] * (len + 9)), 'L', { color: '--series-4', font: font(12) });
    const a = curSdd();
    const inside = insideInterval(a, J.L, J.U);
    const da = dirOf(sd, a);
    c.arrow(o, shift(c, o, da[0] * len * 0.78, da[1] * len * 0.78), { color: inside ? '--fg' : '--danger', width: 1.6, head: 6, dash: [3, 2] });
  }

  // ---------------------------------------------------------------- interval diagram
  const ivBox = () => ({ l: 82, r: iv.width - 104, rows: [14, 33, 52], axis: 70 });
  const ivX = (a: number) => { const b = ivBox(); return b.l + ((a + R) / (2 * R)) * (b.r - b.l); };
  const ivA = (px: number) => { const b = ivBox(); return ((px - b.l) / (b.r - b.l)) * 2 * R - R; };
  const IV = (px: number, py: number) => iv.toWorld(px, py);

  iv.draw = (c) => {
    const b = ivBox(), f = font();
    const { s, sd } = st.probe;
    const x = sd * sd;
    const J = jointIntervals(tab, s, x);
    const a = curSdd();
    const finite = [J.lo[0], J.hi[0], J.lo[1], J.hi[1], a].filter((v) => Number.isFinite(v)).map((v) => Math.abs(v));
    const target = niceUp(1.15 * Math.max(1, ...finite), [2, 5, 10, 20, 50, 100, 200, 500, 1000, 5000]);
    if (target > R || target < R / 4.5) R = target;
    // axis
    c.line(IV(b.l, b.axis), IV(b.r, b.axis), { color: '--grid-strong', width: 1.2 });
    for (const v of [-R, -R / 2, 0, R / 2, R]) {
      c.line(IV(ivX(v), b.axis - 3), IV(ivX(v), b.axis + 3), { color: '--grid-strong', width: 1 });
      c.text(IV(ivX(v), b.axis + 13), fmt(v, R < 5 ? 1 : 0), { color: '--fg-muted', font: f });
    }
    c.line(IV(ivX(0), b.rows[0] - 7), IV(ivX(0), b.axis), { color: '--grid', width: 1 });
    const labels = [`關節 1 ≤ ${fmt(st.tau[0], 0)}`, `關節 2 ≤ ${fmt(st.tau[1], 1)}`, '交集 [L, U]'];
    const cols = ['--series-1', '--series-4', '--ok'];
    const bar = (rowY: number, lo: number, hi: number, col: string) => {
      if (!(lo <= hi)) return false;
      const x0 = ivX(clamp(lo, -R, R)), x1 = ivX(clamp(hi, -R, R));
      c.line(IV(x0, rowY), IV(Math.max(x1, x0 + 1.5), rowY), { color: col, width: 9, cap: 'butt', alpha: 0.75 });
      if (lo < -R) c.text(IV(b.l - 1, rowY), '◀', { color: col, align: 'right', font: font(9) });
      if (hi > R) c.text(IV(b.r + 1, rowY), '▶', { color: col, align: 'left', font: font(9) });
      return true;
    };
    for (let r = 0; r < 3; r++) {
      const y = b.rows[r];
      c.text(IV(6, y), labels[r], { color: r < 2 ? cols[r] : '--fg', align: 'left', font: f });
      c.line(IV(b.l, y), IV(b.r, y), { color: '--grid', width: 1 });
    }
    bar(b.rows[0], J.lo[0], J.hi[0], cols[0]);
    bar(b.rows[1], J.lo[1], J.hi[1], cols[1]);
    if (!bar(b.rows[2], J.L, J.U, cols[2])) {
      c.text(IV(0.5 * (b.l + b.r), b.rows[2]), '∅　兩段不重疊（L > U）', { color: '--danger', font: f, bg: '--sim-bg' });
    }
    // s̈ marker (the handle sits on the axis)
    const ax = ivX(clamp(a, -R, R));
    c.line(IV(ax, b.rows[0] - 8), IV(ax, b.axis), { color: '--fg', width: 1.3, dash: [3, 2] });
    // torques at the marker
    const tau = torquesAt(tab, s, x, a);
    for (let i = 0; i < 2; i++) {
      const over = Math.abs(tau[i]) > st.tau[i] * 1.005 + 0.02;
      c.text(IV(iv.width - 6, b.rows[i]), `τ${i === 0 ? '₁' : '₂'} = ${fmt(tau[i], 1)}${over ? ' ✗' : ''}`, { color: over ? '--danger' : '--fg', align: 'right', font: f });
    }
    const inside = J.L <= J.U && insideInterval(a, J.L, J.U);
    c.text(IV(iv.width - 6, b.rows[2]), inside ? '✓ 兩關節都行' : '✗ 超出上限', { color: inside ? '--ok' : '--danger', align: 'right', font: f });
    c.text(IV(iv.width - 6, b.axis + 1), `d²s/dt² = ${fmt(a, 2)}`, { color: '--fg', align: 'right', font: f });
    c.text(IV(6, b.axis + 1), 'd²s/dt²（1/s²）', { color: '--fg-muted', align: 'left', font: f });
  };

  // ---------------------------------------------------------------- arm drawing
  armCv.draw = (c) => {
    c.grid(0.5);
    c.frame({ x: 0, y: 0, theta: 0 }, 0.3, { labels: ['x', 'y'], width: 1.8 });
    c.arrow([-0.55, 1.7], [-0.55, 1.2], { color: '--fg-muted', width: 2, head: 8 });
    c.text([-0.55, 1.7], 'g', { color: '--fg-muted', dx: 10, dy: 2, font: font(12) });
    c.polyline(tipPath, { color: '--accent', width: 2, alpha: 0.5, dash: [5, 4] });
    const th0 = PATHS[st.path].theta(0), th1 = PATHS[st.path].theta(1);
    drawPlanarArm(c, [1, 1], th0, { alpha: 0.22, linkWidth: 6, eeRadius: 4 });
    drawPlanarArm(c, [1, 1], th1, { alpha: 0.22, linkWidth: 6, eeRadius: 4 });
    c.text(armPoints(th0).tip, '起點', { color: '--fg-muted', dx: 18, font: font() });
    c.text(armPoints(th1).tip, '終點', { color: '--fg-muted', dx: 18, font: font() });
    drawPlanarArm(c, [1, 1], PATHS[st.path].theta(st.probe.s), { linkWidth: 7, eeRadius: 5 });
    c.text([ARM_WORLD.xmax - 0.05, ARM_WORLD.ymax - 0.12], `s = ${fmt(st.probe.s, 2)}`, { color: '--fg', align: 'right', font: font(12) });
  };

  // ---------------------------------------------------------------- C-space preview
  const CS = { x0: 2.85, w: 4.6, h: 2.3 };
  const csPt = (th: Vec2): Pt => [CS.x0 + ((th[0] + Math.PI) / (2 * Math.PI)) * CS.w, (th[1] / Math.PI) * CS.h];
  const raster = new RasterLayer({ xmin: CS.x0, xmax: CS.x0 + CS.w, ymin: -CS.h, ymax: CS.h }, 110, 110);
  const paintCspace = () => {
    const col = toRGBA(cssVar('--danger', '#b91c1c'), 150);
    raster.paint((px, py) => collides([((px - CS.x0) / CS.w) * 2 * Math.PI - Math.PI, (py / CS.h) * Math.PI]), { color: col });
    pvCv.requestRender();
  };
  pvCv.draw = (c) => {
    const f = font();
    c.text([0, 2.42], '工作空間', { color: '--fg-muted', font: f });
    c.circle([0, 0], 2, { color: '--grid-strong', dash: [4, 4], width: 1 });
    c.disk(OBSTACLE.c, OBSTACLE.r, { fill: '--danger', alpha: 0.45 });
    c.polyline(tipPath, { color: '--accent', width: 1.6, alpha: 0.7 });
    drawPlanarArm(c, [1, 1], PATHS[st.path].theta(hitS), { linkWidth: 5, eeRadius: 3.5 });
    c.text(OBSTACLE.c, '碰撞！', { color: '--danger', dx: 30, dy: -16, font: f });
    // C-space panel
    c.polygon([[CS.x0, -CS.h], [CS.x0 + CS.w, -CS.h], [CS.x0 + CS.w, CS.h], [CS.x0, CS.h]], { color: '--grid-strong', width: 1 });
    raster.draw(c);
    const pts: Pt[] = [];
    const hit: boolean[] = [];
    for (let k = 0; k <= 200; k++) {
      const th = PATHS[st.path].theta(k / 200);
      pts.push(csPt(th));
      hit.push(collides(th));
    }
    c.polyline(pts, { color: '--accent', width: 2 });
    let run: Pt[] = [];
    pts.forEach((p, k) => {
      if (hit[k]) run.push(p);
      else if (run.length) { c.polyline(run, { color: '--danger', width: 4 }); run = []; }
    });
    if (run.length) c.polyline(run, { color: '--danger', width: 4 });
    c.point(pts[0], 4.5, { color: '--ok' });
    c.point(pts[pts.length - 1], 4.5, { color: '--accent' });
    c.text([CS.x0 + CS.w / 2, 2.42], '組態空間 (θ₁, θ₂)', { color: '--fg-muted', font: f });
    c.text([CS.x0 + CS.w / 2, -CS.h], 'θ₁', { color: '--fg-muted', dy: 11, font: f });
    c.text([CS.x0, 0], 'θ₂', { color: '--fg-muted', dx: -12, font: f });
    c.text([CS.x0, -CS.h], '−π', { color: '--fg-muted', dy: 11, font: font(10) });
    c.text([CS.x0 + CS.w, -CS.h], 'π', { color: '--fg-muted', dy: 11, font: font(10) });
  };
  paintCspace();
  const unsubTheme = ctx.onThemeChange(() => paintCspace());

  // ---------------------------------------------------------------- handles & pointer
  const probeHandle = cv.addHandle({
    get: () => P(st.probe.s, st.probe.sd),
    set: (p) => {
      const [s, sd] = toData(p);
      st.probe.s = clamp(s, 0, 1);
      st.probe.sd = clamp(sd, 0, sdTop());
      renderAll();
    },
    onStart: () => { stopPlay(); st.sdd = curSdd(); st.following = false; },
    radius: 7,
    color: '--fg',
  });
  iv.addHandle({
    get: () => IV(ivX(clamp(curSdd(), -R, R)), ivBox().axis),
    set: (p) => { const [px] = iv.toScreen(p); st.sdd = clamp(ivA(px), -R, R); renderAll(); },
    onStart: () => { stopPlay(); st.sdd = curSdd(); st.following = false; },
    radius: 7,
    color: '--fg',
  });
  let drawing = false;
  const unsubPP = cv.onPointer({
    down: (p, ev) => {
      if (!st.drawMode) return;
      stopPlay();
      st.following = false;
      drawing = true;
      cv.canvas.setPointerCapture(ev.pointerId);
      const [s, sd] = toData(p);
      drawn = [[clamp(s, 0, 1), clamp(sd, 0, sdTop())]];
      drawnX = null; drawnChk = null;
      cv.requestRender();
    },
    move: (p) => {
      if (!drawing) return;
      const [s0, sd0] = toData(p);
      const s = clamp(s0, 0, 1), sd = clamp(sd0, 0, sdTop());
      while (drawn.length && drawn[drawn.length - 1][0] > s) drawn.pop();
      drawn.push([s, sd]);
      cv.requestRender();
    },
    up: () => { if (!drawing) return; drawing = false; finishDrawing(); },
  });

  // ---------------------------------------------------------------- steps
  let focused: HTMLElement[] = [];
  const setFocus = (els: HTMLElement[]) => {
    focused.forEach((f) => f.classList.remove('mr-focus'));
    focused = els;
    focused.forEach((f) => f.classList.add('mr-focus'));
  };
  const STEP_PATH: PathId[] = ['corner', 'corner', 'corner', 'line', 'corner', 'corner', 'corner'];
  const STEP_TAB: Tab[] = ['c', 'm', 'c', 'tau', 'tau', 'tau', 'tau'];

  const onStep = (i: number) => {
    stopPlay();
    st.step = i;
    st.fresh = true;
    st.t = 0;
    st.tau = [DEFAULT_TAU[0], DEFAULT_TAU[1]];
    sT1.set(st.tau[0], { silent: true });
    sT2.set(st.tau[1], { silent: true });
    st.capOn = i === 6;
    st.cap = DEFAULT_CAP;
    sCap.set(st.cap, { silent: true });
    syncCapUI();
    st.show = {
      field: i === 1 || i === 2,
      limit: i >= 2,
      full: i === 3,
      algo: i === 4,
      opt: i >= 3,
      cubic: i === 0 || i === 3,
      strip: i >= 1,
      drawn: i === 5,
    };
    tgField.set(st.show.field, { silent: true });
    tgCubic.set(st.show.cubic, { silent: true });
    iv.el.style.display = st.show.strip ? '' : 'none';
    algoRow.style.display = i === 4 ? '' : 'none';
    preview.style.display = i === 6 ? '' : 'none';
    plotArea.style.display = i === 6 ? 'none' : '';
    setDrawMode(i === 5);
    st.play = i === 0 ? 'cubic' : 'opt';
    const path = STEP_PATH[i] ?? 'corner';
    if (path !== st.path || !tab) setPath(path); else recompute();
    if (i === 5 && !drawn.length) { drawn = exampleDrawing(); finishDrawing(); }
    if (i === 5) { st.play = drawnX ? 'drawn' : 'opt'; }
    playSel.set(st.play, { silent: true });
    updatePlayCurve();
    tabs.activate(['m', 'c', 'g', 'tau'].indexOf(STEP_TAB[i] ?? 'm'));
    // Where the probe starts in each step (s, ṡ) or which moment of the playback curve it shows.
    st.following = false;
    st.sdd = 0;
    switch (i) {
      case 0: st.probe = { s: 0.45, sd: cubic ? Math.sqrt(cubic.x[Math.round(0.45 * tab.N)]) : 0.4 }; break;
      case 1: st.probe = { s: 0.3, sd: 1.2 }; break;
      case 2: st.probe = { s: 0.45, sd: 1.9 }; break;
      case 3: if (tm) { st.following = true; st.t = 0.35 * tm.T; } break;
      case 4: {
        const fw = sol.stages.find((sg) => sg.kind === 'forward')?.curve;
        st.probe = fw ? { s: fw.stop[0], sd: 0.97 * Math.sqrt(Math.max(0, fw.stop[1])) } : { s: 0.45, sd: 1 };
        // Put the s̈ marker inside the (nearly collapsed) interval at that point.
        const J = jointIntervals(tab, st.probe.s, st.probe.sd ** 2);
        if (J.L <= J.U) st.sdd = 0.5 * (J.L + J.U);
        break;
      }
      case 5: {
        let k = drawnChk ? drawnChk.ok.findIndex((g) => !g) : -1;
        if (k >= 0 && drawnChk) { let e = k; while (e < drawnChk.ok.length && !drawnChk.ok[e]) e++; k = Math.round((k + e) / 2); }
        if (tm && k >= 0) { st.following = true; st.t = timeAtS(k / tab.N); }
        break;
      }
      case 6: if (tm) { st.following = true; st.t = timeAtS(0.62); } break;
      default: break;
    }
    if (st.following) applyPlayState(); else pb.setTime(0);
    updateReadouts();
    renderAll();
    pvCv.requestRender();
    switch (i) {
      case 0: setFocus([armCv.el, pCoef.el]); break;
      case 1: setFocus([iv.el, cv.el]); break;
      case 2: setFocus([cv.el, sT1.el, sT2.el]); break;
      case 3: setFocus([cv.el, pTau.el]); break;
      case 4: setFocus([algoRow, cv.el]); break;
      case 5: setFocus([cv.el, rDraw.el]); break;
      case 6: setFocus([tgCap.el, preview]); break;
      default: setFocus([]);
    }
  };

  setPath(st.path);
  showTab('m');
  onStep(ctx.currentStep);

  return {
    onStep,
    destroy() {
      loop.stop();
      stepper.destroy();
      unsubPP();
      unsubTheme();
      cv.destroy();
      iv.destroy();
      armCv.destroy();
      pvCv.destroy();
      pCoef.destroy();
      pTau.destroy();
    },
  };
}

const lesson: Lesson = { meta, steps, quiz, mount };
export default lesson;
