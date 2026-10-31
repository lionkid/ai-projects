import type { Lesson, LessonContext, LessonHandle, LessonStep, QuizQuestion } from '@/types';
import type * as THREE from 'three';
import {
  el, fmt, controlPanel, row, slider, angleSlider, toggle, button, readout, playback, simTitle, note, split, legend,
  Scene3D, ScrewAxis3D, Plot2D, createLoop, presetChips, plotGroup, mathEl, mdElement, setMatrix,
} from '@/ui';
import {
  type Mat, type Vec, Adjoint, InverseDynamics, QuinticTimeScalingDerivs, RRR3_DIMENSIONS, RotY, RpToTrans,
  linkFramePoses, matMul, matVec, rrr3, transformPoint,
} from '@/mr';
import { meta } from './meta';
import {
  type ChainModel, type CostSample, type RneaTrace, type TorqueParts, baseCurves, bodyWrenchParts, boxSpatialInertia,
  measureCost, measureModelCost, rneaTrace, scaleCurves, spinAt, wrenchInFrame, type BaseCurves,
} from './rne';

// ---------------------------------------------------------------------------
// Narrative
// ---------------------------------------------------------------------------

const steps: LessonStep[] = [
  {
    title: '一個剛體：twist 與 wrench 的關係',
    md: `先放下手臂，只看一個剛體。在質心座標系 $\\{b\\}$ 中，把轉動慣量 $\\mathcal I_b$ 與質量 $\\mathfrak m$ 疊成 6×6 的**空間慣量** $\\mathcal G_b = \\mathrm{diag}(\\mathcal I_b,\\ \\mathfrak m I)$，Newton 與 Euler 的方程就合成一行 twist–wrench 關係：

$$\\mathcal F_b = \\mathcal G_b\\dot{\\mathcal V}_b - [\\mathrm{ad}_{\\mathcal V_b}]^{\\T}\\mathcal G_b\\mathcal V_b$$

第一項是熟悉的「質量 × 加速度」。第二項來自 $\\{b\\}$ 跟著物體一起轉，它的轉動部分正是 $\\omega_b \\times \\mathcal I_b\\omega_b$。所以就算轉速固定（$\\dot{\\mathcal V}_b = 0$），只要轉軸不是主軸，軸承仍得持續提供力矩。`,
    simHint: '盒子以固定轉速繞傾斜的軸旋轉，橘色箭頭是軸承必須施加的力矩。關掉「−adᵀGV 項」，力矩瞬間歸零——這個省略錯在哪？把傾角調成 0° 再看一次。',
  },
  {
    title: '換到別的座標系',
    md: `同一個剛體也能在任何黏在物體上的座標系 $\\{a\\}$ 中描述，例如關節所在的位置。twist 依 $\\mathcal V_a = [\\mathrm{Ad}_{T_{ab}}]\\mathcal V_b$ 換座標；動能 $\\tfrac12\\mathcal V^{\\T}\\mathcal G\\mathcal V$ 不會因為換座標而改變，由此得到

$$\\mathcal G_a = [\\mathrm{Ad}_{T_{ba}}]^{\\T}\\,\\mathcal G_b\\,[\\mathrm{Ad}_{T_{ba}}]$$

$\\{a\\}$ 不在質心時，$\\mathcal G_a$ 的非對角區塊出現 $\\mathfrak m[p]$ 形式的耦合，左上角則多出平行軸定理的 $-\\mathfrak m[p]^2$。方程 $\\mathcal F_a = \\mathcal G_a\\dot{\\mathcal V}_a - [\\mathrm{ad}_{\\mathcal V_a}]^{\\T}\\mathcal G_a\\mathcal V_a$ 的形式完全不變，算出的 wrench 也與 $[\\mathrm{Ad}_{T_{ba}}]^{\\T}\\mathcal F_b$ 一致。`,
    simHint: '轉軸已移到盒子一端的 {a}：看 G_a 中標色的耦合區塊，以及紅色的力箭頭（質心在繞圈）。下方兩種算法的 F_a 差值只有 10⁻¹⁵ 等級；把 r 拉回 0 比較。',
  },
  {
    title: '前向遞迴：速度與加速度往外傳',
    md: `現在把 $n$ 個剛體串成手臂。Newton–Euler 遞迴先**由基座往外**：每根連桿的 twist，是上一根連桿的 twist 搬到自己的座標系，再加上自己關節的貢獻：

$$\\mathcal V_i = [\\mathrm{Ad}_{T_{i,i-1}}]\\,\\mathcal V_{i-1} + \\mathcal A_i\\dot\\theta_i$$

$\\mathcal A_i$ 是關節 $i$ 的螺旋軸在連桿座標系 $\\{i\\}$（位於質心）中的表示。加速度 $\\dot{\\mathcal V}_i$ 照同樣的方式傳遞，另多一項 $[\\mathrm{ad}_{\\mathcal V_i}]\\mathcal A_i\\dot\\theta_i$。一個小技巧：令基座加速度 $\\dot{\\mathcal V}_0 = (0, -\\mathfrak g)$，彷彿整個世界以 $\\mathfrak g$ 向上加速，重力就自動進入每根連桿的加速度。`,
    simHint: '手臂停在軌跡上的一個瞬間。按「▸ 下一階段」依序算出 V₁、V₂、V₃：紫色箭頭是角速度、藍色是質心速度。V̇ᵢ 的線性部分總含有 9.81 m/s² 的向上分量。',
  },
  {
    title: '後向遞迴：wrench 往回傳',
    md: `速度與加速度都有了，再**由末端往回**：連桿 $i$ 承受的 wrench，等於外側連桿傳來的 wrench（換到 $\\{i\\}$）加上自己的單剛體方程：

$$\\mathcal F_i = [\\mathrm{Ad}_{T_{i+1,i}}]^{\\T}\\mathcal F_{i+1} + \\mathcal G_i\\dot{\\mathcal V}_i - [\\mathrm{ad}_{\\mathcal V_i}]^{\\T}\\mathcal G_i\\mathcal V_i$$

起點 $\\mathcal F_{n+1} = \\mathcal F_{\\text{tip}}$ 是末端施加在環境上的 wrench。關節只能沿自己的軸出力，馬達力矩是投影 $\\tau_i = \\mathcal F_i^{\\T}\\mathcal A_i$，其餘分量由軸承與結構承受。越靠近基座，$\\mathcal F_i$ 累積的負擔越大。`,
    simHint: '前向遞迴已完成，畫面停在 F₃。繼續按「▸ 下一階段」：紅色箭頭是關節傳給連桿的力、橙色是力矩；τ₃ → τ₂ → τ₁ 依序算出，最後與 mr.InverseDynamics 比對。',
  },
  {
    title: '沿軌跡的力矩曲線',
    md: `對軌跡上每個取樣點 $(\\theta,\\dot\\theta,\\ddot\\theta)$ 跑一次遞迴，就得到力矩曲線 $\\tau_i(t)$。把輸入拆開分別呼叫，可以看出每部分的貢獻：只給 $\\ddot\\theta$ 得慣性項 $M(\\theta)\\ddot\\theta$，只給 $\\dot\\theta$ 得速度項 $c(\\theta,\\dot\\theta)$，只給 $\\mathfrak g$ 得重力項 $g(\\theta)$，只給 $\\mathcal F_{\\text{tip}}$ 得負載項 $J_b^{\\T}\\mathcal F_{\\text{tip}}$。

四者相加恰好等於一次完整的呼叫：$\\tau$ 對 $\\ddot\\theta$、$\\mathfrak g$、$\\mathcal F_{\\text{tip}}$ 是線性的，對 $\\dot\\theta$ 是二次式，彼此之間沒有交叉項。`,
    simHint: '播放軌跡，游標同步掃過兩張圖。用「分解」切換關節：哪個關節幾乎只在對抗重力？哪個關節完全不受重力影響？',
  },
  {
    title: '快一點、重一點',
    md: `同一條路徑走得更快：時長變成 $T/k$，$\\dot\\theta$ 變成 $k$ 倍、$\\ddot\\theta$ 變成 $k^2$ 倍，所以慣性項與速度項（$\\dot\\theta$ 的二次式）都乘上 $k^2$；重力項只看構型，完全不變。時長減半，動態部分就變成 **4 倍**。

末端負載則從後向遞迴的起點 $\\mathcal F_{\\text{tip}}$ 一路傳進每個關節，關節 $i$ 分到的是負載力對其軸的力矩。rrr3 的基座軸是鉛直的，與重量平行，所以靜態的重量不會在它上面產生力矩。`,
    simHint: '按「時長減半」：虛線是之前的曲線，讀數中動態部分的倍率應為 4.00、重力部分為 1.00。再按「末端加 1 kg」，比較三個關節的變化。',
  },
  {
    title: 'O(n) 與下一課',
    md: `每次遞迴對每根連桿只做固定份量的工作（幾個 6×6 矩陣乘向量），往外一趟、往回一趟，計算量與關節數 $n$ 成正比：$O(n)$。相比之下，$M(\\theta)$ 有 $n^2$ 個元素；用 $n$ 次逆動力學逐欄組出 $M$ 需要 $O(n^2)$，若把 Lagrange 閉式展開成符號公式，項數更會急速膨脹。

逆動力學回答「要這樣動，需要多少力矩」。下一課反過來：給定力矩，解 $\\ddot\\theta = M^{-1}(\\tau - c - g)$ 再做數值積分，讓手臂在模擬中真正動起來。`,
    simHint: '看量測結果：InverseDynamics 的時間大致隨 n 線性成長，組出 M(θ) 則接近 n²。按「重新量測」可在你的電腦上再跑一次。',
  },
];

const quiz: QuizQuestion[] = [
  {
    question: '把同一條軌跡的時長從 2 s 縮成 1 s。在相同的正規化時間 $t/T$ 上比較，哪些力矩分量會變成原來的 4 倍？',
    options: [
      '只有慣性項 $M(\\theta)\\ddot\\theta$',
      '慣性項與速度項 $c(\\theta,\\dot\\theta)$；重力項不變',
      '慣性、速度、重力三項都變成 4 倍',
      '慣性項變 4 倍，速度項變 2 倍',
    ],
    answer: 1,
    explanation: '以 $s = t/T$ 描述同一條路徑：$\\dot\\theta = \\theta\'(s)/T$、$\\ddot\\theta = \\theta\'\'(s)/T^2$。慣性項正比於 $\\ddot\\theta$，速度項是 $\\dot\\theta$ 的二次式，兩者都正比於 $1/T^2$，時長減半就乘以 4；$g(\\theta)$ 只依賴構型，在相同的 $s$ 上完全一樣。',
  },
  {
    question: 'rrr3 手臂靜止不動（$\\dot\\theta = \\ddot\\theta = 0$），末端握著 1 kg 的物體。各關節需要的力矩由什麼決定？',
    options: [
      '$\\tau = g(\\theta) + J_b^{\\T}(\\theta)\\,\\mathcal F_{\\text{tip}}$；鉛直的基座軸與重量平行，物體的重量不會在它上面產生力矩',
      '$\\tau = M(\\theta)\\ddot\\theta$，靜止時全部為 0',
      '只有最靠近末端的關節需要出力，其餘關節不受影響',
      '三個關節平均分擔 1 kg × 9.81 m/s² 的重量',
    ],
    answer: 0,
    explanation: '靜止時慣性項與速度項都是 0。後向遞迴從 $\\mathcal F_{\\text{tip}}$ 出發往內傳，每個關節都收到負載的 wrench，分到的力矩是負載力對自己關節軸的力矩，再加上手臂自身的 $g(\\theta)$。基座軸與重力平行，鉛直力對它沒有力矩；肩、肘兩軸則都得分擔。',
  },
  {
    question: '為什麼遞迴 Newton–Euler 能在 $O(n)$ 時間算出逆動力學，而先組出 $M(\\theta)$ 再計算比較貴？',
    options: [
      '遞迴只需要算 $\\tau$ 這個 $n$ 維向量：每根連桿往外、往回各處理一次，每次工作量固定；$M(\\theta)$ 有 $n^2$ 個元素，光是組出來就至少要 $O(n^2)$',
      '遞迴忽略了科氏與向心項，所以比較快',
      '$M(\\theta)$ 只能用數值微分求得',
      '兩者一樣快，只是遞迴比較好寫',
    ],
    answer: 0,
    explanation: '遞迴把每根連桿的 twist、加速度與 wrench 各算一次，速度項與重力也都包含在內，工作量與 $n$ 成正比。$M(\\theta)$ 是 $n\\times n$ 矩陣，用逆動力學逐欄組出要呼叫 $n$ 次、共 $O(n^2)$；符號展開的閉式則隨關節數爆炸。模擬第 7 步的量測曲線就是這兩種成長率。',
  },
  {
    question: '一塊長方形板以**固定角速度**繞一根通過質心、但**不是主軸**的軸旋轉（$\\dot{\\mathcal V}_b = 0$）。軸承必須施加的 wrench 是？',
    options: [
      '零：轉速固定、沒有加速度，所以不需要任何力或力矩',
      '只有力，沒有力矩',
      '力矩 $\\omega_b \\times \\mathcal I_b\\omega_b$：在 $\\{b\\}$ 中是常數，從空間看則跟著板一起轉',
      '一個在空間中方向固定的力矩',
    ],
    answer: 2,
    explanation: '$\\dot{\\mathcal V}_b = 0$ 時 $\\mathcal F_b = -[\\mathrm{ad}_{\\mathcal V_b}]^{\\T}\\mathcal G_b\\mathcal V_b$，轉動部分是 $\\omega_b\\times\\mathcal I_b\\omega_b$；轉軸不是主軸時 $\\mathcal I_b\\omega_b$ 與 $\\omega_b$ 不平行，這個力矩不為零。它在 $\\{b\\}$ 中固定，而 $\\{b\\}$ 在轉，所以從空間看力矩方向一直在轉——不平衡轉子會讓軸承振動，就是這個原因。質心不動，力為零。',
  },
];

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

const DEG = Math.PI / 180;
const G_VEC = [0, 0, -9.81];
const BOX_DIMS: [number, number, number] = [0.4, 0.2, 0.06];
const BOX_MASS = 3;
const GB = boxSpatialInertia(BOX_MASS, BOX_DIMS);
const BODY_Z = 0.42;
const SPIN_PERIOD = 4;
const N_SAMPLES = 161;
/** Normalised time of the frozen state used for the step-by-step recursion. */
const S_FREEZE = 0.3;
const MODEL: ChainModel = { M: rrr3.M, Mlist: rrr3.Mlist!, Glist: rrr3.Glist!, Slist: rrr3.Slist };
const { L0, L1 } = RRR3_DIMENSIONS;
/** Chain sizes for the cost measurement (each one doubles the previous). */
const COST_NS = [3, 6, 12, 24];
const ARM_VIEW: { pos: [number, number, number]; target: [number, number, number] } = { pos: [1.05, -1.18, 0.98], target: [0.08, 0, 0.36] };
const BODY_VIEW: { pos: [number, number, number]; target: [number, number, number] } = { pos: [0.62, -0.8, 0.74], target: [0, 0, 0.4] };

/** Scale the text sprites under `root` (frame labels are sized for ~1 m robots; this scene is smaller). */
function shrinkLabels(root: THREE.Object3D, k: number, only?: string): void {
  root.traverse((o) => {
    const sp = o as THREE.Sprite;
    if (sp.isSprite && (!only || (sp.userData.themeText as { text?: string } | undefined)?.text === only)) sp.scale.multiplyScalar(k);
  });
}

interface TrajPreset { label: string; title: string; start: [number, number, number]; end: [number, number, number] }
const TRAJS: TrajPreset[] = [
  { label: '綜合動作', title: '三個關節同時運動：θ = (−45°, 0°, 45°) → (45°, −45°, −30°)', start: [-45, 0, 45], end: [45, -45, -30] },
  { label: '水平揮臂', title: '只轉基座關節：θ₁ 從 −70° 到 70°', start: [-70, -15, 40], end: [70, -15, 40] },
  { label: '向上舉起', title: '肩、肘把手臂舉高：θ = (0°, 0°, 60°) → (0°, −60°, 30°)', start: [0, 0, 60], end: [0, -60, 30] },
];

const SUB = '₀₁₂₃₄₅₆₇₈₉';
const sub = (n: number) => String(n).replace(/\d/g, (d) => SUB[Number(d)]);
const SUP = '⁰¹²³⁴⁵⁶⁷⁸⁹';
function sci(x: number, digits = 1): string {
  if (!Number.isFinite(x)) return String(x);
  if (x === 0) return '0';
  let e = Math.floor(Math.log10(Math.abs(x)));
  let m = x / 10 ** e;
  if (Math.abs(Number(m.toFixed(digits))) >= 10) { e += 1; m /= 10; }
  return `${m.toFixed(digits)}×10${String(e).replace('-', '⁻').replace(/\d/g, (d) => SUP[Number(d)])}`;
}

const texLabel = (tex: string, prefix = '', suffix = ''): HTMLSpanElement =>
  el('span', {}, prefix, el('span', { style: { fontSize: '0.95em', lineHeight: '1' } }, mathEl(tex)), suffix);

/** 1 × 6 row vector view (angular | linear) whose entries may still be unknown. */
function vec6(parent: HTMLElement, label: Node): { el: HTMLElement; set(v: Vec | null, hl?: boolean): void } {
  const cells = Array.from({ length: 6 }, () => el('span', { class: 'mr-cell' }, '?'));
  const grid = el('div', { class: 'mr-grid', style: { gridTemplateColumns: 'repeat(3, auto) 0.35rem repeat(3, auto)' } },
    ...cells.slice(0, 3), el('span', {}), ...cells.slice(3));
  const wrap = el('div', { class: 'mr-matrix', style: { margin: '0.1rem 0' } }, el('span', { class: 'mr-mlabel', style: { minWidth: '3.3rem', textAlign: 'right' } }, label), grid);
  parent.appendChild(wrap);
  return {
    el: wrap,
    set(v, hl) {
      cells.forEach((c, k) => {
        const t = v ? fmt(v[k], 3) : '?';
        if (c.textContent !== t) c.textContent = t;
        const cls = `mr-cell${hl ? ' hl' : ''}`;
        if (c.className !== cls) c.className = cls;
      });
    },
  };
}

/** 6 × 6 matrix view with the off-diagonal (coupling) blocks highlighted. */
function blockMatrix(parent: HTMLElement, label: Node): { el: HTMLElement; set(M: Mat): void } {
  const cells = Array.from({ length: 36 }, (_, k) => {
    const r = Math.floor(k / 6), c = k % 6;
    return el('span', { class: `mr-cell${(r < 3) !== (c < 3) ? ' hl' : ''}` }, '');
  });
  const grid = el('div', { class: 'mr-grid', style: { gridTemplateColumns: 'repeat(6, auto)' } }, ...cells);
  const wrap = el('div', { class: 'mr-matrix' }, el('span', { class: 'mr-mlabel' }, label), grid);
  parent.appendChild(wrap);
  return {
    el: wrap,
    set(M) { cells.forEach((c, k) => { const t = fmt(M[Math.floor(k / 6)][k % 6], 3); if (c.textContent !== t) c.textContent = t; }); },
  };
}

const rot = (T: Mat, v: Vec): Vec => [
  T[0][0] * v[0] + T[0][1] * v[1] + T[0][2] * v[2],
  T[1][0] * v[0] + T[1][1] * v[1] + T[1][2] * v[2],
  T[2][0] * v[0] + T[2][1] * v[1] + T[2][2] * v[2],
];
const origin = (T: Mat): Vec => [T[0][3], T[1][3], T[2][3]];
const add3 = (a: Vec, b: Vec, k = 1): Vec => [a[0] + k * b[0], a[1] + k * b[1], a[2] + k * b[2]];
const norm3 = (v: Vec) => Math.hypot(v[0], v[1], v[2]);

// ---------------------------------------------------------------------------
// Simulation
// ---------------------------------------------------------------------------

function mount(host: HTMLElement, ctx: LessonContext): LessonHandle {
  simTitle(host, '單剛體與 Newton–Euler 逆動力學', 'rrr3');
  const chips = presetChips(host, [], { label: '情境' });
  const scene = new Scene3D(host, { aspect: 16 / 9, grid: 2, axes: 0.18, cameraPos: ARM_VIEW.pos, target: ARM_VIEW.target, autoStart: false });
  shrinkLabels(scene.scene, 0.45, '{s}');

  // ---------------- state
  let curStep = -1;
  let mode: 'body' | 'recursion' | 'traj' | null = null;
  // single body
  let omega0 = 4, tiltDeg = 30, offsetR = 0, varying = false, withAd = true;
  let tb = 0;
  // arm
  let trajIdx = 0;
  let Tf = 2;
  let gravityOn = true;
  let loadMass = 0;
  let decJoint = 0;
  let ta = 0;
  let base: BaseCurves = baseCurves(MODEL, TRAJS[0].start.map((d) => d * DEG), TRAJS[0].end.map((d) => d * DEG), N_SAMPLES, G_VEC);
  let curves: TorqueParts[] = scaleCurves(base, Tf, gravityOn, loadMass);
  let before: { curves: TorqueParts[]; label: string } | null = null;
  // recursion
  let stage = 0;
  let trace: RneaTrace | null = null;
  let frozen: { th: Vec; dth: Vec; ddth: Vec; Ftip: Vec } | null = null;
  let autoTimer = 0;
  // costs
  let costs: CostSample[] = [];
  let costTimer = 0;
  let costModelUs = 0;

  // ---------------- 3D objects: single body
  const box = scene.addBox(BOX_DIMS, '--series-1', 0.42);
  const frameB = scene.addFrame({ size: 0.16, label: '{b}' });
  const frameA = scene.addFrame({ size: 0.12, label: '{a}' });
  shrinkLabels(frameB.group, 0.45);
  shrinkLabels(frameA.group, 0.45);
  const axisLine = scene.addPolyline([[0, 0, 0], [0, 0, 1]], '--fg-muted', { dashed: true });
  const pivot = scene.addSphere(0.014, '--fg-muted');
  const wArrow = scene.addArrow({ color: '--series-5', radius: 0.007, head: 0.035 });
  const mArrow = scene.addArrow({ color: '--series-4', radius: 0.009, head: 0.045 });
  const fArrow = scene.addArrow({ color: '--series-2', radius: 0.009, head: 0.045 });
  const bodyObjs = () => [box, axisLine.line, pivot];

  // ---------------- 3D objects: rrr3 arm
  const baseBlock = scene.addBox([0.14, 0.14, 0.03], '--grid-strong');
  baseBlock.position.set(0, 0, 0.015);
  const links = [scene.addLink({ radius: 0.04 }), scene.addLink({ radius: 0.028 }), scene.addLink({ radius: 0.024 })];
  const joints = [scene.addSphere(0.042, '--fg'), scene.addSphere(0.034, '--fg')];
  const comFrames = [1, 2, 3].map((i) => scene.addFrame({ size: 0.085, label: `{${i}}` }));
  comFrames.forEach((f) => shrinkLabels(f.group, 0.4));
  const jointAxis = new ScrewAxis3D({ color: '--accent', length: 0.2, size: 0.35 });
  scene.scene.add(jointAxis.group);
  const vArrow = scene.addArrow({ color: '--series-1', radius: 0.007, head: 0.035 });
  const loadBox = scene.addBox([0.06, 0.06, 0.06], '--warn');
  const rope = scene.addLink({ radius: 0.003, color: '--fg-muted' });
  const toolPath = scene.addTube([[0, 0, 0], [0, 0, 0.01]], { radius: 0.004, color: '--warn', opacity: 0.55 });
  const armObjs = () => [baseBlock, ...links.map((l) => l.mesh), ...joints, toolPath.mesh];

  // ---------------- transport rows
  const panel = controlPanel(host, { compact: true });
  const tRow = row(panel);
  const pbBody = playback(tRow, {
    onPlay: () => startLoop(), onPause: () => stopLoop(), onReset: () => { tb = 0; ta = 0; refresh(); },
    onSpeed: (s) => { loop.speed = s; }, speeds: [0.25, 0.5, 1, 2],
  });
  const pbArm = playback(tRow, {
    onPlay: () => startLoop(), onPause: () => stopLoop(), onReset: () => { ta = 0; refresh(); },
    onSpeed: (s) => { loop.speed = s; }, speeds: [0.25, 0.5, 1, 2], duration: 2 * Tf,
    onScrub: (t) => { ta = t; refresh(); },
  });
  // recursion stage controls
  const stRow = el('div', { class: 'mr-playback' });
  tRow.appendChild(stRow);
  const btnPrev = button(stRow, { label: '◂ 上一階段', small: true, onClick: () => setStage(stage - 1) });
  const btnNext = button(stRow, { label: '▸ 下一階段', small: true, variant: 'primary', onClick: () => setStage(stage + 1) });
  const btnAuto = button(stRow, { label: '▶ 自動', small: true, onClick: () => toggleAuto() });
  button(stRow, { label: '⟲ 重設', small: true, onClick: () => { stopAuto(); setStage(curStep === 3 ? 4 : 0); } });
  const stageLabel = el('span', { class: 'mr-time' }, '');
  stRow.appendChild(stageLabel);

  const section = () => { const d = el('div'); host.appendChild(d); return d; };

  // ---------------- section: single body (steps 1–2)
  const secBody = section();
  const bodyCtl = controlPanel(secBody, { compact: true });
  const sOmega = slider(bodyCtl, { label: '平均轉速 ω₀', min: 0.5, max: 6, step: 0.1, value: omega0, unit: 'rad/s', onInput: (v) => { omega0 = v; chips.clear(); refresh(); } });
  const sTilt = angleSlider(bodyCtl, { label: '轉軸傾角（相對主軸 z_b）', value: tiltDeg * DEG, min: 0, max: Math.PI / 2, onInput: (v) => { tiltDeg = v / DEG; chips.clear(); updateAxisLine(); refresh(); } });
  const sOffset = slider(bodyCtl, { label: '{a} 與轉軸離質心 r', min: 0, max: 0.2, step: 0.01, value: 0, unit: 'm', onInput: (v) => { offsetR = v; chips.clear(); updateAxisLine(); refresh(); } });
  const bodyRow = row(bodyCtl);
  const tVary = toggle(bodyRow, { label: '轉速週期變化', value: false, onChange: (v) => { varying = v; chips.clear(); refresh(); } });
  const tAd = toggle(bodyRow, { label: '包含 −adᵀGV 項', value: true, onChange: (v) => { withAd = v; refresh(); } });
  tAd.el.querySelector('span')?.replaceChildren('包含 ', mathEl('-[\\mathrm{ad}_{\\mathcal V}]^{\\mathsf T}\\mathcal G\\mathcal V'), ' 項');
  const vecBox = el('div', { style: { display: 'flex', flexDirection: 'column', gap: '0.05rem', margin: '0.2rem 0' } });
  secBody.appendChild(vecBox);
  const vGVd = vec6(vecBox, texLabel('\\mathcal G\\dot{\\mathcal V} ='));
  const vAdT = vec6(vecBox, texLabel('-[\\mathrm{ad}]^{\\mathsf T}\\mathcal G\\mathcal V ='));
  const vF = vec6(vecBox, texLabel('\\mathcal F_b ='));
  const frameBox = el('div', { style: { display: 'flex', flexWrap: 'wrap', alignItems: 'center', gap: '0.2rem 0.8rem' } });
  secBody.appendChild(frameBox);
  const mGa = blockMatrix(frameBox, texLabel('\\mathcal G_a ='));
  const vecBoxA = el('div', { style: { display: 'flex', flexDirection: 'column', gap: '0.05rem' } });
  frameBox.appendChild(vecBoxA);
  const vFa = vec6(vecBoxA, texLabel('\\mathcal F_a ='));
  const vFaB = vec6(vecBoxA, texLabel('\\mathrm{Ad}^{\\mathsf T}\\mathcal F_b ='));
  const rDiffA = readout(vecBoxA, { label: '兩者最大差值', minWidth: '8ch' });
  const bodyNote = note(secBody, '');
  const fLegend = legend(secBody, [{ color: 'var(--axis-x)', label: 'x 分量' }, { color: 'var(--axis-y)', label: 'y 分量' }, { color: 'var(--axis-z)', label: 'z 分量' }]);
  fLegend.appendChild(el('span', {}, '實線＝力矩 m（N·m）、虛線＝力 f（N）'));
  const fPlot = new Plot2D(secBody, {
    title: '所需 wrench 的分量（在 {b} 中）', xLabel: 't (s)', yLabel: 'N·m / N', height: 140, xWindow: 6, yPad: 0.12, yMinSpan: 0.2, legend: false,
    series: [
      { name: 'm_x', color: '--axis-x' }, { name: 'm_y', color: '--axis-y' }, { name: 'm_z', color: '--axis-z' },
      { name: 'f_x', color: '--axis-x', dash: [5, 4] }, { name: 'f_y', color: '--axis-y', dash: [5, 4] }, { name: 'f_z', color: '--axis-z', dash: [5, 4] },
    ],
  });

  // ---------------- section: recursion (steps 3–4)
  const secRec = section();
  const pipe = el('div', { style: { display: 'flex', flexWrap: 'wrap', gap: '0.3rem', alignItems: 'center', margin: '0.2rem 0 0.3rem', fontSize: '0.8rem' } });
  secRec.appendChild(pipe);
  const PIPE = ['V₁', 'V₂', 'V₃', 'F₃', 'F₂', 'F₁'];
  const pills = PIPE.map((p, k) => {
    const pill = el('span', { style: { padding: '0.1rem 0.55rem', borderRadius: '999px', border: '1px solid var(--border)', fontFamily: 'var(--mono)' } }, `${k < 3 ? '→' : '←'} ${p}`);
    pipe.appendChild(pill);
    if (k === 2) pipe.appendChild(el('span', { style: { color: 'var(--fg-muted)' } }, '│'));
    return pill;
  });
  const stateLine = el('div', { class: 'mr-note', style: { margin: '0.1rem 0' } });
  secRec.appendChild(stateLine);
  const formula = el('div', { style: { minHeight: '3.4rem', overflowX: 'auto', fontSize: '0.92rem', padding: '0.1rem 0' } });
  secRec.appendChild(formula);
  const recBox = el('div', { style: { display: 'flex', flexDirection: 'column', gap: '0.05rem' } });
  secRec.appendChild(recBox);
  const vV = vec6(recBox, texLabel('\\mathcal V_i ='));
  const vVd = vec6(recBox, texLabel('\\dot{\\mathcal V}_i ='));
  const vFi = vec6(recBox, texLabel('\\mathcal F_i ='));
  const tauRow = el('div', { style: { display: 'flex', flexWrap: 'wrap', gap: '0.1rem 1.2rem', marginTop: '0.2rem' } });
  secRec.appendChild(tauRow);
  const rTau = [0, 1, 2].map((i) => readout(tauRow, { label: `τ${sub(i + 1)} =`, unit: 'N·m', minWidth: '6ch' }));
  const rCheck = readout(tauRow, { label: '與 mr.InverseDynamics 差', minWidth: '7ch' });

  // ---------------- section: trajectory torques (steps 5–7)
  const secTraj = section();
  const trajCtl = controlPanel(secTraj, { compact: true });
  const sTf = slider(trajCtl, { label: '軌跡時長 T', min: 0.5, max: 4, step: 0.1, value: Tf, unit: 's', onInput: (v) => { setTf(v); } });
  const sLoad = slider(trajCtl, { label: '末端負載質量', min: 0, max: 3, step: 0.1, value: 0, unit: 'kg', onInput: (v) => { setLoad(v); } });
  const trajRow = row(trajCtl);
  const tGrav = toggle(trajRow, { label: '重力', value: true, onChange: (v) => { gravityOn = v; chips.clear(); recompute(); } });
  const decChips = presetChips(trajRow, [0, 1, 2].map((i) => ({ label: `關節 ${i + 1}`, apply: () => { decJoint = i; paintDecomposition(); } })), { label: '分解：' });
  const ratioRow = el('div', { style: { display: 'flex', flexWrap: 'wrap', gap: '0.1rem 1.2rem', marginBottom: '0.45rem' } });
  secTraj.appendChild(ratioRow);
  const rRatioDyn = readout(ratioRow, { label: '動態部分（慣性＋速度）峰值倍率', minWidth: '5ch' });
  const rRatioG = readout(ratioRow, { label: '重力部分倍率', minWidth: '5ch' });
  const rPeak = readout(ratioRow, { label: '|τ| 峰值', unit: 'N·m', minWidth: '14ch' });
  const totLegend = legend(secTraj, [{ color: 'var(--series-1)', label: 'τ₁' }, { color: 'var(--series-2)', label: 'τ₂' }, { color: 'var(--series-3)', label: 'τ₃' }]);
  const beforeTag = el('span', {}, '虛線＝改變之前（T = 2 s、無負載）');
  totLegend.appendChild(beforeTag);
  const plotTot = new Plot2D(secTraj, {
    title: '各關節力矩 τᵢ', xLabel: 't / T', yLabel: 'N·m', height: 128, xRange: [0, 1], yPad: 0.12, yMinSpan: 1, legend: false,
    series: [
      { name: 'τ₁', color: '--series-1' }, { name: 'τ₂', color: '--series-2' }, { name: 'τ₃', color: '--series-3' },
      { name: '之前 τ₁', color: '--series-1', dash: [4, 4], width: 1.2 }, { name: '之前 τ₂', color: '--series-2', dash: [4, 4], width: 1.2 }, { name: '之前 τ₃', color: '--series-3', dash: [4, 4], width: 1.2 },
    ],
  });
  const decLegend = legend(secTraj, [
    { color: 'var(--fg)', label: '總和 τ' }, { color: 'var(--series-1)', label: '慣性項' }, { color: 'var(--series-5)', label: '速度項' },
    { color: 'var(--series-3)', label: '重力項' }, { color: 'var(--series-4)', label: '負載項' },
  ]);
  decLegend.style.marginTop = '0.35rem';
  const plotDec = new Plot2D(secTraj, {
    title: '關節 1 的組成', xLabel: 't / T', yLabel: 'N·m', height: 128, xRange: [0, 1], yPad: 0.12, yMinSpan: 1, legend: false,
    series: [
      { name: '總和 τ', color: '--fg', width: 2.4 }, { name: '慣性 Mθ″', color: '--series-1' }, { name: '速度 c', color: '--series-5' },
      { name: '重力 g', color: '--series-3' }, { name: '負載 JᵀF', color: '--series-4' },
    ],
  });
  const plots = plotGroup([plotTot, plotDec]);

  // ---------------- section: cost (step 7)
  const secCost = section();
  const costRow = el('div', { style: { display: 'flex', flexWrap: 'wrap', gap: '0.2rem 1.2rem', alignItems: 'center', marginBottom: '0.45rem' } });
  secCost.appendChild(costRow);
  const btnMeasure = button(costRow, { label: '⟳ 重新量測', small: true, onClick: () => startCosts() });
  const rCostModel = readout(costRow, { label: 'rrr3 一次 InverseDynamics ≈', unit: 'µs', minWidth: '5ch' });
  const rCostStatus = el('span', { class: 'mr-note', style: { margin: '0' } });
  costRow.appendChild(rCostStatus);
  const [costColA, costColB] = split(secCost);
  const costPlotId = new Plot2D(costColA, {
    title: 'InverseDynamics：一次遞迴', xLabel: '關節數 n', yLabel: 'µs', height: 150, xRange: [0, 26], yPad: 0.15,
    series: [{ name: '量測', color: '--series-1', points: true }, { name: '擬合 a + b·n', color: '--series-1', dash: [5, 4] }],
  });
  const costPlotM = new Plot2D(costColB, {
    title: 'MassMatrix：n 次遞迴', xLabel: '關節數 n', yLabel: 'µs', height: 150, xRange: [0, 26], yPad: 0.15,
    series: [{ name: '量測', color: '--series-2', points: true }, { name: '擬合 a + b·n²', color: '--series-2', dash: [5, 4] }],
  });
  secCost.appendChild(mdElement(`**下一課預告：正動力學。** 把方程反過來寫成 $\\ddot\\theta = M(\\theta)^{-1}\\big(\\tau - c(\\theta,\\dot\\theta) - g(\\theta)\\big)$，再用 Euler 或 RK4 一步步積分，就能從力矩算出運動；那時 $M(\\theta)$ 正是用本課的遞迴逐欄組出來的。`, 'callout'));

  // ---------------- helpers: single body
  const spinSpec = () => {
    // The tilted body axis (sin α, 0, cos α) is turned upright, so the box wobbles about a vertical shaft.
    const R0 = RotY(-tiltDeg * DEG);
    const q = [offsetR, 0, 0];
    const Rq = [R0[0][0] * q[0], R0[1][0] * q[0], R0[2][0] * q[0]];
    return {
      axis: [Math.sin(tiltDeg * DEG), 0, Math.cos(tiltDeg * DEG)],
      point: q,
      omega0, varying, period: SPIN_PERIOD,
      Ts0: RpToTrans(R0, [-Rq[0], -Rq[1], BODY_Z - Rq[2]]),
    };
  };
  /** Pose of {a} in {b}: at the axis point, aligned with {b}. */
  const Tba = () => RpToTrans([[1, 0, 0], [0, 1, 0], [0, 0, 1]], [offsetR, 0, 0]);

  function updateAxisLine(): void {
    const sp = spinSpec();
    const T0 = sp.Ts0;
    const q = transformPoint(T0, sp.point);
    const s = rot(T0, sp.axis);
    axisLine.update([add3(q, s, -0.32), add3(q, s, 0.32)]);
    pivot.position.set(q[0], q[1], q[2]);
  }

  function updateBody(): void {
    const sp = spinSpec();
    const st = spinAt(sp, tb);
    setMatrix(box, st.Tsb);
    frameB.setTransform(st.Tsb);
    const inA = curStep === 1;
    const TsA = matMul(st.Tsb, Tba());
    frameA.setTransform(TsA);
    const parts = bodyWrenchParts(GB, st.Vb, st.Vdb);
    const Fb = withAd ? parts.inertial.map((v, i) => v + parts.velocity[i]) : parts.inertial;
    const w = wrenchInFrame(GB, Tba(), st.Vb, st.Vdb, withAd);
    const F = inA ? w.Fa : Fb;
    const Tframe = inA ? TsA : st.Tsb;
    const o = origin(Tframe);
    const mS = rot(Tframe, [F[0], F[1], F[2]]);
    const fS = rot(Tframe, [F[3], F[4], F[5]]);
    const wS = rot(st.Tsb, [st.Vb[0], st.Vb[1], st.Vb[2]]);
    const q = transformPoint(sp.Ts0, sp.point);
    const up = rot(sp.Ts0, sp.axis);
    const wTop = add3(q, up, 0.2);
    wArrow.setFromTo(wTop, add3(wTop, wS, 0.04));
    // the moment (a couple) is drawn at the origin of the frame the wrench is written in
    const mLen = Math.min(0.5, 1.4 * norm3(mS));
    mArrow.setDir(o, mS.map((v) => v / (norm3(mS) || 1)), mLen);
    mArrow.visible = mLen > 0.004;
    // the force is drawn at the centre of mass (lifted just above the box so it does not hide
    // behind the x_b axis): it is what keeps the COM on its circle
    const com = add3(origin(st.Tsb), up, 0.06);
    const fLen = Math.min(0.45, 0.03 * norm3(fS));
    fArrow.setDir(com, fS.map((v) => v / (norm3(fS) || 1)), fLen);
    fArrow.visible = fLen > 0.004;
    fPlot.push(tb, F);
    fPlot.setRanges([Math.max(0, tb - 6), Math.max(6, tb)]);
    if (performance.now() - lastSlow > 90 || !loop.running) {
      lastSlow = performance.now();
      vGVd.set(parts.inertial);
      vAdT.set(withAd ? parts.velocity : null);
      vF.set(Fb, true);
      if (inA) {
        mGa.set(w.Ga);
        vFa.set(w.Fa, true);
        vFaB.set(w.FaFromFb);
        rDiffA.set(sci(Math.max(...w.Fa.map((v, i) => Math.abs(v - w.FaFromFb[i]))), 1));
      }
    }
    scene.requestRender();
  }
  let lastSlow = 0;

  // ---------------- helpers: arm
  const trajStart = () => TRAJS[trajIdx].start.map((d) => d * DEG);
  const trajEnd = () => TRAJS[trajIdx].end.map((d) => d * DEG);
  /** Joint state at normalised path position s (0..1) for the current duration; dir = ±1. */
  function jointState(s: number, dir = 1): { th: Vec; dth: Vec; ddth: Vec } {
    const a = trajStart(), b = trajEnd();
    const ts = QuinticTimeScalingDerivs(Tf, s * Tf);
    return {
      th: a.map((q, i) => q + (b[i] - q) * ts.s),
      dth: a.map((q, i) => dir * (b[i] - q) * ts.sdot),
      ddth: a.map((q, i) => (b[i] - q) * ts.sddot),
    };
  }
  /** Ping-pong playback: forward on [0, T], backward on [T, 2T]. */
  const pathPos = (t: number): { s: number; dir: number } => {
    const u = ((t % (2 * Tf)) + 2 * Tf) % (2 * Tf);
    return u <= Tf ? { s: u / Tf, dir: 1 } : { s: 2 - u / Tf, dir: -1 };
  };

  function setArm(th: Vec): Mat[] {
    const poses = linkFramePoses(th, MODEL.Mlist, MODEL.Slist);
    const shoulder = [0, 0, L0];
    const elbow = transformPoint(poses[1], [L1 / 2, 0, 0]);
    const tip = origin(poses[3]);
    links[0].setEndpoints([0, 0, 0], shoulder);
    links[1].setEndpoints(shoulder, elbow);
    links[2].setEndpoints(elbow, tip);
    joints[0].position.set(shoulder[0], shoulder[1], shoulder[2]);
    joints[1].position.set(elbow[0], elbow[1], elbow[2]);
    comFrames.forEach((f, i) => f.setTransform(poses[i]));
    const showLoad = mode === 'traj' && loadMass > 0 && gravityOn;
    loadBox.visible = showLoad;
    rope.mesh.visible = showLoad;
    if (showLoad) {
      const s = 0.05 * Math.cbrt(Math.max(loadMass, 0.05));
      loadBox.scale.set(s / 0.06, s / 0.06, s / 0.06);
      loadBox.position.set(tip[0], tip[1], tip[2] - 0.07 - s / 2);
      rope.setEndpoints(tip, [tip[0], tip[1], tip[2] - 0.07]);
    }
    return poses;
  }

  function updateToolPath(): void {
    const pts: Vec[] = [];
    for (let k = 0; k <= 60; k++) {
      const { th } = jointState(k / 60);
      pts.push(origin(linkFramePoses(th, MODEL.Mlist, MODEL.Slist)[3]));
    }
    toolPath.update(pts);
  }

  function recompute(full = false): void {
    if (full) base = baseCurves(MODEL, trajStart(), trajEnd(), N_SAMPLES, G_VEC);
    curves = scaleCurves(base, Tf, gravityOn, loadMass);
    tGrav.set(gravityOn, { silent: true });
    paintCurves();
    refresh();
  }

  function setTf(v: number, silent = false): void {
    Tf = v;
    sTf.set(v, { silent: true });
    pbArm.setDuration(2 * Tf);
    if (!silent) { chips.clear(); recompute(); }
  }
  function setLoad(v: number, silent = false): void {
    loadMass = v;
    sLoad.set(v, { silent: true });
    if (!silent) { chips.clear(); recompute(); }
  }

  function paintCurves(): void {
    const s = base.s;
    for (let i = 0; i < 3; i++) plotTot.setXY(i, s, curves.map((p) => p.total[i]));
    for (let i = 0; i < 3; i++) plotTot.setXY(3 + i, before ? s : [], before ? before.curves.map((p) => p.total[i]) : []);
    beforeTag.style.display = before ? '' : 'none';
    paintDecomposition();
    const peakOf = (cs: TorqueParts[], f: (p: TorqueParts, i: number) => number) =>
      Math.max(...[0, 1, 2].map((i) => Math.max(...cs.map((p) => Math.abs(f(p, i))))));
    rPeak.set(`(${[0, 1, 2].map((i) => fmt(Math.max(...curves.map((p) => Math.abs(p.total[i]))), 1)).join(', ')})`);
    if (before) {
      const dyn = (p: TorqueParts, i: number) => p.inertial[i] + p.velocity[i];
      const dA = peakOf(curves, dyn), dB = peakOf(before.curves, dyn);
      const gA = peakOf(curves, (p, i) => p.gravity[i]), gB = peakOf(before.curves, (p, i) => p.gravity[i]);
      rRatioDyn.set(dB > 1e-9 ? `×${fmt(dA / dB, 2)}` : '–');
      rRatioG.set(gB > 1e-9 ? `×${fmt(gA / gB, 2)}` : (gA > 1e-9 ? '新增' : '–'));
    } else { rRatioDyn.set('–'); rRatioG.set('–'); }
  }

  function paintDecomposition(): void {
    const s = base.s, i = decJoint;
    decChips.highlight(i);
    plotDec.setOptions({ title: `關節 ${i + 1} 的組成（τ = 慣性 + 速度 + 重力 + 負載）` });
    plotDec.setXY(0, s, curves.map((p) => p.total[i]));
    plotDec.setXY(1, s, curves.map((p) => p.inertial[i]));
    plotDec.setXY(2, s, curves.map((p) => p.velocity[i]));
    const showG = gravityOn, showL = loadMass > 0 && gravityOn;
    (decLegend.children[3] as HTMLElement).style.display = showG ? '' : 'none';
    (decLegend.children[4] as HTMLElement).style.display = showL ? '' : 'none';
    plotDec.setXY(3, showG ? s : [], showG ? curves.map((p) => p.gravity[i]) : []);
    plotDec.setXY(4, showL ? s : [], showL ? curves.map((p) => p.load[i]) : []);
  }

  function updateArmPlayback(): void {
    const { s, dir } = pathPos(ta);
    const st = jointState(s, dir);
    setArm(st.th);
    plots.setCursor(s);
    pbArm.setTime(ta);
    scene.requestRender();
  }

  // ---------------- recursion
  function freeze(): void {
    const st = jointState(S_FREEZE);
    const Ftip = [0, 0, 0, 0, 0, 0];
    frozen = { ...st, Ftip };
    trace = rneaTrace(st.th, st.dth, st.ddth, G_VEC, Ftip, MODEL);
    setArm(st.th);
    stateLine.replaceChildren(
      `t/T = ${fmt(S_FREEZE, 2)}（T = ${fmt(Tf, 1)} s）： θ = (${st.th.map((v) => fmt(v / DEG, 0) + '°').join(', ')})，`,
      mathEl('\\dot\\theta'), ` = (${st.dth.map((v) => fmt(v, 2)).join(', ')}) rad/s，`,
      mathEl('\\ddot\\theta'), ` = (${st.ddth.map((v) => fmt(v, 2)).join(', ')}) rad/s²`,
    );
  }

  const fwdTex = (i: number) => `\\begin{aligned}\\mathcal V_${i} &= [\\mathrm{Ad}_{T_{${i},${i - 1}}}]\\,\\mathcal V_${i - 1} + \\mathcal A_${i}\\dot\\theta_${i}\\\\ \\dot{\\mathcal V}_${i} &= [\\mathrm{Ad}_{T_{${i},${i - 1}}}]\\,\\dot{\\mathcal V}_${i - 1} + [\\mathrm{ad}_{\\mathcal V_${i}}]\\mathcal A_${i}\\dot\\theta_${i} + \\mathcal A_${i}\\ddot\\theta_${i}\\end{aligned}`;
  const bwdTex = (i: number) => `\\begin{aligned}\\mathcal F_${i} &= [\\mathrm{Ad}_{T_{${i + 1},${i}}}]^{\\mathsf T}\\mathcal F_${i + 1} + \\mathcal G_${i}\\dot{\\mathcal V}_${i} - [\\mathrm{ad}_{\\mathcal V_${i}}]^{\\mathsf T}\\mathcal G_${i}\\mathcal V_${i}${i === 3 ? ',\\quad \\mathcal F_4 = \\mathcal F_{\\text{tip}} = 0' : ''}\\\\ \\tau_${i} &= \\mathcal F_${i}^{\\mathsf T}\\mathcal A_${i}\\end{aligned}`;

  function setStage(k: number): void {
    stage = Math.max(0, Math.min(7, k));
    if (!trace || !frozen) freeze();
    const tr = trace!;
    pills.forEach((p, j) => {
      const done = stage > j + 1, cur = stage === j + 1;
      p.style.background = cur ? 'var(--accent)' : done ? 'var(--accent-soft)' : '';
      p.style.color = cur ? 'var(--accent-fg)' : '';
    });
    stageLabel.textContent = `階段 ${stage} / 7`;
    btnPrev.disabled = stage === 0;
    btnNext.disabled = stage === 7;
    // which link is active: forward 1..3, backward 3..1
    const link = stage >= 1 && stage <= 3 ? stage : stage >= 4 && stage <= 6 ? 7 - stage : 0;
    const forward = stage >= 1 && stage <= 3;
    const backward = stage >= 4 && stage <= 6;
    let tex: string;
    if (stage === 0) tex = '\\mathcal V_0 = 0,\\qquad \\dot{\\mathcal V}_0 = (0,\\,-\\mathfrak g) = (0,0,0,\\ 0,0,9.81)';
    else if (forward) tex = fwdTex(link);
    else if (backward) tex = bwdTex(link);
    else tex = '\\tau = (\\tau_1,\\tau_2,\\tau_3)\\ \\text{完成：每根連桿各處理兩次}';
    formula.replaceChildren(mathEl(tex, true));
    // vectors of the active link (or V_0 at stage 0, link 1 after completion)
    const li = stage === 0 ? 0 : stage === 7 ? 1 : link;
    const fwdKnown = (i: number) => stage >= i;           // V_i known once stage ≥ i
    const bwdKnown = (i: number) => stage >= 7 - i;       // F_i known once stage ≥ 7 − i
    vV.el.querySelector('.mr-mlabel')?.replaceChildren(texLabel(`\\mathcal V_${li} =`));
    vVd.el.querySelector('.mr-mlabel')?.replaceChildren(texLabel(`\\dot{\\mathcal V}_${li} =`));
    vFi.el.querySelector('.mr-mlabel')?.replaceChildren(texLabel(`\\mathcal F_${Math.max(1, li)} =`));
    vV.set(fwdKnown(li) ? tr.V[li] : null, forward);
    vVd.set(fwdKnown(li) ? tr.Vd[li] : null, forward);
    vFi.set(li >= 1 && bwdKnown(li) ? tr.F[li - 1] : null, backward);
    for (let i = 0; i < 3; i++) rTau[i].set(bwdKnown(i + 1) ? fmt(tr.tau[i], 3) : '?');
    if (stage === 7) {
      const ref = InverseDynamics(frozen!.th, frozen!.dth, frozen!.ddth, G_VEC, frozen!.Ftip, MODEL.Mlist, MODEL.Glist, MODEL.Slist);
      rCheck.set(sci(Math.max(...ref.map((v, i) => Math.abs(v - tr.tau[i]))), 1));
    } else rCheck.set('（完成後比對）');
    // 3D highlight
    links.forEach((l, i) => l.setColor(i + 1 === link ? '--accent' : '--fg-muted'));
    const active = link >= 1;
    jointAxis.setVisible(active);
    vArrow.visible = false; wArrow.visible = false; mArrow.visible = false; fArrow.visible = false;
    if (active) {
      const P = tr.poses[link - 1];
      jointAxis.setScrew(matVec(Adjoint(P), tr.A[link - 1]));
      const o = origin(P);
      if (forward) {
        const V = tr.V[link];
        const w = rot(P, [V[0], V[1], V[2]]), v = rot(P, [V[3], V[4], V[5]]);
        if (norm3(w) > 1e-6) { wArrow.setDir(o, w.map((x) => x / norm3(w)), Math.min(0.35, 0.12 * norm3(w))); wArrow.visible = true; }
        if (norm3(v) > 1e-6) { vArrow.setDir(o, v.map((x) => x / norm3(v)), Math.min(0.35, 0.3 * norm3(v))); vArrow.visible = true; }
      } else if (backward) {
        const F = tr.F[link - 1];
        const m = rot(P, [F[0], F[1], F[2]]), f = rot(P, [F[3], F[4], F[5]]);
        if (norm3(f) > 1e-6) { fArrow.setDir(o, f.map((x) => x / norm3(f)), Math.min(0.42, 0.012 * norm3(f))); fArrow.visible = true; }
        if (norm3(m) > 1e-6) { mArrow.setDir(o, m.map((x) => x / norm3(m)), Math.min(0.42, 0.06 * norm3(m))); mArrow.visible = true; }
      }
    }
    scene.requestRender();
  }

  function toggleAuto(): void {
    if (autoTimer) { stopAuto(); return; }
    if (stage >= 7) setStage(curStep === 3 ? 4 : 0);
    btnAuto.textContent = '⏸ 暫停';
    autoTimer = window.setInterval(() => {
      if (stage >= 7) { stopAuto(); return; }
      setStage(stage + 1);
    }, 1100);
  }
  function stopAuto(): void {
    if (autoTimer) { clearInterval(autoTimer); autoTimer = 0; }
    btnAuto.textContent = '▶ 自動';
  }

  // ---------------- costs (step 7)
  /** Least-squares fit y ≈ a + b·f(n): returns the model and its samples on 0..26 for drawing. */
  const fitCurve = (xs: number[], ys: number[], f: (n: number) => number): { at: (n: number) => number; xs: number[]; ys: number[] } => {
    const u = xs.map(f);
    const m = u.length, su = u.reduce((a, b) => a + b, 0), sy = ys.reduce((a, b) => a + b, 0);
    const suu = u.reduce((a, b) => a + b * b, 0), suy = u.reduce((a, b, k) => a + b * ys[k], 0);
    const den = m * suu - su * su;
    const b = den !== 0 ? (m * suy - su * sy) / den : 0;
    const a = (sy - b * su) / m;
    const at = (n: number) => a + b * f(n);
    const nx = Array.from({ length: 27 }, (_, k) => k);
    return { at, xs: nx, ys: nx.map(at) };
  };
  let fitId: ReturnType<typeof fitCurve> | null = null;
  let fitM: ReturnType<typeof fitCurve> | null = null;

  function startCosts(): void {
    if (costTimer) return;
    costs = [];
    costPlotId.clear(); costPlotM.clear();
    btnMeasure.disabled = true;
    rCostStatus.textContent = '量測中…';
    let k = 0;
    const next = () => {
      costTimer = 0;
      if (k === 0) costModelUs = measureModelCost(MODEL, [0.3, -0.4, 0.8]);
      costs.push(measureCost(COST_NS[k]));
      k++;
      const ns = costs.map((c) => c.n);
      costPlotId.setXY(0, ns, costs.map((c) => c.idUs));
      costPlotM.setXY(0, ns, costs.map((c) => c.massUs));
      if (costs.length >= 2) {
        fitId = fitCurve(ns, costs.map((c) => c.idUs), (n) => n);
        costPlotId.setXY(1, fitId.xs, fitId.ys);
        fitM = fitCurve(ns, costs.map((c) => c.massUs), (n) => n * n);
        costPlotM.setXY(1, fitM.xs, fitM.ys);
      }
      rCostModel.set(costModelUs, 1);
      if (k < COST_NS.length) costTimer = window.setTimeout(next, 30);
      else {
        btnMeasure.disabled = false;
        const n1 = COST_NS[COST_NS.length - 2], n2 = COST_NS[COST_NS.length - 1];
        rCostStatus.textContent = fitId && fitM
          ? `依擬合曲線，n 從 ${n1} 加倍到 ${n2}：遞迴 ×${fmt(fitId.at(n2) / fitId.at(n1), 1)}，組出 M ×${fmt(fitM.at(n2) / fitM.at(n1), 1)}`
          : '';
      }
    };
    costTimer = window.setTimeout(next, 60);
  }

  // ---------------- loop
  const loop = createLoop((dt) => {
    if (mode === 'body') { tb += dt; updateBody(); pbBody.setTime(tb); }
    else if (mode === 'traj') { ta += dt; updateArmPlayback(); }
  });
  function startLoop(): void {
    loop.start();
    pbBody.setPlaying(true, { silent: true }); pbArm.setPlaying(true, { silent: true });
  }
  function stopLoop(): void {
    loop.stop();
    pbBody.setPlaying(false, { silent: true }); pbArm.setPlaying(false, { silent: true });
  }

  function refresh(): void {
    if (mode === 'body') updateBody();
    else if (mode === 'traj') updateArmPlayback();
  }

  // ---------------- modes & steps
  function setView(pos: [number, number, number], target: [number, number, number]): void {
    scene.camera.position.set(pos[0], pos[1], pos[2]);
    scene.controls?.target.set(target[0], target[1], target[2]);
    scene.controls?.update();
    scene.requestRender();
  }

  function setMode(m: 'body' | 'recursion' | 'traj'): void {
    const changed = m !== mode;
    mode = m;
    const body = m === 'body';
    bodyObjs().forEach((o) => { o.visible = body; });
    frameB.setVisible(body);
    frameA.setVisible(body && curStep === 1);
    armObjs().forEach((o) => { o.visible = !body; });
    comFrames.forEach((f) => f.setVisible(m === 'recursion'));
    jointAxis.setVisible(false);
    vArrow.visible = false; wArrow.visible = body; mArrow.visible = body; fArrow.visible = body;
    if (m !== 'recursion') links.forEach((l) => l.setColor('--fg-muted'));
    if (body) { loadBox.visible = false; rope.mesh.visible = false; }
    toolPath.mesh.visible = m === 'traj';
    if (changed) {
      const v = body ? BODY_VIEW : ARM_VIEW;
      setView(v.pos, v.target);
    }
    pbBody.el.style.display = body ? '' : 'none';
    pbArm.el.style.display = m === 'traj' ? '' : 'none';
    stRow.style.display = m === 'recursion' ? '' : 'none';
  }

  const BODY_PRESETS = [
    { label: '繞傾斜軸等速轉', apply: () => applyBody({ omega0: 4, tilt: 30, varying: false }) },
    { label: '繞主軸等速轉', apply: () => applyBody({ omega0: 4, tilt: 0, varying: false }) },
    { label: '變速旋轉', apply: () => applyBody({ omega0: 4, tilt: 30, varying: true }) },
  ];
  const FRAME_PRESETS = [
    { label: '轉軸移到桿端（r = 0.18 m）', apply: () => applyBody({ omega0: 3, tilt: 0, varying: true, r: 0.18 }) },
    { label: '轉軸穿過質心（r = 0）', apply: () => applyBody({ omega0: 3, tilt: 0, varying: true, r: 0 }) },
    { label: '桿端 + 傾斜軸', apply: () => applyBody({ omega0: 3, tilt: 35, varying: false, r: 0.15 }) },
  ];
  const trajPresets = () => TRAJS.map((p, i) => ({ label: p.label, title: p.title, apply: () => applyTraj(i) }));
  const COMPARE_PRESETS = [
    { label: '基準（T = 2 s、無負載）', apply: () => applyCompare(2, 0) },
    { label: '時長減半（T = 1 s）', apply: () => applyCompare(1, 0) },
    { label: '末端加 1 kg', apply: () => applyCompare(2, 1) },
    { label: '兩者都要', apply: () => applyCompare(1, 1) },
  ];

  function applyBody(p: { omega0: number; tilt: number; varying: boolean; r?: number }): void {
    omega0 = p.omega0; tiltDeg = p.tilt; varying = p.varying; withAd = true;
    if (p.r != null) offsetR = p.r;
    sOmega.set(omega0, { silent: true }); sTilt.set(tiltDeg * DEG, { silent: true }); sOffset.set(offsetR, { silent: true });
    tVary.set(varying, { silent: true }); tAd.set(true, { silent: true });
    tb = 0;
    fPlot.clear();
    updateAxisLine();
    refresh();
    startLoop();
  }

  function applyTraj(i: number): void {
    trajIdx = i;
    before = null;
    recompute(true);
    updateToolPath();
    if (mode === 'recursion') { trace = null; frozen = null; freeze(); setStage(stage); }
  }

  function applyCompare(T: number, load: number): void {
    // the baseline is always the 2 s / no-load motion of the current path
    const baseline = scaleCurves(base, 2, gravityOn, 0);
    before = T === 2 && load === 0 ? null : { curves: baseline, label: '基準' };
    setTf(T, true);
    setLoad(load, true);
    curves = scaleCurves(base, Tf, gravityOn, loadMass);
    paintCurves();
    ta = 0;
    refresh();
    startLoop();
  }

  let focused: HTMLElement[] = [];
  function onStep(i: number): void {
    const prev = curStep;
    curStep = i;
    stopAuto();
    stopLoop();
    secBody.style.display = i <= 1 ? '' : 'none';
    secRec.style.display = i === 2 || i === 3 ? '' : 'none';
    secTraj.style.display = i === 4 || i === 5 ? '' : 'none';
    secCost.style.display = i === 6 ? '' : 'none';
    sOffset.el.style.display = i === 1 ? '' : 'none';
    frameBox.style.display = i === 1 ? '' : 'none';
    vecBox.style.display = i === 0 ? '' : 'none';
    sLoad.el.style.display = i === 5 ? '' : 'none';
    ratioRow.style.display = i === 5 ? '' : 'none';
    focused.forEach((f) => f.classList.remove('mr-focus'));
    if (i <= 1) {
      setMode('body');
      fPlot.setOptions({ title: i === 0 ? '所需 wrench 的分量（在 {b} 中，力矩以實線、力以虛線）' : '所需 wrench 的分量（在 {a} 中，力矩以實線、力以虛線）' });
      bodyNote.textContent = i === 0
        ? '盒子 0.40 × 0.20 × 0.06 m、3 kg；轉軸穿過質心並固定在空間中。橘色＝力矩、紅色＝力、紫色＝角速度。'
        : '{a} 位於轉軸上、與 {b} 平行；G_a 中標色的是力與角速度之間的耦合區塊。紅色力箭頭畫在質心（讓質心繞圈所需的力），橘色力矩對 {a} 的原點計算。';
      chips.setPresets(i === 0 ? BODY_PRESETS : FRAME_PRESETS);
      if (i === 0) { offsetR = 0; }
      chips.activate(0);
      focused = i === 0 ? [scene.el] : [frameBox];
    } else if (i === 2 || i === 3) {
      setMode('recursion');
      chips.setPresets(trajPresets());
      chips.highlight(trajIdx);
      updateToolPath();
      trace = null; frozen = null;
      freeze();
      setStage(i === 2 ? 0 : 4);
      focused = [secRec];
    } else if (i === 4 || i === 5) {
      setMode('traj');
      if (i === 4) {
        chips.setPresets(trajPresets());
        chips.highlight(trajIdx);
        before = null;
        if (prev !== 5) { setLoad(0, true); if (Tf !== 2) setTf(2, true); }
        recompute(false);
        updateToolPath();
        ta = 0;
        startLoop();
        focused = [plotDec.el];
      } else {
        chips.setPresets(COMPARE_PRESETS);
        updateToolPath();
        chips.activate(1);
        focused = [ratioRow, plotTot.el];
      }
    } else {
      setMode('traj');
      chips.setPresets(trajPresets());
      chips.highlight(trajIdx);
      recompute(false);
      updateToolPath();
      startLoop();
      if (!costs.length) startCosts();
      focused = [costColA.parentElement ?? costColA];
    }
    focused.forEach((f) => f.classList.add('mr-focus'));
  }

  updateAxisLine();
  updateToolPath();
  paintCurves();
  onStep(ctx.currentStep);

  return {
    onStep,
    destroy() {
      stopAuto();
      loop.stop();
      if (costTimer) clearTimeout(costTimer);
      fPlot.destroy(); plotTot.destroy(); plotDec.destroy(); costPlotId.destroy(); costPlotM.destroy();
      scene.destroy();
    },
  };
}

const lesson: Lesson = { meta, steps, quiz, mount };
export default lesson;
