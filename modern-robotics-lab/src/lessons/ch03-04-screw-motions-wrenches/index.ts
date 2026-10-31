import * as THREE from 'three';
import type { Lesson, LessonContext, LessonHandle, LessonStep, QuizQuestion } from '@/types';
import {
  el, fmt, fmtVec, controlPanel, row, slider, angleSlider, button, select, toggle, simTitle, kv, legend, note,
  presetChips, createLoop, jointSliders, Scene3D, Arrow3D, Frame3D, ScrewAxis3D, Robot3D, setMatrix, setThemedColor,
  themedStandardMaterial, type SliderHandle,
} from '@/ui';
import {
  type Mat, type Vec, Adjoint, FKinSpace, RotZ, RpToTrans, TransInv, clamp, cross, eye, getCol, matMul, matVec, norm, rotZYX, rrr3,
  transformPoint, transpose, vecAdd, vecScale, vecSub,
} from '@/mr';
import { meta } from './meta';
import {
  type InterpMethod, type ScrewInfo, dirFromAngles, interpolatePose, interpolationPath, power, prismaticScrew, rotationBlockDet,
  screwBetween, screwFromQSH, screwMotion, screwPath, translationTerms, vParts, wrenchAtPoint, wrenchInFrame,
} from './screws';
import { attachDragHandles, rayLineParam, rayPlaneZ } from './drag3d';
import { captioned, cellMatrix, disposeSprites, eqRow, label3D, setLinePoints, tex } from './widgets';

// ---------------------------------------------------------------------------
// Narrative
// ---------------------------------------------------------------------------

const steps: LessonStep[] = [
  {
    title: 'Chasles：任何位移都是一段螺旋',
    md: `任意挑選剛體的兩個位姿，空間中總有一條直線，讓物體只要「繞這條線轉 $\\theta$、同時沿著它移動 $d$」就能從第一個位姿到達第二個（Chasles–Mozzi 定理）。這條線稱為**螺旋軸**，比值 $h=d/\\theta$ 稱為**節距**。

$$T_{\\text{終}}=e^{[\\vS]\\theta}\\,T_{\\text{起}}$$

轉角 $\\theta$ 就是兩個姿態之間的旋轉角；只有兩個姿態完全相同時，這段運動才退化成純平移。換句話說，「先轉再移」的各種組合，其實都能用同一種一步到位的螺旋運動取代。`,
    simHint: '拖動「沿螺旋前進的比例 τ」或按 ▶：物體從「起點」繞青色螺旋軸邊轉邊移到「終點」。兩條橘色軌跡（{b} 原點與一個角落）都是繞同一條軸的螺旋線。換個終點預設再試試。',
  },
  {
    title: '螺旋軸 S = (ω, v)',
    md: `一條螺旋軸由軸上一點 $q$、單位方向 $\\hat s$ 與節距 $h$ 決定，寫成六維向量：

$$\\vS=\\begin{bmatrix}\\omega\\\\ v\\end{bmatrix}=\\begin{bmatrix}\\hat s\\\\ -\\hat s\\times q+h\\,\\hat s\\end{bmatrix}$$

它正是「以 1 rad/s 沿這條軸做螺旋運動」時的 twist：$-\\hat s\\times q$ 是轉動帶給 $\\{s\\}$ 原點那一點的速度，$h\\hat s$ 是沿軸滑動的速度。純平移沒有轉軸，此時 $\\omega=0$、$v$ 是單位方向（$\\|v\\|=1$），$\\theta$ 代表移動的距離（m）。`,
    simHint: '拖曳軸上的大青色圓球 q 移動軸的位置、拖曳小圓球 ŝ 改方向（也可用滑桿），用 h 改節距：{s} 原點上的兩支箭頭就是 −ŝ×q 與 hŝ。勾選「純平移（ω = 0）」看 S 變成 (0, v̂)。',
  },
  {
    title: 'exp([S]θ)：沿螺旋軸走 θ',
    md: `沿螺旋軸走 $\\theta$，就是對 $[\\vS]\\theta$ 取矩陣指數（$\\|\\omega\\|=1$）：

$$e^{[\\vS]\\theta}=\\begin{bmatrix}e^{[\\omega]\\theta} & G(\\theta)\\,v\\\\ 0 & 1\\end{bmatrix}$$

$$G(\\theta)=I\\theta+(1-\\cos\\theta)[\\omega]+(\\theta-\\sin\\theta)[\\omega]^2$$

旋轉部分就是上一課的 Rodrigues 公式；平移部分 $G(\\theta)v$ 是原本位在 $\\{s\\}$ 原點的那一點最後的位置。物體上每一點都繞著軸走螺旋線：$h=0$ 時是圓弧，$h$ 越大螺旋拉得越開、越接近直線。純平移（$\\omega=0$）時 $G(\\theta)=I\\theta$，物體只是沿 $v$ 移動 $\\theta$ 公尺。`,
    simHint: '按「▶ 播放 θ」讓 θ 一路轉到 360°。從 {s} 原點出發、頭尾相接的三支箭頭是 G(θ)v 的三項（紫、粉紅、橘），終點正好落在 {b} 的原點。把 h 調大，看灰色螺旋線被拉長。',
  },
  {
    title: '反向：log T',
    md: `反過來，給定任意 $T$，\`MatrixLog6\` 先由 $R$ 求出 $\\omega$ 與 $\\theta\\in[0,\\pi]$，再由 $v=G^{-1}(\\theta)\\,p$ 求出 $v$；若 $R=I$，改取 $\\omega=0$、$v=p/\\|p\\|$、$\\theta=\\|p\\|$。

$$\\log T=[\\vS]\\theta\\quad\\Rightarrow\\quad T(\\tau)=e^{[\\vS]\\theta\\tau},\\ \\ \\tau\\in[0,1]$$

這樣內插，整個物體繞同一條固定軸移動，而且 $\\{b\\}$ 的原點不論放在物體的哪裡，動畫都一樣。把 $R$ 與 $p$ 分開內插，只有原點走直線，其他點的路徑取決於原點選在哪；逐元素內插 $R$ 更糟，中間的矩陣根本不是旋轉。`,
    simHint: '拖曳「目標 T」的圓球或用滑桿改變目標位姿，面板即時顯示 log T 的 S 與 θ。把「比較」切到另一種內插再拖動 τ：半透明物體走另一條路；逐元素內插時 det R < 1，物體被壓扁。',
  },
  {
    title: '力旋量（wrench）',
    md: `力 $f$ 作用在物體上的點 $r$（都用 $\\{a\\}$ 的座標），對 $\\{a\\}$ 的原點產生力矩 $m_a=r_a\\times f_a$。把力矩與力疊成六維向量就是**力旋量（wrench）**，和 twist 一樣「轉動的部分在前」：

$$\\vF_a=\\begin{bmatrix}m_a\\\\ f_a\\end{bmatrix},\\qquad m_a=r_a\\times f_a$$

同一個力換一個參考原點，$f$ 只是換了座標，$m$ 卻因為力臂不同而改變。沿著力的作用線移動施力點，$m$ 不變；往側邊移，$m$ 就跟著變。`,
    simHint: '沿著桿子拖曳橘色施力點（或用 d 滑桿），再改變力的分量：紫色的 m_s、m_b 隨施力點改變，f 不變。把施力點拉到 {b} 原點（d = 0），m_b 變成 0，m_s 卻不是。',
  },
  {
    title: '用 Adᵀ 換座標，功率不變',
    md: `物體以 twist $\\vV$ 運動時，wrench 輸入的功率是 $P=\\vV^{\\T}\\vF=\\omega\\cdot m+v\\cdot f$。功率是物理量，不能因為換座標而改變。把 $\\vV_s=[\\Ad_{T_{sb}}]\\vV_b$ 代入 $\\vV_b^{\\T}\\vF_b=\\vV_s^{\\T}\\vF_s$，就得到 wrench 的換法：

$$\\vF_b=[\\Ad_{T_{sb}}]^{\\T}\\vF_s,\\qquad \\vF_a=[\\Ad_{T_{ba}}]^{\\T}\\vF_b$$

注意下標的順序：要得到 $\\vF_b$，用的是「把 twist 從 $\\{b\\}$ 換到 $\\{s\\}$」那個 Adjoint 的轉置。直覺上，參考點平移 $p$ 時（姿態相同），$m$ 多出 $p\\times f$，$v$ 也多出 $p\\times\\omega$；這兩項對功率的貢獻 $\\omega\\cdot(p\\times f)+f\\cdot(p\\times\\omega)$ 恰好互相抵消。`,
    simHint: '用 V_b 滑桿給桿子一個 twist：F_s 與 F_b 的數字完全不同，但 V_sᵀF_s、V_bᵀF_b 以及「力 · 施力點速度」三種算法得到同一個功率。',
  },
  {
    title: '下一章：把關節軸當成螺旋軸',
    md: `機器人的每個旋轉關節都是一條節距 $h=0$ 的螺旋軸（移動關節則是 $\\omega=0$ 的純平移）。把關節 $i$ 轉 $\\theta_i$ 就是乘上 $e^{[\\vS_i]\\theta_i}$；依序相乘後，再乘上零位時的末端位姿 $M$：

$$T(\\theta)=e^{[\\vS_1]\\theta_1}e^{[\\vS_2]\\theta_2}e^{[\\vS_3]\\theta_3}M$$

這就是第 4 章的**指數積（PoE）正向運動學**。完成下方的自我檢測後，就可以進入第 4 章。`,
    simHint: '拉動三個關節角或按 ▶：三條青色螺旋軸隨手臂移動，末端座標系 {b} 的位姿就是三個指數相乘再乘上 M。',
  },
];

const quiz: QuizQuestion[] = [
  {
    question: '門鉸鏈的轉軸沿 $\\hat z$、通過點 $q=(1,0,0)$。它的螺旋軸 $\\vS$ 是什麼？若換成每轉一圈沿 $+\\hat z$ 前進 2 mm 的螺栓，哪裡會改變？',
    options: [
      '$\\vS=(0,0,1,\\ 1,0,0)$；螺栓會讓 $\\omega$ 變長',
      '$\\vS=(0,0,1,\\ 0,1,0)$；螺栓會移動軸上的點 $q$',
      '$\\vS=(0,0,1,\\ 0,-1,0)$；螺栓只讓 $v$ 多出 $h\\hat s$，$h=\\tfrac{2\\ \\text{mm}}{2\\pi}\\approx 0.32$ mm/rad',
      '$\\vS=(0,0,0,\\ 0,0,1)$；鉸鏈其實是沿 $\\hat z$ 的平移',
    ],
    answer: 2,
    explanation: '$\\omega=\\hat s=(0,0,1)$，$v=-\\hat s\\times q=-(0,0,1)\\times(1,0,0)=(0,-1,0)$：這正是繞這條軸以 1 rad/s 轉動時 $\\{s\\}$ 原點那一點的速度。螺栓的位置與方向都沒變，只多了沿軸滑動：$h=0.002/(2\\pi)\\approx 3.2\\times10^{-4}$ m/rad，$v$ 多出 $(0,0,h)$。',
  },
  {
    question: '用扳手在 $\\{b\\}$ 的原點施加一個純力 $f_b$（所以 $m_b=0$），$\\{s\\}$ 的原點在 1 m 外，$T_{sb}=(R,p)$。$\\vF_s$ 的力矩分量 $m_s$ 是什麼？',
    options: [
      '$m_s=0$：施力點沒有力矩，換座標也不會有',
      '$m_s=p\\times f_s$（$f_s=Rf_b$）：作用線不通過 $\\{s\\}$ 原點就有力矩',
      '$m_s=R\\,m_b=0$',
      '$m_s=f_s\\times p$',
    ],
    answer: 1,
    explanation: '由 $\\vF_s=[\\Ad_{T_{bs}}]^{\\T}\\vF_b$ 得 $m_s=R\\,m_b+p\\times(Rf_b)=p\\times f_s$。力矩永遠是「對某一點」而言：$\\{b\\}$ 的原點在作用線上，所以 $m_b=0$；$\\{s\\}$ 的原點一般不在作用線上，力臂是它到作用線的垂直距離（最多 1 m，只有 $f$ 平行於 $p$ 時才是 0）。$f_s\\times p$ 的叉積順序相反，方向錯了。',
  },
  {
    question: '已知起點 $T_1$ 與終點 $T_2$（都相對於 $\\{s\\}$），想用一段螺旋運動把它們連起來做動畫。哪一種做法正確？',
    options: [
      '$[\\vS]\\theta=\\log(T_1^{-1}T_2)$，播放 $T(\\tau)=e^{[\\vS]\\theta\\tau}\\,T_1$',
      '$[\\vS]\\theta=\\log T_2-\\log T_1$，播放 $T(\\tau)=e^{[\\vS]\\theta\\tau}$',
      '對 $R$ 與 $p$ 的每個元素做線性內插，結果相同',
      '$[\\vS]\\theta=\\log(T_2T_1^{-1})$，播放 $T(\\tau)=e^{[\\vS]\\theta\\tau}\\,T_1$',
    ],
    answer: 3,
    explanation: '$T_2T_1^{-1}$ 是用 $\\{s\\}$ 座標寫的位移，它的對數是 $\\{s\\}$ 中的螺旋軸，所以要**前乘**：$T(\\tau)=e^{[\\vS]\\theta\\tau}T_1$。$\\log(T_1^{-1}T_2)$ 給的是用 $\\{b\\}$ 寫的螺旋軸，必須後乘 $T_1e^{[\\vS_b]\\theta\\tau}$ 才對（結果與前者相同）。矩陣對數不能像純量那樣相減，逐元素內插則會離開 $\\SO(3)$。',
  },
  {
    question: '物體以 $\\vV_s=(0,0,2,\\ 0,0,0)$ 繞 $\\hat z_s$ 軸轉動。作用在它上面的 wrench 是 $\\vF_s=(0,0,0,\\ 0,5,0)$（沿 $\\hat y_s$ 的 5 N 力，作用線通過 $\\{s\\}$ 原點）。這個 wrench 輸入的功率是多少？',
    options: ['0 W', '10 W', '5 W', '無法判斷，還需要知道施力點在哪裡'],
    answer: 0,
    explanation: '$P=\\vV_s^{\\T}\\vF_s=\\omega\\cdot m+v\\cdot f=(0,0,2)\\cdot 0+0\\cdot(0,5,0)=0$。作用線（$\\hat y_s$ 軸）穿過轉軸，線上每一點的速度 $\\omega\\times r$ 都沿 $\\hat x_s$，與力垂直，所以不做功。作用線的位置已經編碼在 $m$ 裡，不必另外知道施力點。',
  },
];

// ---------------------------------------------------------------------------
// Simulation constants
// ---------------------------------------------------------------------------

const DEG = Math.PI / 180;
const ORIGIN: Vec = [0, 0, 0];
interface Pose6 { x: number; y: number; z: number; yaw: number; pitch: number; roll: number }
const p6 = (x: number, y: number, z: number, yawDeg: number, pitchDeg: number, rollDeg: number): Pose6 =>
  ({ x, y, z, yaw: yawDeg * DEG, pitch: pitchDeg * DEG, roll: rollDeg * DEG });
const toT = (P: Pose6): Mat => RpToTrans(rotZYX([P.yaw, P.pitch, P.roll]), [P.x, P.y, P.z]);

const BOX: [number, number, number] = [0.4, 0.24, 0.14];
/** A body corner whose path is drawn next to the path of the {b} origin. */
const CORNER: Vec = [0.2, 0.12, 0.07];

// Step 1: start pose and end-pose presets (computed with @/mr so the special cases are exact)
const T1_CH = toT(p6(0.55, -0.1, 0.3, -20, 0, 0));
const CHASLES: { label: string; title: string; T2: Mat }[] = [
  { label: '一般位移', title: '轉動與平移都有', T2: toT(p6(1.25, 0.75, 0.55, 80, 0, 35)) },
  { label: '純轉動', title: '繞一條鉛直軸轉 120°（h = 0）', T2: screwMotion(screwFromQSH([1.15, 0.2, 0], [0, 0, 1], 0), 2.1, T1_CH) },
  { label: '純平移', title: '姿態不變，只有位置改變', T2: matMul(RpToTrans(eye(3), [0.7, 0.6, 0.2]), T1_CH) },
  { label: '翻轉 180°', title: 'θ = π：軸的方向取正或負都可以', T2: screwMotion(screwFromQSH([0.95, 0.3, 0.45], [1, 0, 0], 0.06), Math.PI, T1_CH) },
];

// Step 4: target-pose presets for log T
const LOG_PRESETS: { label: string; title: string; P: Pose6 }[] = [
  { label: '一般', title: '轉動與平移都有', P: p6(0.7, 0.35, 0.45, 100, 0, 30) },
  { label: '純平移', title: 'R = I：ω = 0，θ 是距離', P: p6(0.7, 0.35, 0.45, 0, 0, 0) },
  { label: '轉 180°', title: 'θ = π 的特殊情況', P: p6(0.5, 0.5, 0.3, -180, 0, 0) },
  { label: '小角度', title: '轉角很小：螺旋軸離得很遠', P: p6(0.6, 0.2, 0.2, 8, 0, 0) },
];

// Steps 5–6: a bar of length 1 m along x_b, {b} at its left end
const BAR_LEN = 1;
const T_BAR = RpToTrans(RotZ(40 * DEG), [0.25, 0.45, 0.45]);
const F_SCALE = 0.05; // m of arrow per N
const M_SCALE = 0.085; // m of arrow per N·m
const V_SCALE = 0.6; // m of arrow per m/s

const POE_HOME: Vec = [0.6, -0.5, 0.9];
const TH_MAX_ROT = 2 * Math.PI;
const TH_MAX_LIN = 1.2;
/** Distance of the ŝ handle from q along the axis (steps 2–3). */
const DIR_R = 0.6;

function camAt(target: Vec, d: number, azDeg = -50, elDeg = 27): { pos: Vec; target: Vec } {
  const az = azDeg * DEG, elv = elDeg * DEG;
  return { pos: [target[0] + d * Math.cos(elv) * Math.cos(az), target[1] + d * Math.cos(elv) * Math.sin(az), target[2] + d * Math.sin(elv)], target };
}
const CAMS = [
  camAt([0.75, 0.3, 0.4], 2.5),
  camAt([0.5, 0.1, 0.3], 2.4),
  camAt([0.5, -0.05, 0.25], 2.4),
  camAt([0.35, 0.22, 0.22], 1.8, -75, 58), // from above: the screw arc and the straight decoupled path separate
  camAt([0.55, 0.5, 0.35], 2.4),
  camAt([0.55, 0.5, 0.35], 2.4),
  camAt([0.25, 0.0, 0.3], 1.8),
];

type Mode = 'chasles' | 'axis' | 'log' | 'wrench' | 'poe';
const MODE_OF: Mode[] = ['chasles', 'axis', 'axis', 'log', 'wrench', 'wrench', 'poe'];
type MomentView = 'both' | 's' | 'b';
type Cmp = 'none' | InterpMethod;

const COL_CLS = ['c304-x', 'c304-y', 'c304-z', 'c304-p'];
const tCls = (i: number, j: number): string => (i === 3 ? 'c304-dim' : COL_CLS[j] ?? '');
const screwCls = (_i: number, j: number): string => (j < 3 ? 'c304-w' : 'c304-v');
const wrenchCls = (_i: number, j: number): string => (j < 3 ? 'c304-m' : 'c304-f');

const CSS = `
.c304 .c304-eq { display: flex; flex-wrap: wrap; align-items: center; gap: 0.25rem 0.5rem; margin: 0.35rem 0 0.15rem; font-size: 0.85rem; }
.c304 .c304-eq .mr-matrix { margin: 0; }
.c304 .c304-op { color: var(--fg-muted); font-size: 0.95rem; }
.c304 .c304-cap { display: inline-flex; flex-direction: column; align-items: center; gap: 0.05rem; }
.c304 .c304-capt { font-size: 0.76rem; color: var(--fg-muted); line-height: 1.25; white-space: nowrap; }
.c304 .c304-cap .mr-mlabel:empty { display: none; }
.c304 .c304-pill { position: absolute; left: 0.5rem; top: 0.4rem; font-size: 0.75rem; padding: 0.1rem 0.55rem; border-radius: 999px; pointer-events: none; max-width: calc(100% - 1rem); z-index: 1; }
.c304 .c304-pill:empty { display: none; }
.c304 .c304-pill.ok { background: var(--ok-soft); color: var(--ok); }
.c304 .c304-pill.bad { background: var(--danger-soft); color: var(--danger); }
.c304 .c304-pill.info { background: var(--accent-soft); color: var(--accent); }
.c304 .c304-hint { position: absolute; right: 0.5rem; bottom: 0.3rem; font-size: 0.72rem; color: var(--fg-muted); pointer-events: none; text-align: right; }
.c304 .mr-cell.c304-x { color: var(--axis-x); font-weight: 700; }
.c304 .mr-cell.c304-y { color: var(--axis-y); font-weight: 700; }
.c304 .mr-cell.c304-z { color: var(--axis-z); font-weight: 700; }
.c304 .mr-cell.c304-p { color: var(--series-8); font-weight: 700; }
.c304 .mr-cell.c304-dim { color: var(--fg-muted); }
.c304 .mr-cell.c304-w { color: var(--series-5); font-weight: 700; }
.c304 .mr-cell.c304-v { color: var(--series-6); font-weight: 700; }
.c304 .mr-cell.c304-m { color: var(--series-5); font-weight: 700; }
.c304 .mr-cell.c304-f { color: var(--series-4); font-weight: 700; }
.c304 .mr-cell.c304-t1 { color: var(--series-5); font-weight: 700; }
.c304 .mr-cell.c304-t2 { color: var(--series-7); font-weight: 700; }
.c304 .mr-cell.c304-t3 { color: var(--series-4); font-weight: 700; }
.c304 .mr-controls { padding-top: 0.35rem; }
.c304 .mr-legend { margin-top: 0.35rem; }
.c304 .mr-kv { margin: 0.3rem 0 0.1rem; }
.c304 .mr-note { margin: 0.25rem 0 0; }
`;

// ---------------------------------------------------------------------------
// mount
// ---------------------------------------------------------------------------

function mount(host: HTMLElement, ctx: LessonContext): LessonHandle {
  const S = {
    sc: -1,
    // step 1
    chasles: 0,
    tau: 0.55,
    // steps 2–3
    q: [0.6, 0.2, 0] as Vec,
    az: 0,
    el: 90 * DEG,
    h: 0.1,
    prismatic: false,
    theta: 1.6,
    trails: true,
    // step 4
    target: { ...LOG_PRESETS[0].P } as Pose6,
    cmp: 'none' as Cmp,
    tauLog: 0.5,
    // steps 5–6
    d: 0.75,
    fb: [0, 3, -7] as Vec,
    Vb: [0, 0, 0.8, 0, 0, 0] as Vec,
    mview: 'both' as MomentView,
    // step 7
    q3: POE_HOME.slice(),
    poeT: 0,
  };
  type Anim = 'tau' | 'theta' | 'tauLog' | 'poe';
  let anim: Anim | null = null;
  let pathKey = '';

  const root = el('div', { class: 'c304' });
  root.appendChild(el('style', {}, CSS));
  host.appendChild(root);
  simTitle(root, '螺旋運動與力旋量', '3D');

  // ---- 3D scene ----------------------------------------------------------
  const narrow = host.clientWidth > 0 && host.clientWidth < 520;
  const c0 = CAMS[0];
  const scene = new Scene3D(root, { aspect: narrow ? 1.2 : 1.7, grid: 3, axes: false, cameraPos: [c0.pos[0], c0.pos[1], c0.pos[2]], target: [c0.target[0], c0.target[1], c0.target[2]], autoStart: false });
  const pill = el('div', { class: 'c304-pill info' });
  const hint3d = el('div', { class: 'c304-hint' });
  scene.el.append(pill, hint3d);
  const add = <T extends THREE.Object3D>(o: T): T => { scene.scene.add(o); return o; };

  const frameS = new Frame3D({ size: 0.3, label: '{s}' });
  add(frameS.group);

  const boxGeo = new THREE.BoxGeometry(BOX[0], BOX[1], BOX[2]);
  const edgeMat = new THREE.LineBasicMaterial();
  setThemedColor(edgeMat, '--fg-muted');
  /** A body (translucent box + frame); ghosts use a lower opacity and no frame label. */
  function makeBody(opacity: number, label?: string): { group: THREE.Group; frame: Frame3D } {
    const g = add(new THREE.Group());
    g.matrixAutoUpdate = false;
    g.add(new THREE.Mesh(boxGeo, themedStandardMaterial('--series-8', { transparent: true, opacity, depthWrite: false, roughness: 0.8 })));
    g.add(new THREE.LineSegments(new THREE.EdgesGeometry(boxGeo), edgeMat));
    const frame = new Frame3D({ size: 0.26, label, opacity: opacity < 0.2 ? 0.45 : 1 });
    g.add(frame.group);
    return { group: g, frame };
  }
  const body = makeBody(0.24, '{b}');
  const ghostA = makeBody(0.08);
  const ghostB = makeBody(0.08);
  const ghostCmp = makeBody(0.12);

  const axis = new ScrewAxis3D({ color: '--series-6', length: 1.7, size: 0.8 });
  add(axis.group);
  const trailO = scene.addTube([], { radius: 0.008, color: '--series-4' });
  const trailC = scene.addTube([], { radius: 0.005, color: '--series-4', opacity: 0.6 });
  const cmpO = scene.addPolyline([ORIGIN, ORIGIN], '--fg', { dashed: true });
  const cmpC = scene.addPolyline([ORIGIN, ORIGIN], '--fg', { dashed: true });
  const helix = scene.addTube([], { radius: 0.006, color: '--series-8', opacity: 0.85 });

  const sphere = (r: number, color: string): THREE.Mesh => add(new THREE.Mesh(new THREE.SphereGeometry(r, 18, 14), themedStandardMaterial(color)));
  const qHandle = sphere(0.04, '--series-6');
  const dirHandle = sphere(0.032, '--series-6');
  const tgtHandle = sphere(0.04, '--accent');

  const arrow = (color: string, r = 0.01, head = 0.07): Arrow3D => { const a = new Arrow3D(color, r, head); add(a.group); return a; };
  const aVrot = arrow('--series-5', 0.011, 0.07);
  const aVslide = arrow('--series-7', 0.011, 0.07);
  const aT1 = arrow('--series-5', 0.011, 0.07);
  const aT2 = arrow('--series-7', 0.011, 0.07);
  const aT3 = arrow('--series-4', 0.011, 0.07);

  // wrench page: a bar with {b} at its left end, a draggable application point
  const barGroup = add(new THREE.Group());
  barGroup.matrixAutoUpdate = false;
  const barGeo = new THREE.BoxGeometry(BAR_LEN, 0.07, 0.07);
  barGeo.translate(BAR_LEN / 2, 0, 0);
  barGroup.add(new THREE.Mesh(barGeo, themedStandardMaterial('--series-8', { transparent: true, opacity: 0.45, depthWrite: false })));
  barGroup.add(new THREE.LineSegments(new THREE.EdgesGeometry(barGeo), edgeMat));
  const barFrame = new Frame3D({ size: 0.26, label: '{b}' });
  barGroup.add(barFrame.group);
  setMatrix(barGroup, T_BAR);
  const appDot = sphere(0.035, '--series-4');
  const aF = arrow('--series-4', 0.013, 0.08);
  const aMs = arrow('--series-5', 0.017, 0.09);
  const aMb = arrow('--series-5', 0.017, 0.09);
  const aVp = arrow('--series-6', 0.011, 0.07);
  const lineAction = scene.addPolyline([ORIGIN, ORIGIN], '--fg-muted', { dashed: true });
  const leverS = scene.addPolyline([ORIGIN, ORIGIN], '--series-5', { dashed: true });

  // step 7: a spatial 3R arm whose joint axes are screw axes
  // Robot3D's own axis glyphs have a fixed size (rings too large for this 0.7 m arm), so draw our own.
  const robot = new Robot3D(scene, rrr3, { showAxes: false, frameSize: 0.15, linkRadius: 0.022, eeLabel: '{b}' });
  const jointAxes = [0, 1, 2].map(() => { const a = new ScrewAxis3D({ color: '--series-6', length: 0.45, size: 0.65 }); add(a.group); return a; });

  const L = {
    start: label3D(scene, '起點', '--fg-muted'),
    end: label3D(scene, '終點', '--fg-muted'),
    target: label3D(scene, '目標 T', '--accent'),
    cmp: label3D(scene, '比較', '--fg-muted'),
    axis: label3D(scene, '螺旋軸', '--series-6'),
    q: label3D(scene, 'q', '--series-6'),
    dir: label3D(scene, 'ŝ', '--series-6'),
    vrot: label3D(scene, '−ŝ×q', '--series-5'),
    vslide: label3D(scene, 'hŝ', '--series-7'),
    f: label3D(scene, 'f', '--series-4'),
    ms: label3D(scene, 'm_s', '--series-5'),
    mb: label3D(scene, 'm_b', '--series-5'),
    vp: label3D(scene, 'v_P', '--series-6'),
  };

  // ---- legends -------------------------------------------------------------
  const legChasles = legend(root, [
    { color: 'var(--series-6)', label: '螺旋軸' },
    { color: 'var(--series-4)', label: '{b} 原點與一個角落的軌跡' },
    { color: 'var(--fg)', label: '虛線：比較方法的軌跡' },
  ]);
  const legAxis2 = legend(root, [
    { color: 'var(--series-6)', label: '螺旋軸與軸上一點 q' },
    { color: 'var(--series-8)', label: '{b} 原點的螺旋線' },
    { color: 'var(--series-5)', label: '−ŝ×q' },
    { color: 'var(--series-7)', label: 'hŝ' },
  ]);
  const legAxis3 = legend(root, [
    { color: 'var(--series-6)', label: '螺旋軸' },
    { color: 'var(--series-8)', label: '{b} 原點的螺旋線' },
    { color: 'var(--series-5)', label: 'θv' },
    { color: 'var(--series-7)', label: '(1−cos θ)[ω]v' },
    { color: 'var(--series-4)', label: '(θ−sin θ)[ω]²v' },
  ]);
  const legWrench = legend(root, [
    { color: 'var(--series-4)', label: '力 f（虛線：作用線）' },
    { color: 'var(--series-5)', label: '力矩 m_s、m_b' },
    { color: 'var(--series-6)', label: '施力點速度 v_P' },
  ]);
  const legPoe = legend(root, [{ color: 'var(--series-6)', label: '關節的螺旋軸 S_i（目前位置）' }]);

  // ---- step 1 panel --------------------------------------------------------
  const chBox = el('div');
  root.appendChild(chBox);
  const chPanel = controlPanel(chBox, { compact: true });
  const chChips = presetChips(row(chPanel), CHASLES.map((c, k) => ({ label: c.label, title: c.title, apply: () => { S.chasles = k; pathKey = ''; update(); } })), { label: '終點：' });
  const chRow = row(chPanel);
  const tauSl = slider(chRow, { label: '沿螺旋前進的比例 τ', min: 0, max: 1, step: 0.01, value: S.tau, onInput: (v) => { stopAnim(); S.tau = v; update(); } });
  tauSl.el.style.flex = '1 1 240px';
  const chPlay = button(chRow, { label: '▶ 播放', small: true, onClick: () => toggleAnim('tau', chPlay) });
  const chKv = kv(chBox, ['θ（繞軸轉角）', 'd = hθ（沿軸移動）', '節距 h', 'S = (ω, v)']);

  // ---- steps 2–3 panel -----------------------------------------------------
  const axBox = el('div');
  root.appendChild(axBox);
  const axPanel = controlPanel(axBox, { compact: true });
  const axTop = row(axPanel);
  const prisTog = toggle(axTop, { label: '純平移（ω = 0）', value: false, onChange: (v) => setPrismatic(v) });
  const axPlay = button(axTop, { label: '▶ 播放 θ', small: true, onClick: () => toggleAnim('theta', axPlay) });
  toggle(axTop, { label: '軌跡', value: true, title: '顯示 {b} 原點的螺旋線', onChange: (v) => { S.trails = v; update(); } });
  const qSl = [0, 1, 2].map((k) => slider(axPanel, {
    label: `q（${'xyz'[k]}）`, min: k === 2 ? -0.3 : -1.2, max: k === 2 ? 1 : 1.2, step: 0.01, value: S.q[k], unit: 'm',
    onInput: (v) => { S.q[k] = v; update(); },
  }));
  const azSl = angleSlider(axPanel, { label: 'ŝ 方位角', value: S.az, onInput: (v) => { S.az = v; update(); } });
  const elSl = angleSlider(axPanel, { label: 'ŝ 仰角', value: S.el, min: -Math.PI / 2, max: Math.PI / 2, onInput: (v) => { S.el = v; update(); } });
  const hSl = slider(axPanel, { label: '節距 h', min: -0.3, max: 0.3, step: 0.005, value: S.h, unit: 'm/rad', onInput: (v) => { S.h = v; update(); } });
  const thSl = slider(axTop, {
    label: 'θ', min: 0, max: TH_MAX_ROT, step: 0.01, value: S.theta,
    format: (v) => (S.prismatic ? `${fmt(v, 2)} m` : `${fmt(v / DEG, 0)}°`),
    onInput: (v) => { stopAnim(); S.theta = v; update(); },
  });
  thSl.el.style.flex = '1 1 200px';
  const axEqS = eqRow(axBox);
  const mS = cellMatrix(axEqS, { label: '\\mathcal S^{\\mathsf T}=', digits: 3, cls: screwCls });
  const axEqV = eqRow(axBox);
  const mV = captioned(axEqV, 'v', { cls: () => 'c304-v' });
  const opV1 = tex(axEqV, '=');
  const mVrot = captioned(axEqV, '-\\hat s\\times q', { cls: () => 'c304-t1' });
  const opV2 = tex(axEqV, '+');
  const mVslide = captioned(axEqV, 'h\\,\\hat s', { cls: () => 'c304-t2' });
  const prisNote = note(axBox, '純平移沒有轉軸：q 與 h 不起作用，v 是單位方向，θ 是移動距離。');
  const axEqT = eqRow(axBox);
  const mExp = captioned(axEqT, 'e^{[\\mathcal S]\\theta}', { cls: tCls });
  const axEqG = eqRow(axBox);
  const mG = captioned(axEqG, 'G(\\theta)v', { cls: () => 'c304-p' });
  tex(axEqG, '=');
  const mT1 = captioned(axEqG, '\\theta v', { cls: () => 'c304-t1' });
  tex(axEqG, '+');
  const mT2 = captioned(axEqG, '(1-\\cos\\theta)[\\omega]v', { cls: () => 'c304-t2' });
  tex(axEqG, '+');
  const mT3 = captioned(axEqG, '(\\theta-\\sin\\theta)[\\omega]^2v', { cls: () => 'c304-t3' });

  // ---- step 4 panel --------------------------------------------------------
  const lgBox = el('div');
  root.appendChild(lgBox);
  const lgPanel = controlPanel(lgBox, { compact: true });
  const lgChips = presetChips(row(lgPanel), LOG_PRESETS.map((p, k) => ({ label: p.label, title: p.title, apply: () => setTarget(LOG_PRESETS[k].P) })), { label: '目標：' });
  const setT = (k: keyof Pose6) => (v: number) => { S.target[k] = v; lgChips.clear(); update(); };
  const tgSl: Record<keyof Pose6, SliderHandle> = {
    x: slider(lgPanel, { label: 'x', min: -1.2, max: 1.2, step: 0.01, value: 0, unit: 'm', onInput: setT('x') }),
    y: slider(lgPanel, { label: 'y', min: -1.2, max: 1.2, step: 0.01, value: 0, unit: 'm', onInput: setT('y') }),
    z: slider(lgPanel, { label: 'z', min: -0.3, max: 1.2, step: 0.01, value: 0, unit: 'm', onInput: setT('z') }),
    yaw: angleSlider(lgPanel, { label: '繞 z 轉', value: 0, onInput: setT('yaw') }),
    pitch: angleSlider(lgPanel, { label: '繞 y 轉', value: 0, min: -Math.PI / 2, max: Math.PI / 2, onInput: setT('pitch') }),
    roll: angleSlider(lgPanel, { label: '繞 x 轉', value: 0, onInput: setT('roll') }),
  };
  const lgPanel2 = controlPanel(lgBox, { compact: true });
  const cmpSel = select<Cmp>(lgPanel2, {
    label: '比較另一種內插',
    options: [
      { value: 'none', label: '不比較' },
      { value: 'decoupled', label: '分開內插 R、p' },
      { value: 'elementwise', label: '逐元素內插' },
    ],
    value: 'none',
    onChange: (v) => { S.cmp = v; pathKey = ''; syncVisibility(); update(); },
  });
  const tauLogSl = slider(lgPanel2, { label: 'τ', min: 0, max: 1, step: 0.01, value: S.tauLog, onInput: (v) => { stopAnim(); S.tauLog = v; update(); } });
  const lgPlay = button(row(lgPanel2), { label: '▶ 播放 τ', small: true, onClick: () => toggleAnim('tauLog', lgPlay) });
  const KL = { S: 'S（log T 的螺旋軸）', th: 'θ', h: '節距 h', kind: '情況', det: 'det R(τ)（比較方法）' };
  const lgKv = kv(lgBox, Object.values(KL));

  // ---- steps 5–6 panel -----------------------------------------------------
  const wrBox = el('div');
  root.appendChild(wrBox);
  const wrPanel = controlPanel(wrBox, { compact: true });
  const dSl = slider(wrPanel, { label: '施力點 d（沿 x_b）', min: 0, max: BAR_LEN, step: 0.01, value: S.d, unit: 'm', onInput: (v) => { S.d = v; update(); } });
  const fSl = [0, 1, 2].map((k) => slider(wrPanel, {
    label: `f_b（${'xyz'[k]}）`, min: -10, max: 10, step: 0.1, value: S.fb[k], unit: 'N',
    onInput: (v) => { S.fb[k] = v; update(); },
  }));
  const mviewSel = select<MomentView>(wrPanel, {
    label: '畫出哪個原點的力矩',
    options: [{ value: 'both', label: '{s} 與 {b}' }, { value: 's', label: '只畫 m_s（{s} 原點）' }, { value: 'b', label: '只畫 m_b（{b} 原點）' }],
    value: 'both',
    onChange: (v) => { S.mview = v; update(); },
  });
  const wrVec = el('div');
  wrBox.appendChild(wrVec);
  const mFs = cellMatrix(eqRow(wrVec), { label: '\\mathcal F_s^{\\mathsf T}=', digits: 2, cls: wrenchCls });
  const mFb = cellMatrix(eqRow(wrVec), { label: '\\mathcal F_b^{\\mathsf T}=', digits: 2, cls: wrenchCls });
  const pwBox = el('div');
  wrBox.appendChild(pwBox);
  const vbPanel = controlPanel(pwBox, { compact: true });
  const VB_LABELS = ['ω_b（x）', 'ω_b（y）', 'ω_b（z）', 'v_b（x）', 'v_b（y）', 'v_b（z）'];
  const vbSl = VB_LABELS.map((lab, k) => slider(vbPanel, {
    label: lab, min: k < 3 ? -1.5 : -1, max: k < 3 ? 1.5 : 1, step: k < 3 ? 0.05 : 0.01, value: S.Vb[k], unit: k < 3 ? 'rad/s' : 'm/s',
    onInput: (v) => { S.Vb[k] = v; update(); },
  }));
  const KP = { s: 'V_sᵀ F_s（{s} 的數字）', b: 'V_bᵀ F_b（{b} 的數字）', pt: 'f · v_P（力 · 施力點速度）', chk: '‖[Ad_Tsb]ᵀ F_s − F_b‖（應為 0）' };
  const pwKv = kv(pwBox, Object.values(KP));

  // ---- step 7 panel --------------------------------------------------------
  const poeBox = el('div');
  root.appendChild(poeBox);
  const js = jointSliders(poeBox, { model: rrr3, values: S.q3, onInput: (th) => { stopAnim(); S.q3 = th; update(); } });
  const poePlay = button(row(controlPanel(poeBox, { compact: true })), { label: '▶ 播放', small: true, onClick: () => toggleAnim('poe', poePlay) });
  const poeEq = eqRow(poeBox);
  const mSlist = captioned(poeEq, '[\\,\\mathcal S_1\\ \\mathcal S_2\\ \\mathcal S_3\\,]', { note: '（零位）', digits: 2, cls: (i) => (i < 3 ? 'c304-w' : 'c304-v') });
  const mFk = captioned(poeEq, 'T(\\theta)', { digits: 2, cls: tCls });

  // ---- animation -----------------------------------------------------------
  const playButtons = [chPlay, axPlay, lgPlay, poePlay];
  const loop = createLoop((dt) => {
    if (anim === 'tau') {
      S.tau = Math.min(1, S.tau + dt / 3);
      tauSl.set(S.tau, { silent: true });
      if (S.tau >= 1) stopAnim();
    } else if (anim === 'theta') {
      const max = S.prismatic ? TH_MAX_LIN : TH_MAX_ROT;
      S.theta = Math.min(max, S.theta + dt * (S.prismatic ? 0.3 : 1.1));
      thSl.set(S.theta, { silent: true });
      if (S.theta >= max) stopAnim();
    } else if (anim === 'tauLog') {
      S.tauLog = Math.min(1, S.tauLog + dt / 3);
      tauLogSl.set(S.tauLog, { silent: true });
      if (S.tauLog >= 1) stopAnim();
    } else if (anim === 'poe') {
      S.poeT += dt;
      const t = S.poeT;
      S.q3 = [POE_HOME[0] + 0.9 * Math.sin(0.6 * t), POE_HOME[1] + 0.45 * Math.sin(0.9 * t + 0.5), POE_HOME[2] + 0.8 * Math.sin(1.3 * t + 1)];
      js.set(S.q3, { silent: true });
    }
    update();
  });
  function stopAnim(): void {
    anim = null;
    loop.stop();
    playButtons.forEach((b) => { b.textContent = b === axPlay ? '▶ 播放 θ' : b === lgPlay ? '▶ 播放 τ' : '▶ 播放'; });
  }
  function toggleAnim(a: Anim, btn: HTMLButtonElement): void {
    if (anim === a) { stopAnim(); return; }
    stopAnim();
    // restart from the beginning when the previous run has finished
    if (a === 'tau' && S.tau >= 1) S.tau = 0;
    if (a === 'tauLog' && S.tauLog >= 1) S.tauLog = 0;
    if (a === 'theta' && S.theta >= (S.prismatic ? TH_MAX_LIN : TH_MAX_ROT) - 1e-6) S.theta = 0;
    anim = a;
    btn.textContent = '⏸ 暫停';
    loop.start();
  }

  // ---- drag handles ----------------------------------------------------------
  const v3 = (p: Vec) => new THREE.Vector3(p[0], p[1], p[2]);
  const drag = attachDragHandles(scene, [
    {
      // axis point q: horizontal plane through q
      pos: () => (MODE_OF[S.sc] === 'axis' && !S.prismatic ? v3(S.q) : null),
      drag: (ray) => {
        const p = rayPlaneZ(ray, S.q[2]);
        if (!p) return;
        S.q[0] = clamp(p.x, -1.2, 1.2);
        S.q[1] = clamp(p.y, -1.2, 1.2);
        qSl[0].set(S.q[0], { silent: true });
        qSl[1].set(S.q[1], { silent: true });
        update();
      },
    },
    {
      // axis direction ŝ: a handle on the sphere of radius DIR_R around q (around the origin for a translation)
      pos: () => (MODE_OF[S.sc] === 'axis' ? v3(vecAdd(S.prismatic ? ORIGIN : S.q, vecScale(dirFromAngles(S.az, S.el), DIR_R))) : null),
      drag: (ray) => {
        const c = v3(S.prismatic ? ORIGIN : S.q);
        const oc = ray.origin.clone().sub(c);
        const b = oc.dot(ray.direction);
        const disc = b * b - (oc.lengthSq() - DIR_R * DIR_R);
        // nearest intersection with the sphere; when the ray misses, its closest point to the centre
        const t = disc >= 0 ? -b - Math.sqrt(disc) : -b;
        const d = ray.origin.clone().addScaledVector(ray.direction, t).sub(c);
        if (d.lengthSq() < 1e-12) return;
        d.normalize();
        S.az = Math.atan2(d.y, d.x);
        S.el = Math.asin(clamp(d.z, -1, 1));
        azSl.set(S.az, { silent: true });
        elSl.set(S.el, { silent: true });
        update();
      },
    },
    {
      // target position for log T: horizontal plane through the target origin
      pos: () => (MODE_OF[S.sc] === 'log' ? v3([S.target.x, S.target.y, S.target.z]) : null),
      drag: (ray) => {
        const p = rayPlaneZ(ray, S.target.z);
        if (!p) return;
        S.target.x = clamp(p.x, -1.2, 1.2);
        S.target.y = clamp(p.y, -1.2, 1.2);
        tgSl.x.set(S.target.x, { silent: true });
        tgSl.y.set(S.target.y, { silent: true });
        lgChips.clear();
        update();
      },
    },
    {
      // application point: along the bar's x_b axis
      pos: () => (MODE_OF[S.sc] === 'wrench' ? v3(transformPoint(T_BAR, [S.d, 0, 0])) : null),
      drag: (ray) => {
        const a = v3([T_BAR[0][3], T_BAR[1][3], T_BAR[2][3]]);
        const w = v3([T_BAR[0][0], T_BAR[1][0], T_BAR[2][0]]);
        const s = rayLineParam(ray, a, w);
        if (s == null) return;
        S.d = clamp(s, 0, BAR_LEN);
        dSl.set(S.d, { silent: true });
        update();
      },
    },
  ]);

  // ---- helpers ---------------------------------------------------------------
  function place(a: Arrow3D, from: Vec, to: Vec, on = true): void {
    a.setFromTo(from, to);
    a.visible = on && norm(vecSub(to, from)) > 1e-6;
  }
  function putLabel(s: THREE.Sprite, p: Vec, on = true, dz = 0.07): void {
    s.visible = on;
    if (on) s.position.set(p[0], p[1], p[2] + dz);
  }
  const tipOf = (from: Vec, d: Vec, extra = 0.08): Vec => {
    const n = norm(d);
    return n < 1e-9 ? from : vecAdd(from, vecScale(d, 1 + extra / n));
  };
  const pos = (T: Mat): Vec => [T[0][3], T[1][3], T[2][3]];
  function setPill(text: string, kind: 'info' | 'ok' | 'bad' = 'info'): void {
    if (pill.textContent !== text) pill.textContent = text;
    const cn = `c304-pill ${kind}`;
    if (pill.className !== cn) pill.className = cn;
  }
  const show = (e: HTMLElement, on: boolean) => { const d = on ? '' : 'none'; if (e.style.display !== d) e.style.display = d; };
  const fmtTheta = (i: ScrewInfo): string => (i.kind === 'translation' ? `${fmt(i.theta, 3)} m（距離）` : `${fmt(i.theta, 3)} rad（${fmt(i.theta / DEG, 1)}°）`);
  const fmtH = (i: ScrewInfo): string => (i.kind === 'translation' ? '∞（純平移）' : i.kind === 'identity' ? '—' : `${fmt(i.h, 3)} m/rad`);
  function setCamera(c: { pos: Vec; target: Vec }): void {
    scene.camera.position.set(c.pos[0], c.pos[1], c.pos[2]);
    scene.controls?.target.set(c.target[0], c.target[1], c.target[2]);
    scene.controls?.update();
  }
  function setPrismatic(v: boolean): void {
    S.prismatic = v;
    stopAnim();
    const max = v ? TH_MAX_LIN : TH_MAX_ROT;
    S.theta = v ? Math.min(S.theta, 0.6) : S.theta;
    thSl.setRange(0, max, 0.01);
    thSl.set(S.theta, { silent: true });
    qSl.forEach((s) => { s.disabled = v; });
    hSl.disabled = v;
    pathKey = '';
    syncVisibility();
    update();
  }
  function setTarget(P: Pose6): void {
    S.target = { ...P };
    for (const k of Object.keys(tgSl) as (keyof Pose6)[]) tgSl[k].set(S.target[k], { silent: true });
    pathKey = '';
    update();
  }
  function hideAll(): void {
    for (const g of [body.group, ghostA.group, ghostB.group, ghostCmp.group, barGroup]) g.visible = false;
    for (const a of [aVrot, aVslide, aT1, aT2, aT3, aF, aMs, aMb, aVp]) a.visible = false;
    for (const s of Object.values(L)) s.visible = false;
    for (const m of [trailO.mesh, trailC.mesh, helix.mesh, qHandle, dirHandle, tgtHandle, appDot]) m.visible = false;
    for (const l of [cmpO.line, cmpC.line, lineAction.line, leverS.line]) l.visible = false;
    axis.setVisible(false);
    robot.setVisible(false);
    jointAxes.forEach((a) => a.setVisible(false));
  }

  // ---- per-mode updates --------------------------------------------------------
  function drawScrewAxis(info: ScrewInfo, anchor: Vec): void {
    if (info.kind === 'identity') return;
    axis.setVisible(true);
    axis.setScrew(info.S);
    if (info.kind === 'screw') {
      const dir = info.S.slice(0, 3);
      const t = (anchor[0] - info.q[0]) * dir[0] + (anchor[1] - info.q[1]) * dir[1] + (anchor[2] - info.q[2]) * dir[2];
      putLabel(L.axis, vecAdd(info.q, vecScale(dir, t + 0.75)), true, 0);
    }
  }

  function updateChasles(): void {
    const T2 = CHASLES[S.chasles].T2;
    const info = screwBetween(T1_CH, T2);
    body.group.visible = true;
    setMatrix(body.group, interpolatePose(T1_CH, T2, S.tau, 'screw'));
    ghostA.group.visible = true;
    setMatrix(ghostA.group, T1_CH);
    ghostB.group.visible = true;
    setMatrix(ghostB.group, T2);
    putLabel(L.start, pos(T1_CH), true, -0.16);
    putLabel(L.end, pos(T2), true, 0.2);
    const key = `ch${S.chasles}`;
    if (key !== pathKey) {
      pathKey = key;
      trailO.update(interpolationPath(T1_CH, T2, ORIGIN, 'screw'));
      trailC.update(interpolationPath(T1_CH, T2, CORNER, 'screw'));
    }
    trailO.mesh.visible = true;
    trailC.mesh.visible = true;
    drawScrewAxis(info, pos(T1_CH));
    chKv.set('θ（繞軸轉角）', fmtTheta(info));
    chKv.set('d = hθ（沿軸移動）', info.kind === 'translation' ? `${fmt(info.theta, 3)} m` : `${fmt(info.h * info.theta, 3)} m`);
    chKv.set('節距 h', fmtH(info));
    chKv.set('S = (ω, v)', fmtVec(info.S, 2));
    if (info.kind === 'translation') setPill(`兩個姿態相同：純平移 ${fmt(info.theta, 2)} m（ω = 0）`);
    else if (Math.abs(info.theta - Math.PI) < 1e-6) setPill('θ = π：沿 ±ω̂ 轉半圈都到得了終點，兩個軸方向都正確');
    else setPill(`繞螺旋軸轉 θ = ${fmt(info.theta / DEG, 0)}°，同時沿軸移動 d = ${fmt(info.h * info.theta, 2)} m`);
  }

  function updateAxis(): void {
    const s = dirFromAngles(S.az, S.el);
    const Sv = S.prismatic ? prismaticScrew(s) : screwFromQSH(S.q, s, S.h);
    const T = screwMotion(Sv, S.theta, eye(4));
    body.group.visible = true;
    setMatrix(body.group, T);
    axis.setVisible(true);
    axis.setScrew(Sv);
    const key = `ax${Sv.join(',')}`;
    if (key !== pathKey) {
      pathKey = key;
      helix.update(screwPath(Sv, S.prismatic ? TH_MAX_LIN : TH_MAX_ROT, eye(4), ORIGIN, 160));
    }
    helix.mesh.visible = S.trails;
    const dirTip = vecAdd(S.prismatic ? ORIGIN : S.q, vecScale(s, DIR_R));
    dirHandle.visible = true;
    dirHandle.position.set(dirTip[0], dirTip[1], dirTip[2]);
    putLabel(L.dir, dirTip, true, 0.09);
    if (!S.prismatic) {
      qHandle.visible = true;
      qHandle.position.set(S.q[0], S.q[1], S.q[2]);
      putLabel(L.q, S.q, true, 0.1);
    }
    mS.set([Sv]);
    if (S.sc === 1) {
      if (!S.prismatic) {
        const { rot, slide } = vParts(S.q, s, S.h);
        place(aVrot, ORIGIN, rot);
        place(aVslide, rot, vecAdd(rot, slide));
        putLabel(L.vrot, tipOf(ORIGIN, rot), norm(rot) > 1e-6, 0.02);
        putLabel(L.vslide, tipOf(rot, slide), Math.abs(S.h) > 1e-6, 0.02);
        mVrot.set(rot);
        mVslide.set(slide);
      }
      mV.set(Sv.slice(3, 6));
      setPill(S.prismatic ? '純平移：ω = 0，‖v‖ = 1' : `‖ω‖ = 1，ωᵀv = h = ${fmt(S.h, 3)} m/rad`);
    } else {
      const [t1, t2, t3] = translationTerms(Sv, S.theta);
      const a = t1, b = vecAdd(t1, t2), c = vecAdd(b, t3);
      place(aT1, ORIGIN, a);
      place(aT2, a, b);
      place(aT3, b, c);
      mExp.set(T);
      mG.set(c);
      mT1.set(t1);
      mT2.set(t2);
      mT3.set(t3);
      setPill(S.prismatic ? `平移 θ = ${fmt(S.theta, 2)} m：G = Iθ，只剩第一項` : `θ = ${fmt(S.theta / DEG, 0)}°：三項相加 = {b} 原點的位置 ${fmtVec(c, 2)}`);
    }
  }

  function updateLog(): void {
    const T2 = toT(S.target);
    const info = screwBetween(eye(4), T2);
    body.group.visible = true;
    setMatrix(body.group, interpolatePose(eye(4), T2, S.tauLog, 'screw'));
    ghostB.group.visible = true;
    setMatrix(ghostB.group, T2);
    tgtHandle.visible = true;
    tgtHandle.position.set(S.target.x, S.target.y, S.target.z);
    putLabel(L.target, pos(T2), true, 0.2);
    const key = `lg${Object.values(S.target).join(',')}|${S.cmp}`;
    if (key !== pathKey) {
      pathKey = key;
      trailO.update(interpolationPath(eye(4), T2, ORIGIN, 'screw'));
      trailC.update(interpolationPath(eye(4), T2, CORNER, 'screw'));
      if (S.cmp !== 'none') {
        setLinePoints(cmpO.line, interpolationPath(eye(4), T2, ORIGIN, S.cmp), true);
        setLinePoints(cmpC.line, interpolationPath(eye(4), T2, CORNER, S.cmp), true);
      }
    }
    trailO.mesh.visible = true;
    trailC.mesh.visible = true;
    drawScrewAxis(info, ORIGIN);
    let detTxt = '—';
    if (S.cmp !== 'none') {
      const Tc = interpolatePose(eye(4), T2, S.tauLog, S.cmp);
      ghostCmp.group.visible = true;
      setMatrix(ghostCmp.group, Tc);
      putLabel(L.cmp, pos(Tc), true, -0.16);
      cmpO.line.visible = true;
      cmpC.line.visible = true;
      detTxt = fmt(rotationBlockDet(Tc), 3);
    }
    lgKv.set(KL.S, fmtVec(info.S, 2));
    lgKv.set(KL.th, info.kind === 'identity' ? '0' : fmtTheta(info));
    lgKv.set(KL.h, fmtH(info));
    const nearPi = info.kind === 'screw' && Math.abs(info.theta - Math.PI) < 1e-6;
    const kind = info.kind === 'identity' ? 'T = I：沒有位移' : info.kind === 'translation' ? 'R = I：ω = 0，v = p/‖p‖，θ = ‖p‖'
      : nearPi ? 'θ = π：+ω̂ 與 −ω̂ 都是正確答案' : '一般螺旋：θ ∈ (0, π)';
    lgKv.set(KL.kind, kind);
    lgKv.set(KL.det, detTxt);
    if (S.cmp === 'elementwise') {
      const dt = rotationBlockDet(interpolatePose(eye(4), T2, S.tauLog, 'elementwise'));
      setPill(`逐元素內插：det R(τ) = ${fmt(dt, 3)}${dt < 0.999 ? '，不是旋轉矩陣，物體被壓扁' : ''}`, dt < 0.999 ? 'bad' : 'info');
    } else if (S.cmp === 'decoupled') {
      const thTxt = info.kind === 'translation' ? `${fmt(info.theta, 2)} m` : `${fmt(info.theta / DEG, 0)}°`;
      setPill(`log T：θ = ${thTxt}，h = ${fmtH(info)}｜虛線：分開內插，原點走直線、角落走另一條曲線`);
    } else if (info.kind === 'screw' && Math.hypot(info.q[0], info.q[1], info.q[2]) > 2.5) {
      setPill(`轉角只有 ${fmt(info.theta / DEG, 1)}°：螺旋軸離原點 ${fmt(norm(info.q), 1)} m，運動幾乎是平移`);
    } else setPill(`log T：θ = ${info.kind === 'translation' ? `${fmt(info.theta, 2)} m` : `${fmt(info.theta / DEG, 0)}°`}，h = ${fmtH(info)}`);
  }

  function updateWrench(): void {
    barGroup.visible = true;
    const R = T_BAR.slice(0, 3).map((r) => r.slice(0, 3));
    const pb = pos(T_BAR);
    const rb: Vec = [S.d, 0, 0];
    const rs = transformPoint(T_BAR, rb);
    const fb = S.fb;
    const fs = matVec(R, fb);
    const Fb = wrenchAtPoint(rb, fb);
    const Fs = wrenchInFrame(TransInv(T_BAR), Fb); // F_s = [Ad_{T_bs}]ᵀ F_b
    appDot.visible = true;
    appDot.position.set(rs[0], rs[1], rs[2]);
    place(aF, rs, vecAdd(rs, vecScale(fs, F_SCALE)));
    putLabel(L.f, tipOf(rs, vecScale(fs, F_SCALE)), norm(fs) > 1e-9, 0.02);
    const fn = norm(fs);
    if (fn > 1e-9) {
      const u = vecScale(fs, 1 / fn);
      lineAction.update([vecAdd(rs, vecScale(u, -1.2)), vecAdd(rs, vecScale(u, 1.2))]);
      lineAction.line.visible = true;
    }
    const ms = Fs.slice(0, 3);
    const mbW = matVec(R, Fb.slice(0, 3)); // m_b drawn as a world vector at the {b} origin
    const showS = S.mview !== 'b', showB = S.mview !== 's';
    place(aMs, ORIGIN, vecScale(ms, M_SCALE), showS);
    putLabel(L.ms, tipOf(ORIGIN, vecScale(ms, M_SCALE)), showS && norm(ms) > 1e-9, 0.02);
    place(aMb, pb, vecAdd(pb, vecScale(mbW, M_SCALE)), showB);
    putLabel(L.mb, tipOf(pb, vecScale(mbW, M_SCALE)), showB && norm(mbW) > 1e-9, 0.02);
    if (showS) { leverS.update([ORIGIN, rs]); leverS.line.visible = true; }
    mFs.set([Fs]);
    mFb.set([Fb]);
    if (S.sc === 4) {
      setPill(`同一個力：‖m_s‖ = ${fmt(norm(ms), 2)} N·m，‖m_b‖ = ${fmt(norm(Fb.slice(0, 3)), 2)} N·m，f 的大小都是 ${fmt(fn, 2)} N`);
      return;
    }
    // step 6: power in both frames and via the application point
    const Vb = S.Vb;
    const Vs = matVec(Adjoint(T_BAR), Vb);
    const vP = vecAdd(cross(Vs.slice(0, 3), rs), Vs.slice(3, 6)); // velocity of the application point, in {s}
    place(aVp, rs, vecAdd(rs, vecScale(vP, V_SCALE)));
    putLabel(L.vp, tipOf(rs, vecScale(vP, V_SCALE)), norm(vP) > 1e-9, 0.02);
    const Ps = power(Vs, Fs), Pb = power(Vb, Fb), Pp = fs[0] * vP[0] + fs[1] * vP[1] + fs[2] * vP[2];
    const chk = norm(vecSub(matVec(transpose(Adjoint(T_BAR)), Fs), Fb));
    pwKv.set(KP.s, `${fmt(Ps, 3)} W`);
    pwKv.set(KP.b, `${fmt(Pb, 3)} W`);
    pwKv.set(KP.pt, `${fmt(Pp, 3)} W`);
    pwKv.set(KP.chk, chk.toExponential(1));
    setPill(`P = V_sᵀF_s = V_bᵀF_b = ${fmt(Pb, 3)} W（換座標不會改變功率）`, 'ok');
  }

  function updatePoe(): void {
    robot.setVisible(true);
    const { T, P } = robot.setConfig(S.q3);
    // joint i's current screw axis in {s}: [Ad_{e^[S1]θ1 ⋯ e^[S(i-1)]θ(i-1)}] S_i
    jointAxes.forEach((a, i) => {
      const Si = getCol(rrr3.Slist, i);
      a.setScrew(i === 0 ? Si : matVec(Adjoint(P[i - 1]), Si));
      a.setVisible(true);
    });
    mSlist.set(rrr3.Slist);
    mFk.set(T);
    const T2 = FKinSpace(rrr3.M, rrr3.Slist, S.q3);
    setPill(`末端位置 p = ${fmtVec(pos(T2), 3)} m（= e^[S₁]θ₁ e^[S₂]θ₂ e^[S₃]θ₃ M）`);
  }

  function update(): void {
    hideAll();
    const mode = MODE_OF[S.sc];
    if (mode === 'chasles') updateChasles();
    else if (mode === 'axis') updateAxis();
    else if (mode === 'log') updateLog();
    else if (mode === 'wrench') updateWrench();
    else updatePoe();
    scene.requestRender();
  }

  function syncVisibility(): void {
    const sc = S.sc;
    const mode = MODE_OF[sc];
    show(legChasles, mode === 'chasles' || mode === 'log');
    const cmpItem = legChasles.children[2] as HTMLElement | undefined;
    if (cmpItem) show(cmpItem, mode === 'log' && S.cmp !== 'none');
    show(legAxis2, sc === 1);
    show(legAxis3, sc === 2);
    show(legWrench, mode === 'wrench');
    const vpItem = legWrench.children[2] as HTMLElement | undefined;
    if (vpItem) show(vpItem, sc === 5);
    show(legPoe, mode === 'poe');
    show(chBox, mode === 'chasles');
    show(axBox, mode === 'axis');
    show(axEqS, sc === 1);
    show(axEqV, sc === 1);
    for (const e of [mVrot.box, mVslide.box, opV2]) show(e, !S.prismatic);
    show(opV1, !S.prismatic);
    show(prisNote, sc === 1 && S.prismatic);
    show(axEqT, sc === 2);
    show(axEqG, sc === 2);
    show(lgBox, mode === 'log');
    show(wrBox, mode === 'wrench');
    show(pwBox, sc === 5);
    show(poeBox, mode === 'poe');
    hint3d.textContent = mode === 'axis' ? '拖曳青色圓球：q 移動軸、ŝ 改方向' : mode === 'log' ? '拖曳藍色圓球：移動目標位置'
      : mode === 'wrench' ? '沿桿子拖曳橘色圓球：移動施力點' : '拖曳：轉視角　滾輪：縮放';
    const focus: HTMLElement[] = [chBox, axEqV, axEqG, lgPanel2, wrVec, pwBox, js.el];
    focus.forEach((f, k) => f.classList.toggle('mr-focus', k === sc));
  }

  function applyStep(i: number): void {
    stopAnim();
    const prev = S.sc;
    S.sc = Math.max(0, Math.min(steps.length - 1, i));
    pathKey = '';
    const mode = MODE_OF[S.sc];
    if (mode === 'chasles') {
      S.chasles = 0;
      chChips.highlight(0);
      S.tau = 0.55;
      tauSl.set(S.tau, { silent: true });
    } else if (mode === 'axis') {
      S.q = [0.6, 0.2, 0];
      S.az = 0;
      S.el = 90 * DEG;
      S.h = 0.1;
      qSl.forEach((s, k) => s.set(S.q[k], { silent: true }));
      azSl.set(S.az, { silent: true });
      elSl.set(S.el, { silent: true });
      hSl.set(S.h, { silent: true });
      prisTog.set(false, { silent: true });
      S.prismatic = false;
      qSl.forEach((s) => { s.disabled = false; });
      hSl.disabled = false;
      thSl.setRange(0, TH_MAX_ROT, 0.01);
      S.theta = S.sc === 1 ? 2.2 : 1.6;
      thSl.set(S.theta, { silent: true });
    } else if (mode === 'log') {
      setTarget(LOG_PRESETS[0].P);
      lgChips.highlight(0);
      S.cmp = 'decoupled';
      cmpSel.set('decoupled', { silent: true });
      S.tauLog = 0.5;
      tauLogSl.set(S.tauLog, { silent: true });
    } else if (mode === 'wrench') {
      S.d = 0.75;
      S.fb = [0, 3, -7];
      S.Vb = [0, 0, 0.8, 0, 0, 0];
      S.mview = 'both';
      dSl.set(S.d, { silent: true });
      fSl.forEach((s, k) => s.set(S.fb[k], { silent: true }));
      vbSl.forEach((s, k) => s.set(S.Vb[k], { silent: true }));
      mviewSel.set('both', { silent: true });
    } else {
      S.q3 = POE_HOME.slice();
      S.poeT = 0;
      js.set(S.q3, { silent: true });
    }
    if (S.sc !== prev) setCamera(CAMS[S.sc]);
    syncVisibility();
    update();
  }

  applyStep(ctx.currentStep);

  return {
    onStep: applyStep,
    destroy() {
      stopAnim();
      drag.destroy();
      // the robot's group lives in scene.scene, so Scene3D.destroy() disposes its meshes too
      disposeSprites(scene.scene);
      scene.destroy();
      root.remove();
    },
  };
}

const lesson: Lesson = { meta, steps, quiz, mount };
export default lesson;
