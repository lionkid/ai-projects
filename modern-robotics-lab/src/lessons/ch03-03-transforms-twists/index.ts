import * as THREE from 'three';
import type { Lesson, LessonContext, LessonHandle, LessonStep, QuizQuestion } from '@/types';
import {
  el, fmt, fmtVec, controlPanel, row, slider, angleSlider, button, select, readout, simTitle, kv, legend, note,
  presetChips, playback, createLoop, Scene3D, Arrow3D, Frame3D, Link3D, ScrewAxis3D, setMatrix, setThemedColor,
  themedStandardMaterial, type SliderHandle,
} from '@/ui';
import {
  type Mat, type Vec, Adjoint, MatrixLog6, RotX, RotZ, RpToTrans, TransInv, clamp, eye, matMul, matVec, norm, se3ToVec,
  transformPoint, vecAdd, vecScale, vecSub,
} from '@/mr';
import { meta } from './meta';
import {
  type InverseKind, type PoseParams, type Side, applyFraction, bodyPointPath, changeFrame, closestPointOnLine,
  constantBodyTwistPose, distanceFromIdentity, hasHomogeneousRow, inverseCandidate, loopDuration, numericDerivative,
  originPath, pointVelocity, poseFromParams, screwInfo, twistsFromDerivative,
} from './twists';
import { attachDragHandles, rayPlaneZ } from './drag3d';
import { captioned, cellMatrix, disposeSprites, eqRow, label3D, tex } from './widgets';

// ---------------------------------------------------------------------------
// Narrative
// ---------------------------------------------------------------------------

const steps: LessonStep[] = [
  {
    title: '把 R 與 p 裝進 4×4',
    md: `要描述 $\\{b\\}$ 相對於 $\\{s\\}$ 的位姿，需要兩樣東西：姿態 $R$，以及 $\\{b\\}$ 原點的位置 $p$。把它們排進一個 $4\\times 4$ 矩陣，就是 $T_{sb}\\in\\SE(3)$。

一個點在 $\\{b\\}$ 中的座標是 $r_b$，換成 $\\{s\\}$ 的座標要「先轉、再移」：$r_s = R\\,r_b + p$。在座標後面補上一個 1（**齊次座標**），兩個動作就併成一次矩陣乘法：

$$T_{sb}=\\begin{bmatrix}R & p\\\\ 0 & 1\\end{bmatrix},\\qquad \\begin{bmatrix}r_s\\\\ 1\\end{bmatrix}=T_{sb}\\begin{bmatrix}r_b\\\\ 1\\end{bmatrix}$$

如果補的是 0，最後一欄的 $p$ 會被乘掉、只剩旋轉；方向、速度這類「自由向量」正是這樣換座標的。`,
    simHint: '拉動滑桿改變 $T_{sb}$。橘色點 P 黏在物體上：$r_b$ 永遠不變，$r_s = T_{sb}\\,r_b$ 跟著改變。對照矩陣的前三欄與 {b} 的紅綠藍軸，最後一欄與灰色箭頭 $p$。',
  },
  {
    title: '合成、求逆、換參考系',
    md: `已知 $\\{c\\}$ 相對於 $\\{b\\}$ 的 $T_{bc}$，要得到它相對於 $\\{s\\}$ 的位姿，直接相乘即可：相鄰的下標互相抵消，就像箭頭 $s\\to b\\to c$ 首尾相接。

反過來，$T_{bs}=T_{sb}^{-1}$ 描述「從 $\\{b\\}$ 看 $\\{s\\}$」。因為 $R$ 是正交矩陣，求逆不必做一般的矩陣反運算；但它也**不是**轉置：位置欄要換成 $-R^{\\T}p$，也就是從 $\\{b\\}$ 原點指向 $\\{s\\}$ 原點的向量，用 $\\{b\\}$ 的座標寫出來。

$$T_{sc}=T_{sb}\\,T_{bc},\\qquad T^{-1}=\\begin{bmatrix}R^{\\T} & -R^{\\T}p\\\\ 0 & 1\\end{bmatrix}$$`,
    simHint: '把「滑桿編輯」切到 T_bc 來移動 {c}：紫色箭頭 s→c 永遠等於 s→b→c。再切換「反矩陣候選 X」：只有正確的 X 會讓 $T_{sb}X$ 的半透明座標系疊回 {s}。',
  },
  {
    title: '前乘與後乘',
    md: `同一個變換 $T$ 可以乘在 $T_{sb}$ 的左邊或右邊，意義不同：

- **前乘** $T\\,T_{sb}$：$T$ 的轉軸與平移方向用**固定的** $\\{s\\}$ 來解讀。繞 $\\hat z_s$ 轉的時候，$\\{b\\}$ 的原點也會被帶著繞 $\\hat z_s$ 軸畫圓弧。
- **後乘** $T_{sb}\\,T$：用物體**自己的** $\\{b\\}$ 軸解讀，像是「繞自己的 $\\hat z_b$ 原地轉」或「沿自己的 $\\hat x_b$ 前進」。

規則和旋轉矩陣一樣，差別在於現在連平移也一起受到影響。`,
    simHint: '選一個固定的 T，拉動 u 或按「▶ 動畫」。實線物體是目前選的乘法，半透明的是另一種；虛線軸是 T 的轉軸實際所在的位置。',
  },
  {
    title: '旋量：角速度加上參考點的線速度',
    md: `剛體的速度需要 6 個數字：所有點共用的角速度 $\\omega$，加上**某一個參考點**的線速度 $v$。在 $\\{b\\}$ 中描述時，參考點就是 $\\{b\\}$ 的原點，兩者都寫成 $\\{b\\}$ 的座標：

$$\\vV_b=\\begin{bmatrix}\\omega_b\\\\ v_b\\end{bmatrix}\\in\\R^6,\\qquad \\dot r = v_b+\\omega_b\\times r_b$$

物體上其他點（在 $\\{b\\}$ 中位於 $r_b$）的速度，等於原點的速度再加上轉動造成的 $\\omega_b\\times r_b$，所以離轉軸越遠的點跑得越快。這個六維向量稱為**物體旋量（body twist）**。`,
    simHint: '按 ▶ 播放，或調整 $\\mathcal V_b$ 的六個滑桿。紫色是 ω、青色是 {b} 原點的速度、橘色是四個角落的速度；灰色細管是 {b} 原點接下來的路徑。',
  },
  {
    title: 'V_s 與 V_b',
    md: `同一個運動也可以從固定的 $\\{s\\}$ 來描述。兩種 twist 都從 $\\dot T$ 讀出來，只差在 $T^{-1}$ 乘在哪一邊：

$$[\\vV_b]=T_{sb}^{-1}\\dot T_{sb},\\qquad [\\vV_s]=\\dot T_{sb}\\,T_{sb}^{-1}$$

$\\omega_s$ 和 $\\omega_b$ 是同一支箭頭的兩組座標（$\\omega_s=R\\,\\omega_b$）。但 $v_s$ **不是** $\\{b\\}$ 原點的速度：把物體想像成無限延伸的剛體，$v_s$ 是此刻正好經過 $\\{s\\}$ 原點的那一點的速度。所以只要移動 $\\{s\\}$，即使物體的運動完全沒變，$v_s$ 也會不同。`,
    simHint: '「原地自轉」時 $v_b = 0$，粉紅色的 $v_s$ 卻很長。拖曳粉紅圓環（或用 {s} 位置滑桿）移動 {s}：ω 不變、$v_s$ 改變。按「把 {s} 放到轉軸上」，$v_s$ 變成 0。',
  },
  {
    title: 'Adjoint：換座標的 6×6 矩陣',
    md: `把上一步的兩個定義合併，就得到 twist 換座標的線性關係：

$$\\vV_s=[\\Ad_{T_{sb}}]\\,\\vV_b,\\qquad [\\Ad_T]=\\begin{bmatrix}R & 0\\\\ [p]R & R\\end{bmatrix}$$

上半部只是把 $\\omega$ 轉成新座標；下半部除了轉動 $v$，左下角的 $[p]R$ 還加上 $p\\times\\omega_s$，也就是轉動帶給 $\\{s\\}$ 原點那一點的額外速度（力臂效應）。一般而言 $\\vV_a=[\\Ad_{T_{ab}}]\\,\\vV_b$，下標照樣相消。`,
    simHint: '面板中的 [Ad] 分塊上色：藍底是 R，橘底是 [p]R。左邊的 $\\mathcal V_s$ 由 $\\dot T T^{-1}$ 數值微分獨立算出，與 $[\\mathrm{Ad}]\\mathcal V_b$ 的差應該接近 0。改按「純平移」：ω = 0，橘色那塊就不再起作用。',
  },
  {
    title: '預告：每個 twist 都有一條螺旋軸',
    md: `只要 $\\omega\\neq 0$，任何 twist 都可以看成「繞空間中某一條直線轉動，同時沿著這條線滑動」，這條線就是**瞬時螺旋軸**。用 $\\{s\\}$ 的座標表示：

$$q=\\frac{\\omega\\times v}{\\|\\omega\\|^2},\\qquad h=\\frac{\\omega^{\\T}v}{\\|\\omega\\|^2}$$

軸的方向是 $\\hat\\omega$、通過點 $q$；節距 $h$ 是每轉一弧度沿軸前進的距離。$\\omega=0$ 時是純平移（$h=\\infty$）。本例的 $\\vV_b$ 是常數，所以軸在空間中固定不動，物體的路徑就是繞著它的螺旋線。下一課把它寫成螺旋軸 $\\mathcal S$ 並取指數，得到有限的剛體運動。`,
    simHint: '帶圓環的虛線是瞬時螺旋軸。播放時物體繞著它轉、沿著它滑；改變 $v_b$ 的 z 分量，看節距 h 與螺旋線的坡度一起改變。',
  },
];

const quiz: QuizQuestion[] = [
  {
    question: '物體繞著一條通過點 $q$、方向 $\\hat s$ 的軸以角速度 $\\dot\\theta$ 純轉動（沒有沿軸滑動），而且這條軸**不**通過 $\\{s\\}$ 的原點。它在 $\\{s\\}$ 中的 twist $\\vV_s$ 是？',
    options: [
      '$(\\hat s\\dot\\theta,\\ 0)$：純轉動沒有線速度',
      '$(\\hat s\\dot\\theta,\\ -\\hat s\\dot\\theta\\times q)$',
      '$(0,\\ -\\hat s\\dot\\theta\\times q)$',
      '$(\\hat s\\dot\\theta,\\ \\dot\\theta\\, q)$',
    ],
    answer: 1,
    explanation: '$v_s$ 是物體（延伸之後）此刻經過 $\\{s\\}$ 原點那一點的速度。那一點相對於軸上的 $q$ 位在 $0-q$，速度是 $\\omega\\times(0-q)=-\\hat s\\dot\\theta\\times q$。只有 $\\{s\\}$ 原點落在轉軸上時它才是 0，所以「沒有平移」不代表 $v_s=0$。',
  },
  {
    question: '物體上另外固定了一個座標系 $\\{c\\}$（$T_{bc}$ 不隨時間改變）。已知 $\\vV_b$，那麼 $\\{c\\}$ 的物體 twist $\\vV_c$ 是？',
    options: [
      '$\\vV_c=[\\Ad_{T_{bc}}]\\,\\vV_b$',
      '$\\vV_c=\\vV_b$：同一個物體只有一個 twist',
      '$\\vV_c=[\\Ad_{T_{cb}}]\\,\\vV_b$',
      '$\\vV_c=T_{bc}\\,\\vV_b$',
    ],
    answer: 2,
    explanation: '同一個運動換座標要用 Adjoint，而且下標要能相消：$\\vV_c=[\\Ad_{T_{cb}}]\\vV_b$，其中 $T_{cb}=T_{bc}^{-1}$。$\\omega_c$ 只是 $\\omega_b$ 換了座標，但 $v_c$ 是 $\\{c\\}$ 原點的速度，物體在轉動時它和 $\\{b\\}$ 原點的速度不同，所以兩個 twist 的數字不同。用 $T_{bc}$ 方向就反了，而 $4\\times4$ 的 $T$ 也不能直接乘六維向量。',
  },
  {
    question: '兩個物體在某一瞬間有相同的 $\\vV_s$，但它們的位姿不同。下列敘述何者正確？',
    options: [
      '這一瞬間兩者的速度場完全相同（像同一個剛體的兩部分，繞同一條螺旋軸），但 $\\vV_b$ 一般不同',
      '它們的 $\\vV_b$ 也一定相同',
      '只有角速度相同，各點的線速度可以完全不同',
      '$\\vV_s$ 只描述 $\\{s\\}$ 原點附近的運動，無法比較',
    ],
    answer: 0,
    explanation: '由 $\\vV_s$ 可以算出空間中任一點 $r$（屬於延伸後的物體）的速度 $\\omega_s\\times r+v_s$，所以兩個物體在這一瞬間的速度場、瞬時螺旋軸完全一樣。但 $\\vV_b=[\\Ad_{T_{bs}}]\\vV_s$ 取決於各自的位姿，位姿不同，用物體座標寫出的數字就不同。',
  },
  {
    question: '物體以 1 rad/s 繞 $\\hat z_s$ 軸（通過 $\\{s\\}$ 原點）轉動。此刻 $\\{b\\}$ 的姿態與 $\\{s\\}$ 相同（$R=I$），原點在 $p=(2,0,0)$ m。$v_b$ 是多少？',
    options: ['$(0,0,0)$：純轉動沒有線速度', '$(0,-2,0)$ m/s', '$(2,0,0)$ m/s', '$(0,2,0)$ m/s'],
    answer: 3,
    explanation: '$v_b$ 是 $\\{b\\}$ 原點的速度（用 $\\{b\\}$ 座標寫）。原點離轉軸 2 m，速度是 $\\omega\\times p=(0,0,1)\\times(2,0,0)=(0,2,0)$；$R=I$，所以數字不必再轉換。用 Adjoint 算也一樣：$\\vV_b=[\\Ad_{T_{bs}}]\\vV_s$ 的左下塊 $[-R^{\\T}p]R^{\\T}$ 正好給出這個力臂項。',
  },
];

// ---------------------------------------------------------------------------
// Simulation constants
// ---------------------------------------------------------------------------

const DEG = Math.PI / 180;
const pose = (x: number, y: number, z: number, yawDeg: number, pitchDeg: number, rollDeg: number): PoseParams =>
  ({ x, y, z, yaw: yawDeg * DEG, pitch: pitchDeg * DEG, roll: rollDeg * DEG });

/** Body: a box centred on {b}. P (step 1) sits on a short mast fixed to the box, so r_b is clearly visible. */
const BOX: [number, number, number] = [0.44, 0.24, 0.16];
const RB_P: Vec = [0.3, -0.21, 0.23];
const CORNERS: Vec[] = [[0.22, 0.12, 0.08], [0.22, -0.12, 0.08], [-0.22, -0.12, 0.08], [-0.22, 0.12, 0.08]];
/** Arrow lengths: metres of arrow per m/s and per rad/s. */
const VEL_SCALE = 0.6;
const W_SCALE = 0.45;
const ORIGIN: Vec = [0, 0, 0];

/** Default T_sb for steps 1–3 and T_bc for step 2. */
const SB_DEFAULT: PoseParams[] = [pose(0.85, 0.55, 0.35, 35, 0, -25), pose(0.8, 0.45, 0.3, 30, 0, -20), pose(0.9, 0.2, 0.3, 50, 0, 0)];
const BC_DEFAULT = pose(0.45, -0.15, 0.35, -60, 30, 0);

const FIXED_T: { label: string; title: string; T: Mat }[] = [
  { label: 'Rot(z, 90°)', title: '繞 z 軸轉 90°', T: RpToTrans(RotZ(Math.PI / 2), ORIGIN) },
  { label: 'Trans(x, 0.6)', title: '沿 x 軸平移 0.6 m', T: RpToTrans(eye(3), [0.6, 0, 0]) },
  { label: 'Rot(x, 90°)', title: '繞 x 軸轉 90°', T: RpToTrans(RotX(Math.PI / 2), ORIGIN) },
];

interface TwistPreset { label: string; title: string; Vb: Vec; start: PoseParams }
const TWISTS: TwistPreset[] = [
  { label: '轉彎前進', title: 'ω_b = (0, 0, 0.8)，v_b = (0.4, 0, 0)：像車子轉彎', Vb: [0, 0, 0.8, 0.4, 0, 0], start: pose(0.75, 0.05, 0.35, 20, 0, 0) },
  { label: '原地自轉', title: '繞自己的 z_b 軸轉，{b} 原點不動', Vb: [0, 0, 1, 0, 0, 0], start: pose(0.25, 0.55, 0.4, 30, 0, 0) },
  { label: '純平移', title: 'ω = 0：每一點的速度都相同', Vb: [0, 0, 0, 0.3, 0.15, 0.05], start: pose(0.1, -0.35, 0.3, 20, 0, 0) },
  { label: '螺旋上升', title: '一邊繞鉛直軸轉、一邊往上', Vb: [0, 0, 0.8, 0.35, 0, 0.06], start: pose(0.55, 0.2, 0.3, 0, 0, 0) },
  { label: '傾斜螺旋', title: '物體傾斜，ω_s 與 ω_b 的數字不同', Vb: [0, 0.3, 0.7, 0.25, 0, 0.08], start: pose(0.55, 0.25, 0.4, 30, 0, 20) },
];
/** Twist preset and {s} position used when entering steps 4–7. */
const TWIST_STEP: { preset: number; s: Vec }[] = [
  { preset: 0, s: [0, 0, 0] },
  { preset: 1, s: [0.85, -0.6, 0] },
  { preset: 4, s: [0, 0, 0] },
  { preset: 3, s: [0, 0, 0] },
];

/** Camera looking at `target` from distance d (azimuth −50°, elevation 27° by default). */
function camAt(target: Vec, d: number, azDeg = -50, elDeg = 27): { pos: Vec; target: Vec } {
  const az = azDeg * DEG, elv = elDeg * DEG;
  return { pos: [target[0] + d * Math.cos(elv) * Math.cos(az), target[1] + d * Math.cos(elv) * Math.sin(az), target[2] + d * Math.sin(elv)], target };
}
/** First-view camera of every step (the objects of each step are ~1 m across). */
const CAMS = [
  camAt([0.45, 0.35, 0.3], 2.1),
  camAt([0.55, 0.3, 0.35], 2.4),
  camAt([0.35, 0.45, 0.3], 2.4),
  camAt([0.55, 0.45, 0.35], 2.3),
  camAt([0.55, 0.0, 0.25], 2.5),
  camAt([0.4, 0.3, 0.55], 2.4),
  camAt([0.45, 0.45, 0.5], 2.4),
];

const COL_CLS = ['c303-x', 'c303-y', 'c303-z', 'c303-p'];
const tCls = (i: number, j: number): string => (i === 3 ? 'c303-dim' : COL_CLS[j] ?? '');
const blockCls = (i: number, j: number): string => (i < 3 ? (j < 3 ? 'c303-bR' : 'c303-b0') : (j < 3 ? 'c303-bPR' : 'c303-bR'));

const CSS = `
.c303 .c303-eq { display: flex; flex-wrap: wrap; align-items: center; gap: 0.25rem 0.5rem; margin: 0.35rem 0 0.15rem; font-size: 0.85rem; }
.c303 .c303-eq .mr-matrix { margin: 0; }
.c303 .c303-op { color: var(--fg-muted); font-size: 0.95rem; }
.c303 .c303-cap { display: inline-flex; flex-direction: column; align-items: center; gap: 0.05rem; }
.c303 .c303-capt { font-size: 0.76rem; color: var(--fg-muted); line-height: 1.25; white-space: nowrap; }
.c303 .c303-cap .mr-mlabel:empty { display: none; }
.c303 .c303-pill { position: absolute; left: 0.5rem; top: 0.4rem; font-size: 0.75rem; padding: 0.1rem 0.55rem; border-radius: 999px; pointer-events: none; max-width: calc(100% - 1rem); z-index: 1; }
.c303 .c303-pill:empty { display: none; }
.c303 .c303-pill.ok { background: var(--ok-soft); color: var(--ok); }
.c303 .c303-pill.bad { background: var(--danger-soft); color: var(--danger); }
.c303 .c303-pill.info { background: var(--accent-soft); color: var(--accent); }
.c303 .c303-hint { position: absolute; right: 0.5rem; bottom: 0.3rem; font-size: 0.72rem; color: var(--fg-muted); pointer-events: none; text-align: right; }
.c303 .mr-cell.c303-x { color: var(--axis-x); font-weight: 700; }
.c303 .mr-cell.c303-y { color: var(--axis-y); font-weight: 700; }
.c303 .mr-cell.c303-z { color: var(--axis-z); font-weight: 700; }
.c303 .mr-cell.c303-p { color: var(--accent); font-weight: 700; }
.c303 .mr-cell.c303-dim { color: var(--fg-muted); }
.c303 .mr-cell.c303-bad { color: var(--danger); font-weight: 700; }
.c303 .mr-cell.c303-ptS { color: var(--series-4); font-weight: 700; }
.c303 .mr-cell.c303-ptB { color: var(--series-6); font-weight: 700; }
.c303 .mr-cell.c303-w { color: var(--series-5); font-weight: 700; }
.c303 .mr-cell.c303-vb { color: var(--series-6); font-weight: 700; }
.c303 .mr-cell.c303-vs { color: var(--series-7); font-weight: 700; }
.c303 .c303-blk .mr-grid { gap: 0; }
.c303 .c303-blk .mr-cell { padding: 0.04rem 0.3rem; }
.c303 .mr-cell.c303-bR { background: var(--accent-soft); }
.c303 .mr-cell.c303-bPR { background: var(--warn-soft); color: var(--warn); font-weight: 700; }
.c303 .mr-cell.c303-b0 { color: var(--fg-muted); }
.c303 .mr-controls { padding-top: 0.35rem; }
.c303 .mr-legend { margin-top: 0.35rem; }
.c303 .mr-kv { margin: 0.3rem 0 0.1rem; }
.c303 .mr-note { margin: 0.25rem 0 0; }
`;

// ---------------------------------------------------------------------------
// mount
// ---------------------------------------------------------------------------

function mount(host: HTMLElement, ctx: LessonContext): LessonHandle {
  const S = {
    sc: -1,
    // transforms (steps 1–3)
    Psb: { ...SB_DEFAULT[0] },
    Pbc: { ...BC_DEFAULT },
    edit: 'sb' as 'sb' | 'bc',
    inv: 'negp' as InverseKind,
    fixed: 0,
    side: 'pre' as Side,
    u: 1,
    // twists (steps 4–7)
    Vb: TWISTS[0].Vb.slice(),
    Tstart: poseFromParams(TWISTS[0].start),
    T0: poseFromParams(TWISTS[0].start),
    Tcur: poseFromParams(TWISTS[0].start),
    t: 0,
    D: loopDuration(TWISTS[0].Vb),
    sPos: [0, 0, 0] as Vec,
  };
  let flashMsg = '';
  let flashTimer = 0;
  let pathKey = '';

  const root = el('div', { class: 'c303' });
  root.appendChild(el('style', {}, CSS));
  host.appendChild(root);
  simTitle(root, '齊次變換與旋量（twist）', '3D');

  // ---- 3D scene --------------------------------------------------------
  const narrow = host.clientWidth > 0 && host.clientWidth < 520;
  const cam0 = CAMS[0];
  const scene = new Scene3D(root, { aspect: narrow ? 1.2 : 1.7, grid: 3, axes: false, cameraPos: [cam0.pos[0], cam0.pos[1], cam0.pos[2]], target: [cam0.target[0], cam0.target[1], cam0.target[2]], autoStart: false });
  const pill = el('div', { class: 'c303-pill info' });
  const hint3d = el('div', { class: 'c303-hint' });
  scene.el.append(pill, hint3d);
  const add = <T extends THREE.Object3D>(o: T): T => { scene.scene.add(o); return o; };

  const frameS = new Frame3D({ size: 0.35, label: '{s}' });
  add(frameS.group);

  // the rigid body: translucent box + its frame {b}
  const boxGeo = new THREE.BoxGeometry(BOX[0], BOX[1], BOX[2]);
  const edgeMat = new THREE.LineBasicMaterial();
  setThemedColor(edgeMat, '--fg-muted');
  const bodyGroup = add(new THREE.Group());
  bodyGroup.matrixAutoUpdate = false;
  bodyGroup.add(
    new THREE.Mesh(boxGeo, themedStandardMaterial('--series-8', { transparent: true, opacity: 0.2, depthWrite: false, roughness: 0.8 })),
    new THREE.LineSegments(new THREE.EdgesGeometry(boxGeo), edgeMat),
  );
  const frameB = new Frame3D({ size: 0.3, label: '{b}' });
  bodyGroup.add(frameB.group);
  const dotMat = themedStandardMaterial('--series-4');
  const cornerDots = CORNERS.map((c) => {
    const m = new THREE.Mesh(new THREE.SphereGeometry(0.017, 12, 10), dotMat);
    m.position.set(c[0], c[1], c[2]);
    bodyGroup.add(m);
    return m;
  });
  const pDot = new THREE.Mesh(new THREE.SphereGeometry(0.03, 16, 12), dotMat);
  pDot.position.set(RB_P[0], RB_P[1], RB_P[2]);
  const mast = new Link3D(0.008, '--fg-muted');
  // anchored at a top corner so the mast does not hide the r_b arrow drawn from the {b} origin
  mast.setEndpoints([BOX[0] / 2, -BOX[1] / 2, BOX[2] / 2], RB_P);
  bodyGroup.add(pDot, mast.mesh);

  // ghost: the "other" result (step 3) or the rebuilt {s} (step 2)
  const ghostGroup = add(new THREE.Group());
  ghostGroup.matrixAutoUpdate = false;
  const ghostEdgeMat = new THREE.LineDashedMaterial({ dashSize: 0.03, gapSize: 0.02 });
  setThemedColor(ghostEdgeMat, '--fg-muted');
  const ghostEdges = new THREE.LineSegments(new THREE.EdgesGeometry(boxGeo), ghostEdgeMat);
  ghostEdges.computeLineDistances();
  const ghostBox = new THREE.Group();
  ghostBox.add(new THREE.Mesh(boxGeo, themedStandardMaterial('--series-8', { transparent: true, opacity: 0.08, depthWrite: false })), ghostEdges);
  const ghostFrame = new Frame3D({ size: 0.3, opacity: 0.38 });
  ghostGroup.add(ghostBox, ghostFrame.group);

  const frameC = new Frame3D({ size: 0.25, label: '{c}' });
  add(frameC.group);
  const origFrame = new Frame3D({ size: 0.22, opacity: 0.45 });
  add(origFrame.group);

  const arrow = (color: string, r = 0.01, head = 0.07): Arrow3D => { const a = new Arrow3D(color, r, head); add(a.group); return a; };
  const aP = arrow('--fg-muted', 0.009, 0.06);
  const aRs = arrow('--series-4', 0.008, 0.06);
  const aRb = arrow('--series-6', 0.008, 0.06);
  const aBC = arrow('--fg-muted', 0.008, 0.06);
  const aSC = arrow('--series-5', 0.01, 0.07);
  const aInv = arrow('--ok', 0.009, 0.06);
  const aW = arrow('--series-5', 0.014, 0.08);
  const aV = arrow('--series-6', 0.012, 0.07);
  const aCorner = CORNERS.map(() => arrow('--series-4', 0.007, 0.05));
  const aVs = arrow('--series-7', 0.013, 0.08);

  const extLine = scene.addPolyline([ORIGIN, ORIGIN], '--fg-muted', { dashed: true });
  const pathMain = scene.addTube([], { radius: 0.007, color: '--series-4' });
  const pathAlt = scene.addPolyline([ORIGIN, ORIGIN], '--fg-muted', { dashed: true });
  const trail = scene.addTube([], { radius: 0.006, color: '--series-8', opacity: 0.8 });
  const axis = new ScrewAxis3D({ color: '--fg', length: 1.6, size: 0.8 });
  add(axis.group);

  // draggable {s} origin (steps 5–7)
  const sHandle = add(new THREE.Group());
  const sHandleMat = themedStandardMaterial('--series-7');
  sHandle.add(new THREE.Mesh(new THREE.TorusGeometry(0.13, 0.012, 10, 40), sHandleMat), new THREE.Mesh(new THREE.SphereGeometry(0.03, 14, 12), sHandleMat));

  const L = {
    P: label3D(scene, 'P', '--series-4'),
    rs: label3D(scene, 'r_s', '--series-4'),
    rb: label3D(scene, 'r_b', '--series-6'),
    p: label3D(scene, 'p', '--fg-muted'),
    sc: label3D(scene, 's→c', '--series-5'),
    ghostInv: label3D(scene, 'T_sb·X', '--fg-muted'),
    ghostPre: label3D(scene, '前乘 T·T_sb', '--fg-muted'),
    ghostPost: label3D(scene, '後乘 T_sb·T', '--fg-muted'),
    orig: label3D(scene, '原本的 {b}', '--fg-muted'),
    w: label3D(scene, 'ω', '--series-5'),
    v: label3D(scene, 'v_b', '--series-6'),
    vs: label3D(scene, 'v_s', '--series-7'),
    axis: label3D(scene, '螺旋軸', '--fg'),
  };

  const legendA = legend(root, [
    { color: 'var(--fg-muted)', label: 'p：{b} 原點的位置' },
    { color: 'var(--series-4)', label: 'r_s：P 在 {s} 中' },
    { color: 'var(--series-6)', label: 'r_b：P 在 {b} 中' },
  ]);
  const legendB = legend(root, [
    { color: 'var(--series-5)', label: 'ω' },
    { color: 'var(--series-6)', label: 'v_b：{b} 原點的速度' },
    { color: 'var(--series-4)', label: '角落的速度' },
    { color: 'var(--series-7)', label: 'v_s：經過 {s} 原點那一點的速度' },
  ]);

  // ---- transforms panel (steps 1–3) -------------------------------------
  const framesBox = el('div');
  root.appendChild(framesBox);
  const fPanel = controlPanel(framesBox, { compact: true });
  const fTop = row(fPanel);
  const editSel = select<'sb' | 'bc'>(fTop, {
    label: '滑桿編輯',
    options: [{ value: 'sb', label: 'T_sb（{b} 相對於 {s}）' }, { value: 'bc', label: 'T_bc（{c} 相對於 {b}）' }],
    value: 'sb',
    onChange: (v) => { S.edit = v; syncPoseSliders(); poseChips.clear(); update(); },
  });
  const poseChips = presetChips(fTop, [
    { label: '預設位姿', title: '本步驟的預設值', apply: () => setEdited(S.edit === 'sb' ? SB_DEFAULT[Math.max(0, Math.min(2, S.sc))] : BC_DEFAULT) },
    { label: '純平移（R = I）', title: '三個角度歸零，只留位置', apply: () => setEdited({ ...edited(), yaw: 0, pitch: 0, roll: 0 }) },
    { label: '純旋轉（p = 0）', title: '位置歸零，只留姿態', apply: () => setEdited({ ...edited(), x: 0, y: 0, z: 0 }) },
  ]);
  const setField = (k: keyof PoseParams) => (v: number) => { edited()[k] = v; poseChips.clear(); update(); };
  const poseSl: Record<keyof PoseParams, SliderHandle> = {
    x: slider(fPanel, { label: 'x', min: -1.2, max: 1.2, step: 0.01, value: 0, unit: 'm', onInput: setField('x') }),
    y: slider(fPanel, { label: 'y', min: -1.2, max: 1.2, step: 0.01, value: 0, unit: 'm', onInput: setField('y') }),
    z: slider(fPanel, { label: 'z', min: -1.2, max: 1.2, step: 0.01, value: 0, unit: 'm', onInput: setField('z') }),
    yaw: angleSlider(fPanel, { label: '繞 z 轉 α', value: 0, onInput: setField('yaw') }),
    pitch: angleSlider(fPanel, { label: '繞 y 轉 β', value: 0, min: -Math.PI / 2, max: Math.PI / 2, onInput: setField('pitch') }),
    roll: angleSlider(fPanel, { label: '繞 x 轉 γ', value: 0, onInput: setField('roll') }),
  };

  const invPanel = controlPanel(framesBox, { compact: true });
  const invSel = select<InverseKind>(row(invPanel), {
    label: '反矩陣候選 X（正確的 X 應讓 T_sb·X = I）',
    options: [
      { value: 'negp', label: '[Rᵀ, −p]：只把 p 變號（常見錯誤）' },
      { value: 'transpose', label: 'Tᵀ：整個 4×4 轉置' },
      { value: 'correct', label: '[Rᵀ, −Rᵀp]：正確的反矩陣' },
    ],
    value: 'negp',
    onChange: (v) => { S.inv = v; update(); },
  });

  const ppPanel = controlPanel(framesBox, { compact: true });
  const fixedChips = presetChips(row(ppPanel), FIXED_T.map((f, k) => ({ label: f.label, title: f.title, apply: () => { S.fixed = k; update(); } })), { label: '固定的 T：' });
  const sideSel = select<Side>(ppPanel, {
    label: '乘在哪一邊',
    options: [{ value: 'pre', label: '前乘 T·T_sb' }, { value: 'post', label: '後乘 T_sb·T' }],
    value: 'pre',
    onChange: (v) => { S.side = v; update(); },
  });
  const uSl = slider(ppPanel, { label: '套用比例 u', min: 0, max: 1, step: 0.01, value: 1, onInput: (v) => { animLoop.stop(); S.u = v; update(); } });
  button(row(ppPanel), { label: '▶ 動畫（u 從 0 到 1）', small: true, onClick: () => { S.u = 0; animLoop.start(); } });

  // equation rows
  const eq1 = eqRow(framesBox);
  const mRs = captioned(eq1, 'r_s', { cls: (i) => (i < 3 ? 'c303-ptS' : 'c303-dim') });
  tex(eq1, '=');
  const mTsb1 = captioned(eq1, 'T_{sb}', { cls: tCls });
  const mRb = captioned(eq1, 'r_b', { cls: (i) => (i < 3 ? 'c303-ptB' : 'c303-dim') });

  const eq2 = eqRow(framesBox);
  const mTsc = captioned(eq2, 'T_{sc}=T_{sb}T_{bc}', { cls: tCls });
  let xValid = true;
  let lastTX: Mat = eye(4);
  const mX = captioned(eq2, 'X', { note: '（候選）', cls: (i, j) => (!xValid && i === 3 ? 'c303-bad' : tCls(i, j)) });
  const mTX = captioned(eq2, 'T_{sb}X', { note: '（應為 I）', cls: (i, j) => (Math.abs(lastTX[i][j] - (i === j ? 1 : 0)) > 1e-6 ? 'c303-bad' : 'c303-dim') });

  const eq3 = eqRow(framesBox);
  const mT = captioned(eq3, 'T', { cls: tCls });
  const mRes = captioned(eq3, 'T\\,T_{sb}', { cls: tCls });
  const noteA = note(framesBox, '姿態由三個角度組成：R = Rot(z, α)·Rot(y, β)·Rot(x, γ)。拖曳 3D 畫面可轉動視角、滾輪縮放。');

  // ---- twist panel (steps 4–7) -------------------------------------------
  const twistBox = el('div');
  root.appendChild(twistBox);
  const tPanel = controlPanel(twistBox, { compact: true });
  const twistChips = presetChips(row(tPanel), TWISTS.map((p, k) => ({ label: p.label, title: p.title, apply: () => applyTwistPreset(k) })), { label: '運動：' });
  const vbPanel = controlPanel(twistBox, { compact: true });
  const VB_LABELS = ['ω_b（x）', 'ω_b（y）', 'ω_b（z）', 'v_b（x）', 'v_b（y）', 'v_b（z）'];
  const vbSl = VB_LABELS.map((lab, k) => slider(vbPanel, {
    label: lab, min: k < 3 ? -1.5 : -0.8, max: k < 3 ? 1.5 : 0.8, step: k < 3 ? 0.05 : 0.01, value: S.Vb[k], unit: k < 3 ? 'rad/s' : 'm/s',
    onInput: (v) => { S.Vb[k] = v; twistChips.clear(); reanchor(); update(); },
  }));
  const pbPanel = controlPanel(twistBox, { compact: true });
  const pb = playback(row(pbPanel), {
    duration: S.D,
    onPlay: () => twistLoop.start(),
    onPause: () => twistLoop.stop(),
    onReset: () => { S.T0 = S.Tstart; S.t = 0; pb.setTime(0); rebuildTrail(); update(); },
    onSpeed: (s) => { twistLoop.speed = s; },
    onScrub: (t) => { S.t = t; update(); },
  });
  const sPanel = controlPanel(twistBox, { compact: true });
  const sSl = [0, 1, 2].map((k) => slider(sPanel, {
    label: `{s} 原點 ${'xyz'[k]}`, min: k === 2 ? -0.5 : -1.5, max: 1.5, step: 0.01, value: 0, unit: 'm',
    onInput: (v) => { S.sPos[k] = v; update(); },
  }));
  const sRow = row(sPanel);
  button(sRow, { label: '把 {s} 放到轉軸上', small: true, onClick: () => moveSOntoAxis() });
  button(sRow, { label: '{s} 回到房間原點', small: true, onClick: () => { S.sPos = [0, 0, 0]; syncS(); update(); } });

  const vecBox = el('div');
  twistBox.appendChild(vecBox);
  const mVb = cellMatrix(eqRow(vecBox), { label: '\\mathcal V_b^{\\mathsf T}=', digits: 3, cls: (_i, j) => (j < 3 ? 'c303-w' : 'c303-vb') });
  const rowVs = eqRow(vecBox);
  const mVs = cellMatrix(rowVs, { label: '\\mathcal V_s^{\\mathsf T}=', digits: 3, cls: (_i, j) => (j < 3 ? 'c303-w' : 'c303-vs') });

  const adBox = el('div');
  twistBox.appendChild(adBox);
  const adRow = eqRow(adBox, 'c303-blk');
  const mVsNum = captioned(adRow, '\\mathcal V_s', { note: '（數值 ṪT⁻¹）', digits: 3, cls: (i) => (i < 3 ? 'c303-w' : 'c303-vs') });
  tex(adRow, '=');
  const mAd = captioned(adRow, '[\\mathrm{Ad}_{T_{sb}}]', { digits: 2, cls: blockCls });
  const mVb2 = captioned(adRow, '\\mathcal V_b', { digits: 3, cls: (i) => (i < 3 ? 'c303-w' : 'c303-vb') });
  const errRO = readout(adBox, { label: '‖ 數值 V_s − [Ad]V_b ‖ =', minWidth: '8ch' });

  const screwBox = el('div');
  twistBox.appendChild(screwBox);
  const KV = { kind: '運動類型', w: '‖ω‖', h: '節距 h', q: '軸上一點 q（{s} 座標）', dir: '軸的方向' };
  const screwKv = kv(screwBox, Object.values(KV));
  const noteB = note(twistBox, '格線只是房間；{s} 固定不動，但可以放在房間的任何位置。');

  // ---- animation loops -------------------------------------------------
  const animLoop = createLoop((dt) => {
    S.u = Math.min(1, S.u + dt / 1.6);
    uSl.set(S.u, { silent: true });
    update();
    if (S.u >= 1) animLoop.stop();
  });
  const twistLoop = createLoop((dt) => {
    S.t += dt;
    if (S.t >= S.D) S.t -= S.D * Math.floor(S.t / S.D);
    update();
    pb.setTime(S.t);
  });

  // {s} can be dragged on the horizontal plane through its origin
  const drag = attachDragHandles(scene, [{
    pos: () => (S.sc >= 4 ? new THREE.Vector3(S.sPos[0], S.sPos[1], S.sPos[2]) : null),
    drag: (ray) => {
      const p = rayPlaneZ(ray, S.sPos[2]);
      if (!p) return;
      S.sPos[0] = clamp(p.x, -1.5, 1.5);
      S.sPos[1] = clamp(p.y, -1.5, 1.5);
      syncS();
      update();
    },
  }]);

  // ---- helpers -----------------------------------------------------------
  function edited(): PoseParams { return S.edit === 'sb' ? S.Psb : S.Pbc; }
  function setEdited(P: PoseParams): void {
    if (S.edit === 'sb') S.Psb = { ...P }; else S.Pbc = { ...P };
    syncPoseSliders();
    update();
  }
  function syncPoseSliders(): void {
    const P = edited();
    const lim = S.edit === 'sb' ? 1.2 : 0.8;
    for (const k of ['x', 'y', 'z'] as const) { poseSl[k].setRange(-lim, lim, 0.01); poseSl[k].set(P[k], { silent: true }); }
    for (const k of ['yaw', 'pitch', 'roll'] as const) poseSl[k].set(P[k], { silent: true });
  }
  function syncS(): void { sSl.forEach((s, k) => s.set(S.sPos[k], { silent: true })); }

  function place(a: Arrow3D, from: Vec, to: Vec, on = true): void {
    a.setFromTo(from, to);
    a.visible = on && norm(vecSub(to, from)) > 1e-6;
  }
  function putLabel(s: THREE.Sprite, p: Vec, on = true, dz = 0.07): void {
    s.visible = on;
    if (on) s.position.set(p[0], p[1], p[2] + dz);
  }
  const mid = (a: Vec, b: Vec): Vec => vecScale(vecAdd(a, b), 0.5);
  const tip = (from: Vec, d: Vec, extra = 0.08): Vec => {
    const n = norm(d);
    return n < 1e-9 ? from : vecAdd(from, vecScale(d, 1 + extra / n));
  };
  function setPill(text: string, kind: 'info' | 'ok' | 'bad' = 'info'): void {
    if (pill.textContent !== text) pill.textContent = text;
    const cn = `c303-pill ${kind}`;
    if (pill.className !== cn) pill.className = cn;
  }
  function flash(msg: string): void {
    flashMsg = msg;
    if (flashTimer) clearTimeout(flashTimer);
    flashTimer = window.setTimeout(() => { flashTimer = 0; flashMsg = ''; update(); }, 2600);
    update();
  }
  function setCamera(c: { pos: Vec; target: Vec }): void {
    scene.camera.position.set(c.pos[0], c.pos[1], c.pos[2]);
    scene.controls?.target.set(c.target[0], c.target[1], c.target[2]);
    scene.controls?.update();
  }
  const show = (e: HTMLElement, on: boolean) => { const d = on ? '' : 'none'; if (e.style.display !== d) e.style.display = d; };

  // ---- twist motion ----------------------------------------------------
  function twistPose(t: number): Mat { return constantBodyTwistPose(S.T0, S.Vb, t); }
  function rebuildTrail(): void { trail.update(bodyPointPath(S.T0, S.Vb, ORIGIN, S.D, 120)); }
  /** Keep the body where it is when V_b changes: the current pose becomes the new start of the loop. */
  function reanchor(): void {
    const p = [S.Tcur[0][3], S.Tcur[1][3], S.Tcur[2][3]];
    const inView = Math.abs(p[0]) < 1.6 && Math.abs(p[1]) < 1.6 && p[2] > -0.3 && p[2] < 1.7;
    S.T0 = inView ? S.Tcur : S.Tstart;
    S.t = 0;
    S.D = loopDuration(S.Vb);
    pb.setDuration(S.D);
    pb.setTime(0);
    rebuildTrail();
  }
  function applyTwistPreset(k: number): void {
    const P = TWISTS[k];
    S.Vb = P.Vb.slice();
    S.Tstart = poseFromParams(P.start);
    S.T0 = S.Tstart;
    S.Tcur = S.Tstart;
    S.t = 0;
    S.D = loopDuration(S.Vb);
    pb.setDuration(S.D);
    pb.setTime(0);
    vbSl.forEach((s, i) => s.set(S.Vb[i], { silent: true }));
    rebuildTrail();
    update();
  }
  function moveSOntoAxis(): void {
    const info = screwInfo(changeFrame(S.Tcur, S.Vb));
    if (info.kind !== 'screw') { flash('這個運動沒有轉動（ω = 0）：沒有轉軸，v_s 在任何位置都一樣。'); return; }
    const c = closestPointOnLine(info.q, info.dir, S.sPos);
    S.sPos = [clamp(c[0], -1.5, 1.5), clamp(c[1], -1.5, 1.5), clamp(c[2], -0.5, 1.5)];
    syncS();
    update();
  }

  // ---- per-frame updates -------------------------------------------------
  function hideAll(): void {
    for (const a of [aP, aRs, aRb, aBC, aSC, aInv, aW, aV, aVs, ...aCorner]) a.visible = false;
    for (const s of Object.values(L)) s.visible = false;
    ghostGroup.visible = false;
    frameC.setVisible(false);
    origFrame.setVisible(false);
    extLine.line.visible = false;
    pathMain.mesh.visible = false;
    pathAlt.line.visible = false;
    trail.mesh.visible = false;
    axis.setVisible(false);
    sHandle.visible = false;
  }

  function updateFrames(): void {
    const sc = S.sc;
    frameS.setTransform(eye(4));
    const Tsb = poseFromParams(S.Psb);
    const p: Vec = [Tsb[0][3], Tsb[1][3], Tsb[2][3]];
    pDot.visible = sc === 0;
    mast.mesh.visible = sc === 0;
    cornerDots.forEach((d) => { d.visible = false; });

    if (sc === 0) {
      setMatrix(bodyGroup, Tsb);
      const rs = transformPoint(Tsb, RB_P);
      place(aP, ORIGIN, p);
      place(aRs, ORIGIN, rs);
      place(aRb, p, rs);
      putLabel(L.P, rs, true, 0.09);
      putLabel(L.rs, mid(ORIGIN, rs));
      putLabel(L.rb, vecAdd(p, vecScale(vecSub(rs, p), 0.6)), true, -0.07);
      putLabel(L.p, mid(ORIGIN, p), true, -0.06);
      mRs.set([rs[0], rs[1], rs[2], 1]);
      mTsb1.set(Tsb);
      mRb.set([RB_P[0], RB_P[1], RB_P[2], 1]);
      setPill(`r_s = R·r_b + p = ${fmtVec(rs, 2)} m`);
      return;
    }

    if (sc === 1) {
      setMatrix(bodyGroup, Tsb);
      const Tbc = poseFromParams(S.Pbc);
      const Tsc = matMul(Tsb, Tbc);
      const pc: Vec = [Tsc[0][3], Tsc[1][3], Tsc[2][3]];
      frameC.setTransform(Tsc);
      frameC.setVisible(true);
      place(aP, ORIGIN, p);
      place(aBC, p, pc);
      place(aSC, ORIGIN, pc);
      putLabel(L.p, mid(ORIGIN, p), true, -0.06);
      putLabel(L.sc, mid(ORIGIN, pc), true, 0.08);
      const X = inverseCandidate(Tsb, S.inv);
      xValid = hasHomogeneousRow(X);
      lastTX = matMul(Tsb, X);
      if (xValid) {
        ghostGroup.visible = true;
        ghostBox.visible = false;
        setMatrix(ghostGroup, lastTX);
        const g: Vec = [lastTX[0][3], lastTX[1][3], lastTX[2][3]];
        const ok = distanceFromIdentity(lastTX) < 1e-9;
        aInv.setColor(ok ? '--ok' : '--danger');
        place(aInv, p, g);
        putLabel(L.ghostInv, g, true, -0.12);
        if (ok) setPill('T_sb·X = I：半透明座標系疊回 {s}，X 確實是反矩陣 ✓', 'ok');
        else setPill(`T_sb·X ≠ I：重建出來的 {s} 偏離原點 ${fmt(norm(g), 2)} m，X 不是反矩陣 ✗`, 'bad');
      } else {
        setPill('Tᵀ 的最後一列變成 [pᵀ 1]：它根本不是齊次變換 ✗', 'bad');
      }
      mTsc.set(Tsc);
      mX.set(X);
      mTX.set(lastTX);
      return;
    }

    // sc === 2: pre / post multiplication
    const T = FIXED_T[S.fixed].T;
    const other: Side = S.side === 'pre' ? 'post' : 'pre';
    const Tres = applyFraction(T, Tsb, S.u, S.side);
    const Tghost = applyFraction(T, Tsb, S.u, other);
    setMatrix(bodyGroup, Tres);
    ghostGroup.visible = true;
    ghostBox.visible = true;
    setMatrix(ghostGroup, Tghost);
    origFrame.setTransform(Tsb);
    origFrame.setVisible(true);
    putLabel(L.orig, p, true, -0.16);
    const g: Vec = [Tghost[0][3], Tghost[1][3], Tghost[2][3]];
    putLabel(S.side === 'pre' ? L.ghostPost : L.ghostPre, g, true, 0.26);
    const key = `${S.fixed}|${S.side}|${Object.values(S.Psb).join(',')}`;
    if (key !== pathKey) {
      pathKey = key;
      pathMain.update(originPath(T, Tsb, S.side));
      pathAlt.update(originPath(T, Tsb, other));
    }
    pathMain.mesh.visible = true;
    pathAlt.line.visible = true;
    const S6 = se3ToVec(MatrixLog6(T));
    const rotates = norm(S6.slice(0, 3)) > 1e-9;
    axis.setVisible(rotates);
    if (rotates) axis.setScrew(S.side === 'pre' ? S6 : matVec(Adjoint(Tsb), S6));
    mT.set(T);
    mRes.setCaption(S.side === 'pre' ? 'T^u\\,T_{sb}' : 'T_{sb}\\,T^u', `（u = ${fmt(S.u, 2)}）`);
    mRes.set(Tres);
    setPill(S.side === 'pre' ? '前乘：T 的軸以固定的 {s} 來解讀' : '後乘：T 的軸以物體自己的 {b} 來解讀');
  }

  function updateTwist(): void {
    const sc = S.sc;
    const Twb = twistPose(S.t);
    S.Tcur = Twb;
    setMatrix(bodyGroup, Twb);
    pDot.visible = false;
    mast.mesh.visible = false;
    const Tws = RpToTrans(eye(3), S.sPos);
    frameS.setTransform(Tws);
    const Tsw = TransInv(Tws);
    const Tsb = matMul(Tsw, Twb);
    const Vb = S.Vb;
    const Vs = changeFrame(Tsb, Vb);
    const Vw = changeFrame(Twb, Vb); // world axes are parallel to {s}, so the numbers only differ in v
    const pb0: Vec = [Twb[0][3], Twb[1][3], Twb[2][3]];
    const w = Vw.slice(0, 3);
    const wn = norm(w);

    // body: ω at the origin of {b}, velocity of the {b} origin, corner velocities
    place(aW, pb0, vecAdd(pb0, vecScale(w, W_SCALE)), wn > 1e-6);
    putLabel(L.w, tip(pb0, vecScale(w, W_SCALE)), wn > 1e-6, 0.02);
    const vOrg = pointVelocity(Vw, pb0);
    place(aV, pb0, vecAdd(pb0, vecScale(vOrg, VEL_SCALE)));
    putLabel(L.v, tip(pb0, vecScale(vOrg, VEL_SCALE)), norm(vOrg) > 1e-6, 0.02);
    const showCorners = sc === 3 || sc === 4;
    CORNERS.forEach((c, k) => {
      const r = transformPoint(Twb, c);
      place(aCorner[k], r, vecAdd(r, vecScale(pointVelocity(Vw, r), VEL_SCALE)), showCorners);
      cornerDots[k].visible = showCorners;
    });

    // {s}: the velocity of the (extended) body point passing through its origin
    const showS = sc >= 4;
    const vsW = pointVelocity(Vw, S.sPos);
    place(aVs, S.sPos, vecAdd(S.sPos, vecScale(vsW, VEL_SCALE)), showS);
    putLabel(L.vs, tip(S.sPos, vecScale(vsW, VEL_SCALE)), showS && norm(vsW) > 1e-6, 0.02);
    extLine.update([S.sPos, pb0]);
    extLine.line.visible = showS && sc !== 5;
    sHandle.visible = showS;
    sHandle.position.set(S.sPos[0], S.sPos[1], S.sPos[2]);

    trail.mesh.visible = true;
    const info = screwInfo(Vw);
    const showAxis = (sc === 4 || sc === 6) && info.kind === 'screw';
    axis.setVisible(showAxis);
    if (showAxis && info.kind === 'screw') {
      axis.setScrew(Vw);
      const anchor = closestPointOnLine(info.q, info.dir, pb0);
      putLabel(L.axis, vecAdd(anchor, vecScale(info.dir, 0.6)), true, 0);
    }

    // numbers
    mVb.set([Vb]);
    mVs.set([Vs]);
    if (sc === 5) {
      const f = (t: number) => matMul(Tsw, twistPose(t));
      const { Vs: VsNum } = twistsFromDerivative(Tsb, numericDerivative(f, S.t));
      mVsNum.set(VsNum);
      mAd.set(Adjoint(Tsb));
      mVb2.set(Vb);
      const err = norm(vecSub(VsNum, matVec(Adjoint(Tsb), Vb)));
      errRO.set(err.toExponential(1));
      setPill(`數值微分的 V_s 與 [Ad]·V_b 相差 ${err.toExponential(1)}（只剩有限差分的誤差）`, 'ok');
    } else if (sc === 6) {
      const si = screwInfo(Vs);
      if (si.kind === 'screw') {
        screwKv.set(KV.kind, si.h === 0 || Math.abs(si.h) < 1e-9 ? '純轉動（h = 0）' : '螺旋運動');
        screwKv.set(KV.w, `${fmt(si.rate, 3)} rad/s`);
        screwKv.set(KV.h, `${fmt(si.h, 3)} m/rad`);
        screwKv.set(KV.q, fmtVec(si.q, 2));
        screwKv.set(KV.dir, fmtVec(si.dir, 2));
        setPill(`節距 h = ${fmt(si.h, 3)} m/rad：每轉一圈沿軸前進 ${fmt(2 * Math.PI * si.h, 2)} m`);
      } else if (si.kind === 'translation') {
        screwKv.set(KV.kind, '純平移（ω = 0）');
        screwKv.set(KV.w, '0 rad/s');
        screwKv.set(KV.h, '∞');
        screwKv.set(KV.q, '—（沒有轉軸）');
        screwKv.set(KV.dir, `沿 ${fmtVec(si.dir, 2)}`);
        setPill('ω = 0：純平移，節距 h = ∞（沒有轉軸）');
      } else {
        for (const k of Object.values(KV)) screwKv.set(k, '—');
        setPill('V = 0：物體靜止');
      }
    } else if (sc === 4) {
      const vsn = norm(Vs.slice(3, 6));
      setPill(vsn < 1e-6 ? '{s} 原點在轉軸上：那一點不動，所以 v_s = 0' : `‖v_b‖ = ${fmt(norm(Vb.slice(3, 6)), 2)} m/s，但 ‖v_s‖ = ${fmt(vsn, 2)} m/s`, vsn < 1e-6 ? 'ok' : 'info');
    } else {
      setPill(`‖ω‖ = ${fmt(wn, 2)} rad/s，{b} 原點速率 ${fmt(norm(vOrg), 2)} m/s`);
    }
    if (flashMsg) setPill(flashMsg, 'bad');
  }

  function update(): void {
    hideAll();
    if (S.sc <= 2) updateFrames(); else updateTwist();
    scene.requestRender();
  }

  function syncVisibility(): void {
    const sc = S.sc;
    const frames = sc <= 2;
    show(framesBox, frames);
    show(twistBox, !frames);
    show(legendA, sc === 0);
    show(legendB, !frames);
    const items = legendB.children;
    if (items[2]) show(items[2] as HTMLElement, sc === 3 || sc === 4); // corner velocities
    if (items[3]) show(items[3] as HTMLElement, sc >= 4); // v_s
    show(editSel.el, sc === 1);
    show(invPanel, sc === 1);
    show(ppPanel, sc === 2);
    show(eq1, sc === 0);
    show(eq2, sc === 1);
    show(eq3, sc === 2);
    show(noteA, frames);
    show(sPanel, sc === 4);
    show(vecBox, sc === 3 || sc === 4 || sc === 6);
    show(rowVs, sc !== 3);
    show(adBox, sc === 5);
    show(screwBox, sc === 6);
    show(noteB, sc >= 4);
    hint3d.textContent = sc >= 4 ? '拖曳粉紅圓環：移動 {s}　拖曳空白處：轉視角' : '拖曳：轉視角　滾輪：縮放';
    const focus: HTMLElement[] = [eq1, invPanel, ppPanel, vbPanel, sPanel, adBox, screwBox];
    focus.forEach((f, k) => f.classList.toggle('mr-focus', k === sc));
  }

  function applyStep(i: number): void {
    animLoop.stop();
    twistLoop.stop();
    pb.setPlaying(false, { silent: true });
    if (flashTimer) { clearTimeout(flashTimer); flashTimer = 0; }
    flashMsg = '';
    const prev = S.sc;
    S.sc = Math.max(0, Math.min(steps.length - 1, i));
    if (S.sc <= 2) {
      S.Psb = { ...SB_DEFAULT[S.sc] };
      S.Pbc = { ...BC_DEFAULT };
      S.edit = 'sb';
      editSel.set('sb', { silent: true });
      syncPoseSliders();
      poseChips.highlight(0);
      S.inv = 'negp';
      invSel.set('negp', { silent: true });
      S.fixed = 0;
      fixedChips.highlight(0);
      S.side = 'pre';
      sideSel.set('pre', { silent: true });
      S.u = 1;
      uSl.set(1, { silent: true });
      pathKey = '';
    } else {
      const cfg = TWIST_STEP[S.sc - 3];
      S.sPos = cfg.s.slice();
      syncS();
      twistChips.activate(cfg.preset);
    }
    if (S.sc !== prev) setCamera(CAMS[S.sc]);
    syncVisibility();
    update();
  }

  applyStep(ctx.currentStep);

  return {
    onStep: applyStep,
    destroy() {
      animLoop.stop();
      twistLoop.stop();
      if (flashTimer) clearTimeout(flashTimer);
      drag.destroy();
      disposeSprites(scene.scene);
      scene.destroy();
      root.remove();
    },
  };
}

const lesson: Lesson = { meta, steps, quiz, mount };
export default lesson;
