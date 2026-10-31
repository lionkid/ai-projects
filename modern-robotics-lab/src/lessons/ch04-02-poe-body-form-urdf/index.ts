import type { Lesson, LessonContext, LessonHandle, LessonStep, QuizQuestion } from '@/types';
import * as THREE from 'three';
import {
  el, fmt, controlPanel, row, angleSlider, toggle, button, matrixView, readout, simTitle, note, kv, mathEl,
  presetChips, subscript, setThemedColor, themedStandardMaterial, Scene3D, Link3D, Arrow3D, createLoop,
  type SliderHandle,
} from '@/ui';
import { type Mat, type Vec, FKinBody, FKinSpace, TransInv, eye, getCol, matMul, ur5 } from '@/mr';
import { meta } from './meta';
import { BOOK_P, POSES, TOOL_LENGTH, TOOL_SEGMENT, UR5_ANCHORS, UR5_CAMERA, UR5_SEGMENTS } from './ur5';
import {
  axisPlacement, bodyAxes, bodyPartial, carry, chainTransforms, changedColumns, currentAxes, gluedToB,
  passingProduct, pointVelocity, toolFrame, transformGap,
} from './body';
import { UR5_URDF, homeJointFrames, screwFromFrame, urdfForward, urdfXml } from './urdf';
import { AxisGlyph, ChainView, disposeObject, makeFrame, makeGrid, textSprite } from './view3d';
import { type FactorState, circled, factorStrip, fmt3, fmtSci, screwTable } from './panels';

// ---------------------------------------------------------------------------
// Narrative
// ---------------------------------------------------------------------------

const steps: LessonStep[] = [
  {
    title: '同一條鏈，從末端看',
    md: `同一支手臂也可以從末端這一側描述。在零位姿時，把每條關節軸改用**末端座標系** $\\{b\\}$（此時就是 $M$）的座標來寫，得到 $\\vB_i$。換參考座標系的工具正是 Adjoint：

$$\\vB_i = [\\Ad_{M^{-1}}]\\,\\vS_i$$

$\\vS_i$ 與 $\\vB_i$ 指的是同一條線：$\\omega$ 部分只是方向換了座標；$v$ 部分則取決於軸相對**哪個原點**，$\\vS_i$ 以 $\\{s\\}$ 原點量、$\\vB_i$ 以 $\\{b\\}$ 原點量。手臂一動，$\\vS_i$ 留在 $\\{s\\}$ 裡不動，$\\vB_i$ 則黏著 $\\{b\\}$ 一起走。`,
    simHint: '手臂在零位姿，每條軸同時是 Sᵢ（橘，以 {s} 量）與 Bᵢ（紫，以 {b} 量）。點表格欄標題選關節，比較從 {s} 拉出的 q 與從 {b} 拉出的 qᵇ；再拖動 θ₁，看紫色軸跟著 {b} 走。',
  },
  {
    title: '為什麼可以把 M 移到最前面',
    md: `關鍵是一個交換位置的恆等式。對任何可逆的 $M$ 都有 $M^{-1}e^{A}M = e^{M^{-1}AM}$，因此

$$e^{[\\vS]\\theta}M = M\\,e^{M^{-1}[\\vS]M\\,\\theta} = M\\,e^{[\\vB]\\theta}$$

而 $M^{-1}[\\vS]M$ 正是 $[\\Ad_{M^{-1}}]\\vS$ 寫成矩陣的樣子。從最右邊的因子開始反覆套用：$M$ 每往左跨過一個因子，那個 $\\vS_i$ 就換成 $\\vB_i$；跨完 $n$ 次就得到 body form。每一步的乘積都和原本完全相等。`,
    simHint: '按「▸ M 往左移一格」：剛跨過的那一欄在兩張表中亮起，3D 只顯示它的 Sᵢ（橘）與 Bᵢ（紫）；下方的數值檢查顯示乘積始終等於 T(θ)。',
  },
  {
    title: 'Body form：後乘的順序',
    md: `移完之後就是

$$T(\\theta) = M\\,e^{[\\vB_1]\\theta_1}\\,e^{[\\vB_2]\\theta_2}\\cdots e^{[\\vB_n]\\theta_n}$$

這次由左往右讀，每一次都是**右乘**，也就是在目前的末端座標系中描述運動：從 $M$ 出發先轉關節 1，整支手臂連同 $\\{b\\}$ 一起轉；再轉關節 2……。$\\vB_i$ 黏在 $\\{b\\}$ 上被前面的關節帶著走，而它後面的關節都還在 0，所以它正好落在關節 $i$ 的實際位置。這和 space form「由右往左、軸固定在 $\\{s\\}$」恰好互為鏡像。`,
    simHint: '按「▸ 下一個因子」：先轉關節 1，再轉關節 2……；正要轉的那個關節，紫色的 Bᵢ（黏在 {b}）恰好與青色實線軸重合。',
  },
  {
    title: '在 UR5 上驗證',
    md: `UR5 的六條軸可以直接從幾何讀出（$W_1, W_2, L_1, L_2, H_1, H_2$，見表格）。以 $\\theta = (0, -\\tfrac{\\pi}{2}, 0, 0, \\tfrac{\\pi}{2}, 0)$ 為例，只有 $\\theta_2, \\theta_5$ 不為零，其餘的指數都是單位矩陣：

$$T = e^{[\\vS_2]\\theta_2}e^{[\\vS_5]\\theta_5}M = M\\,e^{[\\vB_2]\\theta_2}e^{[\\vB_5]\\theta_5}$$

兩條路徑給出同一個 $T$，末端位置約為 $(0.095, 0.109, 0.988)$ m。下方並列兩種算法：它們的差只剩 $10^{-16}$ 等級的浮點誤差，四捨五入後顯示完全相同。`,
    simHint: '已選「教材範例」：比對兩個 T 的最後一欄與 (0.095, 0.109, 0.988)。再按「隨機 θ」幾次，‖T_space − T_body‖ 始終在 1e−16 等級。',
  },
  {
    title: 'URDF 說的是另一種話',
    md: `機器人軟體常用 URDF 描述手臂：每個 joint 寫出相對**父連桿**座標系的 \`origin\`（平移 xyz、轉角 rpy），以及在自己座標系中的 \`axis\`。這是一串相鄰座標系之間的相對變換；PoE 則把所有軸都寫在同一個 $\\{s\\}$、同一個零位姿裡。要從 URDF 得到 $\\vS_i$，就沿父鏈把 origin 累乘：

$$T_{s,j_i} = T_{\\text{origin},1}\\cdots T_{\\text{origin},i},\\qquad \\hat\\omega_i = R_{s,j_i}\\,\\text{axis}_i,\\quad q_i = p_{s,j_i}$$

再代入 $\\vS_i = (\\hat\\omega_i, -\\hat\\omega_i\\times q_i)$。這裡的 URDF 是依書中尺寸自行簡化的示意，不是原廠檔案。`,
    simHint: '點樹狀圖中的 joint：對應的連桿與 joint 座標系會亮起，旁邊列出累乘後的 ω、q 與 Sᵢ，可與 PoE 表格逐欄比對。注意 elbow_joint 的 origin 寫在「沿手臂的 z」上。',
  },
  {
    title: '換工具、換零位姿與下一章',
    md: `在法蘭上裝一支沿 $\\hat z_b$ 長 0.1 m、不轉動的工具，只會改變末端座標系：$M_{\\text{tool}} = M\\,T_{bt}$。關節軸在 $\\{s\\}$ 中的位置沒變，所以 $\\vS_i$ 全部不變；$\\vB_i$ 則改以新的原點量測：

$$\\vB_i^{\\text{tool}} = [\\Ad_{T_{bt}^{-1}}]\\,\\vB_i,\\qquad \\omega \\text{ 不變},\\ v \\to v + \\omega\\times p_{bt}$$

UR5 有四條軸平行於 $\\hat z_b$，它們的 $\\vB_i$ 剛好不變。換零位姿則兩組都會變（上一課）。箭頭預告下一章：只讓關節 $i$ 以 1 rad/s 轉動時末端的扭轉（twist），就是 Jacobian 的第 $i$ 欄；箭頭畫的是這個扭轉造成的工具尖端速度。完成後做下方自我檢測。`,
    simHint: '切換「加裝工具」：S 表格完全不變，B 表格只有 B₁、B₅ 的 v 改變（底色）。關節 5 以 1 rad/s 轉動時，青色的尖端速度箭頭隨工具變長；再點表格欄標題 B₆ 改選關節 6：箭頭消失了，想想工具尖端和軸 6 的相對位置。',
  },
];

const quiz: QuizQuestion[] = [
  {
    question: '在 UR5 的法蘭上裝一支沿 $\\hat z_b$ 長 0.1 m、不轉動的工具，末端座標系移到工具尖端。下列哪些數字會改變？',
    options: [
      '只有 $M$',
      '$M$，以及 $\\omega$ 不平行 $\\hat z_b$ 的 $\\vB_i$（這裡是 $\\vB_1$、$\\vB_5$）的 $v$ 部分',
      '$M$ 與全部六個 $\\vS_i$',
      '$M$ 與全部六個 $\\vB_i$（包含 $\\omega$ 部分）',
    ],
    answer: 1,
    explanation: '$\\vS_i$ 描述關節軸在 $\\{s\\}$ 中的位置，和末端座標系放在哪裡無關，完全不變。$\\vB_i$ 是同一條軸以新的 $\\{b\\}$ 原點量：$\\vB_i\' = [\\Ad_{T_{bt}^{-1}}]\\vB_i$。工具不轉動，所以 $\\omega$ 不變，$v\' = v + \\omega\\times p$，$p = (0, 0, 0.1)$。關節 2、3、4、6 的軸都平行 $\\hat z_b$，$\\omega\\times p = 0$，數字剛好不變；$\\vB_1$、$\\vB_5$ 的 $v$ 則差了 0.1。',
  },
  {
    question: 'URDF 中 elbow_joint 寫著 origin xyz = (0, 0, 0.425)、rpy = (0, 0, 0)、axis = (0, 1, 0)。它的父連桿 upper_arm_link 在零位時相對 $\\{s\\}$ 的位置是 $(0, 0, 0.089)$，姿態是繞 $\\hat y$ 轉 90°（連桿的 $z$ 軸沿著手臂，指向 $\\{s\\}$ 的 $+x$）。$\\vS_3$ 怎麼來？',
    options: [
      '直接取 $q = (0, 0, 0.425)$、$\\hat\\omega = (0, 1, 0)$',
      '先算 $T_{s,p}\\,T_{\\text{origin}}$，取其位置 $(0.425, 0, 0.089)$ 為 $q$、$R\\,(0,1,0)$ 為 $\\hat\\omega$，再代入 $(\\hat\\omega, -\\hat\\omega\\times q)$',
      '$\\vS_3 = (0, 1, 0,\\, 0, 0, 0.425)$：xyz 就是 $v$',
      'URDF 的資訊不夠，必須另外量測',
    ],
    answer: 1,
    explanation: 'origin 是相對父連桿座標系寫的，而那個座標系已經被轉過、移過：沿它的 $z$ 走 0.425 m，在 $\\{s\\}$ 中其實是沿 $+x$ 走。所以要先累乘，得到 $q = (0.425, 0, 0.089)$、$\\hat\\omega = (0, 1, 0)$，$\\vS_3 = (0, 1, 0, -0.089, 0, 0.425)$，和 PoE 表格第 3 欄一致。步驟 5 的面板逐 joint 顯示這個累乘。',
  },
  {
    question: '某個應用只會頻繁改變 $\\theta_6$（其他關節固定），想把不變的部分預先算好。哪個說法最正確？',
    options: [
      'Space form 比較省，因為 $\\theta_6$ 的因子緊鄰 $M$',
      'Body form 比較省：預存 $P = M e^{[\\vB_1]\\theta_1}\\cdots e^{[\\vB_5]\\theta_5}$，每次只算一個指數再做一次 $P\\,e^{[\\vB_6]\\theta_6}$',
      '兩者都必須重算全部六個矩陣指數',
      '兩者完全一樣，因為最後結果相同',
    ],
    answer: 1,
    explanation: 'Body form 中 $\\theta_6$ 的因子在最右端，它左邊的東西全部不變，可以預存成一個矩陣 $P$，每次更新只需一個矩陣指數加一次 4×4 乘法。Space form 也只需重算 $e^{[\\vS_6]\\theta_6}$，但它右邊還夾著 $M$，得多乘一次。反過來，若只有 $\\theta_1$ 常變，space form 才是比較省的那一個。',
  },
  {
    question: '只轉 $\\theta_6$（其餘為 0）。固定在 $\\{s\\}$ 的 $\\vS_i$ 與黏在 $\\{b\\}$ 的 $\\vB_i$ 中，哪些仍與關節的實際軸重合？',
    options: [
      '全部的 $\\vS_i$ 與全部的 $\\vB_i$',
      '全部的 $\\vS_i$，以及只有 $\\vB_6$',
      '只有 $\\vS_6$ 與 $\\vB_6$',
      '全部的 $\\vB_i$，以及只有 $\\vS_1$',
    ],
    answer: 1,
    explanation: '$\\vS_i$ 與實際軸重合的條件是「它上游的關節都在 0」：只轉 $\\theta_6$ 時關節 1–5 都在 0，六條 $\\vS_i$ 都還在原處。$\\vB_i$ 重合的條件則是「它下游的關節都在 0」：$\\theta_6$ 讓 $\\{b\\}$ 繞軸 6 轉走，$\\vB_1$–$\\vB_5$ 跟著 $\\{b\\}$ 離開了實際的軸，只有繞自己轉的 $\\vB_6$ 留在原位。這正是兩種形式互為鏡像的地方。',
  },
];

// ---------------------------------------------------------------------------
// Simulation configuration
// ---------------------------------------------------------------------------

const DEG = Math.PI / 180;
const N = 6;
const S_VAR = '--series-4';
const B_VAR = '--series-5';
const P_VAR = '--series-6';
/** Tip-velocity preview: cyan clashes with none of the frame-axis colours (axis glyphs are hidden in that step). */
const V_VAR = '--series-6';
const ANIM_SEC = 0.8;
/** Arrow length per unit speed for the tip-velocity preview (s). */
const VEL_SCALE = 1.6;
const IDENT: Mat[] = Array.from({ length: N + 1 }, () => eye(4));

type Mode = 'free' | 'pass' | 'body';
type Path = 'space' | 'body';

interface Config {
  theta: number[];
  mode?: Mode;
  passK?: number;
  stage?: number;
  path?: Path;
  showS: boolean;
  showB: boolean;
  showPhys: boolean;
  selOnly?: boolean;
  sel?: number;
  levers?: boolean;
  tool?: boolean;
  verify?: boolean;
  urdf?: boolean;
  urdfSel?: number;
  vel?: boolean;
}

const shortName = (name: string): string => name.replace('_joint', '').replace('_fixed', '');

// ---------------------------------------------------------------------------
// mount
// ---------------------------------------------------------------------------

function mount(host: HTMLElement, ctx: LessonContext): LessonHandle {
  const S = {
    theta: [0, 0, 0, 0, 0, 0] as number[],
    mode: 'free' as Mode,
    passK: 0,
    stage: N,
    anim: null as { j: number; u: number } | null,
    autoplay: false,
    path: 'space' as Path,
    showS: true,
    showB: true,
    showPhys: false,
    selOnly: false,
    sel: -1,
    levers: false,
    tool: false,
    verify: false,
    urdf: false,
    urdfSel: 2,
    vel: false,
  };
  // ---- layout -----------------------------------------------------------
  simTitle(host, 'UR5：space form 與 body form 算的是同一個 T', 'UR5');
  const poseChips = presetChips(host, [
    ...POSES.map((p) => ({ label: p.label, title: p.title, apply: () => setTheta(p.theta, true) })),
    { label: '隨機 θ', title: '每個關節取 −180°～180° 的隨機值', apply: () => setTheta(Array.from({ length: N }, () => Math.round((Math.random() * 2 - 1) * 180) * DEG), true) },
  ], { label: '姿勢' });

  const scene = new Scene3D(host, { aspect: 1.75, grid: false, axes: false, autoStart: false, cameraPos: UR5_CAMERA.pos, target: UR5_CAMERA.target });
  const overlay = el('div', { class: 'mr-overlay', style: { right: '0.5rem', whiteSpace: 'normal' } });
  scene.el.appendChild(overlay);
  note(host, '橘虛線 = Sᵢ（固定在 {s}）· 紫點線 = Bᵢ（黏在 {b}）· 青實線 = 關節此刻的實際軸 · 可拖曳');

  const strip = factorStrip(host);

  const passRow = el('div', { class: 'mr-playback', style: { paddingTop: '0.1rem' } });
  host.appendChild(passRow);
  button(passRow, { label: '▸ M 往左移一格', small: true, variant: 'primary', title: '讓 M 跨過它左邊的一個因子', onClick: () => movePass(1) });
  button(passRow, { label: '◂ 退回一格', small: true, onClick: () => movePass(-1) });
  button(passRow, { label: '⇤ 全部移過去', small: true, title: '直接得到 body form', onClick: () => { S.passK = N; refresh(); } });
  button(passRow, { label: '⟲ 回到 space form', small: true, onClick: () => { S.passK = 0; refresh(); } });

  const bodyRow = el('div', { class: 'mr-playback', style: { paddingTop: '0.1rem' } });
  host.appendChild(bodyRow);
  button(bodyRow, { label: '▸ 下一個因子', small: true, variant: 'primary', title: '再右乘一個指數因子（動畫）', onClick: () => nextFactor() });
  button(bodyRow, { label: '▶ 從頭播放', small: true, onClick: () => playAll() });
  button(bodyRow, { label: '⟲ 回到 M', small: true, onClick: () => { S.stage = 0; S.anim = null; S.autoplay = false; refresh(); } });

  const stepStatus = el('p', { class: 'mr-note', style: { margin: '0.1rem 0 0.2rem' } });
  host.appendChild(stepStatus);

  const jointPanel = controlPanel(host, { compact: true });
  jointPanel.style.gridTemplateColumns = 'repeat(auto-fit, minmax(150px, 1fr))';
  const sliders: SliderHandle[] = Array.from({ length: N }, (_, i) => angleSlider(jointPanel, {
    label: `θ${subscript(i + 1)}`, value: 0, min: -Math.PI, max: Math.PI,
    onInput: (v) => { S.theta[i] = v; poseChips.clear(); refresh(); },
  }));

  const optPanel = controlPanel(host);
  const optRow = row(optPanel);
  const tS = toggle(optRow, { label: 'Sᵢ（固定在 {s}）', value: true, onChange: (v) => { S.showS = v; refresh(); } });
  const tB = toggle(optRow, { label: 'Bᵢ（黏在 {b}）', value: true, onChange: (v) => { S.showB = v; refresh(); } });
  const tP = toggle(optRow, { label: '實際關節軸', value: false, onChange: (v) => { S.showPhys = v; refresh(); } });
  const tSel = toggle(optRow, { label: '只看選定關節', value: false, onChange: (v) => { S.selOnly = v; if (v && S.sel < 0) S.sel = 0; refresh(); } });
  const tTool = toggle(optRow, { label: `加裝工具（${TOOL_LENGTH} m）`, value: false, onChange: (v) => { S.tool = v; refresh(); } });
  const tUrdf = toggle(optRow, { label: 'URDF 樹狀圖', value: false, onChange: (v) => { S.urdf = v; updateRows(); refresh(); } });
  const pathChips = presetChips(optRow, [
    { label: 'space form', title: 'T = e^[S1]θ1 ⋯ e^[S6]θ6 M', apply: () => { S.path = 'space'; refresh(); } },
    { label: 'body form', title: 'T = M e^[B1]θ1 ⋯ e^[B6]θ6', apply: () => { S.path = 'body'; refresh(); } },
  ], { label: '公式' });

  const tables = el('div', { style: { display: 'flex', flexWrap: 'wrap', alignItems: 'flex-start', marginTop: '0.2rem' } });
  host.appendChild(tables);
  const pick = (j: number) => { S.sel = S.sel === j && !S.selOnly ? -1 : j; refresh(); };
  const sTable = screwTable(tables, 'Sᵢ：以 {s} 量（零位姿）', pick);
  const bTable = screwTable(tables, 'Bᵢ：以 {b} 量（零位姿）', pick);

  // T computed both ways (step 4)
  const verifyBox = el('div', { style: { display: 'flex', flexWrap: 'wrap', alignItems: 'center', gap: '0 0.4rem', marginTop: '0.3rem' } });
  host.appendChild(verifyBox);
  const tsView = matrixView(verifyBox, { label: 'T（space）=', digits: 3 });
  const tbView = matrixView(verifyBox, { label: 'T（body）=', digits: 3 });
  for (const v of [tsView, tbView]) {
    v.el.style.fontSize = '0.74rem';
    v.el.style.marginRight = '0.2rem';
    (v.el.querySelector('.mr-grid') as HTMLElement | null)?.style.setProperty('column-gap', '0.42rem');
  }
  const diffRow = el('div', { style: { display: 'flex', flexWrap: 'wrap', gap: '0.2rem 1rem', alignItems: 'baseline', flexBasis: '100%' } });
  // the residual comes first so the number the step talks about is in the first view
  verifyBox.prepend(diffRow);
  const diffRead = readout(diffRow, { label: '‖T_space − T_body‖ =', minWidth: '7ch' });
  const bookNote = el('span', { class: 'mr-note', style: { margin: '0' } });
  diffRow.appendChild(bookNote);

  // URDF tree (step 5)
  const urdfBox = el('div', { style: { display: 'flex', flexWrap: 'wrap', gap: '0.4rem 1rem', marginTop: '0.35rem', alignItems: 'flex-start' } });
  host.appendChild(urdfBox);
  const tree = el('div', { style: { fontFamily: 'var(--mono)', fontSize: '0.74rem', display: 'flex', flexDirection: 'column', gap: '0.05rem' } });
  urdfBox.appendChild(tree);
  tree.appendChild(el('div', { style: { color: 'var(--fg-muted)', padding: '0.1rem 0.3rem' } }, 'base_link'));
  const treeRows = UR5_URDF.map((j, i) => {
    const b = el('button', {
      type: 'button',
      title: `選取 ${j.name}`,
      style: { font: 'inherit', textAlign: 'left', background: 'none', border: '1px solid transparent', borderRadius: '6px', padding: '0.1rem 0.3rem', paddingLeft: `${0.3 + 0.5 * (i + 1)}rem`, cursor: 'pointer', color: 'var(--fg)' },
    }, `└ ${j.name}`, el('span', { style: { color: 'var(--fg-muted)' } }, ` → ${j.child}`));
    b.addEventListener('click', () => { S.urdfSel = i; if (j.type === 'revolute') S.sel = i; refresh(); });
    tree.appendChild(b);
    return b;
  });
  const detail = el('div', { style: { flex: '1 1 14rem', minWidth: '0' } });
  urdfBox.appendChild(detail);
  const xmlPre = el('pre', { style: { fontFamily: 'var(--mono)', fontSize: '0.72rem', background: 'var(--bg-sunken)', borderRadius: '6px', padding: '0.4rem 0.55rem', margin: '0 0 0.3rem', whiteSpace: 'pre-wrap', overflowWrap: 'anywhere' } });
  detail.appendChild(xmlPre);
  // the accumulated numbers span the full width below the tree and the XML snippet
  const accum = kv(urdfBox, ['累乘 T_s,joint 的位置 p', '轉軸 ω = R · axis', '得到的螺旋軸']);
  accum.el.style.flexBasis = '100%';

  const K_P = '末端位置 p';
  const K_Q = '從 {s} 量：q';
  const K_QB = '從 {b} 量：qᵇ';
  const K_PASS = '目前乘積 − T(θ)';
  const K_CONV = '剛換好的欄';
  const K_VEL = '尖端速度';
  const info = kv(host, [K_P, K_Q, K_QB, K_PASS, K_CONV, K_VEL]);
  // readouts, the two-way check (step 4) and the URDF tree (step 5) come before the 12-row screw
  // tables, so what each step talks about is visible without scrolling the simulation pane
  for (const e of [info.el, verifyBox, urdfBox]) host.insertBefore(e, tables);

  // ---- 3D -------------------------------------------------------------------
  const root = new THREE.Group();
  scene.scene.add(root);
  root.add(makeGrid(3, 12));
  const sFrame = makeFrame(0.2, '{s}', { labelHeight: 0.075 });
  root.add(sFrame.group);
  const ghost = new ChainView(UR5_SEGMENTS, { opacity: 0.16 });
  const arm = new ChainView(UR5_SEGMENTS);
  root.add(ghost.group, arm.group);
  const toolLink = new Link3D(TOOL_SEGMENT.r * 1.3, '--warn');
  root.add(toolLink.mesh);
  const bFrame = makeFrame(0.17, '{b}', { labelHeight: 0.075 });
  const mFrame = makeFrame(0.14, 'M', { opacity: 0.45, labelHeight: 0.065 });
  const target = makeFrame(0.14, 'T(θ)', { opacity: 0.4, labelHeight: 0.065, labelColor: '--accent' });
  const stageFrames = Array.from({ length: N - 1 }, (_, k) => makeFrame(0.12, circled(k + 1), { opacity: 0.45, labelHeight: 0.07 }));
  for (const f of [bFrame, mFrame, target, ...stageFrames]) root.add(f.group);
  const glyph = (color: string, kind: 'S' | 'B' | 'P', j: number): AxisGlyph => new AxisGlyph({
    color,
    dashed: kind !== 'P',
    back: 0.12,
    front: 0.16,
    radius: kind === 'B' ? 0.0052 : 0.0044,
    dash: kind === 'B' ? 0.011 : undefined,
    gap: kind === 'B' ? 0.012 : undefined,
    arrow: 0.05,
    label: kind === 'P' ? undefined : `${kind}${subscript(j + 1)}`,
    labelAtBack: kind === 'B',
    labelHeight: 0.06,
  });
  const sAx = Array.from({ length: N }, (_, j) => glyph(S_VAR, 'S', j));
  const bAx = Array.from({ length: N }, (_, j) => glyph(B_VAR, 'B', j));
  const pAx = Array.from({ length: N }, (_, j) => glyph(P_VAR, 'P', j));
  for (const g of [...sAx, ...bAx, ...pAx]) root.add(g.group);
  const dashedLine = (color: string): THREE.Line => {
    const mat = new THREE.LineDashedMaterial({ dashSize: 0.025, gapSize: 0.018 });
    setThemedColor(mat, color);
    const l = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3(1, 0, 0)]), mat);
    l.frustumCulled = false;
    root.add(l);
    return l;
  };
  const leverS = dashedLine(S_VAR);
  const leverB = dashedLine(B_VAR);
  const qDot = new THREE.Mesh(new THREE.SphereGeometry(0.014, 14, 10), themedStandardMaterial('--fg'));
  const qLabel = textSprite('q', S_VAR, 0.065);
  const qbLabel = textSprite('qᵇ', B_VAR, 0.065);
  root.add(qDot, qLabel, qbLabel);
  const velArrow = new Arrow3D(V_VAR, 0.008, 0.045);
  // drawn on top of the arm so the preview arrow is never hidden behind the wrist
  velArrow.group.traverse((o) => {
    const m = (o as THREE.Mesh).material as THREE.Material | undefined;
    if (m) m.depthTest = false;
    o.renderOrder = 5;
  });
  const velLabel = textSprite('尖端速度', V_VAR, 0.06);
  root.add(velArrow.group, velLabel);
  // URDF joint frames; the name label lives in world space (above the frame) so it never sits on a housing
  const urdfFrames = UR5_URDF.map((j) => {
    const f = makeFrame(0.12, '');
    const label = textSprite(shortName(j.name), '--accent', 0.065);
    root.add(f.group, label);
    return { f, label };
  });

  // ---- helpers ----------------------------------------------------------------
  function setTheta(th: number[], fit = false): void {
    S.theta = th.slice();
    sliders.forEach((s, i) => s.set(S.theta[i], { silent: true }));
    if (S.mode === 'body') { S.stage = N; S.anim = null; S.autoplay = false; }
    if (fit) fitCamera();
    refresh();
  }

  /**
   * Frame the arm (and the home frame M, plus the home arm in the step-through) from a fixed
   * viewing direction: bounding sphere of the drawn points, distance from the camera's field of view.
   */
  function fitCamera(): void {
    const Mh = S.tool ? toolFrame(ur5.M, TOOL_LENGTH) : ur5.M;
    const L = chainTransforms(ur5.Slist, S.theta);
    const T = matMul(L[N], Mh);

    const pts: Vec[] = [[0, 0, 0], [T[0][3], T[1][3], T[2][3]], [Mh[0][3], Mh[1][3], Mh[2][3]]];
    for (const seg of UR5_SEGMENTS) pts.push(carry(L[seg.body], seg.a), carry(L[seg.body], seg.b));
    if (S.mode === 'body') for (const seg of UR5_SEGMENTS) pts.push(seg.a, seg.b);
    if (S.vel && S.sel >= 0) {
      // keep the tip-velocity arrow (and its label) in view
      const p = [T[0][3], T[1][3], T[2][3]];
      const v = pointVelocity(ur5.Slist, S.theta, S.sel, p);
      pts.push([p[0] + VEL_SCALE * v[0], p[1] + VEL_SCALE * v[1], p[2] + VEL_SCALE * v[2] + 0.06]);
    }
    const lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
    for (const p of pts) for (let k = 0; k < 3; k++) { lo[k] = Math.min(lo[k], p[k]); hi[k] = Math.max(hi[k], p[k]); }
    const c = [0, 1, 2].map((k) => (lo[k] + hi[k]) / 2);
    const r = Math.max(0.25, ...pts.map((p) => Math.hypot(p[0] - c[0], p[1] - c[1], p[2] - c[2]))) + 0.12;
    const cam = scene.camera;
    const half = Math.min((cam.fov * DEG) / 2, Math.atan(Math.tan((cam.fov * DEG) / 2) * (cam.aspect || 1.5)));
    const d = (r / Math.sin(half)) * 0.9;
    const az = -58 * DEG, elv = 22 * DEG;
    cam.position.set(c[0] + d * Math.cos(elv) * Math.cos(az), c[1] + d * Math.cos(elv) * Math.sin(az), c[2] + d * Math.sin(elv));
    scene.controls?.target.set(c[0], c[1], c[2]);
    scene.controls?.update();
    scene.requestRender();
  }

  function movePass(d: number): void {
    S.passK = Math.max(0, Math.min(N, S.passK + d));
    refresh();
  }

  function startFactor(): void {
    if (S.stage >= N) { S.autoplay = false; return; }
    S.anim = { j: S.stage, u: 0 };
    loop.start();
  }

  function nextFactor(): void {
    S.autoplay = false;
    if (S.anim) { S.stage += 1; S.anim = null; refresh(); return; }
    if (S.stage >= N) S.stage = 0;
    startFactor();
    refresh();
  }

  function playAll(): void {
    S.stage = 0;
    S.anim = null;
    S.autoplay = true;
    startFactor();
    refresh();
  }

  const loop = createLoop((dt) => {
    if (S.anim) {
      S.anim.u = Math.min(1, S.anim.u + dt / ANIM_SEC);
      if (S.anim.u >= 1) {
        S.stage += 1;
        S.anim = null;
        if (S.autoplay && S.stage < N) startFactor(); else S.autoplay = false;
      }
    }
    refresh();
    if (!S.anim) loop.stop();
  });

  function updateRows(): void {
    passRow.style.display = S.mode === 'pass' ? '' : 'none';
    bodyRow.style.display = S.mode === 'body' ? '' : 'none';
    stepStatus.style.display = S.mode === 'free' ? 'none' : '';
    verifyBox.style.display = S.verify ? 'flex' : 'none';
    urdfBox.style.display = S.urdf ? 'flex' : 'none';
    pathChips.el.style.display = S.mode === 'free' ? '' : 'none';
  }

  function configure(c: Config): void {
    S.anim = null;
    S.autoplay = false;
    loop.stop();
    S.theta = c.theta.slice();
    S.mode = c.mode ?? 'free';
    S.passK = c.passK ?? 0;
    S.stage = S.mode === 'body' ? (c.stage ?? 0) : N;
    S.path = c.path ?? 'space';
    S.showS = c.showS;
    S.showB = c.showB;
    S.showPhys = c.showPhys;
    S.selOnly = !!c.selOnly;
    S.sel = c.sel ?? -1;
    S.levers = !!c.levers;
    S.tool = !!c.tool;
    S.verify = !!c.verify;
    S.urdf = !!c.urdf;
    S.urdfSel = c.urdfSel ?? 2;
    S.vel = !!c.vel;
    sliders.forEach((s, i) => s.set(S.theta[i], { silent: true }));
    tS.set(S.showS, { silent: true });
    tB.set(S.showB, { silent: true });
    tP.set(S.showPhys, { silent: true });
    tSel.set(S.selOnly, { silent: true });
    tTool.set(S.tool, { silent: true });
    tUrdf.set(S.urdf, { silent: true });
    const pi = POSES.findIndex((p) => p.theta.every((x, i) => Math.abs(x - S.theta[i]) < 1e-12));
    if (pi >= 0) poseChips.highlight(pi); else poseChips.clear();
    fitCamera();
    updateRows();
    refresh();
  }

  // ---- drawing + numbers ----------------------------------------------------
  function refresh(): void {
    const Mh = S.tool ? toolFrame(ur5.M, TOOL_LENGTH) : ur5.M;
    const Bl = bodyAxes(ur5.Slist, Mh);
    const body = S.mode === 'body';
    const th = body ? bodyPartial(S.theta, S.stage, S.anim?.u ?? 0) : S.theta.slice();
    const L = chainTransforms(ur5.Slist, th);
    const T = matMul(L[N], Mh);
    const Tfull = FKinSpace(Mh, ur5.Slist, S.theta);
    const now = currentAxes(ur5.Slist, th);
    const pT = [T[0][3], T[1][3], T[2][3]];

    arm.setBodies(L);
    ghost.setBodies(IDENT);
    ghost.setVisible(body);
    toolLink.mesh.visible = S.tool;
    if (S.tool) toolLink.setEndpoints(carry(L[N], TOOL_SEGMENT.a), carry(L[N], TOOL_SEGMENT.b));
    bFrame.setTransform(T);
    mFrame.setTransform(Mh);

    // in "pass" mode only the factor that just crossed M is shown
    const newest = S.mode === 'pass' && S.passK > 0 ? N - S.passK : -1;
    const sel = S.mode === 'pass' ? newest : S.sel;
    const only = S.mode === 'pass' || S.selOnly;
    const urdfJoint = S.urdf ? S.urdfSel : -1;
    const hl = urdfJoint >= 0 ? Math.min(N, urdfJoint + 1) : sel >= 0 ? sel + 1 : null;
    arm.highlight(S.urdf ? hl : null, '--warn', 0.45);

    for (let j = 0; j < N; j++) {
      const vis = (on: boolean) => on && (!only || j === sel);
      const k = j === sel ? 1.8 : 1;
      sAx[j].set(getCol(ur5.Slist, j), UR5_ANCHORS[j]);
      sAx[j].setVisible(vis(S.showS));
      sAx[j].emphasize(k);
      bAx[j].set(gluedToB(T, Mh, getCol(ur5.Slist, j)), carry(L[N], UR5_ANCHORS[j]));
      bAx[j].setVisible(vis(S.showB));
      bAx[j].emphasize(k);
      pAx[j].set(now[j], carry(L[j], UR5_ANCHORS[j]));
      pAx[j].setVisible(vis(S.showPhys));
      pAx[j].emphasize(body && S.anim?.j === j ? 1.8 : k);
    }

    // body step-through: intermediate frames and the final pose
    for (let k = 1; k < N; k++) {
      const f = stageFrames[k - 1];
      const show = body && k <= S.stage && !(k === S.stage && !S.anim);
      f.setVisible(show);
      if (show) f.setTransform(FKinSpace(Mh, ur5.Slist, bodyPartial(S.theta, k)));
    }
    target.setVisible(body && S.stage < N);
    target.setTransform(Tfull);

    // the selected axis measured from {s} (q) and from {b} (qᵇ)
    const showLev = S.levers && sel >= 0;
    let qS: Vec = [0, 0, 0], qB: Vec = [0, 0, 0];
    if (showLev) {
      qS = axisPlacement(getCol(ur5.Slist, sel), UR5_ANCHORS[sel]).point;
      qB = axisPlacement(gluedToB(T, Mh, getCol(ur5.Slist, sel)), carry(L[N], UR5_ANCHORS[sel])).point;
      (leverS.geometry as THREE.BufferGeometry).setFromPoints([new THREE.Vector3(0, 0, 0), new THREE.Vector3(...qS)]);
      leverS.computeLineDistances();
      (leverB.geometry as THREE.BufferGeometry).setFromPoints([new THREE.Vector3(...pT), new THREE.Vector3(...qB)]);
      leverB.computeLineDistances();
      qDot.position.set(qS[0], qS[1], qS[2]);
      qLabel.position.set(qS[0] / 2, qS[1] / 2 - 0.03, qS[2] / 2 + 0.05);
      qbLabel.position.set((pT[0] + qB[0]) / 2, (pT[1] + qB[1]) / 2 - 0.03, (pT[2] + qB[2]) / 2 + 0.05);
    }
    leverS.visible = qLabel.visible = showLev && S.showS;
    leverB.visible = qbLabel.visible = showLev && S.showB;
    qDot.visible = showLev;

    // tip velocity when only the selected joint turns at 1 rad/s (Jacobian preview)
    const showVel = S.vel && S.sel >= 0;
    let vTip: Vec = [0, 0, 0];
    if (showVel) {
      vTip = pointVelocity(ur5.Slist, S.theta, S.sel, pT);
      velArrow.setFromTo(pT, [pT[0] + VEL_SCALE * vTip[0], pT[1] + VEL_SCALE * vTip[1], pT[2] + VEL_SCALE * vTip[2]]);
      const e = [pT[0] + VEL_SCALE * vTip[0], pT[1] + VEL_SCALE * vTip[1], pT[2] + VEL_SCALE * vTip[2] + 0.05];
      velLabel.position.set(e[0], e[1], e[2]);
    }
    velArrow.group.visible = showVel && Math.hypot(...vTip) > 1e-6;
    velLabel.visible = velArrow.group.visible;

    // URDF joint frames (the selected one)
    const uf = S.urdf ? urdfForward(UR5_URDF, th).frames : [];
    urdfFrames.forEach(({ f, label }, i) => {
      const on = S.urdf && i === S.urdfSel;
      f.setVisible(on);
      label.visible = on;
      if (on) {
        f.setTransform(uf[i]);
        label.position.set(uf[i][0][3], uf[i][1][3] - 0.05, uf[i][2][3] + 0.13);
      }
    });

    // ---- DOM ----
    paintStrip();
    const B0 = ur5.Blist;
    sTable.setTitle(S.tool ? 'Sᵢ：以 {s} 量（裝工具後不變）' : 'Sᵢ：以 {s} 量（零位姿）');
    sTable.set(Array.from({ length: N }, (_, j) => getCol(ur5.Slist, j)), Array.from({ length: N }, (_, j) => `S${subscript(j + 1)}`), Array(N).fill(S_VAR), { selected: sel, digits: 3 });
    bTable.setTitle(S.tool ? 'Bᵢ：以工具尖端 {b} 量（底色 = 改變）' : 'Bᵢ：以 {b} 量（零位姿）');
    bTable.set(Array.from({ length: N }, (_, j) => getCol(Bl, j)), Array.from({ length: N }, (_, j) => `B${subscript(j + 1)}`), Array(N).fill(B_VAR), {
      selected: sel, digits: 3, marked: S.tool ? changedColumns(B0, Bl) : undefined,
    });

    if (S.verify) {
      const Ts = FKinSpace(Mh, ur5.Slist, S.theta);
      const Tb = FKinBody(Mh, Bl, S.theta);
      tsView.set(Ts, { highlightCols: ['x', 'y', 'z', null] });
      tbView.set(Tb, { highlightCols: ['x', 'y', 'z', null] });
      diffRead.set(fmtSci(transformGap(Ts, Tb)));
      const isBook = !S.tool && POSES[1].theta.every((x, i) => Math.abs(x - S.theta[i]) < 1e-9);
      const t = isBook ? `教材：p ≈ ${fmt3(BOOK_P, 3)} m ✓` : '';
      if (bookNote.textContent !== t) bookNote.textContent = t;
    }

    if (S.urdf) paintUrdf();

    info.set(K_P, `${fmt3(pT, 3)} m${S.tool ? '（工具尖端）' : ''}`);
    info.show(K_Q, showLev);
    info.show(K_QB, showLev);
    if (showLev) {
      const Tinv = TransInv(T);
      const qb = carry(Tinv, qB);
      info.set(K_Q, `S${subscript(sel + 1)} 的軸過 ${fmt3(qS, 3)}（{s} 座標）`);
      info.set(K_QB, `B${subscript(sel + 1)} 的軸過 ${fmt3(qb, 3)}（{b} 座標）`);
    }
    info.show(K_PASS, S.mode === 'pass');
    info.show(K_CONV, S.mode === 'pass' && newest >= 0);
    if (S.mode === 'pass') {
      info.set(K_PASS, `${fmtSci(transformGap(passingProduct(ur5.Slist, Bl, Mh, S.theta, S.passK), Tfull))}（每一步都等於 T(θ)）`);
      if (newest >= 0) {
        const b = getCol(Bl, newest);
        info.set(K_CONV, `B${subscript(newest + 1)} = [Ad_M⁻¹] S${subscript(newest + 1)} = (${b.map((x) => fmt(x, 3)).join(', ')})`);
      }
    }
    info.show(K_VEL, showVel);
    if (showVel) {
      const onAxis = Math.hypot(...vTip) < 1e-9 ? '（尖端就在這個關節的轉軸上，所以不動）' : '';
      info.set(K_VEL, `只轉關節 ${S.sel + 1}（1 rad/s）：${fmt3(vTip, 3)} m/s，|v| = ${fmt(Math.hypot(...vTip), 3)} m/s${onAxis}`);
    }

    const ov = S.mode === 'body'
      ? `body form 由左往右 · 已右乘 ${S.stage}/${N} 個因子`
      : S.mode === 'pass' ? `M 已往左跨過 ${S.passK}/${N} 個因子` : `θ = (${S.theta.map((x) => `${fmt(x / DEG, 0)}°`).join(', ')})`;
    if (overlay.textContent !== ov) overlay.textContent = ov;
    paintStatus(newest, T, Tfull);
    scene.requestRender();
  }

  /** One line under the formula strip describing the current stage (may contain typeset math). */
  function paintStatus(newest: number, T: Mat, Tfull: Mat): void {
    const text = (t: string) => setStatus(t, () => [t]);
    if (S.mode === 'pass') {
      if (S.passK === 0) text('目前是 space form：M 在最右邊。');
      else if (S.passK === N) text('六個因子都跨過了：這就是 body form。');
      else {
        const k = newest + 1;
        setStatus(`pass${k}`, () => [mathEl(`e^{[\\vS_{${k}}]\\theta_{${k}}}M = M\\,e^{[\\vB_{${k}}]\\theta_{${k}}}`), '：同一條軸，改用 {b} 的座標寫。']);
      }
    } else if (S.mode === 'body') {
      if (S.anim) text(`正在轉關節 ${S.anim.j + 1}：紫色 B${subscript(S.anim.j + 1)} 黏在 {b} 上；它後面的關節都還在 0，所以正好與實際軸重合。`);
      else if (S.stage === 0) text('從 M 出發：按「▸ 下一個因子」，先轉關節 1（整支手臂一起轉）。');
      else if (S.stage < N) text(`已右乘 ${S.stage} 個因子：關節 ${S.stage === 1 ? '1' : `1–${S.stage}`} 已轉好，後面的關節還在 0。`);
      else text(`全部套用完畢：與 FKinSpace 的差 ${fmtSci(transformGap(T, Tfull))}。`);
    } else text('');
  }

  let lastStatus = '';
  /** Replace the status line only when its content changes (it may contain typeset math). */
  function setStatus(key: string, build: () => (Node | string)[]): void {
    if (key === lastStatus) return;
    lastStatus = key;
    stepStatus.replaceChildren(...build());
  }

  function paintStrip(): void {
    const sF = (j: number) => `e^{[\\vS_{${j + 1}}]\\theta_{${j + 1}}}`;
    const bF = (j: number) => `e^{[\\vB_{${j + 1}}]\\theta_{${j + 1}}}`;
    const mTex = S.tool ? 'M_{\\text{tool}}' : 'M';
    const zero = (j: number) => Math.abs(S.theta[j]) < 1e-12;
    if (S.mode === 'pass') {
      const split = N - S.passK;
      const factors = [...Array.from({ length: split }, (_, j) => sF(j)), mTex, ...Array.from({ length: S.passK }, (_, k) => bF(split + k))];
      strip.build('T(\\theta) =', factors);
      const states: FactorState[] = factors.map((_, i) => (i < split ? 'plain' : i === split ? 'active' : 'applied'));
      const caps = factors.map((_, i) => (i === split + 1 ? '剛跨過 M' : ''));
      strip.paint(states, caps);
    } else if (S.mode === 'body') {
      strip.build('T(\\theta) =', [mTex, ...Array.from({ length: N }, (_, j) => bF(j))]);
      const states: FactorState[] = [
        'applied',
        ...Array.from({ length: N }, (_, j): FactorState => (S.anim?.j === j ? 'active' : j < S.stage ? 'applied' : 'pending')),
      ];
      strip.paint(states, [circled(0), ...Array.from({ length: N }, (_, j) => circled(j + 1))]);
    } else if (S.path === 'body') {
      strip.build('T(\\theta) =', [mTex, ...Array.from({ length: N }, (_, j) => bF(j))]);
      strip.paint(['plain', ...Array.from({ length: N }, (_, j): FactorState => (zero(j) ? 'identity' : 'plain'))], ['', ...Array.from({ length: N }, (_, j) => (zero(j) ? '= I' : ''))]);
    } else {
      strip.build('T(\\theta) =', [...Array.from({ length: N }, (_, j) => sF(j)), mTex]);
      strip.paint([...Array.from({ length: N }, (_, j): FactorState => (zero(j) ? 'identity' : 'plain')), 'plain'], [...Array.from({ length: N }, (_, j) => (zero(j) ? '= I' : '')), '']);
    }
    pathChips.highlight(S.path === 'space' ? 0 : 1);
  }

  function paintUrdf(): void {
    const i = S.urdfSel;
    const j = UR5_URDF[i];
    treeRows.forEach((b, k) => {
      b.style.background = k === i ? 'var(--accent-soft)' : 'none';
      b.style.borderColor = k === i ? 'var(--accent)' : 'transparent';
    });
    const x = urdfXml(j);
    if (xmlPre.textContent !== x) xmlPre.textContent = x;
    const F = homeJointFrames(UR5_URDF)[i];
    const p = [F[0][3], F[1][3], F[2][3]];
    accum.setLabel('累乘 T_s,joint 的位置 p', `累乘 ${Array.from({ length: i + 1 }, (_, k) => `T${subscript(k + 1)}`).join('')} 的位置 p`);
    accum.set('累乘 T_s,joint 的位置 p', `${fmt3(p, 3)}（{s} 座標）`);
    if (j.type === 'revolute') {
      const { omega, S: Si } = screwFromFrame(F, j.axis);
      const ref = getCol(ur5.Slist, i);
      const same = Si.every((v, k) => Math.abs(v - ref[k]) < 1e-9);
      accum.show('轉軸 ω = R · axis', true);
      accum.set('轉軸 ω = R · axis', `${fmt3(omega, 0)}`);
      accum.setLabel('得到的螺旋軸', `S${subscript(i + 1)} = (ω, −ω × p)`);
      accum.set('得到的螺旋軸', `(${Si.map((v) => fmt(v, 3)).join(', ')}) ${same ? '✓ 與 PoE 表格相同' : '✗'}`);
    } else {
      const same = transformGap(F, ur5.M) < 1e-9;
      accum.show('轉軸 ω = R · axis', false);
      accum.setLabel('得到的螺旋軸', 'fixed joint：tool0 的位姿');
      accum.set('得到的螺旋軸', `就是 M ${same ? '✓（與 PoE 的 M 相同）' : '✗'}`);
    }
  }

  // ---- steps ----------------------------------------------------------------
  const work = POSES[2].theta;
  const STEP_SETUP: Config[] = [
    { theta: [0, 0, 0, 0, 0, 0], path: 'body', showS: true, showB: true, showPhys: false, selOnly: true, sel: 2, levers: true },
    { theta: work, mode: 'pass', passK: 1, showS: true, showB: true, showPhys: true },
    { theta: work, mode: 'body', stage: 1, showS: false, showB: true, showPhys: true },
    { theta: POSES[1].theta, path: 'space', showS: false, showB: false, showPhys: false, verify: true },
    { theta: [0, 0, 0, 0, 0, 0], showS: false, showB: false, showPhys: true, selOnly: true, sel: 2, urdf: true, urdfSel: 2 },
    { theta: POSES[3].theta, path: 'body', showS: false, showB: true, showPhys: false, selOnly: true, sel: 4, tool: true, vel: true },
  ];

  let focused: HTMLElement[] = [];
  function setFocus(i: number): void {
    focused.forEach((f) => f.classList.remove('mr-focus'));
    const targets: (HTMLElement | null)[][] = [
      [sTable.el, bTable.el],
      [strip.el, passRow],
      [strip.el, bodyRow],
      [poseChips.el, verifyBox],
      [urdfBox],
      [tTool.el, bTable.el],
    ];
    focused = (targets[i] ?? []).filter((x): x is HTMLElement => !!x);
    focused.forEach((f) => f.classList.add('mr-focus'));
  }

  function onStep(i: number): void {
    const c = STEP_SETUP[i];
    if (c) configure(c);
    setFocus(i);
  }

  onStep(Math.min(ctx.currentStep, steps.length - 1));

  return {
    onStep,
    destroy() {
      S.anim = null;
      S.autoplay = false;
      loop.stop();
      disposeObject(root);
      scene.destroy();
    },
  };
}

const lesson: Lesson = { meta, steps, quiz, mount };
export default lesson;
