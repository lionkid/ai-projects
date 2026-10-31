import type { Lesson, LessonContext, LessonHandle, LessonStep, QuizQuestion } from '@/types';
import {
  el, cssVar, fmt, fmtVec, controlPanel, row, slider, angleSlider, toggle, button, select, matrixView, simTitle, note, kv,
  legend, presetChips, mathEl, Canvas2D, Scene3D, Arrow3D, createLoop, type Pt, type MatrixHandle, type World,
} from '@/ui';
import { type Mat, type Vec, ProjectToSO3, RotX, RotZ, approxEqual, clamp, dot, eye, matMul, matVec, transpose, vecScale } from '@/mr';
import { meta } from './meta';
import {
  type AxisName, type ComposeMode, type ElemRot, type PlanarPose, columns, composeSequence, dropLines, equivalentAxisAngle,
  invertPlanarPose, parallelepipedEdges, planarR, planarToBody, planarToSpace, randomRotation, rotationArc,
  sequenceAxisInSpace, so3Report,
} from './rotations';
import { matrixWidget } from './matrixWidget';
import { AXIS_VARS, DashedSegments, Trail, Triad, disposeSprite, label3D } from './view3d';

// ---------------------------------------------------------------------------
// Narrative
// ---------------------------------------------------------------------------

const steps: LessonStep[] = [
  {
    title: '平面暖身：用 (R, p) 描述一個座標系',
    md: `在平面上要交代座標系 $\\{b\\}$ 放在哪、朝哪裡，只需要一對資料：原點位置 $p_{sb}$（用 $\\{s\\}$ 的座標寫），以及一個 $2\\times 2$ 的旋轉矩陣 $R_{sb}$，它的兩欄就是 $\\hat x_b$、$\\hat y_b$ 在 $\\{s\\}$ 中的方向。下標 $sb$ 讀作「$\\{b\\}$ 相對於 $\\{s\\}$」。

有了這一對，任何一點都能在兩套座標之間換算：

$$p_s = R_{sb}\\,p_b + p_{sb}$$

$p_b$ 是拿 $\\{b\\}$ 的量尺量出來的數字，$p_s$ 是拿 $\\{s\\}$ 的量尺量出來的數字；$R_{sb}$ 與 $p_{sb}$ 負責把前者翻譯成後者。`,
    simHint: '拖曳 {b} 的中心或方向把手（青色），也可以用滑桿 θ、x、y；再拖曳點 P。核對下方算式：R_sb p_b + p_sb 是否永遠等於 p_s？',
  },
  {
    title: '同一個點，兩套座標',
    md: `$R_{sb}\\,p_b + p_{sb}$ 這個運算**沒有移動任何東西**：P 還在原地，我們只是把「從 $\\{b\\}$ 量到的數字」改寫成「從 $\\{s\\}$ 量到的數字」。反方向的換算是 $p_b = R_{sb}^{\\T}(p_s - p_{sb})$。

現在把 P 釘住，改去搬動 $\\{b\\}$。左圖裡 P 紋風不動，$p_s$ 也不變；右圖是「站在 $\\{b\\}$ 上看出去」的畫面，P 在那裡的位置就是 $p_b$，它跟著改變了——換掉的是量尺，不是被量的東西。`,
    simHint: '「釘住 P」已勾選。拖曳 {b}（或拉 θ 滑桿）：左圖的 P 不動、p_s 不變；右圖（從 {b} 看）裡的 P 卻在移動，它的座標正是 p_b。',
  },
  {
    title: '旋轉矩陣的三個欄位',
    md: `進入三維。先只談「朝向」，把 $\\{b\\}$ 的原點放在 $\\{s\\}$ 的原點上。這時 $R_{sb}$ 是 $3\\times 3$ 矩陣，三個欄位依序是 $\\{b\\}$ 的三個單位軸用 $\\{s\\}$ 座標寫出來的向量：

$$R_{sb} = \\begin{bmatrix} \\hat x_b & \\hat y_b & \\hat z_b \\end{bmatrix}$$

所以讀旋轉矩陣最快的方法是**一次讀一欄**：第一欄（紅）就是紅色箭頭尖端的座標，第二欄（綠）、第三欄（藍）同理。九個數字其實是三支箭頭。`,
    simHint: '把滑鼠移到矩陣的某一欄（或點一下）：對應的 {b} 軸變粗，虛線把箭頭尖端投影到 {s} 的軸上，投影長度就是這一欄的三個數字。拉動 A、B 換個姿態再讀一次。',
  },
  {
    title: 'SO(3)：正交、det = 1、三個自由度',
    md: `九個數字不能隨便填。三欄都要是單位長（3 個條件）、兩兩垂直（3 個條件），合起來就是 $R^{\\T}R = I$，所以只剩 $9 - 6 = 3$ 個自由度。還差一個條件 $\\det R = +1$：$\\det R = -1$ 的矩陣也正交，卻把右手座標系變成左手的，那是鏡像，任何實體轉動都做不到。

$$\\SO(3) = \\{\\, R \\in \\R^{3\\times 3} : R^{\\T}R = I,\\ \\det R = 1 \\,\\}$$

圖中的虛線方塊是單位立方體被 $R$ 映射後的形狀，它的（有號）體積正是 $\\det R$。`,
    simHint: '矩陣現在可以直接輸入數字。改動任一格，或按「拉長」「剪切」「鏡射」：看框架怎麼變形、RᵀR 與 det 怎麼偏離；最後按「投影回 SO(3)」拉回最近的旋轉。',
  },
  {
    title: '合成與下標消去',
    md: `若 $\\{c\\}$ 相對 $\\{b\\}$ 的朝向是 $R_{bc}$，那麼 $\\{c\\}$ 相對 $\\{s\\}$ 的朝向就是兩者相乘，相鄰的下標像分數約分一樣「消去」：

$$R_{sc} = R_{sb}\\,R_{bc}, \\qquad R_{bs} = R_{sb}^{-1} = R_{sb}^{\\T}$$

反過來描述（$\\{s\\}$ 相對 $\\{b\\}$）只要轉置，這是正交帶來的好處。寫算式時先檢查下標：像 $R_{sb}R_{cb}$ 這樣中間對不上，幾乎一定是寫錯了。`,
    simHint: '拉動 R_bc 的角度，{c} 相對 {b} 轉動（為了看清楚，{c} 畫在 x_b 的延長線上；這裡只看朝向）。改變 A、B 時，{c} 黏著 {b} 一起走。勾選「以 {b} 為參考」：{b} 擺正，{s} 改由 R_bs = R_sbᵀ 描述。',
  },
  {
    title: '前乘＝固定軸，後乘＝物體軸',
    md: `對已有的姿態 $R$ 再轉一次 $\\mathrm{Rot}(\\hat\\omega, \\theta)$，乘在左邊或右邊意思不同：

$$\\underbrace{\\mathrm{Rot}(\\hat\\omega,\\theta)\\,R}_{\\text{繞 } \\{s\\} \\text{ 的軸}} \\qquad\\qquad \\underbrace{R\\,\\mathrm{Rot}(\\hat\\omega,\\theta)}_{\\text{繞 } \\{b\\} \\text{ 的軸}}$$

前乘時 $\\hat\\omega$ 是 $\\{s\\}$ 的固定軸；後乘時 $\\hat\\omega$ 是 $\\{b\\}$ 自己（已經轉過）的軸。平面上只有一種轉軸，兩者沒有差別；三維裡一般不同。從 $R = I$ 出發連轉兩次時，「固定軸、先 A 後 B」與「物體軸、先 B 後 A」會得到同一個結果。`,
    simHint: '起始姿態已傾斜，A = 繞 ẑ 轉 90°。按「▶ 播放序列」看它轉，再切換後乘／前乘比較（半透明虛影是另一種乘法的結果）。接著選 R₀ = I、把 B 也設成 90°，按「⇄ 交換」：虛影正好落在剛才實線的位置。',
  },
  {
    title: 'R 的三種用途與下一步',
    md: `同一組九個數字，有三種讀法：

1. **姿態**：$R_{sb}$ 說明 $\\{b\\}$ 相對 $\\{s\\}$ 朝向哪裡。
2. **換座標**：$v_s = R_{sb}\\,v_b$，同一支向量換一套量尺，向量本身不動。
3. **算子**：$v' = R\\,v$，把向量真的轉到新方向。

最後一個觀察：不論 $R$ 是怎麼湊出來的，它都等於「繞某一條軸 $\\hat\\omega$ 轉某個角度 $\\theta$」。下一課就用 $(\\hat\\omega, \\theta)$ 來描述旋轉；先做下方的自我檢測。`,
    simHint: '切換「R 的用途」：換座標時橘色向量不動，只有它在 {b} 中的座標改變；當算子時它被轉到粉紅色的位置。青色箭頭是這個 R 的等效轉軸 ω̂（下一課的主角）。',
  },
];

const quiz: QuizQuestion[] = [
  {
    question: '平面上，$\\{b\\}$ 相對 $\\{s\\}$ 逆時針轉了 90°，原點在 $p_{sb} = (2, 0)$。某點在 $\\{b\\}$ 中的座標是 $p_b = (1, 0)$，它在 $\\{s\\}$ 中的座標是？',
    options: ['$(3, 0)$', '$(2, 1)$', '$(2, -1)$', '$(0, 3)$'],
    answer: 1,
    explanation: '$p_s = R_{sb}\\,p_b + p_{sb}$。$R_{sb}$ 把 $(1, 0)$ 轉成 $(0, 1)$（因為 $\\hat x_b$ 指向 $\\hat y_s$），再加上 $p_{sb} = (2, 0)$ 得到 $(2, 1)$。$(3, 0)$ 忘了旋轉；$(2, -1)$ 誤用了 $R_{sb}^{\\T}$；$(0, 3)$ 把 $p_{sb}$ 也一起轉了。',
  },
  {
    question: '$\\{b\\}$ 的 $\\hat z_b$ 指向 $\\{s\\}$ 的 $-\\hat x_s$，$\\hat x_b$ 指向 $\\hat y_s$。$R_{sb}$ 的第二欄（也就是 $\\hat y_b$）是多少？',
    options: ['$(0, 0, 1)$', '$(0, 0, -1)$', '$(-1, 0, 0)$', '$(0, 1, 0)$'],
    answer: 1,
    explanation: '右手座標系滿足 $\\hat y_b = \\hat z_b \\times \\hat x_b = (-1, 0, 0) \\times (0, 1, 0) = (0, 0, -1)$。三欄依序是 $(0, 1, 0)$、$(0, 0, -1)$、$(-1, 0, 0)$，可以驗證 $\\det R_{sb} = +1$。若選 $(0, 0, 1)$，三欄會構成左手座標系（$\\det = -1$）。',
  },
  {
    question: '把單位矩陣的第二、三欄對調，得到 $M = [\\,\\hat x_s\\ \\ \\hat z_s\\ \\ \\hat y_s\\,]$。$M$ 是旋轉矩陣嗎？',
    options: ['是：各欄都是單位長且兩兩垂直', '不是：$M^{\\T}M \\ne I$', '不是：$\\det M = -1$，它是鏡射', '是：它就是繞 $\\hat x$ 轉 90°'],
    answer: 2,
    explanation: '$M^{\\T}M = I$ 確實成立（各欄仍是正交單位向量），但 $\\det M = \\hat x \\cdot (\\hat z \\times \\hat y) = -1$，三欄構成左手座標系。它是對 $y = z$ 平面的鏡射，不在 $\\SO(3)$ 裡。繞 $\\hat x$ 轉 90° 的矩陣是 $[\\,\\hat x\\ \\ \\hat z\\ \\ {-\\hat y}\\,]$，差了一個負號。',
  },
  {
    question: '飛機先繞**自己的** $\\hat z$ 轉 90°（偏航），再繞**自己的** $\\hat y$ 轉 90°（俯仰）。若改用 $\\{s\\}$ 的固定軸做出同一個最終姿態，應該怎麼轉？',
    options: ['先繞 $\\hat z_s$ 轉 90°，再繞 $\\hat y_s$ 轉 90°', '先繞 $\\hat y_s$ 轉 90°，再繞 $\\hat z_s$ 轉 90°', '兩種順序都可以，結果一樣', '做不到：固定軸只能描述平面上的轉動'],
    answer: 1,
    explanation: '物體軸一路後乘：$R = \\mathrm{Rot}(\\hat z, 90^\\circ)\\,\\mathrm{Rot}(\\hat y, 90^\\circ)$。固定軸一路前乘，後做的乘在左邊；要得到同一個乘積，必須先做 $\\mathrm{Rot}(\\hat y, 90^\\circ)$、再做 $\\mathrm{Rot}(\\hat z, 90^\\circ)$，順序剛好相反。可以在第 6 步的模擬裡驗證：選 R₀ = I（這個規則是從 $R = I$ 出發才成立），設 A = 繞 $\\hat z$ 90°、B = 繞 $\\hat y$ 90° 並用後乘，再切換成前乘、按「⇄ 交換」，最終姿態完全一樣。',
  },
];

// ---------------------------------------------------------------------------
// Simulation constants
// ---------------------------------------------------------------------------

const DEG = Math.PI / 180;
const AXES: { value: AxisName; label: string }[] = [
  { value: 'x', label: 'x̂' }, { value: 'y', label: 'ŷ' }, { value: 'z', label: 'ẑ' },
];
const AXIS_TEX: Record<AxisName, string> = { x: '\\hat x', y: '\\hat y', z: '\\hat z' };
/** Tilted start orientation for the pre/post comparison: ẑ_b leans towards +ŷ_s. */
const R0_TILT = matMul(RotZ(20 * DEG), RotX(-55 * DEG));
/** Vector used in the "three uses" step (written in {s}). */
const V_DEMO: Vec = [-0.05, 0.85, 0.9];
const PLANAR_POSE: PlanarPose = { x: 1.4, y: 0.5, theta: 35 * DEG };
const PLANAR_P: Vec = [2.3, 2.1];
const S_WORLD = { xmin: -0.55, xmax: 2.95, ymin: -0.55, ymax: 2.95 };
/** Half-width of the "seen from {b}" view; it grows when P or {s} would leave it (see fitPlanarViews). */
const B_HALF = 2.6;
const B_WORLD = { xmin: -B_HALF, xmax: B_HALF, ymin: -B_HALF, ymax: B_HALF };
const KNOB = 0.95;
/** Length of the frame arrows drawn in the planar views. */
const FRAME_2D = 0.6;
/** Where {c} is mounted on {b} (in {b} coordinates); for display only. Past the x_b tip label (at ~1.14). */
const P_BC: Vec = [1.5, 0, 0];
/** Degrees per second for the sequence playback. */
const PLAY_SPEED = 75 * DEG;
/** Largest magnitude accepted for a typed matrix entry (step 4). */
const EDIT_MAX = 3;

type UseMode = 'pose' | 'coords' | 'operator';

const CSS = `
.c301 .c301-split { display: grid; grid-template-columns: 1fr 1fr; gap: 0.6rem; }
@media (max-width: 860px) { .c301 .c301-split { grid-template-columns: 1fr; } }
.c301 .c301-cap { font-size: 0.78rem; color: var(--fg-muted); margin: 0 0 0.2rem; }
.c301 .c301-eq { display: flex; flex-wrap: wrap; align-items: center; gap: 0.25rem 0.5rem; margin: 0.35rem 0; font-size: 0.85rem; }
.c301 .c301-eq .mr-matrix { margin: 0; }
.c301 .c301-eq .c301-txt { color: var(--fg-muted); font-size: 0.78rem; }
.c301 .c301-inline { display: inline-flex; flex-wrap: wrap; align-items: center; gap: 0.25rem 0.5rem; }
.c301 .c301-mat .mr-cell { padding: 0 0.15rem; border-radius: 3px; cursor: pointer; transition: background 0.15s; }
.c301 .c301-mat .mr-cell.c301-hot { background: var(--accent-soft); }
.c301 .c301-mat.c301-editing .mr-cell { cursor: auto; padding: 1px 0; }
.c301 .c301-mat input { width: 5.6em; -moz-appearance: textfield; appearance: textfield; font: inherit; font-family: var(--mono); text-align: right; color: inherit; background: var(--bg-elev); border: 1px solid var(--border); border-radius: 4px; padding: 0.05rem 0.2rem; }
.c301 .c301-mat input::-webkit-outer-spin-button, .c301 .c301-mat input::-webkit-inner-spin-button { -webkit-appearance: none; margin: 0; }
.c301 .c301-rots { display: grid; grid-template-columns: 1fr 1fr; gap: 0.35rem 0.9rem; margin: 0.3rem 0; }
@media (max-width: 520px) { .c301 .c301-rots { grid-template-columns: 1fr; } }
.c301 .c301-rot { display: flex; align-items: flex-end; gap: 0.45rem; min-width: 0; }
.c301 .c301-rot > .mr-control:last-child { flex: 1; min-width: 0; }
.c301 .c301-rot-name { font-size: 0.82rem; font-weight: 600; padding-bottom: 0.25rem; white-space: nowrap; }
.c301 .c301-pill { position: absolute; font-size: 0.75rem; padding: 0.1rem 0.55rem; border-radius: 999px; pointer-events: none; z-index: 1; }
.c301 .c301-status { left: 0.5rem; top: 0.4rem; }
.c301 .c301-status.ok { background: var(--ok-soft); color: var(--ok); }
.c301 .c301-status.bad { background: var(--danger-soft); color: var(--danger); }
.c301 .c301-anim { right: 0.5rem; top: 0.4rem; background: var(--accent-soft); color: var(--accent); }
.c301 .c301-hint3d { position: absolute; right: 0.5rem; bottom: 0.3rem; font-size: 0.72rem; color: var(--fg-muted); pointer-events: none; }
.c301 .mr-kv { margin: 0.2rem 0; }
`;

// ---------------------------------------------------------------------------
// mount
// ---------------------------------------------------------------------------

function mount(host: HTMLElement, ctx: LessonContext): LessonHandle {
  const S = {
    scenario: -1,
    mode3d: false,
    // planar
    pose: { ...PLANAR_POSE } as PlanarPose,
    P: PLANAR_P.slice(),
    pinned: false,
    // 3D builder: R_sb = R0 · A · B (body) or B · A · R0 (fixed)
    R0: eye(3),
    rotA: { axis: 'z', angle: 45 * DEG } as ElemRot,
    rotB: { axis: 'x', angle: 30 * DEG } as ElemRot,
    mode: 'body' as ComposeMode,
    ghost: false,
    // edit mode (step 4)
    editMode: false,
    Redit: eye(3),
    // column highlight
    hoverCol: null as number | null,
    stickyCol: null as number | null,
    // composition (step 5)
    showC: false,
    rbcAxis: 'z' as AxisName,
    rbcAngle: 50 * DEG,
    viewFromB: false,
    // uses (step 7)
    use: null as UseMode | null,
    showOmega: false,
  };
  let anim: { u: number } | null = null;
  let trailPts: Vec[][] = [[], [], []];

  const root = el('div', { class: 'c301' });
  root.appendChild(el('style', {}, CSS));
  host.appendChild(root);

  const titleEl = simTitle(root, '座標系 {s}、{b} 與旋轉矩陣', '平面模式');
  const badge = titleEl.querySelector('.badge') as HTMLElement;
  const chips = presetChips(root, SCENARIOS.map((sc, i) => ({ label: sc.label, title: sc.title, apply: () => applyScenario(i) })));

  // =========================================================================
  // Planar mode (steps 1–2)
  // =========================================================================
  const planarBox = el('div');
  root.appendChild(planarBox);
  const split = el('div', { class: 'c301-split' });
  planarBox.appendChild(split);
  const leftCol = el('div');
  const rightCol = el('div');
  split.append(leftCol, rightCol);
  leftCol.appendChild(el('div', { class: 'c301-cap' }, '從 {s} 看：固定的地面座標'));
  const cvS = new Canvas2D(leftCol, { aspect: 1, world: S_WORLD, overlay: true });
  rightCol.appendChild(el('div', { class: 'c301-cap' }, '從 {b} 看：量尺黏在 {b} 上'));
  const cvB = new Canvas2D(rightCol, { aspect: 1, world: B_WORLD, overlay: true });

  const eqRow = el('div', { class: 'c301-eq' });
  planarBox.appendChild(eqRow);
  const mvR2 = labelled(eqRow, 'R_{sb}');
  const mvPb = labelled(eqRow, 'p_b');
  eqRow.appendChild(mathEl('+'));
  const mvPsb = labelled(eqRow, 'p_{sb}');
  eqRow.appendChild(mathEl('='));
  const mvPs = labelled(eqRow, 'p_s');
  legend(planarBox, [
    { color: 'var(--series-4)', label: 'p_s：從 {s} 原點量到 P' },
    { color: 'var(--series-5)', label: 'p_b：從 {b} 原點量到 P（虛線沿 {b} 的軸分解）' },
    { color: 'var(--fg-muted)', label: 'p_sb：{b} 的原點' },
  ]);
  const pPanel = controlPanel(planarBox, { compact: true });
  const sTh = angleSlider(pPanel, { label: '{b} 的角度 θ', value: S.pose.theta, onInput: (v) => { S.pose.theta = v; planarChanged(); } });
  const sX = slider(pPanel, { label: 'p_sb 的 x', min: -0.5, max: 2.5, step: 0.01, value: S.pose.x, unit: 'm', onInput: (v) => { S.pose.x = v; planarChanged(); } });
  const sY = slider(pPanel, { label: 'p_sb 的 y', min: -0.5, max: 2.5, step: 0.01, value: S.pose.y, unit: 'm', onInput: (v) => { S.pose.y = v; planarChanged(); } });
  const pRow = row(pPanel);
  const pinTog = toggle(pRow, { label: '釘住點 P（只移動 {b}）', value: S.pinned, onChange: (v) => { S.pinned = v; planarChanged(); } });
  button(pRow, { label: '重設位置', small: true, onClick: () => resetPlanar() });

  const knobPos = (): Pt => [S.pose.x + KNOB * Math.cos(S.pose.theta), S.pose.y + KNOB * Math.sin(S.pose.theta)];
  // P stays below y = 2.4 so that its label never slides under the readout overlay at the top of the canvas
  const clampP = (p: ArrayLike<number>): Vec => [clamp(p[0], -0.45, 2.85), clamp(p[1], -0.45, 2.4)];
  // A view refits (fitPlanarViews) after every change, except while one of its own handles is being dragged:
  // rescaling under the pointer would make the grabbed point slide away. It catches up when the drag ends.
  let draggingIn: Canvas2D | null = null;
  const dragHooks = (cv: Canvas2D) => ({
    onStart: () => { draggingIn = cv; },
    onEnd: () => { draggingIn = null; fitPlanarViews(); },
  });
  // handles: P (accent) and the pose of {b} (cyan centre + heading knob); colours are read at draw time
  cvS.addHandle({
    get: () => [S.P[0], S.P[1]],
    set: (p) => { if (!S.pinned) { S.P = clampP(p); planarChanged(); } },
    label: 'P', radius: 8,
    ...dragHooks(cvS),
    get enabled() { return !S.pinned; },
    get color() { return cssVar('--accent', '#2563eb'); },
  });
  cvS.addHandle({
    get: () => [S.pose.x, S.pose.y],
    set: (p) => {
      S.pose.x = clamp(p[0], -0.5, 2.5); S.pose.y = clamp(p[1], -0.5, 2.5);
      sX.set(S.pose.x, { silent: true }); sY.set(S.pose.y, { silent: true });
      planarChanged();
    },
    radius: 9,
    ...dragHooks(cvS),
    get color() { return cssVar('--series-6', '#0891b2'); },
  });
  cvS.addHandle({
    get: knobPos,
    set: (p) => {
      S.pose.theta = Math.atan2(p[1] - S.pose.y, p[0] - S.pose.x);
      sTh.set(S.pose.theta, { silent: true });
      planarChanged();
    },
    radius: 6,
    ...dragHooks(cvS),
    get color() { return cssVar('--series-6', '#0891b2'); },
  });
  cvB.addHandle({
    get: () => { const pb = planarToBody(S.pose, S.P); return [pb[0], pb[1]]; },
    set: (p) => { if (!S.pinned) { S.P = clampP(planarToSpace(S.pose, p)); planarChanged(); } },
    label: 'P', radius: 8,
    ...dragHooks(cvB),
    get enabled() { return !S.pinned; },
    get color() { return cssVar('--accent', '#2563eb'); },
  });

  cvS.draw = (c) => {
    const muted = cssVar('--fg-muted', '#777');
    const amber = cssVar('--series-4', '#d97706');
    const purple = cssVar('--series-5', '#7c3aed');
    const cyan = cssVar('--series-6', '#0891b2');
    const bg = cssVar('--sim-bg', '#fff');
    const o: Pt = [S.pose.x, S.pose.y];
    const P: Pt = [S.P[0], S.P[1]];
    const pb = planarToBody(S.pose, S.P);
    const ct = Math.cos(S.pose.theta), st = Math.sin(S.pose.theta);
    c.grid(0.5);
    c.frame({ x: 0, y: 0, theta: 0 }, 0.7, { labels: ['x_s', 'y_s'], width: 2 });
    // p_sb
    c.arrow([0, 0], o, { color: muted, width: 1.5, dash: [5, 4] });
    c.text([o[0] * 0.6, o[1] * 0.6], 'p_sb', { color: muted, dx: 12, dy: 12, bg });
    // p_s and its {s} components
    c.line([0, 0], [P[0], 0], { color: amber, width: 1, dash: [3, 4], alpha: 0.55 });
    c.line([P[0], 0], P, { color: amber, width: 1, dash: [3, 4], alpha: 0.55 });
    c.arrow([0, 0], P, { color: amber, width: 2.2 });
    c.text([P[0] * 0.72, P[1] * 0.72], 'p_s', { color: amber, dx: -16, dy: -4, bg });
    // p_b and its {b} components
    const foot: Pt = [o[0] + pb[0] * ct, o[1] + pb[0] * st];
    c.line(o, foot, { color: purple, width: 1.3, dash: [4, 3] });
    c.line(foot, P, { color: purple, width: 1.3, dash: [4, 3] });
    c.arrow(o, P, { color: purple, width: 2.2 });
    c.text([(o[0] + P[0]) / 2, (o[1] + P[1]) / 2], 'p_b', { color: purple, dx: 14, dy: 4, bg });
    // {b}
    c.line(o, knobPos(), { color: cyan, width: 1, dash: [3, 3] });
    c.frame(S.pose, FRAME_2D, { labels: ['x_b', 'y_b'], width: 3 });
    c.text(o, '{b}', { color: cyan, dx: -18, dy: 14 });
    if (S.pinned) drawPinned(c, P);
    c.setOverlayText(`p_s = (${fmt(P[0], 2)}, ${fmt(P[1], 2)})${S.pinned ? '  ← 固定不變' : ''}`);
  };

  cvB.draw = (c) => {
    const muted = cssVar('--fg-muted', '#777');
    const purple = cssVar('--series-5', '#7c3aed');
    const bg = cssVar('--sim-bg', '#fff');
    const inv = invertPlanarPose(S.pose);
    const pb = planarToBody(S.pose, S.P);
    const P: Pt = [pb[0], pb[1]];
    c.grid(0.5);
    // where the ground frame {s} is, seen from {b}
    c.arrow([0, 0], [inv.x, inv.y], { color: muted, width: 1.2, dash: [5, 4] });
    c.frame(inv, FRAME_2D, { labels: ['x_s', 'y_s'], width: 2, alpha: 0.75 });
    c.text([inv.x, inv.y], '{s}', { color: muted, dx: -16, dy: -12 });
    // p_b, now along the screen axes
    c.line([0, 0], [P[0], 0], { color: purple, width: 1.3, dash: [4, 3] });
    c.line([P[0], 0], P, { color: purple, width: 1.3, dash: [4, 3] });
    c.arrow([0, 0], P, { color: purple, width: 2.2 });
    c.text([P[0] / 2, P[1] / 2], 'p_b', { color: purple, dx: -12, dy: -8, bg });
    c.frame({ x: 0, y: 0, theta: 0 }, FRAME_2D, { labels: ['x_b', 'y_b'], width: 3 });
    c.text([0, 0], '{b}', { color: cssVar('--series-6', '#0891b2'), dx: -14, dy: 12 });
    if (S.pinned) drawPinned(c, P);
    c.setOverlayText(`p_b = (${fmt(pb[0], 2)}, ${fmt(pb[1], 2)})${S.pinned ? '  ← 隨 {b} 改變' : ''}`);
  };

  function drawPinned(c: Canvas2D, P: Pt): void {
    const acc = cssVar('--accent', '#2563eb');
    c.point(P, 6, { color: acc });
    c.circle(P, 11 / c.scale, { color: acc, width: 2 });
    c.text(P, 'P（釘住）', { color: acc, dy: -17, bg: cssVar('--sim-bg', '#fff') });
  }

  function planarChanged(): void {
    const pb = planarToBody(S.pose, S.P);
    const R2 = planarR(S.pose.theta);
    mvR2.set(R2, { highlightCols: ['x', 'y'] });
    mvPb.set(pb);
    mvPsb.set([S.pose.x, S.pose.y]);
    mvPs.set(planarToSpace(S.pose, pb));
    pinTog.set(S.pinned, { silent: true });
    fitPlanarViews();
    cvS.requestRender();
    cvB.requestRender();
  }

  /**
   * Keep everything the two planar views talk about inside them. The controls let {b} sit anywhere in
   * [-0.5, 2.5]² and P anywhere in its clamp box, so with fixed windows P (|p_b| up to ~4.4 m) and {s}
   * (|p_bs| up to ~3.5 m) could leave the "seen from {b}" view, and the {b} frame could be cut off at the
   * corners of the {s} view. Each view grows (and shrinks back) as needed; the {b} view stays centred on {b}.
   */
  function fitPlanarViews(): void {
    const ct = Math.cos(S.pose.theta), st = Math.sin(S.pose.theta);
    const o: Pt = [S.pose.x, S.pose.y];
    const pb = planarToBody(S.pose, S.P);
    if (draggingIn !== cvS) {
      // P needs no entry: its clamp box was chosen for S_WORLD. Labelled points get room for their labels.
      const labelled: Pt[] = [o, knobPos(), [o[0] + FRAME_2D * ct, o[1] + FRAME_2D * st], [o[0] - FRAME_2D * st, o[1] + FRAME_2D * ct]];
      const corner: Pt = [o[0] + pb[0] * ct, o[1] + pb[0] * st]; // corner of the dashed p_b decomposition
      setWorldIfChanged(cvS, fitBox(fitBox(S_WORLD, labelled, 0.3), [corner], 0.1));
    }
    if (draggingIn !== cvB) {
      const inv = invertPlanarPose(S.pose);
      const ci = Math.cos(inv.theta), si = Math.sin(inv.theta);
      const pts: Pt[] = [
        [pb[0], pb[1]], [inv.x, inv.y],
        [inv.x + FRAME_2D * ci, inv.y + FRAME_2D * si], [inv.x - FRAME_2D * si, inv.y + FRAME_2D * ci],
      ];
      const h = Math.max(B_HALF, ...pts.map(([x, y]) => snapUp(Math.max(Math.abs(x), Math.abs(y)) + 0.4)));
      setWorldIfChanged(cvB, { xmin: -h, xmax: h, ymin: -h, ymax: h });
    }
  }

  function resetPlanar(): void {
    S.pose = { ...PLANAR_POSE };
    S.P = PLANAR_P.slice();
    sTh.set(S.pose.theta, { silent: true });
    sX.set(S.pose.x, { silent: true });
    sY.set(S.pose.y, { silent: true });
    planarChanged();
  }

  // =========================================================================
  // 3D mode (steps 3–7)
  // =========================================================================
  const spatialBox = el('div');
  root.appendChild(spatialBox);
  // phones get a taller 3D view (the pane is full width there and not sticky)
  const narrow = host.clientWidth > 0 && host.clientWidth < 520;
  const scene = new Scene3D(spatialBox, { aspect: narrow ? 1.25 : 1.7, grid: 3, axes: false, cameraPos: [2.9, -1.36, 1.86], target: [0.05, 0.05, 0.24], autoStart: false });
  const statusEl = el('div', { class: 'c301-pill c301-status ok' });
  const animEl = el('div', { class: 'c301-pill c301-anim', style: { display: 'none' } });
  scene.el.append(statusEl, animEl, el('div', { class: 'c301-hint3d' }, '拖曳：旋轉視角　滾輪：縮放'));

  const triS = new Triad(scene, { radius: 0.009, head: 0.08, opacity: 0.6, names: ['x_s', 'y_s', 'z_s'], labelVar: '--fg-muted', labelScale: 0.65 });
  const triB = new Triad(scene, { radius: 0.026, head: 0.13, names: ['x_b', 'y_b', 'z_b'] });
  const triGhost = new Triad(scene, { radius: 0.022, head: 0.12, opacity: 0.38 });
  triGhost.setOpacity(0.38);
  const triC = new Triad(scene, { radius: 0.017, head: 0.1, names: ['x_c', 'y_c', 'z_c'], labelScale: 0.65 });
  const drops = new DashedSegments(scene, '--fg-muted', 4);
  const cMount = new DashedSegments(scene, '--fg-muted', 1);
  const cube = new DashedSegments(scene, '--fg-muted', 12, { opacity: 0.85 });
  const axisLine = new DashedSegments(scene, '--series-6', 1, { dash: 0.08, gap: 0.05 });
  const axisArrow = new Arrow3D('--series-6', 0.02, 0.12);
  scene.scene.add(axisArrow.group);
  const trails = AXIS_VARS.map((v) => new Trail(scene, v, { opacity: 0.75, max: 900 }));
  const vArrow = new Arrow3D('--series-4', 0.02, 0.11);
  const vPrime = new Arrow3D('--series-7', 0.02, 0.11);
  scene.scene.add(vArrow.group, vPrime.group);
  const vLabel = label3D(scene, 'v', '--series-4');
  const vPrimeLabel = label3D(scene, "v'", '--series-7');
  const vArc = new Trail(scene, '--series-7', { dashed: true, max: 64 });
  const vProj = new DashedSegments(scene, '--series-4', 3);
  const feet = AXIS_VARS.map((v) => scene.addSphere(0.032, v));
  const omegaLine = new DashedSegments(scene, '--series-6', 1, { dash: 0.08, gap: 0.05 });
  const omegaArrow = new Arrow3D('--series-6', 0.022, 0.13);
  scene.scene.add(omegaArrow.group);
  const omegaLabel = label3D(scene, 'ω̂', '--series-6', 0.9);
  const sprites = [vLabel, vPrimeLabel, omegaLabel];

  // ---- builder (R_sb from elementary rotations) ----
  const builderBox = el('div');
  spatialBox.appendChild(builderBox);
  const r0Chips = presetChips(builderBox, [
    { label: 'R₀ = I', title: '從 {s} 的姿態開始', apply: () => setR0(eye(3)) },
    { label: 'R₀ 傾斜', title: '從一個傾斜的姿態開始', apply: () => setR0(R0_TILT) },
    { label: 'R₀ 隨機', title: '隨機的起始姿態', apply: () => setR0(randomRotation()) },
  ], { label: '起始姿態' });
  const rotsBox = el('div', { class: 'c301-rots' });
  builderBox.appendChild(rotsBox);
  const blockA = rotBlock(rotsBox, '旋轉 A', () => S.rotA);
  const blockB = rotBlock(rotsBox, '旋轉 B', () => S.rotB);
  const bPanel = controlPanel(builderBox);
  const bRow = row(bPanel);
  const modeSel = select<ComposeMode>(bRow, {
    options: [
      { value: 'body', label: '後乘 R₀·A·B：繞 {b} 的軸（物體軸）' },
      { value: 'fixed', label: '前乘 B·A·R₀：繞 {s} 的軸（固定軸）' },
    ],
    value: S.mode,
    onChange: (m) => { S.mode = m; builderChanged(); },
  });
  button(bRow, { label: '⇄ 交換 A、B', small: true, title: '交換兩次旋轉的先後順序', onClick: () => swapAB() });
  button(bRow, { label: '▶ 播放序列', small: true, variant: 'primary', title: '從 R₀ 開始，依序做 A、B', onClick: () => playSequence() });
  const ghostTog = toggle(bRow, { label: '虛影：另一種乘法', value: S.ghost, onChange: (v) => { S.ghost = v; update3D(); } });
  const seqEq = el('div', { class: 'c301-eq' });
  builderBox.appendChild(seqEq);

  // ---- the matrix row ----
  const matRow = el('div', { class: 'c301-eq' });
  spatialBox.appendChild(matRow);
  const Rw = matrixWidget(matRow, {
    label: mathEl('R_{sb} ='),
    colTones: ['x', 'y', 'z'],
    onHoverCol: (j) => { S.hoverCol = j; update3D(); },
    onClickCol: (j) => { S.stickyCol = S.stickyCol === j ? null : j; update3D(); },
    // entries are kept within ±EDIT_MAX so a typo cannot throw the frame out of view (the cell shows the clamped value on blur)
    onEdit: (i, j, v) => { S.Redit = S.Redit.map((r) => r.slice()); S.Redit[i][j] = clamp(v, -EDIT_MAX, EDIT_MAX); editChips.clear(); update3D(); },
  });
  const chainSpan = el('span', { class: 'c301-inline' });
  matRow.appendChild(chainSpan);
  chainSpan.appendChild(mathEl('\\times'));
  // R_sb × R_bc = R_sc, or (viewing from {b}) R_bs × R_sc = R_bc
  const mvMid = labelled(chainSpan, 'R_{bc}');
  chainSpan.appendChild(mathEl('='));
  const mvOut = labelled(chainSpan, 'R_{sc}');
  // KaTeX labels are re-typeset only when the reference frame of the display changes (not every frame)
  let chainFromB = false;
  let rwFromB = false;
  const colNote = note(spatialBox, '提示：滑鼠移到某一欄（或點一下固定）會高亮對應的 {b} 軸，虛線把它投影到 {s} 的三個軸上。');

  // ---- edit mode panel (step 4) ----
  const editBox = el('div');
  spatialBox.appendChild(editBox);
  const editChips = presetChips(editBox, [
    { label: '合法旋轉', title: '回到目前 A、B 組出的旋轉', apply: () => setEdit(builderR()) },
    { label: '拉長 x̂_b ×1.5', title: '第一欄乘 1.5', apply: () => setEdit(scaleCol(builderR(), 0, 1.5)) },
    { label: '剪切 ŷ_b', title: '第二欄加上 0.6 倍的第一欄（det 不變，但不正交）', apply: () => setEdit(shearCols(builderR())) },
    { label: '鏡射 ẑ_b → −ẑ_b', title: '第三欄變號：正交但 det = −1', apply: () => setEdit(scaleCol(builderR(), 2, -1)) },
  ], { label: '試試' });
  const editRow = row(controlPanel(editBox));
  button(editRow, { label: '投影回 SO(3)', small: true, variant: 'primary', title: '用 SVD 找出最近的旋轉矩陣（ProjectToSO3）', onClick: () => { editChips.clear(); setEdit(ProjectToSO3(S.Redit)); } });
  const editInfo = el('div', { class: 'c301-eq' });
  editBox.appendChild(editInfo);
  const mvRtR = labelled(editInfo, 'R^{\\T}R');
  const kvEdit = kv(editInfo, ['det R', '‖RᵀR − I‖', '欄長度', '欄內積 x·y, y·z, z·x']);

  // ---- composition panel (step 5) ----
  const chainBox = el('div');
  spatialBox.appendChild(chainBox);
  const chainPanel = el('div', { class: 'c301-rots' });
  chainBox.appendChild(chainPanel);
  const cBlock = el('div', { class: 'c301-rot' });
  chainPanel.appendChild(cBlock);
  cBlock.appendChild(el('span', { class: 'c301-rot-name' }, 'R_bc'));
  const cAxisSel = select<AxisName>(cBlock, { label: '軸', options: AXES, value: S.rbcAxis, onChange: (a) => { S.rbcAxis = a; update3D(); } });
  const cAng = angleSlider(cBlock, { label: '角度（繞 {b} 的軸）', value: S.rbcAngle, onInput: (v) => { S.rbcAngle = v; update3D(); } });
  const fromBTog = toggle(chainPanel, { label: '以 {b} 為參考（改看 R_bs = R_sbᵀ）', value: S.viewFromB, onChange: (v) => { S.viewFromB = v; update3D(); } });
  fromBTog.el.style.alignSelf = 'center';

  // ---- uses panel (step 7) ----
  const useBox = el('div');
  spatialBox.appendChild(useBox);
  const usePanel = controlPanel(useBox);
  const useRow = row(usePanel);
  useRow.appendChild(el('span', { class: 'c301-rot-name' }, 'R 的用途'));
  const useSel = select<UseMode>(useRow, {
    options: [
      { value: 'pose', label: '① 姿態：{b} 朝哪裡' },
      { value: 'coords', label: '② 換座標：v_s = R_sb v_b' },
      { value: 'operator', label: "③ 算子：v' = R v" },
    ],
    value: 'coords',
    onChange: (u) => { S.use = u; update3D(); },
  });
  const omegaTog = toggle(useRow, { label: '等效轉軸 ω̂（下一課）', value: true, onChange: (v) => { S.showOmega = v; update3D(); } });
  const kvUse = kv(useBox, ['v_s（{s} 座標）', 'v_b = R_sbᵀ v_s', "v' = R v_s", '等效轉軸 ω̂、角度 θ']);

  // ---- 3D state helpers ----
  function rots(): ElemRot[] { return [S.rotA, S.rotB]; }
  function builderR(): Mat { return composeSequence(S.R0, rots(), S.mode); }
  function Rbc(): Mat { return composeSequence(eye(3), [{ axis: S.rbcAxis, angle: S.rbcAngle }], 'body'); }

  function rotBlock(parent: HTMLElement, name: string, get: () => ElemRot) {
    const box = el('div', { class: 'c301-rot' });
    parent.appendChild(box);
    box.appendChild(el('span', { class: 'c301-rot-name' }, name));
    const sel = select<AxisName>(box, { label: '軸', options: AXES, value: get().axis, onChange: (a) => { get().axis = a; builderChanged(); } });
    const sl = angleSlider(box, { label: '角度', value: get().angle, onInput: (v) => { get().angle = v; builderChanged(); } });
    return { sel, sl };
  }

  function syncBuilderControls(): void {
    blockA.sel.set(S.rotA.axis, { silent: true });
    blockA.sl.set(S.rotA.angle, { silent: true });
    blockB.sel.set(S.rotB.axis, { silent: true });
    blockB.sl.set(S.rotB.angle, { silent: true });
    modeSel.set(S.mode, { silent: true });
    ghostTog.set(S.ghost, { silent: true });
  }

  function setR0(R: Mat): void { S.R0 = R; builderChanged(); }

  function swapAB(): void {
    const a = S.rotA;
    S.rotA = { ...S.rotB };
    S.rotB = { ...a };
    syncBuilderControls();
    builderChanged();
  }

  function builderChanged(): void {
    stopAnim();
    clearTrails();
    seqEq.replaceChildren(mathEl(sequenceTex()), el('span', { class: 'c301-txt' },
      S.mode === 'body' ? '先 A 後 B，每次都繞 {b} 當下的軸' : '先 A 後 B，每次都繞 {s} 的固定軸'));
    update3D();
  }

  function sequenceTex(): string {
    const rot = (r: ElemRot) => `\\mathrm{Rot}(${AXIS_TEX[r.axis]}, ${Math.round(r.angle / DEG)}^\\circ)`;
    // R₀ only appears in the formula when it is not the identity (it is introduced in the pre/post step)
    const r0 = approxEqual(S.R0, eye(3), 1e-12) ? '' : 'R_0';
    return S.mode === 'body'
      ? `R_{sb} = ${r0}\\,${rot(S.rotA)}\\,${rot(S.rotB)}`
      : `R_{sb} = ${rot(S.rotB)}\\,${rot(S.rotA)}\\,${r0}`;
  }

  function setEdit(M: Mat): void {
    S.Redit = M.map((r) => r.slice());
    update3D();
  }

  function scaleCol(M: Mat, j: number, k: number): Mat {
    return M.map((r) => r.map((x, c) => (c === j ? x * k : x)));
  }

  function shearCols(M: Mat): Mat {
    return M.map((r) => [r[0], r[1] + 0.6 * r[0], r[2]]);
  }

  // ---- sequence playback ----
  /** Advance the sequence progress u ∈ [0, 2] by `budget` radians of turning; zero-angle rotations are skipped. */
  function advance(u: number, budget: number): number {
    u = Math.max(0, u);
    while (u < 2) {
      const k = Math.min(1, Math.floor(u));
      const ang = Math.abs(rots()[k].angle);
      if (ang < 1e-3) { u = k + 1; continue; }
      const need = (k + 1 - u) * ang;
      if (budget < need) return u + budget / ang;
      budget -= need;
      u = k + 1;
    }
    return 2;
  }

  const loop = createLoop((dt) => {
    if (!anim) { loop.stop(); return; }
    // createLoop can report a slightly negative dt on its first frame (rAF timestamp < start time)
    anim.u = advance(anim.u, Math.max(0, dt) * PLAY_SPEED);
    const R = composeSequence(S.R0, rots(), S.mode, anim.u);
    columns(R).forEach((c, j) => trailPts[j].push(c));
    trails.forEach((t, j) => t.set(trailPts[j]));
    if (anim.u >= 2) { anim = null; loop.stop(); }
    update3D();
  });

  function playSequence(): void {
    if (S.editMode) return;
    loop.stop();
    clearTrails();
    anim = { u: 0 };
    trailPts = [[], [], []];
    columns(S.R0).forEach((c, j) => trailPts[j].push(c));
    loop.start();
  }

  function stopAnim(): void {
    if (anim) { anim = null; loop.stop(); }
  }

  function clearTrails(): void {
    trailPts = [[], [], []];
    trails.forEach((t) => t.set([]));
  }

  // ---- main 3D update ----
  function update3D(): void {
    const u = anim ? anim.u : 2;
    const Rb = S.editMode ? S.Redit : composeSequence(S.R0, rots(), S.mode, u);
    const fromB = S.viewFromB && S.showC && !S.editMode;
    const V = fromB ? transpose(Rb) : eye(3);
    const disp = (M: Mat): Mat => matMul(V, M);

    triS.set(disp(eye(3)), 1.3);
    triB.set(disp(Rb), 1);
    const ghostOn = S.ghost && !S.editMode && !S.showC && S.use == null;
    triGhost.setVisible(ghostOn);
    if (ghostOn) triGhost.set(disp(composeSequence(S.R0, rots(), S.mode === 'body' ? 'fixed' : 'body', u)), 1);
    // {c} is drawn mounted at a fixed point of {b} (P_BC, in {b} coordinates) so it visibly rides along with {b};
    // only its orientation R_sc = R_sb R_bc matters here
    const cOn = S.showC && !S.editMode;
    triC.setVisible(cOn);
    cMount.setVisible(cOn);
    if (cOn) {
      const oc = matVec(disp(Rb), P_BC);
      triC.set(disp(matMul(Rb, Rbc())), 0.55, oc);
      cMount.set([[[0, 0, 0], oc]]);
    }

    // column highlight + drop lines
    const col = S.hoverCol ?? S.stickyCol;
    triB.emphasize(col);
    Rw.setHighlight(col);
    const dropOn = col != null && !fromB;
    drops.setVisible(dropOn);
    if (dropOn) drops.set(dropLines(columns(Rb)[col]));

    // SO(3) status + unit-cube image
    const rep = so3Report(Rb);
    cube.setVisible(S.editMode);
    if (S.editMode) {
      cube.set(parallelepipedEdges(Rb));
      cube.setColor(rep.isRotation ? '--fg-muted' : '--danger');
    }
    setPill(statusEl, `c301-pill c301-status ${rep.isRotation ? 'ok' : 'bad'}`, statusText(rep));

    // animation axis
    if (anim) {
      const k = Math.max(0, Math.min(1, Math.floor(anim.u)));
      const a = matVec(V, sequenceAxisInSpace(S.R0, rots(), S.mode, k));
      axisLine.setVisible(true);
      axisLine.set([[vecScale(a, -1.45), vecScale(a, 1.45)]]);
      axisArrow.setDir([0, 0, 0], a, 1.45);
      axisArrow.visible = true;
      const r = rots()[k];
      animEl.style.display = '';
      setPill(animEl, animEl.className, `${k === 0 ? 'A' : 'B'}：繞 ${S.mode === 'body' ? `${r.axis}_b（{b} 的軸）` : `${r.axis}_s（{s} 的軸）`} 轉 ${Math.round(r.angle / DEG)}°`);
    } else {
      axisLine.setVisible(false);
      axisArrow.visible = false;
      animEl.style.display = 'none';
    }

    // three uses of R (step 7)
    updateUses(Rb, V);

    // matrices
    if (S.editMode) Rw.set(Rb);
    else Rw.set(fromB ? transpose(Rb) : Rb);
    if (fromB !== rwFromB) {
      rwFromB = fromB;
      Rw.setLabel(mathEl(fromB ? 'R_{bs} = R_{sb}^{\\T} =' : 'R_{sb} ='));
    }
    if (cOn) {
      const Rsc = matMul(Rb, Rbc());
      if (fromB !== chainFromB) {
        chainFromB = fromB;
        setTexLabel(mvMid, fromB ? 'R_{sc}' : 'R_{bc}');
        setTexLabel(mvOut, fromB ? 'R_{bc}' : 'R_{sc}');
      }
      mvMid.set(fromB ? Rsc : Rbc(), { highlightCols: ['x', 'y', 'z'] });
      mvOut.set(fromB ? Rbc() : Rsc, { highlightCols: ['x', 'y', 'z'] });
    }
    if (S.editMode) {
      mvRtR.set(rep.RtR);
      kvEdit.set('det R', fmt(rep.det, 3) + (rep.det < 0 ? '（鏡射）' : ''));
      kvEdit.set('‖RᵀR − I‖', fmt(rep.orthoError, 3));
      kvEdit.set('欄長度', rep.colNorms.map((x) => fmt(x, 3)).join(', '));
      kvEdit.set('欄內積 x·y, y·z, z·x', rep.colDots.map((x) => fmt(x, 3)).join(', '));
    }
    scene.requestRender();
  }

  function statusText(rep: ReturnType<typeof so3Report>): string {
    if (rep.isRotation) return '✓ R ∈ SO(3)：RᵀR = I，det R = 1';
    if (rep.isReflection) return '✗ det R = −1：鏡射（左手座標系），不是旋轉';
    const why: string[] = [];
    if (rep.colNorms.some((n) => Math.abs(n - 1) > 1e-3)) why.push('欄長度 ≠ 1');
    if (rep.colDots.some((d) => Math.abs(d) > 1e-3)) why.push('欄不互相垂直');
    if (rep.det <= 0) why.push(`det = ${fmt(rep.det, 2)}`);
    return `✗ 不是旋轉：${why.join('、') || 'RᵀR ≠ I'}`;
  }

  function updateUses(Rb: Mat, V: Mat): void {
    const on = S.use != null && !S.editMode;
    const coords = on && S.use === 'coords';
    const oper = on && S.use === 'operator';
    const vS = matVec(V, V_DEMO);
    vArrow.setFromTo([0, 0, 0], vS);
    vArrow.visible = coords || oper;
    vLabel.visible = coords || oper;
    vLabel.position.set(vS[0] * 1.1, vS[1] * 1.1, vS[2] * 1.1 + 0.06);
    // projections of v onto the {b} axes (its {b} coordinates)
    vProj.setVisible(coords);
    feet.forEach((f) => { f.visible = coords; });
    if (coords) {
      const cols = columns(matMul(V, Rb));
      const segs: [Vec, Vec][] = cols.map((c) => [vS, vecScale(c, dot(vS, c))]);
      vProj.set(segs);
      segs.forEach((s, j) => feet[j].position.set(s[1][0], s[1][1], s[1][2]));
    }
    // operator: v' = R v and the arc it travels
    const vP = matVec(V, matVec(Rb, V_DEMO));
    vPrime.setFromTo([0, 0, 0], vP);
    vPrime.visible = oper;
    vPrimeLabel.visible = oper;
    vPrimeLabel.position.set(vP[0] * 1.1, vP[1] * 1.1, vP[2] * 1.1 + 0.06);
    vArc.setVisible(oper);
    if (oper) vArc.set(rotationArc(Rb, V_DEMO, 48).map((p) => matVec(V, p)));
    // equivalent axis ω̂ (preview of the next lesson)
    const eq = equivalentAxisAngle(Rb);
    const omegaOn = on && S.showOmega && eq.axis != null;
    omegaLine.setVisible(omegaOn);
    omegaArrow.visible = omegaOn;
    omegaLabel.visible = omegaOn;
    if (omegaOn && eq.axis) {
      const a = matVec(V, eq.axis);
      omegaLine.set([[vecScale(a, -1.5), vecScale(a, 1.5)]]);
      omegaArrow.setDir([0, 0, 0], a, 1.5);
      omegaLabel.position.set(a[0] * 1.65, a[1] * 1.65, a[2] * 1.65);
    }
    if (on) {
      const vb = matVec(transpose(Rb), V_DEMO);
      kvUse.set('v_s（{s} 座標）', fmtVec(V_DEMO, 2));
      kvUse.set('v_b = R_sbᵀ v_s', fmtVec(vb, 2));
      kvUse.set("v' = R v_s", fmtVec(matVec(Rb, V_DEMO), 2));
      kvUse.set('等效轉軸 ω̂、角度 θ', eq.axis ? `${fmtVec(eq.axis, 2)}，${fmt(eq.angle / DEG, 1)}°` : 'R = I：沒有轉動');
    }
  }

  // =========================================================================
  // Scenarios (one per step) and visibility
  // =========================================================================
  function syncVisibility(): void {
    const sc = S.scenario;
    planarBox.style.display = S.mode3d ? 'none' : '';
    spatialBox.style.display = S.mode3d ? '' : 'none';
    badge.textContent = S.mode3d ? '3D' : '平面模式';
    builderBox.style.display = S.editMode ? 'none' : '';
    // the full sequence editor (start pose, pre/post, playback) belongs to step 6; other steps only need A and B
    const fullBuilder = sc === 5 || sc < 2;
    r0Chips.el.style.display = fullBuilder ? '' : 'none';
    bPanel.style.display = fullBuilder ? '' : 'none';
    seqEq.style.display = fullBuilder || sc === 2 ? '' : 'none';
    editBox.style.display = S.editMode ? '' : 'none';
    chainBox.style.display = S.showC && !S.editMode ? '' : 'none';
    chainSpan.style.display = S.showC && !S.editMode ? '' : 'none';
    useBox.style.display = S.use != null ? '' : 'none';
    colNote.style.display = sc === 2 ? '' : 'none';
    Rw.setEditable(S.editMode);
    // focus outline on the element the step talks about
    const targets: (HTMLElement | null)[] = [cvS.el, cvB.el, Rw.el, Rw.el, chainPanel, modeSel.el, useSel.el];
    for (const t of new Set(targets)) t?.classList.remove('mr-focus');
    targets[sc]?.classList.add('mr-focus');
  }

  function applyScenario(i: number): void {
    S.scenario = i;
    stopAnim();
    clearTrails();
    S.hoverCol = null;
    if (i <= 1) {
      S.mode3d = false;
      S.pinned = i === 1;
      syncVisibility();
      planarChanged();
      return;
    }
    S.mode3d = true;
    S.editMode = false;
    S.showC = false;
    S.viewFromB = false;
    S.use = null;
    S.ghost = false;
    S.stickyCol = null;
    if (i === 2) {
      S.R0 = eye(3);
      S.rotA = { axis: 'z', angle: 40 * DEG };
      S.rotB = { axis: 'x', angle: -60 * DEG };
      S.mode = 'body';
      S.stickyCol = 0;
      r0Chips.activate(0);
    } else if (i === 3) {
      S.editMode = true;
      S.Redit = builderR();
      editChips.activate(0);
    } else if (i === 4) {
      // keep the learner's A and B, but compose them plainly: R_sb = Rot_A Rot_B
      S.R0 = eye(3);
      S.mode = 'body';
      S.showC = true;
      r0Chips.activate(0);
    } else if (i === 5) {
      S.R0 = R0_TILT;
      S.rotA = { axis: 'z', angle: 90 * DEG };
      S.rotB = { axis: 'x', angle: 0 };
      S.mode = 'body';
      S.ghost = true;
      r0Chips.activate(1);
    } else if (i === 6) {
      S.R0 = eye(3);
      S.rotA = { axis: 'z', angle: 30 * DEG };
      S.rotB = { axis: 'x', angle: 70 * DEG };
      S.mode = 'body';
      S.use = 'coords';
      S.showOmega = true;
      r0Chips.activate(0);
    }
    fromBTog.set(S.viewFromB, { silent: true });
    cAxisSel.set(S.rbcAxis, { silent: true });
    cAng.set(S.rbcAngle, { silent: true });
    if (S.use) useSel.set(S.use, { silent: true });
    omegaTog.set(S.showOmega, { silent: true });
    syncBuilderControls();
    syncVisibility();
    builderChanged();
    if (i === 5) playSequence();
  }

  // ---- go ----
  planarChanged();
  const onStep = (i: number) => chips.activate(Math.max(0, Math.min(SCENARIOS.length - 1, i)));
  onStep(ctx.currentStep);

  return {
    onStep,
    destroy() {
      loop.stop();
      cvS.destroy();
      cvB.destroy();
      [triS, triB, triGhost, triC].forEach((t) => t.dispose());
      sprites.forEach(disposeSprite);
      scene.destroy();
      root.remove();
    },
  };
}

const SNAP = 0.25;
const snapUp = (v: number): number => Math.ceil(v / SNAP - 1e-9) * SNAP;
const snapDown = (v: number): number => Math.floor(v / SNAP + 1e-9) * SNAP;

/** `base` grown to contain every point padded by `pad`; grown edges snap outwards to 0.25 m so the view does not jitter. */
function fitBox(base: World, pts: Pt[], pad: number): World {
  let { xmin, xmax, ymin, ymax } = base;
  for (const [x, y] of pts) {
    xmin = Math.min(xmin, snapDown(x - pad));
    xmax = Math.max(xmax, snapUp(x + pad));
    ymin = Math.min(ymin, snapDown(y - pad));
    ymax = Math.max(ymax, snapUp(y + pad));
  }
  return { xmin, xmax, ymin, ymax };
}

function setWorldIfChanged(cv: Canvas2D, w: World): void {
  const c = cv.world;
  if (c.xmin !== w.xmin || c.xmax !== w.xmax || c.ymin !== w.ymin || c.ymax !== w.ymax) cv.setWorld(w);
}

/** Write a status pill only when its class or text actually changes (update3D runs every animation frame). */
function setPill(pill: HTMLElement, cls: string, text: string): void {
  if (pill.className !== cls) pill.className = cls;
  if (pill.textContent !== text) pill.textContent = text;
}

/** A toolkit matrixView whose label is typeset with KaTeX. */
function labelled(parent: HTMLElement, tex: string): MatrixHandle {
  const mv = matrixView(parent, { digits: 2 });
  setTexLabel(mv, tex);
  return mv;
}

function setTexLabel(mv: MatrixHandle, tex: string): void {
  mv.el.querySelector('.mr-mlabel')?.replaceChildren(mathEl(tex));
}

const SCENARIOS: { label: string; title: string }[] = [
  { label: '① 位姿', title: '平面：拖曳 {b} 與點 P' },
  { label: '② 同一點', title: '平面：P 釘住，只移動 {b}' },
  { label: '③ 欄位', title: '3D：R 的欄位就是 {b} 的三個軸' },
  { label: '④ SO(3)', title: '3D：直接編輯矩陣，看何時不再是旋轉' },
  { label: '⑤ 合成', title: '3D：R_sc = R_sb R_bc 與 R_bs = R_sbᵀ' },
  { label: '⑥ 前乘／後乘', title: '3D：固定軸與物體軸' },
  { label: '⑦ 用途', title: '3D：姿態、換座標、算子' },
];

const lesson: Lesson = { meta, steps, quiz, mount };
export default lesson;
