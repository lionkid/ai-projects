import type { Lesson, LessonContext, LessonHandle, LessonStep, QuizQuestion } from '@/types';
import {
  el, cssVar, fmt, controlPanel, row, slider, select, toggle, button, simTitle, legend, kv, mdElement,
  Canvas2D, createLoop, presetChips, planarArmPoints, type Pt, type Handle, type PresetChipsHandle,
} from '@/ui';
import { clamp, wrapToPi } from '@/mr';
import { meta } from './meta';
import {
  annulus, atanTheta1, beta, byBranch, choose, elbowOf, followSegment, newton2R, pickNearest, quadrantOf,
  snapToBoundary, solve2R, solve3RDecoupled, tipOf, type Branch, type FollowMode, type IK2R, type Solution,
  type Solution3R,
} from './ik2r';

const D = Math.PI / 180;
/** Last link of the planar 3R used for the wrist-decoupling demo (step 6). */
const L3 = 0.5;
/** Targets within this many screen px of a boundary circle snap onto it. */
const SNAP_PX = 6;
/** A torus distance above this between 1 cm sub-steps counts as a jump of the displayed solution. */
const JUMP_TOL = 0.35;
/** Distance of the orientation knob from the 3R target (world units). */
const KNOB = 0.45;
/** Fixed initial guess for the Newton preview. */
const NEWTON_GUESS: [number, number] = [0, 45 * D];
const PATH_SECONDS = 5;

interface PathPreset { label: string; title: string; a: Pt; b: Pt; L: [number, number] }
const PATHS: PathPreset[] = [
  { label: '穿過原點', title: 'L₁ = L₂ = 1 m，直線正好經過基座', a: [-1.5, 0], b: [1.5, 0], L: [1, 1] },
  { label: '擦過原點（y = 0.25）', title: '直線離基座 0.25 m，始終留在環帶內部', a: [-1.5, 0.25], b: [1.5, 0.25], L: [1, 1] },
  { label: '穿出外圓再回來', title: '兩端在外圓之外，中段在環帶內', a: [1.45, -1.75], b: [1.45, 1.75], L: [1, 1] },
  { label: '穿過內洞（L₂ = 0.6）', title: 'L₂ = 0.6 m：內圓半徑 0.4 m，中段不可達', a: [-1.2, 0.1], b: [1.2, 0.1], L: [1, 0.6] },
];

const steps: LessonStep[] = [
  {
    title: '給一個點，找兩組角度',
    md: `**逆向運動學**（inverse kinematics, IK）把正向運動學倒過來問：末端要停在 $(x, y)$，關節角該是多少？正向運動學 $\\theta \\mapsto (x, y)$ 只有一個答案；反方向卻常常不只一個。

平面 2R 手臂是最小的例子：目標點在可達範圍內時，手肘可以彎向「基座–目標連線」的這一側或另一側，兩種姿勢的末端落在同一點。畫布用實線畫目前顯示的解、虛線畫另一個解；旁邊的關節空間圖把兩組 $(\\theta_1, \\theta_2)$ 標成兩個點——關節空間裡的兩個點，對應到工作空間的同一個點。`,
    simHint: '拖曳紫色目標點到不同位置：實線是目前顯示的解、虛線是另一解；右側關節空間圖的兩個點會跟著移動。',
  },
  {
    title: '餘弦定理給 θ₂',
    md: `基座、手肘與末端構成三角形，三邊是 $L_1$、$L_2$ 與 $r = \\sqrt{x^2 + y^2}$，手肘處的內角是 $\\pi - |\\theta_2|$。由餘弦定理：

$$\\cos\\theta_2 = \\frac{x^2 + y^2 - L_1^2 - L_2^2}{2L_1L_2}.$$

右邊只和距離 $r$ 有關：$\\theta_2$ 的大小只看「目標離基座多遠」，與方向無關。因為 $\\cos$ 是偶函數，$+\\theta_2$ 與 $-\\theta_2$ 都滿足這個式子，**選哪個正負號就是選分支**。本站沿用數學函式庫的命名：$\\theta_2 > 0$ 叫 elbow-down（目標在右方時手肘在連線下方），$\\theta_2 < 0$ 叫 elbow-up。`,
    simHint: '「三角形」已開啟：按 c₂ = 1、c₂ = 0、c₂ = −1 三個預設或把目標拉近拉遠，看 cos θ₂ 與 θ₂ 的讀數；用「顯示的解」切換 elbow-down / elbow-up。',
  },
  {
    title: 'atan2 給 θ₁',
    md: `有了 $\\theta_2$，$\\theta_1$ 就是「目標方向角 $\\gamma$」減去「三角形在基座處的夾角 $\\beta$」：

$$\\theta_1 = \\atantwo(y, x) - \\atantwo(L_2\\sin\\theta_2,\\ L_1 + L_2\\cos\\theta_2).$$

這裡必須用雙引數的 $\\atantwo$。單引數的 $\\arctan(y/x)$ 只看比值，而 $(x, y)$ 與 $(-x, -y)$ 的比值相同，所以它只會回答 $(-90^\\circ, 90^\\circ)$ 之間的角度。目標在左半平面（$x < 0$）時答案差了 $180^\\circ$，整隻手臂轉到對面，末端落在對稱點 $(-x, -y)$ 上。$\\atantwo$ 同時看 $x$、$y$ 的正負號，四個象限都分得清。`,
    simHint: '目標已移到左半平面：紅色虛線是誤用 atan(y/x) 算出的手臂，末端跑到 (−x, −y)。按「右半平面」比較，並對照角度弧 γ、β、θ₁。',
  },
  {
    title: '可達範圍：環帶與邊界',
    md: `$|\\cos\\theta_2| \\le 1$ 才有實數解；換成距離就是

$$|L_1 - L_2| \\le \\sqrt{x^2 + y^2} \\le L_1 + L_2 .$$

可達區域是一個**環帶（annulus）**：外圓對應手臂伸直（$\\theta_2 = 0$），內圓對應完全摺回（$\\theta_2 = \\pm\\pi$）。環帶內部有兩組解；在邊界上 $\\sin\\theta_2 = 0$，兩組解合而為一；環帶外 $|\\cos\\theta_2| > 1$，無解。$L_1 = L_2$ 時內圓縮成基座一點：原點可達，而且 $\\theta_1$ 任意，有無限多解。實作時記得把 $\\cos\\theta_2$ clamp 到 $[-1, 1]$，否則邊界上的捨入誤差會讓 $\\arccos$ 傳回 NaN。`,
    simHint: '依序按「外邊界」「內邊界」「太遠」「內洞」；也可以把 L₂ 調得比 L₁ 短，內圓半徑就是 |L₁ − L₂|。目標靠近圓時會被吸到圓上。',
  },
  {
    title: '拖著走：分支與連續性',
    md: `真正動起來時，我們要關節角**連續**變化。在關節空間裡，兩個分支是 $\\theta_2 > 0$ 與 $\\theta_2 < 0$ 兩條帶子，被 $\\theta_2 = 0$（伸直）與 $\\theta_2 = \\pm\\pi$（摺疊）兩條奇異線隔開。連續的關節路徑要換分支，就得穿過奇異線，也就是末端必須碰到環帶的邊界。

所以末端路徑留在環帶內部時，鎖定一個分支就能平順跟隨。但若 $L_1 = L_2$ 而路徑正好穿過原點，鎖定分支會讓 $\\theta_1$ 瞬間跳 $180^\\circ$；想保持連續，就得讓 $\\theta_2$ 越過 $\\pm 180^\\circ$，自動換到另一個分支。`,
    simHint: '已用「鎖定 elbow-down」走完「穿過原點」：關節空間的軌跡在中途斷開，灰色虛影是跳之前的姿勢。改選「連續追蹤」比較；按 ▶ 可重播動畫。',
  },
  {
    title: '六軸手臂怎麼辦，以及數值方法',
    md: `腕部三軸交於一點的 6R 手臂（PUMA 型）可以拆成兩半：先由目標位姿算出**腕心**，用前三軸把腕心送到位（肩部左右 × 手肘上下，最多 4 組）；再用球形腕的後三軸對準姿態，每組還有兩種腕部翻轉，合計最多 8 組解。

畫布是平面版的同一招：姿態 $\\phi$ 決定最後一節的方向，腕心 $w = p - L_3(\\cos\\phi, \\sin\\phi)$ 交給 2R 公式，再取 $\\theta_3 = \\phi - \\theta_1 - \\theta_2$。沒有這種幾何結構的手臂，就要用 Jacobian 反覆修正的數值法——下一課的主題。`,
    simHint: '「3R 腕心分解」：拖曳目標與朝向旋鈕，看腕心 w 何時離開環帶；再按「數值迭代預覽」看 Newton 法幾步逼近目標，最後做下方測驗。',
  },
];

const quiz: QuizQuestion[] = [
  {
    question: '$L_1 = L_2 = 1$ m 的 2R 手臂，末端沿著通過基座的直線從 $(-1, 0)$ 走到 $(1, 0)$，並要求關節角連續變化。下列何者正確？',
    options: [
      '全程鎖定 elbow-down 即可：整條路徑都在環帶內，$\\theta$ 自然連續',
      '過原點時讓 $\\theta_2$ 越過 $\\pm180^\\circ$、換到 elbow-up；硬鎖分支則 $\\theta_1$ 跳 $180^\\circ$',
      '做不到：原點不在工作空間內，手臂根本到不了 $(0, 0)$，路徑必須繞開基座',
      '在原點先把 $\\theta_2$ 歸零（伸直）再繼續，分支就能保持不變，關節角也不會跳',
    ],
    answer: 1,
    explanation: '路徑看似都在可達區內，但 $L_1 = L_2$ 時原點正是縮成一點的內圓，也就是 $\\theta_2 = \\pm180^\\circ$ 的奇異線。這時 $\\beta = \\theta_2/2$，同一分支上 $\\theta_1 = \\gamma \\mp |\\theta_2|/2$；末端越過原點時方向角 $\\gamma$ 突變 $180^\\circ$，鎖定的分支只好讓 $\\theta_1$ 也跳 $180^\\circ$。若讓 $\\theta_2$ 從 $180^\\circ$ 繼續轉、包裹成 $-180^\\circ$（換到 elbow-up），$\\theta_1$ 就接得上：手肘停在原來那一側，只是分支標籤換了。原點本身可達，只是那一點的 $\\theta_1$ 可以任意；在原點把 $\\theta_2$ 歸零則會讓末端離開原點，做不到。',
  },
  {
    question: '同一支 $L_1 = L_2 = 1$ m 的手臂加上關節限制 $|\\theta_2| \\le 90^\\circ$。哪些目標點仍然可達？每點有幾組解？',
    options: [
      '整個圓盤 $r \\le 2$，每點剩一組解',
      '$r \\le \\sqrt{2}$ 的圓盤，每點兩組解',
      '$\\sqrt{2} \\le r \\le 2$ 的環帶，內部仍有兩組解',
      '只有 $r = \\sqrt{2}$ 那一圈',
    ],
    answer: 2,
    explanation: '$|\\theta_2| \\le 90^\\circ \\iff \\cos\\theta_2 \\ge 0 \\iff r^2 \\ge L_1^2 + L_2^2 = 2$。兩個分支是 $\\pm\\theta_2$，大小相同，所以這個限制會同時保留或同時刪掉兩組解：$\\sqrt 2 \\le r < 2$ 仍有兩組（$r = \\sqrt 2$ 時是 $\\theta_2 = \\pm 90^\\circ$），$r = 2$ 合而為一，$r < \\sqrt 2$ 則完全到不了。',
  },
  {
    question: '目標在 $(-1.2, -0.5)$。若把 $\\gamma = \\atantwo(y, x)$ 誤寫成 $\\arctan(y/x)$，其餘照公式計算，末端最後會停在哪裡？',
    options: [
      '$(1.2, 0.5)$：角度差 $180^\\circ$，整隻手臂轉到對面',
      '$(-1.2, -0.5)$，對這一點兩者相同',
      '$(-1.2, 0.5)$：只有 $y$ 的正負號錯',
      '算不出來，$\\arctan$ 會傳回 NaN',
    ],
    answer: 0,
    explanation: '$\\arctan(-0.5/-1.2) = \\arctan(0.417) \\approx 22.6^\\circ$，但真正的方向是 $22.6^\\circ - 180^\\circ = -157.4^\\circ$（第三象限）。$\\theta_2$ 只看距離，不受影響；$\\theta_1$ 整體差了 $180^\\circ$，等於把正確的手臂繞基座轉半圈，所以末端落在對稱點 $(1.2, 0.5)$。',
  },
  {
    question: 'PUMA 型 6R 手臂（腕部三軸交於一點）一般最多有幾組 IK 解？理由是什麼？',
    options: [
      '2 組：和 2R 一樣，只有手肘朝上、朝下兩種，腕部沒有選擇',
      '6 組：每個關節各自貢獻一種姿勢選擇，6 個關節就是 6 組',
      '無限多組：6 個關節對 6 個約束，必定有冗餘',
      '8 組：肩部 2 × 手肘 2 × 腕部翻轉 2',
    ],
    answer: 3,
    explanation: '腕部三軸交於一點時，腕心位置與後三軸無關，所以先用前三軸解腕心：轉盤可以面向或背對腕心（2 種），手肘再有上、下兩種，共 4 組。接著後三軸只負責姿態，球形腕對同一個旋轉通常有兩組解（$\\theta_5$ 與 $-\\theta_5$，$\\theta_4$、$\\theta_6$ 各差 $180^\\circ$），所以 $4 \\times 2 = 8$。6 個關節對 6 個約束並沒有冗餘，解是有限個。',
  },
];

const CARD_MD = `**6R（PUMA 型）手臂怎麼拆**

1. 腕部三軸交於一點 → 腕心 $p_w = p - d\\,(R\\hat z)$ 只由前三軸決定。
2. 前三軸把腕心送到位：肩部左／右 × 手肘上／下，最多 4 組。
3. 後三軸（球形腕）實現剩下的旋轉 $R_{03}^{\\T}R$：每組再有兩種腕部翻轉，最多 **8** 組。`;

type Sub = 'arm2' | 'wrist3' | 'newton';

interface Colors {
  down: string; up: string; target: string; danger: string; ann: string; tri: string; trail: string;
  fg: string; muted: string; bg: string; warn: string; cyan: string; newton: string;
}

function readColors(): Colors {
  return {
    down: cssVar('--series-1', '#2563eb'),
    up: cssVar('--series-4', '#d97706'),
    target: cssVar('--series-5', '#7c3aed'),
    danger: cssVar('--danger', '#b91c1c'),
    ann: cssVar('--series-3', '#16a34a'),
    tri: cssVar('--series-6', '#0891b2'),
    trail: cssVar('--series-7', '#db2777'),
    fg: cssVar('--fg', '#222'),
    muted: cssVar('--fg-muted', '#666'),
    bg: cssVar('--sim-bg', '#fff'),
    warn: cssVar('--warn', '#b45309'),
    cyan: cssVar('--series-6', '#0891b2'),
    newton: cssVar('--series-5', '#7c3aed'),
  };
}

const worldFor = (reach: number) => {
  const m = Math.max(1.0, reach * 1.1);
  return { xmin: -m, xmax: m, ymin: -m, ymax: m };
};

/** Fills the ring rIn ≤ r ≤ rOut around the origin (even-odd rule). */
function fillAnnulus(c: Canvas2D, rIn: number, rOut: number, color: string, alpha: number): void {
  const ctx = c.ctx;
  const [ox, oy] = c.toScreen([0, 0]);
  ctx.save();
  ctx.globalAlpha = alpha;
  ctx.fillStyle = color;
  ctx.beginPath();
  ctx.arc(ox, oy, rOut * c.scale, 0, Math.PI * 2);
  if (rIn > 1e-9) { ctx.moveTo(ox + rIn * c.scale, oy); ctx.arc(ox, oy, rIn * c.scale, 0, Math.PI * 2, true); }
  ctx.fill('evenodd');
  ctx.restore();
}

type ArmStyle = 'solid' | 'dashed' | 'ghost';

function drawArm(c: Canvas2D, lengths: number[], thetas: number[], color: string, style: ArmStyle, alpha = 1): Pt[] {
  const pts = planarArmPoints(lengths, thetas);
  const n = pts.length - 1;
  if (style === 'solid') {
    for (let i = 0; i < n; i++) c.line(pts[i], pts[i + 1], { color, width: 7, alpha: 0.85 * alpha });
    for (let i = 0; i < n; i++) {
      c.point(pts[i], 5.5, { color: cssVar('--bg-elev', '#fff'), alpha });
      c.circle(pts[i], 5.5 / c.scale, { color: cssVar('--fg', '#222'), width: 1.8, alpha });
    }
    c.point(pts[n], 5, { color, alpha });
  } else if (style === 'dashed') {
    for (let i = 0; i < n; i++) c.line(pts[i], pts[i + 1], { color, width: 2.5, dash: [7, 5], alpha });
    for (let i = 1; i <= n; i++) c.circle(pts[i], 4 / c.scale, { color, width: 1.5, alpha });
  } else {
    for (let i = 0; i < n; i++) c.line(pts[i], pts[i + 1], { color, width: 5, alpha: 0.3 * alpha });
    for (let i = 1; i <= n; i++) c.point(pts[i], 3, { color, alpha: 0.45 * alpha });
  }
  return pts;
}

function drawBase(c: Canvas2D): void {
  const g = cssVar('--grid-strong', '#bbb');
  c.polygon([[-0.1, -0.11], [0.1, -0.11], [0, 0]], { fill: g, color: g, width: 1 });
  c.line([-0.16, -0.11], [0.16, -0.11], { color: cssVar('--fg-muted'), width: 2 });
  c.frame({ x: 0, y: 0, theta: 0 }, 0.28, { width: 1.5, alpha: 0.8 });
}

function mount(host: HTMLElement, ctx: LessonContext): LessonHandle {
  // ------------------------------------------------------------------ state
  let L1 = 1;
  let L2 = 1;
  let target: Pt = [1.2, 0.9];
  let phi = 30 * D;
  let mode: FollowMode = 'down';
  let sub: Sub = 'arm2';
  let ik: IK2R = solve2R(L1, L2, target[0], target[1]);
  let cur: Solution = choose(ik, mode, null);
  let cur3: Solution | null = null;
  const show = { other: true, tri: false, arcs: false, atan: false };
  let jumpGhost: Solution | null = null;
  let lastBranch: Branch | null = ik.reach === 'two' ? cur.branch : null;
  let step = -1;
  let path: PathPreset | null = null;
  let pathS = 0;
  const trails: { elbow: Pt[][]; tip: Pt[][]; js: Pt[][] } = { elbow: [[]], tip: [[]], js: [[]] };
  let trailCount = 0;

  // ------------------------------------------------------------------ layout
  simTitle(host, '2R 解析 IK：一個目標，兩組角度', '平面 2R');
  const top = el('div', { style: { display: 'flex', flexWrap: 'wrap', gap: '0.6rem', alignItems: 'flex-start' } });
  host.appendChild(top);
  const mainBox = el('div', { style: { flex: '2 1 320px', minWidth: '0' } });
  const sideBox = el('div', { style: { flex: '1 1 190px', minWidth: '0' } });
  top.append(mainBox, sideBox);
  const cv = new Canvas2D(mainBox, { aspect: 1, world: worldFor(L1 + L2), overlay: true });
  const insetWrap = el('div', { style: { maxWidth: '290px', margin: '0 auto' } });
  sideBox.appendChild(insetWrap);
  insetWrap.appendChild(el('div', { class: 'mr-note', style: { margin: '0 0 0.2rem' } }, '關節空間 (θ₁, θ₂)：一個點 = 一個姿勢'));
  const js = new Canvas2D(insetWrap, { aspect: 1, world: { xmin: -Math.PI - 0.95, xmax: Math.PI + 0.25, ymin: -Math.PI - 0.95, ymax: Math.PI + 0.25 } });
  legend(insetWrap, [
    { color: 'var(--series-1)', label: 'elbow-down（θ₂ > 0）' },
    { color: 'var(--series-4)', label: 'elbow-up（θ₂ < 0）' },
    { color: 'var(--series-3)', label: '可達環帶' },
    { color: 'var(--danger)', label: '不可達／奇異線' },
  ]);
  const K = {
    r: 'r', ring: '環帶', reach: '可達？', c2: 'cos θ₂', t2: 'θ₂ = ±acos', g: 'γ = atan2(y,x)', b: 'β', t1d: 'θ₁（down）',
    t1u: 'θ₁（up）', atan: 'atan(y/x)', shown: '顯示中',
  } as const;
  const kvA = kv(host, Object.values(K));
  const KB = { p: 'p, φ', w: 'w（腕心）', rw: '|w|', d: 'θ（down）', u: 'θ（up）' } as const;
  const kvB = kv(host, Object.values(KB));
  kvB.el.style.display = 'none';
  // Two key/value pairs per row when there is room (keeps the pane short), one pair on phones. The
  // style change is deferred to the next frame (and skipped when nothing changes) so it never
  // re-triggers this observer inside its own callback.
  let kvCols = '';
  let kvRaf = 0;
  const applyCols = () => {
    const cols = host.clientWidth >= 520 ? 'auto 1fr auto 1fr' : 'auto 1fr';
    if (cols === kvCols) return;
    kvCols = cols;
    kvA.el.style.gridTemplateColumns = cols;
    kvB.el.style.gridTemplateColumns = cols;
  };
  applyCols();
  const ro = new ResizeObserver(() => { cancelAnimationFrame(kvRaf); kvRaf = requestAnimationFrame(applyCols); });
  ro.observe(host);
  kvA.el.style.margin = kvB.el.style.margin = '0.5rem 0 0';

  const panel = controlPanel(host, { compact: true });
  const chipRow = row(panel);
  const optRow = row(panel);
  const selMode = select<FollowMode>(optRow, {
    label: '顯示的解',
    value: mode,
    options: [
      { value: 'down', label: '鎖定 elbow-down' },
      { value: 'up', label: '鎖定 elbow-up' },
      { value: 'nearest', label: '連續追蹤（最近解）' },
    ],
    onChange: (v) => setMode(v),
  });
  const tOther = toggle(optRow, { label: '另一解（虛線）', value: show.other, onChange: (v) => { show.other = v; redraw(); } });
  const tTri = toggle(optRow, { label: '三角形', value: show.tri, onChange: (v) => { show.tri = v; redraw(); } });
  const tArcs = toggle(optRow, { label: '角度弧', value: show.arcs, onChange: (v) => { show.arcs = v; redraw(); } });
  const tAtan = toggle(optRow, { label: 'atan 對照', value: show.atan, onChange: (v) => { show.atan = v; redraw(); } });
  const sL1 = slider(panel, { label: 'L₁', min: 0.3, max: 1.5, step: 0.05, value: L1, unit: 'm', onInput: (v) => lengthsFromSlider(v, L2) });
  const sL2 = slider(panel, { label: 'L₂', min: 0.3, max: 1.5, step: 0.05, value: L2, unit: 'm', onInput: (v) => lengthsFromSlider(L1, v) });
  const pathRow = row(panel);
  const playBtn = button(pathRow, { label: '▶ 播放路徑', small: true, variant: 'primary', onClick: () => togglePlay() });
  button(pathRow, { label: '⟲ 回到起點', small: true, onClick: () => resetPath() });
  button(pathRow, { label: '清除軌跡', small: true, onClick: () => { clearTrails(); redraw(); } });
  const msgEl = el('div', { class: 'mr-note', style: { fontWeight: '600', display: 'none' } });
  host.appendChild(msgEl);
  const card = el('div', {
    style: { border: '1px solid var(--border)', borderRadius: 'var(--radius-sm)', padding: '0.2rem 0.8rem', margin: '0.3rem 0', background: 'var(--bg-sunken)', fontSize: '0.9rem' },
  }, mdElement(CARD_MD));
  host.appendChild(card);

  // ------------------------------------------------------------------ handles
  const knobPos = (): Pt => [target[0] + KNOB * Math.cos(phi), target[1] + KNOB * Math.sin(phi)];
  const targetHandle: Handle = {
    get: () => target,
    set: (p) => moveTarget(p, true),
    onStart: () => { stopPath(); if (sub === 'arm2') { jumpGhost = null; setMsg(''); } },
    label: '目標',
    radius: 9,
    color: '--series-5',
  };
  const knobHandle: Handle = {
    get: knobPos,
    set: (p) => { phi = Math.atan2(p[1] - target[1], p[0] - target[0]); refresh(); },
    radius: 6,
    enabled: false,
    color: '--series-5',
  };
  cv.addHandle(targetHandle);
  cv.addHandle(knobHandle);

  // ------------------------------------------------------------------ messages
  function setMsg(kind: 'warn' | 'info' | 'bad' | '', text = ''): void {
    msgEl.textContent = text;
    msgEl.style.display = text ? '' : 'none';
    msgEl.style.color = kind === 'warn' ? 'var(--warn)' : kind === 'bad' ? 'var(--danger)' : kind === 'info' ? 'var(--accent)' : '';
  }
  const modeName = (m: FollowMode) => (m === 'nearest' ? '連續追蹤' : `鎖定 elbow-${m}`);

  // ------------------------------------------------------------------ trails (step 5)
  function clearTrails(): void {
    trails.elbow = [[]]; trails.tip = [[]]; trails.js = [[]];
    trailCount = 0;
  }
  function breakTrails(): void {
    trails.elbow.push([]); trails.tip.push([]); trails.js.push([]);
  }
  function pushTrail(): void {
    if (trailCount > 3000) return;
    const jsSeg = trails.js[trails.js.length - 1];
    const q: Pt = [cur.theta1, cur.theta2];
    const last = jsSeg[jsSeg.length - 1];
    if (last && (Math.abs(q[0] - last[0]) > Math.PI || Math.abs(q[1] - last[1]) > Math.PI)) trails.js.push([q]);
    else jsSeg.push(q);
    trails.elbow[trails.elbow.length - 1].push(elbowOf(L1, cur));
    trails.tip[trails.tip.length - 1].push(tipOf(L1, L2, cur));
    trailCount++;
  }

  // ------------------------------------------------------------------ core actions
  function refresh(): void {
    updateReadouts();
    cv.requestRender();
    js.requestRender();
  }
  const redraw = refresh;

  /** Drag / animate the target continuously from its current position (2R or 3R mode). */
  function moveTarget(p: Pt, user: boolean): void {
    if (sub === 'wrist3') {
      target = p;
      refresh();
      return;
    }
    // Before the first layout the canvas scale is meaningless; fall back to a 2 cm snap radius.
    const q = snapToBoundary(L1, L2, p, cv.width > 0 ? SNAP_PX / cv.scale : 0.02);
    const res = followSegment(L1, L2, cur, target, q, mode);
    const recording = step === 4;
    if (res.jump > JUMP_TOL && res.jumpFrom) {
      jumpGhost = res.jumpFrom;
      setMsg('warn', `⚠ ${modeName(mode)}：關節角一步跳了 ${fmt(res.jump / D, 0)}°（灰色虛影是跳之前的姿勢）。路徑經過或極靠近奇異構型，真實手臂無法瞬間換姿勢。`);
      if (recording) breakTrails();
    } else if (user && jumpGhost) {
      jumpGhost = null;
    }
    target = q;
    cur = res.sol;
    ik = res.ik;
    if (ik.reach === 'two') {
      if (mode === 'nearest' && lastBranch && lastBranch !== cur.branch) {
        const where = Math.abs(cur.theta2) > Math.PI / 2 ? '±180°（摺疊）' : '0°（伸直）';
        setMsg('info', `ℹ 連續追蹤：分支標籤由 elbow-${lastBranch} 變成 elbow-${cur.branch}。θ₂ 連續越過 ${where} 這條奇異線，手臂本身沒有跳動。`);
      }
      lastBranch = cur.branch;
    }
    if (recording) pushTrail();
    refresh();
  }

  /** Jump the target to a preset position without continuity bookkeeping. */
  function teleport(p: Pt): void {
    target = p;
    if (sub === 'wrist3') { cur3 = null; refresh(); return; }
    ik = solve2R(L1, L2, p[0], p[1]);
    const keep: FollowMode = mode === 'nearest' ? cur.branch : mode;
    cur = choose(ik, keep, null);
    lastBranch = ik.reach === 'two' ? cur.branch : null;
    jumpGhost = null;
    setMsg('');
    refresh();
  }

  function setRadius(r: number): void {
    const g = Math.hypot(target[0], target[1]) > 1e-6 ? Math.atan2(target[1], target[0]) : 40 * D;
    teleport([r * Math.cos(g), r * Math.sin(g)]);
  }

  function setLengths(a: number, b: number): void {
    L1 = a; L2 = b;
    sL1.set(a, { silent: true });
    sL2.set(b, { silent: true });
    cv.setWorld(worldFor(L1 + L2 + (sub === 'wrist3' ? L3 : 0)));
    ik = solve2R(L1, L2, target[0], target[1]);
    cur = choose(ik, mode, cur);
    lastBranch = ik.reach === 'two' ? cur.branch : null;
    jumpGhost = null;
    refresh();
  }

  function setMode(m: FollowMode): void {
    mode = m;
    selMode.set(m, { silent: true });
    jumpGhost = null;
    setMsg('');
    if (path && step === 4) { resetPath(); runPathInstant(); return; }
    cur = choose(ik, m, cur);
    lastBranch = ik.reach === 'two' ? cur.branch : null;
    refresh();
  }

  function setSub(s: Sub): void {
    sub = s;
    stopPath();
    knobHandle.enabled = s === 'wrist3';
    // In the 3R mode the target is labelled 'p' by drawWrist3 (placed clear of the frame and links).
    targetHandle.label = s === 'wrist3' ? undefined : '目標';
    kvA.el.style.display = s === 'wrist3' ? 'none' : '';
    kvB.el.style.display = s === 'wrist3' ? '' : 'none';
    tTri.el.style.display = tArcs.el.style.display = tAtan.el.style.display = s === 'arm2' ? '' : 'none';
    cv.setWorld(worldFor(L1 + L2 + (s === 'wrist3' ? L3 : 0)));
    jumpGhost = null;
    if (s === 'wrist3') {
      cur3 = null;
      // |w| ≈ 1.46 m: θ₂ ≈ ±87°, so the two decoupled solutions are clearly distinct.
      target = [1.5, 1.2];
      phi = 20 * D;
      setMsg('', '');
    } else {
      if (s === 'newton' && solve2R(L1, L2, target[0], target[1]).solutions.length === 0) target = [1.1, 0.8];
      teleport(target);
      if (s === 'newton') setMsg('info', `Newton 預覽：從固定初值 θ⁰ = (0°, 45°) 出發，每一步用 Jacobian 線性化後修正（下一課詳談）。`);
    }
    refresh();
  }

  // ------------------------------------------------------------------ path demo (step 5)
  const loop = createLoop((dt) => {
    if (!path) { stopPath(); return; }
    pathS = Math.min(1, pathS + dt / PATH_SECONDS);
    const a = path.a, b = path.b;
    moveTarget([a[0] + (b[0] - a[0]) * pathS, a[1] + (b[1] - a[1]) * pathS], false);
    if (pathS >= 1) stopPath();
  });

  function stopPath(): void {
    if (loop.running) loop.stop();
    playBtn.textContent = '▶ 播放路徑';
  }

  function togglePlay(): void {
    if (loop.running) { stopPath(); return; }
    if (sub !== 'arm2') setSub('arm2');
    if (!path) selectPath(0);
    if (pathS >= 1) resetPath();
    playBtn.textContent = '⏸ 暫停';
    loop.start();
  }

  function selectPath(i: number): void {
    path = PATHS[i];
    if (sub !== 'arm2') setSub('arm2');
    if (Math.abs(L1 - path.L[0]) > 1e-9 || Math.abs(L2 - path.L[1]) > 1e-9) setLengths(path.L[0], path.L[1]);
    resetPath();
    runPathInstant();
  }

  /**
   * Follows the whole selected path at once (trails, jump ghost and warning included), so the
   * outcome is visible immediately; ▶ then replays it as an animation from the start.
   */
  function runPathInstant(): void {
    if (!path) return;
    const { a, b } = path;
    const n = 60;
    for (let k = 1; k <= n; k++) {
      const s = k / n;
      moveTarget([a[0] + (b[0] - a[0]) * s, a[1] + (b[1] - a[1]) * s], false);
    }
    pathS = 1;
  }

  /** Link-length slider moved: in the path step, redo the selected path with the new lengths. */
  function lengthsFromSlider(a: number, b: number): void {
    setLengths(a, b);
    if (step === 4 && path) { chipSets[4].clear(); resetPath(); runPathInstant(); }
  }

  function resetPath(): void {
    stopPath();
    if (!path) return;
    pathS = 0;
    clearTrails();
    const a = path.a;
    target = [a[0], a[1]];
    ik = solve2R(L1, L2, a[0], a[1]);
    cur = choose(ik, mode === 'nearest' ? 'down' : mode, null);
    lastBranch = ik.reach === 'two' ? cur.branch : null;
    jumpGhost = null;
    setMsg('');
    pushTrail();
    refresh();
  }

  // ------------------------------------------------------------------ 3R wrist decoupling
  function solve3(): { w: Pt; sols: Solution3R[]; prim: Solution3R | null; ik: IK2R } {
    const res = solve3RDecoupled(L1, L2, L3, target[0], target[1], phi);
    let prim: Solution3R | null = null;
    if (res.sols.length) {
      if (mode === 'nearest' && cur3) {
        const near = pickNearest(res.ik, cur3);
        prim = res.sols.find((s) => s.branch === near?.branch) ?? res.sols[0];
      } else {
        const b: Branch = mode === 'nearest' ? 'down' : mode;
        prim = res.sols.length === 1 ? res.sols[0] : (res.sols.find((s) => s.branch === b) ?? res.sols[0]);
      }
    }
    return { w: res.w, sols: res.sols, prim, ik: res.ik };
  }

  // ------------------------------------------------------------------ readouts
  function updateReadouts(): void {
    if (sub === 'wrist3') {
      const r3 = solve3();
      const deg = (s: Solution3R) => `(${fmt(s.theta1 / D, 0)}°, ${fmt(s.theta2 / D, 0)}°, ${fmt(s.theta3 / D, 0)}°)`;
      kvB.set(KB.p, `(${fmt(target[0], 2)}, ${fmt(target[1], 2)}), ${fmt(phi / D, 0)}°`);
      kvB.set(KB.w, `(${fmt(r3.w[0], 2)}, ${fmt(r3.w[1], 2)})`);
      const { rIn, rOut } = annulus(L1, L2);
      const rw = Math.hypot(r3.w[0], r3.w[1]);
      kvB.set(KB.rw, `${fmt(rw, 2)} m ${r3.sols.length ? '✓' : rw > rOut ? '✗ 太遠' : '✗ 太近'}（環帶 ${fmt(rIn, 1)}–${fmt(rOut, 1)}）`);
      const dn = r3.sols.find((s) => s.branch === 'down');
      const up = r3.sols.find((s) => s.branch === 'up');
      kvB.set(KB.d, dn ? deg(dn) : '—');
      kvB.set(KB.u, up ? deg(up) : r3.sols.length === 1 ? '（與 down 重合）' : '—');
      if (r3.prim) cur3 = r3.prim;
      setMsg(r3.sols.length ? '' : 'bad', r3.sols.length ? '' : '腕心 w 落在環帶外：這個位置配這個朝向到不了。轉動朝向旋鈕，讓 w 回到綠色環帶內。');
      return;
    }
    const { rIn, rOut } = annulus(L1, L2);
    const x = target[0], y = target[1];
    kvA.set(K.r, `${fmt(ik.r, 3)} m`);
    kvA.set(K.ring, `${fmt(rIn, 2)} ≤ r ≤ ${fmt(rOut, 2)}`);
    const reachText: Record<IK2R['reach'], string> = {
      two: '✓ 兩組解',
      one: ik.degenerate ? '原點：θ₁ 任意（∞ 組解）' : '邊界上：一組解',
      outside: '✗ 太遠，無解',
      inside: '✗ 太近，無解',
    };
    kvA.set(K.reach, reachText[ik.reach]);
    kvA.set(K.c2, `${fmt(ik.c2, 3)}${Math.abs(ik.c2) > 1 + 1e-12 ? '（超出 [−1, 1]）' : ''}`);
    const reachable = ik.solutions.length > 0;
    const t2 = Math.acos(clamp(ik.c2, -1, 1));
    // On the boundary sin θ₂ = 0, so ±θ₂ (and ±β) coincide: print a single value there.
    const merged = ik.reach === 'one';
    const pm = merged ? '' : '±';
    kvA.set(K.t2, reachable ? `${pm}${fmt(t2 / D, 1)}°${ik.degenerate ? '（θ₁ 任意）' : merged ? '（兩解合一）' : ''}` : '—（無實數解）');
    kvA.set(K.g, ik.r > 1e-9 ? `${fmt(ik.gamma / D, 1)}°（${quadrantOf(x, y)}）` : '未定義（r = 0）');
    const b = beta(L1, L2, t2);
    kvA.set(K.b, reachable ? `${pm}${fmt(Math.abs(b) / D, 1)}°` : '—');
    const dn = byBranch(ik, 'down');
    const up = byBranch(ik, 'up');
    kvA.set(K.t1d, dn ? (ik.degenerate ? '任意' : `${fmt(dn.theta1 / D, 1)}°`) : '—');
    kvA.set(K.t1u, up ? (ik.degenerate ? '任意' : `${fmt(up.theta1 / D, 1)}°`) : '—');
    if (Math.abs(x) > 1e-9) {
      const at = Math.atan(y / x);
      kvA.set(K.atan, `${fmt(at / D, 1)}°${x < 0 ? '（差 180°！）' : '（= γ）'}`);
    } else kvA.set(K.atan, 'x = 0：y/x 無定義');
    kvA.set(K.shown, !reachable ? '最接近的姿勢'
      : ik.degenerate ? '摺疊（θ₁ 沿用前一個值）'
        : ik.reach === 'one' ? '兩解合一（邊界上）'
          : `elbow-${cur.branch}（${mode === 'nearest' ? '最近解' : '鎖定'}）`);
  }

  // ------------------------------------------------------------------ drawing helpers
  function sideLabel(c: Canvas2D, a: Pt, b: Pt, g: Pt, text: string, color: string): void {
    const m: Pt = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2];
    let nx = m[0] - g[0], ny = m[1] - g[1];
    let len = Math.hypot(nx, ny);
    if (len < 1e-9) { nx = -(b[1] - a[1]); ny = b[0] - a[0]; len = Math.hypot(nx, ny) || 1; }
    const off = 14 / c.scale;
    c.text([m[0] + (nx / len) * off, m[1] + (ny / len) * off], text, { color, bg: cssVar('--sim-bg', '#fff') });
  }

  function drawTriangle(c: Canvas2D, col: Colors): void {
    const O: Pt = [0, 0];
    const E = elbowOf(L1, cur);
    const P = tipOf(L1, L2, cur);
    const G: Pt = [(E[0] + P[0]) / 3, (E[1] + P[1]) / 3];
    c.polygon([O, E, P], { fill: col.tri, color: col.tri, width: 1.5, alpha: 0.16 });
    c.line(O, P, { color: col.tri, width: 1.5, dash: [5, 4] });
    sideLabel(c, O, E, G, 'L₁', col.fg);
    sideLabel(c, E, P, G, 'L₂', col.fg);
    sideLabel(c, O, P, G, `r = ${fmt(ik.r, 2)}`, col.tri);
    const aO = Math.atan2(O[1] - E[1], O[0] - E[0]);
    const d = wrapToPi(Math.atan2(P[1] - E[1], P[0] - E[0]) - aO);
    if (Math.abs(d) > 0.05) {
      const rr = 0.22 * Math.min(L1, L2);
      c.arc(E, rr, Math.min(aO, aO + d), Math.max(aO, aO + d), { color: col.tri, width: 2 });
      const am = aO + d / 2;
      c.text([E[0] + (rr + 0.2) * Math.cos(am), E[1] + (rr + 0.2) * Math.sin(am)], 'π − |θ₂|', { color: col.tri, bg: col.bg });
    }
  }

  function drawArcs(c: Canvas2D, col: Colors): void {
    if (ik.r < 1e-9) return;
    const g = ik.gamma;
    const t1 = cur.theta1;
    const b = beta(L1, L2, cur.theta2);
    const s = Math.min(L1, L2);
    const ref = Math.max(L1, L2) * 0.9;
    c.line([0, 0], [ref, 0], { color: col.muted, width: 1, dash: [3, 3] });
    c.line([0, 0], target, { color: col.cyan, width: 1.2, dash: [5, 4] });
    const arcL = (r: number, a0: number, a1: number, color: string, label: string) => {
      if (Math.abs(a1 - a0) < 1e-3) return;
      c.arc([0, 0], r, Math.min(a0, a1), Math.max(a0, a1), { color, width: 2.2 });
      const am = (a0 + a1) / 2;
      c.text([(r + 0.13) * Math.cos(am), (r + 0.13) * Math.sin(am)], label, { color, bg: col.bg });
    };
    arcL(0.3 * s, 0, t1, col.fg, 'θ₁');
    arcL(0.52 * s, 0, g, col.cyan, 'γ');
    arcL(0.74 * s, t1, t1 + b, col.warn, 'β');
    // θ₂ at the elbow, measured from the extension of link 1
    const E = elbowOf(L1, cur);
    const ext = 0.3 * L2;
    c.line(E, [E[0] + ext * Math.cos(t1), E[1] + ext * Math.sin(t1)], { color: col.muted, width: 1, dash: [3, 3] });
    if (Math.abs(cur.theta2) > 1e-3) {
      const rr = 0.22 * L2;
      c.arc(E, rr, Math.min(t1, t1 + cur.theta2), Math.max(t1, t1 + cur.theta2), { color: col.muted, width: 2 });
      const am = t1 + cur.theta2 / 2;
      c.text([E[0] + (rr + 0.13) * Math.cos(am), E[1] + (rr + 0.13) * Math.sin(am)], 'θ₂', { color: col.fg, bg: col.bg });
    }
  }

  function drawAtan(c: Canvas2D, col: Colors): void {
    if (Math.abs(target[0]) < 1e-9 || ik.degenerate) return;
    const wrong = atanTheta1(target[0], target[1], beta(L1, L2, cur.theta2));
    if (Math.abs(wrapToPi(wrong - cur.theta1)) < 1e-6) {
      c.text([0, -0.5 * Math.max(L1, L2)], 'x > 0：atan 與 atan2 給相同的 θ₁', { color: col.muted, bg: col.bg });
      return;
    }
    const pts = drawArm(c, [L1, L2], [wrong, cur.theta2], col.danger, 'dashed');
    const tp = pts[2];
    const k = 0.07;
    c.line([tp[0] - k, tp[1] - k], [tp[0] + k, tp[1] + k], { color: col.danger, width: 2.5 });
    c.line([tp[0] - k, tp[1] + k], [tp[0] + k, tp[1] - k], { color: col.danger, width: 2.5 });
    c.text(tp, 'atan 的手臂 → (−x, −y)', { color: col.danger, dy: 16, bg: col.bg });
  }

  /** Label the elbow on the outer side of the base–tip chord so it never sits on a link. */
  function elbowLabel(c: Canvas2D, s: Solution, color: string, text = `elbow-${s.branch}`): void {
    const E = elbowOf(L1, s);
    const P = tipOf(L1, L2, s);
    const lp = Math.hypot(P[0], P[1]);
    const u: Pt = lp > 1e-9 ? [P[0] / lp, P[1] / lp] : [1, 0];
    const k = E[0] * u[0] + E[1] * u[1];
    let nx = E[0] - k * u[0], ny = E[1] - k * u[1];
    let ln = Math.hypot(nx, ny);
    if (ln < 1e-9) { nx = -u[1]; ny = u[0]; ln = 1; }
    // Offset the label centre by the pill's half-extent along the normal (+ 8 px), so the whole pill
    // stays clear of the elbow and of the links through it, even for a stretched arm.
    const font = `12px ${cssVar('--font') || 'sans-serif'}`;
    c.ctx.font = font;
    const w = c.ctx.measureText(text).width + 8;
    const ux = nx / ln, uy = ny / ln;
    const off = (8 + (w / 2) * Math.abs(ux) + 8 * Math.abs(uy)) / c.scale;
    c.text([E[0] + ux * off, E[1] + uy * off], text, { color, bg: cssVar('--sim-bg', '#fff'), font });
  }

  const branchColor = (b: Branch, col: Colors) => (b === 'down' ? col.down : col.up);

  function draw2R(c: Canvas2D, col: Colors): void {
    if (step === 4) {
      if (path) c.line(path.a, path.b, { color: col.muted, width: 1.2, dash: [3, 4] });
      for (const seg of trails.tip) c.polyline(seg, { color: col.muted, width: 1.5, alpha: 0.7 });
      for (const seg of trails.elbow) c.polyline(seg, { color: col.trail, width: 2.2, alpha: 0.85 });
    }
    const reachable = ik.solutions.length > 0;
    if (show.other && ik.reach === 'two') {
      const o = ik.solutions.find((s) => s.branch !== cur.branch);
      if (o) { drawArm(c, [L1, L2], [o.theta1, o.theta2], branchColor(o.branch, col), 'dashed'); elbowLabel(c, o, branchColor(o.branch, col)); }
    }
    if (jumpGhost) {
      drawArm(c, [L1, L2], [jumpGhost.theta1, jumpGhost.theta2], col.muted, 'ghost');
      c.text(elbowOf(L1, jumpGhost), '跳之前', { color: col.muted, dy: -14, bg: col.bg });
    }
    if (show.tri && reachable) drawTriangle(c, col);
    if (show.atan && reachable) drawAtan(c, col);
    const pts = drawArm(c, [L1, L2], [cur.theta1, cur.theta2], reachable ? branchColor(cur.branch, col) : col.danger, reachable ? 'solid' : 'dashed');
    // On the boundary both branches coincide (sin θ₂ = 0), so the single arm is not labelled with a branch.
    if (reachable && !ik.degenerate && !show.arcs) elbowLabel(c, cur, branchColor(cur.branch, col), ik.reach === 'one' ? '兩解合一' : undefined);
    if (show.arcs && reachable) drawArcs(c, col);
    if (!reachable) {
      c.text(pts[2], ik.reach === 'outside' ? '伸直也搆不到' : '摺起來還是太遠', { color: col.danger, dy: 18, bg: col.bg });
    } else if (ik.degenerate) {
      c.circle([0, 0], L1, { color: col.down, width: 1, dash: [2, 4] });
      c.text([0, -L1], '手肘可在這圈上任意位置（θ₁ 任意）', { color: col.down, dy: 14, bg: col.bg });
    }
  }

  function drawWrist3(c: Canvas2D, col: Colors): void {
    const r3 = solve3();
    const Ls = [L1, L2, L3];
    if (r3.prim) {
      if (show.other && r3.sols.length === 2) {
        const o = r3.sols.find((s) => s !== r3.prim)!;
        drawArm(c, Ls, [o.theta1, o.theta2, o.theta3], branchColor(o.branch, col), 'dashed');
      }
      const p = r3.prim;
      drawArm(c, Ls, [p.theta1, p.theta2, p.theta3], branchColor(p.branch, col), 'solid');
      elbowLabel(c, p, branchColor(p.branch, col));
    } else {
      const fb = r3.ik.fallback!;
      drawArm(c, [L1, L2], [fb.theta1, fb.theta2], col.danger, 'dashed');
    }
    c.line(r3.w, target, { color: col.target, width: 2, dash: [5, 4] });
    c.point(r3.w, 6, { color: r3.prim ? col.ann : col.danger });
    c.frame({ x: target[0], y: target[1], theta: phi }, 0.3, { width: 2 });
    c.line(target, knobPos(), { color: col.target, width: 1.5, dash: [4, 3] });
    // Labels on the free side of each point so none sits on a link: w on the side of the last link
    // away from the elbow, p on the −y side of the target frame, φ beyond the knob (screen px).
    const n: Pt = [-Math.sin(phi), Math.cos(phi)];
    const E = elbowOf(L1, r3.prim ?? r3.ik.fallback!);
    const away = (E[0] - r3.w[0]) * n[0] + (E[1] - r3.w[1]) * n[1] > 0 ? -1 : 1;
    const kw = 14 + 30 * Math.abs(n[0]);
    c.text(r3.w, 'w（腕心）', { color: r3.prim ? col.fg : col.danger, dx: away * n[0] * kw, dy: -away * n[1] * kw, bg: col.bg });
    const kp = 15 + 6 * Math.abs(n[0]);
    c.text(target, 'p', { color: col.target, dx: -n[0] * kp, dy: n[1] * kp, bg: col.bg });
    c.text(knobPos(), 'φ', { color: col.target, dx: 15 * Math.cos(phi), dy: -15 * Math.sin(phi) });
  }

  function drawNewton(c: Canvas2D, col: Colors): void {
    const res = newton2R(L1, L2, NEWTON_GUESS, target, 8);
    const n = res.thetas.length;
    const tips = res.thetas.map((t) => tipOf(L1, L2, { theta1: t[0], theta2: t[1] }));
    res.thetas.forEach((t, k) => {
      const last = k === n - 1;
      drawArm(c, [L1, L2], [t[0], t[1]], col.newton, last && res.converged ? 'solid' : 'ghost', last ? 1 : 0.5 + 0.5 * (k / Math.max(1, n - 1)));
    });
    for (let k = 0; k + 1 < n; k++) {
      if (Math.hypot(tips[k + 1][0] - tips[k][0], tips[k + 1][1] - tips[k][1]) > 0.04) c.arrow(tips[k], tips[k + 1], { color: col.newton, width: 1.8, head: 9 });
    }
    for (let k = 0; k < Math.min(n, 4); k++) c.text(tips[k], String(k), { color: col.newton, dx: 10, dy: -10, bg: col.bg });
  }

  // ------------------------------------------------------------------ canvases
  cv.draw = (c) => {
    const col = readColors();
    const { rIn, rOut } = annulus(L1, L2);
    c.grid(0.5);
    fillAnnulus(c, rIn, rOut, col.ann, 0.1);
    c.circle([0, 0], rOut, { color: col.ann, width: 1.3, dash: [6, 4] });
    if (rIn > 1e-6) c.circle([0, 0], rIn, { color: col.ann, width: 1.3, dash: [6, 4] });
    drawBase(c);
    if (sub === 'wrist3') drawWrist3(c, col);
    else if (sub === 'newton') drawNewton(c, col);
    else draw2R(c, col);
    const ok = sub === 'wrist3' ? solve3().prim != null : ik.solutions.length > 0;
    // Theme variable names (resolved when the handles are drawn), so a theme switch keeps them right.
    targetHandle.color = ok ? '--series-5' : '--danger';
    if (sub === 'newton') {
      const res = newton2R(L1, L2, NEWTON_GUESS, target, 8);
      c.setOverlayText(`Newton：${res.converged ? `${res.thetas.length - 1} 步收斂` : '8 步內未收斂'}　|e| = ${res.errors.map((e) => e.toExponential(0)).slice(0, 6).join(' → ')}`);
    } else if (sub === 'wrist3') {
      c.setOverlayText(`p = (${fmt(target[0], 2)}, ${fmt(target[1], 2)})  φ = ${fmt(phi / D, 0)}°  L₃ = ${L3}`);
    } else {
      c.setOverlayText(`p = (${fmt(target[0], 2)}, ${fmt(target[1], 2)})  r = ${fmt(ik.r, 2)} m`);
    }
  };

  js.draw = (c) => {
    const col = readColors();
    const P = Math.PI;
    c.polygon([[-P, 0], [P, 0], [P, P], [-P, P]], { fill: col.down, width: 0, alpha: 0.09 });
    c.polygon([[-P, -P], [P, -P], [P, 0], [-P, 0]], { fill: col.up, width: 0, alpha: 0.09 });
    const gridC = cssVar('--grid-strong', '#ccc');
    for (const v of [-90, 90]) {
      c.line([v * D, -P], [v * D, P], { color: gridC, width: 1, dash: [2, 3] });
      c.line([-P, v * D], [P, v * D], { color: gridC, width: 1, dash: [2, 3] });
    }
    c.polygon([[-P, -P], [P, -P], [P, P], [-P, P]], { color: gridC, width: 1 });
    for (const yv of [-P, 0, P]) c.line([-P, yv], [P, yv], { color: col.danger, width: 1.4, dash: [5, 3] });
    const font = `10px ${cssVar('--mono') || 'monospace'}`;
    for (const v of [-180, -90, 0, 90, 180]) {
      c.text([v * D, -P], String(v), { color: col.muted, dy: 9, font });
      c.text([-P, v * D], String(v), { color: col.muted, dx: -15, font });
    }
    c.text([0, -P], 'θ₁ (°)', { color: col.muted, dy: 22 });
    c.text([-P, 0.75 * P], 'θ₂', { color: col.muted, dx: -15 });
    const small = '10px sans-serif';
    c.text([P - 0.1, 0.8 * P], 'elbow-down', { color: col.down, align: 'right', font: small });
    c.text([P - 0.1, -0.8 * P], 'elbow-up', { color: col.up, align: 'right', font: small });
    c.text([-P + 0.1, 0], '伸直', { color: col.danger, align: 'left', dy: -8, font: small });
    c.text([-P + 0.1, P], '摺疊', { color: col.danger, align: 'left', dy: 8, font: small });

    if (sub === 'newton') {
      const res = newton2R(L1, L2, NEWTON_GUESS, target, 8);
      const pts = res.thetas.map((t) => [t[0], t[1]] as Pt);
      for (let k = 0; k + 1 < pts.length; k++) {
        if (Math.abs(pts[k + 1][0] - pts[k][0]) < P && Math.abs(pts[k + 1][1] - pts[k][1]) < P) c.line(pts[k], pts[k + 1], { color: col.newton, width: 1.5 });
      }
      pts.forEach((p, k) => c.point(p, k === pts.length - 1 ? 4.5 : 3, { color: col.newton }));
    }
    if (step === 4 && sub === 'arm2') for (const seg of trails.js) c.polyline(seg, { color: col.trail, width: 2.2 });

    let sols: Solution[];
    let prim: Solution | null;
    if (sub === 'wrist3') { const r3 = solve3(); sols = r3.sols; prim = r3.prim; } else { sols = ik.solutions; prim = sols.length ? cur : null; }
    if (sub !== 'wrist3' && ik.degenerate) {
      c.line([-P, P], [P, P], { color: col.down, width: 4, alpha: 0.6 });
      c.text([0, P], 'θ₁ 任意', { color: col.down, dy: -9, font: '11px sans-serif' });
    }
    if (sub !== 'newton') {
      for (const s of sols) c.point([s.theta1, s.theta2], 5.5, { color: branchColor(s.branch, col) });
      if (prim) c.circle([prim.theta1, prim.theta2], 9 / c.scale, { color: col.fg, width: 2 });
      if (!sols.length && sub === 'arm2') c.circle([cur.theta1, cur.theta2], 6 / c.scale, { color: col.danger, width: 2 });
    }
  };

  // ------------------------------------------------------------------ per-step presets
  const chipSets: PresetChipsHandle[] = [
    presetChips(chipRow, [
      { label: '右上 (1.2, 0.9)', apply: () => teleport([1.2, 0.9]) },
      { label: '左上 (−0.8, 1.3)', apply: () => teleport([-0.8, 1.3]) },
      { label: '下方 (0.4, −1.4)', apply: () => teleport([0.4, -1.4]) },
    ], { label: '目標：' }),
    presetChips(chipRow, [
      { label: 'c₂ = 1（伸直）', apply: () => setRadius(L1 + L2) },
      { label: 'c₂ = 0（θ₂ = ±90°）', apply: () => setRadius(Math.hypot(L1, L2)) },
      { label: 'c₂ = −1（摺疊）', apply: () => setRadius(Math.abs(L1 - L2)) },
    ], { label: '距離：' }),
    presetChips(chipRow, [
      { label: '右半平面 (1.1, 0.6)', apply: () => teleport([1.1, 0.6]) },
      { label: '左半平面 (−1.1, 0.7)', apply: () => teleport([-1.1, 0.7]) },
    ], { label: '目標：' }),
    presetChips(chipRow, [
      { label: '外邊界', apply: () => setRadius(L1 + L2) },
      { label: '內邊界（L₂ = 0.6）', apply: () => { if (Math.abs(L1 - L2) < 0.2) setLengths(1, 0.6); setRadius(Math.abs(L1 - L2)); } },
      { label: '太遠', apply: () => setRadius(L1 + L2 + 0.25) },
      { label: '內洞', apply: () => { if (Math.abs(L1 - L2) < 0.2) setLengths(1, 0.6); setRadius(Math.abs(L1 - L2) * 0.4); } },
      { label: 'L₁ = L₂ = 1', apply: () => setLengths(1, 1) },
    ], { label: '試試：' }),
    presetChips(chipRow, PATHS.map((p, i) => ({ label: p.label, title: p.title, apply: () => selectPath(i) })), { label: '路徑：' }),
    presetChips(chipRow, [
      { label: '3R 腕心分解', apply: () => setSub('wrist3') },
      { label: '數值迭代預覽', apply: () => setSub('newton') },
      { label: '回到 2R 兩解', apply: () => setSub('arm2') },
    ], { label: '模式：' }),
  ];

  // ------------------------------------------------------------------ per-step focus
  let focused: HTMLElement[] = [];
  function setFocus(els: (HTMLElement | null)[]): void {
    focused.forEach((f) => f.classList.remove('mr-focus'));
    focused = els.filter((x): x is HTMLElement => !!x);
    focused.forEach((f) => f.classList.add('mr-focus'));
  }

  function onStep(i: number): void {
    const first = step === -1;
    if (i === step) return;
    step = i;
    stopPath();
    chipSets.forEach((cs, k) => { cs.el.style.display = k === i ? '' : 'none'; cs.clear(); });
    pathRow.style.display = i === 4 ? '' : 'none';
    card.style.display = i === 5 ? '' : 'none';
    if (i !== 5 && sub !== 'arm2') setSub('arm2');
    if (i !== 4) { clearTrails(); path = null; }
    const want = { other: true, tri: i === 1, arcs: i === 2, atan: i === 2 };
    show.other = want.other; tOther.set(want.other, { silent: true });
    show.tri = want.tri; tTri.set(want.tri, { silent: true });
    show.arcs = want.arcs; tArcs.set(want.arcs, { silent: true });
    show.atan = want.atan; tAtan.set(want.atan, { silent: true });
    jumpGhost = null;
    setMsg('');
    if (i === 2) chipSets[2].activate(1);
    else if (i === 3) chipSets[3].activate(0);
    else if (i === 4) { if (mode !== 'down') setMode('down'); chipSets[4].activate(0); }
    else if (i === 5) chipSets[5].activate(0);
    else if (i === 0 && !first) { if (Math.abs(L1 - 1) > 1e-9 || Math.abs(L2 - 1) > 1e-9) setLengths(1, 1); }
    const focusMap: (HTMLElement | null)[][] = [
      [cv.el, js.el],
      [kvA.el, selMode.el],
      [cv.el, tAtan.el],
      [sL1.el, sL2.el],
      [pathRow, selMode.el],
      [card, cv.el],
    ];
    setFocus(focusMap[i] ?? []);
    refresh();
  }

  onStep(ctx.currentStep);

  return {
    onStep,
    destroy() {
      loop.stop();
      ro.disconnect();
      cancelAnimationFrame(kvRaf);
      cv.destroy();
      js.destroy();
    },
  };
}

const lesson: Lesson = { meta, steps, quiz, mount };
export default lesson;
