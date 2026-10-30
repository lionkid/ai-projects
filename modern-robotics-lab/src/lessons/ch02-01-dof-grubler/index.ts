import type { Lesson, LessonContext, LessonHandle, LessonStep, QuizQuestion } from '@/types';
import {
  el, cssVar, fmt, button, select, kv, note, simTitle, angleSlider, presetChips, mdElement, renderMath, split,
  Canvas2D, type Handle, type PresetChipsHandle,
} from '@/ui';
import { meta } from './meta';
import {
  ARM2R, DEFAULT_WORLD, MAX_BODIES, PRESETS, addLink, addPJoint, addPinAt, addRJoint, analyze, arm2RAngles,
  cloneMechanism, dragPin, fixedBody, groupPos, grubler, grublerDof, pinMotion, pointGroups, presetByKey, project,
  removeBody, removeJoint, setArm2R, setGround, shapePins, spatialMobility, worldDir, worldPt,
  type Analysis, type Endpoint, type Mechanism, type PinMotion, type PointGroup, type PointRef, type PresetKey, type Pt,
  type World2,
} from './linkage';

// ---------------------------------------------------------------------------
// Narrative
// ---------------------------------------------------------------------------

const steps: LessonStep[] = [
  {
    title: '一個剛體有幾個自由度？',
    md: `**自由度（degrees of freedom, dof）**：要把一個物體的位置與姿態完全確定下來，最少需要幾個獨立的實數。

數法很直接——先數「點的座標」，再扣掉「讓點與點距離不變」的約束。平面上的一根桿取兩點 A、B，共 4 個座標，但 $|AB|$ 必須固定：

$$\\text{dof} = \\underbrace{2+2}_{A,\\;B} - \\underbrace{1}_{|AB|} = 3$$

這 3 個數也可以換成更順手的 $(x, y, \\phi)$：參考點的位置加上轉角。在 3D 空間要取三個不共線的點：$9-3=6$，也就是 3 個位置加 3 個姿態。`,
    simHint: '拖曳桿子兩端的 A、B：四個座標都在變，但 |AB| 始終不變；右側讀數顯示只要 (x, y, φ) 三個數就能描述整根桿。',
  },
  {
    title: '關節是「拿走」自由度',
    md: `兩根互不相連的桿共有 $3+3=6$ 個自由度。用關節把它們接起來，並**不會**增加運動，而是**限制**兩者之間的相對運動：

- 旋轉關節 R 只允許相對轉動；
- 滑動關節 P 只允許沿一條直線相對滑動。

把關節允許的相對自由度記為 $f_i$，它拿走的自由度（約束數）就是

$$c_i = m - f_i ,$$

其中 $m$ 是單一剛體的自由度：平面 $m=3$、空間 $m=6$。平面中 R、P 的 $f=1$，所以各拿走 2 個。右側表格整理了常見關節。`,
    simHint: '用上方的「無關節／R 關節／P 關節」切換兩根桿的接法：dof 從 6 掉到 4，右側表格會標出用到的關節。',
  },
  {
    title: 'Grübler 公式',
    md: `把整個機構一起算：共 $N$ 根連桿（**地面也算一根**）、$J$ 個關節。先讓 $N-1$ 根會動的桿各自自由，得到 $m(N-1)$；每個關節再拿走 $c_i$。只要這些約束**彼此獨立**：

$$\\text{dof} = m(N-1) - \\sum_{i=1}^{J} c_i = m(N-1-J) + \\sum_{i=1}^{J} f_i$$

兩個常見陷阱：忘了把地面算進 $N$；同一根銷子串起 $k$ 根桿時，要算成 $k-1$ 個關節。四連桿：$N=4$、$J=4$，$\\text{dof}=3(4-1-4)+4=1$。`,
    simHint: '把滑鼠移到（或點一下）計數區的 N、J、Σf：畫面會替被算到的連桿或關節編號。注意地面也拿到 1 號。',
  },
  {
    title: '動手驗證：四連桿、曲柄滑塊、五連桿',
    md: `dof 的物理意義：要讓機構的姿態完全確定，需要幾個**獨立的輸入**（例如幾顆馬達）。

這一步請先預測、再驗證：

1. 選一個機構，從圖上數出 $N$、$J$、$\\sum f_i$（計數區會先遮住）；
2. 在右側點選你預測的 dof，答案才會揭曉；
3. 拖曳手把確認：dof = 1 時，每個點都只能沿一條曲線走（手把旁是雙箭頭）；五連桿的頂點卻能往平面上任何方向移動（十字箭頭）。

提醒：曲柄滑塊的滑塊與地面之間是 P 關節，別漏算。`,
    simHint: '依序選「四連桿」「曲柄滑塊」「五連桿」「六連桿」：先點預測的 dof 才會揭曉答案，再拖手把確認。',
  },
  {
    title: '公式失靈的時候',
    md: `「平行四邊形疊加」有 $N=5$、$J=6$：

$$\\text{dof} = 3(5-1-6)+6 = 0$$

公式判定它是結構，但三根桿等長又互相平行，它其實能動！Grübler 公式的前提是**所有約束彼此獨立**；在這個特殊幾何下，第三根桿的約束早已被另外兩根保證，成了**多餘約束（redundant constraint）**。

模擬器另外用數值方法數出**真正獨立**的約束數（約束 Jacobian 的秩）。所以 Grübler 只是**下限**：實際 dof ≥ 公式值；尺寸「一般」（沒有刻意的等長、平行）時兩者就相等。`,
    simHint: '選「平行四邊形疊加」：公式說 0，手把卻拖得動。再選「中間桿歪一點」——N、J 完全相同，這次真的鎖死了。',
  },
  {
    title: '從自由度到組態空間',
    md: `機構上每一點位置的完整描述叫**組態（configuration）**，所有可能組態的集合叫**組態空間（C-space）**；dof 就是這個空間的**維度**。

2R 平面手臂：$N=3$、$J=2$，$\\text{dof}=3(3-1-2)+2=2$，兩個關節角 $(\\theta_1,\\theta_2)$ 就能描述它。對開鏈來說，dof 正好是各關節 $f_i$ 的總和。

不過「2 維」只說了大小，沒說形狀：$\\theta_1$ 轉一整圈會回到原姿態，所以方格的左右兩邊其實是同一條邊，上下也是。這個空間到底長什麼樣？下一課揭曉。先完成下方的自我檢測吧！`,
    simHint: '拖曳手臂末端或用滑桿改 θ₁、θ₂：小方格裡的點就是目前的組態。拖著手臂繞過正左方（θ₁ = 180°），手臂連續轉動，點卻從右緣跳到左緣。',
  },
];

const quiz: QuizQuestion[] = [
  {
    question: '小華分析一個平面機構：除了固定的地面，還有 5 根會動的連桿，以 7 個 R 關節相連（沒有共點的關節）。他算出 $3(5-1-7)+7=-2$，判定為「拖不動的超靜定結構」。正確的判斷是？',
    options: [
      '他算對了：dof = −2，是超靜定結構',
      '$N$ 必須包含地面：$N=6$，dof $=3(6-1-7)+7=1$，是單自由度機構',
      '每個關節連接兩根桿，$J$ 應該算成 14',
      '連桿多於 4 根時要改用 $m=6$',
    ],
    answer: 1,
    explanation: 'Grübler 公式裡的 $N$ 包含地面，「$-1$」才是把地面固定所扣掉的部分。代入 $N=6$ 得 $3(-2)+7=1$。一個 R 關節只算一次（$k$ 根桿共點時算 $k-1$ 個）；平面機構永遠用 $m=3$。本課的「六連桿」預設正是 $N=6$、$J=7$，可以拖拖看它是否只有一個自由度。',
  },
  {
    question: '模擬中的五連桿 dof = 2。若用「+ 連桿」在兩個原本不相連的接點之間，再加一根兩端都是 R 關節的桿（尺寸一般、沒有特殊的平行關係），dof 會變成？',
    options: ['3：多一根桿就多一些自由度', '2：不受影響', '1', '0'],
    answer: 2,
    explanation: '新桿帶進 $+3$，但兩個 R 關節各拿走 2，共 $-4$，淨變化 $-1$：$3(6-1-7)+7=1$。在閉迴路上加桿、加關節通常讓機構「更緊」而不是更自由——關節是約束，不是馬達。可以在模擬中用「+ 連桿」親手驗證。',
  },
  {
    question: '「平行四邊形疊加」的 Grübler 值是 0，卻能動。若只把中間那根桿的地面支點往右挪一點（不再與另外兩根平行），會發生什麼事？',
    options: [
      'Grübler 值沒變，所以仍然能動',
      '機構卡死：Grübler 仍是 0，而且這次約束全部獨立，實際 dof 也是 0',
      'dof 變成 2，因為幾何變得更一般',
      'Grübler 值變成 −1',
    ],
    answer: 1,
    explanation: 'Grübler 只看 $N$、$J$、$f_i$，挪動支點不改變這些數，所以仍是 0。原本能動是因為特殊幾何（等長且平行）讓一條約束變得多餘；破壞平行之後約束全部獨立，實際 dof 就等於公式給的 0。模擬中的「中間桿歪一點」就是這個情況。',
  },
  {
    question: '空間開鏈：地面 –R– 桿1 –R– 桿2 –R– 桿3 –S– 桿4。它的 dof 是多少？若拿掉桿4，讓 S 關節直接接回地面形成閉迴路，dof 又是多少？（$m=6$）',
    options: ['6；0', '6；6', '4；1', '6；−6'],
    answer: 0,
    explanation: '開鏈：$N=5$、$J=4$、$\\sum f_i=1+1+1+3=6$，$6(5-1-4)+6=6$——開鏈的 dof 就是各關節 $f_i$ 的總和。閉迴路：$N=4$、$J=4$，$6(4-1-4)+6=0$，尺寸一般時是結構。尺寸一般的單一空間閉迴路要 $\\sum f_i \\ge 7$ 才能動（例如 RSSR 的 $\\sum f_i=8$，得 2，其中 1 個是連接桿繞自身軸線的空轉）；平面四連桿只有 $\\sum f_i=4$ 卻能動，正是因為四個 R 軸互相平行的特殊幾何。',
  },
];

// ---------------------------------------------------------------------------
// Static tables and per-step configuration
// ---------------------------------------------------------------------------

const JOINT_TABLE: { key: string; zh: string; en: string; f: number; c3: number | null; c6: number }[] = [
  { key: 'R', zh: '旋轉', en: 'revolute', f: 1, c3: 2, c6: 5 },
  { key: 'P', zh: '滑動', en: 'prismatic', f: 1, c3: 2, c6: 5 },
  { key: 'H', zh: '螺旋', en: 'helical', f: 1, c3: null, c6: 5 },
  { key: 'C', zh: '圓柱', en: 'cylindrical', f: 2, c3: null, c6: 4 },
  { key: 'U', zh: '萬向', en: 'universal', f: 2, c3: null, c6: 4 },
  { key: 'S', zh: '球', en: 'spherical', f: 3, c3: null, c6: 3 },
];

const SPATIAL_MD = `| 空間機構 | N | J | Σf | dof |
|---|---|---|---|---|
| 6R 開鏈手臂（如 UR5） | 7 | 6 | 6 | ${grublerDof(6, 7, 6, 6)} |
| RSSR 空間四連桿 | 4 | 4 | 8 | ${grublerDof(6, 4, 4, 8)}* |
| Stewart 平台（6 支 UPS 腿） | 14 | 18 | 36 | ${grublerDof(6, 14, 18, 36)} |

\\* 其中 1 個是連接桿繞自身軸線的空轉。`;

type Mode = 'drag' | 'link' | 'pivot' | 'R' | 'P' | 'delete' | 'ground';
type Highlight = 'N' | 'J' | 'f' | null;
type PanelKey = 'body' | 'joints' | 'predict' | 'redundant' | 'cspace';

const TOOLS: { mode: Mode; label: string; title: string }[] = [
  { mode: 'drag', label: '拖曳', title: '拖曳藍色手把' },
  { mode: 'link', label: '+ 連桿', title: '依序點兩個端點：接點、桿身或空白處（空白處 = 自由端）' },
  { mode: 'pivot', label: '+ 支點', title: '在畫面上點一下，新增一個固定在地面的接點' },
  { mode: 'R', label: '+ R', title: '點兩根不同桿上的點，用旋轉關節把它們接起來' },
  { mode: 'P', label: '+ P', title: '先點要滑動的接點，再點當滑軌的桿（或地面）' },
  { mode: 'delete', label: '刪除', title: '點關節或連桿刪除（地面不能刪）' },
  { mode: 'ground', label: '設為地面', title: '點一根桿，讓它變成固定的地面（運動學反轉）' },
];

const STEP_CHIPS: { key: PresetKey; label: string }[][] = [
  [{ key: 'free', label: '單一自由桿' }, { key: 'twoFree', label: '兩根自由桿' }],
  [{ key: 'twoFree', label: '無關節' }, { key: 'twoR', label: 'R 關節' }, { key: 'twoP', label: 'P 關節' }],
  [{ key: 'fourbar', label: '四連桿' }, { key: 'truss', label: '三角桁架' }, { key: 'arm2R', label: '2R 手臂（開鏈）' }],
  [{ key: 'fourbar', label: '四連桿' }, { key: 'slider', label: '曲柄滑塊' }, { key: 'fivebar', label: '五連桿' }, { key: 'sixbar', label: '六連桿' }],
  [{ key: 'parallelogram', label: '平行四邊形' }, { key: 'stacked', label: '平行四邊形疊加' }, { key: 'stackedSkew', label: '中間桿歪一點' }, { key: 'truss', label: '三角桁架' }],
  [{ key: 'arm2R', label: '2R 手臂' }],
];
const STEP_CHIP_LABEL = ['試試：', '兩根桿之間：', '套用公式：', '先預測再驗證：', '特殊幾何：', '開鏈：'];
const STEP_DEFAULT: PresetKey[] = ['free', 'twoFree', 'fourbar', 'fourbar', 'stacked', 'arm2R'];
const STEP_PANEL: PanelKey[] = ['body', 'joints', 'joints', 'predict', 'redundant', 'cspace'];

const CSS = `
.dg-root .dg-chiprow { min-height: 2rem; }
.dg-root .dg-tools { display: flex; flex-wrap: wrap; gap: 0.35rem 0.6rem; align-items: center; margin: 0.45rem 0 0.1rem; }
.dg-root .dg-tools .mr-chips { margin: 0; }
.dg-root .dg-tools .mr-control { flex-direction: row; align-items: center; gap: 0.4rem; }
.dg-root .mr-split { margin-top: 0.5rem; align-items: start; }
.dg-root .dg-card { border: 1px solid var(--border); border-radius: var(--radius-sm); background: var(--bg-elev); padding: 0.5rem 0.7rem 0.6rem; font-size: 0.85rem; min-width: 0; }
.dg-root .dg-head { display: flex; flex-wrap: wrap; align-items: center; justify-content: space-between; gap: 0.25rem 0.6rem; margin-bottom: 0.3rem; }
.dg-root .dg-title { font-weight: 600; font-size: 0.86rem; }
.dg-root .dg-head .mr-chips { margin: 0; }
.dg-root .dg-counts { display: flex; flex-wrap: wrap; gap: 0.35rem; }
.dg-root .dg-counts .mr-chip { font-family: var(--mono); font-size: 0.8rem; }
.dg-root .dg-counts .mr-chip:disabled { opacity: 0.75; cursor: default; }
.dg-root .dg-formula { overflow-x: auto; overflow-y: hidden; }
.dg-root .dg-formula .katex-display { margin: 0.35rem 0; }
.dg-root .dg-check { color: var(--fg-muted); font-size: 0.78rem; line-height: 1.45; }
.dg-root .dg-status { margin-top: 0.4rem; padding: 0.35rem 0.55rem; border-radius: var(--radius-sm); font-size: 0.82rem; line-height: 1.45; background: var(--bg-sunken); color: var(--fg-muted); }
.dg-root .dg-status.ok { background: var(--ok-soft); color: var(--ok); }
.dg-root .dg-status.warn { background: var(--warn-soft); color: var(--warn); }
.dg-root .dg-status.lock { background: var(--danger-soft); color: var(--danger); }
.dg-root .dg-spatial { margin-top: 0.45rem; font-size: 0.8rem; }
.dg-root .dg-spatial .md { font-size: 0.8rem; }
.dg-root .dg-spatial table { margin: 0.2rem 0; }
.dg-root .dg-table { border-collapse: collapse; width: 100%; font-size: 0.8rem; }
.dg-root .dg-table th, .dg-root .dg-table td { border: 1px solid var(--border); padding: 0.18rem 0.4rem; text-align: center; }
.dg-root .dg-table th { background: var(--bg-sunken); font-weight: 600; }
.dg-root .dg-table th.on { color: var(--accent); }
.dg-root .dg-table td:first-child { text-align: left; white-space: nowrap; }
.dg-root .dg-table .dg-en { font-size: 0.72rem; color: var(--fg-muted); }
.dg-root .dg-table tr.on .dg-en { color: inherit; }
.dg-root .dg-table .dg-sub { font-size: 0.68rem; font-weight: 400; white-space: nowrap; }
.dg-root .dg-table tr.on td { background: var(--accent-soft); color: var(--accent); font-weight: 600; }
.dg-root .dg-small { font-size: 0.8rem; color: var(--fg-muted); margin: 0.35rem 0; line-height: 1.45; }
.dg-root .dg-result { font-size: 0.84rem; margin: 0.4rem 0; line-height: 1.45; }
.dg-root .dg-result.ok { color: var(--ok); }
.dg-root .dg-result.bad { color: var(--danger); }
.dg-root .dg-cs { max-width: 250px; margin: 0.15rem auto 0; }
.dg-root .dg-card .mr-controls { padding-top: 0.3rem; }
.dg-root .dg-hidden { display: none !important; }
.dg-root .mr-canvas-wrap .mr-overlay { max-width: calc(100% - 1rem); white-space: normal; line-height: 1.35; }
`;

// ---------------------------------------------------------------------------
// Small geometry helpers for drawing / hit testing
// ---------------------------------------------------------------------------

const RAD2DEG = 180 / Math.PI;
const dist = (a: Pt, b: Pt) => Math.hypot(a[0] - b[0], a[1] - b[1]);

function segDist(p: Pt, a: Pt, b: Pt): { d: number; at: Pt } {
  const dx = b[0] - a[0], dy = b[1] - a[1];
  const L2 = dx * dx + dy * dy;
  const t = L2 > 0 ? Math.max(0, Math.min(1, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L2)) : 0;
  const at: Pt = [a[0] + t * dx, a[1] + t * dy];
  return { d: dist(p, at), at };
}

/** Convex hull (monotone chain), counter-clockwise. */
function hull(pts: Pt[]): Pt[] {
  const s = pts.slice().sort((a, b) => a[0] - b[0] || a[1] - b[1]);
  if (s.length < 3) return s;
  const cross = (o: Pt, a: Pt, b: Pt) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const lower: Pt[] = [];
  for (const p of s) { while (lower.length >= 2 && cross(lower[lower.length - 2], lower[lower.length - 1], p) <= 0) lower.pop(); lower.push(p); }
  const upper: Pt[] = [];
  for (let i = s.length - 1; i >= 0; i--) { const p = s[i]; while (upper.length >= 2 && cross(upper[upper.length - 2], upper[upper.length - 1], p) <= 0) upper.pop(); upper.push(p); }
  return lower.slice(0, -1).concat(upper.slice(0, -1));
}

function polyArea(p: Pt[]): number {
  let a = 0;
  for (let i = 0; i < p.length; i++) { const q = p[(i + 1) % p.length]; a += p[i][0] * q[1] - q[0] * p[i][1]; }
  return Math.abs(a) / 2;
}

function farthestPair(pts: Pt[]): [Pt, Pt] {
  let best: [Pt, Pt] = [pts[0], pts[pts.length - 1]], bd = -1;
  for (let i = 0; i < pts.length; i++) for (let j = i + 1; j < pts.length; j++) { const d = dist(pts[i], pts[j]); if (d > bd) { bd = d; best = [pts[i], pts[j]]; } }
  return best;
}

function pointInPoly(p: Pt, poly: Pt[]): boolean {
  let inside = false;
  for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
    const [xi, yi] = poly[i], [xj, yj] = poly[j];
    if ((yi > p[1]) !== (yj > p[1]) && p[0] < ((xj - xi) * (p[1] - yi)) / (yj - yi) + xi) inside = !inside;
  }
  return inside;
}

// ---------------------------------------------------------------------------
// Simulation
// ---------------------------------------------------------------------------

type EP = { kind: 'ref'; ref: PointRef; pos: Pt } | { kind: 'body'; bi: number; pos: Pt } | { kind: 'free'; pos: Pt };
interface Pending { tool: 'link' | 'R' | 'P'; first: EP }
type Hover = { kind: 'group'; gi: number } | { kind: 'body'; bi: number } | { kind: 'joint'; ji: number } | null;
interface Snapshot { mech: Mechanism; key: PresetKey; pristine: boolean }
interface HandleInfo { h: Handle; gi: number; ref: PointRef }

/** Body shape in world coordinates, as drawn. */
type Shape = { kind: 'none' } | { kind: 'block'; p: Pt; ang: number } | { kind: 'bar'; a: Pt; b: Pt } | { kind: 'poly'; pts: Pt[] };

function mount(host: HTMLElement, ctx: LessonContext): LessonHandle {
  const root = el('div', { class: 'dg-root' });
  host.appendChild(root);
  root.appendChild(el('style', {}, CSS));

  // ---------------- state
  let mech: Mechanism = presetByKey('free').build();
  let presetKey: PresetKey = 'free';
  let pristine = true;
  let an: Analysis = analyze(mech);
  let groups: PointGroup[] = pointGroups(mech);
  let handleInfo: HandleInfo[] = [];
  let motions = new Map<number, PinMotion>();
  let mode: Mode = 'drag';
  let hl: Highlight = null;
  let hlSticky: Highlight = null;
  let mSpace: 3 | 6 = 3;
  let step = 0;
  let pending: Pending | null = null;
  let hover: Hover = null;
  let pointer: Pt | null = null;
  let message: { text: string; until: number } | null = null;
  let msgTimer = 0;
  let revealed = false;
  let guess: number | null = null;
  let trail: Pt[] = [];
  let guard = false; // suppresses chip callbacks while we repaint chips programmatically
  const undoStack: Snapshot[] = [];

  // ---------------- layout
  simTitle(root, '平面連桿積木（planar linkage sandbox）', 'dof 計數器');
  const chipHost = el('div', { class: 'dg-chiprow' });
  root.appendChild(chipHost);
  let stepChips: PresetChipsHandle | null = null;

  // Narrow panes (phones, small windows) get a taller canvas and a tighter x range so linkages stay large.
  const narrow = (host.clientWidth || 800) < 560;
  const worldFor = (key: PresetKey): World2 => {
    const w = presetByKey(key).world ?? DEFAULT_WORLD;
    return narrow ? { ...w, xmin: Math.max(w.xmin, -1.85), xmax: Math.min(w.xmax, 1.85) } : w;
  };
  const cv = new Canvas2D(root, { aspect: narrow ? 4 / 3 : 16 / 9, world: worldFor('free'), overlay: true, margin: 0.03 });
  /** Where the symbolic floor of a ground without attachments is drawn (bottom-left of the view). */
  const floorSpan = () => { const v = cv.visible(); const x0 = v.xmin + 0.15; return { x0, x1: x0 + 0.75, y: -0.2 }; };

  const tools = el('div', { class: 'dg-tools' });
  root.appendChild(tools);
  const toolChips = presetChips(tools, TOOLS.map((t) => ({ label: t.label, title: t.title, apply: () => { if (!guard) setMode(t.mode); } })), { label: '工具' });
  button(tools, { label: '↶ 復原', small: true, title: '復原上一個動作', onClick: () => undo() });
  button(tools, { label: '⟲ 重設', small: true, title: '回到目前預設機構的初始狀態', onClick: () => loadPreset(presetKey, true) });
  const presetSel = select<PresetKey>(tools, {
    label: '全部預設',
    options: PRESETS.map((p) => ({ value: p.key, label: p.label })),
    value: 'free',
    onChange: (k) => loadPreset(k, true),
  });

  const [colL, colR] = split(root);

  // Counter card ------------------------------------------------------------
  const counterCard = el('div', { class: 'dg-card' });
  colL.appendChild(counterCard);
  const counterHead = el('div', { class: 'dg-head' }, el('span', { class: 'dg-title' }, 'Grübler 計數'));
  counterCard.appendChild(counterHead);
  const mChips = presetChips(counterHead, [
    { label: 'm = 3 平面', title: '平面機構：每根桿 3 個自由度', apply: () => { if (!guard) setM(3); } },
    { label: 'm = 6 空間', title: '把同一個機構當成空間機構來數', apply: () => { if (!guard) setM(6); } },
  ]);
  const countsRow = el('div', { class: 'dg-counts' });
  counterCard.appendChild(countsRow);
  const countBtn = (k: 'N' | 'J' | 'f', title: string): HTMLButtonElement => {
    const b = el('button', { class: 'mr-chip', type: 'button', title });
    b.addEventListener('mouseenter', () => { if (!b.disabled) { hl = k; paintCountBtns(); cv.requestRender(); } });
    b.addEventListener('mouseleave', () => { hl = hlSticky; paintCountBtns(); cv.requestRender(); });
    b.addEventListener('click', () => { hlSticky = hlSticky === k ? null : k; hl = hlSticky; paintCountBtns(); cv.requestRender(); });
    countsRow.appendChild(b);
    return b;
  };
  const nBtn = countBtn('N', '連桿數（含地面）：滑過或點一下，畫面會替每根桿編號');
  const jBtn = countBtn('J', '關節數：滑過或點一下，畫面會替每個關節編號');
  const fBtn = countBtn('f', '各關節允許的相對自由度總和');
  const formulaEl = el('div', { class: 'dg-formula' });
  const checkEl = el('div', { class: 'dg-check' });
  const statusEl = el('div', { class: 'dg-status' });
  const spatialEl = el('div', { class: 'dg-spatial' }, mdElement(SPATIAL_MD));
  counterCard.append(formulaEl, checkEl, statusEl, spatialEl);

  // Right-hand panels -------------------------------------------------------
  const card = (title: string): HTMLDivElement => {
    const c = el('div', { class: 'dg-card' }, el('div', { class: 'dg-head' }, el('span', { class: 'dg-title' }, title)));
    colR.appendChild(c);
    return c;
  };

  const bodyCard = card('剛體的「點數法」（藍色桿）');
  const bodyKV = kv(bodyCard, ['A', 'B', '|AB|', '點數法', '(x, y, φ)']);
  bodyCard.appendChild(el('p', { class: 'dg-small' }, '3D 中要取三個不共線的點：9 個座標 − 3 個距離 = 6（3 個位置 + 3 個姿態）。'));

  const jointsCard = card('常見關節：允許 f 個、拿走 c = m − f 個');
  const table = el('table', { class: 'dg-table' });
  const th3 = el('th', {}, 'c', el('div', { class: 'dg-sub' }, 'm = 3'));
  const th6 = el('th', {}, 'c', el('div', { class: 'dg-sub' }, 'm = 6'));
  table.appendChild(el('thead', {}, el('tr', {}, el('th', {}, '關節'), el('th', {}, 'f'), th3, th6)));
  const tbody = el('tbody');
  const jointRows = JOINT_TABLE.map((j) => {
    const name = el('td', {}, el('b', {}, j.key), ` ${j.zh} `, el('span', { class: 'dg-en' }, j.en));
    const tr = el('tr', {}, name, el('td', {}, String(j.f)), el('td', {}, j.c3 == null ? '—' : String(j.c3)), el('td', {}, String(j.c6)));
    tbody.appendChild(tr);
    return { key: j.key, tr };
  });
  table.appendChild(tbody);
  jointsCard.appendChild(table);
  jointsCard.appendChild(el('p', { class: 'dg-small' }, '平面機構只用得到 R 與 P；H、C、U、S 是空間關節，平面中沒有意義（以 — 表示）。'));

  const predCard = card('先預測，再揭曉');
  predCard.appendChild(el('p', { class: 'dg-small' }, '從圖上數出 N、J、Σf，你預測這個機構的 dof 是？'));
  const predChips = presetChips(predCard, [0, 1, 2, 3].map((v) => ({ label: `dof = ${v}`, apply: () => { if (!guard) predict(v); } })));
  const predResult = el('div', { class: 'dg-result' });
  predCard.appendChild(predResult);
  button(predCard, { label: '直接看答案', small: true, variant: 'ghost', onClick: () => predict(null) });

  const redCard = card('約束真的都獨立嗎？');
  const redKV = kv(redCard, ['寫下的約束數 Σc', '獨立約束數（rank）', '多餘約束', '實際 dof']);
  const redNote = el('p', { class: 'dg-small' });
  redCard.appendChild(redNote);

  const csCard = card('組態空間預覽 (θ₁, θ₂)');
  const csHost = el('div', { class: 'dg-cs' });
  csCard.appendChild(csHost);
  const cs = new Canvas2D(csHost, { aspect: 1, world: { xmin: -4.35, xmax: 3.7, ymin: -4.35, ymax: 3.7 }, margin: 0.02 });
  const csSliders = el('div', { class: 'mr-controls compact' });
  csCard.appendChild(csSliders);
  const s1 = angleSlider(csSliders, { label: 'θ₁', value: ARM2R.theta0[0], onInput: () => setArmFromSliders() });
  const s2 = angleSlider(csSliders, { label: 'θ₂', value: ARM2R.theta0[1], onInput: () => setArmFromSliders() });
  const csNote = el('p', { class: 'dg-small' });
  csCard.appendChild(csNote);

  const panels: Record<PanelKey, HTMLElement> = { body: bodyCard, joints: jointsCard, predict: predCard, redundant: redCard, cspace: csCard };

  note(root, '藍色圓圈是手把；旁邊的箭頭表示這一點此刻能往哪些方向動（雙箭頭 = 1 個方向、十字 = 2 個、灰色手把 = 被鎖住）。工具列可以自己加桿、加關節、換地面。');

  // ---------------- helpers on the current mechanism
  const movingOrdinal = (bi: number): number => { let k = 0; for (let i = 0; i < bi; i++) if (!mech.bodies[i].fixed) k++; return k; };
  const bodyColor = (bi: number): string => cssVar(`--series-${(movingOrdinal(bi) % 7) + 1}`, '#2563eb');
  const sliderJointOf = (bi: number) => mech.joints.find((j) => j.type === 'P' && j.b === bi);

  function shapeOf(bi: number): Shape {
    const b = mech.bodies[bi];
    const pins = shapePins(mech, bi, groups).map((i) => worldPt(b, i));
    if (pins.length === 0) return { kind: 'none' };
    if (pins.length === 1) {
      const pj = sliderJointOf(bi);
      const ang = pj && pj.type === 'P' ? Math.atan2(worldDir(mech.bodies[pj.a], pj.dir)[1], worldDir(mech.bodies[pj.a], pj.dir)[0]) : b.pose.phi;
      return { kind: 'block', p: pins[0], ang };
    }
    const h = hull(pins);
    if (h.length >= 3 && polyArea(h) > 1e-3) return { kind: 'poly', pts: h };
    const [a, c] = farthestPair(pins);
    return { kind: 'bar', a, b: c };
  }

  function blockCorners(p: Pt, ang: number, w = 0.17, h = 0.1): Pt[] {
    const u: Pt = [Math.cos(ang), Math.sin(ang)], n: Pt = [-u[1], u[0]];
    return [[p[0] - u[0] * w - n[0] * h, p[1] - u[1] * w - n[1] * h], [p[0] + u[0] * w - n[0] * h, p[1] + u[1] * w - n[1] * h],
      [p[0] + u[0] * w + n[0] * h, p[1] + u[1] * w + n[1] * h], [p[0] - u[0] * w + n[0] * h, p[1] - u[1] * w + n[1] * h]];
  }

  /** Ground drawing data: anchor points, rails (P joints on the ground) and whether it has no visible feature. */
  function groundInfo() {
    const gi = fixedBody(mech);
    const anchors: Pt[] = [];
    const spare: Pt[] = [];
    const rails: [Pt, Pt][] = [];
    if (gi < 0) return { gi, anchors, spare, rails };
    groups.forEach((g) => {
      if (!g.fixed || g.railOnly) return;
      (g.rJoints.length || g.sliders.length ? anchors : spare).push(groupPos(mech, g));
    });
    mech.joints.forEach((j) => {
      if (j.type !== 'P' || j.a !== gi) return;
      const o = worldPt(mech.bodies[gi], j.pa);
      const d = worldDir(mech.bodies[gi], j.dir);
      const [e0, e1] = j.extent ?? [-0.6, 0.6];
      rails.push([[o[0] + d[0] * e0, o[1] + d[1] * e0], [o[0] + d[0] * e1, o[1] + d[1] * e1]]);
    });
    return { gi, anchors, spare, rails };
  }

  /** Where the "1 = ground" label goes. */
  function groundLabelPos(): Pt {
    const g = groundInfo();
    const pts = g.anchors.length ? g.anchors : g.rails.length ? g.rails.flat() : g.spare;
    if (!pts.length) { const f = floorSpan(); return [(f.x0 + f.x1) / 2, f.y + 0.1]; }
    const cx = pts.reduce((s, p) => s + p[0], 0) / pts.length;
    const cy = Math.min(...pts.map((p) => p[1]));
    return [cx, cy - 0.28];
  }

  function bodyLabelPos(bi: number): Pt | null {
    const s = shapeOf(bi);
    const px = 1 / cv.scale;
    if (s.kind === 'block') return [s.p[0], s.p[1] + 22 * px];
    if (s.kind === 'poly') return [s.pts.reduce((a, p) => a + p[0], 0) / s.pts.length, s.pts.reduce((a, p) => a + p[1], 0) / s.pts.length];
    if (s.kind === 'bar') {
      const m: Pt = [(s.a[0] + s.b[0]) / 2, (s.a[1] + s.b[1]) / 2];
      const L = dist(s.a, s.b) || 1;
      let n: Pt = [-(s.b[1] - s.a[1]) / L, (s.b[0] - s.a[0]) / L];
      if (n[1] < 0 || (Math.abs(n[1]) < 1e-9 && n[0] < 0)) n = [-n[0], -n[1]];
      return [m[0] + n[0] * 17 * px, m[1] + n[1] * 17 * px];
    }
    return null;
  }

  function jointPos(ji: number): Pt {
    const j = mech.joints[ji];
    return worldPt(mech.bodies[j.b], j.pb);
  }

  // ---------------- drawing
  cv.draw = (c) => {
    const fg = cssVar('--fg', '#222');
    const muted = cssVar('--fg-muted', '#777');
    const accent = cssVar('--accent', '#2563eb');
    const danger = cssVar('--danger', '#b91c1c');
    const bgElev = cssVar('--bg-elev', '#fff');
    const sunken = cssVar('--bg-sunken', '#eee');
    const px = 1 / c.scale;
    c.grid(0.5);

    // Ground ----------------------------------------------------------------
    const gInfo = groundInfo();
    if (gInfo.gi >= 0) {
      for (const [a, b] of gInfo.rails) {
        c.line(a, b, { color: muted, width: 3 });
        const L = dist(a, b), u: Pt = [(b[0] - a[0]) / L, (b[1] - a[1]) / L];
        for (let s = 0; s <= L + 1e-9; s += 12 * px) {
          const q: Pt = [a[0] + u[0] * s, a[1] + u[1] * s];
          c.line(q, [q[0] - u[0] * 4 * px + u[1] * 6 * px, q[1] - u[1] * 4 * px - u[0] * 6 * px], { color: muted, width: 1 });
        }
      }
      if (gInfo.anchors.length >= 2) {
        const h = hull(gInfo.anchors);
        if (h.length >= 3) c.polygon(h, { color: muted, width: 1.5, dash: [6, 5] });
        else c.line(h[0], h[h.length - 1], { color: muted, width: 1.5, dash: [6, 5] });
      }
      for (const p of gInfo.anchors) {
        const w = 9 * px, hh = 13 * px;
        c.polygon([p, [p[0] - w, p[1] - hh], [p[0] + w, p[1] - hh]], { fill: sunken, color: muted, width: 1.5 });
        c.line([p[0] - w * 1.6, p[1] - hh], [p[0] + w * 1.6, p[1] - hh], { color: muted, width: 2 });
        for (let k = -1.4; k <= 1.41; k += 0.7) c.line([p[0] + k * w, p[1] - hh], [p[0] + k * w - 4 * px, p[1] - hh - 5 * px], { color: muted, width: 1 });
      }
      for (const p of gInfo.spare) c.circle(p, 4 * px, { color: muted, width: 1.5 });
      if (!gInfo.anchors.length && !gInfo.rails.length && !gInfo.spare.length) {
        // Ground with no attachment yet: a symbolic floor so it can still be counted.
        const { x0, x1, y } = floorSpan();
        c.line([x0, y], [x1, y], { color: muted, width: 2 });
        for (let x = x0; x <= x1 + 1e-9; x += 10 * px) c.line([x, y], [x - 5 * px, y - 6 * px], { color: muted, width: 1 });
        c.text([(x0 + x1) / 2, y + 12 * px], '地面（固定）', { color: muted });
      } else if (gInfo.anchors.length >= 2 && hl !== 'N') {
        const lp = groundLabelPos();
        c.text([lp[0], lp[1] + 6 * px], '地面（固定桿）', { color: muted });
      }
    }

    // Moving bodies ---------------------------------------------------------------
    mech.bodies.forEach((b, bi) => {
      if (b.fixed) return;
      const s = shapeOf(bi);
      const col = bodyColor(bi);
      const hot = hover?.kind === 'body' && hover.bi === bi;
      const hotCol = mode === 'delete' ? danger : accent;
      if (s.kind === 'bar') {
        if (hot) c.line(s.a, s.b, { color: hotCol, width: 16, cap: 'round', alpha: 0.35 });
        c.line(s.a, s.b, { color: col, width: 10, cap: 'round', alpha: step === 0 ? 0.6 : 0.88 });
      } else if (s.kind === 'poly') {
        c.polygon(s.pts, { color: col, fill: col, width: 0, alpha: 0.25 });
        c.polygon(s.pts, { color: hot ? hotCol : col, width: hot ? 5 : 3 });
      } else if (s.kind === 'block') {
        const corners = blockCorners(s.p, s.ang);
        c.polygon(corners, { color: hot ? hotCol : fg, fill: col, width: hot ? 3 : 1.5, alpha: 0.92 });
      }
    });

    // Slots of P joints on moving rails + small carriages for multi-pin sliders.
    mech.joints.forEach((j) => {
      if (j.type !== 'P') return;
      const A = mech.bodies[j.a];
      const d = worldDir(A, j.dir);
      const ang = Math.atan2(d[1], d[0]);
      if (!A.fixed) {
        const o = worldPt(A, j.pa);
        const proj = shapePins(mech, j.a, groups).map((i) => { const q = worldPt(A, i); return (q[0] - o[0]) * d[0] + (q[1] - o[1]) * d[1]; });
        const lo = Math.min(...proj), hi = Math.max(...proj);
        if (hi - lo > 1e-6) c.line([o[0] + d[0] * lo, o[1] + d[1] * lo], [o[0] + d[0] * hi, o[1] + d[1] * hi], { color: bgElev, width: 3 });
      }
      if (shapePins(mech, j.b, groups).length >= 2) {
        const p = worldPt(mech.bodies[j.b], j.pb);
        c.polygon(blockCorners(p, ang, 0.13, 0.075), { color: fg, fill: bodyColor(j.b), width: 1.5 });
      }
    });

    // Joints and free pins ---------------------------------------------------------
    groups.forEach((g, gi) => {
      if (g.railOnly) return;
      const p = groupPos(mech, g);
      const hot = (hover?.kind === 'group' && hover.gi === gi) || (hover?.kind === 'joint' && g.rJoints.includes(hover.ji));
      if (g.rJoints.length) {
        if (g.rJoints.length > 1) c.circle(p, 10.5 * px, { color: fg, width: 1.3 });
        c.point(p, 6.5, { color: bgElev });
        c.circle(p, 6.5 * px, { color: fg, width: 2 });
      } else if (!g.sliders.length && !g.fixed) {
        c.point(p, 3.5, { color: fg });
      }
      if (hot) c.circle(p, 13 * px, { color: mode === 'delete' ? danger : accent, width: 2.5 });
    });
    if (hover?.kind === 'joint') {
      const j = mech.joints[hover.ji];
      if (j?.type === 'P') c.circle(jointPos(hover.ji), 15 * px, { color: danger, width: 2.5 });
    }

    // Motion hints next to the handles ---------------------------------------------------
    if (mode === 'drag') {
      for (const hi of handleInfo) {
        const mo = motions.get(hi.gi);
        if (!mo) continue;
        const p = worldPt(mech.bodies[hi.ref.body], hi.ref.pin);
        if (mo.dof === 1 && mo.dir) {
          const L = 25 * px;
          for (const sg of [1, -1]) c.arrow(p, [p[0] + sg * mo.dir[0] * L, p[1] + sg * mo.dir[1] * L], { color: accent, width: 1.6, head: 7, alpha: 0.75 });
        } else if (mo.dof === 2) {
          const L = 21 * px;
          for (const v of [[1, 0], [-1, 0], [0, 1], [0, -1]] as Pt[]) c.arrow(p, [p[0] + v[0] * L, p[1] + v[1] * L], { color: accent, width: 1.4, head: 6, alpha: 0.6 });
        } else if (mo.dof === 0) {
          const q: Pt = [p[0] + 12 * px, p[1] + 12 * px], r = 3.5 * px;
          c.line([q[0] - r, q[1] - r], [q[0] + r, q[1] + r], { color: danger, width: 2 });
          c.line([q[0] - r, q[1] + r], [q[0] + r, q[1] - r], { color: danger, width: 2 });
        }
      }
    }

    // Step 1 extras: body frames, A/B labels and the angle φ -------------------------------
    if (step === 0) {
      mech.bodies.forEach((b, bi) => {
        if (b.fixed) return;
        const o: Pt = [b.pose.x, b.pose.y];
        c.frame({ x: o[0], y: o[1], theta: b.pose.phi }, 0.46, { labels: ['x_b', 'y_b'], width: 2.5 });
        c.line(o, [o[0] + 0.36, o[1]], { color: muted, width: 1, dash: [3, 3] });
        const a0 = 0, a1 = b.pose.phi;
        if (Math.abs(a1) > 0.02) {
          c.arc(o, 0.2, Math.min(a0, a1), Math.max(a0, a1), { color: cssVar('--warn'), width: 2 });
          const am = a1 / 2 - Math.sign(a1) * 0.35;
          c.text([o[0] + 0.3 * Math.cos(am), o[1] + 0.3 * Math.sin(am)], 'φ', { color: cssVar('--warn'), font: `bold 13px ${cssVar('--font') || 'sans-serif'}` });
        }
      });
      const first = mech.bodies.findIndex((b) => !b.fixed);
      if (first >= 0) {
        const pins = shapePins(mech, first, groups);
        if (pins.length >= 2) {
          ['A', 'B'].forEach((lab, k) => {
            const p = worldPt(mech.bodies[first], pins[k]);
            c.text(p, lab, { dx: -14, dy: -16, color: fg, font: `bold 13px ${cssVar('--font') || 'sans-serif'}` });
          });
        }
      }
    }

    // 2R arm: joint angles θ₁, θ₂ and the outer reach circle ------------------------------
    if (presetKey === 'arm2R' && pristine && mech.bodies.length === 3) {
      const warnC = cssVar('--warn', '#b45309');
      const base = worldPt(mech.bodies[0], 0), elbow = worldPt(mech.bodies[1], 1);
      const [t1, t2] = arm2RAngles(mech);
      const p1 = mech.bodies[1].pose.phi;
      const lab = `bold 13px ${cssVar('--font') || 'sans-serif'}`;
      c.circle(base, ARM2R.L[0] + ARM2R.L[1], { color: cssVar('--grid-strong', '#ccc'), width: 1, dash: [4, 5] });
      c.line(base, [base[0] + 0.42, base[1]], { color: warnC, width: 1, dash: [3, 3] });
      if (Math.abs(t1) > 0.02) c.arc(base, 0.26, Math.min(0, t1), Math.max(0, t1), { color: warnC, width: 2 });
      c.text([base[0] + 0.42 * Math.cos(t1 / 2), base[1] + 0.42 * Math.sin(t1 / 2)], 'θ₁', { color: warnC, font: lab });
      c.line(elbow, [elbow[0] + 0.36 * Math.cos(p1), elbow[1] + 0.36 * Math.sin(p1)], { color: warnC, width: 1, dash: [3, 3] });
      if (Math.abs(t2) > 0.02) c.arc(elbow, 0.2, Math.min(p1, p1 + t2), Math.max(p1, p1 + t2), { color: warnC, width: 2 });
      c.text([elbow[0] + 0.36 * Math.cos(p1 + t2 / 2), elbow[1] + 0.36 * Math.sin(p1 + t2 / 2)], 'θ₂', { color: warnC, font: lab });
    }

    // N / J / Σf highlight labels ---------------------------------------------------------
    const badgeFont = `bold 12px ${cssVar('--font') || 'sans-serif'}`;
    const accentSoft = cssVar('--accent-soft', '#dbeafe');
    if (hl === 'N') {
      c.text(groundLabelPos(), '1（地面）', { color: accent, bg: accentSoft, font: badgeFont });
      let k = 2;
      mech.bodies.forEach((b, bi) => {
        if (b.fixed) return;
        const p = bodyLabelPos(bi);
        if (p) c.text(p, String(k), { color: accent, bg: accentSoft, font: badgeFont });
        k++;
      });
    } else if (hl === 'J' || hl === 'f') {
      const spots = new Map<string, { p: Pt; labels: string[] }>();
      mech.joints.forEach((j, ji) => {
        const p = jointPos(ji);
        const key = `${p[0].toFixed(3)},${p[1].toFixed(3)}`;
        const s = spots.get(key) ?? { p, labels: [] };
        s.labels.push(hl === 'J' ? `${ji + 1}` : `${j.type}: f=1`);
        spots.set(key, s);
      });
      for (const s of spots.values()) {
        c.circle(s.p, 13 * px, { color: accent, width: 2 });
        c.text(s.p, hl === 'J' ? s.labels.join(',') : s.labels.join(' + '), { dx: 0, dy: -24, color: accent, bg: accentSoft, font: badgeFont });
      }
    }

    // Edit previews ------------------------------------------------------------------------
    if (pending) {
      const p = pending.first.pos;
      c.point(p, 5, { color: accent });
      if (pointer) c.line(p, pointer, { color: accent, width: 1.5, dash: [5, 4] });
    }
    if (mode === 'pivot' && pointer) {
      const w = 9 * px, hh = 13 * px;
      c.polygon([pointer, [pointer[0] - w, pointer[1] - hh], [pointer[0] + w, pointer[1] - hh]], { color: accent, width: 1.5, alpha: 0.6 });
    }

    c.overlay.textContent = overlayText();
  };

  // C-space preview ----------------------------------------------------------------
  cs.draw = (c) => {
    const P = Math.PI;
    const muted = cssVar('--fg-muted', '#777');
    const accent = cssVar('--accent', '#2563eb');
    const ok = presetKey === 'arm2R' && pristine;
    c.polygon([[-P, -P], [P, -P], [P, P], [-P, P]], { fill: cssVar('--bg-sunken', '#eee'), color: cssVar('--grid-strong', '#ccc'), width: 1.5 });
    for (const v of [-P / 2, 0, P / 2]) {
      c.line([v, -P], [v, P], { color: cssVar('--grid', '#ddd'), width: 1 });
      c.line([-P, v], [P, v], { color: cssVar('--grid', '#ddd'), width: 1 });
    }
    // Edge identification marks: left/right share one arrow, bottom/top share a double arrow.
    const warn = cssVar('--warn', '#b45309');
    for (const x of [-P, P]) c.arrow([x, -0.35], [x, 0.35], { color: warn, width: 2, head: 8 });
    for (const y of [-P, P]) { c.arrow([-0.55, y], [0.05, y], { color: warn, width: 2, head: 8 }); c.arrow([-0.15, y], [0.45, y], { color: warn, width: 2, head: 8 }); }
    c.text([-P, -P], '−π', { dy: 12, color: muted });
    c.text([P, -P], 'π', { dy: 12, color: muted });
    c.text([0, -P], 'θ₁', { dy: 14, color: muted });
    c.text([-P, -P], '−π', { dx: -14, dy: -2, color: muted });
    c.text([-P, P], 'π', { dx: -12, color: muted });
    c.text([-P, 0], 'θ₂', { dx: -15, color: muted });
    if (!ok) {
      c.text([0, 0], '選「2R 手臂」預設', { color: muted });
      return;
    }
    // trail, broken where an angle wraps around
    let seg: Pt[] = [];
    for (let i = 0; i < trail.length; i++) {
      if (i > 0 && (Math.abs(trail[i][0] - trail[i - 1][0]) > P || Math.abs(trail[i][1] - trail[i - 1][1]) > P)) {
        c.polyline(seg, { color: accent, width: 1.5, alpha: 0.5 });
        seg = [];
      }
      seg.push(trail[i]);
    }
    c.polyline(seg, { color: accent, width: 1.5, alpha: 0.5 });
    const th = arm2RAngles(mech);
    c.point(th, 5.5, { color: cssVar('--danger', '#b91c1c') });
  };

  // ---------------- counter & panels
  function paintCountBtns(): void {
    [[nBtn, 'N'], [jBtn, 'J'], [fBtn, 'f']].forEach(([b, k]) => (b as HTMLElement).classList.toggle('active', hl === k));
  }

  function setStatus(cls: '' | 'ok' | 'warn' | 'lock', text: string): void {
    statusEl.className = `dg-status ${cls}`.trim();
    statusEl.textContent = text;
  }

  function updateCounter(): void {
    const g = grubler(mech, mSpace);
    const masked = step === 3 && !revealed;
    nBtn.textContent = masked ? 'N = ?' : `N = ${g.N}（含地面）`;
    jBtn.textContent = masked ? 'J = ?' : `J = ${g.J}`;
    fBtn.textContent = masked ? 'Σf = ?' : `Σf = ${g.sumF}`;
    [nBtn, jBtn, fBtn].forEach((b) => { b.disabled = masked; });
    if (masked && hl) { hl = null; hlSticky = null; paintCountBtns(); }
    const tex = masked
      ? `\\mathrm{dof} = ${mSpace}(N-1-J)+\\textstyle\\sum f_i = \\;?`
      : `\\mathrm{dof} = ${mSpace}(${g.N}-1-${g.J})+${g.sumF} = ${g.dof}`;
    formulaEl.innerHTML = renderMath(tex, true);
    spatialEl.classList.toggle('dg-hidden', mSpace !== 6);
    const moving = mech.bodies.filter((b) => !b.fixed).length;
    if (masked) {
      checkEl.textContent = '數值檢查：預測之後才會顯示。';
      setStatus('', '先在右側選擇你預測的 dof。');
      return;
    }
    if (an.maxResidual > 1e-6) { checkEl.textContent = ''; setStatus('warn', '⚠ 機構尚未組裝完成。'); return; }
    if (mSpace === 3) {
      checkEl.textContent = `數值檢查：${moving} 根會動的桿 → ${an.nVars} 個變數；寫下 ${an.sumC} 條約束，其中獨立的有 ${an.rank} 條 → 實際 dof = ${an.nVars} − ${an.rank} = ${an.mobility}`;
      const a = an.mobility;
      if (a === g.dof) {
        if (a > 0) setStatus('ok', `✓ dof = ${a}：需要 ${a} 個獨立的數（輸入）才能決定整個機構的姿態。`);
        else setStatus('lock', '■ dof = 0：這是結構，不是機構——所有手把都拖不動。');
      } else if (g.dof < 0 && a === 0) {
        setStatus('warn', `⚠ 公式給 ${g.dof}，但 dof 不會小於 0：這是超靜定結構，有 ${an.redundant} 條多餘約束。`);
      } else {
        setStatus('warn', `⚠ Grübler 低估了：公式給 ${g.dof}，實際能動 ${a} 個自由度——有 ${an.redundant} 條約束不獨立（特殊幾何）。`);
      }
    } else {
      const sp = spatialMobility(mech, an);
      checkEl.textContent = `把同一個機構放進 3D（R 軸都平行於 z）：實際 dof = ${sp}`;
      if (sp === g.dof) setStatus('ok', `✓ 在 3D 中 dof = ${sp}，與 m = 6 的 Grübler 一致（開鏈或自由物體不受影響）。`);
      else setStatus('warn', `⚠ 用 m = 6 數得 ${g.dof}，實際卻是 ${sp}：平面機構的 R 軸彼此平行，很多空間約束並不獨立。平面機構請用 m = 3。`);
    }
  }

  function updateBodyPanel(): void {
    const bi = mech.bodies.findIndex((b) => !b.fixed);
    const keys = ['A', 'B', '|AB|', '點數法', '(x, y, φ)'];
    if (bi < 0) { keys.forEach((k) => bodyKV.set(k, '—')); return; }
    const b = mech.bodies[bi];
    const pins = shapePins(mech, bi, groups);
    if (pins.length >= 2) {
      const A = worldPt(b, pins[0]), B = worldPt(b, pins[1]);
      bodyKV.set('A', `(${fmt(A[0], 2)}, ${fmt(A[1], 2)})`);
      bodyKV.set('B', `(${fmt(B[0], 2)}, ${fmt(B[1], 2)})`);
      bodyKV.set('|AB|', `${fmt(dist(A, B), 3)}（固定）`);
      bodyKV.set('點數法', '4 個座標 − 1 個約束 = 3');
    } else {
      ['A', 'B', '|AB|'].forEach((k) => bodyKV.set(k, '—'));
      bodyKV.set('點數法', '選「單一自由桿」');
    }
    bodyKV.set('(x, y, φ)', `(${fmt(b.pose.x, 2)}, ${fmt(b.pose.y, 2)}, ${fmt(b.pose.phi * RAD2DEG, 0)}°)`);
  }

  function updateJointTable(): void {
    const present = new Set(mech.joints.map((j) => j.type as string));
    jointRows.forEach((r) => r.tr.classList.toggle('on', present.has(r.key)));
    th3.classList.toggle('on', mSpace === 3);
    th6.classList.toggle('on', mSpace === 6);
  }

  function updatePredict(): void {
    if (!revealed) { predResult.className = 'dg-result'; predResult.textContent = '（尚未預測）'; return; }
    const g = grubler(mech, 3).dof;
    const a = an.mobility;
    const extra = a !== g ? `（不過數值檢查發現它實際能動 ${a} 個——下一步會解釋）` : '';
    if (guess == null) {
      predResult.className = 'dg-result';
      predResult.textContent = `答案：Grübler 給 ${g}${extra}。拖曳手把確認看看。`;
    } else if (guess === g) {
      predResult.className = 'dg-result ok';
      predResult.textContent = `✓ 答對了！Grübler 給 ${g}${extra}。拖曳手把確認它真的有 ${a} 個自由度。`;
    } else {
      predResult.className = 'dg-result bad';
      const hasP = mech.joints.some((j) => j.type === 'P');
      predResult.textContent = `✗ 你預測 ${guess}，Grübler 給 ${g}。再數一次：地面有沒有算進 N？${hasP ? '滑塊的 P 關節有沒有漏算？' : ''}每個關節只算一次。`;
    }
  }

  function updateRedundant(): void {
    const moving = mech.bodies.filter((b) => !b.fixed).length;
    redKV.set('寫下的約束數 Σc', `${an.sumC}（每個 R、P 各 2 條）`);
    redKV.set('獨立約束數（rank）', String(an.rank));
    redKV.set('多餘約束', String(an.redundant));
    redKV.set('實際 dof', `3·${moving} − ${an.rank} = ${an.mobility}`);
    redNote.textContent = an.redundant > 0
      ? `有 ${an.redundant} 條約束只是在重複其他約束已經保證的事，所以實際 dof 比 Grübler 多 ${an.redundant}。`
      : '所有約束都獨立：Grübler 的答案就是真正的 dof。';
  }

  function updateCspace(): void {
    const ok = presetKey === 'arm2R' && pristine;
    s1.disabled = !ok; s2.disabled = !ok;
    if (ok) {
      const th = arm2RAngles(mech);
      s1.set(th[0], { silent: true });
      s2.set(th[1], { silent: true });
      csNote.textContent = '方格的左右兩邊（θ₁ = ±180°）是同一個姿態，上下兩邊也是：成對的箭頭標出要「黏起來」的邊。';
    } else {
      csNote.textContent = '組態空間預覽只對未修改的「2R 手臂」預設有效。';
    }
    cs.requestRender();
  }

  function refresh(): void {
    an = analyze(mech);
    motions = new Map();
    const muted = cssVar('--fg-muted', '#888');
    for (const hi of handleInfo) {
      const mo = pinMotion(mech, an, hi.ref);
      motions.set(hi.gi, mo);
      hi.h.color = mo.dof === 0 ? muted : undefined;
    }
    updateCounter();
    updateBodyPanel();
    updateJointTable();
    updatePredict();
    updateRedundant();
    updateCspace();
    cv.requestRender();
  }

  // ---------------- handles
  function rebuildHandles(): void {
    for (const hi of handleInfo) cv.removeHandle(hi.h);
    handleInfo = [];
    groups.forEach((g, gi) => {
      if (g.fixed || g.railOnly) return;
      const ref = g.members.find((r) => !mech.bodies[r.body].fixed);
      if (!ref) return;
      const h: Handle = {
        get: () => worldPt(mech.bodies[ref.body], ref.pin),
        set: (p) => onDrag(ref, p),
        radius: 8,
        enabled: mode === 'drag',
        onStart: () => pushUndo(),
        onEnd: () => refresh(),
      };
      cv.addHandle(h);
      handleInfo.push({ h, gi, ref });
    });
  }

  function onDrag(ref: PointRef, p: Pt): void {
    let r: ReturnType<typeof dragPin>;
    try { r = dragPin(mech, ref, p); } catch (e) { console.warn(e); return; }
    if (r.locked) flash('這一點動不了：它所在的部分 dof = 0。', 1400);
    if (presetKey === 'arm2R' && pristine) pushTrail();
    refresh();
  }

  // ---------------- structure changes
  function structureChanged(): void {
    groups = pointGroups(mech);
    pending = null;
    hover = null;
    rebuildHandles();
    refresh();
  }

  function pushUndo(): void {
    undoStack.push({ mech: cloneMechanism(mech), key: presetKey, pristine });
    if (undoStack.length > 40) undoStack.shift();
  }

  function undo(): void {
    const s = undoStack.pop();
    if (!s) { flash('沒有可以復原的動作。'); return; }
    mech = s.mech; presetKey = s.key; pristine = s.pristine;
    presetSel.set(presetKey, { silent: true });
    cv.setWorld(worldFor(presetKey));
    syncStepChips();
    if (step === 3) resetPrediction();
    structureChanged();
  }

  function loadPreset(key: PresetKey, withUndo: boolean): void {
    if (withUndo) pushUndo();
    const def = presetByKey(key);
    mech = def.build();
    presetKey = key;
    pristine = true;
    trail = [];
    cv.setWorld(worldFor(key));
    presetSel.set(key, { silent: true });
    syncStepChips();
    if (step === 3) resetPrediction();
    structureChanged();
    if (key === 'arm2R') pushTrail();
  }

  /** Run an edit on a copy, re-assemble it, and commit only if every joint can be satisfied. */
  function applyEdit(fn: (m: Mechanism) => string | null): boolean {
    const work = cloneMechanism(mech);
    let err: string | null;
    try { err = fn(work); } catch (e) { console.warn(e); err = '這個操作無法完成。'; }
    if (err) { flash(err); return false; }
    let res = Infinity;
    try { res = project(work, 80); } catch (e) { console.warn(e); }
    if (!(res < 1e-7)) { flash('接不起來：機構無法同時滿足所有關節（已取消）。'); return false; }
    pushUndo();
    mech = work;
    pristine = false;
    trail = [];
    if (step === 3) resetPrediction();
    structureChanged();
    return true;
  }

  // ---------------- edit tools
  const tolPx = (n: number) => n / cv.scale;

  function nearestGroup(p: Pt, px = 14): number | null {
    let best: number | null = null, bd = tolPx(px);
    groups.forEach((g, gi) => {
      if (g.railOnly) return;
      const d = dist(groupPos(mech, g), p);
      if (d <= bd) { bd = d; best = gi; }
    });
    return best;
  }

  function bodyAt(p: Pt, includeFixed: boolean): { bi: number; at: Pt } | null {
    let best: { bi: number; at: Pt } | null = null, bd = Infinity;
    const tol = tolPx(10);
    mech.bodies.forEach((b, bi) => {
      if (b.fixed) {
        if (!includeFixed) return;
        const gInfo = groundInfo();
        const cands: { d: number; at: Pt }[] = [];
        // A click on an anchor symbol means the ground itself, even though moving links end there too.
        for (const a of [...gInfo.anchors, ...gInfo.spare]) if (dist(a, p) <= tolPx(9)) cands.push({ d: -1, at: a });
        for (const [a, c] of gInfo.rails) cands.push(segDist(p, a, c));
        const h = hull(gInfo.anchors);
        const edges = h.length >= 3 ? h.length : h.length - 1; // closed polygon, or one segment
        for (let i = 0; i < edges; i++) cands.push(segDist(p, h[i], h[(i + 1) % h.length]));
        for (const c of cands) if (c.d < tol && c.d < bd) { bd = c.d; best = { bi, at: c.at }; }
        return;
      }
      const s = shapeOf(bi);
      let d = Infinity, at: Pt = p;
      if (s.kind === 'bar') { const r = segDist(p, s.a, s.b); d = Math.max(0, r.d - tolPx(5)); at = r.at; }
      else if (s.kind === 'block') { d = pointInPoly(p, blockCorners(s.p, s.ang)) ? 0 : dist(p, s.p) - 0.12; at = s.p; }
      else if (s.kind === 'poly') {
        if (pointInPoly(p, s.pts)) { d = 0; at = p; }
        else for (let i = 0; i < s.pts.length; i++) { const r = segDist(p, s.pts[i], s.pts[(i + 1) % s.pts.length]); if (r.d < d) { d = r.d; at = r.at; } }
      }
      // later (top-most) bodies win ties
      if (d < tol && d <= bd) { bd = d; best = { bi, at }; }
    });
    return best;
  }

  function jointAt(p: Pt): number | null {
    let best: number | null = null, bd = tolPx(12);
    mech.joints.forEach((_, ji) => {
      const d = dist(jointPos(ji), p);
      if (d <= bd) { bd = d; best = ji; }
    });
    return best;
  }

  function resolveEndpoint(p: Pt): EP {
    const gi = nearestGroup(p);
    if (gi != null) {
      const g = groups[gi];
      const ref = g.members.find((r) => mech.bodies[r.body].fixed) ?? g.members[0];
      return { kind: 'ref', ref, pos: groupPos(mech, g) };
    }
    const hb = bodyAt(p, true);
    if (hb) return { kind: 'body', bi: hb.bi, pos: hb.at };
    return { kind: 'free', pos: p };
  }

  function materialize(m: Mechanism, e: EP): Endpoint {
    if (e.kind === 'ref') return { ref: e.ref };
    if (e.kind === 'body') return { ref: { body: e.bi, pin: addPinAt(m, e.bi, e.pos) } };
    return { free: e.pos };
  }

  function sameGroup(a: PointRef, b: PointRef): boolean {
    return groups.some((g) => g.members.some((r) => r.body === a.body && r.pin === a.pin) && g.members.some((r) => r.body === b.body && r.pin === b.pin));
  }

  function onEditClick(p: Pt): void {
    if (mode === 'pivot') {
      const gi = fixedBody(mech);
      if (gi < 0) return;
      if (nearestGroup(p) != null) { flash('這裡已經有接點了。'); return; }
      if (applyEdit((m) => { addPinAt(m, gi, p); return null; })) flash('新增了一個地面支點：用「+ 連桿」把桿接上去。', 2200);
      return;
    }
    if (mode === 'link') {
      const ep = resolveEndpoint(p);
      if (!pending) { pending = { tool: 'link', first: ep }; cv.requestRender(); return; }
      const first = pending.first;
      pending = null;
      if (mech.bodies.length >= MAX_BODIES) { flash(`最多 ${MAX_BODIES} 根桿（含地面）。`); return; }
      const name = `桿 ${mech.bodies.length}`;
      applyEdit((m) => {
        const e1 = materialize(m, first), e2 = materialize(m, ep);
        return addLink(m, e1, e2, name) == null ? '兩個端點太近了。' : null;
      });
      return;
    }
    if (mode === 'R') {
      const ep = resolveEndpoint(p);
      if (ep.kind === 'free') { flash('請點在接點或桿身上。'); return; }
      if (!pending) { pending = { tool: 'R', first: ep }; cv.requestRender(); return; }
      const first = pending.first;
      pending = null;
      applyEdit((m) => {
        const a = materialize(m, first), b = materialize(m, ep);
        if (!('ref' in a) || !('ref' in b)) return '請點在接點或桿身上。';
        if (a.ref.body === b.ref.body) return '同一根桿上的兩點本來就固定在一起。';
        if (first.kind === 'ref' && ep.kind === 'ref' && sameGroup(first.ref, ep.ref)) return '這兩點已經用關節接在一起了。';
        return addRJoint(m, a.ref, b.ref) ? null : '這兩點已經用 R 關節接在一起了。';
      });
      return;
    }
    if (mode === 'P') {
      if (!pending) {
        const ep = resolveEndpoint(p);
        if (ep.kind === 'free') { flash('先點要滑動的接點（或桿身上的一點）。'); return; }
        pending = { tool: 'P', first: ep };
        cv.requestRender();
        return;
      }
      const hb = bodyAt(p, true);
      if (!hb) { flash('請點一根桿（或地面支點）當作滑軌。'); return; }
      const first = pending.first;
      pending = null;
      applyEdit((m) => {
        const s = materialize(m, first);
        if (!('ref' in s)) return '請先點一個接點。';
        if (s.ref.body === hb.bi) return '滑軌必須是另一根桿。';
        return addPJoint(m, s.ref, hb.bi) ? null : '無法建立 P 關節。';
      });
      return;
    }
    if (mode === 'delete') {
      const ji = jointAt(p);
      if (ji != null) { applyEdit((m) => (removeJoint(m, ji) ? null : '刪除失敗。')); return; }
      const hb = bodyAt(p, true);
      if (!hb) return;
      if (mech.bodies[hb.bi].fixed) { flash('地面不能刪除：Grübler 公式永遠需要一根固定桿。'); return; }
      applyEdit((m) => (removeBody(m, hb.bi) ? null : '刪除失敗。'));
      return;
    }
    if (mode === 'ground') {
      const hb = bodyAt(p, false);
      if (!hb) return;
      if (applyEdit((m) => (setGround(m, hb.bi) ? null : '目前的地面沒有任何接點，換掉它會讓它變成看不見的漂浮物體。'))) {
        flash('運動學反轉：換一根桿當地面，N、J 都沒變，所以 dof 也不變。', 2800);
      }
    }
  }

  function updateHover(p: Pt): void {
    let h: Hover = null;
    if (mode === 'link' || mode === 'R' || (mode === 'P' && !pending)) {
      const gi = nearestGroup(p);
      if (gi != null) h = { kind: 'group', gi };
      else { const hb = bodyAt(p, true); if (hb && !mech.bodies[hb.bi].fixed) h = { kind: 'body', bi: hb.bi }; }
    } else if (mode === 'P' && pending) {
      const hb = bodyAt(p, true);
      if (hb && !mech.bodies[hb.bi].fixed) h = { kind: 'body', bi: hb.bi };
    } else if (mode === 'delete') {
      const ji = jointAt(p);
      if (ji != null) h = { kind: 'joint', ji };
      else { const hb = bodyAt(p, false); if (hb) h = { kind: 'body', bi: hb.bi }; }
    } else if (mode === 'ground') {
      const hb = bodyAt(p, false);
      if (hb) h = { kind: 'body', bi: hb.bi };
    }
    hover = h;
  }

  const offPointer = cv.onPointer({
    down: (p) => { if (mode !== 'drag') onEditClick(p); },
    move: (p) => { pointer = p; if (mode !== 'drag') { updateHover(p); cv.requestRender(); } },
  });
  const onLeave = () => { pointer = null; hover = null; cv.requestRender(); };
  cv.canvas.addEventListener('pointerleave', onLeave);

  // ---------------- modes, messages, step chips
  function setMode(m: Mode): void {
    mode = m;
    pending = null;
    hover = null;
    for (const hi of handleInfo) hi.h.enabled = m === 'drag';
    guard = true; toolChips.activate(TOOLS.findIndex((t) => t.mode === m)); guard = false;
    cv.canvas.style.cursor = m === 'drag' ? '' : 'crosshair';
    cv.requestRender();
  }

  function setM(v: 3 | 6): void {
    mSpace = v;
    guard = true; mChips.activate(v === 3 ? 0 : 1); guard = false;
    updateCounter();
    updateJointTable();
  }

  function flash(text: string, ms = 2600): void {
    message = { text, until: performance.now() + ms };
    window.clearTimeout(msgTimer);
    msgTimer = window.setTimeout(() => { message = null; cv.requestRender(); }, ms + 30);
    cv.requestRender();
  }

  function overlayText(): string {
    if (message && performance.now() < message.until) return message.text;
    const edited = pristine ? '' : '（已修改）';
    switch (mode) {
      case 'drag': return `拖曳手把${edited}`;
      case 'link': return pending ? '+ 連桿：再點第二個端點（空白處 = 自由端）' : '+ 連桿：點第一個端點（接點、桿身或空白處）';
      case 'pivot': return '+ 支點：點一下，新增固定在地面的接點';
      case 'R': return pending ? '+ R：點另一根桿上的點，兩點會被拉在一起' : '+ R：點第一個接點（或桿身上的一點）';
      case 'P': return pending ? '+ P：點當作滑軌的桿（或地面支點）' : '+ P：先點要滑動的接點';
      case 'delete': return '刪除：點關節或連桿（地面不能刪）';
      case 'ground': return '設為地面：點一根桿，讓它固定不動';
    }
    return '';
  }

  function syncStepChips(): void {
    if (!stepChips) return;
    const idx = STEP_CHIPS[step].findIndex((c) => c.key === presetKey);
    guard = true;
    if (idx >= 0) stepChips.activate(idx); else stepChips.clear();
    guard = false;
  }

  function buildStepChips(i: number): void {
    chipHost.replaceChildren();
    stepChips = presetChips(chipHost, STEP_CHIPS[i].map((c) => ({
      label: c.label,
      title: presetByKey(c.key).title,
      apply: () => { if (!guard) loadPreset(c.key, true); },
    })), { label: STEP_CHIP_LABEL[i] });
    syncStepChips();
  }

  function predict(v: number | null): void {
    guess = v;
    revealed = true;
    if (v == null) { guard = true; predChips.clear(); guard = false; }
    updateCounter();
    updatePredict();
  }

  function resetPrediction(): void {
    guess = null;
    revealed = false;
    guard = true; predChips.clear(); guard = false;
    updatePredict();
  }

  function pushTrail(): void {
    trail.push(arm2RAngles(mech));
    if (trail.length > 600) trail.shift();
  }

  function setArmFromSliders(): void {
    if (!(presetKey === 'arm2R' && pristine)) return;
    setArm2R(mech, [s1.value, s2.value]);
    pushTrail();
    refresh();
  }

  // ---------------- step focus
  let focused: HTMLElement[] = [];
  const FOCUS: HTMLElement[][] = [[bodyCard], [chipHost, jointsCard], [countsRow], [predCard], [statusEl, redCard], [csCard]];
  function setFocus(i: number): void {
    focused.forEach((f) => f.classList.remove('mr-focus'));
    focused = FOCUS[i] ?? [];
    focused.forEach((f) => f.classList.add('mr-focus'));
  }

  function showPanel(k: PanelKey): void {
    (Object.keys(panels) as PanelKey[]).forEach((key) => panels[key].classList.toggle('dg-hidden', key !== k));
    if (k === 'cspace') cs.requestRender();
  }

  function onStep(i: number): void {
    step = Math.max(0, Math.min(steps.length - 1, i));
    setM(3);
    buildStepChips(step);
    hlSticky = step === 2 ? 'N' : null;
    hl = hlSticky;
    paintCountBtns();
    setMode('drag');
    revealed = false;
    guess = null;
    if (!STEP_CHIPS[step].some((c) => c.key === presetKey)) loadPreset(STEP_DEFAULT[step], true);
    if (step === 3) resetPrediction();
    showPanel(STEP_PANEL[step]);
    setFocus(step);
    refresh();
  }

  // ---------------- keyboard: Esc cancels a half-finished edit
  const onKey = (ev: KeyboardEvent) => {
    if (ev.key !== 'Escape') return;
    if (pending) { pending = null; cv.requestRender(); } else if (mode !== 'drag') setMode('drag');
  };
  window.addEventListener('keydown', onKey);
  const unsubTheme = ctx.onThemeChange(() => { refresh(); cs.requestRender(); });

  // ---------------- boot
  loadPreset('free', false);
  onStep(ctx.currentStep);

  return {
    onStep,
    destroy() {
      window.clearTimeout(msgTimer);
      window.removeEventListener('keydown', onKey);
      cv.canvas.removeEventListener('pointerleave', onLeave);
      offPointer();
      unsubTheme();
      for (const hi of handleInfo) cv.removeHandle(hi.h);
      handleInfo = [];
      cv.destroy();
      cs.destroy();
      root.remove();
    },
  };
}

const lesson: Lesson = { meta, steps, quiz, mount };
export default lesson;
