import type { Lesson, LessonContext, LessonHandle, LessonStep, QuizQuestion } from '@/types';
import {
  el, cssVar, fmt, controlPanel, row, slider, toggle, button, select, kv, note, simTitle, legend, presetChips, rafThrottle,
  Canvas2D, RasterLayer, createLoop, type Pt, type RGBA, type ToggleHandle, type World,
} from '@/ui';
import { wrapToPi, type P2 } from '@/mr';
import { meta } from './meta';
import {
  HIT_LINK1, HIT_LINK2, angleCell, armPoints, blockedFraction, cellAngle, checkPath, circleCover, clearance, componentAt,
  computeCobsCodes, linkCollisions, pathAt, seamRuns, torusComponents, torusDist, torusSegmentPieces, wrapQ,
  type ClearanceResult, type CollisionMode, type Components, type Obstacle, type PathCheck,
} from './cspace';
import { LINK_HALF_WIDTH as W, SCENARIOS } from './scenarios';

const steps: LessonStep[] = [
  {
    title: '規劃問題長什麼樣',
    md: `**運動規劃**（motion planning）問的是：給定起點組態 $q_{\\rm start}$、目標組態 $q_{\\rm goal}$ 與障礙物，能否找到一條**連續**路徑把兩者接起來，而且途中機器人的**整個身體**都不碰到障礙？

所有不碰撞的組態構成自由空間 $\\mathcal C_{\\rm free}$，要找的就是一條曲線

$$q:[0,1]\\to\\mathcal C_{\\rm free},\\qquad q(0)=q_{\\rm start},\\; q(1)=q_{\\rm goal}.$$

右圖先只畫出 S、G 與兩者之間的「關節空間直線」。只看末端軌跡（虛線）會以為一路暢通，但真正要檢查的是兩根有粗細的連桿。`,
    simHint: '按「▶ 播放路徑」：末端的虛線軌跡離障礙很遠，手臂卻在半路變紅。是哪個部位撞上的？右圖的紅點標出碰撞發生在直線上的哪一段。',
  },
  {
    title: '一個障礙在 C-space 裡的樣子',
    md: `想知道哪些組態會碰撞，最直接的方法是**掃描**：逐一擺出每個 $(\\theta_1,\\theta_2)$ 做碰撞查詢，撞到就在右圖塗色。塗出來的集合就是 **C-obstacle**：

$$\\mathcal C_{\\rm obs}=\\{\\,q \\mid \\mathcal A(q)\\cap\\mathcal B\\neq\\emptyset\\,\\},\\qquad \\mathcal C_{\\rm free}=\\mathcal C\\setminus\\mathcal C_{\\rm obs}$$

其中 $\\mathcal A(q)$ 是手臂在 $q$ 佔據的區域，$\\mathcal B$ 是障礙。同一個圓在這裡長成兩種形狀：連桿 1 撞不撞與 $\\theta_2$ 無關，所以是**整條垂直帶**（紅），障礙離基座越近、從肩關節看去張得越開，這條帶就越寬；只有連桿 2 撞到的部分則是彎曲的斑塊（橙）。`,
    simHint: '掃描從目前的 θ₁ 開始繞一圈：左圖的扇形是「θ₁ 固定、θ₂ 轉一圈」的所有前臂，紅色的那幾支就對應右圖同一欄被塗色的格子。掃完後把右圖的 q 點拖進色塊試試；想再看一次就按「▶ 掃描」。',
  },
  {
    title: '環面上的障礙',
    md: `關節可以一直轉，$\\theta=180°$ 與 $\\theta=-180°$ 是同一個姿勢，所以 2R 的 C-space 其實是**環面** $T^2$，方格只是沿兩條「接縫」把它剪開攤平。

障礙在手臂背後時，連桿 1 在 $\\theta_1\\approx\\pm180°$ 附近碰撞，C-obstacle 被方格的左右兩邊各切走一部分。邊緣上的彩色標記表示這兩邊在環面上黏在一起；方格外的淡色區是週期延拓的複本。接縫的位置是人為選的：把它移到 $0°$，兩塊就合回一塊。`,
    simHint: '先看左右邊緣的青色標記（θ₁ 接縫）與上下邊緣的粉色標記（θ₂ 接縫），再勾選「接縫移到 0°」。S→G 的直線也跨過了接縫，所以被畫成兩段。',
  },
  {
    title: '碰撞查詢與距離',
    md: `規劃器動輒呼叫上萬次碰撞查詢，所以要快。常見做法是用幾個圓（3D 用球）蓋住每根連桿，障礙也用圓描述，兩圓之間的距離只要比較圓心：

$$d_{ij}=\\|c_i-c_j\\|-r_i-r_j$$

所有圓對的最小值 $d_{\\min}\\le 0$ 就判定碰撞，正值則是**間隙**（clearance）的保守估計。覆蓋圓比連桿略胖，C-obstacle 因此多出一圈紫色的保守區：寧可誤報，絕不漏報。每桿圓數 $K$ 越多，這圈越薄。`,
    simHint: '拖曳手肘或末端靠近障礙，比較 d_min 的精確值與圓覆蓋值（藍色虛線是最近的一對點）；再把 K 從 2 拉到 8，看紫色保守區變薄。',
  },
  {
    title: '連通性：繞得過去嗎',
    md: `$\\mathcal C_{\\rm free}$ 可能裂成好幾個**連通分量（connected components）**。起點與終點若落在不同分量，任何規劃器都找不到路徑：不是演算法不夠聰明，而是解根本不存在。

左圖兩個障礙貼著基座上下兩側。末端看似能從外圍繞過去，但連桿 1 固定在基座上，往上或往下轉都會撞到；在 C-space 裡，這是兩條完整的垂直帶，把環面切成「手臂朝右」與「手臂朝左」兩塊。`,
    simHint: '兩個分量以不同底色顯示。把其中一個障礙拖離基座，看「連通分量」何時由 2 變成 1，S→G 直線的檢查結果又有什麼變化。',
  },
  {
    title: '畫一條路徑，以及下一課',
    md: `換你規劃：從 S 出發，在右圖點選或拖曳出路徑點，靠近 G 時會自動接上。相鄰兩點之間取環面上較短的那段直線，程式每 0.02 rad 做一次碰撞查詢，撞到的地方標成紅點。

比較成本：檢查一條路徑只要幾百次查詢，掃出整張 C-obstacle 卻要查遍每一格；在 6 維的 C-space 中後者幾乎做不到，所以多數規劃器只在需要時呼叫碰撞查詢。下一課先用最直接的離散化：把 C-space 切成格子，再做圖搜尋。`,
    simHint: '畫一條 S→G 的無碰撞路徑（提示：讓手臂往反方向轉，穿過 ±180° 的接縫），再按「▶ 播放路徑」驗證。勾選「格點預覽」先看看下一課的格子。',
  },
];

const quiz: QuizQuestion[] = [
  {
    question: '在「單一障礙」場景中，障礙不動，只把第二桿 $L_2$ 從 1.0 m 拉長到 1.3 m。右圖的 C-obstacle 會怎麼變？',
    options: [
      '整個 C-obstacle 等比例放大約 1.3 倍',
      '紅色垂直帶（連桿 1 碰撞）完全不變，只有橙色斑塊（只有連桿 2 碰撞）改變',
      '紅色垂直帶變寬，橙色斑塊不變',
      '完全不變，因為障礙沒有移動',
    ],
    answer: 1,
    explanation: '連桿 1 撞不撞只取決於 $\\theta_1$ 與 $L_1$，所以那幾欄紅色垂直帶與 $L_2$ 毫無關係。會變的是前臂掃過的範圍：前臂變長、伸得更遠，原本碰不到的 $(\\theta_1,\\theta_2)$ 也會撞上，橙色斑塊只會變大、不會縮小（較長的前臂掃過的範圍包含原本的範圍；本場景的佔比由 13.6% 增為 14.5%）。拉動「第二桿長 L₂」滑桿就能驗證。',
  },
  {
    question: '用圓覆蓋做碰撞查詢，覆蓋圓與障礙之間的最小距離為 $d_{\\min}=0.05$ m。對真正的手臂，哪個結論**一定**成立？',
    options: [
      '手臂沒有碰撞，而且真正的間隙至少 0.05 m',
      '手臂可能已經碰撞，必須再用精確方法確認',
      '真正的間隙剛好等於 0.05 m',
      '真正的間隙一定小於 0.05 m',
    ],
    answer: 0,
    explanation: '覆蓋圓把每根連桿整個包住，手臂上每一點都落在某個覆蓋圓裡，所以手臂離障礙不可能比覆蓋圓更近：真正的間隙 $\\ge 0.05$ m。反方向則不成立：覆蓋圓回報 $d_{\\min}\\le 0$ 時，手臂本身可能還沒碰到（紫色保守區）。正值可以放心採信，負值只代表「可能碰撞」。',
  },
  {
    question: '在「夾住基座」場景中，S 讓手臂朝右、G 讓手臂朝左。下列哪個做法能讓規劃成功？',
    options: [
      '把 C-space 的格子切得更細',
      '換一個更聰明的搜尋演算法',
      '允許 $\\theta_2$ 轉超過 ±180°',
      '把其中一個障礙移出連桿 1 掃過的範圍',
    ],
    answer: 3,
    explanation: 'S 與 G 落在 $\\mathcal C_{\\rm free}$ 的不同連通分量，再細的格子、再好的演算法都跨不過去。$\\theta_2$ 本來就能整圈轉（環面），也無濟於事，因為擋路的是連桿 1 造成的垂直帶，與 $\\theta_2$ 無關。只有改變問題本身，例如移開障礙，兩個分量才會連起來。',
  },
  {
    question: '為什麼多數規劃器只呼叫碰撞查詢，而不先把 C-obstacle 完整算出來？',
    options: [
      '完整建構要涵蓋整個 C-space，查詢次數隨維度指數成長；碰撞查詢只在真正用到的組態上做',
      'C-obstacle 只在 2 維時才有定義',
      '碰撞查詢一定比 C-obstacle 精確',
      'C-obstacle 會隨時間改變，無法事先計算',
    ],
    answer: 0,
    explanation: '以每軸 100 格計，2 維 C-space 要 $10^4$ 次查詢，6 維卻是 $10^{12}$ 次。C-obstacle 在任何維度都有定義，只是算不完；規劃器只檢查搜尋過程中真正碰到的組態或候選路徑，成本取決於搜尋本身，而不是整個 C-space 的體積。',
  },
];

// ---------------------------------------------------------------------------

const PI = Math.PI;
const TAU = 2 * Math.PI;
const D2R = PI / 180;
/** Width (rad) of the periodic copy shown around the fundamental square. */
const MARGIN = 0.6;
const MAX_OBS = 4;
const SWEEP_SECONDS = 3.6;
/** Playback speed along a joint-space path (rad/s). */
const PLAY_SPEED = 1.4;
/** A waypoint closer than this (torus distance, rad) to G closes the drawn path. */
const SNAP_G = 0.25;
const FREEHAND_STEP = 0.12;
const PREVIEW_CELLS = 16;

type PathMode = 'none' | 'straight' | 'drawn';
interface StepCfg { scen: number; cobs: boolean; cover: boolean; comps: boolean; nearest: boolean; path: PathMode; sweep?: boolean }
const STEP_CFG: StepCfg[] = [
  { scen: 0, cobs: false, cover: false, comps: false, nearest: false, path: 'straight' },
  { scen: 0, cobs: true, cover: false, comps: false, nearest: false, path: 'straight', sweep: true },
  { scen: 1, cobs: true, cover: false, comps: false, nearest: false, path: 'straight' },
  { scen: 2, cobs: true, cover: true, comps: false, nearest: true, path: 'none' },
  { scen: 3, cobs: true, cover: false, comps: true, nearest: false, path: 'straight' },
  { scen: 0, cobs: true, cover: false, comps: false, nearest: false, path: 'drawn' },
];

const degStr = (rad: number): string => `${fmt(rad / D2R, 0)}°`;
const qStr = (q: P2): string => `(${degStr(q[0])}, ${degStr(q[1])})`;
const clampN = (x: number, lo: number, hi: number): number => Math.max(lo, Math.min(hi, x));

/** Resolve a theme colour variable into RGBA bytes for the raster layer. */
function rgba(name: string, alpha: number, fallback: [number, number, number]): RGBA {
  const s = cssVar(name);
  let rgb: [number, number, number] = fallback;
  const m6 = /^#([0-9a-f]{6})$/i.exec(s);
  const m3 = /^#([0-9a-f]{3})$/i.exec(s);
  const mf = /^rgba?\(([^)]+)\)/i.exec(s);
  if (m6) { const v = parseInt(m6[1], 16); rgb = [(v >> 16) & 255, (v >> 8) & 255, v & 255]; }
  else if (m3) { rgb = [0, 1, 2].map((i) => parseInt(m3[1][i] + m3[1][i], 16)) as [number, number, number]; }
  else if (mf) { const p = mf[1].split(',').map((x) => parseFloat(x)); if (p.length >= 3 && p.every((x) => Number.isFinite(x))) rgb = [p[0], p[1], p[2]]; }
  return [rgb[0], rgb[1], rgb[2], Math.round(alpha * 255)];
}

function tickLabel(v: number): string {
  const w = wrapToPi(v);
  if (Math.abs(Math.abs(w) - PI) < 1e-6) return '±180°';
  return `${fmt(w / D2R, 0)}°`;
}

function mount(host: HTMLElement, ctx: LessonContext): LessonHandle {
  // ---------------- state
  let L2 = 1;
  const Ls = (): [number, number] => [1, L2];
  let obstacles: Obstacle[] = [];
  let S: P2 = [0, 0];
  let G: P2 = [0, 0];
  let q: P2 = [0, 0];
  let path: P2[] = [];
  let pathMode: PathMode = 'straight';
  let pathDone = true;
  let drawMode = false;
  let hover: P2 | null = null;
  let res = 128;
  let K = 5;
  let showCobs = true;
  let showCover = false;
  let showNearest = false;
  let seam0 = false;
  let showComps = false;
  let showPreview = false;
  let sweepCol: number | null = null;
  /** Column where the current sweep started (the sweep goes once around the torus from there). */
  let sweepStart = 0;
  let playU: number | null = null;
  let playDur = 1;
  let codes: Uint8Array = new Uint8Array(res * res);
  let coverCodes: Uint8Array | null = null;
  let comp: Components = torusComponents(codes, res);
  let fracExact = 0;
  let fracCover = 0;
  let chk: PathCheck | null = null;
  let tipTrace: Pt[] = [];
  let curBits = 0;
  let curEx: ClearanceResult | null = null;
  let curCv: ClearanceResult | null = null;
  let raster: RasterLayer | null = null;
  let rollCache: Uint8Array | null = null;
  let destroyed = false;

  const mode = (): CollisionMode => (showCover ? 'cover' : 'exact');
  const blocked = (): Uint8Array => (showCover && coverCodes ? coverCodes : codes);
  const cen = (): P2 => (seam0 ? [PI, PI] : [0, 0]);
  const disp = (a: P2): Pt => { const c = cen(); return [wrapToPi(a[0] - c[0]) + c[0], wrapToPi(a[1] - c[1]) + c[1]]; };
  const squareWorld = (): World => { const c = cen(); return { xmin: c[0] - PI, xmax: c[0] + PI, ymin: c[1] - PI, ymax: c[1] + PI }; };
  const csWorld = (): World => { const w = squareWorld(); return { xmin: w.xmin - MARGIN, xmax: w.xmax + MARGIN, ymin: w.ymin - MARGIN, ymax: w.ymax + MARGIN }; };

  // ---------------- layout
  simTitle(host, '2R 手臂：工作空間 ↔ C-space', 'C-obstacle');
  const chips = presetChips(host, SCENARIOS.map((s, i) => ({ label: s.label, apply: () => loadScenario(i) })), { label: '場景' });
  const views = el('div', { class: 'mr-split' });
  host.appendChild(views);
  const left = el('div');
  const right = el('div');
  views.append(left, right);
  const capStyle = { margin: '0 0 0.25rem' };
  left.appendChild(el('div', { class: 'mr-note', style: capStyle }, '工作空間：拖曳障礙（內部移動、邊緣縮放）、手肘或末端'));
  right.appendChild(el('div', { class: 'mr-note', style: capStyle }, 'C-space：橫軸 θ₁、縱軸 θ₂，黑點 q 為目前組態'));
  const ws = new Canvas2D(left, { aspect: 1, world: { xmin: -2.45, xmax: 2.45, ymin: -2.45, ymax: 2.45 }, overlay: true });
  const cs = new Canvas2D(right, { aspect: 1, world: csWorld(), margin: 0.02 });
  legend(host, [
    { color: 'var(--series-2)', label: '連桿 1 碰撞（整條垂直帶）' },
    { color: 'var(--series-4)', label: '只有連桿 2 碰撞' },
    { color: 'var(--series-5)', label: '圓覆蓋多出的保守區' },
    { color: 'var(--series-3)', label: '起點 S' },
    { color: 'var(--series-1)', label: '目標 G' },
  ]);

  const panel = controlPanel(host, { compact: true });
  const r1 = row(panel);
  const sweepBtn = button(r1, { label: '▶ 掃描', title: '逐欄掃描 (θ₁, θ₂)，把碰撞的組態塗色', onClick: () => startSweep() });
  const playBtn = button(r1, { label: '▶ 播放路徑', onClick: () => (playU === null ? startPlay() : stopPlay()) });
  const drawBtn = button(r1, { label: '✎ 畫路徑', onClick: () => (drawMode ? finishDrawing() : startDrawing()) });
  button(r1, { label: '⌫ 清除路徑', small: true, onClick: () => clearPath() });
  const addBtn = button(r1, { label: '＋ 障礙', small: true, onClick: () => addObstacle() });
  const delBtn = button(r1, { label: '－ 障礙', small: true, onClick: () => removeObstacle() });
  const r2 = row(panel);
  const tCobs = toggle(r2, { label: 'C-obstacle', value: showCobs, onChange: (v) => { showCobs = v; if (!v) stopSweep(); repaint(); } });
  const tCover = toggle(r2, { label: '圓覆蓋', value: showCover, onChange: (v) => { showCover = v; sK.disabled = !v; rebuild(); } });
  const tNear = toggle(r2, { label: '最短距離', value: showNearest, onChange: (v) => { showNearest = v; ws.requestRender(); } });
  const tSeam = toggle(r2, { label: '接縫移到 0°', value: seam0, onChange: (v) => { seam0 = v; rollCache = null; cs.setWorld(csWorld()); repaint(); } });
  const tComps = toggle(r2, { label: '連通分量', value: showComps, onChange: (v) => { showComps = v; repaint(); } });
  const tPrev = toggle(r2, { label: '格點預覽', value: showPreview, onChange: (v) => { showPreview = v; cs.requestRender(); } });
  const sL2 = slider(panel, { label: '第二桿長 L₂', min: 0.5, max: 1.3, step: 0.05, value: L2, unit: 'm', onInput: (v) => { L2 = v; chips.clear(); scheduleRebuild(); } });
  const sK = slider(panel, { label: '每桿覆蓋圓數 K', min: 2, max: 8, step: 1, value: K, onInput: (v) => { K = Math.round(v); scheduleRebuild(); } });
  sK.disabled = !showCover;
  select(panel, {
    label: '解析度（每軸格數）',
    options: [64, 128, 192].map((n) => ({ value: String(n), label: `${n} × ${n}` })),
    value: String(res),
    onChange: (v) => { stopSweep(); res = Number(v); rebuild(); },
  });
  const info = kv(host, ['組態 q', '狀態', 'd_min（精確）', 'd_min（圓覆蓋）', 'C-obstacle 佔比', '連通分量', '路徑檢查']);
  note(host, '連桿畫成實際粗細（半寬 0.06 m），碰撞以「膠囊對圓盤」精確計算；勾選「圓覆蓋」後，碰撞判定、路徑檢查與連通分量都改用覆蓋圓的圓心距離。C-space 方格外圍的淡色區是週期複本（θ 與 θ ± 360° 是同一個姿勢）。');

  // ---------------- colours (re-resolved on theme change)
  let pal: RGBA[] = [];
  const makePalette = (): void => {
    const none: RGBA = [0, 0, 0, 0];
    const link1 = rgba('--series-2', 0.85, [225, 29, 72]);
    pal = [
      none,
      link1,                                     // 1: link 1 only
      rgba('--series-4', 0.8, [217, 119, 6]),    // 2: link 2 only
      link1,                                     // 3: both links
      rgba('--series-5', 0.5, [124, 58, 237]),   // 4: extra cells of the circle cover
      rgba('--series-1', 0.18, [37, 99, 235]),   // 5..8: free components
      rgba('--series-3', 0.22, [22, 163, 74]),
      rgba('--series-6', 0.22, [8, 145, 178]),
      rgba('--series-7', 0.2, [219, 39, 119]),
      rgba('--ok', 0.13, [21, 128, 61]),          // 9: already scanned and free (sweep only)
    ];
  };
  makePalette();

  // ---------------- derived data
  function rebuild(): void {
    if (destroyed) return;
    const L = Ls();
    codes = computeCobsCodes(res, 'exact', L, W, obstacles);
    coverCodes = showCover ? computeCobsCodes(res, 'cover', L, W, obstacles, K) : null;
    comp = torusComponents(blocked(), res);
    fracExact = blockedFraction(codes);
    fracCover = coverCodes ? blockedFraction(coverCodes) : 0;
    rollCache = null;
    recheckPath();
    updateQ();
    repaint();
  }
  const scheduleRebuild = rafThrottle(rebuild);

  /** Blocked cells re-indexed in display order (rows/columns start at the left/bottom edge of the square). */
  function rolledBlocked(): Uint8Array {
    if (rollCache) return rollCache;
    const n = res;
    const src = blocked();
    const s = seam0 ? n / 2 : 0;
    const out = new Uint8Array(n * n);
    for (let jd = 0; jd < n; jd++) {
      const j = (jd + s) % n;
      for (let id = 0; id < n; id++) out[jd * n + id] = src[j * n + ((id + s) % n)];
    }
    rollCache = out;
    return out;
  }

  function paintRaster(): void {
    if (!raster || raster.nx !== res) raster = new RasterLayer(squareWorld(), res, res);
    raster.world = squareWorld();
    const n = res;
    const s = seam0 ? n / 2 : 0;
    const x0 = raster.world.xmin;
    const y0 = raster.world.ymin;
    const lim = sweepCol === null ? n : Math.floor(sweepCol);
    const cov = showCover ? coverCodes : null;
    const lab = showComps ? comp.labels : null;
    const cds = codes;
    raster.paint((x, y) => {
      const id = clampN(Math.floor(((x - x0) / TAU) * n), 0, n - 1);
      const jd = clampN(Math.floor(((y - y0) / TAU) * n), 0, n - 1);
      const i = (id + s) % n;
      const j = (jd + s) % n;
      if ((i - sweepStart + n) % n >= lim) return NaN;
      const k = j * n + i;
      if (cds[k]) return cds[k];
      if (cov && cov[k]) return 4;
      if (lab && lab[k] >= 0) return 5 + (lab[k] % 4);
      return lim < n ? 9 : NaN;
    }, { colormap: (v) => pal[v] ?? pal[0], range: [0, 1] });
  }

  function repaint(): void {
    if (destroyed) return;
    paintRaster();
    cs.requestRender();
    ws.requestRender();
  }

  function recheckPath(): void {
    chk = path.length ? checkPath(path, mode(), Ls(), W, obstacles, 0.02, K) : null;
    const L = Ls();
    tipTrace = chk ? chk.samples.map((sm) => armPoints(L, sm.q)[2] as Pt) : [];
  }

  function updateQ(): void {
    if (destroyed) return;
    const L = Ls();
    curEx = clearance('exact', L, W, q, obstacles);
    curCv = clearance('cover', L, W, q, obstacles, K);
    curBits = linkCollisions(mode(), L, W, q, obstacles, K);
    hQ.color = curBits ? '--danger' : '--fg';
    readouts();
    ws.requestRender();
    cs.requestRender();
  }

  const fmtD = (d: number | undefined): string => (d === undefined || !Number.isFinite(d) ? '∞（沒有障礙）' : `${fmt(d, 3)} m`);

  function pathSummary(): string {
    if (!chk || path.length < 2) return drawMode ? '繪製中：在 C-space 點選路徑點' : '—（目前沒有路徑）';
    const tail = pathMode === 'drawn' && !pathDone ? '，尚未到達 G' : '';
    if (chk.hits === 0) return `✓ 無碰撞（${path.length - 1} 段，長 ${fmt(chk.length, 2)} rad${tail}）`;
    return `✗ ${chk.hits} / ${chk.samples.length} 個取樣碰撞（${chk.badSegments} 段有問題${tail}）`;
  }

  function readouts(): void {
    info.set('組態 q', qStr(q));
    info.set('狀態', curBits === 0 ? '自由（在 C-free 中）' : `碰撞：${curBits === (HIT_LINK1 | HIT_LINK2) ? '兩根連桿' : curBits === HIT_LINK1 ? '連桿 1' : '連桿 2'}`);
    info.set('d_min（精確）', fmtD(curEx?.d));
    info.set('d_min（圓覆蓋）', showCover ? `${fmtD(curCv?.d)}（K = ${K}）` : '—（勾選「圓覆蓋」）');
    info.set('C-obstacle 佔比', `${fmt(100 * fracExact, 1)} %${showCover ? `；圓覆蓋 ${fmt(100 * fracCover, 1)} %` : ''}`);
    const cS = componentAt(comp, res, S);
    const cG = componentAt(comp, res, G);
    info.set('連通分量', `${comp.count}${cS < 0 || cG < 0 ? '（S 或 G 在 C-obstacle 內）' : cS === cG ? '（S、G 在同一分量）' : '（S、G 不在同一分量 → 無解）'}`);
    info.set('路徑檢查', pathSummary());
  }

  // ---------------- workspace drawing
  function drawArm(c: Canvas2D, qq: P2, col1: string, col2: string, alpha: number): Pt[] {
    const pts = armPoints(Ls(), qq) as Pt[];
    const wpx = Math.max(3, 2 * W * c.scale);
    c.line(pts[0], pts[1], { color: col1, width: wpx, alpha });
    c.line(pts[1], pts[2], { color: col2, width: wpx, alpha });
    c.point(pts[0], wpx * 0.3, { color: cssVar('--bg-elev'), alpha });
    c.point(pts[1], wpx * 0.3, { color: cssVar('--bg-elev'), alpha });
    c.point(pts[2], 3, { color: cssVar('--fg'), alpha });
    return pts;
  }

  ws.draw = (c) => {
    const L = Ls();
    const danger = cssVar('--danger');
    const muted = cssVar('--fg-muted');
    const accent = cssVar('--accent');
    const gs = cssVar('--grid-strong');
    c.grid(0.5);
    c.circle([0, 0], L[0] + L[1], { color: gs, dash: [4, 4], width: 1 });
    obstacles.forEach((o) => {
      const hit = linkCollisions(mode(), L, W, q, [o], K) !== 0;
      c.disk(o.c, o.r, { fill: muted, alpha: 0.42 });
      c.circle(o.c, o.r, { color: hit ? danger : muted, width: hit ? 2.5 : 1.5 });
      c.point([o.c[0] + o.r * Math.SQRT1_2, o.c[1] + o.r * Math.SQRT1_2], 3, { color: muted });
    });
    c.polygon([[-0.13, -0.13], [0.13, -0.13], [0, 0]], { fill: gs, color: gs, width: 1 });
    const colS = cssVar('--series-3');
    const colG = cssVar('--series-1');
    const pS = drawArm(c, S, colS, colS, 0.32);
    const pG = drawArm(c, G, colG, colG, 0.32);
    c.text(pS[2], 'S', { color: colS, dx: 10, dy: -10, font: `bold 13px ${cssVar('--font') || 'sans-serif'}` });
    c.text(pG[2], 'G', { color: colG, dx: 10, dy: -10, font: `bold 13px ${cssVar('--font') || 'sans-serif'}` });
    if (chk && tipTrace.length > 1) {
      c.polyline(tipTrace, { color: cssVar('--warn'), width: 1.6, dash: [5, 4] });
      let shown = 0;
      let lastSeg = -1;
      for (const sm of chk.samples) {
        if (sm.hit && sm.seg !== lastSeg && shown < 3) { drawArm(c, sm.q, danger, danger, 0.28); lastSeg = sm.seg; shown++; }
      }
    }
    if (sweepCol !== null) {
      // every forearm of the column being scanned: red = this (θ₁, θ₂) is in C_obs
      const i = (sweepStart + Math.min(res - 1, Math.floor(sweepCol))) % res;
      const th1 = cellAngle(i, res);
      const e = armPoints(L, [th1, 0])[1] as Pt;
      for (let k = 0; k < 36; k++) {
        const th2 = -PI + ((k + 0.5) / 36) * TAU;
        const bits = linkCollisions(mode(), L, W, [th1, th2], obstacles, K);
        const tip = armPoints(L, [th1, th2])[2] as Pt;
        c.line(e, tip, { color: bits ? danger : cssVar('--ok'), width: 2, alpha: 0.6 });
      }
      const h1 = linkCollisions(mode(), L, W, [th1, 0], obstacles, K) & HIT_LINK1;
      c.line([0, 0], e, { color: h1 ? danger : muted, width: Math.max(3, 2 * W * c.scale), alpha: 0.85 });
    }
    if (showCover) for (const ci of circleCover(L, W, q, K)) c.circle(ci.c, ci.r, { color: cssVar('--series-5'), width: 1.2 });
    drawArm(c, q, curBits & HIT_LINK1 ? danger : muted, curBits & HIT_LINK2 ? danger : muted, sweepCol !== null ? 0.25 : 1);
    if (showNearest) {
      const r = showCover ? curCv : curEx;
      if (r && Number.isFinite(r.d) && r.d > 0) {
        c.line(r.pArm, r.pObs, { color: accent, width: 2, dash: [4, 3] });
        c.point(r.pArm, 3.5, { color: accent });
        c.point(r.pObs, 3.5, { color: accent });
      }
    }
    const dNow = showCover ? curCv?.d : curEx?.d;
    c.setOverlayText(`θ = ${qStr(q)}${showNearest && dNow !== undefined && Number.isFinite(dNow) ? `  d_min = ${fmt(dNow, 3)} m` : ''}`);
  };

  // ---------------- C-space drawing
  const rect = (x0: number, y0: number, x1: number, y1: number): Pt[] => [[x0, y0], [x1, y0], [x1, y1], [x0, y1]];

  cs.draw = (c) => {
    const sq = squareWorld();
    const c0 = cen();
    const n = res;
    const muted = cssVar('--fg-muted');
    const accent = cssVar('--accent');
    const danger = cssVar('--danger');
    const font10 = `10px ${cssVar('--font') || 'sans-serif'}`;
    if (showCobs && raster) {
      for (let a = -1; a <= 1; a++) {
        for (let b = -1; b <= 1; b++) {
          c.image(raster.canvas, { xmin: sq.xmin + a * TAU, xmax: sq.xmax + a * TAU, ymin: sq.ymin + b * TAU, ymax: sq.ymax + b * TAU }, { alpha: a || b ? 0.5 : 1 });
        }
      }
    }
    // fade the periodic copies so the fundamental square stands out
    const v = c.visible();
    const wash = { fill: cssVar('--bg-sunken'), color: 'transparent', width: 0, alpha: 0.6 };
    c.polygon(rect(v.xmin, v.ymin, v.xmax, sq.ymin), wash);
    c.polygon(rect(v.xmin, sq.ymax, v.xmax, v.ymax), wash);
    c.polygon(rect(v.xmin, sq.ymin, sq.xmin, sq.ymax), wash);
    c.polygon(rect(sq.xmax, sq.ymin, v.xmax, sq.ymax), wash);
    // 90° grid lines
    const gl = cssVar('--grid-strong');
    for (let k = 1; k < 4; k++) {
      c.line([sq.xmin + (k * PI) / 2, v.ymin], [sq.xmin + (k * PI) / 2, v.ymax], { color: gl, width: 1, alpha: 0.7 });
      c.line([v.xmin, sq.ymin + (k * PI) / 2], [v.xmax, sq.ymin + (k * PI) / 2], { color: gl, width: 1, alpha: 0.7 });
    }
    if (showPreview) {
      const blk = rolledBlocked();
      const m = n / PREVIEW_CELLS;
      const h = TAU / PREVIEW_CELLS;
      for (let b = 0; b < PREVIEW_CELLS; b++) {
        for (let a = 0; a < PREVIEW_CELLS; a++) {
          let any = false;
          for (let jj = b * m; jj < (b + 1) * m && !any; jj++) for (let ii = a * m; ii < (a + 1) * m; ii++) if (blk[jj * n + ii]) { any = true; break; }
          if (any) c.polygon(rect(sq.xmin + a * h, sq.ymin + b * h, sq.xmin + (a + 1) * h, sq.ymin + (b + 1) * h), { fill: cssVar('--fg'), color: 'transparent', width: 0, alpha: 0.18 });
        }
      }
      for (let k = 0; k <= PREVIEW_CELLS; k++) {
        c.line([sq.xmin + k * h, sq.ymin], [sq.xmin + k * h, sq.ymax], { color: accent, width: 1, alpha: 0.45 });
        c.line([sq.xmin, sq.ymin + k * h], [sq.xmax, sq.ymin + k * h], { color: accent, width: 1, alpha: 0.45 });
      }
    }
    c.polygon(rect(sq.xmin, sq.ymin, sq.xmax, sq.ymax), { color: muted, width: 1.5 });
    // seam marks: blocked cells touching an edge of the square continue on the opposite edge
    if (showCobs && sweepCol === null) {
      const blk = rolledBlocked();
      const h = TAU / n;
      const c1 = cssVar('--series-6');
      const c2 = cssVar('--series-7');
      const o = 4 / c.scale;
      for (const [k0, k1] of seamRuns(blk, n, 0, 0)) {
        const ya = sq.ymin + k0 * h;
        const yb = sq.ymin + (k1 + 1) * h;
        c.line([sq.xmin - o, ya], [sq.xmin - o, yb], { color: c1, width: 5, cap: 'butt' });
        c.line([sq.xmax + o, ya], [sq.xmax + o, yb], { color: c1, width: 5, cap: 'butt' });
      }
      for (const [k0, k1] of seamRuns(blk, n, 1, 0)) {
        const xa = sq.xmin + k0 * h;
        const xb = sq.xmin + (k1 + 1) * h;
        c.line([xa, sq.ymin - o], [xb, sq.ymin - o], { color: c2, width: 5, cap: 'butt' });
        c.line([xa, sq.ymax + o], [xb, sq.ymax + o], { color: c2, width: 5, cap: 'butt' });
      }
    }
    if (!showCobs) {
      c.text([c0[0], sq.ymax - 0.45], 'C-obstacle 尚未畫出', { color: muted, font: `13px ${cssVar('--font') || 'sans-serif'}`, bg: cssVar('--bg-elev') });
    } else if (drawMode && path.length <= 1) {
      c.text([c0[0], sq.ymax - 0.45], '從 S 開始點選路徑點，靠近 G 會自動接上', { color: accent, font: `12px ${cssVar('--font') || 'sans-serif'}`, bg: cssVar('--bg-elev') });
    }
    // joint-space path (shortest torus segments, with faded periodic copies)
    const seg = (a: P2, b: P2, color: string, width: number, dash?: number[]): void => {
      for (const [p0, p1] of torusSegmentPieces([a[0] - c0[0], a[1] - c0[1]], [b[0] - c0[0], b[1] - c0[1]])) {
        for (let ox = -1; ox <= 1; ox++) {
          for (let oy = -1; oy <= 1; oy++) {
            const dx = c0[0] + ox * TAU;
            const dy = c0[1] + oy * TAU;
            c.line([p0[0] + dx, p0[1] + dy], [p1[0] + dx, p1[1] + dy], { color, width, dash, alpha: ox || oy ? 0.35 : 1 });
          }
        }
      }
    };
    for (let k = 0; k + 1 < path.length; k++) seg(path[k], path[k + 1], accent, 2.5);
    if (chk) for (const sm of chk.samples) if (sm.hit) c.point(disp(sm.q), 2.6, { color: danger });
    if (pathMode === 'drawn') for (let k = 1; k < path.length - (pathDone ? 1 : 0); k++) c.point(disp(path[k]), 3.5, { color: accent });
    if (drawMode && hover && path.length) {
      const last = path[path.length - 1];
      const bad = checkPath([last, hover], mode(), Ls(), W, obstacles, 0.03, K).hits > 0;
      seg(last, hover, bad ? danger : accent, 1.6, [5, 4]);
    }
    if (drawMode) {
      const marker = (p: Pt, color: string, label: string): void => {
        c.point(p, 8, { color: cssVar('--bg-elev') });
        c.circle(p, 8 / c.scale, { color, width: 2 });
        c.point(p, 3.2, { color });
        if (label) c.text(p, label, { dy: -17, color });
      };
      marker(disp(S), cssVar('--series-3'), 'S');
      marker(disp(G), cssVar('--series-1'), 'G');
      c.point(disp(q), 4, { color: curBits ? danger : cssVar('--fg') });
    }
    if (sweepCol !== null) {
      const x = disp([cellAngle((sweepStart + Math.min(n - 1, Math.floor(sweepCol))) % n, n), 0])[0];
      c.line([x, sq.ymin], [x, sq.ymax], { color: accent, width: 2 });
    }
    // ticks
    for (let k = 0; k <= 4; k++) {
      const x = sq.xmin + (k * PI) / 2;
      c.text([x, sq.ymin], tickLabel(x), { dy: 10, font: font10, color: muted });
    }
    for (let k = 1; k <= 3; k++) {
      const y = sq.ymin + (k * PI) / 2;
      c.text([sq.xmin, y], tickLabel(y), { dx: -4, align: 'right', font: font10, color: muted });
    }
  };

  // ---------------- handles
  const hS = cs.addHandle({ get: () => disp(S), set: (p) => { S = wrapQ(p as P2); onSGChanged(); }, color: '--series-3', label: 'S', radius: 8, onStart: () => stopPlay() });
  const hG = cs.addHandle({ get: () => disp(G), set: (p) => { G = wrapQ(p as P2); onSGChanged(); }, color: '--series-1', label: 'G', radius: 8, onStart: () => stopPlay() });
  const hQ = cs.addHandle({ get: () => disp(q), set: (p) => { q = wrapQ(p as P2); updateQ(); }, color: '--fg', radius: 6, onStart: () => stopPlay() });
  ws.addHandle({
    get: () => armPoints(Ls(), q)[1] as Pt,
    set: (p) => { q = [Math.atan2(p[1], p[0]), q[1]]; updateQ(); },
    color: '--accent', radius: 7, onStart: () => stopPlay(),
  });
  ws.addHandle({
    get: () => armPoints(Ls(), q)[2] as Pt,
    set: (p) => { const e = armPoints(Ls(), q)[1]; q = [q[0], wrapToPi(Math.atan2(p[1] - e[1], p[0] - e[0]) - q[0])]; updateQ(); },
    color: '--accent', radius: 7, onStart: () => stopPlay(),
  });
  function syncObsHandles(): void {
    addBtn.disabled = obstacles.length >= MAX_OBS;
    delBtn.disabled = obstacles.length === 0;
  }

  // ---------------- workspace pointer: drag an obstacle's interior to move it, its rim to resize it
  let obsDrag: { k: number; resize: boolean; off: Pt } | null = null;
  const pickObstacle = (p: Pt): { k: number; resize: boolean } | null => {
    const tol = 7 / ws.scale;
    for (let k = obstacles.length - 1; k >= 0; k--) {
      const o = obstacles[k];
      const d = Math.hypot(p[0] - o.c[0], p[1] - o.c[1]);
      if (Math.abs(d - o.r) <= tol) return { k, resize: true };
      if (d < o.r) return { k, resize: false };
    }
    return null;
  };
  const nearArmHandle = (p: Pt): boolean => {
    const pts = armPoints(Ls(), q);
    return [pts[1], pts[2]].some((h) => Math.hypot(h[0] - p[0], h[1] - p[1]) * ws.scale < 14);
  };
  const offWsPointer = ws.onPointer({
    down: (p, ev) => {
      const hit = pickObstacle(p);
      if (!hit) return;
      stopPlay();
      const o = obstacles[hit.k];
      obsDrag = { ...hit, off: [p[0] - o.c[0], p[1] - o.c[1]] };
      ev.preventDefault(); // dragging an obstacle must not start a text selection on the page
      try { ws.canvas.setPointerCapture(ev.pointerId); } catch { /* synthetic events have no capture target */ }
    },
    move: (p, ev) => {
      if (obsDrag && (ev.buttons & 1)) {
        const o = obstacles[obsDrag.k];
        if (!o) { obsDrag = null; return; }
        if (obsDrag.resize) o.r = clampN(Math.hypot(p[0] - o.c[0], p[1] - o.c[1]), 0.08, 0.7);
        else o.c = [clampN(p[0] - obsDrag.off[0], -2.3, 2.3), clampN(p[1] - obsDrag.off[1], -2.3, 2.3)];
        chips.clear();
        scheduleRebuild();
        return;
      }
      if (nearArmHandle(p)) return; // the canvas shows its own cursor over handles
      const hit = pickObstacle(p);
      ws.canvas.style.cursor = hit ? (hit.resize ? 'nwse-resize' : 'move') : '';
    },
    up: () => { obsDrag = null; },
  });

  // ---------------- C-space pointer: click / drag to set q, or draw a path
  let dragQ = false;
  let drawing = false;
  const offPointer = cs.onPointer({
    down: (p, ev) => {
      const qq = wrapQ(p as P2);
      ev.preventDefault(); // no text selection while dragging q or drawing a path
      try { cs.canvas.setPointerCapture(ev.pointerId); } catch { /* synthetic events have no capture target */ }
      if (drawMode) { addWaypoint(qq); drawing = drawMode; return; }
      stopPlay();
      q = qq;
      dragQ = true;
      updateQ();
    },
    move: (p, ev) => {
      const qq = wrapQ(p as P2);
      if (drawMode) {
        hover = qq;
        if (drawing && (ev.buttons & 1)) {
          const last = path[path.length - 1];
          if (!last || torusDist(last, qq) >= FREEHAND_STEP) addWaypoint(qq);
        } else {
          q = qq; // preview the arm at the hovered configuration
          updateQ();
        }
        cs.requestRender();
        return;
      }
      if (dragQ && (ev.buttons & 1)) { q = qq; updateQ(); }
    },
    up: () => { dragQ = false; drawing = false; },
  });

  // ---------------- path editing
  function setDrawMode(v: boolean): void {
    drawMode = v;
    drawBtn.textContent = v ? '✓ 結束繪製' : '✎ 畫路徑';
    hS.enabled = !v;
    hG.enabled = !v;
    hQ.enabled = !v;
    cs.canvas.style.cursor = v ? 'crosshair' : '';
    if (!v) hover = null;
  }
  function startDrawing(): void {
    stopPlay();
    stopSweep();
    pathMode = 'drawn';
    path = [S];
    pathDone = false;
    q = S;
    setDrawMode(true);
    recheckPath();
    updateQ();
  }
  function finishDrawing(): void {
    setDrawMode(false);
    readouts();
    cs.requestRender();
  }
  function addWaypoint(qq: P2): void {
    if (!drawMode) return;
    if (path.length === 0) path = [S];
    if (torusDist(qq, G) < SNAP_G) {
      path.push(G);
      pathDone = true;
      setDrawMode(false);
    } else {
      path.push(qq);
    }
    q = path[path.length - 1];
    recheckPath();
    updateQ();
  }
  function clearPath(): void {
    stopPlay();
    setDrawMode(false);
    pathMode = 'none';
    path = [];
    pathDone = true;
    recheckPath();
    updateQ();
  }
  function onSGChanged(): void {
    if (pathMode === 'straight') path = [S, G];
    else if (pathMode === 'drawn' && path.length) {
      path[0] = S;
      if (pathDone && path.length > 1) path[path.length - 1] = G;
    }
    recheckPath();
    readouts();
    cs.requestRender();
    ws.requestRender();
  }

  // ---------------- obstacles
  function addObstacle(): void {
    if (obstacles.length >= MAX_OBS) return;
    const spots: P2[] = [[-0.9, -1.2], [1.4, -1.0], [-1.4, 1.2], [0.3, 1.7], [-1.9, -0.2]];
    const c = spots.find((p) => obstacles.every((o) => Math.hypot(o.c[0] - p[0], o.c[1] - p[1]) > o.r + 0.45)) ?? spots[0];
    obstacles.push({ c: [c[0], c[1]], r: 0.2 });
    chips.clear();
    syncObsHandles();
    rebuild();
  }
  function removeObstacle(): void {
    if (!obstacles.length) return;
    obstacles.pop();
    chips.clear();
    syncObsHandles();
    rebuild();
  }

  // ---------------- animation: C-obstacle sweep and path playback
  const loop = createLoop((dt) => {
    if (destroyed) return;
    let busy = false;
    if (sweepCol !== null) {
      sweepCol += (dt / SWEEP_SECONDS) * res;
      if (sweepCol >= res) { sweepCol = null; sweepBtn.disabled = false; }
      paintRaster();
      busy = true;
    }
    if (playU !== null) {
      playU = Math.min(1, playU + dt / playDur);
      q = pathAt(path, playU);
      if (playU >= 1) { playU = null; playBtn.textContent = '▶ 播放路徑'; }
      updateQ();
      busy = true;
    }
    ws.requestRender();
    cs.requestRender();
    if (!busy) loop.stop();
  });
  function startSweep(): void {
    stopPlay();
    if (!showCobs) { showCobs = true; tCobs.set(true, { silent: true }); }
    sweepCol = 0;
    sweepStart = angleCell(q[0], res);
    sweepBtn.disabled = true;
    paintRaster();
    loop.start();
  }
  function stopSweep(): void {
    if (sweepCol === null) return;
    sweepCol = null;
    sweepBtn.disabled = false;
    paintRaster();
    ws.requestRender();
    cs.requestRender();
  }
  function startPlay(): void {
    if (drawMode) setDrawMode(false);
    if (path.length < 2) { info.set('路徑檢查', '—（先畫一條路徑或選一個場景）'); return; }
    stopSweep();
    playDur = Math.max(1.5, (chk?.length ?? 0) / PLAY_SPEED);
    playU = 0;
    q = path[0];
    playBtn.textContent = '⏸ 停止';
    loop.start();
  }
  function stopPlay(): void {
    if (playU === null) return;
    playU = null;
    playBtn.textContent = '▶ 播放路徑';
  }

  // ---------------- scenarios and steps
  function loadScenario(i: number, doRebuild = true): void {
    const s = SCENARIOS[i];
    stopPlay();
    stopSweep();
    setDrawMode(false);
    obstacles = s.obstacles.map((o) => ({ c: [o.c[0], o.c[1]] as P2, r: o.r }));
    S = [s.S[0], s.S[1]];
    G = [s.G[0], s.G[1]];
    q = [s.q[0], s.q[1]];
    L2 = 1;
    sL2.set(1, { silent: true });
    pathMode = 'straight';
    path = [S, G];
    pathDone = true;
    syncObsHandles();
    if (doRebuild) rebuild();
  }

  const setT = (t: ToggleHandle, v: boolean): void => t.set(v, { silent: true });
  const focusMap: HTMLElement[][] = [
    [ws.el, playBtn],
    [cs.el, sweepBtn],
    [cs.el, tSeam.el],
    [tCover.el, sK.el, info.el],
    [cs.el, tComps.el],
    [cs.el, drawBtn],
  ];
  let focused: HTMLElement[] = [];
  function applyStep(i: number): void {
    const k = clampN(i, 0, STEP_CFG.length - 1);
    const cfg = STEP_CFG[k];
    chips.highlight(cfg.scen);
    loadScenario(cfg.scen, false);
    showCobs = cfg.cobs; setT(tCobs, showCobs);
    showCover = cfg.cover; setT(tCover, showCover); sK.disabled = !showCover;
    showComps = cfg.comps; setT(tComps, showComps);
    showNearest = cfg.nearest; setT(tNear, showNearest);
    seam0 = false; setT(tSeam, false); cs.setWorld(csWorld());
    showPreview = false; setT(tPrev, false);
    if (cfg.path === 'none') { pathMode = 'none'; path = []; }
    rebuild();
    if (cfg.path === 'drawn') startDrawing();
    if (cfg.sweep) startSweep();
    focused.forEach((f) => f.classList.remove('mr-focus'));
    focused = focusMap[k];
    focused.forEach((f) => f.classList.add('mr-focus'));
  }

  // handle colours are theme variables (resolved at draw time); only the raster palette needs re-resolving
  const offTheme = ctx.onThemeChange(() => { makePalette(); repaint(); });

  let lastStep = clampN(ctx.currentStep, 0, STEP_CFG.length - 1);
  let skipNext = true;
  applyStep(lastStep);

  return {
    onStep(i: number) {
      // the shell calls onStep once right after mount with the same index: skip that duplicate
      if (skipNext && i === lastStep) { skipNext = false; return; }
      skipNext = false;
      lastStep = i;
      applyStep(i);
    },
    destroy() {
      destroyed = true;
      loop.stop();
      offPointer();
      offWsPointer();
      offTheme();
      ws.destroy();
      cs.destroy();
    },
  };
}

const lesson: Lesson = { meta, steps, quiz, mount };
export default lesson;
