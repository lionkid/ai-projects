import type { Lesson, LessonContext, LessonHandle, LessonStep, QuizQuestion } from '@/types';
import {
  el, fmt, cssVar, themeColor, controlPanel, row, slider, toggle, button, select, kv, note, simTitle, legend, presetChips,
  rafThrottle, mdElement, Canvas2D, Plot2D, createLoop, type Pt, type Style, type ToggleHandle, type World,
} from '@/ui';
import { wrapToPi, type P2 } from '@/mr';
import { meta } from './meta';
import {
  DIRS, bestFirst, descend, downhill, downsample, edgesFrom, forNeighbors, heuristic, nearestFree, paintMaster, rrtPreview, wavefront,
  type Conn, type Grid, type HeuristicKind, type RRTPreview, type SearchTrace,
} from './grid';
import { ARM_2R, MAPS, MASTER, PROBE, arm2RHits, arm2RPoints, cellAngles } from './maps';
import { cellCount, fmtCount, fmtDuration, powLabel } from './scale';

const steps: LessonStep[] = [
  {
    title: '把空間切成格子',
    md: `把 C-space 的每一軸切成 $k$ 段，就得到一張**格點地圖**。格子裡只要有一部分落在 $\\mathcal C_{\\rm obs}$，整格就當成障礙（保守的離散化）；其餘每個自由格都是一個**節點**，相鄰的自由格之間連一條**邊**：

- **4 連通**：上下左右四個鄰居，每條邊代價 1；
- **8 連通**：再加上四個斜角，代價 $\\sqrt2$。斜向兩側只要有一格是障礙，就不准斜著「切角」擠過去。

運動規劃於是變成**圖搜尋**：找一條從 S 到 G、邊代價總和最小的路徑。2R 手臂的 C-space 是環面，所以最左欄與最右欄、最上列與最下列也互為鄰居。`,
    simHint: '拖曳紫色的「探針」到不同格子：箭頭是可以走的邊（標出代價），紅 × 是障礙，紅色虛線是被禁止的切角。切換「鄰居」4／8 連通，看地圖左上角的節點數與邊數怎麼變。',
  },
  {
    title: 'Dijkstra：按代價一圈圈擴散',
    md: `**Dijkstra** 演算法把等著處理的節點放在**開放串列**（open list，橘色）。每一輪取出累積代價 $g$ 最小的節點**展開**（移到已關閉集合，青色），再用它更新每個鄰居 $m$：

$$g(m)\\leftarrow\\min\\big(g(m),\\;g(n)+c(n,m)\\big)$$

因為永遠先處理最便宜的節點，一個節點被關閉時，它的 $g$ 已經是從 S 出發的最短距離；G 被取出的那一刻，最短路徑就確定了。代價是它完全不知道 G 在哪個方向，只能朝四面八方一圈圈擴散。`,
    simHint: '看搜尋自己跑：深淺相間的青色帶是一圈圈等代價的「波」，橘色是開放串列，紫色虛線是目前節點回溯到 S 的路。拖「回放」滑桿可以倒帶，滑鼠停在格子上可讀出它的 g。',
  },
  {
    title: 'A*：加上往目標的估計',
    md: `**A*** 改用 $f=g+h$ 排序，其中 $h(n)$ 是從 $n$ 到 G 還要花多少的**估計**。只要估計**可容許**（admissible），也就是永遠不超過真正的剩餘代價 $h^*(n)$，G 第一次被取出時的路徑仍然最短：

$$f(n)=g(n)+h(n),\\qquad 0\\le h(n)\\le h^*(n)$$

直線（歐氏）距離不可能比任何格點路徑長，所以可容許。$h$ 讓背對 G 的節點 $f$ 變大、一直排在後面，很多到結束都輪不到；Dijkstra 就是 $h\\equiv0$ 的特例。`,
    simHint: '地圖左上角同時列出 A* 與 Dijkstra 的展開數，跑完後再標出代價：與最短相同，展開的節點卻少得多；灰色虛線框出的是 Dijkstra 展開過的範圍。把 G 拖到別處，控制項下方的比較表會立刻更新。',
  },
  {
    title: '啟發式的權重 ε',
    md: `把估計乘上權重，改用 $f=g+\\varepsilon\\,h$：

- $\\varepsilon=0$ 就是 Dijkstra，$\\varepsilon=1$ 是標準 A*；
- $\\varepsilon>1$ 時 $\\varepsilon h$ 可能高估，搜尋更急著往 G 衝，展開的節點變少，但不再保證最短。若 $h$ 是一致的（滿足三角不等式），找到的代價最多是最短代價的 $\\varepsilon$ 倍。

U 形陷阱的開口正對著 S，估計會先把搜尋引進 U 裡。若重新規劃的速度比路徑短一點更重要（例如環境一直在變），用一點最佳性換速度很划算。`,
    simHint: '把 ε 從 0 拉到 3：灰色虛線是最短路徑，紫色是目前找到的路徑。地圖旁的「ε 掃描」曲線畫出展開節點數與路徑代價隨 ε 的變化（都以 Dijkstra 為 1），直線標出目前的 ε。',
  },
  {
    title: 'Wavefront：從終點倒推',
    md: `**Wavefront** 規劃器反過來從 G 出發，一圈圈往外替每個自由格標上到 G 的距離 $d(n)$（4 連通時就是廣度優先搜尋的步數）。距離場只算一次，之後**任何**起點都只要一直走向讓「邊代價加上鄰居的 $d$」最小的鄰居，就會沿最短路徑滑到 G：

$$n_{k+1}=\\arg\\min_{m\\in N(n_k)}\\big(c(n_k,m)+d(m)\\big)$$

這張是上一課 2R 手臂的 C-space。目標姿勢固定、出發姿勢一直在變的工作（多次查詢），最適合這種做法。`,
    simHint: '在 C-space 點任何一格當起點：路徑沿著箭頭（d 遞減的方向）走到 G，完全不用重新搜尋，旁邊的手臂也會照著動。注意波是穿過 ±180° 的接縫，從另一側繞過來的。',
  },
  {
    title: '格點的極限與下一課',
    md: `格點法是**解析度完備**（resolution complete）的：在選定的解析度下，有解一定找得到，沒解就回報失敗。但「沒解」也可能只是格子太粗：保守的離散化會把比一格還窄的通道整個塗成障礙。

把格子切細要付出代價。每軸 $k$ 格、$n$ 維的 C-space 共有

$$N=k^{\\,n}$$

個格子；6 軸手臂每軸 100 格就是 $10^{12}$ 格，就算每次碰撞查詢只要 1 μs，光是逐格查一遍就要將近 12 天，還沒開始搜尋。下一課改用**隨機取樣**：不把格子建完，只在需要的地方查詢。`,
    simHint: '16 × 16 時窄縫被格子吃掉，A* 回報失敗；把「解析度」改成 32 × 32 再看一次。勾選「預覽：RRT」看看下一課的隨機樹如何鑽過窄縫。',
  },
];

const quiz: QuizQuestion[] = [
  {
    question: '在 8 連通（斜邊代價 $\\sqrt2$）的格點上，下列哪個 $h$ 能保證 A* 找到最短路徑？',
    options: [
      '曼哈頓距離 $|\\Delta i|+|\\Delta j|$',
      '歐氏距離 $\\sqrt{\\Delta i^2+\\Delta j^2}$',
      '兩者都可以，因為它們都是距離',
      '兩者都不行，8 連通只能用 Dijkstra',
    ],
    answer: 1,
    explanation: '可容許的意思是**永遠不高估**剩下的代價。斜走一步的真實代價是 $\\sqrt2\\approx1.41$，曼哈頓距離卻把它算成 2，朝斜方向的估計偏高，A* 可能太早接受一條較長的路（在 32 × 32 的「三個方塊」地圖選 8 連通加曼哈頓距離，表格裡的代價就比 Dijkstra 多一成以上）。歐氏距離是直線長度，任何格點路徑都不會比直線短，所以一定可容許。4 連通時曼哈頓距離才可容許，而且比歐氏距離更貼近真實代價。',
  },
  {
    question: '在「U 形陷阱」上把 A* 的 $\\varepsilon$ 從 1 調到 2，最可能看到什麼？',
    options: [
      '展開的節點變少，路徑可能變長，但不會超過最短路徑的 2 倍',
      '展開的節點變少，而且路徑一定一樣短',
      '展開的節點變多，但路徑變短',
      '找不到路徑，因為 $\\varepsilon h$ 高估了',
    ],
    answer: 0,
    explanation: '$\\varepsilon h$ 會高估，A* 失去最短路徑的保證；但它更積極地朝 G 推進，展開數下降（32 × 32、8 連通時大約少四成，代價多約 5%）。只要 $h$ 是一致的，代價仍不超過最短代價的 $\\varepsilon$ 倍。完備性不受影響：只要 S 與 G 連通，一樣找得到路徑。',
  },
  {
    question: '機器人每次任務結束都要回到同一個待命姿勢，但出發時的姿勢每次都不同。哪種做法最划算？',
    options: [
      '每次都從目前姿勢跑一次 A*',
      '從待命姿勢算一次 wavefront 距離場，之後每次都沿 $d$ 遞減的方向走',
      '每次都用 $\\varepsilon=3$ 的加權 A*，比較快',
      '把解析度加倍，搜尋就會變快',
    ],
    answer: 1,
    explanation: 'Wavefront 的計算只取決於地圖與目標：算一次就能服務所有起點，每一步只要比較鄰居的 $d$，完全不用搜尋。A* 每換一個起點就得重新搜尋。反過來說，如果目標常常改變，或障礙會移動，距離場就得一直重算，這時針對單次查詢的 A* 比較划算。',
  },
  {
    question: '「窄縫」地圖在 16 × 16 時回報找不到路徑，32 × 32 時就找到了。若要在 6 軸手臂上用同樣的「每軸格數加倍」來解決，格子總數會變成原來的幾倍？',
    options: ['2 倍', '12 倍', '64 倍', '4096 倍'],
    answer: 2,
    explanation: '格子數是 $k^n$。每軸加倍得到 $(2k)^n=2^n k^n$：平面地圖是 4 倍，6 維就是 $2^6=64$ 倍。解析度完備只保證「格子夠細就找得到」，卻沒說要多細；在高維空間每加倍一次都要付出指數級的代價，這正是下一課改用隨機取樣的原因。',
  },
];

// ---------------------------------------------------------------------------------------------

type Algo = 'dijkstra' | 'astar' | 'wave';
type Tool = 'paint' | 'start' | 'goal';
type RGB = [number, number, number];

const RES = [16, 32, 64, 128];
const SPEEDS = [0.25, 0.5, 1, 2, 4, 16];
/**
 * The map occupies the unit square. The margin on the left / bottom holds the θ tick labels of the
 * 2R map; the strip on top keeps the text overlay from covering the first row of cells.
 */
const WORLD: World = { xmin: -0.085, xmax: 1.012, ymin: -0.07, ymax: 1.062 };
const ARM_WORLD: World = { xmin: -2.25, xmax: 2.25, ymin: -2.25, ymax: 2.25 };
const HEUR_LABEL: Record<HeuristicKind, string> = { euclid: '歐氏距離（Euclidean）', octile: '八方距離（octile）', manhattan: '曼哈頓距離（Manhattan）' };
const HEUR_SHORT: Record<HeuristicKind, string> = { euclid: '歐氏', octile: '八方', manhattan: '曼哈頓' };
const mapIndex = (key: string): number => Math.max(0, MAPS.findIndex((m) => m.key === key));

interface StepCfg {
  map: number; n: number; conn: Conn; algo: Algo; eps: number; heur: HeuristicKind;
  /** hidden: no search drawn yet; play: replay the expansion order from k = 0; done: show the finished search. */
  search: 'hidden' | 'play' | 'done';
  speed?: number; probe?: boolean; graph?: boolean; nums?: boolean; arrows?: boolean; outline?: boolean; sweep?: boolean; tool?: Tool;
}
const STEP_CFG: StepCfg[] = [
  { map: mapIndex('blocks'), n: 16, conn: 8, algo: 'dijkstra', eps: 1, heur: 'euclid', search: 'hidden', probe: true, graph: true },
  { map: mapIndex('blocks'), n: 32, conn: 8, algo: 'dijkstra', eps: 1, heur: 'euclid', search: 'play', speed: 1 },
  { map: mapIndex('blocks'), n: 32, conn: 8, algo: 'astar', eps: 1, heur: 'euclid', search: 'play', speed: 0.25, outline: true },
  { map: mapIndex('utrap'), n: 32, conn: 8, algo: 'astar', eps: 2, heur: 'euclid', search: 'done', sweep: true },
  { map: mapIndex('cspace2r'), n: 16, conn: 4, algo: 'wave', eps: 1, heur: 'euclid', search: 'done', nums: true, arrows: true, tool: 'start' },
  { map: mapIndex('gap'), n: 16, conn: 8, algo: 'astar', eps: 1, heur: 'euclid', search: 'done' },
];

const clampN = (x: number, lo: number, hi: number): number => Math.max(lo, Math.min(hi, x));
const degStr = (rad: number): string => `${fmt((rad * 180) / Math.PI, 0)}°`;

function mount(host: HTMLElement, ctx: LessonContext): LessonHandle {
  // ------------------------------------------------------------------ state
  let mapIdx = 0;
  let master = MAPS[0].build();
  let wrap = MAPS[0].wrap;
  let n = 32;
  let conn: Conn = 8;
  let algo: Algo = 'astar';
  let eps = 1;
  let heur: HeuristicKind = 'euclid';
  /** S, G and the probe are stored as fractions of the map so they survive resolution changes. */
  let Sf: P2 = [MAPS[0].S[0], MAPS[0].S[1]];
  let Gf: P2 = [MAPS[0].G[0], MAPS[0].G[1]];
  let probeF: P2 = [PROBE[0], PROBE[1]];
  let grid: Grid = { n, blocked: new Uint8Array(n * n), wrap };
  let gridVersion = 0;
  let S = 0;
  let G = 0;
  let freeCount = 0;
  let edgeCount = 0;
  let trace!: SearchTrace;
  /** Dijkstra from S to G: the optimal reference for every comparison. */
  let opt!: SearchTrace;
  let wavePath: number[] = [];
  /** Number of expansions currently shown (the replay position) and its fractional accumulator. */
  let k = 0;
  let kf = 0;
  let playing = false;
  let speed = 1;
  let showSearch = true;
  let showProbe = false;
  let showGraph = false;
  let showNums = false;
  let showArrows = false;
  let showOutline = false;
  let showSweep = false;
  let showRRT = false;
  let tool: Tool = 'paint';
  let hover = -1;
  let rrt: RRTPreview | null = null;
  let armAnim: { pts: P2[]; u: number; dur: number } | null = null;
  let armTips: Pt[] = [];
  let sweepKey = '';
  let sideTimer = 0;
  let destroyed = false;

  const is2R = (): boolean => MAPS[mapIdx].key === 'cspace2r';
  const ijOf = (c: number): [number, number] => [c % n, (c - (c % n)) / n];
  const centre = (c: number): Pt => { const [i, j] = ijOf(c); return [(i + 0.5) / n, (j + 0.5) / n]; };
  const cellOfFrac = (f: readonly [number, number]): number =>
    clampN(Math.floor(f[1] * n), 0, n - 1) * n + clampN(Math.floor(f[0] * n), 0, n - 1);
  const cellAt = (p: Pt): number => (p[0] < 0 || p[1] < 0 || p[0] >= 1 || p[1] >= 1 ? -1 : cellOfFrac(p));
  const clampFrac = (p: Pt): P2 => [clampN(p[0], 0, 0.99999), clampN(p[1], 0, 0.99999)];
  /** Replay speed at 1×: an n × n map floods in roughly six seconds. */
  const baseRate = (): number => (n * n) / 6;
  const effEps = (): number => (algo === 'dijkstra' ? 0 : eps);
  const finalPath = (): number[] => (algo === 'wave' ? wavePath : trace.path);
  /** Is the S → G path already visible at the current replay position? */
  const pathShown = (): boolean => {
    if (!showSearch) return false;
    if (algo === 'wave') { const cs = trace.closedAt[S]; return cs >= 0 && cs < k; }
    return trace.goalStep > 0 && k >= trace.goalStep;
  };

  // ------------------------------------------------------------------ colours (theme variables → RGB for the cell raster)
  const rgbCache = new Map<string, RGB>();
  let parseCtx: CanvasRenderingContext2D | null = null;
  function rgbOf(color: string): RGB {
    const css = themeColor(color, '#808080');
    const hit = rgbCache.get(css);
    if (hit) return hit;
    if (!parseCtx) {
      const pc = document.createElement('canvas');
      pc.width = 1; pc.height = 1;
      parseCtx = pc.getContext('2d', { willReadFrequently: true });
    }
    let rgb: RGB = [128, 128, 128];
    if (parseCtx) {
      parseCtx.clearRect(0, 0, 1, 1);
      parseCtx.fillStyle = '#808080';
      parseCtx.fillStyle = css;
      parseCtx.fillRect(0, 0, 1, 1);
      const d = parseCtx.getImageData(0, 0, 1, 1).data;
      rgb = [d[0], d[1], d[2]];
    }
    rgbCache.set(css, rgb);
    return rgb;
  }

  // ------------------------------------------------------------------ layout
  simTitle(host, '格點地圖上的圖搜尋', 'Dijkstra · A* · wavefront');
  const chips = presetChips(host, MAPS.map((m, i) => ({ label: m.label, apply: () => loadMap(i) })), { label: '地圖' });
  const views = el('div', { style: { display: 'flex', flexWrap: 'wrap', gap: '0.6rem', alignItems: 'flex-start' } });
  host.appendChild(views);
  // The map is capped at 460 px so the transport and the main selectors stay in the first view of the
  // sim pane; the side column (2R workspace view, ε sweep plot) sits next to it and wraps below on phones.
  const gridBox = el('div', { style: { flex: '1.6 1 300px', minWidth: '0', maxWidth: '460px', margin: '0 auto' } });
  const sideBox = el('div', { style: { flex: '1 1 190px', minWidth: '0', display: 'none', flexDirection: 'column', gap: '0.6rem' } });
  const armBox = el('div', { style: { minWidth: '0', display: 'none' } });
  sideBox.appendChild(armBox);
  views.append(gridBox, sideBox);
  const cv = new Canvas2D(gridBox, { aspect: 1, world: WORLD, margin: 0.01, overlay: true });
  // slightly smaller than the default so the A* line (counts + cost) fits on one line next to the side column
  Object.assign(cv.overlay.style, { top: '0.15rem', background: 'var(--bg-elev)', opacity: '0.92', padding: '0 0.35rem', borderRadius: '4px', fontSize: '0.68rem' });
  armBox.appendChild(el('div', { class: 'mr-note', style: { margin: '0 0 0.25rem' } }, '工作空間：滑鼠移到 C-space 的格子上可預覽那個姿勢'));
  const armCv = new Canvas2D(armBox, { aspect: 1, world: ARM_WORLD, margin: 0.02, overlay: true });
  const armRow = el('div', { class: 'mr-playback' });
  armBox.appendChild(armRow);
  const armBtn = button(armRow, { label: '▶ 手臂沿路徑走', small: true, title: '讓手臂從 S 沿目前的路徑動到 G', onClick: () => startArm() });

  legend(host, [
    { color: 'var(--fg-muted)', label: '障礙格' },
    { color: 'var(--series-6)', label: '已展開（closed；深淺相間 = 代價帶）' },
    { color: 'var(--series-4)', label: '開放串列（open）' },
    { color: 'var(--series-5)', label: '路徑' },
    { color: 'var(--series-3)', label: '起點 S' },
    { color: 'var(--series-1)', label: '目標 G' },
  ]);

  // transport: single step / auto / finish / reset / speed, plus a replay scrubber
  const transport = el('div', { class: 'mr-playback' });
  host.appendChild(transport);
  const stepBtn = button(transport, { label: '▸ 單步', small: true, title: '展開一個節點', onClick: () => stepOnce() });
  const playBtn = button(transport, { label: '▶ 自動', small: true, variant: 'primary', title: '自動播放 / 暫停', onClick: () => (playing ? pause() : play()) });
  const endBtn = button(transport, { label: '⏭ 跑完', small: true, title: '直接跳到搜尋結束', onClick: () => finish() });
  button(transport, { label: '⟲ 重設', small: true, title: '回到還沒展開任何節點的時候', onClick: () => resetSearch() });
  const speedSel = select<string>(transport, {
    options: SPEEDS.map((s) => ({ value: String(s), label: `${s}×` })), value: '1',
    onChange: (v) => { speed = Number(v); },
  });
  speedSel.el.title = '播放速度';
  const statusEl = el('span', { class: 'mr-note', style: { margin: '0' } });
  transport.appendChild(statusEl);
  const scrub = slider(host, {
    label: '回放：已展開節點數 k', min: 0, max: 1, step: 1, value: 0,
    format: (v) => `${fmt(v, 0)} / ${trace ? trace.expanded : 0}`,
    onInput: (v) => scrubTo(v),
  });

  const panel = controlPanel(host, { compact: true });
  const algoSel = select<Algo>(panel, {
    label: '演算法', value: algo,
    options: [
      { value: 'dijkstra', label: 'Dijkstra（h ≡ 0）' },
      { value: 'astar', label: 'A*（f = g + εh）' },
      { value: 'wave', label: 'Wavefront（從 G 倒推）' },
    ],
    onChange: (v) => { algo = v; showSearch = true; recompute(); },
  });
  const epsSl = slider(panel, {
    label: '啟發式權重 ε', min: 0, max: 3, step: 0.1, value: eps, format: (v) => fmt(v, 1),
    onInput: (v) => {
      eps = Math.round(v * 10) / 10;
      // ε belongs to A*: moving it from another algorithm switches to A*
      if (algo !== 'astar') { algo = 'astar'; algoSel.set('astar', { silent: true }); }
      showSearch = true;
      scheduleRecompute();
    },
  });
  const heurSel = select<HeuristicKind>(panel, {
    label: '啟發式 h', value: heur,
    options: (['euclid', 'octile', 'manhattan'] as HeuristicKind[]).map((h) => ({ value: h, label: HEUR_LABEL[h] })),
    onChange: (v) => { heur = v; recompute(); },
  });
  const connSel = select<string>(panel, {
    label: '鄰居', value: '8',
    options: [{ value: '4', label: '4 連通（代價 1）' }, { value: '8', label: '8 連通（斜邊 √2）' }],
    onChange: (v) => { conn = v === '4' ? 4 : 8; recompute(); },
  });
  const resSel = select<string>(panel, {
    label: '解析度（每軸 k 格）', value: String(n),
    options: RES.map((r) => ({ value: String(r), label: `${r} × ${r}` })),
    onChange: (v) => { n = Number(v); rebuildGrid(); recompute(); },
  });
  const toolSel = select<Tool>(panel, {
    label: '點擊格子', value: tool,
    options: [{ value: 'paint', label: '畫／擦障礙' }, { value: 'start', label: '設定起點 S' }, { value: 'goal', label: '設定目標 G' }],
    onChange: (v) => { tool = v; },
  });
  const togRow = row(panel);
  const tProbe = toggle(togRow, { label: '鄰居探針', value: showProbe, onChange: (v) => { showProbe = v; hProbe.enabled = v; cv.requestRender(); } });
  const tGraph = toggle(togRow, { label: '整張圖（節點與邊）', value: showGraph, onChange: (v) => { showGraph = v; cv.requestRender(); } });
  const tNums = toggle(togRow, { label: 'g／d 數值', value: showNums, onChange: (v) => { showNums = v; cv.requestRender(); } });
  const tArrows = toggle(togRow, { label: '父節點箭頭', value: showArrows, onChange: (v) => { showArrows = v; cv.requestRender(); } });
  const tOutline = toggle(togRow, { label: 'Dijkstra 範圍', value: showOutline, onChange: (v) => { showOutline = v; cv.requestRender(); } });
  const tSweep = toggle(togRow, { label: 'ε 掃描圖', value: showSweep, onChange: (v) => { showSweep = v; syncSweepBox(); scheduleSide(); } });
  const tRRT = toggle(togRow, { label: '預覽：RRT', value: showRRT, onChange: (v) => { showRRT = v; refresh(); } });

  // live comparison of all algorithms on the current map (placed right under the controls, above the read-outs)
  const cmpBox = el('div', { style: { margin: '0.5rem 0 0.3rem' } });
  host.appendChild(cmpBox);
  const info = kv(host, ['地圖', '圖', '演算法', '進度', '目前節點', '結果']);
  cmpBox.appendChild(el('div', { class: 'mr-note', style: { margin: '0 0 0.2rem' } }, '同一張地圖、同一組 S、G：各演算法要展開多少節點？（地圖或 S、G 一改就重算）'));
  const cellStyle = (num: boolean): Record<string, string> => ({
    padding: '0.15rem 0.4rem', borderBottom: '1px solid var(--border)', textAlign: num ? 'right' : 'left',
    fontFamily: num ? 'var(--mono)' : 'inherit', fontVariantNumeric: 'tabular-nums', whiteSpace: num ? 'nowrap' : 'normal',
  });
  const table = el('table', { style: { borderCollapse: 'collapse', width: '100%', fontSize: '0.8rem' } });
  cmpBox.appendChild(table);
  table.appendChild(el('tr', {}, ...['演算法', '展開節點', '路徑代價', '與最短相比'].map((t, i) =>
    el('th', { style: { ...cellStyle(i === 1 || i === 2), fontWeight: '600', color: 'var(--fg-muted)', fontFamily: 'inherit' } }, t))));
  const cmpRows = [0, 1, 2, 3].map(() => {
    const cells = [0, 1, 2, 3].map((i) => el('td', { style: cellStyle(i === 1 || i === 2) }));
    const tr = el('tr', {}, ...cells);
    table.appendChild(tr);
    return { tr, cells };
  });

  // ε sweep: lives in the side column next to the map so it is visible together with the search
  const sweepBox = el('div', { style: { display: 'none', minWidth: '0' } });
  sideBox.appendChild(sweepBox);
  sweepBox.appendChild(el('div', { class: 'mr-note', style: { margin: '0 0 0.25rem' } }, 'A* 的 ε 掃描：目前的地圖與 S、G，兩條曲線都以 Dijkstra 為 1'));
  const plot = new Plot2D(sweepBox, {
    xLabel: 'ε', yLabel: '÷ Dijkstra',
    series: [{ name: '展開節點數', color: '--series-6', width: 2.2 }, { name: '路徑代價', color: '--series-5', width: 2.2 }],
    xRange: [0, 3], yRange: [0, 1.5], height: 210,
  });

  const scaleCard = el('div', { class: 'card', style: { padding: '0.5rem 0.75rem', margin: '0.5rem 0' } });
  host.appendChild(scaleCard);
  scaleCard.appendChild(mdElement('**格子數 $N=k^{\\,n}$ 怎麼長？** $k$ 是每軸格數（跟著上面的解析度），$n$ 是 C-space 的維度；最後一欄假設每格只做一次 1 μs 的碰撞查詢。'));
  const scaleTable = el('table', { style: { borderCollapse: 'collapse', width: '100%', fontSize: '0.8rem', marginTop: '0.3rem' } });
  scaleCard.appendChild(scaleTable);
  scaleTable.appendChild(el('tr', {}, ...['維度 n', '格子數', '逐格查碰撞'].map((t, i) =>
    el('th', { style: { ...cellStyle(i > 0), fontWeight: '600', color: 'var(--fg-muted)', fontFamily: 'inherit' } }, t))));
  const scaleDims: [number, string][] = [[2, '2（平面地圖、2R 手臂）'], [3, '3（平面移動機器人 x, y, θ）'], [6, '6（6 軸手臂）']];
  const scaleRows = scaleDims.map(([, label]) => {
    const cells = [el('td', { style: cellStyle(false) }, label), el('td', { style: cellStyle(true) }), el('td', { style: cellStyle(true) })];
    scaleTable.appendChild(el('tr', {}, ...cells));
    return cells;
  });
  scaleTable.appendChild(el('tr', {},
    el('td', { style: cellStyle(false) }, '6，每軸 100 格'),
    el('td', { style: cellStyle(true) }, `${powLabel(100, 6)} = ${fmtCount(cellCount(100, 6))}`),
    el('td', { style: cellStyle(true) }, fmtDuration(cellCount(100, 6) * 1e-6))));

  note(host, '格子裡只要有一部分是障礙，整格就算障礙（保守離散化）；8 連通時不准斜著擠過兩個障礙格的角。拖曳 S、G 或在地圖上畫障礙，搜尋會立即重算；「▶ 自動」與回放滑桿可以重看展開順序。');

  // ------------------------------------------------------------------ derived data
  function countGraph(): void {
    let f = 0;
    let e = 0;
    const inc = (): void => { e++; };
    for (let c = 0; c < n * n; c++) {
      if (grid.blocked[c]) continue;
      f++;
      forNeighbors(grid, c, conn, inc);
    }
    freeCount = f;
    edgeCount = e / 2;
  }

  /** The cell of a stored fraction; a blocked cell is replaced by the nearest free one (and the fraction moved there). */
  function placeEndpoint(which: 'S' | 'G'): number {
    const f = which === 'S' ? Sf : Gf;
    let c = cellOfFrac(f);
    if (grid.blocked[c]) {
      const alt = nearestFree(grid, c);
      if (alt >= 0) {
        c = alt;
        const p = centre(c);
        if (which === 'S') Sf = [p[0], p[1]]; else Gf = [p[0], p[1]];
      }
    }
    return c;
  }

  function rebuildGrid(): void {
    grid = { n, blocked: downsample(master, MASTER, n), wrap };
    gridVersion++;
    S = placeEndpoint('S');
    G = placeEndpoint('G');
    syncSide();
    updateScaleCard();
  }

  /** The side column shows the 2R workspace (C-space map only) and / or the ε sweep plot. */
  function syncSide(): void {
    const arm = is2R();
    armBox.style.display = arm ? '' : 'none';
    sweepBox.style.display = showSweep ? '' : 'none';
    sideBox.style.display = arm || showSweep ? 'flex' : 'none';
  }

  function recompute(): void {
    if (destroyed) return;
    countGraph();
    if (algo === 'wave') {
      trace = wavefront(grid, G, conn);
      opt = bestFirst(grid, S, G, { conn, eps: 0 });
      wavePath = descend(grid, trace.g, S, conn);
    } else {
      trace = bestFirst(grid, S, G, { conn, eps: effEps(), heuristic: heur });
      opt = effEps() === 0 ? trace : bestFirst(grid, S, G, { conn, eps: 0 });
      wavePath = [];
    }
    const N = trace.expanded;
    if (playing) { k = 0; kf = 0; } else { k = showSearch ? N : 0; kf = k; }
    scrub.setRange(0, Math.max(1, N), 1);
    scrub.set(k, { silent: true });
    rrt = null;
    armAnim = null;
    updateArmTips();
    refresh();
    scheduleSide();
  }
  const scheduleRecompute = rafThrottle(recompute);

  /** Moving S does not change the wavefront: only the descent path (and the references) are updated. */
  function onStartMoved(): void {
    armAnim = null;
    if (algo === 'wave') {
      opt = bestFirst(grid, S, G, { conn, eps: 0 });
      wavePath = descend(grid, trace.g, S, conn);
      rrt = null;
      updateArmTips();
      refresh();
      scheduleSide();
    } else scheduleRecompute();
  }

  function updateArmTips(): void {
    armTips = is2R() ? finalPath().map((c) => arm2RPoints(cellAngles(n, c))[2] as Pt) : [];
    armBtn.disabled = armTips.length < 2;
  }

  // ------------------------------------------------------------------ readouts
  function algoText(): string {
    if (algo === 'dijkstra') return 'Dijkstra：f = g（h ≡ 0）';
    if (algo === 'wave') return 'Wavefront：從 G 標出每格到 G 的距離 d';
    const tag = eps === 0 ? '，ε = 0 就是 Dijkstra' : eps > 1 ? '，加權（不保證最短）' : '';
    return `A*：f = g + ${fmt(eps, 1)}·h（${HEUR_SHORT[heur]}${tag}）`;
  }
  const fmtG = (g: number): string => (Number.isFinite(g) ? fmt(g, Math.abs(g - Math.round(g)) < 1e-6 ? 0 : 2) : '∞');

  function currentText(): string {
    if (!showSearch || k <= 0) return '—';
    const c = trace.order[k - 1];
    const [i, j] = ijOf(c);
    if (algo === 'wave') return `(${i}, ${j})：d = ${fmtG(trace.g[c])}`;
    const g = trace.g[c];
    const h = heuristic(grid, c, G, heur);
    const e = effEps();
    return `(${i}, ${j})：g = ${fmt(g, 2)}，h = ${fmt(h, 2)}，f = g + ${fmt(e, 1)}h = ${fmt(g + e * h, 2)}`;
  }

  function resultText(): string {
    if (!showSearch) return '—（還沒開始搜尋）';
    const N = trace.expanded;
    if (algo === 'wave') {
      if (!Number.isFinite(trace.g[S]) && k >= N) return '這個起點到不了 G（不連通）';
      const cs = trace.closedAt[S];
      if (cs < 0 || cs >= k) return '波還沒傳到 S';
      return `S 的 d = ${fmtG(trace.g[S])}：沿箭頭走 ${wavePath.length - 1} 步到 G，不必再搜尋`;
    }
    if (trace.goalStep > 0 && k >= trace.goalStep) {
      const extra = trace.cost > opt.cost + 1e-9 ? `，比最短多 ${fmt(100 * (trace.cost / opt.cost - 1), 1)}%` : '，是最短路徑';
      return `找到路徑：代價 ${fmt(trace.cost, 2)}（最短 ${fmt(opt.cost, 2)}${extra}）`;
    }
    if (k >= N) return '開放串列空了：在這個解析度下 S 與 G 不連通';
    return '搜尋中…';
  }

  function statusText(): string {
    if (!showSearch) return '按「▶ 自動」或「▸ 單步」開始搜尋';
    const N = trace.expanded;
    if (k < N) return playing ? '展開中…' : '暫停';
    if (algo === 'wave') return '✓ 波前完成：每個可達的格子都有 d';
    return trace.goalStep > 0 ? '✓ 取出 G，搜尋結束' : '✗ 找不到路徑';
  }

  function readouts(): void {
    const m = MAPS[mapIdx];
    info.set('地圖', `${m.label}：${n} × ${n} = ${fmtCount(n * n)} 格（自由 ${fmtCount(freeCount)}）${wrap ? '，環面' : ''}`);
    info.set('圖', `${fmtCount(freeCount)} 個節點、${fmtCount(edgeCount)} 條邊（${conn} 連通）`);
    info.set('演算法', algoText());
    const N = trace.expanded;
    info.set('進度', showSearch ? `已展開 ${k} / ${N}，open list ${trace.openCount[clampN(k, 0, N)]} 個` : '—');
    info.set('目前節點', currentText());
    info.set('結果', resultText());
    statusEl.textContent = statusText();
  }

  function syncTransport(): void {
    playBtn.textContent = playing ? '⏸ 暫停' : '▶ 自動';
    const done = showSearch && k >= trace.expanded;
    stepBtn.disabled = done;
    endBtn.disabled = done;
  }

  function refresh(): void {
    if (destroyed) return;
    syncTransport();
    readouts();
    cv.requestRender();
    if (is2R()) armCv.requestRender();
  }

  // ------------------------------------------------------------------ comparison table, ε sweep, scale card
  function scheduleSide(): void {
    if (sideTimer) clearTimeout(sideTimer);
    sideTimer = window.setTimeout(() => { sideTimer = 0; updateSide(); }, 120);
  }

  function setRow(r: number, label: string, expanded: number, cost: number, best: number, extra?: string): void {
    const cells = cmpRows[r].cells;
    const put = (i: number, t: string): void => { if (cells[i].textContent !== t) cells[i].textContent = t; };
    put(0, label);
    put(1, String(expanded));
    put(2, Number.isFinite(cost) ? fmt(cost, 2) : '—（無路徑）');
    put(3, extra ?? (!Number.isFinite(cost) ? '—' : cost <= best + 1e-9 ? '最短' : `+${fmt(100 * (cost / best - 1), 1)}%`));
  }

  function updateSide(): void {
    if (destroyed) return;
    const d = opt;
    const a1 = algo === 'astar' && eps === 1 ? trace : bestFirst(grid, S, G, { conn, eps: 1, heuristic: heur });
    const weighted = algo === 'astar' && eps !== 0 && eps !== 1;
    const w = algo === 'wave' ? trace : wavefront(grid, G, conn);
    setRow(0, 'Dijkstra（h ≡ 0）', d.expanded, d.cost, d.cost);
    setRow(1, `A*（ε = 1，${HEUR_SHORT[heur]}）`, a1.expanded, a1.cost, d.cost);
    if (weighted) setRow(2, `加權 A*（ε = ${fmt(eps, 1)}，${HEUR_SHORT[heur]}）`, trace.expanded, trace.cost, d.cost);
    cmpRows[2].tr.style.display = weighted ? '' : 'none';
    setRow(3, 'Wavefront（從 G）', w.expanded, w.g[S], d.cost, '算一次，所有起點共用');
    const cur = algo === 'wave' ? 3 : algo === 'dijkstra' || eps === 0 ? 0 : eps === 1 ? 1 : 2;
    cmpRows.forEach((r, i) => { r.tr.style.background = i === cur ? 'var(--accent-soft)' : ''; });
    if (showSweep) updateSweep(d);
  }

  function updateSweep(d: SearchTrace): void {
    const key = `${gridVersion}|${n}|${conn}|${heur}|${S}|${G}`;
    if (key !== sweepKey) {
      sweepKey = key;
      const xs: number[] = [];
      const ye: number[] = [];
      const yc: number[] = [];
      if (Number.isFinite(d.cost) && d.expanded > 0) {
        const step = n >= 128 ? 0.2 : 0.1;
        for (let s = 0; s * step <= 3 + 1e-9; s++) {
          const e = s * step;
          const r = bestFirst(grid, S, G, { conn, eps: e, heuristic: heur });
          xs.push(e); ye.push(r.expanded / d.expanded); yc.push(r.cost / d.cost);
        }
      }
      plot.setXY(0, xs, ye);
      plot.setXY(1, xs, yc);
    }
    // no label on the ε line: it collided with the legend near ε = 1 and was clipped at ε = 3 (the slider shows ε)
    plot.setMarkers([{ y: 1, color: '--fg-muted' }, { x: eps, color: '--accent' }]);
  }

  function syncSweepBox(): void {
    syncSide();
    if (showSweep) sweepKey = '';
  }

  function updateScaleCard(): void {
    scaleDims.forEach(([dim], r) => {
      const N = cellCount(n, dim);
      scaleRows[r][1].textContent = `${powLabel(n, dim)} = ${fmtCount(N)}`;
      scaleRows[r][2].textContent = fmtDuration(N * 1e-6);
    });
  }

  // ------------------------------------------------------------------ transport
  function play(): void {
    showSearch = true;
    if (k >= trace.expanded) { k = 0; kf = 0; }
    playing = true;
    loop.start();
    refresh();
  }
  function pause(): void { playing = false; refresh(); }
  function stepOnce(): void {
    showSearch = true;
    playing = false;
    if (k < trace.expanded) k++;
    kf = k;
    scrub.set(k, { silent: true });
    refresh();
  }
  function finish(): void {
    showSearch = true;
    playing = false;
    k = trace.expanded; kf = k;
    scrub.set(k, { silent: true });
    refresh();
  }
  function resetSearch(): void {
    showSearch = true;
    playing = false;
    k = 0; kf = 0;
    scrub.set(0, { silent: true });
    refresh();
  }
  function scrubTo(v: number): void {
    showSearch = true;
    playing = false;
    k = clampN(Math.round(v), 0, trace.expanded); kf = k;
    refresh();
  }

  // ------------------------------------------------------------------ the 2R arm (only for the C-space map)
  function startArm(): void {
    if (!is2R()) return;
    if (!pathShown()) finish(); // the replay jumps to the end so the path being followed is on screen
    const path = finalPath();
    if (path.length < 2) { armAnim = null; armCv.requestRender(); return; }
    // unwrap the joint angles so the arm turns the short way across ±180°
    const pts: P2[] = [];
    for (const c of path) {
      const q = cellAngles(n, c);
      const p = pts[pts.length - 1];
      pts.push(p ? [p[0] + wrapToPi(q[0] - p[0]), p[1] + wrapToPi(q[1] - p[1])] : q);
    }
    armAnim = { pts, u: 0, dur: Math.max(1.2, 0.16 * (pts.length - 1)) };
    loop.start();
  }
  function armAt(a: { pts: P2[]; u: number }): P2 {
    const m = a.pts.length - 1;
    if (m < 1) return a.pts[0] ?? cellAngles(n, S);
    const t = clampN(a.u, 0, 1) * m;
    const s = clampN(Math.floor(t), 0, m - 1);
    const f = t - s;
    const p = a.pts[s], q = a.pts[s + 1];
    return [wrapToPi(p[0] + f * (q[0] - p[0])), wrapToPi(p[1] + f * (q[1] - p[1]))];
  }
  function armQ(): P2 {
    if (armAnim && armAnim.u < 1) return armAt(armAnim);
    if (hover >= 0) return cellAngles(n, hover);
    if (armAnim) return armAt(armAnim);
    return cellAngles(n, S);
  }
  function drawArm(c: Canvas2D, q: P2, color: string | null, alpha: number): number {
    const pts = arm2RPoints(q) as Pt[];
    const hits = color ? 0 : arm2RHits(q);
    const w = Math.max(3, 2 * ARM_2R.halfWidth * c.scale);
    c.line(pts[0], pts[1], { color: color ?? (hits & 1 ? '--danger' : '--fg-muted'), width: w, alpha });
    c.line(pts[1], pts[2], { color: color ?? (hits & 2 ? '--danger' : '--fg-muted'), width: w, alpha });
    c.point(pts[0], w * 0.32, { color: '--bg-elev', alpha });
    c.point(pts[1], w * 0.32, { color: '--bg-elev', alpha });
    c.point(pts[2], 2.5, { color: '--fg', alpha });
    return hits;
  }
  armCv.draw = (c) => {
    if (!trace) return;
    c.grid(0.5);
    c.circle([0, 0], ARM_2R.L[0] + ARM_2R.L[1], { color: '--grid-strong', dash: [4, 4], width: 1 });
    const ob = ARM_2R.obstacle;
    c.disk(ob.c, ob.r, { fill: '--fg-muted', alpha: 0.45 });
    c.circle(ob.c, ob.r, { color: '--fg-muted', width: 1.5 });
    c.polygon([[-0.12, -0.12], [0.12, -0.12], [0, 0]], { fill: '--grid-strong', color: '--grid-strong', width: 1 });
    drawArm(c, cellAngles(n, S), '--series-3', 0.35);
    drawArm(c, cellAngles(n, G), '--series-1', 0.35);
    if (armTips.length > 1 && pathShown()) c.polyline(armTips, { color: '--series-5', width: 1.6, dash: [5, 4] });
    const q = armQ();
    const hits = drawArm(c, q, null, 1);
    c.setOverlayText(`θ = (${degStr(q[0])}, ${degStr(q[1])})${hits ? '  碰撞' : ''}`);
  };

  // ------------------------------------------------------------------ grid canvas drawing
  let cellCanvas: HTMLCanvasElement | null = null;
  let cellCtx: CanvasRenderingContext2D | null = null;
  let cellImg: ImageData | null = null;

  /** One pixel per cell: obstacles, closed cells in alternating cost bands, open cells. */
  function paintCells(): void {
    if (!cellCanvas || cellCanvas.width !== n) {
      cellCanvas = document.createElement('canvas');
      cellCanvas.width = n; cellCanvas.height = n;
      cellCtx = cellCanvas.getContext('2d');
      cellImg = cellCtx ? cellCtx.createImageData(n, n) : null;
    }
    if (!cellCtx || !cellImg) return;
    const d = cellImg.data;
    const obs = rgbOf('--fg-muted');
    const closed = rgbOf('--series-6');
    const open = rgbOf('--series-4');
    const band = n / 8;
    const t = trace;
    for (let j = 0; j < n; j++) {
      for (let i = 0; i < n; i++) {
        const c = j * n + i;
        const o = ((n - 1 - j) * n + i) * 4;
        let rgb: RGB | null = null;
        let a = 0;
        if (grid.blocked[c]) { rgb = obs; a = 0.85; }
        else if (showSearch) {
          const ca = t.closedAt[c];
          if (ca >= 0 && ca < k) { rgb = closed; a = Math.floor(t.g[c] / band) % 2 ? 0.55 : 0.3; }
          else if (t.openedAt[c] >= 0 && t.openedAt[c] <= k) { rgb = open; a = 0.85; }
        }
        if (rgb) { d[o] = rgb[0]; d[o + 1] = rgb[1]; d[o + 2] = rgb[2]; d[o + 3] = Math.round(a * 255); }
        else d[o + 3] = 0;
      }
    }
    cellCtx.putImageData(cellImg, 0, 0);
  }

  /** Batched line segments [x0, y0, x1, y1, ...] in world coordinates. */
  function strokeSegs(c: Canvas2D, segs: number[], color: string, width: number, alpha: number, dash: number[] = []): void {
    if (!segs.length) return;
    const g = c.ctx;
    g.save();
    g.strokeStyle = themeColor(color, '#888');
    g.lineWidth = width;
    g.globalAlpha = alpha;
    g.setLineDash(dash);
    g.lineCap = 'butt';
    g.beginPath();
    for (let s = 0; s < segs.length; s += 4) {
      const a = c.toScreen([segs[s], segs[s + 1]]);
      const b = c.toScreen([segs[s + 2], segs[s + 3]]);
      g.moveTo(a[0], a[1]);
      g.lineTo(b[0], b[1]);
    }
    g.stroke();
    g.restore();
  }

  /** Grid step from cell a to its neighbour b, taking the short way across a seam. */
  function stepDelta(a: number, b: number): [number, number] {
    const [ai, aj] = ijOf(a);
    const [bi, bj] = ijOf(b);
    let di = bi - ai;
    let dj = bj - aj;
    if (Math.abs(di) > 1) di = -Math.sign(di);
    if (Math.abs(dj) > 1) dj = -Math.sign(dj);
    return [di, dj];
  }

  /** Polyline through cell centres; a step across a seam leaves through one edge and re-enters at the opposite one. */
  function cellPath(c: Canvas2D, cells: readonly number[], style: Style): void {
    const h = 1 / n;
    let run: Pt[] = [];
    const flush = (): void => { if (run.length > 1) c.polyline(run, style); run = []; };
    for (let s = 0; s < cells.length; s++) {
      const p = centre(cells[s]);
      if (s > 0) {
        const [pi, pj] = ijOf(cells[s - 1]);
        const [i, j] = ijOf(cells[s]);
        if (Math.abs(i - pi) > 1 || Math.abs(j - pj) > 1) {
          const [di, dj] = stepDelta(cells[s - 1], cells[s]);
          const prev = run[run.length - 1];
          if (prev) run.push([prev[0] + di * h * 0.5, prev[1] + dj * h * 0.5]);
          flush();
          run.push([p[0] - di * h * 0.5, p[1] - dj * h * 0.5]);
        }
      }
      run.push(p);
    }
    flush();
  }

  function drawGridLines(c: Canvas2D): void {
    const segs: number[] = [];
    for (let s = 1; s < n; s++) segs.push(s / n, 0, s / n, 1, 0, s / n, 1, s / n);
    strokeSegs(c, segs, '--grid-strong', 1, 0.45);
  }

  /** Boundary of the region the Dijkstra reference run expanded (step 3's comparison). */
  function drawOutline(c: Canvas2D): void {
    const inSet = (i: number, j: number): boolean => {
      if (wrap) { i = (i + n) % n; j = (j + n) % n; } else if (i < 0 || j < 0 || i >= n || j >= n) return false;
      return opt.closedAt[j * n + i] >= 0;
    };
    const h = 1 / n;
    const segs: number[] = [];
    for (let j = 0; j < n; j++) {
      for (let i = 0; i < n; i++) {
        if (!inSet(i, j)) continue;
        if (!inSet(i + 1, j)) segs.push((i + 1) * h, j * h, (i + 1) * h, (j + 1) * h);
        if (!inSet(i - 1, j)) segs.push(i * h, j * h, i * h, (j + 1) * h);
        if (!inSet(i, j + 1)) segs.push(i * h, (j + 1) * h, (i + 1) * h, (j + 1) * h);
        if (!inSet(i, j - 1)) segs.push(i * h, j * h, (i + 1) * h, j * h);
      }
    }
    strokeSegs(c, segs, '--fg', 1.6, 0.7, [4, 3]);
  }

  function drawGraph(c: Canvas2D, px: number): void {
    if (px < 9) return;
    const h = 1 / n;
    const segs: number[] = [];
    const g = c.ctx;
    g.save();
    g.fillStyle = themeColor('--fg-muted', '#888');
    g.globalAlpha = 0.8;
    g.beginPath();
    const r = Math.min(2.4, px * 0.1);
    for (let cc = 0; cc < n * n; cc++) {
      if (grid.blocked[cc]) continue;
      const [i, j] = ijOf(cc);
      const x = (i + 0.5) * h;
      const y = (j + 0.5) * h;
      const sp = c.toScreen([x, y]);
      g.moveTo(sp[0] + r, sp[1]);
      g.arc(sp[0], sp[1], r, 0, 2 * Math.PI);
      forNeighbors(grid, cc, conn, (nb, _cost, d) => {
        if (d !== 0 && d !== 1 && d !== 4 && d !== 5) return; // each undirected edge once
        const [di, dj] = DIRS[d];
        const [ni, nj] = ijOf(nb);
        if (ni === i + di && nj === j + dj) segs.push(x, y, x + di * h, y + dj * h);
        else {
          // edge across a seam of the torus: half an edge out of each side
          segs.push(x, y, x + di * h * 0.5, y + dj * h * 0.5);
          const nx = (ni + 0.5) * h;
          const ny = (nj + 0.5) * h;
          segs.push(nx, ny, nx - di * h * 0.5, ny - dj * h * 0.5);
        }
      });
    }
    g.fill();
    g.restore();
    strokeSegs(c, segs, '--fg-muted', 1, 0.35);
  }

  /**
   * Arrows: for a forward search, closed cell → its back pointer (towards S). For the wavefront, the
   * downhill neighbour argmin(c + d) that descend() follows, so the drawn S → G path always runs along
   * the arrows (the search's own parent pointer may pick a different, equally short neighbour). A cell
   * with a strictly smaller d is closed earlier, so this is also correct while the wave is replayed.
   */
  function drawArrows(c: Canvas2D, px: number, withNumbers: boolean): void {
    if (px < 9) return;
    const h = 1 / n;
    const t = trace;
    const shafts: number[] = [];
    const heads: number[] = [];
    const hs = clampN(px * 0.2, 3, 7);
    const wave = algo === 'wave';
    for (let cc = 0; cc < n * n; cc++) {
      const ca = t.closedAt[cc];
      if (ca < 0 || ca >= k) continue;
      const p = wave ? downhill(grid, t.g, cc, conn) : t.parent[cc];
      if (p < 0) continue;
      const [di, dj] = stepDelta(cc, p);
      const L = Math.hypot(di, dj);
      const ux = di / L;
      const uy = dj / L;
      const o = centre(cc);
      // the head sits on the cell border it leaves through; the shaft is omitted when numbers fill the centre
      const tip: Pt = [o[0] + di * h * 0.5, o[1] + dj * h * 0.5];
      if (!withNumbers) shafts.push(o[0] + ux * h * 0.12, o[1] + uy * h * 0.12, tip[0] - ux * h * 0.1, tip[1] - uy * h * 0.1);
      heads.push(tip[0], tip[1], ux, uy);
    }
    strokeSegs(c, shafts, '--fg', 1.2, 0.55);
    const g = c.ctx;
    g.save();
    g.fillStyle = themeColor('--fg', '#222');
    g.globalAlpha = 0.75;
    g.beginPath();
    for (let s = 0; s < heads.length; s += 4) {
      const sp = c.toScreen([heads[s], heads[s + 1]]);
      const ux = heads[s + 2];
      const uy = -heads[s + 3]; // screen y points down
      g.moveTo(sp[0] + ux * hs * 0.5, sp[1] + uy * hs * 0.5);
      g.lineTo(sp[0] - ux * hs * 0.5 - uy * hs * 0.45, sp[1] - uy * hs * 0.5 + ux * hs * 0.45);
      g.lineTo(sp[0] - ux * hs * 0.5 + uy * hs * 0.45, sp[1] - uy * hs * 0.5 - ux * hs * 0.45);
      g.closePath();
    }
    g.fill();
    g.restore();
  }

  function drawNumbers(c: Canvas2D, px: number): void {
    if (px < 13) return;
    const t = trace;
    const g = c.ctx;
    g.save();
    g.font = `${Math.round(clampN(px * 0.36, 8, 12))}px ${cssVar('--mono') || 'monospace'}`;
    g.textAlign = 'center';
    g.textBaseline = 'middle';
    g.fillStyle = themeColor('--fg', '#222');
    for (let cc = 0; cc < n * n; cc++) {
      const ca = t.closedAt[cc];
      if (ca < 0 || ca >= k) continue;
      const v = t.g[cc];
      const s = Math.abs(v - Math.round(v)) < 1e-6 ? String(Math.round(v)) : px >= 30 ? v.toFixed(1) : String(Math.round(v));
      const sp = c.toScreen(centre(cc));
      g.fillText(s, sp[0], sp[1]);
    }
    g.restore();
  }

  function drawPaths(c: Canvas2D): void {
    const t = trace;
    const N = t.expanded;
    if (algo === 'wave') {
      const cs = t.closedAt[S];
      if (cs >= 0 && cs < k && wavePath.length > 1) cellPath(c, wavePath, { color: '--series-5', width: 3.5 });
    } else if (t.goalStep > 0 && k >= t.goalStep) {
      if (opt.goalStep > 0 && t.cost > opt.cost + 1e-9) cellPath(c, opt.path, { color: '--fg', width: 2, dash: [6, 4], alpha: 0.6 });
      cellPath(c, t.path, { color: '--series-5', width: 3.5 });
    } else if (k > 0 && k < N) {
      // the best route to the node just expanded, read off the parent pointers
      const chain: number[] = [];
      for (let x = t.order[k - 1]; x >= 0 && chain.length <= N; x = t.parent[x]) chain.push(x);
      chain.reverse();
      cellPath(c, chain, { color: '--series-5', width: 2, dash: [4, 3], alpha: 0.85 });
    }
    if (k > 0 && k < N) {
      const [i, j] = ijOf(t.order[k - 1]);
      const h = 1 / n;
      c.polygon([[i * h, j * h], [(i + 1) * h, j * h], [(i + 1) * h, (j + 1) * h], [i * h, (j + 1) * h]], { color: '--series-2', width: 2.2 });
    }
  }

  function drawProbe(c: Canvas2D): void {
    const pc = cellOfFrac(probeF);
    const [i, j] = ijOf(pc);
    const h = 1 / n;
    const labels = c.scale * h >= 18; // cost labels need room: one per neighbour cell
    c.polygon([[i * h, j * h], [(i + 1) * h, j * h], [(i + 1) * h, (j + 1) * h], [i * h, (j + 1) * h]], { color: '--series-5', width: 2.5 });
    if (grid.blocked[pc]) return;
    const o = centre(pc);
    const x = (p: Pt): void => {
      const r = h * 0.18;
      c.line([p[0] - r, p[1] - r], [p[0] + r, p[1] + r], { color: '--danger', width: 2.5 });
      c.line([p[0] - r, p[1] + r], [p[0] + r, p[1] - r], { color: '--danger', width: 2.5 });
    };
    for (const e of edgesFrom(grid, pc, conn)) {
      if (e.status === 'off') continue;
      const [di, dj] = DIRS[e.dir];
      const to: Pt = [o[0] + di * h, o[1] + dj * h];
      if (e.status === 'ok') {
        // arrow up to the neighbour's border region, cost label at the neighbour's centre (labels never overlap)
        const len = Math.hypot(di, dj);
        const stop = 1 - 0.3 / len;
        c.arrow([o[0] + di * h * 0.22, o[1] + dj * h * 0.22], [o[0] + di * h * stop, o[1] + dj * h * stop], { color: '--series-5', width: 2, head: 7 });
        if (labels) c.text(to, e.cost === 1 ? '1' : '√2', { color: '--series-5', font: `bold 11px ${cssVar('--font') || 'sans-serif'}`, bg: '--bg-elev' });
      } else if (e.status === 'blocked') {
        x(to);
      } else {
        c.line(o, to, { color: '--danger', width: 2, dash: [4, 3] });
        x([o[0] + di * h * 0.5, o[1] + dj * h * 0.5]);
      }
    }
  }

  function drawRRT(c: Canvas2D): void {
    if (!rrt) rrt = rrtPreview({ n: MASTER, blocked: master, wrap: false }, [Sf[0] * MASTER, Sf[1] * MASTER], [Gf[0] * MASTER, Gf[1] * MASTER], { seed: 7 });
    const segs: number[] = [];
    for (let s = 1; s < rrt.nodes.length; s++) {
      const a = rrt.nodes[rrt.parent[s]];
      const b = rrt.nodes[s];
      segs.push(a[0] / MASTER, a[1] / MASTER, b[0] / MASTER, b[1] / MASTER);
    }
    strokeSegs(c, segs, '--series-7', 1.2, 0.8);
    if (rrt.goalNode > 0) {
      const pts: Pt[] = [];
      for (let s = rrt.goalNode; s >= 0; s = rrt.parent[s]) pts.push([rrt.nodes[s][0] / MASTER, rrt.nodes[s][1] / MASTER]);
      c.polyline(pts, { color: '--series-7', width: 3 });
    }
  }

  function drawAxes(c: Canvas2D): void {
    if (!is2R()) return;
    const font = `10px ${cssVar('--font') || 'sans-serif'}`;
    const lab = ['−180°', '−90°', '0°', '90°', '180°'];
    for (let s = 0; s <= 4; s++) {
      c.text([s / 4, 0], lab[s], { dy: 9, font, color: '--fg-muted' });
      if (s > 0 && s < 4) c.text([0, s / 4], lab[s], { dx: -4, align: 'right', font, color: '--fg-muted' });
    }
    c.text([0.875, 0], 'θ₁', { dy: 9, font: `bold ${font}`, color: '--fg' });
    c.text([0, 0.875], 'θ₂', { dx: -4, align: 'right', font: `bold ${font}`, color: '--fg' });
  }

  function overlayText(): string {
    if (hover >= 0) {
      const [i, j] = ijOf(hover);
      const q = is2R() ? cellAngles(n, hover) : null;
      const pos = `(${i}, ${j})${q ? ` θ = (${degStr(q[0])}, ${degStr(q[1])})` : ''}`;
      if (grid.blocked[hover]) return `${pos} 障礙`;
      if (!showSearch) return `${pos} 自由格`;
      const ca = trace.closedAt[hover];
      if (ca >= 0 && ca < k) {
        if (algo === 'wave') return `${pos} d = ${fmtG(trace.g[hover])}`;
        const h = heuristic(grid, hover, G, heur);
        return `${pos} g = ${fmt(trace.g[hover], 2)}  h = ${fmt(h, 2)}`;
      }
      const oa = trace.openedAt[hover];
      return `${pos} ${oa >= 0 && oa <= k ? '在開放串列中' : '還沒碰到'}`;
    }
    // step 1 compares 4- and 8-connectivity: keep the graph size in view
    if (!showSearch) return `${n} × ${n} 格 · ${conn} 連通：${fmtCount(freeCount)} 節點、${fmtCount(edgeCount)} 條邊`;
    const open = trace.openCount[clampN(k, 0, trace.expanded)];
    if (algo === 'dijkstra') return `Dijkstra · 已展開 ${k} · open ${open}`;
    if (algo === 'wave') return `Wavefront · 已標記 ${k} 格 · open ${open}`;
    // the number to compare with stays in view: how many nodes Dijkstra needs for the same query
    // ...and, once G is popped, how the cost compares with the optimum
    let tail = '';
    if (trace.goalStep > 0 && k >= trace.goalStep && Number.isFinite(opt.cost)) {
      tail = trace.cost > opt.cost + 1e-9 ? ` · 代價 +${fmt(100 * (trace.cost / opt.cost - 1), 1)}%` : ' · 代價 = 最短';
    }
    return `A* ε=${fmt(eps, 1)} · 展開 ${k}（Dijkstra ${opt.expanded}）${tail}`;
  }

  cv.draw = (c) => {
    if (!trace) return;
    paintCells();
    if (cellCanvas) c.image(cellCanvas, { xmin: 0, xmax: 1, ymin: 0, ymax: 1 });
    const px = c.scale / n;
    if (px >= 7) drawGridLines(c);
    if (showSearch && showOutline && algo === 'astar') drawOutline(c);
    if (showGraph) drawGraph(c, px);
    const nums = showSearch && showNums && px >= 13;
    if (showSearch && showArrows) drawArrows(c, px, nums);
    if (nums) drawNumbers(c, px);
    if (showSearch) drawPaths(c);
    if (showProbe) drawProbe(c);
    if (showRRT && !wrap) drawRRT(c);
    c.polygon([[0, 0], [1, 0], [1, 1], [0, 1]], { color: '--grid-strong', width: 1.2 });
    drawAxes(c);
    const skipped: string[] = [];
    if (showGraph && px < 9) skipped.push('整張圖');
    if (showSearch && showNums && px < 13) skipped.push('數值');
    if (showSearch && showArrows && px < 9) skipped.push('箭頭');
    if (skipped.length) c.text([1, 1.033], `格子太小，不畫${skipped.join('、')}`, { align: 'right', color: '--fg-muted', font: `11px ${cssVar('--font') || 'sans-serif'}` });
    if (showRRT && wrap) c.text([0.5, 0.96], 'RRT 預覽只用在平面地圖（下一課再處理環面）', { color: '--fg-muted', bg: '--bg-elev', font: `11px ${cssVar('--font') || 'sans-serif'}` });
    c.setOverlayText(overlayText());
  };

  // ------------------------------------------------------------------ handles
  function dragEndpoint(which: 'S' | 'G', p: Pt): void {
    const f = clampFrac(p);
    const c = cellOfFrac(f);
    if (grid.blocked[c] || c === (which === 'S' ? G : S)) return; // obstacles and the other endpoint are off limits
    if (which === 'S') {
      Sf = f;
      if (c !== S) { S = c; onStartMoved(); }
    } else {
      Gf = f;
      if (c !== G) { G = c; scheduleRecompute(); }
    }
    cv.requestRender();
  }
  function snapEndpoint(which: 'S' | 'G'): void {
    const p = centre(which === 'S' ? S : G);
    if (which === 'S') Sf = [p[0], p[1]]; else Gf = [p[0], p[1]];
    cv.requestRender();
  }
  // the canvas does not report pointer moves during a handle drag, so a hover read-out would go stale
  const clearHover = (): void => { hover = -1; };
  cv.addHandle({ get: () => [Sf[0], Sf[1]], set: (p) => dragEndpoint('S', p), onStart: clearHover, onEnd: () => snapEndpoint('S'), color: '--series-3', label: 'S', radius: 8 });
  cv.addHandle({ get: () => [Gf[0], Gf[1]], set: (p) => dragEndpoint('G', p), onStart: clearHover, onEnd: () => snapEndpoint('G'), color: '--series-1', label: 'G', radius: 8 });
  const hProbe = cv.addHandle({
    get: () => [probeF[0], probeF[1]],
    set: (p) => { probeF = clampFrac(p); cv.requestRender(); },
    onStart: clearHover,
    onEnd: () => { const q = centre(cellOfFrac(probeF)); probeF = [q[0], q[1]]; cv.requestRender(); },
    color: '--series-5', radius: 7, enabled: false,
  });

  // ------------------------------------------------------------------ pointer: paint obstacles or click to place S / G
  let painting = false;
  let paintVal: 0 | 1 = 1;
  let lastPaint = -1;
  function paintCell(c: number): void {
    if (c < 0 || c === S || c === G || grid.blocked[c] === paintVal) return;
    const [i, j] = ijOf(c);
    paintMaster(master, MASTER, n, i, j, paintVal);
    grid.blocked[c] = paintVal;
    gridVersion++;
    chips.clear();
    scheduleRecompute();
  }
  /** Paint every cell on the straight line between two cells (so fast strokes leave no gaps). */
  function paintLine(a: number, b: number): void {
    const [ai, aj] = ijOf(a);
    const [bi, bj] = ijOf(b);
    const m = Math.max(Math.abs(bi - ai), Math.abs(bj - aj), 1);
    for (let s = 1; s <= m; s++) paintCell(Math.round(aj + ((bj - aj) * s) / m) * n + Math.round(ai + ((bi - ai) * s) / m));
  }
  function setEndpointCell(which: 'S' | 'G', c: number): void {
    if (grid.blocked[c] || c === (which === 'S' ? G : S)) return;
    const p = centre(c);
    if (which === 'S') {
      Sf = [p[0], p[1]];
      if (c === S) return;
      S = c;
      onStartMoved();
      if (algo === 'wave') startArm();
    } else {
      Gf = [p[0], p[1]];
      if (c === G) return;
      G = c;
      recompute();
    }
  }
  const offPointer = cv.onPointer({
    down: (p, ev) => {
      const c = cellAt(p);
      if (c < 0) return;
      ev.preventDefault(); // a stroke on the map must not start a text selection on the page
      if (tool === 'paint') {
        painting = true;
        paintVal = grid.blocked[c] ? 0 : 1;
        lastPaint = c;
        paintCell(c);
        try { cv.canvas.setPointerCapture(ev.pointerId); } catch { /* synthetic events have no capture target */ }
      } else setEndpointCell(tool === 'start' ? 'S' : 'G', c);
    },
    move: (p, ev) => {
      const c = cellAt(p);
      if (painting && (ev.buttons & 1)) {
        if (c >= 0 && c !== lastPaint) { paintLine(lastPaint, c); lastPaint = c; }
        return;
      }
      painting = false;
      if (c !== hover) {
        hover = c;
        cv.requestRender();
        if (is2R()) armCv.requestRender();
      }
    },
    up: () => { painting = false; },
  });
  const onLeave = (): void => {
    if (hover === -1) return;
    hover = -1;
    cv.requestRender();
    if (is2R()) armCv.requestRender();
  };
  cv.canvas.addEventListener('pointerleave', onLeave);

  // ------------------------------------------------------------------ animation (search replay + arm motion)
  const loop = createLoop((rawDt) => {
    if (destroyed) return;
    // The first frame after loop.start() inside an input handler can report a negative dt (its rAF
    // timestamp precedes the performance.now() taken in start()); a negative step would run the
    // replay / arm animation backwards (armAnim.u < 0 indexed pts[-1]).
    const dt = Math.max(0, rawDt);
    let busy = false;
    if (playing) {
      const N = trace.expanded;
      kf = Math.min(N, kf + dt * speed * baseRate());
      const nk = Math.floor(kf);
      if (nk !== k) { k = nk; scrub.set(k, { silent: true }); }
      if (k >= N) playing = false;
      refresh();
      busy = playing;
    }
    if (armAnim && armAnim.u < 1) {
      armAnim.u = Math.min(1, armAnim.u + dt / armAnim.dur);
      armCv.requestRender();
      busy = busy || armAnim.u < 1;
    }
    if (!busy) loop.stop();
  });

  // ------------------------------------------------------------------ maps and steps
  function loadMap(i: number): void {
    const m = MAPS[i];
    mapIdx = i;
    master = m.build();
    wrap = m.wrap;
    Sf = [m.S[0], m.S[1]];
    Gf = [m.G[0], m.G[1]];
    hover = -1;
    rebuildGrid();
    recompute();
    if (playing) loop.start();
  }

  const setT = (t: ToggleHandle, v: boolean): void => t.set(v, { silent: true });
  const focusMap: HTMLElement[][] = [
    [cv.el, connSel.el],
    [cv.el, transport],
    [cmpBox, algoSel.el],
    [epsSl.el, sweepBox],
    [cv.el, armBox],
    [resSel.el, scaleCard],
  ];
  let focused: HTMLElement[] = [];

  function applyStep(i: number): void {
    const s = clampN(i, 0, STEP_CFG.length - 1);
    const cfg = STEP_CFG[s];
    playing = false;
    armAnim = null;
    hover = -1;
    mapIdx = cfg.map;
    const m = MAPS[mapIdx];
    master = m.build();
    wrap = m.wrap;
    Sf = [m.S[0], m.S[1]];
    Gf = [m.G[0], m.G[1]];
    probeF = [PROBE[0], PROBE[1]];
    n = cfg.n; conn = cfg.conn; algo = cfg.algo; eps = cfg.eps; heur = cfg.heur;
    speed = cfg.speed ?? 1;
    tool = cfg.tool ?? 'paint';
    showProbe = !!cfg.probe; showGraph = !!cfg.graph; showNums = !!cfg.nums; showArrows = !!cfg.arrows;
    showOutline = !!cfg.outline; showSweep = !!cfg.sweep; showRRT = false;
    algoSel.set(algo, { silent: true });
    epsSl.set(eps, { silent: true });
    heurSel.set(heur, { silent: true });
    connSel.set(String(conn), { silent: true });
    resSel.set(String(n), { silent: true });
    toolSel.set(tool, { silent: true });
    speedSel.set(String(speed), { silent: true });
    setT(tProbe, showProbe); setT(tGraph, showGraph); setT(tNums, showNums); setT(tArrows, showArrows);
    setT(tOutline, showOutline); setT(tSweep, showSweep); setT(tRRT, showRRT);
    hProbe.enabled = showProbe;
    syncSweepBox();
    chips.highlight(mapIdx);
    rebuildGrid();
    showSearch = cfg.search !== 'hidden';
    playing = cfg.search === 'play';
    recompute();
    if (playing) loop.start();
    focused.forEach((f) => f.classList.remove('mr-focus'));
    focused = focusMap[s];
    focused.forEach((f) => f.classList.add('mr-focus'));
  }

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
      playing = false;
      armAnim = null;
      loop.stop();
      if (sideTimer) { clearTimeout(sideTimer); sideTimer = 0; }
      offPointer();
      cv.canvas.removeEventListener('pointerleave', onLeave);
      cv.destroy();
      armCv.destroy();
      plot.destroy();
      cellCanvas = null;
      cellCtx = null;
      cellImg = null;
      parseCtx = null;
      rgbCache.clear();
    },
  };
}

const lesson: Lesson = { meta, steps, quiz, mount };
export default lesson;
