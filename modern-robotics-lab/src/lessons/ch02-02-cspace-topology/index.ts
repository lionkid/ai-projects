import type { Lesson, LessonContext, LessonHandle, LessonStep, QuizQuestion } from '@/types';
import * as THREE from 'three';
import {
  el, cssVar, fmt, controlPanel, row, slider, angleSlider, button, select, kv, note, simTitle, legend, split,
  Canvas2D, Scene3D, Plot2D, createLoop, presetChips, drawPlanarArm, makeTextSprite, mdElement, rafThrottle,
  setThemedColor, themedStandardMaterial,
  type KVHandle, type Loop, type Pt,
} from '@/ui';
import { planarFK, wrapToPi } from '@/mr';
import { meta } from './meta';
import {
  PATH_PRESETS, PI, TWO_PI, appendUnwrapped, densify, glueStages, ikNearest, implicitCoords, morphNormal, morphPoint,
  otherElbow, pathLength, pushTrail, samplePath, torusDist, trailPieces, unwrapSeq, wrapPolylines, wrapTh,
  type Th, type Trail, type V3,
} from './torus';

const steps: LessonStep[] = [
  {
    title: '工作空間不是組態空間',
    md: `**組態（configuration）**是「機器人每個部位在哪裡」的完整描述；對平面 2R 手臂，兩個關節角 $(\\theta_1, \\theta_2)$ 就夠了。所有組態的集合稱為**組態空間（C-space）**。

**工作空間（workspace）**只記錄末端能到達的位置，可以想成 C-space 經過正向運動學投下的「影子」。這個影子不是一對一的：同一個末端位置，通常有**肘上**、**肘下**兩種姿勢，它們在 C-space 裡是**兩個不同的點**。

$$\\theta \\in \\mathcal{C} \\;\\xrightarrow{\\;\\text{FK}\\;}\\; p_{\\text{tip}} \\in \\text{workspace}$$`,
    simHint: '拖曳「③ 手臂」視圖的末端（藍色把手）：淡色手臂是同一末端位置的另一組解；對照它在「① 方格」與「② 環面」上落在哪裡。',
  },
  {
    title: '把角度畫成方格',
    md: `最直接的畫法是把兩個角度當成平面座標：各取 $[-\\pi, \\pi)$，C-space 就成了邊長 $2\\pi$ 的方格。這種用最少數字（2 個）描述 2 個自由度的方式，叫做**顯式座標（explicit coordinates）**。

方格中每一點都是一個姿勢；拖曳組態點時會留下軌跡。但請注意邊緣：$\\theta_1$ 從 $179°$ 再轉 $2°$，讀數會跳成 $-179°$，點從右緣瞬間移到左緣——手臂本身卻只動了一點點。外圍淡色區是方格的「週期複製」，提示右緣之外其實接著左緣。`,
    simHint: '示範中 θ₁ 越過 180°：青色軌跡在右緣斷開、從左緣接上，手臂卻只是平順地轉了一點。換你在「① 方格」中拖曳組態點越過邊界試試看。',
  },
  {
    title: '黏合邊界：環面',
    md: `既然 $\\theta_1 = -\\pi$ 與 $\\theta_1 = \\pi$ 是同一個姿勢，方格的左右兩邊應該視為同一條線並**黏合**；上下兩邊（$\\theta_2 = \\pm\\pi$）也一樣。先黏一對邊得到圓柱，再把圓柱兩端接起來，就是甜甜圈形狀的**環面（torus）**：

$$\\mathcal{C}_{2R} = S^1 \\times S^1 = T^2$$

環面沒有邊界，手臂的任何連續運動都是一條不斷的曲線。**拓撲**（topology）描述的正是這個形狀本身，與我們用哪一套座標去畫它無關。`,
    simHint: '看黏合動畫（或拖曳「黏合」滑桿）：橘色邊（θ₁ = ±π）與紫色邊（θ₂ = ±π）各自黏成環面上的哪一圈？上一步在方格裡斷成兩段的青色軌跡，黏好後變成什麼樣子？',
  },
  {
    title: '拓撲 vs 表示法',
    md: `同一個環面可以有不同的**表示法（representation）**：

- **顯式**：$(\\theta_1, \\theta_2)$，2 個數、沒有約束；代價是在 $\\pm\\pi$ 需要包裹（wrap），數值會跳。
- **隱式（implicit）**：$(c_1, s_1, c_2, s_2) = (\\cos\\theta_1, \\sin\\theta_1, \\cos\\theta_2, \\sin\\theta_2)$，4 個數加上 2 條約束

$$c_i^2 + s_i^2 = 1, \\qquad i = 1, 2$$

隱式表示在整個環面上都平滑、不跳變，但數字比自由度多，運算時必須一直維持約束成立。第 3 章用 9 個數表示 3 自由度的旋轉矩陣，就是同一種取捨。`,
    simHint: '示範讓 θ₁ 轉一整圈：圖中 θ₁（藍）在 ±180° 跳了一下，cos θ₁（紅）與 sin θ₁（綠）卻一路平滑。「④ 表示法」已切到隱式，看兩條約束的殘差是否一直是 0；再切回顯式比較。',
  },
  {
    title: '路徑在三個視圖中長什麼樣',
    md: `同一個動作在三個視圖裡是同一件事，只是長相不同：

- 方格中的**直線**（兩關節等速轉動）在工作空間裡通常是一條**曲線**。
- 穿越方格邊界的路徑在方格上被切成兩段，在環面上卻連續不斷。
- 讓 $\\theta_1$ 轉一圈的路徑，在方格上是兩端貼邊的線段，在環面上卻是**封閉的圈**。這種繞著「洞」走的圈無法在環面上連續縮成一點——這正是環面與平面、球面在拓撲上不同的證據。`,
    simHint: '點選一條預設路徑（或按「✎ 畫路徑」自己畫，可畫進外圍淡色區），再按「▶ 播放路徑」，看三個視圖同步移動。',
  },
  {
    title: '其他系統的 C-space',
    md: `用同樣的眼光看其他系統（圖庫中可點選）：

- 平面上的剛體：$\\R^2 \\times S^1$，3 維。
- 球面擺：$S^2$，2 維。
- 能轉又能滑的旋鈕：$\\R \\times S^1$（圓柱），2 維。
- 轉盤上自轉的咖啡杯：$S^1 \\times S^1 = T^2$，與 2R 手臂相同。

維度相同不代表形狀相同：球面上任何圈都能縮成一點，環面不行。另外，經緯度無法在極點上無奇異地描述球面——表示法的麻煩常常來自拓撲本身。下一課會看到，**速度**上的限制（例如輪子不能側滑）並不會改變 C-space 的形狀。`,
    simHint: '點選「③ 圖庫」中的卡片閱讀說明；拖曳環面或方格上的組態點時，「咖啡杯」卡片會跟著轉——它和 2R 手臂共用同一個 C-space。',
  },
];

const quiz: QuizQuestion[] = [
  {
    question: '一支沒有關節極限的 2R 手臂，組態從 $(\\theta_1, \\theta_2) = (170°, 0°)$ 移到 $(-170°, 0°)$。關節 1 **最少**需要轉多少？',
    options: ['340°', '20°', '0°，兩者是同一個組態', '180°'],
    answer: 1,
    explanation: '在方格上兩點看似相隔很遠，但 $\\pm180°$ 是被黏合的同一條線：從 170° 往正方向轉 20° 就越過接縫到達 −170°。方格（顯式座標）把環面「剪開」才會造成這種距離上的錯覺；兩者並非同一組態，因為 170° 與 −170° 差了 20°。',
  },
  {
    question: '把 2R 手臂的連桿長度從 $L_1 = L_2 = 1$ 改成 $L_1 = 1, L_2 = 0.5$，下列敘述何者正確？',
    options: [
      'workspace 從實心圓盤變成有洞的環帶，C-space 仍然是環面 $T^2$',
      'C-space 也會跟著出現一個洞',
      'C-space 變成球面 $S^2$',
      'workspace 與 C-space 都不變',
    ],
    answer: 0,
    explanation: 'C-space 只由關節決定：兩個能自由旋轉的轉動關節就是 $S^1 \\times S^1$，與連桿長度無關。連桿長度只影響正向運動學這個「投影」，所以 workspace 的內半徑從 $|L_1 - L_2| = 0$ 變成 $0.5$。',
  },
  {
    question: '下列哪個系統的 C-space 與無關節極限的 2R 手臂**拓撲相同**？',
    options: [
      '平放在桌上、可以滑動與旋轉的硬幣',
      '轉盤上可以自轉的咖啡杯（兩者都能無限轉圈）',
      '球面擺',
      '可沿軸滑動又能旋轉的旋鈕',
    ],
    answer: 1,
    explanation: '咖啡杯有兩個獨立、可無限轉圈的角度，正是 $S^1 \\times S^1 = T^2$。硬幣是 $\\R^2 \\times S^1$（3 維）；球面擺 $S^2$ 雖然也是 2 維，但球面上每個圈都能縮成一點，環面上繞洞的圈不行；旋鈕是 $\\R \\times S^1$，滑動方向不會繞回來。',
  },
  {
    question: '用 3×3 旋轉矩陣的 9 個元素描述只有 3 個自由度的空間轉動，這是哪一種表示法？主要代價是什麼？',
    options: [
      '顯式表示；數值在 ±π 會跳變',
      '隱式表示；數字比自由度多，必須同時維持 6 條約束（正交、單位長度）',
      '顯式表示；需要 9 條約束',
      '隱式表示；有些轉動無法表示',
    ],
    answer: 1,
    explanation: '9 個數 − 6 條約束（三欄各為單位長度、兩兩正交）= 3 個自由度，這是嵌入更高維空間的隱式表示。好處是平滑、沒有包裹或奇異點；代價是數字較多，而且數值計算的誤差會讓矩陣逐漸偏離約束，必須定期修正。',
  },
];

// ---------------------------------------------------------------------------

const R_T = 1;      // torus major radius (3D view units)
const r_T = 0.42;   // torus minor radius
const NU = 72;      // θ1 segments of the torus mesh
const NV = 36;      // θ2 segments
const TRAIL_MAX = 400;
const TRAIL_STEP = 0.025; // min. spacing (rad, on the torus) between stored trail samples
const PATH_DOTS_MAX = 700;
const MAP_PX = 512;
const LABEL_FONT = 44;
const LABEL_SCALE = 0.0036;
/** Step-2 demo: a short move that crosses the θ1 = ±π seam near the top of the tube (θ2 ≈ π/2), unwrapped. */
const SEAM_SWEEP: Th[] = [[2.4, 1.85], [4.1, 1.3]];

interface Card { title: string; space: string; md: string }
const CARDS: Card[] = [
  {
    title: '平面剛體', space: 'ℝ² × S¹ · 3 維',
    md: '**平面剛體**：位置 $(x, y) \\in \\R^2$ 加上轉角 $\\phi \\in S^1$。顯式座標 $(x, y, \\phi)$ 只有 $\\phi$ 需要包裹；隱式可用 $(x, y, \\cos\\phi, \\sin\\phi)$ 加 1 條約束。下一課的單輪車就住在這個 3 維空間。',
  },
  {
    title: '球面擺', space: 'S² · 2 維',
    md: '**球面擺**：擺錘落在以支點為中心的球面上，C-space 是 $S^2$。經緯度是顯式座標，但在南北極經度沒有定義（座標奇異）；隱式表示 $(x, y, z)$ 加上 $x^2 + y^2 + z^2 = L^2$ 則處處平滑。',
  },
  {
    title: '旋轉＋滑動旋鈕', space: 'ℝ × S¹ · 圓柱',
    md: '**能轉又能滑的旋鈕**：沿軸滑動是 $\\R$，繞軸旋轉是 $S^1$，合起來是 $\\R \\times S^1$——一個無限長的圓柱。它和環面一樣是 2 維，但滑動方向不會「繞回來」。',
  },
  {
    title: '咖啡杯遊樂設施', space: 'T² · 與 2R 相同',
    md: '**轉盤上的咖啡杯**：轉盤角與杯子自轉角都能無限轉圈，C-space 是 $S^1 \\times S^1 = T^2$，與 2R 手臂相同。圖中轉盤角取 $\\theta_1$、杯子自轉角取 $\\theta_2$，會跟著其他視圖一起動。（2R 手臂若有關節極限，C-space 只剩方格中的一塊矩形，就不再是完整的環面。）',
  },
];

function mount(host: HTMLElement, ctx: LessonContext): LessonHandle {
  // ---------------- state
  let th: Th = [1.2, 1.3];
  let L1 = 1;
  let L2 = 1;
  let glue = 2;                                   // 0 flat square → 1 cylinder → 2 torus
  let glueAnim: { t: number; dur: number; camMoved: boolean } | null = null;
  let trail: Trail = [];                          // wrapped samples; null = break (the configuration jumped)
  let path: Th[] | null = null;                   // densified, unwrapped
  let pathDots: Th[] = [];                        // evenly spaced samples of `path` for the 3D view
  let playS: number | null = null;                // playback progress ∈ [0, 1] or null
  let sweep: { pts: Th[]; s: number; dur: number } | null = null; // step demos (moves θ without showing a path)
  let destroyed = false;
  let drawing: Th[] | null = null;                // unwrapped raw points while drawing
  let drawMode = false;
  let rep: 'explicit' | 'implicit' = 'explicit';
  let showGallery = false;
  let cardPick = 3;
  let stepNow = ctx.currentStep;
  let armNote: '' | 'singular' | 'clamped' = '';
  // The animation loop (glue morph, camera tween, path playback, plot sampling) only runs while something
  // changes and stops ~1.5 s after the last change; the 3D view renders on demand. An idle lesson does no work.
  let loopRef: Loop | null = null;
  let lastActive = 0;
  function kick(): void {
    lastActive = performance.now();
    if (loopRef && !loopRef.running) loopRef.start();
  }

  // ---------------- layout: 2 × 2 views, then a compact control panel
  simTitle(host, '2R 手臂的組態空間：方格、環面與工作空間', 'T² = S¹ × S¹');
  const [sqHost, torusHost] = split(host);
  const [armHost, infoHost] = split(host);

  // "map" of the square: checkerboard + θ = 0 lines + coloured seams. The 2D square and the 3D surface use the
  // same map; the 3D copy gets lighter tiles in the dark theme because the lit surface would otherwise vanish.
  const mapCanvas = document.createElement('canvas');
  const texCanvas = document.createElement('canvas');
  mapCanvas.width = mapCanvas.height = texCanvas.width = texCanvas.height = MAP_PX;
  const paintMap = (cv: HTMLCanvasElement, forScene: boolean) => {
    const g = cv.getContext('2d');
    if (!g) return;
    const dark = document.documentElement.dataset.theme === 'dark';
    const [tileA, tileB] = !dark ? ['#f4f7fb', '#d9e1ee'] : forScene ? ['#8492ae', '#68748f'] : ['#2b3547', '#1f2735'];
    const N = MAP_PX;
    const cell = N / 8;
    g.fillStyle = tileA;
    g.fillRect(0, 0, N, N);
    g.fillStyle = tileB;
    for (let i = 0; i < 8; i++) for (let j = 0; j < 8; j++) if ((i + j) % 2) g.fillRect(i * cell, j * cell, cell, cell);
    g.strokeStyle = cssVar('--fg-muted', '#5d6676');
    g.globalAlpha = 0.6; g.lineWidth = 3;
    g.beginPath(); g.moveTo(N / 2, 0); g.lineTo(N / 2, N); g.moveTo(0, N / 2); g.lineTo(N, N / 2); g.stroke();
    g.globalAlpha = 1;
    const band = 9;
    g.fillStyle = cssVar('--series-4', '#d97706');
    g.fillRect(0, 0, band, N); g.fillRect(N - band, 0, band, N);
    g.fillStyle = cssVar('--series-5', '#7c3aed');
    g.fillRect(0, 0, N, band); g.fillRect(0, N - band, N, band);
  };
  const paintMaps = () => { paintMap(mapCanvas, false); paintMap(texCanvas, true); };
  paintMaps();

  const W = 1.3 * PI;
  const sq = new Canvas2D(sqHost, { aspect: 1, world: { xmin: -W, xmax: W, ymin: -W, ymax: W }, overlay: true, margin: 0.02 });
  const scene = new Scene3D(torusHost, { aspect: 1, grid: false, axes: false, cameraPos: [2.4, -3.1, 2.4], target: [0, 0, 0], fov: 40, autoStart: false });
  const torusOverlay = el('div', { class: 'mr-overlay' });
  scene.el.appendChild(torusOverlay);
  const arm = new Canvas2D(armHost, { aspect: 1, world: { xmin: -2.5, xmax: 2.5, ymin: -2.5, ymax: 2.5 }, overlay: true, margin: 0.02 });
  const gal = new Canvas2D(armHost, { aspect: 2 / 2.16, world: { xmin: 0, xmax: 2, ymin: 0, ymax: 2.16 }, margin: 0.015, overlay: true });
  gal.overlay.textContent = '③ 圖庫：點選卡片';
  gal.el.style.display = 'none';
  for (const o of [sq.overlay, torusOverlay, arm.overlay, gal.overlay]) {
    Object.assign(o.style, { whiteSpace: 'pre', background: 'color-mix(in srgb, var(--sim-bg) 82%, transparent)', padding: '0.05rem 0.3rem', borderRadius: '4px' });
  }

  // ④ numbers (explicit vs implicit) — replaced by the gallery text in the last step
  const infoBox = el('div');
  const cardBox = el('div', { class: 'md', style: { display: 'none', fontSize: '0.92rem' } });
  infoHost.append(infoBox, cardBox);
  const repSel = select(infoBox, {
    label: '④ 表示法',
    options: [{ value: 'explicit', label: '顯式：(θ₁, θ₂)' }, { value: 'implicit', label: '隱式：(cos θᵢ, sin θᵢ)' }],
    value: 'explicit',
    onChange: (v) => { rep = v; buildKV(); refresh(); },
  });
  const kvBox = el('div', { style: { margin: '0.35rem 0 0.2rem' } });
  infoBox.appendChild(kvBox);
  let numKV: KVHandle = kv(kvBox, []);
  // cos θ1 / sin θ1 are the x / y components of link 1's direction, hence the x-red / y-green axis colours
  const plot = new Plot2D(infoBox, {
    title: 'θ₁ 的兩種記錄（隨時間）',
    series: [
      { name: 'θ₁ (rad)', color: '--accent', width: 2.2 },
      { name: 'cos θ₁', color: '--axis-x' },
      { name: 'sin θ₁', color: '--axis-y' },
    ],
    xWindow: 10, height: 128, yRange: [-3.5, 3.5],
  });

  // ---------------- controls
  const panel = controlPanel(host, { compact: true });
  panel.style.gridTemplateColumns = 'repeat(auto-fit, minmax(94px, 1fr))';
  const sT1 = angleSlider(panel, { label: 'θ₁', value: th[0], onInput: (v) => { stopPlay(); setTheta([v, th[1]], 'slider'); } });
  const sT2 = angleSlider(panel, { label: 'θ₂', value: th[1], onInput: (v) => { stopPlay(); setTheta([th[0], v], 'slider'); } });
  slider(panel, { label: 'L₁', min: 0.3, max: 1.2, step: 0.05, value: L1, unit: 'm', onInput: (v) => { L1 = v; refresh(); } });
  slider(panel, { label: 'L₂', min: 0.3, max: 1.2, step: 0.05, value: L2, unit: 'm', onInput: (v) => { L2 = v; refresh(); } });
  const glueShort = (g: number) => (g < 0.005 ? '方格' : g < 1 ? `圓柱 ${Math.round(g * 100)}%` : g < 1.995 ? `環 ${Math.round((g - 1) * 100)}%` : '環面');
  const glueLong = (g: number) => (g < 0.005 ? '方格（未黏合）' : g < 1 ? `黏上下邊 → 圓柱 ${Math.round(g * 100)}%` : g < 1.995 ? `黏左右邊 → 環面 ${Math.round((g - 1) * 100)}%` : '黏合完成');
  const sGlue = slider(panel, { label: '黏合', min: 0, max: 2, step: 0.01, value: glue, format: glueShort, title: '0 = 方格，1 = 圓柱，2 = 環面', onInput: (v) => { glueAnim = null; setGlue(v); } });

  const btnRow = row(panel);
  const glueBtn = button(btnRow, { label: '▶ 黏合動畫', small: true, variant: 'primary', title: '從方格開始：先黏上下邊成圓柱，再黏左右邊成環面', onClick: () => startGlueAnim() });
  button(btnRow, { label: '⇄ 換另一組解', small: true, title: '末端位置不變，換到另一個肘部方向的關節角', onClick: () => { stopPlay(); setTheta(otherElbow(L1, L2, th).th, 'button', true); } });
  button(btnRow, { label: '清除軌跡', small: true, onClick: () => { trail = []; trailDirty = true; refresh(); } });
  const drawBtn = button(btnRow, { label: '✎ 畫路徑', small: true, title: '在方格上按住拖曳，畫出一條路徑', onClick: () => setDrawMode(!drawMode) });
  const playBtn = button(btnRow, { label: '▶ 播放路徑', small: true, onClick: () => togglePlay() });
  button(btnRow, { label: '清除路徑', small: true, onClick: () => { stopPlay(); setPath(null); chips.clear(); } });
  const chipRow = row(panel);
  const chips = presetChips(chipRow, PATH_PRESETS.map((p, i) => ({ label: p.label, title: p.title, apply: () => loadPreset(i, true) })), { label: '預設路徑：' });
  const pathInfo = el('span', { class: 'mr-note', style: { margin: '0' } });
  chipRow.appendChild(pathInfo);

  // var(...) swatches follow theme switches without any bookkeeping
  legend(host, [
    { color: 'var(--accent)', label: '目前組態' },
    { color: 'var(--fg-muted)', label: '同一末端的另一組解' },
    { color: 'var(--series-4)', label: 'θ₁ = ±π 接縫' },
    { color: 'var(--series-5)', label: 'θ₂ = ±π 接縫' },
    { color: 'var(--series-6)', label: '軌跡' },
    { color: 'var(--series-7)', label: '路徑' },
  ]);
  note(host, '可拖曳：方格中的點、環面上的藍球、手臂末端。點一下環面可直接跳到該組態；拖曳環面空白處可旋轉視角。');

  // ---------------- three.js torus
  const torusGroup = new THREE.Group();
  scene.scene.add(torusGroup);
  const texture = new THREE.CanvasTexture(texCanvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  const nVert = (NU + 1) * (NV + 1);
  const posArr = new Float32Array(nVert * 3);
  const nrmArr = new Float32Array(nVert * 3);
  const uvArr = new Float32Array(nVert * 2);
  const idx: number[] = [];
  for (let i = 0; i <= NU; i++) {
    for (let j = 0; j <= NV; j++) {
      const k = i * (NV + 1) + j;
      uvArr[2 * k] = i / NU;
      uvArr[2 * k + 1] = j / NV;
      if (i < NU && j < NV) {
        const a = k, b = k + NV + 1, c = k + NV + 2, d = k + 1;
        idx.push(a, b, d, b, c, d);
      }
    }
  }
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(posArr, 3));
  geo.setAttribute('normal', new THREE.BufferAttribute(nrmArr, 3));
  geo.setAttribute('uv', new THREE.BufferAttribute(uvArr, 2));
  geo.setIndex(idx);
  const torusMesh = new THREE.Mesh(geo, new THREE.MeshStandardMaterial({ map: texture, side: THREE.DoubleSide, roughness: 0.85, metalness: 0 }));
  torusGroup.add(torusMesh);

  // markers: a solid, depth-tested sphere plus a faint "x-ray" copy that stays visible behind the surface.
  // Colours are CSS variable names, so Scene3D re-resolves them when the theme changes.
  const markerPair = (radius: number, color: string, xrayOpacity: number) => {
    const solid = new THREE.Mesh(new THREE.SphereGeometry(radius, 20, 14), themedStandardMaterial(color, { roughness: 0.35 }));
    const xrayMat = new THREE.MeshBasicMaterial({ transparent: true, opacity: xrayOpacity, depthTest: false, depthWrite: false });
    setThemedColor(xrayMat, color);
    const xray = new THREE.Mesh(new THREE.SphereGeometry(radius, 16, 12), xrayMat);
    xray.renderOrder = 10;
    torusGroup.add(solid, xray);
    return { solid, xray };
  };
  const cur = markerPair(0.075, '--accent', 0.45);
  const ghost = markerPair(0.062, '--fg-muted', 0.4);
  // trail / path dots: one round point sprite per sample (cheap even for software WebGL)
  const dotCanvas = document.createElement('canvas');
  dotCanvas.width = dotCanvas.height = 32;
  const dctx = dotCanvas.getContext('2d');
  if (dctx) { dctx.fillStyle = '#ffffff'; dctx.beginPath(); dctx.arc(16, 16, 14, 0, Math.PI * 2); dctx.fill(); }
  const dotTex = new THREE.CanvasTexture(dotCanvas);
  const makeDots = (max: number, color: string, size: number) => {
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(new Float32Array(max * 3), 3));
    g.setDrawRange(0, 0);
    const mat = new THREE.PointsMaterial({ size, map: dotTex, alphaTest: 0.5, sizeAttenuation: true });
    setThemedColor(mat, color);
    const m = new THREE.Points(g, mat);
    m.frustumCulled = false;
    torusGroup.add(m);
    return m;
  };
  const trailDots = makeDots(TRAIL_MAX, '--series-6', 0.045);
  const pathDotsMesh = makeDots(PATH_DOTS_MAX, '--series-7', 0.06);
  // seam labels: theme-bound text sprites; their scale is divided by the group scale (see rebuildSurface) so they
  // keep a readable size while the long flat sheet is shown shrunk
  const seamLabels = [
    makeTextSprite('θ₁ = ±π', '--series-4', LABEL_FONT, LABEL_SCALE),
    makeTextSprite('θ₂ = ±π', '--series-5', LABEL_FONT, LABEL_SCALE),
  ];
  const labelBase = seamLabels.map((s) => s.scale.clone());
  seamLabels.forEach((s) => torusGroup.add(s));

  let ab = glueStages(glue);
  let groupScale = 1;
  let trailDirty = true;
  let pathDotsDirty = true;
  const surf = (t: Th, off: number): V3 => {
    const p = morphPoint(t[0], t[1], ab.a, ab.b, R_T, r_T);
    const n = morphNormal(t[0], t[1], ab.a, ab.b, R_T, r_T);
    return [p[0] + n[0] * off, p[1] + n[1] * off, p[2] + n[2] * off];
  };
  const rebuildSurface = () => {
    ab = glueStages(glue);
    let xmin = Infinity, xmax = -Infinity, ymin = Infinity, ymax = -Infinity, zmin = Infinity, zmax = -Infinity;
    for (let i = 0; i <= NU; i++) {
      const t1 = -PI + (TWO_PI * i) / NU;
      for (let j = 0; j <= NV; j++) {
        const t2 = -PI + (TWO_PI * j) / NV;
        const k = i * (NV + 1) + j;
        const p = morphPoint(t1, t2, ab.a, ab.b, R_T, r_T);
        const n = morphNormal(t1, t2, ab.a, ab.b, R_T, r_T);
        posArr[3 * k] = p[0]; posArr[3 * k + 1] = p[1]; posArr[3 * k + 2] = p[2];
        nrmArr[3 * k] = n[0]; nrmArr[3 * k + 1] = n[1]; nrmArr[3 * k + 2] = n[2];
        if (p[0] < xmin) xmin = p[0]; if (p[0] > xmax) xmax = p[0];
        if (p[1] < ymin) ymin = p[1]; if (p[1] > ymax) ymax = p[1];
        if (p[2] < zmin) zmin = p[2]; if (p[2] > zmax) zmax = p[2];
      }
    }
    geo.attributes.position.needsUpdate = true;
    geo.attributes.normal.needsUpdate = true;
    geo.computeBoundingSphere();
    // keep the shape centred and scale the long flat sheet / cylinder down so it fits the view
    const extent = Math.max(xmax - xmin, ymax - ymin, zmax - zmin);
    groupScale = Math.min(1, 2.7 / extent);
    torusGroup.scale.setScalar(groupScale);
    torusGroup.position.set(-((xmin + xmax) / 2) * groupScale, -((ymin + ymax) / 2) * groupScale, -((zmin + zmax) / 2) * groupScale);
    // θ1 label: pulled inward from the sheet's end while the ring is open (it would be clipped at the view edge);
    // θ2 label: moves from the top edge of the sheet into the hole of the torus (off its own purple band)
    const lbl1 = surf([PI - 0.6 * (1 - ab.b), 0.5 * PI], 0.32);
    const lbl2 = surf([0.5 * PI, PI], 0.22 + 0.3 * ab.b);
    seamLabels[0].position.set(lbl1[0], lbl1[1], lbl1[2]);
    seamLabels[1].position.set(lbl2[0], lbl2[1], lbl2[2]);
    seamLabels.forEach((s, k) => s.scale.copy(labelBase[k]).multiplyScalar(1 / groupScale));
    pathDotsDirty = true;
    trailDirty = true;
    torusOverlay.textContent = `② 環面　${glueLong(glue)}`;
  };

  const setDots = (pts3: THREE.Points, pts: Th[], max: number) => {
    const n = Math.min(pts.length, max);
    const start = pts.length - n;
    const attr = pts3.geometry.attributes.position as THREE.BufferAttribute;
    const arr = attr.array as Float32Array;
    for (let i = 0; i < n; i++) {
      const p = surf(pts[start + i], 0.02);
      arr[3 * i] = p[0]; arr[3 * i + 1] = p[1]; arr[3 * i + 2] = p[2];
    }
    attr.needsUpdate = true;
    pts3.geometry.setDrawRange(0, n);
  };
  // 3D renders are on demand and capped at ~30 per second (a trailing render keeps the last state)
  let last3D = 0;
  let pending3D = 0;
  const render3D = () => {
    if (destroyed) return;
    const now = performance.now();
    if (now - last3D >= 33) { last3D = now; scene.requestRender(); return; }
    if (!pending3D) pending3D = window.setTimeout(() => { pending3D = 0; if (destroyed) return; last3D = performance.now(); scene.requestRender(); }, 34 - (now - last3D));
  };
  const placeMarkers = () => {
    const p = surf(th, 0.04);
    cur.solid.position.set(p[0], p[1], p[2]);
    cur.xray.position.copy(cur.solid.position);
    const o = otherElbow(L1, L2, th);
    const q = surf(o.th, 0.035);
    ghost.solid.position.set(q[0], q[1], q[2]);
    ghost.xray.position.copy(ghost.solid.position);
    const showGhost = !o.atOrigin && torusDist(o.th, th) > 1e-6;
    ghost.solid.visible = showGhost;
    ghost.xray.visible = showGhost;
    if (trailDirty) { setDots(trailDots, trail.filter((p): p is Th => p !== null), TRAIL_MAX); trailDirty = false; }
    if (pathDotsDirty) { setDots(pathDotsMesh, pathDots, PATH_DOTS_MAX); pathDotsDirty = false; }
    render3D();
  };

  // ---------------- camera aiming (spherical coordinates about the origin, tweened in the loop)
  const CAM_DIST = 4.7;
  let camTween: { from: V3; to: V3; t: number; dur: number } | null = null;
  const camSph = (): V3 => {
    const v = scene.camera.position;
    const d = v.length() || CAM_DIST;
    return [Math.atan2(v.y, v.x), Math.asin(Math.max(-1, Math.min(1, v.z / d))), d];
  };
  const setCamSph = (s: V3) => {
    const [az, elv, d] = s;
    scene.camera.position.set(d * Math.cos(elv) * Math.cos(az), d * Math.cos(elv) * Math.sin(az), d * Math.sin(elv));
    scene.controls?.target.set(0, 0, 0);
    scene.controls?.update();
    render3D();
  };
  const aimCamera = (az: number, elv: number, dur = 0.8) => {
    const from = camSph();
    const to: V3 = [from[0] + wrapToPi(az - from[0]), elv, CAM_DIST];
    if (dur <= 0) { camTween = null; setCamSph(to); } else { camTween = { from, to, t: 0, dur }; kick(); }
  };
  const circMean = (a: number, b: number) => Math.atan2(Math.sin(a) + Math.sin(b), Math.cos(a) + Math.cos(b));

  // ---------------- 3D picking: drag the blue marker, click the surface to jump; other drags orbit
  const raycaster = new THREE.Raycaster();
  const ndc = new THREE.Vector2();
  const setNDC = (ev: PointerEvent) => {
    const rect = scene.canvas.getBoundingClientRect();
    ndc.set(((ev.clientX - rect.left) / rect.width) * 2 - 1, -(((ev.clientY - rect.top) / rect.height) * 2 - 1));
    raycaster.setFromCamera(ndc, scene.camera);
  };
  const pickTheta = (ev: PointerEvent): Th | null => {
    setNDC(ev);
    const hit = raycaster.intersectObject(torusMesh, false)[0];
    if (!hit || !hit.uv) return null;
    return wrapTh([hit.uv.x * TWO_PI - PI, hit.uv.y * TWO_PI - PI]);
  };
  const nearMarker = (ev: PointerEvent): boolean => {
    const wp = cur.solid.getWorldPosition(new THREE.Vector3());
    const pr = wp.clone().project(scene.camera);
    const rect = scene.canvas.getBoundingClientRect();
    const sx = rect.left + ((pr.x + 1) / 2) * rect.width;
    const sy = rect.top + ((1 - pr.y) / 2) * rect.height;
    if (Math.hypot(ev.clientX - sx, ev.clientY - sy) > 22) return false;
    // ignore the marker while it is hidden behind the surface
    setNDC(ev);
    const hit = raycaster.intersectObject(torusMesh, false)[0];
    return !hit || hit.distance > scene.camera.position.distanceTo(wp) - 0.12 * groupScale;
  };
  let drag3D = false;
  let downAt: { x: number; y: number; t: number } | null = null;
  const onDown3D = (ev: PointerEvent) => {
    if (ev.button !== 0) return;
    if (nearMarker(ev)) {
      drag3D = true;
      stopPlay();
      camTween = null;
      ev.stopPropagation(); ev.preventDefault();
      try { scene.el.setPointerCapture(ev.pointerId); } catch { /* ignore */ }
      scene.canvas.style.cursor = 'grabbing';
      return;
    }
    camTween = null;
    downAt = { x: ev.clientX, y: ev.clientY, t: performance.now() };
  };
  const onMove3D = (ev: PointerEvent) => {
    if (!drag3D) {
      if (ev.buttons === 0) scene.canvas.style.cursor = nearMarker(ev) ? 'grab' : '';
      return;
    }
    ev.stopPropagation();
    const t = pickTheta(ev);
    if (t) setTheta(t, 'torus');
  };
  const onUp3D = (ev: PointerEvent) => {
    if (drag3D) {
      drag3D = false;
      ev.stopPropagation();
      try { scene.el.releasePointerCapture(ev.pointerId); } catch { /* ignore */ }
      scene.canvas.style.cursor = '';
      return;
    }
    if (downAt && Math.hypot(ev.clientX - downAt.x, ev.clientY - downAt.y) < 5 && performance.now() - downAt.t < 450) {
      const t = pickTheta(ev);
      if (t) { stopPlay(); setTheta(t, 'torus', true); }
    }
    downAt = null;
  };
  scene.el.addEventListener('pointerdown', onDown3D, true);
  scene.el.addEventListener('pointermove', onMove3D, true);
  scene.el.addEventListener('pointerup', onUp3D, true);

  // ---------------- state updates
  /** `jump`: the configuration is set, not moved (preset start, other elbow, click on the torus): break the trail. */
  function setTheta(t: Th, source: 'slider' | 'square' | 'torus' | 'arm' | 'button' | 'path', jump = false): void {
    const w = wrapTh(t);
    if (pushTrail(trail, w, { jump, minStep: TRAIL_STEP, max: TRAIL_MAX })) trailDirty = true;
    th = w;
    if (source !== 'slider') { sT1.set(th[0], { silent: true }); sT2.set(th[1], { silent: true }); }
    refresh();
    kick();
  }

  function setGlue(g: number): void {
    glue = Math.min(2, Math.max(0, g));
    sGlue.set(glue, { silent: true });
    rebuildSurface();
    refresh();
  }

  function startGlueAnim(): void {
    glueAnim = { t: 0, dur: 3.8, camMoved: false };
    kick();
    setGlue(0);
    aimCamera(0, 0.22, 0.5);          // face the flat sheet first
  }

  function setPath(p: Th[] | null): void {
    path = p && p.length > 1 ? densify(p, 0.03) : null;
    pathDots = [];
    if (path) {
      const L = pathLength(path);
      const n = Math.min(PATH_DOTS_MAX, Math.max(2, Math.ceil(L / 0.035) + 1));
      for (let i = 0; i < n; i++) pathDots.push(samplePath(path, i / (n - 1)));
    }
    pathDotsDirty = true;
    pathInfo.textContent = path ? `長度 ${fmt(pathLength(path), 2)} rad` : '';
    refresh();
  }

  function loadPreset(i: number, play: boolean): void {
    setDrawMode(false);
    stopPlay();
    setPath(PATH_PRESETS[i].pts);
    if (path) {
      setTheta(path[0], 'path', true);
      // turn the view so the interesting part of the path faces the camera (e.g. the θ1 = ±π crossing)
      const cam = PATH_PRESETS[i].cam;
      aimCamera(cam ? cam.az : wrapToPi(samplePath(path, 0.5)[0]), cam ? cam.el : 0.5);
    }
    if (play) startPlay();
  }

  function startPlay(): void {
    if (!path) { chips.activate(0); return; } // activating a preset loads it and calls startPlay again
    if (playS == null || playS >= 1) { playS = 0; setTheta(path[0], 'path', true); }
    playBtn.textContent = '⏸ 暫停路徑';
    kick();
  }
  function stopPlay(): void {
    if (playS != null && playS < 1) playS = null;
    sweep = null;
    playBtn.textContent = '▶ 播放路徑';
  }

  /** Step demo: move the configuration along an (unwrapped) polyline; it leaves a trail but no path. */
  function startSweep(pts: Th[], dur: number): void {
    stopPlay();
    const dense = densify(pts, 0.02);
    setTheta(dense[0], 'path', true);
    sweep = { pts: dense, s: 0, dur };
    kick();
  }

  /** Put the seam-crossing demo trail in place at once (used when a step needs it but step 2 was skipped). */
  function seedSeamTrail(): void {
    stopPlay();
    const pts = densify(SEAM_SWEEP, TRAIL_STEP * 1.5);
    trail = pts.map((p) => wrapTh(p));
    trailDirty = true;
    setTheta(pts[pts.length - 1], 'path');
  }
  function togglePlay(): void {
    if (playS != null && playS < 1) stopPlay(); else startPlay();
  }

  function setDrawMode(on: boolean): void {
    drawMode = on;
    drawing = null;
    cfgHandle.enabled = !on;
    drawBtn.textContent = on ? '✕ 取消畫路徑' : '✎ 畫路徑';
    drawBtn.classList.toggle('primary', on);
    if (on) { stopPlay(); chips.clear(); }
    sq.requestRender();
  }

  // ---------------- ① square: drawing + interaction
  const cfgHandle = sq.addHandle({
    get: () => [th[0], th[1]],
    set: (p) => setTheta([p[0], p[1]], 'square'),
    onStart: () => stopPlay(),
    radius: 8,
    color: '--accent',
  });
  sq.onPointer({
    down: (p) => {
      if (!drawMode) return;
      drawing = [];
      appendUnwrapped(drawing, wrapTh([p[0], p[1]]));
      sq.requestRender();
    },
    move: (p) => {
      if (!drawMode || !drawing) return;
      const last = drawing[drawing.length - 1];
      const nxt = wrapTh([p[0], p[1]]);
      if (torusDist(nxt, last) > 0.03) { appendUnwrapped(drawing, nxt); sq.requestRender(); }
    },
    up: () => {
      if (!drawMode || !drawing) return;
      const pts = drawing;
      setDrawMode(false);
      if (pts.length > 2 && pathLength(pts) > 0.2) {
        setPath(pts);
        setTheta(pts[0], 'path', true);
        if (path) aimCamera(wrapToPi(samplePath(path, 0.5)[0]), 0.5);
        startPlay();
      }
    },
  });

  const tileOffsets: Pt[] = [];
  for (let i = -1; i <= 1; i++) for (let j = -1; j <= 1; j++) if (i || j) tileOffsets.push([i * TWO_PI, j * TWO_PI]);

  const drawWrapped = (c: Canvas2D, pieces: Th[][], color: string, width: number, alpha = 1) => {
    for (const piece of pieces) {
      c.polyline(piece, { color, width, alpha });
      for (const [ox, oy] of tileOffsets) c.polyline(piece.map((p) => [p[0] + ox, p[1] + oy] as Pt), { color, width: Math.max(1, width - 1), alpha: alpha * 0.3 });
    }
  };

  sq.draw = (c) => {
    const accent = cssVar('--accent');
    const muted = cssVar('--fg-muted');
    const seam1 = cssVar('--series-4');
    const seam2 = cssVar('--series-5');
    const bg = cssVar('--sim-bg');
    for (const [ox, oy] of tileOffsets) c.image(mapCanvas, { xmin: -PI + ox, xmax: PI + ox, ymin: -PI + oy, ymax: PI + oy }, { alpha: 0.3, smooth: true });
    c.image(mapCanvas, { xmin: -PI, xmax: PI, ymin: -PI, ymax: PI }, { smooth: true });
    // seams + gluing marks (single arrows: θ1 edges, double arrows: θ2 edges)
    c.line([-PI, -PI], [-PI, PI], { color: seam1, width: 3 });
    c.line([PI, -PI], [PI, PI], { color: seam1, width: 3 });
    c.line([-PI, PI], [PI, PI], { color: seam2, width: 3 });
    c.line([-PI, -PI], [PI, -PI], { color: seam2, width: 3 });
    for (const x of [-PI, PI]) c.arrow([x, -0.45], [x, 0.45], { color: seam1, width: 2.5, head: 11 });
    for (const y of [-PI, PI]) {
      c.arrow([-0.75, y], [0.05, y], { color: seam2, width: 2.5, head: 11 });
      c.arrow([-0.15, y], [0.65, y], { color: seam2, width: 2.5, head: 11 });
    }
    const lbl = { bg, font: `11px ${cssVar('--font')}`, color: muted };
    c.text([-PI, -PI], '−π', { ...lbl, dy: 13 });
    c.text([PI, -PI], 'π', { ...lbl, dy: 13 });
    c.text([1.6, -PI], 'θ₁ →', { ...lbl, dy: 13 });
    c.text([-PI, PI], 'π', { ...lbl, dx: -14 });
    c.text([-PI, -1.6], 'θ₂ ↑', { ...lbl, dx: -19 });
    for (const piece of trailPieces(trail)) drawWrapped(c, wrapPolylines(unwrapSeq(piece)), cssVar('--series-6'), 2, 0.9);
    if (path) drawWrapped(c, wrapPolylines(path), cssVar('--series-7'), 2.6);
    if (drawing && drawing.length > 1) drawWrapped(c, wrapPolylines(drawing), cssVar('--series-7'), 2.6);
    const o = otherElbow(L1, L2, th);
    if (!o.atOrigin && torusDist(o.th, th) > 1e-6) {
      c.point(o.th, 7, { color: bg });
      c.circle(o.th, 7 / c.scale, { color: muted, width: 2.5 });
      if (stepNow === 0) c.text(o.th, '另一組解', { color: muted, dy: -16, bg });
    }
    for (const [ox, oy] of tileOffsets) c.point([th[0] + ox, th[1] + oy], 5, { color: accent, alpha: 0.35 });
    if (stepNow === 0) c.text(th, '目前組態', { color: accent, dy: -18, bg });
    c.overlay.textContent = drawMode
      ? '① 方格\n✎ 按住拖曳畫路徑（可畫進外圍＝越過邊界）'
      : `① 方格（顯式座標）\nθ = (${fmt((th[0] * 180) / PI, 0)}°, ${fmt((th[1] * 180) / PI, 0)}°)`;
  };

  // ---------------- ③ arm: drawing + interaction
  arm.addHandle({
    get: () => { const p = planarFK([L1, L2], th).tip; return [p[0], p[1]]; },
    set: (p) => {
      const res = ikNearest(L1, L2, [p[0], p[1]], th);
      armNote = res.singular ? 'singular' : res.clamped ? 'clamped' : '';
      setTheta(res.th, 'arm');
    },
    onStart: () => stopPlay(),
    onEnd: () => { armNote = ''; arm.requestRender(); },
    radius: 9,
    color: '--accent',
  });
  arm.draw = (c) => {
    const accent = cssVar('--accent');
    c.grid(0.5);
    const ro = L1 + L2;
    const ri = Math.abs(L1 - L2);
    c.circle([0, 0], ro, { fill: cssVar('--accent-soft'), color: accent, width: 1.2, dash: [5, 4] });
    if (ri > 1e-3) c.circle([0, 0], ri, { fill: cssVar('--sim-bg'), color: accent, width: 1.2, dash: [5, 4] });
    c.text([ro * 0.7, -ro * 0.72], 'workspace', { color: accent, dy: 10, font: `11px ${cssVar('--font')}` });
    c.frame({ x: 0, y: 0, theta: 0 }, 0.45, { labels: 's', width: 1.6 });
    if (path) c.polyline(path.map((q) => planarFK([L1, L2], q).tip as Pt), { color: cssVar('--series-7'), width: 2.4 });
    for (const piece of trailPieces(trail)) c.polyline(piece.map((q) => planarFK([L1, L2], q).tip as Pt), { color: cssVar('--series-6'), width: 1.8, alpha: 0.85 });
    const o = otherElbow(L1, L2, th);
    if (!o.atOrigin && torusDist(o.th, th) > 1e-6) drawPlanarArm(c, [L1, L2], o.th, { alpha: 0.42, eeRadius: 4, linkWidth: 7 });
    drawPlanarArm(c, [L1, L2], th, { angleArcs: true, linkWidth: 8 });
    const tip = planarFK([L1, L2], th).tip;
    let msg = `③ 手臂／workspace\np = (${fmt(tip[0], 2)}, ${fmt(tip[1], 2)})  |p| = ${fmt(Math.hypot(tip[0], tip[1]), 2)}`;
    if (o.atOrigin || armNote === 'singular') msg += '\n末端在原點：θ₁ 任意 → 無限多組解（奇異）';
    else if (o.coincident) msg += '\n在 workspace 邊界：兩組解重合';
    else if (armNote === 'clamped') msg += '\n目標超出可達範圍 → 投影到邊界';
    c.overlay.textContent = msg;
  };

  // ---------------- gallery (last step)
  const drawCard = (c: Canvas2D, i: number) => {
    const x0 = i % 2;
    const y0 = i < 2 ? 1 : 0;
    const fg = cssVar('--fg');
    const muted = cssVar('--fg-muted');
    const accent = cssVar('--accent');
    const font = cssVar('--font');
    const sel = i === cardPick;
    c.polygon([[x0 + 0.03, y0 + 0.03], [x0 + 0.97, y0 + 0.03], [x0 + 0.97, y0 + 0.97], [x0 + 0.03, y0 + 0.97]], {
      fill: sel ? cssVar('--accent-soft') : cssVar('--bg-elev'), color: sel ? accent : cssVar('--border'), width: sel ? 2.5 : 1,
    });
    c.text([x0 + 0.5, y0 + 0.86], CARDS[i].title, { color: fg, font: `600 12px ${font}` });
    c.text([x0 + 0.5, y0 + 0.11], CARDS[i].space, { color: accent, font: `12px ${font}` });
    const cx = x0 + 0.5;
    const cy = y0 + 0.48;
    if (i === 0) {
      const ph = 0.45;
      const cs = Math.cos(ph), sn = Math.sin(ph);
      const box: Pt[] = [[-0.17, -0.1], [0.17, -0.1], [0.17, 0.1], [-0.17, 0.1]].map(([u, v]) => [cx - 0.08 + cs * u - sn * v, cy - 0.04 + sn * u + cs * v] as Pt);
      c.polygon(box, { fill: cssVar('--grid-strong'), color: muted, width: 1.5 });
      c.frame({ x: cx - 0.08, y: cy - 0.04, theta: ph }, 0.2, { width: 2 });
      c.arc([cx - 0.08, cy - 0.04], 0.12, 0, ph, { color: cssVar('--warn'), width: 1.5 });
      c.text([cx + 0.12, cy - 0.2], '(x, y, φ)', { color: muted, font: `11px ${font}` });
    } else if (i === 1) {
      const piv: Pt = [cx, cy + 0.2];
      c.circle(piv, 0.27, { color: muted, width: 1.2, dash: [4, 3] });
      c.ellipse(piv, 0.27, 0.07, 0, { color: muted, width: 1, dash: [3, 3] });
      const bob: Pt = [piv[0] + 0.15, piv[1] - 0.22];
      c.line(piv, bob, { color: fg, width: 2 });
      c.disk(bob, 0.04, { fill: accent });
      c.point(piv, 3, { color: fg });
    } else if (i === 2) {
      const ky = cy + 0.04;
      c.line([x0 + 0.1, ky], [x0 + 0.9, ky], { color: muted, width: 3 });
      c.polygon([[cx - 0.07, ky - 0.12], [cx + 0.07, ky - 0.12], [cx + 0.07, ky + 0.12], [cx - 0.07, ky + 0.12]], { fill: cssVar('--grid-strong'), color: fg, width: 1.5 });
      c.arrow([cx + 0.13, ky + 0.2], [cx + 0.33, ky + 0.2], { color: cssVar('--warn'), width: 2, head: 8 });
      c.arrow([cx - 0.13, ky + 0.2], [cx - 0.33, ky + 0.2], { color: cssVar('--warn'), width: 2, head: 8 });
      c.ellipse([cx, ky], 0.045, 0.17, 0, { color: cssVar('--series-5'), width: 1.5, dash: [3, 2] });
      c.text([cx, ky - 0.27], '滑動 ℝ ＋ 旋轉 S¹', { color: muted, font: `11px ${font}` });
    } else {
      // teacup ride driven by the current (θ1, θ2)
      c.circle([cx, cy], 0.27, { color: muted, width: 1.5, fill: cssVar('--bg-sunken') });
      const cup: Pt = [cx + 0.17 * Math.cos(th[0]), cy + 0.17 * Math.sin(th[0])];
      c.line([cx, cy], cup, { color: muted, width: 1.5, dash: [3, 3] });
      c.circle(cup, 0.08, { color: fg, width: 1.5, fill: cssVar('--bg-elev') });
      const hd = th[0] + th[1];
      c.line([cup[0] + 0.08 * Math.cos(hd), cup[1] + 0.08 * Math.sin(hd)], [cup[0] + 0.13 * Math.cos(hd), cup[1] + 0.13 * Math.sin(hd)], { color: accent, width: 4 });
      c.point([cx, cy], 3, { color: fg });
    }
  };
  gal.draw = (c) => { for (let i = 0; i < 4; i++) drawCard(c, i); };
  const showCard = () => cardBox.replaceChildren(mdElement(CARDS[cardPick].md));
  gal.onPointer({
    down: (p) => {
      cardPick = (p[1] >= 1 ? 0 : 2) + (p[0] >= 1 ? 1 : 0);
      showCard();
      gal.requestRender();
    },
  });
  showCard();

  function setGallery(on: boolean): void {
    showGallery = on;
    arm.el.style.display = on ? 'none' : '';
    gal.el.style.display = on ? '' : 'none';
    infoBox.style.display = on ? 'none' : '';
    cardBox.style.display = on ? '' : 'none';
    if (on) gal.requestRender(); else arm.requestRender();
  }

  // ---------------- ④ numbers
  function buildKV(): void {
    kvBox.replaceChildren();
    numKV = rep === 'explicit'
      ? kv(kvBox, ['θ₁', 'θ₂', '數 / 約束', '代價'])
      : kv(kvBox, ['(c₁, s₁)', '(c₂, s₂)', 'c₁²+s₁²−1', 'c₂²+s₂²−1', '數 / 約束', '代價']);
  }
  buildKV();
  const setRep = (v: 'explicit' | 'implicit') => {
    if (v === rep) return;
    rep = v;
    repSel.set(v, { silent: true });
    buildKV();
    refresh();
  };
  const updateNumbers = () => {
    if (rep === 'explicit') {
      numKV.set('θ₁', `${fmt(th[0], 3)} rad（${fmt((th[0] * 180) / PI, 0)}°）`);
      numKV.set('θ₂', `${fmt(th[1], 3)} rad（${fmt((th[1] * 180) / PI, 0)}°）`);
      numKV.set('數 / 約束', '2 個數、0 條 → 2 DOF');
      numKV.set('代價', '在 ±π 要包裹，數值會跳');
    } else {
      const ic = implicitCoords(th);
      numKV.set('(c₁, s₁)', `(${fmt(ic.c1, 3)}, ${fmt(ic.s1, 3)})`);
      numKV.set('(c₂, s₂)', `(${fmt(ic.c2, 3)}, ${fmt(ic.s2, 3)})`);
      numKV.set('c₁²+s₁²−1', fmt(ic.g1, 6));
      numKV.set('c₂²+s₂²−1', fmt(ic.g2, 6));
      numKV.set('數 / 約束', '4 個數、2 條 → 2 DOF');
      numKV.set('代價', '多 2 個數，須維持約束');
    }
  };

  const refresh = rafThrottle(() => {
    if (destroyed) return;
    sq.requestRender();
    if (showGallery) gal.requestRender(); else arm.requestRender();
    placeMarkers();
    updateNumbers();
    if (path && playS != null) pathInfo.textContent = `長度 ${fmt(pathLength(path), 2)} rad · ${Math.round(Math.min(1, playS) * 100)}%`;
  });

  // ---------------- animation loop: glue morph, camera tween, path playback, plot sampling
  let tNow = 0;
  let lastPush = -1;
  const loop = createLoop((dt) => {
    tNow += dt;
    if (glueAnim) {
      glueAnim.t += dt;
      const u = Math.min(1, glueAnim.t / glueAnim.dur);
      setGlue(2 * u);
      if (!glueAnim.camMoved && u > 0.45) {
        // raise the camera so the far side, where the left/right edges meet, stays visible
        glueAnim.camMoved = true;
        aimCamera(-0.75, 0.66, 1.6);
      }
      if (u >= 1) glueAnim = null;
    }
    if (camTween) {
      camTween.t += dt;
      const u = Math.min(1, camTween.t / camTween.dur);
      const e = u * u * (3 - 2 * u);
      const f = camTween.from, g = camTween.to;
      setCamSph([f[0] + (g[0] - f[0]) * e, f[1] + (g[1] - f[1]) * e, f[2] + (g[2] - f[2]) * e]);
      if (u >= 1) camTween = null;
    }
    if (path && playS != null && playS < 1) {
      const dur = Math.max(3, pathLength(path) / 1.4);
      playS = Math.min(1, playS + dt / dur);
      setTheta(samplePath(path, playS), 'path');
      if (playS >= 1) playBtn.textContent = '▶ 播放路徑';
    }
    if (sweep) {
      sweep.s = Math.min(1, sweep.s + dt / sweep.dur);
      setTheta(samplePath(sweep.pts, sweep.s), 'path');
      if (sweep.s >= 1) sweep = null;
    }
    if (tNow - lastPush >= 1 / 30) {
      plot.push(tNow, [th[0], Math.cos(th[0]), Math.sin(th[0])]);
      lastPush = tNow;
    }
    const busy = !!glueAnim || !!camTween || !!sweep || (path != null && playS != null && playS < 1);
    if (busy) lastActive = performance.now();
    else if (performance.now() - lastActive > 1500) loop.stop();
  });
  loopRef = loop;

  // Materials, sprites, handles, plot series and legend are theme-bound already (CSS variable names);
  // only the painted checkerboard map has to be redrawn.
  const unsubTheme = ctx.onThemeChange(() => {
    paintMaps();
    texture.needsUpdate = true;
    refresh();
    render3D();
  });

  rebuildSurface();
  refresh();

  // ---------------- per-step configuration
  let focused: HTMLElement[] = [];
  const focusOn = (els: HTMLElement[]) => {
    focused.forEach((f) => f.classList.remove('mr-focus'));
    focused = els;
    focused.forEach((f) => f.classList.add('mr-focus'));
  };
  let first = true;
  const onStep = (i: number) => {
    stepNow = i;
    const dur = first ? 0 : 0.8;
    first = false;
    if (i !== 2) glueAnim = null;
    setGallery(i === 5);
    if (i === 0) {
      setGlue(2);
      const o = otherElbow(L1, L2, th);
      aimCamera(circMean(th[0], o.th[0]), 0.5, dur);
      focusOn([arm.el, sq.el, scene.el]);
    } else if (i === 1) {
      // explicit coordinates: replay a short move across the θ1 = ±π seam so the split trail is visible at once
      setGlue(0);
      aimCamera(0, 0.22, dur);
      setRep('explicit');
      trail = [];
      trailDirty = true;
      startSweep(SEAM_SWEEP, 1.5);
      focusOn([sq.el]);
    } else if (i === 2) {
      // gluing: the seam-crossing trail from the previous step becomes one unbroken curve
      if (trail.length < 5) seedSeamTrail();
      startGlueAnim();
      focusOn([scene.el, sGlue.el, glueBtn]);
    } else if (i === 3) {
      // representations: one full turn of joint 1 makes θ1 jump once while cos θ1, sin θ1 stay smooth
      setGlue(2);
      setRep('implicit');
      startSweep([[th[0], th[1]], [th[0] + TWO_PI, th[1]]], 4.5);
      focusOn([repSel.el, plot.el]);
    } else if (i === 4) {
      setGlue(2);
      if (!path) chips.activate(0);
      focusOn([chipRow, drawBtn, playBtn]);
    } else {
      setGlue(2);
      focusOn([gal.el, cardBox]);
    }
    refresh();
    kick();
  };
  onStep(ctx.currentStep);
  kick(); // draw a few plot samples so the first view is not empty

  return {
    onStep,
    destroy() {
      destroyed = true;
      loop.stop();
      unsubTheme();
      scene.el.removeEventListener('pointerdown', onDown3D, true);
      scene.el.removeEventListener('pointermove', onMove3D, true);
      scene.el.removeEventListener('pointerup', onUp3D, true);
      clearTimeout(pending3D);
      focusOn([]);
      dotTex.dispose();
      for (const s of seamLabels) s.material.map?.dispose();
      texture.dispose();
      sq.destroy();
      arm.destroy();
      gal.destroy();
      plot.destroy();
      scene.destroy();
    },
  };
}

const lesson: Lesson = { meta, steps, quiz, mount };
export default lesson;
