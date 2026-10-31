import type { Lesson, LessonContext, LessonHandle } from '@/types';
import {
  el, cssVar, fmt, themeColor, controlPanel, row, slider, toggle, button, select, kv, note, simTitle, legend, presetChips,
  algorithmStepper, mdElement, Canvas2D, Plot2D, RasterLayer, createLoop,
  type Preset, type RGBA,
} from '@/ui';
import { meta } from './meta';
import { quiz, steps } from './content';
import {
  CSpace, GridIndex, PRM, RRT, RRTConnect, coverage, mulberry32, pathLength, pointAlong, prmQuery, prmTrial, rrtTrial, shortcut,
  type CheckPoint, type Proposal, type Q, type QueryResult,
} from './planners';
import {
  ARM_L, ARM_OBSTACLES, ARM_W, FLOOR, NARROW, TORUS, armFree, armPoints, makeMap, makeSpace, obstacleDistance, segmentHits,
  type MapId, type PlanMap,
} from './maps';

// ---------------------------------------------------------------------------------------------
// Simulation
// ---------------------------------------------------------------------------------------------

type Algo = 'demo' | 'rrt' | 'naive' | 'connect' | 'prm';
const ALGO_LABEL: Record<Algo, string> = {
  demo: '入門：隨機取樣＋連線',
  rrt: 'RRT',
  naive: '對照：隨機節點擴張',
  connect: 'RRT-Connect（雙向）',
  prm: 'PRM（路網）',
};
const MAP_IDS: MapId[] = ['clutter', 'empty', 'wall', 'maze', 'narrow', 'arm'];
const MAP_LABEL: Record<MapId, string> = { clutter: '散落障礙', empty: '空地', wall: '一道牆', maze: '迷宮', narrow: '窄通道', arm: '2R 手臂 C-space' };
const STAGES = ['① sample：取樣 q_rand', '② nearest：找 q_near', '③ steer：走一步 δ 得 q_new', '④ check：檢查線段後加入'];
const MAX_NODES = 4000;
const MAX_ITERS = 4000;
const DEMO_R = 1.8;
const DEMO_K = 3;
const DEMO_MAX = 400;
const EPS_ARM = 0.02;
const EXP_XS = [25, 50, 100, 200, 400, 800];
const EXP_SEEDS = 20;
const EMPTY_GOAL: Q = [10.8, 7.8];
const QUERIES: [Q, Q][] = [[[1.0, 0.9], [11.0, 8.1]], [[1.0, 8.2], [11.4, 4.6]], [[4.2, 3.4], [7.2, 8.4]]];
const PALETTE = ['--series-6', '--series-1', '--series-4', '--series-5', '--series-7', '--series-3', '--series-2', '--series-8'];
const TAU = 2 * Math.PI;

interface DemoEdge { a: Q; b: Q; ok: boolean; missed: boolean; checks: CheckPoint[] }
interface DemoState {
  rng: () => number;
  samples: { q: Q; free: boolean }[];
  nodes: Q[];
  index: GridIndex;
  edges: DemoEdge[];
  last: DemoEdge[];
  lastSample: Q | null;
  missed: number;
}

/** Resolve a theme colour (CSS variable name) into RGBA bytes for raster layers. */
function rgbaOf(name: string, alpha: number): RGBA {
  const s = themeColor(name, '#888888').trim();
  let rgb: [number, number, number] = [136, 136, 136];
  const m6 = /^#([0-9a-f]{6})$/i.exec(s);
  const m3 = /^#([0-9a-f]{3})$/i.exec(s);
  const mf = /^rgba?\(([^)]+)\)/i.exec(s);
  if (m6) { const v = parseInt(m6[1], 16); rgb = [(v >> 16) & 255, (v >> 8) & 255, v & 255]; }
  else if (m3) rgb = [0, 1, 2].map((i) => parseInt(m3[1][i] + m3[1][i], 16)) as [number, number, number];
  else if (mf) { const p = mf[1].split(',').map((x) => parseFloat(x)); if (p.length >= 3 && p.slice(0, 3).every(Number.isFinite)) rgb = [p[0], p[1], p[2]]; }
  return [rgb[0], rgb[1], rgb[2], Math.round(alpha * 255)];
}

const deg = (r: number): string => `${fmt((r * 180) / Math.PI, 0)}°`;

function mount(host: HTMLElement, ctx: LessonContext): LessonHandle {
  // ------------------------------------------------------------------ parameters & state
  let mapId: MapId = 'clutter';
  let gap = 0.6;
  let map: PlanMap = makeMap(mapId, gap);
  let start: Q = [...map.start];
  let goal: Q | null = map.goal ? [...map.goal] : null;
  let algo: Algo = 'rrt';
  let deltaFloor = 0.6;
  let deltaArm = 0.25;
  let pGoal = 0.05;
  let prmN = 150;
  let rFloor = 1.5;
  let rArm = 0.7;
  let epsFloor = 0.05;
  let seed = 1;
  let stageMode = false;
  let showVor = false;
  let hideObs = false;
  let torusOn = true;
  let space: CSpace = makeSpace(map, epsFloor);

  let rrt: RRT | null = null;
  let con: RRTConnect | null = null;
  let prm: PRM | null = null;
  let demo: DemoState | null = null;
  let pending: Proposal | null = null;
  let stage = 0;
  let lastProp: Proposal | null = null;
  let qres: QueryResult | null = null;
  let smooth: Q[] | null = null;
  let hoverQ: Q | null = null;
  let playQ: Q | null = null;
  let playU = 0;
  let curStep = -1;
  let destroyed = false;

  const isArm = (): boolean => mapId === 'arm';
  const delta = (): number => (isArm() ? deltaArm : deltaFloor);
  const radius = (): number => (isArm() ? rArm : rFloor);
  const unit = (): string => (isArm() ? 'rad' : 'm');
  const goalFor = (a: Algo): Q | null => goal ?? (a === 'connect' || a === 'prm' ? EMPTY_GOAL : null);
  const currentPath = (): Q[] | null => rrt?.path ?? con?.path ?? qres?.path ?? null;

  // ------------------------------------------------------------------ layout
  simTitle(host, '取樣式規劃器實驗台', 'RRT / PRM');
  const chips = presetChips(host, [], { label: '預設' });
  // Stage strip for the single-stage RRT mode (DOM, so it never covers the canvas or its handles).
  const stageStrip = el('div', { class: 'mr-chips', style: { margin: '0 0 0.3rem' } }, el('span', { class: 'mr-chips-label' }, '本輪階段'));
  const stagePills = STAGES.map((s) => {
    const pill = el('span', { class: 'mr-chip', style: { cursor: 'default' } }, s);
    stageStrip.appendChild(pill);
    return pill;
  });
  host.appendChild(stageStrip);
  const views = el('div');
  host.appendChild(views);
  const leftCol = el('div');
  const rightCol = el('div');
  views.append(leftCol, rightCol);
  const cv = new Canvas2D(leftCol, { aspect: 4 / 3, world: FLOOR, margin: 0.02, overlay: true });
  const wsCap = el('div', { class: 'mr-note', style: { margin: '0 0 0.25rem' } }, '工作空間：手臂跟著 C-space 裡的點移動（滑過左圖可預覽姿勢）');
  rightCol.appendChild(wsCap);
  const ws = new Canvas2D(rightCol, { aspect: 1, world: { xmin: -2.35, xmax: 2.35, ymin: -2.35, ymax: 2.35 }, margin: 0.02, overlay: true });
  const legendHost = el('div');
  host.appendChild(legendHost);

  // experiment + preview (only in the narrow-passage step)
  const expBox = el('div', { style: { margin: '0.4rem 0' } });
  host.appendChild(expBox);
  const expPlot = new Plot2D(expBox, {
    title: '成功率實驗：每個 N 用 20 個種子', xLabel: 'PRM 節點數 N', yLabel: '成功率', series: [{ name: '—' }],
    xRange: [0, 820], yRange: [0, 1.05], height: 170, legend: true,
  });
  const expRow = el('div', { class: 'mr-playback' });
  expBox.appendChild(expRow);
  const expBtn = button(expRow, { label: '▶ 成功率實驗', small: true, title: '用目前的通道寬度，對每個 N 各跑 20 個種子', onClick: () => startExperiment() });
  button(expRow, { label: '清除曲線', small: true, onClick: () => { cancelExperiment(); expRuns = []; paintExperiment(); } });
  const expStatus = el('span', { class: 'mr-note', style: { margin: 0 } }, '');
  expRow.appendChild(expStatus);

  const transport = el('div');
  host.appendChild(transport);
  const stepper = algorithmStepper(transport, {
    onStep: () => doStep(),
    onReset: () => buildPlanner(),
    intervalMs: 250,
    speeds: [1, 4, 16, 64],
    countLabel: '點擊',
  });
  stepper.el.querySelector('.mr-time')?.remove();
  const speedSel = stepper.el.querySelector('select');
  const extra = el('div', { class: 'mr-playback' });
  transport.appendChild(extra);
  const finishBtn = button(extra, { label: '⏩ 跑到完成', small: true, title: '一次跑到找到路徑或達到上限', onClick: () => finishNow() });
  const smoothBtn = button(extra, { label: '✂ 捷徑平滑', small: true, title: '在路徑上反覆任取兩點，能直線相連就剪掉中間', onClick: () => doSmooth() });
  const playBtn = button(extra, { label: '▶ 播放', small: true, title: '讓手臂沿著路徑移動', onClick: () => (play.running ? stopPlay() : startPlay()) });
  button(extra, { label: '🎲 換種子', small: true, title: '同樣的設定，換一組亂數', onClick: () => { sSeed.set(seed % 99 + 1); } });

  const panel = controlPanel(host, { compact: true });
  const selMap = select(panel, {
    label: '地圖', value: mapId,
    options: MAP_IDS.map((m) => ({ value: m, label: MAP_LABEL[m] })),
    onChange: (v) => { setMap(v); restart(); },
  });
  const selAlgo = select(panel, {
    label: '演算法', value: algo,
    options: (Object.keys(ALGO_LABEL) as Algo[]).map((a) => ({ value: a, label: ALGO_LABEL[a] })),
    onChange: (v) => { setAlgo(v); restart(); },
  });
  const sDelta = slider(panel, {
    label: '步長 δ', min: 0.2, max: 1.5, step: 0.05, value: deltaFloor, format: (v) => `${fmt(v, 2)} ${unit()}`,
    onInput: (v) => { if (isArm()) deltaArm = v; else deltaFloor = v; chips.clear(); restart(); },
  });
  const sPGoal = slider(panel, { label: '目標偏置 p_goal', min: 0, max: 1, step: 0.01, value: pGoal, onInput: (v) => { pGoal = v; chips.clear(); restart(); } });
  const sN = slider(panel, { label: 'PRM 節點數 N', min: 20, max: 600, step: 10, value: prmN, format: (v) => fmt(v, 0), onInput: (v) => { prmN = Math.round(v); chips.clear(); restart(); } });
  const sR = slider(panel, {
    label: '連線半徑 r', min: 0.5, max: 3, step: 0.05, value: rFloor, format: (v) => `${fmt(v, 2)} ${unit()}`,
    onInput: (v) => { if (isArm()) rArm = v; else rFloor = v; chips.clear(); restart(); },
  });
  const sEps = slider(panel, {
    label: '檢查間距 ε', min: 0.02, max: 0.6, step: 0.01, value: epsFloor, format: (v) => `${fmt(v, 2)} m`,
    onInput: (v) => { epsFloor = v; chips.clear(); restart(); },
  });
  const sGap = slider(panel, {
    label: '通道寬度', min: NARROW.minGap, max: NARROW.maxGap, step: 0.05, value: gap, unit: 'm',
    onInput: (v) => { gap = v; chips.clear(); setMap('narrow'); restart(); },
  });
  const sSeed = slider(panel, {
    label: '隨機種子', min: 1, max: 99, step: 1, value: seed, format: (v) => fmt(v, 0),
    // steps that show a finished plan re-plan right away with the new seed (like dragging S / G)
    onInput: (v) => { seed = Math.round(v); restart(); if (curStepCfg()?.pre === 'finish') preRun('finish'); },
  });
  const togRow = row(panel);
  const tStage = toggle(togRow, { label: '逐階段（四個動作分開看）', value: stageMode, onChange: (v) => { stageMode = v; restart(); } });
  const tVor = toggle(togRow, { label: 'Voronoi 區域', value: showVor, onChange: (v) => { showVor = v; vorDirty = true; updateVisibility(); cv.requestRender(); } });
  const tHide = toggle(togRow, { label: '隱藏障礙（規劃器的視角）', value: hideObs, onChange: (v) => { hideObs = v; cv.requestRender(); } });
  const tTorus = toggle(togRow, { label: '環面接縫（±180° 相接）', value: torusOn, onChange: (v) => { torusOn = v; chips.clear(); restart(); } });
  const info = kv(host, ['狀態', '迭代 / 節點', '碰撞查詢次數', '被拒絕的延伸', '涵蓋率', 'q_near 的 Voronoi 面積', '路徑長度', '建圖成本', '本次查詢成本', '入門統計']);
  const footNote = note(host, '');

  const previewBox = el('div', { class: 'card', style: { padding: '0.5rem 0.7rem', margin: '0.5rem 0' } });
  host.appendChild(previewBox);
  previewBox.appendChild(mdElement('**下一課預告：位能場。** 不取樣也不長樹：目標是谷底、障礙是山丘，機器人沿著 $-\\nabla U$ 往下滑。下圖是同一張地圖的位能（對數色階）：藍色的谷底在目標，離目標越遠越高（暖色），障礙周圍再墊起一圈陡峭的山丘。'));
  const pv = new Canvas2D(previewBox, { aspect: 2.4, world: FLOOR, margin: 0.02 });

  // ------------------------------------------------------------------ rasters (theme-aware, repainted on demand)
  let cobs: RasterLayer | null = null;
  let cobsDirty = true;
  let vor: RasterLayer | null = null;
  let vorDirty = true;
  let vorAt = 0;
  let vorTimer = 0;
  let vorCounts: Int32Array | null = null;
  let vorIds: Int32Array | null = null;
  let vorNear = -1;
  /** Number of tree nodes the Voronoi raster was computed for. */
  let vorN = 0;
  let pvRaster: RasterLayer | null = null;
  let pvDirty = true;

  function paintCobs(): void {
    if (!cobs) cobs = new RasterLayer(TORUS, 200, 200);
    const fill = rgbaOf('--fg-muted', 0.5);
    cobs.paint((x, y) => !armFree(ARM_OBSTACLES, [x, y]), { color: fill });
    cobsDirty = false;
  }

  /** Nearest-node labelling of every raster pixel; boundaries and the q_near cell are highlighted. */
  function paintVoronoi(): void {
    const T = rrt?.tree;
    vorDirty = false;
    vorAt = performance.now();
    if (!T) { vor = null; return; }
    const arm = isArm();
    const nx = arm ? 160 : 180, ny = arm ? 160 : 135;
    const b = map.bounds;
    if (!vor || vor.nx !== nx || vor.ny !== ny) vor = new RasterLayer(b, nx, ny);
    vor.world = b;
    // The proposal on screen and its q_near (none before the nearest stage of a staged iteration).
    const shown = stageMode && pending ? pending : lastProp;
    const near = !shown ? -1 : stageMode && pending && stage < 2 ? -1 : shown.near;
    // q_near was picked from the tree as it was *before* q_new joined it: show that tree's Voronoi
    // cells, so q_rand lies in the highlighted cell and its area is the probability of the pick.
    const excludeLast = !!shown && shown.ok === true && T.size > 1 && T.nodes[T.size - 1] === shown.qNew;
    const n = excludeLast ? T.size - 1 : T.size;
    let index = T.index;
    if (excludeLast) {
      const pts = T.nodes.slice(0, n);
      index = new GridIndex(space, pts, Math.max(delta(), Math.min(space.W, space.H) / 48));
      for (let k = 0; k < n; k++) index.insert(k);
    }
    const ids = new Int32Array(nx * ny);
    const counts = new Int32Array(n);
    for (let j = 0; j < ny; j++) {
      for (let i = 0; i < nx; i++) {
        const [x, y] = vor.cell(i, j);
        const id = index.nearest([x, y]);
        ids[j * nx + i] = id;
        counts[id]++;
      }
    }
    vorIds = ids;
    vorCounts = counts;
    vorN = n;
    vorNear = near;
    // pastel cells in every colour except the one reserved for the highlighted q_near cell
    const pal = PALETTE.filter((p) => p !== '--series-7').map((p) => rgbaOf(p, 0.2));
    const edge = rgbaOf('--fg-muted', 0.55);
    const hi = rgbaOf('--series-7', 0.42);
    const codes = new Int32Array(nx * ny);
    for (let j = 0; j < ny; j++) {
      for (let i = 0; i < nx; i++) {
        const k = j * nx + i;
        const id = ids[k];
        const boundary = (i + 1 < nx && ids[k + 1] !== id) || (j + 1 < ny && ids[k + nx] !== id);
        codes[k] = boundary ? -1 : id === near ? -2 : id;
      }
    }
    let k = 0;
    vor.paint(() => codes[k++], { colormap: (v) => (v === -1 ? edge : v === -2 ? hi : pal[((v * 2654435761) >>> 0) % pal.length]), range: [0, 1] });
  }

  function paintPreview(): void {
    pvDirty = false;
    if (isArm()) return;
    if (!pvRaster) pvRaster = new RasterLayer(FLOOR, 144, 108);
    const g = goalFor('prm')!;
    const obs = map.obstacles;
    const lo = rgbaOf('--series-1', 1), hi = rgbaOf('--series-4', 1);
    pvRaster.paint((x, y) => {
      const d = obstacleDistance(obs, [x, y]);
      if (d <= 0) return NaN;
      const att = 0.5 * ((x - g[0]) ** 2 + (y - g[1]) ** 2);
      const rep = d < 1.2 ? 1.5 * (1 / d - 1 / 1.2) ** 2 : 0;
      return Math.log1p(att + rep);
    }, {
      // same theme-aware look as the potential-field lesson: blue valley, amber hills
      colormap: (v) => {
        const t = Math.max(0, Math.min(1, v));
        return t < 0.5 ? [lo[0], lo[1], lo[2], 255 * 0.45 * (1 - 2 * t)] : [hi[0], hi[1], hi[2], 255 * 0.8 * (2 * t - 1)];
      },
      range: [0, 5.2],
    });
  }

  // ------------------------------------------------------------------ planner lifecycle
  function setMap(id: MapId): void {
    const changed = id !== mapId || id === 'narrow';
    mapId = id;
    map = makeMap(id, gap);
    if (changed) { start = [...map.start]; goal = map.goal ? [...map.goal] : null; }
    selMap.set(id, { silent: true });
    const arm = isArm();
    sDelta.setRange(arm ? 0.05 : 0.2, arm ? 0.6 : 1.5, arm ? 0.01 : 0.05);
    sDelta.set(delta(), { silent: true });
    sR.setRange(arm ? 0.2 : 0.5, arm ? 1.5 : 3, arm ? 0.05 : 0.05);
    sR.set(radius(), { silent: true });
    views.className = arm ? 'mr-split' : '';
    rightCol.style.display = arm ? '' : 'none';
    cv.setSize({ aspect: arm ? 1 : 4 / 3 });
    // the torus view leaves room on the left / bottom for the tick labels and the θ₁, θ₂ axis titles
    cv.setWorld(arm ? { xmin: -Math.PI - 1.3, xmax: Math.PI + 0.12, ymin: -Math.PI - 1.0, ymax: Math.PI + 0.3 } : FLOOR);
    pvDirty = true;
    cobsDirty = true;
    updateVisibility();
  }

  function setAlgo(a: Algo): void {
    algo = a;
    selAlgo.set(a, { silent: true });
    updateVisibility();
  }

  function newDemo(rng: () => number): DemoState {
    const nodes: Q[] = [];
    return { rng, samples: [], nodes, index: new GridIndex(space, nodes, DEMO_R / 2), edges: [], last: [], lastSample: null, missed: 0 };
  }

  function buildPlanner(): void {
    space = makeSpace(map, isArm() ? EPS_ARM : epsFloor, isArm() ? torusOn : false);
    const rng = mulberry32(seed * 7919 + 13);
    rrt = null; con = null; prm = null; demo = null;
    pending = null; stage = 0; lastProp = null; qres = null; smooth = null;
    stopPlay();
    const g = goalFor(algo);
    switch (algo) {
      case 'demo': demo = newDemo(rng); break;
      case 'rrt':
      case 'naive':
        rrt = new RRT(space, start, g, { delta: delta(), pGoal, maxNodes: MAX_NODES, maxIterations: MAX_ITERS, variant: algo === 'naive' ? 'naive' : 'rrt' }, rng);
        break;
      case 'connect': con = new RRTConnect(space, start, g!, { delta: delta(), maxNodes: MAX_NODES, maxIterations: MAX_ITERS }, rng); break;
      case 'prm': prm = new PRM(space, { n: prmN, radius: radius(), kMax: 15 }, rng); break;
    }
    vorDirty = true;
    refresh();
  }

  /** Reset through the stepper so its buttons are re-enabled; the stepper calls buildPlanner(). */
  function restart(): void { stepper.reset(); }

  function demoStep(): void {
    const d = demo!;
    const q = space.sample(d.rng);
    const free = space.isFree(q);
    d.samples.push({ q, free });
    d.last = [];
    d.lastSample = q;
    if (!free) return;
    const ids = d.index.within(q, DEMO_R).sort((u, v) => space.dist(q, d.nodes[u]) - space.dist(q, d.nodes[v])).slice(0, DEMO_K);
    for (const j of ids) {
      const checks: CheckPoint[] = [];
      const ok = space.checkEdge(q, d.nodes[j], checks);
      const missed = ok && segmentHits(map.obstacles, q, d.nodes[j]);
      if (missed) d.missed++;
      const e: DemoEdge = { a: q, b: d.nodes[j], ok, missed, checks };
      d.edges.push(e);
      d.last.push(e);
    }
    d.nodes.push(q);
    d.index.insert(d.nodes.length - 1);
  }

  function advanceStage(): void {
    if (!rrt || rrt.finished) return;
    if (!pending || stage >= 4) { pending = rrt.propose(); stage = 1; return; }
    stage++;
    if (stage === 4) { rrt.commit(pending); lastProp = pending; }
  }

  function runQuery(): void {
    if (!prm || prm.phase !== 'done') { qres = null; return; }
    qres = prmQuery(prm, start, goalFor('prm')!);
    smooth = null;
  }

  /** One click of "single step" (or one auto tick). Returns false when the planner has finished. */
  function doStep(): boolean {
    if (demo) { demoStep(); refresh(); return demo.samples.length < DEMO_MAX; }
    if (rrt) {
      if (stageMode && algo === 'rrt') advanceStage();
      else lastProp = rrt.step();
      refresh();
      return !rrt.finished;
    }
    if (con) { con.step(); refresh(); return !con.finished; }
    if (prm) {
      prm.step(prm.phase === 'sample' ? 6 : 4);
      if (prm.phase === 'done') runQuery();
      refresh();
      return prm.phase !== 'done';
    }
    return false;
  }

  /** Run the current planner to completion (or to its limit) without animation. */
  function finishNow(): void {
    stepper.pause();
    if (demo) { while (demo.samples.length < 80) demoStep(); }
    else if (rrt) {
      if (pending && stage < 4) { rrt.commit(pending); lastProp = pending; }
      pending = null; stage = 0;
      rrt.run();
      lastProp = null;
    } else if (con) con.run();
    else if (prm) { prm.finish(); runQuery(); }
    refresh();
  }

  function preRun(n: number | 'finish'): void {
    if (n === 'finish') { finishNow(); return; }
    for (let k = 0; k < n; k++) {
      if (demo) demoStep();
      else if (rrt && !rrt.finished) lastProp = rrt.step();
      else if (con && !con.finished) con.step();
      else if (prm && prm.phase !== 'done') prm.step(8);
    }
    if (prm && prm.phase === 'done') runQuery();
    refresh();
  }

  function autoRun(speed: number): void {
    if (speedSel) { speedSel.value = String(speed); speedSel.dispatchEvent(new Event('change')); }
    stepper.run();
  }

  function doSmooth(): void {
    const p = currentPath();
    if (!p || p.length < 3) return;
    smooth = shortcut(space, p, 80, mulberry32(seed + 1000)).path;
    refresh();
  }

  // ------------------------------------------------------------------ arm playback
  const play = createLoop((dt) => {
    const p = smooth ?? currentPath();
    if (!p) { stopPlay(); return; }
    const L = pathLength(space, p);
    playU = Math.min(L, playU + dt * 1.1);
    playQ = pointAlong(space, p, playU).q;
    ws.requestRender(); cv.requestRender();
    if (playU >= L) stopPlay(false);
  });
  function startPlay(): void {
    if (!isArm() || !(smooth ?? currentPath())) return;
    playU = 0; play.start(); playBtn.textContent = '⏸ 暫停';
  }
  function stopPlay(clear = true): void {
    play.stop();
    if (clear) playQ = null;
    playBtn.textContent = '▶ 播放';
  }

  // ------------------------------------------------------------------ readouts
  function statusText(): string {
    if (demo) return `已取樣 ${demo.samples.length} 個`;
    if (rrt) {
      if (rrt.status === 'success') return `✓ 找到路徑（${rrt.iterations} 次迭代）`;
      if (rrt.status === 'failed') return `✗ 已達 ${MAX_ITERS} 次迭代，仍未找到路徑`;
      if (rrt.status === 'limit') return '已達節點上限（探索模式，沒有目標）';
      return stageMode && pending ? `生長中：${STAGES[stage - 1] ?? ''}` : rrt.goal ? '生長中…' : '探索中（沒有目標）';
    }
    if (con) {
      if (con.status === 'success') return `✓ 兩棵樹相遇（${con.iterations} 次迭代）`;
      if (con.status === 'failed') return '✗ 已達上限，兩棵樹沒有相遇';
      return '兩棵樹生長中…';
    }
    if (prm) {
      if (prm.phase === 'sample') return `建圖：取樣 ${prm.nodes.length} / ${prm.opts.n}`;
      if (prm.phase === 'connect') return `建圖：連線中（${prm.edges.length} 條邊）`;
      if (!qres) return '路網完成';
      if (qres.reason === 'ok') return '✓ 查詢成功';
      if (qres.reason === 'start') return '✗ 起點接不上路網';
      if (qres.reason === 'goal') return '✗ 目標接不上路網';
      return '✗ 起點與目標落在路網的不同連通分量';
    }
    return '—';
  }

  function refresh(): void {
    if (destroyed) return;
    const st = statusText();
    info.set('狀態', st);
    if (rrt) {
      info.set('迭代 / 節點', `${rrt.iterations} / ${rrt.tree.size}`);
      info.set('被拒絕的延伸', String(rrt.rejected));
      const cellsX = isArm() ? 24 : 24, cellsY = isArm() ? 24 : 18;
      info.set('涵蓋率', `${fmt(100 * coverage(space, rrt.tree.nodes, cellsX, cellsY), 0)} %（含節點的自由格比例）`);
    } else if (con) {
      info.set('迭代 / 節點', `${con.iterations} / ${con.trees[0].size} + ${con.trees[1].size}`);
    } else if (prm) {
      const comps = prm.phase === 'done' ? prm.components().count : 0;
      info.set('迭代 / 節點', `${prm.nodes.length} 個節點、${prm.edges.length} 條邊${comps ? `、${comps} 個連通分量` : ''}`);
      info.set('建圖成本', `${prm.buildChecks} 次碰撞查詢（取樣被拒 ${prm.rejectedSamples}、連線失敗 ${prm.blocked} / ${prm.attempts}）`);
      info.set('本次查詢成本', qres ? `${qres.checks} 次碰撞查詢＋A* 展開 ${qres.expanded} 個節點` : '—（路網完成後才能查詢）');
    } else if (demo) {
      const free = demo.samples.filter((s) => s.free).length;
      const ok = demo.edges.filter((e) => e.ok).length;
      info.set('迭代 / 節點', `${demo.samples.length} 個樣本，${free} 個自由`);
      info.set('入門統計', `連線 ${ok} / ${demo.edges.length} 條通過檢查${demo.missed ? `，其中 ${demo.missed} 條其實穿過障礙（漏檢）` : ''}`);
    }
    info.set('碰撞查詢次數', String(space.checks));
    const p = currentPath();
    info.set('路徑長度', p ? `${fmt(pathLength(space, p), 2)} ${unit()}${smooth ? ` → 平滑後 ${fmt(pathLength(space, smooth), 2)} ${unit()}` : ''}` : '—');
    const finished = (rrt?.finished ?? false) || (con?.finished ?? false) || (prm ? prm.phase === 'done' : false);
    if (finished) stepper.finish(st); else stepper.setStatus('');
    smoothBtn.disabled = !p || p.length < 3;
    playBtn.disabled = !p;
    paintStages();
    vorDirty = true;
    cv.requestRender();
    if (isArm()) ws.requestRender();
  }

  /** Highlight the current stage (stage mode only): done stages solid, the current one accented, later ones faded. */
  function paintStages(): void {
    const on = stageMode && algo === 'rrt' && !!rrt && !rrt.finished;
    stageStrip.style.display = on ? '' : 'none';
    if (!on) return;
    stagePills.forEach((pill, i) => {
      pill.classList.toggle('active', i + 1 === stage);
      pill.style.opacity = i + 1 > stage ? '0.55' : '1';
    });
  }

  function updateVisibility(): void {
    const show = (e: HTMLElement, v: boolean) => { e.style.display = v ? '' : 'none'; };
    const tree = algo === 'rrt' || algo === 'naive' || algo === 'connect';
    show(sDelta.el, tree);
    show(sPGoal.el, algo === 'rrt');
    show(sN.el, algo === 'prm');
    show(sR.el, algo === 'prm');
    show(sEps.el, !isArm());
    show(sGap.el, mapId === 'narrow');
    show(tStage.el, algo === 'rrt');
    show(tVor.el, algo === 'rrt' || algo === 'naive');
    show(tTorus.el, isArm());
    show(playBtn, isArm());
    show(smoothBtn, algo !== 'demo');
    show(finishBtn, true);
    info.show('被拒絕的延伸', algo === 'rrt' || algo === 'naive');
    info.show('涵蓋率', algo === 'rrt' || algo === 'naive');
    info.show('q_near 的 Voronoi 面積', (algo === 'rrt' || algo === 'naive') && showVor);
    info.show('建圖成本', algo === 'prm');
    info.show('本次查詢成本', algo === 'prm');
    info.show('入門統計', algo === 'demo');
    info.show('路徑長度', algo !== 'demo');
    show(expBox, mapId === 'narrow');
    expPlot.setOptions({ xLabel: algo === 'prm' ? 'PRM 節點數 N' : '迭代上限' });
    sHandle.enabled = algo !== 'demo';
    gHandle.enabled = algo !== 'demo' && !!goalFor(algo);
    // legend
    const items: { color: string; label: string }[] = [];
    if (algo !== 'demo') items.push({ color: 'var(--series-3)', label: '起點 S（可拖曳）' });
    if (algo !== 'demo' && goalFor(algo)) items.push({ color: 'var(--series-1)', label: '目標 G（可拖曳）' });
    if (algo === 'demo') items.push({ color: 'var(--fg)', label: '自由樣本' }, { color: 'var(--danger)', label: '碰撞樣本 ×／檢查失敗點' }, { color: 'var(--warn)', label: '漏檢：通過檢查卻穿過障礙' });
    if (algo === 'rrt' || algo === 'naive') items.push({ color: 'var(--series-7)', label: 'q_rand' }, { color: 'var(--accent)', label: 'q_near / q_new' });
    if (algo === 'connect') items.push({ color: 'var(--series-3)', label: '起點樹' }, { color: 'var(--series-1)', label: '目標樹' });
    if (algo === 'prm') items.push({ color: 'var(--series-6)', label: '節點顏色＝連通分量' });
    if (algo !== 'demo') items.push({ color: 'var(--warn)', label: '路徑' }, { color: 'var(--series-5)', label: '捷徑平滑後' });
    legendHost.replaceChildren();
    legend(legendHost, items);
    footNote.textContent = isArm()
      ? '左圖橫軸 θ₁、縱軸 θ₂（範圍 −180°～180°），灰色是 C-obstacle；連桿畫成實際粗細（半寬 7 cm），碰撞檢查以 0.02 rad 的間距沿局部路徑取樣。'
      : '地圖 12 m × 9 m，點機器人；碰撞檢查以間距 ε 沿局部路徑取樣。「碰撞查詢次數」是規劃器付出的主要成本。';
  }

  // ------------------------------------------------------------------ drawing helpers
  const font = (px: number, bold = false): string => `${bold ? 'bold ' : ''}${px}px ${cssVar('--font') || 'sans-serif'}`;

  /** A local path drawn on the torus may leave the square; draw it twice (shifted copies), clipped later. */
  function segsFor(a: Q, b: Q): [Q, Q][] {
    if (!space.torus) return [[a, b]];
    const d = space.diff(a, b);
    const e: Q = [a[0] + d[0], a[1] + d[1]];
    const out: [Q, Q][] = [[a, e]];
    if (Math.abs(e[0] - b[0]) > 1e-9 || Math.abs(e[1] - b[1]) > 1e-9) out.push([[b[0] - d[0], b[1] - d[1]], b]);
    return out;
  }

  function strokeSegs(c: Canvas2D, segs: [Q, Q][], color: string, width: number, alpha = 1, dash: number[] = []): void {
    if (!segs.length) return;
    const g = c.ctx;
    g.save();
    g.strokeStyle = themeColor(color, '#888');
    g.lineWidth = width; g.globalAlpha = alpha; g.setLineDash(dash); g.lineCap = 'round';
    g.beginPath();
    for (const [a, b] of segs) {
      const [ax, ay] = c.toScreen(a); const [bx, by] = c.toScreen(b);
      g.moveTo(ax, ay); g.lineTo(bx, by);
    }
    g.stroke();
    g.restore();
  }

  function fillDots(c: Canvas2D, pts: readonly Q[], rPx: number, color: string, alpha = 1): void {
    if (!pts.length) return;
    const g = c.ctx;
    g.save();
    g.fillStyle = themeColor(color, '#888'); g.globalAlpha = alpha;
    g.beginPath();
    for (const p of pts) { const [x, y] = c.toScreen(p); g.moveTo(x + rPx, y); g.arc(x, y, rPx, 0, TAU); }
    g.fill();
    g.restore();
  }

  function crosses(c: Canvas2D, pts: readonly Q[], sPx: number, color: string, width = 1.6): void {
    if (!pts.length) return;
    const g = c.ctx;
    g.save();
    g.strokeStyle = themeColor(color, '#c00'); g.lineWidth = width;
    g.beginPath();
    for (const p of pts) { const [x, y] = c.toScreen(p); g.moveTo(x - sPx, y - sPx); g.lineTo(x + sPx, y + sPx); g.moveTo(x - sPx, y + sPx); g.lineTo(x + sPx, y - sPx); }
    g.stroke();
    g.restore();
  }

  function pathSegs(p: readonly Q[]): [Q, Q][] {
    const out: [Q, Q][] = [];
    for (let i = 0; i + 1 < p.length; i++) out.push(...segsFor(p[i], p[i + 1]));
    return out;
  }

  function treeSegs(nodes: readonly Q[], parent: readonly number[]): [Q, Q][] {
    const out: [Q, Q][] = [];
    for (let i = 1; i < nodes.length; i++) out.push(...segsFor(nodes[parent[i]], nodes[i]));
    return out;
  }

  function drawFloor(c: Canvas2D): void {
    c.grid(1);
    const b = FLOOR;
    c.polygon([[b.xmin, b.ymin], [b.xmax, b.ymin], [b.xmax, b.ymax], [b.xmin, b.ymax]], { color: '--grid-strong', width: 1.5 });
    if (hideObs) return;
    for (const o of map.obstacles) {
      if (o.kind === 'rect') c.polygon([[o.x0, o.y0], [o.x1, o.y0], [o.x1, o.y1], [o.x0, o.y1]], { fill: '--fg-muted', color: '--fg-muted', width: 1, alpha: 0.45 });
      else c.circle(o.c, o.r, { fill: '--fg-muted', color: '--fg-muted', width: 1, alpha: 0.45 });
    }
  }

  function drawTorusFrame(c: Canvas2D): void {
    const P = Math.PI;
    const muted = '--fg-muted';
    for (let k = -2; k <= 2; k++) {
      const v = (k * P) / 2;
      c.line([v, -P], [v, P], { color: '--grid', width: 1 });
      c.line([-P, v], [P, v], { color: '--grid', width: 1 });
      c.text([v, -P], deg(v), { dy: 11, color: muted, font: font(10) });
      c.text([-P, v], deg(v), { dx: -5, align: 'right', color: muted, font: font(10) });
    }
    c.text([0, -P], 'θ₁', { dy: 27, color: '--fg', font: font(12, true) });
    // vertical axis title, rotated so it never collides with the tick labels
    const [sx, sy] = c.toScreen([-P, 0]);
    const g = c.ctx;
    g.save();
    g.translate(sx - 40, sy);
    g.rotate(-Math.PI / 2);
    g.font = font(12, true); g.textAlign = 'center'; g.textBaseline = 'middle';
    g.fillStyle = themeColor('--fg');
    g.fillText('θ₂', 0, 0);
    g.restore();
  }

  function drawTorusEdges(c: Canvas2D): void {
    const P = Math.PI;
    if (torusOn) {
      c.line([-P, -P], [-P, P], { color: '--series-6', width: 3, dash: [6, 4] });
      c.line([P, -P], [P, P], { color: '--series-6', width: 3, dash: [6, 4] });
      c.line([-P, -P], [P, -P], { color: '--series-6', width: 3, dash: [6, 4] });
      c.line([-P, P], [P, P], { color: '--series-6', width: 3, dash: [6, 4] });
    } else {
      c.polygon([[-P, -P], [P, -P], [P, P], [-P, P]], { color: '--danger', width: 3.5 });
      c.text([0, P], '接縫被當成牆（錯誤示範）', { dy: 10, color: '--danger', font: font(11, true), bg: '--bg-elev' });
    }
  }

  function clipToSquare(c: Canvas2D): void {
    const b = map.bounds;
    const [x0, y0] = c.toScreen([b.xmin, b.ymax]);
    const [x1, y1] = c.toScreen([b.xmax, b.ymin]);
    c.ctx.save();
    c.ctx.beginPath();
    c.ctx.rect(x0, y0, x1 - x0, y1 - y0);
    c.ctx.clip();
  }

  function drawDemo(c: Canvas2D, d: DemoState): void {
    const okEdges = d.edges.filter((e) => e.ok && !e.missed).map((e) => [e.a, e.b] as [Q, Q]);
    strokeSegs(c, okEdges, '--fg-muted', 1.3, 0.75);
    strokeSegs(c, d.edges.filter((e) => e.missed).map((e) => [e.a, e.b] as [Q, Q]), '--warn', 3, 0.95);
    for (const e of d.last) {
      if (!e.ok) strokeSegs(c, [[e.a, e.b]], '--danger', 1.4, 0.8, [5, 4]);
      fillDots(c, e.checks.filter((k) => k.free).map((k) => k.q), 2.2, '--ok');
      fillDots(c, e.checks.filter((k) => !k.free).map((k) => k.q), 3.5, '--danger');
    }
    fillDots(c, d.samples.filter((s) => s.free).map((s) => s.q), 3, '--fg');
    crosses(c, d.samples.filter((s) => !s.free).map((s) => s.q), 4, '--danger');
    if (d.lastSample) c.circle(d.lastSample, 0, { color: '--accent' });
    if (d.lastSample) { const [x, y] = c.toScreen(d.lastSample); c.ctx.save(); c.ctx.strokeStyle = themeColor('--accent'); c.ctx.lineWidth = 2; c.ctx.beginPath(); c.ctx.arc(x, y, 8, 0, TAU); c.ctx.stroke(); c.ctx.restore(); }
    const firstMissed = d.edges.find((e) => e.missed);
    if (firstMissed) c.text([(firstMissed.a[0] + firstMissed.b[0]) / 2, (firstMissed.a[1] + firstMissed.b[1]) / 2], '漏檢！', { color: '--warn', font: font(12, true), bg: '--bg-elev', dy: -12 });
  }

  function ringAt(c: Canvas2D, q: Q, rPx: number, color: string, width = 2): void {
    const [x, y] = c.toScreen(q);
    const g = c.ctx;
    g.save(); g.strokeStyle = themeColor(color); g.lineWidth = width; g.beginPath(); g.arc(x, y, rPx, 0, TAU); g.stroke(); g.restore();
  }

  function star(c: Canvas2D, q: Q, color: string): void {
    const [x, y] = c.toScreen(q);
    const g = c.ctx;
    g.save(); g.fillStyle = themeColor(color); g.beginPath();
    for (let k = 0; k < 10; k++) {
      const r = k % 2 ? 3.2 : 8, a = -Math.PI / 2 + (k * Math.PI) / 5;
      const px = x + r * Math.cos(a), py = y + r * Math.sin(a);
      if (k) g.lineTo(px, py); else g.moveTo(px, py);
    }
    g.closePath(); g.fill(); g.restore();
  }

  function drawProposal(c: Canvas2D, p: Proposal, s: number, faint: boolean): void {
    const T = rrt!.tree;
    const qn = T.nodes[p.near];
    const a = faint ? 0.55 : 1;
    if (s >= 1 && algo === 'rrt') { star(c, p.qRand, '--series-7'); if (!faint) c.text(p.qRand, p.fromGoal ? 'q_rand = q_goal' : 'q_rand', { dy: -14, color: '--series-7', font: font(12, true), bg: '--bg-elev' }); }
    if (s >= 2) {
      if (algo === 'rrt') strokeSegs(c, segsFor(qn, p.qRand), '--series-7', 1.3, a * 0.8, [5, 4]);
      ringAt(c, qn, 7, '--accent', 2.2);
      if (!faint) c.text(qn, 'q_near', { dx: -26, dy: 12, color: '--accent', font: font(12, true), bg: '--bg-elev' });
    }
    if (s >= 3) {
      if (!faint) c.circle(qn, delta(), { color: '--fg-muted', width: 1, dash: [3, 4] });
      fillDots(c, [p.qNew], 4, '--accent', a);
      if (!faint) c.text(p.qNew, 'q_new', { dx: 22, dy: -10, color: '--accent', font: font(12, true), bg: '--bg-elev' });
    }
    if (s >= 4 && p.ok !== null) {
      fillDots(c, p.checks.filter((k) => k.free).map((k) => k.q), 2.2, '--ok', a);
      const bad = p.checks.filter((k) => !k.free).map((k) => k.q);
      if (p.ok) strokeSegs(c, segsFor(qn, p.qNew), '--accent', 3, a);
      else { strokeSegs(c, segsFor(qn, p.qNew), '--danger', 2, a, [4, 3]); crosses(c, bad, 5, '--danger', 2.2); if (!faint) c.text(p.qNew, '撞到 → 丟棄', { dy: 16, color: '--danger', font: font(12, true), bg: '--bg-elev' }); }
    }
  }

  function drawPRM(c: Canvas2D, P: PRM): void {
    const comps = P.phase === 'done' ? P.components().labels : null;
    strokeSegs(c, P.edges.map(([i, j]) => [P.nodes[i], P.nodes[j]] as [Q, Q]).flatMap(([a, b]) => segsFor(a, b)), '--fg-muted', 1, 0.5);
    // highlight the connections tried in the last build step (only while the roadmap is being built)
    if (!comps) for (const t of P.lastTried) strokeSegs(c, segsFor(P.nodes[t.a], P.nodes[t.b]), t.ok ? '--accent' : '--danger', t.ok ? 2 : 1.4, 0.9, t.ok ? [] : [4, 3]);
    if (comps) {
      const groups = new Map<number, Q[]>();
      comps.forEach((l, i) => { const k = Math.min(l, PALETTE.length - 1); if (!groups.has(k)) groups.set(k, []); groups.get(k)!.push(P.nodes[i]); });
      for (const [k, pts] of groups) fillDots(c, pts, 3, PALETTE[k]);
    } else fillDots(c, P.nodes, 3, '--fg');
    if (qres) {
      const g = goalFor('prm')!;
      strokeSegs(c, qres.startLinks.flatMap((j) => segsFor(start, P.nodes[j])), '--series-3', 1.4, 0.8, [4, 3]);
      strokeSegs(c, qres.goalLinks.flatMap((j) => segsFor(g, P.nodes[j])), '--series-1', 1.4, 0.8, [4, 3]);
    }
  }

  // ------------------------------------------------------------------ main canvas
  cv.draw = (c) => {
    const arm = isArm();
    if (arm) {
      drawTorusFrame(c);
      if (!hideObs) { if (cobsDirty || !cobs) paintCobs(); cobs!.draw(c); }
    } else drawFloor(c);
    if (showVor && rrt) {
      const now = performance.now();
      if (vorDirty) {
        if (!stepper.running || now - vorAt > 150) paintVoronoi();
        else if (!vorTimer) vorTimer = window.setTimeout(() => { vorTimer = 0; cv.requestRender(); }, 160);
      }
      vor?.draw(c);
    }
    if (arm) clipToSquare(c);
    if (demo) drawDemo(c, demo);
    if (rrt) {
      strokeSegs(c, treeSegs(rrt.tree.nodes, rrt.tree.parent), '--series-3', 1.3, 0.85);
      fillDots(c, rrt.tree.nodes, 1.8, '--series-3');
      if (stageMode && pending && algo === 'rrt') drawProposal(c, pending, stage, false);
      else if (lastProp && !rrt.finished) drawProposal(c, lastProp, 4, true);
    }
    if (con) {
      strokeSegs(c, treeSegs(con.trees[0].nodes, con.trees[0].parent), '--series-3', 1.3, 0.85);
      strokeSegs(c, treeSegs(con.trees[1].nodes, con.trees[1].parent), '--series-1', 1.3, 0.85);
      fillDots(c, con.trees[0].nodes, 1.8, '--series-3');
      fillDots(c, con.trees[1].nodes, 1.8, '--series-1');
      if (!con.finished) for (const s of con.last) strokeSegs(c, segsFor(s.from, s.to), s.ok ? '--accent' : '--danger', s.ok ? 2.4 : 1.6, 0.9, s.ok ? [] : [4, 3]);
      if (con.meet) ringAt(c, con.trees[0].nodes[con.meet[0]], 9, '--accent', 2.5);
    }
    if (prm) drawPRM(c, prm);
    const p = currentPath();
    if (p) {
      strokeSegs(c, pathSegs(p), '--warn', smooth ? 2 : 3.5, smooth ? 0.7 : 0.95, smooth ? [6, 4] : []);
      if (smooth) strokeSegs(c, pathSegs(smooth), '--series-5', 3.5, 0.95);
    }
    if (arm) {
      const mk = playQ ?? hoverQ;
      if (mk) { ringAt(c, mk, 6, '--fg', 2); fillDots(c, [mk], 2.5, '--fg'); }
      c.ctx.restore();
      drawTorusEdges(c);
    }
    if (showVor && rrt && vorCounts && vorIds) {
      const near = vorNear;
      const frac = near >= 0 && near < vorCounts.length ? vorCounts[near] / vorIds.length : NaN;
      info.set('q_near 的 Voronoi 面積', Number.isFinite(frac) ? `${fmt(100 * frac, 1)} %（選它當下樹有 ${vorN} 個節點，平均每個 ${fmt(100 / Math.max(1, vorN), 1)} %）` : '—（單步一次就會標出）');
    }
    const n = rrt ? rrt.tree.size : con ? con.size : prm ? prm.nodes.length : demo ? demo.samples.length : 0;
    // the torus view is narrow (side by side with the arm): its numbers live in the table below instead
    c.setOverlayText(arm ? '' : `${ALGO_LABEL[algo]} · ${algo === 'demo' ? '樣本' : '節點'} ${n} · 碰撞查詢 ${space.checks}`);
  };

  const sHandle = cv.addHandle({
    get: () => start, color: '--series-3', label: 'S', radius: 9,
    set: (p) => { start = clampQ(p); onDragMove(); },
    onStart: () => stepper.pause(), onEnd: () => onDragEnd(),
  });
  const gHandle = cv.addHandle({
    get: () => goalFor(algo) ?? EMPTY_GOAL, color: '--series-1', label: 'G', radius: 9,
    set: (p) => { goal = clampQ(p); onDragMove(); },
    onStart: () => stepper.pause(), onEnd: () => onDragEnd(),
  });
  function clampQ(p: Q): Q {
    const b = map.bounds;
    return [Math.min(b.xmax - 1e-6, Math.max(b.xmin, p[0])), Math.min(b.ymax - 1e-6, Math.max(b.ymin, p[1]))];
  }
  function onDragMove(): void {
    chips.clear();
    if (prm && prm.phase === 'done') { runQuery(); refresh(); return; }
    cv.requestRender();
    if (isArm()) ws.requestRender();
  }
  function onDragEnd(): void {
    if (prm && prm.phase === 'done') return;
    const keepFinish = curStepCfg()?.pre === 'finish';
    restart();
    if (keepFinish) preRun('finish');
    pvDirty = true;
  }

  const offPointer = cv.onPointer({
    move: (p) => {
      if (!isArm()) return;
      const inside = Math.abs(p[0]) <= Math.PI && Math.abs(p[1]) <= Math.PI;
      hoverQ = inside ? [p[0], p[1]] : null;
      ws.requestRender(); cv.requestRender();
    },
  });
  const onLeave = (): void => { if (hoverQ) { hoverQ = null; ws.requestRender(); cv.requestRender(); } };
  cv.canvas.addEventListener('pointerleave', onLeave);

  // ------------------------------------------------------------------ workspace canvas (2R arm)
  function drawArm(c: Canvas2D, q: Q, color: string, alpha: number): void {
    const [b, e, t] = armPoints(q);
    const w = Math.max(3, 2 * ARM_W * c.scale);
    c.line(b, e, { color, width: w, alpha });
    c.line(e, t, { color, width: w, alpha });
    c.point(b, w * 0.32, { color: '--bg-elev', alpha });
    c.point(e, w * 0.32, { color: '--bg-elev', alpha });
    c.point(t, 3, { color: '--fg', alpha });
  }

  ws.draw = (c) => {
    c.grid(0.5);
    c.circle([0, 0], ARM_L[0] + ARM_L[1], { color: '--grid-strong', dash: [4, 4], width: 1 });
    for (const o of ARM_OBSTACLES) c.circle(o.c, o.r, { fill: '--fg-muted', color: '--fg-muted', width: 1, alpha: 0.5 });
    c.polygon([[-0.13, -0.13], [0.13, -0.13], [0, 0]], { fill: '--grid-strong', color: '--grid-strong', width: 1 });
    const g = goalFor(algo);
    drawArm(c, start, '--series-3', 0.35);
    if (g) drawArm(c, g, '--series-1', 0.35);
    c.text(armPoints(start)[2], 'S', { color: '--series-3', dx: 10, dy: -10, font: font(13, true) });
    if (g) c.text(armPoints(g)[2], 'G', { color: '--series-1', dx: 10, dy: -10, font: font(13, true) });
    const p = smooth ?? currentPath();
    if (p) {
      const tips: Q[] = [];
      const L = pathLength(space, p);
      const n = Math.max(2, Math.ceil(L / 0.03));
      for (let k = 0; k <= n; k++) tips.push(armPoints(pointAlong(space, p, (k / n) * L).q)[2] as Q);
      c.polyline(tips, { color: smooth ? '--series-5' : '--warn', width: 1.8, dash: [5, 4] });
    }
    const q = playQ ?? hoverQ ?? start;
    const hit = !armFree(ARM_OBSTACLES, q);
    drawArm(c, q, hit ? '--danger' : '--fg-muted', 1);
    c.setOverlayText(`θ = (${deg(q[0])}, ${deg(q[1])})${hit ? '  碰撞！' : ''}`);
  };

  // ------------------------------------------------------------------ preview canvas
  pv.draw = (c) => {
    if (pvDirty || !pvRaster) paintPreview();
    pvRaster?.draw(c, 1, true);
    for (const o of map.obstacles) {
      if (o.kind === 'rect') c.polygon([[o.x0, o.y0], [o.x1, o.y0], [o.x1, o.y1], [o.x0, o.y1]], { fill: '--fg-muted', color: '--fg-muted', width: 1, alpha: 0.8 });
      else c.circle(o.c, o.r, { fill: '--fg-muted', color: '--fg-muted', width: 1, alpha: 0.8 });
    }
    const g = goalFor('prm')!;
    c.point(g, 5, { color: '--series-1' });
    c.text(g, 'G（谷底）', { dy: -12, color: '--fg', font: font(11, true), bg: '--bg-elev' });
  };

  // ------------------------------------------------------------------ success-rate experiment
  interface ExpRun { name: string; color: string; xs: number[]; ys: number[] }
  let expRuns: ExpRun[] = [];
  let expJob: { run: ExpRun; algo: Algo; gap: number; xi: number; s: number; succ: number; space: CSpace; m: PlanMap } | null = null;
  let expRaf = 0;
  const EXP_COLORS = ['--series-1', '--series-4', '--series-5'];

  function paintExperiment(): void {
    const runs = expRuns.length ? expRuns : [{ name: '（尚無資料）', color: '--fg-muted', xs: [], ys: [] }];
    expPlot.setOptions({ series: runs.map((r) => ({ name: r.name, color: r.color, width: 2.2 })) });
    runs.forEach((r, i) => expPlot.setXY(i, r.xs, r.ys));
    expPlot.setMarkers([{ y: 1, label: '100 %', color: '--ok' }]);
  }

  function startExperiment(): void {
    if (expJob) return;
    const a: Algo = algo === 'rrt' || algo === 'connect' ? algo : 'prm';
    const m = makeMap('narrow', gap);
    const run: ExpRun = { name: `${a === 'prm' ? 'PRM' : ALGO_LABEL[a]}，通道 ${fmt(gap, 2)} m`, color: EXP_COLORS[expRuns.length % 3], xs: [], ys: [] };
    expRuns = [...expRuns.slice(-2), run];
    expRuns.forEach((r, i) => { r.color = EXP_COLORS[i % 3]; });
    expJob = { run, algo: a, gap, xi: 0, s: 1, succ: 0, space: makeSpace(m, epsFloor), m };
    expBtn.disabled = true;
    paintExperiment();
    expRaf = requestAnimationFrame(pumpExperiment);
  }

  function cancelExperiment(): void {
    if (expRaf) cancelAnimationFrame(expRaf);
    expRaf = 0; expJob = null; expBtn.disabled = false; expStatus.textContent = '';
  }

  function pumpExperiment(): void {
    expRaf = 0;
    const job = expJob;
    if (!job || destroyed) return;
    const t0 = performance.now();
    while (expJob && performance.now() - t0 < 12) {
      const x = EXP_XS[job.xi];
      const ok = job.algo === 'prm'
        ? prmTrial(job.space, job.m.start, job.m.goal!, { n: x, radius: rFloor, kMax: 15 }, job.s).success
        : rrtTrial(job.space, job.m.start, job.m.goal!, { delta: deltaFloor, pGoal, budget: x, connect: job.algo === 'connect' }, job.s).success;
      if (ok) job.succ++;
      job.s++;
      if (job.s > EXP_SEEDS) {
        job.run.xs.push(x); job.run.ys.push(job.succ / EXP_SEEDS);
        job.xi++; job.s = 1; job.succ = 0;
        paintExperiment();
        if (job.xi >= EXP_XS.length) { expJob = null; break; }
      }
    }
    if (expJob) {
      expStatus.textContent = `計算中… N = ${EXP_XS[job.xi]}，種子 ${job.s}/${EXP_SEEDS}`;
      expRaf = requestAnimationFrame(pumpExperiment);
    } else {
      expBtn.disabled = false;
      const last = job.run.ys[job.run.ys.length - 1];
      expStatus.textContent = `完成：N = ${EXP_XS[EXP_XS.length - 1]} 時成功率 ${fmt(100 * last, 0)} %`;
    }
  }

  // ------------------------------------------------------------------ steps
  interface StepCfg {
    map: MapId; algo: Algo; pre: number | 'finish';
    stage?: boolean; vor?: boolean; hide?: boolean; delta?: number; pGoal?: number; n?: number; gap?: number; eps?: number; torus?: boolean;
    chips: () => Preset[];
    focus: () => (HTMLElement | null)[];
    after?: () => void;
  }
  const rerun = (pre: number | 'finish', speed?: number): void => { restart(); preRun(pre); if (speed) autoRun(speed); };
  /** Restart the current tree planner and grow it to `n` nodes at once (for the coverage comparison). */
  const growTo = (n: number): void => { restart(); while (rrt && !rrt.finished && rrt.tree.size < n) lastProp = rrt.step(); refresh(); };
  const STEP_CFG: StepCfg[] = [
    {
      map: 'clutter', algo: 'demo', pre: 30, eps: 0.05, hide: false,
      chips: () => [
        { label: 'ε = 0.05 m（細）', apply: () => { epsFloor = 0.05; sEps.set(0.05, { silent: true }); rerun(30); } },
        { label: 'ε = 0.5 m（太粗）', apply: () => { epsFloor = 0.5; sEps.set(0.5, { silent: true }); rerun(120); } },
        { label: '規劃器的視角（隱藏障礙）', apply: () => tHide.set(true) },
        { label: '上帝視角（顯示障礙）', apply: () => tHide.set(false) },
      ],
      focus: () => [cv.el, sEps.el, tHide.el],
    },
    {
      map: 'clutter', algo: 'rrt', pre: 14, stage: true, delta: 0.9, pGoal: 0.05, eps: 0.05,
      chips: () => [
        { label: '逐階段：一次一個動作', apply: () => { tStage.set(true, { silent: true }); stageMode = true; rerun(14); advanceStage(); refresh(); } },
        { label: '整輪迭代', apply: () => { tStage.set(false, { silent: true }); stageMode = false; rerun(14); } },
        { label: '長到找到路徑', apply: () => { tStage.set(false, { silent: true }); stageMode = false; rerun(0, 16); } },
      ],
      focus: () => [stepper.el, cv.el],
      after: () => { advanceStage(); refresh(); },
    },
    {
      map: 'empty', algo: 'rrt', pre: 60, vor: true, delta: 0.5, pGoal: 0, eps: 0.05,
      chips: () => [
        { label: 'RRT（有 Voronoi 偏置）', apply: () => { setAlgo('rrt'); rerun(60); } },
        { label: '對照：隨機節點擴張', apply: () => { setAlgo('naive'); rerun(60); } },
        { label: 'RRT 長到 300 個節點', apply: () => { setAlgo('rrt'); growTo(300); } },
        { label: '隨機擴張長到 300 個節點', apply: () => { setAlgo('naive'); growTo(300); } },
      ],
      focus: () => [tVor.el, info.el],
    },
    {
      map: 'wall', algo: 'rrt', pre: 'finish', delta: 0.6, pGoal: 0.05, eps: 0.05,
      chips: () => [
        { label: 'p_goal = 0', apply: () => { setAlgo('rrt'); pGoal = 0; sPGoal.set(0, { silent: true }); rerun(0, 64); } },
        { label: 'p_goal = 0.05', apply: () => { setAlgo('rrt'); pGoal = 0.05; sPGoal.set(0.05, { silent: true }); rerun(0, 64); } },
        { label: 'p_goal = 1（貪婪）', apply: () => { setAlgo('rrt'); pGoal = 1; sPGoal.set(1, { silent: true }); rerun(0, 64); } },
        { label: 'RRT-Connect', apply: () => { setAlgo('connect'); rerun(0, 16); } },
      ],
      focus: () => [sPGoal.el, selAlgo.el],
    },
    {
      map: 'clutter', algo: 'prm', pre: 'finish', n: 150, eps: 0.05,
      chips: () => [
        ...QUERIES.map(([s, g], i): Preset => ({ label: `查詢 ${'ABC'[i]}`, apply: () => { start = [...s]; goal = [...g]; runQuery(); refresh(); } })),
        { label: '重建路網（逐步動畫）', apply: () => { rerun(0, 16); } },
      ],
      focus: () => [cv.el, info.el],
    },
    {
      map: 'arm', algo: 'connect', pre: 'finish', delta: 0.25, pGoal: 0.05, torus: true,
      chips: () => [
        { label: 'RRT-Connect', apply: () => { setAlgo('connect'); rerun('finish'); } },
        { label: 'RRT', apply: () => { setAlgo('rrt'); rerun('finish'); } },
        { label: '把接縫當牆（錯誤示範）', apply: () => { tTorus.set(false, { silent: true }); torusOn = false; rerun('finish'); } },
        { label: '恢復環面', apply: () => { tTorus.set(true, { silent: true }); torusOn = true; rerun('finish'); } },
        { label: '✂ 捷徑平滑後播放', apply: () => { doSmooth(); startPlay(); } },
      ],
      focus: () => [ws.el, tTorus.el, smoothBtn],
    },
    {
      map: 'narrow', algo: 'prm', pre: 'finish', n: 120, gap: 0.6, eps: 0.05, delta: 0.6, pGoal: 0.05,
      chips: () => [
        { label: '通道 1.2 m', apply: () => { sGap.set(1.2); preRun('finish'); } },
        { label: '通道 0.6 m', apply: () => { sGap.set(0.6); preRun('finish'); } },
        { label: '通道 0.3 m', apply: () => { sGap.set(0.3); preRun('finish'); } },
        { label: 'N = 400', apply: () => { prmN = 400; sN.set(400, { silent: true }); rerun('finish'); } },
      ],
      focus: () => [expBox, sGap.el],
      after: () => { if (!expRuns.length && !expJob) startExperiment(); },
    },
  ];
  const curStepCfg = (): StepCfg | undefined => STEP_CFG[curStep];

  let focused: HTMLElement[] = [];
  function onStep(i: number): void {
    const cfg = STEP_CFG[Math.max(0, Math.min(STEP_CFG.length - 1, i))];
    if (i === curStep) return;
    curStep = i;
    cancelExperimentIfLeaving(cfg);
    // parameters of this step
    if (cfg.eps !== undefined) { epsFloor = cfg.eps; sEps.set(cfg.eps, { silent: true }); }
    if (cfg.gap !== undefined) { gap = cfg.gap; sGap.set(cfg.gap, { silent: true }); }
    if (cfg.n !== undefined) { prmN = cfg.n; sN.set(cfg.n, { silent: true }); }
    if (cfg.pGoal !== undefined) { pGoal = cfg.pGoal; sPGoal.set(cfg.pGoal, { silent: true }); }
    if (cfg.torus !== undefined) { torusOn = cfg.torus; tTorus.set(cfg.torus, { silent: true }); }
    stageMode = !!cfg.stage; tStage.set(stageMode, { silent: true });
    showVor = !!cfg.vor; tVor.set(showVor, { silent: true });
    hideObs = !!cfg.hide; tHide.set(hideObs, { silent: true });
    setMap(cfg.map);
    if (cfg.delta !== undefined) { if (isArm()) deltaArm = cfg.delta; else deltaFloor = cfg.delta; sDelta.set(cfg.delta, { silent: true }); }
    setAlgo(cfg.algo);
    rFloor = 1.5; rArm = 0.7; sR.set(radius(), { silent: true });
    seed = 1; sSeed.set(1, { silent: true });
    restart();
    preRun(cfg.pre);
    cfg.after?.();
    chips.setPresets(cfg.chips());
    previewBox.style.display = i === STEP_CFG.length - 1 ? '' : 'none';
    pv.requestRender();
    focused.forEach((f) => f.classList.remove('mr-focus'));
    focused = cfg.focus().filter((x): x is HTMLElement => !!x);
    focused.forEach((f) => f.classList.add('mr-focus'));
  }
  function cancelExperimentIfLeaving(cfg: StepCfg): void { if (cfg.map !== 'narrow') cancelExperiment(); }

  const offTheme = ctx.onThemeChange(() => { cobsDirty = true; vorDirty = true; pvDirty = true; cv.requestRender(); ws.requestRender(); pv.requestRender(); paintExperiment(); });

  paintExperiment();
  onStep(ctx.currentStep);

  return {
    onStep,
    destroy() {
      destroyed = true;
      cancelExperiment();
      if (vorTimer) clearTimeout(vorTimer);
      stepper.destroy();
      play.stop();
      offTheme();
      offPointer();
      cv.canvas.removeEventListener('pointerleave', onLeave);
      cv.destroy(); ws.destroy(); pv.destroy(); expPlot.destroy();
    },
  };
}

const lesson: Lesson = { meta, steps, quiz, mount };
export default lesson;
