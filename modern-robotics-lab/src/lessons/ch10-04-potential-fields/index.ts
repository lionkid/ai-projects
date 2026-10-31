import type { Lesson, LessonContext, LessonHandle } from '@/types';
import {
  el, cssVar, fmt, themeColor, controlPanel, row, slider, toggle, button, select, kv, note, simTitle, legend, presetChips,
  mdElement, split, Canvas2D, Plot2D, RasterLayer, physicsRunner,
  type Handle, type Preset, type RGBA,
} from '@/ui';
import { createPID, rk4Step, type P2 } from '@/mr';
import { meta } from './meta';
import { quiz, steps } from './content';
import {
  BASIN_STUCK, BasinJob, BiasedRRT, DEFAULT_ROBOT, PointRobot, contourSegments, evalField, fieldEnv, mulberry32,
  navDescentField, navEnv, navFunction, nearestClearance, potentialAt, refineMinimum, translateObstacle,
  type Env, type FieldParams, type Model, type Obstacle, type ObstacleGroup, type SphereWorld,
} from './potential';
import { SPHERE_WORLD, WORLD, makeScene, type Scene, type SceneId } from './scenes';

// ---------------------------------------------------------------------------------------------
// Simulation
// ---------------------------------------------------------------------------------------------

type Mode = 'field' | 'nav' | 'rrt';
interface Show { heat: boolean; contours: boolean; arrows: boolean; influence: boolean; basin: boolean }
interface Ghost { label: string; segs: P2[][]; dist: [number, number][] }

const TAU = 2 * Math.PI;
const HEAT_NX = 150, HEAT_NY = 112;
const GRID_NX = 81, GRID_NY = 61;
const BASIN_NX = 50, BASIN_NY = 38;
const ARROWS_NX = 25, ARROWS_NY = 19;
const GHOST_COLORS = ['--series-5', '--series-8'];
const RRT_DELTA = 0.4;
const RRT_MAX_IT = 2000;
const CMP_SEEDS = 30;
const CMP_VARIANTS: { label: string; p: number; K: number }[] = [
  { label: '均勻 RRT（p = 0）', p: 0, K: 1 },
  { label: '偏置（p = 0.5、K = 4）', p: 0.5, K: 4 },
  { label: '只信位能（p = 1、K = 16）', p: 1, K: 16 },
];

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

const len2 = (v: P2): number => Math.hypot(v[0], v[1]);

function mount(host: HTMLElement, ctx: LessonContext): LessonHandle {
  // ------------------------------------------------------------------ state
  let sceneId: SceneId = 'open';
  let scene: Scene = makeScene(sceneId);
  let groups: ObstacleGroup[] = scene.groups;
  let start: P2 = [...scene.start];
  let goal: P2 = [...scene.goal];
  let mode: Mode = 'field';
  let model: Model = 'first';
  let kAtt = 1, kRep = 1, d0 = 1, damping = 1.5, kappa = 1;
  let pBias = 0.5, Kbias = 4, rrtSeed = 1;
  const show: Show = { heat: true, contours: true, arrows: true, influence: false, basin: false };
  let destroyed = false;
  let curStep = -1;

  let obstacles: Obstacle[] = [];
  let env!: Env; // assigned by rebuildEnv() before first use
  let robot: PointRobot;
  /** Finished segments of the current run (a random kick starts a new segment). */
  let prevSegs: P2[][] = [];
  let prevDist: [number, number][] = [];
  let tOffset = 0;
  let kicks = 0;
  let ghosts: Ghost[] = [];
  let minMarker: { q: P2; U: number } | null = null;
  let runLabel = '';

  // ------------------------------------------------------------------ layout
  simTitle(host, '虛擬位能場實驗台', 'potential field');
  const chips = presetChips(host, [], { label: '預設' });
  const cv = new Canvas2D(host, { aspect: 4 / 3, world: WORLD, margin: 0.02, overlay: true });
  const legendHost = el('div');
  host.appendChild(legendHost);

  // transport for the robot (field / navigation-function modes)
  const transport = el('div', { class: 'mr-playback' });
  host.appendChild(transport);
  const playBtn = button(transport, { label: '▶ 執行', small: true, variant: 'primary', title: '讓機器人沿著 −∇U 移動 / 暫停', onClick: () => togglePlay() });
  button(transport, { label: '⟲ 重設', small: true, title: '機器人回到 S', onClick: () => resetRun(false) });
  const kickBtn = button(transport, { label: '🎲 隨機擾動', small: true, title: '把機器人往隨機方向踢 0.5–1.4 m 後繼續', onClick: () => kick() });
  button(transport, { label: '清除舊軌跡', small: true, onClick: () => { ghosts = []; paintPlot(); cv.requestRender(); } });
  const speedSel = el('select', { title: '模擬速度' });
  for (const sp of [0.5, 1, 2, 4]) speedSel.appendChild(el('option', { value: sp }, `${sp}×`));
  speedSel.value = '1';
  speedSel.addEventListener('change', () => { runner.speed = Number(speedSel.value); });
  transport.appendChild(speedSel);
  const statusEl = el('span', { class: 'mr-note', style: { margin: 0 } }, '');
  transport.appendChild(statusEl);

  // transport for the RRT demo
  const rrtBar = el('div', { class: 'mr-playback' });
  host.appendChild(rrtBar);
  const rrtPlayBtn = button(rrtBar, { label: '▶ 長樹', small: true, variant: 'primary', onClick: () => (rrtRaf ? stopRRT() : startRRT()) });
  button(rrtBar, { label: '⟲ 重來', small: true, onClick: () => { newRRT(); startRRT(); } });
  button(rrtBar, { label: '🎲 換種子', small: true, onClick: () => { rrtSeed = (rrtSeed % 97) + 1; newRRT(); startRRT(); } });
  const cmpBtn = button(rrtBar, { label: `${CMP_SEEDS} 個種子比較`, small: true, title: '三種取樣方式各跑 30 個種子（迭代上限 2000）', onClick: () => startCompare() });
  const rrtStatus = el('span', { class: 'mr-note', style: { margin: 0 } }, '');
  rrtBar.appendChild(rrtStatus);

  const plot = new Plot2D(host, {
    title: '到目標的距離', xLabel: 't (s)', yLabel: '‖q − q_goal‖ (m)', height: 130, legend: true, yRange: [0, 11],
    series: [{ name: '目前', color: '--accent', width: 2 }, { name: '—', color: GHOST_COLORS[0], dash: [5, 4] }, { name: '—', color: GHOST_COLORS[1], dash: [2, 3] }],
  });

  // navigation-function comparison card (step 5; appended below the readouts)
  const navCard = el('div', { class: 'card', style: { padding: '0.5rem 0.7rem', margin: '0.5rem 0' } });
  navCard.appendChild(mdElement('**哪些世界寫得出導航函數？** 左：障礙與邊界都是圓（球形世界），有公式。右：U 形加雜物的一般地圖，沒有通用的構造，只能換成取樣式規劃，或把位能場當成局部的反應層。'));
  const [navL, navR] = split(navCard);
  const thumbL = new Canvas2D(navL, { aspect: 1.6, world: WORLD, margin: 0.03 });
  const thumbR = new Canvas2D(navR, { aspect: 1.6, world: WORLD, margin: 0.03 });

  const panel = controlPanel(host, { compact: true });
  const selModel = select<Model>(panel, {
    label: '機器人模型', value: model,
    options: [{ value: 'first', label: '一階（梯度下降）' }, { value: 'second', label: '二階（慣性＋阻尼）' }],
    onChange: (v) => { model = v; chips.clear(); updateVisibility(); resetRun(true); },
  });
  const sKatt = slider(panel, { label: '吸引增益 k_att', min: 0.2, max: 3, step: 0.1, value: kAtt, onInput: (v) => { kAtt = v; chips.clear(); fieldChanged(); resetRun(true); } });
  const sKrep = slider(panel, { label: '排斥增益 k', min: 0, max: 5, step: 0.05, value: kRep, onInput: (v) => { kRep = v; chips.clear(); fieldChanged(); resetRun(true); } });
  const sD0 = slider(panel, { label: '影響距離 d₀', min: 0.3, max: 2.5, step: 0.05, value: d0, unit: 'm', onInput: (v) => { d0 = v; chips.clear(); fieldChanged(); resetRun(true); } });
  const sDamp = slider(panel, { label: '阻尼 b', min: 0, max: 4, step: 0.1, value: damping, unit: '1/s', onInput: (v) => { damping = v; chips.clear(); resetRun(true); } });
  const sKappa = slider(panel, { label: '導航函數 κ', min: 1, max: 10, step: 0.5, value: kappa, onInput: (v) => { kappa = v; chips.clear(); fieldChanged(); resetRun(true); } });
  const sP = slider(panel, { label: '偏置機率 p', min: 0, max: 1, step: 0.05, value: pBias, onInput: (v) => { pBias = v; chips.clear(); newRRT(); startRRT(); } });
  const sK = slider(panel, { label: '候選數 K', min: 1, max: 16, step: 1, value: Kbias, format: (v) => fmt(v, 0), onInput: (v) => { Kbias = Math.round(v); chips.clear(); newRRT(); startRRT(); } });
  const togRow = row(panel);
  const tHeat = toggle(togRow, { label: '熱圖', value: show.heat, onChange: (v) => { show.heat = v; invalidateStatic(); } });
  const tCont = toggle(togRow, { label: '等高線', value: show.contours, onChange: (v) => { show.contours = v; invalidateStatic(); } });
  const tArrows = toggle(togRow, { label: '梯度方向', value: show.arrows, onChange: (v) => { show.arrows = v; invalidateStatic(); } });
  const tInfl = toggle(togRow, { label: '影響範圍 d₀', value: show.influence, onChange: (v) => { show.influence = v; invalidateStatic(); } });
  const tBasin = toggle(togRow, { label: '會卡住的起點', value: show.basin, onChange: (v) => { show.basin = v; if (v) startBasin(false); else cancelBasin(); updateVisibility(); refreshInfo(); invalidateStatic(); } });

  const info = kv(host, ['狀態', 'U(q)', '‖∇U‖', '到目標距離', '路徑長度／時間', '局部極小 q*', '會卡住的起點', 'RRT', ...CMP_VARIANTS.map((v) => v.label)]);
  host.appendChild(navCard);
  const footNote = note(host, '');

  // preview of the next chapter (step 6)
  const previewBox = el('div', { class: 'card', style: { padding: '0.5rem 0.7rem', margin: '0.5rem 0' } });
  host.appendChild(previewBox);
  previewBox.appendChild(mdElement('**下一章預告：回授控制。** 單一關節 $\\ddot\\theta=u$ 用 PD 控制 $u=K_p(\\theta_d-\\theta)-K_d\\dot\\theta$ 追一個 1 rad 的步階：$K_d$ 太小會超越再折返，夠大則平順收斂，和本課二階模型的阻尼 $b$ 是同一件事。'));
  const pidPlot = new Plot2D(previewBox, {
    xLabel: 't (s)', yLabel: 'θ (rad)', height: 120, legend: true, xRange: [0, 6], yRange: [0, 1.6],
    series: [{ name: 'K_p = 4, K_d = 1', color: '--series-2', width: 2 }, { name: 'K_p = 4, K_d = 4', color: '--series-3', width: 2 }],
  });

  // ------------------------------------------------------------------ field / environment
  const sphereWorld = (): SphereWorld => ({
    center: SPHERE_WORLD.center, R: SPHERE_WORLD.R,
    disks: groups.map((g) => { const o = g.parts[0]; return o.kind === 'disk' ? { c: o.c, r: o.r } : { c: [0, 0] as P2, r: 0 }; }),
  });
  const fparams = (): FieldParams => ({ kAtt, kRep, d0, goal });
  /** The field shown in RRT mode (fixed gains, so only the map matters). */
  const rrtParams = (): FieldParams => ({ kAtt: 1, kRep: 1, d0: 1, goal });

  function rebuildEnv(): void {
    obstacles = groups.flatMap((g) => g.parts);
    env = mode === 'nav' ? navEnv(sphereWorld(), goal, kappa) : fieldEnv(mode === 'rrt' ? rrtParams() : fparams(), obstacles, WORLD);
    if (robot) robot.env = env;
  }

  /** Value used for colouring and contours: normalised log-potential, or φ itself. */
  function displayValueFn(): (x: number, y: number) => number {
    if (mode === 'nav') {
      const w = sphereWorld();
      return (x, y) => {
        const e = navFunction(w, goal, kappa, x, y);
        if (e.free) return e.phi;
        return Math.hypot(x - w.center[0], y - w.center[1]) > w.R ? NaN : 1;
      };
    }
    const p = mode === 'rrt' ? rrtParams() : fparams();
    let far = 0;
    for (const c of [[WORLD.xmin, WORLD.ymin], [WORLD.xmax, WORLD.ymin], [WORLD.xmin, WORLD.ymax], [WORLD.xmax, WORLD.ymax]]) far = Math.max(far, Math.hypot(c[0] - goal[0], c[1] - goal[1]));
    const hiLog = Math.log1p(p.kAtt * far * far);
    const obs = obstacles;
    return (x, y) => Math.min(1, Math.log1p(potentialAt(p, obs, x, y)) / hiLog);
  }

  // ------------------------------------------------------------------ rasters, contours, arrows (cached)
  const heat = new RasterLayer(WORLD, HEAT_NX, HEAT_NY);
  let heatDirty = true;
  let gridDirty = true;
  let contourSegs: number[] = [];
  let influenceSegs: number[] = [];
  let arrowList: number[] = [];

  function paintHeat(): void {
    heatDirty = false;
    const f = displayValueFn();
    // diverging, theme-aware: low values (the valley) tinted blue, high values (the hills) amber,
    // the middle left transparent so contours and arrows stay readable in both themes
    const lo = rgbaOf('--series-1', 1), hi = rgbaOf('--series-4', 1);
    heat.paint(f, {
      colormap: (v) => {
        const t = Math.max(0, Math.min(1, v));
        return t < 0.5 ? [lo[0], lo[1], lo[2], 255 * 0.4 * (1 - 2 * t)] : [hi[0], hi[1], hi[2], 255 * 0.75 * (2 * t - 1)];
      },
      range: [0, 1],
    });
  }

  /** Contours of the display value, the d0 outline of the obstacles and the gradient-direction field. */
  function computeGrid(): void {
    gridDirty = false;
    const f = displayValueFn();
    const vals = new Float64Array(GRID_NX * GRID_NY);
    const dist = new Float64Array(GRID_NX * GRID_NY);
    for (let j = 0; j < GRID_NY; j++) {
      for (let i = 0; i < GRID_NX; i++) {
        const x = WORLD.xmin + (i * (WORLD.xmax - WORLD.xmin)) / (GRID_NX - 1);
        const y = WORLD.ymin + (j * (WORLD.ymax - WORLD.ymin)) / (GRID_NY - 1);
        const v = f(x, y);
        vals[j * GRID_NX + i] = Number.isFinite(v) ? v : 1;
        dist[j * GRID_NX + i] = obstacles.length ? nearestClearance(obstacles, x, y).d : 99;
      }
    }
    contourSegs = [];
    const levels = mode === 'nav' ? [0.02, 0.05, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 0.95, 0.99] : Array.from({ length: 15 }, (_, k) => 0.06 + k * 0.064);
    for (const L of levels) contourSegments(vals, GRID_NX, GRID_NY, WORLD, L, contourSegs);
    influenceSegs = [];
    if (mode === 'field' && obstacles.length) contourSegments(dist, GRID_NX, GRID_NY, WORLD, d0, influenceSegs);
    // direction of −∇U on a coarse grid (fixed length: where would the robot go from here?)
    arrowList = [];
    const L = 0.22;
    const w = mode === 'nav' ? sphereWorld() : null;
    for (let j = 0; j < ARROWS_NY; j++) {
      for (let i = 0; i < ARROWS_NX; i++) {
        const x = WORLD.xmin + ((i + 0.5) * (WORLD.xmax - WORLD.xmin)) / ARROWS_NX;
        const y = WORLD.ymin + ((j + 0.5) * (WORLD.ymax - WORLD.ymin)) / ARROWS_NY;
        let dx: number, dy: number;
        if (w) {
          const e = navFunction(w, goal, kappa, x, y);
          if (!e.free) continue;
          [dx, dy] = e.dir;
        } else {
          if (obstacles.length && nearestClearance(obstacles, x, y).d < 0.12) continue;
          const g = env.field(x, y).g;
          const n = len2(g);
          if (!(n > 1e-9)) continue;
          dx = -g[0] / n; dy = -g[1] / n;
        }
        if (Math.hypot(x - goal[0], y - goal[1]) < 0.25) continue;
        arrowList.push(x - (L / 2) * dx, y - (L / 2) * dy, x + (L / 2) * dx, y + (L / 2) * dy);
      }
    }
  }

  function fieldChanged(): void {
    rebuildEnv();
    heatDirty = true;
    gridDirty = true;
    bgDirty = true;
    minMarker = null;
    invalidateBasin();
    cv.requestRender();
  }

  // ------------------------------------------------------------------ basins ("會卡住的起點")
  const basinLayer = new RasterLayer(WORLD, BASIN_NX, BASIN_NY);
  let basinJob: BasinJob | null = null;
  let basinOutline: number[] = [];
  let basinReady = false;
  let basinRaf = 0;
  let basinTimer = 0;

  function makeBasinJob(): BasinJob {
    if (mode === 'nav') {
      const w = sphereWorld();
      return new BasinJob(WORLD, BASIN_NX, BASIN_NY, navDescentField(w, goal, kappa), (x, y) => !navFunction(w, goal, kappa, x, y).free, goal, { step: 0.08, maxSteps: 1500, goalTol: 0.15 });
    }
    const p = fparams(), obs = obstacles;
    return new BasinJob(WORLD, BASIN_NX, BASIN_NY, (x, y) => { const e = evalField(p, obs, x, y); return { U: e.U, g: e.g }; },
      (x, y) => obs.length > 0 && nearestClearance(obs, x, y).d <= 0.02, goal, { step: 0.1, maxSteps: 1500, goalTol: 0.15 });
  }

  function paintBasin(): void {
    const job = basinJob;
    if (!job) return;
    const col = rgbaOf('--danger', 0.32);
    basinLayer.paint((x, y) => job.cells[job.cellOf(x, y)] === BASIN_STUCK, { color: col });
    // outline of the trapped region (marching squares on the cell centres)
    const mask = new Float64Array(BASIN_NX * BASIN_NY);
    for (let k = 0; k < mask.length; k++) mask[k] = job.cells[k] === BASIN_STUCK ? 1 : 0;
    basinOutline = [];
    contourSegments(mask, BASIN_NX, BASIN_NY, {
      xmin: WORLD.xmin + job.cw / 2, xmax: WORLD.xmax - job.cw / 2, ymin: WORLD.ymin + job.ch / 2, ymax: WORLD.ymax - job.ch / 2,
    }, 0.5, basinOutline);
    bgDirty = true;
  }

  /** Classify every start cell; `sync` finishes now (step entry), otherwise it runs a few ms per frame. */
  function startBasin(sync: boolean): void {
    cancelBasin();
    if (!show.basin || mode === 'rrt') return;
    basinJob = makeBasinJob();
    basinReady = false;
    const pump = (): void => {
      basinRaf = 0;
      if (destroyed || !basinJob) return;
      if (basinJob.run(6)) { basinReady = true; paintBasin(); refreshInfo(); cv.requestRender(); return; }
      refreshInfo();
      basinRaf = requestAnimationFrame(pump);
    };
    if (sync) { basinJob.run(1e9); basinReady = true; paintBasin(); refreshInfo(); cv.requestRender(); }
    else basinRaf = requestAnimationFrame(pump);
  }

  function cancelBasin(): void {
    if (basinRaf) cancelAnimationFrame(basinRaf);
    if (basinTimer) clearTimeout(basinTimer);
    basinRaf = 0; basinTimer = 0; basinJob = null; basinReady = false;
    bgDirty = true;
  }

  /** The field changed: drop the old result and recompute shortly (not on every slider tick). */
  function invalidateBasin(): void {
    cancelBasin();
    if (!show.basin || mode === 'rrt') return;
    basinTimer = window.setTimeout(() => { basinTimer = 0; if (!dragging) startBasin(false); }, 180);
  }

  // ------------------------------------------------------------------ robot run
  const robotParams = () => ({ ...DEFAULT_ROBOT, model: mode === 'nav' ? 'first' as Model : model, damping });

  function describeRun(): string {
    if (mode === 'nav') return `κ = ${fmt(kappa, 1)}`;
    const kTxt = obstacles.length ? ` k=${fmt(kRep, 2)}` : '';
    let s = model === 'first' ? `一階${kTxt}` : `二階 b=${fmt(damping, 1)}${kTxt}`;
    if (Math.abs(kAtt - 1) > 1e-9) s += ` k_att=${fmt(kAtt, 1)}`;
    if (Math.abs(d0 - 1) > 1e-9) s += ` d₀=${fmt(d0, 2)}`;
    return s;
  }

  const allDist = (): [number, number][] => prevDist.concat(robot.dist.map(([t, d]) => [t + tOffset, d] as [number, number]));
  const allSegs = (): P2[][] => [...prevSegs, robot.trace];

  function resetRun(autoStart: boolean): void {
    runner.stop();
    // a run that lasted at least ~1 s stays on screen (dashed) for comparison
    if (robot && robot.trace.length > 20 && mode !== 'rrt') {
      ghosts = [{ label: runLabel, segs: allSegs(), dist: allDist() }, ...ghosts].slice(0, 2);
    }
    robot = new PointRobot(env, robotParams(), start);
    prevSegs = []; prevDist = []; tOffset = 0; kicks = 0; minMarker = null;
    runLabel = describeRun();
    paintPlot();
    if (autoStart && mode !== 'rrt') runner.start();
    syncPlayBtn();
    refreshInfo();
    cv.requestRender();
  }

  function togglePlay(): void {
    if (runner.running) { runner.stop(); syncPlayBtn(); return; }
    if (robot.status !== 'running') resetRun(true);
    else runner.start();
    syncPlayBtn();
  }

  function syncPlayBtn(): void { playBtn.textContent = runner.running ? '⏸ 暫停' : '▶ 執行'; }

  /** Kick the robot 0.5–1.4 m in a random free direction and keep going (a new trace segment). */
  function kick(): void {
    if (mode === 'rrt') return;
    const w = mode === 'nav' ? sphereWorld() : null;
    for (let tries = 0; tries < 60; tries++) {
      const a = Math.random() * TAU, m = 0.5 + 0.9 * Math.random();
      const q: P2 = [robot.q[0] + m * Math.cos(a), robot.q[1] + m * Math.sin(a)];
      if (q[0] < WORLD.xmin + 0.1 || q[0] > WORLD.xmax - 0.1 || q[1] < WORLD.ymin + 0.1 || q[1] > WORLD.ymax - 0.1) continue;
      if (w ? !navFunction(w, goal, kappa, q[0], q[1]).free || env.clearance(q[0], q[1]).d < 0.15 : obstacles.length > 0 && nearestClearance(obstacles, q[0], q[1]).d < 0.15) continue;
      prevSegs.push(robot.trace);
      prevDist = allDist();
      tOffset += robot.t;
      robot = new PointRobot(env, robotParams(), q);
      kicks++;
      minMarker = null;
      runner.start();
      syncPlayBtn();
      return;
    }
  }

  function onRunEnd(): void {
    if (robot.status === 'stuck') {
      const m = refineMinimum(env.field, robot.q);
      // only mark it when the refinement really lands on a critical point near the robot
      if (m.gradNorm < 1e-3 && Math.hypot(m.q[0] - robot.q[0], m.q[1] - robot.q[1]) < 0.6) minMarker = { q: m.q, U: m.U };
      else minMarker = { q: [robot.q[0], robot.q[1]], U: env.field(robot.q[0], robot.q[1]).U };
    }
    syncPlayBtn();
    refreshInfo();
    paintPlot();
  }

  let frameCount = 0;
  const runner = physicsRunner({
    h: 1 / 240,
    budgetMs: 6,
    step: (h) => robot.step(h),
    onFrame: () => {
      if (destroyed) return;
      if (robot.status !== 'running' && runner.running) { runner.stop(); onRunEnd(); }
      frameCount++;
      if (frameCount % 4 === 0) paintPlot();
      refreshInfo();
      cv.requestRender();
    },
  });

  // ------------------------------------------------------------------ RRT demo (step 6)
  let rrt: BiasedRRT | null = null;
  let rrtRaf = 0;
  const rrtFree = (x: number, y: number): boolean => nearestClearance(obstacles, x, y).d > 0.05;

  const endpointsFree = (): boolean => rrtFree(start[0], start[1]) && rrtFree(goal[0], goal[1]);

  function newRRT(): void {
    stopRRT();
    // a tree rooted inside an obstacle (S or G dragged onto one) would be meaningless
    if (mode !== 'rrt' || !endpointsFree()) { rrt = null; refreshInfo(); cv.requestRender(); return; }
    const p = rrtParams(), obs = obstacles;
    rrt = new BiasedRRT(WORLD, rrtFree, (x, y) => potentialAt(p, obs, x, y), start, goal,
      { delta: RRT_DELTA, eps: 0.05, maxIterations: RRT_MAX_IT, pBias, K: Kbias }, mulberry32(rrtSeed * 7919 + 1));
    refreshInfo();
    cv.requestRender();
  }

  function startRRT(): void {
    if (!rrt || rrt.finished || rrtRaf) return;
    const tick = (): void => {
      rrtRaf = 0;
      if (destroyed || !rrt) return;
      for (let k = 0; k < 6 && !rrt.finished; k++) rrt.step();
      refreshInfo();
      cv.requestRender();
      if (!rrt.finished) rrtRaf = requestAnimationFrame(tick);
      rrtPlayBtn.textContent = rrtRaf ? '⏸ 暫停' : '▶ 長樹';
    };
    rrtRaf = requestAnimationFrame(tick);
    rrtPlayBtn.textContent = '⏸ 暫停';
  }

  function stopRRT(): void {
    if (rrtRaf) cancelAnimationFrame(rrtRaf);
    rrtRaf = 0;
    rrtPlayBtn.textContent = '▶ 長樹';
  }

  // 30-seed comparison, a few ms per frame
  interface CmpRow { ok: number; it: number; nodes: number; done: number }
  let cmp: CmpRow[] | null = null;
  let cmpJob: { v: number; s: number } | null = null;
  let cmpRaf = 0;

  function startCompare(): void {
    cancelCompare();
    if (!endpointsFree()) return;
    cmp = CMP_VARIANTS.map(() => ({ ok: 0, it: 0, nodes: 0, done: 0 }));
    cmpJob = { v: 0, s: 1 };
    cmpBtn.disabled = true;
    const p = rrtParams(), obs = obstacles, s0 = start, g0 = goal;
    const U = (x: number, y: number) => potentialAt(p, obs, x, y);
    const pump = (): void => {
      cmpRaf = 0;
      if (destroyed || !cmpJob || !cmp) return;
      const t0 = performance.now();
      while (cmpJob && performance.now() - t0 < 8) {
        const V = CMP_VARIANTS[cmpJob.v];
        const r = new BiasedRRT(WORLD, rrtFree, U, s0, g0, { delta: RRT_DELTA, eps: 0.05, maxIterations: RRT_MAX_IT, pBias: V.p, K: V.K }, mulberry32(cmpJob.s * 7919 + 1)).run();
        const row = cmp[cmpJob.v];
        row.done++;
        if (r.status === 'success') { row.ok++; row.it += r.iterations; row.nodes += r.nodes.length; }
        if (++cmpJob.s > CMP_SEEDS) { cmpJob.s = 1; if (++cmpJob.v >= CMP_VARIANTS.length) cmpJob = null; }
      }
      refreshInfo();
      if (cmpJob) cmpRaf = requestAnimationFrame(pump);
      else cmpBtn.disabled = false;
    };
    cmpRaf = requestAnimationFrame(pump);
    refreshInfo();
  }

  function cancelCompare(): void {
    if (cmpRaf) cancelAnimationFrame(cmpRaf);
    cmpRaf = 0; cmpJob = null; cmpBtn.disabled = false;
  }

  // ------------------------------------------------------------------ readouts
  function statusText(): string {
    if (mode === 'rrt') {
      if (!rrt) return endpointsFree() ? '—' : '✗ S 或 G 落在障礙物裡：把它拖到空地上';
      if (rrt.status === 'success') return `✓ 找到路徑（${rrt.iterations} 次迭代、${rrt.nodes.length} 個節點）`;
      if (rrt.status === 'failed') return `✗ ${RRT_MAX_IT} 次迭代內沒有找到路徑`;
      return `生長中…（${rrt.iterations} 次迭代）`;
    }
    const s = robot.status;
    if (s === 'reached') return `✓ 到達目標（${fmt(tOffset + robot.t, 1)} s）`;
    if (s === 'stuck') return `✗ 卡在局部極小（離目標 ${fmt(robot.distToGoal, 2)} m）${robot.oscillating ? '：一階控制器在極小點兩側來回抖動' : ''}`;
    if (s === 'timeout') return '逾時（60 s）';
    return runner.running ? '前進中…' : '暫停';
  }

  function refreshInfo(): void {
    if (destroyed) return;
    const st = statusText();
    info.set('狀態', st);
    if (mode === 'rrt') {
      if (rrtStatus.textContent !== st) rrtStatus.textContent = st;
      info.set('RRT', rrt ? `${rrt.iterations} 次迭代、${rrt.nodes.length} 個節點（其中 ${rrt.biased} 個樣本經過位能偏置）` : '—');
      CMP_VARIANTS.forEach((v, i) => {
        const r = cmp?.[i];
        info.set(v.label, !r ? '—（按「30 個種子比較」）' : r.done < CMP_SEEDS ? `計算中… ${r.done}/${CMP_SEEDS}` :
          `成功 ${r.ok}/${CMP_SEEDS}${r.ok ? `，平均 ${fmt(r.it / r.ok, 0)} 次迭代、${fmt(r.nodes / r.ok, 0)} 個節點` : ''}`);
      });
    } else {
      if (statusEl.textContent !== st) statusEl.textContent = st;
      const q = robot.q;
      const gTxt = (v: number): string => (v < 1e-3 ? v.toExponential(1) : fmt(v, 4));
      if (mode === 'nav') {
        const e = navFunction(sphereWorld(), goal, kappa, q[0], q[1]);
        info.set('U(q)', fmt(e.phi, 4));
        info.set('‖∇U‖', gTxt(len2(e.g)));
      } else {
        const e = evalField(fparams(), obstacles, q[0], q[1]);
        info.set('U(q)', fmt(e.U, 3));
        info.set('‖∇U‖', `${fmt(len2(e.g), 3)}（吸引 ${fmt(len2(e.gAtt), 2)}、排斥 ${fmt(len2(e.gRep), 2)}）`);
      }
      info.set('到目標距離', `${fmt(robot.distToGoal, 2)} m`);
      info.show('局部極小 q*', !!minMarker);
      if (minMarker) {
        const m = minMarker.q;
        if (mode === 'nav') {
          const e = navFunction(sphereWorld(), goal, kappa, m[0], m[1]);
          info.set('局部極小 q*', `(${fmt(m[0], 2)}, ${fmt(m[1], 2)})：φ = ${fmt(e.phi, 4)}，‖∇φ‖ = ${gTxt(len2(e.g))}`);
        } else {
          const e = evalField(fparams(), obstacles, m[0], m[1]);
          info.set('局部極小 q*', `(${fmt(m[0], 2)}, ${fmt(m[1], 2)})：吸引 ${fmt(len2(e.gAtt), 2)} ＝ 排斥 ${fmt(len2(e.gRep), 2)}，‖∇U‖ = ${gTxt(len2(e.g))}`);
        }
      }
      const L = prevSegs.reduce((a, s) => a + polyLen(s), 0) + robot.pathLength;
      info.set('路徑長度／時間', `${fmt(L, 2)} m ／ ${fmt(tOffset + robot.t, 1)} s${kicks ? `（擾動 ${kicks} 次）` : ''}`);
      if (show.basin) {
        const j = basinJob;
        info.set('會卡住的起點', !j ? '—' : !basinReady ? `計算中… ${fmt(100 * j.progress, 0)} %` :
          `${fmt(100 * (1 - j.goalFraction()), 0)} % 的起點到不了目標；找到 ${j.minima.length} 個局部極小`);
      }
    }
    const e2 = mode === 'nav' ? `φ = ${fmt(navFunction(sphereWorld(), goal, kappa, robot.q[0], robot.q[1]).phi, 3)}` : '';
    cv.setOverlayText(mode === 'rrt'
      ? `RRT · p = ${fmt(pBias, 2)} · K = ${Kbias}`
      : `${mode === 'nav' ? '導航函數' : model === 'first' ? '一階' : '二階'} · t = ${fmt(tOffset + robot.t, 1)} s${e2 ? ` · ${e2}` : ''}`);
  }

  const polyLen = (s: P2[]): number => { let L = 0; for (let i = 1; i < s.length; i++) L += Math.hypot(s[i][0] - s[i - 1][0], s[i][1] - s[i - 1][1]); return L; };

  function paintPlot(): void {
    const series = [{ name: `目前：${runLabel}`, color: '--accent', width: 2 }, ...ghosts.map((g, i) => ({ name: g.label, color: GHOST_COLORS[i], dash: i ? [2, 3] : [5, 4], width: 1.6 }))];
    while (series.length < 3) series.push({ name: '—', color: '--fg-muted', dash: [2, 3], width: 1 });
    plot.setOptions({ series });
    const d = allDist();
    plot.setXY(0, d.map((p) => p[0]), d.map((p) => p[1]));
    for (let i = 0; i < 2; i++) {
      const g = ghosts[i];
      plot.setXY(i + 1, g ? g.dist.map((p) => p[0]) : [], g ? g.dist.map((p) => p[1]) : []);
    }
    let tmax = 6, dmax = 1;
    for (const s of [d, ...ghosts.map((g) => g.dist)]) for (const [t, v] of s) { tmax = Math.max(tmax, t); dmax = Math.max(dmax, v); }
    plot.setRanges([0, Math.ceil(tmax)], [0, Math.ceil(dmax * 1.05)]);
  }

  function updateVisibility(): void {
    const vis = (e: HTMLElement, v: boolean) => { e.style.display = v ? '' : 'none'; };
    const fieldMode = mode === 'field', navMode = mode === 'nav', rrtMode = mode === 'rrt';
    vis(transport, !rrtMode);
    vis(rrtBar, rrtMode);
    vis(plot.el, !rrtMode);
    vis(navCard, navMode);
    vis(selModel.el, fieldMode);
    vis(sKatt.el, fieldMode);
    vis(sKrep.el, fieldMode);
    vis(sD0.el, fieldMode);
    vis(sDamp.el, fieldMode && model === 'second');
    vis(sKappa.el, navMode);
    vis(sP.el, rrtMode);
    vis(sK.el, rrtMode);
    vis(tInfl.el, fieldMode);
    vis(tBasin.el, !rrtMode);
    vis(tArrows.el, !rrtMode);
    vis(kickBtn, !rrtMode);
    for (const k of ['U(q)', '‖∇U‖', '到目標距離', '路徑長度／時間']) info.show(k, !rrtMode);
    info.setLabel('U(q)', navMode ? 'φ(q)' : 'U(q)');
    info.setLabel('‖∇U‖', navMode ? '‖∇φ‖' : '‖∇U‖');
    if (rrtMode) info.show('局部極小 q*', false);
    info.show('會卡住的起點', !rrtMode && show.basin);
    info.show('RRT', rrtMode);
    for (const v of CMP_VARIANTS) info.show(v.label, rrtMode);
    // legend
    const items: { color: string; label: string }[] = [
      { color: 'var(--series-6)', label: 'S 起點（可拖曳）' },
      { color: 'var(--ok)', label: 'G 目標（可拖曳）' },
    ];
    if (!rrtMode) {
      items.push({ color: 'var(--accent)', label: '機器人與軌跡' });
      if (fieldMode) items.push({ color: 'var(--series-3)', label: '吸引力 −∇U_att' }, { color: 'var(--series-2)', label: '排斥力 −∇U_rep' });
      items.push({ color: 'var(--danger)', label: '局部極小／會卡住的起點' });
    } else {
      items.push({ color: 'var(--series-3)', label: 'RRT 樹' }, { color: 'var(--series-7)', label: '偏置樣本' }, { color: 'var(--warn)', label: '路徑' });
    }
    items.push({ color: 'var(--series-1)', label: navMode ? '熱圖：φ 低（谷）' : '熱圖：U 低（谷）' }, { color: 'var(--series-4)', label: navMode ? 'φ 高（山，邊界 φ = 1）' : 'U 高（山，對數刻度）' });
    legendHost.replaceChildren();
    legend(legendHost, items);
    footNote.textContent = rrtMode
      ? `RRT 步長 δ = ${RRT_DELTA} m；「偏置」樣本是 K 個均勻樣本中位能最低者（位能用 k_att = k = 1、d₀ = 1 m）。`
      : navMode
        ? '球形世界：外圈是工作區邊界、三個圓是障礙（都可拖曳）。機器人沿 −∇φ 的方向以 v_max 前進，靠近目標才減速；φ 在遠處非常平，所以只看方向。'
        : '點機器人，速度上限 v_max = 1.5 m/s；一階模型每 0.1 s 讀一次梯度，二階模型以 1/240 s 的固定子步積分，力的大小裁剪在 4 m/s²。排斥項的距離下限夾在 3 cm。拖曳灰色障礙上的把手可以移動它。';
  }

  // ------------------------------------------------------------------ drawing
  const font = (px: number, bold = false): string => `${bold ? 'bold ' : ''}${px}px ${cssVar('--font') || 'sans-serif'}`;
  type ToScreen = (p: P2) => P2;

  // Static layers (grid, heat map, basins, contours, d0 outline, obstacles, direction field) are
  // rendered into an off-screen canvas only when one of them changes; an animation frame then costs
  // one drawImage plus the robot. Re-stroking thousands of contour segments every frame starves
  // software-rendered GPUs (the page stalled for seconds in headless Chromium).
  const bg = document.createElement('canvas');
  const bgc = bg.getContext('2d')!;
  let bgDirty = true;
  let bgKey = '';
  function invalidateStatic(): void { bgDirty = true; cv.requestRender(); }

  function strokeFlat(g: CanvasRenderingContext2D, T: ToScreen, segs: ArrayLike<number>, color: string, width: number, alpha: number, dash: number[] = []): void {
    if (!segs.length) return;
    g.save();
    g.strokeStyle = themeColor(color, '#888'); g.lineWidth = width; g.globalAlpha = alpha; g.setLineDash(dash); g.lineCap = 'round';
    g.beginPath();
    for (let k = 0; k + 3 < segs.length; k += 4) {
      const [ax, ay] = T([segs[k], segs[k + 1]]);
      const [bx, by] = T([segs[k + 2], segs[k + 3]]);
      g.moveTo(ax, ay); g.lineTo(bx, by);
    }
    g.stroke();
    g.restore();
  }

  function drawArrowsField(g: CanvasRenderingContext2D, T: ToScreen): void {
    if (!arrowList.length) return;
    g.save();
    g.strokeStyle = themeColor('--fg-muted'); g.fillStyle = themeColor('--fg-muted');
    g.globalAlpha = 0.55; g.lineWidth = 1.1;
    g.beginPath();
    const heads: number[] = [];
    for (let k = 0; k + 3 < arrowList.length; k += 4) {
      const [ax, ay] = T([arrowList[k], arrowList[k + 1]]);
      const [bx, by] = T([arrowList[k + 2], arrowList[k + 3]]);
      g.moveTo(ax, ay); g.lineTo(bx, by);
      heads.push(ax, ay, bx, by);
    }
    g.stroke();
    g.beginPath();
    for (let k = 0; k < heads.length; k += 4) {
      const ax = heads[k], ay = heads[k + 1], bx = heads[k + 2], by = heads[k + 3];
      const L = Math.hypot(bx - ax, by - ay) || 1, ux = (bx - ax) / L, uy = (by - ay) / L, h = 4;
      g.moveTo(bx, by);
      g.lineTo(bx - ux * h - uy * h * 0.55, by - uy * h + ux * h * 0.55);
      g.lineTo(bx - ux * h + uy * h * 0.55, by - uy * h - ux * h * 0.55);
      g.closePath();
    }
    g.fill();
    g.restore();
  }

  function drawObstaclesTo(g: CanvasRenderingContext2D, T: ToScreen, scale: number): void {
    g.save();
    g.fillStyle = themeColor('--fg-muted'); g.strokeStyle = themeColor('--fg'); g.lineWidth = 1; g.globalAlpha = 0.85;
    for (const o of obstacles) {
      g.beginPath();
      if (o.kind === 'disk') { const [x, y] = T(o.c); g.arc(x, y, o.r * scale, 0, TAU); }
      else o.pts.forEach((p, i) => { const [x, y] = T(p); if (i) g.lineTo(x, y); else g.moveTo(x, y); });
      g.closePath();
      g.fill(); g.stroke();
    }
    g.restore();
  }

  function drawSphereBoundary(g: CanvasRenderingContext2D, T: ToScreen, scale: number): void {
    const w = SPHERE_WORLD;
    const [cx, cy] = T(w.center);
    const [x0, y0] = T([WORLD.xmin, WORLD.ymax]);
    const [x1, y1] = T([WORLD.xmax, WORLD.ymin]);
    g.save();
    g.fillStyle = themeColor('--fg-muted'); g.globalAlpha = 0.28;
    g.beginPath();
    g.rect(x0, y0, x1 - x0, y1 - y0);
    g.arc(cx, cy, w.R * scale, 0, TAU, true);
    g.fill('evenodd');
    g.globalAlpha = 1;
    g.strokeStyle = themeColor('--fg'); g.lineWidth = 2.5;
    g.beginPath(); g.arc(cx, cy, w.R * scale, 0, TAU); g.stroke();
    g.restore();
  }

  function drawRasterTo(g: CanvasRenderingContext2D, T: ToScreen, layer: RasterLayer, alpha: number, smooth: boolean): void {
    const [x0, y0] = T([layer.world.xmin, layer.world.ymax]);
    const [x1, y1] = T([layer.world.xmax, layer.world.ymin]);
    g.save();
    g.globalAlpha = alpha;
    g.imageSmoothingEnabled = smooth;
    g.drawImage(layer.canvas, x0, y0, x1 - x0, y1 - y0);
    g.restore();
  }

  function drawGridTo(g: CanvasRenderingContext2D, T: ToScreen): void {
    g.save();
    g.strokeStyle = themeColor('--grid', '#e5e5e5'); g.lineWidth = 1;
    g.beginPath();
    for (let x = WORLD.xmin; x <= WORLD.xmax + 1e-9; x += 1) { const [a, b] = T([x, WORLD.ymin]); const [, d] = T([x, WORLD.ymax]); g.moveTo(Math.round(a) + 0.5, b); g.lineTo(Math.round(a) + 0.5, d); }
    for (let y = WORLD.ymin; y <= WORLD.ymax + 1e-9; y += 1) { const [a, b] = T([WORLD.xmin, y]); const [c2] = T([WORLD.xmax, y]); g.moveTo(a, Math.round(b) + 0.5); g.lineTo(c2, Math.round(b) + 0.5); }
    g.stroke();
    g.restore();
  }

  /** Re-render the static layers when they changed or the canvas was resized. */
  function renderStatic(c: Canvas2D): void {
    const key = `${c.canvas.width}x${c.canvas.height}:${c.scale.toFixed(5)}`;
    if (!bgDirty && key === bgKey) return;
    if (heatDirty) paintHeat();
    if (gridDirty) computeGrid();
    bgDirty = false;
    bgKey = key;
    if (bg.width !== c.canvas.width || bg.height !== c.canvas.height) { bg.width = c.canvas.width; bg.height = c.canvas.height; }
    const g = bgc;
    g.setTransform(1, 0, 0, 1, 0, 0);
    g.clearRect(0, 0, bg.width, bg.height);
    const dpr = c.canvas.width / Math.max(1, c.width);
    g.setTransform(dpr, 0, 0, dpr, 0, 0);
    const T: ToScreen = (p) => c.toScreen(p);
    drawGridTo(g, T);
    g.save();
    if (mode === 'nav') {
      // the sphere world ends at its circle: keep the rasters inside it
      const [cx, cy] = T(SPHERE_WORLD.center);
      g.beginPath(); g.arc(cx, cy, SPHERE_WORLD.R * c.scale, 0, TAU); g.clip();
    }
    if (show.heat) drawRasterTo(g, T, heat, mode === 'rrt' ? 0.55 : 1, true);
    if (show.basin && basinReady && mode !== 'rrt') {
      drawRasterTo(g, T, basinLayer, 1, false);
      strokeFlat(g, T, basinOutline, '--danger', 2, 0.9);
    }
    g.restore();
    if (show.contours) strokeFlat(g, T, contourSegs, '--fg-muted', 1, 0.5);
    if (mode === 'nav') drawSphereBoundary(g, T, c.scale);
    if (show.influence && mode === 'field') strokeFlat(g, T, influenceSegs, '--fg', 1.4, 0.7, [5, 4]);
    drawObstaclesTo(g, T, c.scale);
    if (show.arrows && mode !== 'rrt') drawArrowsField(g, T);
  }

  function drawTrace(c: Canvas2D, segs: P2[][], color: string, width: number, alpha: number, dash: number[] = []): void {
    for (const s of segs) c.polyline(s, { color, width, alpha, dash });
  }

  function drawRobot(c: Canvas2D): void {
    const q = robot.q;
    // forces at the robot: attraction (green), repulsion (red), their sum (dark). Once the run is
    // stuck they are drawn at the refined minimum q*, where they cancel exactly (the first-order
    // controller itself keeps hopping a few cm around q*).
    if (mode === 'field') {
      const at: P2 = minMarker && robot.status === 'stuck' ? minMarker.q : q;
      const e = evalField(fparams(), obstacles, at[0], at[1]);
      const Fa: P2 = [-e.gAtt[0], -e.gAtt[1]], Fr: P2 = [-e.gRep[0], -e.gRep[1]];
      const m = Math.max(len2(Fa), len2(Fr), 0.8);
      const s = 1.1 / m;
      const tip = (F: P2): P2 => [at[0] + s * F[0], at[1] + s * F[1]];
      if (len2(Fa) * s > 0.04) c.arrow(at, tip(Fa), { color: '--series-3', width: 3, head: 9 });
      if (len2(Fr) * s > 0.04) c.arrow(at, tip(Fr), { color: '--series-2', width: 3, head: 9 });
      const Ft: P2 = [Fa[0] + Fr[0], Fa[1] + Fr[1]];
      if (len2(Fr) > 1e-3 && len2(Ft) * s > 0.04) c.arrow(at, tip(Ft), { color: '--fg', width: 1.6, head: 7, dash: [4, 3] });
    } else if (robot.status === 'running') {
      const v = robot.cmd;
      if (len2(v) > 1e-6) c.arrow(q, [q[0] + 0.6 * v[0] / len2(v), q[1] + 0.6 * v[1] / len2(v)], { color: '--fg', width: 2, head: 8 });
    }
    c.disk(q, 0.13, { fill: '--accent' });
    c.circle(q, 0.13, { color: '--bg-elev', width: 2 });
  }

  function drawMinMarker(c: Canvas2D): void {
    if (minMarker) {
      const [x, y] = c.toScreen(minMarker.q);
      const g = c.ctx;
      g.save(); g.strokeStyle = themeColor('--danger'); g.lineWidth = 2.5; g.beginPath(); g.arc(x, y, 15, 0, TAU); g.stroke(); g.restore();
      const below = minMarker.q[1] > WORLD.ymax - 1;
      c.text(minMarker.q, '局部極小：∇U = 0', { dy: below ? 30 : -30, color: '--danger', font: font(12, true), bg: '--bg-elev' });
    }
    if (show.basin && basinReady && basinJob) {
      for (const m of basinJob.minima) {
        if (minMarker && Math.hypot(m[0] - minMarker.q[0], m[1] - minMarker.q[1]) < 0.3) continue;
        const [x, y] = c.toScreen(m);
        const g = c.ctx;
        g.save(); g.strokeStyle = themeColor('--danger'); g.lineWidth = 2.2; g.beginPath();
        g.moveTo(x - 6, y - 6); g.lineTo(x + 6, y + 6); g.moveTo(x - 6, y + 6); g.lineTo(x + 6, y - 6); g.stroke(); g.restore();
      }
    }
  }

  function drawRRT(c: Canvas2D): void {
    if (!rrt) return;
    const g = c.ctx;
    g.save();
    g.strokeStyle = themeColor('--series-3'); g.lineWidth = 1.3; g.globalAlpha = 0.9;
    g.beginPath();
    for (let i = 1; i < rrt.nodes.length; i++) {
      const [ax, ay] = c.toScreen(rrt.nodes[rrt.parent[i]]);
      const [bx, by] = c.toScreen(rrt.nodes[i]);
      g.moveTo(ax, ay); g.lineTo(bx, by);
    }
    g.stroke();
    g.fillStyle = themeColor('--series-3');
    g.beginPath();
    for (const p of rrt.nodes) { const [x, y] = c.toScreen(p); g.moveTo(x + 1.8, y); g.arc(x, y, 1.8, 0, TAU); }
    g.fill();
    g.restore();
    if (rrt.lastSample && !rrt.finished) {
      const [x, y] = c.toScreen(rrt.lastSample);
      g.save(); g.strokeStyle = themeColor(rrt.lastBiased ? '--series-7' : '--fg-muted'); g.lineWidth = 2;
      g.beginPath(); g.arc(x, y, 6, 0, TAU); g.stroke(); g.restore();
    }
    if (rrt.path) c.polyline(rrt.path, { color: '--warn', width: 3.5 });
  }

  cv.draw = (c) => {
    renderStatic(c);
    c.ctx.drawImage(bg, 0, 0, c.width, c.height);
    if (mode === 'rrt') { drawRRT(c); return; }
    ghosts.forEach((gh, i) => drawTrace(c, gh.segs, GHOST_COLORS[i], 2, 0.85, i ? [2, 3] : [6, 4]));
    drawTrace(c, prevSegs, '--accent', 2.4, 0.9);
    // dashed jumps where the robot was kicked
    for (let i = 0; i < prevSegs.length; i++) {
      const a = prevSegs[i][prevSegs[i].length - 1];
      const b = i + 1 < prevSegs.length ? prevSegs[i + 1][0] : robot.trace[0];
      c.arrow(a, b, { color: '--warn', width: 1.8, dash: [4, 3], head: 8 });
    }
    c.polyline(robot.trace.concat([robot.q]), { color: '--accent', width: 2.4 });
    if (robot.params.model === 'first' && mode === 'field') {
      // one dot per control update: the discrete steps of the controller
      const g = c.ctx;
      g.save(); g.fillStyle = themeColor('--accent'); g.beginPath();
      const tk = robot.ticks;
      for (let i = Math.max(0, tk.length - 400); i < tk.length; i++) { const [x, y] = c.toScreen(tk[i]); g.moveTo(x + 1.7, y); g.arc(x, y, 1.7, 0, TAU); }
      g.fill(); g.restore();
    }
    drawMinMarker(c);
    drawRobot(c);
  };

  // ------------------------------------------------------------------ handles (S, G, obstacle groups)
  let dragging = false;
  let groupHandles: Handle[] = [];
  const sHandle = cv.addHandle({
    get: () => start, color: '--series-6', label: 'S', radius: 9,
    set: (p) => { start = clampWorld(p); if (mode !== 'rrt') { runner.stop(); robot = new PointRobot(env, robotParams(), start); prevSegs = []; prevDist = []; tOffset = 0; minMarker = null; } cv.requestRender(); },
    onStart: () => { dragging = true; chips.clear(); runner.stop(); syncPlayBtn(); },
    onEnd: () => { dragging = false; afterDrag(false); },
  });
  const gHandle = cv.addHandle({
    get: () => goal, color: '--ok', label: 'G', radius: 9,
    set: (p) => { goal = clampWorld(p); fieldChanged(); },
    onStart: () => { dragging = true; chips.clear(); runner.stop(); syncPlayBtn(); },
    onEnd: () => { dragging = false; afterDrag(true); },
  });
  void gHandle;

  function clampWorld(p: P2): P2 {
    return [Math.min(WORLD.xmax - 0.15, Math.max(WORLD.xmin + 0.15, p[0])), Math.min(WORLD.ymax - 0.15, Math.max(WORLD.ymin + 0.15, p[1]))];
  }

  /** Handle of a group: the centre of its first part (for the U: its top arm, off the symmetry line). */
  function anchorOf(gr: ObstacleGroup): P2 {
    const o = gr.parts[0];
    if (o.kind === 'disk') return [o.c[0], o.c[1]];
    let x = 0, y = 0;
    for (const p of o.pts) { x += p[0]; y += p[1]; }
    return [x / o.pts.length, y / o.pts.length];
  }

  function rebuildGroupHandles(): void {
    for (const h of groupHandles) cv.removeHandle(h);
    groupHandles = groups.map((gr, gi) => cv.addHandle({
      get: () => anchorOf(groups[gi]), color: '--fg-muted', radius: 7,
      set: (p) => {
        const a = anchorOf(groups[gi]);
        const q = clampWorld(p);
        const dx = q[0] - a[0], dy = q[1] - a[1];
        groups[gi] = { name: groups[gi].name, parts: groups[gi].parts.map((o) => translateObstacle(o, dx, dy)) };
        fieldChanged();
      },
      onStart: () => { dragging = true; chips.clear(); runner.stop(); stopRRT(); syncPlayBtn(); },
      onEnd: () => { dragging = false; afterDrag(true); },
    }));
    // keep S and G on top of the obstacle handles
    cv.removeHandle(sHandle); cv.removeHandle(gHandle);
    cv.addHandle(sHandle); cv.addHandle(gHandle);
  }

  function afterDrag(fieldMoved: boolean): void {
    if (fieldMoved) { fieldChanged(); }
    if (mode === 'rrt') {
      // a comparison computed on the old map would be misleading
      cancelCompare();
      cmp = null;
      newRRT();
      startRRT();
      return;
    }
    resetRun(true);
  }

  // ------------------------------------------------------------------ scene / step configuration
  function setScene(id: SceneId, m: Mode): void {
    const changed = id !== sceneId || m !== mode;
    sceneId = id;
    mode = m;
    scene = makeScene(id);
    groups = scene.groups.map((gr) => ({ name: gr.name, parts: gr.parts.slice() }));
    start = [...scene.start];
    goal = [...scene.goal];
    if (changed) ghosts = [];
    stopRRT();
    cancelCompare();
    cmp = null;
    rebuildEnv();
    // a fresh robot: the previous scene's run must not turn into a comparison ghost here
    runner.stop();
    robot = new PointRobot(env, robotParams(), start);
    prevSegs = []; prevDist = []; tOffset = 0; kicks = 0; minMarker = null;
    rebuildGroupHandles();
    heatDirty = true;
    gridDirty = true;
    bgDirty = true;
    updateVisibility();
  }

  const setParams = (o: { kAtt?: number; kRep?: number; d0?: number; damping?: number; kappa?: number; model?: Model }): void => {
    if (o.kAtt !== undefined) { kAtt = o.kAtt; sKatt.set(kAtt, { silent: true }); }
    if (o.kRep !== undefined) { kRep = o.kRep; sKrep.set(kRep, { silent: true }); }
    if (o.d0 !== undefined) { d0 = o.d0; sD0.set(d0, { silent: true }); }
    if (o.damping !== undefined) { damping = o.damping; sDamp.set(damping, { silent: true }); }
    if (o.kappa !== undefined) { kappa = o.kappa; sKappa.set(kappa, { silent: true }); }
    if (o.model !== undefined) { model = o.model; selModel.set(model, { silent: true }); }
    updateVisibility();
    fieldChanged();
  };

  const setShow = (o: Partial<Show>): void => {
    Object.assign(show, o);
    tHeat.set(show.heat, { silent: true });
    tCont.set(show.contours, { silent: true });
    tArrows.set(show.arrows, { silent: true });
    tInfl.set(show.influence, { silent: true });
    tBasin.set(show.basin, { silent: true });
    updateVisibility();
  };

  /** Re-run on the corridor map with new settings (keeps the previous runs as dashed ghosts). */
  const corridorRun = (o: Parameters<typeof setParams>[0]): void => {
    if (sceneId !== 'corridor') { setScene('corridor', 'field'); }
    setParams({ kAtt: 1, d0: 1, ...o });
    resetRun(true);
  };

  /** Run a whole simulation instantly and keep it as a ghost (for a ready-made comparison). */
  function precomputeGhost(): void {
    const r = new PointRobot(env, robotParams(), start).run();
    ghosts = [{ label: describeRun(), segs: [r.trace], dist: r.dist }, ...ghosts].slice(0, 2);
  }

  interface StepCfg { enter: () => void; chips: () => Preset[]; focus: () => HTMLElement[] }
  const STEPS: StepCfg[] = [
    {
      enter: () => {
        setScene('open', 'field');
        setShow({ heat: true, contours: true, arrows: true, influence: false, basin: false });
        setParams({ kAtt: 1, kRep: 1, d0: 1, model: 'first', damping: 1.5 });
        resetRun(true);
      },
      chips: () => [
        { label: '重新出發', apply: () => resetRun(true) },
        { label: 'S 移到左上角', apply: () => { start = [0.9, 6.8]; resetRun(true); } },
        { label: 'S 移到右下角', apply: () => { start = [9.4, 0.7]; resetRun(true); } },
        { label: 'k_att = 0.4（平緩）', apply: () => { setParams({ kAtt: 0.4 }); resetRun(true); } },
        { label: 'k_att = 2（陡峭）', apply: () => { setParams({ kAtt: 2 }); resetRun(true); } },
      ],
      focus: () => [cv.el, sKatt.el],
    },
    {
      enter: () => {
        setScene('convex', 'field');
        setShow({ heat: true, contours: true, arrows: true, influence: true, basin: false });
        setParams({ kAtt: 1, kRep: 1, d0: 1, model: 'first', damping: 1.5 });
        resetRun(true);
      },
      chips: () => [
        { label: 'd₀ = 0.5 m', apply: () => { setParams({ d0: 0.5 }); resetRun(true); } },
        { label: 'd₀ = 1.5 m', apply: () => { setParams({ d0: 1.5 }); resetRun(true); } },
        { label: 'k = 0.2', apply: () => { setParams({ kRep: 0.2 }); resetRun(true); } },
        { label: 'k = 3', apply: () => { setParams({ kRep: 3 }); resetRun(true); } },
      ],
      focus: () => [cv.el, sD0.el],
    },
    {
      enter: () => {
        setScene('utrap', 'field');
        setShow({ heat: true, contours: true, arrows: true, influence: false, basin: true });
        setParams({ kAtt: 1, kRep: 1, d0: 1, model: 'first', damping: 1.5 });
        startBasin(true);
        resetRun(true);
      },
      chips: () => [
        { label: '重新出發', apply: () => { start = [...makeScene('utrap').start]; resetRun(true); } },
        { label: '🎲 隨機擾動', apply: () => kick() },
        { label: 'k_att = 2（拉得更用力）', apply: () => { setParams({ kAtt: 2 }); resetRun(true); } },
        { label: 'S 移到左上方', apply: () => { start = [1.0, 6.9]; resetRun(true); } },
      ],
      focus: () => [cv.el, tBasin.el, kickBtn],
    },
    {
      enter: () => {
        setScene('corridor', 'field');
        setShow({ heat: true, contours: true, arrows: false, influence: false, basin: false });
        setParams({ kAtt: 1, kRep: 0.3, d0: 1, model: 'first', damping: 1.5 });
        precomputeGhost();
        setParams({ kRep: 1.5 });
        resetRun(true);
      },
      chips: () => [
        { label: '一階 k = 0.3', apply: () => corridorRun({ model: 'first', kRep: 0.3 }) },
        { label: '一階 k = 1.5（鋸齒）', apply: () => corridorRun({ model: 'first', kRep: 1.5 }) },
        { label: '一階 k = 3（進不去）', apply: () => corridorRun({ model: 'first', kRep: 3 }) },
        { label: '二階 b = 1.5', apply: () => corridorRun({ model: 'second', kRep: 1, damping: 1.5 }) },
        { label: '二階 b = 0.8（慣性）', apply: () => corridorRun({ model: 'second', kRep: 1, damping: 0.8 }) },
        { label: '二階 b = 0.5（空地）', apply: () => { setScene('open', 'field'); setParams({ kAtt: 1, kRep: 1, d0: 1, model: 'second', damping: 0.5 }); resetRun(true); } },
      ],
      focus: () => [cv.el, sKrep.el, selModel.el, plot.el],
    },
    {
      enter: () => {
        setScene('sphere', 'nav');
        setShow({ heat: true, contours: true, arrows: true, influence: false, basin: true });
        setParams({ kappa: 1, model: 'first' });
        startBasin(true);
        resetRun(true);
      },
      chips: () => [1, 2, 3, 4, 8].map((k): Preset => ({ label: `κ = ${k}`, apply: () => { setParams({ kappa: k }); startBasin(true); resetRun(true); } })),
      focus: () => [sKappa.el, cv.el],
    },
    {
      enter: () => {
        setScene('clutter', 'rrt');
        setShow({ heat: true, contours: false, arrows: false, influence: false, basin: false });
        pBias = 0.5; Kbias = 4; rrtSeed = 1;
        sP.set(pBias, { silent: true }); sK.set(Kbias, { silent: true });
        fieldChanged();
        runner.stop();
        newRRT();
        for (let k = 0; k < 80 && rrt && !rrt.finished; k++) rrt.step();
        startRRT();
      },
      chips: () => [
        ...CMP_VARIANTS.map((v): Preset => ({ label: v.label, apply: () => { pBias = v.p; Kbias = v.K; sP.set(v.p, { silent: true }); sK.set(v.K, { silent: true }); newRRT(); startRRT(); } })),
        { label: `${CMP_SEEDS} 個種子比較`, apply: () => startCompare() },
      ],
      focus: () => [rrtBar, info.el],
    },
  ];

  let focused: HTMLElement[] = [];
  function onStep(i: number): void {
    const k = Math.max(0, Math.min(STEPS.length - 1, i));
    if (k === curStep) return;
    curStep = k;
    cancelBasin();
    stopRRT();
    cancelCompare();
    const cfg = STEPS[k];
    cfg.enter();
    chips.setPresets(cfg.chips());
    previewBox.style.display = k === STEPS.length - 1 ? '' : 'none';
    focused.forEach((f) => f.classList.remove('mr-focus'));
    focused = cfg.focus();
    focused.forEach((f) => f.classList.add('mr-focus'));
    refreshInfo();
    cv.requestRender();
  }

  // ------------------------------------------------------------------ static pictures
  thumbL.draw = (c) => {
    const w = SPHERE_WORLD;
    c.circle(w.center, w.R, { color: '--fg', width: 2 });
    for (const d of w.disks) c.circle(d.c, d.r, { fill: '--fg-muted', color: '--fg', width: 1, alpha: 0.85 });
    c.text([5, 0.25], '球形世界：有公式 ✓', { color: '--ok', font: font(12, true), bg: '--bg-elev' });
  };
  thumbR.draw = (c) => {
    for (const o of makeScene('clutter').groups.flatMap((g) => g.parts)) {
      if (o.kind === 'disk') c.circle(o.c, o.r, { fill: '--fg-muted', color: '--fg', width: 1, alpha: 0.85 });
      else c.polygon(o.pts, { fill: '--fg-muted', color: '--fg', width: 1, alpha: 0.85 });
    }
    c.polygon([[0, 0], [10, 0], [10, 7.5], [0, 7.5]], { color: '--fg', width: 2 });
    c.text([5, 0.35], '一般地圖：沒有通用構造 ✗', { color: '--danger', font: font(12, true), bg: '--bg-elev' });
  };

  // PD step responses of a unit-inertia joint (preview of the next chapter), simulated once
  function pidResponse(kp: number, kd: number): [number[], number[]] {
    const pid = createPID({ kp, ki: 0, kd, derivativeOnMeasurement: true });
    const dt = 0.005;
    let x = [0, 0];
    const ts: number[] = [], ys: number[] = [];
    for (let t = 0; t <= 6 + 1e-9; t += dt) {
      ts.push(t); ys.push(x[0]);
      const u = pid.update(1 - x[0], dt, x[0]);
      x = rk4Step((_t, s) => [s[1], u], t, x, dt);
    }
    return [ts, ys];
  }
  const pdA = pidResponse(4, 1), pdB = pidResponse(4, 4);
  pidPlot.setXY(0, pdA[0], pdA[1]);
  pidPlot.setXY(1, pdB[0], pdB[1]);
  pidPlot.setMarkers([{ y: 1, label: 'θ_d', color: '--fg-muted', dash: [4, 4] }]);

  // ------------------------------------------------------------------ boot
  rebuildEnv();
  robot = new PointRobot(env, robotParams(), start);
  const offTheme = ctx.onThemeChange(() => {
    heatDirty = true;
    bgDirty = true;
    if (basinReady) paintBasin();
    cv.requestRender();
  });
  onStep(ctx.currentStep);

  return {
    onStep,
    destroy() {
      destroyed = true;
      runner.stop();
      cancelBasin();
      stopRRT();
      cancelCompare();
      offTheme();
      focused.forEach((f) => f.classList.remove('mr-focus'));
      cv.destroy(); thumbL.destroy(); thumbR.destroy(); plot.destroy(); pidPlot.destroy();
    },
  };
}

const lesson: Lesson = { meta, steps, quiz, mount };
export default lesson;
