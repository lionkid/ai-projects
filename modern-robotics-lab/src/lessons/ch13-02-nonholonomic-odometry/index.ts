import type { Lesson, LessonContext, LessonHandle } from '@/types';
import {
  el, fmt, fmtDeg, controlPanel, row, slider, select, button, matrixView, playback, simTitle, note, legend, kv,
  split, mdElement, mathEl, Canvas2D, Plot2D, physicsRunner, keyboardDrive, presetChips,
  type Handle, type Preset, type Pt,
} from '@/ui';
import { matVec, wrapToPi } from '@/mr';
import { meta } from './meta';
import { quiz, steps } from './content';
import {
  Gdiff, Gunicycle, WheelOdometry, bracketPrediction, bracketSequence, clampInput, headingDriftPrediction, inputSetPolys,
  parallelParkPlan, planDuration, poseAlong, poseError, samplePlan, steeringAngle, stepPose, vwToWheels,
  type DiffParams, type Limits, type ModelKey, type OdoRule, type Pose, type Segment,
} from './mobile';

// ---------------------------------------------------------------------------------------------
// Settings
// ---------------------------------------------------------------------------------------------

const MAP = { xmin: -3.0, xmax: 3.0, ymin: -1.5, ymax: 1.5 };
const WALL = 0.28;                 // the reference point keeps this distance from the walls (m)
const P: DiffParams = { r: 0.06, d: 0.18 };
const ELL = 0.45;                  // car wheelbase (m)
const KW = 0.2;                    // input plot: vertical coordinate = KW·ω
const H_STEP = 1 / 120;
const KEY_V = 0.4, KEY_W = 1.0;
const PARK_START: Pose = { phi: 0, x: -2.2, y: -0.7 };
const PARK_TARGET: Pose = { phi: 0, x: -2.2, y: 0.3 };
const PARK_TOL = 0.12, PARK_TOL_ANG = (8 * Math.PI) / 180;
const NAMES: Record<ModelKey, string> = { unicycle: '單輪車', diff: '差速驅動', car: '車型' };
// step 3 zooms onto the four-move manoeuvre (its net displacement is only a few decimetres); the
// zoom follows the step length a, so the small-step preset is magnified too
const BR_START: Pose = { phi: 0, x: 0.8, y: 0.45 };
const BR_START_CAR: Pose = { phi: 0, x: 0.55, y: -0.45 };
const ZOOM = (c: Pt, hw = 1.35) => ({ xmin: c[0] - hw, xmax: c[0] + hw, ymin: c[1] - hw / 2, ymax: c[1] + hw / 2 });
/** Below this step length the four move labels would pile up on each other, so only the path is drawn. */
const BR_LABEL_MIN_A = 0.35;
const BR_LABEL_OFF: [number, number][] = [[0, -16], [36, 0], [0, 18], [46, 20]];
const R2D = 180 / Math.PI;

interface PlanRun { segs: Segment[]; i: number; tSeg: number; loop: boolean; kind: 'bracket' | 'park' | 'auto' }
interface BracketShow { start: Pose; path: Pt[]; labels: { p: Pt; text: string }[]; pred: Pt; final: Pose; model: ModelKey; a: number }

/**
 * Figure-eight autopilot: a left circle then a right circle of radius R through the start pose.
 * Each loop takes exactly `loop` seconds (8 s = 960 physics steps), so encoder samples at any whole
 * number of Hz never straddle the switch between the two circles: the twist is constant within
 * every sample and the exponential update is exact.
 */
function figure8(R: number, loop = 8): Segment[] {
  const w = (2 * Math.PI) / loop, v = w * R;
  return [{ v, w, T: loop, label: '左圈' }, { v, w: -w, T: loop, label: '右圈' }];
}

const transformer = (p: Pose) => {
  const c = Math.cos(p.phi), s = Math.sin(p.phi);
  return (b: Pt): Pt => [p.x + c * b[0] - s * b[1], p.y + s * b[0] + c * b[1]];
};
const rect = (x0: number, y0: number, x1: number, y1: number): Pt[] => [[x0, y0], [x1, y0], [x1, y1], [x0, y1]];

function mount(host: HTMLElement, ctx: LessonContext): LessonHandle {
  // ------------------------------------------------------------------ state
  let model: ModelKey = 'diff';
  const lim: Limits = { vmax: 0.6, wmax: 1.5, umax: 10, rmin: 1.0 };
  let q: Pose = { phi: 0, x: 0, y: 0 };
  const sp = { v: 0, w: 0 };                    // joystick set-point
  let plan: PlanRun | null = null;
  let cmdV = 0, cmdW = 0;                       // applied inputs (after the input-set limits)
  let reqV = 0, reqW = 0;                       // requested inputs
  let reqClamped = false;
  let trueTrail: Pt[] = [];
  let odoTrail: Pt[] = [];
  let dist = 0, sdist = 0, lastPlotS = -1;
  // the estimate only changes at encoder samples: compare it with the truth AT those instants,
  // otherwise the plots would show the sampling lag instead of the odometry error
  let truthAtUpd: Pose = { ...q };
  let wallUntil = 0;
  let bracket: BracketShow | null = null;
  let brA = 0.6, brB = 0.7;
  const park = { active: false, reversals: 0, lastSign: 0, doneAt: null as number | null, t0: 0 };
  const odoCfg = { rate: 20, rule: 'exp' as OdoRule, slip: 0, delta: 0, seed: 7 };
  const odo = new WheelOdometry(q, { p: P, ...odoCfg });
  let simT = 0;
  let curStep = -1;
  let destroyed = false;

  // ------------------------------------------------------------------ layout
  simTitle(host, '非完整底盤：G(q)、Lie bracket 與里程計', 'q̇ = G(q) u');
  const cv = new Canvas2D(host, { aspect: 2, world: MAP, overlay: true, margin: 0.012 });
  Object.assign(cv.overlay.style, {
    whiteSpace: 'pre', background: 'color-mix(in srgb, var(--sim-bg) 82%, transparent)', padding: '0.05rem 0.3rem', borderRadius: '4px',
  });
  const kb = keyboardDrive(cv.el, {
    hint: '點一下地圖啟用鍵盤：W/S 前後、A/D 轉向（車型＝打方向盤）、空白鍵停',
    onChange: (pressed) => {
      if (pressed.size === 0) return;
      plan = null;
      chips.clear();
      sp.v = 0; sp.w = 0;
      play();
    },
  });
  // keyboardDrive lets arrow keys bubble; the lesson view uses ← / → on window to change steps
  const stopArrowBubble = (ev: KeyboardEvent) => { if (ev.key.startsWith('Arrow')) ev.stopPropagation(); };
  cv.el.addEventListener('keydown', stopArrowBubble);

  const panel = controlPanel(host, { compact: true });
  const r1 = row(panel);
  const pb = playback(r1, {
    playing: true,
    showTime: false,
    onPlay: () => runner.start(),
    onPause: () => runner.stop(),
    onReset: () => { const i = chips.active; if (i >= 0) chips.activate(i); else placeRobot({ phi: 0, x: 0, y: 0 }); refresh(); },
    onSpeed: (s) => { runner.speed = s; },
  });
  const modelSel = select<ModelKey>(r1, {
    options: [{ value: 'diff', label: '差速驅動' }, { value: 'unicycle', label: '單輪車' }, { value: 'car', label: '車型（car-like）' }],
    value: model,
    onChange: (m) => { setModel(m); refresh(); },
  });
  button(r1, { label: '■ 停止', small: true, title: '輸入歸零（空白鍵）', onClick: () => brake() });
  const resetOdoBtn = button(r1, { label: '⟲ 重置里程計', small: true, title: '把估計拉回真實位姿、清除誤差圖（同一組亂數種子）', onClick: () => { resetOdometry(); refresh(); } });

  const chipsHost = el('div', { style: { gridColumn: '1 / -1' } });
  panel.appendChild(chipsHost);
  const chips = presetChips(chipsHost, [], { label: '預設：' });

  const sR = slider(panel, {
    label: '車型最小轉彎半徑 r_min', min: 0.5, max: 2, step: 0.05, value: lim.rmin, unit: 'm',
    onInput: (v) => { lim.rmin = v; refresh(); },
  });
  const sA = slider(panel, { label: '四段序列：步長 a', min: 0.1, max: 0.8, step: 0.05, value: brA, unit: 'm', onInput: (v) => { brA = v; chips.clear(); } });
  const sB = slider(panel, {
    label: '四段序列：轉角 b', min: 0.1, max: 1.0, step: 0.05, value: brB, unit: 'rad', onInput: (v) => { brB = v; chips.clear(); },
  });
  const brRow = el('div', { style: { gridColumn: '1 / -1', display: 'flex', flexWrap: 'wrap', gap: '0.5rem', alignItems: 'center' } });
  panel.appendChild(brRow);
  button(brRow, { label: '▶ 四段序列', small: true, variant: 'primary', title: '依目前的 a、b 播放四段運動', onClick: () => playBracket(false) });
  const brNote = el('span', { class: 'mr-note', style: { margin: '0' } });
  brRow.appendChild(brNote);

  const ruleSel = select<OdoRule>(panel, {
    label: '里程計更新',
    options: [{ value: 'exp', label: 'T ← T·exp([V_b]Δt)' }, { value: 'euler', label: '沿舊車頭直加（Euler）' }],
    value: 'exp',
    onChange: (v) => { odoCfg.rule = v; odo.cfg.rule = v; resetOdometry(); chips.clear(); refresh(); },
  });
  const sRate = slider(panel, {
    label: '編碼器頻率', min: 2, max: 50, step: 1, value: odoCfg.rate, unit: 'Hz',
    onInput: (v) => { odoCfg.rate = v; odo.cfg.rate = v; chips.clear(); },
  });
  const sSlip = slider(panel, {
    label: '打滑雜訊 σ', min: 0, max: 10, step: 0.5, value: 0, unit: '%',
    onInput: (v) => { odoCfg.slip = v / 100; odo.cfg.slip = v / 100; chips.clear(); },
  });
  const sDelta = slider(panel, {
    label: '輪徑差 δ（右 − 左）', min: -3, max: 3, step: 0.25, value: 0, unit: '%',
    onInput: (v) => { odoCfg.delta = v / 100; odo.cfg.delta = v / 100; chips.clear(); },
  });

  const [leftCol, rightCol] = split(host);
  // input set (v, ω), also the joystick
  const cvU = new Canvas2D(leftCol, { aspect: 1.1, world: { xmin: -0.75, xmax: 0.75, ymin: -0.75, ymax: 0.75 }, overlay: true, margin: 0.03 });
  Object.assign(cvU.overlay.style, { background: 'color-mix(in srgb, var(--sim-bg) 82%, transparent)', padding: '0.05rem 0.3rem', borderRadius: '4px' });
  // odometry error plots (steps 5–6) share the column
  const plotBox = el('div', { style: { display: 'none' } });
  leftCol.appendChild(plotBox);
  const plotPos = new Plot2D(plotBox, {
    title: '位置誤差 |p̂ − p|', xLabel: '行駛距離 s (m)', yLabel: 'm', height: 135, legend: false, yMinSpan: 0.02,
    series: [{ name: '位置誤差', color: '--danger' }],
  });
  const plotAng = new Plot2D(plotBox, {
    title: '車頭誤差 φ̂ − φ', xLabel: '行駛距離 s (m)', yLabel: '度', height: 135, yMinSpan: 0.5,
    series: [{ name: '量測', color: '--warn' }, { name: '預測 −δs/(2d)', color: '--fg-muted', dash: [5, 4] }],
  });

  const eqStyle = { display: 'flex', flexWrap: 'wrap', alignItems: 'center', gap: '0.1rem 0.15rem' };
  const gBox = el('div', { style: eqStyle });
  rightCol.appendChild(gBox);
  const qdView = matrixView(gBox, { label: 'q̇ =', digits: 2 });
  const GView = matrixView(gBox, { label: '= G(q)', digits: 2 });
  const uView = matrixView(gBox, { label: '', digits: 2 });
  for (const m of [qdView, GView, uView]) Object.assign(m.el.style, { fontSize: '0.74rem', marginRight: '0.2rem' });
  const info = kv(rightCol, ['輸入 u', '(v, ω)', '轉彎半徑 |v/ω|', '前輪轉角 ψ', '側向速度']);
  const brInfo = kv(rightCol, ['序列', '淨位移 (x_b, y_b)', '二階預測', '相對差']);
  const parkInfo = kv(rightCol, ['位置誤差', '車頭誤差', '換檔次數', '狀態']);
  const odoBox = el('div', { style: { display: 'none' } });
  rightCol.appendChild(odoBox);
  const formula = el('div', { style: { margin: '0.1rem 0 0.35rem', fontSize: '0.9rem' } });
  odoBox.appendChild(formula);
  const odoInfo = kv(odoBox, ['Δθ_L, Δθ_R', 'V_bΔt', '估計 q̂', '真實 q', '誤差 |Δp|／Δφ', '行駛距離 s', '更新次數']);
  note(odoBox, '每次編碼器更新時，拿估計和「同一時刻」的真實位姿比較；q 以 (φ, x, y) 表示。');
  let formulaRule: OdoRule | null = null;

  legend(host, [
    { color: 'var(--accent)', label: '真實軌跡' },
    { color: 'var(--warn)', label: '里程計估計' },
    { color: 'var(--danger)', label: '禁止的側向速度' },
    { color: 'var(--series-5)', label: '瞬時旋轉中心／淨位移' },
  ]);
  note(host, '位姿用 SE(2) 指數映射精確積分。打滑雜訊用固定種子的亂數產生，按「重置里程計」會重播同一組雜訊。地圖四周是牆。');
  const preview = mdElement('**下一課預告：軌跡追蹤**。右側的粉紅虛線是一條參考軌跡。只靠開迴路的輸入，誤差會像里程計一樣越積越多；下一課用回授（前饋＋誤差修正）讓非完整的車子收斂到參考軌跡上。', 'callout');
  preview.style.display = 'none';
  host.appendChild(preview);

  // ------------------------------------------------------------------ model helpers
  function setModel(m: ModelKey): void {
    model = m;
    modelSel.set(m, { silent: true });
    sR.el.style.display = m === 'car' ? '' : 'none';
    sB.el.style.display = m === 'car' || curStep !== 2 ? 'none' : '';
    info.show('(v, ω)', m === 'diff');
    info.show('前輪轉角 ψ', m === 'car');
  }

  const inArena = (p: Pose) => p.x > MAP.xmin + WALL && p.x < MAP.xmax - WALL && p.y > MAP.ymin + WALL && p.y < MAP.ymax - WALL;

  function inputsNow(): [number, number] {
    if (kb.pressed.size > 0) {
      if (kb.has('brake')) return [0, 0];
      const v = kb.axis('down', 'up') * KEY_V;
      const steer = kb.axis('right', 'left'); // +1 = left
      // car: A/D set full lock; the turn rate follows the speed (ω = v tan ψ / ℓ = ±v / r_min)
      return model === 'car' ? [v, (steer * v) / lim.rmin] : [v, steer * KEY_W];
    }
    return [sp.v, sp.w];
  }

  function steerDisplay(): number {
    const psiMax = Math.atan(ELL / lim.rmin);
    if (kb.pressed.size > 0) { const s = kb.axis('right', 'left'); if (s) return s * psiMax; }
    return Math.max(-psiMax, Math.min(psiMax, steeringAngle(cmdV, cmdW, ELL)));
  }

  // ------------------------------------------------------------------ simulation
  function advance(h: number): void {
    let rest = h;
    let guard = 0;
    while (rest > 1e-12 && guard++ < 16) {
      let v: number, w: number, dt = rest;
      if (plan) {
        const g = plan.segs[plan.i];
        const left = g.T - plan.tSeg;
        if (left <= 1e-12) {
          plan.i++;
          plan.tSeg = 0;
          if (plan.i >= plan.segs.length) { if (plan.loop) plan.i = 0; else plan = null; }
          continue;
        }
        dt = Math.min(rest, left);
        v = g.v; w = g.w;
        plan.tSeg += dt;
      } else {
        [v, w] = inputsNow();
      }
      reqV = v; reqW = w;
      const c = clampInput(model, v, w, lim, P);
      reqClamped = c.clamped;
      cmdV = c.v; cmdW = c.w;
      const nq = stepPose(q, [cmdW, cmdV, 0], dt);
      if (!inArena(nq) && inArena(q)) {
        wallUntil = performance.now() + 1500;
        plan = null;
        chips.clear();
        cmdV = 0; cmdW = 0;
        break;
      }
      q = nq;
      dist += Math.abs(cmdV) * dt;
      sdist += cmdV * dt;
      if (odo.step(cmdV, cmdW, dt)) onOdoUpdate();
      if (park.active) trackPark();
      rest -= dt;
    }
    simT += h;
    recordTrails();
  }

  function recordTrails(): void {
    const push = (arr: Pt[], p: Pose) => {
      const last = arr[arr.length - 1];
      if (!last || Math.hypot(p.x - last[0], p.y - last[1]) > 0.015) {
        arr.push([p.x, p.y]);
        if (arr.length > 5000) arr.splice(0, arr.length - 4500);
      }
    };
    push(trueTrail, q);
    push(odoTrail, odo.est);
  }

  function onOdoUpdate(): void {
    truthAtUpd = { ...q };
    if (dist - lastPlotS < 0.05) return;
    lastPlotS = dist;
    const e = poseError(odo.est, truthAtUpd);
    const pred = odoCfg.delta !== 0 ? headingDriftPrediction(odoCfg.delta, sdist, P.d) * R2D : undefined;
    plotPos.push(dist, [e.dist]);
    plotAng.push(dist, pred === undefined ? [e.dphi * R2D] : [e.dphi * R2D, pred]);
  }

  function trackPark(): void {
    const sg = Math.abs(cmdV) > 0.02 ? Math.sign(cmdV) : 0;
    if (sg && park.lastSign && sg !== park.lastSign) park.reversals++;
    if (sg) park.lastSign = sg;
    const e = poseError(q, PARK_TARGET);
    if (park.doneAt == null && e.dist < PARK_TOL && Math.abs(e.dphi) < PARK_TOL_ANG) park.doneAt = simT;
  }

  function prerun(seconds: number): void {
    const n = Math.round(seconds / H_STEP);
    for (let k = 0; k < n; k++) advance(H_STEP);
  }

  const runner = physicsRunner({
    h: H_STEP,
    step: (h) => advance(h),
    onFrame: () => refresh(),
    guard: () => Number.isFinite(q.x) && Number.isFinite(q.y) && Number.isFinite(q.phi),
    onBlowUp: () => placeRobot({ phi: 0, x: 0, y: 0 }),
  });

  function play(): void { if (!runner.running) { runner.start(); pb.setPlaying(true, { silent: true }); } }

  function brake(): void {
    plan = null;
    chips.clear();
    sp.v = 0; sp.w = 0;
    refresh();
  }

  function resetOdometry(): void {
    odo.reset(q, odoCfg.seed);
    truthAtUpd = { ...q };
    odoTrail = [[q.x, q.y]];
    dist = 0; sdist = 0; lastPlotS = -1;
    plotPos.clear();
    plotAng.clear();
  }

  function placeRobot(p: Pose): void {
    q = { ...p };
    plan = null;
    trueTrail = [[q.x, q.y]];
    park.active = false;
    resetOdometry();
  }

  function cruise(v: number, w: number, pose?: Pose, pre = 0): void {
    if (pose) placeRobot(pose);
    plan = null;
    sp.v = v; sp.w = w;
    if (pre > 0) prerun(pre);
    play();
    refresh();
  }

  function startPlan(kind: PlanRun['kind'], segs: Segment[], loop: boolean): void {
    sp.v = 0; sp.w = 0;
    plan = { segs, i: 0, tSeg: 0, loop, kind };
    play();
  }

  function playBracket(instant: boolean): void {
    const start: Pose = model === 'car' ? BR_START_CAR : BR_START;
    applyView();
    placeRobot(start);
    const segs = bracketSequence(model, brA, brB, lim.rmin);
    const s = samplePlan(start, segs);
    // label each move at the pose halfway through it
    let t = 0;
    const labels = segs.map((g) => {
      const mid = poseAlong(start, segs, t + g.T / 2).pose;
      t += g.T;
      return { p: [mid.x, mid.y] as Pt, text: g.label };
    });
    bracket = { start, path: s.path, labels, pred: bracketPrediction(model, brA, brB, lim.rmin, start), final: s.final, model, a: brA };
    startPlan('bracket', segs, false);
    if (instant) prerun(planDuration(segs) + 0.05);
    refresh();
  }

  function startPark(m: ModelKey = 'car'): void {
    setModel(m);
    placeRobot(PARK_START);
    park.active = true;
    park.reversals = 0;
    park.lastSign = 0;
    park.doneAt = null;
    park.t0 = simT;
    sp.v = 0; sp.w = 0;
    play();
    refresh();
  }

  function setOdo(o: Partial<typeof odoCfg>): void {
    Object.assign(odoCfg, o);
    Object.assign(odo.cfg, odoCfg);
    ruleSel.set(odoCfg.rule, { silent: true });
    sRate.set(odoCfg.rate, { silent: true });
    sSlip.set(odoCfg.slip * 100, { silent: true });
    sDelta.set(odoCfg.delta * 100, { silent: true });
  }

  function autopilot8(pre: number): void {
    setModel('diff');
    placeRobot({ phi: 0, x: 0, y: 0 });
    startPlan('auto', figure8(0.55), true);
    if (pre > 0) prerun(pre);
    refresh();
  }

  // ------------------------------------------------------------------ presets per step
  const Pz = (label: string, apply: () => void, title?: string): Preset => ({ label, apply, title });
  function presetsFor(i: number): Preset[] {
    switch (i) {
      case 0: return [
        Pz('車型', () => { setModel('car'); cruise(0.3, 0.3, { phi: 0, x: -0.6, y: -0.95 }, 3.5); }, '沿半徑 1 m 的圓弧行駛'),
        Pz('差速驅動', () => { setModel('diff'); cruise(0.3, 0.3, { phi: 0, x: -0.6, y: -0.95 }, 3.5); }),
        Pz('單輪車', () => { setModel('unicycle'); cruise(0.3, 0.3, { phi: 0, x: -0.6, y: -0.95 }, 3.5); }),
      ];
      case 1: return [
        Pz('轉圈', () => cruise(0.3, 0.5, { phi: 0, x: 1.0, y: -0.9 }, 3), 'v = 0.3 m/s、ω = 0.5 rad/s'),
        Pz('原地轉', () => cruise(0, 1.0, { phi: 0, x: 1.0, y: -0.3 }, 1), 'v = 0：車型做不到'),
        Pz('直走', () => cruise(0.35, 0, { phi: 0.3, x: -2.4, y: -0.9 }, 2)),
      ];
      case 2: return [
        Pz('大步：a = 0.6 m、b = 0.7 rad', () => { if (model === 'car') setModel('diff'); setBracket(0.6, 0.7); playBracket(true); }),
        Pz('小步：a = 0.25 m、b = 0.15 rad', () => { if (model === 'car') setModel('diff'); setBracket(0.25, 0.15); playBracket(true); }),
        Pz('車型：四段全打弧線', () => { setModel('car'); setBracket(0.5, brB); playBracket(true); }),
      ];
      case 3: return [
        Pz('開始挑戰', () => startPark('car'), '車回到起點，自己開進車位'),
        Pz('示範解法', () => { startPark('car'); startPlan('park', parallelParkPlan(PARK_TARGET.y - PARK_START.y, lim.rmin), false); }, '左弧＋右弧＋直線倒車'),
        Pz('差速對照', () => {
          startPark('diff');
          const w = 0.9;
          startPlan('park', [
            { v: 0, w, T: Math.PI / 2 / w, label: '原地左轉 90°' },
            { v: 0.35, w: 0, T: 1 / 0.35, label: '前進 1 m' },
            { v: 0, w: -w, T: Math.PI / 2 / w, label: '原地右轉 90°' },
          ], false);
        }, '差速車：轉、走、轉回來'),
      ];
      case 4: return [
        Pz('完美輪子＋exp', () => { setOdo({ rule: 'exp', rate: 20, slip: 0, delta: 0 }); autopilot8(14); }),
        Pz('Euler、2 Hz', () => { setOdo({ rule: 'euler', rate: 2, slip: 0, delta: 0 }); autopilot8(14); }),
        Pz('exp、2 Hz', () => { setOdo({ rule: 'exp', rate: 2, slip: 0, delta: 0 }); autopilot8(14); }, '取樣再慢，exp 也不會偏'),
        Pz('自己開', () => { setOdo({ rule: 'exp', rate: 20, slip: 0, delta: 0 }); setModel('diff'); cruise(0, 0, { phi: 0, x: -2.6, y: -1.0 }); }),
      ];
      default: return [
        Pz('輪徑差 1%', () => { setOdo({ rule: 'exp', rate: 20, slip: 0, delta: 0.01 }); autopilot8(45); }),
        Pz('打滑雜訊 3%', () => { setOdo({ rule: 'exp', rate: 20, slip: 0.03, delta: 0 }); autopilot8(45); }),
        Pz('兩者', () => { setOdo({ rule: 'exp', rate: 20, slip: 0.03, delta: 0.01 }); autopilot8(45); }),
      ];
    }
  }

  function setBracket(a: number, b: number): void {
    brA = a; brB = b;
    sA.set(a, { silent: true });
    sB.set(b, { silent: true });
  }

  // ------------------------------------------------------------------ input-set plot (joystick)
  const stick: Handle = {
    get: () => [sp.v, KW * sp.w],
    set: (p) => {
      plan = null;
      chips.clear();
      sp.v = Math.max(-0.72, Math.min(0.72, p[0]));
      sp.w = Math.max(-0.72, Math.min(0.72, p[1])) / KW;
      play();
      refresh();
    },
    color: '--warn',
    label: '搖桿',
    radius: 9,
  };
  cvU.addHandle(stick);

  cvU.draw = (c) => {
    for (const v of [-0.6, -0.3, 0, 0.3, 0.6]) {
      c.line([v, -0.75], [v, 0.75], { color: v === 0 ? '--grid-strong' : '--grid', width: 1 });
      if (v !== 0) c.text([v, -0.7], fmt(v, 1), { color: '--fg-muted', font: '10px monospace' });
    }
    for (let w = -3; w <= 3; w++) {
      c.line([-0.75, KW * w], [0.75, KW * w], { color: w === 0 ? '--grid-strong' : '--grid', width: 1 });
      if (w !== 0 && Math.abs(w) <= 2) c.text([-0.7, KW * w], String(w), { color: '--fg-muted', font: '10px monospace' });
    }
    c.text([0.66, 0.05], 'v', { color: '--fg-muted' });
    c.text([0.06, 0.66], 'ω', { color: '--fg-muted' });
    for (const poly of inputSetPolys(model, lim, P)) {
      const pts = poly.map((p) => [p[0], KW * p[1]] as Pt);
      c.polygon(pts, { color: '--accent', fill: '--accent', alpha: 0.13, width: 0 });
      c.polygon(pts, { color: '--accent', width: 2 });
    }
    if (reqClamped && !plan) {
      c.line([reqV, KW * reqW], [cmdV, KW * cmdW], { color: '--danger', width: 1.5, dash: [3, 3] });
      c.circle([reqV, KW * reqW], 0.03, { color: '--danger', width: 1.5 });
    }
    c.point([cmdV, KW * cmdW], 5, { color: '--accent' });
    const shape = model === 'diff' ? '菱形 |u_L|,|u_R| ≤ u_max' : model === 'car' ? '領結 |ω| ≤ |v|/r_min' : '長方形';
    cvU.setOverlayText(`輸入集合（${shape}）`);
  };

  // ------------------------------------------------------------------ map drawing
  function drawRobot(c: Canvas2D, pose: Pose, ghost: boolean, alpha = 1): void {
    const T = transformer(pose);
    const line = ghost ? '--warn' : '--fg';
    const fill = ghost ? undefined : '--bg-sunken';
    const a = (ghost ? 0.9 : 1) * alpha;
    const wheel = (cx: number, cy: number, hl: number, hw: number, ang = 0) => {
      const ca = Math.cos(ang), sa = Math.sin(ang);
      const pts: Pt[] = rect(-hl, -hw, hl, hw).map(([x, y]) => [cx + ca * x - sa * y, cy + sa * x + ca * y] as Pt);
      c.polygon(pts.map(T), { color: line, fill: ghost ? undefined : '--fg-muted', width: 1.2, alpha: a });
    };
    if (model === 'car') {
      c.polygon(rect(-0.13, -0.19, 0.6, 0.19).map(T), { color: line, fill, width: 1.5, alpha: a, dash: ghost ? [4, 3] : undefined });
      if (!ghost) c.line(T([0.4, -0.16]), T([0.4, 0.16]), { color: '--fg-muted', width: 1.2 });
      const psi = ghost ? 0 : steerDisplay();
      wheel(0, 0.15, 0.07, 0.028); wheel(0, -0.15, 0.07, 0.028);
      wheel(ELL, 0.15, 0.07, 0.028, psi); wheel(ELL, -0.15, 0.07, 0.028, psi);
    } else if (model === 'diff') {
      const ring: Pt[] = Array.from({ length: 32 }, (_, k) => [0.22 * Math.cos((k * Math.PI) / 16), 0.22 * Math.sin((k * Math.PI) / 16)] as Pt);
      c.polygon(ring.map(T), { color: line, fill, width: 1.5, alpha: a, dash: ghost ? [4, 3] : undefined });
      wheel(0, P.d, 0.075, 0.025); wheel(0, -P.d, 0.075, 0.025);
      if (!ghost) c.circle(T([-0.15, 0]), 0.025, { color: '--fg-muted', width: 1.2 });
    } else {
      const ring: Pt[] = Array.from({ length: 24 }, (_, k) => [0.1 * Math.cos((k * Math.PI) / 12), 0.1 * Math.sin((k * Math.PI) / 12)] as Pt);
      c.polygon(ring.map(T), { color: line, width: 1.2, alpha: a, dash: [3, 3] });
      wheel(0, 0, 0.13, 0.035);
    }
    c.arrow(T([0, 0]), T([model === 'car' ? 0.78 : 0.36, 0]), { color: ghost ? '--warn' : '--accent', width: 2, head: 8, alpha: a });
  }

  function drawConstraintCues(c: Canvas2D): void {
    const T = transformer(q);
    // the forbidden sideways direction at the reference point (rear axle centre); both ±y_b are
    // forbidden, so draw it on the side away from the ICR, which also lies on the axle line
    const sd = cmdW > 0.02 ? -1 : 1;
    const tip = T([0, sd * 0.42]);
    c.arrow(T([0, sd * 0.05]), tip, { color: '--danger', width: 2, head: 8, alpha: 0.85 });
    const mid = T([0, sd * 0.25]);
    const s = 0.06;
    const ex = transformer({ ...q, x: mid[0], y: mid[1] });
    c.line(ex([-s, -s]), ex([s, s]), { color: '--danger', width: 2.5 });
    c.line(ex([-s, s]), ex([s, -s]), { color: '--danger', width: 2.5 });
    c.text(T([0, sd * 0.58]), '不能側滑', { color: '--danger', bg: '--sim-bg' });
    // instantaneous centre of rotation: always on the wheel-axle line
    if (Math.abs(cmdW) > 0.02) {
      const yc = cmdV / cmdW;
      if (Math.abs(yc) < 3) {
        const icr = T([0, yc]);
        c.line(T([0, 0]), icr, { color: '--series-5', width: 1, dash: [3, 4] });
        c.point(icr, 4.5, { color: '--series-5' });
        c.text(icr, 'ICR', { color: '--series-5', dx: 14, dy: -8 });
      }
    }
  }

  function drawBracket(c: Canvas2D, b: BracketShow): void {
    c.polyline(b.path, { color: '--fg-muted', width: 2, dash: [6, 4] });
    if (b.a >= BR_LABEL_MIN_A) {
      b.labels.forEach((l, k) => {
        const [dx, dy] = b.model === 'car' ? [0, k % 2 ? 16 : -16] : BR_LABEL_OFF[k];
        c.text(l.p, l.text, { color: '--fg-muted', bg: '--sim-bg', dx, dy });
      });
    }
    const s: Pt = [b.start.x, b.start.y];
    const pe: Pt = [s[0] + b.pred[0], s[1] + b.pred[1]];
    c.arrow(s, pe, { color: '--fg-muted', width: 1.6, dash: [4, 3], head: 8 });
    if (!plan || plan.kind !== 'bracket') {
      const fe: Pt = [q.x, q.y];
      if (Math.hypot(fe[0] - s[0], fe[1] - s[1]) > 0.01) {
        c.arrow(s, fe, { color: '--series-5', width: 3, head: 10 });
        c.text([(s[0] + fe[0]) / 2, (s[1] + fe[1]) / 2], '實際淨位移', { color: '--series-5', align: 'left', dx: 14, dy: -8, bg: '--sim-bg' });
      }
    }
    c.text(pe, b.model === 'car' ? '預測 a²·[g_L, g_R]' : '預測 ab·[g₁, g₂]', { color: '--fg-muted', align: 'right', dx: -12, bg: '--sim-bg' });
    c.frame({ x: s[0], y: s[1], theta: b.start.phi }, 0.25, { width: 1.5, alpha: 0.7 });
  }

  function drawPark(c: Canvas2D): void {
    const T = transformer(PARK_TARGET);
    c.polygon(rect(-0.22, -0.28, 0.69, 0.28).map(T), { color: '--ok', fill: '--ok', alpha: 0.12, width: 0 });
    c.polygon(rect(-0.22, -0.28, 0.69, 0.28).map(T), { color: '--ok', width: 2, dash: [6, 4] });
    c.frame({ x: PARK_TARGET.x, y: PARK_TARGET.y, theta: PARK_TARGET.phi }, 0.25, { width: 1.5 });
    c.text(T([0.24, 0.42]), '目標車位（往左 1 m）', { color: '--ok', bg: '--sim-bg' });
    const S = transformer(PARK_START);
    c.polygon(rect(-0.2, -0.26, 0.67, 0.26).map(S), { color: '--fg-muted', width: 1, dash: [2, 4] });
  }

  function drawReference(c: Canvas2D): void {
    const pts: Pt[] = [];
    for (let k = 0; k <= 60; k++) { const x = 1.0 + (1.75 * k) / 60; pts.push([x, 0.55 + 0.3 * Math.sin(3.2 * (x - 1.0))]); }
    c.polyline(pts, { color: '--series-7', width: 2.5, dash: [8, 5] });
    c.text(pts[30], '下一課：參考軌跡', { color: '--series-7', dy: -18, bg: '--sim-bg' });
  }

  cv.draw = (c) => {
    c.grid(0.5);
    c.polygon(rect(MAP.xmin + 0.02, MAP.ymin + 0.02, MAP.xmax - 0.02, MAP.ymax - 0.02), { color: '--fg-muted', width: 3 });
    if (curStep !== 2) c.frame({ x: 0, y: 0, theta: 0 }, 0.3, { labels: 's', width: 1.5, alpha: 0.6 });
    const odoStep = curStep >= 4;
    if (curStep === 5) drawReference(c);
    if (curStep === 3) drawPark(c);
    if (model === 'car' && curStep <= 3) {
      const T = transformer(q);
      c.circle(T([0, lim.rmin]), lim.rmin, { color: '--fg-muted', width: 1.2, dash: [5, 5] });
      c.circle(T([0, -lim.rmin]), lim.rmin, { color: '--fg-muted', width: 1.2, dash: [5, 5] });
      const r = lim.rmin;
      const W = cv.world;
      const free = (p: Pt) => p[0] > W.xmin + 0.5 && p[0] < W.xmax - 0.5 && p[1] > W.ymin + 0.2 && p[1] < W.ymax - 0.15
        && !(p[1] > W.ymax - 0.55 && p[0] < W.xmin + 0.55 * (W.xmax - W.xmin))
        && !(p[1] < W.ymin + 0.45 && p[0] > W.xmin + 0.3 * (W.xmax - W.xmin)); // keyboard hint
      const at = [T([0, 2 * r]), T([0, -2 * r]), T([r, r]), T([r, -r]), T([-r, r]), T([-r, -r])].find(free);
      if (at) c.text(at, `r_min = ${fmt(r, 2)} m`, { color: '--fg-muted', bg: '--sim-bg' });
    }
    if (trueTrail.length > 1) c.polyline(trueTrail, { color: '--accent', width: 2, alpha: 0.85 });
    if (odoStep && odoTrail.length > 1) c.polyline(odoTrail, { color: '--warn', width: 2, dash: [6, 4] });
    if (odoStep) drawRobot(c, odo.est, true);
    drawRobot(c, q, false, curStep === 2 ? 0.35 : 1);
    // step 3: the manoeuvre annotations go on top of the faded robot so its body never hides them
    if (curStep === 2 && bracket) drawBracket(c, bracket);
    if (curStep <= 1) drawConstraintCues(c);
    const wall = performance.now() < wallUntil ? '   ⚠ 撞到牆了：倒車或轉向' : '';
    const clampMsg = reqClamped && !plan ? '   ⚠ 輸入超出可行集合，已夾制' : '';
    c.setOverlayText(`${NAMES[model]}   q = (φ, x, y) = (${fmtDeg(q.phi, 0)}, ${fmt(q.x, 2)}, ${fmt(q.y, 2)})\nv = ${fmt(cmdV, 2)} m/s   ω = ${fmt(cmdW, 2)} rad/s${wall}${clampMsg}`);
  };

  // ------------------------------------------------------------------ readouts
  function updatePanel(): void {
    // q̇ = G(q) u
    const G = model === 'diff' ? Gdiff(q.phi, P) : Gunicycle(q.phi);
    const u = model === 'diff' ? vwToWheels(cmdV, cmdW, P) : [cmdV, cmdW];
    qdView.set(matVec(G, u));
    GView.set(G, { digits: model === 'diff' ? 3 : 2 });
    uView.set(u);
    info.set('輸入 u', model === 'diff' ? `(u_L, u_R) = (${fmt(u[0], 2)}, ${fmt(u[1], 2)}) rad/s` : `(v, ω) = (${fmt(cmdV, 2)}, ${fmt(cmdW, 2)})`);
    info.set('(v, ω)', `(${fmt(cmdV, 2)} m/s, ${fmt(cmdW, 2)} rad/s)`);
    info.set('轉彎半徑 |v/ω|', Math.abs(cmdW) < 1e-3 ? (Math.abs(cmdV) < 1e-3 ? '—（靜止）' : '∞（直走）') : `${fmt(Math.abs(cmdV / cmdW), 2)} m`);
    info.set('前輪轉角 ψ', fmtDeg(steerDisplay(), 1));
    const qd = matVec(G, u);
    info.set('側向速度', `${fmt(-Math.sin(q.phi) * qd[1] + Math.cos(q.phi) * qd[2], 3)} m/s（恆為 0）`);
    if (bracket) {
      const c = Math.cos(bracket.start.phi), s = Math.sin(bracket.start.phi);
      const dx = q.x - bracket.start.x, dy = q.y - bracket.start.y;
      const nb: Pt = [c * dx + s * dy, -s * dx + c * dy];
      const pb2: Pt = [c * bracket.pred[0] + s * bracket.pred[1], -s * bracket.pred[0] + c * bracket.pred[1]];
      const running = !!plan && plan.kind === 'bracket';
      brInfo.set('序列', bracket.model === 'car' ? `四段弧，每段 ${fmt(brA, 2)} m` : `a = ${fmt(brA, 2)} m、b = ${fmt(brB, 2)}`);
      brInfo.set('淨位移 (x_b, y_b)', running ? '（執行中…）' : `(${fmt(nb[0], 3)}, ${fmt(nb[1], 3)})`);
      brInfo.set('二階預測', `(${fmt(pb2[0], 3)}, ${fmt(pb2[1], 3)})`);
      const np = Math.hypot(pb2[0], pb2[1]);
      brInfo.set('相對差', running || np < 1e-9 ? '—' : `${fmt((100 * Math.hypot(nb[0] - pb2[0], nb[1] - pb2[1])) / np, 1)} %`);
    }
    {
      const e = poseError(q, PARK_TARGET);
      parkInfo.set('位置誤差', `${fmt(e.dist, 3)} m（< ${PARK_TOL} m）`);
      parkInfo.set('車頭誤差', `${fmtDeg(e.dphi, 1)}（< 8°）`);
      parkInfo.set('換檔次數', String(park.reversals));
      parkInfo.set('狀態', park.doneAt != null ? `✓ 停好了！用時 ${fmt(park.doneAt - park.t0, 1)} s` : plan?.kind === 'park' ? '示範中…' : '開進綠色車位');
    }
    if (curStep >= 4) {
      if (formulaRule !== odoCfg.rule) {
        formulaRule = odoCfg.rule;
        formula.replaceChildren(mathEl(odoCfg.rule === 'exp'
          ? 'T_{k+1}=T_k\\exp([\\mathcal V_b]\\Delta t)'
          : 'x_{k+1}=x_k+\\Delta x_b\\cos\\phi_k,\\ \\ y_{k+1}=y_k+\\Delta x_b\\sin\\phi_k'));
      }
      const e = poseError(odo.est, truthAtUpd);
      const D = odo.lastDelta;
      odoInfo.set('Δθ_L, Δθ_R', `${fmt(odo.lastDL, 3)}, ${fmt(odo.lastDR, 3)} rad`);
      odoInfo.set('V_bΔt', `(${fmt(D[0], 3)}, ${fmt(D[1], 3)}, 0)`);
      odoInfo.set('估計 q̂', `(${fmtDeg(wrapToPi(odo.est.phi), 0)}, ${fmt(odo.est.x, 2)}, ${fmt(odo.est.y, 2)})`);
      odoInfo.set('真實 q', `(${fmtDeg(wrapToPi(truthAtUpd.phi), 0)}, ${fmt(truthAtUpd.x, 2)}, ${fmt(truthAtUpd.y, 2)})`);
      odoInfo.set('誤差 |Δp|／Δφ', `${fmt(e.dist, 3)} m／${fmtDeg(e.dphi, 1)}`);
      odoInfo.set('行駛距離 s', `${fmt(dist, 1)} m`);
      odoInfo.set('更新次數', `${odo.updates}（${fmt(odoCfg.rate, 0)} Hz）`);
    }
    const nm = model === 'car' ? `車型：淨位移朝左，約 a²·2/r_min = ${fmt((2 * brA * brA) / lim.rmin, 3)} m` : `預測側移 ≈ a·b = ${fmt(brA * brB, 3)} m（朝右）`;
    if (brNote.textContent !== nm) brNote.textContent = nm;
  }

  function refresh(): void {
    if (destroyed) return;
    cv.requestRender();
    cvU.requestRender();
    updatePanel();
  }

  // ------------------------------------------------------------------ phone layout: taller map
  let mapAspect = 2;
  let fitQueued = false;
  const fitMap = () => {
    fitQueued = false;
    if (destroyed) return;
    const w = cv.el.clientWidth;
    const a = w > 0 && w < 520 ? 1.55 : 2;
    if (a !== mapAspect) { mapAspect = a; cv.setSize({ aspect: a }); }
  };
  const ro = new ResizeObserver(() => { if (!fitQueued) { fitQueued = true; requestAnimationFrame(fitMap); } });
  ro.observe(host);

  // ------------------------------------------------------------------ steps
  let focused: HTMLElement[] = [];
  function setFocus(i: number): void {
    focused.forEach((f) => f.classList.remove('mr-focus'));
    const targets: HTMLElement[][] = [
      [cv.el],
      [gBox, cvU.el],
      [cv.el, brInfo.el],
      [cv.el, parkInfo.el],
      [odoBox],
      [plotBox],
    ];
    focused = targets[i] ?? [];
    focused.forEach((f) => f.classList.add('mr-focus'));
  }

  function applyView(): void {
    if (curStep !== 2) { cv.setWorld(MAP); return; }
    const a = brA;
    if (model === 'car') {
      const hw = Math.min(1.35, Math.max(0.8, 1.6 * a + 0.55));
      cv.setWorld(ZOOM([BR_START_CAR.x + a, BR_START_CAR.y + a / 2], hw));
    } else {
      // the start pose stays clear of the readout overlay (top left) at every zoom
      const hw = Math.min(1.35, Math.max(0.8, 1.6 * a + 0.4));
      cv.setWorld(ZOOM([BR_START.x + a / 2, BR_START.y - 0.02 - 0.236 * (hw - 0.8)], hw));
    }
  }

  function onStep(i: number): void {
    const prev = curStep;
    curStep = i;
    applyView();
    const odoStep = i >= 4;
    cvU.el.style.display = odoStep ? 'none' : '';
    plotBox.style.display = odoStep ? '' : 'none';
    gBox.style.display = odoStep ? 'none' : '';
    info.el.style.display = i === 0 || i === 1 || i === 3 ? '' : 'none';
    brInfo.el.style.display = i === 2 ? '' : 'none';
    parkInfo.el.style.display = i === 3 ? '' : 'none';
    odoBox.style.display = odoStep ? '' : 'none';
    preview.style.display = i === 5 ? '' : 'none';
    resetOdoBtn.style.display = odoStep ? '' : 'none';
    for (const s of [ruleSel.el, sRate.el, sSlip.el, sDelta.el]) s.style.display = odoStep ? '' : 'none';
    sA.el.style.display = i === 2 ? '' : 'none';
    brRow.style.display = i === 2 ? '' : 'none';
    if (i !== 3) park.active = false;
    if (i === 1 && prev !== 1) setModel('diff');
    setModel(model);
    chips.setPresets(presetsFor(i));
    if (prev !== i) chips.activate(0);
    setFocus(i);
    refresh();
  }

  // ------------------------------------------------------------------ boot
  setModel(model);
  onStep(ctx.currentStep);
  runner.start();

  return {
    onStep,
    destroy() {
      destroyed = true;
      runner.stop();
      kb.destroy();
      cv.el.removeEventListener('keydown', stopArrowBubble);
      ro.disconnect();
      cv.destroy();
      cvU.destroy();
      plotPos.destroy();
      plotAng.destroy();
    },
  };
}

const lesson: Lesson = { meta, steps, quiz, mount };
export default lesson;
