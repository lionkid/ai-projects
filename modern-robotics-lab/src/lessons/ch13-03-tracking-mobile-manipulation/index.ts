import type { Lesson, LessonContext, LessonHandle } from '@/types';
import {
  el, fmt, controlPanel, row, slider, toggle, select, playback, simTitle, split, kv, legend, matrixView, note,
  Canvas2D, Plot2D, plotGroup, presetChips, createLoop, poseHandle,
  type Handle, type Marker, type Preset, type Pt, type SeriesSpec, type World,
} from '@/ui';
import { meta } from './meta';
import { quiz, steps } from './content';
import { makeReference, REF_LABELS, type RefKind, type Reference } from './references';
import { indexAt, lateralMode, simulateTracking, type BaseKind, type ErrFrame, type TrackingConfig, type TrackingTrial } from './tracking';
import { minManipulability, simulateMobile, type MMConfig, type MMTrial, type Redundancy } from './mobileManip';
import { between, compose, pose, toWorld, type Pose } from './se2';
import { drawArm, drawBase, labelledArrow } from './draw';

// ---------------------------------------------------------------------------
// Scenario data
// ---------------------------------------------------------------------------

type Page = 'track' | 'mm' | 'sum';

interface TrackState {
  ref: RefKind;
  speed: number;
  base: BaseKind;
  ff: boolean;
  fb: boolean;
  errFrame: ErrFrame;
  k1: number; k2: number; k3: number;
  kp: number; ki: number;
  bias: number;
  push: boolean;
  /** Robot start pose relative to the reference start (in the reference's start frame). */
  offset: Pose;
  showErr: boolean;
  /** Camera follows the robot (close-up of the error). */
  follow: boolean;
}

interface MMState { base: BaseKind; mode: Redundancy; kn: number; Tf: number; Bx: number; ghost: boolean }

const TRACK_DEFAULT: TrackState = {
  ref: 'circle', speed: 1, base: 'diff', ff: true, fb: true, errFrame: 'body',
  k1: 1.5, k2: 4, k3: 4, kp: 1.5, ki: 0, bias: 0, push: false, offset: pose(0, 0, 0), showErr: false, follow: false,
};
const MM_DEFAULT: MMState = { base: 'diff', mode: 'minnorm', kn: 1.5, Tf: 16, Bx: 3, ghost: true };

/** 17° heading error and 0.3 m to the right of the reference start. */
const OFF_MAIN = pose(0.3, 0, -0.3);
/** Pure lateral offset of 0.3 m (reference on the robot's left). */
const OFF_LAT = pose(0, 0, -0.3);
const PUSH = { t: 6, d: pose(0.35, 0, -0.3) };
const WORLD_A: World = { xmin: -2.45, xmax: 2.45, ymin: -1.4, ymax: 1.4 };
/** Default view per reference (aspect 1.75): the short manoeuvres get a closer view. */
const REF_WORLD: Record<RefKind, World> = {
  circle: WORLD_A, scurve: WORLD_A,
  parking: { xmin: -1.75, xmax: 1.75, ymin: -1, ymax: 1 },
  sideways: { xmin: -1.75, xmax: 1.75, ymin: -1, ymax: 1 },
  still: { xmin: -1.4, xmax: 1.4, ymin: -0.8, ymax: 0.8 },
};
/** Width (m) of the close-up view when the camera follows the robot. */
const FOLLOW_W = 2.6;
const WORLD_B: World = { xmin: -0.75, xmax: 3.95, ymin: -0.42, ymax: 1.62 };
const ASPECT_A = 1.75, ASPECT_B = 2.3;
const LINE_Y = 1.05;
const WALL_Y = 1.17;
const MM_KP = 2;
const W_WARN = 0.15;
const ARROW_SCALE = 2.2; // m drawn per m/s

interface TrackPreset { label: string; title: string; s: Partial<TrackState> }
interface MMPreset { label: string; title: string; s: Partial<MMState> }

const STEP1: TrackPreset[] = [
  { label: '直線插值 ✗', title: '把起點與終點的 (φ, x, y) 直接線性插值：要求側移', s: { ref: 'sideways' } },
  { label: '停車 ✓', title: '同樣的起點與終點：前進 S 彎＋倒車', s: { ref: 'parking' } },
  { label: '圓 ✓', title: '固定 (v_d, ω_d) 積分出的圓', s: { ref: 'circle' } },
  { label: 'S 彎 ✓', title: '左右交替的圓弧', s: { ref: 'scurve' } },
];
const STEP2: TrackPreset[] = [
  { label: '前饋＋初始誤差', title: '只送 u_d(t)，起點偏 0.3 m、斜 17°', s: { ff: true, fb: false, offset: OFF_MAIN } },
  { label: '前饋＋輪速偏差', title: '起點正確，但車身多了 0.15 rad/s 的漂移', s: { ff: true, fb: false, bias: 0.15 } },
  { label: '加上回授（對照）', title: '同樣的初始誤差，前饋＋回授', s: { ff: true, fb: true, offset: OFF_MAIN } },
];
const STEP3: TrackPreset[] = [
  { label: '誤差在 {b}', title: '以機器人座標定義 (x_e, y_e)', s: { offset: OFF_MAIN, showErr: true, follow: true } },
  { label: '誤差在 {s}（錯誤示範）', title: '同一條控制律改讀世界座標的 x_d − x, y_d − y', s: { offset: OFF_MAIN, showErr: true, errFrame: 'world' } },
  { label: '側推一下', title: 't = 6 s 時把車往右推 0.3 m、轉 20°', s: { showErr: true, push: true, follow: true } },
];
const STEP4: TrackPreset[] = [
  { label: 'ζ = 1（k₂ = 4, k₃ = 4）', title: '臨界阻尼的側向誤差', s: { offset: OFF_LAT, showErr: true, follow: true } },
  { label: 'k₂ 過大（振盪）', title: 'k₂ = 50、k₃ = 1.5 → ζ ≈ 0.11', s: { offset: OFF_LAT, showErr: true, follow: true, k2: 50, k3: 1.5 } },
  { label: '純回授（無前饋）', title: '拿掉 v_d cos φ_e 與 ω_d，參考速度 ×1.5', s: { offset: OFF_LAT, showErr: true, follow: true, ff: false, speed: 1.5 } },
  { label: 'v_d = 0（停住）', title: '參考停在原地：只修得掉 x_e', s: { ref: 'still', offset: pose(0, -0.3, 0.25), showErr: true } },
  { label: '倒車停車', title: '停車參考（含倒車），起點有偏差', s: { ref: 'parking', offset: pose(0.2, -0.15, -0.15), showErr: true } },
];
const STEP5: TrackPreset[] = [
  { label: 'P＋漂移', title: '全向底盤，K_i = 0，車身漂移 0.3 rad/s', s: { base: 'omni', bias: 0.3, follow: true, showErr: true } },
  { label: 'PI＋漂移', title: '加上 K_i = 1', s: { base: 'omni', bias: 0.3, ki: 1, follow: true, showErr: true } },
  { label: '側移參考（全向可行）', title: '差速做不到的直線插值', s: { base: 'omni', ref: 'sideways' } },
  { label: '全向＋初始誤差', title: '三個方向的誤差同時修正', s: { base: 'omni', offset: pose(0.4, 0.2, -0.3), showErr: true, follow: true } },
];
const STEP6: MMPreset[] = [
  { label: '最小範數（差速）', title: 'J_e^† V：手臂分擔運動', s: { base: 'diff', mode: 'minnorm' } },
  { label: '手臂居中（差速）', title: '加上零空間項把手臂拉回舒適姿態', s: { base: 'diff', mode: 'center' } },
  { label: '最小範數（全向）', title: '全向底盤：J_e 是 3×5', s: { base: 'omni', mode: 'minnorm' } },
  { label: '手臂居中（全向）', title: '全向底盤＋零空間項', s: { base: 'omni', mode: 'center' } },
];

const LINKS: { ch: string; topic: string; use: string; links: [string, string][] }[] = [
  { ch: 'Ch 2', topic: '組態空間與約束', use: '非完整約束決定哪些參考可行', links: [['ch02-03-constraints-task-space', '2-3 約束與任務空間']] },
  { ch: 'Ch 3', topic: '剛體運動與 twist', use: 'X⁻¹X_d、log 與 [Ad] 前饋', links: [['ch03-02-exponential-coordinates', '3-2 指數座標'], ['ch03-03-transforms-twists', '3-3 齊次變換與 twist']] },
  { ch: 'Ch 5', topic: 'Jacobian 與奇異', use: 'J_arm、可操作度 w', links: [['ch05-01-jacobian-geometry', '5-1 Jacobian'], ['ch05-02-singularities-manipulability', '5-2 奇異與可操作度']] },
  { ch: 'Ch 6', topic: '偽逆與冗餘', use: 'J_e^†、零空間投影', links: [['ch06-02-numerical-ik', '6-2 數值逆運動學']] },
  { ch: 'Ch 9', topic: '時間縮放', use: '五次多項式的直線參考', links: [['ch09-01-time-scaling', '9-1 時間縮放']] },
  { ch: 'Ch 11', topic: '前饋＋回授', use: 'PI 消除固定偏差', links: [['ch11-01-error-dynamics-pid', '11-1 誤差動態與 PID']] },
  { ch: 'Ch 13', topic: '輪式底盤', use: 'u = H(0)V_b、q̇ = G(q)u、里程計', links: [['ch13-01-omnidirectional-kinematics', '13-1 全向底盤'], ['ch13-02-nonholonomic-odometry', '13-2 非完整與里程計']] },
];

const deg = (rad: number, d = 1) => `${fmt((rad * 180) / Math.PI, d)}°`;
/** Keep every n-th sample (and the last one). */
function decimate<T>(a: T[], n: number): T[] {
  const out: T[] = [];
  for (let i = 0; i < a.length; i += n) out.push(a[i]);
  if (a.length && (a.length - 1) % n !== 0) out.push(a[a.length - 1]);
  return out;
}
const sgn = (x: number, d = 3) => (x < 0 ? `− ${fmt(-x, d)}` : `+ ${fmt(x, d)}`);

// ---------------------------------------------------------------------------
// Simulation
// ---------------------------------------------------------------------------

function mount(host: HTMLElement, ctx: LessonContext): LessonHandle {
  let page: Page = 'track';
  let destroyed = false;
  const ts: TrackState = { ...TRACK_DEFAULT };
  const ms: MMState = { ...MM_DEFAULT };
  let refA: Reference = makeReference(ts.ref, ts.speed);
  let trialA: TrackingTrial | null = null;
  let pathA: Pt[] = [];
  /** Every few samples of the robot path: enough for drawing, cheaper to stroke. */
  let drawPathA: Pt[] = [];
  let worldA: World = WORLD_A;
  let trialB: MMTrial | null = null;
  let altB: MMTrial | null = null;
  let pathB: Pt[] = [], eePathB: Pt[] = [];
  let tA = 0, tB = 0;

  // ---- layout -------------------------------------------------------------
  const title = simTitle(host, '軌跡追蹤與移動操作');
  const tabs = el('span', { style: { marginLeft: 'auto', display: 'inline-flex', gap: '0.3rem', flexWrap: 'wrap' } });
  title.style.flexWrap = 'wrap';
  title.appendChild(tabs);
  const PAGES: [Page, string][] = [['track', '① 底盤追蹤'], ['mm', '② 移動操作'], ['sum', '③ 總結']];
  const tabBtns = PAGES.map(([p, label]) => el('button', { class: 'mr-chip', type: 'button', onClick: () => { if (p !== page) chips.clear(); setPage(p); } }, label));
  tabs.append(...tabBtns);
  const chips = presetChips(host, [], { label: '情境' });
  const card = summaryCard();
  host.appendChild(card);
  const cv = new Canvas2D(host, { aspect: ASPECT_A, world: WORLD_A, overlay: true });
  cv.overlay.style.whiteSpace = 'pre';
  const legA = legend(host, [
    { color: 'var(--fg-muted)', label: '參考 q_d(t)（虛線）' },
    { color: 'var(--series-1)', label: '機器人 q(t)' },
    { color: 'var(--axis-x)', label: 'x_e' },
    { color: 'var(--axis-y)', label: 'y_e' },
    { color: 'var(--series-5)', label: 'φ_e' },
  ]);
  const legB = legend(host, [
    { color: 'var(--accent)', label: '末端的期望直線' },
    { color: 'var(--series-4)', label: '底盤貢獻 J_base·u' },
    { color: 'var(--series-5)', label: '手臂貢獻 J_arm·dθ/dt' },
    { color: 'var(--danger)', label: '手臂接近奇異（w < 0.15）' },
  ]);
  const pbWrap = el('div');
  host.appendChild(pbWrap);
  const pb = playback(pbWrap, {
    duration: refA.T,
    playing: true,
    onPlay: () => { if (curT() >= curEnd() - 1e-9) setCurT(0); loop.start(); },
    onPause: () => loop.stop(),
    onReset: () => { setCurT(0); frame(true); },
    onSpeed: (s) => { loop.speed = s; },
    onScrub: (t) => { setCurT(t); frame(true); },
  });
  const noteEl = note(host, '');

  // ---- page A: tracking -----------------------------------------------------
  const pageA = el('div');
  host.appendChild(pageA);
  const [pa1, pa2] = split(pageA);
  const errPlot = new Plot2D(pa1, {
    title: '誤差（機器人座標 {b}）', xLabel: 't (s)', yLabel: 'm / rad', height: 132, yMinSpan: 0.2,
    series: [{ name: 'φ_e', color: '--series-5' }, { name: 'x_e', color: '--axis-x' }, { name: 'y_e', color: '--axis-y' }],
  });
  const uPlot = new Plot2D(pa2, { title: '輸入 u = (v, ω)', xLabel: 't (s)', height: 132, yMinSpan: 0.5, series: [{ name: 'v' }] });
  const groupA = plotGroup([errPlot, uPlot]);
  const kvA = kv(pageA, ['err', 'v', 'w', 'mode', 'feas']);
  const panelA = controlPanel(pageA, { compact: true });
  const rowA1 = row(panelA);
  const selRef = select<RefKind>(rowA1, {
    label: '參考軌跡', value: ts.ref,
    options: (['circle', 'scurve', 'parking', 'sideways', 'still'] as RefKind[]).map((k) => ({ value: k, label: REF_LABELS[k] })),
    onChange: (v) => { ts.ref = v; refChanged(); },
  });
  const selBase = select<BaseKind>(rowA1, {
    label: '底盤', value: ts.base,
    options: [{ value: 'diff', label: '差速（不能側移）' }, { value: 'omni', label: '全向（mecanum）' }],
    onChange: (v) => { ts.base = v; syncA(); touchedA(); },
  });
  const selFrame = select<ErrFrame>(rowA1, {
    label: '誤差座標', value: ts.errFrame,
    options: [{ value: 'body', label: '機器人 {b}' }, { value: 'world', label: '世界 {s}（錯誤示範）' }],
    onChange: (v) => { ts.errFrame = v; touchedA(); },
  });
  const rowA2 = row(panelA);
  const tFF = toggle(rowA2, { label: '前饋', value: ts.ff, onChange: (v) => { ts.ff = v; touchedA(); } });
  const tFB = toggle(rowA2, { label: '回授', value: ts.fb, onChange: (v) => { ts.fb = v; touchedA(); } });
  const tPush = toggle(rowA2, { label: '側推（t = 6 s）', value: ts.push, onChange: (v) => { ts.push = v; touchedA(); } });
  const tErr = toggle(rowA2, { label: '顯示誤差分解', value: ts.showErr, onChange: (v) => { ts.showErr = v; chips.clear(); cv.requestRender(); } });
  const tFollow = toggle(rowA2, { label: '鏡頭跟隨（放大）', value: ts.follow, onChange: (v) => { ts.follow = v; chips.clear(); if (!v && page === 'track') cv.setWorld(worldA); frame(true); } });
  const sK1 = slider(panelA, { label: 'k₁', min: 0, max: 5, step: 0.1, value: ts.k1, unit: '1/s', onInput: (v) => { ts.k1 = v; touchedA(); } });
  const sK2 = slider(panelA, { label: 'k₂', min: 0, max: 60, step: 0.5, value: ts.k2, unit: '1/m²', onInput: (v) => { ts.k2 = v; touchedA(); } });
  const sK3 = slider(panelA, { label: 'k₃', min: 0, max: 12, step: 0.1, value: ts.k3, unit: '1/m', onInput: (v) => { ts.k3 = v; touchedA(); } });
  const sKp = slider(panelA, { label: 'K_p', min: 0, max: 5, step: 0.1, value: ts.kp, unit: '1/s', onInput: (v) => { ts.kp = v; touchedA(); } });
  const sKi = slider(panelA, { label: 'K_i', min: 0, max: 4, step: 0.1, value: ts.ki, unit: '1/s²', onInput: (v) => { ts.ki = v; touchedA(); } });
  const sBias = slider(panelA, { label: '漂移 ω_bias', min: -0.5, max: 0.5, step: 0.01, value: ts.bias, unit: 'rad/s', onInput: (v) => { ts.bias = v; touchedA(); } });
  const sSpeed = slider(panelA, { label: '參考速度倍率', min: 0.5, max: 2, step: 0.05, value: ts.speed, unit: '×', onInput: (v) => { ts.speed = v; refChanged(); } });

  // ---- page B: mobile manipulation ------------------------------------------
  const pageB = el('div');
  host.appendChild(pageB);
  const [pb1, pb2] = split(pageB);
  const wPlot = new Plot2D(pb1, {
    title: '手臂可操作度 w = |sin θ₂|', xLabel: 't (s)', height: 132, yRange: [0, 1.05],
    series: [{ name: '目前解法', color: '--series-1', width: 2.2 }, { name: '另一解法', color: '--series-8', dash: [5, 4] }],
  });
  const sPlot = new Plot2D(pb2, {
    title: '沿直線方向的末端速度 (m/s)', xLabel: 't (s)', height: 132,
    series: [{ name: '底盤', color: '--series-4', width: 2 }, { name: '手臂', color: '--series-5', width: 2 }, { name: '需求', color: '--fg-muted', dash: [5, 4] }],
  });
  const groupB = plotGroup([wPlot, sPlot]);
  const jRow = el('div', { style: { display: 'flex', flexWrap: 'wrap', gap: '0.3rem 1.2rem', alignItems: 'center', margin: '0.3rem 0' } });
  pageB.appendChild(jRow);
  const Jview = matrixView(jRow, { label: 'J_e =', digits: 2, title: '橘色欄＝J_base，紫色欄＝J_arm（末端 body twist [ω, v_x, v_y]）' });
  const kvB = kv(jRow, ['w', 'theta', 'u', 'e']);
  kvB.setLabel('w', '可操作度 w');
  kvB.setLabel('theta', '手臂 (θ₁, θ₂)');
  kvB.setLabel('u', '底盤輸入 u');
  kvB.setLabel('e', '末端誤差');
  const panelB = controlPanel(pageB, { compact: true });
  const rowB1 = row(panelB);
  const selMode = select<Redundancy>(rowB1, {
    label: '冗餘解法', value: ms.mode,
    options: [{ value: 'minnorm', label: '最小範數 J_e^† V' }, { value: 'center', label: '手臂居中（零空間項）' }],
    onChange: (v) => { ms.mode = v; touchedB(); },
  });
  const selBaseB = select<BaseKind>(rowB1, {
    label: '底盤', value: ms.base,
    options: [{ value: 'diff', label: '差速（J_e 為 3×4）' }, { value: 'omni', label: '全向（J_e 為 3×5）' }],
    onChange: (v) => { ms.base = v; touchedB(); },
  });
  const tGhost = toggle(rowB1, { label: '顯示另一解法', value: ms.ghost, onChange: (v) => { ms.ghost = v; cv.requestRender(); } });
  const sKn = slider(panelB, { label: '零空間增益 k_n', min: 0, max: 4, step: 0.1, value: ms.kn, unit: '1/s', onInput: (v) => { ms.kn = v; touchedB(); } });
  const sTf = slider(panelB, { label: '畫線時間 T_f', min: 8, max: 30, step: 1, value: ms.Tf, unit: 's', onInput: (v) => { ms.Tf = v; touchedB(); } });

  // ---- handles ----------------------------------------------------------------
  const toPose2 = (q: Pose) => ({ x: q.x, y: q.y, theta: q.phi });
  const startHandle = poseHandle(cv, toPose2(startPose()), {
    color: '--series-1', knob: 0.45, frame: false,
    onChange: (p) => { ts.offset = between(refA.start, pose(p.theta, p.x, p.y)); setCurT(0); touchedA(); },
  });
  const bHandle: Handle = cv.addHandle({
    get: () => [ms.Bx, LINE_Y],
    set: (p) => { ms.Bx = Math.max(0.8, Math.min(3.3, Math.round(p[0] * 20) / 20)); touchedB(false); },
    onEnd: () => scheduleB(true),
    label: 'B', color: '--accent', radius: 9,
  });

  // ---- helpers ----------------------------------------------------------------
  function startPose(): Pose { return compose(refA.start, ts.offset); }
  const curT = () => (page === 'track' ? tA : tB);
  const curEnd = () => (page === 'track' ? refA.T : (trialB?.T ?? 1));
  function setCurT(t: number): void { if (page === 'track') tA = Math.max(0, Math.min(refA.T, t)); else tB = Math.max(0, Math.min(trialB?.T ?? 0, t)); }

  function trackConfig(): TrackingConfig {
    return {
      base: ts.base, ff: ts.ff, fb: ts.fb, errFrame: ts.errFrame,
      diff: { k1: ts.k1, k2: ts.k2, k3: ts.k3 }, omni: { kp: ts.kp, ki: ts.ki },
      bias: ts.bias, push: ts.push && refA.T > PUSH.t + 1 ? PUSH : null, start: startPose(),
    };
  }
  function mmConfig(mode: Redundancy = ms.mode): MMConfig {
    return { base: ms.base, mode, A: [0, LINE_Y], B: [ms.Bx, LINE_Y], phiE: Math.PI / 2, Tf: ms.Tf, kp: MM_KP, kn: ms.kn };
  }

  // ---- recomputation (rAF-debounced; a whole run takes a few tens of ms) ---------
  let pendA = false, pendB = false, pendAlt = false;
  function scheduleA(): void {
    if (pendA) return;
    pendA = true;
    requestAnimationFrame(() => { pendA = false; if (!destroyed) recomputeA(); });
  }
  function scheduleB(withAlt: boolean): void {
    pendAlt = pendAlt || withAlt;
    if (pendB) return;
    pendB = true;
    requestAnimationFrame(() => { pendB = false; const a = pendAlt; pendAlt = false; if (!destroyed) recomputeB(a); });
  }
  function touchedA(): void { chips.clear(); scheduleA(); }
  function touchedB(withAlt = true): void { chips.clear(); if (!withAlt) altB = null; scheduleB(withAlt); }
  function refChanged(): void {
    refA = makeReference(ts.ref, ts.speed);
    touchedA();
  }

  function recomputeA(): void {
    trialA = simulateTracking(refA, trackConfig());
    pathA = trialA.q.map((q) => [q.x, q.y] as Pt);
    drawPathA = decimate(pathA, 3);
    startHandle.set(toPose2(startPose()), { silent: true });
    fitWorldA();
    tA = Math.min(tA, refA.T);
    if (page === 'track') pb.setDuration(refA.T);
    updatePlotsA();
    frame(true);
  }

  function recomputeB(withAlt: boolean): void {
    trialB = simulateMobile(mmConfig());
    if (withAlt) altB = simulateMobile(mmConfig(ms.mode === 'minnorm' ? 'center' : 'minnorm'));
    pathB = decimate(trialB.q.map((q) => [q.x, q.y] as Pt), 2);
    eePathB = decimate(trialB.X.map((x) => [x.x, x.y] as Pt), 2);
    tB = Math.min(tB, trialB.T);
    if (page !== 'track') pb.setDuration(trialB.T);
    updatePlotsB();
    frame(true);
  }

  function fitWorldA(): void {
    // Keep the default view unless the robot leaves it (e.g. the world-frame counter-example).
    let { xmin, xmax, ymin, ymax } = REF_WORLD[ts.ref];
    for (const [x, y] of pathA) { xmin = Math.min(xmin, x - 0.4); xmax = Math.max(xmax, x + 0.4); ymin = Math.min(ymin, y - 0.4); ymax = Math.max(ymax, y + 0.4); }
    const w: World = { xmin, xmax, ymin, ymax };
    if (w.xmin !== worldA.xmin || w.xmax !== worldA.xmax || w.ymin !== worldA.ymin || w.ymax !== worldA.ymax) {
      worldA = w;
      if (page === 'track' && !ts.follow) cv.setWorld(worldA);
    }
  }

  // ---- plots ------------------------------------------------------------------
  function setSeries(plot: Plot2D, specs: SeriesSpec[], data: [number[], number[]][], maxSlots = 5): void {
    plot.setOptions({ series: specs });
    data.forEach(([xs, ys], i) => plot.setXY(i, xs, ys));
    for (let i = data.length; i < maxSlots; i++) plot.setXY(i, [], []);
  }

  function updatePlotsA(): void {
    const tr = trialA;
    if (!tr) return;
    const T = refA.T;
    // Every 2nd sample (20 ms) is plenty for ~280 px wide plots.
    const D = <X>(a: X[]): X[] => decimate(a, 2);
    const t = D(tr.t), err = D(tr.err), u = D(tr.u), ud = D(tr.ud), integ = D(tr.integ);
    errPlot.setRanges([0, T], undefined);
    errPlot.setXY(0, t, err.map((e) => e[0]));
    errPlot.setXY(1, t, err.map((e) => e[1]));
    errPlot.setXY(2, t, err.map((e) => e[2]));
    uPlot.setRanges([0, T], undefined);
    if (ts.base === 'diff') {
      uPlot.setOptions({ title: '輸入 u = (v, ω)', yLabel: 'm/s · rad/s' });
      setSeries(uPlot, [
        { name: 'v', color: '--series-1', width: 2 }, { name: 'ω', color: '--series-4', width: 2 },
        { name: 'v_d', color: '--series-1', dash: [5, 4], width: 1.4 }, { name: 'ω_d', color: '--series-4', dash: [5, 4], width: 1.4 },
      ], [[t, u.map((x) => x[1])], [t, u.map((x) => x[0])], [t, ud.map((x) => x[1])], [t, ud.map((x) => x[0])]]);
    } else {
      uPlot.setOptions({ title: '車身 twist V_b（指令）', yLabel: 'rad/s · m/s' });
      const specs: SeriesSpec[] = [{ name: 'ω', color: '--series-4', width: 2 }, { name: 'v_x', color: '--series-1', width: 2 }, { name: 'v_y', color: '--series-3', width: 2 }];
      const data: [number[], number[]][] = [[t, u.map((x) => x[0])], [t, u.map((x) => x[1])], [t, u.map((x) => x[2])]];
      if (ts.fb && ts.ki > 0) { specs.push({ name: 'K_i∫ω_err', color: '--series-4', dash: [5, 4], width: 1.6 }); data.push([t, integ.map((I) => ts.ki * I[0])]); }
      setSeries(uPlot, specs, data);
    }
    // Unlabelled: a label at the top of the plot would collide with the legend (the canvas names the push).
    const m: Marker[] = tr.pushIndex >= 0 ? [{ x: tr.t[tr.pushIndex], color: '--warn' }] : [];
    groupA.setMarkers(0, m);
    groupA.setMarkers(1, m);
  }

  function updatePlotsB(): void {
    const tr = trialB;
    if (!tr) return;
    const names: Record<Redundancy, string> = { minnorm: '最小範數', center: '手臂居中' };
    const other: Redundancy = ms.mode === 'minnorm' ? 'center' : 'minnorm';
    wPlot.setRanges([0, tr.T], [0, 1.05]);
    const specs: SeriesSpec[] = [{ name: names[ms.mode], color: '--series-1', width: 2.2 }];
    const data: [number[], number[]][] = [[tr.t, tr.w]];
    if (altB) { specs.push({ name: `${names[other]}（對照）`, color: '--series-8', dash: [5, 4], width: 1.6 }); data.push([altB.t, altB.w]); }
    setSeries(wPlot, specs, data, 3);
    const mw = minManipulability(tr);
    groupB.setMarkers(0, [
      { y: W_WARN, color: '--danger', label: '接近奇異' },
      { x: mw.t, y: mw.w, color: '--series-1' },
    ]);
    sPlot.setRanges([0, tr.T], undefined);
    sPlot.setXY(0, tr.t, tr.sBase);
    sPlot.setXY(1, tr.t, tr.sArm);
    sPlot.setXY(2, tr.t, tr.sDes);
  }

  // ---- readouts -----------------------------------------------------------------
  function updateReadoutsA(): void {
    const tr = trialA;
    if (!tr) return;
    const k = indexAt(tr, tA);
    const e = tr.err[k], u = tr.u[k], ff = tr.uff[k], fb = tr.ufb[k], ud = tr.ud[k];
    const sat = tr.sat[k] ? '（飽和 ⚠）' : '';
    if (ts.base === 'diff') {
      kvA.setLabel('err', '誤差 (φ_e, x_e, y_e)');
      kvA.set('err', `(${deg(e[0])}, ${fmt(e[1], 3)} m, ${fmt(e[2], 3)} m)`);
      kvA.setLabel('v', 'v = 前饋 + 回授');
      kvA.set('v', `${fmt(ff[1], 3)} ${sgn(fb[1])} → ${fmt(u[1], 3)} m/s${sat}`);
      kvA.setLabel('w', 'ω = 前饋 + 回授');
      kvA.set('w', `${fmt(ff[0], 3)} ${sgn(fb[0])} → ${fmt(u[0], 3)} rad/s`);
      const lm = lateralMode(ud[1], ts.k2, ts.k3);
      kvA.setLabel('mode', '側向模態（線性化）');
      kvA.set('mode', ts.fb ? `ζ = k₃/(2√k₂) = ${Number.isFinite(lm.zeta) ? fmt(lm.zeta, 2) : '∞'}，ωn = |v_d|√k₂ = ${fmt(lm.wn, 2)} rad/s` : '沒有回授：誤差不會被修正');
    } else {
      kvA.setLabel('err', '誤差 (φ_e, x_e, y_e)');
      kvA.set('err', `(${deg(e[0])}, ${fmt(e[1], 3)} m, ${fmt(e[2], 3)} m)`);
      kvA.setLabel('v', '前饋 [Ad]V_d');
      kvA.set('v', `(${fmt(ff[0], 2)}, ${fmt(ff[1], 2)}, ${fmt(ff[2], 2)})`);
      kvA.setLabel('w', '回授 K_p X_err + K_i∫X_err');
      kvA.set('w', `(${fmt(fb[0], 2)}, ${fmt(fb[1], 2)}, ${fmt(fb[2], 2)})${sat}`);
      kvA.setLabel('mode', '穩態預測');
      kvA.set('mode', !ts.fb ? '沒有回授：誤差不會被修正'
        : ts.ki > 0 ? '有積分項：固定偏差會被 ∫X_err 抵銷 → 誤差歸零'
          : ts.bias !== 0 ? `只有 K_p：φ_e ≈ −ω_bias/K_p = ${ts.kp > 0 ? fmt(-ts.bias / ts.kp, 3) : '∞'} rad` : '無偏差：K_p 足以讓誤差指數收斂');
    }
    kvA.setLabel('feas', '參考可行性');
    kvA.set('feas', ts.base === 'omni' ? '✓ 全向底盤可產生任何車身 twist'
      : refA.maxLateral > 1e-6 ? `✗ 需要側向速度（最大 ${fmt(refA.maxLateral, 2)} m/s），差速底盤做不到`
        : refA.kind === 'still' ? '⚠ v_d ≡ 0：可行，但側向誤差修不掉'
          : '✓ 由同一模型積分產生：側向速度恆為 0');
  }

  function colorJ(nb: number): void {
    const cells = Jview.el.querySelectorAll<HTMLElement>('.mr-cell');
    const n = nb + 2;
    cells.forEach((c, i) => {
      const col = `var(${i % n < nb ? '--series-4' : '--series-5'})`;
      if (c.style.color !== col) c.style.color = col;
    });
  }

  function updateReadoutsB(): void {
    const tr = trialB;
    if (!tr) return;
    const k = indexAt(tr, tB);
    const th = tr.theta[k], u = tr.u[k];
    const mw = minManipulability(tr);
    kvB.set('w', `${fmt(tr.w[k], 3)}（全程最低 ${fmt(mw.w, 2)}）${tr.w[k] < W_WARN ? ' ⚠' : ''}`);
    kvB.set('theta', `(${deg(th[0], 0)}, ${deg(th[1], 0)})`);
    kvB.set('u', ms.base === 'diff' ? `v = ${fmt(u[0], 2)} m/s，ω = ${fmt(u[1], 2)} rad/s` : `(ω, v_x, v_y) = (${fmt(u[0], 2)}, ${fmt(u[1], 2)}, ${fmt(u[2], 2)})`);
    kvB.set('e', `${fmt(tr.errNorm[k] * 1000, 1)} mm`);
    Jview.set(tr.J[k], { digits: 2 });
    colorJ(tr.J[k][0].length - 2);
  }

  // ---- drawing ------------------------------------------------------------------
  function drawErr(c: Canvas2D, q: Pose, qd: Pose): void {
    const E = between(q, qd);
    const lab = (len: number, s: string) => (Math.abs(len) > 0.08 ? s : '');
    if (ts.errFrame === 'body' || ts.base === 'omni') {
      const p1 = toWorld(q, [E.x, 0]);
      labelledArrow(c, [q.x, q.y], p1, '--axis-x', lab(E.x, 'x_e'), { dy: 12 });
      labelledArrow(c, p1, [qd.x, qd.y], '--axis-y', lab(E.y, 'y_e'));
    } else {
      const p1: Pt = [qd.x, q.y];
      labelledArrow(c, [q.x, q.y], p1, '--axis-x', lab(qd.x - q.x, 'x_d − x（{s}）'), { dy: 12 });
      labelledArrow(c, p1, [qd.x, qd.y], '--axis-y', lab(qd.y - q.y, 'y_d − y（{s}）'));
    }
    if (Math.abs(E.phi) > 0.03) {
      const r = 0.42, a0 = q.phi, a1 = q.phi + E.phi;
      c.arc([q.x, q.y], r, Math.min(a0, a1), Math.max(a0, a1), { color: '--series-5', width: 2.5 });
      c.line([q.x, q.y], [q.x + (r + 0.06) * Math.cos(a1), q.y + (r + 0.06) * Math.sin(a1)], { color: '--series-5', width: 1, dash: [3, 3] });
      const am = (a0 + a1) / 2;
      c.text([q.x + (r + 0.14) * Math.cos(am), q.y + (r + 0.14) * Math.sin(am)], 'φ_e', { color: '--series-5', bg: '--sim-bg' });
    }
  }

  function drawTrack(c: Canvas2D): void {
    const tr = trialA;
    if (!tr) return;
    const ref = refA;
    const k = indexAt(tr, tA);
    c.grid(0.5);
    c.frame({ x: 0, y: 0, theta: 0 }, 0.28, { labels: 's', width: 1.5, alpha: 0.5 });
    const infeasible = ts.base === 'diff' && ref.maxLateral > 1e-6;
    c.polyline(ref.path.map((p) => [p.x, p.y] as Pt), { color: infeasible ? '--danger' : '--fg-muted', width: 1.8, dash: [7, 5] });
    for (const cp of ref.cusps) c.circle([cp.x, cp.y], 0.07, { color: '--warn', width: 2 });
    if (ref.kind === 'sideways') {
      // Ghost poses along the interpolated reference, with the sideways velocity they demand.
      const move = ref.T - 2;
      const goal = ref.sample(ref.T).q;
      drawBase(c, goal, ts.base, { ghost: true, alpha: 0.55, color: infeasible ? '--danger' : '--fg-muted' });
      c.text(toWorld(goal, [0, 0.3]), '終點', { color: infeasible ? '--danger' : '--fg-muted' });
      const mid = ref.sample(move / 2);
      if (infeasible) labelledArrow(c, [mid.q.x + 0.42, mid.q.y - 0.25], [mid.q.x + 0.42, mid.q.y - 0.25 + mid.V[2] * 2], '--danger', '要求側移 ✗', { dx: 10, dy: 0 });
      else c.text([mid.q.x + 0.45, mid.q.y], '全向底盤可直接側移 ✓', { color: '--ok', align: 'left', bg: '--sim-bg' });
    }
    if (ref.kind === 'still') c.text([ref.start.x, ref.start.y], 'q_d 停住（v_d = 0）', { color: '--fg-muted', dy: 30, bg: '--sim-bg' });
    // robot: the whole simulated path (faint) and the part travelled so far
    c.polyline(drawPathA, { color: '--series-1', width: 1.4, alpha: 0.3 });
    const travelled = drawPathA.slice(0, Math.floor(k / 3) + 1);
    travelled.push(pathA[k]);
    c.polyline(travelled, { color: '--series-1', width: 2.6 });
    if (tr.pushIndex > 0) {
      const a = tr.q[tr.pushIndex - 1], b = tr.q[tr.pushIndex];
      c.arrow([a.x, a.y], [b.x, b.y], { color: '--warn', width: 2.4, head: 8 });
      c.text([b.x, b.y], '側推', { color: '--warn', dy: 16, bg: '--sim-bg' });
    }
    const qd = tr.qd[k], q = tr.q[k];
    drawBase(c, qd, ts.base, { ghost: true });
    c.text(toWorld(qd, [-0.05, 0.28]), 'q_d', { color: '--fg-muted' });
    drawBase(c, q, ts.base, { frame: true, frameLabel: 'b' });
    if (ts.showErr) drawErr(c, q, qd);
    // Gear-change labels go on top of the robot (it sits on the cusp while it changes direction).
    for (const cp of ref.cusps) c.text([cp.x, cp.y], '換檔：改倒車', { color: '--warn', dx: 10, dy: -16, align: 'left', bg: '--sim-bg' });
    startHandle.draw();
    const e = tr.err[k];
    c.setOverlayText(`t = ${fmt(tA, 2)} s\nφ_e = ${deg(e[0])}  x_e = ${fmt(e[1], 3)} m  y_e = ${fmt(e[2], 3)} m`);
  }

  function drawMM(c: Canvas2D): void {
    const tr = trialB;
    if (!tr) return;
    const k = indexAt(tr, tB);
    const v = c.visible();
    c.grid(0.5);
    // wall
    c.polygon([[v.xmin, WALL_Y], [v.xmax, WALL_Y], [v.xmax, v.ymax], [v.xmin, v.ymax]], { fill: '--bg-sunken', width: 0 });
    for (let x = Math.floor(v.xmin / 0.15) * 0.15; x < v.xmax; x += 0.15) c.line([x, WALL_Y], [x + 0.1, WALL_Y + 0.1], { color: '--grid-strong', width: 1 });
    c.line([v.xmin, WALL_Y], [v.xmax, WALL_Y], { color: '--fg-muted', width: 2 });
    c.text([v.xmax - 0.1, WALL_Y + 0.22], '牆面', { color: '--fg-muted', align: 'right' });
    // desired straight line for the end-effector
    c.line([0, LINE_Y], [ms.Bx, LINE_Y], { color: '--accent', width: 2.2, dash: [8, 5] });
    c.point([0, LINE_Y], 5, { color: '--accent' });
    c.text([0, LINE_Y], 'A', { color: '--accent', dx: -10, dy: -10 });
    // comparison robot (other redundancy solution) as a faint ghost
    if (ms.ghost && altB && page === 'mm') {
      const ka = indexAt(altB, tB);
      drawBase(c, altB.q[ka], ms.base, { ghost: true, alpha: 0.55 });
      drawArm(c, altB.q[ka], altB.theta[ka], { alpha: 0.22, width: 5, gripper: false });
    }
    const kb = Math.floor(k / 2) + 1;
    c.polyline(pathB, { color: '--series-1', width: 1.2, alpha: 0.25 });
    c.polyline([...pathB.slice(0, kb), [tr.q[k].x, tr.q[k].y]], { color: '--series-1', width: 2 });
    c.polyline([...eePathB.slice(0, kb), [tr.X[k].x, tr.X[k].y]], { color: '--accent', width: 2.6 });
    const q = tr.q[k], th = tr.theta[k], w = tr.w[k];
    drawBase(c, q, ms.base, { frame: true, frameLabel: 'b' });
    const pts = drawArm(c, q, th, { color: w < W_WARN ? '--danger' : '--fg-muted' });
    if (w < W_WARN) c.text(pts[1], '手臂接近奇異', { color: '--danger', dy: 18, bg: '--sim-bg' });
    // velocity contributions at the end-effector: base + arm = total
    const tip = pts[2];
    const vb = tr.vBase[k], va = tr.vArm[k];
    const p1: Pt = [tip[0] + ARROW_SCALE * vb[0], tip[1] + ARROW_SCALE * vb[1]];
    const p2: Pt = [p1[0] + ARROW_SCALE * va[0], p1[1] + ARROW_SCALE * va[1]];
    const big = (x: [number, number]) => Math.hypot(x[0], x[1]) > 0.03;
    labelledArrow(c, tip, p1, '--series-4', big(vb) ? '底盤' : '', { dx: 6, dy: 14 });
    labelledArrow(c, p1, p2, '--series-5', big(va) ? '手臂' : '', { dx: 6, dy: -12 });
    c.setOverlayText(`t = ${fmt(tB, 1)} s\nw = ${fmt(w, 2)}  θ₂ = ${deg(th[1], 0)}`);
  }

  cv.draw = (c) => { if (page === 'track') drawTrack(c); else drawMM(c); };

  // ---- animation ----------------------------------------------------------------
  let lastSlow = -Infinity;
  /** Close-up view centred between the robot and its reference (follow camera). */
  function followWorld(): World | null {
    if (!trialA) return null;
    const k = indexAt(trialA, tA);
    const q = trialA.q[k], qd = trialA.qd[k];
    const cx = (q.x + qd.x) / 2, cy = (q.y + qd.y) / 2;
    const H = FOLLOW_W / ASPECT_A;
    return { xmin: cx - FOLLOW_W / 2, xmax: cx + FOLLOW_W / 2, ymin: cy - H / 2, ymax: cy + H / 2 };
  }

  function frame(force = false): void {
    if (page === 'track' && ts.follow) { const w = followWorld(); if (w) cv.setWorld(w); }
    cv.requestRender();
    const now = performance.now();
    const t = curT();
    // Plot cursors repaint whole plots, and readouts touch the DOM: ~8 Hz while playing.
    if (force || !loop.running || now - lastSlow > 120 || t >= curEnd()) {
      lastSlow = now;
      if (page === 'track') { groupA.setCursor(t); updateReadoutsA(); } else { groupB.setCursor(t); updateReadoutsB(); }
    }
    pb.setTime(t);
  }
  // The runs are precomputed, so a slow frame may advance time by a large step without
  // affecting the result: playback stays real-time on slow machines instead of crawling.
  const loop = createLoop((dt) => {
    setCurT(curT() + dt);
    frame();
    if (curT() >= curEnd() - 1e-9) { loop.stop(); pb.setPlaying(false, { silent: true }); frame(true); }
  }, { maxDt: 0.25 });
  function play(): void { pb.setPlaying(true, { silent: true }); loop.start(); }

  // ---- pages & presets ----------------------------------------------------------
  function syncA(): void {
    selRef.set(ts.ref, { silent: true });
    selBase.set(ts.base, { silent: true });
    selFrame.set(ts.errFrame, { silent: true });
    tFF.set(ts.ff, { silent: true }); tFB.set(ts.fb, { silent: true });
    tPush.set(ts.push, { silent: true }); tErr.set(ts.showErr, { silent: true }); tFollow.set(ts.follow, { silent: true });
    sK1.set(ts.k1, { silent: true }); sK2.set(ts.k2, { silent: true }); sK3.set(ts.k3, { silent: true });
    sKp.set(ts.kp, { silent: true }); sKi.set(ts.ki, { silent: true });
    sBias.set(ts.bias, { silent: true }); sSpeed.set(ts.speed, { silent: true });
    const diff = ts.base === 'diff';
    for (const s of [sK1, sK2, sK3]) s.el.style.display = diff ? '' : 'none';
    for (const s of [sKp, sKi]) s.el.style.display = diff ? 'none' : '';
    selFrame.el.style.display = diff ? '' : 'none';
    errPlot.setOptions({ title: diff ? '誤差（機器人座標 {b}）' : '誤差（{b} 中的 X⁻¹X_d）' });
  }
  function syncB(): void {
    selMode.set(ms.mode, { silent: true });
    selBaseB.set(ms.base, { silent: true });
    tGhost.set(ms.ghost, { silent: true });
    sKn.set(ms.kn, { silent: true });
    sTf.set(ms.Tf, { silent: true });
  }

  function setPage(p: Page): void {
    const changed = p !== page;
    const wasA = page === 'track';
    page = p;
    tabBtns.forEach((b, i) => b.classList.toggle('active', PAGES[i][0] === p));
    const isA = p === 'track';
    pageA.style.display = isA ? '' : 'none';
    pageB.style.display = p === 'mm' ? '' : 'none';
    card.style.display = p === 'sum' ? '' : 'none';
    legA.style.display = isA ? '' : 'none';
    legB.style.display = isA ? 'none' : '';
    noteEl.textContent = isA
      ? '拖曳藍色把手改變起點（中心＝位置，小圓＝車頭方向），整段模擬立即重算；曲線圖的直線游標是目前時間。'
      : p === 'mm'
        ? `末端用 V = [Ad]V_d + K_p X_err（K_p = ${MM_KP}）沿直線前進，J_e^† 以極小阻尼（λ = 10⁻³）計算，J_e 掉秩也不會爆衝。拖曳 B 改變長度；虛線曲線與淡色機器人是另一種冗餘解法。`
        : '移動操作模擬仍在下方播放；切回「② 移動操作」可以調整解法。';
    startHandle.setEnabled(isA);
    bHandle.enabled = p === 'mm';
    if (!isA && !trialB) recomputeB(true);
    // Resizing reallocates the canvas, so only do it when the page (and its aspect) changes.
    if (changed && isA !== wasA) cv.setSize({ aspect: isA ? ASPECT_A : ASPECT_B });
    if (changed || isA) cv.setWorld(isA ? ((ts.follow && followWorld()) || worldA) : WORLD_B);
    pb.setDuration(curEnd());
    if (changed) frame(true);
  }

  function applyTrack(s: Partial<TrackState>): void {
    Object.assign(ts, TRACK_DEFAULT, s);
    refA = makeReference(ts.ref, ts.speed);
    syncA();
    setPage('track');
    tA = 0;
    recomputeA();
    play();
  }
  function applyMM(s: Partial<MMState>, p: Page): void {
    Object.assign(ms, MM_DEFAULT, s);
    syncB();
    setPage(p);
    tB = 0;
    recomputeB(true);
    play();
  }

  const trackChips = (list: TrackPreset[]): Preset[] => list.map((p) => ({ label: p.label, title: p.title, apply: () => applyTrack(p.s) }));
  const mmChips = (list: MMPreset[], p: Page): Preset[] => list.map((m) => ({ label: m.label, title: m.title, apply: () => applyMM(m.s, p) }));

  const stepSetup: { presets: () => Preset[]; active: number; focus: () => HTMLElement[] }[] = [
    { presets: () => trackChips(STEP1), active: 0, focus: () => [cv.el, kvA.el] },
    { presets: () => trackChips(STEP2), active: 0, focus: () => [errPlot.el, tFF.el, tFB.el] },
    { presets: () => trackChips(STEP3), active: 0, focus: () => [cv.el, selFrame.el] },
    { presets: () => trackChips(STEP4), active: 0, focus: () => [errPlot.el, sK2.el, sK3.el] },
    { presets: () => trackChips(STEP5), active: 0, focus: () => [uPlot.el, sKi.el] },
    { presets: () => mmChips(STEP6, 'mm'), active: 0, focus: () => [wPlot.el, selMode.el] },
    { presets: () => mmChips(STEP6.slice(0, 2), 'sum'), active: 1, focus: () => [card] },
  ];
  let focused: HTMLElement[] = [];
  function onStep(i: number): void {
    const st = stepSetup[Math.max(0, Math.min(stepSetup.length - 1, i))];
    chips.setPresets(st.presets());
    chips.activate(st.active);
    focused.forEach((f) => f.classList.remove('mr-focus'));
    focused = st.focus();
    focused.forEach((f) => f.classList.add('mr-focus'));
  }

  // Canvas2D / Plot2D / legends follow theme switches by themselves; repaint once for good measure.
  const unsubTheme = ctx.onThemeChange(() => { cv.requestRender(); frame(true); });
  onStep(ctx.currentStep);

  return {
    onStep,
    destroy() {
      destroyed = true;
      loop.stop();
      unsubTheme();
      startHandle.remove();
      cv.destroy();
      errPlot.destroy(); uPlot.destroy(); wPlot.destroy(); sPlot.destroy();
    },
  };
}

/** Summary card for the last step: what each chapter contributed, with links back. */
function summaryCard(): HTMLElement {
  const items = LINKS.map((it) => el('div', {
    style: { border: '1px solid var(--border)', borderRadius: 'var(--radius-sm)', padding: '0.35rem 0.55rem', background: 'var(--bg-elev)' },
  },
  el('div', { style: { fontWeight: '600', fontSize: '0.84rem' } }, el('span', { style: { color: 'var(--accent)', marginRight: '0.4rem' } }, it.ch), it.topic),
  el('div', { style: { fontSize: '0.78rem', color: 'var(--fg-muted)', margin: '0.1rem 0 0.15rem' } }, it.use),
  el('div', { style: { fontSize: '0.78rem', display: 'flex', flexWrap: 'wrap', gap: '0.15rem 0.6rem' } },
    it.links.map(([id, label]) => el('a', { href: `#/lesson/${id}` }, `→ ${label}`)))));
  return el('div', { style: { display: 'none', margin: '0.2rem 0 0.5rem' } },
    el('div', { style: { fontWeight: '600', margin: '0 0 0.35rem' } }, '這一課用到的工具（點連結回去複習）'),
    el('div', { style: { display: 'grid', gridTemplateColumns: 'repeat(auto-fit, minmax(165px, 1fr))', gap: '0.4rem' } }, items),
    el('div', { style: { fontSize: '0.8rem', color: 'var(--fg-muted)', marginTop: '0.4rem' } },
      '全部串起來就是一台移動操作機器人。', el('a', { href: '#/' }, '回到課程地圖'), '，挑出還不熟的章節再走一次。'));
}

const lesson: Lesson = { meta, steps, quiz, mount };
export default lesson;
