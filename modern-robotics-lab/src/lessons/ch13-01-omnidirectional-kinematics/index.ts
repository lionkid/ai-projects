import type { Lesson, LessonContext, LessonHandle } from '@/types';
import {
  el, fmt, fmtDeg, controlPanel, row, slider, select, button, toggle, matrixView, playback, simTitle, note, legend, kv,
  split, mdElement, Canvas2D, physicsRunner, keyboardDrive, presetChips, torqueBars,
  type BarsHandle, type Handle, type MatrixHighlight, type Preset, type Pt, type World,
} from '@/ui';
import { matVec, pinv, wrapToPi, type Mat, type Vec } from '@/mr';
import { meta } from './meta';
import { quiz, steps } from './content';
import {
  Hmatrix, bodyFromQdot, chassisFromWheels, clampTwist, contactVelocity, fourMecanum, isotropicSpeed, leftNullBasis,
  qdotFromBody, rankOf, sliceAt, sliceUnbounded, stepPose, threeOmni, wheelGeometry,
  type ClampResult, type Pose, type Twist2, type Wheel,
} from './wheels';

// ---------------------------------------------------------------------------------------------
// Robot geometry and view settings
// ---------------------------------------------------------------------------------------------

type ModelKey = 'mecanum4' | 'omni3';
type FrameMode = 'body' | 'world';
type ViewMode = 'map' | 'close';
/** A velocity command: ω plus two linear components in {b} ('body') or in {s} ('world'). */
interface Cmd { w: number; a: number; b: number; frame: FrameMode }

const R_W = 0.07;                 // wheel radius (m)
const OMNI_D = 0.22;              // omniwheel distance from the chassis centre (m)
const MEC_L = 0.25, MEC_W = 0.22; // mecanum wheel centres at (±l, ±w) (m)
// map periods are multiples of both grid spacings (0.5 m on the map, 0.2 m in the close-up), so the
// wrap-around is seamless
const MAP_WIDE: World = { xmin: -2.0, xmax: 2.0, ymin: -1.0, ymax: 1.0 };
const MAP_TALL: World = { xmin: -2.0, xmax: 2.0, ymin: -1.5, ymax: 1.5 };
const CLOSE: World = { xmin: -0.95, xmax: 0.95, ymin: -0.45, ymax: 0.55 };
const VBOX = 1.0;                 // half-range of the velocity plot (m/s)
const KEY_V = 0.35, KEY_W = 0.9;  // keyboard speeds
const H_STEP = 1 / 120;
const SUB = '₀₁₂₃₄₅₆₇₈₉';
const WHEEL_NAMES: Record<ModelKey, string[]> = {
  mecanum4: ['左前', '右前', '右後', '左後'],
  omni3: ['上', '右下', '左下'],
};
const MODEL_LABEL: Record<ModelKey, string> = { mecanum4: '四輪 mecanum', omni3: '三輪 omni' };

const add = (a: Pt, b: Pt, s = 1): Pt => [a[0] + s * b[0], a[1] + s * b[1]];
const dot2 = (a: Pt, b: Pt): number => a[0] * b[0] + a[1] * b[1];
const vecStr = (v: ArrayLike<number>, d = 2): string => `(${Array.from(v, (x) => fmt(x, d)).join(', ')})`;

function mount(host: HTMLElement, ctx: LessonContext): LessonHandle {
  // ------------------------------------------------------------------ state
  let model: ModelKey = 'mecanum4';
  let gammaMag = Math.PI / 4;
  let U = 10;
  let wheels: Wheel[] = [];
  let H: Mat = [];
  let Hp: Mat = [];
  let nullDirs: Vec[] = [];
  let unbounded = false;
  let rank = 3;
  let q: Pose = { phi: 0, x: 0, y: 0 };
  let frame: FrameMode = 'body';
  const sp = { w: 0, a: 0, b: 0 };              // joystick set-point (components in the frame `frame`)
  let auto: ((t: number) => Cmd) | null = null;  // scripted command (presets), cancelled by any manual input
  let autoT0 = 0;
  let wheelMode = false;                         // command wheel speeds directly instead of a twist
  let uCmd: number[] = [0, 0, 0, 0];
  let reqV: Twist2 = [0, 0, 0];
  let actV: Twist2 = [0, 0, 0];
  let clampInfo: ClampResult = { V: [0, 0, 0], scale: 1, omegaClamped: false, clamped: false };
  let u: number[] = [];
  let slip: number[] = [];
  let slipNorm = 0;
  const wheelAngle = [0, 0, 0, 0];
  let trail: Pt[][] = [[]];
  let trailCount = 0;
  let hlWheel = -1;
  let view: ViewMode = 'map';
  let simT = 0;
  let curStep = -1;
  let destroyed = false;
  let mapAspect = 2;
  let mapWorld: World = MAP_WIDE;
  let showSlide = false;

  // ------------------------------------------------------------------ layout
  simTitle(host, '全向底盤：底盤 twist ↔ 輪速', 'u = H(0) V_b');
  const cv = new Canvas2D(host, { aspect: 2, world: MAP_WIDE, overlay: true, margin: 0.02 });
  Object.assign(cv.overlay.style, {
    whiteSpace: 'pre', background: 'color-mix(in srgb, var(--sim-bg) 82%, transparent)', padding: '0.05rem 0.3rem', borderRadius: '4px',
  });
  const kb = keyboardDrive(cv.el, {
    hint: '點一下地圖啟用鍵盤：W/S 前後、A/D 側移、Q/E 自轉、空白鍵停',
    onChange: (pressed) => {
      if (pressed.size === 0) return;
      // manual driving takes over: drop scripted motion and the set-point, so releasing the keys stops the base
      auto = null;
      chips.clear();
      if (wheelMode) setWheelMode(false);
      sp.w = 0; sp.a = 0; sp.b = 0;
      sW.set(0, { silent: true });
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
    onReset: () => resetPose(),
    onSpeed: (s) => { runner.speed = s; },
  });
  const modelSel = select<ModelKey>(r1, {
    options: [{ value: 'mecanum4', label: '四輪 mecanum' }, { value: 'omni3', label: '三輪 omni' }],
    value: model,
    onChange: (m) => setModel(m),
  });
  const frameSel = select<FrameMode>(r1, {
    options: [{ value: 'body', label: '搖桿：機身 {b}' }, { value: 'world', label: '搖桿：世界 {s}' }],
    value: frame,
    onChange: (f) => setFrame(f),
  });
  button(r1, { label: '■ 停止', small: true, title: '把速度歸零（空白鍵）', onClick: () => brake() });

  const chipsHost = el('div', { style: { gridColumn: '1 / -1', display: 'flex', flexWrap: 'wrap', alignItems: 'center', gap: '0 0.9rem' } });
  panel.appendChild(chipsHost);
  const chips = presetChips(chipsHost, [], { label: '預設：' });
  const slideTg = toggle(chipsHost, { label: '顯示滑動方向', value: false, title: '在每顆輪子上畫出滾子的自由滑動方向', onChange: (v) => { showSlide = v; refresh(); } });

  const sW = slider(panel, {
    label: '自轉 ω', min: -2.5, max: 2.5, step: 0.05, value: 0, unit: 'rad/s',
    onInput: (v) => { takeManual(); sp.w = v; refresh(); },
  });
  const sU = slider(panel, {
    label: '輪速上限 u_max', min: 4, max: 12, step: 0.5, value: U, unit: 'rad/s',
    onInput: (v) => { U = v; onLimitChange(); },
  });
  const sG = slider(panel, {
    label: '滾子角 |γ|（mecanum）', min: 0, max: 60, step: 5, value: 45, unit: '°',
    onInput: (v) => { gammaMag = (v * Math.PI) / 180; rebuild(); refresh(); },
  });

  // direct wheel-speed command (step 5)
  const wheelBox = el('div', { style: { gridColumn: '1 / -1', display: 'none' } });
  panel.appendChild(wheelBox);
  const wheelTg = toggle(wheelBox, {
    label: '直接指定輪速 u（不經過 H）', value: false,
    onChange: (v) => { chips.clear(); setWheelMode(v); },
  });
  const wheelPanel = controlPanel(wheelBox, { compact: true });
  wheelPanel.style.gridTemplateColumns = 'repeat(auto-fit, minmax(115px, 1fr))';
  const wheelSliders = [0, 1, 2, 3].map((i) => slider(wheelPanel, {
    label: `u${SUB[i + 1]}`, min: -U, max: U, step: 0.5, value: 0, unit: 'rad/s',
    onInput: (v) => { uCmd[i] = v; chips.clear(); if (!wheelMode) setWheelMode(true); play(); refresh(); },
  }));

  const [leftCol, rightCol] = split(host);
  const cvV = new Canvas2D(leftCol, { aspect: 1.05, world: { xmin: -VBOX, xmax: VBOX, ymin: -VBOX, ymax: VBOX }, overlay: true, margin: 0.03 });
  Object.assign(cvV.overlay.style, { background: 'color-mix(in srgb, var(--sim-bg) 82%, transparent)', padding: '0.05rem 0.3rem', borderRadius: '4px' });
  const vStatus = el('div', { class: 'mr-note', style: { minHeight: '2.6em', margin: '0.25rem 0 0' } });
  leftCol.appendChild(vStatus);

  const eqStyle = { display: 'flex', flexWrap: 'wrap', alignItems: 'center', gap: '0.1rem 0.15rem' };
  const eqBox = el('div', { style: eqStyle });
  rightCol.appendChild(eqBox);
  const uView = matrixView(eqBox, { label: 'u =', digits: 1 });
  const HView = matrixView(eqBox, { label: '= H(0)', digits: 1 });
  const VView = matrixView(eqBox, { label: '', digits: 2 });
  // q̇ = R(φ)·V_b (step 6) takes the place of u = H V_b at the top of the column
  const qdBox = el('div', { style: { ...eqStyle, display: 'none' } });
  rightCol.appendChild(qdBox);
  const qdView = matrixView(qdBox, { label: 'q̇ =', digits: 2 });
  const RView = matrixView(qdBox, { label: '=', digits: 2 });
  const Vb2View = matrixView(qdBox, { label: '', digits: 2 });
  for (const m of [uView, HView, VView, qdView, RView, Vb2View]) Object.assign(m.el.style, { fontSize: '0.74rem', marginRight: '0.2rem' });
  const barsHost = el('div');
  rightCol.append(barsHost);
  // step 5 swaps the velocity plot for the skid bars (same column, so they stay in the first view)
  const slipHost = el('div', { style: { display: 'none' } });
  leftCol.appendChild(slipHost);
  let bars: BarsHandle | null = null;
  let slipBars: BarsHandle | null = null;
  const nullNote = el('div', { class: 'mr-note', style: { margin: '0.1rem 0 0.3rem' } });
  slipHost.appendChild(nullNote);

  const info = kv(rightCol, ['V_b 要求', 'V_b 實際']);
  const rowBox = el('div', { style: { display: 'none' } });
  rightCol.appendChild(rowBox);
  const rowInfo = kv(rowBox, ['輪子', 'pᵢ = (xᵢ, yᵢ)', 'βᵢ ／ γᵢ', 'd̂ᵢ（βᵢ + γᵢ 方向）', '力臂 (pᵢ × d̂ᵢ)_z', '1 / (rᵢ cos γᵢ)', 'hᵢ(0)', 'uᵢ = hᵢ(0) · V_b']);

  const legendMain = legend(host, [
    { color: 'var(--accent)', label: '輪子推動方向與轉速 r·u' },
    { color: 'var(--danger)', label: '超過 u_max' },
    { color: 'var(--series-6)', label: '底盤中心軌跡' },
    { color: 'var(--series-5)', label: '滾子自由滑動方向' },
  ]);
  const legendRow = legend(host, [
    { color: 'var(--fg)', label: '著地點速度' },
    { color: 'var(--warn)', label: '要靠輪子滾出（沿 d̂ᵢ）' },
    { color: 'var(--series-5)', label: '由滾子滑掉' },
    { color: 'var(--series-7)', label: '力臂 (pᵢ × d̂ᵢ)_z' },
  ]);
  legendRow.style.display = 'none';
  note(host, '輪子放大顯示；輪上的短線是著地滾子的軸向（沿 d̂ᵢ，與滑動方向垂直）。位姿每一小步用 SE(2) 指數映射 T ← T exp([V_b]Δt) 積分；地圖邊界是循環的。');
  const preview = mdElement(`**下一課預告：差速驅動**。兩顆沒有滾子的一般輪子裝在同一根軸上，輪子不能側滑，所以 $v_y\\equiv 0$，只剩 $(\\omega, v_x)$ 兩個輸入，$H$ 少了側移的能力。它到不了旁邊嗎？到得了——要靠前進、轉向、後退的組合「繞」過去。`, 'callout');
  preview.style.display = 'none';
  host.appendChild(preview);

  // ------------------------------------------------------------------ model
  function rebuild(): void {
    wheels = model === 'mecanum4' ? fourMecanum(MEC_L, MEC_W, R_W, gammaMag) : threeOmni(OMNI_D, R_W);
    H = Hmatrix(wheels);
    Hp = pinv(H);
    nullDirs = leftNullBasis(H);
    unbounded = sliceUnbounded(H);
    rank = rankOf(H);
    const m = wheels.length;
    u = new Array(m).fill(0);
    slip = new Array(m).fill(0);
    if (hlWheel >= m) hlWheel = m - 1;
    wheelSliders.forEach((s, i) => { s.el.style.display = i < m ? '' : 'none'; });
    sG.el.style.display = model === 'mecanum4' ? '' : 'none';
  }

  function buildBars(): void {
    barsHost.replaceChildren();
    for (const ch of [...slipHost.children]) if (ch !== nullNote) ch.remove();
    const labels = wheels.map((_, i) => `u${SUB[i + 1]}`);
    bars = torqueBars(barsHost, { labels, max: U * 1.2, limit: U, unit: 'rad/s', digits: 1, title: '輪速 u（紅線 = ±u_max）', colors: labels.map(() => '--accent') });
    slipBars = torqueBars(slipHost, { labels, max: 6, unit: 'rad/s', digits: 2, title: '打滑 u − H(0)·V̂_b（不對應任何底盤運動的部分）', colors: labels.map(() => '--danger') });
  }

  function setModel(m: ModelKey): void {
    if (m === model) return;
    model = m;
    modelSel.set(m, { silent: true });
    rebuild();
    buildBars();
    if (curStep >= 0) { chips.setPresets(presetsFor(curStep)); const d = defaultPreset(curStep); if (d >= 0) chips.activate(d); }
    refresh();
  }

  function setFrame(f: FrameMode): void {
    if (f === frame) return;
    // keep the motion continuous: re-express the linear set-point in the new frame
    const c = Math.cos(q.phi), s = Math.sin(q.phi);
    const [a, b] = f === 'world' ? [c * sp.a - s * sp.b, s * sp.a + c * sp.b] : [c * sp.a + s * sp.b, -s * sp.a + c * sp.b];
    sp.a = a; sp.b = b;
    frame = f;
    frameSel.set(f, { silent: true });
    auto = null;
    chips.clear();
    syncHandle();
    refresh();
  }

  function setWheelMode(v: boolean): void {
    wheelMode = v;
    wheelTg.set(v, { silent: true });
    if (v) auto = null;
    syncHandle();
    refresh();
  }

  function onLimitChange(): void {
    bars?.setMax(U * 1.2);
    bars?.setLimit(U);
    wheelSliders.forEach((s, i) => { s.setRange(-U, U); uCmd[i] = Math.max(-U, Math.min(U, uCmd[i])); s.set(uCmd[i], { silent: true }); });
    refresh();
  }

  // ------------------------------------------------------------------ simulation
  function currentCommand(): Cmd {
    if (kb.pressed.size > 0) {
      if (kb.has('brake')) return { w: 0, a: 0, b: 0, frame };
      const fwd = kb.axis('down', 'up');
      const side = kb.axis('right', 'left');
      const spin = (kb.has('a') ? 1 : 0) - (kb.has('b') ? 1 : 0);
      return frame === 'body'
        ? { w: spin * KEY_W, a: fwd * KEY_V, b: side * KEY_V, frame }
        : { w: spin * KEY_W, a: -side * KEY_V, b: fwd * KEY_V, frame };
    }
    if (auto) return auto(simT - autoT0);
    return { w: sp.w, a: sp.a, b: sp.b, frame };
  }

  function advance(h: number): void {
    simT += h;
    const cmd = currentCommand();
    const Vb: Twist2 = cmd.frame === 'body' ? [cmd.w, cmd.a, cmd.b] : bodyFromQdot(q.phi, [cmd.w, cmd.a, cmd.b]);
    reqV = Vb;
    clampInfo = clampTwist(H, U, Vb);
    const m = wheels.length;
    const uu = wheelMode ? uCmd.slice(0, m) : matVec(H, clampInfo.V);
    // the chassis follows the least-squares twist of the wheel speeds (exact when they are consistent)
    const fit = chassisFromWheels(H, Hp, uu);
    u = uu;
    actV = fit.V;
    slip = fit.slip;
    slipNorm = fit.slipNorm;
    q = stepPose(q, actV, h);
    q.phi = wrapToPi(q.phi);
    for (let i = 0; i < m; i++) wheelAngle[i] += uu[i] * h;
    wrapAndTrail();
  }

  function wrapAndTrail(): void {
    const W = mapWorld;
    const wx = W.xmax - W.xmin, wy = W.ymax - W.ymin;
    let wrapped = false;
    if (q.x > W.xmax) { q.x -= wx; wrapped = true; } else if (q.x < W.xmin) { q.x += wx; wrapped = true; }
    if (q.y > W.ymax) { q.y -= wy; wrapped = true; } else if (q.y < W.ymin) { q.y += wy; wrapped = true; }
    if (wrapped) trail.push([]);
    const seg = trail[trail.length - 1];
    const last = seg[seg.length - 1];
    if (!last || Math.hypot(q.x - last[0], q.y - last[1]) > 0.012) {
      seg.push([q.x, q.y]);
      trailCount++;
      while (trailCount > 2200 && trail.length) {
        const first = trail[0];
        const drop = Math.min(first.length, trailCount - 2000);
        first.splice(0, drop);
        trailCount -= drop;
        if (!first.length && trail.length > 1) trail.shift();
        else if (!first.length) break;
      }
    }
  }

  function clearTrail(): void { trail = [[[q.x, q.y]]]; trailCount = 1; }

  /** Run the model silently for `seconds` (gives every preset an informative first view). */
  function prerun(seconds: number): void {
    const n = Math.round(seconds / H_STEP);
    for (let k = 0; k < n; k++) advance(H_STEP);
  }

  const runner = physicsRunner({
    h: H_STEP,
    step: (h) => advance(h),
    onFrame: () => refresh(),
    guard: () => Number.isFinite(q.x) && Number.isFinite(q.y) && Number.isFinite(q.phi),
    onBlowUp: () => { q = { phi: 0, x: 0, y: 0 }; clearTrail(); },
  });

  function play(): void { if (!runner.running) { runner.start(); pb.setPlaying(true, { silent: true }); } }

  function takeManual(): void { auto = null; chips.clear(); if (wheelMode) setWheelMode(false); play(); }

  function brake(): void {
    auto = null;
    chips.clear();
    sp.w = 0; sp.a = 0; sp.b = 0;
    sW.set(0, { silent: true });
    uCmd = [0, 0, 0, 0];
    wheelSliders.forEach((s) => s.set(0, { silent: true }));
    refresh();
  }

  function resetPose(): void {
    const i = chips.active;
    if (i >= 0) { chips.activate(i); return; }
    q = { phi: 0, x: 0, y: 0 };
    clearTrail();
    refresh();
  }

  interface MotionOpts { frame: FrameMode; w: number; a: number; b: number; pose?: Pose; prerun?: number; auto?: (t: number) => Cmd }

  function startMotion(o: MotionOpts): void {
    if (wheelMode) setWheelMode(false);
    frame = o.frame;
    frameSel.set(frame, { silent: true });
    sp.w = o.w; sp.a = o.a; sp.b = o.b;
    sW.set(o.w, { silent: true });
    auto = o.auto ?? null;
    autoT0 = simT;
    if (o.pose) { q = { ...o.pose }; clearTrail(); }
    if (o.prerun) prerun(o.prerun);
    syncHandle();
    play();
    refresh();
  }

  function wheelPreset(uu: number[]): void {
    auto = null;
    uCmd = [0, 1, 2, 3].map((i) => Math.max(-U, Math.min(U, uu[i] ?? 0)));
    wheelSliders.forEach((s, i) => s.set(uCmd[i], { silent: true }));
    setWheelMode(true);
    q = { phi: 0, x: 0.9, y: -0.25 };
    clearTrail();
    prerun(1.5);
    play();
    refresh();
  }

  /** Vertex of the ω = 0 slice farthest from the origin; ties go to the direction closest to `prefer` (rad). */
  function fastestVertex(omega: number, prefer: number): Pt | null {
    if (unbounded) return null;
    const poly = sliceAt(H, U, omega, VBOX * 4);
    let best: Pt | null = null, bestR = -1, bestAng = Infinity;
    for (const p of poly) {
      const r = Math.hypot(p[0], p[1]);
      const ang = Math.abs(wrapToPi(Math.atan2(p[1], p[0]) - prefer));
      if (r > bestR + 1e-6 || (Math.abs(r - bestR) <= 1e-6 && ang < bestAng)) { best = p; bestR = r; bestAng = ang; }
    }
    return best;
  }

  // ------------------------------------------------------------------ presets per step
  const P = (label: string, apply: () => void, title?: string): Preset => ({ label, apply, title });
  const pose = (x: number, y: number, phi = 0): Pose => ({ phi, x, y });

  const tourPresets = (): Preset[] => [
    P('邊轉邊直走', () => startMotion({ frame: 'world', w: 0.7, a: 0.22, b: 0, pose: pose(-1.75, 0.3), prerun: 6 }), '世界座標速度固定、同時自轉'),
    P('前進', () => startMotion({ frame: 'body', w: 0, a: 0.3, b: 0, pose: pose(-1.4, -0.4), prerun: 2 })),
    P('側移', () => startMotion({ frame: 'body', w: 0, a: 0, b: 0.3, pose: pose(-0.7, -0.75), prerun: 2 })),
    P('原地轉', () => startMotion({ frame: 'body', w: 0.8, a: 0, b: 0, pose: pose(0.8, -0.2), prerun: 1 })),
    P('繞圈', () => startMotion({ frame: 'body', w: 0.5, a: 0.3, b: 0, pose: pose(0, -0.6), prerun: 9 }), '固定的 V_b ⇒ 等速圓周'),
    P('斜走 45°', () => startMotion({ frame: 'body', w: 0, a: 0.25, b: 0.25, pose: pose(-1.4, -0.8), prerun: 3 })),
  ];
  const purePresets = (): Preset[] => [
    P('前進', () => startMotion({ frame: 'body', w: 0, a: 0.3, b: 0, pose: pose(-1.4, -0.4), prerun: 2 })),
    P('側移', () => startMotion({ frame: 'body', w: 0, a: 0, b: 0.3, pose: pose(-0.7, -0.75), prerun: 2 })),
    P('原地轉', () => startMotion({ frame: 'body', w: 0.8, a: 0, b: 0, pose: pose(0.8, -0.2), prerun: 1 })),
    P('斜走 45°', () => startMotion({ frame: 'body', w: 0, a: 0.25, b: 0.25, pose: pose(-1.4, -0.8), prerun: 3 })),
  ];
  const rowPresets = (): Preset[] => wheels.map((_, i) => P(`輪 ${i + 1}（${WHEEL_NAMES[model][i]}）`, () => { hlWheel = i; refresh(); }));
  const limitPresets = (): Preset[] => [
    P('要求過快', () => startMotion({ frame: 'body', w: 0, a: 0.9, b: 0.4, pose: pose(-1.7, -0.5), prerun: 1 }), '搖桿點落在多面體外'),
    P('最快方向', () => {
      const v = fastestVertex(0, model === 'mecanum4' ? 0 : Math.PI / 2) ?? [0.6, 0];
      // start half a pre-run behind (−0.9, −0.35), so the base ends up mid-map whichever way is fastest
      const pre = 1.5, k = 0.97;
      startMotion({ frame: 'body', w: 0, a: k * v[0], b: k * v[1], pose: pose(-0.9 - 0.5 * pre * k * v[0], -0.35 - 0.5 * pre * k * v[1]), prerun: pre });
    }, '沿 ω = 0 切片最遠的頂點'),
    P('斜 45°（較慢）', () => startMotion({ frame: 'body', w: 0, a: 0.6, b: 0.6, pose: pose(-1.6, -0.8), prerun: 1.5 })),
    P('邊轉邊直走（太快）', () => startMotion({ frame: 'world', w: 1.0, a: 0.4, b: 0, pose: pose(-1.7, 0.2), prerun: 3 }), '自轉吃掉輪速，平移被迫放慢'),
  ];
  const wheelPresets = (): Preset[] => {
    const m = wheels.length;
    const list: Preset[] = [];
    if (nullDirs.length) {
      const n = nullDirs[0];
      const k = 6 / Math.max(...n.map(Math.abs));
      list.push(P('內耗組合 n', () => wheelPreset(n.map((x) => k * x)), '前輪往前、後輪往後：H 的左零空間'));
    }
    for (let i = 0; i < m; i++) list.push(P(`單輪 ${i + 1}`, () => { const uu = [0, 0, 0, 0]; uu[i] = 6; wheelPreset(uu); }));
    list.push(P('一致：u = H·(0, 0, 0.3)', () => wheelPreset(matVec(H, [0, 0, 0.3]))));
    list.push(P('歸零', () => wheelPreset([0, 0, 0, 0])));
    return list;
  };
  const worldPresets = (): Preset[] => [
    P('邊轉邊走正方形', () => startMotion({
      // left half of the map: the keyboard hint sits at the bottom right until the canvas is focused
      frame: 'world', w: 0.6, a: 0.2, b: 0, pose: pose(-1.3, -0.45), prerun: 15.5,
      auto: (t) => { const k = Math.floor(t / 4) % 4; const a = (k * Math.PI) / 2; return { w: 0.6, a: 0.2 * Math.cos(a), b: 0.2 * Math.sin(a), frame: 'world' }; },
    }), '世界速度每 4 s 轉 90°，車身一直自轉'),
    P('邊轉邊直走', () => startMotion({ frame: 'world', w: 0.7, a: 0.22, b: 0, pose: pose(-1.75, 0.3), prerun: 6 })),
    P('繞圈（車頭朝前）', () => startMotion({ frame: 'body', w: 0.5, a: 0.3, b: 0, pose: pose(0, -0.6), prerun: 9 })),
  ];

  function presetsFor(i: number): Preset[] {
    switch (i) {
      case 0: return tourPresets();
      case 1: return rowPresets();
      case 2: return purePresets();
      case 3: return limitPresets();
      case 4: return wheelPresets();
      default: return worldPresets();
    }
  }
  const defaultPreset = (i: number): number => (i === 1 ? Math.max(0, hlWheel) : i === 2 ? 1 : 0);

  // ------------------------------------------------------------------ velocity plot (also the joystick)
  const stick: Handle = {
    get: () => [sp.a, sp.b],
    set: (p) => {
      auto = null;
      chips.clear();
      sp.a = Math.max(-VBOX, Math.min(VBOX, p[0]));
      sp.b = Math.max(-VBOX, Math.min(VBOX, p[1]));
      play();
      refresh();
    },
    color: '--warn',
    label: '搖桿',
    radius: 9,
  };
  cvV.addHandle(stick);
  function syncHandle(): void { stick.enabled = frame === 'body' && !wheelMode; cvV.requestRender(); }

  cvV.draw = (c) => {
    c.grid(0.2);
    c.text([VBOX * 0.9, 0], 'v_x', { color: '--axis-x', dy: -10 });
    c.text([0, VBOX * 0.78], 'v_y', { color: '--axis-y', dx: 14 });
    const w = clampInfo.V[0];
    const poly = sliceAt(H, U, w, VBOX * 4);
    if (poly.length) {
      c.polygon(poly, { color: '--accent', fill: '--accent', alpha: 0.13, width: 0 });
      c.polygon(poly, { color: '--accent', width: 2 });
    }
    const iso = isotropicSpeed(H, U, w);
    if (Number.isFinite(iso) && iso > 1e-3) c.circle([0, 0], iso, { color: '--fg-muted', width: 1.2, dash: [4, 4] });
    const best = fastestVertex(w, model === 'mecanum4' ? 0 : Math.PI / 2);
    if (best) {
      c.line([0, 0], best, { color: '--ok', width: 1.2, dash: [2, 3] });
      c.point(best, 4.5, { color: '--ok' });
      c.text(best, `最快 ${fmt(Math.hypot(best[0], best[1]), 2)} m/s`, { color: '--ok', dy: 15, dx: best[0] > 0.45 ? -26 : 0, bg: '--sim-bg' });
    }
    if (unbounded) c.text([0, VBOX * 0.72], 'v_y 不受輪速限制（推不動，也擋不住）', { color: '--danger', bg: '--sim-bg' });
    const req: Pt = [reqV[1], reqV[2]];
    const act: Pt = [actV[1], actV[2]];
    if (wheelMode) {
      c.point(act, 6, { color: '--accent' });
      c.text(act, 'V̂_b', { color: '--accent', dx: 14, dy: -10 });
    } else {
      if (clampInfo.clamped) {
        const cl: Pt = [clampInfo.V[1], clampInfo.V[2]];
        c.line(req, cl, { color: '--danger', width: 1.5, dash: [3, 3] });
        c.circle(req, 0.035, { color: '--danger', width: 1.5 });
      }
      if (frame === 'world' || kb.pressed.size > 0 || auto) c.circle(req, 0.035, { color: '--warn', width: 2 });
      c.point(act, 5, { color: '--accent' });
    }
    cvV.setOverlayText(`ω = ${fmt(w, 2)} rad/s 時的 (v_x, v_y) 切片`);
  };

  // ------------------------------------------------------------------ map drawing
  function drawRotatedGrid(c: Canvas2D, w2v: (p: Pt) => Pt): void {
    const R = 1.4, st = 0.2;
    const x0 = Math.floor((q.x - R) / st) * st, y0 = Math.floor((q.y - R) / st) * st;
    for (let x = x0; x <= q.x + R; x += st) c.line(w2v([x, q.y - R]), w2v([x, q.y + R]), { color: '--grid', width: 1 });
    for (let y = y0; y <= q.y + R; y += st) c.line(w2v([q.x - R, y]), w2v([q.x + R, y]), { color: '--grid', width: 1 });
  }

  function drawChassis(c: Canvas2D, b2v: (p: Pt) => Pt): void {
    if (model === 'mecanum4') {
      const hx = 0.31, hy = 0.165;
      const pts: Pt[] = [[-hx, -hy], [hx, -hy], [hx, hy], [-hx, hy]];
      c.polygon(pts.map(b2v), { color: '--fg-muted', fill: '--bg-sunken', width: 1.5 });
      c.line(b2v([hx - 0.05, -hy + 0.03]), b2v([hx - 0.05, hy - 0.03]), { color: '--fg-muted', width: 1.2 });
    } else {
      const pts: Pt[] = Array.from({ length: 36 }, (_, k) => {
        const a = (2 * Math.PI * k) / 36;
        return [0.27 * Math.cos(a), 0.27 * Math.sin(a)] as Pt;
      });
      c.polygon(pts.map(b2v), { color: '--fg-muted', fill: '--bg-sunken', width: 1.5 });
    }
  }

  function drawWheel(c: Canvas2D, w: Wheel, i: number, b2v: (p: Pt) => Pt): void {
    const hl = i === hlWheel && view === 'close';
    const g = wheelGeometry(w);
    const P0: Pt = [w.x, w.y];
    const along = g.drive, across: Pt = [-g.drive[1], g.drive[0]];
    const L = w.r, Wd = 0.028;
    const corners: Pt[] = [add(add(P0, along, -L), across, -Wd), add(add(P0, along, L), across, -Wd), add(add(P0, along, L), across, Wd), add(add(P0, along, -L), across, Wd)];
    c.polygon(corners.map(b2v), { color: hl ? '--accent' : '--fg', fill: '--bg-elev', width: hl ? 2.5 : 1.4 });
    // rollers: short strokes along the roller axis (β + γ), drifting with the wheel rotation
    const spacing = 0.034;
    const off = (((w.r * wheelAngle[i]) % spacing) + spacing) % spacing;
    const ra: Pt = g.dir;
    const rl = 0.012;
    for (let s = -L + off; s <= L; s += spacing) {
      if (s < -L + 0.013 || s > L - 0.013) continue;
      const cc = add(P0, along, s);
      c.line(b2v(add(cc, ra, -rl)), b2v(add(cc, ra, rl)), { color: '--fg-muted', width: 1.6 });
    }
    // free-sliding direction of the contact roller
    if (showSlide || hl) c.line(b2v(add(P0, g.slide, -0.11)), b2v(add(P0, g.slide, 0.11)), { color: '--series-5', width: 1.6, dash: [4, 3] });
    // driving speed arrow, length ∝ r·u
    const ui = u[i] ?? 0;
    const k = view === 'map' ? 0.8 : 0.45;
    if (Math.abs(ui) > 0.05) {
      const sat = Math.abs(ui) > U * (1 + 1e-6);
      c.arrow(b2v(P0), b2v(add(P0, along, k * w.r * ui)), { color: sat ? '--danger' : '--accent', width: 2.5, head: 8 });
    }
    // wheel number, placed outside the chassis; mecanum numbers lean towards the chassis centre so
    // they stay clear of the γ arc that the close-up draws on the outer side of the axle. The
    // highlighted wheel of the close-up is already named by its chip, outline and the row readout,
    // and its number would sit under the annotation, so it is left out.
    if (hl) return;
    const rad = Math.hypot(w.x, w.y) || 1;
    const out: Pt = model === 'mecanum4' ? [-0.62 * Math.sign(w.x), 0.78 * Math.sign(w.y)] : [w.x / rad, w.y / rad];
    c.text(b2v(add(P0, out, view === 'map' ? 0.12 : 0.075)), String(i + 1), { color: hl ? '--accent' : '--fg-muted', font: '600 12px system-ui, sans-serif' });
  }

  function drawRowAnnotation(c: Canvas2D, i: number, b2v: (p: Pt) => Pt): void {
    const w = wheels[i];
    if (!w) return;
    const g = wheelGeometry(w);
    const P0: Pt = [w.x, w.y];
    // position vector p_i
    c.line(b2v([0, 0]), b2v(P0), { color: '--fg-muted', width: 1.4, dash: [5, 4] });
    c.text(b2v([P0[0] * 0.55, P0[1] * 0.55]), 'pᵢ', { color: '--fg-muted', dx: 9, dy: -9 });
    // line through the wheel along d̂_i and the moment arm from the chassis centre
    const pd = dot2(P0, g.dir);
    const foot: Pt = [P0[0] - pd * g.dir[0], P0[1] - pd * g.dir[1]];
    c.line(b2v(add(foot, g.dir, -0.2)), b2v(add(P0, g.dir, 0.35)), { color: '--warn', width: 1, dash: [2, 4], alpha: 0.8 });
    // contact-point velocity split into "rolled by the wheel" + "slid by the roller"
    const vc = contactVelocity(w, actV);
    const k = 0.9;
    const alongD = dot2(vc, g.dir);
    const tipD = add(P0, g.dir, k * alongD);
    const tipV: Pt = [P0[0] + k * vc[0], P0[1] + k * vc[1]];
    const fl = Math.hypot(foot[0], foot[1]);
    if (fl > 0.01) {
      c.line(b2v([0, 0]), b2v(foot), { color: '--series-7', width: 3 });
      // short label beside the arm, at the candidate spot farthest from the other annotations
      // (wheel, velocity arrows, {b} axis labels); the value itself is in the row readout
      const off: Pt = [foot[1] / fl, -foot[0] / fl]; // arm rotated by −90°
      const avoid: Pt[] = [P0, tipV, tipD, [0, 0.14], [0.14, 0], [0, 0]];
      let best: { p: Pt; s: number; d: number } = { p: [foot[0] * 0.5, foot[1] * 0.5], s: 1, d: -1 };
      for (const t of [0.35, 0.5, 0.7]) {
        for (const s of [1, -1]) {
          const q2: Pt = [foot[0] * t + s * 0.09 * off[0], foot[1] * t + s * 0.09 * off[1]];
          const d = Math.min(...avoid.map((a) => Math.hypot(q2[0] - a[0], q2[1] - a[1])));
          if (d > best.d) best = { p: [foot[0] * t, foot[1] * t], s, d };
        }
      }
      c.text(b2v(best.p), '力臂', { color: '--series-7', bg: '--sim-bg', dx: 26 * best.s * off[0], dy: -26 * best.s * off[1] });
    }
    const vn = Math.hypot(vc[0], vc[1]);
    if (vn > 0.01) {
      c.arrow(b2v(P0), b2v(tipV), { color: '--fg', width: 2, head: 8 });
      c.arrow(b2v(P0), b2v(tipD), { color: '--warn', width: 3.2, head: 9 });
      c.arrow(b2v(tipD), b2v(tipV), { color: '--series-5', width: 2, dash: [4, 3], head: 7 });
      // label just beyond the arrow head, on the side the arrow points to
      const ux = vc[0] / vn, uy = vc[1] / vn;
      c.text(b2v(tipV), '著地點速度', { color: '--fg', bg: '--sim-bg', align: ux >= 0 ? 'left' : 'right', dx: 8 * Math.sign(ux || 1), dy: -12 * uy - 2 });
    }
    // γ: angle between the wheel axle (β + 90°) and the free-sliding direction
    if (Math.abs(w.gamma) > 1e-3) {
      const a0 = w.beta + Math.PI / 2;
      const r = 0.075;
      const ctr = b2v(P0);
      c.arc(ctr, r, Math.min(a0, a0 + w.gamma), Math.max(a0, a0 + w.gamma), { color: '--series-5', width: 1.8 });
      c.line(ctr, add(ctr, [Math.cos(a0), Math.sin(a0)], r * 1.25), { color: '--fg-muted', width: 1, dash: [2, 2] });
      const am = a0 + w.gamma / 2;
      c.text(add(ctr, [Math.cos(am), Math.sin(am)], r + 0.035), 'γ', { color: '--series-5' });
    }
    if (Math.abs(w.beta) > 1e-3) {
      const ctr = b2v(P0);
      const r = 0.055;
      c.line(ctr, add(ctr, [1, 0], r * 1.3), { color: '--fg-muted', width: 1, dash: [2, 2] });
      c.arc(ctr, r, Math.min(0, w.beta), Math.max(0, w.beta), { color: '--warn', width: 1.6 });
      c.text(add(ctr, [Math.cos(w.beta / 2), Math.sin(w.beta / 2)], r + 0.035), 'β', { color: '--warn' });
    }
  }

  cv.draw = (c) => {
    const cph = Math.cos(q.phi), sph = Math.sin(q.phi);
    const b2v = (p: Pt): Pt => (view === 'map' ? [q.x + cph * p[0] - sph * p[1], q.y + sph * p[0] + cph * p[1]] : p);
    const w2v = (p: Pt): Pt => {
      if (view === 'map') return p;
      const dx = p[0] - q.x, dy = p[1] - q.y;
      return [cph * dx + sph * dy, -sph * dx + cph * dy];
    };
    if (view === 'map') {
      c.grid(0.5);
      c.frame({ x: 0, y: 0, theta: 0 }, 0.3, { labels: 's', width: 1.5, alpha: 0.6 });
    } else {
      drawRotatedGrid(c, w2v);
    }
    for (const seg of trail) if (seg.length > 1) c.polyline(view === 'map' ? seg : seg.map(w2v), { color: '--series-6', width: 2, alpha: 0.85 });
    drawChassis(c, b2v);
    wheels.forEach((w, i) => drawWheel(c, w, i, b2v));
    const o = b2v([0, 0]);
    c.frame({ x: o[0], y: o[1], theta: view === 'map' ? q.phi : 0 }, view === 'map' ? 0.2 : 0.11, { labels: 'b', width: 2 });
    if (view === 'close' && hlWheel >= 0) drawRowAnnotation(c, hlWheel, b2v);
    const qd = qdotFromBody(q.phi, actV);
    const sat = !wheelMode && clampInfo.clamped ? '   ⚠ 已夾制' : '';
    c.setOverlayText(`φ = ${fmtDeg(q.phi, 0)}   (x, y) = (${fmt(q.x, 2)}, ${fmt(q.y, 2)}) m\nV_b = ${vecStr(actV)}   |v| = ${fmt(Math.hypot(qd[1], qd[2]), 2)} m/s${sat}`);
  };

  // ------------------------------------------------------------------ readouts
  function updatePanel(): void {
    const m = wheels.length;
    const HlRows: MatrixHighlight = wheels.map((_, i) => (curStep === 1 && i === hlWheel ? 'hl' : null));
    uView.setLabel(wheelMode ? 'u（指令）=' : 'u =');
    HView.setLabel(wheelMode ? '≈ H(0)' : '= H(0)');
    uView.set(u.slice(0, m), { highlightRows: HlRows });
    HView.set(H, { highlightRows: HlRows });
    VView.set(wheelMode ? actV : clampInfo.V);
    bars?.set(u);
    slipBars?.set(slip);
    info.set('V_b 要求', wheelMode ? '（直接指定輪速）' : vecStr(reqV));
    info.set('V_b 實際', vecStr(actV));
    if (curStep === 1 && hlWheel >= 0 && wheels[hlWheel]) {
      const w = wheels[hlWheel];
      const g = wheelGeometry(w);
      const h = H[hlWheel];
      rowInfo.set('輪子', `輪 ${hlWheel + 1}（${WHEEL_NAMES[model][hlWheel]}）`);
      rowInfo.set('pᵢ = (xᵢ, yᵢ)', `(${fmt(w.x, 3)}, ${fmt(w.y, 3)}) m`);
      rowInfo.set('βᵢ ／ γᵢ', `${fmtDeg(w.beta, 0)} ／ ${fmtDeg(w.gamma, 0)}`);
      rowInfo.set('d̂ᵢ（βᵢ + γᵢ 方向）', vecStr(g.dir, 3));
      rowInfo.set('力臂 (pᵢ × d̂ᵢ)_z', `${fmt(g.lever, 3)} m`);
      rowInfo.set('1 / (rᵢ cos γᵢ)', `${fmt(g.gain, 2)} 1/m`);
      rowInfo.set('hᵢ(0)', `[${h.map((x) => fmt(x, 2)).join(', ')}]`);
      rowInfo.set('uᵢ = hᵢ(0) · V_b', `${fmt(u[hlWheel] ?? 0, 2)} rad/s`);
    }
    if (curStep === 5) {
      const c = Math.cos(q.phi), s = Math.sin(q.phi);
      qdView.set(qdotFromBody(q.phi, actV));
      RView.set([[1, 0, 0], [0, c, -s], [0, s, c]]);
      Vb2View.set(actV);
    }
    // status under the velocity plot
    let st = '';
    if (wheelMode) {
      st = slipNorm > 0.05
        ? `⚠ 輪速不一致：‖u − H·V̂_b‖ = ${fmt(slipNorm, 2)} rad/s 的部分只會打滑。`
        : '✓ 輪速一致：u 落在 H 的欄空間裡，沒有打滑。';
    } else if (clampInfo.omegaClamped) {
      st = `⚠ ω 超過純自轉上限，已降到 ${fmt(clampInfo.V[0], 2)} rad/s；切片縮成一點附近。`;
    } else if (clampInfo.clamped) {
      st = `⚠ 要求的速度在多面體外，已沿原方向縮回邊界（× ${fmt(clampInfo.scale, 2)}）。`;
    } else {
      st = frame === 'world' ? '世界座標命令：圓圈是目前的 V_b，會隨車頭轉動在切片裡繞。' : '✓ 要求的 V_b 在多面體內：每顆輪子都在上限以內。';
    }
    if (rank < 3) st += ' 目前 H 只有秩 2：輪子推不出側向速度。';
    if (vStatus.textContent !== st) vStatus.textContent = st;
    const nn = nullDirs.length
      ? `H 的左零空間（${nullDirs.length} 維）：n = ${vecStr(nullDirs[0], 2)}${nullDirs.length > 1 ? ' …' : ''}`
      : `${MODEL_LABEL[model]}：H 是可逆方陣，任何輪速都對應唯一的底盤運動。`;
    if (nullNote.textContent !== nn) nullNote.textContent = nn;
  }

  function refresh(): void {
    if (destroyed) return;
    cv.requestRender();
    cvV.requestRender();
    updatePanel();
  }

  // ------------------------------------------------------------------ layout adaptation (phones)
  function applyView(v: ViewMode): void {
    view = v;
    cv.setWorld(v === 'close' ? CLOSE : mapWorld);
  }
  let fitQueued = false;
  const fitMap = () => {
    fitQueued = false;
    if (destroyed) return;
    const w = cv.el.clientWidth;
    const a = w > 0 && w < 520 ? 1.5 : 2;
    if (a !== mapAspect) {
      mapAspect = a;
      mapWorld = a < 1.8 ? MAP_TALL : MAP_WIDE;
      cv.setSize({ aspect: a });
      applyView(view);
    }
  };
  const ro = new ResizeObserver(() => { if (!fitQueued) { fitQueued = true; requestAnimationFrame(fitMap); } });
  ro.observe(host);

  // ------------------------------------------------------------------ steps
  let focused: HTMLElement[] = [];
  function setFocus(i: number): void {
    focused.forEach((f) => f.classList.remove('mr-focus'));
    const targets: (HTMLElement | null)[][] = [
      [cv.el],
      [cv.el, rowBox],
      [eqBox],
      [cvV.el, sU.el],
      [slipHost, chipsHost],
      [cv.el, qdBox],
    ];
    focused = (targets[i] ?? []).filter((x): x is HTMLElement => !!x);
    focused.forEach((f) => f.classList.add('mr-focus'));
  }

  function onStep(i: number): void {
    const prev = curStep;
    curStep = i;
    if (i === 4 && model !== 'mecanum4') setModel('mecanum4');
    if (i === 1 && hlWheel < 0) hlWheel = 0;
    if (i !== 1) hlWheel = -1;
    applyView(i === 1 ? 'close' : 'map');
    wheelBox.style.display = i === 4 ? '' : 'none';
    slipHost.style.display = i === 4 ? '' : 'none';
    cvV.el.style.display = i === 4 ? 'none' : '';
    sW.el.style.display = i === 4 ? 'none' : '';
    rowBox.style.display = i === 1 ? '' : 'none';
    eqBox.style.display = i === 5 ? 'none' : '';
    barsHost.style.display = i === 1 ? 'none' : '';
    info.el.style.display = i === 1 ? 'none' : '';
    qdBox.style.display = i === 5 ? '' : 'none';
    preview.style.display = i === 5 ? '' : 'none';
    legendRow.style.display = i === 1 ? '' : 'none';
    legendMain.style.display = i === 1 ? 'none' : '';
    if (i !== 4 && wheelMode) setWheelMode(false);
    slideTg.set(i === 1, { silent: true });
    showSlide = i === 1;
    chips.setPresets(presetsFor(i));
    if (i === 1) {
      // the close-up follows the base while it creeps forward and turns slowly, so V_b ≠ 0 and every wheel works
      startMotion({ frame: 'body', w: 0.2, a: 0.25, b: 0, pose: pose(0, 0), prerun: 0.5 });
      chips.highlight(hlWheel);
    } else if (prev !== i) {
      chips.activate(defaultPreset(i));
    }
    setFocus(i);
    refresh();
  }

  // ------------------------------------------------------------------ boot
  rebuild();
  buildBars();
  syncHandle();
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
      cvV.destroy();
    },
  };
}

const lesson: Lesson = { meta, steps, quiz, mount };
export default lesson;
