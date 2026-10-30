import type { Lesson, LessonContext, LessonHandle } from '@/types';
import {
  el, cssVar, fmt, controlPanel, row, slider, angleSlider, playback, readout, simTitle, note,
  Canvas2D, Plot2D, Scene3D, createLoop, substep, type Pt,
} from '@/ui';
import { meta } from './meta';

/**
 * Reference lesson. It exercises every toolkit piece once (2D canvas with a drag handle,
 * streaming plot, 3D scene, sliders, playback) so it doubles as a template for authors.
 */

const steps = [
  {
    title: '歡迎！這一頁長什麼樣子',
    md: `這個實驗室把 *Modern Robotics* 的每個核心概念拆成一課。每一課都有三個區塊：

1. **分步導讀**（你正在讀的這裡）：一次只講一個想法，搭配公式與直覺。
2. **互動模擬**（右側）：可以拖曳、拉滑桿、播放的小實驗，會隨著步驟切換焦點。
3. **自我檢測**（導讀下方）：幾題小測驗，答錯會有解釋。

> 建議的節奏：讀一步 → 照著右上角的提示玩一下 → 再讀下一步。`,
    simHint: '右邊是一個單擺。先別急著讀公式，試著拖曳擺錘，再按「播放」。',
  },
  {
    title: '顏色與座標慣例',
    md: `整個實驗室共用同一套慣例，之後每一課都會用到：

- **座標軸顏色**：$\\hat{x}$ 紅、$\\hat{y}$ 綠、$\\hat{z}$ 藍，右手座標系。
- **3D 場景 $z$ 軸朝上**（機器人學的習慣），地面格線在 $xy$ 平面。
- **角度一律用弧度**計算，介面上會同時顯示度數方便閱讀。
- **座標系用大括號標記**：$\\{s\\}$ 是固定的空間座標系（space frame），$\\{b\\}$ 是跟著物體動的物體座標系（body frame）。
- 向量與矩陣以 $\\R^n$、$\\SO(3)$、$\\SE(3)$ 等符號表示，與教材一致。`,
    simHint: '把視線移到 3D 場景：固定的 {s} 與跟著擺錘轉動的 {b}，三個顏色各代表哪個軸？',
  },
  {
    title: '操作 2D 模擬：拖曳與滑桿',
    md: `2D 畫布裡圓形的**把手**都可以直接拖曳；本課的把手是擺錘。拖曳時模擬會暫停，放開後再按播放。

滑桿控制模型參數（擺長 $L$、阻尼 $b$）。單擺的運動方程式是

$$\\ddot\\theta = -\\frac{g}{L}\\sin\\theta - b\\,\\dot\\theta ,$$

拉長 $L$ 會讓週期變長；把阻尼調到 0，擺動就不會停下來。`,
    simHint: '把擺長從 0.5 m 拉到 2 m，看看週期怎麼變；再把阻尼調成 0。',
  },
  {
    title: '操作 3D 場景',
    md: `3D 場景使用**軌道攝影機**：

- 左鍵拖曳：旋轉視角
- 滾輪：縮放
- 右鍵（或雙指）拖曳：平移

試著把視角轉到從 $+z$ 往下看，你會看到 $\\{b\\}$ 的 $\\hat{x}_b$ 軸正指向擺桿方向。這就是「物體座標系」的意思——它黏在物體上。`,
    simHint: '轉動 3D 視角到俯視（從 +z 往下看），比對 {b} 的紅色 x 軸與擺桿方向。',
  },
  {
    title: '讀圖表與播放控制',
    md: `許多課會用**時間序列圖**呈現模擬狀態。本課畫的是 $\\theta(t)$：

- 有阻尼時曲線的振幅逐漸衰減；
- 大角度時擺動不是完美的弦波——因為 $\\sin\\theta \\neq \\theta$。

播放列可以暫停、重設、調整播放速度。放慢到 0.25× 可以更清楚看到細節。`,
    simHint: '把初始角度拉到接近 180°，播放後觀察 θ(t) 的形狀與小角度時有何不同。',
  },
  {
    title: '對照教材與學習路徑',
    md: `每課標題旁的 📖 標籤是對應的教材章節（例如 §3.2.3）。本站的文字是獨立撰寫的導讀，**不是教材摘錄**，建議搭配原書閱讀與做習題。

建議路徑：

1. **Ch 2–3** 建立語言：組態空間、旋轉與剛體運動。
2. **Ch 4–6** 運動學：正向、速度、逆向。
3. **Ch 8–9、11** 動力學、軌跡與控制——這是「機器人控制理論」的主線。
4. **Ch 7、10、12、13** 依興趣補充：閉鏈、規劃、抓取、移動機器人。

準備好了就往下做自我檢測，然後前往第 2 章。`,
  },
];

function mount(host: HTMLElement, ctx: LessonContext): LessonHandle {
  // ---- state
  const g = 9.81;
  let L = 1.0, b = 0.3;
  let theta = Math.PI / 3, omega = 0;
  let t = 0;

  // ---- layout
  simTitle(host, '單擺（Simple pendulum）', '導覽範例');
  const top = el('div', { class: 'mr-split' });
  host.appendChild(top);
  const left = el('div'); const right = el('div');
  top.append(left, right);
  const cv = new Canvas2D(left, { aspect: 1, world: { xmin: -2.3, xmax: 2.3, ymin: -2.4, ymax: 0.6 }, overlay: true });
  const scene = new Scene3D(right, { aspect: 1, grid: 4, axes: 0.6, cameraPos: [2.4, -3.2, 1.6], target: [0, 0, -0.6] });
  const panel = controlPanel(host);
  const sL = slider(panel, { label: '擺長 L', min: 0.3, max: 2, step: 0.05, value: L, unit: 'm', onInput: (v) => { L = v; redraw(); } });
  const sB = slider(panel, { label: '阻尼 b', min: 0, max: 2, step: 0.05, value: b, unit: '1/s', onInput: (v) => { b = v; } });
  const sTh = angleSlider(panel, { label: '初始角度 θ₀', value: theta, onInput: (v) => { theta = v; omega = 0; reset(false); } });
  const r = row(panel);
  const pb = playback(r, {
    onPlay: () => loop.start(), onPause: () => loop.stop(),
    onReset: () => { reset(true); },
    onSpeed: (s) => { loop.speed = s; },
  });
  const rTheta = readout(r, { label: 'θ =', unit: 'rad' });
  const rE = readout(r, { label: '能量 E =', unit: 'J/kg' });
  const plot = new Plot2D(host, { title: 'θ(t)', xLabel: 't (s)', yLabel: 'θ (rad)', series: [{ name: 'θ' }], xWindow: 12, height: 180, yRange: [-Math.PI, Math.PI] });
  note(host, '拖曳擺錘可設定初始角度；拖曳時模擬會暫停。能量 E = ½L²θ̇² + gL(1 − cos θ)（單位質量）。');

  // ---- 3D objects: {b} frame rotates about z by theta; rod along x_b, hanging down means theta measured from -z... keep it simple:
  // we draw the pendulum in the x–z plane of {s}: rod from origin along direction (sin θ, 0, −cos θ).
  const frameB = scene.addFrame({ size: 0.45, label: '{b}' });
  const rod = scene.addLink({ radius: 0.025 });
  const bob = scene.addSphere(0.11, '--accent');
  const pivot = scene.addSphere(0.05, '--fg-muted');
  pivot.position.set(0, 0, 0);

  // ---- 2D drawing
  const bobPos = (): Pt => [L * Math.sin(theta), -L * Math.cos(theta)];
  cv.addHandle({
    get: bobPos,
    set: (p) => { theta = Math.atan2(p[0], -p[1]); omega = 0; sTh.set(theta, { silent: true }); redraw(); },
    onStart: () => { wasPlaying = loop.running; loop.stop(); pb.setPlaying(false, { silent: true }); },
    onEnd: () => { reset(false); if (wasPlaying) { loop.start(); pb.setPlaying(true, { silent: true }); } },
    label: '拖曳我',
    radius: 11,
  });
  let wasPlaying = false;
  cv.draw = (c) => {
    c.grid(0.5);
    c.frame({ x: 0, y: 0, theta: 0 }, 0.4, { labels: ['x', 'y'] });
    // arc showing θ from the downward vertical
    c.line([0, 0], [0, -L * 0.6], { color: cssVar('--fg-muted'), dash: [4, 4], width: 1 });
    const a0 = -Math.PI / 2, a1 = -Math.PI / 2 + theta;
    c.arc([0, 0], 0.45, Math.min(a0, a1), Math.max(a0, a1), { color: cssVar('--warn'), width: 2 });
    c.text([0.6 * Math.sin(theta / 2), -0.6 * Math.cos(theta / 2)], 'θ', { color: cssVar('--warn') });
    c.link([0, 0], bobPos(), { color: cssVar('--fg-muted'), width: 6 });
    c.disk(bobPos(), 0.12, { fill: cssVar('--accent') });
    c.overlay.textContent = `θ = ${fmt(theta, 3)} rad (${fmt((theta * 180) / Math.PI, 1)}°)   θ̇ = ${fmt(omega, 3)} rad/s`;
  };

  function redraw(): void {
    cv.requestRender();
    const sx = Math.sin(theta), cz = -Math.cos(theta);
    // {b}: x_b along the rod, y_b = y_s, z_b = x_b × y_b  → rotation about y by angle φ where rod = (sinθ, 0, −cosθ)
    const T = [
      [sx, 0, cz, 0],
      [0, 1, 0, 0],
      [-cz, 0, sx, 0],
      [0, 0, 0, 1],
    ];
    frameB.setTransform(T);
    rod.setEndpoints([0, 0, 0], [L * sx, 0, L * cz]);
    bob.position.set(L * sx, 0, L * cz);
    rTheta.set(theta);
    rE.set(0.5 * L * L * omega * omega + g * L * (1 - Math.cos(theta)), 3);
    pb.setTime(t);
  }

  function reset(full: boolean): void {
    if (full) { theta = sTh.value; }
    omega = 0; t = 0; loop.reset();
    plot.clear();
    plot.push(0, [theta]);
    redraw();
  }

  const loop = createLoop((dt) => {
    substep(dt, 1 / 240, (h) => {
      // semi-implicit Euler keeps the undamped pendulum energy bounded
      omega += (-(g / L) * Math.sin(theta) - b * omega) * h;
      theta += omega * h;
      t += h;
    });
    if (theta > Math.PI) theta -= 2 * Math.PI; else if (theta < -Math.PI) theta += 2 * Math.PI;
    plot.push(t, [theta]);
    redraw();
  });

  reset(false);
  void ctx;

  // ---- per-step focus
  const focusTargets: (HTMLElement | null)[] = [null, scene.el, cv.el, scene.el, plot.el, null];
  const stepControls: (HTMLElement | null)[] = [null, null, sL.el, null, sB.el, null];
  let focused: HTMLElement[] = [];
  const setFocus = (i: number) => {
    focused.forEach((f) => f.classList.remove('mr-focus'));
    focused = [focusTargets[i], stepControls[i]].filter((x): x is HTMLElement => !!x);
    focused.forEach((f) => f.classList.add('mr-focus'));
  };
  setFocus(ctx.currentStep);

  return {
    onStep: setFocus,
    destroy() { loop.stop(); cv.destroy(); plot.destroy(); scene.destroy(); },
  };
}

const lesson: Lesson = {
  meta,
  steps,
  quiz: [
    {
      question: '在本站的 3D 場景中，藍色的座標軸代表哪個方向？',
      options: ['$\\hat{x}$', '$\\hat{y}$', '$\\hat{z}$（朝上）', '重力方向'],
      answer: 2,
      explanation: '慣例是 x 紅、y 綠、z 藍，且 z 軸朝上。重力方向是 $-\\hat{z}$，但軸本身是 $+\\hat{z}$。',
    },
    {
      question: '把阻尼 $b$ 調成 0 之後，單擺的 $\\theta(t)$ 會怎樣？',
      options: ['振幅逐漸衰減到 0', '振幅維持不變、持續擺動', '擺錘停在最高點', '週期變成 0'],
      answer: 1,
      explanation: '沒有阻尼就沒有能量耗散，能量守恆，所以振幅不會衰減（模擬用的積分法會讓能量維持在固定值附近）。',
    },
    {
      question: '符號 $\\{b\\}$ 在本站代表什麼？',
      options: ['固定不動的空間座標系', '黏在運動物體上的物體座標系', '一個 3×3 矩陣', '旋轉的角度'],
      answer: 1,
      explanation: '$\\{s\\}$ 是 space frame（固定），$\\{b\\}$ 是 body frame（跟著物體動）。這對後面理解「同一個運動在不同座標系的表示」非常重要。',
    },
  ],
  mount,
};

export default lesson;
