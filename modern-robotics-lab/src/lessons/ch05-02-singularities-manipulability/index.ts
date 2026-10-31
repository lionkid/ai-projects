import type { Lesson, LessonContext, LessonHandle, LessonStep, QuizQuestion } from '@/types';
import {
  cssVar, fmt, controlPanel, row, angleSlider, select, toggle, button, readout, matrixView, simTitle, note,
  legend, split, Canvas2D, Plot2D, presetChips, torqueBars, RasterLayer, viridisLike, createLoop, drawPlanarArm,
  type Pt, type PresetChipsHandle, type TextStyle,
} from '@/ui';
import { planarFK, type Mat } from '@/mr';
import { meta } from './meta';
import {
  L2R, L3R, sigmaInfo, tipJacobian, solveTip, elbowBranch, virtualLink, reachAnnulus, sweepTheta2,
  sigmaMinProfile, sampleProfile, radialRates, radialPathConfig, radialWarp, tipVelocity, wrapAngle, type SigmaInfo,
} from './manip';

const D = Math.PI / 180;
/** World units per m/s for the velocity ellipse, the Jacobian columns and J θ̇. */
const VEL_SCALE = 0.4;
/** World units per N for the force-ellipse preview (torque budget |τ| ≤ 1 N·m). */
const FORCE_SCALE = 0.3;
const FORCE_CAP = 3.5;
const RADIAL_SPEED = 0.1; // m/s
/** Slow-motion factor of the radial run while the arm is still far from the singularity. */
const RADIAL_PLAYBACK = 0.7;
/**
 * Closer in, the run is slowed further so that θ2 shrinks by a fixed factor per real second
 * (e^-1.2 ≈ 0.3). At constant tip speed the joint rates blow up in the last few millimetres; in
 * plain slow motion that whole phase would flash by in about 0.1 s.
 */
const RADIAL_LOG_RATE = 1.2;
const QDOT_LIMIT = 2; // rad/s, nominal joint speed limit drawn on the bars
const HEAT_RANGE: [number, number] = [0, 0.9];

const steps: LessonStep[] = [
  {
    title: '伸直的手臂推不動自己',
    md: `末端的彩色箭頭是 Jacobian 的欄位：$J_1$ 是「只轉關節 1、速度 1 rad/s」時末端得到的速度，$J_2$ 則只轉關節 2。任何末端速度都是它們的組合：

$$v = J_v(\\theta)\\,\\dot\\theta = J_1\\dot\\theta_1 + J_2\\dot\\theta_2 .$$

手臂越接近伸直，兩支箭頭越垂直於手臂、也越平行；$\\theta_2 = 0$ 時完全平行，沿手臂方向（徑向）的速度怎麼組合都做不出來。這時 $J_v$ 的秩從 2 掉到 1——秩小於它可能的最大值，就稱為**運動學奇異（kinematic singularity）**。`,
    simHint: '依序按「θ₂ = 20°」「θ₂ = 5°」「θ₂ = 0°（伸直）」，看 J₁、J₂ 何時變平行、rank 讀數何時掉到 1；也可以直接拖曳末端的藍色把手。',
  },
  {
    title: '奇異是構型的性質，不是位置的性質',
    md: `切到平面 3R（$L = 1, 0.8, 0.6$ m）。構型 A 把第三節折回，所有關節和末端排成一直線；構型 B 是彎曲的姿勢，末端卻停在**同一點**。A 的三個欄位全部平行、$J_v$ 降秩；B 則是滿秩，速度橢圓很飽滿。

可見奇異屬於關節空間中的點 $\\theta$，而不是工作空間裡的某個位置。對 3R 的末端位置來說，判準是所有關節與末端共線：

$$\\operatorname{rank} J_v(\\theta) = 1 \\iff \\theta_2,\\ \\theta_3 \\in \\{0, \\pi\\}.$$`,
    simHint: '比較「構型 A」與「構型 B」；再按「鎖住末端，拖 θ₃」，把 θ₃ 從 90° 拖到 180°：末端不動，橢圓卻逐漸壓扁成線段。',
  },
  {
    title: '可操作度橢球',
    md: `讓關節速度走遍單位圓 $\\|\\dot\\theta\\| = 1$，末端速度 $v = J_v\\dot\\theta$ 就描出一個橢圓，稱為**可操作度橢球（manipulability ellipsoid）**。$J_v$ 滿秩時可寫成

$$v^{\\mathsf T}\\,(J_vJ_v^{\\mathsf T})^{-1}\\,v \\le 1 .$$

主軸方向是 $J_vJ_v^{\\mathsf T}$ 的特徵向量，半軸長是特徵值的平方根 $\\sqrt{\\lambda_i}$，也就是 $J_v$ 的奇異值 $\\sigma_i$：長軸方向好動、短軸方向難動。這裡只用線速度的 $2\\times n$ 子 Jacobian，避免把 m/s 與 rad/s 混進同一個橢圓。`,
    simHint: '「掃描單位關節速度」已開啟：右下角單位圓上的紫點是 $\\dot\\theta$，末端的紫色箭頭 $J_v\\dot\\theta$ 沿橢圓繞圈。也可以拖那個點；打開「J 欄向量」會發現 J₁、J₂ 的尖端正好在橢圓上。',
  },
  {
    title: '量化：σ 與條件數',
    md: `兩個常用指標描述橢圓「多圓」與「多大」：

$$\\mu_1 = \\frac{\\sigma_{\\max}}{\\sigma_{\\min}} \\ge 1, \\qquad \\mu_3 = \\sqrt{\\det(J_vJ_v^{\\mathsf T})} = \\sigma_1\\sigma_2 .$$

$\\mu_1$ 越接近 1 越各向同性（$J_vJ_v^{\\mathsf T}$ 的條件數 $\\mu_2 = \\mu_1^2$）；$\\mu_3$ 正比於橢圓面積。下方曲線把 $\\theta_2$ 從 $-180^\\circ$ 掃到 $180^\\circ$：$\\sigma_{\\min}$ 只在 $0^\\circ$ 與 $\\pm180^\\circ$ 歸零，但 $\\mu_1$ 在 $\\theta_2 = 10^\\circ$ 就已接近 30。接近奇異是連續的，數值上的麻煩在抵達之前就開始了。`,
    simHint: '把 θ₂ 慢慢滑向 0°，盯著右圖 μ₁ 的標記點；再按「3R：θ₃ = 90°」，看 σ_min 曲線還會不會碰到 0。',
  },
  {
    title: '整個工作空間的地圖',
    md: `把每個末端位置的 $\\sigma_{\\min}$ 塗成顏色（亮＝好動，暗＝接近奇異），就得到工作空間的可操作度地圖。2R 的地圖是同心圓：$\\sigma$ 只和 $\\theta_2$ 有關，而 $\\theta_2$ 由末端到基座的距離決定，兩個肘部分支的值也相同，所以最暗的地方是外邊界（伸直，$\\theta_2 = 0$）與基座附近（折回，$\\theta_2 = \\pm180^\\circ$）。

3R 有冗餘，必須先選一個構型族：這裡固定 $\\theta_3$ 與分支，再解另外兩軸。改變 $\\theta_3$，暗環會移到 3R 工作空間的內部（$\\theta_3 = 180^\\circ$ 時在 $r = 0.8$ 與 $1.2$ m），甚至完全消失——地圖的明暗跟著構型走，不只跟著位置走。`,
    simHint: '熱圖已開啟。切換「3R：θ₃ = 180°／90°／0°」看暗環的位置；把末端拖向暗環（它就在虛線圓上，也就是這個構型族到得了的邊緣），同時注意 μ₁ 讀數與紅色虛線。',
  },
  {
    title: '接近奇異的代價與下一課',
    md: `要讓末端以固定速率沿徑向移動，所需關節速度為

$$\\dot\\theta = J_v^{+}\\,v \\qquad (\\text{2R 時 } J_v^{+} = J_v^{-1}),$$

若 $v$ 沿短軸方向，$\\|\\dot\\theta\\| = \\|v\\|/\\sigma_{\\min}$。2R 越接近伸直，$\\dot\\theta_2$ 就越大：0.1 m/s 的徑向速度在 $\\theta_2 = 3^\\circ$ 時就要將近 4 rad/s，遠超過一般關節的速度上限。

奇異並非一無是處。下一課的靜力學 $\\tau = J^{\\mathsf T}\\mathcal F$ 會說明：同一個伸直姿勢沿連桿方向承受負載時，幾乎不需要關節力矩——先打開力橢圓預覽看看它朝哪裡變長。`,
    simHint: '按「▶ 2R 徑向伸直」：末端以 0.1 m/s 往外移（慢動作，越接近伸直放得越慢），看關節速度條何時越過 ±2 rad/s 的紅線。最後打開「預覽力橢圓」，比較它和速度橢圓的長軸方向。',
  },
];

const quiz: QuizQuestion[] = [
  {
    question: '2R 手臂（$L_1 = L_2 = 1$ m）要讓末端以 0.1 m/s 沿徑向移動。當 $\\theta_2$ 從 $30^\\circ$ 變成 $3^\\circ$，所需的 $|\\dot\\theta_2|$ 大約變成原來的幾倍？',
    options: ['差不多不變，因為末端速度沒變', '約 3 倍', '約 10 倍', '變成無限大'],
    answer: 2,
    explanation: '徑向距離只和 $\\theta_2$ 有關：$\\dot r = -\\dfrac{L_1L_2\\sin\\theta_2}{r}\\,\\dot\\theta_2$，所以 $|\\dot\\theta_2| = r|\\dot r|/(L_1L_2|\\sin\\theta_2|)$。$\\sin 30^\\circ/\\sin 3^\\circ \\approx 9.6$，而 $r$ 只從 1.93 m 變到 2.00 m，合起來約 10 倍（0.39 → 3.8 rad/s）。只有在 $\\theta_2 = 0$ 正好奇異時才真正發散。',
  },
  {
    question: '平面 3R（$L = 1, 0.8, 0.6$ m）。下列哪一個構型的末端線速度 Jacobian $J_v$ 降秩？',
    options: ['$\\theta = (0^\\circ, 0^\\circ, 180^\\circ)$', '$\\theta = (0^\\circ, 90^\\circ, -90^\\circ)$', '$\\theta = (0^\\circ, 45^\\circ, 0^\\circ)$', '所有末端距基座 1.2 m 的構型'],
    answer: 0,
    explanation: '每個欄位都垂直於「該關節到末端」的連線，只有所有關節與末端共線（$\\theta_2, \\theta_3 \\in \\{0, \\pi\\}$）時三欄才全部平行。$(0, 0, 180^\\circ)$ 正是如此，末端落在 1.2 m 處；但同一點也能用彎曲、滿秩的姿勢到達，所以「距離 1.2 m」本身並不是奇異的條件。',
  },
  {
    question: '6R 手臂的腕部三軸交於一點。若腕部第 4 軸與第 6 軸轉到共線，下列描述何者正確？',
    options: [
      '末端位置從此無法改變',
      '兩個關節產生完全相同的末端運動，$J$ 降秩，末端在這個構型下失去一個瞬時轉動方向',
      '這種情況只會發生在工作空間的邊界',
      '只要把關節速度調大，就能補回失去的方向',
    ],
    answer: 1,
    explanation: '兩個旋轉軸共線時，它們的螺旋軸（screw axis）相同（軸向相反時只差一個負號），Jacobian 有兩個線性相依的欄位，秩至少掉 1。再大的關節速度也只能在剩下的方向上組合，補不回失去的那個轉動方向。這種腕部奇異可以出現在工作空間內部任何地方——和 3R 的例子一樣，它是構型的性質。',
  },
  {
    question: '某構型的可操作度橢圓長半軸為 1.5、短半軸為 0.15（單位：每 1 rad/s 關節速度範數對應的 m/s）。哪個解讀正確？',
    options: [
      '$\\mu_1 = 0.1$；此構型幾乎各向同性',
      '$\\mu_3 = 1.65$，代表橢圓的周長',
      '沿長軸方向最難移動',
      '$\\mu_1 = 10$；沿短軸方向產生 0.1 m/s 需要約 0.67 rad/s 的 $\\|\\dot\\theta\\|$',
    ],
    answer: 3,
    explanation: '$\\mu_1 = \\sigma_{\\max}/\\sigma_{\\min} = 1.5/0.15 = 10$，離各向同性（$\\mu_1 = 1$）很遠。沿短軸 $\\|\\dot\\theta\\| = \\|v\\|/\\sigma_{\\min} = 0.1/0.15 \\approx 0.67$ rad/s，沿長軸只要約 0.067 rad/s。$\\mu_3 = \\sigma_1\\sigma_2 = 0.225$，正比於面積而不是周長。',
  },
];

type ModelKey = '2R' | '3R';

function mount(host: HTMLElement, ctx: LessonContext): LessonHandle {
  // ---------------------------------------------------------------- state
  let model: ModelKey = '2R';
  let L = L2R;
  let th: number[] = [25 * D, 20 * D];
  let branch: 1 | -1 = 1;
  let step = -1;
  const show = { ellipse: false, cols: true, heat: false, sweep: false, force: false };
  let sweepPaused = false;
  let psi = 0.6; // parameter of the unit joint velocity being swept
  let lockTip = false;
  let lockTarget: Pt | null = null;
  let lockClamped = false;
  let ghost: number[] | null = null;
  let radial: { tip0: Pt; s: number; sMax: number; th0: number[]; br: 1 | -1; t: number; warp: number } | null = null;
  let radialMsg = '';
  const labels = new LabelPlacer();

  // derived quantities (refreshed by recompute)
  let J: Mat = tipJacobian(L, th);
  let S: SigmaInfo = sigmaInfo(J);
  let pts: Pt[] = [];

  const tipOf = (q: number[]): Pt => { const t = planarFK(L, q).tip; return [t[0], t[1]]; };

  // ---------------------------------------------------------------- layout
  simTitle(host, '速度橢圓與奇異點', '平面 2R / 3R');
  // Fixed height keeps the pane short on wide screens; the world favours the upper half-plane
  // (where all presets live) so the arm is drawn large. Sliders still reach every configuration.
  const cv = new Canvas2D(host, { height: 340, world: { xmin: -2.5, xmax: 2.5, ymin: -1.3, ymax: 2.5 }, overlay: true });
  const heatLegend = legend(host, [
    { color: rgbCss(viridisLike(0)), label: 'σ_min ≈ 0（奇異）' },
    { color: rgbCss(viridisLike(0.5)), label: 'σ_min ≈ 0.45' },
    { color: rgbCss(viridisLike(1)), label: 'σ_min ≥ 0.9（好動）' },
    { color: 'transparent', label: '3R：固定目前的 θ₃ 與分支' },
  ]);

  const rp = controlPanel(host, { compact: true });
  // room for the focus outline of the readout row (the plots below would otherwise paint over its lower edge)
  rp.style.marginBottom = '0.45rem';
  const rrow = row(rp);
  const rRank = readout(rrow, { label: 'rank J_v' });
  const rSmax = readout(rrow, { label: 'σ_max' });
  const rSmin = readout(rrow, { label: 'σ_min' });
  const rMu1 = readout(rrow, { label: 'μ₁' });
  const rMu3 = readout(rrow, { label: 'μ₃' });
  const mJ = matrixView(rrow, { label: 'J_v =', digits: 2 });

  const [pl, pr] = split(host);
  const plotsWrap = pl.parentElement as HTMLElement;
  const plotS = new Plot2D(pl, {
    title: 'σ 隨 θ₂ 變化（其餘關節固定）', xLabel: 'θ₂ (°)', height: 130, xRange: [-180, 180], yRange: [0, 3],
    series: [{ name: 'σ_max' }, { name: 'σ_min' }, { name: 'μ₃ = σ₁σ₂', dash: [5, 4] }],
  });
  // The μ₁ = 10 reference line is explained in the title: a label on the line itself collides with
  // the live marker label whenever the marker sits near the right edge.
  const plotM = new Plot2D(pr, {
    title: '條件數 μ₁ = σ_max/σ_min（虛線 = 10）', xLabel: 'θ₂ (°)', height: 130, xRange: [-180, 180], yRange: [0, 30],
    series: [{ name: 'μ₁', color: '--series-5' }],
  });

  const panel = controlPanel(host, { compact: true });
  const chipsRow = row(panel);
  const optRow = row(panel);
  const selModel = select<ModelKey>(optRow, {
    value: '2R',
    options: [{ value: '2R', label: '平面 2R（1, 1 m）' }, { value: '3R', label: '平面 3R（1, 0.8, 0.6 m）' }],
    onChange: (v) => { chipGroups[step]?.clear(); interrupt(); setModel(v); },
  });
  const selBranch = select<'+' | '-'>(optRow, {
    value: '+',
    options: [{ value: '+', label: '肘部分支 +（θ₂′ ≥ 0）' }, { value: '-', label: '肘部分支 −（θ₂′ < 0）' }],
    onChange: (v) => { chipGroups[step]?.clear(); flipBranch(v === '+' ? 1 : -1); },
  });
  selBranch.el.title = 'θ₂′ 是把第 2、3 節視為一根虛擬連桿時的肘角；2R 時 θ₂′ = θ₂。切換分支會把手肘翻到另一側，末端不動。';
  const tEllipse = toggle(optRow, { label: '速度橢圓', value: show.ellipse, onChange: (v) => { show.ellipse = v; cv.requestRender(); } });
  const tCols = toggle(optRow, { label: 'J 欄向量', value: show.cols, onChange: (v) => { show.cols = v; cv.requestRender(); } });
  const tHeat = toggle(optRow, { label: 'σ_min 熱圖', value: show.heat, onChange: (v) => { show.heat = v; heatLegend.style.display = v ? '' : 'none'; cv.requestRender(); } });
  const tSweep = toggle(optRow, { label: '掃描單位關節速度', value: show.sweep, onChange: (v) => setSweep(v) });
  const tLock = toggle(optRow, { label: '鎖住末端（自運動）', value: false, onChange: (v) => setLock(v) });
  const tForce = toggle(optRow, { label: '預覽力橢圓', value: show.force, onChange: (v) => { show.force = v; cv.requestRender(); } });
  const s1 = angleSlider(panel, { label: 'θ₁', value: th[0], onInput: (v) => setJoint(0, v) });
  // max slightly above π so a folded joint (exactly π) shows as 180° instead of snapping to 179°
  const s2 = angleSlider(panel, { label: 'θ₂', value: th[1], max: Math.PI + 1e-6, onInput: (v) => setJoint(1, v) });
  const s3 = angleSlider(panel, { label: 'θ₃', value: 60 * D, max: Math.PI + 1e-6, onInput: (v) => setJoint(2, v) });

  // Step 6: required joint rates for a radial tip speed
  const ratePanel = controlPanel(host, { compact: true });
  const rateRow = row(ratePanel);
  const runBtn = button(rateRow, { label: '▶ 2R 徑向伸直 0.1 m/s', small: true, variant: 'primary', onClick: () => (radial ? stopRadial() : startRadial()) });
  // Joint rates are written dθ/dt in HTML/canvas text: the combining dot of θ̇ renders as a stray tick there.
  const rNorm = readout(rateRow, { label: '所需 ‖dθ/dt‖ =' });
  const rRate = readout(rateRow, { label: '' });
  const bars2 = torqueBars(ratePanel, { labels: ['dθ₁/dt', 'dθ₂/dt'], max: 8, limit: QDOT_LIMIT, unit: 'rad/s', title: '徑向 0.1 m/s 所需關節速度 J⁺v（紅線 = ±2 rad/s 上限）' });
  const bars3 = torqueBars(ratePanel, { labels: ['dθ₁/dt', 'dθ₂/dt', 'dθ₃/dt'], max: 8, limit: QDOT_LIMIT, unit: 'rad/s', title: '徑向 0.1 m/s 所需關節速度：最小範數 J⁺v（紅線 = ±2 rad/s 上限）' });
  bars2.el.style.gridColumn = '1 / -1';
  bars3.el.style.gridColumn = '1 / -1';
  // wider value column so '-22.36 rad/s ⚠' near the singularity does not wrap (the bars would jump)
  for (const b of [bars2, bars3]) b.el.querySelectorAll<HTMLElement>('.mr-bar-row').forEach((r) => { r.style.gridTemplateColumns = 'minmax(2.5em, auto) 1fr 9.5em'; });

  note(host, '比例：速度 1 m/s ↔ 0.4 m，力橢圓 1 N ↔ 0.3 m（‖τ‖ ≤ 1 N·m）。σ_min/σ_max < 10⁻⁶ 判為降秩。3R 拖曳末端時固定 θ₃。');

  // ---------------------------------------------------------------- heat map (cached; repainted only when its inputs change)
  const heat = new RasterLayer({ xmin: -2.5, xmax: 2.5, ymin: -2.5, ymax: 2.5 }, 240, 240);
  let heatKey = '';
  function ensureHeat(): void {
    const key = model === '2R' ? '2R' : `3R|${th[2].toFixed(4)}|${branch}`;
    if (key === heatKey) return;
    heatKey = key;
    const prof = sigmaMinProfile(L, th, branch, 500);
    heat.paint((x, y) => sampleProfile(prof, Math.hypot(x, y)), { colormap: viridisLike, range: HEAT_RANGE });
  }

  // ---------------------------------------------------------------- σ sweep plots (recomputed when θ3 / model changes)
  let sweepKey = '';
  function ensureSweep(): void {
    const key = model === '2R' ? '2R' : `3R|${th[2].toFixed(5)}`;
    if (key === sweepKey) return;
    sweepKey = key;
    const sw = sweepTheta2(L, th, 361);
    plotS.setXY(0, sw.deg, sw.smax);
    plotS.setXY(1, sw.deg, sw.smin);
    plotS.setXY(2, sw.deg, sw.mu3);
    plotM.setXY(0, sw.deg, sw.mu1);
  }

  // ---------------------------------------------------------------- core update
  function recompute(): void {
    J = tipJacobian(L, th);
    S = sigmaInfo(J);
    pts = planarFK(L, th).joints.map((p) => [p[0], p[1]] as Pt);
    // The branch follows the configuration, except exactly on the boundary (θ2′ = 0 or π) where it is ambiguous.
    const { beta } = virtualLink(L, th);
    if (Math.abs(Math.sin(th[1] + beta)) > 1e-6) branch = elbowBranch(L, th);
    selBranch.set(branch > 0 ? '+' : '-', { silent: true });
  }

  function update(): void {
    recompute();
    rRank.set(S.rank === 2 ? '2（滿秩）' : `${S.rank}（降秩 ⚠）`);
    rSmax.set(S.smax, 3);
    rSmin.set(S.smin, 3);
    rMu1.set(Number.isFinite(S.mu1) ? fmt(S.mu1, 2) : '∞');
    rMu3.set(S.mu3, 3);
    if (step <= 1) mJ.set(J, { digits: 2 });
    ensureSweep();
    const t2d = wrapAngle(th[1]) / D;
    // colours as CSS variable names so the plots follow a theme switch
    plotS.setMarkers([{ x: t2d }, { x: t2d, y: S.smax, color: '--series-1' }, { x: t2d, y: S.smin, color: '--series-2' }]);
    plotM.setMarkers([
      { y: 10, dash: [2, 4] },
      { x: t2d },
      { x: t2d, y: Math.min(S.mu1, 29.4), color: '--series-5', label: Number.isFinite(S.mu1) ? `μ₁ = ${fmt(S.mu1, 1)}` : 'μ₁ = ∞' },
    ]);
    s1.set(th[0], { silent: true });
    s2.set(th[1], { silent: true });
    if (th.length > 2) s3.set(th[2], { silent: true });
    if (step === 5) updateRates();
    cv.requestRender();
  }

  function updateRates(): void {
    const rr = radialRates(L, th, RADIAL_SPEED);
    const bars = model === '3R' ? bars3 : bars2;
    if (rr.qdot) {
      bars.set(rr.qdot);
      rNorm.set(`${fmt(rr.norm, 2)} rad/s`);
      const over = Math.max(...rr.qdot.map(Math.abs)) > QDOT_LIMIT;
      const run = radial ? `t = ${fmt(radial.t, 2)} s（慢動作 ×${fmt(radial.warp, radial.warp < 0.1 ? 3 : 2)}）　` : '';
      rRate.set(radialMsg || run + (over ? '⚠ 超過關節速度上限' : '在上限之內'));
    } else {
      bars.set(new Array(L.length).fill(0));
      rNorm.set('∞');
      rRate.set('J 降秩：向外的徑向速度根本做不出來');
    }
  }

  // ---------------------------------------------------------------- actions
  function setConfig(q: number[]): void {
    th = q.map(wrapAngle);
    lockClamped = false;
    if (lockTip) lockTarget = tipOf(th);
    update();
  }

  function setModel(k: ModelKey): void {
    if (k === model) return;
    model = k;
    L = k === '2R' ? L2R : L3R;
    th = k === '3R' ? [th[0], th[1], 60 * D] : [th[0], th[1]];
    ghost = null;
    if (k === '2R') setLock(false);
    selModel.set(k, { silent: true });
    s3.el.style.display = k === '3R' ? '' : 'none';
    tLock.el.style.display = k === '3R' ? '' : 'none';
    bars2.el.style.display = k === '2R' ? '' : 'none';
    bars3.el.style.display = k === '3R' ? '' : 'none';
    heatKey = '';
    update();
  }

  /** The learner changed the configuration by hand: end a radial run and drop its final message. */
  function interrupt(): void {
    stopRadial();
    radialMsg = '';
  }

  function setJoint(i: number, v: number): void {
    if (i >= L.length) return; // hidden θ3 slider while the 2R is shown
    interrupt();
    chipGroups[step]?.clear();
    const q = th.slice();
    q[i] = v;
    if (lockTip && lockTarget && model === '3R' && i === 2) {
      // self-motion: re-solve θ1, θ2 so that the tip stays at the locked point
      const res = solveTip(L, q, lockTarget, branch);
      th = res.th;
      lockClamped = res.clamped;
    } else {
      th = q;
      ghost = null;
      lockClamped = false;
      if (lockTip) lockTarget = tipOf(th);
    }
    update();
  }

  function dragTip(p: Pt): void {
    interrupt();
    chipGroups[step]?.clear();
    ghost = null;
    th = solveTip(L, th, p, branch).th;
    lockClamped = false;
    if (lockTip) lockTarget = tipOf(th);
    update();
  }

  function flipBranch(b: 1 | -1): void {
    if (b === branch) return;
    interrupt();
    branch = b;
    ghost = null;
    th = solveTip(L, th, tipOf(th), b).th;
    update();
  }

  function setLock(v: boolean): void {
    lockTip = v && model === '3R';
    tLock.set(lockTip, { silent: true });
    lockTarget = lockTip ? tipOf(th) : null;
    lockClamped = false;
    cv.requestRender();
  }

  function setSweep(v: boolean): void {
    show.sweep = v;
    sweepPaused = false;
    tSweep.set(v, { silent: true });
    insetHandle.enabled = v;
    ensureLoop();
    cv.requestRender();
  }

  function startRadial(): void {
    chipGroups[step]?.clear();
    if (model !== '2R') setModel('2R');
    let tip0 = tipOf(th);
    // Too close to the boundary (or folded at the base): restart from a comfortable elbow angle.
    if (2 - Math.hypot(tip0[0], tip0[1]) < 0.01 || Math.hypot(tip0[0], tip0[1]) < 0.05) {
      setConfig([th[0], 60 * D * branch]);
      tip0 = tipOf(th);
    }
    const r0 = Math.hypot(tip0[0], tip0[1]);
    radial = { tip0, s: 0, sMax: 2 - 2e-5 - r0, th0: th.slice(), br: branch, t: 0, warp: RADIAL_PLAYBACK };
    radialMsg = '';
    runBtn.textContent = '⏸ 停止';
    ensureLoop();
  }

  function stopRadial(): void {
    if (!radial) return;
    radial = null;
    runBtn.textContent = '▶ 2R 徑向伸直 0.1 m/s';
    ensureLoop();
  }

  // ---------------------------------------------------------------- animation (θ̇ sweep and the radial run)
  const anim = createLoop((dt) => {
    if (show.sweep && !sweepPaused) psi = wrapAngle(psi + 1.3 * dt);
    if (radial) {
      // Slow motion that slows further near θ2 = 0 so the growth of the required rates stays watchable;
      // radial.t is the physical time of the 0.1 m/s motion.
      radial.warp = radialWarp(L2R, th, RADIAL_SPEED, RADIAL_PLAYBACK, RADIAL_LOG_RATE);
      const h = dt * radial.warp;
      radial.t += h;
      radial.s = Math.min(radial.sMax, radial.s + RADIAL_SPEED * h);
      th = radialPathConfig(L2R, radial.tip0, radial.s, radial.br, radial.th0);
      if (radial.s >= radial.sMax) {
        const t = radial.t;
        stopRadial();
        radialMsg = `停在 θ₂ ≈ ${fmt(Math.abs(wrapAngle(th[1])) / D, 2)}°（實際只過了 t = ${fmt(t, 1)} s）：再往外，所需關節速度會發散`;
      }
      update();
    } else {
      cv.requestRender();
    }
  });
  function ensureLoop(): void {
    const need = (show.sweep && !sweepPaused) || !!radial;
    if (need && !anim.running) anim.start();
    else if (!need && anim.running) anim.stop();
  }

  // ---------------------------------------------------------------- unit θ̇ shown in the joint-velocity inset
  function rowSpaceBasis(): [number[], number[]] {
    const n = L.length;
    const JT = (u: Pt) => Array.from({ length: n }, (_, k) => J[0][k] * u[0] + J[1][k] * u[1]);
    const unit = (a: number[]) => { const m = Math.hypot(...a) || 1; return a.map((x) => x / m); };
    const v1 = unit(JT(S.u1));
    let v2: number[];
    if (S.rank === 2) v2 = unit(JT(S.u2));
    else {
      // rank 1: any unit vector orthogonal to v1 lies in the null space and maps to zero tip velocity
      const e = Math.abs(v1[0]) < 0.9 ? [1, 0, 0].slice(0, n) : [0, 1, 0].slice(0, n);
      const d = e.reduce((s, x, k) => s + x * v1[k], 0);
      v2 = unit(e.map((x, k) => x - d * v1[k]));
    }
    return [v1, v2];
  }
  /** Unit joint velocity for sweep parameter a: the θ̇1–θ̇2 circle for 2R, a circle in the row space of J_v for 3R. */
  function sweepQdot(a: number): number[] {
    if (L.length === 2) return [Math.cos(a), Math.sin(a)];
    const [v1, v2] = rowSpaceBasis();
    return v1.map((x, k) => Math.cos(a) * x + Math.sin(a) * v2[k]);
  }
  function insetGeom(): { c: Pt; R: number; P: number } {
    const Rpx = Math.max(24, Math.min(48, cv.height * 0.14, cv.width * 0.085));
    const c = cv.toWorld(cv.width - Rpx - 26, cv.height - Rpx - 28);
    return { c, R: Rpx / cv.scale, P: (Rpx + 20) / cv.scale };
  }

  // ---------------------------------------------------------------- handles
  cv.addHandle({
    get: () => pts[pts.length - 1] ?? [0, 0],
    set: (p) => dragTip(p),
    radius: 9,
  });
  const insetHandle = cv.addHandle({
    get: () => { const g = insetGeom(); return [g.c[0] + g.R * Math.cos(psi), g.c[1] + g.R * Math.sin(psi)]; },
    set: (p) => {
      const g = insetGeom();
      psi = Math.atan2(p[1] - g.c[1], p[0] - g.c[0]);
      sweepPaused = true;
      ensureLoop();
      cv.requestRender();
    },
    radius: 7,
    color: '--series-5', // resolved at draw time, so it follows a theme switch
    enabled: false,
  });

  // ---------------------------------------------------------------- drawing
  const colColors = () => [cssVar('--series-6', '#0891b2'), cssVar('--series-4', '#d97706'), cssVar('--series-7', '#db2777')];
  cv.draw = (c) => {
    const fg = cssVar('--fg', '#222'), muted = cssVar('--fg-muted', '#777'), accent = cssVar('--accent', '#2563eb');
    const danger = cssVar('--danger', '#b91c1c'), purple = cssVar('--series-5', '#7c3aed'), rose = cssVar('--series-2', '#e11d48');
    const bgElev = cssVar('--bg-elev', '#fff');
    labels.reset();
    c.grid(0.5);
    if (show.heat) {
      ensureHeat();
      heat.draw(c, 0.8, true);
      const [ra, rb] = reachAnnulus(L, th);
      if (ra > 1e-3) c.circle([0, 0], ra, { color: accent, dash: [3, 4], width: 1 });
      c.circle([0, 0], rb, { color: accent, dash: [3, 4], width: 1 });
    }
    const reach = L.reduce((a, b) => a + b, 0);
    c.circle([0, 0], reach, { color: cssVar('--grid-strong', '#ccc'), dash: [4, 4], width: 1 });
    c.frame({ x: 0, y: 0, theta: 0 }, 0.35, { labels: 's', alpha: 0.75 });

    // reference configuration with the same tip (hidden once the locked self-motion has reached it)
    if (ghost && !ghost.every((g, i) => Math.abs(wrapAngle(g - th[i])) < 1e-3)) {
      drawPlanarArm(c, L, ghost, { alpha: 0.28 });
      const gS = sigmaInfo(tipJacobian(L, ghost));
      const gj = planarFK(L, ghost).joints;
      const gt = gj[gj.length - 1];
      c.ellipse([gt[0], gt[1]], gS.smax * VEL_SCALE, gS.smin * VEL_SCALE, Math.atan2(gS.u1[1], gS.u1[0]), { color: accent, width: 1.5, dash: [4, 4], alpha: 0.7 });
      labels.inside(c, [gj[1][0], gj[1][1]], '另一個構型（同一末端）', { color: muted, dy: 18, bg: bgElev });
    }
    drawPlanarArm(c, L, th, { eeRadius: 5 });
    const tip = pts[pts.length - 1];
    labels.reserve(c, tip, 12); // the tip handle
    if (show.sweep) { const g = insetGeom(); labels.reserve(c, g.c, g.P * c.scale); }

    // Rank deficient or badly conditioned: show the hard direction (minor axis) through the tip.
    if (S.rank < 2 || S.mu1 > 12) {
      const u = S.u2, len = 0.9;
      const out = u[0] * tip[0] + u[1] * tip[1] >= 0 ? 1 : -1; // label on the side away from the base
      c.line([tip[0] - u[0] * len, tip[1] - u[1] * len], [tip[0] + u[0] * len, tip[1] + u[1] * len], { color: danger, dash: [6, 5], width: 1.5 });
      labels.inside(c, [tip[0] + out * u[0] * (len + 0.1), tip[1] + out * u[1] * (len + 0.1)], S.rank < 2 ? '✗ 這個方向動不了' : '這個方向很難動', { color: danger, bg: bgElev });
      if (S.rank < 2) c.polyline(pts, { color: danger, width: 1, dash: [2, 3], alpha: 0.8 });
    }

    if (show.cols) {
      const cols = colColors();
      for (let k = 0; k < L.length; k++) {
        const jx = J[0][k], jy = J[1][k];
        const m = Math.hypot(jx, jy);
        if (m < 1e-9) continue;
        const e: Pt = [tip[0] + jx * VEL_SCALE, tip[1] + jy * VEL_SCALE];
        c.arrow(tip, e, { color: cols[k], width: 2.5 });
        // label beyond the head, nudged sideways (alternating) so parallel columns stay readable
        const ux = jx / m, uy = jy / m, side = k === 0 ? 0 : k === 1 ? 1 : -1;
        c.text(e, `J${'₁₂₃'[k]}`, { color: cols[k], dx: 12 * ux + side * 11 * uy, dy: -12 * uy + side * 11 * ux, font: 'bold 12px sans-serif' });
      }
    }

    if (show.ellipse) {
      const ang = Math.atan2(S.u1[1], S.u1[0]);
      const a = S.smax * VEL_SCALE, b = S.smin * VEL_SCALE;
      c.ellipse(tip, a, b, ang, { fill: accent, alpha: 0.12, width: 0 });
      c.ellipse(tip, a, b, ang, { color: accent, width: 2 });
      const e1: Pt = [tip[0] + S.u1[0] * a, tip[1] + S.u1[1] * a];
      const e2: Pt = [tip[0] + S.u2[0] * b, tip[1] + S.u2[1] * b];
      c.line(tip, e1, { color: accent, width: 1.2, dash: [3, 3] });
      c.line(tip, e2, { color: accent, width: 1.2, dash: [3, 3] });
      c.text(e1, 'σ₁', { color: accent, dx: 9 * S.u1[0], dy: -9 * S.u1[1] - 2 });
      if (b > 0.03) c.text(e2, 'σ₂', { color: accent, dx: 9 * S.u2[0], dy: -9 * S.u2[1] - 2 });
    }

    if (show.force) {
      const ang = Math.atan2(S.u1[1], S.u1[0]);
      const fa = FORCE_SCALE / Math.max(S.smax, 1e-9);
      const fbRaw = S.smin > 1e-9 ? FORCE_SCALE / S.smin : Infinity;
      const fb = Math.min(FORCE_CAP, fbRaw);
      c.ellipse(tip, fa, fb, ang, { color: rose, width: 2, dash: [7, 4] });
      const out = S.u2[0] * tip[0] + S.u2[1] * tip[1] >= 0 ? 1 : -1;
      const end: Pt = [tip[0] + out * S.u2[0] * Math.min(fb, 1.2), tip[1] + out * S.u2[1] * Math.min(fb, 1.2)];
      labels.inside(c, end, fbRaw > FORCE_CAP ? '力橢圓（此方向 → ∞）' : '力橢圓', { color: rose, bg: bgElev, dy: -12 });
    }

    if (show.sweep) {
      // image of the unit θ̇ circle: the recently swept part plus the current arrow J θ̇
      const trail: Pt[] = [];
      for (let k = 0; k <= 48; k++) {
        const v = tipVelocity(J, sweepQdot(psi - (1.6 * Math.PI * (48 - k)) / 48));
        trail.push([tip[0] + v[0] * VEL_SCALE, tip[1] + v[1] * VEL_SCALE]);
      }
      c.polyline(trail, { color: purple, width: 4, alpha: 0.5 });
      const v = tipVelocity(J, sweepQdot(psi));
      const head: Pt = [tip[0] + v[0] * VEL_SCALE, tip[1] + v[1] * VEL_SCALE];
      c.arrow(tip, head, { color: purple, width: 3 });
      // joint rates are written dθ/dt in canvas text (a combining dot renders as a stray tick);
      // the label sits past the arrow head, i.e. just outside the ellipse, on a background pill
      const vm = Math.hypot(v[0], v[1]) || 1;
      labels.beside(c, head, 'J·dθ/dt', { color: purple, bg: bgElev, font: 'bold 12px sans-serif' }, [v[0] / vm, -v[1] / vm], 6);
      // inset: the unit circle of joint velocities
      const g = insetGeom();
      const [x0, y0] = g.c;
      const P = g.P;
      const Rpx = g.R * c.scale;
      c.polygon([[x0 - P, y0 - P], [x0 + P, y0 - P], [x0 + P, y0 + P], [x0 - P, y0 + P]], { fill: bgElev, color: cssVar('--border', '#ddd'), width: 1, alpha: 0.94 });
      c.line([x0 - g.R * 1.2, y0], [x0 + g.R * 1.2, y0], { color: muted, width: 1 });
      c.line([x0, y0 - g.R * 1.2], [x0, y0 + g.R * 1.2], { color: muted, width: 1 });
      c.circle([x0, y0], g.R, { color: fg, width: 1.5 });
      // axis names inside the circle, clear of the moving handle on the rim
      const [lx, ly] = L.length === 2 ? ['dθ₁/dt', 'dθ₂/dt'] : ['v₁', 'v₂'];
      const axFont = `${Rpx < 40 ? 9 : 10}px sans-serif`; // the inset shrinks on phones
      c.text([x0, y0], lx, { color: muted, align: 'right', dx: Rpx - 5, dy: 8, font: axFont });
      c.text([x0, y0], ly, { color: muted, align: 'left', dx: 4, dy: -0.42 * Rpx, font: axFont });
      c.text([x0, y0 - P], L.length === 2 ? '‖dθ/dt‖ = 1' : '列空間中 ‖dθ/dt‖ = 1', { color: muted, dy: -9, font: '11px sans-serif' });
    }

    if (step === 5) {
      const r = Math.hypot(tip[0], tip[1]);
      if (r > 1e-6) {
        const u: Pt = [tip[0] / r, tip[1] / r];
        // a direction marker: at the 0.4 m per m/s scale, 0.1 m/s would be only 4 cm long
        const e: Pt = [tip[0] + u[0] * 0.45, tip[1] + u[1] * 0.45];
        const ok = cssVar('--ok', '#15803d');
        c.arrow(tip, e, { color: ok, width: 2.5 });
        labels.beside(c, e, 'v 的方向（徑向）', { color: ok, bg: bgElev }, [u[0], -u[1]], 6);
      }
    }

    if (lockTip && lockTarget) {
      const w = cssVar('--warn', '#b45309');
      c.circle(lockTarget, 0.09, { color: w, width: 2 });
      c.line([lockTarget[0] - 0.14, lockTarget[1]], [lockTarget[0] + 0.14, lockTarget[1]], { color: w, width: 1.5 });
      c.line([lockTarget[0], lockTarget[1] - 0.14], [lockTarget[0], lockTarget[1] + 0.14], { color: w, width: 1.5 });
    }
    labels.flush();

    const degs = th.map((t) => `${fmt(t / D, 0)}°`).join(', ');
    const status = S.rank < 2 ? 'rank J_v = 1：奇異！' : `rank J_v = 2，μ₁ = ${fmt(S.mu1, 1)}${S.mu1 > 12 ? '（接近奇異）' : ''}`;
    c.setOverlayText(`${model}  θ = (${degs})  ${status}${lockClamped ? '  ⚠ 此 θ₃ 下鎖住的點到不了' : ''}`);
  };

  // ---------------------------------------------------------------- per-step presets
  const A3 = [30 * D, 0, Math.PI];
  const tipA = (() => { const t = planarFK(L3R, A3).tip; return [t[0], t[1]] as Pt; })();
  const B3 = solveTip(L3R, [0, 0, 90 * D], tipA, 1).th;
  const B3b = solveTip(L3R, [0, 0, 90 * D], tipA, -1).th;
  // presets end a running radial run first (otherwise its next frame would overwrite the preset)
  const use2R = (q: number[]) => () => { interrupt(); setModel('2R'); ghost = null; setConfig(q); };
  const use3R = (q: number[], g: number[] | null = null, lock = false) => () => {
    interrupt(); setModel('3R'); setLock(false); setConfig(q); ghost = g; if (lock) setLock(true); update();
  };
  const chipSpecs: { label: string; apply: () => void }[][] = [
    [
      { label: 'θ₂ = 60°', apply: use2R([25 * D, 60 * D]) },
      { label: 'θ₂ = 20°', apply: use2R([25 * D, 20 * D]) },
      { label: 'θ₂ = 5°', apply: use2R([25 * D, 5 * D]) },
      { label: 'θ₂ = 0°（伸直）', apply: use2R([25 * D, 0]) },
    ],
    [
      { label: '構型 A（奇異）', apply: use3R(A3, B3) },
      { label: '構型 B（非奇異）', apply: use3R(B3, A3) },
      { label: '構型 B′（另一分支）', apply: use3R(B3b, A3) },
      { label: '鎖住末端，拖 θ₃', apply: use3R(B3, A3, true) },
    ],
    [
      { label: 'θ₂ = 90°', apply: use2R([20 * D, 90 * D]) },
      { label: 'θ₂ = 30°', apply: use2R([20 * D, 30 * D]) },
      { label: 'θ₂ = 8°', apply: use2R([20 * D, 8 * D]) },
    ],
    [
      { label: '2R', apply: use2R([20 * D, 45 * D]) },
      { label: '3R：θ₃ = 180°', apply: use3R([20 * D, 45 * D, Math.PI]) },
      { label: '3R：θ₃ = 90°', apply: use3R([20 * D, 45 * D, 90 * D]) },
    ],
    [
      { label: '2R 地圖', apply: use2R([20 * D, 70 * D]) },
      { label: '3R：θ₃ = 180°', apply: use3R([20 * D, 60 * D, Math.PI]) },
      { label: '3R：θ₃ = 90°', apply: use3R([20 * D, 60 * D, 90 * D]) },
      { label: '3R：θ₃ = 0°', apply: use3R([20 * D, 60 * D, 0]) },
    ],
    [
      { label: '2R：θ₂ = 60°', apply: use2R([20 * D, 60 * D]) },
      { label: '2R：θ₂ = 15°', apply: use2R([20 * D, 15 * D]) },
      { label: '2R：θ₂ = 3°', apply: use2R([20 * D, 3 * D]) },
    ],
  ];
  const chipGroups: PresetChipsHandle[] = chipSpecs.map((specs) => {
    const g = presetChips(chipsRow, specs, { label: '預設：' });
    g.el.style.display = 'none';
    return g;
  });

  // view per step: which overlays are on, which preset is applied, what gets the focus ring
  const VIEW: { ellipse: boolean; cols: boolean; heat: boolean; sweep: boolean; chip: number }[] = [
    { ellipse: false, cols: true, heat: false, sweep: false, chip: 1 },
    { ellipse: true, cols: true, heat: false, sweep: false, chip: 0 },
    { ellipse: true, cols: false, heat: false, sweep: true, chip: 0 },
    { ellipse: true, cols: false, heat: false, sweep: false, chip: 0 },
    { ellipse: true, cols: false, heat: true, sweep: false, chip: 0 },
    { ellipse: true, cols: false, heat: false, sweep: false, chip: 0 },
  ];
  const focusFor = (i: number): HTMLElement[] => [
    [cv.el, rrow], [chipGroups[1].el], [cv.el], [plotsWrap], [tHeat.el, cv.el], [ratePanel],
  ][i] ?? [];
  let focused: HTMLElement[] = [];

  const onStep = (i: number): void => {
    const k = Math.max(0, Math.min(steps.length - 1, i));
    stopRadial();
    radialMsg = '';
    step = k;
    chipGroups.forEach((g, j) => { g.el.style.display = j === k ? '' : 'none'; });
    const v = VIEW[k];
    tEllipse.set(v.ellipse);
    tCols.set(v.cols);
    tHeat.set(v.heat);
    tForce.set(false);
    setSweep(v.sweep);
    setLock(false);
    ghost = null;
    ratePanel.style.display = k === 5 ? '' : 'none';
    plotsWrap.style.display = k === 5 ? 'none' : '';
    mJ.el.style.display = k <= 1 ? '' : 'none';
    chipGroups[k].activate(v.chip);
    focused.forEach((f) => f.classList.remove('mr-focus'));
    focused = focusFor(k);
    focused.forEach((f) => f.classList.add('mr-focus'));
    update();
  };

  // initial state
  s3.el.style.display = 'none';
  tLock.el.style.display = 'none';
  bars3.el.style.display = 'none';
  heatLegend.style.display = 'none';
  onStep(ctx.currentStep);

  return {
    onStep,
    destroy() {
      anim.stop();
      radial = null;
      cv.destroy();
      plotS.destroy();
      plotM.destroy();
    },
  };
}

/**
 * Places the canvas labels of one frame: each label is centred on its anchor, pulled back inside the
 * canvas so it is never clipped, and nudged vertically when it would cover a label placed earlier in
 * the same frame (near a singularity several labels point the same way). The labels are drawn by
 * flush() at the end of the frame, on top of every line and ellipse. Call reset() first.
 */
class LabelPlacer {
  private rects: [number, number, number, number][] = [];
  private queue: (() => void)[] = [];

  reset(): void { this.rects = []; this.queue = []; }

  flush(): void { for (const draw of this.queue) draw(); this.queue = []; }

  /** Keep labels off a screen square around world point p (e.g. a drag handle Canvas2D draws last). */
  reserve(c: Canvas2D, p: Pt, halfPx: number): void {
    const [sx, sy] = c.toScreen(p);
    this.rects.push([sx - halfPx, sy - halfPx, sx + halfPx, sy + halfPx]);
  }

  inside(c: Canvas2D, p: Pt, str: string, style: TextStyle): void {
    c.ctx.font = style.font ?? `12px ${cssVar('--font') || 'sans-serif'}`;
    const w = c.ctx.measureText(str).width;
    const half = w / 2 + 8, hw = w / 2 + 5, hh = 9;
    const [sx, sy] = c.toScreen(p);
    const x = Math.min(c.width - half, Math.max(half, sx + (style.dx ?? 0)));
    const clampY = (y: number) => Math.min(c.height - 12, Math.max(12, y));
    const y0 = clampY(sy + (style.dy ?? 0));
    const free = (y: number) => this.rects.every(([a, b, cc, d]) => x + hw <= a || x - hw >= cc || y + hh <= b || y - hh >= d);
    let y = y0;
    for (const k of [0, 1, -1, 2, -2, 3, -3]) {
      const cand = clampY(y0 + k * 2 * hh);
      if (free(cand)) { y = cand; break; }
    }
    this.rects.push([x - hw, y - hh, x + hw, y + hh]);
    const at = c.toWorld(x, y);
    this.queue.push(() => c.text(at, str, { ...style, dx: 0, dy: 0 }));
  }

  /** Label just beyond an anchor along a screen-space direction (e.g. past an arrow head). */
  beside(c: Canvas2D, p: Pt, str: string, style: TextStyle, dir: Pt, gap = 8): void {
    c.ctx.font = style.font ?? `12px ${cssVar('--font') || 'sans-serif'}`;
    const halfW = c.ctx.measureText(str).width / 2 + 4;
    const off = Math.abs(dir[0]) * halfW + Math.abs(dir[1]) * 9 + gap;
    this.inside(c, p, str, { ...style, dx: dir[0] * off, dy: dir[1] * off });
  }
}

function rgbCss(c: [number, number, number, number]): string {
  return `rgb(${Math.round(c[0])}, ${Math.round(c[1])}, ${Math.round(c[2])})`;
}

const lesson: Lesson = { meta, steps, quiz, mount };
export default lesson;
