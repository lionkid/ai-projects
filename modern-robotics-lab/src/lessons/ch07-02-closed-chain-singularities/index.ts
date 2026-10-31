import type { Lesson, LessonContext, LessonHandle, LessonStep, QuizQuestion } from '@/types';
import {
  el, cssVar, fmt, controlPanel, row, angleSlider, toggle, simTitle, legend, kv, matrixView, playback,
  Canvas2D, Plot2D, presetChips, torqueBars, createLoop,
  type Handle, type PresetChipsHandle, type TextStyle,
} from '@/ui';
import { meta } from './meta';
import {
  DEFAULT_GEOM, basePoint, fkNearest, fkMode, driveMotors, driveTip, constraintJacobians, eeJacobian,
  jacobianFD, inverseRates, velocityEllipse, indicators, tipVelocity, holdingTorques, lockStretch, softestDirection,
  motorSlice, symmetricMarginAngle, straightLegConfig, doubleSingularConfig, buildSnapPath, snapConfig,
  type Config, type FiveBarGeom, type Pt, type Side, type SnapPath,
} from './fiveBarDiff';

const D = Math.PI / 180;
const WORLD = { xmin: -2.6, xmax: 2.6, ymin: -1.0, ymax: 2.45 };
/** World length of 1 m/s of tip speed (ellipse, J θ̇ arrow). */
const VEL_SCALE = 0.35;
/** Longest drawn semi-axis / arrow (world units); longer ones are drawn dashed with "→ ∞". */
const CAP = 2.2;
/** World length of the 1 N load arrow. */
const FORCE_LEN = 0.55;
/** Motor speed during the snap-through animation (rad/s). */
const OMEGA = 0.3;
const DEFAULT_MOTORS: [number, number] = [115 * D, 65 * D];

const steps: LessonStep[] = [
  {
    title: '把閉合方程式微分',
    md: `上一課的迴路閉合式 $g(\\theta_a, \\theta_p) = p_L - p_R = 0$ 在運動中恆成立，對時間微分就得到速度層級的約束：

$$H_a(\\theta)\\,\\dot\\theta_a + H_p(\\theta)\\,\\dot\\theta_p = 0,\\qquad H_a = \\frac{\\partial g}{\\partial \\theta_a},\\ \\ H_p = \\frac{\\partial g}{\\partial \\theta_p}.$$

五連桿的 $H_a, H_p$ 都是 $2\\times 2$，每一欄是「只轉一個關節時，那條腿端點的速度」（左腿取正號、右腿取負號）。只要 $H_p$ 可逆，被動關節的速度就由馬達唯一決定：

$$\\dot\\theta_p = -H_p^{-1} H_a\\,\\dot\\theta_a .$$`,
    simHint: '轉動 θ₁、θ₂（或拖藍色末端），看 H_a、H_p 與 det H_p 即時改變。下方曲線：固定 θ₂、把 θ₁ 從一端掃到另一端時，三個奇異指標怎麼變。',
  },
  {
    title: '末端速度橢球',
    md: `末端速度可以沿左腿計算：$v = J_a\\dot\\theta_a + J_p\\dot\\theta_p$。代入上一步的被動速度，就只剩馬達速度：

$$v = J\\,\\dot\\theta_a,\\qquad J = J_a - J_p H_p^{-1} H_a .$$

讓馬達速度走遍單位圓 $\\|\\dot\\theta_a\\| = 1$，$v$ 就描出一個橢圓，兩個半軸長是 $J$ 的奇異值。這和開鏈的可操作度橢球是同一回事，只是閉鏈的 $J$ 裡藏著 $H_p^{-1}$。打開「有限差分 J」可以確認：直接對 FK 做數值微分，得到的矩陣完全相同。`,
    simHint: '拖曳右下角單位圓上的紫點（馬達速度方向 θ̇），末端的紫色箭頭 Jθ̇ 會沿著橢圓繞行；打開「有限差分 J 對照」比較兩個矩陣。',
  },
  {
    title: '末端奇異：有方向動不了',
    md: `把一條腿完全伸直（或折疊），它就像伸直的 2R 手臂：端點沿腿方向的速度必為零，兩顆馬達怎麼組合都推不動末端往那個方向走。橢圓在這個方向被壓成線段，稱為**末端奇異（end-effector singularity）**。

改用「每條腿的遠端桿長固定」來寫更容易看出原因：$A v = B\\,\\dot\\theta_a$，其中 $A$ 的兩列是 $(p - B_i)^{\\mathsf T}$，

$$B = \\diag(L_1L_2\\sin\\phi_1,\\ L_3L_4\\sin\\phi_2).$$

腿伸直時 $\\sin\\phi_i = 0$，$B$ 降秩，$J = A^{-1}B$ 也跟著降秩；此時末端正好位在工作空間的邊界上。`,
    simHint: '按「左腿伸直」「右腿伸直」：橢圓壓成一條線，紅色虛線是動不了的方向，曲線圖上的 sin φ 也剛好穿過 0。',
  },
  {
    title: '致動器奇異：鎖住馬達還能動',
    md: `另一種情況是兩根遠端桿排成一直線，也就是上一課 FK 兩解合一的地方。這時 $H_p$ 的兩欄平行（$\\times$ 為二維外積）：

$$\\det H_p = -(p - B_1)\\times(p - B_2) = 0 .$$

被動速度不再由馬達唯一決定，$J$ 裡的 $H_p^{-1}$ 發散，速度橢圓沿垂直於遠端桿的方向**爆開**，而不是壓扁。它的物理意義是：就算把兩顆馬達鎖死，末端仍可沿這個方向微動，因為兩根遠端桿只需要二階小量的伸長就能容許這個位移。這稱為**致動器奇異（actuator singularity）**。`,
    simHint: '馬達已鎖住。拖動藍色末端：沿紫色雙箭頭（垂直於遠端桿）拖時，兩根桿幾乎不必伸長；改成沿桿的方向拖，需要的伸長量立刻和位移一樣大。',
  },
  {
    title: '為什麼致動器奇異更危險',
    md: `末端奇異只是「某個方向動不了」：沿那個方向的負載由伸直的腿以拉、壓承受，馬達反而不必出力。致動器奇異恰好相反，要平衡末端負載 $f$ 所需的馬達力矩是

$$\\tau = J^{\\mathsf T} f ,$$

而 $J$ 在這裡發散：沿自由方向的負載需要趨近無限大的力矩，馬達撐不住。更麻煩的是兩個裝配模式在此合一，回程時馬達無法決定機構走哪一支，機構可能突然**翻面**到另一個裝配模式，造成衝擊甚至碰撞。`,
    simHint: '按 ▶ 播放：θ₁ 等速轉向奇異構型，末端速率曲線在奇異瞬間暴衝，回程時機構翻到另一個裝配模式。再用預設比較兩類奇異下撐住 1 N 負載的馬達力矩。',
  },
  {
    title: '冗餘致動與下一章',
    md: `在左肘再裝一顆馬達，用三顆馬達驅動 2 個自由度，稱為**冗餘致動（redundant actuation）**。鎖住全部馬達後左腿已是剛體，末端完全動不了：$H_p$ 只剩右肘那一欄，長度恆為 $L_4$，永遠滿秩，致動器奇異因此消失。末端奇異卻還在：腿伸直時，這條腿的幾何依然禁止末端沿腿方向運動，多一顆馬達也變不出這個方向的速度。代價是三顆馬達必須協調出力，否則會互相拉扯。

下一章進入動力學：質量矩陣 $M(\\theta)$ 同樣會隨構型改變。`,
    simHint: '比較「兩顆馬達」與「三顆馬達」兩個預設：同樣在遠端桿共線處，力矩長條從飽和變成有限，鎖住馬達也拖不動；再按「三顆馬達：左腿伸直」，橢圓仍然壓扁。',
  },
];

const quiz: QuizQuestion[] = [
  {
    question: '平面 3RRR 並聯平台：三條腿各由底座馬達驅動近端桿，遠端桿再以被動關節連到三角形平台。依照五連桿的經驗，你預期它的**致動器奇異**在什麼幾何條件下發生？',
    options: [
      '三根遠端桿的延長線交於同一點（或三者互相平行）',
      '任一條腿的近端桿與遠端桿伸直成一線',
      '平台三角形的面積為零',
      '三顆馬達的角度相等',
    ],
    answer: 0,
    explanation: '鎖住馬達後，每根遠端桿只限制平台接點沿桿方向的速度分量，等於三條沿遠端桿的約束線。三條線交於一點時，平台繞該點的微小轉動不會拉長任何一根桿；三條線平行時，垂直於它們的微小平移也一樣。這就是不受馬達控制的運動，和五連桿兩根遠端桿共線是同一個道理。選項 B 是腿伸直，對應的是末端奇異。',
  },
  {
    question: '為什麼工程上通常把致動器奇異看得比末端奇異更危險？',
    options: [
      '致動器奇異時末端速度一定是零',
      '沿自由方向的負載需要趨近無限大的馬達力矩，鎖住馬達也擋不住，機構還可能翻到另一個裝配模式',
      '致動器奇異只出現在工作空間邊界，容易撞到極限',
      '末端奇異會讓馬達力矩變成無限大，致動器奇異不會',
    ],
    answer: 1,
    explanation: '由 $\\tau = J^{\\mathsf T}f$，$J$ 在致動器奇異發散，沿自由方向的負載要求無限大的力矩，等於馬達失去控制權；兩個裝配模式在此合一，機構也可能翻面。末端奇異則相反：伸直的腿以拉、壓承受沿腿方向的負載，馬達力矩反而是零。致動器奇異通常出現在工作空間內部，而不是邊界。',
  },
  {
    question: '在五連桿的左肘多裝一顆馬達（共三顆）。下列何者正確？',
    options: [
      '兩類奇異都消失',
      '只消除致動器奇異：鎖住三顆馬達後左腿已是剛體；但腿伸直時末端仍無法沿腿方向運動',
      '只消除末端奇異',
      '兩類奇異都還在，只是換了位置',
    ],
    answer: 1,
    explanation: '致動器奇異的本質是「鎖住所有馬達後機構仍有自由度」。左肘也鎖住時左腿完全剛化，這種自由度不可能出現（$H_p$ 只剩一欄，長度恆為 $L_4$）。末端奇異來自腿的幾何：伸直的腿不論怎麼驅動，都無法讓端點沿腿方向移動，增加馬達改變不了這件事。',
  },
  {
    question: '某構型下 $\\det H_p \\approx 0.003\\ \\text{m}^2$，而 $\\sin\\phi_1 \\approx -0.9$、$\\sin\\phi_2 \\approx 0.8$（$L_2 = L_4 = 1.3$ m）。模擬中的末端速度橢圓最可能長什麼樣子？',
    options: [
      '幾乎是圓形',
      '在某個方向被壓扁成線段，因為某條腿接近伸直',
      '沿垂直於遠端桿的方向被拉得非常長',
      '整個橢圓縮成一個點',
    ],
    answer: 2,
    explanation: '$|\\det H_p| = |\\det A|$ 很小（除以 $L_2L_4$ 後約 0.002）代表兩根遠端桿幾乎共線，接近致動器奇異；兩個 $\\sin\\phi_i$ 都離 0 很遠，表示沒有腿接近伸直，$B$ 是良態的。$J = A^{-1}B$ 因 $A^{-1}$ 而沿 $A$ 的近零空間方向（垂直於遠端桿）放大，所以橢圓沿該方向被拉長，而不是壓扁。',
  },
];

const deg = (x: number, d = 1): string => `${fmt(x / D, d)}°`;

function mount(host: HTMLElement, ctx: LessonContext): LessonHandle {
  // ---------------------------------------------------------------- state
  const g: FiveBarGeom = { ...DEFAULT_GEOM };
  let cfg: Config = fkNearest(g, DEFAULT_MOTORS[0], DEFAULT_MOTORS[1], null, 1)!;
  let locked = false;
  let redundant = false;
  let virt: Pt | null = null;
  let rateDir = 0.6;
  let loadDir = 90 * D;
  let step = -1;
  let msg = '';
  const show = { ellipse: true, fd: false, inset: false, matrices: true, slice: true, bars: false, snap: false, load: false };
  let snap: SnapPath | null = null;
  let snapT = 0;
  let flip = true;
  let trail: Pt[] = [];

  // ---------------------------------------------------------------- layout
  simTitle(host, '五連桿的速度與兩類奇異', 'H_a θ̇_a + H_p θ̇_p = 0');
  const cv = new Canvas2D(host, { aspect: 1.7, world: WORLD, overlay: true });
  const status = el('p', { class: 'mr-note', style: { minHeight: '1.2em' } });
  host.appendChild(status);
  legend(host, [
    { color: 'var(--accent)', label: '馬達（實心方塊）' },
    { color: 'var(--accent)', label: '速度橢圓（‖θ̇_a‖ = 1）' },
    { color: 'var(--danger)', label: '末端動不了的方向' },
    { color: 'var(--series-5)', label: '鎖住馬達仍可動的方向 / Jθ̇' },
  ]);

  const panel = controlPanel(host, { compact: true });
  const chipsRow = row(panel);
  const optRow = row(panel);
  const tLock = toggle(optRow, { label: '鎖住馬達（試拖末端）', onChange: (v) => { clearChips(); setLocked(v); update(); } });
  const tRed = toggle(optRow, { label: '第三顆馬達（左肘）', onChange: (v) => { clearChips(); setRedundant(v); update(); } });
  const tEll = toggle(optRow, { label: '速度橢圓', value: true, onChange: (v) => { show.ellipse = v; update(); } });
  const tFD = toggle(optRow, { label: '有限差分 J 對照', onChange: (v) => { show.fd = v; update(); } });
  const s1 = angleSlider(panel, { label: 'θ₁（左馬達）', value: cfg.th1, onInput: (v) => { clearChips(); requestMotor(0, v); } });
  const s2 = angleSlider(panel, { label: 'θ₂（右馬達）', value: cfg.th2, onInput: (v) => { clearChips(); requestMotor(1, v); } });
  const snapRow = row(panel);
  const pb = playback(snapRow, {
    duration: 1,
    onPlay: () => { if (!snap) startSnap(); if (snap && snapT >= 2 * snap.T1 - 1e-9) snapT = 0; anim.start(); syncUI(); },
    onPause: () => { anim.stop(); syncUI(); },
    onReset: () => { anim.stop(); pb.setPlaying(false, { silent: true }); snapT = 0; applySnap(); },
    onSpeed: (sp) => { anim.speed = sp; },
    onScrub: (t) => { anim.stop(); pb.setPlaying(false, { silent: true }); snapT = t; applySnap(); },
  });
  const tFlip = toggle(snapRow, { label: '回程翻到另一個裝配模式', value: true, onChange: (v) => { flip = v; computeSnapSeries(); applySnap(); } });
  const sLoad = angleSlider(panel, { label: '末端負載 f 的方向（1 N）', value: loadDir, onInput: (v) => { loadDir = v; update(); } });

  const matWrap = el('div', { style: { overflowX: 'auto' } });
  host.appendChild(matWrap);
  const mHa = matrixView(matWrap, { label: 'H_a =', digits: 2 });
  const mHp = matrixView(matWrap, { label: 'H_p =', digits: 2 });
  const mJ = matrixView(matWrap, { label: 'J =', digits: 2 });
  const mFD = matrixView(matWrap, { label: 'J_FD =', digits: 2 });

  const K = {
    act: '致動器指標 det H_p ⁄ (L₂L₄)', ee: '末端指標 sin φ₁, sin φ₂', ell: '速度橢圓半軸（m/s 每 rad/s）',
    lock: '鎖住馬達：最軟方向剛性', drag: '虛擬位移 → 遠端桿伸長', tau: '撐住 1 N 負載的馬達力矩',
  };
  const info = kv(host, Object.values(K));
  const rowOf = (key: string): HTMLElement[] => {
    const dts = Array.from(info.el.querySelectorAll('dt'));
    const dt = dts.find((d) => d.textContent === key);
    return dt ? [dt, dt.nextElementSibling as HTMLElement] : [];
  };
  const showRow = (key: string, on: boolean) => rowOf(key).forEach((e) => { e.style.display = on ? '' : 'none'; });

  const bars2 = torqueBars(host, { labels: ['τ₁', 'τ₂'], max: 6, limit: 3, unit: 'N·m', title: '撐住末端 1 N 負載所需的馬達力矩（紅線：±3 N·m）' });
  const bars3 = torqueBars(host, { labels: ['τ₁', 'τ_φ₁', 'τ₂'], max: 6, limit: 3, unit: 'N·m', title: '三顆馬達：最小範數的平衡力矩（紅線：±3 N·m）' });

  const plotSlice = new Plot2D(host, {
    title: '固定 θ₂、掃描 θ₁：奇異指標', xLabel: 'θ₁ (°)', height: 150, yRange: [-1.1, 1.1],
    series: [{ name: 'det H_p ⁄ (L₂L₄)', width: 2.4 }, { name: 'sin φ₁', dash: [6, 4] }, { name: 'sin φ₂', dash: [2, 3] }],
  });
  const plotSnap = new Plot2D(host, {
    title: '穿越致動器奇異：末端速率（θ₁ 等速 0.3 rad/s）', xLabel: 't (s)', yLabel: '|v| (m/s)', height: 150, yRange: [0, 3],
    series: [{ name: '|v|', color: cssVar('--series-5', '#7c3aed') }],
  });

  // ---------------------------------------------------------------- core actions
  const general = (): Config => fkNearest(g, DEFAULT_MOTORS[0], DEFAULT_MOTORS[1], null, 1)!;
  const tangentCfg = (margin: number): Config => {
    const t2 = symmetricMarginAngle(g, margin)!;
    return fkMode(g, Math.PI - t2, t2, 1)!;
  };

  function setCfg(c: Config): void {
    cfg = c;
    virt = null;
  }

  function requestMotor(side: Side, th: number): void {
    if (locked || anim.running) { update(); return; }
    stopSnapState();
    const res = driveMotors(g, cfg, side === 0 ? th : cfg.th1, side === 1 ? th : cfg.th2);
    setCfg(res.cfg);
    msg = res.blocked ? '⚠ 兩根遠端桿已共線（兩圓相切）：再轉就無法裝配，馬達停在致動器奇異上' : '';
    update();
  }

  function requestTip(p: Pt): void {
    const v = cv.visible();
    const q: Pt = [Math.min(v.xmax - 0.08, Math.max(v.xmin + 0.08, p[0])), Math.min(v.ymax - 0.08, Math.max(v.ymin + 0.08, p[1]))];
    if (locked) { virt = q; update(); return; }
    stopSnapState();
    const res = driveTip(g, cfg, q);
    setCfg(res.cfg);
    msg = res.blocked ? '⚠ 一條腿已完全伸直（或折疊）：末端到了工作空間邊界，這正是末端奇異' : '';
    update();
  }

  function setLocked(v: boolean): void {
    locked = v;
    virt = null;
    tLock.set(v, { silent: true });
    syncUI();
  }

  function setRedundant(v: boolean): void {
    redundant = v;
    tRed.set(v, { silent: true });
    plotSlice.setOptions({
      series: [
        { name: v ? '三顆馬達：|H_p| ⁄ L₄（恆為 1）' : 'det H_p ⁄ (L₂L₄)', width: 2.4 },
        { name: 'sin φ₁', dash: [6, 4] },
        { name: 'sin φ₂', dash: [2, 3] },
      ],
    });
    syncUI();
  }

  function clearChips(): void {
    chipGroups[step]?.clear();
  }

  // ---------------------------------------------------------------- snap-through animation
  function startSnap(): void {
    const start = tangentCfg(0.6);
    snap = buildSnapPath(g, start, OMEGA);
    snapT = 0;
    if (snap) pb.setDuration(2 * snap.T1);
    computeSnapSeries();
    applySnap();
  }

  function stopSnapState(): void {
    if (anim.running) anim.stop();
    pb.setPlaying(false, { silent: true });
    trail = [];
  }

  function snapSpeed(t: number): number {
    if (!snap) return 0;
    const s = snapConfig(g, snap, t, flip);
    if (!s.cfg) return NaN;
    const v = tipVelocity(g, s.cfg, [s.rate, 0]);
    return v ? Math.hypot(v[0], v[1]) : Infinity;
  }

  function computeSnapSeries(): void {
    if (!snap) return;
    const N = 400;
    const xs: number[] = [];
    const ys: number[] = [];
    for (let k = 0; k <= N; k++) {
      const t = (2 * snap.T1 * k) / N;
      xs.push(t);
      ys.push(Math.min(3.05, snapSpeed(t)));
    }
    plotSnap.setXY(0, xs, ys);
  }

  function applySnap(): void {
    if (!snap) return;
    const s = snapConfig(g, snap, snapT, flip);
    if (s.cfg) setCfg(s.cfg);
    trail = [];
    const n = Math.max(2, Math.ceil((snapT / (2 * snap.T1)) * 160));
    for (let k = 0; k <= n; k++) {
      const q = snapConfig(g, snap, (snapT * k) / n, flip).cfg;
      if (q) trail.push(q.P);
    }
    pb.setTime(snapT);
    const near = Math.abs(snapT - snap.T1) < 0.06;
    msg = near ? '⚠ 奇異瞬間：兩根遠端桿共線，兩個裝配模式合而為一，馬達無法決定機構往哪一支走'
      : snapT > snap.T1 ? (flip ? '⚠ 回程：機構翻到了另一個裝配模式（馬達角和去程一樣，末端卻在另一側）' : '回程沿原本的裝配模式回到起點')
        : '去程：θ₁ 等速轉動，越靠近奇異構型，末端跑得越快';
    plotSnap.setMarkers([{ x: snap.T1, label: '奇異', dash: [3, 3] }, { x: snapT, color: cssVar('--fg', '#222'), dash: [] }]);
    update();
  }

  const anim = createLoop((dt) => {
    if (!snap) { anim.stop(); return; }
    snapT = Math.min(2 * snap.T1, snapT + dt);
    applySnap();
    if (snapT >= 2 * snap.T1) { anim.stop(); pb.setPlaying(false, { silent: true }); syncUI(); }
  });

  // ---------------------------------------------------------------- handles
  const mkMotorHandle = (side: Side): Handle => cv.addHandle({
    get: () => {
      const A = basePoint(g, side);
      const B = side === 0 ? cfg.B1 : cfg.B2;
      return [A[0] + 0.62 * (B[0] - A[0]), A[1] + 0.62 * (B[1] - A[1])];
    },
    set: (p) => {
      clearChips();
      const A = basePoint(g, side);
      requestMotor(side, Math.atan2(p[1] - A[1], p[0] - A[0]));
    },
    radius: 8,
  });
  const hM1 = mkMotorHandle(0);
  const hM2 = mkMotorHandle(1);
  const hTip = cv.addHandle({
    get: () => virt ?? cfg.P,
    set: (p) => { clearChips(); requestTip(p); },
    onStart: () => { if (locked) virt = [cfg.P[0], cfg.P[1]]; },
    onEnd: () => { if (locked) { virt = null; msg = '放開後末端彈回原位：馬達仍鎖著'; update(); } },
    radius: 9,
  });
  const insetGeom = (): { c: Pt; R: number; P: number } => {
    const Rpx = Math.max(22, Math.min(42, cv.height * 0.13, cv.width * 0.08));
    const c = cv.toWorld(cv.width - Rpx - 24, cv.height - Rpx - 26);
    return { c, R: Rpx / cv.scale, P: (Rpx + 18) / cv.scale };
  };
  const hRate = cv.addHandle({
    get: () => { const q = insetGeom(); return [q.c[0] + q.R * Math.cos(rateDir), q.c[1] + q.R * Math.sin(rateDir)]; },
    set: (p) => { const q = insetGeom(); rateDir = Math.atan2(p[1] - q.c[1], p[0] - q.c[0]); update(); },
    radius: 7,
    enabled: false,
  });

  function syncUI(): void {
    const playing = anim.running;
    hM1.enabled = hM2.enabled = !locked && !playing;
    s1.disabled = s2.disabled = locked || playing;
    hTip.enabled = !playing;
    hRate.enabled = show.inset && !redundant;
    matWrap.style.display = show.matrices ? '' : 'none';
    mFD.el.style.display = show.fd && !redundant ? '' : 'none';
    snapRow.style.display = show.snap ? '' : 'none';
    sLoad.el.style.display = show.load ? '' : 'none';
    bars2.el.style.display = show.bars && !redundant ? '' : 'none';
    bars3.el.style.display = show.bars && redundant ? '' : 'none';
    plotSlice.el.style.display = show.slice ? '' : 'none';
    plotSnap.el.style.display = show.snap ? '' : 'none';
    showRow(K.lock, locked);
    showRow(K.drag, locked);
    showRow(K.tau, show.bars);
    cv.requestRender();
  }

  // ---------------------------------------------------------------- readouts
  function updateMatrices(): void {
    if (!show.matrices) return;
    const { Ha, Hp } = constraintJacobians(g, cfg, redundant);
    mHa.setLabel(redundant ? 'H_a (2×3) =' : 'H_a =');
    mHp.setLabel(redundant ? 'H_p (2×1) =' : 'H_p =');
    mHa.set(Ha);
    mHp.set(Hp);
    if (redundant) {
      mJ.setLabel('θ̇_a = K v，K =');
      mJ.set(inverseRates(g, cfg, true));
    } else {
      const J = eeJacobian(g, cfg);
      mJ.setLabel(J ? 'J =' : 'J = （H_p 不可逆：發散）');
      if (J) mJ.set(J); else mJ.set([] as number[]);
      if (show.fd) {
        const F = jacobianFD(g, cfg);
        if (F) {
          const diff = J ? Math.max(...[0, 1].flatMap((i) => [0, 1].map((j) => Math.abs(F[i][j] - J[i][j])))) : NaN;
          mFD.setLabel(`J_FD（差 ${Number.isFinite(diff) ? diff.toExponential(1) : '—'}）=`);
          mFD.set(F);
        }
      }
    }
  }

  let slicePts: { th1: number[]; act: number[]; s1: number[]; s2: number[] } | null = null;
  let sliceKey = '';
  function updateSlice(): void {
    if (!show.slice) return;
    const key = `${cfg.th2.toFixed(6)}|${cfg.mode}|${redundant}|${Math.round(cfg.th1 / (5 * D))}`;
    if (key !== sliceKey) {
      sliceKey = key;
      const sl = motorSlice(g, cfg, 0.01);
      const th1 = sl.th1.map((t) => t / D);
      slicePts = {
        th1,
        act: sl.configs.map((q) => (redundant ? 1 : indicators(g, q).act)),
        s1: sl.configs.map((q) => Math.sin(q.phi1)),
        s2: sl.configs.map((q) => Math.sin(q.phi2)),
      };
      plotSlice.setXY(0, th1, slicePts.act);
      plotSlice.setXY(1, th1, slicePts.s1);
      plotSlice.setXY(2, th1, slicePts.s2);
    }
    const ind = indicators(g, cfg);
    // place the cursor on the unwrapped axis of the slice
    let x = cfg.th1 / D;
    if (slicePts && slicePts.th1.length) {
      const lo = slicePts.th1[0];
      while (x < lo - 1e-6) x += 360;
      while (x > lo + 360) x -= 360;
    }
    plotSlice.setMarkers([
      { y: 0, dash: [2, 3] },
      { x },
      { x, y: redundant ? 1 : ind.act, color: cssVar('--series-1', '#2563eb') },
      { x, y: ind.s1, color: cssVar('--series-2', '#e11d48') },
      { x, y: ind.s2, color: cssVar('--series-3', '#16a34a') },
    ]);
  }

  function defaultStatus(ind: { act: number; s1: number; s2: number }): string {
    if (!redundant && Math.abs(ind.act) < 0.03) return '⚠ 兩根遠端桿（幾乎）共線：H_p 接近不可逆，這是致動器奇異';
    if (Math.abs(ind.s1) < 0.03 || Math.abs(ind.s2) < 0.03) return '⚠ 有一條腿（幾乎）伸直或折疊：B 降秩，這是末端奇異';
    return redundant ? '三顆馬達：H_p 只剩一欄，永遠滿秩' : 'H_p 可逆：被動關節速度 θ̇_p = −H_p⁻¹H_a θ̇_a 由馬達唯一決定';
  }

  function update(): void {
    s1.set(cfg.th1, { silent: true });
    s2.set(cfg.th2, { silent: true });
    const ind = indicators(g, cfg);
    info.set(K.act, redundant ? '（三顆馬達）|H_p| ⁄ L₄ = 1，永不為 0' : `${fmt(ind.act, 4)}${Math.abs(ind.act) < 0.03 ? '　→ 接近致動器奇異' : ''}`);
    info.set(K.ee, `${fmt(ind.s1, 3)}, ${fmt(ind.s2, 3)}${Math.min(Math.abs(ind.s1), Math.abs(ind.s2)) < 0.03 ? '　→ 接近末端奇異' : ''}`);
    const e = velocityEllipse(inverseRates(g, cfg, redundant));
    const fr = (r: number) => (Number.isFinite(r) && r < 1e6 ? fmt(r, 3) : '∞');
    info.set(K.ell, `σ₁ = ${fr(e.r[0])}，σ₂ = ${fr(e.r[1])}`);
    if (locked) {
      const sd = softestDirection(g, cfg, redundant);
      info.set(K.lock, `σ_min(Â) = ${fmt(sd.sigma, 3)}${sd.sigma < 0.03 ? '　→ 有一個方向幾乎不受約束' : ''}`);
      if (virt) {
        const lt = lockStretch(g, cfg, virt, redundant);
        const dlt = Math.hypot(virt[0] - cfg.P[0], virt[1] - cfg.P[1]);
        info.set(K.drag, `位移 ${fmt(1000 * dlt, 1)} mm → ${lt.rigidLeft ? '左腿（鎖死）偏離' : 'e₁'} ${fmt(1000 * lt.e1, 2)} mm，e₂ ${fmt(1000 * lt.e2, 2)} mm`);
      } else info.set(K.drag, '拖動末端看看');
    }
    if (show.bars) {
      const f: Pt = [Math.cos(loadDir), Math.sin(loadDir)];
      const t = holdingTorques(g, cfg, f, redundant);
      const bars = redundant ? bars3 : bars2;
      bars.set(t.exact ? t.tau : t.tau.map(() => bars === bars3 ? 0 : 1e3));
      if (!t.exact) bars.el.querySelectorAll('.mr-bar-value').forEach((e) => { e.textContent = '∞ ⚠'; });
      const nrm = Math.hypot(...t.tau);
      info.set(K.tau, t.exact ? `‖τ‖ = ${fmt(nrm, 2)} N·m${nrm > 3 ? '　⚠ 超過 3 N·m' : ''}` : '∞：沒有有限的馬達力矩能平衡這個負載');
    }
    updateMatrices();
    updateSlice();
    status.textContent = msg || defaultStatus(ind);
    const txt = status.textContent;
    status.style.color = txt.startsWith('⚠') ? 'var(--warn)' : '';
    cv.requestRender();
  }

  // ---------------------------------------------------------------- drawing
  const labelInside = (c: Canvas2D, p: Pt, str: string, style: TextStyle): void => {
    c.ctx.font = style.font ?? `12px ${cssVar('--font') || 'sans-serif'}`;
    const half = c.ctx.measureText(str).width / 2 + 8;
    const [sx, sy] = c.toScreen(p);
    const x = Math.min(c.width - half, Math.max(half, sx + (style.dx ?? 0)));
    const y = Math.min(c.height - 12, Math.max(12, sy + (style.dy ?? 0)));
    c.text(c.toWorld(x, y), str, { ...style, dx: 0, dy: 0 });
  };

  const motorSymbol = (c: Canvas2D, p: Pt): void => {
    const s = 8 / c.scale;
    const accent = cssVar('--accent', '#2563eb');
    c.polygon([[p[0] - s, p[1] - s], [p[0] + s, p[1] - s], [p[0] + s, p[1] + s], [p[0] - s, p[1] + s]], { fill: accent, color: accent, width: 1 });
  };
  const passiveSymbol = (c: Canvas2D, p: Pt, alpha = 1): void => {
    c.point(p, 6, { color: cssVar('--bg-elev', '#fff'), alpha });
    c.circle(p, 6 / c.scale, { color: cssVar('--fg', '#222'), width: 2, alpha });
  };

  function drawGround(c: Canvas2D): void {
    const muted = cssVar('--fg-muted', '#777');
    const A1 = basePoint(g, 0);
    const A2 = basePoint(g, 1);
    c.line([A1[0] - 0.3, 0], [A2[0] + 0.3, 0], { color: muted, width: 2 });
    for (let x = A1[0] - 0.24; x <= A2[0] + 0.3 + 1e-9; x += 0.12) c.line([x, 0], [x - 0.08, -0.08], { color: muted, width: 1 });
  }

  function drawMechanism(c: Canvas2D, q: Config, alpha = 1, thin = false): void {
    const A1 = basePoint(g, 0);
    const A2 = basePoint(g, 1);
    const colL = cssVar('--series-1', '#2563eb');
    const colR = cssVar('--series-4', '#d97706');
    const w = thin ? 3 : 8;
    c.line(A1, q.B1, { color: colL, width: w, alpha });
    c.line(q.B1, q.P, { color: colL, width: w * 0.75, alpha });
    c.line(A2, q.B2, { color: colR, width: w, alpha });
    c.line(q.B2, q.P, { color: colR, width: w * 0.75, alpha });
    if (thin) return;
    motorSymbol(c, A1);
    motorSymbol(c, A2);
    if (redundant) motorSymbol(c, q.B1); else passiveSymbol(c, q.B1);
    passiveSymbol(c, q.B2);
    passiveSymbol(c, q.P);
  }

  function drawEllipse(c: Canvas2D): void {
    const accent = cssVar('--accent', '#2563eb');
    const bg = cssVar('--bg-elev', '#fff');
    const e = velocityEllipse(inverseRates(g, cfg, redundant));
    const big = e.r[0] * VEL_SCALE;
    const small = e.r[1] * VEL_SCALE;
    const capped = !(big <= CAP);
    const a = Math.min(CAP, big);
    const b = Math.min(CAP, small);
    const ang = Math.atan2(e.dirs[0][1], e.dirs[0][0]);
    c.ellipse(cfg.P, a, Math.max(b, 0.004), ang, { fill: accent, alpha: 0.13, width: 0 });
    c.ellipse(cfg.P, a, Math.max(b, 0.004), ang, { color: accent, width: 2, dash: capped ? [7, 5] : undefined });
    if (capped) {
      const tip: Pt = [cfg.P[0] + e.dirs[0][0] * a, cfg.P[1] + e.dirs[0][1] * a];
      labelInside(c, tip, '橢圓 → ∞', { color: accent, bg, font: 'bold 12px sans-serif', dy: -12 });
    }
  }

  function drawLostDirection(c: Canvas2D): void {
    const ind = indicators(g, cfg);
    if (Math.min(Math.abs(ind.s1), Math.abs(ind.s2)) > 0.03) return;
    const danger = cssVar('--danger', '#b91c1c');
    const e = velocityEllipse(inverseRates(g, cfg, redundant));
    const u = e.dirs[1];
    const L = 0.9;
    c.line([cfg.P[0] - u[0] * L, cfg.P[1] - u[1] * L], [cfg.P[0] + u[0] * L, cfg.P[1] + u[1] * L], { color: danger, width: 2, dash: [7, 5] });
    const out = u[0] * cfg.P[0] + u[1] * (cfg.P[1] - 0.5) >= 0 ? 1 : -1;
    labelInside(c, [cfg.P[0] + out * u[0] * (L + 0.12), cfg.P[1] + out * u[1] * (L + 0.12)], '✗ 這個方向動不了', { color: danger, bg: cssVar('--bg-elev', '#fff'), font: 'bold 12px sans-serif' });
  }

  function drawFreeDirection(c: Canvas2D): void {
    if (redundant) return;
    const sd = softestDirection(g, cfg, false);
    if (sd.sigma > 0.03 && !locked) return;
    const col = cssVar('--series-5', '#7c3aed');
    const strong = sd.sigma <= 0.03;
    const L = strong ? 0.55 : 0.35;
    const n = sd.n;
    c.arrow(cfg.P, [cfg.P[0] + n[0] * L, cfg.P[1] + n[1] * L], { color: col, width: strong ? 3 : 1.5, head: 11, alpha: strong ? 1 : 0.6 });
    c.arrow(cfg.P, [cfg.P[0] - n[0] * L, cfg.P[1] - n[1] * L], { color: col, width: strong ? 3 : 1.5, head: 11, alpha: strong ? 1 : 0.6 });
    if (strong) labelInside(c, [cfg.P[0] + n[0] * (L + 0.1), cfg.P[1] + n[1] * (L + 0.1)], '鎖住馬達仍可微動', { color: col, bg: cssVar('--bg-elev', '#fff'), font: 'bold 12px sans-serif', dy: -6 });
  }

  function drawVirtual(c: Canvas2D): void {
    if (!locked || !virt) return;
    const ok = cssVar('--ok', '#15803d');
    const danger = cssVar('--danger', '#b91c1c');
    const bg = cssVar('--bg-elev', '#fff');
    const lt = lockStretch(g, cfg, virt, redundant);
    const tol = 0.002; // 2 mm
    if (lt.rigidLeft) {
      c.line(cfg.P, virt, { color: danger, width: 2, dash: [4, 3] });
    } else {
      c.line(cfg.B1, virt, { color: Math.abs(lt.e1) < tol ? ok : danger, width: 4 });
    }
    c.line(cfg.B2, virt, { color: Math.abs(lt.e2) < tol ? ok : danger, width: 4 });
    c.point(virt, 5, { color: cssVar('--fg', '#222') });
    const f1 = (x: number) => `${x >= 0 ? '+' : ''}${fmt(1000 * x, 1)} mm`;
    labelInside(c, virt, lt.rigidLeft ? `左腿鎖死：偏離 ${f1(lt.e1)}；e₂ ${f1(lt.e2)}` : `e₁ ${f1(lt.e1)}，e₂ ${f1(lt.e2)}`, { color: cssVar('--fg', '#222'), bg, dy: -22, font: '12px sans-serif' });
  }

  function drawInset(c: Canvas2D): void {
    if (!show.inset || redundant) return;
    const purple = cssVar('--series-5', '#7c3aed');
    const muted = cssVar('--fg-muted', '#777');
    const bg = cssVar('--bg-elev', '#fff');
    const q = insetGeom();
    const [x0, y0] = q.c;
    c.polygon([[x0 - q.P, y0 - q.P], [x0 + q.P, y0 - q.P], [x0 + q.P, y0 + q.P], [x0 - q.P, y0 + q.P]], { fill: bg, color: cssVar('--border', '#ddd'), width: 1, alpha: 0.95 });
    c.line([x0 - q.R * 1.2, y0], [x0 + q.R * 1.2, y0], { color: muted, width: 1 });
    c.line([x0, y0 - q.R * 1.2], [x0, y0 + q.R * 1.2], { color: muted, width: 1 });
    c.circle([x0, y0], q.R, { color: cssVar('--fg', '#222'), width: 1.5 });
    c.text([x0 + q.R * 1.2, y0], 'θ̇₁', { color: muted, align: 'right', dy: -9 });
    c.text([x0, y0 + q.R * 1.2], 'θ̇₂', { color: muted, align: 'left', dx: 4, dy: 2 });
    c.text([x0, y0 - q.P], '‖θ̇_a‖ = 1', { color: muted, dy: -9, font: '11px sans-serif' });
    // the corresponding tip velocity
    const rates: [number, number] = [Math.cos(rateDir), Math.sin(rateDir)];
    const v = tipVelocity(g, cfg, rates);
    if (v) {
      const n = Math.hypot(v[0], v[1]);
      const len = Math.min(CAP, n * VEL_SCALE);
      if (n > 1e-9) {
        const head: Pt = [cfg.P[0] + (v[0] / n) * len, cfg.P[1] + (v[1] / n) * len];
        c.arrow(cfg.P, head, { color: purple, width: 3, head: 12 });
        labelInside(c, head, n * VEL_SCALE > CAP ? 'Jθ̇（很大）' : 'Jθ̇', { color: purple, dx: 14 * Math.sign(v[0] || 1), dy: -10, bg, font: 'bold 12px sans-serif' });
      }
    } else {
      labelInside(c, cfg.P, 'Jθ̇ 發散（H_p 不可逆）', { color: purple, dy: 26, bg, font: 'bold 12px sans-serif' });
    }
  }

  function drawLoad(c: Canvas2D): void {
    if (!show.load) return;
    const danger = cssVar('--danger', '#b91c1c');
    const f: Pt = [Math.cos(loadDir), Math.sin(loadDir)];
    const head: Pt = [cfg.P[0] + f[0] * FORCE_LEN, cfg.P[1] + f[1] * FORCE_LEN];
    c.arrow(cfg.P, head, { color: danger, width: 3, head: 12 });
    labelInside(c, head, 'f = 1 N', { color: danger, dx: 16 * f[0], dy: -16 * f[1], bg: cssVar('--bg-elev', '#fff'), font: 'bold 12px sans-serif' });
  }

  cv.draw = (c) => {
    hM1.color = cssVar('--series-1', '#2563eb');
    hM2.color = cssVar('--series-4', '#d97706');
    hTip.color = cssVar('--accent', '#2563eb');
    hRate.color = cssVar('--series-5', '#7c3aed');
    c.grid(0.5);
    drawGround(c);
    if (trail.length > 1) c.polyline(trail, { color: cssVar('--series-5', '#7c3aed'), width: 2, alpha: 0.8 });
    if (show.ellipse) drawEllipse(c);
    if (locked && virt) drawMechanism(c, cfg, 0.25, true);
    drawMechanism(c, cfg, locked && virt ? 0.55 : 1);
    drawLostDirection(c);
    drawFreeDirection(c);
    drawVirtual(c);
    drawLoad(c);
    drawInset(c);
    const ind = indicators(g, cfg);
    c.overlay.textContent = `θ = (${deg(cfg.th1, 0)}, ${deg(cfg.th2, 0)})  detH_p/(L₂L₄) = ${redundant ? '—' : fmt(ind.act, 3)}  sinφ = (${fmt(ind.s1, 2)}, ${fmt(ind.s2, 2)})${locked ? '  🔒' : ''}`;
  };

  // ---------------------------------------------------------------- presets
  const put = (c: Config | null, opts: { lock?: boolean; red?: boolean; load?: number; text?: string } = {}) => () => {
    stopSnapState();
    snap = null;
    setRedundant(opts.red ?? false);
    setLocked(opts.lock ?? false);
    setCfg(c ?? general());
    if (opts.load !== undefined) { loadDir = opts.load; sLoad.set(loadDir, { silent: true }); }
    msg = opts.text ?? '';
    update();
  };
  const leftStraight = () => straightLegConfig(g, 0, 60 * D);
  const rightStraight = () => straightLegConfig(g, 1, 112 * D);
  const nearStraight = () => fkNearest(g, 66 * D, 65 * D, general().P);
  const both = () => doubleSingularConfig(g);
  const chipSpecs: { label: string; apply: () => void }[][] = [
    [
      { label: '一般構型', apply: put(general()) },
      { label: '靠近遠端桿共線', apply: put(tangentCfg(0.02)) },
      { label: '靠近左腿伸直', apply: put(nearStraight()) },
    ],
    [
      { label: '一般構型', apply: put(general()) },
      { label: '另一組馬達角', apply: put(fkNearest(g, 100 * D, 40 * D, null, 1)) },
      { label: '靠近遠端桿共線', apply: put(tangentCfg(0.05)) },
    ],
    [
      { label: '左腿伸直', apply: put(leftStraight()) },
      { label: '右腿伸直', apply: put(rightStraight()) },
      { label: '接近伸直', apply: put(nearStraight()) },
    ],
    [
      { label: '遠端桿共線（鎖住馬達）', apply: put(tangentCfg(0), { lock: true }) },
      { label: '接近共線（鎖住）', apply: put(tangentCfg(0.05), { lock: true }) },
      { label: '一般構型（鎖住）', apply: put(general(), { lock: true }) },
      { label: '兩類同時發生', apply: put(both(), { text: '⚠ 左腿折疊（末端奇異）且兩根遠端桿共線（致動器奇異）：橢圓退化成一條沒有寬度、長度無限的線' }) },
    ],
    [
      {
        label: '▶ 穿越致動器奇異',
        apply: () => {
          put(tangentCfg(0.6))();
          startSnap();
          pb.setPlaying(true);
        },
      },
      { label: '負載：接近致動器奇異', apply: put(tangentCfg(0.005), { load: 90 * D }) },
      { label: '負載：左腿伸直', apply: put(leftStraight(), { load: 60 * D, text: '負載沿著伸直的左腿：由桿件拉壓承受，兩顆馬達幾乎不必出力' }) },
      { label: '負載：一般構型', apply: put(general(), { load: 90 * D }) },
    ],
    [
      { label: '兩顆馬達 @ 遠端桿共線', apply: put(tangentCfg(0.005), { load: 90 * D }) },
      { label: '三顆馬達 @ 遠端桿共線', apply: put(tangentCfg(0.005), { red: true, load: 90 * D }) },
      { label: '三顆馬達：鎖住後拖末端', apply: put(tangentCfg(0), { red: true, lock: true, load: 90 * D }) },
      { label: '三顆馬達：左腿伸直', apply: put(leftStraight(), { red: true, load: 60 * D }) },
    ],
  ];
  const chipGroups: PresetChipsHandle[] = chipSpecs.map((specs) => {
    const grp = presetChips(chipsRow, specs, { label: '預設：' });
    grp.el.style.display = 'none';
    return grp;
  });

  interface View { ellipse: boolean; inset: boolean; matrices: boolean; slice: boolean; bars: boolean; snap: boolean; load: boolean; chip: number }
  const VIEWS: View[] = [
    { ellipse: false, inset: false, matrices: true, slice: true, bars: false, snap: false, load: false, chip: 0 },
    { ellipse: true, inset: true, matrices: true, slice: false, bars: false, snap: false, load: false, chip: 0 },
    { ellipse: true, inset: true, matrices: false, slice: true, bars: false, snap: false, load: false, chip: 0 },
    { ellipse: true, inset: false, matrices: false, slice: true, bars: false, snap: false, load: false, chip: 0 },
    { ellipse: true, inset: false, matrices: false, slice: false, bars: true, snap: true, load: true, chip: 1 },
    { ellipse: true, inset: false, matrices: false, slice: true, bars: true, snap: false, load: true, chip: 1 },
  ];
  const focusFor = (i: number): HTMLElement[] => [
    [matWrap, plotSlice.el],
    [cv.el, matWrap],
    [cv.el, plotSlice.el],
    [cv.el, info.el],
    [snapRow, bars2.el, plotSnap.el],
    [tRed.el, bars3.el],
  ][i] ?? [];
  let focused: HTMLElement[] = [];

  const onStep = (i: number): void => {
    const k = Math.max(0, Math.min(steps.length - 1, i));
    step = k;
    stopSnapState();
    snap = null;
    chipGroups.forEach((grp, j) => { grp.el.style.display = j === k ? '' : 'none'; });
    const v = VIEWS[k];
    show.ellipse = v.ellipse;
    show.inset = v.inset;
    show.matrices = v.matrices;
    show.slice = v.slice;
    show.bars = v.bars;
    show.snap = v.snap;
    show.load = v.load;
    show.fd = false;
    tEll.set(v.ellipse, { silent: true });
    tFD.set(false, { silent: true });
    if (k === 4) {
      flip = true;
      tFlip.set(true, { silent: true });
      startSnap();
    }
    chipGroups[k].activate(v.chip);
    focused.forEach((f) => f.classList.remove('mr-focus'));
    focused = focusFor(k);
    focused.forEach((f) => f.classList.add('mr-focus'));
    syncUI();
    update();
  };

  // ---------------------------------------------------------------- start
  onStep(ctx.currentStep);

  return {
    onStep,
    destroy() {
      anim.stop();
      cv.destroy();
      plotSlice.destroy();
      plotSnap.destroy();
    },
  };
}

const lesson: Lesson = { meta, steps, quiz, mount };
export default lesson;
