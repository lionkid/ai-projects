import type { Lesson, LessonContext, LessonHandle, LessonStep, QuizQuestion } from '@/types';
import {
  el, cssVar, fmt, controlPanel, row, slider, angleSlider, toggle, button, select, simTitle, legend, kv, note,
  Canvas2D, RasterLayer, presetChips, poseHandle, createLoop,
  type Handle, type PresetChipsHandle, type TextStyle,
} from '@/ui';
import { wrapToPi } from '@/mr';
import { meta } from './meta';
import {
  DEFAULT_GEOM, DEFAULT_RPR, RPR_PRESETS, BRANCHES, basePoint, legLinks, elbowPoint, legPoints, loopResidual,
  ikConfig, ikAll, branchOf, fkAll, fkNearest, driveMotors, driveTip, tipShift, fkJacobianFD, sigmaMax,
  motorSweep, symmetricMarginAngle, rprIK, rprFKAll, rprJoints,
  type Branch, type Config, type ElbowSide, type FiveBarGeom, type Mode, type Pose2, type Pt, type Side,
} from './fiveBar';

const D = Math.PI / 180;
const DEFAULT_MOTORS: [number, number] = [115 * D, 65 * D];
const WORLD = { xmin: -2.6, xmax: 2.6, ymin: -0.8, ymax: 2.45 };
/** The 3xRPR solutions reach a little lower than the five-bar. */
const RPR_WORLD = { xmin: -2.6, xmax: 2.6, ymin: -1.1, ymax: 2.5 };
/** The tip shift caused by a 1° motor step is drawn this many times longer. */
const SENS_SCALE = 10;
/** Motor handles sit on the proximal links at this fraction of their length. */
const HANDLE_FRAC = 0.62;

type ViewMode = 'fk' | 'ik' | 'open' | 'rpr';

const steps: LessonStep[] = [
  {
    title: '哪些關節是馬達，哪些是被動的',
    md: `平面五連桿（five-bar）由四根活動連桿加上地桿首尾相接，圍成**一個迴路**，共有五個旋轉關節。只有底座的兩個關節裝了馬達，稱為**致動關節（actuated joint）**；兩個肘關節與末端關節沒有馬達，是**被動關節（passive joint）**，它們的角度完全由迴路決定。

用 Grübler 公式數自由度（$N = 5$ 根連桿含地桿，$J = 5$ 個各有 1 自由度的關節）：

$$\\text{dof} = 3(N - 1 - J) + \\sum_{i=1}^{J} f_i = 3(5-1-5) + 5 = 2 .$$

兩個自由度剛好交給兩顆馬達。像這樣由好幾條「腿」從地面一起撐住末端的機構，就叫**並聯機構（parallel mechanism）**。`,
    simHint: '拖曳近端桿上的圓形把手（或拉 θ₁、θ₂ 滑桿）：只有兩個實心方塊是馬達，三個空心關節自己跟著動。再按「鎖住 θ₁」：只剩一個自由度，末端只能沿虛線走。',
  },
  {
    title: '迴路閉合方程式',
    md: `在末端關節處把迴路「剪開」，五連桿就成了兩條從底座出發的 2R 開鏈：左腿由 $(\\theta_1, \\phi_1)$ 決定端點 $p_L$，右腿由 $(\\theta_2, \\phi_2)$ 決定端點 $p_R$。機構組得起來，兩個端點就必須重合：

$$g(\\theta_a, \\theta_p) = p_L(\\theta_1, \\phi_1) - p_R(\\theta_2, \\phi_2) = 0 .$$

這就是**迴路閉合方程式（loop-closure equation）**，其中 $\\theta_a = (\\theta_1, \\theta_2)$ 是致動角、$\\theta_p = (\\phi_1, \\phi_2)$ 是被動角：兩條純量方程、四個關節角。指定兩個馬達角之後，兩個被動角便跟著確定，末端關節角 $\\psi$ 也一樣。`,
    simHint: '按「拆開迴路：自己對準」：四個角都能自由設定，紅色箭頭就是殘差 g。試著只拖兩條腿的末端（調 φ₁、φ₂）讓 ‖g‖ 歸零，再按「自動閉合」對照。',
  },
  {
    title: 'IK：拆成兩個 2R',
    md: `反向運動學（IK）是「給定末端位置 $p$，求馬達角」。末端正是兩條腿的接點，所以 $p$ 一旦給定，兩條腿就互不相干，各自是一個有解析解的 2R 問題，例如左腿：

$$\\cos\\phi_1 = \\frac{\\|p - A_1\\|^2 - L_1^2 - L_2^2}{2L_1L_2}.$$

每條腿有「肘朝外」與「肘朝內」兩個解，組合起來最多 $2\\times 2 = 4$ 組分支，而且左腿的答案完全不受右腿影響。只要 $p$ 同時落在兩條腿的可達環形區內（淡藍色區域），IK 就有封閉解。`,
    simHint: '拖曳藍色的末端把手：淡色是另外三組分支。用上方四個按鈕切換「左外・右內」等組合；把末端拖出淡藍色區域，看哪條腿先伸直。',
  },
  {
    title: 'FK：兩圓的交點',
    md: `正向運動學（FK）方向相反：給定 $\\theta_1, \\theta_2$，兩個肘點 $B_1, B_2$ 立刻確定，末端則必須同時滿足

$$\\|p - B_1\\| = L_2, \\qquad \\|p - B_2\\| = L_4 ,$$

也就是兩個圓的交點。一般有兩個交點，對應兩種**裝配模式（assembly mode）**：末端位於 $B_1B_2$ 連線的一側或另一側。兩者都符合同一組馬達角；機構實際停在哪一個，取決於它從哪裡連續動過來，所以模擬中轉動馬達時，一律選離上一個位置最近的交點。兩圓不相交時，這組馬達角**無法裝配**。`,
    simHint: '轉動馬達，兩個虛線圓會跟著移動：實心交點是目前的裝配模式，空心交點與淡色機構是另一個。試試「另一個裝配模式」與「無法裝配的馬達角」。',
  },
  {
    title: '解會合併、會翻面',
    md: `把兩個肘點越拉越開，兩個交點會互相靠近；當 $\\|B_1 - B_2\\| = L_2 + L_4$ 時兩圓相切，兩個 FK 解合而為一，兩根遠端桿排成一直線。這正是「可裝配馬達角」的邊界，再往外就沒有解。

接近相切時，交點離 $B_1B_2$ 連線的距離約為 $\\sqrt{L\\,\\delta}$（$\\delta$ 是剩下的間隙，$L$ 為遠端桿長），它對 $\\delta$ 的變化率趨近無限大：馬達只轉 1°，末端可能移動好幾公分。這種馬達幾乎管不住末端的構型，就是下一課要正式定義的奇異點。`,
    simHint: '依序按「遠離相切」「接近相切」「正好相切」，比較兩支 1° 位移箭頭與下方的位移讀數；再把 θ₁ 往左拉，馬達會停在相切邊界。',
  },
  {
    title: '3×RPR、Stewart–Gough 與下一課',
    md: `「IK 容易、FK 困難」是並聯機構的共同特徵。平面 **3×RPR** 平台用三條可伸縮的腿撐住一塊三角板，中間的移動關節（P）是馬達：給定平台位姿，每條腿長只是一個距離公式；反過來由三個腿長求位姿，消去變數後得到六次多項式，最多 **6** 組實數解。空間中的 **Stewart–Gough 平台**有 6 條腿，IK 仍是 6 個距離公式，FK 卻最多有 **40** 組解。

下一課把迴路閉合方程式微分，區分兩種奇異：遠端桿共線的**致動器奇異**，與一條腿伸直的**末端奇異**。`,
    simHint: '拖動平台中心或外側的轉向把手：三個腿長立刻更新（IK），淡色虛線是同一組腿長的其他 FK 解。按「6 組解」看極端情況；最後兩個按鈕預覽下一課的兩種奇異。',
  },
];

const quiz: QuizQuestion[] = [
  {
    question: 'Stewart–Gough 平台有 6 條可伸縮的腿。為什麼常說它的 IK 比 FK 容易？',
    options: [
      '給定平台位姿，每條腿長都是一個獨立的距離公式；反過來給 6 個腿長，必須聯立 6 條耦合的非線性方程，最多可有 40 組解',
      'IK 永遠只有唯一解，而 FK 一定無解',
      '6 條腿彼此獨立，所以 FK 也只是 6 個互不相關的公式',
      '因為平台只有 3 個自由度，比腿的數目少',
    ],
    answer: 0,
    explanation: '給定位姿 $(R, p)$，第 $i$ 條腿長就是 $\\|p + R\\,b_i - a_i\\|$，六條腿各算各的，就像五連桿的 IK 拆成兩個 2R。反過來，六個腿長同時約束同一個位姿，必須聯立求解；這個多項式系統最多有 40 個（複數）解，實數解也常不只一個。平台有 6 個自由度，正好由 6 條腿驅動。',
  },
  {
    question: '把五連桿右底座的馬達拆掉，改裝在左肘，也就是改由 $\\theta_1$ 與 $\\phi_1$ 驅動。下列哪個說法正確？',
    options: [
      'FK 變簡單了：左腿成為兩個關節都有馬達的 2R 開鏈，用 2R 正向運動學就得到唯一的末端位置',
      'FK 仍然是兩圓求交點，而且一樣有兩種裝配模式',
      '機構的自由度變成 1',
      'IK 從此無解',
    ],
    answer: 0,
    explanation: '馬達換位置不改變 Grübler 算出的 dof（仍是 2），卻改變了哪個問題容易。左腿兩個關節都被指定後就是普通的 2R 開鏈，末端位置唯一確定；右腿兩個關節都是被動的，只要末端還在它的可達範圍內，它就只是跟著動、不再限制末端。IK 也還能解：只用左腿的 2R IK（兩組解）。可見「IK 易、FK 難」取決於馬達裝在哪裡。',
  },
  {
    question: '五連桿的兩個 FK 解在什麼時候合併成一個？',
    options: [
      '兩肘點距離 $\\|B_1 - B_2\\|$ 等於 $L_2 + L_4$（或 $|L_2 - L_4|$），兩圓相切，兩根遠端桿共線',
      '兩個馬達角相等時',
      '末端位於兩個底座正中間時',
      '左腿完全伸直時',
    ],
    answer: 0,
    explanation: 'FK 的兩個解是兩個圓的交點，只有兩圓相切時才會重合，這時 $B_1$、$p$、$B_2$ 在同一直線上。這個構型位於可裝配馬達角的邊界：再往外就無解，而在它附近末端對馬達角極度敏感。左腿伸直是另一回事：那是 IK 可達區的邊界，下一課會看到它屬於另一類奇異。',
  },
  {
    question: '在 IK 模式中，把末端拖到距離左底座正好 $L_1 + L_2$ 的位置（右腿仍可彎曲）。模擬中會剩幾組彼此不同的分支？',
    options: ['4 組', '2 組', '1 組', '0 組'],
    answer: 1,
    explanation: '左腿完全伸直時，「肘朝外」與「肘朝內」是同一個姿勢，左腿只剩 1 個解；右腿仍有 2 個。不同的組合從 $2\\times 2 = 4$ 變成 $1\\times 2 = 2$。IK 的分支是每條腿分開計數再相乘，這正是「拆成兩個獨立 2R」的結果。',
  },
];

const sideName = (s: ElbowSide): string => (s === 1 ? '外' : '內');
const branchName = (b: Branch): string => `左${sideName(b[0])}・右${sideName(b[1])}`;
const deg = (x: number, d = 1): string => `${fmt(x / D, d)}°`;
const sci = (x: number): string => (x === 0 ? '0' : x < 1e-3 ? x.toExponential(1) : fmt(x, 3));
const centroid = (pts: Pt[]): Pt => [pts.reduce((s, p) => s + p[0], 0) / pts.length, pts.reduce((s, p) => s + p[1], 0) / pts.length];

function mount(host: HTMLElement, ctx: LessonContext): LessonHandle {
  // ---------------------------------------------------------------- state
  let g: FiveBarGeom = { ...DEFAULT_GEOM };
  let mode: ViewMode = 'fk';
  let cfg: Config | null = null;
  /** Motor angles shown while the mechanism cannot be assembled (cfg === null). */
  let badMotors: [number, number] = [DEFAULT_MOTORS[0], DEFAULT_MOTORS[1]];
  let lastTip: Pt | null = null;
  let branch: Branch = [1, 1];
  const open = { th1: 0, th2: 0, phi1: 0, phi2: 0 };
  let msg = '';
  let lockTh1 = false;
  let sweepPath: Pt[] = [];
  let shifts: [Pt | null, Pt | null] = [null, null];
  let step = -1;
  const show = { ghost: false, circles: false, loop: false, sens: false, workspace: false, labels: true };
  let rprSols: Pose2[] = [];
  let tween: { from: [number, number]; to: [number, number]; t: number } | null = null;

  // ---------------------------------------------------------------- layout
  simTitle(host, '平面五連桿：IK 與 FK', 'closed chain');
  const cv = new Canvas2D(host, { aspect: 1.7, world: WORLD, overlay: true });
  const status = note(host, '');
  status.style.minHeight = '1.2em';
  const legFive = legend(host, [
    { color: 'var(--accent)', label: '馬達（致動關節，實心方塊）' },
    { color: 'var(--fg-muted)', label: '被動關節（空心圓）' },
    { color: 'var(--series-1)', label: '左腿 L₁、L₂' },
    { color: 'var(--series-4)', label: '右腿 L₃、L₄' },
  ]);
  const legRpr = legend(host, [
    { color: 'var(--accent)', label: '移動關節 P：馬達（腿長 ℓᵢ）' },
    { color: 'var(--fg-muted)', label: '旋轉關節 R：被動' },
    { color: 'var(--series-2)', label: '彩色虛線：同一組腿長的其他 FK 解（#編號）' },
  ]);

  const panel = controlPanel(host, { compact: true });
  const chipsRow = row(panel);
  const modeRow = row(panel);
  const selMode = select<ViewMode>(modeRow, {
    value: 'fk',
    options: [
      { value: 'fk', label: 'FK：轉馬達' },
      { value: 'ik', label: 'IK：拖末端' },
      { value: 'open', label: '拆開迴路' },
      { value: 'rpr', label: '3×RPR 平台' },
    ],
    onChange: (v) => { clearChips(); setMode(v); update(); },
  });
  const flipBtn = button(modeRow, { label: '⇅ 切換裝配模式', small: true, onClick: () => { clearChips(); flipAssembly(); } });
  const closeBtn = button(modeRow, { label: '⟳ 自動閉合（解 φ₁, φ₂）', small: true, onClick: () => autoClose() });
  const tLock = toggle(modeRow, { label: '鎖住 θ₁', value: false, onChange: (v) => { clearChips(); setLock(v); update(); } });
  const nextSolBtn = button(modeRow, { label: '⟳ 換到下一個 FK 解（腿長不變）', small: true, onClick: () => { clearChips(); nextRprSolution(); } });
  const s1 = angleSlider(panel, { label: 'θ₁（左馬達）', value: DEFAULT_MOTORS[0], onInput: (v) => { clearChips(); onMotorInput(0, v); } });
  const s2 = angleSlider(panel, { label: 'θ₂（右馬達）', value: DEFAULT_MOTORS[1], onInput: (v) => { clearChips(); onMotorInput(1, v); } });
  const sp1 = angleSlider(panel, { label: 'φ₁（左肘，被動）', value: 0, onInput: (v) => { stopTween(); open.phi1 = v; msg = ''; update(); } });
  const sp2 = angleSlider(panel, { label: 'φ₂（右肘，被動）', value: 0, onInput: (v) => { stopTween(); open.phi2 = v; msg = ''; update(); } });
  const optRow = row(panel);
  const tGhost = toggle(optRow, { label: '其他解', onChange: (v) => { show.ghost = v; update(); } });
  const tCircles = toggle(optRow, { label: 'FK 兩圓', onChange: (v) => { show.circles = v; update(); } });
  const tLoop = toggle(optRow, { label: '迴路向量', onChange: (v) => { show.loop = v; update(); } });
  const tSens = toggle(optRow, { label: '1° 箭頭 ×10', onChange: (v) => { show.sens = v; update(); } });
  const tWs = toggle(optRow, { label: '工作空間', onChange: (v) => { show.workspace = v; update(); } });

  const geoBox = el('details', { class: 'mr-note' }, el('summary', { style: { cursor: 'pointer' } }, '調整桿長與底座間距'));
  host.appendChild(geoBox);
  const geoPanel = controlPanel(geoBox, { compact: true });
  const sProx = slider(geoPanel, { label: '近端桿 L₁ = L₃', min: 0.6, max: 1.4, step: 0.05, value: g.L1, unit: 'm', onInput: (v) => setGeom({ L1: v, L3: v }) });
  const sDist = slider(geoPanel, { label: '遠端桿 L₂ = L₄', min: 0.8, max: 1.8, step: 0.05, value: g.L2, unit: 'm', onInput: (v) => setGeom({ L2: v, L4: v }) });
  const sBase = slider(geoPanel, { label: '底座間距 d（地桿）', min: 0.4, max: 1.6, step: 0.05, value: g.d, unit: 'm', onInput: (v) => setGeom({ d: v }) });

  const info = el('div');
  host.appendChild(info);
  const gWrap = el('div', { style: { marginBottom: '0.5rem' } });
  const fWrap = el('div');
  info.append(gWrap, fWrap);
  const K = {
    q: '致動 θ₁, θ₂ ｜ 被動 φ₁, φ₂, ψ', tip: '末端 p (m)', res: '迴路殘差 ‖g‖', sol: '解的數目', sens: '馬達 +1° → 末端移動',
  };
  const fKV = kv(fWrap, Object.values(K));
  const GK = { nj: 'Grübler（平面 m = 3）', dof: 'dof = 3(N−1−J) + Σfᵢ' };
  const gKV = kv(gWrap, Object.values(GK));
  const RK = { len: '腿長 ℓ₁, ℓ₂, ℓ₃（IK）', pose: '平台位姿 (x, y, θ)', n: 'FK：同一組腿長的實數解', sg: '6 腿 Stewart–Gough' };
  const rKV = kv(host, Object.values(RK));

  // ---------------------------------------------------------------- workspace raster (repainted when the lengths change)
  const ws = new RasterLayer({ xmin: -2.8, xmax: 2.8, ymin: -2.6, ymax: 2.8 }, 224, 216);
  let wsKey = '';
  const inAnnulus = (side: Side, x: number, y: number): boolean => {
    const A = basePoint(g, side);
    const [Lp, Ld] = legLinks(g, side);
    const r = Math.hypot(x - A[0], y - A[1]);
    return r <= Lp + Ld && r >= Math.abs(Lp - Ld);
  };
  function ensureWorkspace(): void {
    const key = `${g.L1}|${g.L2}|${g.L3}|${g.L4}|${g.d}`;
    if (key === wsKey) return;
    wsKey = key;
    ws.paint((x, y) => inAnnulus(0, x, y) && inAnnulus(1, x, y), { color: [37, 99, 235, 46] });
  }

  // ---------------------------------------------------------------- helpers on the state
  const currentMotors = (): [number, number] => {
    if (mode === 'open') return [open.th1, open.th2];
    return cfg ? [cfg.th1, cfg.th2] : [badMotors[0], badMotors[1]];
  };

  function setCfg(c: Config): void {
    cfg = c;
    lastTip = [c.P[0], c.P[1]];
    if (lockTh1) sweepPath = motorSweep(g, c, 1, 0.01).configs.map((q) => q.P);
  }

  function presetMotors(th1: number, th2: number, prefer: Mode = 1): void {
    const c = fkNearest(g, th1, th2, null, prefer);
    if (c) { setCfg(c); msg = ''; }
    else { cfg = null; badMotors = [th1, th2]; msg = '✕ 無法裝配：兩個圓不相交，找不到同時滿足兩條約束的末端位置'; }
  }

  function resetGeom(): void {
    g = { ...DEFAULT_GEOM };
    sProx.set(g.L1, { silent: true });
    sDist.set(g.L2, { silent: true });
    sBase.set(g.d, { silent: true });
    wsKey = '';
  }

  /** Default pose for the current lengths (falls back to the default lengths if needed). */
  function resetConfig(): void {
    let c = fkNearest(g, DEFAULT_MOTORS[0], DEFAULT_MOTORS[1], null, 1);
    for (const t2 of [50, 80, 35, 95, 20, 110].map((a) => a * D)) {
      if (c) break;
      c = fkNearest(g, Math.PI - t2, t2, null, 1);
    }
    if (!c) { resetGeom(); c = fkNearest(g, DEFAULT_MOTORS[0], DEFAULT_MOTORS[1], null, 1); }
    setCfg(c!);
    msg = '';
  }

  /** Symmetric pose with a given gap to tangency (resets the lengths when that is impossible). */
  function presetMargin(margin: number, prefer: Mode): void {
    let t2 = symmetricMarginAngle(g, margin);
    if (t2 === null) { resetGeom(); t2 = symmetricMarginAngle(g, margin)!; }
    presetMotors(Math.PI - t2, t2, prefer);
  }

  function legEdgeMessage(c: Config): string {
    const out: string[] = [];
    ([0, 1] as Side[]).forEach((side) => {
      const A = basePoint(g, side);
      const [Lp, Ld] = legLinks(g, side);
      const r = Math.hypot(c.P[0] - A[0], c.P[1] - A[1]);
      const name = side === 0 ? '左腿' : '右腿';
      if (Math.abs(r - (Lp + Ld)) < 1e-4) out.push(`${name}已完全伸直`);
      else if (Math.abs(r - Math.abs(Lp - Ld)) < 1e-4) out.push(`${name}已完全折疊`);
    });
    return out.length ? `⚠ ${out.join('、')}：末端到達工作空間邊界，再往外 IK 無解` : '';
  }

  function clampToView(p: Pt): Pt {
    const v = cv.visible();
    const m = 0.08;
    return [Math.min(v.xmax - m, Math.max(v.xmin + m, p[0])), Math.min(v.ymax - m, Math.max(v.ymin + m, p[1]))];
  }

  // ---------------------------------------------------------------- actions
  function requestMotors(th1: number, th2: number): void {
    if (cfg) {
      const res = driveMotors(g, cfg, th1, th2);
      setCfg(res.cfg);
      msg = res.blocked ? '⚠ 已到兩圓相切的邊界（兩根遠端桿共線）：再轉就無法裝配，馬達停在這裡' : '';
    } else {
      const c = fkNearest(g, th1, th2, lastTip, 1);
      if (c) { setCfg(c); msg = ''; }
      else { badMotors = [th1, th2]; msg = '✕ 無法裝配：兩個圓不相交'; }
    }
    update();
  }

  function onMotorInput(side: Side, th: number): void {
    if (mode === 'open') {
      stopTween();
      if (side === 0) open.th1 = th; else open.th2 = th;
      msg = '';
      update();
      return;
    }
    if (mode !== 'fk') return;
    if (side === 0 && lockTh1) return;
    const [a, b] = currentMotors();
    requestMotors(side === 0 ? th : a, side === 1 ? th : b);
  }

  function requestTip(p: Pt): void {
    const q = clampToView(p);
    if (!cfg) {
      const c = ikConfig(g, q, branch);
      if (c) { setCfg(c); msg = ''; }
      update();
      return;
    }
    const res = driveTip(g, cfg, q, branch);
    setCfg(res.cfg);
    msg = res.blocked ? legEdgeMessage(res.cfg) || '⚠ 末端到達工作空間邊界' : '';
    update();
  }

  function setBranch(b: Branch): void {
    branch = [b[0], b[1]];
    if (cfg) {
      const c = ikConfig(g, cfg.P, branch);
      if (c) setCfg(c);
    }
    msg = '';
    update();
  }

  function flipAssembly(): void {
    if (mode !== 'fk' || !cfg) return;
    const cur = cfg;
    const res = fkAll(g, cur.th1, cur.th2);
    const other = res.solutions.find((s) => s.mode !== cur.mode);
    if (other) { setCfg(other); msg = '同一組馬達角的另一個裝配模式'; }
    else msg = res.status === 'tangent' ? '兩圓相切：只剩一個解，沒有另一個模式可切換' : '';
    update();
  }

  function setLock(v: boolean): void {
    lockTh1 = v && mode === 'fk';
    tLock.set(lockTh1, { silent: true });
    sweepPath = lockTh1 && cfg ? motorSweep(g, cfg, 1, 0.01).configs.map((q) => q.P) : [];
    syncUI();
  }

  function setGeom(p: Partial<FiveBarGeom>): void {
    clearChips();
    stopTween();
    g = { ...g, ...p };
    wsKey = '';
    if (mode === 'rpr' || mode === 'open') { update(); return; }
    if (mode === 'ik' && cfg) {
      const c = ikConfig(g, cfg.P, branch);
      if (c) { setCfg(c); msg = ''; update(); return; }
    }
    const [a, b] = currentMotors();
    const c = fkNearest(g, a, b, lastTip, cfg?.mode ?? 1);
    if (c) { setCfg(c); msg = mode === 'ik' ? '末端不在新尺寸的工作空間內：改用目前的馬達角重新裝配' : ''; }
    else { cfg = null; badMotors = [a, b]; msg = '✕ 這組馬達角在新尺寸下無法裝配（兩個圓不相交）'; }
    update();
  }

  function enterOpen(offset: [number, number] = [0, 0]): void {
    if (!cfg) resetConfig();
    const c = cfg!;
    open.th1 = c.th1;
    open.th2 = c.th2;
    open.phi1 = wrapToPi(c.phi1 + offset[0]);
    open.phi2 = wrapToPi(c.phi2 + offset[1]);
  }

  function openEnds(): { L: ReturnType<typeof legPoints>; R: ReturnType<typeof legPoints>; gap: Pt } {
    const L = legPoints(g, 0, open.th1, open.phi1);
    const R = legPoints(g, 1, open.th2, open.phi2);
    const gap = loopResidual(g, open);
    return { L, R, gap };
  }

  function autoClose(): void {
    if (mode !== 'open') return;
    const { L, R } = openEnds();
    const mid: Pt = [(L.tip[0] + R.tip[0]) / 2, (L.tip[1] + R.tip[1]) / 2];
    const c = fkNearest(g, open.th1, open.th2, mid);
    if (!c) { msg = '✕ 這組馬達角無法閉合：兩個圓不相交'; update(); return; }
    tween = { from: [open.phi1, open.phi2], to: [c.phi1, c.phi2], t: 0 };
    msg = '只改被動角 φ₁、φ₂，馬達角不動……';
    if (!anim.running) anim.start();
    update();
  }

  function stopTween(): void {
    tween = null;
    if (anim.running) anim.stop();
  }

  const anim = createLoop((dt) => {
    if (!tween) { anim.stop(); return; }
    tween.t = Math.min(1, tween.t + dt / 0.8);
    const s = 0.5 - 0.5 * Math.cos(Math.PI * tween.t);
    open.phi1 = wrapToPi(tween.from[0] + s * wrapToPi(tween.to[0] - tween.from[0]));
    open.phi2 = wrapToPi(tween.from[1] + s * wrapToPi(tween.to[1] - tween.from[1]));
    if (tween.t >= 1) {
      tween = null;
      anim.stop();
      msg = '✓ 迴路閉合：馬達角給定後，被動角由閉合方程式決定';
    }
    update();
  });

  function setMode(m: ViewMode): void {
    stopTween();
    const prev = mode;
    if (m !== prev) {
      mode = m;
      if (m === 'ik') { if (!cfg) resetConfig(); branch = branchOf(cfg!); }
      if (m === 'open') enterOpen();
      if (m === 'fk' && prev === 'open') {
        const { L, R } = openEnds();
        const mid: Pt = [(L.tip[0] + R.tip[0]) / 2, (L.tip[1] + R.tip[1]) / 2];
        const c = fkNearest(g, open.th1, open.th2, mid);
        if (c) setCfg(c); else { cfg = null; badMotors = [open.th1, open.th2]; }
      }
      if (m === 'rpr') updateRpr();
      cv.setWorld(m === 'rpr' ? RPR_WORLD : WORLD);
      msg = '';
    }
    if (m !== 'fk') lockTh1 = false;
    selMode.set(m, { silent: true });
    tLock.set(lockTh1, { silent: true });
    syncUI();
  }

  // ---------------------------------------------------------------- 3xRPR platform
  const ph = poseHandle(cv, { ...RPR_PRESETS.six }, {
    knob: 0.62,
    frameSize: 0.3,
    enabled: false,
    onChange: (p) => {
      clearChips();
      msg = '';
      const q = clampToView([p.x, p.y]);
      p.x = q[0];
      p.y = q[1];
      updateRpr();
      update();
    },
  });
  const isSame = (a: Pose2, b: Pose2): boolean =>
    Math.hypot(a.x - b.x, a.y - b.y) < 1e-6 && Math.abs(wrapToPi(a.theta - b.theta)) < 1e-6;
  function updateRpr(): void {
    const pose = ph.pose;
    rprSols = rprFKAll(DEFAULT_RPR, rprIK(DEFAULT_RPR, pose)).sort((a, b) => a.theta - b.theta);
    if (!rprSols.some((s) => isSame(s, pose))) rprSols.push({ ...pose });
  }
  function nextRprSolution(): void {
    const pose = ph.pose;
    const i = rprSols.findIndex((s) => isSame(s, pose));
    const nxt = rprSols[(i + 1) % rprSols.length];
    if (nxt) ph.set({ ...nxt }, { silent: true });
    msg = `換到 #${((i + 1) % rprSols.length) + 1}：三個腿長完全相同，平台卻在另一個位姿`;
    update();
  }
  function setRprPose(p: Pose2): void {
    ph.set({ ...p }, { silent: true });
    updateRpr();
  }

  // ---------------------------------------------------------------- handles
  const motorHandlePos = (side: Side): Pt => {
    const A = basePoint(g, side);
    const B = elbowPoint(g, side, currentMotors()[side]);
    return [A[0] + HANDLE_FRAC * (B[0] - A[0]), A[1] + HANDLE_FRAC * (B[1] - A[1])];
  };
  const mkMotorHandle = (side: Side): Handle => cv.addHandle({
    get: () => motorHandlePos(side),
    set: (p) => {
      clearChips();
      const A = basePoint(g, side);
      onMotorInput(side, Math.atan2(p[1] - A[1], p[0] - A[0]));
    },
    radius: 8,
  });
  const hM1 = mkMotorHandle(0);
  const hM2 = mkMotorHandle(1);
  const hTip = cv.addHandle({ get: () => cfg?.P ?? lastTip ?? [0, 1.5], set: (p) => { clearChips(); requestTip(p); }, radius: 9 });
  const mkEndHandle = (side: Side): Handle => cv.addHandle({
    get: () => (side === 0 ? openEnds().L.tip : openEnds().R.tip),
    set: (p) => {
      clearChips();
      stopTween();
      const ends = openEnds();
      const B = side === 0 ? ends.L.elbow : ends.R.elbow;
      const th = side === 0 ? open.th1 : open.th2;
      const phi = wrapToPi(Math.atan2(p[1] - B[1], p[0] - B[0]) - th);
      if (side === 0) open.phi1 = phi; else open.phi2 = phi;
      msg = '';
      update();
    },
    radius: 7,
  });
  const hE1 = mkEndHandle(0);
  const hE2 = mkEndHandle(1);

  // ---------------------------------------------------------------- UI sync
  function syncUI(): void {
    const five = mode !== 'rpr';
    hM1.enabled = (mode === 'fk' && !lockTh1) || mode === 'open';
    hM2.enabled = mode === 'fk' || mode === 'open';
    hTip.enabled = mode === 'ik';
    hE1.enabled = hE2.enabled = mode === 'open';
    ph.setEnabled(mode === 'rpr');
    flipBtn.style.display = mode === 'fk' ? '' : 'none';
    closeBtn.style.display = mode === 'open' ? '' : 'none';
    nextSolBtn.style.display = mode === 'rpr' ? '' : 'none';
    tLock.el.style.display = mode === 'fk' ? '' : 'none';
    s1.el.style.display = s2.el.style.display = five ? '' : 'none';
    s1.disabled = mode === 'ik' || (mode === 'fk' && lockTh1);
    s2.disabled = mode === 'ik';
    sp1.el.style.display = sp2.el.style.display = mode === 'open' ? '' : 'none';
    optRow.style.display = five ? '' : 'none';
    geoBox.style.display = five ? '' : 'none';
    legFive.style.display = five ? '' : 'none';
    legRpr.style.display = five ? 'none' : '';
    info.style.display = five ? '' : 'none';
    gWrap.style.display = step === 0 ? '' : 'none';
    rKV.el.style.display = five ? 'none' : '';
    cv.requestRender();
  }

  function clearChips(): void {
    chipGroups[step]?.clear();
  }

  // ---------------------------------------------------------------- readouts
  function updateFiveReadouts(): void {
    gKV.set(GK.nj, 'N = 5 根連桿（含地桿），J = 5 個 R 關節（f = 1）');
    gKV.set(GK.dof, lockTh1 ? '3(5−1−5) + 5 = 2；鎖住 θ₁ 後只剩 1' : '3(5−1−5) + 5 = 2 ＝ 致動 2（θ₁, θ₂），被動 3（φ₁, φ₂, ψ）');
    if (mode === 'open') {
      const { gap } = openEnds();
      const n = Math.hypot(gap[0], gap[1]);
      fKV.set(K.q, `${deg(open.th1)}, ${deg(open.th2)} ｜ ${deg(open.phi1)}, ${deg(open.phi2)}, —`);
      fKV.set(K.tip, n < 1e-6 ? 'p_L = p_R（兩端重合）' : '兩條腿的端點分開');
      fKV.set(K.res, `${sci(n)} m${n < 1e-6 ? '（閉合）' : '（未閉合）'}`);
      const f = fkAll(g, open.th1, open.th2);
      fKV.set(K.sol, f.status === 'none' ? 'FK：0（這組馬達角無法閉合）' : f.status === 'coincident' ? 'FK：無限多個' : `FK：${f.solutions.length} 個閉合方式`);
      fKV.set(K.sens, '—');
      return;
    }
    if (!cfg) {
      fKV.set(K.q, `${deg(badMotors[0])}, ${deg(badMotors[1])} ｜ —`);
      fKV.set(K.tip, '—');
      fKV.set(K.res, '—');
      fKV.set(K.sol, 'FK：0（無法裝配）');
      fKV.set(K.sens, '—');
      return;
    }
    const c = cfg;
    const r = loopResidual(g, c);
    fKV.set(K.q, `${deg(c.th1)}, ${deg(c.th2)} ｜ ${deg(c.phi1)}, ${deg(c.phi2)}, ${deg(c.psi)}`);
    fKV.set(K.tip, `(${fmt(c.P[0], 3)}, ${fmt(c.P[1], 3)})`);
    fKV.set(K.res, `${sci(Math.hypot(r[0], r[1]))} m`);
    if (mode === 'ik') {
      const n = ikAll(g, c.P).length;
      fKV.set(K.sol, `IK：${n} 組分支，目前 ${branchName(branchOf(c))}`);
    } else {
      const f = fkAll(g, c.th1, c.th2);
      const which = f.solutions.length === 2 ? `，目前在 B₁→B₂ ${c.mode === 1 ? '左' : '右'}側` : '';
      fKV.set(K.sol, f.status === 'tangent' ? 'FK：1（兩解合一：相切）' : f.status === 'coincident' ? 'FK：無限多個（兩肘重合）' : `FK：${f.solutions.length}${which}`);
    }
    shifts = [tipShift(g, c, 0, D), tipShift(g, c, 1, D)];
    const cm = (sh: Pt | null) => (sh ? `${fmt(100 * Math.hypot(sh[0], sh[1]), 2)} cm` : '無法裝配');
    const J = fkJacobianFD(g, c);
    fKV.set(K.sens, `θ₁: ${cm(shifts[0])}，θ₂: ${cm(shifts[1])}${J ? `（最大 ≈ ${fmt(100 * sigmaMax(J) * D, 2)} cm/°）` : ''}`);
  }

  function updateRprReadouts(): void {
    const p = ph.pose;
    const L = rprIK(DEFAULT_RPR, p);
    rKV.set(RK.len, `(${L.map((v) => fmt(v, 3)).join(', ')}) m`);
    rKV.set(RK.pose, `(${fmt(p.x, 2)}, ${fmt(p.y, 2)}, ${deg(p.theta)})`);
    rKV.set(RK.n, `${rprSols.length} 組（最多 6 組）`);
    rKV.set(RK.sg, 'IK：6 個距離公式；FK：最多 40 組解');
  }

  function defaultStatus(): string {
    if (mode === 'rpr') return `IK：三個腿長由位姿直接算出。FK：這組腿長共有 ${rprSols.length} 個平台位姿。`;
    if (mode === 'open') return '迴路已拆開：兩條腿各自是 2R 開鏈，紅色箭頭是閉合殘差 g = p_L − p_R。';
    if (mode === 'ik') return `IK：給定末端，兩條腿各解一次 2R；目前分支 ${branchName(branch)}。`;
    if (lockTh1) return '鎖住 θ₁ 後只剩 θ₂ 一個輸入：末端只能沿虛線（四連桿的耦合曲線）移動。';
    return 'FK：馬達角決定兩個肘點，末端在兩個圓的交點上。';
  }

  function update(): void {
    if (mode === 'open') {
      s1.set(open.th1, { silent: true });
      s2.set(open.th2, { silent: true });
      sp1.set(open.phi1, { silent: true });
      sp2.set(open.phi2, { silent: true });
    } else if (mode !== 'rpr') {
      const [a, b] = currentMotors();
      s1.set(a, { silent: true });
      s2.set(b, { silent: true });
    }
    if (mode === 'rpr') updateRprReadouts(); else updateFiveReadouts();
    status.textContent = msg || defaultStatus();
    status.style.color = msg.startsWith('✕') ? 'var(--danger)' : msg.startsWith('⚠') ? 'var(--warn)' : '';
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

  function motorSymbol(c: Canvas2D, p: Pt, alpha = 1): void {
    const s = 8 / c.scale;
    const accent = cssVar('--accent', '#2563eb');
    c.polygon([[p[0] - s, p[1] - s], [p[0] + s, p[1] - s], [p[0] + s, p[1] + s], [p[0] - s, p[1] + s]], { fill: accent, color: accent, width: 1, alpha });
  }

  function passiveSymbol(c: Canvas2D, p: Pt, alpha = 1, rPx = 6): void {
    c.point(p, rPx, { color: cssVar('--bg-elev', '#fff'), alpha });
    c.circle(p, rPx / c.scale, { color: cssVar('--fg', '#222'), width: 2, alpha });
  }

  function drawGround(c: Canvas2D): void {
    const muted = cssVar('--fg-muted', '#777');
    const A1 = basePoint(g, 0);
    const A2 = basePoint(g, 1);
    const x0 = A1[0] - 0.3;
    const x1 = A2[0] + 0.3;
    c.line([x0, 0], [x1, 0], { color: muted, width: 2 });
    for (let x = x0 + 0.06; x <= x1 + 1e-9; x += 0.12) c.line([x, 0], [x - 0.08, -0.08], { color: muted, width: 1 });
    if (show.labels) c.text([0, -0.1], `地桿 d = ${fmt(g.d, 2)} m`, { color: muted, dy: 8, font: '11px sans-serif', bg: cssVar('--bg-elev', '#fff') });
  }

  function drawLegs(c: Canvas2D, A1: Pt, B1: Pt, E1: Pt, A2: Pt, B2: Pt, E2: Pt, o: { alpha?: number; thin?: boolean } = {}): void {
    const a = o.alpha ?? 1;
    const w = o.thin ? 3 : 8;
    const colL = cssVar('--series-1', '#2563eb');
    const colR = cssVar('--series-4', '#d97706');
    c.line(A1, B1, { color: colL, width: w, alpha: a });
    c.line(B1, E1, { color: colL, width: w * 0.75, alpha: a });
    c.line(A2, B2, { color: colR, width: w, alpha: a });
    c.line(B2, E2, { color: colR, width: w * 0.75, alpha: a });
  }

  function drawFiveBar(c: Canvas2D, q: Config, o: { ghost?: boolean; labels?: boolean } = {}): void {
    const A1 = basePoint(g, 0);
    const A2 = basePoint(g, 1);
    if (o.ghost) {
      drawLegs(c, A1, q.B1, q.P, A2, q.B2, q.P, { alpha: 0.38, thin: true });
      for (const p of [q.B1, q.B2, q.P]) passiveSymbol(c, p, 0.55, 3.5);
      return;
    }
    drawLegs(c, A1, q.B1, q.P, A2, q.B2, q.P);
    motorSymbol(c, A1);
    motorSymbol(c, A2);
    passiveSymbol(c, q.B1);
    passiveSymbol(c, q.B2);
    passiveSymbol(c, q.P);
    if (o.labels) drawJointLabels(c, q);
  }

  function drawJointLabels(c: Canvas2D, q: Config): void {
    const accent = cssVar('--accent', '#2563eb');
    const fg = cssVar('--fg', '#222');
    const warn = cssVar('--warn', '#b45309');
    const bg = cssVar('--bg-elev', '#fff');
    const A1 = basePoint(g, 0);
    const A2 = basePoint(g, 1);
    // motor angles measured from +x
    c.arc(A1, 0.3, Math.min(0, q.th1), Math.max(0, q.th1), { color: warn, width: 1.5 });
    c.arc(A2, 0.3, Math.min(0, q.th2), Math.max(0, q.th2), { color: warn, width: 1.5 });
    c.line(A1, [A1[0] + 0.38, A1[1]], { color: warn, width: 1, dash: [3, 3] });
    c.line(A2, [A2[0] + 0.38, A2[1]], { color: warn, width: 1, dash: [3, 3] });
    c.text([A1[0] + 0.42 * Math.cos(q.th1 / 2), A1[1] + 0.42 * Math.sin(q.th1 / 2)], 'θ₁', { color: accent, font: 'bold 13px sans-serif' });
    c.text([A2[0] + 0.42 * Math.cos(q.th2 / 2), A2[1] + 0.42 * Math.sin(q.th2 / 2)], 'θ₂', { color: accent, font: 'bold 13px sans-serif' });
    // passive joints: push the label away from the centre of the mechanism
    const ctr = centroid([A1, A2, q.B1, q.B2, q.P]);
    const away = (p: Pt, d: number): Pt => {
      const vx = p[0] - ctr[0];
      const vy = p[1] - ctr[1];
      const n = Math.hypot(vx, vy) || 1;
      return [p[0] + (d * vx) / n, p[1] + (d * vy) / n];
    };
    c.text(away(q.B1, 0.24), 'φ₁', { color: fg, font: '13px sans-serif', bg });
    c.text(away(q.B2, 0.24), 'φ₂', { color: fg, font: '13px sans-serif', bg });
    c.text(away(q.P, 0.24), 'ψ', { color: fg, font: '13px sans-serif', bg });
  }

  function drawWorkspace(c: Canvas2D): void {
    ensureWorkspace();
    ws.draw(c, 1);
    const cols = [cssVar('--series-1', '#2563eb'), cssVar('--series-4', '#d97706')];
    ([0, 1] as Side[]).forEach((side) => {
      const A = basePoint(g, side);
      const [Lp, Ld] = legLinks(g, side);
      c.circle(A, Lp + Ld, { color: cols[side], width: 1, dash: [5, 5], alpha: 0.7 });
      if (Math.abs(Lp - Ld) > 1e-3) c.circle(A, Math.abs(Lp - Ld), { color: cols[side], width: 1, dash: [5, 5], alpha: 0.7 });
    });
  }

  function drawCircles(c: Canvas2D, th1: number, th2: number, current: Config | null): void {
    const colL = cssVar('--series-1', '#2563eb');
    const colR = cssVar('--series-4', '#d97706');
    const accent = cssVar('--accent', '#2563eb');
    const res = fkAll(g, th1, th2);
    c.circle(res.B1, g.L2, { color: colL, width: 1.5, dash: [6, 5], alpha: 0.85 });
    c.circle(res.B2, g.L4, { color: colR, width: 1.5, dash: [6, 5], alpha: 0.85 });
    for (const s of res.solutions) {
      const active = current && Math.hypot(s.P[0] - current.P[0], s.P[1] - current.P[1]) < 1e-6;
      if (active) continue; // the active tip is drawn by the mechanism itself
      c.circle(s.P, 7 / c.scale, { color: accent, width: 2.5 });
      if (current) labelInside(c, s.P, show.sens ? '另一解' : '另一個交點（另一個裝配模式）', { color: accent, dx: show.sens ? 34 : 0, dy: 20, bg: cssVar('--bg-elev', '#fff'), font: '12px sans-serif' });
    }
    if (current && res.status === 'tangent') labelInside(c, current.P, '兩圓相切：兩解合一', { color: cssVar('--danger', '#b91c1c'), dy: -24, bg: cssVar('--bg-elev', '#fff'), font: 'bold 12px sans-serif' });
  }

  function drawLoopArrows(c: Canvas2D, q: Config): void {
    const col = cssVar('--series-5', '#7c3aed');
    const A1 = basePoint(g, 0);
    const A2 = basePoint(g, 1);
    const chain: Pt[] = [A1, q.B1, q.P, q.B2, A2, A1];
    const ctr = centroid([A1, A2, q.B1, q.B2, q.P]);
    for (let i = 0; i < chain.length - 1; i++) {
      const a = chain[i];
      const b = chain[i + 1];
      const L = Math.hypot(b[0] - a[0], b[1] - a[1]);
      if (L < 1e-6) continue;
      // offset towards the inside of the loop so the arrows do not sit on top of the links
      let nx = -(b[1] - a[1]) / L;
      let ny = (b[0] - a[0]) / L;
      const mx = (a[0] + b[0]) / 2 - ctr[0];
      const my = (a[1] + b[1]) / 2 - ctr[1];
      if (nx * mx + ny * my > 0) { nx = -nx; ny = -ny; }
      const off = 0.1;
      const sh = 0.16 / L; // shorten both ends so the arrow heads do not hide the joints
      const s0: Pt = [a[0] + sh * (b[0] - a[0]) + off * nx, a[1] + sh * (b[1] - a[1]) + off * ny];
      const s1: Pt = [b[0] - sh * (b[0] - a[0]) + off * nx, b[1] - sh * (b[1] - a[1]) + off * ny];
      c.arrow(s0, s1, { color: col, width: 2.5, head: 11 });
    }
    c.text(ctr, '繞一圈：向量和 = 0', { color: col, font: 'bold 12px sans-serif', bg: cssVar('--bg-elev', '#fff') });
  }

  function drawSensitivity(c: Canvas2D, q: Config): void {
    const cols = [cssVar('--series-1', '#2563eb'), cssVar('--series-4', '#d97706')];
    const danger = cssVar('--danger', '#b91c1c');
    const bg = cssVar('--bg-elev', '#fff');
    shifts.forEach((s, m) => {
      if (!s) {
        labelInside(c, q.P, `θ${m === 0 ? '₁' : '₂'} +1°：無法裝配`, { color: danger, dx: m === 0 ? -70 : 70, dy: 30, bg });
        return;
      }
      const head: Pt = [q.P[0] + SENS_SCALE * s[0], q.P[1] + SENS_SCALE * s[1]];
      c.arrow(q.P, head, { color: cols[m], width: 3, head: 12 });
      const n = Math.hypot(s[0], s[1]) || 1;
      labelInside(c, head, `θ${m === 0 ? '₁' : '₂'} +1°`, { color: cols[m], dx: (14 * s[0]) / n, dy: (-14 * s[1]) / n, bg });
    });
  }

  function drawOpen(c: Canvas2D): void {
    const { L, R, gap } = openEnds();
    const danger = cssVar('--danger', '#b91c1c');
    const ok = cssVar('--ok', '#15803d');
    const bg = cssVar('--bg-elev', '#fff');
    drawLegs(c, L.base, L.elbow, L.tip, R.base, R.elbow, R.tip);
    motorSymbol(c, L.base);
    motorSymbol(c, R.base);
    passiveSymbol(c, L.elbow);
    passiveSymbol(c, R.elbow);
    const n = Math.hypot(gap[0], gap[1]);
    if (n < 1e-6) {
      passiveSymbol(c, L.tip);
      labelInside(c, L.tip, '閉合 ✓', { color: ok, dy: -22, bg, font: 'bold 12px sans-serif' });
    } else {
      c.point(L.tip, 5, { color: cssVar('--series-1', '#2563eb') });
      c.point(R.tip, 5, { color: cssVar('--series-4', '#d97706') });
      if (n > 0.02) c.arrow(R.tip, L.tip, { color: danger, width: 2.5, head: 11 });
      const mid: Pt = [(L.tip[0] + R.tip[0]) / 2, (L.tip[1] + R.tip[1]) / 2];
      labelInside(c, mid, `g（‖g‖ = ${fmt(n, 3)} m）`, { color: danger, dy: -16, bg, font: 'bold 12px sans-serif' });
      labelInside(c, L.tip, 'p_L', { color: cssVar('--series-1', '#2563eb'), dx: -16, dy: 14 });
      labelInside(c, R.tip, 'p_R', { color: cssVar('--series-4', '#d97706'), dx: 16, dy: 14 });
    }
    if (show.circles) drawCircles(c, open.th1, open.th2, null);
  }

  function drawInvalid(c: Canvas2D): void {
    const danger = cssVar('--danger', '#b91c1c');
    const colL = cssVar('--series-1', '#2563eb');
    const colR = cssVar('--series-4', '#d97706');
    const A1 = basePoint(g, 0);
    const A2 = basePoint(g, 1);
    const B1 = elbowPoint(g, 0, badMotors[0]);
    const B2 = elbowPoint(g, 1, badMotors[1]);
    c.line(A1, B1, { color: colL, width: 8 });
    c.line(A2, B2, { color: colR, width: 8 });
    c.circle(B1, g.L2, { color: colL, width: 1.5, dash: [6, 5] });
    c.circle(B2, g.L4, { color: colR, width: 1.5, dash: [6, 5] });
    motorSymbol(c, A1);
    motorSymbol(c, A2);
    passiveSymbol(c, B1);
    passiveSymbol(c, B2);
    labelInside(c, [(B1[0] + B2[0]) / 2, (B1[1] + B2[1]) / 2 + 0.25], '✕ 兩圓不相交：無法裝配', { color: danger, font: 'bold 13px sans-serif', bg: cssVar('--bg-elev', '#fff') });
  }

  function drawRpr(c: Canvas2D): void {
    const accent = cssVar('--accent', '#2563eb');
    const muted = cssVar('--fg-muted', '#777');
    const bg = cssVar('--bg-elev', '#fff');
    const base = DEFAULT_RPR.base;
    const pose = ph.pose;
    // other FK solutions (same leg lengths)
    rprSols.forEach((s, k) => {
      if (isSame(s, pose)) return;
      const col = cssVar(`--series-${[2, 3, 5, 6, 7, 8][k % 6]}`, '#888');
      const qq = rprJoints(DEFAULT_RPR, s);
      for (let i = 0; i < 3; i++) c.line(base[i], qq[i], { color: col, width: 1, dash: [2, 4], alpha: 0.45 });
      c.polygon(qq, { fill: col, alpha: 0.06, width: 0 });
      c.polygon(qq, { color: col, width: 1.6, dash: [6, 4], alpha: 0.9 });
      c.text(centroid(qq), `#${k + 1}`, { color: col, font: 'bold 11px sans-serif', bg: cssVar('--bg-elev', '#fff') });
    });
    const q = rprJoints(DEFAULT_RPR, pose);
    const lens = rprIK(DEFAULT_RPR, pose);
    for (let i = 0; i < 3; i++) {
      const a = base[i];
      const b = q[i];
      const L = lens[i] || 1e-9;
      const u: Pt = [(b[0] - a[0]) / L, (b[1] - a[1]) / L];
      const sl = Math.min(0.42, 0.6 * L);
      const m: Pt = [a[0] + u[0] * sl, a[1] + u[1] * sl];
      c.line(a, m, { color: muted, width: 11 });
      c.line(m, b, { color: accent, width: 4 });
      const mid: Pt = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2];
      labelInside(c, mid, `ℓ${'₁₂₃'[i]} = ${fmt(L, 2)}`, { color: accent, bg, font: '12px sans-serif', dx: -12 * u[1], dy: -12 * u[0] });
    }
    c.polygon(q, { fill: accent, alpha: 0.14, width: 0 });
    c.polygon(q, { color: accent, width: 2 });
    for (const a of base) {
      c.polygon([[a[0] - 0.1, a[1] - 0.12], [a[0] + 0.1, a[1] - 0.12], [a[0], a[1]]], { fill: cssVar('--grid-strong', '#bbb'), color: cssVar('--grid-strong', '#bbb'), width: 1 });
      passiveSymbol(c, a);
    }
    for (const p of q) passiveSymbol(c, p, 1, 5);
    ph.draw();
    const cur = rprSols.findIndex((s) => isSame(s, pose));
    c.overlay.textContent = `3×RPR：${rprSols.length} 組 FK 實數解（目前是 #${cur + 1}）`;
  }

  cv.draw = (c) => {
    hM1.color = hE1.color = cssVar('--series-1', '#2563eb');
    hM2.color = hE2.color = cssVar('--series-4', '#d97706');
    hTip.color = cssVar('--accent', '#2563eb');
    c.grid(0.5);
    if (mode === 'rpr') { drawRpr(c); return; }
    if (show.workspace) drawWorkspace(c);
    drawGround(c);
    if (mode === 'open') {
      drawOpen(c);
      c.overlay.textContent = `迴路拆開  θ = (${deg(open.th1, 0)}, ${deg(open.th2, 0)})`;
      return;
    }
    if (!cfg) {
      drawInvalid(c);
      c.overlay.textContent = `FK  θ = (${deg(badMotors[0], 0)}, ${deg(badMotors[1], 0)})  無法裝配`;
      return;
    }
    const q = cfg;
    if (show.ghost) {
      if (mode === 'ik') {
        for (const s of ikAll(g, q.P)) if (s.branch[0] !== branch[0] || s.branch[1] !== branch[1]) drawFiveBar(c, s.cfg, { ghost: true });
      } else {
        for (const s of fkAll(g, q.th1, q.th2).solutions) if (Math.hypot(s.P[0] - q.P[0], s.P[1] - q.P[1]) > 1e-6) drawFiveBar(c, s, { ghost: true });
      }
    }
    if (lockTh1 && sweepPath.length > 1) {
      c.polyline(sweepPath, { color: cssVar('--series-5', '#7c3aed'), width: 2, dash: [6, 4] });
    }
    if (show.circles) drawCircles(c, q.th1, q.th2, q);
    drawFiveBar(c, q, { labels: show.labels });
    if (show.loop) drawLoopArrows(c, q);
    if (show.sens) drawSensitivity(c, q);
    if (mode === 'ik') {
      labelInside(c, q.P, branchName(branch), { color: cssVar('--accent', '#2563eb'), dy: -24, bg: cssVar('--bg-elev', '#fff'), font: 'bold 12px sans-serif' });
      c.overlay.textContent = `IK  p = (${fmt(q.P[0], 2)}, ${fmt(q.P[1], 2)})  → θ = (${deg(q.th1, 0)}, ${deg(q.th2, 0)})`;
    } else {
      const n = fkAll(g, q.th1, q.th2).solutions.length;
      c.overlay.textContent = `FK  θ = (${deg(q.th1, 0)}, ${deg(q.th2, 0)})  → ${n} 個解`;
    }
  };

  // ---------------------------------------------------------------- per-step presets
  const asFk = (fn: () => void) => () => { setMode('fk'); setLock(false); fn(); update(); };
  const leftStraight = (): void => {
    for (const a of [60, 70, 50, 80, 40].map((x) => x * D)) {
      const A1 = basePoint(g, 0);
      const P: Pt = [A1[0] + (g.L1 + g.L2) * Math.cos(a), A1[1] + (g.L1 + g.L2) * Math.sin(a)];
      const c = ikConfig(g, P, [1, 1]);
      if (c) { setCfg(c); return; }
    }
    resetConfig();
  };
  const chipSpecs: { label: string; apply: () => void; title?: string }[][] = [
    [
      { label: '兩顆馬達都能轉（dof = 2）', apply: asFk(() => presetMotors(DEFAULT_MOTORS[0], DEFAULT_MOTORS[1])) },
      { label: '鎖住 θ₁（剩 1 dof）', apply: () => { setMode('fk'); presetMotors(DEFAULT_MOTORS[0], DEFAULT_MOTORS[1]); setLock(true); update(); } },
    ],
    [
      { label: '閉合：轉馬達（‖g‖ ≈ 0）', apply: asFk(() => presetMotors(DEFAULT_MOTORS[0], DEFAULT_MOTORS[1])) },
      {
        label: '拆開迴路：自己對準',
        apply: () => {
          setMode('fk');
          presetMotors(DEFAULT_MOTORS[0], DEFAULT_MOTORS[1]);
          setMode('open');
          enterOpen([28 * D, -35 * D]);
          update();
        },
      },
    ],
    BRANCHES.map((b) => ({
      label: branchName(b),
      apply: () => { setMode('ik'); if (!cfg) resetConfig(); setBranch(b); },
    })),
    [
      { label: '一般構型（兩個交點）', apply: asFk(() => presetMotors(DEFAULT_MOTORS[0], DEFAULT_MOTORS[1], 1)) },
      { label: '另一個裝配模式', apply: asFk(() => presetMotors(DEFAULT_MOTORS[0], DEFAULT_MOTORS[1], -1)) },
      { label: '無法裝配的馬達角', apply: asFk(() => presetMotors(160 * D, 20 * D)) },
    ],
    [
      { label: '遠離相切', apply: asFk(() => presetMargin(0.6, 1)) },
      { label: '接近相切', apply: asFk(() => presetMargin(0.02, 1)) },
      { label: '正好相切（兩解合一）', apply: asFk(() => presetMargin(0, 1)) },
      { label: '接近相切：另一個模式', apply: asFk(() => presetMargin(0.02, -1)) },
    ],
    [
      { label: '3×RPR：6 組解', apply: () => { setMode('rpr'); setRprPose(RPR_PRESETS.six); update(); } },
      { label: '4 組解', apply: () => { setMode('rpr'); setRprPose(RPR_PRESETS.four); update(); } },
      { label: '2 組解', apply: () => { setMode('rpr'); setRprPose(RPR_PRESETS.two); update(); } },
      {
        label: '預告：遠端桿共線',
        apply: asFk(() => { presetMargin(0, 1); msg = '⚠ 下一課：兩根遠端桿共線＝致動器奇異（actuator singularity）'; }),
      },
      {
        label: '預告：一條腿伸直',
        apply: asFk(() => { leftStraight(); msg = '⚠ 下一課：一條腿完全伸直＝末端奇異（end-effector singularity）'; }),
      },
    ],
  ];
  const chipGroups: PresetChipsHandle[] = chipSpecs.map((specs) => {
    const grp = presetChips(chipsRow, specs, { label: '預設：' });
    grp.el.style.display = 'none';
    return grp;
  });

  interface StepView { ghost: boolean; circles: boolean; loop: boolean; sens: boolean; workspace: boolean; labels: boolean; chip: number }
  const VIEWS: StepView[] = [
    { ghost: false, circles: false, loop: false, sens: false, workspace: false, labels: true, chip: 0 },
    { ghost: false, circles: false, loop: true, sens: false, workspace: false, labels: true, chip: 0 },
    { ghost: true, circles: false, loop: false, sens: false, workspace: true, labels: false, chip: 0 },
    { ghost: true, circles: true, loop: false, sens: false, workspace: false, labels: false, chip: 0 },
    { ghost: true, circles: true, loop: false, sens: true, workspace: false, labels: false, chip: 1 },
    { ghost: false, circles: false, loop: false, sens: false, workspace: false, labels: false, chip: 0 },
  ];
  const focusFor = (i: number): HTMLElement[] => [
    [cv.el, gWrap],
    [cv.el, fWrap],
    [chipGroups[2].el, cv.el],
    [cv.el, flipBtn],
    [fWrap, chipGroups[4].el],
    [cv.el, rKV.el],
  ][i] ?? [];
  let focused: HTMLElement[] = [];

  const onStep = (i: number): void => {
    const k = Math.max(0, Math.min(steps.length - 1, i));
    step = k;
    stopTween();
    chipGroups.forEach((grp, j) => { grp.el.style.display = j === k ? '' : 'none'; });
    const v = VIEWS[k];
    show.ghost = v.ghost;
    show.circles = v.circles;
    show.loop = v.loop;
    show.sens = v.sens;
    show.workspace = v.workspace;
    show.labels = v.labels;
    tGhost.set(v.ghost, { silent: true });
    tCircles.set(v.circles, { silent: true });
    tLoop.set(v.loop, { silent: true });
    tSens.set(v.sens, { silent: true });
    tWs.set(v.workspace, { silent: true });
    if (k <= 4) { setMode('fk'); resetConfig(); }
    chipGroups[k].activate(v.chip);
    focused.forEach((f) => f.classList.remove('mr-focus'));
    focused = focusFor(k);
    focused.forEach((f) => f.classList.add('mr-focus'));
    syncUI();
    update();
  };

  // ---------------------------------------------------------------- initial state
  resetConfig();
  updateRpr();
  onStep(ctx.currentStep);

  return {
    onStep,
    destroy() {
      stopTween();
      anim.stop();
      ph.remove();
      cv.destroy();
    },
  };
}

const lesson: Lesson = { meta, steps, quiz, mount };
export default lesson;
