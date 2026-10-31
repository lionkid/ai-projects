import type { Lesson, LessonContext, LessonHandle, LessonStep, QuizQuestion } from '@/types';
import {
  el, cssVar, fmt, controlPanel, row, slider, angleSlider, select, toggle, readout, simTitle, note, kv,
  Canvas2D, presetChips, torqueBars, drawPlanarArm,
  type Pt, type PresetChipsHandle, type TextStyle,
} from '@/ui';
import { planarFK, type Mat } from '@/mr';
import { meta } from './meta';
import {
  MODELS, lengthsOf, staticTorques, tipJacobian, ellipseAxes, momentArms, solveTip, otherSolution, elbowBranch,
  POSTURES_2R, powers, vnorm, wrapAngle, type ModelKey, type StaticsResult, type AxesInfo,
} from './statics';

const D = Math.PI / 180;
/** World units per newton for the force arrow and the force ellipse (they share one scale). */
const K_F = 0.025;
/** World units per m/s for the velocity ellipse and the virtual tip velocity. */
const K_V = 0.4;
const F_MAX = 40; // N
const TAU_BAR_MAX = 40; // N·m, full scale of the torque bars and arcs
const ELLIPSE_CAP = 4; // world units; longer semi-axes are drawn as "→ ∞"
const G = 9.8;

const steps: LessonStep[] = [
  {
    title: '外力到關節力矩',
    md: `末端抓著東西或頂住牆時，末端和環境之間有一個力 $f$；手臂要保持不動，每個關節的馬達都得出力矩。本課的箭頭畫的是**末端施於環境**的力（托住重物時，手往上推）。

對旋轉關節來說，關節 $i$ 需要的力矩就是 $f$ 對該關節軸的力矩：

$$\\tau_i = \\big((p - p_i)\\times f\\big)_z = d_i\\,\\|f\\| ,$$

$p$ 是末端、$p_i$ 是關節位置，$d_i$ 是力臂（關節到力作用線的垂直距離）。作用線穿過哪個關節，那個關節就完全不必出力。這裡忽略連桿自重。`,
    simHint: '拖曳橘色力箭頭的尖端改變 f；按「力穿過關節 2」，看虛線力臂 d₂ 與 τ₂ 一起歸零。',
  },
  {
    title: '用功率推導 τ = Jᵀ F',
    md: `換個角度：讓關節以任意的虛擬速度 $\\dot\\theta$ 微動。靜止、無摩擦時，馬達輸入的功率必須全部變成末端對環境做的功率：

$$\\dot\\theta^{\\mathsf T}\\tau = \\mathcal V^{\\mathsf T}\\mathcal F = (J\\dot\\theta)^{\\mathsf T}\\mathcal F \\quad \\text{對所有 } \\dot\\theta ,$$

因此 $\\tau = J^{\\mathsf T}(\\theta)\\,\\mathcal F$。前提是 $\\mathcal V$ 與 $\\mathcal F$ 寫在同一個座標系：$\\tau = J_b^{\\mathsf T}\\mathcal F_b = J_s^{\\mathsf T}\\mathcal F_s$，其中 $\\mathcal F_s = [\\Ad_{T_{bs}}]^{\\mathsf T}\\mathcal F_b$。拿 $J_s$ 去配 $\\mathcal F_b$ 算出來的數字沒有物理意義。`,
    simHint: '拉動虛擬速度 θ̇₁、θ̇₂：兩邊的功率永遠相等。下方表格用 PoE 的 J_b、J_s 算 Jᵀℱ，看哪一種配錯了座標系。',
  },
  {
    title: '力橢球',
    md: `限制關節力矩的大小 $\\|\\tau\\| \\le \\tau_{\\max}$，末端能施出的力就落在一個橢圓裡：

$$\\|\\tau\\|^2 = f^{\\mathsf T}(J_vJ_v^{\\mathsf T})\\,f \\le \\tau_{\\max}^{2} .$$

它和速度橢圓用的是同一個矩陣 $J_vJ_v^{\\mathsf T}$，只差在一個取逆：主軸方向相同，半軸長從 $\\sigma_i$ 變成 $\\tau_{\\max}/\\sigma_i$，互為倒數。速度容易的方向出力吃虧，速度困難的方向反而出得了大力——和槓桿一樣，速度與力不可兼得。力箭頭的尖端落在紅色橢圓內，就代表 $\\|\\tau\\| \\le \\tau_{\\max}$。`,
    simHint: '兩個橢圓都已打開。拖曳末端改變構型，看一個橢圓變胖時另一個怎麼變；再把力箭頭拖到紅色橢圓外，‖τ‖ 就超過預算。',
  },
  {
    title: '奇異構型反而能扛',
    md: `把手臂完全伸直，再讓力沿著連桿方向：每個關節到作用線的力臂都是 0，所以 $\\tau = 0$，負載全由連桿以軸向力承擔，手臂就像一根撐竿。

用橢圓的語言：伸直時 $\\sigma_{\\min} = 0$，速度橢圓沿手臂方向壓成線段，力橢圓在同一方向的半軸 $\\tau_{\\max}/\\sigma_{\\min}$ 變成無限大，也就是

$$f \\in \\mathcal N\\big(J_v^{\\mathsf T}\\big) \\;\\Rightarrow\\; \\tau = J_v^{\\mathsf T} f = 0 .$$

代價是這個方向完全沒有運動能力，而且力只要稍微偏離連桿方向，力矩就會冒出來。`,
    simHint: '按「伸直＋沿連桿施力」：τ ≈ 0。再按「偏 10°」與「垂直施力」，看力矩條怎麼跳起來。',
  },
  {
    title: '同一個力，不同的構型',
    md: `同樣是托住 2 kg 的物體，姿勢不同，馬達的負擔可以差很多。下表用目前的力 $f$ 計算幾種 2R 姿勢的力矩：

- $\\tau_1 = (p \\times f)_z$ 只跟末端位置有關，所以同一末端點的肘上、肘下兩解 $\\tau_1$ 相同；
- $\\tau_2$ 取決於手肘的位置，兩解可以差很多。

讓力的作用線靠近關節、讓負載沿著連桿傳遞，是安排工作姿勢時最省力的原則；選姿勢本身就是設計決策。`,
    simHint: '點表格左欄的姿勢名稱套用該姿勢，比較 ‖τ‖；淡色虛影是同一末端位置的另一組解，它的力矩 τ′ 顯示在上方讀數列。',
  },
  {
    title: '方向約定與下一章',
    md: `$\\mathcal F$ 是誰對誰施力？本站與教材都約定 $\\mathcal F$ 是**末端施於環境**的力旋量，所以 $\\tau = J^{\\mathsf T}\\mathcal F$。若拿到的是**環境施於末端**的負載 $\\mathcal F_{\\text{ext}}$（例如物體的重量），維持靜止需要

$$\\tau = -J^{\\mathsf T}\\mathcal F_{\\text{ext}} .$$

切換約定時箭頭不動、力矩全部變號；同一個物理情況用兩種約定正確描述，會得到同一組 $\\tau$。下一章談逆向運動學：給定末端位置，2R 通常有兩組解（虛影），而它們承受同一個力時的力矩並不相同。`,
    simHint: '依序按三個預設：同一個 2 kg 負載用兩種約定描述，τ 相同；把向下的重量直接套 τ = Jᵀℱ，力矩全部變號。',
  },
];

const quiz: QuizQuestion[] = [
  {
    question: '人用伸直的手臂推牆，比彎著手肘推牆輕鬆得多。用力橢圓解釋，哪個說法最恰當？',
    options: [
      '伸直時接近奇異，力橢圓沿手臂方向的半軸 $\\tau_{\\max}/\\sigma_{\\min}$ 非常大，沿手臂推牆幾乎不需要關節力矩',
      '伸直時速度橢圓最大，所以能出的力也最大',
      '伸直時 Jacobian 的秩變大，所以力量增加',
      '力橢圓與姿勢無關，差別只在肌肉疲勞',
    ],
    answer: 0,
    explanation: '伸直時 $J_v$ 的欄位都垂直於手臂，沿手臂方向的力滿足 $J_v^{\\mathsf T}f \\approx 0$：各關節的力臂幾乎為零，力由骨骼（連桿）軸向承擔。這就是力橢圓沿手臂方向變得很長的原因；速度橢圓在同一方向則被壓扁——兩者的半軸互為倒數。',
  },
  {
    question: '平面 3R 只考慮末端的純力 $f \\in \\mathbb R^2$（$J_v$ 為 $2\\times 3$ 且滿秩）。下列敘述何者正確？',
    options: [
      '給定 $f$，$\\tau = J_v^{\\mathsf T} f$ 唯一；但不是每個 $\\tau \\in \\mathbb R^3$ 都能被某個末端力平衡',
      '給定 $f$，有無限多組 $\\tau$ 都能讓手臂靜止',
      '任意的 $\\tau$ 都對應唯一的 $f$',
      '$J_v^{\\mathsf T}$ 是方陣，所以可以直接求逆得到 $f$',
    ],
    answer: 0,
    explanation: '$J_v^{\\mathsf T}$ 是 $3\\times 2$：把 2 維的 $f$ 映到力矩空間裡的一個 2 維子空間，所以給定 $f$ 時 $\\tau$ 唯一。反過來，若 $\\tau$ 有分量落在 $J_v$ 的零空間方向（自運動方向），任何末端力都平衡不了它——即使末端被固定住，手臂也會沿自運動方向動起來。',
  },
  {
    question: '要讓 2R 手臂在某處穩穩抬起重物（需要很大的垂直向上力），理想姿勢應讓力橢圓的長軸朝哪裡？此時速度橢圓在垂直方向如何？',
    options: [
      '力橢圓長軸朝垂直方向；速度橢圓在垂直方向是短軸——抬得動，但抬得慢',
      '力橢圓長軸朝水平方向；速度橢圓在垂直方向也最長',
      '兩個橢圓的長軸一定指向同一個方向',
      '力橢圓的形狀無法靠改變姿勢來調整',
    ],
    answer: 0,
    explanation: '兩個橢圓都由 $J_vJ_v^{\\mathsf T}$ 決定，主軸方向相同、半軸長互為倒數（$\\sigma_i$ 對 $1/\\sigma_i$）。讓垂直方向成為速度的短軸，它就成了力的長軸：同樣的力矩預算撐得起更大的垂直力，代價是垂直方向動得比較慢。',
  },
  {
    question: '末端抓著 2 kg 物體靜止（$g = 9.8$ m/s²）。若把物體作用在手上的重量 $\\mathcal F_{\\text{ext}} = (0, -19.6)$ N 直接代入 $\\tau = J^{\\mathsf T}\\mathcal F$，會得到什麼？',
    options: [
      '正確的馬達力矩',
      '大小正確、但每個分量都差一個負號；應該用 $\\tau = -J^{\\mathsf T}\\mathcal F_{\\text{ext}}$',
      '全部為零',
      '兩倍大的力矩',
    ],
    answer: 1,
    explanation: '$\\tau = J^{\\mathsf T}\\mathcal F$ 裡的 $\\mathcal F$ 是末端施於環境的力：手要往上托，$\\mathcal F = (0, +19.6)$ N。物體作用在手上的重量是它的反作用力 $\\mathcal F_{\\text{ext}} = -\\mathcal F$，直接代入會讓每個力矩變號——馬達反而幫倒忙，手臂會往下掉。',
  },
];

type Conv = 'env' | 'ext';

function mount(host: HTMLElement, ctx: LessonContext): LessonHandle {
  // ---------------------------------------------------------------- state
  let key: ModelKey = '2R';
  let model = MODELS[key];
  let L = lengthsOf(key);
  let th: number[] = [10 * D, 60 * D];
  let branch: 1 | -1 = 1;
  let f: Pt = [0, 2 * G]; // the drawn arrow (N, {s} coordinates)
  let conv: Conv = 'env';
  let qdot: number[] = [0.6, -0.4, 0.3];
  let tauBar = 20;
  let step = -1;
  const show = { vel: false, force: false, arms: true, ghost: false };

  // derived
  let res: StaticsResult = staticTorques(model, th, f);
  let J: Mat = tipJacobian(L, th);
  let ax: AxesInfo = ellipseAxes(J);
  let pts: Pt[] = [];
  let ghostTh: number[] | null = null;

  const fEnv = (): Pt => (conv === 'env' ? f : [-f[0], -f[1]]);

  // ---------------------------------------------------------------- layout
  simTitle(host, '末端力 → 關節力矩', '平面 2R / 3R');
  const cv = new Canvas2D(host, { height: 320, world: { xmin: -2.5, xmax: 2.5, ymin: -1.3, ymax: 2.5 }, overlay: true });

  const rp = controlPanel(host, { compact: true });
  const rrow = row(rp);
  const rNorm = readout(rrow, { label: '‖τ‖ =' });
  const rF = readout(rrow, { label: '|f| =' });
  const rConv = readout(rrow, { label: '' });
  const rD = readout(rrow, { label: '力臂 d =' });
  const rGhost = readout(rrow, { label: '另一組解 τ′ =' });
  const bars2 = torqueBars(rp, { labels: ['τ₁', 'τ₂'], max: TAU_BAR_MAX, unit: 'N·m', digits: 1 });
  const bars3 = torqueBars(rp, { labels: ['τ₁', 'τ₂', 'τ₃'], max: TAU_BAR_MAX, unit: 'N·m', digits: 1 });
  // bars live in their own box so a step can hide them without touching the model switch
  const barsBox = el('div', { style: { gridColumn: '1 / -1' } });
  rp.appendChild(barsBox);
  barsBox.append(bars2.el, bars3.el);

  // step 2: virtual power and frame pairing
  const powerPanel = controlPanel(host, { compact: true });
  const qdSliders = [0, 1, 2].map((i) => slider(powerPanel, {
    label: `虛擬 θ̇${'₁₂₃'[i]}`, min: -1, max: 1, step: 0.05, value: qdot[i], unit: 'rad/s',
    onInput: (v) => { qdot[i] = v; chipGroups[step]?.clear(); update(); },
  }));
  const powCell = el('div', { style: { display: 'grid', gap: '0.15rem' } });
  powerPanel.appendChild(powCell);
  const rPj = readout(powCell, { label: '馬達 θ̇ᵀτ =', unit: 'W' });
  const rPt = readout(powCell, { label: '末端 𝒱ᵀℱ =', unit: 'W' });
  const frameKv = kv(powerPanel, ['ℱ_b（{b} 座標）', 'ℱ_s = [Ad_Tbs]ᵀ ℱ_b', 'J_bᵀℱ_b ＝ J_sᵀℱ_s ✓', 'J_sᵀℱ_b ✗ 座標系配錯']);
  frameKv.el.style.gridColumn = '1 / -1';

  // steps 3–4: ellipse axes
  const ellPanel = controlPanel(host, { compact: true });
  const sBar = slider(ellPanel, { label: '力矩預算 τ_max', min: 5, max: 40, step: 1, value: tauBar, unit: 'N·m', onInput: (v) => { tauBar = v; update(); } });
  const rInside = readout(ellPanel, { label: 'f 在力橢圓內？' });
  rInside.el.style.gridColumn = 'span 2';
  const ellKv = kv(ellPanel, ['主軸 u₁', '主軸 u₂']);
  ellKv.el.style.gridColumn = '1 / -1';
  void sBar;

  // step 5: posture comparison table
  const postureBox = el('div', { style: { margin: '0.4rem 0', overflowX: 'auto' } });
  host.appendChild(postureBox);
  const table = el('table', { style: { borderCollapse: 'collapse', fontSize: '0.8rem', width: '100%' } });
  postureBox.appendChild(table);
  const cellStyle = { padding: '0.12rem 0.45rem', borderBottom: `1px solid ${cssVar('--border', '#ddd')}`, textAlign: 'right', fontFamily: cssVar('--mono') || 'monospace' };
  table.appendChild(el('tr', {}, ...['姿勢（點選套用）', 'τ₁ (N·m)', 'τ₂ (N·m)', '‖τ‖'].map((h, i) => el('th', { style: { ...cellStyle, textAlign: i ? 'right' : 'left', fontFamily: 'inherit', color: cssVar('--fg-muted') } }, h))));
  const postureRows = [...POSTURES_2R.map((p) => p.label), '目前姿勢'].map((label, i) => {
    const btn = i < POSTURES_2R.length
      ? el('button', { class: 'mr-chip', type: 'button', style: { padding: '0.05rem 0.55rem', fontSize: '0.75rem' }, onClick: () => { setModel('2R'); setConfig(POSTURES_2R[i].th); } }, label)
      : el('span', { style: { fontWeight: '600' } }, label);
    const cells = [0, 1, 2].map(() => el('td', { style: cellStyle }, '–'));
    const tr = el('tr', {}, el('td', { style: { ...cellStyle, textAlign: 'left', fontFamily: 'inherit' } }, btn), ...cells);
    table.appendChild(tr);
    return { tr, cells };
  });

  // controls
  const panel = controlPanel(host, { compact: true });
  const chipsRow = row(panel);
  const optRow = row(panel);
  const selModel = select<ModelKey>(optRow, {
    value: '2R',
    options: [{ value: '2R', label: '平面 2R（1, 1 m）' }, { value: '3R', label: '平面 3R（1, 0.8, 0.6 m）' }],
    onChange: (v) => { chipGroups[step]?.clear(); setModel(v); },
  });
  const selConv = select<Conv>(optRow, {
    value: 'env',
    options: [{ value: 'env', label: '箭頭 = 末端施於環境的 ℱ（教材）' }, { value: 'ext', label: '箭頭 = 環境施於末端的 ℱ_ext' }],
    onChange: (v) => { chipGroups[step]?.clear(); conv = v; update(); },
  });
  const tVel = toggle(optRow, { label: '速度橢圓', value: show.vel, onChange: (v) => { show.vel = v; cv.requestRender(); } });
  const tForce = toggle(optRow, { label: '力橢圓', value: show.force, onChange: (v) => { show.force = v; cv.requestRender(); } });
  const tArms = toggle(optRow, { label: '力臂', value: show.arms, onChange: (v) => { show.arms = v; update(); } });
  const tGhost = toggle(optRow, { label: '另一組 IK 解', value: show.ghost, onChange: (v) => { show.ghost = v; update(); } });
  const sliderPanel = controlPanel(host, { compact: true });
  sliderPanel.style.gridTemplateColumns = 'repeat(auto-fit, minmax(125px, 1fr))';
  const sTh = [0, 1, 2].map((i) => angleSlider(sliderPanel, {
    label: `θ${'₁₂₃'[i]}`, value: th[i] ?? 60 * D, max: i ? Math.PI + 1e-6 : Math.PI, onInput: (v) => setJoint(i, v),
  }));
  const sFm = slider(sliderPanel, { label: '|f|', min: 0, max: F_MAX, step: 0.1, value: vnorm(f), unit: 'N', onInput: (v) => setForcePolar(v, sFa.value) });
  const sFa = angleSlider(sliderPanel, { label: '∠f', value: Math.atan2(f[1], f[0]), onInput: (v) => setForcePolar(vnorm(f), v) });

  note(host, '比例：力 10 N ↔ 0.25 m（箭頭與力橢圓共用），速度 1 m/s ↔ 0.4 m。忽略連桿自重；3R 拖曳末端時固定 θ₃。');

  // ---------------------------------------------------------------- update
  function update(): void {
    const fe = fEnv();
    res = staticTorques(model, th, fe);
    J = tipJacobian(L, th);
    ax = ellipseAxes(J);
    pts = planarFK(L, th).joints.map((p) => [p[0], p[1]] as Pt);
    const br = elbowBranch(L, th);
    if (br !== 0) branch = br;
    ghostTh = show.ghost ? otherSolution(L, th) : null;
    const tau = res.tau;
    const tn = vnorm(tau);
    rNorm.set(`${fmt(tn, 2)} N·m`);
    rF.set(`${fmt(vnorm(f), 1)} N`);
    rConv.set(conv === 'env' ? 'τ = Jᵀℱ' : 'τ = −Jᵀℱ_ext');
    rD.el.style.display = show.arms ? '' : 'none';
    rGhost.el.style.display = ghostTh ? '' : 'none';
    if (ghostTh) rGhost.set(`(${staticTorques(model, ghostTh, fe).tau.map((x) => fmt(x, 1)).join(', ')}) N·m`);
    if (show.arms) rD.set(`(${momentArms(L, th, f).map((a) => fmt(a.d, 2)).join(', ')}) m`);
    (key === '3R' ? bars3 : bars2).set(tau);
    sTh.forEach((s, i) => { if (i < th.length) s.set(th[i], { silent: true }); });
    sFm.set(vnorm(f), { silent: true });
    if (vnorm(f) > 1e-9) sFa.set(Math.atan2(f[1], f[0]), { silent: true });
    if (step === 1) {
      const pw = powers(model, th, qdot.slice(0, L.length), fe);
      rPj.set(pw.joint, 3);
      rPt.set(pw.tip, 3);
      const w = (F: number[]) => `(${F.slice(0, 3).map((x) => fmt(x, 1)).join(', ')} ; ${F.slice(3).map((x) => fmt(x, 1)).join(', ')})`;
      const v = (x: number[]) => `(${x.map((y) => fmt(y, 2)).join(', ')})`;
      frameKv.set('ℱ_b（{b} 座標）', w(res.Fb));
      frameKv.set('ℱ_s = [Ad_Tbs]ᵀ ℱ_b', w(res.Fs));
      frameKv.set('J_bᵀℱ_b ＝ J_sᵀℱ_s ✓', `${v(res.tau)} ＝ ${v(res.tauSpace)}`);
      frameKv.set('J_sᵀℱ_b ✗ 座標系配錯', v(res.tauWrong));
    }
    if (step === 2 || step === 3) {
      const f1 = ax.s1 > 1e-9 ? tauBar / ax.s1 : Infinity, f2 = ax.s2 > 1e-6 * ax.s1 ? tauBar / ax.s2 : Infinity;
      ellKv.set('主軸 u₁', `速度 σ₁ = ${fmt(ax.s1, 3)} m/s ｜ 力 τ_max/σ₁ = ${Number.isFinite(f1) ? fmt(f1, 1) : '∞'} N`);
      ellKv.set('主軸 u₂', `速度 σ₂ = ${fmt(ax.s2, 3)} m/s ｜ 力 τ_max/σ₂ = ${Number.isFinite(f2) ? fmt(f2, 1) : '∞'} N`);
      rInside.set(tn <= tauBar + 1e-9 ? `是：‖τ‖ ≤ τ_max = ${fmt(tauBar, 0)} N·m` : `否：‖τ‖ 超過 τ_max = ${fmt(tauBar, 0)} N·m`);
    }
    if (step === 4) {
      const rows = [...POSTURES_2R.map((p) => (key === '2R' ? staticTorques(MODELS['2R'], p.th, fe).tau : null)), key === '2R' ? tau : null];
      rows.forEach((t, i) => {
        const r = postureRows[i];
        const cur = i < POSTURES_2R.length && key === '2R' && POSTURES_2R[i].th.every((a, k) => Math.abs(wrapAngle(a - th[k])) < 0.5 * D);
        r.tr.style.background = cur || i === POSTURES_2R.length ? cssVar('--accent-soft', 'rgba(37,99,235,0.08)') : '';
        if (t) { r.cells[0].textContent = fmt(t[0], 1); r.cells[1].textContent = fmt(t[1], 1); r.cells[2].textContent = fmt(vnorm(t), 1); }
        else r.cells.forEach((c) => { c.textContent = '–'; });
      });
    }
    cv.requestRender();
  }

  // ---------------------------------------------------------------- actions
  function setConfig(q: number[]): void { th = q.map(wrapAngle); update(); }

  function setModel(k: ModelKey): void {
    if (k === key) return;
    key = k;
    model = MODELS[k];
    L = lengthsOf(k);
    th = k === '3R' ? [th[0], th[1], 60 * D] : [th[0], th[1]];
    selModel.set(k, { silent: true });
    sTh[2].el.style.display = k === '3R' ? '' : 'none';
    qdSliders[2].el.style.display = k === '3R' ? '' : 'none';
    bars2.el.style.display = k === '2R' ? '' : 'none';
    bars3.el.style.display = k === '3R' ? '' : 'none';
    update();
  }

  function setJoint(i: number, v: number): void {
    if (i >= L.length) return; // hidden θ3 slider while the 2R is shown
    chipGroups[step]?.clear();
    th = th.slice();
    th[i] = v;
    update();
  }

  function setForce(v: Pt): void {
    const m = Math.hypot(v[0], v[1]);
    f = m > F_MAX ? [(v[0] * F_MAX) / m, (v[1] * F_MAX) / m] : [v[0], v[1]];
    update();
  }
  function setForcePolar(m: number, a: number): void {
    chipGroups[step]?.clear();
    setForce([m * Math.cos(a), m * Math.sin(a)]);
  }
  const setConv = (c: Conv) => { conv = c; selConv.set(c, { silent: true }); };

  // ---------------------------------------------------------------- handles (force first so a short arrow can still be grabbed)
  cv.addHandle({
    get: () => { const t = pts[pts.length - 1] ?? [0, 0]; return [t[0] + f[0] * K_F, t[1] + f[1] * K_F]; },
    set: (p) => { chipGroups[step]?.clear(); const t = pts[pts.length - 1]; setForce([(p[0] - t[0]) / K_F, (p[1] - t[1]) / K_F]); },
    radius: 8,
    color: cssVar('--warn', '#b45309'),
  });
  cv.addHandle({
    get: () => pts[pts.length - 1] ?? [0, 0],
    set: (p) => { chipGroups[step]?.clear(); th = solveTip(L, th, p, branch); update(); },
    radius: 9,
  });

  // ---------------------------------------------------------------- drawing
  cv.draw = (c) => {
    const muted = cssVar('--fg-muted', '#777'), accent = cssVar('--accent', '#2563eb');
    const warn = cssVar('--warn', '#b45309'), rose = cssVar('--series-2', '#e11d48'), purple = cssVar('--series-5', '#7c3aed');
    const bgElev = cssVar('--bg-elev', '#fff');
    const jointColors = [cssVar('--series-1', '#2563eb'), cssVar('--series-2', '#e11d48'), cssVar('--series-3', '#16a34a')];
    c.grid(0.5);
    c.circle([0, 0], L.reduce((a, b) => a + b, 0), { color: cssVar('--grid-strong', '#ccc'), dash: [4, 4], width: 1 });
    c.frame({ x: 0, y: 0, theta: 0 }, 0.3, { alpha: 0.6 });
    const tip = pts[pts.length - 1];
    const fe = fEnv();

    if (ghostTh) {
      drawPlanarArm(c, L, ghostTh, { alpha: 0.28 });
      const gj = planarFK(L, ghostTh).joints;
      const ge = Math.hypot(gj[1][0], gj[1][1]) || 1;
      labelBeside(c, [gj[1][0], gj[1][1]], '另一組解', { color: muted, bg: bgElev }, [gj[1][0] / ge, -gj[1][1] / ge], 10);
    }

    // line of action and moment arms
    if (show.arms && vnorm(f) > 1e-6) {
      const m = vnorm(f), fh: Pt = [f[0] / m, f[1] / m];
      c.line([tip[0] - fh[0] * 6, tip[1] - fh[1] * 6], [tip[0] + fh[0] * 6, tip[1] + fh[1] * 6], { color: muted, dash: [5, 5], width: 1 });
      momentArms(L, th, f).forEach((a, i) => {
        const name = `d${'₁₂₃'[i]}`;
        if (Math.abs(a.d) < 1e-3) {
          c.point(a.joint, 5, { color: cssVar('--ok', '#15803d') });
          labelInside(c, a.joint, `${name} = 0`, { color: cssVar('--ok', '#15803d'), dx: -22, dy: -14, bg: bgElev, font: 'bold 11px sans-serif' });
          return;
        }
        c.line(a.joint, a.foot, { color: jointColors[i], width: 1.5, dash: [2, 3] });
        c.point(a.foot, 3, { color: jointColors[i] });
        const mid: Pt = [(a.joint[0] + a.foot[0]) / 2, (a.joint[1] + a.foot[1]) / 2];
        const sgn = fh[0] * mid[0] + fh[1] * mid[1] >= 0 ? 1 : -1; // put the label on the side away from the base
        labelInside(c, mid, name, { color: jointColors[i], bg: bgElev, font: 'bold 11px sans-serif', dx: sgn * 11 * fh[0], dy: -sgn * 11 * fh[1] });
      });
    }

    drawPlanarArm(c, L, th, { eeRadius: 5 });

    // joint torques as arcs: sweep ∝ |τ_i|, arrow head gives the sense (CCW = positive)
    pts.slice(0, L.length).forEach((p, i) => {
      const t = res.tau[i];
      const mag = Math.min(1, Math.abs(t) / TAU_BAR_MAX);
      const phi = th.slice(0, i + 1).reduce((s, x) => s + x, 0);
      const a0 = phi + Math.PI / 2; // start perpendicular to the outgoing link
      const r = i === 0 ? 0.26 : 0.2;
      const col = jointColors[i];
      if (mag > 0.004) {
        const sg = Math.sign(t);
        const a1 = a0 + sg * (0.25 + mag * 1.75 * Math.PI);
        c.arc(p, r, Math.min(a0, a1), Math.max(a0, a1), { color: col, width: 2 + 3 * mag });
        // arrow head: a small triangle tangent to the arc at its end
        const end: Pt = [p[0] + r * Math.cos(a1), p[1] + r * Math.sin(a1)];
        const tg: Pt = [-Math.sin(a1) * sg, Math.cos(a1) * sg];
        const nr: Pt = [Math.cos(a1), Math.sin(a1)];
        const hl = 0.11, hw = 0.055;
        c.polygon([
          [end[0] + tg[0] * hl * 0.6, end[1] + tg[1] * hl * 0.6],
          [end[0] - tg[0] * hl * 0.4 + nr[0] * hw, end[1] - tg[1] * hl * 0.4 + nr[1] * hw],
          [end[0] - tg[0] * hl * 0.4 - nr[0] * hw, end[1] - tg[1] * hl * 0.4 - nr[1] * hw],
        ], { fill: col, color: col, width: 1 });
      }
      // label on the exterior side of the joint (away from both links); values are on the bars below
      const uo: Pt = [Math.cos(phi), Math.sin(phi)];
      let dir: Pt;
      if (i === 0) dir = [-uo[0], -uo[1] - 0.8];
      else {
        const pin = phi - th[i];
        dir = [Math.cos(pin) - uo[0], Math.sin(pin) - uo[1]];
        if (Math.hypot(dir[0], dir[1]) < 0.3) dir = [-uo[1], uo[0]];
      }
      const dm = Math.hypot(dir[0], dir[1]) || 1;
      const lp: Pt = [p[0] + (dir[0] / dm) * (r + 0.2), p[1] + (dir[1] / dm) * (r + 0.2)];
      labelInside(c, lp, `τ${'₁₂₃'[i]}`, { color: col, bg: bgElev, font: 'bold 12px sans-serif' });
    });

    // ellipses at the tip
    const ang = Math.atan2(ax.u1[1], ax.u1[0]);
    if (show.vel) {
      c.ellipse(tip, ax.s1 * K_V, ax.s2 * K_V, ang, { fill: accent, alpha: 0.1, width: 0 });
      c.ellipse(tip, ax.s1 * K_V, ax.s2 * K_V, ang, { color: accent, width: 2 });
      const e1: Pt = [tip[0] + ax.u1[0] * ax.s1 * K_V, tip[1] + ax.u1[1] * ax.s1 * K_V];
      c.text(e1, '速度', { color: accent, dx: 14 * ax.u1[0], dy: -14 * ax.u1[1] });
    }
    if (show.force) {
      const raw1 = ax.s1 > 1e-9 ? (tauBar / ax.s1) * K_F : Infinity;
      const raw2 = ax.s2 > 1e-9 ? (tauBar / ax.s2) * K_F : Infinity;
      const a1 = Math.min(ELLIPSE_CAP, raw1), a2 = Math.min(ELLIPSE_CAP, raw2);
      c.ellipse(tip, a1, a2, ang, { fill: rose, alpha: 0.08, width: 0 });
      c.ellipse(tip, a1, a2, ang, { color: rose, width: 2, dash: raw2 > ELLIPSE_CAP ? [7, 4] : undefined });
      const out = ax.u2[0] * tip[0] + ax.u2[1] * tip[1] >= 0 ? 1 : -1;
      const e2: Pt = [tip[0] + out * ax.u2[0] * Math.min(a2, 1.0), tip[1] + out * ax.u2[1] * Math.min(a2, 1.0)];
      labelInside(c, e2, raw2 > ELLIPSE_CAP ? '力橢圓（此方向 → ∞）' : `力橢圓（‖τ‖ ≤ ${fmt(tauBar, 0)} N·m）`, { color: rose, bg: bgElev, dy: -12 });
    }

    // virtual tip velocity (power step)
    if (step === 1) {
      const pw = powers(model, th, qdot.slice(0, L.length), fe);
      const e: Pt = [tip[0] + pw.v[0] * K_V, tip[1] + pw.v[1] * K_V];
      c.arrow(tip, e, { color: purple, width: 2.5, dash: [5, 3] });
      const vm = Math.hypot(pw.v[0], pw.v[1]) || 1;
      labelBeside(c, e, '𝒱 = J θ̇（虛擬）', { color: purple, bg: bgElev }, [pw.v[0] / vm, -pw.v[1] / vm]);
    }

    // the force arrow (as drawn by the learner) and its reaction
    if (vnorm(f) > 1e-6) {
      const m = vnorm(f);
      const head: Pt = [tip[0] + f[0] * K_F, tip[1] + f[1] * K_F];
      // perpendicular to the arrow, on the side facing away from the base (keeps labels off the links)
      let wp: Pt = [f[1] / m, -f[0] / m];
      if (wp[0] * tip[0] + wp[1] * tip[1] < 0) wp = [-wp[0], -wp[1]];
      const side: Pt = [wp[0], -wp[1]]; // screen coordinates (y down)
      if (step === 0 || step === 5) {
        // the reaction (what the environment does to the tip) only where the convention matters
        const back: Pt = [tip[0] - f[0] * K_F * 0.6, tip[1] - f[1] * K_F * 0.6];
        c.arrow(tip, back, { color: muted, width: 1.5, dash: [4, 3], alpha: 0.7 });
        const rl = step === 0 ? '反作用' : conv === 'env' ? '反作用：環境 → 末端' : '反作用：末端 → 環境';
        labelBeside(c, back, rl, { color: muted, font: '11px sans-serif', bg: bgElev }, [-f[0] / m, f[1] / m]);
      }
      c.arrow(tip, head, { color: warn, width: 3.5, head: 13 });
      // narrow canvases (phones) get the short symbol; the select box spells out the convention
      const wide = c.width >= 460;
      const fl = conv === 'env' ? (wide ? 'ℱ（末端 → 環境）' : 'ℱ') : (wide ? 'ℱ_ext（環境 → 末端）' : 'ℱ_ext');
      labelBeside(c, head, fl, { color: warn, bg: bgElev, font: wide ? undefined : 'bold 13px sans-serif' }, side);
    }

    const degs = th.map((t) => `${fmt(t / D, 0)}°`).join(', ');
    c.overlay.textContent = `${key}  θ = (${degs})  f = (${fmt(f[0], 1)}, ${fmt(f[1], 1)}) N  ${ax.rank < 2 ? '（奇異構型）' : ''}`;
  };

  // ---------------------------------------------------------------- per-step presets
  const along = (q: number[], deg: number, mag = 20): (() => void) => () => {
    setModel('2R');
    setConv('env');
    const a = q.slice(0, 2).reduce((s, x) => s + x, 0) + deg * D;
    th = q.map(wrapAngle);
    setForce([mag * Math.cos(a), mag * Math.sin(a)]);
  };
  // elbow-up pose for step 1: the two moment arms sit at clearly different heights
  const POSE0 = [60 * D, -90 * D];
  const chipSpecs: { label: string; apply: () => void }[][] = [
    [
      { label: '托住 2 kg 重物', apply: () => { setModel('2R'); setConv('env'); th = [...POSE0]; setForce([0, 2 * G]); } },
      { label: '水平推牆', apply: () => { setModel('2R'); setConv('env'); th = [...POSE0]; setForce([20, 0]); } },
      {
        label: '力穿過關節 2',
        apply: () => {
          setModel('2R'); setConv('env'); th = [...POSE0];
          const j = planarFK(L, th).joints;
          const d: Pt = [j[2][0] - j[1][0], j[2][1] - j[1][1]];
          const m = Math.hypot(d[0], d[1]);
          setForce([(20 * d[0]) / m, (20 * d[1]) / m]);
        },
      },
    ],
    [
      { label: 'θ̇ = (1, 0)', apply: () => { qdot = [1, 0, 0]; syncQd(); } },
      { label: 'θ̇ = (0, 1)', apply: () => { qdot = [0, 1, 0]; syncQd(); } },
      { label: 'θ̇ = (0.7, −0.7)', apply: () => { qdot = [0.7, -0.7, 0]; syncQd(); } },
    ],
    [
      { label: '肘部 90°', apply: () => { setModel('2R'); setConfig([20 * D, 90 * D]); } },
      { label: 'θ₂ = 30°', apply: () => { setModel('2R'); setConfig([20 * D, 30 * D]); } },
      { label: 'θ₂ = 10°', apply: () => { setModel('2R'); setConfig([20 * D, 10 * D]); } },
      { label: '3R', apply: () => { setModel('3R'); setConfig([20 * D, 60 * D, 60 * D]); } },
    ],
    [
      { label: '伸直＋沿連桿施力', apply: along([30 * D, 0], 0) },
      { label: '伸直＋偏 10°', apply: along([30 * D, 0], 10) },
      { label: '伸直＋垂直施力', apply: along([30 * D, 0], 90) },
      { label: '彎曲＋沿前臂施力', apply: along([30 * D, 70 * D], 0) },
    ],
    [
      { label: '負載 2 kg（往上托）', apply: () => { setModel('2R'); setConv('env'); th = [0, Math.PI / 2]; setForce([0, 2 * G]); } },
    ],
    [
      { label: 'ℱ = 往上托（教材約定）', apply: () => { setModel('2R'); setConv('env'); th = [10 * D, 60 * D]; setForce([0, 2 * G]); } },
      { label: 'ℱ_ext = 重量（負載約定）', apply: () => { setModel('2R'); setConv('ext'); th = [10 * D, 60 * D]; setForce([0, -2 * G]); } },
      { label: '重量直接套 τ = Jᵀℱ（錯）', apply: () => { setModel('2R'); setConv('env'); th = [10 * D, 60 * D]; setForce([0, -2 * G]); } },
    ],
  ];
  function syncQd(): void { qdSliders.forEach((s, i) => s.set(qdot[i], { silent: true })); update(); }
  const chipGroups: PresetChipsHandle[] = chipSpecs.map((specs) => {
    const g = presetChips(chipsRow, specs, { label: '預設：' });
    g.el.style.display = 'none';
    return g;
  });

  const VIEW: { vel: boolean; force: boolean; arms: boolean; ghost: boolean }[] = [
    { vel: false, force: false, arms: true, ghost: false },
    { vel: false, force: false, arms: false, ghost: false },
    { vel: true, force: true, arms: false, ghost: false },
    { vel: true, force: true, arms: true, ghost: false },
    { vel: false, force: false, arms: false, ghost: true },
    { vel: false, force: false, arms: false, ghost: true },
  ];
  const barsWrap = rp;
  const focusFor = (i: number): HTMLElement[] => [
    [cv.el, barsWrap], [powerPanel], [ellPanel], [barsWrap], [postureBox], [selConv.el, barsWrap],
  ][i] ?? [];
  let focused: HTMLElement[] = [];

  const onStep = (i: number): void => {
    const k = Math.max(0, Math.min(steps.length - 1, i));
    step = k;
    chipGroups.forEach((g, j) => { g.el.style.display = j === k ? '' : 'none'; });
    const v = VIEW[k];
    tVel.set(v.vel, { silent: true }); show.vel = v.vel;
    tForce.set(v.force, { silent: true }); show.force = v.force;
    tArms.set(v.arms, { silent: true }); show.arms = v.arms;
    tGhost.set(v.ghost, { silent: true }); show.ghost = v.ghost;
    powerPanel.style.display = k === 1 ? '' : 'none';
    ellPanel.style.display = k === 2 || k === 3 ? '' : 'none';
    postureBox.style.display = k === 4 ? '' : 'none';
    barsBox.style.display = k === 1 || k === 4 ? 'none' : '';
    chipGroups[k].activate(0);
    focused.forEach((f2) => f2.classList.remove('mr-focus'));
    focused = focusFor(k);
    focused.forEach((f2) => f2.classList.add('mr-focus'));
    update();
  };

  // initial state
  sTh[2].el.style.display = 'none';
  qdSliders[2].el.style.display = 'none';
  bars3.el.style.display = 'none';
  onStep(ctx.currentStep);

  return {
    onStep,
    destroy() { cv.destroy(); },
  };
}

/** Centered text whose anchor is pulled back inside the visible canvas so long labels never get clipped. */
function labelInside(c: Canvas2D, p: Pt, str: string, style: TextStyle): void {
  c.ctx.font = style.font ?? `12px ${cssVar('--font') || 'sans-serif'}`;
  const half = c.ctx.measureText(str).width / 2 + 8;
  const [sx, sy] = c.toScreen(p);
  const x = Math.min(c.width - half, Math.max(half, sx + (style.dx ?? 0)));
  const y = Math.min(c.height - 12, Math.max(12, sy + (style.dy ?? 0)));
  c.text(c.toWorld(x, y), str, { ...style, dx: 0, dy: 0 });
}

/** Label placed next to an anchor along a screen-space direction (e.g. beside an arrow head), kept inside the canvas. */
function labelBeside(c: Canvas2D, p: Pt, str: string, style: TextStyle, dir: Pt, gap = 8): void {
  c.ctx.font = style.font ?? `12px ${cssVar('--font') || 'sans-serif'}`;
  const halfW = c.ctx.measureText(str).width / 2 + 4;
  const off = Math.abs(dir[0]) * halfW + Math.abs(dir[1]) * 9 + gap;
  labelInside(c, p, str, { ...style, dx: dir[0] * off, dy: dir[1] * off });
}

const lesson: Lesson = { meta, steps, quiz, mount };
export default lesson;
