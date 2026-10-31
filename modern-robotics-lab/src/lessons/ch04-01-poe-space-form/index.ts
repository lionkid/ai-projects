import type { Lesson, LessonContext, LessonHandle, LessonStep, QuizQuestion } from '@/types';
import * as THREE from 'three';
import {
  el, fmt, controlPanel, row, slider, angleSlider, toggle, button, matrixView, simTitle, note, kv,
  presetChips, subscript, setThemedColor, themedStandardMaterial, Scene3D, Link3D, createLoop,
  type Frame3D, type SliderHandle,
} from '@/ui';
import { type Mat, type Vec, Adjoint, FKinSpace, clamp, cross, eye, getCol, matMul, matVec } from '@/mr';
import { meta } from './meta';
import { MODELS, MODEL_ORDER, type ChainModel, type ModelKey } from './models';
import {
  axisPlacement, carry, chainTransforms, factorIndex, leftToRightPartial, movedAxes, originGap, rehome,
  reversedTransforms, rightToLeftPartial, transformGap,
} from './poe';
import { AxisGlyph, ChainView, disposeObject, makeFrame, makeGrid, textSprite } from './view3d';
import { type FactorState, circled, factorStrip, fmt3, fmtSci, screwTable } from './panels';

// ---------------------------------------------------------------------------
// Narrative
// ---------------------------------------------------------------------------

const steps: LessonStep[] = [
  {
    title: '一個關節就是一條螺旋軸',
    md: `先看只有一個旋轉關節的手臂。關節角為 0 時，末端座標系 $\\{b\\}$ 相對 $\\{s\\}$ 的位姿記成 $M$。把關節轉 $\\theta$，等於讓整根連桿連同 $\\{b\\}$ 繞關節軸做一次螺旋運動：

$$T(\\theta) = e^{[\\vS]\\theta}M, \\qquad \\vS = \\begin{bmatrix}\\hat\\omega \\\\ -\\hat\\omega\\times q\\end{bmatrix}$$

$\\hat\\omega$ 是軸的方向，$q$ 是軸上任意一點；沿著軸移動 $q$ 不會改變 $\\hat\\omega\\times q$。本例 $\\hat\\omega = (0,1,0)$、$q = (0.2, 0, 0.3)$，得到 $v = (-0.3, 0, 0.2)$：軸沒有穿過 $\\{s\\}$ 的原點，$v$ 就不是零。`,
    simHint: '拖動 θ₁：{b} 繞橘色虛線軸轉動，淡色的 M 留在原地。下方的 S₁、q 與 v 讀數完全不隨 θ 改變。',
  },
  {
    title: '零位姿：寫下 M 與每條 S_i',
    md: `關節再多，也只需要在**零位姿**（所有關節變數為 0）量一次：

1. 末端座標系的位姿 $M \\in \\SE(3)$；
2. 每個關節的軸，用 $\\{s\\}$ 的座標寫成螺旋軸 $\\vS_i$。

$$\\text{R：}\\ \\vS_i = (\\hat\\omega_i,\\ -\\hat\\omega_i\\times q_i) \\qquad \\text{P：}\\ \\vS_i = (0,\\ \\hat v_i)$$

空間 RRR 的底座軸是穿過原點的鉛直線，所以 $\\vS_1 = (0,0,1,0,0,0)$；肩軸與肘軸都平行 $\\hat y$、高 0.3 m，肘軸再往前 0.4 m。此時每個指數都是單位矩陣，$T(0) = M$。`,
    simHint: '手臂停在零位姿。點表格的欄標題「S₃」，用 $q_3 = (0.4, 0, 0.3)$ 手算 $-\\hat\\omega_3 \\times q_3$，核對第 3 欄的 $v$。',
  },
  {
    title: '為什麼是從右邊乘起',
    md: `$n$ 個關節的正向運動學是

$$T(\\theta) = e^{[\\vS_1]\\theta_1}\\,e^{[\\vS_2]\\theta_2}\\cdots e^{[\\vS_n]\\theta_n}\\,M$$

由右往左讀：先只轉關節 $n$，其他關節都還在 0，它的軸正好在零位置 $\\vS_n$；再轉關節 $n-1$，把轉好的末段整個帶走；最後轉關節 1。每個因子作用的當下，它上游的關節都還沒動，所以零位姿量到的軸就是當下真正的軸。若改成由左往右、仍套用零位軸，就會繞錯軸轉。`,
    simHint: '按「▸ 下一個因子」從 $e^{[\\vS_3]\\theta_3}$ 開始一次乘一個，每步留下一個淡色座標系 ①②；轉動中的關節，實線軸恰好與虛線重合。再試「由左往右（錯誤示範）」。',
  },
  {
    title: '動一個關節，看看誰跟著動',
    md: `公式裡的 $\\vS_i$ 永遠是零位姿的數字，可是實體的關節軸會被上游關節帶走。關節 $i$ 此刻的軸是

$$\\vS_i^{\\text{now}} = \\big[\\Ad_{e^{[\\vS_1]\\theta_1}\\cdots e^{[\\vS_{i-1}]\\theta_{i-1}}}\\big]\\,\\vS_i$$

只轉 $\\theta_1$：下游的軸 2、3 被帶走，軸 1 不動。只轉最後一個關節：所有軸都不動，只有末端在轉。這組「此刻的軸」就是下一章空間 Jacobian 的各欄。`,
    simHint: '已鎖定「只動 θ₁」：實線（此刻的軸）2、3 離開了虛線（零位軸），表格裡的 $\\vS_i$ 卻不變。再切到「只動 θ₂」「只動 θ₃」比較，也可按「擺動」。',
  },
  {
    title: '換個零位姿試試',
    md: `零位姿是我們選的。若把姿勢 $\\theta_0$ 宣告為新的零位，新的關節變數是 $\\phi = \\theta - \\theta_0$，$M'$ 與 $\\vS_i'$ 都在那個姿勢重新量：

$$M' = T(\\theta_0), \\qquad e^{[\\vS_1']\\phi_1}\\cdots e^{[\\vS_n']\\phi_n}M' = T(\\theta_0 + \\phi)$$

$\\vS_1'$ 一定等於 $\\vS_1$（它沒有上游關節），其餘的軸與 $M$ 則跟著零位姿一起改變。描述換了一套數字，手臂的實際運動完全相同。`,
    simHint: '已套用新零位 θ₀ = (90°, −90°, 90°)：表格換成 S′ᵢ 與 M′，滑桿改成 φ，手臂本身沒有跳動。拖動任一個 φ，看下方的差距一直停在 1e−16 等級。',
  },
  {
    title: '平面 3R、RRRP 與下一課',
    md: `同一套做法適用於任何開鏈。**平面 3R**：三條軸都是 $\\hat z$，關節位於 $(x_i, 0)$，所以 $\\vS_i = (0,0,1,0,-x_i,0)$，$M$ 只是沿 $\\hat x$ 平移 2.4 m。**RRRP**：前三軸鉛直；第 4 關節沿 $\\hat z$ 平移，$\\vS_4 = (0,0,0,0,0,1)$，只有方向、沒有位置；此例的 $M$ 還把工具的 $z$ 軸轉向下方。

下一課把 $M$ 移到乘積的最前面，改用末端座標系描述同一個 FK（body form）。現在先做下方的自我檢測。`,
    simHint: '切換「平面 3R」與「RRRP」，對照表格中 $\\vS_i$ 的規律；在 RRRP 拉動 θ₄（單位 m），只有 $v$ 分量的第 4 欄讓工具上下滑動。',
  },
];

const quiz: QuizQuestion[] = [
  {
    question: '把空間 RRR 的零位改成姿勢 $\\theta_0 = (90^\\circ, -90^\\circ, 90^\\circ)$（新變數 $\\phi = \\theta - \\theta_0$）。下列何者正確？',
    options: [
      '$M$ 與所有 $\\vS_i$ 都不變，只是關節角平移了 $\\theta_0$',
      '$M$ 改變；$\\vS_1$ 不變、$\\vS_2$ 與 $\\vS_3$ 改變；同一個實際姿勢算出的 $T$ 相同',
      '$M$ 不變，三條 $\\vS_i$ 全部改變',
      '$M$ 與 $\\vS_i$ 都改變，所以同一個實際姿勢會算出不同的 $T$',
    ],
    answer: 1,
    explanation: '新的 $M\' = T(\\theta_0)$ 是那個 L 形姿勢的末端位姿，和伸直時的 $M$ 不同。$\\vS_i\'$ 是關節 $i$ 在 $\\theta_0$ 時的軸：關節 1 沒有上游，軸不動；關節 2、3 被前面的關節帶到別處，所以數字改變。兩組參數描述的是同一支手臂，因此 $e^{[\\vS_1\']\\phi_1}\\cdots M\' = T(\\theta_0+\\phi)$；模擬中的差距讀數一直是 $10^{-16}$ 等級。',
  },
  {
    question: 'RRRP 的第 4 關節沿 $+\\hat z$ 平移。若把這個平移關節改放在鏈的最前面（整支手臂一起上下移動，零位時方向同樣是 $+\\hat z$），它的螺旋軸是什麼？',
    options: [
      '$(0,0,1,\\,0,0,0)$',
      '$(0,0,0,\\,0,0,1)$，和放在第 4 個時相同',
      '$(0,0,0,\\,0.6,0,1)$：要加上工具的位置',
      '寫不出來：P 關節只能放在鏈的最後',
    ],
    answer: 1,
    explanation: '平移關節的螺旋軸只有線速度部分：$\\vS = (0, \\hat v)$，$\\hat v$ 是零位時的滑動方向。平移在空間中處處相同，沒有「軸的位置」可言，所以不論它掛在鏈上哪裡，只要零位時方向是 $+\\hat z$，就是 $(0,0,0,0,0,1)$。選項 A 把方向放進了 $\\omega$，那其實是繞 $z$ 軸的旋轉關節。',
  },
  {
    question: '在空間 RRR 上只轉 $\\theta_2$（$\\theta_1 = \\theta_3 = 0$）。哪些關節「此刻的軸」會離開零位置？',
    options: ['只有軸 2', '只有軸 3', '軸 1 與軸 3', '三條軸都會'],
    answer: 1,
    explanation: '關節 2 轉動時帶走的是它下游的連桿，裝在連桿 2 末端的關節 3 軸因此被搬到新位置。軸 2 是繞自己轉，這條線本身不變；軸 1 在上游，完全不受影響。公式裡的 $\\vS_1, \\vS_2, \\vS_3$ 則始終是零位姿的數字。可以在步驟 4 用「只動 θ₂」驗證。',
  },
  {
    question: '為什麼 $T(\\theta) = e^{[\\vS_1]\\theta_1}\\cdots e^{[\\vS_n]\\theta_n}M$ 可以全部使用零位姿量到的軸，不必知道關節 1 轉動後關節 2 的軸跑到哪裡？',
    options: [
      '因為矩陣指數彼此可交換，順序無所謂',
      '由右往左看，套用 $e^{[\\vS_i]\\theta_i}$ 時它上游的關節都還在 0，軸就在零位置；之後左邊的因子再把整段帶走',
      '因為 $\\vS_i$ 會隨著 $\\theta$ 自動更新',
      '因為所有關節軸都互相平行',
    ],
    answer: 1,
    explanation: '矩陣指數一般不可交換（步驟 3 的「錯誤示範」就是反例，連桿會脫節）。正確的讀法是由右往左：每個因子作用時，比它更靠近基座的關節尚未轉動，所以零位姿量到的 $\\vS_i$ 就是當下真正的軸；接著左邊的因子把整段（含已轉好的部分）一起搬走，軸自然被帶到正確位置。',
  },
];

// ---------------------------------------------------------------------------
// Simulation configuration
// ---------------------------------------------------------------------------

const DEG = Math.PI / 180;
/** Joint colours (CSS variables), kept away from the x/y/z axis colours. */
const JOINT_VARS = ['--series-4', '--series-5', '--series-6', '--series-7'];
/** Seconds to animate one factor of the step-through. */
const ANIM_SEC = 0.9;
/** Example "new home" poses per model (step 5). */
const REHOME_EXAMPLE: Record<ModelKey, number[]> = {
  oneR: [-90 * DEG],
  rrr3: [90 * DEG, -90 * DEG, 90 * DEG],
  planar3R: [90 * DEG, 90 * DEG, 90 * DEG],
  rrrp: [90 * DEG, 90 * DEG, 90 * DEG, 0.1],
};

type Order = 'rtl' | 'ltr';

interface Layer {
  group: THREE.Group;
  arm: ChainView;
  ghost: ChainView;
  homeAxes: AxisGlyph[];
  nowAxes: AxisGlyph[];
  bFrame: Frame3D;
  mFrame: Frame3D;
  m2Frame: Frame3D;
  origFrame: Frame3D;
  target: Frame3D;
  stages: Frame3D[];
  gaps: Link3D[];
  lever: THREE.Line;
  qDot: THREE.Mesh;
  qLabel: THREE.Sprite;
}

interface Config {
  model: ModelKey;
  /** Physical joint values (original parametrization). */
  theta: number[];
  rehome?: number[] | null;
  home: boolean;
  now: boolean;
  ghost: boolean;
  through?: boolean;
  stage?: number;
  solo?: number;
  sel?: number;
  /** Camera override (default: the model's camera). */
  camera?: { pos: [number, number, number]; target: [number, number, number] };
}

const fmtJoint = (m: ChainModel, i: number, x: number): string =>
  m.types[i] === 'P' ? `${fmt(x, 3)} m` : `${fmt(x / DEG, 0)}°`;

// ---------------------------------------------------------------------------
// mount
// ---------------------------------------------------------------------------

function mount(host: HTMLElement, ctx: LessonContext): LessonHandle {
  const S = {
    model: 'rrr3' as ModelKey,
    /** Joint variables of the current parametrization (φ once a new home is set). */
    theta: [0, 0, 0] as number[],
    /** New home in original joint coordinates (null = the model's own home). */
    theta0: null as number[] | null,
    showHome: true,
    showNow: false,
    showGhost: true,
    through: false,
    stage: 0,
    order: 'rtl' as Order,
    anim: null as { j: number; u: number } | null,
    autoplay: false,
    solo: -1,
    wiggle: null as { j: number; t: number; base: number } | null,
    sel: -1,
  };
  let step = ctx.currentStep;
  let layer: Layer | null = null;
  let sliders: SliderHandle[] = [];

  const model = (): ChainModel => MODELS[S.model];
  const nJ = (): number => model().types.length;

  // ---- layout -----------------------------------------------------------
  const titleEl = simTitle(host, 'PoE 空間形式：用零位姿的螺旋軸組出手臂', MODELS.rrr3.label);
  const badge = titleEl.querySelector('.badge') as HTMLElement | null;
  const modelChips = presetChips(host, MODEL_ORDER.map((k) => ({
    label: MODELS[k].label,
    title: `切換到 ${MODELS[k].label}`,
    apply: () => configure({
      model: k, theta: MODELS[k].demo, home: S.showHome, now: S.showNow, ghost: S.showGhost,
      through: S.through, stage: S.through ? 1 : undefined, solo: S.solo >= 0 ? 0 : -1,
    }),
  })), { label: '模型' });

  const scene = new Scene3D(host, { aspect: 1.55, grid: false, axes: false, autoStart: false, cameraPos: MODELS.rrr3.camera.pos, target: MODELS.rrr3.camera.target });
  const overlay = el('div', { class: 'mr-overlay', style: { right: '0.5rem', whiteSpace: 'normal' } });
  scene.el.appendChild(overlay);
  note(host, '虛線 = 零位姿量到的軸 Sᵢ（公式裡用的數字）· 實線 = 關節此刻真正的軸 · 淡色 = 零位姿的手臂 · 拖曳可旋轉視角');

  const strip = factorStrip(host);

  // step-through controls (step 3)
  const throughRow = el('div', { class: 'mr-playback', style: { paddingTop: '0.1rem' } });
  host.appendChild(throughRow);
  button(throughRow, { label: '▸ 下一個因子', small: true, variant: 'primary', title: '再乘上一個指數因子（動畫）', onClick: () => nextFactor() });
  button(throughRow, { label: '▶ 從頭播放', small: true, title: '從 M 開始依序套用所有因子', onClick: () => playAll() });
  button(throughRow, { label: '⟲ 回到 M', small: true, title: '一個因子都不套用', onClick: () => resetThrough() });
  const orderChips = presetChips(throughRow, [
    { label: '由右往左（正確）', title: '先轉最後一個關節，最後轉關節 1', apply: () => setOrder('rtl') },
    { label: '由左往右（錯誤示範）', title: '先轉關節 1，之後仍繞零位置的軸轉', apply: () => setOrder('ltr') },
  ]);
  const throughStatus = el('span', { class: 'mr-note', style: { margin: '0' } });
  throughRow.appendChild(throughStatus);

  // single-joint mode (step 4)
  const soloRow = el('div', { class: 'mr-playback', style: { paddingTop: '0.1rem' } });
  host.appendChild(soloRow);
  const soloChips = presetChips(soloRow, [], { label: '只動一個關節' });
  const btnWiggle = button(soloRow, { label: '▶ 擺動', small: true, title: '讓目前可動的關節來回擺動', onClick: () => toggleWiggle() });

  // new home (step 5)
  const rehomeRow = el('div', { class: 'mr-playback', style: { paddingTop: '0.1rem' } });
  host.appendChild(rehomeRow);
  const btnExample = button(rehomeRow, { label: '新零位 θ₀ = 範例', small: true, variant: 'primary', onClick: () => setRehome(REHOME_EXAMPLE[S.model]) });
  button(rehomeRow, { label: '目前姿勢 → 新零位', small: true, title: '把手臂此刻的姿勢宣告為新的零位姿', onClick: () => setRehome(physicalTheta()) });
  button(rehomeRow, { label: '恢復原零位', small: true, onClick: () => setRehome(null) });

  const jointPanel = controlPanel(host, { compact: true });
  jointPanel.style.gridTemplateColumns = 'repeat(auto-fit, minmax(120px, 1fr))';

  const optPanel = controlPanel(host);
  const optRow = row(optPanel);
  const tHome = toggle(optRow, { label: '零位軸 Sᵢ（虛線）', value: true, onChange: (v) => { S.showHome = v; refresh(); } });
  const tNow = toggle(optRow, { label: '此刻的軸（實線）', value: false, onChange: (v) => { S.showNow = v; updateRows(); refresh(); } });
  const tGhost = toggle(optRow, { label: '零位姿殘影', value: true, onChange: (v) => { S.showGhost = v; refresh(); } });

  const tables = el('div', { style: { display: 'flex', flexWrap: 'wrap', alignItems: 'flex-start', gap: '0.2rem 0.4rem', marginTop: '0.2rem' } });
  host.appendChild(tables);
  const sTable = screwTable(tables, '零位姿的螺旋軸 Sᵢ（在 {s} 中）', (j) => { S.sel = S.sel === j ? -1 : j; refresh(); });
  const nowTable = screwTable(tables, '此刻的軸（底色 = 已離開零位置）');
  const mats = el('div', { style: { display: 'flex', flexWrap: 'wrap', alignItems: 'center' } });
  tables.appendChild(mats);
  const mView = matrixView(mats, { label: 'M =', digits: 2 });
  const tView = matrixView(mats, { label: 'T(θ) =', digits: 2 });

  const K_P = '末端位置 p';
  const K_AX = '選定的軸';
  const K_V = 'v = −ω × q';
  const K_TH = 'θ = θ₀ + φ';
  const K_RE = '‖T′(φ) − T(θ₀ + φ)‖';
  const K_GAP = '與正確 T(θ) 相比';
  const info = kv(host, [K_P, K_AX, K_V, K_TH, K_RE, K_GAP]);

  // ---- 3D layer (rebuilt when the model or the home pose changes) ---------
  function buildLayer(): Layer {
    const m = model();
    const n = nJ();
    const u = m.unit;
    const re = !!S.theta0;
    const group = new THREE.Group();
    group.add(makeGrid(m.grid, Math.round(m.grid * 4)));
    const sFrame = makeFrame(0.42 * u, '{s}', { labelHeight: 0.16 * u });
    group.add(sFrame.group);
    const ghost = new ChainView(m.segments, { opacity: 0.18 });
    const arm = new ChainView(m.segments);
    group.add(ghost.group, arm.group);
    const glyph = (j: number, dashed: boolean): AxisGlyph => new AxisGlyph({
      color: JOINT_VARS[j % JOINT_VARS.length],
      dashed,
      back: m.glyph.back * u,
      front: m.glyph.front * u,
      radius: (dashed ? 0.0105 : 0.0125) * u,
      arrow: 0.17 * u,
      ring: dashed && m.types[j] === 'R' ? 0.1 * u : 0,
      label: dashed ? `S${re ? '′' : ''}${subscript(j + 1)}${m.types[j] === 'P' ? '（P）' : ''}` : undefined,
      labelHeight: 0.17 * u,
    });
    const homeAxes = Array.from({ length: n }, (_, j) => glyph(j, true));
    const nowAxes = Array.from({ length: n }, (_, j) => glyph(j, false));
    for (const g of [...homeAxes, ...nowAxes]) group.add(g.group);
    const bFrame = makeFrame(0.4 * u, '{b}', { labelHeight: 0.16 * u });
    const mFrame = makeFrame(0.34 * u, 'M', { opacity: 0.45, labelHeight: 0.15 * u });
    const m2Frame = makeFrame(0.34 * u, 'M′', { opacity: 0.55, labelHeight: 0.15 * u, labelColor: '--warn' });
    const origFrame = makeFrame(0.28 * u, 'M（原）', { opacity: 0.3, labelHeight: 0.13 * u });
    const target = makeFrame(0.34 * u, 'T(θ)', { opacity: 0.4, labelHeight: 0.15 * u, labelColor: '--accent' });
    const stages = Array.from({ length: Math.max(0, n - 1) }, (_, k) => makeFrame(0.28 * u, circled(k + 1), { opacity: 0.45, labelHeight: 0.16 * u }));
    for (const f of [bFrame, mFrame, m2Frame, origFrame, target, ...stages]) group.add(f.group);
    const gaps = Array.from({ length: n }, () => {
      const g = new Link3D(0.012 * u, '--danger');
      group.add(g.mesh);
      return g;
    });
    const leverMat = new THREE.LineDashedMaterial({ dashSize: 0.05 * u, gapSize: 0.035 * u });
    setThemedColor(leverMat, '--fg-muted');
    const lever = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3(1, 0, 0)]), leverMat);
    lever.frustumCulled = false;
    const qDot = new THREE.Mesh(new THREE.SphereGeometry(0.03 * u, 16, 12), themedStandardMaterial('--fg'));
    const qLabel = textSprite('q', '--fg', 0.16 * u);
    group.add(lever, qDot, qLabel);
    scene.scene.add(group);
    return { group, arm, ghost, homeAxes, nowAxes, bFrame, mFrame, m2Frame, origFrame, target, stages, gaps, lever, qDot, qLabel };
  }

  function rebuildLayer(): void {
    if (layer) disposeObject(layer.group);
    layer = buildLayer();
  }

  function setCamera(cam?: Config['camera']): void {
    const c = cam ?? model().camera;
    scene.camera.position.set(c.pos[0], c.pos[1], c.pos[2]);
    scene.controls?.target.set(c.target[0], c.target[1], c.target[2]);
    scene.controls?.update();
    scene.requestRender();
  }

  // ---- joint sliders ----------------------------------------------------
  function limitsNow(i: number): [number, number] {
    const m = model();
    let [lo, hi] = m.limits[i];
    if (S.theta0) { lo -= S.theta0[i]; hi -= S.theta0[i]; }
    if (m.types[i] === 'R') {
      // whole degrees, so the slider grid contains 0
      lo = Math.ceil(lo / DEG - 1e-9) * DEG;
      hi = Math.floor(hi / DEG + 1e-9) * DEG;
    }
    return [lo, hi];
  }

  function buildSliders(): void {
    const m = model();
    jointPanel.replaceChildren();
    sliders = m.types.map((type, i) => {
      const [lo, hi] = limitsNow(i);
      const name = `${S.theta0 ? 'φ' : 'θ'}${subscript(i + 1)}`;
      const onInput = (v: number) => onJoint(i, v);
      return type === 'P'
        ? slider(jointPanel, { label: `${name}（平移）`, min: lo, max: hi, step: 0.005, value: S.theta[i], unit: 'm', format: (v) => fmt(v, 3), onInput })
        : angleSlider(jointPanel, { label: name, min: lo, max: hi, value: S.theta[i], onInput });
    });
    applySolo();
  }

  function syncSliders(): void {
    sliders.forEach((s, i) => s.set(S.theta[i], { silent: true }));
  }

  function applySolo(): void {
    sliders.forEach((s, i) => { s.disabled = S.solo >= 0 && i !== S.solo; });
  }

  function onJoint(i: number, v: number): void {
    if (S.wiggle && S.wiggle.j === i) stopWiggle(false);
    S.theta[i] = v;
    if (S.solo < 0) soloChips.clear();
    modelChips.highlight(MODEL_ORDER.indexOf(S.model));
    refresh();
  }

  /** Joint values in the model's original parametrization. */
  function physicalTheta(): number[] {
    return S.theta.map((x, i) => x + (S.theta0 ? S.theta0[i] : 0));
  }

  // ---- configuration ----------------------------------------------------
  function configure(c: Config): void {
    stopAnim();
    stopWiggle(false);
    S.model = c.model;
    const n = nJ();
    S.theta0 = c.rehome ? c.rehome.slice() : null;
    S.theta = c.theta.map((x, i) => x - (S.theta0 ? S.theta0[i] : 0));
    S.showHome = c.home;
    S.showNow = c.now;
    S.showGhost = c.ghost;
    S.through = !!c.through;
    S.order = 'rtl';
    S.stage = S.through ? clamp(c.stage ?? 0, 0, n) : n;
    S.solo = c.solo ?? -1;
    S.sel = c.sel ?? -1;
    rebuildLayer();
    setCamera(c.camera);
    if (badge) badge.textContent = model().label;
    modelChips.highlight(MODEL_ORDER.indexOf(S.model));
    soloChips.setPresets([
      ...model().types.map((_, i) => ({ label: `只動 θ${subscript(i + 1)}`, apply: () => setSolo(i) })),
      { label: '全部可動', apply: () => setSolo(-1) },
    ]);
    if (S.solo >= 0) soloChips.highlight(S.solo); else soloChips.highlight(n);
    orderChips.highlight(0);
    btnExample.textContent = `新零位 θ₀ = (${REHOME_EXAMPLE[S.model].map((x, i) => fmtJoint(model(), i, x)).join(', ')})`;
    tHome.set(S.showHome, { silent: true });
    tNow.set(S.showNow, { silent: true });
    tGhost.set(S.showGhost, { silent: true });
    buildSliders();
    updateRows();
    refresh();
  }

  function setSolo(i: number): void {
    stopWiggle(true);
    S.solo = i;
    if (i >= 0) {
      // keep the other joints where they are; only joint i may move from now on
      S.showNow = true;
      tNow.set(true, { silent: true });
    }
    applySolo();
    updateRows();
    refresh();
  }

  function setRehome(theta0: number[] | null): void {
    stopAnim();
    stopWiggle(false);
    const phys = physicalTheta();
    S.theta0 = theta0 ? theta0.slice() : null;
    S.theta = phys.map((x, i) => x - (S.theta0 ? S.theta0[i] : 0));
    S.through = false;
    S.stage = nJ();
    rebuildLayer();
    buildSliders();
    updateRows();
    refresh();
  }

  // ---- step-through -------------------------------------------------------
  function startFactor(): void {
    const n = nJ();
    if (S.stage >= n) { S.autoplay = false; return; }
    S.anim = { j: factorIndex(n, S.stage, S.order), u: 0 };
    loop.start();
  }

  function nextFactor(): void {
    stopWiggle(true);
    if (!S.through) { S.through = true; S.stage = 0; updateRows(); }
    S.autoplay = false;
    if (S.anim) { S.stage += 1; S.anim = null; refresh(); return; }
    if (S.stage >= nJ()) S.stage = 0;
    startFactor();
    refresh();
  }

  function playAll(): void {
    stopWiggle(true);
    S.through = true;
    S.stage = 0;
    S.anim = null;
    S.autoplay = true;
    updateRows();
    startFactor();
    refresh();
  }

  function resetThrough(): void {
    S.through = true;
    S.stage = 0;
    S.anim = null;
    S.autoplay = false;
    updateRows();
    refresh();
  }

  function setOrder(o: Order): void {
    S.order = o;
    S.through = true;
    S.stage = 0;
    S.anim = null;
    S.autoplay = false;
    updateRows();
    refresh();
  }

  function stopAnim(): void {
    S.anim = null;
    S.autoplay = false;
    if (!S.wiggle) loop.stop();
  }

  // ---- wiggle (single-joint mode) -----------------------------------------
  function toggleWiggle(): void {
    if (S.wiggle) { stopWiggle(true); refresh(); return; }
    const j = S.solo >= 0 ? S.solo : 0;
    if (S.solo < 0) setSolo(0);
    S.wiggle = { j, t: 0, base: S.theta[j] };
    btnWiggle.textContent = '⏸ 停止擺動';
    loop.start();
  }

  function stopWiggle(restore: boolean): void {
    if (!S.wiggle) return;
    if (restore) S.theta[S.wiggle.j] = S.wiggle.base;
    S.wiggle = null;
    btnWiggle.textContent = '▶ 擺動';
    syncSliders();
    if (!S.anim) loop.stop();
  }

  const loop = createLoop((dt) => {
    if (S.anim) {
      S.anim.u = Math.min(1, S.anim.u + dt / ANIM_SEC);
      if (S.anim.u >= 1) {
        S.stage += 1;
        S.anim = null;
        if (S.autoplay && S.stage < nJ()) startFactor(); else S.autoplay = false;
      }
    }
    if (S.wiggle) {
      const w = S.wiggle;
      w.t += dt;
      const [lo, hi] = limitsNow(w.j);
      const amp = model().types[w.j] === 'P' ? 0.07 : 0.6;
      S.theta[w.j] = clamp(w.base + amp * Math.sin((2 * Math.PI * w.t) / 2.6), lo, hi);
      sliders[w.j]?.set(S.theta[w.j], { silent: true });
    }
    refresh();
    if (!S.anim && !S.wiggle) loop.stop();
  });

  // ---- per-step rows --------------------------------------------------------
  function updateRows(): void {
    throughRow.style.display = step === 2 || S.through ? '' : 'none';
    soloRow.style.display = step === 3 || S.solo >= 0 ? '' : 'none';
    rehomeRow.style.display = step === 4 || S.theta0 ? '' : 'none';
    nowTable.el.style.display = step === 3 || (S.showNow && !S.through) ? '' : 'none';
  }

  // ---- drawing + numbers ----------------------------------------------------
  function refresh(): void {
    const ly = layer;
    if (!ly) return;
    const m = model();
    const n = nJ();
    const re = S.theta0 ? rehome(m.M, m.Slist, S.theta0) : null;
    const Muse = re ? re.M : m.M;
    const Suse = re ? re.Slist : m.Slist;
    const H: Mat[] = S.theta0 ? chainTransforms(m.Slist, S.theta0) : Array.from({ length: n + 1 }, () => eye(4));
    const ltr = S.through && S.order === 'ltr';
    const u = S.anim?.u ?? 0;
    const phi = S.through ? (ltr ? leftToRightPartial(S.theta, S.stage, u) : rightToLeftPartial(S.theta, S.stage, u)) : S.theta.slice();
    const L = ltr ? reversedTransforms(Suse, phi) : chainTransforms(Suse, phi);
    const D = L.map((Li, i) => matMul(Li, H[i]));
    const T = matMul(L[n], Muse);
    const Tfull = FKinSpace(Muse, Suse, S.theta);

    ly.arm.setBodies(D);
    ly.ghost.setBodies(H);
    ly.ghost.setVisible(S.showGhost);
    ly.bFrame.setTransform(T);
    ly.mFrame.setTransform(m.M);
    ly.mFrame.setVisible(!re);
    ly.m2Frame.setTransform(Muse);
    ly.m2Frame.setVisible(!!re);
    ly.origFrame.setTransform(m.M);
    ly.origFrame.setVisible(!!re);

    // axes: dashed = formula (home) axes, solid = where each joint axis is now (on its parent body)
    const nowS: Vec[] = [];
    for (let j = 0; j < n; j++) {
      const Sj = getCol(Suse, j);
      ly.homeAxes[j].set(Sj, carry(H[j], m.anchors[j]));
      ly.homeAxes[j].setVisible(S.showHome);
      ly.homeAxes[j].emphasize(S.sel === j || S.anim?.j === j ? 1.9 : 1);
      const Sn = matVec(Adjoint(L[j]), Sj);
      nowS.push(Sn);
      ly.nowAxes[j].set(Sn, carry(D[j], m.anchors[j]));
      ly.nowAxes[j].setVisible(S.showNow);
    }

    // step-through: intermediate frames ①②… and the correct final pose
    for (let k = 1; k < n; k++) {
      const f = ly.stages[k - 1];
      const show = S.through && k <= S.stage && !(k === S.stage && !S.anim);
      f.setVisible(show);
      if (show) {
        const Tk = ltr
          ? matMul(reversedTransforms(Suse, leftToRightPartial(S.theta, k))[n], Muse)
          : FKinSpace(Muse, Suse, rightToLeftPartial(S.theta, k));
        f.setTransform(Tk);
      }
    }
    ly.target.setVisible(S.through && (S.stage < n || ltr));
    ly.target.setTransform(Tfull);

    // wrong order: show where joints came apart (revolute joints only; a P joint slides on purpose)
    const broken: number[] = [];
    for (let j = 0; j < n; j++) {
      const g = ly.gaps[j];
      if (!ltr || m.types[j] !== 'R') { g.mesh.visible = false; continue; }
      const a = carry(D[j], m.anchors[j]);
      const b = carry(D[j + 1], m.anchors[j]);
      const d = Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
      g.mesh.visible = d > 2e-3;
      if (d > 2e-3) { g.setEndpoints(a, b); broken.push(j + 1); }
    }

    // selected joint: point q on its home axis and the lever from the {s} origin
    const sel = S.sel >= 0 && S.sel < n ? S.sel : -1;
    const selS = sel >= 0 ? getCol(Suse, sel) : null;
    const selR = sel >= 0 && m.types[sel] === 'R';
    let q: Vec = [0, 0, 0];
    if (selS && selR) {
      q = axisPlacement(selS, carry(H[sel], m.anchors[sel])).point;
      (ly.lever.geometry as THREE.BufferGeometry).setFromPoints([new THREE.Vector3(0, 0, 0), new THREE.Vector3(q[0], q[1], q[2])]);
      ly.lever.computeLineDistances();
      ly.qDot.position.set(q[0], q[1], q[2]);
      // label the position vector q at the middle of the lever, nudged towards the viewer
      const off = 0.06 * m.unit;
      ly.qLabel.position.set(q[0] / 2, q[1] / 2 - off, q[2] / 2 + off);
    }
    ly.lever.visible = ly.qDot.visible = ly.qLabel.visible = !!(selS && selR && S.showHome);

    // ---- DOM ----
    const prime = re ? '′' : '';
    strip.build(
      ltr ? '\\widetilde{T} =' : re ? 'T(\\theta_0+\\phi) =' : 'T(\\theta) =',
      [...displayOrder(n, ltr).map((j) => `e^{[\\vS${re ? "'" : ''}_{${j + 1}}]${re ? '\\phi' : '\\theta'}_{${j + 1}}}`), re ? "M'" : 'M'],
    );
    const order = displayOrder(n, ltr);
    const states: FactorState[] = order.map((j) => {
      if (!S.through) return Math.abs(S.theta[j]) < 1e-12 ? 'identity' : 'plain';
      if (S.anim?.j === j) return 'active';
      return (ltr ? j + 1 : n - j) <= S.stage ? 'applied' : 'pending';
    });
    const caps = order.map((j) => (S.through ? circled(ltr ? j + 1 : n - j) : Math.abs(S.theta[j]) < 1e-12 ? '= I' : ''));
    states.push(S.through ? 'applied' : 'plain');
    caps.push(S.through ? circled(0) : '');
    strip.paint(states, caps);

    const heads = m.types.map((_, j) => `S${prime}${subscript(j + 1)}`);
    sTable.setTitle(re ? '新零位姿的螺旋軸 S′ᵢ（在 {s} 中）' : '零位姿的螺旋軸 Sᵢ（在 {s} 中）');
    sTable.set(m.types.map((_, j) => getCol(Suse, j)), heads, JOINT_VARS, { selected: sel });
    if (nowTable.el.style.display !== 'none') {
      nowTable.set(nowS, m.types.map((_, j) => `軸${subscript(j + 1)}`), JOINT_VARS, { marked: movedAxes(Suse, phi) });
    }
    mView.setLabel(re ? 'M′ =' : 'M =');
    mView.set(Muse, { highlightCols: ['x', 'y', 'z', null] });
    tView.setLabel(ltr ? 'T（錯誤順序）=' : re ? 'T(θ₀+φ) =' : 'T(θ) =');
    tView.set(T, { highlightCols: ['x', 'y', 'z', null] });

    info.set(K_P, `${fmt3([T[0][3], T[1][3], T[2][3]], 3)} m`);
    info.show(K_AX, sel >= 0);
    info.show(K_V, sel >= 0 && selR);
    if (selS) {
      if (selR) {
        info.set(K_AX, `${heads[sel]}：方向 ω = ${fmt3(selS.slice(0, 3), 2)}，軸上一點 q = ${fmt3(q, 2)}`);
        info.set(K_V, `${fmt3(cross(q, selS.slice(0, 3)), 2)}（= ${heads[sel]} 的後三個分量）`);
      } else {
        info.set(K_AX, `${heads[sel]}：P 關節，ω = 0，滑動方向 v = ${fmt3(selS.slice(3, 6), 2)}（沒有位置）`);
      }
    }
    info.show(K_TH, !!re);
    info.show(K_RE, !!re);
    if (re && S.theta0) {
      const phys = S.theta.map((x, i) => x + S.theta0![i]);
      info.set(K_TH, `(${phys.map((x, i) => fmtJoint(m, i, x)).join(', ')})`);
      info.set(K_RE, `${fmtSci(transformGap(Tfull, FKinSpace(m.M, m.Slist, phys)))}（兩種描述給出同一個位姿）`);
    }
    info.show(K_GAP, S.through);
    if (S.through) {
      const gap = originGap(T, Tfull);
      info.set(K_GAP, S.stage >= n && !S.anim
        ? (ltr ? `末端偏離 ${fmt(gap, 3)} m ✗` : `‖T − T(θ)‖ = ${fmtSci(transformGap(T, Tfull))} ✓`)
        : `還差 ${n - S.stage} 個因子（末端距 T(θ) ${fmt(gap, 3)} m）`);
    }

    // overlay + step-through status
    const vals = S.theta.map((x, i) => fmtJoint(m, i, x)).join(', ');
    let ov = `${re ? 'φ' : 'θ'} = (${vals})`;
    if (S.through) ov = ltr ? `錯誤示範：由左往右、都繞零位軸 · 已套用 ${S.stage}/${n}` : `由右往左 · 已套用 ${S.stage}/${n} 個因子`;
    if (overlay.textContent !== ov) overlay.textContent = ov;
    let st = '';
    if (S.through && ltr) {
      st = broken.length
        ? `關節 ${broken.join('、')} 脫節（紅色短線）：後面的因子繞的是零位置的虛線軸，不是關節此刻的實線軸。`
        : S.anim ? `正在繞零位置的軸 ${S.anim.j + 1} 轉……` : '先轉關節 1，之後仍繞「零位置」的軸轉下一個關節。';
    } else if (S.through) {
      if (S.anim) st = `正在轉關節 ${S.anim.j + 1}：它上游的關節都還在 0，所以實線軸與虛線（零位軸）重合。`;
      else if (S.stage === 0) st = '從 M 出發：按「▸ 下一個因子」，先轉最後一個關節。';
      else if (S.stage < n) st = `已套用 ${S.stage} 個因子：關節 ${S.stage === 1 ? n : `${n - S.stage + 1}–${n}`} 已轉好，前面的關節仍在 0。`;
      else st = '全部套用完畢：結果與 FKinSpace 完全一致。';
    }
    if (throughStatus.textContent !== st) throughStatus.textContent = st;
    scene.requestRender();
  }

  /** Factor display order: rtl → 1..n; ltr (wrong) → n..1, so the strip always applies from the right. */
  function displayOrder(n: number, ltr: boolean): number[] {
    const idx = Array.from({ length: n }, (_, j) => j);
    return ltr ? idx.reverse() : idx;
  }

  // ---- steps ----------------------------------------------------------------
  const STEP_SETUP: (() => void)[] = [
    () => configure({ model: 'oneR', theta: MODELS.oneR.demo, home: true, now: false, ghost: true, sel: 0 }),
    () => configure({ model: 'rrr3', theta: [0, 0, 0], home: true, now: false, ghost: false, sel: 2 }),
    () => configure({ model: 'rrr3', theta: MODELS.rrr3.demo, home: true, now: true, ghost: true, through: true, stage: 1 }),
    () => configure({ model: 'rrr3', theta: [50 * DEG, 0, 0], home: true, now: true, ghost: true, solo: 0 }),
    () => configure({
      model: 'rrr3', theta: [60 * DEG, -60 * DEG, 110 * DEG], rehome: REHOME_EXAMPLE.rrr3, home: true, now: false, ghost: true,
      camera: { pos: [0.98, -0.95, 0.98], target: [0.16, 0.12, 0.33] },
    }),
    () => configure({ model: 'rrrp', theta: MODELS.rrrp.demo, home: true, now: false, ghost: true }),
  ];

  let focused: HTMLElement[] = [];
  function setFocus(i: number): void {
    focused.forEach((f) => f.classList.remove('mr-focus'));
    const targets: (HTMLElement | null)[][] = [
      [jointPanel, info.el],
      [sTable.el, mView.el],
      [strip.el, throughRow],
      [soloRow, nowTable.el],
      [rehomeRow, info.el],
      [modelChips.el, sTable.el],
    ];
    focused = (targets[i] ?? []).filter((x): x is HTMLElement => !!x);
    focused.forEach((f) => f.classList.add('mr-focus'));
  }

  function onStep(i: number): void {
    step = i;
    STEP_SETUP[i]?.();
    setFocus(i);
  }

  onStep(Math.min(ctx.currentStep, steps.length - 1));

  return {
    onStep,
    destroy() {
      S.anim = null;
      S.wiggle = null;
      loop.stop();
      if (layer) disposeObject(layer.group);
      layer = null;
      scene.destroy();
    },
  };
}

const lesson: Lesson = { meta, steps, quiz, mount };
export default lesson;
