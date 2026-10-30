import type { Lesson, LessonContext, LessonHandle, LessonStep, QuizQuestion } from '@/types';
import {
  el, cssVar, fmt, fmtDeg, controlPanel, row, slider, angleSlider, select, toggle, playback, simTitle, note, kv,
  matrixView, presetChips, split, Canvas2D, Plot2D, Scene3D, createLoop, drawPlanarArm, poseHandle,
  type Pt, type Marker, type SeriesSpec, type Frame3D, type Link3D, type ScrewAxis3D,
} from '@/ui';
import {
  QuinticTimeScaling, ScrewTrajectory, CartesianTrajectory, RotZ, RpToTrans, matMul, clamp, type Mat,
} from '@/mr';
import { meta } from './meta';
import {
  LINKS, PATH_KINDS, planarPaths, compareSE3, wristPoint,
  type Pose2, type PathKind, type Elbow, type IKPolicy, type PlanarPaths, type SE3Comparison,
} from './paths';
import { PRESETS_2D, PRESETS_3D, type Preset2D } from './presets';

const steps: LessonStep[] = [
  {
    title: '三種「直線」',
    md: `給定起點位姿 $X_{\\rm start}$ 與終點位姿 $X_{\\rm end}$，「走直線」至少有三種意思：

1. **關節空間直線**：每個關節角都按同一比例，從起點值走到終點值。
2. **螺旋（screw）內插**：整個剛體繞著同一條螺旋軸、以固定的運動旋量（twist）移動。
3. **解耦的笛卡兒直線**：工具原點走真正的直線，姿態另外沿最短的旋轉內插。

三者端點相同，中間的運動卻不同。右邊的平面 3R 手臂剛好有 3 個自由度對應平面位姿 $(x,y,\\phi)$，只要可達，三種路徑都能精確跟隨。`,
    simHint: '拖曳起點／終點的圓點（位置）與小把手（方向），比較藍（關節）、紫（螺旋）、橘（笛卡兒）三條末端軌跡；按 ▶ 播放看手臂怎麼走。',
  },
  {
    title: '關節空間直線',
    md: `$$\\theta(s)=\\theta_{\\rm start}+s\\,(\\theta_{\\rm end}-\\theta_{\\rm start}),\\qquad s\\in[0,1].$$

只要在兩個端點各解一次 IK，運動途中完全不用解。所有關節同時起步、同時停下，$d\\theta/ds$ 是常數；若兩端都在關節限制內，整條路徑也一定在限制內（限制範圍是個方盒，方盒是凸集合）。

代價是末端走的是**弧線**：正向運動學是非線性的，關節空間裡的直線，映到工作空間就彎了。`,
    simHint: '畫面只剩藍色軌跡：左下 θ(s) 的三條線都是直線、右下 ‖dθ/ds‖ 是水平線，末端卻畫出弧線。',
  },
  {
    title: '螺旋內插',
    md: `把整段相對運動寫成一個運動旋量 $[\\mathcal V]=\\log(X_{\\rm start}^{-1}X_{\\rm end})$，再以固定速率套用：

$$X(s)=X_{\\rm start}\\exp\\big(\\log(X_{\\rm start}^{-1}X_{\\rm end})\\,s\\big).$$

物體繞同一條螺旋軸轉動、同時沿軸平移，所以原點走的是螺旋線。平面上的螺距為零、螺旋軸垂直紙面，原點就沿著以「極點」為圓心的圓弧走。注意必須用 $\\log(X_{\\rm start}^{-1}X_{\\rm end})$；把 $\\log X_{\\rm end}-\\log X_{\\rm start}$ 拿來內插，一般不會得到這條路徑。`,
    simHint: '已切到 3D 視圖：拖 Δz 看原點畫出螺旋、紫色虛線是螺旋軸；按「純平移（α = 0）」，螺旋路徑退化成直線。',
  },
  {
    title: '解耦的笛卡兒直線',
    md: `焊接、塗膠、插銷這類工作在乎工具「走直線」。做法是把位置與姿態分開內插：

$$p(s)=p_0+s\\,(p_1-p_0),\\qquad R(s)=R_0\\exp\\big(\\log(R_0^{\\T}R_1)\\,s\\big).$$

有趣的是，$R(s)$ 與螺旋路徑**完全相同**——兩者的旋轉部分都是 $\\log(R_0^{\\T}R_1)$——差別只在原點怎麼走。代價是：運動途中**每個樣本都要解 IK**，下一步會看到這帶來的風險。`,
    simHint: '只留橘色軌跡，看沿途的工具座標系 R(s)；再切到 3D 視圖，讀「max ‖R_screw − R_cart‖」（恆為 0）與位置差。',
  },
  {
    title: '笛卡兒路徑的代價',
    md: `沿笛卡兒直線逐點解 IK，可能遇到三種麻煩：

- **靠近奇異**：腕點 $w$ 接近 $|w|=L_1+L_2$ 或 $|L_1-L_2|$ 時 $\\sin\\theta_2\\to0$，$\\|d\\theta/ds\\|$ 暴增。因為 $\\dot\\theta=\\theta'(s)\\,\\dot s$，換哪一種時間縮放都消不掉尖峰，只能整段放慢或改路徑。
- **不可達**：直線的一部分落在工作空間外，IK 無解。
- **分支跳變**：IK 有兩組解（手肘上／下）。若每個樣本都從同一個初值重新求解，可能突然換到另一組，關節角瞬間跳動；以上一個樣本的解為初值（warm start）才能保持連續。`,
    simHint: '依序點「靠近基座：θ̇ 尖峰」與「穿過不可達區」，看 ‖dθ/ds‖ 圖與紅色標記；再點「冷啟動 IK」並切換 IK 初值，比較跳變。',
  },
  {
    title: '避障與選擇，以及經過點',
    md: `加上障礙物後要檢查的是**整隻手臂**，不只是末端：末端走直線，不代表連桿掃過的區域安全；反過來，關節空間的弧線也可能剛好繞開。

選擇原則：
- 任務本身要求直線（焊縫、插入）→ 笛卡兒路徑，但要先檢查奇異與可達性。
- 只要從 A 移到 B → 關節空間直線最省事：不必逐點 IK、不怕途中奇異。
- 希望剛體以單一螺旋運動「自然地」邊轉邊移 → 螺旋路徑。

下一課：路徑必須經過好幾個中間點（via points）時，如何把各段平順地接起來。`,
    simHint: '點「障礙物 A」與「障礙物 B」，再自己拖曳紅色障礙物，讀下方表格每條路徑的「碰撞」欄。',
  },
];

const quiz: QuizQuestion[] = [
  {
    question: '焊縫要求末端走直線，但起點與終點之間的直線會經過靠近奇異的區域。最合理的做法是？',
    options: [
      '改用關節空間直線，反正兩端相同',
      '保持笛卡兒直線，但移動工件、調整工具姿態或手肘分支，讓整段直線遠離奇異區',
      '在奇異附近加快速度，快速通過',
      '改用螺旋路徑，它也是直線',
    ],
    answer: 1,
    explanation: '任務規定了直線，關節空間路徑會讓焊道變成弧線；螺旋路徑的原點一般也不走直線。奇異附近 $\\|d\\theta/ds\\|$ 很大，加快只會讓 $\\dot\\theta=\\theta\'(s)\\dot s$ 更大。正確做法是改變幾何，讓整段直線落在遠離奇異的區域（或整段放慢）。',
  },
  {
    question: '在同一個 $s$ 比較螺旋路徑與解耦笛卡兒路徑的位姿，下列何者正確？',
    options: [
      '$R(s)$ 與 $p(s)$ 都相同',
      '$R(s)$ 相同，$p(s)$ 一般不同',
      '$p(s)$ 相同，$R(s)$ 不同',
      '兩者都不同',
    ],
    answer: 1,
    explanation: '$\\log(X_0^{-1}X_1)$ 的旋轉部分就是 $\\log(R_0^{\\T}R_1)$，所以兩條路徑的 $R(s)$ 一樣。位置則不同：螺旋路徑的原點繞軸轉（螺旋線），笛卡兒路徑走直線；只有沒有轉動，或起點原點剛好在螺旋軸上時，兩者才重合。',
  },
  {
    question: '一條關節空間直線路徑途中經過 $\\theta_2=0$（手臂伸直）附近，會發生什麼事？',
    options: [
      '關節速度暴增',
      'IK 無解，路徑中斷',
      '關節速度照常平順；只是在那附近，末端幾乎無法沿手臂方向移動',
      '手肘會自動翻到另一邊',
    ],
    answer: 2,
    explanation: '關節空間路徑直接規定 $\\theta(s)$，$\\dot\\theta=(\\theta_{\\rm end}-\\theta_{\\rm start})\\dot s$ 一直有界，完全不需要反解 Jacobian。奇異只代表 $J\\dot\\theta$ 缺少某個方向（伸直時沒有沿手臂的徑向速度）；只有在「規定末端怎麼走、反求關節」時（例如笛卡兒路徑），奇異才會造成尖峰。',
  },
  {
    question: '某條笛卡兒路徑在奇異附近 $\\|d\\theta/ds\\|$ 出現尖峰。若把時間縮放從五次改成梯形、並把 $T$ 加倍，尖峰處的 $\\|d\\theta/ds\\|$ 會？',
    options: ['減半', '不變；它只由路徑幾何（與 IK）決定', '變成 0', '加倍'],
    answer: 1,
    explanation: '$d\\theta/ds$ 只跟路徑形狀和手臂的 IK 有關，跟時間無關。時間縮放改變的是 $\\dot s$：$\\dot\\theta=\\theta\'(s)\\dot s$。$T$ 加倍讓 $\\dot s$ 約減半，關節速度跟著變小，但尖峰的形狀仍在，根本解法是避開奇異區。',
  },
];

const NAME: Record<PathKind, string> = { joint: '關節空間直線', screw: '螺旋路徑', cartesian: '笛卡兒直線' };
const SHORT: Record<PathKind, string> = { joint: '關節', screw: '螺旋', cartesian: '笛卡兒' };
const VAR: Record<PathKind, string> = { joint: '--series-1', screw: '--series-5', cartesian: '--series-4' };
const colorOf = (k: PathKind) => cssVar(VAR[k], '#36f');
// Fixed colours for the 3D view (three.js materials are not theme-aware); readable on both backgrounds.
const C3 = { screw: 0x8b5cf6, cart: 0xf59e0b, link: 0x9ca3af };

const TDUR = 2; // seconds; quintic time scaling for the playback
const SAMPLES3 = 41; // samples of each SE(3) path (indices 10, 20, 30 = s 0.25, 0.5, 0.75)
const WORLD = { xmin: -1.55, xmax: 2.45, ymin: -0.8, ymax: 1.95 };
const deg = Math.PI / 180;

interface Scene3Parts {
  scene: Scene3D;
  fStart: Frame3D; fEnd: Frame3D; fScrew: Frame3D; fCart: Frame3D;
  ghosts: { screw: Frame3D[]; cart: Frame3D[] };
  trailS: Link3D[];
  trailC: Link3D[];
  axis: ScrewAxis3D;
  gap: Link3D;
  dotS: { position: { set(x: number, y: number, z: number): void } };
  dotC: { position: { set(x: number, y: number, z: number): void } };
}

function mount(host: HTMLElement, ctx: LessonContext): LessonHandle {
  const st = {
    view: '2d' as '2d' | '3d',
    start: { ...PRESETS_2D.basic.start } as Pose2,
    end: { ...PRESETS_2D.basic.end } as Pose2,
    elbow: 'up' as Elbow,
    policy: 'warm' as IKPolicy,
    sel: 'joint' as PathKind,
    show: { joint: true, screw: true, cartesian: true } as Record<PathKind, boolean>,
    wrist: false,
    obstacle: false,
    obC: [1.35, 0.95] as Pt,
    obR: 0.16,
    frames: true,
    t: 1.0,
    // 3D
    alpha: 120 * deg, dx: 0.8, dy: 0.8, dz: 0.45,
    show3: { screw: true, cart: true, frames: true, axis: true },
  };
  let res: PlanarPaths = planarPaths(st.start, st.end, st.elbow, st.policy, null);
  let cmp: SE3Comparison | null = null;
  let X3: { Xs: Mat; Xe: Mat } = { Xs: [], Xe: [] };
  const sNow = () => QuinticTimeScaling(TDUR, st.t);
  const N = () => res.paths.joint.s.length;
  const idx = () => Math.round(sNow() * (N() - 1));

  // ------------------------------------------------------------------ layout
  simTitle(host, '同樣的起點與終點，三種直線', 'joint / screw / Cartesian');
  const topPanel = controlPanel(host);
  const topRow = row(topPanel);
  const selView = select<'2d' | '3d'>(topRow, {
    options: [{ value: '2d', label: '視圖：平面 3R 手臂（2D）' }, { value: '3d', label: '視圖：SE(3) 螺旋 vs 解耦（3D）' }],
    value: '2d',
    onChange: (v) => setView(v),
  });
  const legendEl = el('div', { class: 'mr-legend' });
  topRow.appendChild(legendEl);

  const view2 = el('div');
  const view3 = el('div', { style: { display: 'none' } });
  host.append(view2, view3);
  const cv = new Canvas2D(view2, { aspect: 1.85, world: WORLD, overlay: true });

  const pbPanel = controlPanel(host);
  const pbRow = row(pbPanel);
  const pb = playback(pbRow, {
    duration: TDUR,
    onPlay: () => { if (st.t >= TDUR - 1e-6) st.t = 0; loop.start(); },
    onPause: () => loop.stop(),
    onReset: () => { loop.stop(); pb.setPlaying(false, { silent: true }); st.t = 0; frame(); },
    onSpeed: (s) => { loop.speed = s; },
    onScrub: (t) => { st.t = t; frame(); },
  });
  pbRow.appendChild(el('span', { class: 'mr-note', style: { margin: '0' } }, `五次時間縮放，T = ${TDUR} s`));

  const ctl2 = el('div');
  const ctl3 = el('div', { style: { display: 'none' } });
  host.append(ctl2, ctl3);

  // ---- 2D controls
  const panel2 = controlPanel(ctl2, { compact: true });
  const chips2 = presetChips(row(panel2), [
    { label: '三種直線', apply: () => applyPreset(PRESETS_2D.basic, { sel: 'joint', wrist: false }) },
    { label: '靠近基座：θ̇ 尖峰', apply: () => applyPreset(PRESETS_2D.nearBase, { sel: 'cartesian', wrist: true }) },
    { label: '穿過不可達區', apply: () => applyPreset(PRESETS_2D.unreachable, { sel: 'cartesian', wrist: true }) },
    { label: '冷啟動 IK', apply: () => applyPreset(PRESETS_2D.cold, { sel: 'cartesian', wrist: false }) },
    { label: '障礙物 A', apply: () => applyPreset(PRESETS_2D.obstacleA, { sel: 'joint', wrist: false }) },
    { label: '障礙物 B', apply: () => applyPreset(PRESETS_2D.obstacleB, { sel: 'cartesian', wrist: false }) },
  ], { label: '情境' });

  const rowSel = row(panel2);
  const selPath = select<PathKind>(rowSel, {
    options: PATH_KINDS.map((k) => ({ value: k, label: `播放：${NAME[k]}` })),
    value: st.sel,
    onChange: (k) => { st.sel = k; st.show[k] = true; showT[k].set(true, { silent: true }); refresh2D(); },
  });
  const selElbow = select<Elbow>(rowSel, {
    options: [{ value: 'up', label: '手肘：θ₂ < 0（up）' }, { value: 'down', label: '手肘：θ₂ > 0（down）' }],
    value: st.elbow,
    onChange: (v) => { st.elbow = v; chips2.clear(); recompute(); },
  });
  const selPolicy = select<IKPolicy>(rowSel, {
    options: [{ value: 'warm', label: 'IK：延續上一點（warm）' }, { value: 'cold', label: 'IK：每點從起點（cold）' }],
    value: st.policy,
    onChange: (v) => { st.policy = v; chips2.clear(); recompute(); },
  });
  const showRow = row(panel2);
  const showWrap = el('div', { class: 'mr-row', style: { gap: '0.2rem 0.8rem' } }, el('span', { class: 'mr-note', style: { margin: '0' } }, '顯示：'));
  showRow.appendChild(showWrap);
  const showT = {} as Record<PathKind, ReturnType<typeof toggle>>;
  for (const k of PATH_KINDS) showT[k] = toggle(showWrap, { label: SHORT[k], value: st.show[k], onChange: (v) => { st.show[k] = v; refresh2D(); } });

  const tWrist = toggle(showWrap, { label: '腕點與奇異邊界', value: st.wrist, onChange: (v) => { st.wrist = v; cv.requestRender(); } });
  const tFrames = toggle(showWrap, { label: '工具座標系', value: st.frames, onChange: (v) => { st.frames = v; cv.requestRender(); } });
  const rowOpt = row(panel2);
  const tObs = toggle(rowOpt, { label: '障礙物', value: st.obstacle, onChange: (v) => { st.obstacle = v; obHandle.enabled = v; chips2.clear(); recompute(); } });
  const sObR = slider(rowOpt, { label: '障礙物半徑', min: 0.05, max: 0.4, step: 0.01, value: st.obR, unit: 'm', onInput: (v) => { st.obR = v; chips2.clear(); recompute(); } });
  sObR.el.style.minWidth = '150px';
  sObR.el.style.flex = '1';

  const status2 = el('div', { class: 'mr-note', style: { fontSize: '0.84rem', margin: '0.35rem 0' } });
  ctl2.appendChild(status2);
  // Bottom area as two tabs (plots / comparison table) to keep the pane short.
  const tabRow2 = el('div');
  ctl2.appendChild(tabRow2);
  const paneA = el('div'), paneB = el('div');
  ctl2.append(paneA, paneB);
  const tabs2 = presetChips(tabRow2, [
    { label: '圖表：θ(s) 與 ‖dθ/ds‖', apply: () => { paneA.style.display = ''; paneB.style.display = 'none'; } },
    { label: '比較表', apply: () => { paneA.style.display = 'none'; paneB.style.display = ''; } },
  ]);
  const [pl, pr] = split(paneA);
  const pTheta = new Plot2D(pl, { title: 'θ(s)：播放中的路徑', xLabel: 's', yLabel: 'rad', series: thetaSpecs(), xRange: [0, 1], height: 110 });
  const pSpeed = new Plot2D(pr, { title: '‖dθ/ds‖：三條路徑', xLabel: 's', series: speedSpecs(), xRange: [0, 1], height: 110 });
  const tableWrap = el('div', { class: 'md', style: { overflowX: 'auto' } });
  paneB.appendChild(tableWrap);
  const tbody = el('tbody');
  tableWrap.appendChild(el('table', { style: { fontSize: '0.8rem', margin: '0.4rem 0 0.2rem' } },
    el('thead', {}, el('tr', {}, ['路徑', '末端路徑長', '最大 ‖dθ/ds‖', '最小 |sin θ₂|', 'IK 無解', '關節跳變', '碰撞'].map((h) => el('th', {}, h)))),
    tbody));

  function thetaSpecs(): SeriesSpec[] {
    return [{ name: 'θ₁', color: cssVar('--series-6') }, { name: 'θ₂', color: cssVar('--series-7') }, { name: 'θ₃', color: cssVar('--series-3') }];
  }
  function speedSpecs(): SeriesSpec[] {
    return PATH_KINDS.map((k) => ({ name: SHORT[k], color: colorOf(k), width: k === st.sel ? 2.6 : 1.5 }));
  }

  // ---- 3D controls
  const panel3 = controlPanel(ctl3);
  const chips3 = presetChips(row(panel3), PRESETS_3D.map((p) => ({ label: p.label, apply: () => set3(p.p) })), { label: '情境' });
  const sAlpha = angleSlider(panel3, { label: '終點繞 z 轉 α', value: st.alpha, min: -170 * deg, max: 170 * deg, onInput: (v) => { st.alpha = v; chips3.clear(); update3D(); } });
  const sDx = slider(panel3, { label: '位移 Δx', min: -1, max: 1, step: 0.05, value: st.dx, unit: 'm', onInput: (v) => { st.dx = v; chips3.clear(); update3D(); } });
  const sDy = slider(panel3, { label: '位移 Δy', min: -1, max: 1, step: 0.05, value: st.dy, unit: 'm', onInput: (v) => { st.dy = v; chips3.clear(); update3D(); } });
  const sDz = slider(panel3, { label: '升降 Δz（沿軸 → 螺距）', min: -0.6, max: 0.6, step: 0.05, value: st.dz, unit: 'm', onInput: (v) => { st.dz = v; chips3.clear(); update3D(); } });
  const row3 = row(panel3);
  toggle(row3, { label: '螺旋路徑', value: true, onChange: (v) => { st.show3.screw = v; update3D(); } });
  toggle(row3, { label: '笛卡兒路徑', value: true, onChange: (v) => { st.show3.cart = v; update3D(); } });
  toggle(row3, { label: '沿途座標系', value: true, onChange: (v) => { st.show3.frames = v; update3D(); } });
  toggle(row3, { label: '螺旋軸', value: true, onChange: (v) => { st.show3.axis = v; update3D(); } });
  const kv3 = kv(ctl3, ['轉角 |ω|', '螺距 h', 'max ‖R_screw(s) − R_cart(s)‖', 'max ‖p_screw(s) − p_cart(s)‖', '此刻 s、‖p_screw − p_cart‖']);
  const Vview = matrixView(ctl3, { label: '[ω; v] = log(X_start⁻¹ X_end) =', digits: 3 });
  note(ctl3, '紫：螺旋路徑（虛線為螺旋軸）；橘：解耦的笛卡兒路徑；灰線連接同一個 s 時兩者的原點。起點姿態固定，終點 = 起點先繞 z 轉 α、再平移 (Δx, Δy, Δz)。');

  // ------------------------------------------------------------------ 2D drawing
  const clampPt = (x: number, y: number): Pt => [clamp(x, WORLD.xmin + 0.05, WORLD.xmax - 0.05), clamp(y, WORLD.ymin + 0.05, WORLD.ymax - 0.05)];
  const handleColor = cssVar('--fg-muted', '#667');
  const hStart = poseHandle(cv, { x: st.start.x, y: st.start.y, theta: st.start.phi }, {
    label: '起點', color: handleColor, knob: 0.38, frameSize: 0.22,
    onChange: (p) => { const [x, y] = clampPt(p.x, p.y); st.start = { x, y, phi: p.theta }; chips2.clear(); recompute(); },
  });
  const hEnd = poseHandle(cv, { x: st.end.x, y: st.end.y, theta: st.end.phi }, {
    label: '終點', color: handleColor, knob: 0.38, frameSize: 0.22,
    onChange: (p) => { const [x, y] = clampPt(p.x, p.y); st.end = { x, y, phi: p.theta }; chips2.clear(); recompute(); },
  });
  const obHandle = cv.addHandle({
    get: () => st.obC,
    set: (p) => { st.obC = clampPt(p[0], p[1]); chips2.clear(); recompute(); },
    color: cssVar('--danger', '#b91c1c'), radius: 7, enabled: st.obstacle,
  });

  cv.draw = (c) => {
    const danger = cssVar('--danger');
    c.grid(0.5);
    c.frame({ x: 0, y: 0, theta: 0 }, 0.28, { labels: ['x', 'y'], width: 1.5 });
    if (st.wrist) {
      c.circle([0, 0], LINKS[0] + LINKS[1], { color: danger, width: 1.2, dash: [6, 4], alpha: 0.6 });
      c.circle([0, 0], Math.abs(LINKS[0] - LINKS[1]), { color: danger, width: 1.2, dash: [3, 3], alpha: 0.8 });
      c.text([-1.5, 1.42], '腕點外邊界 |w| = 1.8：伸直（θ₂ = 0）', { color: danger, align: 'left', font: '11px sans-serif' });
      c.text([0.2, -0.22], '內邊界 |w| = 0.2：摺疊（θ₂ = ±π）', { color: danger, align: 'left', font: '11px sans-serif' });
    }
    if (st.obstacle) {
      c.circle(st.obC, st.obR, { fill: danger, alpha: 0.16, width: 0 });
      c.circle(st.obC, st.obR, { color: danger, width: 1.5 });
    }
    if (res.ok) {
      if (st.show.screw && res.pole) {
        const pole = res.pole as Pt;
        c.line(pole, [st.start.x, st.start.y], { color: colorOf('screw'), width: 1, dash: [2, 4], alpha: 0.7 });
        c.line(pole, [st.end.x, st.end.y], { color: colorOf('screw'), width: 1, dash: [2, 4], alpha: 0.7 });
        c.circle(pole, 0.045, { color: colorOf('screw'), width: 2 });
        c.point(pole, 2.5, { color: colorOf('screw') });
        c.text(pole, '螺旋軸 ⊙', { color: colorOf('screw'), dx: -30, dy: 12, font: '11px sans-serif', bg: cssVar('--bg-elev') });
      }
      for (const k of PATH_KINDS) if (st.show[k]) drawTrail(c, k);
      const P = res.paths[st.sel];
      if (st.wrist) c.polyline(P.target.map((t) => wristPoint(t) as Pt), { color: colorOf(st.sel), width: 1.2, dash: [5, 3], alpha: 0.9 });
      const k = idx();
      for (const kind of PATH_KINDS) {
        if (!st.show[kind] || kind === st.sel) continue;
        const th = res.paths[kind].theta[k];
        if (th) drawPlanarArm(c, LINKS, th, { alpha: 0.28, linkColor: colorOf(kind), linkWidth: 5, eeRadius: 4 });
      }
      if (st.frames) {
        for (const sk of [0.25, 0.5, 0.75]) {
          const tg = P.target[Math.round(sk * (N() - 1))];
          c.frame({ x: tg.x, y: tg.y, theta: tg.phi }, 0.15, { width: 1.4, alpha: 0.7 });
        }
      }
      const th = P.theta[k];
      const tg = P.target[k];
      if (th) {
        drawPlanarArm(c, LINKS, th, { eeColor: colorOf(st.sel) });
        c.frame({ x: tg.x, y: tg.y, theta: tg.phi }, 0.24, { width: 2 });
        if (P.collide[k]) c.text([tg.x, tg.y], '碰撞！', { color: danger, dy: -20, bg: cssVar('--danger-soft') });
      } else {
        c.frame({ x: tg.x, y: tg.y, theta: tg.phi }, 0.24, { width: 2, alpha: 0.45 });
        c.text([tg.x, tg.y], 'IK 無解：此位姿不可達', { color: danger, dy: -20, bg: cssVar('--danger-soft') });
      }
    }
    hStart.draw(); hEnd.draw();
    const P = res.paths[st.sel];
    const th = res.ok ? P.theta[idx()] : null;
    c.overlay.textContent = res.ok
      ? `s = ${fmt(sNow(), 2)} · ${NAME[st.sel]} · θ = ${th ? `(${th.map((x) => fmt(x / deg, 0) + '°').join(', ')})` : '—'}`
      : `${res.unreachable === 'start' ? '起點' : '終點'}不可達`;
  };

  function drawTrail(c: Canvas2D, k: PathKind): void {
    const P = res.paths[k];
    const pts = P.target.map((t) => [t.x, t.y] as Pt);
    const color = colorOf(k);
    const danger = cssVar('--danger');
    const width = k === st.sel ? 3 : 2;
    if (P.failCount > 0) c.polyline(pts, { color: danger, width: 1.4, dash: [3, 4], alpha: 0.85 });
    // Samples where the arm hits the obstacle: a translucent red band under the trail.
    let hit: Pt[] = [];
    const flushHit = () => { if (hit.length > 1) c.polyline(hit, { color: danger, width: 9, alpha: 0.3 }); else if (hit.length) c.point(hit[0], 4.5, { color: danger, alpha: 0.4 }); hit = []; };
    for (let i = 0; i < pts.length; i++) { if (P.collide[i]) hit.push(pts[i]); else flushHit(); }
    flushHit();
    let run: Pt[] = [];
    const flush = () => { if (run.length > 1) c.polyline(run, { color, width, alpha: k === st.sel ? 1 : 0.75 }); run = []; };
    for (let i = 0; i < pts.length; i++) {
      if (P.fail[i]) { flush(); continue; }
      run.push(pts[i]);
    }
    flush();
    for (const j of P.jumps) {
      c.circle(pts[j], 0.08, { color: danger, width: 2.2 });
      c.text(pts[j], '關節跳變', { color: danger, dy: -17, bg: cssVar('--danger-soft') });
    }
  }

  // ------------------------------------------------------------------ 2D logic
  function applyPreset(p: Preset2D, o: { sel: PathKind; wrist: boolean }): void {
    st.start = { ...p.start }; st.end = { ...p.end };
    hStart.set({ x: p.start.x, y: p.start.y, theta: p.start.phi }, { silent: true });
    hEnd.set({ x: p.end.x, y: p.end.y, theta: p.end.phi }, { silent: true });
    st.elbow = p.elbow; selElbow.set(p.elbow, { silent: true });
    st.policy = p.policy; selPolicy.set(p.policy, { silent: true });
    st.obstacle = !!p.obstacle; tObs.set(st.obstacle, { silent: true }); obHandle.enabled = st.obstacle;
    if (p.obstacle) { st.obC = [p.obstacle.c[0], p.obstacle.c[1]]; st.obR = p.obstacle.r; sObR.set(p.obstacle.r, { silent: true }); }
    st.sel = o.sel; selPath.set(o.sel, { silent: true });
    st.wrist = o.wrist; tWrist.set(o.wrist, { silent: true });
    for (const k of PATH_KINDS) { st.show[k] = true; showT[k].set(true, { silent: true }); }
    loop.stop(); pb.setPlaying(false, { silent: true });
    st.t = 1.0;
    recompute();
  }

  function recompute(): void {
    res = planarPaths(st.start, st.end, st.elbow, st.policy, st.obstacle ? { c: st.obC, r: st.obR } : null);
    refresh2D();
  }

  function refresh2D(): void {
    const P = res.paths[st.sel];
    const s = P.s;
    for (let i = 0; i < 3; i++) pTheta.setXY(i, s, P.theta.map((t) => (t ? t[i] : NaN)));
    PATH_KINDS.forEach((k, i) => {
      const Q = res.paths[k];
      if (st.show[k]) pSpeed.setXY(i, Q.s, Q.speed); else pSpeed.setXY(i, [], []);
    });
    pTheta.setOptions({ series: thetaSpecs() });
    pSpeed.setOptions({ series: speedSpecs() });
    renderLegend();
    renderStatus();
    renderTable();
    frame();
  }

  function markers2D(): void {
    const danger = cssVar('--danger'), muted = cssVar('--fg-muted');
    const cur: Marker = { x: sNow(), color: muted, dash: [2, 3] };
    const P = res.paths[st.sel];
    const m: Marker[] = [cur];
    P.jumps.forEach((j, n) => m.push({ x: P.s[j], color: danger, dash: [5, 3], label: n === 0 ? '跳變' : undefined }));
    let inFail = false;
    P.fail.forEach((f, i) => {
      if (f && !inFail) m.push({ x: P.s[i], color: danger, dash: [1, 3], label: 'IK 無解' });
      inFail = f;
    });
    pTheta.setMarkers(m);
    pSpeed.setMarkers([cur]);
  }

  function renderLegend(): void {
    if (st.view === '3d') {
      legendEl.replaceChildren(
        el('span', {}, el('span', { class: 'sw', style: { background: '#8b5cf6' } }), '螺旋路徑'),
        el('span', {}, el('span', { class: 'sw', style: { background: '#f59e0b' } }), '笛卡兒路徑'));
      return;
    }
    legendEl.replaceChildren(...PATH_KINDS.filter((k) => st.show[k]).map((k) =>
      el('span', { style: { fontWeight: k === st.sel ? '700' : '400' } }, el('span', { class: 'sw', style: { background: colorOf(k) } }), NAME[k])));
  }

  function renderStatus(): void {
    const danger = cssVar('--danger'), ok = cssVar('--ok'), fg = cssVar('--fg');
    if (!res.ok) {
      status2.textContent = `⚠ ${res.unreachable === 'start' ? '起點' : '終點'}不可達：腕點必須落在 0.2 m ≤ |w| ≤ 1.8 m 的環內。把它拖回手臂附近。`;
      status2.style.color = danger;
      return;
    }
    const P = res.paths[st.sel];
    const parts: string[] = [];
    if (P.failCount) parts.push(`${P.failCount} 個樣本 IK 無解（路徑有一段不可達）`);
    if (P.jumps.length) parts.push(`${P.jumps.length} 次關節跳變${st.policy === 'cold' && st.sel !== 'joint' ? '（每點都從起點組態求解，換到了另一組手肘解）' : ''}`);
    const nearSing = Number.isFinite(P.minSin2) && P.minSin2 < 0.25;
    if (nearSing && st.sel !== 'joint') parts.push(`接近奇異（最小 |sin θ₂| = ${fmt(P.minSin2, 2)}，‖dθ/ds‖ 最大 ${fmt(P.maxSpeed, 1)}）`);
    if (P.firstCollision >= 0) parts.push(`在 s = ${fmt(P.s[P.firstCollision], 2)} 撞到障礙物`);
    if (st.sel !== 'joint' && P.endElbow && P.endElbow !== st.elbow) parts.push('終點的手肘方向和起點不同');
    let text = `${NAME[st.sel]}：` + (parts.length ? parts.join('；') + '。'
      : st.sel === 'joint'
        ? `不需逐點解 IK，dθ/ds 恆定${nearSing ? '；途中雖經過奇異附近，關節速度仍平順' : ''}。`
        : '一路可達、連續，遠離奇異。');
    if (Math.abs(res.dphi) < 1e-9) text += ' （起終點姿態相同，Δφ = 0：螺旋路徑退化成直線，與笛卡兒路徑重合。）';
    status2.textContent = text;
    status2.style.color = parts.length ? danger : (st.sel === 'joint' ? fg : ok);
  }

  function renderTable(): void {
    const danger = cssVar('--danger');
    const cell = (txt: string, bad = false) => el('td', { style: { fontFamily: 'var(--mono)', color: bad ? danger : '', whiteSpace: 'nowrap' } }, txt);
    if (!res.ok) { tbody.replaceChildren(); return; }
    tbody.replaceChildren(...PATH_KINDS.map((k) => {
      const P = res.paths[k];
      return el('tr', { style: { background: k === st.sel ? 'var(--accent-soft)' : '' } },
        el('td', { style: { whiteSpace: 'nowrap' } }, el('span', { style: { display: 'inline-block', width: '10px', height: '10px', borderRadius: '2px', background: colorOf(k), marginRight: '0.35rem' } }), SHORT[k]),
        cell(`${fmt(P.tipLength, 2)} m`),
        cell(fmt(P.maxSpeed, 1), P.maxSpeed > 3 * res.paths.joint.maxSpeed + 1e-9),
        cell(fmt(P.minSin2, 2), k !== 'joint' && P.minSin2 < 0.25),
        cell(P.failCount ? `${P.failCount} 點` : '—', P.failCount > 0),
        cell(P.jumps.length ? `${P.jumps.length} 次` : '—', P.jumps.length > 0),
        cell(st.obstacle ? (P.firstCollision >= 0 ? `s = ${fmt(P.s[P.firstCollision], 2)}` : '無') : '—', P.firstCollision >= 0));
    }));
  }

  // ------------------------------------------------------------------ 3D
  let s3: Scene3Parts | null = null;
  function ensureScene(): Scene3Parts {
    if (s3) return s3;
    const scene = new Scene3D(view3, { aspect: 16 / 10, grid: 2, axes: 0.25, cameraPos: [1.2, -1.5, 1.0], target: [-0.1, 0.1, 0.33] });
    const tube = (color: number) => Array.from({ length: SAMPLES3 - 1 }, () => scene.addLink({ radius: 0.008, color }));
    const ghost = () => { const f = scene.addFrame({ size: 0.11, opacity: 0.45 }); return f; };
    s3 = {
      scene,
      fStart: scene.addFrame({ size: 0.2, label: 'X_start' }),
      fEnd: scene.addFrame({ size: 0.2, label: 'X_end' }),
      fScrew: scene.addFrame({ size: 0.16 }),
      fCart: scene.addFrame({ size: 0.16 }),
      ghosts: { screw: [ghost(), ghost(), ghost()], cart: [ghost(), ghost(), ghost()] },
      trailS: tube(C3.screw),
      trailC: tube(C3.cart),
      axis: scene.addScrewAxis({ color: C3.screw, length: 0.9 }),
      gap: scene.addLink({ radius: 0.004, color: C3.link }),
      dotS: scene.addSphere(0.028, C3.screw),
      dotC: scene.addSphere(0.028, C3.cart),
    };
    return s3;
  }

  function set3(p: { alpha: number; dx: number; dy: number; dz: number }): void {
    st.alpha = p.alpha; st.dx = p.dx; st.dy = p.dy; st.dz = p.dz;
    sAlpha.set(p.alpha, { silent: true }); sDx.set(p.dx, { silent: true }); sDy.set(p.dy, { silent: true }); sDz.set(p.dz, { silent: true });
    update3D();
  }

  function update3D(): void {
    const R0 = RotZ(-30 * deg);
    const p0 = [-0.5, -0.3, 0.15];
    const Xs = RpToTrans(R0, p0);
    const Xe = RpToTrans(matMul(RotZ(st.alpha), R0), [p0[0] + st.dx, p0[1] + st.dy, p0[2] + st.dz]);
    X3 = { Xs, Xe };
    cmp = compareSE3(Xs, Xe, SAMPLES3);
    if (s3) {
      const S = s3;
      const posOf = (T: Mat) => [T[0][3], T[1][3], T[2][3]];
      S.fStart.setTransform(Xs); S.fEnd.setTransform(Xe);
      const setTrail = (segs: Link3D[], Ts: Mat[], vis: boolean) => segs.forEach((l, k) => {
        l.setEndpoints(posOf(Ts[k]), posOf(Ts[k + 1]));
        if (!vis) l.mesh.visible = false;
      });
      setTrail(S.trailS, cmp.screw, st.show3.screw);
      setTrail(S.trailC, cmp.cartesian, st.show3.cart);
      [10, 20, 30].forEach((k, i) => {
        S.ghosts.screw[i].setTransform(cmp!.screw[k]); S.ghosts.screw[i].setVisible(st.show3.frames && st.show3.screw);
        S.ghosts.cart[i].setTransform(cmp!.cartesian[k]); S.ghosts.cart[i].setVisible(st.show3.frames && st.show3.cart);
      });
      if (cmp.axisDir) { S.axis.setScrew(cmp.Vs); S.axis.setVisible(st.show3.axis); } else S.axis.setVisible(false);
    }
    kv3.set('轉角 |ω|', fmtDeg(cmp.angle, 1));
    kv3.set('螺距 h', Number.isFinite(cmp.pitch) ? `${fmt(cmp.pitch, 3)} m/rad` : '∞（純平移，沒有轉軸）');
    kv3.set('max ‖R_screw(s) − R_cart(s)‖', cmp.maxRotDiff < 1e-12 ? '0（到機器精度）' : cmp.maxRotDiff.toExponential(1));
    kv3.set('max ‖p_screw(s) − p_cart(s)‖', `${fmt(cmp.maxPosDiff, 3)} m`);
    Vview.set([cmp.Vb], { highlightCols: ['hl', 'hl', 'hl', null, null, null] });
    frame3D();
  }

  function frame3D(): void {
    if (!cmp) return;
    const s = sNow();
    const scal = (Tf: number, t: number) => (s * t) / Tf; // reaches s at t = Tf: one sample of each @/mr path at parameter s
    const Ts = ScrewTrajectory(X3.Xs, X3.Xe, 1, 2, scal)[1];
    const Tc = CartesianTrajectory(X3.Xs, X3.Xe, 1, 2, scal)[1];
    const ps = [Ts[0][3], Ts[1][3], Ts[2][3]], pc = [Tc[0][3], Tc[1][3], Tc[2][3]];
    const d = Math.hypot(ps[0] - pc[0], ps[1] - pc[1], ps[2] - pc[2]);
    kv3.set('此刻 s、‖p_screw − p_cart‖', `s = ${fmt(s, 2)}，${fmt(d, 3)} m`);
    if (!s3) return;
    s3.fScrew.setTransform(Ts); s3.fScrew.setVisible(st.show3.screw);
    s3.fCart.setTransform(Tc); s3.fCart.setVisible(st.show3.cart);
    s3.dotS.position.set(ps[0], ps[1], ps[2]);
    s3.dotC.position.set(pc[0], pc[1], pc[2]);
    s3.gap.setEndpoints(ps, pc);
  }

  // ------------------------------------------------------------------ views, playback, steps
  function setView(v: '2d' | '3d'): void {
    st.view = v;
    selView.set(v, { silent: true });
    const is3 = v === '3d';
    view2.style.display = is3 ? 'none' : '';
    ctl2.style.display = is3 ? 'none' : '';
    view3.style.display = is3 ? '' : 'none';
    ctl3.style.display = is3 ? '' : 'none';
    if (is3) { ensureScene().scene.start(); update3D(); } else { s3?.scene.stop(); cv.requestRender(); }
    renderLegend();
    frame();
  }

  function frame(): void {
    pb.setTime(st.t);
    if (st.view === '3d') { frame3D(); return; }
    markers2D();
    cv.requestRender();
  }

  const loop = createLoop((dt) => {
    st.t += dt;
    if (st.t >= TDUR) { st.t = TDUR; loop.stop(); pb.setPlaying(false, { silent: true }); }
    frame();
  });

  const stepSetup: (() => void)[] = [
    () => { setView('2d'); chips2.activate(0); },
    () => {
      setView('2d');
      for (const k of PATH_KINDS) { st.show[k] = k === 'joint'; showT[k].set(st.show[k], { silent: true }); }
      st.sel = 'joint'; selPath.set('joint', { silent: true }); st.t = 1.0; refresh2D();
    },
    () => { setView('3d'); chips3.activate(2); st.t = 1.0; frame(); },
    () => {
      setView('2d');
      for (const k of PATH_KINDS) { st.show[k] = k === 'cartesian'; showT[k].set(st.show[k], { silent: true }); }
      st.sel = 'cartesian'; selPath.set('cartesian', { silent: true });
      st.frames = true; tFrames.set(true, { silent: true }); st.t = 1.0; refresh2D();
    },
    () => { setView('2d'); chips2.activate(1); },
    () => { setView('2d'); chips2.activate(5); },
  ];
  const focusOf = (i: number): HTMLElement[] => [
    [cv.el],
    [paneA],
    [view3],
    [cv.el],
    [pSpeed.el, chips2.el],
    [paneB, tObs.el],
  ][i] ?? [];
  let focused: HTMLElement[] = [];
  const onStep = (i: number) => {
    stepSetup[i]?.();
    tabs2.activate(i === 5 ? 1 : 0);
    focused.forEach((f) => f.classList.remove('mr-focus'));
    focused = focusOf(i);
    focused.forEach((f) => f.classList.add('mr-focus'));
  };

  update3D();
  recompute();
  onStep(ctx.currentStep);
  const unsubTheme = ctx.onThemeChange(() => { refresh2D(); });

  return {
    onStep,
    destroy() {
      loop.stop();
      unsubTheme();
      cv.destroy();
      pTheta.destroy();
      pSpeed.destroy();
      s3?.scene.destroy();
      s3 = null;
    },
  };
}

const lesson: Lesson = { meta, steps, quiz, mount };
export default lesson;
