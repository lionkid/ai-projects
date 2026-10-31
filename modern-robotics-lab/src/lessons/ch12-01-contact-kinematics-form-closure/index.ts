import type { Lesson, LessonContext, LessonHandle, LessonStep, QuizQuestion } from '@/types';
import {
  el, cssVar, fmt, controlPanel, row, button, select, toggle, simTitle, note, legend, kv, mdElement,
  Canvas2D, presetChips, createLoop, type Handle, type World,
} from '@/ui';
import { meta } from './meta';
import {
  type P2, type V3, type PolyShape, type ContactMode, type ContactState, type ForceLine, type BoundaryPoint, type ResistResult,
  makeShape, boundaryAt, projectToBoundary, perimeter, contactWrench, forceWrench, twistAbout, translationTwist, corOf,
  pointVelocity, classifyContact, planarExp, applyPose, displacementScale, preferredEscape, closureCertificate,
  wrenchRank, corRegion, polygonAreaAbs, feasibleCorPoints, translationCone, resistWrench, autoFormClosure, insidePolygon,
  lineIntersection, add2, sub2, scale2, norm2, dot2, perp, unit2,
} from './contact';

// ------------------------------------------------------------------ content
const steps: LessonStep[] = [
  {
    title: '一個接觸能做什麼',
    md: `想像一根細手指頂在物體邊上：接觸點是 $p$，$\\hat n$ 是該點**指向物體內部**的單位法向量。沒有摩擦時，手指只能沿 $\\hat n$ 推，推力 $f_n \\ge 0$；它不能拉，也給不出切向力。

把注意力換到運動上。手指不動時，物體在接觸點附近只有三種合法的走法：

- **脫離（breaking）**：接觸點往外離開手指；
- **滑動（sliding）**：貼著手指沿切線方向走；
- **滾動（rolling）**：接觸點本身不動，物體繞它轉。

唯一被禁止的是**穿透**——接觸點往手指裡面鑽。`,
    simHint: '拖曳物體本身（或藍色箭頭尖端）：往上是脫離、沿著底邊水平是滑動、往下會穿透而變紅。再切到「↺ 逆時針轉」，把 CoR 拖到接觸點上就是滾動。',
  },
  {
    title: '一階分析：內積的符號',
    md: `把接觸的單位法向力寫成 wrench $\\vF = (p\\times\\hat n,\\ \\hat n)$，平面上就是 $(m_z, f_x, f_y)$；物體的 twist 寫成 $\\vV = (\\omega, v_x, v_y)$。兩者的內積恰好是接觸點沿法向的速度：

$$\\vF^{\\T}\\vV = \\hat n^{\\T}\\dot p .$$

$\\vF^{\\T}\\vV > 0$ 表示脫離；$= 0$ 表示維持接觸（接觸點速度為零是滾動，否則是滑動）；$< 0$ 是穿透，不可能。

手指自己也在動時，改用相對速度：條件變成 $\\vF^{\\T}\\vV \\ge \\hat n^{\\T} v_f$。注意這只是**一階**判斷：剛好等於 0 時，真的能不能滑過去要看曲率等二階效應。`,
    simHint: '「推一下」：手指以 v_f 往上推，物體也往上平移。拖曳橘色 v_f 箭頭的尖端：比物體慢 → 脫離；一樣 → 滾動；多一點側向分量 → 滑動；比物體快 → 穿透。',
  },
  {
    title: '旋轉中心圖法',
    md: `平面上 $\\omega \\ne 0$ 的 twist 都是繞某一點的純旋轉，這點叫**旋轉中心（center of rotation, CoR）**；純平移相當於 CoR 在無窮遠。讓物體以角速度 $\\omega$ 繞 $c$ 轉，代入上式得

$$\\vF^{\\T}\\vV = \\omega\\,\\big((p-c)\\times\\hat n\\big)_z .$$

所以沿 $\\hat n$ 畫出接觸的法線：法線**左側**的點標 $+$（繞它只能逆時針轉）、**右側**標 $-$（只能順時針轉）、**線上**標 $\\pm$（兩個方向都只會滑動；剛好在接觸點上就是滾動）。一個接觸把整個平面切成兩半。`,
    simHint: '拖曳 CoR 十字到法線兩側，再切換 ↺ / ↻，看判定和區域顏色是否一致；拖近虛線或接觸點時會自動吸附。',
  },
  {
    title: '多個接觸：交集',
    md: `每個接觸各自給平面一套 $+/-$ 標記。一個旋轉要被**所有**接觸接受，它的 CoR 必須同時落在每條法線的同一側：

- 可逆時針轉的 CoR = 各接觸 $+$ 半平面的交集（藍）；
- 可順時針轉的 CoR = 各接觸 $-$ 半平面的交集（紫）。

半平面的交集一定是凸區域，每多一個接觸就多切一刀，可行區只會縮小。平移則看右上角的小羅盤：方向 $v$ 必須讓每個 $\\hat n_i^{\\T} v \\ge 0$。`,
    simHint: 'L 形物體放在地上、靠著左牆：繞藍區的點逆時針轉，L 會同時離開牆和地面；繞紫區的點順時針轉，就是往右翻倒。按「＋接觸」或拖動手指，看區域被切小。',
  },
  {
    title: '形封閉',
    md: `如果除了 $\\vV = 0$ 以外沒有任何運動被允許，物體只靠幾何就被卡死，稱為**形封閉（form closure）**：

$$\\{\\vV : \\vF_i^{\\T}\\vV \\ge 0,\\ i = 1,\\dots,j\\} = \\{0\\} .$$

等價地說，$j$ 個接觸 wrench 的非負組合能湊出任何 wrench。平面 twist 是 3 維，要用非負係數「張滿」$\\R^3$ 至少需要 4 個向量，所以平面至少 4 個接觸；空間是 6 維，至少 7 個。數量只是必要條件：四個接觸都放在方塊邊的中點，法線交於中心，繞中心的轉動在一階上仍然沒被排除。`,
    simHint: '按「自動找形封閉」：兩塊可行區都消失、羅盤也全空，徽章變綠。再按「法線共點」反例：同樣 4 個接觸，中心標著 ±。',
  },
  {
    title: '只推不拉，以及摩擦',
    md: `形封閉也能從力的角度看：對任何外力 wrench $\\vF_{\\text{ext}}$，都找得到

$$k_i \\ge 0,\\qquad \\sum_i k_i\\vF_i = -\\vF_{\\text{ext}} .$$

$k_i \\ge 0$ 是關鍵：無摩擦點接觸只能推、不能拉。沒有形封閉時，總有某些外力需要某個 $k_i < 0$，也就是要手指「拉住」物體，物體就會沿那個方向被帶走。

加上摩擦後，每個接觸能出的力從一條線變成一個錐，所需的接觸數跟著變少——兩根手指就可能抓牢。這就是下一課的力封閉。`,
    simHint: '拖曳橘色外力（尖端改大小方向、圓點改作用點）：形封閉時綠色接觸力總能平衡它。再按「外力：L 形三點」，把外力往上或往右拉，就會出現紅色的「需要拉」。',
  },
];

const quiz: QuizQuestion[] = [
  {
    question: '方塊放在地上並靠著左牆：底邊兩個接觸（法線朝上），左邊一個接觸（法線朝右）。讓方塊繞**右下角**轉，哪個方向可行？',
    options: [
      '只有逆時針可行',
      '兩個方向都可行',
      '只有順時針（向右翻倒）可行，而且三個接觸都會脫離',
      '兩個方向都不可行',
    ],
    answer: 2,
    explanation: '右下角 $c$ 位在兩條底邊法線（朝上）的右側，也在左牆法線（朝右）的右側，三個接觸都把它標成 $-$：只允許順時針。代入 $\\vF_i^{\\T}\\vV = \\omega((p_i-c)\\times\\hat n_i)_z$，$\\omega<0$ 時三項都 $>0$，所以三個接觸都脫離——方塊離開牆、繞右下角往右翻。逆時針會讓左下部分壓進地面。',
  },
  {
    question: '為什麼在空間中，用無摩擦點接觸達成形封閉至少需要 7 個接觸？',
    options: [
      '因為長方體有 6 個面，再多一個抵抗重力',
      'twist（或 wrench）空間是 6 維；要用非負係數張滿 $\\R^6$，至少需要 6 + 1 = 7 個向量',
      '因為每個接觸會限制 2 個自由度',
      '因為空間中的接觸必須成對出現',
    ],
    answer: 1,
    explanation: '每個無摩擦接觸只給一條**單邊**不等式 $\\vF_i^{\\T}\\vV \\ge 0$。$n$ 個向量最多只是一組基底，它們的非負組合是一個尖錐，負方向永遠湊不出來；至少要 $n+1$ 個向量才可能正張成 $\\R^n$。平面 $n=3$ 得 4，空間 $n=6$ 得 7。',
  },
  {
    question: '在方塊四條邊的中點各放一個無摩擦接觸。依一階分析，下列何者正確？',
    options: [
      '是形封閉，因為接觸數已經達到 4',
      '不是形封閉，因為方塊還能往任意方向平移',
      '是形封閉，但只能抵抗力，不能抵抗力矩',
      '不是（一階）形封閉：四條法線交於中心，繞中心轉動時每個接觸都是 $\\vF_i^{\\T}\\vV = 0$，兩個方向都沒被排除',
    ],
    answer: 3,
    explanation: '四個法向力都通過中心，對中心的力矩全為 0：wrench 只張出 2 維（rank = 2），任何繞中心的力偶都平衡不了。平移倒是被四個方向卡死了。至於方塊實際上轉不轉得動，屬於二階效應（平邊轉一點就會頂到手指），一階分析無法下結論，所以它不算一階形封閉；實務上會把法線錯開。',
  },
  {
    question: '某個試探運動得到三個接觸的 $\\vF_i^{\\T}\\vV$ 分別為 $0,\\ 0.4,\\ -0.1$。這代表什麼？',
    options: [
      '這個運動不可能發生：第三個接觸會穿透',
      '可行：接觸 1 滑動或滾動、接觸 2 脫離、接觸 3 也脫離',
      '可行，因為三個數加起來大於 0',
      '只要第三根手指出更大的力就能做到',
    ],
    answer: 0,
    explanation: '每個接觸都必須各自滿足 $\\vF_i^{\\T}\\vV \\ge 0$，不能互相抵銷。$-0.1$ 代表接觸點往手指內部移動，也就是穿透；手指出再大的力也只會更強烈地阻止它。',
  },
];

// ------------------------------------------------------------------ sim constants
type ShapeKey = 'square' | 'L' | 'tri';
type ProbeMode = 'ccw' | 'cw' | 'trans';

const SHAPES: Record<ShapeKey, PolyShape> = {
  square: makeShape('square', '方塊', [[-0.75, -0.75], [0.75, -0.75], [0.75, 0.75], [-0.75, 0.75]]),
  L: makeShape('L', 'L 形', [[0, 0], [1.5, 0], [1.5, 0.55], [0.55, 0.55], [0.55, 1.3], [0, 1.3]]),
  tri: makeShape('tri', '三角形', [[-0.85, -0.49], [0.85, -0.49], [0, 0.98]]),
};
/** L on the floor (two contacts) against a wall on the left (one contact), in the L's drawing coordinates. */
const L_FLOOR_WALL: P2[] = [[0.25, 0], [1.3, 0], [0, 0.9]];
const SQUARE_MIDPOINTS = [0.75, 2.25, 3.75, 5.25];

const K_V = 0.4; // world units per m/s (velocity arrows)
const K_F = 0.1; // world units per N (force arrows)
const FINGER = 0.075; // fingertip centre offset from the contact point
const GRIP = 0.1; // arrow handles sit this far beyond the arrow tip
const F_MAX = 10; // N, external force magnitude limit
const ANIM_T = 1.2; // s
const MAX_CONTACTS = 6;

const MODE_VAR: Record<ContactMode, string> = { rolling: '--ok', sliding: '--warn', breaking: '--series-6', penetrating: '--danger' };
const MODE_TXT: Record<ContactMode, string> = { rolling: '滾動 R', sliding: '滑動 S', breaking: '脫離 B', penetrating: '穿透 ✗' };
const MODE_IDX: Record<ProbeMode, number> = { ccw: 0, cw: 1, trans: 2 };

const STYLE = `
.ch12a-ct { border-collapse: collapse; font-size: 0.8rem; width: 100%; }
.ch12a-ct th, .ch12a-ct td { padding: 0.12rem 0.45rem; border-bottom: 1px solid var(--border); text-align: left; white-space: nowrap; }
.ch12a-ct th { color: var(--fg-muted); font-weight: 600; }
.ch12a-ct td.num { font-family: var(--mono); font-variant-numeric: tabular-nums; }
.ch12a-pill { display: inline-block; padding: 0 0.5rem; border-radius: 999px; font-size: 0.74rem; color: var(--sim-bg); font-weight: 600; }
.ch12a-card { border: 1px dashed var(--border); border-radius: var(--radius-sm); padding: 0.35rem 0.7rem; background: var(--bg-elev); margin: 0.4rem 0; font-size: 0.86rem; }
.ch12a-card p { margin: 0.25em 0; }
`;

const fmtV = (v: ArrayLike<number>, d = 2): string => `(${Array.from(v, (x) => fmt(x, d)).join(', ')})`;
const ease = (t: number): number => { const u = Math.min(1, Math.max(0, t)); return u * u * (3 - 2 * u); };
const clamp = (x: number, lo: number, hi: number): number => Math.min(hi, Math.max(lo, x));

function describeTwist(V: V3): string {
  const c = corOf(V);
  if (!c) return `沿 ${fmt((Math.atan2(V[2], V[1]) * 180) / Math.PI, 0)}° 方向平移`;
  return `繞 (${fmt(c[0], 2)}, ${fmt(c[1], 2)}) ${V[0] > 0 ? '逆時針' : '順時針'}轉`;
}

// ------------------------------------------------------------------ simulation
function mount(host: HTMLElement, ctx: LessonContext): LessonHandle {
  // ---------------- state
  let shapeKey: ShapeKey = 'square';
  let shape = SHAPES[shapeKey];
  let contacts: number[] = [0.95]; // arc-length parameters along the boundary
  let mode: ProbeMode = 'trans';
  let cor: P2 = [-0.9, -0.25];
  let tv: P2 = [0.22, -0.18]; // translation probe = displacement of the ghost
  const push = { on: false, vf: [0, 1] as P2 };
  const ext = { on: false, a: [0.3, 0.25] as P2, f: [-3, 4] as P2 };
  const show = { cor: false, vel: true };
  let anim = -1;
  let configuring = false;
  let dragging = false;
  let grab: P2 = [0, 0];
  let step = -1;
  const autoCache: Partial<Record<ShapeKey, number[]>> = {};

  // ---------------- derived
  let cs: BoundaryPoint[] = [];
  let Fs: V3[] = [];
  let lines: ForceLine[] = [];
  let V: V3 = [0, 0, 1];
  let states: ContactState[] = [];
  let escape: V3 | null = null;
  let cert: number[] | null = null;
  let rank = 0;
  let resist: ResistResult | null = null;

  // ---------------- layout
  const styleEl = el('style', {}, STYLE);
  host.appendChild(styleEl);
  const titleEl = simTitle(host, '平面物體與無摩擦點接觸');
  const badge = el('span', { class: 'badge' }, '—');
  titleEl.appendChild(badge);
  const cv = new Canvas2D(host, { aspect: 16 / 11, world: { xmin: -2.3, xmax: 2.3, ymin: -1.55, ymax: 1.55 }, overlay: true });

  const panel = controlPanel(host);
  const chips = presetChips(row(panel), [
    { label: '單一接觸', apply: () => configure({ shape: 'square', contacts: [0.95], mode: 'trans', tv: [0.22, -0.18], showCor: false, vel: true }) },
    { label: '推一下', apply: () => configure({ shape: 'square', contacts: [0.95], mode: 'trans', tv: [0, 0.3], showCor: false, vel: true, push: true, vf: [0, 1] }) },
    { label: 'CoR：一個接觸', apply: () => configure({ shape: 'square', contacts: [0.95], mode: 'ccw', cor: [-0.9, -0.25], showCor: true, vel: false }) },
    { label: 'L 形三點', apply: () => configure({ shape: 'L', points: L_FLOOR_WALL, mode: 'ccw', cor: [-1.15, 1.05], showCor: true, vel: false }) },
    { label: '四點形封閉', apply: () => configure({ shape: 'square', auto: true, mode: 'ccw', cor: [0, 0], showCor: true, vel: false }) },
    { label: '法線共點', apply: () => configure({ shape: 'square', contacts: SQUARE_MIDPOINTS, mode: 'ccw', cor: [0, 0], showCor: true, vel: true }) },
    { label: '外力測試', apply: () => configure({ shape: 'square', auto: true, mode: 'ccw', cor: [0, 0], showCor: false, vel: false, ext: true, extA: [0.3, 0.25], extF: [-3, 4] }) },
    { label: '外力：L 形三點', apply: () => configure({ shape: 'L', points: L_FLOOR_WALL, mode: 'ccw', cor: [-1.15, 1.05], showCor: false, vel: false, ext: true, extA: [0, 0], extF: [2.5, 3] }) },
  ], { label: '情境' });

  const r1 = row(panel);
  const shapeSel = select<ShapeKey>(r1, {
    label: '物體',
    options: (Object.keys(SHAPES) as ShapeKey[]).map((k) => ({ value: k, label: SHAPES[k].name })),
    value: shapeKey,
    onChange: (k) => { setShape(k); chips.clear(); },
  });
  const addBtn = button(r1, { label: '＋接觸', small: true, onClick: () => { addContact(); chips.clear(); } });
  const delBtn = button(r1, {
    label: '－接觸', small: true,
    onClick: () => { if (contacts.length > 1) { contacts.pop(); rebuildFingerHandles(); chips.clear(); update(); } },
  });
  const autoBtn = button(r1, { label: '自動找形封閉（4 點）', small: true, variant: 'primary', onClick: () => { autoClose(); chips.clear(); } });
  button(r1, { label: '▶ 試動', small: true, title: '播放試探運動', onClick: () => startAnim() });

  const r2 = row(panel);
  const probeChips = presetChips(r2, [
    { label: '↺ 逆時針轉', apply: () => setMode('ccw') },
    { label: '↻ 順時針轉', apply: () => setMode('cw') },
    { label: '⇢ 平移', apply: () => setMode('trans') },
  ], { label: '試探運動' });
  const corTog = toggle(r2, { label: 'CoR 可行區', value: show.cor, onChange: (v) => { show.cor = v; chips.clear(); update(); } });
  const velTog = toggle(r2, { label: '接觸點速度', value: show.vel, onChange: (v) => { show.vel = v; chips.clear(); update(); } });

  // push ("推一下") panel
  const pushPanel = controlPanel(host, { compact: true });
  const pushTog = toggle(row(pushPanel), {
    label: '推一下：給手指 1 一個速度 v_f（拖橘色箭頭尖端）', value: push.on,
    onChange: (v) => { push.on = v; chips.clear(); update(); },
  });
  const pushKv = kv(pushPanel, ['𝓕₁ᵀ𝒱（物體）', 'n̂₁ᵀv_f（手指）', '相對法向速度', '接觸 1 的模式']);
  pushKv.el.style.gridColumn = '1 / -1';

  // external force panel
  const extPanel = controlPanel(host, { compact: true });
  const extTog = toggle(row(extPanel), {
    label: '外力測試：拖曳外力的尖端（大小、方向）與圓點（作用點）', value: ext.on,
    onChange: (v) => { ext.on = v; chips.clear(); update(); },
  });
  const extKv = kv(extPanel, ['外力 𝓕_ext = (m, fₓ, f_y)', '接觸力 kᵢ (N)', '結論']);
  extKv.el.style.gridColumn = '1 / -1';

  const table = el('table', { class: 'ch12a-ct' });
  host.appendChild(el('div', { style: { overflowX: 'auto', margin: '0.35rem 0' } }, table));
  const info = kv(host, ['試探 twist 𝒱', '判定', '形封閉（一階）', '檢查']);
  legend(host, [
    { color: 'var(--ok)', label: '滾動 R' },
    { color: 'var(--warn)', label: '滑動 S' },
    { color: 'var(--series-6)', label: '脫離 B' },
    { color: 'var(--danger)', label: '穿透 ✗' },
    { color: 'var(--series-1)', label: 'CoR 標 +：只能逆時針' },
    { color: 'var(--series-5)', label: 'CoR 標 −：只能順時針' },
  ]);
  const preview = mdElement(
    '**下一課預告：摩擦錐。** 有摩擦時，接觸力可以偏離法線，只要 $|f_t| \\le \\mu f_n$——能出的力從一條線變成半角 $\\tan^{-1}\\mu$ 的錐。兩根手指只要連線落在兩個錐內，就能抵抗任何 wrench。',
    'ch12a-card',
  );
  host.appendChild(preview);
  note(host, '判定只用一階（速度）分析；虛影把運動放大畫出。𝓕ᵀ𝒱 剛好為 0 時，實際能否滑過去取決於曲率等二階效應。twist 已正規化：旋轉 |ω| = 1，平移 |v| = 1。');

  // ---------------- handles
  let fingerHandles: Handle[] = [];
  const fingerTip = (i: number): P2 => {
    const b = boundaryAt(shape.verts, contacts[i]);
    return [b.p[0] - FINGER * b.n[0], b.p[1] - FINGER * b.n[1]];
  };
  function rebuildFingerHandles(): void {
    fingerHandles.forEach((h) => cv.removeHandle(h));
    fingerHandles = contacts.map((_, i) => cv.addHandle({
      get: () => fingerTip(i),
      set: (q) => { contacts[i] = projectToBoundary(shape.verts, q).s; chips.clear(); update(); },
      radius: 9,
      label: String(i + 1),
    }));
  }

  // Arrow handles sit GRIP beyond the arrow tip so the arrowhead stays visible; `untip` maps the
  // handle position back to the tip.
  const untip = (from: P2, q: P2): P2 => { const d = sub2(q, from); return add2(from, scale2(unit2(d), Math.max(0, norm2(d) - GRIP))); };
  const corHandle = cv.addHandle({ get: () => cor, set: (q) => { cor = snapCor(q); chips.clear(); update(); }, radius: 8, label: 'CoR' });
  const transHandle = cv.addHandle({
    get: () => add2(tv, scale2(unit2(tv), GRIP)),
    set: (q) => { tv = snapTrans(untip([0, 0], q)); chips.clear(); update(); },
    radius: 8,
  });
  const vfHandle = cv.addHandle({
    get: () => (cs.length ? add2(cs[0].p, add2(scale2(push.vf, K_V), scale2(unit2(push.vf), GRIP))) : [0, 0]),
    set: (q) => { if (cs.length) push.vf = snapVf(untip(cs[0].p, q)); chips.clear(); update(); },
    radius: 7,
  });
  const extAHandle = cv.addHandle({
    get: () => ext.a,
    set: (q) => { if (insidePolygon(q, shape.verts)) ext.a = q; chips.clear(); update(); },
    radius: 6,
  });
  const extFHandle = cv.addHandle({
    get: () => add2(ext.a, add2(scale2(ext.f, K_F), scale2(unit2(ext.f), GRIP))),
    set: (q) => {
      let f = scale2(sub2(untip(ext.a, q), ext.a), 1 / K_F);
      const L = norm2(f);
      if (L > F_MAX) f = scale2(f, F_MAX / L);
      ext.f = f; chips.clear(); update();
    },
    radius: 8,
  });

  // drag the body itself → translation probe
  const offPointer = cv.onPointer({
    down: (p, ev) => {
      if (!insidePolygon(p, shape.verts)) return;
      dragging = true; grab = p;
      cv.canvas.setPointerCapture(ev.pointerId);
      if (mode !== 'trans') probeChips.activate(MODE_IDX.trans);
    },
    move: (p) => { if (dragging) { tv = snapTrans(sub2(p, grab)); chips.clear(); update(); } },
    up: () => { dragging = false; },
  });

  // ---------------- snapping helpers
  function snapCor(q: P2): P2 {
    const vis = cv.visible();
    const c: P2 = [clamp(q[0], vis.xmin + 0.05, vis.xmax - 0.05), clamp(q[1], vis.ymin + 0.05, vis.ymax - 0.05)];
    const px = 1 / Math.max(cv.scale, 1e-6);
    for (const b of cs) if (norm2(sub2(c, b.p)) < 11 * px) return [b.p[0], b.p[1]];
    for (let i = 0; i < lines.length; i++) {
      for (let j = i + 1; j < lines.length; j++) {
        const X = lineIntersection(lines[i], lines[j]);
        if (X && norm2(sub2(c, X)) < 11 * px) return X;
      }
    }
    for (const L of lines) {
      const foot = add2(L.p, scale2(L.d, dot2(sub2(c, L.p), L.d)));
      if (norm2(sub2(c, foot)) < 7 * px) return foot;
    }
    return c;
  }

  function snapTrans(q: P2): P2 {
    let v: P2 = [q[0], q[1]];
    const L = norm2(v);
    if (L < 0.06) v = scale2(L > 1e-9 ? unit2(v) : [1, 0], 0.06);
    else if (L > 0.6) v = scale2(v, 0.6 / L);
    const len = norm2(v);
    const cos4 = Math.cos((4 * Math.PI) / 180);
    for (const b of cs) {
      for (const t of [perp(b.n), scale2(perp(b.n), -1)]) if (dot2(unit2(v), t) > cos4) return scale2(t, len);
    }
    return v;
  }

  function snapVf(q: P2): P2 {
    if (!cs.length) return push.vf;
    const b = cs[0];
    let w = scale2(sub2(q, b.p), 1 / K_V);
    const L = norm2(w);
    if (L > 2.5) w = scale2(w, 2.5 / L);
    const vp = pointVelocity(V, b.p);
    const px = 1 / Math.max(cv.scale, 1e-6);
    if (norm2(sub2(w, vp)) * K_V < 10 * px) return [vp[0], vp[1]];
    const off = dot2(b.n, sub2(w, vp));
    if (Math.abs(off) * K_V < 7 * px) return sub2(w, scale2(b.n, off));
    return w;
  }

  // ---------------- model updates
  const probeTwist = (): V3 => (mode === 'trans' ? translationTwist(unit2(tv)) : twistAbout(cor, mode === 'ccw' ? 1 : -1));

  function recompute(): void {
    cs = contacts.map((s) => boundaryAt(shape.verts, s));
    Fs = cs.map((b) => contactWrench(b.p, b.n));
    lines = cs.map((b) => ({ p: b.p, d: b.n }));
    V = probeTwist();
    states = cs.map((b, i) => classifyContact(b.p, b.n, V, push.on && i === 0 ? push.vf : [0, 0]));
    escape = preferredEscape(Fs);
    rank = wrenchRank(Fs);
    cert = escape ? null : closureCertificate(Fs);
    resist = ext.on ? resistWrench(Fs, forceWrench(ext.a, ext.f)) : null;
  }

  function refreshDom(): void {
    const closed = escape === null;
    badge.className = closed ? 'badge ok' : 'badge';
    badge.textContent = closed ? '形封閉（一階）✓' : `未形封閉 · ${contacts.length} 個接觸`;
    addBtn.disabled = contacts.length >= MAX_CONTACTS;
    delBtn.disabled = contacts.length <= 1;

    // contact table: probe columns (normal rate, mode) or, in the external-force test, the contact forces
    const head = el('tr', {},
      el('th', {}, '接觸'), el('th', {}, '𝓕ᵢ = (m, fₓ, f_y)'),
      ext.on ? null : el('th', {}, '法向速度'), ext.on ? null : el('th', {}, '模式'),
      ext.on ? el('th', {}, resist?.ok ? '推力 kᵢ (N)' : '最小平方 kᵢ (N)') : null);
    const rowsEl = cs.map((_, i) => {
      const st = states[i];
      const rel = push.on && i === 0;
      const kval = ext.on && resist ? (resist.ok ? resist.k[i] : resist.kLS[i]) : 0;
      return el('tr', {},
        el('td', {}, String(i + 1)),
        el('td', { class: 'num' }, fmtV(Fs[i])),
        ext.on ? null : el('td', { class: 'num' }, `${fmt(st.normalRate, 3)}${rel ? '（相對）' : ''}`),
        ext.on ? null : el('td', {}, el('span', { class: 'ch12a-pill', style: { background: `var(${MODE_VAR[st.mode]})` } }, MODE_TXT[st.mode])),
        ext.on ? el('td', { class: 'num', style: { color: kval < -1e-9 ? 'var(--danger)' : '' } }, fmt(kval, 2)) : null);
    });
    table.replaceChildren(head, ...rowsEl);

    // summary (the probe rows are irrelevant while the external-force test is on)
    info.show('試探 twist 𝒱', !ext.on);
    info.show('判定', !ext.on);
    info.set('試探 twist 𝒱', `${fmtV(V)} — ${describeTwist(V)}`);
    const bad = states.map((s, i) => (s.mode === 'penetrating' ? i + 1 : 0)).filter((i) => i > 0);
    info.set('判定', bad.length ? `✗ 不可能：接觸 ${bad.join('、')} 會穿透` : '✓ 可行：每個接觸都 ≥ 0');
    info.set('形封閉（一階）', closed ? '✓ 除了 𝒱 = 0 沒有其他可行運動' : `✗ 仍可${describeTwist(escape!)}（例）`);
    info.set('檢查', `j = ${contacts.length}（平面至少 4）· rank = ${rank}` + (cert ? ` · 證書 k = ${fmtV(cert)} > 0，Σkᵢ𝓕ᵢ = 0` : ''));

    // push panel
    pushPanel.style.display = step === 1 || push.on ? '' : 'none';
    if (cs.length) {
      const st = states[0];
      pushKv.set('𝓕₁ᵀ𝒱（物體）', fmt(st.power, 3));
      pushKv.set('n̂₁ᵀv_f（手指）', push.on ? fmt(dot2(cs[0].n, push.vf), 3) : '0（手指不動）');
      pushKv.set('相對法向速度', `${fmt(st.normalRate, 3)} ${st.normalRate < -1e-7 ? '< 0 ✗' : '≥ 0 ✓'}`);
      pushKv.set('接觸 1 的模式', MODE_TXT[st.mode]);
    }

    // external force panel
    extPanel.style.display = step === 5 || ext.on ? '' : 'none';
    if (ext.on && resist) {
      extKv.set('外力 𝓕_ext = (m, fₓ, f_y)', `${fmtV(forceWrench(ext.a, ext.f), 2)}  (N·m, N, N)`);
      if (resist.ok) {
        extKv.set('接觸力 kᵢ (N)', fmtV(resist.k, 2));
        extKv.set('結論', '✓ 只用推力（kᵢ ≥ 0）就能平衡');
      } else {
        const neg = resist.kLS.map((k, i) => (k < -1e-9 ? i + 1 : 0)).filter((i) => i > 0);
        extKv.set('接觸力 kᵢ (N)', `最小平方解 ${fmtV(resist.kLS, 2)}`);
        extKv.set('結論', neg.length && resist.residual < 1e-6
          ? `✗ 需要接觸 ${neg.join('、')} 出負力（拉住物體）——做不到，物體會被帶走`
          : '✗ 沒有任何接觸力組合能產生這個 wrench，物體會被帶走');
      }
    } else {
      extKv.set('外力 𝓕_ext = (m, fₓ, f_y)', '（關閉）');
      extKv.set('接觸力 kᵢ (N)', '–');
      extKv.set('結論', '–');
    }
    preview.style.display = step === 5 ? '' : 'none';

    // handles
    corHandle.enabled = mode !== 'trans' && !ext.on;
    transHandle.enabled = mode === 'trans' && !ext.on;
    vfHandle.enabled = push.on && cs.length > 0;
    extAHandle.enabled = ext.on;
    extFHandle.enabled = ext.on;
  }

  function update(): void {
    recompute();
    refreshDom();
    cv.requestRender();
  }

  function setMode(m: ProbeMode): void {
    mode = m;
    if (!configuring) chips.clear();
    update();
  }

  function setShape(k: ShapeKey): void {
    shapeKey = k;
    shape = SHAPES[k];
    const P = perimeter(shape.verts);
    contacts = contacts.map((_, i) => (P * (i + 0.15)) / contacts.length);
    if (!insidePolygon(ext.a, shape.verts)) ext.a = [0, 0];
    rebuildFingerHandles();
    update();
  }

  function addContact(): void {
    if (contacts.length >= MAX_CONTACTS) return;
    const P = perimeter(shape.verts);
    let bestS = 0, bestD = -1;
    for (let k = 0; k < 96; k++) {
      const b = boundaryAt(shape.verts, (P * k) / 96);
      const d = cs.length ? Math.min(...cs.map((c) => norm2(sub2(c.p, b.p)))) : 1;
      if (d > bestD) { bestD = d; bestS = b.s; }
    }
    contacts.push(bestS);
    rebuildFingerHandles();
    update();
  }

  function autoContacts(): number[] {
    if (!autoCache[shapeKey]) autoCache[shapeKey] = autoFormClosure(shape.verts)?.s ?? [0.3, 1.9, 3.4, 5.1];
    return autoCache[shapeKey]!.slice();
  }

  function autoClose(): void {
    contacts = autoContacts();
    show.cor = true;
    corTog.set(true, { silent: true });
    rebuildFingerHandles();
    update();
  }

  interface Config {
    shape: ShapeKey; contacts?: number[]; points?: P2[]; auto?: boolean;
    mode: ProbeMode; cor?: P2; tv?: P2; showCor: boolean; vel: boolean; push?: boolean; vf?: P2;
    ext?: boolean; extA?: P2; extF?: P2;
  }
  function configure(c: Config): void {
    configuring = true;
    shapeKey = c.shape;
    shape = SHAPES[c.shape];
    shapeSel.set(c.shape, { silent: true });
    if (c.auto) contacts = autoContacts();
    else if (c.points) contacts = c.points.map((q) => projectToBoundary(shape.verts, sub2(q, shape.offset)).s);
    else contacts = (c.contacts ?? [0.95]).slice();
    if (c.cor) cor = [c.cor[0], c.cor[1]];
    if (c.tv) tv = [c.tv[0], c.tv[1]];
    show.cor = c.showCor; corTog.set(c.showCor, { silent: true });
    show.vel = c.vel; velTog.set(c.vel, { silent: true });
    push.on = !!c.push; pushTog.set(push.on, { silent: true });
    if (c.vf) push.vf = [c.vf[0], c.vf[1]];
    ext.on = !!c.ext; extTog.set(ext.on, { silent: true });
    if (c.extA) ext.a = [c.extA[0], c.extA[1]];
    if (c.extF) ext.f = [c.extF[0], c.extF[1]];
    if (!insidePolygon(ext.a, shape.verts)) ext.a = [0, 0];
    anim = -1;
    rebuildFingerHandles();
    probeChips.activate(MODE_IDX[c.mode]); // applies the mode and updates everything
    configuring = false;
  }

  // ---------------- animation of the probe motion
  const loop = createLoop((dt) => {
    anim += dt;
    if (anim >= ANIM_T) { anim = -1; loop.stop(); }
    cv.requestRender();
  });
  function startAnim(): void { anim = 0; loop.start(); }

  function ghostPose(phase: number): { x: number; y: number; theta: number } {
    if (mode === 'trans') return { x: tv[0] * phase, y: tv[1] * phase, theta: 0 };
    return planarExp(V, displacementScale(V, shape.verts, 0.24) * phase);
  }

  // ---------------- drawing
  function vecArrow(c: Canvas2D, from: P2, v: P2, k: number, color: string, cap = 0.75): void {
    let d = scale2(v, k);
    const L = norm2(d);
    if (L < 1e-6) { c.point(from, 4.5, { color }); return; }
    if (L > cap) d = scale2(d, cap / L);
    c.arrow(from, add2(from, d), { color, width: 2.4, head: 9 });
  }

  function rotGlyph(c: Canvas2D, ctr: P2, r: number, sense: 1 | -1, color: string): void {
    const a0 = 0.4, a1 = 0.4 + 1.55 * Math.PI;
    c.arc(ctr, r, a0, a1, { color, width: 2 });
    const aEnd = sense > 0 ? a1 : a0;
    const tip: P2 = [ctr[0] + r * Math.cos(aEnd), ctr[1] + r * Math.sin(aEnd)];
    const tan: P2 = sense > 0 ? [-Math.sin(aEnd), Math.cos(aEnd)] : [Math.sin(aEnd), -Math.cos(aEnd)];
    c.arrow(sub2(tip, scale2(tan, 0.07)), tip, { color, width: 2, head: 9 });
  }

  /**
   * Anchor for a region label: a grid point inside the region, on screen (clear of the overlay line at the
   * top), whose label box (≈ 1.1 × 0.24 world units) keeps the most clearance from the `avoid` points,
   * with a mild preference for the middle of the region.
   */
  function regionLabelPoint(poly: P2[], vis: World, avoid: P2[]): P2 {
    let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity, cx = 0, cy = 0;
    for (const q of poly) { x0 = Math.min(x0, q[0]); x1 = Math.max(x1, q[0]); y0 = Math.min(y0, q[1]); y1 = Math.max(y1, q[1]); cx += q[0]; cy += q[1]; }
    const cen: P2 = [cx / poly.length, cy / poly.length];
    x0 = Math.max(x0, vis.xmin + 0.6); x1 = Math.min(x1, vis.xmax - 0.6);
    y0 = Math.max(y0, vis.ymin + 0.15); y1 = Math.min(y1, vis.ymax - 0.38);
    const clearance = (q: P2, a: P2) => Math.max(Math.abs(q[0] - a[0]) - 0.55, Math.abs(q[1] - a[1]) - 0.12);
    let best: P2 | null = null, bestScore = -Infinity;
    const N = 10;
    for (let i = 0; i <= N && x1 >= x0 && y1 >= y0; i++) {
      for (let j = 0; j <= N; j++) {
        const q: P2 = [x0 + ((x1 - x0) * i) / N, y0 + ((y1 - y0) * j) / N];
        if (!insidePolygon(q, poly)) continue;
        const clear = Math.min(0.6, ...avoid.map((a) => clearance(q, a)));
        const score = clear - 0.03 * norm2(sub2(q, cen));
        if (score > bestScore) { bestScore = score; best = q; }
      }
    }
    return best ?? cen;
  }

  /** Region labels of the CoR picture, drawn after the body and fingers so nothing covers them. */
  let corLabels: { p: P2; text: string; color: string }[] = [];

  function drawCorPicture(c: Canvas2D, vis: World, font: string): void {
    corLabels = [];
    const pad = 0.3;
    const box: P2[] = [[vis.xmin - pad, vis.ymin - pad], [vis.xmax + pad, vis.ymin - pad], [vis.xmax + pad, vis.ymax + pad], [vis.xmin - pad, vis.ymax + pad]];
    const plusC = cssVar('--series-1'), minusC = cssVar('--series-5'), muted = cssVar('--fg-muted'), warn = cssVar('--warn');
    const bg = cssVar('--bg-elev');
    const Rp = corRegion(lines, 1, box), Rm = corRegion(lines, -1, box);
    const aP = polygonAreaAbs(Rp), aM = polygonAreaAbs(Rm);
    if (aP > 1e-4) { c.polygon(Rp, { fill: plusC, alpha: 0.15, width: 0 }); c.polygon(Rp, { color: plusC, alpha: 0.55, width: 1.2 }); }
    if (aM > 1e-4) { c.polygon(Rm, { fill: minusC, alpha: 0.15, width: 0 }); c.polygon(Rm, { color: minusC, alpha: 0.55, width: 1.2 }); }
    for (const L of lines) c.line(add2(L.p, scale2(L.d, -12)), add2(L.p, scale2(L.d, 12)), { color: muted, width: 1, dash: [5, 5], alpha: 0.75 });
    if (lines.length <= 3) {
      // per-line labels just outside the finger: + on the left of the directed normal, ± on it, − on the right
      for (const L of lines) {
        const base = add2(L.p, scale2(L.d, -0.62));
        c.text(add2(base, scale2(perp(L.d), 0.2)), '+', { color: plusC, font: `bold 15px ${font}` });
        c.text(add2(base, scale2(perp(L.d), -0.2)), '−', { color: minusC, font: `bold 15px ${font}` });
        c.text(base, '±', { color: warn, font: `bold 13px ${font}`, bg });
      }
    }
    const compass: P2 = [vis.xmax - 0.42, vis.ymax - 0.4];
    const avoid: P2[] = [
      compass, ...cs.map((_, i) => fingerTip(i)), ...(mode === 'trans' ? [] : [cor, add2(cor, [0, 0.17])]),
      ...(lines.length <= 3 ? lines.map((L) => add2(L.p, scale2(L.d, -0.62))) : []),
    ];
    if (aP > 1e-4) corLabels.push({ p: regionLabelPoint(Rp, vis, avoid), text: '+ 區：可逆時針轉', color: '--series-1' });
    if (aM > 1e-4) corLabels.push({ p: regionLabelPoint(Rm, vis, avoid), text: '− 區：可順時針轉', color: '--series-5' });
    for (const q of feasibleCorPoints(lines)) {
      const both = q.plus && q.minus;
      c.point(q.c, 4.5, { color: both ? warn : q.plus ? plusC : minusC });
      if (both && aP <= 1e-4 && aM <= 1e-4) c.text(q.c, '± 兩向都沒被排除', { dy: -14, color: warn, bg, font: `600 12px ${font}` });
    }
    // translation compass (CoR at infinity)
    const R = 0.25;
    const ctr = compass;
    const okC = cssVar('--ok');
    c.circle(ctr, R + 0.06, { fill: bg, color: cssVar('--border'), width: 1, alpha: 0.95 });
    c.circle(ctr, R, { color: cssVar('--grid-strong'), width: 1.2 });
    const tc = translationCone(cs.map((b) => b.n));
    if (tc.kind === 'all') c.circle(ctr, R, { color: okC, width: 4 });
    else if (tc.kind === 'arcs') {
      for (const [a0, a1] of tc.arcs) {
        const pts: P2[] = [ctr];
        for (let k = 0; k <= 24; k++) { const a = a0 + ((a1 - a0) * k) / 24; pts.push([ctr[0] + R * Math.cos(a), ctr[1] + R * Math.sin(a)]); }
        c.polygon(pts, { fill: okC, alpha: 0.3, width: 0 });
        c.arc(ctr, R, a0, a1, { color: okC, width: 4 });
      }
      for (const a of tc.rays) c.arrow(ctr, [ctr[0] + R * Math.cos(a), ctr[1] + R * Math.sin(a)], { color: okC, width: 2.5, head: 7 });
    } else c.text(ctr, '✗', { color: cssVar('--danger'), font: `bold 15px ${font}` });
    c.text([ctr[0], ctr[1] - R - 0.15], '可行平移', { color: muted, font: `11px ${font}` });
  }

  cv.draw = (c) => {
    const font = cssVar('--font') || 'sans-serif';
    const fg = cssVar('--fg'), muted = cssVar('--fg-muted'), accent = cssVar('--accent'), warn = cssVar('--warn');
    const danger = cssVar('--danger'), okC = cssVar('--ok'), bg = cssVar('--bg-elev');
    // theme-aware handle colours
    fingerHandles.forEach((h, i) => { h.color = ext.on ? muted : cssVar(MODE_VAR[states[i]?.mode ?? 'breaking']); });
    corHandle.color = cssVar('--series-5');
    transHandle.color = accent;
    vfHandle.color = warn;
    extAHandle.color = warn;
    extFHandle.color = warn;

    c.grid(0.5);
    const vis = c.visible();
    if (show.cor) drawCorPicture(c, vis, font);

    // body
    c.polygon(shape.verts, { fill: cssVar('--bg-sunken'), color: fg, width: 2 });
    c.point([0, 0], 3, { color: muted });

    // ghost of the probe motion
    const bad = states.some((s) => s.mode === 'penetrating');
    if (!ext.on) {
      const pose = ghostPose(anim < 0 ? 1 : ease(anim / ANIM_T));
      const gpts = shape.verts.map((q) => applyPose(pose, q));
      const gcol = bad ? danger : accent;
      c.polygon(gpts, { fill: gcol, alpha: 0.1, width: 0 });
      c.polygon(gpts, { color: gcol, width: 1.8, dash: [6, 4], alpha: 0.9 });
    }

    // fingers and normals (in the force test the contact forces take the place of the normal arrows)
    cs.forEach((b, i) => {
      const tip = fingerTip(i);
      c.line(tip, add2(tip, scale2(b.n, -0.34)), { color: muted, width: 10, cap: 'round', alpha: ext.on ? 0.25 : 0.45 });
      if (!ext.on) c.arrow(b.p, add2(b.p, scale2(b.n, 0.3)), { color: fg, width: 1.6, head: 8 });
    });

    // contact-point velocities and mode labels
    if (show.vel && !ext.on) {
      cs.forEach((b, i) => {
        const st = states[i];
        const mc = cssVar(MODE_VAR[st.mode]);
        const vp = pointVelocity(V, b.p);
        if (push.on && i === 0) {
          vecArrow(c, b.p, vp, K_V, accent);
          vecArrow(c, b.p, push.vf, K_V, warn);
          c.text(add2(b.p, scale2(push.vf, K_V)), 'v_f', { dx: 14, dy: -6, color: warn, font: `600 12px ${font}` });
          if (norm2(vp) > 1e-6) c.text(add2(b.p, scale2(vp, K_V)), 'ṗ', { dx: -12, dy: -6, color: accent, font: `600 13px ${font}` });
        } else vecArrow(c, b.p, vp, K_V, mc);
        c.text(add2(b.p, scale2(b.n, -0.6)), MODE_TXT[st.mode], { color: mc, bg, font: `600 12px ${font}` });
      });
    }

    // probe glyphs
    if (!ext.on) {
      if (mode === 'trans') {
        c.arrow([0, 0], tv, { color: accent, width: 2.6, head: 11 });
      } else {
        rotGlyph(c, cor, 0.2, mode === 'ccw' ? 1 : -1, cssVar('--series-5'));
      }
    }
    if (show.cor) for (const L of corLabels) c.text(L.p, L.text, { color: L.color, bg, font: `600 12px ${font}` });

    // external force test
    if (ext.on && resist) {
      const tip = add2(ext.a, scale2(ext.f, K_F));
      c.arrow(ext.a, tip, { color: warn, width: 3, head: 12 });
      c.text(add2(tip, scale2(unit2(ext.f), GRIP + 0.2)), '𝓕_ext', { color: warn, bg, font: `600 12px ${font}` });
      // force of each finger on the body, drawn from the contact point: a push points into the body,
      // a (physically impossible) pull points out of it
      cs.forEach((b, i) => {
        const k = resist!.ok ? resist!.k[i] : resist!.kLS[i];
        if (Math.abs(k) < 1e-6) { c.point(b.p, 3.5, { color: muted }); return; }
        const len = Math.max(0.12, Math.min(0.9, Math.abs(k) * K_F));
        if (k > 0) {
          const col = resist!.ok ? okC : muted;
          c.arrow(b.p, add2(b.p, scale2(b.n, len)), { color: col, width: 3.2, head: 11, dash: resist!.ok ? undefined : [4, 3] });
        } else {
          // beside the finger (which would hide it), pointing out of the body
          const start = add2(b.p, scale2(perp(b.n), 0.15));
          const end = add2(start, scale2(b.n, -Math.max(0.32, len)));
          c.arrow(start, end, { color: danger, width: 3.2, head: 11 });
          c.text(add2(end, scale2(b.n, -0.13)), '需要拉', { color: danger, bg, font: `600 12px ${font}` });
        }
      });
    }

    c.setOverlayText(ext.on
      ? `𝓕_ext 作用於 (${fmt(ext.a[0], 2)}, ${fmt(ext.a[1], 2)})`
      : `𝒱 = ${fmtV(V)}  ${bad ? '✗ 穿透' : '✓'}`);
  };

  // ---------------- steps
  const STEP_PRESET = [0, 1, 2, 3, 4, 6];
  const focusables: (HTMLElement | null)[][] = [
    [cv.el], [pushPanel], [cv.el, corTog.el], [cv.el, addBtn], [autoBtn, badge], [extPanel],
  ];
  let focused: HTMLElement[] = [];
  const onStep = (i: number): void => {
    const changed = i !== step;
    step = i;
    if (changed) chips.activate(STEP_PRESET[i] ?? 0);
    else update();
    focused.forEach((f) => f.classList.remove('mr-focus'));
    focused = (focusables[i] ?? []).filter((x): x is HTMLElement => !!x);
    focused.forEach((f) => f.classList.add('mr-focus'));
  };
  onStep(ctx.currentStep);

  return {
    onStep,
    destroy() {
      loop.stop();
      offPointer();
      cv.destroy();
      styleEl.remove();
    },
  };
}

const lesson: Lesson = { meta, steps, quiz, mount };
export default lesson;
