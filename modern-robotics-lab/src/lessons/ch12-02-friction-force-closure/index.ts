import type { Lesson, LessonContext, LessonHandle, LessonStep, QuizQuestion } from '@/types';
import {
  el, cssVar, fmt, controlPanel, row, button, select, toggle, slider, angleSlider, simTitle, note, legend, kv, mdElement,
  Canvas2D, presetChips, createLoop, type Handle, type World,
} from '@/ui';
import { meta } from './meta';
import {
  type P2, type V3, type Body, type BoundaryPoint, type HoldResult, type TwoFingerResult,
  makePoly, makeDisc, boundaryAt, projectToBoundary, perimeterOf, insideBody, insidePolygon, outline, bodyRadius, gyrationRadius,
  frictionAngle, unitConeEdges, forceWrench, edgeWrenches, edgeLines, coneCheck, isForceClosure, closureCertificate,
  originMargin, scaleMoments, wrenchRank, twoFingerTest, minGripForces, preferredFreeTwist, drivenFreeTwist, pushAlong,
  accelTwist, corOf, corRegion, polygonAreaAbs, translationCone, planarExp, applyPose, displacementScale,
  add2, sub2, scale2, norm2, perp, unit2, dot2, cross2,
} from './grasp';
import { WrenchHullView } from './hullView';

// ------------------------------------------------------------------ content
const steps: LessonStep[] = [
  {
    title: '有摩擦的接觸',
    md: `上一課的手指只能沿法線推。真實的指尖有摩擦：依**庫倫摩擦（Coulomb friction）**，只要切向力不超過法向力的 $\\mu$ 倍，接觸點就不會相對滑動：

$$|f_t| \\le \\mu f_n ,\\qquad f_n \\ge 0 .$$

於是手指能施加的力不再只是一條線，而是以法線為軸、半角 $\\alpha = \\tan^{-1}\\mu$ 的**摩擦錐（friction cone）**。平面上這個錐剛好由兩條**邊緣向量** $\\hat n \\pm \\mu\\hat t$ 張成：錐裡任何力都是這兩條邊的非負組合。空間中的摩擦錐是圓錐，實務上常用 4～8 條邊的多角錐近似。`,
    simHint: '拖 μ 滑桿看錐張開或收窄。拖曳橘色力箭頭的尖端：留在錐內是「黏住」；超出錐面就變紅——需要的切向力比摩擦給得起的多，手指會滑。',
  },
  {
    title: '兩指抓取的判據',
    md: `兩根有摩擦的手指，能不能抵抗**任何**外力 wrench？平面上有個好記的幾何判據：

> 兩指的連線同時落在兩個摩擦錐的**內部** ⇔ **力封閉（force closure）**。

理由是：沿連線互相擠壓的一對力 $f$ 與 $-f$ 彼此抵消，又都在錐內，所以手指可以任意加大這股**內力（internal force）**，換來更多切向摩擦去對付外力。實際檢查時，比較連線和各自法線的夾角 $\\beta_i$ 與摩擦角：

$$\\beta_1 < \\alpha \\quad\\text{且}\\quad \\beta_2 < \\alpha .$$`,
    simHint: '上下拖動任一根手指，看連線何時跑出錐外、徽章翻紅；或把 μ 調小。再按「兩指夾圓盤」：兩指要多接近正對面才夠？',
  },
  {
    title: '多指：wrench 錐是否張滿',
    md: `手指一多就改用代數判據。每個錐的兩條邊各對應一個 wrench $\\vF = (p\\times f,\\ f)$，$j$ 根手指共 $2j$ 個。接觸能產生的 wrench 就是它們的非負組合，**力封閉**的定義是

$$\\mathrm{pos}\\{\\vF_1, \\ldots, \\vF_{2j}\\} = \\R^3 .$$

等價的檢查：這些點的**凸包（convex hull）**把原點**嚴格**包在內部；或用線性規劃（LP）找出係數全部 $k_i \\ge 1$、使 $\\sum_i k_i \\vF_i = 0$ 的組合，並確認秩為 3。下方 3D 圖把每個邊緣 wrench 畫成 $(f_x, f_y, m/\\rho)$ 的一點，$\\rho$ 是物體尺寸，用來統一單位。`,
    simHint: '三指抓三角形：μ = 0.3 時原點在凸包內（綠）。按「μ = 0」：三條法線交於中心，凸包壓成一片、原點落在面上（紅）——回到上一課「法線共點」的反例。拖曳 3D 圖可旋轉視角。',
  },
  {
    title: '閉合不等於抓得住',
    md: `力封閉只保證：對任何外力 wrench，**存在**一組錐內的接觸力能平衡它——沒說要多大。外力越大、抓取越勉強（連線越貼近錐面），需要的夾持力就越大；手指出力有上限 $f_{\\max}$，超過就會滑脫。

例如兩指水平夾住方塊、承受重量 $W$：兩邊各用摩擦分擔 $W/2$，每指需要

$$f_n \\ge \\frac{W}{2\\mu} .$$

模擬用 LP 找出「最大法向力最小」的力分配，記為 $f^\\ast$，再和 $f_{\\max}$ 比較。`,
    simHint: '按「▶ 施加擾動」：重量從 0 慢慢加到設定值，看長條何時超過 f_max 而滑脫。再把 μ 或 f_max 調大，讓它抓得住；也可以拖擾動箭頭改方向與作用點。',
  },
  {
    title: '力與運動的對偶',
    md: `未達力封閉時，接觸 wrench 錐 $\\mathrm{pos}\\{\\vF_i\\}$ 不是整個空間，必定存在非零 twist $\\vV$ 使

$$\\vF_i^{\\T}\\vV \\ge 0 \\quad \\text{對每條錐邊都成立。}$$

這就是接觸**擋不住**的運動：每個接觸點都朝遠離手指的方向離開，且和每條錐邊的夾角都不超過 90°，錐內任何接觸力對它都只能做非負的功。把上一課的 CoR 圖改用錐的邊緣線來畫（每個接觸兩條）：標 $+$ 的點，所有接觸力對它的力矩都 $\\ge 0$，對它逆時針的擾動就平衡不了，物體會繞它逆時針轉走；$-$ 同理。力封閉 ⇔ 圖上沒有任何標記。`,
    simHint: '兩指錯開、μ = 0.25：藍區是方塊能逆時針轉走的轉心，橘色虛線是一個擋不住的擾動。把 μ 慢慢調大，看藍區縮小、在 μ ≈ 0.6 消失；按「▶ 試動」看自由運動。',
  },
  {
    title: '回顧與下一章',
    md: `兩種「封閉」並排比較：

| | 形封閉 | 力封閉 |
|---|---|---|
| 靠什麼 | 幾何（無摩擦接觸） | 摩擦＋手指主動擠壓 |
| 平面最少接觸 | 4 | 2（$\\mu > 0$） |
| 空間最少點接觸 | 7 | 3 |
| 判據 | 法線 wrench 正張成 | 錐邊緣 wrench 正張成 |

兩者都只回答「存在與否」；抓不抓得住，還要看外力大小和手指出力上限。下一章離開手臂，談**輪型移動機器人**：例如全向輪（omniwheel）底盤，同樣用平面 twist $(\\omega, v_x, v_y)$ 描述它的運動。`,
    simHint: '自由切換情境、拖手指與 μ，用連線判據、3D 凸包和對偶圖互相驗證；準備好就往下做自我檢測。',
  },
];

const quiz: QuizQuestion[] = [
  {
    question: '兩指夾一個圓盤，$\\mu = 0.2$。兩個接觸點在圓周上相隔的圓心角 $\\Delta\\varphi$ 至少要多大，才有力封閉？',
    options: ['大於 90° 就夠', '大於約 157°', '大於約 169°', '必須剛好 180°（正對面）'],
    answer: 1,
    explanation: '圓盤的法線都指向圓心，弦與兩端法線的夾角都是 $(180^\\circ - \\Delta\\varphi)/2$。力封閉要求它小於 $\\alpha = \\tan^{-1}0.2 \\approx 11.3^\\circ$，所以 $\\Delta\\varphi > 180^\\circ - 22.6^\\circ \\approx 157^\\circ$。正對面當然可以，但摩擦給了兩指一段容許的偏差。',
  },
  {
    question: '兩指夾住方塊的一對平行面，$\\mu = 0.3$。兩指連線相對於面的法線最多能傾斜幾度，仍保持力封閉？',
    options: ['約 0.3°', '約 30°', '約 73°', '約 16.7°'],
    answer: 3,
    explanation: '平行面的兩條法線方向相反，連線和兩條法線的夾角相同，判據要求它小於 $\\alpha = \\tan^{-1}0.3 \\approx 16.7^\\circ$。73.3° 是 $90^\\circ - \\alpha$；0.3° 與 30° 則是把 $\\mu$ 直接當成角度。',
  },
  {
    question: '某兩指抓取未達力封閉。用錐邊緣線畫的 CoR 圖上，只剩物體中心附近的一塊 $+$ 區。下列哪個擾動**一定**擋不住？',
    options: ['順時針的純力偶', '任何擾動都擋不住', '逆時針的純力偶', '手指夾得夠緊，就什麼都擋得住'],
    answer: 2,
    explanation: '$+$ 區裡的點 $c$ 代表所有接觸力對 $c$ 的力矩都 $\\ge 0$。逆時針純力偶對任何點的力矩都是正的，要平衡它，接觸力必須對 $c$ 產生負的力矩——做不到，物體會繞 $c$ 逆時針轉走。順時針力偶則可能被擋住；夾得更緊只是放大既有的力，改變不了力矩的符號限制。',
  },
  {
    question: '兩指水平夾住一塊重 6 N 的方塊（$\\mu = 0.5$），連線落在兩個錐內，已經力封閉。每根手指至少要出多少法向夾持力？',
    options: ['6 N', '3 N', '12 N', '0 N：力封閉就保證抓得住'],
    answer: 0,
    explanation: '兩指各用摩擦分擔一半重量 $f_t = 3$ N，而 $|f_t| \\le \\mu f_n$ 要求 $f_n \\ge 3 / 0.5 = 6$ N。力封閉只保證「存在」平衡的力，不保證手指出得起；若手指上限只有 5 N，方塊就會往下滑。',
  },
];

// ------------------------------------------------------------------ sim constants
type ShapeKey = 'square' | 'disc' | 'tri';

const TRI_VERTS: P2[] = [0, 1, 2].map((k) => {
  const a = Math.PI / 2 + (2 * Math.PI * k) / 3;
  return [1.1 * Math.cos(a), 1.1 * Math.sin(a)] as P2;
});
const BODIES: Record<ShapeKey, Body> = {
  square: makePoly('square', '方塊', [[-0.75, -0.75], [0.75, -0.75], [0.75, 0.75], [-0.75, 0.75]]),
  disc: makeDisc('disc', '圓盤', 0.85),
  tri: makePoly('tri', '三角形', TRI_VERTS),
};
const TRI_MIDS: P2[] = TRI_VERTS.map((a, k) => { const b = TRI_VERTS[(k + 1) % 3]; return [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2] as P2; });
const polar = (r: number, deg: number): P2 => [r * Math.cos((deg * Math.PI) / 180), r * Math.sin((deg * Math.PI) / 180)];

const FINGER_VARS = ['--series-6', '--series-7', '--series-3', '--series-8'];
const MAX_F = 4;
const CANVAS_ASPECT = 1.55;
const FTIP = 0.075; // fingertip radius (world units)
const CONE_LEN = 0.62; // drawn length of the friction cones
const K_PROBE = 0.6; // world units per unit of probe force
const K_F = 0.07; // world units per N (disturbance and contact forces)
const GRIP = 0.1; // arrow handles sit this far beyond the arrow tip so the arrowhead stays visible
const F_DIST_MAX = 15; // N
const BAR_MAX = 16; // N, full scale of the grip-force bars
const T_RAMP = 1.6, T_HOLD = 1.1, T_FREE = 1.3; // s

interface DistConfig { mag: number; ang: number; a: P2; fmax: number }
interface Config {
  shape: ShapeKey; points: P2[]; mu: number; probe?: boolean; dual?: boolean; hull?: boolean; dist?: DistConfig;
  /** Optional zoomed view (same aspect as the canvas); the default view otherwise. */
  world?: World;
}

const WORLD: World = { xmin: -2.1, xmax: 2.1, ymin: -1.36, ymax: 1.36 };
const PRESETS: { label: string; title: string; cfg: Config }[] = [
  { label: '單指摩擦錐', title: '一根手指與它的摩擦錐', cfg: { shape: 'square', points: [[0, -0.75]], mu: 0.5, probe: true, world: { xmin: -1.7, xmax: 1.7, ymin: -1.35, ymax: 0.85 } } },
  { label: '兩指夾方塊', title: '左右兩面各一指，連線略微傾斜', cfg: { shape: 'square', points: [[-0.75, -0.2], [0.75, 0.25]], mu: 0.5 } },
  { label: '兩指夾圓盤', title: '圓盤的法線都指向圓心', cfg: { shape: 'disc', points: [polar(0.85, 205), polar(0.85, 15)], mu: 0.3 } },
  { label: '三指三角形', title: '三指在三邊中點：法線交於中心', cfg: { shape: 'tri', points: TRI_MIDS, mu: 0.3, hull: true } },
  { label: '重量擾動', title: '水平夾持，承受 6 N 重量', cfg: { shape: 'square', points: [[-0.75, 0], [0.75, 0]], mu: 0.4, dist: { mag: 6, ang: -Math.PI / 2, a: [0, 0], fmax: 5 } } },
  { label: '對偶', title: '兩指錯開、摩擦小：看接觸擋不住的運動', cfg: { shape: 'square', points: [[-0.75, -0.45], [0.75, 0.45]], mu: 0.25, dual: true } },
];
const STEP_PRESET = [0, 1, 3, 4, 5, 2];

const STYLE = `
.ch12b-box { margin: 0.35rem 0; }
.ch12b-bars { display: grid; gap: 0.3rem; font-size: 0.8rem; margin: 0.35rem 0; }
.ch12b-bar { display: grid; grid-template-columns: 3.6rem 1fr 6.2rem; gap: 0.5rem; align-items: center; }
.ch12b-track { position: relative; height: 0.72rem; background: var(--bg-sunken); border-radius: 999px; }
.ch12b-fill { position: absolute; left: 0; top: 0; bottom: 0; border-radius: 999px; transition: width 60ms linear; }
.ch12b-lim { position: absolute; top: -4px; bottom: -4px; width: 2px; margin-left: -1px; background: var(--danger); }
.ch12b-val { font-family: var(--mono); font-variant-numeric: tabular-nums; text-align: right; }
.ch12b-val.over { color: var(--danger); font-weight: 600; }
.ch12b-cap { font-size: 0.78rem; color: var(--fg-muted); margin: 0.15rem 0; }
.ch12b-card { display: flex; gap: 0.8rem; align-items: center; border: 1px dashed var(--border); border-radius: var(--radius-sm); padding: 0.4rem 0.7rem; background: var(--bg-elev); margin: 0.4rem 0; font-size: 0.86rem; }
.ch12b-card p { margin: 0.25em 0; }
.ch12b-card svg { flex: none; }
.badge.ch12b-warn { background: var(--warn-soft); color: var(--warn); border-color: transparent; }
.badge.ch12b-bad { background: var(--danger-soft); color: var(--danger); border-color: transparent; }
`;

/** Static sketch of a three-omniwheel base (theme colours through CSS variables). */
const OMNI_SVG = (() => {
  const wheel = (deg: number) => `<g transform="rotate(${-deg})"><rect x="33" y="-14" width="11" height="28" rx="3" fill="var(--fg-muted)"/>`
    + [-9, -3, 3, 9].map((y) => `<line x1="33.5" y1="${y}" x2="43.5" y2="${y}" stroke="var(--bg-elev)" stroke-width="2"/>`).join('') + '</g>';
  return `<svg viewBox="-56 -56 112 112" width="104" height="104" aria-hidden="true">
<circle r="31" fill="var(--bg-sunken)" stroke="var(--fg-muted)" stroke-width="2"/>
${[90, 210, 330].map(wheel).join('')}
<line x1="0" y1="0" x2="19" y2="0" stroke="var(--axis-x)" stroke-width="2.5"/><polygon points="25,0 18,-4 18,4" fill="var(--axis-x)"/>
<line x1="0" y1="0" x2="0" y2="-19" stroke="var(--axis-y)" stroke-width="2.5"/><polygon points="0,-25 -4,-18 4,-18" fill="var(--axis-y)"/>
<path d="M 12 0 A 12 12 0 1 0 0 12" fill="none" stroke="var(--accent)" stroke-width="2"/><polygon points="5,12 -1,8 -1,16" fill="var(--accent)"/>
<circle r="2.5" fill="var(--fg)"/>
</svg>`;
})();

const deg = (rad: number): number => (rad * 180) / Math.PI;
const fmtV = (v: ArrayLike<number>, d = 2): string => `(${Array.from(v, (x) => fmt(x, d)).join(', ')})`;
const ease = (t: number): number => { const u = Math.min(1, Math.max(0, t)); return u * u * (3 - 2 * u); };

function describeTwist(V: V3): string {
  const c = corOf(V);
  if (!c) return `沿 ${fmt(deg(Math.atan2(V[2], V[1])), 0)}° 方向平移走`;
  return `繞 (${fmt(c[0], 2)}, ${fmt(c[1], 2)}) ${V[0] > 0 ? '逆時針' : '順時針'}轉走`;
}

/** Unsigned per-finger force bars with a common limit marker. */
function gripBars(parent: HTMLElement, n: number) {
  const rows = Array.from({ length: n }, (_, i) => {
    const fill = el('span', { class: 'ch12b-fill' });
    const lim = el('span', { class: 'ch12b-lim' });
    const val = el('span', { class: 'ch12b-val' }, '–');
    const r = el('div', { class: 'ch12b-bar' }, el('span', {}, `手指 ${i + 1}`), el('span', { class: 'ch12b-track' }, fill, lim), val);
    return { r, fill, lim, val };
  });
  const wrap = el('div', { class: 'ch12b-bars' }, rows.map((x) => x.r));
  parent.appendChild(wrap);
  return {
    el: wrap,
    /** `values[i]` = normal force of finger i (NaN = no solution). */
    set(values: number[], limit: number): void {
      rows.forEach((x, i) => {
        const on = i < values.length;
        x.r.style.display = on ? '' : 'none';
        if (!on) return;
        const v = values[i];
        const over = Number.isFinite(v) && v > limit + 1e-9;
        x.fill.style.width = `${Number.isFinite(v) ? Math.max(0, Math.min(1, v / BAR_MAX)) * 100 : 0}%`;
        x.fill.style.background = over ? 'var(--danger)' : `var(${FINGER_VARS[i]})`;
        x.lim.style.left = `${Math.min(100, (limit / BAR_MAX) * 100)}%`;
        x.val.classList.toggle('over', over);
        const t = Number.isFinite(v) ? `${fmt(v, 2)} N${over ? ' ⚠' : ''}` : '—';
        if (x.val.textContent !== t) x.val.textContent = t;
      });
    },
  };
}

// ------------------------------------------------------------------ simulation
function mount(host: HTMLElement, ctx: LessonContext): LessonHandle {
  // ---------------- state
  let shapeKey: ShapeKey = 'square';
  let body: Body = BODIES.square;
  let contacts: number[] = [0.75];
  let mu = 0.5;
  const probe = { on: false, fn: 1, ft: 0.35 }; // force of finger 1 on the body in its contact frame (n̂, t̂)
  const dist = { on: false, mag: 6, ang: -Math.PI / 2, a: [0, 0] as P2 };
  let fmax = 5;
  const show = { dual: false, hull: false };
  let anim: { kind: 'ramp' | 'free'; t: number } | null = null;
  let step = -1;
  let configuring = false;

  // ---------------- derived
  let cs: BoundaryPoint[] = [];
  let Fs: V3[] = [];
  let closed = false;
  let rank = 0;
  let cert: number[] | null = null;
  let margin = 0;
  let two: TwoFingerResult | null = null;
  let free: V3 | null = null;
  let Fext: V3 = [0, 0, 0];
  let hold: HoldResult | null = null;
  let driven: V3 | null = null;
  let rho = bodyRadius(body);
  let rhoG = gyrationRadius(body);

  // ---------------- layout
  const styleEl = el('style', {}, STYLE);
  host.appendChild(styleEl);
  const titleEl = simTitle(host, '平面抓取：摩擦錐與力封閉');
  const badge = el('span', { class: 'badge' }, '—');
  titleEl.appendChild(badge);
  const cv = new Canvas2D(host, { aspect: CANVAS_ASPECT, world: WORLD, overlay: true });
  let zoomed = false;
  const setView = (w?: World) => { zoomed = !!w; cv.setWorld(w ?? WORLD); };

  const panel = controlPanel(host);
  const chips = presetChips(row(panel), PRESETS.map((p) => ({ label: p.label, title: p.title, apply: () => configure(p.cfg) })), { label: '情境' });
  const muSl = slider(panel, {
    label: '摩擦係數 μ', min: 0, max: 1.2, step: 0.01, value: mu,
    format: (v) => `${fmt(v, 2)}（α = ${fmt(deg(frictionAngle(v)), 1)}°）`,
    onInput: (v) => { mu = v; if (!configuring) chips.clear(); update(); },
  });
  const shapeSel = select<ShapeKey>(panel, {
    label: '物體',
    options: (Object.keys(BODIES) as ShapeKey[]).map((k) => ({ value: k, label: BODIES[k].name })),
    value: shapeKey,
    onChange: (k) => { setShape(k); chips.clear(); },
  });
  const r2 = row(panel);
  const addBtn = button(r2, { label: '＋手指', small: true, onClick: () => { addFinger(); chips.clear(); } });
  const delBtn = button(r2, {
    label: '－手指', small: true,
    onClick: () => { if (contacts.length > 1) { contacts.pop(); rebuildFingerHandles(); chips.clear(); update(); } },
  });
  const mu0Btn = button(r2, { label: 'μ = 0（回到形封閉）', small: true, title: '摩擦錐縮成法線', onClick: () => muSl.set(0) });
  // one play button: the free motion normally, the slowly applied disturbance in the disturbance test
  const playBtn = button(r2, { label: '▶ 試動', small: true, onClick: () => (dist.on ? startRamp() : startFree()) });
  const dualTog = toggle(r2, { label: '對偶：自由運動的 CoR 圖', value: false, onChange: (v) => { show.dual = v; chips.clear(); update(); } });
  const hullTog = toggle(r2, { label: 'wrench 空間（3D）', value: false, onChange: (v) => { setHull(v); chips.clear(); } });
  const distTog = toggle(r2, { label: '擾動測試', value: false, onChange: (v) => { dist.on = v; anim = null; chips.clear(); update(); } });

  const info = kv(host, ['力封閉']);

  // step 1: the force of one finger against its cone
  const probeBox = el('div', { class: 'ch12b-box' });
  host.appendChild(probeBox);
  const probeTog = toggle(probeBox, { label: '手指 1 的力 f（拖曳橘色箭頭尖端）', value: false, onChange: (v) => { probe.on = v; chips.clear(); update(); } });
  const probeKv = kv(probeBox, ['f_n（法向）, f_t（切向）', '|f_t| ÷ f_n', '判定']);

  // step 2: two-finger criterion
  const twoBox = el('div', { class: 'ch12b-box' });
  host.appendChild(twoBox);
  const twoKv = kv(twoBox, ['連線與法線夾角 β₁, β₂', '兩指判據 βᵢ < α']);

  // step 3: wrench-space hull
  const hullBox = el('div', { class: 'ch12b-box', style: { display: 'none' } });
  host.appendChild(hullBox);
  const hullHost = el('div');
  hullBox.append(hullHost, el('p', { class: 'ch12b-cap' }, 'wrench 空間：每一點是一條錐邊的 (f_x, f_y, m/ρ)，細線是從原點出發的射線；拖曳可旋轉視角。'));
  const hullKv = kv(hullBox, ['錐邊 wrench', 'LP 證書（kᵢ ≥ 1，Σkᵢ𝓕ᵢ = 0）', '原點到凸包邊界']);
  let hullView: WrenchHullView | null = null;

  // step 4: disturbance and finger force limit
  const distBox = el('div', { class: 'ch12b-box' });
  host.appendChild(distBox);
  const distGrid = controlPanel(distBox, { compact: true });
  const magSl = slider(distGrid, {
    label: '擾動大小 |f_ext|', min: 0, max: F_DIST_MAX, step: 0.1, value: dist.mag, unit: 'N',
    onInput: (v) => { dist.mag = v; anim = null; chips.clear(); update(); },
  });
  const angSl = angleSlider(distGrid, { label: '擾動方向', value: dist.ang, onInput: (v) => { dist.ang = v; anim = null; chips.clear(); update(); } });
  const fmaxSl = slider(distGrid, {
    label: '每指夾持力上限 f_max', min: 1, max: 15, step: 0.1, value: fmax, unit: 'N',
    onInput: (v) => { fmax = v; anim = null; chips.clear(); update(); },
  });
  const bars = gripBars(distBox, MAX_F);
  const distKv = kv(distBox, ['擾動 𝓕_ext = (m, fₓ, f_y)', '所需最大夾持力 f*', '結論']);

  // step 5: duality
  const dualBox = el('div', { class: 'ch12b-box', style: { display: 'none' } });
  host.appendChild(dualBox);
  const dualKv = kv(dualBox, ['自由運動（例）', '擋不住的擾動（例）']);

  // step 6: summary / preview card
  const summary = el('div', { class: 'ch12b-card', style: { display: 'none' } },
    el('div', { html: OMNI_SVG }),
    mdElement('**下一章預告：輪型移動機器人。** 全向輪（omniwheel）的輪緣裝著可自由轉動的小滾子；三個輪子各自轉動，就能讓底盤產生任意平面 twist $(\\omega, v_x, v_y)$，不必先轉向再前進。'));
  host.appendChild(summary);

  legend(host, [
    { color: 'var(--series-6)', label: '手指 1 與摩擦錐' },
    { color: 'var(--series-7)', label: '手指 2' },
    { color: 'var(--ok)', label: '連線在兩錐內' },
    { color: 'var(--danger)', label: '出錐／擋不住' },
    { color: 'var(--warn)', label: '擾動 𝓕_ext' },
    { color: 'var(--series-1)', label: 'CoR +：可逆時針轉走' },
    { color: 'var(--series-5)', label: 'CoR −：可順時針轉走' },
  ]);
  note(host, '一階（準靜態）分析：力封閉看錐邊緣 wrench 是否正張成 R³；夾持力用 LP 求「最大法向力最小」的分配。滑脫的虛影只示意方向，不是動力學模擬。');

  // ---------------- handles
  let fingerHandles: Handle[] = [];
  const fingerHandlePos = (i: number): P2 => {
    const b = cs[i] ?? boundaryAt(body, contacts[i]);
    return sub2(b.p, scale2(b.n, 0.24));
  };
  function rebuildFingerHandles(): void {
    fingerHandles.forEach((h) => cv.removeHandle(h));
    fingerHandles = contacts.map((_, i) => cv.addHandle({
      get: () => fingerHandlePos(i),
      set: (q) => { contacts[i] = projectToBoundary(body, q).s; anim = null; chips.clear(); update(); },
      radius: 9,
      label: String(i + 1),
      color: FINGER_VARS[i],
    }));
  }

  const probeForce = (): P2 => (cs.length ? add2(scale2(cs[0].n, probe.fn), scale2(perp(cs[0].n), probe.ft)) : [0, 0]);
  const probeHandle = cv.addHandle({
    get: () => {
      if (!cs.length) return [0, 0];
      const f = probeForce();
      return add2(cs[0].p, add2(scale2(f, K_PROBE), scale2(unit2(f), GRIP)));
    },
    set: (q) => {
      if (!cs.length) return;
      const b = cs[0];
      const d = sub2(q, b.p);
      let f = scale2(unit2(d), Math.max(0.05, norm2(d) - GRIP) / K_PROBE);
      const L = norm2(f);
      if (L > 2) f = scale2(f, 2 / L);
      probe.fn = dot2(f, b.n); probe.ft = dot2(f, perp(b.n));
      chips.clear(); update();
    },
    radius: 8,
    color: '--series-4',
  });
  const fext = (): P2 => [dist.mag * Math.cos(dist.ang), dist.mag * Math.sin(dist.ang)];
  const distAHandle = cv.addHandle({
    get: () => dist.a,
    set: (q) => { if (insideBody(body, q)) { dist.a = q; anim = null; chips.clear(); update(); } },
    radius: 6,
    color: '--warn',
  });
  const distFHandle = cv.addHandle({
    get: () => add2(dist.a, scale2([Math.cos(dist.ang), Math.sin(dist.ang)], dist.mag * K_F + GRIP)),
    set: (q) => {
      const d = sub2(q, dist.a);
      const f = scale2(unit2(d), Math.max(0, norm2(d) - GRIP) / K_F);
      dist.mag = Math.min(F_DIST_MAX, norm2(f));
      if (norm2(d) > 1e-9) dist.ang = Math.atan2(d[1], d[0]);
      magSl.set(dist.mag, { silent: true });
      angSl.set(dist.ang, { silent: true });
      anim = null; chips.clear(); update();
    },
    radius: 8,
    color: '--warn',
  });

  // ---------------- model
  function recompute(): void {
    cs = contacts.map((s) => boundaryAt(body, s));
    Fs = edgeWrenches(cs, mu);
    closed = isForceClosure(Fs);
    rank = wrenchRank(Fs);
    cert = closed ? closureCertificate(Fs) : null;
    margin = originMargin(scaleMoments(Fs, rho));
    two = cs.length === 2 ? twoFingerTest(cs[0], cs[1], mu) : null;
    free = closed ? null : preferredFreeTwist(Fs);
    Fext = forceWrench(dist.a, fext());
    hold = dist.on ? minGripForces(cs, mu, Fext) : null;
    driven = hold && !hold.feasible ? drivenFreeTwist(Fs, Fext, rho) : null;
  }

  /** Fraction of the disturbance applied (the ramp animation scales it from 0 to 1). */
  const lambda = (): number => (anim?.kind === 'ramp' ? Math.min(1, anim.t / T_RAMP) : 1);

  /** Ghost displacement when the fingers cannot supply the needed force (illustration only). */
  function slipDistance(): number {
    if (!hold || !hold.feasible || hold.fStar <= fmax + 1e-9) return 0;
    if (anim?.kind !== 'ramp') return 0.3;
    const ts = (T_RAMP * fmax) / hold.fStar; // moment the required force reaches the limit
    const dt = anim.t - ts;
    return dt <= 0 ? 0 : Math.min(0.3, 1.4 * dt * dt);
  }

  function refreshBadge(): void {
    let cls = 'badge';
    let txt: string;
    if (dist.on && hold) {
      const lam = lambda();
      if (!hold.feasible) { cls = 'badge ch12b-bad'; txt = '擋不住這個擾動'; }
      else if (hold.fStar * lam > fmax + 1e-9) { cls = 'badge ch12b-warn'; txt = closed ? '力封閉，但力不夠 → 滑脫' : '力不夠 → 滑脫'; }
      else { cls = 'badge ok'; txt = lam < 1 ? '加載中…目前抓得住' : '抓得住 ✓'; }
    } else if (closed) { cls = 'badge ok'; txt = '力封閉 ✓'; }
    else txt = `未力封閉 · ${contacts.length} 指`;
    if (badge.className !== cls) badge.className = cls;
    if (badge.textContent !== txt) badge.textContent = txt;
  }

  function refreshDistDom(): void {
    refreshBadge();
    const lam = lambda();
    if (!dist.on || !hold) {
      bars.set(cs.map(() => 0), fmax);
      distKv.set('擾動 𝓕_ext = (m, fₓ, f_y)', '（擾動測試關閉：勾選「擾動測試」）');
      distKv.set('所需最大夾持力 f*', '–');
      distKv.set('結論', '–');
      return;
    }
    distKv.set('擾動 𝓕_ext = (m, fₓ, f_y)', `${fmtV([Fext[0] * lam, Fext[1] * lam, Fext[2] * lam])}  (N·m, N, N)`);
    if (!hold.feasible) {
      bars.set(cs.map(() => NaN), fmax);
      distKv.set('所需最大夾持力 f*', '∞：錐內的力無論多大都平衡不了');
      distKv.set('結論', driven ? `✗ 物體會${describeTwist(driven)}（未力封閉）` : '✗ 擋不住');
      return;
    }
    bars.set(hold.fn.map((x) => x * lam), fmax);
    const need = hold.fStar * lam;
    distKv.set('所需最大夾持力 f*', `${fmt(need, 2)} N  ${need > fmax + 1e-9 ? '>' : '≤'}  f_max = ${fmt(fmax, 1)} N`);
    distKv.set('結論', need > fmax + 1e-9
      ? (closed ? '✗ 力封閉成立，但手指出力不夠：會滑脫' : '✗ 手指出力不夠：會滑脫')
      : '✓ 錐內的接觸力足以平衡擾動');
  }

  function refreshDom(): void {
    refreshBadge();
    addBtn.disabled = contacts.length >= MAX_F;
    delBtn.disabled = contacts.length <= 1;
    const playLabel = dist.on ? '▶ 施加擾動' : '▶ 試動';
    if (playBtn.textContent !== playLabel) playBtn.textContent = playLabel;
    playBtn.title = dist.on ? '擾動從 0 慢慢加到設定值' : '播放一個接觸擋不住的自由運動';
    playBtn.className = dist.on ? 'btn primary small' : 'btn small';
    playBtn.disabled = !dist.on && closed;

    if (closed) {
      info.set('力封閉', two?.ok ? '✓ 兩指連線落在兩個錐內' : cert ? '✓ 錐邊緣 wrench 正張成 R³（找得到 LP 證書）' : '✓ 錐邊緣 wrench 正張成 R³');
    } else {
      info.set('力封閉', free ? `✗ 仍可${describeTwist(free)}（例）` : '✗');
    }

    // probe (step 1)
    probeBox.style.display = step === 0 || probe.on ? '' : 'none';
    if (cs.length) {
      const chk = coneCheck(cs[0].n, mu, probeForce());
      probeKv.set('f_n（法向）, f_t（切向）', probe.on ? `${fmt(chk.fn, 2)}, ${fmt(chk.ft, 2)}` : '（勾選上方開關）');
      probeKv.set('|f_t| ÷ f_n', !probe.on ? '–' : chk.fn > 1e-9 ? `${fmt(Math.abs(chk.ft) / chk.fn, 2)} ${chk.inside ? '≤' : '>'} μ = ${fmt(mu, 2)}` : '—（f_n ≤ 0）');
      probeKv.set('判定', !probe.on ? '–' : chk.inside ? '✓ 在錐內：黏住，不滑' : chk.fn <= 1e-9 ? '✗ 法向力 ≤ 0：手指只能推、不能拉' : '✗ 在錐外：摩擦不夠，手指會沿表面滑');
    }

    // two-finger criterion
    twoBox.style.display = two && !dist.on ? '' : 'none';
    if (two) {
      twoKv.set('連線與法線夾角 β₁, β₂', `${fmt(deg(two.beta[0]), 1)}°, ${fmt(deg(two.beta[1]), 1)}°（α = ${fmt(deg(two.alpha), 1)}°）`);
      twoKv.set('兩指判據 βᵢ < α', two.ok ? '✓ 兩個夾角都小於 α：力封閉' : mu <= 0 ? '✗ μ = 0：錐縮成法線，兩指不可能力封閉' : `✗ ${two.beta[0] >= two.alpha ? 'β₁' : 'β₂'} ≥ α：連線跑出錐外`);
    }

    // wrench-space hull
    hullBox.style.display = show.hull ? '' : 'none';
    if (show.hull) {
      hullKv.set('錐邊 wrench', `${Fs.length} 個 · rank = ${rank}`);
      hullKv.set('LP 證書（kᵢ ≥ 1，Σkᵢ𝓕ᵢ = 0）', cert ? '找得到 ✓' : rank < 3 ? `找不到 ✗（rank ${rank} < 3）` : '找不到 ✗');
      hullKv.set('原點到凸包邊界', margin > 1e-9 ? `${fmt(margin, 3)}（原點嚴格在內；越大越穩）` : '≤ 0：原點不在凸包內部');
      if (hullView) {
        const pts = scaleMoments(Fs, rho).map((F) => [F[1], F[2], F[0]] as V3);
        hullView.update(pts, cs.flatMap((_, i) => [FINGER_VARS[i], FINGER_VARS[i]]), closed);
      }
    }

    // disturbance
    distBox.style.display = dist.on || step === 3 ? '' : 'none';
    refreshDistDom();

    // duality
    dualBox.style.display = show.dual ? '' : 'none';
    if (show.dual) {
      dualKv.set('自由運動（例）', free ? describeTwist(free) : '無：力封閉，沒有擋不住的運動');
      if (free) {
        const pa = pushAlong(body, free);
        dualKv.set('擋不住的擾動（例）', `在 (${fmt(pa.a[0], 2)}, ${fmt(pa.a[1], 2)}) 沿 ${fmt(deg(Math.atan2(pa.f[1], pa.f[0])), 0)}° 推`);
      } else dualKv.set('擋不住的擾動（例）', '無');
    }

    summary.style.display = step === 5 ? '' : 'none';

    // handles
    probeHandle.enabled = probe.on && cs.length > 0;
    distAHandle.enabled = dist.on;
    distFHandle.enabled = dist.on;
  }

  function update(): void {
    recompute();
    refreshDom();
    cv.requestRender();
  }

  function setShape(k: ShapeKey): void {
    shapeKey = k;
    body = BODIES[k];
    rho = bodyRadius(body);
    rhoG = gyrationRadius(body);
    shapeSel.set(k, { silent: true });
    const P = perimeterOf(body);
    contacts = contacts.map((_, i) => (P * (i + 0.25)) / contacts.length);
    if (!insideBody(body, dist.a)) dist.a = [0, 0];
    anim = null;
    if (zoomed) setView();
    rebuildFingerHandles();
    update();
  }

  function addFinger(): void {
    if (contacts.length >= MAX_F) return;
    if (zoomed) setView(); // the zoomed single-finger view would cut off fingers elsewhere
    const P = perimeterOf(body);
    let bestS = 0, bestD = -1;
    for (let k = 0; k < 96; k++) {
      const b = boundaryAt(body, (P * k) / 96);
      const d = cs.length ? Math.min(...cs.map((c) => norm2(sub2(c.p, b.p)))) : 1;
      if (d > bestD) { bestD = d; bestS = b.s; }
    }
    contacts.push(bestS);
    rebuildFingerHandles();
    update();
  }

  function setHull(v: boolean): void {
    const changed = v !== show.hull;
    show.hull = v;
    hullTog.set(v, { silent: true });
    hullBox.style.display = v ? '' : 'none';
    // a shorter 2D view leaves room for the 3D plot in the first screen
    if (changed) cv.setSize({ aspect: v ? 2.3 : CANVAS_ASPECT });
    if (v && !hullView) hullView = new WrenchHullView(hullHost, { aspect: host.clientWidth < 520 ? 1.6 : 2.5 });
    update();
  }

  function configure(c: Config): void {
    configuring = true;
    anim = null;
    loop.stop();
    shapeKey = c.shape;
    body = BODIES[c.shape];
    rho = bodyRadius(body);
    rhoG = gyrationRadius(body);
    shapeSel.set(c.shape, { silent: true });
    contacts = c.points.map((q) => projectToBoundary(body, q).s);
    mu = c.mu;
    muSl.set(c.mu, { silent: true });
    probe.on = !!c.probe; probeTog.set(probe.on, { silent: true });
    if (c.probe) { probe.fn = 1; probe.ft = 0.35; }
    show.dual = !!c.dual; dualTog.set(show.dual, { silent: true });
    dist.on = !!c.dist; distTog.set(dist.on, { silent: true });
    if (c.dist) {
      dist.mag = c.dist.mag; dist.ang = c.dist.ang; dist.a = [c.dist.a[0], c.dist.a[1]]; fmax = c.dist.fmax;
      magSl.set(dist.mag, { silent: true }); angSl.set(dist.ang, { silent: true }); fmaxSl.set(fmax, { silent: true });
    }
    if (!insideBody(body, dist.a)) dist.a = [0, 0];
    setView(c.world);
    rebuildFingerHandles();
    setHull(!!c.hull); // also updates everything
    configuring = false;
  }

  // ---------------- animations
  const loop = createLoop((dt) => {
    if (!anim) { loop.stop(); return; }
    anim.t += dt;
    const T = anim.kind === 'ramp' ? T_RAMP + T_HOLD : T_FREE;
    if (anim.t >= T) { anim = null; loop.stop(); }
    if (dist.on) refreshDistDom();
    cv.requestRender();
  });
  function startRamp(): void {
    if (!dist.on) return;
    anim = { kind: 'ramp', t: 0 };
    loop.start();
  }
  function startFree(): void {
    if (!free || dist.on) return;
    anim = { kind: 'free', t: 0 };
    loop.start();
  }

  // ---------------- drawing
  function drawCone(c: Canvas2D, b: BoundaryPoint, col: string): void {
    if (mu < 1e-6) {
      c.line(b.p, add2(b.p, scale2(b.n, CONE_LEN)), { color: col, width: 3 });
      return;
    }
    const [ep, em] = unitConeEdges(b.n, mu);
    const a0 = Math.atan2(em[1], em[0]);
    const span = 2 * frictionAngle(mu);
    const pts: P2[] = [b.p];
    for (let k = 0; k <= 18; k++) {
      const a = a0 + (span * k) / 18;
      pts.push([b.p[0] + CONE_LEN * Math.cos(a), b.p[1] + CONE_LEN * Math.sin(a)]);
    }
    c.polygon(pts, { fill: col, alpha: 0.2, width: 0 });
    c.line(b.p, add2(b.p, scale2(ep, CONE_LEN)), { color: col, width: 1.6 });
    c.line(b.p, add2(b.p, scale2(em, CONE_LEN)), { color: col, width: 1.6 });
    c.line(b.p, add2(b.p, scale2(b.n, CONE_LEN * 0.85)), { color: col, width: 1, dash: [3, 3], alpha: 0.85 });
  }

  function drawFinger(c: Canvas2D, b: BoundaryPoint, col: string): void {
    c.line(sub2(b.p, scale2(b.n, FTIP * 1.5)), sub2(b.p, scale2(b.n, 0.46)), { color: col, width: 11, cap: 'round', alpha: 0.4 });
    c.disk(sub2(b.p, scale2(b.n, FTIP)), FTIP, { fill: col, alpha: 0.9 });
  }

  /** Where the β label of the angle between normal n and direction d at p goes (on the bisector). */
  const betaLabelPos = (p: P2, n: P2, d: P2): P2 => {
    const mid = Math.atan2(n[1], n[0]) + Math.atan2(cross2(n, d), dot2(n, d)) / 2;
    return [p[0] + 0.5 * Math.cos(mid), p[1] + 0.5 * Math.sin(mid)];
  };

  function angleMark(c: Canvas2D, p: P2, n: P2, d: P2, beta: number, col: string, label: string, font: string, bg: string): void {
    const an = Math.atan2(n[1], n[0]);
    const diff = Math.atan2(cross2(n, d), dot2(n, d));
    c.arc(p, 0.3, Math.min(an, an + diff), Math.max(an, an + diff), { color: col, width: 2 });
    c.text(betaLabelPos(p, n, d), `${label} ${fmt(deg(beta), 0)}°`, { color: col, bg, font: `600 11px ${font}` });
  }

  function drawTwo(c: Canvas2D, font: string, bg: string): void {
    if (!two) return;
    const [b1, b2] = cs;
    const col = two.ok ? '--ok' : '--danger';
    c.line(b1.p, b2.p, { color: col, width: 2.2, dash: two.ok ? undefined : [7, 5] });
    const d = unit2(sub2(b2.p, b1.p));
    angleMark(c, b1.p, b1.n, d, two.beta[0], col, 'β₁', font, bg);
    angleMark(c, b2.p, b2.n, scale2(d, -1), two.beta[1], col, 'β₂', font, bg);
    if (two.ok) {
      // the squeeze pair (internal force) along the line
      c.arrow(b1.p, add2(b1.p, scale2(d, 0.3)), { color: '--ok', width: 2.6, head: 9 });
      c.arrow(b2.p, add2(b2.p, scale2(d, -0.3)), { color: '--ok', width: 2.6, head: 9 });
    }
  }

  function drawProbe(c: Canvas2D, font: string, bg: string, muted: string): void {
    const b = cs[0];
    const f = probeForce();
    const chk = coneCheck(b.n, mu, f);
    const col = chk.inside ? '--series-4' : '--danger';
    const tip = add2(b.p, scale2(f, K_PROBE));
    const mid = add2(b.p, scale2(b.n, probe.fn * K_PROBE));
    c.line(b.p, mid, { color: muted, width: 1.2, dash: [4, 3] });
    c.line(mid, tip, { color: muted, width: 1.2, dash: [4, 3] });
    c.text(add2(mid, scale2(perp(b.n), probe.ft >= 0 ? -0.13 : 0.13)), 'f_n', { color: muted, font: `11px ${font}` });
    c.text(add2(scale2(add2(mid, tip), 0.5), scale2(b.n, 0.1)), 'f_t', { color: muted, font: `11px ${font}` });
    c.arrow(b.p, tip, { color: col, width: 3, head: 11 });
    // label beyond the drag handle (which sits GRIP past the tip)
    const lp = add2(tip, scale2(unit2(f), GRIP + 0.24));
    c.text(lp, chk.inside ? 'f：黏住' : chk.fn <= 1e-9 ? 'f：不能拉' : 'f：會滑', { color: col, bg, font: `600 12px ${font}` });
  }

  function drawGhost(c: Canvas2D, out: P2[], V: V3, dist0: number, col: string, label: string, font: string, bg: string): void {
    if (dist0 <= 1e-4) return;
    const pose = planarExp(V, displacementScale(V, out, dist0));
    const g = out.map((q) => applyPose(pose, q));
    c.polygon(g, { fill: col, alpha: 0.1, width: 0 });
    c.polygon(g, { color: col, width: 1.8, dash: [6, 4], alpha: 0.9 });
    if (label) {
      // just outside the ghost, on the side it moves towards (below it for a pure spin)
      const gc = applyPose(pose, [0, 0]);
      const m = norm2(gc) > 1e-3 ? unit2(gc) : ([0, -1] as P2);
      const ext = Math.max(...g.map((v) => dot2(sub2(v, gc), m)));
      c.text(add2(gc, scale2(m, ext + 0.14)), label, { color: col, bg, font: `600 12px ${font}` });
    }
  }

  function rotGlyph(c: Canvas2D, ctr: P2, r: number, sense: 1 | -1, col: string): void {
    const a0 = 0.4, a1 = 0.4 + 1.55 * Math.PI;
    c.arc(ctr, r, a0, a1, { color: col, width: 2 });
    const aEnd = sense > 0 ? a1 : a0;
    const tip: P2 = [ctr[0] + r * Math.cos(aEnd), ctr[1] + r * Math.sin(aEnd)];
    const tan: P2 = sense > 0 ? [-Math.sin(aEnd), Math.cos(aEnd)] : [Math.sin(aEnd), -Math.cos(aEnd)];
    c.arrow(sub2(tip, scale2(tan, 0.07)), tip, { color: col, width: 2, head: 9 });
    c.point(ctr, 3.5, { color: col });
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

  /** Labels of the dual map, drawn after the body and the fingers so nothing covers them. */
  let dualLabels: { p: P2; text: string; color: string }[] = [];

  function drawDual(c: Canvas2D, vis: World, font: string, bg: string, muted: string): void {
    dualLabels = [];
    const pad = 0.3;
    const box: P2[] = [[vis.xmin - pad, vis.ymin - pad], [vis.xmax + pad, vis.ymin - pad], [vis.xmax + pad, vis.ymax + pad], [vis.xmin - pad, vis.ymax + pad]];
    const lines = edgeLines(cs, mu);
    const Rp = corRegion(lines, 1, box), Rm = corRegion(lines, -1, box);
    const aP = polygonAreaAbs(Rp), aM = polygonAreaAbs(Rm);
    if (aP > 1e-4) { c.polygon(Rp, { fill: '--series-1', alpha: 0.14, width: 0 }); c.polygon(Rp, { color: '--series-1', alpha: 0.55, width: 1.2 }); }
    if (aM > 1e-4) { c.polygon(Rm, { fill: '--series-5', alpha: 0.14, width: 0 }); c.polygon(Rm, { color: '--series-5', alpha: 0.55, width: 1.2 }); }
    for (const L of lines) c.line(add2(L.p, scale2(L.d, -12)), add2(L.p, scale2(L.d, 12)), { color: muted, width: 1, dash: [5, 5], alpha: 0.6 });
    // free translations (CoR at infinity)
    const R = 0.25;
    const ctr: P2 = [vis.xmax - 0.42, vis.ymax - 0.4];
    c.circle(ctr, R + 0.06, { fill: bg, color: '--border', width: 1, alpha: 0.95 });
    c.circle(ctr, R, { color: '--grid-strong', width: 1.2 });
    const tc = translationCone(Fs.map((F) => [F[1], F[2]] as P2));
    if (tc.kind === 'all') c.circle(ctr, R, { color: '--ok', width: 4 });
    else if (tc.kind === 'arcs') {
      for (const [a0, a1] of tc.arcs) {
        const pts: P2[] = [ctr];
        for (let k = 0; k <= 24; k++) { const a = a0 + ((a1 - a0) * k) / 24; pts.push([ctr[0] + R * Math.cos(a), ctr[1] + R * Math.sin(a)]); }
        c.polygon(pts, { fill: '--ok', alpha: 0.3, width: 0 });
        c.arc(ctr, R, a0, a1, { color: '--ok', width: 4 });
      }
      for (const a of tc.rays) c.arrow(ctr, [ctr[0] + R * Math.cos(a), ctr[1] + R * Math.sin(a)], { color: '--ok', width: 2.5, head: 7 });
    } else c.text(ctr, '✗', { color: '--danger', font: `bold 15px ${font}` });
    c.text([ctr[0], ctr[1] - R - 0.15], '自由平移', { color: muted, font: `11px ${font}` });
    const fc = free ? corOf(free) : null;
    const avoid: P2[] = [ctr, ...cs.map((_, i) => fingerHandlePos(i)), ...(fc ? [fc] : [])];
    if (two && !probe.on && cs.length === 2) {
      const d = unit2(sub2(cs[1].p, cs[0].p));
      avoid.push(betaLabelPos(cs[0].p, cs[0].n, d), betaLabelPos(cs[1].p, cs[1].n, scale2(d, -1)));
    }
    if (free) { const pa = pushAlong(body, free); avoid.push(add2(pa.a, scale2(pa.f, 0.92))); }
    if (aP > 1e-4) dualLabels.push({ p: regionLabelPoint(Rp, vis, avoid), text: '+ 區：可逆時針轉走', color: '--series-1' });
    if (aM > 1e-4) dualLabels.push({ p: regionLabelPoint(Rm, vis, avoid), text: '− 區：可順時針轉走', color: '--series-5' });
    if (aP <= 1e-4 && aM <= 1e-4 && tc.kind === 'none') {
      dualLabels.push({ p: [vis.xmin + 1.05, vis.ymax - 0.25], text: '沒有任何標記：力封閉', color: '--ok' });
    }
  }

  function drawFree(c: Canvas2D, out: P2[], font: string, bg: string): void {
    if (!free) return;
    const ph = anim?.kind === 'free' ? ease(anim.t / T_FREE) : 1;
    const cor = corOf(free);
    const col = !cor ? '--accent' : free[0] > 0 ? '--series-1' : '--series-5';
    drawGhost(c, out, free, 0.24 * ph, col, '', font, bg);
    if (cor) rotGlyph(c, cor, 0.16, free[0] > 0 ? 1 : -1, col);
    const pa = pushAlong(body, free);
    const tip = add2(pa.a, scale2(pa.f, 0.6));
    c.arrow(pa.a, tip, { color: '--warn', width: 2.8, head: 11, dash: [6, 4] });
    c.text(add2(tip, scale2(pa.f, 0.32)), '擋不住的擾動（例）', { color: '--warn', bg, font: `600 12px ${font}` });
  }

  function drawDisturbance(c: Canvas2D, out: P2[], font: string, bg: string): void {
    const lam = lambda();
    const f = scale2(fext(), lam);
    if (hold && hold.feasible) {
      const h = hold;
      cs.forEach((b, i) => {
        const fi = scale2(h.f[i], lam);
        if (norm2(fi) < 1e-6) return;
        c.arrow(b.p, add2(b.p, scale2(fi, K_F)), { color: FINGER_VARS[i], width: 3, head: 10 });
      });
      const d = slipDistance();
      if (d > 0) drawGhost(c, out, accelTwist(Fext, rhoG), d, '--danger', '滑脫（示意）', font, bg);
    } else if (hold && driven) {
      drawGhost(c, out, driven, 0.3 * lam, '--danger', '被帶走', font, bg);
    }
    if (norm2(f) > 1e-9) {
      const tip = add2(dist.a, scale2(f, K_F));
      c.arrow(dist.a, tip, { color: '--warn', width: 3.2, head: 12 });
      c.text(tip, '𝓕_ext', { dx: 18, dy: -8, color: '--warn', font: `600 12px ${font}` });
    }
  }

  cv.draw = (c) => {
    const font = cssVar('--font') || 'sans-serif';
    const fg = cssVar('--fg'), muted = cssVar('--fg-muted'), bg = cssVar('--bg-elev');
    // while the disturbance ramps up, its handles (which mark the final value) are hidden
    const ramping = anim?.kind === 'ramp';
    distAHandle.enabled = dist.on && !ramping;
    distFHandle.enabled = dist.on && !ramping;
    c.grid(0.5);
    const vis = c.visible();
    if (show.dual && !dist.on) drawDual(c, vis, font, bg, muted);
    const out = outline(body);
    c.polygon(out, { fill: cssVar('--bg-sunken'), color: fg, width: 2 });
    c.point([0, 0], 3, { color: muted });
    cs.forEach((b, i) => drawCone(c, b, FINGER_VARS[i]));
    if (two && !dist.on && !probe.on) drawTwo(c, font, bg);
    cs.forEach((b, i) => drawFinger(c, b, FINGER_VARS[i]));
    if (probe.on && cs.length) drawProbe(c, font, bg, muted);
    if (dist.on) drawDisturbance(c, out, font, bg);
    else if ((show.dual || anim?.kind === 'free') && free) drawFree(c, out, font, bg);
    if (show.dual && !dist.on) for (const L of dualLabels) c.text(L.p, L.text, { color: L.color, bg, font: `600 12px ${font}` });
    const a = deg(frictionAngle(mu));
    // short on purpose: the top-right corner holds the translation compass (β is labelled on the arcs)
    c.setOverlayText(`μ = ${fmt(mu, 2)} · α = ${fmt(a, 1)}°`);
  };

  // ---------------- steps
  const focusables: HTMLElement[][] = [
    [cv.el, probeBox], [cv.el, twoBox], [hullBox, mu0Btn], [distBox, playBtn], [cv.el, dualBox], [summary],
  ];
  let focused: HTMLElement[] = [];
  const onStep = (i: number): void => {
    const changed = i !== step;
    step = i;
    if (changed) chips.activate(STEP_PRESET[i] ?? 0);
    else update();
    focused.forEach((f) => f.classList.remove('mr-focus'));
    focused = focusables[i] ?? [];
    focused.forEach((f) => f.classList.add('mr-focus'));
  };
  rebuildFingerHandles();
  onStep(ctx.currentStep);

  return {
    onStep,
    destroy() {
      loop.stop();
      anim = null;
      cv.destroy();
      hullView?.destroy();
      hullView = null;
      styleEl.remove();
    },
  };
}

const lesson: Lesson = { meta, steps, quiz, mount };
export default lesson;
