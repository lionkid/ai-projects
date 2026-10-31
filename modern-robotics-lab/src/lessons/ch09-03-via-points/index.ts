import type { Lesson, LessonContext, LessonHandle, LessonStep, QuizQuestion } from '@/types';
import {
  el, cssVar, fmt, controlPanel, row, select, toggle, button, matrixView, playback, simTitle, legend,
  presetChips, split, mdElement, Canvas2D, Plot2D, plotGroup, createLoop, drawPlanarArm,
  type Handle, type Marker, type Pt, type SeriesSpec,
} from '@/ui';
import type { ViaPointSpline } from '@/mr';
import { meta } from './meta';
import {
  asTrajectory, buildSpline, c2SplineVelocities, centredVelocities, clampViaTime, findOvershoots, globalPolynomial,
  insertVia, quinticFromCubic, removeVia, accelJumps, segmentSpeeds, trackIK,
  type Overshoot, type Pt2, type Trajectory2, type VelStrategy,
} from './viaSpline';
import { SCENARIOS, SCENARIO_IDS, type ScenarioId } from './scenarios';

const steps: LessonStep[] = [
  {
    title: '不只兩個點',
    md: `前兩課的軌跡都是「從 A 到 B」。真實任務卻常要求依序**經過好幾個點**：繞過障礙物的中途點、相機要停下來拍照的位置……每個**經過點（via point）** $p_k$ 都附帶一個到達時刻 $t_k$。

最直接的想法是用一條多項式一口氣穿過所有點。模擬畫布上的灰色虛線就是這樣做的：5 個點再加上兩端靜止共 7 個條件，需要 6 次多項式。在這組時刻下它已經在中段鼓起、高過所有經過點；點越多、時刻越不均勻，擺盪越嚴重，而且動任何一個點，整條曲線都會改變。

比較好的做法是**分段**：每兩個相鄰經過點之間各放一段三次多項式，要求接縫處位置與速度相等。平面上的 $x$、$y$（或關節空間的每個 $\\theta_i$）各自獨立套用同一套公式。`,
    simHint: '按「▶ 播放」看圓點依序經過 P₀…P₄；再拖曳經過點，或拖動下方時間軸上的時間標記，比較藍色分段三次曲線與灰色虛線（單一 6 次多項式）的變化。',
  },
  {
    title: '每段四個條件',
    md: `第 $j$ 段從 $t_j$ 走到 $t_{j+1}$，歷時 $T_j=t_{j+1}-t_j$。以 $\\tau=t-t_j$ 表示段內時間，每個座標寫成

$$q(\\tau)=a_0+a_1\\tau+a_2\\tau^2+a_3\\tau^3 .$$

四個係數恰好由**兩端的位置與速度**決定：$a_0=q_j$、$a_1=\\dot q_j$，另外兩個由終點條件解出

$$a_2=\\frac{3\\Delta q-(2\\dot q_j+\\dot q_{j+1})T_j}{T_j^{2}},\\qquad a_3=\\frac{-2\\Delta q+(\\dot q_j+\\dot q_{j+1})T_j}{T_j^{3}},$$

其中 $\\Delta q=q_{j+1}-q_j$。相鄰兩段在接縫共用同一組位置與速度，所以速度自動連續；剩下唯一要決定的，是**每個經過點的速度要給多少**。`,
    simHint: '點一下曲線上的某一段（或下方時間軸上的色塊），對照面板裡的「邊界條件」與「係數」兩個矩陣；再拖動這段的終點，看 a₂、a₃ 如何跟著變。',
  },
  {
    title: '經過點的速度該是多少',
    md: `常見的三種給法：

- **零速**：每個經過點速度都是 0。最單純，但機器人會在每個點**停一下**，而且每段速度峰值是該段平均速度的 1.5 倍。
- **啟發式**：對每個座標，取前後兩段平均斜率 $\\Delta q/T$ 的平均值；若兩段斜率正負相反（這個座標在此點折返）就給 0。起點與終點仍是 0。
- **自訂**：直接指定速度向量，例如希望末端沿某個方向平順地掠過經過點。

不論哪一種，位置與速度都連續；差別在曲線的形狀，以及速度在各段之間怎麼分配。`,
    simHint: '這裡先套用「零速」：速度圖在每個經過點時刻都歸零。改選「啟發式」比較；再選「自訂」，拖曳紫色箭頭的尖端改變經過點速度。',
  },
  {
    title: '過衝從哪裡來',
    md: `啟發式只看相鄰兩段的平均斜率。若一段很短、下一段很長，經過點的速度會被短段「拉高」；帶著這個速度進入長段後，三次多項式只好先衝過頭再折回來，這就是**過衝（overshoot）**，同時也會出現不合理的大速度。

一個實用的判斷：在某一段內，若兩端速度的方向都與該段位移相同，而且大小都不超過該段平均速度 $\\Delta q/T$ 的 3 倍，這段就不會過衝（只有一端有速度時，3 倍恰好是界線）。所以修正的方法通常是**重新分配時間**，讓相鄰段的平均速度接近，而不是硬去移動經過點。`,
    simHint: '這裡套用「短段 → 過衝」：紅圈標出 y 座標衝過頭的位置。在曲線下方的時間軸拖動 P₁ 的時間標記，把第一段拉長，直到紅圈消失。',
  },
  {
    title: '加速度連續嗎？',
    md: `分段三次只保證**位置與速度**連續。每段的加速度 $\\ddot q=2a_2+6a_3\\tau$ 由該段自己的四個條件決定，接縫兩側一般不相等，所以加速度圖在經過點會**跳一下**；連起步那一刻也從 0 跳到 $2a_2$。加速度跳變代表力矩突變，容易激起機構振動。

兩種補救方式：

- **五次多項式段**：每段再指定兩端的加速度，6 個條件定出 6 個係數；
- **整體三次樣條（C² spline）**：仍用三次，但經過點速度不再自由選擇，而是解一組聯立方程，讓每個接縫的加速度相等。代價是**失去局部性**：動一個點，所有段都會改變。`,
    simHint: '已套用「等時距」情境，右下圖切到「加速度」：實線在經過點時刻斷開跳變，虛線（五次段）則連續。再把「對照曲線」換成「整體三次樣條」比較，並看畫布上兩條路徑差多少。',
  },
  {
    title: '手臂跟著走，以及時間最佳化',
    md: `把這條平面路徑交給 2R 手臂（$L_1=L_2=1$ m，基座在原點）：每個時刻用逆向運動學解出關節角，並挑選與前一刻最接近的那組解，避免手肘半途翻面。若路徑離基座超過 2 m，那一段就追蹤不到。

還有一個沒處理的問題：各經過點的時刻 $t_k$ 是我們隨手指定的，但手臂的力矩有上限，路徑上某些地方可能根本跑不了那麼快。下一課換個問法：**路徑固定時，在力矩限制下最快能多快走完？** 答案要到 $(s,\\dot s)$ 相平面上去找。`,
    simHint: '已打開「2R 手臂跟隨」：播放時看右下的關節角圖；試著把一個經過點拖到虛線圓（工作空間邊界）外，觀察紅色無法到達的區段。',
  },
];

const quiz: QuizQuestion[] = [
  {
    question: '三個經過點排在同一直線上，兩段一樣長，但第一段只給 0.5 s、第二段給 4 s，使用啟發式速度。末端會怎麼動？',
    options: [
      '沿直線前進，而且不會越過任何一點',
      '離開這條直線，畫出一段弧',
      '仍然留在直線上，但第二段會先衝過終點再退回來',
      '在中間的經過點停一下再出發',
    ],
    answer: 2,
    explanation: '兩段位移平行，各座標的斜率同號，所以啟發式速度（兩段平均斜率的平均）仍沿著這條直線，三次曲線也只在線上移動。可是中間點的速度約為第二段平均速度的 $\\tfrac12(1+8)=4.5$ 倍，超過約 3 倍的界線，第二段只好先衝過終點再折回。可以套用「共線三點」情境驗證。',
  },
  {
    question: '若每一段只根據自己兩端的條件決定，又要求經過點處的**加速度**也連續，每段至少需要幾次多項式？',
    options: ['三次', '四次', '五次', '七次'],
    answer: 2,
    explanation: '兩端各指定位置、速度、加速度，共 6 個條件，需要 6 個係數，也就是五次多項式。如果堅持用三次，就得放棄「各段獨立」：把經過點速度當成未知數整體求解（三次樣條），代價是動一個點會影響所有段。',
  },
  {
    question: '拾取–放置任務依序經過：起點 → 物體上方 → 抓取點 → 物體上方 → 箱子上方 → 放置點。哪些點應設為零速度？',
    options: [
      '全部設成零速度最安全',
      '只有起點與最後的放置點',
      '起點、抓取點與放置點要靜止；「上方」這類讓路用的經過點應帶著速度通過',
      '都不需要，只要速度連續就好',
    ],
    answer: 2,
    explanation: '夾爪在抓取與放下的瞬間必須相對物體靜止，所以這些點速度要為 0（起點本來就靜止）。「物體上方」「箱子上方」只是避開碰撞的中途點，在那裡停下既浪費時間，也讓馬達反覆加減速。',
  },
  {
    question: '經過點與時刻都不變，只把速度策略從「啟發式」改成「零速」。畫布上的路徑會變成什麼樣子？',
    options: [
      '同一條平滑曲線，只是走得比較慢',
      '相鄰經過點之間的直線段（折線），而且在每個轉角停一下',
      '一條仍然平滑、但更貼近折線的曲線',
      '每一段都會衝過下一個經過點再折回',
    ],
    answer: 1,
    explanation: '兩端速度都為 0 時，每段的每個座標都是 $q_j+\\Delta q\\,(3u^2-2u^3)$（$u=\\tau/T_j$）。$x$ 與 $y$ 共用同一個純量函數 $s(u)=3u^2-2u^3$，所以 $p(\\tau)=p_j+s(u)\\,(p_{j+1}-p_j)$ 只沿著兩點間的弦移動——這正是前一課「直線路徑＋三次時間縮放」。轉角處速度為 0，方向才能瞬間改變；$s(u)$ 單調遞增，所以也不會過衝。可以套用「零速」情境驗證。',
  },
];

// ------------------------------------------------------------------------------------------------

type Compare = 'none' | 'quintic' | 'c2' | 'global';
type Tab = 'vel' | 'acc' | 'joint';

const WORLD = { xmin: -0.35, xmax: 2.25, ymin: -0.5, ymax: 1.25 };
const ARROW = 0.35; // arrow length = velocity × 0.35 s
const PER_SEG = 60; // samples per segment
const TL_ASPECT = 10.5; // timeline strip width / height
const SUBS = '₀₁₂₃₄₅₆₇₈₉';
const sub = (n: number) => String(n).split('').map((d) => SUBS[Number(d)]).join('');
const COMPARE_LABEL: Record<Compare, string> = {
  none: '無', quintic: '五次段（加速度連續）', c2: '整體三次樣條（C²）', global: '單一高次多項式',
};

interface Sampled { t: number[]; q: Pt2[]; qd: Pt2[]; qdd: Pt2[]; seg: number[] }

/** Sample a trajectory segment by segment (the first sample of a segment is just right of its start). */
function sampleSegments(tr: Trajectory2): Sampled {
  const out: Sampled = { t: [], q: [], qd: [], qdd: [], seg: [] };
  const T = tr.times;
  for (let k = 0; k < T.length - 1; k++) {
    for (let i = 0; i <= PER_SEG; i++) {
      const t = T[k] + ((T[k + 1] - T[k]) * i) / PER_SEG;
      const r = tr.evaluate(i === 0 && k > 0 ? t + 1e-9 : t);
      out.t.push(t); out.q.push(r.q); out.qd.push(r.qd); out.qdd.push(r.qdd); out.seg.push(k);
    }
  }
  return out;
}

function mount(host: HTMLElement, ctx: LessonContext): LessonHandle {
  const st = {
    points: [] as Pt2[],
    times: [] as number[],
    strategy: 'heuristic' as VelStrategy,
    custom: [] as Pt2[],
    selSeg: 1,
    selPt: 1,
    compare: 'none' as Compare,
    arm: false,
    t: 0,
    tab: 'vel' as Tab,
  };
  let sp!: ViaPointSpline;
  let tr!: Trajectory2;
  let cmp: Trajectory2 | null = null;
  let S!: Sampled;
  let C: Sampled | null = null;
  let overs: Overshoot[] = [];
  let thetas: Pt2[] = [];
  let reach: boolean[] = [];
  const Ttot = () => st.times[st.times.length - 1];

  // ---------------------------------------------------------------- layout
  simTitle(host, '經過點軌跡實驗台', '分段三次多項式');
  const cv = new Canvas2D(host, { aspect: 1.49, world: WORLD, overlay: true });
  Object.assign(cv.overlay.style, { left: 'auto', right: '0.5rem' });
  const tl = new Canvas2D(host, { aspect: TL_ASPECT, world: { xmin: -0.3, xmax: 4.5, ymin: 0, ymax: 4.8 / TL_ASPECT } });
  tl.el.style.marginTop = '0.3rem';

  const panel = controlPanel(host, { compact: true });
  const chipRow = row(panel);
  const chips = presetChips(chipRow, SCENARIO_IDS.map((id) => ({
    label: SCENARIOS[id].label, title: SCENARIOS[id].title, apply: () => applyScenario(id),
  })), { label: '情境' });
  const selRow = row(panel);
  const stratSel = select<VelStrategy>(selRow, {
    label: '經過點速度策略',
    options: [{ value: 'zero', label: '零速' }, { value: 'heuristic', label: '啟發式' }, { value: 'custom', label: '自訂（拖箭頭）' }],
    value: st.strategy,
    onChange: (v) => { setStrategy(v); chips.clear(); },
  });
  const cmpSel = select<Compare>(selRow, {
    label: '對照曲線（虛線）',
    options: (Object.keys(COMPARE_LABEL) as Compare[]).map((k) => ({ value: k, label: COMPARE_LABEL[k] })),
    value: st.compare,
    onChange: (v) => { st.compare = v; rebuild(); },
  });
  const armToggle = toggle(selRow, { label: '2R 手臂跟隨', value: false, onChange: (v) => { st.arm = v; rebuild(); } });
  const pbRow = row(panel);
  const pb = playback(pbRow, {
    duration: 4,
    onPlay: () => { if (st.t >= Ttot() - 1e-6) st.t = 0; loop.start(); },
    onPause: () => loop.stop(),
    onReset: () => { loop.stop(); pb.setPlaying(false, { silent: true }); setTime(0); },
    onSpeed: (s) => { loop.speed = s; },
    onScrub: (t) => setTime(t),
  });
  button(pbRow, { label: '＋插入點', small: true, title: '在選取段的中點（時間與位置都取中點）插入一個經過點', onClick: () => doInsert() });
  const delBtn = button(pbRow, { label: '－刪除點', small: true, title: '刪除選取的經過點（至少保留 3 點）', onClick: () => doDelete() });

  // Coefficient panel for the selected segment.
  const coef = el('div', { style: { display: 'flex', flexWrap: 'wrap', alignItems: 'center', gap: '0 0.9rem', margin: '0.15rem 0' } });
  host.appendChild(coef);
  const segLabel = el('div', { class: 'mr-note', style: { margin: '0', flexBasis: '100%' } });
  coef.appendChild(segLabel);
  const mBound = matrixView(coef, { label: '[q₀ q̇₀ q₁ q̇₁] =', digits: 2, title: '選取段的邊界條件：起點位置、起點速度、終點位置、終點速度；列＝x（紅）、y（綠）' });
  const mCoef = matrixView(coef, { label: '[a₀ a₁ a₂ a₃] =', digits: 2, title: '選取段的三次多項式係數；列＝x（紅）、y（綠）' });

  const status = el('div', { class: 'mr-note', style: { fontSize: '0.84rem', margin: '0.2rem 0 0.35rem' } });
  host.appendChild(status);

  const [colA, colB] = split(host);
  const plotOpts = { height: 165, legend: false, xLabel: 't (s)' };
  // Colours are CSS variable names: Plot2D resolves them at draw time, so theme switches just work.
  const xy = (a: string, b: string, cmp: boolean): SeriesSpec[] => [
    { name: a, color: '--axis-x' }, { name: b, color: '--axis-y' },
    ...(cmp ? [{ name: `${a} 對照`, color: '--axis-x', dash: [5, 4], width: 1.3 }, { name: `${b} 對照`, color: '--axis-y', dash: [5, 4], width: 1.3 }] : []),
  ];
  const velSeries = (): SeriesSpec[] => [...xy('ẋ', 'ẏ', false), { name: '|v|', color: '--fg-muted', dash: [3, 3], width: 1.3 }];
  const jSeries = (): SeriesSpec[] => [{ name: 'θ₁', color: '--series-1' }, { name: 'θ₂', color: '--series-4' }];
  // Extra headroom: overshoot markers sit at extrema and their labels are drawn above the dot.
  const pPos = new Plot2D(colA, { ...plotOpts, yPad: 0.2, title: '位置 x(t)、y(t)（m）', series: xy('x', 'y', true) });
  const tabRow = el('div');
  colB.appendChild(tabRow);
  const paneVel = el('div'), paneAcc = el('div'), paneJ = el('div');
  colB.append(paneVel, paneAcc, paneJ);
  const tabs = presetChips(tabRow, [
    { label: '速度', apply: () => showTab('vel') },
    { label: '加速度', apply: () => showTab('acc') },
    { label: '關節角', apply: () => showTab('joint') },
  ]);
  const pVel = new Plot2D(paneVel, { ...plotOpts, height: 135, title: '速度 ẋ、ẏ、|v|（m/s）', series: velSeries() });
  const pAcc = new Plot2D(paneAcc, { ...plotOpts, height: 135, title: '加速度 ẍ、ÿ（m/s²）', series: xy('ẍ', 'ÿ', true) });
  const pJ = new Plot2D(paneJ, { ...plotOpts, height: 135, title: '2R 關節角 θ₁（藍）、θ₂（橘）（rad）', series: jSeries() });
  const plots = [pPos, pVel, pAcc, pJ];
  const group = plotGroup(plots);
  legend(host, [
    { color: 'var(--accent)', label: '分段三次路徑' }, { color: 'var(--series-5)', label: '經過點速度（箭頭＝v×0.35 s）' },
    { color: 'var(--warn)', label: '目前位置' }, { color: 'var(--danger)', label: '過衝／無法到達' }, { color: 'var(--fg-muted)', label: '虛線＝對照曲線' },
  ]);

  // Preview card for the next lesson (step 6 only; it takes the place of the coefficient panel).
  const preview = el('div', { class: 'card', style: { padding: '0.4rem 0.6rem', margin: '0.2rem 0', display: 'none' } });
  host.insertBefore(preview, coef);
  preview.appendChild(mdElement('**下一課預告：$(s,\\dot s)$ 相平面。** 路徑固定後，力矩上限讓每個狀態 $(s,\\dot s)$ 只能有一段加速度區間 $[L,U]$；最快的走法只在全力加速與全力減速之間切換，而且不能越過速度極限曲線（紅），頂多在切點擦過它。'));
  const pv = new Canvas2D(preview, { aspect: 4.2, world: { xmin: -0.06, xmax: 1.06, ymin: -0.025, ymax: 0.245 } });

  // ---------------------------------------------------------------- sampling / rebuild
  function velocitiesFor(strategy: VelStrategy): Pt2[] {
    return buildSpline(st.points, st.times, strategy, st.custom).velocities.map((v) => [v[0], v[1]] as Pt2);
  }

  function rebuild(): void {
    sp = buildSpline(st.points, st.times, st.strategy, st.custom);
    tr = asTrajectory(sp);
    cmp = st.compare === 'quintic' ? quinticFromCubic(sp)
      : st.compare === 'c2' ? asTrajectory(buildSpline(st.points, st.times, 'custom', c2SplineVelocities(st.points, st.times)))
        : st.compare === 'global' ? globalPolynomial(st.points, st.times) : null;
    S = sampleSegments(tr);
    C = cmp ? sampleSegments(cmp) : null;
    overs = findOvershoots(sp, 0.02);
    const ik = trackIK(S.q);
    thetas = ik.thetas; reach = ik.reachable;
    st.selSeg = Math.min(st.selSeg, st.points.length - 2);
    st.selPt = Math.min(st.selPt, st.points.length - 1);
    st.t = Math.min(st.t, Ttot());
    syncHandles();
    updateTimelineWorld();
    updatePlots();
    updateCoef();
    updateStatus();
    pb.setDuration(Ttot());
    frame();
  }

  function updatePlots(): void {
    const x = S.q.map((q) => q[0]), y = S.q.map((q) => q[1]);
    pPos.setXY(0, S.t, x); pPos.setXY(1, S.t, y);
    pPos.setXY(2, C ? C.t : [], C ? C.q.map((q) => q[0]) : []);
    pPos.setXY(3, C ? C.t : [], C ? C.q.map((q) => q[1]) : []);
    pVel.setXY(0, S.t, S.qd.map((v) => v[0])); pVel.setXY(1, S.t, S.qd.map((v) => v[1]));
    pVel.setXY(2, S.t, S.qd.map((v) => Math.hypot(v[0], v[1])));
    // Break the acceleration lines at the via points so the jumps show as gaps.
    const brk = (arr: Sampled, j: number) => {
      const xs: number[] = [], ys: number[] = [];
      arr.t.forEach((t, i) => {
        if (i > 0 && arr.seg[i] !== arr.seg[i - 1]) { xs.push(t); ys.push(NaN); }
        xs.push(t); ys.push(arr.qdd[i][j]);
      });
      return [xs, ys] as const;
    };
    const [ax, ay] = brk(S, 0), [, by] = brk(S, 1);
    pAcc.setXY(0, ax, ay); pAcc.setXY(1, ax, by);
    if (C) { const [cx, cy] = brk(C, 0), [, dy] = brk(C, 1); pAcc.setXY(2, cx, cy); pAcc.setXY(3, cx, dy); } else { pAcc.setXY(2, [], []); pAcc.setXY(3, [], []); }
    pJ.setXY(0, S.t, thetas.map((q) => q[0]));
    pJ.setXY(1, S.t, thetas.map((q) => q[1]));
    group.setXRange([0, Ttot()]);

    const via: Marker[] = st.times.slice(1, -1).map((t) => ({ x: t, color: '--fg-muted', dash: [3, 4] }));
    group.setMarkers(0, [...via, ...overs.map((o) => ({ x: o.t, y: o.q[o.coord], color: '--danger', label: '過衝' }))]);
    group.setMarkers(1, via);
    const jumps = accelJumps(sp);
    // Label the largest jump at an interior via point (the step is about the joints between segments).
    let big = 0, bigIdx = -1;
    jumps.forEach((j, k) => {
      if (k === 0 || k === jumps.length - 1) return;
      const m = Math.hypot(j.jump[0], j.jump[1]);
      if (m > big) { big = m; bigIdx = k; }
    });
    const jm: Marker[] = [...via];
    jumps.forEach((j, k) => {
      const cBig = Math.abs(j.jump[0]) >= Math.abs(j.jump[1]) ? 0 : 1;
      for (const c of [0, 1] as const) {
        const col = c === 0 ? '--axis-x' : '--axis-y';
        const label = k === bigIdx && c === cBig ? `跳 ${fmt(Math.abs(j.jump[c]), 2)}` : undefined;
        // The label is drawn above its dot, so attach it to the lower one to keep it inside the plot.
        const lowerIsLeft = j.left[c] <= j.right[c];
        jm.push(
          { x: j.t, y: j.left[c], color: col, label: lowerIsLeft ? label : undefined },
          { x: j.t, y: j.right[c], color: col, label: lowerIsLeft ? undefined : label },
        );
      }
    });
    group.setMarkers(2, jm);
    group.setMarkers(3, via);
  }

  function updateCoef(): void {
    const k = st.selSeg;
    const T = st.times[k + 1] - st.times[k];
    const v0 = sp.velocities[k], v1 = sp.velocities[k + 1];
    const p0 = st.points[k], p1 = st.points[k + 1];
    mBound.set([[p0[0], v0[0], p1[0], v1[0]], [p0[1], v0[1], p1[1], v1[1]]], { highlightRows: ['x', 'y'] });
    mCoef.set([sp.coefficients[k][0], sp.coefficients[k][1]], { highlightRows: ['x', 'y'] });
    segLabel.textContent = `選取段 ${k + 1}：P${sub(k)} → P${sub(k + 1)}，T = ${fmt(T, 2)} s　左：邊界條件　右：係數（列 = x 紅、y 綠）　選取點 P${sub(st.selPt)}`;
    delBtn.disabled = st.points.length <= 3;
  }

  function updateStatus(): void {
    const parts: string[] = [];
    let peak = 0, peakSeg = 0, ratio = 0;
    for (let k = 0; k < st.points.length - 1; k++) {
      const s = segmentSpeeds(tr, st.points, k, 40);
      if (s.peak > peak) { peak = s.peak; peakSeg = k; ratio = s.avg > 1e-9 ? s.peak / s.avg : 0; }
    }
    parts.push(`最大速率 ${fmt(peak, 2)} m/s（段 ${peakSeg + 1}，為該段平均速度的 ${fmt(ratio, 1)} 倍）。`);
    if (st.strategy === 'zero') parts.push('零速策略：每個經過點都停一下。');
    if (overs.length) {
      const o = overs.reduce((a, b) => (b.excess > a.excess ? b : a));
      parts.push(`⚠ 過衝：段 ${o.seg + 1} 的 ${o.coord === 0 ? 'x' : 'y'} 座標超出兩端範圍 ${fmt(o.excess, 2)} m（紅圈）。`);
    }
    const bad = st.arm ? reach.filter((r) => !r).length : 0;
    if (bad) parts.push(`⚠ 有 ${fmt((100 * bad) / reach.length, 0)}% 的路徑在 2R 手臂工作空間外（紅色）。`);
    status.textContent = parts.join(' ');
    status.style.color = overs.length || bad ? 'var(--danger)' : 'var(--fg-muted)';
  }

  // ---------------------------------------------------------------- main canvas
  const at = (t: number) => tr.evaluate(t);
  function armAt(t: number): Pt2 | null {
    if (!thetas.length) return null;
    let a = 0, b = S.t.length - 1;
    if (t <= S.t[0]) return thetas[0];
    if (t >= S.t[b]) return thetas[b];
    while (b - a > 1) { const m = (a + b) >> 1; if (S.t[m] <= t) a = m; else b = m; }
    const f = S.t[b] > S.t[a] ? (t - S.t[a]) / (S.t[b] - S.t[a]) : 0;
    return [thetas[a][0] + f * (thetas[b][0] - thetas[a][0]), thetas[a][1] + f * (thetas[b][1] - thetas[a][1])];
  }

  cv.draw = (c) => {
    const accent = cssVar('--accent'), muted = cssVar('--fg-muted'), danger = cssVar('--danger'), purple = cssVar('--series-5');
    c.grid(0.5);
    c.frame({ x: 0, y: 0, theta: 0 }, 0.25, { labels: ['x', 'y'], width: 1.8 });
    if (st.arm) c.circle([0, 0], 2, { color: cssVar('--grid-strong'), dash: [5, 5], width: 1.2 });
    // comparison curve
    if (C) c.polyline(C.q, { color: muted, width: 1.6, dash: [6, 5] });
    // selected segment halo
    const selPts = S.q.filter((_, i) => S.seg[i] === st.selSeg);
    c.polyline(selPts, { color: accent, width: 10, alpha: 0.18 });
    c.polyline(S.q, { color: accent, width: 2.6 });
    // unreachable stretches
    if (st.arm) {
      let run: Pt[] = [];
      S.q.forEach((q, i) => {
        if (!reach[i]) run.push(q);
        else { if (run.length > 1) c.polyline(run, { color: danger, width: 4 }); run = []; }
      });
      if (run.length > 1) c.polyline(run, { color: danger, width: 4 });
    }
    // overshoots
    for (const o of overs) {
      c.circle(o.q, 0.07, { color: danger, width: 2.2 });
      c.text(o.q, '過衝', { color: danger, dy: -16, bg: cssVar('--bg-elev') });
    }
    // via velocity arrows and labels
    const K = st.points.length;
    st.points.forEach((p, k) => {
      const v = sp.velocities[k];
      const tip: Pt = [p[0] + v[0] * ARROW, p[1] + v[1] * ARROW];
      const onSel = k === st.selSeg || k === st.selSeg + 1;
      if (Math.hypot(v[0], v[1]) > 1e-6) c.arrow(p, tip, { color: purple, width: onSel ? 3 : 2, head: 9 });
      const below = k === 0 || k === K - 1;
      // Near the bottom edge a label below the point would be cut off: put it to the left instead.
      const pos = !below ? { dy: -18 } : (p[1] - WORLD.ymin) * cv.scale > 26 ? { dy: 18 } : { dx: -12, align: 'right' as const };
      c.text(p, `P${sub(k)}  t=${fmt(st.times[k], 1)}`, { color: cssVar('--fg'), ...pos, bg: cssVar('--bg-elev'), font: '11px ' + (cssVar('--font') || 'sans-serif') });
    });
    // arm or point robot
    const r = at(st.t);
    const th = st.arm ? armAt(st.t) : null;
    if (th) drawPlanarArm(c, [1, 1], th, { linkWidth: 7, eeRadius: 5, alpha: 0.95 });
    c.disk(r.q, 0.045, { fill: cssVar('--warn') });
    const sp0 = Math.hypot(r.qd[0], r.qd[1]);
    if (sp0 > 1e-4) c.arrow(r.q, [r.q[0] + r.qd[0] * ARROW, r.q[1] + r.qd[1] * ARROW], { color: cssVar('--warn'), width: 2, head: 8 });
    c.setOverlayText(`t = ${fmt(st.t, 2)} s  |v| = ${fmt(sp0, 2)} m/s`);
  };

  // Via-point handles (rebuilt when the number of points changes) and velocity-tip handles.
  let ptHandles: Handle[] = [];
  let velHandles: Handle[] = [];
  let wasPlaying = false;
  const grab = () => { wasPlaying = loop.running; loop.stop(); pb.setPlaying(false, { silent: true }); };
  const release = () => { if (wasPlaying) { loop.start(); pb.setPlaying(true, { silent: true }); } };
  const clampW = (p: Pt): Pt2 => [Math.max(WORLD.xmin + 0.05, Math.min(WORLD.xmax - 0.05, p[0])), Math.max(WORLD.ymin + 0.05, Math.min(WORLD.ymax - 0.05, p[1]))];

  function syncHandles(): void {
    const K = st.points.length;
    if (ptHandles.length !== K) {
      ptHandles.forEach((h) => cv.removeHandle(h));
      velHandles.forEach((h) => cv.removeHandle(h));
      ptHandles = st.points.map((_, k) => cv.addHandle({
        get: () => st.points[k] as Pt,
        set: (p) => { st.points[k] = clampW(p); chips.clear(); rebuild(); },
        onStart: () => { grab(); st.selPt = k; st.selSeg = Math.min(k, K - 2); updateCoef(); },
        onEnd: release,
        radius: 8,
      }));
      velHandles = st.points.map((_, k) => cv.addHandle({
        get: () => { const p = st.points[k], v = sp.velocities[k]; return [p[0] + v[0] * ARROW, p[1] + v[1] * ARROW]; },
        set: (q) => {
          const p = st.points[k];
          st.custom[k] = [(q[0] - p[0]) / ARROW, (q[1] - p[1]) / ARROW];
          chips.clear(); rebuild();
        },
        onStart: () => { grab(); st.selPt = k; st.selSeg = Math.min(k, K - 2); },
        onEnd: release,
        radius: 6, color: '--series-5',
      }));
    }
    velHandles.forEach((h, k) => { h.enabled = st.strategy === 'custom' && k > 0 && k < K - 1; });
  }

  // Click near the curve to select a segment.
  const unsubPointer = cv.onPointer({
    down: (p) => {
      let best = Infinity, seg = -1;
      S.q.forEach((q, i) => { const d = Math.hypot(q[0] - p[0], q[1] - p[1]); if (d < best) { best = d; seg = S.seg[i]; } });
      if (seg >= 0 && best * cv.scale < 14) { st.selSeg = seg; updateCoef(); cv.requestRender(); tl.requestRender(); }
    },
  });

  // ---------------------------------------------------------------- timeline
  const tlY = () => { const v = tl.visible(); return { lo: v.ymin, hi: v.ymax, mid: v.ymin + 0.55 * (v.ymax - v.ymin) }; };
  function updateTimelineWorld(): void {
    const xmax = Math.max(4.5, Ttot() + 0.5);
    tl.setWorld({ xmin: -0.3, xmax, ymin: 0, ymax: (xmax + 0.3) / TL_ASPECT });
  }
  tl.draw = (c) => {
    const { lo, hi, mid } = tlY();
    const h = hi - lo;
    const muted = cssVar('--fg-muted'), accent = cssVar('--accent'), fg = cssVar('--fg');
    const font = '11px ' + (cssVar('--font') || 'sans-serif');
    c.line([0, mid], [Ttot(), mid], { color: cssVar('--grid-strong'), width: 2 });
    for (let k = 0; k < st.times.length - 1; k++) {
      const a = st.times[k], b = st.times[k + 1];
      const sel = k === st.selSeg;
      c.line([a, mid], [b, mid], { color: sel ? accent : cssVar('--series-8'), width: sel ? 9 : 6, alpha: sel ? 0.55 : 0.28, cap: 'butt' });
      c.text([0.5 * (a + b), mid + 0.32 * h], `T${sub(k + 1)}=${fmt(b - a, 2)}`, { color: sel ? accent : muted, font });
    }
    for (let tt = 0; tt <= Ttot() + 1e-9; tt += 1) c.text([tt, lo + 0.13 * h], `${fmt(tt, 0)} s`, { color: muted, font });
    c.line([st.t, mid - 0.25 * h], [st.t, mid + 0.2 * h], { color: cssVar('--warn'), width: 2 });
    c.point([0, mid], 5, { color: fg });
  };
  const tlHandles: Handle[] = [];
  function syncTimelineHandles(): void {
    while (tlHandles.length > st.times.length - 1) tl.removeHandle(tlHandles.pop()!);
    while (tlHandles.length < st.times.length - 1) {
      const k = tlHandles.length + 1;
      tlHandles.push(tl.addHandle({
        get: () => [st.times[k], tlY().mid],
        set: (p) => {
          st.times[k] = clampViaTime(st.times, k, p[0]);
          chips.clear(); rebuild();
        },
        onStart: () => { grab(); st.selSeg = k - 1; st.selPt = k; updateCoef(); cv.requestRender(); },
        onEnd: () => { release(); updateTimelineWorld(); },
        radius: 7,
      }));
    }
  }
  const unsubTl = tl.onPointer({
    down: (p) => {
      for (let k = 0; k < st.times.length - 1; k++) {
        if (p[0] >= st.times[k] && p[0] <= st.times[k + 1]) { st.selSeg = k; updateCoef(); cv.requestRender(); tl.requestRender(); break; }
      }
    },
  });

  // ---------------------------------------------------------------- preview sketch (static)
  // A qualitative sketch with constant maximal acceleration / deceleration (ṡ² linear in s). The
  // limit curve has a V-shaped dip; the fastest curve is bang-bang and only TOUCHES the limit curve
  // at the bottom of the dip (a tangent point), it never runs along it.
  const PV = { sT: 0.55, dip: 0.075, w: 0.09, acc: 0.03, dec: 0.04 };
  const pvLim = (s: number) => {
    const u = Math.max(0, 1 - Math.abs(s - PV.sT) / PV.w);
    return 0.18 + 0.02 * Math.cos(2 * Math.PI * s) - PV.dip * u * u;
  };
  const pvSdT = pvLim(PV.sT); // speed at the tangent point
  const pvOpt = (s: number) => {
    const fwd = Math.sqrt(2 * PV.acc * s), bwd = Math.sqrt(2 * PV.dec * (1 - s));
    const intoDip = Math.sqrt(pvSdT ** 2 + 2 * PV.dec * Math.max(0, PV.sT - s));
    const outOfDip = Math.sqrt(pvSdT ** 2 + 2 * PV.acc * Math.max(0, s - PV.sT));
    return s <= PV.sT ? Math.min(fwd, intoDip) : Math.min(outOfDip, bwd);
  };
  // Switch points: where the acceleration curve meets the deceleration curve (ṡ² equal).
  const pvS1 = (pvSdT ** 2 + 2 * PV.dec * PV.sT) / (2 * (PV.acc + PV.dec));
  const pvS3 = (2 * PV.dec - pvSdT ** 2 + 2 * PV.acc * PV.sT) / (2 * (PV.acc + PV.dec));
  pv.draw = (c) => {
    const muted = '--fg-muted', danger = '--danger', purple = '--series-5';
    c.line([0, 0], [1.02, 0], { color: muted, width: 1.2 });
    c.line([0, 0], [0, 0.23], { color: muted, width: 1.2 });
    c.text([1.03, -0.05], 's', { color: muted });
    c.text([-0.045, 0.22], 'ṡ', { color: muted });
    const L: Pt[] = [], O: Pt[] = [];
    for (let i = 0; i <= 400; i++) {
      const s = i / 400;
      L.push([s, pvLim(s)]);
      O.push([s, pvOpt(s)]);
    }
    c.polyline(L, { color: danger, width: 2 });
    c.polyline(O, { color: purple, width: 2.4 });
    for (const s of [pvS1, PV.sT, pvS3]) {
      c.point([s, pvOpt(s)], 4.5, { color: purple });
      c.point([s, pvOpt(s)], 2, { color: '--bg-elev' });
    }
    const font = '11px ' + (cssVar('--font') || 'sans-serif');
    c.text([0.03, 0.222], '速度極限曲線（上方不可行）', { color: danger, font, align: 'left' });
    c.text([0.5, 0.03], '最快：加速／減速交替，只在谷底的切點擦過', { color: purple, font });
  };

  // ---------------------------------------------------------------- actions
  function setStrategy(v: VelStrategy): void {
    if (v === 'custom' && st.strategy !== 'custom') st.custom = velocitiesFor(st.strategy);
    st.strategy = v;
    stratSel.set(v, { silent: true });
    rebuild();
  }

  function applyScenario(id: ScenarioId): void {
    const s = SCENARIOS[id];
    loop.stop(); pb.setPlaying(false, { silent: true });
    st.points = s.points.map((p) => [p[0], p[1]] as Pt2);
    st.times = s.times.slice();
    st.strategy = s.strategy;
    st.custom = s.strategy === 'custom' ? centredVelocities(st.points, st.times) : st.points.map(() => [0, 0] as Pt2);
    stratSel.set(s.strategy, { silent: true });
    st.selSeg = Math.min(1, st.points.length - 2);
    st.selPt = 1;
    st.t = 0;
    syncTimelineHandles();
    rebuild();
  }

  function doInsert(): void {
    const k = st.selSeg;
    const vNew = tr.evaluate(0.5 * (st.times[k] + st.times[k + 1])).qd;
    const r = insertVia(st.points, st.times, k);
    st.custom = [...st.custom.slice(0, k + 1), [vNew[0], vNew[1]] as Pt2, ...st.custom.slice(k + 1)];
    st.points = r.points; st.times = r.times;
    st.selPt = k + 1;
    chips.clear(); syncTimelineHandles(); rebuild();
  }

  function doDelete(): void {
    if (st.points.length <= 3) return;
    const k = st.selPt;
    const r = removeVia(st.points, st.times, k);
    st.custom = st.custom.filter((_, i) => i !== k);
    st.points = r.points; st.times = r.times;
    st.selPt = Math.max(0, Math.min(k, st.points.length - 1));
    st.selSeg = Math.min(st.selSeg, st.points.length - 2);
    chips.clear(); syncTimelineHandles(); rebuild();
  }

  function showTab(t: Tab): void {
    st.tab = t;
    paneVel.style.display = t === 'vel' ? '' : 'none';
    paneAcc.style.display = t === 'acc' ? '' : 'none';
    paneJ.style.display = t === 'joint' ? '' : 'none';
  }

  function setTime(t: number): void { st.t = Math.max(0, Math.min(Ttot(), t)); frame(); }

  function frame(): void {
    group.setCursor(st.t);
    pb.setTime(st.t);
    cv.requestRender();
    tl.requestRender();
  }

  const loop = createLoop((dt) => {
    st.t += dt;
    if (st.t >= Ttot()) { st.t = Ttot(); loop.stop(); pb.setPlaying(false, { silent: true }); }
    frame();
  });

  // ---------------------------------------------------------------- steps
  let focused: HTMLElement[] = [];
  const setFocus = (els: HTMLElement[]) => {
    focused.forEach((f) => f.classList.remove('mr-focus'));
    focused = els;
    focused.forEach((f) => f.classList.add('mr-focus'));
  };
  const setView = (o: { compare?: Compare; arm?: boolean; tab?: Tab; seg?: number }) => {
    if (o.compare) { st.compare = o.compare; cmpSel.set(o.compare, { silent: true }); }
    if (o.arm != null) { st.arm = o.arm; armToggle.set(o.arm, { silent: true }); }
    if (o.tab) tabs.activate(['vel', 'acc', 'joint'].indexOf(o.tab));
    if (o.seg != null) st.selSeg = Math.min(o.seg, st.points.length - 2);
    rebuild();
  };
  const chip = (id: ScenarioId) => chips.activate(SCENARIO_IDS.indexOf(id));
  const onStep = (i: number) => {
    preview.style.display = i === 5 ? '' : 'none';
    coef.style.display = i === 5 ? 'none' : '';
    if (i === 5) pv.requestRender();
    switch (i) {
      case 0: chip('demo'); setView({ compare: 'global', arm: false, tab: 'vel' }); setFocus([cv.el]); break;
      case 1: setView({ compare: 'none', arm: false, tab: 'vel', seg: 1 }); setFocus([coef, cv.el]); break;
      case 2: chip('zero'); setView({ compare: 'none', arm: false, tab: 'vel' }); setFocus([stratSel.el, pVel.el]); break;
      case 3: chip('overshoot'); setView({ compare: 'none', arm: false, tab: 'vel', seg: 1 }); setFocus([tl.el, cv.el]); break;
      case 4: chip('equal'); setView({ compare: 'quintic', arm: false, tab: 'acc' }); setFocus([pAcc.el, cmpSel.el]); break;
      case 5: setView({ compare: 'none', arm: true, tab: 'joint' }); setFocus([armToggle.el, preview]); break;
      default: setFocus([]);
    }
  };

  applyScenario('demo');
  chip('demo');
  tabs.activate(0);
  onStep(ctx.currentStep);

  return {
    onStep,
    destroy() {
      loop.stop();
      unsubPointer();
      unsubTl();
      cv.destroy();
      tl.destroy();
      pv.destroy();
      plots.forEach((p) => p.destroy());
    },
  };
}

const lesson: Lesson = { meta, steps, quiz, mount };
export default lesson;
