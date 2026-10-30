import type { Lesson, LessonContext, LessonHandle } from '@/types';
import {
  el, cssVar, fmt, controlPanel, row, angleSlider, select, button, toggle, readout, matrixView, simTitle, note, kv,
  Canvas2D, Scene3D, Robot3D, drawPlanarArm, type SliderHandle,
} from '@/ui';
import { FKinSpace, planar2R, planar3R, rrr3, ur5, type RobotModel } from '@/mr';
import { meta } from './meta';

type RobotKey = 'planar2R' | 'planar3R' | 'rrr3' | 'ur5';
const MODELS: Record<RobotKey, RobotModel> = { planar2R, planar3R, rrr3, ur5 };
const LABELS: Record<RobotKey, string> = { planar2R: '平面 2R（L₁ = L₂ = 1 m）', planar3R: '平面 3R', rrr3: '空間 RRR', ur5: 'UR5（6R 工業手臂）' };

const steps = [
  {
    title: '開鏈機器人長什麼樣',
    md: `之後的課程幾乎都以**開鏈（open chain / serial chain）**機器人為例：一串**連桿（link）**用**關節（joint）**首尾相接，一端固定在地面，另一端是**末端執行器（end-effector）**。

- 本站用到的關節都是**旋轉關節（revolute, R）**，每個關節貢獻 1 個自由度。
- 固定端定義**空間座標系** $\\{s\\}$，末端執行器上黏著**物體座標系** $\\{b\\}$。
- 機器人要做的事，說到底就是：**給定關節角度，末端在哪裡？**（正向運動學）以及反過來的問題。`,
    simHint: '拉動 θ₁、θ₂，觀察末端（藍點）與 {b} 座標系怎麼動。',
  },
  {
    title: '關節變數與零位組態',
    md: `把所有關節角度排成一個向量 $\\theta = (\\theta_1, \\ldots, \\theta_n)$，就是機器人的**組態（configuration）**。

所有角度都是 0 的姿勢稱為**零位組態（home configuration）**，教材裡用 $M \\in \\SE(3)$ 表示這時末端座標系 $\\{b\\}$ 相對於 $\\{s\\}$ 的位姿。本站所有模型的零位都是「手臂伸直」。

> 試試看：把兩個角度都拉回 0°，末端應該落在 $(2, 0)$——因為兩根連桿各長 1 m。`,
    simHint: '按「回到零位」，再分別只動 θ₁ 或只動 θ₂，比較末端走的圓弧半徑。',
  },
  {
    title: '四個常駐模型',
    md: `| 模型 | 用途 |
|---|---|
| 平面 2R | 幾乎所有「可以畫在紙上」的概念：C-space、Jacobian、動力學、控制 |
| 平面 3R | 多一個冗餘自由度，用來談奇異點與逆運動學的多解 |
| 空間 RRR | 最簡單的 3D 手臂，用來看旋量與螺旋軸 |
| UR5 | 教材第 4 章的 6R 範例，參數與書上一致（W₁ = 0.109, W₂ = 0.082, L₁ = 0.425, L₂ = 0.392, H₁ = 0.089, H₂ = 0.095 m） |

下拉選單可以切換模型；3D 模型可用滑鼠旋轉視角。`,
    simHint: '切到「平面 3R」，試著讓末端停在同一個位置但用不同的關節角度組合。',
  },
  {
    title: '末端位姿用一個 4×4 矩陣描述',
    md: `面板下方顯示的 $T_{sb}$ 是末端座標系 $\\{b\\}$ 相對於 $\\{s\\}$ 的**齊次變換矩陣**：

$$T_{sb} = \\begin{bmatrix} R & p \\\\ 0 & 1 \\end{bmatrix}$$

- 前三個欄位（以紅綠藍標示）是 $\\{b\\}$ 的 $\\hat{x}_b, \\hat{y}_b, \\hat{z}_b$ 在 $\\{s\\}$ 中的方向；
- 最後一欄 $p$ 是 $\\{b\\}$ 原點的位置。

第 3 章會完整解釋這個矩陣；現在只要能把矩陣的欄位和畫面上的箭頭對起來就夠了。`,
    simHint: '切到「空間 RRR」，轉動 θ₁，看 T 的前兩欄如何跟著 {b} 的紅、綠箭頭改變。',
  },
  {
    title: '用教材的 UR5 範例做健檢',
    md: `教材在 §4.1 用 UR5 示範正向運動學：當 $\\theta = (0, -\\tfrac{\\pi}{2}, 0, 0, \\tfrac{\\pi}{2}, 0)$ 時，末端位置應為

$$p \\approx (0.095,\; 0.109,\; 0.988)\\ \\text{m}.$$

這個數字是本站數學函式庫的單元測試之一。之後學到 PoE 公式時，你會親手算出它；現在先按下預設按鈕，確認模擬和書上一致。`,
    simHint: '選「UR5」，按「教材範例組態」，核對末端位置讀數。',
  },
  {
    title: '接下來',
    md: `這兩課的導覽到此結束。從第 2 章開始，我們會回到最基本的問題：**描述一個機器人需要幾個數字？這些數字構成什麼樣的空間？**

完成下方的自我檢測後，前往 **Ch 2 組態空間**。`,
  },
];

function mount(host: HTMLElement, ctx: LessonContext): LessonHandle {
  simTitle(host, '機器人模型瀏覽器', '導覽範例');
  const viewHost = el('div');
  host.appendChild(viewHost);
  const panel = controlPanel(host);
  const topRow = row(panel);
  let current: RobotKey = 'planar2R';
  let thetas: number[] = [0, 0];
  let cv: Canvas2D | null = null;
  let scene: Scene3D | null = null;
  let robot: Robot3D | null = null;
  let sliders: SliderHandle[] = [];
  const jointPanel = el('div', { class: 'mr-controls compact' });
  const info = kv(host, ['末端位置 p (m)', '關節數 n']);
  const Tview = matrixView(host, { label: 'T_sb =', digits: 3 });
  note(host, '矩陣前三欄以 x 紅、y 綠、z 藍標示 {b} 的三個軸；最後一欄是位置。');

  const sel = select(topRow, {
    label: '模型',
    options: (Object.keys(MODELS) as RobotKey[]).map((k) => ({ value: k, label: LABELS[k] })),
    value: current,
    onChange: (k) => setRobot(k),
  });
  button(topRow, { label: '回到零位', small: true, onClick: () => setThetas(thetas.map(() => 0)) });
  const presetBtn = button(topRow, { label: '教材範例組態', small: true, title: 'θ = (0, −90°, 0, 0, 90°, 0)', onClick: () => { if (current === 'ur5') setThetas([0, -Math.PI / 2, 0, 0, Math.PI / 2, 0]); } });
  const axesToggle = toggle(topRow, { label: '顯示螺旋軸', value: false, onChange: (v) => robot?.showAxes(v) });
  const framesToggle = toggle(topRow, { label: '顯示關節座標系', value: false, onChange: (v) => robot?.showJointFrames(v) });
  host.insertBefore(jointPanel, info.el);
  const pRead = readout(topRow, { label: '', value: '' });
  pRead.el.style.display = 'none';

  function update(): void {
    const m = MODELS[current];
    const T = FKinSpace(m.M, m.Slist, thetas);
    if (robot) robot.setConfig(thetas);
    cv?.requestRender();
    info.set('末端位置 p (m)', `(${fmt(T[0][3], 3)}, ${fmt(T[1][3], 3)}, ${fmt(T[2][3], 3)})`);
    info.set('關節數 n', String(m.dof));
    Tview.set(T, { highlightCols: ['x', 'y', 'z', null] });
  }

  function setThetas(v: number[]): void {
    thetas = v.slice();
    sliders.forEach((s, i) => s.set(thetas[i], { silent: true }));
    update();
  }

  function buildSliders(m: RobotModel): void {
    jointPanel.replaceChildren();
    sliders = [];
    for (let i = 0; i < m.dof; i++) {
      const lim = m.jointLimits?.[i] ?? [-Math.PI, Math.PI];
      sliders.push(angleSlider(jointPanel, {
        label: `θ${'₀₁₂₃₄₅₆₇₈₉'[i + 1]}`, value: thetas[i] ?? 0, min: lim[0], max: lim[1],
        onInput: (v) => { thetas[i] = v; update(); },
      }));
    }
  }

  function setRobot(k: RobotKey): void {
    current = k;
    sel.set(k, { silent: true });
    const m = MODELS[k];
    thetas = new Array(m.dof).fill(0);
    cv?.destroy(); cv = null;
    robot?.destroy(); robot = null;
    scene?.destroy(); scene = null;
    viewHost.replaceChildren();
    const planar = k === 'planar2R' || k === 'planar3R';
    presetBtn.style.display = k === 'ur5' ? '' : 'none';
    axesToggle.el.style.display = planar ? 'none' : '';
    framesToggle.el.style.display = planar ? 'none' : '';
    if (planar) {
      const L = m.linkLengths ?? [1, 1];
      const reach = L.reduce((a, b) => a + b, 0) * 1.15;
      cv = new Canvas2D(viewHost, { aspect: 16 / 10, world: { xmin: -reach, xmax: reach, ymin: -reach * 0.7, ymax: reach * 0.7 }, overlay: true });
      cv.draw = (c) => {
        c.grid(0.5);
        c.circle([0, 0], L.reduce((a, b) => a + b, 0), { color: cssVar('--grid-strong'), dash: [4, 4], width: 1 });
        c.frame({ x: 0, y: 0, theta: 0 }, 0.35, { labels: 's' });
        drawPlanarArm(c, L, thetas, { frames: true, labels: true, angleArcs: true, frameSize: 0.3 });
        c.overlay.textContent = `θ = (${thetas.map((t) => fmt((t * 180) / Math.PI, 0) + '°').join(', ')})`;
      };
    } else {
      const isUR5 = k === 'ur5';
      scene = new Scene3D(viewHost, {
        aspect: 16 / 10, grid: isUR5 ? 2 : 2, axes: 0.3,
        cameraPos: isUR5 ? [1.5, -1.5, 1.1] : [1.2, -1.3, 0.9],
        target: isUR5 ? [0.2, 0.1, 0.5] : [0.2, 0, 0.35],
      });
      robot = new Robot3D(scene, m, { showAxes: true, showJointFrames: true, frameSize: isUR5 ? 0.15 : 0.18, linkRadius: isUR5 ? 0.025 : 0.02 });
      robot.showAxes(axesToggle.value);
      robot.showJointFrames(framesToggle.value);
    }
    buildSliders(m);
    update();
  }

  setRobot('planar2R');

  const perStep: (RobotKey | null)[] = ['planar2R', 'planar2R', 'planar3R', 'rrr3', 'ur5', null];
  let lastStep = -1;
  const onStep = (i: number) => {
    const want = perStep[i];
    if (want && want !== current && i !== lastStep) setRobot(want);
    if (i === 4 && current === 'ur5') setThetas([0, -Math.PI / 2, 0, 0, Math.PI / 2, 0]);
    lastStep = i;
  };
  onStep(ctx.currentStep);

  return {
    onStep,
    destroy() { cv?.destroy(); robot?.destroy(); scene?.destroy(); },
  };
}

const lesson: Lesson = {
  meta,
  steps,
  quiz: [
    {
      question: '一個有 6 個旋轉關節的開鏈機器人，關節空間的維度是多少？',
      options: ['3', '6', '12', '取決於連桿長度'],
      answer: 1,
      explanation: '開鏈中每個旋轉關節貢獻一個獨立的自由度，所以 $n$ 個關節就是 $n$ 維的組態空間；連桿長度只影響末端能到哪裡，不影響維度。',
    },
    {
      question: '平面 2R 手臂（L₁ = L₂ = 1 m）在零位組態時，末端位於哪裡？',
      options: ['$(0, 0)$', '$(1, 1)$', '$(2, 0)$', '$(0, 2)$'],
      answer: 2,
      explanation: '零位時兩根連桿都沿著 $\\hat{x}$ 伸直，末端在 $(L_1 + L_2, 0) = (2, 0)$。',
    },
    {
      question: '$T_{sb}$ 矩陣的第一欄（紅色）代表什麼？',
      options: ['$\\{s\\}$ 原點的位置', '$\\{b\\}$ 的 $\\hat{x}_b$ 軸在 $\\{s\\}$ 中的方向', '關節 1 的角度', '連桿 1 的長度'],
      answer: 1,
      explanation: '旋轉矩陣 $R$ 的三個欄位分別是 $\\{b\\}$ 的 $\\hat{x}_b, \\hat{y}_b, \\hat{z}_b$ 用 $\\{s\\}$ 的座標寫出來的向量；位置在最後一欄。',
    },
  ],
  mount,
};

export default lesson;
