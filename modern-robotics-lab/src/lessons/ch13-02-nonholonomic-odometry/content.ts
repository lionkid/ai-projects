/**
 * Narrative of ch13-02 (steps + quiz), kept free of DOM code so content.test.ts can render every
 * string through the real markdown + KaTeX pipeline. Original wording; the book is only a
 * reference for concepts and section numbers.
 *
 * CJK + CommonMark: write `**詞**（English）`, not `**詞（English）**後文`.
 */
import type { LessonStep, QuizQuestion } from '@/types';

export const steps: LessonStep[] = [
  {
    title: '三種底盤，一種限制',
    md: `**單輪車**（unicycle）、**差速驅動**（differential drive）與**車型**（car-like）機器人用的都是沒有滾子的一般輪：可以往前滾，卻不能往輪軸方向滑。寫成約束，就是車身的側向速度為零：

$$-\\dot x\\sin\\phi+\\dot y\\cos\\phi=0$$

所以每一瞬間，3 維的速度只剩 2 個方向可以用。三者的差別在輸入受到什麼限制：單輪車與差速車可以原地轉；車型機器人的前輪轉角有上限，轉彎半徑至少是 $r_{\\min}$，車子不動就轉不了向。`,
    simHint: '紅色箭頭（打叉）是被禁止的側向速度，紫點是瞬時旋轉中心，永遠落在輪軸（車型是後輪軸）的延長線上。用上方按鈕切換三種底盤；點一下地圖後可用 W/S 前後、A/D 轉向。',
  },
  {
    title: 'q̇ = G(q) u',
    md: `把允許的速度寫成兩個輸入的線性組合，就得到**無漂移**（driftless）的控制系統 $\\dot q=G(q)u$：輸入為零時機器人就停住。單輪車與車型的輸入是 $u=(v,\\omega)$：

$$\\dot q=\\begin{bmatrix}\\dot\\phi\\\\\\dot x\\\\\\dot y\\end{bmatrix}=\\begin{bmatrix}0&1\\\\\\cos\\phi&0\\\\\\sin\\phi&0\\end{bmatrix}\\begin{bmatrix}v\\\\\\omega\\end{bmatrix}$$

差速車的輸入是左右輪轉速，$v=\\tfrac r2(u_L+u_R)$、$\\omega=\\tfrac{r}{2d}(u_R-u_L)$，代進去就是它的 $G$。$G$ 的兩欄隨 $\\phi$ 轉動，卻永遠沒有側向分量。左下圖畫出三者的輸入集合：長方形、菱形與領結。`,
    simHint: '拖曳左下圖的點（或用鍵盤）改變輸入，看 G(q) 的欄隨 φ 變化。切到車型：領結在 v = 0 收成一點，按「原地轉」它也轉不動。',
  },
  {
    title: '側向可達：四段序列',
    md: `不能側滑，不代表到不了旁邊。試試「前進 $a$、原地左轉 $b$、後退 $a$、右轉 $b$」：每一段都合法，合起來卻留下一個淨位移，大小約 $ab$、方向朝車身右側。這正是兩個輸入向量場的 **Lie bracket**：

$$[g_1,g_2]=\\frac{\\partial g_2}{\\partial q}g_1-\\frac{\\partial g_1}{\\partial q}g_2=\\begin{bmatrix}0\\\\\\sin\\phi\\\\-\\cos\\phi\\end{bmatrix}$$

其中 $g_1$ 是前進、$g_2$ 是轉向。bracket 補上了缺少的側向方向，所以機器人仍到得了附近任何組態（可控）；代價是側移量只與步長的乘積成正比，步子小就得來回很多趟。`,
    simHint: '按「▶ 四段序列」：灰線是四段路徑，紫色箭頭是實際淨位移，虛線箭頭是 ab·[g₁, g₂] 的預測。把步長調小，兩者越來越吻合；切到車型改用四段全打的弧線。',
  },
  {
    title: '停車挑戰',
    md: `車型機器人連原地轉都不行，側移更費工。挑戰：把車往左平移 1 m、車頭方向不變，停進綠色車位。

每一段都只能走直線或半徑不小於 $r_{\\min}$ 的圓弧。一個經典解法：方向盤左打到底前進一段弧，再右打到底前進同樣的角度 $\\alpha$，車頭回正、整台車已往左移了

$$2r_{\\min}(1-\\cos\\alpha)$$

最後直線倒車，退回原來的前後位置。$r_{\\min}$ 越大，需要的前後空間越多。換成差速車就容易多了：原地轉 90°、前進、再轉回來。`,
    simHint: '用 W/S 前後、A/D 打方向盤，把車停進綠色車位（位置誤差 < 12 cm、角度 < 8°）。卡住時按「示範解法」，或把 r_min 調大看看會難多少。',
  },
  {
    title: '里程計',
    md: `機器人怎麼知道自己在哪？最簡單的方法是**里程計**（odometry）：每隔 $\\Delta t$ 讀一次左右輪編碼器的增量 $\\Delta\\theta_L,\\Delta\\theta_R$，換算成這段時間的車身位移 $\\Delta\\phi=\\tfrac{r}{2d}(\\Delta\\theta_R-\\Delta\\theta_L)$、$\\Delta x_b=\\tfrac r2(\\Delta\\theta_L+\\Delta\\theta_R)$，再假設期間 twist 不變，用指數映射更新位姿：

$$T_{k+1}=T_k\\exp([\\mathcal V_b]\\Delta t),\\qquad \\mathcal V_b\\Delta t=(\\Delta\\phi,\\ \\Delta x_b,\\ 0)$$

若偷懶寫成「沿舊車頭方向前進 $\\Delta x_b$」，等於假設這段時間車頭沒轉；一轉彎，位置就被算偏。`,
    simHint: '藍線是真實軌跡、橘色虛線是里程計估計；輪子完美時兩者重合。把更新方式切到「Euler」、編碼器頻率降到 2 Hz，看估計怎麼在轉彎時偏掉。',
  },
  {
    title: '漂移與下一課',
    md: `真實的輪子會打滑，兩輪半徑也不會剛好相等，這些誤差都被一路積分進去。最要命的是角度：若右輪比左輪大 $\\delta$（相對值），每走 $s$ 公尺，估計的車頭就偏 $\\delta s/(2d)$；角度誤差再乘上之後走的距離變成位置誤差：沿直線開時，側向誤差約 $\\delta s^2/(4d)$，隨 $s^2$ 成長；繞圈時則是整條估計路徑慢慢轉開。只靠里程計，開得越遠越不準，必須用地標、GPS 或雷射掃描來修正。

下一課換個問題：給定一條參考軌跡，怎麼用回授讓非完整的車子追上它？`,
    simHint: '「輪徑差 1%：直線」：估計往右偏，車頭誤差沿 −δs/(2d) 直線成長，位置誤差沿 δs²/(4d) 往上彎（虛線是預測）。「8 字」：每繞完一圈位置誤差會暫時變小，但估計的 8 字整個在轉。再試「打滑雜訊」。',
  },
];

export const quiz: QuizQuestion[] = [
  {
    question: '差速機器人朝 $+\\hat x$ 停在原點，依序執行「前進 0.2 m、原地左轉 0.2 rad、後退 0.2 m、原地右轉 0.2 rad」。結束時它大約在哪裡？',
    options: [
      '回到原點：每一段都被抵銷了',
      '原點右側（$-\\hat y$）約 4 cm，車頭方向不變',
      '原點左側（$+\\hat y$）約 4 cm，車頭方向不變',
      '原點前方約 4 cm',
    ],
    answer: 1,
    explanation: '淨位移是 $(a(1-\\cos b),\\ -a\\sin b)\\approx(0.004,\\ -0.040)$ m。二階近似正是 $ab\\,[g_1,g_2]$，而 $\\phi=0$ 時 $[g_1,g_2]=(0,0,-1)$ 指向右側。順序很重要：把左轉、右轉對調，bracket 變號，就往左移。',
  },
  {
    question: '車型機器人的 $r_{\\min}=1$ m。下列哪個 $(v,\\omega)$（單位 m/s、rad/s）它做不到，差速車卻可能做得到？',
    options: ['$(0.5,\\ 0.4)$', '$(-0.5,\\ 0.4)$', '$(0.2,\\ 0.5)$', '$(0.3,\\ -0.25)$'],
    answer: 2,
    explanation: '車型的限制是 $|\\omega|\\le|v|/r_{\\min}$，也就是轉彎半徑 $|v/\\omega|$ 至少 1 m。$(0.2,0.5)$ 的轉彎半徑只有 0.4 m，落在領結外；其他三個的半徑都不小於 1 m（倒車也能轉彎）。差速車只要兩輪轉速不超過上限就做得到，連原地轉都可以。',
  },
  {
    question: '差速機器人的右輪實際比左輪大 1%，里程計卻假設兩輪一樣大。機器人真的沿直線開 10 m 時，里程計的估計會怎樣？',
    options: [
      '也是直線，只是長度差 1%',
      '向右彎：角度誤差隨距離線性增加，位置誤差增加得更快（約與距離平方成正比）',
      '向左彎，而且誤差很快就穩定下來',
      '左右隨機抖動，平均起來沒有誤差',
    ],
    answer: 1,
    explanation: '要走直線，較大的右輪轉得比較少；里程計以為兩輪一樣大，就讀成「右輪慢」，誤以為車在右轉。角度誤差約 $\\delta s/(2d)$，隨距離線性成長；它再乘上之後走的距離變成側向誤差，約 $\\delta s^2/(4d)$。這是系統性誤差，不會平均掉。',
  },
  {
    question: '為什麼里程計用 $T_{k+1}=T_k\\exp([\\mathcal V_b]\\Delta t)$ 更新，而不是直接加上 $v\\Delta t\\,(\\cos\\phi_k,\\ \\sin\\phi_k)$？',
    options: [
      '因為 exp 算得比較快',
      '取樣期間車頭一直在轉，車其實走了一段圓弧；直接相加等於沿舊車頭走直線，轉彎時每一步的位移方向都錯了約 $\\omega\\Delta t/2$',
      '因為 exp 可以消除輪子打滑的誤差',
      '只有車型機器人需要 exp，差速車不需要',
    ],
    answer: 1,
    explanation: '只要 $(v,\\omega)$ 在取樣期間不變，$\\exp$ 給出的就是那段等速圓弧的終點，沒有積分誤差；直接相加則沿起點的車頭方向前進，與真正的弦差了 $\\omega\\Delta t/2$。取樣率越低、轉得越快，偏差越明顯。至於打滑這類感測誤差，用哪種更新都消除不了。',
  },
];
