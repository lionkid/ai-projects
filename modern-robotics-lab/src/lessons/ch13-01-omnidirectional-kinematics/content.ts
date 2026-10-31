/**
 * Narrative of ch13-01 (steps + quiz), kept free of DOM code so content.test.ts can render every
 * string through the real markdown + KaTeX pipeline. Original wording; the book is only a
 * reference for concepts and section numbers.
 *
 * CJK + CommonMark: write `**詞**（English）`, not `**詞（English）**後文` (a closing `**` after
 * full-width punctuation followed by a CJK letter does not close the bold).
 */
import type { LessonStep, QuizQuestion } from '@/types';

export const steps: LessonStep[] = [
  {
    title: '全向底盤能做什麼',
    md: `一般的車只能朝車頭方向走、靠轉彎改變方向。**全向底盤**（omnidirectional base）卻能在平面上往任何方向平移、原地自轉，或兩者同時做。祕密在輪子：輪緣上裝著一圈可以自由轉動的小**滾子**（roller），輪子只負責朝某個方向「推」，與它垂直的分量交給滾子滑掉。

底盤某一瞬間的運動用**物體 twist** 描述：

$$\\mathcal V_b=(\\omega,\\ v_x,\\ v_y)$$

$\\omega$ 是繞 $\\hat z$ 的角速度，$(v_x,v_y)$ 是 $\\{b\\}$ 原點的速度、以 $\\{b\\}$ 的座標寫出。三個分量可以各自指定，所以底盤有 3 個速度自由度。`,
    simHint: '底盤正一邊自轉一邊沿直線前進。點一下地圖用鍵盤駕駛：W/S 前後、A/D 側移、Q/E 自轉、空白鍵停；也可以試試上面的預設動作，看右下的輪速長條怎麼變。',
  },
  {
    title: '一顆輪子貢獻 H 的一列',
    md: `設輪子 $i$ 的中心在 $p_i=(x_i,y_i)$，驅動方向與 $\\hat x_b$ 夾 $\\beta_i$，滾子的自由滑動方向與輪軸夾 $\\gamma_i$（omni 輪 $\\gamma=0$，mecanum 輪 $\\pm45^\\circ$）。

著地點的速度是 $v+\\omega\\hat z\\times p_i$。沿滑動方向的分量由滾子吃掉；只有與它垂直、沿 $\\hat d_i$（角度 $\\beta_i+\\gamma_i$）的分量必須靠輪子滾出來，而轉速 $u_i$ 在 $\\hat d_i$ 方向只貢獻 $r_iu_i\\cos\\gamma_i$。因此

$$u_i=\\frac{1}{r_i\\cos\\gamma_i}\\begin{bmatrix}(p_i\\times\\hat d_i)_z & \\hat d_i^{\\T}\\end{bmatrix}\\mathcal V_b$$

第一個元素是力臂：離中心越遠、越橫著推的輪子，對自轉越敏感。`,
    simHint: '近景跟著底盤走。黑箭頭（著地點速度）＝藍箭頭（輪子沿驅動方向滾 r·uᵢ）＋紫虛線（滾子滑動）；兩者沿 d̂ᵢ 的分量相同，就是橘色粗箭頭。粉紅線是力臂。點「輪 1…」換輪子、拉 |γ| 看這一列怎麼變。',
  },
  {
    title: '疊起來：u = H(0) V_b',
    md: `把每顆輪子的列疊起來：

$$u=H(0)\\,\\mathcal V_b,\\qquad H(0)\\in\\R^{m\\times3}$$

給定想要的底盤 twist，乘一次矩陣就知道每顆輪子該轉多快——這就是全向底盤的逆運動學，不必解任何方程式。三輪 omni 的 $H$ 是可逆的 3×3 方陣，四輪 mecanum 則是 4×3。

$H(0)$ 的「0」是指 $\\phi=0$：如果命令給的是世界座標的 $\\dot q=(\\dot\\phi,\\dot x,\\dot y)$，要先把它轉回 $\\{b\\}$ 再乘，合起來寫成 $u=H(\\phi)\\dot q$。`,
    simHint: '依序按「前進」「側移」「原地轉」：每種純運動只用到 H 的一欄，u 就是那一欄乘上速度。注意 mecanum 側移時對角的兩輪同向。',
  },
  {
    title: '輪速上限 → 速度多面體',
    md: `馬達的轉速有上限 $|u_i|\\le u_{\\max}$。每個不等式在 $(\\omega,v_x,v_y)$ 空間切出夾在兩個平行平面之間的一片，全部取交集，就是可行 twist 的**凸多面體**；左下圖是它在目前 $\\omega$ 的切片。

這個集合不是球。mecanum 在 $\\omega=0$ 時是菱形：正前方和正側方最快，斜 45° 只剩 $1/\\sqrt2$。三輪 omni 是六邊形，垂直於某顆輪子驅動方向的方向最快。一旦同時自轉，輪速被分走一部分，切片就整個縮小。搖桿要求的速度若在切片外，模擬會沿原方向把它縮回邊界。`,
    simHint: '把搖桿點拖到多面體外面，看它被夾回邊界；再拉 ω，看切片縮小。虛線圓是「每個方向都做得到」的速度，標記點是最快的方向。',
  },
  {
    title: '四輪的不一致',
    md: `四輪 mecanum 有 4 個輪速，底盤卻只有 3 個自由度，所以只有落在 $H$ 欄空間（3 維）裡的輪速組合才對應得到真正的運動。剩下的方向

$$n\\propto(1,\\ 1,\\ -1,\\ -1),\\qquad H^{\\T}n=0$$

是「前輪往前推、後輪往後拉」：底盤哪裡也去不了，只會打滑、磨輪胎、耗電。若硬是下達任意輪速，模擬用最小平方 $\\hat{\\mathcal V}_b=H^{\\dagger}u$ 當作底盤實際的運動，剩下的 $u-H\\hat{\\mathcal V}_b$ 就是打滑量。反過來，只要輪速是用 $u=H\\mathcal V_b$ 算出來的，就一定一致。`,
    simHint: '按「內耗組合 n」：四個輪子都在轉，底盤卻原地不動，打滑長條全亮。再試「單輪 1」，最後切到三輪 omni 比較——三輪永遠不會打滑。',
  },
  {
    title: '積分到世界座標，與下一課',
    md: `$\\mathcal V_b$ 是在 $\\{b\\}$ 裡寫的；要在地圖上移動，先把它轉到世界座標：

$$\\dot q=\\begin{bmatrix}1&0&0\\\\0&\\cos\\phi&-\\sin\\phi\\\\0&\\sin\\phi&\\cos\\phi\\end{bmatrix}\\mathcal V_b$$

模擬每一小步都假設 twist 不變，用 $T\\leftarrow T\\exp([\\mathcal V_b]\\Delta t)$ 精確積分（等速圓弧），所以邊轉邊直走也不會飄。全向底盤的三個速度方向都能直接下令；下一課的差速車與汽車只有兩個輸入、側向被輪子鎖死，我們會看到它們怎麼靠來回繞行仍然到得了旁邊。`,
    simHint: '搖桿改用世界座標：W/A/S/D 對應地圖的上左下右，按住 Q/E 自轉，路徑依然筆直。試試「邊轉邊走正方形」，再看下方的下一課預告。',
  },
];

export const quiz: QuizQuestion[] = [
  {
    question: 'mecanum 底盤若把四個滾子角 $\\gamma_i$ 全部改成 0（變成四顆沿 $\\hat x_b$ 驅動的 omni 輪），會怎樣？',
    options: [
      '還是可以側移，只是側向最高速度變低',
      '$H$ 的 $v_y$ 欄變成 0：輪子既推不出、也擋不住側向運動，只剩前後與自轉可以控制',
      '$H$ 變成 4×4 的奇異矩陣',
      '底盤完全動不了',
    ],
    answer: 1,
    explanation: '$\\gamma=0$ 時每顆輪子的 $\\hat d_i$ 都沿 $\\hat x_b$，滑動方向都沿 $\\hat y_b$，第三欄 $\\sin(\\beta_i+\\gamma_i)/(r\\cos\\gamma_i)$ 全為 0，$H$ 的秩掉到 2。輪速只決定 $\\omega$ 與 $v_x$（像差速車），側向成了滾子自由滑動、無法主動控制的方向。把模擬的 |γ| 拉到 0，多面體在 $v_y$ 方向就沒有邊界了。',
  },
  {
    question: '要讓 mecanum 底盤原地逆時針自轉（$\\omega>0$、$v_x=v_y=0$），四輪（左前、右前、右後、左後）轉速的正負號是？',
    options: ['(+, +, +, +)', '(−, +, +, −)：左側往後、右側往前', '(+, −, +, −)：對角兩兩同向', '(+, +, −, −)'],
    answer: 1,
    explanation: '純自轉只用到 $H$ 的第一欄 $\\pm(\\ell+w)/r$：左側兩輪在 $+y$ 那邊，逆時針轉時它們的著地點往後走，所以 $u<0$；右側兩輪往前。(+, −, +, −) 是往右側移的模式，(+, +, −, −) 則是哪裡也去不了的內耗組合。',
  },
  {
    question: '四輪 mecanum 的 $H(0)$ 是 4×3。這代表什麼？',
    options: [
      '多一顆輪子就多一個運動自由度',
      '只有 3 維的輪速組合對應得到真實運動；多出的方向（前後輪互推）只會造成打滑，所以輪速應由 $u=H\\mathcal V_b$ 算出',
      '$H$ 不可逆，所以無法由 $\\mathcal V_b$ 算出輪速',
      '總有一顆輪子不需要轉',
    ],
    answer: 1,
    explanation: '$H$ 把 3 維的 twist 映到 4 維的輪速，像只落在 3 維的欄空間裡；正交補 $n\\propto(1,1,-1,-1)$ 滿足 $H^{\\T}n=0$，輪速沿 $n$ 的分量不對應任何底盤運動。由 $\\mathcal V_b$ 算 $u$ 永遠沒問題；反過來從量到的輪速推回 $\\mathcal V_b$ 才需要最小平方，殘差就是打滑。',
  },
  {
    question: '三輪 omni 的三個驅動方向互差 120°、輪速上限相同。只平移不旋轉時，哪個方向能跑最快？',
    options: [
      '每個方向一樣快：全向就是等向',
      '沿著某顆輪子的驅動方向',
      '垂直於某顆輪子的驅動方向：那顆輪子閒著、另兩顆分攤，大約快 15%',
      '只有正前方 $\\hat x_b$ 最快',
    ],
    answer: 2,
    explanation: '$\\omega=0$ 的切片是正六邊形，邊的法向量就是三個驅動方向。沿某個驅動方向走，那顆輪子得全速轉、成為瓶頸，最高只有 $u_{\\max}r$；垂直於它走時它只靠滾子滑動，另外兩顆各只需 $\\cos30^\\circ$ 的份量，速度可達 $u_{\\max}r/\\cos30^\\circ\\approx1.155\\,u_{\\max}r$——落在六邊形的頂點。',
  },
];
