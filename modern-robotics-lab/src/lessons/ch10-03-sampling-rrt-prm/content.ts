/**
 * Narrative of the RRT / PRM lesson (steps + quiz), kept free of DOM code so that
 * content.test.ts can render every string through the real markdown pipeline.
 *
 * CJK + CommonMark note: a closing `**` that follows full-width punctuation such as `）` and is
 * followed by a CJK letter does not close the bold, so write `**詞**（English）` instead of
 * `**詞（English）**後文`.
 */
import type { LessonStep, QuizQuestion } from '@/types';

export const steps: LessonStep[] = [
  {
    title: '不要格子，改用亂數',
    md: `格點法要把 C-space 的每一維都切成很多格，維度一高格子數就爆炸。**取樣式規劃**（sampling-based planning）換個做法：不去畫出整個 $\\mathcal C_{\\rm obs}$，只向**碰撞檢查器**問兩種問題：

1. 組態 $q$ 會不會撞？
2. 從 $q_a$ 直線走到 $q_b$ 的**局部路徑**（local path）安不安全？做法是沿線每隔 $\\varepsilon$ 取一點逐一檢查：

$$q_k=q_a+\\tfrac{k}{n}\\,(q_b-q_a),\\qquad n=\\big\\lceil \\|q_b-q_a\\|/\\varepsilon\\big\\rceil$$

右圖每一步丟一個亂數組態：撞到的打 ×，沒撞到的留下，並試著連到附近的點。規劃器自始至終**看不到地圖**，只知道這些問答的結果；$\\varepsilon$ 取得太大，薄障礙就可能落在兩個檢查點之間而被漏掉。`,
    simHint: '按「▸ 單步」多丟幾個樣本，再選「規劃器的視角」把障礙藏起來。最後選「ε = 0.5 m」：橘色連線通過了粗略檢查，卻穿過了細長的障礙。',
  },
  {
    title: 'RRT 的一個迭代',
    md: `**RRT**（rapidly-exploring random tree）從 $q_{\\rm start}$ 長出一棵樹，每一輪只做四件事：

1. **sample**：在 C-space 均勻取一點 $q_{\\rm rand}$；
2. **nearest**：找出樹上離它最近的節點 $q_{\\rm near}$；
3. **steer**：朝 $q_{\\rm rand}$ 只走一步 $\\delta$（若 $q_{\\rm rand}$ 本來就在 $\\delta$ 以內，直接取 $q_{\\rm new}=q_{\\rm rand}$）：
$$q_{\\rm new}=q_{\\rm near}+\\delta\\,\\frac{q_{\\rm rand}-q_{\\rm near}}{\\|q_{\\rm rand}-q_{\\rm near}\\|}$$
4. **check & add**：局部路徑 $q_{\\rm near}\\to q_{\\rm new}$ 通過檢查，才把 $q_{\\rm new}$ 接到樹上。

新節點離目標不到 $\\delta$ 而且能直線接上時，就沿著父節點一路回溯出整條路徑。`,
    simHint: '「逐階段」已開啟：每按一次「▸ 單步」只做一個動作，畫布上方的四格會標出目前階段。看完幾輪後按「▶ 自動」讓樹自己長。',
  },
  {
    title: '為什麼樹會往空曠處長',
    md: `$q_{\\rm rand}$ 是均勻取樣，所以某個節點被選成 $q_{\\rm near}$ 的機率，等於它的 **Voronoi 區域**（比起其他節點離它最近的那塊範圍）佔全圖的比例。

樹的外緣節點面對大片還沒探索的空間，Voronoi 區域最大；內部節點被鄰居包圍，區域很小。所以新節點大多長在外緣，樹自然往空曠處衝，這就是 **Voronoi 偏置**（Voronoi bias），也是名稱裡 *rapidly-exploring* 的由來。

對照組「隨機節點擴張」每次任選一個節點、往隨機方向長，沒有這種偏置，樹會擠在起點附近。`,
    simHint: '開著 Voronoi 區域按「▸ 單步」：星號 q_rand 總是落在 q_near 的色塊（高亮）裡。按「▶ 自動」看大色塊最先被吃掉；再用兩個「長到 300 個節點」預設比較兩者的「涵蓋率」。',
  },
  {
    title: '目標偏置與雙向生長',
    md: `均勻取樣完全不管目標在哪。常見的改良是**目標偏置**（goal bias）：以機率 $p_{\\rm goal}$ 直接拿 $q_{\\rm goal}$ 當 $q_{\\rm rand}$，把樹往目標拉。可是 $p_{\\rm goal}$ 太大就變成一路貪婪地衝向目標，前方有牆時只會反覆撞牆，失去探索能力。

另一個改良是**雙向 RRT**（RRT-Connect）：起點與目標各長一棵樹。每一輪先讓一棵樹朝亂數點長一步，另一棵樹再朝這個新節點連續前進，直到被擋住或接上為止；兩棵樹一相遇就得到路徑。在本課的牆與迷宮地圖上，它平均約省兩成碰撞查詢，個別種子還可能更慢；目標被障礙半包圍、單向的樹很難鑽進去時，差距才會明顯拉大。`,
    simHint: '依序按 p_goal = 0、0.05、1 三個預設，比較找到路徑需要的迭代數；p_goal = 1 時注意「被拒絕的延伸」一直累積。最後按「RRT-Connect」並換幾個種子，比較「碰撞查詢次數」（兩者每輪做的事不同，迭代數不能直接比）。',
  },
  {
    title: 'PRM：先建路網，再回答很多問題',
    md: `**PRM**（probabilistic roadmap）把工作拆成兩個階段：

- **建圖**：在 $\\mathcal C_{\\rm free}$ 取 $N$ 個樣本，每個樣本試著用局部路徑連到半徑 $r$ 內的鄰居，得到一張**路網**（roadmap）；
- **查詢**：把起點與終點接到附近的節點，再用 A* 在路網上搜尋。

建圖要花上萬次碰撞檢查，但只做一次；之後每個查詢只要把起點、終點接上路網（檢查次數通常不到建圖的 5%），再做一次圖搜尋。環境固定、查詢很多時，PRM 這種**多查詢**（multiple-query）規劃器最划算；RRT 每次都從起點重新長樹，屬於**單查詢**（single-query）規劃器。`,
    simHint: '拖曳 S 或 G：路徑幾乎立刻更新。比較「建圖成本」與「本次查詢成本」兩個讀數，再試試三個查詢預設。',
  },
  {
    title: '換成 2R 手臂的 C-space',
    md: `規劃器本身不用改，只要換掉**距離**與**碰撞檢查**兩個零件。2R 手臂的 C-space 是環面：$\\theta_i$ 與 $\\theta_i\\pm2\\pi$ 是同一個姿勢，所以兩組關節角的差要先繞回 $[-\\pi,\\pi)$：

$$\\Delta\\theta_i=\\operatorname{wrap}(\\theta_{b,i}-\\theta_{a,i}),\\qquad d(a,b)=\\|\\Delta\\theta\\|$$

最近鄰、steer 與局部路徑都改用這個差；碰撞檢查則是擺出手臂，量兩根有厚度的連桿到圓形障礙的距離。

RRT 的路徑由許多小段 $\\delta$ 接成，會左彎右拐。**捷徑法**（shortcutting）反覆在路徑上任取兩點，若兩點間的直線無碰撞，就把中間那段剪掉。`,
    simHint: '注意樹枝從左邊界穿出、從右邊界接回來。取消「環面接縫」看規劃會怎樣；再按「✂ 捷徑平滑」與「▶ 播放」，在右圖看手臂沿路徑移動。',
  },
  {
    title: '窄通道、完備性與下一課',
    md: `取樣式規劃器的保證比較弱，稱為**機率完備**（probabilistically complete）：只要存在一條和障礙保有些許間隙的解，樣本數越多，找到解的機率就越接近 1：

$$P\\big(\\text{找到解}\\mid N\\text{ 個樣本}\\big)\\;\\longrightarrow\\;1\\quad (N\\to\\infty)$$

但它不保證在有限時間內找到。窄通道是最難的情況：樣本落進通道的機率大約等於通道面積佔 $\\mathcal C_{\\rm free}$ 的比例，通道越窄，需要的 $N$ 越大；實務上會改用偏向障礙邊緣的取樣（例如 bridge test）。

下一課換個完全不同的想法：不取樣也不長樹，讓機器人沿著**虛擬位能場**的梯度往下滑。`,
    simHint: '下方的曲線是成功率實驗（每個 N 用 20 個種子）。按「通道 0.3 m」再按「▶ 成功率實驗」，比較兩條曲線何時接近 100%。',
  },
];

export const quiz: QuizQuestion[] = [
  {
    question: '把 RRT 的目標偏置設成 $p_{\\rm goal}=1$，在「一道牆」地圖上會發生什麼事？',
    options: [
      '每一步都朝目標走，所以會比 $p_{\\rm goal}=0.05$ 更快找到路徑',
      '樹變成一條朝目標的直線；撞上牆之後每一輪都提出同一個被拒絕的延伸，樹不再長大',
      '演算法會自動改用 PRM',
      '和 $p_{\\rm goal}=0$ 完全一樣，因為取樣仍是隨機的',
    ],
    answer: 1,
    explanation: '$q_{\\rm rand}$ 永遠是目標，所以 $q_{\\rm near}$ 永遠是離目標最近的那個節點、steer 的方向也永遠相同。一旦這一步撞牆，往後每一輪都重複同一個失敗的延伸。均勻取樣帶來的探索（Voronoi 偏置）完全消失，演算法也就不再機率完備。小小的 $p_{\\rm goal}$ 才是有用的折衷。',
  },
  {
    question: '倉庫的貨架位置固定，搬運機器人整天要回答上千次不同起點與終點的路徑查詢。比較合適的做法是？',
    options: [
      '每次查詢都跑一次 RRT，因為 RRT 是單查詢規劃器，速度最快',
      '先花時間建一次 PRM 路網，之後每個查詢只要接上起終點再做圖搜尋',
      '每次查詢都重新把 C-space 切成格子',
      '只用目標偏置 $p_{\\rm goal}=1$ 的 RRT',
    ],
    answer: 1,
    explanation: '環境不變時，建圖的成本（大量碰撞檢查）只付一次，就能分攤給所有查詢；每個查詢只需要把起點、終點接上路網（檢查次數遠少於建圖）再做一次 A*。RRT 的樹以起點為根，換了起點就得重長，無法重複利用。',
  },
  {
    question: 'RRT 找到的路徑為什麼通常要再做平滑（例如捷徑法）？',
    options: [
      '因為 RRT 的路徑常常會穿過障礙',
      '因為樹枝是朝亂數點一步步長出來的，路徑跟著亂數方向左彎右拐；RRT 只求可行，不求最短',
      '因為步長 $\\delta$ 越小路徑越長，平滑可以把 $\\delta$ 變大',
      '因為平滑之後碰撞檢查就不需要了',
    ],
    answer: 1,
    explanation: 'RRT 保證的是可行（每一段都通過碰撞檢查），不是最佳。路徑的形狀取決於當時抽到的亂數點，所以會有多餘的轉折。捷徑法把可以直線相連的兩點之間的轉折剪掉，路徑通常短很多；被剪掉的部分換成新的直線，仍然要通過碰撞檢查。',
  },
  {
    question: '在 2R 手臂的 C-space 地圖上，如果把關節角當成一般的方形區域（$-180°$ 與 $180°$ 不相接），最可能的結果是？',
    options: [
      '找到一樣的路徑，因為環面只是畫法不同',
      '規劃一定更快，因為距離比較好算',
      '規劃器回報失敗，或只找到繞遠路的解，因為穿過 $\\pm180°$ 的那條路被當成了牆',
      '手臂會撞到自己',
    ],
    answer: 2,
    explanation: '$\\theta$ 與 $\\theta\\pm2\\pi$ 是同一個姿勢。若忽略這點，最近鄰與局部路徑都不會穿過接縫，等於在 C-space 加了一道不存在的牆。本課的地圖中，起點與目標只能經由 $\\pm180°$ 相連，所以忽略環面時規劃器找不到任何解，即使解確實存在。',
  },
];
