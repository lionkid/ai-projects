# Modern Robotics Lab 課程地圖（curriculum）

本文件是 `docs/curriculum.json` 的人類可讀版。每課的完整規格（目標、步驟、simState、公式、測驗構想、相依）請看 JSON；實作規範請看 `docs/lesson-authoring.md`。教材 *Modern Robotics v2*（Lynch & Park）只作為概念與章節編號（`bookRef`）的參考，所有文字皆為原創。

- 課程數：**37**（核心路徑 intro+core 30 課，advanced 延伸 7 課）
- 預估總時數：約 1195 分鐘
- 每章課數：Ch2 3、Ch3 4、Ch4 2、Ch5 3、Ch6 2、Ch7 2、Ch8 4、Ch9 4、Ch10 4、Ch11 4、Ch12 2、Ch13 3


## 章節 → 課程 → 模擬 → 教材章節


### 第 2 章 組態空間（Configuration Space）

| # | 課程 id | 課名 | 模擬（一句話） | 類型 / 模型 | bookRef | 分鐘 | 難度 |
|---|---|---|---|---|---|---|---|
| 1 | `ch02-01-dof-grubler` | 自由度與 Grübler 公式（degrees of freedom） | 平面連桿積木：從預設機構（四連桿、曲柄滑塊、五連桿、平行四邊形疊加、三角桁架）出發，或自行放置連桿與 R/P 關節、指定接地桿；側欄即時顯示 N、J、Σf_i 與 Grübler 結果，可拖動自由關節驗證機構真的能動。 | 2D 畫布 / `linkage` | §2.1–2.2 | 25 | intro |
| 2 | `ch02-02-cspace-topology` | C-space 的拓撲與表示法（topology & representation） | 三聯動視圖：左為 (θ₁, θ₂) 方格、中為 Three.js 環面、右為 2R 手臂與 workspace 環帶。 | 混合（畫布＋圖表／3D） / `planar2R` | §2.3 | 30 | intro |
| 3 | `ch02-03-constraints-task-space` | 完整／非完整約束、任務空間與工作空間（holonomic, Pfaffian） | 俯視圖的單輪車／硬幣：以滑桿或鍵盤給前進速度 v 與轉向率 ω，軌跡畫在地面，側欄顯示 q=(φ, x, y) 與約束矩陣 A(q)。 | 2D 畫布 / `unicycle` | §2.4–2.5 | 30 | core |

**ch02-01-dof-grubler** — 學會數出剛體與連桿機構的自由度，用 Grübler 公式快速計算平面／空間機構的 dof，並認得公式失效的特殊幾何。  
步驟：一個剛體有幾個自由度？ → 關節是「拿走」自由度 → Grübler 公式 → 動手驗證：四連桿、曲柄滑塊、五連桿 → 公式失靈的時候 → 從自由度到組態空間  
相依：—

**ch02-02-cspace-topology** — 用 2R 手臂看懂組態空間（C-space）的真正形狀是環面而非方格，並比較顯式座標與隱式表示的取捨。  
步驟：工作空間不是組態空間 → 把角度畫成方格 → 黏合邊界：環面 → 拓撲 vs 表示法 → 路徑在三個視圖中長什麼樣 → 其他系統的 C-space  
相依：`ch02-01-dof-grubler`

**ch02-03-constraints-task-space** — 用俯視的單輪車體會「速度被限制，構型卻沒有被限制」，學會把約束寫成 Pfaffian 形式並判斷可積性，同時釐清 task space 與 workspace。  
步驟：硬幣只能往前滾 → 寫成 Pfaffian 形式 → 能不能積分？完整 vs 非完整 → 停車挑戰 → Task space 與 workspace → 下一步：描述剛體的位姿  
相依：`ch02-02-cspace-topology`

### 第 3 章 剛體運動（Rigid-Body Motions）

| # | 課程 id | 課名 | 模擬（一句話） | 類型 / 模型 | bookRef | 分鐘 | 難度 |
|---|---|---|---|---|---|---|---|
| 1 | `ch03-01-rotation-matrices` | 剛體位姿與旋轉矩陣（rotation matrix, SO(3)） | Three.js 場景中的 {s} 與可操作的 {b}。步驟 1–2 鎖定為俯視平面模式（只繞 z 旋轉、xy 平移），之後開放三維。 | Three.js 3D / `frames` | §3.1, 3.2.1 | 35 | intro |
| 2 | `ch03-02-exponential-coordinates` | 角速度與旋轉的指數座標（exponential coordinates, Rodrigues） | 單位球上可拖的旋轉軸 ω̂ 與一個物體座標系；拖 θ 看物體繞軸轉並留下圓弧軌跡，面板顯示 exp([ω̂]θ) 的三項分解。 | Three.js 3D / `frames` | §3.2.2–3.2.3 | 35 | core |
| 3 | `ch03-03-transforms-twists` | 齊次變換、旋量（twist）與 Adjoint | 場景中有 {s}、{b}、{c} 三個座標系；滑桿設定 T_sb、T_bc，面板顯示 T_sc 與反矩陣。 | Three.js 3D / `frames` | §3.3.1–3.3.2 | 35 | core |
| 4 | `ch03-04-screw-motions-wrenches` | 螺旋運動的指數座標與力旋量（screw motion, wrench） | 可拖的螺旋軸（點 q、方向 ŝ、節距 h）；拖 θ 看座標系沿軸滑動與旋轉並留下螺旋軌跡，面板顯示 exp([S]θ)。 | Three.js 3D / `frames` | §3.3.3–3.4 | 40 | core |

**ch03-01-rotation-matrices** — 從平面座標系暖身到三維旋轉矩陣：把 R 的三個欄位讀成 {b} 軸的方向，理解 R 的三種用途，並建立「前乘＝固定軸、後乘＝物體軸」的規則。  
步驟：平面暖身：用 (R, p) 描述一個座標系 → 同一個點，兩套座標 → 旋轉矩陣的三個欄位 → SO(3)：正交、det = 1、三個自由度 → 合成與下標消去 → 前乘＝固定軸，後乘＝物體軸 → R 的三種用途與下一步  
相依：`ch02-02-cspace-topology`

**ch03-02-exponential-coordinates** — 把旋轉看成「繞某軸轉某角」：由角速度導出 so(3)，用 Rodrigues 公式計算 exp，再用矩陣對數反推軸與角，並處理 θ=0、π 的特殊情況。  
步驟：旋轉中的物體：角速度向量 → ω_s 與 ω_b 是同一個向量 → 繞固定軸轉 θ：Rodrigues 公式 → 反向：矩陣對數 → θ = π 與 θ = 0 的特殊情況 → 指數座標不是 Euler 角  
相依：`ch03-01-rotation-matrices`

**ch03-03-transforms-twists** — 把 R 與 p 裝進 4×4 的 T 來合成、求逆、換參考系；再把剛體速度寫成六維旋量，理解 V_s 與 V_b 的差異，並用 Adjoint 在座標系之間轉換。  
步驟：把 R 與 p 裝進 4×4 → 合成、求逆、換參考系 → 前乘／後乘再一次 → 旋量：角速度加參考原點的線速度 → V_s 與 V_b → Adjoint：換座標的 6×6 矩陣 → 預告：每個 twist 都有一條螺旋軸  
相依：`ch03-02-exponential-coordinates`

**ch03-04-screw-motions-wrenches** — 任何剛體位移都是沿某條螺旋軸的一段運動：由 (q, ŝ, h) 建構 S，動畫 exp([S]θ)，再用 log 反推；最後把力與力矩合成 wrench，用 Adᵀ 換座標並確認功率不變。  
步驟：Chasles：任何位移都是一段螺旋 → 螺旋軸 S = (ω, v) → exp([S]θ) 動畫 → 反向：log T → 力旋量 wrench → 用 Adᵀ 換座標，功率不變 → 下一章：把關節軸當成螺旋軸  
相依：`ch03-03-transforms-twists`

### 第 4 章 正向運動學（Forward Kinematics）

| # | 課程 id | 課名 | 模擬（一句話） | 類型 / 模型 | bookRef | 分鐘 | 難度 |
|---|---|---|---|---|---|---|---|
| 1 | `ch04-01-poe-space-form` | 指數積公式的空間形式（PoE, space form） | 以空間 RRR（rrr3）為主、平面 3R 與 RRRP 為預設範例的 Three.js 場景：零位姿時畫出每條 S_i，關節滑桿驅動手臂；「逐項套用」按鈕從 e^{[S_n]θ_n} 開始一次乘一項，顯示中間座標系；也可只動單一關節，看下游的軸被帶走、上游的軸不動。 | Three.js 3D / `rrr3` | §4.1.1–4.1.2 | 35 | core |
| 2 | `ch04-02-poe-body-form-urdf` | PoE 物體形式、UR5 範例與 URDF（body form） | UR5 桿狀模型（書中幾何參數 W1, W2, L1, L2, H1, H2）。 | Three.js 3D / `ur5` | §4.1.3, 4.2 | 35 | core |

**ch04-01-poe-space-form** — 把每個關節當成一條螺旋軸，在零位姿寫下 M 與 S_i，用連乘的指數把手臂「組」出來，並理解乘法順序的由來。  
步驟：一個關節就是一條螺旋軸 → 零位姿：寫下 M 與每條 S_i → 為什麼是從右邊乘起 → 動一個關節，看看誰跟著動 → 換個零位姿試試 → 平面 3R、RRRP 與下一課  
相依：`ch03-04-screw-motions-wrenches`

**ch04-02-poe-body-form-urdf** — 用末端座標系的螺旋軸 B_i 寫出同一個 FK，推導 B_i = Ad_{M⁻¹} S_i，在 UR5 上驗證兩種形式一致，並對照 URDF 的關節描述。  
步驟：同一條鏈，從末端看 → 為什麼可以把 M 移到最前面 → Body form：後乘的順序 → 在 UR5 上驗證 → URDF 說的是另一種話 → 換工具、換零位姿與下一章  
相依：`ch04-01-poe-space-form`

### 第 5 章 速度運動學與靜力學（Velocity Kinematics and Statics）

| # | 課程 id | 課名 | 模擬（一句話） | 類型 / 模型 | bookRef | 分鐘 | 難度 |
|---|---|---|---|---|---|---|---|
| 1 | `ch05-01-jacobian-geometry` | 空間與物體 Jacobian 的幾何意義（space / body Jacobian） | 平面 3R（2D 畫布）為主、rrr3（3D）可切換：每個關節畫出目前的螺旋軸（即 J 的欄）。 | 混合（畫布＋圖表／3D） / `planar3R` | §5.1.1–5.1.4 | 35 | core |
| 2 | `ch05-02-singularities-manipulability` | 奇異點與可操作度橢球（singularity, manipulability ellipsoid） | 平面 2R／3R 手臂末端畫出速度橢球（由末端線速度子 Jacobian 的 JJᵀ 特徵分解）；底部曲線圖顯示 σ_max、σ_min 與條件數隨構型變化；可開啟 workspace 的 σ_min 熱圖，拖末端逼近伸直或折疊看橢球壓扁。 | 2D 畫布 / `planar2R` | §5.3–5.4 | 35 | core |
| 3 | `ch05-03-statics-force-ellipsoid` | 靜力學 τ = Jᵀ F 與力橢球（statics, force ellipsoid） | 2R／3R 手臂末端可拖出一個外力向量 F；每個關節以弧形寬度與數值條顯示 τ_i；可疊畫速度橢球與力橢球（主軸互為倒數），並有「伸直構型沿連桿施力」的預設。 | 2D 畫布 / `planar2R` | §5.2, 5.4 | 30 | core |

**ch05-01-jacobian-geometry** — 把 Jacobian 的每一欄讀成「該關節在目前構型的螺旋軸」，看懂末端 twist 是各欄乘關節速度的疊加，並用 Adjoint 連結 J_s 與 J_b。  
步驟：一次動一個關節 → J_s 的欄位怎麼算 → 疊加：V = J θ̇ → J_b：從末端看同一件事 → 兩個 Jacobian 之間只差一個 Adjoint → v 不是末端速度，以及下一課  
相依：`ch04-02-poe-body-form-urdf`

**ch05-02-singularities-manipulability** — 用幾何辨認 Jacobian 降秩的構型，畫出可操作度橢球並解讀長短軸，用條件數量化「離奇異點多近」。  
步驟：伸直的手臂推不動自己 → 奇異是構型的性質 → 可操作度橢球 → 量化：σ 與條件數 → 整個工作空間的地圖 → 接近奇異的代價與下一課  
相依：`ch05-01-jacobian-geometry`

**ch05-03-statics-force-ellipsoid** — 用虛功推導 τ = JᵀF，看懂力橢球與速度橢球互為對偶，並解釋為何奇異構型反而適合承受特定方向的負載。  
步驟：外力到關節力矩 → 用功率推導 τ = Jᵀ F → 力橢球 → 奇異構型反而能扛 → 同一個力，不同的構型 → 方向約定與下一章  
相依：`ch05-02-singularities-manipulability`、`ch03-04-screw-motions-wrenches`

### 第 6 章 逆向運動學（Inverse Kinematics）

| # | 課程 id | 課名 | 模擬（一句話） | 類型 / 模型 | bookRef | 分鐘 | 難度 |
|---|---|---|---|---|---|---|---|
| 1 | `ch06-01-analytic-ik` | 解析逆運動學：2R 手臂的兩組解（analytic IK） | 拖目標點，同時畫出兩個 IK 解（實線／虛線），workspace 環帶著色，目標落在環帶外時變紅；數值面板顯示餘弦定理的中間量 cos θ₂ 與 atan2 的象限；旁邊的小關節空間圖標出兩個解的位置。 | 2D 畫布 / `planar2R` | §6.1 | 25 | intro |
| 2 | `ch06-02-numerical-ik` | 數值逆運動學：Newton–Raphson 與阻尼最小平方（numerical IK, DLS） | 3R 平面手臂加目標位姿（含朝向）。「單步」執行一次 Newton 迭代並畫出手臂軌跡與中間構型；下方曲線圖顯示 ‖ω_b‖、‖v_b‖ 隨迭代變化（對數軸）。 | 混合（畫布＋圖表／3D） / `planar3R` | §6.2–6.3 | 40 | advanced |

**ch06-01-analytic-ik** — 用餘弦定理與 atan2 導出 2R 手臂的 elbow-up / elbow-down 解，判斷可達性與邊界上的單解，並預覽 6R 手臂「位置三關節＋姿態三關節」的分解思路。  
步驟：給一個點，找兩組角度 → 餘弦定理給 θ₂ → atan2 給 θ₁ → 可達範圍：環帶與邊界 → 拖著走：分支與連續性 → 六軸手臂怎麼辦，以及數值方法  
相依：`ch05-02-singularities-manipulability`、`ch02-02-cspace-topology`

**ch06-02-numerical-ik** — 把 IK 寫成求根問題，用 log 定義 twist 誤差並逐步執行 Newton 迭代觀察收斂曲線；近奇異點時改用阻尼最小平方，並連結到逆速度運動學。  
步驟：把 IK 變成求根 → 一步 Newton → 收斂曲線與初值 → 奇異附近會發生什麼 → 阻尼最小平方 → 冗餘與最小範數解 → 逆速度運動學與下一章  
相依：`ch06-01-analytic-ik`、`ch05-01-jacobian-geometry`

### 第 7 章 閉鏈運動學（Kinematics of Closed Chains）

| # | 課程 id | 課名 | 模擬（一句話） | 類型 / 模型 | bookRef | 分鐘 | 難度 |
|---|---|---|---|---|---|---|---|
| 1 | `ch07-01-five-bar-ik-fk` | 平面五連桿並聯機構：IK 容易、FK 多解（closed chain） | 兩個底座馬達驅動的平面五連桿。IK 模式：拖末端點，畫出四種分支組合（每邊 up/down）；FK 模式：拖兩個馬達角，畫出兩個 FK 解（兩圓交點）並以淡色顯示另一個裝配模式；側欄顯示 loop-closure 殘差。 | 2D 畫布 / `fiveBar` | §7.1 | 30 | core |
| 2 | `ch07-02-closed-chain-singularities` | 閉鏈的微分運動學與兩類奇異點（actuator / end-effector singularity） | 五連桿上即時顯示末端速度橢球與兩個約束矩陣的行列式曲線；「鎖馬達」按鈕鎖住兩個致動角後讓學習者用外力拖末端，若仍能微動即致動器奇異；預設按鈕可跳到兩類奇異構型；有限差分 Jacobian 作為對照。 | 2D 畫布 / `fiveBar` | §7.2–7.3 | 30 | advanced |

**ch07-01-five-bar-ik-fk** — 以兩顆底座馬達驅動的五連桿為例，體會閉鏈中致動關節與被動關節的區分，IK 可拆成兩個 2R 子問題，而 FK 需解兩圓交點且通常有多個解。  
步驟：哪些關節是馬達，哪些是被動的 → 迴路閉合方程式 → IK：拆成兩個 2R → FK：兩圓的交點 → 解會合併、會翻面 → Stewart–Gough 與下一課  
相依：`ch06-01-analytic-ik`、`ch02-01-dof-grubler`

**ch07-02-closed-chain-singularities** — 把閉鏈的速度關係寫成致動與被動兩塊的約束 Jacobian，區分「鎖住馬達末端仍能動」的致動器奇異與「某方向動不了」的末端奇異，並在五連桿上親手製造。  
步驟：把閉合方程式微分 → 末端速度橢球 → 末端奇異：有方向動不了 → 致動器奇異：鎖住馬達還能動 → 為什麼致動器奇異更危險 → 冗餘致動與下一章  
相依：`ch07-01-five-bar-ik-fk`、`ch05-02-singularities-manipulability`

### 第 8 章 開鏈動力學（Dynamics of Open Chains）

| # | 課程 id | 課名 | 模擬（一句話） | 類型 / 模型 | bookRef | 分鐘 | 難度 |
|---|---|---|---|---|---|---|---|
| 1 | `ch08-01-lagrangian-mass-matrix` | Lagrangian 動力學與質量矩陣 M(θ)（mass matrix） | 2R 擺臂自由擺動（RK4 固定步長），旁邊畫 M(θ) 的橢圓與能量條（K、U、總能）。 | 混合（畫布＋圖表／3D） / `planar2R` | §8.1 | 35 | core |
| 2 | `ch08-02-newton-euler-inverse-dynamics` | 單剛體動力學與 Newton–Euler 逆動力學（inverse dynamics） | rrr3 在 3D 場景追蹤一條預設關節軌跡（可調時長）；單步模式逐連桿高亮前向／後向遞迴的中間量（V_i、V̇_i、F_i）；下方圖表疊畫 τ_i(t)，可切換顯示慣性、速度、重力三部分的分解，並可加末端負載與開關重力。 | 混合（畫布＋圖表／3D） / `rrr3` | §8.2–8.3 | 40 | advanced |
| 3 | `ch08-03-forward-dynamics-integration` | 正動力學與數值積分：Euler vs RK4（forward dynamics） | 2R 手臂正動力學模擬，左右並排 Euler 與 RK4 兩個分身從相同初始狀態出發，下方疊畫兩者的總能量曲線；可調步長 Δt、施加開迴路 τ(t) 脈衝；爆走時自動重置並提示。 | 混合（畫布＋圖表／3D） / `planar2R` | §8.4–8.5 | 30 | core |
| 4 | `ch08-04-actuation-gearing-friction` | 致動、齒輪與摩擦（gearing, reflected inertia, friction） | 上方：單關節的馬達—減速機—連桿示意圖，可調 G、J_m、摩擦係數，顯示從馬達側與關節側看的慣量；下方：2R 正動力學加入齒輪與摩擦後的響應圖，與無齒輪情況並列，顯示耦合程度指標。 | 混合（畫布＋圖表／3D） / `planar2R` | §8.9 | 30 | core |

**ch08-01-lagrangian-mass-matrix** — 由動能與位能寫出 2R 手臂的 Lagrangian，導出 τ = M(θ)θ̈ + c(θ, θ̇) + g(θ)，把 M(θ) 讀成隨構型改變的慣量橢圓，並分辨科氏／向心項與重力項的物理來源。  
步驟：讓手臂自己擺 → 動能、位能與 Lagrangian → 質量矩陣是個橢圓 → 科氏與向心項 → 重力項 → 用能量檢查模擬 → 兩種推導，同一個答案  
相依：`ch05-01-jacobian-geometry`

**ch08-02-newton-euler-inverse-dynamics** — 先用 twist–wrench 形式寫單剛體方程，再看遞迴演算法如何前向傳遞速度與加速度、後向傳遞 wrench，沿一條軌跡產生各關節的力矩曲線並拆解成慣性、速度與重力貢獻。  
步驟：一個剛體：twist 與 wrench 的關係 → 換到別的座標系 → 前向遞迴：速度與加速度往外傳 → 後向遞迴：wrench 往回傳 → 沿軌跡的力矩曲線 → 快一點、重一點 → O(n) 與下一課  
相依：`ch08-01-lagrangian-mass-matrix`、`ch03-04-screw-motions-wrenches`

**ch08-03-forward-dynamics-integration** — 由力矩解出加速度 θ̈ = M⁻¹(τ − c − g) 並積分出運動，比較 Euler 與 RK4 在同一步長下的能量漂移，學會選步長與積分法。  
步驟：反過來：從力矩算加速度 → 用逆動力學湊出正動力學 → Euler 積分 → RK4 積分 → 步長的代價 → 開迴路力矩與下一課  
相依：`ch08-01-lagrangian-mass-matrix`

**ch08-04-actuation-gearing-friction** — 把馬達與減速機加進模型：齒輪比帶來 G²J_m 的表觀慣量使動態近似解耦，摩擦模型帶來低速黏滑，並理解高減速比對背驅動性與安全的代價。  
步驟：馬達、減速機、連桿 → 反射慣量 → 耦合變弱了 → 背驅動性與安全 → 摩擦模型 → 修正後的逆動力學與下一章  
相依：`ch08-03-forward-dynamics-integration`

### 第 9 章 軌跡生成（Trajectory Generation）

| # | 課程 id | 課名 | 模擬（一句話） | 類型 / 模型 | bookRef | 分鐘 | 難度 |
|---|---|---|---|---|---|---|---|
| 1 | `ch09-01-time-scaling` | 路徑 vs 軌跡：時間縮放 s(t)（time scaling） | 上方：單關節從 0 到 1 的動畫。下方三張同步圖：s、ṡ、s̈（可加 jerk）。 | 即時圖表 / `singleJoint` | §9.1, 9.2.2 | 25 | intro |
| 2 | `ch09-02-straight-line-paths` | 直線路徑：關節空間、螺旋與笛卡兒（joint / screw / Cartesian paths） | 3D 場景中 rrr3 從起點位姿到終點位姿，三種路徑的末端軌跡同時疊畫（不同顏色），下方圖表顯示對應的關節角與關節速度曲線；可拖起終點、放置球形障礙，並在 2D 模式切到平面 3R 觀察 IK 分支跳變。 | 混合（畫布＋圖表／3D） / `rrr3` | §9.2.1 | 30 | core |
| 3 | `ch09-03-via-points` | 經過點多項式軌跡（via points, cubic splines） | 平面上拖放經過點並設定時間戳；畫出分段三次曲線與速度箭頭，下方圖表顯示每個座標的位置、速度、加速度曲線；可選擇經過點速度策略，並可切換成 2R 手臂跟隨該路徑。 | 2D 畫布 / `pointOrPlanar2R` | §9.3 | 25 | core |
| 4 | `ch09-04-time-optimal-phase-plane` | 時間最佳時間縮放：(s, ṡ) 相平面（time-optimal time scaling） | 2R 手臂沿固定路徑運動；相平面顯示速度極限曲線與每點可行的 s̈ 區間 [L, U]（以小箭頭場表示）。 | 即時圖表 / `planar2R` | §9.4 | 40 | advanced |

**ch09-01-time-scaling** — 把軌跡拆成幾何路徑 θ(s) 與時間縮放 s(t)，比較三次、五次、梯形與 S 曲線的 ṡ、s̈ 特性，並依速度／加速度上限選擇合適的縮放。  
步驟：路徑不是軌跡 → 鏈鎖律：速度和加速度怎麼來 → 三次與五次多項式 → 梯形速度曲線 → S 曲線與 jerk → 選哪一個？以及下一課  
相依：`ch04-01-poe-space-form`

**ch09-02-straight-line-paths** — 在同一對起終點位姿上比較三種「直線」：關節空間直線、螺旋內插與解耦的笛卡兒直線，看末端軌跡與關節角曲線的差異，並判斷何時該用哪一種。  
步驟：三種「直線」 → 關節空間直線 → 螺旋內插 → 解耦的笛卡兒直線 → 笛卡兒路徑的代價 → 避障與選擇，以及經過點  
相依：`ch09-01-time-scaling`、`ch06-02-numerical-ik`、`ch03-04-screw-motions-wrenches`

**ch09-03-via-points** — 用分段三次多項式通過多個經過點並保證速度連續，比較指定經過點速度的策略（零速、啟發式、自訂），並辨識過衝與不合理的中間速度。  
步驟：不只兩個點 → 每段四個條件 → 經過點的速度該是多少 → 過衝從哪裡來 → 加速度連續嗎？ → 手臂跟著走，以及時間最佳化  
相依：`ch09-01-time-scaling`

**ch09-04-time-optimal-phase-plane** — 把致動器力矩限制轉成相平面上的可行加速度區間與速度極限曲線，理解最快軌跡的 bang-bang 結構，並用前向／後向積分找出切換點。  
步驟：沿著路徑的一維動力學 → 力矩限制變成加速度區間 → 速度極限曲線 → 最快就是 bang-bang → 找切換點 → 手繪檢查 → 假設與下一章  
相依：`ch09-01-time-scaling`、`ch08-03-forward-dynamics-integration`

### 第 10 章 運動規劃（Motion Planning）

| # | 課程 id | 課名 | 模擬（一句話） | 類型 / 模型 | bookRef | 分鐘 | 難度 |
|---|---|---|---|---|---|---|---|
| 1 | `ch10-01-cspace-obstacles` | C-space 障礙物與碰撞檢測（C-obstacles） | 左：workspace 中的 2R 手臂與可拖放／縮放的圓形障礙；右：(θ₁, θ₂) 環面展開圖即時著色 C-obstacle（離屏柵格化）。 | 2D 畫布 / `planar2R` | §10.1–10.2.2 | 30 | core |
| 2 | `ch10-02-grid-search` | 格點法：Dijkstra、A* 與 wavefront（grid search） | 可塗抹障礙的格點地圖（可載入上一課的 2R C-space 柵格）。 | 2D 畫布 / `gridRobot` | §10.2.3–10.4 | 30 | core |
| 3 | `ch10-03-sampling-rrt-prm` | 取樣式規劃：RRT 與 PRM（sampling-based planning） | 在 2D 地圖或 2R 的 C-space 環面上動畫生長 RRT / 雙向 RRT / PRM；可調步長、目標偏置機率、PRM 節點數與鄰居半徑；顯示迭代次數、路徑長與樹的 Voronoi 區域（選配）。 | 2D 畫布 / `planar2R` | §10.5 | 30 | core |
| 4 | `ch10-04-potential-fields` | 虛擬位能場與局部極小（potential fields） | 點機器人在 2D 位能場中滑動，背景顯示等高線（或熱圖）與梯度箭頭；可拖障礙物製造 U 形陷阱、調整增益與影響距離，切換一階梯度下降與加阻尼的二階動態，並可用隨機擾動嘗試逃逸。 | 2D 畫布 / `pointRobot` | §10.6 | 25 | core |

**ch10-01-cspace-obstacles** — 把工作空間裡的障礙映射成 2R 手臂的 C-space 障礙，理解規劃就是在 C-free 中找連續曲線，並認識以球體覆蓋做碰撞檢測與距離計算的做法。  
步驟：規劃問題長什麼樣 → 一個障礙在 C-space 裡的樣子 → 環面上的障礙 → 碰撞查詢與距離 → 連通性：繞得過去嗎 → 畫一條路徑，以及下一課  
相依：`ch02-02-cspace-topology`

**ch10-02-grid-search** — 把 C-space 離散成格點圖，逐步播放 Dijkstra、A* 與 wavefront 的展開順序，理解可容許啟發式與最佳性、解析度完備性與格點數爆炸。  
步驟：把空間切成格子 → Dijkstra：按代價一圈圈擴散 → A*：加上往目標的估計 → 啟發式的權重 → Wavefront：從終點倒推 → 格點的極限與下一課  
相依：`ch10-01-cspace-obstacles`

**ch10-03-sampling-rrt-prm** — 逐步執行 RRT 的 sample–nearest–steer–connect 迴圈，理解 Voronoi 偏置讓樹快速探索空曠區，比較單查詢（RRT）與多查詢（PRM）的適用場景與窄通道問題。  
步驟：不要格子，改用亂數 → RRT 的一個迭代 → 為什麼樹會往空曠處長 → 目標偏置與雙向生長 → PRM：先建路網，再回答很多問題 → 窄通道、完備性與下一課  
相依：`ch10-02-grid-search`

**ch10-04-potential-fields** — 建構吸引與排斥位能並以梯度下降導航，親手製造局部極小陷阱與震盪，認識導航函數的概念與位能場作為規劃器啟發式的用途。  
步驟：把目標當成山谷 → 把障礙當成山丘 → 局部極小的陷阱 → 增益、震盪與動態 → 導航函數 → 位能場的正確用法與下一章  
相依：`ch10-01-cspace-obstacles`

### 第 11 章 機器人控制（Robot Control）

| # | 課程 id | 課名 | 模擬（一句話） | 類型 / 模型 | bookRef | 分鐘 | 難度 |
|---|---|---|---|---|---|---|---|
| 1 | `ch11-01-error-dynamics-pid` | 誤差動態與單關節 PID（error dynamics, PID） | 單關節慣量＋重力／摩擦＋力矩飽和，PID 控制。即時曲線：θ、θ_d、τ、積分器狀態；可施加階躍、斜坡、正弦目標與擾動力矩；小圖顯示特徵根位置隨 K_p、K_d 移動；可切換速度輸入模式對照。 | 混合（畫布＋圖表／3D） / `singleJoint` | §11.1–11.3.1, 11.4.1 | 35 | core |
| 2 | `ch11-02-multijoint-computed-torque` | 多關節：PD＋重力補償 vs 計算力矩法（computed torque） | 2R 手臂追蹤一條可調速度的關節軌跡（或任務空間直線），三種控制器可切換（PD / PD+g / computed torque），疊畫追蹤誤差曲線與力矩中前饋／回饋的比例；模型質量可故意加入 ±20% 誤差，並可切換任務空間控制觀察奇異附近的行為。 | 混合（畫布＋圖表／3D） / `planar2R` | §11.4.2–11.4.3 | 35 | core |
| 3 | `ch11-03-force-hybrid-control` | 力控制與混合運動—力控制（force control, hybrid motion–force） | 2R 手臂末端接觸一面可調剛度的桌面（單邊彈簧—阻尼接觸模型）：沿切線追蹤位置、沿法線追蹤力。 | 混合（畫布＋圖表／3D） / `planar2R` | §11.5–11.6 | 35 | advanced |
| 4 | `ch11-04-impedance-admittance` | 阻抗控制與導納控制（impedance / admittance） | 2R 末端（或 1-DOF 簡化模型）對一面牆：使用者拖動目標位置或對末端施加外力；可切換阻抗／導納模式，調整虛擬 M、B、K、牆面剛度與感測器延遲，曲線顯示位置、接觸力與能量，觀察彈回、穿牆與震盪。 | 混合（畫布＋圖表／3D） / `planar2R` | §11.7 | 30 | advanced |

**ch11-01-error-dynamics-pid** — 把閉迴路行為寫成誤差的線性微分方程，用阻尼比與自然頻率預測超調與安定時間，觀察 I 項消除穩態誤差的代價與積分飽和。  
步驟：控制就是讓誤差消失 → 速度輸入：一階誤差動態 → 力矩輸入：二階誤差動態 → 擾動與穩態誤差 → 加上積分項 → 飽和與積分飽和 → 多關節會怎樣？  
相依：`ch08-03-forward-dynamics-integration`

**ch11-02-multijoint-computed-torque** — 看關節耦合如何讓單關節 PID 在多關節手臂失效，用重力補償消除靜態誤差、用計算力矩法把整個系統線性化並解耦，再評估模型誤差的影響與任務空間控制的差異。  
步驟：兩個獨立 PID 的下場 → 補上重力 → 計算力矩：先抵消，再當線性系統控制 → 前饋與回饋各做什麼 → 模型不準怎麼辦 → 任務空間控制與下一課  
相依：`ch11-01-error-dynamics-pid`、`ch08-01-lagrangian-mass-matrix`

**ch11-03-force-hybrid-control** — 理解純力控制需要重力補償與阻尼，把環境約束寫成 A(θ)V = 0 分離運動與力的子空間，並用投影矩陣設計擦桌子任務的混合控制器。  
步驟：推桌子：位置控制會出什麼事 → 純力控制 → 自然約束與人工約束 → 投影矩陣 P → 擦桌子：混合控制器 → 矛盾的指令與下一課  
相依：`ch11-02-multijoint-computed-torque`、`ch05-03-statics-force-ellipsoid`

**ch11-04-impedance-admittance** — 把機器人末端設計成虛擬的質量—彈簧—阻尼器，分辨阻抗控制（量運動、出力）與導納控制（量力、出運動），並觀察自由空間與硬接觸時各自的優缺點。  
步驟：把手臂變成彈簧 → 阻抗控制：量運動、出力 → 導納控制：量力、出運動 → 碰牆 → 調 M、B、K → 選哪一種？與本章總結  
相依：`ch11-03-force-hybrid-control`

### 第 12 章 抓取與操作（Grasping and Manipulation）

| # | 課程 id | 課名 | 模擬（一句話） | 類型 / 模型 | bookRef | 分鐘 | 難度 |
|---|---|---|---|---|---|---|---|
| 1 | `ch12-01-contact-kinematics-form-closure` | 接觸運動學與形封閉（contact kinematics, form closure） | 平面多邊形物體與可拖放的無摩擦點接觸。每個接觸在旋轉中心平面畫出 ± 區域，交集顯示可行運動；拖物體嘗試移動時違規會變紅；接觸數足夠且法線配置合適時標示形封閉；「推一下」模式顯示接觸的滾動／滑動／脫離分類。 | 2D 畫布 / `planarObject` | §12.1 | 30 | core |
| 2 | `ch12-02-friction-force-closure` | 摩擦錐與力封閉（friction cone, force closure） | 平面物體上放置帶摩擦的指尖，顯示摩擦錐；程式計算所有接觸 wrench 的正錐是否張滿整個平面 wrench 空間（線性規劃或凸包含原點檢查）。 | 2D 畫布 / `planarObject` | §12.2 | 30 | core |

**ch12-01-contact-kinematics-form-closure** — 用接觸法線做一階分析判斷滾動、滑動或脫離，在平面用旋轉中心圖法找出可行運動，並判斷一組無摩擦點接觸是否達成形封閉。  
步驟：一個接觸能做什麼 → 一階分析：內積的符號 → 旋轉中心圖法 → 多個接觸：交集 → 形封閉 → 只推不拉，以及摩擦  
相依：`ch03-04-screw-motions-wrenches`、`ch02-03-constraints-task-space`

**ch12-02-friction-force-closure** — 用摩擦係數建構摩擦錐並以邊緣向量近似，判斷兩指抓取是否力封閉（連線落在兩個錐內），並理解力與運動自由度的對偶關係。  
步驟：有摩擦的接觸 → 兩指抓取的判據 → 多指：wrench 錐是否張滿 → 閉合不等於抓得住 → 力與運動的對偶 → 回顧與下一章  
相依：`ch12-01-contact-kinematics-form-closure`、`ch05-03-statics-force-ellipsoid`

### 第 13 章 輪式移動機器人（Wheeled Mobile Robots）

| # | 課程 id | 課名 | 模擬（一句話） | 類型 / 模型 | bookRef | 分鐘 | 難度 |
|---|---|---|---|---|---|---|---|
| 1 | `ch13-01-omnidirectional-kinematics` | 全向移動機器人運動學（omniwheel / mecanum） | 俯視的三輪 omni 或四輪 mecanum 底盤：以虛擬搖桿給 V_b = (ω, v_x, v_y)，顯示每顆輪子的轉速與滾子方向，旁圖畫出可行速度多面體（輪速上限下的 (v_x, v_y) 切片）；也可反向只驅動一顆輪子看底盤怎麼動。 | 2D 畫布 / `omniBase` | §13.1–13.2.1 | 30 | core |
| 2 | `ch13-02-nonholonomic-odometry` | 非完整機器人：差速、車型與里程計（nonholonomic, odometry） | 俯視地圖，鍵盤駕駛差速或車型機器人；真實軌跡與里程計估計軌跡並列，輪子加入滑移雜訊，曲線圖顯示位置與角度誤差隨距離累積。 | 混合（畫布＋圖表／3D） / `diffDrive` | §13.3.1–13.3.2, 13.4 | 35 | core |
| 3 | `ch13-03-tracking-mobile-manipulation` | 軌跡追蹤回授控制與移動操作（trajectory tracking, mobile manipulation） | 差速（或全向）機器人追蹤參考軌跡（虛線），可加初始誤差與擾動，曲線顯示機器人座標下的 (φ_e, x_e, y_e)；第二頁：底盤＋2R 手臂讓末端沿直線走，顯示 J_base 與 J_arm 的貢獻，並可選擇冗餘解法讓手臂遠離奇異。 | 混合（畫布＋圖表／3D） / `diffDriveArm` | §13.2.3, 13.3.3–13.3.4, 13.5 | 40 | advanced |

**ch13-01-omnidirectional-kinematics** — 由輪子幾何建立底盤 twist 到輪速的關係 u = H(0)V_b，理解輪速限制如何變成底盤速度的多面體，並比較三輪 omni 與四輪 mecanum。  
步驟：全向底盤能做什麼 → 一顆輪子的一列 → u = H(0) V_b → 輪速上限 → 速度多面體 → 四輪的不一致 → 積分到世界座標，與下一課  
相依：`ch02-03-constraints-task-space`、`ch03-03-transforms-twists`

**ch13-02-nonholonomic-odometry** — 寫出單輪車、差速與車型機器人的控制系統 q̇ = G(q)u，用 Lie bracket 的直覺理解側滑方向為何仍可達（可控性），並由輪子編碼器積分里程計觀察漂移。  
步驟：三種底盤，一種限制 → q̇ = G(q) u → 側向可達：四段序列 → 停車挑戰 → 里程計 → 漂移與下一課  
相依：`ch13-01-omnidirectional-kinematics`

**ch13-03-tracking-mobile-manipulation** — 理解為何直線插值對車型機器人不可行而需要可行參考軌跡，設計前饋＋回授的追蹤控制律並觀察收斂，最後把底盤與手臂合併成移動操作的 Jacobian。  
步驟：參考軌跡要可行 → 只有前饋會怎樣 → 在機器人座標定義誤差 → 前饋＋回授的追蹤律 → 全向底盤的追蹤 → 移動操作：底盤加手臂 → 課程總結  
相依：`ch13-02-nonholonomic-odometry`、`ch11-01-error-dynamics-pid`、`ch05-01-jacobian-geometry`


## 學習路徑

- **核心路徑**（intro + core）：`ch02-01-dof-grubler` → `ch02-02-cspace-topology` → `ch02-03-constraints-task-space` → `ch03-01-rotation-matrices` → `ch03-02-exponential-coordinates` → `ch03-03-transforms-twists` → `ch03-04-screw-motions-wrenches` → `ch04-01-poe-space-form` → `ch04-02-poe-body-form-urdf` → `ch05-01-jacobian-geometry` → `ch05-02-singularities-manipulability` → `ch05-03-statics-force-ellipsoid` → `ch06-01-analytic-ik` → `ch07-01-five-bar-ik-fk` → `ch08-01-lagrangian-mass-matrix` → `ch08-03-forward-dynamics-integration` → `ch08-04-actuation-gearing-friction` → `ch09-01-time-scaling` → `ch09-02-straight-line-paths` → `ch09-03-via-points` → `ch10-01-cspace-obstacles` → `ch10-02-grid-search` → `ch10-03-sampling-rrt-prm` → `ch10-04-potential-fields` → `ch11-01-error-dynamics-pid` → `ch11-02-multijoint-computed-torque` → `ch12-01-contact-kinematics-form-closure` → `ch12-02-friction-force-closure` → `ch13-01-omnidirectional-kinematics` → `ch13-02-nonholonomic-odometry`
- **進階延伸**（advanced）：`ch06-02-numerical-ik`、`ch07-02-closed-chain-singularities`、`ch08-02-newton-euler-inverse-dynamics`、`ch09-04-time-optimal-phase-plane`、`ch11-03-force-hybrid-control`、`ch11-04-impedance-admittance`、`ch13-03-tracking-mobile-manipulation`


## 共用模擬元件（sharedComponents）

`exists` 表示 `src/ui` 或 `src/mr` 已提供；`new` 為建議由工具組新增，實作前請先確認最新的 `src/ui/index.ts`。

| 元件 | 狀態 | 位置／建議位置 | 用途 | 使用課程 |
|---|---|---|---|---|
| `planarArmRenderer` | exists | @/ui robot2d: drawPlanarArm / planarArmPoints / drawTrail | 平面 2R/3R 手臂繪製（座標系、角度弧、半透明 ghost、尾跡） | ch02-02, ch05-01, ch05-02, ch05-03, ch06-01, ch06-02, ch08-01, ch08-03, ch08-04, ch09-02, ch09-03, ch10-01, ch11-02, ch11-03, ch11-04, ch13-03 |
| `frameTriad` | exists | Canvas2D.frame / Scene3D.addFrame(...).setTransform | 2D/3D 座標系三軸（含標籤、可調大小） | ch02-03, ch03-01, ch03-02, ch03-03, ch03-04, ch04-01, ch04-02, ch13-01, ch13-02 |
| `robot3dStick` | exists | @/ui robot3d: Robot3D / homeJointPoints | 任意 PoE 鏈的 3D 桿狀機器人（關節座標系、透明度） | ch04-01, ch04-02, ch05-01, ch08-02, ch09-02 |
| `screwAxisGizmo` | exists | Scene3D.addScrewAxis(...).setScrew | 3D 螺旋軸（方向、通過點、節距標示） | ch03-03, ch03-04, ch04-01, ch05-01, ch09-02 |
| `matrixViewHighlight` | exists | @/ui controls: matrixView(...).set(M,{highlightCols}) | 矩陣面板與欄／區塊高亮 | ch03-01, ch03-02, ch03-03, ch03-04, ch04-01, ch04-02, ch05-01, ch07-02, ch13-01, ch13-02 |
| `ellipseDrawer` | exists | @/ui robot2d: drawEllipse2x2 / eig2x2Sym | 由 2×2 對稱矩陣畫橢圓（速度／力／質量橢圓，inverse 選項） | ch05-02, ch05-03, ch07-02, ch08-01 |
| `playbackScrubber` | exists | @/ui controls: playback / createLoop | 播放、暫停、速度、拖曳時間 | ch03-02, ch03-04, ch08-02, ch09-01, ch09-02, ch09-03, ch09-04 |
| `syncedPlots` | new | 建議 src/ui/plotGroup.ts | 多張 Plot2D 共享 x 範圍與時間游標（s/ṡ/s̈、θ/τ/誤差疊圖） | ch08-02, ch09-01, ch09-03, ch11-01, ch11-02, ch11-03, ch11-04, ch13-02, ch13-03 |
| `physicsRunner` | new | 建議 src/ui/physicsRunner.ts（包 createLoop + substep + mr.rk4Step） | 固定子步 RK4 積分器、能量讀值、爆走保護與重置 | ch02-03, ch08-01, ch08-03, ch08-04, ch11-01, ch11-02, ch11-03, ch11-04, ch13-02, ch13-03 |
| `gainPanel` | new | 建議 src/ui/gainPanel.ts（配 mr.createPID / pidStep） | K_p/K_i/K_d（或 M/B/K）滑桿組、飽和與抗飽和開關、特徵根小圖 | ch11-01, ch11-02, ch11-03, ch11-04, ch13-03 |
| `jointSliderBank` | new | 建議 src/ui/jointSliders.ts | 依機器人模型自動產生 n 個 angleSlider，含預設姿態、單關節脈衝與隨機按鈕 | ch04-01, ch04-02, ch05-01, ch05-02, ch05-03, ch08-02, ch09-02 |
| `algorithmStepper` | new | 建議 src/ui/stepper.ts | 單步／自動／重置／速度與迭代計數，給迭代式演算法（Newton IK、A*、RRT、NE 遞迴、相平面積分） | ch06-02, ch08-02, ch09-04, ch10-02, ch10-03 |
| `torusMap` | new | 建議 src/ui/torusMap.ts | (θ₁,θ₂) 方格的 wrap-aware 繪圖：可拖點、畫路徑、柵格覆蓋、邊界黏合提示 | ch02-02, ch10-01, ch10-03 |
| `rasterLayer` | new | 建議 src/ui/raster.ts | 離屏柵格化一個 2D 謂詞／純量場並快取成熱圖（C-obstacle、σ_min、位能場） | ch05-02, ch10-01, ch10-04 |
| `collision2D` | new | 建議 src/mr/geometry2d.ts（純函式＋測試） | 線段—圓、圓—圓、點—多邊形、線段取樣碰撞與最短距離 | ch09-02, ch10-01, ch10-02, ch10-03 |
| `gridMapEditor` | new | 建議 src/ui/gridMap.ts | 塗抹障礙、設定起終點、解析度與連通性；輸出占據格 | ch10-02, ch10-03, ch10-04 |
| `draggablePose2D` | new | 建議 src/ui/poseHandle.ts | SE(2) 手把（位置＋朝向旋鈕），用於目標位姿 | ch02-03, ch06-02, ch09-02, ch13-02, ch13-03 |
| `wheeledBaseRenderer` | new | 建議 src/ui/wheeledBase.ts | 俯視底盤（unicycle / diff / car / omni / mecanum）、輪速指示、尾跡、{s}/{b} | ch02-03, ch13-01, ch13-02, ch13-03 |
| `keyboardDrive` | new | 建議 src/ui/keyboard.ts | WASD／方向鍵輸入（含 destroy 清理與焦點處理） | ch02-03, ch13-01, ch13-02, ch13-03 |
| `contactPolygonEditor` | new | 建議 src/lessons/ch12-*/shared 或 src/ui/contacts.ts | 多邊形物體、貼邊接觸點與法線、摩擦錐、旋轉中心區域、wrench 空間凸包 | ch12-01, ch12-02 |
| `wallContactModel` | new | 建議 src/mr/contact.ts（純函式＋測試） | 單邊彈簧—阻尼接觸（桌面／牆），支援子步與延遲 | ch11-03, ch11-04 |
| `torqueBars` | new | 建議 src/ui/bars.ts | 每關節帶符號、可夾制的力矩條／弧；亦用於能量條（K/U/總能） | ch05-03, ch08-01, ch08-02, ch08-03, ch08-04, ch11-02 |
| `presetChips` | new | 建議 src/ui/presets.ts | 一排預設按鈕，可由 onStep 觸發，顯示目前啟用的預設 | ch02-01, ch03-02, ch04-01, ch05-02, ch07-02, ch09-02, ch10-03, ch13-03 |
| `galleryCard` | new | 建議 src/ui/card.ts | 小型原創示意卡（inline SVG＋說明），用於「下一課預告」與概念對照 | ch02-02, ch02-03, ch06-01, ch07-01, ch10-02, ch10-04, ch11-01, ch12-02, ch13-01 |
| `linkageSolver` | new (lesson-local) | src/lessons/ch02-01-dof-grubler/linkage.ts | 平面連桿迴路閉合求解（小步＋warm start）與 Grübler 計數 | ch02-01 |
| `fiveBarKinematics` | new (chapter-local) | src/lessons/ch07-01-five-bar-ik-fk/fivebar.ts（ch07-02 複製或共享） | 五連桿 IK（兩個 2R）、FK（兩圓交點）、約束 Jacobian H_a/H_p | ch07-01, ch07-02 |


## 跨章節慣例

- 同一組「house」機器人模型貫穿全課程：planar2R（L=1,1）、planar3R、rrr3、ur5（皆在 src/mr/robots.ts），其他模型（fiveBar、unicycle、diffDrive、omniBase、planarObject、singleJoint）為該章專用。
- 顏色：x 紅 / y 綠 / z 藍；{s} 灰、{b} 橘；twist 藍、wrench 紅、奇異構型洋紅；2D 畫布與 Three.js 一致，透過 cssVar 讀取主題變數。
- 「前乘＝固定座標系、後乘＝物體座標系」是貫穿規則：ch03-01 引入，ch03-03、ch04-02、ch09-02 明確回扣。
- 反覆出現的迷思各課以一行提醒回扣：『R_ab p 不是轉動 p 而是換座標』、『twist 的 v 不是物體原點速度』、『奇異是構型而非位置的性質』。
- 每個模擬都要回應 onStep：步驟負責把模擬設成該步談論的狀態（預設組態、顯示／隱藏覆蓋層、鎖定控制項），不要求學習者自己重現設定；但不隨意清除學習者的探索。
- 每個模擬至少顯示一個可以手算驗證的數值（例如 det R、‖T_space−T_body‖、Grübler 結果、功率 VᵀF）。
- 數值衛生在課內點名：atan2 而非 atan、θ≈π 的 log 分支、接近奇異的偽逆／DLS、庫侖摩擦死區、Euler 能量漂移、角度包裹、固定子步 RK4。
- 核心數學一律呼叫 src/mr（exp/log、Adjoint、PoE、Jacobian、IK、Newton–Euler、RK4、時間縮放、控制），課程不自行重新實作；課程專用演算法（A*、RRT、五連桿、位能場、接觸分析）留在課程資料夾並附測試。
- 測驗規則：每課至少一題遷移題使用模擬中沒出現過的機構或情境；解析要說明「為什麼」，不只標對錯。
- 節奏：前 1–2 步用具體失敗或現象引起動機，中段建立正式工具，最後一步接到下一課或測驗。
- 難度標籤：intro / core 組成「核心路徑」，advanced 為延伸；側欄可依此過濾。
- 文字：台灣繁體中文，英文專有名詞首次出現時以括號保留（例：旋量（twist））；LaTeX 使用 R, p, T, S, V, F, J, M 等標準符號；程式註解英文。
- 版權：教材僅作為概念與章節編號的參考（bookRef 指向 v2 章節）；所有敘述、題目、圖說皆為原創，不得抄錄或近似改寫。


## 與基線草案的差異（設計決策）

- 第 3 章壓縮為 4 課：平面剛體運動併入 `ch03-01` 的前兩步（俯視平面模式），wrench 併入 `ch03-04`（螺旋運動＋力旋量）；每課仍維持單一主題的模擬。
- 第 8 章拆出 `ch08-04`（致動、齒輪、摩擦，§8.9），因為反射慣量與背驅動性是第 11 章阻抗／導納選型的前置知識。
- 第 9 章 bookRef 依 TOC 修正：時間縮放對應 §9.1, 9.2.2；三種直線路徑對應 §9.2.1；經過點 §9.3；時間最佳 §9.4。
- 第 13 章第 3 課同時涵蓋全向底盤的回授控制（§13.2.3）、非完整底盤的規劃與追蹤（§13.3.3–13.3.4）與移動操作（§13.5）。
- 第 12 章保留兩課：接觸運動學＋形封閉（§12.1）、摩擦錐＋力封閉（§12.2）；提案 B 的「接觸運動學」獨立課以「推一下」模式併入 `ch12-01`。
- 所有課程的 `dependsOn` 只向前指（已由建置腳本驗證無反向相依）。
