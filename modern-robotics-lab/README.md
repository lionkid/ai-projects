# Modern Robotics Lab｜互動式機器人學實驗室

用網頁互動與模擬，一步一步理解機器人力學、規劃與控制。教材對照
*Modern Robotics: Mechanics, Planning, and Control*（Kevin M. Lynch & Frank C. Park，v2 線上版：
<https://modernrobotics.org>）。本站所有文字為獨立撰寫的導讀，**不是教材摘錄**；請搭配原書閱讀。

## 特色

- **每課三段式**：分步導讀（Markdown + KaTeX 公式）→ 互動模擬（2D 畫布 / Three.js 3D / 即時圖表）→ 自我檢測。
- **模擬跟著步驟走**：切換步驟時模擬會改變焦點或預設組態，幫助把公式對應到畫面。
- **章節與教材一致**（Ch 2–13），每課標示對應的教材章節（例如 §3.2.3）。
- **內建機器人數學函式庫** `src/mr`：以 TypeScript 重新實作，函式命名對齊 Modern Robotics 開源程式庫
  （`MatrixExp6`、`FKinSpace`、`JacobianBody`、`InverseDynamics`、`ComputedTorque` …），附單元測試。
- 學習進度與測驗結果保存在瀏覽器 `localStorage`；支援深色 / 淺色主題與手機版面。
- 純靜態網站，`npm run build` 後的 `dist/` 可放在任何靜態主機（GitHub Pages、S3 …）。

<!-- lessons:start -->
## 課程列表

共 39 課（2 課導覽 + 第 2–13 章 37 課）。每課都經過實作與獨立稽核（數學、物理、內容、介面與程式碼）。

| 章 | 課程 | 教材章節 |
|---|---|---|
| 0 開始之前 | 如何使用這個實驗室 | Ch. 1（導論） |
| 0 開始之前 | 實驗室裡的機器人模型 | Ch. 1、§4.1（預覽） |
| 2 組態空間 | 自由度與 Grübler 公式（degrees of freedom） | §2.1–2.2 |
| 2 組態空間 | C-space 的拓撲與表示法（topology & representation） | §2.3 |
| 2 組態空間 | 完整／非完整約束、任務空間與工作空間（holonomic, Pfaffian） | §2.4–2.5 |
| 3 剛體運動 | 剛體位姿與旋轉矩陣（rotation matrix, SO(3)） | §3.1, 3.2.1 |
| 3 剛體運動 | 角速度與旋轉的指數座標（exponential coordinates, Rodrigues） | §3.2.2–3.2.3 |
| 3 剛體運動 | 齊次變換、旋量（twist）與 Adjoint | §3.3.1–3.3.2 |
| 3 剛體運動 | 螺旋運動的指數座標與力旋量（screw motion, wrench） | §3.3.3–3.4 |
| 4 正向運動學 | 指數積公式的空間形式（PoE, space form） | §4.1.1–4.1.2 |
| 4 正向運動學 | PoE 物體形式、UR5 範例與 URDF（body form） | §4.1.3, 4.2 |
| 5 速度運動學與靜力學 | 空間與物體 Jacobian 的幾何意義（space / body Jacobian） | §5.1.1–5.1.4 |
| 5 速度運動學與靜力學 | 奇異點與可操作度橢球（singularity, manipulability ellipsoid） | §5.3–5.4 |
| 5 速度運動學與靜力學 | 靜力學 τ = Jᵀ F 與力橢球（statics, force ellipsoid） | §5.2, 5.4 |
| 6 逆向運動學 | 解析逆運動學：2R 手臂的兩組解（analytic IK） | §6.1 |
| 6 逆向運動學 | 數值逆運動學：Newton–Raphson 與阻尼最小平方（numerical IK, DLS） | §6.2–6.3 |
| 7 閉鏈運動學 | 平面五連桿並聯機構：IK 容易、FK 多解（closed chain） | §7.1 |
| 7 閉鏈運動學 | 閉鏈的微分運動學與兩類奇異點（actuator / end-effector singularity） | §7.2–7.3 |
| 8 開鏈動力學 | Lagrangian 動力學與質量矩陣 M(θ)（mass matrix） | §8.1 |
| 8 開鏈動力學 | 單剛體動力學與 Newton–Euler 逆動力學（inverse dynamics） | §8.2–8.3 |
| 8 開鏈動力學 | 正動力學與數值積分：Euler vs RK4（forward dynamics） | §8.4–8.5 |
| 8 開鏈動力學 | 致動、齒輪與摩擦（gearing, reflected inertia, friction） | §8.9 |
| 9 軌跡生成 | 路徑 vs 軌跡：時間縮放 s(t)（time scaling） | §9.1, 9.2.2 |
| 9 軌跡生成 | 直線路徑：關節空間、螺旋與笛卡兒（joint / screw / Cartesian paths） | §9.2.1 |
| 9 軌跡生成 | 經過點多項式軌跡（via points, cubic splines） | §9.3 |
| 9 軌跡生成 | 時間最佳時間縮放：(s, ṡ) 相平面（time-optimal time scaling） | §9.4 |
| 10 運動規劃 | C-space 障礙物與碰撞檢測（C-obstacles） | §10.1–10.2.2 |
| 10 運動規劃 | 格點法：Dijkstra、A* 與 wavefront（grid search） | §10.2.3–10.4 |
| 10 運動規劃 | 取樣式規劃：RRT 與 PRM（sampling-based planning） | §10.5 |
| 10 運動規劃 | 虛擬位能場與局部極小（potential fields） | §10.6 |
| 11 機器人控制 | 誤差動態與單關節 PID（error dynamics, PID） | §11.1–11.3.1, 11.4.1 |
| 11 機器人控制 | 多關節：PD＋重力補償 vs 計算力矩法（computed torque） | §11.4.2–11.4.3 |
| 11 機器人控制 | 力控制與混合運動—力控制（force control, hybrid motion–force） | §11.5–11.6 |
| 11 機器人控制 | 阻抗控制與導納控制（impedance / admittance） | §11.7 |
| 12 抓取與操作 | 接觸運動學與形封閉（contact kinematics, form closure） | §12.1 |
| 12 抓取與操作 | 摩擦錐與力封閉（friction cone, force closure） | §12.2 |
| 13 輪式移動機器人 | 全向移動機器人運動學（omniwheel / mecanum） | §13.1–13.2.1 |
| 13 輪式移動機器人 | 非完整機器人：差速、車型與里程計（nonholonomic, odometry） | §13.3.1–13.3.2, 13.4 |
| 13 輪式移動機器人 | 軌跡追蹤回授控制與移動操作（trajectory tracking, mobile manipulation） | §13.2.3, 13.3.3–13.3.4, 13.5 |
<!-- lessons:end -->

## 快速開始

```bash
cd modern-robotics-lab
npm install
npm run dev        # http://localhost:5173
```

其他指令：

| 指令 | 說明 |
|---|---|
| `npm run build` | 產生靜態網站到 `dist/`（`base: './'`，可放在任意子路徑） |
| `npm run preview` | 預覽 build 結果 |
| `npm run typecheck` | TypeScript 型別檢查 |
| `npm test` | 執行 vitest（數學函式庫與課程邏輯測試） |
| `npm run smoke` | 以 headless Chromium 開啟每一課、走完所有步驟、操作所有控制項並檢查錯誤（需要 Playwright） |
| `npm run check` | typecheck + test + build |
| `node scripts/smoke.mjs --lesson <id> --screens <dir> --step-shots` | 只測指定課程，並存下每一步的第一個畫面截圖；加 `--dark`、`--mobile` 檢查深色與手機版面 |

> `smoke` 需要可解析的 `playwright` 套件與 Chromium。本機使用：`npm i -D playwright && npx playwright install chromium`。

## 專案結構

```
modern-robotics-lab/
├─ index.html
├─ src/
│  ├─ main.ts            # 進入點：主題、shell、路由
│  ├─ types.ts           # Lesson / LessonMeta / Quiz 等共用契約
│  ├─ app/               # shell（側欄）、首頁、課程頁、路由、進度儲存、主題
│  ├─ ui/                # 工具組：DOM、Markdown+KaTeX、控制元件、Canvas2D、Plot2D、Scene3D、loop
│  ├─ mr/                # 機器人數學函式庫（SO(3)/SE(3)、PoE、Jacobian、IK、動力學、軌跡、控制）+ 測試
│  ├─ content/           # 章節資訊、教材連結
│  └─ lessons/<id>/      # 每課一個目錄：meta.ts（導覽用）+ index.ts（步驟、測驗、模擬）
├─ scripts/smoke.mjs     # Playwright 煙霧測試
└─ docs/
   ├─ curriculum.md/.json   # 課程地圖與每課設計規格
   └─ lesson-authoring.md   # 新增課程的規範與工具組說明
```

## 新增一課

1. 建立 `src/lessons/ch<CC>-<NN>-<slug>/meta.ts` 與 `index.ts`（參考 `src/lessons/ch00-01-how-to-use/`）。
2. 不需要修改任何共用檔案：`src/app/registry.ts` 會用 `import.meta.glob` 自動發現。
3. 依 `docs/lesson-authoring.md` 的規範撰寫步驟、模擬與測驗。
4. `npm run typecheck && node scripts/smoke.mjs --lesson <id>`。

## 慣例

- 座標軸顏色：x 紅、y 綠、z 藍；3D 場景 z 軸朝上；內部一律使用弧度。
- 座標系以 `{s}`（space frame）、`{b}`（body frame）標記；矩陣為列主序 `number[][]`；螺旋軸為 `[ω; v]`。
- 介面與導讀文字使用台灣繁體中文並保留英文專有名詞；程式註解使用英文。

## 授權與致謝

- 本專案程式碼與導讀文字為原創內容。
- 教材 *Modern Robotics* © Lynch & Park / Cambridge University Press；本站僅引用章節編號與概念，不含教材內文。
- 數學函式庫的函式命名參考 MIT 授權的 [ModernRobotics](https://github.com/NxRLab/ModernRobotics) 程式庫，實作為獨立撰寫。
