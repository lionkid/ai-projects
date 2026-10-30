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
