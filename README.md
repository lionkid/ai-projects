# ai-projects

這個 repo 收集我和 AI coding agent（Claude）一起開發的個人專案。每個子目錄都是獨立專案，各有自己的 README、相依套件和建置指令，彼此互不影響。

## 專案列表

| 專案 | 簡介 | 技術 | 線上閱讀 |
|---|---|---|---|
| [modern-robotics-lab](modern-robotics-lab/) | 互動式機器人學實驗室。共 39 課，每課包含分步導讀、互動模擬與自我檢測，對照 *Modern Robotics*（Lynch & Park）第 2–13 章 | TypeScript、Vite、Three.js、KaTeX | [GitHub Pages](https://lionkid.github.io/ai-projects/modern-robotics-lab/) |

### Modern Robotics Lab

從組態空間、剛體運動、正／逆向運動學、動力學、軌跡生成、運動規劃，一路學到控制、抓取與輪式移動機器人。每個概念都能在 2D 畫布或 3D 場景裡直接拖拉參數，觀察公式算出的結果。

- 內建 `src/mr` 數學函式庫：用 TypeScript 重新實作，函式命名對齊 Modern Robotics 開源程式庫（`MatrixExp6`、`FKinSpace`、`JacobianBody`、`InverseDynamics` …），並附有單元測試。
- 純靜態網站，學習進度存在瀏覽器的 `localStorage`，支援深色／淺色主題與手機版面。
- 詳細說明、完整課程列表與開發規範請見 [modern-robotics-lab/README.md](modern-robotics-lab/README.md)。

## 本機執行

需要 Node.js 20 以上版本（開發環境使用 Node 24）。

```bash
cd modern-robotics-lab
npm ci
npm run dev        # 開發伺服器：http://localhost:5173
npm run check      # typecheck + test + build，輸出到 dist/
npm run preview    # 預覽 build 結果
```

## 線上部署（GitHub Pages）

推送到預設分支時，[`.github/workflows/deploy-pages.yml`](.github/workflows/deploy-pages.yml) 會自動執行以下步驟：

1. 在 `modern-robotics-lab/` 執行 `npm ci` 和 `npm run check`。只要型別檢查或測試失敗，就不會部署。
2. 把 `dist/` 放到網站的 `/modern-robotics-lab/` 路徑，並在網站根目錄產生一個導向頁。
3. 用 GitHub Pages 發布，網址是 <https://lionkid.github.io/ai-projects/>。

也可以到 Actions 頁籤找 **Deploy to GitHub Pages**，用 **Run workflow** 手動觸發。

首次使用需要啟用：repo 的 **Settings → Pages → Build and deployment → Source** 選 **GitHub Actions**。

> 注意：GitHub Pages 發布的網站一律公開，即使 repo 是 private 也一樣。private repo 要使用 Pages，帳號必須是 GitHub Pro 以上的方案。

## 目錄結構

```
ai-projects/
├─ README.md                      # 本文件
├─ .github/workflows/
│  └─ deploy-pages.yml            # build 並部署到 GitHub Pages
└─ modern-robotics-lab/           # 互動式機器人學實驗室（Vite + TypeScript）
```

## 新增專案

1. 在根目錄建立新的子目錄，放入該專案自己的 `README.md`。
2. 把專案加到上方的「專案列表」。
3. 如果是要上線的靜態網站，在 `deploy-pages.yml` 加上對應的 build 步驟，把輸出複製到 `_site/<專案名稱>/`，並更新根目錄導向頁的連結。
