# generator：機器人模型產生程式（各版本共用）

讀取 `../robot_v<N>/params.yaml`，產生該版的列印零件、URDF 與檢查報告（第 5、6 步的 USD 轉檔與模擬檢查在 `simulation/robot_assets/`）。程式所有版本共用；
每版產生時把 git commit 記在 `robot_v<N>/reports/mass_properties.json`，打 tag `robot-v<N>` 後在該版資料夾放一個 `FROZEN` 檔，
之後程式拒絕重新產生該版（要改就開新版本）。

## 執行順序（專案根目錄）

| 步驟 | 指令 | 環境 | 產出 |
|---|---|---|---|
| 1. 零件 | `.venv_cad\Scripts\python.exe hardware\generator\build_parts.py --version 1` | CAD（build123d） | `parts/step`、`parts/stl`、`parts/assembly_v<N>.step`（整機）、`meshes/`、`reports/mass_properties.json`、`parts_bom.md`、`interference.md`、`fasteners.md`、`servo_map.md` |
| 2. URDF | `.venv_cad\Scripts\python.exe hardware\generator\build_urdf.py --version 1` | CAD | `urdf/custom_biped_v1.urdf` |
| 3. 靜態檢查 | `.venv_cad\Scripts\python.exe hardware\generator\check_static.py --version 1` | CAD | `reports/static_check.md`、`.json`（站姿、扭矩、可達範圍、尾巴） |
| 4. URDF 檢查 | `tools\isaacpy.cmd tools\urdf_inspect.py hardware\robot_v1\urdf\custom_biped_v1.urdf` | Isaac | 終端輸出（要求 0 warning） |
| 5. USD | `tools\isaacpy.cmd simulation\robot_assets\convert_usd.py --version 1` | Isaac | `simulation/robot_assets/usd/custom_biped_v1/`（不進 git） |
| 6. 模擬檢查 | `tools\isaacpy.cmd simulation\robot_assets\check_sim.py --version 1 --enable_cameras` | Isaac | `outputs/robot_v1/simulation/robot_check/<時間>/`（report.json、截圖） |
| 7. 互動圖 | `.venv_cad\Scripts\python.exe hardware\generator\build_viewer.py --version 1` | CAD | `robot_v1/viewer.html`（本機直接開）＋ `outputs/robot_v1/hardware/viewer/`（發佈 Artifact 用） |

輸出中文時，CAD 環境要先設定 `set PYTHONIOENCODING=utf-8`（PowerShell：`$env:PYTHONIOENCODING="utf-8"`）。
CAD 環境的建立方式見根目錄 `CLAUDE.md`「工作準則」（uv 裝在 `tools/bin/`，環境在 `.venv_cad/`）。

## 檔案

| 檔案 | 內容 |
|---|---|
| `params.py` | 讀取 params.yaml，推導零位時每個關節的位置（純 Python，不需 CAD） |
| `parts.py` | 所有列印件、馬達、外購件的 build123d 幾何（零位、世界座標）；設計原則寫在檔案開頭 |
| `build_parts.py` | 匯出列印檔與每個連桿的外觀 mesh、計算質量性質、關節干涉掃描 |
| `build_urdf.py` | 產生 URDF：外觀用 mesh、碰撞用 box／cylinder、慣量取自 CAD |
| `check_static.py` | 離線檢查：站姿（重心在腳掌上方）、靜態扭矩、步長與抬腳高度、尾巴造成的重心側移 |
| （`simulation/robot_assets/convert_usd.py`） | URDF → USD（Isaac Lab `UrdfConverter`），2026-10-15 移到模擬那一側 |
| （`simulation/robot_assets/check_sim.py`） | 模擬檢查：10 個關節的正方向、5 秒站立、截圖 |
| `build_viewer.py`、`viewer_template.html` | 互動 3D 圖（three.js）：可調關節、即時重心與支撐面、檢查報表 |

## 干涉檢查涵蓋什麼（`reports/interference.md`）

1. 每個關節在 `params.yaml` 的範圍內掃描（父、子連桿，以及跨兩、三個關節的組合）；
2. 同一連桿內的零件、螺絲頭、連接器插頭保留空間；
3. 零位與站姿時所有不相鄰的連桿；
4. 左右腿互撞（參考用）；
5. 電子部件佔位方塊。

實體包含列印件、馬達本體與擺臂、**螺絲頭**（M2 盤頭 Ø4 × 1.6）、**連接器插頭保留空間**（側面往外 10 mm）。

## 質量怎麼算

列印件 = CAD 體積 × `print.effective_density`（預設 1.05 g/cm³，約等於 PETG 1.27 × 83% 實心率）＋ 每個零件的螺絲數 × 0.3 g；
馬達 21 g；電子部件是 params.yaml 中的方塊；碳管按長度；尾端配重 = `tail.tip.mass` − 配重座質量。
實際列印後把秤到的重量回填 params.yaml（例如調整 effective_density），再重跑 1–6。
