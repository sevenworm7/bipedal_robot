# Architecture — 雙足機器人（帶主動尾巴）

## 目標

自行設計一台小型雙足機器人（每腿 4 DOF ＋ 2 DOF 主動尾巴、10 顆 Feetech HD-1910），在 Isaac Lab 中訓練直線行走，再 3D 列印成實體，在真實環境中走起來。
**不做**：上半身、轉向與倒退（目前範圍）、依賴外部定位的控制。

## 主要模組（2026-10-15 使用者確認：依「是什麼」分層，虛實分開）

| 資料夾 | 虛／實 | 職責 | 規格與規則 |
|---|---|---|---|
| `hardware/` | 共用（設計源頭） | 機構設計：`robot_v<N>/params.yaml` → 列印零件（STEP／STL）＋ URDF；組裝說明、BOM、檢查報告 | `hardware/CLAUDE.md` |
| `core/` | 共用 | 介面契約（觀測／動作）、左右鏡像、機器人定義（`robots/custom_biped.py`）；不依賴 Isaac，實機也會用 | 根目錄 `CLAUDE.md`「跨流程介面契約」 |
| `simulation/` | 虛擬 | Isaac Lab 環境、地形、場景版本、指標、影片；`robot_assets/`：URDF → USD、模擬檢查 | `simulation/env/README.md` |
| `control/rule_based/` | 虛擬 | 規則式行走控制器（老師） | `control/rule_based/PROMPT.md` |
| `control/flat_pipeline/` | 虛擬 | 平地：rule-based → BC → PPO → 自我蒸餾 ×3 | `control/flat_pipeline/PROMPT.md` |
| `control/sim2real/` | 虛擬 | 強健性訓練：隨機化、可部署觀測、步態多樣性、teacher → student 蒸餾；含訓練範圍內的驗收 | `control/sim2real/PROMPT.md` |
| `evaluation/` | 虛擬 | 不訓練：`generalization/`（訓練範圍以外的壓力測試）、`load_analysis/`（馬達負載） | `evaluation/generalization/CLAUDE.md` |
| `deploy/` | 實體 | 實機：台架測試、校正、板上控制程式、實地測試；不可依賴 Isaac | `deploy/README.md` |
| `docs/` | — | 跨領域文件：進度與計畫、待辦、決策、成果總表、本檔 | [`README.md`](README.md) |
| `meeting/` | — | 會議報告（2026-11-23 起），一次報告一個資料夾 `<YYYYMMDD>_<主題>/`（`report.html`、影片走 git LFS） | `meeting/README.md` |
| `archive/exp1_reference/` | — | 實驗一（封存，自成一套） | `archive/exp1_reference/CLAUDE.md` |
| `tools/` | — | 開發工具（`isaacpy.cmd`、`urdf_inspect.py`），兩次實驗共用 | `tools/README.md` |
| `references/` | — | 參考資料（唯讀）：實驗一使用其中的 URDF；`mango_x4_handoff_20261117/` 是組員的芒果磚交接包（不進 git，decisions #26） | — |
| `outputs/` | — | 實驗二的產出（不進 git），結構對應程式資料夾 | — |

## 資料流 / 依賴方向

```
 hardware ──→ simulation ──→ control ──────────────→ evaluation ──→ deploy
 機構設計      模擬平台       老師 → 平地 → sim2real     壓力測試、負載    實機
    ↑                            ↑                                     │
    └──────────── 實機發現的問題回饋（新版模型、重新訓練） ─────────────────┘
```

- 依序相依：hardware → simulation → control（rule_based → flat_pipeline → sim2real）→ evaluation → deploy。
- `core/` 被所有虛擬與實體程式 import；`core/` 和 `deploy/` **不可依賴 Isaac**（例外：`deploy/export/` 在 PC 上產生韌體標頭與標準答案，會用到 torch 與模擬）。芒果磚韌體（C）透過匯出的 `robot_config.h` 取得 `core/` 的定義。
- `control/sim2real` import `flat_pipeline` 的網路、BC 與固定評估集設定；`flat_pipeline` 用 `rule_based` 當老師。
- `archive/exp1_reference/` 和實驗二互不相依。
- 機器人版本用環境變數 `CUSTOM_BIPED_VERSION` 切換（預設 1）。

## 目錄結構

```
README.md               # 專案總覽（一頁）
CLAUDE.md               # 工作規則
docs/                   # 跨領域文件：progress、backlog、decisions、results、architecture；videos/ 精選影片（git LFS）
meeting/                # 會議報告：<YYYYMMDD>_<主題>/report.html、videos/（git LFS）
hardware/               # generator/ 產生程式（含 check_cable.py）、robot_v<N>/ 每版的參數、零件、URDF、報告、design_calc；design_refs/ 設計構想與 HD-1910 資料包
core/                   # interface.py、interface_v2.py、symmetry.py、robots/custom_biped.py
simulation/             # env/（環境、地形、場景版本）、metrics、rollout、影片；robot_assets/（URDF → USD、模擬檢查、usd/ 不進 git）
control/                # rule_based/、flat_pipeline/、sim2real/（README ＋ log_robot_v<N>.md）
evaluation/             # generalization/、load_analysis/
deploy/                 # README（步驟）、test_log.md（實機紀錄）、export/（模型 → C 標頭）、firmware/（芒果磚 RP2350 韌體）、host/（PC 通用指令工具 robot_cli）、calibration/（校正檔）、actuator_bench/（單顆馬達測試）、field_tests/（整機懸空／平地測試）
archive/exp1_reference/ # 實驗一：shared/、scripts/、rule_based/、training_pipeline/、outputs/（不進 git）
references/             # 唯讀
tools/                  # isaacpy.cmd、urdf_inspect.py、bin/uv.exe（不進 git）
outputs/                # robot_v<N>/{hardware, simulation, control/{rule_based, flat_pipeline, sim2real}, evaluation/{generalization, load_analysis}, deploy}/
```

- **匯入方式**：腳本把專案根目錄（`core`、`simulation`）與 `control/`（`rule_based`、`flat_pipeline`、`sim2real`）加入 `sys.path`。
- **刻意保留的複本**：實驗一的程式凍結不動，實驗二移植時複製一份再修改，所以有些檔案兩邊內容相同（`rollout.py`、`video_overlay.py`、`eval_videos.py`、`leg_kinematics.py`、`PROMPT.md`）。

## 技術選型

- 模擬與訓練：Isaac Sim 6.0.1、Isaac Lab 3.0.0、rsl_rl 5.0.1、torch 2.10（cu128），一律透過 `tools\isaacpy.cmd` 執行。
- CAD：build123d，放在獨立的 `.venv_cad`（uv 建立，`tools/bin/uv.exe`、`.uv/`）。
- 馬達：Feetech HD-1910-C001（串列匯流排、ID 定址）；控制板：芒果磚（台灣開發板）。
- 理由見 [`decisions.md`](decisions.md)。

## 已知限制

- 沒有 ankle roll 和 hip yaw：側向平衡靠尾巴與落腳點，轉向目前不做。
- 沒有外部定位：側向位置無法修正。
- 馬達參數隨機化範圍尚未實測（`backlog.md` #10）。
- robot v2 原地抬腳上限只有 24 mm。

## 沿革

- 2026-10-09 以前為 `01_robot_construction`、`02_rule_based`、`03_training_pipeline`、`04_sim2real`；2026-10-09 為 `01_`–`04_generalization`；2026-10-15 重整為上表。
  舊 run 的 `config.yaml`／`stats.json` 中的路徑是歷史紀錄，不改寫。
- 2026-11-18：新增 `docs/`，計畫、待辦、決策、成果集中到這裡。
