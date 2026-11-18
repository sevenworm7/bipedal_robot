# 雙足機器人（帶主動尾巴）：設計、模擬訓練、實機部署

自行設計一台 0.69 kg、每腿 4 DOF＋2 DOF 主動尾巴的小型雙足機器人（10 顆 Feetech HD-1910 伺服），
在 NVIDIA Isaac Lab 中訓練直線行走，之後 3D 列印成實體，在真實環境中走起來。工作規則見 [`CLAUDE.md`](CLAUDE.md)。

## 專案架構（2026-10-15 起，依「是什麼」分層）

| 資料夾 | 虛／實 | 內容 |
|---|---|---|
| [`hardware/`](hardware/README.md) | 共用（設計源頭） | 機構設計：參數檔 → 列印零件（STEP／STL）＋ URDF；每版 `robot_v<N>/`、組裝說明、BOM |
| [`core/`](core/README.md) | 共用 | 介面契約（觀測／動作）、左右鏡像、機器人定義（關節、站姿、致動器、伺服對照）；不依賴 Isaac |
| [`simulation/`](simulation/README.md) | 虛擬 | Isaac Lab 環境、地形、場景版本、指標、評估影片、URDF → USD |
| [`control/`](control/README.md) | 虛擬 | 產生行走策略：規則式老師 → 平地訓練 → sim-to-real 強健性訓練（含訓練範圍內的驗收） |
| [`evaluation/`](evaluation/README.md) | 虛擬 | 不訓練的評估：泛化／壓力測試、馬達負載分析 |
| [`deploy/`](deploy/README.md) | 實體 | 實機：馬達台架測試、校正、板上控制程式、實地測試（拿到器材後開始） |
| [`archive/`](archive/README.md) | — | 封存的實驗一（參考模型，驗證管線可行） |
| [`tools/`](tools/README.md) | — | 開發工具（`isaacpy.cmd`、URDF 檢查） |
| `references/` | — | 參考資料（唯讀） |
| `outputs/` | — | 所有產出（不進 git），結構對應上表：`outputs/robot_v<N>/{hardware, simulation, control, evaluation, deploy}/`；總表 `outputs/robot_v1/summary.md` |

## 路線圖

```
 hardware ──→ simulation ──→ control ──────────────→ evaluation ──→ deploy
 機構設計      模擬平台       老師 → 平地 → sim2real     壓力測試、負載    實機
    ↑                            ↑                                     │
    └──────────── 實機發現的問題回饋（v2 設計、重新訓練） ──────────────────┘
```

| 階段 | 位置 | 狀態 | 重點結果 |
|---|---|---|---|
| 實驗一：參考模型驗證管線 | `archive/exp1_reference/` | 完成（2026-09-29） | rule-based 95% → 最終蒸餾 99%／跌倒 0% |
| 機器人模型 v1 | `hardware/robot_v1/` | 凍結（2026-09-30，tag `robot-v1`） | 0.69 kg、10 顆 HD-1910 |
| 規則式老師 | `control/rule_based/` | 完成（2026-10-01） | 100%／0% |
| 平地訓練 | `control/flat_pipeline/` | 完成（2026-10-01） | 99%／1%（需要 base 線速度，不可部署） |
| sim-to-real 強健性訓練 | `control/sim2real/` | 完成（2026-10-07） | **student v5**：平地與隨機化 100%、凹凸 94%、組合 85.7%（可部署：只用 IMU 與伺服讀值） |
| 泛化／壓力測試 | `evaluation/generalization/` | 完成（2026-10-09） | 延遲 60 ms、摩擦 0.2–1.5、馬達 50%、質量 ×1.5 仍 ≥ 95%；弱點：上 10 mm 台階、凸起 > 10 mm、延遲 ≥ 80 ms |
| 實機誤差測試 `stress_v3` | `evaluation/generalization/` | 完成（2026-10-15） | 關節零點要校正到 ±2°；IMU 傾斜容許約 8°；指令遺失 40% 仍正常 |
| 馬達負載分析 | `evaluation/load_analysis/` | 完成（2026-10-15） | 各關節 RMS ≤ 額定 70%（最吃力 hip pitch）；整機平均約 3 A、峰值 8.8 A；站立時雙腳可能卡住 |
| 實機部署 | `deploy/` | **進行中**（2026-11-18 回報） | 實體已列印組裝、芒果磚控制；模擬模型在實機可走約 8 步，之後腿軟跌倒（電流問題由他人處理） |
| 機器人模型 v2 | `hardware/v2_backlog.md` | **規劃中**（2026-11-18） | 腿部馬達線只有約 12 cm → 縮短大腿與小腿、改用折返接法；尾巴維持 250 mm |

## 執行環境

Windows、RTX 5080 Laptop；Isaac Sim 6.0.1、Isaac Lab 3.0.0。模擬與訓練一律用 `tools\isaacpy.cmd <script>`（專案根目錄執行）；
CAD（build123d）用 `.venv_cad\Scripts\python.exe`。細節見 `CLAUDE.md` 與 `/isaac-run` skill。
