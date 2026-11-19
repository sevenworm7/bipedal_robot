# 雙足機器人（帶主動尾巴）：設計、模擬訓練、實機部署

自行設計一台約 0.7 kg、每腿 4 DOF＋2 DOF 主動尾巴的小型雙足機器人（10 顆 Feetech HD-1910 伺服），
在 NVIDIA Isaac Lab 中訓練直線行走，之後 3D 列印成實體，在真實環境中走起來。工作規則見 [`CLAUDE.md`](CLAUDE.md)。

## 專案架構（2026-10-15 起，依「是什麼」分層）

| 資料夾 | 虛／實 | 內容 |
|---|---|---|
| [`docs/`](docs/README.md) | — | **跨領域文件**：進度與下一步（`progress.md`）、待辦（`backlog.md`）、決策（`decisions.md`）、成果總表（`results.md`）、架構（`architecture.md`） |
| [`hardware/`](hardware/README.md) | 共用（設計源頭） | 機構設計：參數檔 → 列印零件（STEP／STL）＋ URDF；每版 `robot_v<N>/`、組裝說明、BOM |
| [`core/`](core/README.md) | 共用 | 介面契約（觀測／動作）、左右鏡像、機器人定義（關節、站姿、致動器、伺服對照）；不依賴 Isaac |
| [`simulation/`](simulation/README.md) | 虛擬 | Isaac Lab 環境、地形、場景版本、指標、評估影片、URDF → USD |
| [`control/`](control/README.md) | 虛擬 | 產生行走策略：規則式老師 → 平地訓練 → sim-to-real 強健性訓練（含訓練範圍內的驗收） |
| [`evaluation/`](evaluation/README.md) | 虛擬 | 不訓練的評估：泛化／壓力測試、馬達負載分析 |
| [`deploy/`](deploy/README.md) | 實體 | 實機：馬達台架測試、校正、板上控制程式、實地測試（進行中） |
| [`archive/`](archive/README.md) | — | 封存的實驗一（參考模型，驗證管線可行） |
| [`tools/`](tools/README.md) | — | 開發工具（`isaacpy.cmd`、URDF 檢查） |
| `references/` | — | 參考資料（唯讀）；`mango_x4_handoff_20261117/` 是組員的芒果磚交接包（不進 git） |
| `outputs/` | — | 所有產出（不進 git），結構對應上表：`outputs/robot_v<N>/{hardware, simulation, control, evaluation, deploy}/`；數字總表在 [`docs/results.md`](docs/results.md) |

## 目前狀態（2026-11-19）

- **robot v1**：實體已組裝；組員以開迴路回放 student v5 的模擬軌跡，在桌上走約 8 步後跌倒（不是模型在板上閉迴路推論）。
- **robot v2**：縮短大腿（70 mm）與小腿（55 mm）讓馬達線夠長，已凍結（tag `robot-v2`）；最終模型 student v2ft 通過全部模擬門檻與壓力測試。
- **v2 板上程式**：模型匯出成 int8、芒果磚（RP2350）50 Hz 閉迴路韌體、PC 指令工具已離線完成並驗證（`deploy/`）；等 v2 大腿、小腿列印完成後上實機。

完整的計畫、各階段狀態與重點結果見 [`docs/progress.md`](docs/progress.md)，各階段的數字見 [`docs/results.md`](docs/results.md)。

```
 hardware ──→ simulation ──→ control ──────────────→ evaluation ──→ deploy
 機構設計      模擬平台       老師 → 平地 → sim2real     壓力測試、負載    實機
    ↑                            ↑                                     │
    └──────────── 實機發現的問題回饋（新版模型、重新訓練） ─────────────────┘
```

## 執行環境

Windows、RTX 5080 Laptop；Isaac Sim 6.0.1、Isaac Lab 3.0.0。模擬與訓練一律用 `tools\isaacpy.cmd <script>`（專案根目錄執行）；
CAD（build123d）用 `.venv_cad\Scripts\python.exe`；韌體建置（CMake、Ninja）與 PC 指令工具（pyserial）用 `.venv_fw\`，見 `deploy/firmware/README.md`。細節見 `CLAUDE.md` 與 `/isaac-run` skill。
