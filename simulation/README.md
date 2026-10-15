# simulation：模擬平台（Isaac Lab，實驗二）

機器人在模擬中的一切：環境、地形、帶版本的場景、指標、rollout、評估影片，以及 URDF → USD 的轉檔與模擬檢查。
**只在模擬中使用**；虛實共用的介面與機器人定義在 [`core/`](../core/README.md)。2026-10-15 從原 `shared/` 拆出。
程式從 `archive/exp1_reference/shared/` 移植（2026-10-01），只替換 robot 設定。

| 檔案／資料夾 | 內容 |
|---|---|
| `env/` | 直線行走環境 `BipedWalkEnv`（gym id：`Biped-Walk-v0`）、隨機化、地形、帶版本的場景（含 `evaluation/generalization` 的壓力測試條件），見 [`env/README.md`](env/README.md) |
| `metrics.py` | 每個 episode 的指標，以及 rule-based 的成功判定 `rule_based_success` |
| `rollout.py` | 平行 rollout 與統計（`run_first_episodes`、`aggregate`、分組統計、扭矩百分位、推擠測試） |
| `video_overlay.py`、`eval_videos.py` | 評估影片的 HUD 與錄影（格式依 `/eval-video`），rule-based 與訓練共用 |
| `robot_assets/` | URDF → USD 轉檔、模擬中的機器人檢查，見 [`robot_assets/README.md`](robot_assets/README.md) |

## 匯入方式

腳本把專案根目錄加入 `sys.path`，再用 `from simulation.env import BipedWalkEnv` 等方式匯入。
`env/`、`rollout.py`、`eval_videos.py` 需要先啟動 Isaac Sim（`AppLauncher`）。

## 修改規則

`env/` 的新功能一律預設關閉；每次修改後用 `control/sim2real/evaluate.py --compare` 重跑 `flat_v1` 回歸檢查，結果必須完全一致（規則見 `control/sim2real/CLAUDE.md`）。
最近一次：2026-10-15 `regress10`（資料夾重整後），完全一致。
