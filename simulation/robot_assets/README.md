# simulation/robot_assets：模擬用的機器人資產

把 `hardware/robot_v<N>/urdf/` 的 URDF 轉成 Isaac Sim 的 USD，並在模擬中檢查機器人是否正確（關節方向、站姿、質量）。
這兩個步驟需要 Isaac Lab，所以放在模擬這一側；URDF 本身由 `hardware/generator/` 產生（流程第 1–4 步見 [`hardware/generator/README.md`](../../hardware/generator/README.md)）。

| 檔案 | 指令（專案根目錄） | 產出 |
|---|---|---|
| `convert_usd.py` | `tools\isaacpy.cmd simulation\robot_assets\convert_usd.py --version 1` | `usd/custom_biped_v1/`（不進 git，隨時可重建） |
| `check_sim.py` | `tools\isaacpy.cmd simulation\robot_assets\check_sim.py --version 1 --enable_cameras` | `outputs/robot_v1/simulation/robot_check/<時間>/`（report.json、截圖） |

`core/robots/custom_biped.py` 從 `usd/custom_biped_v<N>/` 載入 USD（版本由環境變數 `CUSTOM_BIPED_VERSION` 決定，預設 1）。
2026-10-15 以前這兩個程式在 `01_robot_construction/model/generator/`，USD 在 `shared/robots/usd/`。
