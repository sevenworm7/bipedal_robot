# 01_robot_construction/model：實驗二的參數化機器人模型

每個版本一個子資料夾 `v<N>/`，**所有版本都保留**（規則見根目錄 `CLAUDE.md`「版本控制」）；產生程式在 `generator/`，各版共用。

| 版本 | 狀態 | 重點 |
|---|---|---|
| [`v1/`](v1/README.md) | **已凍結**（2026-09-30，tag `robot-v1`） | 第一個實際建模的版本：每腿 4 DOF＋2 DOF 尾巴、10 顆 HD-1910、0.69 kg |

| 資料夾 | 內容 |
|---|---|
| `generator/` | params.yaml → 零件（build123d）→ URDF → USD → 檢查，執行方式見 [`generator/README.md`](generator/README.md) |
| `v<N>/` | 該版的 `params.yaml`（唯一參數來源）、`CHANGELOG.md`、`ASSEMBLY.md`、列印檔、URDF、檢查報告 |

新增版本：複製上一版的 `params.yaml` 到 `v<N+1>/`，把 `robot.version` 改成 N+1、修改參數、寫 CHANGELOG，再跑 generator 的 1–6 步。
之後發現的模型問題記在 [`v2_backlog.md`](v2_backlog.md)，開 v2 時一次處理。
USD 轉檔結果放在 `shared/robots/usd/custom_biped_v<N>/`（不進 git），下游用 `CUSTOM_BIPED_VERSION` 環境變數選版本（預設見 `shared/robots/custom_biped.py`）。
