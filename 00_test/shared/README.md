# 00_test/shared：實驗一的共用程式

rule-based 控制器與訓練管線都**只能**從這裡取得介面定義，不可各自定義（見根目錄 CLAUDE.md「跨流程介面契約」）。
這裡的程式碼不含任何機器人專屬的數值；這些數值只放在 `robots/<robot>.py`。實驗二會把這個資料夾移植到根目錄的 `shared/`。

| 檔案 | 內容 |
|---|---|
| `interface.py` | **介面契約**：觀測向量（38 維，含步頻 `gait_freq`）、動作向量（8 維）、控制頻率、動作縮放、canonical 座標系 |
| `metrics.py` | 每個 episode 的指標（步數、前進距離、側向偏移、扭矩飽和比例…），以及 rule-based 的成功判定 |
| `video_overlay.py` | 評估影片的 HUD 疊加與錄影（格式依 `/eval-video` skill） |
| `env/` | Isaac Lab 直線行走環境 `BipedWalkEnv`（gym id：`Biped-Test-Walk-v0`） |
| `robots/` | 機器人設定（`ref_biped.py`）與轉換出的 USD |

## 介面摘要（詳細說明在 `interface.py` 的 docstring）

- 控制頻率 50 Hz（物理 200 Hz × decimation 4）
- 觀測（38 維）：base 線速度(3)、角速度(3)、重力投影(3)、速度指令(1)、步頻指令 `gait_freq`(1，2026-09-19 加入)、方向誤差(1)、關節角度(8)、關節角速度(8)、上一次動作(8)、步態相位 sin/cos(2)，全部使用 SI 單位，不做正規化
- 動作：`q_target = q_default + 0.25 × action`，action 會被 clip 到 ±5
- 關節順序：一律使用 `robots/<robot>.py` 的 `LEG_JOINT_NAMES`，不使用模擬器內部的順序
- 座標：canonical base frame，+x 為前進方向、+y 為左、+z 為上

## 匯入方式

`00_test` 的名稱以數字開頭，無法當作 Python 套件匯入，所以腳本會先把 `00_test/` 加入 `sys.path`，再用 `import shared...` 匯入。
`env/` 與 `robots/*.make_articulation_cfg()` 需要先啟動 Isaac Sim（`AppLauncher`）才能匯入或呼叫。
