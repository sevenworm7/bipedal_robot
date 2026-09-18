# 00_test/shared/env：直線行走環境

`BipedWalkEnv`（Isaac Lab `DirectRLEnv`），gym id 為 `Biped-Test-Walk-v0`。rule-based 評估、資料收集、BC 評估、PPO 訓練都使用同一個環境。
所有參數都在 `biped_env_cfg.py`（`BipedWalkEnvCfg`）中。

## 每個 episode 的設定

| 項目 | 預設值 |
|---|---|
| 地形 | 平地，摩擦係數 1.0 |
| 長度 | 20 s |
| 速度指令 `cmd_vx` | 0.05–0.20 m/s |
| 步態頻率 | 0.8–1.2 Hz（一個週期兩步），用來驅動觀測中的步態相位 |
| 初始朝向 | 全部朝向前進方向（世界座標 +X，也就是機器人的 +Y）；目標方向等於初始朝向（直線行走） |
| 排列 | 並排成列：左右間距 1 m、每列 64 台、列與列相距 10 m（`spawn_origins`，取代 Isaac Lab 預設的方格排列） |
| 初始關節雜訊 | ±0.05 rad |

## 結束條件

- 跌倒：base CoM 低於 0.22 m，或傾斜超過 40°
- 時間到

## 指標（`shared/metrics.py`）

- 步數：腳掌由離地變為著地的次數，前 0.1 s 的初始落地不計入
- 前進距離：沿目標方向的位移
- 側向偏移：垂直於目標方向的位移
- 其他：方向誤差、扭矩飽和比例、各關節扭矩的平均值與最大值
- 評估腳本設定 `record_episodes=True` 後，每個結束的 episode 摘要會存在 `env.completed_episodes`

## 獎勵（初始權重，在 PPO 階段調整）

速度追蹤、方向追蹤、側向速度、側向偏移、姿態、base 高度、存活、跌倒懲罰、扭矩、動作變化率、關節極限、腳掌騰空時間、接觸力。
每一項會分開記錄到 `extras["log"]`，可以在 TensorBoard 上看到。

## 錄影

設定 `cfg.eval_camera = True` 並加上 `--enable_cameras` 後，`env.capture_frame(env_id)` 會從機器人側前方拍攝（相機跟隨目標方向），
再交給 `shared/video_overlay.py` 疊加 HUD。

## 實作注意事項

- 關節一律用 `LEG_JOINT_NAMES` 的順序（`self.leg_ids`）存取；模擬器內部的關節順序不同。
- 每隻腳各有一個接觸感測器（`foot_sensors`），並在 cloning 之前補上 contact report API（原因見 `00_test/README.md`）。
- 步態相位由 episode 時間計算，不另外保存狀態：`phase = (phase0 + f × t) mod 1`。
