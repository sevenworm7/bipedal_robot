# shared/env：直線行走環境（實驗二）

`BipedWalkEnv`（Isaac Lab `DirectRLEnv`），gym id 為 `Biped-Walk-v0`。rule-based 評估、資料收集、BC 評估、PPO 訓練都使用同一個環境。
程式從 `00_test/shared/env/` 移植（2026-10-01），邏輯不變，只有下面「和實驗一的差異」列出的地方有改。所有參數都在 `biped_env_cfg.py`（`BipedWalkEnvCfg`）中。

## 和實驗一的差異

| 項目 | 實驗一 | 實驗二 |
|---|---|---|
| 預設機器人 | `ref_biped` | `custom_biped`（版本由 `CUSTOM_BIPED_VERSION` 決定，預設 v1） |
| 動作／觀測中的關節 | `LEG_JOINT_NAMES`（8） | `ACTION_JOINT_NAMES`：腿 8 ＋ 尾巴 2 = 10（使用者 2026-09-30 決定）；觀測 44 維 |
| 和機器人大小有關的設定 | 寫在 cfg 中 | cfg 中預設為 `None`，由 robot 模組的 `ENV_DEFAULTS` 填入；腳本明確設定的值會保留 |
| 扭矩飽和判定 | 扭矩 ≥ 0.99 × effort limit | 致動器模型有截斷扭矩時（`computed_torque ≠ applied_torque`），DC 馬達的扭矩–速度曲線也算在內 |
| 跟拍鏡頭 | 對準 CoM 下方 0.12 m | 由 `follow_cam_target_drop` 決定 |

`leg_ids`、`leg_pos`、`leg_torque` 等變數沿用實驗一的名稱，但涵蓋全部 10 個動作關節（含尾巴）。

## 每個 episode 的設定（custom_biped v1）

| 項目 | 數值 |
|---|---|
| 地形 | 平地，摩擦係數 1.0（rule-based 階段只用平地，使用者 2026-09-30 決定） |
| 長度 | 20 s |
| 速度指令 `cmd_vx` | 0.05–0.15 m/s（使用者 2026-09-30 決定） |
| 步態頻率 | 1.2–2.0 Hz（一個週期兩步；使用者 2026-09-30 決定） |
| 初始朝向 | 全部朝向世界座標 +X；目標方向等於初始朝向（直線行走） |
| 排列 | 並排成列：左右間距 1.0 m、每列 64 台、列與列相距 5 m |
| 初始關節雜訊 | ±0.05 rad（10 個關節） |

## 結束條件

- 跌倒：base CoM 低於 0.146 m（站姿時 base CoM 高 0.243 m 的 60%），或傾斜超過 40°（2026-10-01 修正：原本誤用全身 CoM 高度 0.183 m，門檻只有 0.110 m）
- 時間到

## 指標（`shared/metrics.py`）

- 步數：腳掌由離地變為著地的次數，前 0.1 s 的初始落地不計入；騰空時間需 ≥ 0.03 s（`min_step_air_time`）
- 前進距離、側向偏移、方向誤差、扭矩飽和比例、各關節扭矩的平均值與最大值
- 評估腳本設定 `record_episodes=True` 後，每個結束的 episode 摘要會存在 `env.completed_episodes`

## 獎勵（訓練用，初始權重沿用實驗一）

和實驗一相同的 14 項。其中和機器人大小有關的 `contact_force_limit`（1.5 × 體重 ≈ 10 N）、`target_air_time`（0.15 s）已由 `ENV_DEFAULTS` 換算；
`gait_swing_start`／`gait_swing_frac` 仍是實驗一教師策略量到的值，進入訓練前要在實驗二的教師策略上重新量測。
`base_height`、`lateral_deviation` 等權重是以實驗一的尺寸調的，也要在訓練階段確認。

## 推擠、錄影、實作注意事項

與實驗一相同，見 `00_test/shared/env/README.md`：推擠預設關閉，`push_interval_s` 設定後開啟；錄影要 `cfg.eval_camera = True` 加 `--enable_cameras`；
接觸感測器是延遲更新的，著地由 `_update_touchdowns()` 自行判斷。
