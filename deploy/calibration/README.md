# deploy/calibration：實機校正資料

每台實體機器人一份校正檔，`deploy/export/export_policy.py` 會把它寫進韌體的 `robot_config.h`。改了校正檔就要重新匯出、重新編譯。

## `robot_v2.json`（目前是**暫定值**，`verified: false`）

| 欄位 | 意義 | 目前來源 |
|---|---|---|
| `joints[].id` | 伺服 ID。沿用組員已寫進舵機的 **TOPDOWN_V1**：右腿 1–4、左腿 5–8（髖左右、髖前後、膝、踝），尾巴 9 = yaw、10 = pitch（`docs/decisions.md` #27） | 交接包 `README_交接總整理.md` |
| `joints[].sign` | 位置值增加時關節角的方向（模型慣例，見 `core/robots/custom_biped.py`） | 組員 v1 實測（`friend_policy_c/full_gait_export.json`） |
| `joints[].zero_count` | 關節角 0（雙腿伸直平行、腳底水平；尾巴水平朝後）時的位置值 | 組員 v1 實測（`live_mirror/parallel-reference.json`、`tail-reference.json`） |
| `imu.sensor_to_body` | IMU 感測器軸 → 機身軸（+X 前、+Y 左、+Z 上） | 組員暫定 body = (−sY, sX, sZ)，只確認過前傾 |
| `joint_velocity_source` | 關節速度：`finite_difference`（位置差分）；伺服速度暫存器的單位還沒確認 | — |

關節角 [rad] = `sign` × (位置值 − `zero_count`) × 2π / 4096。伺服是多圈模式，上電後回報單圈位置，韌體會把零點平移整圈對齊目前讀值。

## 要在實物上確認的事（v2 組裝後）

1. 大腿、小腿重印後，膝、踝（可能也包括髖前後）的擺臂重新安裝，**零點一定要重量**；方向如果安裝方式和 v1 相同應該不變，但仍要逐顆小幅確認。
2. IMU 左右傾斜的方向、陀螺儀偏置（韌體開機時會自動量 2 秒，要保持靜止）。
3. 全部確認後把 `verified` 改成 true，記錄日期與證據（log 路徑），再重新匯出。

注意：`hardware/robot_v2/reports/servo_map.json` 是建模時假設「左腿 1–4、零點 2048」產生的，和實機不同；實機一律以這個檔案為準。
