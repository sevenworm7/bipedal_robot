# deploy/calibration：實機校正資料

每台實體機器人一份校正檔，`deploy/export/export_policy.py` 會把它寫進韌體的 `robot_config.h`。改了校正檔就要重新匯出、重新編譯。

## 工具：`joint_probe.py`（只讀，扭矩關閉）

```powershell
# 手扳對應：依序扳每個關節（先往指定方向約 30°、扳回、停 2 秒），自動判斷是哪個 ID、方向是否和校正檔一致
.venv_fw\Scripts\python.exe deploy\calibration\joint_probe.py [--joints left_ankle_pitch,tail_yaw]
# 靜態取樣：擺好姿勢後讀 3 秒，記錄每顆的平均值、標準差（零點姿勢、檢查姿勢）
.venv_fw\Scripts\python.exe deploy\calibration\joint_probe.py --capture pose_a_zero
```

輸出在 `outputs/robot_v2/deploy/calibration/{probe,capture}_<時間>/`。hip roll 兩邊靠得很近，往內扳會互撞，所以兩邊都先往外扳（程式中的 `FIRST_DIR`）。

## `robot_v2.json`（2026-11-23 實測，`verified: true`）

| 欄位 | 意義 | 來源 |
|---|---|---|
| `joints[].id` | 伺服 ID。**右腿 1、2、7、8，左腿 5、6、3、4**（髖左右、髖前後、膝、踝），尾巴 9 = yaw、10 = pitch。重新組裝時兩隻小腿的膝、踝馬達左右對調（ID 本身沒改，`docs/decisions.md` #27） | 手扳測試 `probe_20261123_094006`、`probe_20261123_094229` |
| `joints[].sign` | 位置值增加時關節角的方向（模型慣例，見 `core/robots/custom_biped.py`）。兩個膝蓋和 v1 相反 | 手扳測試；膝蓋另以彎 90° 的姿勢確認 |
| `joints[].zero_count` | 關節角 0（雙腿伸直平行、腳底水平；尾巴水平朝後）時的位置值。膝、踝是新量的；髖、尾巴的擺臂沒拆過，沿用 v1 | `capture_20261123_095320_pose_a_zero_level`；v1：交接包 `live_mirror/*-reference.json` |
| `joints[].zero_verified` | hip roll（兩次量測都偏右約 3.3°）、尾巴（量測時沒有固定）為 false，其他 true | — |
| `imu.sensor_to_body` | IMU 感測器軸 → 機身軸（+X 前、+Y 左、+Z 上），前後、左右都已確認 | 後仰 11° 與左傾約 35° 的取樣 |
| `joint_velocity_source` | 關節速度：`finite_difference`（位置差分）；伺服速度暫存器的單位還沒確認 | — |

關節角 [rad] = `sign` × (位置值 − `zero_count`) × 2π / 4096。伺服是多圈模式，上電後回報單圈位置，韌體會把零點平移整圈對齊目前讀值。

## 還要確認的事

1. hip roll 零點：有扭矩的站姿時量兩腳間距，和模型比對，決定沿用 v1 或改用新量的值（`docs/backlog.md` #18）。
2. 尾巴零點：固定成水平朝後再量一次。
3. 陀螺儀偏置：韌體開機時自動量 2 秒（要保持靜止）。

注意：`hardware/robot_v2/reports/servo_map.json` 是建模時假設的 ID 與零點，和實機不同；實機一律以這個檔案為準。
