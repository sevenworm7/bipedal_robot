# deploy/firmware：芒果磚（RP2350）板上控制程式（robot v2）

在芒果磚 X4 上以 50 Hz 跑閉迴路：讀 10 顆伺服＋ BMI270 IMU → 估測姿態 → 狀態機（固定站姿／學生模型）→ 一次同步寫入 10 顆目標位置。
不依賴 Isaac；機器人相關的數字全部來自 `deploy/export` 產生的 `robot_config.h` 與 `policy_int8.h`。

## 和組員韌體的關係

組員交接包（`references/mango_x4_handoff_20261117/`，唯讀、不進 git）是 robot v1 的**懸空回放**韌體，沒有板上推論。這裡是重新寫的 v2 閉迴路程式，只沿用他實測過的硬體做法：

| 沿用的內容 | 來源（交接包 `mango_feetech/`） |
|---|---|
| 伺服匯流排：UART1 GPIO8/9、1 Mbps 半雙工、收到自己的回音要略過、腳位 pad 設定 `0x56` | `scan_readonly.c`、`right_leg_motion.h` |
| 暫存器位置（40 扭矩、42 目標位置帶正負號位元、56–70 回授）、同步寫入 `0x83`、寫入後讀回確認 | `right_leg_motion.h`、`stance_setup/hanging_replay_v2.h` |
| 開動前檢查：型號 10/31、ID、模式 4、扭矩關閉 | `stance_setup/servo_command.c` |
| BMI270 SPI0 初始化（CS21、TFT CS33 拉高）、Bosch 官方驅動 | `stance_setup/imu_reference.h`、`stance_setup/vendor/bmi270/`（原樣複製到 `vendor/bmi270/`，含授權與 SHA256） |
| 板子定義 | `boards/mango_x4.h`（原樣複製，SHA256 開頭 `0086e85e`） |
| 伺服 ID（TOPDOWN_V1）、方向、零點 | 寫進 `deploy/calibration/robot_v2.json`（暫定值） |

## 程式結構

| 檔案 | 內容 | PC 測試 |
|---|---|---|
| `src/policy.c` | MLP 推論（int8 或 fp32），非有限值時拒絕輸出 | ✅ 和 PyTorch 比對 |
| `src/obs.c` | 學生觀測：每幀 42 維、5 幀、最新在前，順序同 `core/interface_v2.py` | ✅ 和模擬逐步比對 |
| `src/attitude.c` | Mahony 姿態濾波：陀螺儀積分＋加速度修正 roll／pitch；輸出重力方向與航向（航向會漂移，沒有磁力計） | ✅ 合成資料 |
| `src/joints.c` | 位置值 ↔ 關節角、多圈零點對齊、目標角限制在關節極限內 | ✅ |
| `src/feetech.c` | FT-SCS 封包組裝與解析（read、write、sync write、sync read），只做資料處理不碰硬體 | ✅ |
| `src/controller.c` | 狀態機：OFF → INIT（慢速到站姿）→ STAND（固定站姿，不跑模型）→ WALK（模型）；所有安全停止 | ✅ |
| `src/main_rp2350.c` | 硬體 I/O、50 Hz 迴圈、USB 指令、遙測 | 只編譯（需要實機） |

### 狀態機與安全

- **STAND**：目標固定為預設站姿（＝動作 0，`decisions.md` #24），但每個週期仍把觀測（指令 0、步頻 0、上一動作 0）推進歷史，和模擬中驗證過的交接流程（`evaluation/load_analysis/stand_handover.py`，100%／0%）一致。
- **WALK**：步態時鐘從 0 開始，目標航向＝交接當下的航向；指令範圍限制在訓練範圍（0–0.15 m/s、1.2–2.0 Hz）。
- **WALK → STAND**：0.5 秒平滑回到站姿（模擬中還沒測過）。
- **停止並關扭矩（機器人不再被支撐！）**：傾斜 > 40°、任一伺服電壓 < 6.5 V、溫度 ≥ 60 °C、伺服狀態錯誤、關節超出極限 10°、追隨誤差 > 25° 連續 3 週期、讀取失敗連續 3 週期、推論輸出非有限值、迴圈超時連續 5 週期、USB 斷線、STOP／ESC／Ctrl-C。
- **校正未確認前拒絕 INIT**：`robot_v2.json` 的 `verified` 是 false 時，韌體不會開扭矩。台架測試另外用 `-DALLOW_UNVERIFIED_CALIB=ON` 編譯。

### USB 指令

`PING`、`STATUS`、`TELEM 0|1`、`BENCH`（板上推論自我測試＋計時）、`BUSTEST <次數>`（只讀，比較 sync read 和逐顆讀取的時間）、`IMUCAL`（陀螺儀偏置，需靜止）、`INIT`、`WALK <vx> <freq> [clearance]`、`SET <vx> <freq>`、`STAND`、`STOP`。
PC 端用 [`deploy/host/robot_cli.py`](../host/README.md) 送指令並記錄遙測。

## 建置

```bash
# PC 單元測試（MSYS2 gcc），fp32 與 int8 各跑一次
PATH="/c/msys64/ucrt64/bin:$PATH" bash deploy/firmware/build_host_tests.sh
# 韌體（不燒錄）：輸出 outputs/robot_v2/deploy/firmware_build/<時間>/biped_runtime.uf2
bash deploy/firmware/build_firmware.sh
bash deploy/firmware/build_firmware.sh "" -DALLOW_UNVERIFIED_CALIB=ON   # 台架用
```

工具：CMake／Ninja 裝在專案內的 `.venv_fw/`（uv，不進 git）；ARM GCC 14.2、Pico SDK 2.1、picotool 直接用交接包 `.tools/` 裡的版本（唯讀）。
燒錄還沒做，要和使用者一起進行：按住 BOOT 接上 USB，把 `.uf2` 複製到 RP2350 磁碟。燒錄前先備份組員韌體（交接包 `build/mango_servo_command.uf2` 已有一份）。

## 紀錄

### 2026-11-19：離線完成（A–C）

- PC 單元測試全部通過：
  - float32 推論和 PyTorch 最大差 1.6e-6。
  - 由模擬原始讀值重建 4 台 × 252 步的觀測，最大差 6e-8。
  - int8 推論和 Python 參考實作最大差 1.2e-6；和 float32 模型的動作差最大 0.018。
  - 狀態機、封包、多圈對齊、姿態濾波的測試也全部通過。
- 韌體編譯 0 個警告；int8 權重 281 KB 在 SRAM（`.data.policy`），推論迴圈在 SRAM（`.time_critical.policy`）；RAM 共用約 300 KB／512 KB。
- **還沒在實機上執行過**。待做（需要實機）：①板上 `BENCH`（推論時間、結果一致）②`BUSTEST`（確認 sync read 是否支援、讀取時間）③ IMU 軸向與偏置 ④ v2 組裝後的零點與方向校正 ⑤懸空閉迴路。
