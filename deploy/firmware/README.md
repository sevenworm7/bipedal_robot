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

`PING`、`STATUS`、`TELEM 0|1`、`BENCH`（板上推論自我測試＋計時）、`BUSTEST <次數>`（只讀，比較 sync read 和逐顆讀取的時間）、`IMUCAL`（陀螺儀偏置，需靜止）、`SCAN [max_id]`（只讀掃描匯流排：每個有回應的 ID 的型號、模式、扭矩、位置、電壓、溫度；2026-11-23 新增）、`ALIGN`（重新對準多圈零點，關節移回範圍內後使用）、`JOG <關節名稱> <角度> [速度]`（只開一顆的扭矩：±15° 內、≤ 20°/s，出去 → 停 1 s → 回來 → 停 1 s，100 Hz 回報目標與實際位置；PC 送任何字元就關扭矩）、`INIT`、`WALK <vx> <freq> [clearance]`、`SET <vx> <freq>`、`STAND`、`STOP`。
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
燒錄（2026-11-23 起）：

```powershell
# 預設燒最新的 firmware_build；1200 baud 讓板子進 BOOTSEL，再複製 UF2（做法取自組員的 flash.ps1）
powershell -ExecutionPolicy Bypass -File deploy\firmware\flash_firmware.ps1 [-Uf2 <file.uf2>] [-Port COM3]
```

燒錄不會寫馬達 EEPROM，也不會開扭矩。組員韌體的備份：`outputs/robot_v2/deploy/teammate_fw_backup_20261123/mango_servo_command.uf2`（SHA256 和交接包 MANIFEST 相同），要還原時用 `-Uf2` 燒回去。

## 紀錄

### 2026-11-23（傍晚）：PLAY 與第一次 WALK

- 新增 `PLAY <倍速> [次數]`：站姿下開迴路播放模擬步態（`playback.h`，由 `deploy/export/make_playback.py` 產生；沒有這個檔案時指令回報錯誤），中止條件和控制器相同。
- 主迴圈的 dt 超過 0.1 s（阻塞指令之後）改用一個週期，避免姿態濾波器誤判傾斜。
- 閉迴路 `WALK` 控制週期 13.9 ms；0.34 秒後因供電壓降停機（`deploy/test_log.md`）。

### 2026-11-23（下午）：第一次開扭矩

- `JOG` 單顆延遲 25–35 ms、靜止誤差 ≤ 0.26°；懸空與桌上的固定站姿誤差 ≤ 0.5°，電壓 7.6–7.8 V（細節見 `deploy/test_log.md`）。
- 新增：`ALIGN`、`JOG`；安全範圍 `RANGE_LOWER/UPPER`（匯出時由校正檔的 `range_check_deg` 決定，尾巴 ±170°），寫入目標時截在安全範圍 ± 10°，模型輸出在控制器截在訓練範圍；遙測加上每顆的負載與電流暫存器（表頭見 `deploy/host/robot_cli.py`，共 65 欄）。

### 2026-11-23：第一次上實機（robot v2，只讀）

- `BENCH`：板上推論結果和 PC 一致，但 int8 權重 × float activation 要 **15.5 ms**（每次乘加約 8.5 個 cycle），加上讀取會超過 20 ms。
  改成 activation 逐層動態量化成 int16（最大值對應 16383，512 項累加不會溢位 int32），用 Cortex-M33 的 `SXTB16`／`SMLAD` 一次做兩個乘加 → **6.8 ms**。
  和 Python int8 參考（float activation）最大差 2.3e-4，和 fp32 模型的動作差 0.018（改前 0.018），所以模擬閉迴路的 int8 結果仍適用。
  PC 測試的比對門檻改為 `POLICY_REF_TOL`（int8 1e-3、fp32 1e-4，`src/policy.h`），PC 端用 C 模擬這兩個指令，計算方式相同。
- `BUSTEST 100`：sync read 支援，10 顆 6.35 ms、0 失敗；逐顆讀 30.8 ms。一個控制週期預估：讀取 6.35 ＋ 推論 6.8 ＋ IMU／寫入約 0.5 ≈ **13.7 ms**（< 20 ms）。
- 新增 `SCAN` 指令與 `flash_firmware.ps1`；校正完成後重新匯出、燒錄，開機 `calib_verified=1 aligned=1`。

### 2026-11-19：離線完成（A–C）

- PC 單元測試全部通過：
  - float32 推論和 PyTorch 最大差 1.6e-6。
  - 由模擬原始讀值重建 4 台 × 252 步的觀測，最大差 6e-8。
  - int8 推論和 Python 參考實作最大差 1.2e-6；和 float32 模型的動作差最大 0.018。
  - 狀態機、封包、多圈對齊、姿態濾波的測試也全部通過。
- 韌體編譯 0 個警告；int8 權重 281 KB 在 SRAM（`.data.policy`），推論迴圈在 SRAM（`.time_critical.policy`）；RAM 共用約 300 KB／512 KB。
- 當時還沒上實機；①–④（BENCH、BUSTEST、IMU 軸向、零點與方向）已在 2026-11-23 完成，⑤懸空閉迴路待做。
