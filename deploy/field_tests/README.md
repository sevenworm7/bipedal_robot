# deploy/field_tests：實機整機測試（`deploy/README.md` 步驟 5–7）

在實體機器人上整機測試：懸吊（步驟 5）→ 安全繩上桌、平地行走（步驟 6）→ 進階測試（步驟 7）。
2026-11-25 建立，`hanging_test` 從 `deploy/host/` 移過來；`host/` 只保留通用的 USB 指令工具 `robot_cli.py`（這裡的程式會引用它的遙測欄位定義）。
測試結果寫在 [`deploy/test_log.md`](../test_log.md)，原始紀錄在 `outputs/robot_v2/deploy/sessions/<時間>/`。

## `hanging_test.ps1`／`hanging_test.py`：分階段懸空測試＋馬達數據表

取代 2026-11-23 的 `demo_hanging.ps1`（已刪除）：一樣有 `play`／`walk` 兩種流程，另外加上韌體與伺服檢查、每顆馬達的數據表。

| 模式 `--mode` | 流程 |
|---|---|
| `play`（預設） | `INIT` → 站姿下以 `--rate` 倍速開迴路播放模擬中 student 的走路軌跡（`PLAY`）→ 回站姿 → `STOP`。0.5 倍：步態 8.1 秒＋前後 1.5 秒過渡 |
| `walk` | `INIT` → 模型閉迴路 `WALK --vx --freq`（實際速度）`--seconds` 秒 → `STAND` → `STOP`；遇到 FAULT 立即結束 |

| 階段 | 做什麼 | 扭矩 |
|---|---|---|
| `check` | 板子有回應；燒錄的是哪一版韌體（`STATUS` 的 `build=`）；`SCAN` 找到 10 顆（ID、電壓、溫度） | 關 |
| `flash` | 板上不是該模型的指定版本時，用 `deploy/firmware/flash_firmware.ps1` 燒錄，再重新 `check`（`all` 會自動判斷） | 關 |
| `run` | `ALIGN` → `INIT` → 查詢 `STATUS` 直到 `STAND` → `TELEM 1` → 該模式的流程 → `STOP` | 開 → 關 |
| `report` | 從 `servo_telemetry.csv` 整理每顆馬達的追隨誤差、電壓、溫度、電流／負載暫存器，以及整機最低電壓與總電流，寫成 `report.md` | — |

```powershell
# 舊模型 v2ft 0.5 倍慢速回放（預設）
powershell -ExecutionPolicy Bypass -File deploy\field_tests\hanging_test.ps1 all
# 閉迴路 WALK：v2slow 1.0 Hz、v2ft 1.6 Hz 對照（換供電後的 B 測試）
powershell -ExecutionPolicy Bypass -File deploy\field_tests\hanging_test.ps1 all --mode walk --model v2slow
powershell -ExecutionPolicy Bypass -File deploy\field_tests\hanging_test.ps1 all --mode walk --model v2ft
# 只做單一階段
powershell -ExecutionPolicy Bypass -File deploy\field_tests\hanging_test.ps1 check
powershell -ExecutionPolicy Bypass -File deploy\field_tests\hanging_test.ps1 report [--session <資料夾>]
```

- 其他參數：`--rate` 0.05–1、`--loops` 1–5、`--vx`（預設 0.05）、`--freq`（預設 v2ft 1.6、v2slow 1.0；實際範圍由韌體依模型檢查）、`--seconds` 1–30、`--vstop` 5.5–7.0（韌體預設 6.5；低於 6.5 只限懸空）、`--port`、`--yes`（略過「是否懸空」的確認）。
- 韌體 0.2 版（`outputs/robot_v2/deploy/firmware_build/`）：v2ft `20261125_155757`（匯出 `20261123_103850`，回放模擬中 vx 0.1 m/s、1.6 Hz 的步態）、v2slow `20261125_155809`（匯出 `20261124_193736`，回放 0.05 m/s、1.0 Hz）。
- 韌體 0.2 的改動：`TELEM 1` 時 `PLAY` 每週期輸出一行 `PLT`（10 顆的位置、目標、電壓、溫度、負載、電流）；`T` 行末端加上每顆的電壓、溫度（共 85 欄，前 65 欄不變）；`STATUS` 的 `MODEL` 行加上 `build=`、`playback=`。
- `PLAY` 期間 PC 送出任何字元都會中止並關扭矩，所以程式等到 `PLAY done` 才送 `STOP`。Ctrl-C 一樣會送 `STOP`。
- 結束時 `STOP` 會關扭矩，腿和尾巴會垂下。紀錄在 `outputs/robot_v2/deploy/sessions/<時間>/`：`console.log`、`servo_telemetry.csv`、`test.txt`、`report.md`。
- 電流換算 6.5 mA／格是 HLS 系列的數值，**HD-1910 尚未校正**（`docs/backlog.md` #21，E4 用電表校正），表中標 `A*`。

## 紀錄

- 2026-11-25：建立；`hanging_test.py`／`.ps1` 從 `deploy/host/` 移來（同日新增，取代 2026-11-23 的 `demo_hanging.ps1`）。報表已用假資料驗證，**實機尚未跑**。
