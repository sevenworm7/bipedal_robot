# deploy/host：PC 端指令與遙測工具

透過 USB 和芒果磚上的 v2 韌體（[`deploy/firmware`](../firmware/README.md)）溝通：送指令、即時顯示回覆，並把每一行記錄下來。初期實機測試都用 USB 連 PC（使用者 2026-11-19 決定）。

## 執行

```powershell
# 用專案內的 .venv_fw（已裝 pyserial）；自動以 USB 名稱「BIPED V2 RUNTIME」找板子
.venv_fw\Scripts\python.exe deploy\host\robot_cli.py
.venv_fw\Scripts\python.exe deploy\host\robot_cli.py --port COM5
# 只讀的自我測試：送完指令 3 秒後結束
.venv_fw\Scripts\python.exe deploy\host\robot_cli.py --run PING STATUS BENCH "BUSTEST 100" --exit-after 3
```

- 每次執行建立 `outputs/robot_v2/deploy/sessions/<時間>/`：`console.log`（所有收發）、`telemetry.csv`（`TELEM 1` 時的每週期資料，欄位見程式開頭的 `HEADER`）。
- **Ctrl-C 或程式結束時一定會送 `STOP`**，扭矩會關掉、機器人不再被支撐，測試前先扶好或吊好。

## `demo_hanging.ps1`：懸空 demo（2026-11-23 測過的兩個流程）

| 模式 | 流程 | 時間 |
|---|---|---|
| `play` | `INIT` → 站姿下以 `-Rate` 倍速開迴路播放模擬中 student 的走路軌跡（`PLAY`）→ 回站姿 → `STOP` | 0.5 倍約 25 秒 |
| `walk` | `INIT` → 模型閉迴路 `WALK`（實際速度）`-Seconds` 秒 → `STAND` → `STOP` | 6 秒時約 22 秒 |

```powershell
# 0.5 倍慢速回放
powershell -ExecutionPolicy Bypass -File deploy\host\demo_hanging.ps1 -Mode play -Rate 0.5
# 模型全速，懸空；目前供電不足，要暫時把電壓門檻降到 5.5 V 才走得完
powershell -ExecutionPolicy Bypass -File deploy\host\demo_hanging.ps1 -Mode walk -Vx 0.05 -Freq 1.6 -Seconds 6 -VStop 5.5
# 只印出指令序列，不連線
powershell -ExecutionPolicy Bypass -File deploy\host\demo_hanging.ps1 -Mode walk -DryRun
```

- 開始前會問一次「機器人是否懸空、有人扶住」，要輸入 `yes`（`-Yes` 可略過）。
- 參數範圍：`-Rate` 0.05–1、`-Vx` 0–0.15、`-Freq` 0.8–2.0（實際範圍由韌體依燒錄的模型檢查）、`-Seconds` 1–30、`-VStop` 5.5–7.0（預設 6.5；低於 6.5 只限懸空）。 INIT 後等 15 秒再 WALK（尾巴下垂 −75° 時 INIT 需要 11.3 秒，2026-11-24）。
- 結束時 `STOP` 會關扭矩，腿和尾巴會垂下。紀錄在 `outputs/robot_v2/deploy/sessions/<時間>/`。
- 需要燒錄 2026-11-23 之後的韌體（有 `PLAY`、`VSTOP`）。

## `jog_analyze.py`：單顆 JOG 分析

```powershell
.venv_fw\Scripts\python.exe deploy\host\jog_analyze.py [session 資料夾]   # 預設最新的 session
```

從 `console.log` 讀 `JOG` 的 100 Hz 紀錄，算方向、靜止誤差、最大追隨誤差、延遲（最小平方擬合 0–200 ms）、速度暫存器和差分速度的比值。

## 紀錄

- 2026-11-23：新增 `demo_hanging.ps1`（`play`／`walk` 兩個懸空 demo）。
- 2026-11-23：`--run` 支援 `WAIT <秒>`（PC 端暫停，例如等 `INIT` 到站姿再送 `PLAY`／`WALK`）。
- 2026-11-23：第一次接實機；遙測加上每顆的負載、電流暫存器（65 欄）；新增 `jog_analyze.py`。

- 2026-11-19：建立；語法檢查通過，尚未接實機。
