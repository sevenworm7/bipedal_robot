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

## `jog_analyze.py`：單顆 JOG 分析

```powershell
.venv_fw\Scripts\python.exe deploy\host\jog_analyze.py [session 資料夾]   # 預設最新的 session
```

從 `console.log` 讀 `JOG` 的 100 Hz 紀錄，算方向、靜止誤差、最大追隨誤差、延遲（最小平方擬合 0–200 ms）、速度暫存器和差分速度的比值。

## 紀錄

- 2026-11-23：第一次接實機；遙測加上每顆的負載、電流暫存器（65 欄）；新增 `jog_analyze.py`。

- 2026-11-19：建立；語法檢查通過，尚未接實機。
