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

## 紀錄

- 2026-11-19：建立；語法檢查通過，尚未接實機。
