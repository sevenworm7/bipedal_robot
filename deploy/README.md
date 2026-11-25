# deploy：實體機器人（實驗二，進行中）

把模擬中訓練好的策略（robot v2：`control/sim2real` 的 student v2ft，`docs/decisions.md` #23；慢步態 student v2slow 作對照，#30）放到 3D 列印的實體機器人上走起來。
這裡的程式在實機或 PC 上執行，**不可依賴 Isaac Lab**（`export/` 的匯出程式例外：只在 PC 上用來產生檔案）。
控制板是「芒果磚」MANGO X4（RP2350，台灣的開發板）；板上程式由本專案負責（使用者 2026-11-19 決定）。

- **實機測試紀錄**：[`test_log.md`](test_log.md)。
- **各步驟的狀態**：[`docs/progress.md`](../docs/progress.md)「實機部署的子步驟」；待辦問題在 [`docs/backlog.md`](../docs/backlog.md)。
- **伺服供電**：見 [`power_supply.md`](power_supply.md)（唯一的規格來源）。**目前使用 SD026**（2026-11-25 起；12 V → 8 V，持續 15 A／最大 20 A，限流要調到 ≥ 15 A）；舊的 HW-133A（3 A）**已停用**，2026-11-24 以前的實機壓降數據都是舊模組量到的。
- **馬達資料**：通訊協定、暫存器表、上機前檢查表在 [`hardware/design_refs/hd1910/docs/`](../hardware/design_refs/hd1910/README.md)。
- **組員的 v1 交接包**（懸空回放韌體、實測的 ID／方向／零點、IMU 與供電紀錄）：`references/mango_x4_handoff_20261117/`，唯讀、不進 git（4.1 GB）。要用其中的程式或資料時複製到這裡，並註明來源路徑與 SHA256（`docs/decisions.md` #26）。

## 子資料夾

| 資料夾 | 內容 |
|---|---|
| [`export/`](export/README.md) | 把學生模型匯出成 C 標頭檔（int8 權重放 SRAM）＋標準答案測試資料 |
| [`firmware/`](firmware/README.md) | 芒果磚上的 50 Hz 閉迴路程式（C，Pico SDK）與 PC 單元測試 |
| [`host/`](host/README.md) | PC 端通用 USB 指令與遙測記錄工具（`robot_cli.py`），其他資料夾的測試程式都透過它或引用它的欄位定義 |
| [`calibration/`](calibration/README.md) | 實機校正檔（伺服 ID、方向、零點、IMU 軸向）：2026-11-23 實測，`verified: true`；hip roll（偏約 3.3°）與尾巴零點待確認 |
| [`actuator_bench/`](actuator_bench/README.md) | 單顆馬達測試（步驟 2）：暫存器、步階響應、正弦掃頻（E0–E2）、JOG 分析，和模擬馬達模型比對 |
| [`field_tests/`](field_tests/README.md) | 整機測試（步驟 5–7）：分階段懸空測試 `hanging_test`（開迴路回放／閉迴路 WALK＋每顆馬達電壓電流表） |

## 步驟

| 步驟 | 子資料夾 | 內容 | 通過條件 |
|---|---|---|---|
| 1. 採購與試印 | — | 依 `hardware/robot_v1/reports/parts_bom.md` 備料、試印關鍵件、秤重、確認孔位與馬達轉向（`docs/backlog.md` #3、#4） | 實測密度回填 |
| 2. 單顆馬達台架測試 | [`actuator_bench/`](actuator_bench/README.md) | 量測反應延遲（要求 ≤ 40 ms，`docs/decisions.md` #25）、追隨能力、受力偏移、回讀電流，和模擬的馬達模型（kp、kd、延遲範圍 0–40 ms）比對 | 落在訓練的隨機化範圍內；否則回到 `control/sim2real` 微調 |
| 3. 組裝與校正 | `calibration/` | 零點、轉向、IMU 軸向；量總重與重心；站姿和模擬比對 | 校正檔 `verified: true` |
| 4. 控制程式 | `export/`、`firmware/`、`host/` | 在板上執行 student v2ft 或 v2slow（int8）、50 Hz 控制迴圈、**站立用固定站姿、行走才交給模型**（`docs/decisions.md` #24）、安全停止 | 板上推論和 PC 一致；讀取＋推論＋寫入 < 20 ms |
| 5. 懸吊測試 | [`field_tests/`](field_tests/README.md) | 吊起來、腳不著地，確認方向與步態；再加安全繩放到地上 | 方向正確、不抖動 |
| 6. 平地行走 | `field_tests/` | 正式走路，和模擬比較速度、直線度、步頻、馬達負載與溫度 | 直線走完 2 m |
| 7. 進階測試 | `field_tests/` | 凹凸地面、推擠、長時間溫升 | — |

步驟 2 之前的模擬準備（馬達負載分析、`stress_v3`）在 [`evaluation/`](../evaluation/README.md)。產出放在 `outputs/robot_v<N>/deploy/`（匯出、韌體建置、實機 session 紀錄）。

## 整理紀錄

- 2026-11-25：新增 `field_tests/`（`hanging_test` 從 `host/` 移入，取代 `demo_hanging.ps1`）；`jog_analyze.py` 從 `host/` 移到 `actuator_bench/`；更新校正狀態與供電。
