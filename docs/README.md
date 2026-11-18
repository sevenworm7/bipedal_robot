# docs：跨領域的專案文件

2026-11-18 文件整理時新增。計畫、待辦、決策、成果這些**跨領域、會一直變動**的內容只寫在這裡，各資料夾的 README 不再重複。
檔名沿用使用者的 apply-framework 範本（`progress.md`、`architecture.md`），但沒有套用整套流程。

## 想知道什麼 → 看哪裡

| 想知道 | 看這裡 |
|---|---|
| 目前做到哪、下一步、各階段狀態 | [`progress.md`](progress.md) |
| 還沒解決的問題、待實物確認的項目 | [`backlog.md`](backlog.md) |
| 為什麼這樣決定 | [`decisions.md`](decisions.md) |
| 各版本、各階段的數字 | [`results.md`](results.md) |
| 資料夾分工、依賴方向、目錄結構 | [`architecture.md`](architecture.md) |
| 工作規則（給 Claude 與協作者） | 根目錄 [`CLAUDE.md`](../CLAUDE.md) |
| 某個機器人版本的規格、組裝、接線 | `hardware/robot_v<N>/`（README、CHANGELOG、ASSEMBLY） |
| v2 的腿長怎麼算出來的 | [`hardware/robot_v2/design_calc.md`](../hardware/robot_v2/design_calc.md) |
| 某個階段怎麼執行、過程紀錄 | 該資料夾的 README；sim2real 的過程在 `control/sim2real/log_robot_v<N>.md` |
| 某個階段的原始需求 | 該資料夾的 `PROMPT.md`（不修改） |
| 實機測試紀錄 | [`deploy/test_log.md`](../deploy/test_log.md) |
| 馬達通訊協定、暫存器表、上機前檢查 | [`hardware/design_refs/hd1910/`](../hardware/design_refs/hd1910/README.md)（`docs/feetech_SCS_protocol_v1.0.md`、`docs/feetech_STS_memory_table_v1.1.md`、`docs/hd1910_pretrain_checklist.md`） |

## 寫文件的規則

- **計畫與狀態**：只改 `progress.md`。完成一個可驗證的步驟就更新狀態；舊內容移到 Archive，前面保持精簡。
- **新發現的問題**：加到 `backlog.md`（不論領域），附日期、現象的數字或 run 路徑；解決後改狀態，不刪除。
- **使用者確認的決定**：加一列到 `decisions.md`；推翻時新增一列並標記舊列被取代。
- **數字**：每個階段完成後，把固定評估集的結果加到 `results.md`（進 git）。`outputs/` 裡的報告和影片仍是詳細版。
- **資料夾 README**：只寫用途、內容、執行方式和該階段的紀錄，連到這裡的文件，不寫「下一步」。
