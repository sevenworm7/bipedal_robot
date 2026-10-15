# archive：已完成並封存的實驗

| 資料夾 | 內容 |
|---|---|
| [`exp1_reference/`](exp1_reference/README.md) | 實驗一（2026-09-29 使用者確認完成）：用 `references/` 的參考 URDF 驗證 rule-based → 訓練管線可行，程式之後移植到實驗二。結論見 `exp1_reference/outputs/summary.md` |

封存的實驗**不再修改程式**，以保持可重現（2026-10-15 從 `00_test/` 移到這裡時，只更新了文件中的路徑，以及 `shared/robots/ref_biped.py` 推算專案根目錄的那一行）。
實驗一的程式自成一套（有自己的 `shared/`、`rule_based/`、`training_pipeline/`），和實驗二的 `core/`、`simulation/`、`control/` 互不相依。
