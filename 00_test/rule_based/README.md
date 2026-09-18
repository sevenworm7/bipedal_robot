# 00_test/rule_based：規則式控制器（實驗一）

**狀態：尚未實作**（階段 C）。

- `PROMPT.md`、`CLAUDE.md`：從 `02_rule_based/` 複製過來的規格，測試過程中可以修改，原始版本保留在 02。
- 將會放在這裡的內容：`RuleBasedBipedController`（`reset()`、`compute_action(obs)`）、步態參數 config、評估腳本、說明文件。
- 介面：從 `00_test/shared/interface.py` 匯入，環境使用 `shared/env`。
- 驗收標準：100 個 episode 中成功率 ≥ 70%（未跌倒、至少 10 步、距離 ≥ 0.8 × 步數 × 步長），判定函式為 `shared/metrics.py` 的 `rule_based_success`。
