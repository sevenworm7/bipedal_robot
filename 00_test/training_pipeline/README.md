# 00_test/training_pipeline：訓練管線（實驗一）

**狀態：尚未實作**（階段 D）。rule-based 已完成並經使用者確認（2026-09-19），下一步。

- `PROMPT.md`、`CLAUDE.md`：從 `03_training_pipeline/` 複製過來的規格，測試過程中可以修改，原始版本保留在 03。
- 將會放在這裡的內容：資料收集（HDF5）、BC、PPO（rsl_rl，以 BC 權重初始化）、篩選、蒸餾 ×3 輪、評估影片。
- 環境：`00_test/shared/env`（gym id `Biped-Test-Walk-v0`），觀測 38 維、動作 8 維，獎勵權重在 `BipedWalkEnvCfg.rewards`。
- 教師策略：`rule_based.RuleBasedBipedController` ＋ `rule_based/params/cem2_best.json`（100 episode 成功率 95%、跌倒 0%，見 `rule_based/README.md`）。
- 篩選門檻（暫定）與「篩選前先統計、資料太少就停下來討論」的規則，見 `00_test/CLAUDE.md`。
