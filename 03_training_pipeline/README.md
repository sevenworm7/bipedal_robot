# 03_training_pipeline：訓練管線（實驗二）

**狀態：三輪完成（2026-10-01），使用者已檢視並確認（2026-10-06）。最終模型 `outputs/robot_v1/training/iter3_distill_20261001_132024/model.pt`。** 機器人：`custom_biped` v1（tag `robot-v1`）。產出：`outputs/robot_v1/training/`。

- `PROMPT.md`、`CLAUDE.md`：原始規格（不修改）。
- `training_pipeline/`：管線程式（從 `00_test/training_pipeline/` 移植），執行方式、和實驗一的差異與執行紀錄見 [`training_pipeline/README.md`](training_pipeline/README.md)。
- 教師策略：`02_rule_based/rule_based/params/robot_v1_cem1_best.json`（100 episode 成功率 100%、跌倒 0%）。

## 開始前確認的事項（PROMPT.md「若有不確定之處」，使用者 2026-10-01 決定）

| 項目 | 決定 |
|---|---|
| rule-based 介面、觀測／動作 | 與 rule-based 相同：`shared/interface.py`，觀測 44 維、動作 10 維（腿 8 ＋ 尾巴 2） |
| 機器人 | `custom_biped` v1 |
| 地形與情境 | 平地；PPO 訓練時加輕推擠 0.1 m/s（每 4–8 秒、隨機方向）；指令 0.05–0.15 m/s、1.2–2.0 Hz；評估不推擠（推擠測試 0.15 m/s 僅供參考） |
| 篩選門檻 | **先跑第 1 輪 PPO 與「只統計」分析，依實際分布再和使用者決定**（老師的側向偏移只有 0.019 m，實驗一的 0.15 m 在這裡沒有篩選作用） |
| sim-to-real 隨機化 | 這一輪不加（伺服延遲、摩擦、質量、馬達增益），先照實驗一跑完三輪；之後另開一輪或在 robot v2 加入 |
