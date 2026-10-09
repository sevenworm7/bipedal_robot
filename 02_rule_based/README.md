# 02_rule_based：規則式行走控制器（實驗二）

**狀態：完成，使用者已檢視影片並確認（2026-10-01）。100 episode 成功率 100%、跌倒 0%，採用 `rule_based/params/robot_v1_cem1_best.json`。** 機器人：`custom_biped` v1（tag `robot-v1`）。產出：`outputs/robot_v1/02_rule_based/`。

- `PROMPT.md`、`CLAUDE.md`：原始規格（不修改）。
- `rule_based/`：控制器與所有腳本（從 `00_test/rule_based/` 移植），說明與開發紀錄見 [`rule_based/README.md`](rule_based/README.md)。
- 介面與環境：根目錄 `shared/`（觀測 44 維、動作 10 維、50 Hz）。

## 開始前確認的事項（PROMPT.md「若有不確定之處」，使用者 2026-09-30 決定）

| 項目 | 決定 |
|---|---|
| 機器人 | `custom_biped` v1：每腿 4 DOF ＋ 2 DOF 尾巴，URDF 在 `01_robot_construction/model/v1/urdf/`，USD 在 `shared/robots/usd/custom_biped_v1/` |
| 地形 | 只用平地 |
| 觀測／動作維度 | 尾巴放進介面：J = 10，觀測 14 + 3 × 10 = 44 維、動作 10 維；結構與順序規則和實驗一相同 |
| 指令範圍 | 速度 0.05–0.15 m/s、步頻 1.2–2.0 Hz（依 v1 靜態檢查：每步最多 ±78 mm） |

## 驗收標準

與實驗一相同：100 個 episode（指令、步頻、初始關節角隨機），成功率 ≥ 70%（未跌倒、至少 10 步、距離 ≥ 0.8 × 步數 × 步長；也就是跌倒率 < 30%），
並統計平均行走距離、速度與方向誤差；3–5 段評估影片（`/eval-video` 格式）。

## 發現的模型問題

rule-based 過程中發現的機器人問題記到 `01_robot_construction/model/v2_backlog.md`（v1 已凍結）。
