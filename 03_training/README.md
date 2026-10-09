# 03_training：產生學生（訓練，實驗二）

**用途**：從 02 的老師出發，訓練出可以行走的策略（學生），並在**訓練範圍內**做驗收測試（平地、隨機化、凹凸地形、站立／啟停，部署門檻那一套）。
訓練範圍以外的測試在 [`04_generalization/`](../04_generalization/README.md)。2026-10-09 起，原 `03_training_pipeline/` 與 `04_sim2real/` 的訓練部分合併在這裡。

| 子資料夾 | 內容 | 規格 | 最終模型 | 產出 |
|---|---|---|---|---|
| [`training_pipeline/`](training_pipeline/README.md) | 平地：老師資料 → BC → PPO → 自我蒸餾 ×3（理想馬達、44 維觀測，需要 base 線速度） | `PROMPT.md`、`CLAUDE.md`（原始規格） | `iter3_distill_20261001_132024/model.pt` | `outputs/robot_v1/03_training/pipeline/` |
| [`sim2real/`](sim2real/README.md) | 強健性：特權 teacher PPO → 可部署 student（DAgger）；延遲、雜訊、摩擦、質量、凹凸地形、步態多樣性、左右對稱 | `sim2real/PROMPT.md`、`sim2real/CLAUDE.md` | **student v5**：`student_v5_20261007_003835/student.pt`（只用 IMU 與伺服讀值，最終部署候選） | `outputs/robot_v1/03_training/sim2real/` |

`sim2real` 以 `training_pipeline` 的最終模型為起點，並直接 import 它的網路、BC 與固定評估集設定。

---

# training_pipeline：平地訓練管線

**狀態：三輪完成（2026-10-01），使用者已檢視並確認（2026-10-06）。最終模型 `outputs/robot_v1/03_training/pipeline/iter3_distill_20261001_132024/model.pt`。** 機器人：`custom_biped` v1（tag `robot-v1`）。產出：`outputs/robot_v1/03_training/pipeline/`。

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
