# 02 Rule-based 控制器（實驗二）

完整需求在 `PROMPT.md`，動工前先讀完。實驗一（`00_test/rule_based/`）也使用同一份規格，以下摘要同樣適用。

**實驗一完成前，不要在這個資料夾中建立程式碼。** 實驗二的做法是移植 `00_test/rule_based/` 已驗證的程式碼，只把 robot 設定換成 `custom_biped`。

- 產出 `RuleBasedBipedController`（提供 `reset()` 與 `compute_action(obs) -> action`），作為訓練管線的教師策略。
- 控制器中所有和機器人相關的數值（腿長、關節順序、站姿、步態參數預設值）都從 `shared/robots/<robot>.py` 或 config 讀取，**不可寫死**；IK 以機器人的運動學參數計算。
- 觀測／動作空間從該實驗的 `shared/` import，不可在這裡另外定義（見根目錄 CLAUDE.md「跨流程介面契約」）。
- 可調參數（放在 config）：步長、抬腳高度、步頻／步態週期、目標方向與速度。
- 必須有平衡修正機制，例如依 base 傾角調整關節目標，或依 COM／capture point 調整落腳點。
- 驗收標準：評估 50–100 組 rollout（初始姿態、速度指令隨機化），跌倒率 < 30%；需統計平均行走距離、平均速度與方向誤差。
- 交付項目：程式碼＋config、統計報告、3–5 段評估影片（依 `/eval-video` 格式）、說明文件（觀測／動作定義、參數清單、已知限制）。
- 評估時使用大量平行環境（Isaac Lab vectorized env），不要逐一執行 rollout。
- 產出放在 `<產出根目錄>/rule_based/<run>/`（實驗一：`00_test/outputs/`；實驗二：`outputs/`）。
