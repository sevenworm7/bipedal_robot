# 03_training/sim2real：Sim-to-real 強健性訓練（實驗二）

完整需求在 `PROMPT.md`，動工前先讀完；「若有不確定之處」的項目要先問使用者。
這部分是使用者 2026-10-06 決定新增的（平地訓練完成後，不做 robot v2，直接針對實機部署加強訓練）；原為 `04_sim2real/`，2026-10-09 併入 03。
訓練範圍以外的測試不放這裡，放 `04_generalization/`。

## 規則

- **不可覆蓋舊場景**（使用者 2026-10-06 強調）：
  - `shared/env` 的新功能一律預設關閉；關閉時的行為要和 `training_pipeline` 完全相同。每次修改 `shared/env` 後，用 `training_pipeline` 最終模型在 `flat_v1` 固定評估集重跑，
    數字必須和 `outputs/robot_v1/03_training/pipeline/` 中的紀錄一致，否則不可繼續。
  - 環境設定以具名、帶版本的場景管理；場景被正式 run 使用後就凍結，修改要開新版本（`_v2`），舊版保留。
  - 每個 run 的 config 記錄場景名稱、介面版本、起始 checkpoint 與 git commit。
- **介面**：新的觀測介面另開版本，`shared/interface.py` 現有的 44 維定義不動（02、03 依賴它）。
  rule-based 與 `training_pipeline` 的程式碼不需要因為 sim2real 而修改。
- **產出**：`outputs/robot_v1/03_training/sim2real/<stage>_<tag>_<YYYYMMDD_HHMMSS>/`，資料夾已存在時停下來；結果整理進 `outputs/robot_v1/summary.md`。
- **步態多樣性**（使用者 2026-10-06 指出）：目前步態過於單一，是實機失效的主要風險之一；每一輪訓練都要量化步態多樣性與擾動恢復，不只看成功率。

## `training_pipeline` 留下的經驗（直接沿用）

- 一個行程只能建立一個 Isaac Lab 環境：訓練、評估、錄影、資料收集要分開執行。
- **策略觀測不到的量，不要用重懲罰去逼**：`training_pipeline` 加重 `lateral_deviation`（策略看不到側向位置）反而讓成功率掉到 94%。同理，特權資訊只能給 critic。
- **獎勵權重要依機器人尺寸換算**：v1 的扭矩約是參考模型的 1/100，權重沿用會讓該項失效；新增任何項目前，先用現有策略跑一次看各項的量級。
- `NOMINAL_COM_HEIGHT` 是 base link 的 CoM（0.243 m），不是全身 CoM。
- 用固定評估集（固定 seed）比較各階段；長時間訓練一律 `run_in_background`，stdout 寫入 log。
- `training_pipeline` 的 PPO 每輪 4096 環境 × 1500 iterations 約 31 分鐘；加入地形與隨機化後會變慢，先用小規模確認再放大。
