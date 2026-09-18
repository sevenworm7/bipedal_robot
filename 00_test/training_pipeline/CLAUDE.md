# 訓練管線（實驗一，複製自 `03_training_pipeline/`）

完整需求在同資料夾的 `PROMPT.md`，動工前先讀完。實驗一的篩選門檻等決定見 `00_test/CLAUDE.md`，若和本檔不同，以 `00_test/CLAUDE.md` 為準。

本檔與 `PROMPT.md` 是 `03_training_pipeline/` 的複本，測試過程中可以依實際情況修改；原始版本保留在 `03_training_pipeline/` 不動，實驗一結束後再和使用者一起比較差異。

- 管線程式碼**不綁定特定機器人**，只透過 config 的 `robot` 欄位切換；獎勵權重等超參數可以依機器人分別設定 config，但程式邏輯要共用。
- 實驗一的目的是驗證管線是否可行：遇到問題時，要記錄是管線本身的問題，還是參考模型的限制（例如腳掌太重、缺少自由度），以便實驗二參考。

1. **資料收集**：執行同一個實驗的 rule-based 控制器，把 (state, action) 存成 HDF5，並記錄 episode 邊界與情境參數。
2. **BC**：用 MLP 監督式學習（MSE／NLL），**依 episode 切分** train/val（不可依 step 切分，否則會資料洩漏）。
3. **PPO**：以 BC 權重初始化 actor（rsl_rl 5.0.1），獎勵項與權重要寫在 config 中並列出。
   - BC 的 MLP 架構、激活函數、觀測正規化方式必須與 rsl_rl actor 完全一致，才能直接載入權重。
   - 初始的 action std 要設小一點，避免一開始就把 BC 學到的行為破壞掉。
4. **自我蒸餾**：rollout → 依量化門檻篩選 → BC → 作為下一輪 PPO 的初始化，**共 3 輪**。篩選門檻要先和使用者確認。
5. **評估影片**：每輪都要產出，格式依 `/eval-video`，檔名例如 `exp1_iter3_ppo_eval_20260918.mp4`。

- 跨輪流程依 `/pipeline-iteration` skill 執行。每輪的 checkpoint、log、config、影片放在 `<產出根目錄>/training/iter<N>_<stage>_<date>/`（實驗一：`00_test/outputs/`；實驗二：`outputs/`）。
- TensorBoard 至少要記錄：reward 各分項、episode length、跌倒率。
- 最終交付：三輪之間的指標比較（跌倒率、行走距離、直線度、步態品質）。
