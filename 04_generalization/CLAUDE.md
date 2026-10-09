# 04 泛化與壓力測試（實驗二）

- **只評估，不訓練**：測試對象來自 `03_training` 的模型（checkpoint 路徑寫在報告裡）；要改進模型就回到 03 訓練新版本，再回來重測。
- **一次只改一個條件**，其餘維持理想平地（`flat_v1`），才能分清各因素的影響；組合條件另外定義成新的測試版本。
- **測試條件是帶版本的場景**（`shared/env/scenarios.py` 的 `STRESS_V<N>`）：用過即凍結，要改就開 `STRESS_V<N+1>`；`shared/env` 的新功能一律預設關閉，
  修改後用 `03_training/sim2real/evaluate.py --compare` 重跑 `flat_v1` 回歸檢查，必須完全一致。
- **成功率與跌倒率以 20 秒、100 episode 的統計為準**；影片是 10 秒片段，只用來看步態，可能錯過較晚發生的失敗。
- 結果接近門檻時（±4% 是 100 episode 的統計誤差）加大樣本（`evaluate.py --episodes 504`）再下結論。
- 產出：`outputs/robot_v1/04_generalization/<測試版本>_<日期>/`，資料夾已存在時程式會停下或續跑（不覆寫已完成的條件）。
