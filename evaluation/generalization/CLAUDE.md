# evaluation/generalization：泛化與壓力測試（實驗二）

- **只評估，不訓練**：測試對象來自 `control/` 的模型（checkpoint 路徑寫在報告裡）；要改進模型就回到 `control/sim2real` 訓練新版本，再回來重測。
- **一次只改一個條件**，其餘維持理想平地（`flat_v1`），才能分清各因素的影響；組合條件另外定義成新的測試版本。
- **測試條件是帶版本的場景**（`simulation/env/scenarios.py` 的 `STRESS_V<N>`）：用過即凍結，要改就開 `STRESS_V<N+1>`；`simulation/env` 的新功能一律預設關閉，
  修改後用 `control/sim2real/evaluate.py --compare` 重跑 `flat_v1` 回歸檢查，必須完全一致。
- **地形有高低變化（斜坡、台階）時，跌倒判定必須以腳下地面為基準**（`fall_height_ref="feet"`）：預設以出生點高度為基準，往下走約 10 cm 就會被誤判為跌倒（2026-10-09 在 `stress_v1` 發現，以 `stress_v2` 重做）。結果異常時先檢查判定本身，再下結論。
- **成功率與跌倒率以 20 秒、100 episode 的統計為準**；影片是 10 秒片段，只用來看步態，可能錯過較晚發生的失敗。
- **結果不單調或和相鄰條件差很多時，先重跑確認可重現**（同 seed 的模擬是確定性的，`evaluate.py --compare` 可逐項比對）；2026-10-15 `stress_v3` 有一次執行受干擾。**同一時間只跑一個模擬程式**（共用 GPU）。
- 結果接近門檻時（±4% 是 100 episode 的統計誤差）加大樣本（`evaluate.py --episodes 504`）再下結論。
- 產出：`outputs/robot_v1/evaluation/generalization/<測試版本>_<日期>/`，資料夾已存在時程式會停下或續跑（不覆寫已完成的條件）。
