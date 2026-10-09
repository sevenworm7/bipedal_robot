---
name: pipeline-iteration
description: Runbook for one round of the training pipeline (exp1 in 00_test/training_pipeline, exp2 in 03_training) (collect → BC → PPO → filter → distill → eval video) including artifact layout, go/no-go checks and the cross-iteration comparison. Use when running, resuming, or reporting on any training stage or self-distillation iteration.
---

# 訓練迭代流程

前置條件：該實驗的 `shared/robots/<robot>.py` 已定案；同一台機器人的 rule-based 控制器跌倒率 < 30%；自我蒸餾的篩選門檻已經使用者確認。

每次執行前，先確認這次屬於哪個實驗：實驗一在 `00_test/` 中使用 `ref_biped`；實驗二在 `03_training/training_pipeline/` 中使用 `custom_biped`（sim-to-real 強健性訓練在 `03_training/sim2real/`，流程見其 README）。確認 config 中的 `robot` 欄位正確，且產出寫入對應實驗的 outputs。

## 產出檔案結構
```
<產出資料夾>/                      # 實驗一：00_test/outputs/training/；實驗二：outputs/robot_v1/03_training/pipeline/
  iter0_collect_<date>/   dataset.h5, config.yaml, stats.json
  iter0_bc_<date>/        model.pt, config.yaml, tb/, stats.json
  iter1_ppo_<date>/       model_*.pt, config.yaml, tb/, stats.json, videos/
  iter1_filter_<date>/    dataset.h5, filter_report.json
  iter1_distill_<date>/   model.pt, ...
  ...
  summary.md              # 各輪比較表
```
每個 run 都要保存：當下的 config、隨機種子（seed）、起始 checkpoint 的路徑、`stats.json`。

## 單輪流程與檢查點
| 步驟 | 通過標準（未通過就停下並回報） |
|---|---|
| 收集資料 | 各情境都有資料；紀錄中沒有 NaN；episode 邊界正確 |
| BC | 驗證集 loss 已收斂；**在模擬中實際 rollout**，跌倒率與教師策略相近 |
| PPO | 載入 BC 權重後，第 0 個 iteration 的 reward 與 BC rollout 一致（驗證權重確實有載入）；reward 曲線上升，且跌倒率下降 |
| 篩選 | 報告保留比例與各門檻淘汰數量；保留的資料量足夠訓練 BC |
| 蒸餾 | 同 BC 的標準；且不應比該輪 PPO 策略差太多 |
| 評估 | 依 `/eval-video` 產出影片，並用固定 seed 的評估集計算指標 |

## 評估指標（每輪都一樣，寫入 `stats.json`）
使用固定的評估集（固定 seed、固定指令與地形組合，至少 100 個 episode）：
- 跌倒率、平均行走距離、平均前進速度與指令速度的誤差
- 平均與最大側向偏移（直線度）、平均扭矩或能耗
- 平均步數與步頻

## 跨輪比較
每輪結束後更新 `summary.md`，用表格列出各輪指標，並附上影片路徑。若某輪表現退步，要回報可能原因（例如篩選過嚴、資料分布變窄、BC 過擬合），不可跳過。

## 實驗結束
- 實驗一結束後，在 `00_test/outputs/summary.md` 整理：管線各階段是否有效、需要修正的地方、哪些問題來自參考模型本身的限制。實驗二開始前，先和使用者一起檢視這份整理。
- 實驗二結束後，把兩次實驗的最終指標並列比較。

## 執行方式
- 長時間訓練使用 `run_in_background`，並將 stdout 寫入 run 資料夾。
- 中斷後恢復時，從 run 資料夾的 config 與最新的 checkpoint 接續，不要覆寫既有的 run。
