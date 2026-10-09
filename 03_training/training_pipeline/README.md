# 03_training/training_pipeline：訓練管線程式（實驗二）

collect → BC → PPO → 篩選 → 蒸餾 ×3 → 評估影片。程式從 `00_test/training_pipeline/` 移植（2026-10-01），邏輯相同；
檔案說明與設計重點（BC 直接訓練 rsl_rl `MLPModel`、PPO 凍結 actor 正規化器、critic 預熱、依 episode 切分）見 `00_test/training_pipeline/README.md`。

## 執行方式（專案根目錄，每個指令一個 Isaac Lab 行程）

| 步驟 | 指令 |
|---|---|
| 量測步態時鐘 | `tools\isaacpy.cmd 03_training\training_pipeline\measure_gait.py` |
| 收集老師資料 | `tools\isaacpy.cmd 03_training\training_pipeline\collect.py --iter 0` |
| BC | `tools\isaacpy.cmd 03_training\training_pipeline\bc.py --iter 0 --data <iter0_collect>\dataset.h5` |
| PPO | `tools\isaacpy.cmd 03_training\training_pipeline\ppo.py --iter 1 --init <iter0_bc>\model.pt` |
| 蒸餾 rollout | `tools\isaacpy.cmd 03_training\training_pipeline\collect.py --iter 1 --agent policy:<model_1499.pt> --tag ppo` |
| 篩選（先只統計） | `tools\isaacpy.cmd 03_training\training_pipeline\filter.py --iter 1 --data <dataset.h5> --stats_only` |
| 蒸餾 BC | `tools\isaacpy.cmd 03_training\training_pipeline\bc.py --iter 1 --stage distill --data <filtered.h5> --prev <iter0 dataset>` |
| 固定集評估 | `tools\isaacpy.cmd 03_training\training_pipeline\evaluate.py --iter 1 --stage ppo --agent policy:<ckpt>` |
| 評估影片 | 同上加 `--video_only --enable_cameras` |

產出在 `outputs/robot_v1/03_training/pipeline/iter<N>_<stage>[_<tag>]_<時間>/`，資料夾已存在時程式會停下來。

## 和實驗一的差異

| 項目 | 實驗一 | 實驗二 |
|---|---|---|
| 機器人／標籤 | `ref_biped`、`exp1` | `custom_biped`、`exp2`；影片標籤 `exp2 / robot_v1 / iter<N> / <stage>` |
| 觀測／動作 | 38／8 | 44／10（含尾巴） |
| 蒸餾 rollout 的探索雜訊 | 0.3 × policy std（PPO 結束時 std 只剩約 0.015，幾乎沒有雜訊） | **固定 σ = 0.05**（和老師資料一致，實驗一的建議） |
| 訓練推擠 | 0.2 m/s | 0.1 m/s |
| 推擠測試（評估，參考用） | 0.3 m/s | 0.15 m/s |
| 步態時鐘獎勵的擺動區間 | 0.075–0.275 | 0.05–0.50（`measure_gait.py` 實測老師，`PipelineCfg.env_overrides`） |
| 篩選門檻 | 側向 0.15 m 等 | 暫定側向 0.05 m，第 1 輪統計後和使用者決定 |
| 側向門檻掃描（`--stats_only`） | 0.05–0.30 m | 0.02–0.15 m |
| 新增 | — | `measure_gait.py`：量測任一 agent 在步態時鐘上的離地區間 |

## 執行紀錄

### 第 0 輪：步態時鐘與老師資料（2026-10-01）
- **步態時鐘**（`iter0_gait_20261001_093020`）：老師兩腳在腿相位 0.05–0.50 離地（P(air) 幾乎是 0 或 1），設為 `gait_swing_start` 0.05、`gait_swing_frac` 0.45。
- **第一次收集**（`iter0_collect_20261001_093116`，**作廢**）：4096 episode、408 萬步、成功率 99.2%、跌倒 0.2%、無 NaN。
  獎勵分項發現 `base_height` 每個 episode −1.43，相當於固定 6 cm 的高度誤差：環境量的是 base link 的 CoM（站姿 0.243 m），
  但 `NOMINAL_COM_HEIGHT` 誤設成全身 CoM（0.183 m，尾巴與腿把重心拉低）。跌倒門檻也因此偏低（0.110 → 0.146 m）。
  修正 `shared/robots/custom_biped.py` 後重跑 rule-based 評估與收集。
- **修正後**：
  - rule-based 重新評估（`rule_based/eval_cem1_comfix_20261001_093234`）：成功率 100%、跌倒 0%、1.95 m、側向 0.018 m、推擠 99%，和修正前相同。
  - 老師資料（**採用** `iter0_collect_20261001_093347`）：4096 episode、4,082,659 步、成功率 99.2%、跌倒 0.2%、無 NaN。
  - 老師每個 episode（20 s）的獎勵分項：gait_contact +38.2、track_vx +26.2、alive +10.0、track_heading +9.8、lateral_deviation −1.1；其餘都小於 0.2。
- **扭矩懲罰換算**：沿用實驗一的權重 −2e-4 時，老師的 torque 項每個 episode 只有 −0.0008（實驗一老師 −2.7）。v1 伺服的扭矩約是參考模型的 1/100，
  權重不換算的話能耗項等於沒有作用。改為 −0.7（`PipelineCfg.reward_overrides`），讓老師約 −2.8，和實驗一同量級。
  老師資料的 `rew_torque` 是用舊權重記錄的（BC 不使用獎勵，所以不需要重新收集）。

### 第 0 輪：BC（`iter0_bc_20261001_093456`）
- 3686／410 個 episode（依 episode 切分）、367 萬／41 萬步，60 epoch；驗證 MSE 0.00003（標籤變異數 0.332，R² 1.000）。
- 固定集評估：成功率 100%、跌倒 0%、1.95 m、速度比中位數 1.02、側向偏移中位數 0.014 m、方向誤差中位數 0.8°、推擠 99%，和老師相同。

### 第 1 輪：PPO（第一版 `iter1_ppo_20261001_093733`，留作對照）
- 4096 環境 × 1500 iterations、31 分鐘；載入 BC 後兩者輸出差異 0。第 50 iteration（預熱剛結束）已結束 episode 的跌倒率 32%，第 100 之後一直是 0；最終動作 std 0.01。
- 固定集評估（`iter1_eval_ppo_20261001_100902`）：成功率 100%、跌倒 0%、1.95 m、速度比 1.013，但**直線度比老師差**：
  最大側向偏移中位數 0.014 → 0.046 m（p90 0.037 → 0.087 m）、方向誤差中位數 0.8° → 1.7°（p10 也有 1.4°，表示有固定的偏航）；推擠 99%、跌倒 1%。
- 獎勵分項：PPO 比 BC 多拿 track_vx +2.1，只多扣 lateral_deviation −0.6。權重 −40 和 `sigma_heading` 0.2 rad 是依實驗一的大機器人設的，在 v1 上 5 cm 的偏移幾乎不扣分。
- 蒸餾 rollout（`iter1_collect_ppo_20261001_101045`，4096 episode、σ 0.05）＋只統計（`iter1_filterstats_20261001_101130`）：
  除側向外所有門檻 100% 通過；側向 < 0.03／0.04／0.05／0.06／0.08／0.10 m 分別 16／31／45／59／79／92%。
- **決定（使用者，2026-10-01）**：`lateral_deviation` −40 → −400、`sigma_heading` 0.2 → 0.05 rad，從同一個 BC 重跑第 1 輪 PPO（run tag `lat`）；篩選門檻在重跑後依分布決定。

### 第 1 輪：PPO 重跑（`iter1_ppo_lat_20261001_101741`，留作對照，**不採用**）
- 權重 `lateral_deviation` −400、`sigma_heading` 0.05 rad。前 200 iterations 平均 reward 約 −1.9（側向項每 episode 扣 55–70 分），300 之後才恢復。
- 固定集（`iter1_eval_ppo_lat_20261001_104915`）：成功率 **94%**、跌倒 0%、1.86 m、側向偏移中位數 0.052 m（p90 0.098）、方向 1.5°、速度比 p10 0.89、推擠 93%。
  蒸餾 rollout 有 30% 沒通過「步數與距離」（拖步）。**加重 10 倍沒有讓側向偏移變小，反而讓步態變差。**
- 修正後的判斷：**策略觀測不到自己的側向位置**（觀測只有方向誤差；老師用內部積分 `lat_est` 修正，MLP 沒有記憶），
  加重看不到的項目只會增加 return 的雜訊。實機也量不到側向位置（只有 IMU 方向），所以不考慮把它加進觀測。
- **決定（使用者，2026-10-01）**：第 1 輪採用第一版 PPO（`iter1_ppo_20261001_093733`），權重改回實驗一的值；直線度交給篩選與蒸餾逐輪改善。
- **篩選門檻（使用者，2026-10-01）**：第 1 輪側向 < 0.05 m，其他硬性門檻沿用實驗一；之後每一輪都先看「只統計」的分布再決定，不預先寫死。

### 第 1 輪：篩選與蒸餾
- **篩選**（`iter1_filter_20261001_105536`）：通過全部硬性門檻 45.1%（淘汰只來自側向 < 0.05 m）；各分組通過 658／723／468，取前 50% 後保留 329／362／234，共 925 episode、92 萬步。
- **蒸餾 BC**（`iter1_distill_20261001_105546`）：混入 233 個老師 episode（20%）；驗證 MSE 0.00020（R² 1.000）。
  固定集：成功率 100%、跌倒 0%、1.96 m、速度比 1.02、側向偏移中位數 **0.038 m**（p90 0.060；PPO 0.046／0.087）、方向 1.6°、推擠 **100%**。
  **比該輪 PPO 更直**（實驗一第 1 輪蒸餾比 PPO 差，改用固定雜訊 σ 0.05 後沒有再發生）。
- 第 1 輪 PPO 影片：`iter1_videos_ppo_20261001_105732`（慢／中／快，全部成功）。

### 第 2 輪
- **PPO**（`iter2_ppo_20261001_110534`，從第 1 輪蒸餾初始化，31 分鐘）。固定集（`iter2_eval_ppo_20261001_113712`）：成功率 99%、跌倒 1%、1.91 m、
  側向偏移中位數 0.039 m（p90 0.070）、方向 2.0°（p90 3.4°）、推擠 100%。保住了蒸餾的直線度，方向誤差略差。
- **只統計**（`iter2_filterstats_20261001_113917`）：側向 < 0.04／0.05／0.06／0.08 m 分別 27／44／59／78%；其餘門檻淘汰 ≤ 0.8%；
  0.05 m 時各分組通過 940／648／201（快速指令偏移較大）。
- **門檻（使用者，2026-10-01）**：維持 0.05 m。
- **篩選**（`iter2_filter_20261001_122302`）：通過 43.7%，保留 895 episode、89 萬步。
- **蒸餾 BC**（`iter2_distill_20261001_122314`）：混入 224 個第 1 輪篩選後的 episode；驗證 MSE 0.00012（R² 1.000）。
  固定集：成功率 100%、跌倒 0%、1.92 m、速度比 1.003、側向偏移中位數 **0.034 m**（p90 0.056）、方向 2.1°（p90 3.5°）、推擠 100%。
- 第 2 輪 PPO 影片：`iter2_videos_ppo_20261001_122430`（慢／中／快）。

### 第 3 輪
- **PPO**（`iter3_ppo_20261001_123337`，從第 2 輪蒸餾初始化，33 分鐘）。
- 固定集（`iter3_eval_ppo_20261001_130713`）：成功率 100%、跌倒 0%、2.01 m、速度比中位數 1.054（p90 1.13，比指令快約 5%）、
  側向偏移中位數 0.036 m（p90 **0.055**，各輪 PPO 最好）、方向 1.8°（p90 2.8°）、推擠 100%。
- **只統計**（`iter3_filterstats_20261001_130932`）：rollout 側向中位數 0.047 m（第 2 輪 0.054）；< 0.04／0.05 m 分別 36／56%，
  0.04 m 時各分組 397／631／463。
- **門檻（使用者，2026-10-01）**：收緊到 0.04 m（資料分布允許）。
- **篩選**（`iter3_filter_20261001_132012`）：通過 36.4%，保留 747 episode、75 萬步。
- **最終蒸餾 BC**（`iter3_distill_20261001_132024`）：混入 187 個第 2 輪篩選後的 episode；驗證 MSE 0.00009（R² 1.000）。
  固定集：成功率 99%、跌倒 1%、1.98 m、速度比 1.053、側向偏移中位數 0.034 m（p90 0.055）、方向 1.8°（p90 3.1°）、推擠 99%（跌倒 1%）。
- 影片：`iter3_videos_ppo_20261001_132130`、`iter3_videos_distill_20261001_132918`（慢／中／快，全部成功）；抽格確認 HUD 標籤與步數（10.6 秒 34 步）。

## 結論（2026-10-01）

各階段的固定集比較見 `outputs/robot_v1/summary.md`。
1. **管線在 v1 上可以直接運作**：BC 完全重現老師（R² 1.000），三輪 PPO 與蒸餾成功率都 ≥ 99%、跌倒 ≤ 1%，推擠測試 99–100%。
2. **直線度是主要的變化**：PPO 第一次就把側向偏移從老師的 0.014 m 拉高到 0.046 m；之後每輪的篩選＋蒸餾把它拉回來（p90 0.087 → 0.055 m），但沒有回到老師的水準。
   原因是策略觀測不到側向位置（老師有內部積分器），加重 `lateral_deviation` 反而變差（`iter1_ppo_lat`）。
3. **方向誤差**比老師大（中位數 0.8° → 1.6–2.1°），有固定的偏航；第 3 輪 PPO 走得比指令快約 5%。
4. 和實驗一相比：老師本身已經很好（成功率 100%），所以訓練的效益不在成功率，而在抗推擠（99 → 100%）與 PPO 帶來的步態多樣性；
   直線度目前**不如**老師。

## 事後修正（2026-10-06，03_training/sim2real 發現）
`shared/rollout.py` 的 `run_first_episodes` 原本先清空 `completed_episodes` 再 reset，導致同一行程中第二次 rollout（推擠測試）會混入上一次 rollout 還在跑的 episode。
本資料夾各輪的推擠測試只受到 1 個 episode 影響：成功率／跌倒率不變，距離、時間等平均值差 < 1%。資料收集每次只跑一輪，沒有受影響。修正見 `03_training/sim2real/README.md`。


## 產出清理（2026-10-07，使用者同意）
為釋放空間（專案 18.9 GB → 3.2 GB），刪除了以下不在 git 中的產出；完整清單見 `outputs/cleanup_list_20261007.txt`：
- 所有訓練資料集 `dataset.h5`（收集與篩選，12.9 GB）。需要時用相同指令與 seed 重新收集（模擬是確定性的）；04 的 PPO 與 DAgger 都在模擬中即時產生資料，不需要它們。
- 中間版本的影片（1.8 GB）。保留：各實驗的 rule-based 與最終模型、student v5 全部、student v3 跛行對照（`dr_rough_eval_v1`）、student v4 HUD 修正版；被刪影片的 run 仍保留 `videos.json`（各段指標）。
- 中間 checkpoint（203 個，0.9 GB）。每個 run 保留最後一個，另外保留被引用的 `teacher_v2/model_800.pt`（student v2 的來源）與 `teacher_v1/model_1000.pt`（第 1000 iteration 的評估）。
- 速度／煙霧測試 run（`teacher_speedtest`、`teacher_symtest`、`teacher_v5test`、`smoke`）。
所有 `config.yaml`、`stats.json`、`history.json`、`episodes.pt`、TensorBoard log 與 logs 都保留；刪除後重新載入 30 個保留模型，全部正常。
