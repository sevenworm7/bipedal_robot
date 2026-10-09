# 00_test/training_pipeline：訓練管線（實驗一）

**狀態：完成（2026-09-23）。三輪自我蒸餾跑完，最終模型成功率 99%、跌倒 0%、推擠 100%。**進度與數據見下方「執行紀錄」。

- `PROMPT.md`、`CLAUDE.md`：從 `03_training/` 複製過來的規格，測試過程中可以修改，原始版本保留在 03。
- 環境：`00_test/shared/env`（gym id `Biped-Test-Walk-v0`），觀測 38 維、動作 8 維，獎勵權重在 `BipedWalkEnvCfg.rewards`。
- 教師策略：`rule_based.RuleBasedBipedController` ＋ `rule_based/params/cem2_best.json`（100 episode 成功率 95%、跌倒 0%）。
- 決定（2026-09-22，使用者確認）：平地＋直線；PPO 只加輕微推擠（0.2 m/s，每 4–8 秒）；每輪 PPO 4096 環境 × 1500 iterations；每輪 PPO 與蒸餾 BC 都用同一組固定評估集。
- 篩選門檻（暫定）與「篩選前先統計、資料太少就停下來討論」的規則，見 `00_test/CLAUDE.md`。

## 檔案

| 檔案 | 內容 |
|---|---|
| `config.py` | 所有超參數：`ActorCfg`（BC 與 PPO 共用的網路）、`CollectCfg`、`BCCfg`、`PPOCfg`、`FilterCfg`、`EvalCfg` |
| `policy.py` | 建立 rsl_rl `MLPModel` actor、設定觀測正規化、存讀 checkpoint；`TeacherAgent`、`PolicyAgent`、`load_agent` |
| `dataset.py` | 平行收集完整 episode 的 (state, action)，HDF5 讀寫，依 episode 取子集 |
| `common.py` | run 資料夾、`config.yaml`、建立環境 |
| `evaluation.py` | 固定評估集（100 episode、seed 42、另有推擠測試），`stats.json` 格式和 rule-based 相同 |
| `collect.py` | 階段一：收集資料（老師，或蒸餾用的 PPO rollout） |
| `bc.py` | 階段二／蒸餾：BC 訓練＋模擬中評估 |
| `ppo.py` | 階段三：以 BC 初始化的 PPO（rsl_rl OnPolicyRunner） |
| `filter.py` | 階段四：篩選（`--stats_only` 只統計）；純 Python，不需要模擬器 |
| `evaluate.py` | 任一 agent 的固定集評估；`--video_only --enable_cameras` 錄評估影片 |

## 設計重點

- **state / action**：即 `shared/interface.py` 的 38 維觀測與 8 維動作（正規化關節目標角偏移，`q = q_default + 0.25 × a`）。
- **網路**：MLP 512-256-128、ELU，輸入經 `EmpiricalNormalization`。BC 直接訓練 rsl_rl 的 `MLPModel`（和 PPO actor 同一個類別、同樣的參數），所以 BC checkpoint 可以原封不動載入 PPO；`ppo.py` 載入後會比對兩者輸出（實測差異 0）。
- **正規化**：BC 用訓練集的平均與變異數設定正規化器；PPO 期間凍結 actor 的正規化器（critic 的正規化器正常更新）。
- **BC loss**：MSE（正規化動作單位）；**依 episode 切分** 90% 訓練／10% 驗證；Adam 1e-3、cosine 衰減、60 epochs，保留驗證 loss 最低的模型。
- **老師資料**：4096 個完整 episode（約 409 萬步），執行時對動作加 σ = 0.05 的雜訊，標籤用老師的乾淨動作（DART），讓資料包含從偏離狀態修正回來的示範。
- **PPO**：初始動作 std 0.2（= 0.05 rad）；先凍結 actor、只訓練 critic 50 iterations，再重設學習率（3e-4、adaptive KL 0.01）訓練全部；訓練時有輕微推擠；episode 起始時間隨機化。
- **蒸餾**：PPO policy rollout（執行動作 = 平均 + 0.3 × std × 雜訊，標籤 = 平均動作）→ 篩選 → 從頭訓練新的 MLP，混入 20% 上一輪資料 → 作為下一輪 PPO 的初始化。

## 獎勵（`BipedWalkEnvCfg.rewards`，每項 × control dt）

| 項目 | 權重 | 內容 |
|---|---|---|
| track_vx | 1.5 | exp(−((v_x − cmd)/0.1)²) |
| track_heading | 0.5 | exp(−(方向誤差/0.2)²) |
| lateral_vel | −1.0 | v_y² |
| lateral_deviation | −40（2026-09-23 前為 −10） | 側向偏移² |
| orientation | −2.0 | 重力投影 xy 分量² |
| base_height | −20 | (CoM 高度 − 0.36)² |
| alive | 0.5 | 每步 |
| termination | −20 | 跌倒 |
| torque | −2e-4 | Σ τ² |
| action_rate | −0.01 | Σ (a_t − a_{t−1})² |
| joint_limits | −2.0 | 超出軟極限的量 |
| feet_air_time | 1.0 | 著地時 (騰空時間 − 0.3 s) |
| contact_force | −1e-3 | 超過 400 N 的接觸力 |
| gait_contact | 2.0 | 兩腳平均的「接觸狀態符合步態時鐘」：時鐘相位在該腳的擺動區間（左腳 0.075–0.275，右腳 +0.5）時應離地，其餘時間應著地。2026-09-22 加入（使用者同意），區間取自老師實測的離地機率 |

老師每個 episode 的 gait_contact 為 19.0／20（舊的第 1 輪 PPO 17.2）。老師（含雜訊）其他平均分項（20 s）：track_vx +17.0、alive +10.0、track_heading +8.8、lateral_vel −3.9、torque −2.7、lateral_deviation −1.3，其餘都小於 0.3。

## 執行順序（每一輪）

```powershell
# iteration 0：老師資料 → BC
tools\isaacpy.cmd 00_test\training_pipeline\collect.py --iter 0
tools\isaacpy.cmd 00_test\training_pipeline\bc.py --iter 0 --data <iter0_collect>\dataset.h5
# iteration N = 1..3
tools\isaacpy.cmd 00_test\training_pipeline\ppo.py --iter N --init <上一輪 BC 或蒸餾>\model.pt
tools\isaacpy.cmd 00_test\training_pipeline\evaluate.py --iter N --stage ppo --agent policy:<iterN_ppo>\model_1499.pt
tools\isaacpy.cmd 00_test\training_pipeline\evaluate.py --iter N --stage ppo --agent policy:<ckpt> --video_only --enable_cameras
tools\isaacpy.cmd 00_test\training_pipeline\collect.py --iter N --agent policy:<ckpt> --tag ppo
tools\isaacpy.cmd 00_test\training_pipeline\filter.py --iter N --data <iterN_collect_ppo>\dataset.h5 --stats_only   # 第一次篩選前必做
tools\isaacpy.cmd 00_test\training_pipeline\filter.py --iter N --data <iterN_collect_ppo>\dataset.h5
tools\isaacpy.cmd 00_test\training_pipeline\bc.py --iter N --stage distill --data <iterN_filter>\dataset.h5 --prev <上一輪資料>
```
TensorBoard：`tools\isaacpy.cmd -m tensorboard.main --logdir 00_test\outputs\training`。

## 執行紀錄

### iteration 0（2026-09-22）
- **收集**：4096 episode、4,091,199 步，跌倒 0.1%、成功率 83%（雜訊造成多餘的著地，不是跌倒）、無 NaN。
- **BC**：最佳驗證 MSE 0.00019（標籤變異數 0.485，R² ≈ 0.9996）。模擬評估和老師幾乎相同：

| 指標 | 老師（rule-based） | BC iter0 |
|---|---|---|
| 成功率 | 95% | 95% |
| 跌倒率 | 0% | 0% |
| 速度比中位數 | 1.02 | 1.03 |
| 最大側向偏移中位數 | 0.136 m | 0.134 m |
| 最大方向誤差中位數 | 7.2° | 6.7° |
| 推擠測試成功率 | 88% | 92% |

- **篩選門檻預查**（用老師資料跑 `filter.py --stats_only`，僅供參考）：通過全部硬性門檻 23.8%；側向偏移 < 0.15 m 淘汰 51%、方向誤差 < 10° 淘汰 54%；快速分組只剩 14 個。側向偏移 < 0.12 m 只有 3%、< 0.08 m 為 0%。

### iteration 1，第一版（對照組，原獎勵，2026-09-22）
- **PPO**：1500 iterations、約 29 分鐘。訓練中 reward 19.7 → 45.7，速度追蹤 0.72 → 1.48（上限 1.5），最大側向偏移 0.24 → 0.11 m，跌倒 0%。動作 std 從 0.2 降到 0.01（探索幾乎消失）。
- **固定集評估**（`iter1_eval_ppo_20260922_220518`）：成功率 80%（BC 95%）、跌倒 0%、速度比中位數 0.93、最大側向偏移中位數 0.097 m（BC 0.134）、方向誤差中位數 6.3°；推擠測試 69%、跌倒 0%。
  - 失敗的 20 個 episode 都沒有跌倒，原因是**著地次數和時鐘不同步**：實際步數為時鐘應有步數的 1.2–1.8 倍（腳多踏幾下），距離門檻（0.8 × 實際步數 × 步長）跟著變高；也有步數只有 0.5 倍（拖腳）的情況。目前的獎勵沒有任何一項要求著地節奏跟上時鐘。
  - 低步頻 0.8–0.93 Hz 成功率 55%、中速 0.10–0.15 m/s 成功率 58%。
- **影片**（`iter1_videos_ppo_20260922_220734/videos/`）：slow、fast、failure（中速組 4 台都沒有通過判定，沒有 medium）。
- **PPO rollout**：4096 episode（執行動作 = 平均 + 0.3 × std × 雜訊），跌倒 0%、成功率 73%。
- **只統計分析**（`iter1_filterstats_20260922_222020/stats_report.json`）：通過全部硬性門檻 61%；各門檻淘汰：步數與距離 27%、側向偏移 < 0.15 m 14%、方向 < 10° 2%、速度 0%、扭矩 0%；各分組通過 1060／398／1043。側向偏移 < 0.115 m 約 62%、< 0.08 m 28%。
- **決定（使用者，2026-09-22）**：加入 `gait_contact` 獎勵（權重 2），用同一個 BC 重跑第 1 輪 PPO（run tag `gait`）；第一版保留作為對照。篩選門檻維持暫定值。

### iteration 1，第二版（加入 gait_contact，2026-09-22）
- **PPO**（`iter1_ppo_gait_20260922_223147`）：1500 iterations、約 46 分鐘。
- **固定集評估**（`iter1_eval_ppo_gait_20260922_231748`）：

| 指標 | BC iter0 | PPO 第一版（原獎勵） | PPO 第二版（gait_contact） |
|---|---|---|---|
| 成功率 | 95% | 80% | **100%** |
| 跌倒率 | 0% | 0% | 0% |
| 速度比（p10／中位數／p90） | — ／1.03／— | 0.90／0.93／0.96 | 0.96／0.985／1.03 |
| 步數／時鐘應有步數（p10–p90） | — | 0.52–1.38 | 0.97–1.01 |
| 最大方向誤差中位數 | 6.7° | 6.3° | 4.3° |
| 最大側向偏移中位數（p90） | 0.134 m | 0.097 m（0.152） | 0.139 m（0.225） |
| 推擠測試成功率／跌倒 | 92%／0% | 69%／0% | **100%／0%** |

  三個速度分組、三個步頻分組的成功率都是 100%；扭矩 p95：髖 roll 約 23 N·m，其他 6–10 N·m。
  節奏跟上時鐘後，搖擺步態的側向擺動也回到和老師相近的水準。
- **影片**（`iter1_videos_ppo_20260922_232018/videos/`）：slow、medium、fast（16 台都成功，沒有失敗案例）。
- **PPO rollout**（`iter1_collect_ppo_gait_*`）：4096 episode，跌倒 0%、成功率 100%。
- **只統計分析**（`iter1_filterstats_20260922_233317`）：通過全部硬性門檻 63.5%；淘汰幾乎只來自側向偏移 < 0.15 m（36.5%），其餘門檻淘汰 ≤ 0.1%；各分組通過 990／852／759。側向偏移 < 0.12 m 30%、< 0.10 m 7%、< 0.08 m 0%。
- **篩選**（`iter1_filter_20260922_233330`）：各分組依 return 取前 50%，保留 1301 episode（495／426／380）、1,299,699 步。
- **蒸餾**（`iter1_distill_20260922_233423`）：篩選資料 1301 episode ＋ 老師資料 326 episode（20%）；依 episode 切分 1464／163。
  驗證 MSE 0.00028（R² 0.999）。固定集評估：成功率 95%、跌倒 0%、速度比中位數 1.05、側向偏移中位數 0.160 m、方向誤差 7.6°、推擠 96%。
  比該輪 PPO（100%、0.139 m）稍差。可能原因：PPO 結束時動作 std 只剩約 0.015，rollout 的探索雜訊（0.3 × std）幾乎為零，資料缺少「偏離後修正」的示範（老師資料則有 σ = 0.05 的雜訊）。
- 修正：`dataset.merge` 只保留各資料都有的 episode 欄位（老師資料在加入 gait_contact 前收集，缺 `rew_gait_contact`）。

### iteration 2，第一版（對照組，lateral_deviation −10，2026-09-23）
- **PPO**（`iter2_ppo_20260922_233809`，由 `iter1_distill` 初始化）：固定集成功率 100%、跌倒 0%、速度比中位數 0.98、方向誤差中位數 4.3°、推擠 99%（1 次跌倒）；
  但最大側向偏移中位數 0.183 m（p90 0.337 m），比第 1 輪差。
- **影片**（`iter2_videos_ppo_20260923_003017/videos/`）：slow、medium、fast。
- **只統計分析**（`iter2_filterstats_20260923_004327`，當時門檻 0.115 m）：通過 26%，慢速組 0 個 → **停止規則觸發**。
  原因是慢速指令的路徑漂移：慢／中／快分組的最大側向偏移中位數 0.298／0.175／0.109 m（第 1 輪 0.12／0.13／0.144 m），結束時的偏移約為最大值的 0.68 倍（漂移而非搖擺）。
  即使門檻 0.15 m，慢速組也只剩 1 個。`lateral_deviation` 權重 −10 太弱：偏移 0.3 m 一整個 episode 只扣約 2.5 分，速度追蹤約 30 分。
- **決定（使用者，2026-09-23）**：`lateral_deviation` −10 → −40，從 `iter1_distill` 重跑第 2 輪 PPO（run tag `lat`）；側向門檻三輪都維持 0.15 m。

### iteration 2，第二版（lateral_deviation −40，2026-09-23）
- **PPO**（`iter2_ppo_lat_20260923_004943`）：固定集成功率 100%、跌倒 0%、速度比中位數 0.997、**最大側向偏移中位數 0.114 m**、方向誤差中位數 3.0°、推擠 100%／0 跌倒。側向偏移與方向都比第 1 輪更好。
- **影片**：`iter2_videos_ppo_20260923_013450/videos/`（slow、medium、fast）。
- **PPO rollout**：4096 episode，跌倒 0%、成功率 99.8%。
- **只統計分析**（`iter2_filterstats_20260923_014401`）：通過全部門檻 84.2%（第一版 26%）；淘汰只來自側向 < 0.15 m（15.8%）；各分組 1135／1180／1134。側向 < 0.12 m 46%、< 0.10 m 31%、< 0.08 m 13%。
- **篩選**（`iter2_filter_20260923_014419`）：保留 1725 episode、1,723,275 步。
- **蒸餾**（`iter2_distill_20260923_014435`）：篩選資料 ＋ 第 1 輪 rollout 432 episode（20%）；驗證 MSE 0.00007（R² 1.000）。
  固定集：成功率 99%、跌倒 1%、速度比 0.995、側向偏移中位數 0.107 m、方向 2.9°、推擠 95%。品質比第 1 輪蒸餾（95%、0.160 m）好很多。

### iteration 3（2026-09-23）
- **PPO**（`iter3_ppo_20260923_014638`，由 `iter2_distill` 初始化）：固定集成功率 100%、跌倒 0%、速度比中位數 0.971、**最大側向偏移中位數 0.067 m**（p90 0.126）、方向誤差中位數 2.6°、推擠 100%／0 跌倒。
- **影片**：`iter3_videos_ppo_20260923_021848/videos/`（slow、medium、fast）。
- **PPO rollout**：4096 episode，跌倒 0%、成功率 100%。
- **只統計分析**（`iter3_filterstats_*`）：通過全部門檻 96.0%；淘汰只來自側向 < 0.15 m（4.0%）；各分組 1248／1392／1292。
  側向 < 0.08 m 已有 66%（第 2 輪只有 2%）→ 原計畫第 3 輪的 0.08 m 門檻此時其實可行，但當初是在第 2 輪資料上判斷的。
- **篩選**：保留 1966 episode、1,964,034 步。
- **蒸餾（最終模型，`iter3_distill_20260923_022831`）**：篩選資料 ＋ 第 2 輪 rollout 492 episode（20%）；驗證 MSE 0.00005（R² 1.000）。
  固定集：成功率 99%、跌倒 0%、速度比 0.973、側向偏移中位數 0.072 m、方向 2.6°、**推擠 100%／0 跌倒**。
- 三輪比較表與結論見 `00_test/outputs/summary.md`。

## 產出清理（2026-10-07）
為釋放空間，實驗一的 `dataset.h5`、中間 checkpoint 與中間版本影片已刪除（清單見 `outputs/cleanup_list_20261007.txt`）。
上方提到的 `iter1_videos_ppo_*`、`iter2_videos_ppo_*`、`iter3_videos_ppo_*` 影片已不存在（`videos.json` 保留）；
保留的影片只有 rule-based（`rule_based/videos_rule_based_20260919_162742`）與最終蒸餾（`training/iter3_videos_distill_20260923_023022`）。
各 run 的最終模型、`config.yaml`、`stats.json` 與 TensorBoard log 都保留。
