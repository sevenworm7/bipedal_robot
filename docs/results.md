# 成果總表（各機器人版本）

> 各階段的數字摘要，**進 git**（2026-11-18 起）。原本只在不進 git 的 `outputs/robot_v1/summary.md`，該檔仍保留影片連結與更多細節。
> 精選影片（壓縮版，git LFS）在 [`videos/`](videos/README.md)。表中的 run 路徑相對於 `outputs/robot_v<N>/`；每一輪的過程在各資料夾的 README 或 `log_robot_v<N>.md`。
> 新版本的結果加在對應段落，並在「跨版本比較」補一列。

## 跨版本比較（最終可部署模型，sim-to-real 判定，100 episode、seed 42，成功率／跌倒率）

| 機器人 | 模型 | `flat_v1` | `dr_v1` | `rough_eval_v1` | `dr_rough_eval_v1` | `gait_eval_v1` |
|---|---|---|---|---|---|---|
| robot v1（0.696 kg） | student v5 | 100%／0% | 100%／0% | 94%／1% | 83%／2%（504 ep：85.7%／3.2%） | 100%／0% |
| robot v2（0.670 kg） | v1 student v5 零樣本 | 92%／0% | 66%／0% | 75%／0% | 23%／0% | 100%／0% |
| robot v2 | **student v2ft**（微調後） | **100%／0%** | **100%／0%** | **99%／0%** | **93%／1%（504 ep：92.7%／0.4%）** | **100%／0%** |

門檻：`flat_v1` 與 `dr_v1` ≥ 95%、`rough_eval_v1` ≥ 85%，各難度跌倒率 ≤ 5%。

# robot v2

機器人：`custom_biped` v2（tag `robot-v2`，0.670 kg；大腿 70、小腿 55 mm，膝 0–110°；其他同 v1）。評估集、判定與 v1 相同（sim-to-real 判定，`rescore.py`）。過程見 [`control/sim2real/log_robot_v2.md`](../control/sim2real/log_robot_v2.md)。
run 路徑相對於 `outputs/robot_v2/control/sim2real/`。

## v2 訓練（接續 v1 模型，2026-11-18）

| 步驟 | run | 內容 |
|---|---|---|
| 0 零樣本 | `eval_<場景>_v1s5_zeroshot_20261118_*` | v1 student v5 直接放到 v2 |
| 1 teacher 微調 | `teacher_v2ft_20261118_221706`（`model_1499.pt`） | 從 v1 teacher v5 接續 1500 iterations，`dr_rough_gait_train_v2`、`obs_lateral`、鏡像對稱；49 分鐘 |
| 2 student 蒸餾 | `student_v2ft_20261118_230657`（`student.pt`） | DAgger 120 輪、含鏡像樣本；val_mse 0.0028；13 分鐘 |
| 3 評估 | `eval_<場景>_v2s1_20261118_*`、`eval_dr_rough_eval_v1_v2s1_504_*`、`eval_dr_rough_eval_v1_v2t1_*` | 固定評估集 |

每格為成功率／跌倒率（100 episode、seed 42）：

| 模型 | `flat_v1` | `dr_v1` | `rough_eval_v1` | `dr_rough_eval_v1` | `gait_eval_v1` |
|---|---|---|---|---|---|
| v1 student v5 零樣本 | 92%／0% | 66%／0% | 75%／0% | 23%／0% | 100%／0% |
| teacher v2ft（特權，參考） | — | — | — | 95%／0% | — |
| **student v2ft（v2 最終）** | **100%／0%** | **100%／0%** | **99%／0%** | **93%／1%（504 ep：92.7%／0.4%）** | **100%／0%** |

student v2ft 的細項（未跌倒的 episode）：

| 場景 | 平均距離 | 速度比 p50 | 最大側向偏移 p50 | 最大方向誤差 p50 | 推擠 0.15 m/s 跌倒 |
|---|---|---|---|---|---|
| `flat_v1` | 2.05 m | 1.07 | 0.161 m | 1.4° | 0% |
| `dr_v1` | 1.98 m | 1.04 | 0.103 m | 2.1° | 0% |
| `rough_eval_v1` | 2.01 m | 1.05 | 0.125 m | 3.5° | 0% |
| `dr_rough_eval_v1` | 1.96 m | 1.02 | 0.158 m | 6.0° | 1% |
| `gait_eval_v1` | 1.78 m | 1.08 | 0.133 m | 1.7° | 0% |

- **零樣本**：不跌倒，失敗主要來自速度。v2 的腿較短，同樣的關節擺幅步幅較小，所以速度比 p50 只有 0.71–0.85（v1 上是 1.09）；速度在 ±25% 內的比例在 `dr_rough_eval_v1` 只有 30%，方向誤差 < 10° 的比例也只有 56%。
- **微調後**：速度比回到 1.02–1.08，全部通過門檻，而且比 v1 student v5 更好（`rough_eval_v1` 94 → 99%、組合場景 504 episode 85.7 → 92.7%、跌倒 3.2 → 0.4%）。
- 和 v1 相同的弱點：沒有外部定位，側向位置無法修正（側向偏移 p50 0.10–0.16 m，和 v1 同級）。
- **步態**（`gait_symmetry.py`，平地 0.10 m/s、1.6 Hz）：左右步長 p50 30.6／30.6 mm（名目 31.2 mm），不對稱度 0.00（v1 student v5 0.025）；左腳仍有少量修正小碎步（步長 p10 −24 mm）。
- **自發停頓**（`stop_analysis.py`）：`dr_rough_eval_v1` 0%；`gait_eval_v1` 有行走指令時停住 1.0%（0.28–0.32 s，站立後重新起步），和 v1 student v4 的 1.5% 同級。
- 組合場景的失敗幾乎都是方向誤差 > 10°（不跌倒、速度合格的 episode 占 99%）。

影片（student v2ft，medium）：[`flat_v1`](videos/robot_v2/control/exp2_sim2real_student_v2ft_flat_v1_medium_eval_20261118.mp4)、[`dr_v1`](videos/robot_v2/control/exp2_sim2real_student_v2ft_dr_v1_medium_eval_20261118.mp4)、[`rough_eval_v1`](videos/robot_v2/control/exp2_sim2real_student_v2ft_rough_eval_v1_medium_eval_20261118.mp4)、[`dr_rough_eval_v1`](videos/robot_v2/control/exp2_sim2real_student_v2ft_dr_rough_eval_v1_medium_eval_20261118.mp4)（[失敗例](videos/robot_v2/control/exp2_sim2real_student_v2ft_dr_rough_eval_v1_failure_eval_20261118.mp4)）、[`gait_eval_v1`](videos/robot_v2/control/exp2_sim2real_student_v2ft_gait_eval_v1_medium_eval_20261119.mp4)。完整三種速度在 `outputs/robot_v2/control/sim2real/videos_<場景>_student_v2ft_*`。

## v2 泛化、壓力測試與馬達負載（evaluation，2026-11-19，不重新訓練）

對象 student v2ft；條件、seed、episode 數和 v1 完全相同（使用者 2026-11-19 選擇精簡版：只跑和實機相關的條件）。
產出：`outputs/robot_v2/evaluation/generalization/stress_v{1,2,3}_20261119/`、`outputs/robot_v2/evaluation/load_analysis/load_v1_20261119/`。共 33 個壓力條件＋12 個負載情境，統計 27 分鐘。

### 壓力測試（成功率／跌倒率，100 episode）

| 條件 | v1 student v5 | v2 student v2ft | 比較 |
|---|---|---|---|
| 延遲 0–40 ms（訓練範圍） | 100%／0% | 99–100%／0% | 相同 |
| **延遲 60 ms** | 100%／0% | **46%／0%**（速度比 1.26） | **v2 較差**：不跌倒，但走太快超出 ±25% |
| 延遲 80／100 ms | 75%／2%、0%／48% | 27%／0%、27%／0% | v2 不跌倒，但速度失控 |
| 凸起 8／10／12／16 mm | 92／86／58／9%（跌倒 3／4／18／69%） | 98／96／85／23%（跌倒 0／1／3／44%） | v2 較好 |
| 凸起間距 5 cm（8 mm） | 71%／4% | 95%／0% | v2 較好 |
| 上台階 5 mm／下台階 5、10 mm（`stress_v2`） | 99／99／95%（跌倒 1%） | 原始 95%／5%；扣除出生點異常 100%／0% | 相同 |
| 上台階 10 mm（`stress_v2`） | 40%／20% | 原始 70%／10%；扣除出生點異常 74%／5% | v2 較好 |
| 關節零點偏差 1／2／3／5° | 100／100／93／39%（5° 跌倒 27%） | 100／99／98／59%（5° 跌倒 10%） | v2 較好；側向漂移 2° 時 0.22 m |
| IMU 安裝傾斜 2／5／8／12° | 100／100／99／49%（12° 跌倒 13%） | 100／100／99／79%（12° 跌倒 0%） | v2 較好 |
| 組合（零點 2°＋IMU 3°＋遺失 5%） | 99%／1% | 99%／0% | 相同 |

- **出生點異常**（台階地形，和模型無關）：固定 5 台（env 6、13、20、32、40，y = −45.5、−38.5、−31.5、−19.5、−11.5 m）出生時一隻腳高 3–8 mm，0.1–0.2 s 內傾斜超過 40° 判定跌倒，一步都沒走；v1 也有 1 台（env 20）。見 `backlog.md` #16。
- **延遲**：v2 的餘裕比 v1 小，實機「指令到伺服動作」要 **≤ 40 ms**（`decisions.md` #25）；超過時是速度失控，不是跌倒。
- 影片：[延遲 60 ms](videos/robot_v2/generalization/stress_v1_01_delay_060ms.mp4)、[上台階 10 mm](videos/robot_v2/generalization/stress_v2_07_steps_up_10mm.mp4)、[零點偏差 5°](videos/robot_v2/generalization/stress_v3_13_joint_offset_5deg.mp4)；其他條件的影片在各 `outputs` 資料夾。

### 馬達負載（128 台 × 20 s；RMS 扭矩 N·m，額定 0.294）

| 情境 | hip pitch v1 → v2 | knee v1 → v2 | ankle v1 → v2 | hip roll v1 → v2 | 整機電流平均／最大 v1 → v2 |
|---|---|---|---|---|---|
| **靜止站立** | 0.186 → 0.125 | 0.146 → 0.092 | 0.098 → 0.085 | **0.194 → 0.415（141%）** | 2.9／4.6 → 3.2／6.3 A |
| 平地中速 0.10 m/s | 0.175 → 0.191 | 0.145 → 0.141 | 0.103 → 0.081 | 0.146 → 0.152 | 3.1／5.9 → 3.1／5.6 A |
| 最高指令 0.15 m/s、2 Hz | 0.206 → **0.240（82%）** | 0.163 → 0.187 | 0.123 → 0.118 | 0.172 → 0.152 | 3.3／5.6 → 3.3／5.8 A |
| ±8 mm 凹凸 | 0.173 → 0.191 | 0.150 → 0.140 | 0.106 → 0.089 | 0.153 → 0.146 | 3.2／**8.8** → 3.1／6.5 A |
| 整機質量 ×1.5 | 0.233 → 0.234 | 0.214 → 0.186 | 0.127 → 0.103 | 0.204 → 0.201 | 3.7／5.7 → 3.5／6.4 A |
| 中等隨機化 | 0.170 → 0.187 | 0.208 → 0.226 | 0.120 → 0.105 | 0.153 → 0.165 | 3.3／7.8 → 3.3／7.4 A |

- 走路時：knee、ankle 多數情境變輕，hip pitch 約重 10%（最高指令時 RMS 82%、26% 的時間超過額定）；整機電流和 v1 同級，最大值反而較低。電源建議不變（2S、可持續 ≥ 10 A）。
- **新問題：靜止站立時兩腳互頂**（128 台全部）。站立指令下模型把左 hip roll 推到 −14.7°，實際停在 −11.7°；兩腳 foot body 間距固定 55.8 mm（v1 61–67 mm），左 hip roll 持續 0.41 N·m（額定 141%，99% 的時間超過額定）。走路、原地踏步正常。[影片](videos/robot_v2/load_analysis/load_v1_01_stand.mp4)。
- **對策（使用者 2026-11-19 決定，`decisions.md` #24）：實機站立時不跑模型，改用固定站姿（action 0 ＝預設關節角），要走時才交給模型。**
  模擬驗證（`evaluation/load_analysis/stand_handover.py`，128 台，站 3 s → 隨機行走指令 20 s）：

| 場景 | 站立時 hip roll RMS | 站立時腳距 p50 | 交接後成功／跌倒 | 速度比 p50 | 方向誤差 p50 |
|---|---|---|---|---|---|
| `flat_v1` | 0.062／0.069 N·m | 59.7 mm | 100%／0% | 1.06 | 1.4° |
| `dr_v1` | 0.070／0.093 N·m | 59.0 mm | 100%／0% | 1.01 | 2.3° |

## v2 板上部署準備（deploy，2026-11-19，尚未上實機）

student v2ft 匯出給芒果磚（`deploy/export/`），權重改用 int8 放 SRAM（`decisions.md` #28）。

**int8 量化後的模擬閉迴路**（`student_int8.pt`，100 episode，sim-to-real 判定，成功率／跌倒率）：

| 場景 | float32（原模型） | int8 |
|---|---|---|
| `flat_v1` | 100%／0% | **100%／0%** |
| `dr_v1` | 100%／0% | **100%／0%** |
| `rough_eval_v1` | 99%／0% | **100%／0%** |
| 組合（`dr_rough_eval_v1`） | 93%／1% | **94%／0%** |

run：`outputs/robot_v2/control/sim2real/eval_<場景>_v2s1_int8_20261119_*`。int8 和 float32 的動作差最大 0.025（關節目標 0.36°）。

**C 程式和 PyTorch／模擬的一致性**（PC 單元測試，`deploy/firmware/build_host_tests.sh`）：

| 項目 | 結果 |
|---|---|
| float32 推論 vs PyTorch（256 組） | 最大差 1.6e-6 |
| 觀測重建 vs 模擬（4 台 × 252 步，站 1 s → 走 4 s） | 最大差 6e-8 |
| int8 推論 vs Python 參考 | 最大差 1.2e-6 |
| 韌體 | 編譯 0 警告；SRAM 使用約 300 KB／512 KB（權重 281 KB） |

**芒果磚實測（2026-11-23，robot v2，扭矩關閉）**：

| 項目 | 結果 |
|---|---|
| 板上推論（int8 權重 × float activation） | 15.5 ms（太慢） |
| 板上推論（int8 × int16 activation，DSP `SMLAD`） | **6.8 ms**；和 Python int8 參考差 2.3e-4，和 fp32 動作差 0.018 |
| 讀 10 顆馬達 | sync read **6.35 ms**（100 次 0 失敗）；逐顆 30.8 ms |
| 一個控制週期（預估） | 約 13.7 ms（上限 20 ms） |
| 匯流排 | 10 顆（ID 1–10），7.8–8.0 V，26–31 °C |

**實機開扭矩（2026-11-23，robot v2，懸空與桌上）**：

| 項目 | 結果 |
|---|---|
| 單顆 JOG（6°，10°/s） | 延遲 25–35 ms、靜止誤差 ≤ 0.26° |
| 固定站姿（懸空／桌上） | 誤差 ≤ 0.5°；電壓 7.7／7.6 V；溫度 ≤ 35 °C |
| 步態慢速回放 0.25／0.5 倍（開迴路） | 最大誤差 3.3／3.7°；電壓 ≥ 7.4 V |
| 閉迴路 WALK 0.05 m/s、1.6 Hz（懸空，門檻 6.5 V） | 0.34 秒電壓停機（7.7 → 6.4 V） |
| 同上，門檻 5.5 V | 走完 6 秒；週期 14 ms；電壓 6.1–7.7 V；誤差最大 20.8°（每 20 ms 的最大誤差依電壓分組平均：≥ 7.2 V 時 7.0°，< 6.6 V 時 16.0°） |

模擬閉迴路的 int8 結果（上表）用的是 float activation；改成 int16 後的差距（2.3e-4）比 int8 權重本身的量化誤差小約 80 倍。

# robot v1

機器人：`custom_biped` v1（tag `robot-v1`，0.696 kg，每腿 4 DOF ＋ 2 DOF 尾巴）。平地、直線、0.05–0.15 m/s、1.2–2.0 Hz、20 秒 episode。
所有數字來自同一組固定評估集（100 episode、seed 42、無推擠）；推擠測試為 0.15 m/s × 3 次（第 5、10、15 秒），僅供參考。

## 平地訓練（control/flat_pipeline，2026-10-01）

| 階段 | run | 成功率 | 跌倒率 | 平均距離 | 速度比 p50 | 最大側向偏移 p50／p90 | 方向誤差 p50／p90 | 推擠 成功／跌倒 |
|---|---|---|---|---|---|---|---|---|
| rule-based（老師） | `control/rule_based/eval_cem1_comfix_20261001_093234` | 100% | 0% | 1.95 m | — | 平均 0.018 m | 平均 0.9° | 99%／0% |
| BC | `control/flat_pipeline/iter0_bc_20261001_093456` | 100% | 0% | 1.95 m | 1.018 | 0.014／0.037 m | 0.8／1.5° | 99%／0% |
| 第 1 輪 PPO | `control/flat_pipeline/iter1_eval_ppo_20261001_100902` | 100% | 0% | 1.95 m | 1.013 | 0.046／0.087 m | 1.7／2.1° | 99%／1% |
| （對照）第 1 輪 PPO 側向 −400 | `control/flat_pipeline/iter1_eval_ppo_lat_20261001_104915` | 94% | 0% | 1.86 m | 0.998 | 0.052／0.098 m | 1.5／1.9° | 93%／0% |
| 第 1 輪蒸餾 | `control/flat_pipeline/iter1_distill_20261001_105546` | 100% | 0% | 1.96 m | 1.021 | 0.038／0.060 m | 1.6／2.1° | 100%／0% |
| 第 2 輪 PPO | `control/flat_pipeline/iter2_eval_ppo_20261001_113712` | 99% | 1% | 1.91 m | 1.009 | 0.039／0.070 m | 2.0／3.4° | 100%／0% |
| 第 2 輪蒸餾 | `control/flat_pipeline/iter2_distill_20261001_122314` | 100% | 0% | 1.92 m | 1.003 | 0.034／0.056 m | 2.1／3.5° | 100%／0% |
| 第 3 輪 PPO | `control/flat_pipeline/iter3_eval_ppo_20261001_130713` | 100% | 0% | 2.01 m | 1.054 | 0.036／0.055 m | 1.8／2.8° | 100%／0% |
| 第 3 輪蒸餾（最終） | `control/flat_pipeline/iter3_distill_20261001_132024` | 99% | 1% | 1.98 m | 1.053 | 0.034／0.055 m | 1.8／3.1° | 99%／1% |

影片（medium）：[rule-based 老師](videos/robot_v1/control/exp2_rule_based_medium_eval_20261001.mp4)、[平地最終蒸餾](videos/robot_v1/control/exp2_iter3_distill_medium_eval_20261001.mp4)。

篩選（側向門檻／通過率／保留 episode）：第 1 輪 0.05 m／45%／925；第 2 輪 0.05 m／44%／895；第 3 輪 0.04 m／36%／747。

### 觀察
- 成功率從老師開始就是 100%，訓練的變化集中在直線度：PPO 讓側向偏移變大，篩選＋蒸餾逐輪拉回（p90 0.087 → 0.055 m），仍不如老師（0.037 m）。
- 策略觀測不到側向位置，加重側向懲罰無效（對照組）；方向誤差有約 1–2° 的固定偏航。
- 第 3 輪起速度比指令快約 5%（p90 1.13），仍在 25% 門檻內。

## 和實驗一（參考模型）的最終比較

| | 實驗一 老師 | 實驗一 最終蒸餾 | 實驗二 v1 老師 | 實驗二 v1 最終蒸餾 |
|---|---|---|---|---|
| 成功率／跌倒 | 95%／0% | 99%／0% | 100%／0% | 99%／1% |
| 最大側向偏移 p50／p90 | 0.136／0.228 m | 0.072／0.131 m | 0.014／0.037 m（BC） | 0.034／0.055 m |
| 方向誤差 p50 | 7.2° | 2.6° | 0.8°（BC） | 1.8° |
| 推擠成功率 | 88%（0.3 m/s） | 100% | 99%（0.15 m/s） | 99% |

- 實驗一：訓練讓每一項都比老師進步。實驗二：v1 的老師（尾巴做側向平衡）本身就很好，訓練後成功率、推擠持平，直線度與方向**比老師差**，
  但仍比實驗一的最終模型直約 2 倍（以腿長相近的兩台機器人比較）。
- 推擠測試的強度不同（實驗一 0.3 m/s、28 kg；實驗二 0.15 m/s、0.7 kg），只能看趨勢。

## Sim-to-real（control/sim2real，2026-10-06）

判定改用 sim-to-real 準則（未跌倒、走完、速度在指令 ±25%、方向誤差 < 10°；允許修正步）。每格為成功率／跌倒率，100 episode、seed 42。

| 模型 | `flat_v1` | `dr_v1` | `rough_eval_v1` | `dr_rough_eval_v1` | `gait_eval_v1` |
|---|---|---|---|---|---|
| 平地最終蒸餾（`flat_pipeline` iter3，44 維，需線速度） | 99%／1% | 1%／41% | 0%／96% | 0%／99% | — |
| student v1（可部署） | 91%／0% | 98%／0% | 64%／0% | 39%／14% | 100%／0% |
| student v2final | 100%／0% | 97%／0% | 97%／2% | 56%／20% | 100%／0% |
| **student v3（可部署，達標）** | **100%／0%** | **100%／0%** | **99%／0%** | **87%／5%** | **100%／0%** |
| teacher v3（特權，參考） | 100%／0% | 100%／0% | 99%／0% | 90%／2% | 100%／0% |
| student v4（鏡像對稱） | 100%／0% | 100%／0% | 95%／2% | 85%／4%（504 ep：83.9%／7.7%） | 98%／2% |
| **student v5（最終，鏡像對稱＋只罰看得到的側向速度）** | **100%／0%** | **100%／0%** | **94%／1%** | **83%／2%（504 ep：85.7%／3.2%）** | **100%／0%** |

部署門檻（使用者 2026-10-06 確認）：平地與中等隨機化 ≥ 95%、不規則地形 ≥ 85%、各難度跌倒 ≤ 5%。
student v3 通過但步態跛行（左右步長 49／14 mm）；v4 加鏡像對稱後對稱（0.003）；**最終採用 student v5**：對稱（0.025）、無自發停頓、
側向漂移 p50 平地 0.10 m／中等隨機化 0.14 m（v3 為 0.57／0.31 m），組合場景 504 episode 85.7%／跌倒 3.2%。
影片（student v5，medium）：[`flat_v1`](videos/robot_v1/control/exp2_sim2real_student_v5_flat_v1_medium_eval_20261007.mp4)、[`dr_v1`](videos/robot_v1/control/exp2_sim2real_student_v5_dr_v1_medium_eval_20261007.mp4)、[`rough_eval_v1`](videos/robot_v1/control/exp2_sim2real_student_v5_rough_eval_v1_medium_eval_20261007.mp4)、[`dr_rough_eval_v1`](videos/robot_v1/control/exp2_sim2real_student_v5_dr_rough_eval_v1_medium_eval_20261007.mp4)（[失敗例](videos/robot_v1/control/exp2_sim2real_student_v5_dr_rough_eval_v1_failure_eval_20261007.mp4)）、[`gait_eval_v1`](videos/robot_v1/control/exp2_sim2real_student_v5_gait_eval_v1_medium_eval_20261007.mp4)；對照：[student v3 跛行](videos/robot_v1/control/exp2_sim2real_student_v3_dr_rough_eval_v1_medium_eval_20261006.mp4)。
已知限制：沒有外部定位，側向位置無法修正（由操作者或上層控制給方向）；馬達參數範圍是未實測的估計值。

## 泛化與壓力測試（evaluation/generalization，2026-10-09，不重新訓練）

對象 student v5；一次只改一個條件，其餘同 `flat_v1`；每個條件 100 episode × 20 s、seed 42。斜坡與台階以修正跌倒判定後的 `stress_v2_20261009` 為準。完整報告與影片：`outputs/robot_v1/evaluation/generalization/stress_v1_20261009/README.md`、`stress_v2_20261009/README.md`。

| 項目 | 訓練範圍 | 維持 ≥ 95% 成功的範圍 | 開始失效 |
|---|---|---|---|
| 伺服延遲 | 0–40 ms | **到 60 ms** | 80 ms 成功 75%；100 ms 跌倒 48% |
| 地面摩擦 | 0.4–1.2 | **0.2–1.5** 全部 100% | — |
| 馬達力量 | 80–120% | **到 50%** | — |
| 整機質量 | ×0.85–1.15 | **到 ×1.5** | — |
| IMU 雜訊／方向偏差 | ×1／±2° | **×3／±10°** | — |
| 速度指令／步頻 | 0.05–0.15 m/s／1.2–2.0 Hz | **0.22 m/s／1.0–2.3 Hz** | — |
| 推擠 | 0.25 m/s | 0.25 m/s | 0.35 m/s 跌倒 12%；0.45 m/s 跌倒 36% |
| 凸起高度 | ±8 mm | 4 mm（8 mm 92%／3%） | 10 mm 86%／4%；12 mm 跌倒 18%；16 mm 跌倒 69% |
| 上坡（未訓練，stress_v2） | — | **到 5°** | 8° 成功 80%（變慢 0.79 倍，不跌倒） |
| 下坡（未訓練，stress_v2） | — | **−3°**（97%） | −5° 79%、−8° 9%：速度過快（1.21／1.36 倍），**跌倒僅 1%** |
| 台階（未訓練，stress_v2） | — | 上下 5 mm、下 10 mm（95%） | **上 10 mm：成功 40%、跌倒 20%** |

代表性影片（通過與失效例子）見 [`videos/README.md`](videos/README.md)。

最明顯的弱點：上 10 mm 台階、超過 10 mm 的凸起、延遲 ≥ 80 ms、推擠 ≥ 0.35 m/s；下坡不會跌倒，但速度控制不住。

### 實機誤差測試 stress_v3（2026-10-15）

| 項目 | 結果 | 實機要求 |
|---|---|---|
| 關節零點偏差（每顆 ±） | 2°：100%；3°：93%；5°：39%／跌倒 27% | **組裝後做軟體零點校正，每顆 ±2° 以內** |
| IMU 安裝傾斜 | 8°：99%；12°：49%／跌倒 13% | 一般固定即可（±2–3°） |
| 伺服指令遺失率 | 40% 仍 100% | 不是風險（延遲才是） |
| 組合（零點 2°＋IMU 3°＋遺失 5%） | 99%／跌倒 1% | — |

## 馬達負載分析（evaluation/load_analysis，2026-10-15）

12 個情境 × 128 台 × 20 s（額定 0.294 N·m、上限 0.8 N·m）。完整報告：`outputs/robot_v1/evaluation/load_analysis/load_v1_20261015/README.md`。

| 情境 | 最吃力關節 RMS（佔額定） | 整機電流 平均／p99／最大（估計） |
|---|---|---|
| 靜止站立 | hip pitch 0.19（65%）；5% 的機器人雙腳卡住，右 hip roll 頂在上限 | 2.9／4.3／4.6 A |
| 平地中速 0.10 m/s | hip pitch 0.18（60%） | 3.1／4.5／5.9 A |
| ±8 mm 凹凸 | hip pitch 0.17（59%） | 3.2／4.3／8.8 A |
| 整機質量 ×1.5 | hip pitch 0.23（**79%**，33% 時間超過額定） | 3.7／4.9／5.8 A |

結論：走路時各關節發熱有餘裕（RMS ≤ 額定 70%），最吃力的是 hip pitch；實機總重要控制在設計值附近。電源建議 2S、可持續 ≥ 10 A。站立時雙腳可能卡住（`backlog.md` #1）。
