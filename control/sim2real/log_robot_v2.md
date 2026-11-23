# sim2real 執行紀錄：robot v2（2026-11-18 ～ 11-19）

robot v2（tag `robot-v2`，大腿 70、小腿 55 mm）接續 v1 的 sim2real 模型訓練，計畫見 [`docs/progress.md`](../../docs/progress.md)，數字見 [`docs/results.md`](../../docs/results.md)「robot v2」。
所有指令都先設 `CUSTOM_BIPED_VERSION=2`，產出在 `outputs/robot_v2/control/sim2real/`。

**結論：student v2ft 通過全部部署門檻，而且在不規則地形與組合場景上比 v1 的 student v5 更好**（組合場景 504 episode：92.7%／跌倒 0.4%，v1 是 85.7%／3.2%）。
訓練流程和 v1 第五輪完全相同，沒有修改任何程式、場景或獎勵。

## 注意：`evaluate.py` 印出的成功率不是 sim-to-real 判定

`evaluate.py` 的 `[eval] success` 和 `stats.json` 的 `nominal.success_rate` 是 02／03 的 rule-based 判定（距離 ≥ 0.8 × 步數 × 步長）。
腿變短之後名目步長也不同，這個數字會嚴重低估（例如 student v2ft 的 `dr_rough_eval_v1` 印出 55%，sim-to-real 判定是 93%）。
**門檻一律用 `rescore.py` 的 sim-to-real 判定**（未跌倒、走完、速度 ±25%、方向 < 10°），和 v1 的表格相同。第 0 步一開始誤用了印出的數字，已更正。

## 第 0 步：零樣本評估（2026-11-18）

```
tools\isaacpy.cmd control\sim2real\evaluate.py --scenario <場景> --agent student:outputs/robot_v1/control/sim2real/student_v5_20261007_003835/student.pt --tag v1s5_zeroshot
tools\isaacpy.cmd control\sim2real\rescore.py <eval 資料夾…>
```

- sim-to-real 判定：`flat_v1` 92%、`dr_v1` 66%、`rough_eval_v1` 75%、`dr_rough_eval_v1` 23%、`gait_eval_v1` 100%；**跌倒全部 0%**。
- 失敗原因是速度：腿變短，同樣的關節擺幅換算成較小的步幅，速度比 p50 只有 0.71–0.85（v1 上 1.09）。
  `dr_rough_eval_v1` 速度在 ±25% 內的只有 30%，方向誤差 < 10° 的只有 56%。
- log 裡的 `Failed to find contact report API / rigid body ... right_foot` 警告在 v1 上用同樣指令也會出現（巢狀的 link 路徑），是既有的無害警告。
- 結論：照計畫做第 1 步，迭代次數沿用 v1 第五輪的 1500。

## 第 1 步：teacher 微調（2026-11-18，49 分鐘）

```
tools\isaacpy.cmd control\sim2real\teacher_ppo.py --tag v2ft --symmetry --reward_preset obs_lateral --scenario dr_rough_gait_train_v2 --init outputs/robot_v1/control/sim2real/teacher_v5_20261006_234626/model_1499.pt --max_iterations 1500
```

- run：`teacher_v2ft_20261118_221706`（`model_1499.pt`）。`--init` 直接載入 teacher v5，所以 `config.py` 的 v1 `init_checkpoint` 沒有被用到。
- 訓練結束時：跌倒率 1.7%、平均距離 1.70 m、扭矩飽和 5.8%（訓練場景含推擠 0.25 m/s 與組合隨機化），action std 0.011–0.055。
- `dr_rough_eval_v1`（特權，參考）：95%／0%。

## 第 2 步：student 蒸餾（2026-11-18，13 分鐘）

```
tools\isaacpy.cmd control\sim2real\student_distill.py --teacher outputs/robot_v2/control/sim2real/teacher_v2ft_20261118_221706/model_1499.pt --tag v2ft --symmetry --reward_preset obs_lateral --scenario dr_rough_gait_train_v2
```

- run：`student_v2ft_20261118_230657`（`student.pt`，210 維輸入，介面同 v1）。DAgger 120 輪，最後 val_mse 0.0028。
- 和計畫不同的地方：`student_distill.py` 沒有載入既有 student 權重的選項，student 是從新的 teacher 重新蒸餾，而不是從 v1 student v5 接續（v1 第五輪也是這樣做）。

## 第 3 步：評估（2026-11-18 ～ 11-19）

| 場景 | sim-to-real 成功／跌倒 | 速度比 p50 | 側向偏移 p50 | 方向誤差 p50 |
|---|---|---|---|---|
| `flat_v1` | 100%／0% | 1.07 | 0.161 m | 1.4° |
| `dr_v1` | 100%／0% | 1.04 | 0.103 m | 2.1° |
| `rough_eval_v1` | 99%／0% | 1.05 | 0.125 m | 3.5° |
| `dr_rough_eval_v1` | 93%／1%（504 ep：92.7%／0.4%） | 1.02 | 0.158 m | 6.0° |
| `gait_eval_v1` | 100%／0% | 1.08 | 0.133 m | 1.7° |

- 組合場景的失敗幾乎都是方向誤差 > 10°（速度合格 99%、未跌倒 99.6%）。
- 步態對稱：左右步長 p50 30.6／30.6 mm，不對稱度 0.00（`gait_symmetry.py`）。
- 自發停頓：`dr_rough_eval_v1` 0%；`gait_eval_v1` 1.0%（0.28–0.32 s，站立後重新起步，和 v1 同級）。
- 影片：`videos_<場景>_student_v2ft_*`（slow／medium／fast，組合場景另有失敗例）；medium 與失敗例 6 段已壓縮放進 `docs/videos/robot_v2/control/`。
- v2 原地抬腳上限較低（24 mm，v1 56 mm），原本擔心不規則地形會變差，結果反而更好（94 → 99%）。

## 慢步態模型 slow（2026-11-23，實驗卡；使用者 2026-11-23 要求訓練，作為和 v2ft 的比較）

**動機**：實機懸空 `WALK 0.05 1.6` 要求膝 p95 270–400°/s，供電壓降時跟不上（`deploy/test_log.md` 2026-11-23）；只降 v2ft 的步頻指令不會讓擺腿變慢（`docs/results.md`「v2 慢步態評估」）。

**設定（使用者 2026-11-23 決定，`docs/decisions.md` #30）**
- 步頻 0.8–1.2 Hz、速度 0.03–0.08 m/s（0.8 Hz 時每步 ≤ 50 mm；v2 單步上限 67 mm）。
- 模擬伺服速度上限 9.6 → **5.2 rad/s**（約 300°/s，DC 馬達扭矩–速度曲線在此歸零；新增 `DomainRandCfg.actuator_velocity_limit`，預設關閉）；擺動騰空時間目標 0.15 → 0.25 s。
- 其餘和 v2ft 相同：中等隨機化＋凹凸地形 curriculum＋步態變化、`obs_lateral`、鏡像對稱、1500 iterations，從 teacher v2ft 接續。
- 場景（新增，不改舊場景）：訓練 `dr_rough_gait_slow_train_v1`；評估 `flat_slow_v1`、`dr_slow_v1`、`rough_eval_slow_v1`、`dr_rough_eval_slow_v1`、`gait_eval_slow_v1`（全部用相同的慢指令與 5.2 rad/s 伺服）。

**指令**（先 `set CUSTOM_BIPED_VERSION=2`；整串用 `bash control/sim2real/run_slow_gait_chain.sh` 依序執行，log 在 `outputs/robot_v2/control/sim2real/slow_chain_logs_<時間>/`）
```
tools\isaacpy.cmd control\sim2real\teacher_ppo.py --tag v2slow --symmetry --reward_preset obs_lateral --scenario dr_rough_gait_slow_train_v1 --init outputs/robot_v2/control/sim2real/teacher_v2ft_20261118_221706/model_1499.pt --max_iterations 1500
tools\isaacpy.cmd control\sim2real\student_distill.py --teacher outputs/robot_v2/control/sim2real/teacher_v2slow_<時間>/model_1499.pt --tag v2slow --symmetry --reward_preset obs_lateral --scenario dr_rough_gait_slow_train_v1
tools\isaacpy.cmd control\sim2real\evaluate.py --scenario <評估場景> --agent student:<student.pt> --tag v2slow   （v2ft 同場景 --tag v2ft_on_slow 作對照）
tools\isaacpy.cmd control\sim2real\rescore.py <eval 資料夾…>
```

**輸出**：`outputs/robot_v2/control/sim2real/{teacher_v2slow_*, student_v2slow_*, eval_<場景>_v2slow_*, eval_<場景>_v2ft_on_slow_*}`；關節速度與電流用負載分析（新增慢步態案例）。

**預估**：teacher 約 50 分鐘、student 約 15 分鐘、評估約 15 分鐘。

**驗證目標**
1. 部署門檻（sim-to-real 判定）：`flat_slow_v1`、`dr_slow_v1` ≥ 95%，凹凸 ≥ 85%，跌倒 ≤ 5%。
2. **擺腿真的變慢**：膝、髖 pitch 速度 p95 明顯低於 v2ft（模擬 v2ft 落地約 135–190°/s），騰空時間 p50 ≥ 0.2 s（v2ft 低步頻時 0.06–0.10 s）。
3. 和 v2ft 在相同慢場景下比較成功率、跌倒率、關節速度、電流。

**跑完後**：rescore → 負載分析 → 寫入 `docs/results.md`、本檔、`docs/progress.md`；通過的話再決定是否匯出 int8 上實機（另外問使用者）。

### 結果（2026-11-23 晚）

- 開跑前：`flat_v1` 回歸檢查 IDENTICAL（`eval_flat_v1_regress_slowopt_20261123_165356` vs `eval_flat_v1_v2s1_20261118_232049`）；試跑 `teacher_v2slow_smoke_20261123_165517`（512 環境 × 10 iterations，只確認能跑、速度上限生效，不使用）。
- teacher `teacher_v2slow_20261123_165730`（79 分鐘）：起點跌倒率 1.5%，結束 8.0%（訓練場景含推擠與組合隨機化；v2ft 結束時 1.7%）；`feet_air_time` 獎勵結束時 −0.006（騰空時間仍短於 0.25 s 目標）。
- student `student_v2slow_20261123_181749`（22 分鐘）：val_mse 0.0023。
- 評估（`eval_<慢場景>_v2slow_*`，對照 `eval_<慢場景>_v2ft_on_slow_*`，已 rescore）與負載分析（`outputs/robot_v2/evaluation/load_analysis/load_v3_{v2slow,v2ft}_20261123`）：數字見 `docs/results.md`「v2 慢步態模型 student v2slow」。
- **結論**：成功率與側向偏移比 v2ft 好，但膝速度不變（p95 約 135–148°/s）、電流相同；騰空時間 0.04 → 0.10–0.12 s。模擬中關節速度原本就只有約 2.4 rad/s，5.2 rad/s 的上限幾乎沒有約束力（v2ft 在 5.2 與 9.6 rad/s 下的負載數字完全相同）。要再慢需要關節速度懲罰或更低的上限（`docs/backlog.md` #25）。
