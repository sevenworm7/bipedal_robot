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
