# sim2real 執行紀錄：robot v2（2026-11-18 起）

robot v2（tag `robot-v2`，大腿 70、小腿 55 mm）接續 v1 的 sim2real 模型訓練，計畫見 [`docs/progress.md`](../../docs/progress.md)，數字見 [`docs/results.md`](../../docs/results.md)「robot v2」。
所有指令都先設 `CUSTOM_BIPED_VERSION=2`，產出在 `outputs/robot_v2/control/sim2real/`。

## 第 0 步：零樣本評估（2026-11-18）

```
tools\isaacpy.cmd control\sim2real\evaluate.py --scenario <場景> --agent student:outputs/robot_v1/control/sim2real/student_v5_20261007_003835/student.pt --tag v1s5_zeroshot
```

- 結果：5 個場景都**不跌倒**（0–4%，含推擠），但成功率只有 0–36%。原因是速度不足：速度比 p50 只有 0.71–0.85，v1 是 1.09。
  腿變短，同樣的關節擺幅換算成較小的步幅，所以超出成功判定的 ±25%。
- log 裡的 `Failed to find contact report API ... right_foot` 警告，在 v1 上用同樣指令也會出現，是既有的無害警告，不是 v2 的問題。
- 結論：照計畫做第 1 步 teacher 微調，迭代次數沿用 v1 第五輪的 1500。

## 第 1 步：teacher 微調（2026-11-18，進行中）

```
tools\isaacpy.cmd control\sim2real\teacher_ppo.py --tag v2ft --symmetry --reward_preset obs_lateral --scenario dr_rough_gait_train_v2 --init outputs/robot_v1/control/sim2real/teacher_v5_20261006_234626/model_1499.pt --max_iterations 1500
```
