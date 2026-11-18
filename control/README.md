# control：產生行走策略（模擬，實驗二）

從規則式老師到可部署的學生策略。這裡的程式都在模擬中執行（Isaac Lab），**只產生策略並做訓練範圍內的驗收**；訓練範圍以外的測試在 [`evaluation/`](../evaluation/README.md)，實機在 [`deploy/`](../deploy/README.md)。

| 子資料夾 | 內容 | 最終產物 |
|---|---|---|
| [`rule_based/`](rule_based/README.md) | 規則式控制器（老師）：步態時鐘＋IK＋尾巴平衡，CEM 調參 | `rule_based/params/robot_v1_cem1_best.json`（100%／0%） |
| [`flat_pipeline/`](flat_pipeline/README.md) | 平地：老師資料 → BC → PPO → 自我蒸餾 ×3 | `iter3_distill_20261001_132024/model.pt`（需要 base 線速度，不可部署） |
| [`sim2real/`](sim2real/README.md) | 強健性：特權 teacher PPO → 可部署 student（DAgger）；隨機化、凹凸地形、步態多樣性、左右對稱 | **student v5** `student_v5_20261007_003835/student.pt`（只用 IMU 與伺服讀值，部署候選） |

依序相依：`flat_pipeline` 用 `rule_based` 當老師；`sim2real` 以 `flat_pipeline` 的最終模型為起點，並 import 它的網路、BC 與固定評估集設定。
匯入方式：腳本把專案根目錄（`core`、`simulation`）與 `control/`（`rule_based`、`flat_pipeline`、`sim2real`）加入 `sys.path`。
產出：`outputs/robot_v<N>/control/{rule_based, flat_pipeline, sim2real}/`；進度與下一步見 [`docs/progress.md`](../docs/progress.md)。

沿革：原為 `control/rule_based/`、`control/`（2026-10-15 重整為 `control/`）。
