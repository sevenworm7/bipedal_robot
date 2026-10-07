# 04_sim2real/sim2real：sim-to-real 階段的程式

特權 teacher（PPO）→ 可部署 student（DAgger 蒸餾）→ 各場景評估、影片與步態診斷。網路、BC 與固定評估集的設定沿用 `03_training_pipeline/training_pipeline`（直接 import，不複製）。
環境與場景在 `shared/env/`（`scenarios.py`），觀測介面 v2 在 `shared/interface_v2.py`，左右鏡像在 `shared/symmetry.py`。
執行紀錄、決定與結果見上一層的 [`README.md`](../README.md)。

## 執行方式（專案根目錄，每個指令一個 Isaac Lab 行程）

| 用途 | 指令 |
|---|---|
| teacher PPO（第五輪的做法） | `tools\isaacpy.cmd 04_sim2real\sim2real\teacher_ppo.py --tag <名稱> --symmetry --reward_preset obs_lateral --scenario dr_rough_gait_train_v2 --init <teacher checkpoint> --max_iterations 1500` |
| student 蒸餾 | `tools\isaacpy.cmd 04_sim2real\sim2real\student_distill.py --teacher <teacher model_*.pt> --tag <名稱> --symmetry --reward_preset obs_lateral --scenario dr_rough_gait_train_v2` |
| 固定集評估 | `tools\isaacpy.cmd 04_sim2real\sim2real\evaluate.py --scenario <場景> --agent student:<student.pt>`（`policy:<ckpt>` 也可；`--episodes 504` 加大樣本） |
| 回歸檢查 | `evaluate.py --scenario flat_v1 --agent policy:<03 最終 model.pt> --compare <參考 stats.json>`（修改 `shared/env` 後必跑，必須完全一致） |
| 重新計分 | `tools\isaacpy.cmd 04_sim2real\sim2real\rescore.py <eval 資料夾…>`（sim-to-real 成功判定，不需要模擬） |
| 評估影片 | `tools\isaacpy.cmd 04_sim2real\sim2real\record_videos.py --scenario <場景> --agent student:<student.pt> --tag <名稱> --enable_cameras` |
| 左右對稱 | `tools\isaacpy.cmd 04_sim2real\sim2real\gait_symmetry.py --agent <agent> --tag <名稱>` |
| 自發停頓 | `tools\isaacpy.cmd 04_sim2real\sim2real\stop_analysis.py --scenario <場景> --agent <agent> --tag <名稱>` |
| 鏡像映射檢查 | `tools\isaacpy.cmd 04_sim2real\sim2real\check_symmetry.py --agent policy:<ckpt> [--agent …]` |
| v2 觀測煙霧測試 | `tools\isaacpy.cmd 04_sim2real\sim2real\check_env_v2.py --scenario <場景>` |

產出在 `outputs/robot_v1/sim2real/<名稱>_<時間>/`，資料夾已存在時程式會停下來。

## 檔案

| 檔案 | 內容 |
|---|---|
| `config.py` | `TeacherCfg`（場景、PPO 參數、推擠、獎勵覆寫、`reward_presets`、`env_overrides`、對稱）、`StudentCfg`（DAgger 輪數、緩衝、鏡像樣本） |
| `common.py` | run 資料夾、git commit、套用場景與 03 設定建立環境 |
| `agents.py` | 依 checkpoint 輸入維度選觀測組（44＝03 模型、55＝teacher、210＝student）；`StudentAgent` |
| `teacher_ppo.py` | 特權 teacher：從 03 模型（第一層補 0）或既有 teacher 接續；critic 預熱；可選鏡像對稱 |
| `student_distill.py` | DAgger 蒸餾成只用 IMU 與伺服讀值的 student |
| `symmetry.py` | rsl_rl 對稱資料擴充的掛鉤（`sim2real.symmetry:augment`） |
| `scoring.py`、`rescore.py` | sim-to-real 成功判定（未跌倒、走完、速度 ±25%、方向 < 10°）與事後重新計分 |
| `evaluate.py`、`record_videos.py` | 固定集評估、評估影片（`/eval-video` 格式，指令會變時 HUD 顯示 `cmd now`） |
| `gait_symmetry.py`、`stop_analysis.py`、`check_symmetry.py`、`check_env_v2.py` | 診斷：左右步長、自發停頓、鏡像映射、v2 觀測 |

最終模型：`outputs/robot_v1/sim2real/student_v5_20261007_003835/student.pt`（210 維輸入，介面 `shared/interface_v2.py` 的 `student`）。
