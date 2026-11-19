# control/sim2real：sim-to-real 強健性訓練（實驗二）

**狀態：robot v1 完成（2026-10-07，最終模型 student v5）；robot v2 接續訓練完成（2026-11-19，最終模型 student v2ft），兩者都通過全部部署門檻（進度只寫在 [`docs/progress.md`](../../docs/progress.md)）。** 起點：`control/flat_pipeline` 的最終蒸餾模型。
產出：`outputs/robot_v<N>/control/sim2real/`（`CUSTOM_BIPED_VERSION` 環境變數選機器人版本）；各模型的數字總表在 [`docs/results.md`](../../docs/results.md)。

**2026-10-09 起**：原 `04_sim2real/` 併入這裡（訓練屬於 03「產生學生」），訓練範圍以外的壓力測試移到 [`evaluation/generalization/`](../../evaluation/generalization/README.md)。

- `PROMPT.md`：這部分的需求規格（使用者 2026-10-06 同意新增）。
- `CLAUDE.md`：規則與從 `flat_pipeline` 沿用的經驗。
- 程式：本資料夾的 `*.py`，見下方「程式與執行方式」。

**用語**：本文件與 `PROMPT.md` 寫於資料夾重整前（原 `04_sim2real/` → `03_training/sim2real/` → 2026-10-15 `control/sim2real/`），其中的「03」「03 最終模型」「03 模型」指平地訓練 `control/flat_pipeline`（最終模型 `outputs/robot_v1/control/flat_pipeline/iter3_distill_20261001_132024/model.pt`），「02」指 `control/rule_based`，「04」指本資料夾。run 內 `config.yaml`／`stats.json` 的舊路徑是歷史紀錄。

## 開始前確認的事項（PROMPT.md「若有不確定之處」，使用者 2026-10-06 決定）

| 項目 | 決定 |
|---|---|
| actor 觀測 | 拿掉 base 線速度（只給 critic）；加入最近 5 個控制步（0.1 s）的歷史；保留步態時鐘（相位 sin/cos 與步頻）作為輸入，但放寬時鐘獎勵 |
| 致動器隨機化 | 先用寬範圍（延遲 0–40 ms、kp／kd ±30%、扭矩上限 ±20%）；有實體馬達後實測一顆 HD-1910，再以新場景版本收窄 |
| 步態多樣性 | 包含站立、原地踏步、啟動／停止（速度指令含 0、episode 中途改變速度）；步態參數變化（步頻、抬腳高度等），並放寬步態時鐘獎勵。不做轉向與倒退 |
| 擾動上限（中等） | 地形起伏 ±8 mm、推擠 0.25 m/s、摩擦 0.4–1.2、質量 ±15%、延遲 0–40 ms；全部用 curriculum 從簡單開始 |
| 可以部署的門檻 | （使用者 2026-10-06 確認）`flat_v1` 與中等隨機化成功率 ≥ 95%、不規則地形 ≥ 85%，各難度跌倒率 ≤ 5% |

## 為什麼新增這個階段

03 在理想條件下（平地、理想馬達、輕推擠）已經達到成功率 99%，但離實機部署還有四個落差：
觀測含實機量不到的 base 線速度、致動器沒有延遲與參數誤差、只走過平地與固定摩擦、步態過於單一。
這些改變觀測介面與環境，目標也不同，所以當初獨立成一個階段（`04_sim2real`），2026-10-09 併入 `control/sim2real`；平地訓練的結果保持凍結、可重現。

## 程式與執行方式

### 執行方式（專案根目錄，每個指令一個 Isaac Lab 行程）

| 用途 | 指令 |
|---|---|
| teacher PPO（第五輪的做法） | `tools\isaacpy.cmd control\sim2real\teacher_ppo.py --tag <名稱> --symmetry --reward_preset obs_lateral --scenario dr_rough_gait_train_v2 --init <teacher checkpoint> --max_iterations 1500` |
| student 蒸餾 | `tools\isaacpy.cmd control\sim2real\student_distill.py --teacher <teacher model_*.pt> --tag <名稱> --symmetry --reward_preset obs_lateral --scenario dr_rough_gait_train_v2` |
| 固定集評估 | `tools\isaacpy.cmd control\sim2real\evaluate.py --scenario <場景> --agent student:<student.pt>`（`policy:<ckpt>` 也可；`--episodes 504` 加大樣本） |
| 回歸檢查 | `evaluate.py --scenario flat_v1 --agent policy:<flat_pipeline 最終 model.pt> --compare <參考 stats.json>`（修改 `simulation/env` 後必跑，必須完全一致） |
| 重新計分 | `tools\isaacpy.cmd control\sim2real\rescore.py <eval 資料夾…>`（sim-to-real 成功判定，不需要模擬；**門檻一律看這個**，`evaluate.py` 印出的 success 是 02／03 的舊判定） |
| 評估影片 | `tools\isaacpy.cmd control\sim2real\record_videos.py --scenario <場景> --agent student:<student.pt> --tag <名稱> --enable_cameras` |
| 左右對稱 | `tools\isaacpy.cmd control\sim2real\gait_symmetry.py --agent <agent> --tag <名稱>` |
| 自發停頓 | `tools\isaacpy.cmd control\sim2real\stop_analysis.py --scenario <場景> --agent <agent> --tag <名稱>` |
| 鏡像映射檢查 | `tools\isaacpy.cmd control\sim2real\check_symmetry.py --agent policy:<ckpt> [--agent …]` |
| v2 觀測煙霧測試 | `tools\isaacpy.cmd control\sim2real\check_env_v2.py --scenario <場景>` |

產出在 `outputs/robot_v1/control/sim2real/<名稱>_<時間>/`，資料夾已存在時程式會停下來。

### 檔案

| 檔案 | 內容 |
|---|---|
| `config.py` | `TeacherCfg`（場景、PPO 參數、推擠、獎勵覆寫、`reward_presets`、`env_overrides`、對稱）、`StudentCfg`（DAgger 輪數、緩衝、鏡像樣本） |
| `common.py` | run 資料夾、git commit、套用場景與 `flat_pipeline` 設定建立環境 |
| `agents.py` | 依 checkpoint 輸入維度選觀測組（44＝03 模型、55＝teacher、210＝student）；`StudentAgent` |
| `teacher_ppo.py` | 特權 teacher：從 03 模型（第一層補 0）或既有 teacher 接續；critic 預熱；可選鏡像對稱 |
| `student_distill.py` | DAgger 蒸餾成只用 IMU 與伺服讀值的 student |
| `symmetry.py` | rsl_rl 對稱資料擴充的掛鉤（`sim2real.symmetry:augment`） |
| `scoring.py`、`rescore.py` | sim-to-real 成功判定（未跌倒、走完、速度 ±25%、方向 < 10°）與事後重新計分 |
| `evaluate.py`、`record_videos.py` | 固定集評估、評估影片（`/eval-video` 格式，指令會變時 HUD 顯示 `cmd now`） |
| `gait_symmetry.py`、`stop_analysis.py`、`check_symmetry.py`、`check_env_v2.py` | 診斷：左右步長、自發停頓、鏡像映射、v2 觀測 |

最終模型：v1 `outputs/robot_v1/control/sim2real/student_v5_20261007_003835/student.pt`；v2 `outputs/robot_v2/control/sim2real/student_v2ft_20261118_230657/student.pt`（都是 210 維輸入，介面 `core/interface_v2.py` 的 `student`）。

## 執行紀錄

| 機器人 | 紀錄 | 內容 |
|---|---|---|
| robot v1 | [`log_robot_v1.md`](log_robot_v1.md) | 階段一～四、第三～五輪訓練（2026-10-06 ～ 10-07）、產出清理 |
| robot v2 | [`log_robot_v2.md`](log_robot_v2.md) | 接續 v1 模型：零樣本 → teacher 微調 → student 蒸餾 → 評估（2026-11-18 ～ 11-19），最終 student v2ft |

## 結論（2026-10-07，使用者決定模擬測試告一段落）

- **最終可部署策略**：student v5。只用 IMU 與伺服讀值（5 步歷史、210 維，介面 `core/interface_v2.py`），通過全部部署門檻：
  平地與中等隨機化 100%、不規則地形 94%、隨機化＋地形 85.7%（504 episode），各難度跌倒 ≤ 3.2%，推擠 0.15 m/s 跌倒 ≤ 4%，步態左右對稱、沒有自發停頓。
- **相對 03**：03 模型在中等隨機化下跌倒 41%、在 ±8 mm 地形上 96%，且需要實機量不到的 base 線速度。
- **過程中學到的事**：
  1. 單項隨機化影響都不大，組合起來會崩潰；最主要的是伺服延遲與傾角雜訊。
  2. 放寬步態時鐘讓策略學會修正步，但沒有對稱約束時會長出跛行式步態；鏡像對稱（資料擴充＋鏡像損失）直接解決，還順帶降低扭矩飽和與側向漂移。
  3. 策略看不到的量（側向位置）不要放進獎勵：它是 03 起最大的單項懲罰，換成看得到的「平均側向速度」後側向漂移大幅下降。
  4. 指令會變化的影片，HUD 必須顯示目前指令，否則容易把指令要求的停止誤判為策略問題。
  5. 100 episode 的固定集約有 ±4% 的誤差，接近門檻時要加大樣本再判斷。
- **交付項目 5（部署用匯出）**：2026-11-19 改為匯出成 C 標頭檔給芒果磚，見 `deploy/export/`（輸入正規化、觀測歷史順序在 `deploy/firmware/src/obs.c`，動作換算成伺服位置值在 `joints.c`）；
  馬達參數仍是未實測的寬範圍，取得實體 HD-1910 後應實測並以新場景版本收窄。


