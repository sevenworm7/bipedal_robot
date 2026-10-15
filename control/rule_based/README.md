# control/rule_based：規則式行走控制器（老師，實驗二）

`RuleBasedBipedController`（`reset(env_ids)`、`compute_action(obs) -> action`），作為訓練的教師策略。控制方法從實驗一移植（2026-10-01），不變。2026-10-15 起，原 `control/rule_based/`（規格與紀錄）與 `control/rule_based/`（程式）合併在這一層。

**狀態：完成，使用者已檢視影片並確認（2026-10-01）。100 episode 成功率 100%、跌倒 0%，採用 `params/robot_v1_cem1_best.json`。** 機器人：`custom_biped` v1（tag `robot-v1`）。產出：`outputs/robot_v1/control/rule_based/`。

- `PROMPT.md`、`CLAUDE.md`：原始規格（不修改）。
- 程式：本資料夾（從 `archive/exp1_reference/rule_based/` 移植），以套件 `rule_based` 匯入（`control/` 在 `sys.path` 上）。
- 介面：`core/interface.py`（觀測 44 維、動作 10 維、50 Hz）；環境：`simulation/env/`。

## 開始前確認的事項（PROMPT.md「若有不確定之處」，使用者 2026-09-30 決定）

| 項目 | 決定 |
|---|---|
| 機器人 | `custom_biped` v1：每腿 4 DOF ＋ 2 DOF 尾巴，URDF 在 `hardware/robot_v1/urdf/`，USD 在 `simulation/robot_assets/usd/custom_biped_v1/` |
| 地形 | 只用平地 |
| 觀測／動作維度 | 尾巴放進介面：J = 10，觀測 14 + 3 × 10 = 44 維、動作 10 維；結構與順序規則和實驗一相同 |
| 指令範圍 | 速度 0.05–0.15 m/s、步頻 1.2–2.0 Hz（依 v1 靜態檢查：每步最多 ±78 mm） |

## 驗收標準

與實驗一相同：100 個 episode（指令、步頻、初始關節角隨機），成功率 ≥ 70%（未跌倒、至少 10 步、距離 ≥ 0.8 × 步數 × 步長；也就是跌倒率 < 30%），
並統計平均行走距離、速度與方向誤差；3–5 段評估影片（`/eval-video` 格式）。

## 發現的模型問題

rule-based 過程中發現的機器人問題記到 `hardware/v2_backlog.md`（v1 已凍結）。

## 執行方式（專案根目錄）

| 步驟 | 指令 | 產出（`outputs/robot_v<N>/control/rule_based/<名稱>_<時間>/`） |
|---|---|---|
| 1. 離線 IK 檢查 | `tools\isaacpy.cmd control\rule_based\check_ik.py` | `check_ik_*`：report.json、trajectories.png |
| 2. 時序診斷 | `tools\isaacpy.cmd control\rule_based\diagnose.py --cmd 0.1 --freq 1.6 --params <json>` | `diag_*`：timeseries.png/.npz（加 `--video --enable_cameras` 另存影片） |
| 3. 參數搜尋（CEM） | `tools\isaacpy.cmd control\rule_based\optimize.py --tag cem1 --gens 30 --w_speed 1 --w_lat 0.3 --w_heading 0.3` | `cem*`：history.json、final.json、best_params.json、objective.json |
| 4. 正式評估 | `tools\isaacpy.cmd control\rule_based\evaluate.py --params <json> --episodes 100 --push` | `eval*`：stats.json、episodes_*.pt |
| 5. 評估影片 | `tools\isaacpy.cmd control\rule_based\record_videos.py --params <json> --enable_cameras` | `videos_*`：慢／中／快＋失敗案例，HUD 格式依 `/eval-video` |

run 資料夾名稱含日期與時間，已存在時程式會停下來（`runs.py`），不會覆寫。實驗一的 `sweep.py`（格點掃描）沒有移植：參數超過 10 個時格點掃描無效，直接用 CEM。

## 檔案

| 檔案 | 內容 |
|---|---|
| `config.py` | `GaitParams`（所有可調參數與預設值）、`RuleBasedCfg`（隨機取值範圍、IK 邊界）、`load_cfg()` |
| `leg_kinematics.py` | 平面兩連桿 IK/FK（與實驗一相同） |
| `controller.py` | `RuleBasedBipedController` |
| `runs.py` | 建立不會覆寫的 run 資料夾 |
| `check_ik.py`、`diagnose.py`、`optimize.py`、`evaluate.py`、`record_videos.py` | 見上表 |
| `params/` | CEM 找到的參數檔 |

## 和實驗一的差異

1. **關節向量**：使用 robot 模組的 `ACTION_JOINT_NAMES`（腿 8 ＋ 尾巴 2），腿的索引用關節名稱查，不假設順序。
2. **和機器人大小有關的預設值不寫死**：`stance_leg_length`、`foot_x_offset`、`lift_height`、`max_step_x`、`max_place_y` 在 config 中是 `None`，
   由 `RuleBasedCfg.resolve(rb)` 從 robot 模組填入（站姿的髖–踝距離、腳在髖後方 21 mm、抬腳上限的 35%…）。CEM 的搜尋範圍也按這些值縮放。
3. **尾巴（新增 5 個參數）**：
   - `tail_yaw_amp`、`tail_yaw_lead`：tail_yaw 跟著側向搖擺的波形擺向支撐腳那一側，讓全身重心移到支撐腳上方（v1 擺 30° 重心側移 10.9 mm）。
   - `k_tail_roll`：身體往一側傾時，尾巴往另一側擺（姿態回授）。
   - `tail_pitch_offset`、`k_tail_pitch`：尾巴 pitch 的固定偏移與依身體前後傾的回授。
4. **指令範圍**：0.05–0.15 m/s、1.2–2.0 Hz（使用者 2026-09-30 決定），平地。

控制方法的其餘部分（步態時序、支撐腳行程 `cmd/f × (1 − swing_frac)`、倒單擺髖高、IK、平行四邊形側向搖擺、Raibert 落腳點、姿態回授）與實驗一相同，說明見 `archive/exp1_reference/rule_based/README.md`。

## 開發紀錄

### 步驟 1：離線 IK 檢查（2026-10-01）
- FK(IK) 來回誤差 3e-8 m；預設站姿（hip −9.9°、knee 34°）可以還原，踝在髖後方 21.3 mm、下方 170.8 mm。
- 髖到踝的可用距離 98–180 mm（實驗一只有 177–190 mm）。
- 指令範圍內 12 個組合（0–0.15 m/s × 1.2／1.6／2.0 Hz）：步長 13–62 mm，**全部搆得到、腳掌保持水平、沒有關節碰到極限**。
  實驗一在這一步發現的抬腳高度與踝關節限制，在 v1 上都不存在。

### 步驟 2：原地踏步（預設參數，未調整）
- `diagnose.py --cmd 0 --freq 1.6`：左右腳乾淨交替離地，抬腳 10–18 mm，4.2 秒內 17 步；關節追蹤延遲約 0.05 s（kp 8）。
- 4.2 秒時往右側倒下（側傾逐漸放大），和實驗一一樣交給 CEM 調整。

### 步驟 3：CEM 參數搜尋（`cem1`，2026-10-01）
- 24 個參數（實驗一的 19 個＋尾巴 5 個），每代 256 組 × 12 環境，30 代，episode 20 秒，指令與步頻全範圍隨機；
  目標函數一開始就包含速度誤差（w 1.0）、最大側向偏移（w 0.3，以 0.10 m 正規化）、最大方向誤差（w 0.3）。共約 30 分鐘。
- 族群成功率：第 0 代 7% → 第 9 代 67% → 第 29 代 97.6%（跌倒 0.3%）。
- 最後前 8 名各用 256 個環境重新評估：最佳一組成功率 100%、跌倒 0%、平均 2.02 m、側向偏移 0.022 m → `params/robot_v1_cem1_best.json`。
- 找到的步態特徵：
  - **側向重心移動主要靠尾巴**：`tail_yaw_amp` 0.31 rad（18°），hip roll 搖擺只剩 0.03 rad（實驗一需要 0.27 rad，讓兩腳翻到腳掌邊緣）。
  - `swing_frac` 0.43：雙腳支撐時間很短，接近走路與小跑之間；抬腳 18 mm；尾巴 pitch 抬高 0.30 rad（17°）。
  - 腳放在髖後方 27 mm（站姿是 21 mm），配合尾巴把重心往後拉。

### 步驟 4：正式評估（`eval_cem1_20261001_003138`）
100 episode，速度 0.05–0.15 m/s、步頻 1.2–2.0 Hz、初始關節角 ±0.05 rad，全部隨機：

| 指標 | 結果 | 實驗一（參考） |
|---|---|---|
| 成功率（門檻 70%） | **100%** | 95% |
| 跌倒率 | **0%** | 0% |
| 平均行走距離（20 秒） | 1.95 m | — |
| 平均速度／指令 | 0.097／0.096 m/s（相對誤差 3.6%） | — |
| 最大側向偏移（平均） | 0.019 m | 0.136 m（中位數） |
| 最大方向誤差（平均） | 0.9° | 7.2° |
| 扭矩飽和比例 | 0.18% | — |
| 推擠測試（0.15 m/s，第 5、10、15 秒，參考用） | 成功 99%、跌倒 0% | — |

- 速度三個區間（0.05–0.08／0.08–0.12／0.12–0.15 m/s）、步頻三個區間全部 100% 成功。
- 各關節扭矩平均 0.08–0.14 N·m、p95 0.15–0.28 N·m（連續上限 0.8 N·m）；最大值 0.8 N·m 出現在生成落地的瞬間。

### 步驟 5：評估影片（`videos_rule_based_20261001_003931`）
- `exp2_rule_based_{slow,medium,fast}_eval_20261001.mp4`（0.06／0.10／0.14 m/s、1.6 Hz）；12 台全部成功，沒有失敗案例可留。
- 抽格確認：HUD 步數與時鐘一致（10.6 秒 33 步，1.6 Hz 預期約 34 步）。
- 第一次錄影（`videos_rule_based_20261001_003250`）鏡頭被隔壁機器人擋住（鏡頭側向 0.7 m 大於間距 0.5 m），
  改成間距 1.0 m、鏡頭 (0.3, −0.5, 0.04) m 後重錄；舊資料夾的影片已於 2026-10-07 清理（設定與 `videos.json` 保留）。

## 已知限制
- 只在平地、直線行走、0.05–0.15 m/s、1.2–2.0 Hz 的範圍內驗證；沒有轉向指令（目標方向固定）。
- 模擬中的馬達是 DCMotor 模型（kp 8、kd 0.3），沒有模擬伺服的位置量化、通訊延遲（`command_delay` 6 ms 尚未加入環境）與背隙。
- 觀測中用到 base 線速度，實機需要估測（IMU 積分或狀態估測器）。
- 步態很依賴尾巴做側向平衡；尾巴馬達故障時無法行走。
