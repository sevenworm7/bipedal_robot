# 00_test：實驗一（測試管線）

使用參考模型，略過建模，直接執行 rule-based → 訓練管線。目的是確認整套流程可行，之後再移植到實驗二（01 → 02 → 03）。

- 機器人：`references/雙足機器人/雙足機器人_髖關節位置修正/Biped_Robot_Master_URDF_unity_urdf/Biped_Robot_Master_URDF.urdf`，robot 名稱為 `ref_biped`。
- 規格：`rule_based/PROMPT.md` 與 `training_pipeline/PROMPT.md`（複製自 02、03，兩份都要完整讀過），重點摘要在同資料夾的 `CLAUDE.md`。
- 這些複本可以隨測試結果修改，**不要修改 02、03 中的原始版本**。實驗一結束後，和使用者一起比較原始版本與測試後版本的差異。
- 影片檔名使用 `exp1_` 開頭，例如 `exp1_iter3_ppo_eval_20260918.mp4`。

## 進度（詳見 `README.md`）
- 階段 A（機器人接入模擬器）、階段 B（介面／環境／指標／影片）：已完成，2026-09-18
- 階段 C（rule-based）：**完成，使用者已確認（2026-09-19）**。採用參數 `rule_based/params/cem2_best.json`：100 episode 成功率 95%、跌倒 0%、側向偏移中位數 0.136 m、方向誤差中位數 7.2°、速度比中位數 1.02；推擠測試 88%。數據、影片與已知限制見 `rule_based/README.md`
- 階段 D（訓練管線）：**完成（2026-09-23），等待使用者檢視**。三輪自我蒸餾跑完：PPO 各輪成功率 100%、跌倒 0%；最終蒸餾模型成功率 99%、跌倒 0%、側向偏移中位數 0.072 m、方向誤差 2.6°、推擠 100%。各輪數據見 `training_pipeline/README.md`，總結與結論見 `outputs/summary.md`
- 修改 `shared/` 或 `robots/` 後，必須重新執行 `scripts/check_ref_biped.py` 與 `scripts/smoke_env.py`

## 實作重點
- 介面契約在 `shared/interface.py`（觀測 38 維、動作 8 維、50 Hz；2026-09-19 使用者同意加入 `gait_freq`，原本 37 維）；環境為 `shared/env`（gym id `Biped-Test-Walk-v0`）。
- 關節一律用 `LEG_JOINT_NAMES` 的順序存取（env 的 `leg_ids`），模擬器內部的關節順序不同。
- rule-based 的成功判定使用 `shared/metrics.py` 的 `rule_based_success`；評估時設定 `cfg.record_episodes=True`，從 `env.completed_episodes` 讀取每個 episode 的摘要。
- 錄影：設定 `cfg.eval_camera=True` 並加上 `--enable_cameras`，再用 `env.capture_frame()` 與 `shared/video_overlay.py`；不要使用 `env.render()`，因為 viewer 的取景不準。

## 資料夾
```
shared/              # 本實驗的介面契約：觀測/動作定義、影片疊加、評估指標
  robots/ref_biped.py  # ArticulationCfg、關節順序、站姿、致動器參數
  robots/usd/          # 由 URDF 轉換出的 USD（不進 git，可隨時重新產生）
rule_based/          # PROMPT.md、CLAUDE.md（複本）、RuleBasedBipedController、config、評估腳本
training_pipeline/   # PROMPT.md、CLAUDE.md（複本）、資料收集、BC、PPO、篩選、蒸餾、評估
outputs/             # rule_based/<run>/、training/<run>/、summary.md（不進 git）
```
- 只能 import `00_test/shared/` 與 `tools/`，**不要 import 根目錄的 `shared/`**（那是實驗二用的），讓兩次實驗保持獨立。
- 撰寫時就要考慮之後的移植：管線程式碼不可寫死 `ref_biped` 的任何數值。移植時的做法是把 `rule_based/`、`training_pipeline/`、`shared/` 複製到 02、03 與根目錄 `shared/`，再把 robot 設定換成 `custom_biped`。

## 參考模型的處理方式
`references/` 不可修改，所有修正都寫在 `shared/robots/ref_biped.py` 的 `ArticulationCfg` 與轉換參數中：
- **前進方向**：設計圖標示 URDF 的 +Y 為前方。root 繞 Z 軸旋轉 −90°（`INIT_YAW`，四元數為 XYZW），讓世界座標的 +X 成為前進方向；模擬中已確認左腳位於世界座標的 +Y。由參數 `FORWARD_AXIS` 控制，**已確認使用 +Y**（膝蓋像人類；缺點是腳尖只有 5.5 cm、腳跟有 12.5 cm）。
- **場景排列（已決定）**：所有機器人一開始都朝向前進方向（`init_yaw_range=(0,0)`），並排成一列（左右間距 1 m，每列 64 台，列與列相距 10 m），然後同時往前走。不使用隨機初始朝向。
- **高度**：base_link 原點在零位時位於腳底平面。微蹲站姿（髖 −0.25、膝 +0.5、踝 −0.25 rad）的初始高度由幾何計算（`INIT_BASE_HEIGHT`），並已在模擬中確認。
- **致動器**：URDF 中的 effort／velocity 都是預設值 100，改用 `actuators` 設定。**扭矩上限（已決定）：8 個腿部關節統一 80 N·m。** 另外自行設定：速度上限 20 rad/s、kp 300、kd 10、armature 0.01（都在 `ref_biped.py`）。每次評估都要記錄各關節實際用到的扭矩（平均值、第 95 百分位、最大值），作為實驗二選馬達的依據；靜態估算顯示 hip roll 約需 25–36 N·m，pitch 類關節約 2–8 N·m。
- **碰撞體**：URDF 使用 STL mesh，轉換時使用凸包（Convex Hull）。
- **自由度**：每隻腳只有 hip roll、hip pitch、knee、ankle pitch 4 個關節，沒有 hip yaw 與 ankle roll，因此轉向和側向平衡只能靠 hip roll 與落腳點調整。
- **關節正方向**（模擬中實測）：髖 roll + 腳往右｜髖 pitch + 腳往後｜膝 + 小腿往後｜踝 + 腳尖往下。
- STL mesh 不進 git，本實驗需要本機上的 `references/` 資料夾完整存在。
- **軀幹關節（已決定：固定）**：`Torso_Yaw_Joint`、`Torso_Pitch_Joint` 用高剛性 PD 維持在 0，不放入觀測和動作空間。上身只有 1.65 kg（佔總重 5.8%），固定後對平衡的影響可忽略。
- **側向平衡是最大難點**：單腳支撐時，全身重心在支撐腳中心內側約 11 cm，但腳掌半寬只有 6 cm；又因為沒有 ankle roll，無法用靜態方式把重心移到支撐腳上方。rule-based 需要採用左右搖擺的動態步態，並使用短的單腳支撐時間。

## rule-based 驗收標準
- 每個 episode 的成功條件：未跌倒、至少走 10 步（步數以腳掌著地次數計算，見 `/eval-video`），且前進距離 ≥ 距離下限。
- 步數定義補充（2026-09-19 使用者確認）：騰空 ≥ 0.03 s（`min_step_air_time`，2 個控制步）才算一步，濾掉腳掌邊緣翻滾造成的接觸抖動；著地由環境自行追蹤離地→著地轉換（`compute_first_contact` 有漏算 bug，見 `shared/env/README.md`）。
- 距離下限：`0.8 × 實際步數 × 指令步長`；預設步長 0.10 m 時，10 步至少 0.8 m。使用**實際步數**計算，原地踏步無法通過。
- 100 個評估 episode 中，成功率 ≥ 70%（相當於 PROMPT 的跌倒率 < 30%），才能進入訓練階段。
- **推擠測試**：另外做一組推擠評估，只作為參考指標，不納入 70% 門檻。
- 以上全部已經使用者確認；觀測／動作定義（`shared/interface.py`）也已確認，之後有問題再討論。

## 訓練管線的決定（2026-09-22 使用者確認）
- **情境**：平地＋直線（和 rule-based 相同），只隨機速度指令、步頻、初始關節姿態。不做不平地形與轉向。
- **強健性隨機化**：PPO 訓練時只加輕微推擠（約 0.2 m/s）；摩擦、質量、觀測雜訊留到實驗二。
- **獎勵調整（2026-09-22 使用者確認）**：第 1 輪 PPO 的著地節奏和時鐘脫節（步數為時鐘的 0.5–1.8 倍，成功率 95% → 80%），因此加入「著地節奏跟上時鐘」的獎勵，並用同一個 BC 重跑第 1 輪 PPO；舊的第 1 輪結果保留作為對照。
- **篩選門檻（2026-09-22 使用者確認）**：第 1 輪維持暫定門檻。
- **側向偏移（2026-09-23 使用者確認）**：第 2 輪 PPO 在慢速指令側向漂移（慢速組最大側向偏移中位數 0.30 m，篩選後慢速組為空 → 停止規則觸發）。處理方式：`lateral_deviation` 權重 −10 → −40，從第 1 輪蒸餾模型重跑第 2 輪 PPO（舊第 2 輪留作對照）；側向門檻三輪都維持 0.15 m，不再收緊（搖擺步態本身就擺約 0.1 m）。
- **訓練量**：每輪 PPO 4096 環境 × 約 1500 iterations。每輪的 PPO 與蒸餾 BC 都用同一組固定評估集（100 episode、固定 seed）計算指標；影片錄每輪 PPO，另加最後一輪蒸餾模型。

## 自我蒸餾的篩選門檻（暫定，使用者對可行性有疑慮）
1. **硬性門檻**（每個 episode 都要全部通過）：
   - 未跌倒且跑完整個 episode
   - 滿足上面的步數與距離下限
   - 最大側向偏移 < 0.15 m
   - 前進方向誤差 < 10°
   - 平均速度誤差 < 指令速度的 25%
   - 扭矩達到上限的時間比例 < 5%
2. **排名**：通過硬性門檻的 episode，依指令分組（例如慢、中、快），各組分別取 reward 前 50%，避免資料集中在簡單的指令。
3. **標記方式**：rollout 時保留少量探索雜訊，但標籤使用 policy 的平均動作；每輪混入約 20% 上一輪的資料。
4. **逐輪收緊**：原計畫側向偏移門檻第 1 輪 0.15 m、第 3 輪 0.08 m；**2026-09-23 使用者決定三輪都維持 0.15 m**（見上方「訓練管線的決定」）。
5. **側向偏移的背景（2026-09-19）**：rule-based 的搖擺步態本身就會左右偏移（最大側向偏移中位數 0.136 m、p90 0.228 m），約一半的教師 episode 超過 0.15 m。使用者的看法：這是步態特性，交給 PPO 訓練時慢慢改善即可。篩選前的「只統計」分析要特別列出這一項的分布。

**篩選後必須先檢查再繼續**：
- `filter_report.json` 要列出每個門檻各自淘汰的比例、各指令分組的保留量，以及總保留的 episode 數與 step 數。
- 若總保留率 < 30%，或任一指令分組沒有資料：**停下來回報使用者，一起討論門檻**。不可自行放寬門檻後繼續。
- 第一次篩選前，先對 PPO rollout 做一次「只統計、不篩選」的分析（各指標的分布與百分位數），讓使用者先看到門檻設在哪裡會淘汰多少資料。

## 完成條件
rule-based 的跌倒率 < 30%，並完成 3 輪自我蒸餾，每輪都有評估影片與指標。最後在 `outputs/summary.md` 整理：
- 各輪指標比較表
- 管線中需要修正的地方
- 哪些問題來自參考模型本身的限制
- 規格文件的差異：`00_test/{rule_based,training_pipeline}/` 中的 PROMPT.md、CLAUDE.md，與 02、03 原始版本的差異與修改原因

使用者確認實驗一完成後，才開始實驗二。
