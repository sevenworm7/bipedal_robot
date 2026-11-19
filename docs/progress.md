# Progress — 雙足機器人（帶主動尾巴）

> **計畫、狀態、下一步只寫在這裡**（2026-11-18 文件整理後的規則）。各資料夾的 README 只寫用途、執行方式與該階段的紀錄，不再寫「下一步」。
> 待辦問題見 [`backlog.md`](backlog.md)，決策見 [`decisions.md`](decisions.md)，數字見 [`results.md`](results.md)。

## 目前狀態（2026-11-19）

- 分支 `robot-v2`（已 push 到 GitHub `origin`；合併回 `master` 等使用者決定）；robot v2 已凍結（tag `robot-v2`：大腿 70、小腿 55 mm、膝 0–110°，接線方案 B）。
- **robot v2 訓練完成（2026-11-19）**：最終模型 **student v2ft**（`outputs/robot_v2/control/sim2real/student_v2ft_20261118_230657/student.pt`），通過全部部署門檻，詳見下方與 [`results.md`](results.md)「robot v2」。
- **v2 壓力測試與負載分析完成（2026-11-19）**：多數條件比 v1 好；兩個實機要求：**站立時改用固定站姿**（模型站立時兩腳互頂，`decisions.md` #24）、**伺服延遲 ≤ 40 ms**（#25）。詳見 [`results.md`](results.md)「v2 泛化、壓力測試與馬達負載」。
- **v2 板上程式離線完成（2026-11-19）**：student v2ft 匯出成 int8（模擬閉迴路和原模型相當）、芒果磚 50 Hz 閉迴路韌體、PC 指令工具；PC 單元測試和模擬逐步一致，韌體可編譯，**尚未上實機**（`deploy/firmware/README.md`、[`results.md`](results.md)「v2 板上部署準備」）。
- 實機：組員交接包調查完成（`deploy/test_log.md` 2026-11-19）。v1 的「約 8 步」是開迴路回放模擬軌跡，**實機閉迴路尚未驗證**；v2 的大腿、小腿尚未重印。

## 下一步

1. **板上量測（需要實機，不動馬達）**：燒錄 `biped_runtime.uf2`，執行 `BENCH`（板上推論一致性與時間）、`BUSTEST 100`（sync read 是否支援、讀取時間），確認讀取＋推論＋寫入 < 20 ms（`backlog.md` #17）。燒錄前先和使用者確認。
2. 重印 v2 的大腿、小腿，依 `hardware/robot_v2/ASSEMBLY.md` 改裝與接線（先在實物上比對最長的幾段線）。
3. v2 校正：方向、零點、IMU 軸向與偏置，更新 `deploy/calibration/robot_v2.json` 為 verified（`backlog.md` #18），重新匯出與編譯。
4. 懸空閉迴路 → 安全繩上桌（`deploy/README.md` 步驟 5、6）；硬體（供電 #21）稍後與使用者討論。

## robot v2 接續訓練（使用者 2026-11-18 同意；2026-11-19 完成）

v2 的觀測、動作、控制頻率都和 v1 相同（10 DOF、student 210 維），所以不重跑 rule-based、BC 和平地訓練，直接接續 v1 的 sim2real 模型。
過程見 [`control/sim2real/log_robot_v2.md`](../control/sim2real/log_robot_v2.md)。成功率一律用 `rescore.py` 的 sim-to-real 判定（`evaluate.py` 印出的是舊判定，見 log）。

| 步驟 | 內容 | run（`outputs/robot_v2/control/sim2real/`） | 耗時 | 結果 |
|---|---|---|---|---|
| 0 | 零樣本評估：v1 student v5 直接放到 v2 | `eval_<場景>_v1s5_zeroshot_20261118_*` | 6 分鐘 | 不跌倒，但速度比 0.71–0.85；`flat_v1` 92%、`dr_v1` 66%、`rough_eval_v1` 75%、組合 23% → 需要微調 |
| 1 | teacher 微調（v1 第五輪相同設定，1500 iterations） | `teacher_v2ft_20261118_221706` | 49 分鐘 | 組合場景 95%／0%（特權） |
| 2 | student 蒸餾（DAgger 120 輪，含鏡像樣本） | `student_v2ft_20261118_230657` | 13 分鐘 | val_mse 0.0028 |
| 3 | 評估、對稱、停頓、影片 | `eval_<場景>_v2s1_*`、`videos_<場景>_student_v2ft_*` | 約 50 分鐘 | **`flat_v1` 100%／0%、`dr_v1` 100%／0%、`rough_eval_v1` 99%／0%、組合 92.7%／0.4%（504 ep）、`gait_eval_v1` 100%／0%**；對稱 0.00、無自發停頓 |

- 和 v1 student v5 相比，不規則地形 94 → 99%、組合場景 85.7 → 92.7%、跌倒 3.2 → 0.4%。原本擔心的風險（原地抬腳上限 56 → 24 mm）沒有出現。
- 剩下的弱點和 v1 相同：沒有外部定位，側向偏移 p50 0.10–0.16 m／20 秒；組合場景的失敗主要是方向誤差 > 10°。

## 各階段狀態

| 階段 | 位置 | 狀態 | 重點結果 |
|---|---|---|---|
| 實驗一：參考模型驗證管線 | `archive/exp1_reference/` | 完成（2026-09-29） | rule-based 95% → 最終蒸餾 99%／跌倒 0% |
| 機器人模型 v1 | `hardware/robot_v1/` | 凍結（2026-09-30，tag `robot-v1`） | 0.696 kg、10 顆 HD-1910 |
| 規則式老師 | `control/rule_based/` | 完成（2026-10-01） | 100%／0% |
| 平地訓練 | `control/flat_pipeline/` | 完成（2026-10-01） | 99%／1%（需要 base 線速度，不可部署） |
| sim-to-real 強健性訓練（v1） | `control/sim2real/` | 完成（2026-10-07） | **student v5**：平地與隨機化 100%、凹凸 94%、組合 85.7%（只用 IMU 與伺服讀值） |
| 泛化／壓力測試 | `evaluation/generalization/` | 完成（2026-10-09；stress_v3 2026-10-15） | 延遲 60 ms、摩擦 0.2–1.5、馬達 50%、質量 ×1.5 仍 ≥ 95%；關節零點要校正到 ±2° |
| 馬達負載分析 | `evaluation/load_analysis/` | 完成（2026-10-15） | 各關節 RMS ≤ 額定 70%；整機平均約 3 A、峰值 8.8 A |
| 實機部署（v1） | `deploy/` | **進行中** | 實體已列印組裝、芒果磚控制；開迴路回放模擬軌跡在桌上走約 8 步（2026-11-19 更正：不是閉迴路） |
| 機器人模型 v2 | `hardware/robot_v2/` | 凍結（2026-11-18，tag `robot-v2`） | 縮短腿讓馬達線夠長；只需重印大腿、小腿；模擬站立通過 |
| v2 訓練（接續 v1 模型） | `control/sim2real/` | 完成（2026-11-19） | **student v2ft**：平地與隨機化 100%、凹凸 99%、組合 92.7%／跌倒 0.4%（504 ep） |
| v2 泛化／壓力測試與負載分析 | `evaluation/` | 完成（2026-11-19） | 凸起、零點、IMU 傾斜都比 v1 好；延遲 60 ms 46%（實機要 ≤ 40 ms）；站立時兩腳互頂 → 實機用固定站姿（交接模擬 100%／0%） |

### 實機部署的子步驟（定義見 [`deploy/README.md`](../deploy/README.md)）

| 步驟 | 狀態（2026-11-19） |
|---|---|
| 1. 採購與試印 | 列印與備料完成；秤重、回填尚未做 |
| 2. 單顆馬達台架測試 | 未做 |
| 3. 組裝與校正 | v1 已組裝；組員的 v1 方向／零點已整理成 v2 暫定校正檔（`deploy/calibration/robot_v2.json`，未驗證） |
| 4. 控制程式 | v2 程式離線完成（`deploy/export`、`firmware`、`host`），尚未上實機；組員 v1 韌體只做開迴路回放 |
| 5. 懸吊測試 | — |
| 6. 平地行走 | v1 開迴路回放約 8 步（不是模型閉迴路；受供電與線長問題限制） |
| 7. 進階測試 | 未做 |

## Archive

- 2026-11-19：調查組員的芒果磚交接包（放 `references/`，decisions #26）；v2 板上程式離線完成：int8 匯出（模擬 100／100／100／94%）、50 Hz 韌體、PC 工具，PC 單元測試與模擬一致；沿用實機 ID（#27）、板上 int8 推論（#28）。
- 2026-11-19：robot v2 壓力測試（33 條件）與負載分析（12 情境），和 v1 對照；發現站立互頂（backlog #15）與台階出生點異常（#16）；決定實機站立用固定站姿（decisions #24）、延遲 ≤ 40 ms（#25）；影片 4 段放進 `docs/videos/robot_v2/`。
- 2026-11-19：robot v2 接續訓練完成（零樣本 → teacher 微調 → student 蒸餾 → 評估），student v2ft 通過全部門檻；v2 影片 6 段（18.6 MB）放進 `docs/videos/robot_v2/`。
- 2026-11-18：精選影片 21 段（45.8 MB，git LFS）放進 `docs/videos/`。
- 2026-11-18：文件整理，新增 `docs/`（本檔、`backlog.md`、`decisions.md`、`results.md`、`architecture.md`）；`hardware/v2_backlog.md` 拆到 `docs/backlog.md` 與 `hardware/robot_v2/design_calc.md`；sim2real 的 v1 紀錄拆到 `control/sim2real/log_robot_v1.md`。
- 2026-11-18：robot v2 建模並凍結（腿長計算 → 產生零件與 URDF → 干涉、靜態、模擬檢查）。
- 2026-11-18：使用者回報實機測試（2026-10-15 ～ 11-18），見 `deploy/test_log.md`。
- 2026-10-15：專案依「是什麼」分層重整；完成 stress_v3 與馬達負載分析。
- 2026-10-09：泛化與壓力測試。
- 2026-10-06 ～ 10-07：sim2real 強健性訓練，最終 student v5。
- 2026-10-01：規則式老師、平地訓練三輪。
- 2026-09-30：robot v1 建模並凍結。
- 2026-09-29：實驗一完成，開始實驗二。
