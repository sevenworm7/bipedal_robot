# Progress — 雙足機器人（帶主動尾巴）

> **計畫、狀態、下一步只寫在這裡**（2026-11-18 文件整理後的規則）。各資料夾的 README 只寫用途、執行方式與該階段的紀錄，不再寫「下一步」。
> 待辦問題見 [`backlog.md`](backlog.md)，決策見 [`decisions.md`](decisions.md)，數字見 [`results.md`](results.md)。

## 目前狀態（2026-11-18）

- 分支 `robot-v2`；robot v2 已凍結（tag `robot-v2`：大腿 70、小腿 55 mm、膝 0–110°，接線方案 B）。
- 進行中：**robot v2 接續訓練**（下方「下一步」）。
- 實機：v1 實體可走約 8 步後腿軟跌倒（電流問題由其他成員處理）；v2 的大腿、小腿尚未重印。

## 下一步：robot v2 接續訓練（使用者 2026-11-18 同意）

v2 的觀測、動作、控制頻率都和 v1 相同（10 DOF、student 210 維），所以**不重跑 rule-based、BC 和平地訓練**，直接接續 v1 的 sim2real 模型。
執行時設 `CUSTOM_BIPED_VERSION=2`，產出放在 `outputs/robot_v2/control/sim2real/`，過程寫進 `control/sim2real/log_robot_v2.md`，數字寫進 [`results.md`](results.md)。
執行方式見 [`control/sim2real/README.md`](../control/sim2real/README.md)「程式與執行方式」。

| 步驟 | 內容 | 起點 | 預估 | 狀態 |
|---|---|---|---|---|
| 0 | **零樣本評估**：v1 的 student v5 直接放到 v2 上，跑固定評估集（`flat_v1`、`dr_v1`、`rough_eval_v1`、`dr_rough_eval_v1`、`gait_eval_v1`） | `outputs/robot_v1/control/sim2real/student_v5_20261007_003835/student.pt` | 約 10 分鐘 | 未開始 |
| 1 | **teacher 微調**：在 v2 上用 v1 最後一輪相同的場景與獎勵接續訓練；若第 0 步已接近門檻就縮短 | `outputs/robot_v1/control/sim2real/teacher_v5_20261006_234626/` | 約 50–80 分鐘 | 未開始 |
| 2 | **student 蒸餾**（DAgger，含鏡像樣本） | student v5 | 約 10 分鐘 | 未開始 |
| 3 | **評估**：同一組固定評估集與門檻，產生和 v1 的比較表與影片（`/eval-video`） | — | 約 30 分鐘 | 未開始 |

- **沿用 v1 的決定**（`control/sim2real/README.md`「開始前確認的事項」）：觀測（5 步歷史）、隨機化範圍、指令範圍（最高 0.15 m/s）、sim-to-real 判定（`scoring.py`）。
- **部署門檻不變**：`flat_v1` 與中等隨機化成功率 ≥ 95%、不規則地形 ≥ 85%，各難度跌倒率 ≤ 5%。
- **風險**：v2 的原地抬腳上限從 56 mm 降到 24 mm，不規則地形（最難 ±8 mm）可能比 v1 差。若過不了門檻，要先和使用者確認是降低地形難度還是繼續加強訓練。
- 和高度有關的設定：`fall_com_height` 由 `NOMINAL_COM_HEIGHT` 自動推導；`fall_foot_height` 0.026（踝高不變，可沿用）。
  `control/sim2real/config.py` 的 `init_checkpoint`、`control/flat_pipeline/config.py` 的 teacher_params 是 v1 專用，跑 v2 時要用參數指定。

## 之後的步驟

1. v2 訓練通過門檻 → 匯出 student（TorchScript／ONNX）與實機說明（`backlog.md` #9）。
2. 重印 v2 的大腿、小腿，依 `hardware/robot_v2/ASSEMBLY.md` 改裝與接線（先在實物上比對最長的幾段線）。
3. 實機校正與測試（下表）。

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
| 實機部署（v1） | `deploy/` | **進行中** | 實體已列印組裝、芒果磚控制；可走約 8 步後腿軟跌倒 |
| 機器人模型 v2 | `hardware/robot_v2/` | 凍結（2026-11-18，tag `robot-v2`） | 縮短腿讓馬達線夠長；只需重印大腿、小腿；模擬站立通過 |
| **v2 訓練（接續 v1 模型）** | `control/sim2real/` | **下一步** | — |

### 實機部署的子步驟（定義見 [`deploy/README.md`](../deploy/README.md)）

| 步驟 | 狀態（2026-11-18） |
|---|---|
| 1. 採購與試印 | 列印與備料完成；秤重、回填尚未做 |
| 2. 單顆馬達台架測試 | 未做 |
| 3. 組裝與校正 | 已組裝；校正數據尚未記錄到專案中 |
| 4. 控制程式 | 已能在芒果磚上執行；程式尚未放進專案 |
| 5. 懸吊測試 | — |
| 6. 平地行走 | 最多約 8 步（受電流與線長問題限制） |
| 7. 進階測試 | 未做 |

## Archive

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
