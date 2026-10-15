# evaluation：模擬中的評估（不訓練，實驗二）

對 `control/` 產出的策略做**訓練以外**的測試：找出失效邊界、分析硬體負擔。這裡**不訓練任何模型**；要改進模型就回到 `control/` 訓練新版本，再回來重測。

| 子資料夾 | 內容 | 狀態 |
|---|---|---|
| [`generalization/`](generalization/README.md) | 壓力測試：一次只改一個條件（延遲、凸起、斜坡、台階、摩擦、馬達、質量、推擠、IMU、指令），找出失效邊界 | v1（56 條件）、v2（斜坡／台階修正跌倒判定）完成，2026-10-09 |
| `load_analysis/` | 馬達負載分析：平地、凹凸、靜止、推擠等情境下各關節的扭矩、發熱（RMS 對額定 0.294 N·m）、峰值與整機電流估計；影片 HUD 標註即時出力 | 規劃中（使用者 2026-10-15 同意） |
| （規劃）`generalization` `stress_v3` | 關節零點偏差、IMU 安裝傾斜、控制迴圈掉幀 | 規劃中 |

產出：`outputs/robot_v1/evaluation/<子資料夾>/`。原 `04_generalization/`，2026-10-15 重整為 `evaluation/generalization/`。
