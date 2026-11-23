# docs/videos：精選評估影片（git LFS）

`outputs/` 不進 git，這裡保存**重要影片的壓縮複本**，讓結果在 git（之後是 GitHub）裡也看得到。2026-11-18 新增，使用者選定方案 A：以 medium 速度為主，加上代表性的失效例子。

- **怎麼產生**：`tools\isaacpy.cmd tools\archive_videos.py` 讀 [`manifest.csv`](manifest.csv)（dest、source、note），用 H.264 重新壓縮（解析度不變、CRF 28、無音訊），約為原檔的 1/8（主評估影片 27 MB → 3.3 MB，HUD 文字仍清楚）。
- **只新增、不覆蓋**：已存在的檔案會跳過；要換影片就用新檔名，舊的保留。
- **git LFS**：`.gitattributes` 讓 `docs/videos/**/*.mp4` 走 LFS；clone 後若影片只是指標檔，執行 `git lfs pull`。
- **預算**：每個機器人版本約 50 MB 以內。新增影片時在 `manifest.csv` 加一列，再跑一次工具。
- 完整的三種速度、全部壓力測試條件仍在 `outputs/`（各資料夾的 `videos.json` 記錄每段的指標）。

## robot v1（21 段，共 45.8 MB）

| 影片 | 內容 |
|---|---|
| [`control/exp2_rule_based_medium_eval_20261001.mp4`](robot_v1/control/exp2_rule_based_medium_eval_20261001.mp4) | 規則式老師 100%／0%；尾巴擺動做側向平衡 |
| [`control/exp2_iter3_distill_medium_eval_20261001.mp4`](robot_v1/control/exp2_iter3_distill_medium_eval_20261001.mp4) | 平地最終蒸餾 99%／1%（需要 base 線速度，不可部署） |
| [`control/exp2_sim2real_student_v5_flat_v1_medium_eval_20261007.mp4`](robot_v1/control/exp2_sim2real_student_v5_flat_v1_medium_eval_20261007.mp4) | student v5 平地 100%／0% |
| [`control/exp2_sim2real_student_v5_dr_v1_medium_eval_20261007.mp4`](robot_v1/control/exp2_sim2real_student_v5_dr_v1_medium_eval_20261007.mp4) | student v5 中等隨機化 100%／0% |
| [`control/exp2_sim2real_student_v5_rough_eval_v1_medium_eval_20261007.mp4`](robot_v1/control/exp2_sim2real_student_v5_rough_eval_v1_medium_eval_20261007.mp4) | student v5 ±8 mm 凸起 94%／1% |
| [`control/exp2_sim2real_student_v5_dr_rough_eval_v1_medium_eval_20261007.mp4`](robot_v1/control/exp2_sim2real_student_v5_dr_rough_eval_v1_medium_eval_20261007.mp4) | student v5 隨機化＋地形 85.7%／3.2%（504 ep） |
| [`control/exp2_sim2real_student_v5_dr_rough_eval_v1_failure_eval_20261007.mp4`](robot_v1/control/exp2_sim2real_student_v5_dr_rough_eval_v1_failure_eval_20261007.mp4) | student v5 最難場景的失敗例（方向或速度未達標） |
| [`control/exp2_sim2real_student_v5_gait_eval_v1_medium_eval_20261007.mp4`](robot_v1/control/exp2_sim2real_student_v5_gait_eval_v1_medium_eval_20261007.mp4) | student v5 站立／原地踏步／啟停 100%／0% |
| [`control/exp2_sim2real_student_v3_dr_rough_eval_v1_medium_eval_20261006.mp4`](robot_v1/control/exp2_sim2real_student_v3_dr_rough_eval_v1_medium_eval_20261006.mp4) | 對照：student v3 跛行步態（左 49／右 14 mm，鏡像對稱訓練前） |
| [`generalization/stress_v1_01_delay_060ms.mp4`](robot_v1/generalization/stress_v1_01_delay_060ms.mp4) | 伺服延遲 60 ms：通過（訓練範圍 0–40 ms） |
| [`generalization/stress_v1_01_delay_080ms.mp4`](robot_v1/generalization/stress_v1_01_delay_080ms.mp4) | 伺服延遲 80 ms：開始失效（成功 75%） |
| [`generalization/stress_v1_02_bumps_08mm.mp4`](robot_v1/generalization/stress_v1_02_bumps_08mm.mp4) | 凸起 8 mm：92%／3% |
| [`generalization/stress_v1_02_bumps_12mm.mp4`](robot_v1/generalization/stress_v1_02_bumps_12mm.mp4) | 凸起 12 mm：跌倒 18% |
| [`generalization/stress_v1_09_push_35cms.mp4`](robot_v1/generalization/stress_v1_09_push_35cms.mp4) | 推擠 0.35 m/s：跌倒 12%（訓練 0.25 m/s） |
| [`generalization/stress_v1_05_motor_050pct.mp4`](robot_v1/generalization/stress_v1_05_motor_050pct.mp4) | 馬達力量 50%：仍通過 |
| [`generalization/stress_v2_07_steps_up_10mm.mp4`](robot_v1/generalization/stress_v2_07_steps_up_10mm.mp4) | 上 10 mm 台階：成功 40%、跌倒 20%（最明顯的弱點） |
| [`generalization/stress_v2_06_slope_down_8deg.mp4`](robot_v1/generalization/stress_v2_06_slope_down_8deg.mp4) | 下坡 −8°：速度過快 1.36 倍，但幾乎不跌倒 |
| [`generalization/stress_v3_13_joint_offset_5deg.mp4`](robot_v1/generalization/stress_v3_13_joint_offset_5deg.mp4) | 關節零點偏差 5°：39%／跌倒 27%（實機要校正到 ±2°） |
| [`generalization/stress_v3_16_combined_typical.mp4`](robot_v1/generalization/stress_v3_16_combined_typical.mp4) | 實機常見誤差組合（零點 2°＋IMU 3°＋遺失 5%）：99%／1% |
| [`load_analysis/load_v1_04_flat_medium.mp4`](robot_v1/load_analysis/load_v1_04_flat_medium.mp4) | 平地中速的馬達出力 HUD（hip pitch RMS 60%） |
| [`load_analysis/load_v1_01_stand.mp4`](robot_v1/load_analysis/load_v1_01_stand.mp4) | 靜止站立：5% 雙腳卡住，右 hip roll 頂在上限（backlog #1） |

## robot v2（6 段，共 18.6 MB）

| 影片 | 內容 |
|---|---|
| [`control/exp2_sim2real_student_v2ft_flat_v1_medium_eval_20261118.mp4`](robot_v2/control/exp2_sim2real_student_v2ft_flat_v1_medium_eval_20261118.mp4) | v2 student v2ft 平地 100%／0% |
| [`control/exp2_sim2real_student_v2ft_dr_v1_medium_eval_20261118.mp4`](robot_v2/control/exp2_sim2real_student_v2ft_dr_v1_medium_eval_20261118.mp4) | v2 student v2ft 中等隨機化 100%／0% |
| [`control/exp2_sim2real_student_v2ft_rough_eval_v1_medium_eval_20261118.mp4`](robot_v2/control/exp2_sim2real_student_v2ft_rough_eval_v1_medium_eval_20261118.mp4) | v2 student v2ft 不規則地形 ±8 mm 99%／0% |
| [`control/exp2_sim2real_student_v2ft_dr_rough_eval_v1_medium_eval_20261118.mp4`](robot_v2/control/exp2_sim2real_student_v2ft_dr_rough_eval_v1_medium_eval_20261118.mp4) | v2 student v2ft 隨機化＋地形 93%／1% |
| [`control/exp2_sim2real_student_v2ft_dr_rough_eval_v1_failure_eval_20261118.mp4`](robot_v2/control/exp2_sim2real_student_v2ft_dr_rough_eval_v1_failure_eval_20261118.mp4) | v2 student v2ft 隨機化＋地形的失敗例（不跌倒，方向誤差超過 10°） |
| [`control/exp2_sim2real_student_v2ft_gait_eval_v1_medium_eval_20261119.mp4`](robot_v2/control/exp2_sim2real_student_v2ft_gait_eval_v1_medium_eval_20261119.mp4) | v2 student v2ft 步態多樣性（站立／原地／變速）100%／0% |
| [`control/exp2_sim2real_student_v2slow_f08_flat_slow_v1_medium_eval_20261123.mp4`](robot_v2/control/exp2_sim2real_student_v2slow_f08_flat_slow_v1_medium_eval_20261123.mp4) | 慢步態模型 v2slow（2026-11-23）：平地慢場景 0.8 Hz、0.05 m/s，10 秒約 50 步（碎步，未照時鐘的約 16 步）；慢場景成功 100%／0% |
| [`control/exp2_sim2real_student_v2ft_f08_flat_slow_v1_medium_eval_20261123.mp4`](robot_v2/control/exp2_sim2real_student_v2ft_f08_flat_slow_v1_medium_eval_20261123.mp4) | 對照：v2ft 在同一慢場景，10 秒約 28 步 |

## robot v2 評估（4 段，共 4.8 MB）

| 影片 | 內容 |
|---|---|
| [`load_analysis/load_v1_01_stand.mp4`](robot_v2/load_analysis/load_v1_01_stand.mp4) | v2 靜止站立：兩腳互頂，左 hip roll 持續 0.41 N·m（額定 141%）→ 實機改用固定站姿 |
| [`generalization/stress_v1_01_delay_060ms.mp4`](robot_v2/generalization/stress_v1_01_delay_060ms.mp4) | v2 延遲 60 ms：46%／0%（不跌倒但走太快；v1 100%）→ 實機延遲要 ≤ 40 ms |
| [`generalization/stress_v2_07_steps_up_10mm.mp4`](robot_v2/generalization/stress_v2_07_steps_up_10mm.mp4) | v2 上台階 10 mm：74%／5%（扣除出生點異常；v1 40%／20%） |
| [`generalization/stress_v3_13_joint_offset_5deg.mp4`](robot_v2/generalization/stress_v3_13_joint_offset_5deg.mp4) | v2 關節零點偏差 5°：59%／10%（v1 39%／27%；實機仍要校正到 ±2°） |
