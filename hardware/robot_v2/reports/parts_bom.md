# 零件清單（自動產生）

模型 v2，git 80ebc8b-dirty，2026-11-18 19:32。列印件質量 = CAD 體積 × 等效密度 1.05 g/cm³（實際列印後請秤重回填）。

| 零件 | 連桿 | 類型 | 體積 (cm³) | 質量 (g) | 螺絲數 | 數量 | 列印方式 |
|---|---|---|---|---|---|---|---|
| servo_left_hip_roll | base | servo | 16.57 | 21.0 | 0 |  |  |
| servo_left_hip_pitch | SIDE_thigh | servo | 16.57 | 21.0 | 0 |  |  |
| servo_left_knee | SIDE_thigh | servo | 16.57 | 21.0 | 0 |  |  |
| servo_left_ankle | SIDE_shin | servo | 16.57 | 21.0 | 0 |  |  |
| hip_block_left | SIDE_hip | print | 13.78 | 14.5 | 16 | x2（左右同一個檔案） | 側躺列印（一片 pitch 手臂的外側面貼床）；另一片 pitch 手臂懸空，需要支撐 |
| thigh_left | SIDE_thigh | print | 8.85 | 9.3 | 8 | x2（左右同一個檔案） | 側立列印：側板的 21 mm 寬度朝上，層線沿大腿長度 |
| shin_left | SIDE_shin | print | 8.94 | 9.4 | 12 | x2（左右同一個檔案） | 側立列印：手臂的 21 mm 寬度朝上，層線沿小腿長度 |
| foot_left | SIDE_foot | print | 24.68 | 25.9 | 8 | x2（左右同一個檔案） | 腳底朝下平放列印 |
| foot_pads_left | SIDE_foot | tpu | 1.15 | 1.2 | 0 | 每腳 4 片，TPU（共 8 片，STL 內含 4 片） | TPU 列印，用瞬間膠黏在腳底 0.5 mm 的凹槽 |
| torso_shell | base | print | 84.80 | 89.0 | 0 | x1 | 頂面朝下列印（開口朝上）；hip_roll 馬達上方的橫樑是短橋接 |
| torso_floor | base | print | 13.44 | 14.1 | 4 | x1 | 平放列印；4 顆 M2 螺絲鎖進軀幹底部的熱熔螺母，拆下即可換電池 |
| servo_tail_yaw | base | servo | 16.57 | 21.0 | 0 |  |  |
| servo_tail_pitch | tail_yaw_link | servo | 16.57 | 21.0 | 0 |  |  |
| tail_mount | base | print | 9.83 | 10.3 | 8 | x1 | 背板貼床列印（上下夾板朝上長出） |
| tail_yaw_bracket | tail_yaw_link | print | 8.87 | 9.3 | 12 | x1 | 側立列印（y 面貼床） |
| tail_pitch_bracket | tail_pitch_link | print | 10.11 | 10.6 | 9 | x1 | 側立列印；碳管用 AB 膠固定，M2 止付螺絲（熱熔螺母）輔助 |
| tail_rod | tail_pitch_link | bought | 4.84 | 7.7 | 0 | 碳纖維管 Ø8/6 mm，長 220 mm |  |
| tail_tip | tail_pitch_link | print | 10.99 | 11.5 | 1 | x1 | 圓柱軸向立著列印；後端 Ø20 × 20 mm 孔放鋼配重，AB 膠封口 |
| tail_ballast | tail_pitch_link | bought | 6.16 | 28.5 | 0 | 鋼配重，質量 = tip.mass − 列印座質量 |  |

| 連桿 | 質量 (g) | 質心（連桿座標，mm） |
|---|---|---|
| base | 302.1 | (25.1, 0.1, 13.3) |
| left_hip | 19.3 | (0.0, 0.0, -17.0) |
| left_thigh | 53.7 | (-0.0, -0.0, -35.0) |
| left_shin | 34.0 | (-0.0, 0.0, -36.2) |
| left_foot | 29.5 | (0.0, 0.0, -19.4) |
| right_hip | 19.3 | (0.0, -0.0, -17.0) |
| right_thigh | 53.7 | (-0.0, 0.0, -35.0) |
| right_shin | 34.0 | (-0.0, -0.0, -36.2) |
| right_foot | 29.5 | (0.0, -0.0, -19.4) |
| tail_yaw_link | 33.9 | (-28.2, -0.0, -0.0) |
| tail_pitch_link | 61.3 | (-187.3, 0.0, -0.0) |
| **合計** | **670.2** | |
