# 馬達方向與安裝表（自動產生）

`direction` = 馬達位置值增加時，關節往 + 方向（`params.yaml` joints 的正方向）轉動為 +1，反之 −1。
計算方式：主擺臂朝向 · 關節軸 × `servo.position_increase_ccw`。目前假設位置值增加 = 從主擺臂看逆時針，**第一次上電前要實測確認**；若實測相反，把 `position_increase_ccw` 改成 −1 重新產生，整張表會一起翻轉。

關節角（rad）= direction × (位置值 − 2048) × 2π / 4096。

| ID | 關節 | 匯流排 | 主擺臂朝向（世界座標，零位） | 本體朝向 | direction |
|---|---|---|---|---|---|
| 1 | left_hip_roll | left | (1, 0, 0) | (0, 0, 1) | +1 |
| 2 | left_hip_pitch | left | (0, 1, 0) | (0, 0, -1) | +1 |
| 3 | left_knee | left | (0, 1, 0) | (0, 0, 1) | +1 |
| 4 | left_ankle_pitch | left | (0, 1, 0) | (0, 0, 1) | +1 |
| 5 | right_hip_roll | right | (1, 0, 0) | (0, 0, 1) | +1 |
| 6 | right_hip_pitch | right | (0, -1, 0) | (0, 0, -1) | -1 |
| 7 | right_knee | right | (0, -1, 0) | (0, 0, 1) | -1 |
| 8 | right_ankle_pitch | right | (0, -1, 0) | (0, 0, 1) | -1 |
| 9 | tail_yaw | tail | (0, 0, -1) | (1, 0, 0) | -1 |
| 10 | tail_pitch | tail | (0, 1, 0) | (1, 0, 0) | +1 |
