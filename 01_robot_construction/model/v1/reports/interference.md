# 干涉檢查

計算兩個連桿所有實體的重疊體積（列印件、馬達本體與擺臂、**螺絲頭**、**連接器插頭保留空間**），重疊 > 0.5 mm³ 視為干涉。
連接器保留空間假設插頭從馬達側面插入、再往外留 10 mm 給插頭與線材彎曲（實物待確認）。

## 1. 關節掃描（左腿；右腿是鏡像）

| 父連桿 | 子連桿 | 取樣關節 | 姿態數 | 最大重疊 (mm³) | 發生位置 |
|---|---|---|---|---|---|
| base | left_hip | left_hip_roll | 11 | 0.0 | — |
| left_hip | left_thigh | left_hip_pitch | 21 | 0.0 | — |
| left_thigh | left_shin | left_knee | 25 | 0.0 | — |
| left_shin | left_foot | left_ankle_pitch | 25 | 0.0 | — |
| base | left_thigh | left_hip_roll, left_hip_pitch | 30 | 0.0 | — |
| left_hip | left_shin | left_hip_pitch, left_knee | 16 | 0.0 | — |
| left_thigh | left_foot | left_knee, left_ankle_pitch | 20 | 0.0 | — |
| base | left_shin | left_hip_roll, left_hip_pitch, left_knee | 36 | 0.0 | — |
| base | left_foot | left_hip_pitch, left_knee, left_ankle_pitch | 12 | **92.1** | servo_tail_yaw × foot_left @ left_hip_pitch=45.0, left_knee=120.0, left_ankle_pitch=60.0 |
| base | tail_yaw_link | tail_yaw | 25 | 0.0 | — |
| tail_yaw_link | tail_pitch_link | tail_pitch | 22 | 0.0 | — |
| base | tail_pitch_link | tail_yaw, tail_pitch | 15 | 0.0 | — |

## 2. 同一連桿內（零件、插頭保留空間、螺絲頭、其他馬達）

| 連桿 | 最大重疊 (mm³) | 位置 |
|---|---|---|
| base | 0.0 | — |
| left_thigh | 0.0 | — |
| right_thigh | 0.0 | — |
| left_shin | 0.0 | — |
| right_shin | 0.0 | — |
| left_hip | 0.0 | — |
| right_hip | 0.0 | — |
| left_foot | 0.0 | — |
| right_foot | 0.0 | — |
| tail_yaw_link | 0.0 | — |
| tail_pitch_link | 0.0 | — |

## 3. 不相鄰的連桿（零位與站姿，所有連桿兩兩比對）

| 姿態 | 最大重疊 (mm³) | 位置 |
|---|---|---|
| 零位 | 0.0 | — |
| 站姿 | 0.0 | — |

## 4. 左右腿互撞（兩腳同時內收，參考用，不計入問題數）

| 兩側 hip_roll 內收 (deg) | 最大重疊 (mm³) | 位置 |
|---|---|---|
| 0 | 0.0 | — |
| 1 | 1112.7 | foot_left × foot_right |
| 2 | 3796.3 | foot_left × foot_right |
| 3 | 6020.5 | foot_left × foot_right |
| 6 | 7610.4 | foot_left × foot_right |

## 5. 電子部件（佔位方塊）

| 部件 | 對象 | 最大重疊 (mm³) |
|---|---|---|
| battery | — | 0.0 |
| board | — | 0.0 |
| ttl | — | 0.0 |
| bec | — | 0.0 |
| imu | — | 0.0 |
