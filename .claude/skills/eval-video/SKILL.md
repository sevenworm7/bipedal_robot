---
name: eval-video
description: The shared evaluation-video spec (HUD overlay metrics, format, filename) used by the rule-based controller and every training iteration, in both experiments (00_test and 02/03). Use when recording, overlaying, or reviewing evaluation rollouts.
---

# 評估影片規格（兩次實驗、rule-based 與訓練共用）

每個實驗的實作只有一份，放在實驗根目錄的 `shared/`（例如 `00_test/shared/video_overlay.py`），rule-based 和訓練都呼叫這份，避免兩邊格式不一致。實驗二直接移植實驗一的版本。

## 疊加資訊（每個 frame 都要更新）
固定在左上角，使用等寬字體，白字搭配半透明黑底；在 720p 下仍需清楚可讀。

```
exp1 / iter2 / ppo t = 00:12.34
steps     18       dist   4.52 m
lateral  +0.08 m   v_x    0.41 m/s
status   WALKING   (FALLEN / TIMEOUT / DONE)
```

- `steps`：腳掌接觸狀態由離地轉為著地的次數（左右腳合計），使用接觸感測器判斷，不要用相位推算。騰空至少 0.03 s（`min_step_air_time`）才算一步；著地由環境自行追蹤離地→著地轉換，不使用 `compute_first_contact`（見 `/isaac-run` 陷阱清單）。
- `dist`：base 從起點沿指令方向的位移 [m]。
- `lateral`：垂直於指令方向的偏移 [m]，帶正負號；右下角另外畫出小型俯視軌跡。
- `t`：**模擬時間**（step 數 × dt），不是實際經過的時間；格式為 `MM:SS.cc`。
- `status`：跌倒時整個畫面加上紅色邊框。
- 版本標籤：前面加上實驗代號。rule-based 使用 `exp1 / rule_based`；訓練使用 `exp1 / iter<N> / <bc|ppo|distill>`。

## 格式
- mp4（H.264），至少 1280×720、30 fps。錄影 fps 與控制頻率不同時，依模擬時間取樣 frame，不可直接逐 step 寫入。
- 檔名：`<exp>_<tag>_eval_<YYYYMMDD>[_<k>].mp4`，其中 `<exp>` 為 `exp1` 或 `exp2`，例如 `exp1_iter3_ppo_eval_20260918.mp4`、`exp2_rule_based_eval_20261001_2.mp4`。
- 存放位置：`<實驗根目錄>/outputs/<rule_based|training>/<run>/videos/`（實驗根目錄：實驗一為 `00_test/`，實驗二為專案根目錄）。
- 兩次實驗的影片格式要完全相同，方便直接比較。

## 實作（實驗一已完成，實驗二直接移植）
- HUD 與錄影：`shared/video_overlay.py`（`HudState`、`EvalVideoRecorder`、`video_filename`），使用 PIL + Consolas 字型，並用 imageio（H.264）寫檔。
- 畫面：環境設定 `cfg.eval_camera=True`，並加上 `--enable_cameras`，再呼叫 `env.capture_frame(env_id)`（獨立的 Camera 感測器，位於機器人側前方，跟隨目標方向）。**不要使用 `env.render()` 或 viewer 的 asset_root 模式**，取景會跑掉。
- 指標：`env.tracker`（步數、距離、側向偏移）與 `env.base_lin_vel`，都和畫面來自同一個 step。
- 範例：`00_test/scripts/smoke_env.py --video`；多台並排、每台各一段影片：`00_test/rule_based/record_videos.py`。
- episode 結束時環境會自動 reset，結束畫面（定格 1 秒）要沿用最後一個還在走的 frame，不可在 reset 後重新拍攝。
- 錄影時只開少量環境（1–16 個）。
- 數值和畫面必須來自同一個 step，禁止用前一個 step 的數值。
- 錄完後抽查：用 `cv2.VideoCapture` 讀取第一個、中間和最後一個 frame 存成 png，用 Read 工具檢視文字是否清楚、數值是否合理。
