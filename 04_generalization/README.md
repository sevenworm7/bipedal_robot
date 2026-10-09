# 04_generalization：泛化與壓力測試（實驗二）

**用途**：對 `03_training` 產出的策略（學生）做**訓練沒有涵蓋**的測試，找出失效邊界與適用範圍。這一步**不訓練**任何模型。
（2026-10-09 使用者決定的分工：01 機器人模型、02 老師、03 產生學生並做訓練範圍內的驗收、04 訓練範圍以外的測試。）

| 檔案 | 內容 |
|---|---|
| `stress_test.py` | 壓力測試批次：統計（每個條件 100 episode × 20 s）、影片（每個條件一段中速 10 秒片段）、報告（表格＋失效邊界圖＋影片連結）；可中斷續跑 |
| `CLAUDE.md` | 這一步的規則 |

評估、錄影與成功判定直接使用 `03_training/sim2real/` 的 `evaluate.py`、`record_videos.py`、`scoring.py`；測試條件（場景）定義在 `shared/env/scenarios.py` 的 `STRESS_V1`。

## 執行方式（專案根目錄）

```
tools\isaacpy.cmd 04_generalization\stress_test.py --out outputs\robot_v1\04_generalization\stress_v1_<日期> --stage stats   [--only 01_ 06_]
tools\isaacpy.cmd 04_generalization\stress_test.py --out outputs\robot_v1\04_generalization\stress_v1_<日期> --stage videos  [--only …]
tools\isaacpy.cmd 04_generalization\stress_test.py --out outputs\robot_v1\04_generalization\stress_v1_<日期> --stage report
```
`--agent student:<student.pt>` 可換成其他模型（預設為 03 的最終模型 student v5）。產出放在 `outputs/robot_v1/04_generalization/`。

## 壓力測試 v1（2026-10-09）
- **對象**：最終模型 student v5（`outputs/robot_v1/03_training/sim2real/student_v5_20261007_003835/student.pt`）。
- **方法**：一次只改一個條件，其餘為理想平地（`flat_v1`）；每個條件 100 episode × 20 s、seed 42；成功判定同 sim-to-real 準則。
  56 個條件定義在 `shared/env/scenarios.py` 的 `STRESS_V1`（名稱 `stress_v1/<條件>`，只用於評估、用過即凍結）：
  伺服延遲 0–100 ms、凸起高度 0–16 mm、凸起間距 5／10／20 cm、摩擦 0.2–1.5、馬達力量 50–100%、斜坡上下 3／5／8°（訓練未見過）、
  台階上下 5／10 mm（訓練未見過）、整機質量 ×1.0–1.5、推擠 0.1–0.45 m/s、IMU 雜訊 ×1–3、IMU 方向偏差 ±2–10°、速度指令 0.18／0.22 m/s、步頻 1.0／2.3 Hz。
- **新增程式**：
  - `shared/env/terrain.py` 的 `x_profile`：只沿前進方向變化的斜坡與台階（從地塊中央出發，0.3 m 平地後開始；台階邊緣為垂直面），起點在世界高度 0。
  - 地形設定新增選填項目（不填時行為不變）：`profile`、`bump_spacing`、`exact`（凸起高度固定為指定值）、`slope_threshold`、`spawn_x`、`spawn_z_abs`。
  - `stress_test.py`（本資料夾）：批次執行（統計、影片、報告三個階段，可中斷續跑）；`03_training/sim2real/evaluate.py` 新增 `--push_only`／`--push_speed`／`--out_dir`；
    `03_training/sim2real/record_videos.py` 新增 `--episode_s`／`--quality`／`--speeds`／`--freq`／`--keep_all`／`--push_speed`；影片編碼品質改為參數（預設 8，02／03 不變）。
- **影片**：每個條件只保留中速（0.10 m/s）的 10 秒片段（使用者 2026-10-09 決定），4 台錄製、quality 5（每段約 2.7 MB；quality 8 約 14.6 MB）。
- 開發時發現：斜坡／台階原本在中央後 1 m 才開始，慢速機器人 20 秒只走 1 m、碰不到；改為 0.3 m 並直接指定起點高度（站立時 base CoM 0.2435 m，與平地相同）。
- 回歸檢查（`regress7`）：`flat_v1` 仍與參考值完全一致。

- 結果與影片：`outputs/robot_v1/04_generalization/stress_v1_20261009/README.md`（統計已完成；影片與報告進行中）。
