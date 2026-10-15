# evaluation/generalization：泛化與壓力測試（實驗二）

**用途**：對 `control` 產出的策略（學生）做**訓練沒有涵蓋**的測試，找出失效邊界與適用範圍。這一步**不訓練**任何模型。
原 `04_generalization/`（2026-10-09 建立），2026-10-15 隨專案分層重整為 `evaluation/generalization/`；產生模型與訓練範圍內的驗收在 `control/`。

| 檔案 | 內容 |
|---|---|
| `stress_test.py` | 壓力測試批次：統計（每個條件 100 episode × 20 s）、影片（每個條件一段中速 10 秒片段）、報告（表格＋失效邊界圖＋影片連結）；可中斷續跑 |
| `CLAUDE.md` | 這一步的規則 |

評估、錄影與成功判定直接使用 `control/sim2real/` 的 `evaluate.py`、`record_videos.py`、`scoring.py`；測試條件（場景）定義在 `simulation/env/scenarios.py` 的 `STRESS_V1`。

## 執行方式（專案根目錄）

```
tools\isaacpy.cmd evaluation\generalization\stress_test.py --out outputs\robot_v1\evaluation\generalization\stress_v1_<日期> --stage stats   [--only 01_ 06_]
tools\isaacpy.cmd evaluation\generalization\stress_test.py --out outputs\robot_v1\evaluation\generalization\stress_v1_<日期> --stage videos  [--only …]
tools\isaacpy.cmd evaluation\generalization\stress_test.py --out outputs\robot_v1\evaluation\generalization\stress_v1_<日期> --stage report
```
`--suite v2` 執行 v2（斜坡／台階，修正跌倒判定），`--suite v3` 執行 v3（實機誤差：零點、IMU 傾斜、指令遺失）。`--agent student:<student.pt>` 可換成其他模型（預設為 `control/sim2real` 的最終模型 student v5）。產出放在 `outputs/robot_v1/evaluation/generalization/`。

## 壓力測試 v1（2026-10-09）
- **對象**：最終模型 student v5（`outputs/robot_v1/control/sim2real/student_v5_20261007_003835/student.pt`）。
- **方法**：一次只改一個條件，其餘為理想平地（`flat_v1`）；每個條件 100 episode × 20 s、seed 42；成功判定同 sim-to-real 準則。
  56 個條件定義在 `simulation/env/scenarios.py` 的 `STRESS_V1`（名稱 `stress_v1/<條件>`，只用於評估、用過即凍結）：
  伺服延遲 0–100 ms、凸起高度 0–16 mm、凸起間距 5／10／20 cm、摩擦 0.2–1.5、馬達力量 50–100%、斜坡上下 3／5／8°（訓練未見過）、
  台階上下 5／10 mm（訓練未見過）、整機質量 ×1.0–1.5、推擠 0.1–0.45 m/s、IMU 雜訊 ×1–3、IMU 方向偏差 ±2–10°、速度指令 0.18／0.22 m/s、步頻 1.0／2.3 Hz。
- **新增程式**：
  - `simulation/env/terrain.py` 的 `x_profile`：只沿前進方向變化的斜坡與台階（從地塊中央出發，0.3 m 平地後開始；台階邊緣為垂直面），起點在世界高度 0。
  - 地形設定新增選填項目（不填時行為不變）：`profile`、`bump_spacing`、`exact`（凸起高度固定為指定值）、`slope_threshold`、`spawn_x`、`spawn_z_abs`。
  - `stress_test.py`（本資料夾）：批次執行（統計、影片、報告三個階段，可中斷續跑）；`control/sim2real/evaluate.py` 新增 `--push_only`／`--push_speed`／`--out_dir`；
    `control/sim2real/record_videos.py` 新增 `--episode_s`／`--quality`／`--speeds`／`--freq`／`--keep_all`／`--push_speed`；影片編碼品質改為參數（預設 8，02／03 不變）。
- **影片**：每個條件只保留中速（0.10 m/s）的 10 秒片段（使用者 2026-10-09 決定），4 台錄製、quality 5（每段約 2.7 MB；quality 8 約 14.6 MB）。
- 開發時發現：斜坡／台階原本在中央後 1 m 才開始，慢速機器人 20 秒只走 1 m、碰不到；改為 0.3 m 並直接指定起點高度（站立時 base CoM 0.2435 m，與平地相同）。
- 回歸檢查：`regress7`（新增地形功能後）與 `regress8`（2026-10-09 資料夾重整後）`flat_v1` 都與參考值完全一致；重整後 `flat_pipeline` 評估（99%／1%）與 rule-based `check_ik` 也正常執行。

- 結果與影片（56 個條件，每個都有中速影片）：`outputs/robot_v1/evaluation/generalization/stress_v1_20261009/README.md`。**斜坡（06）與台階（07）以 v2 為準**。
- 錄影時發現 `simulation/eval_videos.py` 的問題：`--keep_all` 下同一段影片可能同時被選為「slow」與「failure」，搬移兩次後程式中止、中速影片遺失（5 個條件），已修正並重錄。

## 壓力測試 v2（2026-10-09）：修正斜坡與台階的跌倒判定

- **發現**：v1 下坡 −3° 跌倒 57%、−5° 95%，但中速影片前 10 秒都沒有跌倒；跌倒位置集中在**累積下降約 9–10 cm** 處
  （−3° 在 2.00 m、−5° 在 1.35 m、下台階 10 mm 在第 9–10 階），正好等於站姿 CoM 0.243 m − 跌倒門檻 0.146 m = 0.097 m。
  原因是環境的跌倒高度以**固定的出生點高度**為基準（`biped_env.py` 的 `com_pos`）：往下走就被誤判為跌倒，往上走則可能漏判。訓練只用平地與 ±8 mm 凸起，不受影響。
- **修正**：新增 `fall_height_ref="feet"`（預設關閉，`flat_v1` 回歸檢查 `regress9` 完全一致）：以較低那隻腳的 foot body 高度 − 0.026 m（站立時實測 0.025–0.029 m）估計腳下地面。
  走路時「CoM − 較低的腳」最低 0.201 m，離門檻（0.146 − 0.026 = 0.120 m）有足夠餘裕。
- **`STRESS_V2`**：v1 的 10 個斜坡／台階條件＋平地對照 `02_bumps_00mm`，只改跌倒判定；v1 凍結不動，報告加註以 v2 為準。
  執行：`tools\isaacpy.cmd evaluation\generalization\stress_test.py --suite v2 --out outputs\robot_v1\evaluation\generalization\stress_v2_20261009 --stage <stats|videos|report>`
- 結果：`outputs/robot_v1/evaluation/generalization/stress_v2_20261009/README.md`（平地對照與 v1 完全相同）。

| 條件 | v1（判定錯誤） | **v2** | 說明 |
|---|---|---|---|
| 下坡 −3° | 41%／跌倒 57% | **97%／1%** | |
| 下坡 −5° | 0%／95% | **79%／1%** | 失敗是速度過快（p50 1.21 倍），不是跌倒 |
| 下坡 −8° | 0%／100% | **9%／1%** | 速度 1.36 倍超出 ±25%；不會跌倒，但無法控制下坡速度 |
| 下台階 10 mm | 58%／40% | **95%／1%** | |
| 上台階 10 mm | 40%／20% | 40%／20%（不變） | 真正的跌倒，方向誤差 p50 11° |
| 上坡 3／5／8° | 99／99／80% | 不變 | 沒有漏判；8° 失敗是速度過慢（0.79 倍） |

## 壓力測試 v3（2026-10-15）：實機常見、但訓練沒有隨機化的誤差

- **目的**：決定實機組裝與校正要做到多精準（使用者 2026-10-15 同意）。對象同為 student v5，一次只改一個條件，其餘同 `flat_v1`。
- **新增環境功能**（`simulation/env`，預設關閉；`flat_v1` 回歸檢查 `regress11` 與 student v5 平地評估都完全一致）：
  - `joint_offset`：伺服零點偏差。每個關節固定偏移 ±值（正負號隨機），伺服回讀的角度少了這個偏移、走到的位置多了這個偏移，所以機器人的實際姿勢和策略以為的不同。
  - `imu_tilt`：IMU 安裝傾斜。重力方向與角速度改在傾斜的座標系中量測（傾斜方向隨機）。
  - `action_drop_prob`：伺服指令遺失。每個控制步以該機率沿用上一個目標（匯流排寫入失敗）；策略仍以為新指令已送出。
- 結果：`outputs/robot_v1/evaluation/generalization/stress_v3_20261015/README.md`（16 個條件、每個都有中速影片）。

| 條件 | 0（對照） | 1° | 2° | 3° | 5° | 8° | 12° |
|---|---|---|---|---|---|---|---|
| 關節零點偏差（每個關節 ±） | 100%／0% | 100%／0% | **100%／0%** | 93%／2% | **39%／跌倒 27%** | — | — |
| IMU 安裝傾斜 | 100%／0% | — | 100%／0% | — | 100%／0% | **99%／0%** | 49%／跌倒 13% |

| 條件 | 0 | 5% | 10% | 20% | 40% |
|---|---|---|---|---|---|
| 伺服指令遺失率 | 100%／0% | 100%／0% | 100%／0% | 100%／0% | **100%／0%**（方向誤差 p50 1.5° → 3.8°） |

- 組合（零點 2°＋IMU 3°＋遺失 5%）：**99%／跌倒 1%**，側向漂移 p50 0.10 → 0.29 m。
- **結論（實機要求）**：
  1. **關節零點要校正到每顆 ±2° 以內**：2° 不影響成功率（但側向漂移 0.10 → 0.25 m），3° 開始掉（93%），5° 跌倒 27%。HD-1910 一圈 4096 格（0.088°／格），軟體校正可以做到；
     擺臂花鍵裝偏一齒的角度遠大於 2°，所以**組裝後一定要做軟體零點校正**，不能只靠機構對位。
  2. **IMU 安裝傾斜容許到約 8°**，一般機構固定（±2–3°）不需要額外校正。
  3. **偶爾遺失伺服指令幾乎沒有影響**（40% 仍 100%），匯流排偶發錯誤不是風險；真正的風險是**延遲**（v1：80 ms 起失效）。
- **一次性異常（已處理）**：第一次統計中 IMU 傾斜 5° 出現一半機器人原地不動（成功 40%），重跑兩次都是 100%，其他條件重跑與原紀錄完全相同，
  判定為該次執行受干擾（執行時間 41 s，其他約 30 s；可能是同時有其他模擬程式使用 GPU）。原結果移到 `runs_invalid/`（保留），以重跑結果為準。
  之後**模擬一次只跑一個**，結果不單調時先重跑確認可重現再下結論。

## 結論（2026-10-09）

student v5 在訓練範圍外仍有相當大的餘裕（各項 ≥ 95% 成功、跌倒 ≤ 1%）：
伺服延遲到 60 ms、摩擦 0.2–1.5、馬達力量 50%、質量 ×1.5、IMU 雜訊 ×3、方向偏差 ±10°、速度 0.22 m/s、步頻 1.0–2.3 Hz、上坡 5°、下坡 3°、上下 5 mm 台階、下 10 mm 台階。

失效邊界（實機要避開或下一版要加強的地方）：
1. **上台階 10 mm**（約擺動腳離地高度 18 mm 的一半）：跌倒 20%，是唯一明顯會跌倒的「未訓練地形」。
2. **凸起超過訓練範圍**：10 mm 跌倒 4%、12 mm 18%、16 mm 69%；凸起密集（間距 5 cm）時方向偏移增加（成功 71%）。
3. **延遲 ≥ 80 ms**：80 ms 成功 75%、100 ms 跌倒 48%。實機的伺服通訊延遲要量測並控制在 60 ms 以內。
4. **推擠 ≥ 0.35 m/s**：跌倒 12%（0.45 m/s 跌倒 36%）。
5. **下坡速度控制**：−5° 起速度超出指令 20% 以上（不跌倒），8° 上坡則變慢；策略觀測不到坡度，若實機要走坡道，下一版訓練應加入斜坡。
