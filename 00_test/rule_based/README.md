# 00_test/rule_based：規則式控制器（實驗一）

**狀態：完成（2026-09-19）。100 episode 成功率 95%（門檻 70%）、跌倒率 0%；使用者已檢視影片並確認，可以進入訓練階段。**

採用的參數：`params/cem2_best.json`（載入方式：`--params 00_test/rule_based/params/cem2_best.json`）。

- `PROMPT.md`、`CLAUDE.md`：從 `02_rule_based/` 複製過來的規格，測試過程中可以修改，原始版本保留在 02。
- 介面：從 `00_test/shared/interface.py` 匯入（觀測 38 維、動作 8 維、50 Hz），環境使用 `shared/env`。
- 驗收標準：100 個 episode 中成功率 ≥ 70%（未跌倒、至少 10 步、距離 ≥ 0.8 × 步數 × 步長），判定函式為 `shared/metrics.py` 的 `rule_based_success`。

## 檔案

| 檔案 | 內容 |
|---|---|
| `config.py` | `GaitParams`（所有可調參數與預設值）、`RuleBasedCfg`（每個 episode 隨機取值的範圍、IK 邊界） |
| `leg_kinematics.py` | 平面兩連桿 IK/FK（`SagittalLegIK`），腿長與關節極限從 robot 模組讀取 |
| `controller.py` | `RuleBasedBipedController`：`reset(env_ids)`、`compute_action(obs) -> action`，批次處理 N 個環境 |
| `check_ik.py` | 步驟 1：離線檢查 IK 與關節目標（不需要模擬器） |
| `sweep.py` | 步驟 2–4：平行執行並掃描參數（格點） |
| `diagnose.py` | 少量環境的時序診斷圖（腳高、接觸、姿態、關節目標與實際值）與影片 |
| `optimize.py` | 步驟 4：CEM 參數搜尋（指令、步頻全範圍隨機） |
| `evaluate.py` | 步驟 5：正式評估（100 episode）＋推擠測試，輸出 `stats.json` |
| `record_videos.py` | 步驟 6：評估影片（慢／中／快＋失敗案例），HUD 格式依 `/eval-video`；錄影邏輯在 `shared/eval_videos.py` |
| `params/` | CEM 找到的參數（`cem1_best.json` 第一輪、`cem2_best.json` 第二輪＝採用版本） |

## 控制器做法

只使用 38 維觀測與機器人的幾何常數；內部狀態只有：reset 後經過的時間（漸進啟動）、積分的側向速度（側向偏移估計）。步頻直接讀取觀測中的 `gait_freq`。

1. **步態時序**：觀測中的時鐘相位與步頻 `gait_freq`，左腳相位 = 時鐘，右腳 = 時鐘 + 0.5；每隻腳在 `swing_frac` 的時間內擺動。
2. **前後軌跡**（相對髖 pitch 軸）：支撐腳以身體速度往後移，行程 `D = cmd/f × (1 − swing_frac)`；擺動腳以餘弦曲線移到 Raibert 落腳點，高度為 sin 曲線。
3. **髖高**：每個環境只有一個髖高，取兩腿 `sqrt(r² − x²)` 的最小值（倒單擺式起伏），包含擺動腿，因此落地前髖就先降下，時間上連續。
4. **IK**：兩連桿 IK 算出髖 pitch 與膝，踝 pitch 讓腳掌與地面平行。
5. **側向搖擺**：左右 hip roll 同步轉動（平行四邊形），把骨盆移到支撐腳上方；波形為正弦，可調成接近方波（`sway_shape`）。
6. **平衡回授**：支撐腿髖 pitch/roll 姿態伺服、擺動腿依傾角補償、前後與側向 Raibert 落腳點、側向偏移修正、依方向誤差調整左右步長。

## 開發紀錄

### 步驟 1：離線驗證 IK（`check_ik.py`）
- FK(IK) 來回誤差 3e-8 m；預設站姿（髖 −0.25、膝 0.5）可以還原。
- **參考模型的幾何限制**：膝只能 ±45°、踝只能 ±20°，腿（大腿＋小腿）只有 0.19 m：
  - 髖到踝的距離只能在 0.177–0.190 m 之間變化 → 單靠膝蓋，抬腳高度約只有 1 cm。
  - 腳掌要保持水平時，小腿前傾不能超過 20° → 支撐末段腳跟必然抬起（類似人的蹬地）。
  - 指令角落 `cmd 0.2 m/s、f 0.8 Hz` 需要 12.5 cm 的步長，腿搆不到（差約 15 mm）。

### 步驟 2：原地踏步（`sweep.py --cmd 0`、`diagnose.py`）
- 只縮短擺動腿時，腳不會離地：重心在兩腳中間，身體直接往擺動側倒，腳又被壓回地面。
- 兩個 hip roll 同步轉動可以讓身體側向移動 ±10 cm（兩腳一起翻到腳掌邊緣）。掃描相位後，原地踏步可以乾淨地交替離地（每次 0.15–0.25 s）。
- **發現並修正環境的步數計數 bug**（見 `shared/env/README.md`）：`compute_first_contact` 漏掉大部分著地。修正後以影片逐格確認 HUD 步數與實際離地一致。

### 步驟 3：固定參數往前走
- 0.1 m/s、1 Hz：576 個環境全部沒有跌倒，但 20 秒最多只前進 0.42 m（應為 2 m）；擺動腳大部分時間在地上拖，前後速度來回擺盪 ±0.4 m/s。
- 格點掃描（432 組 × 8 環境）最好的是 20 秒 1.05 m。

### 步驟 4：參數搜尋（CEM）
- `optimize.py`：19 個參數，每代 256 組 × 12 環境，指令與步頻全範圍隨機；最後前 8 名各用 256 個環境重新評估。
- **第一輪**（30 代，目標＝成功率＋前進量－抖動－跌倒）：成功率 89%，但速度是指令的 1.5 倍、側向偏移中位數 0.37 m、方向誤差 25°。
- **第二輪**（25 代，從第一輪出發，目標加入速度誤差、最大側向偏移、最大方向誤差的懲罰，以蒸餾篩選門檻 0.15 m／10°／25% 正規化）：
  成功率 95%、跌倒 0%、側向偏移 0.15 m → 採用。
- 期間把觀測從 37 維改為 38 維（加入 `gait_freq`，2026-09-19 使用者同意），控制器改為直接讀取步頻。

### 步驟 5：正式評估（`evaluate.py`，100 episode × 20 s，指令 0.05–0.20 m/s、步頻 0.8–1.2 Hz、初始關節雜訊 ±0.05 rad 皆隨機）

| 指標 | 第一輪 `cem1_best` | **第二輪 `cem2_best`（採用）** |
|---|---|---|
| 成功率（門檻 70%） | 89% | **95%** |
| 跌倒率 | 2% | **0%** |
| 平均行走距離（20 s） | 3.14 m | 2.42 m |
| 平均速度／平均指令 | 0.158／0.119 m/s | **0.121／0.119 m/s** |
| 速度比（p10／中位數／p90） | 1.07／1.52／1.73 | **0.89／1.02／1.23** |
| 最大側向偏移（中位數／p90） | 0.37／0.95 m | **0.136／0.228 m** |
| 最大方向誤差（中位數／p90） | 25.5°／50.4° | **7.2°／13.7°** |
| 推擠測試成功率／跌倒率（0.3 m/s，t = 5、10、15 s，僅供參考） | 83%／11% | **88%／0%** |

第二輪各區間成功率：指令 0.05–0.10：98%｜0.10–0.15：94%｜0.15–0.20：93%；步頻 0.8–0.93 Hz：83%｜0.93–1.07：100%｜1.07–1.2：100%。
每步實際著地數／時鐘應有步數 = 0.96（幾乎沒有抖動或漏步）。

**扭矩（N·m，第二輪，平均／p95／最大）**，作為實驗二選馬達的依據：

| 關節 | 平均 | p95 | 最大 |
|---|---|---|---|
| 髖 roll（左／右） | 11.0／11.4 | 28.3／28.7 | 63.7／65.4 |
| 髖 pitch | 4.6／5.1 | 11.9／12.5 | 32.8／48.0 |
| 膝 | 4.2／4.3 | 10.3／10.6 | 49.0／35.5 |
| 踝 pitch | 4.1／4.3 | 12.3／13.1 | 53.4／44.4 |

扭矩飽和（達 80 N·m）的時間比例 0%。髖 roll 的 p95 約 28 N·m，和靜態估算（25–36 N·m）一致；最大值來自著地衝擊。

### 步驟 6：評估影片（`record_videos.py`）
16 台並排（慢 0.07、中 0.125、快 0.18 m/s 各 4 台，其餘 4 台隨機），步頻 0.9 Hz，每台各一個跟拍鏡頭，HUD 依 `/eval-video`。
產出在 `00_test/outputs/rule_based/videos_rule_based_20260919_162742/videos/`：

| 影片 | 內容 |
|---|---|
| `exp1_rule_based_slow_eval_20260919.mp4` | 0.07 m/s，20 s 約 34 步、1.4 m |
| `exp1_rule_based_medium_eval_20260919.mp4` | 0.125 m/s，約 34 步、2.5 m |
| `exp1_rule_based_fast_eval_20260919.mp4` | 0.18 m/s，約 41 步、3.3 m |
| `exp1_rule_based_failure_eval_20260919.mp4` | 未通過判定的案例（快速，沒有跌倒，但步數比時鐘多 → 距離門檻 0.8 × 步數 × 步長沒達到） |

run 資料夾中的 `videos.json` 記錄 16 台的步數、距離、側向偏移與是否成功。正式統計在 `eval_cem2_20260919_154611/stats.json`。

## 已知限制
- **只在平地、直線行走驗證**；沒有 hip yaw，無法主動轉向，方向只能靠左右步長差修正（最大方向誤差中位數 7°）。
- **側向搖擺很大**：沒有 ankle roll，只能用兩個 hip roll 同步轉動讓身體左右擺（腳掌會翻到邊緣），步態像企鵝搖擺；hip roll 使用到接近 ±20° 的極限。
- **低步頻較弱**：0.8–0.93 Hz 成功率 83%（其餘 100%）。
- **幾何角落**：`cmd 0.2 m/s、f 0.8 Hz` 需要 12.5 cm 步長，超出腿的可達範圍（IK 會截斷）。
- 抬腳高度只有約 1.5 cm（膝 ±45°、踝 ±20° 的限制），無法跨越障礙。
- 參數是 CEM 針對目前的物理設定（摩擦 1.0、kp 300、kd 10）找到的，改變這些設定需要重新搜尋。
- 控制器內部狀態：reset 後經過的時間（前 1.5 個週期漸進啟動）與積分的側向速度；BC 學生看不到這兩個量（影響應該很小：前者只影響開頭 1–2 秒）。

## 執行方式（在專案根目錄）

```powershell
tools\isaacpy.cmd 00_test\rule_based\check_ik.py
tools\isaacpy.cmd 00_test\rule_based\sweep.py --tag march --cmd 0 --freq 1.0 --episode_s 10
tools\isaacpy.cmd 00_test\rule_based\diagnose.py --tag walk --cmd 0.1 --seconds 6 --video --enable_cameras
tools\isaacpy.cmd 00_test\rule_based\optimize.py --tag cem2 --init 00_test\rule_based\params\cem1_best.json --init_std 0.12 --gens 25 --episode_s 20 --prog_cap 1.0 --w_speed 1.0 --w_lat 0.3 --w_heading 0.3
tools\isaacpy.cmd 00_test\rule_based\evaluate.py --tag eval_cem2 --params 00_test\rule_based\params\cem2_best.json --episodes 100 --push
tools\isaacpy.cmd 00_test\rule_based\record_videos.py --params 00_test\rule_based\params\cem2_best.json --num_envs 16 --freq 0.9 --enable_cameras
```
產出都在 `00_test/outputs/rule_based/`。
