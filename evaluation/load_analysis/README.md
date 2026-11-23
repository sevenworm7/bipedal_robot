# evaluation/load_analysis：馬達負載分析（模擬，不訓練）

**用途**：在實機之前，估計每顆伺服在各種情境下要出多少力、會不會過熱、整機要多大的電流（使用者 2026-10-15 提出）。
結果用來決定電池與電源的規格、找出最吃力的關節，也是之後實機台架測試（`deploy/actuator_bench/`）的比對基準。

| 檔案 | 內容 |
|---|---|
| `cases.py` | 情境定義 `CASES_V1`（既有場景＋固定指令＋推擠）與 HD-1910 的 6 V 規格；用過即凍結，要改就新增 `CASES_V2` |
| `record_load.py` | 一個情境一個 Isaac Lab 行程：`--mode stats` 記錄 128 台 × 20 s 的扭矩、關節速度、著地；`--mode video` 錄 10 秒片段，HUD 標註每顆馬達的即時出力 |
| `run_load.py` | 批次執行所有情境（可中斷續跑）並產生報告：總覽表、各關節 RMS／p99 熱度圖、扭矩時序圖（標示著地）、整機電流 |

## 執行方式（專案根目錄）

```
tools\isaacpy.cmd evaluation\load_analysis\run_load.py --out outputs\robot_v1\evaluation\load_analysis\load_v1_<日期> --stage stats
tools\isaacpy.cmd evaluation\load_analysis\run_load.py --out outputs\robot_v1\evaluation\load_analysis\load_v1_<日期> --stage videos
tools\isaacpy.cmd evaluation\load_analysis\run_load.py --out outputs\robot_v1\evaluation\load_analysis\load_v1_<日期> --stage report
```

影片 HUD（`simulation/video_overlay.py` 的 `joint_load`，預設關閉，其他影片不受影響）：每顆馬達一條橫條，
綠色＝低於額定 0.294 N·m、黃色＝超過額定、紅色＝達到模擬上限 0.8 N·m；黃線、紅線分別標示額定與上限，條的全長是堵轉 1.18 N·m；最下方是整機電流估計。

## 負載分析 v1（2026-10-15）

- 對象：student v5。12 個情境，每個 128 台 × 20 s、同一個指令，統計每台的第一個 episode（略過前 0.5 s）。
- 結果與影片：`outputs/robot_v1/evaluation/load_analysis/load_v1_20261015/README.md`。

| 情境 | 最吃力的關節（RMS，佔額定） | 超過額定的時間（最高關節） | 整機電流 平均／p99／最大（估計） |
|---|---|---|---|
| 靜止站立 | 左 hip pitch 0.19（65%）；另見下方「雙腳卡住」 | 5%（卡住的機器人） | 2.9／4.3／4.6 A |
| 原地踏步 | 左 knee 0.15（52%） | 4% | 3.0／4.1／5.1 A |
| 平地慢／中／快（0.06／0.10／0.14 m/s） | hip pitch 0.15／0.18／0.20（51／60／69%） | 2／3／15% | 3.1–3.3／4.1–4.5／5.5–5.9 A |
| 訓練範圍最高指令（0.15 m/s、2 Hz） | hip pitch 0.21（70%） | 8% | 3.3／4.8／5.6 A |
| ±8 mm 凹凸 | hip pitch 0.17（59%） | 3% | 3.2／4.3／**8.8** A |
| 推擠 0.25 m/s | hip pitch 0.17（59%） | 3% | 3.1／4.4／6.4 A |
| 上坡 5°／上台階 10 mm | hip pitch 0.17／0.18（57／60%） | 2／5% | 3.1／4.0–4.6／5.9–6.6 A |
| **整機質量 ×1.5** | hip pitch **0.23（79%）** | **33%** | 3.7／4.9／5.8 A |
| 中等隨機化（延遲、馬達參數） | knee 0.21（71%） | 12% | 3.3／5.0／7.9 A |

**結論**

1. **走路時的發熱有餘裕，但不大**：各關節 RMS 都在額定的 50–70%，最吃力的是 **hip pitch**（支撐期持續約 0.25 N·m，接近額定 0.294）。
   機器人如果比設計重 50%，hip pitch 的 RMS 到 79%、三分之一的時間超過額定，長時間行走可能過熱。**實機的總重要控制在設計值（0.69 kg）附近**，列印件秤重（`docs/backlog.md` #3）很重要。
2. **峰值來自著地衝擊**：knee 與 ankle 在每次著地有 0.3–0.5 N·m、不到 0.05 s 的尖峰（見時序圖），平地 p99 約 0.4–0.5 N·m，不到堵轉的一半；短時間尖峰對伺服沒有問題。
3. **電源規格**：整機平均約 3 A、p99 約 4.5–5 A、最大 8.8 A（凹凸地面）。估計方式偏保守（每顆都算 0.18 A 空載電流）。
   建議 **2S 鋰電、可持續放電 ≥ 10 A**（例如 1000 mAh 25C 以上），伺服直接接電池；配線與接頭按 10 A 峰值設計。約 3 A 平均時 1000 mAh 可走約 15–20 分鐘。
4. **新發現：站立時雙腳可能卡住**：128 台站立的機器人中有 7 台（5%）的右 hip roll **整段 20 秒都頂在 0.8 N·m 上限**。原因是兩腳腳掌互相卡住
   （兩腳 foot body 間距只有 48 mm，正常 64 mm；hip roll 往內轉 0.09 rad 後推不回來），起因是初始關節角的隨機誤差。走路時會在第一步分開，只有站著不動時才會卡住。
   實機上如果發生，**該伺服會持續堵轉發熱**。對策：開機站起的程序要把雙腳放在正確的寬度；之後的版本加大兩腳間距（`docs/backlog.md` #1，v2 尚未處理）。

**限制**：模擬的扭矩來自 PD 模型（kp 8、kd 0.3），真實伺服的內部控制器更硬，著地尖峰可能更大；電流是由扭矩線性估計。
這些數字要在實機台架測試時用伺服回讀的電流與負載校正。

## 負載分析 robot v2（2026-11-19）

- 對象 student v2ft，情境同 `CASES_V1`。`run_load.py` 2026-11-19 新增 `--agent`（不填時仍是 v1 student v5），報告標題與模型路徑取自它：
  ```
  set CUSTOM_BIPED_VERSION=2
  tools\isaacpy.cmd evaluation\load_analysis\run_load.py --out outputs\robot_v2\evaluation\load_analysis\load_v1_20261119 --stage stats --agent student:<student.pt>
  ```
- 結果與 v1 對照：[`docs/results.md`](../../docs/results.md)「v2 泛化、壓力測試與馬達負載」。走路時 knee、ankle 變輕、hip pitch 約重 10%（最高指令 RMS 82%），整機電流同級（平均約 3.1 A、最大 7.4 A）。
- **站立時兩腳互頂**（128 台全部）：左 hip roll 持續 0.41 N·m（額定 141%）。實機改用固定站姿，要走時才交給模型（`docs/decisions.md` #24）。

| 檔案 | 內容 |
|---|---|
| `stand_handover.py` | 部署檢查：固定站姿（action 0）站 `--stand_s` 秒，再給隨機行走指令交給模型走 `--walk_s` 秒；印出站立時的扭矩、腳距與交接後的成功率 |

```
set CUSTOM_BIPED_VERSION=2
tools\isaacpy.cmd evaluation\load_analysis\stand_handover.py --agent student:<student.pt> --scenario flat_v1
```
結果（2026-11-19，128 台）：`flat_v1` 與 `dr_v1` 都是站立 0 跌倒、交接後 100%／0%，站立時 hip roll RMS 0.06–0.09 N·m、腳距 p50 59–60 mm。

## 負載分析 v2 案例：慢步態（2026-11-23）

- `cases.py` 新增 `CASES_V2`（0.05 m/s，步頻 1.6／1.2／1.0／0.8 Hz，平地與 `dr_v1`）；`run_load.py` 用 `--cases v2` 選擇（預設 v1，舊結果不受影響）。
  ```
  tools\isaacpy.cmd evaluation\load_analysis\run_load.py --cases v2 --out outputs\robot_v2\evaluation\load_analysis\load_v2_20261123 --stage stats --agent student:<student.pt>
  ```
- 結果：膝、髖 pitch 速度 p95 與整機電流幾乎不隨步頻改變，見 [`docs/results.md`](../../docs/results.md)「v2 慢步態評估」。
