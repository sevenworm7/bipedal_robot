# archive/exp1_reference：實驗一（管線測試）

使用 `references/` 中的參考機器人（髖關節位置修正版），略過建模，直接測試 **rule-based → 訓練管線** 是否可行。
測試通過後，再把驗證過的程式碼移植到 `01_`、`02_`、`03_`（實驗二）。規則與決策紀錄見 `CLAUDE.md`。

## 進度

| 階段 | 內容 | 狀態 |
|---|---|---|
| A. 機器人接入模擬器 | URDF → USD、方向/高度修正、80 N·m 致動器、站立測試 | ✅ 完成（2026-09-18） |
| B. 共用介面 | 觀測/動作定義、Isaac Lab 環境（機器人並排成列、朝同一方向）、步數/距離指標、影片 HUD | ✅ 完成（2026-09-18），定義已經使用者確認；2026-09-19 觀測加入 `gait_freq`（38 維）、修正步數計數 |
| C. Rule-based 控制器 | `rule_based/` | ✅ 完成（2026-09-19），成功率 95%、跌倒 0%，使用者已確認 |
| D. 訓練管線 | `training_pipeline/` | ✅ 完成（2026-09-23）：三輪自我蒸餾，最終模型成功率 99%、跌倒 0%、推擠 100%，待使用者檢視 |

## 資料夾

| 路徑 | 用途 |
|---|---|
| `CLAUDE.md` | 實驗一的規則、已確認的決策、參考模型的處理方式 |
| `shared/` | 本實驗共用的程式：介面契約、環境、指標、影片 HUD、機器人設定 |
| `scripts/` | 可直接執行的工具腳本（轉檔、檢查、煙霧測試） |
| `rule_based/` | 規則式控制器（規格：`rule_based/PROMPT.md`） |
| `training_pipeline/` | 訓練管線（規格：`training_pipeline/PROMPT.md`） |
| `outputs/` | 所有產出（報告、截圖、影片、log），不進 git |

## 從零重建環境（在專案根目錄執行）

```powershell
tools\isaacpy.cmd archive\exp1_reference\scripts\convert_ref_biped.py                       # 1. URDF -> USD（約 10 秒）
tools\isaacpy.cmd archive\exp1_reference\scripts\check_ref_biped.py --enable_cameras        # 2. 關節方向、站立測試、截圖
tools\isaacpy.cmd archive\exp1_reference\scripts\smoke_env.py --num_envs 1024               # 3. 環境煙霧測試
tools\isaacpy.cmd archive\exp1_reference\scripts\smoke_env.py --num_envs 4 --video --enable_cameras   # 4. 影片 HUD 測試
```
rule-based 的評估與錄影指令見 `rule_based/README.md`；訓練管線每一輪的執行順序見 `training_pipeline/README.md`。
三輪的比較表、管線需修正處、參考模型的限制與實驗二建議：`outputs/summary.md`（不進 git，可由上述指令重新產生）。

## 階段 A、B 的驗證結果

- 站立測試（微蹲姿勢，5 秒）：base 高度變化 0.1 mm、最大傾斜 1.65°、最大扭矩 3.5 N·m → 通過
- 關節正方向（+0.3 rad）：髖 pitch 腳往後、膝蓋小腿往後（和人類一樣）、髖 roll 腳往右、踝 pitch 腳尖往下
- 環境：觀測 38 維（2026-09-19 加入 gait_freq，原為 37 維）、動作 8 維、無 NaN；腳掌接觸力總和 / 體重 = 1.00；站立不動時跌倒率 0%
- 吞吐量（RTX 5080 Laptop，headless）：256 個環境約 9,100 env-steps/s；4096 個環境約 95,600 env-steps/s
- 影片：1280×720、30 fps，HUD 每一幀都會更新，相機跟隨機器人

## 過程中發現的問題（Isaac Lab 3.0 / Isaac Sim 6）

1. 參考 URDF 所在路徑含中文，因此先複製到 ASCII 路徑再轉檔。
2. 新版 URDF 轉換器會把所有 link 巢狀排列，造成：
   - `fix_root_link=True` 會產生兩個 ArticulationRoot → 改成每一步把 base 寫回原位來模擬固定底座
   - `activate_contact_sensors` 只作用在最外層的剛體 → 在 env 中逐一補上 contact report API
   - 同一個 regex 感測器只抓得到一隻腳 → 改成每隻腳各一個感測器
3. 內建 viewer 的跟隨模式取景不準 → 改用獨立的 Camera 感測器錄影。
4. `ContactSensor` 是延遲更新的，`compute_first_contact` 會漏掉大部分著地 → 環境自行追蹤離地→著地轉換（2026-09-19，見 `shared/env/README.md`）。
