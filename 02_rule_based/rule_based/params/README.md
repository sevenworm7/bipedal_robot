# rule_based/params：步態參數檔（實驗二）

`optimize.py`（CEM）找到的 `GaitParams` 數值。評估、錄影、診斷與之後的資料收集都用 `--params <檔案>` 載入；
沒有列出的參數使用 `config.py` 的預設值（和機器人大小有關的由 robot 模組推導）。檔名開頭標示機器人版本。

| 檔案 | 來源 | 100 episode 評估 | 狀態 |
|---|---|---|---|
| `robot_v1_cem1_best.json` | 第一輪 CEM（24 參數、30 代，目標函數含速度／側向／方向誤差） | 成功率 100%、跌倒 0%、側向 0.019 m、方向 0.9° | **採用** |

搜尋過程與完整輸出在 `outputs/robot_v1/02_rule_based/cem1_20261001_000238/`（history.json、final.json、objective.json）。
