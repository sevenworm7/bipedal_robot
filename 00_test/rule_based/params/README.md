# rule_based/params：步態參數檔

`optimize.py`（CEM）找到的 `GaitParams` 數值。任何評估、錄影或資料收集腳本都可以用 `--params <檔案>` 載入，沒有列出的參數使用 `config.py` 的預設值。

| 檔案 | 來源 | 100 episode 評估 | 狀態 |
|---|---|---|---|
| `cem1_best.json` | 第一輪 CEM（目標：成功率＋前進量） | 成功率 89%，速度 1.5 倍、側向偏移大 | 保留比較用 |
| `cem2_best.json` | 第二輪 CEM（從第一輪出發，加入速度／側向／方向誤差懲罰） | 成功率 95%、跌倒 0% | **採用** |

搜尋過程與完整輸出在 `00_test/outputs/rule_based/cem1_*`、`cem2_*`（`history.json`、`final.json`、`objective.json`）。
