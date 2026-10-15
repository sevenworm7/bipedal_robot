# archive/exp1_reference/scripts：可執行的工具腳本

所有腳本都在**專案根目錄**執行，並透過 `tools\isaacpy.cmd` 呼叫。產出會寫到 `archive/exp1_reference/outputs/`。

| 腳本 | 用途 | 產出 |
|---|---|---|
| `convert_ref_biped.py` | 把參考 URDF 複製到 ASCII 路徑後轉成 USD（凸包碰撞體） | `shared/robots/usd/ref_biped/` |
| `check_ref_biped.py` | 機器人健全性檢查：零位幾何、各關節正方向、5 秒站立測試、截圖 | `outputs/robot_check/<時間>/report.json`、png |
| `smoke_env.py` | 環境煙霧測試：維度、NaN、接觸力、跌倒率、吞吐量；加上 `--video` 時測試 HUD 影片 | `outputs/smoke/<時間>/report.json`、mp4、png |

```powershell
tools\isaacpy.cmd archive\exp1_reference\scripts\convert_ref_biped.py
tools\isaacpy.cmd archive\exp1_reference\scripts\check_ref_biped.py --enable_cameras
tools\isaacpy.cmd archive\exp1_reference\scripts\smoke_env.py --num_envs 1024
tools\isaacpy.cmd archive\exp1_reference\scripts\smoke_env.py --num_envs 4 --video --enable_cameras
```

修改機器人設定（`shared/robots/ref_biped.py`）或環境後，都要重新執行 `check_ref_biped.py` 與 `smoke_env.py`。
