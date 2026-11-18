# tools：開發工具（兩次實驗共用）

| 檔案 | 用途 |
|---|---|
| `isaacpy.cmd` | 用 Isaac Lab 的 Python 環境執行腳本（包裝 `C:\isaaclab\isaaclab.bat -p`）。系統沒有可用的 `python`，所有 Python 都透過它執行。可以用 `ISAACLAB_PATH` 環境變數覆寫安裝路徑。 |
| `archive_videos.py` | 依 `docs/videos/manifest.csv` 把 `outputs/` 中的重要影片壓縮（H.264 CRF 28，約原檔 1/8）複製到 `docs/videos/`（git LFS）；已存在的檔案不覆蓋。`tools\isaacpy.cmd tools\archive_videos.py [--dry_run]` |
| `urdf_inspect.py` | 檢查 URDF：關節軸與範圍、各 link 質量、總重、鏈長；對 mesh 碰撞體、預設的 effort/velocity、不合理的慣量發出警告（有警告時 exit code 為 1）。只使用標準函式庫。 |

```powershell
tools\isaacpy.cmd tools\urdf_inspect.py <file.urdf>
tools\isaacpy.cmd -c "import torch; print(torch.cuda.is_available())"
```

每次執行都會印出 `setup_conda_env.bat is missing` 的警告，這是無害的。
