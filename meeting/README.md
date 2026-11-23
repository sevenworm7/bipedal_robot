# meeting/：會議報告

每次會議報告一個資料夾，命名 `<YYYYMMDD>_<主題>/`（日期是報告日期），只新增、不覆蓋舊報告。

每個報告資料夾的內容：
- `report.html`：報告本體，直接用瀏覽器開啟（單一檔案，圖表是內嵌 SVG，不需網路）。
- `videos/`：報告用的影片，`.mp4` 走 git LFS（`.gitattributes` 的 `meeting/**/*.mp4`）。手機拍的原檔是 HEVC（H.265），很多瀏覽器無法播放，一律轉成 H.264 再放進來；原檔放 `outputs/robot_v2/deploy/videos_raw_<日期>/`（不進 git）：

  ```powershell
  ffmpeg -i <原檔>.mp4 -vf scale=720:-2 -c:v libx264 -profile:v high -pix_fmt yuv420p -crf 23 -preset slow -c:a aac -b:a 128k -movflags +faststart <輸出>.mp4
  ```
  （直式影片用 `scale=720:-2`，橫式用 `scale=-2:720`。）
- `analyze_data.py`、`make_report.py`、`report_data.json`：從 `outputs/` 的原始資料產生報告數字與 HTML（可重現）。

## 報告清單

| 報告日期 | 資料夾 | 內容 |
|---|---|---|
| 2026-11-25 | [`20261125_robot_v2_real_test/`](20261125_robot_v2_real_test/report.html) | robot v2 實機測試（2026-11-23）：馬達確認、懸空／桌上站姿、0.5 倍步態回放、模型閉迴路 WALK；結論為供電電流不足 |

## 重新產生 2026-11-25 的報告

需要 `outputs/robot_v2/deploy/sessions/20261123_*`（不進 git，只在本機）：

```powershell
.venv_fw\Scripts\python.exe meeting\20261125_robot_v2_real_test\analyze_data.py   # -> report_data.json
.venv_fw\Scripts\python.exe meeting\20261125_robot_v2_real_test\make_report.py    # -> report.html
```

報告文字寫在 `make_report.py` 的 HTML 範本裡；改文字後重跑 `make_report.py` 即可（不需原始資料）。
