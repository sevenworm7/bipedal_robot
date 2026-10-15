# vendor：飛特官方軟體與 SDK（2026-09-30 從 gitee 下載）

飛特電子使用手冊的「資料下載」區連到 gitee，瀏覽器需要登入才能下載；這裡用 gitee 的 raw 檔案網址直接抓下來。
不進 git（`.gitignore` 已排除 `vendor/fddebug/`），需要時重新下載即可。

| 位置 | 內容 | 來源 |
|---|---|---|
| `fddebug/FD1.9.8.5(250729).7z` | **FD 除錯軟體**（Windows，最新版）：掃描馬達、改 ID、中位校準、改保護參數、即時看位置／電流／溫度 | https://gitee.com/ftservo/fddebug |
| `fddebug/CH343SER.ZIP`、`fddebug/CH341SER.ZIP` | USB 轉 TTL 轉接板（例如飛特 URT-1）用的 Windows 驅動 | 同上 |
| `fddebug/README.md` | 軟體說明 | 同上 |
| `FTServo_Python/` | **飛特 Python SDK**：`scservo_sdk/`（通訊函式庫，`sms_sts.py` 對應 STS 系列、`hls.py` 對應 HLS 系列）與範例（`sms_sts/`、`hls/` 資料夾中的 ping、read、write、sync_read、sync_write） | https://gitee.com/ftservo/FTServo_Python |

使用方式：解壓 `.7z` 需要 7-Zip。第一次接馬達：裝驅動 → 開 FD → 選 COM 埠與 1 Mbps → 掃描。
**一次只接一顆**（出廠 ID 都是 1），改完 ID 再接下一顆。
