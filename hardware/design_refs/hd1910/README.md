# hd1910：Feetech HD-1910-C001 伺服的資料包（2026-09-30 下載）

實驗二採用的馬達。本資料夾是**第三方資料的本機副本**，用來建模、寫 `params.yaml` 與之後的實機設定。
每份檔案都保留來源；數值以廠商正式規格書為準，這裡只是轉錄。

## 最重要的數字（廠商《HD-1910-C001 串型規格書》A/0，2026-09-07，經 microduck-replica 轉錄）

| 項目 | 7.4 V | 6 V |
|---|---|---|
| 堵轉扭矩 | **15 kg·cm ＝ 1.47 N·m** | 12 kg·cm ＝ 1.18 N·m |
| 額定扭矩 | 3.7 kg·cm ＝ 0.363 N·m | 3.0 kg·cm ＝ 0.294 N·m |
| 空載轉速 | 113 rpm ＝ 11.8 rad/s（0.088 s/60°） | 92 rpm ＝ 9.6 rad/s |
| 堵轉電流 | 2.0 A | 1.6 A |
| 額定電流 | 900 mA | 690 mA |
| 空載電流 | ≤ 210 mA | ≤ 180 mA |

- 其他：Kt 7.5 kg·cm/A（0.74 N·m/A）、減速比 **320:1**、齒輪虛位 ≤ 0.5°、12 bit 磁編碼（4096，中位 2048）、
  外形 **34 × 20 × 23 mm**、重量 **21 ± 2 g**、外殼 PA66+GF43%、金屬齒、壽命 > 100000 次、電壓 **4–8.4 V**。
- 通訊：TTL 半雙工單線，8N1，38400 bps–1 Mbps（**出廠 1 Mbps**），ID 0–253（**出廠 ID 1**），Feetech SCS/STS 協定。
- 連接器 AMP-3P。**腳位順序依型號不同，接線前必須用規格書或三用電表確認**（同平台的 HL-2915 是 1=GND／2=Vcc／3=Signal）。
- 出廠控制模式：**模式 4「純位置 PD（Sim2Real）」**，適合強化學習上機。另有角度伺服、恆速、恆流、PWM。
- 保護：過熱 > 80 ℃（預設開）；**過流、過壓預設關閉**，裝機前要打開。
- 產品頁的「10 kg·cm」是保守值；正式規格書是 12 kg·cm @ 6 V、15 kg·cm @ 7.4 V。
- 2S 鋰電的工作區間 6.6–8.2 V 落在額定範圍內，最接近 7.4 V 那一欄；但**滿電 8.4 V 等於電壓上限，沒有餘量**。

## 內容

| 位置 | 內容 | 來源與授權 |
|---|---|---|
| `cad/HD-1910-c001-dan.stp` | **HD-1910 單出軸版 STEP**（使用者由 SLDPRT 線上轉檔，2026-09-30）。尺寸量測結果見 `../architecture_v0_1.md` 第 1 節 | 由下列 SLDPRT 轉出，不進 git |
| `cad/HD-1910-c001-dan.SLDPRT` | HD-1910 3D 模型，單出軸版（SolidWorks 零件） | fanhao375/microduck-replica-cad v2.1（CC BY-NC-SA 4.0），不進 git |
| `cad/HD-1910-c001-shuang.stl` | **雙出軸版 STL**（使用者由 SLDPRT 經網頁工具轉檔，STEP 轉檔失敗）。尺寸見 `../architecture_v0_1.md` 第 1 節 | 不進 git |
| `cad/HD-1910-c001-shuang.SLDPRT` | 雙出軸版原始檔（Fusion 360 開不了，網頁工具可以） | 同上 |
| `cad/SolidWorks-FT2026-09-25.zip` | Microduck 飛特版完整 SolidWorks 組裝（17 組件＋43 零件，343 MB），可參考支架怎麼包馬達 | 同上 |
| `docs/hardware_reverse_engineering.md` | 含 HD-1910 規格書轉錄、保護、電氣介面、電流估算（第 493 行起） | fanhao375/microduck-replica |
| `docs/hd1910_pretrain_checklist.md` | 訓練前資料清單：規格、暫存器換算、BAM 辨識台架建議 | 同上 |
| `docs/actuator_selection.md` | XL330／HD-1910／其他伺服的比較 | 同上 |
| `docs/feetech_official/STS_memory_table.md` | **飛特官方 STS 系列內存表**（HD-1910 所屬系列），2026-09-30 直接從文件站 API 擷取 | doc.feetech.cn（q=2506abfb1cb0） |
| `docs/feetech_official/HLS_memory_table.md` | 官方 HLS 系列內存表（HD-1910 部分韌體行為與 HLS 相同，例如中位校準方式），參考用 | doc.feetech.cn（q=250689e628d7） |
| `docs/feetech_official/FT_custom_protocol.md` | 官方「FT 自定義協議」（封包格式、指令） | doc.feetech.cn（q=2506ba160b93） |
| `docs/feetech_STS_memory_table_v1.1.md`、`docs/feetech_SCS_protocol_v1.0.md` | 同上兩份的舊副本（microduck-replica 於 2026-09-17 擷取），保留對照 | microduck-replica |
| `vendor/` | **飛特 FD 除錯軟體、USB 轉接板驅動、Python SDK**，見 `vendor/README.md` | gitee.com/ftservo |
| `docs/feetech_docs_index.md` | 上述官方文件的索引與取得方式 | microduck-replica |
| `docs/feetech_SMS_STS_HTS_hex_command_generator_250508.xlsx` | 官方 Excel：填參數自動產生十六進位指令（改 ID、中位校準等） | 飛特官方，不進 git |
| `sim/1910_m6.json` | **HD-1910 的 BAM M6 致動器模型參數**（摩擦、Kt、電阻、armature、延遲） | LuwuDynamics/xgoduck_rl（Apache-2.0，`sim/LICENSE_Apache-2.0`） |
| `sim/feetech_bam.py`、`sim/microduck_hd1910_env_cfg.py`、`sim/hd1910-baseline.md` | 上述參數在 mjlab 中的用法與驗證紀錄 | microduck-replica（Apache-2.0 部分） |

## 取得方式

- **飛特電子使用手冊**（<http://doc.feetech.cn>）是前端網頁，瀏覽器無法直接下載；正文可以用它的 API 取得 Markdown：
  索引 `http://longbos.com:9009/workflow/yf-specification/getBysrcType/FTServo-emanual-ab4947a479784a71a8cb7928`，
  單篇 `http://longbos.com:9009/workflow/yf-specification/getQr/<q>`（回傳 JSON，`data.mdcontent` 是正文）。
- **gitee 上的軟體**（FD、SDK）瀏覽器要登入；用 `https://gitee.com/ftservo/<repo>/raw/<branch>/<path>` 可以直接下載單一檔案。
  其他 SDK：Arduino `FTServo_Arduino`、Linux `FTServo_Linux`、STM32 `FTServo_stm32HAL`（決定控制板後再抓）。
- **仍然拿不到的**：正式規格書 PDF《HD-1910-C001 串型規格書》與官方 STEP `HD-1910-C001.stp`，是廠商發給客戶的文件，
  要向賣家（例如 [OpenELAB](https://openelab.com/products/feetech-hd1910-microduck-coreless-servo)）或[飛特官網](https://www.feetechrc.com/)索取。
- BAM 致動器辨識工具：<https://github.com/Rhoban/bam>

## 對本專案的用途

- **建模**：已由 STEP 量出輸出軸位置（距一端 9.5 mm）、擺臂與固定孔位，見 `../architecture_v0_1.md` 第 1 節。
- **模擬**：Isaac Lab 沒有 BAM。可以把 M6 模型移植成自訂 actuator，或先用 `DCMotorCfg` 加上摩擦與延遲近似，之後用實機階躍響應比對。
- **實機**：出廠 ID 都是 1，組裝前要逐顆改 ID（左腿 1–4、右腿 5–8、尾巴 9–10；改 ID 時一次只接一顆，避免和出廠的 ID 1 衝突）；打開過流、過壓保護；用 0x0B 位置校準指令做中位校準（HD-1910 韌體不接受寫 128 的校準方式）。
