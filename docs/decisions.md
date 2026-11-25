# 決策紀錄

> 使用者確認過的重要決定，一筆一列，依日期排序。新決定加在最下面；推翻舊決定時新增一列，並在舊列的「狀態」寫「被 #N 取代」，不要刪改舊列。
> 細節與推導在「出處」那份文件。已定案的決策不要在新對話裡重新爭論，除非出現新的實測證據。
> 2026-11-18 以前的決策是從各資料夾的 README（「已決定」「開始前確認的事項」）回填的，原文仍保留在出處。

| # | 日期 | 領域 | 決定 | 原因 | 出處 | 狀態 |
|---|---|---|---|---|---|---|
| 1 | 2026-09-29 | 專案 | 分兩次實驗：實驗一用參考 URDF 驗證管線，實驗二用自行設計的模型 | 先確認 rule-based 與訓練管線可行，再投入建模 | `CLAUDE.md`「兩次實驗」 | 有效 |
| 2 | 2026-09-29 | hardware | 每腿 4 DOF（hip roll、hip pitch、knee、ankle pitch）＋ 2 DOF 主動尾巴，共 10 顆馬達；不做上半身 | 設備上限 10 顆；尾巴是研究主題，取代 ankle roll 與 hip yaw | `hardware/README.md`「已決定」、`hardware/design_refs/tail_balance_research.md` | 有效 |
| 3 | 2026-09-29 | hardware | 恐龍式站姿：腳踝在髖關節後方，抵銷尾巴往後拉的重心 | 尾巴把重心往後拉約 21 mm | `hardware/design_refs/architecture_v0_1.md` | 有效 |
| 4 | 2026-09-30 | hardware | 馬達用 Feetech HD-1910-C001（堵轉 1.18 N·m @ 6 V、21 g、金屬齒輪、雙出軸） | 扭矩足夠（靜態需求約堵轉的 16–22%）；扭矩不足時靠縮小尺寸解決，不換馬達 | `hardware/design_refs/hd1910/docs/actuator_selection.md` | 有效 |
| 5 | 2026-09-30 | hardware | 零件 CAD 用 build123d 程式產生，`robot_v<N>/params.yaml` 是唯一參數來源，同時產生 STEP／STL 與 URDF；build123d 放在獨立的 `.venv_cad` | 改一個尺寸就能重新產生全部零件與 URDF；不動 Isaac Lab 的環境 | `hardware/generator/README.md` | 有效 |
| 6 | 2026-09-30 | hardware | 組裝用 M2 螺絲＋熱熔銅螺母；PLA／PETG、0.4 mm 噴嘴、±0.2 mm 公差、單件 ≤ 180 mm | 印表機未定，用一般規格設計 | `hardware/robot_v1/ASSEMBLY.md` | 有效 |
| 7 | 2026-09-30 | 專案 | 版本控制：模型、rule-based、訓練每一版都保留；模型放 `robot_v<N>/` 並打 tag；run 資料夾名稱含日期時間，不可覆寫 | 預期會迭代多次，需要能重現與比較 | `CLAUDE.md`「版本控制」 | 有效 |
| 8 | 2026-09-30 | control | rule-based：只用平地；尾巴放進介面（觀測 44 維、動作 10 維）；指令 0.05–0.15 m/s、1.2–2.0 Hz | 依 v1 靜態檢查的步幅上限 ±78 mm | `control/rule_based/README.md`「開始前確認的事項」 | 有效 |
| 9 | 2026-10-01 | control | 平地訓練：沿用實驗一的管線（BC → PPO → 蒸餾 ×3）；PPO 加輕推擠 0.1 m/s；這一輪不加 sim-to-real 隨機化；篩選門檻看第 1 輪資料分布再定 | 先照實驗一跑通，再加強健性 | `control/flat_pipeline/README.md`「開始前確認的事項」 | 有效 |
| 10 | 2026-10-06 | control | 新增 sim2real 階段（當時先不做 robot v2）：actor 拿掉 base 線速度、加 5 步歷史；致動器寬範圍隨機化；步態多樣性（站立、原地踏步、啟停）；不做轉向與倒退 | 平地模型需要實機量不到的 base 線速度，且沒有延遲與地形 | `control/sim2real/README.md`「開始前確認的事項」 | 有效 |
| 11 | 2026-10-06 | control | 部署門檻：`flat_v1` 與中等隨機化成功率 ≥ 95%、不規則地形 ≥ 85%、各難度跌倒率 ≤ 5% | — | `control/sim2real/README.md` | 有效 |
| 12 | 2026-10-06 | control | 加入鏡像對稱（資料擴充＋鏡像損失）；獎勵只懲罰策略看得到的平均側向速度 | student v3 出現跛行步態（左右步長 49／14 mm）；看不到的側向位置懲罰無效 | `control/sim2real/log_robot_v1.md` 第四、五輪 | 有效 |
| 13 | 2026-10-07 | control | 最終可部署模型是 student v5，模擬測試告一段落 | 通過全部門檻、步態對稱、沒有自發停頓 | `control/sim2real/README.md`「結論」 | 有效 |
| 14 | 2026-10-09 | evaluation | 訓練範圍以外的壓力測試獨立成 `evaluation/generalization/`，不訓練 | 分開「產生模型」與「找失效邊界」 | `evaluation/generalization/README.md` | 有效 |
| 15 | 2026-10-15 | 專案 | 資料夾依「是什麼」分層、虛實分開：`hardware`、`core`、`simulation`、`control`、`evaluation`、`deploy`、`archive`；名稱一律英文 | 原本的 `01_`–`04_` 編號在加入實機與 v2 後不好擴充 | [`architecture.md`](architecture.md) | 有效 |
| 16 | 2026-11-18 | hardware | 做 robot v2：只縮短大腿與小腿，其他零件沿用；尾巴維持 250 mm（可以碰地） | 腿部馬達線太短（`backlog.md` #5）；不考慮分線板或自製線 | `hardware/robot_v2/design_calc.md` | 有效 |
| 17 | 2026-11-18 | hardware | v2 腿長：大腿 70 mm、小腿 55 mm，膝 0–110° | 折返接法最長段 109 mm（≤ 120 mm）；膝 120° 時腳掌會撞 hip pitch 馬達 | `hardware/robot_v2/design_calc.md`、`CHANGELOG.md` | 有效 |
| 18 | 2026-11-18 | hardware | 接線採方案 B：一條線串接全部馬達，最後一顆不接回芒果磚：芒果磚 → 左1 → 尾巴 yaw → 尾巴 pitch → 右1 → 右3 → 右4 → 右2 → 左2 → 左3 → 左4 | 只有右腿折返，沒有腳鍊；匯流排用 ID 定址，順序不影響通訊 | `hardware/robot_v2/ASSEMBLY.md` 第 2 節 | 有效 |
| 19 | 2026-11-18 | hardware | 模型要規定關節範圍，且要和實體一致（等於或略小於實體極限），同時用在 URDF、模擬、訓練與 viewer；兩腳互撞由模擬的自我碰撞擋住 | 訓練出的動作本來就在合理範圍，模型和實體要一致才不會有落差 | 本檔；實測待辦見 `backlog.md` #13 | 有效 |
| 20 | 2026-11-18 | control | v2 訓練接續 v1 的 sim2real 模型（零樣本評估 → teacher 微調 → student 蒸餾 → 評估），不重跑 rule-based、BC、平地訓練；門檻同 v1 | 觀測、動作、控制頻率都沒變 | [`progress.md`](progress.md) | 有效 |
| 21 | 2026-11-18 | 專案 | 文件整理：計畫與狀態只寫 `docs/progress.md`，待辦寫 `docs/backlog.md`，決策寫本檔，數字寫 `docs/results.md`（進 git）；資料夾 README 只寫用途、執行方式和該階段的紀錄 | 計畫原本散在 6 份 README，成果總表在不進 git 的 `outputs/` | [`README.md`](README.md) | 有效 |
| 22 | 2026-11-18 | 專案 | 重要影片壓縮後放進 `docs/videos/`，用 git LFS 追蹤（只限 `docs/videos/**/*.mp4`）；方案 A：以 medium 為主，加代表性失效例子，每版約 50 MB；只新增不覆蓋 | `outputs/` 不進 git；專案之後要放到 GitHub，趁影片還沒 commit 時用 LFS，避免日後改寫歷史 | [`videos/README.md`](videos/README.md) | 有效 |
| 23 | 2026-11-19 | control | robot v2 的最終模型 = **student v2ft**（`outputs/robot_v2/control/sim2real/student_v2ft_20261118_230657/student.pt`），作為實機部署的模型 | 通過全部門檻（`flat_v1`／`dr_v1` 100%、`rough_eval_v1` 99%、組合 92.7%／跌倒 0.4%）；使用者看過影片，步態良好 | [`results.md`](results.md)「robot v2」、`control/sim2real/log_robot_v2.md` | 有效 |
| 24 | 2026-11-19 | deploy | 實機**站立時不跑模型**，改用固定站姿（action 0 ＝預設關節角）；要走時才把控制交給 student v2ft，停下時切回固定站姿。不為此重訓 | v2 模型在站立指令下兩腳互頂，左 hip roll 持續 141% 額定（`backlog.md` #15）；固定站姿 hip roll 只有 0.06–0.09 N·m，模擬交接後 `flat_v1`／`dr_v1` 都是 100%／0% | [`results.md`](results.md)、`evaluation/load_analysis/stand_handover.py` | 有效 |
| 25 | 2026-11-19 | deploy | 實機的伺服延遲（指令送出到伺服開始動作）要求 **≤ 40 ms**，台架測試時量測；實測超過才重訓放寬延遲隨機化 | v2 在訓練範圍 0–40 ms 內 99–100%，60 ms 掉到 46%（不跌倒但速度失控；v1 60 ms 仍 100%） | [`results.md`](results.md)「v2 泛化、壓力測試與馬達負載」 | 有效 |
| 26 | 2026-11-19 | deploy | 組員的芒果磚交接包（robot v1，4.1 GB）整包放在 `references/mango_x4_handoff_20261117/`，唯讀、不進 git；要用的程式或資料**複製**到 `deploy/` 並註明來源路徑與 SHA256 | 包內有工具鏈、二進位檔、我們自己專案的舊快照（2.4 GB）和會寫 EEPROM 的維護腳本，不適合整包進 repo | `deploy/README.md`、`deploy/test_log.md` | 有效 |
| 27 | 2026-11-19 | deploy | v2 沿用實機舵機已寫入的 ID（TOPDOWN_V1：右腿 1–4、左腿 5–8、尾巴 9 = yaw、10 = pitch），不改寫 EEPROM；ID 對照寫在 `deploy/calibration/robot_v2.json`，凍結的 `hardware/robot_v2` 不改 | 匯流排以 ID 定址，接線順序（#18）不影響；舵機在 v2 裝回原關節即可沿用 | `deploy/calibration/README.md` | 有效 |
| 28 | 2026-11-19 | deploy | v2 板上程式由本專案負責：模型在芒果磚（RP2350）上推論，權重用 **int8 放 SRAM**；初期由 PC 經 USB 送指令並記錄遙測；建置工具 CMake／Ninja 用 uv 裝在專案內 `.venv_fw/` | float32 權重 1.09 MB 放不進 520 KB SRAM，從 flash 讀約 30 ms 超過 20 ms 週期；int8 在模擬閉迴路 `flat_v1`／`dr_v1`／`rough_eval_v1` 都是 100%／0% | `deploy/firmware/README.md`、`deploy/export/README.md` | 有效 |
| 29 | 2026-11-23 | control | （被 #30 取代）**暫不重新訓練慢步態模型**，維持 student v2ft；先解決供電（`backlog.md` #21）。依據：降步頻指令（1.2／1.0／0.8 Hz）在模擬中不跌倒，但關節速度與電流不變（`results.md`「v2 慢步態評估」）；放慢步態也不會降低電流 | 換供電後實機仍跟不上時，再依 `backlog.md` #25 重新微調 |
| 30 | 2026-11-23 | control | **訓練慢步態模型作為比較**（推翻 #29 的「暫不重訓」）：步頻 0.8–1.2 Hz、速度 0.03–0.08 m/s、模擬伺服速度上限 5.2 rad/s、騰空時間目標 0.25 s；從 teacher v2ft 接續 | 使用者：無論效果好不好都可以作為比較；計畫見 `control/sim2real/log_robot_v2.md`「慢步態模型 slow」 |
| 31 | 2026-11-25 | deploy | **伺服供電改用降壓模組 SD026**（300W 20A 恆壓恆流，持續 15 A、最大 20 A，輸出調 8.0 V、限流 ≥ 15 A）；**舊的 HW-133A（3 A）停用，不再使用** | HW-133A 在懸空 WALK 時讓電壓掉到 5.1–6.1 V、伺服欠壓失聯（`backlog.md` #21）；模擬行走峰值約 8.8 A | `deploy/power_supply.md`、`deploy/test_log.md` 2026-11-25 | 有效（待實測） |
