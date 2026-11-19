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
