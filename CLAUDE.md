# 雙足機器人行走專案

在 NVIDIA Isaac Sim / Isaac Lab 中設計一台雙足機器人，並訓練其穩定直線行走。
使用者以繁體中文溝通，文件與回覆一律使用繁體中文；程式碼、識別字與 commit message 使用英文。
- **回覆語言（使用者特別強調）：主要語言一律是繁體中文。** 專業術語可以中英搭配（例如「扭矩（torque）」「強化學習（RL）」，或直接使用 PPO、URDF、`params.yaml` 這類通用英文名稱），但敘述句本身要用中文寫，不要整段或整篇使用英文。即使查到的資料、工具輸出或 skill 內容是英文，也要用中文轉述。

## 專案架構與流程（使用者 2026-10-15 確認：依「是什麼」分層，虛實分開）

| 資料夾 | 虛／實 | 內容 | 詳細需求 |
|---|---|---|---|
| `hardware/` | 共用（設計源頭） | 機構設計：`robot_v<N>/params.yaml` → 列印零件＋URDF | 見下方「機器人模型」、`hardware/CLAUDE.md` |
| `core/` | 共用 | 介面契約、左右鏡像、機器人定義（`robots/custom_biped.py`）；不依賴 Isaac，實機也會用 | 見下方「跨流程介面契約」 |
| `simulation/` | 虛擬 | Isaac Lab 環境、地形、場景版本、指標、影片；`robot_assets/`：URDF → USD、模擬檢查 | `simulation/env/README.md` |
| `control/rule_based/` | 虛擬 | 規則式行走控制器（老師） | `control/rule_based/PROMPT.md` |
| `control/flat_pipeline/` | 虛擬 | 平地：Rule-based → BC → PPO → 自我蒸餾 ×3 | `control/flat_pipeline/PROMPT.md` |
| `control/sim2real/` | 虛擬 | 強健性訓練（使用者 2026-10-06 新增）：隨機化、可部署觀測、步態多樣性、teacher → student 蒸餾；含訓練範圍內的驗收 | `control/sim2real/PROMPT.md` |
| `evaluation/` | 虛擬 | 不訓練：`generalization/`（訓練範圍以外的壓力測試，2026-10-09）、`load_analysis/`（馬達負載，2026-10-15） | `evaluation/generalization/CLAUDE.md` |
| `deploy/` | 實體 | 實機：台架測試、校正、板上控制程式、實地測試（**進行中**，狀態見 `docs/progress.md`）；不可依賴 Isaac | `deploy/README.md` |
| `docs/` | — | 跨領域文件：進度與計畫（`progress.md`）、待辦（`backlog.md`）、決策（`decisions.md`）、成果總表（`results.md`）、架構（`architecture.md`） | `docs/README.md` |
| `archive/exp1_reference/` | — | 實驗一（封存） | `archive/exp1_reference/CLAUDE.md` |

- 依序相依：hardware → simulation → control（rule_based → flat_pipeline → sim2real）→ evaluation → deploy；實機發現的問題回饋到 hardware（新版模型）或 control（重新訓練）。各階段狀態與下一步見 `docs/progress.md`。
- 程式資料夾與輸出資料夾一一對應：`outputs/robot_v<N>/{hardware, simulation, control/{rule_based, flat_pipeline, sim2real}, evaluation/generalization, deploy}/`。
- 沿革：2026-10-09 以前為 `01_robot_construction`、`02_rule_based`、`03_training_pipeline`、`04_sim2real`；2026-10-09 為 `01_`–`04_generalization`；2026-10-15 重整為上表。舊 run 的 `config.yaml`／`stats.json` 中的路徑是歷史紀錄，不改寫。

- 開始實作 rule-based、訓練管線、sim-to-real 或實機部署前（不論是在 `archive/exp1_reference/` 還是 `control/`、`evaluation/`、`deploy/` 中），**必須先完整閱讀對應的 PROMPT.md**；其中「若有不確定之處」列出的項目，不明確時要先問使用者，不可自行假設。
- 尚未決定的事項記錄在 `docs/backlog.md`，已定案的在 `docs/decisions.md`。遇到 backlog 中阻擋性的決策時要主動詢問，不可自行假設。

## 文件分工（使用者 2026-11-18 確認）

**開始一個工作前，先讀 `docs/progress.md`**（目前狀態與下一步）。跨領域、會變動的內容只寫在 `docs/`，各寫一份，不在資料夾 README 重複：

| 內容 | 寫在哪裡 |
|---|---|
| 計畫、狀態、下一步 | `docs/progress.md`（完成一個可驗證的步驟就更新；舊內容移到 Archive） |
| 尚未解決的問題、待實物確認（所有領域） | `docs/backlog.md`（附日期、數字或 run 路徑；解決後改狀態，不刪除） |
| 使用者確認的決定 | `docs/decisions.md`（一筆一列；推翻時新增一列，舊列標「被 #N 取代」） |
| 各版本、各階段的數字 | `docs/results.md`（進 git；`outputs/` 內的報告與影片是詳細版） |
| 重要的評估影片 | `docs/videos/`（壓縮複本、git LFS；`manifest.csv` ＋ `tools/archive_videos.py`，只新增不覆蓋） |
| 資料夾分工、依賴、目錄結構、沿革 | `docs/architecture.md` |
| 某資料夾的用途、執行方式、該階段紀錄 | 該資料夾的 `README.md`；紀錄太長時拆成 `log_robot_v<N>.md`（例如 `control/sim2real/`） |
| 某機器人版本的規格、組裝、接線、設計計算 | `hardware/robot_v<N>/`（README、CHANGELOG、ASSEMBLY、design_calc） |
| 實機測試紀錄 | `deploy/test_log.md` |
| 原始需求 | 各資料夾的 `PROMPT.md`（不修改） |

commit 前的同步檢查要包含 `docs/`。

## 兩次實驗

上述流程總共執行兩次：

| 實驗 | 資料夾 | 機器人 | 流程 | 目的 |
|---|---|---|---|---|
| 實驗一（測試） | `archive/exp1_reference/` | `references/` 中的參考 URDF（髖關節位置修正版） | 略過建模，直接 rule-based → 訓練 | 確認 rule-based 與訓練管線可行 |
| 實驗二（正式） | `hardware/`、`core/`、`simulation/`、`control/`、`evaluation/`、`deploy/` | 自行設計的參數化模型 | 建模 → 老師 → 訓練 → 評估 → 實機 | 在新模型上套用已驗證的管線，強化到可部署，再做成實體 |

- **實驗一（`archive/exp1_reference/`）已完成，使用者於 2026-09-29 確認，目前進入實驗二（從 `hardware/` 開始）。** 實驗一的細節與參考模型的處理方式見 `archive/exp1_reference/CLAUDE.md`，結論見 `archive/exp1_reference/outputs/summary.md`。
- **實驗一的經驗已經整理進實驗二的規格**：機器人設計要求與馬達選型在 `hardware/CLAUDE.md`；rule-based 的做法與陷阱在 `control/rule_based/CLAUDE.md`；訓練管線（獎勵設計、BC→PPO 權重接續、篩選門檻）在 `control/flat_pipeline/CLAUDE.md`。動工前先讀。
- 需求規格：`control/rule_based/`、`control/flat_pipeline/` 中的 PROMPT.md 與 CLAUDE.md 是原始版本，已複製一份到 `archive/exp1_reference/rule_based/` 與 `archive/exp1_reference/training_pipeline/`。實驗一只修改複本，原始版本保持不動；實驗一結束後，比較兩者的差異，再決定實驗二要採用哪個版本。
- 實驗一的程式碼要**可以直接移植**：和機器人相關的內容（asset config、關節順序、站姿、致動器參數、觀測／動作定義）集中在 robot 設定檔中，管線程式碼不寫死任何機器人假設。實驗一完成後，把驗證過的程式碼移植到實驗二（`core/`、`simulation/`、`control/`），只替換 robot 設定，不從頭重寫。
- 實驗一的結論（哪些部分有效、哪些需要調整、各項指標、哪些問題來自參考模型本身）寫在 `archive/exp1_reference/outputs/summary.md`，作為實驗二的起點。**最終成績**：rule-based 成功率 95%／跌倒 0% → PPO 三輪都是 100%／0% → 最終蒸餾模型 99%／0%、推擠測試 100%。

## 機器人模型

- `references/` **僅供參考，不可修改**（權限設定已禁止寫入）。實驗一直接讀取其中的 URDF；實驗二的新模型不需沿用它的設計。
- 實驗二的新模型採**程式參數化 URDF**：由單一參數檔（尺寸、質量、關節限制、致動器）產生 URDF，再轉成 USD。流程見 `/robot-model` skill。
- **實驗二的機器人之後會 3D 列印成實體，在真實環境測試**：設計時就要考慮零件可列印性、馬達固定方式、組裝順序、線材走向與維修性，不能只做「模擬能跑」的幾何。模擬中的質量、扭矩上限、關節範圍都要對應真實零件。
- **零件分開、組裝方式要特別留意**（使用者 2026-09-30 強調）：每個列印件是獨立檔案；零件之間怎麼接（螺絲規格與位置、熱熔螺母、卡榫、定位銷、公差）要在設計時就定義清楚，並寫進該版的組裝說明。
- 實驗二的機器人：每腿 4 DOF（hip roll、hip pitch、knee、ankle pitch）＋ 2 DOF 主動尾巴（yaw、pitch），共 10 顆 Feetech HD-1910-C001 伺服，無上半身。細節見 `hardware/CLAUDE.md`。
- 座標慣例依照 Isaac Lab：**+X 前進、+Y 左、+Z 上**，SI 單位，角度用 rad。
- 碰撞體使用 box/capsule/sphere 等基本形狀；mesh 只用於外觀。

### 參考模型的已知問題
以下是參考模型（`雙足機器人_髖關節位置修正`）的已知問題。實驗一的處理方式見 `archive/exp1_reference/CLAUDE.md`；實驗二的新設計應避免這些問題：
- 共 10 個關節：每隻腳 4 個（hip roll、hip pitch、knee、ankle pitch）＋軀幹 yaw、pitch 各 1 個。**沒有 hip yaw 與 ankle roll**，轉向和側向平衡能力有限。
- 總重 28.4 kg，但兩隻腳掌各 6.03 kg（合計 42%），質量分布不合理。
- 大腿 0.10 m、小腿 0.09 m，相對身體偏短；膝關節可正反兩向彎曲 ±45°。
- effort／velocity 都是預設值 100，碰撞體全為 mesh，摩擦係數 0.2 偏低。
- 由 Fusion 360 匯出，前進方向為 Y 軸，與 Isaac Lab 的 +X 慣例不同。
- 檢查任何 URDF：`tools\isaacpy.cmd tools\urdf_inspect.py <file.urdf>`（有 warning 時 exit code 為 1）。

## 跨流程介面契約（最重要）

同一個實驗中，rule-based 控制器和訓練出的 policy **必須使用完全相同的觀測／動作空間**，兩邊不可各自定義。
- 觀測、動作、關節順序、控制頻率、動作縮放方式只能定義在該實驗的共用資料夾中：實驗一是 `archive/exp1_reference/shared/`，實驗二是根目錄的 `core/`（`interface.py`、`interface_v2.py`）。機器人特有的部分放在 `core/robots/<robot>.py`，rule-based、訓練與之後的實機程式都從這裡 import。
- 兩次實驗的機器人自由度不同，因此觀測／動作的維度可以不同；但定義的**結構和順序規則**要一致，這樣移植時不需要修改管線程式碼。
- 任何修改都要同時確認兩邊的相容性，並更新共用模組的 docstring（包含維度、單位、順序）。
- 規則式控制器的介面固定為 `RuleBasedBipedController`，提供 `reset()` 與 `compute_action(observation) -> action`。
- 評估影片疊加資訊的格式在兩次實驗、rule-based 與訓練之間都相同，見 `/eval-video` skill。

## 執行環境（Windows）

- GPU：RTX 5080 Laptop（16 GB）。Isaac Sim 6.0.1 位於 `C:\isaacsim`，Isaac Lab 3.0.0 位於 `C:\isaaclab`。
- 系統沒有可用的 `python`（只有 Microsoft Store 的空殼）。**所有 Python 都透過 `tools\isaacpy.cmd` 執行**（包裝 `C:\isaaclab\isaaclab.bat -p`），可用套件：torch 2.10 (cu128)、isaaclab、rsl_rl 5.0.1、skrl、h5py、imageio、cv2、tensorboard、mujoco。
- 每次執行都會印出 `setup_conda_env.bat is missing` 的警告，這是無害的，可忽略。
- 啟動模擬器的細節、旗標與常見問題見 `/isaac-run` skill。
- 參考 Isaac Lab 原始碼（`C:\isaaclab\source`，例如 `isaaclab_tasks/.../locomotion/velocity/config/{cassie,h1,g1}`）時，優先以本機原始碼為準；查詢外部函式庫文件時使用 Context7 MCP。Isaac Lab 3.0 很新，不要依賴記憶中的舊版 API。

## 目錄規劃

完整的目錄結構、依賴方向與沿革見 `docs/architecture.md`（2026-11-18 從這裡移過去）。

- **匯入方式**：腳本把專案根目錄（`core`、`simulation`）與 `control/`（`rule_based`、`flat_pipeline`、`sim2real`）加入 `sys.path`。
- **命名**：資料夾與檔案名稱一律使用英文（使用者 2026-10-15 規則）。
- **刻意保留的複本**：實驗一的程式（`archive/exp1_reference/shared/`、`archive/exp1_reference/rule_based/`）凍結不動，實驗二移植時複製一份再修改，所以有些檔案目前兩邊內容相同（`rollout.py`、`video_overlay.py`、`eval_videos.py`、`leg_kinematics.py`）。這是為了讓兩次實驗各自可重現，實驗二只改實驗二的複本，不回頭改 `archive/exp1_reference/`。`PROMPT.md` 的複本也是同樣道理。
- 大型產出一律放在 `<產出根目錄>/<stage>/<run_name>/`：實驗一的產出根目錄是 `archive/exp1_reference/outputs/`，實驗二是 `outputs/`。實驗一的 `<stage>` 為 `rule_based` 或 `training`（例如 `archive/exp1_reference/outputs/training/iter1_ppo_20260918/`）；實驗二的 `<stage>` 對應程式資料夾：`hardware`、`simulation`、`control/rule_based`、`control/flat_pipeline`、`control/sim2real`、`evaluation/generalization`、`deploy`（2026-10-15 起），run 名稱都包含日期。
- 所有超參數放在 config 檔（dataclass 或 YAML），不要寫死在程式中。每次訓練或評估都保存當下使用的 config。

## 版本控制（實驗二，使用者 2026-09-30 要求）

機器人模型、rule-based、訓練都預期會迭代多次（v1、v2、v3…）。**每個版本的設計與結果都要保留，不可覆蓋。**
- **機器人模型**：每個版本一個資料夾 `hardware/robot_v<N>/`，放該版的 `params.yaml`、產出的 URDF、列印零件檔與 `CHANGELOG`（相對上一版改了什麼、為什麼）。
  產生程式共用，但每版記錄產生時的 git commit，並打 git tag `robot-v<N>`，確保舊版可以重建。
- **下游一律標明機器人版本**：robot 設定、USD、rule-based 參數、訓練產出都帶版本，例如 `outputs/robot_v1/control/rule_based/<run>/`、`outputs/robot_v1/control/flat_pipeline/<run>/`。
- **run 資料夾不可覆寫**：run 名稱含日期與時間（例如 `iter1_ppo_20261005_1430`），程式遇到已存在的資料夾要停下來，不可覆蓋。
- 各版本、各階段的數字整理在 `docs/results.md`（進 git，含跨版本比較）；`outputs/robot_v<N>/summary.md` 是不進 git 的詳細版（含影片連結），可以同步維護。
- 資料夾結構以 2026-10-15 使用者確認的分層為準（見上方「專案架構與流程」）。

## 工作準則

- 除非使用者要求，否則不要安裝新的 MCP server 或大型依賴；使用者重視 token 用量。
- 耗時的模擬或訓練使用 `run_in_background` 執行，不要一直輪詢。
- 回報結果時，要附上實際數字（跌倒率、平均行走距離、側向偏移），不可只寫「看起來正常」。
- 使用者要求時才 commit；commit 前確認沒有把 `outputs/` 或大型二進位檔加進去。
- **GitHub**（2026-11-18）：remote `origin` = https://github.com/sevenworm7/bipedal_robot（影片走 git LFS）。**使用者授權：每次 commit 後直接 push 目前的 branch 到 origin**，不用再問；其他遠端操作（force push、刪 branch、改 tag、merge 回 `master`）仍要先問。`robot-v2` 合併回 `master` 等使用者決定。branch 和 tag 同名（`robot-v2`）時，push 要寫完整 refspec（`refs/heads/…`、`refs/tags/…`）。
- **每個新增的資料夾都要有 `README.md`**，說明用途、內容、做了哪些事、如何執行；內容有變動時同步更新。
- **CAD 用的 Python 環境（實驗二，2026-09-30 使用者同意）**：build123d 不能裝進 Isaac Lab 的環境，改用 uv 建立獨立環境，**全部放在專案目錄內**：
  `tools/bin/uv.exe`（uv 執行檔）、`.uv/`（uv 下載的 Python 與快取，設 `UV_PYTHON_INSTALL_DIR`、`UV_CACHE_DIR`）、`.venv_cad/`（build123d 環境）。三者都不進 git。
  CAD 程式一律用這個環境執行，模擬與訓練仍用 `tools\isaacpy.cmd`。
- Isaac Lab 3.0 的 API 與舊版差異很大（四元數 XYZW、`.torch`、`_index` 寫入方法、巢狀的 URDF 匯入結果），動手前先看 `/isaac-run` skill 的陷阱清單。
