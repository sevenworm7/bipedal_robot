# 雙足機器人行走專案

在 NVIDIA Isaac Sim / Isaac Lab 中設計一台雙足機器人，並訓練其穩定直線行走。
使用者以繁體中文溝通，文件與回覆一律使用繁體中文；程式碼、識別字與 commit message 使用英文。
- **回覆語言（使用者特別強調）：主要語言一律是繁體中文。** 專業術語可以中英搭配（例如「扭矩（torque）」「強化學習（RL）」，或直接使用 PPO、URDF、`params.yaml` 這類通用英文名稱），但敘述句本身要用中文寫，不要整段或整篇使用英文。即使查到的資料、工具輸出或 skill 內容是英文，也要用中文轉述。

## 開發流程（依序相依）

| 資料夾 | 內容 | 詳細需求 |
|---|---|---|
| `01_robot_construction/` | 機器人模型設計（程式參數化 URDF → USD） | 見下方「機器人模型」 |
| `02_rule_based/` | 規則式行走控制器，作為教師策略 | `02_rule_based/PROMPT.md` |
| `03_training_pipeline/` | Rule-based → BC → PPO → 自我蒸餾 ×3 → 評估影片 | `03_training_pipeline/PROMPT.md` |
| `04_sim2real/` | 實驗二新增（使用者 2026-10-06）：sim-to-real 隨機化、可部署觀測、步態多樣性、student 蒸餾 | `04_sim2real/PROMPT.md` |

- 開始實作 rule-based、訓練管線或 sim-to-real 前（不論是在 `00_test/` 還是 02、03、04 中），**必須先完整閱讀對應的 PROMPT.md**；其中「若有不確定之處」列出的項目，不明確時要先問使用者，不可自行假設。
- 實驗二尚未決定的設計事項（尺寸、質量分配、關節範圍、尾巴參數、訓練目標）記錄在 `01_robot_construction/README.md` 的「待決定事項」。這些是阻擋性的決策，遇到時要主動詢問。

## 兩次實驗

上述流程總共執行兩次：

| 實驗 | 資料夾 | 機器人 | 流程 | 目的 |
|---|---|---|---|---|
| 實驗一（測試） | `00_test/` | `references/` 中的參考 URDF（髖關節位置修正版） | 略過建模，直接 rule-based → 訓練 | 確認 rule-based 與訓練管線可行 |
| 實驗二（正式） | `01_`、`02_`、`03_` | 自行設計的參數化模型 | 01 → 02 → 03 | 在新模型上套用已驗證的管線 |

- **實驗一（`00_test/`）已完成，使用者於 2026-09-29 確認，目前進入實驗二（從 `01_robot_construction/` 開始）。** 實驗一的細節與參考模型的處理方式見 `00_test/CLAUDE.md`，結論見 `00_test/outputs/summary.md`。
- **實驗一的經驗已經整理進實驗二的規格**：機器人設計要求與馬達選型在 `01_robot_construction/CLAUDE.md`；rule-based 的做法與陷阱在 `02_rule_based/CLAUDE.md`；訓練管線（獎勵設計、BC→PPO 權重接續、篩選門檻）在 `03_training_pipeline/CLAUDE.md`。動工前先讀。
- 需求規格：02、03 中的 PROMPT.md 與 CLAUDE.md 是原始版本，已複製一份到 `00_test/rule_based/` 與 `00_test/training_pipeline/`。實驗一只修改複本，原始版本保持不動；實驗一結束後，比較兩者的差異，再決定實驗二要採用哪個版本。
- 實驗一的程式碼要**可以直接移植**：和機器人相關的內容（asset config、關節順序、站姿、致動器參數、觀測／動作定義）集中在 robot 設定檔中，管線程式碼不寫死任何機器人假設。實驗一完成後，把驗證過的程式碼移植到 02、03，只替換 robot 設定，不從頭重寫。
- 實驗一的結論（哪些部分有效、哪些需要調整、各項指標、哪些問題來自參考模型本身）寫在 `00_test/outputs/summary.md`，作為實驗二的起點。**最終成績**：rule-based 成功率 95%／跌倒 0% → PPO 三輪都是 100%／0% → 最終蒸餾模型 99%／0%、推擠測試 100%。

## 機器人模型

- `references/` **僅供參考，不可修改**（權限設定已禁止寫入）。實驗一直接讀取其中的 URDF；實驗二的新模型不需沿用它的設計。
- 實驗二的新模型採**程式參數化 URDF**：由單一參數檔（尺寸、質量、關節限制、致動器）產生 URDF，再轉成 USD。流程見 `/robot-model` skill。
- **實驗二的機器人之後會 3D 列印成實體，在真實環境測試**：設計時就要考慮零件可列印性、馬達固定方式、組裝順序、線材走向與維修性，不能只做「模擬能跑」的幾何。模擬中的質量、扭矩上限、關節範圍都要對應真實零件。
- **零件分開、組裝方式要特別留意**（使用者 2026-09-30 強調）：每個列印件是獨立檔案；零件之間怎麼接（螺絲規格與位置、熱熔螺母、卡榫、定位銷、公差）要在設計時就定義清楚，並寫進該版的組裝說明。
- 實驗二的機器人：每腿 4 DOF（hip roll、hip pitch、knee、ankle pitch）＋ 2 DOF 主動尾巴（yaw、pitch），共 10 顆 Feetech HD-1910-C001 伺服，無上半身。細節見 `01_robot_construction/CLAUDE.md`。
- 座標慣例依照 Isaac Lab：**+X 前進、+Y 左、+Z 上**，SI 單位，角度用 rad。
- 碰撞體使用 box/capsule/sphere 等基本形狀；mesh 只用於外觀。

### 參考模型的已知問題
以下是參考模型（`雙足機器人_髖關節位置修正`）的已知問題。實驗一的處理方式見 `00_test/CLAUDE.md`；實驗二的新設計應避免這些問題：
- 共 10 個關節：每隻腳 4 個（hip roll、hip pitch、knee、ankle pitch）＋軀幹 yaw、pitch 各 1 個。**沒有 hip yaw 與 ankle roll**，轉向和側向平衡能力有限。
- 總重 28.4 kg，但兩隻腳掌各 6.03 kg（合計 42%），質量分布不合理。
- 大腿 0.10 m、小腿 0.09 m，相對身體偏短；膝關節可正反兩向彎曲 ±45°。
- effort／velocity 都是預設值 100，碰撞體全為 mesh，摩擦係數 0.2 偏低。
- 由 Fusion 360 匯出，前進方向為 Y 軸，與 Isaac Lab 的 +X 慣例不同。
- 檢查任何 URDF：`tools\isaacpy.cmd tools\urdf_inspect.py <file.urdf>`（有 warning 時 exit code 為 1）。

## 跨流程介面契約（最重要）

同一個實驗中，rule-based 控制器和訓練出的 policy **必須使用完全相同的觀測／動作空間**，兩邊不可各自定義。
- 觀測、動作、關節順序、控制頻率、動作縮放方式只能定義在該實驗的共用資料夾中：實驗一是 `00_test/shared/`，實驗二是根目錄的 `shared/`。機器人特有的部分放在 `shared/robots/<robot>.py`，rule-based 和訓練程式都從這裡 import。
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

```
references/             # 參考資料（唯讀）：雙足機器人/ 為舊模型，實驗一使用其中的 URDF
00_test/                # 實驗一（已完成，2026-09-29 使用者確認）
  shared/               #   介面契約、環境、指標、影片 HUD；robots/ref_biped.py、robots/usd/（不進 git）
  scripts/              #   轉檔、機器人檢查、環境煙霧測試
  rule_based/           #   規則式控制器（規格：同資料夾的 PROMPT.md 複本）
  training_pipeline/    #   訓練管線（規格：同資料夾的 PROMPT.md 複本）
  outputs/              #   實驗一的產出（不進 git）
01_robot_construction/  # 實驗二：model/v<N>/ 每版的參數、零件、URDF；model/generator/ 產生程式；model/v2_backlog.md
02_rule_based/          # 實驗二：rule_based/ 為控制器套件與腳本（從 00_test/rule_based 移植），params/ 為採用的參數檔
03_training_pipeline/   # 實驗二：training_pipeline/ 為管線程式（從 00_test 移植），三輪完成（2026-10-01）
04_sim2real/            # 實驗二：sim-to-real 強健性訓練（2026-10-06 新增；2026-10-07 模擬測試完成，最終 student v5）
shared/                 # 實驗二的共用模組（從 00_test/shared 移植）：介面、env/、指標、rollout、影片；robots/custom_biped.py
tools/                  # 開發工具（urdf_inspect.py、isaacpy.cmd），兩次實驗共用
outputs/                # 實驗二的產出（不進 git）
```

- **刻意保留的複本**：實驗一的程式（`00_test/shared/`、`00_test/rule_based/`）凍結不動，實驗二移植時複製一份再修改，所以有些檔案目前兩邊內容相同（`rollout.py`、`video_overlay.py`、`eval_videos.py`、`leg_kinematics.py`）。這是為了讓兩次實驗各自可重現，實驗二只改實驗二的複本，不回頭改 `00_test/`。`PROMPT.md` 的複本也是同樣道理。
- 大型產出一律放在 `<產出根目錄>/<stage>/<run_name>/`：實驗一的產出根目錄是 `00_test/outputs/`，實驗二是 `outputs/`。`<stage>` 為 `rule_based`、`training` 或 `sim2real`，run 名稱包含日期，例如 `00_test/outputs/training/iter1_ppo_20260918/`。
- 所有超參數放在 config 檔（dataclass 或 YAML），不要寫死在程式中。每次訓練或評估都保存當下使用的 config。

## 版本控制（實驗二，使用者 2026-09-30 要求）

機器人模型、rule-based、訓練都預期會迭代多次（v1、v2、v3…）。**每個版本的設計與結果都要保留，不可覆蓋。**
- **機器人模型**：每個版本一個資料夾 `01_robot_construction/model/v<N>/`，放該版的 `params.yaml`、產出的 URDF、列印零件檔與 `CHANGELOG`（相對上一版改了什麼、為什麼）。
  產生程式共用，但每版記錄產生時的 git commit，並打 git tag `robot-v<N>`，確保舊版可以重建。
- **下游一律標明機器人版本**：robot 設定、USD、rule-based 參數、訓練產出都帶版本，例如 `outputs/robot_v1/rule_based/<run>/`、`outputs/robot_v1/training/<run>/`。
- **run 資料夾不可覆寫**：run 名稱含日期與時間（例如 `iter1_ppo_20261005_1430`），程式遇到已存在的資料夾要停下來，不可覆蓋。
- 每個機器人版本在 `outputs/robot_v<N>/summary.md` 整理 rule-based 與各輪訓練的比較，跨版本的比較放在根目錄的總表。
- 以上資料夾結構是 2026-09-30 的提案，細節以使用者確認後的版本為準（見 `01_robot_construction/README.md`）。

## 工作準則

- 除非使用者要求，否則不要安裝新的 MCP server 或大型依賴；使用者重視 token 用量。
- 耗時的模擬或訓練使用 `run_in_background` 執行，不要一直輪詢。
- 回報結果時，要附上實際數字（跌倒率、平均行走距離、側向偏移），不可只寫「看起來正常」。
- 使用者要求時才 commit；commit 前確認沒有把 `outputs/` 或大型二進位檔加進去。
- **每個新增的資料夾都要有 `README.md`**，說明用途、內容、做了哪些事、如何執行；內容有變動時同步更新。
- **CAD 用的 Python 環境（實驗二，2026-09-30 使用者同意）**：build123d 不能裝進 Isaac Lab 的環境，改用 uv 建立獨立環境，**全部放在專案目錄內**：
  `tools/bin/uv.exe`（uv 執行檔）、`.uv/`（uv 下載的 Python 與快取，設 `UV_PYTHON_INSTALL_DIR`、`UV_CACHE_DIR`）、`.venv_cad/`（build123d 環境）。三者都不進 git。
  CAD 程式一律用這個環境執行，模擬與訓練仍用 `tools\isaacpy.cmd`。
- Isaac Lab 3.0 的 API 與舊版差異很大（四元數 XYZW、`.torch`、`_index` 寫入方法、巢狀的 URDF 匯入結果），動手前先看 `/isaac-run` skill 的陷阱清單。
