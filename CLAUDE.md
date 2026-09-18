# 雙足機器人行走專案

在 NVIDIA Isaac Sim / Isaac Lab 中設計一台雙足機器人，並訓練其穩定直線行走。
使用者以繁體中文溝通，文件與回覆一律使用繁體中文；程式碼、識別字與 commit message 使用英文。

## 三個開發流程（依序相依）

| 資料夾 | 內容 | 詳細需求 |
|---|---|---|
| `01_robot_construction/` | 機器人模型設計（程式參數化 URDF → USD） | 見下方「機器人模型」 |
| `02_rule_based/` | 規則式行走控制器，作為教師策略 | `02_rule_based/PROMPT.md` |
| `03_training_pipeline/` | Rule-based → BC → PPO → 自我蒸餾 ×3 → 評估影片 | `03_training_pipeline/PROMPT.md` |

- 開始實作 rule-based 或訓練管線前（不論是在 `00_test/` 還是 02、03 中），**必須先完整閱讀對應的 PROMPT.md**；其中「若有不確定之處」列出的項目，不明確時要先問使用者，不可自行假設。
- `00_note.md` 記錄尚未決定的事項：機器人尺寸、高度、自由度，以及訓練目標（步數、行走距離）。這些是阻擋性的決策，遇到時要主動詢問。

## 兩次實驗

上述流程總共執行兩次：

| 實驗 | 資料夾 | 機器人 | 流程 | 目的 |
|---|---|---|---|---|
| 實驗一（測試） | `00_test/` | `references/` 中的參考 URDF（髖關節位置修正版） | 略過建模，直接 rule-based → 訓練 | 確認 rule-based 與訓練管線可行 |
| 實驗二（正式） | `01_`、`02_`、`03_` | 自行設計的參數化模型 | 01 → 02 → 03 | 在新模型上套用已驗證的管線 |

- **目前進行中：實驗一（`00_test/`）。** 完成之前，不要在 01、02、03 中建立程式碼。實驗一的細節與參考模型的處理方式見 `00_test/CLAUDE.md`。
- 需求規格：02、03 中的 PROMPT.md 與 CLAUDE.md 是原始版本，已複製一份到 `00_test/rule_based/` 與 `00_test/training_pipeline/`。實驗一只修改複本，原始版本保持不動；實驗一結束後，比較兩者的差異，再決定實驗二要採用哪個版本。
- 實驗一的程式碼要**可以直接移植**：和機器人相關的內容（asset config、關節順序、站姿、致動器參數、觀測／動作定義）集中在 robot 設定檔中，管線程式碼不寫死任何機器人假設。實驗一完成後，把驗證過的程式碼移植到 02、03，只替換 robot 設定，不從頭重寫。
- 實驗一的結論（哪些部分有效、哪些需要調整、各項指標、哪些問題來自參考模型本身）寫在 `00_test/outputs/summary.md`，作為實驗二的起點。

## 機器人模型

- `references/` **僅供參考，不可修改**（權限設定已禁止寫入）。實驗一直接讀取其中的 URDF；實驗二的新模型不需沿用它的設計。
- 實驗二的新模型採**程式參數化 URDF**：由單一參數檔（尺寸、質量、關節限制、致動器）產生 URDF，再轉成 USD。流程見 `/robot-model` skill。
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
00_test/                # 實驗一（進行中）
  shared/               #   介面契約、環境、指標、影片 HUD；robots/ref_biped.py、robots/usd/（不進 git）
  scripts/              #   轉檔、機器人檢查、環境煙霧測試
  rule_based/           #   規則式控制器（規格：同資料夾的 PROMPT.md 複本）
  training_pipeline/    #   訓練管線（規格：同資料夾的 PROMPT.md 複本）
  outputs/              #   實驗一的產出（不進 git）
01_robot_construction/  # 實驗二：model/ 放參數化模型與產出的 urdf/usd
02_rule_based/          # 實驗二
03_training_pipeline/   # 實驗二
shared/                 # 實驗二的共用模組（robots/custom_biped.py）
tools/                  # 開發工具（urdf_inspect.py、isaacpy.cmd），兩次實驗共用
outputs/                # 實驗二的產出（不進 git）
```

- 大型產出一律放在 `<產出根目錄>/<stage>/<run_name>/`：實驗一的產出根目錄是 `00_test/outputs/`，實驗二是 `outputs/`。`<stage>` 為 `rule_based` 或 `training`，run 名稱包含日期，例如 `00_test/outputs/training/iter1_ppo_20260918/`。
- 所有超參數放在 config 檔（dataclass 或 YAML），不要寫死在程式中。每次訓練或評估都保存當下使用的 config。

## 工作準則

- 除非使用者要求，否則不要安裝新的 MCP server 或大型依賴；使用者重視 token 用量。
- 耗時的模擬或訓練使用 `run_in_background` 執行，不要一直輪詢。
- 回報結果時，要附上實際數字（跌倒率、平均行走距離、側向偏移），不可只寫「看起來正常」。
- 使用者要求時才 commit；commit 前確認沒有把 `outputs/` 或大型二進位檔加進去。
- **每個新增的資料夾都要有 `README.md`**，說明用途、內容、做了哪些事、如何執行；內容有變動時同步更新。
- Isaac Lab 3.0 的 API 與舊版差異很大（四元數 XYZW、`.torch`、`_index` 寫入方法、巢狀的 URDF 匯入結果），動手前先看 `/isaac-run` skill 的陷阱清單。
