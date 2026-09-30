# 01_robot_construction/model：實驗二的參數化機器人模型

**狀態：尚未開始**（第一版將是 `v1/`，以 `../design_refs/architecture_v0_1.md` 為起點）。每個版本一個子資料夾 `v<N>/`，保留所有版本，規則見根目錄 `CLAUDE.md`「版本控制」。

預計內容（流程見 `/robot-model` skill）：

| 檔案 | 用途 |
|---|---|
| `params.yaml` | 唯一的參數來源：尺寸、質量、自由度、關節範圍、致動器 |
| `build_urdf.py` | 由參數產生 URDF（基本形狀碰撞體，慣量由形狀和質量計算） |
| `urdf/biped.urdf` | 產出檔，禁止手動修改 |
| `usd/` | 轉換出的 USD，不進 git |

設計時參考實驗一的結論（`00_test/outputs/summary.md`），以及參考模型的已知問題（根目錄 `CLAUDE.md`）。
