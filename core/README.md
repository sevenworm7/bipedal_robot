# core：虛實共用的介面與機器人定義（實驗二）

**模擬與實機都會用到、而且不依賴 Isaac Lab 的程式**放在這裡：觀測／動作的介面契約、左右鏡像、機器人定義（關節順序、站姿、致動器、伺服 ID 對照）。
rule-based、訓練、評估與之後的實機控制程式（`deploy/`）**只能**從這裡取得介面定義，不可各自定義（見根目錄 `CLAUDE.md`「跨流程介面契約」）。
芒果磚韌體是 C 程式，不能 import Python：`deploy/export/export_policy.py` 從這裡讀出關節順序、站姿、動作縮放等，產生 `robot_config.h`；觀測組裝（`deploy/firmware/src/obs.c`）照 `interface_v2.py` 實作，並用模擬錄下的資料逐步比對一致。改了這裡的介面就要重新匯出、重跑 `deploy/firmware/build_host_tests.sh`。
2026-10-15 從原 `shared/` 拆出；模擬專用的部分在 [`simulation/`](../simulation/README.md)。

| 檔案 | 內容 |
|---|---|
| `interface.py` | **介面契約 v1**：觀測 44 維、動作 10 維、控制頻率 50 Hz、動作縮放、canonical 座標系（`control/rule_based`、`control/flat_pipeline` 使用） |
| `interface_v2.py` | **介面 v2**（`control/sim2real`，2026-10-06）：特權 teacher 觀測 55 維（v1 ＋ 摩擦、質量、致動器、延遲、地形、著地等 11 維，也是 critic 輸入）；可部署 student 觀測 5 × 42 = 210 維（去掉 base 線速度、加離地高度指令、5 步歷史）。動作與關節順序同 v1 |
| `symmetry.py` | 左右鏡像 `Mirror`：觀測與動作的 y → −y 對應（PPO 對稱資料增強／鏡像損失、student 蒸餾）；符號來自 robot 模組的 `MIRROR_SIGN` |
| `robots/` | 機器人定義 `custom_biped.py`（數值全部讀自 `hardware/robot_v<N>/`），見 [`robots/README.md`](robots/README.md) |

## 介面摘要（v1，詳細說明在 `interface.py` 的 docstring）

- 觀測 44 維：base 線速度(3)、角速度(3)、重力投影(3)、速度指令(1)、步頻指令(1)、方向誤差(1)、
  關節角度(10)、關節角速度(10)、上一次動作(10)、步態相位 sin/cos(2)；SI 單位，不做正規化。
- 動作 10 維：`q_target = q_default + 0.25 × action`，action clip 到 ±5。
- 關節順序：`ACTION_JOINT_NAMES`，腿 8（左 hip_roll、hip_pitch、knee、ankle_pitch，右腳同順序）＋ `tail_yaw`、`tail_pitch`。
- 結構與順序規則和實驗一相同（實驗一 J = 8 → 38 維），差別只有關節數。
- 實機可部署的是 v2 的 student 觀測（不需要 base 線速度）。

## 匯入方式

腳本把專案根目錄加入 `sys.path`，再用 `from core import interface`、`from core.robots import custom_biped` 匯入。
這裡的模組在匯入時不載入 Isaac Lab；只有 `robots/*.make_articulation_cfg()` 需要先啟動 Isaac Sim（`AppLauncher`）。
