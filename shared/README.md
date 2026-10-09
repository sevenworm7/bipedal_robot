# shared：實驗二的共用程式

rule-based 與訓練**只能**從這裡取得介面定義，不可各自定義（見根目錄 CLAUDE.md「跨流程介面契約」）。
程式從 `00_test/shared/` 移植（2026-10-01），只替換 robot 設定；機器人專屬的數值只放在 `robots/<robot>.py`。

| 檔案 | 內容 |
|---|---|
| `interface.py` | **介面契約**：觀測、動作、控制頻率（50 Hz）、動作縮放、canonical 座標系（02 與 `03_training/training_pipeline` 使用，44 維） |
| `interface_v2.py` | 介面 v2（`03_training/sim2real`，2026-10-06）：特權 teacher 觀測 55 維（v1 ＋ 摩擦、質量、致動器、延遲、地形、著地等 11 維，也是 critic 輸入）；可部署 student 觀測 5 × 42 = 210 維（去掉 base 線速度、加離地高度指令、5 步歷史）。動作與關節順序同 v1 |
| `symmetry.py` | 左右鏡像（`Mirror`）：觀測與動作的 y → −y 對應，供 PPO 的對稱資料增強／鏡像損失與 student 蒸餾使用；符號來自 robot 模組的 `MIRROR_SIGN` |
| `metrics.py` | 每個 episode 的指標，以及 rule-based 的成功判定 `rule_based_success` |
| `rollout.py` | 平行 rollout 與統計（`run_first_episodes`、`aggregate`、分組統計、扭矩百分位、推擠測試） |
| `video_overlay.py`、`eval_videos.py` | 評估影片的 HUD 與錄影（格式依 `/eval-video`），rule-based 與訓練共用 |
| `env/` | Isaac Lab 直線行走環境 `BipedWalkEnv`（gym id：`Biped-Walk-v0`）、隨機化、地形、帶版本的場景（含 04 的壓力測試條件），見 [`env/README.md`](env/README.md) |
| `robots/` | 機器人設定（`custom_biped.py`）與轉出的 USD，見 [`robots/README.md`](robots/README.md) |

## 介面摘要（v1，custom_biped，詳細說明在 `interface.py` 的 docstring；v2 見 `interface_v2.py`）

- 觀測 44 維：base 線速度(3)、角速度(3)、重力投影(3)、速度指令(1)、步頻指令(1)、方向誤差(1)、
  關節角度(10)、關節角速度(10)、上一次動作(10)、步態相位 sin/cos(2)；SI 單位，不做正規化。
- 動作 10 維：`q_target = q_default + 0.25 × action`，action clip 到 ±5。
- 關節順序：robot 模組的 `ACTION_JOINT_NAMES`，腿 8（左 hip_roll、hip_pitch、knee、ankle_pitch，右腳同順序）＋ `tail_yaw`、`tail_pitch`。
- 結構與順序規則和實驗一相同（實驗一 J = 8 → 38 維），差別只有關節數。

## 匯入方式

腳本把專案根目錄加入 `sys.path`，再用 `import shared...` 匯入。`env/` 與 `robots/*.make_articulation_cfg()` 需要先啟動 Isaac Sim（`AppLauncher`）。
