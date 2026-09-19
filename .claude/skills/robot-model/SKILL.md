---
name: robot-model
description: Workflow for turning a URDF into a working Isaac Lab robot — designing the parametric custom model (exp2, 01_robot_construction), or wrapping the read-only reference URDF (exp1, references/) — covering validation, USD conversion, ArticulationCfg in <experiment>/shared/robots/, and in-sim sanity checks. Use whenever creating or modifying any robot asset.
---

# 機器人模型流程

- **實驗一（`ref_biped`）**：已完成（2026-09-18）。URDF 不可修改，所有修正都寫在 `00_test/shared/robots/ref_biped.py`；轉檔與檢查使用 `00_test/scripts/convert_ref_biped.py` 和 `check_ref_biped.py`。細節見 `00_test/CLAUDE.md`。實驗二可以直接參考這兩個腳本。
- **實驗二（`custom_biped`）**：完整執行步驟 0–5，robot 設定放在根目錄的 `shared/robots/custom_biped.py`。`references/` 只能讀取，不需沿用其中的設計。

## 0. 先確認規格（`00_note.md` 中尚未決定）
動工前，先向使用者確認或提出建議：
- 身高／腿長、總重，以及各部位的質量分配（腳掌應 ≤ 總重 5–8%）
- 每隻腳的自由度：建議 6（hip yaw/roll/pitch、knee、ankle pitch/roll），最少 5；是否需要上半身
- 致動器規格（最大扭矩、最大速度），需對應實際可取得的馬達
- 膝關節只能單向彎曲

## 1. 檔案結構（`01_robot_construction/model/`）
```
params.yaml        # 唯一的參數來源：尺寸、質量、關節限制、致動器、PD 增益
build_urdf.py      # params.yaml -> urdf/biped.urdf（基本形狀碰撞體，由形狀和質量計算慣量）
urdf/biped.urdf    # 產出檔，禁止手動修改
usd/biped.usd      # 由 URDF 轉換產出
```
- 慣量由幾何形狀和質量計算（box/cylinder 公式），不要手動填數值。
- 座標慣例：+X 前進、+Y 左、+Z 上；base 原點在骨盆中心；關節零位代表直立站姿。
- 關節命名固定且左右對稱：`{left,right}_{hip_yaw,hip_roll,hip_pitch,knee,ankle_pitch,ankle_roll}`，順序與 `shared/` 中的定義一致。

## 2. 驗證
```powershell
tools\isaacpy.cmd 01_robot_construction\model\build_urdf.py
tools\isaacpy.cmd tools\urdf_inspect.py 01_robot_construction\model\urdf\biped.urdf
```
要求 warning 為 0（除非使用者同意保留），並檢查總重、腿長、關節範圍是否符合規格。

## 3. 轉成 USD（輸出到 `<實驗根目錄>/shared/robots/usd/<robot>/`）
使用 Python 的 `UrdfConverter` 轉檔（範本：`00_test/scripts/convert_ref_biped.py`）。Isaac Lab 3.0 的轉換器輸出路徑是**資料夾**，檔名由 robot 名稱自動決定：`<usd_dir>/<robot_name>/<robot_name>.usda`。
- 致動器的 PD 增益由 Isaac Lab 的 `ArticulationCfg.actuators` 控制，轉檔時設為 0。
- 碰撞體：`collision_type="Convex Hull"`（基本形狀碰撞體會直接保留）。
- 路徑含非 ASCII 字元時，先複製到 ASCII 路徑再轉。
- 轉出的 link 會巢狀排列，會影響 `fix_root_link` 與接觸感測器，見 `/isaac-run` 的陷阱清單。

## 4. 接到 Isaac Lab
- 在 `<實驗根目錄>/shared/robots/<robot>.py` 中建立 `ArticulationCfg`，包含 init_state（站姿關節角、base 高度，實驗一還要加上旋轉）和 actuators（`ImplicitActuatorCfg` 或 `DCMotorCfg`；實驗二的數值取自 params.yaml，實驗一需和使用者確認）。
- 同一個檔案中也定義這台機器人的關節順序、站姿與觀測／動作的維度，rule-based 和訓練程式都從這裡 import。
- 管線用到的欄位清單（`LEG_JOINT_LAYOUT`、`JOINT_AXIS_SIGN`、`LEG_JOINT_LIMITS`、腿長等）見 `00_test/shared/robots/README.md`；新機器人必須提供同樣的欄位，管線程式才能不經修改直接使用。
- 可參考 `C:\isaaclab\source\isaaclab_assets\isaaclab_assets\robots\cassie.py` 與 `unitree.py`。

## 5. 模擬中的健全性檢查（每次修改模型後）
範本：`00_test/scripts/check_ref_biped.py`（產出 report.json 與截圖）。
1. 固定 base（每一步寫回 root pose）：記錄零位時各腳的位置，並把每個關節轉 +0.3 rad，確認腳的移動方向（關節正方向）。
2. 放開 base，以 PD 控制維持站姿 5 秒：base 高度的漂移應小於 2 cm、傾斜小於 5°，且不會倒下。
3. 用 Camera 感測器截圖（側視、正視、近距離側視），用 Read 工具檢查外觀與方向。
結果（通過或失敗，以及相關數值）要回報給使用者。
