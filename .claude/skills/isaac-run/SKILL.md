---
name: isaac-run
description: How to launch Isaac Sim / Isaac Lab scripts on this Windows machine — the Python wrapper, headless vs. viewer, cameras/video, training & play commands, TensorBoard, and known warnings. Use before running any simulation, training, evaluation, or Python script in this project.
---

# 在本機執行 Isaac Lab

## Python 入口
系統沒有可用的 `python`。所有指令都在專案根目錄透過 wrapper 執行：

```powershell
tools\isaacpy.cmd path\to\script.py --arg value
tools\isaacpy.cmd -c "import torch; print(torch.cuda.is_available())"
tools\isaacpy.cmd -m pytest tests\test_x.py
```

wrapper 會呼叫 `C:\isaaclab\isaaclab.bat -p`，可透過環境變數 `ISAACLAB_PATH` 覆寫安裝路徑。
- `setup_conda_env.bat is missing` 的警告是無害的。
- 腳本失敗時，最後會多印出 `[ERROR] Command failed ...`；真正的錯誤訊息在這行之前。

## 模擬器旗標（Isaac Lab 3.0 AppLauncher）
- **預設就是 headless**。`--headless` 已不建議使用，不要再加。
- 要開 GUI 視窗：`--viz kit`（CSV 格式，可選 kit、newton、rerun、viser；本機尚未實際測試過 GUI 模式）。
- 需要錄影或相機時：`--enable_cameras`。
- 其他常用旗標：`--device cuda:0`、`--num_envs N`、`--seed S`。
- 自寫的 standalone 腳本必須先建立 `AppLauncher`，**之後才能** import `isaaclab.*` 的其他模組：

```python
import argparse
from isaaclab.app import AppLauncher
parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app = AppLauncher(args).app
# 之後才 import isaaclab.envs / isaaclab.assets ...
```

## 訓練與播放（Isaac Lab 內建腳本）
```powershell
tools\isaacpy.cmd C:\isaaclab\scripts\reinforcement_learning\train.py --rl_library rsl_rl --task <TaskId> --num_envs 4096
tools\isaacpy.cmd C:\isaaclab\scripts\reinforcement_learning\play.py  --rl_library rsl_rl --task <TaskId> --num_envs 16 --viz
```
- 加上 `--video --video_length 300 --enable_cameras` 可以錄影。
- 自訂任務必須先在 gym 註冊，並確保被 import，內建腳本才找得到。

## 長時間任務
- 訓練與大量 rollout 一律使用 Bash/PowerShell 的 `run_in_background`，並把輸出導向 該 run 的輸出資料夾中的 `stdout.log`。
- 開啟 TensorBoard：`tools\isaacpy.cmd -m tensorboard.main --logdir archive\exp1_reference\outputs\training`（實驗二改為 `outputs\robot_v1\control\flat_pipeline` 或 `outputs\robot_v1\control\sim2real`），然後打開 http://localhost:6006。

## Isaac Lab 3.0 / Isaac Sim 6 的陷阱（本機已實際遇到）
- **四元數是 XYZW**（舊版是 WXYZ）。例如 `init_state.rot`、`root_pose[:, 3:7]`。網路上的範例大多還是舊格式。
- `.data.*` 回傳的是 `ProxyArray`，要加上 `.torch` 才能拿到 tensor，例如 `robot.data.joint_pos.torch`。
- 寫入模擬器的方法都分成 `_index` 與 `_mask` 兩種：`write_root_pose_to_sim_index(root_pose=..., env_ids=...)`、`set_joint_position_target_index(target=..., joint_ids=...)`。
- URDF 轉換器輸出 `<dir>/<robot_name>/<robot_name>.usda`，所有 link 會**巢狀**排列在父 link 底下，因此：
  - `fix_root_link=True` 會造成兩個 ArticulationRoot 而報錯 → 改成每一步寫回 root pose
  - `activate_contact_sensors` 只作用在最外層的剛體 → 需要自己遍歷所有剛體並補上 API（見 `archive/exp1_reference/shared/env/biped_env.py`）
  - 用一個 regex 的 ContactSensor 橫跨左右腳時，只抓得到第一隻 → 每隻腳各建一個感測器
- 原始檔的路徑含中文時，先複製到 ASCII 路徑再轉檔。
- `ContactSensor` 是延遲更新的：讀取 `.data` 才會取樣接觸狀態（每個控制步一次）。`compute_first_contact(dt)` 不會觸發更新，且 `contact_time < dt` 的判斷卡在 float32 邊界，會漏掉大部分著地 → 用 `foot_in_contact()` 自己追蹤「離地 → 著地」轉換（見 `archive/exp1_reference/shared/env/biped_env.py` 的 `_update_touchdowns`）。
- 錄影時用獨立的 `Camera` 感測器搭配 `set_world_poses_from_view`；viewer 的 `asset_root` 跟隨模式取景不準。
- 最終以本機原始碼 `C:\isaaclab\docs\source\migration\migrating_to_isaaclab_3-0.rst` 為準。

## 資源
- 實測（`ref_biped`、`Biped-Test-Walk-v0`、headless）：256 個環境約 9,100 env-steps/s，4096 個環境約 95,600 env-steps/s。若出現 OOM，先減少 `num_envs`。
- 開啟 `--enable_cameras` 會大幅增加記憶體用量，錄影時只開少量環境（1–16 個）。
- 可參考的範例：`C:\isaaclab\source\isaaclab_tasks\isaaclab_tasks\manager_based\locomotion\velocity\config\{cassie,h1,g1}`、`C:\isaaclab\scripts\tutorials`。
