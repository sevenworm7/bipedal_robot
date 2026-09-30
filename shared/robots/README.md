# shared/robots：實驗二的機器人設定

| 檔案 | 內容 |
|---|---|
| `__init__.py` | `get_robot(name)`：管線程式用名稱選擇機器人 |
| `custom_biped.py` | 尾巴雙足機器人：關節順序（腿 8＋尾巴 2）、站姿、幾何、關節極限、致動器（HD-1910，`DCMotorCfg`）、`make_articulation_cfg()` |
| `usd/` | 轉檔產出，不進 git：`usd/custom_biped_v<N>/` |

- **所有數字都讀自 `01_robot_construction/model/v<N>/`**（params.yaml、reports/mass_properties.json、reports/static_check.json），這裡不重複填寫。
- 版本由 `ROBOT_VERSION` 決定（預設 1，可用環境變數 `CUSTOM_BIPED_VERSION` 覆寫）；下游產出放在 `outputs/robot_v<N>/`。
- 管線需要的欄位和實驗一的 `00_test/shared/robots/README.md` 相同；多了 `TAIL_JOINT_NAMES`、`TAIL_JOINT_LIMITS`、`ACTUATED_JOINT_NAMES`，
  以及實機用的 `SERVO_MAP`（馬達 ID、匯流排、位置值與關節角的方向，讀自 `reports/servo_map.json`）。
- 關節正方向（模擬中已驗證，`outputs/robot_v1/robot_check/`）：hip_roll + 腳往左、hip_pitch／knee + 往後、ankle + 腳尖往下、tail_yaw + 尾尖往右、tail_pitch + 尾尖往上。所有 `JOINT_AXIS_SIGN` 都是 +1。

重新產生 USD：`tools\isaacpy.cmd 01_robot_construction\model\generator\convert_usd.py --version 1`
