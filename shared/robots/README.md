# shared/robots：實驗二的機器人設定

| 檔案 | 內容 |
|---|---|
| `__init__.py` | `get_robot(name)`：管線程式用名稱選擇機器人 |
| `custom_biped.py` | 尾巴雙足機器人：關節順序（腿 8＋尾巴 2）、站姿、幾何、關節極限、致動器（HD-1910，`DCMotorCfg`）、`make_articulation_cfg()` |
| `usd/` | 轉檔產出，不進 git：`usd/custom_biped_v<N>/` |

- **所有數字都讀自 `01_robot_construction/model/v<N>/`**（params.yaml、reports/mass_properties.json、reports/static_check.json），這裡不重複填寫。
- 版本由 `ROBOT_VERSION` 決定（預設 1，可用環境變數 `CUSTOM_BIPED_VERSION` 覆寫）；下游產出放在 `outputs/robot_v<N>/`。
- 管線需要的欄位和實驗一的 `00_test/shared/robots/README.md` 相同，另外新增：

| 欄位 | 用途 |
|---|---|
| `ACTION_JOINT_NAMES` | 觀測／動作的關節順序（腿 8 ＋ 尾巴 2）；實驗二的管線用它取代 `LEG_JOINT_NAMES` |
| `TAIL_JOINT_NAMES`、`TAIL_JOINT_LIMITS`、`ACTUATED_JOINT_NAMES` | 尾巴關節 |
| `ENV_DEFAULTS` | 和機器人大小有關的環境設定（指令範圍、跌倒高度、排列間距、鏡頭、接觸力門檻…） |
| `STANCE_LEG_LENGTH`、`STANCE_FOOT_X`、`MAX_STEP_X`、`MAX_LIFT` | rule-based 步態參數的預設值（讀自 `reports/static_check.json`） |
| `SERVO_MAP` | 實機用：馬達 ID、匯流排、位置值與關節角的方向（讀自 `reports/servo_map.json`） |
| `MIRROR_SIGN` | 左右鏡像時各關節角的正負號（`shared/symmetry.py` 使用，2026-10-06 新增） |

- `LEG_EFFORT_LIMIT` 是 DCMotor 實際能輸出的上限 0.8 N·m（連續值），堵轉值 1.18 N·m 只用在扭矩–速度曲線（2026-10-01 修正）。
- 關節正方向（模擬中已驗證，`outputs/robot_v1/01_model/robot_check/`）：hip_roll + 腳往左、hip_pitch／knee + 往後、ankle + 腳尖往下、tail_yaw + 尾尖往右、tail_pitch + 尾尖往上。所有 `JOINT_AXIS_SIGN` 都是 +1。

重新產生 USD：`tools\isaacpy.cmd 01_robot_construction\model\generator\convert_usd.py --version 1`
