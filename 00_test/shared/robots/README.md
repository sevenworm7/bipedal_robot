# 00_test/shared/robots：機器人設定

| 檔案 | 內容 |
|---|---|
| `__init__.py` | `get_robot(name)`：管線程式用名稱選擇機器人 |
| `ref_biped.py` | 參考機器人：關節順序、站姿、幾何尺寸（供 IK 使用）、致動器參數、`ArticulationCfg` |
| `usd/` | 轉換產出，不進 git。`usd/ref_biped/src/` 放 URDF 的 ASCII 複本，`usd/ref_biped/usd/` 放 USD |

## ref_biped 的修正方式（`references/` 不可修改，所有修正都寫在程式中）

| 問題 | 處理方式 |
|---|---|
| 原始路徑含中文 | 轉檔前先複製到 `usd/ref_biped/src/` |
| 前進方向是 URDF 的 +Y | root 旋轉 −90°（`INIT_ROT_XYZW`，XYZW 順序），讓世界座標 +X 成為前進方向；由參數 `FORWARD_AXIS` 控制 |
| base 原點在腳底 | 微蹲站姿的初始高度由幾何計算（`INIT_BASE_HEIGHT`） |
| effort/velocity 是預設值 100 | 腿部 80 N·m（使用者決定）、20 rad/s、kp 300、kd 10；軀幹 kp 1000 固定在 0 |
| 碰撞體是 mesh | 轉檔時使用凸包（Convex Hull） |

## 已驗證的數值（`scripts/check_ref_biped.py`）

- 關節正方向：髖 roll + 腳往右｜髖 pitch + 腳往後｜膝 + 小腿往後｜踝 + 腳尖往下
- 預設站姿：髖 pitch −0.25、膝 +0.5、踝 −0.25 rad（微蹲，避免膝蓋伸直時的 IK 奇異點）
- 腳掌：長 18 cm（腳踝前 5.5 cm、後 12.5 cm）、寬 12 cm
- 總重 28.4 kg，站立時 base CoM 高度約 0.364 m

## 重新產生 USD

```powershell
tools\isaacpy.cmd 00_test\scripts\convert_ref_biped.py
```
