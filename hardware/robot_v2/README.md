# 機器人模型 v2（2026-11-18）

v1 縮短腿部的版本。起因是實機的馬達線太短（原廠線 16 cm，插座之間最多約 12 cm），v1 的腿太長，只能讓線跨過兩腳腳踝（腳鍊）。
v2 只改大腿、小腿和膝關節範圍，讓「折返接法」的每一段線都在 12 cm 以內。修改與原因見 [`CHANGELOG.md`](CHANGELOG.md)，
從 v1 實機改裝的步驟與接線見 [`ASSEMBLY.md`](ASSEMBLY.md)，計算過程見 [`../v2_backlog.md`](../v2_backlog.md)。

**狀態：已凍結（2026-11-18，tag `robot-v2`、`FROZEN` 檔）。** hardware 端檢查、USD 轉檔與模擬檢查都通過；generator 拒絕重新產生這一版。
芒果磚的實際尺寸、質量和列印件秤重還是估計值，之後要修正就開 v3。

互動 3D 圖：直接開啟 [`viewer.html`](viewer.html)（完全離線）。整機組裝檔：`parts/assembly_v2.step`。

## 規格摘要（和 v1 不同的項目加粗）

| 項目 | 數值 |
|---|---|
| 自由度 | 每腿 4（hip_roll、hip_pitch、knee、ankle_pitch）＋尾巴 2（tail_yaw、tail_pitch） |
| 馬達 | Feetech HD-1910-C001 雙出軸 × 10 |
| 總重 | **0.670 kg**（v1 0.696 kg） |
| 站姿 | **hip_pitch −3.6°**、knee 34°、**ankle_pitch −30.4°**；**base 離地 176 mm、重心高 144 mm** |
| 腿 | 髖距 60 mm、hip_roll→hip_pitch 34 mm、**大腿 70、小腿 55**、踝高 25 mm；腳掌 90 × 55 mm |
| 膝關節 | **0～110°**（v1 0～120°） |
| 尾巴 | 尾長 250 mm、尾端 40 g、碳管 Ø8（不變） |
| 軀幹 | 127 × 86 × 58 mm（不變） |

## 內容

| 檔案 | 說明 |
|---|---|
| `params.yaml` | **唯一的參數來源**（從 v1 複製後修改） |
| `CHANGELOG.md` | 相對 v1 的修改、原因與結果比較 |
| `ASSEMBLY.md` | 從 v1 改裝：重印哪些零件、接線順序、關節範圍 |
| `viewer.html` | 互動 3D 圖（本機版） |
| `parts/stl/`、`parts/step/` | 列印零件；**只有 `thigh`、`shin` 需要重印**，其他和 v1 相同 |
| `meshes/`、`urdf/custom_biped_v2.urdf` | URDF 與外觀 mesh（產生檔，不要手動修改） |
| `reports/cable_check.md` | **v2 新增**：整條匯流排（`wiring`）每一段的最壞插座距離與建議插座，以及單腿接線順序的比較 |
| `reports/parts_bom.md`、`interference.md`、`fasteners.md`、`servo_map.md`、`static_check.md` | 同 v1 |

## 檢查結果（2026-11-18）

- **線長**：一條線串接全部 10 顆馬達（方案 B：芒果磚 → 左1 → 尾巴 yaw → 尾巴 pitch → 右1 → 右3 → 右4 → 右2 → 左2 → 左3 → 左4），
  10 段全部在約 120 mm 以內，最長 109 mm（v1 的腿要折返需要 162 mm）。順序在 `params.yaml` 的 `wiring`，接法見 `ASSEMBLY.md` 第 2 節。
- **干涉**：12 組關節掃描、同一連桿內、不相鄰連桿、電子部件全部 0 重疊（v1 剩的極端組合碰撞也消失了）。
- **URDF 檢查**：0 warning；11 個 link、10 個致動關節；左右腿鏈長 0.189 m。
- **靜態扭矩**（6 V 堵轉 1.18 N·m）：單腳支撐最大是 hip_roll 0.155 N·m（額定 53%），knee 從 v1 的 0.21 降到 0.15 N·m。
- **可達範圍**：前後 ±67 mm；**原地抬腳最高 24 mm**（v1 56 mm），是 v2 最大的代價。
- **模擬檢查**（Isaac Sim，`outputs/robot_v2/simulation/robot_check/20261118_203336/`）：
  - 10 個關節的正方向都正確（和 v1 相同）；模擬中總重 0.670 kg，和 URDF 一致。
  - 站立 5 秒：高度漂移 0 mm、水平位移 0 mm、最大傾斜 1.1°，通過。
  - 生成落地時有 37 mm 的沉降位移（v1 是 20 mm），站立期間不再移動。

## 已知風險

1. **抬腳高度只剩 24 mm**：v1 調好的規則式步態抬 18.5 mm，還在範圍內，但跨凹凸地面的餘量變小。瓶頸是踝關節 ±60°。
2. **線長餘量約 11 mm**（第 5、7 段），芒果磚那段要看板上接頭位置：直線距離是下限，接線前先在實物上比對（`ASSEMBLY.md` 第 2 節）。
3. 芒果磚的尺寸、質量仍是佔位值；v1 列出的待實物確認項目（`../v2_backlog.md` #2–#4）也還沒回填。
