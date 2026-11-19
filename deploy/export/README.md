# deploy/export：把模型匯出給芒果磚

把 sim2real 的最終學生模型（robot v2：student v2ft）轉成 C 標頭檔，給 [`deploy/firmware`](../firmware/README.md) 編譯；同時產生「標準答案」測試資料，用來確認 C 程式和 PyTorch／模擬完全一致。
這裡的程式用 `tools\isaacpy.cmd` 執行（需要 torch；`record_golden.py` 需要 Isaac 模擬）。實機不需要這個資料夾。

## 產出（`outputs/robot_v2/deploy/export_<模型 run>_<時間>/`，不覆寫）

| 檔案 | 內容 |
|---|---|
| `robot_config.h` | 關節順序、伺服 ID／方向／零點（來自 `deploy/calibration/robot_v2.json`）、預設站姿、關節極限、動作縮放、控制週期、IMU 軸向 |
| `policy_int8.h` | **韌體預設**：int8 權重（每個輸出列一個縮放值）＋輸入正規化，約 276 KB，放進 SRAM |
| `policy_fp32.h` | float32 權重（1.09 MB，只能放 flash；比較用） |
| `golden_mlp.bin` | 256 組輸入 → PyTorch 輸出（fp32 與 int8 兩種） |
| `golden_rollout.bin` | 模擬錄下的原始讀值、學生觀測與模型輸出（站 1 秒 → 走 4 秒，4 台） |
| `student_int8.pt` | 權重換成 int8 還原值的模型，用來在模擬中確認量化後步態不變 |
| `meta.json` | 來源檔案與 SHA256、維度、量化誤差 |

**為什麼用 int8**：RP2350 只有 520 KB SRAM。float32 權重 1.09 MB 只能放 flash，每 20 ms 從 QSPI flash 讀一遍約需 30 ms，超過控制週期；int8 權重 276 KB 可以整個放進 SRAM。

## 執行

```powershell
set CUSTOM_BIPED_VERSION=2
tools\isaacpy.cmd deploy\export\export_policy.py
tools\isaacpy.cmd deploy\export\record_golden.py --export outputs\robot_v2\deploy\export_<...>
# int8 模型的模擬閉迴路檢查（和 v2ft 的評估相同，最後用 rescore.py 判定）
tools\isaacpy.cmd control\sim2real\evaluate.py --scenario flat_v1 --agent student:outputs\robot_v2\deploy\export_<...>\student_int8.pt --tag v2s1_int8
```

改了校正檔（`deploy/calibration/robot_v2.json`）就重新執行 `export_policy.py`，再重新編譯韌體。

## 紀錄（2026-11-19）

- 匯出 student v2ft：維度 210 → 512 → 256 → 128 → 10，正規化公式 `(x − mean) / (std + 0.01)`（rsl_rl `EmpiricalNormalization`）。
- int8 和 float32 的動作差：最大 0.025、平均 0.0038（標準化動作單位），換算成關節目標最多 0.36°。
- 標準答案：4 台機器人站 1 秒、以 0.10 m/s、1.6 Hz 走 4 秒，都走了 0.44–0.45 m，無跌倒。
- int8 模擬閉迴路結果見 [`docs/results.md`](../../docs/results.md)。
