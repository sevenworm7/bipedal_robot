# 尾巴平衡的原理與文獻整理

2026-09-29 建立；同日修訂（見第 1 節的「修訂說明」）。為了決定尾巴的自由度配置、長度、配重與控制方式而做的調查。
結論摘要在 `../README.md`，實際擺放與尺寸在 `architecture_v0_1.md`。

## 1. 尾巴平衡側向的兩個機制

判斷「會不會往旁邊倒」看的是 **ZMP（零力矩點）是否落在支撐腳掌內**。對點質量來說：

```
ZMP_y = Σ m_i · ( g · y_i  −  z_i · ÿ_i ) / (M · g)
```

尾巴質量 `m_t` 在高度 `z_t`、側向位置 `y_t` 時，對 ZMP 的貢獻有兩項：

1. **靜態配重**：`m_t · y_t / M`。尾巴擺到哪一側，重心就往那一側移。
   yaw 軸是垂直的，所以**把尾巴停在側邊不需要馬達出力**（重力平行於轉軸）→ 這個效果**可以持續任意久**，
   例如單腳站立時把尾巴擺到支撐腳那一側。
2. **動態擺動**：尾巴以步態頻率做正弦擺動 `y_t = A·sin(ωt)` 時，`ÿ_t = −ω²·y_t`，第二項變成 `+ z_t·ω²·y_t`，
   **和靜態項同號** → ZMP 位移被放大為 `(1 + ω²·z_t / g)` 倍。
   尾巴在髖部高度（`z_t ≈ 0.25 m`）、步頻 1.5 Hz 時放大約 **3.2 倍**。
   這就是 Coelophysis 模擬中「尾巴左右擺動、作用類似人類擺臂」的機制。

→ **尾巴平衡側向的主要方式是「左右（yaw）擺動」**，擺到支撐腳那一側。

兩項都和尾巴的**水平距離 × 質量**（一次矩 `Σ m·r`）成正比，動態項還和尾端**離地高度**成正比
→ **尾巴中立姿態應該接近水平或略微上揚**，不要往下垂。

### 修訂說明（2026-09-29）
初版筆記寫了「尾巴只能提供有限時間的力矩脈衝、無法對抗持續的重力力矩」以及「中立俯仰角建議 30–45° 向下」。
兩者都不對：初版只算了尾巴繞自身根部的角動量（反作用力矩），漏掉了尾巴質量本身造成的重心與 ZMP 移動。
用 ZMP 重算後：側向效果以**靜態配重＋動態擺動**為主，而且**水平尾巴效果最大**，向下垂反而會同時削弱兩項。

## 2. 尾巴的其他用途

- **轉向（yaw 反作用力矩）**：尾巴繞垂直軸加速時，機體受到反向的 yaw 力矩，可以取代 `hip_yaw`。
- **前後俯仰（pitch）**：尾巴上下加速產生 pitch 反作用力矩，用於起步、停止、著地時的前後平衡（袋鼠、蜥蜴的機制）。
- **暫態擾動**：被推時快速甩尾，在 0.1–0.3 s 內把身體轉回來。

## 3. 生物的做法

| 生物 | 尾巴的角色 | 對本專案的啟示 |
|---|---|---|
| 蜥蜴（壁虎、飛蜥） | 起跳後主動擺尾控制俯仰角；有主動尾巴的機器人比剛性尾巴／無尾巴轉動更少 | 主動控制 > 被動配重。尾巴要進觀測與動作空間 |
| 獸腳類恐龍（Coelophysis 模擬） | 尾巴不是靜態配重，而是**左右擺動（lateroflexion）**，被動調節角動量、降低能耗，作用類似人類走路的擺臂 | 尾巴的主要平面可能是**水平（yaw）**而不是垂直，和「恐龍尾巴是配重」的直覺相反 |
| 袋鼠 | 跳躍時身體前後俯仰，水平後伸的尾巴當動態配重；站立時當第五肢支撐 | 尾巴的中立姿態接近水平後伸；若要當支撐腳則需要接地，本專案初版不做 |
| 獵豹／貓 | 高速轉彎時用尾巴產生 yaw 反作用力矩 | yaw 尾巴的轉向效果最直接 |

## 4. 機器人文獻

- **Libby et al., "Tail-assisted pitch control in lizards, robots and dinosaurs", Nature 2012**：
  蜥蜴大小的主動尾巴機器人，靠感測回授穩定俯仰角。證明「主動擺尾的身體，轉動量小於剛性尾巴／柔順尾巴／無尾巴」。
- **Johnson et al., "Comparative Design, Scaling, and Control of Appendages for Inertial Reorientation" (arXiv:1511.05958)**：
  比較各種慣性重定向附肢（尾巴、反作用輪、腿的擺動），結論是**在合理的設計範圍內，尾巴的性能上限最高**
  （同樣質量下，尾巴把質量放在最遠處，`I = m r²` 最有效率）。
- **Penn Jerboa**：2 DOF 尾巴的雙足機器人，已實作 roll 方向的穩定行為。
- **Liu & Ben-Tzvi, "Maneuvering and stabilization control of a bipedal robot with a universal-spatial robotic tail" (Bioinspir. Biomim. 2019)**：
  尾巴協助雙足機器人做 yaw 轉向與穩定抬腿。用 ZMP 規劃尾巴軌跡，在不穩定期間把 ZMP 拉回支撐多邊形內。
- **3-DoF morphable inertial tail（CUHK BMT Lab、arXiv:2209.15337）**：可變慣量／可伸縮尾巴，
  pitch −90°~180°、yaw ±180°、長度 0.12–0.49 m。對應概念圖「後續發展：伸縮尾巴」。

## 5. 粗估：這條尾巴夠力嗎？（以 `architecture_v0_1.md` 的 v0.1 數字）

總重 0.67 kg、尾巴長 250 mm（tail_pitch 軸到尾端配重中心）、尾端配重 40 g、碳管 15 g、
尾巴水平、位於離地約 0.25 m、步態頻率 1.5 Hz（`ω² ≈ 89 s⁻²`）。

| 項目 | 擺幅 ±30° | 擺幅 ±45° | 說明 |
|---|---|---|---|
| 尾巴一次矩 `Σ m·r`（對 yaw 軸） | 0.0155 kg·m | 同左 | 決定兩個機制的強度 |
| 靜態重心側移 | 11.6 mm | 16.4 mm | 停住不動時就有 |
| 步行時的等效 ZMP 側移 | **~37 mm** | **~53 mm** | 靜態 × 3.2 |
| 單腳支撐需要的側移（髖距 60 mm） | ~30 mm | | 見 `leg_design_options.md` |
| tail_yaw 需要的峰值扭矩 `I·ω²·A` | 0.19 N·m（16%） | 0.28 N·m（24%） | 括號為佔 HD-1910 堵轉 1.18 N·m（6 V）的比例（暫定 XL330 時是 36%／53%） |
| 單腳支撐時腳底能抵抗的 yaw 摩擦力矩 | ~0.14 N·m | | μ 0.8、腳掌 90 × 55 mm |

判讀：
- **側向平衡能力充足**：±30° 的擺動就足以提供單腳支撐需要的 ZMP 側移（粗估，需模擬驗證）。
- **扭矩限制擺幅**：換成 HD-1910 後，±45° 也在堵轉的 30% 以內；限制擺幅的主要因素變成下一點的腳底打滑。步頻越高，需要的扭矩越大（∝ ω²）。
- **新的風險：yaw 反作用力矩可能讓支撐腳原地打滑旋轉**。±30° 時 0.19 N·m 已經超過腳底摩擦上限 0.14 N·m。
  擺腿本身也會產生 yaw 力矩，兩者相位適當時可以互相抵消（這正是人類擺臂的作用），這要交給 rule-based 的相位搜尋與 PPO 處理。
  模擬時要記錄支撐腳的 yaw 滑移量。
- **tail_pitch 持續受重力負載**：尾巴水平時約 0.12 N·m（23%）。可以接受，但實體可加彈簧或橡皮筋抵消。
- **前後重心**：尾巴一次矩同樣會把重心往後拉約 30 mm，所以軀幹要往前延伸、腳踝要放在髖關節後方（恐龍式配置），見 `architecture_v0_1.md`。

**提高尾巴效果最有效的順序**：加長尾巴（`Σ m·r` 與慣量都變大）＞ 加重尾端 ＞ 提高尾巴離地高度。
但**加長與加重都會同時增加 yaw 峰值扭矩、腳底打滑風險與往後的重心偏移**，三者要一起看。

## 6. 對本專案的具體建議

1. **尾巴的左右擺動（yaw）是主要工作**：步行時擺到支撐腳那一側，同時處理側向平衡與轉向。
2. **尾巴的上下（pitch）主要用來設定並維持高度**（中立 0° 水平～略微上揚），以及起步、停止時的前後平衡。
3. 尾巴長度、尾端配重、中立俯仰角都放進 `params.yaml`，作為之後可以掃描的設計參數。尾端配重做成可更換（20–80 g）。
4. 觀測空間加入尾巴的關節角與角速度（4 維），動作空間加入 2 維，沿用實驗一的結構。
5. **rule-based 階段就要用尾巴**：`tail_yaw` 跟著步態時鐘做正弦擺動，相位對齊支撐腳，振幅與相位偏移交給 CEM 搜尋。
6. 模擬時額外記錄：支撐腳 yaw 滑移、tail_yaw／tail_pitch 扭矩、ZMP 軌跡。

## 來源

- [Tail-assisted pitch control in lizards, robots and dinosaurs (Nature 2012)](https://polypedal.berkeley.edu/publications/106_Libby_TailAssistedPitchControl_Nature_2012.pdf)
- [Comparative Design, Scaling, and Control of Appendages for Inertial Reorientation (arXiv:1511.05958)](https://arxiv.org/abs/1511.05958)
- [Maneuvering and stabilization control of a bipedal robot with a universal-spatial robotic tail (IOP)](https://iopscience.iop.org/article/10.1088/1748-3190/aaf188)
- [Design of a Bipedal Hopping Robot With Morphable Inertial Tail (CUHK BMT Lab)](https://biomedirobotics.com/design-of-a-bipedal-hopping-robot-with-morphable-inertial-tail-for-agile-locomotion/)
- [Towards Safe Landing of Falling Quadruped Robots Using a 3-DoF Morphable Inertial Tail (arXiv:2209.15337)](https://arxiv.org/pdf/2209.15337)
- [Predictive simulations of running gait reveal a critical dynamic role for the tail in bipedal dinosaur locomotion (Science Advances)](https://pmc.ncbi.nlm.nih.gov/articles/PMC8457660/)
- [Kangaroo Tails: The Fifth Limb for Balance and Movement](https://asmr.education/faq/kangaroos/kangaroo-tail-fifth-limb)
- [Feetech HD-1910-C001 Coreless Servo（OpenELAB）](https://openelab.com/products/feetech-hd1910-microduck-coreless-servo)
- [Feetech HD1910M 規格（Feixiang RC）](https://lxrcmodel.com/product/feetech-hd1910m-6v-12kg-metal-gear-coreless-ttl-serial-bus-servo-diy-microduck-open-source-robot-actuator/)
