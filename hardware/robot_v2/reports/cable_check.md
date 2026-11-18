# 腿部馬達線長檢查（v2，自動產生）

由 `generator/check_cable.py` 產生。距離是插座之間直線距離的最壞值（線跨過的關節掃描整個可動範圍），限制約 120 mm；實際走線會繞過支架，要保留餘量。採用的接法與中文對照見 `../ASSEMBLY.md` 第 2 節。

## 1. 整條匯流排（`params.yaml` 的 `wiring`，方案 B）

`check_cable.py --version 2 --chain`

```
bus wiring (params `wiring`), limit 120 mm: longest segment 109 mm
socket names: the world direction the servo side face points to at zero pose;
the board is its centre point (header position not modelled).
   1. board             [board] -> left_hip_roll     [b(+Y)]  worst    78 mm  ok       (fixed)
   2. left_hip_roll     [a(-Y)] -> tail_yaw          [b(+Y)]  worst    51 mm  ok       (fixed)
   3. tail_yaw          [a(-Y)] -> tail_pitch        [b(+Z)]  worst    49 mm  ok       (tail_yaw -60)
   4. tail_pitch        [a(-Z)] -> right_hip_roll    [b(-Y)]  worst   105 mm  ok       (tail_yaw -50)
   5. right_hip_roll    [a(+Y)] -> right_knee        [a(+X)]  worst   109 mm  ok       (right_hip_pitch 5, right_hip_roll -25)
   6. right_knee        [b(-X)] -> right_ankle_pitch [b(-X)]  worst    55 mm  ok       (right_hip_pitch -5, right_hip_roll -8, right_knee 0)
   7. right_ankle_pitch [a(+X)] -> right_hip_pitch   [b(+X)]  worst   108 mm  ok       (right_hip_pitch 15, right_hip_roll -25, right_knee 20)
   8. right_hip_pitch   [a(-X)] -> left_hip_pitch    [b(+X)]  worst    88 mm  ok       (left_hip_pitch 45, left_hip_roll 25, right_hip_pitch -45, right_hip_roll -25)
   9. left_hip_pitch    [a(-X)] -> left_knee         [b(-X)]  worst    52 mm  ok       (left_hip_pitch -15, left_hip_roll -8)
  10. left_knee         [a(+X)] -> left_ankle_pitch  [b(-X)]  worst    60 mm  ok       (left_hip_pitch -5, left_hip_roll 17, left_knee 30)
```

## 2. 單腿接線順序的比較（設計用參考）

`check_cable.py --version 2`

```
thigh 70 mm, shin 55 mm, hip_roll axis height 184 mm
  servo 1 hip_roll    sockets: a(-Y) (-7, 20, 193), b(+Y) (-7, 40, 193)
  servo 2 hip_pitch   sockets: a(-X) (-10, 23, 141), b(+X) (10, 23, 141)
  servo 3 knee        sockets: a(+X) (10, 23, 89), b(-X) (-10, 23, 89)
  servo 4 ankle_pitch sockets: a(+X) (10, 23, 34), b(-X) (-10, 23, 34)

  fold_back_1342: longest segment 109 mm
    1->3  a(-Y)      -> b(-X)       zero pose   104  worst   109 mm  (hip_roll 25, hip_pitch -22)
    3->4  a(+X)      -> b(-X)       zero pose    59  worst    60 mm  (hip_roll 17, hip_pitch -13, knee 37)
    4->2  a(+X)      -> b(+X)       zero pose   106  worst   108 mm  (hip_roll -25, hip_pitch 20, knee 18)

  straight_1234: longest segment 61 mm
    1->2  a(-Y)      -> b(+X)       zero pose    55  worst    61 mm  (hip_roll 25, hip_pitch 37)
    2->3  a(-X)      -> b(-X)       zero pose    52  worst    52 mm  (hip_roll -17, hip_pitch -5)
    3->4  a(+X)      -> b(-X)       zero pose    59  worst    60 mm  (hip_roll 17, hip_pitch -13, knee 37)

  2 -> other leg's 1 (cross at hip level), worst 85 mm
  1 -> control board centre (56, 0, 216), worst 70 mm (header position not modelled)
  2 -> control board centre (56, 0, 216), worst 105 mm (header position not modelled)
```
