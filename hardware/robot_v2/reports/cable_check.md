# 腿部馬達線長檢查（v2，自動產生）

由 `generator/check_cable.py --version 2` 產生。插座之間直線距離的最壞值（線跨過的關節掃描整個可動範圍），限制約 120 mm；實際走線會繞過支架，要保留餘量。採用的接法見 `../ASSEMBLY.md` 第 2 節（兩腿都 1→3→4→2，最上面兩顆接回芒果磚）。

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
