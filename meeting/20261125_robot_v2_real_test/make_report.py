"""Build report.html from report_data.json (run analyze_data.py first)."""
import json, sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
d = json.load(open(HERE / "report_data.json"))
OUT = HERE / "report.html"
J = list(d["walk_joint"].keys())
JZH = {"left_hip_roll": "左 hip roll", "left_hip_pitch": "左 hip pitch", "left_knee": "左膝", "left_ankle_pitch": "左踝",
       "right_hip_roll": "右 hip roll", "right_hip_pitch": "右 hip pitch", "right_knee": "右膝", "right_ankle_pitch": "右踝",
       "tail_yaw": "尾巴 yaw", "tail_pitch": "尾巴 pitch"}


def r0(x):
    return f"{x:.0f}"


def r1(x):
    return f"{x:.1f}"


# ---------- chart 1: WALK time series, three small multiples sharing x ----------
def ts_chart():
    s = d["walk_series"]
    W, L, R, PH, GAP, T = 760, 56, 16, 120, 34, 10
    panels = [("電壓（V）", 1, 6.0, 8.0, [6.0, 6.5, 7.0, 7.5, 8.0], (6.5, "停機門檻 6.5 V")),
              ("電流暫存器合計（raw）", 2, 0, 3200, [0, 1000, 2000, 3000], None),
              ("最大追隨誤差（°）", 3, 0, 25, [0, 5, 10, 15, 20, 25], (25, "誤差保護 25°"))]
    H = T + len(panels) * (PH + GAP) + 10
    xs = lambda t: L + (W - L - R) * t / 6.0
    g = [f'<svg class="chart" id="ts" viewBox="0 0 {W} {H}" role="img" aria-label="WALK 6 秒期間的電壓、電流與追隨誤差">']
    for k, (name, col, lo, hi, ticks, ref) in enumerate(panels):
        y0 = T + k * (PH + GAP) + 18
        ys = lambda v, y0=y0, lo=lo, hi=hi: y0 + PH - PH * (v - lo) / (hi - lo)
        g.append(f'<text class="ptitle" x="{L}" y="{y0 - 6}">{name}</text>')
        for tk in ticks:
            g.append(f'<line class="grid" x1="{L}" x2="{W - R}" y1="{ys(tk):.1f}" y2="{ys(tk):.1f}"/>'
                     f'<text class="tick" x="{L - 6}" y="{ys(tk) + 4:.1f}" text-anchor="end">{tk:g}</text>')
        if ref:
            g.append(f'<line class="ref" x1="{L}" x2="{W - R}" y1="{ys(ref[0]):.1f}" y2="{ys(ref[0]):.1f}"/>'
                     f'<text class="reflab" x="{W - R}" y="{ys(ref[0]) + (16 if ref[0] < 20 else -5):.1f}" text-anchor="end">{ref[1]}</text>')
        pts = " ".join(f"{xs(p[0]):.1f},{ys(p[col]):.1f}" for p in s)
        g.append(f'<polyline class="line" points="{pts}"/>')
    yb = T + len(panels) * (PH + GAP) - GAP + 18
    for tk in range(7):
        g.append(f'<text class="tick" x="{xs(tk):.1f}" y="{yb + 16}" text-anchor="middle">{tk}</text>')
    g.append(f'<text class="tick" x="{W - R}" y="{yb + 30}" text-anchor="end">WALK 開始後時間（秒）</text>')
    g.append(f'<line class="cross" x1="0" x2="0" y1="{T}" y2="{yb}" visibility="hidden"/>')
    g.append(f'<rect class="hit" x="{L}" y="{T}" width="{W - L - R}" height="{yb - T}"/></svg>')
    return "".join(g), H, yb


# ---------- chart 2: mean max error per voltage bin ----------
def bin_chart():
    b = d["err_bins"]
    keys = list(b.keys())
    W, L, R, T, PH = 760, 56, 16, 24, 220
    n = len(keys)
    bw = (W - L - R) / n
    ys = lambda v: T + PH - PH * v / 20
    g = [f'<svg class="chart" viewBox="0 0 {W} {T + PH + 50}" role="img" aria-label="各電壓區間的平均最大追隨誤差">']
    for tk in [0, 5, 10, 15, 20]:
        g.append(f'<line class="grid" x1="{L}" x2="{W - R}" y1="{ys(tk):.1f}" y2="{ys(tk):.1f}"/>'
                 f'<text class="tick" x="{L - 6}" y="{ys(tk) + 4:.1f}" text-anchor="end">{tk}°</text>')
    for i, k in enumerate(keys):
        cnt, m = b[k]
        x = L + i * bw + 2
        cls = "bar" if cnt >= 3 else "bar thin"
        tip = f"電壓 {k} V：{cnt} 筆，平均最大誤差 {m:.1f}°"
        g.append(f'<g class="mark" data-tip="{tip}"><rect class="{cls}" x="{x:.1f}" y="{ys(m):.1f}" width="{bw - 4:.1f}" '
                 f'height="{PH - (ys(m) - T):.1f}" rx="3"/><rect class="hitbar" x="{x - 2:.1f}" y="{T}" width="{bw:.1f}" height="{PH}"/></g>')
        g.append(f'<text class="tick" x="{x + (bw - 4) / 2:.1f}" y="{T + PH + 16}" text-anchor="middle">{k}</text>')
    g.append(f'<text class="tick" x="{W - R}" y="{T + PH + 36}" text-anchor="end">供電電壓（V，0.1 V 一格）</text></svg>')
    return "".join(g)


# ---------- chart 3: current sum per scenario ----------
def cur_chart():
    rows = [("扭矩關閉", d["off"]["cur_mean"], False), ("懸空站姿", d["hang_cur"]["cur_mean"], False),
            ("桌上站姿", d["table"]["cur_mean"], False), ("WALK 平均", d["walk"]["cur_mean"], True),
            ("WALK 峰值", d["walk"]["cur_max"], True)]
    W, L, R, BH, G = 760, 96, 70, 26, 12
    H = len(rows) * (BH + G) + 10
    xs = lambda v: (W - L - R) * v / 3200
    g = [f'<svg class="chart" viewBox="0 0 {W} {H}" role="img" aria-label="各情境的電流暫存器合計">']
    for i, (name, v, hot) in enumerate(rows):
        y = 6 + i * (BH + G)
        g.append(f'<g class="mark" data-tip="{name}：10 顆電流暫存器絕對值合計 {v:.0f}">'
                 f'<text class="lab" x="{L - 8}" y="{y + BH / 2 + 5}" text-anchor="end">{name}</text>'
                 f'<rect class="{"bar hot" if hot else "bar"}" x="{L}" y="{y}" width="{max(xs(v), 3):.1f}" height="{BH}" rx="4"/>'
                 f'<text class="val" x="{L + max(xs(v), 3) + 6:.1f}" y="{y + BH / 2 + 5}">{v:.0f}</text>'
                 f'<rect class="hitbar" x="0" y="{y - G / 2}" width="{W}" height="{BH + G}"/></g>')
    g.append("</svg>")
    return "".join(g)


ts_svg, ts_h, ts_yb = ts_chart()
w, wj, hng, tab = d["walk"], d["walk_joint"], d["hang"], d["table"]
joint_rows = "".join(
    f"<tr><td>{JZH[j]}</td><td>{r0(wj[j]['cur_mean'])}</td><td>{r0(wj[j]['cur_max'])}</td><td>{r1(wj[j]['err_max'])}</td>"
    f"<td>{r0(wj[j]['vel_p95'])}</td><td>{r0(wj[j]['vel_max'])}</td></tr>" for j in J)
stance_cur_rows = "".join(
    f"<tr><td>{JZH[j]}</td><td>{r0(d['hang_joint_cur'][j])}</td><td>{r0(d['table_joint_cur'][j])}</td><td>{d['table_joint_err'][j]:.2f}</td></tr>"
    for j in J)
series_json = json.dumps([[p[0], p[1], p[2], p[3]] for p in d["walk_series"]])
eb = d["err_by_v"]
ratio = w["cur_mean"] / d["hang_cur"]["cur_mean"]

html = f"""<!doctype html>
<html lang="zh-Hant">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>robot v2 實機測試報告</title>
<style>
:root {{
  color-scheme: light;
  --bg: #f6f5f2; --surface: #fcfcfb; --ink: #0b0b0b; --ink-2: #52514e; --ink-3: #7a7974;
  --rule: #e4e2dc; --grid: #ecebe6; --accent: #2a78d6; --hot: #eb6834; --ref: #a3a29b;
  --warn-bg: #fff4e5; --warn-ink: #7a4a00; --ok-bg: #eaf5ee; --ok-ink: #1d5b33;
}}
@media (prefers-color-scheme: dark) {{
  :root:not([data-theme="light"]) {{
    color-scheme: dark;
    --bg: #121211; --surface: #1a1a19; --ink: #f0efec; --ink-2: #c3c2b7; --ink-3: #94938b;
    --rule: #2e2e2b; --grid: #262624; --accent: #3987e5; --hot: #d95926; --ref: #6d6c66;
    --warn-bg: #33270f; --warn-ink: #f2c879; --ok-bg: #15291d; --ok-ink: #9fd8b1;
  }}
}}
:root[data-theme="dark"] {{
  color-scheme: dark;
  --bg: #121211; --surface: #1a1a19; --ink: #f0efec; --ink-2: #c3c2b7; --ink-3: #94938b;
  --rule: #2e2e2b; --grid: #262624; --accent: #3987e5; --hot: #d95926; --ref: #6d6c66;
  --warn-bg: #33270f; --warn-ink: #f2c879; --ok-bg: #15291d; --ok-ink: #9fd8b1;
}}
* {{ box-sizing: border-box; }}
body {{ margin: 0; background: var(--bg); color: var(--ink);
  font: 16px/1.7 "Noto Sans TC", "Microsoft JhengHei", system-ui, sans-serif; }}
main {{ max-width: 880px; margin: 0 auto; padding: 40px 16px 80px; }}
header {{ border-bottom: 1px solid var(--rule); padding-bottom: 20px; margin-bottom: 28px; }}
h1 {{ font-size: 30px; line-height: 1.3; margin: 0 0 6px; }}
.meta {{ color: var(--ink-2); font-size: 14px; }}
h2 {{ font-size: 21px; margin: 44px 0 10px; padding-top: 8px; border-top: 1px solid var(--rule); }}
h2 .num {{ color: var(--ink-3); margin-right: 8px; font-variant-numeric: tabular-nums; }}
h3 {{ font-size: 17px; margin: 24px 0 6px; }}
p, li {{ color: var(--ink); }}
.muted {{ color: var(--ink-2); font-size: 14px; }}
code {{ font: 13px/1.4 ui-monospace, Consolas, monospace; background: var(--surface); border: 1px solid var(--rule);
  padding: 1px 5px; border-radius: 4px; }}
nav.toc {{ background: var(--surface); border: 1px solid var(--rule); border-radius: 10px; padding: 12px 18px; font-size: 14px; }}
nav.toc ol {{ margin: 0; padding-left: 20px; columns: 2; }}
nav.toc a {{ color: var(--ink-2); text-decoration: none; }} nav.toc a:hover {{ color: var(--accent); }}
.tiles {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(150px, 1fr)); gap: 10px; margin: 18px 0; }}
.tile {{ background: var(--surface); border: 1px solid var(--rule); border-radius: 10px; padding: 12px 14px; }}
.tile .v {{ font-size: 26px; font-weight: 700; font-variant-numeric: tabular-nums; line-height: 1.2; }}
.tile .k {{ color: var(--ink-2); font-size: 13px; }}
.tile.hot .v {{ color: var(--hot); }}
.callout {{ border-radius: 10px; padding: 12px 16px; margin: 16px 0; }}
.callout.warn {{ background: var(--warn-bg); color: var(--warn-ink); }}
.callout.ok {{ background: var(--ok-bg); color: var(--ok-ink); }}
.callout b {{ color: inherit; }}
.tablewrap {{ overflow-x: auto; margin: 12px 0; }}
table {{ border-collapse: collapse; width: 100%; font-size: 14px; font-variant-numeric: tabular-nums; }}
th, td {{ text-align: left; padding: 6px 10px; border-bottom: 1px solid var(--rule); white-space: nowrap; }}
th {{ color: var(--ink-2); font-weight: 600; }}
td:not(:first-child), th:not(:first-child) {{ text-align: right; }}
table.text td, table.text th {{ text-align: left; white-space: normal; }}
figure {{ margin: 18px 0; background: var(--surface); border: 1px solid var(--rule); border-radius: 10px; padding: 14px; }}
figcaption {{ color: var(--ink-2); font-size: 13px; margin-top: 8px; }}
figure .ftitle {{ font-weight: 600; margin-bottom: 6px; }}
video {{ display: block; max-width: 100%; max-height: 75vh; margin: 0 auto; border-radius: 8px; background: #000; }}
.flow {{ display: flex; flex-wrap: wrap; gap: 6px; align-items: stretch; margin: 10px 0; }}
.flow .step {{ flex: 1 1 140px; background: var(--surface); border: 1px solid var(--rule); border-radius: 8px; padding: 8px 10px; font-size: 13px; }}
.flow .step b {{ display: block; font-size: 14px; color: var(--ink); }}
.flow .arrow {{ align-self: center; color: var(--ink-3); }}
.evidence {{ counter-reset: ev; list-style: none; padding: 0; }}
.evidence > li {{ counter-increment: ev; position: relative; padding: 10px 0 10px 40px; border-bottom: 1px solid var(--rule); }}
.evidence > li::before {{ content: counter(ev); position: absolute; left: 0; top: 10px; width: 26px; height: 26px; border-radius: 50%;
  background: var(--accent); color: #fff; font-weight: 700; font-size: 14px; display: grid; place-items: center; line-height: 1; }}
svg.chart {{ width: 100%; height: auto; display: block; overflow: visible; }}
svg .grid {{ stroke: var(--grid); stroke-width: 1; }}
svg .tick {{ fill: var(--ink-3); font-size: 12px; font-variant-numeric: tabular-nums; }}
svg .ptitle {{ fill: var(--ink-2); font-size: 13px; font-weight: 600; }}
svg .lab {{ fill: var(--ink-2); font-size: 13px; }}
svg .val {{ fill: var(--ink); font-size: 13px; font-weight: 600; font-variant-numeric: tabular-nums; }}
svg .line {{ fill: none; stroke: var(--accent); stroke-width: 2; stroke-linejoin: round; }}
svg .ref {{ stroke: var(--ref); stroke-width: 1.5; stroke-dasharray: 5 4; }}
svg .reflab {{ fill: var(--ink-3); font-size: 12px; }}
svg .bar {{ fill: var(--accent); }} svg .bar.hot {{ fill: var(--hot); }} svg .bar.thin {{ opacity: .45; }}
svg .mark:hover .bar {{ filter: brightness(1.12); }}
svg .hit, svg .hitbar {{ fill: transparent; }}
svg .cross {{ stroke: var(--ink-3); stroke-width: 1; }}
svg .dot {{ fill: var(--accent); stroke: var(--surface); stroke-width: 2; }}
.tip {{ position: fixed; pointer-events: none; background: var(--ink); color: var(--bg); font-size: 13px; padding: 6px 9px;
  border-radius: 6px; white-space: pre; opacity: 0; transition: opacity .08s; z-index: 10; font-variant-numeric: tabular-nums; }}
.legend {{ display: flex; gap: 16px; font-size: 13px; color: var(--ink-2); margin-bottom: 6px; flex-wrap: wrap; }}
.legend i {{ display: inline-block; width: 12px; height: 12px; border-radius: 3px; margin-right: 6px; vertical-align: -1px; }}
@media print {{ video {{ display: none; }} figure, .tile {{ break-inside: avoid; }} }}
</style>
</head>
<body>
<main>
<header>
  <h1>robot v2 實機測試報告</h1>
  <div class="meta">報告日期 2026-11-25 ・ 測試日期 2026-11-23 ・ 機器人 robot v2 ・ 模型 student v2ft（int8）・ 硬體 MANGO X4（RP2350）＋ 10 顆 Feetech HD-1910</div>
</header>

<nav class="toc"><ol>
  <li><a href="#s1">摘要</a></li><li><a href="#s2">測試環境</a></li><li><a href="#s3">馬達確認</a></li>
  <li><a href="#s4">站立姿態</a></li><li><a href="#s5">模型如何放上芒果磚</a></li><li><a href="#s6">步態 0.5 倍回放</a></li>
  <li><a href="#s7">實際模型推論 WALK</a></li><li><a href="#s8">結論：供電電流不足</a></li><li><a href="#s9">其他硬體問題</a></li>
  <li><a href="#s10">下一步</a></li>
</ol></nav>

<h2 id="s1"><span class="num">1</span>摘要</h2>
<p>這次把重新組裝的 robot v2 接上 v2 韌體，依序做了馬達確認、站立姿態、步態慢速回放與<b>第一次在實機上用模型閉迴路控制（WALK）</b>。
韌體與 int8 模型在芒果磚上可以即時運作；但模型以實際速度擺腿時，供電電壓從 7.7 V 一路掉到 6.1 V，追隨誤差隨之放大，<b>供電電流不足是目前唯一的阻擋項目</b>。</p>
<div class="tiles">
  <div class="tile"><div class="v">14 ms</div><div class="k">WALK 控制週期（上限 20 ms）</div></div>
  <div class="tile"><div class="v">6.8 ms</div><div class="k">板上 int8 推論時間</div></div>
  <div class="tile"><div class="v">≤ 0.5°</div><div class="k">站姿追隨誤差（懸空／桌上）</div></div>
  <div class="tile hot"><div class="v">{w['v_min']:.1f} V</div><div class="k">WALK 最低電壓（站姿時 7.7 V）</div></div>
  <div class="tile hot"><div class="v">× {ratio:.0f}</div><div class="k">WALK 電流 ÷ 懸空站姿電流（平均）</div></div>
</div>

<h2 id="s2"><span class="num">2</span>測試環境</h2>
<div class="tablewrap"><table class="text">
<tr><th>項目</th><th>內容</th></tr>
<tr><td>機器人</td><td>robot v2（大腿、小腿重新列印），每腿 4 DOF（hip roll、hip pitch、knee、ankle pitch）＋ 尾巴 2 DOF（yaw、pitch）</td></tr>
<tr><td>馬達</td><td>Feetech HD-1910 × 10，規格電壓 4–8.4 V，單顆堵轉電流約 1.6 A（6 V）</td></tr>
<tr><td>控制板</td><td>MANGO X4（RP2350，Cortex-M33），USB 接 PC 下指令與記錄 telemetry（50 Hz）</td></tr>
<tr><td>供電</td><td>HW-133A 降壓模組（標示 3 A），輸出約 7.7–7.8 V</td></tr>
<tr><td>模型</td><td>student v2ft（sim2real 蒸餾後的學生模型），MLP 210 → 512 → 256 → 128 → 10，int8 量化</td></tr>
<tr><td>安全保護</td><td>電壓 &lt; 6.5 V、溫度、追隨誤差 &gt; 25°、關節超出範圍、傾斜、匯流排讀取失敗 → 立即關扭矩</td></tr>
<tr><td>擺放</td><td>除桌上站立外，全部<b>懸空</b>（腳與尾巴不碰任何東西），有人扶著</td></tr>
</table></div>

<h2 id="s3"><span class="num">3</span>馬達確認（ID、方向、運轉）</h2>
<p class="muted">這部分組員在 v1 已經做過，v2 重新組裝後再確認一次，結果簡述如下。</p>
<ul>
  <li><b>ID</b>：匯流排掃描（<code>SCAN</code>）剛好 10 顆、ID 1–10。髖與尾巴的 6 顆 ID 不變；<b>膝、踝 4 顆左右對調</b>（右腿 1、2、7、8；左腿 5、6、3、4）。</li>
  <li><b>方向</b>：逐顆手扳確認正負方向；兩個膝蓋的方向和 v1 相反，以膝彎 90° 的姿勢再次確認。</li>
  <li><b>零點</b>：膝、踝重新量測；髖、尾巴的擺臂沒拆過，沿用 v1；tail_yaw 實測和舊零點差約 7°，已修正。IMU 前後、左右兩軸都以傾斜機身驗證。</li>
  <li><b>單顆運轉（<code>JOG</code>）</b>：一次只開一顆扭矩，轉 5–6°（10°/s）。方向全部正確，靜止誤差 ≤ 0.26°，指令到動作的延遲 <b>25–35 ms</b>（要求 ≤ 40 ms）。</li>
  <li><b>通訊</b>：同步讀取（sync read）10 顆只要 6.35 ms（逐顆讀要 30.8 ms），100 次 0 失敗。</li>
</ul>

<h2 id="s4"><span class="num">4</span>站立姿態（懸空 → 桌上）</h2>
<p><b>測試方式</b>：<code>INIT</code> 用約 9 秒把 10 顆馬達從目前位置緩慢移到模型訓練用的站姿，之後固定在站姿（不跑模型）。先懸空觀察，再由使用者把機器人放到桌上、手輕扶，站立約 100 秒。</p>
<div class="tablewrap"><table>
<tr><th>項目</th><th>懸空站姿</th><th>桌上站姿</th></tr>
<tr><td>最大追隨誤差</td><td>{hng['err_max']:.2f}°</td><td>{tab['err_max']:.2f}°</td></tr>
<tr><td>電壓</td><td>{hng['v_min']:.1f}–{hng['v_max']:.1f} V</td><td>{tab['v_min']:.1f}–{tab['v_max']:.1f} V</td></tr>
<tr><td>最高溫度</td><td>{hng['temp_max']} °C</td><td>{tab['temp_max']} °C</td></tr>
<tr><td>電流暫存器合計（平均／最大）</td><td>{d['hang_cur']['cur_mean']:.0f} ／ {d['hang_cur']['cur_max']:.0f}</td><td>{tab['cur_mean']:.0f} ／ {tab['cur_max']:.0f}</td></tr>
<tr><td>控制週期</td><td>{hng['loop_ms']:.1f} ms</td><td>{tab['loop_ms']:.1f} ms</td></tr>
<tr><td>保護觸發</td><td>無</td><td>無</td></tr>
</table></div>
<h3>各關節電流（站姿）</h3>
<p class="muted">電流暫存器（register 69–70）的單位尚未用電錶校正，這裡只做相對比較。懸空時主要是兩個膝蓋和尾巴 pitch 撐住自重；放到桌上後換成左踝、右 hip roll 吃力。</p>
<div class="tablewrap"><table>
<tr><th>關節</th><th>懸空 電流平均</th><th>桌上 電流平均</th><th>桌上 最大誤差（°）</th></tr>
{stance_cur_rows}
</table></div>
<h3>過程中修正的問題</h3>
<ul>
  <li><b>開扭矩瞬間跳動</b>：尾巴跳了 17°、左膝 4.9°，原因是寫入目標時被截在訓練用的關節範圍。改成寫入時用安全範圍 ± 10°，模型輸出另外在控制器截在訓練範圍。</li>
  <li><b>尾巴範圍</b>：尾巴幾乎不會撞到，安全檢查範圍放寬到 ±170°。</li>
  <li><b>左膝誤差偏大（2.28°）</b>：左膝馬達的固定螺絲（4 顆）掉了 1 顆，馬達在支架裡晃動；鎖回後降到 0.52°。</li>
  <li><b>hip roll 零點</b>：腳底平貼桌面時，零點偏差會直接變成機身左右傾。實測數據支持保留 v1 零點。</li>
</ul>
<div class="callout ok"><b>小結</b>：靜態站立（懸空與桌上）都穩定，誤差 ≤ 0.54°，電壓幾乎不變（7.6–7.8 V），供電在<b>靜態</b>負載下足夠。</div>

<h2 id="s5"><span class="num">5</span>模型如何放上芒果磚</h2>
<p>模型不是整個 PyTorch 檔放上去，而是<b>轉成 C 語言的權重陣列，和韌體一起編譯</b>：</p>
<div class="flow">
  <div class="step"><b>1. 學生模型</b>PyTorch（fp32），約 27 萬個參數</div><div class="arrow">→</div>
  <div class="step"><b>2. int8 量化</b>權重壓成 8 位元整數，每層一個縮放係數（<code>export_policy.py</code>）</div><div class="arrow">→</div>
  <div class="step"><b>3. 產生標頭檔</b>權重與觀測正規化參數寫成 <code>policy_int8.h</code>，約 270 KB（fp32 需約 1.1 MB）</div><div class="arrow">→</div>
  <div class="step"><b>4. 編譯燒錄</b>和韌體一起編成 UF2，USB 燒錄進芒果磚</div>
</div>
<p>板上每 20 ms（50 Hz）跑一次控制迴圈：</p>
<div class="flow">
  <div class="step"><b>讀感測</b>同步讀 10 顆馬達位置（6.35 ms）＋ IMU</div><div class="arrow">→</div>
  <div class="step"><b>組觀測</b>42 維 × 最近 5 步 = 210 維（角速度、重力方向、指令、關節角與速度、上一步動作）</div><div class="arrow">→</div>
  <div class="step"><b>int8 推論</b>Cortex-M33 DSP 整數指令，<b>6.8 ms</b>（原本 15.5 ms）</div><div class="arrow">→</div>
  <div class="step"><b>寫目標</b>站姿 ＋ 0.25 × 動作，截在關節範圍內後送給 10 顆馬達</div>
</div>
<p class="muted">驗證：板上推論和 PC 端 int8 參考的差距 2.3e-4，和原始 fp32 模型差 0.018；整個控制週期實測 13.9–14.3 ms。</p>

<h2 id="s6"><span class="num">6</span>步態測試一：0.5 倍速度回放（開迴路，懸空）</h2>
<p><b>測試方式</b>：先不讓模型做決定，只檢查馬達跟不跟得上步態。做法是把模擬中學生模型走路 4 秒的關節軌跡（202 格，每格 20 ms）存進韌體，在站姿下以 0.25 倍、0.5 倍放慢播放（<code>PLAY</code>），前後各有 1.5 秒的緩衝過渡。這是<b>開迴路</b>：不讀 IMU、不跑模型，只照表送角度。</p>
<div class="tablewrap"><table>
<tr><th>倍速</th><th>時長</th><th>最大追隨誤差</th><th>最低電壓</th><th>結果</th></tr>
<tr><td>0.25 倍</td><td>19.2 s</td><td>3.3°（左膝）</td><td>7.4 V</td><td>順暢</td></tr>
<tr><td>0.5 倍</td><td>11.1 s</td><td>3.7°（右膝）</td><td>{d['play05']['v_min']:.1f} V</td><td>順暢</td></tr>
</table></div>
<figure>
  <div class="ftitle">影片：0.5 倍速度步態回放（懸空）</div>
  <video controls preload="metadata" src="videos/play_0.5x_hanging.mp4#t=1"></video>
  <figcaption><code>videos/play_0.5x_hanging.mp4</code>。動作順暢，電壓維持在 7.6 V 以上。</figcaption>
</figure>
<div class="callout ok"><b>小結</b>：放慢後馬達完全跟得上步態，電壓也穩定。代表關節方向、零點、軌跡本身都正確。</div>

<h2 id="s7"><span class="num">7</span>步態測試二：實際模型推論 WALK（閉迴路，懸空）⭐</h2>
<p><b>測試方式</b>：<code>INIT</code> 到站姿、等 10 秒後送 <code>WALK 0.05 1.6</code>（前進速度 0.05 m/s、步頻 1.6 Hz，低速設定），由模型根據 IMU 與關節讀值<b>即時</b>決定每一步的關節目標，也就是<b>全速</b>、真正的 sim-to-real 閉迴路。6 秒後送 <code>STAND</code> 回站姿，再關扭矩。全程懸空。</p>
<div class="tablewrap"><table>
<tr><th>項目</th><th>第 1 次（電壓門檻 6.5 V）</th><th>第 2 次（門檻暫時降到 5.5 V）</th></tr>
<tr><td>結果</td><td><b>0.34 秒</b>電壓保護停機</td><td>完整走完 <b>6 秒</b>，回到站姿</td></tr>
<tr><td>電壓</td><td>7.7 → {d['walk65']['v_min']:.1f} V（停機時 6.4 V）</td><td>{w['v_min']:.1f}–{w['v_max']:.1f} V（平均 {w['v_mean']:.2f} V）</td></tr>
<tr><td>電流暫存器合計（平均／峰值）</td><td>{d['walk65']['cur_mean']:.0f} ／ {d['walk65']['cur_max']:.0f}</td><td>{w['cur_mean']:.0f} ／ {w['cur_max']:.0f}</td></tr>
<tr><td>最大追隨誤差</td><td>{d['walk65']['err_max']:.1f}°</td><td>{w['err_max']:.1f}°（左膝）</td></tr>
<tr><td>控制週期（中位數）</td><td>{d['walk65']['loop_ms']:.1f} ms</td><td>{w['loop_ms']:.1f} ms</td></tr>
<tr><td>最高溫度</td><td>{d['walk65']['temp_max']} °C</td><td>{w['temp_max']} °C</td></tr>
</table></div>
<p class="muted">門檻降到 5.5 V 只用於懸空測試（HD-1910 規格下限 4 V），目的是看清楚壓降時發生什麼事；上桌行走前會改回 6.5 V。</p>
<figure>
  <div class="ftitle">影片：模型推論 WALK（懸空，門檻 5.5 V 的第 2 次）</div>
  <video controls preload="metadata" src="videos/walk_policy_hanging.mp4#t=1"></video>
  <figcaption><code>videos/walk_policy_hanging.mp4</code>。步態節奏正常，懸空時機身前後左右擺動 ±9° 以內。</figcaption>
</figure>
<figure>
  <div class="ftitle">圖 1：WALK 6 秒期間的電壓、電流與追隨誤差（第 2 次，50 Hz telemetry）</div>
  {ts_svg}
  <figcaption>滑鼠移到圖上可看各時間點的數值。電壓只要出力就往下掉（最低 {w['v_min']:.1f} V），電流合計在 1000–3000 間跳動；誤差在電壓低的時候變大。</figcaption>
</figure>
<h3>各關節數據（第 2 次 WALK）</h3>
<div class="tablewrap"><table>
<tr><th>關節</th><th>電流平均</th><th>電流峰值</th><th>最大誤差（°）</th><th>速度 p95（°/s）</th><th>速度最大（°/s）</th></tr>
{joint_rows}
</table></div>
<p class="muted">關節速度由位置差分求得。膝、髖 pitch 的電流與速度都最大；左膝速度 p95 已到 {wj['left_knee']['vel_p95']:.0f}°/s，接近 HD-1910 的空載最高速（約 550°/s）。</p>
<div class="callout ok"><b>小結</b>：控制程式本身沒有問題：推論 + 通訊每個週期 14 ms（上限 20 ms），步態相位正常前進，6 秒內 0 次讀取失敗。問題出在<b>出力時供電撐不住</b>。</div>

<h2 id="s8"><span class="num">8</span>結論：供電電流不足</h2>
<p>我們沒有直接用電錶量到電流（見下方限制），「電流不足」是由以下四項證據交叉推得：</p>
<ol class="evidence">
  <li><b>只有模型全速走路時電壓會掉。</b>靜態站姿（懸空 7.7 V、桌上 7.6–7.7 V）與 0.5 倍回放（≥ 7.6 V）都穩定；WALK 一開始 0.1 秒內就掉到 6.8 V，最低 {w['v_min']:.1f} V。降壓模組輸出電壓固定，負載一大電壓就下滑，表示模組已經到電流上限。</li>
  <li><b>WALK 的電流是站姿的 {ratio:.0f} 倍。</b>電流暫存器合計：懸空站姿平均 {d['hang_cur']['cur_mean']:.0f}、桌上 {tab['cur_mean']:.0f}，WALK 平均 {w['cur_mean']:.0f}、峰值 {w['cur_max']:.0f}。電流和電壓的相關係數 −0.83：電流越大，電壓越低。
    <figure>
      <div class="ftitle">圖 2：各情境的電流暫存器合計（10 顆絕對值相加，raw）</div>
      <div class="legend"><span><i style="background:var(--accent)"></i>靜態</span><span><i style="background:var(--hot)"></i>模型 WALK</span></div>
      {cur_chart()}
      <figcaption>若單位是 Feetech STS 系列慣例的 6.5 mA（<b>未驗證</b>），約為懸空站姿 1.3 A、桌上 2.3 A、WALK 平均 10.8 A、峰值 19.6 A。</figcaption>
    </figure></li>
  <li><b>電壓越低，追隨誤差越大。</b>電壓 ≥ 7.2 V 時平均最大誤差 {eb['hi_mean']:.1f}°（{eb['hi_n']} 筆），&lt; 6.6 V 時 {eb['lo_mean']:.1f}°（{eb['lo_n']} 筆），約 {eb['lo_mean'] / eb['hi_mean']:.1f} 倍；電壓和誤差的相關係數 −0.78。馬達的扭矩與最高速都和供電電壓成正比，電壓一掉就追不上模型要的角度。
    <figure>
      <div class="ftitle">圖 3：各電壓區間的平均最大追隨誤差（第 2 次 WALK）</div>
      {bin_chart()}
      <figcaption>每 20 ms 取 10 顆中誤差最大的一顆，再依當時的電壓分組平均；樣本少於 3 筆的區間以淡色表示。</figcaption>
    </figure></li>
  <li><b>規格估算也對得上。</b>HD-1910 單顆堵轉約 1.6 A（6 V），走路時膝、髖 pitch 等多顆同時大出力，10 顆峰值可超過 10 A；HW-133A 標示 3 A，遠低於需求。這也和組員先前在 v1 開迴路走路時觀察到的「腿軟」一致。</li>
</ol>
<div class="callout warn"><b>限制（避免過度解讀）</b>
<ul style="margin:6px 0 0">
  <li>電流暫存器的單位<b>還沒用電錶校正</b>，上面換算的安培數只是推估；結論主要建立在電壓下降與誤差的關係上。</li>
  <li>電壓與誤差同時受「這一刻的動作有多用力」影響，相關性不完全等於因果；換供電後重測是最直接的驗證。</li>
  <li>膝、髖 pitch 的速度已接近馬達空載最高速，即使供電足夠，<b>馬達速度可能是第二個瓶頸</b>；現在是懸空，落地承重後負載會更大。</li>
</ul></div>
<div class="callout ok"><b>結論</b>：韌體與 int8 模型已能在實機上閉迴路運作；<b>目前阻擋上桌行走的是供電</b>。HW-133A（3 A）在走路時讓電壓在 6.1–7.7 V 間擺動，追隨誤差因此放大約 {eb['lo_mean'] / eb['hi_mean']:.1f} 倍。</div>

<h2 id="s9"><span class="num">9</span>其他硬體問題（組裝注意事項）</h2>
<ul>
  <li><b>匯流排斷線</b>：第一次手扳測試時匯流排約 87 秒完全沒有回應，重新整理線路後恢復，之後 0 失敗。接觸不良的可能性仍在，每次測試都監看讀取失敗數。</li>
  <li><b>馬達固定螺絲</b>：左膝馬達的 4 顆固定螺絲掉了 1 顆，造成讀值漂移與站姿誤差變大。組裝後建議逐一檢查並考慮加螺絲膠。</li>
</ul>

<h2 id="s10"><span class="num">10</span>下一步</h2>
<ol>
  <li><b>換供電</b>：連續 ≥ 8 A、峰值 ≥ 15 A 的降壓模組，或高放電 2S 鋰電池；馬達電源端並聯 2200–4700 µF 電容吸收瞬間電流。</li>
  <li><b>重跑懸空 WALK</b>（電壓門檻改回 6.5 V）：目標電壓維持 ≥ 7 V、追隨誤差約 5°。</li>
  <li>通過後，<b>綁安全繩上桌低速行走</b>。</li>
  <li>用電錶量實際電流，校正電流暫存器的單位。</li>
  <li>可選（不需實機）：在模擬中加入電壓下降 → 扭矩與速度下降的條件，評估模型能承受多少壓降。</li>
</ol>

<h2>附錄：資料來源</h2>
<ul class="muted">
  <li>詳細測試紀錄：<code>deploy/test_log.md</code>（2026-11-23 三段）；待辦 <code>docs/backlog.md</code> #21。</li>
  <li>原始資料：<code>outputs/robot_v2/deploy/sessions/20261123_*</code>（不進 git）；站姿 <code>103932</code>、<code>104449</code>，回放 <code>105642</code>，WALK <code>105717</code>（6.5 V）、<code>110511</code>（5.5 V）。</li>
  <li>重現 demo：<code>deploy/host/demo_hanging.ps1 -Mode play -Rate 0.5</code>、<code>-Mode walk</code>。</li>
</ul>
</main>
<div class="tip" id="tip"></div>
<script>
(function () {{
  const tip = document.getElementById('tip');
  function show(e, text) {{ tip.textContent = text; tip.style.opacity = 1;
    const x = Math.min(e.clientX + 14, window.innerWidth - tip.offsetWidth - 8); tip.style.left = x + 'px'; tip.style.top = (e.clientY + 14) + 'px'; }}
  function hide() {{ tip.style.opacity = 0; }}
  document.querySelectorAll('.mark').forEach(m => {{
    m.addEventListener('mousemove', e => show(e, m.dataset.tip)); m.addEventListener('mouseleave', hide); }});
  const S = {series_json};
  const svg = document.getElementById('ts'); if (!svg) return;
  const hit = svg.querySelector('.hit'), cross = svg.querySelector('.cross');
  const L = 56, W = 760, R = 16;
  hit.addEventListener('mousemove', e => {{
    const p = svg.createSVGPoint(); p.x = e.clientX; p.y = e.clientY;
    const q = p.matrixTransform(svg.getScreenCTM().inverse());
    const t = (q.x - L) / (W - L - R) * 6;
    let best = S[0]; for (const s of S) if (Math.abs(s[0] - t) < Math.abs(best[0] - t)) best = s;
    const x = L + (W - L - R) * best[0] / 6;
    cross.setAttribute('x1', x); cross.setAttribute('x2', x); cross.setAttribute('visibility', 'visible');
    show(e, `t = ${{best[0].toFixed(2)}} s\\n電壓 ${{best[1].toFixed(1)}} V\\n電流合計 ${{best[2]}}\\n最大誤差 ${{best[3].toFixed(1)}}°`);
  }});
  hit.addEventListener('mouseleave', () => {{ hide(); cross.setAttribute('visibility', 'hidden'); }});
}})();
</script>
</body>
</html>
"""
OUT.write_text(html, encoding="utf-8", newline="\n")
print("wrote", OUT, len(html))
