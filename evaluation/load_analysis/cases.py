"""Load-analysis cases v1 (2026-10-15): how hard each servo works while standing, stepping in place, walking at
several speeds, on bumps, slopes and steps, under pushes, with a heavier robot and with randomised actuators.

Each case = an existing scenario (``simulation/env/scenarios.py``, unchanged) + one fixed command for every robot
(``vx`` [m/s] and ``freq`` [Hz]; ``freq`` 0 = stand, clock frozen, as in training) + optional pushes.
Frozen once used: add ``CASES_V2`` instead of editing.
"""

CASES_V1: dict[str, dict] = {
    "01_stand": {"title": "靜止站立", "scenario": "flat_v1", "vx": 0.0, "freq": 0.0},
    "02_in_place": {"title": "原地踏步（1.6 Hz）", "scenario": "flat_v1", "vx": 0.0, "freq": 1.6},
    "03_flat_slow": {"title": "平地慢速 0.06 m/s", "scenario": "flat_v1", "vx": 0.06, "freq": 1.6},
    "04_flat_medium": {"title": "平地中速 0.10 m/s", "scenario": "flat_v1", "vx": 0.10, "freq": 1.6},
    "05_flat_fast": {"title": "平地快速 0.14 m/s", "scenario": "flat_v1", "vx": 0.14, "freq": 1.6},
    "06_flat_max": {"title": "訓練範圍最高指令 0.15 m/s、2.0 Hz", "scenario": "flat_v1", "vx": 0.15, "freq": 2.0},
    "07_bumps_8mm": {"title": "±8 mm 凹凸地面（訓練最難一級）", "scenario": "rough_eval_v1", "vx": 0.10, "freq": 1.6},
    "08_push_25": {"title": "平地＋推擠 0.25 m/s（每 5 秒）", "scenario": "flat_v1", "vx": 0.10, "freq": 1.6,
                   "push_speed": 0.25},
    "09_slope_up_5deg": {"title": "上坡 5°", "scenario": "stress_v2/06_slope_up_5deg", "vx": 0.10, "freq": 1.6},
    "10_steps_up_10mm": {"title": "上台階 10 mm（失效邊界）", "scenario": "stress_v2/07_steps_up_10mm", "vx": 0.10,
                         "freq": 1.6},
    "11_mass_150": {"title": "整機質量 ×1.5", "scenario": "stress_v1/08_mass_150", "vx": 0.10, "freq": 1.6},
    "12_dr_moderate": {"title": "中等隨機化（延遲、馬達參數、摩擦、質量）", "scenario": "dr_v1", "vx": 0.10, "freq": 1.6},
}

# HD-1910-C001 at 6 V (conservative numbers used by the simulation; hardware/design_refs/hd1910/README.md)
SERVO_6V = {"stall_torque": 1.18, "rated_torque": 0.294, "sim_limit": 0.8,
            "stall_current": 1.6, "rated_current": 0.69, "noload_current": 0.18}

# Cases v2 (2026-11-23): slow gait for the real robot. Same slow speed 0.05 m/s (the real-robot WALK command), gait
# frequency 1.6 (what the robot ran on 2026-11-23) / 1.2 / 1.0 / 0.8 Hz -> joint speed and current vs frequency.
CASES_V2: dict[str, dict] = {
    **{f"21_flat_f{int(f * 10):02d}": {"title": f"平地 0.05 m/s、{f} Hz", "scenario": "flat_v1", "vx": 0.05, "freq": f}
       for f in (1.6, 1.2, 1.0, 0.8)},
    **{f"22_dr_f{int(f * 10):02d}": {"title": f"中等隨機化 0.05 m/s、{f} Hz", "scenario": "dr_v1", "vx": 0.05, "freq": f}
       for f in (1.6, 1.0, 0.8)},
}
# Cases v3 (2026-11-23): slow-gait scenarios (servo velocity limit 5.2 rad/s), to compare student v2slow with v2ft.
CASES_V3: dict[str, dict] = {
    **{f"31_flat_slow_f{int(f * 10):02d}": {"title": f"慢場景平地 0.05 m/s、{f} Hz", "scenario": "flat_slow_v1", "vx": 0.05,
                                            "freq": f} for f in (1.2, 1.0, 0.8)},
    **{f"32_dr_slow_f{int(f * 10):02d}": {"title": f"慢場景中等隨機化 0.05 m/s、{f} Hz", "scenario": "dr_slow_v1", "vx": 0.05,
                                          "freq": f} for f in (1.0, 0.8)},
}
CASE_SETS = {"v1": CASES_V1, "v2": CASES_V2, "v3": CASES_V3}
