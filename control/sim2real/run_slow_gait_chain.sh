#!/usr/bin/env bash
# Slow-gait training chain for robot v2 (2026-11-23, control/sim2real/log_robot_v2.md "慢步態模型 slow").
# teacher -> student -> evaluation of the slow model and of v2ft on the slow scenarios -> rescore. Sequential (one sim at a time).
set -u
cd "$(dirname "$0")/../.."
export CUSTOM_BIPED_VERSION=2
O=outputs/robot_v2/control/sim2real
LOG=$O/slow_chain_logs_$(date +%Y%m%d_%H%M%S); mkdir -p "$LOG"
P="tools/isaacpy.cmd"
echo "[chain] teacher $(date +%T)"
$P control/sim2real/teacher_ppo.py --tag v2slow --symmetry --reward_preset obs_lateral --scenario dr_rough_gait_slow_train_v1 \
   --init $O/teacher_v2ft_20261118_221706/model_1499.pt --max_iterations 1500 > "$LOG/teacher.log" 2>&1 || { echo "[chain] teacher FAILED"; exit 1; }
T=$(ls -d $O/teacher_v2slow_2* | tail -1); echo "[chain] teacher done: $T $(date +%T)"
$P control/sim2real/student_distill.py --teacher "$T/model_1499.pt" --tag v2slow --symmetry --reward_preset obs_lateral \
   --scenario dr_rough_gait_slow_train_v1 > "$LOG/student.log" 2>&1 || { echo "[chain] student FAILED"; exit 1; }
S=$(ls -d $O/student_v2slow_2* | tail -1); echo "[chain] student done: $S $(date +%T)"
for sc in flat_slow_v1 dr_slow_v1 rough_eval_slow_v1 dr_rough_eval_slow_v1 gait_eval_slow_v1; do
  $P control/sim2real/evaluate.py --scenario $sc --agent "student:$S/student.pt" --tag v2slow > "$LOG/eval_${sc}_v2slow.log" 2>&1
  $P control/sim2real/evaluate.py --scenario $sc --agent "student:$O/student_v2ft_20261118_230657/student.pt" --tag v2ft_on_slow > "$LOG/eval_${sc}_v2ft.log" 2>&1
  echo "[chain] eval $sc $(date +%T)"
done
$P control/sim2real/rescore.py $(ls -d $O/eval_*_slow_v1_v2slow_2* $O/eval_*_slow_v1_v2ft_on_slow_2*) > "$LOG/rescore.log" 2>&1
echo "[chain] all done $(date +%T); logs $LOG"
