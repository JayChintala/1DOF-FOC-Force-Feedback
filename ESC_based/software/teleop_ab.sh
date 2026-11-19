#!/usr/bin/env bash
# Teleop at the 2026-10-29 operating point (TEST_LOG 0c UPDATE 3) with a given
# current-loop gain (CHANGE 8). Prints PHASE lines for record_listen.py --say.
# Usage: ./teleop_ab.sh <gain %> [label] [seconds]
G=${1:?gain %}; L=${2:-Run}; T=${3:-90}
echo "PHASE 1: $L starting. Hands off the leader."
sleep 3
TELEOP_POS_K=1e-3 TELEOP_POS_WINDOW=30 \
TELEOP_LEADER_K=2e-3 TELEOP_LEADER_K_MAX_A=0.4 TELEOP_LEADER_CONTACT_DAMP=1e-5 \
TELEOP_FRAME_QUEUE=1 TELEOP_SHADOW_TAU_ENV=1 TELEOP_FOLLOWER_MODEL=follower_model_arm3d.json \
TELEOP_FELT_SOURCE=tau_env TELEOP_K_F=1.0 TELEOP_FELT_DEADBAND_A=0.02 TELEOP_KB_DAMP=0.35e-5 \
TELEOP_HOLD_K=0.5e-3 TELEOP_HOLD_MAX_A=0.05 TELEOP_HOLD_RAMP_S=0.3 TELEOP_IQ1_MAX_A=0.8 \
TELEOP_LEADER_CAL=1 TELEOP_LOG_DIR=/dev/shm/teleop ESC_CURR_GAIN=$G \
timeout -s INT "$T" taskset -c 2,3 python3 -u teleop_test.py 2>&1
echo "PHASE 9: $L done."
