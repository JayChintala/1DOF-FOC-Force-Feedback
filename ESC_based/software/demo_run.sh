#!/usr/bin/env bash
#
# Unattended demo supervisor for the leader-follower teleop.
#
#   ./demo_run.sh            # run until Ctrl+C
#   CYCLE=600 ./demo_run.sh  # shorter restart cycle (default 1200 s)
#   KEEP_LOGS=1 ./demo_run.sh
#
# Why a supervisor: teleop_test.py logs several MB/min. Left running it fills
# /dev/shm in a few hours, the CSV write throws, and the run exits -- safely
# (motors zero), but the demo is then dead and nobody notices. This restarts
# it on a fixed cycle and clears that cycle's output, so nothing accumulates
# and a crash costs seconds rather than the rest of the afternoon.
#
# All demo output goes to logs/demo/ (TELEOP_OUT_DIR) and the RAM staging dir
# below. Nothing outside those two directories is ever removed.
#
# Restarts are seamless while the ESCs stay powered: START on an already
# calibrated board reaches RUN in ~0.05 s, not the 2 s / 0.8 A alignment that
# only runs on the first START after a board power-up.
#
set -u
cd "$(dirname "$0")"

# 0 = never restart on a timer (the default, and what you want for a demo).
# Each START runs a hands-off ~30 s leader calibration sweep in which the
# leader spins ITSELF, so a timed restart means the handle comes alive in
# someone's hand every CYCLE seconds. With TELEOP_LOG_FRAMES=0 the logs run
# at ~4 MB/min, so a 1.9 GB /dev/shm lasts ~8 h -- a whole demo day in one
# session. Set CYCLE>0 only if you have a reason to.
CYCLE="${CYCLE:-0}"
LIVE="${LIVE:-/dev/shm/teleop_demo}"    # RAM staging while a run is live
OUT="logs/demo"                         # finished demo logs (cleared each cycle)
KEEP_LOGS="${KEEP_LOGS:-0}"             # 1 = keep each run's CSV in logs/demo

say() { printf '\n[demo %s] %s\n' "$(date +%H:%M:%S)" "$*"; }

# ---- pre-flight ------------------------------------------------------------
say "pre-flight"
thr=$(vcgencmd get_throttled 2>/dev/null || echo "throttled=?")
echo "  Pi: $thr   (want throttled=0x0)"
[ "$thr" = "throttled=0x0" ] || echo "  !! under-voltage/throttling - check the supply"

ip -br link show can0 2>/dev/null | grep -q UP \
  || { echo "  !! can0 is not up. Aborting."; exit 1; }
timeout 3 candump -n 2 can0 >/dev/null 2>&1 \
  || { echo "  !! no CAN traffic - are the ESCs powered? Aborting."; exit 1; }
echo "  can0 up, ESCs streaming"

# Idle Iq far from zero means a board has not been STARTed since power-up, so
# the next START runs the 2 s / 0.8 A alignment and visibly jerks the arm.
# Get that over with now, not in front of an audience.
if python3 - <<'PY'
import can, struct, sys, time
b = can.interface.Bus(channel="can0", interface="socketcan",
                      can_filters=[{"can_id": 0x012, "can_mask": 0x7FF},
                                   {"can_id": 0x032, "can_mask": 0x7FF}])
vals, t0 = [], time.monotonic()
while time.monotonic() - t0 < 1.5:
    m = b.recv(timeout=0.3)
    if m and len(m.data) == 8:
        vals.append(abs(struct.unpack("<fHH", m.data)[0]))
b.shutdown()
sys.exit(0 if vals and max(vals) > 5.0 else 1)
PY
then
  say "boards are UNCALIBRATED -- running zero_iq_check first"
  echo "  >>> hands off; the follower arm needs room to swing ~45 deg <<<"
  sleep 3
  taskset -c 2,3 python3 zero_iq_check.py || { echo "  !! zero_iq_check failed"; exit 1; }
else
  echo "  boards already calibrated - no alignment twitch on start"
fi

# ---- the operating point (TEST_LOG 0c UPDATE 3, 2026-10-29) ----------------
export TELEOP_POS_K=1e-3 TELEOP_POS_WINDOW=30 \
       TELEOP_LEADER_K=2e-3 TELEOP_LEADER_K_MAX_A=0.4 TELEOP_LEADER_CONTACT_DAMP=1e-5 \
       TELEOP_FRAME_QUEUE=1 TELEOP_SHADOW_TAU_ENV=1 \
       TELEOP_FOLLOWER_MODEL=follower_model_arm3d.json \
       TELEOP_FELT_SOURCE=tau_env TELEOP_K_F=1.0 TELEOP_FELT_DEADBAND_A=0.02 \
       TELEOP_KB_DAMP=0.35e-5 \
       TELEOP_HOLD_K=0.5e-3 TELEOP_HOLD_MAX_A=0.05 TELEOP_HOLD_RAMP_S=0.3 \
       TELEOP_IQ1_MAX_A=0.8 TELEOP_LEADER_CAL=1 \
       TELEOP_LOG_DIR="$LIVE" TELEOP_OUT_DIR="$OUT" \
       TELEOP_LOG_FRAMES=0 TELEOP_PLOT=0

mkdir -p "$LIVE" "$OUT"
stop=0
trap 'stop=1' INT TERM

if [ "$CYCLE" -gt 0 ]; then
  say "running. Ctrl+C to stop. TIMED RESTART every ${CYCLE}s -- note each restart"
  echo "       spins the leader for ~30 s of calibration. Output in $OUT."
else
  say "running. Ctrl+C to stop. One session; restarts only if teleop exits."
  echo "       Calibration runs ONCE, now. Output in $OUT (KEEP_LOGS=$KEEP_LOGS)."
fi
n=0
while [ "$stop" -eq 0 ]; do
  n=$((n + 1))
  if [ "$n" -gt 1 ]; then
    say "RESTARTING -- the leader will spin itself for ~30 s of calibration"
    echo "  >>> hands off both handles until this prints 'run $n' output <<<"
    sleep 4
  fi
  say "run $n   free in $(dirname "$LIVE"): $(df -h "$(dirname "$LIVE")" | awk 'NR==2{print $4}')"
  if [ "$CYCLE" -gt 0 ]; then
    timeout -s INT "$CYCLE" taskset -c 2,3 python3 teleop_test.py 2>&1 \
      | grep -viE 'deprecationwarning|bustype' | grep -vE '^[[:space:]]*$' | tail -4
  else
    # Stream it: teleop prints a status line at 10 Hz, which is the only
    # sign of life during a session that may run for hours.
    stdbuf -oL taskset -c 2,3 python3 teleop_test.py 2>&1 \
      | grep --line-buffered -viE 'deprecationwarning|bustype'
    [ "$stop" -eq 0 ] && say "teleop exited on its own -- recovering"
  fi
  [ "$stop" -eq 1 ] && break
  # Scoped cleanup: only ordinary files directly inside the two demo dirs.
  [ "$KEEP_LOGS" = "1" ] || find "$OUT" -maxdepth 1 -type f -name 'teleop_*' -delete
  find "$LIVE" -maxdepth 1 -type f -delete
  sleep 1
done

say "stopping - zeroing both motors"
python3 - <<'PY'
import can, struct, time
b = can.interface.Bus(channel="can0", interface="socketcan")
for base in (0x000, 0x020):
    b.send(can.Message(arbitration_id=base + 0x003,
                       data=struct.pack("<f", 0.0), is_extended_id=False))
time.sleep(0.05)
for base in (0x000, 0x020):
    b.send(can.Message(arbitration_id=base + 0x002, data=b"", is_extended_id=False))
b.shutdown()
print("  SET_IQ 0 + STOP sent to both")
PY
find "$LIVE" -maxdepth 1 -type f -delete 2>/dev/null
say "done after $n run(s)"
