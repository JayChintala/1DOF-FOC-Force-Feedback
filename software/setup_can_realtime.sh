#!/usr/bin/env bash
#
# Host-side latency tuning for the CAN telemetry path. Run once per boot,
# as root, BEFORE starting a control script:
#
#     sudo ./setup_can_realtime.sh          # tune, using the defaults below
#     sudo ./setup_can_realtime.sh -n       # show what it would do, change nothing
#
# WHY THIS EXISTS
#
# can_interface.py already does everything a process can do to itself: it
# raises SO_RCVBUF on the socket and puts its RX thread on SCHED_FIFO. What
# it cannot do is decide which CPU the kernel services the CAN adapter's
# interrupt on.
# By default that is CPU0, which is also where Linux puts most other
# housekeeping and where the control loop is as likely to land as anywhere
# else. When the two collide, the RX thread's wakeup waits behind whatever
# else that core is doing -- which is the mechanism behind the multi-
# millisecond stalls telemetry_rate_probe.py measured (p99 ~33 ms, worst
# 48 ms), not raw CPU cost. There is no shortage of CPU here; there is a
# queueing problem on one core.
#
# So: CAN IRQ on its own core, control loop kept off that core.
#
# WHAT IT DOES NOT DO
#
# It does not pin the control loop. That is per-script -- launch it with
#     taskset -c 2,3 python3 teleop_test.py
# or whatever cores you left free. Pinning the IRQ and letting the control
# loop float onto the same core undoes most of the benefit.
#
# It is not persistent. Nothing here survives a reboot. Wire it into a
# systemd unit or your launch script once the values are settled.

set -euo pipefail

IFACE="${IFACE:-can0}"
IRQ_CPU="${IRQ_CPU:-1}"      # core to service the CAN IRQ on
RMEM_MAX="${RMEM_MAX:-4194304}"

DRY_RUN=0
[[ "${1:-}" == "-n" ]] && DRY_RUN=1

run() {
  if (( DRY_RUN )); then
    echo "  would run: $*"
  else
    "$@"
  fi
}

if (( ! DRY_RUN )) && (( EUID != 0 )); then
  echo "error: needs root (or -n to preview)." >&2
  exit 1
fi

# ---- 1. Find the IRQ ------------------------------------------------------
# Discovered, never hardcoded. The IRQ number is assigned at probe time and
# depends on the adapter, the kernel and the order devices came up; a stale
# hardcoded number would silently retune some unrelated device's interrupt.
#
# THIS BUS IS gs_usb OVER USB (confirm with: ip -d link show can0). That
# matters more than it sounds. A USB CAN adapter raises no interrupt of its
# own -- there is no line in /proc/interrupts named can0 to pin. The
# interrupt that actually carries these frames belongs to the USB HOST
# CONTROLLER the adapter is plugged into, and pinning it moves every device
# on that controller along with it. That is normally fine and is still the
# right move, but it is a broader action than "pin the CAN IRQ", so the
# script says which controller it is about to affect.
#
# Two consequences worth knowing before reading the results:
#   - gs_usb batches frames into URBs, so some arrival jitter is the USB
#     microframe schedule and will NOT respond to affinity tuning.
#   - if the adapter shares a controller with something bursty (a webcam, a
#     USB disk), moving it to a different physical port can beat any amount
#     of affinity tuning.

IRQS=()

# Direct case first: a native CAN controller names its own IRQ.
readarray -t IRQS < <(
  awk -v ifc="$IFACE" '$NF == ifc { sub(":", "", $1); print $1 }' /proc/interrupts 2>/dev/null
)
IRQ_SOURCE="native controller ($IFACE)"

# USB case: walk sysfs from the netdev up to the first parent exposing an
# irq attribute -- that is the host controller.
if (( ${#IRQS[@]} == 0 )); then
  dev_path=$(readlink -f "/sys/class/net/${IFACE}/device" 2>/dev/null || true)
  while [[ -n "$dev_path" && "$dev_path" != "/sys/devices" && "$dev_path" != "/" ]]; do
    if [[ -r "$dev_path/irq" ]]; then
      IRQS=("$(cat "$dev_path/irq")")
      IRQ_SOURCE="USB host controller $(basename "$dev_path") -- shared with every device on it"
      break
    fi
    dev_path=$(dirname "$dev_path")
  done
fi

if (( ${#IRQS[@]} == 0 )); then
  echo "warning: could not identify an IRQ for '$IFACE'." >&2
  echo "         Inspect 'ip -d link show $IFACE' and /proc/interrupts by" >&2
  echo "         hand. Continuing with the socket-buffer step." >&2
else
  echo "IRQ source: $IRQ_SOURCE"
  for irq in "${IRQS[@]}"; do
    aff="/proc/irq/${irq}/smp_affinity_list"
    if [[ ! -e "$aff" ]]; then
      echo "IRQ $irq: no smp_affinity_list (kernel-managed); skipping." >&2
      continue
    fi
    echo "IRQ $irq: affinity -> CPU $IRQ_CPU  (was $(cat "$aff" 2>/dev/null || echo '?'))"
    # smp_affinity_list takes a CPU list directly, so there is no hex mask
    # to get wrong. Writing it fails on a device whose driver pins its own
    # affinity; that is reported, not fatal.
    if (( DRY_RUN )); then
      echo "  would write: $IRQ_CPU > $aff"
    elif ! echo "$IRQ_CPU" > "$aff" 2>/dev/null; then
      echo "  failed -- driver may manage affinity itself; skipping." >&2
    fi
  done

  # irqbalance will happily undo the above within seconds. Only worth
  # mentioning if it is actually running.
  if systemctl is-active --quiet irqbalance 2>/dev/null; then
    echo
    echo "WARNING: irqbalance is running and WILL revert the affinity above." >&2
    echo "         Disable it:  sudo systemctl disable --now irqbalance" >&2
  fi
fi

# ---- 2. Socket buffer ceiling --------------------------------------------
# can_interface.py asks for RX_BUFFER_BYTES; the kernel silently clamps the
# request to rmem_max instead of refusing it, so a low ceiling here shows up
# as dropped frames rather than as an error.
CUR_RMEM=$(sysctl -n net.core.rmem_max 2>/dev/null || echo 0)
if (( CUR_RMEM == 0 )); then
  echo "net.core.rmem_max: could not read it; skipping." >&2
elif (( CUR_RMEM < RMEM_MAX )); then
  echo "net.core.rmem_max: $CUR_RMEM -> $RMEM_MAX"
  run sysctl -w "net.core.rmem_max=$RMEM_MAX"
else
  echo "net.core.rmem_max: $CUR_RMEM (already >= $RMEM_MAX, unchanged)"
fi

# ---- 3. Report ------------------------------------------------------------
echo
echo "Queue length on $IFACE: $(ip -d -s link show "$IFACE" 2>/dev/null | awk '/qlen/ {print $NF; exit}' || echo '?')"
echo
echo "Remaining, per control script: pin it OFF CPU $IRQ_CPU, e.g."
echo "    taskset -c 2,3 python3 teleop_test.py"
echo
echo "Verify with telemetry_rate_probe.py: the number that should move is the"
echo "p99 handling lag, not the frame rate."
