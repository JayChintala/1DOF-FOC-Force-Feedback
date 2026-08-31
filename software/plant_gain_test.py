"""
Plant gain (inertia) measurement -- how much acceleration each amp buys.

WHY THIS EXISTS
Every gain in teleop_test.py has been chosen by bench feel, and the two
useful designs left both need a NUMBER instead:

  - the achievable KV_SLOPE follows directly from the plant gain and the
    loop's phase lag: a velocity loop on this plant closes at
    omega_c = G * KV rad/s, and it goes unstable when omega_c * T ~ pi/2,
    so KV_max ~ pi / (2 * G * T). With T measured at ~4.5 ms, G is the
    only unknown left in that expression.
  - a disturbance observer that estimates how hard a hand is pushing on
    Motor B needs B's inertia explicitly: tau_ext = J * dw/dt - Kt * i,
    and 1/G is exactly that J/Kt ratio in the units this project uses
    (encoder counts, amps).

Regressing acceleration against current in the ordinary teleop logs gave
G = 1.3e6 to 4.6e6 cnt/s^2 per A with correlations of only 0.44-0.58 --
a factor of 3.5 of uncertainty, because in those logs a hand and a limit
cycle are both injecting torque the regression cannot see. This measures
it in open loop with nothing else touching the shaft.

METHOD
Bang-bang current around zero SPEED, not fixed-duration pulses. A step
would be unsafe: sign_check_test.py measured 0.15 A spinning a free motor
at ~20 rev/s, so a sustained level runs away.

The first version of this script used fixed-duration alternating pulses
and it did not survive repeat runs (5 consecutive runs: G spread 5.8e6 to
8.4e6, the friction intercept swung +-0.03 A when it should be ~0, and the
velocity watchdog tripped 5 times, always at iq = -0.15 A). Cause: the two
directions do not produce equal accelerations -- 950,324 vs -1,032,868
cnt/s^2 in one run, 8% asymmetric -- so every +/- cycle left ~3,800 cnt/s
of residual speed and ten cycles walked the shaft into the 40,000 cnt/s
watchdog. Worse than the aborts, the drift meant each current level was
measured at a different mean speed, so viscous drag masqueraded as a
change in G and wrecked the friction split.

Reversing on a SPEED BAND instead of a clock removes the failure mode by
construction: hold +i until the shaft reaches +V_BAND_CNT_S, then -i until
it reaches -V_BAND_CNT_S, and repeat. Speed cannot leave the band no
matter how asymmetric the two directions are, every level is measured over
the same speed range, and the reversals come out evenly spaced in both
directions for free.

Acceleration is estimated by fitting a QUADRATIC TO POSITION over each
constant-current segment, p(t) = p0 + v0*t + a*t^2/2, rather than by
differentiating a velocity estimate. Position is the raw measurement --
integers, but hundreds of counts of travel per segment -- so a quadratic
through all of a segment's samples is far quieter than double-
differencing, which amplifies the 1-count quantisation by 1/dt^2. The old
approach also divided by frame-timestamp differences that can be ZERO: at
a 500 Hz sample loop against 1 kHz telemetry, two consecutive samples
sometimes read the same frame, and np.gradient then returned inf/nan
(observed: one run reported G = nan).

Friction is then separated by regressing all segments together on

    a = G*i - b*v_mid - c*sign(v_mid)

which fits the plant gain, viscous drag and Coulomb friction jointly. The
sign(v) column is what makes the fit insensitive to any offset that does
not flip with direction (gravity on an unbalanced shaft, bearing preload),
and the v column stops residual speed differences between levels from
being absorbed into G -- the specific failure of the first version.

SAFETY
  - Both shafts must be FREE and UNTOUCHED: this commands real torque with
    no position feedback. Nothing attached that torque could sling.
  - Current reverses on a speed band (V_BAND_CNT_S), so speed stays near
    zero by construction rather than by luck. V_BAND_CNT_S is set a factor
    of 4 below the watchdog, so the watchdog is a backstop that should
    never fire, not part of normal operation.
  - A velocity watchdog aborts and zeroes both motors if either shaft
    exceeds WATCHDOG_VEL_CNT_S, before the fit ever runs.
  - Every level is followed by a 0 A dwell, so a bad level cannot hand off
    its speed to the next one.

VERIFIED RESULT -- 2026-08-31, 5 consecutive runs of the speed-band version,
no watchdog trips, no failed fits (logs/plant_gain_7..11.csv):

    Motor A:  G = 7,622,000 cnt/s^2 per A  (1,906 rev/s^2 per A)
    Motor B:  G = 7,915,000 cnt/s^2 per A  (1,979 rev/s^2 per A)
    run-to-run standard error ~4.4%;  G(A)/G(B) = 0.96, so the two shafts
    are matched to within 4% and symmetric gains are justified.
    Coulomb friction +0.0004 A (A) and +0.0011 A (B) -- essentially zero,
    i.e. there is no stiction floor: any push, however light, moves a shaft.

Per-run scatter is ~10% and the fit residual ~12%, against 0.9% on
synthetic data, so something real is unmodelled. It is partly cogging: a
14/rev component (twice the 7 pole pairs) explains 7.3% of Motor A's
residual variance and 20.6% of Motor B's, at 0.009 A and 0.016 A
equivalent. That figure doubles as the noise floor for any torque
observer built on these shafts. Regressing on measured rather than
commanded current does NOT help (measured/commanded is 0.93-1.01 in the
fitted windows, and iq_readback adds its own noise), so the current loop
is not the culprit.

The viscous/Coulomb split is poorly identified by this excitation -- a
bang-bang test spends most of its time near +/-V_BAND_CNT_S, so the v and
sign(v) columns are nearly collinear and b comes out anywhere from 12 to
48 1/s. Their COMBINED effect is small (~0.02 A at 2.5 rev/s) and G is
insensitive to how it is split, but do not quote b from this test.

Run from software/ with the venv active, hands off both shafts:
    python3 plant_gain_test.py
"""

import csv
import os
import sys
import time

import numpy as np

from can_interface import MotorCANInterface, ENC_PULSE_NBR

MOTORS = (("A", 0x000), ("B", 0x020))

LEVELS_A = (0.06, 0.10, 0.15)   # per sign_check_test.py, 0.15 A is already
                                 # enough to spin a free shaft ~20 rev/s, so
                                 # this is the ceiling, not a starting point.
                                 # Three levels so the fit can show whether the
                                 # response is actually linear in current.
V_BAND_CNT_S = 10_000.0          # +-2.5 rev/s: current reverses here. Sets the
                                 # speed range every level is measured over, so
                                 # viscous drag enters identically at each level
                                 # instead of scaling with it.
LEVEL_DURATION_S = 1.5           # per level; at 0.15 A a band-to-band ramp is
                                 # ~25 ms, at 0.06 A ~60 ms, so this yields
                                 # roughly 25-60 reversals per level.
DWELL_S = 0.4                    # 0 A between levels, lets speed decay to ~0

SAMPLE_RATE_HZ = 1000            # NOT teleop's 500 Hz: at 0.15 A a band-to-band
                                 # ramp lasts only ~20 ms, which is ~10 samples
                                 # at 500 Hz and drops below MIN_SEG_SAMPLES once
                                 # trimmed -- the highest current level was being
                                 # silently discarded from the fit. 1 kHz was
                                 # measured achievable for this duty cycle
                                 # (999 Hz actual) and telemetry arrives at
                                 # 1 kHz anyway, so nothing is being invented;
                                 # repeated frames are deduplicated in the fit.
MIN_VEL_DT_S = 0.002             # velocity differenced over encoder frame
                                 # timestamps, same scheme as teleop_test.py
WATCHDOG_VEL_CNT_S = 40_000.0    # 10 rev/s -- 4x above V_BAND_CNT_S, so this is
                                 # a backstop that should never fire. It firing
                                 # means the band logic itself failed.
MIN_SEG_SAMPLES = 8              # a segment shorter than this cannot support a
                                 # 3-parameter quadratic fit worth trusting
FIT_TRIM = 0.20                  # ignore the first/last 20% of each segment:
                                 # the current loop needs a moment to reach the
                                 # new setpoint after a reversal

LOG_DIR = "logs"


def measure_motor(name, node_base, csv_writer):
    """Bang-bang one motor's current around zero speed and return its samples."""
    m = MotorCANInterface(channel="can0", node_base=node_base)
    m.start_listening()
    samples = []          # (t, iq_cmd, pos, bus_time, iq_readback)
    aborted = False

    try:
        print(f"\n=== Motor {name} (node_base=0x{node_base:03X}) ===")
        m.send_start()
        time.sleep(0.2)
        m.send_set_iq(0.0)
        time.sleep(0.3)

        t0 = m.get_telemetry()
        if t0["enc_count_unwrapped"] is None:
            print(f"  No encoder telemetry from Motor {name} -- skipping.")
            return None

        dt = 1.0 / SAMPLE_RATE_HZ
        t_start = time.monotonic()
        deadline = time.monotonic()
        # Velocity tracked here purely to drive the reversals and the watchdog;
        # the acceleration fit later works from position, not from this.
        prev_pos = None
        prev_bus = None
        vel = 0.0
        reversals = 0

        for level in list(LEVELS_A) + [0.0]:
            seg_end = time.monotonic() + (LEVEL_DURATION_S if level else DWELL_S)
            sign = 1.0
            iq = level * sign
            m.send_set_iq(iq)
            while time.monotonic() < seg_end and not aborted:
                tel = m.get_telemetry()
                pos = tel["enc_count_unwrapped"]
                bus_t = tel["enc_count_bus_time"]
                if pos is not None and bus_t is not None:
                    # Only a NEW frame far enough from the last one updates the
                    # velocity -- a repeated bus_time would divide by zero.
                    if prev_bus is not None and bus_t - prev_bus >= MIN_VEL_DT_S:
                        vel = (pos - prev_pos) / (bus_t - prev_bus)
                        prev_pos, prev_bus = pos, bus_t
                    elif prev_bus is None:
                        prev_pos, prev_bus = pos, bus_t
                    samples.append((time.monotonic() - t_start, iq, pos, bus_t,
                                    tel["iq_readback"]))

                    if abs(vel) > WATCHDOG_VEL_CNT_S:
                        print(f"  WATCHDOG: |vel| = {abs(vel):.0f} cnt/s "
                              f"({abs(vel)/ENC_PULSE_NBR:.1f} rev/s) at "
                              f"iq={iq:+.3f} A -- aborting. The speed band "
                              f"should have prevented this.")
                        aborted = True
                        break

                    # Reverse at the band edges. This is what keeps speed
                    # bounded regardless of how asymmetric the directions are.
                    if level and ((sign > 0 and vel > V_BAND_CNT_S)
                                  or (sign < 0 and vel < -V_BAND_CNT_S)):
                        sign = -sign
                        iq = level * sign
                        m.send_set_iq(iq)
                        reversals += 1

                deadline += dt
                slack = deadline - time.monotonic()
                if slack > 0:
                    time.sleep(slack)
                else:
                    deadline = time.monotonic()
            if aborted:
                break
        m.send_set_iq(0.0)
        if not aborted:
            print(f"  {len(samples)} samples, {reversals} reversals, "
                  f"|vel| stayed within {V_BAND_CNT_S:,.0f} cnt/s")

    finally:
        try:
            m.send_set_iq(0.0)
            time.sleep(0.05)
            m.send_stop()
        except Exception as e:
            print(f"  [warn] stop sequence: {e}")
        m.stop_listening()

    for s_ in samples:
        csv_writer.writerow([name, f"{s_[0]:.5f}", f"{s_[1]:.4f}", f"{s_[2]:.0f}",
                             f"{s_[3]:.6f}",
                             f"{s_[4]:.4f}" if s_[4] is not None else ""])
    if aborted or len(samples) < 200:
        print(f"  Motor {name}: insufficient data ({len(samples)} samples).")
        return None
    return samples


def segment_accelerations(samples):
    """
    Split samples into constant-current segments and fit a quadratic to
    POSITION within each, returning one (current, accel, v_mid) per segment.

    Position is the raw integer measurement, so a quadratic through all of a
    segment's samples recovers acceleration without ever differentiating --
    which is what makes this robust to both the 1-count quantisation and to
    repeated frame timestamps (the old velocity-based fit divided by those and
    returned nan).
    """
    t = np.array([s[0] for s in samples])
    iq = np.array([s[1] for s in samples])
    pos = np.array([s[2] for s in samples])
    bus = np.array([s[3] for s in samples])

    edges = np.flatnonzero(np.diff(iq) != 0) + 1
    starts = np.concatenate(([0], edges))
    ends = np.concatenate((edges, [len(iq)]))

    out = []
    for a, b in zip(starts, ends):
        if iq[a] == 0.0 or (b - a) < MIN_SEG_SAMPLES:
            continue
        trim = int((b - a) * FIT_TRIM)
        sl = slice(a + trim, b - trim)
        if sl.stop - sl.start < MIN_SEG_SAMPLES:
            continue
        # Deduplicate by frame timestamp: repeated frames carry no new position
        # information and would weight the fit toward whatever was sampled twice.
        tb, pb = bus[sl], pos[sl]
        keep = np.concatenate(([True], np.diff(tb) > 0))
        tb, pb = tb[keep], pb[keep]
        if len(tb) < MIN_SEG_SAMPLES:
            continue
        tau = tb - tb[0]
        # p = p0 + v0*tau + a*tau^2/2
        A = np.vstack([np.ones_like(tau), tau, 0.5 * tau ** 2]).T
        coef, *_ = np.linalg.lstsq(A, pb, rcond=None)
        _, v0, accel = coef
        if not np.isfinite(accel):
            continue
        v_mid = v0 + accel * tau[-1] / 2.0
        out.append((iq[a], accel, v_mid))
    return np.array(out) if out else None


def report(name, seg):
    """Joint fit of a = G*i - b*v - c*sign(v) over every segment."""
    i, a, v = seg[:, 0], seg[:, 1], seg[:, 2]
    A = np.vstack([i, -v, -np.sign(v)]).T
    sol, *_ = np.linalg.lstsq(A, a, rcond=None)
    G, b_visc, c = sol
    resid = a - A @ sol
    rms = float(np.sqrt(np.mean(resid ** 2)))

    print(f"\n  Motor {name} RESULT   ({len(seg)} segments)")
    print(f"    plant gain G       = {G:,.0f} cnt/s^2 per A")
    print(f"                       = {G / ENC_PULSE_NBR:,.1f} rev/s^2 per A")
    print(f"    viscous drag b     = {b_visc:,.2f} 1/s "
          f"({b_visc * ENC_PULSE_NBR / G:.4f} A per rev/s)")
    print(f"    Coulomb friction   = {c:,.0f} cnt/s^2 ({c / G:+.4f} A equivalent)")
    print(f"    fit residual (rms) = {rms:,.0f} cnt/s^2 "
          f"({100 * rms / max(abs(a).max(), 1):.1f}% of range)")

    # Per-level accel/current: the linearity check the joint fit can hide.
    print(f"    {'level':>7s} {'n':>4s} {'accel/A both dirs':>19s} {'+dir':>11s} {'-dir':>11s}")
    for level in sorted({abs(x) for x in i}):
        mp = a[(i == level)]
        mm = a[(i == -level)]
        if not len(mp) or not len(mm):
            continue
        both = (np.median(mp) - np.median(mm)) / (2 * level)
        print(f"    {level:7.3f} {len(mp) + len(mm):4d} {both:19,.0f} "
              f"{np.median(mp) / level:11,.0f} {np.median(mm) / -level:11,.0f}")

    T = 0.0045
    print(f"    KV_SLOPE small-signal stability limit at T={1000 * T:.1f} ms: "
          f"{np.pi / (2 * G * T):.2e}")
    return G


def main():
    os.makedirs(LOG_DIR, exist_ok=True)
    log_path = os.path.join(LOG_DIR, "plant_gain.csv")
    suffix = 2
    while os.path.exists(log_path):
        log_path = os.path.join(LOG_DIR, f"plant_gain_{suffix}.csv")
        suffix += 1

    print(__doc__.split("Run from software/")[0].strip()[:0] or "", end="")
    print("PLANT GAIN MEASUREMENT")
    print(f"  levels {LEVELS_A} A, current reverses at "
          f"+/-{V_BAND_CNT_S:,.0f} cnt/s ({V_BAND_CNT_S / ENC_PULSE_NBR:.1f} rev/s), "
          f"{LEVEL_DURATION_S:.1f}s per level")
    print(f"  watchdog at {WATCHDOG_VEL_CNT_S:,.0f} cnt/s "
          f"({WATCHDOG_VEL_CNT_S / ENC_PULSE_NBR:.0f} rev/s)")
    print("\n  *** BOTH SHAFTS MUST BE FREE AND UNTOUCHED -- this commands real")
    print("      torque with no position feedback. Starting in 3 s. ***")
    time.sleep(3.0)

    csv_file = open(log_path, "w", newline="")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(["motor", "t_s", "iq_cmd", "pos", "bus_time", "iq_readback"])

    results = {}
    try:
        for name, node_base in MOTORS:
            samples = measure_motor(name, node_base, csv_writer)
            if samples is not None:
                seg = segment_accelerations(samples)
                if seg is None or len(seg) < 6:
                    print(f"  Motor {name}: too few usable segments to fit.")
                else:
                    results[name] = report(name, seg)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        csv_file.close()
        print(f"\nRaw samples: {log_path}")

    if len(results) == 2:
        print(f"\nG(A) / G(B) = {results['A'] / results['B']:.2f} "
              "-- how well matched the two shafts are. Far from 1.0 means the "
              "two sides need different gains, which nothing in teleop_test.py "
              "currently accounts for.")


if __name__ == "__main__":
    sys.exit(main())
