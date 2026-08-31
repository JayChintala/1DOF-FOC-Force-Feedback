"""
Teleop test -- velocity-driven slave with current-based force reflection.

Motor B (ESC 2) is the CONTROLLER: it's spun by hand and drives nothing on its own. 
Motor A (ESC 1) is the ROBOT: it tries to match Motor B's VELOCITY (not position).

Channel 1 -- velocity drives the slave (A tracks B's speed), PD on
velocity error:
    verr      = vel_B - vel_A
    accel_err = d(verr)/dt, EMA-filtered
    iq_a_cmd  = clamp(IQ1_MAX_A*tanh(KV_SLOPE*verr/IQ1_MAX_A) + KD_SLOPE*accel_err,
                       -IQ1_MAX_A, IQ1_MAX_A)

The plant here (current -> torque -> acceleration -> velocity) is
essentially an integrator. A pure-proportional (P-only) controller closed
around an integrator plant WILL ring once gain*loop-delay crosses a
threshold -- that's what every oscillation so far has actually been
(confirmed: vel_a rang 2.4-7x harder than vel_b regardless of B-side
damping, i.e. Motor A oscillating on its own, chasing zero velocity).
Every fix before this one (output filtering, velocity filtering) was
trying to patch that with low-pass filters, which remove high-frequency
content but don't add phase margin the way real derivative action does.
KD_SLOPE adds a term that opposes the RATE of change of the velocity
error specifically -- it engages hard during a fast oscillatory swing and
barely at all during a slow, steady push, which is what lets KV_SLOPE be
raised again without reintroducing the ring.

tanh (rather than a plain linear KV*verr on the P term) still gives a
steep initial ramp for small verr -- most use is at low speed -- while
its slope naturally decreases toward the IQ1_MAX_A ceiling instead of
ramping linearly into a hard clip.

A never corrects any position offset accumulated while blocked -- once free
again it matches B's speed going forward but does not catch back up to
B's absolute position. 

Channel 2 -- Motor A's actual sensed current is reflected onto Motor B,
so whatever resistance the robot (A) is fighting gets felt on the
controller (B), plus a damping term on B's own velocity:
    iq_a_filt = EMA(iq_a_readback)
    iq_b_cmd  = clamp(FEEDBACK_GAIN * iq_a_filt - KB_DAMP * vel_b_filt,
                       -IQ2_MAX_A, IQ2_MAX_A)

CORRECTION: these two channels are NOT actually independent once you
account for the physical hardware, only in the software's data-flow.
verr (which drives Channel 1) depends on vel_b; Channel 1's output drives
Channel 2's input (iq_a_filt); Channel 2's output physically spins Motor
B, which changes vel_b right back -- a genuine closed loop through the
hardware, with no term anywhere damping B's OWN velocity independent of
what A is doing. Confirmed on the bench: holding B tightly (but not with
literally infinite rigidity) still oscillated once slightly disturbed --
a sustained ~5Hz ring with vel_a/vel_b swinging tens of thousands of
counts/s, i.e. feedback howl through the A<->B loop, not local noise/lag
in Channel 1 alone. KB_DAMP adds real damping directly on the part of the
loop that had none, without touching the steady-state resistance you
feel from a sustained push (that still comes entirely from
FEEDBACK_GAIN * iq_a_filt).

LATENCY INSTRUMENTATION (diagnostic columns in the CSV; no control effect):
enc_age_a/b, iq_age_a, enc_lag_a/b, fresh_a/b, loop_dt. Ages are measured
from the KERNEL's receive timestamp, not the parse timestamp, because
telemetry_rate_probe.py showed those differ: the CAN path itself is clean
(1000 frames/s per ID, 1.00 ms gaps, zero device drops, a new encoder
sample on 100% of 192 Hz polls), but because can_interface.py opens one
unfiltered socket per motor, 2.7% of frames are PARSED more than 5 ms late
(p99 ~33 ms, worst 48 ms). The stale-telemetry check below cannot see that
-- it compares against enc_count_time, which is set at parse time, so a
frame handled 40 ms late still looks fresh. enc_lag_a/b isolate exactly
that component. FIXED: can_interface.py now shares one kernel-filtered
socket and one dispatch thread across all motors instead of one unfiltered
socket each, which took the handling lag to a p99 of 0.205 ms and max
0.469 ms with zero frames over 5 ms, measured under this loop's full duty
cycle. enc_lag_a/b stay in the log as the regression check.

WHAT WAS FIXED (this change): the dominant lag was the two EMAs --
VEL_FILTER_ALPHA tau ~17 ms plus IQ_A_CMD_FILTER_ALPHA tau ~42 ms at the
measured 165 Hz loop rate, ~60 ms total, which puts the usable
gain-crossover near 1/(2*pi*0.06) = 2.6 Hz against a ring measured at
5.7-6.9 Hz in every unstable run. The output filter is now off, the velocity
filter is down to tau ~3.3 ms, velocity is differenced over encoder frame
timestamps instead of the jittery loop period, and the loop is paced to a
deadline instead of sleeping a fixed dt after a variable amount of work.
Remaining systematic lag is ~6-8 ms, i.e. a crossover limit around 20 Hz
rather than 2.6 Hz. Checked on the recorded logs before changing anything:
at rest the lighter filtering raises velocity noise from 16 to 30 cnt/s,
which at KV_SLOPE=1.8e-5 is 0.0005 A of command jitter -- noise floor.

SECOND LATENCY PASS (500 Hz loop): raising KV_SLOPE to 1e-4 traded the old
5.7 Hz ring for a 25.1 Hz one -- 97% of Motor A's velocity energy in
18-35 Hz, at the same amplitude as the original ring (3.08M vs 3.24M), and
saturated: mean |iq_a_cmd| 0.746 A with 39.6% of samples pinned above
0.99 A and 44.7 sign changes/s, i.e. a bang-bang limit cycle rather than a
linear ring. 25 Hz is exactly the phase-crossover limit of the ~6 ms lag
that was left (1/(2*pi*0.006) = 26.5 Hz), so the mode was the delay budget,
not the gain per se. This pass cuts that budget to ~4 ms (limit ~40 Hz) by
running the loop at 500 Hz and shortening the velocity filter -- see
CONTROL_RATE_HZ and VEL_FILTER_ALPHA.

Worth knowing where the felt vibration actually comes from, measured on
that run: iq_b_cmd -- the current in the hand-held shaft -- was 84.7%
18-35 Hz buzz and only 12.4% useful 0.3-3 Hz signal, and 90% of that buzz
arrived through the KB_DAMP * vel_b_filt term rather than the reflected
force term (IQ_FILTER_ALPHA already strips the buzz off the force path).
Shortening VEL_FILTER_ALPHA in this pass makes vel_b_filt pass 25 Hz MORE
freely, so if the ring survives, expect the buzz in the hand to feel worse
even though Channel 1 is objectively faster. Filtering vel_b_filt inside
the damping term would mask that, but it is masking, not a fix.

RESULT of the first latency pass, at the unchanged KV_SLOPE=1.8e-5, across four
runs (logs/teleop_KVs1.8e-05_FB-1{,_2,_3,_4}.csv): vel_a RMS 27526 -> 3731,
peak 123501 -> 18751, and the share of Motor A's velocity energy in the
3-10 Hz ring band 99% -> 16% while the 0.3-3 Hz hand-motion band went
1% -> 64%. vel_a RMS (3731) also fell BELOW vel_b (4495) for the first
time -- A is following now, not oscillating on its own, which is the
opposite of the 2.4-7x ratio noted higher up in this docstring. Only then
was KV_SLOPE raised, see its comment below.

There's no need for an artificial bias current on either motor: Motor
A's Iq command is never idle-zero by design (it's whatever the velocity
controller computes), so IQ_READBACK on A is meaningful without one, and
there's no startup lurch.
"""

import csv
import math
import os
import sys
import time

from can_interface import MotorCANInterface
from plot_run import plot_teleop_log

# ---- Nodes ----
NODE_BASE_A = 0x000    # Motor A -- the "robot" (slave, position-tracking)
NODE_BASE_B = 0x020    # Motor B -- the "controller" (master, hand-driven)

# ---- Channel 1: velocity -> Motor A (PD on velocity error) ----
# History: KV_SLOPE=0.0001 (P-only) rang badly. Adding a D term made it
# WORSE (KD_SLOPE=0.000005 + KV_SLOPE=0.00001 hit 198k cnt/s peak, the
# worst yet) -- accel_err_filt is a numerical double-derivative of
# position, which amplifies encoder/timing noise by roughly 1/dt^2;
# confirmed by zeroing KD_SLOPE at the same KV_SLOPE=0.00001, which
# passed the let-go-completely test with NO oscillation (but very weak
# feedback, as expected -- 0.00001 is the original "too weak" baseline).
# D term stays at 0 until it can be redone with a properly (heavily)
# filtered acceleration estimate rather than raw double-differencing.
# THE SMALL-INCREMENTS APPROACH IS RETIRED. It was fighting the wrong
# problem: at 0.000018 a typical push (p75 |verr| = 2763 cnt/s, measured
# from logs/teleop_KVs1.8e-05_FB-1_4.csv) commands 0.05 A, and even the
# hardest shove in that run only reached 0.31 A. No increment of a gain
# that small produces force, and every past increment large enough to be
# felt also rang -- because the loop carried ~60 ms of phase lag, which
# capped usable crossover near 2.6 Hz while the ring sat at 5.7-6.9 Hz.
# The lag is now ~6 ms (velocity filter tau 3.3 ms + ~2.5 ms sampling +
# 0.06 ms RX handling), i.e. a crossover limit near 20-25 Hz and roughly
# 10x the gain headroom, so this jumps 5.6x in one step instead of
# creeping. Sized from that same log's measured hand speeds:
#   |verr| p75 2763 cnt/s -> 0.27 A    p90 5111 -> 0.47 A
#   |verr| p95 6209       -> 0.55 A    max 17950 -> 0.95 A
# Predicted stability edge is ~1.8e-4, so this sits comfortably inside it.
# If it still feels light, 1.5e-4 is the next step; do NOT go to 2.5e-4 --
# there the tanh saturates everything above p90 to 0.86-1.0 A and the feel
# loses its gradation (a wall instead of a force).
# NOTE the real ceiling is probably ~0.8 A, not IQ1_MAX_A's 1.0:
# NOMINAL_CURRENT_A = 0.8 in the firmware's pmsm_motor_parameters.h.
KV_SLOPE = 0.00003       # A/(count/s), initial P-term slope near verr=0.
                         # 1e-4 -> 3e-5. NOT a retreat: force no longer comes
                         # from this term at all, it comes from the hand-torque
                         # feedforward below. The measured small-signal limit is
                         # pi/(2*G*T) = 4.5e-5 (G = 7.7e6 measured, T = 4.5 ms),
                         # and 1e-4 was 2.2x OVER it -- which is why every run
                         # at that value ended in a limit cycle. 3e-5 sits at
                         # 66% of the limit, leaving margin for the +-4.4%
                         # uncertainty in G. This term's remaining job is
                         # velocity tracking and damping, not force.
IQ1_MAX_A = 1.0          # hard clamp/asymptote on Motor A's commanded
                         # current -- the "door's" max resistance.
                         # RESOLVED: the old note here guessed that
                         # oscillation, via the output filter, was what
                         # capped peak commands near 0.6 A. The filter half
                         # was right -- deleting it (IQ_A_CMD_FILTER_ALPHA
                         # 0.12 -> 1.0) took peak iq_a_cmd from 0.598 A to
                         # 0.85 A at an unchanged gain, so it really was
                         # preventing a fast-reversing command from settling
                         # at an extreme. The oscillation half was wrong:
                         # the ring is gone now and peak current is set by
                         # KV_SLOPE * verr, nothing else.
                         # CAUTION: 1.0 is above the firmware's own limit --
                         # NOMINAL_CURRENT_A = 0.8 in
                         # STM32/MCWorkbench/Inc/pmsm_motor_parameters.h.
                         # Readback has been seen at 0.83-0.85 A so it is
                         # not a hard clip on the Iq reference path, but do
                         # not count on commands above ~0.8 A being honoured.

# D term: opposes the RATE OF CHANGE of the velocity error (relative
# acceleration), not the error itself. Raised 10x from 0.0000005 -- that
# value was too small to have any visible braking effect against the
# accelerations actually seen during the ring (tens of thousands of
# counts/s^2). ACCEL_FILTER_ALPHA smooths the (noisy, double-differenced)
# acceleration estimate before use. Both remain unvalidated -- tune
# KD_SLOPE first if oscillation persists (higher damps harder) or if the
# feel goes mushy/laggy (lower).
KD_SLOPE = 0.0             # A/(count/s^2) of relative acceleration -- TEMPORARILY
                            # zeroed as a diagnostic: last run (KD_SLOPE=0.000005)
                            # got WORSE (peak vel_a 198k cnt/s, worst yet) even
                            # though KV_SLOPE was cut to its most conservative
                            # value. accel_err_filt is a numerical double-
                            # derivative of position, which amplifies encoder/
                            # timing noise by roughly 1/dt^2 -- suspect the D
                            # term is injecting noise-driven commands rather
                            # than damping. Zeroing it isolates whether D is the
                            # problem before touching anything else.
ACCEL_FILTER_ALPHA = 0.15

# Low-pass filter on the OUTPUT command (iq_a_cmd), on top of the PD terms
# above. DISABLED (1.0 = pass-through). At 0.12 this was an EMA with a time
# constant of ~42 ms at the measured loop rate -- by far the largest single
# phase lag in the loop, and phase lag is what sets the gain ceiling:
# 1/(2*pi*0.06) ~ 2.6 Hz of usable crossover against a ring measured at
# 5.7-6.9 Hz in every unstable run. It was added to suppress velocity noise
# that came from dividing by a jittery loop period; that noise source is
# gone now (velocity is differenced over encoder timestamps, see
# MIN_VEL_DT_S), so the filter is no longer paying for itself. Dial it back
# down toward ~0.5 only if the command turns out to be genuinely noisy --
# and prefer VEL_FILTER_ALPHA below for that, since filtering the input is
# cheaper in phase than filtering the output.
IQ_A_CMD_FILTER_ALPHA = 1.0

# EMA: vel_filt = ALPHA*vel_raw + (1-ALPHA)*vel_filt_prev.
# 0.25 -> 0.6 -> 0.7. tau = dt*(1-ALPHA)/ALPHA, so at the 2 ms loop period
# 0.7 is tau ~0.86 ms, against ~3.3 ms for 0.6 at the old 5 ms period: a 4x
# cut in this term's contribution to phase lag, which is the whole point of
# raising CONTROL_RATE_HZ alongside it.
# Not taken to 1.0 (no filter): velocity is an integer count difference over
# a ~2 ms window, so it moves in steps of 1/0.002 = 500 cnt/s, which at
# KV_SLOPE=1e-4 is 0.05 A of command staircase. Measured at rest the encoder
# is completely quiet (zero count changes over 6 s, so zero dither noise) --
# the quantisation only appears while moving, and 0.7 averages just enough to
# take the edge off it without giving back the phase.
VEL_FILTER_ALPHA = 0.7

# Velocity is differenced over the interval between the ENCODER FRAMES the
# two positions came from (their kernel receive timestamps), NOT over the
# control loop's own period. Those two are not the same thing: loop_dt was
# measured at 5.86 ms median but 7.15 ms p95 and 29.9 ms max, so dividing by
# the loop period injected +-20% of timing error straight into every
# velocity sample -- noise that no amount of output filtering can undo,
# because it is proportional to the signal. A position and its frame
# timestamp travel together, so this estimate is immune to when the loop
# happens to read them. MIN_VEL_DT_S guards the degenerate case where two
# iterations read the same frame (dt = 0).
# Deliberately left at 2 ms after the move to a 500 Hz (2 ms) loop, rather
# than shrunk to match: it is what bounds the quantisation above. Frames
# arrive at 1 kHz, so with loop jitter roughly 65% of iterations find a frame
# at least 2 ms newer than the one the last estimate used and produce a fresh
# velocity; the rest hold the previous value for one iteration (~0.7 ms of
# lag on average). Shrinking this to 1 ms would update every iteration but
# double the quantisation step to 1000 cnt/s = 0.1 A -- the wrong trade.
MIN_VEL_DT_S = 0.002

WATCHDOG_VEL_LIMIT_CNT_S = 200_000.0   # counts/s, either shaft -- genuine
                                        # runaway-speed protection. No
                                        # position-error watchdog: with a
                                        # pure velocity controller, a large
                                        # position gap while A is blocked
                                        # is the expected steady state, not
                                        # a fault.

# sign_check_test.py - Increasing Iq = 1, Decreasing Iq = -1
SIGN = 1

# ---- Channel 2: Motor A's current -> Motor B ----
# err (pull A toward B) -- feeding that same sign to B pushes B further
# AWAY from A (positive feedback/runaway, confirmed on the bench: holding
# A and spinning B kept accelerating B in the same direction instead of
# resisting). Negating it makes B get pulled back toward A's actual
# position instead, which is the correct restoring/resistance feel.
FEEDBACK_GAIN = -1.0    # A/A -- how much of A's sensed current is felt on B
IQ2_MAX_A = IQ1_MAX_A   # match Motor A's ceiling -- the point of Channel 2 is
                         # to feel exactly what A feels, so there's no reason
                         # for B's clamp to be lower than A's. (Was capped
                         # lower, 0.3 then 0.7, purely as an extra caution
                         # while validating this on the bench -- not because
                         # the hardware itself differs between the two motors.)
IQ_FILTER_ALPHA = 0.034  # EMA low-pass on Motor A's IQ_READBACK before mirroring.
                         # 0.15 -> 0.08 (reported jittery/inconsistent on the
                         # bench; heavier smoothing traded lag for a steadier
                         # felt resistance) -> 0.034. That last step is NOT a
                         # retune: these alphas are per-iteration, so raising
                         # CONTROL_RATE_HZ from 200 to 500 would silently have
                         # cut this filter's time constant from ~57 ms to
                         # ~23 ms. 0.034 at a 2 ms loop restores tau ~57 ms,
                         # keeping Channel 2's behaviour exactly as it was
                         # validated so this change stays a single-variable
                         # test of Channel 1's phase lag.
                         # tau = dt*(1-ALPHA)/ALPHA -- rescale this whenever
                         # CONTROL_RATE_HZ moves, or the felt force quietly
                         # changes character underneath you.

# Damping on Motor B's OWN velocity, independent of Channel 1 -- the
# actual fix for the ~5Hz, huge-amplitude oscillation, which turned out
# to be a closed loop through the physical hardware (vel_b -> Channel 1
# -> iq_a -> Channel 2 -> iq_b -> physically spins B -> vel_b), not a
# Channel-1-only problem. Nothing upstream of this damped B's own motion.
# Sized so it's small relative to a normal push (a few thousand counts/s)
# but substantial at the oscillation's observed amplitude (tens of
# thousands of counts/s) -- tune this first if oscillation persists or if
# it now feels like B is dragging during a normal push.
KB_DAMP = 0.00002       # A/(count/s) of Motor B's own filtered velocity


# ---- Hand-torque observer: the force channel that is NOT loop gain ----
# THE PROBLEM THIS SOLVES. Force on Channel 1 came only from KV_SLOPE, a
# proportional gain on velocity error, which raises loop gain equally at
# every frequency. Force is wanted at 0.3-3 Hz; the instability lives at
# the phase-crossover frequency. Raising KV buys both in fixed proportion,
# so no value of it gives useful force and stability at once. Measured:
# the small-signal limit is KV = pi/(2*G*T) = 4.5e-5 at G = 7.7e6 and
# T = 4.5 ms, where a typical push (|verr| p75 = 2763 cnt/s) yields only
# 0.08 A. Running 1e-4 gave good force ONLY because the loop was 2.2x over
# its stability limit: disturbances grew until tanh compression dropped the
# effective gain to 4.0e-5 (measured), and that equilibrium IS the 35 Hz
# limit cycle. The oscillation was the mechanism delivering the force.
#
# HOW THIS WORKS INSTEAD. The plant is dv/dt = G*(i + tau_h), where tau_h
# is any external torque expressed in amps of equivalent motor current. So
# the hand's torque on Motor B is observable from B's own motion and the
# current B is already being commanded:
#     tau_h = (dv_b/dt)/G_B - i_b
# Rather than differentiate velocity (which amplifies quantisation by
# 1/dt^2 -- the mistake the KD_SLOPE experiment made), this is computed in
# the standard disturbance-observer form, which needs no derivative at all.
# With x = v_b/G_B and a first-order cutoff w:
#     tau_h_hat = w * (x - LPF_w(x)) - LPF_w(i_b)
# because s*w/(s+w) = w*(1 - w/(s+w)), i.e. the high-pass of x times w.
#
# WHY IT IS NOT POSITIVE FEEDBACK. Channel 2 drives B with -iq_a_filt, so
# raising A's current raises the -LPF(i_b) term -- which looks like a loop.
# It is not, provided G_B is right: when i_b changes, v_b responds, and the
# acceleration term cancels the current term exactly. Hold B still against
# a pushing motor and the observer correctly reports that the hand must be
# supplying the balancing torque; let B go and it correctly reports zero.
# Residual loop gain is roughly the fractional error in G_B, measured at
# 4.4% -- hence the insistence on measuring G before building this.
#
# WHAT IT BUYS. tau_h_hat is FEEDFORWARD: it adds no loop gain, so force
# and stability are finally separate knobs. It also works at zero velocity,
# which the velocity channel fundamentally cannot -- a static push produces
# no verr at all, but does produce a current/acceleration imbalance.
G_A_CNT_S2_PER_A = 7_622_000    # measured, plant_gain_test.py, 5 runs, +-4.4%
G_B_CNT_S2_PER_A = 7_915_000    # measured, same run set. G(A)/G(B) = 0.96.
OBS_CUTOFF_HZ = 20.0            # observer bandwidth. Higher = more responsive
                                 # force but more noise: the high-pass gains up
                                 # velocity quantisation by w, giving about
                                 # 0.008 A of noise here, on top of ~0.016 A of
                                 # 14/rev cogging ripple that no filter removes.
                                 # Against 0.2-0.5 A of real push that is a
                                 # noise floor near 5%.
FF_GAIN = 1.0                   # amps of A per amp of estimated hand torque.
                                 # 1.0 = you feel exactly your own push, which
                                 # is the design target.
                                 # Shipped at 0.3 first to verify the sign,
                                 # since a sign error here drives A the wrong
                                 # way and Channel 2 then ASSISTS the push.
                                 # Sign CONFIRMED on the bench
                                 # (logs/teleop_KVs3e-05_FB-1.csv): tau_h agrees
                                 # in sign with vel_b on 93.5% of samples where
                                 # |vel_b| > 2000 cnt/s, corr(tau_h, vel_b) =
                                 # +0.840. Raised to 1.0 after that run.
                                 # Verified safe to raise: the observer's output
                                 # is spectrally clean -- tau_h carries 95% of
                                 # its energy in the 0.3-3 Hz band a hand
                                 # actually moves in and only 1% in the 18-35 Hz
                                 # band Motor A still shakes in, so this gain
                                 # scales force without scaling the shake. At
                                 # 0.3 the feedforward already supplied a median
                                 # 37% of A's command, with tau_h reading p50
                                 # 0.129 A / p90 0.412 A / peak 0.663 A of hand
                                 # torque.
FF_MAX_A = 1.0                   # clamp on the feedforward term alone, so a bad
                                 # observer estimate cannot exceed what the
                                 # velocity channel could have commanded anyway.

# 200 -> 500 Hz. Telemetry has always arrived at 1 kHz, so this costs
# nothing on the wire and halves the sampling contribution to phase lag.
# Verified achievable before changing it: with this loop's full duty cycle
# (poll both nodes, two SET_IQ sends, write a 21-column CSV row) a 500 Hz
# deadline-paced loop measured 2.000 ms median / 2.033 ms p95 / 3.084 ms max
# with 100% fresh telemetry. 1000 Hz also held (999 Hz achieved), but at a
# 1 ms differencing window the velocity quantisation triples, so 500 Hz is
# the better point -- see MIN_VEL_DT_S.
# Lag budget after this change: ~0.86 ms velocity filter + ~0.7 ms estimate
# hold + ~1 ms window centring + ~1 ms sampling + ~0.5 ms telemetry ~= 4 ms,
# for a phase-crossover limit near 40 Hz, against ~6 ms and ~26 Hz before.
# The 25 Hz limit cycle observed at KV_SLOPE=1e-4 sat right at that old
# limit, which is what this is meant to move out of the way.
CONTROL_RATE_HZ = 500

# Discretisation correction on the observer's acceleration term. The
# derivative-free form w*(x - LPF(x)) loses exactly w*dt/2 of the
# acceleration when implemented with a matched-pole discrete LPF -- a half
# sample of integration error, not a modelling approximation. Verified in
# simulation: the deficit is 0.122 at 20 Hz/500 Hz, 0.063 at 10 Hz/500 Hz,
# 0.231 at 40 Hz/500 Hz and 0.062 at 20 Hz/1 kHz, against pi*f*dt of 0.126,
# 0.063, 0.251, 0.063 -- constant in amplitude, so a single factor removes
# it exactly. Derived from the constants rather than hardcoded so it stays
# correct if OBS_CUTOFF_HZ or CONTROL_RATE_HZ is changed. Without it the
# observer under-reads the hand by 12% whenever Motor B is accelerating
# (the static-hold case is unaffected -- there the term is zero).
OBS_COMP = 1.0 / (1.0 - math.pi * OBS_CUTOFF_HZ / CONTROL_RATE_HZ)
STALE_TIMEOUT_S = 0.05  # telemetry older than this -> fail safe
STALE_WARN_EVERY_S = 1.0
PRINT_EVERY_S = 0.1     # throttle console output so it can't bottleneck the loop

LOG_DIR = "logs"


def clamp(value, lo, hi=None):
    if hi is None:
        lo, hi = -lo, lo
    return max(lo, min(hi, value))


def main():
    os.makedirs(LOG_DIR, exist_ok=True)
    base_name = f"teleop_KVs{KV_SLOPE:g}_FB{FEEDBACK_GAIN:g}"
    log_path = os.path.join(LOG_DIR, f"{base_name}.csv")
    suffix = 2
    while os.path.exists(log_path):
        log_path = os.path.join(LOG_DIR, f"{base_name}_{suffix}.csv")
        suffix += 1

    a = MotorCANInterface(channel="can0", node_base=NODE_BASE_A)
    b = MotorCANInterface(channel="can0", node_base=NODE_BASE_B)
    a.start_listening()
    b.start_listening()

    csv_file = open(log_path, "w", newline="")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(
        ["t_s", "pos_a", "pos_b", "d_a", "d_b", "err", "vel_a", "vel_b",
         "iq_a_cmd", "iq_a_readback", "iq_a_filt", "iq_b_cmd", "iq_b_readback",
         # Latency instrumentation -- diagnostic only, no effect on control.
         # enc_age_*/iq_age_a: TRUE age (s) of the telemetry behind this
         # iteration's torque command, measured from the kernel's receive
         # timestamp. enc_lag_*: how much of that age is this process
         # handling the frame late (parse time - arrival time) rather than
         # the loop not having polled yet -- telemetry_rate_probe.py measured
         # a p99 of ~33 ms there, and the staleness check below cannot see
         # it. fresh_*: 1 when a NEW encoder sample arrived since the
         # previous iteration. loop_dt: measured loop period vs
         # 1/CONTROL_RATE_HZ nominal.
         "enc_age_a", "enc_age_b", "iq_age_a", "enc_lag_a", "enc_lag_b",
         "fresh_a", "fresh_b", "loop_dt",
         # Hand-torque observer: its estimate, and the feedforward current it
         # produced. tau_h is in amps of equivalent Motor B current -- if the
         # sign of iq_a_ff ever disagrees with the direction of the push, that
         # is the FF_GAIN sign problem and it will be obvious here.
         "tau_h", "iq_a_ff"]
    )

    stale_count = 0
    rows_logged = 0
    # Telemetry-freshness tallies -- how many iterations saw a NEW encoder
    # sample. Declared out here so the end-of-run summary in `finally` can
    # report them even if the run aborts early.
    fresh_a_count = 0
    fresh_b_count = 0
    loop_dt_sum = 0.0

    try:
        print("Sending START to both...")
        a.send_start()
        b.send_start()
        time.sleep(0.2)
        a.send_set_iq(0.0)
        b.send_set_iq(0.0)
        time.sleep(0.3)

        ta0 = a.get_telemetry()
        tb0 = b.get_telemetry()
        if ta0["enc_count_unwrapped"] is None or tb0["enc_count_unwrapped"] is None:
            print("No encoder telemetry from one/both motors -- check CAN link/node_base.")
            return

        base_a = ta0["enc_count_unwrapped"]
        base_b = tb0["enc_count_unwrapped"]
        print(f"Baselines: A={base_a:.0f}  B={base_b:.0f} counts")
        print(f"Channel 1 (velocity->A): KV_SLOPE={KV_SLOPE} KD_SLOPE={KD_SLOPE} "
              f"IQ1_MAX={IQ1_MAX_A}A SIGN={SIGN}")
        print(f"Channel 2 (A current->B): FEEDBACK_GAIN={FEEDBACK_GAIN} "
              f"KB_DAMP={KB_DAMP} IQ2_MAX={IQ2_MAX_A}A")
        print(f"Logging to {log_path}")
        print("Spin B; A should follow. Hold A; feel resistance in B. Ctrl+C to stop.\n")

        dt = 1.0 / CONTROL_RATE_HZ
        next_deadline = time.monotonic()
        prev_pos_a = base_a
        prev_pos_b = base_b
        # Encoder-frame timestamps paired with prev_pos_* above -- the time
        # base for the velocity estimate. None until the first frame lands.
        prev_vel_time_a = None
        prev_vel_time_b = None
        vel_a_raw = 0.0
        vel_b_raw = 0.0
        prev_time = time.monotonic()
        start_time = prev_time
        vel_a_filt = 0.0
        vel_b_filt = 0.0
        iq_a_filt = 0.0
        iq_a_cmd_filt = 0.0
        prev_verr = 0.0
        accel_err_filt = 0.0
        # Hand-torque observer state: one low-pass on B's scaled velocity and
        # one on B's measured current. See the OBS_CUTOFF_HZ block above.
        obs_x_lpf = None       # None until the first velocity sample, so the
                               # filter starts at the real value instead of
                               # ramping up from zero and inventing a transient
        obs_i_lpf = 0.0
        tau_h_hat = 0.0
        last_stale_warn = 0.0
        last_print = 0.0
        watchdog_Atripped = False
        # Previous telemetry arrival timestamps, for the fresh_* flags --
        # a repeated timestamp means no new frame arrived this iteration.
        prev_ta_time = None
        prev_tb_time = None

        while True:
            ta = a.get_telemetry()
            tb = b.get_telemetry()
            now = time.monotonic()
            now_wall = time.time()  # telemetry timestamps are wall-clock

            pos_a = ta["enc_count_unwrapped"]
            pos_b = tb["enc_count_unwrapped"]
            ta_time = ta["enc_count_time"]
            tb_time = tb["enc_count_time"]
            iq_a = ta["iq_readback"]
            iq_a_time = ta["iq_readback_time"]
            # Kernel arrival timestamps -- the ones that measure real latency.
            ta_bus_time = ta["enc_count_bus_time"]
            tb_bus_time = tb["enc_count_bus_time"]
            iq_a_bus_time = ta["iq_readback_bus_time"]

            # Fail-safe on missing/stale encoder telemetry from EITHER motor.
            enc_stale = (
                pos_a is None or pos_b is None
                or ta_time is None or tb_time is None
                or (now_wall - ta_time) > STALE_TIMEOUT_S
                or (now_wall - tb_time) > STALE_TIMEOUT_S
            )
            if enc_stale:
                a.send_set_iq(0.0)
                b.send_set_iq(0.0)
                stale_count += 1
                if now - last_stale_warn > STALE_WARN_EVERY_S:
                    print(f"[warn] encoder telemetry stale/missing "
                          f"(stale_count={stale_count}) -- both motors zeroed.")
                    last_stale_warn = now
                # Re-base the pacing deadline: this iteration bailed out
                # before doing any control work, so there is nothing to catch
                # up on when telemetry returns.
                time.sleep(dt)
                next_deadline = time.monotonic()
                continue

            elapsed = now - prev_time

            # Velocity over the encoder frames' own timestamps -- see
            # MIN_VEL_DT_S. A new estimate is produced only when enough time
            # has passed since the frame the previous estimate used; until
            # then the previous raw value is held, rather than dividing a
            # tiny position delta by a tiny (or zero) interval.
            if ta_bus_time is not None:
                if prev_vel_time_a is None:
                    prev_pos_a, prev_vel_time_a = pos_a, ta_bus_time
                elif ta_bus_time - prev_vel_time_a >= MIN_VEL_DT_S:
                    vel_a_raw = (pos_a - prev_pos_a) / (ta_bus_time - prev_vel_time_a)
                    prev_pos_a, prev_vel_time_a = pos_a, ta_bus_time
            if tb_bus_time is not None:
                if prev_vel_time_b is None:
                    prev_pos_b, prev_vel_time_b = pos_b, tb_bus_time
                elif tb_bus_time - prev_vel_time_b >= MIN_VEL_DT_S:
                    vel_b_raw = (pos_b - prev_pos_b) / (tb_bus_time - prev_vel_time_b)
                    prev_pos_b, prev_vel_time_b = pos_b, tb_bus_time

            vel_a_filt = VEL_FILTER_ALPHA * vel_a_raw + (1 - VEL_FILTER_ALPHA) * vel_a_filt
            vel_b_filt = VEL_FILTER_ALPHA * vel_b_raw + (1 - VEL_FILTER_ALPHA) * vel_b_filt

            d_a = pos_a - base_a
            d_b = pos_b - base_b
            err = d_b - d_a       # logged for visibility only -- not used for control
            verr = vel_b_filt - vel_a_filt

            # D term: rate of change of verr (relative acceleration),
            # filtered -- see KD_SLOPE/ACCEL_FILTER_ALPHA comment above.
            if elapsed < 0.5 * dt:
                accel_err_raw = 0.0
            else:
                accel_err_raw = (verr - prev_verr) / elapsed
            accel_err_filt = (ACCEL_FILTER_ALPHA * accel_err_raw
                               + (1 - ACCEL_FILTER_ALPHA) * accel_err_filt)
            prev_verr = verr

            # Watchdog -- before computing/sending torque. Position error
            # (err) is NOT checked here: with a pure velocity controller, a
            # large/growing gap while A is blocked is the expected steady
            # state (the whole point of the door model), not a fault.
            if (abs(vel_a_filt) > WATCHDOG_VEL_LIMIT_CNT_S
                    or abs(vel_b_filt) > WATCHDOG_VEL_LIMIT_CNT_S):
                print(
                    f"\nWATCHDOG TRIPPED: vel_a={vel_a_filt:.0f}, "
                    f"vel_b={vel_b_filt:.0f} cnt/s. Zeroing both and aborting."
                )
                watchdog_tripped = True
                break

            # Hand-torque observer on Motor B. Uses B's MEASURED current, not
            # its command: the cancellation that keeps this from being a
            # feedback path depends on comparing real current against real
            # acceleration. Falls back to holding the previous estimate if
            # B's current telemetry is stale, rather than feeding a zero
            # (which would read as "the hand let go").
            iq_b_meas = tb["iq_readback"]
            iq_b_meas_time = tb["iq_readback_time"]
            iq_b_stale = (
                iq_b_meas is None or iq_b_meas_time is None
                or (now_wall - iq_b_meas_time) > STALE_TIMEOUT_S
            )
            if not iq_b_stale:
                obs_alpha = 1.0 - math.exp(-2.0 * math.pi * OBS_CUTOFF_HZ * dt)
                obs_x = OBS_COMP * vel_b_filt / G_B_CNT_S2_PER_A
                if obs_x_lpf is None:
                    obs_x_lpf = obs_x
                obs_x_lpf += obs_alpha * (obs_x - obs_x_lpf)
                obs_i_lpf += obs_alpha * (iq_b_meas - obs_i_lpf)
                # tau_h = w*(x - LPF(x)) - LPF(i_b), the derivative-free form.
                tau_h_hat = (2.0 * math.pi * OBS_CUTOFF_HZ * (obs_x - obs_x_lpf)
                             - obs_i_lpf)

            # Channel 1: velocity tracking (P with tanh saturation, plus the
            # still-zeroed D term) PLUS the hand-torque feedforward. The
            # feedforward is what produces force now; the velocity terms only
            # track and damp. Keeping them separate is the point -- the
            # feedforward adds no loop gain, so raising it cannot move the
            # stability boundary the way KV_SLOPE does.
            iq_a_p = IQ1_MAX_A * math.tanh(SIGN * KV_SLOPE * verr / IQ1_MAX_A)
            iq_a_d = SIGN * KD_SLOPE * accel_err_filt
            iq_a_ff = clamp(SIGN * FF_GAIN * tau_h_hat, FF_MAX_A)
            iq_a_cmd_raw = clamp(iq_a_p + iq_a_d + iq_a_ff, IQ1_MAX_A)
            iq_a_cmd_filt = (IQ_A_CMD_FILTER_ALPHA * iq_a_cmd_raw
                              + (1 - IQ_A_CMD_FILTER_ALPHA) * iq_a_cmd_filt)
            iq_a_cmd = iq_a_cmd_filt
            a.send_set_iq(iq_a_cmd)

            # Channel 2: Motor A's sensed current -> Motor B.
            iq_a_stale = (
                iq_a is None or iq_a_time is None
                or (now_wall - iq_a_time) > STALE_TIMEOUT_S
            )
            if iq_a_stale:
                iq_b_cmd = clamp(-KB_DAMP * vel_b_filt, IQ2_MAX_A)
            else:
                iq_a_filt = IQ_FILTER_ALPHA * iq_a + (1 - IQ_FILTER_ALPHA) * iq_a_filt
                iq_b_cmd = clamp(FEEDBACK_GAIN * iq_a_filt - KB_DAMP * vel_b_filt, IQ2_MAX_A)
            b.send_set_iq(iq_b_cmd)

            t_rel = now - start_time
            iq_a_act = ta["iq_readback"]
            iq_b_act = tb["iq_readback"]

            # Latency instrumentation -- measured, never fed back into control.
            fresh_a = 1 if ta_time != prev_ta_time else 0
            fresh_b = 1 if tb_time != prev_tb_time else 0
            prev_ta_time = ta_time
            prev_tb_time = tb_time
            fresh_a_count += fresh_a
            fresh_b_count += fresh_b
            loop_dt_sum += elapsed
            iq_age_a = (now_wall - iq_a_bus_time) if iq_a_bus_time is not None else None
            enc_age_a = (now_wall - ta_bus_time) if ta_bus_time is not None else None
            enc_age_b = (now_wall - tb_bus_time) if tb_bus_time is not None else None
            enc_lag_a = (ta_time - ta_bus_time) if ta_bus_time is not None else None
            enc_lag_b = (tb_time - tb_bus_time) if tb_bus_time is not None else None

            csv_writer.writerow(
                [f"{t_rel:.4f}", pos_a, pos_b, d_a, d_b, err,
                 f"{vel_a_filt:.1f}", f"{vel_b_filt:.1f}",
                 f"{iq_a_cmd:.4f}",
                 iq_a_act if iq_a_act is not None else "",
                 f"{iq_a_filt:.4f}",
                 f"{iq_b_cmd:.4f}",
                 iq_b_act if iq_b_act is not None else "",
                 f"{enc_age_a:.5f}" if enc_age_a is not None else "",
                 f"{enc_age_b:.5f}" if enc_age_b is not None else "",
                 f"{iq_age_a:.5f}" if iq_age_a is not None else "",
                 f"{enc_lag_a:.5f}" if enc_lag_a is not None else "",
                 f"{enc_lag_b:.5f}" if enc_lag_b is not None else "",
                 fresh_a, fresh_b, f"{elapsed:.5f}",
                 f"{tau_h_hat:.4f}", f"{iq_a_ff:.4f}"]
            )
            rows_logged += 1

            if now - last_print > PRINT_EVERY_S:
                print(
                    f"t={t_rel:5.1f}s  err={err:>8.0f}  "
                    f"iqA_cmd={iq_a_cmd:>7.3f}  tau_h={tau_h_hat:>7.3f}  "
                    f"iqA_ff={iq_a_ff:>7.3f}  iqB_cmd={iq_b_cmd:>7.3f}"
                )
                last_print = now

            prev_time = now

            # Deadline pacing: sleep until the NEXT tick, not for a fixed dt
            # after however long this iteration took. sleep(dt) made the
            # period dt + work_time, which measured 5.86 ms against a 5.00 ms
            # nominal (165 Hz vs 200 Hz) and drifted with logging load. If an
            # iteration overruns its deadline, skip ahead rather than trying
            # to catch up in a burst.
            next_deadline += dt
            slack = next_deadline - time.monotonic()
            if slack > 0:
                time.sleep(slack)
            else:
                next_deadline = time.monotonic()

        if not watchdog_tripped:
            pass

    except KeyboardInterrupt:
        print("\nStopping (interrupted)...")
    finally:
        csv_file.close()
        try:
            a.send_set_iq(0.0)
            b.send_set_iq(0.0)
            time.sleep(0.05)
            a.send_stop()
            b.send_stop()
        except Exception as e:
            print(f"[warn] error during stop sequence: {e}")
        a.stop_listening()
        b.stop_listening()
        print("CAN interfaces closed.")
        if rows_logged == 0:
            os.remove(log_path)
            print("No data logged -- log discarded.")
        else:
            # Latency summary -- the headline numbers from the new columns,
            # so they don't have to be dug out of the CSV by hand.
            loop_hz = rows_logged / loop_dt_sum if loop_dt_sum > 0 else 0.0
            frac_a = fresh_a_count / rows_logged
            frac_b = fresh_b_count / rows_logged
            print(f"\nLoop rate: {loop_hz:.0f} Hz measured "
                  f"(CONTROL_RATE_HZ={CONTROL_RATE_HZ} nominal)")
            print(f"New encoder sample on {100 * frac_a:.1f}% of iterations for A "
                  f"({loop_hz * frac_a:.0f} Hz effective), "
                  f"{100 * frac_b:.1f}% for B ({loop_hz * frac_b:.0f} Hz).")
            print("Firmware sends ENC_COUNT at 1000 Hz -- a large shortfall here "
                  "is dropped/late telemetry, not a slow shaft.")
            print(f"Log saved: {log_path}")
            png_path = plot_teleop_log(log_path)
            if png_path:
                print(f"Plot saved: {png_path}")


if __name__ == "__main__":
    sys.exit(main())
