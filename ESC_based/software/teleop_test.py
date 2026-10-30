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
import collections
import json
import shutil

from contact_spring import ContactSpring
from leader_comp import LeaderComp
import leader_cal
from estimators import AlphaBetaVelocity, ExternalTorqueObserver, HandTorqueObserver
from fit_follower_model import model_terms
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
IQ1_MAX_A = float(os.environ.get("TELEOP_IQ1_MAX_A", "1.0"))
                         # hard clamp/asymptote on Motor A's commanded
                         # current -- the "door's" max resistance.
                         # 2026-10-26: 0.8 A is the motor's thermal rating
                         # (user); with the contact spring, holds sit at the
                         # clamp, so run TELEOP_IQ1_MAX_A=0.8.
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
KB_DAMP = float(os.environ.get("TELEOP_KB_DAMP", "0.00002"))  # A/(count/s) of Motor B's own filtered velocity
# (C3, 2026-10-25: env override TELEOP_KB_DAMP so the leader damping can be
# stepped down with the tau_env felt path, which no longer echoes the
# follower's inertia; default unchanged.)


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
# B1 (ESC_based plan): drain EVERY TELEM frame each iteration instead of
# reading only the latest value, and log all of them, with the ESC's own
# mcu_time_us, to <log>_frames.csv. The control law still uses the newest
# frame (exactly what get_telemetry() returns), so with this on the loop's
# behaviour is unchanged; B2/B3 will feed the frames to the estimators.
# False keeps the original latest-value path for rollback. Override without
# editing: TELEOP_FRAME_QUEUE=1 python3 teleop_test.py
USE_FRAME_QUEUE = os.environ.get("TELEOP_FRAME_QUEUE", "0") == "1"
# B2: velocity estimator feeding vel_a_filt / vel_b_filt (tracking, damping,
# watchdog, observer). "legacy" = the original delta-pos over >= 2 ms of
# kernel time + EMA. "ab55" = estimators.AlphaBetaVelocity(55 Hz) on every
# frame against mcu_time_us, exactly 0 at rest -- offline (b2_eval.py) it
# was 3.4-3.9x quieter at constant speed with 2.6 ms lag vs legacy's 3.1-3.3.
# Needs USE_FRAME_QUEUE. Override: TELEOP_VEL_ESTIMATOR=ab55
VEL_ESTIMATOR = os.environ.get("TELEOP_VEL_ESTIMATOR", "legacy")
VEL_AB_BANDWIDTH_HZ = 55.0
# B3: leader hand-torque observer. "inline" = the original code below
# (nominal dt for both the LPF alpha and OBS_COMP). "class" =
# estimators.HandTorqueObserver, which reproduced the inline tau_h within
# 0.03 % RMS on four logs when fed the same nominal dt. OBS_G_SOURCE picks
# G_B: "old" (7,915,000, pre-direct-drive) or "A2" (9,672,000, measured
# 2026-10-20). OBS_DT_SOURCE: "nominal" (1/CONTROL_RATE_HZ) or "mcu" (the
# ESC-clock interval between B's newest frames; no new frame -> no update).
# Overrides: TELEOP_OBSERVER=class TELEOP_OBS_G=A2 TELEOP_OBS_DT=mcu
OBSERVER_IMPL = os.environ.get("TELEOP_OBSERVER", "inline")
OBS_G_SOURCE = os.environ.get("TELEOP_OBS_G", "old")
OBS_DT_SOURCE = os.environ.get("TELEOP_OBS_DT", "nominal")
G_B_A2_CNT_S2_PER_A = 9_672_000
# C2: shadow follower external-torque estimate, computed live and logged
# only (column tau_env, plus tau_env_N via SHADOW_N_PER_A) -- nothing
# here feeds control or the felt path. Exactly the estimator validated
# offline in c1_eval.py (2026-10-25): ExternalTorqueObserver at 20 Hz with
# A2's G_A, fed every follower frame with its own AB 55 Hz velocity and the
# current delayed SHADOW_ALIGN_FRAMES to match that velocity's ~2.6 ms lag.
# Sign is the brief's: positive = the environment pushes the follower toward
# +counts, i.e. tau_env = -(i - a/G). Needs USE_FRAME_QUEUE.
# Override: TELEOP_SHADOW_TAU_ENV=1
SHADOW_TAU_ENV = os.environ.get("TELEOP_SHADOW_TAU_ENV", "0") == "1"
G_A_A2_CNT_S2_PER_A = 9_025_000
SHADOW_ALIGN_FRAMES = 3
SHADOW_KT_NM_PER_A = 0.0646           # B7
SHADOW_R_CONTACT_M = 0.0757           # contact radius of the 3D-printed arm
# tau_env_N uses the force factor MEASURED at the follower tip against the
# scale (TEST_LOG 2026-10-28 static calibration: 1.044 N/A, R^2 0.995,
# levels 0.1-0.45 A), 22 % above Kt/r = 0.853 N/A. Valid below ~0.5 A, where
# the current measurement starts to break down.
SHADOW_N_PER_A = 1.044
# C2: optional fitted follower model (fit_follower_model.py): its G replaces
# G_A_A2_CNT_S2_PER_A and its friction + gravity terms are subtracted from
# the shadow estimate. Unset = the C1 estimator unchanged.
# Override: TELEOP_FOLLOWER_MODEL=follower_model_arm3d.json
FOLLOWER_MODEL_PATH = os.environ.get("TELEOP_FOLLOWER_MODEL", "")
                                      # (3D-printed arm, 2026-10-25; was 0.04)
STALE_TIMEOUT_S = 0.05  # telemetry older than this -> fail safe
STALE_WARN_EVERY_S = 1.0
PRINT_EVERY_S = 0.1     # throttle console output so it can't bottleneck the loop

LOG_DIR = "logs"
# Host-stall fix (confirmed 2026-10-22: SD-card write() calls block the main
# loop for up to 1.6 s). With TELEOP_LOG_DIR set (e.g. /dev/shm/teleop) the
# CSVs are written there during the run -- RAM, no SD stalls -- and moved,
# with the plot, into LOG_DIR at exit under the usual unique name. Unset =
# write straight to LOG_DIR as before.
LIVE_LOG_DIR = os.environ.get("TELEOP_LOG_DIR", "")

# C3: what the operator feels on Motor B.
#   "iq_a_filt" (default) = FEEDBACK_GAIN * EMA(follower measured current), the
#                           original path (2.8 Hz filter; in contact it echoes
#                           the follower's own push, see TEST_LOG A1).
#   "tau_env"             = FELT_K_F * shadow external-torque estimate (20 Hz,
#                           validated live in C2), minus KB_DAMP * vel_b.
#                           In a static press tau_env = -iq_a, so at K_F = 1 it
#                           matches the old path's static force; in free
#                           motion it is ~0 instead of the follower's own
#                           inertia + friction current.
# FELT_DEADBAND_A: soft dead band on tau_env before it is felt (|x| <= DB ->
# 0, else sign(x)*(|x| - DB)), to keep stiction-held current (up to ~20 mA
# at rest in C2) from being felt as contact. 0 = off.
# Overrides: TELEOP_FELT_SOURCE=tau_env TELEOP_K_F=0.25 TELEOP_FELT_DEADBAND_A=0.02
FELT_SOURCE = os.environ.get("TELEOP_FELT_SOURCE", "iq_a_filt")
FELT_K_F = float(os.environ.get("TELEOP_K_F", "0.25"))
FELT_DEADBAND_A = float(os.environ.get("TELEOP_FELT_DEADBAND_A", "0"))

# Sustained-hold fix (TEST_LOG 2026-10-26): contact-latched spring added to
# the follower command, see contact_spring.py. HOLD_K in A/count of
# leader-past-contact-point; 0 = off (the C3 behaviour). HOLD_MAX_A bounds
# the term (and so how fast a held force can wind up). Needs the tau_env
# felt path. Overrides: TELEOP_HOLD_K=0.5e-3 TELEOP_HOLD_MAX_A=0.1
HOLD_K = float(os.environ.get("TELEOP_HOLD_K", "0"))
HOLD_MAX_A = float(os.environ.get("TELEOP_HOLD_MAX_A", "0.1"))
HOLD_RAMP_S = float(os.environ.get("TELEOP_HOLD_RAMP_S", "0.3"))   # 0 = full strength at engage (HOLD runs 1-2)

# Leader contact spring (option A, TEST_LOG 2026-10-29): on the hold's latch,
# the leader gets -K * (leader past the contact point), clamped to MAX_A,
# plus CONTACT_DAMP * vel_b while latched. Without it the leader travels
# ~35-40 deg while the felt force builds, whatever the object, so rigid and
# sponge feel alike. leader_spring_sim.py: no wind-up (the hand observer
# sees the true hand torque). Needs HOLD_K. 0 = off.
# Overrides: TELEOP_LEADER_K=2e-3 TELEOP_LEADER_K_MAX_A=0.4 TELEOP_LEADER_CONTACT_DAMP=1e-5
LEADER_K = float(os.environ.get("TELEOP_LEADER_K", "0"))
LEADER_K_MAX_A = float(os.environ.get("TELEOP_LEADER_K_MAX_A", "0.4"))
LEADER_CONTACT_DAMP = float(os.environ.get("TELEOP_LEADER_CONTACT_DAMP", "0"))

# Weak slipping position coupling on the follower (TEST_LOG 2026-10-29):
# Channel 1 only matches velocities, so with the leader still the follower
# sags under gravity in stick-slip steps (~15 deg in 1-2 s). Adds
# SIGN * POS_K * (err - pos_ref) to the follower command, where pos_ref
# slides so that |err - pos_ref| <= POS_WINDOW: small drifts are pulled
# back, but the big leader-ahead gaps left by pushing through the current
# cap are absorbed instead of yanking the follower (pull <= POS_K *
# POS_WINDOW). 0 = off. Overrides: TELEOP_POS_K=1e-3 TELEOP_POS_WINDOW=50
POS_K = float(os.environ.get("TELEOP_POS_K", "0"))
POS_WINDOW = float(os.environ.get("TELEOP_POS_WINDOW", "50"))

# Leader ripple + friction compensation (TEST_LOG 2026-10-26, leader_comp.py):
# model file from friction_map.py sweeps; "" = off. Feed-forward d(th, v) on
# iq_b_cmd, and (LEADER_COMP_OBS) i_b - d into the hand-torque observer.
# Overrides: TELEOP_LEADER_COMP=leader_comp_B.json TELEOP_LEADER_COMP_RIPPLE=1.0
#            TELEOP_LEADER_COMP_FRIC=0.5 TELEOP_LEADER_COMP_OBS=1
LEADER_COMP_PATH = os.environ.get("TELEOP_LEADER_COMP", "")
LEADER_COMP_RIPPLE = float(os.environ.get("TELEOP_LEADER_COMP_RIPPLE", "1.0"))
LEADER_COMP_FRIC = float(os.environ.get("TELEOP_LEADER_COMP_FRIC", "0.5"))
LEADER_COMP_OBS = os.environ.get("TELEOP_LEADER_COMP_OBS", "1") == "1"
# In-session leader calibration (leader_cal.py): after START, a hands-off
# ~30 s leader sweep (follower at 0 A) is fitted and used as the leader comp
# for this session -- the 7/rev amplitude changes every START and the 12/rev
# phase every power-up, so a fixed file cannot follow them. Supersedes
# TELEOP_LEADER_COMP. Override: TELEOP_LEADER_CAL=1
LEADER_CAL = os.environ.get("TELEOP_LEADER_CAL", "0") == "1"
# Warm-up before the leader cal: hold both at 0 A in RUN this many seconds.
# The leader's 7/rev grows in RUN (tau ~160 s, ~60-65 mA warm; TEST_LOG
# 2026-10-28) and a START-time cal misses it; 300 s reaches ~85 %, 480 s
# ~95 %. 0 = no warm-up. Override: TELEOP_LEADER_WARMUP_S=300
LEADER_WARMUP_S = float(os.environ.get("TELEOP_LEADER_WARMUP_S", "0"))


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
    final_log_path = log_path
    if LIVE_LOG_DIR:
        os.makedirs(LIVE_LOG_DIR, exist_ok=True)
        log_path = os.path.join(LIVE_LOG_DIR, os.path.basename(final_log_path))
    if FELT_SOURCE not in ("iq_a_filt", "tau_env"):
        raise SystemExit(f"TELEOP_FELT_SOURCE={FELT_SOURCE!r}: use iq_a_filt or tau_env")
    if FELT_SOURCE == "tau_env" and not (SHADOW_TAU_ENV and FOLLOWER_MODEL_PATH):
        raise SystemExit("TELEOP_FELT_SOURCE=tau_env needs TELEOP_SHADOW_TAU_ENV=1 and "
                         "TELEOP_FOLLOWER_MODEL (the C2-validated estimator)")
    if not 0.0 <= FELT_K_F <= 1.0:
        raise SystemExit("TELEOP_K_F must be in [0, 1] (C3 ramps 0.25 -> 0.5 -> 1.0)")
    if HOLD_K and FELT_SOURCE != "tau_env":
        raise SystemExit("TELEOP_HOLD_K needs TELEOP_FELT_SOURCE=tau_env (contact is detected from tau_env)")
    if not (0.0 <= HOLD_K <= 2e-3 and 0.0 <= HOLD_MAX_A <= 0.3):
        raise SystemExit("TELEOP_HOLD_K must be in [0, 2e-3] A/count and TELEOP_HOLD_MAX_A in [0, 0.3] A")
    if (LEADER_K or LEADER_CONTACT_DAMP) and not HOLD_K:
        raise SystemExit("TELEOP_LEADER_K / TELEOP_LEADER_CONTACT_DAMP need TELEOP_HOLD_K (they use its contact latch)")
    if not (0.0 <= LEADER_K <= 2e-2 and 0.0 <= LEADER_K_MAX_A <= 0.8 and 0.0 <= LEADER_CONTACT_DAMP <= 5e-5):
        raise SystemExit("TELEOP_LEADER_K must be in [0, 2e-2] A/count, TELEOP_LEADER_K_MAX_A in [0, 0.8] A, "
                         "TELEOP_LEADER_CONTACT_DAMP in [0, 5e-5] A/(count/s)")
    hold = ContactSpring(HOLD_K, HOLD_MAX_A, sign_=SIGN, ramp_s=HOLD_RAMP_S) if HOLD_K else None
    leader_spring = bool(hold and (LEADER_K or LEADER_CONTACT_DAMP))
    if not (0.0 <= POS_K <= 3e-3 and 0.0 < POS_WINDOW <= 200):
        raise SystemExit("TELEOP_POS_K must be in [0, 3e-3] A/count and TELEOP_POS_WINDOW in (0, 200] counts")
    if not (0.0 <= LEADER_COMP_RIPPLE <= 1.5 and 0.0 <= LEADER_COMP_FRIC <= 1.0):
        raise SystemExit("TELEOP_LEADER_COMP_RIPPLE must be in [0, 1.5], TELEOP_LEADER_COMP_FRIC in [0, 1]")
    lcomp = (LeaderComp.load(LEADER_COMP_PATH, gain_ripple=LEADER_COMP_RIPPLE, gain_fric=LEADER_COMP_FRIC)
             if LEADER_COMP_PATH else None)
    iq_b_comp = 0.0
    iq_b_spring = 0.0
    iq_a_pos = 0.0
    pos_ref = None
    if LEADER_CAL and not USE_FRAME_QUEUE:
        raise SystemExit("TELEOP_LEADER_CAL=1 needs TELEOP_FRAME_QUEUE=1")
    if LEADER_WARMUP_S and not LEADER_CAL:
        raise SystemExit("TELEOP_LEADER_WARMUP_S only applies with TELEOP_LEADER_CAL=1")

    # pair_dbg=False releases each frame on arrival; pairing with the 10 Hz
    # direct-mode DBG would hold every record ~1 ms for nothing.
    a = MotorCANInterface(channel="can0", node_base=NODE_BASE_A,
                          pair_dbg=not USE_FRAME_QUEUE)
    b = MotorCANInterface(channel="can0", node_base=NODE_BASE_B,
                          pair_dbg=not USE_FRAME_QUEUE)
    a.start_listening()
    b.start_listening()

    if VEL_ESTIMATOR not in ("legacy", "ab55"):
        raise SystemExit(f"VEL_ESTIMATOR={VEL_ESTIMATOR!r}: use 'legacy' or 'ab55'")
    if VEL_ESTIMATOR != "legacy" and not USE_FRAME_QUEUE:
        raise SystemExit("VEL_ESTIMATOR=ab55 needs USE_FRAME_QUEUE (TELEOP_FRAME_QUEUE=1)")
    vel_est = ({"A": AlphaBetaVelocity(VEL_AB_BANDWIDTH_HZ),
                "B": AlphaBetaVelocity(VEL_AB_BANDWIDTH_HZ)}
               if VEL_ESTIMATOR == "ab55" else None)
    vel_out = {"A": 0.0, "B": 0.0}
    if (OBSERVER_IMPL not in ("inline", "class") or OBS_G_SOURCE not in ("old", "A2")
            or OBS_DT_SOURCE not in ("nominal", "mcu")):
        raise SystemExit(f"bad observer config: {OBSERVER_IMPL}/{OBS_G_SOURCE}/{OBS_DT_SOURCE}")
    if OBSERVER_IMPL == "inline" and (OBS_G_SOURCE, OBS_DT_SOURCE) != ("old", "nominal"):
        raise SystemExit("OBS_G_SOURCE / OBS_DT_SOURCE only apply to TELEOP_OBSERVER=class")
    hand_obs = (HandTorqueObserver(G_B_CNT_S2_PER_A if OBS_G_SOURCE == "old"
                                   else G_B_A2_CNT_S2_PER_A, OBS_CUTOFF_HZ)
                if OBSERVER_IMPL == "class" else None)
    prev_obs_mcu_b = None
    if SHADOW_TAU_ENV and not USE_FRAME_QUEUE:
        raise SystemExit("TELEOP_SHADOW_TAU_ENV=1 needs TELEOP_FRAME_QUEUE=1")
    follower_model = None
    if FOLLOWER_MODEL_PATH:
        if not SHADOW_TAU_ENV:
            raise SystemExit("TELEOP_FOLLOWER_MODEL only applies with TELEOP_SHADOW_TAU_ENV=1")
        with open(FOLLOWER_MODEL_PATH) as f:
            follower_model = json.load(f)
    shadow_g = follower_model["G_cnt_s2_per_A"] if follower_model else G_A_A2_CNT_S2_PER_A
    shadow_obs = ExternalTorqueObserver(shadow_g, OBS_CUTOFF_HZ) if SHADOW_TAU_ENV else None
    shadow_vel = AlphaBetaVelocity(VEL_AB_BANDWIDTH_HZ) if SHADOW_TAU_ENV else None
    shadow_iq = collections.deque(maxlen=SHADOW_ALIGN_FRAMES + 1)
    shadow_prev_us = [None]
    tau_env = 0.0

    frames_file = frames_writer = None
    frames_logged = {"A": 0, "B": 0}
    if USE_FRAME_QUEUE:
        frames_path = log_path[:-4] + "_frames.csv"
        frames_file = open(frames_path, "w", newline="")
        frames_writer = csv.writer(frames_file)
        frames_writer.writerow(["loop_iter", "motor", "bus_time", "mcu_time_us",
                                "enc", "iq_meas"])

    def shadow_update(r):
        """C2: one follower frame into the shadow estimator (log only)."""
        nonlocal tau_env
        v = shadow_vel.update(r["mcu_time_us"], r["enc"])
        shadow_iq.append(r["iq_meas"])
        dt = ((r["mcu_time_us"] - shadow_prev_us[0]) * 1e-6
              if shadow_prev_us[0] is not None else 1e-3)
        shadow_prev_us[0] = r["mcu_time_us"]
        motor_side = shadow_obs.update(v, shadow_iq[0], dt)  # oldest = delayed
        if follower_model is not None:
            motor_side -= model_terms(follower_model["coeffs_A"], v, r["enc"])
        tau_env = -motor_side

    def drain_frames(loop_iter):
        """Queue path: write every frame since the last call, both motors."""
        for name, iface in (("A", a), ("B", b)):
            for r in iface.drain_records():
                if vel_est is not None:
                    vel_out[name] = vel_est[name].update(r["mcu_time_us"], r["enc"])
                if shadow_obs is not None and name == "A":
                    shadow_update(r)
                frames_writer.writerow([loop_iter, name, f"{r['bus_time']:.6f}",
                                        f"{r['mcu_time_us']:.0f}", f"{r['enc']:.0f}",
                                        r["iq_meas"]])
                frames_logged[name] += 1

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
        + (["tau_env", "tau_env_N"] if SHADOW_TAU_ENV else [])
        + (["iq_a_hold", "hold_e0"] if hold else [])
        + (["iq_b_comp"] if (lcomp or LEADER_CAL) else [])
        + (["iq_b_spring"] if leader_spring else [])
        + (["iq_a_pos"] if POS_K else [])
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

        if LEADER_CAL and LEADER_WARMUP_S > 0:
            print(f"Warm-up: {LEADER_WARMUP_S:.0f} s in RUN at 0 A (hands off, follower limp)...")
            t_w = time.monotonic()
            next_note = t_w + 60.0
            while time.monotonic() - t_w < LEADER_WARMUP_S:
                a.send_set_iq(0.0)
                b.send_set_iq(0.0)
                a.drain_records()
                b.drain_records()
                if time.monotonic() >= next_note:
                    print(f"  warm-up {time.monotonic() - t_w:.0f} / {LEADER_WARMUP_S:.0f} s")
                    next_note += 60.0
                time.sleep(0.005)
        if LEADER_CAL:
            print("Leader calibration: HANDS OFF the leader (~30 s sweep, 1 rev each way)...")
            cal_rows = leader_cal.sweep(b, a, log_path=log_path[:-4] + "_cal.csv")
            if cal_rows is None:
                return
            cal_model = leader_cal.fit(cal_rows)
            with open(log_path[:-4] + "_cal.json", "w") as f:
                json.dump(cal_model, f, indent=1)
            print("Leader calibration: " + leader_cal.summary(cal_model))
            lcomp = LeaderComp(cal_model, gain_ripple=LEADER_COMP_RIPPLE, gain_fric=LEADER_COMP_FRIC)
            print("Leader comp from this calibration is ON. Take the leader.\n")
            time.sleep(1.0)

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
        print(f"Logging to {log_path}  (frame queue {'on' if USE_FRAME_QUEUE else 'off'}, "
              f"velocity estimator {VEL_ESTIMATOR}, observer {OBSERVER_IMPL}"
              + (f"/G {OBS_G_SOURCE}/dt {OBS_DT_SOURCE}" if hand_obs else "")
              + (", shadow tau_env on" if SHADOW_TAU_ENV else "")
              + (f" with model {FOLLOWER_MODEL_PATH} (G {shadow_g:,.0f})" if follower_model else "") + ")")
        print(f"Felt force: {FELT_SOURCE}" + (f", K_F {FELT_K_F:g}, dead band {FELT_DEADBAND_A * 1e3:g} mA"
                                              if FELT_SOURCE == "tau_env" else "")
              + (f"; contact spring K {HOLD_K:g} A/count, max {HOLD_MAX_A:g} A, ramp {HOLD_RAMP_S:g} s" if hold else "")
              + (f"; leader contact spring K {LEADER_K:g} A/count, max {LEADER_K_MAX_A:g} A, "
                 f"damp {LEADER_CONTACT_DAMP:g} A/(count/s)" if leader_spring else "")
              + (f"; follower position coupling K {POS_K:g} A/count, window {POS_WINDOW:g} counts" if POS_K else "")
              + (f"; leader comp {'in-session cal' if LEADER_CAL else LEADER_COMP_PATH} (ripple x{LEADER_COMP_RIPPLE:g}, friction x{LEADER_COMP_FRIC:g}, "
                 f"observer {'on' if LEADER_COMP_OBS else 'off'})" if lcomp else "")
              + (f"; logging live to {LIVE_LOG_DIR}, moved to {LOG_DIR}/ at exit" if LIVE_LOG_DIR else ""))
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

        if USE_FRAME_QUEUE:
            drain_frames(-1)  # START transient and pre-loop frames
        loop_iter = 0
        while True:
            ta = a.get_telemetry()
            tb = b.get_telemetry()
            if USE_FRAME_QUEUE:
                drain_frames(loop_iter)
            loop_iter += 1
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
            if vel_est is not None:
                # B2: replaces the legacy estimate everywhere downstream.
                vel_a_filt, vel_b_filt = vel_out["A"], vel_out["B"]

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
            if lcomp is not None:
                iq_b_comp = lcomp.d(pos_b, vel_b_filt)
                if LEADER_COMP_OBS and not iq_b_stale:
                    # the observer then sees hand torque, not ripple/friction
                    iq_b_meas = iq_b_meas - iq_b_comp
            if not iq_b_stale and hand_obs is not None:
                # B3 class path. dt from the ESC clock or nominal, per config.
                if OBS_DT_SOURCE == "nominal":
                    tau_h_hat = hand_obs.update(vel_b_filt, iq_b_meas, dt)
                else:
                    mcu_b = tb["mcu_time_us"]
                    if prev_obs_mcu_b is not None and mcu_b is not None and mcu_b > prev_obs_mcu_b:
                        tau_h_hat = hand_obs.update(vel_b_filt, iq_b_meas,
                                                    (mcu_b - prev_obs_mcu_b) * 1e-6)
                    prev_obs_mcu_b = mcu_b
            elif not iq_b_stale:
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
            iq_a_hold = hold.update(now, err, tau_env, vel_a_filt) if hold else 0.0
            if POS_K:
                if pos_ref is None:
                    pos_ref = err
                pos_ref = min(max(pos_ref, err - POS_WINDOW), err + POS_WINDOW)
                iq_a_pos = SIGN * POS_K * (err - pos_ref)
            iq_a_cmd_raw = clamp(iq_a_p + iq_a_d + iq_a_ff + iq_a_hold + iq_a_pos, IQ1_MAX_A)
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
                iq_b_spring = 0.0
                iq_b_cmd = clamp(-KB_DAMP * vel_b_filt, IQ2_MAX_A)
            else:
                iq_a_filt = IQ_FILTER_ALPHA * iq_a + (1 - IQ_FILTER_ALPHA) * iq_a_filt
                if FELT_SOURCE == "tau_env":
                    x = tau_env
                    if FELT_DEADBAND_A > 0:
                        x = math.copysign(max(0.0, abs(x) - FELT_DEADBAND_A), x)
                    if leader_spring and hold.active:
                        iq_b_spring = (hold.leader(LEADER_K, LEADER_K_MAX_A)
                                       - LEADER_CONTACT_DAMP * vel_b_filt)
                    else:
                        iq_b_spring = 0.0
                    iq_b_cmd = clamp(FELT_K_F * x - KB_DAMP * vel_b_filt + iq_b_comp + iq_b_spring,
                                     IQ2_MAX_A)
                else:
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
                + ([f"{tau_env:.4f}",
                    f"{tau_env * SHADOW_N_PER_A:.4f}"]
                   if SHADOW_TAU_ENV else [])
                + ([f"{iq_a_hold:.4f}", f"{hold.e0:.0f}" if hold.active else ""] if hold else [])
                + ([f"{iq_b_comp:.4f}"] if lcomp else [])
                + ([f"{iq_b_spring:.4f}"] if leader_spring else [])
                + ([f"{iq_a_pos:.4f}"] if POS_K else [])
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
        if frames_file is not None:
            try:
                drain_frames(-2)  # whatever arrived after the last iteration
            finally:
                frames_file.close()
            print(f"Frames logged (every TELEM): A={frames_logged['A']} "
                  f"B={frames_logged['B']} -> {frames_path}")
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
            # keep a leader-cal log (e.g. an aborted cal) for diagnosis
            for src in (log_path[:-4] + "_cal.csv", log_path[:-4] + "_cal.json"):
                if LIVE_LOG_DIR and os.path.exists(src):
                    print(f"Kept {shutil.move(src, os.path.join(LOG_DIR, os.path.basename(src)))}")
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
            png_path = plot_teleop_log(log_path)
            if LIVE_LOG_DIR:
                # Off the live path now: move RAM logs to their final place.
                moved = [shutil.move(src, os.path.join(LOG_DIR, os.path.basename(src)))
                         for src in (log_path, log_path[:-4] + "_frames.csv", png_path,
                                     log_path[:-4] + "_cal.csv", log_path[:-4] + "_cal.json")
                         if src and os.path.exists(src)]
                log_path = final_log_path
                png_path = next((m for m in moved if m.endswith(".png")), None)
                print(f"Moved {len(moved)} file(s) from {LIVE_LOG_DIR} to {LOG_DIR}/")
            print(f"Log saved: {log_path}")
            if png_path:
                print(f"Plot saved: {png_path}")


if __name__ == "__main__":
    sys.exit(main())
