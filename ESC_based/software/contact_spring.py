"""
Contact-latched spring on the follower command (fix for sustained holds,
TEST_LOG 2026-10-26 "Reference run").

Why: in a static press the felt force circulates follower -> tau_env ->
leader -> hand-torque estimate -> follower with loop gain K_F * FF_GAIN = 1,
and nothing ties its level to how far the leader is pushed in. The dead
band and leader friction drain a fixed amount per pass, so a held press
decays linearly to zero in 1-1.5 s.

What: at contact onset (|tau_env| > engage) latch e0 = err (leader minus
follower, counts) and the push direction s = -sign(tau_env) (the direction
the follower pushes into the object). While latched,

    i_hold = s * clamp(s * sign_ * k * (err - e0), 0, i_max)

i.e. only toward the object, only once the leader is past where contact
began. Because the circulating loop integrates any excess over the leak,
i_hold does not set the held force by itself: the force settles where the
spring just refills the leak (err - e0 ~ leak / k, a few tens of counts),
which pins the leader at the contact point like a wall -- the hand then
sets the force level. i_max bounds how fast the force can wind up if the
hand is held rigidly past the contact point.

Ramp-in (ramp_s): the spring's output is scaled by (t - t_engage)/ramp_s,
up to 1. Live at full strength from the first sample it doubled the
leader's ringing in the first 0.3 s of contact (median 4 velocity sign
flips vs 0-2 without; user: "a little kick and some oscillations at first
contact"). The fade it fixes takes ~1 s to develop, so ramping over
0.3 s keeps it out of the impact transient. ramp_s = 0 = no ramp.

Release when |tau_env| stays below `release` for `release_s` seconds
(timestamps, not loop counts), or the follower moves faster than `v_max`
counts/s (not blocked: a false engage in free motion or a bounce). A
contact in the opposite direction re-latches.

Per-instance state; no globals, so one instance per follower.
"""

import math


class ContactSpring:
    def __init__(self, k, i_max=0.1, engage=0.08, release=0.04, release_s=0.05,
                 v_max=3000.0, sign_=1, ramp_s=0.3):
        self.k, self.i_max, self.ramp_s = k, i_max, ramp_s
        self.engage, self.release, self.release_s = engage, release, release_s
        self.v_max, self.sign_ = v_max, sign_
        self.reset()

    def reset(self):
        self.active = False
        self.e0 = 0.0
        self.t_on = 0.0
        self.s = 0
        self.below_since = None
        self.i = 0.0

    def update(self, t, err, tau_env, vel_a):
        """t in seconds (any monotonic clock). Returns the hold current (A)."""
        a = abs(tau_env)
        push = -1 if tau_env > 0 else 1
        if a > self.engage and (not self.active or push != self.s):
            self.active, self.e0, self.s, self.below_since, self.t_on = True, err, push, None, t
        elif self.active:
            if abs(vel_a) > self.v_max:
                self.reset()
            elif a < self.release:
                if self.below_since is None:
                    self.below_since = t
                elif t - self.below_since >= self.release_s:
                    self.reset()
            else:
                self.below_since = None
        if not self.active:
            self.i = 0.0
            return 0.0
        x = self.s * self.sign_ * self.k * (err - self.e0)
        ramp = min(1.0, (t - self.t_on) / self.ramp_s) if self.ramp_s > 0 else 1.0
        self.i = self.s * ramp * min(max(x, 0.0), self.i_max)
        return self.i


if __name__ == "__main__":
    # Offline replay on a teleop log: when would it engage, and how much
    # would it add (open loop -- the log's own feedback is not re-simulated).
    import argparse
    import csv

    import numpy as np

    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--k", type=float, default=0.5e-3)
    ap.add_argument("--imax", type=float, default=0.1)
    a = ap.parse_args()
    rows = list(csv.DictReader(open(a.csv)))
    col = lambda n: np.array([float(r[n]) for r in rows])
    t, err, tau, va = col("t_s"), col("err"), col("tau_env"), col("vel_a")
    cs = ContactSpring(a.k, a.imax)
    out = np.array([cs.update(*z) for z in zip(t, err, tau, va)])
    on = out != 0
    edges = np.diff(np.r_[0, (np.array([1 if x else 0 for x in on])), 0])
    st, en = np.where(edges == 1)[0], np.where(edges == -1)[0]
    contact = np.abs(tau) > 0.08
    print(f"{len(st)} nonzero-hold intervals; hold nonzero {100 * on.mean():.1f} % of the run, "
          f"in contact {100 * contact.mean():.1f} %")
    print(f"hold nonzero while |tau_env| < 40 mA: {100 * np.mean(on & (np.abs(tau) < 0.04)):.2f} % of the run")
    # sign check: hold must have the follower command's sign during contact
    agree = np.sign(out[on]) == np.sign(col("iq_a_cmd")[on])
    print(f"hold sign agrees with the follower push: {100 * agree.mean():.1f} %")
    for s, e in zip(st, en):
        if t[e - 1] - t[s] > 0.3:
            print(f"  {t[s]:6.2f}-{t[e - 1]:6.2f} s  peak {np.abs(out[s:e]).max() * 1e3:4.0f} mA, "
                  f"at the clamp {100 * np.mean(np.abs(out[s:e]) >= a.imax - 1e-9):3.0f} % of the interval")
