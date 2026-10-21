"""
Per-instance state estimators for the ESC_based host code (plan B2/B3).

Every estimator here keeps its own state (one instance per motor) and takes
dt from the timestamps it is given -- the ESC's mcu_time_us for TELEM-driven
estimators -- never from a nominal loop period. That is what makes them
portable: they behave the same at 500 Hz or 1 kHz, over CAN or UART.

Velocity (B2), all in counts/s, fed every TELEM frame via update(t_us, pos):
  LsVelocity         sliding least-squares slope over the last `window_s`
  QuadLsVelocity     least-squares quadratic, slope at the newest sample
  AlphaBetaVelocity  alpha-beta tracker (position/velocity) with fixed gains
  LegacyVelocity     replica of teleop_test.py's original scheme, for
                     offline comparison only: latest frame per loop tick,
                     delta-pos over >= 2 ms of KERNEL time, EMA alpha 0.7

Both new estimators have a rest detector: when the position range over the
last `rest_window_s` is <= `rest_counts` (the AS5047P/ABI dithers +-1 count
at rest, ~0.5 % of frames in A3's 10 min log), the output is exactly 0.
Side effect: true speeds below ~rest_counts/rest_window_s (50 counts/s at
the defaults) also read 0 -- that is the encoder's resolution floor over
that window anyway.
"""

import collections
import math


class LsVelocity:
    """Sliding least-squares slope of position vs time over `window_s`."""

    def __init__(self, window_s=0.006, rest_window_s=0.02, rest_counts=1):
        self.window_s = window_s
        self.rest_window_s = rest_window_s
        self.rest_counts = rest_counts
        self.buf = collections.deque()
        self.v = 0.0

    def update(self, t_us, pos):
        t = t_us * 1e-6
        self.buf.append((t, pos))
        horizon = max(self.window_s, self.rest_window_s)
        while self.buf and t - self.buf[0][0] > horizon + 1e-9:
            self.buf.popleft()
        if _at_rest(self.buf, t, self.rest_window_s, self.rest_counts):
            self.v = 0.0
            return self.v
        pts = [(tt, p) for tt, p in self.buf if t - tt <= self.window_s + 1e-9]
        if len(pts) < 3:
            return self.v
        mt = sum(p[0] for p in pts) / len(pts)
        mx = sum(p[1] for p in pts) / len(pts)
        den = sum((p[0] - mt) ** 2 for p in pts)
        if den > 0:
            self.v = sum((p[0] - mt) * (p[1] - mx) for p in pts) / den
        return self.v

    # Group delay of a centred LS slope is half the window.
    @property
    def nominal_lag_s(self):
        return self.window_s / 2


class QuadLsVelocity(LsVelocity):
    """
    Least-squares QUADRATIC fit of position over `window_s`, slope taken at
    the newest sample: p(t) = a + v*(t - t_now) + c*(t - t_now)^2. Unlike the
    linear slope (which reads the speed at the window's centre, a lag of
    window/2), this reads it at the end -- near-zero lag for constant
    acceleration -- at the cost of more noise for the same window.
    """

    def update(self, t_us, pos):
        t = t_us * 1e-6
        self.buf.append((t, pos))
        horizon = max(self.window_s, self.rest_window_s)
        while self.buf and t - self.buf[0][0] > horizon + 1e-9:
            self.buf.popleft()
        if _at_rest(self.buf, t, self.rest_window_s, self.rest_counts):
            self.v = 0.0
            return self.v
        pts = [(tt - t, p) for tt, p in self.buf if t - tt <= self.window_s + 1e-9]
        if len(pts) < 4:
            return self.v
        # normal equations for [1, x, x^2]
        s0 = len(pts)
        s1 = sum(x for x, _ in pts); s2 = sum(x * x for x, _ in pts)
        s3 = sum(x ** 3 for x, _ in pts); s4 = sum(x ** 4 for x, _ in pts)
        y0 = sum(p for _, p in pts); y1 = sum(x * p for x, p in pts)
        y2 = sum(x * x * p for x, p in pts)
        m = ((s0, s1, s2), (s1, s2, s3), (s2, s3, s4))
        det = _det3(m)
        if det != 0:
            mv = ((s0, y0, s2), (s1, y1, s3), (s2, y2, s4))   # Cramer, column 1
            self.v = _det3(mv) / det
        return self.v


def _det3(m):
    return (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))


class AlphaBetaVelocity:
    """
    Alpha-beta tracker. Gains are set from a bandwidth-like parameter via
    the critically damped choice beta = alpha^2 / (2 - alpha), with alpha
    derived per sample from dt so the response does not depend on the rate.
    """

    def __init__(self, bandwidth_hz=60.0, rest_window_s=0.02, rest_counts=1):
        self.bw = bandwidth_hz
        self.rest_window_s = rest_window_s
        self.rest_counts = rest_counts
        self.x = None
        self.v = 0.0
        self.t = None
        self.buf = collections.deque()

    def update(self, t_us, pos):
        t = t_us * 1e-6
        self.buf.append((t, pos))
        while self.buf and t - self.buf[0][0] > self.rest_window_s + 1e-9:
            self.buf.popleft()
        if self.x is None:
            self.x, self.t = float(pos), t
            return 0.0
        dt = t - self.t
        self.t = t
        if dt <= 0:
            return self.v
        # alpha from the tracker's natural frequency: w*dt small -> alpha ~ 2*w*dt
        w = 2 * math.pi * self.bw
        alpha = min(1.0, 1.0 - math.exp(-2.0 * w * dt))
        beta = alpha * alpha / (2.0 - alpha)
        xp = self.x + self.v * dt
        r = pos - xp
        self.x = xp + alpha * r
        self.v = self.v + beta * r / dt
        if _at_rest(self.buf, t, self.rest_window_s, self.rest_counts):
            return 0.0
        return self.v


class LegacyVelocity:
    """
    teleop_test.py's original velocity, for offline comparison: called once
    per loop tick with the LATEST frame (pos, kernel bus_time); a new raw
    estimate only when >= min_dt_s of bus time has passed, then EMA.
    """

    def __init__(self, min_dt_s=0.002, ema_alpha=0.7):
        self.min_dt = min_dt_s
        self.a = ema_alpha
        self.prev = None
        self.raw = 0.0
        self.filt = 0.0

    def update_tick(self, bus_time_s, pos):
        if self.prev is None:
            self.prev = (bus_time_s, pos)
        elif bus_time_s - self.prev[0] >= self.min_dt:
            self.raw = (pos - self.prev[1]) / (bus_time_s - self.prev[0])
            self.prev = (bus_time_s, pos)
        self.filt = self.a * self.raw + (1 - self.a) * self.filt
        return self.filt


def _at_rest(buf, t_now, window_s, counts):
    if not buf or t_now - buf[0][0] < window_s * 0.9:
        return False
    lo = hi = None
    for tt, p in buf:
        if t_now - tt <= window_s + 1e-9:
            lo = p if lo is None or p < lo else lo
            hi = p if hi is None or p > hi else hi
    return lo is not None and (hi - lo) <= counts


class HandTorqueObserver:
    """
    Leader (Motor B) hand-torque observer, plan B3 -- the class form of the
    inline observer in teleop_test.py (lines ~632-640 there):

        x     = OBS_COMP * v / G                 (v in counts/s -> A-equivalent)
        tau_h = w * (x - LPF_w(x)) - LPF_w(i)    (derivative-free form)

    in amps of equivalent motor current, positive toward +counts. Unlike the
    inline version, the LPF coefficient AND the discretisation correction
    OBS_COMP = 1 / (1 - pi * fc * dt) are computed from the dt passed with
    each sample, not from a nominal loop rate. Pass the nominal period to
    reproduce the original exactly; pass the measured one (ideally from
    mcu_time_us) to be rate-independent.
    """

    def __init__(self, g_cnt_s2_per_a, fc_hz=20.0):
        self.G = g_cnt_s2_per_a
        self.fc = fc_hz
        self.w = 2.0 * math.pi * fc_hz
        self.x_lpf = None
        self.i_lpf = 0.0
        self.tau = 0.0

    def update(self, v, i, dt):
        if dt <= 0:
            return self.tau
        alpha = 1.0 - math.exp(-self.w * dt)
        comp = 1.0 / (1.0 - math.pi * self.fc * dt)
        x = comp * v / self.G
        if self.x_lpf is None:
            self.x_lpf = x       # start at the real value, no invented transient
        self.x_lpf += alpha * (x - self.x_lpf)
        self.i_lpf += alpha * (i - self.i_lpf)
        self.tau = self.w * (x - self.x_lpf) - self.i_lpf
        return self.tau


class RippleModel:
    """
    Angle- and current-dependent ripple, plan B6 (A4 model):

        r(theta, i) = sum_n c_n(theta) + i * sum_m g_m(theta)
        c_n(theta)  = a_n cos(n theta) + b_n sin(n theta), n in c_harmonics
        g_m(theta)  = p_m cos(m theta) + q_m sin(m theta), m in g_harmonics

    theta from the raw encoder count (theta = 2 pi * (count mod 4000)/4000),
    r in amps of equivalent current (the current that does not become
    acceleration). Coefficients come from fit() on logged frames, never
    hand-copied; save()/load() keep them as JSON next to the logs.
    """

    ENC = 4000

    def __init__(self, c_harmonics=(12, 14, 24, 28, 36, 42, 84),
                 g_harmonics=(14, 42, 84), coeffs=None):
        self.ch = tuple(c_harmonics)
        self.gh = tuple(g_harmonics)
        self.coeffs = coeffs  # flat list: [a,b per c harmonic] + [p,q per g harmonic]

    def columns(self, count, i):
        import numpy as np
        th = 2 * np.pi * (np.asarray(count) % self.ENC) / self.ENC
        i = np.asarray(i, dtype=float)
        cols = []
        for n in self.ch:
            cols += [np.cos(n * th), np.sin(n * th)]
        for m in self.gh:
            cols += [i * np.cos(m * th), i * np.sin(m * th)]
        return np.vstack(cols).T

    def eval(self, count, i):
        """r for one sample (floats) -- cheap enough for the live loop."""
        th = 2 * math.pi * (count % self.ENC) / self.ENC
        k = 0
        r = 0.0
        for n in self.ch:
            r += self.coeffs[k] * math.cos(n * th) + self.coeffs[k + 1] * math.sin(n * th)
            k += 2
        for m in self.gh:
            r += i * (self.coeffs[k] * math.cos(m * th) + self.coeffs[k + 1] * math.sin(m * th))
            k += 2
        return r

    def amplitudes(self):
        out, k = {}, 0
        for n in self.ch:
            out[f"c{n}"] = math.hypot(self.coeffs[k], self.coeffs[k + 1]); k += 2
        for m in self.gh:
            out[f"g{m}"] = math.hypot(self.coeffs[k], self.coeffs[k + 1]); k += 2
        return out

    def save(self, path, meta=None):
        import json
        with open(path, "w") as f:
            json.dump({"c_harmonics": self.ch, "g_harmonics": self.gh,
                       "coeffs": list(self.coeffs), "meta": meta or {}}, f, indent=1)

    @classmethod
    def load(cls, path):
        import json
        d = json.load(open(path))
        return cls(d["c_harmonics"], d["g_harmonics"], d["coeffs"])


class ExternalTorqueObserver:
    """
    Follower external-torque observer (C1 form, used by B6 for testing):

        tau = LPF_w(i - r(theta, i)) - w * (x - LPF_w(x)),  x = OBS_COMP * v / G

    i.e. the measured current that did NOT become acceleration, low-passed,
    in amps. Sign as written in the brief (i - a/G); C1 must still verify
    it against a push of known direction. ripple=None disables B6.
    """

    def __init__(self, g_cnt_s2_per_a, fc_hz=20.0, ripple=None):
        self.G = g_cnt_s2_per_a
        self.fc = fc_hz
        self.w = 2.0 * math.pi * fc_hz
        self.ripple = ripple
        self.x_lpf = None
        self.i_lpf = 0.0
        self.tau = 0.0

    def update(self, v, i, dt, count=None):
        if dt <= 0:
            return self.tau
        alpha = 1.0 - math.exp(-self.w * dt)
        comp = 1.0 / (1.0 - math.pi * self.fc * dt)
        x = comp * v / self.G
        if self.x_lpf is None:
            self.x_lpf = x
        self.x_lpf += alpha * (x - self.x_lpf)
        i_eff = i - (self.ripple.eval(count, i) if self.ripple is not None else 0.0)
        self.i_lpf += alpha * (i_eff - self.i_lpf)
        self.tau = self.i_lpf - self.w * (x - self.x_lpf)
        return self.tau
