# Offline check for a leader contact spring (option A, TEST_LOG 2026-10-29).
# Run: python3 leader_spring_sim.py
# Toy press sim (inferred model, for trends), extends hold_sim.py with follower
# motion and a wall of finite stiffness. +x = into the object; both start
# touching it at x = 0 (err = e0 = 0 at contact).
#   Leader:   xb'' = G_B*(h + mb - fric);  hand h = kh*(p_h - xb) - bh*vb,
#             p_h ramps 0 -> P_H over RAMP_S (a deliberate press), or with
#             hand_f a force-controlled push h = hand_f (ramped) - bh*vb
#             (the light-hold / wind-up check in light_holds()).
#   Leader motor mb = -felt - KB*vb - spring_L - BL*vb (spring/BL only in contact),
#             spring_L = clamp(KL*(err - e0), 0, IL), err = xb - xa.
#   Observer: h_hat = LPF20(ab/G_B - mb)  (sees the spring as motor current).
#   Follower: xa'' = G_A*(ia - Fw - fric);  ia = h_hat + KV*(vb - va) + hold_a,
#             delayed 3 ms; Fw = kw*max(0, xa).
#   tau_env = LPF20(Fw delayed 3 ms); felt = deadband(tau_env).
# Reports, per (KL, wall), leader travel from tau_env 0.14 A to 0.67 A (the
# measured 0.15 -> 0.7 N window), follower travel, the time, the final force,
# and leader velocity sign flips in contact (ringing).
import math
import sys

sys.path.insert(0, ".")
from contact_spring import ContactSpring

G_B, G_A = 7.9e6, 3.85e6
KB, KV, DB, FR_B, FR_A, CAP = 0.35e-5, 3e-5, 0.02, 0.03, 0.025, 0.8
WALLS = {"rigid": 0.22, "squishy": 0.024, "sponge": 0.012}   # A/count, from today's per-press data
P_H, RAMP_S, T = 900.0, 0.4, 2.0


def lpf(fc, dt):
    return 1 - math.exp(-2 * math.pi * fc * dt)


def run(kl, il, bl, kw, kh=3e-3, bh=3e-5, hold_k=0.5e-3, hold_max=0.05, hand_f=None, trace=None):
    dt = 1e-4
    al = lpf(20, dt)
    hold = ContactSpring(hold_k, hold_max, ramp_s=0.3) if hold_k else None
    xb = vb = xa = va = 0.0
    h_hat = tenv = 0.0
    ia = ib = 0.0
    qa = [0.0] * int(3e-3 / dt)          # command path to the follower
    qf = [0.0] * int(3e-3 / dt)          # contact force -> tau_env estimate
    e0, latched = 0.0, False
    t14 = t67 = None
    xb14 = xa14 = xb67 = xa67 = 0.0
    flips, last_sign = 0, 0
    for n in range(int(T / dt)):
        t = n * dt
        ph = P_H * min(1.0, t / RAMP_S)
        h = (hand_f * min(1.0, t / RAMP_S) if hand_f is not None else kh * (ph - xb)) - bh * vb
        err = xb - xa
        felt = max(0.0, tenv - DB)
        if n % 20 == 0:                  # 500 Hz host loop
            if not latched and tenv > 0.08:
                latched, e0 = True, err
            spring = min(max(kl * (err - e0), 0.0), il) if latched else 0.0
            damp = bl * vb if latched else 0.0
            ib = max(-CAP, min(CAP, -felt - KB * vb - spring - damp))
            hd = hold.update(t, -err, tenv, 0.0) if hold else 0.0
            ia = max(-CAP, min(CAP, h_hat + KV * (vb - va) - hd))
        # leader
        net = h + ib
        if abs(vb) < 50 and abs(net) < FR_B:
            ab, vb = 0.0, 0.0
        else:
            ab = G_B * (net - FR_B * (1 if vb > 0 or (vb == 0 and net > 0) else -1))
        vb += ab * dt
        xb += vb * dt
        h_hat += al * ((ab / G_B - ib) - h_hat)
        # follower
        qa.append(ia)
        i_now = qa.pop(0)
        fw = kw * max(0.0, xa) + (2e-5 * va if xa > 0 else 0.0)
        neta = i_now - fw
        if abs(va) < 50 and abs(neta) < FR_A:
            aa, va = 0.0, 0.0
        else:
            aa = G_A * (neta - FR_A * (1 if va > 0 or (va == 0 and neta > 0) else -1))
        va += aa * dt
        xa += va * dt
        qf.append(kw * max(0.0, xa))
        tenv += al * (qf.pop(0) - tenv)
        if t14 is None and tenv > 0.14:
            t14, xb14, xa14 = t, xb, xa
        if t67 is None and tenv > 0.67:
            t67, xb67, xa67 = t, xb, xa
        if latched and abs(vb) > 200:
            s = 1 if vb > 0 else -1
            if last_sign and s != last_sign:
                flips += 1
            last_sign = s
        if trace is not None and n % 1000 == 0:
            trace.append((t, tenv, xb, xa))
    lead = (xb67 - xb14) if t67 else float("nan")
    foll = (xa67 - xa14) if t67 else float("nan")
    return lead, foll, (t67 - t14) if t67 else float("nan"), tenv, xb, flips


def main():
    print("KL A/cnt   IL A   BL A/(cnt/s) | wall     | leader cnt  follower cnt  ms  | final tau_env A, leader pos | flips")
    for kl, il, bl in ((0, 0, 0), (2e-3, 0.4, 0), (5e-3, 0.4, 1e-5), (1e-2, 0.4, 1e-5),
                       (2e-2, 0.4, 2e-5), (4e-2, 0.4, 3e-5)):
        res = {}
        for w, kw in WALLS.items():
            lead, foll, ms, f, xb, fl = run(kl, il, bl, kw)
            res[w] = lead
            print(f"{kl:8.0e}  {il:4.2f}  {bl:8.0e}   | {w:8s} | {lead:9.0f}  {foll:10.0f}  {ms * 1e3:5.0f} | "
                  f"{f:5.2f}  {xb:6.0f} | {fl}")
        print(f"{'':32s}  leader travel sponge/rigid = {res['sponge'] / res['rigid']:.2f}")


def light_holds():
    print("\nForce-controlled hand (steady push), tau_env A / leader pos at 0.5, 1, 2, 4 s:")
    global T
    T = 4.0
    for f in (0.2, 0.4):
        for kl, il, bl in ((0, 0, 0), (5e-3, 0.4, 1e-5), (1e-2, 0.4, 1e-5)):
            for w in ("rigid", "sponge"):
                tr = []
                run(kl, il, bl, WALLS[w], hand_f=f, trace=tr)
                pick = [min(tr, key=lambda z: abs(z[0] - tt)) for tt in (0.5, 1.0, 2.0, 3.9)]
                print(f"  hand {f:.1f} A  KL {kl:6.0e} {w:6s}: "
                      + "  ".join(f"{z[1]:5.2f}/{z[2]:5.0f}" for z in pick))


if __name__ == "__main__":
    main()
    light_holds()
