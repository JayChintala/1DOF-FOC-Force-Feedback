# Offline check for contact_spring.py (TEST_LOG 2026-10-26). Run: python3 hold_sim.py
# Toy press-and-hold sim (inferred model, for trends). +x = into the wall.
# Leader: x'' = G_B*(h + m_b); hand h = kh*(p_h - x) - bh*v; motor m_b = -felt - KB*v.
# Hand observer: h_hat = LPF20(h) (ideal derivative-free observer).
# Follower against a rigid wall at 0: wall reaction F = max(0, i_a); tau_env = LPF20(delay3(F)).
# i_a = h_hat + KV*v_b + hold.  felt = deadband(tau_env).
import math, sys
sys.path.insert(0, ".")
from contact_spring import ContactSpring
GB=7.9e6; KB=0.35e-5; KV=3e-5; DB=0.02; FR=0.03   # leader Coulomb ~30 mA (running)
def run(K, kh, bh, imax=0.1, p_h=300.0, T=4.0):
    cs=ContactSpring(K, imax) if K else None
    h_hat=tenv=0.0; x=v=0.0; q=[0.0]*6; out=[]; dt=1e-3; al=1-math.exp(-2*math.pi*20*dt)
    ia=0.0
    for n in range(int(T/dt)):
        t=n*dt
        ph = p_h*min(1,t/0.3) if t<2.5 else p_h*max(0,1-(t-2.5)/0.3)
        h = kh*(ph-x) - bh*v
        felt = max(0.0, tenv-DB)
        mb = -felt - KB*v
        net = h+mb
        if abs(v)<50 and abs(net)<FR: a=0.0; v=0.0      # stiction
        else: a=GB*(net - FR*(1 if v>0 or (v==0 and net>0) else -1))
        v+=a*dt; x+=v*dt
        h_hat += al*((a/GB - mb)-h_hat)   # what the leader observer measures: a/G - motor current
        if n%2==0:   # 500 Hz host loop
            hold = cs.update(t, -x, tenv, 0.0) if cs else 0.0   # real convention: tau_env>0 <-> err falling
            # sign: tenv>0 => push dir s=-1; err=x>0 into wall -> make spring positive into wall
            ia = h_hat + KV*v + (-hold)
        q.append(max(0.0, ia)); F=q.pop(0)
        tenv += al*(F-tenv)
        out.append((t,tenv,x))
    return out
pick=lambda o,tt: min(o,key=lambda z:abs(z[0]-tt))
print("hand kh A/cnt, bh A/(cnt/s) | tau_env mA at 0.4/0.8/1.5/2.4 s | leader past contact cnt at 2.4 s | 3.5 s after release")
for K in (0, 0.25e-3, 0.5e-3, 1e-3, 2e-3):
    for kh,bh in ((2e-3,2e-5),(5e-3,4e-5)):
        o=run(K,kh,bh)
        tv=[pick(o,tt)[1]*1e3 for tt in (0.4,0.8,1.5,2.4)]
        late=[z[1] for z in o if 1.0<z[0]<2.5]
        print(f"K {K:7.2g} hand {kh:.0e}/{bh:.0e}: " + " ".join(f"{x:5.0f}" for x in tv)
              + f" | {pick(o,2.4)[2]:5.0f} | {pick(o,3.5)[1]*1e3:4.0f} mA | ripple 1-2.5 s {1e3*(max(late)-min(late)):4.0f} mA")
