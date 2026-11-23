"""
Felt free-space dynamics on the operator's handle (motor B), for teleop and
P2P logs with the SAME code (Peer_to_Peer/testing.md, "P2P vs teleop").

    python3 p2p_feel.py C3_kf100_kb3.5e-6 teleop_KVs3e-05_FB-1_27 p2p_<...>_FREE_air_A0.8

(log base names, no extension; run from logs/). The teleop current on B
(iq_b_cmd) excludes the handle's own inertia and friction, which the hand pays
passively, while P2P's B current also cancels them, so neither controller's
current is "what the hand feels". This replays one hand-force observer
(g = 80 rad/s, G_B, ripple removed with each run's own calibration) on B's
measured current and encoder, then fits, over free motion,

    hand force = m * a + Fc * sign(v) + c      (a low-passed like the force)

m is the inertia the hand feels (nA per count/s^2; B's rotor alone is 113),
Fc the friction it feels. Free motion: teleop = |tau_env| < 80 mA with a
0.15 s margin; P2P = the --guide free phase.
"""
import csv, json, sys
import numpy as np

GB = 8.877e6   # counts/s^2 per A, motor B
g = 80.0       # observer cutoff, rad/s (the P2P operating point)
def lpf(x, us):
    y=np.zeros_like(x); y[0]=x[0]
    for i in range(1,len(x)):
        dt=(us[i]-us[i-1])*1e-6; y[i]=y[i-1]+min(1,g*dt)*(x[i]-y[i-1])
    return y
def hand_force(us, enc, iq, rip):
    n=len(us); v=np.zeros(n); f=np.zeros(n); k=g/GB; z=None; ie=iq-rip
    for i in range(n):
        dt=(us[i]-us[i-1])*1e-6 if i else 1e-3
        if i and dt>0: v[i]=0.5*(enc[i]-enc[i-1])/dt+0.5*v[i-1]
        elif i: v[i]=v[i-1]
        if z is None: z=ie[i]+k*v[i]
        z+=min(1,g*dt)*(ie[i]+k*v[i]-z); f[i]=-(z-k*v[i])
    return v,f
def rip_of(cal, enc):
    th=2*np.pi*(enc%4000)/4000
    if cal is None: return 0*enc
    if "harmonics_A" in cal: return sum(a*np.cos(int(h)*th)+b*np.sin(int(h)*th) for h,(a,b) in cal["harmonics_A"].items())
    return sum(cal["ab_A"][2*k]*np.cos(h*th)+cal["ab_A"][2*k+1]*np.sin(h*th) for k,h in enumerate(cal["harmonics"]))
def feel(us, enc, v, f, mask):
    t=(us-us[0])*1e-6; k=5; a=np.zeros_like(v); a[k:-k]=(v[2*k:]-v[:-2*k])/(t[2*k:]-t[:-2*k])
    af=lpf(a,us)                                  # same filter as the force
    m=mask&(np.abs(v)>300)
    if m.sum()<2000: return None
    X=np.vstack([af[m],np.sign(v[m]),np.ones(m.sum())]).T
    c,*_=np.linalg.lstsq(X,f[m],rcond=None); r2=1-np.var(f[m]-X@c)/np.var(f[m])
    return dict(n_s=mask.sum()/1000, frms=1e3*np.sqrt(np.mean(f[mask]**2)), vrms=np.sqrt(np.mean(v[mask]**2)),
                arms=np.sqrt(np.mean(a[mask]**2)), inertia=c[0]*1e9, fric=1e3*c[1], r2=r2)
def teleop(base):
    fr=[r for r in csv.DictReader(open(base+"_frames.csv")) if r["motor"]=="B" and int(r["loop_iter"])>=0]
    us=np.array([float(r["mcu_time_us"]) for r in fr]); enc=np.array([float(r["enc"]) for r in fr]); iq=np.array([float(r["iq_meas"]) for r in fr])
    try: cal=json.load(open(base+"_cal.json"))
    except FileNotFoundError: cal=None
    v,f=hand_force(us,enc,iq,rip_of(cal,enc))
    main=list(csv.DictReader(open(base+".csv"))); tm=np.array([float(r["t_s"]) for r in main]); te=np.array([abs(float(r["tau_env"])) if r["tau_env"] else 0 for r in main])
    bad=np.zeros(len(tm),bool)
    for i in np.where(te>0.08)[0]: bad[max(0,i-75):i+75]=True
    t=(us-us[0])*1e-6; free=~(np.interp(t,tm-tm[0],bad.astype(float))>0.5); free[:2000]=False
    return feel(us,enc,v,f,free)
def p2p(base):
    cfg=json.load(open(base+".json")); cal=json.load(open(base+"_cal.json"))
    allb=[r for r in csv.DictReader(open(base+".csv")) if r["motor"]=="B"]; rows=[r for r in allb if r["engaged"]=="1"]
    cr=[r for r in csv.DictReader(open(base+"_cal.csv")) if r["motor"]=="B"]; off=float(cr[-1]["enc"])-float(allb[0]["pos"])
    us=np.array([float(r["mcu_time_us"]) for r in rows]); enc=np.array([float(r["pos"]) for r in rows])+off; iq=np.array([float(r["iq_meas"]) for r in rows])
    v,f=hand_force(us,enc,iq,rip_of(cal["B"],enc))
    el=(us-cfg["engage_mcu_us"]["B"])*1e-6
    return feel(us,enc,v,f,(el>=5)&(el<35))
print("Inertia refs (nA per cnt/s^2): B alone %.0f | P2P Kf 50%% expected 2/Kf %.0f | passive pair %.0f"%(1e9/GB,1e9*2/(0.5*2*GB),1e9/2.796e6))
print(f"{'run':30s} {'free s':>6s} {'hand rms mA':>11s} {'|v| rms':>8s} {'|a| rms':>8s} {'inertia':>8s} {'friction mA':>11s} {'R2':>5s}")
for b in sys.argv[1:]:
    r=p2p(b) if b.startswith("p2p") else teleop(b)
    if r is None: print(f"{b:30s}  too little free motion"); continue
    print(f"{b[:30]:30s} {r['n_s']:6.1f} {r['frms']:11.1f} {r['vrms']:8.0f} {r['arms']/1e6:7.2f}M {r['inertia']:8.0f} {r['fric']:11.1f} {r['r2']:5.2f}")
