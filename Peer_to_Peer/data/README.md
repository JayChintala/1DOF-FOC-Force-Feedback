# P2P data

## 2026-11-22: P2P vs teleop comparison

`2026-11-22_p2p_vs_teleop_logs.zip` (21 MB) holds the raw logs behind the comparison in [../testing.md](../testing.md) ("P2P vs the old teleop controller"). They're copied from the Pi's git-ignored `ESC_based/software/logs/`.

| Files | What |
|---|---|
| `p2p_WN20_Z1_KF50_G80_FR0_R100_FREE_air_A0.8*` | P2P free space (`--guide free`) |
| `p2p_..._TAPS_rigid_A0.8*` | P2P taps on the rigid stop |
| `p2p_..._CONTACT_rigid_A0.8_2*` | P2P presses and holds, rigid (the first attempt stopped in calibration and is excluded) |
| `p2p_..._CONTACT_sponge_A0.8*` | P2P presses and holds, sponge |
| `C3_kf100_kb3.5e-6*` | Teleop, C3 operating point (2026-10-25), free motion |
| `teleop_KVs3e-05_FB-1_{22,27,28}*` | Teleop, current operating point (2026-11-15 / 11-18) |

Each P2P run has `.csv` (1 kHz, one row per motor per tick), `.json` (settings, guide phases, summary) and `_cal.csv/_cal.json` (ripple calibration). Each teleop run has the main `.csv` (500 Hz loop), `_frames.csv` (1 kHz per-motor frames) and `_cal.json` (leader ripple calibration).

**Reproduce** (unzip, then run from that folder, with `ESC_based/software` on `PYTHONPATH`):

```
python3 p2p_compare.py p2p_*_FREE_air_A0.8.csv p2p_*_TAPS_rigid_A0.8.csv p2p_*_CONTACT_rigid_A0.8_2.csv p2p_*_CONTACT_sponge_A0.8.csv
python3 p2p_feel.py C3_kf100_kb3.5e-6 teleop_KVs3e-05_FB-1_22 teleop_KVs3e-05_FB-1_27 teleop_KVs3e-05_FB-1_28 p2p_WN20_Z1_KF50_G80_FR0_R100_FREE_air_A0.8
```

`2026-11-22_p2p_compare_results.json` is `p2p_compare.py`'s output for these runs.
