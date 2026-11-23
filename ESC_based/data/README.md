# ESC_based data

## 2026-11-23: Motor parameters vs MC Workbench

`2026-11-23_motor_params_logs.zip` holds every `motor_params.py` run from that session (spin and stall, motors A and B). Each run has a `.csv` (DIAG3 rows: step, rx_time, vbus_v, vq_v, vd_v, iq_a) and a `.json` (per-step means, fit result, error if aborted). Results and caveats: `../software/TEST_LOG.md`, entry "Motor parameters vs MC Workbench".

Re-fit offline (unzip, then from `ESC_based/software/`): `python3 motor_params.py fit <path>/motor_params_spin_B_20261123_004431.json`. Use the ≥ 2 rev/s spin runs for Ke: `spin_B_..._004431` (0.5–4 rev/s; fit 2–4 only) and `spin_A_..._004702` (2–3 rev/s). The stall runs used for Rs are `stall_A_..._005208` and `stall_B_..._005245`; the other two stall files are aborted first attempts (shaft not yet blocked).
