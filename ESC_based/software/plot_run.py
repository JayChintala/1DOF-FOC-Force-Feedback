"""
Shared plotting utilities for the motor test CSV logs.

Each test writes its own CSV schema and calls the matching plotter here, so
all plotting code lives in one place rather than being duplicated per test:
  - plot_log                  <- position_hold_test.py   (single-motor PD hold)
  - plot_position_mirror_log        <- position_mirror_test.py        (two-motor coupling)
  - plot_torque_tracking_log  <- torque_tracking_test.py  (open-loop Iq steps)
  - plot_iq_probe_log         <- iq_readback_probe.py     (back-drive sensing probe)
  - plot_torque_mirror_log    <- torque_mirror_test.py    (bidirectional current mirror)
  - plot_teleop_log           <- teleop_test.py           (position slave + current feedback)

Can be used two ways:
  1. Imported and called directly from a test script:
         from plot_run import plot_log
         plot_log("logs/position_hold_....csv")

         from plot_run import plot_position_mirror_log
         plot_position_mirror_log("logs/position_mirror_....csv")

         from plot_run import plot_torque_tracking_log
         plot_torque_tracking_log("logs/torque_tracking....csv")

         from plot_run import plot_iq_probe_log
         plot_iq_probe_log("logs/iq_readback_probe....csv")

  2. Run standalone from the command line -- the format is auto-detected
     from the CSV header, so either log type works:
         python3 plot_run.py logs/position_hold_....csv
         python3 plot_run.py logs/position_mirror_....csv

Each produces a diagnostic PNG next to the CSV (same name, .png extension).

If matplotlib isn't installed:
    pip install matplotlib --break-system-packages
"""

import csv
import os
import sys

import matplotlib
matplotlib.use("Agg")  # no GUI needed -- just saves a PNG
import matplotlib.pyplot as plt


def load_log(path):
    rows = {"t": [], "pos": [], "target": [], "error": [], "vel": [],
            "iq_cmd": [], "iq_actual": []}
    with open(path, newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            rows["t"].append(float(row["t_s"]))
            rows["pos"].append(float(row["pos"]))
            rows["target"].append(float(row["target_pos"]))
            rows["error"].append(float(row["error"]))
            rows["vel"].append(float(row["vel"]))
            rows["iq_cmd"].append(float(row["iq_cmd"]))
            iq_act = row["iq_actual"]
            rows["iq_actual"].append(float(iq_act) if iq_act else None)
    return rows


def plot_log(csv_path, out_path=None):
    """
    Reads a position_hold_test.py CSV log and saves a 4-panel diagnostic
    PNG. Returns the output path, or None if there was no data to plot.

    csv_path: path to the CSV log
    out_path: optional explicit output path; defaults to csv_path with
              the extension swapped to .png
    """
    data = load_log(csv_path)

    if not data["t"]:
        print(f"No data rows found in {csv_path} -- nothing to plot.")
        return None

    fig, axes = plt.subplots(4, 1, figsize=(11, 12), sharex=True)

    axes[0].plot(data["t"], data["pos"], color="tab:blue", label="pos")
    axes[0].plot(data["t"], data["target"], color="gray", linestyle="--",
                 linewidth=1, label="target")
    axes[0].set_ylabel("Position\n(counts)")
    axes[0].legend(loc="upper right", fontsize=8)
    axes[0].set_title(os.path.basename(csv_path))

    axes[1].plot(data["t"], data["vel"], color="tab:red")
    axes[1].axhline(0, color="gray", linestyle="--", linewidth=1)
    axes[1].set_ylabel("Velocity\n(counts/s)")

    axes[2].plot(data["t"], data["iq_cmd"], color="tab:green",
                 label="Iq_cmd", linewidth=1.2)
    iq_max = max((abs(v) for v in data["iq_cmd"]), default=0)
    if any(v is not None for v in data["iq_actual"]):
        t_act = [t for t, v in zip(data["t"], data["iq_actual"]) if v is not None]
        v_act = [v for v in data["iq_actual"] if v is not None]
        axes[2].plot(t_act, v_act, color="tab:orange", label="Iq_actual",
                     alpha=0.6, linewidth=0.8)
    axes[2].axhline(iq_max, color="gray", linestyle=":", linewidth=0.8)
    axes[2].axhline(-iq_max, color="gray", linestyle=":", linewidth=0.8)
    axes[2].set_ylabel("Current (A)")
    axes[2].legend(loc="upper right", fontsize=8)

    axes[3].plot(data["t"], data["error"], color="tab:purple")
    axes[3].axhline(0, color="gray", linestyle="--", linewidth=1)
    axes[3].set_ylabel("Error\n(counts)")
    axes[3].set_xlabel("Time (s)")

    plt.tight_layout()

    if out_path is None:
        out_path = os.path.splitext(csv_path)[0] + ".png"
    plt.savefig(out_path, dpi=150)
    plt.close(fig)  # important when called repeatedly from another script
    return out_path


def load_position_mirror_log(path):
    """
    Load a position_mirror_test.py CSV. iq_*_actual columns may be blank in a row
    (telemetry not yet seen), so those are kept nullable and filtered by the
    plotter; every other column is numeric.
    """
    rows = {k: [] for k in ("t", "d_a", "d_b", "err", "vel_a", "vel_b",
                            "iq_a_cmd", "iq_b_cmd", "iq_a_actual", "iq_b_actual")}
    with open(path, newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            rows["t"].append(float(row["t_s"]))
            rows["d_a"].append(float(row["d_a"]))
            rows["d_b"].append(float(row["d_b"]))
            rows["err"].append(float(row["err"]))
            rows["vel_a"].append(float(row["vel_a"]))
            rows["vel_b"].append(float(row["vel_b"]))
            rows["iq_a_cmd"].append(float(row["iq_a_cmd"]))
            rows["iq_b_cmd"].append(float(row["iq_b_cmd"]))
            rows["iq_a_actual"].append(float(row["iq_a_actual"]) if row["iq_a_actual"] else None)
            rows["iq_b_actual"].append(float(row["iq_b_actual"]) if row["iq_b_actual"] else None)
    return rows


def plot_position_mirror_log(csv_path, out_path=None):
    """
    Read a position_mirror_test.py CSV log and save a 4-panel diagnostic PNG.
    Returns the output path, or None if there was no data to plot.

    Panels (shared time axis):
      1. Displacement of each shaft from its baseline (d_a vs d_b) -- how
         well B tracks A. The gap between the traces is the coupling error.
      2. Coupling error (err = d_a - d_b) -- the position disagreement the
         virtual spring acts on; this is what generates the felt force.
      3. Filtered per-shaft velocity (vel_a, vel_b).
      4. Commanded current per motor (iq_a_cmd, iq_b_cmd, equal-and-opposite
         by construction) with measured IQ_READBACK overlaid faintly and
         +/- clamp reference lines drawn from the largest command magnitude.

    csv_path: path to the CSV log.
    out_path: optional explicit output path; defaults to csv_path with the
              extension swapped to .png.
    """
    data = load_position_mirror_log(csv_path)

    if not data["t"]:
        print(f"No data rows found in {csv_path} -- nothing to plot.")
        return None

    fig, axes = plt.subplots(4, 1, figsize=(11, 12), sharex=True)

    # Panel 1: per-shaft displacement -- the tracking view.
    axes[0].plot(data["t"], data["d_a"], color="tab:blue", label="A displacement")
    axes[0].plot(data["t"], data["d_b"], color="tab:orange", label="B displacement")
    axes[0].set_ylabel("Displacement\n(counts)")
    axes[0].legend(loc="upper right", fontsize=8)
    axes[0].set_title(os.path.basename(csv_path))

    # Panel 2: coupling error -- source of the felt force.
    axes[1].plot(data["t"], data["err"], color="tab:purple")
    axes[1].axhline(0, color="gray", linestyle="--", linewidth=1)
    axes[1].set_ylabel("Coupling err\n(counts)")

    # Panel 3: velocities feeding the KD damping term.
    axes[2].plot(data["t"], data["vel_a"], color="tab:blue", label="vel A")
    axes[2].plot(data["t"], data["vel_b"], color="tab:orange", label="vel B")
    axes[2].axhline(0, color="gray", linestyle="--", linewidth=1)
    axes[2].set_ylabel("Velocity\n(counts/s)")
    axes[2].legend(loc="upper right", fontsize=8)

    # Panel 4: commanded current (solid) vs measured IQ_READBACK (faint).
    axes[3].plot(data["t"], data["iq_a_cmd"], color="tab:blue",
                 label="iq A cmd", linewidth=1.2)
    axes[3].plot(data["t"], data["iq_b_cmd"], color="tab:orange",
                 label="iq B cmd", linewidth=1.2)
    for key, color in (("iq_a_actual", "tab:cyan"), ("iq_b_actual", "tab:red")):
        t_act = [t for t, v in zip(data["t"], data[key]) if v is not None]
        v_act = [v for v in data[key] if v is not None]
        if t_act:
            axes[3].plot(t_act, v_act, color=color, alpha=0.5, linewidth=0.7,
                         label=key.replace("iq_", "iq ").replace("_", " "))
    iq_max = max((abs(v) for v in data["iq_a_cmd"] + data["iq_b_cmd"]), default=0)
    if iq_max > 0:
        axes[3].axhline(iq_max, color="gray", linestyle=":", linewidth=0.8)
        axes[3].axhline(-iq_max, color="gray", linestyle=":", linewidth=0.8)
    axes[3].set_ylabel("Current (A)")
    axes[3].set_xlabel("Time (s)")
    axes[3].legend(loc="upper right", fontsize=7, ncol=2)

    plt.tight_layout()

    if out_path is None:
        out_path = os.path.splitext(csv_path)[0] + ".png"
    plt.savefig(out_path, dpi=150)
    plt.close(fig)  # important when called repeatedly from another script
    return out_path


def load_torque_tracking_log(path):
    """
    Load a torque_tracking_test.py CSV. iq_actual/iq_filtered/iq_err/
    iq_readback_age_s may be blank in a row (telemetry not yet seen), so
    those are kept nullable; every other column is numeric. Older logs
    predating the iq_filtered column simply won't have it -- treated the
    same as an all-blank column.
    """
    rows = {k: [] for k in ("t", "step_idx", "iq_cmd", "iq_actual",
                             "iq_filtered", "iq_err", "vel", "age")}
    with open(path, newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            rows["t"].append(float(row["t_s"]))
            rows["step_idx"].append(int(row["step_idx"]))
            rows["iq_cmd"].append(float(row["iq_cmd"]))
            rows["iq_actual"].append(float(row["iq_actual"]) if row["iq_actual"] else None)
            rows["iq_filtered"].append(float(row["iq_filtered"]) if row.get("iq_filtered") else None)
            rows["iq_err"].append(float(row["iq_err"]) if row["iq_err"] else None)
            rows["vel"].append(float(row["vel"]))
            rows["age"].append(float(row["iq_readback_age_s"]) if row["iq_readback_age_s"] else None)
    return rows


def plot_torque_tracking_log(csv_path, out_path=None):
    """
    Read a torque_tracking_test.py CSV log and save a 4-panel diagnostic
    PNG. Returns the output path, or None if there was no data to plot.

    Panels (shared time axis):
      1. Commanded Iq (the staircase) vs raw IQ_READBACK vs the 3-tap
         causal median-filtered IQ_READBACK -- how closely actual current
         tracks each commanded step, and how much the filter cleans up
         the raw noise.
      2. Tracking error (filtered - cmd) -- should collapse toward zero
         after each step's transient. Uses the filtered signal since
         that's what the PASS/FAIL report is judged against.
      3. Shaft velocity -- sanity check that a step didn't run away
         (this is what the watchdog in the test script guards against).
      4. IQ_READBACK age at each sample -- how stale the telemetry the
         controller is acting on ever gets; spikes here mean the CAN
         link isn't keeping up, not that the current loop is wrong.

    csv_path: path to the CSV log.
    out_path: optional explicit output path; defaults to csv_path with the
              extension swapped to .png.
    """
    data = load_torque_tracking_log(csv_path)

    if not data["t"]:
        print(f"No data rows found in {csv_path} -- nothing to plot.")
        return None

    fig, axes = plt.subplots(4, 1, figsize=(11, 12), sharex=True)

    # Panel 1: commanded staircase vs raw vs filtered measured current.
    axes[0].plot(data["t"], data["iq_cmd"], color="tab:green",
                 label="Iq_cmd", linewidth=1.2)
    t_act = [t for t, v in zip(data["t"], data["iq_actual"]) if v is not None]
    v_act = [v for v in data["iq_actual"] if v is not None]
    if t_act:
        axes[0].plot(t_act, v_act, color="tab:orange", label="Iq_actual (raw)",
                     alpha=0.4, linewidth=0.6)
    t_filt = [t for t, v in zip(data["t"], data["iq_filtered"]) if v is not None]
    v_filt = [v for v in data["iq_filtered"] if v is not None]
    if t_filt:
        axes[0].plot(t_filt, v_filt, color="tab:red", label="Iq_filtered",
                     alpha=0.9, linewidth=1.0)
    axes[0].set_ylabel("Current (A)")
    axes[0].legend(loc="upper right", fontsize=8)
    axes[0].set_title(os.path.basename(csv_path))

    # Panel 2: tracking error.
    t_err = [t for t, v in zip(data["t"], data["iq_err"]) if v is not None]
    v_err = [v for v in data["iq_err"] if v is not None]
    axes[1].plot(t_err, v_err, color="tab:purple")
    axes[1].axhline(0, color="gray", linestyle="--", linewidth=1)
    axes[1].set_ylabel("Tracking error\n(A)")

    # Panel 3: velocity -- confirms no step ran away.
    axes[2].plot(data["t"], data["vel"], color="tab:red")
    axes[2].axhline(0, color="gray", linestyle="--", linewidth=1)
    axes[2].set_ylabel("Velocity\n(counts/s)")

    # Panel 4: telemetry staleness at time of use.
    t_age = [t for t, v in zip(data["t"], data["age"]) if v is not None]
    v_age = [v for v in data["age"] if v is not None]
    axes[3].plot(t_age, v_age, color="tab:brown")
    axes[3].set_ylabel("IQ_READBACK\nage (s)")
    axes[3].set_xlabel("Time (s)")

    plt.tight_layout()

    if out_path is None:
        out_path = os.path.splitext(csv_path)[0] + ".png"
    plt.savefig(out_path, dpi=150)
    plt.close(fig)  # important when called repeatedly from another script
    return out_path


def load_iq_probe_log(path):
    """
    Load an iq_readback_probe.py CSV. iq/iq_readback_age_s/pos_rev may be
    blank in a row (telemetry not yet seen), so those are kept nullable.
    """
    rows = {k: [] for k in ("t", "iq", "age", "pos")}
    with open(path, newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            rows["t"].append(float(row["t_s"]))
            rows["iq"].append(float(row["iq"]) if row["iq"] else None)
            rows["age"].append(float(row["iq_readback_age_s"]) if row["iq_readback_age_s"] else None)
            rows["pos"].append(float(row["pos_rev"]) if row["pos_rev"] else None)
    return rows


def plot_iq_probe_log(csv_path, out_path=None):
    """
    Read an iq_readback_probe.py CSV log and save a 3-panel diagnostic
    PNG. Returns the output path, or None if there was no data to plot.

    Panels (shared time axis):
      1. IQ_READBACK while Iq is commanded to 0 -- what shows up when the
         shaft is pushed by hand.
      2. Shaft position (rev) -- correlates pushes/twists with panel 1.
      3. IQ_READBACK age -- staleness of the telemetry each sample was
         read from; spikes/plateaus here are CAN/firmware dropouts, not
         evidence about the current loop itself.

    csv_path: path to the CSV log.
    out_path: optional explicit output path; defaults to csv_path with the
              extension swapped to .png.
    """
    data = load_iq_probe_log(csv_path)

    if not data["t"]:
        print(f"No data rows found in {csv_path} -- nothing to plot.")
        return None

    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)

    t_iq = [t for t, v in zip(data["t"], data["iq"]) if v is not None]
    v_iq = [v for v in data["iq"] if v is not None]
    axes[0].plot(t_iq, v_iq, color="tab:orange", linewidth=0.8)
    axes[0].axhline(0, color="gray", linestyle="--", linewidth=1)
    axes[0].set_ylabel("IQ_READBACK\n(A)")
    axes[0].set_title(os.path.basename(csv_path))

    t_pos = [t for t, v in zip(data["t"], data["pos"]) if v is not None]
    v_pos = [v for v in data["pos"] if v is not None]
    axes[1].plot(t_pos, v_pos, color="tab:blue")
    axes[1].set_ylabel("Position\n(rev)")

    t_age = [t for t, v in zip(data["t"], data["age"]) if v is not None]
    v_age = [v for v in data["age"] if v is not None]
    axes[2].plot(t_age, v_age, color="tab:brown")
    axes[2].set_ylabel("IQ_READBACK\nage (s)")
    axes[2].set_xlabel("Time (s)")

    plt.tight_layout()

    if out_path is None:
        out_path = os.path.splitext(csv_path)[0] + ".png"
    plt.savefig(out_path, dpi=150)
    plt.close(fig)  # important when called repeatedly from another script
    return out_path


def load_torque_mirror_log(path):
    """
    Load a torque_mirror_test.py CSV. All columns are numeric and always
    populated -- rows are only written once a fresh reading from both
    motors is available. iq1_filt/iq2_filt may be absent in older logs
    predating the EMA filter -- treated as an all-blank/nullable column.
    """
    rows = {k: [] for k in ("t", "iq1_readback", "iq2_readback",
                             "iq1_filt", "iq2_filt",
                             "iq1_cmd", "iq2_cmd", "dev_a", "dev_b")}
    with open(path, newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            rows["t"].append(float(row["t_s"]))
            rows["iq1_readback"].append(float(row["iq1_readback"]))
            rows["iq2_readback"].append(float(row["iq2_readback"]))
            rows["iq1_filt"].append(float(row["iq1_filt"]) if row.get("iq1_filt") else None)
            rows["iq2_filt"].append(float(row["iq2_filt"]) if row.get("iq2_filt") else None)
            rows["iq1_cmd"].append(float(row["iq1_cmd"]))
            rows["iq2_cmd"].append(float(row["iq2_cmd"]))
            rows["dev_a"].append(float(row["dev_a"]))
            rows["dev_b"].append(float(row["dev_b"]))
    return rows


def plot_torque_mirror_log(csv_path, out_path=None):
    """
    Read a torque_mirror_test.py CSV log and save a 3-panel diagnostic PNG.
    Returns the output path, or None if there was no data to plot.

    Panels (shared time axis):
      1. Motor 1: commanded Iq vs measured IQ_READBACK.
      2. Motor 2: commanded Iq vs measured IQ_READBACK.
      3. Each motor's deviation from its own bias (dev_a, dev_b) -- the
         signal actually being mirrored into the other motor's command.
         Sustained large deviations on both sides at once, or ringing,
         indicate the loop gain is too high for a stable bidirectional
         coupling.

    csv_path: path to the CSV log.
    out_path: optional explicit output path; defaults to csv_path with the
              extension swapped to .png.
    """
    data = load_torque_mirror_log(csv_path)

    if not data["t"]:
        print(f"No data rows found in {csv_path} -- nothing to plot.")
        return None

    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)

    axes[0].plot(data["t"], data["iq1_cmd"], color="tab:blue",
                 label="iq1 cmd", linewidth=1.2)
    axes[0].plot(data["t"], data["iq1_readback"], color="tab:cyan",
                 label="iq1 readback (raw)", alpha=0.4, linewidth=0.6)
    t1f = [t for t, v in zip(data["t"], data["iq1_filt"]) if v is not None]
    v1f = [v for v in data["iq1_filt"] if v is not None]
    if t1f:
        axes[0].plot(t1f, v1f, color="navy", label="iq1 filt", linewidth=1.0)
    axes[0].set_ylabel("Motor 1\nCurrent (A)")
    axes[0].legend(loc="upper right", fontsize=8)
    axes[0].set_title(os.path.basename(csv_path))

    axes[1].plot(data["t"], data["iq2_cmd"], color="tab:orange",
                 label="iq2 cmd", linewidth=1.2)
    axes[1].plot(data["t"], data["iq2_readback"], color="tab:red",
                 label="iq2 readback (raw)", alpha=0.4, linewidth=0.6)
    t2f = [t for t, v in zip(data["t"], data["iq2_filt"]) if v is not None]
    v2f = [v for v in data["iq2_filt"] if v is not None]
    if t2f:
        axes[1].plot(t2f, v2f, color="darkred", label="iq2 filt", linewidth=1.0)
    axes[1].set_ylabel("Motor 2\nCurrent (A)")
    axes[1].legend(loc="upper right", fontsize=8)

    axes[2].plot(data["t"], data["dev_a"], color="tab:blue", label="dev A")
    axes[2].plot(data["t"], data["dev_b"], color="tab:orange", label="dev B")
    axes[2].axhline(0, color="gray", linestyle="--", linewidth=1)
    axes[2].set_ylabel("Deviation\nfrom bias (A)")
    axes[2].set_xlabel("Time (s)")
    axes[2].legend(loc="upper right", fontsize=8)

    plt.tight_layout()

    if out_path is None:
        out_path = os.path.splitext(csv_path)[0] + ".png"
    plt.savefig(out_path, dpi=150)
    plt.close(fig)  # important when called repeatedly from another script
    return out_path


def load_teleop_log(path):
    """
    Load a teleop_test.py CSV. iq_a_readback/iq_b_readback may be blank in
    a row (telemetry not yet seen), so those are kept nullable; every
    other column is numeric.

    The latency-instrumentation columns (enc_age_a/enc_age_b/iq_age_a/
    enc_lag_a/enc_lag_b/fresh_a/fresh_b/loop_dt) are absent from logs
    recorded before they were added, so they're read only if the header carries them -- older logs
    still load and plot, just without the latency panel.
    """
    rows = {k: [] for k in ("t", "d_a", "d_b", "err", "vel_a", "vel_b",
                             "iq_a_cmd", "iq_a_readback", "iq_a_filt",
                             "iq_b_cmd", "iq_b_readback",
                             "enc_age_a", "enc_age_b", "iq_age_a",
                             "enc_lag_a", "enc_lag_b",
                             "fresh_a", "fresh_b", "loop_dt",
                             "tau_h", "iq_a_ff")}
    with open(path, newline="") as f:
        r = csv.DictReader(f)
        has_latency = r.fieldnames is not None and "enc_age_a" in r.fieldnames
        has_obs = r.fieldnames is not None and "tau_h" in r.fieldnames
        for row in r:
            if has_obs:
                for k in ("tau_h", "iq_a_ff"):
                    rows[k].append(float(row[k]) if row[k] else None)
            if has_latency:
                for k in ("enc_age_a", "enc_age_b", "iq_age_a",
                          "enc_lag_a", "enc_lag_b",
                          "fresh_a", "fresh_b", "loop_dt"):
                    rows[k].append(float(row[k]) if row[k] else None)
            rows["t"].append(float(row["t_s"]))
            rows["d_a"].append(float(row["d_a"]))
            rows["d_b"].append(float(row["d_b"]))
            rows["err"].append(float(row["err"]))
            rows["vel_a"].append(float(row["vel_a"]))
            rows["vel_b"].append(float(row["vel_b"]))
            rows["iq_a_cmd"].append(float(row["iq_a_cmd"]))
            rows["iq_a_readback"].append(float(row["iq_a_readback"]) if row["iq_a_readback"] else None)
            rows["iq_a_filt"].append(float(row["iq_a_filt"]))
            rows["iq_b_cmd"].append(float(row["iq_b_cmd"]))
            rows["iq_b_readback"].append(float(row["iq_b_readback"]) if row["iq_b_readback"] else None)
    return rows


def plot_teleop_log(csv_path, out_path=None):
    """
    Read a teleop_test.py CSV log and save a 4- or 5-panel diagnostic PNG
    (the 5th, latency, panel appears only for logs that carry the
    instrumentation columns). Returns the output path, or None if there was
    no data to plot.

    Panels (shared time axis):
      1. Displacement of each shaft from its baseline (d_a vs d_b) -- how
         well the slave (A) tracks the controller (B).
      2. Position coupling error (err = d_b - d_a) -- what Channel 1 acts
         on to drive Motor A.
      3. Motor A (slave) current: commanded vs raw vs EMA-filtered
         readback -- the effort A is spending to track/fight resistance.
      4. Motor B (controller) current: commanded vs measured readback --
         the force reflected back to the hand from Motor A's filtered
         current (Channel 2).
      5. Telemetry age, RX handling lag and loop period (ms), plus the
         fraction of recent iterations that saw a new encoder sample -- the
         phase lag actually present in the loop. Ages are measured from the
         kernel's receive timestamp; the handling-lag traces are the part of
         that age caused by this process parsing the frame late (measured
         p99 ~33 ms for the second interface, see telemetry_rate_probe.py).
         Only drawn if the log has those columns.

    csv_path: path to the CSV log.
    out_path: optional explicit output path; defaults to csv_path with the
              extension swapped to .png.
    """
    data = load_teleop_log(csv_path)

    if not data["t"]:
        print(f"No data rows found in {csv_path} -- nothing to plot.")
        return None

    has_latency = any(v is not None for v in data["enc_age_a"])
    n_panels = 5 if has_latency else 4
    fig, axes = plt.subplots(n_panels, 1, figsize=(11, 3 * n_panels), sharex=True)

    axes[0].plot(data["t"], data["d_a"], color="tab:blue", label="A displacement (slave)")
    axes[0].plot(data["t"], data["d_b"], color="tab:orange", label="B displacement (master)")
    axes[0].set_ylabel("Displacement\n(counts)")
    axes[0].legend(loc="upper right", fontsize=8)
    axes[0].set_title(os.path.basename(csv_path))

    axes[1].plot(data["t"], data["err"], color="tab:purple")
    axes[1].axhline(0, color="gray", linestyle="--", linewidth=1)
    axes[1].set_ylabel("Coupling err\n(counts)")

    axes[2].plot(data["t"], data["iq_a_cmd"], color="tab:blue",
                 label="iq A cmd", linewidth=1.2)
    t_act = [t for t, v in zip(data["t"], data["iq_a_readback"]) if v is not None]
    v_act = [v for v in data["iq_a_readback"] if v is not None]
    if t_act:
        axes[2].plot(t_act, v_act, color="tab:cyan", label="iq A readback (raw)",
                     alpha=0.4, linewidth=0.6)
    axes[2].plot(data["t"], data["iq_a_filt"], color="navy", label="iq A filt",
                 linewidth=1.0)
    # Hand-torque observer, when the log has it: tau_h is the estimated hand
    # torque on Motor B (in amps of equivalent current) and iq_a_ff is the
    # feedforward current it produced. If iq_a_ff ever opposes the direction of
    # the push, FF_GAIN has the wrong sign -- that is what this shows.
    if any(v is not None for v in data["tau_h"]):
        t_th = [t for t, v in zip(data["t"], data["tau_h"]) if v is not None]
        v_th = [v for v in data["tau_h"] if v is not None]
        axes[2].plot(t_th, v_th, color="tab:green", label="tau_h (est. hand)",
                     linewidth=1.0, alpha=0.8)
        t_ff = [t for t, v in zip(data["t"], data["iq_a_ff"]) if v is not None]
        v_ff = [v for v in data["iq_a_ff"] if v is not None]
        axes[2].plot(t_ff, v_ff, color="tab:olive", label="iq A feedforward",
                     linewidth=1.0, linestyle="--")
    axes[2].set_ylabel("Motor A\nCurrent (A)")
    axes[2].legend(loc="upper right", fontsize=8)

    axes[3].plot(data["t"], data["iq_b_cmd"], color="tab:orange",
                 label="iq B cmd", linewidth=1.2)
    t_bct = [t for t, v in zip(data["t"], data["iq_b_readback"]) if v is not None]
    v_bct = [v for v in data["iq_b_readback"] if v is not None]
    if t_bct:
        axes[3].plot(t_bct, v_bct, color="tab:red", label="iq B readback",
                     alpha=0.5, linewidth=0.7)
    axes[3].set_ylabel("Motor B\nCurrent (A)")
    axes[3].legend(loc="upper right", fontsize=8)

    if has_latency:
        for key, color, label in (
            ("enc_age_a", "tab:blue", "enc age A"),
            ("enc_age_b", "tab:orange", "enc age B"),
            ("iq_age_a", "tab:cyan", "iq age A"),
            ("enc_lag_a", "tab:red", "RX handling lag A"),
            ("enc_lag_b", "tab:purple", "RX handling lag B"),
            ("loop_dt", "gray", "loop dt"),
        ):
            t_v = [t for t, v in zip(data["t"], data[key]) if v is not None]
            v_v = [1000.0 * v for v in data[key] if v is not None]
            if t_v:
                axes[4].plot(t_v, v_v, color=color, label=label, linewidth=0.8)
        axes[4].set_ylabel("Telemetry age /\nloop period (ms)")
        axes[4].legend(loc="upper left", fontsize=8)

        # Fresh-sample fraction on a twin axis: rolling mean of fresh_a over
        # ~0.5 s. 1.0 means every iteration got a new encoder sample; 0.6
        # means 40% of the loop's torque commands were computed from a
        # position it had already used.
        fresh = [v for v in data["fresh_a"] if v is not None]
        if fresh:
            t_fresh = [t for t, v in zip(data["t"], data["fresh_a"]) if v is not None]
            dt_med = (t_fresh[-1] - t_fresh[0]) / max(1, len(t_fresh) - 1)
            win = max(1, int(round(0.5 / dt_med)) if dt_med > 0 else 1)
            roll = []
            run_sum = 0.0
            for i, v in enumerate(fresh):
                run_sum += v
                if i >= win:
                    run_sum -= fresh[i - win]
                roll.append(run_sum / min(i + 1, win))
            ax_fresh = axes[4].twinx()
            ax_fresh.plot(t_fresh, roll, color="tab:green", linewidth=1.0,
                          label="fresh enc A (0.5s mean)")
            ax_fresh.set_ylabel("Fresh sample\nfraction")
            ax_fresh.set_ylim(0, 1.05)
            ax_fresh.legend(loc="lower right", fontsize=8)

    axes[-1].set_xlabel("Time (s)")

    plt.tight_layout()

    if out_path is None:
        out_path = os.path.splitext(csv_path)[0] + ".png"
    plt.savefig(out_path, dpi=150)
    plt.close(fig)  # important when called repeatedly from another script
    return out_path


def _detect_and_plot(csv_path):
    """
    Pick the right plotter by sniffing the CSV header, so the CLI accepts
    any log format. Bilateral logs carry 'pos_a'/'pos_b'; torque-tracking
    logs carry 'step_idx'/'iq_err'; iq-probe logs carry 'iq'/'pos_rev'
    (no cmd/target columns); position_hold logs carry 'target_pos';
    torque-mirror logs carry 'iq1_cmd'/'iq2_cmd' (no step_idx/pos columns);
    teleop logs carry 'iq_a_filt'/'iq_b_cmd' (position + current columns
    together, unlike either torque-mirror or position-mirror alone).
    """
    with open(csv_path, newline="") as f:
        header = set(next(csv.reader(f), []))
    if {"iq_a_filt", "iq_b_cmd"}.issubset(header):
        return plot_teleop_log(csv_path)
    if {"pos_a", "pos_b"}.issubset(header):
        return plot_position_mirror_log(csv_path)
    if {"step_idx", "iq_err"}.issubset(header):
        return plot_torque_tracking_log(csv_path)
    if {"iq", "pos_rev"}.issubset(header):
        return plot_iq_probe_log(csv_path)
    if {"iq1_cmd", "iq2_cmd"}.issubset(header):
        return plot_torque_mirror_log(csv_path)
    return plot_log(csv_path)


def main():
    if len(sys.argv) != 2:
        print("Usage: python3 plot_run.py <path-to-csv>")
        sys.exit(1)

    out_path = _detect_and_plot(sys.argv[1])
    if out_path:
        print(f"Saved plot: {out_path}")


if __name__ == "__main__":
    main()
