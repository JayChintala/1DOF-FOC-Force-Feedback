"""
Shared plotting utilities for the motor test CSV logs.

Each test writes its own CSV schema and calls the matching plotter here, so
all plotting code lives in one place rather than being duplicated per test:
  - plot_log                  <- position_hold_test.py   (single-motor PD hold)
  - plot_bilateral_log        <- bilateral_test.py        (two-motor coupling)
  - plot_torque_tracking_log  <- torque_tracking_test.py  (open-loop Iq steps)
  - plot_iq_probe_log         <- iq_readback_probe.py     (back-drive sensing probe)

Can be used two ways:
  1. Imported and called directly from a test script:
         from plot_run import plot_log
         plot_log("logs/position_hold_....csv")

         from plot_run import plot_bilateral_log
         plot_bilateral_log("logs/bilateral_....csv")

         from plot_run import plot_torque_tracking_log
         plot_torque_tracking_log("logs/torque_tracking....csv")

         from plot_run import plot_iq_probe_log
         plot_iq_probe_log("logs/iq_readback_probe....csv")

  2. Run standalone from the command line -- the format is auto-detected
     from the CSV header, so either log type works:
         python3 plot_run.py logs/position_hold_....csv
         python3 plot_run.py logs/bilateral_....csv

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


def load_bilateral_log(path):
    """
    Load a bilateral_test.py CSV. iq_*_actual columns may be blank in a row
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


def plot_bilateral_log(csv_path, out_path=None):
    """
    Read a bilateral_test.py CSV log and save a 4-panel diagnostic PNG.
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
    data = load_bilateral_log(csv_path)

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


def _detect_and_plot(csv_path):
    """
    Pick the right plotter by sniffing the CSV header, so the CLI accepts
    any log format. Bilateral logs carry 'pos_a'/'pos_b'; torque-tracking
    logs carry 'step_idx'/'iq_err'; iq-probe logs carry 'iq'/'pos_rev'
    (no cmd/target columns); position_hold logs carry 'target_pos'.
    """
    with open(csv_path, newline="") as f:
        header = set(next(csv.reader(f), []))
    if {"pos_a", "pos_b"}.issubset(header):
        return plot_bilateral_log(csv_path)
    if {"step_idx", "iq_err"}.issubset(header):
        return plot_torque_tracking_log(csv_path)
    if {"iq", "pos_rev"}.issubset(header):
        return plot_iq_probe_log(csv_path)
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
