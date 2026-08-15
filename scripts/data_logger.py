"""
data_logger.py — logging for teleoperation runs, and the plotting pass.

Two jobs:

  1. DataLogger — imported by teleop.py. No matplotlib at module level, so it
     stays safe to import inside Isaac Sim's interpreter.

  2. Plotting — run this file directly with your system Python (the one with
     matplotlib) to turn a finished run into figures:

         python scripts/data_logger.py data/run_20260812_141530_events.csv

Two files per run, both with exactly the seven fields the brief specifies:

    data/run_<stamp>_events.csv      the deliverable — one row per event
    data/run_<stamp>_telemetry.csv   supporting data — sampled at TELEMETRY_HZ

Splitting them keeps the deliverable small and readable (a handful of rows
per cycle) while still giving the plots a continuous signal to draw. Both
share one schema, so they concatenate cleanly if you want a single table.

    timestamp_sim    Isaac Sim simulation time (seconds)
    timestamp_wall   wall-clock time (unix epoch seconds)
    event            package_spawned | package_arrived | grasp | lift |
                     drop | grip_lost | package_lost   (telemetry: sample)
    joint_positions  all joint angles at that moment, JSON list
    ee_position      end-effector XYZ in world frame, JSON list
    gripper_state    open | closed
    package_id       which package is being handled

Cycle numbers are not stored — they are derived at plot time by counting
`drop` events, so the CSV carries nothing that is not in the spec.
"""

import csv
import json
import os
import sys
import time

# Our settings module is `sim_config`, not `config` — Isaac Sim's bundled
# OpenCV puts a clashing `config.py` on sys.path. Make this directory win.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

FIELDS = ["timestamp_sim", "timestamp_wall", "event",
          "joint_positions", "ee_position", "gripper_state", "package_id"]

# Discrete events, in the order a cycle produces them.
CYCLE_EVENTS = ("package_spawned", "package_arrived", "grasp", "lift", "drop")

# Categorical palette, fixed order — validated for six series:
# adjacent-pair CVD dE 9.1 (protan), normal-vision dE 19.6. Never reorder.
SERIES_COLORS = ["#2a78d6", "#eb6834", "#1baf7a",
                 "#eda100", "#e87ba4", "#008300"]
INK_PRIMARY, INK_SECONDARY, INK_MUTED = "#0b0b0b", "#52514e", "#8a8880"
SURFACE, GRIDLINE = "#fcfcfb", "#e6e5e1"

EVENT_STYLE = {
    "package_arrived": (INK_MUTED, "arrived"),
    "grasp":           ("#008300", "grasp"),
    "lift":            ("#2a78d6", "lift"),
    "drop":            ("#eb6834", "drop"),
    "grip_lost":       ("#e34948", "grip lost"),
}


# ===========================================================================
# Logging
# ===========================================================================
class DataLogger:
    """Writes the event log and the telemetry stream side by side."""

    def __init__(self, out_dir=None, run_name=None, joint_names=None):
        if out_dir is None:
            import sim_config as C
            out_dir = C.DATA_DIR
        self.out_dir = os.path.normpath(out_dir)
        os.makedirs(self.out_dir, exist_ok=True)

        self.run_name = run_name or f"run_{time.strftime('%Y%m%d_%H%M%S')}"
        base = os.path.join(self.out_dir, self.run_name)
        self.events_path = f"{base}_events.csv"
        self.telemetry_path = f"{base}_telemetry.csv"
        self.meta_path = f"{base}_meta.json"

        self._ev_fh = open(self.events_path, "w", newline="", encoding="utf-8")
        self._tm_fh = open(self.telemetry_path, "w", newline="", encoding="utf-8")
        self._ev = csv.DictWriter(self._ev_fh, fieldnames=FIELDS)
        self._tm = csv.DictWriter(self._tm_fh, fieldnames=FIELDS)
        self._ev.writeheader()
        self._tm.writeheader()

        with open(self.meta_path, "w", encoding="utf-8") as f:
            json.dump({"run_name": self.run_name,
                       "joint_names": list(joint_names or []),
                       "created": time.strftime("%Y-%m-%d %H:%M:%S")},
                      f, indent=2)

        self.cycles = 0
        self._counts = {}
        self._n_events = self._n_samples = 0
        print(f"[logger] events    -> {self.events_path}")
        print(f"[logger] telemetry -> {self.telemetry_path}")

    @staticmethod
    def _encode(row):
        return {
            "timestamp_sim":   round(float(row.get("timestamp_sim", 0.0)), 4),
            "timestamp_wall":  round(float(row.get("timestamp_wall", 0.0)), 4),
            "event":           row.get("event", "sample"),
            "joint_positions": json.dumps([round(float(v), 5)
                                           for v in row.get("joint_positions") or []]),
            "ee_position":     json.dumps([round(float(v), 5)
                                           for v in row.get("ee_position") or []]),
            "gripper_state":   row.get("gripper_state", "open"),
            "package_id":      row.get("package_id") or "",
        }

    def log(self, row):
        """One discrete event. Flushed immediately."""
        ev = row.get("event", "")
        self._ev.writerow(self._encode(row))
        self._ev_fh.flush()
        self._n_events += 1
        self._counts[ev] = self._counts.get(ev, 0) + 1
        if ev == "drop":
            self.cycles += 1

    def sample(self, row):
        """One telemetry row. Buffered."""
        self._tm.writerow(self._encode(row))
        self._n_samples += 1

    def close(self):
        for fh in (self._ev_fh, self._tm_fh):
            if fh:
                fh.close()
        print(f"\n[logger] {self._n_events} events, {self._n_samples} samples")
        print(f"[logger] completed cycles: {self.cycles}")
        for ev, n in sorted(self._counts.items()):
            print(f"           {ev:18s} {n}")

# ===========================================================================
# Plotting
# ===========================================================================
def load_run(events_csv):
    """Load a run from its events CSV; picks up telemetry and meta alongside."""
    base = events_csv[:-len("_events.csv")] if events_csv.endswith("_events.csv") \
        else os.path.splitext(events_csv)[0]

    def _read(path):
        if not os.path.exists(path):
            return []
        with open(path, newline="", encoding="utf-8") as f:
            return list(csv.DictReader(f))

    events = _read(events_csv)
    telemetry = _read(f"{base}_telemetry.csv")
    meta = {}
    if os.path.exists(f"{base}_meta.json"):
        with open(f"{base}_meta.json", encoding="utf-8") as f:
            meta = json.load(f)
    return events, telemetry, meta


def _cycle_bounds(events, cycle):
    """(t_start, t_end) of one cycle: cycle N runs up to the Nth drop."""
    drops = [float(r["timestamp_sim"]) for r in events if r["event"] == "drop"]
    if not drops:
        return None
    if cycle >= len(drops):
        raise SystemExit(f"run has {len(drops)} completed cycles; "
                         f"--cycle {cycle} is out of range")
    start = 0.0 if cycle == 0 else drops[cycle - 1]
    return start, drops[cycle]


def make_plots(events_csv, cycle=None, out_dir=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    events, telemetry, meta = load_run(events_csv)
    if not events:
        raise SystemExit(f"{events_csv} has no rows")

    if out_dir is None:
        import sim_config as C
        out_dir = C.PLOTS_DIR
    os.makedirs(out_dir, exist_ok=True)

    n_drops = sum(1 for r in events if r["event"] == "drop")
    if n_drops == 0:
        raise SystemExit("no completed cycles in this run (no drop events)")
    cycle = 0 if cycle is None else cycle
    t_lo, t_hi = _cycle_bounds(events, cycle)

    stream = telemetry or events
    if not telemetry:
        print("[plot] no telemetry file — plotting from event rows only, "
              "the traces will be sparse")

    sel = [r for r in stream if t_lo <= float(r["timestamp_sim"]) <= t_hi]
    if len(sel) < 2:
        raise SystemExit(f"cycle {cycle} has too few rows to plot")
    print(f"[plot] cycle {cycle} of {n_drops}: {len(sel)} rows, "
          f"{t_hi - t_lo:.1f} s")

    t = [float(r["timestamp_sim"]) - t_lo for r in sel]
    q = [json.loads(r["joint_positions"]) for r in sel]
    ee = [json.loads(r["ee_position"]) for r in sel]
    n_joints = min(len(v) for v in q)
    names = meta.get("joint_names") or [f"joint {i}" for i in range(n_joints)]
    names = (names + [f"joint {i}" for i in range(n_joints)])[:n_joints]

    # Strict lower bound: the previous cycle's `drop` sits exactly on t_lo and
    # would otherwise be drawn on this cycle's left edge.
    marks = [(float(r["timestamp_sim"]) - t_lo, r["event"]) for r in events
             if t_lo < float(r["timestamp_sim"]) <= t_hi
             and r["event"] in EVENT_STYLE]

    # -- Figure 1: joints, plus EE X/Y/Z as small multiples ----------------
    # X, Y and Z sit at roughly -6, 0 and 2 in this world frame, so they get
    # one panel each. Never a second y-scale on the same axes.
    fig = plt.figure(figsize=(11, 8))
    fig.patch.set_facecolor(SURFACE)
    gs = fig.add_gridspec(2, 3, height_ratios=[1.5, 1], hspace=0.32, wspace=0.28)
    ax_q = fig.add_subplot(gs[0, :])
    ax_ee = [fig.add_subplot(gs[1, i], sharex=ax_q) for i in range(3)]

    for i in range(n_joints):
        ax_q.plot(t, [v[i] for v in q], lw=1.8, solid_capstyle="round",
                  color=SERIES_COLORS[i % len(SERIES_COLORS)], label=names[i])

    for j, lbl in enumerate("XYZ"):
        ax_ee[j].plot(t, [v[j] for v in ee], lw=1.8, solid_capstyle="round",
                      color=SERIES_COLORS[j])
        ax_ee[j].set_title(f"EE {lbl} (m)", fontsize=9, color=INK_SECONDARY,
                           loc="left", pad=6)

    for ax in [ax_q] + ax_ee:
        ax.set_facecolor(SURFACE)
        ax.grid(True, color=GRIDLINE, lw=0.8)
        ax.set_axisbelow(True)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        for side in ("left", "bottom"):
            ax.spines[side].set_color("#d8d7d2")
        ax.tick_params(colors=INK_SECONDARY, labelsize=8)
        for x, ev in marks:
            ax.axvline(x, color=EVENT_STYLE[ev][0], lw=1.2,
                       ls=(0, (4, 3)), alpha=0.85, zorder=0)

    lo, hi = ax_q.get_ylim()                     # headroom for event labels
    ax_q.set_ylim(lo, hi + 0.20 * (hi - lo))
    for x, ev in marks:
        colour, lbl = EVENT_STYLE[ev]
        ax_q.annotate(lbl, (x, ax_q.get_ylim()[1]), xytext=(0, -11),
                      textcoords="offset points", ha="center", va="top",
                      fontsize=8, color=colour, weight="bold")

    ax_q.set_ylabel("joint angle (rad)", color=INK_SECONDARY, fontsize=9)
    ax_ee[0].set_ylabel("world frame (m)", color=INK_SECONDARY, fontsize=9)
    ax_ee[1].set_xlabel("simulation time since start of cycle (s)",
                        color=INK_SECONDARY, fontsize=9)
    ax_q.set_title(f"Pick-and-place cycle {cycle} — joint angles and "
                   "end-effector trajectory",
                   color=INK_PRIMARY, fontsize=12, loc="left", pad=30)
    ax_q.legend(ncol=min(n_joints, 6), fontsize=8, frameon=False,
                loc="lower left", bbox_to_anchor=(0, 1.005),
                labelcolor=INK_SECONDARY, columnspacing=1.6, handlelength=1.6)

    p1 = os.path.join(out_dir, f"cycle_{cycle}_joints_and_ee.png")
    fig.savefig(p1, dpi=160, bbox_inches="tight", facecolor=SURFACE)
    print(f"[plot] wrote {p1}")

    # -- Figure 2: the end-effector path through space ---------------------
    fig2 = plt.figure(figsize=(7.5, 6))
    fig2.patch.set_facecolor(SURFACE)
    ax3 = fig2.add_subplot(111, projection="3d")
    ax3.plot([v[0] for v in ee], [v[1] for v in ee], [v[2] for v in ee],
             lw=1.8, color=SERIES_COLORS[0], label="EE path")

    for r in events:
        ts = float(r["timestamp_sim"])
        if r["event"] in ("grasp", "drop") and t_lo <= ts <= t_hi:
            p = json.loads(r["ee_position"])
            colour, lbl = EVENT_STYLE[r["event"]]
            ax3.scatter(p[0], p[1], p[2], s=70, color=colour, edgecolor=SURFACE,
                        linewidth=1.5, depthshade=False, label=lbl)

    handles, labels = ax3.get_legend_handles_labels()
    uniq = dict(zip(labels, handles))
    ax3.legend(uniq.values(), uniq.keys(), fontsize=8, frameon=False,
               labelcolor=INK_SECONDARY)
    ax3.set_xlabel("X (m)", fontsize=9, color=INK_SECONDARY)
    ax3.set_ylabel("Y (m)", fontsize=9, color=INK_SECONDARY)
    ax3.set_zlabel("Z (m)", fontsize=9, color=INK_SECONDARY)
    ax3.set_title(f"End-effector path — cycle {cycle}",
                  color=INK_PRIMARY, fontsize=12, loc="left")

    p2 = os.path.join(out_dir, f"cycle_{cycle}_ee_path.png")
    fig2.savefig(p2, dpi=160, bbox_inches="tight", facecolor=SURFACE)
    print(f"[plot] wrote {p2}")
    return p1, p2


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser(description="Plot a logged teleoperation run.")
    ap.add_argument("events_csv", help="path to data/run_*_events.csv")
    ap.add_argument("--cycle", type=int, default=None,
                    help="which cycle to plot, 0-based (default: 0)")
    ap.add_argument("--out", default=None, help="output directory for figures")
    a = ap.parse_args()
    make_plots(a.events_csv, cycle=a.cycle, out_dir=a.out)
