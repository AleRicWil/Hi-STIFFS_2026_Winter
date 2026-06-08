"""
force_position_comparison.py

Compare force-calculation decay characteristics between two Hi-STIFFS
lab transient tests.

Selection is now performed on RAW STRAIN (Strain_C1_raw) for clear visual
identification of contact events. Force is still calculated and used for
the final aligned averaging and statistics.
"""

import csv
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import warnings

from process import HiSTIFFSData
import config

warnings.filterwarnings("ignore", category=UserWarning, module="matplotlib")


# === CONFIGURABLE CONSTANTS ================================================
SENSOR = "E"
CYCLE_DURATION = 8.0
RESAMPLE_HZ = 100.0
NOMINAL_LOAD = 50.0
FORCE_THRESH_FOR_HOLD = 45.0


def find_hold_start(t_seg, f_seg, thresh=FORCE_THRESH_FOR_HOLD):
    high_idx = np.where(f_seg > thresh)[0]
    return float(t_seg[high_idx[0]]) if len(high_idx) > 0 else float(t_seg[0])


def interactive_select_events(data, sensor_label):
    """
    Interactive GUI for selecting cycle start times using RAW STRAIN.
    """
    if sensor_label not in data.sensor_labels:
        sensor_label = data.sensor_labels[0]

    s = data.data_dict[f"Sensor_{sensor_label}"]

    # Always compute force (needed later for averaging), but do NOT use it for selection
    if "force" not in s or "time" not in s:
        data.calc_force_position(clip=False)
        s = data.data_dict[f"Sensor_{sensor_label}"]

    t = s["time"]
    # === USE RAW STRAIN FOR SELECTION (as requested) ===
    y_data = s["strain_1_raw"].astype(float)
    y_label = f"Sensor {sensor_label} Strain 1 (raw ADC)"

    fig, ax = plt.subplots(figsize=(14, 6.5))
    ax.plot(t, y_data,
            linestyle='None',
            marker='.',
            markersize=4,
            color="#1f77b4",
            alpha=0.75,
            markeredgewidth=0)

    ax.set_xlabel("Time (s)")
    ax.set_ylabel(y_label)
    ax.set_title(
        f"Interactive Cyclic Event Selection — {Path(data.data_csv_path).name}\n"
        f"LEFT-CLICK to set START of {CYCLE_DURATION:.1f}s cycle → Yellow  |  "
        f"Enter = confirm (green)  |  u = undo  |  q = finish\n"
        f"(Selection performed on raw strain — force used only for final statistics)",
        fontsize=10
    )
    ax.grid(True, alpha=0.3)

    # Reasonable y-limits for raw ADC strain
    ax.set_ylim(bottom=np.nanmin(y_data) - 50000, top=np.nanmax(y_data) + 50000)

    selected_events = []
    current_start = None
    temp_artists = []
    permanent_artists = []

    def clear_temp():
        nonlocal temp_artists
        for a in temp_artists:
            try:
                a.remove()
            except Exception:
                pass
        temp_artists = []

    def draw_temp_window(start):
        clear_temp()
        end = start + CYCLE_DURATION
        spn = ax.axvspan(start, end, alpha=0.35, color="yellow")
        temp_artists.append(spn)
        fig.canvas.draw_idle()

    def draw_permanent_window(start):
        end = start + CYCLE_DURATION
        spn = ax.axvspan(start, end, alpha=0.18, color="#2ca02c")
        permanent_artists.append(spn)
        fig.canvas.draw_idle()

    def on_click(event):
        # Respect zoom/pan toolbar state
        if fig.canvas.toolbar is not None:
            mode = fig.canvas.toolbar.mode
            if mode in ("zoom rect", "pan/zoom"):
                return

        if event.inaxes != ax or event.button != 1:
            return

        nonlocal current_start
        current_start = event.xdata
        draw_temp_window(current_start)
        print(f"  Start set at {current_start:.3f} s")

    fig.canvas.mpl_connect("button_press_event", on_click)

    # Buttons
    ax_add = fig.add_axes([0.12, 0.01, 0.13, 0.055])
    ax_undo = fig.add_axes([0.27, 0.01, 0.13, 0.055])
    ax_finish = fig.add_axes([0.42, 0.01, 0.13, 0.055])
    btn_add = Button(ax_add, "Confirm Start\n(Enter)")
    btn_undo = Button(ax_undo, "Undo Last\n(u)")
    btn_finish = Button(ax_finish, "Finish &\nSave (q)")

    def add_event(event):
        nonlocal current_start, selected_events
        if current_start is None:
            print("  No start selected. LEFT-CLICK on the plot first.")
            return
        start = current_start
        selected_events.append((start, start + CYCLE_DURATION))
        draw_permanent_window(start)
        clear_temp()
        print(f"  ✓ Event #{len(selected_events)} confirmed")
        current_start = None
        fig.canvas.draw_idle()

    def undo_last(event):
        nonlocal selected_events, permanent_artists, current_start
        if not selected_events:
            if current_start is not None:
                current_start = None
                clear_temp()
            return
        selected_events.pop()
        if permanent_artists:
            try:
                permanent_artists.pop().remove()
            except Exception:
                pass
        print(f"  ↩ Removed. {len(selected_events)} remain.")
        fig.canvas.draw_idle()

    def finish(event):
        plt.close(fig)

    btn_add.on_clicked(add_event)
    btn_undo.on_clicked(undo_last)
    btn_finish.on_clicked(finish)

    def on_key(event):
        if event.key in ("enter", "return"):
            add_event(None)
        elif event.key in ("u", "backspace"):
            undo_last(None)
        elif event.key in ("q", "escape"):
            finish(None)

    fig.canvas.mpl_connect("key_press_event", on_key)
    fig.tight_layout(rect=[0, 0.09, 1, 0.96])

    print(f"\nGUI ready. LEFT-CLICK on the raw strain plot to set the START of each {CYCLE_DURATION:.1f}s cycle.")
    plt.show(block=True)
    return selected_events


def average_aligned_events(data, events, sensor_label, max_duration=CYCLE_DURATION, resample_hz=RESAMPLE_HZ):
    """Align events using calculated force and return mean ± std force trace."""
    if not events:
        raise ValueError("No events supplied for averaging.")
    s = data.data_dict[f"Sensor_{sensor_label}"]
    t_full, f_full = s["time"], s["strain_1_raw"].astype(float)

    n_points = int(max_duration * resample_hz) + 1
    common_t = np.linspace(0.0, max_duration, n_points)
    aligned = []

    for start, end in events:
        mask = (t_full >= start) & (t_full <= end)
        t_seg, f_seg = t_full[mask], f_full[mask]
        if len(t_seg) < 5:
            continue
        hold_start = find_hold_start(t_seg, f_seg)
        t_rel = t_seg - hold_start
        valid = (t_rel >= 0) & (t_rel <= max_duration)
        t_rel, f_seg = t_rel[valid], f_seg[valid]
        if len(t_rel) < 3:
            continue
        f_interp = np.interp(common_t, t_rel, f_seg, left=np.nan, right=f_seg[-1])
        aligned.append(f_interp)

    if not aligned:
        raise RuntimeError("All selected events were rejected after alignment.")
    arr = np.vstack(aligned)
    return common_t, np.nanmean(arr, axis=0), np.nanstd(arr, axis=0, ddof=1)


def save_event_selections(events, data, tag):
    out_dir = Path(data.data_csv_path).parent
    out_path = out_dir / f"{tag}_selected_cyclic_events.csv"
    with open(out_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Event", "Start_s", "End_s", "Duration_s", "Source_CSV"])
        for i, (st, en) in enumerate(events, 1):
            w.writerow([i, f"{st:.4f}", f"{en:.4f}", f"{en - st:.4f}", data.data_csv_path.name])
    return out_path


def plot_comparison(t, mean1, std1, mean2, std2, sensor, n1, n2, nominal=NOMINAL_LOAD):
    fig, ax = plt.subplots(figsize=(13, 7.5))
    c1, c2 = "#1f77b4", "#ff7f0e"

    for mean, std, color, label in [(mean1, std1, c1, f"Dataset 1 (n={n1})"),
                                    (mean2, std2, c2, f"Dataset 2 (n={n2})")]:
        ax.fill_between(t, mean - 3 * std, mean + 3 * std, color=color, alpha=0.12, zorder=1)
        ax.fill_between(t, mean - 2 * std, mean + 2 * std, color=color, alpha=0.22, zorder=2)
        ax.fill_between(t, mean - 1 * std, mean + 1 * std, color=color, alpha=0.32, zorder=3)
        ax.plot(t, mean, color=color, lw=2.8, zorder=5, label=label)

    ax.axhline(nominal, color="#555555", ls="--", lw=1.2, alpha=0.7, zorder=4, label="Nominal load (ref)")
    ax.set_xlabel("Time since hold start (s)", fontsize=11)
    ax.set_ylabel(f"Calculated Force (N) — Sensor {sensor}", fontsize=11)
    ax.set_title(
        "Average Cyclic Loading Event & Force-Decay Variability\n"
        "Shaded: innermost ±1σ  •  middle ±2σ  •  outermost ±3σ\n"
        "(Selection performed on raw strain; statistics on calculated force)",
        fontsize=11, pad=12
    )
    ax.legend(loc="upper right", framealpha=0.95)
    ax.grid(True, alpha=0.25, zorder=0)
    ax.set_xlim(0, t[-1])

    y_min = min(np.nanmin(mean1 - 3 * std1), np.nanmin(mean2 - 3 * std2)) - 3
    y_max = max(np.nanmax(mean1 + 3 * std1), np.nanmax(mean2 + 3 * std2)) + 5
    ax.set_ylim(bottom=max(0, y_min), top=y_max)

    fig.tight_layout()
    return fig


def main():
    print("\n" + "=" * 72)
    print("Hi-STIFFS Force-Decay Comparison — Raw Strain Selection Mode")
    print("=" * 72)

    print("\nLoading Dataset 1 (2026-06-02_142659_01)...")
    data1 = HiSTIFFSData(date="2026-06-02", time="142659", nano_label="01", debug=True)

    print("Loading Dataset 2 (2026-06-01_140207_01)...")
    data2 = HiSTIFFSData(date="2026-06-01", time="140207", nano_label="01", debug=True)

    if not data1.exists or not data2.exists:
        print("ERROR: One or both datasets failed to load.")
        return

    # Ensure force is calculated (used later for averaging)
    if not data1.has_force_pos:
        data1.calc_force_position(clip=False)
    if not data2.has_force_pos:
        data2.calc_force_position(clip=False)

    print(f"\n[1/3] Selecting cycles for Dataset 1 (Sensor {SENSOR}) using RAW STRAIN")
    events1 = interactive_select_events(data1, SENSOR)
    if not events1:
        print("No events selected for Dataset 1. Exiting.")
        return
    save_event_selections(events1, data1, "dataset1")

    print(f"\n[2/3] Selecting cycles for Dataset 2 (Sensor {SENSOR}) using RAW STRAIN")
    events2 = interactive_select_events(data2, SENSOR)
    if not events2:
        print("No events selected for Dataset 2. Exiting.")
        return
    save_event_selections(events2, data2, "dataset2")

    print("\n[3/3] Computing aligned force averages and variability...")
    t1, mean1, std1 = average_aligned_events(data1, events1, SENSOR)
    t2, mean2, std2 = average_aligned_events(data2, events2, SENSOR)

    out_dir = Path("comparison_results")
    out_dir.mkdir(exist_ok=True)

    np.savez_compressed(out_dir / "dataset1_average_cyclic_event.npz",
                        time=t1, mean_force=mean1, std_force=std1, n_events=len(events1))
    np.savez_compressed(out_dir / "dataset2_average_cyclic_event.npz",
                        time=t2, mean_force=mean2, std_force=std2, n_events=len(events2))

    print("\nGenerating comparison figure...")
    fig = plot_comparison(t1, mean1, std1, mean2, std2, SENSOR, len(events1), len(events2))
    fig.savefig(out_dir / "force_decay_comparison.png", dpi=300, bbox_inches="tight")
    print(f"Figure saved → {out_dir / 'force_decay_comparison.png'}")

    print("\nDone.")
    plt.show()


if __name__ == "__main__":
    main()