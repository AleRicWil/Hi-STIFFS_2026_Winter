# Interactive stalk labeling, review, and stiffness-span refinement.
# Split out of process.py so HiSTIFFSData stays data-processing only.
#
# Public entry points
# -------------------
# interactive_detect_stalks(data, ...)
#     First-pass time-domain labeling. Writes Plot, Stalk, {A,B,C}_Start/End.
# display_stalk_selections(data)
#     View-only paging through saved rows (original + refine if present).
# refine_stalk_selections(data)
#     Pick a force-vs-probe-position sub-span inside each original bound.
#     Writes {A,B,C}_Refine_Start/End onto the same CSV (blank = unused sensor).
#
# Cross-platform: Windows 10/11, Ubuntu, Raspberry Pi 5 + touchscreen.
# matplotlib only (same stack as process.py). No extra dependencies.
#
# Stalk work is A/B/C only. D and E are not part of this UI or the CSV.

import csv

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button

from config import Config

# Display-only time alignment so A/B/C traces sit on one x-axis. CSV times
# are always the raw per-sensor clock (this shift is added back on write).
TIME_SHIFT = {'A': 0.0, 'B': 1.3, 'C': 2.6}

# Stalk labeling / refine / gather all share this set. D and E are gone.
STALK_SENSORS = ('A', 'B', 'C')

# Original bounds plus the sub-span used by the stiffness pipeline.
_BOUND_PAIRS = tuple(
    (f'{l}_Start', f'{l}_End', f'{l}_Refine_Start', f'{l}_Refine_End')
    for l in STALK_SENSORS
)
_CSV_HEADER = (
    ['Plot', 'Stalk']
    + [name for pair in _BOUND_PAIRS for name in pair]
)


def interactive_detect_stalks(parent_data, num_plots=3, stalks_per_plot=10):
    '''
    Interactive point-index labeling of stalks on sensors A, B, and C.

    Parameters
    ----------
    data : HiSTIFFSData
        Loaded run with force/position available (calc_force_position already
        called, or this function will call it).

    Change num_plots / stalks_per_plot at the call site (or the defaults
    below) before running. The session walks plots high→low and stalks
    high→low, e.g. 3 plots × 10 stalks:
        [plot 3, stalk 10], [plot 3, stalk 09], ... [plot 1, stalk 01]

    Interaction
    -----------
    * Bounds are taken in fixed order: Amin, Amax, Bmin, Bmax, Cmin, Cmax.
    * A click on that sensor's force OR position axis snaps to the nearest
      sample in 2-D display (pixel) space — not an arbitrary time value.
    * Left / Right arrows nudge the current bound by ±1 sample index.
    * Up / Down arrows move between the six bounds of the current stalk.
    * Confirm is the only way to lock a stalk and advance to the next
      [plot, stalk]. Finish writes the CSV.

    Visuals
    -------
    * No shaded SpanSelector. Samples inside each sensor's selected index
      range are recolored pure black (so A stays visible while B/C are set).
    * The active bound gets a green ring on both of that sensor's axes.
    * After Confirm, thickness=1 vertical lines are drawn at both bound
      times on that sensor's force and position axes.

    Times are stored exactly as picked. Amin may be after Amax in time;
    we never swap or reject on time order. Downstream readers that assume
    Start < End will need their own update later.

    CSV columns: Plot, Stalk, A_Start, A_End, B_Start, B_End, C_Start, C_End
    '''

    # ── Edit these before running if the call site does not pass them ──
    # Defaults match the current field layout (3 plots × 10 stalks).
    # They are parameters so a one-line call-site change is enough.
    n_plots = int(num_plots)
    n_stalks = int(stalks_per_plot)
    if n_plots < 1 or n_stalks < 1:
        print("Status: num_plots and stalks_per_plot must be >= 1")
        return

    if not parent_data.exists:
        print("Status: No data loaded - call HiSTIFFSData(...) first")
        return

    if not parent_data.has_force_pos:
        parent_data.calc_force_position()  # force/position must exist to plot

    # D/E are omitted on purpose for this labeling pass.
    sensors = [l for l in ('A', 'B', 'C') if l in parent_data.sensor_labels]
    if sensors != ['A', 'B', 'C']:
        print(f"Status: interactive_detect_stalks needs sensors A, B, and C "
              f"(found {parent_data.sensor_labels})")
        return

    # Queue: plot N..1, and within each plot stalk M..1.
    jobs = [(p, k) for p in range(n_plots, 0, -1)
            for k in range(n_stalks, 0, -1)]
    # Bound order is fixed; each entry is (sensor_label, 'min'|'max').
    steps = [(l, bound) for l in sensors for bound in ('min', 'max')]

    n_rows = len(sensors)
    fig, axs = plt.subplots(n_rows, 2, figsize=(12, 1.8 * n_rows),
                            sharex=True, squeeze=False)
    fig.suptitle("Interactive Stalk Detection", fontsize=12)

    # Per-sensor arrays actually drawn. No position masking here —
    # masking is applied upstream of this tool.
    series = {}          # label -> {t, force, pos_mm, n}
    base_lines = {}      # (label, 'force'|'pos') -> Line2D (decimated view)
    sel_scatters = {}    # (label, 'force'|'pos') -> black overlay of selected pts
    base_rgba = {}       # label -> (4,) rgba of the sensor's default color
    rings = {}           # (label, 'force'|'pos') -> overlay PathCollection

    # Draw caps — full traces are kept in `series` for snapping/CSV; only a
    # windowed, strided copy is sent to the artist. Scatter-of-all-points
    # plus per-click facecolor rebuilds is what made the UI stall.
    default_ms = 2.4          # base marker size (points), Line2D path
    selected_size = 18.0      # black overlay marker area (pt^2)
    max_base_pts = 8000       # max markers drawn per axis in the current view
    max_sel_pts = 4000        # max black overlay markers per sensor
    max_snap_pts = 20000      # max candidates for a full 2-D pixel snap

    for i, l in enumerate(sensors):
        s = parent_data.data_dict[f'Sensor_{l}']
        t = np.asarray(s['time'], dtype=np.float64) - TIME_SHIFT[l]
        force = np.asarray(s['force'], dtype=np.float64)
        pos_mm = np.asarray(s['position'], dtype=np.float64) * 1000.0
        if t.size == 0:
            print(f"Status: Sensor {l} has no samples")
            return
        series[l] = {'t': t, 'force': force, 'pos_mm': pos_mm, 'n': t.size}
        base_rgba[l] = np.asarray(plt.matplotlib.colors.to_rgba(parent_data.colors[i % len(parent_data.colors)]))

        # Line2D + rasterize is far cheaper than a PathCollection of N points.
        # Data is filled by update_base_traces() after the view is known.
        line_f, = axs[i, 0].plot([], [], linestyle='None', marker='.',
                                 markersize=default_ms, color=base_rgba[l],
                                 rasterized=True, zorder=2)
        axs[i, 0].set_ylabel(f'{l} Force (N)')
        axs[i, 0].grid(True, alpha=0.3)

        line_p, = axs[i, 1].plot([], [], linestyle='None', marker='.',
                                 markersize=default_ms, color=base_rgba[l],
                                 rasterized=True, zorder=2)
        axs[i, 1].set_ylabel(f'{l} Position (mm)')
        axs[i, 1].grid(True, alpha=0.3)

        sel_f = axs[i, 0].scatter([], [], s=selected_size, c='k',
                                  linewidths=0, zorder=4)
        sel_p = axs[i, 1].scatter([], [], s=selected_size, c='k',
                                  linewidths=0, zorder=4)

        # Green ring overlay: empty until a bound exists on this sensor.
        ring_f = axs[i, 0].scatter([], [], s=110, facecolors='none',
                                   edgecolors='lime', linewidths=1.6, zorder=5)
        ring_p = axs[i, 1].scatter([], [], s=110, facecolors='none',
                                   edgecolors='lime', linewidths=1.6, zorder=5)

        base_lines[(l, 'force')] = line_f
        base_lines[(l, 'pos')] = line_p
        sel_scatters[(l, 'force')] = sel_f
        sel_scatters[(l, 'pos')] = sel_p
        rings[(l, 'force')] = ring_f
        rings[(l, 'pos')] = ring_p

    axs[-1, 0].set_xlabel('Time (s)')
    axs[-1, 1].set_xlabel('Time (s)')
    fig.subplots_adjust(left=0.02, right=0.995, bottom=0.12, top=0.98,
                    wspace=0.12, hspace=0.08)

    # Mutable session state. Nested callbacks close over this dict so we
    # do not park transient UI state on the HiSTIFFSData instance.
    state = {
        'job_i': 0,          # index into jobs
        'step_i': 0,         # index into steps (0..5)
        # selections[job_i][(sensor, 'min'|'max')] = sample index
        'selections': [dict() for _ in jobs],
        'confirmed': set(),  # job indices locked by Confirm
        # vlines[job_i][sensor] = [4 Line2D]  (force-min, force-max, pos-min, pos-max)
        'vlines': {},
        'select_on': True,      # click-to-snap enabled; toggle to pan/zoom
        'updating_base': False, # re-entrancy guard for xlim callback
    }

    # Resume from an existing stalks CSV so one stalk can be edited
    # without re-picking the rest. Keys are (Plot, Stalk) as written.
    existing_rows = load_stalk_rows(parent_data.stalks_csv_path)
    existing_by_key = {(rec['Plot'], rec['Stalk']): rec for rec in existing_rows}
    job_index = {job: i for i, job in enumerate(jobs)}

    def _index_from_raw_time(sensor, t_raw):
        '''Map a CSV raw-clock time onto this sensor's display-shifted series.'''
        t_disp = float(t_raw) - TIME_SHIFT[sensor]
        t = series[sensor]['t']
        return int(np.argmin(np.abs(t - t_disp)))

    n_loaded = 0
    for rec in existing_rows:
        key = (rec['Plot'], rec['Stalk'])
        job_i = job_index.get(key)
        if job_i is None:
            continue
        sel = state['selections'][job_i]
        complete = True
        for l in sensors:
            t0 = rec.get(f'{l}_Start')
            t1 = rec.get(f'{l}_End')
            if t0 is None or t1 is None:
                complete = False
                continue
            sel[(l, 'min')] = _index_from_raw_time(l, t0)
            sel[(l, 'max')] = _index_from_raw_time(l, t1)
        if complete and all((l, b) in sel for l, b in steps):
            state['confirmed'].add(job_i)
            n_loaded += 1

    # Land on the first job that still needs bounds so a partial file
    # continues. If everything loaded, stay on job 0 and use Prev to
    # reach the stalk being edited.
    for i in range(len(jobs)):
        if i not in state['confirmed']:
            state['job_i'] = i
            break
    if existing_rows:
        print(f"Status: loaded {n_loaded}/{len(jobs)} confirmed stalk(s) "
              f"from {parent_data.stalks_csv_path}")

    status_text = fig.text(0.50, 0.065, '', ha='center', va='center', fontsize=9)

    def current_job():
        return jobs[state['job_i']]

    def current_step():
        return steps[state['step_i']]

    def bound_time(job_i, sensor, which):
        idx = state['selections'][job_i].get((sensor, which))
        if idx is None:
            return None
        return float(series[sensor]['t'][idx])

    def stalk_complete(job_i):
        sel = state['selections'][job_i]
        return all((l, b) in sel for l, b in steps)

    def _window_indices(t, x0, x1):
        '''Inclusive-lo / exclusive-hi sample window for a sorted time axis.'''
        i0 = int(np.searchsorted(t, x0, side='left'))
        i1 = int(np.searchsorted(t, x1, side='right'))
        if i1 <= i0:
            return 0, t.size
        return i0, i1

    def _strided(i0, i1, cap):
        n = i1 - i0
        if n <= cap:
            return np.arange(i0, i1, dtype=np.int64)
        step = int(np.ceil(n / cap))
        idx = np.arange(i0, i1, step, dtype=np.int64)
        last = np.int64(i1 - 1)
        if idx.size == 0 or idx[-1] != last:
            idx = np.concatenate((idx, [last]))
        return idx

    def update_base_traces(_ax=None):
        '''Redraw each base Line2D from the visible time window only.

        Called on xlim change and once at startup. Shared-x means one
        callback covers all six axes. Y-limits are left alone so a zoom
        in Y is not undone by a pan in X.
        '''
        if state['updating_base']:
            return
        state['updating_base'] = True
        try:
            x0, x1 = axs[0, 0].get_xlim()
            for l in sensors:
                t = series[l]['t']
                i0, i1 = _window_indices(t, x0, x1)
                idx = _strided(i0, i1, max_base_pts)
                base_lines[(l, 'force')].set_data(t[idx], series[l]['force'][idx])
                base_lines[(l, 'pos')].set_data(t[idx], series[l]['pos_mm'][idx])
        finally:
            state['updating_base'] = False

    def nearest_index(ax, t, y, event):
        '''Snap click to nearest sample in 2-D display space.

        Only the current xlim is searched (time is sorted, so this is a
        pair of searchsorted calls). If that window is still huge we snap
        by time first, then refine 2-D in a local neighborhood — same
        result when zoomed in, without transforming hundreds of thousands
        of points on every click.
        '''
        if event.xdata is None or event.ydata is None:
            return 0
        x0, x1 = ax.get_xlim()
        i0, i1 = _window_indices(t, x0, x1)
        n_win = i1 - i0
        if n_win <= 0:
            return int(np.clip(np.searchsorted(t, event.xdata), 0, t.size - 1))

        if n_win > max_snap_pts:
            center = int(np.clip(np.searchsorted(t, event.xdata), i0, i1 - 1))
            half = max_snap_pts // 2
            i0 = max(i0, center - half)
            i1 = min(i1, center + half)

        sl = slice(i0, i1)
        pts = np.column_stack((t[sl], y[sl]))
        disp = ax.transData.transform(pts)
        delta = disp - np.array([event.x, event.y], dtype=np.float64)
        return i0 + int(np.argmin(np.einsum('ij,ij->i', delta, delta)))

    def clear_vlines(job_i):
        artists = state['vlines'].pop(job_i, {})
        for group in artists.values():
            for line in group:
                try:
                    line.remove()
                except ValueError:
                    pass

    def draw_vlines(job_i):
        '''Confirmed-stalk markers: lw=1 at each bound time, both axes.'''
        clear_vlines(job_i)
        state['vlines'][job_i] = {}
        for i, l in enumerate(sensors):
            t_min = bound_time(job_i, l, 'min')
            t_max = bound_time(job_i, l, 'max')
            if t_min is None or t_max is None:
                continue
            lines = []
            for ax, t_b in ((axs[i, 0], t_min), (axs[i, 0], t_max),
                            (axs[i, 1], t_min), (axs[i, 1], t_max)):
                lines.append(ax.axvline(t_b, color='0.15', lw=1.0,
                                        ls='-', zorder=3))
            state['vlines'][job_i][l] = lines

    def _selected_indices(sensor):
        '''Unique sample indices that should render black on this sensor.'''
        parts = []
        for sel in state['selections']:
            i0 = sel.get((sensor, 'min'))
            i1 = sel.get((sensor, 'max'))
            if i0 is not None and i1 is not None:
                lo, hi = (i0, i1) if i0 <= i1 else (i1, i0)
                parts.append((lo, hi + 1))
            else:
                if i0 is not None:
                    parts.append((i0, i0 + 1))
                if i1 is not None:
                    parts.append((i1, i1 + 1))
        if not parts:
            return np.empty(0, dtype=np.int64)
        n = series[sensor]['n']
        span = sum(hi - lo for lo, hi in parts)
        # Boolean merge when ranges may overlap or cover most of the trace.
        if span > n or len(parts) > 1:
            mask = np.zeros(n, dtype=bool)
            for lo, hi in parts:
                mask[lo:hi] = True
            return np.flatnonzero(mask)
        lo, hi = parts[0]
        return np.arange(lo, hi, dtype=np.int64)

    def refresh_selection_overlays():
        '''Paint selected samples as a small black overlay — not a restyle
        of the full base PathCollection. Cap marker count so a wide
        accidental range cannot freeze the canvas.
        '''
        empty = np.empty((0, 2))
        for l in sensors:
            idx = _selected_indices(l)
            if idx.size > max_sel_pts:
                step = int(np.ceil(idx.size / max_sel_pts))
                idx = np.unique(np.concatenate((idx[::step], idx[[0, -1]])))
            if idx.size == 0:
                offs_f = empty
                offs_p = empty
            else:
                offs_f = np.column_stack((series[l]['t'][idx], series[l]['force'][idx]))
                offs_p = np.column_stack((series[l]['t'][idx], series[l]['pos_mm'][idx]))
            sel_scatters[(l, 'force')].set_offsets(offs_f)
            sel_scatters[(l, 'pos')].set_offsets(offs_p)

    def refresh_rings():
        '''Green ring on the active bound, both axes of that sensor.'''
        for l in sensors:
            for kind in ('force', 'pos'):
                rings[(l, kind)].set_offsets(np.empty((0, 2)))

        l, which = current_step()
        idx = state['selections'][state['job_i']].get((l, which))
        if idx is None:
            return
        t = series[l]['t'][idx]
        rings[(l, 'force')].set_offsets([[t, series[l]['force'][idx]]])
        rings[(l, 'pos')].set_offsets([[t, series[l]['pos_mm'][idx]]])

    def update_title():
        plot, stalk = current_job()
        l, which = current_step()
        idx = state['selections'][state['job_i']].get((l, which))
        if idx is None:
            pick = 'click force or position'
        else:
            pick = f'idx={idx}  t={series[l]["t"][idx]:.3f}s'
        done = 'confirmed' if state['job_i'] in state['confirmed'] else 'editing'
        mode = 'SELECT' if state['select_on'] else 'NAVIGATE'
        fig.suptitle(
            f"Interactive Stalk Detection  [{plot=}, stalk={stalk:02d}]  "
            f"{l}{which}  ({done})  [{mode}]",
            fontsize=11,
        )
        status_text.set_text(
            f"{l} {which}  {pick}   • click {l} axes to set   "
            f"• ←/→ sample   • ↑/↓ bound   • space = select on/off   • q = finish"
        )

    def redraw(selection=True):
        if selection:
            refresh_selection_overlays()
        refresh_rings()
        update_title()
        fig.canvas.draw_idle()

    def set_bound(idx):
        '''Write the current bound, clamp to the sensor's sample count.'''
        l, which = current_step()
        n = series[l]['n']
        idx = int(max(0, min(n - 1, idx)))
        state['selections'][state['job_i']][(l, which)] = idx
        # Editing a locked stalk invalidates Confirm until they re-lock.
        if state['job_i'] in state['confirmed']:
            state['confirmed'].discard(state['job_i'])
            clear_vlines(state['job_i'])

    def on_click(event):
        if event.inaxes is None or event.button != 1:
            return
        # Off: clicks pass through so the toolbar can pan/zoom.
        if not state['select_on']:
            return
        # Toolbar pan/zoom holds widgetlock and sets toolbar.mode.
        tb = getattr(fig.canvas, 'toolbar', None)
        if tb is not None and getattr(tb, 'mode', ''):
            return
        if fig.canvas.widgetlock.locked():
            return
        # Ignore clicks on the control buttons (they live on their own axes).
        l, _which = current_step()
        row = sensors.index(l)
        target_axes = {axs[row, 0], axs[row, 1]}
        if event.inaxes not in target_axes:
            return

        if event.inaxes is axs[row, 0]:
            y = series[l]['force']
        else:
            y = series[l]['pos_mm']
        idx = nearest_index(event.inaxes, series[l]['t'], y, event)
        set_bound(idx)
        # Fast path: six clicks fill Amin..Cmax. Arrows still edit the
        # bound we just left if the user steps back with Up.
        if state['step_i'] < len(steps) - 1:
            state['step_i'] += 1
        redraw()

    def nudge(delta):
        l, which = current_step()
        cur = state['selections'][state['job_i']].get((l, which))
        if cur is None:
            # No pick yet: plant a bound at the first sample, then apply delta.
            cur = 0
        set_bound(cur + delta)
        redraw()

    def step_bound(delta):
        state['step_i'] = int(max(0, min(len(steps) - 1, state['step_i'] + delta)))
        redraw(selection=False)  # bound cursor only; overlay unchanged

    def undo_bound(event=None):
        '''Clear the current bound, or step back one bound if it is empty.'''
        sel = state['selections'][state['job_i']]
        key = current_step()
        if key in sel:
            sel.pop(key)
            if state['job_i'] in state['confirmed']:
                state['confirmed'].discard(state['job_i'])
                clear_vlines(state['job_i'])
        elif state['step_i'] > 0:
            state['step_i'] -= 1
            sel.pop(current_step(), None)
            if state['job_i'] in state['confirmed']:
                state['confirmed'].discard(state['job_i'])
                clear_vlines(state['job_i'])
        redraw()

    def prev_stalk(event=None):
        '''Revisit the previous [plot, stalk]. Does not confirm anything.'''
        if state['job_i'] <= 0:
            print("Already on the first [plot, stalk]")
            return
        state['job_i'] -= 1
        state['step_i'] = 0
        redraw()

    def confirm_stalk(event=None):
        '''Lock the current stalk (all six bounds required) and advance.

        This is the only forward move through the [plot, stalk] queue.
        '''
        job_i = state['job_i']
        plot, stalk = jobs[job_i]
        if not stalk_complete(job_i):
            missing = [f"{l}{w}" for l, w in steps
                       if (l, w) not in state['selections'][job_i]]
            print(f"[plot {plot}, stalk {stalk:02d}] incomplete — missing {missing}")
            return

        state['confirmed'].add(job_i)
        draw_vlines(job_i)
        print(f"Confirmed [plot {plot}, stalk {stalk:02d}] "
              f"({len(state['confirmed'])}/{len(jobs)})")

        if job_i < len(jobs) - 1:
            state['job_i'] = job_i + 1
            state['step_i'] = 0
        else:
            print("Last [plot, stalk] confirmed — Finish & Save when ready")
        redraw()

    def finish(event=None):
        '''Write every complete job, keep incomplete jobs at their CSV
        values if we loaded them, and keep refine columns already on disk.

        A stalk the user edited but did not re-Confirm is still written
        from the in-memory picks as long as all six bounds are present.
        '''
        out = []
        written = set()
        n_from_session = 0
        for job_i, (plot, stalk) in enumerate(jobs):
            key = (plot, stalk)
            if stalk_complete(job_i):
                rec = dict(existing_by_key.get(key, {'Plot': plot, 'Stalk': stalk}))
                rec['Plot'] = plot
                rec['Stalk'] = stalk
                for l in sensors:
                    rec[f'{l}_Start'] = bound_time(job_i, l, 'min') + TIME_SHIFT[l]
                    rec[f'{l}_End'] = bound_time(job_i, l, 'max') + TIME_SHIFT[l]
                    rec.setdefault(f'{l}_Refine_Start', None)
                    rec.setdefault(f'{l}_Refine_End', None)
                out.append(rec)
                written.add(key)
                n_from_session += 1
            elif key in existing_by_key:
                out.append(dict(existing_by_key[key]))
                written.add(key)

        for rec in existing_rows:
            key = (rec['Plot'], rec['Stalk'])
            if key not in written:
                out.append(dict(rec))

        if not out:
            print("Status: No stalks to write")
            plt.close(fig)
            return

        write_stalk_rows(parent_data.stalks_csv_path, out)
        print(f"Status: wrote {len(out)} row(s) "
              f"({n_from_session} from this session) to "
              f"{parent_data.stalks_csv_path}")
        plt.close(fig)

    def toggle_select(event=None):
        '''Flip click-to-snap so the same left button can pan/zoom.'''
        state['select_on'] = not state['select_on']
        on = state['select_on']
        btn_select.label.set_text('Select: ON' if on else 'Select: OFF')
        btn_select.color = 'lightgreen' if on else '0.82'
        # Button.color is used on the next hover/leave; force a face update.
        btn_select.ax.set_facecolor(btn_select.color)
        update_title()
        fig.canvas.draw_idle()

    # Touch-friendly controls for Win / Ubuntu / RPi5 + external keyboard.
    ax_prev    = fig.add_axes([0.03, 0.012, 0.11, 0.045])
    ax_undo    = fig.add_axes([0.15, 0.012, 0.11, 0.045])
    ax_confirm = fig.add_axes([0.27, 0.012, 0.13, 0.045])
    ax_finish  = fig.add_axes([0.41, 0.012, 0.13, 0.045])
    ax_select  = fig.add_axes([0.55, 0.012, 0.13, 0.045])

    btn_prev    = Button(ax_prev,    '← Prev Stalk')
    btn_undo    = Button(ax_undo,    'Undo Bound')
    btn_confirm = Button(ax_confirm, 'Confirm Stalk')
    btn_finish  = Button(ax_finish,  'Finish & Save')
    btn_select  = Button(ax_select,  'Select: ON', color='lightgreen')

    btn_prev.on_clicked(prev_stalk)
    btn_undo.on_clicked(undo_bound)
    btn_confirm.on_clicked(confirm_stalk)
    btn_finish.on_clicked(finish)
    btn_select.on_clicked(toggle_select)

    def on_key(event):
        key = event.key
        if key == 'left':
            nudge(-1)
        elif key == 'right':
            nudge(+1)
        elif key == 'up':
            step_bound(-1)
        elif key == 'down':
            step_bound(+1)
        elif key in ('backspace', 'delete'):
            undo_bound()
        elif key == 'enter':
            # Same action as the Confirm button — not a second advance path.
            confirm_stalk()
        elif key in ('p', 'P'):
            prev_stalk()
        elif key in ('q', 'Q'):
            finish()
        elif key in (' ', 'm', 'M'):
            toggle_select()

    # Matplotlib's figure-manager keymap is connected by default and
    # intercepts the same keys we use: left/right = view history, p = pan,
    # q = quit, s = save, g = grid, h/r = home. Disconnect it so only
    # on_key() handles the keyboard. Mouse toolbar (zoom/pan icons) still
    # works. Figure-local — does not change rcParams for later plots.
    mgr = fig.canvas.manager
    if mgr is not None:
        for attr in ('key_press_handler_id', 'key_release_handler_id'):
            cid = getattr(mgr, attr, None)
            if cid is not None:
                fig.canvas.mpl_disconnect(cid)
                setattr(mgr, attr, None)

    fig.canvas.mpl_connect('button_press_event', on_click)
    fig.canvas.mpl_connect('key_press_event', on_key)

    # Fit x to the data, then fill the decimated Line2Ds. sharex means one
    # xlim_changed hook keeps all six axes in sync while panning/zooming.
    t_lo = min(float(series[l]['t'][0]) for l in sensors)
    t_hi = max(float(series[l]['t'][-1]) for l in sensors)
    axs[0, 0].set_xlim(t_lo, t_hi)
    update_base_traces()
    for i, l in enumerate(sensors):
        axs[i, 0].relim()
        axs[i, 0].autoscale(axis='y')
        axs[i, 1].relim()
        axs[i, 1].autoscale(axis='y')
    axs[0, 0].callbacks.connect('xlim_changed', update_base_traces)

    for job_i in sorted(state['confirmed']):
        draw_vlines(job_i)
    redraw()
    plt.show(block=True)


# =============================================================================
# CSV helpers shared by review / refine and by
# HiSTIFFSData.gather_stalk_traces() (refine time bounds only).
# =============================================================================

def _to_float_or_none(value):
    '''Blank CSV cells and non-numeric junk become None, not 0.0.'''
    if value is None:
        return None
    if isinstance(value, float) and np.isnan(value):
        return None
    text = str(value).strip()
    if text == '':
        return None
    try:
        return float(text)
    except (TypeError, ValueError):
        return None


def _to_int_or_none(value):
    number = _to_float_or_none(value)
    if number is None:
        return None
    return int(number)


def load_stalk_rows(csv_path):
    '''Read the stalks CSV written by interactive_detect_stalks / refine.

    Returns a list of dicts in file order (same high→low plot/stalk order
    the labeling UI walks, provided the user confirmed in order). Missing
    refine columns (older files) come back as None.
    '''
    csv_path = csv_path
    if not csv_path or not hasattr(csv_path, 'exists'):
        # Accept raw strings as well as pathlib.Path.
        from pathlib import Path
        csv_path = Path(csv_path)
    if not csv_path.exists():
        return []

    with open(csv_path, 'r', newline='') as f:
        rows = list(csv.reader(f))

    data_idx = None
    for i, row in enumerate(rows):
        if len(row) == 1 and row[0].strip() == Config.STALK_TIMES_MARKER:
            data_idx = i
            break
    if data_idx is None or data_idx + 1 >= len(rows):
        print(f"Status: no {Config.STALK_TIMES_MARKER} table in {csv_path}")
        return []

    header = [h.strip() for h in rows[data_idx + 1]]
    col = {name: j for j, name in enumerate(header)}
    if 'Plot' not in col or 'Stalk' not in col:
        print(f"Status: stalks CSV is missing Plot/Stalk columns: {csv_path}")
        return []

    out = []
    for raw in rows[data_idx + 2:]:
        if not raw or all(not str(c).strip() for c in raw):
            continue

        def cell(name):
            j = col.get(name)
            if j is None or j >= len(raw):
                return None
            return raw[j]

        record = {
            'Plot': _to_int_or_none(cell('Plot')),
            'Stalk': _to_int_or_none(cell('Stalk')),
        }
        for start_n, end_n, r_start_n, r_end_n in _BOUND_PAIRS:
            record[start_n] = _to_float_or_none(cell(start_n))
            record[end_n] = _to_float_or_none(cell(end_n))
            record[r_start_n] = _to_float_or_none(cell(r_start_n))
            record[r_end_n] = _to_float_or_none(cell(r_end_n))
        if record['Plot'] is None or record['Stalk'] is None:
            continue
        out.append(record)
    return out


def write_stalk_rows(csv_path, records):
    '''Rewrite the stalks CSV, always emitting refine columns.

    Original Start/End are preserved exactly (3-decimal strings rebuilt
    from the float we loaded). Empty sensors stay blank cells.
    '''
    def fmt(value):
        return '' if value is None else f'{float(value):.3f}'

    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([Config.STALK_TIMES_MARKER])
        writer.writerow(_CSV_HEADER)
        for rec in records:
            row = [rec['Plot'], rec['Stalk']]
            for start_n, end_n, r_start_n, r_end_n in _BOUND_PAIRS:
                row.extend([
                    fmt(rec.get(start_n)),
                    fmt(rec.get(end_n)),
                    fmt(rec.get(r_start_n)),
                    fmt(rec.get(r_end_n)),
                ])
            writer.writerow(row)


def _pair_ok(t0, t1):
    return t0 is not None and t1 is not None


def _time_window(t0, t1):
    '''Inclusive raw-time window. Start may be after End; we do not swap
    on disk, but a window always runs lo → hi so masks are non-empty.'''
    return (float(t0), float(t1)) if t0 <= t1 else (float(t1), float(t0))


def refine_time_window(rec, sensor_label):
    '''(lo, hi) from {L}_Refine_Start/End, or None if that pair is blank.

    Stiffness gather requires refine cells. Original Start/End are not
    a fallback. Shared so the pipeline and this module read the same
    columns the same way.
    '''
    t0 = rec.get(f'{sensor_label}_Refine_Start')
    t1 = rec.get(f'{sensor_label}_Refine_End')
    if not _pair_ok(t0, t1):
        return None
    return _time_window(t0, t1)


def inclusive_time_mask(t, t0, t1):
    '''Boolean mask: samples with t in [min(t0,t1), max(t0,t1)] inclusive.

    Time array only — no force or position filtering. Missing or
    non-finite bounds yield all-False. Shared by the refine UI and
    HiSTIFFSData.read_stalk_on_sensor.
    '''
    t = np.asarray(t)
    empty = np.zeros(t.shape, dtype=bool)
    if t0 is None or t1 is None:
        return empty
    try:
        a = float(t0)
        b = float(t1)
    except (TypeError, ValueError):
        return empty
    if not (np.isfinite(a) and np.isfinite(b)):
        return empty
    lo, hi = (a, b) if a <= b else (b, a)
    return (t >= lo) & (t <= hi)


def _nearest_index_in_segment(t_raw, t_target):
    '''Map a saved raw time back onto the samples that make this segment.'''
    if t_raw.size == 0:
        return None
    idx = int(np.argmin(np.abs(t_raw - float(t_target))))
    return idx


# =============================================================================
# Review / refine UI
# =============================================================================

def display_stalk_selections(parent_data):
    '''Page through saved stalk rows. No edits, no CSV write.

    Figure 1 (context): full A/B/C force and position vs shifted time,
    original span highlighted, refine span overlaid when present.
    Figure 2 (stalk): force vs probe position for this row only.
    Walks CSV order — the same high→low [plot, stalk] order as labeling.
    '''
    _review_stalks(parent_data, refine=False)


def refine_stalk_selections(parent_data):
    '''Page through saved stalk rows and shrink each sensor to the
    force-vs-probe-position sub-span that stiffness should use.

    Interaction (refine figure; context figure is navigation-only)
    --------------------------------------------------------------
    * Bounds: Amin, Amax, Bmin, Bmax, Cmin, Cmax — skip a sensor that
      has no original span or that you mark empty.
    * Click the current sensor's F–P axes to snap to the nearest sample
      in display space. The pick is clamped to the original time span.
    * ← / → nudge the current bound by one sample. This is the primary
      edit path; clicks are there for a coarse plant.
    * ↑ / ↓ move between bounds that still exist for this stalk.
    * Skip Sensor leaves that sensor's refine cells blank (unused).
    * Confirm locks the row and advances. Finish writes the CSV.

    Default refine span is the full original span so Confirm without
    edits still produces a usable sub-span (the original window).
    '''
    _review_stalks(parent_data, refine=True)


def _review_stalks(parent_data, refine):
    if not parent_data.exists:
        print("Status: No data loaded - call HiSTIFFSData(...) first")
        return

    if not parent_data.has_force_pos:
        parent_data.calc_force_position()

    sensors = [l for l in STALK_SENSORS if l in parent_data.sensor_labels]
    if sensors != list(STALK_SENSORS):
        print(f"Status: stalk review needs sensors A, B, and C "
              f"(found {parent_data.sensor_labels})")
        return

    records = load_stalk_rows(parent_data.stalks_csv_path)
    if not records:
        print(f"Status: no stalk rows in {parent_data.stalks_csv_path}")
        return

    # ------------------------------------------------------------------
    # Full-run series for the context figure (decimated like the labeler).
    # ------------------------------------------------------------------
    full = {}
    for i, l in enumerate(sensors):
        s = parent_data.data_dict[f'Sensor_{l}']
        t_raw = np.asarray(s['time'], dtype=np.float64)
        full[l] = {
            't_raw': t_raw,
            't_disp': t_raw - TIME_SHIFT[l],
            'force': np.asarray(s['force'], dtype=np.float64),
            'pos_mm': np.asarray(s['position'], dtype=np.float64) * 1000.0,
            'n': t_raw.size,
            'rgba': np.asarray(
                plt.matplotlib.colors.to_rgba(
                    parent_data.colors[i % len(parent_data.colors)]
                )
            ),
        }
        if full[l]['n'] == 0:
            print(f"Status: Sensor {l} has no samples")
            return

    # ------------------------------------------------------------------
    # Per-row, per-sensor original segments. Refine indices live in this
    # segment's sample space so ±1 arrow steps cannot leave the original
    # bound. Time window only (inclusive_time_mask). Probe position:
    #     (length - sensor_position) + start_pos
    # ------------------------------------------------------------------
    segments = []
    for rec in records:
        per_sensor = {}
        for l in sensors:
            t0, t1 = rec.get(f'{l}_Start'), rec.get(f'{l}_End')
            if not _pair_ok(t0, t1):
                per_sensor[l] = None
                continue
            lo, hi = _time_window(t0, t1)
            t_raw = full[l]['t_raw']
            # Inclusive on both ends: the labeler stores the clicked sample
            # times, and Start may be after End.
            mask = inclusive_time_mask(t_raw, lo, hi)
            idx = np.flatnonzero(mask)
            if idx.size == 0:
                per_sensor[l] = None
                continue
            s = parent_data.data_dict[f'Sensor_{l}']
            pos_m = np.asarray(s['position'], dtype=np.float64)[idx]
            per_sensor[l] = {
                'full_idx': idx,
                't_raw': t_raw[idx],
                't_disp': full[l]['t_disp'][idx],
                'force': full[l]['force'][idx],
                'pos_m': pos_m,
                'probe_m': (s['length'] - pos_m) + s['start_pos'],
                'n': int(idx.size),
            }
        segments.append(per_sensor)

    def default_refine_state(job_i):
        '''Full original span, or skipped when that sensor has no samples.'''
        sel = {}
        skipped = set()
        rec = records[job_i]
        for l in sensors:
            seg = segments[job_i][l]
            if seg is None:
                skipped.add(l)
                continue
            i0 = 0
            i1 = seg['n'] - 1
            # Resume a previous refine session if both cells are present
            # and land inside this segment.
            r0 = rec.get(f'{l}_Refine_Start')
            r1 = rec.get(f'{l}_Refine_End')
            if _pair_ok(r0, r1):
                j0 = _nearest_index_in_segment(seg['t_raw'], r0)
                j1 = _nearest_index_in_segment(seg['t_raw'], r1)
                if j0 is not None and j1 is not None:
                    i0, i1 = j0, j1
            elif rec.get(f'{l}_Refine_Start') is None and rec.get(f'{l}_Refine_End') is None:
                # Distinguish "never refined" (use full span) from "user
                # saved an empty sensor". A previous Skip writes blanks
                # for refine AND we persist that by treating a row that
                # already has *any* refine cell filled on another sensor
                # plus blanks here as skipped. If the whole row has no
                # refine cells at all, every present sensor starts full.
                pass
            sel[(l, 'min')] = i0
            sel[(l, 'max')] = i1
        return sel, skipped

    def row_was_previously_refined(rec):
        return any(
            rec.get(f'{l}_Refine_Start') is not None
            or rec.get(f'{l}_Refine_End') is not None
            for l in sensors
        )

    # Selections for every row up front so paging back shows the same picks.
    selections = []
    skipped_by_row = []
    previously_refined = set()
    for job_i, rec in enumerate(records):
        sel, skipped = default_refine_state(job_i)
        # If this row was saved with refine blanks on a sensor that *has*
        # an original span, that was an explicit Skip — keep it skipped.
        if row_was_previously_refined(rec):
            previously_refined.add(job_i)
            for l in sensors:
                if segments[job_i][l] is None:
                    continue
                if not _pair_ok(rec.get(f'{l}_Refine_Start'), rec.get(f'{l}_Refine_End')):
                    skipped.add(l)
                    sel.pop((l, 'min'), None)
                    sel.pop((l, 'max'), None)
        selections.append(sel)
        skipped_by_row.append(skipped)

    def steps_for(job_i):
        '''Bound order Amin…Cmax, omitting sensors with no original span.'''
        out = []
        for l in sensors:
            if segments[job_i][l] is None:
                continue
            out.append((l, 'min'))
            out.append((l, 'max'))
        return out

    # ------------------------------------------------------------------
    # Context figure — full traces, highlights update per row.
    # ------------------------------------------------------------------
    ctx_fig, ctx_axs = plt.subplots(
        len(sensors), 2,
        figsize=(12, 1.7 * len(sensors)),
        sharex=True, squeeze=False,
    )
    ctx_fig.suptitle("Stalk context (full traces)", fontsize=12)

    default_ms = 2.4
    selected_size = 18.0
    max_base_pts = 8000
    max_sel_pts = 4000
    max_snap_pts = 20000

    ctx_base = {}
    ctx_orig = {}
    ctx_ref = {}
    ctx_rings = {}
    ctx_vlines = {l: [] for l in sensors}

    for i, l in enumerate(sensors):
        rgba = full[l]['rgba']
        line_f, = ctx_axs[i, 0].plot(
            [], [], linestyle='None', marker='.', markersize=default_ms,
            color=rgba, rasterized=True, zorder=2,
        )
        line_p, = ctx_axs[i, 1].plot(
            [], [], linestyle='None', marker='.', markersize=default_ms,
            color=rgba, rasterized=True, zorder=2,
        )
        ctx_axs[i, 0].set_ylabel(f'{l} Force (N)')
        ctx_axs[i, 1].set_ylabel(f'{l} Position (mm)')
        ctx_axs[i, 0].grid(True, alpha=0.3)
        ctx_axs[i, 1].grid(True, alpha=0.3)
        ctx_base[(l, 'force')] = line_f
        ctx_base[(l, 'pos')] = line_p

        orig_f = ctx_axs[i, 0].scatter([], [], s=10, c=[rgba], linewidths=0, zorder=3)
        orig_p = ctx_axs[i, 1].scatter([], [], s=10, c=[rgba], linewidths=0, zorder=3)
        ref_f = ctx_axs[i, 0].scatter([], [], s=selected_size, c='k', linewidths=0, zorder=4)
        ref_p = ctx_axs[i, 1].scatter([], [], s=selected_size, c='k', linewidths=0, zorder=4)
        ring_f = ctx_axs[i, 0].scatter(
            [], [], s=110, facecolors='none', edgecolors='lime',
            linewidths=1.6, zorder=5,
        )
        ring_p = ctx_axs[i, 1].scatter(
            [], [], s=110, facecolors='none', edgecolors='lime',
            linewidths=1.6, zorder=5,
        )
        ctx_orig[(l, 'force')] = orig_f
        ctx_orig[(l, 'pos')] = orig_p
        ctx_ref[(l, 'force')] = ref_f
        ctx_ref[(l, 'pos')] = ref_p
        ctx_rings[(l, 'force')] = ring_f
        ctx_rings[(l, 'pos')] = ring_p

    ctx_axs[-1, 0].set_xlabel('Time (s, display-shifted)')
    ctx_axs[-1, 1].set_xlabel('Time (s, display-shifted)')
    ctx_fig.subplots_adjust(left=0.06, right=0.995, bottom=0.10, top=0.92,
                            wspace=0.12, hspace=0.08)

    # ------------------------------------------------------------------
    # Stalk figure — ONE force vs probe-position axes. Three independent
    # scatters (A/B/C), each colored by that sensor's own raw time.
    # Same marker for every sensor; no colorbar.
    # ------------------------------------------------------------------
    fp_fig, fp_ax = plt.subplots(figsize=(11, 6.2))
    fp_fig.subplots_adjust(left=0.08, right=0.99, bottom=0.16, top=0.90)

    fp_base = {}
    fp_sel = {}
    fp_rings = {}
    for i, l in enumerate(sensors):
        rgba = full[l]['rgba']
        base = fp_ax.scatter(
            [], [], s=30, c=[], cmap='viridis', marker='o',
            linewidths=0, rasterized=True, zorder=2,
        )
        sel = fp_ax.scatter(
            [], [], s=5, c='k', marker='o',
            linewidths=0, zorder=4,
        )
        ring = fp_ax.scatter(
            [], [], s=140, facecolors='none', edgecolors='lime',
            marker='o', linewidths=1.8, zorder=5,
        )
        start_pos = parent_data.data_dict[f'Sensor_{l}']['start_pos']
        fp_ax.axvline(start_pos, color=rgba, linewidth=0.6, alpha=0.5, zorder=1)
        fp_base[l] = base
        fp_sel[l] = sel
        fp_rings[l] = ring
    fp_ax.set_xlabel('Probe position (m)')
    fp_ax.set_ylabel('Force (N)')
    fp_ax.grid(True, alpha=0.3)

    status_text = fp_fig.text(0.50, 0.072, '', ha='center', va='center', fontsize=9)

    state = {
        'job_i': 0,
        'step_i': 0,
        'select_on': bool(refine),
        'updating_ctx': False,
        'confirmed': set(previously_refined) if refine else set(range(len(records))),
    }

    def current_steps():
        return steps_for(state['job_i'])

    def current_step():
        steps = current_steps()
        if not steps:
            return None
        state['step_i'] = int(max(0, min(len(steps) - 1, state['step_i'])))
        return steps[state['step_i']]

    def _window_indices(t, x0, x1):
        i0 = int(np.searchsorted(t, x0, side='left'))
        i1 = int(np.searchsorted(t, x1, side='right'))
        if i1 <= i0:
            return 0, t.size
        return i0, i1

    def _strided(i0, i1, cap):
        n = i1 - i0
        if n <= cap:
            return np.arange(i0, i1, dtype=np.int64)
        step = int(np.ceil(n / cap))
        idx = np.arange(i0, i1, step, dtype=np.int64)
        last = np.int64(i1 - 1)
        if idx.size == 0 or idx[-1] != last:
            idx = np.concatenate((idx, [last]))
        return idx

    def update_context_base(_ax=None):
        if state['updating_ctx']:
            return
        state['updating_ctx'] = True
        try:
            x0, x1 = ctx_axs[0, 0].get_xlim()
            for l in sensors:
                t = full[l]['t_disp']
                i0, i1 = _window_indices(t, x0, x1)
                idx = _strided(i0, i1, max_base_pts)
                ctx_base[(l, 'force')].set_data(t[idx], full[l]['force'][idx])
                ctx_base[(l, 'pos')].set_data(t[idx], full[l]['pos_mm'][idx])
        finally:
            state['updating_ctx'] = False

    def _cap_idx(idx, cap):
        if idx.size <= cap:
            return idx
        step = int(np.ceil(idx.size / cap))
        return np.unique(np.concatenate((idx[::step], idx[[0, -1]])))

    def _seg_slice(sel, l):
        i0 = sel.get((l, 'min'))
        i1 = sel.get((l, 'max'))
        if i0 is None or i1 is None:
            return None
        lo, hi = (i0, i1) if i0 <= i1 else (i1, i0)
        return lo, hi

    def refresh_context_overlays():
        empty = np.empty((0, 2))
        job_i = state['job_i']
        sel = selections[job_i]
        skipped = skipped_by_row[job_i]

        for l in sensors:
            for group in ctx_vlines[l]:
                for line in group:
                    try:
                        line.remove()
                    except ValueError:
                        pass
            ctx_vlines[l] = []

            rec = records[job_i]
            t0, t1 = rec.get(f'{l}_Start'), rec.get(f'{l}_End')
            if _pair_ok(t0, t1):
                for t_b in (t0, t1):
                    t_disp = float(t_b) - TIME_SHIFT[l]
                    lines = [
                        ctx_axs[sensors.index(l), 0].axvline(
                            t_disp, color='0.45', lw=1.0, ls='--', zorder=3,
                        ),
                        ctx_axs[sensors.index(l), 1].axvline(
                            t_disp, color='0.45', lw=1.0, ls='--', zorder=3,
                        ),
                    ]
                    ctx_vlines[l].append(lines)

            seg = segments[job_i][l]
            if seg is None:
                ctx_orig[(l, 'force')].set_offsets(empty)
                ctx_orig[(l, 'pos')].set_offsets(empty)
                ctx_ref[(l, 'force')].set_offsets(empty)
                ctx_ref[(l, 'pos')].set_offsets(empty)
                continue

            orig_idx = _cap_idx(np.arange(seg['n'], dtype=np.int64), max_sel_pts)
            ctx_orig[(l, 'force')].set_offsets(
                np.column_stack((seg['t_disp'][orig_idx],
                                 full[l]['force'][seg['full_idx'][orig_idx]]))
            )
            ctx_orig[(l, 'pos')].set_offsets(
                np.column_stack((seg['t_disp'][orig_idx],
                                 full[l]['pos_mm'][seg['full_idx'][orig_idx]]))
            )

            sl = _seg_slice(sel, l)
            if l in skipped or sl is None:
                ctx_ref[(l, 'force')].set_offsets(empty)
                ctx_ref[(l, 'pos')].set_offsets(empty)
            else:
                lo, hi = sl
                ref_idx = _cap_idx(np.arange(lo, hi + 1, dtype=np.int64), max_sel_pts)
                ctx_ref[(l, 'force')].set_offsets(
                    np.column_stack((seg['t_disp'][ref_idx],
                                     full[l]['force'][seg['full_idx'][ref_idx]]))
                )
                ctx_ref[(l, 'pos')].set_offsets(
                    np.column_stack((seg['t_disp'][ref_idx],
                                     full[l]['pos_mm'][seg['full_idx'][ref_idx]]))
                )

    def frame_context_on_stalk():
        '''Zoom the context x-axis to this row's original spans + pad.'''
        rec = records[state['job_i']]
        times = []
        for l in sensors:
            t0, t1 = rec.get(f'{l}_Start'), rec.get(f'{l}_End')
            if _pair_ok(t0, t1):
                times.extend([t0 - TIME_SHIFT[l], t1 - TIME_SHIFT[l]])
        if not times:
            t_lo = min(float(full[l]['t_disp'][0]) for l in sensors)
            t_hi = max(float(full[l]['t_disp'][-1]) for l in sensors)
        else:
            t_lo, t_hi = min(times), max(times)
            pad = max(0.4, 0.15 * (t_hi - t_lo))
            t_lo -= pad
            t_hi += pad
        ctx_axs[0, 0].set_xlim(t_lo, t_hi)
        update_context_base()

    def refresh_fp_plot(reset_view=True):
        empty = np.empty((0, 2))
        job_i = state['job_i']
        sel = selections[job_i]
        skipped = skipped_by_row[job_i]
        step = current_step()

        for l in sensors:
            seg = segments[job_i][l]
            fp_rings[l].set_offsets(empty)
            if seg is None:
                fp_base[l].set_offsets(empty)
                fp_base[l].set_array(np.empty(0))
                fp_sel[l].set_offsets(empty)
                continue
            fp_base[l].set_offsets(
                np.column_stack((seg['probe_m'], seg['force']))
            )
            t = np.asarray(seg['t_raw'], dtype=np.float64)
            fp_base[l].set_array(t)
            if t.size:
                t_lo = float(np.nanmin(t))
                t_hi = float(np.nanmax(t))
                if t_hi <= t_lo:
                    t_hi = t_lo + 1e-3
                fp_base[l].set_clim(t_lo, t_hi)
            sl = _seg_slice(sel, l)
            if l in skipped or sl is None:
                fp_sel[l].set_offsets(empty)
            else:
                lo, hi = sl
                idx = np.arange(lo, hi + 1, dtype=np.int64)
                fp_sel[l].set_offsets(
                    np.column_stack((seg['probe_m'][idx], seg['force'][idx]))
                )
            if step is not None and step[0] == l and l not in skipped:
                which = step[1]
                idx = sel.get((l, which))
                if idx is not None:
                    fp_rings[l].set_offsets(
                        [[seg['probe_m'][idx], seg['force'][idx]]]
                    )
        if reset_view:
            fp_ax.relim()
            fp_ax.autoscale(enable=True)
            fp_ax.set_xlim(-1.0,0.9)
            fp_ax.set_ylim(-1, 50)

    def refresh_context_rings():
        empty = np.empty((0, 2))
        for l in sensors:
            ctx_rings[(l, 'force')].set_offsets(empty)
            ctx_rings[(l, 'pos')].set_offsets(empty)
        step = current_step()
        if step is None:
            return
        l, which = step
        if l in skipped_by_row[state['job_i']]:
            return
        idx = selections[state['job_i']].get((l, which))
        seg = segments[state['job_i']][l]
        if idx is None or seg is None:
            return
        t = seg['t_disp'][idx]
        full_i = seg['full_idx'][idx]
        ctx_rings[(l, 'force')].set_offsets([[t, full[l]['force'][full_i]]])
        ctx_rings[(l, 'pos')].set_offsets([[t, full[l]['pos_mm'][full_i]]])

    def update_titles():
        rec = records[state['job_i']]
        plot, stalk = rec['Plot'], rec['Stalk']
        step = current_step()
        mode = 'REFINE' if refine else 'VIEW'
        locked = 'confirmed' if state['job_i'] in state['confirmed'] else 'editing'
        if step is None:
            pick = 'no original spans on this row'
            bound = '—'
        else:
            l, which = step
            idx = selections[state['job_i']].get((l, which))
            bound = f'{l}{which}'
            if l in skipped_by_row[state['job_i']]:
                pick = 'skipped'
            elif idx is None:
                pick = 'not set'
            else:
                seg = segments[state['job_i']][l]
                pick = (
                    f'idx={idx}/{seg["n"]-1}  '
                    f't={seg["t_raw"][idx]:.3f}s  '
                    f'x={seg["pos_m"][idx]:.4f}m  '
                    f'F={seg["force"][idx]:.2f}N'
                )
        fp_fig.suptitle(
            f"{mode}  [plot={plot}, stalk={stalk:02d}]  "
            f"{bound}  ({locked})  "
            f"{state['job_i']+1}/{len(records)}",
            fontsize=11,
        )
        ctx_fig.suptitle(
            f"Context  [plot={plot}, stalk={stalk:02d}]  "
            f"gray = original span   black = refine span",
            fontsize=11,
        )
        if refine:
            status_text.set_text(
                f"{bound}  {pick}   • click {bound[0] if step else ''} F–P axes   "
                f"• ←/→ sample   • ↑/↓ bound   • x = skip sensor   "
                f"• enter = confirm   • q = finish"
            )
        else:
            status_text.set_text(
                f"[plot={plot}, stalk={stalk:02d}]  {pick}   "
                f"• ← prev   • → next   • q = close"
            )

    def redraw(reset_zoom_flag=True):
        refresh_context_overlays()
        refresh_context_rings()
        refresh_fp_plot(reset_view=reset_zoom_flag)
        update_titles()
        ctx_fig.canvas.draw_idle()
        fp_fig.canvas.draw_idle()

    def goto(job_i, reset_step=True):
        state['job_i'] = int(max(0, min(len(records) - 1, job_i)))
        if reset_step:
            state['step_i'] = 0
        frame_context_on_stalk()
        redraw()

    def set_bound(idx):
        if not refine:
            return
        step = current_step()
        if step is None:
            return
        l, which = step
        if l in skipped_by_row[state['job_i']]:
            return
        seg = segments[state['job_i']][l]
        if seg is None:
            return
        idx = int(max(0, min(seg['n'] - 1, idx)))
        selections[state['job_i']][(l, which)] = idx
        if state['job_i'] in state['confirmed']:
            state['confirmed'].discard(state['job_i'])

    def nearest_fp_index(ax, probe, force, event):
        if event.xdata is None or event.ydata is None or probe.size == 0:
            return 0
        n = probe.size
        if n > max_snap_pts:
            # F–P is not sorted, so fall back to a coarse stride then a
            # local 2-D refine around the winner.
            stride = int(np.ceil(n / max_snap_pts))
            coarse = np.arange(0, n, stride, dtype=np.int64)
            pts = np.column_stack((probe[coarse], force[coarse]))
            disp = ax.transData.transform(pts)
            delta = disp - np.array([event.x, event.y], dtype=np.float64)
            winner = coarse[int(np.argmin(np.einsum('ij,ij->i', delta, delta)))]
            lo = max(0, winner - max_snap_pts // 2)
            hi = min(n, winner + max_snap_pts // 2)
            sl = slice(lo, hi)
            pts = np.column_stack((probe[sl], force[sl]))
            disp = ax.transData.transform(pts)
            delta = disp - np.array([event.x, event.y], dtype=np.float64)
            return lo + int(np.argmin(np.einsum('ij,ij->i', delta, delta)))
        pts = np.column_stack((probe, force))
        disp = ax.transData.transform(pts)
        delta = disp - np.array([event.x, event.y], dtype=np.float64)
        return int(np.argmin(np.einsum('ij,ij->i', delta, delta)))

    def on_fp_click(event):
        if not refine or not state['select_on']:
            return
        if event.inaxes is None or event.button != 1:
            return
        tb = getattr(fp_fig.canvas, 'toolbar', None)
        if tb is not None and getattr(tb, 'mode', ''):
            return
        if fp_fig.canvas.widgetlock.locked():
            return
        step = current_step()
        if step is None:
            return
        l, _which = step
        if event.inaxes is not fp_ax:
            return
        if l in skipped_by_row[state['job_i']]:
            return
        seg = segments[state['job_i']][l]
        if seg is None:
            return
        idx = nearest_fp_index(event.inaxes, seg['probe_m'], seg['force'], event)
        set_bound(idx)
        redraw()

    def nudge(delta):
        if not refine:
            # View mode: left/right pages stalks.
            goto(state['job_i'] + delta)
            return
        step = current_step()
        if step is None:
            return
        l, which = step
        cur = selections[state['job_i']].get((l, which))
        if cur is None:
            cur = 0
        set_bound(cur + delta)
        redraw(reset_zoom_flag=False)

    def step_bound(delta):
        if not refine:
            return
        steps = current_steps()
        if not steps:
            return
        state['step_i'] = int(max(0, min(len(steps) - 1, state['step_i'] + delta)))
        redraw(reset_zoom_flag=False)

    def undo_bound(event=None):
        if not refine:
            return
        step = current_step()
        if step is None:
            return
        sel = selections[state['job_i']]
        if step in sel:
            sel.pop(step)
            state['confirmed'].discard(state['job_i'])
        elif state['step_i'] > 0:
            state['step_i'] -= 1
            sel.pop(current_step(), None)
            state['confirmed'].discard(state['job_i'])
        redraw(reset_zoom_flag=False)

    def skip_sensor(event=None):
        '''Toggle: mark the current sensor empty for stiffness, or restore
        its full original span if it was already skipped.'''
        if not refine:
            return
        step = current_step()
        if step is None:
            return
        l, _which = step
        job_i = state['job_i']
        seg = segments[job_i][l]
        state['confirmed'].discard(job_i)
        if l in skipped_by_row[job_i]:
            skipped_by_row[job_i].discard(l)
            if seg is not None:
                selections[job_i][(l, 'min')] = 0
                selections[job_i][(l, 'max')] = seg['n'] - 1
        else:
            skipped_by_row[job_i].add(l)
            selections[job_i].pop((l, 'min'), None)
            selections[job_i].pop((l, 'max'), None)
            steps = current_steps()
            for k, (lab, _w) in enumerate(steps):
                if lab != l:
                    state['step_i'] = k
                    break
        redraw()

    def prev_stalk(event=None):
        if state['job_i'] <= 0:
            print("Already on the first [plot, stalk]")
            return
        goto(state['job_i'] - 1)

    def next_stalk(event=None):
        if state['job_i'] >= len(records) - 1:
            print("Already on the last [plot, stalk]")
            return
        goto(state['job_i'] + 1)

    def row_complete(job_i):
        '''Every sensor with an original span is either skipped or fully bound.'''
        sel = selections[job_i]
        skipped = skipped_by_row[job_i]
        for l in sensors:
            if segments[job_i][l] is None:
                continue
            if l in skipped:
                continue
            if (l, 'min') not in sel or (l, 'max') not in sel:
                return False
        # At least one sensor must remain if the row had any original data.
        usable = [
            l for l in sensors
            if segments[job_i][l] is not None and l not in skipped
        ]
        return True if usable or all(segments[job_i][l] is None for l in sensors) else False

    def confirm_stalk(event=None):
        if not refine:
            next_stalk()
            return
        job_i = state['job_i']
        rec = records[job_i]
        if not row_complete(job_i):
            missing = []
            for l in sensors:
                if segments[job_i][l] is None or l in skipped_by_row[job_i]:
                    continue
                if (l, 'min') not in selections[job_i]:
                    missing.append(f'{l}min')
                if (l, 'max') not in selections[job_i]:
                    missing.append(f'{l}max')
            print(f"[plot {rec['Plot']}, stalk {rec['Stalk']:02d}] "
                  f"incomplete — missing {missing}")
            return
        state['confirmed'].add(job_i)
        print(f"Confirmed [plot {rec['Plot']}, stalk {rec['Stalk']:02d}] "
              f"({len(state['confirmed'])}/{len(records)})")
        if job_i < len(records) - 1:
            goto(job_i + 1)
        else:
            print("Last [plot, stalk] confirmed — Finish & Save when ready")
            redraw()

    def apply_refine_to_records():
        '''Copy in-memory picks onto the record dicts that get written.'''
        for job_i, rec in enumerate(records):
            if job_i not in state['confirmed']:
                continue
            skipped = skipped_by_row[job_i]
            sel = selections[job_i]
            for l in sensors:
                seg = segments[job_i][l]
                if seg is None or l in skipped:
                    rec[f'{l}_Refine_Start'] = None
                    rec[f'{l}_Refine_End'] = None
                    continue
                sl = _seg_slice(sel, l)
                if sl is None:
                    rec[f'{l}_Refine_Start'] = None
                    rec[f'{l}_Refine_End'] = None
                    continue
                i0 = sel[(l, 'min')]
                i1 = sel[(l, 'max')]
                rec[f'{l}_Refine_Start'] = float(seg['t_raw'][i0])
                rec[f'{l}_Refine_End'] = float(seg['t_raw'][i1])

    def finish(event=None):
        if refine:
            apply_refine_to_records()
            write_stalk_rows(parent_data.stalks_csv_path, records)
            print(f"Status: wrote {len(records)} row(s) "
                  f"({len(state['confirmed'])} confirmed this session) to "
                  f"{parent_data.stalks_csv_path}")
        plt.close(fp_fig)
        plt.close(ctx_fig)

    def toggle_select(event=None):
        if not refine:
            return
        state['select_on'] = not state['select_on']
        on = state['select_on']
        btn_select.label.set_text('Select: ON' if on else 'Select: OFF')
        btn_select.color = 'lightgreen' if on else '0.82'
        btn_select.ax.set_facecolor(btn_select.color)
        update_titles()
        fp_fig.canvas.draw_idle()

    # Touch-friendly controls live on the stalk (F–P) figure.
    if refine:
        ax_prev    = fp_fig.add_axes([0.02, 0.012, 0.10, 0.045])
        ax_undo    = fp_fig.add_axes([0.13, 0.012, 0.10, 0.045])
        ax_skip    = fp_fig.add_axes([0.24, 0.012, 0.12, 0.045])
        ax_confirm = fp_fig.add_axes([0.37, 0.012, 0.13, 0.045])
        ax_finish  = fp_fig.add_axes([0.51, 0.012, 0.13, 0.045])
        ax_select  = fp_fig.add_axes([0.65, 0.012, 0.13, 0.045])
        btn_prev    = Button(ax_prev,    '← Prev Stalk')
        btn_undo    = Button(ax_undo,    'Undo Bound')
        btn_skip    = Button(ax_skip,    'Skip Sensor')
        btn_confirm = Button(ax_confirm, 'Confirm Stalk')
        btn_finish  = Button(ax_finish,  'Finish & Save')
        btn_select  = Button(ax_select,  'Select: ON', color='lightgreen')
        btn_prev.on_clicked(prev_stalk)
        btn_undo.on_clicked(undo_bound)
        btn_skip.on_clicked(skip_sensor)
        btn_confirm.on_clicked(confirm_stalk)
        btn_finish.on_clicked(finish)
        btn_select.on_clicked(toggle_select)
    else:
        ax_prev   = fp_fig.add_axes([0.02, 0.012, 0.12, 0.045])
        ax_next   = fp_fig.add_axes([0.16, 0.012, 0.12, 0.045])
        ax_close  = fp_fig.add_axes([0.30, 0.012, 0.12, 0.045])
        btn_prev  = Button(ax_prev,  '← Prev')
        btn_next  = Button(ax_next,  'Next →')
        btn_close = Button(ax_close, 'Close')
        btn_prev.on_clicked(prev_stalk)
        btn_next.on_clicked(next_stalk)
        btn_close.on_clicked(finish)

    def on_key(event):
        key = event.key
        if refine:
            if key == 'left':
                nudge(-1)
            elif key == 'right':
                nudge(+1)
            elif key == 'up':
                step_bound(-1)
            elif key == 'down':
                step_bound(+1)
            elif key in ('backspace', 'delete'):
                undo_bound()
            elif key == 'enter':
                confirm_stalk()
            elif key in ('p', 'P'):
                prev_stalk()
            elif key in ('x', 'X'):
                skip_sensor()
            elif key in ('q', 'Q'):
                finish()
            elif key in (' ', 'm', 'M'):
                toggle_select()
        else:
            if key in ('left', 'p', 'P'):
                prev_stalk()
            elif key in ('right', 'enter', 'n', 'N'):
                next_stalk()
            elif key in ('q', 'Q', 'escape'):
                finish()

    def _disable_default_keymap(fig):
        # Same reason as interactive_detect_stalks: figure-manager keymap
        # steals arrows / q / p. Figure-local — does not touch rcParams.
        mgr = fig.canvas.manager
        if mgr is None:
            return
        for attr in ('key_press_handler_id', 'key_release_handler_id'):
            cid = getattr(mgr, attr, None)
            if cid is not None:
                fig.canvas.mpl_disconnect(cid)
                setattr(mgr, attr, None)

    _disable_default_keymap(fp_fig)
    _disable_default_keymap(ctx_fig)
    fp_fig.canvas.mpl_connect('button_press_event', on_fp_click)
    fp_fig.canvas.mpl_connect('key_press_event', on_key)
    ctx_fig.canvas.mpl_connect('key_press_event', on_key)
    ctx_axs[0, 0].callbacks.connect('xlim_changed', update_context_base)

    # Initial y-scale on the context figure from the full traces.
    for i, l in enumerate(sensors):
        ctx_axs[i, 0].set_ylim(
            np.nanmin(full[l]['force']) - 1.0,
            np.nanmax(full[l]['force']) + 1.0,
        )
        ctx_axs[i, 1].set_ylim(
            min(0.0, float(np.nanmin(full[l]['pos_mm']))),
            float(np.nanmax(full[l]['pos_mm'])) * 1.05 + 1.0,
        )

    goto(0)
    plt.show(block=True)

