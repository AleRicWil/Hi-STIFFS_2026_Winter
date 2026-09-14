# Shared constants, drawing helpers, and stalks-CSV I/O for the labeling UI.
# Imported by stalk_detect.py / stalk_review.py. Public names are re-exported
# from stalk_detector.py so process.py imports stay the same.

import csv
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button

from config import Config


# =============================================================================
# Constants
# =============================================================================

# Display-only time alignment so A/B/C traces sit on one x-axis. CSV times
# are always the raw per-sensor clock (this shift is added back on write).
TIME_SHIFT = {'A': 0.0, 'B': 1.4, 'C': 2.75}

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

# Same 3x2 panel set as display_stalk_derivs / HiSTIFFSData.plot_derivs.
_DERIV_LEFT_PANELS = (
    ('force', 'Force (N)', 1.0),
    ('dF_dt', 'dF/dt (N/s)', 1.0),
    ('d2F_dt2', 'd²F/dt² (N/s²)', 1.0),
)
_DERIV_RIGHT_PANELS = (
    ('position', 'Position (mm)', 1000.0),
    ('dx_dt', 'dx/dt (mm/s)', 1000.0),
    ('d2x_dt2', 'd²x/dt² (mm/s²)', 1000.0),
)
_DERIV_KEYS = ('dF_dt', 'd2F_dt2', 'dx_dt', 'd2x_dt2')

# Draw caps — full traces stay in numpy; only a windowed, strided copy is
# sent to the artist. Scatter-of-all-points plus per-click facecolor
# rebuilds is what made the UI stall.
_DEFAULT_MS = 2.4
_SELECTED_SIZE = 18.0
_MAX_BASE_PTS = 8000       # max markers drawn per axis in the current view
_MAX_SEL_PTS = 20000       # max black overlay markers per sensor
_MAX_SNAP_PTS = 20000      # max candidates for a full 2-D pixel snap


def _empty_xy():
    return np.empty((0, 2))


# =============================================================================
# Array / drawing helpers
# =============================================================================

def _panel_y(sensor_dict, key, scale):
    '''Display array for a detect/deriv panel. Position-family keys in mm.'''
    src = 'position' if key == 'position' else key
    return np.asarray(sensor_dict[src], dtype=np.float64) * scale


def _sensor_rgba(parent_data, i):
    return np.asarray(
        plt.matplotlib.colors.to_rgba(
            parent_data.colors[i % len(parent_data.colors)]
        )
    )


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


def _cap_idx(idx, cap):
    if idx.size <= cap:
        return idx
    step = int(np.ceil(idx.size / cap))
    return np.unique(np.concatenate((idx[::step], idx[[0, -1]])))


def _nearest_display_offset(ax, xs, ys, event):
    '''Index of the sample closest to the click in 2-D display (pixel) space.'''
    pts = np.column_stack((xs, ys))
    disp = ax.transData.transform(pts)
    delta = disp - np.array([event.x, event.y], dtype=np.float64)
    return int(np.argmin(np.einsum('ij,ij->i', delta, delta)))


def _nearest_index_in_view(ax, t, y, event, max_snap_pts=_MAX_SNAP_PTS):
    '''Snap click to nearest sample in the current xlim (sorted time axis).

    If that window is still huge we snap by time first, then refine 2-D in
    a local neighborhood — same result when zoomed in, without transforming
    hundreds of thousands of points on every click.
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
    return i0 + _nearest_display_offset(ax, t[i0:i1], y[i0:i1], event)


def _nearest_index_unsorted(ax, x, y, event, max_snap_pts=_MAX_SNAP_PTS):
    '''Snap click to nearest sample when the x-axis is not sorted (F–P).'''
    if event.xdata is None or event.ydata is None or x.size == 0:
        return 0
    n = x.size
    if n > max_snap_pts:
        stride = int(np.ceil(n / max_snap_pts))
        coarse = np.arange(0, n, stride, dtype=np.int64)
        winner = coarse[_nearest_display_offset(
            ax, x[coarse], y[coarse], event,
        )]
        lo = max(0, winner - max_snap_pts // 2)
        hi = min(n, winner + max_snap_pts // 2)
        return lo + _nearest_display_offset(ax, x[lo:hi], y[lo:hi], event)
    return _nearest_display_offset(ax, x, y, event)


def _remove_artists(artists):
    for item in artists:
        try:
            item.remove()
        except ValueError:
            pass


# =============================================================================
# Matplotlib widget helpers
# =============================================================================

def _disable_figure_keymap(fig):
    '''Drop the figure-manager keymap so arrows / q / p reach our handler.

    Matplotlib's default keymap intercepts the same keys we use: left/right
    = view history, p = pan, q = quit, s = save, g = grid, h/r = home.
    Mouse toolbar (zoom/pan icons) still works. Figure-local — does not
    change rcParams for later plots.
    '''
    mgr = fig.canvas.manager
    if mgr is None:
        return
    for attr in ('key_press_handler_id', 'key_release_handler_id'):
        cid = getattr(mgr, attr, None)
        if cid is not None:
            fig.canvas.mpl_disconnect(cid)
            setattr(mgr, attr, None)


def _click_blocked(event):
    '''True when the click is not a free left-button pick on an axes.

    Toolbar pan/zoom holds widgetlock and sets toolbar.mode.
    '''
    if event.inaxes is None or event.button != 1:
        return True
    canvas = event.canvas
    tb = getattr(canvas, 'toolbar', None)
    if tb is not None and getattr(tb, 'mode', ''):
        return True
    if canvas.widgetlock.locked():
        return True
    return False


def _bind_canvas(fig, *, on_click=None, on_key=None, on_close=None):
    _disable_figure_keymap(fig)
    if on_click is not None:
        fig.canvas.mpl_connect('button_press_event', on_click)
    if on_key is not None:
        fig.canvas.mpl_connect('key_press_event', on_key)
    if on_close is not None:
        fig.canvas.mpl_connect('close_event', on_close)


def _add_button(fig, rect, label, callback, color=None):
    ax = fig.add_axes(rect)
    kwargs = {} if color is None else {'color': color}
    btn = Button(ax, label, **kwargs)
    btn.on_clicked(callback)
    return btn


def _set_select_button(btn, on):
    btn.label.set_text('Select: ON' if on else 'Select: OFF')
    btn.color = 'lightgreen' if on else '0.82'
    # Button.color is used on the next hover/leave; force a face update.
    btn.ax.set_facecolor(btn.color)


def _draw_idle(*figs):
    for fig in figs:
        fig.canvas.draw_idle()


# =============================================================================
# Parent-data checks
# =============================================================================

def _ensure_loaded(parent_data):
    if not parent_data.exists:
        print("Status: No data loaded - call HiSTIFFSData(...) first")
        return False
    return True


def _ensure_force_pos(parent_data):
    if not _ensure_loaded(parent_data):
        return False
    if not parent_data.has_force_pos:
        parent_data.calc_force_position()
    return True


def _require_stalk_sensors(parent_data, purpose):
    sensors = [l for l in STALK_SENSORS if l in parent_data.sensor_labels]
    if sensors != list(STALK_SENSORS):
        print(f"Status: {purpose} needs sensors A, B, and C "
              f"(found {parent_data.sensor_labels})")
        return None
    return sensors


def _sensor_missing_derivs(sensor_dict, also=()):
    keys = _DERIV_KEYS + tuple(also)
    return any(k not in sensor_dict for k in keys)


def _ensure_derivs(parent_data, sensors):
    needs_derivs = any(
        _sensor_missing_derivs(parent_data.data_dict.get(f'Sensor_{l}', {}))
        for l in sensors
    )
    if needs_derivs:
        parent_data.calc_derivs()
    for l in sensors:
        s = parent_data.data_dict.get(f'Sensor_{l}', {})
        if _sensor_missing_derivs(s):
            print(f"Status: missing derivative data for Sensor {l}")
            return False
    return True


# =============================================================================
# CSV I/O  (also used by HiSTIFFSData.gather_stalk_traces)
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

    Returns a list of dicts in file order (high→low plot, low→high stalk —
    the same order the labeling UI walks, provided the user confirmed in
    order). Missing refine columns (older files) come back as None.
    '''
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

        def cell(name, _raw=raw, _col=col):
            j = _col.get(name)
            if j is None or j >= len(_raw):
                return None
            return _raw[j]

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
    return int(np.argmin(np.abs(t_raw - float(t_target))))

