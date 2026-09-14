# First-pass interactive stalk labeling (time-domain, optional deriv panels).
# Public wrappers: interactive_detect_stalks, interactive_detect_stalks_derivs.

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
import matplotlib.pyplot as plt

from stalk_common import (
    TIME_SHIFT,
    _DEFAULT_MS,
    _DERIV_LEFT_PANELS,
    _DERIV_RIGHT_PANELS,
    _MAX_BASE_PTS,
    _MAX_SEL_PTS,
    _SELECTED_SIZE,
    _add_button,
    _bind_canvas,
    _click_blocked,
    _draw_idle,
    _empty_xy,
    _ensure_derivs,
    _ensure_force_pos,
    _nearest_index_in_view,
    _panel_y,
    _remove_artists,
    _require_stalk_sensors,
    _sensor_rgba,
    _set_select_button,
    _strided,
    _window_indices,
    load_stalk_rows,
    write_stalk_rows,
)


# =============================================================================
# Layout: interactive detection figures
# =============================================================================

def _build_detect_layout(parent_data, sensors, derivs):
    '''Force/position (one fig) or per-sensor 3x2 deriv figures.

    Returns a dict of artists and axis lookups used by the labeler. Full
    arrays stay in `series`; Line2D data is filled later from the view.
    '''
    if derivs:
        left_panels = _DERIV_LEFT_PANELS
        right_panels = _DERIV_RIGHT_PANELS
    else:
        left_panels = (('force', '{l} Force (N)', 1.0),)
        right_panels = (('position', '{l} Position (mm)', 1000.0),)

    panel_keys = [k for k, _y, _s in left_panels + right_panels]
    n_rows = len(left_panels)

    series = {}
    base_lines = {}
    sel_scatters = {}
    base_rgba = {}
    rings = {}
    ax_lookup = {}
    sensor_axes = {l: [] for l in sensors}
    figs = []
    fig_by_sensor = {}
    status_texts = []
    xlim_axes = []
    all_axes = []

    def _fill_sensor_series(i, l):
        s = parent_data.data_dict[f'Sensor_{l}']
        t = np.asarray(s['time'], dtype=np.float64) - TIME_SHIFT[l]
        if t.size == 0:
            return None
        row = {'t': t, 'n': t.size}
        for key, _ylabel, scale in left_panels + right_panels:
            row[key] = _panel_y(s, key, scale)
        series[l] = row
        base_rgba[l] = _sensor_rgba(parent_data, i)
        return True

    def _attach_panels(axs, l, rgba, xlabel=True):
        for col, panels in ((0, left_panels), (1, right_panels)):
            for row, (key, ylabel, _scale) in enumerate(panels):
                ax = axs[row, col]
                line, = ax.plot(
                    [], [], linestyle='None', marker='.',
                    markersize=_DEFAULT_MS, color=rgba,
                    rasterized=True, zorder=2,
                )
                ax.set_ylabel(ylabel.replace('{l}', l))
                ax.grid(True, alpha=0.3)
                sel = ax.scatter(
                    [], [], s=_SELECTED_SIZE, c='k',
                    linewidths=0, zorder=4,
                )
                ring = ax.scatter(
                    [], [], s=110, facecolors='none',
                    edgecolors='lime', linewidths=1.6, zorder=5,
                )
                base_lines[(l, key)] = line
                sel_scatters[(l, key)] = sel
                rings[(l, key)] = ring
                ax_lookup[ax] = (l, key)
                sensor_axes[l].append(ax)
                all_axes.append(ax)
        if xlabel:
            axs[-1, 0].set_xlabel('Time (s)')
            axs[-1, 1].set_xlabel('Time (s)')

    if derivs:
        for i, l in enumerate(sensors):
            if _fill_sensor_series(i, l) is None:
                print(f"Status: Sensor {l} has no samples")
                for opened in figs:
                    plt.close(opened)
                return None
            fig, axs = plt.subplots(
                n_rows, 2, figsize=(14, 9.4), sharex=True, squeeze=False,
            )
            fig._sensor_label = l
            fig.suptitle(
                f"Force & Position Derivatives - Sensor {l}",
                fontsize=12,
            )
            _attach_panels(axs, l, base_rgba[l])
            fig.subplots_adjust(
                left=0.08, right=0.995, bottom=0.10, top=0.90,
                wspace=0.18, hspace=0.12,
            )
            status_texts.append(
                fig.text(0.50, 0.058, '', ha='center', va='center', fontsize=9)
            )
            figs.append(fig)
            fig_by_sensor[l] = fig
            xlim_axes.append(axs[0, 0])
    else:
        for i, l in enumerate(sensors):
            if _fill_sensor_series(i, l) is None:
                print(f"Status: Sensor {l} has no samples")
                return None
        fig, axs = plt.subplots(
            len(sensors), 2, figsize=(12, 1.8 * len(sensors)),
            sharex=True, squeeze=False,
        )
        fig.suptitle("Interactive Stalk Detection", fontsize=12)
        for i, l in enumerate(sensors):
            row_axs = np.array([[axs[i, 0], axs[i, 1]]])
            _attach_panels(
                row_axs, l, base_rgba[l],
                xlabel=(i == len(sensors) - 1),
            )
        fig.subplots_adjust(
            left=0.02, right=0.995, bottom=0.12, top=0.98,
            wspace=0.12, hspace=0.08,
        )
        status_texts.append(
            fig.text(0.50, 0.065, '', ha='center', va='center', fontsize=9)
        )
        figs.append(fig)
        for l in sensors:
            fig_by_sensor[l] = fig
        xlim_axes.append(axs[0, 0])

    return {
        'figs': figs,
        'fig_by_sensor': fig_by_sensor,
        'series': series,
        'base_lines': base_lines,
        'sel_scatters': sel_scatters,
        'rings': rings,
        'ax_lookup': ax_lookup,
        'sensor_axes': sensor_axes,
        'panel_keys': panel_keys,
        'xlim_axes': xlim_axes,
        'all_axes': all_axes,
        'status_texts': status_texts,
        'derivs': derivs,
    }


# =============================================================================
# Interactive first-pass labeling
# =============================================================================

@dataclass
class _DetectJob:
    '''One stalk slot on one plot. Confirmed / vlines travel with the job
    so insert/drop is a list splice instead of reindexing parallel arrays.'''
    plot: int
    seq: int
    selections: dict = field(default_factory=dict)
    assigned_stalk: int | None = None
    loaded_key: tuple | None = None
    skipped: set = field(default_factory=set)
    confirmed: bool = False
    vlines: dict = field(default_factory=dict)


class DetectSession:
    '''Point-index labeling of stalks on sensors A, B, and C.

    Mutable UI state lives on this instance so we do not park it on
    HiSTIFFSData. Run via interactive_detect_stalks().
    '''

    def __init__(self, parent_data, num_plots=3, derivs=False):
        self.parent_data = parent_data
        self.num_plots = num_plots
        self.derivs = derivs
        self.jobs: list[_DetectJob] = []
        self.job_i = 0
        self.step_i = 0
        self.select_on = True
        self.updating_base = False
        self.closed_plots: set[int] = set()
        self.closing = False
        self.select_buttons = []
        self._buttons = []

    # ----- setup -----

    def run(self):
        if not self._setup():
            return
        self._load_existing()
        self._open_initial_slot()
        self._wire_ui()
        self._initial_view()
        plt.show(block=True)

    def _setup(self):
        n_plots = int(self.num_plots)
        if n_plots < 1:
            print("Status: num_plots must be >= 1")
            return False
        self.n_plots = n_plots

        if not _ensure_force_pos(self.parent_data):
            return False

        sensors = _require_stalk_sensors(
            self.parent_data, 'interactive_detect_stalks',
        )
        if sensors is None:
            return False
        self.sensors = sensors
        # Bound order is fixed; each entry is (sensor_label, 'min'|'max').
        self.steps = [(l, bound) for l in sensors for bound in ('min', 'max')]

        if self.derivs and not _ensure_derivs(self.parent_data, sensors):
            return False

        layout = _build_detect_layout(self.parent_data, sensors, self.derivs)
        if layout is None:
            return False
        self.figs = layout['figs']
        self.fig_by_sensor = layout['fig_by_sensor']
        self.series = layout['series']
        self.base_lines = layout['base_lines']
        self.sel_scatters = layout['sel_scatters']
        self.rings = layout['rings']
        self.ax_lookup = layout['ax_lookup']
        self.sensor_axes = layout['sensor_axes']
        self.panel_keys = layout['panel_keys']
        self.xlim_axes = layout['xlim_axes']
        self.all_axes = layout['all_axes']
        self.status_texts = layout['status_texts']
        return True

    # ----- job list -----

    def insert_job(self, at, plot, seq, sel=None, assigned=None,
                   loaded_key=None, skipped=None):
        '''Insert a stalk slot. Later inserts at or before job_i shift it.'''
        occupied = len(self.jobs) > 0
        self.jobs.insert(at, _DetectJob(
            plot=plot,
            seq=seq,
            selections=dict() if sel is None else sel,
            assigned_stalk=assigned,
            loaded_key=loaded_key,
            skipped=set() if skipped is None else set(skipped),
        ))
        # First insert into an empty list: leave job_i at 0. Later inserts
        # that land at or before the current slot shift it right.
        if occupied and self.job_i >= at:
            self.job_i += 1
        return at

    def drop_job(self, at):
        self._clear_vlines(self.jobs[at])
        self.jobs.pop(at)
        if self.job_i > at:
            self.job_i -= 1
        elif self.job_i >= len(self.jobs):
            self.job_i = max(0, len(self.jobs) - 1)

    def jobs_for_plot(self, plot):
        return [i for i, job in enumerate(self.jobs) if job.plot == plot]

    def insert_position_for_plot(self, plot):
        '''First index of a lower-numbered plot, or len(jobs).'''
        for i, job in enumerate(self.jobs):
            if job.plot < plot:
                return i
        return len(self.jobs)

    def current_job(self):
        if not self.jobs:
            return None
        return self.jobs[self.job_i]

    def current_step(self):
        return self.steps[self.step_i]

    def bound_time(self, job_i, sensor, which):
        idx = self.jobs[job_i].selections.get((sensor, which))
        if idx is None:
            return None
        return float(self.series[sensor]['t'][idx])

    def usable_steps(self, job_i):
        '''Step indices whose sensor is not skipped on this stalk.'''
        skipped = self.jobs[job_i].skipped
        return [i for i, (l, _w) in enumerate(self.steps) if l not in skipped]

    def first_usable_step(self, job_i):
        usable = self.usable_steps(job_i)
        return usable[0] if usable else 0

    def stalk_complete(self, job_i):
        '''Every sensor is either skipped or has both bounds; at least one used.'''
        job = self.jobs[job_i]
        any_used = False
        for l in self.sensors:
            if l in job.skipped:
                continue
            if (l, 'min') not in job.selections or (l, 'max') not in job.selections:
                return False
            any_used = True
        return any_used

    def stalk_label(self, job_i):
        '''Human-readable stalk id: assigned CSV number, or pick-order.'''
        assigned = self.jobs[job_i].assigned_stalk
        if assigned is not None:
            return f"{assigned:02d}"
        return f"#{self.jobs[job_i].seq}"

    def _index_from_raw_time(self, sensor, t_raw):
        '''Map a CSV raw-clock time onto this sensor's display-shifted series.'''
        t_disp = float(t_raw) - TIME_SHIFT[sensor]
        t = self.series[sensor]['t']
        return int(np.argmin(np.abs(t - t_disp)))

    def _unconfirm(self, job_i):
        job = self.jobs[job_i]
        if job.confirmed:
            job.confirmed = False
            self._clear_vlines(job)

    def _clear_assigned_for_plot(self, plot):
        '''Stalk numbers are unknown again once a plot is re-opened.'''
        for i in self.jobs_for_plot(plot):
            self.jobs[i].assigned_stalk = None
        self.closed_plots.discard(plot)

    # ----- load / resume -----

    def _load_existing(self):
        # Resume from an existing stalks CSV so one stalk can be edited
        # without re-picking the rest. Keys are (Plot, Stalk) as written.
        existing_rows = load_stalk_rows(self.parent_data.stalks_csv_path)
        self.existing_rows = existing_rows
        self.existing_by_key = {
            (rec['Plot'], rec['Stalk']): rec for rec in existing_rows
        }

        by_plot = {}
        for rec in existing_rows:
            p = rec['Plot']
            if p is None:
                continue
            by_plot.setdefault(p, []).append(rec)

        n_loaded = 0
        for p in range(self.n_plots, 0, -1):
            recs = by_plot.get(p, [])
            recs.sort(key=lambda r: (r['Stalk'] if r['Stalk'] is not None else 0))
            if not recs:
                continue
            for seq, rec in enumerate(recs, start=1):
                sel = {}
                skipped = set()
                complete = True
                any_used = False
                for l in self.sensors:
                    t0 = rec.get(f'{l}_Start')
                    t1 = rec.get(f'{l}_End')
                    if t0 is None and t1 is None:
                        # Blank pair = this stalk did not show on that sensor.
                        skipped.add(l)
                        continue
                    if t0 is None or t1 is None:
                        complete = False
                        if t0 is not None:
                            sel[(l, 'min')] = self._index_from_raw_time(l, t0)
                        if t1 is not None:
                            sel[(l, 'max')] = self._index_from_raw_time(l, t1)
                        continue
                    sel[(l, 'min')] = self._index_from_raw_time(l, t0)
                    sel[(l, 'max')] = self._index_from_raw_time(l, t1)
                    any_used = True
                if not any_used:
                    complete = False
                at = self.insert_job(
                    len(self.jobs), p, seq, sel=sel,
                    assigned=rec['Stalk'], loaded_key=(p, rec['Stalk']),
                    skipped=skipped,
                )
                if complete:
                    self.jobs[at].confirmed = True
                    n_loaded += 1
            self.closed_plots.add(p)

        if existing_rows:
            print(f"Status: loaded {n_loaded} confirmed stalk(s) "
                  f"from {self.parent_data.stalks_csv_path}")

    def _open_initial_slot(self):
        # Open an empty slot on the highest plot that has no stalks yet so a
        # partial file continues. If every plot already loaded, stay on job 0
        # and use Prev to reach the stalk being edited.
        opened = False
        for p in range(self.n_plots, 0, -1):
            if self.jobs_for_plot(p):
                continue
            at = self.insert_job(self.insert_position_for_plot(p), p, 1)
            self.job_i = at
            opened = True
            break
        if not opened:
            for i, job in enumerate(self.jobs):
                if not job.confirmed:
                    self.job_i = i
                    break

    # ----- drawing -----

    def update_base_traces(self, _ax=None):
        '''Redraw each base Line2D from the visible time window only.

        Called on xlim change and once at startup. Shared-x means one
        callback covers all axes on a figure. With derivs=True the three
        sensor figures share xlim so A/B/C stay aligned after TIME_SHIFT.
        Y-limits are left alone so a zoom in Y is not undone by a pan in X.
        '''
        if self.updating_base:
            return
        self.updating_base = True
        try:
            src = _ax if _ax is not None else self.xlim_axes[0]
            x0, x1 = src.get_xlim()
            for ax in self.xlim_axes:
                if ax is src:
                    continue
                cur0, cur1 = ax.get_xlim()
                if cur0 != x0 or cur1 != x1:
                    ax.set_xlim(x0, x1)
            for l in self.sensors:
                t = self.series[l]['t']
                i0, i1 = _window_indices(t, x0, x1)
                idx = _strided(i0, i1, _MAX_BASE_PTS)
                tt = t[idx]
                for key in self.panel_keys:
                    self.base_lines[(l, key)].set_data(tt, self.series[l][key][idx])
        finally:
            self.updating_base = False

    def _clear_vlines(self, job):
        artists = job.vlines
        job.vlines = {}
        for group in artists.values():
            _remove_artists(group)

    def draw_vlines(self, job_i):
        '''Confirmed-stalk markers: lw=1 at each bound time, every axis.'''
        job = self.jobs[job_i]
        self._clear_vlines(job)
        job.vlines = {}
        for l in self.sensors:
            t_min = self.bound_time(job_i, l, 'min')
            t_max = self.bound_time(job_i, l, 'max')
            if t_min is None or t_max is None:
                continue
            lines = []
            for ax in self.sensor_axes[l]:
                lines.append(ax.axvline(t_min, color='0.15', lw=1.0,
                                        ls='-', zorder=3))
                lines.append(ax.axvline(t_max, color='0.15', lw=1.0,
                                        ls='-', zorder=3))
            job.vlines[l] = lines

    def _selected_indices(self, sensor):
        '''Unique sample indices that should render black on this sensor.'''
        parts = []
        for job in self.jobs:
            i0 = job.selections.get((sensor, 'min'))
            i1 = job.selections.get((sensor, 'max'))
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
        n = self.series[sensor]['n']
        span = sum(hi - lo for lo, hi in parts)
        # Boolean merge when ranges may overlap or cover most of the trace.
        if span > n or len(parts) > 1:
            mask = np.zeros(n, dtype=bool)
            for lo, hi in parts:
                mask[lo:hi] = True
            return np.flatnonzero(mask)
        lo, hi = parts[0]
        return np.arange(lo, hi, dtype=np.int64)

    def refresh_selection_overlays(self):
        '''Paint selected samples as a small black overlay — not a restyle
        of the full base PathCollection. Cap marker count so a wide
        accidental range cannot freeze the canvas.
        '''
        for l in self.sensors:
            idx = self._selected_indices(l)
            if idx.size > _MAX_SEL_PTS:
                step = int(np.ceil(idx.size / _MAX_SEL_PTS))
                idx = np.unique(np.concatenate((idx[::step], idx[[0, -1]])))
            if idx.size == 0:
                for key in self.panel_keys:
                    self.sel_scatters[(l, key)].set_offsets(_empty_xy())
                continue
            tt = self.series[l]['t'][idx]
            for key in self.panel_keys:
                self.sel_scatters[(l, key)].set_offsets(
                    np.column_stack((tt, self.series[l][key][idx]))
                )

    def refresh_rings(self):
        '''Green ring on the active bound, every axis of that sensor.'''
        for l in self.sensors:
            for key in self.panel_keys:
                self.rings[(l, key)].set_offsets(_empty_xy())

        if not self.jobs:
            return
        l, which = self.current_step()
        job = self.jobs[self.job_i]
        if l in job.skipped:
            return
        idx = job.selections.get((l, which))
        if idx is None:
            return
        t = self.series[l]['t'][idx]
        for key in self.panel_keys:
            self.rings[(l, key)].set_offsets([[t, self.series[l][key][idx]]])

    def _set_status(self, text):
        for st in self.status_texts:
            st.set_text(text)

    def _set_titles(self, session_line):
        if self.derivs:
            active = None if not self.jobs else self.current_step()[0]
            for sl, f in self.fig_by_sensor.items():
                mark = '  [ACTIVE]' if sl == active else ''
                f.suptitle(
                    f"Force & Position Derivatives - Sensor {sl}{mark}\n"
                    f"{session_line}",
                    fontsize=11,
                )
        else:
            self.figs[0].suptitle(session_line, fontsize=11)

    def update_title(self):
        job = self.current_job()
        mode = 'SELECT' if self.select_on else 'NAVIGATE'
        if job is None:
            self._set_titles(
                f"Interactive Stalk Detection  [no stalks]  [{mode}]"
            )
            self._set_status("n = done with plot   • q = finish")
            return
        plot = job.plot
        l, which = self.current_step()
        skipped = job.skipped
        idx = job.selections.get((l, which))
        if l in skipped:
            pick = 'skipped'
        elif idx is None:
            pick = 'click any panel' if self.derivs else 'click force or position'
        else:
            pick = f'idx={idx}  t={self.series[l]["t"][idx]:.3f}s'
        done = 'confirmed' if job.confirmed else 'editing'
        n_here = sum(1 for i in self.jobs_for_plot(plot) if self.stalk_complete(i))
        skip_txt = f"  skip={','.join(sorted(skipped))}" if skipped else ''
        self._set_titles(
            f"Interactive Stalk Detection  [{plot=}, stalk={self.stalk_label(self.job_i)}]  "
            f"{n_here} in plot  {l}{which}  ({done}){skip_txt}  [{mode}]"
        )
        self._set_status(
            f"{l} {which}  {pick}   • click {l} axes to set   "
            f"• ←/→ sample   • ↑/↓ bound   • x = skip sensor   "
            f"• n = done with plot   • space = select on/off   • q = finish"
        )

    def redraw(self, selection=True):
        if selection:
            self.refresh_selection_overlays()
        self.refresh_rings()
        self.update_title()
        _draw_idle(*self.figs)

    def close_figures(self):
        if self.closing:
            return
        self.closing = True
        for f in self.figs:
            plt.close(f)

    def on_close(self, _event=None):
        self.close_figures()

    def _initial_view(self):
        t_lo = min(float(self.series[l]['t'][0]) for l in self.sensors)
        t_hi = max(float(self.series[l]['t'][-1]) for l in self.sensors)
        for ax in self.xlim_axes:
            ax.set_xlim(t_lo, t_hi)
        self.update_base_traces()
        for ax in self.all_axes:
            ax.relim()
            ax.autoscale(axis='y')
        for ax in self.xlim_axes:
            ax.callbacks.connect('xlim_changed', self.update_base_traces)

        for job_i, job in enumerate(self.jobs):
            if job.confirmed:
                self.draw_vlines(job_i)
        if self.jobs:
            self.step_i = self.first_usable_step(self.job_i)
        self.redraw()

    # ----- bound edits -----

    def set_bound(self, idx):
        '''Write the current bound, clamp to the sensor's sample count.'''
        if not self.jobs:
            return
        l, which = self.current_step()
        job = self.jobs[self.job_i]
        job.skipped.discard(l)
        n = self.series[l]['n']
        idx = int(max(0, min(n - 1, idx)))
        job.selections[(l, which)] = idx
        # Editing a locked stalk invalidates Confirm until they re-lock.
        self._unconfirm(self.job_i)

    def on_click(self, event):
        if not self.jobs or not self.select_on:
            return
        if _click_blocked(event):
            return
        # Ignore clicks on the control buttons (they live on their own axes).
        info = self.ax_lookup.get(event.inaxes)
        if info is None:
            return
        l_click, key = info
        l, _which = self.current_step()
        if l_click != l:
            return

        y = self.series[l][key]
        idx = _nearest_index_in_view(event.inaxes, self.series[l]['t'], y, event)
        self.set_bound(idx)
        # Fast path: clicks fill the remaining unskipped bounds in order.
        usable = self.usable_steps(self.job_i)
        after = [i for i in usable if i > self.step_i]
        if after:
            self.step_i = after[0]
        self.redraw()

    def nudge(self, delta):
        if not self.jobs:
            return
        l, which = self.current_step()
        cur = self.jobs[self.job_i].selections.get((l, which))
        if cur is None:
            # No pick yet: plant a bound at the first sample, then apply delta.
            cur = 0
        self.set_bound(cur + delta)
        self.redraw()

    def step_bound(self, delta):
        # Walk all six bounds, including skipped sensors, so Skip can be
        # toggled off again. Click-to-advance still jumps over skipped.
        if not self.jobs:
            return
        self.step_i = int(max(0, min(len(self.steps) - 1, self.step_i + delta)))
        self.redraw(selection=False)  # bound cursor only; overlay unchanged

    def undo_bound(self, event=None):
        '''Clear the current bound, or step back one bound if it is empty.'''
        if not self.jobs:
            return
        job = self.jobs[self.job_i]
        key = self.current_step()
        if key in job.selections:
            job.selections.pop(key)
            self._unconfirm(self.job_i)
        elif self.step_i > 0:
            self.step_i -= 1
            job.selections.pop(self.current_step(), None)
            self._unconfirm(self.job_i)
        self.redraw()

    def skip_sensor(self, event=None):
        '''Toggle: this stalk did not show on the current sensor.

        Blank Start/End are written for a skipped sensor. At least one
        sensor must remain for Confirm. Toggle again to restore it.
        '''
        if not self.jobs:
            return
        l, _which = self.current_step()
        job = self.jobs[self.job_i]
        self._unconfirm(self.job_i)
        if l in job.skipped:
            job.skipped.discard(l)
            self.step_i = self.steps.index((l, 'min'))
        else:
            job.skipped.add(l)
            job.selections.pop((l, 'min'), None)
            job.selections.pop((l, 'max'), None)
            usable = self.usable_steps(self.job_i)
            if usable:
                after = [i for i in usable if i >= self.step_i]
                self.step_i = after[0] if after else usable[-1]
        print(f"[plot {job.plot}, stalk {self.stalk_label(self.job_i)}] "
              f"{'restored' if l not in job.skipped else 'skipped'} sensor {l}")
        self.redraw()

    def prev_stalk(self, event=None):
        '''Revisit the previous [plot, stalk]. Does not confirm anything.'''
        if not self.jobs or self.job_i <= 0:
            print("Already on the first [plot, stalk]")
            return
        self.job_i -= 1
        self.step_i = self.first_usable_step(self.job_i)
        self.redraw()

    def confirm_stalk(self, event=None):
        '''Lock the current stalk (each sensor skipped or fully bound) and
        open the next slot on the same plot when this is that plot's last
        stalk.
        '''
        if not self.jobs:
            print("No stalk slot to confirm")
            return
        job_i = self.job_i
        job = self.jobs[job_i]
        plot, seq = job.plot, job.seq
        skipped = job.skipped
        if not self.stalk_complete(job_i):
            if all(l in skipped for l in self.sensors):
                print(f"[plot {plot}, stalk {self.stalk_label(job_i)}] "
                      f"incomplete — all sensors skipped")
                return
            missing = [f"{l}{w}" for l, w in self.steps
                       if l not in skipped and (l, w) not in job.selections]
            print(f"[plot {plot}, stalk {self.stalk_label(job_i)}] incomplete — "
                  f"missing {missing}")
            return

        job.confirmed = True
        self.draw_vlines(job_i)
        n_here = sum(1 for i in self.jobs_for_plot(plot) if self.stalk_complete(i))
        skip_note = (f", skipped {','.join(sorted(skipped))}"
                     if skipped else '')
        print(f"Confirmed [plot {plot}, stalk {self.stalk_label(job_i)}] "
              f"({n_here} complete in this plot{skip_note})")

        plot_ids = self.jobs_for_plot(plot)
        is_last_of_plot = (job_i == plot_ids[-1])
        if is_last_of_plot:
            self._clear_assigned_for_plot(plot)
            insert_at = job_i + 1
            self.insert_job(insert_at, plot, seq + 1)
            self.job_i = insert_at
            self.step_i = 0
        elif job_i < len(self.jobs) - 1:
            self.job_i = job_i + 1
            self.step_i = self.first_usable_step(self.job_i)
        self.redraw()

    # ----- write / plot close -----

    def write_session(self):
        '''Write complete jobs. Plots touched this session replace their
        old CSV rows. Other plots keep whatever was already on disk.
        '''
        out = []
        session_plots = {job.plot for job in self.jobs} | set(self.closed_plots)
        n_from_session = 0
        for job_i, job in enumerate(self.jobs):
            if not self.stalk_complete(job_i):
                continue
            stalk = job.assigned_stalk
            if stalk is None:
                complete = [i for i in self.jobs_for_plot(job.plot)
                            if self.stalk_complete(i)]
                stalk = complete.index(job_i) + 1
            loaded = job.loaded_key
            if loaded is not None and loaded == (job.plot, stalk):
                rec = dict(self.existing_by_key[loaded])
            else:
                rec = {'Plot': job.plot, 'Stalk': stalk}
            rec['Plot'] = job.plot
            rec['Stalk'] = stalk
            skipped = job.skipped
            for l in self.sensors:
                if l in skipped:
                    rec[f'{l}_Start'] = None
                    rec[f'{l}_End'] = None
                    rec[f'{l}_Refine_Start'] = None
                    rec[f'{l}_Refine_End'] = None
                    continue
                t0 = self.bound_time(job_i, l, 'min')
                t1 = self.bound_time(job_i, l, 'max')
                rec[f'{l}_Start'] = None if t0 is None else t0 + TIME_SHIFT[l]
                rec[f'{l}_End'] = None if t1 is None else t1 + TIME_SHIFT[l]
                rec.setdefault(f'{l}_Refine_Start', None)
                rec.setdefault(f'{l}_Refine_End', None)
            out.append(rec)
            n_from_session += 1
        written = {(r['Plot'], r['Stalk']) for r in out}
        for rec in self.existing_rows:
            if rec['Plot'] in session_plots:
                continue
            key = (rec['Plot'], rec['Stalk'])
            if key not in written:
                out.append(dict(rec))
        out.sort(key=lambda r: (-(r['Plot'] or 0), (r['Stalk'] or 0)))
        if not out:
            print("Status: No stalks to write")
            return 0
        write_stalk_rows(self.parent_data.stalks_csv_path, out)
        print(f"Status: wrote {len(out)} row(s) "
              f"({n_from_session} from this session) to "
              f"{self.parent_data.stalks_csv_path}")
        return len(out)

    def close_plot(self, plot, write=True):
        '''Drop incomplete slots, number remaining stalks low→high.

        First pick in the plot becomes stalk 1, last pick becomes stalk K.
        Numbers already loaded from CSV are kept unless a new stalk was
        added (assigned_stalk cleared on Confirm of the last slot).
        '''
        for i in reversed(self.jobs_for_plot(plot)):
            if not self.stalk_complete(i):
                self.drop_job(i)
        indices = self.jobs_for_plot(plot)
        k = len(indices)
        already = k > 0 and all(self.jobs[i].assigned_stalk is not None
                                for i in indices)
        if not already:
            for rank, i in enumerate(indices):
                self.jobs[i].seq = rank + 1
                self.jobs[i].assigned_stalk = rank + 1
        self.closed_plots.add(plot)
        print(f"Plot {plot}: {k} stalk(s)")
        if write:
            self.write_session()
        return k

    def next_plot(self, event=None):
        '''No more stalks in this plot: save the count and move on.'''
        job = self.current_job()
        if job is None:
            print("No plot to close")
            return
        plot = job.plot
        self.close_plot(plot, write=True)

        next_p = plot - 1
        if next_p < 1:
            print("Last plot closed — Finish & Save when ready")
            if self.jobs and self.job_i >= len(self.jobs):
                self.job_i = len(self.jobs) - 1
            if self.jobs:
                self.step_i = self.first_usable_step(self.job_i)
            self.redraw()
            return

        existing = self.jobs_for_plot(next_p)
        if existing:
            unconfirmed = [i for i in existing if not self.jobs[i].confirmed]
            self.job_i = unconfirmed[0] if unconfirmed else existing[0]
        else:
            at = self.insert_job(self.insert_position_for_plot(next_p), next_p, 1)
            self.job_i = at
        self.step_i = self.first_usable_step(self.job_i)
        self.redraw()

    def finish(self, event=None):
        '''Close the current plot if it is still open, write, and exit.

        A stalk the user edited but did not re-Confirm is still written
        from the in-memory picks as long as each sensor is skipped or
        fully bound.
        '''
        job = self.current_job()
        if job is not None:
            if job.plot not in self.closed_plots:
                self.close_plot(job.plot, write=False)
        self.write_session()
        self.close_figures()

    def toggle_select(self, event=None):
        '''Flip click-to-snap so the same left button can pan/zoom.'''
        self.select_on = not self.select_on
        for btn in self.select_buttons:
            _set_select_button(btn, self.select_on)
        self.update_title()
        _draw_idle(*self.figs)

    def on_key(self, event):
        key = event.key
        if key == 'left':
            self.nudge(-1)
        elif key == 'right':
            self.nudge(+1)
        elif key == 'up':
            self.step_bound(-1)
        elif key == 'down':
            self.step_bound(+1)
        elif key in ('backspace', 'delete'):
            self.undo_bound()
        elif key in ('x', 'X'):
            self.skip_sensor()
        elif key == 'enter':
            # Same action as the Confirm button — not a second advance path.
            self.confirm_stalk()
        elif key in ('p', 'P'):
            self.prev_stalk()
        elif key in ('n', 'N'):
            self.next_plot()
        elif key in ('q', 'Q'):
            self.finish()
        elif key in (' ', 'm', 'M'):
            self.toggle_select()

    def _wire_ui(self):
        # Touch-friendly controls for Win / Ubuntu / RPi5 + external keyboard.
        # Derivs mode puts the same row on every sensor figure so whichever
        # window is in front can Confirm / skip / finish.
        for f in self.figs:
            self._buttons.extend([
                _add_button(f, [0.01, 0.012, 0.09, 0.045],
                            '← Prev Stalk', self.prev_stalk),
                _add_button(f, [0.11, 0.012, 0.09, 0.045],
                            'Undo Bound', self.undo_bound),
                _add_button(f, [0.21, 0.012, 0.10, 0.045],
                            'Skip Sensor', self.skip_sensor),
                _add_button(f, [0.32, 0.012, 0.11, 0.045],
                            'Confirm Stalk', self.confirm_stalk),
                _add_button(f, [0.44, 0.012, 0.12, 0.045],
                            'Done with Plot', self.next_plot),
                _add_button(f, [0.57, 0.012, 0.11, 0.045],
                            'Finish & Save', self.finish),
            ])
            btn_select = _add_button(
                f, [0.69, 0.012, 0.11, 0.045],
                'Select: ON', self.toggle_select, color='lightgreen',
            )
            self._buttons.append(btn_select)
            self.select_buttons.append(btn_select)
            _bind_canvas(
                f,
                on_click=self.on_click,
                on_key=self.on_key,
                on_close=self.on_close,
            )


def interactive_detect_stalks(parent_data, num_plots=3, derivs=False):
    '''
    Interactive point-index labeling of stalks on sensors A, B, and C.

    Parameters
    ----------
    data : HiSTIFFSData
        Loaded run with force/position available (calc_force_position already
        called, or this function will call it).
    num_plots : int
        How many plots to walk, high→low (plot N, then N-1, ... plot 1).
        Stalk count per plot is not fixed — Confirm adds another stalk on
        the same plot; Done with Plot closes that plot and moves on.
    derivs : bool
        If True, each sensor is its own 3x2 force/position + derivative
        figure (same layout as display_stalk_derivs). Clicks on any of
        that sensor's six axes set the current bound.

    The session walks plots high→low. Within a plot, stalks are picked
    first-in-the-row first (low time first). When the plot is closed,
    those picks are numbered low→high (first pick = stalk 01, last
    pick = stalk K), so stalk count increases with time.

    Interaction
    -----------
    * Bounds are taken in fixed order: Amin, Amax, Bmin, Bmax, Cmin, Cmax.
    * A click on that sensor's axes snaps to the nearest sample in 2-D
      display (pixel) space — not an arbitrary time value. Default layout
      is force or position; derivs=True accepts any of the six panels.
    * Left / Right arrows nudge the current bound by ±1 sample index.
    * Up / Down arrows move between the bounds of sensors that are not
      skipped. Skip Sensor (x) leaves that sensor's Start/End blank —
      the stalk did not show on it. At least one sensor must remain.
    * Confirm locks the current stalk and opens a new empty stalk on the
      same plot. Done with Plot drops an unfinished trailing stalk,
      numbers the plot, writes the CSV, and advances to the next plot.
    * Finish closes the current plot the same way if it is still open,
      writes the CSV, and closes the figure.

    Visuals
    -------
    * No shaded SpanSelector. Samples inside each sensor's selected index
      range are recolored pure black (so A stays visible while B/C are set).
    * The active bound gets a green ring on every axis of that sensor.
    * After Confirm, thickness=1 vertical lines are drawn at both bound
      times on every axis of that sensor.

    Times are stored exactly as picked. Amin may be after Amax in time;
    we never swap or reject on time order. Downstream readers that assume
    Start < End will need their own update later.

    CSV columns: Plot, Stalk, A_Start, A_End, B_Start, B_End, C_Start, C_End
    '''
    DetectSession(parent_data, num_plots=num_plots, derivs=derivs).run()


def interactive_detect_stalks_derivs(parent_data, num_plots=3):
    '''Same labeling session as interactive_detect_stalks, with more traces.

    Each of A/B/C is a 3x2 figure matching display_stalk_derivs:

        force        | position
        dF/dt        | dx/dt
        d²F/dt²      | d²x/dt²

    Click any panel of the current sensor to plant the bound. Green rings,
    black selected samples, and confirmed vertical lines are drawn on all
    six axes. Time is display-shifted like the original labeler so the
    three figures share an x-axis; CSV times are still the raw clocks.
    '''
    interactive_detect_stalks(parent_data, num_plots=num_plots, derivs=True)
