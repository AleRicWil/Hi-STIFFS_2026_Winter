# Stalk review, refine (force vs probe position), and derivative overlay.
# Public wrappers: display_stalk_selections, refine_stalk_selections,
# display_stalk_derivs.

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
    _cap_idx,
    _click_blocked,
    _draw_idle,
    _empty_xy,
    _ensure_force_pos,
    _ensure_loaded,
    _nearest_index_in_segment,
    _nearest_index_unsorted,
    _pair_ok,
    _remove_artists,
    _require_stalk_sensors,
    _sensor_missing_derivs,
    _sensor_rgba,
    _set_select_button,
    _strided,
    _time_window,
    _window_indices,
    inclusive_time_mask,
    load_stalk_rows,
    write_stalk_rows,
)


# =============================================================================
# Derivative overlay (view-only)
# =============================================================================

def display_stalk_derivs(parent_data, sensors='A,B,C,D,E,F', return_figs=False):
    '''Derivative traces with original labelled Start/End overlaid.

    Same per-sensor 3x2 layout as HiSTIFFSData.plot_derivs. Dashed
    vertical lines and a dark overlay mark each row's {L}_Start/{L}_End
    from the stalks CSV. Refine bounds are not drawn.

    Drawing matches the interactive labeler: full arrays stay off-canvas;
    each axis gets a rasterized Line2D filled with a windowed, strided
    copy (max 8000 pts). Overlay PathCollections are capped the same
    way and both refresh on xlim change.
    '''
    if not _ensure_loaded(parent_data):
        return [] if return_figs else None

    sensors_to_plot = [label.strip() for label in sensors.split(',')]
    removed = [label for label in sensors_to_plot if label not in parent_data.sensor_labels]
    for label in removed:
        print(f"Sensor {label} not in CSV data")
    sensors_to_plot = [label for label in sensors_to_plot if label in parent_data.sensor_labels]
    if not sensors_to_plot:
        print("No valid sensors to plot.")
        return [] if return_figs else None

    needs_derivs = False
    for l in sensors_to_plot:
        s = parent_data.data_dict.get(f'Sensor_{l}', {})
        if _sensor_missing_derivs(s, also=('force', 'position')):
            needs_derivs = True
            break
    if needs_derivs:
        parent_data.calc_derivs()

    records = load_stalk_rows(parent_data.stalks_csv_path)
    if not records:
        print(f"Status: no stalk rows in {parent_data.stalks_csv_path}")
    else:
        print(f"Overlaying original Start/End for {len(records)} stalk row(s)")

    sensor_order = 'ABCDEF'
    ordered_sensors = sorted(sensors_to_plot, key=lambda x: sensor_order.index(x))
    figs = []

    for i, l in enumerate(ordered_sensors):
        s = parent_data.data_dict.get(f'Sensor_{l}', {})
        if (_sensor_missing_derivs(s, also=('force',))
                or 'time' not in s or 'force' not in s):
            print(f"Missing derivative data for Sensor {l}")
            continue

        t = np.asarray(s['time'], dtype=np.float64)
        if t.size == 0:
            print(f"Status: Sensor {l} has no samples")
            continue

        c = parent_data.colors[i % len(parent_data.colors)]
        fig, axs = plt.subplots(3, 2, sharex=True, figsize=(14, 9), squeeze=False)

        base_lines = []
        overlays = []
        ys = []
        for col, panels in ((0, _DERIV_LEFT_PANELS), (1, _DERIV_RIGHT_PANELS)):
            for row, (key, ylabel, scale) in enumerate(panels):
                y = np.asarray(s[key], dtype=np.float64) * scale
                line, = axs[row, col].plot(
                    [], [], linestyle='None', marker='.',
                    markersize=_DEFAULT_MS, color=c,
                    rasterized=True, zorder=2,
                )
                sc = axs[row, col].scatter(
                    [], [], s=5, c='0.15', linewidths=0,
                    rasterized=True, zorder=4,
                )
                axs[row, col].set_ylabel(ylabel)
                axs[row, col].grid(True, alpha=0.3)
                base_lines.append(line)
                overlays.append(sc)
                ys.append(y)

        axs[-1, 0].set_xlabel('Time (s)')
        axs[-1, 1].set_xlabel('Time (s)')

        overlay_mask = np.zeros(t.size, dtype=bool)
        n_overlaid = 0
        for rec in records:
            t0, t1 = rec.get(f'{l}_Start'), rec.get(f'{l}_End')
            if not _pair_ok(t0, t1):
                continue
            lo, hi = _time_window(t0, t1)
            for ax in axs.flat:
                ax.axvline(lo, color='0.45', lw=1.0, ls='--', zorder=3)
                ax.axvline(hi, color='0.45', lw=1.0, ls='--', zorder=3)
            overlay_mask |= inclusive_time_mask(t, t0, t1)
            n_overlaid += 1
        overlay_idx = np.flatnonzero(overlay_mask)

        fig.suptitle(
            f"Force & Position Derivatives - Sensor {l}\n"
            f"Test: {parent_data.test_type}  |  "
            f"{n_overlaid} labelled bound(s)",
            fontsize=12,
        )
        fig.subplots_adjust(left=0.08, right=0.995, bottom=0.07, top=0.90,
                            wspace=0.18, hspace=0.12)

        view = _DerivView(
            t=t, ys=ys, lines=base_lines, overlays=overlays,
            overlay_idx=overlay_idx, axs=axs,
        )
        view.attach()
        fig._stalk_deriv_view = view
        figs.append(fig)

    if return_figs:
        return figs
    for fig in figs:
        plt.show(block=False)
    return None


class _DerivView:
    '''xlim-driven redraw for one display_stalk_derivs figure.'''

    def __init__(self, t, ys, lines, overlays, overlay_idx, axs):
        self.t = t
        self.ys = ys
        self.lines = lines
        self.overlays = overlays
        self.overlay_idx = overlay_idx
        self.axs = axs
        self.updating = False

    def attach(self):
        self.axs[0, 0].set_xlim(float(self.t[0]), float(self.t[-1]))
        self.update_view()
        for ax in self.axs.flat:
            ax.relim()
            ax.autoscale(axis='y')
        self.axs[0, 0].callbacks.connect('xlim_changed', self.update_view)

    def update_view(self, _ax=None):
        if self.updating:
            return
        self.updating = True
        try:
            x0, x1 = self.axs[0, 0].get_xlim()
            i0, i1 = _window_indices(self.t, x0, x1)
            vis = _strided(i0, i1, _MAX_BASE_PTS)
            for line, y in zip(self.lines, self.ys):
                line.set_data(self.t[vis], y[vis])
            if self.overlay_idx.size:
                in_win = self.overlay_idx[
                    (self.overlay_idx >= i0) & (self.overlay_idx < i1)
                ]
                in_win = _cap_idx(in_win, _MAX_SEL_PTS)
            else:
                in_win = self.overlay_idx
            if in_win.size:
                tt = self.t[in_win]
                for sc, y in zip(self.overlays, self.ys):
                    sc.set_offsets(np.column_stack((tt, y[in_win])))
            else:
                for sc in self.overlays:
                    sc.set_offsets(_empty_xy())
        finally:
            self.updating = False


# =============================================================================
# Review / refine
# =============================================================================

class ReviewSession:
    '''Page through saved stalk rows. refine=True writes sub-spans.

    Figure 1 (context): full A/B/C force and position vs shifted time.
    Figure 2 (stalk): force vs probe position for this row only.
    '''

    def __init__(self, parent_data, refine):
        self.parent_data = parent_data
        self.refine = refine
        self.job_i = 0
        self.step_i = 0
        self.select_on = bool(refine)
        self.updating_ctx = False
        self._buttons = []

    def run(self):
        if not self._setup():
            return
        self._wire_ui()
        self._initial_view()
        plt.show(block=True)

    def _setup(self):
        if not _ensure_force_pos(self.parent_data):
            return False
        sensors = _require_stalk_sensors(self.parent_data, 'stalk review')
        if sensors is None:
            return False
        self.sensors = sensors

        records = load_stalk_rows(self.parent_data.stalks_csv_path)
        if not records:
            print(f"Status: no stalk rows in {self.parent_data.stalks_csv_path}")
            return False
        self.records = records

        if not self._build_series():
            return False
        self._build_segments()
        self._init_selections()
        self._build_context_figure()
        self._build_fp_figure()
        return True

    def _build_series(self):
        full = {}
        for i, l in enumerate(self.sensors):
            s = self.parent_data.data_dict[f'Sensor_{l}']
            t_raw = np.asarray(s['time'], dtype=np.float64)
            full[l] = {
                't_raw': t_raw,
                't_disp': t_raw - TIME_SHIFT[l],
                'force': np.asarray(s['force'], dtype=np.float64),
                'pos_mm': np.asarray(s['position'], dtype=np.float64) * 1000.0,
                'n': t_raw.size,
                'rgba': _sensor_rgba(self.parent_data, i),
            }
            if full[l]['n'] == 0:
                print(f"Status: Sensor {l} has no samples")
                return False
        self.full = full
        return True

    def _build_segments(self):
        # Per-row, per-sensor original segments. Refine indices live in this
        # segment's sample space so ±1 arrow steps cannot leave the original
        # bound. Time window only (inclusive_time_mask). Probe position:
        #     (length - sensor_position) + start_pos
        segments = []
        for rec in self.records:
            per_sensor = {}
            for l in self.sensors:
                t0, t1 = rec.get(f'{l}_Start'), rec.get(f'{l}_End')
                if not _pair_ok(t0, t1):
                    per_sensor[l] = None
                    continue
                lo, hi = _time_window(t0, t1)
                t_raw = self.full[l]['t_raw']
                # Inclusive on both ends: the labeler stores the clicked sample
                # times, and Start may be after End.
                mask = inclusive_time_mask(t_raw, lo, hi)
                idx = np.flatnonzero(mask)
                if idx.size == 0:
                    per_sensor[l] = None
                    continue
                s = self.parent_data.data_dict[f'Sensor_{l}']
                pos_m = np.asarray(s['position'], dtype=np.float64)[idx]
                per_sensor[l] = {
                    'full_idx': idx,
                    't_raw': t_raw[idx],
                    't_disp': self.full[l]['t_disp'][idx],
                    'force': self.full[l]['force'][idx],
                    'pos_m': pos_m,
                    'probe_m': (s['length'] - pos_m) + s['start_pos'],
                    'n': int(idx.size),
                }
            segments.append(per_sensor)
        self.segments = segments

    def _default_refine_state(self, job_i):
        '''Full original span, or skipped when that sensor has no samples.'''
        sel = {}
        skipped = set()
        rec = self.records[job_i]
        for l in self.sensors:
            seg = self.segments[job_i][l]
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

    @staticmethod
    def _row_was_previously_refined(rec, sensors):
        return any(
            rec.get(f'{l}_Refine_Start') is not None
            or rec.get(f'{l}_Refine_End') is not None
            for l in sensors
        )

    def _init_selections(self):
        # Selections for every row up front so paging back shows the same picks.
        selections = []
        skipped_by_row = []
        previously_refined = set()
        for job_i, rec in enumerate(self.records):
            sel, skipped = self._default_refine_state(job_i)
            # If this row was saved with refine blanks on a sensor that *has*
            # an original span, that was an explicit Skip — keep it skipped.
            if self._row_was_previously_refined(rec, self.sensors):
                previously_refined.add(job_i)
                for l in self.sensors:
                    if self.segments[job_i][l] is None:
                        continue
                    if not _pair_ok(rec.get(f'{l}_Refine_Start'),
                                    rec.get(f'{l}_Refine_End')):
                        skipped.add(l)
                        sel.pop((l, 'min'), None)
                        sel.pop((l, 'max'), None)
            selections.append(sel)
            skipped_by_row.append(skipped)
        self.selections = selections
        self.skipped_by_row = skipped_by_row
        self.confirmed = (
            set(previously_refined) if self.refine
            else set(range(len(self.records)))
        )

    def steps_for(self, job_i):
        '''Bound order Amin…Cmax, omitting sensors with no original span.'''
        out = []
        for l in self.sensors:
            if self.segments[job_i][l] is None:
                continue
            out.append((l, 'min'))
            out.append((l, 'max'))
        return out

    def current_steps(self):
        return self.steps_for(self.job_i)

    def current_step(self):
        steps = self.current_steps()
        if not steps:
            return None
        self.step_i = int(max(0, min(len(steps) - 1, self.step_i)))
        return steps[self.step_i]

    def _build_context_figure(self):
        sensors = self.sensors
        ctx_fig, ctx_axs = plt.subplots(
            len(sensors), 2,
            figsize=(12, 1.7 * len(sensors)),
            sharex=True, squeeze=False,
        )
        ctx_fig.suptitle("Stalk context (full traces)", fontsize=12)

        ctx_base = {}
        ctx_orig = {}
        ctx_ref = {}
        ctx_rings = {}
        ctx_vlines = {l: [] for l in sensors}

        for i, l in enumerate(sensors):
            rgba = self.full[l]['rgba']
            line_f, = ctx_axs[i, 0].plot(
                [], [], linestyle='None', marker='.', markersize=_DEFAULT_MS,
                color=rgba, rasterized=True, zorder=2,
            )
            line_p, = ctx_axs[i, 1].plot(
                [], [], linestyle='None', marker='.', markersize=_DEFAULT_MS,
                color=rgba, rasterized=True, zorder=2,
            )
            ctx_axs[i, 0].set_ylabel(f'{l} Force (N)')
            ctx_axs[i, 1].set_ylabel(f'{l} Position (mm)')
            ctx_axs[i, 0].grid(True, alpha=0.3)
            ctx_axs[i, 1].grid(True, alpha=0.3)
            ctx_base[(l, 'force')] = line_f
            ctx_base[(l, 'pos')] = line_p

            orig_f = ctx_axs[i, 0].scatter(
                [], [], s=10, c=[rgba], linewidths=0, zorder=3,
            )
            orig_p = ctx_axs[i, 1].scatter(
                [], [], s=10, c=[rgba], linewidths=0, zorder=3,
            )
            ref_f = ctx_axs[i, 0].scatter(
                [], [], s=_SELECTED_SIZE, c='k', linewidths=0, zorder=4,
            )
            ref_p = ctx_axs[i, 1].scatter(
                [], [], s=_SELECTED_SIZE, c='k', linewidths=0, zorder=4,
            )
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
        self.ctx_fig = ctx_fig
        self.ctx_axs = ctx_axs
        self.ctx_base = ctx_base
        self.ctx_orig = ctx_orig
        self.ctx_ref = ctx_ref
        self.ctx_rings = ctx_rings
        self.ctx_vlines = ctx_vlines

    def _build_fp_figure(self):
        # ONE force vs probe-position axes. Three independent scatters
        # (A/B/C), each colored by that sensor's own raw time. Same marker
        # for every sensor; no colorbar.
        fp_fig, fp_ax = plt.subplots(figsize=(11, 6.2))
        fp_fig.subplots_adjust(left=0.08, right=0.99, bottom=0.16, top=0.90)

        fp_base = {}
        fp_sel = {}
        fp_rings = {}
        for i, l in enumerate(self.sensors):
            rgba = self.full[l]['rgba']
            base = fp_ax.scatter(
                [], [], s=30, c=[], cmap='viridis', marker='o',
                linewidths=0, rasterized=True, zorder=2,
            )
            sel = fp_ax.scatter(
                [], [], s=5, c='red', marker='o',
                linewidths=0, zorder=4,
            )
            ring = fp_ax.scatter(
                [], [], s=140, facecolors='none', edgecolors='lime',
                marker='o', linewidths=1.8, zorder=5,
            )
            start_pos = self.parent_data.data_dict[f'Sensor_{l}']['start_pos']
            fp_ax.axvline(start_pos, color=rgba, linewidth=0.6, alpha=0.5, zorder=1)
            fp_base[l] = base
            fp_sel[l] = sel
            fp_rings[l] = ring
        fp_ax.set_xlabel('Probe position (m)')
        fp_ax.set_ylabel('Force (N)')
        fp_ax.grid(True, alpha=0.3)

        self.fp_fig = fp_fig
        self.fp_ax = fp_ax
        self.fp_base = fp_base
        self.fp_sel = fp_sel
        self.fp_rings = fp_rings
        self.status_text = fp_fig.text(
            0.50, 0.072, '', ha='center', va='center', fontsize=9,
        )

    # ----- drawing -----

    def update_context_base(self, _ax=None):
        if self.updating_ctx:
            return
        self.updating_ctx = True
        try:
            x0, x1 = self.ctx_axs[0, 0].get_xlim()
            for l in self.sensors:
                t = self.full[l]['t_disp']
                i0, i1 = _window_indices(t, x0, x1)
                idx = _strided(i0, i1, _MAX_BASE_PTS)
                self.ctx_base[(l, 'force')].set_data(t[idx], self.full[l]['force'][idx])
                self.ctx_base[(l, 'pos')].set_data(t[idx], self.full[l]['pos_mm'][idx])
        finally:
            self.updating_ctx = False

    @staticmethod
    def _seg_slice(sel, l):
        i0 = sel.get((l, 'min'))
        i1 = sel.get((l, 'max'))
        if i0 is None or i1 is None:
            return None
        lo, hi = (i0, i1) if i0 <= i1 else (i1, i0)
        return lo, hi

    def refresh_context_overlays(self):
        job_i = self.job_i
        sel = self.selections[job_i]
        skipped = self.skipped_by_row[job_i]
        rec = self.records[job_i]

        for l in self.sensors:
            for group in self.ctx_vlines[l]:
                _remove_artists(group)
            self.ctx_vlines[l] = []

            t0, t1 = rec.get(f'{l}_Start'), rec.get(f'{l}_End')
            if _pair_ok(t0, t1):
                row = self.sensors.index(l)
                for t_b in (t0, t1):
                    t_disp = float(t_b) - TIME_SHIFT[l]
                    self.ctx_vlines[l].append([
                        self.ctx_axs[row, 0].axvline(
                            t_disp, color='0.45', lw=1.0, ls='--', zorder=3,
                        ),
                        self.ctx_axs[row, 1].axvline(
                            t_disp, color='0.45', lw=1.0, ls='--', zorder=3,
                        ),
                    ])

            seg = self.segments[job_i][l]
            if seg is None:
                self.ctx_orig[(l, 'force')].set_offsets(_empty_xy())
                self.ctx_orig[(l, 'pos')].set_offsets(_empty_xy())
                self.ctx_ref[(l, 'force')].set_offsets(_empty_xy())
                self.ctx_ref[(l, 'pos')].set_offsets(_empty_xy())
                continue

            orig_idx = _cap_idx(np.arange(seg['n'], dtype=np.int64), _MAX_SEL_PTS)
            self.ctx_orig[(l, 'force')].set_offsets(
                np.column_stack((seg['t_disp'][orig_idx],
                                 self.full[l]['force'][seg['full_idx'][orig_idx]]))
            )
            self.ctx_orig[(l, 'pos')].set_offsets(
                np.column_stack((seg['t_disp'][orig_idx],
                                 self.full[l]['pos_mm'][seg['full_idx'][orig_idx]]))
            )

            sl = self._seg_slice(sel, l)
            if l in skipped or sl is None:
                self.ctx_ref[(l, 'force')].set_offsets(_empty_xy())
                self.ctx_ref[(l, 'pos')].set_offsets(_empty_xy())
            else:
                lo, hi = sl
                ref_idx = _cap_idx(
                    np.arange(lo, hi + 1, dtype=np.int64), _MAX_SEL_PTS,
                )
                self.ctx_ref[(l, 'force')].set_offsets(
                    np.column_stack((seg['t_disp'][ref_idx],
                                     self.full[l]['force'][seg['full_idx'][ref_idx]]))
                )
                self.ctx_ref[(l, 'pos')].set_offsets(
                    np.column_stack((seg['t_disp'][ref_idx],
                                     self.full[l]['pos_mm'][seg['full_idx'][ref_idx]]))
                )

    def frame_context_on_stalk(self):
        '''Zoom the context x-axis to this row's original spans + pad.'''
        rec = self.records[self.job_i]
        times = []
        for l in self.sensors:
            t0, t1 = rec.get(f'{l}_Start'), rec.get(f'{l}_End')
            if _pair_ok(t0, t1):
                times.extend([t0 - TIME_SHIFT[l], t1 - TIME_SHIFT[l]])
        if not times:
            t_lo = min(float(self.full[l]['t_disp'][0]) for l in self.sensors)
            t_hi = max(float(self.full[l]['t_disp'][-1]) for l in self.sensors)
        else:
            t_lo, t_hi = min(times), max(times)
            pad = max(0.4, 0.15 * (t_hi - t_lo))
            t_lo -= pad
            t_hi += pad
        self.ctx_axs[0, 0].set_xlim(t_lo, t_hi)
        self.update_context_base()

    def refresh_fp_plot(self, reset_view=True):
        job_i = self.job_i
        sel = self.selections[job_i]
        skipped = self.skipped_by_row[job_i]
        step = self.current_step()

        for l in self.sensors:
            seg = self.segments[job_i][l]
            self.fp_rings[l].set_offsets(_empty_xy())
            if seg is None:
                self.fp_base[l].set_offsets(_empty_xy())
                self.fp_base[l].set_array(np.empty(0))
                self.fp_sel[l].set_offsets(_empty_xy())
                continue
            self.fp_base[l].set_offsets(
                np.column_stack((seg['probe_m'], seg['force']))
            )
            t = np.asarray(seg['t_raw'], dtype=np.float64)
            self.fp_base[l].set_array(t)
            if t.size:
                t_lo = float(np.nanmin(t))
                t_hi = float(np.nanmax(t))
                if t_hi <= t_lo:
                    t_hi = t_lo + 1e-3
                self.fp_base[l].set_clim(t_lo, t_hi)
            sl = self._seg_slice(sel, l)
            if l in skipped or sl is None:
                self.fp_sel[l].set_offsets(_empty_xy())
            else:
                lo, hi = sl
                idx = np.arange(lo, hi + 1, dtype=np.int64)
                self.fp_sel[l].set_offsets(
                    np.column_stack((seg['probe_m'][idx], seg['force'][idx]))
                )
            if step is not None and step[0] == l and l not in skipped:
                which = step[1]
                idx = sel.get((l, which))
                if idx is not None:
                    self.fp_rings[l].set_offsets(
                        [[seg['probe_m'][idx], seg['force'][idx]]]
                    )
        if reset_view:
            self.fp_ax.relim()
            self.fp_ax.autoscale(enable=True)
            self.fp_ax.set_xlim(-1.0, 0.9)
            self.fp_ax.set_ylim(-1, 50)

    def refresh_context_rings(self):
        for l in self.sensors:
            self.ctx_rings[(l, 'force')].set_offsets(_empty_xy())
            self.ctx_rings[(l, 'pos')].set_offsets(_empty_xy())
        step = self.current_step()
        if step is None:
            return
        l, which = step
        if l in self.skipped_by_row[self.job_i]:
            return
        idx = self.selections[self.job_i].get((l, which))
        seg = self.segments[self.job_i][l]
        if idx is None or seg is None:
            return
        t = seg['t_disp'][idx]
        full_i = seg['full_idx'][idx]
        self.ctx_rings[(l, 'force')].set_offsets(
            [[t, self.full[l]['force'][full_i]]]
        )
        self.ctx_rings[(l, 'pos')].set_offsets(
            [[t, self.full[l]['pos_mm'][full_i]]]
        )

    def update_titles(self):
        rec = self.records[self.job_i]
        plot, stalk = rec['Plot'], rec['Stalk']
        step = self.current_step()
        mode = 'REFINE' if self.refine else 'VIEW'
        locked = 'confirmed' if self.job_i in self.confirmed else 'editing'
        if step is None:
            pick = 'no original spans on this row'
            bound = '—'
        else:
            l, which = step
            idx = self.selections[self.job_i].get((l, which))
            bound = f'{l}{which}'
            if l in self.skipped_by_row[self.job_i]:
                pick = 'skipped'
            elif idx is None:
                pick = 'not set'
            else:
                seg = self.segments[self.job_i][l]
                pick = (
                    f'idx={idx}/{seg["n"]-1}  '
                    f't={seg["t_raw"][idx]:.3f}s  '
                    f'x={seg["pos_m"][idx]:.4f}m  '
                    f'F={seg["force"][idx]:.2f}N'
                )
        self.fp_fig.suptitle(
            f"{mode}  [plot={plot}, stalk={stalk:02d}]  "
            f"{bound}  ({locked})  "
            f"{self.job_i+1}/{len(self.records)}",
            fontsize=11,
        )
        self.ctx_fig.suptitle(
            f"Context  [plot={plot}, stalk={stalk:02d}]  "
            f"gray = original span   black = refine span",
            fontsize=11,
        )
        if self.refine:
            self.status_text.set_text(
                f"{bound}  {pick}   • click {bound[0] if step else ''} F–P axes   "
                f"• ←/→ sample   • ↑/↓ bound   • x = skip sensor   "
                f"• enter = confirm   • q = finish"
            )
        else:
            self.status_text.set_text(
                f"[plot={plot}, stalk={stalk:02d}]  {pick}   "
                f"• ← prev   • → next   • q = close"
            )

    def redraw(self, reset_zoom_flag=True):
        self.refresh_context_overlays()
        self.refresh_context_rings()
        self.refresh_fp_plot(reset_view=reset_zoom_flag)
        self.update_titles()
        _draw_idle(self.ctx_fig, self.fp_fig)

    def goto(self, job_i, reset_step=True):
        self.job_i = int(max(0, min(len(self.records) - 1, job_i)))
        if reset_step:
            self.step_i = 0
        self.frame_context_on_stalk()
        self.redraw()

    # ----- bound edits -----

    def set_bound(self, idx):
        if not self.refine:
            return
        step = self.current_step()
        if step is None:
            return
        l, which = step
        if l in self.skipped_by_row[self.job_i]:
            return
        seg = self.segments[self.job_i][l]
        if seg is None:
            return
        idx = int(max(0, min(seg['n'] - 1, idx)))
        self.selections[self.job_i][(l, which)] = idx
        self.confirmed.discard(self.job_i)

    def on_fp_click(self, event):
        if not self.refine or not self.select_on:
            return
        if _click_blocked(event):
            return
        step = self.current_step()
        if step is None:
            return
        l, _which = step
        if event.inaxes is not self.fp_ax:
            return
        if l in self.skipped_by_row[self.job_i]:
            return
        seg = self.segments[self.job_i][l]
        if seg is None:
            return
        idx = _nearest_index_unsorted(
            event.inaxes, seg['probe_m'], seg['force'], event,
        )
        self.set_bound(idx)
        self.redraw()

    def nudge(self, delta):
        if not self.refine:
            self.goto(self.job_i + delta)
            return
        step = self.current_step()
        if step is None:
            return
        l, which = step
        cur = self.selections[self.job_i].get((l, which))
        if cur is None:
            cur = 0
        self.set_bound(cur + delta)
        self.redraw(reset_zoom_flag=False)

    def step_bound(self, delta):
        if not self.refine:
            return
        steps = self.current_steps()
        if not steps:
            return
        self.step_i = int(max(0, min(len(steps) - 1, self.step_i + delta)))
        self.redraw(reset_zoom_flag=False)

    def undo_bound(self, event=None):
        if not self.refine:
            return
        step = self.current_step()
        if step is None:
            return
        sel = self.selections[self.job_i]
        if step in sel:
            sel.pop(step)
            self.confirmed.discard(self.job_i)
        elif self.step_i > 0:
            self.step_i -= 1
            sel.pop(self.current_step(), None)
            self.confirmed.discard(self.job_i)
        self.redraw(reset_zoom_flag=False)

    def skip_sensor(self, event=None):
        '''Toggle: mark the current sensor empty for stiffness, or restore
        its full original span if it was already skipped.'''
        if not self.refine:
            return
        step = self.current_step()
        if step is None:
            return
        l, _which = step
        job_i = self.job_i
        seg = self.segments[job_i][l]
        self.confirmed.discard(job_i)
        if l in self.skipped_by_row[job_i]:
            self.skipped_by_row[job_i].discard(l)
            if seg is not None:
                self.selections[job_i][(l, 'min')] = 0
                self.selections[job_i][(l, 'max')] = seg['n'] - 1
        else:
            self.skipped_by_row[job_i].add(l)
            self.selections[job_i].pop((l, 'min'), None)
            self.selections[job_i].pop((l, 'max'), None)
            steps = self.current_steps()
            for k, (lab, _w) in enumerate(steps):
                if lab != l:
                    self.step_i = k
                    break
        self.redraw()

    def prev_stalk(self, event=None):
        if self.job_i <= 0:
            print("Already on the first [plot, stalk]")
            return
        self.goto(self.job_i - 1)

    def next_stalk(self, event=None):
        if self.job_i >= len(self.records) - 1:
            print("Already on the last [plot, stalk]")
            return
        self.goto(self.job_i + 1)

    def row_complete(self, job_i):
        '''Every sensor with an original span is either skipped or fully bound.'''
        sel = self.selections[job_i]
        skipped = self.skipped_by_row[job_i]
        for l in self.sensors:
            if self.segments[job_i][l] is None:
                continue
            if l in skipped:
                continue
            if (l, 'min') not in sel or (l, 'max') not in sel:
                return False
        # At least one sensor must remain if the row had any original data.
        usable = [
            l for l in self.sensors
            if self.segments[job_i][l] is not None and l not in skipped
        ]
        return True if usable or all(
            self.segments[job_i][l] is None for l in self.sensors
        ) else False

    def confirm_stalk(self, event=None):
        if not self.refine:
            self.next_stalk()
            return
        job_i = self.job_i
        rec = self.records[job_i]
        if not self.row_complete(job_i):
            missing = []
            for l in self.sensors:
                if self.segments[job_i][l] is None or l in self.skipped_by_row[job_i]:
                    continue
                if (l, 'min') not in self.selections[job_i]:
                    missing.append(f'{l}min')
                if (l, 'max') not in self.selections[job_i]:
                    missing.append(f'{l}max')
            print(f"[plot {rec['Plot']}, stalk {rec['Stalk']:02d}] "
                  f"incomplete — missing {missing}")
            return
        self.confirmed.add(job_i)
        print(f"Confirmed [plot {rec['Plot']}, stalk {rec['Stalk']:02d}] "
              f"({len(self.confirmed)}/{len(self.records)})")
        if job_i < len(self.records) - 1:
            self.goto(job_i + 1)
        else:
            print("Last [plot, stalk] confirmed — Finish & Save when ready")
            self.redraw()

    def apply_refine_to_records(self):
        '''Copy in-memory picks onto the record dicts that get written.'''
        for job_i, rec in enumerate(self.records):
            if job_i not in self.confirmed:
                continue
            skipped = self.skipped_by_row[job_i]
            sel = self.selections[job_i]
            for l in self.sensors:
                seg = self.segments[job_i][l]
                if seg is None or l in skipped:
                    rec[f'{l}_Refine_Start'] = None
                    rec[f'{l}_Refine_End'] = None
                    continue
                sl = self._seg_slice(sel, l)
                if sl is None:
                    rec[f'{l}_Refine_Start'] = None
                    rec[f'{l}_Refine_End'] = None
                    continue
                i0 = sel[(l, 'min')]
                i1 = sel[(l, 'max')]
                rec[f'{l}_Refine_Start'] = float(seg['t_raw'][i0])
                rec[f'{l}_Refine_End'] = float(seg['t_raw'][i1])

    def finish(self, event=None):
        if self.refine:
            self.apply_refine_to_records()
            write_stalk_rows(self.parent_data.stalks_csv_path, self.records)
            print(f"Status: wrote {len(self.records)} row(s) "
                  f"({len(self.confirmed)} confirmed this session) to "
                  f"{self.parent_data.stalks_csv_path}")
        plt.close(self.fp_fig)
        plt.close(self.ctx_fig)

    def toggle_select(self, event=None):
        if not self.refine:
            return
        self.select_on = not self.select_on
        _set_select_button(self.btn_select, self.select_on)
        self.update_titles()
        self.fp_fig.canvas.draw_idle()

    def on_key(self, event):
        key = event.key
        if self.refine:
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
            elif key == 'enter':
                self.confirm_stalk()
            elif key in ('p', 'P'):
                self.prev_stalk()
            elif key in ('x', 'X'):
                self.skip_sensor()
            elif key in ('q', 'Q'):
                self.finish()
            elif key in (' ', 'm', 'M'):
                self.toggle_select()
        else:
            if key in ('left', 'p', 'P'):
                self.prev_stalk()
            elif key in ('right', 'enter', 'n', 'N'):
                self.next_stalk()
            elif key in ('q', 'Q', 'escape'):
                self.finish()

    def _wire_ui(self):
        # Touch-friendly controls live on the stalk (F–P) figure.
        fp_fig = self.fp_fig
        if self.refine:
            self._buttons.extend([
                _add_button(fp_fig, [0.02, 0.012, 0.10, 0.045],
                            '← Prev Stalk', self.prev_stalk),
                _add_button(fp_fig, [0.13, 0.012, 0.10, 0.045],
                            'Undo Bound', self.undo_bound),
                _add_button(fp_fig, [0.24, 0.012, 0.12, 0.045],
                            'Skip Sensor', self.skip_sensor),
                _add_button(fp_fig, [0.37, 0.012, 0.13, 0.045],
                            'Confirm Stalk', self.confirm_stalk),
                _add_button(fp_fig, [0.51, 0.012, 0.13, 0.045],
                            'Finish & Save', self.finish),
            ])
            self.btn_select = _add_button(
                fp_fig, [0.65, 0.012, 0.13, 0.045],
                'Select: ON', self.toggle_select, color='lightgreen',
            )
            self._buttons.append(self.btn_select)
        else:
            self._buttons.extend([
                _add_button(fp_fig, [0.02, 0.012, 0.12, 0.045],
                            '← Prev', self.prev_stalk),
                _add_button(fp_fig, [0.16, 0.012, 0.12, 0.045],
                            'Next →', self.next_stalk),
                _add_button(fp_fig, [0.30, 0.012, 0.12, 0.045],
                            'Close', self.finish),
            ])

        _bind_canvas(fp_fig, on_click=self.on_fp_click, on_key=self.on_key)
        _bind_canvas(self.ctx_fig, on_key=self.on_key)
        self.ctx_axs[0, 0].callbacks.connect(
            'xlim_changed', self.update_context_base,
        )

    def _initial_view(self):
        for i, l in enumerate(self.sensors):
            self.ctx_axs[i, 0].set_ylim(
                np.nanmin(self.full[l]['force']) - 1.0,
                np.nanmax(self.full[l]['force']) + 1.0,
            )
            self.ctx_axs[i, 1].set_ylim(
                min(0.0, float(np.nanmin(self.full[l]['pos_mm']))),
                float(np.nanmax(self.full[l]['pos_mm'])) * 1.05 + 1.0,
            )
        self.goto(0)


def display_stalk_selections(parent_data):
    '''Page through saved stalk rows. No edits, no CSV write.

    Figure 1 (context): full A/B/C force and position vs shifted time,
    original span highlighted, refine span overlaid when present.
    Figure 2 (stalk): force vs probe position for this row only.
    Walks CSV order — high→low plot, low→high stalk, matching labeling.
    '''
    ReviewSession(parent_data, refine=False).run()


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
    ReviewSession(parent_data, refine=True).run()
