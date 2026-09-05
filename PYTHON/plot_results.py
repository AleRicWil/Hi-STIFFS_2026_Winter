"""Plot EI estimates from one or more ``save_stiffnesses()`` result CSVs.

Two figure families, three estimates each (AB, CB, AB-CB):

**Overlay** — every Hi-STIFFS run as a scatter, with matching
``(Plot, Stalk)`` pairs sharing an x location. Optional DARLING
replicates overlay as a red x (per-stalk mean) with ±3σ error bars.

**Median comparison** — per-stalk median Hi-STIFFS vs median DARLING.
Optional ±3σ error crosses, 1:1 overlap markers, and a subtitle that
counts overlap classes and which device has the larger sample SD.

**Median vs SD** — per-stalk sample SD against median EI for both
devices. A 10% CV line and each device's pooled sample SD and pooled
median-based CV show whether scatter grows with stiffness and how
the two instruments compare.

Edit ``RESULT_CSVS`` / ``DARLING_CSV`` in ``__main__``, or pass paths on
the command line. Stack matches ``process.py``: pathlib, csv, numpy,
matplotlib.
"""

import argparse
import csv
from collections import defaultdict
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt


# ---------------------------------------------------------------------------
# Column names / knobs
# ---------------------------------------------------------------------------

# Written by HiSTIFFSData.save_stiffnesses().
ESTIMATE_COLS = (
    'Estimate AB (N*m^2)',
    'Estimate CB (N*m^2)',
    'Estimate AB-CB (N*m^2)',
)
ESTIMATE_TITLES = (
    'Estimate AB',
    'Estimate CB',
    'Estimate AB-CB',
)

# Empty stalk-index units inserted between plot groups on the overlay x-axis.
PLOT_GAP = 2

# Error bars / crosses: mean or median ± this many sample standard deviations.
DARLING_SIGMA = 3.0

# |sd_h - sd_d| / max(sd_h, sd_d) below this counts as equal SDs.
SD_EQUAL_FRAC = 0.10

# Reference line on SD-vs-median figures: SD = CV_REF * median.
CV_REF = 0.10

# ylim / axis upper bound = data max * this.
_AXIS_PAD = 1.1


# ---------------------------------------------------------------------------
# CSV parsing
# ---------------------------------------------------------------------------

def _to_float(value):
    """Blank / 'NaN' / junk → np.nan. Matches save_stiffnesses() tokens."""
    if value is None:
        return np.nan
    text = str(value).strip()
    if text == '' or text.upper() == 'NAN':
        return np.nan
    try:
        return float(text)
    except (TypeError, ValueError):
        return np.nan


def _to_int(value):
    """Finite number → int, otherwise None."""
    number = _to_float(value)
    if not np.isfinite(number):
        return None
    return int(number)


def _parse_stalk(value):
    """Stalk token → int. Accepts 1, '01', 's01', 'S01'."""
    if value is None:
        return None
    text = str(value).strip()
    if text[:1].lower() == 's':
        text = text[1:]
    return _to_int(text)


def _read_csv_rows(csv_path, kind='results'):
    """Return ``(Path, list[row])``. Raises if the file is missing."""
    csv_path = Path(csv_path)
    if not csv_path.exists():
        raise FileNotFoundError(f'No {kind} CSV at {csv_path}')
    with open(csv_path, 'r', newline='') as f:
        return csv_path, list(csv.reader(f))


def _plot_stalk_header(rows, csv_path):
    """Index and stripped header of the first row that has Plot and Stalk."""
    for i, row in enumerate(rows):
        cells = [c.strip() for c in row]
        if 'Plot' in cells and 'Stalk' in cells:
            return i, [h.strip() for h in rows[i]]
    raise ValueError(f'No Plot/Stalk header in {csv_path}')


def _is_blank_row(row):
    return not row or all(not str(c).strip() for c in row)


def _cell(row, columns, name):
    """Value of *name* in *row*, or '' if the row is short."""
    j = columns[name]
    return row[j] if j < len(row) else ''


def load_stiffness_csv(csv_path):
    """Read one results file written by ``save_stiffnesses()``.

    Skips the preamble (note / method / height rows) and starts at the
    header that contains Plot and Stalk.

    Returns
    -------
    list[dict]
        One dict per data row with ``Plot``, ``Stalk``, ``source``,
        ``path``, and the three EI columns as floats (NaN if missing).
    """
    csv_path, rows = _read_csv_rows(csv_path, kind='results')
    header_i, header = _plot_stalk_header(rows, csv_path)
    columns = {name: j for j, name in enumerate(header)}
    missing = [name for name in ('Plot', 'Stalk') + ESTIMATE_COLS if name not in columns]
    if missing:
        raise ValueError(f'{csv_path} is missing columns: {missing}')

    records = []
    for raw in rows[header_i + 1:]:
        if _is_blank_row(raw):
            continue
        plot_n = _to_int(_cell(raw, columns, 'Plot'))
        stalk_n = _to_int(_cell(raw, columns, 'Stalk'))
        if plot_n is None or stalk_n is None:
            continue
        rec = {
            'Plot': plot_n,
            'Stalk': stalk_n,
            'source': csv_path.name,
            'path': str(csv_path),
        }
        for name in ESTIMATE_COLS:
            rec[name] = _to_float(_cell(raw, columns, name))
        records.append(rec)
    return records


def load_darling_csv(csv_path):
    """Read DARLING replicates from a wide ``stiffnesses.csv``.

    Header must contain Plot and Stalk. Every ``Test_*`` column is one
    replicate. Stalk tokens like ``s01`` map to integer 1.

    Returns
    -------
    list[dict]
        Flat list of ``{Plot, Stalk, y, source, path}``, one per finite
        test value.
    """
    csv_path, rows = _read_csv_rows(csv_path, kind='DARLING')
    header_i, header = _plot_stalk_header(rows, csv_path)
    columns = {name: j for j, name in enumerate(header)}
    test_cols = [name for name in header if name.startswith('Test_')]
    if not test_cols:
        raise ValueError(f'{csv_path} has no Test_* columns')

    records = []
    for raw in rows[header_i + 1:]:
        if _is_blank_row(raw):
            continue
        plot_n = _to_int(_cell(raw, columns, 'Plot'))
        stalk_n = _parse_stalk(_cell(raw, columns, 'Stalk'))
        if plot_n is None or stalk_n is None:
            continue
        for name in test_cols:
            y = _to_float(_cell(raw, columns, name))
            if not np.isfinite(y):
                continue
            records.append({
                'Plot': plot_n,
                'Stalk': stalk_n,
                'y': y,
                'source': csv_path.name,
                'path': str(csv_path),
            })
    return records


# ---------------------------------------------------------------------------
# Per-stalk statistics
# ---------------------------------------------------------------------------

def _group_finite(records, value_fn):
    """``(Plot, Stalk) -> list of finite value_fn(rec)``."""
    grouped = defaultdict(list)
    for rec in records:
        y = value_fn(rec)
        if y is None or not np.isfinite(y):
            continue
        grouped[(rec['Plot'], rec['Stalk'])].append(float(y))
    return grouped


def _col_getter(col_name):
    """``rec -> rec[col_name]``, bound now so loop closures stay correct."""
    return lambda rec, name=col_name: rec[name]


def _median_by_key(records, value_fn):
    """``(Plot, Stalk) -> median`` of finite values from *value_fn*."""
    return {
        key: float(np.median(vals))
        for key, vals in _group_finite(records, value_fn).items()
    }


def _std_by_key(records, value_fn):
    """``(Plot, Stalk) -> sample std`` (ddof=1). A single replicate → 0."""
    out = {}
    for key, vals in _group_finite(records, value_fn).items():
        out[key] = float(np.std(vals, ddof=1)) if len(vals) >= 2 else 0.0
    return out


def _median_sd_pairs(records, value_fn):
    """``(median, sd)`` for stalks with at least two finite replicates."""
    pairs = []
    for vals in _group_finite(records, value_fn).values():
        if len(vals) < 2:
            continue
        pairs.append((float(np.median(vals)), float(np.std(vals, ddof=1))))
    return pairs


def _pooled_sd(records, value_fn):
    """Pooled sample SD over stalks with n ≥ 2.

    ``sqrt(Σ (n_i-1) SD_i² / Σ (n_i-1))``. Equal n=10 on 30 stalks is
    ``sqrt(Σ 9 SD_i² / 270)``.
    """
    num = 0.0
    den = 0.0
    for vals in _group_finite(records, value_fn).values():
        n = len(vals)
        if n < 2:
            continue
        sd = float(np.std(vals, ddof=1))
        num += (n - 1) * sd * sd
        den += (n - 1)
    if den == 0.0:
        return np.nan
    return float(np.sqrt(num / den))


def _pooled_cv(records, value_fn):
    """Pooled median-based CV over stalks with n ≥ 2 and median > 0.

    ``CV_i = SD_i / median_i``, then
    ``sqrt(Σ (n_i-1) CV_i² / Σ (n_i-1))``.
    """
    num = 0.0
    den = 0.0
    for vals in _group_finite(records, value_fn).values():
        n = len(vals)
        if n < 2:
            continue
        med = float(np.median(vals))
        if med <= 0:
            continue
        sd = float(np.std(vals, ddof=1))
        cv = sd / med
        num += (n - 1) * cv * cv
        den += (n - 1)
    if den == 0.0:
        return np.nan
    return float(np.sqrt(num / den))


def darling_mean_and_err(darling_recs, sigma_level=DARLING_SIGMA):
    """Per ``(Plot, Stalk)`` mean and ``sigma_level * sample std``.

    A single replicate has no std, so the error is 0 and the marker is
    just the observed value.
    """
    stats = []
    for key, ys in _group_finite(darling_recs, _col_getter('y')).items():
        arr = np.asarray(ys, dtype=float)
        mean = float(np.mean(arr))
        if arr.size >= 2:
            err = float(sigma_level * np.std(arr, ddof=1))
        else:
            err = 0.0
        stats.append((key, mean, err))
    return stats


def _sd_relation(sd_h, sd_d, frac=SD_EQUAL_FRAC):
    """Which sample SD is larger, or ``'equal'``.

    Equal means both are zero, or they differ by less than *frac* of the
    larger SD: ``|sd_h - sd_d| / max(sd_h, sd_d) < frac``.
    """
    bigger = max(sd_h, sd_d)
    if bigger == 0.0 or abs(sd_h - sd_d) / bigger < frac:
        return 'equal'
    return 'histiffs' if sd_h > sd_d else 'darling'


def _count_sd_relations(keys, hs_sd, darling_sd):
    """Return ``{equal, histiffs, darling}`` counts over *keys*."""
    counts = {'equal': 0, 'histiffs': 0, 'darling': 0}
    for key in keys:
        rel = _sd_relation(hs_sd.get(key, 0.0), darling_sd.get(key, 0.0))
        counts[rel] += 1
    return counts


def _n_arms_on_one_to_one(x, y, xerr, yerr):
    """How many error-cross arms reach ``y = x`` (0, 1, or 2).

    DARLING (horizontal) reaches the line when ``|y - x| <= xerr``.
    Hi-STIFFS (vertical) reaches the line when ``|y - x| <= yerr``.
    """
    delta = abs(y - x)
    return int(delta <= xerr) + int(delta <= yerr)


# ---------------------------------------------------------------------------
# Figure helpers
# ---------------------------------------------------------------------------

def stalk_x_positions(keys):
    """Map each unique ``(plot, stalk)`` to a single x coordinate.

    Within a plot, x follows stalk number. Between plots a ``PLOT_GAP``
    is left empty so groups do not run together.
    """
    plots = sorted({p for p, _k in keys})
    max_stalk = max((k for _p, k in keys), default=1)
    x_of = {}
    cursor = 0.0
    tick_x = []
    tick_label = []
    for plot_n in plots:
        stalks = sorted(k for p, k in keys if p == plot_n)
        for stalk_n in stalks:
            x = cursor + float(stalk_n)
            x_of[(plot_n, stalk_n)] = x
            tick_x.append(x)
            tick_label.append(f'P{plot_n}-S{stalk_n:02d}')
        cursor = cursor + float(max_stalk) + float(PLOT_GAP)
    return x_of, tick_x, tick_label


def _cycle_colors():
    return plt.rcParams['axes.prop_cycle'].by_key()['color']


def _title_slug(title):
    """``'Estimate AB-CB'`` → ``'estimate_abcb'``."""
    return title.lower().replace(' ', '_').replace('-', '')


def _save_figure(fig, save_dir, filename):
    if save_dir is None:
        return
    out_dir = Path(save_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    out = out_dir / filename
    fig.savefig(out, dpi=150)
    print(f'Wrote {out}')


def _show_if(show):
    if show:
        plt.show(block=True)


def _apply_title_with_sublines(ax, main, sub_lines):
    """Main title plus smaller subtitle rows just above the axes."""
    ax.set_title(main, pad=14 + 12 * len(sub_lines))
    if not sub_lines:
        return
    ax.text(
        0.5, 1.0,
        '\n'.join(sub_lines),
        transform=ax.transAxes,
        ha='center',
        va='bottom',
        fontsize=9,
        color='0.3',
    )


def _keys_by_plot(keys):
    """``plot_n -> list of (plot, stalk) keys``, plots sorted at use site."""
    by_plot = defaultdict(list)
    for key in keys:
        by_plot[key[0]].append(key)
    return by_plot


# ---------------------------------------------------------------------------
# Overlay figures
# ---------------------------------------------------------------------------

def plot_stiffnesses(csv_paths, show=True, save_dir=None, darling_csv=None):
    """Scatter every loaded EI value onto three figures (one per estimate).

    Parameters
    ----------
    csv_paths : sequence of path-like
        Results files from ``save_stiffnesses()``. Same ``(Plot, Stalk)``
        in different files share an x location and are colored by file.
    show : bool
        ``plt.show()`` at the end (block=True).
    save_dir : path-like or None
        If set, write ``estimate_ab.png`` / ``estimate_cb.png`` /
        ``estimate_abcb.png``.
    darling_csv : path-like or None
        Optional DARLING ``stiffnesses.csv``. Per-stalk mean is drawn
        behind the other series as a red x with ±3σ error bars.
    """
    csv_paths = [Path(p) for p in csv_paths]
    if not csv_paths:
        raise ValueError('No results CSV paths given')

    by_source = []
    all_keys = set()
    for path in csv_paths:
        recs = load_stiffness_csv(path)
        print(f'Loaded {len(recs)} row(s) from {path}')
        by_source.append((path.name, recs))
        for rec in recs:
            all_keys.add((rec['Plot'], rec['Stalk']))

    darling_recs = []
    if darling_csv is not None:
        darling_recs = load_darling_csv(darling_csv)
        print(f'Loaded {len(darling_recs)} DARLING replicate(s) from {darling_csv}')
        for rec in darling_recs:
            all_keys.add((rec['Plot'], rec['Stalk']))

    if not all_keys:
        raise ValueError('No Plot/Stalk rows in the given CSVs')

    x_of, tick_x, tick_label = stalk_x_positions(all_keys)
    colors = _cycle_colors()
    figs = []
    # Shared across the three figures so later panels keep the same ylim.
    max_stiff = 0.0

    for col_name, title in zip(ESTIMATE_COLS, ESTIMATE_TITLES):
        fig, ax = plt.subplots(figsize=(max(8.0, 0.35 * len(tick_x) + 2.0), 5.5))

        if darling_recs:
            dxs, dys, derr = [], [], []
            for key, mean, err in darling_mean_and_err(darling_recs):
                dxs.append(x_of[key])
                dys.append(mean)
                derr.append(err)
                max_stiff = max(max_stiff, mean + err)
            ax.errorbar(
                dxs, dys, yerr=derr,
                fmt='x',
                color='red',
                markersize=6,
                markeredgewidth=0.9,
                elinewidth=0.9,
                capsize=3,
                capthick=0.9,
                alpha=0.65,
                zorder=2,
                label='DARLING',
            )

        for i, (label, recs) in enumerate(by_source):
            xs, ys = [], []
            for rec in recs:
                y = rec[col_name]
                if not np.isfinite(y):
                    continue
                xs.append(x_of[(rec['Plot'], rec['Stalk'])])
                ys.append(y)
                max_stiff = max(max_stiff, y)
            ax.scatter(
                xs, ys,
                s=10,
                c=colors[i % len(colors)],
                marker='o',
                alpha=0.85,
                zorder=3,
                label=label,
            )

        ax.set_title(f'{title}  (EI, N·m²)')
        ax.set_xlabel('Stalk')
        ax.set_ylabel('EI (N·m²)')
        ax.set_xticks(tick_x)
        ax.set_xticklabels(tick_label, rotation=75, ha='right', fontsize=8)
        ax.set_ylim(0, max_stiff * _AXIS_PAD)
        ax.grid(True, axis='y', alpha=0.3)
        if len(by_source) > 1 or darling_recs:
            ax.legend(loc='best', fontsize=8)
        fig.tight_layout()
        figs.append(fig)
        _save_figure(fig, save_dir, f'{_title_slug(title)}.png')

    _show_if(show)
    return figs


# ---------------------------------------------------------------------------
# Median-comparison figures
# ---------------------------------------------------------------------------

def plot_median_comparison(csv_paths, darling_csv, show=True, save_dir=None,
                          error_crosses=False):
    """Scatter per-stalk median Hi-STIFFS vs median DARLING.

    Three figures, one per estimate. x is the DARLING median of
    ``Test_*`` replicates; y is the Hi-STIFFS median of that estimate
    across the loaded result CSVs. A stalk is drawn only when both
    medians exist.

    If *error_crosses* is True, each point also gets a ±3σ cross
    (DARLING horizontal, Hi-STIFFS vertical). An arm overlaps the 1:1
    line when ``|y - x|`` is within that device's 3σ. Extra markers:
    blue x if only one device overlaps, red x if neither does.

    Per-stalk sample SDs are compared too: if they differ by less than
    10% of the larger SD they count as equal, otherwise the larger
    device is counted. Those counts go on a subtitle row.
    """
    csv_paths = [Path(p) for p in csv_paths]
    if not csv_paths:
        raise ValueError('No results CSV paths given')
    if darling_csv is None:
        print('Skipping median comparison: no DARLING CSV given')
        return []

    darling_csv = Path(darling_csv)
    if not darling_csv.exists():
        print(f'Skipping median comparison: no DARLING CSV at {darling_csv}')
        return []

    histiffs_recs = []
    for path in csv_paths:
        histiffs_recs.extend(load_stiffness_csv(path))

    darling_recs = load_darling_csv(darling_csv)
    print(
        f'Median comparison: {len(histiffs_recs)} Hi-STIFFS row(s), '
        f'{len(darling_recs)} DARLING replicate(s)'
    )
    darling_med = _median_by_key(darling_recs, _col_getter('y'))
    if not darling_med:
        print('Skipping median comparison: no finite DARLING values')
        return []

    darling_sd = _std_by_key(darling_recs, _col_getter('y'))
    colors = _cycle_colors()
    figs = []

    for col_name, title in zip(ESTIMATE_COLS, ESTIMATE_TITLES):
        getter = _col_getter(col_name)
        hs_med = _median_by_key(histiffs_recs, getter)
        keys = sorted(set(hs_med) & set(darling_med))
        if not keys:
            print(f'Skipping {title} median comparison: no paired stalks')
            continue

        hs_sd = _std_by_key(histiffs_recs, getter)
        sd_counts = _count_sd_relations(keys, hs_sd, darling_sd)
        by_plot = _keys_by_plot(keys)

        xs_all = [darling_med[k] for k in keys]
        ys_all = [hs_med[k] for k in keys]
        hi = max(max(xs_all), max(ys_all)) * _AXIS_PAD

        xerr = yerr = None
        if error_crosses:
            xerr = [DARLING_SIGMA * darling_sd.get(k, 0.0) for k in keys]
            yerr = [DARLING_SIGMA * hs_sd.get(k, 0.0) for k in keys]
            hi = max(
                hi,
                max(x + e for x, e in zip(xs_all, xerr)) * _AXIS_PAD,
                max(y + e for y, e in zip(ys_all, yerr)) * _AXIS_PAD,
            )
        if hi <= 0:
            hi = 1.0

        fig, ax = plt.subplots(figsize=(6.5, 6.5))
        ax.plot([0.0, hi], [0.0, hi], color='0.5', lw=1.0, zorder=1, label='1:1')

        if error_crosses:
            ax.errorbar(
                xs_all, ys_all, xerr=xerr, yerr=yerr,
                fmt='none',
                ecolor='0.45',
                elinewidth=0.8,
                capsize=2,
                capthick=0.8,
                alpha=0.65,
                zorder=2,
            )

        for i, plot_n in enumerate(sorted(by_plot)):
            xs = [darling_med[k] for k in by_plot[plot_n]]
            ys = [hs_med[k] for k in by_plot[plot_n]]
            ax.scatter(
                xs, ys,
                s=28,
                c=colors[i % len(colors)],
                marker='o',
                alpha=0.85,
                zorder=3,
                label=f'Plot {plot_n}',
            )

        n_one = n_neither = 0
        if error_crosses:
            blue_x, blue_y, red_x, red_y = [], [], [], []
            for x, y, ex, ey in zip(xs_all, ys_all, xerr, yerr):
                n_overlap = _n_arms_on_one_to_one(x, y, ex, ey)
                if n_overlap == 1:
                    blue_x.append(x)
                    blue_y.append(y)
                elif n_overlap == 0:
                    red_x.append(x)
                    red_y.append(y)
            n_one, n_neither = len(blue_x), len(red_x)
            if blue_x:
                ax.scatter(
                    blue_x, blue_y,
                    s=42, c='blue', marker='x', linewidths=1.2,
                    zorder=4, label='one device overlaps 1:1',
                )
            if red_x:
                ax.scatter(
                    red_x, red_y,
                    s=42, c='red', marker='x', linewidths=1.2,
                    zorder=4, label='neither overlaps 1:1',
                )

        ax.set_xlim(0.0, hi)
        ax.set_ylim(0.0, hi)
        ax.set_aspect('equal', adjustable='box')

        sub_lines = []
        if error_crosses:
            n_both = len(keys) - n_one - n_neither
            sub_lines.append(
                f'both overlap: {n_both}    '
                f'one overlap: {n_one}    '
                f'neither: {n_neither}'
            )
        sub_lines.append(
            f'SD equal: {sd_counts["equal"]}    '
            f'Hi-STIFFS larger: {sd_counts["histiffs"]}    '
            f'DARLING larger: {sd_counts["darling"]}'
        )
        _apply_title_with_sublines(
            ax, f'{title}  (median vs DARLING median)', sub_lines,
        )
        ax.set_xlabel('DARLING median EI (N·m²)')
        ax.set_ylabel(f'{title} median EI (N·m²)')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='best', fontsize=8)
        fig.tight_layout()
        figs.append(fig)

        slug = _title_slug(title).replace('estimate_', '')
        _save_figure(fig, save_dir, f'median_{slug}.png')

    _show_if(show)
    return figs


# ---------------------------------------------------------------------------
# Median vs SD (robust CV)
# ---------------------------------------------------------------------------

def _fmt_sd_summary(name, pooled_sd, pooled_cv):
    """One subtitle clause: pooled sample SD and pooled median-based CV."""
    sd_s = f'{pooled_sd:.2f}' if np.isfinite(pooled_sd) else 'n/a'
    cv_s = f'{pooled_cv:.3f}' if np.isfinite(pooled_cv) else 'n/a'
    return f'{name} pooled SD: {sd_s}    pooled CV: {cv_s}'


def _draw_median_sd_series(ax, pairs, color, label):
    """Scatter of per-stalk (median, SD) for one device."""
    if not pairs:
        return
    ax.scatter(
        [med for med, _sd in pairs],
        [sd for _med, sd in pairs],
        s=28,
        c=color,
        marker='o',
        alpha=0.85,
        zorder=3,
        label=label,
    )


def plot_median_vs_sd(csv_paths, darling_csv, show=True, save_dir=None):
    """Scatter per-stalk sample SD against median EI, both devices.

    Three figures, one per Hi-STIFFS estimate. x is the median of that
    stalk's replicates; y is the sample SD. DARLING uses the ``Test_*``
    replicates and is the same on every figure. A gray dashed line
    marks 10% CV (``SD = 0.10 * median``). Each device also gets a
    dashed horizontal at its pooled sample SD and a dashed diagonal
    at its pooled median-based CV (``CV_i = SD_i / median_i``). The
    subtitle reports those pooled values.

    Stalks with fewer than two finite replicates are omitted. Stalks
    with median ≤ 0 are omitted from pooled CV.
    """
    csv_paths = [Path(p) for p in csv_paths]
    if not csv_paths:
        raise ValueError('No results CSV paths given')
    if darling_csv is None:
        print('Skipping median vs SD: no DARLING CSV given')
        return []

    darling_csv = Path(darling_csv)
    if not darling_csv.exists():
        print(f'Skipping median vs SD: no DARLING CSV at {darling_csv}')
        return []

    histiffs_recs = []
    for path in csv_paths:
        histiffs_recs.extend(load_stiffness_csv(path))

    darling_recs = load_darling_csv(darling_csv)
    darling_getter = _col_getter('y')
    darling_pairs = _median_sd_pairs(darling_recs, darling_getter)
    darling_pooled = _pooled_sd(darling_recs, darling_getter)
    darling_pooled_cv = _pooled_cv(darling_recs, darling_getter)
    print(
        f'Median vs SD: {len(histiffs_recs)} Hi-STIFFS row(s), '
        f'{len(darling_recs)} DARLING replicate(s), '
        f'{len(darling_pairs)} DARLING stalk(s) with n≥2'
    )

    hs_color = _cycle_colors()[0]
    figs = []
    for col_name, title in zip(ESTIMATE_COLS, ESTIMATE_TITLES):
        getter = _col_getter(col_name)
        hs_pairs = _median_sd_pairs(histiffs_recs, getter)
        hs_pooled = _pooled_sd(histiffs_recs, getter)
        hs_pooled_cv = _pooled_cv(histiffs_recs, getter)
        if not hs_pairs and not darling_pairs:
            print(f'Skipping {title} median vs SD: no stalks with n≥2')
            continue

        fig, ax = plt.subplots(figsize=(6.5, 6.5))
        _draw_median_sd_series(ax, hs_pairs, hs_color, 'Hi-STIFFS')
        _draw_median_sd_series(ax, darling_pairs, 'red', 'DARLING')

        meds = [med for med, _sd in hs_pairs + darling_pairs]
        sds = [sd for _med, sd in hs_pairs + darling_pairs]
        x_hi = max(meds) * _AXIS_PAD if meds else 1.0
        y_hi = max(sds) * _AXIS_PAD if sds else 1.0
        if x_hi <= 0:
            x_hi = 1.0
        if y_hi <= 0:
            y_hi = 1.0
        y_hi = max(y_hi, CV_REF * x_hi)
        ax.plot(
            [0.0, x_hi], [0.0, CV_REF * x_hi],
            color='0.5', ls='--', lw=1.0, zorder=1, label='10% CV',
        )
        for pooled, color in ((hs_pooled, hs_color), (darling_pooled, 'red')):
            if np.isfinite(pooled):
                ax.plot(
                    [0.0, x_hi], [pooled, pooled],
                    color=color, ls='--', lw=1.0, zorder=1,
                )
        for cv, color in ((hs_pooled_cv, hs_color), (darling_pooled_cv, 'red')):
            if np.isfinite(cv):
                ax.plot(
                    [0.0, x_hi], [0.0, cv * x_hi],
                    color=color, ls='--', lw=1.0, zorder=1,
                )
                y_hi = max(y_hi, cv * x_hi)
        ax.set_xlim(0.0, x_hi)
        ax.set_ylim(0.0, y_hi)
        ax.set_xlabel('Median EI (N·m²)')
        ax.set_ylabel('Sample SD (N·m²)')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='best', fontsize=8)
        _apply_title_with_sublines(
            ax,
            f'{title}  (SD vs median)',
            [
                _fmt_sd_summary('Hi-STIFFS', hs_pooled, hs_pooled_cv),
                _fmt_sd_summary('DARLING', darling_pooled, darling_pooled_cv),
            ],
        )
        fig.tight_layout()
        figs.append(fig)
        slug = _title_slug(title).replace('estimate_', '')
        _save_figure(fig, save_dir, f'sd_vs_median_{slug}.png')

    _show_if(show)
    return figs


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _parse_args():
    parser = argparse.ArgumentParser(
        description='Scatter EI estimates from save_stiffnesses() CSVs.',
    )
    parser.add_argument(
        'csvs',
        nargs='*',
        help='Results CSV path(s). If omitted, RESULT_CSVS in __main__ is used.',
    )
    parser.add_argument(
        '--save-dir',
        default=None,
        help='Optional directory to write PNG figures (overlay, median, SD).',
    )
    parser.add_argument(
        '--no-show',
        action='store_true',
        help='Skip plt.show() (use with --save-dir).',
    )
    parser.add_argument(
        '--darling',
        default=None,
        help='DARLING stiffnesses.csv. If omitted, DARLING_CSV in __main__ is used.',
    )
    return parser.parse_args()


if __name__ == '__main__':
    # Edit this list when you are not passing paths on the command line.
    # Each file is one scatter series; matching (Plot, Stalk) share an x.
    RESULT_CSVS = [
        r'Hi-STIFFS_2026_Winter\Results\2026-08-28\104213_stiffnesses.csv',
        r'Hi-STIFFS_2026_Winter\Results\2026-08-28\110545_stiffnesses.csv',
        r'Hi-STIFFS_2026_Winter\Results\2026-08-28\112006_stiffnesses.csv',
        r'Hi-STIFFS_2026_Winter\Results\2026-08-28\113413_stiffnesses.csv',
        r'Hi-STIFFS_2026_Winter\Results\2026-08-28\114710_stiffnesses.csv',
        r'Hi-STIFFS_2026_Winter\Results\2026-08-28\120712_stiffnesses.csv',
        r'Hi-STIFFS_2026_Winter\Results\2026-08-28\122508_stiffnesses.csv',
        r'Hi-STIFFS_2026_Winter\Results\2026-08-28\123851_stiffnesses.csv',
        r'Hi-STIFFS_2026_Winter\Results\2026-08-28\125311_stiffnesses.csv',
        r'Hi-STIFFS_2026_Winter\Results\2026-08-28\130654_stiffnesses.csv',
    ]
    DARLING_CSV = r'stiffnesses.csv'

    args = _parse_args()
    paths = [Path(p) for p in (args.csvs if args.csvs else RESULT_CSVS)]
    darling_path = Path(args.darling) if args.darling else Path(DARLING_CSV)
    if not paths:
        print('Status: add paths to RESULT_CSVS or pass them as arguments')
    else:
        plot_stiffnesses(
            paths,
            show=False,
            save_dir=args.save_dir,
            darling_csv=darling_path,
        )
        plot_median_comparison(
            paths,
            darling_path,
            show=False,
            save_dir=args.save_dir,
            error_crosses=True,
        )
        plot_median_vs_sd(
            paths,
            darling_path,
            show=not args.no_show,
            save_dir=args.save_dir,
        )
