# Plot EI estimates from one or more save_stiffnesses() result CSVs.
#
# Each (Plot, Stalk) pair owns one x location so runs overlay on the
# same stalks. Three figures: Estimate AB, Estimate CB, Estimate AB-CB.
#
# Cross-platform: pathlib + matplotlib. Same stack as process.py.
# Edit RESULT_CSVS in __main__ (or pass paths on the command line).

import argparse
import csv
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt


# Column names written by HiSTIFFSData.save_stiffnesses().
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

# Visual gap (in stalk-index units) inserted between plots on the x-axis
# so plot groups stay readable when several plots share a figure.
PLOT_GAP = 2


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
    number = _to_float(value)
    if not np.isfinite(number):
        return None
    return int(number)


def load_stiffness_csv(csv_path):
    """Read one results file written by save_stiffnesses().

    Skips the preamble (note / method / height rows) and starts at the
    header that contains Plot and Stalk. Returns a list of dicts with
    Plot, Stalk, and the three EI columns as floats (NaN if missing).
    """
    csv_path = Path(csv_path)
    if not csv_path.exists():
        raise FileNotFoundError(f'No results CSV at {csv_path}')

    with open(csv_path, 'r', newline='') as f:
        rows = list(csv.reader(f))

    header_i = None
    for i, row in enumerate(rows):
        cells = [c.strip() for c in row]
        if 'Plot' in cells and 'Stalk' in cells:
            header_i = i
            break
    if header_i is None:
        raise ValueError(f'No Plot/Stalk header in {csv_path}')

    header = [h.strip() for h in rows[header_i]]
    col = {name: j for j, name in enumerate(header)}
    missing = [name for name in ('Plot', 'Stalk') + ESTIMATE_COLS if name not in col]
    if missing:
        raise ValueError(f'{csv_path} is missing columns: {missing}')

    records = []
    for raw in rows[header_i + 1:]:
        if not raw or all(not str(c).strip() for c in raw):
            continue

        def cell(name):
            j = col[name]
            return raw[j] if j < len(raw) else ''

        plot_n = _to_int(cell('Plot'))
        stalk_n = _to_int(cell('Stalk'))
        if plot_n is None or stalk_n is None:
            continue
        rec = {
            'Plot': plot_n,
            'Stalk': stalk_n,
            'source': csv_path.name,
            'path': str(csv_path),
        }
        for name in ESTIMATE_COLS:
            rec[name] = _to_float(cell(name))
        records.append(rec)
    return records


def stalk_x_positions(keys):
    """Map each unique (plot, stalk) to a single x coordinate.

    Within a plot, x follows stalk number. Between plots a PLOT_GAP is
    left empty so groups do not run together. Keys may be any set of
    (plot, stalk) pairs seen across the loaded CSVs.
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


def plot_stiffnesses(csv_paths, show=True, save_dir=None):
    """Scatter every loaded EI value onto three figures (one per estimate).

    Parameters
    ----------
    csv_paths : sequence of path-like
        Results files from save_stiffnesses(). Same (Plot, Stalk) in
        different files share an x location and are colored by file.
    show : bool
        plt.show() at the end (block=True).
    save_dir : path-like or None
        If set, write estimate_ab.png / estimate_cb.png / estimate_abcb.png.
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

    if not all_keys:
        raise ValueError('No Plot/Stalk rows in the given CSVs')

    x_of, tick_x, tick_label = stalk_x_positions(all_keys)
    colors = plt.rcParams['axes.prop_cycle'].by_key()['color']

    figs = []
    max_stiff = 0.0
    for col_name, title in zip(ESTIMATE_COLS, ESTIMATE_TITLES):
        fig, ax = plt.subplots(figsize=(max(8.0, 0.35 * len(tick_x) + 2.0), 5.5))
        for i, (label, recs) in enumerate(by_source):
            xs = []
            ys = []
            for rec in recs:
                y = rec[col_name]
                if not np.isfinite(y):
                    continue
                xs.append(x_of[(rec['Plot'], rec['Stalk'])])
                ys.append(y)
                if y > max_stiff: max_stiff = y
            ax.scatter(
                xs, ys,
                s=46,
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
        ax.set_ylim(0,max_stiff*1.1)
        ax.grid(True, axis='y', alpha=0.3)
        if len(by_source) > 1:
            ax.legend(loc='best', fontsize=8)
        fig.tight_layout()
        figs.append(fig)

        if save_dir is not None:
            out_dir = Path(save_dir)
            out_dir.mkdir(parents=True, exist_ok=True)
            slug = title.lower().replace(' ', '_').replace('-', '')
            out = out_dir / f'{slug}.png'
            fig.savefig(out, dpi=150)
            print(f'Wrote {out}')

    if show:
        plt.show(block=True)
    return figs


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
        help='Optional directory to write the three PNG figures.',
    )
    parser.add_argument(
        '--no-show',
        action='store_true',
        help='Skip plt.show() (use with --save-dir).',
    )
    return parser.parse_args()


if __name__ == '__main__':
    # Edit this list when you are not passing paths on the command line.
    # Each file is one scatter series; matching (Plot, Stalk) share an x.
    RESULT_CSVS = [
        r'Hi-STIFFS_2026_Winter\Results\2026-08-28\104213_stiffnesses.csv',
        r'Hi-STIFFS_2026_Winter\Results\2026-08-28\110545_stiffnesses.csv',
        r'Hi-STIFFS_2026_Winter\Results\2026-08-28\112006_stiffnesses.csv'
    ]

    args = _parse_args()
    paths = [Path(p) for p in (args.csvs if args.csvs else RESULT_CSVS)]
    if not paths:
        print('Status: add paths to RESULT_CSVS or pass them as arguments')
    else:
        plot_stiffnesses(
            paths,
            show=not args.no_show,
            save_dir=args.save_dir,
        )
