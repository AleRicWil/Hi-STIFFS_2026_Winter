import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import csv
from pathlib import Path
from collections import defaultdict

# Loading-segment filter (one pipeline for every DARLING push)
ANGLE_MIN_DEG = 25.0
ANGLE_MAX_DEG = 65.0
ANGLE_JUMP_DEG = 10.0
ANGLE_JUMP_MAX_DT_S = 0.05
LBF_TO_N = 4.4482216152605      # 1 lbf = 4.4482216152605 N
LOAD_CONTACT_ABS = 0.5          # N
LOAD_CONTACT_FRAC = 0.15
DISPLACEMENT_ONSET_M = 0.005
ONSET_WINDOW_S = 0.050
MIN_FIT_POINTS = 25
MIN_FIT_SPAN_M = 0.02


def get_csv_files(directory_path: str) -> list[str]:
    """
    Returns a list of full paths to all .csv files in the specified directory.
    
    Parameters:
        directory_path (str): Path to the directory to search.
    
    Returns:
        list[str]: List of absolute file paths to CSV files.
    """
    directory = Path(directory_path)
    
    # Ensure the directory exists
    if not directory.is_dir():
        raise NotADirectoryError(f"Directory not found: {directory_path}")
    
    # Collect all CSV files (case-insensitive)
    csv_files = [
        str(file.absolute())
        for file in directory.iterdir()
        if file.is_file() and file.suffix.lower() == '.csv'
    ]

    # Timestamp-prefixed filenames sort into chronological / test order
    csv_files.sort()
    return csv_files


def loading_mask(
    time: np.ndarray,
    angle_pot: np.ndarray,
    displacement: np.ndarray,
    load_x: np.ndarray,
) -> np.ndarray:
    """Boolean mask of samples on the forward loading ramp.

    Order: physical angle limits, impossible angle jumps, cut after the
    earlier of peak displacement / peak load, contact-load floor, then
    5 mm of arm travel before the fit starts.
    """
    mask = (
        np.isfinite(time)
        & np.isfinite(angle_pot)
        & np.isfinite(displacement)
        & np.isfinite(load_x)
        & (angle_pot >= ANGLE_MIN_DEG)
        & (angle_pot <= ANGLE_MAX_DEG)
    )

    idx = np.flatnonzero(mask)
    if idx.size >= 2:
        d_ang = np.abs(np.diff(angle_pot[idx]))
        d_t = np.diff(time[idx])
        jumped = (d_ang > ANGLE_JUMP_DEG) & (d_t < ANGLE_JUMP_MAX_DT_S)
        mask[idx[1:][jumped]] = False

    idx = np.flatnonzero(mask)
    if idx.size == 0:
        return mask

    i_end_rel = min(int(np.argmax(displacement[idx])), int(np.argmax(load_x[idx])))
    mask[idx[i_end_rel] + 1:] = False

    idx = np.flatnonzero(mask)
    if idx.size == 0:
        return mask

    peak_load = float(np.max(load_x[idx]))
    load_cut = max(LOAD_CONTACT_ABS, LOAD_CONTACT_FRAC * peak_load)
    mask[idx] = mask[idx] & (load_x[idx] >= load_cut)

    idx = np.flatnonzero(mask)
    if idx.size == 0:
        return mask

    t0 = time[idx[0]]
    onset_idx = idx[time[idx] <= t0 + ONSET_WINDOW_S]
    if onset_idx.size == 0:
        onset_idx = idx[:1]
    x0 = float(np.median(displacement[onset_idx]))
    mask[idx] = mask[idx] & (displacement[idx] >= x0 + DISPLACEMENT_ONSET_M)
    return mask


def process_test(file_path: str, plot_flag: bool=True) -> tuple[int, str, float, float] | None:
    """
    Processes single DARLING file to calculate flexural stiffness of stalk subject
    
    Parameters:
        file_path (str): Path to the DARLING file for processing.
    
    Returns:
        int: Plot number.
        str: Stalk ID.
        float: calculated flexural stiffness.
        float: linear fit residual error
        None if the file is skipped (operator is not Alex, or the
        loading-segment filter left too little data to fit).
    """
    # Parse header metadata by field title rather than row index
    meta = {}
    pre_test_notes = []

    # Last two '_' -separated pieces of the filename. Plot and test_num in plot.
    parts = Path(file_path).stem.split('_')
    if len(parts) < 2:
        raise ValueError(f'Expected at least two "_" tokens in filename: {file_path}')
    plot_str, test_num_str = parts[-2], parts[-1]

    with open(file_path, mode='r', newline='', encoding='utf-8') as file:
        reader = csv.reader(file)
        for row in reader:
            if not row:
                continue
            key = row[0].strip()
            # Stop once the timeseries block begins
            if key.startswith('----------TEST DATA') or key.startswith('TIME (milliseconds)'):
                break
            value = row[1].strip() if len(row) > 1 else ''
            meta[key] = value
            if key.startswith('PRE_TEST_NOTE') and value:
                pre_test_notes.append(value)

    operator_name = meta.get('DEVICE OPERATOR', '')
    if operator_name.strip() != 'Alex':
        print(f"Skipping {Path(file_path).name}: operator is '{operator_name}'")
        return None

    time_str = meta.get('TIME', '')          # e.g. '11:21:13'
    plot_num = int(meta['PLOT'])              # plot number
    height = float(meta['HEIGHT']) * 1e-2     # cm -> m

    # Stalk ID lives in a PRE_TEST_NOTE field as 's##'
    stalk_ID = next(
        (note for note in pre_test_notes
         if len(note) >= 2 and note[0] in 'sS' and note[1:].isdigit()),
        ''
    )
    if not stalk_ID:
        raise ValueError(f'No stalk ID (s##) found in PRE_TEST_NOTE fields: {file_path}')

    # load data — find TEST DATA section instead of hard-coding skiprows
    test_data_idx = None
    with open(file_path, mode='r', newline='', encoding='utf-8') as file:
        for i, row in enumerate(csv.reader(file)):
            if row and 'TEST DATA' in row[0]:
                test_data_idx = i
                break
    if test_data_idx is None:
        raise ValueError(f'TEST DATA section not found: {file_path}')

    df = pd.read_csv(file_path, skiprows=test_data_idx + 1)
    time = df['TIME (milliseconds)'].to_numpy() * 1e-3
    angle_pot = df['ANGLE_POT'].to_numpy()
    angle_imu = df['ANGLE_IMU'].to_numpy()
    # CSV LOAD_X is lbf; convert immediately so the rest of the pipeline is SI
    load_x = df['LOAD_X'].to_numpy() * LBF_TO_N
    load_y = df['LOAD_Y'].to_numpy()

    # compute stalk deflection at load cell contact point
    displacement = height * np.sin(-np.radians(angle_pot - 45))
    keep = loading_mask(time, angle_pot, displacement, load_x)
    n_kept = int(np.count_nonzero(keep))
    n_total = displacement.size
    skip_reason = None
    dLdx = inter = residual_error = stiffness = None

    if n_kept < MIN_FIT_POINTS:
        skip_reason = f'only {n_kept} points after filtering (need {MIN_FIT_POINTS})'
    else:
        x_kept = displacement[keep]
        load_kept = load_x[keep]
        span = float(np.max(x_kept) - np.min(x_kept))
        if span < MIN_FIT_SPAN_M:
            skip_reason = f'displacement span {span:.3f} m after filtering (need {MIN_FIT_SPAN_M} m)'
        else:
            dLdx, inter = np.polyfit(x_kept, load_kept, deg=1)
            residual_error = float(np.sqrt(np.mean((load_kept - (dLdx * x_kept + inter))**2)))
            stiffness = (dLdx * height**3) / 3

    if skip_reason:
        print(f"Skipping {Path(file_path).name}: {skip_reason}")

    # Optional display results
    if plot_flag:
        plt.figure()
        plt.plot(displacement, load_x, color='0.85', lw=0.8, zorder=1)
        rejected = ~keep
        if np.any(rejected):
            plt.scatter(displacement[rejected], load_x[rejected],
                        c='0.75', s=10, zorder=2)
        if n_kept:
            sc = plt.scatter(displacement[keep], load_x[keep],
                             c=time[keep], s=12, cmap='viridis', zorder=3)
            plt.colorbar(sc, label='Time (s)')
        if dLdx is not None:
            x_line = np.linspace(np.min(displacement[keep]), np.max(displacement[keep]), 50)
            plt.plot(x_line, dLdx * x_line + inter, c='red', zorder=4)
            title_result = rf'Stiffness - {stiffness:.2f} $N/m^2$'
        else:
            title_result = f'Skipped - {skip_reason}'
        plt.title(
            rf'Test# - {test_num_str}, Plot ID - {plot_num}, Stalk ID - {stalk_ID}'
            + f'\n{title_result}  ({n_kept}/{n_total} pts)'
        )
        plt.xlabel('Lateral Displacement (m)')
        plt.ylabel('Load (N)')
        x_lo, x_hi = -0.04, 0.16
        y_lo, y_hi = -2.0, 20.0
        if n_kept:
            x_min, x_max = float(np.min(displacement[keep])), float(np.max(displacement[keep]))
            y_min, y_max = float(np.min(load_x[keep])), float(np.max(load_x[keep]))
            if x_min < x_lo:
                x_lo = x_min - 0.05 * abs(x_min)
            if x_max > x_hi:
                x_hi = x_max + 0.05 * abs(x_max)
            if y_min < y_lo:
                y_lo = y_min - 0.05 * abs(y_min)
            if y_max > y_hi:
                y_hi = y_max + 0.05 * abs(y_max)
        plt.xlim(x_lo, x_hi)
        plt.ylim(y_lo, y_hi)

    if skip_reason:
        return None
    return plot_num, stalk_ID, stiffness, residual_error


if __name__ == "__main__":
    # Main execution
    folder = r"Hi-STIFFS_2026_Winter\Raw Data\DARLING Raw Data\aug28_2026_Chesterfield_DARLING02"
    csv_list = get_csv_files(folder)
    print(f"Found {len(csv_list)} CSV files.")

    # Collect all test results grouped by (plot, stalk)
    stalk_data = defaultdict(list)

    for path in csv_list:
        result = process_test(path, plot_flag=False)
        if result is None:
            continue
        plot_num, stalk_ID, stiffness, residual_error = result
        stalk_data[(plot_num, stalk_ID)].append((stiffness, residual_error))
        plt.show()  # Displays plot for each test (as in original script)

    # Gather stiffness estimates
    stiffnesses = {}
    for (plot_num, stalk_ID), tests in stalk_data.items():
        if not tests:
            continue
        stiffs = [stiff for stiff, err in tests]
        stiffnesses[(plot_num, stalk_ID)] = stiffs
        num_tests = len(tests)
        print(f"Plot {plot_num}, Stalk {stalk_ID}: {num_tests} tests processed.")

    # Summary output
    print("\nStiffness values per stalk (object ready for later work):")
    for (plot_num, stalk_ID), stiffs in sorted(stiffnesses.items()):
        mean_stiff = np.mean(stiffs) if stiffs else None
        print(f"  Plot {plot_num}, {stalk_ID}: {stiffs} (mean: {mean_stiff:.2f} if applicable)")

    if not stiffnesses:
        print("\nNo tests survived filtering.")
    else:
        # Pandas DataFrame + CSV (wide format, one row per plot/stalk)
        max_tests = max(len(stiffs) for stiffs in stiffnesses.values())

        records = []
        for (plot_num, stalk_ID), stiffs in stiffnesses.items():
            row = {'Plot': plot_num, 'Stalk': stalk_ID}
            for i in range(max_tests):
                col_name = f'Test_{i+1:02d}'
                row[col_name] = stiffs[i] if i < len(stiffs) else np.nan
            records.append(row)

        df_best = pd.DataFrame(records)
        df_best = df_best.sort_values(['Plot', 'Stalk']).reset_index(drop=True)

        df_best.to_csv('stiffnesses.csv', index=False)
        print("\nData saved to:")
        print("   - stiffnesses.csv (DataFrame format)")