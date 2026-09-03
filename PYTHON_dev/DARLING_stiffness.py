import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import csv
from pathlib import Path
from collections import defaultdict


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
    
    return csv_files


def process_test(file_path: str, plot_flag: bool=True) -> tuple[str, float, float]:
    """
    Processes single DARLING file to calculate flexural stiffness of stalk subject
    
    Parameters:
        file_path (str): Path to the DARLING file for processing.
    
    Returns:
        str: Stalk ID.
        float: calculated flexural stiffness.
        float: linear fit residual error
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
    load_x = df['LOAD_X'].to_numpy()
    load_y = df['LOAD_Y'].to_numpy()

    # compute stalk deflection at load cell contact point
    displacement = height * np.sin(-np.radians(angle_pot - 45))

    # linear fit to force (L) vs displacement (x) trace
    dLdx, inter = np.polyfit(displacement, load_x, deg=1)
    fitted_line = dLdx*displacement + inter
    residuals = load_x - fitted_line
    residual_error = np.sqrt(np.mean(residuals**2))

    # flexural stiffness from cantilever beam
    stiffness = (dLdx * height**3) / 3

    # Optional display results
    if plot_flag:
        plt.figure()
        plt.plot(displacement, load_x, color='0.7', lw=0.8, zorder=1)
        sc = plt.scatter(displacement, load_x, c=time, s=12, cmap='viridis', zorder=2)
        plt.colorbar(sc, label='Time (s)')
        plt.plot(displacement, fitted_line, c='red', zorder=3)
        plt.title(rf'Test# - {test_num_str}, Plot ID - {plot_num}, Stalk ID - {stalk_ID}'+f'\nStiffness - {stiffness:.2f} $N/m^2$')
        plt.xlabel('Lateral Displacement (m)')
        plt.ylabel('Load (N)')
        # adaptive plot limits
        x_lo, x_hi = -0.04, 0.16
        y_lo, y_hi = -2.0, 20.0

        x_min, x_max = np.min(displacement), np.max(displacement)
        y_min, y_max = np.min(load_x), np.max(load_x)

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

    return stalk_ID, stiffness, residual_error


if __name__ == "__main__":
    # Main execution
    folder = r"Hi-STIFFS_2026_Winter\Raw Data\DARLING Raw Data\aug28_2026_Chesterfield_DARLING02"
    csv_list = get_csv_files(folder)
    print(f"Found {len(csv_list)} CSV files.")

    # Collect all test results grouped by stalk_ID
    stalk_data = defaultdict(list)

    for path in csv_list:
        stalk_ID, stiffness, residual_error = process_test(path, plot_flag=True)
        stalk_data[stalk_ID].append((stiffness, residual_error))
        plt.show()  # Displays plot for each test (as in original script)

    # Gather stiffness estimates
    stiffnesses = {}
    for stalk_ID, tests in stalk_data.items():
        if not tests:
            continue
        stiffs = [stiff for stiff, err in tests]
        stiffnesses[stalk_ID] = stiffs
        num_tests = len(tests)
        print(f"Stalk {stalk_ID}: {num_tests} tests processed.")

    # Summary output
    print("\nStiffness values per stalk (object ready for later work):")
    for stalk, stiffs in sorted(stiffnesses.items()):
        mean_stiff = np.mean(stiffs) if stiffs else None
        print(f"  {stalk}: {stiffs} (mean: {mean_stiff:.2f} if applicable)")

    # 2. Pandas DataFrame + CSV (wide format, one row per stalk)
    max_tests = max(len(stiffs) for stiffs in stiffnesses.values())

    records = []
    for stalk_ID, stiffs in stiffnesses.items():
        row = {'Stalk': stalk_ID}
        for i in range(max_tests):
            col_name = f'Test_{i+1:02d}'
            row[col_name] = stiffs[i] if i < len(stiffs) else np.nan
        records.append(row)

    df_best = pd.DataFrame(records)
    df_best = df_best.sort_values('Stalk').reset_index(drop=True)

    df_best.to_csv('stiffnesses.csv', index=False)
    print("\nData saved to:")
    print("   - stiffnesses.csv (DataFrame format)")