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
    # row indices of various metadata 
    test_row_idx = 11
    height_row_idx = 12
    stalk_row_idx = 17

    # get key test info from header metadata
    with open(file_path, mode='r', newline='', encoding='utf-8') as file:
        reader = csv.reader(file)
        
        # Read until we reach the desired row
        for current_index, row in enumerate(reader):
            if current_index == test_row_idx:
                test_row = row
            
            if current_index == height_row_idx:
                height_row = row

            if current_index == stalk_row_idx:
                stalk_row = row
                break
        
    test_num = int(test_row[1])
    height = float(height_row[1]) * 1e-2
    stalk_ID = stalk_row[1]

    # load data
    df = pd.read_csv(file_path, skiprows=36)
    time = df['TIME (milliseconds)'].to_numpy() * 1e-3
    angle_pot = df['ANGLE_POT'].to_numpy()
    angle_imu = df['ANGLE_IMU'].to_numpy()
    load_x = df['LOAD_X'].to_numpy()
    load_y = df['LOAD_Y'].to_numpy()

    # compute stalk deflection at load cell contact point
    displacement = height * np.sin(np.radians(angle_pot - 90))

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
        plt.plot(displacement, load_x)
        plt.scatter(displacement, load_x, s=1)
        plt.plot(displacement, fitted_line, c='red')
        plt.title(rf'Test# - {test_num}, Stiffness - {stiffness:.2f} $N/m^2$'+f'\nStalk ID - {stalk_ID}, Height - {height} (m)')
        plt.xlabel('Lateral Displacement (m)')
        plt.ylabel('Load (N)')
        plt.xlim(-0.04, 0.16)
        plt.ylim(-2, 20)

    return stalk_ID, stiffness, residual_error


if __name__ == "__main__":
    # Main execution
    folder = r"Hi-STIFFS_2026_Winter\Raw Data\2026-07-13\DARLING\MED_07_13"
    csv_list = get_csv_files(folder)
    print(f"Found {len(csv_list)} CSV files.")

    # Collect all test results grouped by stalk_ID
    stalk_data = defaultdict(list)

    for path in csv_list:
        stalk_ID, stiffness, residual_error = process_test(path, plot_flag=True)
        stalk_data[stalk_ID].append((stiffness, residual_error))
        plt.show()  # Displays plot for each test (as in original script)

    # Select the 6 best stiffness values (lowest residual error) per stalk
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