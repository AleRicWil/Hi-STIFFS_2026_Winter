import serial
import csv
import keyboard
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
import time
import os
from sklearn.linear_model import LinearRegression
from scipy.optimize import minimize
from collect_data3 import DataReceiverWriter
from process2 import HiSTIFFSData

# Hardcoded Paths
CALIBRATION_DIRECTORY = r'Hi-STIFFS_2026_Winter/Calibration'

def run_calibration(sensor_label, sensor_serial_number, loads, positions):
    """Run calibration by collecting strain data for each mass-position pair.

    Args:
        sensor_label (str): label in Hi-STIFFS probe of sensor to be calibrated
        sensor_serial_number (str): Configuration dictionary with calibration parameters.
    """
    l = sensor_label
    sn = sensor_serial_number
    print(f"Calibration procedure for sensor {sn} on Hi-STIFFS DAQ position {l}")
    print(f"Will collect static loading for {loads} N each at {positions} mm")
    file_paths = {}
    for pos in positions:
        for load in loads:       
            header_content = [
                "Test Type: Calibration",
                "Number of ICB-Sensors: 1, Sensor Label(s): A",
                f"Sensor Serial#: {sn}, Sensor Label on Hi-STIFFS DAQ: {l}",
                f'Load Type: Force (N), Force Control: Uniaxial Test Machine, Position Control: Visual Tick Marks (mm)',
                f"Loads: [{' '.join(f'{l}' for l in loads)}], Positions: [{' '.join(f'{p}' for p in positions)}]",
                f"This Load: {load} N, This Position: {pos} mm"
                ]
            
            wait = True
            print(f"\nPress 'space' to start collection {load}N_{pos}mm")
            while wait:
                if keyboard.is_pressed('space'):
                    wait = False
                time.sleep(0.01)

            print(f'Starting collection for {load}N_{pos}mm...')
            ReadWrite = DataReceiverWriter(5, header_content= header_content)
            file_paths[f'{load}N_{pos}mm'] = ReadWrite.csv_path
            ReadWrite.status_signal.connect(print) 
            ReadWrite.start()      

            print(f'Collection initializing for {load}N_{pos}mm...')
            while ReadWrite.first_packet_time is None:
                time.sleep(0.1)

            wait = True
            print("Press 'space' to stop after 3 seconds ('space' is blocked until 3sec timer expires).")
            time.sleep(1)      
            while wait:
                if keyboard.is_pressed('space'):
                    wait = False
                    ReadWrite.running = False
                    ReadWrite.wait()
                time.sleep(0.01)
               
            print('Collection stopped.')
            time.sleep(1)

            print('Calculating average...')
            data = HiSTIFFSData(None, None, file_path=ReadWrite.csv_path)
            s = data.data_dict[f'Sensor_{l}']
            t_min = np.min(s['time'])
            t_max = np.max(s['time'])
            strain_1 = s['strain_1_raw']
            strain_2 = s['strain_2_raw']

            mask = (s['time'] >= t_min + 1) & (s['time'] <= t_max - 1)
            avg_1 = np.average(strain_1[mask])
            avg_2 = np.average(strain_2[mask])

            calibration_path = ReadWrite.csv_path
            # remove existing file name, add /calibration_{sn}.csv
            calibration_path = os.path.join(os.path.dirname(calibration_path), f"calibration_{sn}.csv")

            # Create calibration.csv and headers if it doesn't exist.
            headers = ["Load (N)", "Position (mm)", f"Strain_{l}1_avg", f"Strain_{l}2_avg"]
            if not os.path.exists(calibration_path):
                with open(calibration_path, 'w', newline='') as csvfile:
                    writer = csv.writer(csvfile)
                    writer.writerow(headers)

            # append row to calibration_{sn}.csv
            row = [load, pos, avg_1, avg_2]
            with open(calibration_path, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(row)

            print('Stored average')

    print("\nCollected file paths:")
    for key in file_paths:
        print(f"{key} → {file_paths[key]}")
        data = HiSTIFFSData(None, None, file_path=file_paths[key])
        data.plot_raw_strains()       

    return calibration_path


def calculate_coefficients(calibration_path):
    """Calculate calibration coefficients using multiple linear regression to directly fit k, d, and c.

    Args:
        calibration_data (list): List of rows from the calibration summary CSV.
        cal_status_var (tk.StringVar): Tkinter variable to update the UI status.

    Returns:
        str: Formatted string of calculated coefficients.
    """
    sn = calibration_path[-7:-4]    # serial number of sensor
    print(sn)

    with open(calibration_path, 'r') as f:
        reader = csv.reader(f)
        next(reader, None)  # skip header
        calibration_data = [row for row in reader]


    if not calibration_data:
        print("Status: No calibration data loaded")
        return ""

    try:
        # Extract and filter data, ensuring all columns are valid numbers
        valid_data = []
        for row in calibration_data:
            try:
                load = float(row[0])        # Newtons
                position = float(row[1])    # mm
                strain_1 = float(row[2])    # raw ADC count
                strain_2 = float(row[3])    # raw ADC count
                valid_data.append((load, position, strain_1, strain_2))
            except ValueError:
                continue

        if not valid_data:
            print("Status: No valid data for calculation")
            return ""

        # Unpack data
        loads, positions, strains_1, strains_2 = zip(*valid_data)
        loads = np.array(loads)
        positions = np.array(positions)
        strains_1 = np.array(strains_1)
        strains_2 = np.array(strains_2)

        # Match units
        positions = positions * 1e-3  # mm to m

        # Create design matrix A = [F*x, -F] for multiple linear regression
        A = np.column_stack((loads * positions, -loads))

        # Fit models for each strain type: V = c + k*(F*x) + beta2*(-F), where beta2 = k*d
        model_1 = LinearRegression().fit(A, strains_1)
        c_1 = model_1.intercept_
        k_1 = model_1.coef_[0]
        beta2_1 = model_1.coef_[1]
        d_1 = beta2_1 / k_1 if k_1 != 0 else 0

        model_2 = LinearRegression().fit(A, strains_2)
        c_2 = model_2.intercept_
        k_2 = model_2.coef_[0]
        beta2_2 = model_2.coef_[1]
        d_2 = beta2_2 / k_2 if k_2 != 0 else 0

        # Format result
        result = {f"{sn}": {'k1': k_1, 'd1': d_1, 'c1': c_1,
                            'k2': k_2, 'd2': d_2, 'c2': c_2}}
        print(result)

        # Predict strains
        strain_1_pred = model_1.predict(A)
        strain_2_pred = model_2.predict(A)

        # Compute R-squared for goodness of fit
        r2_1 = model_1.score(A, strains_1)
        r2_2 = model_2.score(A, strains_2)
        print(f"Channel 1 R-squared: {r2_1:.4f}")
        print(f"Channel 2 R-squared: {r2_2:.4f}")

        # Optional plotting: Measured vs Predicted for both channels on a single plot
        plt.figure(figsize=(8, 6))

        # Channel 1 scatter and fit line
        plt.scatter(strains_1, strain_1_pred, label='Channel 1 Data', color='green', alpha=0.7)
        parity_model_1 = LinearRegression().fit(strains_1.reshape(-1, 1), strain_1_pred)
        x_line_1 = np.linspace(min(strains_1), max(strains_1), 100)
        y_line_1 = parity_model_1.predict(x_line_1.reshape(-1, 1))
        plt.plot(x_line_1, y_line_1, color='green', linestyle='-', label='Channel 1 Fit')

        # Channel 2 scatter and fit line
        plt.scatter(strains_2, strain_2_pred, label='Channel 2 Data', color='red', alpha=0.7)
        parity_model_2 = LinearRegression().fit(strains_2.reshape(-1, 1), strain_2_pred)
        x_line_2 = np.linspace(min(strains_2), max(strains_2), 100)
        y_line_2 = parity_model_2.predict(x_line_2.reshape(-1, 1))
        plt.plot(x_line_2, y_line_2, color='red', linestyle='-', label='Channel 2 Fit')

        # Perfect fit line (y = x)
        all_measured = np.concatenate((strains_1, strains_2))
        all_predicted = np.concatenate((strain_1_pred, strain_2_pred))
        min_val = min(min(all_measured), min(all_predicted))
        max_val = max(max(all_measured), max(all_predicted))
        plt.plot([min_val, max_val], [min_val, max_val], 'k--', label='Perfect Fit (y = x)', alpha=0.5)

        plt.xlabel('Measured Strain')
        plt.ylabel('Predicted Strain')
        plt.title(f'Measured vs Predicted Strain\n(Channel 1 R² = {r2_1:.8f}, Channel 2 R² = {r2_2:.8f})')
        plt.legend()
        plt.axis('equal')
        plt.grid(True)
        plt.tight_layout()
        plt.show()

        # Write coefficients to files
        os.makedirs(CALIBRATION_DIRECTORY, exist_ok=True)
        now = datetime.now().isoformat()

        coeffs = result[sn]
        row = [now, coeffs['k1'], coeffs['d1'], coeffs['c1'], coeffs['k2'], coeffs['d2'], coeffs['c2']]

        # Append to history file for this sn
        history_path = os.path.join(CALIBRATION_DIRECTORY, f"calibration_history_{sn}.csv")
        history_headers = ['datetime', 'k1', 'd1', 'c1', 'k2', 'd2', 'c2']
        if not os.path.exists(history_path):
            with open(history_path, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(history_headers)
        with open(history_path, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(row)

        # Update current_calibrations file
        current_path = os.path.join(CALIBRATION_DIRECTORY, "current_calibrations.csv")
        current_headers = ['sn', 'datetime', 'k1', 'd1', 'c1', 'k2', 'd2', 'c2']
        current_rows = []
        if os.path.exists(current_path):
            with open(current_path, 'r') as csvfile:
                reader = csv.reader(csvfile)
                headers = next(reader, None)
                for r in reader:
                    if r and r[0] != sn:
                        current_rows.append(r)
        current_rows.append([sn] + row)
        with open(current_path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(current_headers)
            writer.writerows(current_rows)

        return result

    except Exception as e:
        print(f"Status: Error calculating coefficients: {str(e)}")
        return ""


def generate_summary(paths):
    '''
    creates calibration_{sn}.csv for a given set of raw data files.
    reads load and position from each file's header
    
    :param paths: list of file paths where a previous calibration raw data was stored
    '''
    if not paths:
        print("No paths provided.")
        return

    # Parse sn and l from the first file
    with open(paths[0], 'r') as f:
        lines = f.readlines()

    sn = None
    l = None
    for line in lines:
        if "Sensor Serial#:" in line:
            # Format: "Sensor Serial#: {sn}, Sensor Label on Hi-STIFFS DAQ: {l}"
            parts = line.split(',')
            if len(parts) >= 2:
                sn_part = parts[0].split(':')
                l_part = parts[1].split(':')
                if len(sn_part) > 1 and len(l_part) > 1:
                    sn = sn_part[1].strip()
                    l = l_part[1].strip()
                    print(sn, l)
            break

    if not sn or not l:
        print("Error: Could not parse sensor serial number or label from the first file.")
        return

    # Check that all files have the same sn and l
    for path in paths[1:]:
        with open(path, 'r') as f:
            lines = f.readlines()

        this_sn = None
        this_l = None
        for line in lines:
            if "Sensor Serial#:" in line:
                parts = line.split(',')
                if len(parts) >= 2:
                    sn_part = parts[0].split(':')
                    l_part = parts[1].split(':')
                    if len(sn_part) > 1 and len(l_part) > 1:
                        this_sn = sn_part[1].strip()
                        this_l = l_part[1].strip()
                break

        if this_sn != sn or this_l != l:
            print(f"Error: Mismatch in sensor serial number or label in file {path}. Expected sn: {sn}, l: {l}; Found sn: {this_sn}, l: {this_l}")
            return

    # Prepare headers for the summary CSV
    headers = ["Load (N)", "Position (mm)", f"Strain_{l}1_avg", f"Strain_{l}2_avg"]

    # Collect data rows
    data_rows = []
    for path in paths:
        # Parse load and position from header
        with open(path, 'r') as f:
            lines = f.readlines()

        load = None
        pos = None
        for line in lines:
            if "This Load:" in line:
                # Format: "This Load: {load} N, This Position: {pos} mm"
                parts = line.split(',')
                if len(parts) >= 2:
                    load_part = parts[2].split(':')
                    pos_part = parts[3].split(':')
                    print(load_part, pos_part)
                    if len(load_part) > 1 and len(pos_part) > 1:
                        load_str = load_part[1].strip().split(' ')[0]
                        pos_str = pos_part[1].strip().split(' ')[0]
                        try:
                            load = float(load_str)
                            pos = float(pos_str)
                        except ValueError:
                            pass
                break

        if load is None or pos is None:
            print(f"Warning: Could not parse load or position from {path}. Skipping.")
            continue

        # Load and process the data
        data = HiSTIFFSData(None, None, file_path=path)
        s = data.data_dict[f'Sensor_{l}']
        t_min = np.min(s['time'])
        t_max = np.max(s['time'])
        strain_1 = s['strain_1_raw']
        strain_2 = s['strain_2_raw']

        mask = (s['time'] >= t_min + 1) & (s['time'] <= t_max - 1)
        avg_1 = np.average(strain_1[mask])
        avg_2 = np.average(strain_2[mask])

        data_rows.append([load, pos, avg_1, avg_2])

    if not data_rows:
        print("No valid data rows collected.")
        return

    # Determine the directory from the first path and create calibration path
    dir_path = os.path.dirname(paths[0])
    calibration_path = os.path.join(dir_path, f"calibration_{sn}.csv")

    # Write the summary CSV
    with open(calibration_path, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(headers)
        writer.writerows(data_rows)

    print(f"Summary CSV generated at: {calibration_path}")

if __name__ == "__main__":
    # run_calibration('A', '001', [10, 50], [60, 100])   
    old_paths = [r'Hi-STIFFS_2026_Winter\Raw Data\2026-02-13\2026-02-13_163645_01.csv',
                 r'Hi-STIFFS_2026_Winter\Raw Data\2026-02-13\2026-02-13_163754_01.csv',
                 r'Hi-STIFFS_2026_Winter\Raw Data\2026-02-13\2026-02-13_163843_01.csv',
                 r'Hi-STIFFS_2026_Winter\Raw Data\2026-02-13\2026-02-13_163945_01.csv']
    # generate_summary(old_paths)
    calculate_coefficients(r'Hi-STIFFS_2026_Winter\Raw Data\2026-02-13\calibration_001.csv')