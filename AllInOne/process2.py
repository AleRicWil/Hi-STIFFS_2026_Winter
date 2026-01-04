# This file reads data from a CSV and processes it with multiple methods and options availible to the user
# The CSV must have columns titled like 'Time_A_sec, Strain_A1_raw, Strain_A2_raw...' and so on for each lettered sensor
# Or columns like 'Time_A_sec, Strain_A1_V, Strain_A2_V...'. This is raw ADC values or calculated voltages in volts.
# The __init__() will automatically detect which type is present, or if both are, defaults to raw option (more efficient/precise).
# Calibration is done with raw values. mV option is for human comprehension and only viable for displays, not in calculations.

import os
import csv
import pandas as pd
import numpy as np

# Hardcoded paths (adjust as needed for your environment)
CALIBRATION_PATH = r'Hi-STIFFS_2026_Winter\AllInOne\calibration_history.csv'
RAW_DATA_BASE = r'Hi-STIFFS_2026_Winter\Raw Data'
RESULTS_BASE = r'Hi-STIFFS_2026_Winter\Results'
HEADER_MARKER = r'===END_METADATA==='
DATA_MARKER = r'===BEGIN_DATA==='

class HiSTIFFSData:
    def __init__(self, date, time, debug=False):
        # Form CSV path
        self.date = date
        self.time = time
        self.data_csv_path = os.path.join(RAW_DATA_BASE, rf"{date}\{date}_test_{time}.csv")
        if not os.path.exists(self.data_csv_path):
            self.exist = False
            print(f"No such data file at: {self.data_csv_path}")
            return
        
        # Get all data from file
        with open(self.data_csv_path, 'r') as f:
            self.exist = True
            csv_reader = csv.reader(f)

            # Find metadata end and data start tags, which must be in immediate sequence
            # Hence, reset check1 flag if data start isn't next in loop
            check1 = False
            data_index = -1
            self.header_rows = []
            for row_index, row in enumerate(csv_reader):
                self.header_rows.append(row)
                if check1 and len(row) == 1 and row[0].strip() == DATA_MARKER:
                    data_index = row_index
                    break

                if len(row) == 1 and row[0].strip() == HEADER_MARKER:
                    check1 = True
                    continue
                else: 
                    check1 = False
            if data_index == -1: raise ValueError("No marker for end metadata / start data in CSV")

            # Parse required header data
            try:
                test_info = next((row for row in self.header_rows if row and "Test Type" in row[0]), None)
                test_type = ' '.join(test_info[0].split()[2:])
                print(f'test info: {test_info}')
                if debug: print(f"test type: {test_type}")
                
                if test_type == 'Force Cycle':
                    self.cycle_force = next((s.split()[-1] for s in test_info if "Force:" in s), None)
                    s = self.cycle_force
                    self.cycle_force, self.cycle_force_units = next(((s[:i], s[i:]) for i in range(1, len(s)) 
                                                                     if s[i].isalpha() and s[i-1].isdigit()), (s, ''))
                    if debug: print(f'cycle force: {self.cycle_force}, units: {self.cycle_force_units}')
                
                sensors_info = next((row for row in self.header_rows if row and "Number of ICB-Sensors" in row[0]), None)
                num_sensors = sensors_info[0][-1]
                self.sensor_labels = next(([s.partition("Label(s):")[2].strip()] for s in sensors_info if "Label(s)" in s), None)
                print(f'sensor info: {sensors_info}')
                if debug: print(f"num sensors: {num_sensors}"); print(f"sensor labels: {self.sensor_labels}")

                calibration_dict = {}
                for l in self.sensor_labels:
                    coeffs = [f'k{l}1', f'd{l}1', f'c{l}1', f'k{l}2', f'd{l}2', f'c{l}2']
                    cal_info = next((row for row in self.header_rows if row and "ICB-Sensor A's Latest Calibration" in row[0]), None)
                    if debug: print(f"sensor {l} cal info: {cal_info}")
                    c = []
                    for coeff in coeffs:
                        c.append(next((s.split()[-1] for s in cal_info if coeff in s), None))

                    coeff_dict = {f"k{l}1": c[0], f"d{l}1": c[1], f"c{l}1": c[2], 
                                  f"k{l}2": c[3], f"d{l}2": c[4], f"c{l}2": c[5]}
                    calibration_dict[f"Sensor_{l}"] = coeff_dict
                if debug: print(calibration_dict)

            except:
                raise ValueError("Error while parsing metadata")

            # Load sensor data
            data = pd.read_csv(f)

        # Re-pack all data in appropriate data-types
        self.data_dict = {}
        for l in self.sensor_labels:
            sensor_data = {'time': data[f'Time_{l}_sec'].to_numpy(dtype=np.float64),
                           'strain_1_raw': data[f'Strain_{l}1_raw'].to_numpy(dtype=np.int32),
                           'strain_2_raw': data[f'Strain_{l}2_raw'].to_numpy(dtype=np.int32)}
            self.data_dict[f'Sensor_{l}'] = sensor_data

        split_colons = data['Processed_Time'].str.split(':', expand=True)
        if split_colons.shape[1] != 3:
            raise ValueError("Invalid 'Processed Time' format: Expected hh:mm:ss.us")
        split_seconds = split_colons[2].str.split('.', expand=True)
        if split_seconds.shape[1] != 2:
            raise ValueError("Invalid 'Processed Time' format: Expected hh:mm:ss.us")
        
        hh = split_colons[0].astype(np.int64)
        mm = split_colons[1].astype(np.int64)
        ss = split_seconds[0].astype(np.int64)
        us = split_seconds[0].astype(np.int64)
        processed_time = (hh*np.timedelta64(1,'h') + mm*np.timedelta64(1, 'm')
                           + ss*np.timedelta64(1, 's') + us*np.timedelta64(1, 'us'))
        self.data_dict['Processed Time'] = processed_time.to_numpy(dtype='timedelta64[us]')

    def plot_raw_strains(self, sensors='A,B,C,D,E'):
        sensors_to_plot = sensors.split(',')

        removed = [label for label in sensors_to_plot if label not in self.sensor_labels]
        for label in removed:
            print(f"Sensor {label} not in CSV data")
        sensors_to_plot = [label for label in sensors_to_plot if label in self.sensor_labels]





if __name__ == "__main__":
    data = HiSTIFFSData(date="2026-01-03", time="135838", debug=False)

    data.plot_raw_strains()
