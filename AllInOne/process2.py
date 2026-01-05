# This file reads data from a CSV and processes it with multiple methods and options availible to the user
# The CSV must have columns titled like 'Time_A_sec, Strain_A1_raw, Strain_A2_raw...' and so on for each lettered sensor
# Or columns like 'Time_A_sec, Strain_A1_V, Strain_A2_V...'. This is raw ADC values or calculated voltages in volts.
# The __init__() will automatically detect which type is present, or if both are, defaults to raw option (more efficient/precise).
# Calibration is done with raw values. mV option is for human comprehension and only viable for displays, not in calculations.

# Plot number ranges:
#   0-9: raw strains


import os
import csv
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import StrMethodFormatter
from scipy.signal import savgol_filter

# Hardcoded paths (adjust as needed for your environment)
CALIBRATION_PATH = r'Hi-STIFFS_2026_Winter\AllInOne\calibration_history.csv'
RAW_DATA_BASE = r'Hi-STIFFS_2026_Winter\Raw Data'
RESULTS_BASE = r'Hi-STIFFS_2026_Winter\Results'
HEADER_MARKER = r'===END_METADATA==='
DATA_MARKER = r'===BEGIN_DATA==='

class HiSTIFFSData:
    def __init__(self, date, time, debug=False, base=RAW_DATA_BASE):
        # Form CSV path
        # Note: Using os.path.join ensures cross-platform compatibility for path construction on Windows, Linux, and Raspberry Pi.
        self.date = date
        self.time = time
        self.data_csv_path = os.path.join(base, date, f"{date}_test_{time}.csv")
        if not os.path.exists(self.data_csv_path):
            self.exists = False
            print(f"No such data file at: {self.data_csv_path}")
            return
        
        # Get all data from file
        with open(self.data_csv_path, 'r') as f:
            self.exists = True
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
                self.data_dict = {}
                # Test info
                test_info = next((row for row in self.header_rows if row and "Test Type" in row[0]), None)
                test_type = ' '.join(test_info[0].split()[2:])
                print(f'test info: {test_info}')
                if debug: print(f"test type: {test_type}")
                
                if test_type == 'Force Cycle':
                    self.cycle_force = next((s.split()[-1] for s in test_info if "Force:" in s), None)
                    s = self.cycle_force
                    self.cycle_force, self.cycle_force_units = next(((s[:i], s[i:]) for i in range(1, len(s)) 
                                                                     if s[i].isalpha() and s[i-1].isdigit()), (s, ''))
                    next_row = next((row for row in self.header_rows if row and "Test Number" in row[0]), None)
                    self.test_number = int(next_row[0].split()[-1])
                    self.test_rest = next_row[1].split()[-1]

                    if debug: print(f'cycle force: {self.cycle_force}, units: {self.cycle_force_units}')
                
                # Sensors info
                sensors_info = next((row for row in self.header_rows if row and "Number of ICB-Sensors" in row[0]), None)
                num_sensors = sensors_info[0][-1]
                self.sensor_labels = next(([s.partition("Label(s):")[2].strip()] for s in sensors_info if "Label(s)" in s), None)
                print(f'sensors info: {sensors_info}')
                if debug: print(f"num sensors: {num_sensors}"); print(f"sensor labels: {self.sensor_labels}")

                # Sensor info
                sensor_info = []
                for l in self.sensor_labels:
                    sensor_info.append(next((row for row in self.header_rows if row and f"ICB-Sensor {l}" in row[0]), None))
                
                self.sensor_info_dict = {}
                for l in self.sensor_labels:
                    for row in sensor_info:
                        if row and f"ICB-Sensor {l}" in row[0]:
                            temp_dict = {'SN': row[0].split()[-1]}
                            self.sensor_info_dict[f'Sensor_{l}'] = temp_dict
                            self.data_dict[f'Sensor_{l}'] = temp_dict
                
                if debug: print(f'sensor info: {sensor_info}'); print(f'sensor dict: {self.sensor_info_dict}')

                # Sensor(s) calibration
                calibration_dict = {}
                for l in self.sensor_labels:
                    coeffs = [f'k{l}1', f'd{l}1', f'c{l}1', f'k{l}2', f'd{l}2', f'c{l}2']
                    cal_info = next((row for row in self.header_rows if row and f"ICB-Sensor {l}'s Latest Calibration" in row[0]), None)
                    if debug: print(f"sensor {l} cal info: {cal_info}")
                    c = []
                    for coeff in coeffs:
                        c.append(next((s.split()[-1] for s in cal_info if coeff in s), None))

                    coeff_dict = {f"k{l}1": c[0], f"d{l}1": c[1], f"c{l}1": c[2], 
                                  f"k{l}2": c[3], f"d{l}2": c[4], f"c{l}2": c[5]}
                    calibration_dict[f"Sensor_{l}"] = coeff_dict
                if debug: print(f'cal dict: {calibration_dict}')

                # ADS1220 info
                ADC_info = next((row for row in self.header_rows if row and "Analog" in row[0]), None)
                data_rate_str = next((s.split()[-1] for s in ADC_info if 'Rate' in s), None)
                self.data_rate = int(data_rate_str.replace('DR_', '').replace('SPS', ''))
                self.filter_window = int(round(self.data_rate/20.0 + 0.01, 0))
                if debug: print(f'ADC info: {ADC_info}'); print(f'data rate: {data_rate_str}, {self.data_rate}Hz')

            
            except:
                raise ValueError("Error while parsing metadata")

            # Load sensor data
            # Note: Using skiprows ensures we read only the data section, cross-platform compatible with pandas.
            data = pd.read_csv(self.data_csv_path, skiprows=data_index + 1)

        # Re-pack all data in appropriate data-types
        for l in self.sensor_labels:
            sensor_data = {'time': data[f'Time_{l}_sec'].to_numpy(dtype=np.float64),
                           'strain_1_raw': data[f'Strain_{l}1_raw'].to_numpy(dtype=np.int32),
                           'strain_2_raw': data[f'Strain_{l}2_raw'].to_numpy(dtype=np.int32)}
            self.data_dict[f'Sensor_{l}'].update(sensor_data)

        split_colons = data['Processed_Time'].str.split(':', expand=True)
        if split_colons.shape[1] != 3:
            raise ValueError("Invalid 'Processed Time' format: Expected hh:mm:ss.us")
        split_seconds = split_colons[2].str.split('.', expand=True)
        if split_seconds.shape[1] != 2:
            raise ValueError("Invalid 'Processed Time' format: Expected hh:mm:ss.us")
        
        hh = split_colons[0].astype(np.int64)
        mm = split_colons[1].astype(np.int64)
        ss = split_seconds[0].astype(np.int64)
        us = split_seconds[1].astype(np.int64)
        processed_time = (hh*np.timedelta64(1,'h') + mm*np.timedelta64(1, 'm')
                           + ss*np.timedelta64(1, 's') + us*np.timedelta64(1, 'us'))
        self.data_dict['Processed Time'] = processed_time.to_numpy(dtype='timedelta64[us]')

    def describe_channels(self, time_cutoff=1.0):
        for l in self.sensor_labels:
            s = self.data_dict[f'Sensor_{l}']
            avg_initial_value_1_raw = np.average(s['strain_1_raw'][s['time'] <= time_cutoff])
            avg_initial_value_2_raw = np.average(s['strain_2_raw'][s['time'] <= time_cutoff])
            avg_end_value_1_raw = np.average(s['strain_1_raw'][s['time'] >= s['time'][-1] - time_cutoff])
            avg_end_value_2_raw = np.average(s['strain_2_raw'][s['time'] >= s['time'][-1] - time_cutoff])
            self.data_dict[f'Sensor_{l}']['ini_1'] = avg_initial_value_1_raw
            self.data_dict[f'Sensor_{l}']['ini_2'] = avg_initial_value_2_raw
            self.data_dict[f'Sensor_{l}']['end_1'] = avg_end_value_1_raw
            self.data_dict[f'Sensor_{l}']['end_2'] = avg_end_value_2_raw

    def filter_channels(self, window=None, order=1):
        if window is None:
            window = self.filter_window
        
        for l in self.sensor_labels:
            s = self.data_dict[f'Sensor_{l}']
            self.data_dict[f'Sensor_{l}']['strain_1_filter'] = savgol_filter(s['strain_1_raw'], window, order)
            self.data_dict[f'Sensor_{l}']['strain_2_filter'] = savgol_filter(s['strain_2_raw'], window, order)

    def plot_raw_strains(self, sensors='A,B,C,D,E', return_figs=False):
        sensors_to_plot = sensors.split(',')

        removed = [label for label in sensors_to_plot if label not in self.sensor_labels]
        for label in removed:
            print(f"Sensor {label} not in CSV data")
        sensors_to_plot = [label for label in sensors_to_plot if label in self.sensor_labels]

        figs = []  # List to collect figures for optional return

        for i, l in enumerate(sensors_to_plot):
            s = self.data_dict[f'Sensor_{l}']
            if not hasattr(s, 'ini_1'):
                self.describe_channels()
            if not hasattr(s, 'strain_1_filter'):
                self.filter_channels()

            fig, ax = plt.subplots(1,2, sharex=True, figsize=(12, 8))
            ax[0].plot(s['time'], s['strain_1_raw'], c='C0', linewidth=0.5, label=f'{l}1_raw')
            ax[0].plot(s['time'], s['strain_1_filter'], c='C1', linewidth=1.0, label=f'{l}1_filter')
            ax[0].axhline(s['ini_1'], c='red', linewidth=0.2)
            ax[0].axhline(s['end_1'], c='green', linewidth=0.2)
            ax[0].set_xlabel('Time (s)')
            ax[0].set_ylabel(r'ADC Integer Value ($\pm 2^{23}$)')
            ax[0].yaxis.set_major_formatter(StrMethodFormatter('{x:,}'))
            ax[0].legend(loc='upper right')

            ax[1].plot(s['time'], s['strain_2_raw'], c='C0', linewidth=0.5, label=f'{l}2_raw')
            ax[1].plot(s['time'], s['strain_2_filter'], c='C1', linewidth=1.0, label=f'{l}2_filter')
            ax[1].axhline(s['ini_2'], c='red', linewidth=0.2)
            ax[1].axhline(s['end_2'], c='green', linewidth=0.2)
            ax[1].set_xlabel('Time(s)')
            ax[1].yaxis.set_major_formatter(StrMethodFormatter('{x:,}'))
            ax[1].legend(loc='upper right')

            fig.suptitle(f'Sensor S/N: {s['SN']}, '+
                         f'Cycle Load: {self.cycle_force}{self.cycle_force_units}, '+
                         f'Test # in Session: {self.test_number}. '+
                         f'Pre-rest Time: {self.test_rest}')
            fig.tight_layout()
            figs.append(fig)  # Collect figure for return if requested

        if return_figs:
            return figs


if __name__ == "__main__":
    data = HiSTIFFSData(date="2026-01-03", time="215113", debug=True)
    if data.exists:
        data.plot_raw_strains()
        plt.show()