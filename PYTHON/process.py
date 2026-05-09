# This file reads data from a CSV and processes it with multiple methods and options availible to the user
# The CSV must have columns titled like 'Time_A_sec, Strain_A1_raw, Strain_A2_raw...' and so on for each lettered sensor
# Or columns like 'Time_A_sec, Strain_A1_V, Strain_A2_V...'. This is raw ADC values.
# The __init__() will automatically detect which type is present, or if both are, defaults to raw option (more efficient/precise).
# Calibration is done with raw values. mV option is for human comprehension and only viable for displays, not in calculations.

# Standard libraries
import os
import csv
from pathlib import Path
import copy
import keyboard
import time

# Installed packages
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import SpanSelector, Button, RadioButtons
from matplotlib.ticker import StrMethodFormatter
import matplotlib.gridspec as gridspec
from matplotlib.colors import ListedColormap
import pandas as pd
from scipy.signal import savgol_filter

# Workspace scripts
from config import Config

class HiSTIFFSData:
    # === load raw data and calculate force & position ===
    def __init__(self, date, time, nano_label='01', debug=False, file_path=None):
        # Fully cross-platform: uses pathlib.Path everywhere (Win / Ubuntu / RPi5 identical)
        self.nano_label = nano_label
        self.exists = False
        self.has_force_pos = False
        self.colors = ['r', 'g', 'c', 'y', 'm']
        self.sensor_starts = np.array([0.0, 142.47, 345.33, 508.12, 670.59]) * 1e-3
        # self.sensor_starts_dy = np.array([0.0, 141.36, 335.09, 496.19, 650.19]) * 1e-3  # v3.1
        self.sensor_starts_dy = np.array([0.0, 0.0, 362.18, 545.37, 699.37]) * 1e-3     # v3.2
        self.min_pos = 0.05
        self.fp_thresh = 0.10
        self.yaw = np.radians(20)
        self.rel_deflection = 0.06 # m, parallel distance between frist and third sensor
        self.height = 0.785  # m, height of contact point between probe and stalk
        self.stalk_spacing = 0.1524 # m, physical spacing between stalks

        if file_path is None:
            self.date = date
            self.time = time
            self.data_csv_path = Config.RAW_DATA_BASE / self.date / f"{self.date}_{self.time}_{self.nano_label}.csv"
        else:
            self.data_csv_path = Path(file_path).resolve()   # Path normalises \ / automatically
            filename = self.data_csv_path.name               # no replace('\\','/') needed
            file_parts = filename.rsplit('_', 2)
            if len(file_parts) < 3:
                raise ValueError("Invalid filename format for raw data file")
            self.date = file_parts[0]
            self.time = file_parts[1]

        self.stalks_csv_path = Config.RAW_DATA_BASE / self.date / f"stalks_{self.time}_{self.nano_label}.csv"

        if not self.data_csv_path.exists():   # Path.exists() works on all three OSes
            self.exists = False
            print(f"No such data file at: {self.data_csv_path}")
            return
        
        # Get all data from file
        with open(self.data_csv_path, 'r') as f:
            self.exists = True
            self.csv_reader = csv.reader(f)
            self.parse_metadata(debug)
            # Load actual data — pandas accepts Path directly (cross-platform)
            data = pd.read_csv(self.data_csv_path, skiprows=self.data_index + 1)

        # Re-pack all data in appropriate data-types
        self.repack_data(data)

        print('Raw data reading complete\n')

    def parse_metadata(self, debug):
        # Find metadata end and data start tags — unchanged, works on all OSes
        check1 = False
        self.data_index = -1
        self.header_rows = []
        for row_index, row in enumerate(self.csv_reader):
            self.header_rows.append(row)
            if check1 and len(row) == 1 and row[0].strip() == Config.DATA_MARKER:
                self.data_index = row_index
                break
            if len(row) == 1 and row[0].strip() == Config.HEADER_MARKER:
                check1 = True
                continue
            else: 
                check1 = False
        if self.data_index == -1: raise ValueError("No marker for end_metadata/start_data in CSV")

        # Parse required header data
        try:
            # ADS1220 info — unchanged
            ADC_info = next((row for row in self.header_rows if row and "Analog" in row[0]), None)
            data_rate_str = next((s.split()[-1] for s in ADC_info if 'Rate' in s), None)
            self.data_rate = int(data_rate_str.replace('DR_', '').replace('SPS', ''))
            self.filter_window = int(round(self.data_rate/20.0 + 0.01, 0))
            if debug: print(f'\nADC info: {ADC_info}'); print(f'data rate: {data_rate_str}, {self.data_rate}Hz')

            self.data_dict = {}

            # Test info — unchanged
            test_info = next((row for row in self.header_rows if row and "Test Type" in row[0]), None)
            self.test_type = ' '.join(test_info[0].split()[2:]) if test_info else 'Unknown'
            if debug: print(f'\nTest info: {test_info}'); print(f"test type: {self.test_type}")
            
            if self.test_type == 'Force Cycle':
                self.cycle_force = next((s.split()[-1] for s in test_info if "Force:" in s), None)
                s = self.cycle_force
                self.cycle_force, self.cycle_force_units = next(((s[:i], s[i:]) for i in range(1, len(s)) 
                                                                    if s[i].isalpha() and s[i-1].isdigit()), (s, ''))
                next_row = next((row for row in self.header_rows if row and "Test Number" in row[0]), None)
                self.test_number = int(next_row[0].split()[-1])
                self.test_rest = next_row[1].split()[-1]
                if debug: print(f'cycle force: {self.cycle_force}, units: {self.cycle_force_units}')
            
            # === SENSORS INFO — FIXED: fully dynamic, no hard-coded 5-sensor override ===
            sensors_info = next((row for row in self.header_rows if row and "Number of ICB-Sensors" in row[0]), None)
            print(f'\nSensors info: {sensors_info}')

            self.num_sensors = None
            self.sensor_labels = None
            self.sensor_sns = None
            if sensors_info:
                number_part, labels_part, sns_part = sensors_info
                try:
                    num_str = number_part.split(":")[-1].strip()
                    self.num_sensors = int(num_str)

                    label_str = labels_part.split(":")[-1].strip()
                    self.sensor_labels = [lbl.strip().upper() for lbl in label_str.split(' ') if lbl.strip() in 'ABCDE']
                    
                    sns_str = sns_part.split(":")[-1].strip()
                    self.sensor_sns = [sns for sns in sns_str.split(' ')]

                    if debug: print('Successfully parsed sensor info')
                except:
                    print('Unable to parse sensors info. Using defaults')
                    self.num_sensors = 5
                    self.sensor_labels = ['A','B','C','D','E']
                    self.sensor_sns = ['001', '002', '003', '004', '005']
            else:
                print('No sensors info found. Using defaults')
                self.num_sensors = 5
                self.sensor_labels = ['A','B','C','D','E']
                self.sensor_sns = ['001', '002', '003', '004', '005']

            if debug: 
                print(f"num sensors: {len(self.sensor_labels)}")
                print(f"sensor labels: {self.sensor_labels}")
                print(f'sensor serial#s: {self.sensor_sns}')

            if self.test_type == 'Calibration':
                fp_info = next((row for row in self.header_rows if row and "Loads (N):" in row[0]), None)
                print(f'\nForce/Position info: {fp_info}')
                loads_part, positions_part = fp_info
                
                loads_str = loads_part.split(':')[-1].strip()
                self.loads = [float(load) for load in loads_str.split(' ')]

                positions_str = positions_part.split(':')[-1].strip()
                self.positions = [float(pos) for pos in positions_str.split(' ')]

                print(f'Loads (N): {self.loads}, Positions (mm): {self.positions}')

                # Create placeholder for this sensor for .update() method used later
                for l in self.sensor_labels:  
                    self.data_dict[f'Sensor_{l}'] = {'place_holder': None}
                return

            # Sensor info — now uses the dynamic list
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
                        break

            # Sensor(s) calibration — unchanged, already loops over labels
            calibration_dict = {}
            for l in self.sensor_labels:
                coeffs = [f'k{l}1', f'd{l}1', f'c{l}1', f'k{l}2', f'd{l}2', f'c{l}2']
                cal_info = next((row for row in self.header_rows if row and f"ICB-Sensor {l}'s Latest Calibration" in row[0]), None)
                c = []
                for coeff in coeffs:
                    c.append(next((s.split()[-1] for s in cal_info if coeff in s), None) if cal_info else None)

                coeff_dict = {f"k{l}1": c[0], f"d{l}1": c[1], f"c{l}1": c[2], 
                              f"k{l}2": c[3], f"d{l}2": c[4], f"c{l}2": c[5]}
                calibration_dict[f"Sensor_{l}"] = coeff_dict
                self.sensor_info_dict[f'Sensor_{l}'].update(coeff_dict)

        except Exception as e:
            raise ValueError(f"Error while parsing metadata: {e}")

    def repack_data(self, data):
        for i, l in enumerate(self.sensor_labels):
            sensor_data = {'time': data[f'Time_{l}_sec'].to_numpy(dtype=np.float64),
                           'strain_1_raw': data[f'Strain_{l}1_raw'].to_numpy(dtype=np.int32),
                           'strain_2_raw': data[f'Strain_{l}2_raw'].to_numpy(dtype=np.int32)}
            self.data_dict[f'Sensor_{l}'].update(sensor_data)
            if l in ['A', 'C', 'E']:
                self.data_dict[f'Sensor_{l}']['length'] = 0.100
                self.data_dict[f'Sensor_{l}']['type'] = 'straight'
            elif l in ['B', 'D']:
                self.data_dict[f'Sensor_{l}']['length'] = 0.120
                self.data_dict[f'Sensor_{l}']['type'] = 'angled'
                self.data_dict[f'Sensor_{l}']['abs(yaw)_rad'] = self.yaw
            if l == 'C':
                self.data_dict[f'Sensor_{l}']['parallel_deflection'] = self.rel_deflection
            self.data_dict[f'Sensor_{l}']['start_pos'] = self.sensor_starts_dy[i]

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
        print(f'Gathering raw data descriptors over first {time_cutoff} second(s)...')
        for l in self.sensor_labels:
            s = self.data_dict[f'Sensor_{l}']
            t_min = np.min(s['time'])
            s_time_cutoff = time_cutoff + t_min

            avg_initial_value_1_raw = np.average(s['strain_1_raw'][s['time'] <= s_time_cutoff])
            avg_initial_value_2_raw = np.average(s['strain_2_raw'][s['time'] <= s_time_cutoff])
            avg_end_value_1_raw = np.average(s['strain_1_raw'][s['time'] >= s['time'][-1] - s_time_cutoff])
            avg_end_value_2_raw = np.average(s['strain_2_raw'][s['time'] >= s['time'][-1] - s_time_cutoff])
            self.data_dict[f'Sensor_{l}']['ini_1'] = avg_initial_value_1_raw
            self.data_dict[f'Sensor_{l}']['ini_2'] = avg_initial_value_2_raw
            self.data_dict[f'Sensor_{l}']['end_1'] = avg_end_value_1_raw
            self.data_dict[f'Sensor_{l}']['end_2'] = avg_end_value_2_raw

    def filter_channels(self, window=None, order=1):
        if window is None:
            window = self.filter_window
        
        print(f'Filtering raw data with window size {window}...')
        for l in self.sensor_labels:
            s = self.data_dict[f'Sensor_{l}']
            self.data_dict[f'Sensor_{l}']['strain_1_filter'] = savgol_filter(s['strain_1_raw'], window, order)
            self.data_dict[f'Sensor_{l}']['strain_2_filter'] = savgol_filter(s['strain_2_raw'], window, order)

    def calc_force_position(self, filter_out=False, clip=True):
        print('Calculating force and position from raw data...')
        for l in self.sensor_labels:
            try:
                s = self.data_dict[f'Sensor_{l}']
                # Ensure initial values exist (cross-platform safe)
                if 'ini_1' not in s or 'ini_2' not in s:
                    self.describe_channels()

                if 'strain_1_filter' not in s or 'strain_2_filter' not in s:
                    self.filter_channels()

                k1 = float(self.sensor_info_dict[f'Sensor_{l}'][f'k{l}1'])
                d1 = float(self.sensor_info_dict[f'Sensor_{l}'][f'd{l}1'])
                c1 = float(self.sensor_info_dict[f'Sensor_{l}'][f'c{l}1'])
                c1 = s['ini_1']  # override with actual initial value for better accuracy
                k2 = float(self.sensor_info_dict[f'Sensor_{l}'][f'k{l}2'])
                d2 = float(self.sensor_info_dict[f'Sensor_{l}'][f'd{l}2'])
                c2 = float(self.sensor_info_dict[f'Sensor_{l}'][f'c{l}2'])
                c2 = s['ini_2']  # override with actual initial value for better accuracy

                num = k2*(s['strain_1_filter'] - c1) - k1*(s['strain_2_filter'] - c2)
                den = k1*k2*(d2 - d1)
                s['force'] = np.clip(np.where(np.abs(den) > 1e-12, num/den, 0.0), -84, 84) # santity check in Newtons

                num = k2*d2*(s['strain_1_filter'] - c1) - k1*d1*(s['strain_2_filter'] - c2)
                den = k2*(s['strain_1_filter'] - c1) - k1*(s['strain_2_filter'] - c2)                
                for i, val in enumerate(den):
                    if val >= 1e-6 and val <= 1e16:
                        den[i] = val
                    else:
                        den[i] = 1.0
                s['position'] = np.clip(np.where(np.abs(den) > 1e9, num/den, 0.0), 0.03, 0.15) # sanity check in centimeters  

                if filter_out:
                    s['force'] =    savgol_filter(s['force'],    self.filter_window, 1)
                    s['position'] = savgol_filter(s['position'], self.filter_window, 1)

                if clip:    # set all non-sensical fp data points to 0
                    mask = (s['position'] >= self.min_pos) & (s['position'] <= s['length'])
                    s['force'] = np.where(mask, s['force'], 0.0)
                    s['position'] = np.where(mask, s['position'], 0.0)
            
            except Exception as e:
                print(f"Missing data for Sensor {l}. Cannot calculate force/position. Error: {e}")
                continue

        print('Calculate force and position complete.\n')
        self.has_force_pos = True

    # === gather and clean stalk data ===
    def _cleanup_sensor_traces(self, num_iters=3):
        print('Cleaning sensor traces before stalk association...')

        self.clean_dict = copy.deepcopy(self.data_dict)

        # for l in self.sensor_labels:
        #     s = self.clean_dict[f'Sensor_{l}']
        #     mask = (s['position'] >= self.min_pos + 0.005*0)
        #     # s['time'] = np.where(mask, s['time'], 0.0)
        #     s['force'] = np.where(mask, s['force'], 0.0)
        #     s['position'] = np.where(mask, s['position'], 0.0)

        print(f'\tEnforcing decreasing sensor position with {num_iters} passes')
        for _ in range(num_iters):
            for l in self.sensor_labels:
                s = self.clean_dict[f'Sensor_{l}']
                diffs = np.diff(s['position'], append=0.0)
                mask = (diffs <= 0.0001) # allow 1mm slide back between data points
                # s['time'] = np.where(mask, s['time'], 0.0)
                s['force'] = np.where(mask, s['force'], 0.0)
                s['position'] = np.where(mask, s['position'], 0.0)
    
        print(f'\tEnforcing {self.fp_thresh} "force x position" step size with {num_iters} passes')
        for _ in range(num_iters):
            for l in self.sensor_labels:
                s = self.clean_dict[f'Sensor_{l}']
                mask = self._get_sensor_fp_mask(s, self.fp_thresh)
                # s['time'] = np.where(mask, s['time'], 0.0)
                s['force'] = np.where(mask, s['force'], 0.0)
                s['position'] = np.where(mask, s['position'], 0.0)


        for l in self.sensor_labels:
            s = self.clean_dict[f'Sensor_{l}']
            mask_nz = s['position'] > 0.0
            f_nz = s['force'][mask_nz]
            p_nz = s['position'][mask_nz]
 
            std_fnz = np.std(f_nz)
            if std_fnz >= 2.0:
                mask_remove_floor = s['force'] >= 2.0 
                s['force'] = np.where(mask_remove_floor, s['force'], 0.0)
                s['position'] = np.where(mask_remove_floor, s['position'], 0.0)

            # plt.figure()
            # plt.title(f'{np.std(f_nz)}')
            # plt.hist(f_nz, bins=30)

    def _get_sensor_fp_mask(self, sensor, fp_thresh=None):
        if fp_thresh is None:
            fp_thresh = self.fp_thresh

        points = np.column_stack((sensor['force'], sensor['position']))
        diffs = points[1:] - points[:-1]
        fp_gap = np.linalg.norm(diffs, axis=1)
        fp_gap = np.append(fp_gap, 0)

        return (fp_gap <= fp_thresh)

    def _find_segments(self, sensor, time_gap: float=0.2):
        """
        Identifies start and end indices of active runs (segments) based solely on time differences.
        
        A run is closed whenever a value in the 'diffs' array exceeds 0.2.
        This is the simplified implementation requested.
        
        Parameters:
        - sensor: dict containing 'position' and 'time' arrays.
        - time_gap: float of seconds between end of one segment and start of another.
        
        Returns:
        - List of tuples: Each tuple is (start_index, end_index) for a run.
        - Number of segments found.
        """
        s = sensor
        segments = []
        if len(s['position']) < 2:
            return segments, 0

        # Identify indices where the sensor is active (position > 0)
        active_idx = np.where(s['position'] > 0.0)[0]
        if len(active_idx) == 0:
            return segments, 0

        # Compute the 'diffs' array (time differences between consecutive active points)
        active_time = s['time'][active_idx]
        diffs = np.diff(active_time)

        # Keep the original debug visualizations of the diffs array
        # plt.figure()
        # plt.hist(active_time, bins=100)
        # plt.figure()
        # plt.scatter(active_time[:-1], diffs)  # diffs has length len(active_time) - 1

        # Group into runs using only the diffs array
        start = active_idx[0]
        end = active_idx[0]

        for i in range(len(diffs)):
            if diffs[i] > time_gap:
                # Close the current run
                segments.append((start, end))
                # Start a new run
                start = active_idx[i + 1]
                end = active_idx[i + 1]
            else:
                # Continue the current run
                end = active_idx[i + 1]

        # Add the final run
        segments.append((start, end))

        # plt.scatter(range(len(binary)), binary, s=0.5)
        # for run in segments:
        #     plt.axvline(run[0], c='green')
        #     plt.axvline(run[1], c='red')
        # plt.show()
        print(len(segments))

        return segments, len(segments)

    def _associate_segments(self):
        # get probe velocity from sensor position data
        velocities_y = []
        # for l in self.sensor_labels:
        #     if l in ['B', 'C', 'D']: continue
        #     s = copy.deepcopy(self.clean_dict[f'Sensor_{l}'])
        #     for start, end in s['segments_idx']:
        #         time = s['time'][start:end]
        #         pos = s['position'][start:end]

        #         dpos_dt, _ = np.polyfit(time, pos, deg=1)
        #         print(dpos_dt)
                
        #         if s['type'] == 'straight': vel_y = -dpos_dt
        #         elif s['type'] == 'angled': vel_y = -dpos_dt *np.cos(s['abs(yaw)_rad'])
        #         else: raise ValueError(f'Sensor type in probe not available for Sensor_{l}. "straight" or "angled"')
        #         velocities_y.append(vel_y)

        self.avg_probe_vel = 0.12# np.median(velocities_y)
        self.nominal_stalk_gap = self.stalk_spacing / self.avg_probe_vel 
        # plt.figure()
        # plt.hist(velocities_y, bins=50)
        # plt.axvline(self.avg_probe_vel, c='red')
        print(self.avg_probe_vel)
        # plt.show()

        # compute time offsets to align each sensor
        for i, l in enumerate(self.sensor_labels):
            s = self.clean_dict[f'Sensor_{l}']
            start_pos = self.sensor_starts_dy[i]
            s['time_offset'] = s['start_pos'] / self.avg_probe_vel

            print(i, l, s['start_pos'], s['time_offset'])

        # self.plot_force_position(filter_level='clean', offset_time=True)

        # associate stalks across sensors
        s_top = self.clean_dict['Sensor_A']
        self.stalk_sensor_starts = []
        self.stalk_sensor_ends = []
        for start_top, end_top in s_top['segments_idx']:
            ref_time = s_top['time'][start_top] - s_top['time_offset']
            sensor_starts = {'A': start_top, 'B': 0, 'C': 0, 'D': 0, 'E': 0}
            sensor_ends = {'A': end_top, 'B': 0, 'C': 0, 'D': 0, 'E': 0}
            for l in self.sensor_labels:
                if l == 'A': continue
                s_test = self.clean_dict[f'Sensor_{l}']
                for start_test, end_test in s_test['segments_idx']:
                    if abs(ref_time - (s_test['time'][start_test] - s_test['time_offset'])) <= self.nominal_stalk_gap/2.5:
                        sensor_starts[f'{l}'] = start_test
                        sensor_ends[f'{l}'] = end_test
                        break
            self.stalk_sensor_starts.append(sensor_starts)
            self.stalk_sensor_ends.append(sensor_ends)

        # print(self.stalk_sensor_starts)
        # print(len(self.stalk_sensor_starts))

    def _save_stalk_association(self):
        # Write stalks data to new CSV of same time in name
        with open(self.stalks_csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([Config.STALK_TIMES_MARKER])

            # Wide-format header (one column pair per sensor, always in A-B-C-... order)
            header = ['Stalk']
            for l in self.sensor_labels:          # ordered_sensors from enclosing scope
                header.extend([f'{l}_Start', f'{l}_End'])
            writer.writerow(header)

            # One row per stalk, all sensors present as columns
            for i, (sensor_starts, sensor_ends) in enumerate(zip(self.stalk_sensor_starts, self.stalk_sensor_ends)):
                row = [i]
                for l in self.sensor_labels:
                    s = self.data_dict[f'Sensor_{l}']
                    if sensor_starts[l] != 0 and sensor_ends[l] != 0:
                        start, end = s['time'][sensor_starts[l]] - self.nominal_stalk_gap/4, s['time'][sensor_ends[l]] + self.nominal_stalk_gap/4
                        row.extend([f"{start:.3f}", f"{end:.3f}"])
                    else:
                        row.extend(['', ''])       # missing sensor for this stalk
                writer.writerow(row)

        print(f"Status: All stalk spans saved to {self.stalks_csv_path} (wide format)")



    def detect_stalks(self, time_gap=0.2, min_seg=25, plot=False):
        if not self.exists:
            print("Status: No data loaded - call HiSTIFFSData(...) first")
            return

        if not self.has_force_pos:
            self.calc_force_position()

        self._cleanup_sensor_traces()
        # self.plot_force_position(filter_level='valid')
        # self.plot_force_position(filter_level='clean')
        

        print(f'\tBreaking sensor traces into segments with time gap {time_gap}s and min size {min_seg} points')
        for l in self.sensor_labels:
            s = self.clean_dict[f'Sensor_{l}']
            sc = copy.deepcopy(self.clean_dict[f'Sensor_{l}'])
            s['segments_idx'], s['num_stalks'] = self._find_segments(sc, time_gap)

        self._associate_segments()
        self._save_stalk_association()


        if plot:
            n_rows = len(self.sensor_labels)
            fig, axs = plt.subplots(n_rows, 2, figsize=(12, 1.5 * n_rows),
                                    sharex=True, squeeze=False)
            fig.suptitle("Stalk Detection", fontsize=12)

            # Plot lines (stored for live xdata updates on shift)
            lines = {}
            for i, l in enumerate(self.sensor_labels):
                s = self.data_dict[f'Sensor_{l}']
                sc = self.clean_dict[f'Sensor_{l}']

                # Force (left column) – selection target
                line_f = axs[i, 0].scatter(sc['time']- sc['time_offset'], sc['force'], color=self.colors[i % len(self.colors)],
                                        s=1, label=f'{l} Force (N)')
                axs[i, 0].set_ylabel(f'{l} Force (N)')
                # axs[i, 0].legend(loc='upper right')
                axs[i, 0].grid(True, alpha=0.3)

                # Position (right column)
                line_p = axs[i, 1].scatter(sc['time'] - sc['time_offset'], sc['position'] * 1000, color=self.colors[i % len(self.colors)],
                                        s=1, label=f'{l} Position (mm)')
                for idxs in sc['segments_idx']:
                    axs[i, 1].axvline(sc['time'][idxs[0]]- sc['time_offset'], c='green', linewidth=1)
                    axs[i, 1].axvline(sc['time'][idxs[1]]- sc['time_offset'], c='red', linewidth=1)
                axs[i, 1].set_ylabel(f'{l} Position (mm)')
                # axs[i, 1].legend(loc='upper right')
                axs[i, 1].grid(True, alpha=0.3)

                lines[(l, 'force')] = line_f
                lines[(l, 'pos')] = line_p

            # Bottom row gets x-labels
            axs[-1, 0].set_xlabel('Time (s)')
            axs[-1, 1].set_xlabel('Time (s)')

            fig.tight_layout(rect=[0, 0.04, 1, 0.96])  # leave bottom space for controls

    def interactive_detect_stalks(self):
        '''
        Creates an interactive figure to correlate sensors and select stalks from force/position plots

        1. User selects spans on each sensor for stalk 1 (5 unique spans per stalk)
        2. When all 5 spans are selected for stalk 1, save time spans to Raw_Data/{date}/{time}.csv for later retrieval
        3. Repeat selection until user indicates no more stalks
        '''

        if not self.exists:
            print("Status: No data loaded - call HiSTIFFSData(...) first")
            return

        if not self.has_force_pos:
            self.calc_force_position()  # ensures force/position exist

        # ── Sensor setup (dynamic, respects parsed labels) ─────────────────────
        sensors = self.sensor_labels
        sensor_order = 'ABCDE'
        ordered_sensors = sorted(sensors, key=lambda x: sensor_order.find(x) if x in sensor_order else 999)
        n_rows = len(ordered_sensors)
        if n_rows == 0:
            print("Status: No sensors to process")
            return

        # ── Figure with exact same structure as plot_force_position combined=True ──
        fig, axs = plt.subplots(n_rows, 2, figsize=(12, 1.5 * n_rows),
                                sharex=True, squeeze=False)
        fig.suptitle("Interactive Stalk Detection", fontsize=12)

        # State (all on self so callbacks and re-entrancy work cleanly)
        self.current_stalk = 1
        self.selected_spans = {}          # stalk_num → {sensor: (start_orig_s, end_orig_s)}
        self.current_selections = {l: None for l in ordered_sensors}  # temporary plot-coord spans
        self.permanent_shades = {}        # stalk_num → {sensor: axvspan_artist}
        self.span_selectors = []

        # Plot lines (stored for live xdata updates on shift)
        lines = {}
        for i, l in enumerate(ordered_sensors):
            s = self.data_dict[f'Sensor_{l}']

            force = s['force'].copy()
            position = s['position'].copy()
            if l in ['A', 'C', 'E']:
                mask = (position > 0.10) | (position < 0.05)
            elif l in ['B', 'D']:
                mask = (position > 0.12) | (position < 0.05)
            force[mask] = 0.0
            position[mask] = 0.0

            # Force (left column) – selection target
            line_f = axs[i, 0].scatter(s['time'], force, color=self.colors[i % len(self.colors)],
                                     s=1, label=f'{l} Force (N)')
            axs[i, 0].set_ylabel(f'{l} Force (N)')
            # axs[i, 0].legend(loc='upper right')
            axs[i, 0].grid(True, alpha=0.3)

            # Position (right column)
            line_p = axs[i, 1].scatter(s['time'], position * 1000, color=self.colors[i % len(self.colors)],
                                     s=1, label=f'{l} Position (mm)')
            axs[i, 1].set_ylabel(f'{l} Position (mm)')
            # axs[i, 1].legend(loc='upper right')
            axs[i, 1].grid(True, alpha=0.3)

            lines[(l, 'force')] = line_f
            lines[(l, 'pos')] = line_p

        # Bottom row gets x-labels
        axs[-1, 0].set_xlabel('Time (s)')
        axs[-1, 1].set_xlabel('Time (s)')

        fig.tight_layout(rect=[0, 0.1, 1, 0.96])  # leave bottom space for controls

        # ── SpanSelectors – one per force subplot (yellow during drag) ───────────
        def make_onselect(label):
            def onselect(min_t, max_t):
                self.current_selections[label] = (min_t, max_t)  # stored in plotted (shifted) coords
            return onselect

        for i, l in enumerate(ordered_sensors):
            sp = SpanSelector(axs[i, 0], make_onselect(l), 'horizontal',
                              useblit=True, props=dict(alpha=0.35, facecolor='yellow'),
                              interactive=True)
            self.span_selectors.append(sp)

        # ── Control widgets (touch-friendly for RPi 5) ───────────────────────────
        # Bottom row layout (all percentages of figure)
        ax_prev    = fig.add_axes([0.05, 0.01, 0.09, 0.05])
        ax_next    = fig.add_axes([0.15, 0.01, 0.09, 0.05])
        ax_confirm = fig.add_axes([0.26, 0.01, 0.09, 0.05])
        ax_finish  = fig.add_axes([0.40, 0.01, 0.09, 0.05])

        btn_prev    = Button(ax_prev,    '← Prev Stalk')
        btn_next    = Button(ax_next,    'Next Stalk →')
        btn_confirm = Button(ax_confirm, 'Confirm Stalk')
        btn_finish  = Button(ax_finish,  'Finish & Save')

        # ── Callbacks ────────────────────────────────────────────────────────────
        def prev_stalk(event):
            if self.current_stalk > 1:
                self.current_stalk -= 1
                update_title()
        
        def next_stalk(event):
            self.current_stalk += 1
            if self.current_stalk not in self.selected_spans:
                self.selected_spans[self.current_stalk] = {}
            update_title()
        
        def confirm_stalk(event):
            stalk = self.current_stalk
            if stalk not in self.selected_spans:
                self.selected_spans[stalk] = {}

            updated_any = False
            for l in ordered_sensors:
                if self.current_selections[l] is not None:
                    min_plot, max_plot = self.current_selections[l]

                    # Remove old vlines for this stalk/sensor
                    if (stalk in self.permanent_shades and
                            l in self.permanent_shades[stalk]):
                        try:
                            for vl in self.permanent_shades[stalk][l]:
                                vl.remove()
                        except:
                            pass

                    # Add two ultra-light axvlines (start + end) — this is the perf fix
                    ax_idx = ordered_sensors.index(l)
                    ax = axs[ax_idx, 0]
                    start_vl = ax.axvline(min_plot, color='lime', lw=2.0, ls='-', alpha=0.7)
                    end_vl   = ax.axvline(max_plot, color='lime', lw=2.0, ls='-', alpha=0.7)

                    self.permanent_shades.setdefault(stalk, {})[l] = [start_vl, end_vl]

                    self.selected_spans[stalk][l] = (min_plot, max_plot)
                    updated_any = True
                    self.current_selections[l] = None

            if updated_any:
                print(f"Stalk {stalk} confirmed ({len(self.selected_spans[stalk])} sensors)")
            else:
                print("Drag on a force plot first")
            fig.canvas.draw_idle()
            next_stalk(None)   # auto-advance like original
        
        def finish(event):
            if not self.selected_spans:
                print("Status: No stalks selected")
                plt.close(fig)
                return

            # Write stalks data to new CSV of same time in name
            with open(self.stalks_csv_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([Config.STALK_TIMES_MARKER])

                # Wide-format header (one column pair per sensor, always in A-B-C-... order)
                header = ['Stalk']
                for l in ordered_sensors:          # ordered_sensors from enclosing scope
                    header.extend([f'{l}_Start', f'{l}_End'])
                writer.writerow(header)

                # One row per stalk, all sensors present as columns
                for stalk_num in sorted(self.selected_spans.keys()):
                    row = [stalk_num]
                    for l in ordered_sensors:
                        if l in self.selected_spans[stalk_num]:
                            start, end = self.selected_spans[stalk_num][l]
                            row.extend([f"{start:.3f}", f"{end:.3f}"])
                        else:
                            row.extend(['', ''])       # missing sensor for this stalk
                    writer.writerow(row)

            print(f"Status: All stalk spans saved to {self.stalks_csv_path} (wide format)")
            plt.close(fig)
        
        btn_prev.on_clicked(prev_stalk)
        btn_next.on_clicked(next_stalk)
        btn_confirm.on_clicked(confirm_stalk)
        btn_finish.on_clicked(finish)

        # Keyboard shortcuts (works on all platforms including RPi touchscreen with external keyboard)
        def on_key(event):
            if event.key == 'n':
                next_stalk(None)
            elif event.key == 'p':
                prev_stalk(None)
            elif event.key == 'enter':
                confirm_stalk(None)
            elif event.key == 'q':
                finish(None)
        fig.canvas.mpl_connect('key_press_event', on_key)

        # Title helper
        def update_title():
            fig.suptitle(f"Interactive Stalk Detection - Stalk {self.current_stalk} "
                         f"• Drag on Force plot = select span "
                         f"• Confirm = lock spans (green) | ←/→ = change stalk | q = finish",
                         fontsize=11)
        update_title()

        # Initial draw
        fig.canvas.draw_idle()
        plt.show(block=True)

    def gather_stalk_traces(self):
        if not self.has_force_pos:
            self.calc_force_position()
        
        print("Gathering data for detected stalks' traces across all sensors...")
        path = self.stalks_csv_path
        with open(path, 'r') as f:
            csv_reader = csv.reader(f)
            for i, row in enumerate(csv_reader):
                if len(row) == 1 and row[0].strip() == Config.STALK_TIMES_MARKER:
                    data_idx = i
        df = pd.read_csv(path, skiprows=data_idx+1)
        self.num_stalks = len(df)

        self.stalks_time = []
        self.stalks_force = []
        self.stalks_position = []
        self.stalks_probe_position = [] 
        for i in range(self.num_stalks):
            stalk_time = {}
            stalk_force = {}
            stalk_position = {}
            stalk_probe_position = {}
            for j, l in enumerate(self.sensor_labels):
                s = self.data_dict[f'Sensor_{l}']
                mask = self._get_valid_mask(l, df.iloc[i])
                stalk_time[l] = s['time'][mask]
                stalk_force[l] = s['force'][mask]
                stalk_position[l] = s['position'][mask]
                stalk_probe_position[l] = (s['length'] - s['position'][mask]) + s['start_pos']

            self.stalks_time.append(stalk_time)
            self.stalks_force.append(stalk_force)
            self.stalks_position.append(stalk_position)
            self.stalks_probe_position.append(stalk_probe_position)

    def _cleanup_stalk_traces(self, num_iters=5):
        if not self.has_force_pos:
            self.calc_force_position()
        
        if not hasattr(self, 'stalks_time'):
            self.gather_stalk_traces()

        print('Cleaning stalk traces...')
        self.clean_stalks_time = copy.deepcopy(self.stalks_time)
        self.clean_stalks_force = copy.deepcopy(self.stalks_force)
        self.clean_stalks_position = copy.deepcopy(self.stalks_position)
        self.clean_stalks_probe_position = copy.deepcopy(self.stalks_probe_position)

        print(f'\tEnforcing increasing probe position with {num_iters*2} passes')
        for i in range(self.num_stalks):
            for _ in range(num_iters*2):
                for l in self.sensor_labels:
                    diffs = np.diff(self.clean_stalks_probe_position[i][l], append=0.0)
                    mask = (diffs >= 0.0)
                    self.clean_stalks_time[i][l] = self.clean_stalks_time[i][l][mask]
                    self.clean_stalks_force[i][l] = self.clean_stalks_force[i][l][mask]
                    self.clean_stalks_position[i][l] = self.clean_stalks_position[i][l][mask]
                    self.clean_stalks_probe_position[i][l] = self.clean_stalks_probe_position[i][l][mask]

        print(f'\tEnforcing {self.fp_thresh} "force x position" step size with {num_iters} passes')
        for i in range(self.num_stalks):
            for _ in range(num_iters):
                for l in self.sensor_labels:
                    if len(self.clean_stalks_time[i][l]) <= 10:
                        continue
                    mask = self._get_stalk_fp_mask(i, l, self.fp_thresh)
                    self.clean_stalks_time[i][l] = self.clean_stalks_time[i][l][mask]
                    self.clean_stalks_force[i][l] = self.clean_stalks_force[i][l][mask]
                    self.clean_stalks_position[i][l] = self.clean_stalks_position[i][l][mask]
                    self.clean_stalks_probe_position[i][l] = self.clean_stalks_probe_position[i][l][mask]

    def _get_valid_mask(self, sensor_label, time_bounds):
        l = sensor_label
        s = self.data_dict[f'Sensor_{l}']
        return (s['time'] > time_bounds[f'{l}_Start']) & \
               (s['time'] <= time_bounds[f'{l}_End']) & \
               (s['position'] >= self.min_pos) & (s['position'] <= s['length'])

    def _get_stalk_fp_mask(self, stalk_idx, sensor_label, fp_thresh=None):
        if fp_thresh is None:
            fp_thresh = self.fp_thresh
        
        if not hasattr(self, 'clean_stalks_time'):  # this catch is currently cicular. Won't work if triggered
            print('dang')
            self.gather_stalk_traces()
        
        # calculate the 2D force x position distance between neighboring points
        points = np.column_stack((self.clean_stalks_force[stalk_idx][sensor_label], 
                                  self.clean_stalks_position[stalk_idx][sensor_label]))
        differences = points[1:] - points[:-1]
        fp_gap = np.linalg.norm(differences, axis=1)
        fp_gap = np.append(fp_gap, 0)

        return (fp_gap <= fp_thresh)

    # === estimate stalk stiffness ===
    def estimate_all_stalks_stiffness(self, method='quasi-static average'):
        self.method = method
        if not hasattr(self, 'clean_stalks_time'):
            self._cleanup_stalk_traces()

        print(f'\nEstimating stiffness of {self.num_stalks} stalks with "{method}" method...')
        self.stiffnesses = np.empty(self.num_stalks, dtype=np.float64)
        self.estimates = []
        zipped = zip(self.clean_stalks_time, self.clean_stalks_force,
                         self.clean_stalks_position, self.clean_stalks_probe_position)
    
        parallel_deflection = self.data_dict[f'Sensor_C']['parallel_deflection']
        yaw_B = self.data_dict[f'Sensor_B']['abs(yaw)_rad']
        yaw_D = self.data_dict[f'Sensor_D']['abs(yaw)_rad']
        beam_constant = self.height**3 / 3
        for i, stalk_trace_data in enumerate(zipped):
            print(f'Processing stalk {i+1}')
            sensor_readings = {}
            for l in self.sensor_labels:
                sensor_readings[l] = self.read_stalk_on_sensor(stalk_trace_data, l)
            estimate_1 = (sensor_readings['C'] - sensor_readings['A']) / parallel_deflection
            estimate_2 = (sensor_readings['C'] - sensor_readings['E']) / parallel_deflection
            estimate_3 = sensor_readings['B'] / np.sin(yaw_B) * np.cos(np.radians(38))
            estimate_4 = sensor_readings['D'] / np.sin(yaw_D)

            estimates_raw = np.array([estimate_1, estimate_2, estimate_3, estimate_4]) * beam_constant
            # print(estimates_raw)
            estimates_filt = []
            for estimate in estimates_raw:
                if np.isnan(estimate):
                    continue
                estimates_filt.append(estimate)
            
            self.estimates.append(estimates_raw)
            self.stiffnesses[i] = np.average(estimates_filt)

        self.estimates = np.array(self.estimates)
        self.stiffnesses = np.array(self.stiffnesses)

    def read_stalk_on_sensor(self, stalk_trace_data, sensor_label):
        time, force, position, probe_position = stalk_trace_data
        l = sensor_label
        sensor_type = self.data_dict[f'Sensor_{l}']['type']

        if not np.any(force[l]):
            return np.nan
        
        if sensor_type == 'straight':
            mean = np.average(force[l])
            reading = mean
        elif sensor_type == 'angled':
            slope, y_intercept = np.polyfit(probe_position[l], force[l], deg=1)
            reading = abs(slope)

        return reading

    # === display data and output results ===
    def plot_detections(self, filter_level='valid'):
        if not self.has_force_pos:
            self.calc_force_position()
        
        if not hasattr(self, 'all_stalks_time'):
            self.gather_stalk_traces()

        if not hasattr(self, 'clean_stalks_time'):
            self._cleanup_stalk_traces()

        if filter_level == 'valid':
            zipped = zip(self.stalks_time, self.stalks_force,
                         self.stalks_position, self.stalks_probe_position)
        elif filter_level == 'clean':
            zipped = zip(self.clean_stalks_time, self.clean_stalks_force,
                         self.clean_stalks_position, self.clean_stalks_probe_position)
        else:
            raise ValueError('Invalid filter level. Only "valid" or "clean"')
        
        print(f'\nPlotting {self.num_stalks} stalk detections with {filter_level} filter...')
        count = 0
        for time, force, position, probe_position in zipped:
            # if count > 2:
            #     break
            count += 1
            fig = plt.figure(figsize=(8, 5))
            gs = gridspec.GridSpec(2, 2)
            ax1 = fig.add_subplot(gs[0, 0])
            ax2 = fig.add_subplot(gs[0, 1])
            ax3 = fig.add_subplot(gs[1, :])
            for start, c in zip(self.sensor_starts_dy, self.colors):
                ax3.axvline(start, c=c, linewidth=0.5)

            for l, c in zip(self.sensor_labels, self.colors):
                ax1.scatter(time[l], force[l], s=1, c=c)
                ax2.scatter(time[l], position[l], s=1, c=c)
                ax3.scatter(probe_position[l], force[l], s=1, c=time[l], cmap='viridis')

            ax1.set_ylim(0, 20)
            ax1.set_ylabel('Force (N)')
            ax1.set_xlabel('Time (s)')
            
            ax2.set_ylim(0, 0.15)
            ax2.set_ylabel('Sensor Position (m)')
            ax2.set_xlabel('Time (s)')
            
            ax3.set_ylim(0, 20)
            ax3.set_ylabel('Force (N)')
            ax3.set_xlabel('Probe Position (m)')

            fig.tight_layout()
  
    def plot_force_position(self, sensors='A,C,D,E', combined=True, return_figs=False, filter_level='valid', offset_time: bool=False):
        sensors_to_plot = [label.strip() for label in sensors.split(',')]

        # Filter out invalid sensor labels (always safe on Windows, Ubuntu, and RPi 5)
        removed = [label for label in sensors_to_plot if label not in self.sensor_labels]
        for label in removed:
            print(f"Sensor {label} not in CSV data")
        sensors_to_plot = [label for label in sensors_to_plot if label in self.sensor_labels]

        if not sensors_to_plot:
            print("No valid sensors to plot.")
            return [] if return_figs else None

        figs = []

        if combined:
            # ── NEW LAYOUT: 5-sensor (or subset) multi-row figure ─────────────────
            # Always ordered A → B → C → D → E top-to-bottom among the requested sensors.
            # Each row = one sensor: left = Force (N), right = Position (mm).
            # All subplots share the x-axis (time). Fully cross-platform via matplotlib
            # (identical appearance and behaviour on Windows 10/11, Ubuntu, and RPi 5 touchscreen).

            sensor_order = 'ABCDE'
            ordered_sensors = sorted(sensors_to_plot, key=lambda x: sensor_order.index(x))
            n_rows = len(ordered_sensors)

            # Taller figure for multiple rows; width kept comfortable for any screen
            fig, axs = plt.subplots(n_rows, 2,
                                    sharex=True,
                                    figsize=(14, 3.5 * n_rows),   # scales nicely with row count
                                    squeeze=False)                # always 2D array for easy indexing

            fig.suptitle(f"Calculated Force & Position - Sensors {', '.join(ordered_sensors)}\n"
                         f"Test: {self.test_type}", fontsize=12)

            for i, l in enumerate(ordered_sensors):
                if filter_level == 'valid': s = self.data_dict[f'Sensor_{l}']
                elif filter_level == 'clean': s = self.clean_dict[f'Sensor_{l}']
                if offset_time: 
                    t0 = s['time_offset'] 
                else: 
                    t0 = 0.0
                c = self.colors[i]
                
                if 'force' not in s or 'position' not in s:
                    self.calc_force_position()   # computes for ALL sensors (idempotent)
                

                # Left column: Force
                axs[i, 0].scatter(s['time']-t0, s['force'], c=c, s=1, linewidth=1.4, label=f'{l} Force')
                axs[i, 0].set_ylabel(f'{l} Force (N)')
                # axs[i, 0].legend(loc='upper right')
                axs[i, 0].grid(True, alpha=0.3)

                # Right column: Position (converted to mm for readability)
                axs[i, 1].scatter(s['time']-t0, s['position'] * 1000, c=c, s=1, linewidth=1.4, label=f'{l} Position')
                axs[i, 1].set_ylabel(f'{l} Position (mm)')
                # axs[i, 1].legend(loc='upper right')
                axs[i, 1].grid(True, alpha=0.3)
                axs[i, 1].set_ylim(0, 120)

            all_forces = np.concatenate(
                [self.data_dict[f'Sensor_{l}']['force'] for l in ordered_sensors]
            )
            all_pos_mm = np.concatenate(
                [self.data_dict[f'Sensor_{l}']['position'] * 1000 for l in ordered_sensors]
            )

            if len(all_forces) > 0:
                f_min, f_max = np.min(all_forces), np.max(all_forces)
                pad_f = 0.05 * (f_max - f_min) if (f_max > f_min) else 5.0
                for r in range(n_rows):
                    axs[r, 0].set_ylim(f_min - pad_f, f_max + pad_f)

            if len(all_pos_mm) > 0:
                p_min, p_max = np.min(all_pos_mm), np.max(all_pos_mm)
                pad_p = 0.05 * (p_max - p_min) if (p_max > p_min) else 10.0
                for r in range(n_rows):
                    axs[r, 1].set_ylim(0, p_max + pad_p)

            # ── Link y-limits so zooming/panning on ANY force subplot instantly updates ALL
            #     other force subplots (and same for position column). X-zoom already syncs
            #     via sharex=True.
            #     Reentrancy guard prevents infinite recursion (matplotlib callback gotcha).
            #     Fully cross-platform — identical behaviour on Windows 10/11, Ubuntu,
            #     and Raspberry Pi 5 touchscreen.
            force_axes = [axs[r, 0] for r in range(n_rows)]
            pos_axes   = [axs[r, 1] for r in range(n_rows)]

            updating = False                     # ← reentrancy protection

            def on_ylim_changed(ax):
                nonlocal updating
                if updating:                     # ← skip if already updating
                    return
                updating = True
                try:
                    ylim = ax.get_ylim()
                    target = force_axes if ax in force_axes else pos_axes
                    for other in target:
                        if other is not ax:
                            other.set_ylim(ylim)
                finally:
                    updating = False

            for ax in force_axes + pos_axes:
                ax.callbacks.connect('ylim_changed', on_ylim_changed)

            # Only the bottom row gets the x-label (shared)
            axs[-1, 0].set_xlabel('Time (s)')
            axs[-1, 1].set_xlabel('Time (s)')

            fig.tight_layout(rect=[0, 0, 1, 0.96])
            figs.append(fig)

        else:
            # ── Unchanged: one separate figure per sensor ─────────────────────
            for l in sensors_to_plot:
                s = self.data_dict[f'Sensor_{l}']
                if 'force' not in s or 'position' not in s:
                    self.calc_force_position()

                fig, ax = plt.subplots(1, 2, sharex=True, figsize=(12, 7))
                ax[0].plot(s['time'], s['force'], linewidth=1.4, label=f'{l} Force')
                ax[0].set_xlabel('Time (s)')
                ax[0].set_ylabel('Force (N)')
                ax[0].legend(loc='upper right')

                ax[1].plot(s['time'], s['position'] * 1000, linewidth=1.4, label=f'{l} Position')
                ax[1].set_xlabel('Time (s)')
                ax[1].set_ylabel('Position (mm)')
                ax[1].legend(loc='upper right')

                fig.tight_layout()
                figs.append(fig)

        if return_figs:
            return figs
        else:
            for fig in figs:
                plt.show(block=False)
            return None

    def plot_raw_strains(self, sensors='A,B,C,D,E', combined=True, return_figs=False):
        """
        Plot raw and filtered strain data for the specified sensors.

        Parameters
        ----------
        sensors : str, optional
            Comma-separated list of sensor labels (e.g., 'A,B,C'), by default 'A,B,C,D,E'
        combined : bool, optional
            If True, plot all sensors on the same figure (one subplot per channel).
            If False, create a separate figure for each sensor.
        return_figs : bool, optional
            If True, return a list of figure objects instead of showing them.
        """

        sensors_to_plot = [label.strip() for label in sensors.split(',')]

        # Filter out invalid sensor labels
        removed = [label for label in sensors_to_plot if label not in self.sensor_labels]
        for label in removed:
            print(f"Sensor {label} not in CSV data")
        sensors_to_plot = [label for label in sensors_to_plot if label in self.sensor_labels]

        if not sensors_to_plot:
            print("No valid sensors to plot.")
            return [] if return_figs else None

        colors = ['r', 'g', 'c', 'y', 'm']
        # → red, green, cyan, yellow, magenta
        figs = []

        if combined:
            # ── Single figure with all sensors ────────────────────────────────
            fig, ax = plt.subplots(1, 2, sharex=True, sharey=True, figsize=(14, 8))
            fig.suptitle(f"Raw & Filtered Strain – All Sensors\n"
                        f"Test: {self.test_type}", fontsize=12)

            for l, c in zip(sensors_to_plot, colors):
                s = self.data_dict[f'Sensor_{l}']
                if not hasattr(s, 'ini_1'):
                    self.describe_channels()
                if not hasattr(s, 'strain_1_filter') and self.test_type != 'Calibration':
                    self.filter_channels()
                elif self.test_type == 'Calibration':
                    s['strain_1_filter'] = s['strain_1_raw']
                    s['strain_2_filter'] = s['strain_2_raw']

                # Channel 1 (left subplot)
                ax[0].plot(s['time'], s['strain_1_raw'] - s['ini_1'],
                        linewidth=0.6, alpha=0.8)
                ax[0].plot(s['time'], s['strain_1_filter'] - s['ini_1'],
                        linewidth=1.4, label=f'{l}1', c=c)
                ax[0].set_xlabel('Time (s)')
                ax[0].set_ylabel(r"0'ed ADC Integer Value ($\pm 2^{23}$)")
                ax[0].yaxis.set_major_formatter(StrMethodFormatter('{x:,}'))
                ax[0].legend(loc='upper right')

                # Channel 2 (right subplot)
                ax[1].plot(s['time'], s['strain_2_raw'] - s['ini_2'],
                        linewidth=0.6, alpha=0.8)
                ax[1].plot(s['time'], s['strain_2_filter'] - s['ini_2'],
                        linewidth=1.4, label=f'{l}2', c=c)
                ax[1].set_xlabel('Time (s)')
                ax[1].yaxis.set_major_formatter(StrMethodFormatter('{x:,}'))
                ax[1].legend(loc='upper right')

            # Optional extra info in title (if available)
            if self.test_type == 'Force Cycle':
                extra = (f"Cycle Load: {self.cycle_force}{self.cycle_force_units}, "
                        f"Pre-rest: {self.test_rest}")
                fig.suptitle(fig._suptitle.get_text() + f"\n{extra}", fontsize=11)

            fig.tight_layout(rect=[0, 0, 1, 0.96])
            figs.append(fig)

        else:
            # ── One figure per sensor (original behavior) ─────────────────────
            for l in sensors_to_plot:
                s = self.data_dict[f'Sensor_{l}']
                if not hasattr(s, 'ini_1'):
                    self.describe_channels()
                if not hasattr(s, 'strain_1_filter'):
                    self.filter_channels()

                fig, ax = plt.subplots(1, 2, sharex=True, figsize=(12, 7))

                # Channel 1
                ax[0].plot(s['time'], s['strain_1_raw'] - s['ini_1'],
                        c='C0', linewidth=0.5, label=f'{l}1_raw')
                ax[0].plot(s['time'], s['strain_1_filter'] - s['ini_1'],
                        c='C1', linewidth=1.0, label=f'{l}1_filter')
                ax[0].axhline(0, c='red', linewidth=0.4, label='Initial')
                ax[0].axhline(s['end_1'] - s['ini_1'], c='green', linewidth=0.4, label='End')
                ax[0].set_xlabel('Time (s)')
                ax[0].set_ylabel(r"0'ed ADC Integer Value ($\pm 2^{23}$)")
                ax[0].yaxis.set_major_formatter(StrMethodFormatter('{x:,}'))
                ax[0].legend(loc='upper right')

                # Channel 2
                ax[1].plot(s['time'], s['strain_2_raw'] - s['ini_2'],
                        c='C0', linewidth=0.5, label=f'{l}2_raw')
                ax[1].plot(s['time'], s['strain_2_filter'] - s['ini_2'],
                        c='C1', linewidth=1.0, label=f'{l}2_filter')
                ax[1].axhline(0, c='red', linewidth=0.4)
                ax[1].axhline(s['end_2'] - s['ini_2'], c='green', linewidth=0.4)
                ax[1].set_xlabel('Time (s)')
                ax[1].yaxis.set_major_formatter(StrMethodFormatter('{x:,}'))
                ax[1].legend(loc='upper right')

                if self.test_type == 'Force Cycle':
                    fig.suptitle(f'Sensor S/N: {s["SN"]}, '
                                f'Cycle Load: {self.cycle_force}{self.cycle_force_units}, '
                                f'Test # {self.test_number}  |  Pre-rest: {self.test_rest}')

                fig.tight_layout()
                figs.append(fig)

        if return_figs:
            return figs
        else:
            for fig in figs:
                plt.show()
            return None

    def save_stiffnesses(self, directory=None, note=None):
        if directory is None:
            directory = Config.RESULTS_BASE

        folder = directory / self.date 
        self.results_path = folder / f'{self.time}_stiffnesses.csv'
        os.makedirs(folder, exist_ok=True)

        header = ['Stalk', 'Stiffness (N/m^2)', 'Estimate 1', 'Estimate 2', 'Estimate 3', 'Estimate 4']
        
        with open(self.results_path, 'w', newline='') as f:
            writer = csv.writer(f)
            if note:
                writer.writerow(['Note: ' + note])
            writer.writerow(['Estimation Method: ' + self.method])
            writer.writerow(['Test Type: ' + self.test_type])
            writer.writerow([Config.STIFFNESSES_MARKER])
            writer.writerow(header)

            for i in range(self.num_stalks):
                row = [i+1, self.stiffnesses[i], self.estimates[i][0], self.estimates[i][1], self.estimates[i][2], self.estimates[i][3]]
                writer.writerow(row)

        print(f'Wrote stiffness results to {self.results_path}')


def run_stiffness_pipeline(data: HiSTIFFSData, results_note: str='None') -> None:
    data.detect_stalks(plot=False)
    # data.plot_detections(filter_level='clean')
    data.estimate_all_stalks_stiffness()
    data.save_stiffnesses(note=results_note)

if __name__ == "__main__":
    times = ['203337', '203450', '203555', '203701', '203807', '203911', '204131', '204238', '204347', '204451', '204558', '204634', '204711', '204747', '204826', '204904', '204941', '205021', '205101', '205140']
    data = HiSTIFFSData(date="2026-05-06", time='194317', debug=True)
    if data.exists:
        # data.plot_raw_strains(combined=False)
        # data.describe_channels()
        # data.shift_initials()
        # data.calc_force_position(clip=False)
        # data.plot_force_position(combined=True)
        # plt.show()

        # data.interactive_detect_stalks()
        run_stiffness_pipeline(data, results_note='dummy trials')

        plt.show()
        # keyboard.wait('space')
