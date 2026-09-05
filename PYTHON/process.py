# This file reads data from a CSV and processes it with multiple methods and options availible to the user
# The CSV must have columns titled like 'Time_A_sec, Strain_A1_raw, Strain_A2_raw...' and so on for each lettered sensor
# Or columns like 'Time_A_sec, Strain_A1_V, Strain_A2_V...'. This is raw ADC values.
# The __init__() will automatically detect which type is present, or if both are, defaults to raw option (more efficient/precise).
# Calibration is done with raw values. mV option is for human comprehension and only viable for displays, not in calculations.

# Standard libraries
import os
import csv
from pathlib import Path
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
from scipy.signal import savgol_filter, butter, filtfilt, medfilt
from scipy.ndimage import gaussian_filter1d
# from statsmodels.nonparametric.smoothers_lowess import lowess
from scipy.interpolate import PchipInterpolator
# from hampel import hampel

# Workspace scripts
from config import Config
from stalk_detector import (
    interactive_detect_stalks,
    display_stalk_selections,
    refine_stalk_selections,
    load_stalk_rows,
    refine_time_window,
    inclusive_time_mask,
    STALK_SENSORS,
)

class HiSTIFFSData:
    # === load raw data and calculate force & position ===
    def __init__(self, date, time, nano_label='01', debug=False, file_path=None, t_lims=[None,None]):
        # Fully cross-platform: uses pathlib.Path everywhere (Win / Ubuntu / RPi5 identical)
        self.nano_label = nano_label
        self.exists = False
        self.has_force_pos = False
        self.colors = ['r', 'g', 'c', 'y', 'm']
        self.sensor_starts = np.array([0.0, 142.47, 345.33, 508.12, 670.59]) * 1e-3
        # self.sensor_starts_dy = np.array([0.0, 141.36, 335.09, 496.19, 650.19]) * 1e-3  # v3.1
        self.sensor_starts_dy = np.array([-0.786, -0.100, 0.577, 0.0, 0.0])
        self.min_pos = 0.055
        self.fp_thresh = 0.30
        self.yaw = np.radians(20)
        self.rel_deflection = 0.060 # m, parallel distance between lead and middle sensor
        self.height = 0.902  # m, height of contact point between probe and stalk
        self.stalk_spacing = 0.1524 # m, physical spacing between stalks

        self.t_min, self.t_max = t_lims     # time bounds of selected portion

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
        self.apply_time_bounds()

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
            self.filter_window = int(round(self.data_rate/40.0 + 0.01, 0))
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
            if l in ['A','B','C','D','E','F']:
                self.data_dict[f'Sensor_{l}']['length'] = 0.155
                self.data_dict[f'Sensor_{l}']['type'] = 'straight'
            if l in ['B','E']:
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

        # Chronological sort by each sensor's Time_*_sec.
        # Downstream filters, gradients, and time-bounds assume monotonic time.
        # mergesort keeps original CSV row order on identical timestamps.
        # Only arrays that share the time axis are permuted; scalars (length,
        # type, start_pos, cal coeffs) stay put.
        shared_order = None
        for l in self.sensor_labels:
            s = self.data_dict[f'Sensor_{l}']
            if 'time' not in s:
                continue
            t = s['time']
            order = np.argsort(t, kind='mergesort')
            if shared_order is None:
                shared_order = order
            if np.array_equal(order, np.arange(t.shape[0])):
                continue
            for field, val in list(s.items()):
                if isinstance(val, np.ndarray) and val.shape[0] == t.shape[0]:
                    s[field] = val[order]

        # Processed Time is a CSV-row clock, not per-sensor. Reorder it with
        # the first sensor's permutation so it stays aligned with the table.
        proc = self.data_dict.get('Processed Time')
        if (
            shared_order is not None
            and isinstance(proc, np.ndarray)
            and proc.shape[0] == shared_order.shape[0]
            and not np.array_equal(shared_order, np.arange(shared_order.shape[0]))
        ):
            self.data_dict['Processed Time'] = proc[shared_order]

    def apply_time_bounds(self):
        """Applies bounds defined by self.t_min and self.t_max to sensor data dictionaries.
        Discards all data points outside of bounds.
        """
        t_min, t_max = self.t_min, self.t_max
        if t_min is None and t_max is None:
            return

        shared_mask = None
        for l in self.sensor_labels:
            key = f'Sensor_{l}'
            if key not in self.data_dict:
                continue
            s = self.data_dict[key]
            if 'time' not in s:
                continue

            t = s['time']
            mask = np.ones(t.shape, dtype=bool)
            if t_min is not None:
                if l == 'A':
                    mask &= (t >= t_min)
                elif l == 'B':
                    mask &= (t >= t_min+1.4)
                elif l == 'C':
                    mask &= (t >= t_min+2.8)
            if t_max is not None:
                if l == 'A':
                    mask &= (t <= t_max-2.8)
                elif l == 'B':
                    mask &= (t <= t_max-1.4)
                elif l == 'C':
                    mask &= (t <= t_max)

            n_keep = int(np.count_nonzero(mask))
            if n_keep == 0:
                print(f"Warning: apply_time_bounds() removed all points for Sensor {l} "
                      f"(t_min={t_min}, t_max={t_max})")

            for field, val in list(s.items()):
                if isinstance(val, np.ndarray) and val.shape[0] == t.shape[0]:
                    s[field] = val[mask]

            if shared_mask is None:
                shared_mask = mask
            else:
                # row-aligned table: keep intersection so Processed Time stays consistent
                if shared_mask.shape == mask.shape:
                    shared_mask &= mask

        proc = self.data_dict.get('Processed Time')
        if shared_mask is not None and isinstance(proc, np.ndarray) and proc.shape[0] == shared_mask.shape[0]:
            self.data_dict['Processed Time'] = proc[shared_mask]

        print(f"Applied time bounds [{t_min}, {t_max}]")

    def describe_channels(self, avg_duration=0.25):
        for l in self.sensor_labels:
            s = self.data_dict[f'Sensor_{l}']
            t_min = np.min(s['time'])
            t_max = np.max(s['time'])

            avg_initial_value_1_raw = np.median(s['strain_1_raw'][s['time'] <= t_min + avg_duration])
            avg_initial_value_2_raw = np.median(s['strain_2_raw'][s['time'] <= t_min + avg_duration])
            avg_end_value_1_raw =     np.median(s['strain_1_raw'][s['time'] >= t_max - avg_duration])
            avg_end_value_2_raw =     np.median(s['strain_2_raw'][s['time'] >= t_max - avg_duration])
            self.data_dict[f'Sensor_{l}']['ini_1'] = avg_initial_value_1_raw
            self.data_dict[f'Sensor_{l}']['ini_2'] = avg_initial_value_2_raw
            self.data_dict[f'Sensor_{l}']['end_1'] = avg_end_value_1_raw
            self.data_dict[f'Sensor_{l}']['end_2'] = avg_end_value_2_raw

            print(f'[describe_channels()] Zero-ed Sensor_{l} to average within t=[{t_min:.3f},{t_min + avg_duration:.3f}]s, [{avg_initial_value_1_raw:.0f},{avg_initial_value_2_raw:.0f}]')

    def filter_channels(self, window=None, order=1):
        if window is None:
            window = self.filter_window
        
        for l in self.sensor_labels:
            print(f'[filter_channels()] Filtering Sensor_{l} raw data with window size {window}...')
            s = self.data_dict[f'Sensor_{l}']
            self.data_dict[f'Sensor_{l}']['strain_1_filter'] = savgol_filter(s['strain_1_raw'], window, order)
            self.data_dict[f'Sensor_{l}']['strain_2_filter'] = savgol_filter(s['strain_2_raw'], window, order)

    def calc_force_position(self, filter_out=False, clip=False):
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
                s['force'] = np.clip(np.where(np.abs(den) > 1e-12, num/den, 0.0), -1000, 1000) # santity check in Newtons

                num = k2*d2*(s['strain_1_filter'] - c1) - k1*d1*(s['strain_2_filter'] - c2)
                den = k2*(s['strain_1_filter'] - c1) - k1*(s['strain_2_filter'] - c2)                
                # prevent divide by zero error
                for i, val in enumerate(den):
                    if np.abs(val) >= 1e-12:
                        den[i] = val
                    else:
                        den[i] = 1.0
                s['position'] = np.clip(np.where(np.abs(den) > 10**8.5, num/den, 0.0), -0.300, 0.300) # sanity check in meters  

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

    # === gather refined stalk bounds ===
    def gather_stalk_traces(self):
        """Load Plot/Stalk ids and required refine time bounds from CSV.

        Does not slice force/position here. Bounds live on
        self.refine_t0 / self.refine_t1 with shape (n_stalks, 3) in
        STALK_SENSORS order (A, B, C). NaN = that sensor was skipped.
        Original Start/End are ignored — refine cells are required.

        The actual sample cut happens later in read_stalk_on_sensor(),
        which applies inclusive_time_mask to self.data_dict.
        """
        path = self.stalks_csv_path
        records = load_stalk_rows(path)
        n = len(records)
        n_sens = len(STALK_SENSORS)

        self.num_stalks = n
        self.stalks_plot = np.empty(n, dtype=np.int32)
        self.stalks_id = np.empty(n, dtype=np.int32)
        self.refine_t0 = np.full((n, n_sens), np.nan, dtype=np.float64)
        self.refine_t1 = np.full((n, n_sens), np.nan, dtype=np.float64)

        if n == 0:
            print(f"Status: no stalk rows to gather from {path}")
            return

        print(f"Gathering refine bounds for {n} stalk row(s) from {path} "
              f"(sensors {','.join(STALK_SENSORS)}; refine required)")

        for i, rec in enumerate(records):
            self.stalks_plot[i] = rec['Plot']
            self.stalks_id[i] = rec['Stalk']
            for j, l in enumerate(STALK_SENSORS):
                window = refine_time_window(rec, l)
                if window is None:
                    continue
                self.refine_t0[i, j] = window[0]
                self.refine_t1[i, j] = window[1]

    # === estimate stalk stiffness ===
    def estimate_all_stalks_stiffness(self, method='quasi-static EI AB/CB'):
        """Estimate flexural rigidity EI (N·m^2) from refined A/B/C windows.

        Geometry (intent)
        -----------------
        A and C contact the stalk at essentially the same deflection
        (0 cm extra, relative to B). B contacts after the known extra
        probe offset self.rel_deflection (6 cm). Median force in each
        refined window is the load at that deflection.

        Cantilever relation at contact height H = self.height:
            δ = F H^3 / (3 EI)  →  EI = ΔF H^3 / (3 δ)
        with ΔF_AB = F_B − F_A, ΔF_CB = F_B − F_C, δ = self.rel_deflection.

        Three values are stored per stalk:
            Estimate AB, Estimate CB, Estimate AB-CB
        AB-CB is the mean of whichever of AB/CB are finite. A skipped or
        empty window makes that pair NaN; if only one pair exists, AB-CB
        is that pair (not NaN).
        """
        self.method = method
        if not hasattr(self, 'refine_t0'):
            self.gather_stalk_traces()
        if not self.has_force_pos:
            self.calc_force_position()

        print(f'\nEstimating EI of {self.num_stalks} stalks with "{method}"...')

        # H^3 / 3 converts a force increment over known δ into EI.
        # δ is the designed A/C-vs-B offset, not a per-sample position.
        height_m = float(self.height)
        delta_m = float(self.rel_deflection)
        beam_scale = (height_m ** 3) / (3.0 * delta_m)

        n = int(self.num_stalks)
        self.estimate_ab = np.full(n, np.nan, dtype=np.float64)
        self.estimate_cb = np.full(n, np.nan, dtype=np.float64)
        self.estimate_abcb = np.full(n, np.nan, dtype=np.float64)
        self.stiffnesses = np.full(n, np.nan, dtype=np.float64)
        # estimates[:, 0:3] == AB, CB, AB-CB — kept for save_stiffnesses
        self.estimates = np.full((n, 3), np.nan, dtype=np.float64)

        if n == 0:
            print('No stalk rows gathered — nothing to estimate')
            return

        for i in range(n):
            plot_n = int(self.stalks_plot[i])
            stalk_n = int(self.stalks_id[i])
            print(f'Processing [plot={plot_n}, stalk={stalk_n}]')

            # Bounds applied to self.data_dict inside read_stalk_on_sensor.
            f_a = self.read_stalk_on_sensor(i, 'A')
            f_b = self.read_stalk_on_sensor(i, 'B')
            f_c = self.read_stalk_on_sensor(i, 'C')

            ei_ab = (f_b - f_a) * beam_scale
            ei_cb = (f_b - f_c) * beam_scale
            # One finite pair → AB-CB is that pair. Both missing → NaN.
            # Avoid np.nanmean on [nan, nan] (RuntimeWarning).
            pair = [v for v in (ei_ab, ei_cb) if np.isfinite(v)]
            ei_avg = float(np.mean(pair)) if pair else np.nan

            self.estimate_ab[i] = ei_ab
            self.estimate_cb[i] = ei_cb
            self.estimate_abcb[i] = ei_avg
            self.stiffnesses[i] = ei_avg
            self.estimates[i, :] = (ei_ab, ei_cb, ei_avg)
            print(f'    F(A,B,C)=({f_a:.3f}, {f_b:.3f}, {f_c:.3f}) N  '
                  f'EI AB={ei_ab:.4g}  CB={ei_cb:.4g}  AB-CB={ei_avg:.4g} N·m^2')

    def read_stalk_on_sensor(self, stalk_index, sensor_label):
        """Median force in this stalk's refined time window on one sensor.

        Window comes from self.refine_t0/t1 (gather_stalk_traces). The
        cut is inclusive_time_mask on data_dict time only — no position
        or force filter. Blank refine bounds or an empty slice → NaN.
        """
        try:
            j = STALK_SENSORS.index(sensor_label)
        except ValueError:
            return np.nan

        t0 = self.refine_t0[stalk_index, j]
        t1 = self.refine_t1[stalk_index, j]
        key = f'Sensor_{sensor_label}'
        if key not in self.data_dict or 'force' not in self.data_dict[key]:
            return np.nan

        s = self.data_dict[key]
        mask = inclusive_time_mask(s['time'], t0, t1)
        samples = np.asarray(s['force'], dtype=np.float64)[mask]
        if samples.size == 0 or not np.any(np.isfinite(samples)):
            return np.nan
        return float(np.nanmedian(samples))

    # === display data and output results ===
    def plot_force_position(self, sensors='A,B,C,D,E', combined=True, return_figs=False, filter_level='valid', offset_time: bool=False):
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

            sensor_order = 'ABCDEF'
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
                # 'clean' path removed with _cleanup_sensor_traces.
                s = self.data_dict[f'Sensor_{l}']

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
                if 'ini_1' not in s or 'ini_2' not in s:
                    self.describe_channels()
                if ('strain_1_filter' not in s or 'strain_2_filter' not in s) and self.test_type != 'Calibration':
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

            fig.tight_layout(rect=[0, 0, 1, 0.96])
            figs.append(fig)

        else:
            # ── One figure per sensor (original behavior) ─────────────────────
            for l in sensors_to_plot:
                s = self.data_dict[f'Sensor_{l}']
                if 'ini_1' not in s or 'ini_2' not in s:
                    self.describe_channels()
                if ('strain_1_filter' not in s or 'strain_2_filter' not in s) and self.test_type != 'Calibration':
                    self.filter_channels()
                elif self.test_type == 'Calibration':
                    s['strain_1_filter'] = s['strain_1_raw']
                    s['strain_2_filter'] = s['strain_2_raw']

                fig, ax = plt.subplots(1, 2, sharex=True, figsize=(12, 7))
                t = s['time']
                s1 = s['strain_1_raw'] - s['ini_1']
                s1_filt = s['strain_1_filter'] - s['ini_1']
                s2 = s['strain_2_raw'] - s['ini_2']
                s2_filt = s['strain_2_filter'] - s['ini_2']

                # Channel 1
                ax[0].plot(t, s1, c='C0', linewidth=0.5, label=f'{l}1_raw')
                ax[0].plot(t, s1_filt, c='C1', linewidth=1.0, label=f'{l}1_filter')
                ax[0].axhline(0, c='red', linewidth=0.4, label='Initial')
                ax[0].axhline(s['end_1'] - s['ini_1'], c='green', linewidth=0.4, label='End')
                ax[0].set_xlabel('Time (s)')
                ax[0].set_ylabel(r"0'ed ADC Integer Value ($\pm 2^{23}$)")
                ax[0].yaxis.set_major_formatter(StrMethodFormatter('{x:,}'))
                ax[0].legend(loc='upper right')

                # Channel 2
                ax[1].plot(t, s2, c='C0', linewidth=0.5, label=f'{l}2_raw')
                ax[1].plot(t, s2_filt, c='C1', linewidth=1.0, label=f'{l}2_filter')
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

    def save_stiffnesses(self, directory=None, note=None):
        """Write Plot/Stalk plus the three EI estimates (N·m^2).

        Empty / skipped pairs are the literal token NaN so the file is
        readable without a numeric parser guessing 0. Column order:
        Estimate AB, Estimate CB, Estimate AB-CB.
        """
        if directory is None:
            directory = Config.RESULTS_BASE

        folder = directory / self.date
        self.results_path = folder / f'{self.time}_stiffnesses.csv'
        os.makedirs(folder, exist_ok=True)

        def _fmt_ei(value):
            # One write path for missing windows and true non-finite EI.
            if value is None:
                return 'NaN'
            try:
                number = float(value)
            except (TypeError, ValueError):
                return 'NaN'
            if not np.isfinite(number):
                return 'NaN'
            return f'{number:.6g}'

        header = ['Plot', 'Stalk',
                  'Estimate AB (N*m^2)',
                  'Estimate CB (N*m^2)',
                  'Estimate AB-CB (N*m^2)']

        with open(self.results_path, 'w', newline='') as f:
            writer = csv.writer(f)
            if note:
                writer.writerow(['Note: ' + note])
            writer.writerow(['Estimation Method: ' + getattr(self, 'method', '')])
            writer.writerow(['Test Type: ' + self.test_type])
            writer.writerow([
                f'height_m={self.height}',
                f'rel_deflection_m={self.rel_deflection}',
                'EI = (F_B - F_low) * height^3 / (3 * rel_deflection)',
            ])
            writer.writerow([Config.STIFFNESSES_MARKER])
            writer.writerow(header)

            for i in range(self.num_stalks):
                plot_n = self.stalks_plot[i] if hasattr(self, 'stalks_plot') else ''
                stalk_n = self.stalks_id[i] if hasattr(self, 'stalks_id') else i + 1
                ei_ab = self.estimate_ab[i] if hasattr(self, 'estimate_ab') else self.estimates[i][0]
                ei_cb = self.estimate_cb[i] if hasattr(self, 'estimate_cb') else self.estimates[i][1]
                ei_avg = self.estimate_abcb[i] if hasattr(self, 'estimate_abcb') else self.estimates[i][2]
                writer.writerow([
                    plot_n, stalk_n,
                    _fmt_ei(ei_ab), _fmt_ei(ei_cb), _fmt_ei(ei_avg),
                ])

        print(f'Wrote stiffness results to {self.results_path}')

    # +++ additional functions, reorganize later... +++

    def moving_baseline(self, show_plots):
        # function globals
        SAMPLE_RATE = self.data_rate  # Hz
        SENSOR_SPEED_FPM = 25 # feet per minute
        STALK_CONTACT_PERIOD = 0.220 # seconds, based on 25 fpm
        PROBE_LENGTH_AtoC = 1.363 # meters, tip of A to tip of C

        print('*'*80)
        print('moving_baseline()')
        print(f'{SAMPLE_RATE=}, {SENSOR_SPEED_FPM=}, {STALK_CONTACT_PERIOD=}')
        
        for label in self.sensor_labels:

            # import data
            # if label not in 'A':
            #     continue
            sensor = f"Sensor_{label}"
            t = self.data_dict[sensor]['time']
            s1 = self.data_dict[sensor]['strain_1_raw']
            s2 = self.data_dict[sensor]['strain_2_raw']
            ds1 = np.gradient(s1, t)
            ds2 = np.gradient(s2, t)

            # smooth data 
            smooth1 = savgol_filter(s1, window_length=17, polyorder=1)
            smooth2 = savgol_filter(s2, window_length=17, polyorder=1)
            
            # calculate remove estimated event windows (currently detects event beginnings, not endings)
            thresh_fact = 2
            (
                dsmooth1,
                derivative_threshold1,
                event_beginnings1,
                _,
                events_only_mask1,
                events_removed_t1,
                events_removed_y1,
            ) = remove_event_windows(t, smooth1, SAMPLE_RATE, STALK_CONTACT_PERIOD, thresh_fact=thresh_fact)

            (
                dsmooth2,
                derivative_threshold2,
                event_beginnings2,
                _,
                events_only_mask2,
                events_removed_t2,
                events_removed_y2,
            ) = remove_event_windows(t, smooth2, SAMPLE_RATE, STALK_CONTACT_PERIOD, thresh_fact=thresh_fact)

            print(f'{len(event_beginnings1)} events detected for Sensor {label} channel 1')
            print(f'{len(event_beginnings2)} events detected for Sensor {label} channel 2')

            ####################################################################################################
            if show_plots['Detect and Remove Events']:
                _, ax = plt.subplots(2, 2, sharex=True, figsize=(12, 7), constrained_layout=True)

                ax[0, 0].plot(t, s1, label='Raw Strain 1', alpha=0.5, linewidth=0.4)
                ax[0, 0].plot(t, smooth1, label='Smooth 1', linewidth=0.4)
                ax[0, 0].plot(events_removed_t1, events_removed_y1, label='Events Removed', linewidth=2)
                ax[0, 0].set_title('Events Removed: Sensor ' + label + ' Channel 1')
                ax[0, 0].set_ylabel("Strain Reading (ADC count)")

                ax[0, 1].plot(t, dsmooth1, label='1st Derivative 1', linewidth=0.4)
                ax[0, 1].axhline(derivative_threshold1, color='red', linestyle='--', label='Threshold')
                ax[0, 1].scatter(t[event_beginnings1], dsmooth1[event_beginnings1], color='orange', label='Events Detected', s=10)
                ax[0, 1].set_title('Events Detected: Sensor ' + label + ' Channel 1')
                ax[0, 1].set_ylabel("Strain Reading (ADC count/s)")

                ax[1, 0].plot(t, s2, label='Raw Strain 2', alpha=0.5, linewidth=0.4)
                ax[1, 0].plot(t, smooth2, label='Smooth 2', linewidth=0.4)
                ax[1, 0].plot(events_removed_t2, events_removed_y2, label='Events Removed', linewidth=2)
                ax[1, 0].set_title('Events Removed: Sensor ' + label + ' Channel 2')
                ax[1, 0].set_ylabel("Strain Reading (ADC count)")

                ax[1, 1].plot(t, dsmooth2, label='1st Derivative 2', linewidth=0.4)
                ax[1, 1].axhline(derivative_threshold2, color='red', linestyle='--', label='Threshold')
                ax[1, 1].scatter(t[event_beginnings2], dsmooth2[event_beginnings2], color='orange', label='Events Detected', s=10)
                ax[1, 1].set_title('Events Detected: Sensor ' + label + ' Channel 2')
                ax[1, 1].set_ylabel("Strain Reading (ADC count/s)")

                for a in ax.flat:
                    a.legend(loc='upper right')
                    a.set_xlabel("Time (s)")


            # Eliminate outliers
            window_sec = STALK_CONTACT_PERIOD * 10  # seconds of data, default is 5x event window
            kernel_size = int(window_sec * SAMPLE_RATE)  # convert to samples
            if kernel_size % 2 == 0:
                kernel_size += 1  # ensure kernel size is odd
            # hampel_baseline1 = _hampel_filter(events_removed_y1, window_size=kernel_size)
            # hampel_baseline2 = _hampel_filter(events_removed_y2, window_size=kernel_size)
            medfilt_baseline1 = medfilt(events_removed_y1, kernel_size=kernel_size)
            medfilt_baseline2 = medfilt(events_removed_y2, kernel_size=kernel_size)

            # Interpolate across the removed event windows
            interp1 = PchipInterpolator(events_removed_t1, medfilt_baseline1)
            interp2 = PchipInterpolator(events_removed_t2, medfilt_baseline2)
            # interp1h = PchipInterpolator(events_removed_t1, hampel_baseline1)
            # interp2h = PchipInterpolator(events_removed_t2, hampel_baseline2)

            interp_baseline1 = interp1(t)
            interp_baseline2 = interp2(t)
            # interp_baseline1h = interp1h(t)
            # interp_baseline2h = interp2h(t)

            # Apply Gaussian filter
            sigma = kernel_size / 6  # standard deviation for Gaussian filter, default is 1/6 of kernel size (# ~99.7% of Gaussian spans kernel)

            gaussian_baseline1 = gaussian_filter1d(interp_baseline1, sigma=sigma)
            gaussian_baseline2 = gaussian_filter1d(interp_baseline2, sigma=sigma)
            # gaussian_baseline1h = gaussian_filter1d(interp_baseline1h, sigma=sigma)
            # gaussian_baseline2h = gaussian_filter1d(interp_baseline2h, sigma=sigma)

            ####################################################################################################
            if show_plots['Remove Outliers, Interpolate, Smooth']:
                _, ax = plt.subplots(2, 2, sharex=True, figsize=(12, 7), constrained_layout=True)
                ax[0, 0].plot(events_removed_t1, events_removed_y1, label='Events Removed', linewidth=0.4)
                ax[0, 0].plot(events_removed_t1, medfilt_baseline1, label='Medfilt', linewidth=0.4)
                # ax[0, 0].plot(events_removed_t1, hampel_baseline1, label='Hampel', linewidth=0.4)
                ax[0, 0].plot(t, interp_baseline1, label='Interpolated', linewidth=0.4)
                ax[0, 0].plot(t, gaussian_baseline1, label='Channel Independent Baseline', linewidth=2)
                # ax[0, 0].plot(t, gaussian_baseline1h, label='Channel Independent Baseline (Hampel)', linewidth=2, linestyle='--')
                ax[0, 0].set_title('Remove Outliers, Interpolate, Smooth: Sensor ' + label + ' Channel 1')

                ax[0, 1].plot(t, gaussian_baseline1, label='Channel Independent Baseline', linewidth=2)
                # ax[0, 1].plot(t, gaussian_baseline1h, label='Channel Independent Baseline (Hampel)', linewidth=2, linestyle='--')
                ax[0, 1].plot(t, smooth1, label='Smooth 1', linewidth=0.4)
                ax[0, 1].set_title('Smooth Strain vs Channel Independent Baseline: Sensor ' + label + ' Channel 1')

                ax[1, 0].plot(events_removed_t2, events_removed_y2, label='Events Removed', linewidth=0.4)
                ax[1, 0].plot(events_removed_t2, medfilt_baseline2, label='Medfilt', linewidth=0.4)
                # ax[1, 0].plot(events_removed_t2, hampel_baseline2, label='Hampel', linewidth=0.4)
                ax[1, 0].plot(t, interp_baseline2, label='Interpolated', linewidth=0.4)
                ax[1, 0].plot(t, gaussian_baseline2, label='Channel Independent Baseline', linewidth=2)
                # ax[1, 0].plot(t, gaussian_baseline2h, label='Channel Independent Baseline (Hampel)', linewidth=2, linestyle='--')
                ax[1, 0].set_title('Remove Outliers, Interpolate, Smooth: Sensor ' + label + ' Channel 2')

                ax[1, 1].plot(t, gaussian_baseline2, label='Channel Independent Baseline', linewidth=2)
                # ax[1, 1].plot(t, gaussian_baseline2h, label='Channel Independent Baseline (Hampel)', linewidth=2, linestyle='--')
                ax[1, 1].plot(t, smooth2, label='Smooth 2', linewidth=0.4)
                ax[1, 1].set_title('Smooth Strain vs Channel Independent Baseline: Sensor ' + label + ' Channel 2')

                for a in ax.flat:
                    a.set_xlabel("Time (s)")
                    a.set_ylabel("Strain Reading (ADC count)")
                    a.legend(loc='upper right')

            # translate smooth1 by gausian median to align with gaussian baseline for comparison
            smooth1_adjusted = smooth1 - gaussian_baseline1
            smooth2_adjusted = smooth2 - gaussian_baseline2

            smooth1_translated = smooth1 - np.median(gaussian_baseline1)
            smooth2_translated = smooth2 - np.median(gaussian_baseline2)

            ####################################################################################################
            if show_plots['Compare Smooth vs Channel Independent Baseline Subtracted']:
                _, ax = plt.subplots(1, 2, sharex=True, figsize=(12, 7), constrained_layout=True)
                ax[0].plot(t, smooth1_adjusted, label='Smooth 1 - Channel Independent Baseline', linewidth=0.4)
                ax[0].plot(t, smooth1_translated, label='Smooth 1', linewidth=0.4, alpha=0.5)
                ax[0].set_title('Smooth vs Channel Independent Baseline Subtracted: Sensor ' + label + ' Channel 1')

                ax[1].plot(t, smooth2_adjusted, label='Smooth 2 - Channel Independent Baseline', linewidth=0.4)
                ax[1].plot(t, smooth2_translated, label='Smooth 2', linewidth=0.4, alpha=0.5)
                ax[1].set_title('Smooth vs Channel Independent Baseline Subtracted: Sensor ' + label + ' Channel 2')

                for a in ax.flat:
                    a.set_xlabel("Time (s)")
                    a.set_ylabel("Strain Reading (ADC count)")
                    a.legend(loc='upper right')

            # translate gaussian_baseline1 by its median to align with gaussian_baseline2 for comparison
            gaussian_baseline1_translated = gaussian_baseline1 - np.median(gaussian_baseline1)
            gaussian_baseline2_translated = gaussian_baseline2 - np.median(gaussian_baseline2)

            ####################################################################################################
            if show_plots['Compare Channel Independent Baselines 1 & 2']:
                _, ax = plt.subplots(1, 2, sharex=True, figsize=(12, 7), constrained_layout=True)
                ax[0].plot(t, gaussian_baseline1, label='Channel Independent Baseline 1', linewidth=0.4)
                ax[0].plot(t, gaussian_baseline2, label='Channel Independent Baseline 2', linewidth=0.4)
                ax[0].text(0.5, 0.5, f'Median difference: {np.abs(np.median(gaussian_baseline1) - np.median(gaussian_baseline2)):.0f} ADC count', transform=ax[0].transAxes, fontsize=10, ha='center')
                ax[0].set_title('Compare Channel Independent Baselines: Sensor ' + label + ' Channels 1 & 2')

                ax[1].plot(t, gaussian_baseline1_translated, label='Channel Independent Baseline 1 Translated', linewidth=0.4)
                ax[1].plot(t, gaussian_baseline2_translated, label='Channel Independent Baseline 2 Translated', linewidth=0.4)
                ax[1].set_title('Compare Channel Independent Baselines (Translated): Sensor ' + label + ' Channels 1 & 2')

                for a in ax.flat:
                    a.set_xlabel("Time (s)")
                    a.set_ylabel("Strain Reading (ADC count)")
                    a.legend(loc='upper right')

            ####################################################################################################
            if show_plots['Scatter Plot: derivative vs magnitude'][0]:
                if show_plots['Scatter Plot: derivative vs magnitude'][1]:
                    _, ax = plt.subplots(2, 2, figsize=(12, 7), sharex=True, constrained_layout=True)
                else:
                    _, ax = plt.subplots(1, 2, figsize=(12, 7), sharex=True, constrained_layout=True, squeeze=False)

                ax[0, 0].scatter(dsmooth1, smooth1, s=2, alpha=0.3)
                ax[0, 0].set_title(f'Derivative vs Magnitude: Sensor {label} Channel 1 (smoothed)')

                ax[0, 1].scatter(dsmooth2, smooth2, s=2, alpha=0.3)
                ax[0, 1].set_title(f'Derivative vs Magnitude: Sensor {label} Channel 2 (smoothed)')

                if show_plots['Scatter Plot: derivative vs magnitude'][1]:
                    ax[1, 0].scatter(ds1, s1, s=2, alpha=0.3)
                    ax[1, 0].set_title(f'Derivative vs Magnitude: Sensor {label} Channel 1 (Raw)')

                    ax[1, 1].scatter(ds2, s2, s=2, alpha=0.3)
                    ax[1, 1].set_title(f'Derivative vs Magnitude: Sensor {label} Channel 2 (Raw)')

                for a in ax.flat:
                    a.set_xlabel('Derivative (ADC count/s)')
                    a.set_ylabel('Magnitude (ADC count)')
                    # a.set_xscale('log')
                    # a.set_yscale('log')

            ####################################################################################################
            if show_plots['Histogram: magnitude'][0]:
                if show_plots['Histogram: magnitude'][1]:
                    _, ax = plt.subplots(2, 2, figsize=(12, 7), sharex=True, constrained_layout=True)
                else:
                    _, ax = plt.subplots(1, 2, figsize=(12, 7), sharex=True, constrained_layout=True, squeeze=False)

                ax[0,0].hist(smooth1, bins=100, alpha=0.7)
                ax[0,0].set_title(f'Histogram: Sensor {label} Channel 1 (smoothed)')

                ax[0,1].hist(smooth2, bins=100, alpha=0.7)
                ax[0,1].set_title(f'Histogram: Sensor {label} Channel 2 (smoothed)')
                
                if show_plots['Histogram: magnitude'][1]:
                    ax[1,0].hist(s1, bins=100, alpha=0.7)
                    ax[1,0].set_title(f'Histogram: Sensor {label} Channel 1 (Raw)')

                    ax[1,1].hist(s2, bins=100, alpha=0.7)
                    ax[1,1].set_title(f'Histogram: Sensor {label} Channel 2 (Raw)')
                
                for a in ax.flat:
                    a.set_xlabel('Magnitude (ADC count)')
                    a.set_ylabel('Frequency (count)')
                    # a.set_yscale('log')

            ####################################################################################################
            if show_plots['Histogram: derivative'][0]:
                if show_plots['Histogram: derivative'][1]:
                    _, ax = plt.subplots(2, 2, figsize=(12, 7), sharex=True, constrained_layout=True)
                else:
                    _, ax = plt.subplots(1, 2, figsize=(12, 7), sharex=True, constrained_layout=True, squeeze=False)

                ax[0, 0].hist(dsmooth1, bins=100, alpha=0.7)
                ax[0, 0].axvline(derivative_threshold1, color='red', linestyle='--', label='Threshold')
                ax[0, 0].set_title(f'Histogram: Sensor {label} Channel 1')

                ax[0, 1].hist(dsmooth2, bins=100, alpha=0.7)
                ax[0, 1].axvline(derivative_threshold2, color='red', linestyle='--', label='Threshold')
                ax[0, 1].set_title(f'Histogram: Sensor {label} Channel 2')

                if show_plots['Histogram: derivative'][1]:
                    ax[1, 0].hist(np.gradient(s1, t), bins=100, alpha=0.7)
                    ax[1, 0].set_title(f'Histogram: Sensor {label} Channel 1 (Raw)')

                    ax[1, 1].hist(np.gradient(s2, t), bins=100, alpha=0.7)
                    ax[1, 1].set_title(f'Histogram: Sensor {label} Channel 2 (Raw)')

                for a in ax.flat:
                    a.set_xlabel('Derivative (ADC count/s)')
                    a.set_ylabel('Frequency (count)')
                    a.legend(loc='upper right')
                    a.set_yscale('log')
                
            ####################################################################################################
            if show_plots['Scatter Plot: derivative vs magnitude (just events, channels combined)']:
                _, ax = plt.subplots(1, 1, figsize=(12, 7), sharex=True, constrained_layout=True)
                # ax[0].scatter(dsmooth1[event_beginnings1], smooth1[event_beginnings1], s=10, alpha=0.7, c='orange')
                ax.plot(t[events_only_mask1], smooth1_translated[events_only_mask1], 'o', markersize=0.4, alpha=0.7, c='green')
                ax.plot(t[events_only_mask2], smooth2_translated[events_only_mask2], 'o', markersize=0.4, alpha=0.7, c='blue')
                ax.plot(t[event_beginnings1], smooth1_translated[event_beginnings1], 'x', markersize=15, alpha=1, c='green')
                ax.plot(t[event_beginnings2], smooth2_translated[event_beginnings2], 'x', markersize=15, alpha=1, c='blue')
                ax.set_title(f'Derivative vs Magnitude (Events): Sensor {label} Channels 1 & 2')

                ax.set_xlabel('Derivative (ADC count/s)')
                ax.set_ylabel('Magnitude (ADC count) (channels translated to zero)')
    
        print('*'*80)

        return smooth1_adjusted, smooth2_adjusted



def remove_event_windows(t, signal, sample_rate, contact_period, thresh_fact=2):
    """Detect event windows and return the baseline-only signal."""

    # Calculate derivative and event threshold
    ds = np.gradient(signal, t)
    threshold = thresh_fact * np.std(ds[ds > 0])
    high_ds = np.where(ds > threshold)[0]

    # Determine event window lengths
    ideal_event_window = int(contact_period * sample_rate)  # number of samples = s * Hz
    smaller_event_window = int(ideal_event_window * 0.5)  # decrease event window to allow for event overlap
    larger_event_window = int(ideal_event_window * 1.5)  # increase event window to ensure full event capture

    # Keep only the first threshold crossing of each event
    event_beginnings = [high_ds[0]]  # first detected event
    for idx in high_ds[1:]:
        if idx - event_beginnings[-1] >= smaller_event_window:
            event_beginnings.append(idx)
    event_beginnings = np.array(event_beginnings)

    # Mask out event windows
    events_removed_mask = np.ones_like(t, dtype=bool)
    for idx in event_beginnings:
        end = min(idx + larger_event_window, len(events_removed_mask))
        events_removed_mask[idx:end] = False
    events_only_mask = ~events_removed_mask

    events_removed_t = t[events_removed_mask]
    events_removed_signal = signal[events_removed_mask]

    return (
        ds,
        threshold,
        event_beginnings,
        events_removed_mask,
        events_only_mask,
        events_removed_t,
        events_removed_signal,
    )

def _hampel_filter(signal, window_size, n_sigma=3):
    filtered = signal.copy()
    half = window_size // 2

    for i in range(half, len(signal) - half):
        window = signal[i-half:i+half+1]

        median = np.median(window)
        mad = np.median(np.abs(window - median))

        if mad == 0:
            continue

        threshold = n_sigma * 1.4826 * mad

        if np.abs(signal[i] - median) > threshold:
            filtered[i] = median

    return filtered

def run_stiffness_pipeline(data: HiSTIFFSData, results_note: str='None') -> None:
    """Gather refined A/B/C windows, estimate EI, write the results CSV.

    Bounds come from stalk_detector refine columns only. Gather stores
    Plot/Stalk and refine_t0/t1; estimate applies those bounds to
    data_dict force traces.
    """
    data.gather_stalk_traces()
    data.estimate_all_stalks_stiffness()
    data.save_stiffnesses(note=results_note)

if __name__ == "__main__":
    show_plots = {
    'Detect and Remove Events': True,
    'Remove Outliers, Interpolate, Smooth': False,
    'Compare Smooth vs Channel Independent Baseline Subtracted': False,
    'Compare Channel Independent Baselines 1 & 2': False,
    # [0] = show figure, [1] = include raw-signal row under the smoothed row
    'Scatter Plot: derivative vs magnitude': (False, False),
    'Histogram: magnitude': (False, False),
    'Histogram: derivative': (False, False),
    'Scatter Plot: derivative vs magnitude (just events, channels combined)': False,
    }

    times = ['090947','104213','110545','112006','113413','114710','120712','122508','123851','125311','130654']
    t_lims_df = pd.read_csv(r'Hi-STIFFS_2026_Winter\Raw Data\2026-08-28\last-three-ranges-times.csv')
    starts = t_lims_df['Start Time of Third-to-End Range (s)'].to_numpy()
    ends = t_lims_df['End Time of Run (s)']
    
    idx = 10
    data = HiSTIFFSData(date="2026-08-28", time=times[idx - 0], debug=True, nano_label="01", t_lims=[starts[idx-1], ends[idx-1]])
    if data.exists:
        # data.plot_raw_strains(combined=False)
        # data.describe_channels()
        # data.shift_initials()
        # data.moving_baseline(show_plots=show_plots)
        # data.calc_force_position(clip=False)
        # data.plot_force_position(combined=True, filter_level='valid')
        # plt.show()

        # interactive_detect_stalks(data, num_plots=3, stalks_per_plot=10)
        # display_stalk_selections(data)
        # refine_stalk_selections(data)
        run_stiffness_pipeline(data, results_note='Chesterfield Repeatability')

        plt.show()
        # keyboard.wait('space')
