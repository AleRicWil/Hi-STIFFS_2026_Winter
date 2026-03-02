# This file reads data from a CSV and processes it with multiple methods and options availible to the user
# The CSV must have columns titled like 'Time_A_sec, Strain_A1_raw, Strain_A2_raw...' and so on for each lettered sensor
# Or columns like 'Time_A_sec, Strain_A1_V, Strain_A2_V...'. This is raw ADC values or calculated voltages in volts.
# The __init__() will automatically detect which type is present, or if both are, defaults to raw option (more efficient/precise).
# Calibration is done with raw values. mV option is for human comprehension and only viable for displays, not in calculations.

# Plot number ranges:
#   0-9: raw strains

import os
import csv
from pathlib import Path
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import StrMethodFormatter
from scipy.signal import savgol_filter

from config import Config

# Hardcoded markers
HEADER_MARKER = r'===END_METADATA==='
DATA_MARKER = r'===BEGIN_DATA==='

class HiSTIFFSData:
    def __init__(self, date, time, nano_label='01', debug=False, file_path=None):
        # Fully cross-platform: uses pathlib.Path everywhere (Win / Ubuntu / RPi5 identical)
        self.nano_label = nano_label

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

    def parse_metadata(self, debug):
        # Find metadata end and data start tags — unchanged, works on all OSes
        check1 = False
        self.data_index = -1
        self.header_rows = []
        for row_index, row in enumerate(self.csv_reader):
            self.header_rows.append(row)
            if check1 and len(row) == 1 and row[0].strip() == DATA_MARKER:
                self.data_index = row_index
                break
            if len(row) == 1 and row[0].strip() == HEADER_MARKER:
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
            if debug: print(f'ADC info: {ADC_info}'); print(f'data rate: {data_rate_str}, {self.data_rate}Hz')

            self.data_dict = {}

            # Test info — unchanged
            test_info = next((row for row in self.header_rows if row and "Test Type" in row[0]), None)
            self.test_type = ' '.join(test_info[0].split()[2:]) if test_info else 'Unknown'
            if debug: print(f"test type: {self.test_type}")
            
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
            if sensors_info:
                full_str = ' '.join(str(x) for x in sensors_info)
                # Robust parse that works whether header is one cell or split
                try:
                    num_str = full_str.split("Number of ICB-Sensors:")[-1].split(',')[0].strip()
                    num_sensors = int(num_str)
                    label_str = full_str.split("Label(s):")[-1].strip().split(',')[0].strip() if "Label(s):" in full_str else ""
                    self.sensor_labels = [lbl.strip().upper() for lbl in label_str.split(',') if lbl.strip() in 'ABCDE']
                    if not self.sensor_labels:
                        self.sensor_labels = ['A','B','C','D','E']
                except:
                    self.sensor_labels = ['A','B','C','D','E']
            else:
                self.sensor_labels = ['A','B','C','D','E']

            print(f'sensors info: {sensors_info}')
            if debug: print(f"num sensors: {len(self.sensor_labels)}"); print(f"sensor labels: {self.sensor_labels}")

            if self.test_type == 'Calibration':
                print('This is a calibration file. Ignoring most of metadata')
                for l in self.sensor_labels:   # now dynamic (usually 1)
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

    def shift_initials(self, sensors='A,B,C,D,E'):
        sensors_to_shift = [label.strip() for label in sensors.split(',')]

        # Filter out invalid sensor labels
        removed = [label for label in sensors_to_shift if label not in self.sensor_labels]
        for label in removed:
            print(f"Sensor {label} not in CSV data")
        sensors_to_shift = [label for label in sensors_to_shift if label in self.sensor_labels]

        if not sensors_to_shift:
            print("No valid sensors to shift.")
            return

        for l in sensors_to_shift:
            s = self.data_dict[f'Sensor_{l}']
            if 'ini_1' not in s or 'ini_2' not in s:
                self.describe_channels()

            s['strain_1_raw'] -= s['ini_1'].astype(np.int32)
            s['strain_2_raw'] -= s['ini_2'].astype(np.int32)

    def filter_channels(self, window=None, order=1):
        if window is None:
            window = self.filter_window
        
        for l in self.sensor_labels:
            s = self.data_dict[f'Sensor_{l}']
            self.data_dict[f'Sensor_{l}']['strain_1_filter'] = savgol_filter(s['strain_1_raw'], window, order)
            self.data_dict[f'Sensor_{l}']['strain_2_filter'] = savgol_filter(s['strain_2_raw'], window, order)

    def calc_force_position(self, filter_out=False):
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
                s['position'] = np.clip(np.where(np.abs(den) > 1e9, num/den, 0.0), 0.03, 0.15) # sanity check in centimeters  

                if filter_out:
                    s['force'] =    savgol_filter(s['force'],    self.filter_window, 1)
                    s['position'] = savgol_filter(s['position'], self.filter_window, 1)
            
            except Exception as e:
                print(f"Missing data for Sensor {l}. Cannot calculate force/position. Error: {e}")
                continue

    def plot_force_position(self, sensors='A,B,C,D,E', combined=True, return_figs=False):
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
        colors = ['r', 'g', 'c', 'y', 'm']

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

            fig.suptitle(f"Calculated Force & Position – Sensors {', '.join(ordered_sensors)}\n"
                         f"Test: {self.test_type}", fontsize=12)

            for i, l in enumerate(ordered_sensors):
                s = self.data_dict[f'Sensor_{l}']
                c = colors[i]
                if 'force' not in s or 'position' not in s:
                    self.calc_force_position()   # computes for ALL sensors (idempotent)

                # Left column: Force
                axs[i, 0].plot(s['time'], s['force'], c=c, linewidth=1.4, label=f'{l} Force')
                axs[i, 0].set_ylabel(f'{l} Force (N)')
                axs[i, 0].legend(loc='upper right')
                axs[i, 0].grid(True, alpha=0.3)

                # Right column: Position (converted to mm for readability)
                axs[i, 1].plot(s['time'], s['position'] * 1000, c=c, linewidth=1.4, label=f'{l} Position')
                axs[i, 1].set_ylabel(f'{l} Position (mm)')
                axs[i, 1].legend(loc='upper right')
                axs[i, 1].grid(True, alpha=0.3)

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
                    axs[r, 1].set_ylim(p_min - pad_p, p_max + pad_p)

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
                plt.show()
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


if __name__ == "__main__":
    data = HiSTIFFSData(date="2026-03-01", time="140434", debug=True)
    if data.exists:
        # data.plot_raw_strains(combined=False)
        data.describe_channels()
        # data.shift_initials()
        data.calc_force_position(filter_out=False)
        data.plot_force_position(combined=True)
        plt.show()