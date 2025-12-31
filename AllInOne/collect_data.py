import serial
import csv
import pandas as pd
from datetime import datetime
import pyqtgraph as pg
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtWidgets import QPushButton, QLabel, QHBoxLayout, QVBoxLayout, QWidget
from multiprocessing import Queue
import time
import os
import collections
import keyboard

# === ADS1220 parameters ===
ADS1220_BITS            = 23                    # True resolution in differential mode (signed)
TWO_TO_23               = 1 << ADS1220_BITS     # 8388608
ADS1220_PGA_GAIN        = 128                   # Your configured gain in Arduino
VREF                    = 5.1                   # External reference voltage (AVDD)
# Volts per LSB (least significant bit of ADC)
VOLTS_PER_LSB = VREF / (ADS1220_PGA_GAIN * TWO_TO_23)

# NOTE ON UNITS:
# - CSV file: all strain values are stored in Volts (required for calibration consistency)
# - Live plots: strain traces are displayed in millivolts (×1000) for readability
# - Force/position calculations use the original Volt values — never altered

# === Plotting parameters ===
PLOT_REFRESH_HZ = 60  # Refresh rate for plot updates in Hz; start at 60, test up to 120 if stable

class SerialReader(QtCore.QThread):
    """Thread for reading serial data, processing, and writing to CSV."""

    data_ready = QtCore.pyqtSignal(list)  # Emits processed data for plotting
    stop_signal = QtCore.pyqtSignal()     # Emits to trigger stop in GUI
    status_signal = QtCore.pyqtSignal(str)  # For status messages
    rate_updated = QtCore.pyqtSignal(float)  # Emits updated input rate in Hz

    def __init__(self, ser, csvfile, csvwriter, k_A1, d_A1, c_A1, k_A2, d_A2, c_A2,
                 k_B1, d_B1, c_B1, k_B2, d_B2, c_B2):
        super().__init__()
        self.ser = ser
        self.csvfile = csvfile
        self.csvwriter = csvwriter
        self.k_A1 = k_A1
        self.d_A1 = d_A1
        self.c_A1 = c_A1
        self.k_A2 = k_A2
        self.d_A2 = d_A2
        self.c_A2 = c_A2
        self.k_B1 = k_B1
        self.d_B1 = d_B1
        self.c_B1 = c_B1
        self.k_B2 = k_B2
        self.d_B2 = d_B2
        self.c_B2 = c_B2
        self.running = True
        self.packet_times = collections.deque(maxlen=10000)  # Timestamps of received packets
        self.last_rate_time = time.time()

    def run(self):
        time_offset_check = True
        time_offset = 0.0
        while self.running:
            if keyboard.is_pressed('space'):
                self.stop_signal.emit()
                break

            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if line == "test ended":
                self.stop_signal.emit()
                break
            if line.startswith('$'):
                reset_time = float(line.split(',')[1]) * 1e-6 if len(line.split(',')) > 1 else 0.0
                self.status_signal.emit(f"Reset at {reset_time}")
                continue
            data = line.split(',')
            expected_len = 6  # For two sensors: tsA, A1, A2, tsB, B1, B2
            if len(data) < expected_len:
                self.status_signal.emit("Invalid data packet")
                continue
            try:
                time_A_sec = float(data[0])
                strain_A1 = int(data[1]) * VOLTS_PER_LSB
                strain_A2 = int(data[2]) * VOLTS_PER_LSB
                time_B_sec = float(data[3])
                strain_B1 = int(data[4]) * VOLTS_PER_LSB
                strain_B2 = int(data[5]) * VOLTS_PER_LSB
            except (ValueError, IndexError):
                self.status_signal.emit("Cannot parse data")
                continue

            if time_offset_check:
                time_offset = time_A_sec
                time_offset_check = False
            time_A_sec -= time_offset

            self.packet_times.append(time.time())  # Record packet arrival time

            now = datetime.now()
            self.csvwriter.writerow([time_A_sec, strain_A1, strain_A2, strain_B1, strain_B2, now.time()])
            self.csvfile.flush()

            # Calculate force and position for sensor A
            force_A = (self.k_A2 * (strain_A1 - self.c_A1) - self.k_A1 * (strain_A2 - self.c_A2)) / (self.k_A1 * self.k_A2 * (self.d_A2 - self.d_A1))
            num_A = (self.k_A2 * self.d_A2 * (strain_A1 - self.c_A1) - self.k_A1 * self.d_A1 * (strain_A2 - self.c_A2))
            den_A = (self.k_A2 * (strain_A1 - self.c_A1) - self.k_A1 * (strain_A2 - self.c_A2))
            position_A = num_A / den_A if abs(den_A) > 2.5e-5 else 0.0
            position_A = 0.0 if position_A > 0.25 or position_A < -0.10 else position_A

            # Calculate force and position for sensor B
            force_B = (self.k_B2 * (strain_B1 - self.c_B1) - self.k_B1 * (strain_B2 - self.c_B2)) / (self.k_B1 * self.k_B2 * (self.d_B2 - self.d_B1))
            num_B = (self.k_B2 * self.d_B2 * (strain_B1 - self.c_B1) - self.k_B1 * self.d_B1 * (strain_B2 - self.c_B2))
            den_B = (self.k_B2 * (strain_B1 - self.c_B1) - self.k_B1 * (strain_B2 - self.c_B2))
            position_B = num_B / den_B if abs(den_B) > 2.5e-5 else 0.0
            position_B = 0.0 if position_B > 0.25 or position_B < -0.10 else position_B

            # Emit data for plotting (use time_A_sec for all, as timestamps are close)
            self.data_ready.emit([time_A_sec, strain_A1, strain_A2, strain_B1, strain_B2,
                                  force_A, position_A * 100, force_B, position_B * 100])

            # Update input rate periodically
            current_time = time.time()
            if current_time - self.last_rate_time > 1.0:
                if self.packet_times:
                    recent_count = sum(1 for t in self.packet_times if current_time - t <= 3.0)
                    rate = recent_count / 3.0
                    self.rate_updated.emit(rate)
                self.last_rate_time = current_time

class RealTimePlotWindow(QtWidgets.QMainWindow):
    """Class to handle real-time strain, force, and position data collection and plotting from an Arduino.

    Attributes:
        ser (serial.Serial): Serial connection to the Arduino.
        csvfile (file): CSV file for data storage.
        csvwriter (csv.writer): Writer for CSV data.
        win_strain (pg.GraphicsLayoutWidget): PyQtGraph window for strain plotting.
        win_force_pos (pg.GraphicsLayoutWidget): PyQtGraph window for force and position plotting.
        plot_ch1 (pg.PlotItem): Plot for channel 1 strains (A1, B1).
        plot_ch2 (pg.PlotItem): Plot for channel 2 strains (A2, B2).
        plot_force (pg.PlotItem): Plot for force.
        plot_pos (pg.PlotItem): Plot for position.
        curve_A1 (pg.PlotDataItem): Plot curve for A1 strain.
        curve_B1 (pg.PlotDataItem): Plot curve for B1 strain.
        curve_A2 (pg.PlotDataItem): Plot curve for A2 strain.
        curve_B2 (pg.PlotDataItem): Plot curve for B2 strain.
        curve_force_A (pg.PlotDataItem): Plot curve for force A.
        curve_force_B (pg.PlotDataItem): Plot curve for force B.
        curve_pos_A (pg.PlotDataItem): Plot curve for position A.
        curve_pos_B (pg.PlotDataItem): Plot curve for position B.
        time_sec (collections.deque): Deque of time values.
        strain_A1 (collections.deque): Deque of A1 strain values.
        strain_B1 (collections.deque): Deque of B1 strain values.
        strain_A2 (collections.deque): Deque of A2 strain values.
        strain_B2 (collections.deque): Deque of B2 strain values.
        force_A (collections.deque): Deque of force A values.
        position_A (collections.deque): Deque of position A values.
        force_B (collections.deque): Deque of force B values.
        position_B (collections.deque): Deque of position B values.
        plot_timer (QtCore.QTimer): Timer for updating plots.
        k_A1, d_A1, c_A1, etc.: Calibration coefficients.
    """

    def __init__(self, port, config, status_queue):
        """Initialize the plot windows and serial connection.

        Args:
            port (str): Serial port for Arduino communication.
            config (dict): Configuration dictionary with test parameters.
            status_queue (Queue): Queue to send status messages to the UI.
        """
        super().__init__()
        self.status_queue = status_queue

        # Load calibration coefficients
        cal_csv_path = r'Hi-STIFFS_2026_Winter\AllInOne\calibration_history.csv'
        try:
            cal_data = pd.read_csv(cal_csv_path)
            latest_cal = cal_data.iloc[-1]

            self.k_A1 = latest_cal['k_A1']
            self.d_A1 = latest_cal['d_A1']
            self.c_A1 = latest_cal['c_A1']
            self.k_A2 = latest_cal['k_A2']
            self.d_A2 = latest_cal['d_A2']
            self.c_A2 = latest_cal['c_A2']

            self.k_B1 = latest_cal['k_B1']
            self.d_B1 = latest_cal['d_B1']
            self.c_B1 = latest_cal['c_B1']
            self.k_B2 = latest_cal['k_B2']
            self.d_B2 = latest_cal['d_B2']
            self.c_B2 = latest_cal['c_B2']
        except Exception as e:
            self.status_queue.put(f"Error loading calibration: {str(e)}")
            return

        try:
            self.ser = serial.Serial(port, 2000000, timeout=1)
        except serial.SerialException as e:
            self.status_queue.put(f"Failed to connect to {port}: {str(e)}")
            return

        count = 0
        while True:
            count += 1
            incoming_data = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if count <= 1:
                time.sleep(2)
                self.status_queue.put("Press 'space' to sync with Arduino")
            if incoming_data == "#" or keyboard.is_pressed('space'):
                self.status_queue.put("Starting data collection")
                break

        # Create parent folder based on date
        parent_folder = os.path.join(r'Hi-STIFFS_2026_Winter\Raw Data', f'{config["date"]}')
        os.makedirs(parent_folder, exist_ok=True)

        # Open CSV file in the parent folder
        csv_path = os.path.join(parent_folder, f'{config["date"]}_test_{config["test_num"]}.csv')
        self.csvfile = open(csv_path, 'w', newline='')
        self.csvwriter = csv.writer(self.csvfile)

        pre_test_notes = [
            ["user_note", '', '', '', '', ''],
            ["configuration", config["configuration"], '', '', '', ''],
            ["sensor calibration (k d c)", f'{self.k_A1} {self.k_A2} {self.k_B1} {self.k_B2}', f'{self.d_A1} {self.d_A2} {self.d_B1} {self.d_B2}', f'{self.c_A1} {self.c_A2} {self.c_B1} {self.c_B2}', '', ''],
            ["stalk array (lo med hi)", config["pvc_stiffness"], '', '', '', ''],
            ["sensor height (cm)", config["height"], '', '', '', ''],
            ["sensor yaw (degrees)", config["yaw"], '', '', '', ''],
            ["sensor pitch (degrees)", config["pitch"], '', '', '', ''],
            ["sensor roll (degrees)", config["roll"], '', '', '', ''],
            ["rate of travel (ft/min)", config["rate_of_travel"], '', '', '', ''],
            ["angle of travel (degrees)", config["angle_of_travel"], '', '', '', ''],
            ["sensor offset (cm to gauge 2)", config["offset_distance"], '', '', '', ''],
            ["====="]
        ]
        for note in pre_test_notes:
            self.csvwriter.writerow(note)

        headers = ['Time', 'Strain A1', 'Strain A2', 'Strain B1', 'Strain B2', 'Current Time']
        self.csvwriter.writerow(headers)

        # Performance optimizations for high refresh rates
        pg.setConfigOptions(useOpenGL=True, antialias=False)

        # Strain plot window with preset buttons for time range
        self.strain_window = QWidget()
        self.strain_window.setWindowTitle(f"Strain Data Plots - Test {config['test_num']}")
        layout = QVBoxLayout()

        self.win_strain = pg.GraphicsLayoutWidget()
        self.plot_ch1 = self.win_strain.addPlot(title='Channel 1 Strains')
        self.plot_ch1.setLabel('left', '', units='mV')
        self.plot_ch1.addLegend()
        # Display-only scaling: actual data in CSV remains in Volts
        self.curve_A1 = self.plot_ch1.plot(pen='r', name='A1 Strain')
        self.curve_B1 = self.plot_ch1.plot(pen='b', name='B1 Strain')

        self.plot_ch2 = self.win_strain.addPlot(title='Channel 2 Strains')
        self.plot_ch2.setLabel('left', '', units='mV')
        self.plot_ch2.addLegend()
        # Display-only scaling: actual data in CSV remains in Volts
        self.curve_A2 = self.plot_ch2.plot(pen='r', name='A2 Strain')
        self.curve_B2 = self.plot_ch2.plot(pen='b', name='B2 Strain')

        layout.addWidget(self.win_strain)

        # Add preset buttons for display time range and rate label
        preset_layout = QHBoxLayout()
        preset_label = QLabel("Time Range (s):")
        preset_layout.addWidget(preset_label)
        presets = [0.1, 0.5, 1, 3, 5, 10, 15, 20]
        for preset in presets:
            btn = QPushButton(str(preset))
            btn.clicked.connect(lambda _, p=preset: self.set_time_range(p))
            preset_layout.addWidget(btn)
        self.rate_label = QLabel("Input Rate: 0 Hz")
        preset_layout.addStretch()
        preset_layout.addWidget(self.rate_label)
        layout.addLayout(preset_layout)

        self.strain_window.setLayout(layout)
        self.strain_window.resize(1000, 600)
        self.strain_window.move(0, 0)
        self.strain_window.show()

        self.display_time_range = 10.0  # Initial time range in seconds

        # Force and position plot window
        self.win_force_pos = pg.GraphicsLayoutWidget(show=True, title=f"Force and Position - Test {config['test_num']}")
        self.win_force_pos.resize(1000, 600)
        self.win_force_pos.move(1000, 0)
        self.plot_force = self.win_force_pos.addPlot(title='Force')
        self.plot_force.addLegend()
        self.curve_force_A = self.plot_force.plot(pen='r', name='Force A')
        self.curve_force_B = self.plot_force.plot(pen='b', name='Force B')

        self.plot_pos = self.win_force_pos.addPlot(title='Position')
        self.plot_pos.addLegend()
        self.curve_pos_A = self.plot_pos.plot(pen='r', name='Position A')
        self.curve_pos_B = self.plot_pos.plot(pen='b', name='Position B')

        # Use deques for plotting data to limit memory usage
        maxlen = 20 * 120  # Sufficient for ~20s at 120 Hz (safety for high inputs)
        self.time_sec = collections.deque(maxlen=maxlen)
        self.strain_A1 = collections.deque(maxlen=maxlen)
        self.strain_A2 = collections.deque(maxlen=maxlen)
        self.strain_B1 = collections.deque(maxlen=maxlen)
        self.strain_B2 = collections.deque(maxlen=maxlen)
        self.force_A = collections.deque(maxlen=maxlen)
        self.position_A = collections.deque(maxlen=maxlen)
        self.force_B = collections.deque(maxlen=maxlen)
        self.position_B = collections.deque(maxlen=maxlen)

        # Create and start the serial reader thread
        self.reader = SerialReader(self.ser, self.csvfile, self.csvwriter,
                                   self.k_A1, self.d_A1, self.c_A1, self.k_A2, self.d_A2, self.c_A2,
                                   self.k_B1, self.d_B1, self.c_B1, self.k_B2, self.d_B2, self.c_B2)
        self.reader.data_ready.connect(self.handle_data)
        self.reader.stop_signal.connect(self.stop_collection)
        self.reader.status_signal.connect(lambda msg: self.status_queue.put(msg))
        self.reader.rate_updated.connect(lambda rate: self.rate_label.setText(f"Input Rate: {rate:.1f} Hz"))
        self.reader.start()

        # Set up timer for fixed-rate plot updates
        self.plot_timer = QtCore.QTimer()
        self.plot_timer.timeout.connect(self.update_plots)
        self.plot_timer.start(int(1000 / PLOT_REFRESH_HZ))  # Interval in ms

        self.status_queue.put("Press 'space' to end data collection")

    def handle_data(self, data_list):
        """Handle emitted data from the thread: append to deques (updates handled by timer)."""
        time_sec, strain_A1, strain_A2, strain_B1, strain_B2, force_A, position_A, force_B, position_B = data_list

        self.time_sec.append(time_sec)
        # === Strain values: convert to mV for display only ===
        # NOTE: CSV file still records true voltage in Volts.
        #       Calibration coefficients (k, d, c) were determined using volts,
        #       so we must NOT change the values written to disk.
        self.strain_A1.append(strain_A1 * 1000.0)   # Display in mV
        self.strain_A2.append(strain_A2 * 1000.0)   # Display in mV
        self.strain_B1.append(strain_B1 * 1000.0) # Display in mV
        self.strain_B2.append(strain_B2 * 1000.0) # Display in mV
        # ====================================================
        self.force_A.append(force_A)
        self.position_A.append(position_A)
        self.force_B.append(force_B)
        self.position_B.append(position_B)

    def update_plots(self):
        """Update all plot curves and ranges."""
        self.curve_A1.setData(self.time_sec, self.strain_A1)
        self.curve_A2.setData(self.time_sec, self.strain_A2)
        self.curve_B1.setData(self.time_sec, self.strain_B1)
        self.curve_B2.setData(self.time_sec, self.strain_B2)
        self.curve_force_A.setData(self.time_sec, self.force_A)
        self.curve_force_B.setData(self.time_sec, self.force_B)
        self.curve_pos_A.setData(self.time_sec, self.position_A)
        self.curve_pos_B.setData(self.time_sec, self.position_B)

        if self.time_sec:
            x_min = max(0, self.time_sec[-1] - self.display_time_range)
            x_max = self.time_sec[-1]
        else:
            x_min = 0
            x_max = 0

        self.plot_ch1.setXRange(x_min, x_max)
        self.plot_ch2.setXRange(x_min, x_max)
        self.plot_force.setXRange(x_min, x_max)
        self.plot_pos.setXRange(x_min, x_max)

    def set_time_range(self, value):
        """Set the display time range based on button preset."""
        self.display_time_range = float(value)
        # Trigger an immediate plot update to reflect the new range
        self.update_plots()

    def keyPressEvent(self, event):
        """Handle key press events for stopping collection."""
        if event.key() == QtCore.Qt.Key_Space:
            self.stop_collection()

    def stop_collection(self):
        """Stop data collection and clean up."""
        self.status_queue.put("Data collection ended")
        self.plot_timer.stop()
        self.reader.running = False
        self.reader.wait()  # Wait for thread to finish
        self.ser.close()
        self.csvfile.close()
        self.strain_window.close()
        self.win_force_pos.close()

def run_collection(port, config, status_queue):
    """Run the real-time plot window in a separate process.
    Args:
        port (str): Serial port for Arduino communication.
        config (dict): Configuration dictionary with test parameters.
        status_queue (Queue): Queue to send status messages to the UI.
    """
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window = RealTimePlotWindow(port, config, status_queue)
    app.exec_()