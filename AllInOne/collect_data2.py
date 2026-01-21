import csv
import collections
import datetime
import os
import time
import keyboard
import pandas as pd
import pyqtgraph as pg
import requests
import argparse
from PyQt5 import QtWidgets, QtCore

# === ADS1220 parameters ===
ADS1220_BITS = 23  # True resolution in differential mode (signed)
TWO_TO_23 = 1 << ADS1220_BITS  # 8388608
ADS1220_PGA_GAIN = 128  # Configured gain
VREF = 5.1  # External reference voltage (AVDD)
VOLTS_PER_LSB = VREF / (ADS1220_PGA_GAIN * TWO_TO_23)

# === Plotting parameters ===
PLOT_REFRESH_HZ = 30  # Refresh rate for plot updates in Hz

# Hardcoded WiFi parameters
WIFI_IP = "192.168.4.1"
WIFI_PORT = 80
WIFI_URL = f"http://{WIFI_IP}:{WIFI_PORT}/"
# Hardcoded paths (adjust as needed for your environment)
CALIBRATION_PATH = r'Hi-STIFFS_2026_Winter\AllInOne\calibration_history.csv'
RAW_DATA_BASE = r'Hi-STIFFS_2026_Winter\Raw Data'

class DataReceiverWriter(QtCore.QThread):
    """Thread for receiving WiFi data, processing to volts, writing to CSV, and emitting signals to other threads."""

    data_ready = QtCore.pyqtSignal(list)  # Emits flat list [time_0, strain_01_v, strain_02_v, time_1, strain_11_v, strain_12_v, ...]
    status_signal = QtCore.pyqtSignal(str)  # For status messages
    rate_updated = QtCore.pyqtSignal(float)  # Emits updated input rate in Hz

    def __init__(self, save_format, num_sensors, sensor_labels, header_content=[" "]):
        super().__init__()
        
        print(f"Datastream status:")
        self.num_sensors = num_sensors
        self.sensor_labels = sensor_labels
        self.save_format = save_format
        self.running = True
        self.packet_times = collections.deque(maxlen=10000)  # Timestamps of received packets
        self.last_rate_time = time.time()
        self.sess = requests.Session()

        # Create CSV file
        now = datetime.datetime.now()
        date_str = now.strftime("%Y-%m-%d")
        time_str = now.strftime("%H%M%S")
        parent_folder = os.path.join(RAW_DATA_BASE, date_str)
        os.makedirs(parent_folder, exist_ok=True)
        csv_path = os.path.join(parent_folder, f'{date_str}_test_{time_str}.csv')
        self.csvfile = open(csv_path, 'w', newline='')
        self.csvwriter = csv.writer(self.csvfile)
        print(f"Created CSV at:\t{csv_path}")

        print(f"Writing metadata to CSV...")
        self.csvwriter.writerow(['===BEGIN_METADATA==='])
        for l in self.sensor_labels:
            header_content.insert(0, f'Test Name: {date_str}_test_{time_str}, yyyy-mm-dd_test_hhmmss')
            header_content += [f"ICB-Sensor {l}'s Latest Calibration: N/A, k{l}1: 1.0, d{l}1: 1.0, c{l}1: 1.0, k{l}2: 1.0, d{l}2: 1.0, c{l}2: 1.0"]
        for row in header_content:
            row = row.split(', ')
            self.csvwriter.writerow(row)
        self.csvwriter.writerow(['===END_METADATA==='])
        self.csvwriter.writerow(['===BEGIN_DATA==='])

        print(f"Writing data headers to CSV...")
        data_headers = []
        for l in self.sensor_labels:
            if save_format == 'volts':
                data_headers += [f'Time_{l}_sec', f'Strain_{l}1_V', f'Strain_{l}2_V']
            else:
                data_headers += [f'Time_{l}_sec', f'Strain_{l}1_raw', f'Strain_{l}2_raw']
        data_headers += ['Processed_Time']
        self.csvwriter.writerow(data_headers)

        print('CSV file opened for writing.')

    def run(self):
        print("Starting collection...")
        while self.running:
            try:
                print(f'Connecting to Wi-Fi at:\t{WIFI_URL}')
                response = self.sess.get(WIFI_URL, stream=True, timeout=5)
                if response.status_code != 200:
                    self.status_signal.emit(f"Connection failed: {response.status_code}")
                    time.sleep(1)
                    continue
                print("Wi-Fi connected. Reading packet datastream...")
                for line in response.iter_lines(chunk_size=1):
                    if not self.running:
                        break

                    line_str = line.decode('utf-8', errors='ignore').strip()
                    if not line_str:
                        continue

                    data = line_str.split(',')
                    if len(data) != self.num_sensors * 3:  # time, channel 1, channel 2 for each sensor
                        self.status_signal.emit(f"Invalid data packet of len:{len(data)}. Expected len:{self.num_sensors*3}")
                        continue

                    try:
                        times = []
                        volts1 = []
                        volts2 = []
                        raws1 = []
                        raws2 = []
                        for i in range(self.num_sensors):
                            time_sec = float(data[i * 3])
                            raw1 = int(data[i * 3 + 1])
                            raw2 = int(data[i * 3 + 2])
                            v1 = raw1 * VOLTS_PER_LSB
                            v2 = raw2 * VOLTS_PER_LSB
                            times.append(time_sec)
                            volts1.append(v1)
                            volts2.append(v2)
                            raws1.append(raw1)
                            raws2.append(raw2)
                    except (ValueError, IndexError):
                        self.status_signal.emit("Cannot parse data")
                        continue

                    self.packet_times.append(time.time())  # Record packet arrival time

                    now = datetime.datetime.now()
                    row = []
                    if self.save_format == 'volts':
                        for j in range(self.num_sensors):
                            row += [f"{times[j]:.6f}", volts1[j], volts2[j]]
                    else:
                        for j in range(self.num_sensors):
                            row += [f"{times[j]:.6f}", f"{raws1[j]:+08d}", f"{raws2[j]:+08d}"]
                    row += [now.time()]
                    self.csvwriter.writerow(row)
                    self.csvfile.flush()

                    # Emit data for plotting (always in volts, flat list)
                    emit_list = []
                    for j in range(self.num_sensors):
                        emit_list += [times[j], volts1[j], volts2[j]]
                    self.data_ready.emit(emit_list)

                    # Update input rate periodically
                    current_time = time.time()
                    if current_time - self.last_rate_time > 1.0:
                        if self.packet_times:
                            recent_count = sum(1 for t in self.packet_times if current_time - t <= 3.0)
                            rate = recent_count / 3.0
                            self.rate_updated.emit(rate)
                        self.last_rate_time = current_time

            except requests.exceptions.RequestException as e:
                self.status_signal.emit(f"Connection error: {str(e)}. Reconnecting...")
                time.sleep(1)
        
        print("Exited datastream loop. Wi-Fi connection closed.")
        self.csvfile.close()
        print("CSV file closed.")

class RealTimePlotWindow(QtWidgets.QMainWindow):
    """
    Class to handle real-time plotting of strain, force, and position. 
    Does not create or write to or know about CSVs.
    """

    def __init__(self, readwrite, num_sensors, sensor_labels):
        super().__init__()

        print(f"\n\tPlot windows status:")
        self.num_sensors = num_sensors
        self.sensor_labels = sensor_labels
        self.ReadWrite = readwrite
        self.ReadWrite.data_ready.connect(self.handle_data)
        self.ReadWrite.status_signal.connect(print)  # Print status to console
        self.ReadWrite.rate_updated.connect(lambda rate: self.rate_label.setText(f"Input Rate: {rate:.1f} Hz"))

        # Load calibration coefficients
        print(f"\tLoading sensor calibration...")
        self.k1 = [1.0] * self.num_sensors
        self.d1 = [1.0] * self.num_sensors
        self.c1 = [1.0] * self.num_sensors
        self.k2 = [1.0] * self.num_sensors
        self.d2 = [1.0] * self.num_sensors
        self.c2 = [1.0] * self.num_sensors
        try:
            cal_data = pd.read_csv(CALIBRATION_PATH)
            latest_cal = cal_data.iloc[-1]
            for i, s in enumerate(self.sensor_labels):
                self.k1[i] = latest_cal.get(f'k_{s}1', 1.0)
                self.d1[i] = latest_cal.get(f'd_{s}1', 1.0)
                self.c1[i] = latest_cal.get(f'c_{s}1', 1.0)
                self.k2[i] = latest_cal.get(f'k_{s}2', 1.0)
                self.d2[i] = latest_cal.get(f'd_{s}2', 1.0)
                self.c2[i] = latest_cal.get(f'c_{s}2', 1.0)
            print(f"\tSuccessfully loaded calibration.")
        except Exception as e:
            print(f"\tError loading calibration: {str(e)}. Using defaults (all 1.0).")

        # Performance optimizations for high refresh rates
        pg.setConfigOptions(useOpenGL=True, antialias=False)

        # Strain plot setup, now directly in this QMainWindow as central widget
        print(f"\tBuilding plot windows...")
        print(f"\t\tCreating strain plots...")
        self.setWindowTitle("Strain Data Plots")  # Set title on self (the main window)
        self.win_strain = pg.GraphicsLayoutWidget()
        self.plot_ch1 = self.win_strain.addPlot(title='Channel 1 Strains')
        self.plot_ch1.setLabel('left', '', units='mV')
        self.plot_ch1.addLegend()
        self.plot_ch2 = self.win_strain.addPlot(title='Channel 2 Strains')
        self.plot_ch2.setLabel('left', '', units='mV')
        self.plot_ch2.addLegend()

        colors = ['r', 'b', 'g', 'y', 'c'][:self.num_sensors]
        self.curves_ch1 = []
        self.curves_ch2 = []
        for i, s in enumerate(self.sensor_labels):
            self.curves_ch1.append(self.plot_ch1.plot(pen=colors[i], name=f'{s}1 Strain'))
            self.curves_ch2.append(self.plot_ch2.plot(pen=colors[i], name=f'{s}2 Strain'))

        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(self.win_strain)

        # Add preset buttons for display time range and rate label
        print("\t\tConnecting plot window buttons...")
        preset_layout = QtWidgets.QHBoxLayout()
        preset_label = QtWidgets.QLabel("Time Range (s):")
        preset_layout.addWidget(preset_label)
        presets = [0.1, 0.5, 1, 3, 5, 10, 15, 30]
        for preset in presets:
            btn = QtWidgets.QPushButton(str(preset))
            btn.clicked.connect(lambda _, p=preset: self.set_time_range(p))
            preset_layout.addWidget(btn)
        print("\t\tConnecting 'Input Rate' indicator...")
        self.rate_label = QtWidgets.QLabel("Input Rate: 0 Hz")
        preset_layout.addStretch()
        preset_layout.addWidget(self.rate_label)
        layout.addLayout(preset_layout)

        # Set the layout on a central widget for this QMainWindow
        central_widget = QtWidgets.QWidget()
        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)
        self.resize(1000, 600)
        self.move(0, 0)
        self.show()  # Show this main window

        # Force and position plot window
        print(f"\t\tCreating force/position plots...")
        self.win_force_pos = pg.GraphicsLayoutWidget(title="Force and Position")
        self.win_force_pos.resize(1000, 600)
        self.win_force_pos.move(1000, 0)
        self.plot_force = self.win_force_pos.addPlot(title='Force')
        self.plot_force.addLegend()
        self.plot_pos = self.win_force_pos.addPlot(title='Position')
        self.plot_pos.addLegend()

        self.curves_force = []
        self.curves_pos = []
        for i, s in enumerate(self.sensor_labels):
            self.curves_force.append(self.plot_force.plot(pen=colors[i], name=f'Force {s}'))
            self.curves_pos.append(self.plot_pos.plot(pen=colors[i], name=f'Position {s}'))
        self.win_force_pos.show()  # Show it here

        # Install event filter on force/pos window to catch space presses there too
        self.win_force_pos.installEventFilter(self)

        self.display_time_range = 10.0  # Initial time range in seconds

        # Deques for plotting data
        maxlen = 30 * 120  # Sufficient for ~30s at 120 Hz
        self.times = [collections.deque(maxlen=maxlen) for _ in range(self.num_sensors)]
        self.strains1 = [collections.deque(maxlen=maxlen) for _ in range(self.num_sensors)]
        self.strains2 = [collections.deque(maxlen=maxlen) for _ in range(self.num_sensors)]
        self.forces = [collections.deque(maxlen=maxlen) for _ in range(self.num_sensors)]
        self.positions = [collections.deque(maxlen=maxlen) for _ in range(self.num_sensors)]
        print(f"\t\tSet up deques for plot elements. Max size:{maxlen}")

        # Set up timer for fixed-rate plot updates
        print(f"\t\tStarting plot refresh timer...")
        self.plot_timer = QtCore.QTimer()
        self.plot_timer.timeout.connect(self.update_plots)
        self.plot_timer.start(int(1000 / PLOT_REFRESH_HZ))  # Interval in ms
        print(f"\tSuccessfully created plot windows.")

    def handle_data(self, data_list):
        """Handle emitted data: compute force/position, append to deques."""
        if len(data_list) != self.num_sensors * 3:
            print("Invalid data list length received for plotting.")
            return

        times = [data_list[j * 3] for j in range(self.num_sensors)]
        strains1 = [data_list[j * 3 + 1] for j in range(self.num_sensors)]
        strains2 = [data_list[j * 3 + 2] for j in range(self.num_sensors)]

        for i in range(self.num_sensors):
            time_sec = times[i]
            strain1 = strains1[i]
            strain2 = strains2[i]

            # Calculate force and position
            force = 0# (self.k2[i] * (strain1 - self.c1[i]) - self.k1[i] * (strain2 - self.c2[i])) / (self.k1[i] * self.k2[i] * (self.d2[i] - self.d1[i]))
            num = 0# (self.k2[i] * self.d2[i] * (strain1 - self.c1[i]) - self.k1[i] * self.d1[i] * (strain2 - self.c2[i]))
            den = 0# (self.k2[i] * (strain1 - self.c1[i]) - self.k1[i] * (strain2 - self.c2[i]))
            position = 0# num / den if abs(den) > 2.5e-5 else 0.0
            position = 0# 0.0 if position > 0.25 or position < -0.10 else position

            self.times[i].append(time_sec)
            self.strains1[i].append(strain1 * 1000.0)  # mV for display
            self.strains2[i].append(strain2 * 1000.0)
            self.forces[i].append(force)
            self.positions[i].append(position * 100)  # to cm

    def update_plots(self):
        """Update all plot curves and ranges."""
        for i in range(self.num_sensors):
            self.curves_ch1[i].setData(self.times[i], self.strains1[i])
            self.curves_ch2[i].setData(self.times[i], self.strains2[i])
            self.curves_force[i].setData(self.times[i], self.forces[i])
            self.curves_pos[i].setData(self.times[i], self.positions[i])

        t_max = 0
        for t in self.times:
            if t:
                t_max = max(t_max, t[-1])
        x_min = max(0, t_max - self.display_time_range)
        x_max = t_max

        self.plot_ch1.setXRange(x_min, x_max)
        self.plot_ch2.setXRange(x_min, x_max)
        self.plot_force.setXRange(x_min, x_max)
        self.plot_pos.setXRange(x_min, x_max)

    def set_time_range(self, value):
        """Set the display time range based on button preset."""
        self.display_time_range = float(value)
        self.update_plots()

    def eventFilter(self, obj, event):
        """Catch key events on filtered windows (e.g., force/pos)."""
        if event.type() == QtCore.QEvent.KeyPress and event.key() == QtCore.Qt.Key_Space:
            self.stop_collection()
            return True  # Event handled
        return super().eventFilter(obj, event)

    def keyPressEvent(self, event):
        """Handle key press events for stopping collection."""
        if event.key() == QtCore.Qt.Key_Space:
            self.stop_collection()

    def stop_collection(self):
        """Stop data collection and clean up."""
        print("Keyboard 'space' was pressed. Exiting datastream loop...")
        self.close()  # Close strain window (self)
        self.win_force_pos.close()  # Close force/pos window
        print(f'Plots closed.')
        self.plot_timer.stop()
        self.ReadWrite.running = False
        self.ReadWrite.wait()  # Wait for thread to finish

def run_collection(save_format='raw', plot=True, sensors='A', header_content=[" "]):
    if sensors.isdigit():
        num = int(sensors)
        if num < 1 or num > 5:
            raise ValueError("Number of sensors must be between 1 and 5.")
        sensor_labels = [chr(65 + i) for i in range(num)]
    else:
        sensor_labels = sorted(set(s.strip().upper() for s in sensors.split(',')), key='ABCDE'.index)
        if not sensor_labels or any(s not in 'ABCDE' for s in sensor_labels):
            raise ValueError("Invalid sensor labels; must be comma-separated from A-E, no duplicates.")
    num_sensors = len(sensor_labels)

    ReadWrite = DataReceiverWriter(save_format, num_sensors, sensor_labels, header_content)
    if plot:
        app = QtWidgets.QApplication([])
        window = RealTimePlotWindow(ReadWrite, num_sensors, sensor_labels)
        ReadWrite.start()
        print("=== Press/Hold 'space' to end data collection ===")
        app.exec_()
    else:
        ReadWrite.status_signal.connect(print)
        ReadWrite.rate_updated.connect(lambda rate: print(f"Input Rate: {rate:.1f} Hz"))
        ReadWrite.start()
        print("=== Press/Hold 'space' to end data collection ===")
        while True:
            if keyboard.is_pressed('space'):
                print("Keyboard 'space' was pressed. Exiting datastream loop...")
                ReadWrite.running = False
                break
            time.sleep(0.1)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Data collection from WiFi stream")
    parser.add_argument('--save-format', choices=['volts', 'raw'], default='raw', help="Format to save strains in CSV: volts or raw")
    parser.add_argument('--plot', type=bool, default=True, help="Enable live plotting")
    parser.add_argument('--sensors', default='A', help="Number of sensors (1-5) or comma-separated labels (e.g., 'A,C,E'). Note: Data must arrive in the specified order; configure Arduino accordingly for non-sequential labels.")
    args = parser.parse_args()

    my_header_content = [
        "Thow-away trial run. Not real data"
        "Test Type: Force Cycle, Force: 20N, Cycles: 66, Dwell Time: 1sec, Tool Accuracy: +/-0.05N",
        "Test Number in Session: 10, Time since Last Test: ~10min",
        f"Number of ICB-Sensors: 1, Sensor Label(s): {args.sensors}",
        "ICB-Sensor B's Serial#: 002, Length: 120mm, Spacing: 40mm, Saturation Load: 80N, Factor of Safety at Saturation: 1.5",
        "Analog-to-Digital Converter: ADS1220, Mode: Turbo, Data Rate: DR_90SPS, Analog Excitation/Reference Voltage: 5.1V +/-2mV",
        "DAQ Microcontroller: Arduino Nano ESP32, ID: Hi-STIFFS_Nano, CPU Clock: 240MHz, Cores: 2, Data-stream Connection: Wi-Fi"]
    run_collection(args.save_format, args.plot, args.sensors, my_header_content)