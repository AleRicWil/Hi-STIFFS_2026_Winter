import csv
import collections
import datetime
import os
import time
import socket
import struct
import keyboard
import pandas as pd
import pyqtgraph as pg
import argparse
from PyQt5 import QtWidgets, QtCore
import queue  # For thread-safe queues
import threading  # For separate processing thread

# === ADS1220 parameters ===
ADS1220_BITS = 23  # True resolution in differential mode (signed)
TWO_TO_23 = 1 << ADS1220_BITS  # 8388608
ADS1220_PGA_GAIN = 128  # Configured gain
VREF = 5.1  # External reference voltage (AVDD)
VOLTS_PER_LSB = VREF / (ADS1220_PGA_GAIN * TWO_TO_23)

# === Plotting parameters ===
PLOT_REFRESH_HZ = 30  # Refresh rate for plot updates in Hz
PROCESS_FPS = 30  # Set desired FPS for processing/dequeuing (e.g., 30 for smooth plotting)

# Hardcoded WiFi parameters (now for host server)
HOST_IP = "192.168.137.1"
# HOST_IP = "192.168.1.238"
HOST_PORT = 80
# HOST_URL = f"http://{HOST_IP}:{HOST_PORT}/data"  # No longer used; switched to persistent TCP

# Hardcoded paths (adjust as needed for your environment) - os.path ensures cross-platform path handling
CALIBRATION_PATH = r'Hi-STIFFS_2026_Winter/AllInOne/calibration_history.csv'
RAW_DATA_BASE = r'Hi-STIFFS_2026_Winter/Raw Data'

# Batching configuration: Process received packets in batches every this interval (ms)
BATCH_PROCESS_INTERVAL_MS = 100  # Default 0.1s; adjust for desired batch processing frequency

class DataReceiverWriter(QtCore.QThread):
    """Thread for hosting a TCP server to receive WiFi data from Nano, processing to volts, writing to CSV, and emitting signals to other threads.
    Now uses persistent TCP socket instead of HTTP. Receives length-prefixed binary frames.
    Adds a queue for received packets, which are batched and processed in a separate thread."""

    data_ready = QtCore.pyqtSignal(list)  # Emits flat list [time_0, strain_01_v, strain_02_v, time_1, strain_11_v, strain_12_v, ...] (batched)
    status_signal = QtCore.pyqtSignal(str)  # For status messages
    rate_updated = QtCore.pyqtSignal(float)  # Emits updated input rate in Hz

    def __init__(self, save_format, num_sensors, sensor_labels, header_content=None):  # Made header_content optional with default None
        super().__init__()
        
        probe_num = '01'
        print(f"Datastream from Nano {probe_num} status:")
        self.num_sensors = num_sensors
        self.sensor_labels = sensor_labels
        self.save_format = save_format
        self.running = True
        self.packet_times = collections.deque(maxlen=10000)  # Timestamps of received packets
        self.last_rate_time = time.time()

        # Create CSV file - os.makedirs and os.path.join ensure cross-platform directory creation and path compatibility
        now = datetime.datetime.now()
        date_str = now.strftime("%Y-%m-%d")
        time_str = now.strftime("%H%M%S")
        parent_folder = os.path.join(RAW_DATA_BASE, date_str)
        os.makedirs(parent_folder, exist_ok=True)
        csv_path = os.path.join(parent_folder, f'{date_str}_{time_str}_{probe_num}.csv')  # Added _01 suffix
        self.csvfile = open(csv_path, 'w', newline='')
        self.csvwriter = csv.writer(self.csvfile)
        print(f"Created CSV for Nano_{probe_num} at:\t{csv_path}")

        print(f"Writing metadata to CSV...")
        self.csvwriter.writerow(['===BEGIN_METADATA==='])
        # Handle header_content: If None (e.g., standalone run), use minimal defaults; GUI will provide full list
        if header_content is None:
            header_content = []  # Empty default; add minimal required for standalone
            for l in self.sensor_labels:
                header_content += [f"ICB-Sensor {l}'s Latest Calibration: N/A, k{l}1: 1.0, d{l}1: 1.0, c{l}1: 1.0, k{l}2: 1.0, d{l}2: 1.0, c{l}2: 1.0"]
        
        header_content.insert(0, f'Test Name: {date_str}_{time_str}_{probe_num}, ' + 'yyyy-mm-dd_hhmmss_{probe_num}')
        for row in header_content:
            row = row.split(', ')
            self.csvwriter.writerow(row)
        self.csvwriter.writerow(['===END_METADATA==='])
        self.csvwriter.writerow(['===BEGIN_DATA==='])

        print(f"Writing data headers to CSV...")
        data_headers = []
        for l in self.sensor_labels:
            data_headers += [f'Time_{l}_sec', f'Strain_{l}1_raw', f'Strain_{l}2_raw']
        data_headers += ['Processed_Time']
        self.csvwriter.writerow(data_headers)

        print('CSV file opened for writing.')

        # Queue for received raw packets (binary data) to be batched and processed
        self.receive_queue = queue.Queue()  # Thread-safe queue
        self.processed_buffer = collections.deque()  # Buffer for processed per-packet emit_lists (post-unpack/volts)
        self.rate_estimate = 10.0  # Initial guess for input rate (Hz); updated from times
        self.last_rate_update = time.perf_counter()  # For periodic re-estimation
        self.collected_first_times = []  # List to accumulate t0 from packets for delta calc

        # Start separate processing thread for batching
        self.processing_thread = threading.Thread(target=self.process_batches, daemon=True)
        self.processing_thread.start()

    def read_fully(self, conn, size):
        data = b''
        while len(data) < size:
            chunk = conn.recv(size - len(data))
            if not chunk:
                raise EOFError("Connection closed during read")
            data += chunk
        return data

    def crc16_ccitt(self, data):
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 1:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc

    def run(self):
        print("Starting TCP server...")
        # Set up TCP socket server
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.bind((HOST_IP, HOST_PORT))
        server_socket.listen(1)  # Listen for one connection (persistent from Nano)
        server_socket.settimeout(0.1)  # Non-blocking with timeout for loop control

        print(f"TCP server listening at {HOST_IP}:{HOST_PORT}")
        conn = None
        while self.running:
            if conn is None:
                try:
                    conn, addr = server_socket.accept()
                    print(f"Accepted persistent connection from {addr}")
                except socket.timeout:
                    continue

            try:
                # Read full header: 2 length + 2 seq + 2 crc (6 bytes, little-endian)
                header = self.read_fully(conn, 6)
                length = struct.unpack('<H', header[:2])[0]
                seq = struct.unpack('<H', header[2:4])[0]
                received_crc = struct.unpack('<H', header[4:6])[0]

                # Read payload fully
                post_data = self.read_fully(conn, length)
                if len(post_data) != length:
                    self.status_signal.emit(f"Invalid data packet of len:{len(post_data)}. Expected len:{length}")
                    continue

                # Compute CRC over payload (adjust to match Arduino's CRC-CCITT; here using CRC32 truncated for example)
                computed_crc = self.crc16_ccitt(post_data)
                if computed_crc != received_crc:
                    self.status_signal.emit(f"CRC mismatch: received {received_crc}, computed {computed_crc}. Dropping packet.")
                    continue

                # Optional: Track sequence for drops (e.g., maintain self.last_seq in __init__ as -1)
                # if hasattr(self, 'last_seq') and seq != self.last_seq + 1:
                #     self.status_signal.emit(f"Sequence drop detected: expected {self.last_seq + 1}, got {seq}")
                # self.last_seq = seq

                # Queue the valid payload for processing
                self.receive_queue.put(post_data)

                self.packet_times.append(time.time())  # Record packet arrival time
                
                # Update input rate periodically
                current_time = time.time()
                if current_time - self.last_rate_time > 1.0:
                    if self.packet_times:
                        recent_count = sum(1 for t in self.packet_times if current_time - t <= 3.0)
                        rate = recent_count / 3.0
                        self.rate_updated.emit(rate)
                    self.last_rate_time = current_time

            except socket.timeout:
                continue
            except Exception as e:
                print(f"Error in receive loop: {e}")
                conn = None

        if conn:
            conn.close()
        server_socket.close()
        print("Exited server loop.")
        self.csvfile.close()
        print("CSV file closed.")

    def process_batches(self):
        """Separate thread to batch-process queued packets at a fixed FPS-like rate.
        Dequeues all available every fixed period (1/FPS sec), processes to buffer,
        then uses token-bucket to emit smoothed subsets based on estimated rate from timestamps."""
        interval_sec = 1.0 / PROCESS_FPS  # Period in seconds
        next_time = time.perf_counter() + interval_sec  # Schedule first tick

        credits = 0.0  # Accumulator for token-bucket emits
        while self.running:
            batch_rows = []

            # Dequeue all available raw packets (non-blocking, thread-safe)
            while not self.receive_queue.empty():
                post_data = self.receive_queue.get()

                # Unpack the binary data (same as before)
                expected_len = 1 + self.num_sensors * 12  # 1 byte ID + sensors * (4 ts + 4 raw1 + 4 raw2)
                if len(post_data) != expected_len:
                    self.status_signal.emit(f"Invalid data packet of len:{len(post_data)}. Expected len:{expected_len}")
                    continue

                fmt = '<B' + 'Iii' * self.num_sensors  # Little-endian: uint8, then per sensor: uint32 ts_us, int32 raw1, int32 raw2
                try:
                    unpacked = struct.unpack(fmt, post_data)
                    probe_id = unpacked[0]
                    if probe_id != 1:  # Assuming NANO_ID="01" -> atoi=1; adjust if different
                        self.status_signal.emit(f"Unexpected Probe ID: {probe_id}")
                        continue
                    
                    times_us = unpacked[1::3]
                    raws1 = unpacked[2::3]
                    raws2 = unpacked[3::3]
                    
                    times = [ts / 1000000.0 for ts in times_us]
                    volts1 = [r * VOLTS_PER_LSB for r in raws1]
                    volts2 = [r * VOLTS_PER_LSB for r in raws2]
                except (ValueError, struct.error):
                    self.status_signal.emit("Cannot unpack binary data")
                    continue

                # Collect first time for rate estimation
                self.collected_first_times.append(times[0])

                # Build CSV row (write all at tick end)
                now = datetime.datetime.now()
                row = []
                for j in range(self.num_sensors):
                    row += [f"{times[j]:.6f}", f"{raws1[j]:+08d}", f"{raws2[j]:+08d}"]
                row += [now.time()]
                batch_rows.append(row)

                # Build and buffer emit list for this packet
                emit_list = []
                for j in range(self.num_sensors):
                    emit_list += [times[j], volts1[j], volts2[j]]
                self.processed_buffer.append(emit_list)

            # Write any new CSV rows (all at once per tick)
            if batch_rows:
                self.csvwriter.writerows(batch_rows)
                self.csvfile.flush()

            # Periodic rate update (every ~1s, if new data)
            if time.perf_counter() - self.last_rate_update > 1.0 and len(self.collected_first_times) > 10:  # Enough for stable avg
                deltas = [self.collected_first_times[i+1] - self.collected_first_times[i] for i in range(len(self.collected_first_times)-1)]
                avg_delta = sum(deltas) / len(deltas) if deltas else 0.1
                self.rate_estimate = 1.0 / avg_delta if avg_delta > 0 else 10.0
                self.collected_first_times = self.collected_first_times[-10:]  # Keep recent for drift adaptation
                self.last_rate_update = time.perf_counter()
                print(f"Updated rate estimate: {self.rate_estimate:.2f} Hz")  # Debug; remove if needed

            # Accumulate credits and decide how many packets to emit
            credits += self.rate_estimate * interval_sec  # E.g., 10 * 0.033 ≈ 0.33
            to_emit = int(credits)
            credits -= to_emit

            if to_emit > 0:
                # Emit up to to_emit packets from buffer (or all if fewer)
                batch_emit_lists = []
                actual_emitted = min(to_emit, len(self.processed_buffer))
                for _ in range(actual_emitted):
                    batch_emit_lists.append(self.processed_buffer.popleft())

                if batch_emit_lists:
                    # Flatten and emit
                    flat_batch_emit = [item for sublist in batch_emit_lists for item in sublist]
                    self.data_ready.emit(flat_batch_emit)

            # Sleep until next fixed tick (precise timing for smooth FPS-like rate)
            sleep_duration = max(0, next_time - time.perf_counter())
            if sleep_duration == 0:
                print("Processing overrun; skipping sleep to catch up.")  # Debug; remove if not needed
            time.sleep(sleep_duration)
            next_time += interval_sec

class RealTimePlotWindow(QtWidgets.QMainWindow):
    """
    Class to handle real-time plotting of strain, force, and position. 
    Does not create or write to or know about CSVs.
    Now handles batched data emits (multiple packets at once).
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

        # Load calibration coefficients - pandas.read_csv is cross-platform for file reading
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

        # Performance optimizations for high refresh rates - pyqtgraph config is cross-platform via PyQt5
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

        # Set up timer for fixed-rate plot updates - QTimer is cross-platform for timed updates
        print(f"\t\tStarting plot refresh timer...")
        self.plot_timer = QtCore.QTimer()
        self.plot_timer.timeout.connect(self.update_plots)
        self.plot_timer.start(int(1000 / PLOT_REFRESH_HZ))  # Interval in ms
        print(f"\tSuccessfully created plot windows.")

    def handle_data(self, data_list):
        """Handle emitted data: compute force/position, append to deques.
        Now handles batched data (multiple packets flattened)."""
        if len(data_list) % (self.num_sensors * 3) != 0:
            print("Invalid batched data list length received for plotting.")
            return

        num_packets = len(data_list) // (self.num_sensors * 3)
        for p in range(num_packets):
            offset = p * (self.num_sensors * 3)
            times = [data_list[offset + j * 3] for j in range(self.num_sensors)]
            strains1 = [data_list[offset + j * 3 + 1] for j in range(self.num_sensors)]
            strains2 = [data_list[offset + j * 3 + 2] for j in range(self.num_sensors)]

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
            if i == 1:
                continue
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
        # self.ReadWrite.wait()  # Wait for thread to finish

def run_collection(save_format='raw', plot=True, sensors='A', header_content=None):  # header_content now optional (GUI provides it)
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

    ReadWrite = DataReceiverWriter(save_format, num_sensors, sensor_labels, header_content)  # Pass header_content directly
    if plot:
        app = QtWidgets.QApplication([])  # QApplication is cross-platform for GUI/plotting
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
    parser.add_argument('--sensors', default='A,B,C,D,E', help="Number of sensors (1-5) or comma-separated labels (e.g., 'A,C,E'). Note: Data must arrive in the specified order; configure Arduino accordingly for non-sequential labels.")
    args = parser.parse_args()

    # For standalone: Use example header_content (GUI will override with dynamic list)
    example_header_content = [
        "Note: Thow-away trial run. Not real data",
        "Test Type: Demo",
        f"Number of ICB-Sensors: 5, Sensor Label(s): {args.sensors}",
        "ICB-Sensor A's Serial#: 001, Length: 120mm, Spacing: 40mm, Saturation Load: 80N, Factor of Safety at Saturation: 1.5",
        "ICB-Sensor A's Latest Calibration: N/A, kA1: 1.0, dA1: 1.0, cA1: 1.0, kA2: 1.0, dA2: 1.0, cA2: 1.0",
        "ICB-Sensor B's Serial#: 002, Length: 120mm, Spacing: 40mm, Saturation Load: 80N, Factor of Safety at Saturation: 1.5",
        "ICB-Sensor B's Latest Calibration: N/A, kB1: 1.0, dB1: 1.0, cB1: 1.0, kB2: 1.0, dB2: 1.0, cB2: 1.0",
        "ICB-Sensor C's Serial#: 003, Length: 120mm, Spacing: 40mm, Saturation Load: 80N, Factor of Safety at Saturation: 1.5",
        "ICB-Sensor C's Latest Calibration: N/A, kC1: 1.0, dC1: 1.0, cC1: 1.0, kC2: 1.0, dC2: 1.0, cC2: 1.0",
        "ICB-Sensor D's Serial#: 004, Length: 120mm, Spacing: 40mm, Saturation Load: 80N, Factor of Safety at Saturation: 1.5",
        "ICB-Sensor D's Latest Calibration: N/A, kD1: 1.0, dD1: 1.0, cD1: 1.0, kD2: 1.0, dD2: 1.0, cD2: 1.0",
        "ICB-Sensor E's Serial#: 005, Length: 120mm, Spacing: 40mm, Saturation Load: 80N, Factor of Safety at Saturation: 1.5",
        "ICB-Sensor E's Latest Calibration: N/A, kE1: 1.0, dE1: 1.0, cE1: 1.0, kE2: 1.0, dE2: 1.0, cE2: 1.0",
        "Analog-to-Digital Converter: ADS1220, Mode: Turbo, Data Rate: DR_90SPS, Analog Excitation/Reference Voltage: 5.1V +/-2mV",
        "DAQ Microcontroller: Arduino Nano ESP32, ID: Hi-STIFFS_Nano, CPU Clock: 240MHz, Cores: 2, Data-stream Connection: Wi-Fi"
    ]
    run_collection(args.save_format, args.plot, args.sensors, example_header_content)