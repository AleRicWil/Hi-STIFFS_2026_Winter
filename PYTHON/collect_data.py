# Standard libraries
import csv
import collections
import datetime
import time
import socket
import struct
import argparse
import queue  # For thread-safe queues
import threading  # For separate processing thread
import platform
import os

# Installed packages
import numpy as np
import pyqtgraph as pg
from PyQt5 import QtWidgets, QtCore

# Workspace scripts
from sensor_registry import SensorRegistry  # Import the SensorRegistry class
from config import Config

# === ADS1220 parameters ===
ADS1220_BITS = 23  # True resolution in differential mode (signed)
TWO_TO_23 = 1 << ADS1220_BITS  # 8388608
ADS1220_PGA_GAIN = 128  # Configured gain
VREF = 5.1  # External reference voltage (AVDD)
VOLTS_PER_LSB = VREF / (ADS1220_PGA_GAIN * TWO_TO_23)

# === Plotting parameters ===
PLOT_REFRESH_HZ = 30  # Refresh rate for plot updates in Hz
PROCESS_FPS = 30  # Set desired FPS for processing/dequeuing (e.g., 30 for smooth plotting)
SCREEN_SCALE = 1.3
SCREEN_WIDTH = int(1920*SCREEN_SCALE)
SCREEN_HEIGHT = int(1080*SCREEN_SCALE)


class DataReceiverWriter(QtCore.QThread):
    """Thread for hosting a TCP server to receive WiFi data from Nano, processing to volts, writing to CSV, and emitting signals to other threads.
    Now uses persistent TCP socket instead of HTTP. Receives length-prefixed binary frames.
    Adds a queue for received packets, which are batched and processed in a separate thread."""

    data_ready = QtCore.pyqtSignal(list)  # Emits flat list [time_0, strain_01_v, strain_02_v, time_1, strain_11_v, strain_12_v, ...] (batched)
    status_signal = QtCore.pyqtSignal(str)  # For status messages
    rate_updated = QtCore.pyqtSignal(float)  # Emits updated input rate in Hz

    def __init__(self, num_sensors, sensor_labels=['A', 'B', 'C', 'D', 'E'], sensor_sns=None, 
                 header_content=None, registry=None):
        super().__init__()
        
        self.host_ip = Config.HOST_IP
        self.host_port = Config.HOST_PORT

        probe_num = '01'
        print(f"Datastream from Nano {probe_num} status:")
        self.num_sensors = num_sensors
        self.sensor_labels = sensor_labels
        if sensor_sns is None:
            sensor_sns = ['unknown'] * self.num_sensors
        if len(sensor_sns) != self.num_sensors:
            raise ValueError("Length of sensor_sns must match num_sensors/sensor_labels")
        self.sensor_sns = sensor_sns
        self.running = True
        self.packet_times = collections.deque(maxlen=10000)  # Timestamps of received packets
        self.last_rate_time = time.time()
        self.first_packet_time = None

        print(self.sensor_labels)
        print(self.sensor_sns)

        # Create CSV file - os.makedirs and os.path.join ensure cross-platform directory creation and path compatibility
 
        self.csv_path, date_str, time_str = Config.get_timestamped_filename("01")
        self.csvfile = open(self.csv_path, 'w', newline='')
        self.csvwriter = csv.writer(self.csvfile)
        print(f"Created CSV for Nano_{probe_num} at:\t{self.csv_path}")

        print(f"Writing metadata to CSV...")
        self.csvwriter.writerow(['===BEGIN_METADATA==='])
        # Handle header_content: If None (e.g., standalone run), use minimal defaults; GUI will provide full list
        if header_content is None:
            header_content = []  # Empty default; add minimal required for standalone
            for i, (l, sn) in enumerate(zip(self.sensor_labels, self.sensor_sns)):
                header_content += [f"ICB-Sensor {l}'s Serial#: {sn}, Length: N/A, Spacing: N/A, Saturation Load: N/A, Factor of Safety at Saturation: N/A"]
            
        header_content.insert(0, f'Test Name: {date_str}_{time_str}_{probe_num}, ' + 'yyyy-mm-dd_hhmmss_{probe_num}')

        # Load calibrations from current_calibrations.csv (cross-platform path handling via os.path)
        self.registry = registry or SensorRegistry()
        if not self.registry.label_to_sn:  # auto-set from passed lists if GUI didn't
            self.registry.set_mapping(dict(zip(self.sensor_labels, self.sensor_sns)))

        self.calibrations = {}
        for l, sn in zip(self.sensor_labels, self.sensor_sns):
            cal = self.registry.get_coeffs(l)          
            self.calibrations[l] = cal
            cal_line = (f"ICB-Sensor {l}'s Latest Calibration: {cal['datetime']}, "
                        f"k{l}1: {cal['k1']}, d{l}1: {cal['d1']}, c{l}1: {cal['c1']}, "
                        f"k{l}2: {cal['k2']}, d{l}2: {cal['d2']}, c{l}2: {cal['c2']}")
            header_content.append(cal_line)   # keep for CSV metadata
        
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
        server_socket.bind((self.host_ip, self.host_port))
        server_socket.listen(1)  # Listen for one connection (persistent from Nano)
        server_socket.settimeout(0.1)  # Non-blocking with timeout for loop control

        print(f"TCP server listening at {self.host_ip}:{self.host_port}")
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
                if self.first_packet_time is None:
                    self.first_packet_time = time.time()
                    print(f"Received first packet")

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
                    emit_list += [times[j], raws1[j], raws2[j]]
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
    
    New feature: `show_raw_strains` flag (default=False) controls whether the secondary raw strain plots window is created.
    - Default (False): Displays ONLY the Force and Position live plot window (lower CPU/GPU/memory usage).
    - True: Displays Force/Position + Raw Strain windows (matches previous behavior).
    - The existing `plot=False` path in run_collection() already provides the third option (no live plots at all).
    """
    def __init__(self, readwrite, num_sensors, sensor_labels, show_raw_strains=False):
        super().__init__()

        print(f"\n\tPlot windows status:")
        self.num_sensors = num_sensors
        self.sensor_labels = sensor_labels
        self.show_raw_strains = show_raw_strains  # New flag – controls raw strain window & related data handling

        # === Device-specific performance tuning (cross-platform) ===
        self.is_pi5 = self._detect_raspberry_pi5()
        if self.is_pi5:
            self.plot_refresh_hz = 4.0          # smoother on Pi5 touchscreen
            self.maxlen = 4 * 300                  # smaller buffer = less memory/CPU
            pg.setConfigOptions(useOpenGL=False)  # OpenGL is unreliable on Pi5 display stack
            print("Detected Raspberry Pi 5 — using Pi5-optimized plot settings")
        else:
            self.plot_refresh_hz = PLOT_REFRESH_HZ
            self.maxlen = 15 * 300 # 15s * 300sps
            # OpenGL stays enabled (default behavior on Windows/Ubuntu)
            print("Detected laptop/desktop — using full-performance plot settings")
        # =============================================================

        self.ReadWrite = readwrite
        self.ReadWrite.data_ready.connect(self.handle_data)
        self.ReadWrite.status_signal.connect(print)
        self.ReadWrite.rate_updated.connect(lambda rate: self.rate_label.setText(f"Input Rate: {rate:.1f} Hz"))

        # Calibration coefficients
        cal = self.ReadWrite.calibrations
        self.k1 = [cal[l]['k1'] for l in sensor_labels]
        self.d1 = [cal[l]['d1'] for l in sensor_labels]
        self.c1 = [cal[l]['c1'] for l in sensor_labels]
        self.k2 = [cal[l]['k2'] for l in sensor_labels]
        self.d2 = [cal[l]['d2'] for l in sensor_labels]
        self.c2 = [cal[l]['c2'] for l in sensor_labels]

        self.calibration_active = [True] * self.num_sensors
        self.cal_start_time = [None] * self.num_sensors
        self.initial_strains1 = [[] for _ in range(self.num_sensors)]
        self.initial_strains2 = [[] for _ in range(self.num_sensors)]
        self.cal_duration = 1.0
        self.min_samples = 150

        pg.setConfigOptions(useOpenGL=True, antialias=False)

        # === Main Window: Force and Position Plots (ALWAYS created when class is instantiated) ===
        self.setWindowTitle("Force and Position - Real-Time Monitoring")
        
        self.win_force_pos = pg.GraphicsLayoutWidget()
        self.plot_force = self.win_force_pos.addPlot(title='Force')
        self.plot_force.setLabel('left', '', units='N')
        self.plot_force.addLegend()
        
        self.plot_pos = self.win_force_pos.addPlot(title='Position')
        self.plot_pos.setLabel('left', '', units='mm')
        self.plot_pos.addLegend()

        colors = ['r', 'g', 'c', 'y', 'm'][:self.num_sensors]
        
        self.curves_force = []
        self.curves_pos = []
        for i, s in enumerate(self.sensor_labels):
            curve = self.plot_force.plot(pen=colors[i], name=f'{s}')
            curve.setDownsampling(method='peak', auto=True)   
            curve.setClipToView(True)                       
            self.curves_force.append(curve)

            curve = self.plot_pos.plot(pen=colors[i], name=f'{s}')
            curve.setDownsampling(method='peak', auto=True)   
            curve.setClipToView(True)                       
            self.curves_pos.append(curve)

        # Main layout with control bar
        main_layout = QtWidgets.QVBoxLayout()
        main_layout.addWidget(self.win_force_pos)

        preset_layout = QtWidgets.QHBoxLayout()
        preset_label = QtWidgets.QLabel("Time Range (s):")
        preset_layout.addWidget(preset_label)
        
        presets = [1, 3, 5, 10, 15, 30]
        for preset in presets:
            btn = QtWidgets.QPushButton(str(preset))
            btn.clicked.connect(lambda _, p=preset: self.set_time_range(p))
            preset_layout.addWidget(btn)

        # Rescale Y button
        rescale_btn = QtWidgets.QPushButton("Rescale Y")
        rescale_btn.clicked.connect(self.rescale_y_axes)
        preset_layout.addWidget(rescale_btn)

        # STOP button — identical behavior to CollectPage Stop button
        # (closes both plot windows, stops DataReceiverWriter, flushes CSV, etc.)
        stop_btn = QtWidgets.QPushButton("STOP")
        stop_btn.setStyleSheet("QPushButton { background-color: #c42; color: white; font-weight: bold; padding: 4px 12px; }")
        stop_btn.setMinimumWidth(200)
        stop_btn.clicked.connect(self.stop_collection)   # re-uses existing method → zero duplication
        preset_layout.addWidget(stop_btn)

        preset_layout.addStretch()
        self.rate_label = QtWidgets.QLabel("Input Rate: 0 Hz")
        preset_layout.addWidget(self.rate_label)
        
        main_layout.addLayout(preset_layout)

        central_widget = QtWidgets.QWidget()
        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)

        self.resize(SCREEN_WIDTH, int(SCREEN_HEIGHT * 0.68))
        self.move(0, 0)
        self.show()

        # === Optional Secondary Window: Raw Strain Plots (only created if flag is True) ===
        self.win_strain = None
        self.curves_ch1 = None
        self.curves_ch2 = None
        self.plot_ch1 = None
        self.plot_ch2 = None
        if self.show_raw_strains:
            print(f"\t\tCreating raw strain plots (secondary window)...")
            self.win_strain = pg.GraphicsLayoutWidget()
            self.win_strain.setWindowTitle("Raw Strain Data (ADC Counts)")
            
            self.plot_ch1 = self.win_strain.addPlot(title='Channel 1 Strains')
            self.plot_ch1.setLabel('left', '', units='ADC Counts')
            self.plot_ch1.addLegend()
            
            self.plot_ch2 = self.win_strain.addPlot(title='Channel 2 Strains')
            self.plot_ch2.setLabel('left', '', units='ADC Counts')
            self.plot_ch2.addLegend()

            self.curves_ch1 = []
            self.curves_ch2 = []
            for i, s in enumerate(self.sensor_labels):
                curve = self.plot_ch1.plot(pen=colors[i], name=f'{s}1')
                curve.setDownsampling(method='peak', auto=True)   
                curve.setClipToView(True)                       
                self.curves_ch1.append(curve)

                curve = self.plot_ch2.plot(pen=colors[i], name=f'{s}2')
                curve.setDownsampling(method='peak', auto=True)   
                curve.setClipToView(True)                       
                self.curves_ch2.append(curve)

            self.win_strain.resize(SCREEN_WIDTH, int(SCREEN_HEIGHT * 0.32))
            self.win_strain.move(0, int(SCREEN_HEIGHT * 0.69))
            self.win_strain.show()

            # Support space key on secondary window (cross-platform Qt event handling)
            self.win_strain.installEventFilter(self)
        else:
            print("\t\tRaw strain plots DISABLED (performance optimization)")

        # Data buffers – strains deques only allocated when flag is True (saves memory/CPU)
        maxlen = self.maxlen
        self.times = [collections.deque(maxlen=maxlen) for _ in range(self.num_sensors)]
        self.forces = [collections.deque(maxlen=maxlen) for _ in range(self.num_sensors)]
        self.positions = [collections.deque(maxlen=maxlen) for _ in range(self.num_sensors)]
        if self.show_raw_strains:
            self.strains1 = [collections.deque(maxlen=maxlen) for _ in range(self.num_sensors)]
            self.strains2 = [collections.deque(maxlen=maxlen) for _ in range(self.num_sensors)]
        else:
            self.strains1 = None
            self.strains2 = None

        # Plot refresh timer (always runs for Force/Position; strain updates are conditional inside update_plots)
        self.plot_timer = QtCore.QTimer()
        self.plot_timer.timeout.connect(self.update_plots)
        self.plot_timer.start(int(1000 / self.plot_refresh_hz))

        self.display_time_range = 10.0        
        self.initial_rescale_done = False
        self.last_y_rescale_time = 0.0          
        self.rescale_interval = 1.0             

        self.display_start_time = 1.1   # first 1.1 s of data will never be plotted or used for y-scaling

        print(f"\tSuccessfully created plot windows (Force/Position main window{' + Raw Strains secondary' if self.show_raw_strains else ''}).")

    def _detect_raspberry_pi5(self):
        """Pure-Python runtime detection. Returns True only on Raspberry Pi 5 (or any Pi).
        On Windows 10/11 or Ubuntu it returns False instantly with zero overhead.
        Cross-platform safe — uses standard library + optional /proc read."""
        if platform.system() != "Linux":
            return False
        try:
            # Most reliable method for Pi5 (works on all Pi models)
            with open("/proc/device-tree/model", "r", encoding="ascii") as f:
                return "Raspberry Pi" in f.read()
        except (OSError, FileNotFoundError):
            # Fallback for older Pi or non-Pi Linux
            try:
                with open("/proc/cpuinfo", "r") as f:
                    return "Raspberry Pi" in f.read()
            except:
                return False

    def handle_data(self, data_list):
        """Handle emitted data: compute force/position, append to deques only if t >= 1.1 s.
        Early data (t < 1.1 s) is still processed for calibration but is never accumulated
        for display or used in any y-scaling decisions.
        Strain deques are only appended when show_raw_strains=True."""
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

                # Calibration logic: Collect initial data per sensor (still runs on all incoming data)
                if self.calibration_active[i]:
                    if self.cal_start_time[i] is None:
                        self.cal_start_time[i] = time.time()
                        print(f"Starting calibration for sensor {self.sensor_labels[i]}")

                    self.initial_strains1[i].append(strain1)
                    self.initial_strains2[i].append(strain2)

                    elapsed = time.time() - self.cal_start_time[i]
                    if elapsed >= self.cal_duration:
                        if len(self.initial_strains1[i]) >= self.min_samples:
                            self.c1[i] = np.mean(self.initial_strains1[i])
                            self.c2[i] = np.mean(self.initial_strains2[i])
                            print(f"Updated offsets for sensor {self.sensor_labels[i]}: c1={self.c1[i]:.2f}, c2={self.c2[i]:.2f}")
                        else:
                            print(f"Warning: Insufficient samples ({len(self.initial_strains1[i])}) for sensor {self.sensor_labels[i]}. Using pre-loaded offsets.")
                        self.calibration_active[i] = False
                        self.initial_strains1[i] = []
                        self.initial_strains2[i] = []

                # Calculate force and position (unchanged)
                try:
                    num = (self.k2[i] * (strain1 - self.c1[i]) - self.k1[i] * (strain2 - self.c2[i]))
                    den = self.k1[i] * self.k2[i] * (self.d2[i] - self.d1[i])
                    force = num / den if abs(den) > 1e-6 else 0.0
                    force = force if abs(force) <= 100.0 else 0.0
                except:
                    force = 0.0
                try:
                    num = (self.k2[i] * self.d2[i] * (strain1 - self.c1[i]) - self.k1[i] * self.d1[i] * (strain2 - self.c2[i]))
                    den = (self.k2[i] * (strain1 - self.c1[i]) - self.k1[i] * (strain2 - self.c2[i]))
                    position = num / den if den != 0 and abs(den) > 1e10 else 0.0
                    position = position if position <= 0.15 and position >= 0.03 else 0.0
                except:
                    position = 0.0

                # Only append to plotting deques after display_start_time
                # (first 1.1 s is never displayed and never affects y-scaling)
                if time_sec >= self.display_start_time:
                    self.times[i].append(time_sec)
                    self.forces[i].append(force)
                    self.positions[i].append(position * 1000)
                    if self.show_raw_strains:
                        self.strains1[i].append(strain1)
                        self.strains2[i].append(strain2)

    def update_plots(self):
        """Update all plot curves and x-ranges. Performs the one-time automatic
        y-rescale using data from t=1.1 s to t=2.1 s once t_max >= 2.1 s.
        Thereafter, continuously auto-rescales all y-axes to fit the full
        vertical range of the data currently visible in the x-window.
        Strain curve updates are skipped when show_raw_strains=False."""
        for i in range(self.num_sensors):
            if i in [1]:
                continue
            self.curves_force[i].setData(self.times[i], self.forces[i])
            self.curves_pos[i].setData(self.times[i], self.positions[i])
            if self.show_raw_strains:
                self.curves_ch1[i].setData(self.times[i], self.strains1[i])
                self.curves_ch2[i].setData(self.times[i], self.strains2[i])

        t_max = 0
        for t in self.times:
            if t:
                t_max = max(t_max, t[-1])
        x_min = max(0, t_max - self.display_time_range)
        x_max = t_max

        for plot in (self.plot_force, self.plot_pos):
            if plot is not None:
                plot.setXRange(x_min, x_max)
        if self.show_raw_strains:
            for plot in (self.plot_ch1, self.plot_ch2):
                if plot is not None:
                    plot.setXRange(x_min, x_max)

        # Automatic one-time initial y-rescale on fixed early window
        if not self.initial_rescale_done and t_max >= 2.1:
            self._perform_initial_y_rescale()
            self.initial_rescale_done = True
            self.last_y_rescale_time = time.time()  # start the throttle clock
        # Continuous auto-rescale y to fit currently visible data (after initial)
        elif self.initial_rescale_done:
            now = time.time()
            if now - self.last_y_rescale_time >= self.rescale_interval:
                plots = [self.plot_force, self.plot_pos]
                if self.show_raw_strains:
                    plots.extend([self.plot_ch1, self.plot_ch2])
                for plot in plots:
                    if plot is not None:
                        plot.enableAutoRange(x=False, y=True)
                self.last_y_rescale_time = now

    def set_time_range(self, value):
        """Set the display time range based on button preset."""
        self.display_time_range = float(value)
        self.update_plots()

    def rescale_y_axes(self):
        """Rescale all y-axes to fit only the data currently being displayed in the visible time window."""
        plots_to_rescale = [self.plot_force, self.plot_pos]
        if self.show_raw_strains:
            plots_to_rescale.extend([self.plot_ch1, self.plot_ch2])
        for plot in plots_to_rescale:
            if plot is not None:
                plot.getViewBox().setAutoVisible(y=True)
                plot.autoRange()

    def _perform_initial_y_rescale(self):
        """One-time automatic rescaling of all y-axes based solely on data in [1.1, 2.1] seconds."""
        window_start = 1.1
        window_end = 2.1

        plot_ydata_pairs = [
            (self.plot_force, self.forces),
            (self.plot_pos, self.positions),
        ]
        if self.show_raw_strains:
            plot_ydata_pairs.extend([
                (self.plot_ch1, self.strains1),
                (self.plot_ch2, self.strains2)
            ])

        for plot, y_data_lists in plot_ydata_pairs:
            if plot is None:
                continue

            y_values_in_window = []
            for i in range(self.num_sensors):
                t_deque = self.times[i]
                y_deque = y_data_lists[i]
                for t_val, y_val in zip(t_deque, y_deque):
                    if window_start <= t_val <= window_end:
                        y_values_in_window.append(y_val)

            if y_values_in_window:
                y_min = min(y_values_in_window)
                y_max = max(y_values_in_window)
                span = y_max - y_min
                padding = max(0.05 * span, abs(y_max) * 0.02) if span > 0 else 1.0
                plot.setYRange(y_min - padding, y_max + padding)
            else:
                plot.autoRange()

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
        """Stop data collection and clean up both windows (conditional on show_raw_strains)."""
        print("Keyboard 'space' was pressed. Exiting datastream loop...")
        self.close()                    # Close main window (Force/Position)
        if self.win_strain is not None:
            self.win_strain.close()     # Close secondary strain window if it exists
        print('Plots closed.')
        self.plot_timer.stop()
        self.ReadWrite.running = False


def run_collection(save_format='raw', plot=True, sensors='A', header_content=None, sensor_sns='001'):  # header_content now optional (GUI provides it)
    print('Starting collection function...')
    if sensors.isdigit():
        num = int(sensors)
        if num < 1 or num > 5:
            raise ValueError("Number of sensors must be between 1 and 5.")
        sensor_labels = [chr(65 + i) for i in range(num)]
    else:
        sensor_labels = sorted(set(s.strip().upper() for s in sensors.split(' ')), key='ABCDE'.index)
        if not sensor_labels or any(s not in 'ABCDE' for s in sensor_labels):
            raise ValueError("Invalid sensor labels; must be comma-separated from A-E, no duplicates.")
    num_sensors = len(sensor_labels)
    print('input', sensor_sns)
    sensor_sns_list = [s.strip() for s in sensor_sns.split(',')]
    print('parsed', sensor_sns_list)

    ReadWrite = DataReceiverWriter(num_sensors, sensor_labels, sensor_sns_list, header_content)  # Pass header_content directly
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
        print("=== Press Ctrl+C to end data collection (cross-platform safe on Windows, Ubuntu, and RPi5) ===")
        try:
            while ReadWrite.running:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("KeyboardInterrupt received. Exiting datastream loop...")
            ReadWrite.running = False

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Data collection from WiFi stream")
    parser.add_argument('--save-format', choices=['volts', 'raw'], default='raw', help="Format to save strains in CSV: volts or raw")
    parser.add_argument('--plot', type=bool, default=True, help="Enable live plotting")
    parser.add_argument('--sensors', default='A B C D E', help="Number of sensors (1-5) or space-separated labels (e.g., 'A C E'). Note: Data must arrive in the specified order; configure Arduino accordingly for non-sequential labels.")
    parser.add_argument('--sensor-sns', default='001,002,003,004,005', help="Comma-separated serial numbers for sensors (e.g., '001,002,003'). Must match number of sensors specified in --sensors. Used for metadata and calibration lookup.")
    args = parser.parse_args()

    # For standalone: Use example header_content (GUI will override with dynamic list)
    example_header_content = [
        "Note: Dummy code tests",
        "Test Type: Medium Lab w/o tops",
        "Speed: 50 ft/min,Stalk Spacing: 6in,Probe Height (m): 0.785",
        f"Number of ICB-Sensors: 5, Sensor Label(s): {args.sensors}, Sensor SNs: 001 002 003 004 005",
        "Analog-to-Digital Converter: ADS1220, Mode: Turbo, Data Rate: DR_330SPS, Analog Excitation/Reference Voltage: 5.1V +/-2mV",
        "DAQ Microcontroller: Arduino Nano ESP32, ID: Hi-STIFFS_Nano, CPU Clock: 240MHz, Cores: 2, Data-stream Connection: Wi-Fi"
    ]
    run_collection(args.save_format, args.plot, args.sensors, example_header_content, sensor_sns=args.sensor_sns)