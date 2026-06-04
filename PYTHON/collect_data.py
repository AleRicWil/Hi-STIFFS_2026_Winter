#!/usr/bin/env python3
"""
collect_data.py - Multi-probe WiFi data collection for Hi-STIFFS.

New architecture (June 2026):
- WiFiDataServer: single shared TCP listener + connection manager. One instance for the whole process.
- DataReceiverWriter: lightweight per-probe QThread. Owns CSV, calibrations, processing, signals, and optional plot.
- The first DataReceiverWriter created automatically instantiates the shared WiFiDataServer.
- All subsequent DataReceiverWriter instances register with the existing server instance.
- Data is demuxed by the nano_id byte sent by each Arduino.

This cleanly separates the network layer from per-probe logic while preserving 100% of the previous functionality
(binary protocol, NavX2 support, CRC, auto-calibration, plotting with Pi5 optimizations, etc.).

Cross-platform: The entire file uses only PyQt5, stdlib (socket, threading, queue, struct, csv, datetime, pathlib via Config),
and the existing platform detection in RealTimePlotWindow. No changes required between Windows 10/11, Ubuntu, and Raspberry Pi 5 touchscreen.

The GUI remains the single human access point and can now easily create one DataReceiverWriter per configured nano_id.
"""

# Standard libraries
import csv
import collections
import datetime
import time
import socket
import struct
import argparse
import queue
import threading
import platform
import os

# Installed packages
import numpy as np
import pyqtgraph as pg
from PyQt5 import QtWidgets, QtCore

# Workspace scripts
from sensor_registry import SensorRegistry
from config import Config

# === ADS1220 parameters ===
ADS1220_BITS = 23
TWO_TO_23 = 1 << ADS1220_BITS
ADS1220_PGA_GAIN = 128
VREF = 5.1
VOLTS_PER_LSB = VREF / (ADS1220_PGA_GAIN * TWO_TO_23)

# === Plotting parameters ===
PLOT_REFRESH_HZ = 30
PROCESS_FPS = 30
SCREEN_SCALE = 1.3
SCREEN_WIDTH = int(1920 * SCREEN_SCALE)
SCREEN_HEIGHT = int(1080 * SCREEN_SCALE)

# === NavX2 packet schema ===
NAVX2_BINARY_FMT = '<BIIhhhHHhhhhhhhiiiHHBB'
NAVX2_BINARY_BYTES = struct.calcsize(NAVX2_BINARY_FMT)
NAVX2_CSV_HEADERS = [
    'NavX2_Flags_raw', 'NavX2_Valid', 'NavX2_Fresh', 'NavX2_Timestamp_us',
    'NavX2_NavX_Timestamp_ms', 'NavX2_Yaw_deg', 'NavX2_Pitch_deg', 'NavX2_Roll_deg',
    'NavX2_Heading_deg', 'NavX2_Fused_Heading_deg',
    'NavX2_Accel_X_g', 'NavX2_Accel_Y_g', 'NavX2_Accel_Z_g',
    'NavX2_Quat_W', 'NavX2_Quat_X', 'NavX2_Quat_Y', 'NavX2_Quat_Z',
    'NavX2_Disp_X_m', 'NavX2_Disp_Y_m', 'NavX2_Disp_Z_m',
    'NavX2_Sensor_Status', 'NavX2_Capability_Flags', 'NavX2_Cal_Status', 'NavX2_Selftest_Status',
]


class WiFiDataServer(QtCore.QObject):
    """
    Shared persistent TCP server for all Hi-STIFFS probes.
    One instance per process. Accepts multiple Nano connections on the same port.
    Routes incoming binary frames to the correct per-probe DataReceiverWriter based on nano_id.
    
    Cross-platform: uses only stdlib socket + threading. Works identically on all supported OSes.
    """

    def __init__(self, host_ip=None, host_port=None):
        super().__init__()
        self.host_ip = host_ip or Config.HOST_IP
        self.host_port = host_port or Config.HOST_PORT
        self.probe_handlers = {}          # nano_id (int) -> DataReceiverWriter instance
        self.server_socket = None
        self.running = False
        self.accept_thread = None
        self.client_threads = []          # keep references for clean shutdown

    def register_probe(self, handler):
        """Register a per-probe DataReceiverWriter so the server can route packets to it."""
        self.probe_handlers[handler.nano_id] = handler
        print(f'[WiFiDataServer] Registered probe "Nano_{handler.nano_id:02d}"')

    def unregister_probe(self, nano_id):
        self.probe_handlers.pop(nano_id, None)
        print(f'[WiFiDataServer] Unregistered probe "Nano_{nano_id:02d}"')

    def start(self):
        if self.running:
            return
        self.running = True
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host_ip, self.host_port))
        self.server_socket.listen(5)
        self.server_socket.settimeout(0.5)
        self.accept_thread = threading.Thread(target=self._accept_loop, daemon=True)
        self.accept_thread.start()
        print(f"[WiFiDataServer] Listening on {self.host_ip}:{self.host_port} (shared for all probes)")

    def stop(self):
        self.running = False
        if self.server_socket:
            try:
                self.server_socket.close()
            except:
                pass
        if self.accept_thread:
            self.accept_thread.join(timeout=2)
        for t in self.client_threads:
            t.join(timeout=1)
        print("[WiFiDataServer] Stopped")

    def _accept_loop(self):
        while self.running:
            try:
                conn, addr = self.server_socket.accept()
                print(f"[WiFiDataServer] Accepted connection from {addr}")
                t = threading.Thread(target=self._client_loop, args=(conn, addr), daemon=True)
                t.start()
                self.client_threads.append(t)
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    print(f"[WiFiDataServer] Accept error: {e}")
                break

    def _read_fully(self, conn, size):
        data = b''
        while len(data) < size:
            chunk = conn.recv(size - len(data))
            if not chunk:
                raise EOFError("Connection closed")
            data += chunk
        return data

    def _client_loop(self, conn, addr):
        while self.running:
            try:
                header = self._read_fully(conn, 6)
                length = struct.unpack('<H', header[:2])[0]
                seq = struct.unpack('<H', header[2:4])[0]
                received_crc = struct.unpack('<H', header[4:6])[0]

                post_data = self._read_fully(conn, length)
                if len(post_data) != length:
                    continue

                computed_crc = self._crc16_ccitt(post_data)
                if computed_crc != received_crc:
                    continue

                # Route by nano_id (first byte of the sensor payload)
                if len(post_data) > 0:
                    nano_id = post_data[0]
                    handler = self.probe_handlers.get(nano_id)
                    if handler is not None:
                        handler.receive_queue.put(post_data)
                    # else:
                    #     print(f"[WiFiDataServer] No handler registered for Nano_{nano_id:02d} — dropping packet")
            except (EOFError, ConnectionResetError, socket.timeout):
                break
            except Exception as e:
                if self.running:
                    print(f"[WiFiDataServer] Client {addr} error: {e}")
                break
        try:
            conn.close()
        except:
            pass
        print(f"[WiFiDataServer] Connection from {addr} closed")

    def _crc16_ccitt(self, data):
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 1:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc


class DataReceiverWriter(QtCore.QThread):
    """
    Per-probe data handler.
    One instance per Nano ID. Owns its CSV, calibrations, processing thread, and signals.
    Registers with the shared WiFiDataServer (first instance creates the server).
    """

    data_ready = QtCore.pyqtSignal(list)
    status_signal = QtCore.pyqtSignal(str)
    rate_updated = QtCore.pyqtSignal(float)

    _shared_server = None   # class-level shared server (created by the first DataReceiverWriter)

    def __init__(self, num_sensors, sensor_labels=['A', 'B', 'C', 'D', 'E'], sensor_sns=None,
                 header_content=None, registry=None, nano_id=1, probe_height_m=None, show_raw_strains=False):
        super().__init__()

        self.nano_id = int(nano_id)
        self.num_sensors = num_sensors
        self.sensor_labels = sensor_labels
        if sensor_sns is None:
            sensor_sns = ['unknown'] * self.num_sensors
        if len(sensor_sns) != self.num_sensors:
            raise ValueError("Length of sensor_sns must match num_sensors")
        self.sensor_sns = sensor_sns
        self.running = True
        self.show_raw_strains = show_raw_strains

        # --- Create / reuse shared server (first probe creates it) ---
        if DataReceiverWriter._shared_server is None:
            DataReceiverWriter._shared_server = WiFiDataServer()
            DataReceiverWriter._shared_server.start()
        DataReceiverWriter._shared_server.register_probe(self)

        # Sensor registry & calibrations
        self.registry = registry or SensorRegistry()
        if not self.registry.label_to_sn:
            self.registry.set_mapping(dict(zip(self.sensor_labels, self.sensor_sns)))
        self.calibrations = {l: self.registry.get_coeffs(l) for l in self.sensor_labels}

        # Per-probe CSV using shared session timestamp
        self.csv_path, date_str, time_str = Config.get_session_filename(self.nano_id)
        self.csvfile = open(self.csv_path, 'w', newline='')
        self.csvwriter = csv.writer(self.csvfile)
        print(f"[DataReceiverWriter Nano_{self.nano_id:02d}] CSV path: {self.csv_path}")

        # Write metadata and headers (same logic as before)
        self.csvwriter.writerow(['===BEGIN_METADATA==='])
        self.csvwriter.writerow([f'Test Name: {date_str}_{time_str}_{self.nano_id:02d}', 'yyyy-mm-dd_hhmmss_{probe#/nano_id}'])
        self.csvwriter.writerow([f'Probe ID: Nano_{self.nano_id:02d}', f'Probe Height (m): {probe_height_m:.4f}'])
        for row in header_content:
            self.csvwriter.writerow(row.split(', '))

        self.csvwriter.writerow([f"Number of ICB-Sensors: {self.num_sensors}", f"Sensor Label(s): {self.sensor_labels}", f"Sensor SNs: {self.sensor_sns}"])

        for l, sn in zip(self.sensor_labels, self.sensor_sns):
            cal = self.calibrations[l]
            self.csvwriter.writerow([f"ICB-Sensor {l}'s Latest Calibration: {cal['datetime']}",
                        f"k{l}1: {cal['k1']}", f"d{l}1: {cal['d1']}", f"c{l}1: {cal['c1']}",
                        f"k{l}2: {cal['k2']}", f"d{l}2: {cal['d2']}", f"c{l}2: {cal['c2']}"])   # keep for CSV metadata
                
        self.csvwriter.writerow([Config.HEADER_MARKER])
        self.csvwriter.writerow([Config.DATA_MARKER])
        
        data_headers = []
        for l in self.sensor_labels:
            data_headers += [f'Time_{l}_sec', f'Strain_{l}1_raw', f'Strain_{l}2_raw']
        data_headers += NAVX2_CSV_HEADERS + ['Processed_Time']
        self.csvwriter.writerow(data_headers)
        print(f"[DataReceiverWriter Nano_{self.nano_id:02d}] Added metadata and headers to CSV")

        # Queues and processing (per-probe)
        self.receive_queue = queue.Queue()
        self.processed_buffer = collections.deque()
        self.packet_times = collections.deque(maxlen=10000)
        self.last_rate_time = time.time()
        self.first_packet_time = None
        self.rate_estimate = 10.0
        self.last_rate_update = time.perf_counter()
        self.collected_first_times = []

        self.processing_thread = threading.Thread(target=self.process_batches, daemon=True)
        self.processing_thread.start()

        # For plotting (optional)
        self.calibration_active = [True] * self.num_sensors
        self.cal_start_time = [None] * self.num_sensors
        self.initial_strains1 = [[] for _ in range(self.num_sensors)]
        self.initial_strains2 = [[] for _ in range(self.num_sensors)]
        self.cal_duration = 1.0
        self.min_samples = 150

    def run(self):
        """QThread main loop — keeps the thread alive for Qt signals while processing happens in background thread."""
        while self.running:
            QtCore.QThread.msleep(200)
        self._cleanup()

    def _cleanup(self):
        if DataReceiverWriter._shared_server:
            DataReceiverWriter._shared_server.unregister_probe(self.nano_id)
        if self.csvfile:
            self.csvfile.close()
        print(f"[DataReceiverWriter Nano_{self.nano_id:02d}] Stopped and CSV closed.")

    def stop(self):
        self.running = False
        if DataReceiverWriter._shared_server:
            DataReceiverWriter._shared_server.unregister_probe(self.nano_id)

    def decode_navx2_payload(self, navx_payload):
        if len(navx_payload) != NAVX2_BINARY_BYTES:
            raise ValueError("Invalid NavX2 payload length")
        unpacked = struct.unpack(NAVX2_BINARY_FMT, navx_payload)
        flags = unpacked[0]
        return {
            'NavX2_Flags_raw': flags,
            'NavX2_Valid': flags & 0x01,
            'NavX2_Fresh': (flags >> 1) & 0x01,
            'NavX2_Timestamp_us': unpacked[1],
            'NavX2_NavX_Timestamp_ms': unpacked[2],
            'NavX2_Yaw_deg': unpacked[3] / 100.0,
            'NavX2_Pitch_deg': unpacked[4] / 100.0,
            'NavX2_Roll_deg': unpacked[5] / 100.0,
            'NavX2_Heading_deg': unpacked[6] / 100.0,
            'NavX2_Fused_Heading_deg': unpacked[7] / 100.0,
            'NavX2_Accel_X_g': unpacked[8] / 1000.0,
            'NavX2_Accel_Y_g': unpacked[9] / 1000.0,
            'NavX2_Accel_Z_g': unpacked[10] / 1000.0,
            'NavX2_Quat_W': unpacked[11] / 16384.0,
            'NavX2_Quat_X': unpacked[12] / 16384.0,
            'NavX2_Quat_Y': unpacked[13] / 16384.0,
            'NavX2_Quat_Z': unpacked[14] / 16384.0,
            'NavX2_Disp_X_m': unpacked[15] / 65536.0,
            'NavX2_Disp_Y_m': unpacked[16] / 65536.0,
            'NavX2_Disp_Z_m': unpacked[17] / 65536.0,
            'NavX2_Sensor_Status': unpacked[18],
            'NavX2_Capability_Flags': unpacked[19],
            'NavX2_Cal_Status': unpacked[20],
            'NavX2_Selftest_Status': unpacked[21],
        }

    def build_navx2_csv_row(self, navx_data=None):
        if navx_data is None:
            return [''] * len(NAVX2_CSV_HEADERS)
        return [navx_data[field] for field in NAVX2_CSV_HEADERS]

    def process_batches(self):
        """Per-probe processing thread (token-bucket smoothed emit at ~PROCESS_FPS)."""
        interval_sec = 1.0 / PROCESS_FPS
        next_time = time.perf_counter() + interval_sec
        credits = 0.0

        while self.running:
            batch_rows = []
            while not self.receive_queue.empty():
                post_data = self.receive_queue.get()
                legacy_len = 1 + self.num_sensors * 12
                extended_len = legacy_len + NAVX2_BINARY_BYTES
                navx_payload = None
                if len(post_data) == legacy_len:
                    sensor_payload = post_data
                elif len(post_data) == extended_len:
                    sensor_payload = post_data[:legacy_len]
                    navx_payload = post_data[legacy_len:]
                else:
                    self.status_signal.emit(f"Invalid packet length for Nano_{self.nano_id:02d}")
                    continue

                fmt = '<B' + 'Iii' * self.num_sensors
                try:
                    unpacked = struct.unpack(fmt, sensor_payload)
                    probe_id = unpacked[0]
                    if probe_id != self.nano_id:
                        continue
                    times_us = unpacked[1::3]
                    raws1 = unpacked[2::3]
                    raws2 = unpacked[3::3]
                    times = [ts / 1000000.0 for ts in times_us]
                    navx_data = self.decode_navx2_payload(navx_payload) if navx_payload is not None else None
                except Exception:
                    continue

                self.collected_first_times.append(times[0])
                now = datetime.datetime.now()
                row = []
                for j in range(self.num_sensors):
                    row += [f"{times[j]:.6f}", f"{raws1[j]:+08d}", f"{raws2[j]:+08d}"]
                row += self.build_navx2_csv_row(navx_data)
                row += [now.time()]
                batch_rows.append(row)

                emit_list = []
                for j in range(self.num_sensors):
                    emit_list += [times[j], raws1[j], raws2[j]]
                self.processed_buffer.append(emit_list)

            if batch_rows:
                self.csvwriter.writerows(batch_rows)
                self.csvfile.flush()

            # Rate estimation (same as original)
            if time.perf_counter() - self.last_rate_update > 1.0 and len(self.collected_first_times) > 10:
                deltas = [self.collected_first_times[i+1] - self.collected_first_times[i]
                          for i in range(len(self.collected_first_times)-1)]
                avg_delta = sum(deltas) / len(deltas) if deltas else 0.1
                self.rate_estimate = 1.0 / avg_delta if avg_delta > 0 else 10.0
                self.collected_first_times = self.collected_first_times[-10:]
                self.last_rate_update = time.perf_counter()

            credits += self.rate_estimate * interval_sec
            to_emit = int(credits)
            credits -= to_emit
            if to_emit > 0:
                batch_emit_lists = []
                actual = min(to_emit, len(self.processed_buffer))
                for _ in range(actual):
                    batch_emit_lists.append(self.processed_buffer.popleft())
                if batch_emit_lists:
                    flat = [item for sub in batch_emit_lists for item in sub]
                    self.data_ready.emit(flat)

            sleep_dur = max(0, next_time - time.perf_counter())
            time.sleep(sleep_dur)
            next_time += interval_sec


class RealTimePlotWindow(QtWidgets.QMainWindow):
    """
    Real-time plotting window (Force/Position + optional Raw Strain).
    Supports per-probe instances and the existing Pi5 performance tunings + show_raw_strains flag.
    (Full implementation restored from original with nano_id awareness — identical behavior.)
    """
    def __init__(self, readwrite, num_sensors, sensor_labels, show_raw_strains=False):
        super().__init__()
        self.num_sensors = num_sensors
        self.sensor_labels = sensor_labels
        self.show_raw_strains = show_raw_strains
        self.ReadWrite = readwrite
        self.ReadWrite.data_ready.connect(self.handle_data)
        self.ReadWrite.status_signal.connect(print)
        self.ReadWrite.rate_updated.connect(lambda rate: self.rate_label.setText(f"Input Rate (Nano_{readwrite.nano_id:02d}): {rate:.1f} Hz"))

        # Device-specific performance tuning (cross-platform)
        self.is_pi5 = self._detect_raspberry_pi5()
        if self.is_pi5:
            self.plot_refresh_hz = 4.0
            self.maxlen = 4 * 300
            pg.setConfigOptions(useOpenGL=False)
            print("Detected Raspberry Pi 5 — using Pi5-optimized plot settings")
        else:
            self.plot_refresh_hz = PLOT_REFRESH_HZ
            self.maxlen = 15 * 300
            print("Detected laptop/desktop — using full-performance plot settings")

        # Calibration coefficients from the per-probe handler
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

        # Main Force/Position window (always created)
        self.setWindowTitle(f"Force and Position - Nano_{readwrite.nano_id:02d}")
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

        main_layout = QtWidgets.QVBoxLayout()
        main_layout.addWidget(self.win_force_pos)

        preset_layout = QtWidgets.QHBoxLayout()
        preset_label = QtWidgets.QLabel("Time Range (s):")
        preset_layout.addWidget(preset_label)
        for preset in [1, 3, 5, 10, 15, 30]:
            btn = QtWidgets.QPushButton(str(preset))
            btn.clicked.connect(lambda _, p=preset: self.set_time_range(p))
            preset_layout.addWidget(btn)

        rescale_btn = QtWidgets.QPushButton("Rescale Y")
        rescale_btn.clicked.connect(self.rescale_y_axes)
        preset_layout.addWidget(rescale_btn)

        stop_btn = QtWidgets.QPushButton("STOP")
        stop_btn.setStyleSheet("QPushButton { background-color: #c42; color: white; font-weight: bold; padding: 4px 12px; }")
        stop_btn.setMinimumWidth(200)
        stop_btn.clicked.connect(self.stop_collection)
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

        # Optional raw strain window
        self.win_strain = None
        self.curves_ch1 = self.curves_ch2 = None
        if self.show_raw_strains:
            self.win_strain = pg.GraphicsLayoutWidget()
            self.win_strain.setWindowTitle(f"Raw Strain Data - Nano_{readwrite.nano_id:02d}")
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
            self.win_strain.installEventFilter(self)

        maxlen = self.maxlen
        self.times = [collections.deque(maxlen=maxlen) for _ in range(self.num_sensors)]
        self.forces = [collections.deque(maxlen=maxlen) for _ in range(self.num_sensors)]
        self.positions = [collections.deque(maxlen=maxlen) for _ in range(self.num_sensors)]
        if self.show_raw_strains:
            self.strains1 = [collections.deque(maxlen=maxlen) for _ in range(self.num_sensors)]
            self.strains2 = [collections.deque(maxlen=maxlen) for _ in range(self.num_sensors)]
        else:
            self.strains1 = self.strains2 = None

        self.plot_timer = QtCore.QTimer()
        self.plot_timer.timeout.connect(self.update_plots)
        self.plot_timer.start(int(1000 / self.plot_refresh_hz))

        self.display_time_range = 10.0
        self.initial_rescale_done = False
        self.last_y_rescale_time = 0.0
        self.rescale_interval = 1.0
        self.display_start_time = 1.1

    def _detect_raspberry_pi5(self):
        if platform.system() != "Linux":
            return False
        try:
            with open("/proc/device-tree/model", "r", encoding="ascii") as f:
                return "Raspberry Pi" in f.read()
        except:
            try:
                with open("/proc/cpuinfo", "r") as f:
                    return "Raspberry Pi" in f.read()
            except:
                return False

    def handle_data(self, data_list):
        if len(data_list) % (self.num_sensors * 3) != 0:
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
                if self.calibration_active[i]:
                    if self.cal_start_time[i] is None:
                        self.cal_start_time[i] = time.time()
                    self.initial_strains1[i].append(strain1)
                    self.initial_strains2[i].append(strain2)
                    if time.time() - self.cal_start_time[i] >= self.cal_duration:
                        if len(self.initial_strains1[i]) >= self.min_samples:
                            self.c1[i] = np.mean(self.initial_strains1[i])
                            self.c2[i] = np.mean(self.initial_strains2[i])
                        self.calibration_active[i] = False
                        self.initial_strains1[i] = []
                        self.initial_strains2[i] = []
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
                    position = position if 0.03 <= position <= 0.15 else 0.0
                except:
                    position = 0.0
                if time_sec >= self.display_start_time:
                    self.times[i].append(time_sec)
                    self.forces[i].append(force)
                    self.positions[i].append(position * 1000)
                    if self.show_raw_strains:
                        self.strains1[i].append(strain1)
                        self.strains2[i].append(strain2)

    def update_plots(self):
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
        if not self.initial_rescale_done and t_max >= 2.1:
            self._perform_initial_y_rescale()
            self.initial_rescale_done = True
            self.last_y_rescale_time = time.time()
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
        self.display_time_range = float(value)
        self.update_plots()

    def rescale_y_axes(self):
        plots_to_rescale = [self.plot_force, self.plot_pos]
        if self.show_raw_strains:
            plots_to_rescale.extend([self.plot_ch1, self.plot_ch2])
        for plot in plots_to_rescale:
            if plot is not None:
                plot.getViewBox().setAutoVisible(y=True)
                plot.autoRange()

    def _perform_initial_y_rescale(self):
        window_start = 1.1
        window_end = 2.1
        plot_ydata_pairs = [(self.plot_force, self.forces), (self.plot_pos, self.positions)]
        if self.show_raw_strains:
            plot_ydata_pairs.extend([(self.plot_ch1, self.strains1), (self.plot_ch2, self.strains2)])
        for plot, y_data_lists in plot_ydata_pairs:
            if plot is None:
                continue
            y_values_in_window = []
            for i in range(self.num_sensors):
                for t_val, y_val in zip(self.times[i], y_data_lists[i]):
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
        if event.type() == QtCore.QEvent.KeyPress and event.key() == QtCore.Qt.Key_Space:
            self.stop_collection()
            return True
        return super().eventFilter(obj, event)

    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key_Space:
            self.stop_collection()

    def stop_collection(self):
        print("Keyboard 'space' pressed. Exiting...")
        self.close()
        if self.win_strain is not None:
            self.win_strain.close()
        self.plot_timer.stop()
        self.ReadWrite.stop()


def run_collection(nano_id=[1], sensors=['A B C D E'], sensor_sns=['001,002,003,004,005'], 
                   probe_height_m=[None], header_content=[None], plot=True, show_raw_strains=False):
    """
    Start data collection for one or more Hi-STIFFS probes.

    Supports lists so you can start multiple nano_ids with one call:

        run_collection(
            nano_id=[1, 2],
            sensors=["A B C D E", "A B C D E"],
            sensor_sns=["001,002,003,004,005", "101,102,103,104,105"],
            plot=True
        )

    - If a parameter is a single value, it is used for every nano_id.
    - If it is a list, values are matched to the nano_id list (shorter lists are padded).
    - Always uses one shared timestamp across all CSVs.
    - Fully backward compatible with old single-probe usage.
    """
    # Normalize nano_id to list of ints
    if isinstance(nano_id, (int, str)):
        nano_ids = [int(nano_id)]
    else:
        nano_ids = [int(x) for x in nano_id]

    n = len(nano_ids)
    if n == 0:
        print("No nano_ids provided.")
        return

    # Expand single values or lists to length n
    def expand(val, default):
        if isinstance(val, (list, tuple)):
            return (list(val) + [default] * (n - len(val)))[:n]
        return [val] * n

    sensors_list    = expand(sensors, 'A B C D E')
    sensor_sns_list = expand(sensor_sns, '001,002,003,004,005')
    header_list     = expand(header_content, None)
    show_raw_list   = expand(show_raw_strains, False)

    print(f'Starting collection function for {n} probes')
    for nano_id_i, sensors_i, sensor_sns_i in zip(nano_ids, sensors_list, sensor_sns_list):
        print(f'Probe ID: {nano_id_i:02d}. Sensor Positions {sensors_i}. Sensor S#s: {sensor_sns_i}')

    # Lock shared timestamp for all probes in this session
    Config.start_new_data_session()
    print(' ')

    handlers = []
    windows = []
    app = QtWidgets.QApplication([])

    for i, nid in enumerate(nano_ids):
        sens_str = sensors_list[i]

        # Parse sensors string for this probe
        if str(sens_str).strip().isdigit():
            num = int(sens_str)
            if num < 1 or num > 5:
                num = 5
            sensor_labels = [chr(65 + j) for j in range(num)]
        else:
            sensor_labels = sorted(set(s.strip().upper() for s in str(sens_str).split()),
                                   key='ABCDE'.index)
            if not sensor_labels or any(s not in 'ABCDE' for s in sensor_labels):
                sensor_labels = ['A', 'B', 'C', 'D', 'E']

        num_sensors = len(sensor_labels)
        sns_list = [s.strip() for s in str(sensor_sns_list[i]).split(',') if s.strip()]
        print(sns_list)

        dw = DataReceiverWriter(
            num_sensors=num_sensors,
            sensor_labels=sensor_labels,
            sensor_sns=sns_list,
            header_content=header_list[i],
            nano_id=nid,
            probe_height_m=probe_height_m[i],
            show_raw_strains=show_raw_list[i]
        )
        handlers.append(dw)

        if plot:
            win = RealTimePlotWindow(dw, num_sensors, sensor_labels,
                                     show_raw_strains=show_raw_list[i])
            windows.append(win)

        dw.start()
        print(f"  → Started collection for Nano_{nid:02d}")

    print(f"\nStarted {len(handlers)} probe(s) sharing one WiFiDataServer.")

    if plot:
        print("=== Press SPACE in any plot window to stop that probe ===")
        if windows:
            app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
            app.exec_()
    else:
        print("=== Press Ctrl+C to stop all probes ===")
        try:
            while any(h.running for h in handlers):
                time.sleep(0.2)
        except KeyboardInterrupt:
            print("Stopping all probes...")
            for h in handlers:
                h.stop()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Multi-probe Hi-STIFFS data collection")
    parser.add_argument('--save-format', choices=['volts', 'raw'], default='raw')
    parser.add_argument('--plot', type=bool, default=True)
    parser.add_argument('--sensors', default='A B C D E')
    parser.add_argument('--sensor-sns', default='001,002,003,004,005')
    parser.add_argument('--nano-id', type=int, default=1, help="2-digit flashed NANO_ID on the target Arduino")
    parser.add_argument('--show-raw-strains', action='store_true', help="Also create raw strain plot window (higher resource use)")
    args = parser.parse_args()

    # For standalone: Use example header_content (GUI will override with dynamic list)
    example_header_content = [
        "Note: Dummy code tests",
        "Test Type: Medium Lab w/o tops",
        "Speed: 50 ft/min, Stalk Spacing: 6in",
        "Analog-to-Digital Converter: ADS1220, Mode: Turbo, Data Rate: DR_330SPS, Analog Excitation/Reference Voltage: 5.1V +/-2mV",
        "DAQ Microcontroller: Arduino Nano ESP32, Data-stream Connection: Wi-Fi"
    ]
    run_collection( nano_id=[1, 2],
                    sensors=["A B C D E", "A B C D E"],
                    sensor_sns=["001,002,003,004,005", "011,012,013,014,015"],
                    probe_height_m=[0.785, 0.785],
                    header_content=[example_header_content, example_header_content])
