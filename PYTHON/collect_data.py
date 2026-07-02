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
SCREEN_SCALE = 1.3
SCREEN_WIDTH = int(1920 * SCREEN_SCALE)
SCREEN_HEIGHT = int(1080 * SCREEN_SCALE)


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
        self._crc_table = self._generate_crc16_table()

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
                header = self._read_fully(conn, 4)
                length = struct.unpack('<H', header[:2])[0]
                received_crc = struct.unpack('<H', header[2:4])[0]

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
                    else:
                        raise ValueError(f"Received packet from Nano_{nano_id:02d} but no handler registered.")
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
        """
        Fast table-driven CRC16 (matches original bit-by-bit implementation exactly).
        Called from the network thread for every packet before queuing.
        """
        crc = 0xFFFF
        for byte in data:
            crc = self._crc_table[(crc ^ byte) & 0xFF] ^ (crc >> 8)
        return crc

    @staticmethod
    def _generate_crc16_table():
        """Build the 256-entry lookup table once (reflected CRC-16, poly 0xA001, init 0xFFFF)."""
        table = [0] * 256
        for i in range(256):
            crc = i
            for _ in range(8):
                if crc & 1:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
            table[i] = crc
        return table


class DataReceiverWriter(QtCore.QThread):
    """
    Per-probe data handler.
    One instance per Nano ID. Owns its CSV, calibrations, processing thread, and signals.
    Registers with the shared WiFiDataServer (first instance creates the server).
    """

    data_ready = QtCore.pyqtSignal(list)
    status_signal = QtCore.pyqtSignal(str)
    rate_updated = QtCore.pyqtSignal(float)
    imu_data_ready = QtCore.pyqtSignal(list)

    _shared_server = None   # class-level shared server (created by the first DataReceiverWriter)
    
    def __init__(self, num_sensors, sensor_labels=['A', 'B', 'C'], sensor_sns=None, imu_mode=False,
                header_content=None, registry=None, nano_id=1, probe_height_m=None, show_raw_strains=False):
        # =============================================================================
        # DataReceiverWriter.__init__  (reorganized + extensive explanatory comments)
        # =============================================================================
        # This __init__ belongs to the per-probe data handler. One instance is created
        # for every Nano probe that will stream data. It owns that probe's CSV file,
        # its calibration coefficients, the background unpacking/writing thread, and
        # the Qt signals that feed the real-time plot window.
        # Call QThread.__init__. This registers the object with Qt's threading
        # system so that pyqtSignal emissions from the background processing thread
        # are safely delivered to the main GUI thread (required for thread safety).
        super().__init__()

        # ---------------------------------------------------------------------
        # 1. INGEST CONSTRUCTOR ARGUMENTS AND ESTABLISH CORE IDENTITY
        # ---------------------------------------------------------------------
        # These attributes define "who this handler is" for the rest of its life.
        # They are intentionally placed first so a reader immediately sees the
        # object's identity and configuration without scrolling.
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
        self.imu_mode = imu_mode

        # Pre-compute the exact byte lengths we expect for the two possible
        # binary packet formats coming from the Arduino. Used in process_batches
        # to decide which struct.unpack format string to apply.
        self.ICB_payload_len = Config.get_sensor_payload_length(Config.SENSOR_TYPE_ICB, num_sensors)
        self.IMU_MAG_payload_len = Config.get_sensor_payload_length(Config.SENSOR_TYPE_IMU_MAG)

        # ---------------------------------------------------------------------
        # 2. PER-SENSOR ONLINE ZERO-CALIBRATION STATE
        # ---------------------------------------------------------------------
        # These lists are used by the automatic zero-offset logic that runs during
        # the first ~1 second of streaming. They live on the handler (even though
        # the actual math is performed in RealTimePlotWindow.handle_data) so that
        # the handler remains a self-contained owner of everything related to one
        # physical probe. The state is duplicated in the plot window for direct
        # access from the GUI thread.
        self.calibration_active = [True] * self.num_sensors
        self.cal_start_time = [None] * self.num_sensors
        self.initial_strains1 = [[] for _ in range(self.num_sensors)]
        self.initial_strains2 = [[] for _ in range(self.num_sensors)]
        self.cal_duration = 1.0
        self.min_samples = 150
        self.last_t = 0.0

        # ---------------------------------------------------------------------
        # 3. SHARED WiFiDataServer SINGLETON (multi-probe support)
        # ---------------------------------------------------------------------
        # Exactly one WiFiDataServer exists per Python process. The first
        # DataReceiverWriter created starts it; every subsequent handler only
        # registers itself so the server knows which handler should receive
        # packets for a given nano_id byte. This is the mechanism that lets
        # run_collection() start several probes with a single TCP listener.
        if DataReceiverWriter._shared_server is None:
            DataReceiverWriter._shared_server = WiFiDataServer()
            DataReceiverWriter._shared_server.start()
        DataReceiverWriter._shared_server.register_probe(self)

        # ---------------------------------------------------------------------
        # 4. SENSOR REGISTRY AND CALIBRATION COEFFICIENTS
        # ---------------------------------------------------------------------
        # The SensorRegistry is the single source of truth for k/d/c coefficients.
        # We either receive an already-populated registry from the GUI or create
        # a fresh one. We then force the label→serial-number mapping so that
        # get_coeffs() can be called by label ('A', 'C', etc.) inside the hot path.
        self.registry = registry or SensorRegistry()
        if not self.registry.label_to_sn:
            self.registry.set_mapping(dict(zip(self.sensor_labels, self.sensor_sns)))
        self.calibrations = {l: self.registry.get_coeffs(l) for l in self.sensor_labels}

        # ---------------------------------------------------------------------
        # 5. PER-PROBE CSV FILE CREATION (shared timestamp across probes)
        # ---------------------------------------------------------------------
        # Config.get_session_filename() guarantees that every probe started in the
        # same run_collection() call receives a filename containing the identical
        # YYYY-MM-DD_HHMMSS prefix. Only the two-digit nano_id suffix differs.
        # This satisfies the requirement that all raw data files from a multi-probe
        # session share the same logical start time.
        self.csv_path, date_str, time_str = Config.get_session_filename(self.nano_id)
        self.csvfile = open(self.csv_path, 'w', newline='')
        self.csvwriter = csv.writer(self.csvfile)
        print(f"[DataReceiverWriter Nano_{self.nano_id:02d}] CSV path: {self.csv_path}")
        if self.imu_mode:
            self.csv_path_imu = str(self.csv_path).replace('.csv', '_IMU.csv')
            self.csvfile_imu = open(self.csv_path_imu, 'w', newline='')
            self.csvwriter_imu = csv.writer(self.csvfile_imu)
            print(f"[DataReceiverWriter Nano_{self.nano_id:02d}] CSV path for IMU: {self.csv_path_imu}")

        # ---------------------------------------------------------------------
        # 6. WRITE METADATA HEADER BLOCK (makes every file self-describing)
        # ---------------------------------------------------------------------
        # The HiSTIFFSData parser in process.py expects these exact markers and
        # key-value lines. Everything written here is later parsed back out when
        # the file is opened for stiffness estimation or plotting.
        self.csvwriter.writerow(['===BEGIN_METADATA==='])
        self.csvwriter.writerow([f'Test Name: {date_str}_{time_str}_{self.nano_id:02d}', 'yyyy-mm-dd_hhmmss_{probe#/nano_id}'])
        self.csvwriter.writerow([f'Probe ID: Nano_{self.nano_id:02d}', f'Probe Height (m): {probe_height_m:.4f}'])
        for row in header_content:
            self.csvwriter.writerow(row.split(', '))

        self.csvwriter.writerow([f"Number of ICB-Sensors: {self.num_sensors}", f"Sensor Label(s): {' '.join(self.sensor_labels)}", f"Sensor SNs: {' '.join(self.sensor_sns)}"])

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
        data_headers += ['Processed_Time']
        self.csvwriter.writerow(data_headers)
        print(f"[DataReceiverWriter Nano_{self.nano_id:02d}] Added metadata and headers to CSV")

        if self.imu_mode:
            self.csvwriter_imu.writerow(['===BEGIN_METADATA==='])
            self.csvwriter_imu.writerow([f'This CSV contains gyroscope, accelerometer, and magnetometer data synced with the test named below.'])
            self.csvwriter_imu.writerow([f'Test Name: {date_str}_{time_str}_{self.nano_id:02d}', 'yyyy-mm-dd_hhmmss_{probe#/nano_id}'])
            self.csvwriter_imu.writerow(['Calibrations: (not implemented yet)'])

            self.csvwriter_imu.writerow([Config.HEADER_MARKER])
            self.csvwriter_imu.writerow([Config.DATA_MARKER])

            data_headers_imu = ['Time (sec)', 'Gyro X', 'Gyro Y', 'Gyro Z', 'Accel X', 'Accel Y', 'Accel Z', 'Mag X', 'Mag Y', 'Mag Z']
            data_headers_imu += ['Processed_Time']
            self.csvwriter_imu.writerow(data_headers_imu)
            print(f"[DataReceiverWriter Nano_{self.nano_id:02d}] Added metadata and headers to CSV for IMU")

        # ---------------------------------------------------------------------
        # 7. CONCURRENCY PRIMITIVES AND BACKGROUND PROCESSING THREAD
        # ---------------------------------------------------------------------
        # receive_queue is the thread-safe rendezvous point between the network
        # thread inside WiFiDataServer and this object's own processing thread.
        # processed_buffer holds fully decoded rows that are about to be emitted
        # to the GUI via the data_ready signal.
        self.receive_queue = queue.Queue()
        self.processed_buffer = collections.deque()
        self.imu_processed_buffer = collections.deque()
        self.last_rate_time = time.time()
        self.first_packet_time = None
        self.rate_estimate = 10.0
        self.last_rate_update = time.perf_counter()

        # The daemon thread that does the actual work: it drains receive_queue,
        # unpacks binary payloads, writes CSV rows, and emits Qt signals.
        # daemon=True guarantees it will be terminated when the main thread exits.
        self.processing_thread = threading.Thread(target=self.process_batches, daemon=True)
        self.processing_thread.start()

    def run(self):
        """QThread main loop — keeps the thread alive for Qt signals while processing happens in background thread."""
        while self.running:
            QtCore.QThread.msleep(200)
        self._cleanup()

    def _cleanup(self):
        if DataReceiverWriter._shared_server:
            DataReceiverWriter._shared_server.unregister_probe(self.nano_id)
        self._maybe_flush_csv(force=True)   # ensure everything is on disk
        if self.csvfile:
            self.csvfile.close()
        if self.imu_mode and self.csvfile_imu:
            self.csvfile_imu.close()
        print(f"[DataReceiverWriter Nano_{self.nano_id:02d}] Stopped and CSV closed.")

    def stop(self):
        self.running = False
        if DataReceiverWriter._shared_server:
            DataReceiverWriter._shared_server.unregister_probe(self.nano_id)

    def process_batches(self):
        """Per-probe processing thread.

        Drains the receive_queue as fast as packets arrive, validates and unpacks
        binary sensor payloads from the Nano, writes timestamped rows to CSV,
        and immediately emits the entire processed batch via data_ready signal.
        Every packet processed in a cycle is emitted together in one flat list 
        for lowest latency and simplest behavior.
        """
        while self.running:
            batch_rows = []
            batch_rows_imu = []

            try:
                # Block for up to 50 ms waiting for first packet. This is the
                # main idle path — far fewer wakeups than previous 2 ms sleep loop.
                first_packet = self.receive_queue.get(block=True, timeout=0.05)
                post_data_list = [first_packet]
                # Drain everything else that arrived during the wait (non-blocking)
                while not self.receive_queue.empty():
                    post_data_list.append(self.receive_queue.get())
            except queue.Empty:
                post_data_list = []

            for post_data in post_data_list:
                # (existing packet length / unpack / validation logic stays exactly the same)
                if len(post_data) == self.ICB_payload_len:
                    sensor_payload = post_data
                    fmt = '<BB' + 'Iii' * self.num_sensors
                    sensor_type = Config.SENSOR_TYPE_ICB
                elif len(post_data) == self.IMU_MAG_payload_len:
                    if not self.imu_mode:
                        continue
                    sensor_payload = post_data
                    fmt = '<B' + 'I' + 'h'*9
                    sensor_type = Config.SENSOR_TYPE_IMU_MAG
                else:
                    self.status_signal.emit(f"Invalid packet length for Nano_{self.nano_id:02d}")
                    continue

                try:
                    unpacked = struct.unpack(fmt, sensor_payload)
                    probe_id = unpacked[0]
                    if probe_id != self.nano_id:
                        continue

                    if sensor_type == Config.SENSOR_TYPE_ICB:
                        times_us = unpacked[2::3]
                        raws1    = unpacked[3::3]
                        raws2    = unpacked[4::3]
                        times = [ts / 1000000.0 for ts in times_us]

                        t_now = times[0]
                        delta = t_now - self.last_t
                        self.rate_estimate = 1.0 / delta if delta > 1e-9 else 0.0
                        self.last_t = t_now

                        row = []
                        for j in range(self.num_sensors):
                            row += [f"{times[j]:.6f}", f"{raws1[j]:+08d}", f"{raws2[j]:+08d}"]
                        row += [datetime.datetime.now().time()]
                        batch_rows.append(row)

                        emit_list = []
                        for j in range(self.num_sensors):
                            emit_list += [times[j], raws1[j], raws2[j]]
                        self.processed_buffer.append(emit_list)

                    elif sensor_type == Config.SENSOR_TYPE_IMU_MAG:
                        # (IMU handling unchanged except it will now also benefit from batched emit in Improvement 4)
                        time_us = unpacked[1]
                        time_s = float(time_us / 1_000_000.0)
                        gyro_x, gyro_y, gyro_z = unpacked[2], unpacked[3], unpacked[4]
                        accel_x, accel_y, accel_z = unpacked[5], unpacked[6], unpacked[7]
                        mag_x, mag_y, mag_z = unpacked[8], unpacked[9], unpacked[10]

                        row = [f"{time_s:.6f}",
                               f"{gyro_x:+08d}", f"{gyro_y:+08d}", f"{gyro_z:+08d}",
                               f"{accel_x:+08d}", f"{accel_y:+08d}", f"{accel_z:+08d}",
                               f"{mag_x:+08d}", f"{mag_y:+08d}", f"{mag_z:+08d}"]
                        row += [datetime.datetime.now().time()]
                        batch_rows_imu.append(row)

                        emit_list_imu = [time_s, gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z, mag_x, mag_y, mag_z]
                        self.imu_processed_buffer.append(emit_list_imu)

                except Exception:
                    print("Error processing a packet's payload")
                    continue

            # Write entire batch to CSV + throttled flush
            if batch_rows:
                self.csvwriter.writerows(batch_rows)
                self._maybe_flush_csv() # only flush CSV writer if many rows have been writen since last flush
            if batch_rows_imu:
                self.csvwriter_imu.writerows(batch_rows_imu)
                self._maybe_flush_csv()

            # Emit ICB batch in one emission
            if self.processed_buffer:
                batch_emit_lists = []
                while self.processed_buffer:
                    batch_emit_lists.append(self.processed_buffer.popleft())
                if batch_emit_lists:
                    flat = [item for sub in batch_emit_lists for item in sub]
                    self.data_ready.emit(flat)
                    self.rate_updated.emit(self.rate_estimate)

            # Emit IMU/MAG batch in one emission
            if self.imu_processed_buffer:
                imu_batch_emit_list = []
                while self.imu_processed_buffer:
                    imu_batch_emit_list.append(self.imu_processed_buffer.popleft())
                if imu_batch_emit_list:
                    self.imu_data_ready.emit(imu_batch_emit_list)
            
    
    def _maybe_flush_csv(self, force=False):
        """
        Throttled CSV flush.
        Intent: Coalesce writes for better throughput + lower jitter on Pi 5 SD /
        Windows storage while keeping data safe. Flush every ~300 ms or 200 rows.
        """
        now = time.time()
        rows_since_flush = getattr(self, '_rows_since_flush', 0) + 1
        self._rows_since_flush = rows_since_flush

        should_flush = force or (now - getattr(self, '_last_csv_flush', 0) > 0.3) or (rows_since_flush > 200)
        if should_flush:
            if self.csvfile:
                self.csvfile.flush()
            if self.imu_mode and self.csvfile_imu:
                self.csvfile_imu.flush()
            self._last_csv_flush = now
            self._rows_since_flush = 0


class RealTimePlotWindow(QtWidgets.QMainWindow):
    """
    Real-time plotting window (Force/Position + optional Raw Strain).
    Supports per-probe instances and the existing Pi5 performance tunings + show_raw_strains flag.
    (Full implementation restored from original with nano_id awareness — identical behavior.)
    """
    
    def __init__(self, readwrite, num_sensors, sensor_labels, show_raw_strains=False, imu_mode=False):
        # =============================================================================
        # RealTimePlotWindow.__init__  
        # =============================================================================
        # This __init__ builds the real-time Force/Position (and optionally Raw Strain)
        # visualisation windows. It receives a fully constructed DataReceiverWriter
        # instance ("readwrite") and connects to its signals. All platform-specific
        # performance decisions (desktop vs Raspberry Pi 5) are made here.
        #
        # When imu_mode=True we also instantiate a separate IMUPlotWindow (defined
        # below) that receives the new imu_data_ready signal. This keeps the two
        # visualisation concerns cleanly separated while sharing the same
        # DataReceiverWriter and WiFiDataServer.
        super().__init__()
        self.num_sensors = num_sensors
        self.sensor_labels = sensor_labels
        self.show_raw_strains = show_raw_strains
        self.imu_mode = imu_mode
        self.ReadWrite = readwrite
        self.imu_window = None

        # ---------------------------------------------------------------------
        # SIGNAL WIRING (must happen before any data can arrive)
        # ---------------------------------------------------------------------
        # Connect the three signals emitted by the handler. data_ready carries
        # batches of unpacked sensor values; rate_updated drives the live Hz label.
        # Connecting immediately after storing the reference guarantees we never
        # miss the first packets that arrive while the window is still being built.
        self.ReadWrite.data_ready.connect(self.handle_data)
        self.ReadWrite.status_signal.connect(print)
        self.ReadWrite.rate_updated.connect(lambda rate: self.rate_label.setText(f"Input Rate (Nano_{readwrite.nano_id:02d}): {rate:.1f} Hz"))
        if self.imu_mode:
            self.imu_window = IMUPlotWindow(self.ReadWrite)
            self.imu_window.show()
            # Position it nicely below the main force/pos window on desktop;
            # on RPi5 the user can drag/resize as needed. We keep it independent
            # so the operator can minimize one or the other during long runs.
            self.imu_window.move(0, int(SCREEN_HEIGHT * 0.68) + 40)


        # ---------------------------------------------------------------------
        # PLATFORM-SPECIFIC PERFORMANCE TUNING
        # ---------------------------------------------------------------------
        # On Raspberry Pi 5 we deliberately lower the plot refresh rate and
        # disable OpenGL because the combination of pyqtgraph + full-rate
        # downsampled curves can overload the Pi5's GPU when running on the
        # official 7-inch touchscreen. maxlen is kept small to limit RAM usage
        # on the embedded device. Desktop/laptop gets the full 30 Hz experience.
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

        # ---------------------------------------------------------------------
        # CACHED CALIBRATION COEFFICIENTS (hot-path optimisation)
        # ---------------------------------------------------------------------
        # We copy the k/d/c values into simple Python lists so that the math
        # inside handle_data does not perform repeated dictionary lookups on
        # every incoming packet. This is a measurable win at 300–1000 Hz aggregate
        # data rates.
        cal = self.ReadWrite.calibrations
        self.k1 = [cal[l]['k1'] for l in sensor_labels]
        self.d1 = [cal[l]['d1'] for l in sensor_labels]
        self.c1 = [cal[l]['c1'] for l in sensor_labels]
        self.k2 = [cal[l]['k2'] for l in sensor_labels]
        self.d2 = [cal[l]['d2'] for l in sensor_labels]
        self.c2 = [cal[l]['c2'] for l in sensor_labels]

        # Duplicate of the handler's online-zero state (see DataReceiverWriter
        # comments). These control the automatic collection of the first-second
        # strain samples used to compute a live zero offset. Having the lists
        # here lets handle_data run entirely on the GUI thread without extra
        # locking.
        self.calibration_active = [True] * self.num_sensors
        self.cal_start_time = [None] * self.num_sensors
        self.initial_strains1 = [[] for _ in range(self.num_sensors)]
        self.initial_strains2 = [[] for _ in range(self.num_sensors)]
        self.cal_duration = 1.0
        self.min_samples = 150

        # Final pg configuration. On non-Pi5 platforms we want antialiasing and
        # OpenGL for crisp traces. On Pi5 the earlier disable is left in effect.
        pg.setConfigOptions(useOpenGL=True, antialias=False)

        # ---------------------------------------------------------------------
        # MAIN FORCE / POSITION WINDOW (always created)
        # ---------------------------------------------------------------------
        # We use a single GraphicsLayoutWidget containing two plots that share
        # the x-axis. This gives the classic "Force on top, Position below"
        # layout with linked zooming/panning behaviour.
        self.setWindowTitle(f"Force and Position - Nano_{readwrite.nano_id:02d}")
        self.win_force_pos = pg.GraphicsLayoutWidget()
        self.plot_force = self.win_force_pos.addPlot(title='Force')
        self.plot_force.setLabel('left', '', units='N')
        self.plot_force.addLegend()
        self.plot_pos = self.win_force_pos.addPlot(title='Position')
        self.plot_pos.setLabel('left', '', units='mm')
        self.plot_pos.addLegend()

        colors = Config.COLORS[:self.num_sensors]
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

        # Horizontal button bar at the bottom of the window.
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

        # ---------------------------------------------------------------------
        # OPTIONAL RAW STRAIN WINDOW (created only when requested)
        # ---------------------------------------------------------------------
        # Creating this second window doubles the number of pyqtgraph curves and
        # the amount of data copied every refresh. It is therefore gated behind
        # the show_raw_strains flag so that the default (and Pi5) experience
        # stays lightweight.
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

        # ---------------------------------------------------------------------
        # RING BUFFERS FOR PLOT DATA (efficient recent-history storage)
        # ---------------------------------------------------------------------
        # We keep only the last N samples per sensor using collections.deque with
        # a maxlen. This gives O(1) append and automatic discarding of old data,
        # which is exactly what a scrolling real-time plot needs. The value of
        # maxlen was already chosen above according to the detected platform.
        maxlen = self.maxlen
        self.times = [collections.deque(maxlen=maxlen) for _ in range(self.num_sensors)]
        self.forces = [collections.deque(maxlen=maxlen) for _ in range(self.num_sensors)]
        self.positions = [collections.deque(maxlen=maxlen) for _ in range(self.num_sensors)]
        if self.show_raw_strains:
            self.strains1 = [collections.deque(maxlen=maxlen) for _ in range(self.num_sensors)]
            self.strains2 = [collections.deque(maxlen=maxlen) for _ in range(self.num_sensors)]
        else:
            self.strains1 = self.strains2 = None

        # ---------------------------------------------------------------------
        # PLOT REFRESH TIMER
        # ---------------------------------------------------------------------
        # A QTimer periodically calls update_plots at the platform-appropriate
        # rate. update_plots pulls the latest values from the deques above and
        # calls setData on the pyqtgraph PlotDataItems. This decouples the high-
        # rate data arrival (via signals) from the actual screen refresh.
        self.plot_timer = QtCore.QTimer()
        self.plot_timer.timeout.connect(self.update_plots)
        self.plot_timer.start(int(1000 / self.plot_refresh_hz))

        # ---------------------------------------------------------------------
        # INTERACTIVE DISPLAY STATE
        # ---------------------------------------------------------------------
        # These variables control the time-window buttons, the one-time initial
        # Y-axis auto-scale, and the periodic re-scaling behaviour. They are
        # placed at the very end because they are only referenced after the
        # widget tree and timers have been fully constructed.
        self.display_time_range = 10.0
        self.initial_rescale_done = False
        self.last_y_rescale_time = 0.0
        self.rescale_interval = 0.5
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
                    position = num / den if den != 0 and abs(den) > 1e9 else 0.0
                    position = position if 0.03 <= position <= 0.16 else 0.0
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
            # if i in [1]:
            #     continue
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
                t_max = 0
                for t in self.times:
                    if t:
                        t_max = max(t_max, t[-1])
                x_min = max(0, t_max - self.display_time_range)
                x_max = t_max

                # Rescale each plot to the actual min/max of data currently on screen
                self._rescale_y_to_visible_data(self.plot_force, self.times, self.forces, x_min, x_max)
                self._rescale_y_to_visible_data(self.plot_pos,   self.times, self.positions, x_min, x_max)

                if self.show_raw_strains:
                    self._rescale_y_to_visible_data(self.plot_ch1, self.times, self.strains1, x_min, x_max)
                    self._rescale_y_to_visible_data(self.plot_ch2, self.times, self.strains2, x_min, x_max)

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

    def _rescale_y_to_visible_data(self, plot, time_deques, data_deques_list, x_min, x_max):
        """
        Set Y range of 'plot' to min/max of all data currently visible in [x_min, x_max].
        'data_deques_list' is a list of deques (one per curve on this plot).
        Called every 1 second from update_plots.
        """
        if not time_deques or not data_deques_list:
            return

        visible_values = []
        for t_deque, y_deque in zip(time_deques, data_deques_list):
            for t, y in zip(t_deque, y_deque):
                if x_min <= t <= x_max:
                    visible_values.append(y)

        if not visible_values:
            return

        y_min = min(visible_values)
        y_max = max(visible_values)
        span = y_max - y_min
        padding = max(0.05 * span, abs(y_max) * 0.02) if span > 0 else 1.0
        plot.setYRange(y_min - padding, y_max + padding)

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
        if self.imu_window is not None:
            self.imu_window.close()
        self.plot_timer.stop()
        self.ReadWrite.stop()


class IMUPlotWindow(QtWidgets.QMainWindow):
    """
    Self-contained live monitor for the 9-DOF IMU stream (gyro + accel + mag).
    Receives data via Qt signal from DataReceiverWriter, maintains rolling
    ring buffers, and renders at a smooth decoupled rate using pyqtgraph.
    """
    # =============================================================================
    # IMUPlotWindow — Live 9-DOF IMU visualiser (ported & adapted from IMUMonitor)
    # =============================================================================
    # This class provides a real-time plotting window for the ISM330DHCX + LIS3MDL
    # stream that is already being received and logged by DataReceiverWriter when
    # imu_mode=True. It is intentionally a near drop-in replacement for the
    # original IMUMonitor in read_IMU_serial.py so that the operator sees
    # identical behaviour and can validate the exact same raw int16 values that
    # will later be used for 1-2 s pose estimation / drift correction on the RPi5.
    #
    # Key design decisions (documented for maintainability):
    # - Uses the same efficient deque + separate render-timer pattern proven in
    #   the serial bring-up tool. Data ingestion (via Qt signal) is completely
    #   decoupled from screen refresh (30-40 Hz on desktop, reduced on Pi5).
    # - All hot-path operations are O(1) or amortized constant. We only convert
    #   deques → np.ndarray inside the plot timer callback.
    # - Cross-platform: identical source runs on Win10/11, Ubuntu, and RPi5
    #   touchscreen. We reuse the existing is_pi5 detection + pg config already
    #   present in RealTimePlotWindow.
    # - The window is self-contained: it owns its buffers, timers, and pyqtgraph
    #   items. It connects itself to the imu_data_ready signal passed in via the
    #   DataReceiverWriter reference.
    # - Rolling time window defaults to 5 s (more context than the 2 s used for
    #   ICB force/pos) but is easily changed via the constructor.
    # - Raw integers are preserved exactly; unit conversion belongs downstream in
    #   the pose / stiffness pipeline (same philosophy as the original monitor).
    # =============================================================================

    # Match the constants used in the original serial monitor for consistency
    DEFAULT_IMU_RATE_HZ = 3000          # conservative upper bound for maxlen calc
    DEFAULT_MAG_RATE_HZ = 1000
    DEFAULT_PLOT_WINDOW_S = 3.0         # longer horizon than ICB for IMU validation

    def __init__(self, data_receiver_writer, plot_window_s: float = DEFAULT_PLOT_WINDOW_S):
        super().__init__()
        self.setWindowTitle("Hi-STIFFS | 9-DOF IMU Live Monitor (Wi-Fi) — Raw Integers")
        self.resize(1100, 900)

        self.ReadWrite = data_receiver_writer
        self.plot_window_s = plot_window_s
        self.start_ts_us = None
        self.pkt_count = 0
        self._last_t = None
        self.rescale_interval = 0.5
        self.last_y_rescale_time = 0.0

        # -----------------------------------------------------------------
        # ROLLING BUFFERS (deque with maxlen = efficient ring buffer)
        # -----------------------------------------------------------------
        # Same strategy as IMUMonitor and as the ICB deques in RealTimePlotWindow.
        # maxlen gives automatic oldest-sample drop. Extra headroom so we can
        # scroll smoothly while the visible window is only plot_window_s seconds.
        maxlen = int(plot_window_s * self.DEFAULT_IMU_RATE_HZ * 1.1) + 200
        self.t_buf   = collections.deque(maxlen=maxlen)
        self.gx_buf  = collections.deque(maxlen=maxlen)
        self.gy_buf  = collections.deque(maxlen=maxlen)
        self.gz_buf  = collections.deque(maxlen=maxlen)
        self.ax_buf  = collections.deque(maxlen=maxlen)
        self.ay_buf  = collections.deque(maxlen=maxlen)
        self.az_buf  = collections.deque(maxlen=maxlen)
        self.mx_buf  = collections.deque(maxlen=maxlen)
        self.my_buf  = collections.deque(maxlen=maxlen)
        self.mz_buf  = collections.deque(maxlen=maxlen)

        # -----------------------------------------------------------------
        # UI LAYOUT — identical structure to the original IMUMonitor
        # -----------------------------------------------------------------
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(4)

        # Status line (monospace, large for quick glance on RPi5 touchscreen)
        self.status_label = QtWidgets.QLabel()
        self.status_label.setStyleSheet("font-family: monospace; font-size: 30px;")
        layout.addWidget(self.status_label)

        # ---- Gyro subplot ----
        self.gyro_plot = pg.PlotWidget(title="Gyroscope — raw int16 (ISM330DHCX)")
        self.gyro_plot.setLabel('left', 'counts')
        self.gyro_plot.setLabel('bottom', 'time since start (s)')
        self.gyro_plot.addLegend()
        self.gyro_plot.showGrid(x=True, y=True, alpha=0.3)
        self.gx_curve = self.gyro_plot.plot(pen=pg.mkPen('r', width=1.5), name='gx')
        self.gy_curve = self.gyro_plot.plot(pen=pg.mkPen('g', width=1.5), name='gy')
        self.gz_curve = self.gyro_plot.plot(pen=pg.mkPen('b', width=1.5), name='gz')
        layout.addWidget(self.gyro_plot, stretch=1)

        # ---- Accel subplot ----
        self.accel_plot = pg.PlotWidget(title="Accelerometer — raw int16 (ISM330DHCX)")
        self.accel_plot.setLabel('left', 'counts')
        self.accel_plot.setLabel('bottom', 'time since start (s)')
        self.accel_plot.addLegend()
        self.accel_plot.showGrid(x=True, y=True, alpha=0.3)
        self.ax_curve = self.accel_plot.plot(pen=pg.mkPen('r', width=1.5), name='ax')
        self.ay_curve = self.accel_plot.plot(pen=pg.mkPen('g', width=1.5), name='ay')
        self.az_curve = self.accel_plot.plot(pen=pg.mkPen('b', width=1.5), name='az')
        layout.addWidget(self.accel_plot, stretch=1)

        # ---- Mag subplot ----
        self.mag_plot = pg.PlotWidget(title="Magnetometer — raw int16 (LIS3MDL)")
        self.mag_plot.setLabel('left', 'counts')
        self.mag_plot.setLabel('bottom', 'time since start (s)')
        self.mag_plot.addLegend()
        self.mag_plot.showGrid(x=True, y=True, alpha=0.3)
        self.mx_curve = self.mag_plot.plot(pen=pg.mkPen('r', width=1.5), name='mx')
        self.my_curve = self.mag_plot.plot(pen=pg.mkPen('g', width=1.5), name='my')
        self.mz_curve = self.mag_plot.plot(pen=pg.mkPen('b', width=1.5), name='mz')
        layout.addWidget(self.mag_plot, stretch=1)

        # Link X axes so all three plots scroll together (critical for pose window analysis)
        self.accel_plot.setXLink(self.gyro_plot)
        self.mag_plot.setXLink(self.gyro_plot)

        # -----------------------------------------------------------------
        # TIMERS — decoupled ingest vs. render (the key to smooth high-rate performance)
        # -----------------------------------------------------------------
        # Plot refresh timer runs at platform-appropriate rate.
        # We detect Pi5 here (same logic as RealTimePlotWindow) and tune accordingly.
        self.is_pi5 = self._detect_raspberry_pi5()
        if self.is_pi5:
            self.plot_refresh_hz = 12.0          # conservative for touchscreen + pyqtgraph
            pg.setConfigOptions(useOpenGL=False)
            print("[IMUPlotWindow] Raspberry Pi 5 detected — using reduced refresh + no OpenGL")
        else:
            self.plot_refresh_hz = 40.0
            pg.setConfigOptions(useOpenGL=True, antialias=False)

        self.plot_timer = QtCore.QTimer()
        self.plot_timer.timeout.connect(self._update_plots)
        self.plot_timer.start(int(1000 / self.plot_refresh_hz))

        # Initial status
        self._update_status("Waiting for first IMU packet...")

        # -----------------------------------------------------------------
        # SIGNAL WIRING — connect to the DataReceiverWriter we were given
        # -----------------------------------------------------------------
        # This must happen before any data can arrive. The signal is emitted
        # from the background processing thread; Qt's queued connection
        # guarantees safe delivery to this GUI thread.
        self.ReadWrite.imu_data_ready.connect(self._handle_imu_packet)

    def _detect_raspberry_pi5(self):
        """Identical helper to the one in RealTimePlotWindow for consistency."""
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

    def _handle_imu_packet(self, imu_batch):
        """
        Called via Qt signal from DataReceiverWriter (background thread).
        Extremely lightweight — just append to deques and bump counters.
        All heavy lifting (array conversion + drawing) happens in _update_plots.
        Supports batched emissions (list-of-packets) for
        reduced Qt event overhead on Pi 5 / desktop. Legacy single-packet calls
        are still supported for safety during the transition.
        """
        if not imu_batch:
            return

        # # Normalize to always iterate over a list of packets
        # # (supports both the new batched emit and any old single-packet calls)
        # if isinstance(imu_batch[0], (list, tuple)) and len(imu_batch[0]) == 10:
        #     packets = imu_batch
        # else:
        #     packets = [imu_batch]

        for data_list in imu_batch:
            if len(data_list) != 10:
                continue

            time_s, gx, gy, gz, ax, ay, az, mx, my, mz = data_list

            if self.start_ts_us is None:
                # First packet — anchor the relative time base exactly like IMUMonitor
                self.start_ts_us = time_s * 1_000_000   # store as us for consistency
                self._first_wall_time = time.perf_counter()

            t_rel = time_s - (self.start_ts_us / 1_000_000.0)

            # Append to ring buffers (O(1))
            self.t_buf.append(t_rel)
            self.gx_buf.append(gx)
            self.gy_buf.append(gy)
            self.gz_buf.append(gz)
            self.ax_buf.append(ax)
            self.ay_buf.append(ay)
            self.az_buf.append(az)
            self.mx_buf.append(mx)
            self.my_buf.append(my)
            self.mz_buf.append(mz)

            self.pkt_count += 1

            # Occasional status update (not every packet — cheap)
            if self.pkt_count % int(self.DEFAULT_IMU_RATE_HZ/2) == 0:
                self._update_status()

    def _update_plots(self):
        """Called at 12-40 Hz by QTimer. Only here do we pay the numpy cost."""
        if len(self.t_buf) < 2:
            return

        t_arr = np.fromiter(self.t_buf, dtype=np.float64)
        t_max = t_arr[-1]

        # Auto-scrolling rolling window (exactly the UX from the serial monitor)
        x_min = max(0.0, t_max - self.plot_window_s)
        self.gyro_plot.setXRange(x_min, t_max, padding=0)
        # --- 1-second Y auto-rescale for all three IMU plots ---
        now = time.time()
        if now - self.last_y_rescale_time >= self.rescale_interval:
            x_min = max(0.0, t_max - self.plot_window_s)
            x_max = t_max

            # Gyro
            self._rescale_imu_plot_y(self.gyro_plot, self.gx_buf, self.gy_buf, self.gz_buf, x_min, x_max)
            # Accel
            self._rescale_imu_plot_y(self.accel_plot, self.ax_buf, self.ay_buf, self.az_buf, x_min, x_max)
            # Mag
            self._rescale_imu_plot_y(self.mag_plot, self.mx_buf, self.my_buf, self.mz_buf, x_min, x_max)

            self.last_y_rescale_time = now

        # Update all nine curves — very cheap setData after the fromiter
        self.gx_curve.setData(t_arr, np.fromiter(self.gx_buf, dtype=np.int16))
        self.gy_curve.setData(t_arr, np.fromiter(self.gy_buf, dtype=np.int16))
        self.gz_curve.setData(t_arr, np.fromiter(self.gz_buf, dtype=np.int16))

        self.ax_curve.setData(t_arr, np.fromiter(self.ax_buf, dtype=np.int16))
        self.ay_curve.setData(t_arr, np.fromiter(self.ay_buf, dtype=np.int16))
        self.az_curve.setData(t_arr, np.fromiter(self.az_buf, dtype=np.int16))

        self.mx_curve.setData(t_arr, np.fromiter(self.mx_buf, dtype=np.int16))
        self.my_curve.setData(t_arr, np.fromiter(self.my_buf, dtype=np.int16))
        self.mz_curve.setData(t_arr, np.fromiter(self.mz_buf, dtype=np.int16))

    def _rescale_imu_plot_y(self, plot_widget, buf_x, buf_y, buf_z, x_min, x_max):
        """Set Y range to min/max of the three axes that are currently visible."""
        visible = []
        for t, gx, gy, gz in zip(self.t_buf, buf_x, buf_y, buf_z):
            if x_min <= t <= x_max:
                visible.extend([gx, gy, gz])

        if visible:
            y_min = min(visible)
            y_max = max(visible)
            span = y_max - y_min
            padding = max(0.08 * span, 5) if span > 0 else 10   # small fixed padding works well for raw int16
            plot_widget.setYRange(y_min - padding, y_max + padding)

    def _update_status(self, extra: str = ""):
        if len(self.t_buf) < 2:
            rate = 0.0
        else:
            # Sliding-window rate over the last self.rate_window_s seconds
            # (or however much data we have accumulated so far).
            # Using deque end-point access is O(1) and very cheap.
            t_first = self.t_buf[0]
            t_last = self.t_buf[-1]
            t_span = t_last - t_first
            rate = (len(self.t_buf) - 1) / max(t_span, 1e-6)

        msg = (f"IMU Packets: {self.pkt_count:6d} | "
               f"Rate: {rate:.1f} Hz | "
               f"Window: {self.plot_window_s:.1f} s | "
               f"Nano_{self.ReadWrite.nano_id:02d}")
        if extra:
            msg += f" | {extra}"
        self.status_label.setText(msg)

    def closeEvent(self, event):
        """Clean shutdown — stop timer and disconnect signal."""
        print("[IMUPlotWindow] Closing...")
        self.plot_timer.stop()
        try:
            self.ReadWrite.imu_data_ready.disconnect(self._handle_imu_packet)
        except:
            pass
        event.accept()


def run_collection(nano_id=[1], sensors=['A B C'], sensor_sns=['001 002 003'], imu_mode=False,
                   probe_height_m=[None], header_content=[None], plot=True, show_raw_strains=False):
    """
    Start data collection for one or more Hi-STIFFS probes.

    Supports lists so you can start multiple nano_ids with one call:

        run_collection(
            nano_id=[1, 2],
            sensors=["A B C D E", "A B C D E"],
            sensor_sns=["001 002 003 004 005", "101 102 103 104 105"],
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

    sensors_list    = expand(sensors, 'A B C')
    sensor_sns_list = expand(sensor_sns, '001 002 003')
    header_list     = expand(header_content, None)
    show_raw_list   = expand(show_raw_strains, False)

    print(f'Starting run_collection function for {n} Hi-STIFFS probes')
    for nano_id_i, sensors_i, sensor_sns_i in zip(nano_ids, sensors_list, sensor_sns_list):
        print(f'Probe ID: {nano_id_i:02d}. Sensor Labels {sensors_i}. Sensor S/Ns: {sensor_sns_i}')

    # Lock shared timestamp for all probes in this session
    Config.start_new_data_session()
    print(' ')

    handlers = []
    windows = []
    app = QtWidgets.QApplication([])

    for i, nid in enumerate(nano_ids):
        sens_str = sensors_list[i]

        # Parse sensors string for this probe
        sensor_labels = [s.strip().upper() for s in str(sens_str).split()]
        if not sensor_labels or any(s not in Config.ALLOWED_LABELS for s in sensor_labels):
            raise ValueError(f'A sensor label (received {sensor_labels}) is not part of the allowed set. Only "{Config.ALLOWED_LABELS}" allowed')

        num_sensors = len(sensor_labels)
        sns_list = [s.strip() for s in str(sensor_sns_list[i]).split() if s.strip()]

        dw = DataReceiverWriter(
            num_sensors=num_sensors,
            sensor_labels=sensor_labels,
            sensor_sns=sns_list,
            header_content=header_list[i],
            nano_id=nid,
            probe_height_m=probe_height_m[i],
            show_raw_strains=show_raw_list[i],
            imu_mode=imu_mode
        )
        handlers.append(dw)

        if plot:
            win = RealTimePlotWindow(dw, num_sensors, sensor_labels,
                                     show_raw_strains=show_raw_list[i], imu_mode=imu_mode)
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
    parser.add_argument('--plot', type=bool, default=True)
    parser.add_argument('--sensors', default='A B C')
    parser.add_argument('--sensor-sns', default='001 002 003')
    parser.add_argument('--nano-id', type=int, default=1, help="2-digit flashed NANO_ID on the target Arduino")
    parser.add_argument('--show-raw-strains', action='store_true', help="Also create raw strain plot window (higher resource use)")
    args = parser.parse_args()

    # For standalone: Use example header_content (GUI will override with dynamic list)
    example_header_content = [
        "Note: new DAQ PCB",
        "Test Type: code testing",
        # "Stalks: Medium B-IN no tops, Probe: v3.3",
        # "Loads (N): 5 35 70, Positions (mm): 60 100 120 154",
        "Analog-to-Digital Converter: ADS1220, Mode: Turbo, Data Rate: DR_330SPS, Analog Excitation/Reference Voltage: 5.1V +/-2mV",
        "DAQ Microcontroller: Arduino Nano ESP32, Data-stream Connection: Wi-Fi"
    ]
    run_collection( nano_id=[1],
                    sensors=["A B C"],
                    sensor_sns=["001 003 005"],
                    probe_height_m=[0.785],
                    header_content=[example_header_content],
                    show_raw_strains=False,
                    imu_mode=True)
    # run_collection( nano_id=[1, 2],
    #                 sensors=["A C E", "A C E"],
    #                 sensor_sns=["001 003 005", "002 004 011"],
    #                 probe_height_m=[0.790, 0.790],
    #                 header_content=[example_header_content, example_header_content],
    #                 )
