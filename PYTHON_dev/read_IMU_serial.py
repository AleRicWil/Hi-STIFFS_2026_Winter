#!/usr/bin/env python3
"""
Hi-STIFFS 9-DOF IMU Real-Time Monitor & Logger
==============================================

Lightweight PyQt5 + pyqtgraph receiver for the binary packet stream
produced by 9DOF_IMU_WING_test.ino (Arduino Nano ESP32 + ISM330DHCX + LIS3MDL).

PURPOSE
-------
- Validate the minimal-overhead binary streaming path before integration
  into the full Hi-STIFFS tractor-mounted probe (RPi5 real-time data
  collection + 3D pose over rolling 1-2 s windows for stalk flexural
  stiffness drift correction).
- Provide immediate visual confirmation that all 9 axes are clean and
  correctly framed at the target IMU ODR.
- Log every packet with rich metadata so you can replay or post-process
  exactly the same raw integers the pose estimator will see.

PACKET FORMAT (exactly as sent by the sketch)
---------------------------------------------
sync (0xAA) | uint64_t timestamp_us | int16 gx,gy,gz,ax,ay,az | int16 mx,my,mz
Total: 27 bytes, little-endian.
struct.unpack("<B Q 9h", packet)  →  sync, ts_us, gx,gy,gz, ax,ay,az, mx,my,mz

KEY DESIGN CHOICES FOR EFFICIENCY & FIELD RELIABILITY
-----------------------------------------------------
- QTimer-based serial poll (5 ms) instead of blocking read or threads
  (sufficient and simpler at ≤1 kHz; scales cleanly when more sensors added).
- collections.deque with maxlen → O(1) append, automatic ring-buffer behavior
  for the rolling 1-2 s pose window without ever copying the whole buffer.
- Plot refresh on a separate 30-60 FPS QTimer → GUI stays responsive even if
  serial momentarily bursts; we only convert deques→np.array when we actually
  draw (not on every packet).
- Robust sync hunt + resync on 0xAA → survives USB glitches, long cables, or
  power-cycling the probe in the field without corrupting the CSV.
- Batched CSV flush every 50 packets + explicit close on exit → minimal I/O
  overhead while guaranteeing no data loss on abrupt shutdown.
- All raw int16 values preserved exactly as received. No conversion here;
  that belongs in the downstream pose / stiffness pipeline.
- argparse interface so the same script works headless in scripts or
  interactively during bring-up.

DEPENDENCIES (already in your Hi-STIFFS / ME 575 environment)
------------------------------------------------------------
pip install pyserial pyqtgraph PyQt5 numpy

USAGE (examples)
----------------
# Linux (most common for Nano ESP32)
python histiffs_imu_monitor.py --port /dev/ttyACM0 --window 2.0

# Windows (check Device Manager → Ports)
python histiffs_imu_monitor.py --port COM7 --window 1.5 --baud 1000000

# Specify your own log name (otherwise auto-timestamped file is created)
python histiffs_imu_monitor.py --port /dev/ttyACM0 --csv my_test_run.csv

The CSV will contain a full header block with sketch constants, date,
packet format description, and then the columnar data. You can load it
directly with numpy.loadtxt or pandas for later analysis / pose replay.

This script is the official bring-up / validation tool for the IMU
integration phase of Hi-STIFFS. Once data looks clean here we will
port the efficient receive + ring-buffer logic into the main RPi5
acquisition node that will run the 1-2 s pose windows in real time.

Author: Lead Engineer (Hi-STIFFS IMU / pose tracking)
Date:   2026-06-28
"""

import sys
import os
import time
import argparse
import struct
import csv
from datetime import datetime
from pathlib import Path
from collections import deque

import numpy as np
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg


# ====================== CONFIGURABLE DEFAULTS (match your sketch) ======================
DEFAULT_IMU_RATE_HZ = 6660          # from TARGET_IMU_RATE_HZ in the .ino
DEFAULT_MAG_RATE_HZ = 1000          # from TARGET_MAG_RATE_HZ in the .ino
DEFAULT_PLOT_WINDOW_S = 2.0        # exactly the rolling pose window horizon
DEFAULT_BAUD = 1000000             # Native USB CDC speed used by the sketch
PACKET_SIZE = 27                   # Fixed binary packet length
SYNC_BYTE = 0xAA


class IMUMonitor(QtWidgets.QMainWindow):
    """
    Main application window.
    Maintains:
      - Efficient serial receive + framing
      - Rolling deques sized for the 1-2 s pose window + headroom
      - Three pyqtgraph subplots (gyro / accel / mag) with auto-scrolling time axis
      - CSV writer with rich metadata header
    All hot-path operations are O(1) or amortized constant time.
    """

    def __init__(self, port: str, baud: int = DEFAULT_BAUD,
                 csv_path: str = None, plot_window_s: float = DEFAULT_PLOT_WINDOW_S):
        super().__init__()
        self.setWindowTitle("Hi-STIFFS | 9-DOF IMU Live Monitor (Raw Integers) — 1-2 s Pose Window")
        self.resize(1100, 900)

        self.port = port
        self.baud = baud
        self.plot_window_s = plot_window_s
        self.start_ts_us = None
        self.pkt_count = 0
        self.last_plot_update = 0.0

        # ====================== ROLLING BUFFERS (deque = efficient ring) ======================
        # maxlen gives automatic oldest-sample drop → perfect for 1-2 s rolling pose windows
        # Extra headroom (≈5-10 s) so we can still scroll smoothly while the view window is 2 s
        maxlen = int(plot_window_s * DEFAULT_IMU_RATE_HZ*2.1) + 200          # generous for testing up to ~1 kHz
        self.t_buf   = deque(maxlen=maxlen)
        self.gx_buf  = deque(maxlen=maxlen)
        self.gy_buf  = deque(maxlen=maxlen)
        self.gz_buf  = deque(maxlen=maxlen)
        self.ax_buf  = deque(maxlen=maxlen)
        self.ay_buf  = deque(maxlen=maxlen)
        self.az_buf  = deque(maxlen=maxlen)
        self.mx_buf  = deque(maxlen=maxlen)
        self.my_buf  = deque(maxlen=maxlen)
        self.mz_buf  = deque(maxlen=maxlen)

        # ====================== SERIAL + BYTE BUFFER ======================
        self.buffer = bytearray()
        try:
            self.ser = serial.Serial(port, baud, timeout=0)   # non-blocking
            print(f"[OK] Opened {port} @ {baud} baud")
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Serial Error",
                                           f"Could not open {port}:\n{e}\n\n"
                                           "Check cable, permissions (dialout group on Linux), "
                                           "or that the Arduino sketch is running.")
            sys.exit(1)

        # ====================== CSV SETUP (always on, field-safe) ======================
        if csv_path is None:
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            csv_path = f"histiffs_imu_{ts}.csv"
        self.csv_path = Path(csv_path)
        self.csv_file = open(self.csv_path, "w", newline="", buffering=8192)
        self.csv_writer = csv.writer(self.csv_file)
        self._write_csv_header()
        print(f"[LOG] Writing raw packets to: {self.csv_path.resolve()}")

        # ====================== UI LAYOUT ======================
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(4)

        # Status line
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

        # ====================== TIMERS ======================
        # Serial poll — lightweight, runs every 5 ms
        self.serial_timer = QtCore.QTimer()
        self.serial_timer.timeout.connect(self._poll_serial)
        self.serial_timer.start(5)

        # Plot refresh — 40 FPS target (GUI-smooth, not tied to packet rate)
        self.plot_timer = QtCore.QTimer()
        self.plot_timer.timeout.connect(self._update_plots)
        self.plot_timer.start(11)          # 40 Hz

        # Initial status
        self._update_status("Waiting for first valid packet...")

    # ------------------------------------------------------------------
    def _write_csv_header(self):
        """Write rich, machine- and human-readable metadata at top of CSV."""
        now = datetime.now().isoformat(timespec='seconds')
        self.csv_file.write(
            f"9-DOF IMU Binary Stream Log\n"
            f"Generated: {now}\n"
            f"Sketch: 9DOF_IMU_WING_test.ino (ISM330DHCX + LIS3MDL)\n"
            f"Target IMU ODR: {DEFAULT_IMU_RATE_HZ} Hz\n"
            f"Target MAG ODR: {DEFAULT_MAG_RATE_HZ} Hz\n"
            f"Packet format: sync(0xAA) + uint64_t ts_us + 6xint16 (gx gy gz ax ay az) + 3xint16 (mx my mz)\n"
            f"All values are RAW sensor integers exactly as received over USB.\n"
            f"Use datasheet sensitivity (Table 42/45 for IMU, Table 22 for MAG) for physical units.\n"
        )
        self.csv_writer.writerow(
            ['Time (sec)', 'Gyro X', 'Gyro Y', 'Gyro Z', 'Accel X', 'Accel Y', 'Accel Z', 'Mag X', 'Mag Y', 'Mag Z']
        )
        self.csv_file.flush()

    # ------------------------------------------------------------------
    def _poll_serial(self):
        """Non-blocking serial read + robust framing. Called from QTimer."""
        if not self.ser or not self.ser.is_open:
            return
        try:
            n = self.ser.in_waiting
            if n > 0:
                self.buffer.extend(self.ser.read(n))
                self._process_buffer()
        except Exception as e:
            self._update_status(f"Serial error: {e}")

    def _process_buffer(self):
        """Find sync bytes, unpack valid 27-byte packets, handle resync."""
        while len(self.buffer) >= PACKET_SIZE:
            if self.buffer[0] != SYNC_BYTE:
                # Resync hunt — critical for long field deployments
                idx = self.buffer.find(SYNC_BYTE)
                if idx == -1:
                    # No sync in buffer; keep last byte in case it starts a packet
                    if len(self.buffer) > 1:
                        self.buffer = self.buffer[-1:]
                    return
                else:
                    del self.buffer[:idx]
                    continue

            # Candidate packet
            pkt = self.buffer[:PACKET_SIZE]
            try:
                unpacked = struct.unpack('<B Q 9h', pkt)
                sync, ts_us, gx, gy, gz, ax, ay, az, mx, my, mz = unpacked

                if sync != SYNC_BYTE:
                    del self.buffer[0]
                    continue

                # Valid packet → handle it
                self._handle_new_packet(ts_us, gx, gy, gz, ax, ay, az, mx, my, mz)
                del self.buffer[:PACKET_SIZE]

            except struct.error:
                # Corrupt packet — advance one byte and try again
                del self.buffer[0]

    def _handle_new_packet(self, ts_us, gx, gy, gz, ax, ay, az, mx, my, mz):
        """Store sample, write CSV, update counters. Extremely lightweight."""
        if self.start_ts_us is None:
            self.start_ts_us = ts_us
        t_s = (ts_us - self.start_ts_us) / 1_000_000.0

        # Append to ring buffers (O(1)) for plotting
        self.t_buf.append(t_s)
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

        # CSV write (batched flush keeps I/O cheap)
        self.csv_writer.writerow([
            t_s, gx, gy, gz, ax, ay, az, mx, my, mz
        ])
        if self.pkt_count % 50 == 0:
            self.csv_file.flush()

        # Occasional status update (not every packet)
        if self.pkt_count % 100 == 0:
            self._update_status(latest_time_us=ts_us)

    # ------------------------------------------------------------------
    def _update_plots(self):
        """Called at ~40 Hz. Only convert deques→array when we actually draw."""
        if len(self.t_buf) < 2:
            return

        t_arr = np.fromiter(self.t_buf, dtype=np.float64)
        t_max = t_arr[-1]

        # Auto-scrolling rolling window (exactly what you need for 1-2 s pose tracking)
        x_min = max(0.0, t_max - self.plot_window_s)
        self.gyro_plot.setXRange(x_min, t_max, padding=0)

        # Update all nine curves (very cheap setData)
        self.gx_curve.setData(t_arr, np.fromiter(self.gx_buf, dtype=np.int16))
        self.gy_curve.setData(t_arr, np.fromiter(self.gy_buf, dtype=np.int16))
        self.gz_curve.setData(t_arr, np.fromiter(self.gz_buf, dtype=np.int16))

        self.ax_curve.setData(t_arr, np.fromiter(self.ax_buf, dtype=np.int16))
        self.ay_curve.setData(t_arr, np.fromiter(self.ay_buf, dtype=np.int16))
        self.az_curve.setData(t_arr, np.fromiter(self.az_buf, dtype=np.int16))

        self.mx_curve.setData(t_arr, np.fromiter(self.mx_buf, dtype=np.int16))
        self.my_curve.setData(t_arr, np.fromiter(self.my_buf, dtype=np.int16))
        self.mz_curve.setData(t_arr, np.fromiter(self.mz_buf, dtype=np.int16))

    def _update_status(self, extra: str = "", latest_time_us: float=0.0):
        if self.start_ts_us is None:
            rate = 0.0
        else:
            elapsed = (latest_time_us - self.start_ts_us) / 1e6
            rate = self.pkt_count / max(elapsed, 0.001)
        msg = (f"Packets: {self.pkt_count:6d} | "
               f"Rate: {rate:.1f} Hz | "
               f"Window: {self.plot_window_s:.1f} s | "
               f"Port: {self.port}")
        if extra:
            msg += f" | {extra}"
        self.status_label.setText(msg)

    # ------------------------------------------------------------------
    def closeEvent(self, event):
        """Guarantee CSV is flushed and closed cleanly — essential for field logs."""
        print("\n[SHUTDOWN] Closing serial and flushing CSV...")
        if hasattr(self, 'ser') and self.ser:
            self.ser.close()
        if hasattr(self, 'csv_file') and self.csv_file:
            self.csv_file.flush()
            self.csv_file.close()
            print(f"[OK] Log saved to {self.csv_path.resolve()}")
        event.accept()


# ====================== ENTRY POINT ======================
def main():
    parser = argparse.ArgumentParser(
        description="Hi-STIFFS 9-DOF IMU real-time monitor (raw integers, rolling 1-2 s pose window)"
    )
    parser.add_argument("--port", default="COM13",
                        help="Serial port (e.g. /dev/ttyACM0 or COM7)")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD,
                        help="Baud rate (default 1000000 for ESP32 native USB)")
    parser.add_argument("--window", type=float, default=DEFAULT_PLOT_WINDOW_S,
                        help="Rolling time window in seconds for pose tracking view (default 2.0)")
    parser.add_argument("--csv", default=None,
                        help="Optional explicit CSV filename (default = auto timestamped)")
    args = parser.parse_args()

    # Optional: quick sanity check that port looks plausible
    if not (args.port.startswith("/dev/") or args.port.upper().startswith("COM")):
        print("Warning: port name looks unusual. Double-check with ls /dev/tty* or Device Manager.")

    app = QtWidgets.QApplication(sys.argv)
    win = IMUMonitor(args.port, args.baud, args.csv, args.window)
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    # Workaround so the script also runs when pasted into some IDE consoles
    import serial  # ensure pyserial is importable before main()
    main()