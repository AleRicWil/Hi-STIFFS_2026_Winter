# Hi-STIFFS_GUI.py
# This script implements a cross-platform GUI for the Hi-STIFFS Operator Control system using PyQt5.
# It is designed to run seamlessly on Windows 10/11, Ubuntu Linux, and Raspberry Pi 5 with touchscreen support,
# without any OS-specific code changes. PyQt5 ensures consistent widget behavior, sizing, and event handling across platforms.
# The GUI serves as the human access point for processing data, with integration to 'process2.py' for data handling.
# Touchscreen optimizations include larger minimum widget sizes for finger taps (e.g., buttons >=60px height) and
# explicit size controls to prevent small, hard-to-tap elements. Scaling is applied dynamically for different screen sizes.

# Standard Libraries
import sys
import os
import json
import re
import calendar
from datetime import date

# Installed packages
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtWidgets import *  # Imports all widgets for concise usage (as per user preference)
from PyQt5.QtGui import QPalette, QColor, QFont
from PyQt5.QtCore import Qt

# Project source code
from process import HiSTIFFSData, run_stiffness_pipeline
from config import Config
import collect_data as cd  # Import at top for cross-platform module access via PyQt5/Python

RAW_DATA_BASE = Config.RAW_DATA_BASE
SETTINGS_FILE = 'Cross-Platform_GUI_Settings.json'  # File for storing user preferences, cross-platform compatible via JSON.

class SettingsDialog(QDialog):
    # Dialog for user preferences, including plot modes and screen sizing.
    # Scaling is applied from the parent MainWindow to ensure consistent appearance on all platforms.
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Preferences")
        layout = QFormLayout(self)

        # Checkbox for plot mode: independent windows vs. embedded (cross-platform, as Matplotlib and Qt handle both).
        self.independent_plots_checkbox = QCheckBox("Use independent plot windows (like standard Matplotlib)")
        layout.addRow("Plot Mode:", self.independent_plots_checkbox)

        # Checkbox for automatic screen detection (uses QApplication.primaryScreen(), consistent across OSes).
        self.detect_screen_size_checkbox = QCheckBox("Use detected screen size")
        layout.addRow("Screen Sizing: ", self.detect_screen_size_checkbox)

        # Spinbox for manual screen width override, with range suitable for common resolutions.
        self.screen_width_spinbox = QSpinBox()
        self.screen_width_spinbox.setRange(800, 3840)
        self.screen_width_spinbox.setSingleStep(100)
        layout.addRow("Screen Width Override:", self.screen_width_spinbox)

        # New: Hardware config for ADC and DAQ (editable lines for last two header rows)
        self.adc_config_edit = QLineEdit()
        layout.addRow("ADC Config:", self.adc_config_edit)
        self.daq_config_edit = QLineEdit()
        layout.addRow("DAQ Config:", self.daq_config_edit)

        # New: Sensor list editor (QTableWidget for editing sensors)
        self.sensor_table = QTableWidget(0, 6)  # Columns: SN, Label, Length, Spacing, Saturation Load, Saturation Microstrain
        self.sensor_table.setHorizontalHeaderLabels(["Serial#", "Default Label", "Length (mm)", "Spacing (mm)", "Sat. Load (N)", "Sat. Microstrain"])
        layout.addRow("Sensors:", self.sensor_table)
        add_sensor_btn = QPushButton("Add Sensor")
        add_sensor_btn.clicked.connect(self.add_sensor_row)
        layout.addRow(add_sensor_btn)

        # Dialog buttons for save/cancel, standard Qt behavior.
        buttons = QDialogButtonBox(QDialogButtonBox.Save | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addRow(buttons)

        # Load current settings from JSON file.
        settings = load_settings()
        self.independent_plots_checkbox.setChecked(settings['ui_preferences'].get('independent_plots', False))
        self.detect_screen_size_checkbox.setChecked(settings['ui_preferences'].get('detect_screen_size', True))
        self.screen_width_spinbox.setValue(settings['ui_preferences'].get('screen_width', 1920))
        self.adc_config_edit.setText(settings['hardware_config'].get('adc_config', ""))
        self.daq_config_edit.setText(settings['hardware_config'].get('daq_config', ""))
        self.load_sensors(settings.get('sensors', []))  # Unchanged

        # Apply scaling from ancestor MainWindow if available (ensures dialog matches main app scale on all platforms).
        if parent:
            current = parent
            while current and not hasattr(current, 'update_all_widgets'):
                current = current.parent()
            if current and hasattr(current, 'update_all_widgets'):
                current.update_all_widgets(self, current.screen_scale)

    def add_sensor_row(self):
        row = self.sensor_table.rowCount()
        self.sensor_table.insertRow(row)
        for col in range(6):
            self.sensor_table.setItem(row, col, QTableWidgetItem(""))

    def load_sensors(self, sensors):
        self.sensor_table.setRowCount(len(sensors))
        for row, sensor in enumerate(sensors):
            self.sensor_table.setItem(row, 0, QTableWidgetItem(sensor.get('sn', '')))
            self.sensor_table.setItem(row, 1, QTableWidgetItem(sensor.get('label', '')))
            self.sensor_table.setItem(row, 2, QTableWidgetItem(sensor.get('length', '')))
            self.sensor_table.setItem(row, 3, QTableWidgetItem(sensor.get('spacing', '')))
            self.sensor_table.setItem(row, 4, QTableWidgetItem(sensor.get('saturation_load', '')))
            self.sensor_table.setItem(row, 5, QTableWidgetItem(sensor.get('saturation_microstrain', '')))

    def get_sensors(self):
        sensors = []
        for row in range(self.sensor_table.rowCount()):
            sensor = {
                'sn': self.sensor_table.item(row, 0).text(),
                'label': self.sensor_table.item(row, 1).text(),
                'length': self.sensor_table.item(row, 2).text(),
                'spacing': self.sensor_table.item(row, 3).text(),
                'saturation_load': self.sensor_table.item(row, 4).text(),
                'saturation_microstrain': self.sensor_table.item(row, 5).text()
            }
            if sensor['sn']:  # Skip empty rows
                sensors.append(sensor)
        return sensors

class HardwareConfigDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Hardware Configuration")
        self.main_layout = QGridLayout(self)

        self.settings = load_settings()
        self.sensors = self.settings.get('sensors', [])
        self.adc_config = self.settings['hardware_config'].get('adc_config', "")
        self.daq_config = self.settings['hardware_config'].get('daq_config', "")

        # Number of Sensors
        self.num_sensors_spin = QSpinBox()
        self.num_sensors_spin.setRange(1, 5)
        self.num_sensors_spin.valueChanged.connect(self.update_sensor_fields)
        self.main_layout.addWidget(QLabel("# of ICB-Sensors:"), 4, 0)
        self.main_layout.addWidget(self.num_sensors_spin, 4, 1)

        # Sensor assignment - container will be created dynamically in update_sensor_fields
        self.sensor_container = None
        self.sensor_layout = None  # Will be recreated each time

        # Edit Sensor List button
        edit_sensors_btn = QPushButton("Edit Sensor List")
        edit_sensors_btn.clicked.connect(self.edit_sensors)
        self.main_layout.addWidget(edit_sensors_btn, 6, 0, 1, 2, Qt.AlignCenter)

        # Hardware config button
        edit_hardware_btn = QPushButton("Edit Hardware Config")
        edit_hardware_btn.clicked.connect(self.edit_hardware)
        self.main_layout.addWidget(edit_hardware_btn, 7, 0, 1, 2, Qt.AlignCenter)

        # Save/Cancel Buttons
        buttons = QDialogButtonBox(QDialogButtonBox.Save | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        self.main_layout.addWidget(buttons, 8, 0, 1, 2, Qt.AlignHCenter)

        # Load defaults
        self.settings = load_settings()
        self.sensors = self.settings.get('sensors', [])
        self.adc_config = self.settings.get('adc_config', "")
        self.daq_config = self.settings.get('daq_config', "")

        # Set default or last sensor config (block signals to prevent double update)
        last_num = self.settings['session_state'].get('last_num_sensors', 5)
        self.num_sensors_spin.blockSignals(True)  # Prevent valueChanged trigger
        self.num_sensors_spin.setValue(last_num)
        self.num_sensors_spin.blockSignals(False)  # Re-enable
        self.update_sensor_fields(last_num)  # Explicit initial population

        # Initial update
        self.update_test_fields(self.test_type_combo.currentText())

    def update_test_fields(self, test_type):
        visible = "Cycle" in test_type
        # Toggle visibility (cross-platform via PyQt5 setVisible on widgets)
        self.test_number_label.setVisible(visible)
        self.test_number_spin.setVisible(visible)
        self.test_rest_label.setVisible(visible)
        self.test_rest_edit.setVisible(visible)
        self.other_edit.setVisible(test_type == "Other")
        # TODO: Add more dynamic fields based on type, e.g., Force for Force Cycle

    def update_sensor_fields(self, num):
        if len(self.sensors) < num:
            QMessageBox.warning(self, "Not Enough Sensors", f'Only {len(self.sensors)} sensor(s) stored, but {num} requested. Add more sensors to list.')
            return
        
        # Completely remove old container + widgets (prevents ANY ghosts/duplicates)
        if self.sensor_container is not None:
            self.main_layout.removeWidget(self.sensor_container)
            self.sensor_container.deleteLater()
            self.sensor_container = None
            self.sensor_layout = None

        # Create fresh container + layout
        self.sensor_container = QWidget()
        self.sensor_layout = QGridLayout(self.sensor_container)

        self.sensor_labels = []
        self.sensor_sns = []

        # Default labels and sorted sns
        default_labels = ['A', 'B', 'C', 'D', 'E'][:num]
        sorted_sns = sorted([s['sn'] for s in self.sensors if s['sn']], key=lambda x: int(x))[:num]

        # Last config if available and matches num
        last_labels = self.settings['session_state'].get('last_labels', default_labels)
        last_sns = self.settings['session_state'].get('last_sns', sorted_sns)
        use_last = len(last_labels) == num and len(last_sns) == num

        for i in range(num):
            label_combo = QComboBox()
            label_combo.addItems(["A", "B", "C", "D", "E"])
            label_combo.setCurrentText(last_labels[i] if use_last else default_labels[i])
            self.sensor_layout.addWidget(QLabel(f"Sensor {i+1} Label:"), i, 0)
            self.sensor_layout.addWidget(label_combo, i, 1)
            sn_combo = QComboBox()
            sn_combo.addItems([s['sn'] for s in self.sensors])
            sn_combo.setCurrentText(last_sns[i] if use_last else sorted_sns[i])
            self.sensor_layout.addWidget(QLabel("Serial#:"), i, 2)
            self.sensor_layout.addWidget(sn_combo, i, 3)
            
            self.sensor_labels.append(label_combo)
            self.sensor_sns.append(sn_combo)

        # Add the fresh container to main layout
        self.main_layout.addWidget(self.sensor_container, 5, 0, 1, 2)

        # Force full layout/paint refresh (critical for eliminating ghosts/duplicates)
        self.sensor_layout.activate()
        self.sensor_container.adjustSize()
        self.sensor_container.update()
        self.update()  # Update the dialog itself

    def accept(self):
        labels = [combo.currentText() for combo in self.sensor_labels]
        sns = [combo.currentText() for combo in self.sensor_sns]
        if len(set(labels)) != len(labels) or len(set(sns)) != len(sns):
            QMessageBox.warning(self, "Duplicate Error", "Sensor labels and serial numbers must be unique.")
            return

        # Save last config (cross-platform settings persistence via JSON)
        self.settings['session_state']['last_num_sensors'] = self.num_sensors_spin.value()
        self.settings['session_state']['last_labels'] = labels
        self.settings['session_state']['last_sns'] = sns
        save_settings(self.settings)

        super().accept()

    def edit_sensors(self):
        dialog = SettingsDialog(self)
        if dialog.exec_() == QDialog.Accepted:
            self.settings['sensors'] = dialog.get_sensors()
            save_settings(self.settings)
            self.sensors = self.settings['sensors']
            self.update_sensor_fields(self.num_sensors_spin.value())

    def edit_hardware(self):
        dialog = SettingsDialog(self)
        if dialog.exec_() == QDialog.Accepted:
            self.settings['hardware_config']['adc_config'] = dialog.adc_config_edit.text()
            self.settings['hardware_config']['daq_config'] = dialog.daq_config_edit.text()
            save_settings(self.settings)
            self.adc_config = self.settings['hardware_config']['adc_config']
            self.daq_config = self.settings['hardware_config']['daq_config']

    def get_header_content(self):
        '''
        Creates header for test file. This is of the following format:

        Test Name: duplicates filename within metadata [assigned within collect_data:__ini__()]
        Note: an optional note provided by the user at start of test
        Test Type: the type of test the probe is being used to complete. Dynamically chooses additional parameters
        Number of ICB-Sensors: [1-10], Sensor Label(s): which sensors in probe ara active [A-J]
        Sensor Info Section:
            Each sensor's serial#, parameters, and calibration
        System Info Section:
            Relevant parameters of the Data Acquistion system
        '''
        
        header = []

        # user note
        note = self.note_edit.text().strip()
        if note: header.append(f"Note: {note}")
        else: header.append(f"Note:")

        # test type
        test_type = self.test_type_combo.currentText()
        if test_type == "Other":
            custom = self.other_edit.text().strip()
            if custom and custom not in self.custom_types:
                self.custom_types.append(custom)
                self.settings['test_config']['custom_test_types'] = self.custom_types
                save_settings(self.settings)
            test_type = custom
        params_str = f"Test Type: {test_type}"
        header.append(params_str)

        if "Cycle" in test_type:
            header.append(f"Test Number in Session: {self.test_number_spin.value()}, Time since Last Test: {self.test_rest_edit.text()}")

        # number of sensors and labels
        labels = ' '.join([combo.currentText() for combo in self.sensor_labels])
        serial_nums = ' '.join([combo.currentText() for combo in self.sensor_sns])
        header.append(f"Number of ICB-Sensors: {self.num_sensors_spin.value()}, Sensor Label(s): {labels}, Sensor Serial#(s): {serial_nums}")

        # sensor info
        for i, l in enumerate([combo.currentText() for combo in self.sensor_labels]):
            sn = [combo.currentText() for combo in self.sensor_sns][i]
            sensor = next((s for s in self.sensors if s['sn'] == sn), {})
            header.append(f"ICB-Sensor {l}'s Serial#: {sn}, Length (mm): {sensor.get('length', '')}, Spacing (mm): {sensor.get('spacing', '')}, Saturation Load (N): {sensor.get('saturation_load', '')}, Microstrain at Saturation: {sensor.get('saturation_microstrain', '')}")

        # system info
        self.adc_config = self.settings['hardware_config'].get('adc_config', "")
        self.daq_config = self.settings['hardware_config'].get('daq_config', "")
        header.append(self.adc_config)
        header.append(self.daq_config)
        return header

class HomePage(QWidget):
    # Home page widget: displays welcome and entry button to processing page.
    # Widgets are sized initially for touch-friendliness, scaled later.
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)

        # Welcome label, centered, with font set via scaling later.
        welcome_label = QLabel("Hi-STIFFS Control 2026 v1.0")
        welcome_label.setAlignment(QtCore.Qt.AlignCenter)
        layout.addWidget(welcome_label)

        # Collect Data button (new, added just above Process)
        collect_button = QPushButton("Collect Data")
        collect_button.setObjectName("collect_button")
        collect_button.clicked.connect(parent.go_to_collect)
        collect_button.setMinimumSize(parent.base_button_width * 2, parent.base_button_height)
        layout.addWidget(collect_button, alignment=Qt.AlignHCenter)

        # Process button: main entry point, sized for touch (wide for prominence).
        process_button = QPushButton("Process Existing Data")
        process_button.setObjectName("process_button")  # Unique ID for targeted scaling.
        process_button.clicked.connect(parent.go_to_processing)
        process_button.setMinimumSize(parent.base_button_width * 2, parent.base_button_height)  # Initial touch-friendly size.
        layout.addWidget(process_button, alignment=Qt.AlignHCenter)  # Centers button horizontally to prevent full-width spanning (cross-platform via Qt alignment).
        base_spacer_height = 100  # Base height in pixels; scale for touch-friendliness.
        spacer = QSpacerItem(20, int(base_spacer_height * parent.screen_scale), QSizePolicy.Minimum, QSizePolicy.Fixed)  # Fixed vertical spacer to raise button (cross-platform pixel control via PyQt5).
        layout.addItem(spacer)

class CollectPage(QWidget):
    # Overhauled CollectPage per user spec.
    # Inline fields for quick start (Note, Test Type, Probe Height, Speed).
    # Number of sensors + SN assignment + hardware + sensor details live ONLY in Hardware Config popup.
    def __init__(self, parent=None):
        super().__init__(parent)
        self.mainwindow = parent
        self.collecting = False

        self.main_layout = QHBoxLayout(self)

        # === LEFT PANE: main configuration (QFormLayout for clean, touch-friendly layout) ===
        self.left_widget = QWidget()
        self.left_layout = QFormLayout(self.left_widget)
        self.left_layout.setFieldGrowthPolicy(QFormLayout.AllNonFixedFieldsGrow)

        # Optional Note
        self.note_edit = QLineEdit()
        self.note_edit.setPlaceholderText("Optional test note (e.g. Field test 25 ft/min)")
        self.left_layout.addRow("Note:", self.note_edit)

        # Test Type
        self.test_type_combo = QComboBox()
        settings = load_settings()
        test_types = ["Demo", "Lab", "Field", "Other"] + settings.get('test_config', {}).get('custom_test_types', [])
        self.test_type_combo.addItems(test_types)
        self.test_type_combo.currentTextChanged.connect(self.update_test_fields)
        self.left_layout.addRow("Test Type:", self.test_type_combo)

        self.other_test_edit = QLineEdit()
        self.other_test_edit.setPlaceholderText("Custom test type")
        self.other_test_edit.setVisible(False)
        self.left_layout.addRow("Custom Test:", self.other_test_edit)

        # Stalk Type
        self.stalk_type_combo = QComboBox()
        stalk_types = ['PVC-Hi', 'PVC-Med', 'Wood-Lo', 'Other']
        self.stalk_type_combo.addItems(stalk_types)
        self.stalk_type_combo.currentTextChanged.connect(self.update_test_fields)
        self.left_layout.addRow('Stalk Type:', self.stalk_type_combo)

        self.other_stalk_edit = QLineEdit()
        self.other_stalk_edit.setPlaceholderText("Custom stalk type")
        self.other_stalk_edit.setVisible(False)
        self.left_layout.addRow("Custom Stalk:", self.other_stalk_edit)

        # New fields requested by user
        self.probe_height_spin = QDoubleSpinBox()
        self.probe_height_spin.setRange(0.1, 2.5)
        self.probe_height_spin.setDecimals(3)
        self.probe_height_spin.setValue(0.785)
        self.probe_height_spin.setSingleStep(0.01)
        self.left_layout.addRow("Probe Height (m):", self.probe_height_spin)

        self.speed_edit = QLineEdit()
        self.speed_edit.setPlaceholderText("e.g. 25 ft/min")
        self.left_layout.addRow("Speed:", self.speed_edit)

        # Live header preview
        self.preview_text = QTextEdit()
        self.preview_text.setReadOnly(True)
        self.preview_text.setMaximumHeight(240)
        self.left_layout.addRow("Header Preview:", self.preview_text)
        
        self.preview_update_button = QPushButton('Update Preview')
        self.preview_update_button.setObjectName('preview_update_button')
        self.preview_update_button.clicked.connect(self.update_header_preview)
        self.preview_update_button.setMinimumSize(self.mainwindow.base_button_width, self.mainwindow.base_button_height)
        self.left_layout.addRow(self.preview_update_button)

        # Status label
        self.status_label = QLabel("Ready. Configure if needed and press 'Start'")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setWordWrap(True)
        self.left_layout.addRow(self.status_label)

        self.main_layout.addWidget(self.left_widget, stretch=6)

        # === RIGHT SIDE: buttons (unchanged layout, only button text updated) ===
        right_vbox = QVBoxLayout()

        # Today's test count readout - ultra-compact (small font + fixed tiny height)
        # Sits in the existing small gap at the top of the right column without shifting buttons
        self.test_count_label = QLabel("Tests done today: 0")
        self.test_count_label.setAlignment(Qt.AlignCenter)
        self.test_count_label.setObjectName("test_count_label")  # enables existing scaling

        # One or two sizes smaller than normal
        small_font = QFont(self.mainwindow.base_normal_font)
        small_font.setPointSize(int(self.mainwindow.small2_font_size * self.mainwindow.screen_scale))
        self.test_count_label.setFont(small_font)

        # Force minimal vertical footprint (cross-platform identical on Win/Ubuntu/RPi5)
        self.test_count_label.setFixedHeight(int(28 * self.mainwindow.screen_scale))
        self.test_count_label.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)

        right_vbox.addWidget(self.test_count_label, stretch=0)

        self.hardware_config = QPushButton("Hardware Config")
        self.hardware_config.setObjectName("config_button")   # keeps existing scaling
        self.hardware_config.clicked.connect(self.open_hardware_config)
        self.hardware_config.setMinimumSize(self.mainwindow.base_button_width, self.mainwindow.base_button_height)
        right_vbox.addWidget(self.hardware_config, stretch=1)

        self.start_button = QPushButton("Start")
        self.start_button.setObjectName("start_button")
        self.start_button.clicked.connect(self.start_collection)
        self.start_button.setMinimumSize(self.mainwindow.base_button_width, self.mainwindow.base_button_height)
        right_vbox.addWidget(self.start_button, stretch=2)

        self.stop_button = QPushButton("Stop")
        self.stop_button.setObjectName("stop_button")
        self.stop_button.clicked.connect(self.stop_collection)
        self.stop_button.setMinimumSize(self.mainwindow.base_button_width, self.mainwindow.base_button_height)
        self.stop_button.setEnabled(False)
        right_vbox.addWidget(self.stop_button, stretch=1)

        self.back_button = QPushButton("Back to Home")
        self.back_button.setObjectName("back_button")
        self.back_button.clicked.connect(parent.go_to_home)
        self.back_button.setMinimumSize(self.mainwindow.base_button_width, self.mainwindow.base_button_height)
        right_vbox.addWidget(self.back_button, stretch=1)

        self.main_layout.addLayout(right_vbox, stretch=1)

        self.load_session_defaults()
        self.update_test_count()

        # Collection monitor timer
        self.collection_timer = QtCore.QTimer(self)
        self.collection_timer.timeout.connect(self.monitor_collection)
        self.collection_timer.setInterval(1000)  # check every second

    def update_test_fields(self, test_type: str):
        """Dynamic visibility for cycle fields and Other field. Pure PyQt5 – cross-platform identical on all three OSes."""
 
        self.other_test_edit.setVisible(self.test_type_combo.currentText() == "Other")
        self.other_stalk_edit.setVisible(self.stalk_type_combo.currentText() == 'Other')
        self.update_header_preview()

    def open_hardware_config(self):
        """Opens the Deep Settings popup (HeaderConfigDialog) that now exclusively handles sensors, SNs, hardware, and number of sensors."""
        dialog = HardwareConfigDialog(self)
        if dialog.exec_() == QDialog.Accepted:
            self.update_header_preview()

    def load_session_defaults(self):
        """Load last-used values from JSON settings (cross-platform persistence)."""
        settings = load_settings()
        session = settings.get('session_state', {})
        self.note_edit.setText(session.get('last_note', ''))
        last_type = session.get('last_test_type', 'Demo')
        idx = self.test_type_combo.findText(last_type)
        if idx >= 0:
            self.test_type_combo.setCurrentIndex(idx)
        self.probe_height_spin.setValue(session.get('last_probe_height', 0.785))
        self.speed_edit.setText(session.get('last_speed', '25 ft/min'))
        self.update_test_fields(self.test_type_combo.currentText())
        self.update_header_preview()

    def save_session_state(self):
        """Persist current UI values for next session."""
        settings = load_settings()
        if 'session_state' not in settings:
            settings['session_state'] = {}
        session = settings['session_state']
        session['last_note'] = self.note_edit.text().strip()
        session['last_test_type'] = self.test_type_combo.currentText()
        session['last_probe_height'] = self.probe_height_spin.value()
        session['last_speed'] = self.speed_edit.text().strip()
        save_settings(settings)

    def build_header_content(self):
        """Builds the complete header list from main-page fields + deep settings from JSON.
        Guarantees DataReceiverWriter and process.py receive exactly the same format they expect."""
        header = []
        settings = load_settings()
        session = settings.get('session_state', {})

        note = self.note_edit.text().strip()
        header.append(f"Note: {note if note else ''}")

        test_type = self.test_type_combo.currentText()
        if test_type == "Other":
            custom = self.other_test_edit.text().strip()
            test_type = custom or "Other"
        header.append(f"Test Type: {test_type}")

        if "Cycle" in test_type:
            header.append(f"Test Number in Session: {self.test_number_spin.value()}, Time since Last Test: {self.test_rest_edit.text()}")

        stalk_type = self.stalk_type_combo.currentText()
        if stalk_type == "Other":
            custom = self.other_stalk_edit.text().strip()
            stalk_type = custom or "Other"
        header.append(f"Stalk Type: {stalk_type}")

        # New fields
        header.append(f"Probe Height (m): {self.probe_height_spin.value():.3f}")
        header.append(f"Speed: {self.speed_edit.text().strip()}")

        # Sensor & hardware info from Deep Settings
        num = session.get('last_num_sensors', 5)
        labels = session.get('last_labels', ['A','B','C','D','E'])[:num]
        sns = session.get('last_sns', [])[:num]
        if len(sns) < num:
            sns += [''] * (num - len(sns))

        header.append(f"Number of ICB-Sensors: {num}, Sensor Label(s): {' '.join(labels)}, Sensor Serial#(s): {' '.join(sns)}")

        sensors = settings.get('sensors', [])
        for i, l in enumerate(labels):
            sn = sns[i] if i < len(sns) else ''
            sensor = next((s for s in sensors if s.get('sn') == sn), {})
            header.append(f"ICB-Sensor {l}'s Serial#: {sn}, Length (mm): {sensor.get('length', '')}, "
                         f"Spacing (mm): {sensor.get('spacing', '')}, Saturation Load (N): {sensor.get('saturation_load', '')}, "
                         f"Microstrain at Saturation: {sensor.get('saturation_microstrain', '')}")

        hw = settings.get('hardware_config', {})
        header.append(hw.get('adc_config', "Analog-to-Digital Converter: ADS1220, Mode: Turbo, Data Rate: DR_90SPS, Analog Excitation/Reference Voltage: 5.1V +/-2mV"))
        header.append(hw.get('daq_config', "DAQ Microcontroller: Arduino Nano ESP32, ID: Hi-STIFFS_Nano_01, CPU Clock: 240MHz, Cores: 2, Data-stream Connection: Wi-Fi"))

        return header

    def update_header_preview(self):
        """Live preview of metadata that will be written to CSV."""
        try:
            content = self.build_header_content()
            self.preview_text.setPlainText('\n'.join(content))
        except Exception:
            self.preview_text.setPlainText("Preview unavailable. Open Hardware Config first")

    def update_test_count(self):
        """Count all .csv files in today's Raw Data folder (no subfolders)."""
        today_str = date.today().isoformat()          # e.g. 2026-05-11
        date_folder = Config.RAW_DATA_BASE / today_str
        if date_folder.exists() and date_folder.is_dir():
            csv_count = sum(1 for f in date_folder.iterdir()
                            if f.is_file() and f.suffix.lower() == '.csv')
        else:
            csv_count = 0
        self.test_count_label.setText(f"Tests done today: {csv_count}")

    def start_collection(self):
        """Directly instantiates DataReceiverWriter + RealTimePlotWindow (mirrors original collect_data.py architecture).
        Fully cross-platform: uses the same PyQt5 QApplication context and classes that behave identically on Windows 10/11, Ubuntu Linux, and Raspberry Pi 5 touchscreen."""
        if self.collecting:
            return

        self.save_session_state()
        self.header_content = self.build_header_content()

        # Pull current session config (already persisted by Deep Settings)
        settings = load_settings()
        session = settings.get('session_state', {})
        num_sensors = session.get('last_num_sensors', 5)
        sensor_labels = session.get('last_labels', ['A', 'B', 'C', 'D', 'E'])[:num_sensors]
        sensor_sns = session.get('last_sns', [])[:num_sensors]
        if len(sensor_sns) < num_sensors:
            sensor_sns += ['unknown'] * (num_sensors - len(sensor_sns))

        self.collecting = True
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.status_label.setText("COLLECTING DATA... (press SPACE in plot window or Stop button)")

        try:
            # Direct object creation - identical to original run_collection() flow
            self.ReadWrite = cd.DataReceiverWriter(
                num_sensors=num_sensors,
                sensor_labels=sensor_labels,
                sensor_sns=sensor_sns,
                header_content=self.header_content
            )

            self.plot_window = cd.RealTimePlotWindow(
                self.ReadWrite,
                num_sensors,
                sensor_labels
            )

            self.ReadWrite.start()   # starts the TCP receiver / CSV writer thread
            self.collection_timer.start()

        except Exception as e:
            QMessageBox.critical(self, "Collection Error", f"Failed to start collection:\n{str(e)}")
            self.stop_collection()

    def monitor_collection(self):
        """
        Polls self.ReadWrite.running every second.
        When the plot window closes the collection (SPACE / STOP button), this auto-calls stop_collection().
        """
        if hasattr(self, 'ReadWrite') and not self.ReadWrite.running:
            self.collection_timer.stop()
            self.stop_collection()
        

    def stop_collection(self):
        """Clean shutdown of collection objects (called by Stop button or plot window close)."""
        if hasattr(self, 'ReadWrite') and self.ReadWrite is not None:
            self.ReadWrite.running = False
        if hasattr(self, 'plot_window') and hasattr(self.plot_window, 'stop_collection'):
            self.plot_window.stop_collection()

        self.collecting = False
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.status_label.setText("Collection stopped. Ready for next test.")
        self.update_test_count()


class FilePage(QWidget):
    # File selection page: allows browsing, year selection, tree view of data, and plotting.
    # Includes dynamic elements like combo box, with initial sizes for touch.
    def __init__(self, parent=None):
        super().__init__(parent)
        self.mainwindow = parent  # Reference to MainWindow for scaling and navigation.
        self.data_files = {}
        self.current_base = None

        # Main horizontal layout for left (tree) and right (buttons) sections.
        self.main_layout = QHBoxLayout(self)

        # Left vertical layout for year controls and tree (takes 6/7 width via stretch).
        left_vbox = QVBoxLayout()
        self.year_layout = QHBoxLayout()
        left_vbox.addLayout(self.year_layout)
        
        # Tree widget for displaying months/days/times, with touch-friendly item heights via stylesheet.
        self.tree = QTreeWidget()
        self.tree.setObjectName("data_tree")  # Unique ID for scaling.
        self.tree.setColumnCount(2)  # Two columns, each to be sized at 3/7 of window width (cross-platform via PyQt5 column management).
        self.tree.setHeaderHidden(True)
        self.tree.setSelectionMode(QAbstractItemView.SingleSelection)
        # Initial stylesheet for item height (cross-platform, Qt stylesheets work identically on Windows/Linux/Pi).
        self.tree.setStyleSheet(f"QTreeWidget::item {{ height: {self.mainwindow.base_tree_item_height}px; }}")
        left_vbox.addWidget(self.tree)
        
        self.main_layout.addLayout(left_vbox, stretch=6)  # Allocates 6/7 of horizontal space to left section (cross-platform layout proportion via PyQt5 stretch factors).

        # Right vertical layout for buttons (takes 1/7 width via stretch).
        right_vbox = QVBoxLayout()

        # Plot button: triggers plotting of selected data.
        plot_button = QPushButton("Plot Selected")
        plot_button.setObjectName("plot_button")  # Unique ID.
        plot_button.clicked.connect(self.plot_selected)
        plot_button.setMinimumSize(self.mainwindow.base_button_width, self.mainwindow.base_button_height)  # Touch size.
        right_vbox.addWidget(plot_button, stretch=2)  # Takes 2/3 vertical space in right column (cross-platform via PyQt5 stretch).

        # Process Stiffness button: triggers pipeline to calculate and save stiffness of detected stalks
        stiff_button = QPushButton("Process Selected")
        stiff_button.setObjectName("stiff_button")
        stiff_button.clicked.connect(self.process_selected)
        stiff_button.setMinimumSize(self.mainwindow.base_button_width, self.mainwindow.base_button_height)
        right_vbox.addWidget(stiff_button, stretch=2)

        # Back button: returns to home.
        back_button = QPushButton("Back to Home")
        back_button.setObjectName("back_button")  # Unique ID.
        back_button.clicked.connect(parent.go_to_home)
        back_button.setMinimumSize(self.mainwindow.base_button_width, self.mainwindow.base_button_height)  # Base size, adjusted later.
        right_vbox.addWidget(back_button, stretch=1)  # Takes 1/3 vertical space, half the size of plot button (cross-platform via PyQt5 stretch).

        self.main_layout.addLayout(right_vbox, stretch=1)  # Allocates 1/7 of horizontal space to right section.

        # Browse button: opens folder dialog (QFileDialog is cross-platform).
        self.browse_button = QPushButton("Browse Folder")
        self.browse_button.setObjectName("browse_button")  # Unique ID.
        self.browse_button.clicked.connect(self.browse_folder)
        self.browse_button.setMinimumSize(self.mainwindow.base_button_width, self.mainwindow.menubar_height)  # Compact.

        # Path label: displays current folder, right-aligned.
        self.path_label = QLabel()
        self.path_label.setObjectName("path_label")
        self.path_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.path_label.setWordWrap(False)  # No wrapping for clean display.

    def browse_folder(self):
        # Opens cross-platform folder dialog to select data directory.
        print('Waiting for user to select folder...')
        print(self.current_base)
        print(self.mainwindow.settings['paths'].get('raw_data_folder', RAW_DATA_BASE))
        try:
            folder_path = QFileDialog.getExistingDirectory(
                        self.mainwindow,
                        "Select Raw Data Folder")
        except:
            print('Bad browse attempt')
        print('Checking good folder path...')
        if folder_path:
            self.load_data(folder_path)
            if self.data_files:
                self.mainwindow.settings['paths']['raw_data_folder'] = folder_path
                save_settings(self.mainwindow.settings)

    def load_data(self, folder_path):
        # Loads data files from folder, populates year combo and tree.
        # Dynamic combo box is created here with initial size for touch.
        self.current_base = folder_path
        self.data_files = self.collect_data_files(folder_path)
        while self.year_layout.count():
            item = self.year_layout.takeAt(0)
            if item.widget():
                item.widget().setParent(None)
        self.tree.clear()
        sorted_years = sorted(self.data_files.keys())
        if sorted_years:
            # Year combo: dynamic, with objectName for scaling.
            combo = QComboBox()
            combo.setObjectName("year_combo")  # Unique ID for targeted updates.
            combo.addItems(sorted_years)
            combo.setCurrentIndex(len(sorted_years) - 1)  # Default to latest year.
            combo.currentTextChanged.connect(self.show_months)
            combo.setMinimumSize(self.mainwindow.base_button_width * 2, self.mainwindow.menubar_height)  # Initial wide touch size.
            self.year_layout.addWidget(combo)
        self.year_layout.addStretch()  # Aligns combo left.
        self.year_layout.addWidget(self.path_label)
        self.year_layout.addWidget(self.browse_button)  # Browse in top right.
        self.path_label.setText(folder_path if folder_path else "No folder selected")
        self.mainwindow.update_scaling()  # Apply scaling after adding dynamic widgets (cross-platform).
        if sorted_years:
            self.show_months(sorted_years[-1])
        else:
            QMessageBox.warning(self, "No Files", "No raw data files found in the selected folder or subfolders.")

    def collect_data_files(self, base_path):
        # Walks directory to collect CSV files matching pattern, organizes by year/month/day/time.
        # Uses os.walk, which is cross-platform for file system traversal.
        data_files = {}
        pattern = r'(\d{4}-\d{2}-\d{2})_(\d{6})_(\d{2})\.csv'
        for root, dirs, files in os.walk(base_path):
            date = os.path.basename(root)
            if re.match(r'\d{4}-\d{2}-\d{2}', date):
                year = date[:4]
                month_num = date[5:7]
                month = calendar.month_abbr[int(month_num)]
                day = date[8:]
                if year not in data_files:
                    data_files[year] = {}
                if month not in data_files[year]:
                    data_files[year][month] = {}
                if day not in data_files[year][month]:
                    data_files[year][month][day] = []
                for f in files:
                    match = re.match(pattern, f)
                    if match and match.group(1) == date:
                        time_str = match.group(2)
                        data_files[year][month][day].append((date, time_str))
        # Sort times for each day.
        for year in data_files:
            for month in data_files[year]:
                for day in data_files[year][month]:
                    data_files[year][month][day].sort(key=lambda x: x[1])
        return data_files

    def show_months(self, year):
        # Populates tree with months/days/times for selected year.
        # Expands/collapses based on data volume for usability (touch-friendly with larger items).
        self.tree.clear()
        sorted_months = sorted(self.data_files[year].keys(), key=lambda m: list(calendar.month_abbr).index(m))
        total_days = 0
        for month in sorted_months:
            sorted_days = sorted(self.data_files[year][month].keys(), key=int)
            total_days += len(sorted_days)
            num_days = len(sorted_days)
            month_item = QTreeWidgetItem([month, f"{num_days} days"])  # Col0: month, Col1: details (cross-platform multi-column tree via PyQt5).
            for day in sorted_days:
                num_times = len(self.data_files[year][month][day])
                day_item = QTreeWidgetItem([day, f"{num_times} files"])  # Col0: day, Col1: details.
                for full_date, t in self.data_files[year][month][day]:
                    formatted_t = f"{t[:2]}:{t[2:4]}:{t[4:]}"
                    child = QTreeWidgetItem([formatted_t, full_date])  # Col0: time, Col1: date.
                    child.setData(0, QtCore.Qt.UserRole, (full_date, t))
                    day_item.addChild(child)
                month_item.addChild(day_item)
            self.tree.addTopLevelItem(month_item)
        num_months = len(sorted_months)
        if total_days < 20 and num_months <= 4:
            for i in range(self.tree.topLevelItemCount()):
                self.tree.expandItem(self.tree.topLevelItem(i))
        else:
            self.tree.collapseAll()

    def plot_selected(self):
        # Plots selected data using HiSTIFFSData from process2.py.
        # Handles independent or embedded plots based on settings (Matplotlib/Qt integration is cross-platform).
        selected_items = self.tree.selectedItems()
        if not selected_items:
            QMessageBox.warning(self, "Selection Error", "Please select a time.")
            return
        item = selected_items[0]
        if not item.parent():
            QMessageBox.warning(self, "Selection Error", "Please select a time, not a month.")
            return
        if not item.parent().parent():
            QMessageBox.warning(self, "Selection Error", "Please select a time, not a day.")
            return
        data_tuple = item.data(0, QtCore.Qt.UserRole)
        if data_tuple is None:
            QMessageBox.warning(self, "Selection Error", "Invalid selection. Please select a valid time entry.")
            return
        date, time = data_tuple
        
        try:
            data = HiSTIFFSData(date, time)
            if not data.exists:
                QMessageBox.warning(self, "Data Error", "Failed to load data.")
                return

            settings = load_settings()
            sensors_str = ','.join(data.sensor_labels)
            if settings['ui_preferences'].get('independent_plots', False):
                data.plot_force_position(sensors=sensors_str)
                plt.show()
            else:
                self.embed_plots(data, sensors_str)
        except ValueError as e:
            QMessageBox.critical(self, "Value Error", f"Failed to process or plot the selected data: {str(e)}")
        except Exception as e:
            QMessageBox.critical(self, "Unexpected Error", f"An unexpected error occurred: {str(e)}")
    
    def process_selected(self):
        selected_items = self.tree.selectedItems()
        if not selected_items:
            QMessageBox.warning(self, "Selection Error", "Please select a file")
            return
        item = selected_items[0]
        if not item.parent():
            QMessageBox.warning(self, "Selection Error", "Please select a time, not a month.")
            return
        if not item.parent().parent():
            QMessageBox.warning(self, "Selection Error", "Please select a time, not a day.")
            return

        data_tuple = item.data(0, QtCore.Qt.UserRole)
        if data_tuple is None:
            QMessageBox.warning(self, "Selection Error", "Invalid selection. Please select a valid time entry.")
            return
        date, time = data_tuple

        try:
            data = HiSTIFFSData(date, time)
            if not data.exists:
                QMessageBox.warning(self, "Data Error", "Failed to load data.")
                return

            settings = load_settings()
            sensors_str = ','.join(data.sensor_labels)

            run_stiffness_pipeline(data)
            QMessageBox.about(self, 'Status', f'Wrote results to {data.results_path}')
            
        except ValueError as e:
            QMessageBox.critical(self, "Value Error", f"Failed to process or plot the selected data: {str(e)}")
        except Exception as e:
            QMessageBox.critical(self, "Unexpected Error", f"An unexpected error occurred: {str(e)}")

    def embed_plots(self, data, sensors_str):
        # Embeds plots in a scrollable dialog (uses Qt for layout, Matplotlib for figures; cross-platform).
        data.describe_channels()
        data.filter_channels()
        figs = data.plot_raw_strains(sensors=sensors_str, return_figs=True)  # Assume this returns list of figs

        plot_dialog = QDialog(self)
        plot_dialog.setWindowTitle("Embedded Raw Strain Plots")
        layout = QVBoxLayout(plot_dialog)
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_widget = QWidget()
        scroll_layout = QVBoxLayout(scroll_widget)
        layout.addWidget(scroll_area)
        scroll_area.setWidget(scroll_widget)

        for fig in figs:
            # Scale figure size based on screen scale (Matplotlib with Qt backend handles resizing cross-platform).
            fig.set_size_inches(10 * self.mainwindow.screen_scale, 6 * self.mainwindow.screen_scale)
            canvas = FigureCanvas(fig)
            scroll_layout.addWidget(canvas)

        plot_dialog.resize(int(1000 * self.mainwindow.screen_scale), int(800 * self.mainwindow.screen_scale))
        plot_dialog.exec_()

class MainWindow(QMainWindow):
    # Main application window: handles stacking pages, menu, scaling, and fullscreen.
    # Scaling logic ensures touch-friendliness and consistency across platforms.
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Hi-STIFFS Control 2026 v1.0")

        # Load settings from JSON (cross-platform file I/O).
        self.design_screen = {'width': 1920, 'height': 1080}  # Design reference for scaling.
        self.screen_scale = 1.0
        self.settings = load_settings()

        # Base values for fonts and sizes, optimized for touch (larger mins for Raspberry Pi touchscreen).
        # All scaling uses these bases, applied consistently via PyQt5 APIs.
        self.title_font_size = 26
        self.base_font_size = 14
        self.small1_font_size = 12
        self.small2_font_size = 10
        self.sub_font_size = 8
        self.clickable1_font_size = 14
        
        # Buttons: increased for touch (~60px height min for finger taps, cross-platform enforcement).
        self.base_button_height = 60  # Touch-optimized.
        self.base_button_width = 180  # Touch-optimized.
        self.base_margin = 20
        self.wide1_button_width = self.base_button_width * 2
        self.tall1_button_height = self.base_button_height * 2
        self.wide2_button_width = self.base_button_width * 3
        self.wide3_button_width = self.base_button_width * 4
        
        # Menus: increased height for touch.
        self.menubar_height = 40
        
        # Tree: item height for touch taps.
        self.base_tree_item_height = 50

        # Base fonts: created once, scaled copies applied to widgets.
        self.base_title_font = QFont()
        self.base_title_font.setPointSize(self.title_font_size)
        self.base_title_font.setBold(True)

        self.base_normal_font = QFont()
        self.base_normal_font.setPointSize(self.base_font_size)

        # Stacked widget for pages (QStackedWidget is cross-platform for navigation).
        self.stack = QStackedWidget()
        self.setCentralWidget(self.stack)

        self.home_page = HomePage(self)
        self.stack.addWidget(self.home_page)

        self.collect_page = CollectPage(self)
        self.stack.addWidget(self.collect_page)  # index 1

        self.file_page = FilePage(self)
        self.stack.addWidget(self.file_page)    # index 2

        # Menu bar: settings and view options.
        settings_menu = self.menuBar().addMenu("Settings")
        self.menuBar().setMinimumHeight(50)
        menu_font = QFont(self.base_normal_font)  # Copy base font.
        menu_font.setPointSize(int(self.clickable1_font_size * self.screen_scale))
        self.menuBar().setFont(menu_font)

        preferences_action = settings_menu.addAction("Preferences")
        preferences_action.triggered.connect(self.open_settings)
        view_menu = self.menuBar().addMenu("View")
        fullscreen_action = view_menu.addAction("Toggle Fullscreen")
        fullscreen_action.triggered.connect(self.toggle_fullscreen)
        update_window_action = view_menu.addAction("Refresh Window")
        update_window_action.triggered.connect(self.size_window)

        # Initial window sizing and scaling.
        self.size_window()

    def open_settings(self):
        # Opens preferences dialog, applies changes, and resizes if needed.
        dialog = SettingsDialog(self)
        if dialog.exec_() == QDialog.Accepted:
            self.settings['ui_preferences']['independent_plots'] = dialog.independent_plots_checkbox.isChecked()
            self.settings['ui_preferences']['screen_width'] = dialog.screen_width_spinbox.value()
            self.settings['ui_preferences']['detect_screen_size'] = dialog.detect_screen_size_checkbox.isChecked()
            self.settings['hardware_config']['adc_config'] = dialog.adc_config_edit.text()
            self.settings['hardware_config']['daq_config'] = dialog.daq_config_edit.text()
            self.settings['sensors'] = dialog.get_sensors()
            save_settings(self.settings)
            self.size_window()  # Apply screen changes

    def go_to_collect(self):
        self.stack.setCurrentIndex(1)

    def go_to_processing(self):
        # Navigates to file page, loads data.
        try:
            path = self.settings['paths'].get('raw_data_folder', RAW_DATA_BASE)
            self.file_page.load_data(path)
        except:
            self.settings['paths'] = {}
            self.settings['paths']['raw_data_folder'] = RAW_DATA_BASE
            save_settings(self.settings)
            path = self.settings['paths'].get('raw_data_folder', RAW_DATA_BASE)
            self.file_page.load_data(path)
        self.stack.setCurrentIndex(2)

    def go_to_home(self):
        if hasattr(self, 'collect_page') and self.collect_page.collecting:
            return  # Lock if collecting
        self.stack.setCurrentIndex(0)

    def size_window(self):
        # Sizes window based on screen detection or override, maintains aspect ratio.
        # Uses QApplication.primaryScreen() for cross-platform screen info.
        screen = QApplication.primaryScreen()
        self.screen_size = screen.size()
        actual_width = self.screen_size.width()

        # Determine width based on settings.
        if self.settings['ui_preferences'].get('detect_screen_size', True):
            use_width = actual_width
        else:
            use_width = self.settings['ui_preferences'].get('screen_width', 1920)
        self.screen_scale = use_width / self.design_screen['width']

        # Set height for 16:9 aspect, center on screen.
        use_height = int(use_width * 9 / 16)
        self.resize(use_width, use_height)
        self.move((actual_width - use_width) // 2, (self.screen_size.height() - use_height) // 2)

        # Update all widgets with new scale.
        self.update_scaling()

    def resizeEvent(self, event):
        # Handles resize: enforces 16:9 aspect (Qt resize events are cross-platform).
        super().resizeEvent(event)
        if self.width() / self.height() != 16 / 9:
            new_height = int(self.width() * 9 / 16)
            self.resize(self.width(), new_height)

        # Update scale based on current size.
        self.screen_scale = min(self.width() / self.design_screen['width'], self.height() / self.design_screen['height'])
        self.update_scaling()

    def update_scaling(self):
        # Triggers recursive update of all widgets (centralWidget handles the stack).
        self.update_all_widgets(self.centralWidget(), self.screen_scale)
        self.update()  # Force redraw if needed (Qt handles automatically, but ensures consistency).

    def update_all_widgets(self, widget, scale_factor):
        """Recursively updates fonts, sizes, and layouts for the widget and children.
        Prioritizes specific widgets by objectName/text for individual control, falls back to types.
        This granularity allows different sizes for similar widgets (e.g., buttons), crucial for touch UX.
        Cross-platform: Uses PyQt5 APIs like setMinimumSize, setStyleSheet, which work identically on all OSes."""
        # Get identifiers for specific checks.
        obj_name = widget.objectName()
        text = widget.text() if hasattr(widget, 'text') else ''
        
        # Specific widget updates: allows per-widget customization.
        if obj_name == "collect_button" or text == "Collect Data":
            scaled_font = QFont(self.base_normal_font)
            scaled_font.setPointSize(int(self.base_font_size * scale_factor))
            widget.setFont(scaled_font)
            prop_width = int(self.wide2_button_width * scale_factor)
            min_width = max(prop_width, self.base_button_width * 3)
            widget.setMinimumSize(min_width, int(self.base_button_height * scale_factor))
            widget.setMaximumWidth(min_width)

        elif obj_name == "config_button" or text == "Configure Header":
            scaled_font = QFont(self.base_normal_font)
            scaled_font.setPointSize(int(self.base_font_size * scale_factor))
            widget.setFont(scaled_font)
            button_width = int(self.design_screen['width'] * (1/7) * scale_factor)
            widget.setFixedWidth(button_width)
            widget.setMinimumHeight(int(self.base_button_height * scale_factor))

        elif obj_name == "start_button" or text == "Start":
            scaled_font = QFont(self.base_normal_font)
            scaled_font.setPointSize(int(self.base_font_size * scale_factor))
            widget.setFont(scaled_font)
            button_width = int(self.design_screen['width'] * (1/7) * scale_factor)
            widget.setFixedWidth(button_width)
            widget.setMinimumHeight(int(self.base_button_height * scale_factor))

        elif obj_name == "stop_button" or text == "Stop":
            scaled_font = QFont(self.base_normal_font)
            scaled_font.setPointSize(int(self.base_font_size * scale_factor))
            widget.setFont(scaled_font)
            button_width = int(self.design_screen['width'] * (1/7) * scale_factor)
            widget.setFixedWidth(button_width)
            widget.setMinimumHeight(int(self.base_button_height * scale_factor))
        
        if obj_name == "process_button" or text == "Process Existing Data":
            scaled_font = QFont(self.base_normal_font)
            scaled_font.setPointSize(int(self.base_font_size * scale_factor))
            widget.setFont(scaled_font)
            prop_width = int(self.wide2_button_width * scale_factor)  # Proportional width based on screen scale (approx. 28% of design width).
            min_width = max(prop_width, self.base_button_width * 3)  # Respect absolute minimum for touch-friendliness (cross-platform pixel enforcement).
            widget.setMinimumSize(min_width, int(self.base_button_height * scale_factor))
            widget.setMaximumWidth(min_width)  # Fixes width to proportional size without spanning full window (cross-platform via Qt size constraints).

        elif obj_name == "plot_button" or text == "Plot Selected":
            scaled_font = QFont(self.base_normal_font)
            scaled_font.setPointSize(int(self.base_font_size * scale_factor))
            widget.setFont(scaled_font)
            button_width = int(self.design_screen['width'] * (1/7) * scale_factor)  # Fixed 1/7 window width (cross-platform dynamic sizing).
            widget.setFixedWidth(button_width)
            widget.setMinimumHeight(int(self.base_button_height * scale_factor))  # Touch-friendly min height.

        elif obj_name == "back_button" or text == "Back to Home":
            scaled_font = QFont(self.base_normal_font)
            scaled_font.setPointSize(int(self.base_font_size * scale_factor))
            widget.setFont(scaled_font)
            button_width = int(self.design_screen['width'] * (1/7) * scale_factor)  # Fixed 1/7 window width.
            widget.setFixedWidth(button_width)
            widget.setMinimumHeight(int(self.base_button_height * scale_factor))

        elif obj_name == "browse_button" or text == "Browse Folder":
            scaled_font = QFont(self.base_normal_font)
            scaled_font.setPointSize(int(self.small1_font_size * scale_factor))
            widget.setFont(scaled_font)
            widget.setMinimumSize(int(self.base_button_width * scale_factor), int(self.menubar_height * scale_factor))  # Header compact.

        elif obj_name == "year_combo":
            scaled_font = QFont(self.base_normal_font)
            scaled_font.setPointSize(int(self.clickable1_font_size * scale_factor))
            widget.setFont(scaled_font)
            widget.setMinimumSize(int(self.base_button_width * scale_factor), int(self.menubar_height * scale_factor))

        elif obj_name == "data_tree":
            scaled_font = QFont(self.base_normal_font)
            scaled_font.setPointSize(int(self.base_font_size * scale_factor))
            widget.setFont(scaled_font)
            # Dynamic stylesheet for item height (touch-friendly, cross-platform).
            widget.setStyleSheet(f"QTreeWidget::item {{ height: {int(self.base_tree_item_height * scale_factor)}px; }}")
            col_width = int(self.design_screen['width'] * (3/7) * scale_factor)  # Each column 3/7 of window width.
            widget.setColumnWidth(0, col_width)
            widget.setColumnWidth(1, col_width)

        elif obj_name == "path_label":
            scaled_font = QFont(self.base_normal_font)
            scaled_font.setPointSize(int(self.small2_font_size * scale_factor))
            widget.setFont(scaled_font)
            widget.setMinimumHeight(int(self.base_button_height * scale_factor))
            widget.setFixedWidth(int(self.wide1_button_width * scale_factor))

        # Type-based fallbacks: for general widgets not specifically named.
        elif isinstance(widget, QLabel) and "Hi-STIFFS" in text:
            scaled_font = QFont(self.base_title_font)
            scaled_font.setPointSize(int(self.title_font_size * scale_factor))
            widget.setFont(scaled_font)
        
        elif isinstance(widget, (QLabel, QPushButton, QCheckBox, 
                                QSpinBox, QTreeWidget, QComboBox)):
            base_font = self.base_normal_font
            scaled_font = QFont(base_font)
            scaled_font.setPointSize(int(base_font.pointSize() * scale_factor))
            widget.setFont(scaled_font)
            
            # Default sizes for clickables.
            if isinstance(widget, QPushButton):
                widget.setMinimumHeight(int(self.base_button_height * scale_factor))
            elif isinstance(widget, QSpinBox):
                widget.setMinimumWidth(int(self.base_button_width * scale_factor))
            elif isinstance(widget, QComboBox):
                widget.setMinimumHeight(int(self.menubar_height * scale_factor))

        # Layout scaling: margins and spacing for better touch spacing.
        if widget.layout():
            layout = widget.layout()
            if isinstance(layout, (QVBoxLayout, QHBoxLayout, QFormLayout)):
                scaled_margin = int(self.base_margin * scale_factor)
                layout.setContentsMargins(scaled_margin, scaled_margin, scaled_margin, scaled_margin)
                layout.setSpacing(int(10 * scale_factor))  # Scaled spacing.

        # Menu bar specific: larger for touch.
        if isinstance(widget, QMenuBar):
            widget.setMinimumHeight(int(self.menubar_height * scale_factor))
            scaled_font = QFont(self.base_normal_font)
            scaled_font.setPointSize(int(self.base_font_size * scale_factor * 1.2))
            widget.setFont(scaled_font)

        # Recurse to children: handles nested and dynamic widgets (findChildren is cross-platform).
        for child in widget.findChildren(QWidget):
            self.update_all_widgets(child, scale_factor)

    def toggle_fullscreen(self):
        # Toggles fullscreen mode (showFullScreen/showNormal are cross-platform).
        if self.isFullScreen():
            self.showNormal()
        else:
            self.showFullScreen()

def load_settings():
    if os.path.exists(SETTINGS_FILE):
        with open(SETTINGS_FILE, 'r') as f:
            return json.load(f)
    # Default nested structure
    return {
        'ui_preferences': {
            'independent_plots': False,
            'detect_screen_size': True,
            'screen_width': 1920
        },
        'hardware_config': {
            'adc_config': "Analog-to-Digital Converter: ADS1220, Mode: Turbo, Data Rate: DR_90SPS, Analog Excitation/Reference Voltage: 5.1V +/-2mV",
            'daq_config': "DAQ Microcontroller: Arduino Nano ESP32, ID: Hi-STIFFS_Nano_01, CPU Clock: 240MHz, Cores: 2, Data-stream Connection: Wi-Fi"
        },
        'sensors': [],  # List of sensor dicts
        'test_config': {
            'custom_test_types': [],
            'parameters_by_type': {}  # e.g., {'Force Cycle': {'default_force': '20N', ...}}
        },
        'session_state': {
            'last_num_sensors': 5,
            'last_labels': ['A', 'B', 'C', 'D', 'E'],
            'last_sns': [],
            'last_note': '',
            'last_test_type': 'Demo',
            'last_test_number': 1,
            'last_test_rest': '',
            'last_probe_height': 0.785,
            'last_speed': '25 ft/min'
        }
    }

def save_settings(settings):
    # Saves settings to JSON (cross-platform).
    with open(SETTINGS_FILE, 'w') as f:
        json.dump(settings, f, indent=2)

def set_dark_mode(app):
    # Sets dark palette for app (QPalette is cross-platform, works on Windows/Linux/Pi for consistent theme).
    app.setStyle('Fusion')
    
    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(53, 53, 53))
    palette.setColor(QPalette.WindowText, Qt.white)
    palette.setColor(QPalette.Base, QColor(25, 25, 25))
    palette.setColor(QPalette.AlternateBase, QColor(53, 53, 53))
    palette.setColor(QPalette.ToolTipBase, Qt.white)
    palette.setColor(QPalette.ToolTipText, Qt.white)
    palette.setColor(QPalette.Text, Qt.white)
    palette.setColor(QPalette.Button, QColor(53, 53, 53))
    palette.setColor(QPalette.ButtonText, Qt.white)
    palette.setColor(QPalette.BrightText, Qt.red)
    palette.setColor(QPalette.Link, QColor(42, 130, 218))
    palette.setColor(QPalette.Highlight, QColor(42, 130, 218))
    palette.setColor(QPalette.HighlightedText, Qt.black)
    
    app.setPalette(palette)

if __name__ == "__main__":
    # Main entry: creates app, sets dark mode, shows window (QApplication is cross-platform core).
    app = QApplication(sys.argv)
    set_dark_mode(app)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())