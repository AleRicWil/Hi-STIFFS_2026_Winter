# Hi-STIFFS_GUI.py
# This script implements a cross-platform GUI for the Hi-STIFFS Operator Control system using PyQt5.
# It is designed to run seamlessly on Windows 10/11, Ubuntu Linux, and Raspberry Pi 5 with touchscreen support,
# without any OS-specific code changes. PyQt5 ensures consistent widget behavior, sizing, and event handling across platforms.
# The GUI serves as the human access point for processing data, with integration to 'process2.py' for data handling.
# Touchscreen optimizations include larger minimum widget sizes for finger taps (e.g., buttons >=60px height) and
# explicit size controls to prevent small, hard-to-tap elements. Scaling is applied dynamically for different screen sizes.

import sys
import os
import json
import re
import calendar
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtWidgets import *  # Imports all widgets for concise usage (as per user preference)
from PyQt5.QtGui import QPalette, QColor, QFont
from PyQt5.QtCore import Qt
from process2 import HiSTIFFSData, RAW_DATA_BASE

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

        # Dialog buttons for save/cancel, standard Qt behavior.
        buttons = QDialogButtonBox(QDialogButtonBox.Save | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addRow(buttons)

        # Load current settings from JSON file.
        settings = load_settings()
        self.independent_plots_checkbox.setChecked(settings.get('independent_plots', False))
        self.detect_screen_size_checkbox.setChecked(settings.get('detect_screen_size', True))
        self.screen_width_spinbox.setValue(settings.get('screen_width', 1920))

        # Apply scaling from parent if available (ensures dialog matches main app scale on all platforms).
        if parent:
            parent.update_all_widgets(self, parent.screen_scale)


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

        # Process button: main entry point, sized for touch (wide for prominence).
        process_button = QPushButton("Process Existing Data")
        process_button.setObjectName("process_button")  # Unique ID for targeted scaling.
        process_button.clicked.connect(parent.go_to_processing)
        process_button.setMinimumSize(parent.base_button_width * 2, parent.base_button_height)  # Initial touch-friendly size.
        layout.addWidget(process_button)


class FilePage(QWidget):
    # File selection page: allows browsing, year selection, tree view of data, and plotting.
    # Includes dynamic elements like combo box, with initial sizes for touch.
    def __init__(self, parent=None):
        super().__init__(parent)
        self.mainwindow = parent  # Reference to MainWindow for scaling and navigation.
        self.data_files = {}
        self.current_base = None

        layout = QVBoxLayout(self)

        # Horizontal layout for year selection and path/browse controls.
        self.year_layout = QHBoxLayout()
        layout.addLayout(self.year_layout)

        # Tree widget for displaying months/days/times, with touch-friendly item heights via stylesheet.
        self.tree = QTreeWidget()
        self.tree.setObjectName("data_tree")  # Unique ID for scaling.
        self.tree.setHeaderHidden(True)
        self.tree.setSelectionMode(QAbstractItemView.SingleSelection)
        # Initial stylesheet for item height (cross-platform, Qt stylesheets work identically on Windows/Linux/Pi).
        self.tree.setStyleSheet(f"QTreeWidget::item {{ height: {self.mainwindow.base_tree_item_height}px; }}")
        layout.addWidget(self.tree)

        # Plot button: triggers plotting of selected data.
        plot_button = QPushButton("Plot Selected")
        plot_button.setObjectName("plot_button")  # Unique ID.
        plot_button.clicked.connect(self.plot_selected)
        plot_button.setMinimumSize(self.mainwindow.base_button_width, self.mainwindow.base_button_height)  # Touch size.
        layout.addWidget(plot_button)

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

        # Back button: returns to home, wider for touch prominence.
        back_button = QPushButton("Back to Home")
        back_button.setObjectName("back_button")  # Unique ID.
        back_button.clicked.connect(parent.go_to_home)
        back_button.setMinimumSize(self.mainwindow.wide1_button_width, self.mainwindow.base_button_height)  # Wider size.
        layout.addWidget(back_button)

    def browse_folder(self):
        # Opens cross-platform folder dialog to select data directory.
        folder_path = QFileDialog.getExistingDirectory(self.mainwindow, "Select Data Folder", self.current_base or self.mainwindow.settings.get('data_folder', RAW_DATA_BASE))
        if folder_path:
            self.load_data(folder_path)
            if self.data_files:
                self.mainwindow.settings['data_folder'] = folder_path
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
        pattern = r'(\d{4}-\d{2}-\d{2})_test_(\d{6})\.csv'
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
            month_item = QTreeWidgetItem([month])
            for day in sorted_days:
                day_item = QTreeWidgetItem([day])
                for full_date, t in self.data_files[year][month][day]:
                    formatted_t = f"{t[:2]}:{t[2:4]}:{t[4:]}"
                    child = QTreeWidgetItem([formatted_t])
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
            if settings.get('independent_plots', False):
                data.plot_raw_strains(sensors=sensors_str)
                plt.show()
            else:
                self.embed_plots(data, sensors_str)
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

        self.file_page = FilePage(self)
        self.stack.addWidget(self.file_page)

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
            self.settings['independent_plots'] = dialog.independent_plots_checkbox.isChecked()
            self.settings['screen_width'] = dialog.screen_width_spinbox.value()
            self.settings['detect_screen_size'] = dialog.detect_screen_size_checkbox.isChecked()
            save_settings(self.settings)
            # Resize and rescale based on new settings (cross-platform via size_window).
            self.size_window()

    def go_to_processing(self):
        # Navigates to file page, loads data.
        path = self.settings.get('data_folder', RAW_DATA_BASE)
        self.file_page.load_data(path)
        self.stack.setCurrentIndex(1)

    def go_to_home(self):
        # Navigates back to home.
        self.stack.setCurrentIndex(0)

    def size_window(self):
        # Sizes window based on screen detection or override, maintains aspect ratio.
        # Uses QApplication.primaryScreen() for cross-platform screen info.
        screen = QApplication.primaryScreen()
        self.screen_size = screen.size()
        actual_width = self.screen_size.width()

        # Determine width based on settings.
        if self.settings.get('detect_screen_size', True):
            use_width = actual_width
        else:
            use_width = self.settings.get('screen_width', 1920)
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
        if obj_name == "process_button" or text == "Process Existing Data":
            scaled_font = QFont(self.base_normal_font)
            scaled_font.setPointSize(int(self.base_font_size * scale_factor))
            widget.setFont(scaled_font)
            widget.setMinimumSize(int(self.wide2_button_width * scale_factor), int(self.base_button_height * scale_factor))  # Wide main action.

        elif obj_name == "plot_button" or text == "Plot Selected":
            scaled_font = QFont(self.base_normal_font)
            scaled_font.setPointSize(int(self.base_font_size * scale_factor))
            widget.setFont(scaled_font)
            widget.setMinimumSize(int(self.base_button_width * scale_factor), int(self.base_button_height * scale_factor))  # Standard size.

        elif obj_name == "back_button" or text == "Back to Home":
            scaled_font = QFont(self.base_normal_font)
            scaled_font.setPointSize(int(self.base_font_size * scale_factor))
            widget.setFont(scaled_font)
            widget.setMinimumSize(int(self.wide1_button_width * scale_factor), int(self.base_button_height * scale_factor))  # Slightly wide.

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
    # Loads settings from JSON, defaults if file missing (cross-platform os.path and json).
    if os.path.exists(SETTINGS_FILE):
        with open(SETTINGS_FILE, 'r') as f:
            return json.load(f)
    return {'independent_plots': False, 'screen_width': 1920, 'data_folder': RAW_DATA_BASE, 'detect_screen_size': True}

def save_settings(settings):
    # Saves settings to JSON (cross-platform).
    with open(SETTINGS_FILE, 'w') as f:
        json.dump(settings, f)

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