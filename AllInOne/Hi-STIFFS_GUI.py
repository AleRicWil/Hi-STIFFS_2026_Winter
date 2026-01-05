import sys
import os
import json
import re
import calendar
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QPalette, QColor, QFont
from PyQt5.QtCore import Qt
from process2 import HiSTIFFSData, RAW_DATA_BASE

SETTINGS_FILE = 'Cross-Platform_GUI_Settings.json'

class SettingsDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Preferences")
        layout = QFormLayout(self)

        self.independent_plots_checkbox = QCheckBox("Use independent plot windows (like standard Matplotlib)")
        layout.addRow("Plot Mode:", self.independent_plots_checkbox)

        self.detect_screen_size_checkbox = QCheckBox("Use detected screen size")
        layout.addRow("Screen Sizing: ", self.detect_screen_size_checkbox)

        self.screen_width_spinbox = QSpinBox()
        self.screen_width_spinbox.setRange(800, 3840)
        self.screen_width_spinbox.setSingleStep(100)
        layout.addRow("Screen Width Override:", self.screen_width_spinbox)

        buttons = QDialogButtonBox(QDialogButtonBox.Save | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addRow(buttons)

        # Load current settings
        settings = load_settings()
        self.independent_plots_checkbox.setChecked(settings.get('independent_plots', False))
        self.detect_screen_size_checkbox.setChecked(settings.get('detect_screen_size', True))
        self.screen_width_spinbox.setValue(settings.get('screen_width', 1920))

        # Apply scaling from parent (if shown after main window is scaled)
        if parent:
            parent.update_all_widgets(self, parent.screen_scale)


class HomePage(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)

        welcome_label = QLabel("Hi-STIFFS Operator Control 2026 v1.0")
        welcome_label.setAlignment(QtCore.Qt.AlignCenter)
        # Font will be set via scaling
        layout.addWidget(welcome_label)

        process_button = QPushButton("Process Existing Data")
        process_button.clicked.connect(parent.go_to_processing)
        layout.addWidget(process_button)


class FilePage(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.mainwindow = parent  # Capture MainWindow reference before reparenting
        self.data_files = {}
        self.current_base = None

        layout = QtWidgets.QVBoxLayout(self)

        self.year_layout = QtWidgets.QHBoxLayout()
        layout.addLayout(self.year_layout)

        self.tree = QtWidgets.QTreeWidget()
        self.tree.setHeaderHidden(True)
        self.tree.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        layout.addWidget(self.tree)

        plot_button = QtWidgets.QPushButton("Plot Selected")
        plot_button.clicked.connect(self.plot_selected)
        layout.addWidget(plot_button)

        self.browse_button = QtWidgets.QPushButton("Browse Folder")
        self.browse_button.clicked.connect(self.browse_folder)

        back_button = QtWidgets.QPushButton("Back to Home")
        back_button.clicked.connect(parent.go_to_home)
        layout.addWidget(back_button)

    def browse_folder(self):
        folder_path = QtWidgets.QFileDialog.getExistingDirectory(self.mainwindow, "Select Data Folder", self.current_base or self.mainwindow.settings.get('data_folder', RAW_DATA_BASE))
        if folder_path:
            self.load_data(folder_path)
            self.mainwindow.settings['data_folder'] = folder_path
            save_settings(self.mainwindow.settings)

    def load_data(self, folder_path):
        self.current_base = folder_path
        self.data_files = self.collect_data_files(folder_path)
        while self.year_layout.count():
            item = self.year_layout.takeAt(0)
            if item.widget():
                item.widget().setParent(None)
        sorted_years = sorted(self.data_files.keys())
        if sorted_years:
            combo = QtWidgets.QComboBox()
            combo.addItems(sorted_years)
            combo.setCurrentIndex(len(sorted_years) - 1)  # Default to most recent year
            combo.currentTextChanged.connect(self.show_months)
            self.year_layout.addWidget(combo)
            self.year_layout.addStretch()  # Align combo to the left
            self.year_layout.addWidget(self.browse_button)  # Add browse button to top right
            self.show_months(sorted_years[-1])
            self.mainwindow.update_scaling()  # Ensure new combo is scaled
        else:
            QtWidgets.QMessageBox.warning(self, "No Files", "No raw data files found in the selected folder or subfolders.")

    def collect_data_files(self, base_path):
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
        # Sort times
        for year in data_files:
            for month in data_files[year]:
                for day in data_files[year][month]:
                    data_files[year][month][day].sort(key=lambda x: x[1])
        return data_files

    def show_months(self, year):
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
            # Scale figure size based on parent scale (cross-platform, as Matplotlib with Qt backend handles resizing)
            fig.set_size_inches(10 * self.mainwindow.screen_scale, 6 * self.mainwindow.screen_scale)
            canvas = FigureCanvas(fig)
            scroll_layout.addWidget(canvas)

        plot_dialog.resize(int(1000 * self.mainwindow.screen_scale), int(800 * self.mainwindow.screen_scale))
        plot_dialog.exec_()


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Hi-STIFFS Operator Control 2026 v1.0")

        # Load settings
        self.design_screen = {'width': 1920, 'height': 1080}  # screen size used to set element/font sizes
        self.screen_scale = 1.0
        self.settings = load_settings()

        # Base reference values for scaling (cross-platform, as PyQt5 fonts are consistent across OSes)
        self.base_title_font_size = 26
        self.base_normal_font_size = 12
        self.base_button_height = 50
        self.base_browse_width = 100
        self.base_margin = 20
        self.menubar_height = 30

        # Create base fonts
        self.base_title_font = QFont()
        self.base_title_font.setPointSize(self.base_title_font_size)
        self.base_title_font.setBold(True)

        self.base_normal_font = QFont()
        self.base_normal_font.setPointSize(self.base_normal_font_size)

        # Stacked widget
        self.stack = QStackedWidget()
        self.setCentralWidget(self.stack)

        self.home_page = HomePage(self)
        self.stack.addWidget(self.home_page)

        self.file_page = FilePage(self)
        self.stack.addWidget(self.file_page)

        # Menu bar
        settings_menu = self.menuBar().addMenu("Settings")
        self.menuBar().setMinimumHeight(50)
        menu_font = QFont(self.base_normal_font)  # Copy base font
        menu_font.setPointSize(int(self.base_normal_font_size * self.screen_scale * 1.2))  # 20% larger than normal, scaled
        self.menuBar().setFont(menu_font)

        preferences_action = settings_menu.addAction("Preferences")
        preferences_action.triggered.connect(self.open_settings)
        view_menu = self.menuBar().addMenu("View")
        fullscreen_action = view_menu.addAction("Toggle Fullscreen")
        fullscreen_action.triggered.connect(self.toggle_fullscreen)
        update_window_action = view_menu.addAction("Refresh Window")
        update_window_action.triggered.connect(self.size_window)

        # Initial sizing and scaling
        self.size_window()

    def open_settings(self):
        dialog = SettingsDialog(self)
        if dialog.exec_() == QDialog.Accepted:
            self.settings['independent_plots'] = dialog.independent_plots_checkbox.isChecked()
            self.settings['screen_width'] = dialog.screen_width_spinbox.value()
            self.settings['detect_screen_size'] = dialog.detect_screen_size_checkbox.isChecked()
            save_settings(self.settings)
            # Resize and rescale based on new settings
            self.size_window()

    def go_to_processing(self):
        path = self.settings.get('data_folder', RAW_DATA_BASE)
        self.file_page.load_data(path)
        self.stack.setCurrentIndex(1)

    def go_to_home(self):
        self.stack.setCurrentIndex(0)

    def size_window(self):
        # Get screen info
        screen = QApplication.primaryScreen()
        self.screen_size = screen.size()
        actual_width = self.screen_size.width()

        # Choose width/scale based on user preference
        if self.settings.get('detect_screen_size', True):
            use_width = actual_width
        else:
            use_width = self.settings.get('screen_width', 1920)
        self.screen_scale = use_width / self.design_screen['width']

        # Assign height and position based on aspect ratio and actual screen
        use_height = int(use_width * 9 / 16)
        self.resize(use_width, use_height)
        self.move((actual_width - use_width) // 2, (self.screen_size.height() - use_height) // 2)

        # Trigger scaling update
        self.update_scaling()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        # Optionally enforce 16:9 aspect ratio (cross-platform, Qt handles resize events consistently)
        if self.width() / self.height() != 16 / 9:
            new_height = int(self.width() * 9 / 16)
            self.resize(self.width(), new_height)

        # Update screen_scale based on current size
        self.screen_scale = min(self.width() / self.design_screen['width'], self.height() / self.design_screen['height'])
        self.update_scaling()

    def update_scaling(self):
        # Recursively update all child widgets (this scaling approach is fully cross-platform, working identically on Windows 10/11, Ubuntu Linux, and Raspberry Pi 5 without code changes, as PyQt5 handles font and widget sizing consistently across OSes)
        self.update_all_widgets(self.centralWidget(), self.screen_scale)

        # Trigger a redraw (Qt often handles automatically, but ensures update)
        self.update()

    def update_all_widgets(self, widget, scale_factor):
        """Recursively update fonts, sizes, and layouts for this widget and its children."""
        # Update font if applicable (for text-based widgets)
        if isinstance(widget, (QLabel, QPushButton, QCheckBox, 
                               QSpinBox, QTreeWidget, QComboBox)):
            # Determine base font: title for welcome-like labels, normal otherwise
            if isinstance(widget, QLabel) and "Hi-STIFFS" in widget.text():
                base_font = self.base_title_font
            else:
                base_font = self.base_normal_font

            scaled_font = QFont(base_font)  # Make a copy
            scaled_font.setPointSize(int(base_font.pointSize() * scale_factor))
            widget.setFont(scaled_font)

        # Update sizes (e.g., for buttons or spinboxes)
        if isinstance(widget, QPushButton) and widget.text() == "Browse Folder":
            widget.setMinimumHeight(int(self.menubar_height * scale_factor))
            widget.setMinimumWidth(int(self.base_browse_width * scale_factor))
        elif isinstance(widget, QPushButton):
            widget.setMinimumHeight(int(self.base_button_height * scale_factor))
        if isinstance(widget, QSpinBox):
            widget.setMinimumWidth(int(100 * scale_factor))  # Example base width
        if isinstance(widget, QComboBox):
            scaled_font = QFont(self.base_normal_font)
            scaled_font.setPointSize(int(self.base_normal_font_size * scale_factor * 1.2))
            widget.setFont(scaled_font)
            widget.setMinimumHeight(int(self.menubar_height * scale_factor))

        # Scale layout margins if the widget has a layout
        if widget.layout():
            layout = widget.layout()
            if isinstance(layout, (QVBoxLayout, QHBoxLayout, QFormLayout)):
                scaled_margin = int(self.base_margin * scale_factor)
                layout.setContentsMargins(scaled_margin, scaled_margin, scaled_margin, scaled_margin)
                layout.setSpacing(int(10 * scale_factor))  # Optional: scale spacing

        if isinstance(widget, QMenuBar):
            widget.setMinimumHeight(int(self.menubar_height * scale_factor))  # Base height 30px, adjust as needed
            scaled_font = QFont(self.base_normal_font)
            scaled_font.setPointSize(int(self.base_normal_font_size * scale_factor * 1.2))
            widget.setFont(scaled_font)

        # Recurse into children (handles nested layouts/widgets, cross-platform as findChildren is Qt core)
        for child in widget.findChildren(QWidget):
            self.update_all_widgets(child, scale_factor)
    
    def toggle_fullscreen(self):
        if self.isFullScreen():
            self.showNormal()
        else:
            self.showFullScreen()


def load_settings():
    if os.path.exists(SETTINGS_FILE):
        with open(SETTINGS_FILE, 'r') as f:
            return json.load(f)
    return {'independent_plots': False, 'screen_width': 1920, 'data_folder': RAW_DATA_BASE, 'detect_screen_size': True}

def save_settings(settings):
    with open(SETTINGS_FILE, 'w') as f:
        json.dump(settings, f)

def set_dark_mode(app):
    # Set Fusion style for cross-platform consistency (works on Windows, Linux, Raspberry Pi)
    app.setStyle('Fusion')
    
    # Define a dark palette
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
    app = QApplication(sys.argv)
    set_dark_mode(app)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())