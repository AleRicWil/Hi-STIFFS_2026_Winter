"""Single source of truth for sensor labels, serial numbers, and calibration coefficients.
Fully cross-platform via pathlib.Path — identical on Windows 10/11, Ubuntu, and Raspberry Pi 5."""

# Standard libraries
from pathlib import Path
import csv
from datetime import datetime

# Installed packages
import pandas as pd

# Workspace scripts
from config import Config

class SensorRegistry:
    def __init__(self, calibration_dir: str = None):
        if calibration_dir is None:
            Config.ensure_dirs()                    # cross-platform folder creation
            self.calibration_dir = Config.CALIBRATION_DIR
        else:
            self.calibration_dir = Path(calibration_dir).resolve()
            
        self.current_cal_path = self.calibration_dir / "current_calibrations.csv"
        self.coeffs_by_sn = {}
        self.label_to_sn = {}          # e.g. {'A': '001', 'C': '003'}
        self.load()

    def load(self):
        """Load latest coeffs from current_calibrations.csv (cross-platform)."""
        if not self.current_cal_path.exists():
            print(f"Warning: {self.current_cal_path} not found. Using defaults.")
            return
        try:
            df = pd.read_csv(self.current_cal_path, dtype={'sn': str})
            for _, row in df.iterrows():
                sn = str(row['sn']).zfill(3)
                self.coeffs_by_sn[sn] = {
                    'datetime': row.get('datetime', 'N/A'),
                    'k1': float(row['k1']), 'd1': float(row['d1']), 'c1': float(row['c1']),
                    'k2': float(row['k2']), 'd2': float(row['d2']), 'c2': float(row['c2']),
                }
        except Exception as e:
            print(f"Warning: Failed to load SensorRegistry: {e}")

    def set_mapping(self, mapping: dict):
        """GUI or caller sets current label → SN mapping. Pure Python dict — cross-platform."""
        self.label_to_sn = {lbl.upper(): str(sn).zfill(3) for lbl, sn in mapping.items()}

    def get_coeffs(self, identifier):
        """identifier = label ('A') or SN ('001' or 1). Returns dict with datetime + k1..c2."""
        if isinstance(identifier, str) and len(identifier) <= 2 and identifier.upper() in 'ABCDE':
            sn = self.label_to_sn.get(identifier.upper())
            if sn is None:
                return self._default_coeffs()
            identifier = sn
        sn = str(identifier).zfill(3)
        print(f'[SensorRegistry] Retrieved calibration for sensor {sn}')
        return self.coeffs_by_sn.get(sn, self._default_coeffs())

    def _default_coeffs(self):
        return {'datetime': 'N/A', 'k1': 1.0, 'd1': 1.0, 'c1': 0.0,
                'k2': 1.0, 'd2': 1.0, 'c2': 0.0}

    def update_coeffs(self, sn: str, coeffs: dict):
        """Centralized update (called from calculate_coefficients). Cross-platform save."""
        sn = str(sn).zfill(3)
        now = datetime.now().strftime("%Y-%m-%d_%H%M%S")
        coeffs_with_dt = {**coeffs, 'datetime': now}
        self.coeffs_by_sn[sn] = coeffs_with_dt

        self._save_current()
        self._append_to_history(sn, now, coeffs)

    def _save_current(self):
        """Save current_calibrations.csv — fully cross-platform."""
        data = [[sn, c['datetime'], c['k1'], c['d1'], c['c1'], c['k2'], c['d2'], c['c2']]
                for sn, c in self.coeffs_by_sn.items()]
        self.calibration_dir.mkdir(parents=True, exist_ok=True)
        pd.DataFrame(data, columns=['sn', 'datetime', 'k1', 'd1', 'c1', 'k2', 'd2', 'c2']
                    ).to_csv(self.current_cal_path, index=False)

    def _append_to_history(self, sn: str, now: str, coeffs: dict):
        history_path = self.calibration_dir / f"calibration_history_{sn}.csv"
        headers = ['datetime', 'k1', 'd1', 'c1', 'k2', 'd2', 'c2']
        row = [now, coeffs['k1'], coeffs['d1'], coeffs['c1'], coeffs['k2'], coeffs['d2'], coeffs['c2']]
        history_path.parent.mkdir(parents=True, exist_ok=True)
        if not history_path.exists():
            with open(history_path, 'w', newline='') as f:
                csv.writer(f).writerow(headers)
        with open(history_path, 'a', newline='') as f:
            csv.writer(f).writerow(row)