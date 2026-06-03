"""Central configuration for the entire Hi-STIFFS library.
The GUI is the single human access point — it can override any path at startup.
Fully cross-platform: identical behaviour on Windows 10/11, Ubuntu, and Raspberry Pi 5 touchscreen."""

# Standard libraries
from pathlib import Path
from datetime import datetime

class Config:
    # Default base (relative to current working directory — works on all three OSes)
    # GUI can call Config.set_base_dir(...) once at launch
    BASE_DIR = Path("Hi-STIFFS_2026_Winter").resolve()

    # Derived directories — all use Path operations (cross-platform safe)
    CALIBRATION_DIR = BASE_DIR / "Calibration"
    RAW_DATA_BASE   = BASE_DIR / "Raw Data"
    RESULTS_BASE    = BASE_DIR / "Results"

    HEADER_MARKER = r'===END_METADATA==='
    DATA_MARKER = r'===BEGIN_DATA==='
    STALK_TIMES_MARKER = r'===BEGIN_STALK_TIMES==='
    STIFFNESSES_MARKER = r'===BEGIN_STIFFNESSES==='

    # Session timestamp for multi-probe collection (shared across all Nanos started together)
    _session_date_str = None
    _session_time_str = None

    # Network / hardware constants (still cross-platform — pure Python)
    HOST_IP   = "192.168.137.1"      # change via GUI if needed
    HOST_PORT = 8080

    @classmethod
    def set_base_dir(cls, new_path: str):
        """GUI calls this once at startup to point everything to the correct folder.
        Works identically on Windows, Ubuntu, and RPi 5."""
        cls.BASE_DIR = Path(new_path).resolve()
        cls.CALIBRATION_DIR = cls.BASE_DIR / "Calibration"
        cls.RAW_DATA_BASE   = cls.BASE_DIR / "Raw Data"
        cls.RESULTS_BASE    = cls.BASE_DIR / "Results"
        print(f"Hi-STIFFS base directory set to: {cls.BASE_DIR}")

    @classmethod
    def ensure_dirs(cls):
        """Create all folders if missing — safe on every OS."""
        cls.CALIBRATION_DIR.mkdir(parents=True, exist_ok=True)
        cls.RAW_DATA_BASE.mkdir(parents=True, exist_ok=True)
        cls.RESULTS_BASE.mkdir(parents=True, exist_ok=True)

    @classmethod
    def get_timestamped_filename(cls, prefix: str, extension: str = ".csv") -> Path:
        """Helper for consistent naming (used by calibration & collection)."""
        now = datetime.now()
        date_str = now.strftime("%Y-%m-%d")
        time_str = now.strftime("%H%M%S")
        parent_folder = Config.RAW_DATA_BASE / date_str
        parent_folder.mkdir(parents=True, exist_ok=True)
        
        return (cls.RAW_DATA_BASE / date_str / f"{date_str}_{time_str}_{prefix}{extension}",
                date_str, 
                time_str)
    
    @classmethod
    def start_new_data_session(cls):
        """Call once when you want to start collection for one or more probes together.
        Locks a single timestamp so every Nano's CSV gets the same {YYYY-MM-DD_HHMMSS}
        prefix (only the nano_id suffix changes). This is exactly what is needed for
        "start all configured probes at once, with the same timestamp/filename (except nano_id suffix)".
        
        Cross-platform: pure stdlib + pathlib; same on every supported OS."""
        now = datetime.now()
        cls._session_date_str = now.strftime("%Y-%m-%d")
        cls._session_time_str = now.strftime("%H%M%S")
        cls.ensure_dirs()
        print(f"[Config] Shared data session timestamp: {cls._session_date_str}_{cls._session_time_str}")

    @classmethod
    def get_session_filename(cls, nano_id, extension: str = ".csv"):
        """Use this for every probe in a multi-Nano run.
        nano_id may be int or str (e.g. 2 or "02" or the flashed NANO_ID from Arduino).
        Always returns zero-padded 2-digit suffix in the filename.
        If start_new_data_session() was never called, it auto-starts (safe fallback).
        
        This member function is the library-side implementation of the shared-timestamp
        requirement while keeping each probe's file, calibrations, and metadata completely independent.
        
        Cross-platform: works without modification on Windows, Ubuntu, and RPi 5 touchscreen."""
        if cls._session_date_str is None:
            cls.start_new_data_session()
        date_str = cls._session_date_str
        time_str = cls._session_time_str
        parent_folder = cls.RAW_DATA_BASE / date_str
        parent_folder.mkdir(parents=True, exist_ok=True)
        nid = f"{int(nano_id):02d}"
        full_path = cls.RAW_DATA_BASE / date_str / f"{date_str}_{time_str}_{nid}{extension}"
        return full_path, date_str, time_str