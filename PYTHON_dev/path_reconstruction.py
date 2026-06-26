"""
Path reconstruction for extended Hi-STIFFS NavX2 CSV logs.

This script reads a raw Hi-STIFFS CSV that contains the extended NavX2 columns
written by collect_data.py, extracts the displacement trace, and overlays it on
one of three XYZ reference paths loaded from the local Paths/ folder.
"""

import csv
import os
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from mpl_toolkits.mplot3d import Axes3D

from config import Config


NAVX_COLUMNS = [
    "NavX2_Disp_X_m",
    "NavX2_Disp_Y_m",
    "NavX2_Disp_Z_m",
]

Hi_STIFFS_DIR = Path(__file__).parent.parent

def load_reference_points(file_name: str) -> np.ndarray:
    csv_path = Hi_STIFFS_DIR / 'PYTHON' / 'Paths' / file_name

    data = pd.read_csv(csv_path)
    required_columns = ["X(mm)", "Y(mm)", "Z(mm)"]
    missing = [column for column in required_columns if column not in data.columns]
    if missing:
        raise ValueError(f"Missing columns in {csv_path.name}: {', '.join(missing)}")

    points_mm = data[required_columns].apply(pd.to_numeric, errors="coerce").dropna().to_numpy(dtype=float)
    if points_mm.size == 0:
        raise ValueError(f"No valid XYZ rows found in {csv_path.name}")

    points_m = points_mm * 1e-3
    return points_m - points_m[0]
REFERENCE_PATHS = {
    "Track 1":load_reference_points("Track 1.csv"),

    "Track 2":load_reference_points("Track 2.csv"),

    "Track 3":load_reference_points("Track 3.csv"),

    "Probe Contour v3.2":load_reference_points("Probe Contour v3.2.csv")
}
REFERENCE_PATHS["Track 3"][:, 1] *= -1


def find_data_start_row(csv_path: Path) -> int:
    check1 = False
    with csv_path.open("r", newline="") as file_handle:
        reader = csv.reader(file_handle)
        for row_index, row in enumerate(reader):
            if check1 and len(row) == 1 and row[0].strip() == Config.DATA_MARKER:
                return row_index
            if len(row) == 1 and row[0].strip() == Config.HEADER_MARKER:
                check1 = True
                continue
            check1 = False
    raise ValueError(f"Could not find data marker in {csv_path}")

def get_path(folder: Path, text):
    files = folder.iterdir()

    for file in files:
        print(file.name)

    usr_selection = input(text)

    return folder / usr_selection

def load_navx_path(csv_path: Path) -> np.ndarray:
    # get first row of data
    data_start = find_data_start_row(csv_path)
    data = pd.read_csv(csv_path, skiprows=data_start + 1)

    # check for missing columns
    missing = [column for column in NAVX_COLUMNS if column not in data.columns]
    if missing:
        raise ValueError(f"Missing NavX2 columns in {csv_path.name}: {', '.join(missing)}")

    # record navx path X,Y,Z
    path = data[NAVX_COLUMNS].apply(pd.to_numeric, errors="coerce").dropna().to_numpy(dtype=float)
    if path.size == 0:
        raise ValueError(f"No valid NavX2 displacement rows found in {csv_path.name}")

    # this sets origin to 0,0,0
    return path - path[0]

def get_track_name():
    for name in REFERENCE_PATHS.keys():
        print(name)
    user_selection = input('type the reference track: ')
    return user_selection

def resample_polyline(points: np.ndarray, num_samples: int = 500) -> np.ndarray:
    if len(points) < 2:
        return points.copy()

    segment_lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    arc_length = np.concatenate(([0.0], np.cumsum(segment_lengths)))
    total_length = arc_length[-1]
    if total_length == 0.0:
        return np.repeat(points[:1], num_samples, axis=0)

    sample_distances = np.linspace(0.0, total_length, num_samples)
    return np.column_stack([
        np.interp(sample_distances, arc_length, points[:, axis])
        for axis in range(points.shape[1])
    ])

def rotate_about_z(path: np.ndarray, z_degrees: float = 90.0) -> np.ndarray:
    theta = np.deg2rad(z_degrees)
    rot_z = np.array([
        [np.cos(theta), -np.sin(theta), 0.0],
        [np.sin(theta),  np.cos(theta), 0.0],
        [0.0,            0.0,           1.0],
    ])
    return path @ rot_z.T

def set_equal_aspect(ax, points: np.ndarray) -> None:
    min_values = points.min(axis=0)
    max_values = points.max(axis=0)
    span = max(max_values - min_values)
    if span == 0:
        span = 1.0

    center = (max_values + min_values) / 2.0
    half_span = span / 2.0
    ax.set_xlim(center[0] - half_span, center[0] + half_span)
    ax.set_ylim(center[1] - half_span, center[1] + half_span)
    ax.set_zlim(center[2] - half_span, center[2] + half_span)

def plot_path(measured_path: np.ndarray, reference_path: np.ndarray, reference_name: str, csv_path: Path) -> None:
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    ax.plot(
        reference_path[:, 0],
        reference_path[:, 1],
        reference_path[:, 2],
        color="tab:blue",
        linewidth=2.0,
        label=f"Reference: {reference_name}",
    )
    ax.plot(
        measured_path[:, 0],
        measured_path[:, 1],
        measured_path[:, 2],
        color="0.35",
        linewidth=1.8,
        label="Measured NavX2 path",
    )

    ax.scatter(*reference_path[0], color="tab:blue", s=40, marker="o")
    ax.scatter(*reference_path[-1], color="tab:blue", s=40, marker="^")
    ax.scatter(*measured_path[0], color="black", s=40, marker="o")
    ax.scatter(*measured_path[-1], color="black", s=40, marker="^")

    combined = np.vstack((reference_path, measured_path))
    set_equal_aspect(ax, combined)

    ax.set_xlabel("X displacement (m)")
    ax.set_ylabel("Y displacement (m)")
    ax.set_zlabel("Z displacement (m)")
    ax.set_title(f"NavX2 Path Reconstruction\n{csv_path.name}")
    ax.legend()
    ax.grid(True)

    fig.tight_layout()
    ax.view_init(elev=90, azim=-90)


def main() -> None:
    mode = 'auto'
    
    # --- MANUAL MODE ---
    if mode == 'manual':
        # get the test path
        test_folders_folder = Hi_STIFFS_DIR / 'Raw Data'
        test_folder_path = get_path(test_folders_folder, 'type the folder containing your test: ')
        test_path = get_path(test_folder_path, 'type test file name: ')

        measured_path = load_navx_path(test_path)
        measured_path = rotate_about_z(measured_path, 90)

        # get the reference path
        track_name = get_track_name()

        points = REFERENCE_PATHS[track_name]
        reference_path = resample_polyline(points)
        
        # plot comparison
        plot_path(measured_path, reference_path, track_name, test_path)
        name = input('type figure name: ')
        plt.savefig(name if name.endswith('.png') else name + '.png', dpi=300, bbox_inches='tight')
        plt.show()

    # --- AUTO MODE ---
    if mode == 'auto':
        # get the test path
        test_folders_folder = Hi_STIFFS_DIR / 'Raw Data'
        test_folder_path = get_path(test_folders_folder, 'type the folder containing your test: ')

        test = [
            ['Track 1', 'fast'],
            ['Track 1', 'fast'],
            ['Track 1', 'fast'],
            ['Track 2', 'fast'],
            ['Track 2', 'fast'],
            ['Track 2', 'fast'],
            ['Track 3', 'fast'],
            ['Track 3', 'fast'],
            ['Track 3', 'fast'],
            ['Track 1', 'slow'],
            ['Track 1', 'slow'],
            ['Track 1', 'slow'],
            ['Track 2', 'slow'],
            ['Track 2', 'slow'],
            ['Track 2', 'slow'],
            ['Track 3', 'slow'],
            ['Track 3', 'slow'],
            ['Track 3', 'slow'],
        ]

        test_paths = os.listdir(test_folder_path)
        for i, test_path in enumerate(test_paths):
            test_path = test_folder_path / test_path

            measured_path = load_navx_path(test_path)
            measured_path = rotate_about_z(measured_path, 90)

            # get the reference path
            track_name = test[i][0]

            points = REFERENCE_PATHS[track_name]
            reference_path = resample_polyline(points)
        
            # plot comparison
            plot_path(measured_path, reference_path, track_name, test_path)
            name = test[i][0] + ' ' + test[i][1] + ' ' + str(i)
            plt.savefig(name if name.endswith('.png') else name + '.png', dpi=300, bbox_inches='tight')
            plt.figure()

if __name__ == "__main__":
    main()