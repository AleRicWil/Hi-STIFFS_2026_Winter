#!/usr/bin/env python3
"""
edit_contour.py – Build / edit a linear-cam contour and export it for the
physics engine.

This is the restructured equivalent of the original geomtry_opt.py example
usage.  Running the script builds the default multi-segment profile,
plots its kinematics, and writes new_profile.csv so that drive.py can
load it immediately.
"""

from __future__ import annotations

import os
import sys

from contour import LinearCamContour


def default_segments():
    """The segment list that was hard-coded in the original geomtry_opt.py."""
    return [
        {'dir': 'lead', 'type': 'dwell', 'length': 0.100+0.015, 'x': 0.06},
        {'dir': 'lead', 'type': 'dwell', 'length': 0.100, 'x': 0.06},
        {'dir': 'lead', 'type': 'rise',  'length': 0.263, 'delta_x': -0.06},
        {'dir': 'lead', 'type': 'dwell', 'length': 0.100+0.015, 'x': 0.00},
        {'dir': 'lead', 'type': 'dwell', 'length': 0.050, 'x': -0.000},
        {'dir': 'lead', 'type': 'rise',  'length': 0.677, 'delta_x': -0.401},

        {'dir': 'trail', 'type': 'dwell', 'length': 0.045, 'x': 0.06},
        {'dir': 'trail', 'type': 'rise',  'length': 0.263, 'delta_x': -0.06},
        {'dir': 'trail', 'type': 'dwell', 'length': 0.050, 'x': 0.00},
        {'dir': 'trail', 'type': 'dwell', 'length': 0.100+0.015, 'x': 0.00},
        {'dir': 'trail', 'type': 'dwell', 'length': 0.053, 'x': 0.00},
        {'dir': 'trail', 'type': 'rise',  'length': 0.390, 'delta_x': -0.401},
    ]


def main() -> None:
    # Forward speed used for kinematics plots (matches original example)
    vy = 0.447  # m/s ≈ 1 mph

    contour = LinearCamContour(vy=vy)
    segments = default_segments()

    print("Building profile with 3-4-5 polynomial transitions …")
    y, x = contour.build_profile(segments, curve_func=contour.poly_345)

    # Write the CSV that drive.py expects
    out_csv = os.path.join(os.path.dirname(__file__), "new_profile.csv")
    contour.export_profile(path=out_csv)

    # Kinematics figure (saved to disk for headless environments)
    kin_png = os.path.join(os.path.dirname(__file__), "contour_kinematics.png")
    contour.plot_kinematics(vy=contour.vy, save_path=kin_png)

    print("\nDone.")
    print(f"  Contour CSV : {out_csv}")
    print(f"  Kinematics  : {kin_png}")
    print("You can now run:  python drive.py")


if __name__ == "__main__":
    main()
