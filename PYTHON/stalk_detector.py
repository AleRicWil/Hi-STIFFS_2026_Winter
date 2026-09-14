# Interactive stalk labeling, review, and stiffness-span refinement.
# Split out of process.py so HiSTIFFSData stays data-processing only.
#
# Public entry points
# -------------------
# interactive_detect_stalks(data, ...)
#     First-pass time-domain labeling. Confirm adds stalks on the current
#     plot; Done with Plot writes that plot's count and advances. Writes
#     Plot, Stalk, {A,B,C}_Start/End.
# interactive_detect_stalks_derivs(data, ...)
#     Same labeling session, but each sensor is a 3x2 force/position +
#     derivative figure (same layout as display_stalk_derivs).
# display_stalk_selections(data)
#     View-only paging through saved rows (original + refine if present).
# display_stalk_derivs(data)
#     Per-sensor derivative figures with original {A,B,C}_Start/End overlaid.
# refine_stalk_selections(data)
#     Pick a force-vs-probe-position sub-span inside each original bound.
#     Writes {A,B,C}_Refine_Start/End onto the same CSV (blank = unused sensor).
#
# Implementation is split across this folder:
#     stalk_common.py   constants, drawing helpers, stalks CSV I/O
#     stalk_detect.py   first-pass labeling session
#     stalk_review.py   review / refine / derivative overlay
#
# Cross-platform: Windows 10/11, Ubuntu, Raspberry Pi 5 + touchscreen.
# matplotlib only (same stack as process.py). No extra dependencies.
#
# Stalk work is A/B/C only. D and E are not part of this UI or the CSV.

from stalk_common import (
    TIME_SHIFT,
    STALK_SENSORS,
    load_stalk_rows,
    write_stalk_rows,
    refine_time_window,
    inclusive_time_mask,
)
from stalk_detect import (
    interactive_detect_stalks,
    interactive_detect_stalks_derivs,
)
from stalk_review import (
    display_stalk_selections,
    display_stalk_derivs,
    refine_stalk_selections,
)

__all__ = [
    'TIME_SHIFT',
    'STALK_SENSORS',
    'interactive_detect_stalks',
    'interactive_detect_stalks_derivs',
    'display_stalk_selections',
    'display_stalk_derivs',
    'refine_stalk_selections',
    'load_stalk_rows',
    'write_stalk_rows',
    'refine_time_window',
    'inclusive_time_mask',
]
