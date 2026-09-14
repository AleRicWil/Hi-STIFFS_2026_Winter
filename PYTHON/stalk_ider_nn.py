"""Train and run a 1D-CNN that finds stalk contacts in one sensor.

Task
----
A sliding 1000-sample window (~2 s at 500 SPS) of seven aligned traces
(time, force, position, and the first and second time derivatives of
force and position) goes into the network. The head emits up to
MAX_STALKS start/end times inside that window. Stalks on one sensor do
not overlap, so the head is an ordered set of slots rather than a
pixel-wise mask plus a hand-written splitter.

Why this head
-------------
The labeler stores interval bounds, not sample masks. Predicting those
bounds directly matches both the CSV and the field use case ("highlight
each contact"). A binary mask would still need a rule to split adjacent
pulses; the user asked the CNN to own that split. Ordered slots work
because contacts are segregated: slot 0 is the earliest fully
contained label in the window, slot 1 the next, and so on. Unused
slots are trained as negatives so a prediction that does not overlap
a label is pushed down.

Architecture (StalkBound1DCNN)
------------------------------
Input tensor shape is (batch, 7, 1000):

    channel 0  relative time in the window, scaled to [0, 1]
    channel 1  force (N), z-scored with training-set statistics
    channel 2  position (m), z-scored the same way
    channel 3  dF/dt (N/s), z-scored independently
    channel 4  d²F/dt² (N/s²), z-scored independently
    channel 5  dx/dt (m/s), z-scored independently
    channel 6  d²x/dt² (m/s²), z-scored independently

Derivatives are the same Savitzky–Golay traces HiSTIFFSData.calc_derivs
writes (dF_dt, d2F_dt2, dx_dt, d2x_dt2), computed on the full cropped
run then sliced into the window. Time is a ramp. After z-scoring the
physical channels it is the only explicit positional cue left once the
backbone pools, which is why it stays in the tensor even though SPS is
nearly constant.

Four downsample stages, each one ResidualDownBlock (two Conv1d +
BatchNorm1d + ReLU with a skip, then MaxPool1d(2)) plus two
same-resolution ResidualRefine units. Six unpooled refine units
follow (three at dilation 1, three at dilation 2). That is 18
residual units / 36 conv layers, three times the original 6 / 12.
Stochastic depth ramps to DROP_PATH_LAST on the refine units; the
last BN in each residual pair starts at zero so a new unit is an
identity until it learns a residual.

    7 → 64  kernel 7  (Down + 2× Refine)
   64 → 128 kernel 5  (Down + 2× Refine)
  128 → 192 kernel 5  (Down + 2× Refine)
  192 → 256 kernel 3  (Down + 2× Refine)
  256 → 256 kernel 3 dilation 1 × 3 (no pool)
  256 → 256 kernel 3 dilation 2 × 3 (no pool)

Each pool halves length, so 1000 samples become 62 feature steps
(~16 samples, ~32 ms per step). Extra depth sits at those four
scales; another pool would drop below the AdaptiveAvgPool grid.
The refine stack keeps that grid and widens context for longer
contacts without another downsample.

A three-layer MLP then maps the pooled feature to MAX_STALKS slots:

    AdaptiveAvgPool1d(32) → flatten 8192
    Linear 8192 → 512 + ReLU + Dropout
    Linear 512 → 512 + ReLU + Dropout
    Linear 512 → 512 + ReLU + Dropout
    presence  Linear 512 → K         (raw logits)
    bounds    Linear 512 → K × 2     (sigmoid → [0, 1] × [0, 1])

Forward() sorts each slot's two numbers so start <= end. Presence is
not a second "task"; it is how a fixed-size head reports a variable
number of contacts. After thresholding you only keep the (start, end)
pairs.

Training match
--------------
Slots are the labeled contacts whose Start and End fall in the
buffered window, in start-time order. Predicted slots are sorted by
start so they line up with those labels. Loss is:

    BCE-with-logits(presence)  +  SmoothL1(bounds)  +  (1 − GIoU)

Presence is 1 on labeled slots and 0 on the rest (including every
slot of an empty window). Bound and IoU terms run on labeled slots
only, so a prediction must overlap its matched label. Extra slots
that fire without a label are penalized as false positives.

Windows
-------
A 1000-sample cut starts at the first sample of the cropped range and
advances 0.05 s each window. A labeled contact is a slot when both
Start and End fall in the window expanded by ±WINDOW_LABEL_BUFFER_S
(a start/end may sit just outside either edge by that amount). Empty
windows are kept. Unlabeled contacts still show up at inference on
the last-three-ranges slice.

Before the first epoch, train() pages those windows in slide order
(sensor A start→end, then B, then C) as interactive force/position
plots (PREVIEW_WINDOWS; 0 skips). That is the cheapest check that
bounds land on real pulses and that a 2 s cut sometimes holds more
than one contact.

Inference
---------
Each run is cropped to last-three-ranges-times.csv when HiSTIFFSData
loads (same t_lims process.py uses). Then slide the window with a hop
of 250 samples, convert slot bounds back to absolute time, and
greedy-NMS duplicate hits of the same contact across overlapping
windows. The plot paints each surviving interval as its own color on
the force and position traces.

This module imports HiSTIFFSData and the stalk CSV helpers. It does
not re-parse raw files. Config and the unused ``keyboard`` dependency
are stubbed only when they are missing so the file still imports in a
bare environment.

Run it by editing the RUN SETTINGS block below and executing
``python stalk_cnn.py``. Command-line flags still work if you pass them.
"""

from __future__ import annotations

import argparse
import csv
import math
import random
import sys
import types
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import DataLoader, Dataset


# ---------------------------------------------------------------------------
# Import path for the existing pipeline (process.py + stalk_detector.py)
# ---------------------------------------------------------------------------
ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
if str(Path.cwd()) not in sys.path:
    sys.path.insert(0, str(Path.cwd()))


def _ensure_pipeline_imports(data_root: Path | None = None):
    """Make ``import process`` work even if config / keyboard are absent.

    process.py imports ``keyboard`` at module level but only uses it in
    ``__main__``. config.py lives next to the raw-data tree on the lab
    machines and is not in this folder. Both are injected as lightweight
    stand-ins so this script can still import HiSTIFFSData here.
    """
    if 'keyboard' not in sys.modules:
        keyboard_stub = types.ModuleType('keyboard')
        keyboard_stub.wait = lambda *args, **kwargs: None
        sys.modules['keyboard'] = keyboard_stub

    try:
        from config import Config  # type: ignore
    except ImportError:
        cfg = types.ModuleType('config')

        class Config:
            # Markers copied from the comments / error strings in process.py
            # and stalk_detector.py. Real config.py on the lab machine wins
            # when it is importable (the try branch above).
            RAW_DATA_BASE = Path('.')
            RESULTS_BASE = Path('.')
            HEADER_MARKER = 'end_metadata'
            DATA_MARKER = 'start_data'
            STALK_TIMES_MARKER = 'start_stalk_times'
            STIFFNESSES_MARKER = 'start_stiffnesses'

        cfg.Config = Config
        sys.modules['config'] = cfg

    from config import Config  # type: ignore
    if data_root is not None:
        Config.RAW_DATA_BASE = Path(data_root)
    return Config


# Pipeline imports happen in functions that already know --data-root, so a
# missing config path does not crash ``--help``.


# ===========================================================================
# RUN SETTINGS
# Edit this block, then run:  python stalk_cnn.py
# CLI flags (--run, --epochs, ...) override these only when you pass them.
# ===========================================================================

# 'train' fits the CNN on labeled original Start/End windows.
# 'infer' asks which RUN_TIMES entry to use, then slides a saved checkpoint
# over that one run and writes highlight plots.
MODE = 'train'
MODE = 'infer'

# None → use Config.RAW_DATA_BASE from config.py (the usual lab path).
# Set to a Path if you need to point at a different raw-data tree.
DATA_ROOT = None  # e.g. Path(r'Hi-STIFFS_2026_Winter/Raw Data')

# Last 10 Chesterfield times from process.py; date matches Raw Data\2026-08-28.
# RUN_TIMES[i] is Run Number i+1 in last-three-ranges-times.csv (same date folder).
# Train and infer both require that CSV; a missing file is fatal (no full-trace fallback).
RUN_DATE = '2026-08-28'
RUN_TIMES = [
    '104213', '110545', '112006', '113413', '114710',
    '120712', '122508', '123851', '125311', '130654',
]
RANGE_TIMES_CSV = 'last-three-ranges-times.csv'
NANO_LABEL = '01'
SENSORS_USED = 'A,B,C'     # comma-separated; model still sees one sensor per window
SEED = 0

# --- train ---
EPOCHS = 300
# Keep going after EPOCHS until this many epochs pass with no F1 gain
# (counted from the last improving epoch). Never stop before EPOCHS.
PATIENCE_EPOCHS = 30
BATCH_SIZE = 64
LR = 5.0e-4            # a bit lower than the 6-unit net; 18 residual units
WEIGHT_DECAY = 3e-4
WARMUP_EPOCHS = 16     # longer ramp so the extra residual units stay quiet
NEGATIVES_PER_SENSOR = 12   # empty windows drawn inside the labeled span only
VAL_FRAC = 0.15
CHECKPOINT_OUT = ROOT / 'checkpoints' / 'stalk_cnn.pt'
PREVIEW_WINDOWS = 1000       # >0 pages every training window in slide order; 0 skips
PREVIEW_INCLUDE_EMPTY = True  # True also pages empty (no-contact) windows

# --- infer ---
CHECKPOINT_IN = ROOT / 'checkpoints' / 'stalk_cnn.pt'
HOP_INFER = 100            # samples between sliding windows (~0.5 s at 500 SPS)
PRESENCE_THRESH = 0.9999     # keep a slot when sigmoid(logit) ≥ this
PLOT_DIR = ROOT / 'plots'
SHOW_PLOTS = True          # open all sensor figures together after they are saved

# Architecture constants. Changing WINDOW_LEN / MAX_STALKS / N_CHANNELS
# or the residual depth / MLP width invalidates an existing checkpoint —
# retrain after you touch them.
WINDOW_LEN = 1000          # samples ≈ 2.0 s at 500 SPS
WINDOW_HOP_S = 0.05        # training windows advance this far each step
WINDOW_LABEL_BUFFER_S = 0.030  # ±s on each window edge; Start/End may sit this far outside
CH_TIME = 0
CH_FORCE = 1
CH_POS = 2
CH_DF_DT = 3
CH_D2F_DT2 = 4
CH_DX_DT = 5
CH_D2X_DT2 = 6
N_CHANNELS = 7             # time, force, position, dF/dt, d²F/dt², dx/dt, d²x/dt²
MAX_STALKS = 7             # slots in the head; one window ≤ 5 contacts
STAGE_UNITS = 3            # residual units per downsample stage (was 1)
REFINE_REPEAT = 3          # copies of each dilated refine unit (was 1)
DROP_PATH_LAST = 0.15      # stochastic depth on the last refine unit; 0 at the first extra
MLP_HIDDEN = 512           # mixer width after AdaptiveAvgPool (was 384)
MLP_DROPOUT = 0.30         # dropout in the MLP (was 0.20)
NMS_IOU = 0.05             # merge the same contact seen in adjacent windows
SENSORS = ('A', 'B', 'C')  # labeled sensors; one model, one sensor at a time


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
class DropPath(nn.Module):
    """Stochastic depth: drop the residual branch per sample while training."""

    def __init__(self, p: float = 0.0):
        super().__init__()
        self.p = float(p)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        if self.p == 0.0 or not self.training:
            return x
        keep = 1.0 - self.p
        shape = (x.shape[0],) + (1,) * (x.ndim - 1)
        mask = x.new_empty(shape).bernoulli_(keep)
        return x * mask.div_(keep)


class ResidualDownBlock(nn.Module):
    """Two-conv residual stage, then pool to lengthen context.

    Kernel sizes start at 7 so the first layer sees ~14 ms at 500 SPS,
    on the order of a contact onset rather than a single sample. Batch
    norm lets CPU training use a modest learning rate without a long
    warmup. Bias is off because BN already supplies a shift. A 1×1 skip
    matches channels when the stage widens. The last BN starts at zero
    so the residual is an identity at init.
    """

    def __init__(self, in_ch: int, out_ch: int, kernel: int):
        super().__init__()
        pad = kernel // 2
        self.conv1 = nn.Conv1d(
            in_ch, out_ch, kernel_size=kernel, padding=pad, bias=False,
        )
        self.bn1 = nn.BatchNorm1d(out_ch)
        self.conv2 = nn.Conv1d(
            out_ch, out_ch, kernel_size=kernel, padding=pad, bias=False,
        )
        self.bn2 = nn.BatchNorm1d(out_ch)
        nn.init.zeros_(self.bn2.weight)
        if in_ch == out_ch:
            self.skip = nn.Identity()
        else:
            self.skip = nn.Sequential(
                nn.Conv1d(in_ch, out_ch, kernel_size=1, bias=False),
                nn.BatchNorm1d(out_ch),
            )
        self.pool = nn.MaxPool1d(kernel_size=2, stride=2)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        identity = self.skip(x)
        y = F.relu(self.bn1(self.conv1(x)), inplace=True)
        y = self.bn2(self.conv2(y))
        return self.pool(F.relu(y + identity, inplace=True))


class ResidualRefine(nn.Module):
    """Unpooled residual pair. Dilation > 1 widens context on the 62-step grid."""

    def __init__(
        self, ch: int, kernel: int = 3, dilation: int = 1, drop_path: float = 0.0,
    ):
        super().__init__()
        pad = dilation * (kernel // 2)
        self.conv1 = nn.Conv1d(
            ch, ch, kernel_size=kernel, padding=pad, dilation=dilation, bias=False,
        )
        self.bn1 = nn.BatchNorm1d(ch)
        self.conv2 = nn.Conv1d(
            ch, ch, kernel_size=kernel, padding=pad, dilation=dilation, bias=False,
        )
        self.bn2 = nn.BatchNorm1d(ch)
        nn.init.zeros_(self.bn2.weight)
        self.drop = DropPath(drop_path)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        y = F.relu(self.bn1(self.conv1(x)), inplace=True)
        y = self.bn2(self.conv2(y))
        return F.relu(self.drop(y) + x, inplace=True)


def _drop_schedule(n: int, last: float) -> list[float]:
    """Linear stochastic-depth rates from 0 to ``last`` over ``n`` units."""
    if n <= 1:
        return [float(last)]
    return [last * i / (n - 1) for i in range(n)]


def _make_stage(
    in_ch: int,
    out_ch: int,
    kernel: int,
    n_units: int,
    drop_rates: list[float],
) -> list[nn.Module]:
    """One downsample residual, then same-resolution refine units."""
    layers: list[nn.Module] = [ResidualDownBlock(in_ch, out_ch, kernel)]
    extra = n_units - 1
    if extra != len(drop_rates):
        raise ValueError(
            f'stage {in_ch}→{out_ch}: expected {extra} drop rates, got {len(drop_rates)}'
        )
    for p in drop_rates:
        layers.append(ResidualRefine(out_ch, kernel=kernel, drop_path=p))
    return layers


class StalkBound1DCNN(nn.Module):
    """1D-CNN → K ordered (presence, start, end) slots.

    Parameters
    ----------
    max_stalks
        Slot count. Must stay in lockstep with the dataset collate
        (both read MAX_STALKS). Raising it only adds empty slots until
        a window actually holds that many fully contained contacts.
    pool_len
        Time steps kept after the backbone. 32 × 256 channels = 8192
        numbers into the MLP, enough to place several 0.22 s contacts.
    hidden
        MLP width. Three 512-d layers mix the pooled steps into K boxes.
    """

    feat_ch = 256

    def __init__(
        self,
        in_ch: int = N_CHANNELS,
        max_stalks: int = MAX_STALKS,
        pool_len: int = 32,
        hidden: int = MLP_HIDDEN,
        dropout: float = MLP_DROPOUT,
        stage_units: int = STAGE_UNITS,
        refine_repeat: int = REFINE_REPEAT,
        drop_path_last: float = DROP_PATH_LAST,
    ):
        super().__init__()
        self.max_stalks = max_stalks
        n_extra = 4 * (stage_units - 1) + 2 * refine_repeat
        drops = _drop_schedule(n_extra, drop_path_last)
        di = 0

        def take(n: int) -> list[float]:
            nonlocal di
            sl = drops[di:di + n]
            di += n
            return sl

        self.backbone = nn.Sequential(
            *_make_stage(in_ch, 64, 7, stage_units, take(stage_units - 1)),
            *_make_stage(64, 128, 5, stage_units, take(stage_units - 1)),
            *_make_stage(128, 192, 5, stage_units, take(stage_units - 1)),
            *_make_stage(192, self.feat_ch, 3, stage_units, take(stage_units - 1)),
        )
        self.refine = nn.Sequential(
            *[
                ResidualRefine(self.feat_ch, kernel=3, dilation=1, drop_path=p)
                for p in take(refine_repeat)
            ],
            *[
                ResidualRefine(self.feat_ch, kernel=3, dilation=2, drop_path=p)
                for p in take(refine_repeat)
            ],
        )
        if di != n_extra:
            raise RuntimeError(f'drop-path schedule leftover: used {di} of {n_extra}')
        self.pool = nn.AdaptiveAvgPool1d(pool_len)
        self.mlp = nn.Sequential(
            nn.Flatten(),
            nn.Linear(self.feat_ch * pool_len, hidden),
            nn.ReLU(inplace=True),
            nn.Dropout(dropout),
            nn.Linear(hidden, hidden),
            nn.ReLU(inplace=True),
            nn.Dropout(dropout),
            nn.Linear(hidden, hidden),
            nn.ReLU(inplace=True),
            nn.Dropout(dropout),
        )
        self.presence_head = nn.Linear(hidden, max_stalks)
        self.bound_head = nn.Linear(hidden, max_stalks * 2)
        self._init_weights()

    def _init_weights(self) -> None:
        for m in self.modules():
            if isinstance(m, nn.Conv1d):
                nn.init.kaiming_normal_(m.weight, nonlinearity='relu')
            elif isinstance(m, nn.Linear):
                nn.init.kaiming_normal_(m.weight, nonlinearity='relu')
                if m.bias is not None:
                    nn.init.zeros_(m.bias)
        nn.init.xavier_uniform_(self.presence_head.weight)
        nn.init.constant_(self.presence_head.bias, -1.5)
        nn.init.xavier_uniform_(self.bound_head.weight)
        nn.init.zeros_(self.bound_head.bias)
        # Last BN of each residual pair starts at 0 so the skip is identity.
        for m in self.modules():
            if isinstance(m, (ResidualDownBlock, ResidualRefine)):
                nn.init.zeros_(m.bn2.weight)

    def forward(self, x: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        """
        Parameters
        ----------
        x : (B, 7, 1000)

        Returns
        -------
        presence_logits : (B, K)
            Raw scores. Train with BCEWithLogits. Infer with sigmoid.
        bounds : (B, K, 2)
            Start and end as fractions of the window, already ordered
            so bounds[..., 0] <= bounds[..., 1].
        """
        feat = self.refine(self.backbone(x))
        h = self.mlp(self.pool(feat))
        presence = self.presence_head(h)
        raw = torch.sigmoid(self.bound_head(h)).view(-1, self.max_stalks, 2)
        start = torch.minimum(raw[..., 0], raw[..., 1])
        end = torch.maximum(raw[..., 0], raw[..., 1])
        bounds = torch.stack((start, end), dim=-1)
        return presence, bounds


def count_parameters(model: nn.Module) -> int:
    return sum(p.numel() for p in model.parameters() if p.requires_grad)


def pick_device() -> torch.device:
    if torch.cuda.is_available():
        return torch.device('cuda')
    mps = getattr(torch.backends, 'mps', None)
    if mps is not None and mps.is_available():
        return torch.device('mps')
    return torch.device('cpu')


# ---------------------------------------------------------------------------
# Dataset: labeled original Start/End → fixed-length windows
# ---------------------------------------------------------------------------
@dataclass
class WindowExample:
    """One training window and the contacts that sit inside it."""

    x: np.ndarray              # (N_CHANNELS, WINDOW_LEN) float32, already normalized
    bounds: np.ndarray         # (K, 2) float32 in [0, 1]; unused slots 0
    presence: np.ndarray       # (K,) float32
    sensor: str
    run_key: str
    t_abs0: float              # absolute time of sample 0 (seconds)
    t_abs1: float              # absolute time of sample -1
    time: np.ndarray = field(default_factory=lambda: np.zeros(0, dtype=np.float64))
    force: np.ndarray = field(default_factory=lambda: np.zeros(0, dtype=np.float64))
    position: np.ndarray = field(default_factory=lambda: np.zeros(0, dtype=np.float64))


@dataclass
class RunSensorSeries:
    """One sensor from one run, plus original (not refine) intervals."""

    run_key: str
    sensor: str
    time: np.ndarray           # (N,) seconds, monotonic
    force: np.ndarray          # (N,) N
    position: np.ndarray       # (N,) m
    dF_dt: np.ndarray          # (N,) N/s
    d2F_dt2: np.ndarray        # (N,) N/s²
    dx_dt: np.ndarray          # (N,) m/s
    d2x_dt2: np.ndarray        # (N,) m/s²
    intervals: list[tuple[float, float]]  # original Start/End, lo <= hi


def _zscore(arr: np.ndarray, mean: float, std: float) -> np.ndarray:
    """Z-score, replace non-finite values, clip to ±5."""
    z = (np.asarray(arr, dtype=np.float64) - mean) / std
    np.nan_to_num(z, copy=False, nan=0.0, posinf=5.0, neginf=-5.0)
    return np.clip(z, -5.0, 5.0)


@dataclass
class NormStats:
    force_mean: float = 0.0
    force_std: float = 1.0
    pos_mean: float = 0.0
    pos_std: float = 1.0
    dF_dt_mean: float = 0.0
    dF_dt_std: float = 1.0
    d2F_dt2_mean: float = 0.0
    d2F_dt2_std: float = 1.0
    dx_dt_mean: float = 0.0
    dx_dt_std: float = 1.0
    d2x_dt2_mean: float = 0.0
    d2x_dt2_std: float = 1.0

    def apply(
        self,
        force: np.ndarray,
        position: np.ndarray,
        dF_dt: np.ndarray,
        d2F_dt2: np.ndarray,
        dx_dt: np.ndarray,
        d2x_dt2: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        return (
            _zscore(force, self.force_mean, self.force_std),
            _zscore(position, self.pos_mean, self.pos_std),
            _zscore(dF_dt, self.dF_dt_mean, self.dF_dt_std),
            _zscore(d2F_dt2, self.d2F_dt2_mean, self.d2F_dt2_std),
            _zscore(dx_dt, self.dx_dt_mean, self.dx_dt_std),
            _zscore(d2x_dt2, self.d2x_dt2_mean, self.d2x_dt2_std),
        )

    def to_dict(self) -> dict:
        return {
            'force_mean': self.force_mean,
            'force_std': self.force_std,
            'pos_mean': self.pos_mean,
            'pos_std': self.pos_std,
            'dF_dt_mean': self.dF_dt_mean,
            'dF_dt_std': self.dF_dt_std,
            'd2F_dt2_mean': self.d2F_dt2_mean,
            'd2F_dt2_std': self.d2F_dt2_std,
            'dx_dt_mean': self.dx_dt_mean,
            'dx_dt_std': self.dx_dt_std,
            'd2x_dt2_mean': self.d2x_dt2_mean,
            'd2x_dt2_std': self.d2x_dt2_std,
        }

    @classmethod
    def from_dict(cls, d: dict) -> 'NormStats':
        return cls(
            force_mean=float(d['force_mean']),
            force_std=float(d['force_std']),
            pos_mean=float(d['pos_mean']),
            pos_std=float(d['pos_std']),
            dF_dt_mean=float(d.get('dF_dt_mean', 0.0)),
            dF_dt_std=float(d.get('dF_dt_std', 1.0)),
            d2F_dt2_mean=float(d.get('d2F_dt2_mean', 0.0)),
            d2F_dt2_std=float(d.get('d2F_dt2_std', 1.0)),
            dx_dt_mean=float(d.get('dx_dt_mean', 0.0)),
            dx_dt_std=float(d.get('dx_dt_std', 1.0)),
            d2x_dt2_mean=float(d.get('d2x_dt2_mean', 0.0)),
            d2x_dt2_std=float(d.get('d2x_dt2_std', 1.0)),
        )


def _ordered_interval(t0, t1) -> tuple[float, float] | None:
    if t0 is None or t1 is None:
        return None
    try:
        a, b = float(t0), float(t1)
    except (TypeError, ValueError):
        return None
    if not (math.isfinite(a) and math.isfinite(b)):
        return None
    return (a, b) if a <= b else (b, a)


_RANGE_TIMES_CACHE: dict[Path, dict[int, tuple[float, float]]] = {}


def _load_range_times_table(csv_path: Path) -> dict[int, tuple[float, float]]:
    """Parse last-three-ranges-times.csv: Run Number → (start, end) seconds."""
    cached = _RANGE_TIMES_CACHE.get(csv_path)
    if cached is not None:
        return cached
    table: dict[int, tuple[float, float]] = {}
    with csv_path.open(newline='') as f:
        for row in csv.DictReader(f):
            n = int(str(row['Run Number']).strip())
            t0 = float(row['Start Time of Third-to-End Range (s)'])
            t1 = float(row['End Time of Run (s)'])
            table[n] = (t0, t1)
    _RANGE_TIMES_CACHE[csv_path] = table
    return table


def t_lims_for_run(date: str, time_id: str) -> list:
    """Return HiSTIFFSData t_lims for this run from last-three-ranges-times.csv.

    RUN_TIMES[i] is CSV Run Number i+1. Missing file or unmapped time_id
    is fatal so train/infer cannot silently use the full trace.
    """
    from config import Config
    csv_path = Path(Config.RAW_DATA_BASE) / date / RANGE_TIMES_CSV
    if not csv_path.exists():
        raise SystemExit(
            f'Missing {RANGE_TIMES_CSV} under {csv_path.parent} — '
            'refusing to load the full trace.'
        )
    try:
        run_n = RUN_TIMES.index(time_id) + 1
    except ValueError:
        raise SystemExit(
            f'{time_id} is not in RUN_TIMES — cannot map to {RANGE_TIMES_CSV}.'
        ) from None
    table = _load_range_times_table(csv_path)
    if run_n not in table:
        raise SystemExit(
            f'Run Number {run_n:02d} missing from {csv_path}.'
        )
    t0, t1 = table[run_n]
    print(f'  t_lims from {RANGE_TIMES_CSV} run {run_n:02d}: [{t0}, {t1}] s')
    return [t0, t1]


def load_run_series(
    date: str,
    time_id: str,
    nano_label: str = '01',
    sensors: Iterable[str] = SENSORS,
) -> list[RunSensorSeries]:
    """Load one raw CSV through HiSTIFFSData and attach original bounds.

    Force/position are computed the same way the labeler saw them
    (describe_channels + Savitzky–Golay + calibration). Time derivatives
    come from HiSTIFFSData.calc_derivs (non-uniform Savitzky–Golay on
    the full cropped traces). Refine columns are ignored on purpose.
    Time range comes from last-three-ranges-times.csv via
    HiSTIFFSData(t_lims=...); a missing CSV is fatal.
    """
    from process import HiSTIFFSData
    from stalk_detector import load_stalk_rows

    t_lims = t_lims_for_run(date, time_id)
    data = HiSTIFFSData(
        date=date, time=time_id, nano_label=nano_label, t_lims=t_lims,
    )
    if not data.exists:
        print(f'  skip missing raw file for {date} {time_id} {nano_label}')
        return []
    data.calc_force_position(filter_out=False, clip=False)
    data.calc_derivs()
    records = load_stalk_rows(data.stalks_csv_path)
    if not records:
        print(f'  no stalk rows in {data.stalks_csv_path}')

    run_key = f'{date}_{time_id}_{nano_label}'
    deriv_keys = ('dF_dt', 'd2F_dt2', 'dx_dt', 'd2x_dt2')
    out: list[RunSensorSeries] = []
    for lab in sensors:
        key = f'Sensor_{lab}'
        if key not in data.data_dict or 'force' not in data.data_dict[key]:
            continue
        s = data.data_dict[key]
        if any(k not in s for k in deriv_keys):
            print(f'  skip {run_key} sensor {lab}: missing time derivatives')
            continue
        t_arr = np.asarray(s['time'], dtype=np.float64)
        t_lo = float(t_arr[0]) if t_arr.size else None
        t_hi = float(t_arr[-1]) if t_arr.size else None
        intervals = []
        for rec in records:
            pair = _ordered_interval(rec.get(f'{lab}_Start'), rec.get(f'{lab}_End'))
            if pair is None:
                continue
            if t_lo is not None and (pair[1] < t_lo or pair[0] > t_hi):
                continue
            intervals.append(pair)
        intervals.sort(key=lambda p: p[0])
        out.append(RunSensorSeries(
            run_key=run_key,
            sensor=lab,
            time=t_arr,
            force=np.asarray(s['force'], dtype=np.float64),
            position=np.asarray(s['position'], dtype=np.float64),
            dF_dt=np.asarray(s['dF_dt'], dtype=np.float64),
            d2F_dt2=np.asarray(s['d2F_dt2'], dtype=np.float64),
            dx_dt=np.asarray(s['dx_dt'], dtype=np.float64),
            d2x_dt2=np.asarray(s['d2x_dt2'], dtype=np.float64),
            intervals=intervals,
        ))
        span = (f't=[{t_lo:.1f},{t_hi:.1f}]s, ' if t_lo is not None else '')
        print(f'  {run_key} sensor {lab}: {span}{t_arr.shape[0]} samples, '
              f'{len(intervals)} labeled contacts')
    return out


def _slice_or_pad(arr: np.ndarray, i0: int, length: int) -> np.ndarray:
    """Take arr[i0:i0+length], edge-padding if the run runs out."""
    n = arr.shape[0]
    sl = arr[max(0, i0):min(n, i0 + length)]
    if sl.shape[0] == length:
        return sl
    out = np.empty(length, dtype=arr.dtype)
    start_pad = max(0, -i0)
    out[:start_pad] = arr[0] if n else 0.0
    usable = sl.shape[0]
    out[start_pad:start_pad + usable] = sl
    out[start_pad + usable:] = arr[-1] if n else 0.0
    return out


def _interval_to_norm(
    t0: float, t1: float, t_win: np.ndarray,
    buffer_s: float = WINDOW_LABEL_BUFFER_S,
) -> tuple[float, float] | None:
    """Scale an absolute interval to [0, 1] if it sits in the buffered window.

    Both Start and End must lie in [t_win[0] - buffer, t_win[-1] + buffer].
    A start or end may sit just outside the plotted cut by that amount.
    Targets are clipped to [0, 1] so the sigmoid head stays in-range.
    """
    w0 = float(t_win[0])
    w1 = float(t_win[-1])
    span = w1 - w0
    if span <= 1e-9:
        return None
    buf = float(buffer_s)
    if t0 < w0 - buf or t1 > w1 + buf or t1 <= t0:
        return None
    n0 = (t0 - w0) / span
    n1 = (t1 - w0) / span
    return float(min(max(n0, 0.0), 1.0)), float(min(max(n1, 0.0), 1.0))


def _pack_slots(
    norms: list[tuple[float, float]],
    k: int = MAX_STALKS,
) -> tuple[np.ndarray, np.ndarray]:
    """Fill slots 0.. with buffered-window labels in start-time order."""
    bounds = np.zeros((k, 2), dtype=np.float32)
    presence = np.zeros((k,), dtype=np.float32)
    norms = sorted(norms, key=lambda p: p[0])[:k]
    for i, (a, b) in enumerate(norms):
        bounds[i, 0] = a
        bounds[i, 1] = b
        presence[i] = 1.0
    return bounds, presence


def pack_window_x(
    t_win: np.ndarray,
    f_win: np.ndarray,
    p_win: np.ndarray,
    dF_win: np.ndarray,
    d2F_win: np.ndarray,
    dx_win: np.ndarray,
    d2x_win: np.ndarray,
    stats: NormStats,
) -> np.ndarray:
    """Stack one window into (N_CHANNELS, WINDOW_LEN) float32."""
    rel_t = (t_win - t_win[0]) / max(float(t_win[-1] - t_win[0]), 1e-9)
    f_n, p_n, dF_n, d2F_n, dx_n, d2x_n = stats.apply(
        f_win, p_win, dF_win, d2F_win, dx_win, d2x_win,
    )
    return np.stack(
        (rel_t, f_n, p_n, dF_n, d2F_n, dx_n, d2x_n), axis=0,
    ).astype(np.float32)


def build_windows(
    series_list: list[RunSensorSeries],
    stats: NormStats,
    hop_s: float = WINDOW_HOP_S,
    label_buffer_s: float = WINDOW_LABEL_BUFFER_S,
) -> list[WindowExample]:
    """Cut 1000-sample windows from every loaded sensor.

    The first window starts at the first sample of the cropped range.
    Each following window starts hop_s later. A labeled contact becomes
    a slot when both Start and End fall in the window expanded by
    ±label_buffer_s.
    """
    examples: list[WindowExample] = []
    hop_s = float(hop_s)
    if hop_s <= 0.0:
        raise ValueError('hop_s must be > 0')

    for ser in series_list:
        n = int(ser.time.shape[0])
        if n < WINDOW_LEN:
            continue
        t = ser.time
        i0 = 0
        last_i0 = -1
        while i0 + WINDOW_LEN <= n:
            if i0 == last_i0:
                i0 += 1
                continue
            last_i0 = i0
            sl = slice(i0, i0 + WINDOW_LEN)
            t_win = t[sl]
            f_win = ser.force[sl]
            p_win = ser.position[sl]
            norms = []
            for lo, hi in ser.intervals:
                norm = _interval_to_norm(lo, hi, t_win, label_buffer_s)
                if norm is not None:
                    norms.append(norm)
            bounds, presence = _pack_slots(norms)
            x = pack_window_x(
                t_win, f_win, p_win,
                ser.dF_dt[sl], ser.d2F_dt2[sl],
                ser.dx_dt[sl], ser.d2x_dt2[sl],
                stats,
            )
            examples.append(WindowExample(
                x=x, bounds=bounds, presence=presence,
                sensor=ser.sensor, run_key=ser.run_key,
                t_abs0=float(t_win[0]), t_abs1=float(t_win[-1]),
                time=np.asarray(t_win, dtype=np.float64),
                force=np.asarray(f_win, dtype=np.float64),
                position=np.asarray(p_win, dtype=np.float64),
            ))
            i_next = int(np.searchsorted(t, float(t[i0]) + hop_s, side='left'))
            i0 = i_next if i_next > i0 else i0 + 1

    return examples


def _std_floor(arr: np.ndarray, floor: float = 1e-6) -> float:
    return float(max(np.nanstd(arr), floor))


def fit_norm_stats(series_list: list[RunSensorSeries]) -> NormStats:
    """Z-score from labeled-span samples only, not the unlabeled tails."""
    names = ('force', 'position', 'dF_dt', 'd2F_dt2', 'dx_dt', 'd2x_dt2')
    buckets: dict[str, list[np.ndarray]] = {k: [] for k in names}
    for ser in series_list:
        if not ser.intervals:
            continue
        mask = np.zeros(ser.time.shape[0], dtype=bool)
        t0 = ser.intervals[0][0]
        t1 = ser.intervals[-1][1]
        mask |= (ser.time >= t0) & (ser.time <= t1)
        if not np.any(mask):
            continue
        for name in names:
            buckets[name].append(getattr(ser, name)[mask])
    if not buckets['force']:
        return NormStats()
    cat = {k: np.concatenate(v) for k, v in buckets.items()}
    return NormStats(
        force_mean=float(np.nanmean(cat['force'])),
        force_std=float(max(np.nanstd(cat['force']), 1e-3)),
        pos_mean=float(np.nanmean(cat['position'])),
        pos_std=float(max(np.nanstd(cat['position']), 1e-3)),
        dF_dt_mean=float(np.nanmean(cat['dF_dt'])),
        dF_dt_std=_std_floor(cat['dF_dt']),
        d2F_dt2_mean=float(np.nanmean(cat['d2F_dt2'])),
        d2F_dt2_std=_std_floor(cat['d2F_dt2']),
        dx_dt_mean=float(np.nanmean(cat['dx_dt'])),
        dx_dt_std=_std_floor(cat['dx_dt']),
        d2x_dt2_mean=float(np.nanmean(cat['d2x_dt2'])),
        d2x_dt2_std=_std_floor(cat['d2x_dt2']),
    )


class StalkWindowDataset(Dataset):
    def __init__(self, examples: list[WindowExample], augment: bool = False):
        self.examples = examples
        self.augment = augment

    def __len__(self) -> int:
        return len(self.examples)

    def __getitem__(self, idx: int) -> dict:
        ex = self.examples[idx]
        x = ex.x
        if self.augment:
            x = x.copy()
            # Channels 1–6 are z-scored force, position, and their time
            # derivatives. Scale each parent with its derivatives so the
            # kinematics stay consistent, then add a little independent noise.
            scale_f = np.float32(np.random.uniform(0.80, 1.20))
            scale_p = np.float32(np.random.uniform(0.85, 1.15))
            x[CH_FORCE] *= scale_f
            x[CH_DF_DT] *= scale_f
            x[CH_D2F_DT2] *= scale_f
            x[CH_POS] *= scale_p
            x[CH_DX_DT] *= scale_p
            x[CH_D2X_DT2] *= scale_p
            x[CH_FORCE] += np.float32(np.random.normal(0.0, 0.05, size=x[CH_FORCE].shape))
            x[CH_POS] += np.float32(np.random.normal(0.0, 0.04, size=x[CH_POS].shape))
            x[CH_DF_DT] += np.float32(np.random.normal(0.0, 0.05, size=x[CH_DF_DT].shape))
            x[CH_D2F_DT2] += np.float32(np.random.normal(0.0, 0.05, size=x[CH_D2F_DT2].shape))
            x[CH_DX_DT] += np.float32(np.random.normal(0.0, 0.04, size=x[CH_DX_DT].shape))
            x[CH_D2X_DT2] += np.float32(np.random.normal(0.0, 0.04, size=x[CH_D2X_DT2].shape))
            np.clip(x[1:], -5.0, 5.0, out=x[1:])
        return {
            'x': torch.from_numpy(np.ascontiguousarray(x)),
            'bounds': torch.from_numpy(ex.bounds),
            'presence': torch.from_numpy(ex.presence),
        }


def split_examples(
    examples: list[WindowExample], val_frac: float = 0.15, seed: int = 0,
) -> tuple[list[WindowExample], list[WindowExample]]:
    """Hold out a fraction of windows for numeric val, grouped by run+sensor.

    Inference on unlabeled contacts still uses the full run. This split
    only answers "did the head learn to place bounds on held-out labeled
    windows from the same files."
    """
    rng = random.Random(seed)
    keys = sorted({(ex.run_key, ex.sensor) for ex in examples})
    rng.shuffle(keys)
    n_val = max(1, int(round(len(keys) * val_frac))) if keys else 0
    val_keys = set(keys[:n_val])
    train = [ex for ex in examples if (ex.run_key, ex.sensor) not in val_keys]
    val = [ex for ex in examples if (ex.run_key, ex.sensor) in val_keys]
    # Tiny sets: fall back to a random window split so training still runs.
    if not train or not val:
        rng.shuffle(examples)
        n = max(1, int(round(len(examples) * val_frac)))
        return examples[n:], examples[:n]
    return train, val


# ---------------------------------------------------------------------------
# Loss and metrics
# ---------------------------------------------------------------------------
def _sort_slots_by_start(
    presence_logits: torch.Tensor,
    bounds: torch.Tensor,
) -> tuple[torch.Tensor, torch.Tensor]:
    """Permute slots so index 0 is the earliest predicted start."""
    order = torch.argsort(bounds[..., 0], dim=1, stable=True)
    logits = torch.gather(presence_logits, 1, order)
    idx = order.unsqueeze(-1).expand_as(bounds)
    bounds = torch.gather(bounds, 1, idx)
    return logits, bounds


def _pair_iou(pred_bounds: torch.Tensor, tgt_bounds: torch.Tensor) -> torch.Tensor:
    """IoU of aligned (B, K) predicted and labeled intervals."""
    a0 = pred_bounds[..., 0]
    a1 = pred_bounds[..., 1]
    b0 = tgt_bounds[..., 0]
    b1 = tgt_bounds[..., 1]
    inter = (torch.minimum(a1, b1) - torch.maximum(a0, b0)).clamp(min=0.0)
    union = (a1 - a0).clamp(min=0.0) + (b1 - b0).clamp(min=0.0) - inter
    return inter / union.clamp(min=1e-6)


def _pair_giou(pred_bounds: torch.Tensor, tgt_bounds: torch.Tensor) -> torch.Tensor:
    """GIoU of aligned (B, K) intervals. In [-1, 1]; 1 is a perfect match.

    Unlike IoU, GIoU still has a gradient when the boxes do not overlap,
    so a miss is pulled toward the label instead of saturating at 0.
    """
    a0 = pred_bounds[..., 0]
    a1 = pred_bounds[..., 1]
    b0 = tgt_bounds[..., 0]
    b1 = tgt_bounds[..., 1]
    inter = (torch.minimum(a1, b1) - torch.maximum(a0, b0)).clamp(min=0.0)
    union = (a1 - a0).clamp(min=0.0) + (b1 - b0).clamp(min=0.0) - inter
    iou = inter / union.clamp(min=1e-6)
    c = (torch.maximum(a1, b1) - torch.minimum(a0, b0)).clamp(min=1e-6)
    return iou - (c - union) / c


def detection_loss(
    presence_logits: torch.Tensor,
    pred_bounds: torch.Tensor,
    tgt_presence: torch.Tensor,
    tgt_bounds: torch.Tensor,
    bound_w: float = 4.0,
    iou_w: float = 2.0,
) -> dict[str, torch.Tensor]:
    """Slot-aligned loss: labels line up with preds; extras are negatives.

    Predicted slots are sorted by start so slot k is scored against
    labeled slot k (labels are already start-ordered). Presence is
    trained on every slot (1 = labeled, 0 = empty). Bound SmoothL1 and
    (1 − GIoU) run on labeled slots so a match must overlap. Empty
    windows therefore push all K slots toward presence 0.
    """
    logits, pred_bounds = _sort_slots_by_start(presence_logits, pred_bounds)
    live = (tgt_presence >= 0.5).float()
    empty = 1.0 - live
    zero = (logits.sum() + pred_bounds.sum()) * 0.0

    bce_elem = F.binary_cross_entropy_with_logits(
        logits, tgt_presence, reduction='none',
    )
    n_pos = live.sum().clamp(min=1.0)
    n_neg = empty.sum().clamp(min=1.0)
    bce_pos = (bce_elem * live).sum() / n_pos
    bce_neg = (bce_elem * empty).sum() / n_neg
    bce = bce_pos + bce_neg

    n_live = live.sum()
    if float(n_live) == 0.0:
        bound = zero
        iou_pen = zero
    else:
        n_live = n_live.clamp(min=1.0)
        raw = F.smooth_l1_loss(pred_bounds, tgt_bounds, reduction='none')
        bound = (raw * live.unsqueeze(-1)).sum() / n_live
        iou_pen = ((1.0 - _pair_giou(pred_bounds, tgt_bounds)) * live).sum() / n_live
        bound = zero + bound
        iou_pen = zero + iou_pen

    total = zero + bce + bound_w * bound + iou_w * iou_pen
    return {
        'loss': total,
        'bce': bce.detach(),
        'bound': bound.detach(),
        'iou': iou_pen.detach(),
    }


def interval_iou(a0: float, a1: float, b0: float, b1: float) -> float:
    lo = max(a0, b0)
    hi = min(a1, b1)
    inter = max(0.0, hi - lo)
    union = max(a1 - a0, 0.0) + max(b1 - b0, 0.0) - inter
    if union <= 0.0:
        return 0.0
    return inter / union


def slot_metrics(
    presence_logits: torch.Tensor,
    pred_bounds: torch.Tensor,
    tgt_presence: torch.Tensor,
    tgt_bounds: torch.Tensor,
    thresh: float = PRESENCE_THRESH,
    iou_min: float = 0.5,
) -> dict[str, float]:
    """Score start-sorted pred slot k against labeled slot k.

    A live slot is TP only when it fires and overlaps the matched
    label by iou_min. Extra slots that fire with no label are FP.
    """
    logits, pred_bounds = _sort_slots_by_start(presence_logits, pred_bounds)
    conf = torch.sigmoid(logits).detach().cpu().numpy()
    pb = pred_bounds.detach().cpu().numpy()
    tp = tgt_presence.detach().cpu().numpy()
    tb = tgt_bounds.detach().cpu().numpy()
    tp_n = fp_n = fn_n = 0
    ious: list[float] = []
    for b in range(conf.shape[0]):
        n_gt = int(np.sum(tp[b] >= 0.5))
        for k in range(conf.shape[1]):
            fired = conf[b, k] >= thresh
            if k < n_gt:
                iou = interval_iou(
                    float(pb[b, k, 0]), float(pb[b, k, 1]),
                    float(tb[b, k, 0]), float(tb[b, k, 1]),
                )
                if not fired:
                    fn_n += 1
                    continue
                if iou >= iou_min:
                    tp_n += 1
                    ious.append(iou)
                else:
                    fp_n += 1
                    fn_n += 1
            elif fired:
                fp_n += 1
    prec = tp_n / max(tp_n + fp_n, 1)
    rec = tp_n / max(tp_n + fn_n, 1)
    return {
        'precision': prec,
        'recall': rec,
        'f1': 0.0 if prec + rec == 0 else 2 * prec * rec / (prec + rec),
        'mean_iou': float(np.mean(ious)) if ious else 0.0,
        'tp': float(tp_n),
        'fp': float(fp_n),
        'fn': float(fn_n),
    }


# ---------------------------------------------------------------------------
# Train / eval loops
# ---------------------------------------------------------------------------
def run_epoch(
    model: StalkBound1DCNN,
    loader: DataLoader,
    optimizer: torch.optim.Optimizer | None,
    device: torch.device,
) -> dict[str, float]:
    train = optimizer is not None
    model.train(train)
    totals = {'loss': 0.0, 'bce': 0.0, 'bound': 0.0, 'iou': 0.0}
    metric_acc = {
        'precision': 0.0, 'recall': 0.0, 'f1': 0.0, 'mean_iou': 0.0,
    }
    n_batches = 0
    for batch in loader:
        x = batch['x'].to(device)
        tgt_b = batch['bounds'].to(device)
        tgt_p = batch['presence'].to(device)
        if train:
            optimizer.zero_grad(set_to_none=True)
        logits, pred_b = model(x)
        parts = detection_loss(logits, pred_b, tgt_p, tgt_b)
        if train:
            parts['loss'].backward()
            nn.utils.clip_grad_norm_(model.parameters(), 1.0)
            optimizer.step()
        for k in totals:
            totals[k] += float(parts[k].detach())
        m = slot_metrics(logits, pred_b, tgt_p, tgt_b)
        for k in metric_acc:
            metric_acc[k] += m[k]
        n_batches += 1
    denom = max(n_batches, 1)
    out = {k: totals[k] / denom for k in totals}
    out.update({k: metric_acc[k] / denom for k in metric_acc})
    return out


def format_metrics(tag: str, epoch: int, m: dict[str, float]) -> str:
    return (
        f'{tag} epoch {epoch:03d}  loss={m["loss"]:.4f}  '
        f'bce={m["bce"]:.4f}  bound={m["bound"]:.4f}  '
        f'iou={m["iou"]:.4f}  '
        f'P={m["precision"]:.3f} R={m["recall"]:.3f} '
        f'F1={m["f1"]:.3f} IoU={m["mean_iou"]:.3f}'
    )


# ---------------------------------------------------------------------------
# Full-run inference + plot
# ---------------------------------------------------------------------------
@dataclass
class Detection:
    t0: float
    t1: float
    score: float
    sensor: str


def greedy_nms(dets: list[Detection], iou_min: float = NMS_IOU) -> list[Detection]:
    """Keep the highest-score box when two intervals cover the same contact."""
    dets = sorted(dets, key=lambda d: d.score, reverse=True)
    kept: list[Detection] = []
    for d in dets:
        if any(interval_iou(d.t0, d.t1, k.t0, k.t1) >= iou_min for k in kept):
            continue
        kept.append(d)
    kept.sort(key=lambda d: d.t0)
    return kept


@torch.no_grad()
def infer_sensor(
    model: StalkBound1DCNN,
    ser: RunSensorSeries,
    stats: NormStats,
    device: torch.device,
    hop: int = HOP_INFER,
    thresh: float = PRESENCE_THRESH,
) -> list[Detection]:
    """Slide a 1000-sample window down one sensor and collect live slots."""
    model.eval()
    n = int(ser.time.shape[0])
    if n == 0:
        return []
    dets: list[Detection] = []
    starts = list(range(0, max(1, n - WINDOW_LEN + 1), hop))
    if starts[-1] != max(0, n - WINDOW_LEN):
        starts.append(max(0, n - WINDOW_LEN))
    for a0 in starts:
        t_win = _slice_or_pad(ser.time, a0, WINDOW_LEN)
        f_win = _slice_or_pad(ser.force, a0, WINDOW_LEN)
        p_win = _slice_or_pad(ser.position, a0, WINDOW_LEN)
        x = pack_window_x(
            t_win, f_win, p_win,
            _slice_or_pad(ser.dF_dt, a0, WINDOW_LEN),
            _slice_or_pad(ser.d2F_dt2, a0, WINDOW_LEN),
            _slice_or_pad(ser.dx_dt, a0, WINDOW_LEN),
            _slice_or_pad(ser.d2x_dt2, a0, WINDOW_LEN),
            stats,
        )
        xt = torch.from_numpy(x).unsqueeze(0).to(device)
        logits, bounds = model(xt)
        conf = torch.sigmoid(logits)[0].cpu().numpy()
        boxes = bounds[0].cpu().numpy()
        span = float(t_win[-1] - t_win[0])
        for k in range(conf.shape[0]):
            if conf[k] < thresh:
                continue
            t0 = float(t_win[0] + boxes[k, 0] * span)
            t1 = float(t_win[0] + boxes[k, 1] * span)
            if t1 - t0 < 1e-3:
                continue
            dets.append(Detection(t0=t0, t1=t1, score=float(conf[k]),
                                  sensor=ser.sensor))
    return greedy_nms(dets)


def plot_sensor_detections(
    ser: RunSensorSeries,
    dets: list[Detection],
    out_path: Path | None = None,
    title: str | None = None,
    show: bool = False,
):
    """Highlight each predicted contact as its own color on force + position.

    Labeled original bounds are drawn as dashed vertical pairs so you
    can see which highlights sit on a CSV row and which are extra
    contacts the labeler never marked.
    """
    import matplotlib.pyplot as plt

    colors = ['#d62728', '#2ca02c', '#1f77b4', '#ff7f0e', '#9467bd',
              '#8c564b', '#e377c2', '#17becf', '#bcbd22', '#7f7f7f']
    fig, axs = plt.subplots(2, 1, sharex=True, figsize=(14, 6.5))
    t = ser.time
    axs[0].scatter(t, ser.force, s=5, color='0.65', zorder=1)
    axs[1].scatter(t, ser.position * 1000.0, s=5, color='0.65', zorder=1)

    for i, d in enumerate(dets):
        c = colors[i % len(colors)]
        mask = (t >= d.t0) & (t <= d.t1)
        axs[0].scatter(
            t[mask], ser.force[mask], s=15, color=c, zorder=3,
            label=f'{i + 1}  {d.t0:.2f}–{d.t1:.2f}s  ({d.score:.2f})',
        )
        axs[1].scatter(t[mask], ser.position[mask] * 1000.0, s=15, color=c, zorder=3)
        axs[0].axvline(d.t0, color=c, lw=0.8, alpha=0.7)
        axs[0].axvline(d.t1, color=c, lw=0.8, alpha=0.7)
        axs[1].axvline(d.t0, color=c, lw=0.8, alpha=0.7)
        axs[1].axvline(d.t1, color=c, lw=0.8, alpha=0.7)

    for a, b in ser.intervals:
        for ax in axs:
            ax.axvline(a, color='k', lw=0.9, ls='--', alpha=0.45, zorder=2)
            ax.axvline(b, color='k', lw=0.9, ls='--', alpha=0.45, zorder=2)

    axs[0].set_ylabel('Force (N)')
    axs[1].set_ylabel('Position (mm)')
    axs[1].set_xlabel('Time (s)')
    axs[0].grid(True, alpha=0.3)
    axs[1].grid(True, alpha=0.3)
    # note: do not apply a legend
    # if dets:
    #     axs[0].legend(loc='upper right', fontsize=8, ncol=2, framealpha=0.9)
    fig.suptitle(title or f'{ser.run_key}  sensor {ser.sensor}  '
                 f'{len(dets)} predicted / {len(ser.intervals)} labeled')
    fig.tight_layout()
    if out_path is not None:
        out_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(out_path, dpi=140)
        print(f'  wrote {out_path}')
    if not show:
        plt.close(fig)
    return fig


def _preview_pool(
    examples: list[WindowExample],
    n: int,
    include_empty: bool,
    seed: int,
) -> list[WindowExample]:
    """Windows in the same order ``build_windows()`` produced them.

    Per run: sensor A from the start of the range to the end, then B,
    then C. Empty windows are omitted unless ``include_empty`` is True.
    ``n`` only gates whether the preview runs (caller passes 0 to skip);
    it is not a random sample size. ``seed`` is unused.
    """
    del seed
    if n <= 0 or not examples:
        return []

    def n_live(ex: WindowExample) -> int:
        return int(ex.presence.sum())

    pool = examples if include_empty else [ex for ex in examples if n_live(ex) > 0]
    if not pool:
        pool = list(examples)

    sensor_rank = {lab: i for i, lab in enumerate(SENSORS)}
    pool.sort(key=lambda ex: (
        ex.run_key,
        sensor_rank.get(ex.sensor, 99),
        ex.sensor,
        ex.t_abs0,
    ))
    return pool


def _wait_until_figure_closed(fig) -> None:
    """Block until ``fig`` is closed (window X or Start train).

    ``plt.show(block=True)`` is enough on a desktop Tk/Qt backend. Some
    IDE / interactive backends return from show() immediately, so keep
    pumping the GUI loop until the figure is actually gone.
    """
    import matplotlib
    import matplotlib.pyplot as plt

    backend = matplotlib.get_backend().lower()
    if backend in {'agg', 'pdf', 'svg', 'ps', 'cairo', 'template'}:
        print(
            f'Preview: matplotlib backend {matplotlib.get_backend()!r} cannot '
            'open a window. Close is implied; epoch 1 will start.'
        )
        plt.close(fig)
        return

    plt.ioff()
    plt.show(block=True)
    while plt.fignum_exists(fig.number):
        fig.canvas.flush_events()
        plt.pause(0.05)


def preview_training_windows(
    examples: list[WindowExample],
    n: int = PREVIEW_WINDOWS,
    include_empty: bool = PREVIEW_INCLUDE_EMPTY,
    seed: int = 0,
) -> None:
    """Page through training windows as interactive force/position plots.

    Pages follow the sliding-window order: sensor A start→end, then B,
    then C. Each page is one 1000-sample cut the CNN will see: gray
    traces for the whole window, a distinct color for every live slot.
    Vertical lines mark the original Start/End that were packed into
    those slots (clipped to the window). Close the figure, press q, or
    hit Start train to continue to epoch 1.

    Physical units are plotted, not the z-scored channels. Normalization
    only rescales amplitude; it does not change where a bound sits.
    """
    sample = _preview_pool(examples, n=n, include_empty=include_empty, seed=seed)
    if not sample:
        print('Preview: no windows to show')
        return

    import matplotlib.pyplot as plt
    from matplotlib.widgets import Button

    colors = ['#d62728', '#2ca02c', '#1f77b4', '#ff7f0e', '#9467bd']
    state = {'i': 0}

    fig, axs = plt.subplots(2, 1, sharex=True, figsize=(13, 6.4))
    plt.subplots_adjust(bottom=0.18, top=0.90)

    def _draw():
        ex = sample[state['i']]
        t = ex.time if ex.time.size else np.linspace(ex.t_abs0, ex.t_abs1, WINDOW_LEN)
        force = ex.force if ex.force.size else np.zeros_like(t)
        pos_mm = (ex.position if ex.position.size else np.zeros_like(t)) * 1000.0
        n_live = int(ex.presence.sum())
        span = max(ex.t_abs1 - ex.t_abs0, 1e-9)

        for ax in axs:
            ax.clear()
            ax.grid(True, alpha=0.3)

        axs[0].scatter(t, force, s=5, color='0.65', zorder=1)
        axs[1].scatter(t, pos_mm, s=5, color='0.65', zorder=1)

        for k in range(ex.presence.shape[0]):
            if ex.presence[k] < 0.5:
                continue
            t0 = ex.t_abs0 + float(ex.bounds[k, 0]) * span
            t1 = ex.t_abs0 + float(ex.bounds[k, 1]) * span
            c = colors[k % len(colors)]
            mask = (t >= t0) & (t <= t1)
            axs[0].scatter(t[mask], force[mask], s=15, color=c, zorder=3,
                           label=f'slot {k}  {t0:.3f}–{t1:.3f} s')
            axs[1].scatter(t[mask], pos_mm[mask], s=15, color=c, zorder=3)
            for ax in axs:
                ax.axvline(t0, color=c, lw=1.0, alpha=0.85, zorder=2)
                ax.axvline(t1, color=c, lw=1.0, alpha=0.85, zorder=2)
                ax.axvspan(t0, t1, color=c, alpha=0.08, zorder=0)

        axs[0].set_ylabel('Force (N)')
        axs[1].set_ylabel('Position (mm)')
        axs[1].set_xlabel('Time (s)')
        # note: do not apply a legend
        # if n_live:
        #     axs[0].legend(loc='upper right', fontsize=8, framealpha=0.9)
        fig.suptitle(
            f'Train window {state["i"] + 1}/{len(sample)}   '
            f'{ex.run_key}  sensor {ex.sensor}   '
            f'{n_live} labeled slot(s)   '
            f'window [{ex.t_abs0:.3f}, {ex.t_abs1:.3f}] s'
        )
        fig.canvas.draw_idle()

    def _step(delta: int):
        state['i'] = (state['i'] + delta) % len(sample)
        _draw()

    def _close(_event=None):
        plt.close(fig)

    ax_prev = fig.add_axes((0.18, 0.05, 0.14, 0.07))
    ax_next = fig.add_axes((0.34, 0.05, 0.14, 0.07))
    ax_go = fig.add_axes((0.58, 0.05, 0.24, 0.07))
    b_prev = Button(ax_prev, 'Prev')
    b_next = Button(ax_next, 'Next')
    b_go = Button(ax_go, 'Start train')
    b_prev.on_clicked(lambda _e: _step(-1))
    b_next.on_clicked(lambda _e: _step(+1))
    b_go.on_clicked(_close)

    def _on_key(event):
        if event.key in ('right', 'n'):
            _step(+1)
        elif event.key in ('left', 'p'):
            _step(-1)
        elif event.key in ('q', 'escape'):
            _close()

    fig.canvas.mpl_connect('key_press_event', _on_key)
    # Keep widget refs so matplotlib does not GC the callbacks.
    fig._preview_buttons = (b_prev, b_next, b_go)  # type: ignore[attr-defined]

    _draw()
    print(
        f'Preview: {len(sample)} window(s). Close the figure or press Start train '
        'before epoch 1. Left/Right (or Prev/Next) pages; q also continues.'
    )
    _wait_until_figure_closed(fig)


# ---------------------------------------------------------------------------
# Checkpoint helpers
# ---------------------------------------------------------------------------
def save_checkpoint(
    path: Path,
    model: StalkBound1DCNN,
    stats: NormStats,
    epoch: int,
    args: argparse.Namespace,
):
    path.parent.mkdir(parents=True, exist_ok=True)
    torch.save(
        {
            'model': model.state_dict(),
            'stats': stats.to_dict(),
            'epoch': epoch,
            'window_len': WINDOW_LEN,
            'max_stalks': MAX_STALKS,
            'n_channels': N_CHANNELS,
            'args': vars(args),
        },
        path,
    )
    print(f'  saved {path}')


def load_checkpoint(path: Path, device: torch.device) -> tuple[StalkBound1DCNN, NormStats, dict]:
    blob = torch.load(path, map_location=device, weights_only=False)
    blob_ch = int(blob.get('n_channels', 3))
    if blob_ch != N_CHANNELS:
        raise SystemExit(
            f'{path} has n_channels={blob_ch}, this code expects {N_CHANNELS}. '
            "Retrain with MODE = 'train'."
        )
    model = StalkBound1DCNN()
    try:
        model.load_state_dict(blob['model'])
    except RuntimeError as exc:
        raise SystemExit(
            f'{path} does not match this StalkBound1DCNN '
            '(n_channels / residual depth / MLP width changed). '
            "Retrain with MODE = 'train'."
        ) from exc
    model.to(device)
    model.eval()
    stats = NormStats.from_dict(blob['stats'])
    return model, stats, blob


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def parse_run_flag(spec: str) -> tuple[str, str, str]:
    """Accept ``YYYY-MM-DD:HHMMSS`` or ``YYYY-MM-DD:HHMMSS:NN``."""
    parts = spec.split(':')
    if len(parts) == 2:
        return parts[0], parts[1], '01'
    if len(parts) == 3:
        return parts[0], parts[1], parts[2]
    raise argparse.ArgumentTypeError(
        f'run spec must be DATE:TIME or DATE:TIME:NANO, got {spec!r}'
    )


def args_from_settings() -> argparse.Namespace:
    """Build the same namespace the CLI would, from the RUN SETTINGS block."""
    return argparse.Namespace(
        cmd=MODE,
        data_root=DATA_ROOT,
        run=[f'{RUN_DATE}:{t}:{NANO_LABEL}' for t in RUN_TIMES],
        sensors=SENSORS_USED,
        seed=SEED,
        epochs=EPOCHS,
        batch_size=BATCH_SIZE,
        lr=LR,
        weight_decay=WEIGHT_DECAY,
        negatives_per_sensor=NEGATIVES_PER_SENSOR,
        val_frac=VAL_FRAC,
        out=CHECKPOINT_OUT,
        checkpoint=CHECKPOINT_IN,
        hop=HOP_INFER,
        thresh=PRESENCE_THRESH,
        plot_dir=PLOT_DIR,
        show=SHOW_PLOTS,
        preview=PREVIEW_WINDOWS,
        preview_empty=PREVIEW_INCLUDE_EMPTY,
    )


def build_argparser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description='1D-CNN stalk-bound detector (one sensor, original Start/End).',
    )
    sub = p.add_subparsers(dest='cmd', required=True)

    def add_shared(sp):
        sp.add_argument('--data-root', type=Path, default=DATA_ROOT,
                        help='Overrides Config.RAW_DATA_BASE (folder of dated run dirs).')
        sp.add_argument(
            '--run', action='append', default=[], metavar='DATE:TIME[:NANO]',
            help='Repeatable. Default comes from RUN_DATE / RUN_TIMES in the file.',
        )
        sp.add_argument('--sensors', default=SENSORS_USED,
                        help='Comma-separated sensor labels to use.')
        sp.add_argument('--seed', type=int, default=SEED)

    tr = sub.add_parser('train', help='Build windows from labeled CSVs and fit the CNN.')
    add_shared(tr)
    tr.add_argument(
        '--epochs', type=int, default=EPOCHS,
        help='Minimum epochs. Training then continues until PATIENCE_EPOCHS '
             'with no F1 gain (from the last improving epoch).',
    )
    tr.add_argument('--batch-size', type=int, default=BATCH_SIZE)
    tr.add_argument('--lr', type=float, default=LR)
    tr.add_argument('--weight-decay', type=float, default=WEIGHT_DECAY)
    tr.add_argument('--negatives-per-sensor', type=int, default=NEGATIVES_PER_SENSOR)
    tr.add_argument('--val-frac', type=float, default=VAL_FRAC)
    tr.add_argument('--out', type=Path, default=CHECKPOINT_OUT)
    tr.add_argument(
        '--preview', type=int, default=PREVIEW_WINDOWS, metavar='N',
        help='>0 pages every training window in slide order before epoch 1 (0 skips).',
    )
    tr.add_argument(
        '--preview-empty', action='store_true', default=PREVIEW_INCLUDE_EMPTY,
        help='Also page empty (no-contact) windows in the preview.',
    )

    inf = sub.add_parser(
        'infer',
        help='Pick one RUN_TIMES entry, slide the CNN over that run, highlight contacts.',
    )
    add_shared(inf)
    inf.add_argument('--checkpoint', type=Path, default=CHECKPOINT_IN)
    inf.add_argument('--hop', type=int, default=HOP_INFER)
    inf.add_argument('--thresh', type=float, default=PRESENCE_THRESH)
    inf.add_argument('--plot-dir', type=Path, default=PLOT_DIR)
    inf.add_argument('--show', action='store_true', default=SHOW_PLOTS)

    return p


def resolve_runs(args: argparse.Namespace) -> list[tuple[str, str, str]]:
    if args.run:
        return [parse_run_flag(s) for s in args.run]
    return [(RUN_DATE, t, NANO_LABEL) for t in RUN_TIMES]


def prompt_select_run(runs: list[tuple[str, str, str]]) -> tuple[str, str, str]:
    """Print numbered RUN_TIMES and return the one the user types."""
    if not runs:
        raise SystemExit('No runs listed — set RUN_TIMES or pass --run DATE:TIME.')
    dates = {d for d, _, _ in runs}
    print()
    if len(dates) == 1:
        print(f'Available RUN_TIMES ({next(iter(dates))}):')
        for i, (_, time_id, nano) in enumerate(runs, start=1):
            suffix = f'  nano {nano}' if nano != NANO_LABEL else ''
            print(f'  {i}. {time_id}{suffix}')
    else:
        print('Available RUN_TIMES:')
        for i, (date, time_id, nano) in enumerate(runs, start=1):
            print(f'  {i}. {date} {time_id}  nano {nano}')
    n = len(runs)
    while True:
        try:
            raw = input(f'Select run number [1-{n}]: ').strip()
        except EOFError:
            raise SystemExit('No run selected (stdin closed).') from None
        try:
            idx = int(raw)
        except ValueError:
            print(f'  enter an integer from 1 to {n}')
            continue
        if 1 <= idx <= n:
            chosen = runs[idx - 1]
            print(f'Selected {chosen[0]} {chosen[1]} {chosen[2]}')
            return chosen
        print(f'  enter an integer from 1 to {n}')


def load_all_series(args: argparse.Namespace) -> list[RunSensorSeries]:
    _ensure_pipeline_imports(args.data_root)
    sensors = tuple(s.strip().upper() for s in args.sensors.split(',') if s.strip())
    series: list[RunSensorSeries] = []
    for date, time_id, nano in resolve_runs(args):
        print(f'Loading {date} {time_id} {nano} ...')
        series.extend(load_run_series(date, time_id, nano, sensors=sensors))
    return series


def cmd_train(args: argparse.Namespace) -> None:
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    random.seed(args.seed)
    device = pick_device()
    print(f'Device: {device}')

    series = load_all_series(args)
    n_labels = sum(len(s.intervals) for s in series)
    if n_labels == 0:
        raise SystemExit(
            'No labeled original Start/End intervals found. '
            'Pass --data-root and --run DATE:TIME until a stalks_*.csv loads.'
        )
    stats = fit_norm_stats(series)
    print(
        f'Norm stats: force {stats.force_mean:.3f}±{stats.force_std:.3f} N  '
        f'pos {stats.pos_mean:.4f}±{stats.pos_std:.4f} m\n'
        f'            dF/dt {stats.dF_dt_mean:.3f}±{stats.dF_dt_std:.3f} N/s  '
        f'd2F/dt2 {stats.d2F_dt2_mean:.3f}±{stats.d2F_dt2_std:.3f} N/s²\n'
        f'            dx/dt {stats.dx_dt_mean:.4f}±{stats.dx_dt_std:.4f} m/s  '
        f'd2x/dt2 {stats.d2x_dt2_mean:.4f}±{stats.d2x_dt2_std:.4f} m/s²'
    )

    examples = build_windows(series, stats)
    n_pos = sum(1 for ex in examples if ex.presence.sum() > 0)
    n_neg = len(examples) - n_pos
    print(f'Windows: {len(examples)}  (positive={n_pos}, empty={n_neg})')
    train_ex, val_ex = split_examples(examples, val_frac=args.val_frac, seed=args.seed)
    print(f'Split: train={len(train_ex)}  val={len(val_ex)}')

    train_loader = DataLoader(
        StalkWindowDataset(train_ex, augment=True), batch_size=args.batch_size,
        shuffle=True, drop_last=False,
        pin_memory=device.type == 'cuda',
    )
    val_loader = DataLoader(
        StalkWindowDataset(val_ex, augment=False), batch_size=args.batch_size,
        shuffle=False,
        pin_memory=device.type == 'cuda',
    ) if val_ex else None

    model = StalkBound1DCNN().to(device)
    print(f'Model parameters: {count_parameters(model):,}')
    opt = torch.optim.AdamW(
        model.parameters(), lr=args.lr, weight_decay=args.weight_decay,
    )
    warmup_epochs = min(WARMUP_EPOCHS, max(1, args.epochs // 10))
    cosine_epochs = max(1, args.epochs - warmup_epochs)
    warmup = torch.optim.lr_scheduler.LinearLR(
        opt, start_factor=0.1, total_iters=warmup_epochs,
    )
    cosine = torch.optim.lr_scheduler.CosineAnnealingLR(
        opt, T_max=cosine_epochs, eta_min=args.lr * 0.05,
    )
    sched = torch.optim.lr_scheduler.SequentialLR(
        opt, schedulers=[warmup, cosine], milestones=[warmup_epochs],
    )

    best_f1 = -1.0
    best_epoch = 0
    best_path = args.out.with_name(args.out.stem + '_best' + args.out.suffix)
    patience = PATIENCE_EPOCHS

    preview_n = getattr(args, 'preview', PREVIEW_WINDOWS)
    if preview_n:
        print(
            'Opening training-window preview. '
            'Close the figure or press Start train to begin epoch 1.'
        )
        preview_training_windows(
            examples,
            n=preview_n,
            include_empty=getattr(args, 'preview_empty', PREVIEW_INCLUDE_EMPTY),
            seed=args.seed,
        )
        print('Preview closed — starting epoch 1.')

    epoch = 0
    while True:
        epoch += 1
        tr = run_epoch(model, train_loader, opt, device)
        print(format_metrics('train', epoch, tr))
        if val_loader:
            va = run_epoch(model, val_loader, None, device)
            print(format_metrics(' val ', epoch, va))
            score = va['f1']
        else:
            score = tr['f1']
        if score >= best_f1:
            best_f1 = score
            best_epoch = epoch
            save_checkpoint(best_path, model, stats, epoch, args)
        # Cosine is sized to EPOCHS; hold the floor LR on any patience tail.
        if epoch <= args.epochs:
            sched.step()
        stalled = epoch - best_epoch
        if epoch >= args.epochs and stalled >= patience:
            print(
                f'Stopping at epoch {epoch}: {stalled} since last F1 gain '
                f'(epoch {best_epoch}), min {args.epochs} done'
            )
            break
    save_checkpoint(args.out, model, stats, epoch, args)
    print(f'Done. last={args.out}  best_f1={best_f1:.3f} @ epoch {best_epoch} → {best_path}')


def cmd_infer(args: argparse.Namespace) -> None:
    date, time_id, nano = prompt_select_run(resolve_runs(args))
    args.run = [f'{date}:{time_id}:{nano}']
    device = pick_device()
    print(f'Device: {device}')
    model, stats, blob = load_checkpoint(args.checkpoint, device)
    print(f'Loaded {args.checkpoint} (epoch {blob.get("epoch")})')
    series = load_all_series(args)
    if not series:
        raise SystemExit('Nothing to infer — check --data-root / --run.')
    args.plot_dir.mkdir(parents=True, exist_ok=True)
    for ser in series:
        dets = infer_sensor(
            model, ser, stats, device,
            hop=args.hop, thresh=args.thresh,
        )
        print(f'{ser.run_key} {ser.sensor}: '
              f'{len(dets)} detections, {len(ser.intervals)} labeled')
        plot_sensor_detections(
            ser, dets,
            out_path=args.plot_dir / f'{ser.run_key}_{ser.sensor}.png',
            show=args.show,
        )
    if args.show:
        import matplotlib.pyplot as plt
        print(f'Showing {len(series)} interactive figure(s). Close the windows to continue.')
        plt.show()


def main(argv: list[str] | None = None) -> None:
    argv = list(sys.argv[1:] if argv is None else argv)
    if argv:
        args = build_argparser().parse_args(argv)
    else:
        args = args_from_settings()
        print(f'Using RUN SETTINGS in stalk_cnn.py  (MODE={args.cmd})')
    if args.cmd == 'train':
        cmd_train(args)
    elif args.cmd == 'infer':
        cmd_infer(args)
    else:
        raise SystemExit(
            f"unknown MODE {args.cmd!r} — set MODE to 'train' or 'infer' "
            f"in the RUN SETTINGS block"
        )


if __name__ == '__main__':
    main()
