#!/usr/bin/env python3
"""
Stalk ↔ Fixed Contour Drive Simulation
======================================

Interactive high-fidelity 2-D simulation of a stalk (mass-spring-damper
circular probe) driven past a static polyline contour.

The spring attachment point ("origin") is translated at constant velocity
in the -Y direction.  The stalk interacts with the contour through the
same impulse-based contact model used in stalk_hockey_test.py:

  - Baumgarte velocity bias for stable resting contact
  - Stick-slip Coulomb friction (static / kinetic)
  - Sequential multi-contact resolution with multiple iterations
    (biased toward accuracy / fidelity rather than real-time speed)

Contour geometry is loaded from a user-supplied CSV file (no hard-coded
shapes).  The camera is fixed and centered on the contour centroid.

Controls
--------
  Mouse click     : focus a text-entry field
  Typing + Enter  : commit speed / lateral-x and reset the run
  R               : reset with currently applied parameters
  Space           : pause / unpause
  1 / 2           : decrease / increase spring stiffness k
  3 / 4           : decrease / increase damping b
  5 / 6           : decrease / increase coefficient of restitution
  7 / 8           : decrease / increase static friction μs
  9 / 0           : decrease / increase kinetic friction μk
  Esc / close     : quit

Units
-----
All physics is SI (m, kg, s).  Rendering scale is chosen automatically
so the contour fits the window with a comfortable margin.
"""

from __future__ import annotations

import os
import sys
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np
import pygame


# ---------------------------------------------------------------------------
# User-configurable paths & defaults
# ---------------------------------------------------------------------------
CONTOUR_CSV = r"C:\Users\Alex R. Williams\Desktop\v3.2.csv"   # <-- set path to your CSV here
CONTOUR_CSV = r'new_profile.csv'

# Default drive parameters (also editable live via the GUI fields)
DEFAULT_DRIVE_SPEED = 0.10       # m/s  (positive; actual velocity is -Y)
DEFAULT_LATERAL_X   = -0.020       # m

# Physical defaults (live-tunable)
MASS        = 1.0               # kg
RADIUS      = 0.020              # m
K_SPRING    = 60.0               # N/m
B_DAMPER    = 2.5                # N·s/m
RESTITUTION = 0.05               # -
MU_STATIC   = 0.25               # -
MU_KINETIC  = 0.10               # -
VEL_STICK_THRESHOLD = 1e-2       # m/s

# Numerical – biased toward accuracy / fidelity
PHYSICS_DT          = 1.0 / 2000.0   # 0.5 ms fixed step
MAX_SUBSTEPS        = 80             # allow slower-than-real-time if needed
CONTACT_ITERATIONS  = 3              # sequential multi-pass resolution
CONTACT_BIAS_FACTOR = 0.15           # Baumgarte (slightly conservative)
CONTACT_SLOP        = 0.0003         # m
CONTOUR_RESAMPLE_POINTS = 1000        # Resample the loaded contour to this many points, evenly spaced in Y.

# Display
WINDOW_WIDTH  = 1200
WINDOW_HEIGHT = 800
TARGET_FPS    = 60


# ---------------------------------------------------------------------------
# Contour loading framework (USER FILLS IN EXACT CSV LOGIC)
# ---------------------------------------------------------------------------
def load_contour_from_csv(csv_path: str) -> np.ndarray:
    """
    Load a sequence of contour points from a CSV file.

    ----------------------------------------------------------------
    USER: replace the body of this function with the exact parsing
    required by your CSV format.  The only hard requirement is that
    the function returns a NumPy array of shape (N, 2) and dtype
    float64 containing [x, y] coordinates in metres.
    ----------------------------------------------------------------

    Current skeleton (works for many simple CSVs):
      - comma-separated
      - optional header row (skipped if it cannot be parsed as float)
      - at least two columns; only the first two are used
    """
    if not os.path.isfile(csv_path):
        raise FileNotFoundError(f"Contour CSV not found: {csv_path}")

    # ---- BEGIN USER-EDITABLE SECTION ---------------------------------
    # Example using numpy.genfromtxt – adjust delimiter, skip_header,
    # comments, usecols, etc. to match your file.
    data = np.genfromtxt(
        csv_path,
        delimiter=",",
        comments="#",
        skip_header=1,          # set to 1 if first row is a header
        usecols=(0, 1),         # columns that contain x and y
        dtype=np.float64,
    )
    # ---- END USER-EDITABLE SECTION -----------------------------------

    if data.ndim == 1:
        # single point edge-case
        data = data.reshape(1, -1)
    if data.shape[1] < 2:
        raise ValueError(
            f"CSV must supply at least two columns (x,y); got shape {data.shape}"
        )
    points = np.ascontiguousarray(data[:, :2], dtype=np.float64)

    if points.shape[0] < 2:
        raise ValueError("Contour must contain at least two points")

    # Basic sanity: replace any NaNs that may have come from a header
    if np.isnan(points).any():
        # drop rows that failed to parse (common when header is present)
        mask = ~np.isnan(points).any(axis=1)
        points = points[mask]
        if points.shape[0] < 2:
            raise ValueError(
                "After removing non-numeric rows the contour has < 2 points. "
                "Check skip_header / CSV format."
            )
        
    # Auto-detect units: if the first few numeric values are large (> 10),
    # assume the CSV is in millimetres and convert to metres.
    n_sample = min(5, points.shape[0])
    sample = np.abs(points[:n_sample]).ravel()
    if sample.size > 0 and np.nanmax(sample) > 10.0:
        points *= 0.001  # mm → m
        print(
            "Contour values appear to be in mm (max sample > 10); "
            "auto-scaled by 0.001 to metres."
        )

    return points


def resample_contour_even_y(points: np.ndarray, n_points: int) -> np.ndarray:
    """
    Resample a polyline so the output points have Y-coordinates that are
    evenly spaced between the original y_max and y_min (descending, matching
    the -Y drive direction).

    For each target Y the corresponding X is obtained by linear interpolation
    on the first polyline segment that straddles that Y.  If no segment
    crosses the target Y (possible with non-monotonic or multi-valued
    contours), the nearest original point in Y is used as a fallback.

    This produces a uniform contact-test density along the drive axis while
    preserving the essential shape of typical stalk / cam profiles.
    """
    if n_points < 2 or points.shape[0] < 2:
        return points

    y = points[:, 1]
    y_min = float(y.min())
    y_max = float(y.max())
    if abs(y_max - y_min) < 1e-12:
        return points  # degenerate (all points at same Y)

    target_ys = np.linspace(y_max, y_min, n_points)
    new_pts = np.empty((n_points, 2), dtype=np.float64)

    for i, ty in enumerate(target_ys):
        found = False
        for j in range(len(points) - 1):
            y0 = points[j, 1]
            y1 = points[j + 1, 1]
            # Does segment [j, j+1] straddle or touch ty?
            if (y0 - ty) * (y1 - ty) <= 0.0:
                if abs(y1 - y0) < 1e-12:
                    x = points[j, 0]
                else:
                    t = (ty - y0) / (y1 - y0)
                    x = points[j, 0] + t * (points[j + 1, 0] - points[j, 0])
                new_pts[i] = (x, ty)
                found = True
                break
        if not found:
            # Fallback: nearest original point in Y
            idx = int(np.argmin(np.abs(y - ty)))
            new_pts[i] = points[idx]

    return new_pts


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------
def closest_point_on_segment(
    point: np.ndarray, a: np.ndarray, b: np.ndarray
) -> Tuple[np.ndarray, float]:
    """Closest point on closed segment [a,b] and the parametric t ∈ [0,1]."""
    ab = b - a
    ab_len2 = np.dot(ab, ab)
    if ab_len2 < 1e-14:
        return a.copy(), 0.0
    t = np.clip(np.dot(point - a, ab) / ab_len2, 0.0, 1.0)
    return a + t * ab, t


def rotate90(v: np.ndarray) -> np.ndarray:
    """Counter-clockwise 90° rotation (2-D cross-product helper)."""
    return np.array([-v[1], v[0]], dtype=np.float64)


# ---------------------------------------------------------------------------
# Contour – static polyline
# ---------------------------------------------------------------------------
@dataclass
class Contour:
    """Static polyline defined by an ordered sequence of points."""

    points: np.ndarray                    # (N, 2)
    segments: List[Tuple[np.ndarray, np.ndarray]] = field(init=False)
    # Per-segment axis-aligned bounding boxes: (xmin, xmax, ymin, ymax)
    seg_aabb: List[Tuple[float, float, float, float]] = field(init=False)
    centroid: np.ndarray = field(init=False)
    y_min: float = field(init=False)
    y_max: float = field(init=False)
    x_min: float = field(init=False)
    x_max: float = field(init=False)

    def __post_init__(self) -> None:
        self.points = np.asarray(self.points, dtype=np.float64)
        self.segments = []
        self.seg_aabb = []
        for i in range(len(self.points) - 1):
            a = self.points[i].copy()
            b = self.points[i + 1].copy()
            self.segments.append((a, b))
            self.seg_aabb.append((
                float(min(a[0], b[0])),
                float(max(a[0], b[0])),
                float(min(a[1], b[1])),
                float(max(a[1], b[1])),
            ))
        self.centroid = self.points.mean(axis=0)
        self.x_min, self.y_min = self.points.min(axis=0)
        self.x_max, self.y_max = self.points.max(axis=0)

    @property
    def extent(self) -> float:
        """Characteristic size used for auto-scaling the view."""
        return max(self.x_max - self.x_min, self.y_max - self.y_min)


# ---------------------------------------------------------------------------
# Stalk – circular body with moving spring origin
# ---------------------------------------------------------------------------
@dataclass
class Stalk:
    """
    Circular disk of mass `mass` whose centre is free.
    An isotropic linear spring-damper connects the centre to a moving
    origin that is driven at constant velocity.
    """

    mass: float
    radius: float
    k: float
    b: float
    pos: np.ndarray = field(default_factory=lambda: np.zeros(2, dtype=np.float64))
    vel: np.ndarray = field(default_factory=lambda: np.zeros(2, dtype=np.float64))
    # driven origin state (updated externally)
    origin_pos: np.ndarray = field(default_factory=lambda: np.zeros(2, dtype=np.float64))
    origin_vel: np.ndarray = field(default_factory=lambda: np.zeros(2, dtype=np.float64))

    def spring_damper_force(self) -> np.ndarray:
        """Force on the stalk from the spring-damper attached to the moving origin."""
        return (
            -self.k * (self.pos - self.origin_pos)
            - self.b * (self.vel - self.origin_vel)
        )

    def integrate(self, dt: float) -> None:
        """Semi-implicit Euler step (spring-damper only; contacts applied later)."""
        acc = self.spring_damper_force() / self.mass
        self.vel += acc * dt
        self.pos += self.vel * dt

    def apply_impulse(self, impulse: np.ndarray) -> None:
        self.vel += impulse / self.mass

    def reset(self, origin: np.ndarray, origin_velocity: np.ndarray) -> None:
        """Place stalk at the origin with matching velocity (zero relative motion)."""
        self.origin_pos[:] = origin
        self.origin_vel[:] = origin_velocity
        self.pos[:] = origin
        self.vel[:] = origin_velocity


# ---------------------------------------------------------------------------
# Contact resolution – circle vs. static segment (high-accuracy path)
# ---------------------------------------------------------------------------
def resolve_circle_static_segment(
    stalk: Stalk,
    a: np.ndarray,
    b: np.ndarray,
    restitution: float,
    mu_s: float,
    mu_k: float,
) -> bool:
    """
    Resolve a single circle-vs-static-segment contact.

    Returns True if an impulse was applied.
    The segment is treated as having infinite mass and zero velocity.
    """
    closest, _ = closest_point_on_segment(stalk.pos, a, b)
    delta = stalk.pos - closest
    dist = np.linalg.norm(delta)

    if dist >= stalk.radius - 1e-10:
        return False

    # Outward normal (from segment toward stalk centre)
    if dist > 1e-9:
        n = delta / dist
    else:
        # Degenerate: use segment normal
        direction = b - a
        n = rotate90(direction)
        n /= np.linalg.norm(n) + 1e-15

    penetration = stalk.radius - dist

    # Relative velocity (contour is static)
    v_rel = stalk.vel.copy()
    v_n = np.dot(v_rel, n)

    # Early-out: separating and only lightly penetrating
    if v_n > 0.0 and penetration < 1e-5:
        return False

    # ----- Baumgarte velocity bias -----
    bias = 0.0
    if penetration > CONTACT_SLOP:
        bias = (CONTACT_BIAS_FACTOR / PHYSICS_DT) * (penetration - CONTACT_SLOP)

    # ----- Normal impulse -----
    # Restitution acts only on the true approaching component
    j_n = -(v_n + bias) * stalk.mass
    if v_n < 0.0:
        j_n -= restitution * v_n * stalk.mass
    j_n = max(j_n, 0.0)

    # ----- Tangential direction & friction -----
    v_t_vec = v_rel - v_n * n
    v_t = np.linalg.norm(v_t_vec)
    if v_t > 1e-12:
        t = v_t_vec / v_t
    else:
        t = np.zeros(2, dtype=np.float64)

    # Stick / slip (attempt stick first – high-rate steps make this reliable)
    j_t_required = -np.dot(v_rel, t) * stalk.mass
    max_static = mu_s * j_n

    if abs(j_t_required) <= max_static and v_t < VEL_STICK_THRESHOLD * 5.0:
        j_t = j_t_required          # stick
    else:
        # Kinetic friction opposes tangential velocity
        j_t = -mu_k * j_n if v_t > 1e-12 else 0.0
        # (sign is already correct because t points along v_t)

    # Apply total impulse
    impulse = j_n * n + j_t * t
    stalk.apply_impulse(impulse)

    # Mild positional correction (bias already does most of the work)
    stalk.pos += 0.85 * penetration * n

    return True


def resolve_circle_contour(
    stalk: Stalk,
    contour: Contour,
    restitution: float,
    mu_s: float,
    mu_k: float,
) -> bool:
    """
    Multi-contact resolution against the whole polyline.

    Performs CONTACT_ITERATIONS sequential sweeps for higher fidelity
    when several segments are active simultaneously.

    Spatial culling: each segment's pre-computed AABB is tested against
    the stalk's expanded AABB before the more expensive closest-point
    calculation.
    """
    any_contact = False

    # Expanded stalk AABB (radius + small numerical margin)
    r = stalk.radius + 2e-3          # 2 mm margin is plenty
    sx0 = stalk.pos[0] - r
    sx1 = stalk.pos[0] + r
    sy0 = stalk.pos[1] - r
    sy1 = stalk.pos[1] + r

    for _ in range(CONTACT_ITERATIONS):
        for i, (a, b) in enumerate(contour.segments):
            xmin, xmax, ymin, ymax = contour.seg_aabb[i]
            # Cheap AABB rejection
            if sx1 < xmin or sx0 > xmax or sy1 < ymin or sy0 > ymax:
                continue
            if resolve_circle_static_segment(
                stalk, a, b, restitution, mu_s, mu_k
            ):
                any_contact = True
    return any_contact


# ---------------------------------------------------------------------------
# Simple pygame text-input field
# ---------------------------------------------------------------------------
class TextInput:
    """Minimal numeric text field (click to focus, type, Enter to commit)."""

    def __init__(
        self,
        rect: pygame.Rect,
        initial: str,
        label: str,
        allowed: str = "0123456789.-",
    ) -> None:
        self.rect = rect
        self.label = label
        self.allowed = allowed
        self.text = initial
        self.active = False
        self.committed = initial

    def handle_event(self, event: pygame.event.Event) -> bool:
        """
        Process an event.
        Returns True when the user presses Enter (value should be applied).
        """
        if event.type == pygame.MOUSEBUTTONDOWN:
            self.active = self.rect.collidepoint(event.pos)
            return False

        if not self.active:
            return False

        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_RETURN:
                self.active = False
                return True
            if event.key == pygame.K_BACKSPACE:
                self.text = self.text[:-1]
            elif event.key == pygame.K_ESCAPE:
                self.text = self.committed
                self.active = False
            else:
                ch = event.unicode
                if ch in self.allowed:
                    self.text += ch
        return False

    def draw(self, surf: pygame.Surface, font: pygame.font.Font) -> None:
        # label
        lbl = font.render(self.label, True, (180, 185, 200))
        surf.blit(lbl, (self.rect.x, self.rect.y - 18))

        # box
        bg = (50, 55, 70) if self.active else (35, 38, 48)
        border = (120, 180, 255) if self.active else (80, 85, 100)
        pygame.draw.rect(surf, bg, self.rect, border_radius=3)
        pygame.draw.rect(surf, border, self.rect, 2, border_radius=3)

        # text
        txt = font.render(self.text, True, (230, 230, 240))
        surf.blit(txt, (self.rect.x + 6, self.rect.y + 5))

    def get_float(self, default: float) -> float:
        try:
            return float(self.text)
        except ValueError:
            return default


# ---------------------------------------------------------------------------
# Rendering helpers
# ---------------------------------------------------------------------------
def world_to_screen(
    pos: np.ndarray,
    centroid: np.ndarray,
    ppm: float,
    cx: int,
    cy: int,
) -> Tuple[int, int]:
    """World (X right, Y up) → screen, view centred on contour centroid."""
    sx = int(cx + (pos[0] - centroid[0]) * ppm)
    sy = int(cy - (pos[1] - centroid[1]) * ppm)
    return sx, sy


def draw_arrow(
    surf: pygame.Surface,
    start: Tuple[int, int],
    vec_world: np.ndarray,
    ppm: float,
    color: Tuple[int, int, int],
    scale: float = 0.12,
) -> None:
    end = (
        start[0] + int(vec_world[0] * ppm * scale),
        start[1] - int(vec_world[1] * ppm * scale),
    )
    pygame.draw.line(surf, color, start, end, 2)
    pygame.draw.circle(surf, color, end, 3)


# ---------------------------------------------------------------------------
# Main application
# ---------------------------------------------------------------------------
def main() -> None:
    pygame.init()
    pygame.display.set_caption("Stalk ↔ Contour Drive – High-Fidelity Contact Test")
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("Consolas", 16)
    font_small = pygame.font.SysFont("Consolas", 14)
    font_ui = pygame.font.SysFont("Consolas", 15)

    # ------------------------------------------------------------------
    # Load contour
    # ------------------------------------------------------------------
    try:
        points = load_contour_from_csv(CONTOUR_CSV)
    except Exception as exc:
        print(f"\nERROR loading contour: {exc}")
        print(f"Edit CONTOUR_CSV (currently '{CONTOUR_CSV}') or the loader function.")
        pygame.quit()
        sys.exit(1)

    n_original = len(points)
    if CONTOUR_RESAMPLE_POINTS > 1:
        points = resample_contour_even_y(points, CONTOUR_RESAMPLE_POINTS)
        print(f"Resampled contour from {n_original} → {len(points)} points "
              f"(evenly spaced in Y)")

    contour = Contour(points)          # <-- create AFTER resampling
    print(f"Using contour: {len(contour.points)} points, "
          f"extent ≈ {contour.extent:.3f} m, "
          f"centroid ({contour.centroid[0]:.3f}, {contour.centroid[1]:.3f})")

    # Auto-scale so contour + margin fills most of the window
    margin = 2.5 * RADIUS + 0.08
    extent = contour.extent + 2.0 * margin
    ppm = min(WINDOW_WIDTH, WINDOW_HEIGHT) * 0.78 / max(extent, 0.05)
    cx, cy = WINDOW_WIDTH // 2, WINDOW_HEIGHT // 2

    # ------------------------------------------------------------------
    # Simulation objects
    # ------------------------------------------------------------------
    stalk = Stalk(mass=MASS, radius=RADIUS, k=K_SPRING, b=B_DAMPER)

    # Mutable live parameters
    restitution = RESTITUTION
    mu_s = MU_STATIC
    mu_k = MU_KINETIC
    drive_speed = DEFAULT_DRIVE_SPEED          # m/s (positive)
    lateral_x   = DEFAULT_LATERAL_X

    # Text-entry fields
    speed_box = TextInput(
        pygame.Rect(12, 210, 110, 26),
        f"{drive_speed:.3f}",
        "Drive speed (m/s)",
    )
    x_box = TextInput(
        pygame.Rect(12, 260, 110, 26),
        f"{lateral_x:.3f}",
        "Lateral x (m)",
    )

    paused = False
    contact_flag = False
    running = True

    def apply_and_reset() -> None:
        """Read the text fields, clamp, and restart the run."""
        nonlocal drive_speed, lateral_x
        speed = speed_box.get_float(drive_speed)
        xoff  = x_box.get_float(lateral_x)
        drive_speed = max(1e-4, abs(speed))          # always positive
        lateral_x   = xoff
        speed_box.text = speed_box.committed = f"{drive_speed:.3f}"
        x_box.text     = x_box.committed     = f"{lateral_x:.3f}"

        # Spawn well above the contour
        y_start = contour.y_max + 0.10 + RADIUS
        origin = np.array([lateral_x, y_start], dtype=np.float64)
        origin_vel = np.array([0.0, -drive_speed], dtype=np.float64)
        stalk.reset(origin, origin_vel)

    # Initial placement
    apply_and_reset()

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------
    while running:
        frame_dt = clock.tick(TARGET_FPS) / 1000.0
        frame_dt = min(frame_dt, 0.05)

        # ---- events ---------------------------------------------------
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_r:
                    apply_and_reset()
                elif event.key == pygame.K_SPACE:
                    paused = not paused
                elif event.key == pygame.K_1:
                    stalk.k = max(1.0, stalk.k * 0.8)
                elif event.key == pygame.K_2:
                    stalk.k *= 1.25
                elif event.key == pygame.K_3:
                    stalk.b = max(0.0, stalk.b * 0.8)
                elif event.key == pygame.K_4:
                    stalk.b *= 1.25
                elif event.key == pygame.K_5:
                    restitution = max(0.0, restitution - 0.05)
                elif event.key == pygame.K_6:
                    restitution = min(1.0, restitution + 0.05)
                elif event.key == pygame.K_7:
                    mu_s = max(0.0, mu_s - 0.05)
                elif event.key == pygame.K_8:
                    mu_s = min(2.0, mu_s + 0.05)
                elif event.key == pygame.K_9:
                    mu_k = max(0.0, mu_k - 0.05)
                elif event.key == pygame.K_0:
                    mu_k = min(2.0, mu_k + 0.05)

            # text fields
            if speed_box.handle_event(event):
                apply_and_reset()
            if x_box.handle_event(event):
                apply_and_reset()

        # ---- physics (fixed-step, accuracy-biased) --------------------
        if not paused:
            accumulator = frame_dt
            substeps = 0
            contact_flag = False
            while accumulator >= PHYSICS_DT and substeps < MAX_SUBSTEPS:
                # Drive the origin
                stalk.origin_pos += stalk.origin_vel * PHYSICS_DT

                # Integrate free motion
                stalk.integrate(PHYSICS_DT)

                # Multi-contact resolution
                if resolve_circle_contour(
                    stalk, contour, restitution, mu_s, mu_k
                ):
                    contact_flag = True

                accumulator -= PHYSICS_DT
                substeps += 1

        # ---- rendering ------------------------------------------------
        screen.fill((22, 25, 32))

        # Subtle grid centred on contour centroid
        grid_color = (38, 42, 52)
        grid_step = 0.05  # m
        # determine visible world range
        half_w = (WINDOW_WIDTH  / 2) / ppm
        half_h = (WINDOW_HEIGHT / 2) / ppm
        x0 = contour.centroid[0] - half_w
        x1 = contour.centroid[0] + half_w
        y0 = contour.centroid[1] - half_h
        y1 = contour.centroid[1] + half_h
        gx = np.floor(x0 / grid_step) * grid_step
        while gx < x1:
            sx, _ = world_to_screen(
                np.array([gx, 0.0]), contour.centroid, ppm, cx, cy
            )
            pygame.draw.line(screen, grid_color, (sx, 0), (sx, WINDOW_HEIGHT), 1)
            gx += grid_step
        gy = np.floor(y0 / grid_step) * grid_step
        while gy < y1:
            _, sy = world_to_screen(
                np.array([0.0, gy]), contour.centroid, ppm, cx, cy
            )
            pygame.draw.line(screen, grid_color, (0, sy), (WINDOW_WIDTH, sy), 1)
            gy += grid_step

        # Contour polyline
        if len(contour.points) >= 2:
            pts_screen = [
                world_to_screen(p, contour.centroid, ppm, cx, cy)
                for p in contour.points
            ]
            pygame.draw.lines(screen, (200, 200, 120), False, pts_screen, 1)
            # for p in pts_screen:
            #     pygame.draw.circle(screen, (200, 200, 120), p, 3)

        # Centroid marker
        c_s = world_to_screen(contour.centroid, contour.centroid, ppm, cx, cy)
        pygame.draw.circle(screen, (160, 160, 90), c_s, 5)
        pygame.draw.line(screen, (160, 160, 90), (c_s[0]-10, c_s[1]), (c_s[0]+10, c_s[1]), 1)
        pygame.draw.line(screen, (160, 160, 90), (c_s[0], c_s[1]-10), (c_s[0], c_s[1]+10), 1)

        # Spring (origin → stalk)
        origin_s = world_to_screen(stalk.origin_pos, contour.centroid, ppm, cx, cy)
        stalk_s  = world_to_screen(stalk.pos, contour.centroid, ppm, cx, cy)
        pygame.draw.line(screen, (90, 150, 210), origin_s, stalk_s, 2)
        pygame.draw.circle(screen, (180, 180, 100), origin_s, 4)

        # Stalk disk
        r_px = max(4, int(stalk.radius * ppm))
        color = (220, 85, 85) if contact_flag else (80, 175, 220)
        pygame.draw.circle(screen, color, stalk_s, r_px)
        pygame.draw.circle(screen, (25, 25, 35), stalk_s, r_px, 2)

        # Velocity arrow
        draw_arrow(screen, stalk_s, stalk.vel, ppm, (255, 200, 70), scale=0.10)

        # ---- HUD ------------------------------------------------------
        def blit(text: str, y: int, col=(195, 198, 210)) -> None:
            screen.blit(font.render(text, True, col), (12, y))

        mph = drive_speed * 2.2369362920544
        blit(f"FPS: {clock.get_fps():5.1f}   dt_phys: {PHYSICS_DT*1000:.2f} ms   "
             f"substeps max {MAX_SUBSTEPS}   contact iters {CONTACT_ITERATIONS}", 8)
        blit(f"Stalk  pos: ({stalk.pos[0]:+.4f}, {stalk.pos[1]:+.4f}) m", 28)
        blit(f"Stalk  vel: ({stalk.vel[0]:+.3f}, {stalk.vel[1]:+.3f}) m/s   "
             f"|v|={np.linalg.norm(stalk.vel):.3f}", 48)
        blit(f"Origin pos: ({stalk.origin_pos[0]:+.4f}, {stalk.origin_pos[1]:+.4f}) m", 68)
        blit(f"k = {stalk.k:7.1f} N/m    b = {stalk.b:5.2f} N·s/m", 88)
        blit(f"e = {restitution:.2f}   μs = {mu_s:.2f}   μk = {mu_k:.2f}", 108)
        blit(f"Drive: {drive_speed:.3f} m/s  ({mph:.2f} mph)   "
             f"lateral x = {lateral_x:+.3f} m", 128)
        blit(f"Contact bias = {CONTACT_BIAS_FACTOR:.2f}   slop = {CONTACT_SLOP*1e3:.2f} mm", 148)

        status = "CONTACT" if contact_flag else "free"
        status_col = (255, 110, 110) if contact_flag else (110, 210, 140)
        blit(f"State: {status}   {'[PAUSED]' if paused else ''}", 168, status_col)

        # Text-entry fields
        speed_box.draw(screen, font_ui)
        x_box.draw(screen, font_ui)
        # mph reminder next to speed box
        mph_surf = font_small.render(f"= {mph:.2f} mph", True, (150, 155, 170))
        screen.blit(mph_surf, (speed_box.rect.right + 8, speed_box.rect.y + 5))

        # Help footer
        help_lines = [
            "Click fields → type → Enter to Apply & Reset     R: reset     Space: pause",
            "1/2: k↓/↑   3/4: b↓/↑   5/6: e↓/↑   7/8: μs↓/↑   9/0: μk↓/↑",
        ]
        for i, line in enumerate(help_lines):
            surf = font_small.render(line, True, (130, 135, 150))
            screen.blit(surf, (12, WINDOW_HEIGHT - 42 + i * 18))

        pygame.display.flip()

    pygame.quit()
    sys.exit(0)


if __name__ == "__main__":
    main()
