#!/usr/bin/env python3
"""
physics.py – Shared high-fidelity 2-D physics engine for stalk ↔ contour / paddle.

Contains:
  - Geometry helpers (closest_point_on_segment, rotate90)
  - Stalk (circular mass-spring-damper body with optional moving origin)
  - LineSegment (kinematic paddle for hockey-style tests)
  - Contact resolution:
      * resolve_circle_static_segment / resolve_circle_contour
        (exact logic & parameters from original stalk_contour_drive.py)
      * resolve_circle_segment
        (exact logic & parameters from original stalk_hockey_test.py)
  - Headless simulate_drive(...) for optimization / evaluation use

All numerical constants that affect contact behaviour are taken verbatim from
the original scripts so that results remain identical after the restructure.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Tuple, Dict

import numpy as np


# ---------------------------------------------------------------------------
# Shared numerical defaults (contour-drive values are the reference for the
# main design path; hockey-specific values are kept inside its resolver)
# ---------------------------------------------------------------------------
MASS                = 1.0
RADIUS              = 0.020
K_SPRING            = 60.0
B_DAMPER            = 2.5
RESTITUTION         = 0.05
MU_STATIC           = 0.25
MU_KINETIC          = 0.10
VEL_STICK_THRESHOLD = 1e-2          # used by contour-drive path

PHYSICS_DT          = 1.0 / 2000.0
MAX_SUBSTEPS        = 80
CONTACT_ITERATIONS  = 3
CONTACT_BIAS_FACTOR = 0.15
CONTACT_SLOP        = 0.0003

# Hockey-test specific (kept for exact behavioural parity)
HOCKEY_MU_STATIC    = 0.40
HOCKEY_MU_KINETIC   = 0.20
HOCKEY_VEL_STICK    = 1e-3
HOCKEY_BIAS_FACTOR  = 0.2
HOCKEY_SLOP         = 0.0005
HOCKEY_PHYSICS_DT   = 1.0 / 1000.0
HOCKEY_MAX_SUBSTEPS = 30
SEGMENT_LENGTH      = 0.3


# ---------------------------------------------------------------------------
# Geometry helpers (identical in both original scripts)
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
# Stalk – circular body with moving spring origin (from contour_drive)
# ---------------------------------------------------------------------------
@dataclass
class Stalk:
    """
    Circular disk of mass `mass` whose centre is free.
    An isotropic linear spring-damper connects the centre to a (possibly moving)
    origin.  When origin is held at (0,0) with zero velocity the behaviour is
    identical to the original hockey-test Stalk.
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

    def reset(self, origin: Optional[np.ndarray] = None,
              origin_velocity: Optional[np.ndarray] = None) -> None:
        """
        Place stalk at the origin with matching velocity (zero relative motion).
        If origin is None, resets to world zero (hockey-style).
        """
        if origin is None:
            origin = np.zeros(2, dtype=np.float64)
        if origin_velocity is None:
            origin_velocity = np.zeros(2, dtype=np.float64)
        self.origin_pos[:] = origin
        self.origin_vel[:] = origin_velocity
        self.pos[:] = origin
        self.vel[:] = origin_velocity


# ---------------------------------------------------------------------------
# LineSegment – kinematic paddle (exact from hockey_test)
# ---------------------------------------------------------------------------
@dataclass
class LineSegment:
    """
    Finite line segment whose midpoint follows the mouse and whose orientation
    is set by the mouse wheel.  Linear and angular velocities are estimated
    from successive states so that the velocity of any contact point can be
    evaluated for impulse calculations.
    """
    length: float
    midpoint: np.ndarray = field(default_factory=lambda: np.zeros(2, dtype=np.float64))
    angle: float = 0.0          # radians, 0 = pointing along +X
    vel: np.ndarray = field(default_factory=lambda: np.zeros(2, dtype=np.float64))
    omega: float = 0.0          # rad/s

    # Internal history for finite-difference velocities
    _prev_mid: np.ndarray = field(default_factory=lambda: np.zeros(2, dtype=np.float64), repr=False)
    _prev_angle: float = field(default=0.0, repr=False)
    _initialized: bool = field(default=False, repr=False)

    def endpoints(self) -> Tuple[np.ndarray, np.ndarray]:
        half = 0.5 * self.length
        direction = np.array([np.cos(self.angle), np.sin(self.angle)])
        return (self.midpoint - half * direction,
                self.midpoint + half * direction)

    def update_kinematics(self, new_mid: np.ndarray, new_angle: float, dt: float) -> None:
        """Advance the kinematic state and estimate velocities by finite differences."""
        if not self._initialized or dt <= 0.0:
            self.midpoint = new_mid.copy()
            self.angle = new_angle
            self.vel[:] = 0.0
            self.omega = 0.0
            self._prev_mid = new_mid.copy()
            self._prev_angle = new_angle
            self._initialized = True
            return

        self.vel = (new_mid - self._prev_mid) / dt
        # Unwrap angle difference to [-pi, pi] for a sensible omega
        dtheta = (new_angle - self._prev_angle + np.pi) % (2.0 * np.pi) - np.pi
        self.omega = dtheta / dt

        self._prev_mid = self.midpoint.copy()
        self._prev_angle = self.angle % (2.0 * np.pi) - np.pi
        self.midpoint = new_mid.copy()
        self.angle = new_angle % (2.0 * np.pi) - np.pi

    def velocity_at(self, point: np.ndarray) -> np.ndarray:
        """Velocity of the rigid segment at an arbitrary world point."""
        r = point - self.midpoint
        return self.vel + self.omega * rotate90(r)


# ---------------------------------------------------------------------------
# Contact resolution – circle vs. static segment (exact from contour_drive)
# ---------------------------------------------------------------------------
def resolve_circle_static_segment(
    stalk: Stalk,
    a: np.ndarray,
    b: np.ndarray,
    restitution: float,
    mu_s: float,
    mu_k: float,
    bias_factor: float = CONTACT_BIAS_FACTOR,
    slop: float = CONTACT_SLOP,
    dt: float = PHYSICS_DT,
    vel_stick_threshold: float = VEL_STICK_THRESHOLD,
) -> bool:
    """
    Resolve a single circle-vs-static-segment contact.

    Returns True if an impulse was applied.
    The segment is treated as having infinite mass and zero velocity.
    Logic & coefficients are identical to the original stalk_contour_drive.py.
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
    if penetration > slop:
        bias = (bias_factor / dt) * (penetration - slop)

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

    if abs(j_t_required) <= max_static and v_t < vel_stick_threshold * 5.0:
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
    contour,                       # Contour object with .segments and .seg_aabb
    restitution: float,
    mu_s: float,
    mu_k: float,
    contact_iterations: int = CONTACT_ITERATIONS,
    bias_factor: float = CONTACT_BIAS_FACTOR,
    slop: float = CONTACT_SLOP,
    dt: float = PHYSICS_DT,
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

    for _ in range(contact_iterations):
        for i, (a, b) in enumerate(contour.segments):
            xmin, xmax, ymin, ymax = contour.seg_aabb[i]
            # Cheap AABB rejection
            if sx1 < xmin or sx0 > xmax or sy1 < ymin or sy0 > ymax:
                continue
            if resolve_circle_static_segment(
                stalk, a, b, restitution, mu_s, mu_k,
                bias_factor=bias_factor, slop=slop, dt=dt,
            ):
                any_contact = True
    return any_contact


# ---------------------------------------------------------------------------
# Contact resolution – circle vs. kinematic segment (exact from hockey_test)
# ---------------------------------------------------------------------------
def resolve_circle_segment(
    stalk: Stalk,
    segment: LineSegment,
    restitution: float,
    mu_s: float,
    mu_k: float,
    is_sticking: bool,
    bias_factor: float = HOCKEY_BIAS_FACTOR,
    slop: float = HOCKEY_SLOP,
    dt: float = HOCKEY_PHYSICS_DT,
    vel_stick_threshold: float = HOCKEY_VEL_STICK,
) -> tuple[bool, bool]:
    """
    Circle vs. kinematic segment with stick-slip Coulomb friction and
    velocity bias (Baumgarte) for stable, jitter-free resting contacts.

    Returns
    -------
    contacted : bool
        True if a contact impulse was applied.
    new_is_sticking : bool
        Updated friction regime after this resolution step.

    Logic is identical to the original stalk_hockey_test.py.
    """
    a, b = segment.endpoints()
    closest, _ = closest_point_on_segment(stalk.pos, a, b)
    delta = stalk.pos - closest
    dist = np.linalg.norm(delta)

    if dist >= stalk.radius - 1e-9:
        return False, False          # no contact → not sticking

    # Normal (from segment toward stalk centre)
    if dist > 1e-8:
        n = delta / dist
    else:
        direction = b - a
        n = rotate90(direction)
        n /= (np.linalg.norm(n) + 1e-12)

    penetration = stalk.radius - dist

    # Relative velocity at contact point
    v_rel = stalk.vel - segment.velocity_at(closest)
    v_n = np.dot(v_rel, n)

    # Only resolve if approaching or deeply penetrating
    if v_n > 0.0 and penetration < 1e-5:
        return False, is_sticking

    # ------------------------------------------------------------------
    # Velocity bias (Baumgarte stabilization)
    # ------------------------------------------------------------------
    bias = 0.0
    if penetration > slop:
        bias = (bias_factor / dt) * (penetration - slop)

    # ------------------------------------------------------------------
    # Normal impulse (kinematic segment → infinite mass)
    # ------------------------------------------------------------------
    j_n = -(v_n + bias) * stalk.mass
    if v_n < 0.0:
        j_n -= restitution * v_n * stalk.mass
    if j_n < 0.0:
        j_n = 0.0

    # ------------------------------------------------------------------
    # Tangential direction
    # ------------------------------------------------------------------
    v_t_vec = v_rel - v_n * n
    v_t = np.linalg.norm(v_t_vec)
    if v_t > 1e-12:
        t = v_t_vec / v_t
    else:
        t = np.zeros(2, dtype=np.float64)

    # ------------------------------------------------------------------
    # Stick / slip decision
    # ------------------------------------------------------------------
    new_sticking = is_sticking

    if is_sticking:
        # Try to enforce zero tangential relative velocity
        j_t_required = -np.dot(v_rel, t) * stalk.mass
        max_static = mu_s * j_n

        if abs(j_t_required) <= max_static:
            # Stay sticking
            j_t = j_t_required
            new_sticking = True
        else:
            # Break into sliding – use kinetic friction
            j_t = -np.sign(j_t_required) * mu_k * j_n
            new_sticking = False
    else:
        # Currently sliding – apply kinetic friction
        if v_t > vel_stick_threshold:
            j_t = -mu_k * j_n          # opposes velocity (t already points along v_t)
            new_sticking = False
        else:
            # Velocity is almost zero – attempt to enter sticking
            j_t_required = -np.dot(v_rel, t) * stalk.mass
            if abs(j_t_required) <= mu_s * j_n:
                j_t = j_t_required
                new_sticking = True
            else:
                j_t = -np.sign(j_t_required) * mu_k * j_n if j_t_required != 0 else 0.0
                new_sticking = False

    # Apply total impulse
    impulse = j_n * n + j_t * t
    stalk.apply_impulse(impulse)

    # Positional correction (kept for robustness; bias already does most of the work)
    stalk.pos += 0.8 * penetration * n

    return True, new_sticking


# ---------------------------------------------------------------------------
# Headless drive simulation (for evaluation / optimization)
# ---------------------------------------------------------------------------
def simulate_drive(
    contour,
    drive_speed: float = 0.10,
    lateral_x: float = -0.020,
    duration: float = 5.0,
    mass: float = MASS,
    radius: float = RADIUS,
    k: float = K_SPRING,
    b: float = B_DAMPER,
    restitution: float = RESTITUTION,
    mu_s: float = MU_STATIC,
    mu_k: float = MU_KINETIC,
    dt: float = PHYSICS_DT,
    record_every: int = 10,
) -> Dict[str, np.ndarray]:
    """
    Run a complete headless stalk-contour drive and return time histories.

    The integration / contact loop is identical to the interactive version in
    drive.py (same fixed step, same multi-iteration contact, same origin drive).

    Returns a dict with keys:
        t, pos_x, pos_y, vel_x, vel_y, acc_x, acc_y, origin_y, contact
    """
    stalk = Stalk(mass=mass, radius=radius, k=k, b=b)

    y_start = contour.y_max + radius
    origin = np.array([lateral_x, y_start], dtype=np.float64)
    origin_vel = np.array([0.0, -drive_speed], dtype=np.float64)
    stalk.reset(origin, origin_vel)

    n_steps = int(duration / dt) + 1
    n_rec = (n_steps // record_every) + 1

    t_hist      = np.empty(n_rec, dtype=np.float64)
    pos_x_hist  = np.empty(n_rec, dtype=np.float64)
    pos_y_hist  = np.empty(n_rec, dtype=np.float64)
    vel_x_hist  = np.empty(n_rec, dtype=np.float64)
    vel_y_hist  = np.empty(n_rec, dtype=np.float64)
    acc_x_hist  = np.empty(n_rec, dtype=np.float64)
    acc_y_hist  = np.empty(n_rec, dtype=np.float64)
    origin_y_hist = np.empty(n_rec, dtype=np.float64)
    contact_hist = np.empty(n_rec, dtype=bool)

    rec = 0
    prev_vel = stalk.vel.copy()

    for step in range(n_steps):
        # Drive the origin (identical to interactive loop)
        stalk.origin_pos += stalk.origin_vel * dt

        # Integrate free motion
        stalk.integrate(dt)

        # Multi-contact resolution
        contacted = resolve_circle_contour(
            stalk, contour, restitution, mu_s, mu_k, dt=dt
        )

        # Acceleration estimate (central difference on recorded points later)
        if step % record_every == 0 and rec < n_rec:
            t_hist[rec] = step * dt
            pos_x_hist[rec] = stalk.pos[0]
            pos_y_hist[rec] = stalk.pos[1]
            vel_x_hist[rec] = stalk.vel[0]
            vel_y_hist[rec] = stalk.vel[1]
            # simple forward difference for accel at record rate
            acc = (stalk.vel - prev_vel) / (dt * record_every)
            acc_x_hist[rec] = acc[0]
            acc_y_hist[rec] = acc[1]
            origin_y_hist[rec] = stalk.origin_pos[1]
            contact_hist[rec] = contacted
            prev_vel = stalk.vel.copy()
            rec += 1

        # Optional early stop once well past the contour
        if stalk.origin_pos[1] < contour.y_min - 3.0 * radius:
            break

    # Trim to actual recorded length
    return {
        "t": t_hist[:rec],
        "pos_x": pos_x_hist[:rec],
        "pos_y": pos_y_hist[:rec],
        "vel_x": vel_x_hist[:rec],
        "vel_y": vel_y_hist[:rec],
        "acc_x": acc_x_hist[:rec],
        "acc_y": acc_y_hist[:rec],
        "origin_y": origin_y_hist[:rec],
        "contact": contact_hist[:rec],
    }
