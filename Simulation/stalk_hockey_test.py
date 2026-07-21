#!/usr/bin/env python3
"""
Interactive GUI for testing 2-D stalk (mass-spring-damper) dynamics against a
mouse-controlled line-segment "paddle".

Purpose
-------
This is the first incremental step toward the full stalk_trib_sim described in
the project header.  The stalk is a circular disk of mass m connected to a fixed
origin by an isotropic linear spring (k) and damper (b).  A finite line segment
is attached to the mouse cursor; its orientation is controlled by the mouse
wheel.  Cursor linear velocity and segment angular velocity are tracked so that
contact impulses are physically consistent.

Velocity bias (Baumgarte stabilization) has been added to the contact resolver
to produce relaxed, natural-feeling resting contacts and eliminate the previous
high-frequency "billiard-ball" jitter while preserving accurate impact response.

The same data structures (Stalk, LineSegment, world-to-screen mapping, fixed-
timestep integrator, impulse-based contact with Coulomb friction) are deliberately
kept modular so they can later be reused when:
  - the single segment is replaced by a full probe polyline (from LinearCamContour),
  - the origin is driven with constant -vy + lateral jitter,
  - static/kinetic friction regimes and multiple simultaneous contacts appear.

Controls
--------
  Mouse motion      : move the paddle
  Mouse wheel       : rotate the paddle
  R                 : reset stalk to origin at rest
  Space             : pause / unpause physics
  1 / 2             : decrease / increase spring stiffness k
  3 / 4             : decrease / increase damping b
  5 / 6             : decrease / increase coefficient of restitution
  7 / 8             : decrease / increase friction coefficient mu
  Esc / close window: quit

Units
-----
All physics is performed in SI (metres, kilograms, seconds).  Rendering uses a
pixels-per-metre scale so that the on-screen size feels natural while the
numbers remain engineering-relevant.
"""

from __future__ import annotations

import sys
from dataclasses import dataclass, field
from typing import Optional, Tuple

import numpy as np
import pygame


# ---------------------------------------------------------------------------
# Tunable physical parameters (SI units)
# ---------------------------------------------------------------------------
MASS = 1.0          # kg  – stalk mass scale
RADIUS = 0.020       # m   – stalk radius
K_SPRING = 60.0      # N/m – spring stiffness
B_DAMPER = 2.5       # N·s/m – viscous damping
RESTITUTION = 0.05   # coefficient of restitution (normal)

MU_STATIC  = 0.40   # static friction coefficient
MU_KINETIC = 0.20   # kinetic friction coefficient
VEL_STICK_THRESHOLD = 1e-3   # m/s – below this we may enter sticking

SEGMENT_LENGTH = 0.3  # m – length of the mouse-controlled paddle

# Numerical
PHYSICS_DT = 1.0 / 1000.0   # fixed physics timestep (s) – high for contact accuracy
MAX_SUBSTEPS = 30           # safety limit per rendered frame
PIXELS_PER_METRE = 450.0   # display scale
WINDOW_WIDTH = 1100
WINDOW_HEIGHT = 750

# Contact stabilization (velocity bias for relaxed resting contacts)
CONTACT_BIAS_FACTOR = 0.2   # Baumgarte factor (0.1–0.3 typical; higher = stronger push-out)
CONTACT_SLOP = 0.0005       # m – residual penetration allowed before bias activates


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------
def closest_point_on_segment(point: np.ndarray,
                             a: np.ndarray,
                             b: np.ndarray) -> Tuple[np.ndarray, float]:
    """
    Return the closest point on the closed segment [a,b] to 'point'
    and the parametric coordinate t ∈ [0,1].
    """
    ab = b - a
    ab_len2 = np.dot(ab, ab)
    if ab_len2 < 1e-12:
        return a.copy(), 0.0
    t = np.clip(np.dot(point - a, ab) / ab_len2, 0.0, 1.0)
    return a + t * ab, t


def rotate90(v: np.ndarray) -> np.ndarray:
    """Counter-clockwise 90° rotation (for 2-D cross product with z-axis)."""
    return np.array([-v[1], v[0]])


# ---------------------------------------------------------------------------
# Stalk – circular rigid body attached to fixed origin by spring-damper
# ---------------------------------------------------------------------------
@dataclass
class Stalk:
    """
    2-D circular disk whose centre is free.  The only force that always acts is
    the linear spring-damper connecting the centre to the fixed world origin.
    External impulses (from contact) are applied directly to velocity.
    """
    mass: float
    radius: float
    k: float
    b: float
    pos: np.ndarray = field(default_factory=lambda: np.zeros(2))
    vel: np.ndarray = field(default_factory=lambda: np.zeros(2))

    def spring_damper_force(self) -> np.ndarray:
        """Force exerted on the stalk by the spring-damper (origin is (0,0))."""
        return -self.k * self.pos - self.b * self.vel

    def integrate(self, dt: float) -> None:
        """
        Semi-implicit Euler step with only the spring-damper force.
        Contact impulses are applied separately after integration.
        """
        acc = self.spring_damper_force() / self.mass
        self.vel += acc * dt
        self.pos += self.vel * dt

    def apply_impulse(self, impulse: np.ndarray) -> None:
        """Instantaneous change of linear momentum (contact)."""
        self.vel += impulse / self.mass

    def reset(self) -> None:
        self.pos[:] = 0.0
        self.vel[:] = 0.0


# ---------------------------------------------------------------------------
# LineSegment – kinematic paddle controlled by mouse + wheel
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
    midpoint: np.ndarray = field(default_factory=lambda: np.zeros(2))
    angle: float = 0.0          # radians, 0 = pointing along +X
    vel: np.ndarray = field(default_factory=lambda: np.zeros(2))
    omega: float = 0.0          # rad/s

    # Internal history for finite-difference velocities
    _prev_mid: np.ndarray = field(default_factory=lambda: np.zeros(2), repr=False)
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
# Contact resolution (circle vs. kinematic segment)
# ---------------------------------------------------------------------------
def resolve_circle_segment(stalk: Stalk,
                           segment: LineSegment,
                           restitution: float,
                           mu_s: float,
                           mu_k: float,
                           is_sticking: bool) -> tuple[bool, bool]:
    """
    Circle vs. kinematic segment with stick-slip Coulomb friction and
    velocity bias (Baumgarte) for stable, jitter-free resting contacts.

    Returns
    -------
    contacted : bool
        True if a contact impulse was applied.
    new_is_sticking : bool
        Updated friction regime after this resolution step.
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
    # Gently drives residual penetration out through the velocity term
    # instead of relying solely on hard positional correction.  This
    # removes the high-frequency "billiard" jitter while leaving
    # high-speed impact response essentially unchanged.
    bias = 0.0
    if penetration > CONTACT_SLOP:
        bias = (CONTACT_BIAS_FACTOR / PHYSICS_DT) * (penetration - CONTACT_SLOP)

    # ------------------------------------------------------------------
    # Normal impulse (kinematic segment → infinite mass)
    # ------------------------------------------------------------------
    # Resolve the biased relative velocity; apply restitution only on
    # the true approaching component so that bias does not amplify bounce.
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
        t = np.zeros(2)

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
        if v_t > VEL_STICK_THRESHOLD:
            j_t = -mu_k * j_n          # opposes velocity (t already points along v_t)
            # (sign is already correct because t = v_t_vec / |v_t|)
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
# Rendering helpers
# ---------------------------------------------------------------------------
def world_to_screen(pos: np.ndarray, ppm: float, cx: int, cy: int) -> Tuple[int, int]:
    """Convert world (X right, Y up) to pygame screen coordinates."""
    sx = int(cx + pos[0] * ppm)
    sy = int(cy - pos[1] * ppm)
    return sx, sy


def draw_arrow(surf: pygame.Surface,
               start: Tuple[int, int],
               vec_world: np.ndarray,
               ppm: float,
               color: Tuple[int, int, int],
               scale: float = 0.15) -> None:
    """Draw a simple velocity / force arrow."""
    end_x = start[0] + int(vec_world[0] * ppm * scale)
    end_y = start[1] - int(vec_world[1] * ppm * scale)
    pygame.draw.line(surf, color, start, (end_x, end_y), 2)
    # tiny head
    pygame.draw.circle(surf, color, (end_x, end_y), 3)


# ---------------------------------------------------------------------------
# Main application
# ---------------------------------------------------------------------------
def main() -> None:
    pygame.init()
    pygame.display.set_caption("Stalk ↔ Mouse Paddle – Collision & Spring-Damper Test")
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("Consolas", 16)
    font_small = pygame.font.SysFont("Consolas", 14)

    # World origin is the centre of the window
    cx, cy = WINDOW_WIDTH // 2, WINDOW_HEIGHT // 2
    ppm = PIXELS_PER_METRE

    # Simulation objects
    stalk = Stalk(mass=MASS, radius=RADIUS, k=K_SPRING, b=B_DAMPER)
    paddle = LineSegment(length=SEGMENT_LENGTH)

    # Mutable parameters (can be changed live with keys)
    restitution = RESTITUTION
    mu_s = MU_STATIC
    mu_k = MU_KINETIC
    is_sticking = False
    paused = False
    contact_flag = False

    # Mouse state
    angle = 0.0
    running = True

    while running:
        frame_dt = clock.tick(120) / 1000.0  # seconds, capped at 60 FPS
        frame_dt = min(frame_dt, 0.05)      # avoid spiral of death

        # ------------------------------------------------------------------
        # Events
        # ------------------------------------------------------------------
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_r:
                    stalk.reset()
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
                    mu_s = min(1.5, mu_s + 0.05)
                elif event.key == pygame.K_9:
                    mu_k = max(0.0, mu_k - 0.05)
                elif event.key == pygame.K_0:
                    mu_k = min(1.5, mu_k + 0.05)
            elif event.type == pygame.MOUSEWHEEL:
                # Positive y = wheel up → increase angle (CCW)
                angle += event.y * 0.12

        # ------------------------------------------------------------------
        # Mouse → world
        # ------------------------------------------------------------------
        mx, my = pygame.mouse.get_pos()
        mouse_world = np.array([(mx - cx) / ppm, (cy - my) / ppm])

        # ------------------------------------------------------------------
        # Physics sub-steps (fixed dt for stability & accuracy)
        # ------------------------------------------------------------------
        if not paused:
            # Update paddle kinematics once per frame (constant during sub-steps)
            paddle.update_kinematics(mouse_world, angle, frame_dt)

            accumulator = frame_dt
            substeps = 0
            contact_flag = False
            while accumulator >= PHYSICS_DT and substeps < MAX_SUBSTEPS:
                stalk.integrate(PHYSICS_DT)
                contacted, is_sticking = resolve_circle_segment(
                    stalk, paddle, restitution, mu_s, mu_k, is_sticking
                )
                if contacted:
                    contact_flag = True
                accumulator -= PHYSICS_DT
                substeps += 1
        else:
            # Still keep paddle under mouse while paused
            paddle.update_kinematics(mouse_world, angle, frame_dt)

        # ------------------------------------------------------------------
        # Rendering
        # ------------------------------------------------------------------
        screen.fill((25, 28, 35))

        # Subtle grid
        grid_color = (40, 44, 55)
        for gx in range(-4, 5):
            x = cx + int(gx * 0.25 * ppm)
            pygame.draw.line(screen, grid_color, (x, 0), (x, WINDOW_HEIGHT), 1)
        for gy in range(-3, 4):
            y = cy - int(gy * 0.25 * ppm)
            pygame.draw.line(screen, grid_color, (0, y), (WINDOW_WIDTH, y), 1)

        # Origin marker
        pygame.draw.circle(screen, (180, 180, 100), (cx, cy), 5)
        pygame.draw.line(screen, (180, 180, 100), (cx - 12, cy), (cx + 12, cy), 2)
        pygame.draw.line(screen, (180, 180, 100), (cx, cy - 12), (cx, cy + 12), 2)

        # Spring (simple line; later can be a coil)
        stalk_screen = world_to_screen(stalk.pos, ppm, cx, cy)
        pygame.draw.line(screen, (100, 160, 220), (cx, cy), stalk_screen, 2)

        # Stalk disk
        r_px = max(3, int(stalk.radius * ppm))
        color = (220, 90, 90) if contact_flag else (90, 180, 220)
        pygame.draw.circle(screen, color, stalk_screen, r_px)
        pygame.draw.circle(screen, (30, 30, 40), stalk_screen, r_px, 2)

        # Velocity arrow of stalk
        draw_arrow(screen, stalk_screen, stalk.vel, ppm, (255, 200, 80), scale=0.12)

        # Paddle segment
        a, b = paddle.endpoints()
        a_s = world_to_screen(a, ppm, cx, cy)
        b_s = world_to_screen(b, ppm, cx, cy)
        pygame.draw.line(screen, (220, 220, 100), a_s, b_s, 5)
        # End caps
        pygame.draw.circle(screen, (220, 220, 100), a_s, 4)
        pygame.draw.circle(screen, (220, 220, 100), b_s, 4)

        # ------------------------------------------------------------------
        # HUD
        # ------------------------------------------------------------------
        def blit_text(text: str, y: int, color=(200, 200, 210)) -> None:
            surf = font.render(text, True, color)
            screen.blit(surf, (12, y))

        blit_text(f"FPS: {clock.get_fps():5.1f}   Physics dt: {PHYSICS_DT*1000:.1f} ms", 8)
        blit_text(f"Stalk pos: ({stalk.pos[0]:+.3f}, {stalk.pos[1]:+.3f}) m", 28)
        blit_text(f"Stalk vel: ({stalk.vel[0]:+.2f}, {stalk.vel[1]:+.2f}) m/s   |v|={np.linalg.norm(stalk.vel):.2f}", 48)
        blit_text(f"k = {stalk.k:6.1f} N/m    b = {stalk.b:5.2f} N·s/m", 68)
        blit_text(f"e = {restitution:.2f}   μs = {mu_s:.2f}   μk = {mu_k:.2f}   {'STICK' if is_sticking else 'SLIP'}", 88)
        blit_text(f"Paddle angle: {np.degrees((angle + np.pi) % (2 * np.pi) - np.pi):+.1f}°   omega: {paddle.omega:+.1f} rad/s", 108)
        blit_text(f"Contact bias = {CONTACT_BIAS_FACTOR:.2f}   slop = {CONTACT_SLOP*1000:.1f} mm", 128)

        status = "CONTACT" if contact_flag else "free"
        status_color = (255, 120, 120) if contact_flag else (120, 220, 140)
        blit_text(f"State: {status}   {'[PAUSED]' if paused else ''}", 148, status_color)

        # Help footer
        help_lines = [
            "R: reset   Space: pause   Wheel: rotate paddle",
            "1/2: k↓/↑   3/4: b↓/↑   5/6: e↓/↑   7/8: μs↓/↑   9/0: μk↓/↑",
        ]
        for i, line in enumerate(help_lines):
            surf = font_small.render(line, True, (140, 145, 160))
            screen.blit(surf, (12, WINDOW_HEIGHT - 42 + i * 18))

        pygame.display.flip()

    pygame.quit()
    sys.exit(0)


if __name__ == "__main__":
    main()