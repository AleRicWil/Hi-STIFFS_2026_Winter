#!/usr/bin/env python3
"""
drive.py – Interactive high-fidelity stalk ↔ fixed-contour drive simulation.

This is the restructured equivalent of the original stalk_contour_drive.py.
All physics, contact parameters, coordinate mapping, controls and HUD are
identical; only the code organisation has changed (imports from physics,
contour and gui).
"""

from __future__ import annotations

import os
import sys

import numpy as np
import pygame

from physics import (
    Stalk,
    resolve_circle_contour,
    MASS, RADIUS, K_SPRING, B_DAMPER, RESTITUTION,
    MU_STATIC, MU_KINETIC, PHYSICS_DT, MAX_SUBSTEPS,
    CONTACT_ITERATIONS, CONTACT_BIAS_FACTOR, CONTACT_SLOP,
)
from contour import load_contour_from_csv, resample_contour_even_y, Contour
from gui import TextInput, world_to_screen_drive, draw_arrow_drive


# ---------------------------------------------------------------------------
# User-configurable paths & defaults (identical to original)
# ---------------------------------------------------------------------------
dir = os.path.dirname(__file__)
CONTOUR_CSV = os.path.join(dir, "new_profile.csv")

DEFAULT_DRIVE_SPEED = 0.224*1       # m/s  (positive; actual velocity is -Y)
DEFAULT_LATERAL_X   = -0.020     # m

CONTOUR_RESAMPLE_POINTS = 1000

WINDOW_WIDTH  = 1600
WINDOW_HEIGHT = 900
TARGET_FPS    = 30


def main() -> None:
    pygame.init()
    pygame.display.set_caption("Stalk ↔ Contour Drive - High-Fidelity Contact Test")
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("Consolas", 16)
    font_small = pygame.font.SysFont("Consolas", 14)
    font_ui = pygame.font.SysFont("Consolas", 15)

    # ------------------------------------------------------------------
    # Load contour
    # ------------------------------------------------------------------
    csv_path = CONTOUR_CSV
    if not os.path.isfile(csv_path):
        # Fall back to the same directory as this script
        alt = os.path.join(os.path.dirname(__file__), csv_path)
        if os.path.isfile(alt):
            csv_path = alt
        else:
            print(f"\nERROR: Contour CSV not found: {CONTOUR_CSV}")
            print("Run edit_contour.py first to generate new_profile.csv,")
            print("or edit CONTOUR_CSV at the top of drive.py.")
            pygame.quit()
            sys.exit(1)

    try:
        points = load_contour_from_csv(csv_path)
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

    contour = Contour(points)
    print(f"Using contour: {len(contour.points)} points, "
          f"extent ≈ {contour.extent:.3f} m, "
          f"centroid ({contour.centroid[0]:.3f}, {contour.centroid[1]:.3f})")

    # Auto-scale so contour + margin fills most of the window
    margin = 0.5 * RADIUS + 0.05
    extent = contour.extent + 2.0 * margin
    ppm = min(WINDOW_WIDTH, WINDOW_HEIGHT) * 1.8 / max(extent, 0.05)
    cx, cy = WINDOW_WIDTH // 2, WINDOW_HEIGHT // 2

    # ------------------------------------------------------------------
    # Simulation objects
    # ------------------------------------------------------------------
    stalk = Stalk(mass=MASS, radius=RADIUS, k=K_SPRING, b=B_DAMPER)

    # Mutable live parameters
    restitution = RESTITUTION
    mu_s = MU_STATIC
    mu_k = MU_KINETIC
    drive_speed = DEFAULT_DRIVE_SPEED
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
        drive_speed = max(1e-4, abs(speed))
        lateral_x   = xoff
        speed_box.text = speed_box.committed = f"{drive_speed:.3f}"
        x_box.text     = x_box.committed     = f"{lateral_x:.3f}"

        # Spawn well above the contour
        y_start = contour.y_max + RADIUS
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
        grid_step = 0.1524  # m
        half_w = (WINDOW_WIDTH  / 2) / ppm
        half_h = (WINDOW_HEIGHT / 2) / ppm
        x0 = contour.centroid[0] - half_h
        x1 = contour.centroid[0] + half_h
        y0 = contour.centroid[1] - half_w
        y1 = contour.centroid[1] + half_w

        # Vertical screen lines = constant world-Y
        gy = np.floor(y0 / grid_step) * grid_step
        while gy < y1:
            sx, _ = world_to_screen_drive(
                np.array([0.0, gy]), contour.centroid, ppm, cx, cy
            )
            pygame.draw.line(screen, grid_color, (sx, 0), (sx, WINDOW_HEIGHT), 1)
            gy += grid_step

        # Horizontal screen lines = constant world-X
        gx = np.floor(x0 / grid_step) * grid_step
        while gx < x1:
            _, sy = world_to_screen_drive(
                np.array([gx, 0.0]), contour.centroid, ppm, cx, cy
            )
            pygame.draw.line(screen, grid_color, (0, sy), (WINDOW_WIDTH, sy), 1)
            gx += grid_step

        # Contour polyline
        if len(contour.points) >= 2:
            pts_screen = [
                world_to_screen_drive(p, contour.centroid, ppm, cx, cy)
                for p in contour.points
            ]
            pygame.draw.lines(screen, (200, 200, 120), False, pts_screen, 1)

        # Centroid marker
        c_s = world_to_screen_drive(contour.centroid, contour.centroid, ppm, cx, cy)
        pygame.draw.circle(screen, (160, 160, 90), c_s, 5)
        pygame.draw.line(screen, (160, 160, 90), (c_s[0]-10, c_s[1]), (c_s[0]+10, c_s[1]), 1)
        pygame.draw.line(screen, (160, 160, 90), (c_s[0], c_s[1]-10), (c_s[0], c_s[1]+10), 1)

        # Spring (origin → stalk)
        origin_s = world_to_screen_drive(stalk.origin_pos, contour.centroid, ppm, cx, cy)
        stalk_s  = world_to_screen_drive(stalk.pos, contour.centroid, ppm, cx, cy)
        pygame.draw.line(screen, (90, 150, 210), origin_s, stalk_s, 2)
        pygame.draw.circle(screen, (180, 180, 100), origin_s, 4)

        # Stalk disk
        r_px = max(4, int(stalk.radius * ppm))
        color = (220, 85, 85) if contact_flag else (80, 175, 220)
        pygame.draw.circle(screen, color, stalk_s, r_px)
        pygame.draw.circle(screen, (25, 25, 35), stalk_s, r_px, 2)

        # Velocity arrow
        draw_arrow_drive(screen, stalk_s, stalk.vel, ppm, (255, 200, 70), scale=0.10)

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
