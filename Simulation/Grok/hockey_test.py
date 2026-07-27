#!/usr/bin/env python3
"""
hockey_test.py – Interactive GUI for testing stalk dynamics against a
mouse-controlled line-segment "paddle".

Restructured equivalent of the original stalk_hockey_test.py.
Physics objects and contact resolver are imported from physics.py;
rendering helpers come from gui.py.  All numerical parameters and the
stick/slip state machine remain exactly as in the original.
"""

from __future__ import annotations

import sys

import numpy as np
import pygame

from physics import (
    Stalk, LineSegment, resolve_circle_segment,
    MASS, RADIUS, K_SPRING, B_DAMPER, RESTITUTION,
    HOCKEY_MU_STATIC, HOCKEY_MU_KINETIC, HOCKEY_PHYSICS_DT,
    HOCKEY_MAX_SUBSTEPS, HOCKEY_BIAS_FACTOR, HOCKEY_SLOP,
    SEGMENT_LENGTH,
)
from gui import world_to_screen_hockey, draw_arrow_hockey


# Display
PIXELS_PER_METRE = 450.0
WINDOW_WIDTH = 1600
WINDOW_HEIGHT = 900


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
    mu_s = HOCKEY_MU_STATIC
    mu_k = HOCKEY_MU_KINETIC
    is_sticking = False
    paused = False
    contact_flag = False

    # Mouse state
    angle = 0.0
    running = True

    while running:
        frame_dt = clock.tick(120) / 1000.0
        frame_dt = min(frame_dt, 0.05)

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
                angle += event.y * 0.12

        # ------------------------------------------------------------------
        # Mouse → world
        # ------------------------------------------------------------------
        mx, my = pygame.mouse.get_pos()
        mouse_world = np.array([(mx - cx) / ppm, (cy - my) / ppm])

        # ------------------------------------------------------------------
        # Physics sub-steps
        # ------------------------------------------------------------------
        if not paused:
            paddle.update_kinematics(mouse_world, angle, frame_dt)

            accumulator = frame_dt
            substeps = 0
            contact_flag = False
            while accumulator >= HOCKEY_PHYSICS_DT and substeps < HOCKEY_MAX_SUBSTEPS:
                stalk.integrate(HOCKEY_PHYSICS_DT)
                contacted, is_sticking = resolve_circle_segment(
                    stalk, paddle, restitution, mu_s, mu_k, is_sticking
                )
                if contacted:
                    contact_flag = True
                accumulator -= HOCKEY_PHYSICS_DT
                substeps += 1
        else:
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

        # Spring
        stalk_screen = world_to_screen_hockey(stalk.pos, ppm, cx, cy)
        pygame.draw.line(screen, (100, 160, 220), (cx, cy), stalk_screen, 2)

        # Stalk disk
        r_px = max(3, int(stalk.radius * ppm))
        color = (220, 90, 90) if contact_flag else (90, 180, 220)
        pygame.draw.circle(screen, color, stalk_screen, r_px)
        pygame.draw.circle(screen, (30, 30, 40), stalk_screen, r_px, 2)

        # Velocity arrow of stalk
        draw_arrow_hockey(screen, stalk_screen, stalk.vel, ppm, (255, 200, 80), scale=0.12)

        # Paddle segment
        a, b = paddle.endpoints()
        a_s = world_to_screen_hockey(a, ppm, cx, cy)
        b_s = world_to_screen_hockey(b, ppm, cx, cy)
        pygame.draw.line(screen, (220, 220, 100), a_s, b_s, 5)
        pygame.draw.circle(screen, (220, 220, 100), a_s, 4)
        pygame.draw.circle(screen, (220, 220, 100), b_s, 4)

        # ------------------------------------------------------------------
        # HUD
        # ------------------------------------------------------------------
        def blit_text(text: str, y: int, color=(200, 200, 210)) -> None:
            surf = font.render(text, True, color)
            screen.blit(surf, (12, y))

        blit_text(f"FPS: {clock.get_fps():5.1f}   Physics dt: {HOCKEY_PHYSICS_DT*1000:.1f} ms", 8)
        blit_text(f"Stalk pos: ({stalk.pos[0]:+.3f}, {stalk.pos[1]:+.3f}) m", 28)
        blit_text(f"Stalk vel: ({stalk.vel[0]:+.2f}, {stalk.vel[1]:+.2f}) m/s   |v|={np.linalg.norm(stalk.vel):.2f}", 48)
        blit_text(f"k = {stalk.k:6.1f} N/m    b = {stalk.b:5.2f} N·s/m", 68)
        blit_text(f"e = {restitution:.2f}   μs = {mu_s:.2f}   μk = {mu_k:.2f}   {'STICK' if is_sticking else 'SLIP'}", 88)
        blit_text(f"Paddle angle: {np.degrees((angle + np.pi) % (2 * np.pi) - np.pi):+.1f}°   omega: {paddle.omega:+.1f} rad/s", 108)
        blit_text(f"Contact bias = {HOCKEY_BIAS_FACTOR:.2f}   slop = {HOCKEY_SLOP*1000:.1f} mm", 128)

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
