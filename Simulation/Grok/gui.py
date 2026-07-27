#!/usr/bin/env python3
"""
gui.py – Rendering helpers and text-input widget for the interactive apps.

Contains two coordinate mappings:
  - world_to_screen_drive  (rotated axes used by contour drive)
  - world_to_screen_hockey (standard axes used by hockey test)

and the shared TextInput class + draw_arrow variants.
"""

from __future__ import annotations

from typing import Tuple

import numpy as np
import pygame


# ---------------------------------------------------------------------------
# Simple pygame text-input field (exact from stalk_contour_drive.py)
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
# Coordinate transforms
# ---------------------------------------------------------------------------
def world_to_screen_drive(
    pos: np.ndarray, centroid: np.ndarray, ppm: float, cx: int, cy: int
) -> Tuple[int, int]:
    """
    World (+X up, +Y left) → screen, view centred on contour centroid.
    Exact mapping used by the original stalk_contour_drive.py.
    """
    sx = int(cx - (pos[1] - centroid[1]) * ppm)
    sy = int(cy - (pos[0] - centroid[0]) * ppm)
    return sx, sy


def world_to_screen_hockey(
    pos: np.ndarray, ppm: float, cx: int, cy: int
) -> Tuple[int, int]:
    """
    Convert world (X right, Y up) to pygame screen coordinates.
    Exact mapping used by the original stalk_hockey_test.py.
    """
    sx = int(cx + pos[0] * ppm)
    sy = int(cy - pos[1] * ppm)
    return sx, sy


# ---------------------------------------------------------------------------
# Arrow drawing
# ---------------------------------------------------------------------------
def draw_arrow_drive(
    surf: pygame.Surface,
    start: Tuple[int, int],
    vec_world: np.ndarray,
    ppm: float,
    color: Tuple[int, int, int],
    scale: float = 0.15,
) -> None:
    """Draw velocity/force arrow using the rotated drive mapping."""
    end_x = start[0] - int(vec_world[1] * ppm * scale)   # +Y → left
    end_y = start[1] - int(vec_world[0] * ppm * scale)   # +X → up
    pygame.draw.line(surf, color, start, (end_x, end_y), 2)
    pygame.draw.circle(surf, color, (end_x, end_y), 3)


def draw_arrow_hockey(
    surf: pygame.Surface,
    start: Tuple[int, int],
    vec_world: np.ndarray,
    ppm: float,
    color: Tuple[int, int, int],
    scale: float = 0.15,
) -> None:
    """Draw a simple velocity / force arrow (standard axes)."""
    end_x = start[0] + int(vec_world[0] * ppm * scale)
    end_y = start[1] - int(vec_world[1] * ppm * scale)
    pygame.draw.line(surf, color, start, (end_x, end_y), 2)
    pygame.draw.circle(surf, color, (end_x, end_y), 3)
