"""pygame drawing helpers shared by writer.py and executor.py"""
import math, pygame
from common import *

BG, WALL_C, TEXT_C, NAV_C = (24, 26, 27), (140, 145, 150), (236, 240, 241), (241, 196, 15)


def init(caption):
    pygame.init()
    screen = pygame.display.set_mode(WINDOW_SIZE)
    pygame.display.set_caption(caption)
    return screen, pygame.font.SysFont("Consolas", 14), pygame.time.Clock()


def draw_world(screen, sources, show_sources):
    screen.fill(BG)
    if show_sources:
        for s in sources:
            if s["active"]:
                c = EVENT_CATALOG[s["type"]]["color"]
                pygame.draw.circle(screen, tuple(v // 3 for v in c), s["pos"], s["range"], 1)
                pygame.draw.circle(screen, tuple(v // 2 for v in c), s["pos"], 5)
    for w in WALLS: pygame.draw.rect(screen, WALL_C, w)


def draw_beacons(screen, beacons, dead=(), goal=None):
    prev = START_POSE[:2]
    for b in beacons:
        p = (int(b["x"]), int(b["y"]))
        pygame.draw.line(screen, (80, 80, 50), (int(prev[0]), int(prev[1])), p, 1)
        prev = p
    for b in beacons:
        p = (int(b["x"]), int(b["y"]))
        et = b["event_type"]
        col = EVENT_CATALOG[et]["color"] if et else NAV_C
        if b["id"] in dead:
            pygame.draw.circle(screen, (90, 90, 90), p, 6, 1)
            pygame.draw.line(screen, (200, 60, 60), (p[0] - 5, p[1] - 5), (p[0] + 5, p[1] + 5), 2)
            continue
        pygame.draw.circle(screen, col, p, 8 if et else 4)
        if et: pygame.draw.circle(screen, (255, 255, 255), p, 11, 1)
        if goal is not None and b["id"] == goal: pygame.draw.circle(screen, (255, 255, 255), p, 16, 2)


def draw_robot(screen, x, y, heading, color):
    pygame.draw.circle(screen, color, (int(x), int(y)), ROBOT_R)
    pygame.draw.line(screen, (255, 255, 255), (int(x), int(y)),
                     (int(x + 20 * math.cos(heading)), int(y + 20 * math.sin(heading))), 2)


def draw_lines(screen, font, lines, x, y, color=TEXT_C, dy=17):
    for i, s in enumerate(lines):
        screen.blit(font.render(s, True, color), (x, y + i * dy))
