import pygame
from pygame.locals import *

from lib.Math.Vector import Vector2

def rounded_surface(size : Vector2, radius : int, color: list or tuple, corners_code: int):
    ALPHA_COLOR = [111,184,241] # Devrait changer en fonction de color

    surface = pygame.Surface((size.x, size.y), DOUBLEBUF)
    surface.fill(ALPHA_COLOR)
    pygame.draw.rect(surface, color, (radius, 0, size.x - 2 * radius, size.y))
    pygame.draw.rect(surface, color, (0, radius, radius, size.y - 2 * radius))
    pygame.draw.rect(surface, color, (size.x - radius, radius, radius, size.y - 2 * radius))

    # Draw the circles in the corners
    # Top left
    if (corners_code >> 3) & 1:
        pygame.draw.circle(surface, color, (radius, radius), radius)
    else:
        pygame.draw.rect(surface, color, (0, 0, radius, radius))
    # Top right
    if (corners_code >> 2) & 1:
        pygame.draw.circle(surface, color, (size.x - radius, radius), radius)
    else:
        pygame.draw.rect(surface, color, (size.x - radius, 0, radius, radius))
    # Bottom right
    if (corners_code >> 1) & 1:
        pygame.draw.circle(surface, color, (size.x - radius, size.y - radius), radius)
    else:
        pygame.draw.rect(surface, color, (size.x - radius, size.y - radius, radius, radius))
    # Bottom left
    if (corners_code) & 1:
        pygame.draw.circle(surface, color, (radius, size.y - radius), radius)
    else:
        pygame.draw.rect(surface, color, (0, size.y - radius, radius, radius))

    surface.set_colorkey(ALPHA_COLOR)

    return surface
