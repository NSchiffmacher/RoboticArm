import pygame
from pygame.locals import *

from lib.Math.Vector import Vector2
from lib.Options import Container

import lib.HUD.custom_drawing as cd

from lib.HUD.CanvasItem import CanvasItem
from lib.HUD.Canvas import Canvas

class PopUp(Canvas):
    TRANSPARENT = Canvas.TRANSPARENT

    def __init__(self, parent, position: Vector2 or None = None, size: Vector2 or None = None):
        super().__init__(parent, self.TRANSPARENT, position, size)