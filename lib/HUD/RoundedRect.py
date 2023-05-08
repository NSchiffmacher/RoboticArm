import pygame
from pygame.locals import *

from lib.Math.Vector import Vector2
from lib.Options import Container

import lib.HUD.custom_drawing as cd

from lib.HUD.CanvasItem import CanvasItem


class RoundedRect(CanvasItem):
    BOTTOM_LEFT     = 1
    BOTTOM_RIGHT    = 2
    TOP_RIGHT       = 4
    TOP_LEFT        = 8

    TOP             = TOP_RIGHT     | TOP_LEFT
    BOTTOM          = BOTTOM_RIGHT  | BOTTOM_LEFT
    RIGHT           = TOP_RIGHT     | BOTTOM_RIGHT
    LEFT            = TOP_LEFT      | BOTTOM_LEFT

    ALL             = TOP           | BOTTOM

    def __init__(self, parent, position: Vector2 or None = None, size: Vector2 or None = None, color:list or tuple or None = None, radius: int or None = None, corners_code: int or None = ALL):
        # Parent : surface or canvasitem
        self.init_canvas(parent)

        if position:
            self.set_position(position)
        else:
            self.position = None
        
        if size:
            self.set_size(size)
        else:
            self.size = None
        if radius:
            self.set_radius(radius)
        else:
            self.set_radius(0)

        if color:
            self.set_color(color)
        else:
            self.color = None

        self.set_corners(corners_code)

        # self.position           = Vector2(self.parent.size.x * self.position_ratio.x, self.parent.size.y * self.position_ratio.y)
        # self.size               = Vector2(self.parent.size.x * self.size_ratio.x, self.parent.size.y * self.size_ratio.y)

    # Color setters
    def set_color(self, color: list or tuple):
        if len(color) != 3:
            raise AttributeError(f'Colors must be RGB')

        for color_val in color:
            if not 0 <= color_val <= 255:
                raise AttributeError('Colors are RGB, from 0 to 255')
        
        self.color = color
        return self

    
    # Radius setters
    def set_radius(self, px : int):
        self.radius = px
        return self
    def set_radius_from_ratio(self, ratio):
        """
        From rect's width ratio
        """
        self.radius = self.size.x * ratio
        return self
    def set_radius_from_height_ratio(self, ratio):
        self.radius = self.size.y * ratio
        return self



    def set_corners(self, code):
        if 0 <= code <= 15:
            self.corners_code = code
        else:
            raise AttributeError('Code must be in [|0;15|]')
        return self


    def load(self):
        if self.color == None:
            raise AttributeError('Color not set')
        self.surface            = cd.rounded_surface(self.size, self.radius, self.color, self.corners_code)
        return self

    def draw(self):
        self.parent.surface.blit(self.surface, self.position.to_pygame())
        return self