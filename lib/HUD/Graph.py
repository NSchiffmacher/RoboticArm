import pygame
from pygame.locals import *

import math

from lib.Math.Vector import Vector2

class Curve:
    AXIS_KEYS = ['X', 'Y']
    RANGE_KEYS = ['MIN', 'MAX']

    MARKER_NONE     = 0
    MARKER_CIRCLE   = 1
    MARKER_X        = 2
    MARKER_CROSS    = 3


    def __init__(self, color: list or None = None, points: list or None = None, points_marker: int or None = None, line_width:int or None = None, display_size: Vector2 or None = None, range: dict or None = None):
        self.color = []
        if color:
            self.set_color(color)
        self.points = []
        if points:
            self.points = points

        self.range = {
            'X': {
                'MIN' : 0,
                'MAX' : 0
            },
            'Y' : {
                'MIN' : 0,
                'MAX' : 0
            }
        }
        if range:
            self.set_range(range)

        self.line_width = 1
        if line_width:
           self.set_line_width(line_width) 

        self.display_size = None
        if display_size:
            self.set_display_size(display_size)

        self.update_draw_points()
        self.update_draw_points_every_add = True



    def set_color(self, color: list or tuple):
        if type(color) in [list, tuple] and len(color) == 3:
            for color_comp in color:
                if not 0 <= color_comp <= 255:
                    raise AttributeError('Colors must be in [0;255]')
            self.color = color
        return self

    def set_range(self, range:dict):
        for axis_key in range:
            if axis_key not in self.AXIS_KEYS:
                raise AttributeError(f'Invalid axis key {axis_key}')
                
            for range_key in range[axis_key]:
                if range_key not in self.RANGE_KEYS:
                    raise AttributeError(f'Range key "{range_key}" not in {self.RANGE_KEYS}')

                self.range[axis_key][range_key] = range[axis_key][range_key]

        return self

    def set_line_width(self, width):
        self.line_width = int(width)
        return self

    def set_display_size(self, size:Vector2):
        self.display_size = size
        if self.update_draw_points_every_add:
            self.update_draw_points()
        return self

    def add_point(self, v: Vector2):
        self.points.append(v)
        if self.display_size and self.update_draw_points_every_add:
            self.update_draw_points()
        return self

    # def )
    def update_draw_points(self):
        self.draw_points = []

        # self.range['VALUE'] = self.range['MIN'] + (self.range['MAX'] - self.range['MIN']) * (self.cursor.position.x + self.cursor_radius - self.position.x) / self.size.x

        for point in self.points:
            p = Vector2(
                (point.x - self.range['X']['MIN']) / (self.range['X']['MAX'] - self.range['X']['MIN']) * (self.display_size.x),
                self.display_size.y - (point.y - self.range['Y']['MIN']) / (self.range['Y']['MAX'] - self.range['Y']['MIN']) * (self.display_size.y)
            )
            self.draw_points.append(p.to_int())

        
    def draw(self, surface):
        if len(self.draw_points) > 1:
            pointA = self.draw_points[0]
            for pointB in self.draw_points[1:]:

                pygame.draw.line(surface, self.color, pointA.to_pygame(), pointB.to_pygame(), self.line_width)

                pointA = pointB