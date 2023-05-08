import pygame
from pygame.locals import *

from lib.Math.Vector import Vector2
from lib.Options import Container

class CanvasItem:
    def init_canvas(self, parent):
        self.size               = None
        self.position           = None

        if type(parent) == pygame.Surface:
            self.parent             = Container()
            self.parent.size        = Vector2(parent.get_width(), parent.get_height())
            self.parent.surface     = parent
        else:
            self.parent             = parent


    # Position setters
    def set_position(self, position: Vector2):
        self.position = position
        return self
    def set_position_ratio(self, ratio: Vector2):
        self.set_position(Vector2(self.parent.size.x * ratio.x, self.parent.size.y * ratio.y))
        return self

    def set_position_by_center(self, center_position: Vector2):
        self.set_position(center_position - self.size / 2)
        return self
    def set_position_by_center_ratio(self, ratio: Vector2):
        self.set_position(Vector2(
            self.parent.size.x * ratio.x - self.size.x / 2,
            self.parent.size.y * ratio.y - self.size.x / 2
        ))
        return self

    def center(self):
        self.set_position(Vector2(
            self.parent.size.x / 2 - self.size.x / 2,
            self.parent.size.y / 2 - self.size.y / 2
        ))
        return self
    def center_horizontally(self, yposition: int):
        self.set_position(Vector2(
            self.parent.size.x / 2 - self.size.x / 2,
            yposition
        ))
        return self
    def center_horizontally_ratio(self, yratio: float):
        self.set_position(Vector2(
            self.parent.size.x / 2 - self.size.x / 2,
            yratio * self.parent.size.y
        ))
        return self
    def center_point_horizontally_ratio(self, yratio: float):
        self.set_position(Vector2(
            self.parent.size.x / 2 - self.size.x / 2,
            yratio * self.parent.size.y - self.size.y / 2
        ))
        return self
    def center_vertically(self, xposition: int):
        self.set_position(Vector2(
            xposition,
            self.parent.size.y / 2 - self.size.y / 2
        ))
        return self
    def center_vertically_ratio(self, xratio: float):
        self.set_position(Vector2(
            xratio * self.parent.size.x,
            self.parent.size.y / 2 - self.size.y / 2
        ))
        return self

    # Size changers
    def change_width(self, px: int):
        px = int(px)
        if self.size != None:
            self.size.x = px
        else:
            self.size = Vector2(px, 0)
        return self
    def change_width_ratio(self, wratio: float):
        self.change_width(wratio * self.parent.size.x)
        return self
    def change_height(self, px: int):
        px = int(px)
        if self.size != None:
            self.size.y = px
        else:
            self.size = Vector2(0, px)
        return self
    def change_height_ratio(self, hratio: float):
        self.change_height(hratio * self.parent.size.y)
        return self


    # Size setters
    def set_size(self, size: Vector2):
        if 0 <= size.x <= self.parent.size.x and 0 <= size.y <= self.parent.size.y:
            self.size = size
        elif 0 <= size.x <= self.parent.size.x:
            raise AttributeError('Illegal size parameter (height too big, or negative)')
        else:
            raise AttributeError('Illegal size parameter (too wide)')
        return self

    def set_size_from_ratio(self, ratio: Vector2):
        self.set_size(Vector2(int(self.parent.size.x * ratio.x), int(self.parent.size.y * ratio.y)))
        return self

    def square(self, px: int):
        self.set_size(Vector2(px))
        return
    def square_from_ratio(self, ratio: float):
        """
        Sets the size to be a square of size parents.size.x * ratio
        """
        self.set_size(Vector2(self.parent.size.x * ratio).to_int())
        return self
    def square_from_height_ratio(self, ratio: float):
        """
        Sets the size to be a square of size parents.size.y * ratio
        """
        self.set_size(Vector2(self.parent.size.y * ratio).to_int())
        return self

    def fit(self):
        self.set_size_from_ratio(Vector2(1,1))
        return self


    def parent_offset_px(self, parent):
        if type(parent) == Container:
            return Vector2(0,0)
        else:
            return parent.position + self.parent_offset_px(parent.parent)

    def add_to_canvas(self, canvas, priority: int or None = None):
        canvas.append_child(self, priority)
        return self