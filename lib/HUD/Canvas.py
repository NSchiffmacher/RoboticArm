import pygame
from pygame.locals import *

from lib.Math.Vector import Vector2
from lib.Options import Container

import lib.HUD.custom_drawing as cd

from lib.HUD.CanvasItem import CanvasItem


class Canvas(CanvasItem):
    TRANSPARENT = -1
    ALPHA_COLOR = [171,36,77]

    def __init__(self, parent, background_color: list or int or None = TRANSPARENT, position: Vector2 or None = None, size: Vector2 or None = None):
        self.init_canvas(parent)

        self.background_color   = background_color
        self.childs             = []
        self.draw_childs        = True


        if position:
            self.set_position(position)
        if size:
            self.set_size(size)
        
    def append_child(self, child, priority: int or None = None):
        self.childs.append({
            'object'  : child,
            'priority': priority if priority != None else 1
        })
        if priority != None:
            self.reorder_childs()

    def reorder_childs(self):
        self.childs.sort(key = lambda child: child['priority'])

    def load(self):
        self.surface = pygame.Surface(self.size.to_pygame(), DOUBLEBUF)
        self.fill()
        return self
    
    def load_childs(self):
        for child in self.childs:
            child['object'].load()
        return self

    def load_all(self):
        self.load()
        self.load_childs()
        return self
    
    def fill(self):
        if self.background_color == self.TRANSPARENT:
            self.surface.fill(self.ALPHA_COLOR)
            self.surface.set_colorkey(self.ALPHA_COLOR)
        else:
            self.surface.fill(self.background_color)
        return self
    
    def draw(self):
        self.fill()
        if self.draw_childs:
            for child in self.childs:
                child['object'].draw()
        self.blit_surface()
    
    def blit_surface(self):
        self.parent.surface.blit(self.surface, self.position.to_pygame())
