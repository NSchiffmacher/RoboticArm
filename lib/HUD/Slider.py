import pygame
from pygame.locals import *

import math

from lib.Math.Vector import Vector2

from lib.HUD.CanvasItem import CanvasItem
from lib.HUD.RoundedRect import RoundedRect
from lib.HUD.Button import NoTextButton


class Slider(CanvasItem):
    COLOR_KEYS = ['SLIDER', 'CURSOR_NORMAL', 'CURSOR_HOVER', 'CURSOR_PRESSED']
    RANGE_KEYS = ['MIN', 'MAX', 'VALUE', 'INCREMENT']
    CALLBACK_KEYS = ['VALUE_CHANGE']

    def __init__(self, parent: pygame.Surface, range: dict or None = None, colors:dict or None = None, cursor_height_ratio: float or None = None, callbacks: dict or None = None, position: Vector2 or None = None, size: Vector2 or None = None):
        self.init_canvas(parent)


        self.colors = {}
        for key in self.COLOR_KEYS:
            self.colors[key] = None
        if colors:
            self.set_colors(colors)

        self.slider = RoundedRect(parent, position=position, size=size, color=self.colors['SLIDER'])


        cursor_colors = {}
        for key in colors:
            if key.startswith('CURSOR_'):
                cursor_colors[key.replace('CURSOR_', '').lower()] = colors[key]
        self.cursor = NoTextButton(parent, colors=cursor_colors, position=position, size=size)
        
        

        self.cursor_radius = 0
        self.cursor_height_ratio = cursor_height_ratio
        if cursor_height_ratio:
            self.set_cursor_radius(cursor_height_ratio)



        if position:
            self.set_position(position)
        if size:
            self.set_size(size)
        self.loaded = False

        self.range = {
            'MIN': 0,
            'MAX': 1,
            'VALUE':0
        }
        if range:
            self.set_range(range)

        self.callbacks = {}
        for key in self.CALLBACK_KEYS:
            self.callbacks[key] = None
        if callbacks:
            self.set_callbacks(callbacks)

        self.holding_cursor = False

    def set_colors(self, colors):
        """
        COLORS KEYS are:
            - SLIDER,
            - CURSOR
        """

        for color_key in colors:
            if color_key not in self.COLOR_KEYS:
                raise AttributeError(f'Color key "{color_key}" not in {self.COLOR_KEYS}')
            self.colors[color_key] = colors[color_key]
        return self

    def set_range(self, range):
        """
        RANGE KEYS are:
            - MIN,
            - MAX,
            - VALUE
        """

        for range_key in range:
            if range_key not in self.RANGE_KEYS:
                raise AttributeError(f'Range key "{color_key}" not in {self.RANGE_KEYS}')
            self.range[range_key] = range[range_key]

        self.update_cursor_position()
        return self

    def set_callbacks(self, callbacks: dict):
        for callback_key in callbacks:
            if callback_key not in self.CALLBACK_KEYS:
                raise AttributeError(f'Callback key "{callback_key}" not in {self.CALLBACK_KEYS}')
            self.callbacks[callback_key] = callbacks[callback_key]
        return self

    def update_cursor_position(self):
        try:
            self.cursor.set_position_by_center(Vector2(
                (self.range['VALUE'] - self.range['MIN']) / (self.range['MAX'] - self.range['MIN']) * self.size.x + self.position.x,
                self.position.y + self.size.y / 2
            ))
            return self
        except:
            raise SystemError('You must set the size and position first')
    
    def calculate_value(self):
        last_val = float(self.range['VALUE'])

        self.range['VALUE'] = self.range['MIN'] + (self.range['MAX'] - self.range['MIN']) * (self.cursor.position.x + self.cursor_radius - self.position.x) / self.size.x
        if self.range['INCREMENT']:
            num_incr = math.floor((self.range['MAX'] - self.range['MIN'])/self.range['INCREMENT'])
            k = (self.range['VALUE'] - self.range['MIN'])/(self.range['MAX']-self.range['MIN']) * num_incr
            k = int(round(k))
            self.range['VALUE'] = self.range['INCREMENT'] * k + self.range['MIN']

            self.update_cursor_position()

        if last_val != self.range['VALUE'] and self.callbacks['VALUE_CHANGE']:
            self.callbacks['VALUE_CHANGE'](self.range['VALUE'])

        return self


    def set_cursor_height(self, ratio):
        """
        arg is the ratio of height
        """
        self.cursor_radius = int(self.size.y * ratio / 2)
        self.cursor_height_ratio = ratio

        self.cursor.set_size(Vector2(self.cursor_radius * 2, self.cursor_radius * 2))
        self.cursor.set_radius(self.cursor_radius)
        return self

    def set_size(self, size:Vector2):
        super().set_size(size)
        self.slider.set_size(size)
        self.slider.set_radius(math.floor(self.slider.size.y / 2))
        if self.loaded:
            self.load()
        return self
    def set_position(self, position:Vector2):
        super().set_position(position)
        self.slider.set_position(position)
        return self

    def load(self):
        print(self.slider.size)
        self.slider.load()
        self.cursor.load()
        self.loaded = True
        return self

    def update(self, events):
        self.cursor.update(events)
        if self.cursor.pressed_rn:
            self.holding_cursor = True
        elif events.mouse.left.up:
            self.holding_cursor = False

        if self.holding_cursor:
            self.cursor.status = 'pressed'
            # Update pos according to mouse pos
            
            mousepos = events.mouse.pos() - self.parent_offset_px(self.parent)
            xpos = max(self.position.x, min(mousepos.x, self.position.x + self.size.x))
            self.cursor.position.x = xpos - self.cursor_radius
            self.calculate_value()

        return self

    def draw(self):
        self.slider.draw()
        self.cursor.draw()
        return self



class VerticalSlider(Slider):
    def set_cursor_height(self, ratio):
        """
        arg is the ratio of height
        """
        self.cursor_radius = int(self.size.x * ratio / 2)
        self.cursor_height_ratio = ratio

        self.cursor.set_size(Vector2(self.cursor_radius * 2, self.cursor_radius * 2))
        self.cursor.set_radius(self.cursor_radius)
        return self
        
    def update_cursor_position(self):
        try:
            self.cursor.set_position_by_center(Vector2(
                self.position.x + self.size.x / 2,
                (self.range['VALUE'] - self.range['MIN']) / (self.range['MAX'] - self.range['MIN']) * self.size.y + self.position.y
            ))
            return self
        except:
            raise SystemError('You must set the size and position first')
    
    def calculate_value(self):
        last_val = float(self.range['VALUE'])

        self.range['VALUE'] = self.range['MIN'] + (self.range['MAX'] - self.range['MIN']) * (self.cursor.position.y + self.cursor_radius - self.position.y) / self.size.y
        if self.range['INCREMENT']:
            num_incr = math.floor((self.range['MAX'] - self.range['MIN'])/self.range['INCREMENT'])
            k = (self.range['VALUE'] - self.range['MIN'])/(self.range['MAX']-self.range['MIN']) * num_incr
            k = int(round(k))
            self.range['VALUE'] = self.range['INCREMENT'] * k + self.range['MIN']

            self.update_cursor_position()

        if last_val != self.range['VALUE'] and self.callbacks['VALUE_CHANGE']:
            self.callbacks['VALUE_CHANGE'](self.range['VALUE'])

        return self

    def update_sizes(self):
        self.slider.set_size(self.size)
        self.slider.set_radius(math.floor(self.slider.size.x / 2))
        if self.loaded:
            self.load()

    def set_size(self, size:Vector2):
        super().set_size(size)
        self.update_sizes()
        return self

    def change_width(self, px:int):
        super().change_width(px)
        self.update_sizes()
        return self
    def change_height(self, px:int):
        super().change_height(px)
        self.update_sizes()
        return self

    def update(self, events):
        self.cursor.update(events)
        if self.cursor.pressed_rn:
            self.holding_cursor = True
        elif events.mouse.left.up:
            self.holding_cursor = False

        if self.holding_cursor:
            self.cursor.status = 'pressed'
            # Update pos according to mouse pos
            
            mousepos = events.mouse.pos() - self.parent_offset_px(self.parent)
            ypos = max(self.position.y, min(mousepos.y, self.position.y + self.size.y))
            self.cursor.position.y = ypos - self.cursor_radius
            self.calculate_value()

        return self