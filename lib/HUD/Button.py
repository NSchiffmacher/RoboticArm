import pygame
from pygame.locals import *

from lib.Math.Vector import Vector2

from lib.HUD.Canvas import Canvas
from lib.HUD.RoundedRect import RoundedRect
from lib.HUD.Label import Label

from lib.Options import Container


class Button(Canvas):
    TRANSPARENT     = Canvas.TRANSPARENT
    LEFT_BUTTON     = 0
    MIDDLE_BUTTON   = 1
    RIGHT_BUTTON    = 2
    
    ALLOWED_KEYS = ['PRESSED', 'PRESSED_RN', 'DCLICK', 'RELEASED']

    def __init__(self, parent: pygame.Surface, rect_radius: int or None = None, colors: dict or None = None, button_text: str or None = None, button_font = None, callbacks: dict or None = None, position: Vector2 or None = None, size: Vector2 or None = None):
        """
        colors: dict with 4 keys,
            normal,
            hover,
            pressed,
            label
        """
        
        super().__init__(parent, self.TRANSPARENT, position, size)

        self.rects = {
            'normal' : RoundedRect(self, ).add_to_canvas(self),
            'hover'  : RoundedRect(self, ).add_to_canvas(self),
            'pressed': RoundedRect(self, ).add_to_canvas(self)
        }
        self.label = Label(self, ).add_to_canvas(self, 2)

        self.normal_rect    = self.rects['normal']
        self.hover_rect     = self.rects['hover']
        self.pressed_rect   = self.rects['pressed']
        self.status         = 'normal'

        # Button pressed variables
        self.pressed        = False
        self.pressed_rn     = False # Right now
        self.just_released  = False
        self.double_click   = False


        self.mouse_button   = self.LEFT_BUTTON

        if colors:
            self.set_colors(colors)
        if rect_radius:
            self.set_radius(rect_radius)
        else:
            self.radius = None

        # Label setup
        if button_text:
            self.label.text = button_text
        if button_font:
            self.label.set_font(button_font)
        self.label.set_text_align(Label.TEXT_CENTERED)

        # Callbacks setup
        self.init_callbacks()
        if callbacks:
            self.set_callbacks(callbacks)

    def init_callbacks(self):
        self.callbacks = {}
        for key in self.ALLOWED_KEYS:
            self.callbacks[key] = None
        return self

    def set_callbacks(self, callbacks):
        """
        4 keys: 
            PRESSED,
            PRESSED_RN,
            DCLICK
            RELEASED,

        """
        for key in callbacks:
            if key not in self.ALLOWED_KEYS:
                raise AttributeError(f'Invalid "{key}" key, key must be in {self.ALLOWED_KEYS}')
            self.callbacks[key] = callbacks[key]
        return self
            


    def load_font(self, font_file, font_size):
        self.label.load_font(font_file, font_size)
        return self

    def change_text(self, text):
        self.label.change_text(text)
        return self

    def set_colors(self, colors):
        """
        colors: dict with 4 keys,
            normal,
            hover,
            pressed,
            label
        """
        for color_key in colors:
            if color_key != 'label':
                self.rects[color_key].set_color(colors[color_key])
            else:
                self.label.set_color(colors['label'])
        return self
        
    def set_radius(self, radius):
        self.radius = radius
        for rect in self.rects.values():
            rect.set_radius(radius)
        return self
    
    def set_corners(self, corner_code):
        for rect in self.rects.values():
            rect.set_corners(corner_code)
        return self

    


    def load(self):
        self.reshape_items()
        super().load_childs()
        super().load()

        self.label.change_width_ratio(1).fit_height().center()


    def reshape_items(self):
        for key_rect in self.rects:
            self.rects[key_rect].fit().set_position(Vector2(0,0))

        try:
            self.label.change_width_ratio(1).fit_height().center()
        except:
            pass
        return self

    def update(self, events):
        self.pressed        = False
        self.pressed_rn     = False # Right now
        self.just_released  = False
        self.double_click   = False


        if self.mouse_button == self.LEFT_BUTTON:
            mouse_button = events.mouse.left
        elif self.mouse_button == self.MIDDLE_BUTTON:
            mouse_button = events.mouse.middle
        else:
            mouse_button = events.mouse.right
            
        pos = events.mouse.pos()
        if type(self.parent) != Container:
            pos -= self.parent_offset_px(self.parent)

        if self.position.x <= pos.x <= (self.position.x + self.size.x) and self.position.y <= pos.y <= (self.position.y + self.size.y):
            # Hover
            self.status = 'hover'
            if mouse_button.down:
                self.status = 'pressed'
                self.pressed = True
                if mouse_button.down_rn:
                    self.pressed_rn = True
                elif mouse_button.double_click:
                    self.double_click = True
            elif mouse_button.up:
                self.just_released = True

            # CALLBACKS
            if self.pressed and self.callbacks['PRESSED']:
                self.callbacks['PRESSED']()
            if self.pressed_rn and self.callbacks['PRESSED_RN']:
                self.callbacks['PRESSED_RN']()
            if self.double_click and self.callbacks['DCLICK']:
                self.callbacks['DCLICK']()
            if self.just_released and self.callbacks['RELEASED']:
                self.callbacks['RELEASED']()


        else:
            self.status = 'normal'

    def draw(self):
        self.fill()
        self.rects[self.status].draw()
        self.label.draw()
        self.blit_surface()
        return self

class NoTextButton(Canvas):
    TRANSPARENT     = Canvas.TRANSPARENT
    LEFT_BUTTON     = 0
    MIDDLE_BUTTON   = 1
    RIGHT_BUTTON    = 2
    
    ALLOWED_KEYS = ['PRESSED', 'PRESSED_RN', 'DCLICK', 'RELEASED']

    def __init__(self, parent: pygame.Surface, rect_radius: int or None = None, colors: dict or None = None, callbacks: dict or None = None, position: Vector2 or None = None, size: Vector2 or None = None):
        """
        colors: dict with 3 keys,
            normal,
            hover,
            pressed,
        """
        
        super().__init__(parent, self.TRANSPARENT, position, size)

        self.rects = {
            'normal' : RoundedRect(self, ).add_to_canvas(self),
            'hover'  : RoundedRect(self, ).add_to_canvas(self),
            'pressed': RoundedRect(self, ).add_to_canvas(self)
        }

        self.normal_rect    = self.rects['normal']
        self.hover_rect     = self.rects['hover']
        self.pressed_rect   = self.rects['pressed']
        self.status         = 'normal'

        # Button pressed variables
        self.pressed        = False
        self.pressed_rn     = False # Right now
        self.just_released  = False
        self.double_click   = False


        self.mouse_button   = self.LEFT_BUTTON

        if colors:
            self.set_colors(colors)
        if rect_radius:
            self.set_radius(rect_radius)
        else:
            self.radius = None

        # Callbacks setup
        self.init_callbacks()
        if callbacks:
            self.set_callbacks(callbacks)

    def init_callbacks(self):
        self.callbacks = {}
        for key in self.ALLOWED_KEYS:
            self.callbacks[key] = None
        return self

    def set_callbacks(self, callbacks):
        """
        4 keys: 
            PRESSED,
            PRESSED_RN,
            DCLICK
            RELEASED,

        """
        for key in callbacks:
            if key not in self.ALLOWED_KEYS:
                raise AttributeError(f'Invalid "{key}" key, key must be in {self.ALLOWED_KEYS}')
            self.callbacks[key] = callbacks[key]
        return self
            

    def set_colors(self, colors):
        """
        colors: dict with 3 keys,
            normal,
            hover,
            pressed,
        """
        for color_key in colors:
            self.rects[color_key].set_color(colors[color_key])
        return self
        
    def set_radius(self, radius):
        self.radius = radius
        for rect in self.rects.values():
            rect.set_radius(radius)
        return self
    
    def set_corners(self, corner_code):
        for rect in self.rects.values():
            rect.set_corners(corner_code)
        return self

    


    def load(self):
        self.reshape_items()
        super().load_childs()
        return super().load()


    def reshape_items(self):
        for key_rect in self.rects:
            self.rects[key_rect].fit().set_position(Vector2(0,0))
        return self

    def update(self, events):
        self.pressed        = False
        self.pressed_rn     = False # Right now
        self.just_released  = False
        self.double_click   = False


        if self.mouse_button == self.LEFT_BUTTON:
            mouse_button = events.mouse.left
        elif self.mouse_button == self.MIDDLE_BUTTON:
            mouse_button = events.mouse.middle
        else:
            mouse_button = events.mouse.right
            
        pos = events.mouse.pos()
        if type(self.parent) != Container:
            pos -= self.parent_offset_px(self.parent)

        if self.position.x <= pos.x <= (self.position.x + self.size.x) and self.position.y <= pos.y <= (self.position.y + self.size.y):
            # Hover
            self.status = 'hover'
            if mouse_button.down:
                self.status = 'pressed'
                self.pressed = True
                if mouse_button.down_rn:
                    self.pressed_rn = True
                elif mouse_button.double_click:
                    self.double_click = True
            elif mouse_button.up:
                self.just_released = True

            # CALLBACKS
            if self.pressed and self.callbacks['PRESSED']:
                self.callbacks['PRESSED']()
            if self.pressed_rn and self.callbacks['PRESSED_RN']:
                self.callbacks['PRESSED_RN']()
            if self.double_click and self.callbacks['DCLICK']:
                self.callbacks['DCLICK']()
            if self.just_released and self.callbacks['RELEASED']:
                self.callbacks['RELEASED']()


        else:
            self.status = 'normal'

    def draw(self):
        self.fill()
        self.rects[self.status].draw()
        self.blit_surface()
        return self