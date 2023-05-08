import pygame
from pygame.locals import *

import time

from lib.Math.Vector import Vector2
from lib.Usefull.VarSaver import VarSaver

class Mouse:
    def __init__(self, double_click_delay = 0.2):
        self.left = MouseButton(double_click_delay)
        self.middle = MouseButton(double_click_delay)
        self.right = MouseButton(double_click_delay)

    def update(self):
        now = time.time()
        buttons = [bool(button) for button in pygame.mouse.get_pressed()]
        
        self.left.update(buttons[0], now)
        self.middle.update(buttons[1], now)
        self.right.update(buttons[2], now)

    def rel_pos(self):
        return Vector2(*pygame.mouse.get_rel())

    def pos(self):
        return Vector2(*pygame.mouse.get_pos())

class MouseButton:
    def __init__(self, double_click_delay):
        self.down = False # La souris est pressée (en ce moment même)
        self.down_rn = False # La souris vient d'être pressée
        self.up = False #La souris vient d'être relevée
        self.double_click = False


        self.last = VarSaver(self)
        self.last.whitelist_add('down', 'down_rn', 'up', 'double_click', 'last_click')
        self.last.list_variables()

        # self.last_down = False # La souris est pressée (en ce moment même)
        # self.last_down_rn = False # La souris vient d'être pressée
        # self.last_up = False #La souris vient d'être relevée
        # self.last_double_click = False

        self.last_click = 0 # timestamps of last click (for double clicks)

        self.double_click_delay = double_click_delay # Temps pour que ce soit compté comme un double click
    
    def update(self, button, timestamp):
        # Reset des variables
        button = bool(button)
        self.down = button
        self.down_rn = False
        self.up = False 
        self.double_click = False


        # Check unique click
        if self.last.down == False and button:
            if timestamp - self.last_click <= self.double_click_delay:
                self.double_click = True
                
            else:
                self.down_rn = True
            self.last_click = timestamp
        elif self.last.down and not button:
            self.up = True


        self.last.update()