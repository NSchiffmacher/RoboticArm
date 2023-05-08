import pygame
from pygame.locals import *

from lib.BaseScene import BaseScene

import lib.HUD.custom_drawing as cd
from lib.HUD.RoundedRect import RoundedRect
from lib.HUD.Label import Label
from lib.HUD.Canvas import Canvas
from lib.HUD.Button import Button
from lib.HUD.Slider import Slider, VerticalSlider
from lib.HUD.Graph import Curve
from lib.HUD.PopUp import PopUp

from lib.Math.Vector import Vector2

import time, numpy as np, math

# Rounded_rect sur une surface de 100x100 avec 25 de radius : 0.0396 ms



class Scene(BaseScene):
    def load(self):
        self.font = pygame.font.Font('lib/HUD/fonts/Oswald.ttf', 32)

        margin = 0.02
        self.text1 = Label(self.window, self.font, 'Vraiment beaucoup de texte ' * 40).set_color((255,255,255))
        self.text1.set_size_from_ratio(Vector2(0.5 - 2 * margin, 1 - 2 * margin)).center_vertically_ratio(margin).load()

        self.text2 = Label(self.window, self.font, 'text de beaucoup Vraiment ' * 40).set_color((255,255,255))
        self.text2.set_size_from_ratio(Vector2(0.5 - 2 * margin, 1 - 2 * margin)).center_vertically_ratio(margin + 0.5).load()

        self.popup_button = Button(self.window, 20, {
            'normal': (125,15,15),
            'hover': (110,10,10),
            'pressed': (90,5,5),
            'label': (255,255,255)
        }, 'PopUP', self.font).set_size_from_ratio(Vector2(0.3,0.1)).center()
        self.popup_button.load()

        # POPUP
        self.popup = PopUp(self.window)
        Button(self.popup, 20, {
            'normal': (125,15,15),
            'hover': (110,10,10),
            'pressed': (90,5,5),
            'label': (255,255,255)
        }, 'Fermer', self.font).set_size_from_ratio(Vector2(0.3,0.1)).set_position_ratio(Vector2(0.05,0.05)).add_to_canvas(self.popup)


        self.popup.load_all()
        
    
    def update(self, dt, events):
        self.popup_button.update(events)


    def physics_update(self, dt):
        pass


    def draw(self, fenetre):
        # self.window.blit(self.s, (0,0))
        # self.text1.draw()
        # self.text2.draw()

        # self.popup_button.draw()

        self.popup.draw()