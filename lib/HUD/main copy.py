import pygame
from pygame.locals import *

from lib.BaseScene import BaseScene

import lib.HUD.custom_drawing as cd
from lib.HUD.RoundedRect import RoundedRect
from lib.HUD.Label import Label
from lib.HUD.Canvas import Canvas
from lib.HUD.Button import Button
from lib.HUD.Slider import Slider, VerticalSlider

from lib.Math.Vector import Vector2

import time

# Rounded_rect sur une surface de 100x100 avec 25 de radius : 0.0396 ms



class Scene(BaseScene):
    def load(self):
        self.font = pygame.font.Font('lib/HUD/fonts/Oswald.ttf', 32)

        button_height = 0.2
        text_margin   = Vector2(0.05, 0.01)
        self.canvas = Canvas(self.window).set_size_from_ratio(Vector2(0.4,0.7)).center()
        self.canvas.append_child(RoundedRect(self.canvas, color=(68, 140, 201), radius=50, corners_code=RoundedRect.TOP_RIGHT | RoundedRect.BOTTOM).fit().center())
        self.text = Label(self.canvas, self.font, 'Lorem ipsum dolor sit amet, consectetur adipiscing elit. Sed volutpat enim quis volutpat malesuada. Integer pharetra libero justo, pharetra maximus risus pellentesque et. Mauris maximus feugiat nulla et placerat. Suspendisse interdum erat mauris, sed maximus dui facilisis vitae. Proin pharetra lacinia porta. Etiam molestie diam eu egestas fringilla. Donec tincidunt velit lacus, a commodo nibh viverra eu. Praesent varius, mi et pharetra auctor, nisi diam tempus nulla, sit amet placerat ante leo sed sem. Cras imperdiet velit eget dui dictum aliquam. Aliquam aliquet elementum faucibus. Ut tincidunt rutrum lacinia. Integer scelerisque iaculis egestas. Praesent malesuada pharetra tortor, malesuada eleifend turpis hendrerit vel. ', color=(255,255,255)).set_size_from_ratio(Vector2(1-2*text_margin.x, 1-2*text_margin.y-button_height)).center_horizontally_ratio(text_margin.y)
        self.text.add_to_canvas(self.canvas)
        self.button = Button(self.canvas, button_text='Un beau bouton', button_font=self.font).set_size_from_ratio(Vector2(1,button_height)).center_horizontally_ratio(1-button_height)
        
        self.text_flip = True
        self.button.set_colors({
            'normal' : (130,20,20),
            'hover'  : (105,15,15),
            'pressed': (70,10,10),
            'label'  : (255,255,0) 
        }).set_callbacks({
            'RELEASED': self.button_callback
        }).set_radius(50).set_corners(RoundedRect.BOTTOM).add_to_canvas(self.canvas)    
        self.canvas.load_all()



        # SLIDER
        # self.slider = Slider(self.window, colors={
        #     'SLIDER': (130,20,20),
        #     'CURSOR_NORMAL': (255,255,255),
        #     'CURSOR_HOVER': (200,200,200),
        #     'CURSOR_PRESSED': (150,150,150)
        # }).set_size_from_ratio(Vector2(0.6,0.02)).center_horizontally_ratio(0.90).set_cursor_height(1.5).set_range({
        #     'MIN': 12,
        #     'MAX': 64,
        #     'VALUE': 32,
        #     'INCREMENT': 1
        # }).set_callbacks({
        #     'VALUE_CHANGE': self.slider_callback
        # }).load()
        self.slider = VerticalSlider(self.window, colors={
            'SLIDER': (130,20,20),
            'CURSOR_NORMAL': (255,255,255),
            'CURSOR_HOVER': (200,200,200),
            'CURSOR_PRESSED': (150,150,150)
        })
        self.slider.change_width_ratio(0.02).change_height(self.canvas.size.y - self.button.size.y).set_position(Vector2(self.canvas.position.x - self.slider.size.x,self.canvas.position.y)).set_cursor_height(1.5).set_range({
            'MIN': 0,
            'MAX': self.text.get_display_height() - self.text.size.y,
            'VALUE': 0,
            'INCREMENT': 1,
        }).set_callbacks({
            'VALUE_CHANGE': self.slider_callback
        })
        self.slider.slider.set_corners(RoundedRect.LEFT)
        self.slider.load()



        
    
    def update(self, dt, events):
        self.button.update(events)
        self.slider.update(events)

    def button_callback(self):
        if self.text_flip:
            self.text.change_text('Donec non magna arcu. Morbi dictum a quam ut laoreet. Curabitur efficitur, purus eget porttitor posuere, purus ex rutrum velit, et sodales mi diam eu nulla. Mauris iaculis ex nec congue posuere. Morbi fermentum hendrerit est, non dictum urna gravida sit amet. Ut sed iaculis nunc. Aliquam vestibulum, metus sed finibus accumsan, tellus mauris auctor lacus, a luctus metus neque in lacus. Ut pulvinar pellentesque diam. Morbi sed volutpat ligula. Nullam scelerisque malesuada euismod. Sed et nisl at risus consequat elementum vel vel turpis. Fusce porta, nisl sit amet hendrerit mattis, ex nisi convallis velit, quis accumsan odio diam ut libero. Quisque condimentum lorem in aliquet gravida. Donec ornare neque quis bibendum vestibulum. Aliquam dui risus, iaculis a eros rhoncus, commodo efficitur ante. Sed porta, ligula vel egestas lobortis, ligula ante tincidunt magna, ac mollis eros nunc a erat. Donec non magna arcu. Morbi dictum a quam ut laoreet. Curabitur efficitur, purus eget porttitor posuere, purus ex rutrum velit, et sodales mi diam eu nulla. Mauris iaculis ex nec congue posuere. Morbi fermentum hendrerit est, non dictum urna gravida sit amet. Ut sed iaculis nunc. Aliquam vestibulum, metus sed finibus accumsan, tellus mauris auctor lacus, a luctus metus neque in lacus. Ut pulvinar pellentesque diam. Morbi sed volutpat ligula. Nullam scelerisque malesuada euismod. Sed et nisl at risus consequat elementum vel vel turpis. Fusce porta, nisl sit amet hendrerit mattis, ex nisi convallis velit, quis accumsan odio diam ut libero. Quisque condimentum lorem in aliquet gravida. Donec ornare neque quis bibendum vestibulum. Aliquam dui risus, iaculis a eros rhoncus, commodo efficitur ante. Sed porta, ligula vel egestas lobortis, ligula ante tincidunt magna, ac mollis eros nunc a erat. ')
        else:
            self.text.change_text('Lorem ipsum dolor sit amet, consectetur adipiscing elit. Sed volutpat enim quis volutpat malesuada. Integer pharetra libero justo, pharetra maximus risus pellentesque et. Mauris maximus feugiat nulla et placerat. Suspendisse interdum erat mauris, sed maximus dui facilisis vitae. Proin pharetra lacinia porta. Etiam molestie diam eu egestas fringilla. Donec tincidunt velit lacus, a commodo nibh viverra eu. Praesent varius, mi et pharetra auctor, nisi diam tempus nulla, sit amet placerat ante leo sed sem. Cras imperdiet velit eget dui dictum aliquam. Aliquam aliquet elementum faucibus. Ut tincidunt rutrum lacinia. Integer scelerisque iaculis egestas. Praesent malesuada pharetra tortor, malesuada eleifend turpis hendrerit vel. ')
        self.slider.set_range({
            'VALUE' : 0,
            'MAX': self.text.get_display_height() - self.text.size.y
        })
        self.slider.update_cursor_position()
        self.text.set_yscroll_offset(0)
        self.text_flip = not self.text_flip
    
    def slider_callback(self, value):
        # self.text.load_font('lib/HUD/fonts/Oswald.ttf', value)
        # self.text.load()
        self.text.set_yscroll_offset(value)
        print('G')

    def physics_update(self, dt):
        pass


    def draw(self, fenetre):
        self.canvas.draw()
        self.slider.draw()