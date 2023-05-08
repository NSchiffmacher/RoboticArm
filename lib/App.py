import pygame
import imgui
from imgui.integrations.pygame import PygameRenderer
from pygame.locals import *

import os
import json

from lib.Options import Options

        


class App():
    def __init__(self, options_json):
        self.options = Options(options_json)
        


    def load(self):
        """
        Loads the app, so mostly pygame and the window
        """

        pygame.init()
        imgui.create_context()

        self.window = pygame.display.set_mode((self.options.window.width, self.options.window.height), DOUBLEBUF | OPENGL | NOFRAME)
        pygame.display.set_caption(self.options.window.title)
        
        self.io = imgui.get_io()
        self.io.fonts.add_font_default()
        self.io.display_size = (self.options.window.width, self.options.window.height)
        self.render_engine = PygameRenderer()

    def create_scene(self, scene_name : str):
        """
        Creates a Scene Object from a scene_name
        Scene names can be for example lib/HUD to access a scene inside a folder
        """

        scene_folder = os.getcwd() + '/' + scene_name
        main_file = scene_folder + '/main.py'
        options_json_file = scene_folder + '/options.json'

        with open(options_json_file) as file:
            options_json = json.loads(file.read())
        
        # scene_name.replace('/', '.') => On peut utiliser lib/HUD pour charger l'app HUD
        module = __import__(scene_name.replace('/', '.') + '.main', fromlist=[''])
        options = Options(options_json)
        
        return module.Scene(self, options)  

    def run(self):
        """
        Runs the first scene, and redirects to the next if required
        """
        next_scene = self.options.entry_scene
        while next_scene != None:
            entry_scene = self.create_scene(next_scene)
            next_scene = entry_scene.run()
