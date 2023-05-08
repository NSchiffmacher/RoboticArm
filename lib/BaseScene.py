import pygame
from pygame.locals import *
import imgui
import sys

import time
import OpenGL.GL as gl

from lib.Options import Options
from lib.Events import Events

class BaseScene:
    def __init__(self, app, options):
        self.app            = app
        self.window         = app.window
        self.io             = app.io
        self.render_engine  = app.render_engine
        self.options        = options
        self.loaded         = False
        self.continuer      = True

        self.events         = Events(self,  options.keys)
        self.return_value   = None

    def _load(self):
        self.load()

    def run(self):
        if not self.loaded:
            self.load()

        self.continuer      = True
        frame_length        = 1/self.options.fps_target
        physicsframe_length = frame_length / self.options.physics_frame

        last_physics_dt     = 0
        last_frame_dt       = 0

        while self.continuer:
            try:
                frame_dt = 0
                for i in range(self.options.physics_frame):
                    start_physics_frame = time.time()
                    self._physics_update(last_physics_dt)
                    # Update last variables
                    
                    if i == self.options.physics_frame - 1:
                        self._update(last_frame_dt)
                        self._draw()

                        if pygame.event.peek(QUIT):
                            self.continuer = False
                        pygame.event.pump()

                    
                    now = time.time()
                    if now - start_physics_frame <= physicsframe_length:
                        time.sleep(physicsframe_length + start_physics_frame - now)
                    
                    re_now = time.time()
                    last_physics_dt = re_now - start_physics_frame
                    frame_dt += last_physics_dt
                
                last_frame_dt = frame_dt
            except Exception as e:
                self.continuer = False
                raise e

        return self.return_value

    def swap_scene(self, scene):
        """
        Used to quit the current scene and go to "scene"
        """
        self.continuer = False
        self.return_value = scene
            
            

    def _update(self, dt):
        # Handle events
        self.events.check()
        # if self.events.quitter:
        #     self.continuer = False

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit(0)
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    sys.exit(0)

            self.render_engine.process_event(event)
        self.update(dt, self.events)

    def _physics_update(self, dt):
        self.physics_update(dt)

    def _draw(self):
        imgui.new_frame()
        gl.glClearColor(*[x / 255 for x in self.options.background_color], 1.) # Background color of the main screen
        gl.glClear(gl.GL_COLOR_BUFFER_BIT)
        
        self.draw(self.window)

        imgui.render()
        self.render_engine.render(imgui.get_draw_data())

        pygame.display.flip()

