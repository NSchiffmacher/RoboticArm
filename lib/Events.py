import pygame
from pygame.locals import *

import time

from lib.Mouse import MouseButton, Mouse
from lib.Options import Container
from lib.Usefull.VarSaver import VarSaver


class Events:
    def __init__(self, scene, groups: Container):
        """
        All events created in the options folder are attributes of this class,
        and for instance events.quit check if the "quit" event has been triggered

        Triggers are:
            - Pygame keys, so for instance K_a for the a key
            - Mouse events written like so
                [MOUSE_ID]MOUSE_[MOUSE_EVENT]

                with MOUSE_ID in ["L", "M", "R"], the mouse button
                and MOUSE_EVENT in ["CLICK", "DCLICK", "UP"]
        """

        self.scene = scene
        self.groups = groups

        self.key_groups = {}
        self.first_prefix = 'on_first_'
        self.first_key_groups = {} # Contains true when event called for the first time
        self.load_attrs()

        # Init mouse object
        self.mouse = Mouse()

        
        self.last = VarSaver(self)
        self.last.whitelist_add('key_groups')
        self.last.list_variables()
        self.keys = []


        self.vars = globals()


    def load_attrs(self):
        self.key_groups = {}
        for group in self.groups.names():
            self.key_groups[group] = False
            self.first_key_groups[group] = False
        

    def check(self):
        """
        Used to check for events
        """
        self.last.update()
        self.mouse.update()
        now = time.time()

        pressed = pygame.key.get_pressed()
        for group_name, group_params in self.groups.items():
            # Triggers
            self.first_key_groups[group_name] = False
            for trigger in group_params.trigger:

                # KEY EVENTS
                if trigger.startswith('K_'):
                    if pressed[self.vars[trigger]]:
                        self.key_groups[group_name] = True
                        break

                # MOUSE CLICK, DCLICK EVENTS
                elif trigger[1:6].upper() == 'MOUSE':
                    trigger = trigger.upper()
                    mouse_id = trigger[0]
                    mouse_event = trigger[7:]
                    
                    # Get mouse object
                    if   mouse_id == 'R': mouse = self.mouse.right 
                    elif mouse_id == 'L': mouse = self.mouse.left 
                    elif mouse_id == 'M': mouse = self.mouse.middle
                    else:
                        raise AttributeError(f'Invalid Mouse ID "{mouse_id}" in "{group_name}"\'s trigger : {trigger}')
            
                    # Check event according to the event
                    if mouse_event == 'CLICK':
                        if mouse.down and not mouse.double_click:
                            self.key_groups[group_name] = True
                            break
                    elif mouse_event == 'DCLICK':
                        if mouse.double_click: 
                            self.key_groups[group_name] = True
                            break
                    elif mouse_event == 'UP':
                        if mouse.up: 
                            self.key_groups[group_name] = True
                            break
                    else:
                        raise AttributeError(f'Invalid Mouse Event "{mouse_event} in "{trigger}"\'s trigger : {trigger} ')
            else:
                self.key_groups[group_name] = False

            if self.key_groups[group_name] and not self.last.key_groups[group_name]:
                self.first_key_groups[group_name] = True
        
            
            # Callback
            if self.key_groups[group_name] and "callback" in group_params and group_params.callback != None:
                getattr(self.scene, group_params.callback)()
            # First Callback
            if self.first_key_groups[group_name] and "first_callback" in group_params and group_params.first_callback != None:
                getattr(self.scene, group_params.first_callback)()

        
        # Mouse handling
        self.keys = pressed
    
    def __getattr__(self, name):
        if name in self.key_groups:
            return self.key_groups[name]
        elif name.startswith(self.first_prefix) and name[len(self.first_prefix):] in self.first_key_groups:
            return self.first_key_groups[name[len(self.first_prefix):]]
        else:
            raise AttributeError

    def __getitem__(self, item):
        return self.__getattr__(item)
