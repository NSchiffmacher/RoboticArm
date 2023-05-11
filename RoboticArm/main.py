from pygame.locals import *
import pygame
import imgui
from pathlib import Path
import easygui
import os

from lib.BaseScene import BaseScene
from lib.Math.Vector import Vector2 as V
import math

# from RoboticArm.colors import *

from RoboticArm.RoboticArm import RoboticArm
from RoboticArm.ObstaclesManager import ObstaclesManager


class Scene(BaseScene):
    def __init__(self, app, options):
        super().__init__(app, options)

        self.draw_list = None
        self.add_obstacle = None
        self.main_focus = None
        self.obstacles = None
        self.arm = None
        self.theta2 = None
        self.theta1 = None
        self.window_size = V(self.app.options.window.width, self.app.options.window.height)

    def global_frame_to_draw_frame(self, vector: V):
        """
        Takes a vector in the global frame as input, and returns the vector in the drawing frame
        """
        # res.x = vector.x * scale_factor + window_width / 2
        # res.y = -vector.y * scale_factor + window_height / 2

        u = self.options.unit_vector_size_in_px
        return V(vector) * V(u, -u) + self.window_size / 2

    def global_scale_to_draw_scale(self, value):
        u = self.options.unit_vector_size_in_px
        return value * u

    def draw_frame_to_global_frame(self, vector: V):
        """
        Takes a vector in the global frame as input, and returns the vector in the drawing frame
        """
        # res.x = vector.x * scale_factor + window_width / 2
        # res.y = -vector.y * scale_factor + window_height / 2
        u = self.options.unit_vector_size_in_px
        return (vector - self.window_size / 2) / V(u, -u)

    @staticmethod
    def get_rgba(color):
        c = [x / 255. for x in color] + [1] if len(color) == 3 else []
        return imgui.get_color_u32_rgba(*c)

    def draw_filled_quad(self, points, color):
        coords = []
        for p in points:
            coords += [*self.global_frame_to_draw_frame(p).to_pygame()]
        self.draw_list.add_quad_filled(*coords, self.get_rgba(color))

    def draw_polygon(self, points, color, width=0):
        n = len(points)
        if n < 2:
            return

        for i in range(len(points) - 1):
            self.draw_line(points[i], points[i + 1], color, width)
        self.draw_line(points[0], points[n - 1], color, width)

    def draw_filled_triangle(self, triangle, color):
        coords = []
        for p in triangle:
            coords += [*self.global_frame_to_draw_frame(p).to_pygame()]

        self.draw_list.add_triangle_filled(
            *coords, self.get_rgba(color)
        )

    def draw_point(self, point: V, color: tuple, radius=5):
        """
        Draws a point on the window

        Arguments:
            point: Vector2 -> The point's position in the global frame
            color: tuple -> color in rgb
            radius: int -> Radius in px
        """
        self.draw_list.add_circle_filled(
            *self.global_frame_to_draw_frame(point).to_pygame(),
            radius,
            self.get_rgba(color)
        )
        # pygame.draw.circle(self.window, color, self.global_frame_to_draw_frame(point).to_pygame(), radius)

    def draw_circle(self, center: V, radius: int, color: tuple, width: int = 5):
        self.draw_list.add_circle(
            *self.global_frame_to_draw_frame(center).to_pygame(),
            self.global_scale_to_draw_scale(radius),
            self.get_rgba(color),
            thickness=width
        )

    def draw_line(self, pointA: V, pointB: V, color: tuple, width: int):
        """
        Draws a line on the window

        Arguments:
            pointA: Vector2 -> One of the endpoints in the global frame
            pointB: Vector2 -> The second endpoint in the global frame
            color: tuple -> color in rgb
            width: int -> width of the line
        """
        self.draw_list.add_line(
            *self.global_frame_to_draw_frame(pointA).to_pygame(),
            *self.global_frame_to_draw_frame(pointB).to_pygame(),
            self.get_rgba(color), width
        )

    def load(self):
        # Robotic Arm
        piece_size = V(30, 4)
        pivot = V(2, 2)
        attach = V(28, 2)
        self.theta1 = math.radians(10)
        self.theta2 = math.radians(-20)
        self.arm = RoboticArm(V(0, 0), self.theta1, self.theta2, piece_size, piece_size, pivot, pivot, attach, attach)

        # Obstacles
        self.obstacles = ObstaclesManager()
        self.obstacles.load_from_json(Path(os.getcwd()) / self.options.obstacles_path / self.options.default_obstacles)

        self.main_focus = False

        # imgui specific
        self.add_obstacle = False

    def update(self, dt, events):
        # Robotic arm events
        if events.on_first_check_intersection and not self.io.key_ctrl:
            angle = math.pi
            n = 100

            print('Compute CSPACE')
            cspace = self.arm.computeCSPace([-angle, angle], [-angle, angle], n, self.obstacles)
            print(cspace)

        keys = pygame.key.get_pressed()
        if keys[K_LEFT] or keys[K_RIGHT] or keys[K_UP] or keys[K_DOWN]:
            speed = 40
            if keys[K_LEFT]:
                self.theta1 += dt * math.radians(speed)
            if keys[K_RIGHT]:
                self.theta1 -= dt * math.radians(speed)
            if keys[K_UP]:
                self.theta2 += dt * math.radians(speed)
            if keys[K_DOWN]:
                self.theta2 -= dt * math.radians(speed)
            self.arm.update_angles(self.theta1, self.theta2)

        # Obstacle events
        if events.on_first_new_obstacle and self.io.key_ctrl:
            self.add_obstacle = True

        if self.add_obstacle and events.on_first_close_obstacle and self.io.key_ctrl:
            self.add_obstacle = False
            self.obstacles.reset_current()

        # left click on main gui
        if self.main_focus and imgui.is_mouse_clicked():
            if self.add_obstacle:
                mouse_pos = self.draw_frame_to_global_frame(V(*imgui.get_mouse_pos()))
                self.obstacles.add_point_to_current_obstacle(mouse_pos)

        # right click on main gui
        if self.main_focus and imgui.is_mouse_clicked(imgui.MOUSE_BUTTON_MIDDLE):
            if self.add_obstacle:
                mouse_pos = self.draw_frame_to_global_frame(V(*imgui.get_mouse_pos()))
                self.obstacles.remove_point_from_current(mouse_pos, 2)

        if self.main_focus and events.on_first_confirm_new_obstacle and self.io.key_ctrl:
            self.obstacles.confirm_current()

    def physics_update(self, dt):
        pass

    def draw(self, window):
        imgui.set_next_window_position(0, 0)
        imgui.set_next_window_size(*self.window_size.to_pygame())
        with imgui.begin("main", False,
                         imgui.WINDOW_NO_RESIZE | imgui.WINDOW_NO_COLLAPSE |
                         imgui.WINDOW_MENU_BAR | imgui.WINDOW_NO_MOVE | imgui.WINDOW_NO_BRING_TO_FRONT_ON_FOCUS):
            # menu bar
            self.main_focus = imgui.is_window_focused()
            with imgui.begin_menu_bar() as obstacles_menu_bar:
                if obstacles_menu_bar.opened:

                    # obstacles menu
                    with imgui.begin_menu('Obstacles') as file_menu:
                        if file_menu.opened:
                            clicked, state = imgui.menu_item('New', "Ctrl+N")
                            if clicked:
                                self.add_obstacle = True
                            clicked, state = imgui.menu_item('Load')
                            if clicked:
                                def_path = Path(
                                    os.getcwd()) / self.options.obstacles_path / self.options.default_obstacles
                                path = easygui.fileopenbox(default=def_path)
                                if path and os.path.exists(path):
                                    self.obstacles.load_from_json(path)
                            clicked, state = imgui.menu_item('Save')
                            if clicked:
                                new_path = Path(os.getcwd()) / self.options.obstacles_path / self.options.new_obstacles
                                path = easygui.filesavebox(default=new_path)
                                if path:
                                    self.obstacles.dump_as_json(path)

                            clicked, state = imgui.menu_item('Clear')
                            if clicked:
                                self.obstacles.clear()

            # add obstacle
            if self.add_obstacle:
                self.add_obstacle_imgui()

            # robot drawing
            self.draw_list = imgui.get_window_draw_list()
            self.arm.draw(self)

            # obstacles drawing
            self.obstacles.draw(self)

    def add_obstacle_imgui(self):
        with imgui.begin("Add obstacle", imgui.WINDOW_NO_FOCUS_ON_APPEARING | imgui.WINDOW_NO_RESIZE):
            imgui.set_window_size(0, 0)

            imgui.text("Left click on the window to add a point to an obstacle")
            imgui.text("Right click next to a point to remove it")
            imgui.new_line()
            add_new = imgui.button("Ctrl+A to add it", imgui.get_window_width() * 0.965)
            if add_new:
                self.obstacles.remove_latest()  # hack because it changes the focus when clicking on the button
                self.obstacles.confirm_current()

            imgui.new_line()
            close = imgui.button("Ctrl+Q to stop", imgui.get_window_width() * 0.965)
            if close:
                self.add_obstacle = False
                self.obstacles.reset_current()
