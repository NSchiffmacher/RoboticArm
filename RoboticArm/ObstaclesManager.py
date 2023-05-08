import numpy as np
from lib.Math.Vector import Vector2 as V

from RoboticArm.colors import *
from RoboticArm.utils.mesh_generation import *
from RoboticArm.Obstacle import Obstacle

import json

class ObstaclesManager:
    def __init__(self, draw_mesh=False, current_draw_mesh=True):
        self.obstacles: list(Obstacle) = []
        self.draw_mesh = draw_mesh
        self.current_draw_mesh = current_draw_mesh
        self.last_pt_added = V(99999999,99999999)

        self.current_new_obstacle = Obstacle([], self.current_draw_mesh, False)

    def get_all_triangles(self):
        out = []
        for o in self.obstacles:
            out += o.triangles
        return out

    def add_obstacle(self, o: Obstacle):
        self.obstacles.append(o)

    def add_point_to_current_obstacle(self, point):
        self.current_new_obstacle.add_point(point)
        self.last_pt_added = point

    def remove_latest(self):
        return self.remove_point_from_current(self.last_pt_added, 0.1)

    def remove_point_from_current(self, point, delete_radius):
        return self.current_new_obstacle.remove_point_if_close(point, delete_radius)
    
    def confirm_current(self):
        self.current_new_obstacle.draw_mesh = self.draw_mesh
        self.obstacles.append(self.current_new_obstacle)
        self.reset_current()

    def reset_current(self):
        self.current_new_obstacle = Obstacle([], self.current_draw_mesh, False)

    def draw(self, scene):
        for obstacle in self.obstacles:
            obstacle.draw(scene)
        self.current_new_obstacle.draw(scene)

    def dump_as_json(self, filepath):
        data = {"obstacles": []}

        for obstacle in self.obstacles:
            data["obstacles"].append(obstacle.save_to_dict())

        with open(filepath, 'w') as file:
            file.write(json.dumps(data, indent=2))

    def load_from_json(self, filepath):
        with open(filepath, 'r') as file:
            data = json.loads(file.read())

        for obstacle_data in data["obstacles"]:
            obstacle = Obstacle.load_from_dict(obstacle_data)
            self.add_obstacle(obstacle)

    def clear(self):
        self.obstacles = []
        self.reset_current()
