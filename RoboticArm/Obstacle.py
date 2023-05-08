import numpy as np
from lib.Math.Vector import Vector2 as V

from RoboticArm.colors import *
from RoboticArm.utils.mesh_generation import *

class Obstacle:
    def __init__(self, points, draw_mesh=False, make=True):
        self.points = points
        self.triangles = []
        self.polygon = []

        self.draw_mesh = draw_mesh
        if make:
            self.make_obstacle()

        # draw colors
        self.draw_options = {
            "points": {
                "color": BLACK,
                "radius": 3
            },
            "triangles": {
                "color": (80, 80, 80),
                "radius": 2
            },
            "outline": {
                "color": (80, 80, 80),
                "width": 2
            },
            "polygon": {
                "color": (120,120,120),
            }
        }


    def add_point(self, point):
        self.points.append(point)
        self.make_obstacle()


    def make_obstacle(self):
        if len(self.points) < 3:
            self.supertriangle = []
            self.triangles = []
            return 
        # Makes the triangles using BowyerWatson algorithm

        # 1) find the super triangle using Weltz algo
        supertriangle = tuple([V(-99999999999.0,-99999999999.0), V(0.0,99999999999.0), V(99999999999.0,0.0)])
        triangulation = {supertriangle}

        # 2) Run the algo (https://en.wikipedia.org/wiki/Bowyer%E2%80%93Watson_algorithm)
        for point in self.points:
            # a) find bag triangles
            bad_triangles = set()
            for triangle in triangulation:
                if point_inside_circumcircle(point, triangle):
                    bad_triangles.add(triangle)

            # b) find the boundary of the polygonal hole
            polygon = set()
            for triangle in bad_triangles:
                for edge in triangle_edges(triangle):
                    if not edge_in_triangles(edge, bad_triangles - {triangle}):
                        polygon.add(edge)
                
            # c) remove bad triangles
            for triangle in bad_triangles:
                triangulation.remove(triangle)

            # d) re-triangulate the hole
            for edge in polygon:
                new_tri = (edge[0], point, edge[1])
                triangulation.add(new_tri)
                
        # 3) Exclude the triangle that are based on the supertriangle
        self.triangles = []
        for triangle in triangulation:
            for triangle_vertex in triangle:
                if triangle_vertex in supertriangle:
                    break
            else:
                self.triangles.append(triangle)
        
        # 4) Make polygon formed by triangles
        self.polygon = make_polygon_from_triangles(self.triangles)

        # print(f'Now has {len(self.triangles)} triangles')

            
    def remove_point_if_close(self, point, delete_radius, remake_obstacle=True):
        delete_any = False
        for p in self.points:
            if abs(point - p) <= delete_radius:
                self.points.remove(p)
                delete_any = True
        
        if delete_any and remake_obstacle:
            self.make_obstacle()
        return delete_any
    

    
    @staticmethod
    def load_vector_list(data):
        out = []
        for item in data:
            out.append(V(item["x"],item["y"]))
        return out
    
    @staticmethod
    def dump_vector_list(data):
        out = []
        for item in data:
            out.append({
                "x": item.x,
                "y": item.y
            })
        return out

    def save_to_dict(self):
        to_save_raw = "draw_mesh draw_options".split(' ')
        data = {}
        for var in to_save_raw:
            data[var] = self.__getattribute__(var)
        data["points"] = self.dump_vector_list(self.points)
        data["triangles"] = []
        for triangle in self.triangles:
            data["triangles"].append(self.dump_vector_list(triangle))
        data["polygon"] = self.dump_vector_list(self.polygon)
        return data
    

    @staticmethod
    def load_from_dict(data):
        to_load = "draw_options".split(' ')

        points = Obstacle.load_vector_list(data["points"])
        polygon = Obstacle.load_vector_list(data["polygon"])
        triangles = []
        for t in data["triangles"]:
            triangles.append(Obstacle.load_vector_list(t))

        o = Obstacle(points, data["draw_mesh"], False)
        for var in to_load:
            o.__setattr__(var, data[var])
        o.polygon = polygon
        o.triangles = triangles
        return o


    def draw(self, scene):
        for triangle in self.triangles:
            scene.draw_filled_triangle(triangle, self.draw_options["polygon"]["color"])

        if self.draw_mesh:
            if len(self.triangles) > 0:
                for triangle in self.triangles:
                    scene.draw_polygon(triangle, self.draw_options["triangles"]["color"], self.draw_options["triangles"]["radius"])
                for point in self.polygon:
                    scene.draw_point(point, self.draw_options["points"]["color"], self.draw_options["points"]["radius"])
            else:
                for point in self.points:
                    scene.draw_point(point, self.draw_options["points"]["color"], self.draw_options["points"]["radius"])
        else:
            scene.draw_polygon(self.polygon, self.draw_options["outline"]["color"], self.draw_options["outline"]["width"])
            for point in self.polygon:
                scene.draw_point(point, self.draw_options["points"]["color"], self.draw_options["points"]["radius"])
