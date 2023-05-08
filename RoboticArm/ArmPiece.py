from RoboticArm.colors import *
from lib.Math.Vector import Vector2 as V

class ArmPiece:
    def __init__(self, pivot_position, theta, size, pivot, attach):
        self.pivot_position = pivot_position
        self.theta = theta
        self.size = size
        self.pivot_offset = pivot # in the arm piece frame
        self.attach_offset = attach

        self.draw_options = {
            "arm": {
                "color": (70,70,70)
            },
            "triangles": {
                "color" : (40,40,40),
                "width": 2
            },
            "corners": {
                "color": BLACK,
                "width": 2
            },
            "pivot": {
                "color": RED,
                "radius": 2
            },
            "attach": {
                "color": BLUE,
                "radius": 2
            }
        }
    
    def update(self, pivot_position, theta):
        self.pivot_position = pivot_position
        self.theta = theta

    @property
    def attach_position(self):
        return self.arm_frame_to_global(self.attach_offset - self.pivot_offset)
    
    def update_theta(self, theta):
        self.theta = theta

    def arm_frame_to_global(self, point):
        return self.pivot_position + point.rotate_by_angle(self.theta)
    
    def get_arm_points(self):
        ref_pivot = V(-self.pivot_offset.x,self.pivot_offset.y)
        A = self.arm_frame_to_global(ref_pivot)
        B = self.arm_frame_to_global(self.size - self.pivot_offset)
        C = self.arm_frame_to_global(V(self.size.x, 0) - self.pivot_offset)
        D = self.arm_frame_to_global(-self.pivot_offset)
        return [A, B, C, D]
    
    def get_arm_triangles(self):
        [A, B, C, D] = self.get_arm_points()
        return [[A, B, C], [A, C, D]]

    def draw(self, scene):
        # draw arm
        scene.draw_filled_quad(self.get_arm_points(), self.draw_options["arm"]["color"])

        # draw triangle outline
        for triangle in self.get_arm_triangles():
            scene.draw_polygon(triangle, self.draw_options["triangles"]["color"], self.draw_options["triangles"]["width"])

        # draw arm exterior points
        for p in self.get_arm_points():
            scene.draw_point(p, self.draw_options["corners"]["color"], self.draw_options["corners"]["width"])
            
        # draw center pivot
        scene.draw_point(self.pivot_position, self.draw_options["pivot"]["color"], self.draw_options["pivot"]["radius"])
            
        # draw attach point
        scene.draw_point(self.attach_position, self.draw_options["attach"]["color"], self.draw_options["attach"]["radius"])
