import numpy as np
from lib.Math.Vector import Vector2 as V

from RoboticArm.colors import *
from RoboticArm.utils.mesh_generation import *
from RoboticArm.ArmPiece import ArmPiece


class RoboticArm:
    def __init__(self, position, theta1, theta2, piece_size1, piece_size2, pivot1, pivot2, attach1, attach2):
        self.arm_piece1 = ArmPiece(position, theta1, piece_size1, pivot1, attach1)
        self.arm_piece2 = ArmPiece(position + attach1.rotate_by_angle(theta1), theta1 + theta2, piece_size2, pivot2, attach2)

    def draw(self, scene):
        self.arm_piece1.draw(scene)
        self.arm_piece2.draw(scene)
