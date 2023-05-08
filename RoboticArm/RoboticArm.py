import numpy as np
from lib.Math.Vector import Vector2 as V

from RoboticArm.colors import *
from RoboticArm.utils.mesh_generation import *
from RoboticArm.ArmPiece import ArmPiece
from RoboticArm.ObstaclesManager import ObstaclesManager

import numpy as np

from dataclasses import dataclass
import matplotlib.pyplot as plt

@dataclass
class CSPaceData:
    theta_1_min: float
    theta_1_max: float
    theta_1_num_samples: int
    theta_2_min: float
    theta_2_max: float
    theta_2_num_samples: int

    data: np.array


class RoboticArm:
    def __init__(self, position: V, theta1: float, theta2: float, piece_size1: V, piece_size2: V, pivot1: V, pivot2: V, attach1: V, attach2: V):
        self.arm_piece1 = ArmPiece(position, theta1, piece_size1, pivot1, attach1)
        self.arm_piece2 = ArmPiece(self.arm_piece1.attach_position, theta1 + theta2, piece_size2, pivot2, attach2)

    def does_intersect(self, obstacles: ObstaclesManager):
        for A in obstacles.get_all_triangles():
            for B in self.get_all_triangles():
                if triangle_intersection(A, B):
                    return True
        return False
                

    def get_all_triangles(self):
        return self.arm_piece1.get_arm_triangles() + self.arm_piece2.get_arm_triangles()
    

    def update_angles(self, theta1, theta2):
        self.arm_piece1.update_theta(theta1)
        self.arm_piece2.update(self.arm_piece1.attach_position, theta1 + theta2)
    

    def computeCSPace(self, theta1_range: float, theta2_range: float, num_samples: float, obstacles: ObstaclesManager):
        m1, M1 = theta1_range
        m2, M2 = theta2_range
        N = num_samples

        data = CSPaceData(
            m1, M1, N,
            m2, M2, N,
            np.zeros(shape=(N,N)))
        
        old_theta1 = self.arm_piece1.theta
        old_theta2 = self.arm_piece2.theta

        for (i, theta1) in enumerate(np.linspace(m1, M1, N)):
          for (j, theta2) in enumerate(np.linspace(m2, M2, N)):
                self.update_angles(theta1, theta2)
                data.data[i, j] = self.does_intersect(obstacles)
                
        self.update_angles(old_theta1, old_theta2 - old_theta1)

        plt.imshow(data.data)
        plt.show()


    def draw(self, scene):
        self.arm_piece1.draw(scene)
        self.arm_piece2.draw(scene)
