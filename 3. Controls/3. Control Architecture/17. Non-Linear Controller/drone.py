import numpy as np
import math


class Drone2D:


    def __init__(self,
                 I_x = 0.1, # moment of inertia around the x-axis
                 m = 0.2,   # mass of the vehicle
                ):

        self.I_x = I_x
        self.m = m

        self.u1 = 0.0
        self.u2 = 0.0
        self.g = 9.81

        self.X = np.array([0.0,0.0,0.0,0.0,0.0,0.0])

    @property
    def y_dot_dot(self):
        phi = self.X[2]
        return self.u1 / self.m * np.sin(phi)

    @property
    def z_dot_dot(self):
        phi = self.X[2]
        return self.g - self.u1*math.cos(phi)/self.m

    @property
    def phi_dot_dot(self):
        return self.u2 / self.I_x

    def advance_state(self, dt):

        X_dot = np.array([self.X[3],
                        self.X[4],
                        self.X[5],
                        self.z_dot_dot,
                        self.y_dot_dot,
                        self.phi_dot_dot])


        # Change in state will be
        self.X = self.X + X_dot * dt
        return self.X

    def set_controls(self, u1, u2):
        self.u1 = u1
        self.u2 = u2