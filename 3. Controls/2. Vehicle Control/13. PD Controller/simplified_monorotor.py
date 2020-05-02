import numpy as np
from matplotlib import pyplot as plt


class Monorotor:

    def __init__(self, m=1.0):
        self.m = m
        self.g = 9.81

        # note that we're no longer thinking of rotation rates.
        # We are thinking directly in terms of thrust.
        self.thrust = 0.0

        # z, z_dot
        self.X = np.array([0.0,0.0])

    @property
    def z(self):
        return self.X[0]

    @property
    def z_dot(self):
        return self.X[1]

    @property
    def z_dot_dot(self):
        f_net = self.m * self.g - self.thrust
        return f_net / self.m

    def advance_state(self, dt):
        X_dot =np.array([
            self.z_dot,
            self.z_dot_dot])

        self.X = self.X + X_dot * dt
        return self.X