import numpy as np
from math import sin, cos
import matplotlib.pyplot as plt
import matplotlib.pylab as pylab
import jdc
from ipywidgets import interactive
from scipy.stats import multivariate_normal
import time

from Cessna import AeroDynamicsCoefficients


class TrimCondition(AeroDynamicsCoefficients):

    def __init__(self):
        super(TrimCondition, self).__init__()

        pass

    def v_min_t(self):
        '''Minimum thrust velocity

        args:
            '''
        v_min_t = np.sqrt(2*self.mass * self.g/(self.rho * self.s)*np.sqrt(self.epsilon/(self.c_d_0)))

        return v_min_t


    def thrust(self,v_a):

        thrust = 0.5 * self.c_d_0 * self.rho * v_a**2 * self.s \
                 + 2 * self.epsilon * self.mass**2 * self.g**2 /(self.rho * v_a**2 * self.s)

        return thrust


    def delta_e(self,alpha_trim):
        '''calculates deflection angle of the elevator.

        args:
            alpha_trim: Trim angle of attack for the desired velocity
        '''
        delta_e = -(self.c_m_0 + self.c_m_alpha * alpha_trim)/self.c_m_delta_e

        return delta_e


    def alpha_for_trim(self, v_a):
        '''
        Calculates trim angle of attack for desired velocity.

        args:
            v_a: velocity of the airplane - true airspeed (TAS)
        '''

        alpha_trim = (2*self.mass * self.g /(self.rho * v_a**2 * self.s)-self.c_l_0)/self.c_l_alpha

        return alpha_trim


    def climbing_velocity(self):

        v_climb = np.sqrt(2*self.mass *self.g /(self.s * self.rho)* np.sqrt(self.epsilon /(3* self.c_d_0)))

        return v_climb


    def angle_of_climb(self,v_climb):
        '''calculates the angle of climb for a given thrust
        '''
        alpha =self.alpha(v_climb)
        drag = self.drag(v_climb, alpha)
        thrust = self.thrust(v_climb)
        climb_angle = np.arcsin((thrust - drag)/(self.mass * self.g))

        return climb_angle
