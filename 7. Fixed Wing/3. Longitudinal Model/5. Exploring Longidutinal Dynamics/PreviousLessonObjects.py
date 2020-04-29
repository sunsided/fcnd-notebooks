import numpy as np
from math import sin, cos
import matplotlib.pyplot as plt
import matplotlib.pylab as pylab
import jdc
from ipywidgets import interactive
from scipy.stats import multivariate_normal

import time


class AeroDynamics():

    def __init__(self):
        '''
        Importing the airplane model coefficients from the other object.
        '''

        self.rho = 1.2682               # [kg/m^3] density of air.
        self.g = 9.81                   # [m/s^2] gravitational acceleration

        self.s = 16.1651                # [m^2] aircraft's wing area
        self.c = 1.49352                # [m] the mean aerodynamic chord
        self.b = 10.9728                # [m]  # wingspan
        self.mass = 1202.02             # [kg] mass of airplane


        self.c_l_0  = 0.307             # non-dimensional coefficient of lift at zero angle of attack
        self.c_l_alpha = 4.41           # non-dimensional lift slope
        self.c_l_delta_e = 0.43         # non-dimensional lift control derivative regarding elevator angle
        self.c_l_alpha_2 = 0.0          # the non-dimensional lift coefficient relative to the square of the angle of attack


        # drag coefficients
        self.c_d_0 = 0.0270             # non-dimensional coefficient of drag at zero angle of attack
        self.epsilon = 0.1592           # induced drag factor

        # pitch moment
        self.c_m_0 =  0.04              # non-dimensional; coefficient of pitching moment at zero angle of attack
        self.c_m_alpha = -0.613         # non-dimensional pitching slope
        self.c_m_delta_e = -1.122       # non-dimensional pitching slope for elevator


        self.c_d_alpha = 0.121
        self.c_l_q = 3.9
        self.c_d_q = 0.0
        self.c_m_q = -12.4             # pitch damping derivative
        self.c_d_delta_e = 0.0

        # propeller data                # Propeller data are from UAV
        self.k_motor = 80
        self.s_prop = 2.83              # [m^2]
        self.c_prop = 1.0
        self.k_Tp = 0.0
        self.k_omega = 0.0



        # Lateral coefficients
        self.c_y_0 = 0.0
        self.c_l_0 = 0.0
        self.c_n_0 = 0.0

        self.c_y_beta = -0.393
        self.c_l_beta = -0.0923         # roll static stability derivative
        self.c_n_beta = 0.0587          # yaw static stability derivative

        self.c_y_p = -0.075
        self.c_l_p = -0.484
        self.c_n_p = -0.0278

        self.c_y_r = 0.214
        self.c_l_r = 0.0798
        self.c_n_r = -0.0937

        self.c_y_delta_a = 0.0
        self.c_l_delta_a = 0.229        # primary control derivative
        self.c_n_delta_a = -0.0216

        self.c_y_delta_r = 0.87
        self.c_l_delta_r = 0.0147
        self.c_n_delta_r = -0.0645      # primary control derivative

        self.j_x = 1285.3154166         # kg*m^2
        self.j_y = 1824.9309607         # kg*m^2
        self.j_z = 2666.89390765        # kg*m^2
        self.j_xz = 0.0                 # kg*m^2

        self.j = np.array([[self.j_x,   0,   -self.j_xz],
                           [0,     self.j_y, 0],
                           [-self.j_xz, 0,   self.j_z]])






    def c_l(self, alpha, delta_e):
        '''
        Variables:
            alpha: angle of attack
            delta_e: the deflection angle of the elevator
        '''
        c_l = self.c_l_0 + self.c_l_alpha * alpha + self.c_l_delta_e * delta_e

        return c_l

    def lift(self, v_a, alpha, delta_e):
        '''
        Variables:
            v_a: velocity of the airplane - true airspeed (TAS)
            alpha: angle of attack
            delta_e: the deflection angle of the elevator

        '''

        q = self.rho * v_a **2 / 2

        l = self.c_l(alpha,delta_e) * q * self.s

        return l

    def c_d(self, alpha):
        '''
        Variables:
            alpha: angle of attack
        '''

        c_d = self.c_d_0 + self.epsilon * self.c_l_0**2 #+ c_d_alpha + c_d_alpha_2 * alpha**2

        return c_d


    def drag(self, v_a, alpha):
        '''
        Variables:
            v_a: velocity of the airplane - true airspeed (TAS)
            alpha: angle of attack
        '''

        q = self.rho * v_a **2 / 2

        d = self.c_d(alpha) * q * self.s

        return d

    def c_m(self,alpha,delta_e):
        '''
        Variables:
            alpha: angle of attack
        '''
        c_m = self.c_m_0 + self.c_m_alpha * alpha + self.c_m_delta_e * delta_e

        return c_m


    def pitch_moment(self, v_a, alpha, delta_e):
        '''
        Variables:
            alpha:
        '''
        q = self.rho * v_a **2 / 2

        m = self.c_m(alpha, delta_e) * q * self.s * self.c

        return m


class TrimCondition(AeroDynamics):

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
