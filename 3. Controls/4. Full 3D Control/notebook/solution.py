import numpy as np
import math
from math import sin, cos, tan, sqrt
import matplotlib.pyplot as plt
import matplotlib.pylab as pylab
from mpl_toolkits.mplot3d import Axes3D
import random


class UDACITYDroneIn3D:

    def __init__(self,
                k_f = 1.0,
                k_m = 1.0,
                m = 0.5,
                L = 0.566, # full rotor to rotor distance
                i_x = 0.1,
                i_y = 0.1,
                i_z = 0.2):

        self.k_f = k_f
        self.k_m = k_m
        self.m = m
        self.l = L / ( 2 * sqrt(2) ) # perpendicular distance to axes
        self.i_x = i_x
        self.i_y = i_y
        self.i_z = i_z

        # x, y, y, phi, theta, psi, x_dot, y_dot, z_dot, p, q, r
        self.X=np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
        self.omega = np.array([0.0,0.0,0.0,0.0])

        self.g = 9.81

    # euler angles [rad] (in world / lab frame)
    @property
    def phi(self):
        return self.X[3]

    @property
    def theta(self):
        return self.X[4]

    @property
    def psi(self):
        return self.X[5]

    # body rates [rad / s] (in body frame)
    @property
    def p(self):
        return self.X[9]

    @property
    def q(self):
        return self.X[10]

    @property
    def r(self):
        return self.X[11]


    # forces from the four propellers
    @property
    def f_1(self):
        f = self.k_f*self.omega[0]**2
        return f

    @property
    def f_2(self):
        f = self.k_f*self.omega[1]**2
        return f

    @property
    def f_3(self):
        f = self.k_f*self.omega[2]**2
        return f

    @property
    def f_4(self):
        f = self.k_f*self.omega[3]**2
        return f

    # collective force
    @property
    def f_total(self):
        f_t = self.f_1 + self.f_2 + self.f_3 + self.f_4
        return f_t


    # Reactive moments 1 through 4
    @property
    def tau_1(self):
        tau = -self.k_m * self.omega[0]**2
        return tau

    @property
    def tau_2(self):
        tau = self.k_m * self.omega[1]**2
        return tau

    @property
    def tau_3(self):
        tau = -self.k_m * self.omega[2]**2
        return tau

    @property
    def tau_4(self):
        tau = self.k_m * self.omega[3]**2
        return tau

    @property
    def tau_x(self):
        tau = self.l*(self.f_1 + self.f_4 - self.f_2 - self.f_3)
        return tau

    @property
    def tau_y(self):
        tau = self.l*(self.f_1 + self.f_2 - self.f_3 - self.f_4)
        return tau

    @property
    def tau_z(self):
        tau = self.tau_1 + self.tau_2 + self.tau_3 + self.tau_4
        return tau

    def set_propeller_angular_velocities(self,
                                        c,
                                        u_bar_p,
                                        u_bar_q,
                                        u_bar_r):

        c_bar = -c * self.m / self.k_f
        p_bar = u_bar_p * self.i_x / (self.k_f * self.l)
        q_bar = u_bar_q * self.i_y / (self.k_f * self.l)
        r_bar = u_bar_r * self.i_z / self.k_m

        omega_4 = (c_bar + p_bar - r_bar - q_bar)/4
        omega_3 = (r_bar - p_bar)/2 + omega_4
        omega_2 = (c_bar - p_bar)/2 - omega_3
        omega_1 = c_bar - omega_2 - omega_3 - omega_4

        self.omega[0] = -np.sqrt(omega_1)
        self.omega[1] = np.sqrt(omega_2)
        self.omega[2] = -np.sqrt(omega_3)
        self.omega[3] = np.sqrt(omega_4)

    def R(self):
        r_x = np.array([[1, 0, 0],
                    [0, cos(self.phi), -sin(self.phi)],
                    [0, sin(self.phi), cos(self.phi)]])

        r_y = np.array([[cos(self.theta), 0, sin(self.theta)],
                        [0, 1, 0],
                        [-sin(self.theta), 0, cos(self.theta)]])

        r_z = np.array([[cos(self.psi), -sin(self.psi), 0],
                        [sin(self.psi), cos(self.psi), 0],
                        [0,0,1]])

        r = np.matmul(r_z,np.matmul(r_y,r_x))
        return r


    def linear_acceleration(self):
        R = self.R()
        g = np.array([0,0,self.g]).T
        c = -self.f_total
        accelerations =  g + np.matmul(R,np.array([0,0,c]).T) / self.m
        return accelerations


    def get_omega_dot(self):
        p_dot = self.tau_x/self.i_x - self.r * self.q *(self.i_z - self.i_y)/self.i_x
        q_dot = self.tau_y/self.i_y - self.r * self.p *(self.i_x - self.i_z)/self.i_y
        r_dot = self.tau_z/self.i_z - self.q * self.p *(self.i_y - self.i_x)/self.i_z

        return np.array([p_dot,q_dot,r_dot])


    def get_euler_derivatives(self):

        euler_rot_mat= np.array([[1, sin(self.phi) * tan(self.theta), cos(self.phi) * tan(self.theta)],
                                 [0, cos(self.phi), -sin(self.phi)],
                                 [0, sin(self.phi) / cos(self.theta), cos(self.phi) / cos(self.theta)]])


        derivatives_in_bodyframe =np.array([self.p,self.q,self.r]).T

        euler_dot= np.matmul(euler_rot_mat,derivatives_in_bodyframe)

        return euler_dot






    def advance_state(self, dt):

        euler_dot_lab = self.get_euler_derivatives()
        body_frame_angle_dot = self.get_omega_dot()
        accelerations = self.linear_acceleration()

        X_dot = np.array([self.X[6],
                          self.X[7],
                          self.X[8],
                          euler_dot_lab[0],
                          euler_dot_lab[1],
                          euler_dot_lab[2],
                          accelerations[0],
                          accelerations[1],
                          accelerations[2],
                          body_frame_angle_dot[0],
                          body_frame_angle_dot[1],
                          body_frame_angle_dot[2]])

        self.X = self.X + X_dot * dt

        return self.X


class UDACITYController:

    def __init__(self,
                z_k_p=1.0,
                z_k_d=1.0,
                x_k_p=1.0,
                x_k_d=1.0,
                y_k_p=1.0,
                y_k_d=1.0,
                k_p_roll=1.0,
                k_p_pitch=1.0,
                k_p_yaw=1.0,
                k_p_p=1.0,
                k_p_q=1.0,
                k_p_r=1.0):


        self.z_k_p = z_k_p
        self.z_k_d = z_k_d
        self.x_k_p = x_k_p
        self.x_k_d = x_k_d
        self.y_k_p = y_k_p
        self.y_k_d = y_k_d
        self.k_p_roll = k_p_roll
        self.k_p_pitch = k_p_pitch
        self.k_p_yaw = k_p_yaw
        self.k_p_p = k_p_p
        self.k_p_q = k_p_q
        self.k_p_r = k_p_r

        print('x: delta = %5.3f'%(x_k_d/2/math.sqrt(x_k_p)), ' omega_n = %5.3f'%(math.sqrt(x_k_p)))
        print('y: delta = %5.3f'%(y_k_d/2/math.sqrt(y_k_p)), ' omega_n = %5.3f'%(math.sqrt(y_k_p)))
        print('z: delta = %5.3f'%(z_k_d/2/math.sqrt(z_k_p)), ' omega_n = %5.3f'%(math.sqrt(z_k_p)))

        self.g= 9.81


    def altitude_controller(self,
                           z_target,
                           z_dot_target,
                           z_dot_dot_target,
                           z_actual,
                           z_dot_actual,
                           rot_mat):

        z_err = z_target - z_actual
        z_err_dot = z_dot_target - z_dot_actual
        b_z = rot_mat[2,2]

        p_term = self.z_k_p * z_err
        d_term = self.z_k_d * z_err_dot

        u_1_bar = p_term + d_term + z_dot_dot_target

        c = (u_1_bar - self.g)/b_z

        return c


    def lateral_controller(self,
                          x_target,
                          x_dot_target,
                          x_dot_dot_target,
                          x_actual,
                          x_dot_actual,
                          y_target,
                          y_dot_target,
                          y_dot_dot_target,
                          y_actual,
                          y_dot_actual,
                          c):

        x_err = x_target - x_actual
        x_err_dot = x_dot_target - x_dot_actual

        p_term_x = self.x_k_p * x_err
        d_term_x = self.x_k_d * x_err_dot

        x_dot_dot_command = p_term_x + d_term_x + x_dot_dot_target

        b_x_c = x_dot_dot_command/c


        y_err = y_target - y_actual
        y_err_dot = y_dot_target - y_dot_actual

        p_term_y = self.y_k_p * y_err
        d_term_y = self.y_k_d * y_err_dot

        y_dot_dot_command = p_term_y + d_term_y + y_dot_dot_target

        b_y_c = y_dot_dot_command/c

        return b_x_c, b_y_c



    def roll_pitch_controller(self,
                              b_x_c,
                              b_y_c,
                              rot_mat):
        b_x = rot_mat[0,2]
        b_x_err = b_x_c - b_x
        b_x_p_term = self.k_p_roll * b_x_err

        b_y = rot_mat[1,2]
        b_y_err = b_y_c - b_y
        b_y_p_term = self.k_p_pitch * b_y_err

        b_x_commanded_dot = b_x_p_term
        b_y_commanded_dot = b_y_p_term

        rot_mat1=np.array([[rot_mat[1,0],-rot_mat[0,0]],[rot_mat[1,1],-rot_mat[0,1]]])/rot_mat[2,2]

        rot_rate = np.matmul(rot_mat1,np.array([b_x_commanded_dot,b_y_commanded_dot]).T)
        p_c = rot_rate[0]
        q_c = rot_rate[1]

        return p_c, q_c


    def yaw_controller(self,
                       psi_target,
                       psi_actual):

        psi_err = psi_target - psi_actual
        r_c = self.k_p_yaw * psi_err

        return r_c


    def body_rate_controller(self,
                             p_c,
                             q_c,
                             r_c,
                             p_actual,
                             q_actual,
                             r_actual):

        p_err= p_c - p_actual
        u_bar_p = self.k_p_p * p_err

        q_err= q_c - q_actual
        u_bar_q = self.k_p_q * q_err

        r_err= r_c - r_actual
        u_bar_r = self.k_p_r * r_err

        return u_bar_p, u_bar_q, u_bar_r

    def attitude_controller(self,
                           b_x_c_target,
                           b_y_c_target,
                           psi_target,
                           psi_actual,
                           p_actual,
                           q_actual,
                           r_actual,
                           rot_mat):

        p_c, q_c = self.roll_pitch_controller(b_x_c_target,
                                              b_y_c_target,
                                              rot_mat)

        r_c = self.yaw_controller(psi_target,
                                  psi_actual)

        u_bar_p, u_bar_q, u_bar_r = self.body_rate_controller(p_c,
                                                              q_c,
                                                              r_c,
                                                              p_actual,
                                                              q_actual,
                                                              r_actual)

        return u_bar_p, u_bar_q, u_bar_r


