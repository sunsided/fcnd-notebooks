import numpy as np 
import math
from math import sin, cos, tan

class ObjectInThreeD:
    
    def __init__(self,
                dt
                ):
        '''
        Initialize the object with zero state space and the gravity constant
        '''
        
        self.dt = dt
        self.X = np.zeros((8, 1))
        self.g = 9.81 * np.array([[0], [0], [-1]])# The opposite of the gravity thus directed opposite of the z-axis. 
        
    
    def rotation_matrix(self, phi, theta):
        '''
        Generates the rotation matrix for the given roll and pitch angles
        The yaw angle is assumed to be equal to zero. 
        '''
        
        psi = 0.0 
        r_x = np.array([[1, 0, 0],
                        [0, np.cos(phi), -np.sin(phi)],
                        [0, np.sin(phi), np.cos(phi)]])

        r_y = np.array([[np.cos(theta), 0, np.sin(theta)],
                        [0, 1, 0],
                        [-np.sin(theta), 0, np.cos(theta)]])

        r_z = np.array([[np.cos(psi), -np.sin(psi), 0],
                        [np.sin(psi), np.cos(psi), 0],
                        [0, 0, 1]])

        r = np.matmul(r_z, np.matmul(r_y, r_x))

        return r 


    def get_euler_derivatives(self, phi, theta, p, q):
        '''
        Converts the measured body rates into the Euler angle derivatives using the estimated pitch and roll angles.
        '''
        # TODO: Calculate the Derivatives for the Euler angles in the inertial frame 
        euler_rot_mat= np.array([[1, np.sin(phi) * np.tan(theta)],
                                 [0, np.cos(phi)]])

        derivatives_in_bodyframe = np.array([p, q])

        euler_dot = np.matmul(euler_rot_mat, derivatives_in_bodyframe)

        return euler_dot


    def linear_acceleration(self, measured_acceleration, phi, theta):
        '''
        Calculates the true accelerations in the inertial frame of reference based 
        on the measured values and the estimated roll and pitch angles. 
        '''
        # TODO: Calculate the true acceleration in body frame by removing the gravity component
        a_body_frame = measured_acceleration - np.matmul(self.rotation_matrix(phi, theta), self.g)

        # TODO: Convert the true acceleration back to the inertial frame
        a_inertial_frame = np.matmul(np.linalg.inv(self.rotation_matrix(phi, theta)), a_body_frame)

        return a_inertial_frame
    
    def dead_reckoning_orientation(self, p, q):
        '''
        Updates the orientation of the drone for the measured body rates
        '''
        # TODO: Update the state vector component of roll and pitch angles
        phi = self.X[4]
        theta = self.X[3]

        euler_dot = self.get_euler_derivatives(phi, theta, p, q)
        self.X[3] = self.X[3] + euler_dot[1] * self.dt         # integrating the roll angle 
        self.X[4] = self.X[4] + euler_dot[0] * self.dt         # integrating the roll angle 
        
        
    def dead_reckoning_position(self, measured_acceleration):
        '''
        Updates the position and the linear velocity in the inertial frame based on the measured accelerations
        '''
        perceived_phi = self.X[4]
        perceived_theta = self.X[3]

        # TODO: Conver the body frame acceleration measurements back to the inertial frame measurements
        a = self.linear_acceleration(measured_acceleration, perceived_phi, perceived_theta) 

        # TODO: 
        # Use the velocity components of the state vector to update the position components of the state vector
        # Use the linear acceleration in the inertial frame to update the velocity components of the state vector

        self.X[0] = self.X[0] + self.X[5] * self.dt   # x coordianate x= x + \dot{x} * dt
        self.X[1] = self.X[1] + self.X[6] * self.dt   # y coordianate y= y + \dot{y} * dt
        self.X[2] = self.X[2] + self.X[7] * self.dt   # z coordianate z= z + \dot{z} * dt
        self.X[5] = self.X[5] + a[0] * self.dt        # change in velocity along the x is a_x * dt 
        self.X[6] = self.X[6] + a[1] * self.dt        # change in velocity along the y is a_y * dt 
        self.X[7] = self.X[7] + a[2] * self.dt        # change in velocity along the z is a_z * dt 
        
    def advance_state(self, measured_acceleration, p, q, dt): 
        '''
        The function of the advance state updated the position of the drone 
        in the inertial frame and then the drone attitude.
        '''

        # Advance the position 
        self.dead_reckoning_position(measured_acceleration)

        # Advance the attitude 
        self.dead_reckoning_orientation(p, q)

        return self.X 