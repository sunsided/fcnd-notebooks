import numpy as np 
from math import sin, cos

class TestCode:

    def __init__(self):

        pass



    def linear_acceleration(measured_acceleration, measured_phi):
        '''
        Returns the linear acceleration value in the inertial frame
        '''
        g = 9.81 * np.array([[0], [-1]]) 
        phi = measured_phi
        r1 = np.array([[cos(phi), -sin(phi)], [sin(phi), cos(phi)]])
        r2 = np.array([[cos(phi), sin(phi)], [-sin(phi), cos(phi)]])
        a_body_frame = measured_acceleration - np.matmul(r1, g)
        a_inertial_frame = np.matmul(r2, a_body_frame)

        return a_inertial_frame


    def test_the_linear_acceleration(student_answer, measured_phi, measured_acceleration,
        test_linear_acceleration = linear_acceleration):

        
        a_inertial_frame_test = test_linear_acceleration(measured_acceleration, measured_phi)

        epsilon = 10**(-4)
        if np.all(abs(student_answer- a_inertial_frame_test) < np.array([[1],[1]])* epsilon):
            print('Test passed')
        else:
            print('Test failed')


    def test_the_IMU_acceleration_measurement(student_answer,acctual_a, phi,sigma_a=0.0):

        r = np.array([[cos(phi), -sin(phi)],
                      [sin(phi), cos(phi)]])
        g = 9.81 * np.array([[0], [-1]]) 

        linear_acc_bodyframe = np.matmul(r, acctual_a)
        gravity_component = np.matmul(r, g)
        error_component = np.random.normal(0.0, sigma_a, (2, 1))
        
        measured_acceleration = linear_acc_bodyframe + gravity_component + error_component


        epsilon = 10**(-4)
        if np.all(abs(student_answer- measured_acceleration) < np.array([[1],[1]])* epsilon):
            print('Test passed')
        else:
            print('Test failed')


        