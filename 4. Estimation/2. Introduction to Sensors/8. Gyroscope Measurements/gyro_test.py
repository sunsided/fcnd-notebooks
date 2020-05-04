import numpy as np
import math
import unittest


class TestCode:

    def __init__(self):
        pass



    def test_calculate_attitude(true_angle,                # The true value of the angular velocity
                              measured_angle,          # Previously estimated angle by integration
                              accumulated_sigma,         # Accumulated effective sigma due to integration
                              omega,                     # The true omega
                              measured_omega,            # Measured omega
                              sigma_omega,               # The uncertainty of each measurement
                              dt):                       # The time interval between measurements

        true_angle = true_angle + omega*dt
        measured_angle = measured_angle + measured_omega*dt
        accumulated_sigma = np.sqrt(accumulated_sigma**2 + dt**2*sigma_omega**2)

        return true_angle, measured_angle, accumulated_sigma



    def test_the_calculate_attitude(student_function,test_calculate_attitude = test_calculate_attitude):
        '''
        This will test the student function for the attitude dt time advancement
        '''

        true_angle = 0.0
        measured_angle = 0.0
        accumulated_sigma = 0.0
        omega = 0.2
        measured_omega =0.21
        sigma_omega = 0.1
        dt = 0.01

        epsilon=10**(-6)

        student_true_angle, student_measured_angle, student_accumulated_sigma = student_function(true_angle,
                                                                                                    measured_angle,
                                                                                                    accumulated_sigma,
                                                                                                    omega,
                                                                                                    measured_omega,
                                                                                                    sigma_omega,
                                                                                                    dt)

        test_true_angle, test_measured_angle, test_accumulated_sigma = test_calculate_attitude(true_angle,
                                                                                                    measured_angle,
                                                                                                    accumulated_sigma,
                                                                                                    omega,
                                                                                                    measured_omega,
                                                                                                    sigma_omega,
                                                                                                    dt)

        if abs(student_true_angle - test_true_angle) < epsilon:
            print('True angle is correct. ')

        if abs(student_measured_angle - test_measured_angle) < epsilon:
            print('Measured angle calculation is correct. ')


        if abs(student_accumulated_sigma - test_accumulated_sigma) < epsilon:
            print('Accumulated sigma calculation is correct.')



