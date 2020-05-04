import numpy as np 
import math

class TestCode:

    def __init__(self):

        pass



    def test_the_linear_acceleration(student_answer, measured_acceleration, phi, theta):
        psi = 0.0 
        g = 9.81 * np.array([[0], [0], [-1]])
        r_x = np.array([[1, 0, 0],
                        [0, np.cos(phi), -np.sin(phi)],
                        [0, np.sin(phi), np.cos(phi)]])

        r_y = np.array([[np.cos(theta), 0, np.sin(theta)],
                        [0, 1, 0],
                        [-np.sin(theta), 0, np.cos(theta)]])

        r_z = np.array([[np.cos(psi), -np.sin(psi), 0],
                        [np.sin(psi), np.cos(psi), 0],
                        [0,0,1]])

        r = np.matmul(r_z, np.matmul(r_y,r_x))

    
        # calculating the true acceleration in body frame by removing the gravity component
        a_body_frame = measured_acceleration - np.matmul(r, g)
        
        # converting the true acceleration back to the inertial frame
        a_inertial_frame = np.matmul(np.linalg.inv(r), a_body_frame)
        
        epsilon = 10**(-4)
        if np.all(abs(student_answer- a_inertial_frame) < np.array([[1],[1],[1]])* epsilon):
            print('Test passed')
        else:
            print('Test failed')




    def test_the_accelerometer_measurement(student_answer, acctual_a, phi, theta, sigma_a=0.0):

        psi = 0.0 
        g = 9.81 * np.array([[0], [0], [-1]])
        r_x = np.array([[1, 0, 0],
                        [0, np.cos(phi), -np.sin(phi)],
                        [0, np.sin(phi), np.cos(phi)]])

        r_y = np.array([[np.cos(theta), 0, np.sin(theta)],
                        [0, 1, 0],
                        [-np.sin(theta), 0, np.cos(theta)]])

        r_z = np.array([[np.cos(psi), -np.sin(psi), 0],
                        [np.sin(psi), np.cos(psi), 0],
                        [0,0,1]])

        r = np.matmul(r_z,np.matmul(r_y,r_x))


        # Converting global frame acceleration into the body frame acceleration
        acctual_a = acctual_a.reshape(3,1)
        linear_acc_bodyframe = np.matmul(r, acctual_a)
         
        # Adding the gravity component
        gravity_component = np.matmul(r, g)
         
        # Error component
        error_component = np.random.normal(0.0, sigma_a, (3, 1))
        
        measured_acceleration = linear_acc_bodyframe + gravity_component + error_component
        
        epsilon = 10**(-3)

        if np.all(abs(student_answer- measured_acceleration) < np.array([[1],[1],[1]])* epsilon):
            print('Test passed')
        else:
            print('Test failed')



    def test_the_gyroscope_measurement(student_answer,phi, theta, phi_dot, theta_dot, sigma_omega=0.0):
        # Conversion matrix 
        R =np.array([[1, np.sin(phi)*np.tan(theta)], [0, np.cos(phi)]])
        
        # Body rate p and q true values 
        body_rate = np.matmul(np.linalg.inv(R) , np.array([[phi_dot], [theta_dot]]))
        
        # Adding an error to the true body rates
        measured_bodyrates = body_rate + np.random.normal(0.0, sigma_omega, (2, 1))
                              
        epsilon = 10**(-4)

        if np.all(abs(student_answer- measured_bodyrates) < np.array([[1],[1]])* epsilon):
            print('Test passed')
        else:
            print('Test failed')
