import numpy as np


class TestAerodynamics:

    @staticmethod
    def test_lift(student_answer,
                    v_a,
                    alpha,
                    delta_e,
                    c_l_0,
                    c_l_alpha,
                    c_l_delta_e,
                    rho,
                    s):
        '''
        we will get the student calculated lift value and the coefficient based on which it was calculated
        Args:
            student_answer:
            v_a: velocity of the airplane
            alpha: angle of attack
            delta_e: the deflection angle of the elevator
            c_l_0: the non-dimensional coefficient of lift at zero angle of attack
            c_l_alpha: the non-dimensional lift slope
            c_l_delta_e: the non-dimensional lift control derivative regarding elevator angle
            rho: the density of air
            s: aircraft's wing area


        Returns:
            printing if test passed or not.

        '''

        c_l = c_l_0 + c_l_alpha * alpha + c_l_delta_e * delta_e

        q = rho * v_a **2 / 2

        l = c_l * q * s

        internal_answer = l

        epsilon = 10**(-4)

        if (student_answer - internal_answer) < epsilon:
            print('Test Passed')
        else:
            print('Test failed')



    @staticmethod
    def test_drag(student_answer,
                  v_a,
                  alpha,
                  epsilon,
                  c_l_0,
                  c_l_alpha,
                  c_l_alpha_2,
                  c_d_0,
                  rho,
                  s):
        '''
        we will get the student calculated lift value and the coefficient based on which it was calculated
        Args:
            student_answer:
            v_a: velocity of the airplane
            alpha: angle of attack
            epsilon: the non-dimensional coefficient of induced drag factor
            c_l_0: the non-dimensional coefficient of lift at zero angle of attack
            c_l_alpha: the non-dimensional lift slope
            c_l_alpha_2: the non-dimensional lift coefficient relative to the square of the angle of attack
            c_d_0: the non-dimensional coefficient of drag at zero angle of attack
            rho: the density of air
            s: aircraft's wing area



        Returns:
            printing if test passed or not.

        '''

        c_d_alpha = 2 * epsilon * c_l_0 * c_l_alpha
        c_d_alpha_2 = epsilon * c_l_alpha_2
        c_d = c_d_0 + epsilon * c_l_0**2 + c_d_alpha + c_d_alpha_2 * alpha**2

        q = rho * v_a **2 / 2

        d = c_d * q * s

        internal_answer = d

        epsilon = 10**(-4)

        if (student_answer - internal_answer) < epsilon:
            print('Test Passed')
        else:
            print('Test failed')


    def test_pitch(student_answer,
                    v_a,
                    alpha,
                    delta_e,
                    c_m_0,
                    c_m_alpha,
                    c_m_delta_e,
                    rho,
                    s,
                    c):
        '''
        we will get the student calculated lift value and the coefficient based on which it was calculated
        Args:
            student_answer:
            v_a: velocity of the airplane
            alpha: angle of attack
            delta_e: the deflection angle of the elevator
            c_l_0: the non-dimensional coefficient of lift at zero angle of attack
            c_l_alpha: the non-dimensional lift slope
            c_l_delta_e: the non-dimensional lift control derivative regarding elevator angle
            rho: the density of air
            s: aircraft's wing area
            c: the mean aerodynamic chord


        Returns:
            printing if test passed or not.

        '''

        c_m = c_m_0 + c_m_alpha * alpha + c_m_delta_e * delta_e

        q_bar = rho * v_a **2 / 2

        m = c_m * q_bar * s * c

        internal_answer = m

        epsilon = 10**(-4)

        if (student_answer - internal_answer) < epsilon:
            print('Test Passed')
        else:
            print('Test failed')