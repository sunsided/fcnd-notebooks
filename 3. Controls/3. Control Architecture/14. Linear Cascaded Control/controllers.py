import math
import numpy as np


class LinearCascadingController:

    def __init__(self,
                 m,   # needed to convert u1_bar to u1
                 I_x, # needed to convert u2_bar to u2
                 z_k_p=1.0,
                 z_k_d=1.0,
                 y_k_p=1.0,
                 y_k_d=1.0,
                 phi_k_p=1.0,
                 phi_k_d=1.0):

        self.z_k_p = z_k_p
        self.z_k_d = z_k_d
        self.y_k_p = y_k_p
        self.y_k_d = y_k_d
        self.phi_k_p = phi_k_p
        self.phi_k_d = phi_k_d

        self.g = 9.81
        self.I_x = I_x
        self.m = m

    def altitude_controller(self,
                    z_target,
                    z_actual,
                    z_dot_target,
                    z_dot_actual,
                    z_dot_dot_target,
                    phi_actual, # unused parameter. Ignore for now.
                    ):
        """
        A PD controller which commands a thrust (u_1)
        for the vehicle.
        """
        z_err = z_target - z_actual
        z_err_dot = z_dot_target - z_dot_actual

        p_term = self.z_k_p * z_err
        d_term = self.z_k_d * z_err_dot

        u_1_bar = p_term + d_term + z_dot_dot_target
        u_1 = self.m* (self.g - u_1_bar)

        return u_1


    def lateral_controller(self,
                        y_target,
                        y_actual,
                        y_dot_target,
                        y_dot_actual,
                        u_1=None, # unused parameter. Ignore for now.
                        y_dot_dot_ff=0.0,
                        ):
        """
        A PD controller which commands a target roll
        angle (phi_commanded).
        """
        y_err = y_target - y_actual
        y_err_dot = y_dot_target - y_dot_actual

        p_term = self.y_k_p * y_err
        d_term = self.y_k_d * y_err_dot

        y_dot_dot_target = p_term + d_term + y_dot_dot_ff
        phi_commanded = y_dot_dot_target / self.g

        return phi_commanded



    def attitude_controller(self,
                            phi_target,
                            phi_actual,
                            phi_dot_actual,
                            phi_dot_target=0.0
                           ):
        """
        A PD controller which commands a moment (u_2)
        about the x axis for the vehicle.
        """

        phi_err = phi_target - phi_actual
        phi_err_dot = phi_dot_target - phi_dot_actual

        p_term = self.phi_k_p * phi_err
        d_term = self.phi_k_d * phi_err_dot

        u_2_bar = p_term + d_term
        u_2 = (u_2_bar) * self.I_x

        return u_2

class NonLinearCascadingController:

    def __init__(self,
                 m,
                 I_x,
                 z_k_p=1.0,
                 z_k_d=1.0,
                 y_k_p=1.0,
                 y_k_d=1.0,
                 phi_k_p=1.0,
                 phi_k_d=1.0):

        self.z_k_p = z_k_p
        self.z_k_d = z_k_d
        self.y_k_p = y_k_p
        self.y_k_d = y_k_d
        self.phi_k_p = phi_k_p
        self.phi_k_d = phi_k_d

        self.g = 9.81
        self.I_x = I_x
        self.m = m

    def altitude_controller(self,
                    z_target,
                    z_actual,
                    z_dot_target,
                    z_dot_actual,
                    z_dot_dot_target,
                    phi_actual):

        z_err = z_target - z_actual
        z_err_dot = z_dot_target - z_dot_actual

        p_term = self.z_k_p * z_err
        d_term = self.z_k_d * z_err_dot

        u_1_bar = p_term + d_term + z_dot_dot_target

        u_1 = self.m* (self.g - u_1_bar)/ math.cos(phi_actual)

        return u_1


    def lateral_controller(self,
                        y_target,
                        y_actual,
                        y_dot_target,
                        y_dot_actual,
                        u_1,
                        y_dot_dot_ff=0.0,
                        ):
        y_err = y_target - y_actual
        y_err_dot = y_dot_target - y_dot_actual

        p_term = self.y_k_p * y_err
        d_term = self.y_k_d * y_err_dot
        y_dot_dot_target = p_term + d_term + y_dot_dot_ff

        x = self.m * y_dot_dot_target / u_1
        x = min(0.99,max(x,-0.99))
        phi_commanded = math.asin( x )

        return phi_commanded


    def attitude_controller(self,
                        phi_target,
                        phi_actual,
                        phi_dot_actual,
                        phi_dot_target=0.0
                        ):
        phi_err = phi_target - phi_actual
        phi_err_dot = phi_dot_target - phi_dot_actual

        p_term = self.phi_k_p * phi_err
        d_term = self.phi_k_d * phi_err_dot

        u_2 = (p_term + d_term ) * self.I_x

        return u_2