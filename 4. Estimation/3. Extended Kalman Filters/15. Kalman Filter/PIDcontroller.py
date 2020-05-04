import numpy as np 
import math

class PIDController_with_ff:
    
    def __init__(self,
                 k_p,
                 k_d,
                 k_i
                ):
        
        self.k_p = k_p
        self.k_d = k_d
        self.k_i = k_i
        self.err_int=0.0
    
    def control(self,
                z_target, 
                z_actual, 
                z_dot_target, 
                z_dot_actual,
                z_dot_dot,
                dt
                ):
        
        err = z_target - z_actual
        err_dot = z_dot_target - z_dot_actual
        self.err_int += err * dt
        
        p_term_thrust = self.k_p * err
        d_term_thrust = self.k_d * err_dot
        i_term_thrust = self.k_i * self.err_int 
        
        u_bar = p_term_thrust + d_term_thrust + i_term_thrust + z_dot_dot
        
        return u_bar