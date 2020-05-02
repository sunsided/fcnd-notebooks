class OpenLoopController:

    def __init__(self, vehicle_mass, initial_state, mass_error=1.0):
        self.vehicle_mass  = vehicle_mass * mass_error
        self.vehicle_state = initial_state
        self.g = 9.81

    def thrust_control(self, target_z, dt):
        """
        Returns a thrust which will be commanded to
        the vehicle. This thrust should cause the vehicle
        to be at target_z in dt seconds.

        The controller's internal model of the vehicle_state
        is also updated in this method.
        """
        # 1. find target velocity needed to get to target_z
        current_z, current_z_dot = self.vehicle_state
        delta_z = target_z - current_z
        target_z_dot = delta_z / dt

        # 2. find target acceleration needed
        delta_z_dot = target_z_dot - current_z_dot
        target_z_dot_dot = delta_z_dot / dt

        # 3. find target NET force
        target_f_net = target_z_dot_dot * self.vehicle_mass

        # 4. find target thrust. Recall this equation:
        #    F_net = mg - thrust
        thrust = self.vehicle_mass * self.g - target_f_net

        # 5. update controller's internal belief of state
        self.vehicle_state += np.array([delta_z, delta_z_dot])

        return thrust



class PController:

    def __init__(self, k_p, m):
        self.k_p = k_p
        self.vehicle_mass = m
        self.g = 9.81

    def thrust_control(self, z_target, z_actual):
        # TODO - implement this method!

        err = z_target - z_actual

        # u_bar is what we want vertical acceleration to be
        u_bar = self.k_p * err

        # u is the thrust command which will cause u_bar
        u = self.vehicle_mass * (self.g - u_bar)

        return u



class PDController:

    def __init__(self, k_p, k_d, m):
        self.k_p = k_p
        self.k_d = k_d
        self.vehicle_mass = m
        self.g = 9.81

    def thrust_control(self,
                z_target,
                z_actual,
                z_dot_target,
                z_dot_actual,
                z_dot_dot_ff=0.0):
        err = z_target - z_actual
        err_dot = z_dot_target - z_dot_actual
        u_bar = self.k_p * err + self.k_d * err_dot + z_dot_dot_ff
        u = self.vehicle_mass * (self.g - u_bar)
        return u


class PIDController:

    def __init__(self, k_p, k_d, k_i, m):
        self.k_p = k_p
        self.k_d = k_d
        self.k_i = k_i
        self.vehicle_mass = m
        self.g = 9.81
        self.integrated_error = 0.0

    def thrust_control(self,
                z_target,
                z_actual,
                z_dot_target,
                z_dot_actual,
                dt,
                z_dot_dot_ff=0.0):

        err = z_target - z_actual
        err_dot = z_dot_target - z_dot_actual
        self.integrated_error += err * dt

        p = self.k_p * err
        i = self.integrated_error * self.k_i
        d = self.k_d * err_dot

        u_bar = p + i + d + z_dot_dot_ff
        u = self.vehicle_mass * (self.g - u_bar)
        return u
