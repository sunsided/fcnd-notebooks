
import math

class Answers:

    def angular_velocities(m, g, i_z, k_f, k_m, a_z, alpha, student_answer_omega_1, student_answer_omega_2) :
        

        term_1 = m * (-a_z + g) /(2 * k_f)
        term_2 = i_z * a_z/(2 * k_m)
        
        omega_1 = - math.sqrt(term_1 + term_2)
        omega_2 =   math.sqrt(term_1 - term_2)
        
        epsilon = 10**(-4)

        if abs(omega_1 - student_answer_omega_1)< epsilon and abs(omega_2 - student_answer_omega_2)< epsilon :
            return 'You calculated correct values'

        elif abs(omega_1 - student_answer_omega_1)< epsilon and abs(omega_2 + student_answer_omega_2)< epsilon:
            return 'Check the sign of the second propeller angular velocity'

        elif abs(omega_1 + student_answer_omega_1)< epsilon and abs(omega_2 - student_answer_omega_2)< epsilon:
            return 'Check the sign of the first propeller angular velocity'

        elif abs(omega_1 + student_answer_omega_1)< epsilon and abs(omega_2 - student_answer_omega_2)< epsilon:
            return 'Check the signs of the propeller angular velocities'

        else:
            return 'Your answer does not coincide with our answer'



    def angular_acceleration(i_z, k_m, omega_1, omega_2, student_answer_alpha):

        cw_torque = k_m * omega_1 **2
        ccw_torque = k_m * omega_2 **2
        
        net_torque = cw_torque - ccw_torque
        angular_acc = net_torque / i_z

        epsilon = 10**(-4)

        if abs(angular_acc - student_answer_alpha) < epsilon:
            return 'You calculated the angular acceleration correctly'

        elif abs(angular_acc + student_answer_alpha) < epsilon:
            return 'Make sure that your sign is correct'

        else:
            return 'Your answer does not coincide with our answer'

        




    def linear_acceleration( m, g, k_f, omega_1, omega_2, student_answer_a_z ):
        f_1 = k_f * omega_1**2
        f_2 = k_f * omega_2**2
        f_g = m * g
        f_total = -f_1 - f_2 + f_g
        
        acceleration = f_total / m 

        epsilon = 10**(-4)

        if abs(acceleration - student_answer_a_z) < epsilon:
            return 'You calculated the acceleration value correctly'

        elif abs(acceleration + student_answer_a_z) < epsilon:
            return 'Make sure that you are using proper sign as z is directed downward'

        else:
            return 'Your answer does not coincide with ours'


    def vertical_acceleration(m, k_f, g ,phi, omega_1, omega_2, student_answer_a_z):

        f_1 = k_f * omega_1**2
        f_2 = k_f * omega_2**2
        
        thrust = f_1 + f_2
        
        a_z = g - thrust * math.cos(phi) / m

        epsilon=10**(-4)

        if abs(a_z-student_answer_a_z) < epsilon:
            return 'You calculated the vertical acceleration value correctly'

        elif abs(a_z+student_answer_a_z) < epsilon:
            return 'Make sure that you are using proper sign as z is directed downward'

        else:
            return 'Your answer does not coincide with ours'
        



    def rolling_acceleration(k_f, i, l, omega_1, omega_2, student_answer_phi_dot_dot):
        f_1 = k_f * omega_1**2
        f_2 = k_f * omega_2**2
        
        torque = (f_1 - f_2)* l 
        
        angular_acc = torque / i
        epsilon=10**(-4)

        if abs(angular_acc-student_answer_phi_dot_dot) < epsilon:
            return 'You calculated the vertical acceleration value correctly'

        elif abs(angular_acc+student_answer_phi_dot_dot) < epsilon:
            return 'Make sure that you are using proper sign as positive direction is clockwise in this case'

        else:
            return 'Your answer does not coincide with ours'

        



    def horizontal_acceleration(m, k_f, g, phi, omega_1, omega_2, student_answer_a_y ):
        f_1 = k_f * omega_1**2
        f_2 = k_f * omega_2**2
        
        thrust = f_1 + f_2
        
        a_y = thrust * math.sin(phi) / m 

        epsilon=10**(-4)

        if abs(a_y-student_answer_a_y) < epsilon:
            return 'You calculated the vertical acceleration value correctly'

        elif abs(a_y+student_answer_a_y) < epsilon:
            return 'Make sure that you are using proper sign as y is directed right'

        else:
            return 'Your answer does not coincide with ours'

        
        

