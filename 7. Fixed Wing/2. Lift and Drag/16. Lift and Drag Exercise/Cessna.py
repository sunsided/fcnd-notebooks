import numpy as np

class AeroDynamics():

    def __init__(self):
        '''
        Importing the airplane model coefficients from the other object.
        '''

        self.rho = 1.2682               # [kg/m^3] density of air.
        self.g = 9.81                   # [m/s^2] gravitational acceleration

        self.s = 16.1651                #[OK]   # [m^2] aircraft's wing area
        self.c = 1.49352                #[OK]   # [m] the mean aerodynamic chord
        self.b = 10.9728                #[OK]   # [m]  # wingspan
        self.mass = 1202.02             #[OK]   # [kg] mass of airplane


        self.c_l_0  = 0.307             #[OK]   # non-dimensional coefficient of lift at zero angle of attack
        self.c_l_alpha = 4.41           #[OK] # non-dimensional lift slope
        self.c_l_delta_e = 0.43         #[OK] # non-dimensional lift control derivative regarding elevator angle
        self.c_l_alpha_2 = 0.0          # the non-dimensional lift coefficient relative to the square of the angle of attack


        # drag coefficients
        self.c_d_0 = 0.0270             #[OK]  # non-dimensional coefficient of drag at zero angle of attack
        self.epsilon = 0.1592           # induced drag factor

        # pitch moment
        self.c_m_0 =  0.04              #[OK] # non-dimensional; coefficient of pitching moment at zero angle of attack
        self.c_m_alpha = -0.613         #[OK] # non-dimensional pitching slope
        self.c_m_delta_e = -1.122       #[OK]  # non-dimensional pitching slope for elevator


        self.c_d_alpha = 0.121          #[OK]
        self.c_l_q = 3.9               #[OK]
        self.c_d_q = 0.0
        self.c_m_q = -12.4             #[OK] # pitch damping derivative
        self.c_d_delta_e = 0.0

        # propeller data            # Propeller data are from UAV
        self.k_motor = 80
        self.s_prop = 0.2027 # [m^2]
        self.c_prop = 1.0
        self.k_Tp = 0.0
        self.k_omega = 0.0



        # Lateral coefficients
        self.c_y_0 = 0.0
        self.c_l_0 = 0.0
        self.c_n_0 = 0.0

        self.c_y_beta = -0.393   #[OK]
        self.c_l_beta = -0.0923  #[OK] # roll static stability derivative
        self.c_n_beta = 0.0587   #[OK] # yaw static stability derivative

        self.c_y_p = -0.075      #[OK]
        self.c_l_p = -0.484      #[OK]
        self.c_n_p = -0.0278     #[OK]

        self.c_y_r = 0.214       #[OK]
        self.c_l_r = 0.0798      #[OK]
        self.c_n_r = -0.0937     #[OK]

        self.c_y_delta_a = 0.0   #[OK]
        self.c_l_delta_a = 0.229 #[OK]  # primary control derivative
        self.c_n_delta_a = -0.0216#[OK]

        self.c_y_delta_r = 0.87    #[OK]
        self.c_l_delta_r = 0.0147  #[OK]
        self.c_n_delta_r = -0.0645 #[OK]  # primary control derivative

        self.j_x = 1285.3154166     #[OK]  # kg*m^2
        self.j_y = 1824.9309607     #[OK]   # kg*m^2
        self.j_z = 2666.89390765    #[OK]   # kg*m^2
        self.j_xz = 0.0             #[OK] # kg*m^2

        self.j = np.array([[self.j_x,   0,   -self.j_xz],
                           [0,     self.j_y, 0],
                           [-self.j_xz, 0,   self.j_z]])