import numpy as np 

def flight_path():
    total_time = 20.0
    dt = 0.01
    g=9.81

    t=np.linspace(0.0,total_time,int(total_time/dt))

    omega_x = 0.8 # 0.8
    omega_y = 0.4 # 0.4
    omega_z = 1.6 # 0.4

    a_x = 1.0 
    a_y = 1.0
    a_z = 1.0

    x_path= a_x * np.sin(omega_x * t) 
    x_dot_path= a_x * omega_x * np.cos(omega_x * t)
    x_dot_dot_path= -a_x * omega_x**2 * np.sin(omega_x * t)

    y_path= a_y * np.cos(omega_y * t)
    y_dot_path= -a_y * omega_y * np.sin(omega_y * t)
    y_dot_dot_path= -a_y * omega_y**2 * np.cos(omega_y * t)

    z_path= a_z * np.cos(omega_z * t)
    z_dot_path= -a_z * omega_z * np.sin(omega_z * t)
    z_dot_dot_path= - a_z * omega_z**2 * np.cos(omega_z * t)


    
    theta_path = np.zeros(z_path.shape)
    #theta_path = np.arctan(x_dot_dot_path/(z_dot_dot_path-g))
    theta_dot_path = np.zeros(theta_path.shape)
    theta_dot_path[0:-1] = np.diff(theta_path)/dt
    theta_dot_path[-1] = theta_dot_path[-2]

    phi_path = np.zeros(z_path.shape)
    #phi_path = np.arctan(y_dot_dot_path/(x_dot_path**2+(z_dot_dot_path-g)**2)**(0.5))
    phi_dot_path = np.zeros(phi_path.shape)
    phi_dot_path[0:-1] = np.diff(phi_path)/dt
    phi_dot_path[-1] = phi_dot_path[-2]


    x = x_path
    x_dot = x_dot_path
    x_dot_dot = x_dot_dot_path

    y = y_path
    y_dot = y_dot_path
    y_dot_dot = y_dot_dot_path

    z = z_path
    z_dot = z_dot_path
    z_dot_dot = z_dot_dot_path

    phi = phi_path
    phi_dot= phi_dot_path

    theta = theta_path
    theta_dot = theta_dot_path


    return t, dt, x, x_dot, x_dot_dot, y, y_dot, y_dot_dot, z, z_dot, z_dot_dot, phi, phi_dot, theta, theta_dot