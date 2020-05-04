
import numpy as np 

def flight_path():
    # Flight path is 
    total_time = 100.0
    dt = 0.01
    omega_z = 0.2
    omega_y = 0.1
    a_z = 1.0
    a_y = 1.0 

    t=np.linspace(0.0,total_time,int(total_time/dt))

    # z path over time 
    z_path = a_z * np.sin(omega_z * t)
    # z velocity over time
    z_dot_path = a_z * omega_z * np.cos(omega_z * t)
    # z acceleration over time 
    z_dot_dot_path= -a_z * omega_z**2 * np.sin(omega_z * t)


    # y path over time 
    y_path = a_y * np.cos(omega_y * t)
    # y velocity over time
    y_dot_path = -a_y * omega_y * np.sin(omega_y * t)
    # y acceleration over time 
    y_dot_dot_path= -a_y * omega_y**2 * np.cos(omega_y * t)

    # the roll angle over time
    phi_path = np.arctan2(y_dot_path,z_dot_path)
    # the angular velocity over time
    phi_dot_path = np.zeros(phi_path.shape)
    phi_dot_path[0:-1] = np.diff(phi_path)/dt
    phi_dot_path[-1] = phi_dot_path[-2]

    return t, dt, z_path, z_dot_path, z_dot_dot_path, y_path, y_dot_path, y_dot_dot_path, phi_path, phi_dot_path