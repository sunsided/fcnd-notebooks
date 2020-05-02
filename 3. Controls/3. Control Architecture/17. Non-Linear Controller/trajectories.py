import numpy as np


def figure_8(omega_z, duration, a_z=1.0, a_y=1.0, dt=0.01):
	omega_y = omega_z / 2
	t=np.linspace(0.0,duration,int(duration/dt))

	# desired path over time
	z = a_z * np.sin(omega_z * t)
	z_d = a_z * omega_z * np.cos(omega_z * t)
	z_dd = -a_z * omega_z**2 * np.sin(omega_z * t)


	# desired path over time
	y = a_y * np.cos(omega_y * t)
	y_d= -a_y * omega_y * np.sin(omega_y * t)
	y_dd= -a_y * omega_y**2 * np.cos(omega_y * t)

	return (z, z_d, z_dd) , (y, y_d, y_dd), t