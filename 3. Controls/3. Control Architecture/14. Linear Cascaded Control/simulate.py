from drone import Drone2D
import math
import numpy as np


def zy_flight(z_traj,
	y_traj,
	t,
	controller,
	inner_loop_speed_up=10):

	dt = t[1] - t[0]

	z_path, z_dot_path, z_dot_dot_path = z_traj
	y_path, y_dot_path, y_dot_dot_path = y_traj
	drone = Drone2D()

	# drone.X = np.array([z_path[0],
	# 					y_path[0],
	# 					math.atan2(y_dot_path[0],z_dot_path[0]),
	# 					z_dot_path[0],
	# 					y_dot_path[0],
	# (math.atan2(y_dot_path[1],z_dot_path[1])-math.atan2(y_dot_path[0],z_dot_path[0]))/dt])

	drone.X = np.array([z_path[0],
						y_path[0],
						math.atan2(y_dot_path[0],z_dot_path[0]),
						z_dot_path[0],
						y_dot_path[0],
						0])

	# array for recording the state history
	linear_drone_state_history = drone.X

	# executing the flight
	for i in range(0,z_path.shape[0]-1):
		u_1 = controller.altitude_controller(z_path[i],
											   drone.X[0],
											   z_dot_path[i],
											   drone.X[3],
											   z_dot_dot_path[i],
											   drone.X[2])
		phi_commanded = controller.lateral_controller(y_path[i],
												drone.X[1],
												y_dot_path[i],
												drone.X[4],
												u_1,
												y_dot_dot_path[i])
		for _ in range(inner_loop_speed_up):
			u_2 = controller.attitude_controller(phi_commanded,
													 drone.X[2],
													 drone.X[5],
													 0.0
													 )

			# calculating the new state vector
			drone.set_controls(u_1, u_2)
			drone_state = drone.advance_state(dt/inner_loop_speed_up)

		# generating a history of vertical positions for the drone
		linear_drone_state_history = np.vstack((linear_drone_state_history, drone_state))
	return linear_drone_state_history



