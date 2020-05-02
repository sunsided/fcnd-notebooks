from solution import UDACITYDroneIn3D, UDACITYController
import numpy as np
import random


def get_controller(controller_class):
	return controller_class(
		z_k_p=1.0,
	    z_k_d=1.1,
	    x_k_p=1.2,
	    x_k_d=1.3,
	    y_k_p=1.4,
	    y_k_d=1.5,
	    k_p_roll=1.6,
	    k_p_pitch=1.7,
	    k_p_yaw=1.8,
	    k_p_p=1.9,
	    k_p_q=2.0,
	    k_p_r=2.1
		)

def random_vals(N):
	return [random.random() for _ in range(N)]

def R():
	drone = UDACITYDroneIn3D()
	drone.X[3] = 0.1
	drone.X[4] = 0.2
	drone.X[5] = 0.3
	return drone.R()

def close_enough(v1, v2):
	return abs(v1 - v2) < 0.00001

def test_exercise_1_1(drone_class):
	student = drone_class()
	solution = UDACITYDroneIn3D()

	for drone in [student, solution]:
		drone.set_propeller_angular_velocities(-5.0, 1.0, -2.0, 0.3)

	for om, om_sol in zip(student.omega, solution.omega):
		if not close_enough(om, om_sol):
			print("Error in exercise 1.1")
			return
	print("Tests pass - exercise 1.1")

def test_exercise_1_2(drone_class):
	student = drone_class()
	solution = UDACITYDroneIn3D()

	for drone in [student, solution]:
		drone.X[3] = 0.1
		drone.X[4] = 0.2
		drone.X[5] = 0.3

	r_stu = student.R()
	r_sol = solution.R()

	for r1, r2 in zip(r_stu, r_sol):
		for v1, v2 in zip(r1, r2):
			if not close_enough(v1,v2):
				print("Error in exercise 1.2")
				return
	print("Tests pass - exercise 1.2")

def test_exercise_1_3(drone_class):
	student = drone_class()
	solution = UDACITYDroneIn3D()

	for drone in [student, solution]:
		drone.X[3] = 0.1
		drone.X[4] = 0.2
		drone.X[5] = 0.3
		drone.X[9] = 0.05
		drone.X[10] = 0.12
		drone.X[11] = 0.23
		drone.set_propeller_angular_velocities(-5.0, 1.0, -2.0, 0.3)

	acc_stu = student.linear_acceleration()
	acc_sol = solution.linear_acceleration()

	for v1, v2 in zip(acc_stu, acc_sol):
		if not close_enough(v1,v2):
			print("Error in exercise 1.3")
			return
	print("Tests pass - exercise 1.3")


def test_exercise_2_1(drone_class):
	student = drone_class()
	solution = UDACITYDroneIn3D()

	for drone in [student, solution]:
		drone.X[3] = 0.1
		drone.X[4] = 0.2
		drone.X[5] = 0.3
		drone.X[9] = 0.05
		drone.X[10] = 0.12
		drone.X[11] = 0.23
		drone.set_propeller_angular_velocities(-5.0, 1.0, -2.0, 0.3)

	OD_stu = student.get_omega_dot()
	OD_sol = solution.get_omega_dot()
	for v1, v2 in zip(OD_stu, OD_sol):
		if not close_enough(v1,v2):
			print("Error in exercise 2.1")
			return
	print("Tests pass - exercise 2.1")

def test_exercise_3_1(drone_class):
	student = drone_class()
	solution = UDACITYDroneIn3D()

	for drone in [student, solution]:
		drone.X[3] = 0.1
		drone.X[4] = 0.2
		drone.X[5] = 0.3
		drone.X[9] = 0.05
		drone.X[10] = 0.12
		drone.X[11] = 0.23
		drone.set_propeller_angular_velocities(-5.0, 1.0, -2.0, 0.3)
	ED_stu = student.get_euler_derivatives()
	ED_sol = solution.get_euler_derivatives()
	for v1, v2 in zip(ED_stu, ED_sol):
		if not close_enough(v1,v2):
			print("Error in exercise 3.1")
			return
	print("Tests pass - exercise 3.1")

def test_exercise_3_2(drone_class):
	student = drone_class()
	solution = UDACITYDroneIn3D()

	for drone in [student, solution]:
		drone.X[3] = 0.1
		drone.X[4] = 0.2
		drone.X[5] = 0.3
		drone.X[6] = -2.3
		drone.X[7] = 1.2
		drone.X[8] = 0.5
		drone.X[9] = 0.05
		drone.X[10] = 0.12
		drone.X[11] = 0.23
		drone.set_propeller_angular_velocities(-5.0, 1.0, -2.0, 0.3)
		drone.advance_state(0.1)

	for v1, v2 in zip(student.X, solution.X):
		if not close_enough(v1,v2):
			print("Error in exercise 3.2")
			return
	print("Tests pass - exercise 3.2")

def test_exercise_4_1(controller_class):
	student = get_controller(controller_class)
	solution = get_controller(UDACITYController)
	args = random_vals(11)
	stu_cmds = student.lateral_controller(*args)
	sol_cmds = solution.lateral_controller(*args)
	for v1, v2 in zip(stu_cmds, sol_cmds):
		if not close_enough(v1,v2):
			print("Error in exercise 4.1")
			return
	print("Tests pass - exercise 4.1")

def test_exercise_4_2(controller_class):
	student = get_controller(controller_class)
	solution = get_controller(UDACITYController)
	args = random_vals(2)
	r = R()

	stu_cmds = student.roll_pitch_controller(*args, r)
	sol_cmds = solution.roll_pitch_controller(*args, r)
	for v1, v2 in zip(stu_cmds, sol_cmds):
		if not close_enough(v1,v2):
			print("Error in exercise 4.2")
			return
	print("Tests pass - exercise 4.2")

def test_exercise_5_1(controller_class):
	student = get_controller(controller_class)
	solution = get_controller(UDACITYController)
	args = random_vals(6)
	stu_cmds = student.body_rate_controller(*args)
	sol_cmds = solution.body_rate_controller(*args)
	for v1, v2 in zip(stu_cmds, sol_cmds):
		if not close_enough(v1,v2):
			print("Error in exercise 5.1")
			return
	print("Tests pass - exercise 5.1")

def test_exercise_5_2(controller_class):
	student = get_controller(controller_class)
	solution = get_controller(UDACITYController)
	args = random_vals(2)
	v1 = student.yaw_controller(*args)
	v2 = solution.yaw_controller(*args)

	if not close_enough(v1,v2):
		print("Error in exercise 5.2")
		return
	print("Tests pass - exercise 5.2")


def test_exercise_5_3(controller_class):
	student = get_controller(controller_class)
	solution = get_controller(UDACITYController)
	args = random_vals(5)
	r = R()

	v1 = student.altitude_controller(*args, r)
	v2 = solution.altitude_controller(*args, r)

	if not close_enough(v1,v2):
		print("Error in exercise 5.3")
		return
	print("Tests pass - exercise 5.3")