from controllers import PController, PDController, PIDController


def close_enough_floats(f1, f2):
    return abs(f1 - f2) < 0.0001

def pct_diff(f,f_expected):
    return 100.0 * (f-f_expected) / f_expected


def p_controller_test(StudentPC):
    k_p = 1.5
    m = 2.5
    z_target = 2.0
    z_actual = 3.2

    pc = PController(k_p,m)
    spc = StudentPC(k_p,m)

    thrust =   pc.thrust_control(z_target, z_actual)
    s_thrust = spc.thrust_control(z_target, z_actual)

    if close_enough_floats(thrust, s_thrust):
        print("Tests pass")
    else:
        print("Tests fail. Off by %3.3f percent"% pct_diff(thrust, s_thrust))

def pd_controller_test(StudentPDC, feed_forward=False):
    k_p = 1.5
    k_d = 2.0
    m = 2.5
    z_target = 2.0
    z_actual = 3.2
    z_dot_target = -2.8
    z_dot_actual = -2.7
    ff = 1.1

    controller = PDController(k_p,k_d,m)
    scontroller = StudentPDC(k_p,k_d,m)

    if feed_forward:
        thrust =   controller.thrust_control(z_target,
                                             z_actual,
                                            z_dot_target,
                                            z_dot_actual,
                                            ff)
        s_thrust = scontroller.thrust_control(z_target,
                                             z_actual,
                                            z_dot_target,
                                            z_dot_actual,
                                            ff)
    else:
        thrust =   controller.thrust_control(z_target,
                                             z_actual,
                                            z_dot_target,
                                            z_dot_actual)
        s_thrust = scontroller.thrust_control(z_target,
                                             z_actual,
                                            z_dot_target,
                                            z_dot_actual)

    if close_enough_floats(thrust, s_thrust):
        print("Tests pass")
    else:
        print("Tests fail. Off by %3.3f percent"% pct_diff(thrust, s_thrust))

def pid_controller_test(StudentPIDC):
    k_p = 1.5
    k_d = 2.0
    k_i = 1.2
    m = 2.5
    z_target = 2.0
    z_actual = 3.2
    z_dot_target = -2.8
    z_dot_actual = -2.7
    dt = 0.1

    controller = PIDController(k_p,k_d,k_i,m)
    scontroller = StudentPIDC(k_p,k_d,k_i,m)

    for _ in range(3):
        thrust =   controller.thrust_control(z_target,
                                             z_actual,
                                            z_dot_target,
                                            z_dot_actual,
                                            dt)
        s_thrust = scontroller.thrust_control(z_target,
                                             z_actual,
                                            z_dot_target,
                                            z_dot_actual,
                                            dt)

    if close_enough_floats(thrust, s_thrust):
        print("Tests pass")
    else:
        print("Tests fail. Off by %3.3f percent"% pct_diff(thrust, s_thrust))