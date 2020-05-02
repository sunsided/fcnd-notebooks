import numpy as np


def cosine(amp, period, num_periods=3, dt=0.01, duration=None):
    omega = 2 * np.pi / period
    total_time = period * num_periods
    if duration:
        total_time = duration
    t=np.linspace(0.0,total_time,1000)
    z_path =          amp * (np.cos(omega *t) - 1)
    z_dot_path =     -amp * omega *  np.sin(omega*t)
    z_dot_dot_path = -amp * omega * omega *np.cos(omega*t)
    return t, z_path, z_dot_path, z_dot_dot_path

def step(to=-1.0, duration=10.0):
    t = np.linspace(0.0, duration, 1000)
    z = to * np.ones(t.shape[0])
    z_dot = np.zeros(t.shape[0])
    return t,z,z_dot