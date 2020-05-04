import numpy as np

def flight_path(total_time,dt,type='constant'):

    if type == 'constant':

        t=np.linspace(0.0,total_time,int(total_time/dt)+1)
        z_path= -np.ones(t.shape[0])

        z_dot_path = np.zeros(t.shape[0])

        # desired acceleration over time in order to execute the given flight path
        z_dot_dot_path= np.zeros(t.shape[0])


    if type == 'periodic':
        t=np.linspace(0.0,total_time,int(total_time/dt)+1)
        z_path= 0.5*np.cos(2*t)-0.5

        z_dot_path= -1*np.sin(2*t) 

        # desired acceleration over time in order to execute the given flight path
        z_dot_dot_path= -2*np.cos(2*t)



    return t, z_path, z_dot_path, z_dot_dot_path

