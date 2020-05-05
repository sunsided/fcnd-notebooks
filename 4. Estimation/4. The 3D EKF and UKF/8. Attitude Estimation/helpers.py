import numpy as np
from matplotlib import pyplot as plt

class IMU:
    def __init__(self, accel_sigma=0.1, gyro_sigma=0.5):
        self.accel_sigma = accel_sigma
        self.gyro_sigma  = gyro_sigma
        
    def measure(self, true_state):
        """true_state is [theta, phi, q, p]"""
        theta = true_state[0] + np.random.normal(0, self.accel_sigma)
        phi   = true_state[1] + np.random.normal(0, self.accel_sigma) 
        q     = true_state[2] + np.random.normal(0, self.gyro_sigma)
        p     = true_state[3] + np.random.normal(0, self.gyro_sigma)
        
        return np.array([theta,phi,q,p])
    
    def make_measurements(self, true_states):
        measurements = np.zeros((4, true_states.shape[1]))
        for i in range(true_states.shape[1]):
            measurements[:,i] = self.measure(true_states[:,i])
        return measurements
    
def plot_compare(truth, estimates, measurements, integrated, dt,index):
    N = truth.shape[1]
    t = np.arange(0,N*dt, dt)
    plt.plot(t, truth.T[:,index],linestyle='-', color='black',lw=4)
    plt.plot(t, measurements.T[:,index], linestyle='-', color='red',alpha=0.5)
    plt.plot(t,estimates.T[:,index],linestyle='-',color='blue')
    plt.plot(t,integrated.T[:,index],linestyle='-',color='green')
    plt.grid()
    if index == 0:
        title = "Pitch"
    else:
        title = "Roll"
    plt.title(title).set_fontsize(20)
    plt.xlabel('$t$ [sec]').set_fontsize(20)
    xlab = '$\Theta - \Theta_0$ [$^{\circ}$]'
    if index == 1:
        xlab = '$\Phi - \Phi$ [$^{\circ}$]'
    plt.ylabel(xlab).set_fontsize(20)
    plt.xticks(fontsize = 14)
    plt.yticks(fontsize = 14)
    plt.legend(['True', 'Measured','Estimated','Gyro integral'],fontsize = 18)
    fig =plt.gcf()
    fig.set_size_inches(10, 10)
    plt.show()