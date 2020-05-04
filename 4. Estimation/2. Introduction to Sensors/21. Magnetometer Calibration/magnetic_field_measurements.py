import numpy as np 



def N(num):
    sigma = 0.1
    return np.random.normal(0.0, sigma,(2,num))


b = np.array([[1.3],[-0.3]])

alpha =-60
k1= 1.5
k2= 0.75
M  = np.array([[k1*np.cos(alpha/180*np.pi),-k2*np.sin(alpha/180*np.pi)],
               [k1*np.sin(alpha/180*np.pi),k2*np.cos(alpha/180*np.pi)]])

I = np.zeros((2,2))



def measure_old(true_val):
    return b + (I + M) @ true_val + N()
def measure(true_val):
    return b + np.matmul((I + M),true_val) + N(true_val.shape[1])

def measured_field():
    
    R = 1
    Sample_N =1000
    true_value = np.zeros((2,Sample_N))
    theta = np.linspace(0 ,2*np.pi,Sample_N)
    true_value[0,:] = R * np.sin(theta)
    true_value[1,:] = R * np.cos(theta)
    measured = measure(true_value)

    return true_value, measured