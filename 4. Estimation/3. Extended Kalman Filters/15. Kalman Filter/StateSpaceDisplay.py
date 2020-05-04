import numpy as np 
import math
import matplotlib.pyplot as plt
import matplotlib.pylab as pylab
from scipy.stats import multivariate_normal

def state_space_display(z,v,mu_0,sigma_0,mu_bar,sigma_bar):
    y_axis = np.array([v-1,v+1])
    x_axis = np.array([z-2,z+2])

    delta = 0.05

    x, y = np.mgrid[x_axis[0]-1:x_axis[1]+1:delta, y_axis[0]-1:y_axis[1]+1:delta]
    configuration_space = np.empty(x.shape + (2,))
    configuration_space[:, :, 0] = x; configuration_space[:, :, 1] = y

    sigma_b_flipped = np.array([[sigma_bar[1,1], sigma_bar[0,1]], [sigma_bar[1,0], sigma_bar[0,0]]])
    predicted_state_space = multivariate_normal([mu_bar[1,0],mu_bar[0,0]], sigma_b_flipped)

    # levels at 1, 2 and 3 sigmas 
    predicted_levels = np.array([np.exp(-4.5),np.exp(-2),np.exp(-0.5)])*predicted_state_space.pdf([mu_bar[1,0],mu_bar[0,0]])
    plt.contour(x, y, predicted_state_space.pdf(configuration_space),predicted_levels,colors='Red')

    sigma_0_flipped = np.array([[sigma_0[1,1], sigma_0[0,1]], [sigma_0[1,0], sigma_0[0,0]]])
    initial_state_space=multivariate_normal([mu_0[1,0],mu_0[0,0]], sigma_0_flipped)
    initial_levels = np.array([np.exp(-4.5),np.exp(-2),np.exp(-0.5)])*initial_state_space.pdf([z,v])
    plt.contour(x, y, initial_state_space.pdf(configuration_space),initial_levels,colors='Green')


    plt.scatter(z,v,color='Green')
    plt.scatter(mu_bar[1,0],mu_bar[0,0],color='Red')
    

    # generating the arrow indicating the direction
    arrow_head_length=0.1
    angle = np.arctan2(mu_bar[0]-v,mu_bar[1]-z)

    arrow_length_z = float(mu_bar[1,0]-z-arrow_head_length*np.cos(angle))
    arrow_length_y = float(mu_bar[0,0]-v-arrow_head_length*np.sin(angle))

    plt.arrow(z, v,
              arrow_length_z,
              arrow_length_y, 
              head_width=0.05, head_length=arrow_head_length, fc='k', ec='k')

    plt.grid()
    plt.title('State Space ').set_fontsize(20)
    plt.xlabel('$z$ [m]').set_fontsize(20)
    plt.ylabel('$\dot{z}$[$m/s$]').set_fontsize(20)
    plt.legend(['Initial','Predicted'],fontsize = 14)
    plt.xticks(fontsize = 14)
    plt.yticks(fontsize = 14)
    plt.axis('equal')
    plt.show()


def state_space_display_updated(z,v,mu_0,sigma_0,mu_bar,sigma_bar,mu_updated,sigma_updated):
    y_axis = np.array([v-1,v+1])
    x_axis = np.array([z-2,z+2])

    delta = 0.05

    x, y = np.mgrid[x_axis[0]-1:x_axis[1]+1:delta, y_axis[0]-1:y_axis[1]+1:delta]
    configuration_space = np.empty(x.shape + (2,))
    configuration_space[:, :, 0] = x; configuration_space[:, :, 1] = y

    sigma_b_flipped = np.array([[sigma_bar[1,1], sigma_bar[0,1]], [sigma_bar[1,0], sigma_bar[0,0]]])
    predicted_state_space = multivariate_normal([mu_bar[1,0],mu_bar[0,0]], sigma_b_flipped)
    # levels at 1, 2 and 3 sigmas 
    predicted_levels = np.array([np.exp(-4.5),np.exp(-2),np.exp(-0.5)]) * predicted_state_space.pdf([mu_bar[1,0],mu_bar[0,0]])
    plt.contour(x, y, predicted_state_space.pdf(configuration_space),predicted_levels,colors='Red')

    sigma_0_flipped = np.array([[sigma_0[1,1], sigma_0[0,1]], [sigma_0[1,0], sigma_0[0,0]]])
    initial_state_space = multivariate_normal([mu_0[1,0],mu_0[0,0]], sigma_0_flipped)
    initial_levels = np.array([np.exp(-4.5),np.exp(-2),np.exp(-0.5)]) * initial_state_space.pdf([z,v])
    plt.contour(x, y, initial_state_space.pdf(configuration_space),initial_levels,colors='Green')

    sigma_u_flipped = np.array([[sigma_updated[1,1], sigma_updated[0,1]], [sigma_updated[1,0], sigma_updated[0,0]]])
    updated_state_space = multivariate_normal([mu_updated[1,0],mu_updated[0,0]], sigma_u_flipped)
    updated_levels = np.array([np.exp(-4.5),np.exp(-2),np.exp(-0.5)]) * updated_state_space.pdf([mu_updated[1,0],mu_updated[0,0]])
    plt.contour(x, y, updated_state_space.pdf(configuration_space),updated_levels,colors='Black')

    plt.scatter(z,v,color='Green')
    plt.scatter(mu_bar[1,0],mu_bar[0,0],color='Red')
    plt.scatter(mu_updated[1,0],mu_updated[0,0],color='Black')

    # generating the arrow indicating the direction
    arrow_head_length=0.1
    angle = np.arctan2(mu_bar[0]-v,mu_bar[1]-z)

    arrow_length_z = float(mu_bar[1,0]-z-arrow_head_length*np.cos(angle))
    arrow_length_y = float(mu_bar[0,0]-v-arrow_head_length*np.sin(angle))

    plt.arrow(z, v,
              arrow_length_z,
              arrow_length_y, 
              head_width=0.05, head_length=arrow_head_length, fc='k', ec='k')


    plt.grid()
    plt.title('State Space').set_fontsize(20)
    plt.xlabel('$z$ [m]').set_fontsize(20)
    plt.ylabel('$\dot{z}$[$m/s$]').set_fontsize(20)
    plt.xticks(fontsize = 14)
    plt.yticks(fontsize = 14)
    plt.legend(['Initial','Predicted','Updated'],fontsize = 14)
    plt.axis('equal')
    plt.show()