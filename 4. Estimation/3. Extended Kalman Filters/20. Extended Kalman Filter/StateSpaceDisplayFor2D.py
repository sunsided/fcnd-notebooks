import numpy as np 
import math
import matplotlib.pyplot as plt
import matplotlib.pylab as pylab
from scipy.stats import multivariate_normal

def state_space_display_predict(y,y_dot,mu_0,sigma_0,mu_bar,sigma_bar):

    y_axis = np.array([y_dot-1,y_dot+1])
    x_axis = np.array([y-2,y+2])

    delta= 0.05

    x_coor, y_coor = np.mgrid[x_axis[0]-1:x_axis[1]+1:delta, y_axis[0]-1:y_axis[1]+1:delta]
    configuration_space = np.empty(x_coor.shape + (2,))
    configuration_space[:, :, 0] = x_coor; configuration_space[:, :, 1] = y_coor

    
    predicted_state_space = multivariate_normal([mu_bar[2,0],mu_bar[1,0]], [[sigma_bar[2,2],0.0],[0.0,sigma_bar[1,1]]])

    # levels at 1,2 and 3 sigmas 
    predicted_levels= np.array([np.exp(-4.5),np.exp(-2),np.exp(-0.5)])*predicted_state_space.pdf([mu_bar[2,0],mu_bar[1,0]])
    plt.contour(x_coor, y_coor, predicted_state_space.pdf(configuration_space),predicted_levels,colors='Red')


    initial_state_space=multivariate_normal([y,y_dot], [[sigma_0[2],0.0],[0.0,sigma_0[1]]])
    initial_levels= np.array([np.exp(-4.5),np.exp(-2),np.exp(-0.5)])*initial_state_space.pdf([y,y_dot])
    plt.contour(x_coor, y_coor, initial_state_space.pdf(configuration_space),initial_levels,colors='Green')


    plt.scatter(y,y_dot,color='Green')
    plt.scatter(mu_bar[2,0],mu_bar[1,0],color='Red')
    

    # generating the arrow indicating the direction
    arrow_head_length=0.1
    angle = np.arctan2(mu_bar[1]-y_dot,mu_bar[2]-y)

    arrow_length_z = float(mu_bar[2,0]-y-arrow_head_length*np.cos(angle))
    arrow_length_y = float(mu_bar[1,0]-y_dot-arrow_head_length*np.sin(angle))

    plt.arrow(y, y_dot,
              arrow_length_z,
              arrow_length_y, 
              head_width=0.05, head_length=arrow_head_length, fc='k', ec='k')

    plt.grid()
    plt.title('State Space ').set_fontsize(20)
    plt.xlabel('$y$ [m]').set_fontsize(20)
    plt.ylabel('$\dot{y}$[$m/s$]').set_fontsize(20)
    plt.legend(['Initial','Predicted'],fontsize = 14)
    plt.xticks(fontsize = 14)
    plt.yticks(fontsize = 14)
    plt.axis('equal')
    plt.show()




def state_space_display_updated(y,y_dot,mu_0,sigma_0,mu_bar,sigma_bar,mu_updated,sigma_updated):
    y_axis = np.array([y_dot-1,y_dot+1])
    x_axis = np.array([y-2,y+2])

    delta= 0.05

    x_coor, y_coor = np.mgrid[x_axis[0]-1:x_axis[1]+1:delta, y_axis[0]-1:y_axis[1]+1:delta]
    configuration_space = np.empty(x_coor.shape + (2,))
    configuration_space[:, :, 0] = x_coor; configuration_space[:, :, 1] = y_coor

    predicted_state_space = multivariate_normal([mu_bar[2,0],mu_bar[1,0]], 
        [[sigma_bar[2,2],0.0],[0.0,sigma_bar[1,1]]])

    # levels at 1,2 and 3 sigmas 
    predicted_levels= np.array([np.exp(-4.5),np.exp(-2),np.exp(-0.5)]) *\
                      predicted_state_space.pdf([mu_bar[2,0],mu_bar[1,0]])

    plt.contour(x_coor, y_coor, predicted_state_space.pdf(configuration_space),predicted_levels,colors='Red')


    updated_state_space = multivariate_normal([mu_updated[2,0],mu_updated[1,0]], 
        [[sigma_updated[2,2],sigma_updated[2,1]],[sigma_updated[1,2],sigma_updated[1,1]]])

    updated_levels= np.array([np.exp(-4.5),np.exp(-2),np.exp(-0.5)]) *\
                     updated_state_space.pdf([mu_updated[2,0],mu_updated[1,0]])
    plt.contour(x_coor, y_coor, updated_state_space.pdf(configuration_space),updated_levels,colors='Black')

    initial_state_space=multivariate_normal([y,y_dot], [[sigma_0[2],0.0],[0.0,sigma_0[1]]])
    initial_levels= np.array([np.exp(-4.5),np.exp(-2),np.exp(-0.5)]) *\
                    initial_state_space.pdf([y,y_dot])
    plt.contour(x_coor, y_coor, initial_state_space.pdf(configuration_space),initial_levels,colors='Green')


    plt.scatter(y,y_dot,color='Green')
    plt.scatter(mu_bar[2,0],mu_bar[1,0],color='Red')
    plt.scatter(mu_updated[2,0],mu_updated[1,0],color='black')



    # generating the arrow indicating the direction
    arrow_head_length=0.1
    angle = np.arctan2(mu_bar[1]-y_dot,mu_bar[2]-y)

    arrow_length_z = float(mu_bar[2,0]-y-arrow_head_length*np.cos(angle))
    arrow_length_y = float(mu_bar[1,0]-y_dot-arrow_head_length*np.sin(angle))

    plt.arrow(y, y_dot,
              arrow_length_z,
              arrow_length_y, 
              head_width=0.05, head_length=arrow_head_length, fc='k', ec='k')


    plt.grid()
    plt.title('State Space').set_fontsize(20)
    plt.xlabel('$y$ [m]').set_fontsize(20)
    plt.ylabel('$\dot{y}$[$m/s$]').set_fontsize(20)
    plt.xticks(fontsize = 14)
    plt.yticks(fontsize = 14)
    plt.legend(['Initial','Predicted','Updated'],fontsize = 14)
    plt.axis('equal')
    plt.show()