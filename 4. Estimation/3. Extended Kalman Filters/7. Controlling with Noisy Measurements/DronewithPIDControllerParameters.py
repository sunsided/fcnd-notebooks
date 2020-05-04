from CoaxialDrone import CoaxialCopter
from PIDcontroller import PIDController_with_ff
import numpy as np 
import math
import matplotlib.pyplot as plt
import matplotlib.pylab as pylab


class DronewithPID(CoaxialCopter,PIDController_with_ff):

    def __init__(self,
                z_path,
                z_dot_path,
                z_dot_dot_path,
                t,
                dt,
                Sensor
                ):

        self.t = t
        self.dt = dt    
        self.z_path = z_path
        self.z_dot_path = z_dot_path
        self.z_dot_dot_path = z_dot_dot_path
        self.Sensor = Sensor 


    def PID_controller_with_measured_values(self,k_p,k_d,k_i,mass_err,sigma,use_measured_height=False):

        # creating the co-axial drone object 
        Controlled_Drone=CoaxialCopter()
        
        # array for recording the state history 
        drone_state_history = Controlled_Drone.X

        # introducing a small error of the actual mass and the mass for which the path has been calculated
        actual_mass = Controlled_Drone.m * mass_err 

        # creating the control system object 
        control_system = PIDController_with_ff(k_p,k_d,k_i)

        # declaring the initial state of the drone with zero hight and zero velocity 
        Controlled_Drone.X = np.array([0.0,0.0,0.0,0.0])
        
        Drone_Sensor = self.Sensor(Controlled_Drone.X, 0.95)
        observation_history = Controlled_Drone.X[0]

        # executing the flight
        for i in range(1,self.z_path.shape[0]-1):
            
            # condition to use height observation to control the drone or
            # use the magically given true state 
            if use_measured_height:
                
                z_observation = Drone_Sensor.measure(Controlled_Drone.X[0],sigma)
                
                u_bar = control_system.control(self.z_path[i],
                                               z_observation,
                                               self.z_dot_path[i],
                                               Controlled_Drone.X[2],
                                               self.z_dot_dot_path[i],
                                               self.dt)
                
                observation_history = np.vstack((observation_history,z_observation))
                
            else:
                
                u_bar = control_system.control(self.z_path[i],
                                               Controlled_Drone.X[0],
                                               self.z_dot_path[i],
                                               Controlled_Drone.X[2],
                                               self.z_dot_dot_path[i],
                                               self.dt)
                
                observation_history = np.vstack((observation_history,self.z_path[i]))
                
                
            Controlled_Drone.set_rotors_angular_velocities(u_bar,0.0)

            # calculating the new state vector 
            drone_state = Controlled_Drone.advance_state(self.dt, actual_mass)
            
            # generating a history of vertical positions for the drone
            drone_state_history = np.vstack((drone_state_history, drone_state))
            

        
        plt.subplot(211)
        plt.plot(self.t,self.z_path,linestyle='-',marker='.',color='red')
        plt.plot(self.t[1:],drone_state_history[:,0],linestyle='-',color='blue',linewidth=3)
        if use_measured_height:
            plt.scatter(self.t[1:],observation_history[:,0],color='black',marker='.',alpha=0.3)
                

        plt.grid()
        if use_measured_height:
            plt.title('Change in height (using measured value)').set_fontsize(20)
        else:
            plt.title('Change in height (ideal case)').set_fontsize(20)
                
        plt.xlabel('$t$ [sec]').set_fontsize(20)
        plt.ylabel('$z-z_0$ [$m$]').set_fontsize(20)
        plt.xticks(fontsize = 14)
        plt.yticks(fontsize = 14)
        if use_measured_height:
            plt.legend(['Planned path','Executed path','Observed value'],fontsize = 14)
        else:
            plt.legend(['Planned path','Executed path'],fontsize = 14)
        plt.show()


        plt.subplot(212)
        plt.plot(self.t[1:],abs(self.z_path[1:]-drone_state_history[:,0]),linestyle='-',marker='.',color='blue')
        plt.grid()
        plt.title('Error value ').set_fontsize(20)
        plt.xlabel('$t$ [sec]').set_fontsize(20)
        plt.ylabel('||$z_{target} - z_{actual}$|| [$m$]').set_fontsize(20)
        plt.xticks(fontsize = 14)
        plt.yticks(fontsize = 14)
        plt.legend(['Error'],fontsize = 14)
        plt.show()


    def PID_controller_with_estimated_values(self,k_p,k_d,k_i,mass_err,sigma,alpha,use_estimated_height=False):
    
        # creating the co-axial drone object 
        Controlled_Drone=CoaxialCopter()
        
        # array for recording the state history 
        drone_state_history = Controlled_Drone.X
        
        # introducing a small error of the actual mass and the mass for which the path has been calculated
        actual_mass = Controlled_Drone.m * mass_err 

        # creating the control system object 
        control_system = PIDController_with_ff(k_p,k_d,k_i)

        # declaring the initial state of the drone with zero hight and zero velocity 
        Controlled_Drone.X = np.array([0.0,0.0,0.0,0.0])
        
        Drone_Sensor = self.Sensor(Controlled_Drone.X, alpha)
        
        # recording the estimated height for each step
        estimated_height_history = Drone_Sensor.x_hat
        
        observation_history = Controlled_Drone.X[0]

        # executing the flight
        for i in range(1,self.z_path.shape[0]-1):
            
            # condition to use height observation to control the drone or
            # use the majically given true state 
            if use_estimated_height:
                
                z_observation = Drone_Sensor.measure(Controlled_Drone.X[0],sigma)
                z_estimated = Drone_Sensor.estimate(z_observation)
                
                
                u_bar = control_system.control(self.z_path[i],
                                               z_estimated,
                                               self.z_dot_path[i],
                                               Controlled_Drone.X[2],
                                               self.z_dot_dot_path[i],
                                               self.dt)
                
            else:
                
                z_observation = Drone_Sensor.measure(Controlled_Drone.X[0],sigma)
                
                u_bar = control_system.control(self.z_path[i],
                                               z_observation,
                                               self.z_dot_path[i],
                                               Controlled_Drone.X[2],
                                               self.z_dot_dot_path[i],
                                               self.dt)
                  
                
            Controlled_Drone.set_rotors_angular_velocities(u_bar,0.0)

            # calculating the new state vector 
            drone_state = Controlled_Drone.advance_state(self.dt, actual_mass)
            
            # generating a history of vertical positions for the drone
            drone_state_history = np.vstack((drone_state_history, drone_state))
            
            # generating the estimated height for each step
            estimated_height_history = np.vstack((estimated_height_history,Drone_Sensor.x_hat))
            observation_history = np.vstack((observation_history,z_observation))
            

        
        plt.subplot(211)
        plt.plot(self.t,self.z_path,linestyle='-',marker='.',color='red', label='Planned path')
        if use_estimated_height:
            plt.plot(self.t[1:],estimated_height_history[:,0],linestyle='-',color='green',linewidth=3, label='Averaged height')

        plt.plot(self.t[1:],drone_state_history[:,0],linestyle='-',color='blue',linewidth=3, label='Executed path')
        plt.scatter(self.t[1:],observation_history[:,0],color='black',marker='.',alpha=0.3, label='Observed value')
        


        plt.grid()
        if use_estimated_height:
            plt.title('Change in height (using averaged value)').set_fontsize(20)
        else:
            plt.title('Change in height (using measured value)').set_fontsize(20)
        plt.xlabel('$t$ [sec]').set_fontsize(20)
        plt.ylabel('$z-z_0$ [$m$]').set_fontsize(20)
        plt.xticks(fontsize = 14)
        plt.yticks(fontsize = 14)
        plt.legend(fontsize = 14)
        plt.show()


        plt.subplot(212)
        plt.plot(self.t[1:],abs(self.z_path[1:]-drone_state_history[:,0]),linestyle='-',marker='.',color='blue')
        plt.grid()
        plt.title('Error value ').set_fontsize(20)
        plt.xlabel('$t$ [sec]').set_fontsize(20)
        plt.ylabel('||$z_{target} - z_{actual}$|| [$m$]').set_fontsize(20)
        plt.xticks(fontsize = 14)
        plt.yticks(fontsize = 14)
        plt.legend(['Error'],fontsize = 14)
        plt.show()