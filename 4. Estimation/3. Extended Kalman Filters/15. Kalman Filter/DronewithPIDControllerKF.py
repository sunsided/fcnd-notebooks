from CoaxialDrone import CoaxialCopter
from PIDcontroller import PIDController_with_ff
import numpy as np 
import math
import matplotlib.pyplot as plt
import matplotlib.pylab as pylab


class DronewithPIDKF(CoaxialCopter,PIDController_with_ff):

    def __init__(self,
                z_path,
                z_dot_path,
                z_dot_dot_path,
                t,
                dt,
                Sensor,
                KF
                ):

        self.t = t
        self.dt = dt    
        self.z_path = z_path
        self.z_dot_path = z_dot_path
        self.z_dot_dot_path = z_dot_dot_path
        self.Sensor = Sensor 
        self.KF = KF  
        

    def PID_controller_with_KF(self, position_sigma, motion_sigma, use_kf=False):
    
        # Controller parameters 
        k_p = 20
        k_d = 5 
        k_i = 5 
        mass_err = 1.0 
        
        velocity_sigma = position_sigma           # velocity uncertainty
        
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
        
        Drone_Sensor = self.Sensor()

        observation_history = Controlled_Drone.X[0]

        mu = np.array([[Controlled_Drone.X[1]],[Controlled_Drone.X[0]]]) 
        sigma_cov = np.matmul(np.identity(2), np.array([velocity_sigma, position_sigma]))

        EKFfilter=self.KF(motion_sigma, velocity_sigma, position_sigma, self.dt)
        EKFfilter.initial_values(mu, sigma_cov)
        EKF_history = mu[1,0]

        sigma_cov_history = sigma_cov[1]
        
        # executing the flight
        for i in range(1,self.z_path.shape[0]-1):
            
            # condition to use height observation to control the drone or
            # use the magically given true state 

            z_observation = Drone_Sensor.measure(Controlled_Drone.X[0],position_sigma)

            if use_kf:
                
                z_EKF=mu[1,0]
                v_EKF=mu[0,0]   

                u_bar = control_system.control(self.z_path[i],
                                               z_EKF,
                                               self.z_dot_path[i],
                                               v_EKF,
                                               self.z_dot_dot_path[i],
                                               self.dt)
                
            else:
                
                u_bar = control_system.control(self.z_path[i],
                                               z_observation,
                                               self.z_dot_path[i],
                                               Controlled_Drone.X[2],
                                               self.z_dot_dot_path[i],
                                               self.dt)
                


            observation_history = np.vstack((observation_history,z_observation))

            u_bar = u_bar + np.random.normal(0.0, motion_sigma)
                

            Controlled_Drone.set_rotors_angular_velocities(u_bar, 0.0)

            # calculating the new state vector 
            drone_state = Controlled_Drone.advance_state(self.dt, actual_mass)
            
            # generating a history of vertical positions for the drone
            drone_state_history = np.vstack((drone_state_history, drone_state))


            #################
            mu_bar, sigma_bar = EKFfilter.predict(u_bar)
            
            
            z_observation= Drone_Sensor.measure(drone_state[0],position_sigma)

            mu, sigma_cov = EKFfilter.update(z_observation)
            #################

            # generating a history of vertical positions for the drone
            EKF_history= np.vstack((EKF_history,mu_bar[1,0]))
            sigma_cov_history = np.vstack((sigma_cov_history,sigma_cov[1,1]))

            

        
        plt.subplot(211)
        plt.plot(self.t,self.z_path,linestyle='-',marker='.',color='red',label = 'Planned path')
        
        if use_kf:
            plt.plot(self.t[1:],EKF_history[:,0],linestyle='-',color='green',linewidth=3,label = 'Estimated path')

        plt.plot(self.t[1:],drone_state_history[:,0],linestyle='-',color='blue',linewidth=3, label = 'Executed path')
        plt.scatter(self.t[1:],observation_history[:,0],color='black',marker='.',alpha=0.3, label = 'Observed value')
                


        plt.grid()
        if use_kf:
            plt.title('Change in height (using estimated value)').set_fontsize(20)
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


class DronewithPIDKFKnobs(CoaxialCopter,PIDController_with_ff):

    def __init__(self,
                z_path,
                z_dot_path,
                z_dot_dot_path,
                t,
                dt,
                Sensor,
                KF
                ):

        self.t = t
        self.dt = dt    
        self.z_path = z_path
        self.z_dot_path = z_dot_path
        self.z_dot_dot_path = z_dot_dot_path
        self.Sensor = Sensor 
        self.KF = KF  


    def PID_controller_with_KF_knobs(self, k_p, k_d, k_i, mass_err, position_sigma, motion_sigma, use_kf=False):
    
        
        velocity_sigma = position_sigma          # velocity uncertainty
        
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
        
        Drone_Sensor = self.Sensor()

        observation_history = Controlled_Drone.X[0]

        mu = np.array([[Controlled_Drone.X[1]],[Controlled_Drone.X[0]]]) 
        sigma_cov = np.matmul(np.identity(2), np.array([velocity_sigma,position_sigma]))

        EKFfilter=self.KF(motion_sigma,velocity_sigma,position_sigma,self.dt)
        EKFfilter.initial_values(mu, sigma_cov)
        EKF_history = mu[1,0]

        sigma_cov_history = sigma_cov[1]
        z_observation= Drone_Sensor.measure(Controlled_Drone.X[0],position_sigma)


        # executing the flight
        for i in range(1,self.z_path.shape[0]-1):
            
            # condition to use height observation to control the drone or
            # use the magically given true state 

            

            if use_kf:

                z_EKF=EKFfilter.mu[1,0]
                v_EKF=EKFfilter.mu[0,0]   
                    
                
                
                u_bar = control_system.control(self.z_path[i],
                                               z_EKF,
                                               self.z_dot_path[i],
                                               v_EKF,
                                               self.z_dot_dot_path[i],
                                               self.dt)
                
                
            else:
                
                

                u_bar = control_system.control(self.z_path[i],
                                               z_observation,
                                               self.z_dot_path[i],
                                               Controlled_Drone.X[2],
                                               self.z_dot_dot_path[i],
                                               self.dt)
                


            observation_history = np.vstack((observation_history,z_observation))

            u_bar=u_bar + np.random.normal(0.0, motion_sigma)
                

            Controlled_Drone.set_rotors_angular_velocities(u_bar,0.0)

            # calculating the new state vector 
            drone_state = Controlled_Drone.advance_state(self.dt, actual_mass)
            
            # generating a history of vertical positions for the drone
            drone_state_history = np.vstack((drone_state_history, drone_state))


            #################
            
            mu_bar, sigma_bar = EKFfilter.predict(Controlled_Drone.z_dot_dot(actual_mass))
            
            z_observation= Drone_Sensor.measure(Controlled_Drone.X[0],position_sigma)
            
            mu, sigma_cov = EKFfilter.update(z_observation)
            #################

            # generating a history of vertical positions for the drone
            EKF_history= np.vstack((EKF_history,mu[1,0]))
            sigma_cov_history = np.vstack((sigma_cov_history,sigma_cov[1,1]))
            

        
        plt.subplot(211)
        plt.plot(self.t,self.z_path,linestyle='-',marker='.',color='red', label = 'Planned path')
        
        
        if use_kf:
            plt.plot(self.t[1:],EKF_history,linestyle='-',color='green',linewidth=3, label='Estimated height')

        plt.plot(self.t[1:],drone_state_history[:,0],linestyle='-',color='blue',linewidth=3, label='Executed path')
        plt.scatter(self.t[1:],observation_history[:,0],color='black',marker='.',alpha=0.3, label='Observed value')
                


        plt.grid()
        if use_kf:
            plt.title('Change in height (using estimated value)').set_fontsize(20)
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


