from PIDcontroller import PIDController_with_ff
import numpy as np 
import math
import matplotlib.pyplot as plt
import matplotlib.pylab as pylab


class DronewithPIDEKF(PIDController_with_ff):

    def __init__(self,
                z_path,
                z_dot_path,
                z_dot_dot_path,
                t,
                dt,
                IMU,
                drone,
                EKF
                ):

        self.t = t
        self.dt = dt    
        self.z_path = z_path
        self.z_dot_path = z_dot_path
        self.z_dot_dot_path = z_dot_dot_path
        self.IMU = IMU 
        self.drone = drone
        self.EKF = EKF  


    def PID_controler_with_EKF(self, angle_error, position_sigma, motion_sigma, use_ekf=False):
    
        # Controller parameters 
        k_p = 20
        k_d = 5 
        k_i = 5 

        velocity_sigma = position_sigma


        Controlled_Drone=self.drone(0.0, 0.0, 1.5)
        

        # array for recording the state history 
        drone_state_history = Controlled_Drone.X

        # creating the control system object 
        control_system = PIDController_with_ff(k_p,k_d,k_i)

        Drone_IMU = self.IMU()

        observation_hostory = Controlled_Drone.X[-1]

        mu = np.array([[Controlled_Drone.X[0]],[Controlled_Drone.X[1]],[Controlled_Drone.X[2]]]) 
        sigma_cov = np.matmul(np.identity(3), np.array([angle_error,velocity_sigma,position_sigma]))

        EKFfilter=self.EKF(motion_sigma,angle_error,velocity_sigma,position_sigma,self.dt)
        EKFfilter.initial_values(mu, sigma_cov)
        EKF_history = mu[2]

        sigma_cov_history = sigma_cov[1]
        z_observation= Drone_IMU.measure(Controlled_Drone.X[-1]/np.cos(Controlled_Drone.X[0]),position_sigma)

        # executing the flight
        for i in range(1,self.z_path.shape[0]-1):

            
            if use_ekf:

                z_EKF=mu[2]
                v_EKF=mu[1]   
                
                u_bar = control_system.control(self.z_path[i],
                                               z_EKF,
                                               self.z_dot_path[i],
                                               v_EKF,
                                               self.z_dot_dot_path[i],
                                               self.dt)

                
            else:

                

                u_bar = control_system.control(self.z_path[i],
                                               z_observation*np.cos(Controlled_Drone.X[0]),
                                               self.z_dot_path[i],
                                               Controlled_Drone.X[1],
                                               self.z_dot_dot_path[i],
                                               self.dt)
                



            observation_hostory = np.vstack((observation_hostory,z_observation))

            #u_bar=u_bar + np.random.normal(0.0, motion_sigma)
            u_bar=u_bar + np.random.normal(0.0, motion_sigma)

            # calculating the new state vector
            

            #u_bar = max(-1/np.sqrt(2),u_bar)
            #u_bar = min(1/np.sqrt(2),u_bar)
            u_bar = - u_bar
            u_bar = max(-1/np.sqrt(2),u_bar)
            u_bar = min(1/np.sqrt(2),u_bar)

            
            
            phi_new=float(np.arcsin(u_bar))
            
            drone_state = Controlled_Drone.advance_state(phi_new,self.dt)
            

            # generating a history of vertical positions for the drone
            drone_state_history = np.vstack((drone_state_history, drone_state))


            #################
            mu_bar, sigma_bar = EKFfilter.predict(phi_new)

            
            z_observation= Drone_IMU.measure(Controlled_Drone.X[2]/np.cos(Controlled_Drone.X[0]),position_sigma)

            mu, sigma_cov = EKFfilter.update(z_observation)
            #################

            # generating a history of vertical positions for the drone
            EKF_history= np.vstack((EKF_history,mu[-1]))
            sigma_cov_history = np.vstack((sigma_cov_history,sigma_cov[1,1]))



        plt.subplot(211)
        plt.plot(self.t,self.z_path,linestyle='-',marker='.',color='red', label='Planned path')
        
        if use_ekf:
            plt.plot(self.t[1:],EKF_history[:,0],linestyle='-',color='green',linewidth=3, label = 'Estimated position')

        plt.plot(self.t[1:],drone_state_history[:,-1],linestyle='-',color='blue',linewidth=3, label= 'Executed path')
        plt.scatter(self.t[1:],observation_hostory[:,0],color='black',marker='.',alpha=0.3, label = 'Observed value')



        plt.grid()
        if use_ekf:
            plt.title('Change in position (using estimated value)').set_fontsize(20)
        else:
            plt.title('Change in position (using measured value)').set_fontsize(20)

        plt.xlabel('$t$ [sec]').set_fontsize(20)
        plt.ylabel('$y$ [$m$]').set_fontsize(20)
        plt.xticks(fontsize = 14)
        plt.yticks(fontsize = 14)
        plt.legend(fontsize = 14)
        plt.show()


        plt.subplot(212)
        plt.plot(self.t[1:],abs(self.z_path[1:]-drone_state_history[:,-1]),linestyle='-',marker='.',color='blue')
        plt.grid()
        plt.title('Error value ').set_fontsize(20)
        plt.xlabel('$t$ [sec]').set_fontsize(20)
        plt.ylabel('||$y_{target} - y_{actual}$|| [$m$]').set_fontsize(20)
        plt.xticks(fontsize = 14)
        plt.yticks(fontsize = 14)
        plt.legend(['Error'],fontsize = 14)
        plt.show()



class DronewithPIDKFKnobs(PIDController_with_ff):

    def __init__(self,
                z_path,
                z_dot_path,
                z_dot_dot_path,
                t,
                dt,
                IMU,
                drone,
                EKF
                ):

        self.t = t
        self.dt = dt    
        self.z_path = z_path
        self.z_dot_path = z_dot_path
        self.z_dot_dot_path = z_dot_dot_path
        self.IMU = IMU 
        self.drone=drone
        self.EKF = EKF  


    def PID_controler_with_KF_knobs(self, k_p,k_d,k_i, angle_error, position_sigma, motion_sigma, use_ekf=False):
    
        
        velocity_sigma = position_sigma


        Controlled_Drone=self.drone(0.0, 0.0, 1.5)
        

        # array for recording the state history 
        drone_state_history = Controlled_Drone.X

        # creating the control system object 
        control_system = PIDController_with_ff(k_p,k_d,k_i)

        Drone_IMU = self.IMU()

        observation_hostory = Controlled_Drone.X[-1]

        mu = np.array([[Controlled_Drone.X[0]],[Controlled_Drone.X[1]],[Controlled_Drone.X[2]]]) 
        sigma_cov = np.matmul(np.identity(3), np.array([angle_error,velocity_sigma,position_sigma]))

        EKFfilter=self.EKF(motion_sigma,angle_error,velocity_sigma,position_sigma,self.dt)
        EKFfilter.initial_values(mu, sigma_cov)
        EKF_history = mu[2]

        sigma_cov_history = sigma_cov[1]
        z_observation= Drone_IMU.measure(Controlled_Drone.X[-1]/np.cos(Controlled_Drone.X[0]),position_sigma)

        # executing the flight
        for i in range(1,self.z_path.shape[0]-1):

            # condition to use height observation to control the drone or
            # use the magically given true state 

            if use_ekf:

                z_EKF=mu[2]
                v_EKF=mu[1]   
                

                u_bar = control_system.control(self.z_path[i],
                                               z_EKF,
                                               self.z_dot_path[i],
                                               v_EKF,
                                               self.z_dot_dot_path[i],
                                               self.dt)


            else:

                

                u_bar = control_system.control(self.z_path[i],
                                               z_observation*np.cos(Controlled_Drone.X[0]),
                                               self.z_dot_path[i],
                                               Controlled_Drone.X[1],
                                               self.z_dot_dot_path[i],
                                               self.dt)
                



            observation_hostory = np.vstack((observation_hostory,z_observation))

            u_bar=u_bar + np.random.normal(0.0, motion_sigma)

            # calculating the new state vector
            #u_bar = max(-1/np.sqrt(2),u_bar)
            #u_bar = min(1/np.sqrt(2),u_bar)
            
            u_bar = - u_bar
            u_bar = max(-1/np.sqrt(2),u_bar)
            u_bar = min(1/np.sqrt(2),u_bar)
            
            
            phi_new=float(np.arcsin(u_bar))
            
            drone_state = Controlled_Drone.advance_state(phi_new,self.dt)
            

            # generating a history of vertical positions for the drone
            drone_state_history = np.vstack((drone_state_history, drone_state))


            #################
            mu_bar, sigma_bar = EKFfilter.predict(phi_new)

            
            z_observation= Drone_IMU.measure(Controlled_Drone.X[2]/np.cos(Controlled_Drone.X[0]),position_sigma)

            mu, sigma_cov = EKFfilter.update(z_observation)
            #################

            # generating a history of vertical positions for the drone
            EKF_history= np.vstack((EKF_history,mu[-1]))
            sigma_cov_history = np.vstack((sigma_cov_history,sigma_cov[1,1]))



        plt.subplot(211)
        plt.plot(self.t,self.z_path,linestyle='-',marker='.',color='red', label='Planned path')
        
        if use_ekf:
            plt.plot(self.t[1:],EKF_history[:,0],linestyle='-',color='green',linewidth=3, label='Estimated position')

        plt.plot(self.t[1:],drone_state_history[:,-1],linestyle='-',color='blue',linewidth=3,label='Executing path')
        plt.scatter(self.t[1:],observation_hostory[:,0],color='black',marker='.',alpha=0.3, label='Observed value')



        plt.grid()
        if use_ekf:
            plt.title('Change in position (using estimated value)').set_fontsize(20)
        else:
            plt.title('Change in position (using measured value)').set_fontsize(20)

        plt.xlabel('$t$ [sec]').set_fontsize(20)
        plt.ylabel('$y$ [$m$]').set_fontsize(20)
        plt.xticks(fontsize = 14)
        plt.yticks(fontsize = 14)
        plt.legend(fontsize = 14)
        plt.show()


        plt.subplot(212)
        plt.plot(self.t[1:],abs(self.z_path[1:]-drone_state_history[:,-1]),linestyle='-',marker='.',color='blue')
        plt.grid()
        plt.title('Error value ').set_fontsize(20)
        plt.xlabel('$t$ [sec]').set_fontsize(20)
        plt.ylabel('||$y_{target} - y_{actual}$|| [$m$]').set_fontsize(20)
        plt.xticks(fontsize = 14)
        plt.yticks(fontsize = 14)
        plt.legend(['Error'],fontsize = 14)
        plt.show()




