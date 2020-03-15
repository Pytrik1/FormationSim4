#!/usr/bin/env python

import rospy
import message_filters
import matplotlib.pyplot as pl
import numpy as np
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import Twist
from math import exp

class Controller:
    ''' The controller uses the (r)elative interagent distances (r_values) to determine the desired velocity of the Nexus '''
    
    def __init__(self):
        ''' Initiate self and subscribe to /r_values and /zeta_value topic '''
        # controller variables
        self.running = np.float32(1)
        # self.d = np.float32(0.8*1.025)
        self.p21_star = np.array([[0],[0.8]]) 
        self.p31_star = np.array([[0.8],[0.8]])
        self.p41_star = np.array([[0.8],[0]]) 
        self.d = np.float32(0.8)
        self.r_safe = np.float32(0.5)
        self.dd = np.float32(np.sqrt(np.square(self.d)+np.square(self.d)))
        self.c = np.float32(0.5)
        self.cf = np.float(1.0)              # new gain for formation control Nelson
        self.calpha = np.float(1.0)
        self.czeta = np.float(1)
        self.U_old = np.array([0, 0])
        self.U_oldd = np.array([0, 0])
        self.k = 0                          # current iteration of the Euler method
        self.h = 0.1                        # stepsize
        self.zeta1_old = np.zeros((2,1))
        self.zeta_values = np.empty(shape=(0,0))
        self.zeta1 = np.zeros((2,1))
        # motion parameters
        self.x_dot = np.float32(0)
        self.y_dot = np.float32(0)
        self.r_dot = np.float32(0)
        self.mu_x = self.x_dot*np.array([0, -1, 0, -1, 0])
        self.mut_x = self.x_dot*np.array([0, 1, 0, 1, 0])
        self.mu_y = self.y_dot*np.array([-1, 0, 0, 0, 1])
        self.mut_y = self.y_dot*np.array([1, 0, 0, 0, -1])
        self.mu_r = self.r_dot*np.array([-1, -1, 0, 1, -1])
        self.mut_r = self.r_dot*np.array([1, 1, 0, -1, 1])
        
        self.mu = self.mu_x+self.mu_y+self.mu_r
        self.mut = self.mut_x+self.mut_y+self.mut_r
                
        # prepare Log arrays
        self.E1_log = np.array([])
        self.E4_log = np.array([])
        self.Un = np.float32([])
        self.U_log = np.array([])
        self.time = np.float64([])
        self.time_log = np.array([])
        self.now = np.float64([rospy.get_time()])
        self.begin = np.float64([rospy.get_time()])
        self.l = 0

        # prepare shutdown
        rospy.on_shutdown(self.shutdown)
        
        # prepare publisher
        self.pub_vel = rospy.Publisher('/n_1/cmd_vel', Twist, queue_size=1)
        self.velocity = Twist()

        self.pub_zeta = rospy.Publisher('/n_1/zeta_values', numpy_msg(Floats), queue_size=1)

        # subscribe to r_values topic
        rospy.Subscriber('/n_1/r_values', numpy_msg(Floats), self.controller)
        rospy.Subscriber('/n_1/obstacles', numpy_msg(Floats), self.obstacleAvoidance)
        
        # subscribe to zeta_values topic of each controller
        zeta2_sub = message_filters.Subscriber('/n_2/zeta_values', numpy_msg(Floats))
        zeta3_sub = message_filters.Subscriber('/n_3/zeta_values', numpy_msg(Floats))
        zeta4_sub = message_filters.Subscriber('/n_4/zeta_values', numpy_msg(Floats))
        # obs_values_sub = message_filters.Subscriber('/n_1/obstacles', numpy_msg(Floats))

        ts1 = message_filters.TimeSynchronizer([zeta2_sub, zeta3_sub, zeta4_sub], 10)
        ts1.registerCallback(self.orderZeta)
        
    def controller(self, data):
        ''' Calculate U based on r_values and save error velocity in log arrays '''    
        if self.running < 10:
            # input for controller
            r_values= data.data
            
            # Nelson's formation shape control
            p41 = np.array([[r_values[1]], [r_values[2]]])
            p31 = np.array([[r_values[4]], [r_values[5]]])
            p21 = np.array([[r_values[7]], [r_values[8]]])

            # Error
            Ed = np.array([[r_values[0]-self.d], \
                                [r_values[6]-self.d]])
            #Nelsons motion to goal control
            #pcen = ((4**(-1)-cI*gamma))

            #Nelsons obstacle avoidance
            # zeta1 = self.obstacleAvoidance()
            
            # Formation motion control
            Ab = np.array([[self.mu[0], 0, self.mu[3], 0], \
                                [0, self.mu[0], 0, self.mu[3]]])
            z = np.array([r_values[7], r_values[8], r_values[1], r_values[2]])
            #z = [ edge 1 , edge 4]
            print 'self.zeta1', self.zeta1
            if self.zeta1[0] != 0 and self.zeta1[1] != 0:
                print 'zeta1', self.zeta1
                U0 = self.zeta1
            else:
                print 'zeta1 set to zero'
                U0 = np.zeros((2,1))
            # Control law
            Uf = self.cf * (p21-self.p21_star+p31-self.p31_star+p41-self.p41_star)
            # U0 = zeta1
            # Ug = self.cp * (pcen
            U = Uf - U0
            # U = self.c*BbDz.dot(Dzt).dot(Ed) + + (Ab.dot(z)).reshape((2, 1))
            # print "U = ", -U
        
            # Saturation
            v_max = 0.3
            v_min = 0.02
            for i in range(len(U)):          
                if U[i] > v_max:
                    U[i] = v_max
                elif U[i] < -v_max:
                    U[i] = -v_max
                elif -v_min < U[i]+self.U_old[i]+self.U_oldd[i] < v_min : # preventing shaking 
                    U[i] = 0
            
            # Set old U values in order to prevent shaking
            self.U_oldd = self.U_old
            self.U_old = U
            

            # Append error and velocity in Log arrays
            self.E1_log = np.append(self.E1_log, Ed[1])
            self.E4_log = np.append(self.E4_log, Ed[0])
            self.Un = np.float32([np.sqrt(np.square(U[0])+np.square(U[1]))])
            self.U_log = np.append(self.U_log, self.Un)
            
            # Save current time in time log array
            if self.l < 1:
                self.begin = np.float64([rospy.get_time()])
                self.l = 10
            self.now = np.float64([rospy.get_time()])
            self.time = np.float64([self.now-self.begin])
            self.time_log = np.append(self.time_log, self.time)
            
            # publish
            self.publish_control_inputs(U[0], U[1])
            self.publish_zeta(self.zeta1)
            

        elif 10 < self.running < 1000:
            self.shutdown()

    def publish_control_inputs(self,x,y):
        ''' Publish the control inputs to command velocities

            NOTE: somehow the y direction has been reversed from the indigo-version from Johan and
            the regular kinetic version. Hence the minus sign.

        '''

        self.velocity.linear.x = x
        self.velocity.linear.y = y
        self.pub_vel.publish(self.velocity)

    # def findEta(self,zeta):
    #     zetadot =  np.array([[0],[0]])
    #     norm_po11 = np.linalg.norm(po11)
    #     alpha1_1 = (po11/(norm_po11))*((norm_po11)**(-1))
    #     zetadot[0] = self.czeta*((zeta2-zeta1)+(zeta3-zeta1)+(zeta4-zeta1))+self.calpha*alpha1_1
    #     return zetadot1 

    def publish_zeta(self, zeta1):
        '''communicates the state variable zeta to the neighbouring agents'''
        self.zeta1 = zeta1
        print 'zeta1', zeta1
        self.pub_zeta.publish(self.zeta1)

    def obstacleAvoidance(self, data):
        obstacles = data.data
        print 'obstacles', obstacles
        
        if self.zeta_values.any():
            print 'self.z_values!!!!', self.zeta_values
            zeta2 = self.zeta_values[0]
            zeta3 = self.zeta_values[1]
            zeta4 = self.zeta_values[2]
        else:
            zeta2 = zeta3 = zeta4 = np.zeros((2,1))

        if obstacles[0] <= 0.8: 
            p01_obs01 = np.array([[obstacles[1]], [obstacles[2]]]) 
            norm_p01_obs01 = obstacles[0]
            print 'norm p01 obs01', norm_p01_obs01
            print 'zeta234', zeta2, zeta3, zeta4
            alpha1_1 = np.divide(p01_obs01, np.square(norm_p01_obs01)) - self.r_safe
            print 'alpha1_1', alpha1_1
            print 'zeta', zeta2
            zeta1dot = self.czeta*((zeta2-self.zeta1_old)+(zeta3-self.zeta1_old)+(zeta4-self.zeta1_old))+self.calpha*alpha1_1
            print 'zzeta1dot', zeta1dot
            print 'zzetaold', self.zeta1_old
            zeta1 = self.zeta1_old + self.h*zeta1dot
            self.zeta1_old = zeta1
            self.zeta1 = np.array(zeta1, dtype=np.float32)
            return self.zeta1
        else:
            alpha1_1 = 0.0
            self.zeta1 = np.zeros((2,1), dtype=np.float32)
            return self.zeta1

    def orderZeta(self, data):
        if data:
            self.zeta_values = data.data
            print 'zeta values', self.zeta_values

    def shutdown(self):
        ''' Stop the robot when shutting down the controller_1 node '''
        rospy.loginfo("Stopping Nexus_1...")
        self.running = np.float32(10000)
        self.velocity = Twist()
        self.pub_vel.publish(self.velocity)

#        np.save('/home/s2604833/Documents/Master Thesis/experiments/experiment_x/E1_log_nx1', self.E1_log)
#        np.save('/home/s2604833/Documents/Master Thesis/experiments/experiment_x/E4_log_nx1', self.E4_log)
#        np.save('/home/s2604833/Documents/Master Thesis/experiments/experiment_x/U_log_nx1', self.U_log)
#        np.save('/home/s2604833/Documents/Master Thesis/experiments/experiment_x/time_log_nx1', self.time_log)

        rospy.sleep(1)


        pl.close("all")       
        pl.figure(0)
        pl.title("Inter-agent distance error measured by Nexus 1")
        pl.plot(self.time_log, self.E1_log, label="e1_nx1", color='b')
        pl.plot(self.time_log, self.E4_log, label="e4_nx1", color='y')
        pl.xlabel("Time [s]")
        pl.ylabel("Error [m]")
        pl.grid()
        pl.legend()
        
        pl.figure(1)
        pl.title("Input velocity Nexus 1 ")
        pl.plot(self.time_log, self.U_log, label="pdot_nx1", color='b')
        pl.xlabel("Time [s]")
        pl.ylabel("Velocity [m/s]")
        pl.grid()
        pl.legend()
        
        pl.pause(0)

# time synchronizer


if __name__ == '__main__':
    try:
        rospy.init_node('controller_1', anonymous=False)
        Controller()
        rospy.spin()
    except:
        rospy.loginfo("Controller node_1 terminated.")  


