#!/usr/bin/env python

import rospy
import message_filters
import matplotlib.pyplot as pl
import numpy as np
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import Twist
from math import exp

class controller:
    ''' The controller uses the interagent distances to determine the desired velocity of the Nexus '''
    
    def __init__(self):
        ''' Initiate self and subscribe to /r_values topic '''
        # controller variables
        self.running = np.float32(1)
        self.p13_star = np.array([[-0.8],[-0.8]]) 
        self.p23_star = np.array([[-0.8],[0]]) 
        self.p43_star = np.array([[0],[-0.8]]) 
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
        self.zeta3_old = np.zeros((2,1))
        self.zeta_values = np.empty(shape=(0,0))
        self.zeta3 = np.zeros((2,1))

        # Motion parameters
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
        self.E2_log = np.array([])
        self.E5_log = np.array([])
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
        self.pub_vel = rospy.Publisher('/n_3/cmd_vel', Twist, queue_size=10)
        self.velocity = Twist()

        self.pub_zeta = rospy.Publisher('/n_3/zeta_values', numpy_msg(Floats), queue_size=1)

        # subscribe to r_values topic
        rospy.Subscriber('/n_3/r_values', numpy_msg(Floats), self.controller)
        rospy.Subscriber('/n_3/obstacles', numpy_msg(Floats), self.obstacleAvoidance)
        
        # subscribe to zeta_values topic of each controller
        zeta1_sub = message_filters.Subscriber('/n_1/zeta_values', numpy_msg(Floats))
        zeta2_sub = message_filters.Subscriber('/n_2/zeta_values', numpy_msg(Floats))
        zeta4_sub = message_filters.Subscriber('/n_4/zeta_values', numpy_msg(Floats))
        # obs_values_sub = message_filters.Subscriber('/n_1/obstacles', numpy_msg(Floats))

        ts3 = message_filters.TimeSynchronizer([zeta1_sub, zeta2_sub, zeta4_sub], 10)
        ts3.registerCallback(self.orderZeta)

    def controller(self, data):
        ''' Calculate U based on r_values '''    
        if self.running < 10:
            # Input for controller
            r_values= data.data
            
            # Nelsons formation shape control
            p23 = np.array([[r_values[1]], [r_values[2]]])
            p13 = np.array([[r_values[4]], [r_values[5]]])
            p43 = np.array([[r_values[7]], [r_values[8]]])

            # Formation shape control
            BbDz = np.array([[r_values[1], r_values[7]], \
                                  [r_values[2], r_values[8]]])
            Dzt = np.array([[(r_values[0])**(-1), 0], \
                                 [0, (r_values[6])**(-1)]])
            Ed = np.array([[r_values[0]-self.d], \
                                [r_values[6]-self.d]])
            
            # Formation motion control
            Ab = np.array([[self.mut[1], 0, self.mu[4], 0], \
                                [0, self.mut[1], 0, self.mu[4]]])
            z = np.array([r_values[1], r_values[2], r_values[7], r_values[8]])
           
            #z = [ edge 1 , edge 4]
            print 'self.zeta3', self.zeta3
            if self.zeta3[0] != 0 and self.zeta3[1] != 0:
                print 'zeta3', self.zeta3
                U0 = self.zeta3
            else:
                print 'zeta1 set to zero'
                U0 = np.zeros((2,1))

            # Control law
            Uf = self.cf * (p13-self.p13_star+p23-self.p23_star+p43-self.p43_star)  # +np.array([[0.2],[0.2]])
            U = Uf - U0
            # U = self.c*BbDz.dot(Dzt).dot(Ed) + (Ab.dot(z)).reshape((2, 1))
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
            
            # Set old U values for moving average in order to prevent shaking
            self.U_oldd = self.U_old
            self.U_old = U

            # Append error and velocity in Log arrays
            self.E2_log = np.append(self.E2_log, Ed[0])
            self.E5_log = np.append(self.E5_log, Ed[1])
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
            self.publish_zeta(self.zeta3)

        elif 10 < self.running < 1000:
            self.shutdown()            

    def publish_control_inputs(self, x,y):
        ''' Publish the control inputs to command velocities

            NOTE: somehow the y direction has been reversed from the indigo-version from Johan and
            the regular kinetic version. Hence the minus sign.

        '''
        # rospy.sleep(1)
        self.velocity.linear.x = x
        self.velocity.linear.y = y
        # print -x,-y
        #        rospy.loginfo(self.velocity)
         
        self.pub_vel.publish(self.velocity)
    
    def publish_zeta(self, zeta3):
        '''communicates the state variable zeta to the neighbouring agents'''
        self.zeta3 = zeta3
        print zeta3.dtype
        self.pub_zeta.publish(self.zeta3)

    def obstacleAvoidance(self, data):
        obstacles = data.data
        print 'obstacles', obstacles
        
        if self.zeta_values.any():
            print 'self.z_values!!!!', self.zeta_values
            zeta1 = self.zeta_values[0]
            zeta2 = self.zeta_values[1]
            zeta4 = self.zeta_values[2]
        else:
            zeta1 = zeta2 = zeta4 = np.zeros((2,1))

        if obstacles[0] <= 0.8: 
            p03_obs01 = np.array([[obstacles[1]], [obstacles[2]]]) 
            norm_p03_obs01 = obstacles[0]
            print 'norm p03 obs01', norm_p03_obs01
            print 'zeta124', zeta1, zeta2, zeta4
            alpha3_1 = np.divide(p03_obs01, np.square(norm_p03_obs01)) - self.r_safe
            print 'alpha1_1', alpha3_1
            zeta3dot = self.czeta*((zeta1-self.zeta3_old)+(zeta2-self.zeta3_old)+(zeta4-self.zeta3_old))+self.calpha*alpha3_1
            print 'zzeta1dot', zeta3dot
            print 'zzetaold', self.zeta3_old
            zeta3 = self.zeta3_old + self.h*zeta3dot
            self.zeta3_old = zeta3
            print 'zeta3', zeta3
            self.zeta3 = np.array(zeta3, dtype=np.float32)
            print self.zeta3.dtype
            print self.zeta3
            return self.zeta3
        else:
            alpha3_1 = 0.0
            self.zeta3 = np.zeros((2,1), dtype = np.float32)
            return self.zeta3

    def orderZeta(self, data):
        if data:
            self.zeta_values = data.data
            print 'zeta_values', self.zeta_values
        else: 
            print 'nothing received'


    def shutdown(self):
        ''' Stop the robot when shutting down the controller_3 node '''
        rospy.loginfo("Stopping Nexus_3...")
        self.running = np.float32(10000)
        self.velocity = Twist()
        self.pub_vel.publish(self.velocity)

#        np.save('/home/s2036975/Documents/Master Thesis/experiments/experiment_x/E2_log_nx3', self.E2_log)
#        np.save('/home/s2036975/Documents/Master Thesis/experiments/experiment_x/E5_log_nx3', self.E5_log)
#        np.save('/home/s2036975/Documents/Master Thesis/experiments/experiment_x/U_log_nx3', self.U_log)
#        np.save('/home/s2036975/Documents/Master Thesis/experiments/experiment_x/time_log_nx3', self.time_log)

        rospy.sleep(1)
        
    
        pl.close("all")       
        pl.figure(4)
        pl.title("Inter-agent distance error measured by Nexus 3")
        pl.plot(self.time_log, self.E2_log, label="e2_nx3", color='g')
        pl.plot(self.time_log, self.E5_log, label="e5_nx3", color='k')
        pl.xlabel("Time [s]")
        pl.ylabel("Error [m]")
        pl.grid()
        pl.legend()
        
        pl.figure(5)
        pl.title("Input velocity Nexus 3 ")
        pl.plot(self.time_log, self.U_log, label="pdot_nx3", color='r')
        pl.xlabel("Time [s]")
        pl.ylabel("Velocity [m/s]")
        pl.grid()
        pl.legend()
        
        pl.pause(0)





if __name__ == '__main__':
    try:
        rospy.init_node('controller_3', anonymous=False)
        controller()
        rospy.spin()
    except:
        rospy.loginfo("Controller node terminated.")  


