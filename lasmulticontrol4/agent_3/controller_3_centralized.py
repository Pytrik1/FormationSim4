#!/usr/bin/env python
import sys
sys.path.insert(1, '/home/s2604833/catkin_ws/src/FormationSim4/lasmultigazebo4/config/')
import rospy
import message_filters
import matplotlib.pyplot as pl
import numpy as np
from rospy_tutorials.msg import Floats
from std_msgs.msg import Float32
from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import exp
import yaml
import params

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
        self.r_safe = params.r_safe
        self.dd = np.float32(np.sqrt(np.square(self.d)+np.square(self.d)))
        self.c = params.c
        self.cf = params.cf              # new gain for formation control Nelson
        self.cI = params.cI
        self.cP = params.cP
        self.cD = params.cD
        self.calpha = params.calpha
        self.czeta = params.czeta
        self.U_old = np.array([0, 0])
        self.U_oldd = np.array([0, 0])
        self.k = 0                          # current iteration of the Euler method
        self.h = params.h                        # stepsize
        self.zeta3_old = np.zeros((2,1))
        self.zeta_values = np.empty(shape=(0,0))
        self.zeta3 = np.zeros((2,1))
        self.gamma_old = np.zeros((2,1))    # sta
        self.gamma = np.zeros((2,1))
        self.pGoal = params.pGoal
        self.pcen_old = np.zeros((2,1))
        self.Ug_lim = params.Ug_lim
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
        self.pub_pcen = rospy.Publisher('/n_3/pcen', Float32, queue_size=1)
        # subscribe to r_values topic
        rospy.Subscriber('/n_3/r_values', numpy_msg(Floats), self.controller)
        rospy.Subscriber('/n_3/obstacles', numpy_msg(Floats), self.obstacleAvoidance)

        # Subscribe to the filtered odometry node
        rospy.Subscriber('/n_3/odometry/filtered', Odometry, self.formationMotion)

        rospy.Subscriber('/n_1/Ug_cmd_vel', Twist, self.Ug_cmd_vel)     
        
        # subscribe to zeta_values topic of each controller
        rospy.Subscriber('/n_1/zeta_values', numpy_msg(Floats), self.zeta1_sub)
        rospy.Subscriber('/n_2/zeta_values', numpy_msg(Floats), self.zeta2_sub)
        rospy.Subscriber('/n_4/zeta_values', numpy_msg(Floats), self.zeta4_sub)
        # obs_values_sub = message_filters.Subscriber('/n_1/obstacles', numpy_msg(Floats))

        # ts3 = message_filters.TimeSynchronizer([zeta1_sub, zeta2_sub, zeta4_sub], 10)
        # ts3.registerCallback(self.orderZeta)

    def controller(self, data):
        ''' Calculate U based on r_values '''    
        if self.running < 10:
            # Input for controller
            r_values= data.data
            
            # Nelsons formation shape control
            self.p13 = np.array([[r_values[1]], [r_values[2]]])
            self.p43 = np.array([[r_values[4]], [r_values[5]]])
            self.p23 = np.array([[r_values[7]], [r_values[8]]])

            # Error
            Ed = np.array([[r_values[0]-self.d], \
                                [r_values[6]-self.d]])
            
            # Formation motion control

            if self.zeta3[0] != 0 and self.zeta3[1] != 0:
                print 'zeta3', self.zeta3
                U0 = self.zeta3
            else:
                U0 = np.zeros((2,1))

            # Control law
            Uf = self.cf * (self.p23-self.p23_star+self.p43-self.p43_star)

            
            try:
                Ug = self.Ug
            except AttributeError:
                Ug = np.zeros((2,1))
            # np.clip(self.Ug, -self.Ug_lim, self.Ug_lim, out=Ug)
            U = Uf + Ug# - U0 + Ug
                    
            # Saturation
            # np.clip(U, -0.5, 0.5, out=U)
            U = self.saturation(U)
            # U = self.saturation(U)

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
        self.pub_zeta.publish(self.zeta3)

    def Ug_cmd_vel(self, msg):
        print 'msg', msg
        self.Ug = np.array([[msg.linear.x], [msg.linear.y]], dtype=np.float32)

    def formationMotion(self, msg):
        pass
    #     p3 = np.array([[msg.pose.pose.position.x], [msg.pose.pose.position.y]])

    #     p2 = self.p23 + p3
    #     p1 = self.p13 + p3
    #     p4 = self.p43 + p3

    #     pcen = np.array((p3 + p2 + p1 + p4)/4, dtype=np.float32)
    #     self.pub_pcen.publish(pcen[0])
    #     gammadot = -pcen + self.pGoal
    #     gamma = self.gamma_old + self.h*gammadot
    #     error = pcen-self.pGoal
    #     error_old = self.pcen_old - self.pGoal
    #     Ug =  -self.cP*(error) + self.cI * gamma + self.cD * (error-error_old)
    #     self.pcen_old = pcen
    #     self.gamma_old = gamma
    #     self.Ug = np.array(Ug, dtype=np.float32)

    def obstacleAvoidance(self, data):
        obstacles = data.data
        try:
            zeta1 = np.array([[self.zeta1[0]],[self.zeta1[1]]])
        except AttributeError:
            zeta1 = np.zeros((2,1))
        try:
            zeta2 = np.array([[self.zeta2[0]],[self.zeta2[1]]])
        except AttributeError:
            zeta2 = np.zeros((2,1))
        try:
            zeta4 = np.array([[self.zeta4[0]],[self.zeta4[1]]])
        except AttributeError:
            zeta4 = np.zeros((2,1))
        
        print 'zeta1', zeta1
        print 'zeta2', zeta2
        print 'zeta4', zeta4
        
        if obstacles[0] > 6 or not obstacles.any():
            self.zeta3 = np.zeros((2,1), dtype = np.float32)
            return
        p03_obs01 = np.array([[obstacles[1]], [obstacles[2]]]) 
        norm_p03_obs01 = obstacles[0]
        alpha3_1 = np.divide(p03_obs01, np.square(norm_p03_obs01)) - self.r_safe
        zeta3dot = self.czeta*((zeta1-self.zeta3_old)+(zeta2-self.zeta3_old)+(zeta4-self.zeta3_old))+self.calpha*alpha3_1
        zeta3 = self.zeta3_old + self.h*zeta3dot
        self.zeta3_old = zeta3
        self.zeta3 = np.array(zeta3/(1+exp(-params.cO*(-obstacles[0]+self.r_safe))), dtype=np.float32)
    
    def saturation(self, U):
        v_max = params.v_max
        v_min = params.v_min
        for i, v in enumerate(U):          
            if v > v_max:
                v = v_max
            elif v < -v_max:
                v = -v_max
            elif 0.03 < v < 0.1 or -0.1 < v < -0.03:
                v = 0.1/(1+10*exp(-300*(v-0.05)))
            elif -v_min < v+self.U_old[i]+self.U_oldd[i] < v_min : # preventing shaking 
                U[i] = 0
        return U

    def zeta1_sub(self, data):
        if data:
            self.zeta1 = data.data
        else:
            print('no zeta1 received')

    def zeta2_sub(self, data):
        if data:
            self.zeta2 = data.data
        else:
            print('no zeta2 received')

    def zeta4_sub(self, data):
        if data:
            self.zeta4 = data.data
        else:
            print('no zeta4 received')

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


