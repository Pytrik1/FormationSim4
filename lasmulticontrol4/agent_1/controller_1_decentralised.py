#!/usr/bin/env python
import sys
sys.path.insert(1, '/home/s2604833/catkin_ws/src/FormationSim4/lasmultigazebo4/config/')
import rospy
import message_filters
import matplotlib.pyplot as pl
import numpy as np
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import exp
import yaml
import params

class Controller:
    ''' The controller uses the (r)elative interagent distances (r_values) to determine the desired velocity of the Nexus '''
    
    def __init__(self):
        ''' Initiate self and subscribe to /r_values and /zeta_value topic '''
        # controller variables
        self.running = np.float32(1)
        self.p21_star = np.array([[0],[0.8]]) 
        self.p31_star = np.array([[0.8],[0.8]])
        self.p41_star = np.array([[0.8],[0]]) 
        self.d = np.float32(0.8)
        self.r_safe = params.r_safe
        self.dd = np.float32(np.sqrt(np.square(self.d)+np.square(self.d)))
        self.c = params.c
        self.cf = params.cf              # new gain for formation control Nelson
        self.cI = params.cI
        self.cP = params.cP
        self.calpha = params.calpha
        self.czeta = params.czeta
        self.U_old = np.array([0, 0])
        self.U_oldd = np.array([0, 0])
        self.k = 0                          # current iteration of the Euler method
        self.h = params.h                        # stepsize
        self.cD = params.cD
        self.zeta1_old = np.zeros((2,1))
        self.zeta_values = np.empty(shape=(0,0))
        self.zeta1 = np.zeros((2,1))
        self.gamma_old = np.zeros((2,1))
        self.pcen_old = np.zeros((2,1))
        self.gamma = np.zeros((2,1))
        self.pGoal = params.pGoal
        self.Ug_lim = params.Ug_lim
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
        self.Pcenlog = np.array([])
        self.U_log = np.array([])
        self.sum_U_log = np.zeros((2,1))
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
        self.pub_pcen = rospy.Publisher('/n_1/pcen', numpy_msg(Floats), queue_size=1)

        # subscribe to r_values topic
        rospy.Subscriber('/n_1/r_values', numpy_msg(Floats), self.controller)
        rospy.Subscriber('/n_1/obstacles', numpy_msg(Floats), self.obstacleAvoidance)

        # Subscribe to the filtered odometry node
        rospy.Subscriber('/n_1/odometry/filtered', Odometry, self.formationMotion)

        rospy.Subscriber('/n_1/Ug_cmd_vel', Twist, self.Ug_cmd_vel)
        
        # subscribe to zeta_values topic of each controller
        rospy.Subscriber('/n_2/zeta_values', numpy_msg(Floats), self.zeta2_sub)
        rospy.Subscriber('/n_3/zeta_values', numpy_msg(Floats), self.zeta3_sub)
        rospy.Subscriber('/n_4/zeta_values', numpy_msg(Floats), self.zeta4_sub)
        # obs_values_sub = message_filters.Subscriber('/n_1/obstacles', numpy_msg(Floats))

        # ts1 = message_filters.TimeSynchronizer([zeta2_sub, zeta3_sub, zeta4_sub], 10)
        # ts1.registerCallback(self.orderZeta)
        
    def controller(self, data):
        ''' Calculate U based on r_values and save error velocity in log arrays '''    
        if self.running < 10:
            # input for controller
            r_values= data.data
            
            # Nelson's formation shape control
            self.p41 = np.array([[r_values[1]], [r_values[2]]])
            self.p31 = np.array([[r_values[4]], [r_values[5]]])
            self.p21 = np.array([[r_values[7]], [r_values[8]]])

            # Error
            Ed = np.array([[r_values[0]-self.d], \
                                [r_values[6]-self.d]])
            
            
            if self.zeta1[0] != 0 and self.zeta1[1] != 0:
                print 'zeta1', self.zeta1
                U0 = self.zeta1
            else:
                print 'zeta1 set to zero'
                U0 = np.zeros((2,1))
            # Control law
            Uf = self.cf * (self.p21-self.p21_star+self.p41-self.p41_star)

            # Nelsons path planning control
            try:
                Ug = self.Ug
            except AttributeError:
                Ug = np.zeros((2,1))
            # print 'Ug', Ug
            # np.clip(self.Ug, -self.Ug_lim, self.Ug_lim, out=Ug)
            print 'Uf', Uf
            # Control Law
            U = Uf  + Ug# - U0 + Ug
            
            # Saturation
            # np.clip(U, -0.5, 0.5, out=U)
            U = self.saturation(U)

            # Set old U values in order to prevent shaking
            self.U_oldd = self.U_old
            self.U_old = U
            
            # Append error and velocity in Log arrays
            self.E1_log = np.append(self.E1_log, Ed[1])
            self.E4_log = np.append(self.E4_log, Ed[0])
            self.Un = np.float32([np.sqrt(np.square(U[0])+np.square(U[1]))])
            self.U_log = np.append(self.U_log, self.Un)

            # Save current time in time log array
            self.sum_U_log.dtype
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
        ''' Publish the control inputs to command velocities '''

        self.velocity.linear.x = x
        self.velocity.linear.y = y
        self.pub_vel.publish(self.velocity)

    def publish_zeta(self, zeta1):
        '''communicates the state variable zeta to the neighbouring agents'''
        self.zeta1 = zeta1
        print 'zeta1', zeta1
        self.pub_zeta.publish(self.zeta1)

    def Ug_cmd_vel(self, msg):
        print msg
        self.Ug = np.array([[msg.linear.x], [msg.linear.y]], dtype=np.float32)

    def formationMotion(self, msg):
        p1 = np.array([[msg.pose.pose.position.x], [msg.pose.pose.position.y]])
        p4 = self.p41 + p1
        p3 = self.p31 + p1
        p2 = self.p21 + p1
        pcen = np.array((p1 + p4 + p3 + p2)/4, dtype=np.float32)
        self.Pcenlog = np.append(self.Pcenlog, pcen)

        # pcen = np.array((p1 + p4 + p3 + p2)/4, dtype=np.float32)
        # self.pub_pcen.publish(pcen)
        # gammadot = -pcen + self.pGoal
        # gamma = self.gamma_old + self.h*gammadot
        # error = pcen-self.pGoal
        # error_old = self.pcen_old - self.pGoal
        # Ug =  -self.cP*(error) + self.cI * gamma + self.cD * (error-error_old)
        # self.pcen_old = pcen
        # self.gamma_old = gamma
        # self.Ug = np.array(Ug, dtype=np.float32)

    def obstacleAvoidance(self, data):
        obstacles = data.data
        
        zeta2 = np.array([[self.zeta2[0]],[self.zeta2[1]]])
        zeta3 = np.array([[self.zeta3[0]],[self.zeta3[1]]])
        zeta4 = np.array([[self.zeta4[0]],[self.zeta4[1]]])

        print 'zeta1', zeta2
        print 'zeta2', zeta3
        print 'zeta4', zeta4

        if obstacles[0] > 6 or not obstacles.any():
            self.zeta1 = np.zeros((2,1), dtype=np.float32)
            return
        p01_obs01 = np.array([[obstacles[1]], [obstacles[2]]]) 
        norm_p01_obs01 = obstacles[0]
        alpha1_1 = np.divide(p01_obs01, np.square(norm_p01_obs01)) - self.r_safe
        zeta1dot = self.czeta*((zeta2-self.zeta1_old)+(zeta3-self.zeta1_old)+(zeta4-self.zeta1_old))+self.calpha*alpha1_1
        zeta1 = self.zeta1_old + self.h*zeta1dot
        self.zeta1_old = zeta1
        self.zeta1 = np.array(zeta1/(1+exp(-params.cO*(-obstacles[0]+self.r_safe))), dtype=np.float32)

    def saturation(self, U):
        v_max = params.v_max
        v_min = params.v_min
        for i, v in enumerate(U):          
            if v > v_max:
                v = v_max
            elif v < -v_max:
                v = -v_max
            elif 0.03 < v < v_min or -v_min < v < -0.03:
                v = 0.1/(1+10*exp(-300*(v-0.05)))
            elif -v_min < v+self.U_old[i]+self.U_oldd[i] < v_min : # preventing shaking 
                U[i] = 0
        return U

    def zeta2_sub(self, data):
        if data:
            self.zeta2 = data.data
        else:
            print('no zeta3 received')

    def zeta3_sub(self, data):
        if data:
            self.zeta3 = data.data
        else:
            print('no zeta3 received')

    def zeta4_sub(self, data):
        if data:
            self.zeta4 = data.data
        else:
            print('no zeta4 received')

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
        pl.savefig('/home/s2604833/catkin_ws/src/FormationSim4/lasmulticontrol4/agent_1/inter_agent_error.eps')
        pl.figure(1)
        pl.title("Input velocity Nexus 1 ")
        pl.plot(self.time_log, self.U_log, label="pdot_nx1", color='b')
        pl.xlabel("Time [s]")
        pl.ylabel("Velocity [m/s]")
        pl.grid()
        pl.legend()
        pl.savefig('/home/s2604833/catkin_ws/src/FormationSim4/lasmulticontrol4/agent_1/input_value_nexus1.eps')
        pl.figure(2)
        pl.title("Path of the formation's centroid")
        pl.plot(self.Pcenlog, label='centroid', color='b')
        pl.xlabel("x")
        pl.ylabel("y")
        pl.grid()
        pl.legend()
        pl.savefig('/home/s2604833/catkin_ws/src/FormationSim4/lasmulticontrol4/agent_1/pcen_path.eps')
        pl.pause(0)

if __name__ == '__main__':
    try:
        rospy.init_node('controller_1', anonymous=False)
        Controller()
        rospy.spin()
    except:
        rospy.loginfo("Controller node_1 terminated.")  


