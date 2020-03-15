#!/usr/bin/env python

import rospy
import matplotlib.pyplot as pl
import numpy as np
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import Twist

class controller:
    ''' The controller uses the interagent distances to determine the desired velocity of the Nexus '''
    
    def __init__(self):
        ''' Initiate self and subscribe to /z_values topic '''
        # controller variables
        self.running = np.float32(1)
        self.p34_star = np.array([[0],[0.8]]) 
        self.p24_star = np.array([[-0.8],[0.8]]) 
        self.p14_star = np.array([[-0.8],[0]]) 
        self.d = np.float32(0.8)
        self.dd = np.float32(np.sqrt(np.square(self.d)+np.square(self.d)))        
        self.c = np.float32(0.5)
        self.cf = np.float(1)
        self.U_old = np.array([0, 0])
        self.U_oldd = np.array([0, 0])

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
        self.E3_log = np.array([])
        self.E4_log = np.array([])
        self.E5_log = np.array([])
        self.Un = np.float32([])
        self.U_log = np.array([])
        self.time = np.float64([])
        self.time_log = np.array([])
        self.now = np.float64([rospy.get_time()])
        self.begin = np.float64([rospy.get_time()])
        self.k = 0
        print ' hello'
        # prepare shutdown
        rospy.on_shutdown(self.shutdown)
        
        # prepare publisher
        self.pub = rospy.Publisher('/n_4/cmd_vel', Twist, queue_size=1)
        self.velocity = Twist()
                
        # subscribe to z_values topic
        rospy.Subscriber('/n_4/r_values', numpy_msg(Floats), self.controller)
        
        # subscribe to controller_variables
        rospy.Subscriber('/controller_variables', numpy_msg(Floats), self.update_controller_variables)
        
    def update_controller_variables(self, data):
        ''' Update controller variables '''
        if self.running < 10:
            # Assign data 
            self.controller_variables = data.data

            # Safe variables
            self.running = np.float32(self.controller_variables[0])
            self.d = np.float32(self.controller_variables[1])
            self.dd = np.float32(self.controller_variables[2])
            self.c = np.float32(self.controller_variables[3])
            self.x_dot = np.float32(self.controller_variables[4])
            self.y_dot = np.float32(self.controller_variables[5])
            self.r_dot = np.float32(self.controller_variables[6])
            
            print "Controller variables = ", self.controller_variables

            # Calculate mu
            self.mu_x = self.x_dot*np.array([0, -1, 0, -1, 0])
            self.mut_x = self.x_dot*np.array([0, 1, 0, 1, 0])
            self.mu_y = self.y_dot*np.array([-1, 0, 0, 0, 1])
            self.mut_y = self.y_dot*np.array([1, 0, 0, 0, -1])
            self.mu_r = self.r_dot*np.array([-1, -1, 0, 1, -1])
            self.mut_r = self.r_dot*np.array([1, 1, 0, -1, 1])
            
            self.mu = self.mu_x+self.mu_y+self.mu_r
            self.mut = self.mut_x+self.mut_y+self.mut_r

            print self.mu, self.mut


        
    def controller(self, data):
        ''' Calculate U based on z_values '''    
        if self.running < 10:
            # Input for controller
            z_values= data.data
            
            # Nelsons formation shape control
            p34 = np.array([[z_values[1]], [z_values[2]]])
            p24 = np.array([[z_values[4]], [z_values[5]]])
            p14 = np.array([[z_values[7]], [z_values[8]]])
            # Formation shape control
            BbDz = np.array([[z_values[1], z_values[4], z_values[7]], \
                                  [z_values[2], z_values[5], z_values[8]]])
            Dzt = np.array([[(z_values[0])**(-1), 0, 0], \
                                 [0, (z_values[3])**(-1), 0], \
                                 [0, 0, (z_values[6])**(-1)]])
            Ed = np.array([[z_values[0]-self.d], \
                                [z_values[3]-self.dd], \
                                [z_values[6]-self.d]])
            
            # Formation motion control
            Ab = np.array([[self.mut[3], 0, self.mut[4], 0], \
                                [0, self.mut[3], 0, self.mut[4]]])


            z = np.array([z_values[7], z_values[8], z_values[1], z_values[2]])
            z = np.array([z_values[1], z_values[2], z_values[4], z_values[5]])

            # Control law
            Uf = self.cf * (p14-self.p14_star+p24-self.p24_star+p34-self.p34_star)
            U = Uf
            #U = self.c*BbDz.dot(Dzt).dot(Ed) + (Ab.dot(z)).reshape((2, 1))
            print "U = ", -U
                    
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
            self.E3_log = np.append(self.E3_log, Ed[1])
            self.E4_log = np.append(self.E4_log, Ed[2])
            self.E5_log = np.append(self.E5_log, Ed[0])
            self.Un = np.float32([np.sqrt(np.square(U[0])+np.square(U[1]))])
            self.U_log = np.append(self.U_log, self.Un)
            
            # Save current time in time log array
            if self.k < 1:
                self.begin = np.float64([rospy.get_time()])
                self.k = 10
            self.now = np.float64([rospy.get_time()])
            self.time = np.float64([self.now-self.begin])
            self.time_log = np.append(self.time_log, self.time)

            # publish
            self.publish_control_inputs(U[0], U[1])

        elif 10 < self.running < 1000:
            self.shutdown()

    def publish_control_inputs(self,x,y):
        ''' Publish the control inputs to command velocities

            NOTE: somehow the y direction has been reversed from the indigo-version from Johan and
            the regular kinetic version. Hence the minus sign.

        '''
        self.velocity.linear.x = x
        self.velocity.linear.y = y
        # print -x,-y
        #        rospy.loginfo(self.velocity)
         
        self.pub.publish(self.velocity)

    def shutdown(self):
        ''' Stop the robot when shutting down the controller_1 node '''
        rospy.loginfo("Stopping Nexus_4...")
        self.running = np.float32(10000)
        self.velocity = Twist()
        self.pub.publish(self.velocity)
        
#        np.save('/home/s2036975/Documents/Master Thesis/experiments/experiment_x/E3_log_nx4', self.E3_log)
#        np.save('/home/s2036975/Documents/Master Thesis/experiments/experiment_x/E4_log_nx4', self.E4_log)
#        np.save('/home/s2036975/Documents/Master Thesis/experiments/experiment_x/E5_log_nx4', self.E5_log)
#        np.save('/home/s2036975/Documents/Master Thesis/experiments/experiment_x/U_log_nx4', self.U_log)
#        np.save('/home/s2036975/Documents/Master Thesis/experiments/experiment_x/time_log_nx4', self.time_log)
        
        rospy.sleep(1)


        pl.close("all")       
        pl.figure(6)
        pl.title("Inter-agent distance error measured by Nexus 4")
        pl.plot(self.time_log, self.E3_log, label="e3_nx4", color='r')
        pl.plot(self.time_log, self.E4_log, label="e4_nx4", color='y')
        pl.plot(self.time_log, self.E5_log, label="e5_nx4", color='k')
        pl.xlabel("Time [s]")
        pl.ylabel("Error [m]")
        pl.grid()
        pl.legend()
        
        pl.figure(7)
        pl.title("Input velocity Nexus 4 ")
        pl.plot(self.time_log, self.U_log, label="pdot_nx4", color='y')
        pl.xlabel("Time [s]")
        pl.ylabel("Velocity [m/s]")
        pl.grid()
        pl.legend()
        
        pl.pause(0)




if __name__ == '__main__':
    try:
        rospy.init_node('controller_4', anonymous=False)
        controller()
        rospy.spin()
    except:
        rospy.loginfo("Controller node terminated.")  


# 3 Neighbours:
##            self.BbDz = np.array([[self.z_values[1], self.z_values[4], self.z_values[7]], \
##                                  [self.z_values[2], self.z_values[5], self.z_values[8]]])
##            self.Dzt = np.array([[(self.z_values[0])**(-1), 0, 0], \
##                                 [0, (self.z_values[3])**(-1), 0], \
##                                 [0, 0, (self.z_values[6])**(-1)]])
##            self.Ed = np.array([[self.z_values[0]-self.d], \
##                                [self.z_values[3]-self.dd], \
##                                [self.z_values[6]-self.d]])

# 2 neighbours
##            self.BbDz = np.array([[self.z_values[1], self.z_values[7]], \
##                                  [self.z_values[2], self.z_values[8]]])
##            self.Dzt = np.array([[(self.z_values[0])**(-1), 0], \
##                                 [0, (self.z_values[6])**(-1)]])
##            self.Ed = np.array([[self.z_values[0]-self.d], \
##                                [self.z_values[6]-self.d]])
##            self.U = self.c*self.BbDz.dot(self.Dzt).dot(self.Ed)
