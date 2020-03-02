#!/usr/bin/env python

import rospy
import matplotlib.pyplot as pl
import numpy as np
from rospy_tutorials.msg import Floats
from std_msgs.msg import Int32
from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import Twist

class controller:
    ''' The controller uses the interagent distances to determine the desired velocity of the Nexus '''
    
    def __init__(self):
        ''' Initiate self and subscribe to /z_values topic '''
        # controller variables
        self.running = np.float32(1)
        self.d = np.float32(0.8)
        # self.dd = np.float32(np.sqrt(np.square(self.d)+np.square(self.d))*1.02)
        self.dd = np.float32(np.sqrt(np.square(self.d)+np.square(self.d)))
        self.c = np.float32(0.5)
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
        self.E1_log = np.array([])
        self.E2_log = np.array([])
        #self.E3_log = np.array([])
        self.Un = np.float32([])
        self.U_log = np.array([])
        self.time = np.float64([])
        self.time_log = np.array([])
        self.now = np.float64([rospy.get_time()])
        self.begin = np.float64([rospy.get_time()])
        self.k = 0

        # prepare shutdown
        rospy.on_shutdown(self.shutdown)
        
        # prepare publisher
        self.pub = rospy.Publisher('/n_3/cmd_vel', Twist, queue_size=1)
        self.velocity = Twist()
                
        # subscribe to z_values topic
        rospy.Subscriber('/n_3/z_values', numpy_msg(Floats), self.controller)
        rospy.Subscriber('/n_3/agents', Int32, self.n)
        # subscribe to controller_variables
        rospy.Subscriber('/controller_variables', numpy_msg(Floats), self.update_controller_variables)

    def n(self, data):
        if self.running < 10:
            self.n=data.data
       
        elif 10 < self.running < 1000:
            self.shutdown()

    def update_controller_variables(self, data):
        ''' Update controller variables '''
        if self.running < 10:
            # Assign data 
            self.controller_variables = data.data

            # Safe variables
            self.running = np.float32(self.controller_variables[0])
            self.d = np.float32(self.controller_variables[1])
            # self.d = np.float32(self.controller_variables[1])
            self.dd = np.float32(self.controller_variables[2])
            self.c = np.float32(self.controller_variables[3])
            self.x_dot = np.float32(self.controller_variables[4])
            self.y_dot = np.float32(self.controller_variables[5])
            self.r_dot = np.float32(self.controller_variables[6])
            
            # Calculate mu
            self.mu_x = self.x_dot*np.array([0, -1, 0, -1, 0])
            self.mut_x = self.x_dot*np.array([0, 1, 0, 1, 0])
            self.mu_y = self.y_dot*np.array([-1, 0, 0, 0, 1])
            self.mut_y = self.y_dot*np.array([1, 0, 0, 0, -1])
            self.mu_r = self.r_dot*np.array([-1, -1, 0, 1, -1])
            self.mut_r = self.r_dot*np.array([1, 1, 0, -1, 1])
            
            self.mu = self.mu_x+self.mu_y+self.mu_r
            self.mut = self.mut_x+self.mut_y+self.mut_r
        
    def controller(self, data):
        ''' Calculate U based on z_values and save error velocity in log arrays '''    
        if self.running < 10:
            # Input for controller
            z_values= data.data

            Bx=np.array([])
            By=np.array([])
            D = np.array([])
            E = np.array([])
            Ed=[]

            for i in range(self.n):
                Bx=np.append(Bx, z_values[1+3*i])
                By=np.append(By, z_values[2+3*i])
                D=np.append(D, z_values[3*i]**(-1))
                E=np.append(E,z_values[3*i]-self.d)
                Ed.append([E[i]])

            BbDz=np.append([Bx],[By], axis=0)
            Dzt=np.diag(D)
            Ed=np.asarray(Ed)
            # print "Error = ", Ed
            # print "Distance = ", [z_values[0], z_values[3], z_values[6]]

            # Formation motion control
            #Ab = np.array([[self.mut[0], 0, self.mu[1], 0], \
                                #[0, self.mut[0], 0, self.mu[1]]])
            # z = np.array([z_values[4], z_values[5], z_values[1], z_values[2]])
            #z = np.array([z_values[1], z_values[2], z_values[7], z_values[8]])

            # Control law
            U = self.c*BbDz.dot(Dzt).dot(Ed) #+ (Ab.dot(z)).reshape((2, 1))
            print "U = ", -U
            v_max = 0.2
            v_min = 0.02       
            # Saturation
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
            self.E1_log = np.append(self.E1_log, Ed[1])
            self.E2_log = np.append(self.E2_log, Ed[0])
            #self.E3_log = np.append(self.E3_log, Ed[2])
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
        #        rospy.loginfo(self.velocity)

         
        self.pub.publish(self.velocity)

    def shutdown(self):
        ''' Stop the robot when shutting down the controller_3 node '''
        rospy.loginfo("Stopping Nexus_3...")
        self.running = np.float32(10000)
        self.velocity = Twist()
        self.pub.publish(self.velocity)

#        np.save('/home/s2036975/Documents/Master Thesis/experiments/experiment_x/E1_log_nx2', self.E1_log)
#        np.save('/home/s2036975/Documents/Master Thesis/experiments/experiment_x/E2_log_nx2', self.E2_log)
#        np.save('/home/s2036975/Documents/Master Thesis/experiments/experiment_x/E3_log_nx2', self.E3_log)
#        np.save('/home/s2036975/Documents/Master Thesis/experiments/experiment_x/U_log_nx2', self.U_log)
#        np.save('/home/s2036975/Documents/Master Thesis/experiments/experiment_x/time_log_nx2', self.time_log)        

        rospy.sleep(1)
        

        pl.close("all")       
        pl.figure(2)
        pl.title("Inter-agent distance error measured by Nexus 3")
        pl.plot(self.time_log, self.E1_log, label="e1_nx2", color='b')
        pl.plot(self.time_log, self.E2_log, label="e2_nx2", color='g')
        pl.plot(self.time_log, self.E3_log, label="e3_nx2", color='r')
        pl.xlabel("Time [s]")
        pl.ylabel("Error [m]")
        pl.grid()
        pl.legend()
        
        pl.figure(3)
        pl.title("Input velocity Nexus 3 ")
        pl.plot(self.time_log, self.U_log, label="pdot_nx2", color='g')
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

# 3 neighbours:
##            self.BbDz = np.array([[self.z_values[1], self.z_values[4], self.z_values[7]], \
##                                  [self.z_values[2], self.z_values[5], self.z_values[8]]])
##            self.Dzt = np.array([[(self.z_values[0])**(-1), 0, 0], \
##                                 [0, (self.z_values[3])**(-1), 0], \
##                                 [0, 0, (self.z_values[6])**(-1)]])
##            self.Ed = np.array([[self.z_values[0]-self.d], \
##                                [self.z_values[3]-self.d], \
##                                [self.z_values[6]-self.dd]])
##            self.U = self.c*self.BbDz.dot(self.Dzt).dot(self.Ed)

# 2 neighbours
##            self.BbDz = np.array([[self.z_values[1], self.z_values[4]], \
##                                  [self.z_values[2], self.z_values[5]]])
##            self.Dzt = np.array([[(self.z_values[0])**(-1), 0], \
##                                 [0, (self.z_values[3])**(-1)]])
##            self.Ed = np.array([[self.z_values[0]-self.d], \
##                                [self.z_values[3]-self.d]])
##            self.U = self.c*self.BbDz.dot(self.Dzt).dot(self.Ed)
