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
        # self.d = np.float32(0.8*1.025)
        self.d = np.float32(0.8)        
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
        self.E4_log = np.array([])
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
        self.pub = rospy.Publisher('/n_1/cmd_vel', Twist, queue_size=1)
        self.velocity = Twist()
                
        # subscribe to z_values topic
        rospy.Subscriber('/n_1/z_values', numpy_msg(Floats), self.controller)
        rospy.Subscriber('/n_1/agents', Int32, self.n)
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

            # print "error = ", Ed
            # print "z_values = ", z_values
            
            # Formation motion control
            #Ab = np.array([[self.mu[0], 0, self.mu[3], 0], \
                                #[0, self.mu[0], 0, self.mu[3]]])
            #z = np.array([z_values[7], z_values[8], z_values[1], z_values[2]])
            # z = [ edge 1 , edge 4]
            
            # Control law
            U = self.c*BbDz.dot(Dzt).dot(Ed) #+ (Ab.dot(z)).reshape((2, 1))
            print "U = ", -U
        
            # Saturation
            v_max = 0.2
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
        
        # print 'cmd_vel NEXUS 1 (x,y)', self.velocity.linear.x, self.velocity.linear.y
        #        rospy.loginfo(self.velocity)

        self.pub.publish(self.velocity)

    def shutdown(self):
        ''' Stop the robot when shutting down the controller_1 node '''
        rospy.loginfo("Stopping Nexus_1...")
        self.running = np.float32(10000)
        self.velocity = Twist()
        self.pub.publish(self.velocity)

#        np.save('/home/s2036975/Documents/Master Thesis/experiments/experiment_x/E1_log_nx1', self.E1_log)
#        np.save('/home/s2036975/Documents/Master Thesis/experiments/experiment_x/E4_log_nx1', self.E4_log)
#        np.save('/home/s2036975/Documents/Master Thesis/experiments/experiment_x/U_log_nx1', self.U_log)
#        np.save('/home/s2036975/Documents/Master Thesis/experiments/experiment_x/time_log_nx1', self.time_log)

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



if __name__ == '__main__':
    try:
        rospy.init_node('controller_1', anonymous=False)
        controller()
        rospy.spin()
    except:
        rospy.loginfo("Controller node_1 terminated.")  


