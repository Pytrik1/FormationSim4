#!/usr/bin/env python

from __future__ import division

from math import *
import rospy
import numpy as np
from rospy_tutorials.msg import Floats				
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan


class determine_z_values:
    ''' Determines the inter-agent distance values and publishes it to z_values topic

    In the init state (k=0) the robots are assumed to be in there approximate initial position.
    After recognizing, they are assigned their proper robot number and tracking is started.

    The proper number derived from there topology, see the picture in the thesis of Johan Siemonsma.
    See the code at the and if the initial state code.

    In the tracking state the z_values are published.
    
    Laser scanner angle zero is in the backward direction wrt the robot as used here.
    We use angles (0:360) as follows:


    '''
   
    def __init__(self):
        ''' Iniate self and subscribe to /scan topic '''
        
        # which nexus?
        self.name = 'n_3'        
        # Desired distance - used for sending if no z is found or if the dataprocessingnode is shutdown: 
        # robot not influenced if one z not found
        self.d = np.float32(0.8)
        self.dd = np.float32(np.sqrt(np.square(self.d)+np.square(self.d)))
        
        # set min and max values for filtering ranges in meter during initiation 
        self.min_range = 0.25
        self.max_range = 3
        
        # set variables for tracking
        self.k = 0


        # prepare shutdown
        self.running = True
        rospy.on_shutdown(self.shutdown)
                        
        # prepare publisher
        self.pub = rospy.Publisher(self.name +'/z_values', numpy_msg(Floats), queue_size=1)
        
        # subscribe to /scan topic with calculate_z as calback
        #rospy.Subscriber('/nexus1/scan', LaserScan, self.calculate_z) #############3
        rospy.Subscriber('/n_3hokuyo_points', LaserScan, self.calculate_z)
        
        np.set_printoptions(precision=2)    	
    #def calculate_z(self, msg):###########3
    def calculate_z(self, msg):
        ''' Calculate the z_values from the scan data '''
        # Check if not shutdown
        
        self.ranges= np.asarray(msg.ranges)  # 3 AND 4 HAVE A PROBLEM WHERE THE ROBOT ON TOP IS DIVIDED BETWEEN THE START AND END OF THE SCAN MATRIX
        #print self.ranges

        if self.running:
            
            # Initially find each other - no disturbances
            if self.k <= 5:
                # Save the angles (hits) of the robots in seperate arrays
                z_a = np.where((self.ranges >= self.min_range) & (self.ranges <= self.max_range))[0] #The zero at the end is to access the first value of the tuple created by "np.where", so z_a is just an array

                n = 1
                
		for i in range(len(z_a)-1):  # Calculates the number of robots

                    if (z_a[i] - z_a[i + 1] >= -10):
                       
                        continue
                   
                    elif (-10 <= z_a[i] - z_a[i-1] <= 10):
                        
                        n = n + 1

		#print(n)
		R=[]
            	r=np.array([])
		Zval=np.array([])

            	P = 0
		
		# Compares difference between angles in z_a to decide if it's a new robot, and if it is it creates and array r, which is then added to the list R
                for i in range(P,len(z_a)-1):
                        
                        
                    if (z_a[i] - z_a[i + 1] >= -10) and i!=(len((z_a))-2):
  
                        r=np.append(r,z_a[i])
   		        #print(r)
		    elif (-10 <= z_a[i] - z_a[i - 1] <= 10):
                        r=np.append(r,z_a[i])
                        R.append(r)
                        P = 1+i
			#print(P)
                        r=np.array([])

		    elif (z_a[i] - z_a[i + 1] >= -10) and i==(len((z_a))-2):
                        r=np.append(r,z_a[i])
                        #print(r)
                        R.append(r)
		#R[:][:]=np.int_(R[:][:])
                #print R
		for i in range(n):  #transform list R to array of integers
                    R[i]=R[i].astype(int)
                
                print(R[3])
                
                self.z_aX=np.zeros([len(R)])
                self.zn_X=np.zeros([len(R)])
		self.z_a_min=np.zeros([len(R)])
		self.z_a_max=np.zeros([len(R)])
		self.zn_min=np.zeros([len(R)])
		self.zn_max=np.zeros([len(R)])

                for j in range(n):

                    if R[j] != np.array([]):

                        self.z_aX[j] = int(np.round((R[j]).mean()))  #CHECK WHAT THIS IS OUTPUTTING
  			
                        self.zn_X[j] = np.float32(np.min(self.ranges[(R[j][0:(len(R[j]))])]))  # + 0.098;
                        
                        # Tracking variables
    
                        self.z_a_min[j] = np.min(np.int_(R[j][0:len(R[j])]))

                        self.z_a_max[j] = np.max(np.int_(R[j][0:len(R[j])]))
 
                        self.zn_min[j] = np.min(np.float32(self.ranges[np.int_(R[j][0:len(R[j])])]))

                        self.zn_max[j] = np.max(np.float32(self.ranges[np.int_(R[j][0:len(R[j])])]))

                        #fprintf('\nNexus: zn_1=%f', self.zn_(j))

                    else:
 
                        self.z_aX[j] = 0

                        self.zn_X[j] = self.d
 
                        print('Nexus = not found')
            
                self.z_aX=self.z_aX.astype(int)  #transform z_aX to an array of integers
                print('dist=',self.zn_X)
                print('ang=',self.z_aX)
                Zval=[]
                self.zx=np.zeros([len(R)])
                self.zy=np.zeros([len(R)])
                self.z_values=[] #np.zeros([3*len(R)])

                for i in range(0,n): #FOR SOME REASON i thought this should go to n-1)

                    self.zx[i] = np.float32(np.cos((self.z_aX[i]-np.int_(180))*2*np.pi/360)*self.zn_X[i])
   
                    self.zy[i] = np.float32(np.sin((self.z_aX[i]-np.int_(180))*2*np.pi/360)*self.zn_X[i])
   
                    Zval.append([self.zn_X[i], self.zx[i], self.zy[i]])

                for i in range(0,n):  #Loop to print the x and y distances of detected agents
                   
                    print 'zx_[',i,'] = ', self.zx[i]
                    print 'zy_[',i,'] = ', self.zy[i]
                    print '--'
               
                print '--------------------'
           

                #print(Zval)
                #self.z_values=[]
                for i in range(0,n): #This loop puts all the values into the z_values matrix

                    self.z_values= np.concatenate(Zval[:][:])

                self.z_values=np.asarray(self.z_values,dtype=np.float32)  #this line ensures that all numbers are type np_float (without it the publisher doesn't publish the proper values)
                print(self.z_values)
                #self.z_values = np.array([self.zn_X[0], self.zx[0], self.zy[0], \
                #                          self.zn_X[1], self.zx[1], self.zy[1], \
                #                          self.zn_X[2], self.zx[2], self.zy[2]], dtype=np.float32)


                # Change max self.da and self.dzn if the robots are in formation or out of formation
#e.g.           if self.zn_1 <= self.d+0.1 and self.zn_2 <= self.d+0.1 and self.zn_3 <= self.d+0.1:
###                self.da = self.da - 2

                # publish
            self.pub.publish(self.z_values)

    
    def shutdown(self):
        # Setting z = d in order to stop the robots when shutting down the dataprocessingnode_2 node
        rospy.loginfo("Stopping dataprocessingnode_3...")
        self.running = False
        self.z_values = np.array([self.d, self.d, 0, \
                                  self.dd, self.dd, 0, \
                                  self.d, self.d, 0], dtype=np.float32)
        self.pub.publish(self.z_values)
        print ('SHUtting Down')
	rospy.sleep(1)

if __name__ == '__main__':
    rospy.init_node('dataprocessingnode_3', anonymous=False)
    determine_z_values()
    rospy.spin() 


# Change for sim:
###        rospy.Subscriber('/nexus1/scan', numpy_msg(Floats), self.calculate_z)
###    def calculate_z(self, data):
###        self.ranges= data.data
