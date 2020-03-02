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
    After recognizing, they are assigned there proper robot number and tracking is started.

    The proper number derived from there topology, see the picture in the thesis of Johan Siemonsma.
    See the code at the and if the initial state code.

    In the tracking state the z_values are published.
    
    Laser scanner angle zero is in the backward direction wrt the robot as used here.
    We use angles (0:360) as follows:


    '''
   
    def __init__(self):
        ''' Initiate self and subscribe to /scan topic '''

        # which nexus?
        self.name = 'n_1'
        # Desired distance - used for sending if no z is found or if the dataprocessingnode is shutdown: 
        # robot not influenced if one z not found
        self.d = np.float32(0.8)
        self.dd = np.float32(np.sqrt(np.square(self.d)+np.square(self.d)))
        # NOTE: not used anymore
        
        # set min and max values for filtering ranges. Will change during tracking
        self.min_range = 0.35
        self.max_range = 1.5
        
        # state of processing
        # 0: init, 1: tracking, 2: 
        self.k = 0

        # the robots
        self.robots = []

        # the obstacles
        self.obstacles = []

        # prepare shutdown
        self.running = True
        rospy.on_shutdown(self.shutdown)
                        
        # prepare publisher
        self.pub = rospy.Publisher(self.name + '/z_values', numpy_msg(Floats), queue_size=1)
        
        # subscribe to /scan topic with calculate_z as calback
        rospy.Subscriber('/n_1hokuyo_points', LaserScan, self.calculate_z)
            	
        np.set_printoptions(precision=2)
        
    def calculate_z(self, msg):
        ''' Calculate the z_values from the scan data '''
        # save measured ranges in local array
        # with correction of distance
        scandata= np.asarray(msg.ranges) #* 0.97
        scandata2 = np.copy(scandata)
        #print scandata
        # use test data
        #scandata = np.roll(self.tdata, 1)
        #self.tdata = np.copy(scandata)
        

        # Check if not shutdown
        if self.running:
            
            if self.k == 2: # tracking state
                #searching_obstacles()
                # for every robot
                print self.robots
                print self.obstacles
                def searching_obstacles(data):
                    print data
                searching_obstacles(scandata2)
                for r in range(len(self.robots)):
                    d, a = self.robots[r]
                    print 'd original:', d
                    # print self.name, r, ': ', d, a
                    # calculate search area
                    # set to distance to robot reflections
                    d = d - 0.09
                    #   d +/-:   halve diameter emmer + mogelijke verplaatsing + marge om rand te zien:
                    dmax = 0.1 + 0.1 + 0.05
                    #   a +/-:
                    amax = int(degrees(atan(0.25/d)))
		    #print amax
                    # get array section
                    #print 'a , amax, dmax',  a , amax, dmax
                    robotdata = np.empty(2*amax)
                    if a-amax < 0:
                        #print 'aparte tak'
                        robotdata[0:-(a-amax)] = scandata[a-amax:]
                        robotdata[-(a-amax):2*amax] = scandata[0:a+amax]
                    elif a+amax > 360:
                        #print 'ind'
                        robotdata[0:-(a+amax-360)] = scandata[a-amax:]
                        robotdata[-(a+amax-360):2*amax] = scandata[0:a+amax-360]
                    else:
                        robotdata = scandata[a-amax:a+amax]
                    #print   robotdata 
                    #print 'scand2', robotdata.size, a-amax, a+amax

                    # remove irrelevant bits
                    #print 'robotdata raw', robotdata
                    for i in range(len(robotdata)):
                        if (robotdata[i] < d - dmax) or (robotdata[i] > d + dmax):
                            robotdata[i] = 0

                    # calculate new angle and distance
                    aas = np.where(robotdata != 0)[0] 
                    anew = int(np.round(aas.mean()))
                    a = a - amax + anew

                    s=0; t=0
                    for i in range(2*amax):
                        if robotdata[i] != 0:
                            s += robotdata[i]; t += 1
                    d = (s / t) #- 0.01
                    print 'd:', d
                    # keep angles "in range"
                    if a < -amax:
                        a = a + 360
                    if a >= 360:
                        a = a - 360
                    self.robots[r] = (d, a)
                    # print 'r= ', self.robots[r]
                
                # for every obstacle
                if len(self.obstacles) != 0:
                    for r in range(len(self.obstacles)):
                        distance, angle = self.obstacles[r]
                        # print self.name, r, ': ', d, a
                        # calculate search area
                        # set to distance to robot reflections
                        distance = distance - 0.09
                        #   d +/-:   halve diameter emmer + mogelijke verplaatsing + marge om rand te zien:
                        d2max = 0.1 + 0.1 + 0.05
                        #   a +/-:
                        a2max = int(degrees(atan(0.25/distance)))
                #print amax
                        # get array section
                        #print 'a , amax, dmax',  a , amax, dmax
                        robotdata2 = np.empty(2*a2max)
                        if angle-a2max < 0:
                            #print 'aparte tak'
                            robotdata2[0:-(angle-a2max)] = scandata2[angle-a2max:]
                            robotdata2[-(angle-a2max):2*a2max] = scandata2[0:angle+a2max]
                        elif angle+a2max > 360:
                            #print 'ind'
                            robotdata2[0:-(angle+a2max-360)] = scandata2[angle-a2max:]
                            robotdata2[-(angle+a2max-360):2*a2max] = scandata2[0:angle+a2max-360]
                        else:
                            robotdata2 = scandata2[angle-a2max:angle+a2max]
                        #print   robotdata 
                        #print 'scand2', robotdata.size, a-amax, a+amax

                        # remove irrelevant bits
                        #print 'robotdata raw', robotdata
                        for i in range(len(robotdata2)):
                            if (robotdata2[i] < distance - d2max) or (robotdata2[i] > distance + d2max):
                                robotdata2[i] = 0
                        #print 'robot', r, ':', robotdata


                        # calculate new angle and distance
                        a2as = np.where(robotdata2 != 0)[0] 
                        a2new = int(np.round(a2as.mean()))
                        angle = angle - a2max + a2new

                        s2=0; t2=0
                        for i in range(2*a2max):
                            if robotdata2[i] != 0:
                                s2 += robotdata2[i]; t2 += 1
                        distance = (s2 / t2) #- 0.01

                        # keep angles "in range"
                        if angle < -a2max:
                            angle = angle + 360
                        if angle >= 360:
                            angle = angle - 360
                        self.obstacles[r] = (distance, angle)
                # all robots still found?
                ok = True
                
                
                    

                # create message to publish
                # Calculate the z_x and _y values for each robot
                self.zx_1 = np.float32(np.cos((self.robots[0][1]-np.int_(180))*2*np.pi/360)*self.robots[0][0]) # edge 4
                self.zy_1 = np.float32(np.sin((self.robots[0][1]-np.int_(180))*2*np.pi/360)*self.robots[0][0]) # edge 4
                self.zx_2 = np.float32(np.cos((self.robots[1][1]-np.int_(180))*2*np.pi/360)*self.robots[1][0]) # edge 6 (new diag)
                self.zy_2 = np.float32(np.sin((self.robots[1][1]-np.int_(180))*2*np.pi/360)*self.robots[1][0]) # edge 6 (new diag)
                self.zx_3 = np.float32(np.cos((self.robots[2][1]-np.int_(180))*2*np.pi/360)*self.robots[2][0]) # edge 1
                self.zy_3 = np.float32(np.sin((self.robots[2][1]-np.int_(180))*2*np.pi/360)*self.robots[2][0]) # edge 1

                #print 'zx_1 = ', self.zx_1
                #print 'zy_1 = ', self.zy_1
                #print '--'
                #print 'zx_2 = ', self.zx_2
                #print 'zy_2 = ', self.zy_2
                #print '--'
                #print 'zx_3 = ', self.zx_3
                #print 'zy_3 = ', self.zy_3
                #print '--------------------'

    
                self.z_values = np.array([self.robots[0][0], self.zx_1, self.zy_1, \
                                          self.robots[1][0], self.zx_2, self.zy_2, \
                                          self.robots[2][0], self.zx_3, self.zy_3], dtype=np.float32)
                
                self.z_values = np.concatenate((self.z_values,[3.0, 4.0, 5.0]))
                print self.name, '(volgorde = 4, 3, 2)'
                print '------------------------------------------------'
                print 'dist to Nexus 4 = ', round(self.z_values[0],2),self.robots[0][1]
                print 'dist to Nexus 3 = ', round(self.z_values[3],2),self.robots[1][1]
                print 'dist to Nexus 2 = ', round(self.z_values[6],2),self.robots[2][1]
                print 'z_values', self.z_values[9], self.z_values[10], self.z_values[11]
                print '------------------------------------------------'
                if len(self.obstacles) > 0:
                    print 'dist to Obstacle 1 =', distance, angle
                print '------------------------------------------------'
                #print self.z_values
                if ok:
                    self.pub.publish(self.z_values)
                else:
                    print 'Not all robots found!'
                    self.max_range = 1.5
                    self.k = 0

            elif self.k == 0:   # initial state
                #scandata2 = np.copy(scandata)
                # remove irrelevant bits for robot recognition
                for i in range(len(scandata)):
                    if (scandata[i] < self.min_range) or (scandata[i] > self.max_range):
                        scandata[i] = 0
                #print scandata
                
                # remove irrelevant bits for obstacle recognition
                scandata2 = scandata2-scandata
                
                for i in range(len(scandata2)):
                    if (scandata2[i] < self.min_range) or (scandata2[i] > 13):
                        scandata2[i] = 0
                #print scandata2
                # where is a relevant reflection?
                wheredata = np.empty(360)
                for i in range(360):
                    if scandata[i] != 0:
                        wheredata[i] = 1
                    else:
                        wheredata[i] = 0
                
                wheredata2 = np.empty(360)
                for i in range(360):
                    if scandata2[i] != 0:
                        wheredata2[i] = 1
                    else:
                        wheredata2[i] = 0

                # accumulate 6 entries in new array
                accdata = np.empty(360)
                for i in range(-3, len(wheredata)-3):
                    accdata[i] = wheredata[i-3] +wheredata[i-2] +wheredata[i-1] +wheredata[i] +wheredata[i+1] +wheredata[i+2]

                accdata2 = np.empty(360)
                for i in range(-3, len(wheredata2)-3):
                    accdata2[i] = wheredata2[i-3] +wheredata2[i-2] +wheredata2[i-1] +wheredata2[i] +wheredata2[i+1] +wheredata2[i+2]

                # find a gap (assume there is at least one) to start the search from
                for i in range(len(scandata)):
                    if (scandata[i] == 0) and  (scandata[i-1] == 0) and  (scandata[i-2] == 0) and  (scandata[(i-3)] == 0):
                        break
                
                for i in range(len(scandata2)):
                    if (scandata2[i] == 0) and  (scandata2[i-1] == 0) and  (scandata2[i-2] == 0) and  (scandata2[(i-3)] == 0):
                        break

                # we assume a robot when accdata[i] >= 3
                # search for "robots"
                robots = []
                obstacles = []
                until = i
                while (i < 360):
                    while (accdata[i]) < 3:
                        i = (i+1)%360
                        if i == until:
                            break
                    if i == until:
                        print 'Init: done for starts on', self.name
                        break
                    # found start of next robot
                    begin = i
                    while (accdata[i] >= 3):
                        i = (i+1)%360
                    if i == until:
                        print 'Init: done for ends on', self.name
                        break
                    # found end of next robot
                    end = i
                    if end < begin:
                        # can only happen with last robot
                        if end < center:
                            center -= 360
                        begin -= 360
                    center = (begin + end) // 2
                    robots.append((begin,center,end))
                    print 'robots',begin, center, end

                # NOTE: position of last robot can start with negative angles!

                print 'search ended at i=', i
                print 'found', len(robots), 'robots'

                while (i < 360):
                    while (accdata2[i]) < 3:
                        i = (i+1)%360
                        if i == until:
                            break
                    if i == until:
                        print 'Init: done for starts on', self.name
                        break
                    # found start of next robot
                    begin = i
                    while (accdata2[i] >= 3):
                        i = (i+1)%360
                    if i == until:
                        print 'Init: done for ends on', self.name
                        break
                    # found end of next robot
                    end = i
                    if end < begin:
                        # can only happen with last robot
                        if end < center:
                            center -= 360
                        begin -= 360
                    center = (begin + end) // 2
                    obstacles.append((begin,center,end))
                    print 'obstacles', begin, center, end
                
                if len(robots) == 3:
                    # calculate robot positions
                    for r in range(len(robots)):
                        b,c,e = robots[r]
                        # calculate distance
                        s = 0; t = 0
                        for i in range(b,e+1):
                            if scandata[i] != 0:
                                s += scandata[i]; t += 1
                        d = (s / t) + 0.03
                        robots[r] = (d,c)

                    # put robots in proper order, only for agent_1 and agent_2.
                    l = 0
                    if ((robots[0][1] < 40) & (self.name == 'nexus1') & (self.name == 'nexus2') ):
                        print 'robots before reordering', robots
                        # should be last
                        l = 1
                    for i in range(3): 
                        self.robots.append(robots[l])
                        l = (l+1) % 3
                    print 'distance and angle', self.robots
                    
                    if len(obstacles) == 1:
                        print 'found the obstacle'
                        #calculate obstacle positions
                        for r in range(len(obstacles)):
                            b,c,e = obstacles[r]
                            # calculate distance
                            s = 0; t = 0
                            for i in range(b,e+1):
                                if scandata2[i] != 0:
                                    s += scandata2[i]; t += 1
                            d = (s / t) + 0.03
                            obstacles[r] = (d,c)
                        self.obstacles.append(obstacles[0])
                    else:
                        print 'no obstacles found!'
                    
                    # robots and obstacles are lists of (distance, angle)
                    # switch to tracking
                    self.max_range = 2.5
                    self.k = 2
                else:
                    print 'no 3 robots found!'         
                
            # if tracking lost, start at init again

    
    def shutdown(self):
        rospy.loginfo("Stopping dataprocessingnode_1...")
        self.running = False
        z_values = np.array([self.d, self.d, 0, \
                                  self.dd, self.dd, 0, \
                                  self.d, self.d, 0], dtype=np.float32)
        self.pub.publish(z_values)
        rospy.sleep(1)

if __name__ == '__main__':
    rospy.init_node('dataprocessingnode_1', anonymous=False)
    determine_z_values()
    rospy.spin() 



# Change for sim:
###        rospy.Subscriber('/nexus2/scan', numpy_msg(Floats), self.calculate_z)
###    def calculate_z(self, data):
###        self.ranges= data.data
