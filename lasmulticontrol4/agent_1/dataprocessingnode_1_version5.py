#!/usr/bin/env python

from __future__ import division

from math import *
import math
import rospy
import numpy as np
from rospy_tutorials.msg import Floats				
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan


class determine_r_values:
    ''' Determines (1) the inter-agent distance values, and publishes them to the r_values topic.
                    (2) the and distance to each obstacles, and publishes them to the obs_values topic.

    In the init state (k=0) the robots are assumed to be in their approximate initial position.
    In addition, the obstacles are assumed to be farther than 1.5 length units away during the init state (k=0)
    After recognizing, they are assigned their proper robot number and tracking is started.

    The proper number derived from their topology, see the picture in the thesis of Johan Siemonsma.
    See the code at the and if the initial state code.

    In the tracking state the r_values and obs_values are published.
    
    Laser scanner angle zero is in the backward direction w.r.t. the robot as used here.
    We use angles from 0 to 360, so an object right in front of the robot would return at value 180.'''
   
    def __init__(self):
        ''' Initiate self and subscribe to /n_1hokuyo_points topic '''

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
        self.pub_r = rospy.Publisher(self.name + '/r_values', numpy_msg(Floats), queue_size=1)
        self.pub_obs = rospy.Publisher(self.name + '/obstacles', numpy_msg(Floats), queue_size=1)

        # subscribe to /scan topic with calculate_z as calback
        rospy.Subscriber('/n_1hokuyo_points', LaserScan, self.calculate_r)

        np.set_printoptions(precision=2)

    def calculate_r(self, msg):
        ''' Calculate the r_values from the scan data '''
        # save measured ranges in local array
        # with correction of distance
        scandata= np.asarray(msg.ranges) #* 0.97
        scandata2 = np.copy(scandata)

        # Check if not shutdown
        if self.running:

            if self.k == 2: # tracking state

                # for every robot

                def obstacle_search():
                    # remove irrelevant data                    
                    for i, v in enumerate(scandata2):
                        if v > 13:
                            scandata2[i] = 0  
                    # set all robotdata to zero                                
                    for i, v in enumerate(self.robots):
                        z, q = self.robots[i][1], self.robots[i][1]+1 
                        while True:
                            if scandata2[z] != 0:
                                z -= 1 
                            if scandata2[q] != 0:
                                q += 1
                            if scandata2[z] == 0 and scandata2[q] == 0:
                                if z < 0:
                                    scandata2[z:] = scandata2[:q] = 0
                                else:
                                    scandata2[z:q] = 0
                                break
                    for idx, val in enumerate(scandata2):
                        if (val < self.min_range) or (val > 13):
                            scandata2[idx] = 0

                    accdata2, i = self.clean_data(scandata2)

                    # for i, v in enumerate(scandata2):
                    #     if ((v == 0) and  (scandata2[i-1] == 0) and
                    #         (scandata2[i-2] == 0) and (scandata2[i-3] == 0)):
                    #             break
                    # accdata2 = self.clean_data(scandata2)
                    print 'accdata2', accdata2
                    obstacles = []
                    until = i
                    print 'i', i
                    self.find_begin_center_end(accdata2, obstacles, until, i)

                    # NOTE: position of last obstacles can start with negative angles!
                    if len(obstacles) >= 1:
                        print 'found', len(obstacles), 'obstacle(s)'
                        self.obstacles = []
                        # calculate obstacle positions
                        for r in range(len(obstacles)):
                            b, c, e = obstacles[r]
                            # calculate distance
                            s = t = 0
                            for i in range(b, e+1):
                                if scandata2[i] != 0:
                                    s += scandata2[i]
                                    t += 1
                            d = (s / t) + 0.03
                            obstacles[r] = (d, c)
                        for r in obstacles:
                            self.obstacles.append(r)
                    else:
                        print 'no obstacles found!'
                        self.obstacles = []

                obstacle_search()
                # for every robot, update its distance and angle based on the current lidar information
                for robot in range(len(self.robots)):
                    d, a = self.robots[robot]
                    print 'd, a', d, a
                    # calculate search area
                    # set to distance to robot reflections
                    d = d - 0.09
                    #   d +/-:   halve diameter emmer + mogelijke verplaatsing + marge om rand te zien:
                    dmax = 0.1 + 0.1 + 0.05
                    #   a +/-:
                    amax = int(degrees(atan(0.25 / d)))

                    # get array section
                    robotdata = np.empty(2*amax)
                    if a - amax < 0:
                        robotdata[0:-(a-amax)] = scandata[a-amax:]
                        robotdata[-(a-amax):2*amax] = scandata[0:a+amax]
                    elif a + amax > 360:                        
                        robotdata[0:-(a+amax-360)] = scandata[a-amax:]
                        robotdata[-(a+amax-360):2*amax] = scandata[0:a+amax-360]
                    else:
                        robotdata = scandata[a-amax:a+amax]

                    self.robots[robot] = self.update_angle_and_distance(robotdata, a, amax, d, dmax)

                # for every obstacle   
                for obstacle in self.obstacles:
                    distance, angle = obstacle
                    # calculate search area
                    # set to distance to robot reflections
                    distance = distance - 0.09
                    #   d +/-:   halve diameter emmer + mogelijke verplaatsing + marge om rand te zien:
                    d2max = 0.1 + 0.1 + 0.05
                    #   a +/-:
                    a2max = int(degrees(atan(0.25 / distance)))
                    # get array section
                    robotdata2 = scandata2[angle-a2max:angle+a2max]
                    obstacle = self.update_angle_and_distance(robotdata2, a, a2max, d, d2max)

                # all robots still found?
                ok = True

                # create message to publish
                # calculate the z_x and _y values for each robot and obstacle
                zxy = []
                for idx, val in enumerate(self.robots):
                    zxy.append(self.calculate_x_and_y_dist(self.robots, idx))

                print 'len', self.obstacles
                if self.obstacles != []:
                    oxy = []
                    print 'len', len(self.obstacles)
                    obstacle_dist_x_y = np.empty([0, 3*len(self.obstacles)], dtype=np.float32)
                    print 'dtype obstacle_dist_x_y', obstacle_dist_x_y.dtype
                    for idx, val in enumerate(self.obstacles):
                        oxy.append(self.calculate_x_and_y_dist(self.obstacles, idx))
                        print oxy[idx]
                        obstacle_dist_x_y = np.append(obstacle_dist_x_y,
                                            (np.array([self.obstacles[idx][0], oxy[idx][0], oxy[idx][1]], dtype=np.float32)))

                self.r_values = np.array([self.robots[0][0], zxy[0][0], zxy[0][1],
                                          self.robots[1][0], zxy[1][0], zxy[1][1],
                                          self.robots[2][0], zxy[2][0], zxy[2][1]], dtype=np.float32)
                
                if self.obstacles != []:
                    self.obs_values = obstacle_dist_x_y
                    self.pub_obs.publish(self.obs_values)
                print self.r_values
                print self.name, '(volgorde = 4, 3, 2)'
                print '------------------------------------------------'
                print 'dist to Nexus 4 = ', round(self.r_values[0], 2), self.robots[0][1]
                print 'dist to Nexus 3 = ', round(self.r_values[3], 2), self.robots[1][1]
                print 'dist to Nexus 2 = ', round(self.r_values[6], 2), self.robots[2][1]
                print '------------------------------------------------'

                if len(self.obstacles) > 0:
                    for index, obstacle in enumerate(self.obstacles):
                        print 'dist to Obstacle', index + 1, ':', obstacle
                print '------------------------------------------------'        

                if ok:
                    self.pub_r.publish(self.r_values)
                
                else:
                    print 'Not all robots found!'
                    self.max_range = 1.5
                    self.k = 0

            elif self.k == 0:   # initial state
                # remove irrelevant bits for robot recognition
                for idx, val in enumerate(scandata):
                    if (val < self.min_range) or (val > self.max_range):
                        scandata[idx] = 0

                # remove irrelevant bits for obstacle recognition
                scandata2 = scandata2 - scandata
                
                for idx, val in enumerate(scandata2):
                    if (val < self.min_range) or (val > 13):
                        scandata2[idx] = 0
                
                accdata, i = self.clean_data(scandata)
                accdata2, i = self.clean_data(scandata2)
                
                # for i, v in enumerate(scandata):
                #     if ((v == 0) and  (scandata[i-1] == 0) and
                #         (scandata[i-2] == 0) and (scandata[i-3] == 0)):
                #             break
                until = i
                # we assume a robot when accdata[i] >= 3
                # search for "robots"
                robots = []        
                self.find_begin_center_end(accdata, robots, until, i)
    
                for i, v in enumerate(scandata2):
                    if ((v == 0) and  (scandata2[i-1] == 0) and
                        (scandata2[i-2] == 0) and (scandata2[i-3] == 0)):
                            break  
                until = i
                print 'until', i
                

                # NOTE: position of last robot can start with negative angles!

                print 'search ended at i=', i
                print 'found', len(robots), 'robots'
                obstacles = []
                self.find_begin_center_end(accdata2, obstacles, until, i)
                print 'these are the obstacles', obstacles
                if len(robots) == 3:
                    # calculate robot positions
                    self.determine_positions(robots, scandata)

                    # put robots in proper order, only for agent_1 and agent_2.
                    l = 0
                    if ((robots[0][1] < 40) & (self.name == 'nexus1') & (self.name == 'nexus2')):
                        print 'robots before reordering', robots
                        # should be last
                        l = 1
                    for _ in range(3): 
                        self.robots.append(robots[l])
                        l = (l+1) % 3
                    print 'distance and angle', self.robots
                    
                    if len(obstacles) >= 1:
                        print 'found the obstacle'
                        #calculate obstacle positions
                        self.determine_positions(obstacles, scandata2)
                        for r in obstacles:
                            self.obstacles.append(r)
                    else:
                        print 'no obstacles found! We will keep searching during the next state'

                    # robots and obstacles are lists of (distance, angle)
                    # switch to tracking
                    self.max_range = 1.5
                    self.k = 2
                else:
                    print 'no 3 robots found!'

            # if tracking lost, start at init again

    def find_begin_center_end(self, _accdata, positions, until, i):
        while (i < 360):
            while (_accdata[i]) < 3:
                i = (i+1) % 360
                if i == until:
                    break
            if i == until:
                print 'Init: done for starts on', self.name
                break
            # found start of next robot
            begin = i
            while (_accdata[i] >= 3):
                i = (i+1) % 360
            if i == until:
                print 'Init: done for ends on', self.name
                break
            # found end of next robot
            end = i
            center = (begin + end) // 2
            if end < begin:
                # can only happen with last robot
                if end < center:
                    center -= 180
                    begin -= 360
            positions.append((begin,center,end))
    
    def determine_positions(self, data, sdata):
        for r in range(len(data)):
                        b, c, e = data[r]
                        # calculate distance
                        s = t = 0
                        for i in range(b, e+1):
                            if sdata[i] != 0:
                                s += sdata[i]
                                t += 1
                        d = (s / t) + 0.03
                        data[r] = (d, c)
    
    def update_angle_and_distance(self, robotdata, a, amax, d, dmax):
        # remove irrelevant bits
        for i in range(len(robotdata)):
            if (robotdata[i] < d - dmax) or (robotdata[i] > d + dmax):
                robotdata[i] = 0

        # calculate new angle and distance
        aas = np.where(robotdata != 0)[0]
        if aas.any(): 
            anew = int(np.round(aas.mean()))
            a = a - amax + anew
            s = t = 0
            for i in range(2*amax):
                if robotdata[i] != 0:
                    s += robotdata[i]
                    t += 1
            d = (s / t) #- 0.01

            # keep angles "in range"
            if a < -amax:
                a += 360
            if a >= 360:
                a -= 360
            return d, a

    def calculate_x_and_y_dist(self, data, it):
        zxy = []
        zxy.append(np.float32(np.cos((data[it][1] - np.int_(180))*2*np.pi/360)*data[it][0]))
        zxy.append(np.float32(np.sin((data[it][1] - np.int_(180))*2*np.pi/360)*data[it][0]))
        return zxy

    def clean_data(self, data):
        # where is a relevant reflection?
        wheredata = np.empty(360)
        for i in range(360):
            if data[i] != 0:
                wheredata[i] = 1
            else:
                wheredata[i] = 0
        # accumulate 6 entries in new array
        accdata = np.empty(360)
        for i, v in enumerate(wheredata):
            accdata[i-3] = (wheredata[i-6] + wheredata[i-5] + wheredata[i-4] +
                            wheredata[i-3] + wheredata[i-2] + wheredata[i-1])
        # find a gap (assume there is at least one) to start the search from
        for i, v in enumerate(data):
            if ((v == 0) and  (data[i-1] == 0) and
                (data[i-2] == 0) and (data[i-3] == 0)):
                    break
        return accdata, i

    def shutdown(self):
        rospy.loginfo("Stopping dataprocessingnode_1...")
        self.running = False
        r_values = np.array([self.d, self.d, 0,
                             self.dd, self.dd, 0,
                             self.d, self.d, 0], dtype=np.float32)
        self.pub_r.publish(r_values)
        rospy.sleep(1)

if __name__ == '__main__':
    rospy.init_node('dataprocessingnode_1', anonymous=False)
    determine_r_values()
    rospy.spin() 

# Change for sim:
#        rospy.Subscriber('/nexus2/scan', numpy_msg(Floats), self.calculate_z)
#    def calculate_z(self, data):
#        self.ranges= data.data
