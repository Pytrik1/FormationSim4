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
        #print scandata
        # use test data
        #scandata = np.roll(self.tdata, 1)
        #self.tdata = np.copy(scandata)
        

        # Check if not shutdown
        if self.running:
            
            if self.k == 2: # tracking state
                
                # for every robot

                def obstacle_search():
                    # remove irrelevant data                    
                    for i in range(len(scandata2)):
                        if scandata2[i] > 13:
                            scandata2[i] = 0  
                    # set all robotdata to zero                                
                    for i, v in enumerate(self.robots):
                        z = self.robots[i][1]
                        q = self.robots[i][1]+1
                        print 'q', q
                        while True:
                            if scandata2[z] != 0:
                                z -= 1 
                            if scandata2[q] != 0:
                                q += 1
                            if scandata2[z] == 0 and scandata2[q] == 0:
                                scandata2[z:q] = 0
                                break

                    # where is a relevant reflection?
                    wheredata2 = np.empty(360)
                    for i in range(360):
                        if scandata2[i] != 0:
                            wheredata2[i] = 1
                        else:
                            wheredata2[i] = 0

                    # accumulate 6 entries in new array
                    accdata2 = np.empty(360)
                    for i, v in enumerate(wheredata2):
                        accdata2[i-3] = (wheredata2[i-6] + wheredata2[i-5] + wheredata2[i-4] +
                                        wheredata2[i-3] + wheredata2[i-2] + wheredata2[i-1])
                    
                    # find a gap (assume there is at least one) to start the search from
                    for i, v in enumerate(scandata2):
                        if ((scandata2[i] == 0) and  (scandata2[i-1] == 0) and
                            (scandata2[i-2] == 0) and  (scandata2[i-3] == 0)):
                            break
                    
                    obstacles = []
                    until = i
                    while (i < 360):
                        while (accdata2[i]) < 3:
                            i = (i + 1) % 360
                            if i == until:
                                break
                        if i == until:
                            print 'Init: done for starts on', self.name
                            break
                        # found start of next robot
                        begin = i
                        while (accdata2[i] >= 3):
                            i = (i + 1) % 360
                        if i == until:
                            print 'Init: done for ends on', self.name
                            break
                        # found end of next robot
                        
                        end = i
                        # if end == 0:
                        #     end  -=1
                        print 'end', end
                        print 'begin', begin
                        center = (begin + end) // 2
                        if end < begin:
                            # can only happen with last robot
                            if end < center:
                                center -= 180
                            begin -= 360
                        
                        obstacles.append((begin, center, end))
                        print 'obstacles', begin, center, end

                    # NOTE: position of last obstacles can start with negative angles!
                    if len(obstacles) == 1:
                        print 'found the obstacle'
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
                        self.obstacles = [obstacles[0]]
                    else:
                        print 'no obstacles found!'
                        self.obstacles = []

                    print scandata2
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
                    amax = int(degrees(atan(0.25/d)))

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

                    # remove irrelevant bits
                    for i in range(len(robotdata)):
                        if (robotdata[i] < d - dmax) or (robotdata[i] > d + dmax):
                            robotdata[i] = 0

                    # calculate new angle and distance
                    aas = np.where(robotdata != 0)[0] 
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
                        a = a + 360
                    if a >= 360:
                        a = a - 360
                    self.robots[robot] = (d,a)

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
                    robotdata2 = np.empty(2*a2max)
                    robotdata2 = scandata2[angle-a2max:angle+a2max]

                    # remove irrelevant bits
                    for i in range(len(robotdata2)):
                        if (robotdata2[i] < distance - d2max) or (robotdata2[i] > distance + d2max):
                            robotdata2[i] = 0

                    # calculate new angle and distance
                    a2as = np.where(robotdata2 != 0)[0] 

                    if a2as.any():
                        a2new = int(np.round(a2as.mean()))
                        angle = angle - a2max + a2new

                        s2 = t2= 0
                        for i in range(2*a2max):
                            if robotdata2[i] != 0:
                                s2 += robotdata2[i]
                                t2 += 1
                        distance = (s2 / t2) #- 0.01

                        # keep angles "in range"
                        if angle < -a2max:
                            angle = angle + 360
                        if angle >= 360:
                            angle = angle - 360
                        obstacle = (distance, angle)

                # all robots still found?
                ok = True

                # create message to publish
                # Calculate the z_x and _y values for each robot and obstacle
                self.zx_1 = np.float32(np.cos((self.robots[0][1]-np.int_(180))*2*np.pi/360)*self.robots[0][0])  # edge 4
                self.zy_1 = np.float32(np.sin((self.robots[0][1]-np.int_(180))*2*np.pi/360)*self.robots[0][0])  # edge 4
                self.zx_2 = np.float32(np.cos((self.robots[1][1]-np.int_(180))*2*np.pi/360)*self.robots[1][0])  # edge 6 (new diag)
                self.zy_2 = np.float32(np.sin((self.robots[1][1]-np.int_(180))*2*np.pi/360)*self.robots[1][0])  # edge 6 (new diag)
                self.zx_3 = np.float32(np.cos((self.robots[2][1]-np.int_(180))*2*np.pi/360)*self.robots[2][0])  # edge 1
                self.zy_3 = np.float32(np.sin((self.robots[2][1]-np.int_(180))*2*np.pi/360)*self.robots[2][0])  # edge 1
                if self.obstacles != []:
                    self.ox_1 = np.float32(np.cos((self.obstacles[0][1]-np.int_(180))*2*np.pi/360)*self.obstacles[0][0])
                    self.oy_1 = np.float32(np.sin((self.obstacles[0][1]-np.int_(180))*2*np.pi/360)*self.obstacles[0][0])
                    print 'ox_1 = ', self.ox_1
                    print 'oy_1 = ', self.oy_1
                    obstacle_dist_x_y = np.array([self.obstacles[0][0], self.ox_1, self.oy_1], dtype=np.float32)
                 
                self.r_values = np.array([self.robots[0][0], self.zx_1, self.zy_1,
                                          self.robots[1][0], self.zx_2, self.zy_2,
                                          self.robots[2][0], self.zx_3, self.zy_3], dtype=np.float32)

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
                    print 'dist to Obstacle 1 =', distance, angle
                    print '------------------------------------------------'

                if ok:
                    self.pub_r.publish(self.r_values)
                
                else:
                    print 'Not all robots found!'
                    self.max_range = 1.5
                    self.k = 0

            elif self.k == 0:   # initial state
                # remove irrelevant bits for robot recognition
                for i in range(len(scandata)):
                    if (scandata[i] < self.min_range) or (scandata[i] > self.max_range):
                        scandata[i] = 0

                # remove irrelevant bits for obstacle recognition
                scandata2 = scandata2 - scandata
                
                for i in range(len(scandata2)):
                    if (scandata2[i] < self.min_range) or (scandata2[i] > 13):
                        scandata2[i] = 0
                
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
                    accdata[i] = (wheredata[i-3] + wheredata[i-2] + wheredata[i-1] +
                                  wheredata[i] + wheredata[i+1] + wheredata[i+2])

                accdata2 = np.empty(360)
                for i, v in enumerate(wheredata2):
                    accdata2[i-3] = (wheredata2[i-6] + wheredata2[i-5] + wheredata2[i-4] +
                                     wheredata2[i-3] + wheredata2[i-2] + wheredata2[i-1])

                # find a gap (assume there is at least one) to start the search from
                for i, v in enumerate(scandata):
                    if ((v == 0) and  (scandata[i-1] == 0) and
                        (scandata[i-2] == 0) and (scandata[i-3] == 0)):
                            break

                for i, v in enumerate(scandata2):
                    if ((v == 0) and  (scandata2[i-1] == 0) and
                        (scandata2[i-2] == 0) and (scandata2[i-3] == 0)):
                            break

                # we assume a robot when accdata[i] >= 3
                # search for "robots"
                robots = []
                obstacles = []
                until = i
                print 'until', i
                self.find_positions(accdata, robots, until, i)
                # while (i < 360):
                #     while (accdata[i]) < 3:
                #         i = (i+1) % 360
                #         if i == until:
                #             break
                #     if i == until:
                #         print 'Init: done for starts on', self.name
                #         break
                #     # found start of next robot
                #     begin = i
                #     while (accdata[i] >= 3):
                #         i = (i+1) % 360
                #     if i == until:
                #         print 'Init: done for ends on', self.name
                #         break
                #     # found end of next robot
                #     end = i
                #     if end < begin:
                #         # can only happen with last robot
                #         if end < center:
                #             center -= 360
                #         begin -= 360
                #     center = (begin + end) // 2
                #     robots.append((begin,center,end))
                #     print 'robots',begin, center, end

                # NOTE: position of last robot can start with negative angles!

                print 'search ended at i=', i
                print 'found', len(robots), 'robots'

                while (i < 360):
                    while (accdata2[i]) < 3:
                        i = (i + 1) % 360
                        if i == until:
                            break
                    if i == until:
                        print 'Init: done for starts on', self.name
                        break
                    # found start of next obstacle
                    begin = i
                    while (accdata2[i] >= 3):
                        i = (i + 1) % 360
                    if i == until:
                        print 'Init: done for ends on', self.name
                        break
                    # found end of next obstacle
                    end = i
                    if end < begin:
                        # can only happen with last obstacle
                        if end < center:
                            center -= 360
                        begin -= 360
                    center = (begin + end) // 2
                    obstacles.append((begin, center, end))
                    print 'obstacles', begin, center, end

                if len(robots) == 3:
                    # calculate robot positions
                    for r in range(len(robots)):
                        b, c, e = robots[r]
                        # calculate distance
                        s = t = 0
                        for i in range(b, e+1):
                            if scandata[i] != 0:
                                s += scandata[i]
                                t += 1
                        d = (s / t) + 0.03
                        robots[r] = (d, c)
                    print 'lookey herey!!!!!!!', robots
                    # put robots in proper order, only for agent_1 and agent_2.
                    l = 0
                    if ((robots[0][1] < 40) & (self.name == 'nexus1') & (self.name == 'nexus2')):
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
                        for obstacle in obstacles:
                            b, c, e = obstacle
                            # calculate distance
                            s = t = 0
                            for i in range(b, e+1):
                                if scandata2[i] != 0:
                                    s += scandata2[i]
                                    t += 1
                            d = (s / t) + 0.03
                            obstacle = (d, c)
                        self.obstacles.append(obstacles[0])
                    else:
                        print 'no obstacles found! We will keep searching during the next state'

                    # robots and obstacles are lists of (distance, angle)
                    # switch to tracking
                    self.max_range = 1.5
                    self.k = 2
                else:
                    print 'no 3 robots found!'

            # if tracking lost, start at init again

    def find_positions(self, _accdata, positions,until, i):
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
            if end < begin:
                # can only happen with last robot
                if end < center:
                    center -= 360
                begin -= 360
            center = (begin + end) // 2
            positions.append((begin,center,end))
            print 'positions',begin, center, end

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
