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
        
        # prepare publisher for modified laser scan data
        self.pub_scan = rospy.Publisher('/n_1scan', LaserScan, queue_size=50)

        self.num_readings = 360
        self.laser_frequency = 80
        # subscribe to /scan topic with calculate_z as calback
        rospy.Subscriber('/n_1hokuyo_points', LaserScan, self.calculate_r)

        np.set_printoptions(precision=2)

    def calculate_r(self, msg):
        ''' Calculate the r_values from the scan data '''
        # save measured ranges in local array
        # with correction of distance
        current_time = rospy.Time.now()

        scan = LaserScan()
        scan.header.stamp = current_time
        scan.header.frame_id = 'n_1_laser_frame'
        scan.angle_min = -3.14159011841
        scan.angle_max = 3.14159011841
        scan.angle_increment = 6.283/self.num_readings
        scan.time_increment = (1.0 / self.laser_frequency) /(self.num_readings)
        scan.range_min = 0.3
        scan.range_max = 30.0
        
        
        scan.intensities = []

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
                        if v is not None:
                            z, q = self.robots[i][1], self.robots[i][1] 
                            while True:
                                if scandata2[z] != 0:
                                    z -= 1 
                                if scandata2[q] != 0 and q != 359:
                                    q += 1
                                elif q == 359:
                                    q -= 360
                                    z -= 360
                                if scandata2[z] == 0 and scandata2[q] == 0:
                                    if z < 0:
                                        scandata2[z:] = scandata2[:q] = 0
                                    else:
                                        scandata2[z:q] = 0
                                    break
                        else:
                            print i, v, 'is NONE'
                    
                    
                    scandata3 = scandata2.copy()
                    scandata3 = [float('Inf') if x<=0 else x for x in scandata2]
                    print'scandata2', scandata3
                    scan.ranges = scandata3
                    for idx, val in enumerate(scandata2):
                        if (val < self.min_range) or (val > 13):
                            scandata2[idx] = 0

                    accdata2, i = self.clean_data(scandata2)

                    obstacles = []
                    until = i

                    self.find_begin_center_end(accdata2, obstacles, until, i)

                    # NOTE: position of last obstacles can start with negative angles!
                    if len(obstacles) >= 1:
                        print 'found', len(obstacles), 'obstacle(s)'
                        self.obstacles = []
                        # calculate obstacle positions
                        for r in xrange(len(obstacles)):
                            b, c, e = obstacles[r]
                            # calculate distance
                            s = t = 0
                            for i in xrange(b, e+1):
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
                for robot in xrange(len(self.robots)):
                    if self.robots[robot] is not None:
                        d, a = self.robots[robot]
                        print 'd original:', d
                        # calculate search area
                        # set to distance to robot reflections
                        d = d + 0.10
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
                    else:
                        print 'not all robots are found'
                        self.k = 0
  
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
                # calculate the z_x and _y values for each robot and obstacle.
                # z_x and z_y are the projections of the distance vector on the x and y axis respectively.
                zxy = []
                for idx, val in enumerate(self.robots):
                    zxy.append(self.calculate_x_and_y_dist(self.robots, idx))
                print 'sum x', sum(row[0] for row in zxy)/4
                print 'sum y', sum(row[1] for row in zxy)/4
                if self.obstacles != []:
                    oxy = []
                    obstacle_dist_x_y = np.empty([0, 3*len(self.obstacles)], dtype=np.float32)
                
                    for idx, val in enumerate(self.obstacles):
                        oxy.append(self.calculate_x_and_y_dist(self.obstacles, idx))
                        obstacle_dist_x_y = np.append(obstacle_dist_x_y,
                                            (np.array([self.obstacles[idx][0], oxy[idx][0], oxy[idx][1]], dtype=np.float32)))
                print 'distance to r4:', self.robots[0][0]
                print 'distance to r3:', self.robots[1][0]
                print 'distance to r2:', self.robots[2][0]
                self.r_values = np.array([self.robots[0][0], zxy[0][0], zxy[0][1],
                                          self.robots[1][0], zxy[1][0], zxy[1][1],
                                          self.robots[2][0], zxy[2][0], zxy[2][1]], dtype=np.float32)
                
                if self.obstacles != []:
                    self.obs_values = obstacle_dist_x_y
                    self.pub_obs.publish(self.obs_values)
                else:
                    self.pub_obs.publish(np.array([0.0, 0.0, 0.0], dtype=np.float32))
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
                    self.pub_scan.publish(scan)
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
                until = i
                # we assume a robot when accdata[i] >= 3
                # search for "robots"
                robots = []        
                self.find_begin_center_end(accdata, robots, until, i)

                accdata2, i = self.clean_data(scandata2)
                until = i

                # NOTE: position of last robot can start with negative angles!

                print 'search ended at i=', i
                print 'found', len(robots), 'robots'
                obstacles = []
                self.find_begin_center_end(accdata2, obstacles, until, i)
                print 'these are the obstacles', obstacles
                if len(robots) == 3:
                    self.robots = []
                    # calculate robot positions
                    self.determine_positions(robots, scandata)

                    # put robots in proper order, only for agent_1 and agent_2.
                    l = 0
                    if ((robots[0][1] < 40) & (self.name == 'nexus1') & (self.name == 'nexus2')):
                        print 'robots before reordering', robots
                        # should be last
                        l = 1
                    for _ in xrange(3): 
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
            positions.append((begin, center, end))
    
    def determine_positions(self, data, sdata):
        for r in xrange(len(data)):
            b, c, e = data[r]
            # calculate distance
            s = t = 0
            for i in xrange(b, e+1):
                if sdata[i] != 0:
                    s += sdata[i]
                    t += 1
            d = (s / t) + 0.1
            data[r] = (d, c)
    
    def update_angle_and_distance(self, robotdata, a, amax, d, dmax):
        # remove irrelevant bits
        for i in xrange(len(robotdata)):
            if (robotdata[i] < d - dmax) or (robotdata[i] > d + dmax):
                robotdata[i] = 0

        # calculate new angle and distance
        aas = np.where(robotdata != 0)[0]
        if aas.any(): 
            anew = int(np.round(aas.mean()))
            a = a - amax + anew
            s = t = 0
            print robotdata
            for i in xrange(len(robotdata)):
                if robotdata[i] != 0:
                    s += robotdata[i]
                    t += 1
            d = (s / t) + 0.09
            print 'd!', d

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
        for i in xrange(360):
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
