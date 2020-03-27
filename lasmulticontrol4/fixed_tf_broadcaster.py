#!/usr/bin/env python  
import roslib
# roslib.load_manifest('learning_tf')

import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('fixed_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        #global coordinate frame odom
        # br.sendTransform((0.0, 0.0, 0.0),
        #                  (0.0, 0.0, 0.0, 1.0),
        #                  rospy.Time.now(),
        #                  "n_1/odom",
        #                  "map")
        # br.sendTransform((0.0, 0.0, 0.0),
        #                  (0.0, 0.0, 0.0, 1.0),
        #                  rospy.Time.now(),
        #                  "n_2/odom",
        #                  "map")
        # br.sendTransform((0.0, 0.0, 0.0),
        #                  (0.0, 0.0, 0.0, 1.0),
        #                  rospy.Time.now(),
        #                  "n_3/odom",
        #                  "map")
        # br.sendTransform((0.0, 0.0, 0.0),
        #                  (0.0, 0.0, 0.0, 1.0),
        #                  rospy.Time.now(),
        #                  "n_4/odom",
        #                  "map")
        # br.sendTransform((0.0, 0.0, 0.0),
        #                  (0.0, 0.0, 0.0, 1.0),
        #                  rospy.Time.now(),
        #                  "n_4/odom",
        #                  "odom")
        # carrot transforms
        br.sendTransform((0.4, 0.4, 0.0),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "n_1/carrot1",
                         "n_1hokuyo_frame")
        br.sendTransform((0.4, -0.4, 0.0),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "n_2/carrot2",
                         "n_2hokuyo_frame")
        br.sendTransform((-0.4, -0.4, 0.0),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "n_3/carrot3",
                         "n_3hokuyo_frame")
        br.sendTransform((-0.4, 0.4, 0.0),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "n_4/carrot4",
                         "n_4hokuyo_frame")
        # obstacle laser transforms
        br.sendTransform((0.0, 0.0, 0.0),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "n_1_laser_frame",
                         "n_1hokuyo_frame")
        br.sendTransform((0.0, 0.0, 0.0),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "n_2_laser_frame",
                         "n_2hokuyo_frame")
        br.sendTransform((0.0, 0.0, 0.0),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "n_3_laser_frame",
                         "n_3hokuyo_frame")
        br.sendTransform((0.0, 0.0, 0.0),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "n_4_laser_frame",
                         "n_4hokuyo_frame")
            
    
        rate.sleep()